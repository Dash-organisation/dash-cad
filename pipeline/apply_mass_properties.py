#!/usr/bin/env python3
"""apply_mass_properties.py: replace the CAD inertials of a closed MJCF by the Fusion mass properties.

ACDC4Robot writes each link's mass and inertia from the CAD material densities. Those densities are
wrong (printed parts are not solid, motors are not modelled with their real mass), so the exported
robot weighs about 5.3 kg against about 15 kg for the real one. This script rewrites every body's
<inertial> from two reviewed inputs:

  properties/mass_properties.json   per-part values extracted from Fusion's "Properties" PDF
                                    (mass g, COM mm, inertia at COM g mm^2, inertia at origin)
  properties/body_mapping_*.json    which PDF part goes on which MJCF body, and how

Usage:
    python pipeline/apply_mass_properties.py <closed.xml> --mapping properties/body_mapping_xshape.json
           [--properties properties/mass_properties.json] [-o out.xml] [--report]

    Default output: <name>_mass.xml next to the input, plus <name>_mass_audit.json.
    --report validates and prints, writes nothing. The input file is never modified.

Mapping methods (per body, key "method"):
    "pdf"        (default) PDF mass, COM and inertia, taken in the export body frame:
                 p_body = Rx(rotation_x_deg) @ p_pdf + translation_mm. The tensor is at the COM, so
                 it is only rotated (no parallel-axis shift for the origin translation).
    "scale_cad"  the part changed in CAD after the PDF was made. Keep the new CAD COM and tensor shape,
                 scale mass and tensor by the density factor of the old version of the part:
                 factor = PDF mass / reference_cad_mass_kg (old CAD mass of that PDF part).
                 Exact for a pure density error, an estimate otherwise. Re-measure when you can.

"extra_body_mass" adds a mass to one body with the same COM and normalised distribution
(tensor scaled by (m + extra) / m): an allowance for cables, screws, electronics not in the PDF.

Checks (any failure stops, nothing is written):
    - PDF units and PDF file hash match the extraction (re-extract if the PDF changed);
    - the MJCF body list equals the mapping body list;
    - PDF COM tensor and origin tensor agree (parallel-axis theorem, 0.1 % relative);
    - every tensor is symmetric, positive definite and satisfies the triangle inequality;
    - each new COM lies inside its mesh bounding box (+/- 1 mm);
    - MuJoCo compiles the result with the requested masses, COMs and full tensors;
    - loop gaps at qpos0 < 0.01 mm, and a 100-step zero-gravity smoke test stays finite.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[1]
ORDER = ("Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def tensor(v):
    """6 values (xx yy zz xy xz yz) or a dict with ORDER keys -> symmetric 3x3."""
    if isinstance(v, dict):
        v = [v[k] for k in ORDER]
    xx, yy, zz, xy, xz, yz = v
    return np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]], dtype=float)


def packed(a):
    return [float(a[i, j]) for i, j in ((0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2))]


def rotation_x(degrees):
    a = np.radians(degrees)
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def validate_inertia(name, mass, com, inertia):
    if not np.isfinite(mass) or mass <= 0:
        raise ValueError(f"{name}: mass must be finite and positive")
    if not np.all(np.isfinite(com)) or not np.all(np.isfinite(inertia)):
        raise ValueError(f"{name}: non-finite COM/inertia")
    if not np.allclose(inertia, inertia.T, atol=1e-14, rtol=0):
        raise ValueError(f"{name}: inertia tensor is not symmetric")
    principal = np.linalg.eigvalsh(inertia)
    if principal[0] <= 0 or principal[0] + principal[1] < principal[2] - 1e-12:
        raise ValueError(f"{name}: nonphysical principal inertias {principal}")
    return principal


def from_pdf(name, part, frame):
    """g -> kg, mm -> m, g mm^2 -> kg m^2 (1e-9). PDF tensors are at the COM, signed off-diagonals."""
    mass = part["mass_g"] * 1e-3
    com = np.asarray(part["com_mm"], dtype=float) * 1e-3
    inertia = tensor(part["inertia_com_g_mm2"]) * 1e-9
    validate_inertia(name, mass, com, inertia)
    origin = tensor(part["inertia_origin_g_mm2"]) * 1e-9
    expected = inertia + mass * (com @ com * np.eye(3) - np.outer(com, com))
    residual = float(np.max(np.abs(origin - expected)) / np.max(np.abs(origin)))
    # The PDF rounds some entries to four significant digits and the COM to 0.001 mm.
    if residual > 0.001:
        raise ValueError(f"{name}: PDF COM and origin tensors disagree (relative error {residual:g})")
    rot = rotation_x(frame.get("rotation_x_deg", 0))
    com = rot @ com + np.asarray(frame.get("translation_mm", [0, 0, 0]), dtype=float) * 1e-3
    return mass, com, rot @ inertia @ rot.T, dict(pdf_parallel_axis_relative_error=residual)


def from_scaled_cad(name, part, frame, inertial):
    factor = part["mass_g"] * 1e-3 / frame["reference_cad_mass_kg"]
    mass = float(inertial.get("mass")) * factor
    com = np.fromstring(inertial.get("pos"), sep=" ")
    inertia = tensor(np.fromstring(inertial.get("fullinertia"), sep=" ")) * factor
    return mass, com, inertia, dict(density_factor=factor)


def mesh_bounds(path):
    raw = Path(path).read_bytes()
    count = int.from_bytes(raw[80:84], "little")
    if len(raw) != 84 + 50 * count:
        raise ValueError(f"expected a binary STL: {path}")
    dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    v = np.frombuffer(raw, dtype=dtype, count=count, offset=84)["vertices"].reshape(-1, 3)
    return v.min(axis=0).astype(float), v.max(axis=0).astype(float)


def body_mesh(root, body, src):
    """STL path and scale of the (first) mesh geom of a body."""
    geom = next((g for g in body.findall("geom") if g.get("mesh")), None)
    if geom is None:
        return None, None
    mesh = next(m for m in root.iter("mesh") if m.get("name") == geom.get("mesh"))
    compiler = root.find("compiler")
    meshdir = compiler.get("meshdir", "") if compiler is not None else ""
    scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
    return src.parent / meshdir / mesh.get("file"), scale


def loop_gaps_mm(model, data):
    out = {}
    for e in range(model.neq):
        if model.eq_type[e] == mujoco.mjtEq.mjEQ_CONNECT and model.eq_objtype[e] == mujoco.mjtObj.mjOBJ_SITE:
            out[model.eq(e).name] = float(np.linalg.norm(
                data.site_xpos[model.eq_obj1id[e]] - data.site_xpos[model.eq_obj2id[e]]) * 1000)
    return out


def build(src, properties_path, mapping_path):
    props = json.loads(properties_path.read_text(encoding="utf-8"))
    mapping = json.loads(mapping_path.read_text(encoding="utf-8"))
    if props["units"] != {"mass": "g", "com": "mm", "inertia": "g mm^2"}:
        raise ValueError("unexpected units in the properties file")
    pdf = properties_path.parent / props["source"]
    if digest(pdf) != props["sha256"]:
        raise ValueError(f"{pdf.name} changed: re-extract and review {properties_path.name} first")

    tree = ET.parse(src)
    root = tree.getroot()
    bodies = {b.get("name"): b for b in root.iter("body")}
    if set(bodies) != set(mapping["bodies"]):
        raise ValueError("MJCF body list differs from the mapping. Missing in mapping: "
                         f"{sorted(set(bodies) - set(mapping['bodies']))}; unknown in MJCF: "
                         f"{sorted(set(mapping['bodies']) - set(bodies))}")
    extra = mapping.get("extra_body_mass") or {"kg": 0.0, "body": None}
    if extra.get("body") and extra["body"] not in bodies:
        raise ValueError(f"extra mass body {extra['body']} not in the MJCF")

    params, rows = {}, []
    for name, frame in mapping["bodies"].items():
        part = props["parts"][frame["part"]]
        inertial = bodies[name].find("inertial")
        method = frame.get("method", "pdf")
        if method == "pdf":
            mass, com, inertia, info = from_pdf(name, part, frame)
        elif method == "scale_cad":
            mass, com, inertia, info = from_scaled_cad(name, part, frame, inertial)
        else:
            raise ValueError(f"{name}: unknown method {method!r}")
        if name == extra.get("body") and extra["kg"]:
            inertia = inertia * (mass + extra["kg"]) / mass
            mass += extra["kg"]
        principal = validate_inertia(name, mass, com, inertia)
        stl, scale = body_mesh(root, bodies[name], src)
        if stl is not None:
            lo, hi = mesh_bounds(stl)
            lo, hi = lo * scale, hi * scale
            if np.any(com < lo - 1e-3) or np.any(com > hi + 1e-3):
                raise ValueError(f"{name}: new COM {np.round(com * 1e3, 1)} mm is outside the mesh bounding box")
        params[name] = dict(mass=mass, com=com, inertia=inertia)
        rows.append(dict(body=name, pdf_part=frame["part"], method=method,
                         cad_mass_kg=float(inertial.get("mass")), new_mass_kg=mass,
                         com_m=com.tolist(), fullinertia_kg_m2=packed(inertia),
                         principal_inertia_kg_m2=principal.tolist(), **info))

    for name, p in params.items():
        inertial = bodies[name].find("inertial")
        for k in ("diaginertia", "quat"):
            inertial.attrib.pop(k, None)
        inertial.set("mass", f"{p['mass']:.12g}")
        inertial.set("pos", " ".join(f"{v:.12g}" for v in p["com"]))
        inertial.set("fullinertia", " ".join(f"{v:.12g}" for v in packed(p["inertia"])))
    compiler = root.find("compiler")
    compiler.set("inertiafromgeom", "false")
    compiler.set("balanceinertia", "false")
    root.insert(0, ET.Comment(f" Inertials from {properties_path.name} via {mapping_path.name} "
                              f"(pipeline/apply_mass_properties.py). Do not edit by hand. "))
    ET.indent(tree, space="  ")
    xml = ET.tostring(root, encoding="unicode") + "\n"

    # compile next to the source so relative mesh paths resolve
    tmp = src.with_name(f".{src.stem}.mass.tmp.xml")
    tmp.write_text(xml)
    try:
        model = mujoco.MjModel.from_xml_path(str(tmp))
    finally:
        tmp.unlink(missing_ok=True)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    total = sum(p["mass"] for p in params.values())
    if not np.isclose(model.body_mass.sum(), total, atol=1e-9, rtol=0):
        raise ValueError("compiled mass differs from the requested mass")
    for name, p in params.items():
        b = model.body(name)
        rot = np.empty(9)
        mujoco.mju_quat2Mat(rot, b.iquat)
        rot = rot.reshape(3, 3)
        actual = rot @ np.diag(b.inertia) @ rot.T
        if np.max(np.abs(actual - p["inertia"])) > 1e-6 * np.max(np.abs(p["inertia"])) + 1e-12:
            raise ValueError(f"{name}: compiled inertia mismatch")
        if not np.allclose(b.ipos, p["com"], atol=1e-11, rtol=0):
            raise ValueError(f"{name}: compiled COM mismatch")
    gaps = loop_gaps_mm(model, data)
    if gaps and max(gaps.values()) > 0.01:
        raise ValueError(f"loop gap above 0.01 mm at qpos0: {gaps}")
    model.opt.gravity[:] = 0
    for _ in range(100):
        mujoco.mj_step(model, data)
    if not (np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()) or any(w.number for w in data.warning):
        raise ValueError("zero-gravity smoke test produced a non-finite state or a warning")

    audit = dict(source=src.name, properties=properties_path.name, mapping=mapping_path.name,
                 mujoco_version=mujoco.__version__,
                 cad_mass_kg=sum(r["cad_mass_kg"] for r in rows), extra_body_mass_kg=extra["kg"],
                 total_mass_kg=total, initial_loop_gap_mm=gaps,
                 notes=mapping.get("notes", []), bodies=rows,
                 inputs={p.name: digest(p) for p in (src, properties_path, mapping_path, pdf)},
                 output_sha256=hashlib.sha256(xml.encode()).hexdigest())
    return xml, audit


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("input", type=Path, help="closed MJCF (output of close_loops.py)")
    ap.add_argument("--mapping", type=Path, required=True)
    ap.add_argument("--properties", type=Path, default=REPO / "properties/mass_properties.json")
    ap.add_argument("-o", "--output", type=Path)
    ap.add_argument("--report", action="store_true", help="validate and print, write nothing")
    args = ap.parse_args(argv)
    try:
        xml, audit = build(args.input, args.properties, args.mapping)
    except ValueError as e:
        print(f"[STOP] {e}")
        return 1
    for r in audit["bodies"]:
        print(f"  {r['body']:28s} {r['method']:9s} {r['cad_mass_kg']:8.4f} -> {r['new_mass_kg']:8.4f} kg")
    print(f"mass: CAD {audit['cad_mass_kg']:.3f} kg -> {audit['total_mass_kg']:.3f} kg "
          f"(incl. {audit['extra_body_mass_kg']:.3f} kg allowance); all checks passed")
    if args.report:
        print("[report] nothing written.")
        return 0
    out = args.output or args.input.with_name(args.input.stem + "_mass.xml")
    out.write_text(xml)
    out.with_name(out.stem + "_audit.json").write_text(json.dumps(audit, indent=1) + "\n")
    print(f"wrote: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
