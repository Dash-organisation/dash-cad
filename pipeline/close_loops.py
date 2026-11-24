#!/usr/bin/env python3
"""
close_loops.py: close the kinematic loops of an MJCF exported by ACDC4Robot.

ACDC4Robot exports a TREE (its preflight refuses loops). After the export, this script adds:
    - a <site> on the link (rod) and a <site> on the attachment body, at the centre of the pin bore;
    - one <equality><connect site1 site2> per loop.

Usage:
    python pipeline/close_loops.py <export_folder | file.xml> [-o out.xml] [-c loops.json] [--dry-run]

    Default output: <name>_closed.xml next to the export (STL paths stay valid),
    plus <name>_closed_closures.json (report: points, radii, offsets, test results).
    The original export is never modified.

Method (per loop, see loops.json):
    1. "link" bodies (rods) found by regex on the name, e.g. pushrod, achilles.
    2. Bore axis = axis of the rod's parent joint (world frame, at qpos0).
    3. Distal bore of the rod: mesh vertices near its end farthest from the origin, projected on the
       plane normal to the axis, least-squares circle fit at the expected radius (bore_radius_m),
       iterated on the inliers.
    4. Attachment body: among the "target" bodies (regex), the one whose mesh has a bore of the same
       radius at the same place (offset < max_bore_offset_m). Names do not need to carry the side.
    5. Closure point = bore centre, at half the rod thickness along the axis.

Units / frames: SI (m, rad), MuJoCo world frame, qpos0 = pose of the Fusion assembly at export.
Assumption: the assembly was exported with the loops in place (pins aligned in Fusion).

Tests (pass/fail, exit code 0 = all OK, 1 = failure):
    - bore found on both parts, centre offset < max_bore_offset_m;
    - connect violation at qpos0 < tol_qpos0_m;
    - sim_duration_s simulation (gravity, fixed base, no actuator): no NaN, max violation < tol_sim_m;
    - remaining DOF printed (rank of the constraint Jacobian) and compared with expected_dof if set.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent


# ----------------------------------------------------------------------------- geometry
def mesh_vertices_world(model, data, body_id):
    """Vertices [m, world] of all meshes of a body, at the current pose."""
    pts = []
    for g in range(model.ngeom):
        if model.geom_bodyid[g] != body_id or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = model.geom_dataid[g]
        a, n = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        v = model.mesh_vert[a:a + n].reshape(-1, 3)
        pts.append(data.geom_xpos[g] + v @ data.geom_xmat[g].reshape(3, 3).T)
    return np.vstack(pts) if pts else np.empty((0, 3))


def plane_basis(axis):
    """Two unit vectors orthogonal to axis (basis of the bore plane)."""
    a = axis / np.linalg.norm(axis)
    helper = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 0, 1.0])
    u = np.cross(a, helper)
    u /= np.linalg.norm(u)
    return a, u, np.cross(a, u)


def fit_circle_2d(p):
    """Least-squares circle (Kasa): centre (2,), radius."""
    A = np.c_[2 * p, np.ones(len(p))]
    c = np.linalg.lstsq(A, (p ** 2).sum(1), rcond=None)[0]
    return c[:2], float(np.sqrt(max(c[2] + c[0] ** 2 + c[1] ** 2, 0.0)))


def find_bore(verts_w, axis_w, center_guess_w, r0, tol, window, min_inliers, axial_window=0.03):
    """Look for a bore of radius ~r0 near center_guess_w (in the plane normal to the axis AND within
    axial_window along the axis, so the opposite leg is never matched). Returns a dict or None."""
    if len(verts_w) == 0:
        return None
    a, u, v = plane_basis(axis_w)
    P = np.c_[verts_w @ u, verts_w @ v]
    c = np.array([center_guess_w @ u, center_guess_w @ v])
    near = (np.linalg.norm(P - c, axis=1) < window) & (np.abs((verts_w - center_guess_w) @ a) < axial_window)
    if near.sum() < min_inliers:
        return None
    P_near, V_near = P[near], verts_w[near]
    # starting point: the points near the estimated centre (the bore wall)
    r = r0
    for _ in range(8):
        res = np.abs(np.linalg.norm(P_near - c, axis=1) - r)
        inl = res < tol
        if inl.sum() < min_inliers:
            return None
        c, r = fit_circle_2d(P_near[inl])
    res = np.abs(np.linalg.norm(P_near[inl] - c, axis=1) - r)
    if abs(r - r0) > tol:
        return None
    s = V_near[inl] @ a
    center_w = c[0] * u + c[1] * v + 0.5 * (s.min() + s.max()) * a  # half thickness along the axis
    return dict(center_w=center_w, radius_m=r, n_inliers=int(inl.sum()),
                resid_max_m=float(res.max()), axial_span_m=(float(s.min()), float(s.max())))


def distal_bore(verts_w, origin_w, axis_w, cfg):
    """Bore at the end of the rod farthest from its origin (proximal joint)."""
    d = np.linalg.norm(verts_w - origin_w, axis=1)
    tip = verts_w[d.argmax()]
    eye = verts_w[np.linalg.norm(verts_w - tip, axis=1) < cfg["eye_window_m"]]
    r0, tol = cfg["bore_radius_m"], cfg["radius_tol_m"]
    # RANSAC: circles through 3 points of the eye (projected on the plane normal to the axis); keep
    # those of radius ~r0, take the one with the most inliers, then refine with find_bore.
    a, u, v = plane_basis(axis_w)
    P = np.unique(np.round(np.c_[eye @ u, eye @ v], 7), axis=0)
    rng = np.random.default_rng(0)
    best_c, best_n = None, 0
    if len(P) >= 3:
        for _ in range(cfg.get("ransac_iters", 3000)):
            p1, p2, p3 = P[rng.choice(len(P), 3, replace=False)]
            dd = 2 * (p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1]))
            if abs(dd) < 1e-12:
                continue
            s1, s2, s3 = p1 @ p1, p2 @ p2, p3 @ p3
            c = np.array([s1 * (p2[1] - p3[1]) + s2 * (p3[1] - p1[1]) + s3 * (p1[1] - p2[1]),
                          s1 * (p3[0] - p2[0]) + s2 * (p1[0] - p3[0]) + s3 * (p2[0] - p1[0])]) / dd
            if abs(np.linalg.norm(p1 - c) - r0) > tol:
                continue
            n = int((np.abs(np.linalg.norm(P - c, axis=1) - r0) < tol / 4).sum())
            if n > best_n:
                best_c, best_n = c, n
    best = None
    if best_c is not None:
        seed_w = best_c[0] * u + best_c[1] * v + (eye.mean(0) @ a) * a
        best = find_bore(verts_w, axis_w, seed_w, r0, tol, 3 * r0, cfg["min_inliers"], cfg["axial_window_m"])
    if best is not None:
        best["link_length_m"] = float(np.linalg.norm(best["center_w"] - origin_w))
    return best, tip


# ----------------------------------------------------------------------------- MJCF
def resolve_input(path: Path) -> Path:
    if path.is_dir():
        xmls = [p for p in path.glob("*.xml") if "_closed" not in p.stem]
        if len(xmls) != 1:
            sys.exit(f"[STOP] {path} contains {len(xmls)} exported .xml files; pass the file instead.")
        return xmls[0]
    return path


def eq_violation_m(data):
    is_eq = data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
    return float(np.abs(data.efc_pos[is_eq]).max(initial=0.0))


def side_of(p_world):
    return "L" if p_world[1] > 0 else "R"  # repo convention: left leg at y > 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("input", type=Path, help="ACDC4Robot export folder or .xml file")
    ap.add_argument("-o", "--output", type=Path, help="closed MJCF (default: <name>_closed.xml)")
    ap.add_argument("-c", "--config", type=Path, default=HERE / "loops.json")
    ap.add_argument("--dry-run", action="store_true", help="detect and test, write nothing")
    args = ap.parse_args(argv)

    cfg = json.loads(args.config.read_text())
    src = resolve_input(args.input)
    dst = args.output or src.with_name(src.stem + "_closed.xml")
    report_path = dst.with_name(dst.stem + "_closures.json")

    model = mujoco.MjModel.from_xml_path(str(src))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)  # qpos0 = export pose
    tree = ET.parse(src)
    root = tree.getroot()
    xml_bodies = {b.get("name"): b for b in root.iter("body")}
    body_names = [model.body(i).name for i in range(model.nbody)]

    # idempotence: an already closed export is refused
    if any(c.get("name", "").startswith("loop_") for c in root.iter("connect")):
        print(f"[STOP] {src.name} already contains 'loop_*' connects: start from the raw export.")
        return 1

    ok = True
    closures = []
    for loop in cfg["loops"]:
        lcfg = {**cfg["defaults"], **loop}
        links = [n for n in body_names if re.search(loop["link"], n)]
        targets = [n for n in body_names if re.search(loop["target"], n)]
        if loop.get("count") is not None and len(links) != loop["count"]:
            print(f"[FAIL] loop '{loop['name']}': {len(links)} rod(s) found ({links}), "
                  f"{loop['count']} expected")
            ok = False
        for link in links:
            b1 = model.body(link).id
            jadr = model.body_jntadr[b1]
            if jadr < 0 or model.jnt_type[jadr] != mujoco.mjtJoint.mjJNT_HINGE:
                print(f"[FAIL] {link}: no parent hinge, bore axis unknown")
                ok = False
                continue
            axis_w = data.xaxis[jadr].copy()
            origin_w = data.xanchor[jadr].copy()
            bore1, tip = distal_bore(mesh_vertices_world(model, data, b1), origin_w, axis_w, lcfg)
            if bore1 is None:
                print(f"[FAIL] {link}: no bore of r~{lcfg['bore_radius_m'] * 1e3:.2f} mm "
                      f"near the end {np.round(tip, 4)} m. Check bore_radius_m in loops.json.")
                ok = False
                continue
            # attachment body: the one with the same bore at the same place
            cands = []
            for t in targets:
                if t == link:
                    continue
                bt = find_bore(mesh_vertices_world(model, data, model.body(t).id), axis_w,
                               bore1["center_w"], lcfg["bore_radius_m"],
                               lcfg["target_radius_tol_m"], 3 * lcfg["bore_radius_m"], lcfg["min_inliers"],
                               lcfg["axial_window_m"])
                if bt is not None:
                    a = axis_w / np.linalg.norm(axis_w)
                    delta = bt["center_w"] - bore1["center_w"]
                    radial = float(np.linalg.norm(delta - (delta @ a) * a))
                    cands.append((radial, t, bt))
            if not cands:
                print(f"[FAIL] {link}: no '{loop['target']}' body has a matching bore")
                ok = False
                continue
            radial, target, bore2 = min(cands, key=lambda x: x[0])
            if radial > lcfg["max_bore_offset_m"]:
                print(f"[FAIL] {link} -> {target}: bore centres {radial * 1e3:.3f} mm apart "
                      f"(> {lcfg['max_bore_offset_m'] * 1e3:.2f} mm). Loop badly assembled in Fusion?")
                ok = False
                continue
            p_w = bore1["center_w"]
            side = side_of(p_w)
            p_t = data.xpos[model.body(target).id]
            if side_of(p_t) != side:
                print(f"[FAIL] {link} ({side}) matched to {target} ({side_of(p_t)}): different sides")
                ok = False
                continue
            closures.append(dict(loop=loop["name"], side=side, link=link, target=target, point_world_m=p_w,
                                 link_bore=bore1, target_bore=bore2, bore_offset_m=radial,
                                 solref=lcfg["solref"], solimp=lcfg["solimp"]))
            print(f"[ok] {loop['name']} {side}: {link} -> {target} | r = {bore1['radius_m'] * 1e3:.3f} / "
                  f"{bore2['radius_m'] * 1e3:.3f} mm, centre offset {radial * 1e3:.3f} mm, "
                  f"pin distance {bore1['link_length_m'] * 1e3:.1f} mm")

    used = [c["target"] for c in closures]
    if len(set(used)) != len(used):
        print(f"[FAIL] one attachment body is used by two rods: {used}")
        ok = False
    names = [(c["loop"], c["side"]) for c in closures]
    if len(set(names)) != len(names):
        print(f"[FAIL] two loops have the same name/side: {names}")
        ok = False
    if not ok:
        print("[STOP] incomplete detection: nothing written.")
        return 1

    # ------------------------------------------------------------------ write XML
    fmt = lambda v: " ".join(f"{x:.7g}" for x in v)
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")
    for c in closures:
        tag = f"loop_{c['loop']}_{c['side']}"
        for role, bname in (("link", c["link"]), ("target", c["target"])):
            bid = model.body(bname).id
            p_local = data.xmat[bid].reshape(3, 3).T @ (c["point_world_m"] - data.xpos[bid])
            xml_bodies[bname].append(ET.Element("site", name=f"{tag}_{role}", pos=fmt(p_local),
                                                size="0.004", rgba="1 0.3 0.1 1"))
        ET.SubElement(equality, "connect", name=tag, site1=f"{tag}_link", site2=f"{tag}_target",
                      solref=c["solref"], solimp=c["solimp"])
    ET.indent(tree, space="  ")
    xml_text = ET.tostring(root, encoding="unicode")

    # ------------------------------------------------------------------ validation
    # compile from the export folder so the relative mesh paths resolve
    tmp = src.with_name(f".{dst.stem}.tmp.xml")
    tmp.write_text(xml_text)
    try:
        m2 = mujoco.MjModel.from_xml_path(str(tmp))
    finally:
        tmp.unlink(missing_ok=True)
    d2 = mujoco.MjData(m2)
    mujoco.mj_forward(m2, d2)
    v0 = eq_violation_m(d2)
    is_eq = d2.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
    J = d2.efc_J.reshape(d2.nefc, m2.nv)[is_eq] if d2.nefc else np.zeros((0, m2.nv))
    sv = np.linalg.svd(J, compute_uv=False) if len(J) else np.zeros(0)
    rank = int((sv > 1e-6 * sv[0]).sum()) if len(sv) else 0
    dof = m2.nv - rank

    vmax = 0.0
    for _ in range(int(cfg["checks"]["sim_duration_s"] / m2.opt.timestep)):
        mujoco.mj_step(m2, d2)
        vmax = max(vmax, eq_violation_m(d2))
    finite = bool(np.all(np.isfinite(d2.qpos)))

    chk = cfg["checks"]
    tests = {
        "violation_qpos0": (v0 < chk["tol_qpos0_m"], f"{v0 * 1e3:.4f} mm (< {chk['tol_qpos0_m'] * 1e3} mm)"),
        "sim_finite": (finite, "no NaN" if finite else "NaN"),
        "violation_sim": (vmax < chk["tol_sim_m"],
                          f"{vmax * 1e3:.3f} mm over {chk['sim_duration_s']} s (< {chk['tol_sim_m'] * 1e3} mm)"),
        "dof": (chk.get("expected_dof") in (None, dof),
                f"{m2.nv} joints - rank {rank} = {dof} DOF"
                + (f" (expected {chk['expected_dof']})" if chk.get("expected_dof") is not None else "")),
    }
    all_ok = all(t[0] for t in tests.values())
    for k, (passed, msg) in tests.items():
        print(f"[{'OK' if passed else 'FAIL'}] {k}: {msg}")

    def js(x):
        if isinstance(x, np.ndarray):
            return [float(v) for v in x]
        if isinstance(x, dict):
            return {k: js(v) for k, v in x.items()}
        if isinstance(x, (list, tuple)):
            return [js(v) for v in x]
        return x

    report = dict(source=str(src.name), output=str(dst.name), config=str(args.config.name),
                  closures=js(closures), tests={k: dict(passed=p, detail=m) for k, (p, m) in tests.items()},
                  mujoco_version=mujoco.__version__)

    if args.dry_run:
        print("[dry-run] nothing written.")
    elif not all_ok:
        print("[STOP] failed tests: nothing written.")
    else:
        dst.write_text(xml_text)
        report_path.write_text(json.dumps(report, indent=1, ensure_ascii=False))
        print(f"wrote: {dst}\nreport: {report_path}")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
