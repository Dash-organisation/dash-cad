#!/usr/bin/env python3
"""repose_init.py: re-pose a closed MJCF so that qpos = 0 is the init (kneeling) pose of a reference.

Fusion exports the assembly in whatever pose it was saved in (usually standing), and that pose
becomes qpos = 0. The controller and the web twin need qpos = 0 = the INIT pose, where the motor
zeros are captured. This script takes the init pose from a reference model (same motor-body
orientations = same motor angles), drives the new model there with the loops solved at every step,
and rewrites only the body pos/euler so that pose becomes qpos = 0.

Usage:
    python pipeline/repose_init.py <reference_init.xml> <closed.xml> -o <out.xml> [--name MODEL]

    e.g. the xshape init model:
    python pipeline/repose_init.py models/v2-init-f7/dash01_homing_v7.xml \\
        models/xshape-standing-f8/EXPORT_assembly-v8_closed.xml -o models/xshape-init-f8/dash01_homing_v8.xml

Joints are found from the body they move (no hard-coded "Revolute-N"):
    motors   hip roll = joint of the "hip" body, cam = "cam", thigh = "thigh" (or the exporter's "thight")
    loops    knee = joints of "pushrod" + "tibia", achilles = joints of "achilles" + "foot"
    side     from the body position at qpos 0 (left leg at y > 0), so side-less foot names work.
Joints, sites and inertials are untouched; "thight" body/geom names are renamed "thigh" so the
web twin's motor regexes match. Needs mujoco and scipy.
"""
from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

PARTS = {"hip": "hip", "cam": "cam", "thigh": "thigh", "pushrod": "pushrod", "tibia": "tibia",
         "achilles": "achilles", "foot": "foot"}


def part_of(name):
    n = name.lower().replace("thight", "thigh")
    for key in ("pushrod", "achilles", "tibia", "thigh", "foot", "hip", "cam"):
        if key in n:
            return key
    return None


def joints_by_part(m):
    """{side: {part: joint id}} for the single hinge of each leg body."""
    d = mujoco.MjData(m)
    mujoco.mj_kinematics(m, d)
    out = {"L": {}, "R": {}}
    for b in range(1, m.nbody):
        part = part_of(m.body(b).name)
        if part is None or m.body_jntnum[b] != 1:
            continue
        side = "L" if d.xpos[b][1] > 0 else "R"
        if part in out[side]:
            raise SystemExit(f"[STOP] two '{part}' bodies on side {side}")
        out[side][part] = m.body_jntadr[b]
    for side, parts in out.items():
        missing = set(PARTS) - set(parts)
        if missing:
            raise SystemExit(f"[STOP] side {side}: no body found for {sorted(missing)}")
    return out


def rel_rot(m, d, b):
    p = m.body_parentid[b]
    return Rot.from_matrix(d.xmat[p].reshape(3, 3)).inv() * Rot.from_matrix(d.xmat[b].reshape(3, 3))


def loop_res(m, d, name):
    e = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, name)
    if e < 0:
        raise SystemExit(f"[STOP] no equality '{name}': run close_loops.py first")
    return d.site_xpos[m.eq_obj1id[e]] - d.site_xpos[m.eq_obj2id[e]]


def solve(m, d, joints, eq):
    idx = [m.jnt_qposadr[j] for j in joints]

    def f(x):
        d.qpos[idx] = x
        mujoco.mj_kinematics(m, d)
        return loop_res(m, d, eq)
    f(least_squares(f, d.qpos[idx].copy(), xtol=1e-14, ftol=1e-14, gtol=1e-14).x)
    return np.linalg.norm(loop_res(m, d, eq))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("reference", help="model whose qpos 0 is the init pose (e.g. dash01_homing_v7.xml)")
    ap.add_argument("closed", help="closed MJCF to re-pose (output of close_loops / apply_mass_properties)")
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("--name", help="model name written in the file (default: output file stem)")
    args = ap.parse_args(argv)

    mr = mujoco.MjModel.from_xml_path(args.reference); dr = mujoco.MjData(mr); mujoco.mj_kinematics(mr, dr)
    m = mujoco.MjModel.from_xml_path(args.closed); d = mujoco.MjData(m); mujoco.mj_kinematics(m, d)
    jr, jn = joints_by_part(mr), joints_by_part(m)
    target = {}
    for s in "LR":
        for part in ("hip", "cam", "thigh"):
            j_ref, j_new = jr[s][part], jn[s][part]
            ax = m.jnt_axis[j_new] / np.linalg.norm(m.jnt_axis[j_new])
            rv = (rel_rot(m, d, m.jnt_bodyid[j_new]).inv() * rel_rot(mr, dr, mr.jnt_bodyid[j_ref])).as_rotvec()
            if np.linalg.norm(rv - (rv @ ax) * ax) > 1e-6:
                raise SystemExit(f"[STOP] {part} {s}: the reference orientation is not reachable about the joint axis "
                                 "(hip/cam/thigh frames differ between the two exports)")
            target[m.jnt_qposadr[j_new]] = rv @ ax
            print(f"{part:6s} {s}  {m.joint(j_new).name:12s} export -> init {np.degrees(rv @ ax):+9.3f} deg")
    worst = 0.0
    for t in np.linspace(0, 1, 121)[1:]:        # continuation keeps the CAD assembly branch
        for a, q in target.items():
            d.qpos[a] = t * q
        for s in "LR":
            worst = max(worst, solve(m, d, [jn[s]["pushrod"], jn[s]["tibia"]], f"loop_knee_{s}"),
                        solve(m, d, [jn[s]["achilles"], jn[s]["foot"]], f"loop_achilles_{s}"))
    print(f"max loop residual on the way: {worst * 1e3:.2e} mm")

    tree = ET.parse(args.closed); root = tree.getroot()
    root.set("model", args.name or args.output.rsplit("/", 1)[-1].removesuffix(".xml"))
    for b in root.iter("body"):
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b.get("name")); p = m.body_parentid[i]
        Rp = d.xmat[p].reshape(3, 3)
        b.set("pos", " ".join(repr(float(v)) for v in Rp.T @ (d.xpos[i] - d.xpos[p])))
        R = Rp.T @ d.xmat[i].reshape(3, 3)
        b.set("euler", " ".join(repr(float(v)) for v in Rot.from_matrix(R).as_euler("xyz")))  # MJCF "XYZ"
    for el in root.iter():
        if el.tag in ("body", "geom") and el.get("name"):
            el.set("name", el.get("name").replace("thight", "thigh"))
    ET.indent(tree, "    ")
    with open(args.output, "w") as f:
        f.write('<?xml version="1.0" ?>\n' + ET.tostring(root, encoding="unicode") + "\n")
    m2 = mujoco.MjModel.from_xml_path(args.output); d2 = mujoco.MjData(m2); mujoco.mj_kinematics(m2, d2)
    err = max(np.linalg.norm(loop_res(m2, d2, f"loop_{k}_{s}")) for k in ("knee", "achilles") for s in "LR")
    print(f"wrote {args.output}; loops at qpos 0: {err * 1e3:.2e} mm; "
          f"body pose error {np.abs(d2.xpos - d.xpos).max() * 1e3:.1e} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
