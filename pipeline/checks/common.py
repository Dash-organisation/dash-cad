"""Shared helpers for the model checks: load a closed MJCF, find motors and feet by body name.

Motors (Morand thesis Tab. 3.1, continuous / peak torque at the joint):
    hip roll  AK60-39   24.0 / 61.2 N m
    cam       AKE90-8   55.0 / 144.5 N m
    thigh     AKE90-8   55.0 / 144.5 N m
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

TORQUE_NM = {"hip": (24.0, 61.2), "cam": (55.0, 144.5), "thigh": (55.0, 144.5)}
LABEL = {"hip": "hip_roll", "cam": "cam", "thigh": "thigh"}


def part_of(name):
    n = name.lower().replace("thight", "thigh")
    for key in ("pushrod", "achilles", "tibia", "thigh", "foot", "hip", "cam"):
        if key in n:
            return key
    return None


def sides(model):
    """{body id: 'L'|'R'} from the body position at qpos 0 (left leg at y > 0)."""
    d = mujoco.MjData(model)
    mujoco.mj_kinematics(model, d)
    return {b: ("L" if d.xpos[b][1] > 0 else "R") for b in range(1, model.nbody)}


def motors(model):
    """Ordered {joint name: (label, cont N m, peak N m)} for the six actuated joints."""
    side = sides(model)
    out = {}
    for s in "RL":
        for part in ("hip", "cam", "thigh"):
            for b in range(1, model.nbody):
                if part_of(model.body(b).name) == part and side[b] == s and model.body_jntnum[b] == 1:
                    j = model.body_jntadr[b]
                    out[model.joint(j).name] = (f"{LABEL[part]}_{s}", *TORQUE_NM[part])
    if len(out) != 6:
        raise SystemExit(f"[STOP] expected 6 motor joints, found {list(out)}")
    return out


def feet(model):
    """{'R': body name, 'L': body name}."""
    side = sides(model)
    out = {side[b]: model.body(b).name for b in range(1, model.nbody) if part_of(model.body(b).name) == "foot"}
    if set(out) != {"L", "R"}:
        raise SystemExit(f"[STOP] expected one foot per side, found {out}")
    return out


def body_points(m, d, body):
    pts = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != m.body(body).id or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        pts.append(v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])
    return np.vstack(pts)


def sole_points(m, d, body, tol):
    w = body_points(m, d, body)
    return w[w[:, 2] < w[:, 2].min() + tol], w[:, 2].min()


def floating_xml(path, *, floor=True, weld_feet=False, kp=None, kv=None, z=None):
    """Closed MJCF -> XML string with a free base (and optionally floor, foot contacts, welds, PD)."""
    path = Path(path)
    root = ET.parse(path).getroot()
    compiler = root.find("compiler")
    compiler.set("meshdir", str((path.parent / compiler.get("meshdir", "")).resolve()))
    world = root.find("worldbody")
    base = world.find("body")
    base.set("pos", f"0 0 {z or 0}")
    base.attrib.pop("euler", None)
    base.insert(0, ET.Element("freejoint", name="root"))
    m0 = mujoco.MjModel.from_xml_path(str(path))
    foot = feet(m0)
    if floor:
        world.insert(0, ET.Element("geom", name="floor", type="plane", size="5 5 0.1",
                                   contype="1", conaffinity="1"))
        for b in root.iter("body"):
            if b.get("name") in foot.values():
                for g in b.findall("geom"):
                    g.set("contype", "1"); g.set("conaffinity", "1"); g.set("friction", "0.8")
    eq = root.find("equality")
    if weld_feet:
        for s, name in foot.items():
            ET.SubElement(eq, "weld", name=f"fix_{s}", body1=name)
    if kp is not None:
        act = ET.SubElement(root, "actuator")
        for j, (_, _, peak) in motors(m0).items():
            ET.SubElement(act, "position", joint=j, kp=str(kp), kv=str(kv or 0),
                          forcerange=f"-{peak} {peak}", forcelimited="true")
    return ET.tostring(root, encoding="unicode")
