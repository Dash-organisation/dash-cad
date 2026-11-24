#!/usr/bin/env python3
"""Static standing check of a closed MJCF, in its qpos 0 pose, with the masses in the file.

Motors locked => 0 DOF => the robot is one rigid body on two flat feet.
  1) CoM vs support polygon (convex hull of both soles): margin, tip angle, push to tip.
  2) Quasi-static motor torques: bias(q) = S^T tau + J_eq^T lam + J_foot^T f (qvel = qacc = 0),
     swept over the (statically indeterminate) fore-aft CoP location, symmetric loading.

Usage:
    python pipeline/checks/static_stand.py models/xshape-standing-f8/EXPORT_assembly-v8_closed_mass.xml [--tol 0.001]

Run it on the *_mass.xml model: the raw export has CAD masses (about 5.3 kg instead of 15).
Frames: MuJoCo world, x forward, y left, z up; m, N, N m.
"""
import argparse

import mujoco
import numpy as np
from scipy.spatial import ConvexHull

from common import feet, motors, sole_points


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("model")
    ap.add_argument("--tol", type=float, default=0.001, help="sole contact tolerance [m]")
    a = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(a.model)
    m.opt.jacobian = 0  # dense
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    MOTORS, FEET = motors(m), feet(m)

    M = m.body_mass[1:].sum(); g = -m.opt.gravity[2]
    com = (m.body_mass[:, None] * d.xipos).sum(0) / M
    soles = {s: sole_points(m, d, b, a.tol) for s, b in FEET.items()}
    zs = min(z for _, z in soles.values())
    print(f"mass {M:.3f} kg | CoM world {np.round(com, 4)} | CoM height above sole {com[2] - zs:.3f} m")
    for s, (p, z) in soles.items():
        print(f"  sole {s}: {len(p)} pts, z={z:.4f}, x[{p[:, 0].min():.3f},{p[:, 0].max():.3f}] "
              f"y[{p[:, 1].min():.3f},{p[:, 1].max():.3f}]")

    xy = np.vstack([p[:, :2] for p, _ in soles.values()])
    hull = ConvexHull(xy)
    marg = -(hull.equations[:, :2] @ com[:2] + hull.equations[:, 2])  # n.x + c <= 0 inside
    print(f"CoM inside support polygon: {bool((marg > 0).all())} | min margin {marg.min() * 100:.1f} cm"
          f" | tip angle {np.degrees(np.arctan(marg.min() / (com[2] - zs))):.1f} deg"
          f" | push to tip at CoM height {M * g * marg.min() / (com[2] - zs):.1f} N")

    eq = d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
    Jeq = d.efc_J.reshape(d.nefc, m.nv)[eq]
    bias = d.qfrc_bias.copy()  # = gravity at qvel = 0
    mdof = [m.jnt_dofadr[m.joint(j).id] for j in MOTORS]
    S = np.zeros((len(mdof), m.nv)); S[range(len(mdof)), mdof] = 1
    yR = soles["R"][0][:, 1].mean(); yL = soles["L"][0][:, 1].mean()
    FR = M * g * (com[1] - yL) / (yR - yL); FL = M * g - FR
    xmin = max(p[:, 0].min() for p, _ in soles.values()); xmax = min(p[:, 0].max() for p, _ in soles.values())
    print(f"\nvertical GRF R {FR:.1f} N, L {FL:.1f} N. Sweep CoP_R x (CoP_L set by moment balance):")
    print("CoP_R x  CoP_L x  " + "  ".join(f"{v[0]:>10s}" for v in MOTORS.values()) + "   resid")
    for xR in np.linspace(xmin, xmax, 9):
        xL = (M * g * com[0] - FR * xR) / FL
        if not (xmin <= xL <= xmax):
            continue
        rhs = bias.copy()
        for s, xp, yp, F in (("R", xR, yR, FR), ("L", xL, yL, FL)):
            jp = np.zeros((3, m.nv)); jr = np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, jp, jr, np.array([xp, yp, zs]), m.body(FEET[s]).id)
            rhs -= jp.T @ np.array([0, 0, F])
        A = np.hstack([S.T, Jeq.T])
        sol, *_ = np.linalg.lstsq(A, rhs, rcond=None)
        tau = sol[:len(mdof)]; res = np.linalg.norm(A @ sol - rhs)
        print(f"{xR:7.3f}  {xL:7.3f}  " + "  ".join(f"{t:10.1f}" for t in tau) + f"   {res:.1e}")
    print("limits cont/peak: " + ", ".join(f"{v[0]} {v[1]:.0f}/{v[2]:.0f}" for v in MOTORS.values()))


if __name__ == "__main__":
    main()
