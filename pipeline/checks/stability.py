#!/usr/bin/env python3
"""Linear static stability of the standing pose: how stiff must the motor PD be?

Potential V(dq_m) = M g z_com on the manifold {loops closed, both feet welded to the world},
in motor coordinates (6 motor angles). The gravity Hessian H is computed by finite differences;
kp_crit = -min eig(H) is the minimum equal position gain on the 6 motors for the pose to be
statically stable (rigid loops, rigid feet, no friction or backlash: a lower bound).

Usage:
    python pipeline/checks/stability.py models/xshape-standing-f8/EXPORT_assembly-v8_closed_mass.xml
"""
import argparse

import mujoco
import numpy as np

from common import floating_xml, motors


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("model")
    a = ap.parse_args()

    m = mujoco.MjModel.from_xml_string(floating_xml(a.model, floor=False, weld_feet=True))
    m.opt.jacobian = 0
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    MOTORS = motors(mujoco.MjModel.from_xml_path(a.model))
    mq = [m.jnt_qposadr[m.joint(j).id] for j in MOTORS]
    mdof = [m.jnt_dofadr[m.joint(j).id] for j in MOTORS]
    S = np.zeros((6, m.nv)); S[range(6), mdof] = 1
    q0 = d.qpos.copy(); M = m.body_mass.sum(); g = -m.opt.gravity[2]

    def potential(delta):
        d.qpos[:] = q0
        tgt = q0[mq] + delta
        for _ in range(50):
            mujoco.mj_forward(m, d)
            eq = d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
            J = np.vstack([d.efc_J.reshape(d.nefc, m.nv)[eq], S])
            r = np.concatenate([d.efc_pos[eq], d.qpos[mq] - tgt])
            if np.abs(r).max() < 1e-12:
                break
            dq = -np.linalg.lstsq(J, r, rcond=None)[0]
            mujoco.mj_integratePos(m, d.qpos, dq, 1.0)
        return M * g * ((m.body_mass[:, None] * d.xipos).sum(0) / M)[2]

    h = 1e-4; H = np.zeros((6, 6))
    for i in range(6):
        for j in range(i, 6):
            e = np.zeros(6); f = np.zeros(6); e[i] = h; f[j] = h
            H[i, j] = H[j, i] = (potential(e + f) - potential(e - f) - potential(-e + f) + potential(-e - f)) / (4 * h * h)
    w, v = np.linalg.eigh(H)
    labels = [lab for lab, _, _ in MOTORS.values()]
    print(f"mass {M:.3f} kg")
    print("gravity Hessian eigenvalues [N m/rad]:", np.round(w, 1))
    print("most unstable mode (motor coords):", dict(zip(labels, np.round(v[:, 0], 2))))
    print(f"=> kp_crit (rigid loops, rigid feet, equal kp on 6 motors) = {-w[0]:.0f} N m/rad")


if __name__ == "__main__":
    main()
