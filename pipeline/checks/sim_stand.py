#!/usr/bin/env python3
"""Free-standing simulation: the robot on the floor, 6 motors held in PD at their qpos 0 angles.

A quick sanity check that the model can stand with a given stiffness, before any training. It is
NOT the training plant (no sole contact patches, no motor model, no noise): see dash-sim2real.

Usage:
    python pipeline/checks/sim_stand.py models/xshape-standing-f8/EXPORT_assembly-v8_closed_mass.xml [--kp 200] [--kv 5] [-t 5]

Prints torso tilt, CoM drop and drift, joint sag, motor torques and the CoP of each foot after t s.
Armature: Morand thesis Tab. 3.1 reflected rotor inertia; passive joints as the previous team's plant.
"""
import argparse

import mujoco
import numpy as np

from common import body_points, feet, floating_xml, motors

ARMATURE = {"hip_roll": 0.046, "cam": 0.0216, "thigh": 0.0216}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("model")
    ap.add_argument("--kp", type=float, default=200.0)
    ap.add_argument("--kv", type=float, default=5.0)
    ap.add_argument("-t", "--duration", type=float, default=5.0)
    a = ap.parse_args()

    m0 = mujoco.MjModel.from_xml_path(a.model); d0 = mujoco.MjData(m0); mujoco.mj_forward(m0, d0)
    FEET, MOTORS = feet(m0), motors(m0)
    z0 = -min(body_points(m0, d0, b)[:, 2].min() for b in FEET.values()) + 0.0005
    m = mujoco.MjModel.from_xml_string(floating_xml(a.model, kp=a.kp, kv=a.kv, z=z0))
    for j, (lab, _, _) in MOTORS.items():
        dof = m.jnt_dofadr[m.joint(j).id]
        m.dof_armature[dof] = ARMATURE[lab.rsplit("_", 1)[0]]; m.dof_damping[dof] = 0.1
    for j in range(m.njnt):
        if m.joint(j).name not in MOTORS and m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
            m.dof_armature[m.jnt_dofadr[j]] = 0.001; m.dof_damping[m.jnt_dofadr[j]] = 0.2
    d = mujoco.MjData(m)
    qa = [m.jnt_qposadr[m.actuator_trnid[i, 0]] for i in range(m.nu)]
    d.ctrl[:] = d.qpos[qa]
    mujoco.mj_forward(m, d)
    M = m.body_mass.sum()
    com = lambda: (m.body_mass[:, None] * d.xipos).sum(0) / M
    c0 = com(); q0 = d.qpos[qa].copy()
    while d.time < a.duration:
        mujoco.mj_step(m, d)
    c1 = com(); q1 = d.qpos[qa]
    up = d.xmat[m.body(m0.body(1).name).id].reshape(3, 3)[:, 2]
    name = lambda i: MOTORS[m.joint(m.actuator_trnid[i, 0]).name][0]
    print(f"mass {M:.3f} kg, kp={a.kp}, kv={a.kv}: after {a.duration}s torso tilt {np.degrees(np.arccos(up[2])):.2f} deg"
          f" | CoM drop {(c0[2] - c1[2]) * 1000:.1f} mm | CoM dx {(c1[0] - c0[0]) * 1000:+.1f} mm")
    print("  joint sag [deg]:", ", ".join(f"{name(i)} {np.degrees(q1[i] - q0[i]):+.2f}" for i in range(m.nu)))
    print("  motor torque [N m]:", ", ".join(f"{name(i)} {d.actuator_force[i]:+.1f}" for i in range(m.nu)))
    for side, fb in FEET.items():
        bid = m.body(fb).id; F = 0.0; Mx = 0.0
        for k in range(d.ncon):
            con = d.contact[k]
            if bid in (m.geom_bodyid[con.geom1], m.geom_bodyid[con.geom2]):
                f6 = np.zeros(6); mujoco.mj_contactForce(m, d, k, f6)
                F += f6[0]; Mx += f6[0] * con.pos[0]
        print(f"  foot {side}: Fz {F:.1f} N, CoP x {Mx / F if F else float('nan'):.3f} m")


if __name__ == "__main__":
    main()
