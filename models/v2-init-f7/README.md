> Written when this model lived in the robot repo (`dash-01CAD/homing/`); paths below refer to that
> layout. In dash-cad, `tools/derive_init.py` is [`pipeline/repose_init.py`](../../pipeline/repose_init.py).

# dash01_homing_v7: the init pose of the rebuilt DASH-01

This folder defines the robot's **init pose: kneeling**. The knees are bent (about 60° more than
standing), the feet are folded back and both hips are against their outward stops. The six drives
are zeroed with the robot held in this pose.

**Two poses, two names:**
- **Init (kneeling)**: where the zero is captured. Files and code identifiers call it "homing"
  (this folder, `dash01_homing_v7.xml`, the `homing-v7` branch); those names stay. In the UI and
  the docs it is "init".
- **Home (standing)**: where 🏠 Home drives the robot (`STAND_POSE_DEG`).

The controller's *normalized* frame (`controller/fixed_gait/webui/calibration.py`) The controller's *normalized* frame (`controller/fixed_gait/webui/calibration.py`)
is anchored here, so **normalized 0° on every motor = this pose**. 🏠 Home does not come back
here: it drives to the **standing pose**, the v36 assembly pose (§4, §5).

Written for whoever (human or LLM) next touches the controller. Section 5 lists what is **not**
yet done.

## 1. Files

| file | what |
|---|---|
| `dash01_homing_v7.xml` | MJCF of the robot in the init pose (kneeling). It is the Fusion 360 export `EXPORT_assembly_homing v7` (ACDC4Robot), with 8 loop-closure sites and 4 `<equality><connect>` added (see §3). Base fixed to the world, 14 hinges, 6 DOF once the loops are closed. |
| `meshes/*.stl` | binary STL, **mm** (`scale="0.001"`), each in its body frame |
| `acdc4robot-export-report.json` | exporter report: 15 links, 14 joints, no errors |

The raw export also lives outside the repo, at `Dash-01/MuJoCo/EXPORT_assembly_homing-v7/`.

Conventions: `compiler angle="radian" eulerseq="XYZ"`. **Uppercase means extrinsic** (fixed axes, `R = Rz·Ry·Rx`).
x is forward, y is left, z is up. Body origin = `EXPORT_body-v4_1` (torso).

**The joint `range`s in the XML are wrong. Do not use them.** They were copied from the
Fusion joint limits, which are expressed in Fusion's own joint frames, not about this pose. They are
identical to the v36 export although qpos 0 moved, and they are not left/right symmetric.

## 2. Joints and the motors that drive them

| motor (controller name) | joint | child body | axis (body frame) | twin sign |
|---|---|---|---|---|
| `left.abd` (hip roll) | Revolute-3 | EXPORT_hip_left-v2_1 | +x | +1 |
| `left.cam` | Revolute-38 | EXPORT_cam_left-v3_1 | +y | +1 |
| `left.thigh` | Revolute-33 | EXPORT_thigh_left-v3_1 | +y | −1 |
| `right.abd` (hip roll) | Revolute-2 | EXPORT_hip_right-v3_1 | +x | −1 |
| `right.cam` | Revolute-30 | EXPORT_cam_right-v3_1 | −y | −1 |
| `right.thigh` | Revolute-27 | EXPORT_thigh_right-v3_1 | −y | +1 |

The passive joints are the knee (R-28 / L-34, tibia on thigh), the ankle (R-31 / L-35, foot on
tibia), the achilles (R-32 / L-36, on the thigh) and the pushrod (R-29 / L-37, on the cam). Note
that `EXPORT_foot-v4_2` is the **right** foot and `EXPORT_foot-v4_1` the left one (no side in the name).

**Map between the twin and the motors:** `mjcf_qpos_rad = radians(sign · norm_deg)`, with no offset,
because qpos 0 *is* the zero pose. The signs follow from the calibration wizard's physical convention,
which is the same on both legs:
- thigh + swings the knee forward;
- cam + moves the crank pin down;
- abd + moves the foot outward.

They are the same signs as for the previous robot (`twinmap.DEFAULT_SIGNS`, unchanged).
`tests/test_twinmap.py` re-derives them from this XML.

## 3. Loop closure

Each leg has two planar four-bars. They are chained, so the knee must be solved before the achilles:

| connect | link (site1) | target (site2) | passive pins |
|---|---|---|---|
| `loop_knee_L/R` | pushrod, `(-0.41, ±0.0043, 0)` | tibia | pushrod + knee |
| `loop_achilles_L/R` | achilles, `(-0.512, ±0.002, 0)` | foot | achilles + ankle |

The site positions were taken from the v36 closed model
(`Dash-01/MuJoCo/EXPORT_assembly_stable_closed`), where they had been fitted to the bores by
`MuJoCo/loop_closure/close_loops.py`. Bodies and frames are the same in both exports. **At qpos 0
all four pins coincide to 0.000 mm**, which shows the init CAD is a consistent closed pose.

The init pose sits close to the knee four-bar's full stretch. With the cam held, the thigh can
swing only about 4° backward (left-thigh qpos −4°) before the knee loop cannot close. The web twin
then draws the closest pose and reports the gap.

## 4. The init pose in numbers

**Normalized frame: all six motors are at 0.0°, by definition.**

For reference, here is the same pose relative to the **v36 assembly pose**, which was the
`qpos = 0` of the earlier closed export and of anything built from it. The sign convention is the
v36 model's (same axes as above).

| joint | homing relative to its parent (CAD) | v36 qpos at homing (deg) | (rad) |
|---|---|---|---|
| hip roll R / L | −25.00 / +25.00 | −25.00 / +25.00 | −0.43633 / +0.43633 |
| thigh R / L | +0.97 / −0.90 | −16.71 / +16.78 | −0.29160 / +0.29281 |
| cam R / L | +55.28 / −55.40 | +52.97 / −53.09 | +0.92456 / −0.92655 |
| knee R / L (passive) | +18.46 / +18.50 | +60.30 / +60.34 | +1.05243 / +1.05313 |
| ankle R / L (passive) | −133.34 / −133.37 | −67.34 / −67.38 | −1.17535 / −1.17594 |
| achilles R / L (passive) | −32.26 / −32.30 | −63.60 / −63.64 | −1.11006 / −1.11070 |
| pushrod R / L (passive) | −50.45 / +50.32 | +58.87 / −59.00 | +1.02744 / −1.02974 |

**In words:**
- **Hips:** both rolled 25° outward, against their stops.
- **Thighs:** about 1° from the hip frame, i.e. about 16.7° behind their v36 angle.
- **Cams:** about 53° crank-up from v36.
- **Knees:** flexed about 60° more than in v36.

Left and right agree to within 0.15°.

**The v36 assembly pose in normalized degrees** (from `norm = −sign · q_v36_at_homing`):

| side | abd | cam | thigh |
|---|---|---|---|
| left | −25.0 | +53.1 | +16.8 |
| right | −25.0 | +53.0 | +16.7 |

Both legs read the same, which is the cross-check that the signs are right.

**Map between a future RL model built in the v36 frame and the motors.** This is the
`controller/deploy/deploy_map.json` form `model_rad = radians(sign·norm + offset)`. It is derived,
not yet verified on hardware:

| motor | sign | offset_deg |
|---|---|---|
| left.abd | +1 | +25.00 |
| left.cam | +1 | −53.09 |
| left.thigh | −1 | +16.78 |
| right.abd | −1 | −25.00 |
| right.cam | −1 | +52.97 |
| right.thigh | +1 | −16.71 |

The sign is valid only if the model keeps these joint axes. **`deploy_map.json` itself was NOT
changed:** it maps the RL plant `RLframework/model/dash01_free.xml`, which is still the previous
robot (no tibia or achilles).

## 5. What the controller does with this, and what is still open

**Done (web UI, `controller/fixed_gait/webui/`):**
- **Digital twin.** `tools/build_twin.py` publishes this XML to `static/twin/` by default.
  `static/twin3d.js` reads the loops from the `<connect>` elements and solves them in dependency
  order (knee, then achilles), in closed form, on the CAD's assembly branch. It colours a leg orange
  when one of its loops cannot close. `tests/js/twin_loops.js` checks the loop solve, run through
  `tests/test_twin_loops_js.py`.
- **🏠 Home = the standing pose.** `daemon.STAND_POSE_DEG` is the v36 assembly pose in normalized
  degrees (the table "The v36 assembly pose in normalized degrees" in §4, per side):
  `abd −25.0 / −25.0, cam +53.09 / +52.97, thigh +16.78 / +16.71` (left / right). In MuJoCo
  (`Dash-01/MuJoCo/EXPORT_assembly_stable_closed`, closed loops, CAD masses, 5.36 kg without the
  motors) that pose has:
  - hips straight;
  - both soles flat and level at z = −863 mm, spanning x −115..+161 mm, centres ±190 mm;
  - the whole-robot CoM 36 mm ahead of the hip axis and 0.2 mm off centre, so over the soles.

  Home, the standing hold, the joint trims and the pose ⚖ Balance regulates around all go through
  `STAND_POSE_DEG`.
- **Tests.** The two pre-existing failures (`test_blackbox.py::test_rotation_and_budget_enforcement`
  and `test_ws_grid_js.py`) also fail without these changes. The policy tests fail independently
  (no working policy bundle).

**Open. These are still on the previous robot, so do not trust them on this one:**
1. **Balance and the stand solver.** `STAND_POSE_DEG` is now the v36 pose. But
   `tools/solve_stand_pose.py`, `tools/balance_sim.py` and the posture maps in `balance.py`
   (D_PITCH / D_COM_X / D_COM_Y) still come from `dash-01CAD/homing/SpiderBotInitPos` (the old
   legs). Do not use ⚖ Balance until they are redone on this CAD. The v36 CoM also leaves out the
   motor masses: re-check it with the weighed masses once they are in a model.
2. **Feasibility net / end-effector LUT.** `fklut.py` and `fk_lut.npz` model the old four-bar.
   Home and guided moves slew under that net (with a relax when the start pose is rejected), so
   rebuild it for the new knee and achilles loops.
3. **Joint limits.** Measure the real limits about this pose (§1: the XML ranges are wrong). Then
   update `daemon.HARD_CLAMP` and the workspace.
4. **RL plant and deploy_map.** Build the new plant from this CAD, then fill `deploy_map.json` (see
   the offsets in §4) and verify it with the robot limp.

## 6. Operating it

1. **Re-zero on the robot.** The calibration stored on the Pi was captured before this change, in
   the old reference pose. Put the robot in the init pose (kneeling, hips against their outward
   stops, the pose in §4), run the zero wizard, and confirm each sign.
2. 🏠 Home then takes the robot from kneeling to the standing pose: hips 25° inward from their stops, cams about 53°,
   thighs about 17°.
3. **Re-publishing the twin after a CAD change:**

```bash
python controller/fixed_gait/webui/tools/build_twin.py
```

Then copy `static/twin/` to the Pi.

**Pi layout (2026-10-27).** The checkout on the Pi is `~/running_robot`, and the web UI lives in
`robot/fixed_gait/webui/` (old folder name, not `controller/`). Its files use CRLF line endings, and
`static/app.js` and `static/index.html` are older than this repo's. So deploy by applying the
diff, not by copying the files over. Services: `runningrobot-webui` (control, :8081) and
`runningrobot-ui` (front end, :8080). Restart them only while the robot is LIMP.
