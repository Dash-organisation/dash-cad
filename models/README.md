# MuJoCo models

One folder per **design × pose**, named `<design>-<pose>-f<Fusion file version>`:

- **design**: `v2` = first articulated-foot design with the original achilles (October 2026);
  `xshape` = achilles relocated so it crosses the leg (November 2026).
- **pose**: `standing` = exported standing (qpos 0 = standing); `init` = qpos 0 is the kneeling
  init pose where the motor zeros are captured.
- **f<N>**: version number of the Fusion file the export came from (`EXPORT_assembly v34` → `f34`).

Inside each folder, files keep the name the exporter gave them, so the journals and older scripts
that cite them can still be followed.

| Folder | Old name (where it lived) | What it is | Used by |
| --- | --- | --- | --- |
| [`v2-standing-f34/`](v2-standing-f34/) | `dash-01CAD-v2/EXPORT_assembly_closed` (`EXPORT_assembly-v34`) | v2 design, standing. First closed-loop export. | GAIT series (`RLframework/model/dash01_cad_v2_free.xml`) |
| [`v2-standing-f36/`](v2-standing-f36/) | `MuJoCo/EXPORT_assembly_stable_closed` (`EXPORT_assembly-v36`) | v2 design, the "stable" standing pose | WALK-01 pose reference; `STAND_POSE_DEG` of the web UI (🏠 Home) |
| [`v2-init-f7/`](v2-init-f7/) | `MuJoCo/EXPORT_assembly_homing-v7`, `dash-01CAD/homing/dash01_homing_v7` | v2 design in the init pose (separate Fusion file `EXPORT_assembly_homing`, v7) | reference init pose for `repose_init.py`; previous web twin |
| [`xshape-standing-f8/`](xshape-standing-f8/) | `MuJoCo/EXPORT_assembly-xshape` (`EXPORT_assembly-v8`) | **current design**, standing | **`BalanceRL/model/stand_xshape.xml`, the deployed plant** |
| [`xshape-init-f8/`](xshape-init-f8/) | `dash-sim2real/dash-01CAD/homing/dash01_homing_v8` | current design re-posed to the init pose (derived, not an export) | web UI twin (`dash01_homing_v8.xml`) |

## Files in a folder

| File | Produced by | Content |
| --- | --- | --- |
| `EXPORT_assembly-vN.xml` | ACDC4Robot | raw export: a tree, 14 hinges, CAD masses. Only kept where we have it (f7, f8). |
| `acdc4robot-export-report.json` | ACDC4Robot | exporter report: 15 links, 14 joints, no errors |
| `meshes/*.stl` | ACDC4Robot | binary STL in **mm** (`scale="0.001"`), each in its body frame |
| `*_closed.xml` | `pipeline/close_loops.py` | + 8 sites and 4 `<connect>`: knee and achilles loops closed, 6 DOF |
| `*_closed_closures.json` | `pipeline/close_loops.py` | closure points, bore radii, offsets, test results |
| `*_mass.xml` | `pipeline/apply_mass_properties.py` | + real masses / COMs / inertia tensors |
| `*_mass_audit.json` | `pipeline/apply_mass_properties.py` | per-body before/after, method, checks, input hashes |
| `dash01_homing_v*.xml` | `pipeline/repose_init.py` (v8) | same model, qpos 0 = init pose; `thight` renamed `thigh` |
| `README.md` (f7, init-f8) | by hand | joint ↔ motor map, signs, init pose in numbers. Written when these lived in the robot repo: paths in them refer to the old layout. |

Never edit a generated file by hand: re-run the pipeline step instead.

## Conventions (all models)

- `compiler angle="radian" eulerseq="XYZ"`: uppercase = extrinsic (fixed axes, `R = Rz·Ry·Rx`).
- World: **x forward, y left, z up**. Left leg at y > 0. Root body = `EXPORT_body-v4_1` (torso), fixed to the world.
- **Joint `range`s are wrong. Do not use them.** They are copied from Fusion's joint limits, expressed in
  Fusion's joint frames, not around qpos 0; they are not left/right symmetric.
- **Foot names carry no side.** v2: `EXPORT_foot-v4_1` = left, `EXPORT_foot-v4_2` = right.
  xshape: `EXPORT_foot-v2_1` = **right**, `EXPORT_foot-v2_2` = left. Find the side from y, never from the name.
- xshape: the exporter spells the thigh `thight` (`EXPORT_thight_left-v2_1`). `repose_init.py` renames it.
- Joint names are Fusion's `Revolute-N` and **Fusion renumbers them** when parts change (the thigh was
  Revolute-33/27 in v7, Revolute-43/40 in v8). Identify joints by the body they move.

## Joint ↔ motor map (xshape-f8)

| Motor (controller) | Joint | Body | Side sign in the twin |
| --- | --- | --- | --- |
| `left.abd` (hip roll) | Revolute-3 | `EXPORT_hip_left-v2_1` | +1 |
| `left.cam` | Revolute-38 | `EXPORT_cam_left-v3_1` | +1 |
| `left.thigh` | Revolute-43 | `EXPORT_thight_left-v2_1` | −1 |
| `right.abd` (hip roll) | Revolute-2 | `EXPORT_hip_right-v3_1` | −1 |
| `right.cam` | Revolute-30 | `EXPORT_cam_right-v3_1` | −1 |
| `right.thigh` | Revolute-40 | `EXPORT_thight_right-v4_1` | +1 |

Passive: knee L-47 / R-41, ankle L-45 / R-39, achilles L-46 / R-42, pushrod L-37 / R-29.
v2 map: [`v2-init-f7/README.md`](v2-init-f7/README.md) §2.
