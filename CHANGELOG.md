# Design history

Newest first. Fusion version = version number of the Fusion document the export came from.

## xshape, Fusion v8 (2026-11-04): current design

- **Achilles relocated**: it now crosses the leg. Pins on the thigh at (−24.5, ·, −290) mm in the thigh
  frame (v2: (0, ·, −415)); on the foot at (24, ·, −70) mm. Pin distance 512 → 492 mm.
- New parts: thigh (`thight_*`), foot (`foot-v2_*`, new frame), achilles. Body, hips, cams, pushrods,
  tibias unchanged (identical geometry and frames). Knee pivot on the thigh unchanged (350.00 mm).
- Ankle/knee coupling |0.78| instead of |1.72|.
- Fusion renumbered the thigh joints (Revolute-43 / 40). Motor axes and signs unchanged.
- Models: [`models/xshape-standing-f8/`](models/xshape-standing-f8/), [`models/xshape-init-f8/`](models/xshape-init-f8/).
- Masses: 9 parts from the v2 PDF, 6 redesigned parts estimated (`scale_cad`). 14.537 kg.
- Used by the deployed standing plant `stand_xshape.xml` (STAND-03/04).

## v2, init pose, Fusion `EXPORT_assembly_homing` v7 (2026-10-27)

- Same design as v36, saved in the **init pose** (kneeling, knees ~60° more bent than standing, hips
  against their outward stops). Defines the motor zeros (normalized 0° on every motor).
- Model: [`models/v2-init-f7/`](models/v2-init-f7/).

## v2, Fusion v36 (2026-10-14): "stable" standing pose

- Same parts as v34, re-posed to a standing pose with flat soles and the CoM over them.
  This pose is the web UI's 🏠 Home (`STAND_POSE_DEG`) and the WALK-01 reference.
- Model: [`models/v2-standing-f36/`](models/v2-standing-f36/).

## v2, Fusion v34 (2026-10-12): first articulated-foot design

- Rebuild of the robot in Fusion: articulated foot driven by an achilles loop (new), knee four-bar.
  First export with ACDC4Robot and first automatic loop closure (`close_loops.py`).
- Mass properties PDF made for this version (2026-10-14): 13.594 kg + 1 kg allowance.
- Model: [`models/v2-standing-f34/`](models/v2-standing-f34/). Used by the GAIT series.

## Before: previous teams

- Earlier DASH-01 generation, in SolidWorks and Inventor (Morand and Aebi master theses): four-bar knee,
  CFRP tube legs, no articulated foot.
  Its MuJoCo model (`dash-01CAD/dash01.xml`, `NCS` meshes) stays in `dash-sim2real` for the old
  experiments. CAD in the `legacy-cad` release: see [`archive/legacy-cad/`](archive/legacy-cad/).
