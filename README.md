# dash-cad

CAD of the **DASH-01** biped and everything needed to turn it into a MuJoCo model that can be
simulated and trained on: the Fusion 360 design, the MuJoCo exports, and the scripts of the
Fusion → MJCF pipeline (loop closure, real masses and inertias, init pose, checks).

Training and the robot stack live in [`dash-sim2real`](https://github.com/Dash-organisation/dash-sim2real);
reports and videos in [`dash-reports`](https://github.com/Dash-organisation/dash-reports).

> **Next designer: read [`TIPS.md`](TIPS.md) before you touch Fusion.** It lists what cost us days.

## Repository layout

| Folder | Contents |
| --- | --- |
| [`fusion/`](fusion/) | **The CAD**: Fusion 360 archives (`.f3z`) and STEP of the current design (`xshape/`), the v2 export and the full v2 robot. All other designs in the `fusion-archive` release. Start here. |
| [`models/`](models/) | MuJoCo models exported from Fusion, one folder per design × pose. Name table in [`models/README.md`](models/README.md). |
| [`pipeline/`](pipeline/) | Fusion → MJCF scripts, in order, with the full procedure in [`pipeline/README.md`](pipeline/README.md). |
| [`properties/`](properties/) | Fusion mass properties (PDF + extracted JSON) and the body ↔ part mappings. |
| [`archive/legacy-cad/`](archive/legacy-cad/) | Pointer to the **old** CAD of the previous teams (SolidWorks, Inventor). Not the current robot. |
| [`TIPS.md`](TIPS.md) | Tips & tricks for the next designer. |
| [`CHANGELOG.md`](CHANGELOG.md) | Design history: what changed between versions and why. |

## The current robot in one table

| | |
| --- | --- |
| Design | **xshape** (achilles relocated so it crosses the leg), Fusion file version 8, November 2026 |
| Mass | 15.14 kg weighed; 14.54 kg in the model (Fusion properties + 1 kg allowance) |
| Actuated DOF | 6: per leg hip roll (AK60-39), cam and thigh (AKE90-8) |
| Passive mechanism | per leg, two closed four-bar loops: knee (cam → pushrod → tibia) and ankle (thigh → achilles → foot) |
| Model with real masses, standing | [`models/xshape-standing-f8/EXPORT_assembly-v8_closed_mass.xml`](models/xshape-standing-f8/) |
| Model with real masses, init pose | [`models/xshape-init-f8/dash01_homing_v8_mass.xml`](models/xshape-init-f8/) |
| Used by | `dash-sim2real/BalanceRL/model/stand_xshape.xml` (deployed plant) and the web UI twin (`dash01_homing_v8.xml`) |

## From Fusion to a usable model (summary)

```
Fusion 360 (Hybrid design, flat, Revolute joints, closure joints deleted)
   │  ACDC4Robot v1.1.1 → MJCF                        (manual, see pipeline/README.md §1–3)
   ▼
<export>.xml + meshes/ + acdc4robot-export-report.json        tree, 14 joints, CAD masses (~5.3 kg)
   │  1. pipeline/close_loops.py
   ▼
<export>_closed.xml                                           4 loops closed, 6 DOF
   │  2. pipeline/apply_mass_properties.py --mapping properties/body_mapping_<design>.json
   ▼
<export>_closed_mass.xml                                      real masses / inertias (~14.5 kg)
   │  3. pipeline/repose_init.py   (qpos 0 = init pose, for the controller and the twin)
   ▼
dash01_homing_<v>_mass.xml
   │  4. pipeline/checks/   static_stand · stability · sim_stand
   ▼
dash-sim2real builds the training plant from it (free base, actuators, sole contacts, PD)
```

Setup, once:

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python -m pytest pipeline/tests -q      # the pipeline reproduces the committed models
```

Tested with MuJoCo 4.0.1 (the exports were first validated with 3.15–3.17; results are identical).

## What is where on the original computer

For the record, the files in this repo come from these folders. **Everything in the `CAD/` folder is
an old version** (previous teams' SolidWorks and Inventor files, early STEP exports): the current
design is only the Fusion file in `fusion/`.

| Original folder | Went to | Note |
| --- | --- | --- |
| `Dash-01/CAD/Full_robot_cad/` | GitHub Release `legacy-cad` (zip) | **old versions**, see [`archive/legacy-cad/`](archive/legacy-cad/) |
| `Dash-01/CAD/Export/` (STEP per part) | GitHub Release `legacy-cad` (zip) | **old versions** (v2 parts) |
| Fusion Hub project `Dash-01` (export of 2026-11-24, local copy `Dash-01/Fusion_export_2026-11-24/`) | `fusion/` (assemblies) + Release `fusion-archive` (everything) | see [`fusion/README.md`](fusion/README.md) |
| `Dash-01/MuJoCo/EXPORT_*` | `models/` (renamed) | see the name table |
| `Dash-01/MuJoCo/loop_closure/` | `pipeline/close_loops.py`, `loops.json` | translated to English |
| `Dash-01/MuJoCo/EXPORT_assembly-xshape/*.py` | `pipeline/checks/` | read the masses from the model instead of a hard-coded table |
| `dash-sim2real` / `DASH-01_leo`: `prepare_cad_v2.py`, `audit_standing03_models.py` | `pipeline/apply_mass_properties.py` | one script, mapping in JSON |
| `dash-sim2real/dash-01CAD/homing/dash01_homing_v8/tools/derive_init.py` | `pipeline/repose_init.py` | joints found by part name |
| `Dash_2print/` | not kept | print files are regenerated from Fusion |

## License

[MPL-2.0](LICENSE), like the other Dash-organisation repos.
