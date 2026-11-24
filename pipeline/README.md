# Pipeline: from Fusion 360 to a usable MuJoCo model

Steps 0–3 below are done by hand in Fusion; steps 4–7 are the scripts of this folder, run in order.
Every script checks its own output and **writes nothing if a check fails** (exit code 1).

```
Fusion 360 ──(ACDC4Robot)──> export ──1 close_loops──> _closed ──2 apply_mass_properties──> _closed_mass
                                                                                                │
                              dash-sim2real (training plant) <── 4 checks <── 3 repose_init ────┘
```

| Step | Script | Input → output |
| --- | --- | --- |
| 1 | [`close_loops.py`](close_loops.py) + [`loops.json`](loops.json) | raw export → `<name>_closed.xml` (+ `_closures.json`) |
| 2 | [`apply_mass_properties.py`](apply_mass_properties.py) | `_closed.xml` + `properties/` → `_closed_mass.xml` (+ `_audit.json`) |
| 3 | [`repose_init.py`](repose_init.py) | reference init model + `_closed[_mass].xml` → `dash01_homing_<v>[_mass].xml` |
| 4 | [`checks/static_stand.py`](checks/static_stand.py) | CoM in the support polygon, quasi-static motor torques |
| 4 | [`checks/stability.py`](checks/stability.py) | gravity Hessian → minimum motor stiffness `kp_crit` |
| 4 | [`checks/sim_stand.py`](checks/sim_stand.py) | free-standing simulation with PD on the 6 motors |
| – | [`tests/`](tests/) | regression: the scripts reproduce the committed models |

The training plant itself (free base, actuators, sole contact patches, PD gains, noise) is built in
`dash-sim2real` (`BalanceRL/model/build_three_standing.py`, `RLframework/model/make_cad_v2_plant.py`)
from the `_closed_mass` model of this repo.

Quick run on a new export `models/<design>-standing-f<N>/EXPORT_assembly-v<N>.xml`:

```bash
python pipeline/close_loops.py models/<folder>/EXPORT_assembly-v<N>.xml
python pipeline/apply_mass_properties.py models/<folder>/EXPORT_assembly-v<N>_closed.xml \
       --mapping properties/body_mapping_<design>.json
python pipeline/repose_init.py models/v2-init-f7/dash01_homing_v7.xml \
       models/<folder>/EXPORT_assembly-v<N>_closed_mass.xml -o models/<design>-init-f<N>/dash01_homing_v<N>_mass.xml
python pipeline/checks/static_stand.py models/<folder>/EXPORT_assembly-v<N>_closed_mass.xml
python pipeline/checks/stability.py    models/<folder>/EXPORT_assembly-v<N>_closed_mass.xml
```

---

## 0. Install ACDC4Robot (once)

- Use **v1.1.1 from GitHub Releases** (`ACDC4Robot-1.1.1.zip`), **not** the App Store package.
  The App Store package (`ACDC4Robot.bundle`) is an old version without preflight: it crashes with
  raw tracebacks instead of usable messages.
- Check the archive: `shasum -a 256 ACDC4Robot-1.1.1.zip` must print
  `cdc15e626f019cd43ed2347fc57a066967ddef7a843d1946645cf952f6d06823`.
- Copy the `ACDC4Robot/` folder (one level only) into
  `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns/`.
- Delete any older copy, found with `find ~/Library -iname "*acdc4robot*"`. Only **one** entry
  must remain in *Shift+S → Add-Ins*.

## 1. Prepare the assembly in Fusion

| # | Rule | Symptom if forgotten |
|---|---|---|
| 1 | Design type **Hybrid** (*Document Settings → pencil*). With type *Assembly*, the Fusion API breaks `allJoints`. | `RuntimeError: 2 : InternalValidationError : jo…` |
| 2 | **Flat** structure: every occurrence directly under the root, each with a BRep solid | preflight: *nested occurrence* / *mesh-only* |
| 3 | Parts linked by Revolute **Joints**, **not** by alignment constraints (the add-in ignores constraints) | preflight: *disconnected link* |
| 4 | Base (`EXPORT_body`) grounded, and no joint to Fusion's root/ground | preflight: *grounded endpoint* |
| 5 | Closure joints **deleted** (Delete, not Suppress): `pushrod↔tibia` and `achilles↔foot`, both sides | preflight: *has more than one parent* |
| 6 | **Pins aligned** at the export pose: the bores of the rod and of the attachment part coincide | `close_loops.py`: *bore centres X mm apart* |
| 7 | Joint limits: enable **both** bounds (min and max), otherwise the joint comes out without `range` | unlimited joint in MuJoCo |
| 8 | Names in `[A-Za-z0-9_-]` only (no "Révolution", no spaces) | renamed or failed export |

For rule 5, keep **two Fusion files**: the **closed** assembly for design work and *Drive Joints*,
and an **open** copy for the export. Before deleting the closure joints in the copy, check that the
pins are aligned (rule 6).

Expected preflight result: **15 links, 14 joints**, no error. The warning *Referenced components…*
is harmless.

## 2. Export

*UTILITIES → Add-Ins → ACDC4Robot*, format **MJCF**, target **MuJoCo**, **empty** output folder.
The add-in writes `<name>.xml`, `meshes/*.stl` (mm, `scale="0.001"`) and
`acdc4robot-export-report.json` there. Put that folder in `models/` under its new name
(`<design>-<pose>-f<N>`, see [`../models/README.md`](../models/README.md)).

## 3. Mass properties in Fusion

For step 2 you need the real mass properties of each part: in Fusion, right-click the component →
*Properties*, for every part, with the **real** material or a mass override (printed parts are
not solid; motors and battery must be included in the part they sit in). Save all of them as one
PDF in `properties/`, then extract them to `mass_properties.json` (format in
[`../properties/README.md`](../properties/README.md)). Do this whenever a part changes.

---

## Step 1: close the loops (`close_loops.py`)

```bash
python pipeline/close_loops.py <export_folder | file.xml> [-o out.xml] [-c loops.json] [--dry-run]
```

| Option | Effect |
|---|---|
| *(none)* | writes `<name>_closed.xml` and `<name>_closed_closures.json` next to the export |
| `-o out.xml` | other output file (keep it in the export folder, the STL paths are relative) |
| `-c config.json` | other configuration than `loops.json` |
| `--dry-run` | detect and test, write nothing |

The original export is never modified. An already closed file is refused.

**How the closure points are found.** For each rod (regex `link` in `loops.json`):

1. **Axis**: the bore axis is the axis of the rod's parent hinge, in the world frame at qpos 0.
2. **Distal bore**: the vertices of the end farthest from the parent joint are projected on the plane
   normal to the axis. A RANSAC finds a circle of radius `bore_radius_m` there, refined by least squares.
3. **Attachment body**: the `target` body with a bore of the same radius **at the same place**, searched
   in the plane and within `axial_window_m` along the axis so the opposite leg is never matched.
   The match is geometric: side-less names such as `foot-v4_1` / `foot-v4_2` are handled.
4. **Point**: bore centre, at half the rod thickness.
5. **Writing**: one `<site>` on each part, then `<connect name="loop_<loop>_<L|R>" site1 site2>`
   with the `solref`/`solimp` of `loops.json`. Side is `L` when y > 0.

Why the bore centre and not the farthest vertex (the previous team's `geometry.loop_anchors()`):
on the v34 export, that vertex is about 9 mm off the pin axis, which skews the lever arm of the loop.

**Tests.**

| Test | Threshold (`loops.json`) |
|---|---|
| Bore found on both parts, radius ≈ `bore_radius_m` | `radius_tol_m` 0.4 mm (rod), `target_radius_tol_m` 0.6 mm |
| Centre offset rod / attachment part | `max_bore_offset_m` 0.5 mm |
| Number of rods per loop | `count` = 2 |
| Same side, and each attachment body used once | — |
| `connect` violation at qpos 0 | `tol_qpos0_m` 0.01 mm |
| 1 s under gravity, fixed base, no actuator: no NaN, max violation | `tol_sim_m` 1 mm |
| Remaining DOF (nv − rank of the constraint Jacobian) | `expected_dof` 6 |

The dynamic threshold is 1 mm because the constraint is **soft** by design (`solref` 5 ms, from the
previous team's model). In fast motion, both legs in phase opposition, it reaches about 1.2 mm.

**Configuration (`loops.json`).** Any `defaults` key can be overridden in one loop, e.g.
`"bore_radius_m": 0.003` for a Ø6 pin. Adding a loop = adding an entry to `loops`, no code change.
`expected_dof` is 6 for the fixed-base export (12 with a free base); `null` disables the test.

**Check it visually.**

```bash
python -m mujoco.viewer --mjcf=<name>_closed.xml
```

- Let the simulation **run** and push a body (double-click it, then Ctrl + right-click and drag).
  The knee must follow the cam, the foot must follow the knee, pushrod and achilles stay attached.
- When paused, the *Joint* sliders set angles **without solving the loops**: the rods look detached. That is normal.
- **Robot Viewer** (viewer.robotsfan.com) does not handle parallel mechanisms. Use it only on the raw export.

**Troubleshooting.**

| Message | Probable cause | Action |
|---|---|---|
| `no bore of r~4.00 mm` | pin of another diameter, or simplified mesh | adjust `bore_radius_m` |
| `bore centres X mm apart` | export made with the loop not aligned | realign in the closed file (*Drive Joints*), copy again, delete the closure joints, re-export |
| `N rod(s) found, 2 expected` | component renamed in Fusion | fix the `link` regex or `count` |
| `different sides` / `used by two rods` | ambiguous match | check `axial_window_m` and the geometry |
| `already contains 'loop_*' connects` | closed file given as input | start from the raw export |
| `violation_sim` fails | constraint too soft | stiffen `solref`, e.g. `0.002 1` |
| `ParseXML: Error opening file` | wrong path, or output outside the export folder | give the export folder, keep `-o` inside it |

**Reference results.**

| | v2-standing-f34 (2026-10-12) | xshape-standing-f8 (2026-11-04) |
|---|---|---|
| Bore radius pushrod / tibia | 4.000 / 4.002 mm | 4.000 / 4.002 mm |
| Bore radius achilles / foot | 4.000 / 4.096–4.101 mm | 4.000 / 4.100 mm |
| Centre offset knee / achilles | 0.000 / 0.126 mm | 0.000 / 0.116 mm |
| Pin distance pushrod / achilles | 410.0 / 512.0 mm | 410.0 / 492.0 mm |
| Violation at qpos 0 / after 1 s | 0.0000 / 0.217 mm | 0.0000 / 0.040 mm |

First-order coupling around qpos 0 (thigh and hip roll held), xshape: knee/cam −1.47, ankle/cam
±1.15, |ankle/knee| ≈ 0.78 (v2: 1.72, the ankle is about 2.2× less driven by the knee). The ankle is
**neither free nor locked**: it is coupled to the knee. Cam sweep ±0.6 rad: no singularity.

## Step 2: real masses and inertias (`apply_mass_properties.py`)

```bash
python pipeline/apply_mass_properties.py <closed.xml> --mapping properties/body_mapping_<design>.json [--report]
```

Writes `<name>_mass.xml` and `<name>_mass_audit.json`. The CAD densities are wrong (printed parts
are not solid, motors are not modelled), so the raw export weighs about 5.3 kg against 15.1 kg for
the real robot. The script rewrites each body's `<inertial>` from `properties/mass_properties.json`,
according to the mapping:

- `"method": "pdf"`: the Fusion values of the part, in the export body frame (optional
  `rotation_x_deg` / `translation_mm` if the PDF frame differs).
- `"method": "scale_cad"`: the part changed after the PDF was made. Its new CAD COM and tensor shape
  are kept, scaled by the density factor of its previous version (`reference_cad_mass_kg`). This is
  an **estimate**: re-measure the part and switch it to `pdf`.
- `extra_body_mass`: an allowance on the torso (1 kg: cables, screws, electronics not in the PDF).

Checks: PDF hash, body list, parallel-axis consistency of the PDF tensors, physical tensors, COM inside
the mesh bounding box, compiled model equal to the request, loops closed, zero-gravity smoke test.

Reproduces exactly (relative error < 1e-9) the inertials of `dash-sim2real/BalanceRL/model/stand_xshape.xml`
(xshape) and of the v2 corrected model (`dash01_v2_corrected.xml`).

## Step 3: init pose (`repose_init.py`)

```bash
python pipeline/repose_init.py models/v2-init-f7/dash01_homing_v7.xml <closed[_mass].xml> -o <out.xml>
```

Fusion exports in the pose the file was saved in (standing). The controller and the web twin need
**qpos 0 = the init pose** (kneeling, hips against their outward stops, where the motor zeros are
captured). The script copies the hip / cam / thigh orientations of the reference init model, drives
the new model there in 120 steps with the loops solved at each step (to stay on the CAD assembly
branch), then rewrites the body `pos`/`euler` so that pose is qpos 0. Joints, sites and inertials are
untouched. It stops if a motor body frame changed between the two exports (then the init pose must
be defined again, by hand, in Fusion).

## Step 4: checks

Run them on the `_mass.xml` model.

- `static_stand.py`: CoM vs support polygon (margin, tip angle, push needed to tip) and quasi-static
  motor torques over the possible CoP positions, against the motor limits.
- `stability.py`: gravity Hessian on the closed-loop, feet-welded manifold, in motor coordinates.
  `kp_crit` = minimum equal stiffness on the 6 motors for the pose to be statically stable (lower bound).
- `sim_stand.py`: robot on the floor, PD on the 6 motors at their qpos 0 angle, for t seconds.

Current results for `xshape-standing-f8` (14.537 kg):

| Check | Result |
|---|---|
| CoM | 0.771 m above the soles, 9.8 cm min margin, tip angle 7.2°, 18 N at CoM height to tip it |
| Static torques | |cam| ≤ 37 N m, |thigh| ≤ 20 N m, |hip roll| 3.5 N m: below the continuous limits |
| `kp_crit` | **329 N m/rad**; unstable mode = cam + thigh, antisymmetric between legs |
| `sim_stand` | **falls backward** at kp 200 and at kp 500 (open issue, see below) |

**Open issue, `sim_stand`.** With the 6 motors at kp 500, the cams still yield about 2.6°, which the
four-bar turns into about 6.5° of knee, and the robot tips backward in about 1 s. With all 14 joints
held it stands, so the passive mechanism is the weak point, not the contact. The original script
(`MuJoCo/EXPORT_assembly-xshape/sim_stand_xshape.py`) behaves the same, and `kp_crit` (329) says
kp 200 cannot hold. Not investigated further yet (soft loop `solref`, passive joint damping and
armature, PD on the cam vs the knee are the first suspects). Treat `sim_stand` as a diagnostic, not a
verdict: the trained plant in `dash-sim2real` has its own contacts and gains.
