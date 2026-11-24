# Tips & tricks for the next designer

What cost us time on DASH-01, so it does not cost you the same. Read it before changing the CAD.
The full procedure is in [`pipeline/README.md`](pipeline/README.md); this page is the "why" and the traps.

## 1. Preparing the CAD for MuJoCo

This is the workflow that works. Follow it in this order.

1. **Prepare each segment as one part, on its own origin.** A segment (thigh, tibia, foot, hip, …) is
   often a sub-assembly of several printed and bought parts. In its own design:
   - merge all its parts into **one body**;
   - place it on the **origin of the design**, with the axes you want for this segment (X, Y, Z);
   - save it as an independent design (`EXPORT_<part>_<left|right>`).

   The origin of each part becomes its body frame in MuJoCo. A part that is not fixed on its own
   origin gives a body with a strange frame and wrong inertia axes.
2. **No timeline.** Turn the design history off (no timeline at the bottom of the window) in **every
   part and in the assembly**: right-click on the root component → *Do not capture Design History*
   (also in *Document Settings*). The design type must be **Hybrid**, not Assembly (*Document Settings*):
   in Assembly mode the Fusion API breaks `allJoints` and the export fails with an
   `InternalValidationError` that says nothing useful.
3. **Build the final assembly from these parts.** Insert the body, hips, cams, thighs, pushrods, tibias,
   achilles and feet, each with its correct origin. **Flat structure**: every component directly
   under the root.
4. **Revolute joints with correct limits.** Connect the parts with Revolute *Joints*, not constraints.
   For each joint, set the **min and max angles** and enable **both limits** (otherwise the joint comes
   out unlimited). Set the angle **0 in the standing pose** if possible: qpos 0 in MuJoCo is the pose the
   file is saved in.
5. **Close the loops with alignment constraints, not joints.** The robot has two closed loops per leg:
   - the **knee four-bar**: the pushrod (driven by the cam) attached to the tibia;
   - the **ankle**: the achilles attached to the foot.

   The exporter **cannot export a closed loop**. So these two attachments are **alignment
   (collinearity) constraints** between the pin axes, not joints. With them, the assembly moves like the
   real robot: drive the joints and check that the mechanism works.
6. **Before the export, remove the alignment constraints** (both sides), so that the loops are open,
   and export. `pipeline/close_loops.py` closes the loops again in MuJoCo, at the pins. The pins must
   be aligned (concentric) in the saved pose: the script finds the loops from the bores and stops if
   they are apart (`bore centres X mm apart`).
7. **Export MJCF, not URDF.** ACDC4Robot can write both; take **MJCF**. Then run the pipeline.

Export details:

- **Install ACDC4Robot v1.1.1 from GitHub, not from the App Store.** The App Store version is old,
  has no preflight and crashes with raw Python tracebacks. Check the SHA-256 (pipeline README §0) and
  make sure only one copy is installed (`find ~/Library -iname "*acdc4robot*"`).
- **ASCII names only.** The French Fusion names (`Révolution-12`) broke the previous team's scripts.
  Name components `EXPORT_<part>_<left|right>` from the start.
- Expected preflight: **15 links, 14 joints**. Anything else means a structure problem.
- **Save assemblies as `.f3z`, not `.f3d`.** An assembly that inserts parts from other files keeps only
  cloud links in an `.f3d`: off the Hub it opens empty. The Hub's folder download skips designs and the
  Fusion API writes only `.f3d`, so export assemblies by hand (*File → Export → Fusion Archive*) and
  check them with `fusion/tools/check_f3z.py`. See [`fusion/README.md`](fusion/README.md).

## 2. What the export gets wrong (and how the pipeline handles it)

- **Masses are wrong by up to 5×.** Densities come from the CAD materials: printed parts are not solid,
  motors are not modelled. The raw export is ~5.3 kg; the robot is 15.1 kg. Never train on the raw
  export: use the `_mass.xml` model ([`properties/README.md`](properties/README.md)).
- **Joint `range`s are wrong.** They are Fusion's limits in Fusion's joint frames, not around qpos 0,
  and not left/right symmetric. Do not use them; in a closed loop they act as real stops in simulation.
- **qpos 0 = whatever pose the file was saved in.** Decide the export pose on purpose (standing for
  balance work) and use `repose_init.py` for the init pose. Exporting in a random pose silently moves
  every joint zero.
- **Names are unstable.** Fusion renumbers `Revolute-N` when you edit parts (thigh: 33/27 → 43/40),
  feet have no side in their name (`foot-v2_1` is the RIGHT foot in xshape, `foot-v4_2` the right one
  in v2), and the exporter wrote `thight`. Identify parts by what they are, never by `Revolute-N`.
- **Body versions in names** (`-v3_1`) change with every edit of a part: the mapping files in
  `properties/` must be updated after each re-export (`apply_mass_properties.py` tells you which bodies
  are missing).

## 3. Masses and inertias

- **Weigh every part** you make, and the assembled segments, and write it down (mass, date, what is
  included). It is the only ground truth. The PDF of the current design is still the v2 one: the xshape
  thigh, foot and achilles are estimates.
- In Fusion *Properties*, set the real material or override the mass, and include motors and battery
  in the part they sit in. Note it in the mapping (`assembly_contents`) so nobody adds them twice.
- Save the PDF and extract it to `mass_properties.json` **as printed** (signed products of inertia);
  the script cross-checks COM and origin tensors and catches typos.
- The CoM position matters more than the inertia for standing: a few millimetres of CoM error in x is
  the difference between a policy that stands and one that tips. Check the CoM of the assembled robot
  on the real thing (balance it on an edge, or use the two-scale method) and compare with
  `checks/static_stand.py`.

## 4. Mechanism

- The ankle is **coupled to the knee** through the achilles loop, neither free nor locked.
  The xshape redesign cut |ankle/knee| from 1.72 to 0.78. Any change to the achilles pins changes the
  coupling: recompute it (pipeline README, reference results) and tell whoever trains policies.
- The init pose sits close to the knee four-bar's full stretch: only ~4° of thigh travel backward
  before the knee loop cannot close. Check the reachable range after every linkage change.
- Hip and cam frames have not changed since v2; that is what lets `repose_init.py` reuse the v7 init
  pose. If you change a motor body frame, the init pose must be redefined in Fusion.
- **Rods in compression rather than in tension.** The attachment of a rod working in tension is hard
  to make strong in 3D print; in compression it is much easier. Keep that in mind when you place a
  linkage.

## 5. Working with the repo

- One person edits the Fusion file at a time: `.f3z` files cannot be merged.
- After any CAD change: export → run the pipeline → run `python -m pytest pipeline/tests` → log it in
  [`CHANGELOG.md`](CHANGELOG.md) → tell the training side which model folder is new.

## 6. Lessons from building the robot

- **Assembly usually goes well.** The printed parts fit, with a little work.
- **Ream the holes of printed parts.** Holes come out undersized or not round: pass a reamer to get the
  right diameter and shape, especially for pins and bearings.
- **Glue the carbon / printed sandwiches.** A CFRP plate or tube held against a printed part only holds
  if it is **glued**. Screwed only, it comes loose and carries much less load.
- **Hip plates: check the holes before printing.** On the current robot the big plates that hold the
  two motors (`Hip_plate`) and the motor PCB holders do not have their holes in the right place. It
  should be corrected in the CAD (`fusion/v2-full/Hip_v44`), but this is not verified: check against
  the motors and PCBs before printing.
- **The robot does not exactly match the CAD.** Several parts are not screwed as drawn: the design
  evolved and some fixes were made on the spot. It holds for testing, but it is not very robust for a
  long final test. Check the screws and fixations before a big run.
- **Calibration plate.** The calibration plate (`fusion/calibration/`) is laser-cut MDF with pins. The
  current plate works for the xshape. For a new pose or design, redraw the holes and laser-cut a new
  MDF plate, then put the pins back: quick and cheap.

## 7. Hardware worth adding

- **Read the BMS over CAN on the Raspberry Pi.** The DALY BMS has a CAN port, unused today. Reading it
  would give the battery temperature, the state of charge (SOC) and the cell voltages to the robot
  stack and the logs.
- **An emergency stop (kill switch) on the battery KEY.** The BMS has a KEY connector: pulling KEY to 0
  cuts the battery. Wire a kill switch to it and set this behaviour in the **DALY BMS app** (the phone
  app used to configure the BMS over Bluetooth). Check the polarity and test it before relying on it.
