# Tips & tricks for the next designer

What cost us time on DASH-01, so it does not cost you the same. Read it before changing the CAD.
The full procedure is in [`pipeline/README.md`](pipeline/README.md); this page is the "why" and the traps.

## 1. Fusion → MuJoCo export (ACDC4Robot)

- **Install ACDC4Robot v1.1.1 from GitHub, not from the App Store.** The App Store version is old,
  has no preflight and crashes with raw Python tracebacks. Check the SHA-256 (pipeline README §0) and
  make sure only one copy is installed (`find ~/Library -iname "*acdc4robot*"`).
- **Design type must be Hybrid**, not Assembly (*Document Settings*). In Assembly mode the Fusion
  API breaks `allJoints` and the export fails with an `InternalValidationError` that says nothing useful.
- **Flat structure, Joints only.** Every component directly under the root, each with a solid body.
  Use Revolute *Joints*; alignment *constraints* are ignored by the exporter and the part ends up
  disconnected.
- **The exporter cannot export a closed loop.** Keep two Fusion files: the closed assembly to design
  and to *Drive Joints*, and an open copy where the closure joints (`pushrod↔tibia`, `achilles↔foot`)
  are **deleted** (Delete, not Suppress). `close_loops.py` re-closes them in MuJoCo.
- **Align the pins before deleting the closure joints.** The script finds the loops from the bores; if
  the pins are not concentric at export, it stops (`bore centres X mm apart`).
- **Enable both joint limits**, otherwise the joint comes out unlimited.
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

## 5. Working with the repo

- One person edits the Fusion file at a time: `.f3z` files cannot be merged.
- After any CAD change: export → run the pipeline → run `python -m pytest pipeline/tests` → log it in
  [`CHANGELOG.md`](CHANGELOG.md) → tell the training side which model folder is new.

## 6. Lessons from building the robot

> To be completed by the designer: printing (materials, tolerances, orientation), assembly order,
> CFRP tubes and their attachments, bearings and fits, wiring and connectors, motors, and what you
> would do differently.
