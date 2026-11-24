# Legacy CAD (old versions)

> **Everything here is an old version. It is not the current robot.** The current design is the
> Fusion file in [`../../fusion/`](../../fusion/).

These are the CAD files that were in the `Dash-01/CAD/` folder of the project computer (about 640 MB).
They are kept for reference, as zip files in the GitHub Release **`legacy-cad`** of this repo, not in
git (binary files that would make every clone heavy forever).

| Zip | Original folder | Content |
| --- | --- | --- |
| `legacy-solidworks.zip` | `CAD/Full_robot_cad/CAD/` | previous team's SolidWorks project: assemblies of solid links, part drawings (PDF), catalogue parts, jigs, templates |
| `legacy-inventor.zip` | `CAD/Full_robot_cad/` (`.ipt`, `.iam`, `RunningRobot/`, `elec/`, `PartURDF/`, `Repair/`, `OldVersions/`) | Inventor version of the robot, electronics parts (battery, BMS, XT60, DC-DC), partial URDF assemblies |
| `legacy-step.zip` | `CAD/Full_robot_cad/000_Assy_Full.STEP`, `CAD/Export/` | full STEP assembly of the old robot; STEP per part of the v2 design (Fusion `EXPORT_*`) |

What they are still good for:

- **Part drawings** (`Drawings/pdf/`): dimensions of the machined parts (chassis plates, CFRP tube
  attachments, hip hinge base, adduction base), if a part has to be re-made.
- **Vendor parts** (motors AK60/AKE90, igus, bearings, connectors): geometry to place in a new design.
- The thesis of the previous team explains the design choices (four-bar knee, CFRP tubes, spring ankle).

Do not start a new design from these files: import the parts you need into the Fusion design instead.
