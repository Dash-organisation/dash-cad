# Fusion 360: the current design

**This folder is the reference CAD of DASH-01.** Everything else in the repo (MuJoCo models, masses,
print files) is derived from it.

> Files pending: the current Fusion export (`.f3z`) and the STEP of the assembly will be added here.

| File | Content |
| --- | --- |
| `DASH-01_<version>.f3z` | Fusion archive of the full assembly with all referenced components (*File → Export → Fusion Archive*). Open it with *File → Open → Open from my computer*. |
| `DASH-01_<version>.step` | STEP of the assembly, readable without Fusion (FreeCAD, SolidWorks, Onshape) |

## Rules for this folder

- **One current design.** When the design changes, replace the files and log the change in
  [`../CHANGELOG.md`](../CHANGELOG.md) (date, Fusion version, what changed, why). Git keeps the old ones.
- Use the Fusion **version number** in the file name and in the MuJoCo folder names (`f<N>`).
- Keep two Fusion documents: the **closed** assembly (design work, *Drive Joints*) and an **open**
  copy for the ACDC4Robot export (closure joints deleted). See [`../pipeline/README.md`](../pipeline/README.md) §1.
- Fusion archives are binary: git cannot diff or merge them. One person edits the CAD at a time.
- Files over 50 MB: GitHub warns; over 100 MB it refuses. Put bigger files in a GitHub Release.
