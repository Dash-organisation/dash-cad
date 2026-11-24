# CLAUDE.md — dash-cad

DASH-01: a 6-DoF biped. This repo holds the reference CAD (Fusion 360), the MuJoCo models derived from it, the
mass properties and the Fusion → MJCF pipeline. Human docs: `README.md`, `TIPS.md`, `pipeline/README.md`.
Write code, docs and commit messages in English.

## Layout

- `fusion/` — Fusion archives: `xshape/` (current design, `.f3z` + STEP), `v2/`, `v2-full/`, `tools/` (export script, `check_f3z.py`).
  **The source of truth**; everything else is derived from it. All other designs: `fusion-archive` GitHub Release.
- `models/<design>-<pose>-f<N>/` — MuJoCo models (`v2-standing-f34`, `v2-standing-f36`, `v2-init-f7`,
  `xshape-standing-f8`, `xshape-init-f8`). Name table and old names: `models/README.md`. Current design: `xshape-*-f8`.
- `properties/` — `inerties_masses.pdf` (Fusion), `mass_properties.json` (its transcription), `body_mapping_*.json`.
- `pipeline/` — `close_loops.py` → `apply_mass_properties.py` → `repose_init.py`, then `checks/`. Tests in `pipeline/tests/`.
- `archive/legacy-cad/` — README only; the old SolidWorks/Inventor CAD is in the `legacy-cad` GitHub Release. Old versions, never a starting point.
- `CHANGELOG.md` — one entry per design / Fusion version.

## Rules

- **Never hand-edit generated models** (`*_closed.xml`, `*_mass.xml`, `*_mass_audit.json`, homing xml). Change the
  input (raw export, `loops.json`, `mass_properties.json`, mapping) and rerun the pipeline step.
- Raw ACDC4Robot exports (`EXPORT_assembly-v<N>.xml`) are kept as exported: do not edit them either.
- Identify parts by what they are (thigh, cam, foot, …), **never by `Revolute-N`** (Fusion renumbers them). Feet
  carry no side in their name: the side comes from the body's y. The exporter's `thight` typo is expected.
- The deployed training plant in `dash-sim2real` (`BalanceRL/model/build_three_standing.py` → `stand_xshape.xml`)
  reads these models. Any change to `models/xshape-*` or `properties/` changes training: say so in the commit
  message and the CHANGELOG, and tell the user to warn the training side (Léo-Paul).
- New design or re-export: new model folder `<design>-<pose>-f<N>`, update `models/README.md` and `CHANGELOG.md`;
  do not overwrite an existing folder.
- Mass properties: `mass_properties.json` is the PDF **as printed** (signed products of inertia). Do not "fix"
  values in it; estimated parts are `scale_cad` entries in the mapping, documented in `properties/README.md`.
- File sizes: GitHub warns above 50 MB and refuses above 100 MB. Bigger files go in a GitHub Release, not in git.

## Workflow

- Work on a branch: `cad/<topic>` (design / export), `pipeline/<topic>` (scripts), `docs/<topic>`. Never commit
  to `main` directly; open a pull request.
- Commit messages: `area: what changes` (e.g. `models: add xshape-standing-f9`).
- Ask before `git push`; never force-push.
- Other repos are read-only references, cloned next to this one: `../dash-sim2real` (robot stack, training),
  `../dash-reports` (reports, videos). Do not edit, switch branches or commit in them from here.

## Tests

```bash
pip install -r requirements.txt
python -m pytest pipeline/tests -q      # ~2 s: loop closure, mass step, re-pose, static check
```
All tests must pass. Run them after any change to `pipeline/`, `models/` or `properties/`.

## Saving tokens

- Do not open binaries: `**/*.stl`, `**/*.pdf`, `fusion/**/*.{f3z,f3d,step}`; check an archive with `fusion/tools/check_f3z.py`. Masses are in `properties/mass_properties.json`.
- Models are ~16 KB of XML: read the bodies or joints you need (`grep -n '<body\|<joint'`), not the whole file.
- `*_mass_audit.json`: use `jq` or a Python one-liner.

## graphify

If `graphify-out/graph.json` exists (`graphify update .`, AST only, not committed), run `graphify query "<question>"`
before grepping for a codebase question and `graphify section "<heading>"` for Markdown sections. If `graphify` is
not installed, skip this.
