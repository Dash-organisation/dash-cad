# AGENTS.md — dash-cad

Instructions for AI coding assistants (Codex, Cursor, Copilot, ChatGPT, …). Claude Code reads `CLAUDE.md`; the
rules there apply to every assistant, so read it first.

## The essentials (full version in CLAUDE.md)

- `fusion/` is the source of truth; `models/` are derived. Never hand-edit generated models (`*_closed.xml`,
  `*_mass.xml`, audits, homing xml) or raw exports: change the inputs and rerun `pipeline/`.
- Identify parts by name (thigh, cam, foot), never by `Revolute-N`. Feet have no side in their name.
- Changes to `models/xshape-*` or `properties/` change the training plant in `dash-sim2real`: log them in
  `CHANGELOG.md` and say so.
- Work on a branch (`cad/<topic>`, `pipeline/<topic>`, `docs/<topic>`), never directly on `main`. Commit messages:
  `area: what changes`, in English. Ask before `git push`; never force-push.
- Run `python -m pytest pipeline/tests -q` after any change; all tests must pass.
