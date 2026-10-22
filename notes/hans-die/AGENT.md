# Context for hans-die's agent

Module: localizacion + navegacion

The root `CLAUDE.md` already loads the rules, the interfaces and the
workflow. This module's decisions are in `notes/hans-die/NOTES.md`.

If a change affects what the module receives or delivers, don't make it:
propose it as an interface change.

## Notes for the agent

- Only propose what is needed for the task. No optional setup or extra checks that
  CI or the simulation server already cover (e.g. no local Docker builds).
- Before describing the repo or starting work: `git fetch` and bring the task
  branch up to date with `origin/development`. Teammates merge often.
- Language for this module (localizacion + navegacion): code, comments, logs and
  commits in Spanish (hans-die's decision, 2026-10-22). Nav2 parameter and node
  names stay as Nav2 defines them.
