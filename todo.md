# TODO

**This document tracks only work that remains. Nothing listed here is done.**
As items complete, delete them; when a section or task file is finished, remove
it entirely. If this file ever describes finished work, that's a bug — clean it
up in the same commit that finishes the work.

Read `CLAUDE.md` first: the pinned C reference is `box3d-cpp-reference/`
(never upstream), ports must match C behavior exactly, and the dynamics-core
bring-up rules apply to everything below.

**Milestone note (context, not a task): the port is functionally complete and
bit-exact with the C scalar reference in both precision modes. Everything
below is quality, presentation, and release work — not core physics.**

**Demo track note (context, not a task): Samples App coverage is complete.
Benchmark DEBUG-count samples are now `live` under the Village precedent
(release scale stays disclosed in SCENE_INFO). Remaining `partial` registry
rows are intentional browser/policy disclosures documented in
`demo/src/registry.ts` (Benchmark Sensor camera gap, Hull Step trial loop,
pointer-lock, dump-loader mechanism, scaled/scoped mesh samples, Tree/Replay
browser limits, Time of Impact label). No further actionable demo-excellence
task file.**

## Upstream tracking (recurring)

Upstream Box3D moves fast (released June 2026; submodule pinned at `540ea38`).
After release readiness:

- [ ] Diff the pinned submodule against the latest upstream tag; triage new
      commits into port-worthy fixes vs features; bump the pin and re-run the
      determinism gate (expected values may change with upstream fixes)
