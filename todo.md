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
Benchmark DEBUG-count samples (including Sensor and Hull) are now `live`
under the Village precedent (release scale stays disclosed in SCENE_INFO).
Remaining `partial` registry rows are intentional browser/policy disclosures
documented in `demo/src/registry.ts` — only these:

1. Character / Rigid Body — pointer-lock on canvas click (browser)
2. Collision / Time of Impact — intentional label correction
3. Issues / Dump Loader — no dump loader API; hand-ported defs
4. Mesh / Height Field — scaled for browser render
5. Mesh / Creation Benchmark — performance.now vs b3GetTicks
6. Tree / Benchmark — fetch/performance.now/portable save format
7. Replay / Viewer — query search index + keyframe-policy skipped

No further actionable demo-excellence task file for completable fidelity
gaps.**

## Upstream tracking (recurring)

Upstream Box3D moves fast (released June 2026; submodule pinned at `540ea38`).
After release readiness:

- [ ] Diff the pinned submodule against the latest upstream tag; triage new
      commits into port-worthy fixes vs features; bump the pin and re-run the
      determinism gate (expected values may change with upstream fixes)
