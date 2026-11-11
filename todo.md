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

**Demo track note (context, not a task): Samples App coverage is complete
except for the registry gap tracked below. Benchmark DEBUG-count samples
(including Sensor and Hull) are now `live` under the Village precedent
(release scale stays disclosed in SCENE_INFO).
Remaining `partial` registry rows are intentional browser/policy disclosures
documented in `demo/src/registry.ts` — only these:

1. Character / Rigid Body — pointer-lock on canvas click (browser)
2. Character / Dynamic Mover — pointer-lock on canvas click / T (browser) and
   C debug-build counts (pyramid 20, release-only skittles skipped)
3. Collision / Time of Impact — intentional label correction
4. Mesh / Height Field — scaled for browser render
5. Mesh / Creation Benchmark — performance.now vs b3GetTicks
6. Tree / Benchmark — fetch/performance.now/portable save format
7. Replay / Viewer — query search index + keyframe-policy skipped

About content lives on the home page. Separate `#/math` and `#/roadmap`
About routes are deliberately not added (and not removed if ever present):
home About covers product context without inventing extra SPA pages.

No further actionable demo-excellence task file for completable fidelity
gaps.**

## Demo registry gap

- [ ] Add registry rows (at least `planned`) for the 23 active upstream
      RegisterSample calls (all present by `efca0c1`) that the registry never
      listed, and move Shapes / Restitution to the Restitution
      category it went to at `79fc559`. The list is in the
      `demo/src/registry.ts` header.

## Upstream tracking (recurring)

Upstream Box3D moves fast (released June 2026; submodule pinned at `38f2ae1`;
the sync to upstream `38f2ae1` is ported, one upstream commit at a time, and
awaits a release).

- [ ] Periodically diff the pinned submodule against upstream main; triage new
      commits into port-worthy fixes vs features; bump the pin and re-run the
      determinism gate (expected values may change with upstream fixes)

## Demo parity with the 3fc20f5 samples (from the 2026-08 sync)

All `3fc20f5` sample changes are now mirrored in `demo/` (the new samples —
Bodies Gyroscopic Precession, Bodies Class Ring, Issues GMod Wheel Stack,
Issues s&box Ghost Collisions — are live and the registry rows are updated).
The only remaining item is optional visual polish, not a fidelity gap:

- [ ] Optional: port the `samples/gfx` renderer/shader work (e.g. shadow PCF).
      Purely cosmetic — no sample scene, value, or behavior depends on it.
