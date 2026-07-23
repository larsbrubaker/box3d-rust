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
3. Mesh / Height Field — scaled for browser render
4. Mesh / Creation Benchmark — performance.now vs b3GetTicks
5. Tree / Benchmark — fetch/performance.now/portable save format
6. Replay / Viewer — query search index + keyframe-policy skipped

About content lives on the home page. Separate `#/math` and `#/roadmap`
About routes are deliberately not added (and not removed if ever present):
home About covers product context without inventing extra SPA pages.

No further actionable demo-excellence task file for completable fidelity
gaps.**

## Upstream tracking (recurring)

Upstream Box3D moves fast (released June 2026; submodule pinned at `c52908c`,
synced 2026-07-23 — upstream main has nothing newer; an unmerged `fixes_09`
branch is in progress upstream and should be triaged once it lands on main).

- [ ] Periodically diff the pinned submodule against upstream main; triage new
      commits into port-worthy fixes vs features; bump the pin and re-run the
      determinism gate (expected values may change with upstream fixes)

## Demo parity with the c52908c samples (from the 2026-07 sync)

The core library and test sync to `c52908c` is complete. The samples app moved
too; the demo site still mirrors the `540ea38` samples. Port the user-visible
sample changes into `demo/`:

- [ ] New samples with no library helper: Benchmark Convex Pile (PEEL LCG seed
      42), Stacking Edge Crossing, Bodies Gyroscopic Precession (now enabled
      upstream), Issues: Restitution Overshoot, Slide Twist Off Center, GMod
      Wheel Stack (317-vert data), s&box Ghost Collisions (~400 LOC procedural
      mesh)
- [ ] Update `demo/src/registry.ts` rows accordingly; renderer/shader work in
      `samples/gfx` (e.g. shadow PCF) is optional visual polish. Note:
      `samples/mover.cpp` is only a refactor extracting the already-ported
      character mover into shared code — no new demo sample needed.

## Test-parity gap (pre-existing, found during the 2026-07 sync)

- [ ] `test_recording.c` `RecTestDrawShape` — the C test covering
      `b3World_Draw`'s lazy `createDebugShape` path through recording was never
      ported; add its Rust counterpart to the recording tests
