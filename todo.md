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

## Parallel tracks

| File | Track | Depends on |
|---|---|---|
| [task-12.md](task-12.md) | Demo excellence: interaction layer (mouse drag, pause/step), visual quality (shadows, colorization), sample-category coverage, site polish | — |

The C samples app has ~90 samples with mouse-drag, pause/single-step, tuning
panels, and debug overlays; our 17 demos play like a viewer, not a playground.
task-12 is the whole gap, ordered so the shared interaction layer lands first.

## Benchmarks

- [ ] Port `benchmark/` scenes as criterion benches (informs whether the
      pooled manifold allocator or SIMD ever become worth it — both stay out
      until benches justify them and bit-exactness is preserved)

## Release readiness

The crate is functionally complete. `0.1.0` is published on crates.io
(https://crates.io/crates/box3d-rust) and tagged `v0.1.0`. Remaining polish:

- [ ] Decide the idiomatic-API question: ship the C-mirror API as-is for 0.1
      (like box2d-rust) or add a thin ergonomic layer — record the decision

## Upstream tracking (recurring)

Upstream Box3D moves fast (released June 2026; submodule pinned at `540ea38`).
After release readiness:

- [ ] Diff the pinned submodule against the latest upstream tag; triage new
      commits into port-worthy fixes vs features; bump the pin and re-run the
      determinism gate (expected values may change with upstream fixes)
