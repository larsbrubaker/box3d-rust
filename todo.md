# TODO

**This document tracks only work that remains. Nothing listed here is done.**
As items complete, delete them; when a section or task file is finished, remove
it entirely. If this file ever describes finished work, that's a bug — clean it
up in the same commit that finishes the work.

Read `CLAUDE.md` first: the pinned C reference is `box3d-cpp-reference/`
(never upstream), ports must match C behavior exactly, and the dynamics-core
bring-up rules apply to everything below.

**Milestone note (context, not a task): the determinism gate passed — the
falling-ragdoll scene matches the C scalar reference bit-for-bit in both
precision modes. Everything below is completeness, not core physics.**

Tasks 8–11 (API completeness, recording/replay/snapshots, debug draw, and
demo samples) are done. Remaining work is polish and release readiness.

## Benchmarks

- [ ] Port `benchmark/` scenes as criterion benches (informs whether the
      pooled manifold allocator or SIMD ever become worth it — both stay out
      until benches justify them and bit-exactness is preserved)

## Release readiness

The crate is functionally complete. Polish for 0.1:

- [ ] Rustdoc pass over the public API (crate-level docs, module docs on the
      main entry points, doc examples for World/body/shape/joint creation)
- [ ] README: quick-start example, docs.rs badge, feature-flag docs
      (`double-precision`)
- [ ] Cargo.toml metadata for crates.io (description, keywords, categories,
      license files, repository)
- [ ] Decide the idiomatic-API question: ship the C-mirror API as-is for 0.1
      (like box2d-rust) or add a thin ergonomic layer — record the decision
- [ ] Tag v0.1.0 aligned with the pinned C reference version

## Upstream tracking (recurring)

Upstream Box3D moves fast (released June 2026; submodule pinned at `540ea38`).
After release readiness:

- [ ] Diff the pinned submodule against the latest upstream tag; triage new
      commits into port-worthy fixes vs features; bump the pin and re-run the
      determinism gate (expected values may change with upstream fixes)
