# box3d-rust

A pure Rust port of [Box3D](https://github.com/erincatto/box3d), Erin Catto's 3D physics
engine for games — exact behavioral match, including cross-platform deterministic math.

[![crates.io](https://img.shields.io/crates/v/box3d-rust.svg)](https://crates.io/crates/box3d-rust)
[![docs.rs](https://docs.rs/box3d-rust/badge.svg)](https://docs.rs/box3d-rust)
[![License](https://img.shields.io/badge/License-MIT-lightblue.svg)](LICENSE)
[![Live Demo](https://img.shields.io/badge/Live_Demo-Interactive-blue)](https://larsbrubaker.github.io/box3d-rust/)

## Demo Site

**[Follow the port live in your browser](https://larsbrubaker.github.io/box3d-rust/)**

[![box3d-rust demo](readme_hero.jpg)](https://larsbrubaker.github.io/box3d-rust/)

The demo site mirrors the upstream `samples` app via WebAssembly — same light SPA shell as
our finished [box2d-rust](https://github.com/larsbrubaker/box2d-rust) demos. Collision-layer
demos are live now (deterministic math, geometry queries, contact manifolds, hull, height
field, mesh, dynamic tree). Full rigid-body simulation samples arrive once world/body/solver
land.

> Part of the [rust-apps](https://github.com/larsbrubaker/rust-apps) suite — a collection of
> Rust graphics and geometry libraries by Lars Brubaker.

## Status: Foundation math landed

Box3D was released by Erin Catto in June 2026. The pinned reference source lives in the
`box3d-cpp-reference/` submodule (v0.1.0+, `540ea38`), and this port follows the same
playbook that took [box2d-rust](https://github.com/larsbrubaker/box2d-rust) to completion:
whole modules in dependency order, each landing with its portion of the upstream C test
suite.

| Area | Ported | Tests |
|---|---|---|
| Foundation: math_functions (Vec3/Quat/Matrix3), core/constants | ✅ | ✅ (test_math.c) |
| Foundation: id, bitset, id_pool, table (container → Vec) | ✅ | ✅ (test_id/bitset/table.c) |
| Collision: aabb, distance (GJK/TOI) | ✅ | ✅ (test_collision AABB + test_distance.c) |
| Collision: hull, geometry, manifolds, mesh, height_field, compound, dynamic_tree | ✅ | ✅ (test_hull/shape/compound/height_field + authored) |
| Broad phase: proxy ops, move buffer (pair update deferred to world) | ✅ | ✅ (authored proxy tests) |
| Dynamics: body/shape/contact lifecycles, constraint graph, solver sets, islands | ⬜ | ⬜ (test_body.c) |
| Joints: distance, motor, prismatic, revolute, spherical, weld, wheel | ⬜ | ⬜ (test_joint.c) |
| Solver: contact solver + step pipeline, sensors, sleeping, continuous | ⬜ | ⬜ (test_world.c) |
| World API: queries, casts, character movers | ⬜ | ⬜ (test_body_query/mover.c) |
| Determinism: hand-rolled trig, bit-exact vs the C build | ⬜ | ⬜ (test_determinism.c) |
| Snapshots and recording/replay | ⬜ | ⬜ (test_recording.c) |
| Large world mode (`double-precision` feature = `BOX3D_DOUBLE_PRECISION`) | ⬜ | ⬜ (test_large_world.c) |

Not ported (by design): the task scheduler/worker threads (the port is serial), SIMD
codepaths (the scalar `BOX3D_DISABLE_SIMD` path is the behavioral reference), and the C
arena/block allocators (Rust `Vec`s).

## Porting principles

- **Exact behavioral match** — same algorithms, same `f32` arithmetic, same edge cases as the C
  source. Floating-point operations are never reordered or "improved".
- **Determinism preserved** — Box3D's hand-rolled `b3Atan2` and `b3ComputeCosSin` (built for
  cross-platform determinism) are ported bit-for-bit, never replaced with std functions.
- **Tests ported too** — every module lands together with its portion of the C test suite.
- **No stubs** — no `todo!()`, no placeholders; modules are ported whole, in dependency order.
- **Large world mode** — the `double-precision` cargo feature mirrors `BOX3D_DOUBLE_PRECISION`.

The approach follows
[HOW_WE_PORTED_CLIPPER2.md](https://github.com/larsbrubaker/clipper2-rust/blob/main/HOW_WE_PORTED_CLIPPER2.md)
and the completed [box2d-rust](https://github.com/larsbrubaker/box2d-rust) port.

## Development

```bash
# Clone with the C reference submodule
git clone --recurse-submodules https://github.com/larsbrubaker/box3d-rust.git

# Run tests (both precision modes)
cargo test
cargo test --features double-precision

# Full pre-commit gauntlet: file lengths, tests, fmt, clippy, build
./scripts/pre-commit-check.ps1   # or .sh
```

### Demo site

The demo site (`demo/`) runs the port in the browser via WebAssembly.

The quickest way to see it — builds the wasm and serves the demo at
`http://localhost:3000`, opening your browser:

```
run_demo.cmd      # Windows (double-click or run from a terminal)
./run_demo.sh     # Linux / macOS
```

Or drive the steps yourself:

```bash
cd demo
bun install
bun run build:wasm   # wasm-pack build (once, and after Rust changes)
bun run dev          # dev server at http://localhost:3000, rebuilds wasm on Rust edits
```

Deployed automatically to GitHub Pages on push to `main`.

## Acknowledgments

- [Erin Catto](https://github.com/erincatto) — Box3D author
- [MatterHackers](https://www.matterhackers.com/) — sponsoring the port
- Ported with [Claude Code](https://claude.com/claude-code)
