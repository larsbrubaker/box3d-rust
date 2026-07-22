# Changelog

All notable changes to box3d-rust are documented here. This project adheres to
semantic versioning (0.x: minor-compatible additive changes bump the patch number).

## 0.2.1

No public API or behavioral changes; existing code upgrades without modification.

- Added unit tests for the mesh creation path (degenerate-triangle filtering and
  vertex weld / identify-edges via `create_mesh`).
- Release-profile tuning (fat LTO, single codegen unit) for in-repo builds.
- Demo site: zoom-to-cursor camera navigation, InstancedMesh sample coverage,
  `M` diagnostics drawer with full debug-draw flags, and Mesh Viewer /
  benchmark fidelity fixes matching the C reference.

## 0.2.0

### Breaking

- `CustomFilterFcn` is now `fn(&World, ShapeId, ShapeId, u64) -> bool` (previously
  `fn(ShapeId, ShapeId, u64) -> bool`), matching C's `b3CustomFilterFcn`, whose
  callback can read shape state mid-step.
  **Migration:** add `&World` as the first parameter of your filter callback. The
  world reference gives callbacks the same shape-state access C callbacks have,
  e.g. `shape_get_user_data(world, id)`.

### Added

- `shape_set_mesh` (`b3Shape_SetMesh` port) to replace the mesh geometry of an
  existing mesh shape.
- `set_stall_threshold` / `get_stall_threshold` (`b3Set/GetStallThreshold` port).
- Tests for the above.

### Demos

- All 20 sample categories now render at full fidelity or honestly-disclosed
  partial coverage. Camera right-drag fix, plus arrow-key aliases for navigation.

## 0.1.2

Additive-only public API and tooling; no behavioral changes to existing APIs.

- Ported `clone_and_transform_hull` (`hull.c:2265`) in `src/hull/create.rs`, with tests.
- Added `DynamicTree::node_views` and `DynamicTree::root_index` accessors for BVH
  inspection.
- Demo site now mirrors the complete C `samples` app (~150 samples) at the project
  homepage: https://larsbrubaker.github.io/box3d-rust/

## 0.1.1 / 0.1.0

Initial crates.io releases — see git history for details.
