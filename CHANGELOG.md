# Changelog

All notable changes to box3d-rust are documented here. This project adheres to
semantic versioning (0.x: minor-compatible additive changes bump the patch number).

## Unreleased

Upstream sync: the C reference pin advances to `3fc20f5` (upstream "Fixes 09"
and "Follow cam"), and this port follows its API and behavior.

### Breaking

- Sensor visitors must now be convex (upstream `b3IsConvex` check); mesh,
  height-field, and compound shapes no longer generate sensor overlaps as
  visitors.
- `MAX_SHAPE_CAST_POINTS` now equals `MAX_HULL_VERTICES` (128, previously 64).
- Recording format minor version advances to 4. Older recordings still
  replay (the loader gates on major version only), but recordings that use
  the new shape-mutator ops are not readable by 0.3.0.

### Changed

- Simplified face-vs-edge contact selection in the hull-capsule and
  triangle-capsule manifolds, matching upstream.

### Fixed

- Replayed-mesh material-index deserialization read the wrong element count
  (one index per triangle is correct); replayed mesh contacts previously
  panicked.

### Added

- `ShapeSetMeshMaterial`, `ShapeSetHull`, and `ShapeSetMesh` are now recorded
  and replayed.
- New tests: overlap hull proxy, transformed box hull, capsule face-deep
  manifolds, and geometry mutator replay.
- New "Class Ring" demo scene in the Bodies category.

## 0.3.0

Upstream sync: the C reference pin advances to `c52908c` and this port follows
its API and behavior. Simulation results are not bit-compatible with 0.2.x.

### Breaking

- `create_compound_shape` is now `create_baked_compound_shape` (upstream
  baked-compound rename, box3d #77).
- `collide_sphere_and_triangle` / `collide_capsule_and_triangle` /
  `collide_hull_and_triangle` are now `collide_triangle_and_sphere` /
  `collide_triangle_and_capsule` / `collide_triangle_and_hull`, matching the
  upstream argument-order flip.
- `BODY_NAME_LENGTH` / `SHAPE_NAME_LENGTH` (18) are replaced by
  `MAX_NAME_LENGTH` (256) and `MAX_QUERY_NAME_LENGTH` (64).
- `edge_edge_separation` was removed by the upstream edge-edge optimization
  port (box3d #63).
- Recording/snapshot formats advanced (`REC_VERSION` 4.3, `SNAP_VERSION` 2,
  new `HULL_VERSION`); recordings from 0.2.x do not replay.

### Changed

- Upstream behavior ports: ghost-collision improvements (#61), edge-edge
  optimization (#63), friction center weighted average (#71), SIMD hull
  collision scalar path (#93), contact and manifold fixes (#94), hull
  stacking (#97) and hull builder (#98) fixes.

### Added

- `DebugShapeCallbacks` test port, with a replay debug-shape release fix.
- Demo parity batches syncing the sample app to `c52908c` (new samples,
  Jenga rework, determinism scenes split into modules).
- 30 new tests; suite green in both precision modes (332 / 333 with
  `double-precision`).

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
