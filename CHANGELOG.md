# Changelog

All notable changes to box3d-rust are documented here. This project adheres to
semantic versioning (0.x: minor-compatible additive changes bump the patch number).

## 0.1.2

Additive-only public API and tooling; no behavioral changes to existing APIs.

- Ported `clone_and_transform_hull` (`hull.c:2265`) in `src/hull/create.rs`, with tests.
- Added `DynamicTree::node_views` and `DynamicTree::root_index` accessors for BVH
  inspection.
- Demo site now mirrors the complete C `samples` app (~150 samples) at the project
  homepage: https://larsbrubaker.github.io/box3d-rust/

## 0.1.1 / 0.1.0

Initial crates.io releases — see git history for details.
