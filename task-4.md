# Task 4 — Remaining b3Shape_* API + shape/body test remainders

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

What's left of the public shape API in `shape.c` and the last unported tests
from `test_shape.c` / `test_body.c`. (The geometry-level ray-cast suite from
`test_shape.c` is already ported under `src/geometry_tests/` — don't redo it.
Mesh narrow-phase moved to task-6.)

Unblocks the deferred task-5 tests (`SetHull` for `TestHullDatabase`,
body-level queries for `test_body_query.c`).

## b3Shape_* API (shape.c)

- [ ] Enable flags get/set: sensor events, contact events, hit events,
      pre-solve events (per-shape bits with contact flag refresh)
- [ ] Geometry get/set: `SetSphere` / `SetCapsule` / `SetHull` / getters,
      with proxy rebuild and contact recreation on change
- [ ] Shape-level queries: ray cast, point test, closest point against a
      single shape (world-space wrappers over the ported geometry kernels)
- [ ] AABB getters, user data get/set, density set (with body mass update),
      shape name get/set (NameCache)

## Tests

- [ ] Port `PointInShapeTest`, `RayCastShapeTest`, `ShapeNameTest` /
      `CheckShapeName`, `ShapeFlagsTest` from `test_shape.c`
- [ ] Port `BodyTest` (the generic coverage test) from `test_body.c`
