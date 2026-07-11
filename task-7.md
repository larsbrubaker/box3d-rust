# Task 7 — Determinism gate + large world

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

The acceptance bar for the dynamics unit: `test_determinism.c` and
`test_large_world.c`. Scene helpers and large-world tests are ported on
`agent/task-7-helpers`. The final hash comparison remains **blocked by task-6**
(the determinism scene drops ragdolls onto grid/torus mesh grounds).

`create_grid_mesh` / `create_torus_mesh` are already ported
(`src/mesh/factory.rs`). Joints, sleep, and CCD have landed, so the ragdoll
builder has everything it needs.

## Done on this branch

- [x] Port `shared/human.c` (ragdoll builder: capsule bones + joint tree,
      friction torque / hertz / damping / group index parameters) → `src/human/`
- [x] Port `shared/determinism.c` (`CreateFallingRagdolls` /
      `UpdateFallingRagdolls`: ragdoll grid over mesh grounds, sleep-step
      detection, world state hash) → `src/determinism.rs`
- [x] Hash function (`b3Hash` / `HASH_INIT`) already in `src/core.rs`;
      `hash_world_transform` hashes `WorldTransform` bytes in C field order
- [x] Port `test/test_large_world.c` (stack / bullet / query) →
      `src/large_world_tests.rs`, float + `--features double-precision`
- [x] Minimal `b3Shape_RayCast` dependency for the query subtest →
      `src/shape/accessors.rs` (landed with task-4; interim world_query.rs dropped on merge)

## Final gate (task-6 mesh_contact is on main)

Mesh narrow-phase has landed. Port the hash gate next:

- [ ] Build the C reference with CMake and `BOX3D_DISABLE_SIMD=ON`
      (scalar single-threaded path is the behavioral reference)
- [ ] Port `test/test_determinism.c` and match `EXPECTED_SLEEP_STEP` /
      `EXPECTED_HASH` for both precision modes
- [ ] On divergence: instrument both sides and diff traces — never guess
      (CLAUDE.md rule; the C build exists precisely for this)
