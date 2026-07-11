# Task 7 — Determinism gate + large world

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

The acceptance bar for the dynamics unit: `test_determinism.c` and
`test_large_world.c`. The final hash comparison is **blocked by task-6**
(the determinism scene drops ragdolls onto grid/torus mesh grounds), but the
scene helpers and the large-world test are portable now and can proceed in
parallel with the mesh work.

`create_grid_mesh` / `create_torus_mesh` are already ported
(`src/mesh/factory.rs`). Joints, sleep, and CCD have landed, so the ragdoll
builder has everything it needs.

## Scene helpers (can start now)

- [ ] Port `shared/human.c` (ragdoll builder: capsule bones + joint tree,
      friction torque / hertz / damping / group index parameters)
- [ ] Port `shared/determinism.c` (`CreateFallingRagdolls` /
      `UpdateFallingRagdolls`: ragdoll grid over mesh grounds, sleep-step
      detection, world state hash)
- [ ] Port the hash function exactly (byte-for-byte over the same fields in
      the same order as C, or the hashes can never match)

## Large world (can start now)

- [ ] Port `test/test_large_world.c` (stack settle at origin vs far origin;
      sleeps on the same step in the same relative configuration)
- [ ] Run under both configurations — the float build and
      `--features double-precision` have different expected sleep steps

## Final gate (needs task-6)

- [ ] Build the C reference with CMake and `BOX3D_DISABLE_SIMD=ON`
      (scalar single-threaded path is the behavioral reference)
- [ ] Port `test/test_determinism.c` and match `EXPECTED_SLEEP_STEP` /
      `EXPECTED_HASH` for both precision modes
- [ ] On divergence: instrument both sides and diff traces — never guess
      (CLAUDE.md rule; the C build exists precisely for this)
