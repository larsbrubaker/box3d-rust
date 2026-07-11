# TODO

**This document tracks only work that remains. Nothing listed here is done.**
As items complete, delete them; when a section or task file is finished, remove
it entirely. If this file ever describes finished work, that's a bug — clean it
up in the same commit that finishes the work.

Read `CLAUDE.md` first: the pinned C reference is `box3d-cpp-reference/`
(never upstream), ports must match C behavior exactly, and the dynamics-core
bring-up rules apply to everything below.

## Parallel tracks

| File | Track | Depends on |
|---|---|---|
| [task-5.md](task-5.md) | Deferred world/body query tests | task-4 (`SetHull`, body-level queries) — shape SetHull landed |
| [task-7.md](task-7.md) | Determinism gate | helpers + large_world done; final `EXPECTED_HASH` unblocked (task-6 on main) |

task-4 and task-6 are complete. task-7 scene helpers and large-world tests are
on main; only the EXPECTED_HASH / sleep-step gate remains.

## Recording, replay, and snapshots

Start after task-4 (op capture spans the public API surface).

- [ ] Port `world_snapshot.c` (serialize/deserialize world state)
- [ ] Port `recording.c` + `recording_ops.inl` (op capture)
- [ ] Port `recording_replay.c` (deterministic replay)
- [ ] Port `test/test_recording.c`

## Benchmarks

After the determinism gate passes (perf work before correctness is wasted).

- [ ] Port `benchmark/` scenes as criterion benches (informs whether the
      pooled manifold allocator or SIMD ever become worth it)

## Demo site samples

Mirror the C `samples/` categories (WebGL, `demo/`, `bun run build`). The
physics for all of these exists now except the mesh scenes:

- [ ] Joint samples (hinge chain, ragdoll — human.c ported)
- [ ] Sensor sample
- [ ] Bullet/CCD sample
- [ ] Query/raycast visualizer
- [ ] Character mover playground
- [ ] Mesh/height-field terrain scene
