# TODO

**This document tracks only work that remains. Nothing listed here is done.**
As items complete, delete them; when a section or task file is finished, remove
it entirely. If this file ever describes finished work, that's a bug — clean it
up in the same commit that finishes the work.

Read `CLAUDE.md` first: the pinned C reference is `box3d-cpp-reference/`
(never upstream), ports must match C behavior exactly, and the dynamics-core
bring-up rules apply to everything below.

## Parallel tracks

These tracks are largely independent and can proceed on separate machines.
Each has its own file; delete the file when the track is done and remove its
row here.

| File | Track | Depends on |
|---|---|---|
| [task-1.md](task-1.md) | Joints (lifecycle, 8 joint types, solver stages, events) | — |
| [task-2.md](task-2.md) | Continuous collision / bullets (CCD) | — |
| [task-3.md](task-3.md) | Sensors (overlap sweep, events, hit reporting) | — |
| [task-4.md](task-4.md) | Remaining shape creates + b3Shape_*/b3Body_* API surface | — |
| [task-5.md](task-5.md) | World queries, casts, explosion, world API surface | — |
| [task-6.md](task-6.md) | Character mover | task-5 (cast/overlap machinery) |

Merge-conflict warning: task-1, task-2, and task-3 all add passes to
`src/solver/solve.rs` (joint stages, bullet pass, sensor-hits report) and
task-1/task-2 both touch `src/solver/integrate.rs`. Land whichever finishes
first and rebase the others.

## Determinism gate (after the parallel tracks)

The acceptance bar for the whole dynamics unit. Needs joints (task-1) and
sleep-step parity; other tracks affect the hash only if their features are used
by the test scene.

- [ ] Build the C reference with CMake and `BOX3D_DISABLE_SIMD=ON` (scalar path
      is the behavioral reference; single-threaded run order)
- [ ] Port `test/test_determinism.c` (falling-stack hash + sleep step)
- [ ] Match `EXPECTED_SLEEP_STEP` and `EXPECTED_HASH` against the C build;
      on divergence, instrument both sides and diff traces — never guess
- [ ] Port `test/test_large_world.c` and run it under
      `--features double-precision` (both configurations must pass)

## Recording, replay, and snapshots

Big surface; needs most of the public API from tasks 1–5 to exist first.

- [ ] Port `world_snapshot.c` (serialize/deserialize world state)
- [ ] Port `recording.c` + `recording_ops.inl` (op capture)
- [ ] Port `recording_replay.c` (deterministic replay)
- [ ] Port `test/test_recording.c`

## Demo site samples

Mirror the C `samples/` categories as features land (WebGL, `demo/`,
`bun run build`). Add a sample when its physics exists:

- [ ] Joint samples (hinge chain, ragdoll-style) — after task-1
- [ ] Sensor samples — after task-3
- [ ] Bullet/CCD samples — after task-2
- [ ] Query/raycast visualizer — after task-5
- [ ] Character mover playground — after task-6
