# Task 1 — Joints

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

Port the joint system: the eight per-type C files, the joint solver stages in
`solver.c`, and the joint-events report pass.

Lifecycle is done: `link_joint` / `unlink_joint`, shared `create_joint`,
`destroy_joint` / `destroy_joint_internal`, `create_filter_joint`,
`joint_set_collide_connected` (Box3D C destroys contacts only here — not on
create), default defs for every joint type, and body-destroy tears down
attached joints.

Touches `src/solver/solve.rs` and `src/solver/integrate.rs` — coordinate with
task-2 (CCD) and task-3 (sensors).

## Per-type ports (one C file per commit)

Each needs: def → sim payload mapping, prepare, warm start, solve, and the
type-specific public getters/setters (`b3DistanceJoint_*` etc.).

- [ ] `distance_joint.c`
- [ ] `motor_joint.c`
- [ ] `parallel_joint.c`
- [ ] `prismatic_joint.c`
- [ ] `revolute_joint.c`
- [ ] `spherical_joint.c`
- [ ] `weld_joint.c`
- [ ] `wheel_joint.c`

## Solver integration (solver.c)

- [ ] `b3PrepareJointsTask` / `b3WarmStartJointsTask` / `b3SolveJointsTask`
      dispatch (serial: loop the graph colors' `joint_sims`)
- [ ] Wire joint stages into `solve()`: prepare before contacts, warm start +
      solve inside each sub-step (joint blocks come before contact blocks per
      color), joints participate in relax and restitution stages
- [ ] Force/torque threshold checks set bits in
      `task_context.joint_state_bit_set` (sized per step like C)
- [ ] Joint events report pass in `solve()` (the C block between finalize and
      hit events) — builds `world.joint_events` from the bit set

## Public API + tests

- [ ] `b3Joint_*` generic API (user data, wake, forces, local frames, ids)
- [ ] Port `test/test_joint.c` module by module alongside the types
- [ ] Test: two sleeping islands joined by a new joint merge/wake correctly
      (exercises `merge_solver_sets` and the sleeping-joint transfer paths)
