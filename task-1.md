# Task 1 — Joints

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

Port the joint system: lifecycle from `joint.c`, the eight per-type C files,
the joint solver stages in `solver.c`, and the joint-events report pass.

The Rust side already has: the full joint data model (`src/joint.rs`), the
graph-color functions (`assign_joint_color`, `create_joint_in_graph`,
`add_joint_to_graph`, `remove_joint_from_graph` in `src/constraint_graph.rs`),
and `merge_solver_sets` / `transfer_joint` in `src/solver_set.rs` — all marked
`#[allow(dead_code)] // bring-up:`. This track makes them reachable; drop the
allowances as that happens. `wake_solver_set` and `try_sleep_island` already
handle joint transfer, so sleeping islands connected by joints should work once
creation exists — verify with tests rather than assuming.

Touches `src/solver/solve.rs` and `src/solver/integrate.rs` — coordinate with
task-2 (CCD) and task-3 (sensors).

## Lifecycle (joint.c)

- [ ] `b3LinkJoint` / `b3UnlinkJoint` from `island.c` into `src/island.rs`
      (JointLink plumbing mirrors the contact versions)
- [ ] Joint id pool + create path (`b3CreateJointInternal`): destination set
      selection (static/awake/sleeping/disabled), island link, graph insert,
      wake-on-create, `merge_solver_sets` when joining two sleeping sets
- [ ] `collide_connected == false`: destroy existing contacts between the two
      bodies on joint create; restore filtering on destroy
- [ ] Destroy path: unlink island, remove from graph or solver set, free id
- [ ] Default defs for every joint type (`b3Default*JointDef`)

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
