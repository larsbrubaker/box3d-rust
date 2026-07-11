# Task 2 — Continuous collision / bullets (CCD)

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

Port the continuous-collision path from `solver.c`. Today
`finalize_bodies` (`src/solver/integrate.rs`) always takes the safe advance —
the doc comment says "CCD is skipped in this bring-up slice". Fast-moving
bodies tunnel through thin geometry until this lands.

Touches `src/solver/solve.rs` and `src/solver/integrate.rs` — coordinate with
task-1 (joints) and task-3 (sensors).

## Finalize fast-path (solver.c b3FinalizeBodiesTask)

- [ ] Fast/bullet detection in `finalize_bodies`: sweep distance vs
      speculative margin, `IS_FAST` flag, keep `center0`/`rotation0` as the
      sweep start instead of snapping them forward
- [ ] Non-bullet fast bodies: full CCD advance inline; bullets: append the body
      sim index to the step's bullet-body list (the C
      `stepContext->bulletBodies` array — a step-local Vec in Rust)
- [ ] Enlarge target AABB handling for fast bodies (`ENLARGE_BOUNDS` body-sim
      flag; the AABB is finished later by continuous collision)

## Continuous solve (solver.c)

- [ ] `b3ContinuousQueryCallback` (broad-phase query filter: skip self, skip
      non-colliding pairs, sensor rules, TOI query per shape)
- [ ] `b3SolveContinuous` (sweep construction, `b3TimeOfImpact` per candidate,
      advance to first TOI, update center0/rotation0, recompute AABBs,
      `sensor_hits` capture for sensor overlaps crossed by the sweep)
- [ ] Bullet pass in `solve()` after the refit/enlarge block: run
      `b3SolveContinuous` for each queued bullet, then serially enlarge bullet
      shape proxies (`b3DynamicTree_EnlargeProxy`; all bullet shapes must
      already be in the move buffer)
- [ ] Extend the existing enlarge pass with the fast-bullet branch: bullets
      buffer moves (`b3BufferMove`) instead of enlarging, because their final
      AABB isn't known until the bullet pass (C solver.c refit block)

## API + tests

- [ ] `b3Body_SetBullet` / `b3Body_IsBullet` with correct body-vs-sim flag
      sync (port `SetBulletDriftTest` from `test_world.c` — it pins a C bug
      fix where `SetMotionLocks` wiped the bullet bit)
- [ ] Honor `world.enable_continuous` toggle (`b3World_EnableContinuous`)
- [ ] Port `TestContinuousMoveEvent` from `test_world.c`
- [ ] Test: fast bullet vs thin static wall does not tunnel; same scene with
      continuous disabled does (documents the toggle)
