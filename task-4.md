# Task 4 — Remaining shape creates + Shape/Body API surface

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

Two related gaps: `create_*_shape` only exists for sphere/capsule/hull
(`src/shape/lifecycle.rs`), and the `b3Shape_*` (~50 fns) / `b3Body_*`
(~78 fns, ~13 ported in `src/body/api.rs`) public API surfaces are mostly
unported. The collision geometry itself (mesh, height field, compound) is
fully ported and tested — only the shape-attach and API layers are missing.

Mostly independent of the solver-touching tracks; small overlap with task-3
on sensor-related shape API.

## Shape creation (shape.c)

- [x] `create_mesh_shape` (mesh data + per-instance scale, multi-material)
- [x] `create_height_field_shape`
- [x] `create_compound_shape`
- [x] Port `TestCompoundHitEvents` from `test_world.c` — the compound branch
      of `Shape::get_shape_user_material_id` is already ported and waiting;
      this test makes it reachable
- [ ] Port `TestMeshDrop` from `test_world.c` (need mesh shape create;
      exercise continuous collision / mesh contact stability)
- [x] Port `TestOverflowColorPile` from `test_world.c`
      (exercise the overflow color path)

## b3Shape_* API (shape.c)

- [ ] Filter get/set (with proxy + contact refresh on change)
- [ ] Material get/set (base + per-index), friction/restitution/rolling
- [ ] Enable flags: sensor events, contact events, hit events, pre-solve
- [ ] Geometry get/set (SetSphere/SetCapsule/SetHull/… with proxy rebuild)
- [ ] Ray cast / point test / closest point against a single shape
- [ ] AABB getters, user data, density (with mass update)

## b3Body_* API (body.c)

- [ ] `SetType` (dynamic/kinematic/static transitions — this is the main
      consumer of `transfer_body` in `src/solver_set.rs`; drop its
      `#[allow(dead_code)] // bring-up:` note when reachable)
- [ ] `Enable` / `Disable` (disabled-set transfers, proxy destroy/create)
- [ ] `SetTransform` (teleport with contact refresh), `SetAwake`
- [ ] Forces/impulses: ApplyForce/Torque/LinearImpulse/AngularImpulse
      (center + point variants, wake semantics)
- [ ] Damping, gravity scale, sleep threshold, `EnableSleep` get/set —
      port `EnableSleepFlagSyncTest` and `EnableSleepNoopUnlockTest`
      (the no-op unlock regression) from `test_world.c`
- [ ] Motion locks get/set (`b3Body_SetMotionLocks`)
- [ ] Name get/set (NameCache), user data, world getters (velocity at point,
      local/world point and vector transforms)

## Tests

- [ ] Port the remainder of `test_body.c` alongside the API slices
- [ ] Port the remainder of `test_shape.c`
