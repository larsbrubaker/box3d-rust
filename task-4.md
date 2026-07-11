# Task 4 — Remaining shape creates + Shape/Body API surface

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

**Done on main (do not re-port):** mesh / height-field / compound `create_*_shape`,
`TestCompoundHitEvents`, `TestOverflowColorPile`, body damping / gravity scale /
`EnableSleep` (+ flag-sync tests), `b3Body_SetMotionLocks` / bullet API.

Collision geometry is fully ported. Remaining work is the public mutator /
query surface and one mesh-drop world test.

## Shape creation leftovers

- [ ] Port `TestMeshDrop` from `test_world.c` (mesh shape create is ready;
      exercises continuous collision / mesh contact stability until sleep)

## b3Shape_* API (shape.c)

- [ ] Filter get/set (with proxy + contact refresh on change)
- [ ] Material get/set (base + per-index), friction/restitution/rolling
- [ ] Enable flags: sensor events, contact events, hit events, pre-solve
- [ ] Geometry get/set (SetSphere/SetCapsule/SetHull/… with proxy rebuild)
- [ ] Ray cast / point test / closest point against a single shape
- [ ] AABB getters, user data, density (with mass update)

## b3Body_* API (body.c)

- [ ] `SetType` (dynamic/kinematic/static transitions — main consumer of
      `transfer_body` in `src/solver_set.rs`; drop its
      `#[allow(dead_code)] // bring-up:` note when reachable)
- [ ] `Enable` / `Disable` (disabled-set transfers, proxy destroy/create)
- [ ] `SetTransform` (teleport with contact refresh), `SetAwake`
- [ ] Forces/impulses: ApplyForce/Torque/LinearImpulse/AngularImpulse
      (center + point variants, wake semantics)
- [ ] Motion locks **get** (`b3Body_GetMotionLocks`) — set already landed
- [ ] Name get/set (NameCache), user data, world getters (velocity at point,
      local/world point and vector transforms)

## Tests

- [ ] Port the remainder of `test_body.c` alongside the API slices
- [ ] Port the remainder of `test_shape.c`
