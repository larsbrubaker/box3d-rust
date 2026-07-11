# Task 4 — Remaining shape creates + Shape/Body API surface

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

**Done on main (do not re-port):** mesh / height-field / compound `create_*_shape`,
`TestCompoundHitEvents`, `TestOverflowColorPile`, body damping / gravity scale /
`EnableSleep` (+ flag-sync tests), motion locks set/get, bullet API, name /
user data, local/world point/vector + point velocity, ApplyForce/Torque/
Impulse variants, `SetTransform`, `SetAwake` / `IsAwake` / `IsEnabled` /
`GetType`, `SetType` / `Enable` / `Disable`, shape filter get/set (proxy +
contact refresh), material get/set (base + per-index friction/restitution/rolling).

Collision geometry and body/shape filter+material mutators are ported. Remaining:
shape enable/geometry/query APIs, mesh narrow-phase (needed by TestMeshDrop),
and test remainders.

## Mesh narrow-phase (blocks TestMeshDrop)

- [ ] Port `mesh_contact.c` (mesh/height narrow-phase in `update_contact`;
      contacts currently clear manifolds — bodies fall through meshes)
- [ ] Port `TestMeshDrop` from `test_world.c` (needs mesh_contact; exercises
      mesh contact stability until sleep)

## b3Shape_* API (shape.c)

- [ ] Enable flags: sensor events, contact events, hit events, pre-solve
- [ ] Geometry get/set (SetSphere/SetCapsule/SetHull/… with proxy rebuild)
- [ ] Ray cast / point test / closest point against a single shape
- [ ] AABB getters, user data, density (with mass update)

## Tests

- [ ] Port the remainder of `test_body.c` alongside the API slices
- [ ] Port the remainder of `test_shape.c`
