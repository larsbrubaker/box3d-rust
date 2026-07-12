# Task 8 — API completeness: introspection, kinematic targets, wind

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

The last unported public API functions, found by diffing `box3d.h` against the
Rust surface (everything else — 360 functions — is matched). Two are real
features (kinematic target transforms, wind); the rest are introspection
accessors that round out embedder ergonomics and unblock the final
`test_world.c` / `test_body.c` coverage tests.

## Features

- [ ] `b3Body_SetTargetTransform` (kinematic bodies: derive velocities that
      reach the target over one step — body.c)
- [ ] `b3Shape_ApplyWind` (wind drag force applied through a shape — shape.c)

## Body introspection (body.c)

- [ ] `b3Body_GetShapeCount` / `b3Body_GetShapes`
- [ ] `b3Body_GetJointCount` / `b3Body_GetJoints`
- [ ] `b3Body_GetContactCapacity` / `b3Body_GetContactData`
- [ ] `b3Body_ComputeAABB`

## Shape introspection (shape.c)

- [ ] `b3Shape_GetContactCapacity` / `b3Shape_GetContactData`
- [ ] `b3Shape_GetSensorCapacity` / `b3Shape_GetSensorData` /
      `b3Shape_GetSensorOverlaps`
- [ ] `b3Shape_ComputeMassData`

## World misc (physics_world.c)

- [ ] `b3World_GetBounds`
- [ ] `b3World_GetWorkerCount` / `b3World_SetWorkerCount` (serial port: setter
      clamps and stores like C with no task system; getter reports it) — port
      `TestSetWorkerCount`
- [ ] `b3World_DumpAwake` / `b3World_DumpShapeBounds` (logging helpers;
      `DumpMemoryStats` is N/A by design — the arena/block allocators are Vecs)

## Tests

- [ ] Port `WorldTest` (generic world coverage) from `test_world.c`
- [ ] Port `DestroyAllBodiesWorld` from `test_world.c`
- [ ] Port the introspection assertions from `BodyTest` if any were skipped
      when it landed
