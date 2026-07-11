# Task 5 — World queries, casts, explosion, world API

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

Port the world-level query/cast API and the remaining `b3World_*` surface
(~57 fns) from `physics_world.c`. The underlying machinery exists — dynamic
tree queries/ray casts, per-shape distance/TOI, broad phase — this track is
the world-level dispatch over shape proxies plus filtering and callbacks.

Independent of the solver-touching tracks. task-6 (mover) depends on the
cast/overlap machinery from this track.

## Queries and casts (physics_world.c)

- [ ] `b3World_OverlapAABB` (proxy filter callback plumbing)
- [ ] `b3World_OverlapShape` (distance-based overlap with filter)
- [ ] `b3World_CastRay` / `b3World_CastRayClosest`
- [ ] `b3World_CastShape` (shape sweep via TOI against candidates)
- [ ] Query filter (`b3QueryFilter`) semantics identical to C
      (category/mask bits, and the C early-out ordering — result fraction
      clipping must match exactly)

## Explosion + misc world ops

- [ ] `b3World_Explode` (overlap sphere, falloff, impulse application,
      wake semantics)
- [ ] `b3World_SetCustomFilterCallback` / `b3World_SetPreSolveCallback`
      registration API (fields already exist on `World`)
- [ ] `b3World_SetFrictionCallback` / `b3World_SetRestitutionCallback`

## World get/set surface

- [ ] Gravity, hit-event threshold, restitution threshold, contact tuning
      (hertz/damping/speed), maximum linear speed get/set
- [ ] Enable toggles: sleeping (`b3World_EnableSleeping` wakes everything on
      disable), continuous, speculative, warm starting
- [ ] Counters (`b3World_GetCounters`), profile getter, `b3World_GetAwakeBodyCount`
- [ ] `b3World_IsValid` / id validation helpers — port `TestIsValid`
- [ ] Event getters mirroring C (`b3World_GetBodyEvents`,
      `GetContactEvents`, `GetSensorEvents`, `GetJointEvents`) as idiomatic
      slices/accessors on `World`

## Tests

- [ ] Port `test/test_body_query.c`
- [ ] Port `TestWorldRecycle`, `TestWorldCoverage`, `TestContactEvents`,
      `EnableContactRecyclingTest`, `TestExplosion`, `TestHullDatabase`
      from `test_world.c`
