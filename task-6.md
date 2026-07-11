# Task 6 — Character mover

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

Port `mover.c`: the plane-solver used for kinematic character controllers.
**Depends on task-5** — the mover collides via world overlap/cast machinery
(`b3World_CollideMover`, `b3World_CastMover`), so start this only after those
land (or land them here if task-5 is stalled; coordinate to avoid duplicates).

## Plane solver (mover.c)

- [ ] `b3CollidePlaneResult` / plane accumulation types
- [ ] `b3SolvePlanes` (iterative position solver against collected planes —
      port the iteration count and epsilon behavior exactly)
- [ ] `b3ClipVector` (velocity clipping against solved planes)

## World integration (physics_world.c)

- [ ] `b3World_CollideMover` (capsule vs world overlap collecting planes)
- [ ] `b3World_CastMover` (conservative capsule sweep)
- [ ] `b3Body_CollideMover` (single-body variant, body.c)

## Tests

- [ ] Port `test/test_mover.c`
