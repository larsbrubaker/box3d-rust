# Task 5 — Deferred world/body query tests

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

Core world query/cast/explode/get-set API and character mover are on main.
Everything below is deferred until task-4 body/shape APIs land.

## Deferred tests (need task-4)

- [ ] Port `test/test_body_query.c` (`b3Body_CastRay` / `CastShape` /
      `OverlapShape` — body-level query API)
- [ ] Port `TestContactEvents`, `EnableContactRecyclingTest`,
      `TestHullDatabase` from `test_world.c` (`SetHull`, contact-recycling
      body flag)

Not ported by design: `TestWorldRecycle` (global world registry).
