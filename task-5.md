# Task 5 — World queries, casts, explosion, world API

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

Core world query/cast/explode/get-set API is ported. Remaining items need
body/shape API from task-4 (or are registry-only).

## Deferred tests (need task-4)

- [ ] Port `test/test_body_query.c` (`b3Body_CastRay` / `CastShape` /
      `OverlapShape` — body-level query API)
- [ ] Port `TestContactEvents`, `EnableContactRecyclingTest`,
      `TestHullDatabase` from `test_world.c` (`SetHull`, contact-recycling
      body flag)

Not ported by design: `TestWorldRecycle` (global world registry).
