# Task 10 — Debug draw

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

Port the `b3DebugDraw` interface and `b3World_Draw` from `physics_world.c`.
The `World` data model already carries the draw bookkeeping bitsets
(`debug_body_set`, `debug_joint_set`, `debug_contact_set`, `debug_island_set`)
— nothing consumes them yet. This is the last piece of `physics_world.c` and
what the demo site's debug visualizer will sit on.

## Interface (types.h b3DebugDraw)

- [ ] `DebugDraw` trait (or struct of callbacks, mirroring C) with the draw
      option flags: shapes, joints, joint extras, bounds, mass, body names,
      contacts, graph colors, contact normals/impulses/features, friction,
      islands
- [ ] Culling AABB support (`drawingBounds`)

## Traversal (physics_world.c)

- [ ] `b3World_Draw`: shape drawing per type (sphere, capsule, hull, mesh,
      height field, compound children) with awake/sleeping/static coloring
- [ ] Joint drawing dispatch (per-type draw fns in the joint files)
- [ ] Contact point / normal / impulse drawing, island overlays, mass axes,
      broad-phase AABBs
- [ ] The debug_*_set bitset maintenance that avoids redrawing shared
      geometry (hulls/meshes referenced by multiple shapes)

## Tests / consumers

- [ ] Smoke test: a scene draws without panicking and hits every shape type
      (a counting DebugDraw impl asserting expected primitive counts)
- [ ] Wire a wasm-side DebugDraw into the demo site as the query/contact
      visualizer backend (see todo.md demo section)
