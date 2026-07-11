# Task 6 — Mesh & height-field narrow phase (mesh_contact.c)

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

Port `mesh_contact.c` (~1200 lines). This is the critical-path gap: today
`update_contact` (`src/contact/update.rs`) clears the manifolds of any contact
whose shape A is a mesh or height field, so **bodies fall straight through
mesh and height-field geometry**. It also blocks the determinism gate — the
`test_determinism.c` scene drops ragdolls onto grid/torus mesh grounds.

The triangle manifold kernels (`triangle_manifold.c` → `src/manifold/triangle*.rs`)
and the mesh/height-field trees and queries are already ported; this track is
the contact-level layer that walks candidate triangles, maintains the
per-contact triangle cache, and merges triangle manifolds.

Touches `src/contact/update.rs` and the `MeshContact` state in
`src/contact/mod.rs` (`triangle_cache`, `query_bounds` exist but are unused).

## Port (mesh_contact.c)

- [ ] Query bounds computation and caching (re-query the mesh BVH only when
      the moving shape leaves `query_bounds`)
- [ ] Candidate triangle gathering (mesh tree overlap; height-field cell
      iteration) with scale handling
- [ ] Per-triangle manifold generation via the triangle manifold kernels,
      including edge-welding flags (`identifyEdges` data) so interior edges
      don't catch
- [ ] Triangle cache maintenance (`TriangleCache` warm-start anchors matched
      by `triangle_index`; C keeps impulses across steps per triangle)
- [ ] Manifold array assembly on the contact (multiple manifolds per contact;
      `ContactSpec.manifold_count` already flows through the solver)
- [ ] Compound children that are meshes (the nested-mesh branch in
      `update_contact` marked `// Nested mesh child: mesh_contact.c.`)
- [ ] Hit-event flag handling (stop clearing `SIM_ENABLE_HIT_EVENT` once real
      manifolds exist)

## Tests

- [ ] Port `TestMeshDrop` from `test_world.c` (body settles and sleeps on a
      mesh)
- [ ] Test: sphere/hull dropped on a height field settles (mirrors mesh drop)
- [ ] Test: rolling across welded interior edges of a grid mesh does not snag
      (exercises edge flags)
