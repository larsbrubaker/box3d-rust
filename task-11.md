# Task 11 — Demo site samples

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

Frontend track (TypeScript/Three.js in `demo/`, wasm bindings) — fully
parallel with the Rust tracks; every needed physics feature is on main.
Mirror the C `samples/` categories. Existing demos live in `demo/src/demos`;
follow their structure. Build with `bun run build`, develop with `bun run dev`;
deploys to GitHub Pages on push to main.

Marquee samples first (one per category beats exhaustive coverage of one):

- [ ] Ragdolls (`sample_ragdoll.cpp`) — the falling-ragdolls determinism scene
      is already ported as `src/human` + `src/determinism`; expose it through
      wasm and it doubles as a soak test
- [ ] Joints (`sample_joint.cpp`) — hinge chain / bridge with motor toggles
- [ ] Mesh & height-field terrain (`sample_mesh.cpp`) — bodies settling on a
      grid/torus mesh
- [ ] Continuous/bullets (`sample_continuous.cpp`) — bullet vs thin wall with
      a CCD on/off toggle showing tunneling
- [ ] Sensors + events (`sample_events.cpp`) — begin/end touch and hit-event
      visualization
- [ ] Character mover (`sample_character.cpp`) — capsule mover on terrain,
      keyboard-driven
- [ ] Queries (`sample_world.cpp` ray/overlap portions) — interactive
      raycast/overlap visualizer (pairs with task-10 debug draw)
- [ ] Stacking/compound scenes (`sample_stacking.cpp`, `sample_compound.cpp`)
      — extend the existing Bodies/Stacking demos

Housekeeping:

- [ ] Wasm surface: expose the new APIs the samples need (joints, mover,
      queries, sensors) through `demo`'s binding layer
- [ ] Keep the deploy workflow green (`.github/workflows/deploy-demo.yml`)
