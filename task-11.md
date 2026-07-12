# Task 11 — Demo site samples

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

Frontend track (TypeScript/Three.js in `demo/`, wasm bindings) — fully
parallel with the Rust tracks; every needed physics feature is on main.
Mirror the C `samples/` categories. Existing demos live in `demo/src/demos`;
follow their structure. Build with `bun run build`, develop with `bun run dev`;
deploys to GitHub Pages on push to main.

Marquee samples first (one per category beats exhaustive coverage of one):

- [ ] Mesh & height-field terrain (`sample_mesh.cpp`) — bodies settling on a
      grid/torus mesh (geometry mesh/HF demos already exist; need dynamics settle)
- [ ] Character mover (`sample_character.cpp`) — capsule mover on terrain,
      keyboard-driven
- [ ] Stacking/compound scenes (`sample_stacking.cpp`, `sample_compound.cpp`)
      — extend the existing Bodies/Stacking demos (pyramid / compound)

Housekeeping:

- [ ] Keep the deploy workflow green (`.github/workflows/deploy-demo.yml`)
