# Task 12 — Demo excellence: interactive, attractive, complete

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

The upstream samples app has ~90 samples across 19 categories with mouse-drag
interaction, pause/single-step, per-sample tuning panels, and debug overlays.
Our site (`demo/`, 17 demos) renders scenes well but plays like a viewer, not
a playground. This track closes that gap. All frontend/wasm-binding work
(TypeScript + `demo/wasm`); parallel-safe with any Rust track.

Work the sections in order — the interaction layer multiplies the value of
every demo that follows.

## 1. Core interaction layer (shared by all demos)

**Done (2026-07-12):** shared TS layer (`demo/src/interaction.ts`) + wasm
`interact`/`sim_*` bindings, wired into Bodies and Stacking.

- [x] Pause / single-step / restart controls with hotkeys (Space, S, R) and a
      time-scale slider — mirror the C sample app's loop controls
- [x] Mouse pick & drag: raycast the pointer into the scene
      (`world_cast_ray_closest`), grab the hit body with a motor joint
      (C samples use motor + kinematic mouse body; same feel as a spherical
      mouse joint), drag on the camera-facing plane, release to fling. Touch
      via pointer events.
- [x] Click-to-spawn: shift-click drops a random shape (sphere/box/capsule)
      into any dynamics demo; ctrl-click deletes the picked body
- [x] Stats overlay: rolling-average step time (ms), body/shape/contact/joint
      counts, awake vs sleeping counts — the wasm side already exposes the
      counters API
- [x] Debug-draw overlay: wire the ported `DebugDraw` through wasm into a
      Three.js line renderer with toggles for contact points/normals/impulses,
      joint frames, AABBs, islands, mass axes — the toggle set from
      `b3DebugDraw`
- [x] Per-demo parameter panel registry: declarative sliders/checkboxes/
      dropdowns per demo (counts, sizes, friction, restitution, gravity...)
      with restart-on-change semantics, like the C ImGui panel
      (framework + applied to Bodies and Stacking)

**Done (2026-07-12):** Samples App Info panel shell matching C `DrawInfoPanel`
(goldenrod name, category, pause, frame ms / step, camera readout, Solver
collapsing section with Hertz/sub-steps/Workers/Recycle/Sleep/Warm
Starting/Continuous/Restart, Recording panel wired to wasm `.b3rec`
start/stop + download, keyboard legend). Hertz/sub-steps drive `sim_step`.

**Remaining for section 1:** opt remaining dynamics demos (ragdolls,
continuous, sensors, queries, terrain) into `attachInteraction` / Samples
shell (joints, character, compound, benchmark now wired).

## 2. Visual quality

**Done (2026-07-12):** PCF-soft shadows on the directional key light; grey
grid floor + muted grey/blue sky + fog; C debug body-color helpers
(static/kinematic/dynamic awake/sleep) with flat-ish materials; tone mapping;
Bodies/Stacking consume the palette from pose `bodyType`/`awake`.

- [ ] Instanced meshes for high-body-count scenes (stacks, pyramids,
      benchmark scenes) so draw calls don't cap scene size
- [ ] Camera polish: per-demo initial framing, orbit damping, auto-reframe on
      restart, double-click to focus a body
- [ ] Refresh remaining dynamics demos to use Samples shell + body colorization
      (ragdolls, continuous, sensors, queries, terrain)

## 3. Coverage — close the sample-category gaps

**Done (2026-07-12, browser-scaled):** Stacking Single Box; Benchmark Large
Pyramid / Junkyard / Falling Trees; Joints Revolute / Gear Lift / Driving;
Compound Simple / Spheres / Hulls / Village; Character BasicMover (+ Village
walk). Recording UI on sim demos (download `.b3rec`); full Replay viewer still
open.

Still missing / partial vs C ~150 samples:

- [ ] Replay viewer (sample_replay.cpp) — scrub / upload / bit-exact replay UI
- [ ] Shapes gallery (sample_shapes.cpp)
- [ ] Events (sample_events.cpp)
- [ ] Explosion / World far scenes (sample_world.cpp)
- [ ] Robustness (sample_robustness.cpp)
- [ ] Remaining Joints (Bridge, Door, Wheel, Prismatic, …)
- [ ] Remaining Benchmark (Rain, Wide/Many Pyramids, Chains, …)
- [ ] Determinism readout on Falling Ragdolls
- [ ] Remaining Bodies / Stacking / Continuous / Mesh / Manifold / Collision samples

## 4. Site polish

- [ ] Landing page: category cards with thumbnails (auto-captured canvas
      screenshots), demo count, port status
- [ ] Deep links (`#demo=ragdolls`) and prev/next navigation between demos
- [ ] Per-demo info box: what it shows, what to try, link to the matching C
      sample source and the Rust code that powers it
- [ ] Mobile: touch controls (orbit + drag), responsive panel layout
- [ ] Footer: version + git hash of the wasm build, GitHub link
- [ ] Refresh `readme_hero.jpg` once shadows and colorization land
