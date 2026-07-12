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

- [ ] Pause / single-step / restart controls with hotkeys (Space, S, R) and a
      time-scale slider — mirror the C sample app's loop controls
- [ ] Mouse pick & drag: raycast the pointer into the scene
      (`world_cast_ray_closest`), grab the hit body with a spherical mouse
      joint, drag on the camera-facing plane, release to fling. Touch
      equivalent for mobile. This is the single biggest "feels alive" feature
      of the C samples
- [ ] Click-to-spawn: shift-click drops a random shape (sphere/box/capsule)
      into any dynamics demo; ctrl-click deletes the picked body
- [ ] Stats overlay: rolling-average step time (ms), body/shape/contact/joint
      counts, awake vs sleeping counts — the wasm side already exposes the
      counters API
- [ ] Debug-draw overlay: wire the ported `DebugDraw` through wasm into a
      Three.js line renderer with toggles for contact points/normals/impulses,
      joint frames, AABBs, islands, mass axes — the toggle set from
      `b3DebugDraw`
- [ ] Per-demo parameter panel registry: declarative sliders/checkboxes/
      dropdowns per demo (counts, sizes, friction, restitution, gravity...)
      with restart-on-change semantics, like the C ImGui panel

## 2. Visual quality

- [ ] Shadows: directional key light with PCF-soft shadow maps sized to each
      scene's bounds (the single largest visual upgrade)
- [ ] Ground & environment: checkered/grid ground material, horizon gradient
      or sky, subtle distance fog; consistent with a dark UI theme
- [ ] Body colorization matching the C debug palette: static/kinematic/
      dynamic-awake/sleeping/bullet each get their color, with a smooth
      transition on sleep/wake (sleep state is visible physics — show it)
- [ ] Instanced meshes for high-body-count scenes (stacks, pyramids,
      benchmark scenes) so draw calls don't cap scene size
- [ ] Camera polish: per-demo initial framing, orbit damping, auto-reframe on
      restart, double-click to focus a body
- [ ] Tone mapping + antialiasing pass; consistent material roughness so
      scenes read as one family

## 3. Coverage — close the sample-category gaps

New demos, roughly one marquee scene per missing C category:

- [ ] Replay (sample_replay.cpp) — record a session in-browser via the ported
      recording API, replay it bit-exactly, download/upload the session file.
      Nothing shows off the determinism story better
- [ ] Shapes gallery (sample_shapes.cpp) — every shape type together, wind
      toggle (`shape_apply_wind`), material property sliders
- [ ] Compound gallery (sample_compound.cpp) — compound bodies with mixed
      children, break/attach toggles
- [ ] Events (sample_events.cpp) — contact begin/end flashes, hit-event
      sparks with approach-speed readout, sensor enter/exit tinting
- [ ] Explosion (sample_world.cpp) — `world_explode` on click with falloff
      radius visualization
- [ ] Robustness (sample_robustness.cpp) — overlap recovery, high mass
      ratios, degenerate stacks
- [ ] Joints expansion (sample_joint.cpp is 16 samples) — motorized
      revolute/prismatic rigs, joint limits/springs playground with live
      sliders, breakable-force demo, bridge
- [ ] Benchmark scenes (sample_benchmark.cpp) — large pyramid / many-ragdoll
      scenes with the stats overlay front and center
- [ ] Determinism readout — the falling-ragdolls demo shows the live world
      hash and sleep step, asserting the golden values in-browser

## 4. Site polish

- [ ] Landing page: category cards with thumbnails (auto-captured canvas
      screenshots), demo count, port status
- [ ] Deep links (`#demo=ragdolls`) and prev/next navigation between demos
- [ ] Per-demo info box: what it shows, what to try, link to the matching C
      sample source and the Rust code that powers it
- [ ] Mobile: touch controls (orbit + drag), responsive panel layout
- [ ] Footer: version + git hash of the wasm build, GitHub link
- [ ] Refresh `readme_hero.jpg` once shadows and colorization land
