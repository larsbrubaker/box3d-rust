# Task 12 — Demo excellence: port Erin's Samples App 1:1

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

## Goal (non-negotiable)

**Port Erin Catto's Box3D Samples App demos 1:1.** Do **not** invent new demos.
Every interactive scene must map to a `RegisterSample(category, name, …)` entry
in `box3d-cpp-reference/samples/`. Same scene construction (exact body/shape
defs, positions, counts, friction/density, joint params), same per-sample
controls, same camera defaults (`SetView` values via the shared `setView`
helper), and the closest feasible match to the C app's rendering and UI shell.

Assets (meshes under `box3d-cpp-reference/data/meshes/`, including
`building.obj`) are **MIT** (Copyright 2026 Erin Catto) — copy/adapt with
attribution; see `demo/public/meshes/README.md`.

## Policies (settled 2026-07-12)

- **Counts**: use the C **release** value when it runs interactively in serial
  wasm; otherwise use the C **debug** value (both are Erin's numbers — never a
  third invented count). Disclose "C release uses N" in the info panel when the
  debug value is chosen.
- **No invented UI**: per-sample sliders/radios exist iff the C sample's
  `DrawControls` has them, with the same ranges and defaults.
- **Randomness**: reproduce C's RNG streams where practical (`XorShift32`
  matches C `RandomInt` bit-for-bit — see `sensor_demo.rs`); otherwise flag as
  inherent divergence in the demo's info text.
- **Structure**: one route per `RegisterSample` entry (category → sample tree
  like the C Samples menu) is the batch-2 target; until then, per-category
  pages with sample selectors are transitional.

## Verified inventory (full audit, 2026-07-12, 7-agent sweep)

~151 active `RegisterSample` entries (a few more are `#if 0` upstream).
**Exact: 8 · Partial: ~26 · Missing: ~117.** Invented content still live: 4
routes + several invented controls (removal in flight, batch 1).

| Category (C file) | Total | Exact | Partial | Missing | Notes |
|---|---|---|---|---|---|
| Compound | 6 | 0 | 4 | 2 | Simple/Spheres/Hulls/Village partial; Tile Floor, Mesh Tile missing; Village has invented dynamic bodies + lost its embedded mover/query viz |
| Bodies | 9 | 8 | 1 | 0 | ported batch 3a; Cast partial (translucent cast proxies / plane quads simplified) |
| Character | 4 | 0 | 1 | 3 | Mover partial (filler boxes instead of test_map01/stairs/torus/door); CapsulePlane, MoverOverlap, Rigid Body missing |
| Stacking | 14 | 6 | 8 | 0 | Cylinder Stack upgraded to clone_and_transform_hull (live); cylinders render as faceted hulls |
| Continuous | 10 | 8 | 2 | 0 | batch 3b; Mesh Drop (ticks-seed/auto-regen) and Stall (stall-threshold API not surfaced) partial |
| Joints | 16 | 13 | 3 | 0 | batch 3b: 12 new live incl. C Wheel slider bug reproduced; Ball and Chain/Driving/Revolute remain partial-labeled |
| Ragdoll | 4 | 4 | 0 | 0 | batch 3b: Mesh/Pile (20 humans)/Incline ported; multi-scene page |
| Benchmark | 17 | 8 | 9 | 0 | batch 3c: all 16 scenes; heavy ones on C debug counts (disclosed); instanceColor engine recolor live |
| World | 4 | 4 | 0 | 0 | ported batch 3a; `#/world` page, old `#/far-pyramid` deep links alias |
| Determinism | 1 | 1 | 0 | 0 | ported batch 3a; sleep step + world hash HUD (2×2×2 humans, C has no debug/release split) |
| Replay | 1 | 0 | 0 | 1 | Replay viewer — needs `b3RecPlayer` wasm bindings |
| Shapes | 12 | 12 | 0 | 0 | ported batch 3a; conveyor.obj shipped (MIT attribution), stamp order gate-tested |
| Events | 6 | 6 | 0 | 0 | batch 3b: Hit/Move/Joint/Persistent Contact ported; Joint break via grab/throw shell |
| Issues | 7 | 0 | 0 | 7 | honestly PLANNED |
| Robustness | 4 | 0 | 0 | 4 | honestly PLANNED |
| Collision | 12 | 9 | 3 | 0 | batch 3c: 11 ported; Mesh Scale/Long Ray Cast/Time of Impact partial (disclosed) |
| Geometry | 5 | 5 | 0 | 0 | batch 3c: #/geometry page (+#/hull alias); clone_and_transform_hull ported into src/ with 6 tests |
| Manifold | 9 | 9 | 0 | 0 | batch 3c: all 9 b3Collide* exercised; C drag/Shift-rotate + cache/feature radios |
| Mesh | 9 | 6 | 3 | 0 | batch 3c: real dynamics scenes + voxel/collision assets shipped; Height Field/Viewer/Creation Benchmark scaled/scoped (disclosed) |
| Tree | 1 | 0 | 0 | 1 | `#/tree` toy is invented (C: bounds-file benchmark, 1024 queries, profiling) |

Also invented: `#/math` canvas demo (no C sample), home-page "18 demos" claim,
Events padding via Benchmark Sensor.

## Rendering / controls / UI shell gap (audit summary)

C app: sokol + hand-written GLSL — Preetham sky driving IBL, 3-cascade CSM
(2048, PCF 3×3), XeGTAO, **AgX** tonemap, analytic sphere/capsule impostors
(spin rings), fat px/world debug lines with HIDE/DIM/DASHED occlusion,
edge-convexity overlay, selection outline; colors/materials come from the
engine via `b3World_Draw` → debug adapter (bodyType roughness
{0.70,0.55,0.40}, material presets in color high byte, transparent-dynamic
α=0.5). Camera: yaw 35°, pitch −25°, r 25, fov 50°, Alt-gated orbit/pan/zoom +
right-drag fly + WASD. Grab: **Ctrl+click**, motor joint hertz 7.5 damping 1.0
(our grab physics already match; trigger + ray-fraction tracking differ).
Keys: P/O/Shift+O/R/[/]/F/Tab/Esc/M/Ctrl+O. UI: menu bar (Sim/View/Render/
Samples/Help), Info panel (solver defaults: substeps 4, hertz 60, recycle
5 cm, sleep/warm/continuous on), diagnostics drawer.

**Decision (batch 2): keep Three.js; port the self-contained signature pieces
1:1** — AgX tonemap, Preetham sky → PMREM environment, GTAO pass, CSM addon,
fat lines (Line2), engine-driven colors via a `b3World_Draw`-equivalent data
path (stop re-deriving colors in TS), C camera gestures/keys, menu-bar shell
with Render/View parity (15 draw flags + force/joint scale), category→sample
tree. Impostors/reverse-Z are out of browser reach — tessellated meshes are
the accepted stand-in.

## Batches

- **Batch 1 — fidelity + integrity: DONE** (merged to main, f52e1f8).
- **Batch 2 — rendering/controls/UI shell parity: DONE** (branch
  `demo-render-ui-batch2`). Landed: AgX tonemap, Preetham sky→PMREM IBL,
  3-cascade CSM (add-time dirty-flag registration), GTAO, procedural C
  ground grid, C camera controller (Alt orbit/pan/zoom, right-drag fly +
  WASD, C sensitivities, default yaw 35/pitch −25), Ctrl+click grab
  (ray-fraction), click select, C-exact Shift+click bullet sphere
  (20·scale, scale 5.0 = 100 m/s, r 0.25, ×4 density), global keys
  P/O/Shift+O/R/[ ]/F/Tab, menu bar (Sim/View/Render/Samples/Help) with all
  15 draw flags + scales, 150-entry sample registry + category tree + deep
  links, engine-driven shape colors via captured `world_draw` style words,
  overlay text channel (body names/mass/sleep/contact features as
  JSON→sprites), ParamDef group/visibleWhen, typed telemetry consts,
  single-source view-flags/render-defaults modules.
  **Carry-overs (fold into batch 3):**
  - Fat debug lines: `render/lines.ts` ready; overlay still draws 1px
    `LineSegments` — adopt Line2 at C's 1.5px default.
  - Instanced/translucent style application (benchmark + sensors forks) —
    do with the Benchmark-family ports (`instanceColor` plumbing).
  - Registry↔page scene keys stringly-typed in 2-3 places — design a
    single-registration pattern before mass sample ports.
  - Style capture is per-frame full-world (buffers reused; no change
    gating) — add dirty/version gating if profiling demands.
  - Engine API gap: `CustomFilterFcn = fn(ShapeId,ShapeId,u64)` has no
    context/world access; C callbacks read shape userData mid-step. Fix as
    its own tested library change.
  - Shift+Ctrl (cylinder) / Shift+Alt (human) launch variants need wasm
    exports; per-sample `launchSpeedScale` override hook (Village = 2).
  - Character NaN report not reproducible (regression tests added:
    `character_demo_tests.rs`, `mover_tests.rs::integration`) — watch.
  - Gesture legend/hints are hand-written strings; derive from a gesture
    config if they churn again.
- **Batch 3a — Bodies (9) + Shapes (12) + World (3) + Determinism (1): DONE**
  (branch `demo-samples-batch3a`), plus the shared scaffold the remaining
  waves must use: `demo_shell!`/`demo_world_toggles!` macros (shell.rs),
  shared `rng.rs` (C-exact XorShift, 5 copies collapsed),
  `vis::capsule_from_body`/`mesh_triangle_edges_offset`, base-frame shift at
  collection (`interact::with_draw_base`), TS `makeInteractAdapter(wasm,
  prefix, overrides?)`, async-reset-safe `runLoop({ready})`,
  `makeStyleGate()` awake-gated style fetches, per-page `SCENES` export +
  `demo/tests/registry.test.ts` (bun test in deploy workflow).
  Known-state: the wasm crate never built under `double-precision`
  (pre-existing f32 demo code); library green under both.
- **Batch 3b — Joints (12) + Stacking (9) + Continuous (7) + Ragdoll (3) +
  Events (4): DONE** (branch `demo-samples-batch3b`). Post-review fixes:
  stacking_scenes/ split, C Wheel damping bug reproduced (sample_joint.cpp
  :1463), Incline timer 1/hertz, Events Joint grab/throw shell, shared
  TextLabelOverlay export, makeStyleGate on all pages (ragdoll_counters
  added), vis::mesh_triangle_edges_transform, draw-scale leak fixed with
  reset_scene_scales() at every world seam, mesh-drop ground-wireframe
  stale-edge contract fixed (NaN console error root-caused: THREE on
  empty/stale buffers, never Rust NaN — 6 finiteness tests added).
  Engine gaps noted: CloneAndTransformHull and Get/SetStallThreshold not
  surfaced (two partial scenes).
- **Batch 3c — Collision (11) + Mesh (9) + Manifold (9) + Geometry (5) +
  Benchmark (13 + instanceColor): DONE** (branch `demo-samples-batch3c`).
  Library port: `clone_and_transform_hull` (hull.c:2265) with 6 tests —
  Cylinder Stack upgraded to the true C path. Review fixes: vis::hull_
  triangles/hull_edges consolidation, instanced-color upload gating,
  cached static casts, shared OBJ parse->build split, registry status
  convention documented + Collision statuses reconciled, and the
  site-wide computeBoundingSphere NaN root-caused (makeWireEdges offset
  param misused as opacity by ~8 callers -> NaN indices; param
  repurposed, verified NaN-free on a 14-route sweep).
  Carry-over for 3d: Benchmark Hull mirrored hull still uses a negated
  point cloud with a stale comment - upgrade to clone_and_transform_hull.
- **Batch 3d** — Character (3 + Mover rebuild), Robustness (4), Issues (7),
  Tree (1), Compound (2 + Village mover/query-viz restoration),
  Replay (1, needs player bindings).

## 4. Site polish (after coverage)

- [ ] Deep links + prev/next between real samples
- [ ] Per-demo link to matching C sample source
- [ ] Mobile touch controls
- [ ] Footer version + git hash
- [ ] Refresh `readme_hero.jpg`
