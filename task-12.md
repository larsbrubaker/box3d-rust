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
| Bodies | 9 | 0 | 0 | 9 | `#/bodies` route is an invented drop scene — remove |
| Character | 4 | 0 | 1 | 3 | Mover partial (filler boxes instead of test_map01/stairs/torus/door); CapsulePlane, MoverOverlap, Rigid Body missing |
| Stacking | 14 | 0 | 5 | 9 | friction 0.3 vs C 0.6 (Single Box, Box Stack); density 1.0 vs 1000; invented count sliders; 4/5 cameras wrong; Jenga capsule mode missing |
| Continuous | 10 | 3 | 0 | 7 | Thin Wall, Bounce House, Bullet vs Stack **exact** |
| Joints | 16 | 1 | 3 | 12 | Gear Lift **exact**; Revolute near-exact; Ball and Chain r=1.5 vs 2.0 + invented count slider (C: 32 fixed); Driving wave 33×33 vs 50×50, camera 12 vs 7, arrows vs WASD |
| Ragdoll | 4 | 0 | 1 | 3 | Box contaminated by invented multi-human slider (C: 1 human, colorize false) |
| Benchmark | 17 | 0 | 4 | 13 | 3 scaled with wrong cameras; Junkyard diverges on ~every value; Benchmark Sensor mis-filed under Events, 12×12 vs 40×40, filter callback dropped |
| World | 4 | 1 | 0 | 3 | **Far Pyramid exact** (reference-quality port); Far Stack, Far Ragdolls, Far Mesh Drop missing |
| Determinism | 1 | 0 | 0 | 1 | Falling Ragdolls hash soak |
| Replay | 1 | 0 | 0 | 1 | Replay viewer — needs `b3RecPlayer` wasm bindings |
| Shapes | 12 | 0 | 0 | 12 | honestly PLANNED, nothing invented |
| Events | 6 | 2 | 0 | 4 | Sensor Visit + Sensor Hits **exact**; category falsely advertised LIVE |
| Issues | 7 | 0 | 0 | 7 | honestly PLANNED |
| Robustness | 4 | 0 | 0 | 4 | honestly PLANNED |
| Collision | 12 | 1 | 0 | 11 | **Cast World exact** except invented 10-sphere pre-spawn |
| Geometry | 5 | 0 | 1 | 4 | Box Hull partial (missing c/r/s sliders + dual-hull compare); `#/geometry` "Geometry Queries" is invented — remove |
| Manifold | 9 | 0 | 4 | 5 | all 4 present use wrong radii/extents/transforms/camera; cursor-follow instead of drag-translate/Shift-rotate; no cache/feature radios; triangle manifolds absent |
| Mesh | 9 | 0 | 3 | 6 | mesh/height-field are static raycast viewers with invented params, not the C dynamics scenes; `#/terrain` framing invented; only `building.obj` shipped of 8 needed assets |
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
- **Batch 3+ — port missing samples per category**, priority: Bodies (9),
  Shapes (12), Joints (12), Stacking (9), Events (4), Manifold (5+4 rebuilds),
  Collision (11), Mesh (6+3 rebuilds, ship .obj assets), Continuous (7),
  Ragdoll (3), Benchmark (13), Character (3 + Mover rebuild), World (3),
  Geometry (4+1), Robustness (4), Issues (7), Tree (1), Compound (2),
  Determinism (1), Replay (1, needs player bindings).

## 4. Site polish (after coverage)

- [ ] Deep links + prev/next between real samples
- [ ] Per-demo link to matching C sample source
- [ ] Mobile touch controls
- [ ] Footer version + git hash
- [ ] Refresh `readme_hero.jpg`
