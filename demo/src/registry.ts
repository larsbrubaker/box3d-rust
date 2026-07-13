// Sample registry — one entry per C `RegisterSample( category, name, … )` in
// box3d-cpp-reference/samples/sample_*.cpp. This is the single source of truth
// for the category→sample tree, the Samples menu, prev/next ordering, and the
// registry-derived home page. Statuses are honest, per this convention:
//   `live`    — a route+scene exists and matches the C sample's scene, values,
//               controls, and camera with no undisclosed divergence.
//   `partial` — a route exists but has a *disclosed* divergence from C (e.g. a
//               scaled-down body/row count, a missing library API worked around,
//               or a corrected label). The divergence is noted at its entry here
//               and/or on the page.
//   `planned` — no route yet.
//
// Enumerated from the pinned submodule (v0.1.0+, 540ea38). 150 active entries
// across 19 categories. Three upstream RegisterSample calls are `#if 0`'d and
// therefore excluded: Bodies "Gyroscopic Precession", Benchmark "Large World"
// (the first one at :203; the live one at :1022 is kept), Ragdoll "Pose". The
// Replay viewer is registered through a non-RegisterSample path (g_replayIndex),
// represented here as its own single-entry "Replay" category (route "replay").
//
// ---------------------------------------------------------------------------
// Single-registration pattern (registry ↔ multi-scene page link)
// ---------------------------------------------------------------------------
// This registry is the ONE source of truth for the category→sample tree AND for
// the scene key each sample maps to inside its hosting page. A multi-scene page
// (stacking, compound, continuous, joints, sensors, benchmark, manifolds, mesh,
// character) never keeps a second, private list that has to be edited in lockstep
// with this file. Instead it validates its internal scene table against the
// registry at page-init with `assertRouteScenes(route, [...its scene keys])`,
// which `console.error`s the moment the two drift apart — no silent default
// fallback (see `scenesFor` / `assertRouteScenes` below).
//
// To add a sample a category agent does exactly TWO things:
//   1. Add ONE `RegisterSample`-mirroring entry here (name, status, route?,
//      scene?), placed in its category `cat(...)` block.
//   2. Implement that `scene` in the page named by `route` (its reset/camera/
//      controls) and include the scene key in the array passed to
//      `assertRouteScenes`.
// Nothing else needs to stay in sync: the tree, Samples menu, prev/next order,
// deep links, and home grid all derive from this array. If step 2 is forgotten
// (or a scene key is renamed on only one side) the page logs a loud error at
// init instead of quietly landing on the wrong scene.

export type SampleStatus = "live" | "partial" | "planned";

export interface SampleEntry {
  /** C category string (first RegisterSample arg). */
  category: string;
  /** C sample name (second RegisterSample arg). */
  name: string;
  /** URL-safe slug derived from the name; unique within a route. */
  slug: string;
  /** Hash route of the hosting demo page, when one exists. */
  route?: string;
  /** Scene key within a multi-scene page (the page's `mode`/`scene`/`kind` value). */
  scene?: string;
  /** Honest port status. */
  status: SampleStatus;
  /** C source file the sample lives in (for a future "view C source" link). */
  cSource: string;
}

/** name → lowercase-hyphen slug. "s&box mover" → "s-box-mover". */
export function slugify(name: string): string {
  return name
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, "-")
    .replace(/^-+|-+$/g, "");
}

// [name, status, route?, scene?]
type Spec = [string, SampleStatus, string?, string?];

function cat(category: string, cSource: string, specs: Spec[]): SampleEntry[] {
  return specs.map(([name, status, route, scene]) => ({
    category,
    name,
    slug: slugify(name),
    status,
    route,
    scene,
    cSource,
  }));
}

/**
 * The full inventory. Grouped by category for readability; `SAMPLES_SORTED`
 * applies the C sort order (Category then Name via strcmp).
 */
export const SAMPLES: SampleEntry[] = [
  ...cat("Bodies", "sample_bodies.cpp", [
    ["Body Type", "live", "bodies", "body-type"],
    ["Spinning Book", "live", "bodies", "spinning-book"],
    ["Gyroscopic Torque", "live", "bodies", "gyroscopic-torque"],
    ["Weeble", "live", "bodies", "weeble"],
    ["Disable", "live", "bodies", "disable"],
    // Live: ray / sphere-cast / overlap / CollideMover queries against a Shift-drag
    // target; cast proxies render as translucent solids and the CollideMover patches
    // as plane quads (C DrawPlane), matching the C sample's debug draw.
    ["Cast", "live", "bodies", "cast"],
    ["Kinematic", "live", "bodies", "kinematic"],
    ["Lock Mixing", "live", "bodies", "lock-mixing"],
    ["Fixed Rotation", "live", "bodies", "fixed-rotation"],
  ]),
  ...cat("Benchmark", "sample_benchmark.cpp", [
    ["Large Pyramid", "partial", "benchmark", "pyramid"],
    ["Wide Pyramid", "live", "benchmark", "wide-pyramid"],
    ["Many Pyramids", "partial", "benchmark", "many-pyramids"],
    ["Rain", "partial", "benchmark", "rain"],
    ["Joint Grid", "partial", "benchmark", "joint-grid"],
    ["Falling Boxes", "live", "benchmark", "falling-boxes"],
    // Partial: count is debug-scaled (4×4×4 = 64 cups; C release 16³ = 4096).
    // Rendering is now faithful — cups draw from the exact 8-sided frustum hull.
    ["Candy Cups", "partial", "benchmark", "candy-cups"],
    ["Explosion", "live", "benchmark", "explosion"],
    ["Height Field", "live", "benchmark", "height-field"],
    ["Falling Trees", "partial", "benchmark", "trees"],
    ["Sensor", "partial", "sensors", "benchmark"],
    // Partial: cube count is debug-scaled (grid 8; C release 20³ = 8000).
    // Rendering is now faithful — the drum draws its real 36 wall + 4 rib hulls.
    ["Washer", "partial", "benchmark", "washer"],
    ["Large World", "partial", "benchmark", "large-world"],
    ["Hull", "partial", "benchmark", "hull"],
    ["Chains", "partial", "benchmark", "chains"],
    ["Destruction", "partial", "benchmark", "destruction"],
    ["Junkyard", "partial", "benchmark", "junkyard"],
  ]),
  ...cat("Character", "sample_character.cpp", [
    ["CapsulePlane", "live", "character", "capsule-plane"],
    ["MoverOverlap", "live", "character", "mover-overlap"],
    ["Mover", "live", "character", "mover"],
    // Partial: RigidbodyCharacter physics, third-person boom, and Debug (V) match C.
    // Remaining disclosed divergence: mouse locks on canvas click (browsers can't
    // auto-lock like sapp_lock_mouse).
    ["Rigid Body", "partial", "character", "rigid-body"],
  ]),
  ...cat("Collision", "sample_collision.cpp", [
    ["Ray Curtain", "live", "queries", "ray-curtain"],
    ["Cast World", "live", "queries", "cast-world"],
    ["Mesh Scale", "live", "queries", "mesh-scale"],
    ["Shape Cast", "live", "queries", "shape-cast"],
    ["Overlap World", "live", "queries", "overlap-world"],
    // Live: the rock now renders from its real `create_rock` convex hull (solid
    // faces + wireframe edges) — the same hull the ray cast collides against.
    ["Long Ray Cast", "live", "queries", "long-ray-cast"],
    ["Initial Overlap", "live", "queries", "initial-overlap"],
    ["Shape Cast Debug", "live", "queries", "shape-cast-debug"],
    ["Distance Debug", "live", "queries", "distance-debug"],
    ["Shape Distance", "live", "queries", "shape-distance"],
    // Partial: the on-screen label corrects a mislabel in the C sample's overlay
    // text (a disclosed, intentional deviation from the C sample's wording).
    ["Time of Impact", "partial", "queries", "time-of-impact"],
    ["Capsule Cast Ray", "live", "queries", "capsule-cast-ray"],
  ]),
  ...cat("Compound", "sample_compound.cpp", [
    // Live: scene matches C SimpleCompound (static hull compound + dynamic sphere).
    ["Simple", "live", "compound", "simple"],
    // Live: 20 compound spheres placed from the shared g_randomSeed XorShift stream
    // (seed 12345), bit-identical to the C sample's RandomVec3 / RandomFloatRange order.
    ["Spheres", "live", "compound", "spheres"],
    // Live: 20 compound box hulls placed from the shared g_randomSeed XorShift stream
    // (seed 12345), matching the C RandomFloatRange / RandomVec3 / RandomQuat order.
    ["Hulls", "live", "compound", "hulls"],
    // Live: physics + placement RNG match C (XorShift g_randomSeed=12345); static
    // tiles render as one instanced mesh (visual-equivalent to C per-hull draw).
    ["Tile Floor", "live", "compound", "tile-floor"],
    // Live: 4 box-mesh compound tiles with C-exact XorShift Y offsets (seed 12345).
    ["Mesh Tile", "live", "compound", "mesh-tile"],
    // Live: the embedded character mover + sweeping ray/shape/overlap query are
    // ported (WASD walkthrough), and prop/building placement now consumes the shared
    // g_randomSeed XorShift stream (seed 12345) in C order. Grid is fixed at the C
    // debug value 8, so this reproduces the C *debug* build bit-for-bit (release 200
    // is too heavy for serial wasm).
    ["Village", "live", "compound", "village"],
  ]),
  ...cat("Continuous", "sample_continuous.cpp", [
    ["Thin Wall", "live", "continuous", "thin"],
    ["Bounce House", "live", "continuous", "bounce"],
    ["Spinning Stick", "live", "continuous", "spin"],
    ["Bullet vs Stack", "live", "continuous", "bullet"],
    ["Needle Mesh", "live", "continuous", "needle"],
    // Live: Generate reseeds from a performance.now()-derived tick value, matching
    // C's `g_randomSeed = b3GetTicks()` (any tick value is faithful); Auto Generate
    // regenerates on settle as in C `MeshDrop::Step`.
    ["Mesh Drop", "live", "continuous", "mesh-drop"],
    ["Mesh Drop Unit Test", "live", "continuous", "mesh-drop-unit"],
    ["Hump Mesh", "live", "continuous", "hump"],
    ["Is Fast", "live", "continuous", "is-fast"],
    // The CCD stall threshold is set to C's 1.0 ms and shown; the C sample's only
    // other use of it is a console log of slow CCD steps (no on-screen element).
    ["Stall", "live", "continuous", "stall"],
  ]),
  ...cat("Determinism", "sample_determinism.cpp", [
    ["Falling Ragdolls", "live", "determinism", "falling-ragdolls"],
  ]),
  ...cat("Events", "sample_events.cpp", [
    ["Sensor Visit", "live", "sensors", "visit"],
    ["Hit", "live", "sensors", "hit"],
    ["Move", "live", "sensors", "move"],
    // Live: the joint-break events are user-triggerable — throw bodies
    // (Shift+click) or grab and yank one (Ctrl+click) to drive a joint past its
    // force/torque threshold and watch it destroyed, matching the C sample.
    ["Joint", "live", "sensors", "joint"],
    ["Persistent Contact", "live", "sensors", "persistent"],
    ["Sensor Hits", "live", "sensors", "hits"],
  ]),
  ...cat("Geometry", "sample_geometry.cpp", [
    ["Box Hull", "live", "geometry", "box-hull"],
    ["Hull", "live", "geometry", "hull"],
    ["Hull Reduction", "live", "geometry", "hull-reduction"],
    ["Hull Transform", "live", "geometry", "hull-transform"],
    ["Capsule Mass", "live", "geometry", "capsule-mass"],
  ]),
  ...cat("Issues", "sample_issues.cpp", [
    // Partial: the C "dump" is emitted C++ source (b3World_Dump output #included into
    // the constructor), not a runtime-loadable format, and box3d-rust ports no dump
    // *loader* API — so the recorded body/shape defs (single rotated cube + ground) are
    // hand-ported inline. Values are bit-exact; only the load mechanism differs.
    ["Dump Loader", "partial", "issues", "dump-loader"],
    ["Crash", "live", "issues", "crash"],
    // Live: the six prismatic joints, ±6 limit, and constraintHertz 240 are exact,
    // and the scene now applies C's m_mouseForceScale = 1e6 via the shared grab's
    // force-scale override, so the mouse-drag strength matches C too.
    ["Multiple Prismatic", "live", "issues", "multiple-prismatic"],
    ["Hull Crash", "live", "issues", "hull-crash"],
    ["Convex Jitter", "live", "issues", "convex-jitter"],
    ["s&box mover", "live", "issues", "s-box-mover"],
    ["Capsule Mesh", "live", "issues", "capsule-mesh"],
  ]),
  ...cat("Joints", "sample_joint.cpp", [
    ["Distance Joint", "live", "joints", "distance"],
    ["Filter", "live", "joints", "filter"],
    ["Motor Joint", "live", "joints", "motor"],
    ["Top Down Friction", "live", "joints", "top-down-friction"],
    ["Prismatic", "live", "joints", "prismatic"],
    ["Spherical", "live", "joints", "spherical"],
    ["Parallel Spring", "live", "joints", "parallel"],
    // Live: hinge plank + shapeless parent, Limit/Motor/Spring DrawControls, and
    // energy HUD match C RevoluteJoint (sample_joint.cpp).
    ["Revolute", "live", "joints", "revolute"],
    ["Weld", "live", "joints", "weld"],
    ["Wheel", "live", "joints", "wheel"],
    // Live: 32 spherical-linked capsules + heavy tip (C hard-codes linkCount=32;
    // no DrawControls), matching BallAndChain scene construction.
    ["Ball and Chain", "live", "joints", "chain"],
    ["Door", "live", "joints", "door"],
    ["Bridge", "live", "joints", "bridge"],
    ["Motion Locks", "live", "joints", "motion-locks"],
    // Live: wave height field + chassis/wheels, Suspension/Motor/Steering
    // DrawControls, WASD drive, and chase-cam telemetry match C Driving.
    ["Driving", "live", "joints", "driving"],
    ["Gear Lift", "live", "joints", "gear"],
  ]),
  ...cat("Manifold", "sample_manifold.cpp", [
    ["Sphere vs Sphere", "live", "manifolds", "sphere-sphere"],
    ["Capsule vs Sphere", "live", "manifolds", "capsule-sphere"],
    ["Hull vs Sphere", "live", "manifolds", "hull-sphere"],
    ["Triangle vs Sphere", "live", "manifolds", "triangle-sphere"],
    ["Capsule vs Capsule", "live", "manifolds", "capsule-capsule"],
    ["Capsule vs Hull", "live", "manifolds", "capsule-hull"],
    ["Triangle vs Capsule", "live", "manifolds", "triangle-capsule"],
    ["Hull vs Hull", "live", "manifolds", "hull-hull"],
    ["Triangle vs Hull", "live", "manifolds", "triangle-hull"],
  ]),
  ...cat("Mesh", "sample_mesh.cpp", [
    ["Grid", "live", "mesh", "grid"],
    ["Big Box", "live", "mesh", "big-box"],
    ["Box", "live", "mesh", "box"],
    ["Reflection", "live", "mesh", "reflection"],
    // Height Field lives on its own single-scene route: row/column counts are
    // scaled down from the C 400/10 for browser render feasibility (disclosed on
    // the page), and both the ray and sphere shape-cast branches are ported.
    ["Height Field", "partial", "height-field"],
    // Viewer: BVH inspector, concave/weld controls, degenerate triangle labels.
    ["Viewer", "live", "mesh", "viewer"],
    // Creation Benchmark builds the four meshes on demand and reports the minimum
    // wall time (page-side timing rather than C's per-step b3GetTicks reduction).
    ["Creation Benchmark", "partial", "mesh", "creation-benchmark"],
    ["Voxel", "live", "mesh", "voxel"],
    ["Hollow Box", "live", "mesh", "hollow-box"],
  ]),
  ...cat("Ragdoll", "sample_ragdoll.cpp", [
    ["Box", "live", "ragdolls", "box"],
    ["Mesh", "live", "ragdolls", "mesh"],
    ["Pile", "live", "ragdolls", "pile"],
    ["Incline", "live", "ragdolls", "incline"],
  ]),
  ...cat("Robustness", "sample_robustness.cpp", [
    ["HighMassRatio1", "live", "robustness", "high-mass-ratio1"],
    ["Tiny Pyramid", "live", "robustness", "tiny-pyramid"],
    ["Overlap Recovery", "live", "robustness", "overlap-recovery"],
    ["Overflow Color Pile", "live", "robustness", "overflow-color-pile"],
  ]),
  ...cat("Shapes", "sample_shapes.cpp", [
    ["Inclined Plane", "live", "shapes", "inclined-plane"],
    ["Rolling Resistance", "live", "shapes", "rolling-resistance"],
    ["High Resistance", "live", "shapes", "high-resistance"],
    ["Isotropic Friction", "live", "shapes", "isotropic-friction"],
    ["Slide Twist", "live", "shapes", "slide-twist"],
    ["Restitution", "live", "shapes", "restitution"],
    ["Static Invoke", "live", "shapes", "static-invoke"],
    ["Conveyor Belt", "live", "shapes", "conveyor-belt"],
    ["Conveyor Mesh", "live", "shapes", "conveyor-mesh"],
    ["Wind", "live", "shapes", "wind"],
    ["Wind Drop", "live", "shapes", "wind-drop"],
    ["Wind Flap", "live", "shapes", "wind-flap"],
  ]),
  ...cat("Stacking", "sample_stacking.cpp", [
    // Live: exact C card dims/counts/materials; SetView + scene match CardHouseThick.
    ["Card House Thick", "live", "stacking", "card-house-thick"],
    // Live: exact C thin-card house; SetView matches CardHouse.
    ["Card House", "live", "stacking", "card-house"],
    // Live: exact C sphere count/materials; SetView matches SphereStack.
    ["Sphere Stack", "live", "stacking", "spheres"],
    // Live: exact C capsule stack; Z/angular locks keep collapse planar (as in C).
    ["Capsule Stack", "live", "stacking", "capsule-stack"],
    // Live: single box + C position HUD; SetView matches SingleBox.
    ["Single Box", "live", "stacking", "single"],
    // Live: 12-sided rolling cylinder hull; forceScale 0.01 and SetView match Cylinder.
    ["Cylinder", "live", "stacking", "cylinder"],
    // Live: builds each body via `b3CloneAndTransformHull` of one base cylinder
    // (identity transform + per-instance scale), exactly as C's
    // `b3CreateTransformedHullShape` does; camera/forceScale/values all match.
    ["Cylinder Stack", "live", "stacking", "cylinder-stack"],
    // Live: exact C box-stack counts; SetView matches BoxStack.
    ["Box Stack", "live", "stacking", "boxes"],
    // Live: Hull/Capsule DrawControls radio + exact C tower; SetView matches JengaStack.
    ["Jenga Stack", "live", "stacking", "jenga"],
    // Live: 30 rings + C kick impulse at reset; SetView matches Dominoes (release).
    ["Dominoes", "live", "stacking", "dominoes"],
    // Live: convex wedge hull scene; SetView matches Wedge.
    ["Wedge", "live", "stacking", "wedge"],
    // Live: convex arch hull scene; SetView matches Arch.
    ["Arch", "live", "stacking", "arch"],
    // Live: dual lines + C linear impulse at reset; SetView matches DoubleDomino.
    ["Double Domino", "live", "stacking", "double-domino"],
    // Live: exact C planar pyramid; Z locks keep collapse in-plane (as in C).
    ["Pyramid2D", "live", "stacking", "pyramid"],
  ]),
  ...cat("World", "sample_world.cpp", [
    ["Far Stack", "live", "world", "far-stack"],
    ["Far Pyramid", "live", "world", "far-pyramid"],
    ["Far Ragdolls", "live", "world", "far-ragdolls"],
    ["Far Mesh Drop", "live", "world", "far-mesh-drop"],
  ]),
  ...cat("Tree", "sample_tree.cpp", [
    // Partial: the AABB record files are fetched by JS (no fopen in the browser) and
    // query/build/profile timings are measured with performance.now() (no std::time in
    // wasm). Save / Load + Load Scale ARE wired (tree_save/tree_load): Save downloads
    // the serialized tree, Load re-reads it and rebuilds, Load Scale rescales — ported
    // as a portable leaf format (magic + real B3_DYNAMIC_TREE_VERSION guard) rather than
    // C's raw b3DynamicTree/b3TreeNode memory dump, which isn't portable across the
    // C/Rust node layouts; a Save→Load round-trip reproduces the tree exactly (wasm
    // round-trip test). Tree build, 1024-query generation (seed 12345 XorShift, exact),
    // ray/overlap/closest profiling, leaf + per-level visualization, and all readouts
    // are faithful.
    ["Benchmark", "partial", "tree", "benchmark"],
  ]),
  // Registered via g_replayIndex (RegisterReplay), not RegisterSample, so it is
  // its own single-entry category. Partial: transport + scrubber + faithful
  // playback ship, plus the outline scene tree + selection inspector (click a body in
  // the outline or the 3D view to highlight it and read its live transform / velocity /
  // state; C DrawOutlineTree + DrawBodyDetail). Still disclosed-skipped: the
  // whole-recording query search index (too heavy for the browser) and the
  // keyframe-policy popup (tunes a backward-seek keyframe ring our restart-and-replay
  // seek never consumes, so it would have no effect). See demos/replay.ts.
  ...cat("Replay", "sample_replay.cpp", [
    ["Viewer", "partial", "replay"],
  ]),
];

/** ASCII/code-unit comparison — matches C `strcmp`, NOT locale-aware collation. */
function strcmp(a: string, b: string): number {
  return a < b ? -1 : a > b ? 1 : 0;
}

/** C main.cpp CompareSamples: Category then Name (strcmp). */
export function compareSamples(a: SampleEntry, b: SampleEntry): number {
  return strcmp(a.category, b.category) || strcmp(a.name, b.name);
}

/** The inventory in the exact order the C Samples menu presents it. */
export const SAMPLES_SORTED: SampleEntry[] = [...SAMPLES].sort(compareSamples);

/** Categories in sorted (strcmp) order. */
export function categoryOrder(): string[] {
  const seen: string[] = [];
  for (const s of SAMPLES_SORTED) {
    if (!seen.includes(s.category)) seen.push(s.category);
  }
  return seen;
}

/** Sorted samples grouped by category, preserving sort order. */
export function samplesByCategory(): Map<string, SampleEntry[]> {
  const map = new Map<string, SampleEntry[]>();
  for (const s of SAMPLES_SORTED) {
    let list = map.get(s.category);
    if (!list) {
      list = [];
      map.set(s.category, list);
    }
    list.push(s);
  }
  return map;
}

export interface CategoryStats {
  live: number;
  partial: number;
  planned: number;
  total: number;
}

export function categoryStats(category: string): CategoryStats {
  const stats: CategoryStats = { live: 0, partial: 0, planned: 0, total: 0 };
  for (const s of SAMPLES) {
    if (s.category !== category) continue;
    stats.total += 1;
    stats[s.status] += 1;
  }
  return stats;
}

/** Aggregate live/partial/planned/total across the whole inventory. */
export function totalStats(): CategoryStats {
  const stats: CategoryStats = { live: 0, partial: 0, planned: 0, total: 0 };
  for (const s of SAMPLES) {
    stats.total += 1;
    stats[s.status] += 1;
  }
  return stats;
}

/** Entries with a working route (live + partial), in sorted order. */
export const NAVIGABLE_SAMPLES: SampleEntry[] = SAMPLES_SORTED.filter((s) => s.route);

/** Entries whose port is bit-exact with C (status === "live"), in sorted order. */
export const LIVE_SAMPLES: SampleEntry[] = SAMPLES_SORTED.filter((s) => s.status === "live");

/** Resolve a `#/<route>/<slug>` deep link to its entry. */
export function findByRouteSlug(route: string, slug: string): SampleEntry | undefined {
  return SAMPLES_SORTED.find((s) => s.route === route && s.slug === slug);
}

/** First routable entry whose page uses `route` (fallback when only a route is given). */
export function firstEntryForRoute(route: string): SampleEntry | undefined {
  return SAMPLES_SORTED.find((s) => s.route === route);
}

/**
 * Resolve a route + C sample name to its entry. Multi-scene pages call
 * `setSampleName(SCENE_LABEL[scene])` when the in-page selector switches scenes;
 * the label is the registry `name`, so this maps the displayed scene back to its
 * entry (used to keep the Info-panel C-source link / prev-next in sync without a
 * hash change). A route can host more than one category (e.g. `sensors` hosts
 * both Events samples and the Benchmark "Sensor"), so match on name within route.
 */
export function findByRouteName(route: string, name: string): SampleEntry | undefined {
  return SAMPLES_SORTED.find((s) => s.route === route && s.name === name);
}

/** Canonical deep-link hash for an entry (`#/<route>/<slug>`). */
export function entryHref(entry: SampleEntry): string {
  return `#/${entry.route}/${entry.slug}`;
}

/**
 * The neighbor of `entry` in the C-sorted navigable order (`NAVIGABLE_SAMPLES`),
 * clamped at the ends. `dir` = -1 previous, +1 next. When `entry` is undefined
 * (not currently on a navigable sample) the walk starts just past the matching
 * end so the first press lands on the first/last sample. Shared by the Sim menu,
 * the `[`/`]` keys, and the Info-panel ◀/▶ buttons so all three walk one order.
 */
export function neighborOf(entry: SampleEntry | undefined, dir: -1 | 1): SampleEntry | null {
  const list = NAVIGABLE_SAMPLES;
  if (list.length === 0) return null;
  let idx = entry ? list.findIndex((s) => s.route === entry.route && s.slug === entry.slug) : -1;
  if (idx === -1) idx = dir === 1 ? -1 : list.length;
  const next = Math.min(list.length - 1, Math.max(0, idx + dir));
  return list[next] ?? null;
}

/**
 * The pinned box3d-cpp-reference submodule commit (full hash), matching
 * `git -C box3d-cpp-reference rev-parse HEAD`. Used to build stable "C source"
 * links into the exact upstream sources this port mirrors.
 */
export const CPP_REFERENCE_COMMIT = "540ea387b0c02bf714fbfdcc8fb88c039c35fe6f";

/** Upstream GitHub URL for the C sample file an entry was ported from, at the pin. */
export function cSourceUrl(entry: SampleEntry): string {
  return `https://github.com/erincatto/box3d/blob/${CPP_REFERENCE_COMMIT}/samples/${entry.cSource}`;
}

/**
 * Registry entries hosted by a multi-scene page `route`, in registry (C sort)
 * order — the live/partial entries that own a working scene. A page can build its
 * selector straight from this instead of a private table, or keep its typed scene
 * table and validate it with {@link assertRouteScenes}. Entries whose `scene` is
 * undefined (single-scene routes) are still returned so callers can tell a route
 * apart from a genuinely multi-scene one.
 *
 * Internal to the registry (only {@link assertRouteScenes} consumes it); pages
 * validate their scene table via `assertRouteScenes` rather than reading this.
 */
function scenesFor(route: string): SampleEntry[] {
  return NAVIGABLE_SAMPLES.filter((s) => s.route === route);
}

/**
 * Dev self-check for a multi-scene page: assert the page implements exactly the
 * scenes the registry declares for `route`. Logs `console.error` on any drift so
 * a mismatch is loud at page-init (never a silent default fallback):
 *   - a registry entry whose `scene` the page does not implement (a RegisterSample
 *     row was added but the scene forgotten, or one side was renamed), and
 *   - a page scene with no matching registry entry — `extra` whitelists internal
 *     scenes intentionally not backed by a RegisterSample (e.g. a second view of a
 *     sample). The Character page dropped its old "village" walkthrough when it was
 *     rebuilt to the four real RegisterSample scenes; the Village walk is owned by
 *     the Compound sample/route.
 *
 * In a shipped build with no drift this logs nothing; a fired error is a real
 * registry↔page bug. Returns the registry scene keys for the route (handy when a
 * page wants to drive its selector from the registry too).
 */
export function assertRouteScenes(
  route: string,
  implemented: readonly string[],
  extra: readonly string[] = [],
): string[] {
  const registryScenes = scenesFor(route)
    .map((e) => e.scene)
    .filter((s): s is string => s != null);
  const impl = new Set(implemented);
  const allowed = new Set<string>([...registryScenes, ...extra]);
  for (const s of registryScenes) {
    if (!impl.has(s)) {
      console.error(
        `[registry] route "${route}": registry declares scene "${s}" but the page does not implement it`,
      );
    }
  }
  for (const s of implemented) {
    if (!allowed.has(s)) {
      console.error(
        `[registry] route "${route}": page implements scene "${s}" with no matching registry entry`,
      );
    }
  }
  return registryScenes;
}
