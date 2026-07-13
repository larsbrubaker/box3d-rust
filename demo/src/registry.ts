// Sample registry — one entry per C `RegisterSample( category, name, … )` in
// box3d-cpp-reference/samples/sample_*.cpp. This is the single source of truth
// for the category→sample tree, the Samples menu, prev/next ordering, and the
// registry-derived home page. Statuses are honest: `live` means a route+scene
// exists and matches the C sample; `partial` means a route exists but diverges
// (wrong counts/cameras/controls per task-12.md); `planned` has no route yet.
//
// Enumerated from the pinned submodule (v0.1.0+, 540ea38). 150 active entries
// across 19 categories. Three upstream RegisterSample calls are `#if 0`'d and
// therefore excluded: Bodies "Gyroscopic Precession", Benchmark "Large World"
// (the first one at :203; the live one at :1022 is kept), Ragdoll "Pose". The
// Replay viewer is registered through a non-RegisterSample path (g_replayIndex)
// and is not represented here.
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
    ["Cast", "partial", "bodies", "cast"],
    ["Kinematic", "live", "bodies", "kinematic"],
    ["Lock Mixing", "live", "bodies", "lock-mixing"],
    ["Fixed Rotation", "live", "bodies", "fixed-rotation"],
  ]),
  ...cat("Benchmark", "sample_benchmark.cpp", [
    ["Large Pyramid", "partial", "benchmark", "pyramid"],
    ["Wide Pyramid", "planned"],
    ["Many Pyramids", "planned"],
    ["Rain", "planned"],
    ["Joint Grid", "planned"],
    ["Falling Boxes", "planned"],
    ["Candy Cups", "planned"],
    ["Explosion", "planned"],
    ["Height Field", "planned"],
    ["Falling Trees", "partial", "benchmark", "trees"],
    ["Sensor", "partial", "sensors", "benchmark"],
    ["Washer", "planned"],
    ["Large World", "planned"],
    ["Hull", "planned"],
    ["Chains", "planned"],
    ["Destruction", "planned"],
    ["Junkyard", "partial", "benchmark", "junkyard"],
  ]),
  ...cat("Character", "sample_character.cpp", [
    ["CapsulePlane", "planned"],
    ["MoverOverlap", "planned"],
    ["Mover", "partial", "character", "mover"],
    ["Rigid Body", "planned"],
  ]),
  ...cat("Collision", "sample_collision.cpp", [
    ["Ray Curtain", "planned"],
    ["Cast World", "live", "queries"],
    ["Mesh Scale", "planned"],
    ["Shape Cast", "planned"],
    ["Overlap World", "planned"],
    ["Long Ray Cast", "planned"],
    ["Initial Overlap", "planned"],
    ["Shape Cast Debug", "planned"],
    ["Distance Debug", "planned"],
    ["Shape Distance", "planned"],
    ["Time of Impact", "planned"],
    ["Capsule Cast Ray", "planned"],
  ]),
  ...cat("Compound", "sample_compound.cpp", [
    ["Simple", "partial", "compound", "simple"],
    ["Spheres", "partial", "compound", "spheres"],
    ["Hulls", "partial", "compound", "hulls"],
    ["Tile Floor", "planned"],
    ["Mesh Tile", "planned"],
    ["Village", "partial", "compound", "village"],
  ]),
  ...cat("Continuous", "sample_continuous.cpp", [
    ["Thin Wall", "live", "continuous", "thin"],
    ["Bounce House", "live", "continuous", "bounce"],
    ["Spinning Stick", "planned"],
    ["Bullet vs Stack", "live", "continuous", "bullet"],
    ["Needle Mesh", "planned"],
    ["Mesh Drop", "planned"],
    ["Mesh Drop Unit Test", "planned"],
    ["Hump Mesh", "planned"],
    ["Is Fast", "planned"],
    ["Stall", "planned"],
  ]),
  ...cat("Determinism", "sample_determinism.cpp", [
    ["Falling Ragdolls", "live", "determinism", "falling-ragdolls"],
  ]),
  ...cat("Events", "sample_events.cpp", [
    ["Sensor Visit", "live", "sensors", "visit"],
    ["Hit", "planned"],
    ["Move", "planned"],
    ["Joint", "planned"],
    ["Persistent Contact", "planned"],
    ["Sensor Hits", "live", "sensors", "hits"],
  ]),
  ...cat("Geometry", "sample_geometry.cpp", [
    ["Box Hull", "partial", "hull"],
    ["Hull", "planned"],
    ["Hull Reduction", "planned"],
    ["Hull Transform", "planned"],
    ["Capsule Mass", "planned"],
  ]),
  ...cat("Issues", "sample_issues.cpp", [
    ["Dump Loader", "planned"],
    ["Crash", "planned"],
    ["Multiple Prismatic", "planned"],
    ["Hull Crash", "planned"],
    ["Convex Jitter", "planned"],
    ["s&box mover", "planned"],
    ["Capsule Mesh", "planned"],
  ]),
  ...cat("Joints", "sample_joint.cpp", [
    ["Distance Joint", "planned"],
    ["Filter", "planned"],
    ["Motor Joint", "planned"],
    ["Top Down Friction", "planned"],
    ["Prismatic", "planned"],
    ["Spherical", "planned"],
    ["Parallel Spring", "planned"],
    ["Revolute", "partial", "joints", "revolute"],
    ["Weld", "planned"],
    ["Wheel", "planned"],
    ["Ball and Chain", "partial", "joints", "chain"],
    ["Door", "planned"],
    ["Bridge", "planned"],
    ["Motion Locks", "planned"],
    ["Driving", "partial", "joints", "driving"],
    ["Gear Lift", "live", "joints", "gear"],
  ]),
  ...cat("Manifold", "sample_manifold.cpp", [
    ["Sphere vs Sphere", "partial", "manifolds", "0"],
    ["Capsule vs Sphere", "planned"],
    ["Hull vs Sphere", "partial", "manifolds", "2"],
    ["Triangle vs Sphere", "planned"],
    ["Capsule vs Capsule", "partial", "manifolds", "1"],
    ["Capsule vs Hull", "planned"],
    ["Triangle vs Capsule", "planned"],
    ["Hull vs Hull", "partial", "manifolds", "3"],
    ["Triangle vs Hull", "planned"],
  ]),
  ...cat("Mesh", "sample_mesh.cpp", [
    ["Grid", "partial", "mesh", "grid"],
    ["Big Box", "planned"],
    ["Box", "partial", "mesh", "box"],
    ["Reflection", "planned"],
    ["Height Field", "partial", "height-field"],
    ["Viewer", "planned"],
    ["Creation Benchmark", "planned"],
    ["Voxel", "planned"],
    ["Hollow Box", "planned"],
  ]),
  ...cat("Ragdoll", "sample_ragdoll.cpp", [
    ["Box", "partial", "ragdolls"],
    ["Mesh", "planned"],
    ["Pile", "planned"],
    ["Incline", "planned"],
  ]),
  ...cat("Robustness", "sample_robustness.cpp", [
    ["HighMassRatio1", "planned"],
    ["Tiny Pyramid", "planned"],
    ["Overlap Recovery", "planned"],
    ["Overflow Color Pile", "planned"],
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
    ["Card House Thick", "planned"],
    ["Card House", "planned"],
    ["Sphere Stack", "partial", "stacking", "spheres"],
    ["Capsule Stack", "planned"],
    ["Single Box", "partial", "stacking", "single"],
    ["Cylinder", "planned"],
    ["Cylinder Stack", "planned"],
    ["Box Stack", "partial", "stacking", "boxes"],
    ["Jenga Stack", "partial", "stacking", "jenga"],
    ["Dominoes", "planned"],
    ["Wedge", "planned"],
    ["Arch", "planned"],
    ["Double Domino", "planned"],
    ["Pyramid2D", "partial", "stacking", "pyramid"],
  ]),
  ...cat("World", "sample_world.cpp", [
    ["Far Stack", "live", "world", "far-stack"],
    ["Far Pyramid", "live", "world", "far-pyramid"],
    ["Far Ragdolls", "live", "world", "far-ragdolls"],
    ["Far Mesh Drop", "live", "world", "far-mesh-drop"],
  ]),
  ...cat("Tree", "sample_tree.cpp", [
    ["Benchmark", "planned"],
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
 *     scenes intentionally not backed by a RegisterSample (e.g. Character's
 *     Village walkthrough, a second view of the Compound/Village sample).
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
