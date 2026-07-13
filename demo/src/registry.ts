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
    ["Body Type", "planned"],
    ["Spinning Book", "planned"],
    ["Gyroscopic Torque", "planned"],
    ["Weeble", "planned"],
    ["Disable", "planned"],
    ["Cast", "planned"],
    ["Kinematic", "planned"],
    ["Lock Mixing", "planned"],
    ["Fixed Rotation", "planned"],
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
    ["Falling Ragdolls", "planned"],
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
    ["Inclined Plane", "planned"],
    ["Rolling Resistance", "planned"],
    ["High Resistance", "planned"],
    ["Isotropic Friction", "planned"],
    ["Slide Twist", "planned"],
    ["Restitution", "planned"],
    ["Static Invoke", "planned"],
    ["Conveyor Belt", "planned"],
    ["Conveyor Mesh", "planned"],
    ["Wind", "planned"],
    ["Wind Drop", "planned"],
    ["Wind Flap", "planned"],
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
    ["Far Stack", "planned"],
    ["Far Pyramid", "live", "far-pyramid"],
    ["Far Ragdolls", "planned"],
    ["Far Mesh Drop", "planned"],
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
