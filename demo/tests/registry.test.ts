// Build-time guard that the sample registry (the single source of truth for the
// category→scene tree) and each multi-scene page's implemented scene table agree
// exactly, in both directions. This is the CI-enforced version of the page-init
// `assertRouteScenes` self-check (which only `console.error`s at runtime): here a
// drift fails `bun test`, so a RegisterSample row added without its page scene —
// or a scene key renamed on only one side — cannot reach main.
//
// Each multi-scene page exports a `SCENES` const (its own scene table); this test
// imports those and diffs them against the registry's declared scenes per route.

import { test, expect } from "bun:test";
import { SAMPLES } from "../src/registry.ts";

import { SCENES as bodies } from "../src/demos/bodies.ts";
import { SCENES as benchmark } from "../src/demos/benchmark.ts";
import { SCENES as character } from "../src/demos/character.ts";
import { SCENES as compound } from "../src/demos/compound.ts";
import { SCENES as continuous } from "../src/demos/continuous.ts";
import { SCENES as determinism } from "../src/demos/determinism.ts";
import { SCENES as geometry } from "../src/demos/geometry.ts";
import { SCENES as joints } from "../src/demos/joints.ts";
import { SCENES as manifolds } from "../src/demos/manifolds.ts";
import { SCENES as mesh } from "../src/demos/mesh.ts";
import { SCENES as queries } from "../src/demos/queries.ts";
import { SCENES as ragdolls } from "../src/demos/ragdolls.ts";
import { SCENES as sensors } from "../src/demos/sensors.ts";
import { SCENES as shapes } from "../src/demos/shapes.ts";
import { SCENES as stacking } from "../src/demos/stacking.ts";
import { SCENES as world } from "../src/demos/world.ts";

/**
 * Each multi-scene route's page SCENES, plus any `extra` scene keys the page
 * implements that are intentionally not backed by a RegisterSample entry (mirrors
 * the `extra` arg to `assertRouteScenes` — e.g. Character's Village walkthrough is
 * a second view of the Compound/Village sample, not its own registry row).
 */
const PAGES: Record<string, { scenes: readonly string[]; extra?: readonly string[] }> = {
  bodies: { scenes: bodies },
  benchmark: { scenes: benchmark },
  character: { scenes: character, extra: ["village"] },
  compound: { scenes: compound },
  continuous: { scenes: continuous },
  determinism: { scenes: determinism },
  geometry: { scenes: geometry },
  joints: { scenes: joints },
  manifolds: { scenes: manifolds },
  mesh: { scenes: mesh },
  queries: { scenes: queries },
  ragdolls: { scenes: ragdolls },
  sensors: { scenes: sensors },
  shapes: { scenes: shapes },
  stacking: { scenes: stacking },
  world: { scenes: world },
};

/** Registry-declared scene keys for a route (entries that own a working scene). */
function registryScenes(route: string): string[] {
  return SAMPLES.filter((s) => s.route === route && s.scene != null).map((s) => s.scene!);
}

for (const [route, { scenes, extra = [] }] of Object.entries(PAGES)) {
  test(`registry <-> ${route} page scenes match exactly`, () => {
    const registry = new Set(registryScenes(route));
    const implemented = new Set(scenes);
    const allowed = new Set<string>([...registry, ...extra]);

    // Direction 1: every registry scene is implemented by the page.
    const missing = [...registry].filter((s) => !implemented.has(s)).sort();
    expect(missing).toEqual([]);

    // Direction 2: every page scene is backed by the registry (or a known extra).
    const unexpected = [...implemented].filter((s) => !allowed.has(s)).sort();
    expect(unexpected).toEqual([]);
  });
}

test("every multi-scene registry route is covered by this test", () => {
  const routesWithScenes = new Set(
    SAMPLES.filter((s) => s.scene != null && s.route).map((s) => s.route!),
  );
  const covered = new Set(Object.keys(PAGES));
  const uncovered = [...routesWithScenes].filter((r) => !covered.has(r)).sort();
  expect(uncovered).toEqual([]);
});
