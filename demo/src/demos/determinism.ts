// Determinism — Falling Ragdolls (sample_determinism.cpp).
//
// A grid of static grid-mesh + torus-mesh tiles catches groups of humans dropped
// from y = 15. The scene runs until every body sleeps, then the settled
// transforms are hashed; the HUD reports the sleep step and world hash exactly
// like the C sample's `printf`. The scene builder, sleep/hash tracking, and the
// hash itself are the ported `box3d_rust::determinism` library functions.

import type * as THREE from "three";
import {
  attachInteraction,
  makeInteractAdapter,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { createCanvasOverlay, createInfoBox } from "../controls.ts";
import { demoPage, makeStyleGate, runLoop } from "./common.ts";
import {
  DemoScene,
  makeTriangleMesh,
  makeWireEdges,
  setView,
  trianglesFromWireframe,
} from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

export const SCENES = ["falling-ragdolls"];

/** Format a u32 as C's `0x%08X`. */
function hex8(v: number): string {
  return "0x" + (v >>> 0).toString(16).toUpperCase().padStart(8, "0");
}

export function init(container: HTMLElement) {
  const wasm = getWasm();
  assertRouteScenes("determinism", SCENES);
  const { canvas, controls, page } = demoPage(
    container,
    "Falling Ragdolls",
    "Determinism soak from <code>sample_determinism.cpp</code>: humans fall onto a grid " +
      "of mesh tiles, and once the pile sleeps the settled world hash is reported.",
    "Runs to sleep, then prints the hash · P/O/R",
    wasm.version(),
    { category: "Determinism", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "Serial WASM uses the upstream counts <code>RAGDOLL_GRID_COUNT = 2</code> / " +
        "<code>RAGDOLL_GROUP_SIZE = 2</code> (no debug/release split): a 2×2 tile field " +
        "with 2 humans per tile. The sleep step and hash come from the ported " +
        "<code>update_falling_ragdolls</code> — the same hash the determinism gate uses.",
    ),
  );

  const overlay = createCanvasOverlay(page);

  // C FallingRagdolls: SetView( 45, 30, 40, b3Pos_zero ).
  const demo = new DemoScene(canvas, {
    target: [0, 0, 0],
    distance: 40,
    fov: 50,
    shadowExtent: 40,
    gridSize: 60,
    gridDivisions: 30,
  });
  const pool = createMeshPool();

  let groundTri: THREE.Mesh | null = null;
  let groundWire: THREE.LineSegments | null = null;

  function clearGround() {
    if (groundTri) {
      demo.content.remove(groundTri);
      groundTri.geometry.dispose();
      (groundTri.material as THREE.Material).dispose();
      groundTri = null;
    }
    if (groundWire) {
      demo.content.remove(groundWire);
      groundWire.geometry.dispose();
      (groundWire.material as THREE.Material).dispose();
      groundWire = null;
    }
  }

  function buildGround() {
    clearGround();
    const wire = wasm.determinism_ground_wireframe();
    if (!wire.length) return;
    const positions = trianglesFromWireframe(wire);
    groundTri = makeTriangleMesh(positions, 0x8a94a6, 0.9);
    groundTri.receiveShadow = true;
    groundWire = makeWireEdges(wire, 0x4a5568, 0.35);
    demo.content.add(groundTri);
    demo.content.add(groundWire);
  }

  let latched = false; // set once the pile sleeps; cleared on restart

  function reset() {
    syncMeshesFromPoses(demo.content, pool, []);
    wasm.determinism_reset();
    buildGround();
    // Show the freshly dropped pile immediately (before the first step).
    syncMeshesFromPoses(demo.content, pool, wasm.determinism_poses(), {
      styles: wasm.determinism_styles(),
    });
    setView(demo, 45, 30, 40, [0, 0, 0]);
    overlay.innerHTML = "settling…";
    latched = false;
  }

  const ctrl = attachInteraction({
    wasm: makeInteractAdapter(wasm, "determinism"),
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: "Falling Ragdolls",
    sampleCategory: "Determinism",
    worldOrigin: [0, 0, 0],
  }) as SimControllerWithTick;

  reset();

  const styleGate = makeStyleGate<Uint32Array>();
  const stop = runLoop(() => {
    // Once the pile sleeps, latch and pause: render the frozen pile instead of
    // stepping + re-syncing every frame. Pause / Step / Restart still work
    // (Step runs a manual tick; Restart clears the latch via reset()).
    if (wasm.determinism_done() && !latched) {
      latched = true;
      ctrl.setPaused(true);
    }
    const stepped = ctrl.tickFrame();
    if (stepped) {
      const awake = wasm.determinism_counters()[5] ?? 0;
      const styles = styleGate(awake, () => wasm.determinism_styles());
      syncMeshesFromPoses(demo.content, pool, wasm.determinism_poses(), { styles });
    }

    if (wasm.determinism_done()) {
      const step = wasm.determinism_sleep_step();
      const hash = wasm.determinism_hash();
      overlay.innerHTML = `sleep step = ${step}<br>hash = ${hex8(hash)}`;
    } else {
      overlay.innerHTML = `settling… step ${wasm.determinism_step_count()}`;
    }
    demo.render();
  }, controls);

  return () => {
    ctrl.dispose();
    stop();
    clearGround();
    disposeMeshPool(pool);
    demo.dispose();
  };
}
