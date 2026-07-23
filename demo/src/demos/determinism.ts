// Determinism — the full sample_determinism.cpp roster. Batch 1 shipped Falling
// Ragdolls; batch 2 adds Wave Pile, Query Spawn, and Mesh Drop (the last moved
// here from Continuous upstream at c52908c).
//
// Falling Ragdolls / Wave Pile / Mesh Drop are "settle" scenes: they run until
// every body sleeps, then latch and report the sleep step + world hash on the HUD
// exactly like the C sample's `printf`. Query Spawn is a zero-gravity, query-driven
// spawner with a per-cycle ray / overlap AABB / swept sphere / spawn overlay. All
// scene builders, sleep/hash tracking, and the hash itself are the ported
// `box3d_rust::determinism` library functions; this page only wires them up.

import * as THREE from "three";
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
  lineMat,
  makeSphere,
  makeTriangleMesh,
  makeWireEdges,
  setView,
  trianglesFromWireframe,
} from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

export const SCENES = ["falling-ragdolls", "wave-pile", "query-spawn", "mesh-drop"];

/** Format a u32 as C's `0x%08X`. */
function hex8(v: number): string {
  return "0x" + (v >>> 0).toString(16).toUpperCase().padStart(8, "0");
}

// b3_color* → hex approximations used by the Query Spawn overlay.
const CC = {
  yellow: 0xffff00,
  green: 0x22c55e,
  red: 0xdc2626,
  magenta: 0xff00ff,
  cyan: 0x22d3ee,
  gray: 0x9ca3af,
  white: 0xffffff,
};

// ===========================================================================
// Settle scenes — Falling Ragdolls, Wave Pile, Mesh Drop.
// ===========================================================================

interface SettleConfig {
  sampleName: string;
  description: string;
  info: string;
  /** wasm reset export selecting this scene (returns render body count). */
  reset: () => number;
  /** C SetView(yaw, pitch, distance, target). */
  view: [number, number, number, [number, number, number]];
  /** DemoScene camera/grid options. */
  demoOpts: {
    target: [number, number, number];
    distance: number;
    fov?: number;
    shadowExtent?: number;
    gridSize?: number;
    gridDivisions?: number;
  };
  /** C GetGuiDraw()->forceScale (Mesh Drop = 0.1); default 1. */
  forceScale?: number;
}

function initSettleScene(container: HTMLElement, cfg: SettleConfig) {
  const wasm = getWasm();
  assertRouteScenes("determinism", SCENES);
  const { canvas, controls, page } = demoPage(
    container,
    cfg.sampleName,
    cfg.description,
    "Runs to sleep, then prints the hash · P/O/R",
    wasm.version(),
    { category: "Determinism", samplesShell: true },
  );

  controls.appendChild(createInfoBox(cfg.info));

  const overlay = createCanvasOverlay(page);

  const demo = new DemoScene(canvas, cfg.demoOpts);
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

  let latched = false; // set once the scene sleeps; cleared on restart

  function reset() {
    syncMeshesFromPoses(demo.content, pool, []);
    cfg.reset();
    // C GetGuiDraw()->forceScale — scales the optional force debug overlay.
    wasm.sim_set_draw_scales(1, cfg.forceScale ?? 1);
    buildGround();
    // Show the freshly dropped scene immediately (before the first step).
    syncMeshesFromPoses(demo.content, pool, wasm.determinism_poses(), {
      styles: wasm.determinism_styles(),
    });
    setView(demo, cfg.view[0], cfg.view[1], cfg.view[2], cfg.view[3]);
    overlay.innerHTML = "settling…";
    latched = false;
  }

  const ctrl = attachInteraction({
    wasm: makeInteractAdapter(wasm, "determinism"),
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: cfg.sampleName,
    sampleCategory: "Determinism",
    worldOrigin: [0, 0, 0],
  }) as SimControllerWithTick;

  reset();

  const styleGate = makeStyleGate<Uint32Array>();
  const stop = runLoop(() => {
    // Once the scene sleeps, latch and pause: render the frozen scene instead of
    // stepping + re-syncing every frame. Pause / Step / Restart still work.
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

function initFallingRagdolls(container: HTMLElement) {
  return initSettleScene(container, {
    sampleName: "Falling Ragdolls",
    description:
      "Determinism soak from <code>sample_determinism.cpp</code>: humans fall onto a grid " +
      "of mesh tiles, and once the pile sleeps the settled world hash is reported.",
    info:
      "Serial WASM uses the upstream counts <code>RAGDOLL_GRID_COUNT = 2</code> / " +
      "<code>RAGDOLL_GROUP_SIZE = 2</code> (no debug/release split): a 2×2 tile field " +
      "with 2 humans per tile. The sleep step and hash come from the ported " +
      "<code>update_falling_ragdolls</code> — the same hash the determinism gate uses.",
    // C FallingRagdolls: SetView( 45, 30, 40, b3Pos_zero ).
    reset: () => wasm().determinism_reset(),
    view: [45, 30, 40, [0, 0, 0]],
    demoOpts: {
      target: [0, 0, 0],
      distance: 40,
      fov: 50,
      shadowExtent: 40,
      gridSize: 60,
      gridDivisions: 30,
    },
  });
}

function initWavePile(container: HTMLElement) {
  return initSettleScene(container, {
    sampleName: "Wave Pile",
    description:
      "Determinism soak from <code>sample_determinism.cpp</code>: 100 mixed convex bodies " +
      "(spheres, capsules, boxes, rocks) drop onto a wave height field and settle into a pile.",
    info:
      "100 bodies (<code>WAVE_PILE_BODY_COUNT</code>) with rolling resistance so the pile " +
      "sleeps quickly. The sleep step and hash come from the ported " +
      "<code>update_wave_pile</code> — the same hash the determinism gate uses. Rocks render " +
      "as low-poly icosahedron stand-ins; every other shape is exact.",
    // C WavePile: SetView( 45, 25, 25, b3Pos_zero ).
    reset: () => wasm().determinism_reset_wave_pile(),
    view: [45, 25, 25, [0, 0, 0]],
    demoOpts: {
      target: [0, 0, 0],
      distance: 25,
      fov: 50,
      shadowExtent: 30,
      gridSize: 40,
      gridDivisions: 20,
    },
  });
}

function initMeshDrop(container: HTMLElement) {
  return initSettleScene(container, {
    sampleName: "Mesh Drop",
    description:
      "Determinism soak from <code>sample_determinism.cpp</code> (moved from Continuous at " +
      "c52908c): a 20×20 grid of thin fast boxes drops onto a wave mesh and settles.",
    info:
      "400 thin fast boxes (<code>MESH_DROP_GRID_COUNT = 20</code>) stress continuous " +
      "collision and mesh contact stability, doubling as a determinism scenario via the " +
      "sleep hash. The sleep step and hash come from the ported <code>update_mesh_drop</code>. " +
      "Force draw scale is 0.1, matching the C sample.",
    // C MeshDropDeterminism: SetView( 0, 30, 20, b3Pos_zero ); forceScale 0.1.
    reset: () => wasm().determinism_reset_mesh_drop(),
    view: [0, 30, 20, [0, 0, 0]],
    forceScale: 0.1,
    demoOpts: {
      target: [0, 0, 0],
      distance: 20,
      fov: 50,
      shadowExtent: 40,
      gridSize: 60,
      gridDivisions: 30,
    },
  });
}

// ===========================================================================
// Query Spawn — zero-gravity query-driven spawning with a per-cycle overlay.
// ===========================================================================

function initQuerySpawn(container: HTMLElement) {
  const w = getWasm();
  assertRouteScenes("determinism", SCENES);
  const { canvas, controls, page } = demoPage(
    container,
    "Query Spawn",
    "Determinism scene from <code>sample_determinism.cpp</code>: in zero gravity a ray cast, " +
      "an overlap query, and a sphere cast each cycle decide where and what to spawn, until " +
      "50 bodies drift into a settled cloud.",
    "Query-driven spawning · P/O/R",
    w.version(),
    { category: "Determinism", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "Each cycle a ray cast picks the spawn position, an overlap query picks the shape type, " +
        "and a sphere cast sets its size (all ported library functions). Overlay: <strong>ray = " +
        "yellow</strong>, <strong>overlap box = magenta</strong>, <strong>sphere cast = cyan</strong> " +
        "(gray on a full-length miss), <strong>spawn = white</strong>.<br><br>" +
        "<strong>Disclosure:</strong> the on-screen version advances the scenario every 10th step " +
        "so each query lingers (C <code>QuerySpawn::Step</code>). That diverges from the unit-test " +
        "cadence, so the settled hash differs from the pinned <code>test_determinism.c</code> constants.",
    ),
  );

  const overlay = createCanvasOverlay(page);

  // C QuerySpawn: SetView( 45, 25, 30, b3Pos_zero ).
  const demo = new DemoScene(canvas, {
    target: [0, 0, 0],
    distance: 30,
    fov: 50,
    shadowExtent: 30,
    gridSize: 40,
    gridDivisions: 20,
  });
  const pool = createMeshPool();

  // Per-frame overlay group, rebuilt each frame from determinism_query_viz.
  const ovGroup = new THREE.Group();
  demo.dynamic.add(ovGroup);
  const ovItems: THREE.Object3D[] = [];

  function clearOverlay() {
    for (const o of ovItems) {
      ovGroup.remove(o);
      const m = o as THREE.Mesh;
      if (m.geometry) m.geometry.dispose();
      const mat = (m as { material?: THREE.Material | THREE.Material[] }).material;
      if (Array.isArray(mat)) mat.forEach((x) => x.dispose());
      else if (mat) mat.dispose();
    }
    ovItems.length = 0;
  }

  function ovLine(a: number[], b: number[], color: number) {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute(
      "position",
      new THREE.Float32BufferAttribute([a[0]!, a[1]!, a[2]!, b[0]!, b[1]!, b[2]!], 3),
    );
    const line = new THREE.Line(geo, lineMat(color));
    ovGroup.add(line);
    ovItems.push(line);
  }

  function ovPoint(p: number[], color: number, size: number) {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute([p[0]!, p[1]!, p[2]!], 3));
    const pts = new THREE.Points(
      geo,
      new THREE.PointsMaterial({ color, size, sizeAttenuation: false }),
    );
    ovGroup.add(pts);
    ovItems.push(pts);
  }

  function ovWireBox(lo: number[], hi: number[], color: number) {
    const geo = new THREE.BoxGeometry(hi[0]! - lo[0]!, hi[1]! - lo[1]!, hi[2]! - lo[2]!);
    const edges = new THREE.EdgesGeometry(geo);
    geo.dispose();
    const lines = new THREE.LineSegments(edges, lineMat(color));
    lines.position.set(
      0.5 * (lo[0]! + hi[0]!),
      0.5 * (lo[1]! + hi[1]!),
      0.5 * (lo[2]! + hi[2]!),
    );
    ovGroup.add(lines);
    ovItems.push(lines);
  }

  function drawOverlay() {
    clearOverlay();
    const d = w.determinism_query_viz();
    if (d.length < 27) return;
    const spawnCount = d[0]! | 0;
    if (spawnCount === 0) return;

    const rayOrigin = [d[3]!, d[4]!, d[5]!];
    const rayPoint = [d[6]!, d[7]!, d[8]!];
    const rayHit = d[9]! > 0.5;
    const rayNormal = [d[10]!, d[11]!, d[12]!];
    const lo = [d[13]!, d[14]!, d[15]!];
    const hi = [d[16]!, d[17]!, d[18]!];
    const castFraction = d[19]!;
    const rayTranslation = [d[20]!, d[21]!, d[22]!];
    const spawn = [d[23]!, d[24]!, d[25]!];
    const castRadius = d[26]!;

    // Ray with hit point + normal, red end on a miss (C QuerySpawn::Render).
    ovLine(rayOrigin, rayPoint, CC.yellow);
    ovPoint(rayOrigin, CC.yellow, 5);
    if (rayHit) {
      ovPoint(rayPoint, CC.green, 8);
      ovLine(rayPoint, [rayPoint[0]! + rayNormal[0]!, rayPoint[1]! + rayNormal[1]!, rayPoint[2]! + rayNormal[2]!], CC.green);
    } else {
      ovPoint(rayPoint, CC.red, 5);
    }

    // Overlap AABB.
    ovWireBox(lo, hi, CC.magenta);

    // Swept sphere stopped at its cast fraction, gray when it reached full length.
    const castCenter = [
      rayOrigin[0]! + castFraction * rayTranslation[0]!,
      rayOrigin[1]! + castFraction * rayTranslation[1]!,
      rayOrigin[2]! + castFraction * rayTranslation[2]!,
    ];
    const castColor = castFraction < 1.0 ? CC.cyan : CC.gray;
    const sphereMesh = makeSphere(castCenter[0]!, castCenter[1]!, castCenter[2]!, castRadius, castColor, 0.3);
    sphereMesh.castShadow = false;
    sphereMesh.receiveShadow = false;
    ovGroup.add(sphereMesh);
    ovItems.push(sphereMesh);

    // Spawn marker (C DrawCross(pos, 0.5): DrawCrossEx spans ±(size*0.5) per axis).
    const s = 0.25;
    ovLine([spawn[0]! - s, spawn[1]!, spawn[2]!], [spawn[0]! + s, spawn[1]!, spawn[2]!], CC.white);
    ovLine([spawn[0]!, spawn[1]! - s, spawn[2]!], [spawn[0]!, spawn[1]! + s, spawn[2]!], CC.white);
    ovLine([spawn[0]!, spawn[1]!, spawn[2]! - s], [spawn[0]!, spawn[1]!, spawn[2]! + s], CC.white);
  }

  function reset() {
    clearOverlay();
    syncMeshesFromPoses(demo.content, pool, []);
    w.determinism_reset_query_spawn();
    w.sim_set_draw_scales(1, 1);
    setView(demo, 45, 25, 30, [0, 0, 0]);
    overlay.innerHTML = "spawning…";
  }

  const ctrl = attachInteraction({
    wasm: makeInteractAdapter(w, "determinism"),
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: "Query Spawn",
    sampleCategory: "Determinism",
    worldOrigin: [0, 0, 0],
  }) as SimControllerWithTick;

  reset();

  const styleGate = makeStyleGate<Uint32Array>();
  const stop = runLoop(() => {
    const stepped = ctrl.tickFrame();
    if (stepped) {
      const awake = w.determinism_counters()[5] ?? 0;
      const styles = styleGate(awake, () => w.determinism_styles());
      syncMeshesFromPoses(demo.content, pool, w.determinism_poses(), { styles });
      drawOverlay();
    }

    const d = w.determinism_query_viz();
    const spawnCount = d[0] ? d[0] | 0 : 0;
    const spawnTotal = d[2] ? d[2] | 0 : 50;
    const queryHits = d[1] ? d[1] | 0 : 0;
    if (w.determinism_done()) {
      const step = w.determinism_sleep_step();
      const hash = w.determinism_hash();
      const queryHash = w.determinism_query_hash();
      overlay.innerHTML =
        `sleep step = ${step}<br>hash = ${hex8(hash)}<br>` +
        `query hits = ${queryHits}, query hash = ${hex8(queryHash)}`;
    } else {
      overlay.innerHTML =
        `spawned = ${spawnCount} of ${spawnTotal}<br>query hits = ${queryHits}<br>` +
        `ray = yellow · overlap box = magenta · sphere cast = cyan · spawn = white`;
    }
    demo.render();
  }, controls);

  return () => {
    ctrl.dispose();
    stop();
    clearOverlay();
    ovGroup.parent?.remove(ovGroup);
    disposeMeshPool(pool);
    demo.dispose();
  };
}

// ===========================================================================
// Multi-scene dispatch.
// ===========================================================================

/** Non-null wasm accessor for the settle-scene config closures. */
function wasm() {
  return getWasm();
}

export function init(container: HTMLElement, scene?: string) {
  switch (scene) {
    case "wave-pile":
      return initWavePile(container);
    case "query-spawn":
      return initQuerySpawn(container);
    case "mesh-drop":
      return initMeshDrop(container);
    default:
      return initFallingRagdolls(container);
  }
}
