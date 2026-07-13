// Benchmark — the full sample_benchmark.cpp set (16 scenes), ported from
// benchmarks.c with Erin's DEBUG/release counts (disclosed per scene in the Info
// panel). Dynamic piles render as per-kind InstancedMeshes with per-instance
// engine-style colors (instanceColor), so sleep/wake recoloring shows on every
// body while the pile stays a single draw call. Capsule scenes (Rain, Chains) use
// the shared mesh pool; Hull is a pure wireframe geometry demo.

import * as THREE from "three";
import {
  attachInteraction,
  makeInteractAdapter,
  type ParamDef,
  type ParamValues,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { createButton, createCanvasOverlay, createInfoBox, createSlider } from "../controls.ts";
import { demoPage, makeStyleGate, runLoop } from "./common.ts";
import {
  applyInstancedStyles,
  DemoScene,
  makeShapeMaterial,
  makeTriangleMesh,
  makeWireEdges,
  setView,
  solidMat,
  trianglesFromWireframe,
} from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

const POSE_STRIDE = 16;

type Scene =
  | "pyramid"
  | "wide-pyramid"
  | "many-pyramids"
  | "rain"
  | "joint-grid"
  | "falling-boxes"
  | "candy-cups"
  | "explosion"
  | "height-field"
  | "trees"
  | "washer"
  | "large-world"
  | "hull"
  | "chains"
  | "destruction"
  | "junkyard";

export const SCENES: Scene[] = [
  "pyramid",
  "wide-pyramid",
  "many-pyramids",
  "rain",
  "joint-grid",
  "falling-boxes",
  "candy-cups",
  "explosion",
  "height-field",
  "trees",
  "washer",
  "large-world",
  "hull",
  "chains",
  "destruction",
  "junkyard",
];

const SCENE_LABEL: Record<Scene, string> = {
  pyramid: "Large Pyramid",
  "wide-pyramid": "Wide Pyramid",
  "many-pyramids": "Many Pyramids",
  rain: "Rain",
  "joint-grid": "Joint Grid",
  "falling-boxes": "Falling Boxes",
  "candy-cups": "Candy Cups",
  explosion: "Explosion",
  "height-field": "Height Field",
  trees: "Falling Trees",
  washer: "Washer",
  "large-world": "Large World",
  hull: "Hull",
  chains: "Chains",
  destruction: "Destruction",
  junkyard: "Junkyard",
};

// C SetView(yaw, pitch, distance, target) per sample_benchmark.cpp.
const SCENE_VIEW: Record<Scene, [number, number, number, [number, number, number]]> = {
  pyramid: [40, -10, 110, [0, 40, 0]],
  "wide-pyramid": [0, 5, 80, [0, 18, 0]],
  "many-pyramids": [-10, 10, 120, [0, 5, 0]],
  rain: [25, 10, 70, [0, 0, 0]],
  "joint-grid": [-25, 25, 94, [30, -30, 30]],
  "falling-boxes": [45, 10, 80, [0, 20, 0]],
  "candy-cups": [45, 20, 20, [0, 0, 0]], // DEBUG count → DEBUG camera radius 20
  explosion: [45, 20, 30, [0, 0, 0]], // release count → release radius 30
  "height-field": [0, 20, 50, [0, 0, 0]],
  trees: [20, 0, 140, [0, 15, 0]],
  washer: [15, 20, 60, [0, 15, 0]],
  "large-world": [0, 10, 250, [0, 0, 0]],
  hull: [0, 15, 5, [0, 0, 0]],
  chains: [0, 15, 50, [0, 5, 0]],
  destruction: [0, 40, 20, [0, 0, 0]], // DEBUG (small) camera
  junkyard: [45, 30, 125, [0, 0, 0]],
};

// Scenes whose bodies include capsules (rendered by the shared mesh pool).
const POOL_SCENES = new Set<Scene>(["rain", "chains"]);
// Scenes with a static mesh ground drawn from bench_mesh_wireframe.
const GROUND_SCENES = new Set<Scene>([
  "rain",
  "chains",
  "trees",
  "explosion",
  "height-field",
  "destruction",
]);

// Info-panel disclosure of the count choice per scene (C release vs DEBUG).
const SCENE_INFO: Record<Scene, string> = {
  pyramid:
    "Large Pyramid: baseCount 20 (C DEBUG; C release 90 is a 3D pyramid of hundreds of thousands of bodies). Sleep disabled like C.",
  "wide-pyramid": "Wide Pyramid: pyramidHeight 15 (C release) — a 1240-box 3D pyramid.",
  "many-pyramids":
    "Many Pyramids: 3×3 pyramids (C DEBUG; C release 14×14 = 10 780 bodies does not hold interactively).",
  rain: "Rain: GRID_COUNT 3, GROUP_SIZE 2 (C DEBUG; C release 10 / 3 = 300 ragdolls). Ragdolls rain onto mesh platforms, recycling column by column.",
  "joint-grid":
    "Joint Grid: n 10 (C DEBUG; C release 100 = 10 000 jointed spheres). Sleep disabled.",
  "falling-boxes": "Falling Boxes: n 50 (C release) — 50×8×8 = 3200 unit cubes.",
  "candy-cups":
    "Candy Cups: 4×4×4 = 64 convex cups (C DEBUG; C release 16³ = 4096). Cups render from the exact 8-sided frustum hull.",
  explosion:
    "Explosion: n 16 (C release) — 1089 cylinders. Set Magnitude then press Explode for a radial impulse from (0,-4,0).",
  "height-field":
    "Height Field: 50×50 wave field. Each readout casts a dense grid of rays (Radius 0) or sphere shapes (Radius > 0) straight down.",
  trees: "Falling Trees: mesh 150×200 (CreateTrees100), 10 trees × 22 tapering hulls (C DEBUG bodyCount; release 50).",
  washer:
    "Washer: gridCount 8 (C DEBUG; C release 20³ = 8000 cubes). A kinematic drum (real 36 wall + 4 rib hulls) tumbles the cubes.",
  "large-world":
    "Large World: 32×32 = 1024 static floor boxes at the origin (a broad-phase move-buffer stress test; C DEBUG grid, release 1000² = 1M is infeasible) with 16 dropped spheres.",
  hull:
    "Hull: 64 random points hulled (green) + mirror-scaled clone (yellow). " +
    "Each step runs create/clone trials × 200 (C DEBUG; C release 2000). " +
    "Timing via performance.now() around the wasm trial loops (no b3GetTicks in wasm).",
  chains: "Chains: gridCount 10 (C DEBUG; C release 25 = 2500 jointed capsules). Spherical-joint chains driven by noisy wind over a wave mesh.",
  destruction:
    "Destruction: gridCount 6, extent 0.75 (C DEBUG; C release 20 / 2.5). The block grid is re-spawned and re-exploded every 80 steps.",
  junkyard:
    "Junkyard: 2×21×21 = 882 rocks (C DEBUG; C release 24 layers). A kinematic cylinder pusher orbits the arena.",
};

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  assertRouteScenes("benchmark", SCENES);
  const interact = makeInteractAdapter(wasm, "bench");

  const { canvas, controls, page } = demoPage(
    container,
    "Benchmark",
    "The full <code>sample_benchmark.cpp</code> set — pyramids, rain, joints, explosions and more, " +
      "ported from <code>benchmarks.c</code> with Erin's DEBUG/release body counts.",
    "Ctrl+click grab · Shift+click spawn · click select · P/O/R",
    wasm.version(),
    { category: "Benchmark", samplesShell: true },
  );

  let scene: Scene =
    initialScene && SCENES.includes(initialScene as Scene) ? (initialScene as Scene) : "pyramid";
  let treeGridSize = 100;
  let ctrl!: SimControllerWithTick;

  const infoBox = createInfoBox(SCENE_INFO[scene]);
  controls.appendChild(infoBox);
  const overlay = createCanvasOverlay(page);

  const demo = new DemoScene(canvas, { target: [0, 8, 0], distance: 55, fov: 50, gridSize: 400 });
  demo.camera.far = 800;
  demo.camera.updateProjectionMatrix();

  // --- Render resources ---
  const pool = createMeshPool();
  let styleGate = makeStyleGate<Uint32Array>();

  // Per-kind unit geometries for the instanced pile path.
  const boxGeo = new THREE.BoxGeometry(2, 2, 2);
  const sphereGeo = new THREE.SphereGeometry(1, 18, 12);
  const cylGeo = new THREE.CylinderGeometry(1, 1, 2, 16);
  const icoGeo = new THREE.IcosahedronGeometry(1, 0);
  const kindGeo: Record<number, THREE.BufferGeometry> = {
    0: boxGeo,
    1: sphereGeo,
    3: cylGeo,
    4: icoGeo,
  };
  // One InstancedMesh + material per kind, lazily grown.
  const instByKind = new Map<number, THREE.InstancedMesh>();
  const matByKind = new Map<number, THREE.MeshStandardMaterial>();

  let groundTri: THREE.Mesh | null = null;
  let groundWire: THREE.LineSegments | null = null;
  let hullWireA: THREE.LineSegments | null = null;
  let hullWireB: THREE.LineSegments | null = null;
  let drumMesh: THREE.Mesh | null = null;
  let drumWire: THREE.LineSegments | null = null;
  // Candy Cups: the real frustum hull, swapped in for the kind-3 (cylinder) slot.
  let candyGeo: THREE.BufferGeometry | null = null;

  const _m = new THREE.Matrix4();
  const _p = new THREE.Vector3();
  const _q = new THREE.Quaternion();
  const _s = new THREE.Vector3();

  // Reusable per-kind scratch buffers for one frame's bucketing.
  const bucketMats: Map<number, THREE.Matrix4[]> = new Map();
  const bucketStyles: Map<number, number[]> = new Map();

  function ensureInstanced(kind: number, count: number): THREE.InstancedMesh {
    let inst = instByKind.get(kind);
    if (inst && inst.instanceMatrix.count >= count) return inst;
    if (inst) {
      demo.content.remove(inst);
      inst.dispose();
    }
    let mat = matByKind.get(kind);
    if (!mat) {
      mat = makeShapeMaterial();
      matByKind.set(kind, mat);
    }
    inst = new THREE.InstancedMesh(kindGeo[kind]!, mat, Math.max(count, 64));
    inst.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    inst.castShadow = true;
    inst.receiveShadow = true;
    instByKind.set(kind, inst);
    demo.content.add(inst);
    return inst;
  }

  function clearInstanced() {
    for (const inst of instByKind.values()) {
      demo.content.remove(inst);
      inst.dispose();
    }
    instByKind.clear();
  }

  function clearGround() {
    for (const obj of [groundTri, groundWire, hullWireA, hullWireB, drumMesh, drumWire]) {
      if (obj) {
        demo.content.remove(obj);
        obj.geometry.dispose();
        (obj.material as THREE.Material).dispose();
      }
    }
    groundTri = groundWire = hullWireA = hullWireB = drumMesh = drumWire = null;
  }

  function clearVisuals() {
    clearInstanced();
    clearGround();
    syncMeshesFromPoses(demo.content, pool, []); // drain the mesh pool
  }

  function buildGround() {
    if (scene === "hull") {
      // Green original hull + yellow mirror clone.
      const a = wasm.bench_mesh_wireframe();
      const b = wasm.bench_hull_wireframe_b();
      if (a.length) {
        hullWireA = makeWireEdges(a, 0x22c55e);
        demo.content.add(hullWireA);
      }
      if (b.length) {
        hullWireB = makeWireEdges(b, 0xeab308);
        demo.content.add(hullWireB);
      }
      return;
    }
    if (!GROUND_SCENES.has(scene)) return;
    const wire = wasm.bench_mesh_wireframe();
    if (!wire.length) return;
    const positions = trianglesFromWireframe(wire);
    groundTri = makeTriangleMesh(positions, 0x8a94a6, 0.95);
    groundTri.receiveShadow = true;
    groundWire = makeWireEdges(wire, 0x4a5568, 0.35);
    demo.content.add(groundTri);
    demo.content.add(groundWire);
  }

  function resetScene() {
    switch (scene) {
      case "pyramid": wasm.bench_reset_large_pyramid(); break;
      case "wide-pyramid": wasm.bench_reset_wide_pyramid(); break;
      case "many-pyramids": wasm.bench_reset_many_pyramids(); break;
      case "rain": wasm.bench_reset_rain(); break;
      case "joint-grid": wasm.bench_reset_joint_grid(); break;
      case "falling-boxes": wasm.bench_reset_falling_boxes(); break;
      case "candy-cups": wasm.bench_reset_candy_cups(); break;
      case "explosion": wasm.bench_reset_explosion(); break;
      case "height-field": wasm.bench_reset_height_field(); break;
      case "trees": wasm.bench_reset_trees(treeGridSize); break;
      case "washer": wasm.bench_reset_washer(); break;
      case "large-world": wasm.bench_reset_large_world(); break;
      case "hull": wasm.bench_reset_hull(); break;
      case "chains": wasm.bench_reset_chains(); break;
      case "destruction": wasm.bench_reset_destruction(); break;
      case "junkyard": wasm.bench_reset_junkyard(); break;
    }
  }

  function reset() {
    clearVisuals();
    resetScene();
    // Candy Cups render the real frustum hull; swap it into the kind-3 render slot.
    if (scene === "candy-cups") {
      if (!candyGeo) {
        const h = wasm.bench_candy_hull();
        candyGeo = new THREE.BufferGeometry();
        candyGeo.setAttribute("position", new THREE.BufferAttribute(new Float32Array(h), 3));
        candyGeo.computeVertexNormals();
      }
      kindGeo[3] = candyGeo;
    } else {
      kindGeo[3] = cylGeo;
    }
    buildGround();
    const [yaw, pitch, dist, target] = SCENE_VIEW[scene];
    setView(demo, yaw, pitch, dist, target);
    infoBox.innerHTML = SCENE_INFO[scene];
    treeGrid.style.display = scene === "trees" ? "" : "none";
    explosionControls.style.display = scene === "explosion" ? "" : "none";
    heightFieldControls.style.display = scene === "height-field" ? "" : "none";
    overlay.innerHTML = "";
    styleGate = makeStyleGate<Uint32Array>();
    ctrl?.setSampleName(SCENE_LABEL[scene]);
  }

  // --- Scene selector (declarative param, restarts on change) ---
  const params: ParamDef[] = [
    {
      type: "select",
      key: "scene",
      label: "Sample",
      options: SCENES.map((s) => ({ label: SCENE_LABEL[s], value: s })),
      default: scene,
      restart: true,
    },
  ];

  ctrl = attachInteraction({
    wasm: interact,
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: SCENE_LABEL[scene],
    sampleCategory: "Benchmark",
    params,
    onParamsChange: (values: ParamValues, key: string) => {
      if (key === "scene") scene = values.scene as Scene;
    },
  }) as SimControllerWithTick;

  // --- Falling Trees grid radio (sample_benchmark.cpp:702-719) ---
  const treeGrid = document.createElement("div");
  treeGrid.className = "control-row";
  for (const [label, value] of [["100 cm", 100], ["50 cm", 50], ["25 cm (~1M tris)", 25]] as const) {
    treeGrid.appendChild(
      createButton(label, () => {
        treeGridSize = value;
        if (scene === "trees") reset();
      }),
    );
  }
  controls.appendChild(treeGrid);

  // --- Explosion Magnitude slider + Explode button (:466-475) ---
  const explosionControls = document.createElement("div");
  explosionControls.appendChild(
    createSlider("Magnitude", 0, 2000, 1000, 1, (v) => wasm.bench_set_explosion_magnitude(v)),
  );
  explosionControls.appendChild(createButton("Explode", () => wasm.bench_explode()));
  controls.appendChild(explosionControls);

  // --- Height Field Radius slider (:543-547) ---
  const heightFieldControls = document.createElement("div");
  heightFieldControls.appendChild(
    createSlider("Radius", 0, 1, 0.1, 0.1, (v) => wasm.bench_set_height_field_radius(v)),
  );
  controls.appendChild(heightFieldControls);

  reset();

  // --- Instanced pile renderer (per-kind, per-instance engine colors) ---
  function renderInstanced(poses: Float32Array, styles: Uint32Array, stylesChanged: boolean) {
    const n = Math.floor(poses.length / POSE_STRIDE);
    for (const arr of bucketMats.values()) arr.length = 0;
    for (const arr of bucketStyles.values()) arr.length = 0;

    for (let i = 0; i < n; i++) {
      const o = i * POSE_STRIDE;
      const kind = poses[o + 14]!;
      const geoKind = kind === 2 ? 0 : kind; // capsules never reach here
      _p.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
      _q.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
      const a0 = poses[o + 7]!;
      const a1 = poses[o + 8]!;
      const a2 = poses[o + 9]!;
      if (geoKind === 0) _s.set(a0, a1, a2);
      else if (geoKind === 3) _s.set(a0, a1, a0);
      else _s.set(a0, a0, a0); // sphere / ico
      _m.compose(_p, _q, _s);

      let mats = bucketMats.get(geoKind);
      if (!mats) {
        mats = [];
        bucketMats.set(geoKind, mats);
      }
      let sts = bucketStyles.get(geoKind);
      if (!sts) {
        sts = [];
        bucketStyles.set(geoKind, sts);
      }
      mats.push(_m.clone());
      sts.push(styles[i] ?? 0);
    }

    // Hide any kind not present this frame, fill the rest.
    for (const [kind, inst] of instByKind) {
      const mats = bucketMats.get(kind);
      if (!mats || mats.length === 0) {
        inst.count = 0;
        continue;
      }
    }
    for (const [kind, mats] of bucketMats) {
      if (mats.length === 0) continue;
      const inst = ensureInstanced(kind, mats.length);
      for (let i = 0; i < mats.length; i++) inst.setMatrixAt(i, mats[i]!);
      inst.count = mats.length;
      inst.instanceMatrix.needsUpdate = true;
      applyInstancedStyles(inst, bucketStyles.get(kind)!, mats.length, 0, stylesChanged);
    }
  }

  // --- Height Field cast readout (throttled; C runs it every Render) ---
  let castTick = 0;
  function renderHeightFieldReadout() {
    castTick = (castTick + 1) % 6;
    if (castTick !== 0) return;
    const t0 = performance.now();
    const r = wasm.bench_height_field_cast();
    const ms = performance.now() - t0;
    const castCount = r[0] ?? 0;
    const hitCount = r[1] ?? 0;
    const aveUs = castCount > 0 ? (1000 * ms) / castCount : 0;
    overlay.innerHTML =
      `count = ${castCount}, hit count = ${hitCount}<br>ave cast time = ${aveUs.toFixed(3)} us`;
  }

  function renderDrum() {
    const d = wasm.bench_washer_drum();
    if (d.length < 9) return;
    if (!drumMesh) {
      // Real drum: 36 wall + 4 rib child hulls (solid faces + wireframe edges).
      const g = wasm.bench_washer_drum_geometry();
      let p = 0;
      const triCount = g[p++]! | 0;
      const tris = g.slice(p, p + triCount);
      p += triCount;
      const edgeCount = g[p++]! | 0;
      const edges = g.slice(p, p + edgeCount);
      if (triCount === 0) return;
      const mg = new THREE.BufferGeometry();
      mg.setAttribute("position", new THREE.BufferAttribute(new Float32Array(tris), 3));
      mg.computeVertexNormals();
      drumMesh = new THREE.Mesh(mg, solidMat(0x6b7280, 1));
      drumMesh.castShadow = true;
      drumMesh.receiveShadow = true;
      demo.content.add(drumMesh);
      drumWire = makeWireEdges(edges, 0x374151);
      demo.content.add(drumWire);
    }
    drumMesh.position.set(d[0]!, d[1]!, d[2]!);
    drumMesh.quaternion.set(d[3]!, d[4]!, d[5]!, d[6]!);
    if (drumWire) {
      drumWire.position.set(d[0]!, d[1]!, d[2]!);
      drumWire.quaternion.set(d[3]!, d[4]!, d[5]!, d[6]!);
    }
  }

  function renderHullReadout() {
    // C BenchmarkHull::Step — create then clone trial loops, timed separately.
    // Page-side performance.now() stands in for b3GetTicks / b3GetMilliseconds.
    const trials = (wasm.bench_hull_info()[0] ?? 200) | 0;
    const t0 = performance.now();
    const createArea = wasm.bench_hull_create_trials();
    const createMs = performance.now() - t0;
    const t1 = performance.now();
    const cloneArea = wasm.bench_hull_clone_trials();
    const cloneMs = performance.now() - t1;
    const createUs = trials > 0 ? (1000 * createMs) / trials : 0;
    const cloneUs = trials > 0 ? (1000 * cloneMs) / trials : 0;
    const ratio = cloneMs > 0 ? createMs / cloneMs : 0;
    overlay.innerHTML =
      `trials = ${trials}<br>` +
      `createTime (us) = ${createUs.toFixed(2)}, area = ${createArea.toFixed(2)}<br>` +
      `cloneTime (us) = ${cloneUs.toFixed(2)}, area = ${cloneArea.toFixed(2)}<br>` +
      `createTime / cloneTime = ${ratio.toFixed(2)}`;
  }

  // Tracks the last style buffer the gate handed back. When the pile is settled
  // the gate returns the very same cached array (referential identity), so an
  // unchanged reference means the per-instance colors need no re-upload this frame.
  let prevStyles: Uint32Array | undefined;
  const stop = runLoop(() => {
    ctrl.tickFrame();
    const poses = wasm.bench_poses() as Float32Array;
    const awake = wasm.bench_counters()[5] ?? 0;
    const styles = styleGate(awake, () => wasm.bench_styles() as Uint32Array);
    const stylesChanged = styles !== prevStyles;
    prevStyles = styles;

    if (scene === "hull") {
      renderHullReadout();
    } else if (POOL_SCENES.has(scene)) {
      syncMeshesFromPoses(demo.content, pool, poses, { styles });
    } else {
      renderInstanced(poses, styles, stylesChanged);
      if (scene === "washer") renderDrum();
      if (scene === "height-field") renderHeightFieldReadout();
    }

    demo.render();
  }, controls);

  return () => {
    ctrl.dispose();
    stop();
    clearVisuals();
    disposeMeshPool(pool);
    demo.dispose();
    boxGeo.dispose();
    sphereGeo.dispose();
    cylGeo.dispose();
    icoGeo.dispose();
    candyGeo?.dispose();
    for (const mat of matByKind.values()) mat.dispose();
  };
}
