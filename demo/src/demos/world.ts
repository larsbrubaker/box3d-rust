// World — Far Stack, Far Pyramid, Far Ragdolls, Far Mesh Drop (sample_world.cpp).
//
// Each scene builds an identical setup a long way from the origin to show off the
// large-world (double-precision) coordinate path. The Rust side shifts every
// position back into the scene's base frame, so the float renderer works near the
// origin no matter how far out the content sits. Far Pyramid keeps its own exact
// STRIDE-11 pose + style-pair path (the reference-quality port); the other three
// share the generic `world_far_*` scene and the 16-float `vis` pose stride.

import * as THREE from "three";
import {
  attachInteraction,
  type InteractWasm,
  makeInteractAdapter,
  type ParamDef,
  type ParamValues,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm, type Box3dWasm } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { createButton, createCanvasOverlay } from "../controls.ts";
import { demoPage, makeStyleGate, runLoop } from "./common.ts";
import {
  applyShapeStyle,
  DemoScene,
  makeShapeMaterial,
  makeTriangleMesh,
  makeWireEdges,
  setView,
  trianglesFromWireframe,
} from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

type Scene = "far-stack" | "far-pyramid" | "far-ragdolls" | "far-mesh-drop";
export const SCENES: Scene[] = ["far-stack", "far-pyramid", "far-ragdolls", "far-mesh-drop"];

const PYRAMID_STRIDE = 11;
const POSE_STRIDE = 16;

/** C `FarStack::DrawControls` offset presets (`sample_world.cpp` :84-85). */
const STACK_PRESETS: { label: string; km: number }[] = [
  { label: "origin", km: 0 },
  { label: "10km", km: 10 },
  { label: "100km", km: 100 },
  { label: "1000km", km: 1000 },
  { label: "10000km", km: 10000 },
];

/** Human-readable Info-panel name per scene (C entry.Name). */
const SCENE_LABEL: Record<Scene, string> = {
  "far-stack": "Far Stack",
  "far-pyramid": "Far Pyramid",
  "far-ragdolls": "Far Ragdolls",
  "far-mesh-drop": "Far Mesh Drop",
};

/**
 * One InteractWasm adapter routing between the Far Pyramid exports (its own
 * scene/pose path) and the generic `world_far_*` exports used by the other three
 * scenes. Both halves are built by the shared factory (`makeInteractAdapter`),
 * which forwards the GLOBAL debug-flag mask + draw scales; a Proxy live-dispatches
 * each call to the active scene's adapter. Far Pyramid overrides `sim_debug_text`
 * to the empty payload (it carries no debug-text channel — byte-identical to the
 * reference port); the generic scenes forward their base-shifted labels.
 */
function worldInteract(wasm: Box3dWasm, getScene: () => Scene): InteractWasm {
  const generic = makeInteractAdapter(wasm, "world_far");
  const pyramid = makeInteractAdapter(wasm, "world_far_pyramid", {
    sim_debug_text: () => "[]",
  });
  const pick = (): InteractWasm => (getScene() === "far-pyramid" ? pyramid : generic);
  return new Proxy({} as InteractWasm, {
    get: (_t, prop) => pick()[prop as keyof InteractWasm],
  });
}

/** Absolute world base of each scene (camera readout coords), verified against
 *  the Rust builders in `world_demo/far.rs` / `world_demo/far_pyramid.rs`. */
function worldOriginFor(scene: Scene, stackOffsetKm: number): [number, number, number] {
  if (scene === "far-pyramid") return [10_000_000, 0, 0];
  if (scene === "far-stack") return [1000 * stackOffsetKm, 0, 0];
  if (scene === "far-ragdolls") return [1_000_000, 0, 0];
  return [1_000_000, 0, 1_000_000]; // far-mesh-drop
}

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  assertRouteScenes("world", SCENES);
  const { canvas, controls, page } = demoPage(
    container,
    "World",
    "Large-world samples from <code>sample_world.cpp</code>: identical scenes built " +
      "thousands of kilometres from the origin to exercise the double-precision path.",
    "Ctrl+click grab · Shift+click spawn · P/O/R",
    wasm.version(),
    { category: "World", samplesShell: true },
  );

  let scene: Scene =
    initialScene && SCENES.includes(initialScene as Scene) ? (initialScene as Scene) : "far-pyramid";
  const getScene = () => scene;
  let stackOffsetKm = wasm.is_double_precision_build() ? 10000 : 0;
  let ctrl!: SimControllerWithTick;

  const overlay = createCanvasOverlay(page);
  const dp = () => (wasm.is_double_precision_build() ? "ON" : "OFF");

  // --- Scene graph / render resources ---
  const demo = new DemoScene(canvas, {
    target: [0, 2, 0],
    distance: 20,
    fov: 50,
    shadowExtent: 60,
    gridSize: 80,
    gridDivisions: 40,
  });
  demo.camera.far = 800;
  demo.camera.updateProjectionMatrix();

  const pool = createMeshPool(); // Far Stack + Far Ragdolls (box / capsule bodies)
  // Awake-gated cache for the per-frame style fetch; reset per scene so a switch
  // never applies the previous scene's (differently sized) style snapshot.
  let styleGate = makeStyleGate<Uint32Array>();
  const boxGeo = new THREE.BoxGeometry(2, 2, 2);

  // Instanced-box path shared by Far Pyramid and Far Mesh Drop (many identical
  // boxes → one draw call, one representative engine style).
  let instances: THREE.InstancedMesh | null = null;
  let groundBox: THREE.Mesh | null = null; // Far Pyramid ground slab
  let groundTri: THREE.Mesh | null = null; // mesh-ground fill (ragdolls / mesh-drop)
  let groundWire: THREE.LineSegments | null = null;
  const instMat = makeShapeMaterial();
  const groundMat = makeShapeMaterial();

  const _m = new THREE.Matrix4();
  const _p = new THREE.Vector3();
  const _q = new THREE.Quaternion();
  const _s = new THREE.Vector3();

  function clearInstances() {
    if (instances) {
      demo.content.remove(instances);
      instances.dispose();
      instances = null;
    }
    if (groundBox) {
      demo.content.remove(groundBox);
      groundBox = null;
    }
  }
  function clearGroundMesh() {
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
  function clearVisuals() {
    clearInstances();
    clearGroundMesh();
    // Drain the shared mesh pool (Far Stack / Ragdolls) between scene switches.
    syncMeshesFromPoses(demo.content, pool, []);
  }

  function ensureInstances(count: number) {
    if (instances && instances.count >= count) return;
    if (instances) {
      demo.content.remove(instances);
      instances.dispose();
    }
    instances = new THREE.InstancedMesh(boxGeo, instMat, Math.max(count, 64));
    instances.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    instances.castShadow = true;
    instances.receiveShadow = true;
    demo.content.add(instances);
  }

  function buildGroundMesh() {
    clearGroundMesh();
    const wire = wasm.world_far_ground_wireframe();
    if (!wire.length) return;
    const positions = trianglesFromWireframe(wire);
    groundTri = makeTriangleMesh(positions, 0x8a94a6, 0.95);
    groundTri.receiveShadow = true;
    groundWire = makeWireEdges(wire, 0x4a5568, 0.4);
    demo.content.add(groundTri);
    demo.content.add(groundWire);
  }

  function setCameraForScene() {
    if (scene === "far-stack") setView(demo, 0, 8, 16, [0, 2, 0]);
    else if (scene === "far-pyramid") setView(demo, 40, -10, 60, [0, 20, 0]);
    else if (scene === "far-ragdolls") setView(demo, 180, 30, 20, [0, 0, 0]);
    else setView(demo, 0, 30, 20, [0, 0, 0]);
  }

  function resetScene() {
    if (scene === "far-stack") wasm.world_far_reset_stack(stackOffsetKm);
    else if (scene === "far-pyramid") wasm.world_reset_far_pyramid();
    else if (scene === "far-ragdolls") wasm.world_far_reset_ragdolls();
    else wasm.world_far_reset_mesh_drop();
  }

  function reset() {
    clearVisuals();
    resetScene();
    if (scene === "far-ragdolls" || scene === "far-mesh-drop") buildGroundMesh();
    setCameraForScene();
    updateStackControls();
    updateOverlayStatic();
    // Per-scene Info-panel name, camera-readout world origin, and shadow extent
    // (restored from the old standalone Far Pyramid page after the migration).
    ctrl.setSampleName(SCENE_LABEL[scene]);
    ctrl.setWorldOrigin(worldOriginFor(scene, stackOffsetKm));
    demo.setShadowExtent(scene === "far-pyramid" ? 80 : 60);
    styleGate = makeStyleGate<Uint32Array>();
  }

  // --- Per-scene HUD (C DrawTextLine) ---
  function updateOverlayStatic() {
    if (scene === "far-pyramid") {
      const km = wasm.world_far_pyramid_offset_km();
      overlay.innerHTML =
        `double precision: ${dp()}<br>pyramid built ${km.toFixed(0)} km from the world origin`;
    } else if (scene === "far-ragdolls") {
      overlay.innerHTML =
        `double precision: ${dp()}<br>20 ragdolls piled 1000 km from the world origin`;
    }
    // far-stack + far-mesh-drop refresh every frame (offset / failure readout).
  }

  function updateOverlayDynamic(poses: Float32Array) {
    if (scene === "far-stack") {
      const km = wasm.world_far_offset_km();
      // Top box is the last body; its base-relative y is the settled height (C
      // measures top.y = (worldCenter - base).y, `sample_world.cpp` :104-108).
      const n = Math.floor(poses.length / POSE_STRIDE);
      const topY = n > 0 ? poses[(n - 1) * POSE_STRIDE + 1]! : 0;
      overlay.innerHTML =
        `double precision: ${dp()}<br>world offset: ${km.toFixed(1)} km` +
        `<br>top box height above ground: ${topY.toFixed(4)} m`;
    } else if (scene === "far-mesh-drop") {
      const failed = wasm.world_far_mesh_drop_failed();
      overlay.innerHTML =
        `double precision: ${dp()}<br>mesh drop running 1000 km from the world origin` +
        (failed ? `<br><span style="color:#f87171">failed!</span>` : "");
    }
  }

  // --- Far Stack offset presets (only shown for that scene) ---
  const stackControls = document.createElement("div");
  stackControls.className = "control-group";
  const stackLabel = document.createElement("label");
  stackLabel.textContent = "World offset";
  stackControls.appendChild(stackLabel);
  const stackBtnRow = document.createElement("div");
  stackBtnRow.className = "control-row";
  for (const preset of STACK_PRESETS) {
    stackBtnRow.appendChild(
      createButton(preset.label, () => {
        stackOffsetKm = preset.km;
        reset();
      }),
    );
  }
  stackControls.appendChild(stackBtnRow);
  function updateStackControls() {
    stackControls.style.display = scene === "far-stack" ? "" : "none";
  }

  // --- Scene selector (declarative param, restarts on change) ---
  const params: ParamDef[] = [
    {
      type: "select",
      key: "scene",
      label: "Sample",
      options: [
        { label: "Far Stack", value: "far-stack" },
        { label: "Far Pyramid", value: "far-pyramid" },
        { label: "Far Ragdolls", value: "far-ragdolls" },
        { label: "Far Mesh Drop", value: "far-mesh-drop" },
      ],
      default: scene,
      restart: true,
    },
  ];

  ctrl = attachInteraction({
    wasm: worldInteract(wasm, getScene),
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: SCENE_LABEL[scene],
    sampleCategory: "World",
    params,
    onParamsChange: (values: ParamValues, key: string) => {
      if (key === "scene") scene = values.scene as Scene;
    },
  }) as SimControllerWithTick;

  controls.appendChild(stackControls);
  reset();

  // Far Pyramid consumes only the [ground, box] style pair (2 words) rather than
  // the full engine style array — feature-checked so the demo keeps building.
  const stylePairFn = (
    wasm as unknown as { world_far_pyramid_style_pair?: () => Uint32Array }
  ).world_far_pyramid_style_pair?.bind(wasm);

  // --- Per-scene renderers ---
  function renderPyramid() {
    const poses = wasm.world_far_pyramid_poses();
    const awake = wasm.world_far_pyramid_counters()[5] ?? 0;
    const styles = styleGate(awake, () =>
      stylePairFn ? stylePairFn() : wasm.world_far_pyramid_styles(),
    );
    const n = Math.floor(poses.length / PYRAMID_STRIDE);
    const dyn = Math.max(0, n - 1);
    ensureInstances(dyn);
    if (instances && n > 1) applyShapeStyle(instances, styles[1]!);

    if (n > 0) {
      if (!groundBox) {
        groundBox = new THREE.Mesh(boxGeo, groundMat);
        groundBox.receiveShadow = true;
        demo.content.add(groundBox);
      }
      applyShapeStyle(groundBox, styles[0]!);
      groundBox.position.set(poses[0]!, poses[1]!, poses[2]!);
      _q.set(poses[3]!, poses[4]!, poses[5]!, poses[6]!);
      groundBox.quaternion.copy(_q);
      groundBox.scale.set(poses[7]!, poses[8]!, poses[9]!);
    }
    for (let i = 1; i < n; i++) {
      const o = i * PYRAMID_STRIDE;
      _p.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
      _q.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
      _s.set(poses[o + 7]!, poses[o + 8]!, poses[o + 9]!);
      _m.compose(_p, _q, _s);
      instances!.setMatrixAt(i - 1, _m);
    }
    if (instances) {
      instances.count = dyn;
      instances.instanceMatrix.needsUpdate = true;
    }
  }

  function renderMeshDrop() {
    const poses = wasm.world_far_poses();
    const awake = wasm.world_far_counters()[5] ?? 0;
    const styles = styleGate(awake, () => wasm.world_far_styles());
    const n = Math.floor(poses.length / POSE_STRIDE);
    ensureInstances(n);
    if (instances && n > 0) applyShapeStyle(instances, styles[0]!);
    for (let i = 0; i < n; i++) {
      const o = i * POSE_STRIDE;
      _p.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
      _q.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
      _s.set(poses[o + 7]!, poses[o + 8]!, poses[o + 9]!);
      _m.compose(_p, _q, _s);
      instances!.setMatrixAt(i, _m);
    }
    if (instances) {
      instances.count = n;
      instances.instanceMatrix.needsUpdate = true;
    }
    updateOverlayDynamic(poses);
  }

  function renderPool() {
    const poses = wasm.world_far_poses();
    const awake = wasm.world_far_counters()[5] ?? 0;
    const styles = styleGate(awake, () => wasm.world_far_styles());
    syncMeshesFromPoses(demo.content, pool, poses, { styles });
    updateOverlayDynamic(poses);
  }

  const stop = runLoop(() => {
    ctrl.tickFrame();
    if (scene === "far-pyramid") renderPyramid();
    else if (scene === "far-mesh-drop") renderMeshDrop();
    else renderPool(); // far-stack + far-ragdolls
    demo.render();
  }, controls);

  return () => {
    ctrl.dispose();
    stop();
    clearVisuals();
    disposeMeshPool(pool);
    demo.dispose();
    boxGeo.dispose();
    instMat.dispose();
    groundMat.dispose();
  };
}
