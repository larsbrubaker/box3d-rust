// Continuous — the full sample_continuous.cpp suite: Thin Wall, Bounce House,
// Spinning Stick, Bullet vs Stack, Needle Mesh, Mesh Drop, Mesh Drop Unit Test,
// Hump Mesh, Is Fast, and Stall. Fast bodies exercising continuous collision.
//
// Parametric kinds 0/1/3 (box/sphere/cylinder) render as per-kind InstancedMeshes
// with unit geometries + matrix scale and per-instance engine-style colors
// (applyInstancedStyles), matching the Stacking / Benchmark path. Capsules
// (kind 2) keep individual meshes — non-uniform scale on a unit capsule is wrong.

import * as THREE from "three";
import {
  createButton,
  createButtonGroup,
  createInfoBox,
  createSlider,
} from "../controls.ts";
import {
  attachInteraction,
  type ParamValues,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { demoPage, makeStyleGate, runLoop } from "./common.ts";
import {
  applyInstancedStyles,
  applyShapeStyle,
  DemoScene,
  makeShapeMaterial,
  makeTriangleMesh,
  makeWireEdges,
  setView,
  trianglesFromWireframe,
} from "../three-scene.ts";

/** `[px..qw, hx,hy,hz, kind, bodyType, awake]` */
const STRIDE = 13;

/** Above this segment count the ground wire is skipped (dense Stall torus). */
const MAX_WIRE_SEGMENTS = 40000;

type Mode =
  | "thin"
  | "bounce"
  | "spin"
  | "bullet"
  | "needle"
  | "mesh-drop"
  | "mesh-drop-unit"
  | "hump"
  | "is-fast"
  | "stall";

export const SCENES: Mode[] = [
  "thin",
  "bounce",
  "spin",
  "bullet",
  "needle",
  "mesh-drop",
  "mesh-drop-unit",
  "hump",
  "is-fast",
  "stall",
];

const CONTINUOUS_NAMES: Record<Mode, string> = {
  thin: "Thin Wall",
  bounce: "Bounce House",
  spin: "Spinning Stick",
  bullet: "Bullet vs Stack",
  needle: "Needle Mesh",
  "mesh-drop": "Mesh Drop",
  "mesh-drop-unit": "Mesh Drop Unit Test",
  hump: "Hump Mesh",
  "is-fast": "Is Fast",
  stall: "Stall",
};

// C SetView(yaw, pitch, distance, target) per sample.
const CAMERAS: Record<Mode, [number, number, number, [number, number, number]]> = {
  thin: [45, 30, 30, [0, 0, 0]],
  bounce: [45, 45, 50, [0, 0, 0]],
  spin: [45, 25, 20, [0, 2, 0]],
  bullet: [15, 20, 30, [0, 2, 0]],
  needle: [45, 25, 4, [0, 1.2, 0]],
  "mesh-drop": [0, 30, 20, [0, 0, 0]],
  "mesh-drop-unit": [0, 30, 20, [0, 0, 0]],
  hump: [45, 25, 10, [0, 1.2, 0]],
  "is-fast": [0, 15, 50, [0, 15, 0]],
  stall: [130, 15, 15, [0, 2, 0]],
};

/** Kinds that share a unit geometry and go through InstancedMesh + matrix scale. */
function isInstancedKind(kind: number): boolean {
  return kind === 0 || kind === 1 || kind === 3;
}

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  // Self-check the scene table against the registry (see registry.ts pattern).
  assertRouteScenes("continuous", SCENES);
  const { canvas, controls } = demoPage(
    container,
    "Continuous",
    "Official Continuous samples from <code>sample_continuous.cpp</code> — fast bodies with " +
      "continuous collision: Thin Wall, Bounce House, Spinning Stick, Bullet vs Stack, Needle Mesh, " +
      "Mesh Drop (+ Unit Test), Hump Mesh, Is Fast, and Stall.",
    "Ctrl+click grab · Shift+click spawn · L launch (Bullet/Stall) · P/O/R",
    wasm.version(),
    { category: "Continuous", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "<strong>Spinning Stick</strong> spins at a random ω (C <code>RandomVec3</code>, seed 12345). " +
        "<strong>Needle/Hump/Stall</strong> drop bodies onto static collision meshes (rendered as " +
        "wireframe). <strong>Mesh Drop</strong> rains 1024 tiny shapes into a wave basin — 1024 " +
        "continuous bodies runs below 60 fps in serial WASM; its RNG seed varies per Generate (C " +
        "seeds from ticks), and Auto Generate regenerates on settle. <strong>Stall</strong> fires a " +
        "600 m/s rock at a 200×200 torus, with the C CCD stall threshold (1.0 ms) set and shown.",
    ),
  );

  let mode: Mode =
    initialScene && SCENES.includes(initialScene as Mode) ? (initialScene as Mode) : "thin";

  // Per-scene control panel (Launch buttons, Mesh Drop combo/slider/buttons).
  const sceneControls = document.createElement("div");
  controls.appendChild(sceneControls);

  // Mesh Drop state.
  let meshDropShape = 0; // 0 box, 1 capsule, 2 cylinder, 3 sphere
  let meshDropAmplitude = 0.5;
  let autoGenerate = false;
  let sawMovement = false;

  const demo = new DemoScene(canvas, {
    target: [0, 10, 0],
    distance: 30,
    shadowExtent: 48,
  });
  // Awake-gated cache for sim_body_styles() (a full world_draw capture): refetch
  // only while bodies are awake, on the first frame, and once on the settle so
  // sleep recoloring is still captured. sim_counters()[5] = awake dynamic count.
  const styleGate = makeStyleGate<Uint32Array>();

  // Ground mesh wireframe (Needle / Mesh Drop / Hump / Stall).
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
    const wire = wasm.sim_cont_ground_wireframe();
    if (!wire.length) return;
    const positions = trianglesFromWireframe(wire);
    groundTri = makeTriangleMesh(positions, 0x8a94a6, 0.9);
    groundTri.receiveShadow = true;
    demo.content.add(groundTri);
    // The dense Stall torus has too many edges to draw as line segments.
    if (wire.length / 6 <= MAX_WIRE_SEGMENTS) {
      groundWire = makeWireEdges(wire, 0x4a5568, 0.35);
      demo.content.add(groundWire);
    }
  }

  // --- Individual meshes for capsules (kind 2) ---
  const meshes: (THREE.Mesh | null)[] = [];

  // --- InstancedMesh path for kinds 0 / 1 / 3 (unit geo + matrix scale) ---
  const boxGeo = new THREE.BoxGeometry(2, 2, 2);
  const sphereGeo = new THREE.SphereGeometry(1, 20, 14);
  const cylGeo = new THREE.CylinderGeometry(1, 1, 2, 16);
  const kindGeo: Record<number, THREE.BufferGeometry> = {
    0: boxGeo,
    1: sphereGeo,
    3: cylGeo,
  };
  const instByKind = new Map<number, THREE.InstancedMesh>();
  const matByKind = new Map<number, THREE.MeshStandardMaterial>();

  const _m = new THREE.Matrix4();
  const _p = new THREE.Vector3();
  const _q = new THREE.Quaternion();
  const _s = new THREE.Vector3();
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

  function disposeMesh(m: THREE.Mesh) {
    demo.content.remove(m);
    m.geometry.dispose();
    (m.material as THREE.Material).dispose();
  }

  function clearMeshes() {
    for (const m of meshes) {
      if (m) disposeMesh(m);
    }
    meshes.length = 0;
  }

  function setCameraForMode() {
    const [yaw, pitch, dist, target] = CAMERAS[mode];
    setView(demo, yaw, pitch, dist, target);
  }

  /** Individual mesh path for capsule (2) only. */
  function ensureMesh(i: number, kind: number): THREE.Mesh {
    let mesh = meshes[i];

    // kind === 2 (capsule)
    if (!mesh || mesh.userData.kind !== 2) {
      if (mesh) disposeMesh(mesh);
      mesh = new THREE.Mesh(new THREE.CapsuleGeometry(0.2, 0.5, 4, 10), makeShapeMaterial());
      mesh.userData.kind = 2;
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      demo.content.add(mesh);
      meshes[i] = mesh;
    }
    return mesh;
  }

  function meshDropControls(): HTMLElement[] {
    const els: HTMLElement[] = [];
    // C `DrawControls` Combo "Type": box / capsule / cylinder / sphere.
    els.push(
      createButtonGroup(
        [
          { label: "box", value: "0" },
          { label: "capsule", value: "1" },
          { label: "cylinder", value: "2" },
          { label: "sphere", value: "3" },
        ],
        String(meshDropShape),
        (v) => {
          meshDropShape = parseInt(v, 10);
          sawMovement = false;
          wasm.sim_cont_mesh_drop_set_type(meshDropShape);
        },
      ),
    );
    // C `DrawControls` SliderFloat "Amplitude" 0..1 — rebuilds ground + grid.
    els.push(
      createSlider("Amplitude", 0, 1, meshDropAmplitude, 0.05, (v) => {
        meshDropAmplitude = v;
        sawMovement = false;
        wasm.sim_cont_mesh_drop_set_amplitude(v);
        buildGround();
      }),
    );
    const row = document.createElement("div");
    row.className = "control-row";
    // C `DrawControls` Button "Generate".
    row.appendChild(
      createButton("Generate", () => {
        sawMovement = false;
        // C `MeshDrop::Generate` seeds from `b3GetTicks()`; a performance.now()-derived
        // u32 is the browser equivalent (any tick value is faithful).
        wasm.sim_cont_mesh_drop_generate(performance.now() >>> 0);
      }),
    );
    // C `DrawControls` Button "Auto Generate" (toggles m_autoGenerate).
    const autoBtn = createButton("Auto Generate", () => {
      autoGenerate = !autoGenerate;
      sawMovement = false;
      autoBtn.classList.toggle("active", autoGenerate);
    });
    autoBtn.classList.toggle("active", autoGenerate);
    row.appendChild(autoBtn);
    els.push(row);
    return els;
  }

  function syncSceneControls() {
    sceneControls.replaceChildren();
    if (mode === "bullet") {
      const row = document.createElement("div");
      row.className = "control-group";
      row.appendChild(createButton("Launch (L)", () => wasm.sim_launch_bullet()));
      sceneControls.appendChild(row);
    } else if (mode === "stall") {
      const row = document.createElement("div");
      row.className = "control-group";
      row.appendChild(createButton("Launch (L)", () => wasm.sim_cont_stall_launch()));
      sceneControls.appendChild(row);
      // C `Stall` ctor sets b3SetStallThreshold(0.001f) to log slow CCD steps.
      const readout = document.createElement("div");
      readout.className = "sample-stat";
      readout.textContent = `CCD stall threshold: ${wasm.sim_cont_stall_threshold_ms().toFixed(1)} ms`;
      sceneControls.appendChild(readout);
    } else if (mode === "mesh-drop") {
      for (const el of meshDropControls()) sceneControls.appendChild(el);
    }
  }

  function reset() {
    clearInstanced();
    clearMeshes();
    autoGenerate = false;
    sawMovement = false;
    switch (mode) {
      case "thin":
        wasm.sim_reset_thin_wall();
        break;
      case "bounce":
        wasm.sim_reset_bounce_house();
        break;
      case "spin":
        wasm.sim_reset_spinning_stick();
        break;
      case "bullet":
        wasm.sim_reset_bullet_vs_stack();
        break;
      case "needle":
        wasm.sim_reset_needle_mesh();
        break;
      case "mesh-drop":
        meshDropShape = 0;
        meshDropAmplitude = 0.5;
        wasm.sim_cont_mesh_drop_set_type(0);
        wasm.sim_cont_mesh_drop_set_amplitude(0.5);
        wasm.sim_reset_mesh_drop();
        break;
      case "mesh-drop-unit":
        wasm.sim_reset_mesh_drop_unit();
        break;
      case "hump":
        wasm.sim_reset_hump_mesh();
        break;
      case "is-fast":
        wasm.sim_reset_is_fast();
        break;
      case "stall":
        wasm.sim_reset_stall();
        break;
    }
    buildGround();
    syncSceneControls();
    setCameraForMode();
  }

  const onKeyDown = (e: KeyboardEvent) => {
    if ((e.key === "l" || e.key === "L") && !e.repeat) {
      if (mode === "bullet") {
        e.preventDefault();
        wasm.sim_launch_bullet();
      } else if (mode === "stall") {
        e.preventDefault();
        wasm.sim_cont_stall_launch();
      }
    }
  };
  window.addEventListener("keydown", onKeyDown);
  canvas.tabIndex = 0;
  canvas.style.outline = "none";

  const ctrl = attachInteraction({
    wasm,
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: CONTINUOUS_NAMES[mode],
    sampleCategory: "Continuous",
    params: [
      {
        type: "select",
        key: "sample",
        label: "Sample",
        options: SCENES.map((s) => ({ label: CONTINUOUS_NAMES[s], value: s })),
        default: mode,
        restart: true,
      },
    ],
    onParamsChange: (values: ParamValues, key: string) => {
      if (key !== "sample") return;
      mode = String(values.sample) as Mode;
      const nameEl = controls.querySelector(".sample-name");
      if (nameEl) nameEl.textContent = CONTINUOUS_NAMES[mode];
    },
  }) as SimControllerWithTick;

  reset();

  const quat = new THREE.Quaternion();
  // Tracks the last style buffer the gate handed back. When the pile is settled
  // the gate returns the very same cached array (referential identity), so an
  // unchanged reference means the per-instance colors need no re-upload this frame.
  let prevStyles: Uint32Array | undefined;
  const stop = runLoop(() => {
    const stepped = ctrl.tickFrame();
    // Mesh Drop "Auto Generate": regenerate once the pile settles (moveCount == 0),
    // mirroring C `MeshDrop::Step` (the fast-forward is elided for a live view).
    if (stepped && mode === "mesh-drop" && autoGenerate) {
      const moving = wasm.sim_cont_mesh_drop_move_count();
      if (moving > 0) sawMovement = true;
      else if (sawMovement) {
        wasm.sim_cont_mesh_drop_generate(performance.now() >>> 0);
        sawMovement = false;
      }
    }
    const poses = wasm.sim_body_poses();
    const awake = wasm.sim_counters()[5] ?? 0;
    const styles = styleGate(awake, () => wasm.sim_body_styles());
    const stylesChanged = styles !== prevStyles;
    prevStyles = styles;
    const n = Math.floor(poses.length / STRIDE);
    while (meshes.length > n) {
      const m = meshes.pop();
      if (m) disposeMesh(m);
    }

    for (const arr of bucketMats.values()) arr.length = 0;
    for (const arr of bucketStyles.values()) arr.length = 0;

    for (let i = 0; i < n; i++) {
      const o = i * STRIDE;
      const kind = poses[o + 10]!;

      if (isInstancedKind(kind)) {
        // Slot flipped from capsule mesh → instanced: dispose the stale mesh.
        const stale = meshes[i];
        if (stale) {
          disposeMesh(stale);
          meshes[i] = null;
        }
        _p.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
        _q.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
        const a0 = poses[o + 7]!;
        const a1 = poses[o + 8]!;
        const a2 = poses[o + 9]!;
        if (kind === 0) _s.set(a0, a1, a2);
        else if (kind === 3) _s.set(a0, a1, a0); // radius, halfLen, radius on unit cyl
        else _s.set(a0, a0, a0); // sphere
        _m.compose(_p, _q, _s);

        let mats = bucketMats.get(kind);
        if (!mats) {
          mats = [];
          bucketMats.set(kind, mats);
        }
        let sts = bucketStyles.get(kind);
        if (!sts) {
          sts = [];
          bucketStyles.set(kind, sts);
        }
        mats.push(_m.clone());
        sts.push(styles[i] ?? 0);
        continue;
      }

      // Capsule (2) — individual mesh.
      const mesh = ensureMesh(i, kind);
      applyShapeStyle(mesh, styles[i]!);
      mesh.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
      quat.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
      mesh.quaternion.copy(quat);
      // kind === 2 capsule: rebuild geo when radius / half-length drift.
      const radius = poses[o + 7]!;
      const halfLen = poses[o + 8]!;
      const geo = mesh.geometry as THREE.CapsuleGeometry;
      if (
        Math.abs(geo.parameters.radius - radius) > 1e-4 ||
        Math.abs(geo.parameters.length - Math.max(1e-4, halfLen * 2)) > 1e-3
      ) {
        geo.dispose();
        mesh.geometry = new THREE.CapsuleGeometry(radius, Math.max(1e-4, halfLen * 2), 4, 10);
      }
      mesh.scale.set(1, 1, 1);
    }

    // Hide any instanced kind not present this frame; fill the rest.
    for (const [kind, inst] of instByKind) {
      const mats = bucketMats.get(kind);
      if (!mats || mats.length === 0) {
        inst.count = 0;
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

    demo.render();
  }, controls);

  return () => {
    window.removeEventListener("keydown", onKeyDown);
    ctrl.dispose();
    stop();
    clearInstanced();
    clearMeshes();
    clearGround();
    demo.dispose();
    boxGeo.dispose();
    sphereGeo.dispose();
    cylGeo.dispose();
    for (const mat of matByKind.values()) mat.dispose();
  };
}
