// Continuous — the full sample_continuous.cpp suite: Thin Wall, Bounce House,
// Spinning Stick, Bullet vs Stack, Needle Mesh, Mesh Drop, Mesh Drop Unit Test,
// Hump Mesh, Is Fast, and Stall. Fast bodies exercising continuous collision.

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
        "600 m/s rock at a 200×200 torus (the C stall-threshold readout is not exposed by the port).",
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

  // Each mesh owns its material — engine style words color bodies per-body.
  const meshes: THREE.Mesh[] = [];
  const boxGeo = new THREE.BoxGeometry(2, 2, 2);
  const sphereGeo = new THREE.SphereGeometry(1, 20, 14);

  function disposeMesh(m: THREE.Mesh) {
    demo.content.remove(m);
    if (m.geometry !== boxGeo && m.geometry !== sphereGeo) m.geometry.dispose();
    (m.material as THREE.Material).dispose();
  }

  function clearMeshes() {
    for (const m of meshes) disposeMesh(m);
    meshes.length = 0;
  }

  function setCameraForMode() {
    const [yaw, pitch, dist, target] = CAMERAS[mode];
    setView(demo, yaw, pitch, dist, target);
  }

  function ensureMesh(i: number, kind: number, bodyType: number): THREE.Mesh {
    let mesh = meshes[i];
    const wantSphere = kind === 1;
    const wantCapsule = kind === 2;
    const wantCylinder = kind === 3;

    if (wantCapsule) {
      if (!mesh || mesh.geometry.type !== "CapsuleGeometry") {
        if (mesh) disposeMesh(mesh);
        mesh = new THREE.Mesh(new THREE.CapsuleGeometry(0.2, 0.5, 4, 10), makeShapeMaterial());
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        demo.content.add(mesh);
        meshes[i] = mesh;
      }
      return mesh;
    }

    if (wantCylinder) {
      if (!mesh || mesh.geometry.type !== "CylinderGeometry") {
        if (mesh) disposeMesh(mesh);
        mesh = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.4, 16), makeShapeMaterial());
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        demo.content.add(mesh);
        meshes[i] = mesh;
      }
      return mesh;
    }

    if (
      !mesh ||
      (wantSphere && mesh.geometry !== sphereGeo) ||
      (!wantSphere && mesh.geometry !== boxGeo)
    ) {
      if (mesh) disposeMesh(mesh);
      mesh = new THREE.Mesh(wantSphere ? sphereGeo : boxGeo, makeShapeMaterial());
      mesh.castShadow = bodyType !== 0;
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
        wasm.sim_cont_mesh_drop_generate();
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
    } else if (mode === "mesh-drop") {
      for (const el of meshDropControls()) sceneControls.appendChild(el);
    }
  }

  function reset() {
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
  const stop = runLoop(() => {
    const stepped = ctrl.tickFrame();
    // Mesh Drop "Auto Generate": regenerate once the pile settles (moveCount == 0),
    // mirroring C `MeshDrop::Step` (the fast-forward is elided for a live view).
    if (stepped && mode === "mesh-drop" && autoGenerate) {
      const moving = wasm.sim_cont_mesh_drop_move_count();
      if (moving > 0) sawMovement = true;
      else if (sawMovement) {
        wasm.sim_cont_mesh_drop_generate();
        sawMovement = false;
      }
    }
    const poses = wasm.sim_body_poses();
    const awake = wasm.sim_counters()[5] ?? 0;
    const styles = styleGate(awake, () => wasm.sim_body_styles());
    const n = Math.floor(poses.length / STRIDE);
    while (meshes.length > n) {
      disposeMesh(meshes.pop()!);
    }
    for (let i = 0; i < n; i++) {
      const o = i * STRIDE;
      const kind = poses[o + 10]!;
      const bodyType = poses[o + 11]! | 0;
      const mesh = ensureMesh(i, kind, bodyType);
      applyShapeStyle(mesh, styles[i]!);
      mesh.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
      quat.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
      mesh.quaternion.copy(quat);
      if (kind === 2) {
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
      } else if (kind === 3) {
        const radius = poses[o + 7]!;
        const halfLen = poses[o + 8]!;
        const geo = mesh.geometry as THREE.CylinderGeometry;
        const h = Math.max(1e-4, halfLen * 2);
        if (
          Math.abs(geo.parameters.radiusTop - radius) > 1e-4 ||
          Math.abs(geo.parameters.height - h) > 1e-3
        ) {
          geo.dispose();
          mesh.geometry = new THREE.CylinderGeometry(radius, radius, h, 16);
        }
        mesh.scale.set(1, 1, 1);
      } else if (kind === 1) {
        mesh.scale.setScalar(poses[o + 7]!);
      } else {
        mesh.scale.set(poses[o + 7]!, poses[o + 8]!, poses[o + 9]!);
      }
    }
    demo.render();
  }, controls);

  return () => {
    window.removeEventListener("keydown", onKeyDown);
    ctrl.dispose();
    stop();
    clearMeshes();
    clearGround();
    demo.dispose();
    boxGeo.dispose();
    sphereGeo.dispose();
  };
}
