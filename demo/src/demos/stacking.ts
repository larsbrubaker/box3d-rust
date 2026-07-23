// Stacking — the full sample_stacking.cpp roster. Batch 1: Jenga, Box Stack,
// Pyramid2D (planar), Sphere Stack, Single Box. Batch 3b adds Card House,
// Capsule Stack, Cylinder, Cylinder Stack, Dominoes, Wedge, Arch, and
// Double Domino — all driven by the ported World::step scalar solver.
//
// Parametric kinds 0/1/3 (box/sphere/cylinder) render as per-kind InstancedMeshes
// with unit geometries + matrix scale and per-instance engine-style colors
// (applyInstancedStyles), matching the Benchmark pile path. Capsules (kind 2)
// keep individual meshes — non-uniform scale on a unit capsule is wrong — and
// convex hulls (kind 4) keep per-body BufferGeometries from sim_hull_geometry().

import * as THREE from "three";
import { createButtonGroup, createCanvasOverlay, createInfoBox, fmt2g } from "../controls.ts";
import {
  attachInteraction,
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
  setView,
} from "../three-scene.ts";

/** `[px..qw, hx,hy,hz, kind, bodyType, awake]` */
const STRIDE = 13;

type Mode =
  | "jenga"
  | "boxes"
  | "pyramid"
  | "spheres"
  | "single"
  | "card-house"
  | "capsule-stack"
  | "cylinder"
  | "cylinder-stack"
  | "dominoes"
  | "wedge"
  | "arch"
  | "double-domino";

export const SCENES: Mode[] = [
  "jenga",
  "boxes",
  "pyramid",
  "spheres",
  "single",
  "card-house",
  "capsule-stack",
  "cylinder",
  "cylinder-stack",
  "dominoes",
  "wedge",
  "arch",
  "double-domino",
];

/** Scenes whose bodies are arbitrary convex hulls (render kind 4). Their per-body
 *  triangle geometry comes from `sim_hull_geometry()`; everything else is a
 *  parametric box/sphere/capsule. */
const HULL_MODES = new Set<Mode>(["cylinder", "cylinder-stack", "wedge", "arch"]);

/** Kinds that share a unit geometry and go through InstancedMesh + matrix scale. */
function isInstancedKind(kind: number): boolean {
  return kind === 0 || kind === 1 || kind === 3;
}

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  // Self-check the scene table against the registry (see registry.ts pattern).
  assertRouteScenes("stacking", SCENES);
  const { canvas, controls, page } = demoPage(
    container,
    "Stacking",
    "The full official Stacking roster — Jenga, Box Stack, Pyramid2D (planar), Sphere/Capsule " +
      "stacks, Card House, Cylinder + Cylinder Stack, Dominoes, Wedge, Arch, and Double " +
      "Domino — driven by the ported <code>World::step</code> scalar solver.",
    "Ctrl+click grab · Shift+click spawn · click select · P/O/R",
    wasm.version(),
    { category: "Stacking", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "Exact C sample counts and materials. <strong>Dominoes</strong> (30 rings) and " +
        "<strong>Double Domino</strong> kick off with the C linear impulse at reset. " +
        "<strong>Cylinder</strong>/<strong>Cylinder Stack</strong>/<strong>Wedge</strong>/" +
        "<strong>Arch</strong> render their true convex hulls. " +
        "<strong>Pyramid2D and Capsule Stack are intentionally planar</strong>: C locks the Z " +
        "axis (and angular axes) so collapse stays in-plane — not a 3D engine bug. " +
        "<strong>Dominoes / timing:</strong> this demo runs the serial scalar solver — for an " +
        "honest comparison with C, build Box3D with <code>BOX3D_DISABLE_SIMD=ON</code> and " +
        "workers=1 (C defaults to workers + SIMD).",
    ),
  );

  // Jenga is the 3D showcase; Pyramid2D must never be the default (it looks like a 2D sim).
  // A deep link (`#/stacking/<slug>`) can request a specific scene via initialScene.
  let mode: Mode =
    initialScene && SCENES.includes(initialScene as Mode) ? (initialScene as Mode) : "jenga";
  // C JengaStack DrawControls radio: Hull (default) or Capsule.
  let jengaShape: "hull" | "capsule" = "hull";

  // Single Box HUD readout (C SingleBox::Step draws "(x, y, z) = ...").
  const hud = createCanvasOverlay(page);
  hud.style.display = "none";

  const demo = new DemoScene(canvas, { target: [0, 2, 0], distance: 14, shadowExtent: 36 });
  // Awake-gated cache for sim_body_styles() (a full world_draw capture): refetch
  // only while bodies are awake, on the first frame, and once on the settle so
  // sleep recoloring is still captured. sim_counters()[5] = awake dynamic count.
  const styleGate = makeStyleGate<Uint32Array>();

  // --- Individual meshes for capsules (kind 2) and hulls (kind 4) ---
  const meshes: (THREE.Mesh | null)[] = [];
  // Per-body convex-hull geometries (kind 4), index-aligned to the pose list.
  // Owned here (not by the meshes) so a hull mesh recycle never double-frees them.
  const hullGeos: (THREE.BufferGeometry | null)[] = [];
  let hullCount = -1;

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
    // kind-4 hull meshes borrow a geometry owned by `hullGeos`; never dispose it here.
    if (m.userData.kind !== 4) {
      m.geometry.dispose();
    }
    (m.material as THREE.Material).dispose();
  }

  function clearMeshes() {
    for (const m of meshes) {
      if (m) disposeMesh(m);
    }
    meshes.length = 0;
  }

  function disposeHullGeos() {
    for (const g of hullGeos) g?.dispose();
    hullGeos.length = 0;
  }

  /** Rebuild `hullGeos` from `sim_hull_geometry()`. Layout: per body a `floatCount`
   *  then that many triangle-vertex floats (`0` for non-hull bodies). */
  function loadHullGeometry() {
    disposeHullGeos();
    const raw = wasm.sim_hull_geometry();
    let p = 0;
    let idx = 0;
    while (p < raw.length) {
      const count = raw[p++]! | 0;
      if (count > 0) {
        const positions = new Float32Array(count);
        for (let k = 0; k < count; k++) positions[k] = raw[p++]!;
        const geo = new THREE.BufferGeometry();
        geo.setAttribute("position", new THREE.BufferAttribute(positions, 3));
        geo.computeVertexNormals();
        hullGeos[idx] = geo;
      } else {
        hullGeos[idx] = null;
      }
      idx++;
    }
  }

  function setCameraForMode() {
    // C SetView(yaw, pitch, distance, target) values from sample_stacking.cpp.
    switch (mode) {
      case "single":
        setView(demo, 0, 25, 10, [0, 0, 0]); // SingleBox :278
        break;
      case "boxes":
        setView(demo, 40, 15, 50, [0, 20, 0]); // BoxStack :356 (c52908c)
        break;
      case "pyramid":
        setView(demo, 40, 15, 50, [0, 12, 0]); // Pyramid2D :814 (c52908c)
        break;
      case "spheres":
        setView(demo, 0, 15, 50, [0, 10, 0]); // SphereStack :173
        break;
      case "card-house":
        setView(demo, 30, 10, 3, [0.75, 1.0, 0.4]); // CardHouse :100
        break;
      case "capsule-stack":
        setView(demo, 0, 15, 50, [0, 10, 0]); // CapsuleStack :234
        break;
      case "cylinder":
        setView(demo, 0, 15, 10, [0, 0, 0]); // Cylinder :323
        break;
      case "cylinder-stack":
        setView(demo, 0, 15, 15, [0, 5, 0]); // CylinderStack :374
        break;
      case "dominoes":
        setView(demo, 0, 15, 75, [0, 0, 0]); // Dominoes :588 (release)
        break;
      case "wedge":
        setView(demo, 75, 10, 10, [0, 0, 0]); // Wedge :654
        break;
      case "arch":
        setView(demo, 25, 10, 30, [0, 5, 0]); // Arch :742
        break;
      case "double-domino":
        setView(demo, 0, 15, 15, [0, 0.5, 1]); // DoubleDomino :849
        break;
      default:
        setView(demo, 35, 15, 30, [0, 10, 0]); // JengaStack :493
        break;
    }
  }

  /** Individual mesh path for capsule (2) and hull (4) only. */
  function ensureMesh(i: number, kind: number): THREE.Mesh {
    let mesh = meshes[i];

    if (kind === 4) {
      const geo = hullGeos[i] ?? null;
      if (!mesh || mesh.userData.kind !== 4 || mesh.userData.hullGeo !== geo) {
        if (mesh) disposeMesh(mesh);
        mesh = new THREE.Mesh(geo ?? boxGeo, makeShapeMaterial());
        mesh.userData.kind = 4;
        mesh.userData.hullGeo = geo;
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        demo.content.add(mesh);
        meshes[i] = mesh;
      }
      return mesh;
    }

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

  function reset() {
    clearInstanced();
    clearMeshes();
    disposeHullGeos();
    hullCount = -1;
    hud.style.display = mode === "single" ? "" : "none";
    // Fixed C sample counts (all light enough for serial wasm).
    switch (mode) {
      case "single":
        wasm.sim_reset_single_box();
        break;
      case "boxes":
        wasm.sim_reset_stacking();
        break;
      case "jenga":
        wasm.sim_reset_jenga(jengaShape === "capsule" ? 1 : 0);
        break;
      case "pyramid":
        wasm.sim_reset_pyramid();
        break;
      case "spheres":
        wasm.sim_reset_sphere_stack();
        break;
      case "card-house":
        wasm.sim_reset_card_house();
        break;
      case "capsule-stack":
        wasm.sim_reset_capsule_stack();
        break;
      case "cylinder":
        wasm.sim_reset_cylinder();
        break;
      case "cylinder-stack":
        wasm.sim_reset_cylinder_stack();
        break;
      case "dominoes":
        wasm.sim_reset_dominoes();
        break;
      case "wedge":
        wasm.sim_reset_wedge();
        break;
      case "arch":
        wasm.sim_reset_arch();
        break;
      case "double-domino":
        wasm.sim_reset_double_domino();
        break;
    }
    // C GetGuiDraw()->forceScale: Cylinder = 0.01, Cylinder Stack = 0.001, else 1.
    if (mode === "cylinder") wasm.sim_set_draw_scales(1, 0.01);
    else if (mode === "cylinder-stack") wasm.sim_set_draw_scales(1, 0.001);
    else wasm.sim_set_draw_scales(1, 1);
    setCameraForMode();
  }

  // C JengaStack DrawControls Capsule/Hull radio — only shown while Jenga is active.
  const jengaControls = document.createElement("div");
  jengaControls.style.display = "none";
  jengaControls.appendChild(
    createButtonGroup(
      [
        { label: "Hull", value: "hull" },
        { label: "Capsule", value: "capsule" },
      ],
      "hull",
      (v) => {
        jengaShape = v as "hull" | "capsule";
        if (mode === "jenga") reset();
      },
    ),
  );

  controls.appendChild(
    createButtonGroup(
      [
        { label: "Jenga Stack", value: "jenga" },
        { label: "Box Stack", value: "boxes" },
        { label: "Pyramid2D (planar)", value: "pyramid" },
        { label: "Sphere Stack", value: "spheres" },
        { label: "Capsule Stack (planar)", value: "capsule-stack" },
        { label: "Single Box", value: "single" },
        { label: "Card House", value: "card-house" },
        { label: "Cylinder", value: "cylinder" },
        { label: "Cylinder Stack", value: "cylinder-stack" },
        { label: "Dominoes", value: "dominoes" },
        { label: "Wedge", value: "wedge" },
        { label: "Arch", value: "arch" },
        { label: "Double Domino", value: "double-domino" },
      ],
      mode,
      (v) => {
        mode = v as Mode;
        jengaControls.style.display = mode === "jenga" ? "" : "none";
        reset();
      },
    ),
  );
  controls.appendChild(jengaControls);
  jengaControls.style.display = mode === "jenga" ? "" : "none";

  const ctrl = attachInteraction({
    wasm,
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: "Stacking",
    sampleCategory: "Stacking",
  }) as SimControllerWithTick;

  reset();

  const quat = new THREE.Quaternion();
  // Tracks the last style buffer the gate handed back. When the pile is settled
  // the gate returns the very same cached array (referential identity), so an
  // unchanged reference means the per-instance colors need no re-upload this frame.
  let prevStyles: Uint32Array | undefined;
  const stop = runLoop(() => {
    ctrl.tickFrame();
    const poses = wasm.sim_body_poses();
    const awake = wasm.sim_counters()[5] ?? 0;
    const styles = styleGate(awake, () => wasm.sim_body_styles());
    const stylesChanged = styles !== prevStyles;
    prevStyles = styles;
    const n = Math.floor(poses.length / STRIDE);
    // Refresh convex-hull geometry when the body count changes (spawn/delete).
    // `sim_hull_geometry` reads live shapes, so it stays aligned to the pose list.
    if (HULL_MODES.has(mode) && n !== hullCount) {
      loadHullGeometry();
      hullCount = n;
    }
    while (meshes.length > n) {
      const m = meshes.pop();
      if (m) disposeMesh(m);
    }
    // Single Box HUD: C SingleBox::Step prints the cube's position (body 1, after ground).
    if (mode === "single" && n > 1) {
      const o = STRIDE;
      hud.textContent =
        `(x, y, z) = (${fmt2g(poses[o]!)}, ${fmt2g(poses[o + 1]!)}, ${fmt2g(poses[o + 2]!)})`;
    }

    for (const arr of bucketMats.values()) arr.length = 0;
    for (const arr of bucketStyles.values()) arr.length = 0;

    for (let i = 0; i < n; i++) {
      const o = i * STRIDE;
      const kind = poses[o + 10]!;

      if (isInstancedKind(kind)) {
        // Slot flipped from capsule/hull mesh → instanced: dispose the stale mesh.
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

      // Capsule (2) or hull (4) — individual mesh.
      const mesh = ensureMesh(i, kind);
      applyShapeStyle(mesh, styles[i]!);
      mesh.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
      quat.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
      mesh.quaternion.copy(quat);
      if (kind === 4) {
        // Hull geometry is already in body-local space; no half-extent scaling.
        mesh.scale.set(1, 1, 1);
      } else {
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
      }
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
    ctrl.dispose();
    stop();
    clearInstanced();
    clearMeshes();
    disposeHullGeos();
    demo.dispose();
    boxGeo.dispose();
    sphereGeo.dispose();
    cylGeo.dispose();
    for (const mat of matByKind.values()) mat.dispose();
  };
}
