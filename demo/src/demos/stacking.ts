// Stacking — the full sample_stacking.cpp roster. Batch 1: Jenga, Box Stack,
// Pyramid2D (planar), Sphere Stack, Single Box. Batch 3b adds Card House Thick,
// Card House, Capsule Stack, Cylinder, Cylinder Stack, Dominoes, Wedge, Arch, and
// Double Domino — all driven by the ported World::step scalar solver.

import * as THREE from "three";
import { createButtonGroup, createCanvasOverlay, createInfoBox, fmt2g } from "../controls.ts";
import {
  attachInteraction,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { demoPage, makeStyleGate, runLoop } from "./common.ts";
import { applyShapeStyle, DemoScene, makeShapeMaterial, setView } from "../three-scene.ts";

/** `[px..qw, hx,hy,hz, kind, bodyType, awake]` */
const STRIDE = 13;

type Mode =
  | "jenga"
  | "boxes"
  | "pyramid"
  | "spheres"
  | "single"
  | "card-house-thick"
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
  "card-house-thick",
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

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  // Self-check the scene table against the registry (see registry.ts pattern).
  assertRouteScenes("stacking", SCENES);
  const { canvas, controls, page } = demoPage(
    container,
    "Stacking",
    "The full official Stacking roster — Jenga, Box Stack, Pyramid2D (planar), Sphere/Capsule " +
      "stacks, both Card Houses, Cylinder + Cylinder Stack, Dominoes, Wedge, Arch, and Double " +
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
        "axis (and angular axes) so collapse stays in-plane — not a 3D engine bug.",
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
  // Each mesh owns its material — engine style words color bodies per-body
  // (sleep/wake/fast/bullet), so a shared material can't be used.
  const meshes: THREE.Mesh[] = [];
  const boxGeo = new THREE.BoxGeometry(2, 2, 2);
  const sphereGeo = new THREE.SphereGeometry(1, 20, 14);
  // Per-body convex-hull geometries (kind 4), index-aligned to the pose list.
  // Owned here (not by the meshes) so a hull mesh recycle never double-frees them.
  const hullGeos: (THREE.BufferGeometry | null)[] = [];
  let hullCount = -1;

  function disposeMesh(m: THREE.Mesh) {
    demo.content.remove(m);
    // kind-4 hull meshes borrow a geometry owned by `hullGeos`; never dispose it here.
    if (m.userData.kind !== 4 && m.geometry !== boxGeo && m.geometry !== sphereGeo) {
      m.geometry.dispose();
    }
    (m.material as THREE.Material).dispose();
  }

  function clearMeshes() {
    for (const m of meshes) disposeMesh(m);
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
        setView(demo, 0, 15, 50, [0, 20, 0]); // BoxStack :432
        break;
      case "pyramid":
        setView(demo, 0, 30, 50, [0, 5, 0]); // Pyramid2D :895
        break;
      case "spheres":
        setView(demo, 0, 15, 50, [0, 10, 0]); // SphereStack :173
        break;
      case "card-house-thick":
        setView(demo, 0, 25, 10, [0, 2, 0]); // CardHouseThick :21
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

  function ensureMesh(i: number, kind: number, bodyType: number): THREE.Mesh {
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

    const wantSphere = kind === 1;
    const wantCapsule = kind === 2;
    const wantCylinder = kind === 3;

    if (wantCapsule) {
      if (!mesh || mesh.geometry.type !== "CapsuleGeometry") {
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

    if (wantCylinder) {
      if (!mesh || mesh.geometry.type !== "CylinderGeometry") {
        if (mesh) disposeMesh(mesh);
        mesh = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.15, 2.0, 16), makeShapeMaterial());
        mesh.userData.kind = 3;
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        demo.content.add(mesh);
        meshes[i] = mesh;
      }
      return mesh;
    }

    if (!mesh || mesh.userData.kind === 4 || mesh.userData.kind === 3 || (wantSphere && mesh.geometry !== sphereGeo) || (!wantSphere && mesh.geometry !== boxGeo)) {
      if (mesh) disposeMesh(mesh);
      mesh = new THREE.Mesh(wantSphere ? sphereGeo : boxGeo, makeShapeMaterial());
      mesh.userData.kind = wantSphere ? 1 : 0;
      mesh.castShadow = bodyType !== 0;
      mesh.receiveShadow = true;
      demo.content.add(mesh);
      meshes[i] = mesh;
    }
    return mesh;
  }

  function reset() {
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
      case "card-house-thick":
        wasm.sim_reset_card_house_thick();
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
        { label: "Card House Thick", value: "card-house-thick" },
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
  const stop = runLoop(() => {
    ctrl.tickFrame();
    const poses = wasm.sim_body_poses();
    const awake = wasm.sim_counters()[5] ?? 0;
    const styles = styleGate(awake, () => wasm.sim_body_styles());
    const n = Math.floor(poses.length / STRIDE);
    // Refresh convex-hull geometry when the body count changes (spawn/delete).
    // `sim_hull_geometry` reads live shapes, so it stays aligned to the pose list.
    if (HULL_MODES.has(mode) && n !== hullCount) {
      loadHullGeometry();
      hullCount = n;
    }
    while (meshes.length > n) {
      disposeMesh(meshes.pop()!);
    }
    // Single Box HUD: C SingleBox::Step prints the cube's position (body 1, after ground).
    if (mode === "single" && n > 1) {
      const o = STRIDE;
      hud.textContent =
        `(x, y, z) = (${fmt2g(poses[o]!)}, ${fmt2g(poses[o + 1]!)}, ${fmt2g(poses[o + 2]!)})`;
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
      if (kind === 4) {
        // Hull geometry is already in body-local space; no half-extent scaling.
        mesh.scale.set(1, 1, 1);
      } else if (kind === 2) {
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
    ctrl.dispose();
    stop();
    disposeHullGeos();
    demo.dispose();
    boxGeo.dispose();
    sphereGeo.dispose();
  };
}
