// Stacking — Jenga, Box Stack, Pyramid2D (planar), Sphere Stack, Single Box (sample_stacking).

import * as THREE from "three";
import { createButtonGroup, createCanvasOverlay, createInfoBox, fmt2g } from "../controls.ts";
import {
  attachInteraction,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { demoPage, runLoop } from "./common.ts";
import { applyShapeStyle, DemoScene, makeShapeMaterial, setView } from "../three-scene.ts";

/** `[px..qw, hx,hy,hz, kind, bodyType, awake]` */
const STRIDE = 13;

type Mode = "jenga" | "boxes" | "pyramid" | "spheres" | "single";

export const SCENES: Mode[] = ["jenga", "boxes", "pyramid", "spheres", "single"];

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  // Self-check the scene table against the registry (see registry.ts pattern).
  assertRouteScenes("stacking", SCENES);
  const { canvas, controls, page } = demoPage(
    container,
    "Stacking",
    "Official Stacking samples — Jenga Stack, Box Stack, Pyramid2D (planar), Sphere Stack, " +
      "and Single Box — driven by the ported <code>World::step</code> scalar solver.",
    "Ctrl+click grab · Shift+click spawn · click select · P/O/R",
    wasm.version(),
    { category: "Stacking", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "Exact C sample counts: <strong>Jenga Stack</strong> 40 planks (Hull or Capsule, C " +
        "<code>DrawControls</code> radio), <strong>Box Stack</strong> 40, <strong>Sphere Stack</strong> " +
        "30. Jenga is the 3D showcase (alternating X/Z planks, no locks). " +
        "<strong>Pyramid2D is intentionally planar</strong>: C locks linear Z + angular X/Y " +
        "so collapse stays in the XY plane — not a 3D engine bug.",
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
  // Each mesh owns its material — engine style words color bodies per-body
  // (sleep/wake/fast/bullet), so a shared material can't be used.
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
    // C SetView(yaw, pitch, distance, target) values from sample_stacking.cpp.
    if (mode === "single") {
      setView(demo, 0, 25, 10, [0, 0, 0]); // SingleBox :278
    } else if (mode === "boxes") {
      setView(demo, 0, 15, 50, [0, 20, 0]); // BoxStack :432
    } else if (mode === "pyramid") {
      setView(demo, 0, 30, 50, [0, 5, 0]); // Pyramid2D :895
    } else if (mode === "spheres") {
      setView(demo, 0, 15, 50, [0, 10, 0]); // SphereStack :173
    } else {
      setView(demo, 35, 15, 30, [0, 10, 0]); // JengaStack :493
    }
  }

  function ensureMesh(i: number, kind: number, bodyType: number): THREE.Mesh {
    let mesh = meshes[i];
    const wantSphere = kind === 1;
    const wantCapsule = kind === 2;

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

    if (!mesh || (wantSphere && mesh.geometry !== sphereGeo) || (!wantSphere && mesh.geometry !== boxGeo)) {
      if (mesh) disposeMesh(mesh);
      mesh = new THREE.Mesh(wantSphere ? sphereGeo : boxGeo, makeShapeMaterial());
      mesh.castShadow = bodyType !== 0;
      mesh.receiveShadow = true;
      demo.content.add(mesh);
      meshes[i] = mesh;
    }
    return mesh;
  }

  function reset() {
    clearMeshes();
    hud.style.display = mode === "single" ? "" : "none";
    // Fixed C sample counts (all light enough for serial wasm).
    if (mode === "single") wasm.sim_reset_single_box();
    else if (mode === "boxes") wasm.sim_reset_stacking();
    else if (mode === "jenga") wasm.sim_reset_jenga(jengaShape === "capsule" ? 1 : 0);
    else if (mode === "pyramid") wasm.sim_reset_pyramid();
    else wasm.sim_reset_sphere_stack();
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
        { label: "Single Box", value: "single" },
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
    const styles = wasm.sim_body_styles();
    const n = Math.floor(poses.length / STRIDE);
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
    demo.dispose();
    boxGeo.dispose();
    sphereGeo.dispose();
  };
}
