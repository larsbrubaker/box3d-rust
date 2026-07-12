// Stacking — Single Box, Box Stack, Pyramid2D, Sphere Stack (sample_stacking).

import * as THREE from "three";
import { createButtonGroup, createInfoBox } from "../controls.ts";
import {
  attachInteraction,
  type ParamValues,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import { COLORS, DemoScene } from "../three-scene.ts";

const STRIDE = 11;

type Mode = "single" | "boxes" | "pyramid" | "spheres";

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Stacking",
    "Official Stacking samples — Single Box, Box Stack, Pyramid2D, and Sphere Stack — " +
      "driven by the ported <code>World::step</code> scalar solver.",
    "Drag body · Shift spawn · Ctrl delete · Space/S/R",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "Single Box matches C exactly (half=0.5, ωy=10). Box Stack / Sphere Stack use " +
        "browser-capped counts (C uses 40 / 30). Pyramid2D locks motion to the XY plane " +
        "with C's layout formula (a=1).",
    ),
  );

  let mode: Mode = "single";
  let stackCount = 12;
  let pyramidSize = 6;
  let sphereCount = 12;

  const demo = new DemoScene(canvas, { target: [0, 2, 0], distance: 14 });
  const meshes: THREE.Mesh[] = [];
  const boxGeo = new THREE.BoxGeometry(2, 2, 2);
  const sphereGeo = new THREE.SphereGeometry(1, 20, 14);
  const groundMat = new THREE.MeshStandardMaterial({
    color: 0x9aa3b2,
    roughness: 0.92,
    metalness: 0.05,
  });
  const boxMat = new THREE.MeshStandardMaterial({
    color: COLORS.accent,
    roughness: 0.4,
    metalness: 0.12,
  });

  function clearMeshes() {
    for (const m of meshes) {
      demo.content.remove(m);
      if (m.geometry !== boxGeo && m.geometry !== sphereGeo) m.geometry.dispose();
    }
    meshes.length = 0;
  }

  function setCameraForMode() {
    if (mode === "single") {
      demo.controls.target.set(0, 0, 0);
      demo.camera.position.set(0, 8, 12);
    } else if (mode === "pyramid") {
      demo.controls.target.set(0, 5, 0);
      demo.camera.position.set(0, 20, 40);
    } else if (mode === "spheres") {
      demo.controls.target.set(0, 10, 0);
      demo.camera.position.set(0, 15, 40);
    } else {
      demo.controls.target.set(0, 12, 0);
      demo.camera.position.set(0, 18, 42);
    }
    demo.controls.update();
  }

  function ensureMesh(i: number, kind: number): THREE.Mesh {
    let mesh = meshes[i];
    const wantSphere = kind === 1;
    const wantCapsule = kind === 2;

    if (wantCapsule) {
      if (!mesh || mesh.geometry.type !== "CapsuleGeometry") {
        if (mesh) {
          demo.content.remove(mesh);
          if (mesh.geometry !== boxGeo && mesh.geometry !== sphereGeo) mesh.geometry.dispose();
        }
        mesh = new THREE.Mesh(new THREE.CapsuleGeometry(0.2, 0.5, 4, 10), boxMat);
        demo.content.add(mesh);
        meshes[i] = mesh;
      } else {
        mesh.material = boxMat;
      }
      return mesh;
    }

    if (!mesh || (wantSphere && mesh.geometry !== sphereGeo) || (!wantSphere && mesh.geometry !== boxGeo)) {
      if (mesh) {
        demo.content.remove(mesh);
        if (mesh.geometry !== boxGeo && mesh.geometry !== sphereGeo) mesh.geometry.dispose();
      }
      mesh = new THREE.Mesh(wantSphere ? sphereGeo : boxGeo, i === 0 ? groundMat : boxMat);
      demo.content.add(mesh);
      meshes[i] = mesh;
    } else {
      mesh.material = i === 0 ? groundMat : boxMat;
    }
    return mesh;
  }

  function reset() {
    clearMeshes();
    if (mode === "single") wasm.sim_reset_single_box();
    else if (mode === "boxes") wasm.sim_reset_stacking(stackCount);
    else if (mode === "pyramid") wasm.sim_reset_pyramid(pyramidSize);
    else wasm.sim_reset_sphere_stack(sphereCount);
    setCameraForMode();
  }

  controls.appendChild(
    createButtonGroup(
      [
        { label: "Single Box", value: "single" },
        { label: "Box Stack", value: "boxes" },
        { label: "Pyramid2D", value: "pyramid" },
        { label: "Sphere Stack", value: "spheres" },
      ],
      "single",
      (v) => {
        mode = v as Mode;
        reset();
      },
    ),
  );

  const ctrl = attachInteraction({
    wasm,
    demo,
    canvas,
    controls,
    onRestart: reset,
    params: [
      {
        type: "slider",
        key: "count",
        label: "Count",
        min: 4,
        max: 24,
        step: 1,
        default: 12,
        restart: true,
      },
    ],
    onParamsChange: (values: ParamValues) => {
      const n = Math.round(Number(values.count) || 12);
      if (mode === "boxes") stackCount = n;
      else if (mode === "spheres") sphereCount = n;
      else if (mode === "pyramid") pyramidSize = Math.min(12, Math.max(2, n));
    },
  }) as SimControllerWithTick;

  reset();

  const quat = new THREE.Quaternion();
  const stop = runLoop(() => {
    ctrl.tickFrame();
    const poses = wasm.sim_body_poses();
    const n = Math.floor(poses.length / STRIDE);
    while (meshes.length > n) {
      const m = meshes.pop()!;
      demo.content.remove(m);
    }
    for (let i = 0; i < n; i++) {
      const o = i * STRIDE;
      const kind = poses[o + 10]!;
      const mesh = ensureMesh(i, kind);
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
    groundMat.dispose();
    boxMat.dispose();
  };
}
