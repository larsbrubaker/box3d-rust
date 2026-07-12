// Stacking — box stack, pyramid, and sphere stack with Samples App Info panel.

import * as THREE from "three";
import { createButtonGroup, createInfoBox } from "../controls.ts";
import {
  attachInteraction,
  type ParamValues,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import { applyBodyColor, DemoScene, makeBodyMaterial } from "../three-scene.ts";

/** `[px..qw, hx,hy,hz, kind, bodyType, awake]` */
const STRIDE = 13;

type Mode = "boxes" | "pyramid" | "spheres";

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Stacking",
    "Vertical box stack, Pyramid2D, and sphere stack — driven by the ported " +
      "<code>World::step</code> scalar solver (mirrors <code>sample_stacking</code>).",
    "Drag body · Shift spawn · Ctrl delete · P/O/R",
    wasm.version(),
    { category: "Stacking", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "Box stack matches upstream Box Stack. Pyramid locks motion to the XY plane. " +
        "Sphere stack uses rolling resistance like Sphere Stack.",
    ),
  );

  let mode: Mode = "boxes";
  let stackCount = 12;
  let pyramidSize = 6;
  let sphereCount = 12;

  const demo = new DemoScene(canvas, { target: [0, 6, 0], distance: 28, shadowExtent: 36 });
  const meshes: THREE.Mesh[] = [];
  const boxGeo = new THREE.BoxGeometry(2, 2, 2);
  const sphereGeo = new THREE.SphereGeometry(1, 20, 14);
  const staticMat = makeBodyMaterial(0, true);
  const dynamicMat = makeBodyMaterial(2, true);
  const sleepMat = makeBodyMaterial(2, false);

  function clearMeshes() {
    for (const m of meshes) {
      demo.content.remove(m);
      if (m.geometry !== boxGeo && m.geometry !== sphereGeo) m.geometry.dispose();
    }
    meshes.length = 0;
  }

  function pickMat(bodyType: number, awake: boolean): THREE.MeshStandardMaterial {
    const mat = bodyType === 0 ? staticMat : awake ? dynamicMat : sleepMat;
    applyBodyColor(mat, bodyType, awake);
    return mat;
  }

  function ensureMesh(i: number, kind: number, bodyType: number, awake: boolean): THREE.Mesh {
    let mesh = meshes[i];
    const wantSphere = kind === 1;
    const wantCapsule = kind === 2;
    const mat = pickMat(bodyType, awake);

    if (wantCapsule) {
      if (!mesh || mesh.geometry.type !== "CapsuleGeometry") {
        if (mesh) {
          demo.content.remove(mesh);
          if (mesh.geometry !== boxGeo && mesh.geometry !== sphereGeo) mesh.geometry.dispose();
        }
        mesh = new THREE.Mesh(new THREE.CapsuleGeometry(0.2, 0.5, 4, 10), mat);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        demo.content.add(mesh);
        meshes[i] = mesh;
      } else {
        mesh.material = mat;
      }
      return mesh;
    }

    if (!mesh || (wantSphere && mesh.geometry !== sphereGeo) || (!wantSphere && mesh.geometry !== boxGeo)) {
      if (mesh) {
        demo.content.remove(mesh);
        if (mesh.geometry !== boxGeo && mesh.geometry !== sphereGeo) mesh.geometry.dispose();
      }
      mesh = new THREE.Mesh(wantSphere ? sphereGeo : boxGeo, mat);
      mesh.castShadow = bodyType !== 0;
      mesh.receiveShadow = true;
      demo.content.add(mesh);
      meshes[i] = mesh;
    } else {
      mesh.material = mat;
    }
    return mesh;
  }

  function reset() {
    clearMeshes();
    if (mode === "boxes") wasm.sim_reset_stacking(stackCount);
    else if (mode === "pyramid") wasm.sim_reset_pyramid(pyramidSize);
    else wasm.sim_reset_sphere_stack(sphereCount);
  }

  controls.appendChild(
    createButtonGroup(
      [
        { label: "Box Stack", value: "boxes" },
        { label: "Pyramid", value: "pyramid" },
        { label: "Spheres", value: "spheres" },
      ],
      "boxes",
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
    sampleName: "Stacking",
    sampleCategory: "Stacking",
    params: [
      {
        type: "slider",
        key: "count",
        label: "Count",
        min: 4,
        max: 20,
        step: 1,
        default: 12,
        restart: true,
      },
    ],
    onParamsChange: (values: ParamValues) => {
      const n = Math.round(Number(values.count) || 12);
      if (mode === "boxes") stackCount = n;
      else if (mode === "spheres") sphereCount = n;
      else pyramidSize = Math.min(10, Math.max(2, n));
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
      const bodyType = poses[o + 11]! | 0;
      const awake = poses[o + 12]! > 0.5;
      const mesh = ensureMesh(i, kind, bodyType, awake);
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
    staticMat.dispose();
    dynamicMat.dispose();
    sleepMat.dispose();
  };
}
