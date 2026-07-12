// Compound — Simple / Spheres / Hulls / Village gallery (sample_compound.cpp).

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

type Mode = "simple" | "spheres" | "hulls" | "village";

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Compound",
    "Compound shape gallery from <code>sample_compound.cpp</code>: Simple, Spheres, Hulls, " +
      "and Village (browser-scaled tile grid). Physics via <code>create_compound</code> / " +
      "<code>create_compound_shape</code>.",
    "Drag body · Shift spawn · Ctrl delete · Space/S/R",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "Village follows the C compound ground pattern (hull tiles + odd-tile sphere/capsule props). " +
        "C debug uses gridCount 8 / release 200; here the grid is 8–16. Building meshes are omitted. " +
        "Walk the village from Character → Village.",
    ),
  );

  let mode: Mode = "simple";
  let villageGrid = 10;

  const demo = new DemoScene(canvas, { target: [0, 2, 0], distance: 22 });
  // Village needs a longer far plane.
  demo.camera.far = 400;
  demo.camera.updateProjectionMatrix();

  const meshes: THREE.Object3D[] = [];
  const boxGeo = new THREE.BoxGeometry(2, 2, 2);
  const sphereGeo = new THREE.SphereGeometry(1, 20, 14);
  const groundMat = new THREE.MeshStandardMaterial({
    color: 0x6b7280,
    roughness: 0.88,
    metalness: 0.08,
  });
  const propMat = new THREE.MeshStandardMaterial({
    color: 0x8b7355,
    roughness: 0.7,
    metalness: 0.05,
  });
  const dynamicMat = new THREE.MeshStandardMaterial({
    color: COLORS.accent,
    roughness: 0.45,
    metalness: 0.15,
  });

  function clearMeshes() {
    for (const m of meshes) {
      demo.content.remove(m);
      const mesh = m as THREE.Mesh;
      if (
        mesh.geometry &&
        mesh.geometry !== boxGeo &&
        mesh.geometry !== sphereGeo
      ) {
        mesh.geometry.dispose();
      }
    }
    meshes.length = 0;
  }

  function reset() {
    clearMeshes();
    if (mode === "simple") wasm.sim_reset_compound_simple();
    else if (mode === "spheres") wasm.sim_reset_compound_spheres();
    else if (mode === "hulls") wasm.sim_reset_compound_hulls();
    else wasm.sim_reset_village(villageGrid);

    if (mode === "village") {
      const half = villageGrid * 4;
      demo.controls.target.set(0, 4, 0);
      demo.camera.position.set(half * 0.6, half * 0.35, half * 0.7);
      demo.controls.update();
    } else if (mode === "spheres" || mode === "hulls") {
      demo.controls.target.set(0, 0, 0);
      demo.camera.position.set(18, 14, 22);
      demo.controls.update();
    } else {
      demo.controls.target.set(0, 1, 0);
      demo.camera.position.set(10, 8, 14);
      demo.controls.update();
    }
  }

  controls.appendChild(
    createButtonGroup(
      [
        { label: "Simple", value: "simple" },
        { label: "Spheres", value: "spheres" },
        { label: "Hulls", value: "hulls" },
        { label: "Village", value: "village" },
      ],
      "simple",
      (v) => {
        mode = v as Mode;
        reset();
      },
    ),
  );

  let subSteps = 4;

  const ctrl = attachInteraction({
    wasm,
    demo,
    canvas,
    controls,
    onRestart: reset,
    params: [
      {
        type: "slider",
        key: "subSteps",
        label: "Sub-steps",
        min: 1,
        max: 8,
        step: 1,
        default: 4,
        restart: false,
      },
      {
        type: "slider",
        key: "villageGrid",
        label: "Village grid",
        min: 8,
        max: 16,
        step: 1,
        default: 10,
        restart: true,
      },
    ],
    onParamsChange: (values: ParamValues, key: string) => {
      subSteps = Number(values.subSteps) || 4;
      ctrl.subSteps = subSteps;
      if (key === "villageGrid") {
        villageGrid = Number(values.villageGrid) || 10;
        if (mode === "village") reset();
      }
    },
  }) as SimControllerWithTick;

  ctrl.subSteps = subSteps;
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

    const staticCount =
      mode === "village"
        ? Math.max(0, n - 3)
        : mode === "simple"
          ? 1
          : mode === "spheres" || mode === "hulls"
            ? 20
            : 0;

    for (let i = 0; i < n; i++) {
      const o = i * STRIDE;
      const kind = poses[o + 10]!;
      const hx = poses[o + 7]!;
      const hy = poses[o + 8]!;
      const hz = poses[o + 9]!;
      let mesh = meshes[i] as THREE.Mesh | undefined;
      const wantSphere = kind === 1;
      const wantCapsule = kind === 2;
      const isStatic = i < staticCount;
      const mat = isStatic ? (wantSphere || wantCapsule ? propMat : groundMat) : dynamicMat;

      if (wantCapsule) {
        const radius = hx;
        const halfLen = Math.max(1e-4, hy);
        const needNew =
          !mesh || (mesh.geometry as THREE.CapsuleGeometry)?.type !== "CapsuleGeometry";
        if (needNew) {
          if (mesh) demo.content.remove(mesh);
          mesh = new THREE.Mesh(
            new THREE.CapsuleGeometry(radius, halfLen * 2, 4, 10),
            mat,
          );
          demo.content.add(mesh);
          meshes[i] = mesh;
        } else {
          mesh.material = mat;
        }
        mesh.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
        quat.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
        mesh.quaternion.copy(quat);
        continue;
      }

      if (
        !mesh ||
        (wantSphere && mesh.geometry !== sphereGeo) ||
        (!wantSphere && mesh.geometry !== boxGeo)
      ) {
        if (mesh) {
          demo.content.remove(mesh);
          if (mesh.geometry !== boxGeo && mesh.geometry !== sphereGeo) {
            mesh.geometry.dispose();
          }
        }
        mesh = new THREE.Mesh(wantSphere ? sphereGeo : boxGeo, mat);
        demo.content.add(mesh);
        meshes[i] = mesh;
      } else {
        mesh.material = mat;
      }
      mesh.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
      quat.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
      mesh.quaternion.copy(quat);
      if (kind === 1) mesh.scale.setScalar(hx);
      else mesh.scale.set(hx, hy, hz);
    }
    demo.render();
  }, controls);

  return () => {
    ctrl.dispose();
    stop();
    clearMeshes();
    demo.dispose();
    boxGeo.dispose();
    sphereGeo.dispose();
    groundMat.dispose();
    propMat.dispose();
    dynamicMat.dispose();
  };
}
