// Bodies — HelloWorld fall + compound ground scene, with full interaction layer.

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

type Mode = "bodies" | "compound";

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Bodies",
    "Live <code>World::step</code>: HelloWorld hulls/spheres, or a compound-shape ground " +
      "with falling bodies (mirrors <code>sample_compound</code> Simple).",
    "Drag body · Shift spawn · Ctrl delete · Space/S/R",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "Compound mode builds a static multi-hull compound via <code>create_compound</code> / " +
        "<code>create_compound_shape</code>, then drops spheres onto it.",
    ),
  );

  let mode: Mode = "bodies";

  const demo = new DemoScene(canvas, { target: [0, 2, 0], distance: 18 });
  const meshes: THREE.Object3D[] = [];
  const boxGeo = new THREE.BoxGeometry(2, 2, 2);
  const sphereGeo = new THREE.SphereGeometry(1, 24, 16);
  const groundMat = new THREE.MeshStandardMaterial({
    color: 0x9aa3b2,
    roughness: 0.9,
    metalness: 0.05,
  });
  const dynamicMat = new THREE.MeshStandardMaterial({
    color: COLORS.accent,
    roughness: 0.45,
    metalness: 0.15,
  });
  const compoundMat = new THREE.MeshStandardMaterial({
    color: 0x6b7280,
    roughness: 0.85,
    metalness: 0.08,
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
    if (mode === "bodies") wasm.sim_reset_bodies();
    else wasm.sim_reset_compound();
  }

  controls.appendChild(
    createButtonGroup(
      [
        { label: "Hello World", value: "bodies" },
        { label: "Compound", value: "compound" },
      ],
      "bodies",
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
    ],
    onParamsChange: (values: ParamValues) => {
      subSteps = Number(values.subSteps) || 4;
      ctrl.subSteps = subSteps;
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
    const compoundHulls = mode === "compound" ? 3 : 1;
    for (let i = 0; i < n; i++) {
      const o = i * STRIDE;
      const kind = poses[o + 10]!;
      const hx = poses[o + 7]!;
      const hy = poses[o + 8]!;
      const hz = poses[o + 9]!;
      let mesh = meshes[i] as THREE.Mesh | undefined;
      const wantSphere = kind === 1;
      const wantCapsule = kind === 2;

      if (wantCapsule) {
        const radius = hx;
        const halfLen = hy;
        const needNew =
          !mesh || (mesh.geometry as THREE.CapsuleGeometry)?.type !== "CapsuleGeometry";
        if (needNew) {
          if (mesh) demo.content.remove(mesh);
          mesh = new THREE.Mesh(
            new THREE.CapsuleGeometry(radius, Math.max(1e-4, halfLen * 2), 4, 10),
            dynamicMat,
          );
          demo.content.add(mesh);
          meshes[i] = mesh;
        } else {
          mesh.material = dynamicMat;
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
          if (
            mesh.geometry !== boxGeo &&
            mesh.geometry !== sphereGeo
          ) {
            mesh.geometry.dispose();
          }
        }
        const mat =
          !wantSphere && i < compoundHulls
            ? mode === "compound"
              ? compoundMat
              : groundMat
            : dynamicMat;
        mesh = new THREE.Mesh(wantSphere ? sphereGeo : boxGeo, mat);
        demo.content.add(mesh);
        meshes[i] = mesh;
      } else {
        mesh.material =
          !wantSphere && i < compoundHulls
            ? mode === "compound"
              ? compoundMat
              : groundMat
            : dynamicMat;
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
    demo.dispose();
    boxGeo.dispose();
    sphereGeo.dispose();
    groundMat.dispose();
    dynamicMat.dispose();
    compoundMat.dispose();
  };
}
