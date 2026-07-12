// Bodies — HelloWorld fall + compound ground scene, with Samples App Info panel.

import * as THREE from "three";
import { createButtonGroup, createInfoBox } from "../controls.ts";
import {
  attachInteraction,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import { applyBodyColor, DemoScene, makeBodyMaterial } from "../three-scene.ts";

/** `[px..qw, hx,hy,hz, kind, bodyType, awake]` */
const STRIDE = 13;

type Mode = "bodies" | "compound";

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Bodies",
    "Live <code>World::step</code>: HelloWorld hulls/spheres, or a compound-shape ground " +
      "with falling bodies (mirrors <code>sample_compound</code> Simple).",
    "Drag body · Shift spawn · Ctrl delete · P/O/R",
    wasm.version(),
    { category: "Bodies", samplesShell: true },
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
  const sphereGeo = new THREE.SphereGeometry(1, 20, 14);
  // Shared materials per body state; colorized each frame from pose bodyType/awake.
  const staticMat = makeBodyMaterial(0, true);
  const dynamicMat = makeBodyMaterial(2, true);
  const sleepMat = makeBodyMaterial(2, false);

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

  const ctrl = attachInteraction({
    wasm,
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: "Bodies",
    sampleCategory: "Bodies",
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
      const hx = poses[o + 7]!;
      const hy = poses[o + 8]!;
      const hz = poses[o + 9]!;
      let mesh = meshes[i] as THREE.Mesh | undefined;
      const wantSphere = kind === 1;
      const wantCapsule = kind === 2;
      const mat =
        bodyType === 0 ? staticMat : awake ? dynamicMat : sleepMat;
      applyBodyColor(mat, bodyType, awake);

      if (wantCapsule) {
        const radius = hx;
        const halfLen = hy;
        const needNew =
          !mesh || (mesh.geometry as THREE.CapsuleGeometry)?.type !== "CapsuleGeometry";
        if (needNew) {
          if (mesh) demo.content.remove(mesh);
          mesh = new THREE.Mesh(
            new THREE.CapsuleGeometry(radius, Math.max(1e-4, halfLen * 2), 4, 10),
            mat,
          );
          mesh.castShadow = true;
          mesh.receiveShadow = true;
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
          if (
            mesh.geometry !== boxGeo &&
            mesh.geometry !== sphereGeo
          ) {
            mesh.geometry.dispose();
          }
        }
        mesh = new THREE.Mesh(wantSphere ? sphereGeo : boxGeo, mat);
        mesh.castShadow = bodyType !== 0;
        mesh.receiveShadow = true;
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
    demo.dispose();
    boxGeo.dispose();
    sphereGeo.dispose();
    staticMat.dispose();
    dynamicMat.dispose();
    sleepMat.dispose();
  };
}
