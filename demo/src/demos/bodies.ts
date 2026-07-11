// Bodies — live World::step with falling hulls and spheres.

import * as THREE from "three";
import { createButtonGroup, createInfoBox, createReadout, updateReadout } from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import { COLORS, DemoScene } from "../three-scene.ts";

const STRIDE = 11;

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Bodies",
    "A live <code>World::step</code> scene: static ground plus dynamic hulls and spheres " +
      "(HelloWorld cube and companions). Gravity, collide, and the scalar contact solver " +
      "run entirely in wasm.",
    "Drag to orbit · simulation steps every frame",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "Restart drops the bodies again. Poses come from <code>sim_body_poses</code> after each step.",
    ),
  );

  let generation = 0;
  const restartGroup = createButtonGroup(
    [{ label: "Restart", value: "restart" }],
    "restart",
    () => {
      generation += 1;
      reset();
    },
  );
  controls.appendChild(restartGroup);
  const readout = createReadout();
  controls.appendChild(readout);

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

  function rebuildMeshes(count: number) {
    for (const m of meshes) {
      demo.content.remove(m);
    }
    meshes.length = 0;
    for (let i = 0; i < count; i++) {
      const mesh = new THREE.Mesh(boxGeo, i === 0 ? groundMat : dynamicMat);
      demo.content.add(mesh);
      meshes.push(mesh);
    }
  }

  function reset() {
    const count = wasm.sim_reset_bodies();
    rebuildMeshes(count);
  }

  reset();

  const quat = new THREE.Quaternion();
  let frame = 0;
  const stop = runLoop(() => {
    wasm.sim_step(1 / 60, 4);
    const poses = wasm.sim_body_poses();
    const n = poses.length / STRIDE;
    for (let i = 0; i < n; i++) {
      const o = i * STRIDE;
      let mesh = meshes[i];
      if (!mesh) continue;
      const kind = poses[o + 10]!;
      const hx = poses[o + 7]!;
      const hy = poses[o + 8]!;
      const hz = poses[o + 9]!;
      if (kind === 1 && mesh instanceof THREE.Mesh && mesh.geometry !== sphereGeo) {
        demo.content.remove(mesh);
        mesh = new THREE.Mesh(sphereGeo, dynamicMat);
        demo.content.add(mesh);
        meshes[i] = mesh;
      } else if (kind === 0 && mesh instanceof THREE.Mesh && mesh.geometry === sphereGeo) {
        demo.content.remove(mesh);
        mesh = new THREE.Mesh(boxGeo, i === 0 ? groundMat : dynamicMat);
        demo.content.add(mesh);
        meshes[i] = mesh;
      }
      mesh.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
      quat.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
      mesh.quaternion.copy(quat);
      if (kind === 1) {
        mesh.scale.setScalar(hx);
      } else {
        mesh.scale.set(hx, hy, hz);
      }
    }
    frame += 1;
    if (frame % 15 === 0) {
      updateReadout(readout, [
        { label: "bodies", value: String(n) },
        { label: "frame", value: String(frame) },
        { label: "resets", value: String(generation) },
      ]);
    }
    demo.render();
  }, readout);

  return () => {
    stop();
    demo.dispose();
    boxGeo.dispose();
    sphereGeo.dispose();
    groundMat.dispose();
    dynamicMat.dispose();
  };
}
