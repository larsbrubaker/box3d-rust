// Stacking — vertical box stack from World::step (sample BoxStack).

import * as THREE from "three";
import {
  createButtonGroup,
  createInfoBox,
  createReadout,
  createSlider,
  updateReadout,
} from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import { COLORS, DemoScene } from "../three-scene.ts";

const STRIDE = 11;

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Stacking",
    "A vertical hull stack driven by the ported <code>World::step</code> " +
      "(pairs → collide → scalar contact solve), mirroring the upstream Box Stack sample.",
    "Drag to orbit · adjust stack height and restart",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "Boxes settle under gravity. Stack count is capped for browser frame rate; " +
        "the C sample uses 40 — try 12–16 here.",
    ),
  );

  let stackCount = 12;
  controls.appendChild(
    createSlider("Boxes", 4, 20, stackCount, 1, (v) => {
      stackCount = Math.round(v);
      reset();
    }),
  );

  const restartGroup = createButtonGroup(
    [{ label: "Restart", value: "restart" }],
    "restart",
    () => reset(),
  );
  controls.appendChild(restartGroup);
  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { target: [0, 6, 0], distance: 28 });
  const meshes: THREE.Mesh[] = [];
  const boxGeo = new THREE.BoxGeometry(2, 2, 2);
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

  function rebuild(count: number) {
    for (const m of meshes) demo.content.remove(m);
    meshes.length = 0;
    for (let i = 0; i < count; i++) {
      const mesh = new THREE.Mesh(boxGeo, i === 0 ? groundMat : boxMat);
      demo.content.add(mesh);
      meshes.push(mesh);
    }
  }

  function reset() {
    const count = wasm.sim_reset_stacking(stackCount);
    rebuild(count);
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
      const mesh = meshes[i];
      if (!mesh) continue;
      mesh.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
      quat.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
      mesh.quaternion.copy(quat);
      mesh.scale.set(poses[o + 7]!, poses[o + 8]!, poses[o + 9]!);
    }
    frame += 1;
    if (frame % 20 === 0) {
      updateReadout(readout, [
        { label: "stacked", value: String(stackCount) },
        { label: "frame", value: String(frame) },
      ]);
    }
    demo.render();
  }, readout);

  return () => {
    stop();
    demo.dispose();
    boxGeo.dispose();
    groundMat.dispose();
    boxMat.dispose();
  };
}
