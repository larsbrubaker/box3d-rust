// Stacking — box stack, pyramid, and sphere stack from World::step.

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

type Mode = "boxes" | "pyramid" | "spheres";

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Stacking",
    "Vertical box stack, Pyramid2D, and sphere stack — driven by the ported " +
      "<code>World::step</code> scalar solver (mirrors <code>sample_stacking</code>).",
    "Drag to orbit · switch scene · adjust count · restart",
    wasm.version(),
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

  const countSlider = createSlider("Count", 4, 20, stackCount, 1, (v) => {
    const n = Math.round(v);
    if (mode === "boxes") stackCount = n;
    else if (mode === "spheres") sphereCount = n;
    else pyramidSize = Math.min(10, Math.max(2, n));
    reset();
  });
  controls.appendChild(countSlider);

  controls.appendChild(
    createButtonGroup([{ label: "Restart", value: "restart" }], "restart", () => reset()),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { target: [0, 6, 0], distance: 28 });
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
    for (const m of meshes) demo.content.remove(m);
    meshes.length = 0;
  }

  function ensureMesh(i: number, kind: number): THREE.Mesh {
    let mesh = meshes[i];
    const wantSphere = kind === 1;
    if (!mesh || (wantSphere && mesh.geometry !== sphereGeo) || (!wantSphere && mesh.geometry !== boxGeo)) {
      if (mesh) demo.content.remove(mesh);
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
    if (mode === "boxes") wasm.sim_reset_stacking(stackCount);
    else if (mode === "pyramid") wasm.sim_reset_pyramid(pyramidSize);
    else wasm.sim_reset_sphere_stack(sphereCount);
  }

  reset();

  const quat = new THREE.Quaternion();
  let frame = 0;
  const stop = runLoop(() => {
    wasm.sim_step(1 / 60, 4);
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
      if (kind === 1) mesh.scale.setScalar(poses[o + 7]!);
      else mesh.scale.set(poses[o + 7]!, poses[o + 8]!, poses[o + 9]!);
    }
    frame += 1;
    if (frame % 20 === 0) {
      updateReadout(readout, [
        { label: "scene", value: mode },
        { label: "bodies", value: String(n) },
        { label: "frame", value: String(frame) },
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
    boxMat.dispose();
  };
}
