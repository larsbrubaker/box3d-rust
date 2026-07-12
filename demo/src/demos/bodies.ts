// Bodies — HelloWorld fall + compound ground scene.

import * as THREE from "three";
import { createButtonGroup, createInfoBox, createReadout, updateReadout } from "../controls.ts";
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
    "Drag to orbit · switch scene · restart",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "Compound mode builds a static multi-hull compound via <code>create_compound</code> / " +
        "<code>create_compound_shape</code>, then drops spheres onto it.",
    ),
  );

  let mode: Mode = "bodies";
  let generation = 0;

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

  controls.appendChild(
    createButtonGroup(
      [{ label: "Restart", value: "restart" }],
      "restart",
      () => {
        generation += 1;
        reset();
      },
    ),
  );
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
  const compoundMat = new THREE.MeshStandardMaterial({
    color: 0x6b7280,
    roughness: 0.85,
    metalness: 0.08,
  });

  function clearMeshes() {
    for (const m of meshes) demo.content.remove(m);
    meshes.length = 0;
  }

  function reset() {
    clearMeshes();
    if (mode === "bodies") wasm.sim_reset_bodies();
    else wasm.sim_reset_compound();
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
    // Compound children are the first static hulls; treat early boxes as ground-colored.
    const compoundHulls = mode === "compound" ? 3 : 1;
    for (let i = 0; i < n; i++) {
      const o = i * STRIDE;
      const kind = poses[o + 10]!;
      const hx = poses[o + 7]!;
      const hy = poses[o + 8]!;
      const hz = poses[o + 9]!;
      let mesh = meshes[i] as THREE.Mesh | undefined;
      const wantSphere = kind === 1;
      if (
        !mesh ||
        (wantSphere && mesh.geometry !== sphereGeo) ||
        (!wantSphere && mesh.geometry !== boxGeo)
      ) {
        if (mesh) demo.content.remove(mesh);
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
    frame += 1;
    if (frame % 15 === 0) {
      updateReadout(readout, [
        { label: "scene", value: mode },
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
    compoundMat.dispose();
  };
}
