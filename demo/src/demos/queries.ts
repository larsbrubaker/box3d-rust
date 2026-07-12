// Queries — interactive raycast against a small world.

import * as THREE from "three";
import {
  createButtonGroup,
  createInfoBox,
  createReadout,
  updateReadout,
} from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import { COLORS, DemoScene, lineMat } from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Queries",
    "Interactive <code>world_cast_ray_closest</code> against static props and a " +
      "falling sphere. Drag to orbit; the sweep ray updates every frame.",
    "Drag to orbit · ray updates automatically",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "Ray origin and tip animate so you can see hits and normals from the ported " +
        "query path (pairs with debug-draw work in task-10).",
    ),
  );

  controls.appendChild(
    createButtonGroup([{ label: "Restart", value: "restart" }], "restart", () => reset()),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { target: [0, 1, 0], distance: 16 });
  const pool = createMeshPool();

  const rayGeo = new THREE.BufferGeometry();
  const rayPositions = new Float32Array(6);
  rayGeo.setAttribute("position", new THREE.BufferAttribute(rayPositions, 3));
  const rayLine = new THREE.Line(rayGeo, lineMat(COLORS.accent, 0.9));
  demo.dynamic.add(rayLine);

  const hitMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.12, 12, 10),
    new THREE.MeshStandardMaterial({ color: COLORS.hit, roughness: 0.4 }),
  );
  hitMarker.visible = false;
  demo.dynamic.add(hitMarker);

  const normalGeo = new THREE.BufferGeometry();
  const normalPositions = new Float32Array(6);
  normalGeo.setAttribute("position", new THREE.BufferAttribute(normalPositions, 3));
  const normalLine = new THREE.Line(normalGeo, lineMat(COLORS.good, 1));
  normalLine.visible = false;
  demo.dynamic.add(normalLine);

  function reset() {
    wasm.query_reset();
  }

  reset();

  let frame = 0;
  const stop = runLoop(() => {
    wasm.query_step(1 / 60, 4);
    const poses = wasm.query_poses();
    syncMeshesFromPoses(demo.content, pool, poses);

    const t = frame / 60;
    const ox = Math.cos(t * 0.7) * 6;
    const oy = 2.5 + Math.sin(t * 0.9) * 0.8;
    const oz = Math.sin(t * 0.7) * 6;
    const tx = -ox * 0.2;
    const ty = 0.5;
    const tz = -oz * 0.2;
    const hit = wasm.query_ray_cast(ox, oy, oz, tx, ty, tz);

    rayPositions[0] = ox;
    rayPositions[1] = oy;
    rayPositions[2] = oz;
    if (hit[0]! > 0.5) {
      rayPositions[3] = hit[1]!;
      rayPositions[4] = hit[2]!;
      rayPositions[5] = hit[3]!;
      hitMarker.visible = true;
      hitMarker.position.set(hit[1]!, hit[2]!, hit[3]!);
      normalLine.visible = true;
      normalPositions[0] = hit[1]!;
      normalPositions[1] = hit[2]!;
      normalPositions[2] = hit[3]!;
      normalPositions[3] = hit[1]! + hit[4]! * 0.8;
      normalPositions[4] = hit[2]! + hit[5]! * 0.8;
      normalPositions[5] = hit[3]! + hit[6]! * 0.8;
      (normalGeo.attributes.position as THREE.BufferAttribute).needsUpdate = true;
    } else {
      rayPositions[3] = tx;
      rayPositions[4] = ty;
      rayPositions[5] = tz;
      hitMarker.visible = false;
      normalLine.visible = false;
    }
    (rayGeo.attributes.position as THREE.BufferAttribute).needsUpdate = true;

    frame += 1;
    if (frame % 15 === 0) {
      updateReadout(readout, [
        { label: "hit", value: hit[0]! > 0.5 ? "yes" : "no" },
        { label: "fraction", value: hit[7]!.toFixed(3) },
        {
          label: "point",
          value:
            hit[0]! > 0.5
              ? `(${hit[1]!.toFixed(2)}, ${hit[2]!.toFixed(2)}, ${hit[3]!.toFixed(2)})`
              : "—",
        },
      ]);
    }
    demo.render();
  }, readout);

  return () => {
    stop();
    disposeMeshPool(pool);
    rayGeo.dispose();
    normalGeo.dispose();
    (hitMarker.material as THREE.Material).dispose();
    hitMarker.geometry.dispose();
    demo.dispose();
  };
}
