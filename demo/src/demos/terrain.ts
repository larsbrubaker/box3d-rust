// Terrain settle — dynamic bodies on a grid mesh or height-field wave.

import type * as THREE from "three";
import {
  createButtonGroup,
  createInfoBox,
  createReadout,
  updateReadout,
} from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import {
  DemoScene,
  makeTriangleMesh,
  makeWireEdges,
  trianglesFromWireframe,
} from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Terrain Settle",
    "Bodies settle on a triangle-mesh grid or a wave height field — dynamics on the " +
      "geometry from <code>sample_mesh</code> (Grid / Height Field).",
    "Drag to orbit · switch terrain · restart",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "Mesh mode uses <code>create_grid_mesh</code> + <code>create_mesh_shape</code>. " +
        "HF mode uses <code>create_wave</code> + <code>create_height_field_shape</code>.",
    ),
  );

  let mode = 0;
  controls.appendChild(
    createButtonGroup(
      [
        { label: "Grid Mesh", value: "mesh" },
        { label: "Height Field", value: "hf" },
      ],
      "mesh",
      (v) => {
        mode = v === "mesh" ? 0 : 1;
        reset();
      },
    ),
  );
  controls.appendChild(
    createButtonGroup([{ label: "Restart", value: "restart" }], "restart", () => reset()),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { target: [0, 2, 0], distance: 28 });
  const pool = createMeshPool();
  let terrainMesh: THREE.Mesh | null = null;
  let terrainWire: THREE.LineSegments | null = null;

  function clearTerrain() {
    if (terrainMesh) {
      demo.content.remove(terrainMesh);
      terrainMesh.geometry.dispose();
      (terrainMesh.material as THREE.Material).dispose();
      terrainMesh = null;
    }
    if (terrainWire) {
      demo.content.remove(terrainWire);
      terrainWire.geometry.dispose();
      (terrainWire.material as THREE.Material).dispose();
      terrainWire = null;
    }
  }

  function rebuildTerrain() {
    clearTerrain();
    const wire = wasm.terrain_wireframe();
    const positions = trianglesFromWireframe(wire);
    terrainMesh = makeTriangleMesh(positions, 0x6b8f71, 0.85);
    terrainWire = makeWireEdges(wire, 0x3d4f44, 0.35);
    demo.content.add(terrainMesh);
    demo.content.add(terrainWire);
  }

  function reset() {
    wasm.terrain_reset(mode);
    rebuildTerrain();
  }

  reset();

  let frame = 0;
  const stop = runLoop(() => {
    wasm.terrain_step(1 / 60, 4);
    const poses = wasm.terrain_poses();
    syncMeshesFromPoses(demo.content, pool, poses, { groundIndex: null });
    frame += 1;
    if (frame % 20 === 0) {
      updateReadout(readout, [
        { label: "terrain", value: mode === 0 ? "mesh" : "hf" },
        { label: "bodies", value: String(Math.floor(poses.length / 16)) },
        { label: "frame", value: String(frame) },
      ]);
    }
    demo.render();
  }, readout);

  return () => {
    stop();
    clearTerrain();
    disposeMeshPool(pool);
    demo.dispose();
  };
}
