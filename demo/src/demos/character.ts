// Character mover — capsule on height-field terrain (WASD / Space / Shift).

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
    "Character",
    "A capsule mover on wave terrain using <code>world_collide_mover</code> / " +
      "<code>solve_planes</code> / <code>world_cast_mover</code> — keyboard-driven like " +
      "the upstream BasicMover sample.",
    "WASD move · Space jump · Shift sprint · drag to orbit",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "Click the canvas to focus keys. Camera-relative movement on the XZ plane; " +
        "pogo spring detects ground. Obstacles are static hulls on the height field.",
    ),
  );
  controls.appendChild(
    createButtonGroup([{ label: "Respawn", value: "restart" }], "restart", () => reset()),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { target: [0, 2, 0], distance: 18 });
  const pool = createMeshPool();
  let terrainMesh: THREE.Mesh | null = null;
  let terrainWire: THREE.LineSegments | null = null;

  const keys = new Set<string>();
  const onKeyDown = (e: KeyboardEvent) => {
    keys.add(e.code);
    if (["KeyW", "KeyA", "KeyS", "KeyD", "Space"].includes(e.code)) e.preventDefault();
  };
  const onKeyUp = (e: KeyboardEvent) => keys.delete(e.code);
  window.addEventListener("keydown", onKeyDown);
  window.addEventListener("keyup", onKeyUp);
  canvas.tabIndex = 0;
  canvas.style.outline = "none";

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
    const wire = wasm.character_terrain_wireframe();
    const positions = trianglesFromWireframe(wire);
    terrainMesh = makeTriangleMesh(positions, 0x5a7a62, 0.9);
    terrainWire = makeWireEdges(wire, 0x2f4035, 0.3);
    demo.content.add(terrainMesh);
    demo.content.add(terrainWire);
  }

  function reset() {
    wasm.character_reset();
    rebuildTerrain();
  }

  reset();

  let frame = 0;
  const stop = runLoop(() => {
    let throttleX = 0;
    let throttleY = 0;
    if (keys.has("KeyW")) throttleX += 1;
    if (keys.has("KeyS")) throttleX -= 1;
    if (keys.has("KeyA")) throttleY -= 1;
    if (keys.has("KeyD")) throttleY += 1;
    const jump = keys.has("Space");
    const sprint = keys.has("ShiftLeft") || keys.has("ShiftRight");

    // Camera-relative forward on XZ (toward look target).
    const cam = demo.camera.position;
    const target = demo.controls.target;
    let fwdX = target.x - cam.x;
    let fwdZ = target.z - cam.z;
    const fl = Math.hypot(fwdX, fwdZ) || 1;
    fwdX /= fl;
    fwdZ /= fl;
    // Right = cross(up, forward) ≈ (-fwdZ, fwdX) for Y-up
    const rightX = -fwdZ;
    const rightZ = fwdX;

    wasm.character_set_input(throttleX, throttleY, jump, sprint, fwdX, fwdZ, rightX, rightZ);
    wasm.character_step(1 / 60, 4);
    const poses = wasm.character_poses();
    syncMeshesFromPoses(demo.content, pool, poses, { groundIndex: null });

    const status = wasm.character_status();
    const mx = status[0]!;
    const my = status[1]!;
    const mz = status[2]!;
    demo.controls.target.set(mx, my + 0.5, mz);

    frame += 1;
    if (frame % 10 === 0) {
      updateReadout(readout, [
        { label: "ground", value: status[6]! > 0.5 ? "yes" : "no" },
        { label: "sprint", value: status[7]! > 0.5 ? "yes" : "no" },
        { label: "y", value: my.toFixed(2) },
        { label: "speed", value: Math.hypot(status[3]!, status[4]!, status[5]!).toFixed(2) },
      ]);
    }
    demo.render();
  }, readout);

  return () => {
    stop();
    window.removeEventListener("keydown", onKeyDown);
    window.removeEventListener("keyup", onKeyUp);
    clearTerrain();
    disposeMeshPool(pool);
    demo.dispose();
  };
}
