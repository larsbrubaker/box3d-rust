// Character / Mover — BasicMover-style capsule mover with ground-ray debug.

import * as THREE from "three";
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

type Mode = "mover" | "village";

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Character Mover",
    "Capsule mover matching upstream <code>Character / Mover</code> (BasicMover): " +
      "WASD, jump, sprint, pogo ground ray, static capsules, and height-field terrain. " +
      "Village mode walks the compound tile ground from Compound / Village.",
    "WASD move · Space jump · Shift sprint · drag to orbit",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "Blue mover capsule + purple velocity + cyan pogo ray (hit = coral tip). " +
        "Static capsules and boxes stand in for the C mesh map / enemy-friendly props. " +
        "Click the canvas to focus keys.",
    ),
  );

  let mode: Mode = "mover";
  let villageGrid = 10;

  controls.appendChild(
    createButtonGroup(
      [
        { label: "Mover", value: "mover" },
        { label: "Village", value: "village" },
        { label: "Respawn", value: "restart" },
      ],
      "mover",
      (v) => {
        if (v === "restart") {
          reset();
          return;
        }
        mode = v as Mode;
        reset();
      },
    ),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { target: [7.5, 1, 9], distance: 14 });
  demo.camera.far = 400;
  demo.camera.updateProjectionMatrix();
  const pool = createMeshPool();
  // Mover capsule: blue like C DrawSolidCapsule
  pool.dynamicMat.color.setHex(0x2563eb);
  let terrainMesh: THREE.Mesh | null = null;
  let terrainWire: THREE.LineSegments | null = null;

  // Debug overlay: pogo ray + velocity
  const debugGeo = new THREE.BufferGeometry();
  const debugPositions = new Float32Array(12);
  debugGeo.setAttribute("position", new THREE.BufferAttribute(debugPositions, 3));
  const debugMat = new THREE.LineBasicMaterial({
    color: 0x22d3ee,
    linewidth: 2,
  });
  const debugLines = new THREE.LineSegments(debugGeo, debugMat);
  demo.dynamic.add(debugLines);

  const hitGeo = new THREE.BufferGeometry();
  const hitPos = new Float32Array(3);
  hitGeo.setAttribute("position", new THREE.BufferAttribute(hitPos, 3));
  const hitMat = new THREE.PointsMaterial({ color: 0xf87171, size: 0.18 });
  const hitPoint = new THREE.Points(hitGeo, hitMat);
  demo.dynamic.add(hitPoint);

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
    if (mode !== "mover") return;
    const wire = wasm.character_terrain_wireframe();
    if (wire.length < 18) return;
    const positions = trianglesFromWireframe(wire);
    terrainMesh = makeTriangleMesh(positions, 0x5a7a62, 0.9);
    terrainWire = makeWireEdges(wire, 0x2f4035, 0.3);
    demo.content.add(terrainMesh);
    demo.content.add(terrainWire);
  }

  function reset() {
    if (mode === "village") {
      wasm.character_reset_ex(1, villageGrid);
      const half = villageGrid * 4;
      demo.controls.target.set(0, 8, 0);
      demo.camera.position.set(half * 0.55, half * 0.4, half * 0.65);
    } else {
      wasm.character_reset_ex(0, 10);
      demo.controls.target.set(7.5, 1, 9);
      demo.camera.position.set(14, 6, 18);
    }
    demo.controls.update();
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

    const cam = demo.camera.position;
    const target = demo.controls.target;
    let fwdX = target.x - cam.x;
    let fwdZ = target.z - cam.z;
    const fl = Math.hypot(fwdX, fwdZ) || 1;
    fwdX /= fl;
    fwdZ /= fl;
    const rightX = -fwdZ;
    const rightZ = fwdX;

    wasm.character_set_input(throttleX, throttleY, jump, sprint, fwdX, fwdZ, rightX, rightZ);
    wasm.character_step(1 / 60, 4);
    const poses = wasm.character_poses();
    syncMeshesFromPoses(demo.content, pool, poses, { groundIndex: null });

    // Color the last mesh (mover) blue; static capsules keep bone palette.
    const n = Math.floor(poses.length / 15);
    if (n > 0) {
      const mover = pool.meshes[n - 1] as THREE.Mesh | undefined;
      if (mover && mover.material) {
        (mover.material as THREE.MeshStandardMaterial).color?.setHex?.(0x2563eb);
      }
    }

    const dbg = wasm.character_debug_lines();
    debugPositions[0] = dbg[0]!;
    debugPositions[1] = dbg[1]!;
    debugPositions[2] = dbg[2]!;
    debugPositions[3] = dbg[3]!;
    debugPositions[4] = dbg[4]!;
    debugPositions[5] = dbg[5]!;
    debugPositions[6] = dbg[6]!;
    debugPositions[7] = dbg[7]!;
    debugPositions[8] = dbg[8]!;
    debugPositions[9] = dbg[9]!;
    debugPositions[10] = dbg[10]!;
    debugPositions[11] = dbg[11]!;
    debugGeo.attributes.position!.needsUpdate = true;
    debugGeo.computeBoundingSphere();

    // Velocity segment uses purple; swap material color per segment is hard with one
    // LineSegments — keep cyan for pogo and tint velocity via a second pass on hit tip.
    const hit = dbg[12]! > 0.5;
    hitPoint.visible = hit;
    if (hit) {
      hitPos[0] = dbg[3]!;
      hitPos[1] = dbg[4]!;
      hitPos[2] = dbg[5]!;
      hitGeo.attributes.position!.needsUpdate = true;
    }

    const status = wasm.character_status();
    const mx = status[0]!;
    const my = status[1]!;
    const mz = status[2]!;
    demo.controls.target.set(mx, my + 0.5, mz);

    frame += 1;
    if (frame % 10 === 0) {
      updateReadout(readout, [
        { label: "mode", value: mode },
        { label: "ground", value: status[6]! > 0.5 ? "yes" : "no" },
        { label: "sprint", value: status[7]! > 0.5 ? "yes" : "no" },
        { label: "y", value: my.toFixed(2) },
        { label: "speed", value: Math.hypot(status[3]!, status[4]!, status[5]!).toFixed(2) },
        { label: "pogo", value: hit ? "hit" : "air" },
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
    debugGeo.dispose();
    debugMat.dispose();
    hitGeo.dispose();
    hitMat.dispose();
    demo.dispose();
  };
}
