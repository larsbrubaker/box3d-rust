// Benchmark — Large Pyramid, Junkyard, Falling Trees (sample_benchmark / benchmarks.c).

import * as THREE from "three";
import { createButtonGroup, createInfoBox } from "../controls.ts";
import {
  attachInteraction,
  type InteractWasm,
  type ParamValues,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm, type Box3dWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import {
  COLORS,
  DemoScene,
  makeTriangleMesh,
  makeWireEdges,
  trianglesFromWireframe,
} from "../three-scene.ts";

const STRIDE = 11;

type Mode = "pyramid" | "junkyard" | "trees";

/** Adapt bench_* wasm exports to the shared InteractWasm surface. */
function benchInteract(wasm: Box3dWasm): InteractWasm {
  return {
    sim_step: (dt, ss) => wasm.bench_step(dt, ss),
    sim_body_poses: () => wasm.bench_body_poses(),
    sim_mouse_down: (ox, oy, oz, tx, ty, tz) => wasm.bench_mouse_down(ox, oy, oz, tx, ty, tz),
    sim_mouse_move: (px, py, pz) => wasm.bench_mouse_move(px, py, pz),
    sim_mouse_up: () => wasm.bench_mouse_up(),
    sim_mouse_active: () => wasm.bench_mouse_active(),
    sim_spawn_random: (ox, oy, oz, tx, ty, tz) => wasm.bench_spawn_random(ox, oy, oz, tx, ty, tz),
    sim_delete_at_ray: (ox, oy, oz, tx, ty, tz) => wasm.bench_delete_at_ray(ox, oy, oz, tx, ty, tz),
    sim_counters: () => wasm.bench_counters(),
    sim_debug_draw: (flags) => wasm.bench_debug_draw(flags),
  };
}

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const interact = benchInteract(wasm);
  const { canvas, controls } = demoPage(
    container,
    "Benchmark",
    "High-visibility Benchmark samples — Large Pyramid, Junkyard, and Falling Trees — " +
      "ported from <code>benchmarks.c</code> with browser-scaled body counts.",
    "Drag body · Shift spawn · Ctrl delete · Space/S/R",
    wasm.version(),
    { category: "Benchmark", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "Scale vs C: Large Pyramid base≈24 (C full=90 / DEBUG=20). Junkyard 2×8×8 rocks " +
        "(C DEBUG 2×21×21). Trees mesh 40×50 with 6×8 cylinders (C Trees25=600×800, " +
        "DEBUG 10×22). Sleep disabled on the pyramid like C.",
    ),
  );

  let mode: Mode = "pyramid";
  let baseCount = 24;

  const demo = new DemoScene(canvas, { target: [0, 8, 0], distance: 55, fov: 50 });
  demo.camera.far = 500;
  demo.camera.updateProjectionMatrix();

  const boxGeo = new THREE.BoxGeometry(2, 2, 2);
  const cylGeo = new THREE.CylinderGeometry(1, 1, 2, 12);
  const groundMat = new THREE.MeshStandardMaterial({
    color: 0x9aa3b2,
    roughness: 0.92,
    metalness: 0.05,
  });
  const boxMat = new THREE.MeshStandardMaterial({
    color: COLORS.accent,
    roughness: 0.45,
    metalness: 0.1,
  });
  const rockMat = new THREE.MeshStandardMaterial({
    color: 0x7a6a58,
    roughness: 0.85,
    metalness: 0.05,
  });
  const cylMat = new THREE.MeshStandardMaterial({
    color: 0x3d8b6e,
    roughness: 0.55,
    metalness: 0.08,
  });
  const pusherMat = new THREE.MeshStandardMaterial({
    color: 0xc45c5c,
    roughness: 0.4,
    metalness: 0.15,
  });

  let boxInstances: THREE.InstancedMesh | null = null;
  let rockInstances: THREE.InstancedMesh | null = null;
  const looseMeshes: THREE.Mesh[] = [];
  let terrainMesh: THREE.Mesh | null = null;
  let terrainWire: THREE.LineSegments | null = null;

  const _m = new THREE.Matrix4();
  const _p = new THREE.Vector3();
  const _q = new THREE.Quaternion();
  const _s = new THREE.Vector3();

  function clearVisuals() {
    if (boxInstances) {
      demo.content.remove(boxInstances);
      boxInstances.dispose();
      boxInstances = null;
    }
    if (rockInstances) {
      demo.content.remove(rockInstances);
      rockInstances.dispose();
      rockInstances = null;
    }
    for (const m of looseMeshes) {
      demo.content.remove(m);
      if (m.geometry !== boxGeo && m.geometry !== cylGeo) m.geometry.dispose();
    }
    looseMeshes.length = 0;
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

  function setCameraForMode() {
    if (mode === "pyramid") {
      demo.controls.target.set(0, 8, 0);
      demo.camera.position.set(18, 22, 48);
    } else if (mode === "junkyard") {
      demo.controls.target.set(0, 10, 0);
      demo.camera.position.set(45, 40, 55);
    } else {
      demo.controls.target.set(0, 4, -4);
      demo.camera.position.set(18, 14, 28);
    }
    demo.controls.update();
  }

  function rebuildTerrain() {
    if (mode !== "trees") return;
    const wire = wasm.bench_mesh_wireframe();
    if (wire.length === 0) return;
    const positions = trianglesFromWireframe(wire);
    terrainMesh = makeTriangleMesh(positions, 0x6b8f71, 0.85);
    terrainWire = makeWireEdges(wire, 0x3d4f44, 0.35);
    demo.content.add(terrainMesh);
    demo.content.add(terrainWire);
  }

  function reset() {
    clearVisuals();
    if (mode === "pyramid") wasm.bench_reset_large_pyramid(baseCount);
    else if (mode === "junkyard") wasm.bench_reset_junkyard();
    else wasm.bench_reset_trees();
    rebuildTerrain();
    setCameraForMode();
  }

  controls.appendChild(
    createButtonGroup(
      [
        { label: "Large Pyramid", value: "pyramid" },
        { label: "Junkyard", value: "junkyard" },
        { label: "Falling Trees", value: "trees" },
      ],
      "pyramid",
      (v) => {
        mode = v as Mode;
        reset();
      },
    ),
  );

  const ctrl = attachInteraction({
    wasm: interact,
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: "Benchmark",
    sampleCategory: "Benchmark",
    params: [
      {
        type: "slider",
        key: "base",
        label: "Pyramid base",
        min: 10,
        max: 36,
        step: 2,
        default: 24,
        restart: true,
      },
    ],
    onParamsChange: (values: ParamValues) => {
      baseCount = Math.round(Number(values.base) || 24);
    },
  }) as SimControllerWithTick;

  reset();

  function ensureBoxInstances(count: number) {
    if (boxInstances && boxInstances.count >= count) return;
    if (boxInstances) {
      demo.content.remove(boxInstances);
      boxInstances.dispose();
    }
    boxInstances = new THREE.InstancedMesh(boxGeo, boxMat, Math.max(count, 64));
    boxInstances.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    demo.content.add(boxInstances);
  }

  function ensureRockInstances(count: number) {
    if (rockInstances && rockInstances.count >= count) return;
    if (rockInstances) {
      demo.content.remove(rockInstances);
      rockInstances.dispose();
    }
    rockInstances = new THREE.InstancedMesh(boxGeo, rockMat, Math.max(count, 64));
    rockInstances.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    demo.content.add(rockInstances);
  }

  function syncLoose(i: number, kind: number, isGround: boolean, isPusher: boolean): THREE.Mesh {
    let mesh = looseMeshes[i];
    const wantCyl = kind === 3;
    if (!mesh || (wantCyl && mesh.geometry !== cylGeo) || (!wantCyl && mesh.geometry !== boxGeo)) {
      if (mesh) {
        demo.content.remove(mesh);
        if (mesh.geometry !== boxGeo && mesh.geometry !== cylGeo) mesh.geometry.dispose();
      }
      const mat = isPusher ? pusherMat : isGround ? groundMat : wantCyl ? cylMat : boxMat;
      mesh = new THREE.Mesh(wantCyl ? cylGeo : boxGeo, mat);
      demo.content.add(mesh);
      looseMeshes[i] = mesh;
    } else {
      mesh.material = isPusher ? pusherMat : isGround ? groundMat : wantCyl ? cylMat : boxMat;
    }
    return mesh;
  }

  const stop = runLoop(() => {
    ctrl.tickFrame();
    const poses = wasm.bench_body_poses();
    const n = Math.floor(poses.length / STRIDE);

    if (mode === "pyramid") {
      // Index 0 is ground; rest are dynamic boxes — instance the dynamics.
      const dyn = Math.max(0, n - 1);
      ensureBoxInstances(dyn);
      // Ground as loose mesh
      while (looseMeshes.length > 1) {
        const m = looseMeshes.pop()!;
        demo.content.remove(m);
      }
      if (n > 0) {
        const o = 0;
        const mesh = syncLoose(0, 0, true, false);
        mesh.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
        _q.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
        mesh.quaternion.copy(_q);
        mesh.scale.set(poses[o + 7]!, poses[o + 8]!, poses[o + 9]!);
      }
      for (let i = 1; i < n; i++) {
        const o = i * STRIDE;
        _p.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
        _q.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
        _s.set(poses[o + 7]!, poses[o + 8]!, poses[o + 9]!);
        _m.compose(_p, _q, _s);
        boxInstances!.setMatrixAt(i - 1, _m);
      }
      if (boxInstances) {
        boxInstances.count = dyn;
        boxInstances.instanceMatrix.needsUpdate = true;
      }
      if (rockInstances) rockInstances.visible = false;
      if (boxInstances) boxInstances.visible = true;
    } else if (mode === "junkyard") {
      // First 5 visuals: floor + 4 walls (ground body). Then rocks, then pusher cylinder.
      const staticCount = 5;
      const rockEnd = n - 1; // last is pusher
      const rockCount = Math.max(0, rockEnd - staticCount);
      ensureRockInstances(rockCount);

      while (looseMeshes.length > staticCount + 1) {
        const m = looseMeshes.pop()!;
        demo.content.remove(m);
      }

      for (let i = 0; i < Math.min(staticCount, n); i++) {
        const o = i * STRIDE;
        const mesh = syncLoose(i, 0, true, false);
        mesh.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
        _q.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
        mesh.quaternion.copy(_q);
        mesh.scale.set(poses[o + 7]!, poses[o + 8]!, poses[o + 9]!);
      }

      for (let i = 0; i < rockCount; i++) {
        const o = (staticCount + i) * STRIDE;
        _p.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
        _q.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
        _s.set(poses[o + 7]!, poses[o + 8]!, poses[o + 9]!);
        _m.compose(_p, _q, _s);
        rockInstances!.setMatrixAt(i, _m);
      }
      if (rockInstances) {
        rockInstances.count = rockCount;
        rockInstances.instanceMatrix.needsUpdate = true;
        rockInstances.visible = true;
      }
      if (boxInstances) boxInstances.visible = false;

      if (n > staticCount) {
        const o = (n - 1) * STRIDE;
        const kind = poses[o + 10]!;
        const mesh = syncLoose(staticCount, kind, false, true);
        mesh.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
        _q.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
        mesh.quaternion.copy(_q);
        // CylinderGeometry unit: radius 1, height 2 → scale (r, halfH, r)
        mesh.scale.set(poses[o + 7]!, poses[o + 8]!, poses[o + 9]!);
      }
    } else {
      // Trees: cylinders as loose meshes (few bodies).
      if (boxInstances) boxInstances.visible = false;
      if (rockInstances) rockInstances.visible = false;
      while (looseMeshes.length > n) {
        const m = looseMeshes.pop()!;
        demo.content.remove(m);
      }
      for (let i = 0; i < n; i++) {
        const o = i * STRIDE;
        const kind = poses[o + 10]!;
        const mesh = syncLoose(i, kind, false, false);
        mesh.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
        _q.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
        mesh.quaternion.copy(_q);
        mesh.scale.set(poses[o + 7]!, poses[o + 8]!, poses[o + 9]!);
      }
    }

    demo.render();
  }, controls);

  return () => {
    ctrl.dispose();
    stop();
    clearVisuals();
    demo.dispose();
    boxGeo.dispose();
    cylGeo.dispose();
    groundMat.dispose();
    boxMat.dispose();
    rockMat.dispose();
    cylMat.dispose();
    pusherMat.dispose();
  };
}
