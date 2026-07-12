// Far Pyramid — World sample (`sample_world.cpp` FarPyramid).

import * as THREE from "three";
import {
  attachInteraction,
  type InteractWasm,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm, type Box3dWasm } from "../wasm.ts";
import { createCanvasOverlay } from "../controls.ts";
import { demoPage, runLoop } from "./common.ts";
import {
  DEBUG_BODY_COLORS,
  DemoScene,
  makeBodyMaterial,
  setView,
} from "../three-scene.ts";

const STRIDE = 11;
const OFFSET_M = 10_000_000;

function farInteract(wasm: Box3dWasm): InteractWasm {
  return {
    sim_step: (dt, ss) => wasm.world_far_pyramid_step(dt, ss),
    sim_body_poses: () => wasm.world_far_pyramid_poses(),
    sim_mouse_down: (ox, oy, oz, tx, ty, tz) =>
      wasm.world_far_pyramid_mouse_down(ox, oy, oz, tx, ty, tz),
    sim_mouse_move: (px, py, pz) => wasm.world_far_pyramid_mouse_move(px, py, pz),
    sim_mouse_up: () => wasm.world_far_pyramid_mouse_up(),
    sim_mouse_active: () => wasm.world_far_pyramid_mouse_active(),
    sim_spawn_random: (ox, oy, oz, tx, ty, tz) =>
      wasm.world_far_pyramid_spawn_random(ox, oy, oz, tx, ty, tz),
    sim_delete_at_ray: (ox, oy, oz, tx, ty, tz) =>
      wasm.world_far_pyramid_delete_at_ray(ox, oy, oz, tx, ty, tz),
    sim_counters: () => wasm.world_far_pyramid_counters(),
    sim_debug_draw: (flags) => wasm.world_far_pyramid_debug_draw(flags),
    sim_step_count: () => wasm.world_far_pyramid_step_count(),
    sim_set_enable_sleep: (flag) => wasm.world_far_pyramid_set_enable_sleep(flag),
    sim_set_enable_warm_starting: (flag) =>
      wasm.world_far_pyramid_set_enable_warm_starting(flag),
    sim_set_enable_continuous: (flag) => wasm.world_far_pyramid_set_enable_continuous(flag),
    sim_set_recycle_distance: (m) => wasm.world_far_pyramid_set_recycle_distance(m),
  };
}

function makeSkyGradient(): THREE.Texture {
  const canvas = document.createElement("canvas");
  canvas.width = 2;
  canvas.height = 256;
  const ctx = canvas.getContext("2d")!;
  const g = ctx.createLinearGradient(0, 0, 0, 256);
  g.addColorStop(0, "#5a8fc4");
  g.addColorStop(0.55, "#9bb0c4");
  g.addColorStop(1, "#c4b8a0");
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, 2, 256);
  const tex = new THREE.CanvasTexture(canvas);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const interact = farInteract(wasm);
  const { canvas, controls, page } = demoPage(
    container,
    "Far Pyramid",
    "Pyramid built 10 000 km from the world origin — float vs double stress test " +
      "from <code>sample_world.cpp</code>.",
    "",
    wasm.version(),
    { category: "World", samplesShell: true },
  );

  const overlay = createCanvasOverlay(page);
  const dp = wasm.is_double_precision_build() ? "ON" : "OFF";
  const km = wasm.world_far_pyramid_offset_km();
  overlay.innerHTML = `double precision: ${dp}<br>pyramid built ${km.toFixed(0)} km from the world origin`;

  const demo = new DemoScene(canvas, {
    target: [0, 20, 0],
    distance: 60,
    fov: 50,
    shadowExtent: 80,
    gridSize: 80,
    gridDivisions: 40,
  });
  demo.camera.far = 500;
  demo.camera.updateProjectionMatrix();
  demo.scene.fog = new THREE.Fog(0x9bb0c4, 80, 220);
  const sky = makeSkyGradient();
  demo.scene.background = sky;

  const boxGeo = new THREE.BoxGeometry(2, 2, 2);
  const groundMat = makeBodyMaterial(0, true);
  const boxMat = makeBodyMaterial(2, true);
  boxMat.color.setHex(DEBUG_BODY_COLORS.dynamicAwake);

  let boxInstances: THREE.InstancedMesh | null = null;
  let groundMesh: THREE.Mesh | null = null;
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
    if (groundMesh) {
      demo.content.remove(groundMesh);
      groundMesh = null;
    }
  }

  function ensureBoxes(count: number) {
    if (boxInstances && boxInstances.count >= count) return;
    if (boxInstances) {
      demo.content.remove(boxInstances);
      boxInstances.dispose();
    }
    boxInstances = new THREE.InstancedMesh(boxGeo, boxMat, Math.max(count, 64));
    boxInstances.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    boxInstances.castShadow = true;
    boxInstances.receiveShadow = true;
    demo.content.add(boxInstances);
  }

  function reset() {
    clearVisuals();
    wasm.world_reset_far_pyramid();
    setView(demo, 40, -10, 60, [0, 20, 0]);
  }

  const ctrl = attachInteraction({
    wasm: interact,
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: "Far Pyramid",
    sampleCategory: "World",
    worldOrigin: [OFFSET_M, 0, 0],
  }) as SimControllerWithTick;

  reset();

  const stop = runLoop(() => {
    ctrl.tickFrame();
    const poses = wasm.world_far_pyramid_poses();
    const n = Math.floor(poses.length / STRIDE);
    const dyn = Math.max(0, n - 1);
    ensureBoxes(dyn);

    if (n > 0) {
      if (!groundMesh) {
        groundMesh = new THREE.Mesh(boxGeo, groundMat);
        groundMesh.receiveShadow = true;
        demo.content.add(groundMesh);
      }
      const o = 0;
      groundMesh.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
      _q.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
      groundMesh.quaternion.copy(_q);
      groundMesh.scale.set(poses[o + 7]!, poses[o + 8]!, poses[o + 9]!);
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

    demo.render();
  }, controls);

  return () => {
    ctrl.dispose();
    stop();
    clearVisuals();
    demo.dispose();
    sky.dispose();
    boxGeo.dispose();
    groundMat.dispose();
    boxMat.dispose();
  };
}
