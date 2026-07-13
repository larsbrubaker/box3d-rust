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
import { applyShapeStyle, DemoScene, makeShapeMaterial, setView } from "../three-scene.ts";

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
  // No fog / no background texture: the Preetham sky dome (DemoScene.env.sky)
  // already fills the background and owns the horizon, so a background texture is
  // dead (occluded by the dome) and fog clashes with the sky's horizon band. C
  // has no fog here either — it culls with drawDistance, not distance fog.

  const boxGeo = new THREE.BoxGeometry(2, 2, 2);
  // The pyramid boxes render as one InstancedMesh (single material), so they
  // share one representative engine style (the first dynamic box); the ground
  // takes its own. Both come from the [groundStyle, boxStyle] style pair below.
  const groundMat = makeShapeMaterial();
  const boxMat = makeShapeMaterial();

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

  // batch-2 fix #7: this demo consumes only [groundStyle, boxStyle], so prefer
  // the Rust `world_far_pyramid_style_pair()` (2 words) over the full-engine
  // `world_far_pyramid_styles()` (~2870 words) when the sibling export is
  // present. Feature-checked so the demo keeps building until it lands; the
  // wasm.ts typed declaration is owned by the wasm/interaction sibling, so a
  // local cast bridges the gap here.
  const stylePairFn = (
    wasm as unknown as { world_far_pyramid_style_pair?: () => Uint32Array }
  ).world_far_pyramid_style_pair?.bind(wasm);

  const stop = runLoop(() => {
    ctrl.tickFrame();
    const poses = wasm.world_far_pyramid_poses();
    const styles = stylePairFn ? stylePairFn() : wasm.world_far_pyramid_styles();
    const n = Math.floor(poses.length / STRIDE);
    const dyn = Math.max(0, n - 1);
    ensureBoxes(dyn);

    // The pyramid boxes render as one InstancedMesh, so a single representative
    // style (body 1) colors every instance.
    if (boxInstances && n > 1) applyShapeStyle(boxInstances, styles[1]!);

    if (n > 0) {
      if (!groundMesh) {
        groundMesh = new THREE.Mesh(boxGeo, groundMat);
        groundMesh.receiveShadow = true;
        demo.content.add(groundMesh);
      }
      applyShapeStyle(groundMesh, styles[0]!); // ground is body 0
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
    boxGeo.dispose();
    groundMat.dispose();
    boxMat.dispose();
  };
}
