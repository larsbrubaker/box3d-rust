// Joints — Ball and Chain, Revolute, Gear Lift, Driving (sample_joint.cpp).

import type * as THREE from "three";
import { createInfoBox, createSlider } from "../controls.ts";
import {
  attachInteraction,
  type InteractWasm,
  type ParamValues,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm, type Box3dWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import {
  DemoScene,
  makeTriangleMesh,
  makeWireEdges,
  trianglesFromWireframe,
} from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

type Scene = "chain" | "revolute" | "gear" | "driving";

function jointAsInteract(wasm: Box3dWasm): InteractWasm {
  return {
    sim_step: (dt, n) => wasm.joint_step(dt, n),
    sim_body_poses: () => wasm.joint_poses(),
    sim_mouse_down: (ox, oy, oz, tx, ty, tz) => wasm.joint_mouse_down(ox, oy, oz, tx, ty, tz),
    sim_mouse_move: (px, py, pz) => wasm.joint_mouse_move(px, py, pz),
    sim_mouse_up: () => wasm.joint_mouse_up(),
    sim_mouse_active: () => wasm.joint_mouse_active(),
    sim_spawn_random: (ox, oy, oz, tx, ty, tz) => wasm.joint_spawn_random(ox, oy, oz, tx, ty, tz),
    sim_delete_at_ray: (ox, oy, oz, tx, ty, tz) => wasm.joint_delete_at_ray(ox, oy, oz, tx, ty, tz),
    sim_counters: () => wasm.joint_counters(),
    sim_debug_draw: (flags) => wasm.joint_debug_draw(flags),
  };
}

function applyRevolute(wasm: Box3dWasm, p: ParamValues) {
  let flags = 0;
  if (p.limit) flags |= 1;
  if (p.motor) flags |= 2;
  if (p.spring) flags |= 4;
  wasm.joint_set_revolute_params(
    flags,
    Number(p.lowerDeg) || -35,
    Number(p.upperDeg) || 35,
    Number(p.motorSpeed) || 0,
    Number(p.motorTorque) || 5000,
    Number(p.hertz) || 2,
    Number(p.damping) || 0.7,
    Number(p.targetDeg) || 0,
  );
}

function cameraFromView(
  demo: DemoScene,
  yawDeg: number,
  pitchDeg: number,
  distance: number,
  target: [number, number, number],
) {
  demo.controls.target.set(target[0], target[1], target[2]);
  const yaw = (yawDeg * Math.PI) / 180;
  const pitch = (pitchDeg * Math.PI) / 180;
  demo.camera.position.set(
    target[0] + distance * Math.cos(pitch) * Math.sin(yaw),
    target[1] + distance * Math.sin(pitch),
    target[2] + distance * Math.cos(pitch) * Math.cos(yaw),
  );
  demo.controls.update();
}

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Joints",
    "Official Joints samples: Ball and Chain, Revolute, Gear Lift, and Driving — " +
      "ported from <code>sample_joint.cpp</code>.",
    "Pick a sample · drag bodies · Space/S/R · arrows drive",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "<strong>Ball and Chain</strong> — spherical links.<br>" +
        "<strong>Revolute</strong> — hanging plank with limit / spring / motor.<br>" +
        "<strong>Gear Lift</strong> — meshed gears raise a gate (simplified basin).<br>" +
        "<strong>Driving</strong> — wheel-joint rover; Arrow keys throttle / steer.",
    ),
  );

  let scene: Scene = "revolute";
  let links = 12;
  let spinSpeed = 30;
  let maxSpinTorque = 5;
  let ctrl!: SimControllerWithTick;

  const demo = new DemoScene(canvas, { target: [0, 2, 0], distance: 15 });
  const pool = createMeshPool();
  let terrainMesh: THREE.Mesh | null = null;
  let terrainWire: THREE.LineSegments | null = null;

  const keys = new Set<string>();
  const onKeyDown = (e: KeyboardEvent) => {
    keys.add(e.code);
    if (scene === "driving" && e.code.startsWith("Arrow")) e.preventDefault();
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
    const wire = wasm.joint_terrain_wireframe();
    if (!wire.length) return;
    const positions = trianglesFromWireframe(wire);
    terrainMesh = makeTriangleMesh(positions, 0x5a7a62, 0.9);
    terrainWire = makeWireEdges(wire, 0x2f4035, 0.25);
    demo.content.add(terrainMesh);
    demo.content.add(terrainWire);
  }

  function setCameraForScene() {
    if (scene === "chain") {
      demo.controls.target.set(6, -4, 0);
      demo.camera.position.set(18, 6, 22);
      demo.controls.update();
    } else if (scene === "revolute") {
      cameraFromView(demo, 45, 30, 15, [0, 2, 0]);
    } else if (scene === "gear") {
      cameraFromView(demo, 18, 12, 17, [-1.5, 4.5, 0]);
    } else {
      cameraFromView(demo, 25, 20, 12, [0, 2, 0]);
    }
  }

  function reset() {
    clearTerrain();
    if (scene === "chain") {
      wasm.joint_reset_chain(links);
    } else if (scene === "revolute") {
      wasm.joint_reset_hinge();
      applyRevolute(wasm, ctrl.params);
    } else if (scene === "gear") {
      wasm.joint_reset_gear_lift();
      // C defaults: motor on, speed -0.3, torque 30000
      applyRevolute(wasm, {
        limit: false,
        spring: false,
        motor: true,
        motorSpeed: -0.3,
        motorTorque: 30000,
        lowerDeg: -35,
        upperDeg: 35,
        hertz: 2,
        damping: 0.7,
        targetDeg: 0,
      });
    } else {
      wasm.joint_reset_driving();
      wasm.joint_set_drive_params(spinSpeed, maxSpinTorque);
      rebuildTerrain();
    }
    setCameraForScene();
  }

  ctrl = attachInteraction({
    wasm: jointAsInteract(wasm),
    demo,
    canvas,
    controls,
    onRestart: reset,
    enableSpawnDelete: false,
    params: [
      {
        type: "select",
        key: "sample",
        label: "Sample",
        options: [
          { label: "Ball and Chain", value: "chain" },
          { label: "Revolute", value: "revolute" },
          { label: "Gear Lift", value: "gear" },
          { label: "Driving", value: "driving" },
        ],
        default: "revolute",
        restart: true,
      },
      {
        type: "slider",
        key: "links",
        label: "Chain links",
        min: 4,
        max: 20,
        step: 1,
        default: 12,
        restart: true,
      },
      {
        type: "checkbox",
        key: "limit",
        label: "Revolute limit",
        default: false,
        restart: false,
      },
      {
        type: "slider",
        key: "lowerDeg",
        label: "Lower angle °",
        min: -180,
        max: 180,
        step: 1,
        default: -35,
        restart: false,
      },
      {
        type: "slider",
        key: "upperDeg",
        label: "Upper angle °",
        min: -180,
        max: 180,
        step: 1,
        default: 35,
        restart: false,
      },
      {
        type: "checkbox",
        key: "motor",
        label: "Motor",
        default: false,
        restart: false,
      },
      {
        type: "slider",
        key: "motorSpeed",
        label: "Motor speed",
        min: -10,
        max: 10,
        step: 0.1,
        default: 0,
        restart: false,
      },
      {
        type: "slider",
        key: "motorTorque",
        label: "Max motor torque",
        min: 0,
        max: 50000,
        step: 100,
        default: 5000,
        restart: false,
      },
      {
        type: "checkbox",
        key: "spring",
        label: "Spring",
        default: false,
        restart: false,
      },
      {
        type: "slider",
        key: "hertz",
        label: "Spring hertz",
        min: 0,
        max: 10,
        step: 0.1,
        default: 2,
        restart: false,
      },
      {
        type: "slider",
        key: "damping",
        label: "Spring damping",
        min: 0,
        max: 2,
        step: 0.1,
        default: 0.7,
        restart: false,
      },
      {
        type: "slider",
        key: "targetDeg",
        label: "Target angle °",
        min: -180,
        max: 180,
        step: 1,
        default: 0,
        restart: false,
      },
    ],
    onParamsChange: (values, key) => {
      if (key === "sample") {
        scene = values.sample as Scene;
        return;
      }
      if (key === "links") {
        links = Math.round(Number(values.links) || 12);
        return;
      }
      if (scene === "revolute") {
        applyRevolute(wasm, values);
      } else if (scene === "gear" && (key === "motor" || key === "motorSpeed" || key === "motorTorque")) {
        applyRevolute(wasm, {
          limit: false,
          spring: false,
          motor: values.motor ?? true,
          motorSpeed: Number(values.motorSpeed) || -0.3,
          motorTorque: Number(values.motorTorque) || 30000,
          lowerDeg: -35,
          upperDeg: 35,
          hertz: 2,
          damping: 0.7,
          targetDeg: 0,
        });
      }
    },
  }) as SimControllerWithTick;

  controls.appendChild(
    createSlider("Drive spin speed", 0, 100, spinSpeed, 1, (v) => {
      spinSpeed = v;
      if (scene === "driving") wasm.joint_set_drive_params(spinSpeed, maxSpinTorque);
    }),
  );
  controls.appendChild(
    createSlider("Drive max torque", 0, 100, maxSpinTorque, 1, (v) => {
      maxSpinTorque = v;
      if (scene === "driving") wasm.joint_set_drive_params(spinSpeed, maxSpinTorque);
    }),
  );

  reset();

  const readout = controls.querySelector(".info-readout") as HTMLElement;
  const stop = runLoop(() => {
    if (scene === "driving") {
      let tx = 0;
      let ty = 0;
      // Arrow keys only — avoids clashing with attachInteraction S=step.
      if (keys.has("ArrowUp")) tx += 1;
      if (keys.has("ArrowDown")) tx -= 1;
      if (keys.has("ArrowLeft")) ty += 1;
      if (keys.has("ArrowRight")) ty -= 1;
      wasm.joint_set_drive_input(tx, ty);
    } else {
      wasm.joint_set_drive_input(0, 0);
    }

    ctrl.tickFrame();
    const poses = wasm.joint_poses();
    syncMeshesFromPoses(demo.content, pool, poses, {
      groundIndex: scene === "driving" ? null : 0,
    });

    if (scene === "driving") {
      const cp = wasm.joint_chassis_pose();
      demo.controls.target.set(cp[0]!, cp[1]! + 0.5, cp[2]!);
    }

    demo.render();
  }, readout);

  return () => {
    stop();
    window.removeEventListener("keydown", onKeyDown);
    window.removeEventListener("keyup", onKeyUp);
    clearTerrain();
    ctrl.dispose();
    disposeMeshPool(pool);
    demo.dispose();
  };
}
