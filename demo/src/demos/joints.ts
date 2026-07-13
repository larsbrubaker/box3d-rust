// Joints — Ball and Chain, Revolute, Gear Lift, Driving (sample_joint.cpp).

import type * as THREE from "three";
import { createInfoBox, createReadout, updateReadout } from "../controls.ts";
import {
  attachInteraction,
  type InteractWasm,
  type ParamDef,
  type ParamValues,
  type SimControllerWithTick,
} from "../interaction.ts";
import { DRIVE_TELEMETRY, REVOLUTE_ENERGY, getWasm, type Box3dWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import {
  DemoScene,
  makeTriangleMesh,
  makeWireEdges,
  setView,
  trianglesFromWireframe,
} from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

type Scene = "chain" | "revolute" | "gear" | "driving";

const SCENES: Scene[] = ["chain", "revolute", "gear", "driving"];

function jointAsInteract(wasm: Box3dWasm): InteractWasm {
  // `joint_debug_text` is a per-demo export the Rust agent may add; guard it.
  const jointDebugText = (wasm as unknown as { joint_debug_text?: () => string }).joint_debug_text;
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
    // Debug-flag mask + draw scales are GLOBAL wasm exports (shared across every
    // demo); forward them so the View menu / panel drive the joint overlay too.
    sim_set_debug_flags: (m) => wasm.sim_set_debug_flags(m),
    sim_set_draw_scales: (j, f) => wasm.sim_set_draw_scales(j, f),
    sim_debug_text: jointDebugText ? () => jointDebugText.call(wasm) : undefined,
  };
}

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Joints",
    "Official Joints samples: Ball and Chain, Revolute, Gear Lift, and Driving — " +
      "ported from <code>sample_joint.cpp</code>.",
    "Ctrl+click grab · click select · P/O/R · WASD drives",
    wasm.version(),
    { category: "Joints", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "<strong>Ball and Chain</strong> — 32 spherical links + heavy ball.<br>" +
        "<strong>Revolute</strong> — hanging plank with limit / spring / motor.<br>" +
        "<strong>Gear Lift</strong> — meshed gears raise a gate over a stairwell basin.<br>" +
        "<strong>Driving</strong> — wheel-joint rover; WASD (or arrows) throttle / steer.",
    ),
  );

  // A deep link (`#/joints/<slug>`) can request a specific scene via initialScene.
  let scene: Scene =
    initialScene && SCENES.includes(initialScene as Scene) ? (initialScene as Scene) : "revolute";
  let ctrl!: SimControllerWithTick;

  // --- Per-sample control apply functions (mirror C member variables /
  // DrawControls). Values come straight from the declarative param panel. ---
  const num = (v: ParamValues, k: string) => v[k] as number;
  const bool = (v: ParamValues, k: string) => v[k] as boolean;

  function applyRevolute(v: ParamValues) {
    let flags = 0;
    if (bool(v, "revLimit")) flags |= 1;
    if (bool(v, "revMotor")) flags |= 2;
    if (bool(v, "revSpring")) flags |= 4;
    wasm.joint_set_revolute_params(
      flags,
      num(v, "revLower"),
      num(v, "revUpper"),
      num(v, "revSpeed"),
      num(v, "revTorque"),
      num(v, "revHertz"),
      num(v, "revDamping"),
      num(v, "revRotation"),
    );
  }
  function applyGear(v: ParamValues) {
    // Gear driver: motor only (no limit/spring). C GearLift::DrawControls.
    wasm.joint_set_revolute_params(
      bool(v, "gearMotor") ? 2 : 0,
      -35,
      35,
      num(v, "gearSpeed"),
      num(v, "gearTorque"),
      2,
      0.7,
      0,
    );
  }
  function applyDrivingSuspension(v: ParamValues) {
    wasm.joint_set_driving_suspension(
      num(v, "driveSuspMin"),
      num(v, "driveSuspMax"),
      num(v, "driveSuspHertz"),
      num(v, "driveSuspDamp"),
    );
  }
  function applyDrivingMotor(v: ParamValues) {
    wasm.joint_set_drive_params(num(v, "driveSpinSpeed"), num(v, "driveMotorTorque"));
  }
  function applyDrivingSteering(v: ParamValues) {
    wasm.joint_set_driving_steering(
      num(v, "driveSteerHertz"),
      num(v, "driveSteerDamp"),
      num(v, "driveSteerTorque"),
      num(v, "driveSteerMinDeg"),
      num(v, "driveSteerMaxDeg"),
    );
  }

  const demo = new DemoScene(canvas, { target: [0, 2, 0], distance: 15 });
  const pool = createMeshPool();
  let terrainMesh: THREE.Mesh | null = null;
  let terrainWire: THREE.LineSegments | null = null;

  // --- Driving keyboard (WASD, arrows aliased). Handler runs before
  // attachInteraction's; for driving movement keys we swallow the event so S
  // (single-step) etc. don't fire while steering. ---
  const keys = new Set<string>();
  const driveKeys = new Set([
    "KeyW",
    "KeyA",
    "KeyS",
    "KeyD",
    "ArrowUp",
    "ArrowDown",
    "ArrowLeft",
    "ArrowRight",
  ]);
  const onKeyDown = (e: KeyboardEvent) => {
    const tag = (e.target as HTMLElement)?.tagName;
    if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
    keys.add(e.code);
    if (scene === "driving" && driveKeys.has(e.code)) {
      e.preventDefault();
      e.stopImmediatePropagation();
    }
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
    const fill = scene === "gear" ? 0x8fbc8f : 0x5a7a62;
    const edge = scene === "gear" ? 0x5f8f5f : 0x2f4035;
    terrainMesh = makeTriangleMesh(positions, fill, 0.9);
    terrainWire = makeWireEdges(wire, edge, 0.25);
    demo.content.add(terrainMesh);
    demo.content.add(terrainWire);
  }

  function setCameraForScene() {
    if (scene === "chain") {
      setView(demo, 180, 15, 50, [0, -20, 0]);
    } else if (scene === "revolute") {
      setView(demo, 45, 30, 15, [0, 2, 0]);
    } else if (scene === "gear") {
      setView(demo, 18, 12, 17, [-1.5, 4.5, 0]);
    } else {
      setView(demo, 25, 20, 7, [0, 2, 0]);
    }
  }

  function reset() {
    clearTerrain();
    const p = ctrl.params;
    if (scene === "chain") {
      wasm.joint_reset_chain();
    } else if (scene === "revolute") {
      wasm.joint_reset_hinge();
      applyRevolute(p);
    } else if (scene === "gear") {
      wasm.joint_reset_gear_lift();
      applyGear(p);
      rebuildTerrain();
    } else {
      wasm.joint_reset_driving();
      applyDrivingSuspension(p);
      applyDrivingMotor(p);
      applyDrivingSteering(p);
      rebuildTerrain();
    }
    setCameraForScene();
  }

  // --- Telemetry / energy HUD (C Render() DrawTextLine) ---
  const readout = createReadout();
  const driveNote = document.createElement("div");
  driveNote.className = "control-note";
  driveNote.textContent =
    "Drive with WASD (or arrow keys). Third-person (T) camera-follow lands in a later batch.";

  // Per-scene controls, expressed declaratively. `visibleWhen` keyed on the
  // `sample` selector hides each scene's group when it is not active; the local
  // gate checkboxes (Limit / Motor / Spring) add a second predicate for their
  // sliders. Defaults / ranges match the batch-1 values verified against C.
  const onRevolute = { key: "sample", equals: "revolute" };
  const onGear = { key: "sample", equals: "gear" };
  const onDriving = { key: "sample", equals: "driving" };
  const params: ParamDef[] = [
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
      default: scene,
      restart: true,
    },
    // Revolute (C RevoluteJoint::DrawControls) — Limit/Motor/Spring gate sliders.
    { type: "checkbox", key: "revLimit", label: "Limit", default: false, restart: false, group: "Revolute", visibleWhen: onRevolute },
    { type: "slider", key: "revLower", label: "Lower Angle °", min: -180, max: 180, step: 1, default: -35, restart: false, group: "Revolute", visibleWhen: [onRevolute, { key: "revLimit", equals: true }] },
    { type: "slider", key: "revUpper", label: "Upper Angle °", min: -180, max: 180, step: 1, default: 35, restart: false, group: "Revolute", visibleWhen: [onRevolute, { key: "revLimit", equals: true }] },
    { type: "checkbox", key: "revMotor", label: "Motor", default: false, restart: false, group: "Revolute", visibleWhen: onRevolute },
    { type: "slider", key: "revTorque", label: "Max Torque", min: 0, max: 50000, step: 100, default: 5000, restart: false, group: "Revolute", visibleWhen: [onRevolute, { key: "revMotor", equals: true }] },
    { type: "slider", key: "revSpeed", label: "Speed", min: -10, max: 10, step: 1, default: 0, restart: false, group: "Revolute", visibleWhen: [onRevolute, { key: "revMotor", equals: true }] },
    { type: "checkbox", key: "revSpring", label: "Spring", default: false, restart: false, group: "Revolute", visibleWhen: onRevolute },
    { type: "slider", key: "revHertz", label: "Hertz", min: 0, max: 10, step: 0.1, default: 2, restart: false, group: "Revolute", visibleWhen: [onRevolute, { key: "revSpring", equals: true }] },
    { type: "slider", key: "revDamping", label: "Damping", min: 0, max: 2, step: 0.1, default: 0.7, restart: false, group: "Revolute", visibleWhen: [onRevolute, { key: "revSpring", equals: true }] },
    { type: "slider", key: "revRotation", label: "Rotation °", min: -180, max: 180, step: 1, default: 0, restart: false, group: "Revolute", visibleWhen: [onRevolute, { key: "revSpring", equals: true }] },
    // Gear Lift (C GearLift::DrawControls) — motor toggle + always-on torque/speed.
    { type: "checkbox", key: "gearMotor", label: "Motor", default: true, restart: false, group: "Gear Lift", visibleWhen: onGear },
    { type: "slider", key: "gearTorque", label: "Max Torque", min: 0, max: 100000, step: 100, default: 30000, restart: false, group: "Gear Lift", visibleWhen: onGear },
    { type: "slider", key: "gearSpeed", label: "Speed", min: -0.3, max: 0.3, step: 0.01, default: -0.3, restart: false, group: "Gear Lift", visibleWhen: onGear },
    // Driving (C Driving::DrawControls) — Suspension / Motor / Steering sections.
    { type: "slider", key: "driveSuspMin", label: "Min", min: -10, max: 10, step: 0.1, default: -0.2, restart: false, group: "Suspension", visibleWhen: onDriving },
    { type: "slider", key: "driveSuspMax", label: "Max", min: -10, max: 10, step: 0.1, default: 0.2, restart: false, group: "Suspension", visibleWhen: onDriving },
    { type: "slider", key: "driveSuspHertz", label: "Hertz", min: 0, max: 10, step: 0.1, default: 4, restart: false, group: "Suspension", visibleWhen: onDriving },
    { type: "slider", key: "driveSuspDamp", label: "Damping", min: 0, max: 2, step: 0.1, default: 0.7, restart: false, group: "Suspension", visibleWhen: onDriving },
    { type: "slider", key: "driveMotorTorque", label: "Max Torque", min: 0, max: 100, step: 1, default: 5, restart: false, group: "Motor", visibleWhen: onDriving },
    { type: "slider", key: "driveSpinSpeed", label: "Speed", min: 0, max: 100, step: 1, default: 30, restart: false, group: "Motor", visibleWhen: onDriving },
    { type: "slider", key: "driveSteerHertz", label: "Hertz", min: 0, max: 10, step: 0.1, default: 10, restart: false, group: "Steering", visibleWhen: onDriving },
    { type: "slider", key: "driveSteerDamp", label: "Damping", min: 0, max: 2, step: 0.1, default: 0.7, restart: false, group: "Steering", visibleWhen: onDriving },
    { type: "slider", key: "driveSteerTorque", label: "Torque", min: 0, max: 20, step: 0.1, default: 5, restart: false, group: "Steering", visibleWhen: onDriving },
    { type: "slider", key: "driveSteerMinDeg", label: "Min Deg", min: -90, max: 0, step: 1, default: -45, restart: false, group: "Steering", visibleWhen: onDriving },
    { type: "slider", key: "driveSteerMaxDeg", label: "Max Deg", min: 0, max: 90, step: 1, default: 45, restart: false, group: "Steering", visibleWhen: onDriving },
  ];

  ctrl = attachInteraction({
    wasm: jointAsInteract(wasm),
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: "Joints",
    sampleCategory: "Joints",
    enableSpawnDelete: false,
    params,
    onParamsChange: (values, key) => {
      if (key === "sample") {
        scene = values.sample as Scene;
        driveNote.style.display = scene === "driving" ? "" : "none";
        // C clears the HUD text when leaving the scenes that print it.
        if (scene !== "revolute" && scene !== "driving") readout.innerHTML = "";
        return; // restart re-applies the active scene's controls via reset()
      }
      if (key.startsWith("rev")) {
        // C clamps the pair so lower ≤ upper (sample_joint.cpp RevoluteJoint).
        if (key === "revLower") values.revLower = Math.min(num(values, "revLower"), num(values, "revUpper"));
        if (key === "revUpper") values.revUpper = Math.max(num(values, "revUpper"), num(values, "revLower"));
        applyRevolute(values);
      } else if (key.startsWith("gear")) {
        applyGear(values);
      } else if (key.startsWith("driveSusp")) {
        if (key === "driveSuspMin") values.driveSuspMin = Math.min(num(values, "driveSuspMin"), num(values, "driveSuspMax"));
        if (key === "driveSuspMax") values.driveSuspMax = Math.max(num(values, "driveSuspMax"), num(values, "driveSuspMin"));
        applyDrivingSuspension(values);
      } else if (key === "driveSpinSpeed" || key === "driveMotorTorque") {
        applyDrivingMotor(values);
      } else if (key.startsWith("driveSteer")) {
        applyDrivingSteering(values);
      }
    },
  }) as SimControllerWithTick;

  driveNote.style.display = scene === "driving" ? "" : "none";
  controls.appendChild(readout);
  controls.appendChild(driveNote);

  reset();

  let frame = 0;
  const stop = runLoop(() => {
    if (scene === "driving") {
      let tx = 0;
      let ty = 0;
      if (keys.has("KeyW") || keys.has("ArrowUp")) tx += 1;
      if (keys.has("KeyS") || keys.has("ArrowDown")) tx -= 1;
      if (keys.has("KeyA") || keys.has("ArrowLeft")) ty += 1;
      if (keys.has("KeyD") || keys.has("ArrowRight")) ty -= 1;
      wasm.joint_set_drive_input(tx, ty);
    } else {
      wasm.joint_set_drive_input(0, 0);
    }

    ctrl.tickFrame();
    const poses = wasm.joint_poses();
    syncMeshesFromPoses(demo.content, pool, poses, {
      groundIndex: scene === "driving" || scene === "gear" ? null : 0,
      styles: wasm.joint_styles(),
    });

    frame += 1;
    // Throttle the telemetry HUD to every 10 frames (matches sensors.ts); camera
    // tracking below still runs every frame so the driving chase-cam stays smooth.
    if (scene === "revolute") {
      if (frame % 10 === 0) {
        const e = wasm.joint_revolute_energy();
        updateReadout(readout, [
          { label: "kinetic energy", value: (e[REVOLUTE_ENERGY.kinetic] ?? 0).toPrecision(4) },
          { label: "potential energy", value: (e[REVOLUTE_ENERGY.potential] ?? 0).toPrecision(4) },
          { label: "total energy", value: (e[REVOLUTE_ENERGY.total] ?? 0).toPrecision(4) },
        ]);
      }
    } else if (scene === "driving") {
      if (frame % 10 === 0) {
        const t = wasm.joint_drive_telemetry();
        updateReadout(readout, [
          { label: "speed", value: (t[DRIVE_TELEMETRY.speed] ?? 0).toFixed(1) },
          {
            label: "spin speed",
            value: `${(t[DRIVE_TELEMETRY.spinL] ?? 0).toFixed(1)}/${(t[DRIVE_TELEMETRY.spinR] ?? 0).toFixed(1)}`,
          },
          {
            label: "spin torque",
            value: `${(t[DRIVE_TELEMETRY.spinTorqueL] ?? 0).toFixed(1)}/${(t[DRIVE_TELEMETRY.spinTorqueR] ?? 0).toFixed(1)}`,
          },
          {
            label: "steering °",
            value: `${(t[DRIVE_TELEMETRY.steerL] ?? 0).toFixed(1)}/${(t[DRIVE_TELEMETRY.steerR] ?? 0).toFixed(1)}`,
          },
          {
            label: "steering torque",
            value: `${(t[DRIVE_TELEMETRY.steerTorqueL] ?? 0).toFixed(1)}/${(t[DRIVE_TELEMETRY.steerTorqueR] ?? 0).toFixed(1)}`,
          },
        ]);
      }
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
