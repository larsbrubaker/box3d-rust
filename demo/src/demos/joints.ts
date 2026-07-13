// Joints — Ball and Chain, Revolute, Gear Lift, Driving (sample_joint.cpp).

import type * as THREE from "three";
import { createButton, createInfoBox, createReadout, updateReadout } from "../controls.ts";
import {
  attachInteraction,
  makeInteractAdapter,
  type ParamDef,
  type ParamValues,
  type SimControllerWithTick,
} from "../interaction.ts";
import { DOOR_READOUT, DRIVE_TELEMETRY, MOTOR_READOUT, REVOLUTE_ENERGY, getWasm } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { demoPage, makeStyleGate, runLoop } from "./common.ts";
import {
  DemoScene,
  makeTriangleMesh,
  makeWireEdges,
  setView,
  trianglesFromWireframe,
} from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

type Scene =
  | "chain"
  | "revolute"
  | "gear"
  | "driving"
  | "distance"
  | "filter"
  | "motor"
  | "top-down-friction"
  | "prismatic"
  | "spherical"
  | "parallel"
  | "weld"
  | "wheel"
  | "door"
  | "bridge"
  | "motion-locks";

export const SCENES: Scene[] = [
  "chain",
  "revolute",
  "gear",
  "driving",
  "distance",
  "filter",
  "motor",
  "top-down-friction",
  "prismatic",
  "spherical",
  "parallel",
  "weld",
  "wheel",
  "door",
  "bridge",
  "motion-locks",
];

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  // Self-check the scene table against the registry (see registry.ts pattern).
  assertRouteScenes("joints", SCENES);
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
  // --- Batch-3b scene control appliers (each mirrors a C DrawControls). ---
  function applyDistance(v: ParamValues) {
    wasm.joint_set_distance_params(
      num(v, "distLength"),
      bool(v, "distSpring"),
      num(v, "distTension"),
      num(v, "distCompression"),
      num(v, "distHertz"),
      num(v, "distDamping"),
      bool(v, "distLimit"),
      num(v, "distMin"),
      num(v, "distMax"),
    );
  }
  function resetDistance(v: ParamValues) {
    // The C Count slider rebuilds the whole chain using the current members.
    wasm.joint_reset_distance(
      num(v, "distCount"),
      num(v, "distHertz"),
      num(v, "distDamping"),
      num(v, "distLength"),
      num(v, "distTension"),
      num(v, "distCompression"),
      num(v, "distMin"),
      num(v, "distMax"),
      bool(v, "distSpring"),
      bool(v, "distLimit"),
    );
    applyDistance(v);
  }
  function applyMotor(v: ParamValues) {
    wasm.joint_motor_set_params(num(v, "motorSpeed"), num(v, "motorForce"), num(v, "motorTorque"));
  }
  function applyPrismatic(v: ParamValues) {
    let flags = 0;
    if (bool(v, "priLimit")) flags |= 1;
    if (bool(v, "priMotor")) flags |= 2;
    if (bool(v, "priSpring")) flags |= 4;
    wasm.joint_set_prismatic_params(
      flags,
      num(v, "priLower"),
      num(v, "priUpper"),
      num(v, "priMaxForce"),
      num(v, "priSpeed"),
      num(v, "priHertz"),
      num(v, "priDamping"),
      num(v, "priTarget"),
    );
  }
  function applySpherical(v: ParamValues) {
    let flags = 0;
    if (bool(v, "sphCone")) flags |= 1;
    if (bool(v, "sphTwist")) flags |= 2;
    if (bool(v, "sphMotor")) flags |= 4;
    if (bool(v, "sphSpring")) flags |= 8;
    wasm.joint_set_spherical_params(
      flags,
      num(v, "sphConeDeg"),
      num(v, "sphLowerTwist"),
      num(v, "sphUpperTwist"),
      num(v, "sphMaxTorque"),
      num(v, "sphVelX"),
      num(v, "sphVelY"),
      num(v, "sphVelZ"),
      num(v, "sphHertz"),
      num(v, "sphDamping"),
      num(v, "sphRotX"),
      num(v, "sphRotY"),
      num(v, "sphRotZ"),
    );
  }
  function applyParallel(v: ParamValues) {
    wasm.joint_set_parallel_params(num(v, "parHertz"), num(v, "parDamping"));
  }
  function applyWeld(v: ParamValues) {
    wasm.joint_set_weld_params(
      num(v, "weldLinHertz"),
      num(v, "weldLinDamp"),
      num(v, "weldAngHertz"),
      num(v, "weldAngDamp"),
    );
  }
  function applyWheel(v: ParamValues) {
    let flags = 0;
    if (bool(v, "whlSuspLimit")) flags |= 1;
    if (bool(v, "whlMotor")) flags |= 2;
    if (bool(v, "whlSuspSpring")) flags |= 4;
    if (bool(v, "whlSteering")) flags |= 8;
    if (bool(v, "whlSteerLimit")) flags |= 16;
    wasm.joint_set_wheel_params(
      flags,
      num(v, "whlSuspMin"),
      num(v, "whlSuspMax"),
      num(v, "whlMaxTorque"),
      num(v, "whlSpinSpeed"),
      num(v, "whlSuspHertz"),
      num(v, "whlSuspDamp"),
      num(v, "whlSteerHertz"),
      num(v, "whlSteerDamp"),
      num(v, "whlSteerDeg"),
      num(v, "whlSteerMinDeg"),
      num(v, "whlSteerMaxDeg"),
    );
  }
  function applyMotionLocks(v: ParamValues) {
    wasm.joint_set_motion_locks(
      bool(v, "mlLinX"),
      bool(v, "mlLinY"),
      bool(v, "mlLinZ"),
      bool(v, "mlAngX"),
      bool(v, "mlAngY"),
      bool(v, "mlAngZ"),
    );
  }

  const demo = new DemoScene(canvas, { target: [0, 2, 0], distance: 15 });
  const pool = createMeshPool();
  // Awake-gated cache for joint_styles() (a full world_draw capture): refetch
  // only while bodies are awake, on the first frame, and once on the settle so
  // sleep recoloring is still captured. joint_counters()[5] = awake dynamic count.
  const styleGate = makeStyleGate<Uint32Array>();
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

  // Per-scene camera, mirroring each sample's `m_camera->SetView(...)` exactly
  // (yaw, pitch, distance, target).
  const CAMERA: Record<Scene, [number, number, number, [number, number, number]]> = {
    chain: [180, 15, 50, [0, -20, 0]],
    revolute: [45, 30, 15, [0, 2, 0]],
    gear: [18, 12, 17, [-1.5, 4.5, 0]],
    driving: [25, 20, 7, [0, 2, 0]],
    distance: [0, 0, 40, [0, 10, 0]],
    filter: [45, 30, 15, [0, 2, 0]],
    motor: [0, 0, 25, [0, 8, 0]],
    "top-down-friction": [0, 0, 26, [0, 10, 0]],
    prismatic: [45, 30, 15, [0, 2, 0]],
    spherical: [45, 30, 15, [0, 2, 0]],
    parallel: [45, 30, 15, [0, 2, 0]],
    weld: [45, 30, 15, [0, 2, 0]],
    wheel: [25, 20, 7, [0, 2, 0]],
    door: [45, 30, 15, [0, 2, 0]],
    bridge: [0, 20, 35, [0, 10, 0]],
    "motion-locks": [0, 30, 40, [0, 5, 0]],
  };

  function setCameraForScene() {
    const [yaw, pitch, dist, target] = CAMERA[scene];
    setView(demo, yaw, pitch, dist, target);
  }

  function reset() {
    clearTerrain();
    const p = ctrl.params;
    switch (scene) {
      case "chain":
        wasm.joint_reset_chain();
        break;
      case "revolute":
        wasm.joint_reset_hinge();
        applyRevolute(p);
        break;
      case "gear":
        wasm.joint_reset_gear_lift();
        applyGear(p);
        rebuildTerrain();
        break;
      case "driving":
        wasm.joint_reset_driving();
        applyDrivingSuspension(p);
        applyDrivingMotor(p);
        applyDrivingSteering(p);
        rebuildTerrain();
        break;
      case "distance":
        resetDistance(p);
        break;
      case "filter":
        wasm.joint_reset_filter();
        break;
      case "motor":
        wasm.joint_reset_motor();
        applyMotor(p);
        break;
      case "top-down-friction":
        wasm.joint_reset_top_down_friction();
        break;
      case "prismatic":
        wasm.joint_reset_prismatic();
        applyPrismatic(p);
        break;
      case "spherical":
        wasm.joint_reset_spherical();
        applySpherical(p);
        break;
      case "parallel":
        // The joint is built at hertz=10 (C ctor); do NOT re-apply the 0-5 slider
        // on reset, so the initial stiffness matches C. The slider live-tunes 0-5.
        wasm.joint_reset_parallel();
        break;
      case "weld":
        wasm.joint_reset_weld();
        applyWeld(p);
        break;
      case "wheel":
        wasm.joint_reset_wheel();
        applyWheel(p);
        break;
      case "door":
        wasm.joint_reset_door(
          num(p, "doorMagnitude"),
          bool(p, "doorTwoJoints"),
          num(p, "doorHertz"),
          num(p, "doorDamping"),
        );
        wasm.joint_door_set_limit(bool(p, "doorLimit"));
        break;
      case "bridge":
        wasm.joint_reset_bridge();
        wasm.joint_set_bridge_gravity(num(p, "bridgeGravity"));
        break;
      case "motion-locks":
        wasm.joint_reset_motion_locks();
        applyMotionLocks(p);
        break;
    }
    updateSceneButtons();
    setCameraForScene();
  }

  // --- Telemetry / energy HUD (C Render() DrawTextLine) ---
  const readout = createReadout();
  const driveNote = document.createElement("div");
  driveNote.className = "control-note";
  driveNote.textContent =
    "Drive with WASD (or arrow keys). Third-person (T) camera-follow lands in a later batch.";

  // --- Per-scene action buttons (C DrawControls ImGui::Button). Shown/hidden by
  // `updateSceneButtons`. Door additionally supports Ctrl+click ray-launch. ---
  const buttonRow = document.createElement("div");
  buttonRow.className = "control-group";
  const motorImpulseBtn = createButton("Apply Impulse", () => wasm.joint_motor_apply_impulse());
  const explodeBtn = createButton("Explode", () => wasm.joint_explode());
  const doorImpulseBtn = createButton("Impulse", () => wasm.joint_door_impulse());
  buttonRow.append(motorImpulseBtn, explodeBtn, doorImpulseBtn);
  const doorNote = document.createElement("div");
  doorNote.className = "control-note";
  doorNote.textContent = "Ctrl+click a body to launch it (C Door::MouseDown ray-pick).";
  function updateSceneButtons() {
    motorImpulseBtn.style.display = scene === "motor" ? "" : "none";
    explodeBtn.style.display = scene === "top-down-friction" ? "" : "none";
    doorImpulseBtn.style.display = scene === "door" ? "" : "none";
    doorNote.style.display = scene === "door" ? "" : "none";
  }

  // Per-scene controls, expressed declaratively. `visibleWhen` keyed on the
  // `sample` selector hides each scene's group when it is not active; the local
  // gate checkboxes (Limit / Motor / Spring) add a second predicate for their
  // sliders. Defaults / ranges match the batch-1 values verified against C.
  const onRevolute = { key: "sample", equals: "revolute" };
  const onGear = { key: "sample", equals: "gear" };
  const onDriving = { key: "sample", equals: "driving" };
  const on = (s: Scene) => ({ key: "sample", equals: s });
  const params: ParamDef[] = [
    {
      type: "select",
      key: "sample",
      label: "Sample",
      options: [
        { label: "Distance Joint", value: "distance" },
        { label: "Filter", value: "filter" },
        { label: "Motor Joint", value: "motor" },
        { label: "Top Down Friction", value: "top-down-friction" },
        { label: "Prismatic", value: "prismatic" },
        { label: "Spherical", value: "spherical" },
        { label: "Parallel Spring", value: "parallel" },
        { label: "Revolute", value: "revolute" },
        { label: "Weld", value: "weld" },
        { label: "Wheel", value: "wheel" },
        { label: "Ball and Chain", value: "chain" },
        { label: "Door", value: "door" },
        { label: "Bridge", value: "bridge" },
        { label: "Motion Locks", value: "motion-locks" },
        { label: "Driving", value: "driving" },
        { label: "Gear Lift", value: "gear" },
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
    // Distance Joint (C DistanceJoint::DrawControls). Count rebuilds the chain.
    { type: "slider", key: "distLength", label: "Length", min: 0.1, max: 4, step: 0.1, default: 1, restart: false, group: "Distance", visibleWhen: on("distance") },
    { type: "checkbox", key: "distSpring", label: "Spring", default: false, restart: false, group: "Distance", visibleWhen: on("distance") },
    { type: "slider", key: "distTension", label: "Tension", min: 0, max: 4000, step: 10, default: 2000, restart: false, group: "Distance", visibleWhen: [on("distance"), { key: "distSpring", equals: true }] },
    { type: "slider", key: "distCompression", label: "Compression", min: 0, max: 200, step: 1, default: 100, restart: false, group: "Distance", visibleWhen: [on("distance"), { key: "distSpring", equals: true }] },
    { type: "slider", key: "distHertz", label: "Hertz", min: 0, max: 15, step: 0.1, default: 5, restart: false, group: "Distance", visibleWhen: [on("distance"), { key: "distSpring", equals: true }] },
    { type: "slider", key: "distDamping", label: "Damping", min: 0, max: 4, step: 0.1, default: 0.5, restart: false, group: "Distance", visibleWhen: [on("distance"), { key: "distSpring", equals: true }] },
    { type: "checkbox", key: "distLimit", label: "Limit", default: false, restart: false, group: "Distance", visibleWhen: on("distance") },
    { type: "slider", key: "distMin", label: "Min", min: 0.1, max: 4, step: 0.1, default: 1, restart: false, group: "Distance", visibleWhen: [on("distance"), { key: "distLimit", equals: true }] },
    { type: "slider", key: "distMax", label: "Max", min: 0.1, max: 4, step: 0.1, default: 1, restart: false, group: "Distance", visibleWhen: [on("distance"), { key: "distLimit", equals: true }] },
    { type: "slider", key: "distCount", label: "Count", min: 1, max: 20, step: 1, default: 1, restart: false, group: "Distance", visibleWhen: on("distance") },
    // Motor Joint (C MotorJoint::DrawControls).
    { type: "slider", key: "motorSpeed", label: "Speed", min: -5, max: 5, step: 1, default: 0, restart: false, group: "Motor Joint", visibleWhen: on("motor") },
    { type: "slider", key: "motorForce", label: "Max Force", min: 0, max: 1000000, step: 1000, default: 400000, restart: false, group: "Motor Joint", visibleWhen: on("motor") },
    { type: "slider", key: "motorTorque", label: "Max Torque", min: 0, max: 1000000, step: 1000, default: 500000, restart: false, group: "Motor Joint", visibleWhen: on("motor") },
    // Prismatic (C PrismaticJoint::DrawControls).
    { type: "checkbox", key: "priLimit", label: "Limit", default: false, restart: false, group: "Prismatic", visibleWhen: on("prismatic") },
    { type: "slider", key: "priLower", label: "Lower Translation", min: -10, max: 10, step: 0.1, default: -1, restart: false, group: "Prismatic", visibleWhen: [on("prismatic"), { key: "priLimit", equals: true }] },
    { type: "slider", key: "priUpper", label: "Upper Translation", min: -10, max: 10, step: 0.1, default: 1, restart: false, group: "Prismatic", visibleWhen: [on("prismatic"), { key: "priLimit", equals: true }] },
    { type: "checkbox", key: "priMotor", label: "Motor", default: false, restart: false, group: "Prismatic", visibleWhen: on("prismatic") },
    { type: "slider", key: "priMaxForce", label: "Max Force", min: 0, max: 100000, step: 100, default: 20, restart: false, group: "Prismatic", visibleWhen: [on("prismatic"), { key: "priMotor", equals: true }] },
    { type: "slider", key: "priSpeed", label: "Speed", min: -10, max: 10, step: 0.1, default: 0, restart: false, group: "Prismatic", visibleWhen: [on("prismatic"), { key: "priMotor", equals: true }] },
    { type: "checkbox", key: "priSpring", label: "Spring", default: true, restart: false, group: "Prismatic", visibleWhen: on("prismatic") },
    { type: "slider", key: "priHertz", label: "Hertz", min: 0, max: 10, step: 0.1, default: 2, restart: false, group: "Prismatic", visibleWhen: [on("prismatic"), { key: "priSpring", equals: true }] },
    { type: "slider", key: "priDamping", label: "Damping", min: 0, max: 2, step: 0.1, default: 0.7, restart: false, group: "Prismatic", visibleWhen: [on("prismatic"), { key: "priSpring", equals: true }] },
    { type: "slider", key: "priTarget", label: "Translation", min: -20, max: 20, step: 0.1, default: 0, restart: false, group: "Prismatic", visibleWhen: [on("prismatic"), { key: "priSpring", equals: true }] },
    // Spherical (C SphericalJoint::DrawControls).
    { type: "checkbox", key: "sphCone", label: "Cone Limit", default: false, restart: false, group: "Spherical", visibleWhen: on("spherical") },
    { type: "slider", key: "sphConeDeg", label: "Cone Angle", min: 0, max: 90, step: 1, default: 30, restart: false, group: "Spherical", visibleWhen: [on("spherical"), { key: "sphCone", equals: true }] },
    { type: "checkbox", key: "sphTwist", label: "Twist Limit", default: false, restart: false, group: "Spherical", visibleWhen: on("spherical") },
    { type: "slider", key: "sphLowerTwist", label: "Lower Twist", min: -180, max: 180, step: 1, default: -35, restart: false, group: "Spherical", visibleWhen: [on("spherical"), { key: "sphTwist", equals: true }] },
    { type: "slider", key: "sphUpperTwist", label: "Upper Twist", min: -180, max: 180, step: 1, default: 35, restart: false, group: "Spherical", visibleWhen: [on("spherical"), { key: "sphTwist", equals: true }] },
    { type: "checkbox", key: "sphMotor", label: "Motor", default: false, restart: false, group: "Spherical", visibleWhen: on("spherical") },
    { type: "slider", key: "sphMaxTorque", label: "Max Torque", min: 0, max: 10000, step: 10, default: 20, restart: false, group: "Spherical", visibleWhen: [on("spherical"), { key: "sphMotor", equals: true }] },
    { type: "slider", key: "sphVelX", label: "Velocity X", min: -10, max: 10, step: 1, default: 0, restart: false, group: "Spherical", visibleWhen: [on("spherical"), { key: "sphMotor", equals: true }] },
    { type: "slider", key: "sphVelY", label: "Velocity Y", min: -10, max: 10, step: 1, default: 0, restart: false, group: "Spherical", visibleWhen: [on("spherical"), { key: "sphMotor", equals: true }] },
    { type: "slider", key: "sphVelZ", label: "Velocity Z", min: -10, max: 10, step: 1, default: 0, restart: false, group: "Spherical", visibleWhen: [on("spherical"), { key: "sphMotor", equals: true }] },
    { type: "checkbox", key: "sphSpring", label: "Spring", default: true, restart: false, group: "Spherical", visibleWhen: on("spherical") },
    { type: "slider", key: "sphHertz", label: "Hertz", min: 0, max: 10, step: 0.1, default: 2, restart: false, group: "Spherical", visibleWhen: [on("spherical"), { key: "sphSpring", equals: true }] },
    { type: "slider", key: "sphDamping", label: "Damping", min: 0, max: 2, step: 0.1, default: 0.7, restart: false, group: "Spherical", visibleWhen: [on("spherical"), { key: "sphSpring", equals: true }] },
    { type: "slider", key: "sphRotX", label: "Rotation X", min: -180, max: 180, step: 1, default: 0, restart: false, group: "Spherical", visibleWhen: [on("spherical"), { key: "sphSpring", equals: true }] },
    { type: "slider", key: "sphRotY", label: "Rotation Y", min: -180, max: 180, step: 1, default: 0, restart: false, group: "Spherical", visibleWhen: [on("spherical"), { key: "sphSpring", equals: true }] },
    { type: "slider", key: "sphRotZ", label: "Rotation Z", min: -180, max: 180, step: 1, default: 0, restart: false, group: "Spherical", visibleWhen: [on("spherical"), { key: "sphSpring", equals: true }] },
    // Parallel Spring (C ParallelJoint::DrawControls). Note: C's ctor sets hertz=10
    // but its own slider range is 0-5; the joint is built at 10 (see Rust builder).
    { type: "slider", key: "parHertz", label: "Hertz", min: 0, max: 5, step: 0.1, default: 5, restart: false, group: "Parallel", visibleWhen: on("parallel") },
    { type: "slider", key: "parDamping", label: "Damping", min: 0, max: 2, step: 0.1, default: 0.7, restart: false, group: "Parallel", visibleWhen: on("parallel") },
    // Weld (C WeldJoint::DrawControls).
    { type: "slider", key: "weldLinHertz", label: "Linear Hertz", min: 0, max: 10, step: 0.1, default: 0, restart: false, group: "Weld", visibleWhen: on("weld") },
    { type: "slider", key: "weldLinDamp", label: "Linear Damping", min: 0, max: 2, step: 0.1, default: 0, restart: false, group: "Weld", visibleWhen: on("weld") },
    { type: "slider", key: "weldAngHertz", label: "Angular Hertz", min: 0, max: 10, step: 0.1, default: 2, restart: false, group: "Weld", visibleWhen: on("weld") },
    { type: "slider", key: "weldAngDamp", label: "Angular Damping", min: 0, max: 2, step: 0.1, default: 0.7, restart: false, group: "Weld", visibleWhen: on("weld") },
    // Wheel (C WheelJoint::DrawControls).
    { type: "checkbox", key: "whlSuspLimit", label: "Suspension Limit", default: false, restart: false, group: "Wheel", visibleWhen: on("wheel") },
    { type: "slider", key: "whlSuspMin", label: "Min", min: -10, max: 10, step: 0.1, default: -1, restart: false, group: "Wheel", visibleWhen: [on("wheel"), { key: "whlSuspLimit", equals: true }] },
    { type: "slider", key: "whlSuspMax", label: "Max", min: -10, max: 10, step: 0.1, default: 1, restart: false, group: "Wheel", visibleWhen: [on("wheel"), { key: "whlSuspLimit", equals: true }] },
    { type: "checkbox", key: "whlMotor", label: "Motor", default: false, restart: false, group: "Wheel", visibleWhen: on("wheel") },
    { type: "slider", key: "whlMaxTorque", label: "Max Torque", min: 0, max: 100, step: 1, default: 20, restart: false, group: "Wheel", visibleWhen: [on("wheel"), { key: "whlMotor", equals: true }] },
    { type: "slider", key: "whlSpinSpeed", label: "Speed", min: -10, max: 10, step: 1, default: 0, restart: false, group: "Wheel", visibleWhen: [on("wheel"), { key: "whlMotor", equals: true }] },
    { type: "checkbox", key: "whlSuspSpring", label: "Suspension Spring", default: false, restart: false, group: "Wheel", visibleWhen: on("wheel") },
    { type: "slider", key: "whlSuspHertz", label: "Susp Hertz", min: 0, max: 10, step: 0.1, default: 2, restart: false, group: "Wheel", visibleWhen: [on("wheel"), { key: "whlSuspSpring", equals: true }] },
    { type: "slider", key: "whlSuspDamp", label: "Susp Damping", min: 0, max: 2, step: 0.1, default: 0.7, restart: false, group: "Wheel", visibleWhen: [on("wheel"), { key: "whlSuspSpring", equals: true }] },
    { type: "checkbox", key: "whlSteering", label: "Steering", default: false, restart: false, group: "Wheel", visibleWhen: on("wheel") },
    { type: "slider", key: "whlSteerHertz", label: "Steer Hertz", min: 0, max: 10, step: 0.1, default: 1, restart: false, group: "Wheel", visibleWhen: [on("wheel"), { key: "whlSteering", equals: true }] },
    { type: "slider", key: "whlSteerDamp", label: "Steer Damping", min: 0, max: 2, step: 0.1, default: 0.7, restart: false, group: "Wheel", visibleWhen: [on("wheel"), { key: "whlSteering", equals: true }] },
    { type: "slider", key: "whlSteerDeg", label: "Steer Degrees", min: -90, max: 90, step: 1, default: 0, restart: false, group: "Wheel", visibleWhen: [on("wheel"), { key: "whlSteering", equals: true }] },
    { type: "checkbox", key: "whlSteerLimit", label: "Steering Limit", default: false, restart: false, group: "Wheel", visibleWhen: [on("wheel"), { key: "whlSteering", equals: true }] },
    { type: "slider", key: "whlSteerMinDeg", label: "Min Degrees", min: -90, max: 0, step: 1, default: -45, restart: false, group: "Wheel", visibleWhen: [on("wheel"), { key: "whlSteering", equals: true }, { key: "whlSteerLimit", equals: true }] },
    { type: "slider", key: "whlSteerMaxDeg", label: "Max Degrees", min: 0, max: 90, step: 1, default: 45, restart: false, group: "Wheel", visibleWhen: [on("wheel"), { key: "whlSteering", equals: true }, { key: "whlSteerLimit", equals: true }] },
    // Door (C Door::DrawControls).
    { type: "slider", key: "doorMagnitude", label: "Magnitude", min: 1000, max: 100000, step: 1000, default: 50000, restart: false, group: "Door", visibleWhen: on("door") },
    { type: "checkbox", key: "doorLimit", label: "Limit", default: false, restart: false, group: "Door", visibleWhen: on("door") },
    { type: "checkbox", key: "doorTwoJoints", label: "Two joints", default: true, restart: false, group: "Door", visibleWhen: on("door") },
    { type: "slider", key: "doorHertz", label: "Hertz", min: 15, max: 240, step: 1, default: 120, restart: false, group: "Door", visibleWhen: on("door") },
    { type: "slider", key: "doorDamping", label: "Damping", min: 0, max: 10, step: 0.1, default: 0, restart: false, group: "Door", visibleWhen: on("door") },
    // Bridge (C Bridge::DrawControls).
    { type: "slider", key: "bridgeGravity", label: "Gravity scale", min: -1, max: 1, step: 0.1, default: 1, restart: false, group: "Bridge", visibleWhen: on("bridge") },
    // Motion Locks (C MotionLocks::DrawControls).
    { type: "checkbox", key: "mlLinX", label: "Lock Linear X", default: false, restart: false, group: "Motion Locks", visibleWhen: on("motion-locks") },
    { type: "checkbox", key: "mlLinY", label: "Lock Linear Y", default: false, restart: false, group: "Motion Locks", visibleWhen: on("motion-locks") },
    { type: "checkbox", key: "mlLinZ", label: "Lock Linear Z", default: false, restart: false, group: "Motion Locks", visibleWhen: on("motion-locks") },
    { type: "checkbox", key: "mlAngX", label: "Lock Angular X", default: false, restart: false, group: "Motion Locks", visibleWhen: on("motion-locks") },
    { type: "checkbox", key: "mlAngY", label: "Lock Angular Y", default: false, restart: false, group: "Motion Locks", visibleWhen: on("motion-locks") },
    { type: "checkbox", key: "mlAngZ", label: "Lock Angular Z", default: false, restart: false, group: "Motion Locks", visibleWhen: on("motion-locks") },
  ];

  ctrl = attachInteraction({
    wasm: makeInteractAdapter(wasm, "joint"),
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
        const hudScenes: Scene[] = ["revolute", "driving", "motor", "wheel", "door"];
        if (!hudScenes.includes(scene)) readout.innerHTML = "";
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
      } else if (key === "distCount") {
        // C Count slider rebuilds the whole chain from the current members.
        resetDistance(values);
      } else if (key.startsWith("dist")) {
        applyDistance(values);
      } else if (key.startsWith("motor")) {
        applyMotor(values);
      } else if (key.startsWith("pri")) {
        if (key === "priLower") values.priLower = Math.min(num(values, "priLower"), num(values, "priUpper"));
        if (key === "priUpper") values.priUpper = Math.max(num(values, "priUpper"), num(values, "priLower"));
        applyPrismatic(values);
      } else if (key.startsWith("sph")) {
        if (key === "sphLowerTwist") values.sphLowerTwist = Math.min(num(values, "sphLowerTwist"), num(values, "sphUpperTwist"));
        if (key === "sphUpperTwist") values.sphUpperTwist = Math.max(num(values, "sphUpperTwist"), num(values, "sphLowerTwist"));
        applySpherical(values);
      } else if (key.startsWith("par")) {
        applyParallel(values);
      } else if (key.startsWith("weld")) {
        applyWeld(values);
      } else if (key.startsWith("whl")) {
        if (key === "whlSuspMin") values.whlSuspMin = Math.min(num(values, "whlSuspMin"), num(values, "whlSuspMax"));
        if (key === "whlSuspMax") values.whlSuspMax = Math.max(num(values, "whlSuspMax"), num(values, "whlSuspMin"));
        applyWheel(values);
      } else if (key === "doorMagnitude") {
        wasm.joint_door_set_magnitude(num(values, "doorMagnitude"));
      } else if (key === "doorLimit") {
        wasm.joint_door_set_limit(bool(values, "doorLimit"));
      } else if (key === "doorTwoJoints") {
        wasm.joint_door_set_two_joints(bool(values, "doorTwoJoints"));
      } else if (key === "doorHertz" || key === "doorDamping") {
        wasm.joint_door_set_tuning(num(values, "doorHertz"), num(values, "doorDamping"));
      } else if (key === "bridgeGravity") {
        wasm.joint_set_bridge_gravity(num(values, "bridgeGravity"));
      } else if (key.startsWith("ml")) {
        applyMotionLocks(values);
      }
    },
  }) as SimControllerWithTick;

  driveNote.style.display = scene === "driving" ? "" : "none";
  controls.appendChild(buttonRow);
  controls.appendChild(readout);
  controls.appendChild(driveNote);
  controls.appendChild(doorNote);
  updateSceneButtons();

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
    // C MotionLocks: hold L to punch the first body (IsKeyDown( KEY_L )).
    if (scene === "motion-locks" && keys.has("KeyL")) {
      wasm.joint_motion_lock_impulse();
    }

    ctrl.tickFrame();
    const poses = wasm.joint_poses();
    const awake = wasm.joint_counters()[5] ?? 0;
    syncMeshesFromPoses(demo.content, pool, poses, {
      groundIndex: scene === "driving" || scene === "gear" ? null : 0,
      styles: styleGate(awake, () => wasm.joint_styles()),
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
    } else if (scene === "motor") {
      if (frame % 10 === 0) {
        const r = wasm.joint_motor_readout();
        updateReadout(readout, [
          { label: "force", value: (r[MOTOR_READOUT.force] ?? 0).toFixed(0) },
          { label: "torque", value: (r[MOTOR_READOUT.torque] ?? 0).toFixed(0) },
        ]);
      }
    } else if (scene === "wheel") {
      if (frame % 10 === 0) {
        const deg = wasm.joint_wheel_steering_angle();
        updateReadout(readout, [{ label: "steering degrees", value: deg.toFixed(1) }]);
      }
    } else if (scene === "door") {
      if (frame % 10 === 0) {
        const d = wasm.joint_door_readout();
        const rows = [
          { label: "translation error 1", value: (d[DOOR_READOUT.error1] ?? 0).toPrecision(4) },
        ];
        if ((d[DOOR_READOUT.hasTwo] ?? 0) > 0.5) {
          rows.push({
            label: "translation error 2",
            value: (d[DOOR_READOUT.error2] ?? 0).toPrecision(4),
          });
        }
        updateReadout(readout, rows);
      }
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
