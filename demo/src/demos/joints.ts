// Joints — Ball and Chain, Revolute, Gear Lift, Driving (sample_joint.cpp).

import type * as THREE from "three";
import { createCheckbox, createInfoBox, createReadout, createSlider, updateReadout } from "../controls.ts";
import {
  attachInteraction,
  type InteractWasm,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm, type Box3dWasm } from "../wasm.ts";
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

function makeSection(title: string): HTMLDivElement {
  const div = document.createElement("div");
  div.className = "param-panel";
  const t = document.createElement("div");
  t.className = "control-section-title";
  t.textContent = title;
  div.appendChild(t);
  return div;
}

function subLabel(text: string): HTMLElement {
  const el = document.createElement("div");
  el.className = "control-note";
  el.style.fontWeight = "600";
  el.textContent = text;
  return el;
}

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Joints",
    "Official Joints samples: Ball and Chain, Revolute, Gear Lift, and Driving — " +
      "ported from <code>sample_joint.cpp</code>.",
    "Pick a sample · drag bodies · P pause / O step / R restart · WASD drives",
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

  let scene: Scene = "revolute";
  let ctrl!: SimControllerWithTick;

  // --- Per-sample control state (mirrors C member variables / DrawControls) ---
  const rev = {
    limit: false,
    lower: -35,
    upper: 35,
    motor: false,
    torque: 5000,
    speed: 0,
    spring: false,
    hertz: 2,
    damping: 0.7,
    target: 0,
  };
  const gear = { motor: true, torque: 30000, speed: -0.3 };
  const drive = {
    suspMin: -0.2,
    suspMax: 0.2,
    suspHertz: 4,
    suspDamp: 0.7,
    motorTorque: 5,
    spinSpeed: 30,
    steerHertz: 10,
    steerDamp: 0.7,
    steerTorque: 5,
    steerMinDeg: -45,
    steerMaxDeg: 45,
  };

  function applyRevolute() {
    let flags = 0;
    if (rev.limit) flags |= 1;
    if (rev.motor) flags |= 2;
    if (rev.spring) flags |= 4;
    wasm.joint_set_revolute_params(
      flags,
      rev.lower,
      rev.upper,
      rev.speed,
      rev.torque,
      rev.hertz,
      rev.damping,
      rev.target,
    );
  }
  function applyGear() {
    // Gear driver: motor only (no limit/spring). C GearLift::DrawControls.
    wasm.joint_set_revolute_params(
      gear.motor ? 2 : 0,
      -35,
      35,
      gear.speed,
      gear.torque,
      2,
      0.7,
      0,
    );
  }
  function applyDrivingSuspension() {
    wasm.joint_set_driving_suspension(drive.suspMin, drive.suspMax, drive.suspHertz, drive.suspDamp);
  }
  function applyDrivingMotor() {
    wasm.joint_set_drive_params(drive.spinSpeed, drive.motorTorque);
  }
  function applyDrivingSteering() {
    wasm.joint_set_driving_steering(
      drive.steerHertz,
      drive.steerDamp,
      drive.steerTorque,
      drive.steerMinDeg,
      drive.steerMaxDeg,
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
    if (scene === "chain") {
      wasm.joint_reset_chain();
    } else if (scene === "revolute") {
      wasm.joint_reset_hinge();
      applyRevolute();
    } else if (scene === "gear") {
      wasm.joint_reset_gear_lift();
      applyGear();
      rebuildTerrain();
    } else {
      wasm.joint_reset_driving();
      applyDrivingSuspension();
      applyDrivingMotor();
      applyDrivingSteering();
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
    sampleName: "Joints",
    sampleCategory: "Joints",
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
    ],
    onParamsChange: (values, key) => {
      if (key === "sample") {
        scene = values.sample as Scene;
        updateSceneControls();
      }
    },
  }) as SimControllerWithTick;

  // --- Revolute controls (C RevoluteJoint::DrawControls, gated by checkbox) ---
  const revSection = makeSection("Revolute");
  revSection.appendChild(
    createCheckbox("Limit", rev.limit, (v) => {
      rev.limit = v;
      updateRevVis();
      applyRevolute();
    }),
  );
  const revLower = createSlider("Lower Angle °", -180, 180, rev.lower, 1, (v) => {
    rev.lower = Math.min(v, rev.upper);
    applyRevolute();
  });
  const revUpper = createSlider("Upper Angle °", -180, 180, rev.upper, 1, (v) => {
    rev.upper = Math.max(v, rev.lower);
    applyRevolute();
  });
  revSection.append(revLower, revUpper);
  revSection.appendChild(
    createCheckbox("Motor", rev.motor, (v) => {
      rev.motor = v;
      updateRevVis();
      applyRevolute();
    }),
  );
  const revTorque = createSlider("Max Torque", 0, 50000, rev.torque, 100, (v) => {
    rev.torque = v;
    applyRevolute();
  });
  const revSpeed = createSlider("Speed", -10, 10, rev.speed, 1, (v) => {
    rev.speed = v;
    applyRevolute();
  });
  revSection.append(revTorque, revSpeed);
  revSection.appendChild(
    createCheckbox("Spring", rev.spring, (v) => {
      rev.spring = v;
      updateRevVis();
      applyRevolute();
    }),
  );
  const revHertz = createSlider("Hertz", 0, 10, rev.hertz, 0.1, (v) => {
    rev.hertz = v;
    applyRevolute();
  });
  const revDamping = createSlider("Damping", 0, 2, rev.damping, 0.1, (v) => {
    rev.damping = v;
    applyRevolute();
  });
  const revRotation = createSlider("Rotation °", -180, 180, rev.target, 1, (v) => {
    rev.target = v;
    applyRevolute();
  });
  revSection.append(revHertz, revDamping, revRotation);
  controls.appendChild(revSection);

  function updateRevVis() {
    revLower.style.display = rev.limit ? "" : "none";
    revUpper.style.display = rev.limit ? "" : "none";
    revTorque.style.display = rev.motor ? "" : "none";
    revSpeed.style.display = rev.motor ? "" : "none";
    revHertz.style.display = rev.spring ? "" : "none";
    revDamping.style.display = rev.spring ? "" : "none";
    revRotation.style.display = rev.spring ? "" : "none";
  }

  // --- Gear Lift controls (C GearLift::DrawControls) ---
  const gearSection = makeSection("Gear Lift");
  gearSection.appendChild(
    createCheckbox("Motor", gear.motor, (v) => {
      gear.motor = v;
      applyGear();
    }),
  );
  gearSection.append(
    createSlider("Max Torque", 0, 100000, gear.torque, 100, (v) => {
      gear.torque = v;
      applyGear();
    }),
    createSlider("Speed", -0.3, 0.3, gear.speed, 0.01, (v) => {
      gear.speed = v;
      applyGear();
    }),
  );
  controls.appendChild(gearSection);

  // --- Driving controls (C Driving::DrawControls) ---
  const driveSection = makeSection("Driving");
  driveSection.appendChild(subLabel("Suspension"));
  driveSection.append(
    createSlider("Min", -10, 10, drive.suspMin, 0.1, (v) => {
      drive.suspMin = Math.min(v, drive.suspMax);
      applyDrivingSuspension();
    }),
    createSlider("Max", -10, 10, drive.suspMax, 0.1, (v) => {
      drive.suspMax = Math.max(v, drive.suspMin);
      applyDrivingSuspension();
    }),
    createSlider("Hertz", 0, 10, drive.suspHertz, 0.1, (v) => {
      drive.suspHertz = v;
      applyDrivingSuspension();
    }),
    createSlider("Damping", 0, 2, drive.suspDamp, 0.1, (v) => {
      drive.suspDamp = v;
      applyDrivingSuspension();
    }),
  );
  driveSection.appendChild(subLabel("Motor"));
  driveSection.append(
    createSlider("Max Torque", 0, 100, drive.motorTorque, 1, (v) => {
      drive.motorTorque = v;
      applyDrivingMotor();
    }),
    createSlider("Speed", 0, 100, drive.spinSpeed, 1, (v) => {
      drive.spinSpeed = v;
      applyDrivingMotor();
    }),
  );
  driveSection.appendChild(subLabel("Steering"));
  driveSection.append(
    createSlider("Hertz", 0, 10, drive.steerHertz, 0.1, (v) => {
      drive.steerHertz = v;
      applyDrivingSteering();
    }),
    createSlider("Damping", 0, 2, drive.steerDamp, 0.1, (v) => {
      drive.steerDamp = v;
      applyDrivingSteering();
    }),
    createSlider("Torque", 0, 20, drive.steerTorque, 0.1, (v) => {
      drive.steerTorque = v;
      applyDrivingSteering();
    }),
    createSlider("Min Deg", -90, 0, drive.steerMinDeg, 1, (v) => {
      drive.steerMinDeg = v;
      applyDrivingSteering();
    }),
    createSlider("Max Deg", 0, 90, drive.steerMaxDeg, 1, (v) => {
      drive.steerMaxDeg = v;
      applyDrivingSteering();
    }),
  );
  const driveNote = document.createElement("div");
  driveNote.className = "control-note";
  driveNote.textContent =
    "Drive with WASD (or arrow keys). Third-person (T) camera-follow lands in a later batch.";
  driveSection.appendChild(driveNote);
  controls.appendChild(driveSection);

  // --- Telemetry / energy HUD (C Render() DrawTextLine) ---
  const readout = createReadout();
  controls.appendChild(readout);

  function updateSceneControls() {
    revSection.style.display = scene === "revolute" ? "" : "none";
    gearSection.style.display = scene === "gear" ? "" : "none";
    driveSection.style.display = scene === "driving" ? "" : "none";
    if (scene === "revolute") updateRevVis();
    if (scene !== "revolute" && scene !== "driving") readout.innerHTML = "";
  }

  updateSceneControls();
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
    });

    frame += 1;
    // Throttle the telemetry HUD to every 10 frames (matches sensors.ts); camera
    // tracking below still runs every frame so the driving chase-cam stays smooth.
    if (scene === "revolute") {
      if (frame % 10 === 0) {
        const e = wasm.joint_revolute_energy();
        updateReadout(readout, [
          { label: "kinetic energy", value: (e[0] ?? 0).toPrecision(4) },
          { label: "potential energy", value: (e[1] ?? 0).toPrecision(4) },
          { label: "total energy", value: (e[2] ?? 0).toPrecision(4) },
        ]);
      }
    } else if (scene === "driving") {
      if (frame % 10 === 0) {
        const t = wasm.joint_drive_telemetry();
        updateReadout(readout, [
          { label: "speed", value: (t[0] ?? 0).toFixed(1) },
          { label: "spin speed", value: `${(t[1] ?? 0).toFixed(1)}/${(t[2] ?? 0).toFixed(1)}` },
          { label: "spin torque", value: `${(t[3] ?? 0).toFixed(1)}/${(t[4] ?? 0).toFixed(1)}` },
          { label: "steering °", value: `${(t[5] ?? 0).toFixed(1)}/${(t[6] ?? 0).toFixed(1)}` },
          { label: "steering torque", value: `${(t[7] ?? 0).toFixed(1)}/${(t[8] ?? 0).toFixed(1)}` },
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
