// Joints — Ball and Chain + Revolute motor hinge.

import {
  createButtonGroup,
  createInfoBox,
  createReadout,
  createSlider,
  updateReadout,
} from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import { DemoScene } from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Joints",
    "Spherical ball-and-chain and a revolute hinge with a motor toggle — " +
      "mirroring the upstream Joints samples.",
    "Drag to orbit · switch scene · toggle motor",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "Ball &amp; Chain uses <code>create_spherical_joint</code>. " +
        "Hinge uses <code>create_revolute_joint</code> with motor enable/speed/torque.",
    ),
  );

  let mode: "chain" | "hinge" = "chain";
  let links = 12;
  let motorOn = false;
  let motorSpeed = 2;
  let motorTorque = 5000;

  controls.appendChild(
    createButtonGroup(
      [
        { label: "Ball & Chain", value: "chain" },
        { label: "Motor Hinge", value: "hinge" },
      ],
      "chain",
      (v) => {
        mode = v as "chain" | "hinge";
        reset();
      },
    ),
  );

  const linkSlider = createSlider("Links", 4, 20, links, 1, (v) => {
    links = Math.round(v);
    if (mode === "chain") reset();
  });
  controls.appendChild(linkSlider);

  controls.appendChild(
    createButtonGroup(
      [
        { label: "Motor Off", value: "off" },
        { label: "Motor On", value: "on" },
      ],
      "off",
      (v) => {
        motorOn = v === "on";
        if (mode === "hinge") {
          wasm.joint_set_motor(motorOn, motorSpeed, motorTorque);
        }
      },
    ),
  );

  controls.appendChild(
    createSlider("Motor speed", -8, 8, motorSpeed, 0.5, (v) => {
      motorSpeed = v;
      if (mode === "hinge") wasm.joint_set_motor(motorOn, motorSpeed, motorTorque);
    }),
  );

  controls.appendChild(
    createButtonGroup([{ label: "Restart", value: "restart" }], "restart", () => reset()),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { target: [6, -4, 0], distance: 28 });
  const pool = createMeshPool();

  function reset() {
    if (mode === "chain") {
      wasm.joint_reset_chain(links);
      demo.controls.target.set(6, -4, 0);
      demo.camera.position.set(18, 6, 22);
    } else {
      wasm.joint_reset_hinge();
      wasm.joint_set_motor(motorOn, motorSpeed, motorTorque);
      demo.controls.target.set(0, 3, 0);
      demo.camera.position.set(8, 6, 12);
    }
    demo.controls.update();
  }

  reset();

  let frame = 0;
  const stop = runLoop(() => {
    wasm.joint_step(1 / 60, 4);
    const poses = wasm.joint_poses();
    syncMeshesFromPoses(demo.content, pool, poses);
    frame += 1;
    if (frame % 20 === 0) {
      updateReadout(readout, [
        { label: "scene", value: mode },
        { label: "bodies", value: String(Math.floor(poses.length / 15)) },
        {
          label: "motor",
          value: mode === "hinge" ? (motorOn ? "on" : "off") : "n/a",
        },
      ]);
    }
    demo.render();
  }, readout);

  return () => {
    stop();
    disposeMeshPool(pool);
    demo.dispose();
  };
}
