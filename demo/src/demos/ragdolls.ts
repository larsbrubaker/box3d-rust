// Ragdolls — CreateHuman on a ground box (upstream Ragdoll / Box sample).

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
    "Ragdolls",
    "Capsule-bone humans from the ported <code>create_human</code> API — spherical and " +
      "revolute joints with friction motors. Same builder used by the falling-ragdoll " +
      "determinism soak.",
    "Drag to orbit · tweak joint params · respawn",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "Joint friction / hertz / damping map to <code>Human_SetJoint*</code>. " +
        "Multiple ragdolls stress the solver (browser soak).",
    ),
  );

  let count = 2;
  let friction = 5;
  let hertz = 1;
  let damping = 0.7;

  controls.appendChild(
    createSlider("Humans", 1, 6, count, 1, (v) => {
      count = Math.round(v);
      reset();
    }),
  );
  controls.appendChild(
    createSlider("Friction", 0, 20, friction, 0.5, (v) => {
      friction = v;
      wasm.ragdoll_set_joint_params(friction, hertz, damping);
    }),
  );
  controls.appendChild(
    createSlider("Hertz", 0, 20, hertz, 0.5, (v) => {
      hertz = v;
      wasm.ragdoll_set_joint_params(friction, hertz, damping);
    }),
  );
  controls.appendChild(
    createSlider("Damping", 0, 4, damping, 0.1, (v) => {
      damping = v;
      wasm.ragdoll_set_joint_params(friction, hertz, damping);
    }),
  );
  controls.appendChild(
    createButtonGroup([{ label: "Respawn", value: "restart" }], "restart", () => reset()),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { target: [0, 1.2, 0], distance: 8 });
  const pool = createMeshPool();

  function reset() {
    wasm.ragdoll_reset(count);
    wasm.ragdoll_set_joint_params(friction, hertz, damping);
  }

  reset();

  let frame = 0;
  const stop = runLoop(() => {
    wasm.ragdoll_step(1 / 60, 4);
    const poses = wasm.ragdoll_poses();
    syncMeshesFromPoses(demo.content, pool, poses);
    frame += 1;
    if (frame % 20 === 0) {
      updateReadout(readout, [
        { label: "humans", value: String(count) },
        { label: "bodies", value: String(Math.floor(poses.length / 16)) },
        { label: "frame", value: String(frame) },
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
