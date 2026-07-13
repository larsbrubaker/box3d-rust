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
import { DemoScene, setView } from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Ragdolls",
    "Capsule-bone human from the ported <code>create_human</code> API — spherical and " +
      "revolute joints with friction motors. Same builder used by the falling-ragdoll " +
      "determinism soak.",
    "Drag to orbit · tweak joint params · respawn",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "Joint friction / hertz / damping map to <code>Human_SetJoint*</code> " +
        "(C RagdollOnBox::DrawControls).",
    ),
  );

  // C RagdollOnBox defaults: jointFrictionTorque 5, jointHertz 1, dampingRatio 0.7.
  let friction = 5;
  let hertz = 1;
  let damping = 0.7;

  // C DrawControls: "Joint Friction" 0–20 (%3.0f), "Hertz" 0–20 (%3.1f),
  // "Damping" 0–4 (%3.1f).
  controls.appendChild(
    createSlider("Joint Friction", 0, 20, friction, 1, (v) => {
      friction = v;
      wasm.ragdoll_set_joint_params(friction, hertz, damping);
    }),
  );
  controls.appendChild(
    createSlider("Hertz", 0, 20, hertz, 0.1, (v) => {
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

  // C RagdollOnBox: SetView( 45, 30, 6, b3Pos_zero ).
  const demo = new DemoScene(canvas, { target: [0, 0, 0], distance: 6 });
  const pool = createMeshPool();

  function reset() {
    wasm.ragdoll_reset();
    wasm.ragdoll_set_joint_params(friction, hertz, damping);
    setView(demo, 45, 30, 6, [0, 0, 0]);
  }

  reset();

  let frame = 0;
  const stop = runLoop(() => {
    wasm.ragdoll_step(1 / 60, 4);
    const poses = wasm.ragdoll_poses();
    syncMeshesFromPoses(demo.content, pool, poses, { styles: wasm.ragdoll_styles() });
    frame += 1;
    if (frame % 20 === 0) {
      updateReadout(readout, [
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
