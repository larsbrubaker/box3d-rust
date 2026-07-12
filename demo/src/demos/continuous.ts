// Continuous — bullet vs thin wall with CCD on/off toggle.

import {
  createButtonGroup,
  createInfoBox,
  createReadout,
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
    "Continuous",
    "Fast bullet sphere vs a thin static wall. Toggle continuous collision to see " +
      "tunneling (CCD off) vs a stop near the wall (CCD on).",
    "Drag to orbit · toggle CCD · fire again",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "Matches the unit-test scene: <code>is_bullet</code> body + " +
        "<code>world.enable_continuous</code>. With CCD off the ball tunnels past x≈0.",
    ),
  );

  let continuous = true;

  controls.appendChild(
    createButtonGroup(
      [
        { label: "CCD On", value: "on" },
        { label: "CCD Off", value: "off" },
      ],
      "on",
      (v) => {
        continuous = v === "on";
        reset();
      },
    ),
  );
  controls.appendChild(
    createButtonGroup([{ label: "Fire again", value: "fire" }], "fire", () => reset()),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { target: [0, 0, 0], distance: 14 });
  const pool = createMeshPool();

  function reset() {
    wasm.continuous_reset(continuous);
  }

  reset();

  let frame = 0;
  const stop = runLoop(() => {
    wasm.continuous_step(1 / 60, 4);
    const poses = wasm.continuous_poses();
    syncMeshesFromPoses(demo.content, pool, poses);
    frame += 1;
    if (frame % 10 === 0) {
      const st = wasm.continuous_status();
      const x = st[0]!;
      const verdict =
        continuous && x > -0.5 && x < 1.5
          ? "stopped"
          : !continuous && x < -1
            ? "tunneled"
            : "in flight";
      updateReadout(readout, [
        { label: "CCD", value: continuous ? "on" : "off" },
        { label: "bullet x", value: x.toFixed(2) },
        { label: "result", value: verdict },
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
