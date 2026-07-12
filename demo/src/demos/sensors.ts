// Sensors — begin/end touch event visualization.

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
    "Sensors",
    "A static sensor volume reports begin/end touch as spheres fall through. " +
      "Events come from <code>world.get_sensor_events</code> after each step.",
    "Drag to orbit · watch begin/end counters",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "Green translucent box is <code>is_sensor</code> with sensor events enabled. " +
        "Visitors also enable sensor events so overlaps are reported.",
    ),
  );

  controls.appendChild(
    createButtonGroup([{ label: "Restart", value: "restart" }], "restart", () => reset()),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { target: [0, 1.5, 0], distance: 14 });
  const pool = createMeshPool();

  function reset() {
    wasm.sensor_reset();
  }

  reset();

  let frame = 0;
  const stop = runLoop(() => {
    wasm.sensor_step(1 / 60, 4);
    const poses = wasm.sensor_poses();
    // Index 1 is the sensor volume (after ground).
    syncMeshesFromPoses(demo.content, pool, poses, { sensorIndex: 1 });
    frame += 1;
    if (frame % 10 === 0) {
      const st = wasm.sensor_event_stats();
      updateReadout(readout, [
        { label: "begin total", value: String(st[0]) },
        { label: "end total", value: String(st[1]) },
        { label: "begin this step", value: String(st[2]) },
        { label: "end this step", value: String(st[3]) },
        { label: "occupied", value: st[4]! > 0 ? "yes" : "no" },
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
