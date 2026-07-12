// Sensors — Sensor Visit, Sensor Hits, Benchmark Sensor (sample_events / sample_benchmark).

import {
  createButton,
  createButtonGroup,
  createCheckbox,
  createInfoBox,
  createReadout,
  updateReadout,
} from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import { DemoScene } from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

type Scene = "visit" | "hits" | "benchmark";

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
    "Sensors",
    "Official Events / Benchmark sensor samples from <code>sample_events.cpp</code> and " +
      "<code>sample_benchmark.cpp</code>: Sensor Visit, Sensor Hits, and Benchmark Sensor.",
    "Pick a sample · Launch (B) on Hits · Restart",
    wasm.version(),
    { category: "Events", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "<strong>Sensor Visit</strong> — kinematic sensor destroys the visitor on begin-touch.<br>" +
        "<strong>Sensor Hits</strong> — static/kinematic mesh sensors + prismatic capsule; " +
        "launch a bullet sphere (checkbox + <kbd>B</kbd>).<br>" +
        "<strong>Benchmark Sensor</strong> — 12×12 grid (C is 40×40); lime tint on overlap; " +
        "bottom active sensors destroy visitors; mid row fuchsia (custom filter skipped).",
    ),
  );

  let scene: Scene = "visit";
  let hitsRow: HTMLElement | null = null;

  controls.appendChild(
    createButtonGroup(
      [
        { label: "Sensor Visit", value: "visit" },
        { label: "Sensor Hits", value: "hits" },
        { label: "Benchmark Sensor", value: "benchmark" },
      ],
      "visit",
      (v) => {
        scene = v as Scene;
        reset();
      },
    ),
  );

  const actionRow = document.createElement("div");
  actionRow.className = "control-row";
  actionRow.appendChild(createButton("Restart", () => reset(), false));
  controls.appendChild(actionRow);

  hitsRow = document.createElement("div");
  hitsRow.className = "control-row";
  hitsRow.style.display = "none";
  hitsRow.appendChild(createCheckbox("Bullet", true, (v) => { wasm.sensor_set_bullet(v); }));
  hitsRow.appendChild(createButton("Launch", () => { wasm.sensor_launch(); }, false));
  controls.appendChild(hitsRow);

  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { target: [0, 5, 0], distance: 20, shadowExtent: 48 });
  demo.camera.far = 500;
  demo.camera.updateProjectionMatrix();
  const pool = createMeshPool();

  function sceneId(): number {
    if (scene === "hits") return 1;
    if (scene === "benchmark") return 2;
    return 0;
  }

  function setCamera() {
    if (scene === "visit") cameraFromView(demo, 0, 30, 20, [0, 5, 0]);
    else if (scene === "hits") cameraFromView(demo, 0, 30, 40, [0, 5, 0]);
    else cameraFromView(demo, 0, 0, 100, [0, 40, 0]);
  }

  function reset() {
    wasm.sensor_reset(sceneId());
    if (hitsRow) hitsRow.style.display = scene === "hits" ? "" : "none";
    setCamera();
  }

  reset();

  const onKey = (e: KeyboardEvent) => {
    if (e.code === "KeyB" && scene === "hits") {
      e.preventDefault();
      wasm.sensor_launch();
    }
  };
  window.addEventListener("keydown", onKey);

  let frame = 0;
  const stop = runLoop(() => {
    wasm.sensor_step(1 / 60, 4);
    syncMeshesFromPoses(demo.content, pool, wasm.sensor_poses(), {
      sensorIndices: wasm.sensor_sensor_indices(),
      colors: wasm.sensor_colors(),
      groundIndex: scene === "hits" ? 0 : null,
    });
    frame += 1;
    if (frame % 10 === 0) {
      const st = wasm.sensor_event_stats();
      if (scene === "benchmark") {
        updateReadout(readout, [
          { label: "max begin touch", value: String(st[0]) },
          { label: "max end touch", value: String(st[1]) },
          { label: "begin this step", value: String(st[2]) },
          { label: "end this step", value: String(st[3]) },
        ]);
      } else {
        updateReadout(readout, [
          { label: "begin touch count", value: String(st[0]) },
          { label: "end touch count", value: String(st[1]) },
          { label: "begin this step", value: String(st[2]) },
          { label: "end this step", value: String(st[3]) },
        ]);
      }
    }
    demo.render();
  }, readout);

  return () => {
    stop();
    window.removeEventListener("keydown", onKey);
    disposeMeshPool(pool);
    demo.dispose();
  };
}
