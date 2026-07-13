// Sensors — Sensor Visit, Sensor Hits, Benchmark Sensor (sample_events / sample_benchmark).

import {
  createButton,
  createButtonGroup,
  createCheckbox,
  createInfoBox,
  createReadout,
  updateReadout,
} from "../controls.ts";
import { getWasm, SENSOR_EVENT_STATS } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { demoPage, runLoop } from "./common.ts";
import { DemoScene, setView } from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

type Scene = "visit" | "hits" | "benchmark";

export const SCENES: Scene[] = ["visit", "hits", "benchmark"];

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  // Self-check the scene table against the registry (see registry.ts pattern).
  // Benchmark Sensor is a Benchmark-category sample hosted on this route.
  assertRouteScenes("sensors", SCENES);
  const { canvas, controls } = demoPage(
    container,
    "Sensors",
    "Events sensor samples from <code>sample_events.cpp</code> (Sensor Visit, Sensor Hits — both exact) " +
      "plus <strong>Benchmark Sensor</strong>, which is a <em>Benchmark-category</em> sample from " +
      "<code>sample_benchmark.cpp</code> hosted here for convenience.",
    "Pick a sample · Launch (B) on Hits · Restart",
    wasm.version(),
    { category: "Events", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "<strong>Sensor Visit</strong> — kinematic sensor destroys the visitor on begin-touch.<br>" +
        "<strong>Sensor Hits</strong> — static/kinematic mesh sensors + prismatic capsule; " +
        "launch a bullet sphere (checkbox + <kbd>B</kbd>).<br>" +
        "<strong>Benchmark Sensor</strong> (Benchmark category, not Events) — full C-scale 40×40 grid; lime tint on overlap; " +
        "bottom active sensors destroy visitors; mid row fuchsia (custom filter active).",
    ),
  );

  let scene: Scene =
    initialScene && SCENES.includes(initialScene as Scene) ? (initialScene as Scene) : "visit";
  let hitsRow: HTMLElement | null = null;

  controls.appendChild(
    createButtonGroup(
      [
        { label: "Sensor Visit", value: "visit" },
        { label: "Sensor Hits", value: "hits" },
        { label: "Benchmark Sensor", value: "benchmark" },
      ],
      scene,
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
    if (scene === "visit") setView(demo, 0, 30, 20, [0, 5, 0]);
    else if (scene === "hits") setView(demo, 0, 30, 40, [0, 5, 0]);
    else setView(demo, 0, 0, 100, [0, 40, 0]);
  }

  // Cache the sensor-index set (up to ~1600 u32 in the benchmark) and refetch it only when
  // Rust reports the sensor topology changed, instead of marshaling it every frame.
  let sensorIndicesCache = new Uint32Array(0);
  let sensorTopoVersion = -1;

  function reset() {
    wasm.sensor_reset(sceneId());
    sensorTopoVersion = -1; // force a refetch of the sensor-index set after reset
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
    const topo = wasm.sensor_topology_version();
    if (topo !== sensorTopoVersion) {
      sensorIndicesCache = wasm.sensor_sensor_indices();
      sensorTopoVersion = topo;
    }
    syncMeshesFromPoses(demo.content, pool, wasm.sensor_poses(), {
      sensorIndices: sensorIndicesCache,
      colors: wasm.sensor_colors(),
      groundIndex: scene === "hits" ? 0 : null,
    });
    frame += 1;
    if (frame % 10 === 0) {
      const st = wasm.sensor_event_stats();
      if (scene === "benchmark") {
        updateReadout(readout, [
          { label: "max begin touch", value: String(st[SENSOR_EVENT_STATS.begin]) },
          { label: "max end touch", value: String(st[SENSOR_EVENT_STATS.end]) },
          { label: "begin this step", value: String(st[SENSOR_EVENT_STATS.beginThisStep]) },
          { label: "end this step", value: String(st[SENSOR_EVENT_STATS.endThisStep]) },
        ]);
      } else {
        updateReadout(readout, [
          { label: "begin touch count", value: String(st[SENSOR_EVENT_STATS.begin]) },
          { label: "end touch count", value: String(st[SENSOR_EVENT_STATS.end]) },
          { label: "begin this step", value: String(st[SENSOR_EVENT_STATS.beginThisStep]) },
          { label: "end this step", value: String(st[SENSOR_EVENT_STATS.endThisStep]) },
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
