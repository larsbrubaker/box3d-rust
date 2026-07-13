// Sensors route — hosts every sample_events.cpp scene plus Benchmark Sensor.
//
// Sensor scenes (verified exact): Sensor Visit, Sensor Hits, Benchmark Sensor.
// Events scenes (sample_events.cpp): Hit, Move, Joint, Persistent Contact — these
// add the debug-draw overlay (hit points / manifold impulses), 3D text labels,
// ground-mesh wireframes, and per-scene HUD readouts.

import * as THREE from "three";
import {
  createButton,
  createButtonGroup,
  createCheckbox,
  createInfoBox,
  createReadout,
  updateReadout,
} from "../controls.ts";
import {
  DebugDrawOverlay,
  makeInteractAdapter,
  parseDebugText,
  pickRay,
  TextLabelOverlay,
} from "../interaction.ts";
import { getWasm, SENSOR_EVENT_STATS } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { demoPage, runLoop } from "./common.ts";
import { DemoScene, makeWireEdges, setView } from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

type Scene = "visit" | "hits" | "benchmark" | "hit" | "move" | "joint" | "persistent";

export const SCENES: Scene[] = ["visit", "hits", "benchmark", "hit", "move", "joint", "persistent"];

// Scenes from the Events samples that use the overlay / text / HUD channels.
const EVENTS_SCENES = new Set<Scene>(["hit", "move", "joint", "persistent"]);
// Scenes whose ground is a triangle mesh drawn as a wireframe (not a pose box).
const MESH_GROUND_SCENES = new Set<Scene>(["hit", "persistent"]);

const SCENE_ID: Record<Scene, number> = {
  visit: 0,
  hits: 1,
  benchmark: 2,
  hit: 3,
  move: 4,
  joint: 5,
  persistent: 6,
};

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  // Self-check the scene table against the registry (see registry.ts pattern).
  // Benchmark Sensor is a Benchmark-category sample hosted on this route.
  assertRouteScenes("sensors", SCENES);
  const { canvas, controls } = demoPage(
    container,
    "Sensors & Events",
    "Every <code>sample_events.cpp</code> scene — Sensor Visit, Sensor Hits, Hit, Move, Joint, " +
      "Persistent Contact — plus <strong>Benchmark Sensor</strong> (a <em>Benchmark-category</em> " +
      "sample from <code>sample_benchmark.cpp</code> hosted here for convenience).",
    "Pick a sample · Launch (B) on Hits · Ctrl/Shift+click to grab/throw · Restart",
    wasm.version(),
    { category: "Events", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "<strong>Sensor Visit / Sensor Hits</strong> — sensor overlap events (kinematic sensor destroys " +
        "the visitor; launch a bullet sphere on Hits).<br>" +
        "<strong>Hit</strong> — a welded capsule chain drops onto a 6-material grid mesh; contact-hit " +
        "events draw yellow points + approach-speed rays labelled <em>speed, material</em>.<br>" +
        "<strong>Move</strong> — a spinning tall box; body move/sleep events shown in the readout.<br>" +
        "<strong>Joint</strong> — distance / prismatic / revolute / weld joints with force+torque " +
        "thresholds; drive a joint over threshold to destroy it via joint events — " +
        "<em>Ctrl+click</em> to grab and yank a body, or <em>Shift+click</em> to throw a projectile " +
        "at one.<br>" +
        "<strong>Persistent Contact</strong> — a rolling sphere; one contact id is tracked and its " +
        "manifold impulses drawn (crimson).<br>" +
        "<strong>Benchmark Sensor</strong> (Benchmark category) — full C-scale 40×40 grid; lime tint on " +
        "overlap; mid row fuchsia (custom filter active).",
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
        { label: "Hit", value: "hit" },
        { label: "Move", value: "move" },
        { label: "Joint", value: "joint" },
        { label: "Persistent Contact", value: "persistent" },
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

  // Debug-draw overlay (hit points / approach-speed rays / manifold impulses) and
  // 3D text labels (approach speed + material, impulse magnitudes).
  const overlay = new DebugDrawOverlay(demo);
  const textSprites = new TextLabelOverlay(demo);

  // Ground mesh wireframe (Hit / Persistent Contact), rebuilt on reset.
  const groundWireGroup = new THREE.Group();
  demo.content.add(groundWireGroup);
  function clearGroundWire() {
    for (const child of [...groundWireGroup.children]) {
      groundWireGroup.remove(child);
      const seg = child as THREE.LineSegments;
      seg.geometry.dispose();
      (seg.material as THREE.Material).dispose();
    }
  }
  function buildGroundWire() {
    clearGroundWire();
    if (!MESH_GROUND_SCENES.has(scene)) return;
    const edges = wasm.sensor_ground_wireframe();
    if (edges.length >= 6) groundWireGroup.add(makeWireEdges(edges, 0x556070));
  }

  function setCamera() {
    // C m_camera->SetView(yaw, pitch, distance, target) per ctor.
    if (scene === "visit") setView(demo, 0, 30, 20, [0, 5, 0]);
    else if (scene === "hits") setView(demo, 0, 30, 40, [0, 5, 0]);
    else if (scene === "hit") setView(demo, 0, 30, 100, [0, 5, 0]);
    else if (scene === "move") setView(demo, 0, 30, 40, [0, 5, 0]);
    else if (scene === "joint") setView(demo, 0, 30, 40, [0, 5, 0]);
    else if (scene === "persistent") setView(demo, 0, 30, 40, [0, 5, 0]);
    else setView(demo, 0, 0, 250, [0, 110, 0]); // benchmark: sample_benchmark.cpp ~746 SetView(0, 0, 250, {0, 110, 0})
  }

  // Move / Joint draw a box ground (pose index 0); Hit / Persistent draw a mesh
  // wireframe instead, and the sensor scenes have no single ground box.
  function groundIndexFor(): number | null {
    if (scene === "hits" || scene === "move" || scene === "joint") return 0;
    return null;
  }

  // Cache the sensor-index set (up to ~1600 u32 in the benchmark) and refetch it only when
  // Rust reports the sensor topology changed, instead of marshaling it every frame.
  let sensorIndicesCache = new Uint32Array(0);
  let sensorTopoVersion = -1;

  function reset() {
    wasm.sensor_reset(SCENE_ID[scene]);
    sensorTopoVersion = -1; // force a refetch of the sensor-index set after reset
    if (hitsRow) hitsRow.style.display = scene === "hits" ? "" : "none";
    overlay.clear();
    textSprites.clear();
    readout.innerHTML = "";
    buildGroundWire();
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

  // --- Grab / throw interaction on the Events scenes ---
  // sample_events.cpp is driven by the same Sample mouse shell as the dynamics
  // samples. The wasm mouse-shell exports (sensor_mouse_down/move/up/active,
  // sensor_spawn_random, sensor_delete_at_ray) map through makeInteractAdapter's
  // "sensor" prefix. Ctrl+click grabs a dynamic body with the mouse spring;
  // Shift+click throws a projectile along the pick ray — throwing bodies into the
  // Joint scene is what drives its joint-break events over threshold, matching C.
  // Guarded on the export family's existence so the page still loads if the wasm
  // pkg is mid-rebuild (interact stays null and no handlers attach).
  const hasMouseShell =
    typeof (wasm as Record<string, unknown>).sensor_mouse_down === "function";
  const interact = hasMouseShell ? makeInteractAdapter(wasm, "sensor") : null;
  let dragging = false;
  let grabFraction = 0;
  let suppressContext = false;

  const onPointerDown = (e: PointerEvent) => {
    if (!interact || !EVENTS_SCENES.has(scene)) return;
    if (e.button !== 0 && e.pointerType === "mouse") return;
    if (e.shiftKey) {
      const { origin, translation } = pickRay(demo, canvas, e.clientX, e.clientY);
      const variant = e.ctrlKey ? 1 : e.altKey ? 2 : 0;
      interact.sim_spawn_random(
        origin.x, origin.y, origin.z, translation.x, translation.y, translation.z, variant,
      );
      e.preventDefault();
      e.stopPropagation();
      suppressContext = true;
      return;
    }
    if (e.ctrlKey) {
      const { origin, translation } = pickRay(demo, canvas, e.clientX, e.clientY);
      const grab = interact.sim_mouse_down(
        origin.x, origin.y, origin.z, translation.x, translation.y, translation.z,
      );
      if (grab[0]! > 0.5) {
        dragging = true;
        const hit = new THREE.Vector3(grab[1]!, grab[2]!, grab[3]!);
        const t2 = translation.dot(translation);
        grabFraction = t2 > 0 ? hit.sub(origin).dot(translation) / t2 : 0;
        try {
          canvas.setPointerCapture(e.pointerId);
        } catch {
          /* pointer may not be capturable (e.g. synthetic events) */
        }
        e.preventDefault();
        e.stopPropagation();
      }
    }
  };

  const onPointerMove = (e: PointerEvent) => {
    if (!interact || !dragging || !interact.sim_mouse_active()) return;
    const { origin, translation } = pickRay(demo, canvas, e.clientX, e.clientY);
    origin.addScaledVector(translation, grabFraction);
    interact.sim_mouse_move(origin.x, origin.y, origin.z);
    e.preventDefault();
  };

  const onPointerUp = (e: PointerEvent) => {
    if (interact && dragging) {
      interact.sim_mouse_up();
      dragging = false;
      try {
        canvas.releasePointerCapture(e.pointerId);
      } catch {
        /* already released */
      }
    }
  };

  const onContext = (e: Event) => {
    if (suppressContext) {
      e.preventDefault();
      suppressContext = false;
    }
  };

  if (interact) {
    canvas.addEventListener("pointerdown", onPointerDown);
    canvas.addEventListener("pointermove", onPointerMove);
    canvas.addEventListener("pointerup", onPointerUp);
    canvas.addEventListener("pointercancel", onPointerUp);
    canvas.addEventListener("contextmenu", onContext);
  }

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
      groundIndex: groundIndexFor(),
    });

    if (EVENTS_SCENES.has(scene)) {
      overlay.update(wasm.sensor_overlay());
      textSprites.update(parseDebugText(wasm.sensor_debug_text()));
    }

    frame += 1;
    if (frame % 10 === 0) {
      if (EVENTS_SCENES.has(scene)) {
        // Events scenes drive the readout from the HUD JSON channel.
        try {
          const rows = JSON.parse(wasm.sensor_hud()) as { label: string; value: string }[];
          updateReadout(readout, Array.isArray(rows) ? rows : []);
        } catch {
          /* leave the readout as-is on a malformed frame */
        }
      } else {
        const st = wasm.sensor_event_stats();
        const beginLabel = scene === "benchmark" ? "max begin touch" : "begin touch count";
        const endLabel = scene === "benchmark" ? "max end touch" : "end touch count";
        updateReadout(readout, [
          { label: beginLabel, value: String(st[SENSOR_EVENT_STATS.begin]) },
          { label: endLabel, value: String(st[SENSOR_EVENT_STATS.end]) },
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
    if (interact) {
      canvas.removeEventListener("pointerdown", onPointerDown);
      canvas.removeEventListener("pointermove", onPointerMove);
      canvas.removeEventListener("pointerup", onPointerUp);
      canvas.removeEventListener("pointercancel", onPointerUp);
      canvas.removeEventListener("contextmenu", onContext);
    }
    overlay.dispose();
    textSprites.dispose();
    clearGroundWire();
    demo.content.remove(groundWireGroup);
    disposeMeshPool(pool);
    demo.dispose();
  };
}
