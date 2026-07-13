// Shapes — the 12 official Shapes samples (sample_shapes.cpp): Inclined Plane,
// Rolling Resistance, High Resistance, Isotropic Friction, Slide Twist, Restitution,
// Static Invoke, Conveyor Belt, Conveyor Mesh, Wind, Wind Drop, Wind Flap.

import * as THREE from "three";
import { createButton, createInfoBox } from "../controls.ts";
import {
  attachInteraction,
  makeInteractAdapter,
  type ParamDef,
  type ParamValues,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { demoPage, makeStyleGate, runLoop } from "./common.ts";
import { DemoScene, makeTriangleMesh, makeWireEdges, setView } from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

type Scene =
  | "inclined-plane"
  | "rolling-resistance"
  | "high-resistance"
  | "isotropic-friction"
  | "slide-twist"
  | "restitution"
  | "static-invoke"
  | "conveyor-belt"
  | "conveyor-mesh"
  | "wind"
  | "wind-drop"
  | "wind-flap";

export const SCENES: Scene[] = [
  "inclined-plane",
  "rolling-resistance",
  "high-resistance",
  "isotropic-friction",
  "slide-twist",
  "restitution",
  "static-invoke",
  "conveyor-belt",
  "conveyor-mesh",
  "wind",
  "wind-drop",
  "wind-flap",
];

// Numeric scene ids for shapes_reset() (mirror the Rust `shapes_reset` match arms;
// restitution/wind/conveyor-mesh have dedicated reset entry points).
const SCENE_ID: Record<Scene, number> = {
  "inclined-plane": 0,
  "rolling-resistance": 1,
  "high-resistance": 2,
  "isotropic-friction": 3,
  "slide-twist": 4,
  restitution: 5,
  "static-invoke": 6,
  "conveyor-belt": 7,
  "conveyor-mesh": 8,
  wind: 9,
  "wind-drop": 10,
  "wind-flap": 11,
};

const CONVEYOR_OBJ_URL = "/public/meshes/conveyor.obj";
let conveyorObjPromise: Promise<string> | null = null;
function loadConveyorObj(): Promise<string> {
  if (!conveyorObjPromise) {
    conveyorObjPromise = fetch(CONVEYOR_OBJ_URL).then((r) => {
      if (!r.ok) throw new Error(`conveyor.obj HTTP ${r.status}`);
      return r.text();
    });
  }
  return conveyorObjPromise;
}

const num = (v: ParamValues, k: string) => v[k] as number;
const str = (v: ParamValues, k: string) => v[k] as string;

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  assertRouteScenes("shapes", SCENES);
  const { canvas, controls } = demoPage(
    container,
    "Shapes",
    "The 12 official Shapes samples ported from <code>sample_shapes.cpp</code> — friction, " +
      "rolling resistance, restitution, conveyor belts (box &amp; mesh), and the wind/lift samples.",
    "Ctrl+click grab · Shift+click spawn · P/O/R",
    wasm.version(),
    { category: "Shapes", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "<strong>Inclined Plane / Rolling / High Resistance</strong> — friction &amp; rolling-resistance ramps.<br>" +
        "<strong>Isotropic Friction / Slide Twist</strong> — friction is direction-independent.<br>" +
        "<strong>Restitution</strong> — bounce ramp (Sphere / Box).<br>" +
        "<strong>Static Invoke</strong> — create a static shape at runtime (Invoke vs Passive contact).<br>" +
        "<strong>Conveyor Belt / Mesh</strong> — tangent-velocity surfaces.<br>" +
        "<strong>Wind / Wind Drop / Wind Flap</strong> — aerodynamic drag &amp; lift.",
    ),
  );

  let scene: Scene =
    initialScene && SCENES.includes(initialScene as Scene) ? (initialScene as Scene) : "inclined-plane";
  let ctrl!: SimControllerWithTick;

  // Gate for the async Conveyor Mesh reset (fetches conveyor.obj before the scene
  // exists in wasm). The loop skips stepping/rendering until this is true.
  let sceneReady = false;
  const hintEl = container.querySelector(".canvas-hint") as HTMLElement | null;
  const baseHint = hintEl?.textContent ?? "";
  const setLoading = (on: boolean) => {
    if (hintEl) hintEl.textContent = on ? "Loading conveyor mesh…" : baseHint;
  };

  const demo = new DemoScene(canvas, { target: [0, 7.5, 0], distance: 60, shadowExtent: 80 });
  demo.camera.far = 800;
  demo.camera.updateProjectionMatrix();
  const pool = createMeshPool();

  // --- Wind arrow (C Wind::Step DrawArrow, fuchsia) ---
  const windArrow = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 0.5, 0),
    1,
    0xff00ff,
    0.28,
    0.16,
  );
  windArrow.visible = false;
  demo.content.add(windArrow);

  // --- Conveyor mesh render group (static: rebuilt on reset) ---
  const conveyorGroup = new THREE.Group();
  demo.content.add(conveyorGroup);

  function clearConveyor() {
    for (const child of [...conveyorGroup.children]) {
      conveyorGroup.remove(child);
      const mesh = child as THREE.Mesh | THREE.LineSegments;
      mesh.geometry.dispose();
      const mat = mesh.material as THREE.Material | THREE.Material[];
      if (Array.isArray(mat)) mat.forEach((m) => m.dispose());
      else mat.dispose();
    }
  }

  // Build the colored conveyor surface (grouped by material color) + velocity lines.
  function buildConveyorRender() {
    clearConveyor();
    const tris = wasm.shapes_conveyor_mesh(); // 9 floats per triangle (world space)
    const colors = wasm.shapes_conveyor_colors(); // one 0xRRGGBB per triangle
    const triCount = Math.floor(tris.length / 9);
    // Group triangles by color so each color is one MeshStandardMaterial mesh.
    const byColor = new Map<number, number[]>();
    for (let t = 0; t < triCount; t++) {
      const c = colors[t]! >>> 0;
      let arr = byColor.get(c);
      if (!arr) {
        arr = [];
        byColor.set(c, arr);
      }
      const o = t * 9;
      for (let k = 0; k < 9; k++) arr.push(tris[o + k]!);
    }
    for (const [color, positions] of byColor) {
      conveyorGroup.add(makeTriangleMesh(new Float32Array(positions), color, 1.0));
    }
    const velLines = wasm.shapes_conveyor_velocity_lines();
    if (velLines.length >= 6) conveyorGroup.add(makeWireEdges(velLines, 0x8a2be2));
  }

  function updateWindArrow() {
    if (scene !== "wind") {
      windArrow.visible = false;
      return;
    }
    const a = wasm.shapes_wind_arrow(); // [x1,y1,z1,x2,y2,z2]
    if (a.length < 6) {
      windArrow.visible = false;
      return;
    }
    const dx = a[3]! - a[0]!;
    const dy = a[4]! - a[1]!;
    const dz = a[5]! - a[2]!;
    const len = Math.hypot(dx, dy, dz);
    windArrow.position.set(a[0]!, a[1]!, a[2]!);
    if (len > 1e-6) {
      windArrow.setDirection(new THREE.Vector3(dx / len, dy / len, dz / len));
      windArrow.setLength(len, len * 0.28, len * 0.16);
      windArrow.visible = true;
    } else {
      windArrow.visible = false;
    }
  }

  function setCamera() {
    // C m_camera->SetView(yaw, pitch, distance, target) per ctor.
    switch (scene) {
      case "inclined-plane": setView(demo, -55, 30, 60, [0, 7.5, 0]); break;
      case "rolling-resistance": setView(demo, -140, 17, 60, [0, 7.5, 0]); break;
      case "high-resistance": setView(demo, 0, 5, 40, [0, 7.5, 0]); break;
      case "isotropic-friction": setView(demo, 45, 30, 150, [0, 0, 0]); break;
      case "slide-twist": setView(demo, -30, 17, 30, [0, 5, 0]); break;
      case "restitution": setView(demo, 0, 25, 85, [0, 20, 0]); break;
      case "static-invoke": setView(demo, 0, 25, 10, [0, 1, 0]); break;
      case "conveyor-belt": setView(demo, 0, 25, 40, [0, 1, 0]); break;
      case "conveyor-mesh": setView(demo, 65, 25, 28, [0, 1, 0]); break;
      case "wind": setView(demo, 0, 0, 5, [0, 1, 0]); break;
      case "wind-drop": setView(demo, -45, 15, 20, [0, 5, 0]); break;
      case "wind-flap": setView(demo, -35, 15, 65, [0, 5, 10]); break;
    }
  }

  const windShapeId = (s: string): number => (s === "circle" ? 0 : s === "capsule" ? 1 : 2);

  function reset(): Promise<void> {
    clearConveyor();
    windArrow.visible = false;
    const p = ctrl.params;
    if (scene === "restitution") {
      wasm.shapes_reset_restitution(str(p, "restShape") === "box");
    } else if (scene === "wind") {
      wasm.shapes_reset_wind(windShapeId(str(p, "windShape")), num(p, "windCount"));
      wasm.shapes_set_wind_live(num(p, "windX"), num(p, "windDrag"), num(p, "windLift"));
    } else if (scene === "static-invoke") {
      wasm.shapes_reset(SCENE_ID["static-invoke"]);
      wasm.shapes_set_invoke(str(p, "invoke") === "invoke");
    } else if (scene === "conveyor-mesh") {
      // Async: the scene does not exist in wasm until conveyor.obj is fetched and
      // shapes_reset_conveyor runs. Hold the ready gate (surfacing a loading
      // state) so the loop's first tick never calls shapes_step on an
      // uninitialized scene — a cold deep-link would otherwise panic.
      sceneReady = false;
      setLoading(true);
      setCamera();
      return loadConveyorObj()
        .then((txt) => {
          if (scene !== "conveyor-mesh") return; // scene changed while fetching
          wasm.shapes_reset_conveyor(txt);
          buildConveyorRender();
          sceneReady = true;
          setLoading(false);
        })
        .catch((err) => {
          console.warn("conveyor.obj load failed", err);
          setLoading(false);
        });
    } else {
      wasm.shapes_reset(SCENE_ID[scene]);
    }
    setCamera();
    sceneReady = true;
    return Promise.resolve();
  }

  // --- Custom Static Invoke Create / Destroy button (C StaticInvoke::DrawControls) ---
  const invokeRow = document.createElement("div");
  invokeRow.className = "control-row";
  const invokeBtn = createButton(
    "Create",
    () => {
      if (wasm.shapes_static_exists()) wasm.shapes_destroy_static();
      else wasm.shapes_create_static();
      updateInvokeButton();
    },
    false,
  );
  invokeRow.appendChild(invokeBtn);
  invokeRow.style.display = "none";
  function updateInvokeButton() {
    if (scene !== "static-invoke") return;
    invokeBtn.textContent = wasm.shapes_static_exists() ? "Destroy" : "Create";
  }

  const on = (s: Scene) => ({ key: "sample", equals: s });
  const params: ParamDef[] = [
    {
      type: "select",
      key: "sample",
      label: "Sample",
      options: SCENES.map((s) => ({ label: sceneLabel(s), value: s })),
      default: scene,
      restart: true,
    },
    // Restitution (C Restitution::DrawControls — Sphere / Box radios rebuild).
    {
      type: "select",
      key: "restShape",
      label: "Shape",
      options: [
        { label: "Sphere", value: "sphere" },
        { label: "Box", value: "box" },
      ],
      default: "sphere",
      restart: true,
      group: "Restitution",
      visibleWhen: on("restitution"),
    },
    // Static Invoke (C StaticInvoke::DrawControls — Invoke / Passive radios).
    {
      type: "select",
      key: "invoke",
      label: "Contact",
      options: [
        { label: "Invoke", value: "invoke" },
        { label: "Passive", value: "passive" },
      ],
      default: "passive",
      restart: false,
      group: "Static Invoke",
      visibleWhen: on("static-invoke"),
    },
    // Wind (C Wind::DrawControls — Shape combo + Wind/Drag/Lift/Count sliders).
    {
      type: "select",
      key: "windShape",
      label: "Shape",
      options: [
        { label: "Circle", value: "circle" },
        { label: "Capsule", value: "capsule" },
        { label: "Box", value: "box" },
      ],
      default: "box",
      restart: true,
      group: "Wind",
      visibleWhen: on("wind"),
    },
    { type: "slider", key: "windX", label: "Wind", min: -50, max: 50, step: 0.1, default: 6, restart: false, group: "Wind", visibleWhen: on("wind") },
    { type: "slider", key: "windDrag", label: "Drag", min: 0, max: 1, step: 0.01, default: 1, restart: false, group: "Wind", visibleWhen: on("wind") },
    { type: "slider", key: "windLift", label: "Lift", min: 0, max: 4, step: 0.01, default: 0.75, restart: false, group: "Wind", visibleWhen: on("wind") },
    { type: "slider", key: "windCount", label: "Count", min: 1, max: 60, step: 1, default: 10, restart: true, group: "Wind", visibleWhen: on("wind") },
  ];

  ctrl = attachInteraction({
    wasm: makeInteractAdapter(wasm, "shapes"),
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: "Shapes",
    sampleCategory: "Shapes",
    enableSpawnDelete: true,
    params,
    onParamsChange: (values, key) => {
      if (key === "sample") {
        scene = values.sample as Scene;
        invokeRow.style.display = scene === "static-invoke" ? "" : "none";
        return; // restart re-applies the active scene via reset()
      }
      if (key === "invoke") {
        wasm.shapes_set_invoke(str(values, "invoke") === "invoke");
      } else if (key === "windX" || key === "windDrag" || key === "windLift") {
        wasm.shapes_set_wind_live(num(values, "windX"), num(values, "windDrag"), num(values, "windLift"));
      }
    },
  }) as SimControllerWithTick;

  // Place the Create/Destroy button right below the parameter panel so it sits
  // next to the Invoke/Passive radios (C StaticInvoke::DrawControls order).
  const paramPanel = controls.querySelector(".param-panel");
  if (paramPanel && paramPanel.parentElement) {
    paramPanel.parentElement.insertBefore(invokeRow, paramPanel.nextSibling);
  } else {
    controls.appendChild(invokeRow);
  }
  invokeRow.style.display = scene === "static-invoke" ? "" : "none";

  reset();

  const styleGate = makeStyleGate<Uint32Array>();
  const stop = runLoop(
    () => {
      ctrl.tickFrame();
      // Awake-gated style refetch (shapes_styles is a full world_draw capture).
      const awake = wasm.shapes_counters()[5] ?? 0;
      const styles = styleGate(awake, () => wasm.shapes_styles());
      syncMeshesFromPoses(demo.content, pool, wasm.shapes_poses(), {
        groundIndex: 0,
        styles,
      });
      updateWindArrow();
      updateInvokeButton();
      demo.render();
    },
    controls,
    { ready: () => sceneReady },
  );

  return () => {
    stop();
    clearConveyor();
    demo.content.remove(conveyorGroup);
    demo.content.remove(windArrow);
    windArrow.dispose();
    ctrl.dispose();
    disposeMeshPool(pool);
    demo.dispose();
  };
}

function sceneLabel(s: Scene): string {
  return s
    .split("-")
    .map((w) => w.charAt(0).toUpperCase() + w.slice(1))
    .join(" ");
}
