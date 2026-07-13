// Robustness — the four sample_robustness.cpp samples: HighMassRatio1, Tiny
// Pyramid, Overlap Recovery, Overflow Color Pile. All bodies are boxes (from the
// ported hull builder); Overlap Recovery rebuilds live from C DrawControls sliders
// and Overflow Color Pile reads back the constraint-graph color overflow.

import {
  createButtonGroup,
  createInfoBox,
  createReadout,
  createSlider,
  updateReadout,
} from "../controls.ts";
import {
  attachInteraction,
  makeInteractAdapter,
  type ParamValues,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { demoPage, makeStyleGate, runLoop } from "./common.ts";
import { DemoScene, setView } from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

type Mode = "high-mass-ratio1" | "tiny-pyramid" | "overlap-recovery" | "overflow-color-pile";

export const SCENES: Mode[] = [
  "high-mass-ratio1",
  "tiny-pyramid",
  "overlap-recovery",
  "overflow-color-pile",
];

const NAMES: Record<Mode, string> = {
  "high-mass-ratio1": "HighMassRatio1",
  "tiny-pyramid": "Tiny Pyramid",
  "overlap-recovery": "Overlap Recovery",
  "overflow-color-pile": "Overflow Color Pile",
};

const SCENE_ID: Record<Mode, number> = {
  "high-mass-ratio1": 0,
  "tiny-pyramid": 1,
  "overlap-recovery": 2,
  "overflow-color-pile": 3,
};

// C SetView(yaw, pitch, distance, target) per sample_robustness.cpp.
const CAMERAS: Record<Mode, [number, number, number, [number, number, number]]> = {
  "high-mass-ratio1": [30, 15, 70, [0, 0, 0]], // :24
  "tiny-pyramid": [-30, 20, 10, [0, 0.5, 0]], // :83
  "overlap-recovery": [45, 20, 15, [0, 0, 0]], // :139
  "overflow-color-pile": [30, 35, 15, [0, 0, 0]], // :252
};

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  assertRouteScenes("robustness", SCENES);
  const { canvas, controls } = demoPage(
    container,
    "Robustness",
    "The four <code>sample_robustness.cpp</code> samples: <strong>HighMassRatio1</strong> " +
      "(pyramids capped by 100–300× density boxes), <strong>Tiny Pyramid</strong> (5 cm boxes), " +
      "<strong>Overlap Recovery</strong> (intentionally interpenetrating pyramid, tuned live), and " +
      "<strong>Overflow Color Pile</strong> (a heavy hub ringed to overflow the constraint-graph " +
      "colors).",
    "Drag to orbit · pick a scene · Overlap Recovery sliders · P/O/R",
    wasm.version(),
    { category: "Robustness", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "Overlap Recovery exposes the C <code>DrawControls</code> sliders driving " +
        "<code>b3World_SetContactTuning</code> (Hertz / Damping / push-out Speed) and rebuilds the " +
        "pyramid on change. Overflow Color Pile reads <code>b3World_GetCounters</code> — the last " +
        "graph color is the overflow bucket.",
    ),
  );

  let mode: Mode =
    initialScene && SCENES.includes(initialScene as Mode) ? (initialScene as Mode) : "high-mass-ratio1";

  // Overlap Recovery live tunables (C OverlapRecovery ctor defaults).
  let orExtent = 0.5;
  let orBaseCount = 4;
  let orOverlap = 0.25;
  let orSpeed = 3;
  let orHertz = 30;
  let orDamping = 10;

  function applyOverlapParams() {
    wasm.robustness_set_overlap_params(orExtent, orBaseCount, orOverlap, orSpeed, orHertz, orDamping);
  }

  // Overlap Recovery slider panel (shown only for that scene).
  const recoveryControls = document.createElement("div");
  function rebuildRecoveryControls() {
    recoveryControls.replaceChildren();
    if (mode !== "overlap-recovery") {
      recoveryControls.style.display = "none";
      return;
    }
    recoveryControls.style.display = "";
    // C DrawControls (:204): Extent, Base Count, Overlap, Speed, Hertz, Damping Ratio, Reset.
    recoveryControls.appendChild(
      createSlider("Extent", 0.1, 1.0, orExtent, 0.1, (v) => {
        orExtent = v;
        applyOverlapParams();
      }),
    );
    recoveryControls.appendChild(
      createSlider("Base Count", 1, 10, orBaseCount, 1, (v) => {
        orBaseCount = Math.round(v);
        applyOverlapParams();
      }),
    );
    recoveryControls.appendChild(
      createSlider("Overlap", 0, 1, orOverlap, 0.01, (v) => {
        orOverlap = v;
        applyOverlapParams();
      }),
    );
    recoveryControls.appendChild(
      createSlider("Speed", 0, 10, orSpeed, 0.1, (v) => {
        orSpeed = v;
        applyOverlapParams();
      }),
    );
    recoveryControls.appendChild(
      createSlider("Hertz", 0, 240, orHertz, 1, (v) => {
        orHertz = v;
        applyOverlapParams();
      }),
    );
    recoveryControls.appendChild(
      createSlider("Damping Ratio", 0, 20, orDamping, 0.1, (v) => {
        orDamping = v;
        applyOverlapParams();
      }),
    );
    recoveryControls.appendChild(
      createButtonGroup([{ label: "Reset Scene", value: "reset" }], "reset", () =>
        applyOverlapParams(),
      ),
    );
  }

  controls.appendChild(
    createButtonGroup(
      SCENES.map((s) => ({ label: NAMES[s], value: s })),
      mode,
      (v) => {
        mode = v as Mode;
        const nameEl = controls.querySelector(".sample-name");
        if (nameEl) nameEl.textContent = NAMES[mode];
        rebuildRecoveryControls();
        reset();
      },
    ),
  );
  controls.appendChild(recoveryControls);

  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { target: [0, 0, 0], distance: 20 });
  demo.camera.far = 400;
  demo.camera.updateProjectionMatrix();
  const pool = createMeshPool();

  function reset() {
    syncMeshesFromPoses(demo.content, pool, []);
    wasm.robustness_reset_scene(SCENE_ID[mode]);
    if (mode === "overlap-recovery") applyOverlapParams();
    const [yaw, pitch, dist, target] = CAMERAS[mode];
    setView(demo, yaw, pitch, dist, target);
  }

  let subSteps = 4;
  const ctrl = attachInteraction({
    wasm: makeInteractAdapter(wasm, "robustness"),
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: NAMES[mode],
    sampleCategory: "Robustness",
    params: [
      {
        type: "slider",
        key: "subSteps",
        label: "Sub-steps",
        min: 1,
        max: 8,
        step: 1,
        default: 4,
        restart: false,
      },
    ],
    onParamsChange: (values: ParamValues) => {
      subSteps = Number(values.subSteps) || 4;
      ctrl.subSteps = subSteps;
    },
  }) as SimControllerWithTick;
  ctrl.subSteps = subSteps;

  rebuildRecoveryControls();
  reset();

  let frame = 0;
  // These settle-heavy scenes (high mass ratio, tiny pyramid, recovery) go to
  // sleep; stop the per-frame world_draw style capture once nothing is awake.
  const styleGate = makeStyleGate<Uint32Array>();
  const stop = runLoop(() => {
    ctrl.tickFrame();
    const poses = wasm.robustness_poses();
    const awake = wasm.robustness_counters()[5] ?? 0;
    const styles = styleGate(awake, () => wasm.robustness_styles());
    syncMeshesFromPoses(demo.content, pool, poses, { styles });

    frame += 1;
    if (frame % 15 === 0) {
      if (mode === "overflow-color-pile") {
        const s = wasm.robustness_overflow_stats();
        updateReadout(readout, [
          { label: "scene", value: NAMES[mode] },
          { label: "neighbors", value: String(s[0] ?? 0) },
          { label: "overflow contacts", value: String(s[1] ?? 0) },
          { label: "total contacts", value: String(s[2] ?? 0) },
        ]);
      } else if (mode === "tiny-pyramid") {
        updateReadout(readout, [
          { label: "scene", value: NAMES[mode] },
          { label: "boxes", value: `${wasm.robustness_tiny_cm().toFixed(1)}cm` },
          { label: "bodies", value: String(wasm.robustness_body_count()) },
        ]);
      } else {
        updateReadout(readout, [
          { label: "scene", value: NAMES[mode] },
          { label: "bodies", value: String(wasm.robustness_body_count()) },
          { label: "frame", value: String(frame) },
        ]);
      }
    }
    demo.render();
  }, readout);

  return () => {
    ctrl.dispose();
    stop();
    disposeMeshPool(pool);
    demo.dispose();
  };
}
