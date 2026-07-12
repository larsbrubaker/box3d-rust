// Shared simulation interaction layer: Samples App Info panel (C DrawInfoPanel),
// pause/step/restart, pick-drag, spawn/delete, Solver/Recording, debug-draw.

import * as THREE from "three";
import {
  createButton,
  createButtonGroup,
  createCheckbox,
  createCollapsingSection,
  createSlider,
  createTextInput,
} from "./controls.ts";
import type { DemoScene } from "./three-scene.ts";

/** Wasm bindings needed by the interaction layer (sim_* today; other demos can adapt). */
export type InteractWasm = {
  sim_step(dt: number, sub_steps: number): number;
  sim_body_poses(): Float32Array;
  sim_mouse_down(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  sim_mouse_move(px: number, py: number, pz: number): void;
  sim_mouse_up(): void;
  sim_mouse_active(): boolean;
  sim_spawn_random(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  sim_delete_at_ray(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): number;
  sim_counters(): Float32Array;
  sim_debug_draw(flags: number): Float32Array;
  sim_step_count?(): number;
  sim_set_enable_sleep?(flag: boolean): void;
  sim_set_enable_warm_starting?(flag: boolean): void;
  sim_set_enable_continuous?(flag: boolean): void;
  sim_set_recycle_distance?(meters: number): void;
  sim_start_recording?(): void;
  sim_stop_recording?(): Uint8Array;
  sim_is_recording?(): boolean;
  sim_record_start_step?(): number;
};

export const DRAW_CONTACTS = 1 << 0;
export const DRAW_CONTACT_NORMALS = 1 << 1;
export const DRAW_CONTACT_FORCES = 1 << 2;
export const DRAW_JOINTS = 1 << 3;
export const DRAW_JOINT_EXTRAS = 1 << 4;
export const DRAW_BOUNDS = 1 << 5;
export const DRAW_MASS = 1 << 6;
export const DRAW_ISLANDS = 1 << 7;

export type ParamDef =
  | {
      type: "slider";
      key: string;
      label: string;
      min: number;
      max: number;
      step: number;
      default: number;
      /** Restart the scene when this value changes (default true). */
      restart?: boolean;
    }
  | {
      type: "checkbox";
      key: string;
      label: string;
      default: boolean;
      restart?: boolean;
    }
  | {
      type: "select";
      key: string;
      label: string;
      options: { label: string; value: string }[];
      default: string;
      restart?: boolean;
    };

export type ParamValues = Record<string, number | boolean | string>;

export type SimController = {
  paused: boolean;
  /** Remaining single-steps to run while paused. */
  stepsPending: number;
  timeScale: number;
  subSteps: number;
  /** Solver frequency in Hz (C sample Hertz). dt = 1/hertz. */
  hertz: number;
  debugFlags: number;
  /** Rolling average step time in ms. */
  stepMsAvg: number;
  stepCount: number;
  params: ParamValues;
  setPaused(v: boolean): void;
  requestStep(n?: number): void;
  restart(): void;
  dispose(): void;
};

export type AttachInteractionOpts = {
  wasm: InteractWasm;
  demo: DemoScene;
  canvas: HTMLCanvasElement;
  controls: HTMLElement;
  /** Rebuild the scene (called on R / Restart / restart-on-change params). */
  onRestart: () => void;
  /** Sample name shown in goldenrod (C entry.Name). */
  sampleName?: string;
  /** Category shown in light gray (C entry.Category). */
  sampleCategory?: string;
  /** Optional declarative parameter panel (applied before solver). */
  params?: ParamDef[];
  onParamsChange?: (values: ParamValues, changedKey: string) => void;
  /** Hide spawn/delete hints when false. Default true. */
  enableSpawnDelete?: boolean;
  /** Show Solver + Recording panels. Default true. */
  showSolverPanel?: boolean;
  /** Fallback base dt when hertz is unavailable (default 1/60). */
  baseDt?: number;
};

const _ndc = new THREE.Vector2();
const _raycaster = new THREE.Raycaster();
const _hitPlane = new THREE.Plane();
const _planeHit = new THREE.Vector3();
const _camDir = new THREE.Vector3();
const _grabPoint = new THREE.Vector3();
const _camOffset = new THREE.Vector3();

function screenToNdc(canvas: HTMLCanvasElement, clientX: number, clientY: number): THREE.Vector2 {
  const rect = canvas.getBoundingClientRect();
  _ndc.x = ((clientX - rect.left) / rect.width) * 2 - 1;
  _ndc.y = -((clientY - rect.top) / rect.height) * 2 + 1;
  return _ndc;
}

function pickRay(
  demo: DemoScene,
  canvas: HTMLCanvasElement,
  clientX: number,
  clientY: number,
): { origin: THREE.Vector3; translation: THREE.Vector3 } {
  screenToNdc(canvas, clientX, clientY);
  _raycaster.setFromCamera(_ndc, demo.camera);
  const origin = _raycaster.ray.origin.clone();
  const translation = _raycaster.ray.direction.clone().multiplyScalar(200);
  return { origin, translation };
}

function dragPointOnCameraPlane(
  demo: DemoScene,
  canvas: HTMLCanvasElement,
  clientX: number,
  clientY: number,
  planePoint: THREE.Vector3,
): THREE.Vector3 {
  screenToNdc(canvas, clientX, clientY);
  _raycaster.setFromCamera(_ndc, demo.camera);
  demo.camera.getWorldDirection(_camDir);
  _hitPlane.setFromNormalAndCoplanarPoint(_camDir, planePoint);
  if (_raycaster.ray.intersectPlane(_hitPlane, _planeHit)) {
    return _planeHit.clone();
  }
  return _raycaster.ray.origin.clone().add(
    _raycaster.ray.direction.clone().multiplyScalar(planePoint.distanceTo(_raycaster.ray.origin)),
  );
}

function cameraReadout(demo: DemoScene): {
  px: number;
  py: number;
  pz: number;
  yaw: number;
  pitch: number;
  radius: number;
} {
  const t = demo.controls.target;
  _camOffset.copy(demo.camera.position).sub(t);
  const radius = Math.max(1e-6, _camOffset.length());
  const yaw = (Math.atan2(_camOffset.x, _camOffset.z) * 180) / Math.PI;
  const pitch = (Math.asin(Math.max(-1, Math.min(1, _camOffset.y / radius))) * 180) / Math.PI;
  return { px: t.x, py: t.y, pz: t.z, yaw, pitch, radius };
}

function downloadBytes(filename: string, data: Uint8Array) {
  const blob = new Blob([data], { type: "application/octet-stream" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename || "recording.b3rec";
  a.click();
  URL.revokeObjectURL(url);
}

/** Build a declarative parameter panel; returns current values + dispose. */
export function createParamPanel(
  parent: HTMLElement,
  defs: ParamDef[],
  onChange: (values: ParamValues, key: string, shouldRestart: boolean) => void,
): { values: ParamValues; dispose: () => void } {
  const values: ParamValues = {};
  const section = document.createElement("div");
  section.className = "param-panel";
  const title = document.createElement("div");
  title.className = "control-section-title";
  title.textContent = "Parameters";
  section.appendChild(title);

  for (const def of defs) {
    const restart = def.restart !== false;
    if (def.type === "slider") {
      values[def.key] = def.default;
      section.appendChild(
        createSlider(def.label, def.min, def.max, def.default, def.step, (v) => {
          values[def.key] = def.step >= 1 ? Math.round(v) : v;
          onChange(values, def.key, restart);
        }),
      );
    } else if (def.type === "checkbox") {
      values[def.key] = def.default;
      section.appendChild(
        createCheckbox(def.label, def.default, (v) => {
          values[def.key] = v;
          onChange(values, def.key, restart);
        }),
      );
    } else {
      values[def.key] = def.default;
      const group = document.createElement("div");
      group.className = "control-group";
      const lbl = document.createElement("label");
      lbl.textContent = def.label;
      group.appendChild(lbl);
      group.appendChild(
        createButtonGroup(def.options, def.default, (v) => {
          values[def.key] = v;
          onChange(values, def.key, restart);
        }),
      );
      section.appendChild(group);
    }
  }

  parent.appendChild(section);
  return {
    values,
    dispose: () => section.remove(),
  };
}

/** Three.js line/point overlay fed by `sim_debug_draw`. */
export class DebugDrawOverlay {
  private lines: THREE.LineSegments | null = null;
  private points: THREE.Points | null = null;
  private readonly group: THREE.Group;

  constructor(private readonly demo: DemoScene) {
    this.group = new THREE.Group();
    this.demo.dynamic.add(this.group);
  }

  update(data: ArrayLike<number>) {
    this.clear();
    if (data.length < 2) return;
    const segCount = data[0]! | 0;
    const pointCount = data[1]! | 0;
    let o = 2;

    if (segCount > 0) {
      const positions: number[] = [];
      const colors: number[] = [];
      for (let i = 0; i < segCount; i++) {
        const x1 = data[o++]!;
        const y1 = data[o++]!;
        const z1 = data[o++]!;
        const x2 = data[o++]!;
        const y2 = data[o++]!;
        const z2 = data[o++]!;
        const rgbBits = data[o++]!;
        const rgb = floatBitsToRgb(rgbBits);
        positions.push(x1, y1, z1, x2, y2, z2);
        colors.push(rgb[0], rgb[1], rgb[2], rgb[0], rgb[1], rgb[2]);
      }
      const geo = new THREE.BufferGeometry();
      geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
      geo.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
      const mat = new THREE.LineBasicMaterial({
        vertexColors: true,
        depthTest: true,
        transparent: true,
        opacity: 0.9,
      });
      this.lines = new THREE.LineSegments(geo, mat);
      this.group.add(this.lines);
    }

    if (pointCount > 0) {
      const positions: number[] = [];
      const colors: number[] = [];
      for (let i = 0; i < pointCount; i++) {
        const x = data[o++]!;
        const y = data[o++]!;
        const z = data[o++]!;
        o++; // size — Points uses fixed size
        const rgbBits = data[o++]!;
        const rgb = floatBitsToRgb(rgbBits);
        positions.push(x, y, z);
        colors.push(rgb[0], rgb[1], rgb[2]);
      }
      const geo = new THREE.BufferGeometry();
      geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
      geo.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
      const mat = new THREE.PointsMaterial({
        size: 0.12,
        vertexColors: true,
        depthTest: true,
        sizeAttenuation: true,
      });
      this.points = new THREE.Points(geo, mat);
      this.group.add(this.points);
    }
  }

  clear() {
    while (this.group.children.length > 0) {
      const obj = this.group.children[0]!;
      this.group.remove(obj);
      const mesh = obj as THREE.LineSegments | THREE.Points;
      mesh.geometry?.dispose();
      const mat = mesh.material;
      if (Array.isArray(mat)) mat.forEach((m) => m.dispose());
      else mat?.dispose();
    }
    this.lines = null;
    this.points = null;
  }

  dispose() {
    this.clear();
    this.demo.dynamic.remove(this.group);
  }
}

function floatBitsToRgb(bitsAsF32: number): [number, number, number] {
  const u = new Float32Array([bitsAsF32]);
  const view = new Uint32Array(u.buffer);
  const hex = view[0]! & 0x00ffffff;
  return [((hex >> 16) & 0xff) / 255, ((hex >> 8) & 0xff) / 255, (hex & 0xff) / 255];
}

/**
 * Attach Samples App Info panel + pause/step/restart, hotkeys, Solver/Recording,
 * pick-drag, spawn/delete, and debug-draw toggles to a dynamics demo.
 */
export function attachInteraction(opts: AttachInteractionOpts): SimControllerWithTick {
  const {
    wasm,
    demo,
    canvas,
    controls,
    onRestart,
    sampleName = "Sample",
    sampleCategory = "Dynamics",
    params: paramDefs = [],
    onParamsChange,
    enableSpawnDelete = true,
    showSolverPanel = true,
    baseDt = 1 / 60,
  } = opts;

  controls.classList.add("samples-info-panel");

  const state: SimController = {
    paused: false,
    stepsPending: 0,
    timeScale: 1,
    subSteps: 4,
    hertz: 60,
    debugFlags: 0,
    stepMsAvg: 0,
    stepCount: 0,
    params: {},
    setPaused(v) {
      state.paused = v;
      pauseBtn.classList.toggle("active", v);
      pauseBtn.textContent = v ? "Resume (P)" : "Pause (P)";
      pauseBadge.hidden = !v;
    },
    requestStep(n = 1) {
      state.stepsPending += n;
    },
    restart() {
      wasm.sim_mouse_up();
      if (wasm.sim_is_recording?.()) {
        // Drop in-progress recording on restart (C SelectSample recreates world).
        wasm.sim_stop_recording?.();
        updateRecordingUi(false);
      }
      onRestart();
      state.stepCount = 0;
    },
    dispose() {
      /* filled below */
    },
  };

  // --- Info header (C DrawInfoPanel top) ---
  const infoHead = document.createElement("div");
  infoHead.className = "samples-info-head";
  infoHead.innerHTML = `
    <div class="sample-name">${escapeHtml(sampleName)}</div>
    <div class="sample-category">${escapeHtml(sampleCategory)}</div>
    <div class="sample-paused" hidden>PAUSED <span class="sample-paused-hint">(P)</span></div>
    <div class="sample-sep"></div>
    <div class="sample-stats">
      <div class="sample-stat frame-ms">0.0 ms</div>
      <div class="sample-stat step-count">step 0</div>
    </div>
    <div class="sample-sep"></div>
    <div class="sample-camera">
      <div class="sample-stat cam-pivot">pivot m (0.0, 0.0, 0.0)</div>
      <div class="sample-stat cam-yaw">yaw/pitch (0.0, 0.0)</div>
      <div class="sample-stat cam-radius">radius m 0.0</div>
    </div>
    <div class="sample-sep"></div>
  `;
  controls.appendChild(infoHead);
  const pauseBadge = infoHead.querySelector(".sample-paused") as HTMLElement;
  const frameMsEl = infoHead.querySelector(".frame-ms") as HTMLElement;
  const stepCountEl = infoHead.querySelector(".step-count") as HTMLElement;
  const camPivotEl = infoHead.querySelector(".cam-pivot") as HTMLElement;
  const camYawEl = infoHead.querySelector(".cam-yaw") as HTMLElement;
  const camRadiusEl = infoHead.querySelector(".cam-radius") as HTMLElement;

  // --- Parameter panel ---
  let paramDispose = () => {};
  if (paramDefs.length > 0) {
    const panel = createParamPanel(controls, paramDefs, (values, key, shouldRestart) => {
      state.params = { ...values };
      onParamsChange?.(values, key);
      if (shouldRestart) state.restart();
    });
    state.params = { ...panel.values };
    paramDispose = panel.dispose;
  }

  // --- Transport ---
  const transport = document.createElement("div");
  transport.className = "control-row samples-transport";
  const pauseBtn = createButton("Pause (P)", () => state.setPaused(!state.paused));
  const stepBtn = createButton("Step (O)", () => {
    state.setPaused(true);
    state.requestStep(1);
  });
  transport.append(pauseBtn, stepBtn);
  controls.appendChild(transport);

  controls.appendChild(
    createSlider("Time scale", 0, 2, 1, 0.05, (v) => {
      state.timeScale = v;
    }),
  );

  // --- Solver (C CollapsingHeader) ---
  let recordingFile = "recording.b3rec";
  let recordingStatusEl: HTMLElement | null = null;
  let recordRestartBtn: HTMLButtonElement | null = null;
  let recordNowBtn: HTMLButtonElement | null = null;
  let recordStopBtn: HTMLButtonElement | null = null;

  const updateRecordingUi = (active: boolean) => {
    if (!recordingStatusEl || !recordRestartBtn || !recordNowBtn || !recordStopBtn) return;
    recordRestartBtn.hidden = active;
    recordNowBtn.hidden = active;
    recordStopBtn.hidden = !active;
    if (active) {
      const from = wasm.sim_record_start_step?.() ?? 0;
      recordingStatusEl.textContent = `recording (from step ${from})`;
      recordingStatusEl.hidden = false;
    } else {
      recordingStatusEl.hidden = true;
      recordingStatusEl.textContent = "";
    }
  };

  if (showSolverPanel) {
    const solver = createCollapsingSection("Solver", true);
    controls.appendChild(solver.root);

    solver.body.appendChild(
      createSlider("Sub-steps", 1, 50, state.subSteps, 1, (v) => {
        state.subSteps = Math.round(v);
      }),
    );
    solver.body.appendChild(
      createSlider("Hertz", 5, 240, state.hertz, 1, (v) => {
        state.hertz = Math.round(v);
      }),
    );

    const workers = createSlider("Workers", 1, 8, 1, 1, () => {});
    workers.querySelector("input")!.setAttribute("disabled", "true");
    workers.title = "WASM demos run single-threaded (serial port)";
    const workersNote = document.createElement("div");
    workersNote.className = "control-note";
    workersNote.textContent = "Workers: 1 (WASM is serial)";
    solver.body.appendChild(workers);
    solver.body.appendChild(workersNote);

    solver.body.appendChild(
      createSlider("Recycle", 0, 10, 0, 0.1, (cm) => {
        const meters = 0.01 * cm;
        wasm.sim_set_recycle_distance?.(meters);
      }),
    );

    solver.body.appendChild(
      createCheckbox("Sleep", true, (v) => wasm.sim_set_enable_sleep?.(v)),
    );
    solver.body.appendChild(
      createCheckbox("Warm Starting", true, (v) => wasm.sim_set_enable_warm_starting?.(v)),
    );
    solver.body.appendChild(
      createCheckbox("Continuous", true, (v) => wasm.sim_set_enable_continuous?.(v)),
    );

    const restartBtn = createButton("Restart", () => state.restart());
    restartBtn.classList.add("control-btn-block");
    solver.body.appendChild(restartBtn);

    // --- Recording ---
    const hasRecordingApi = typeof wasm.sim_start_recording === "function";
    const recording = createCollapsingSection("Recording", true);
    controls.appendChild(recording.root);

    const fileField = createTextInput("File", recordingFile, (v) => {
      recordingFile = v.trim() || "recording.b3rec";
    });
    recording.body.appendChild(fileField.root);

    if (hasRecordingApi) {
      recordRestartBtn = createButton("Record (restart)", () => {
        state.restart();
        wasm.sim_start_recording?.();
        updateRecordingUi(true);
      });
      recordNowBtn = createButton("Record Now", () => {
        wasm.sim_start_recording?.();
        updateRecordingUi(true);
      });
      recordStopBtn = createButton("Stop", () => {
        const bytes = wasm.sim_stop_recording?.() ?? new Uint8Array();
        updateRecordingUi(false);
        if (bytes.length > 0) downloadBytes(recordingFile, bytes);
      });
      recordStopBtn.hidden = true;
      recordingStatusEl = document.createElement("div");
      recordingStatusEl.className = "sample-stat recording-status";
      recordingStatusEl.hidden = true;
      const row = document.createElement("div");
      row.className = "control-row";
      row.append(recordRestartBtn, recordNowBtn, recordStopBtn);
      recording.body.appendChild(row);
      recording.body.appendChild(recordingStatusEl);
    } else {
      const note = document.createElement("div");
      note.className = "control-note";
      note.textContent = "Recording API coming soon";
      const disabledRow = document.createElement("div");
      disabledRow.className = "control-row";
      const a = createButton("Record (restart)", () => {});
      const b = createButton("Record Now", () => {});
      a.disabled = true;
      b.disabled = true;
      disabledRow.append(a, b);
      recording.body.append(disabledRow, note);
    }
  }

  // --- Debug draw toggles ---
  const dbg = createCollapsingSection("Debug draw", false);
  controls.appendChild(dbg.root);

  const flagDefs: { label: string; flag: number }[] = [
    { label: "Contacts", flag: DRAW_CONTACTS },
    { label: "Contact normals", flag: DRAW_CONTACT_NORMALS },
    { label: "Contact forces", flag: DRAW_CONTACT_FORCES },
    { label: "Joints", flag: DRAW_JOINTS },
    { label: "Joint frames", flag: DRAW_JOINT_EXTRAS },
    { label: "AABBs", flag: DRAW_BOUNDS },
    { label: "Mass axes", flag: DRAW_MASS },
    { label: "Islands", flag: DRAW_ISLANDS },
  ];
  for (const f of flagDefs) {
    dbg.body.appendChild(
      createCheckbox(f.label, false, (on) => {
        if (on) state.debugFlags |= f.flag;
        else state.debugFlags &= ~f.flag;
      }),
    );
  }

  // --- Keyboard / mouse legend (C Controls window, adapted) ---
  const keys = createCollapsingSection("Keyboard", false);
  controls.appendChild(keys.root);
  keys.body.innerHTML = `
    <table class="key-legend">
      <tr><td>P / Space</td><td>Pause / resume</td></tr>
      <tr><td>O / S</td><td>Single step (Shift: 5)</td></tr>
      <tr><td>R</td><td>Restart sample</td></tr>
      <tr><td>Drag</td><td>Grab body</td></tr>
      ${
        enableSpawnDelete
          ? "<tr><td>Shift-click</td><td>Spawn shape</td></tr><tr><td>Ctrl-click</td><td>Delete body</td></tr>"
          : ""
      }
      <tr><td>Orbit</td><td>Left-drag empty / right-drag</td></tr>
      <tr><td>Scroll</td><td>Zoom</td></tr>
    </table>
  `;

  const debugOverlay = new DebugDrawOverlay(demo);

  // --- Pointer interaction ---
  let dragging = false;
  let suppressClick = false;

  const onPointerDown = (e: PointerEvent) => {
    if (e.button !== 0 && e.pointerType === "mouse") return;
    const { origin, translation } = pickRay(demo, canvas, e.clientX, e.clientY);

    if (e.shiftKey && enableSpawnDelete) {
      wasm.sim_spawn_random(origin.x, origin.y, origin.z, translation.x, translation.y, translation.z);
      e.preventDefault();
      e.stopPropagation();
      suppressClick = true;
      return;
    }
    if (e.ctrlKey && enableSpawnDelete) {
      wasm.sim_delete_at_ray(origin.x, origin.y, origin.z, translation.x, translation.y, translation.z);
      e.preventDefault();
      e.stopPropagation();
      suppressClick = true;
      return;
    }

    const grab = wasm.sim_mouse_down(
      origin.x,
      origin.y,
      origin.z,
      translation.x,
      translation.y,
      translation.z,
    );
    if (grab[0]! > 0.5) {
      dragging = true;
      demo.controls.enabled = false;
      canvas.setPointerCapture(e.pointerId);
      _grabPoint.set(grab[1]!, grab[2]!, grab[3]!);
      e.preventDefault();
      e.stopPropagation();
    }
  };

  const onPointerMove = (e: PointerEvent) => {
    if (!dragging || !wasm.sim_mouse_active()) return;
    const p = dragPointOnCameraPlane(demo, canvas, e.clientX, e.clientY, _grabPoint);
    _grabPoint.copy(p);
    wasm.sim_mouse_move(p.x, p.y, p.z);
    e.preventDefault();
  };

  const onPointerUp = (e: PointerEvent) => {
    if (dragging) {
      wasm.sim_mouse_up();
      dragging = false;
      demo.controls.enabled = true;
      try {
        canvas.releasePointerCapture(e.pointerId);
      } catch {
        /* already released */
      }
    }
  };

  canvas.addEventListener("pointerdown", onPointerDown);
  canvas.addEventListener("pointermove", onPointerMove);
  canvas.addEventListener("pointerup", onPointerUp);
  canvas.addEventListener("pointercancel", onPointerUp);
  const onContext = (e: Event) => {
    if (suppressClick) {
      e.preventDefault();
      suppressClick = false;
    }
  };
  canvas.addEventListener("contextmenu", onContext);

  // --- Hotkeys (C: P pause, O step, R restart; Space/S kept as aliases) ---
  const onKey = (e: KeyboardEvent) => {
    const tag = (e.target as HTMLElement)?.tagName;
    if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
    if (e.code === "Space" || e.code === "KeyP") {
      e.preventDefault();
      state.setPaused(!state.paused);
    } else if (
      (e.code === "KeyS" || e.code === "KeyO") &&
      !e.ctrlKey &&
      !e.metaKey
    ) {
      e.preventDefault();
      state.setPaused(true);
      state.requestStep(e.shiftKey ? 5 : 1);
    } else if (e.code === "KeyR" && !e.ctrlKey && !e.metaKey) {
      e.preventDefault();
      state.restart();
    }
  };
  window.addEventListener("keydown", onKey);

  let frame = 0;
  let stepEma = 0;
  let lastFrameMs = 0;
  let lastFrameT = performance.now();

  /** Call once per animation frame from the demo loop. Returns whether a step ran. */
  function tickFrame(): boolean {
    const now = performance.now();
    lastFrameMs = now - lastFrameT;
    lastFrameT = now;

    let stepped = false;
    const scale = state.timeScale;
    const shouldStep =
      (!state.paused && scale > 0) || (state.paused && state.stepsPending > 0);

    if (shouldStep) {
      const t0 = performance.now();
      // C Sample::Step: timeStep = 1/hertz (then optionally scaled for browser demos).
      const hz = state.hertz > 0 ? state.hertz : 1 / baseDt;
      const dt = state.paused ? 1 / hz : (1 / hz) * scale;
      wasm.sim_step(dt, state.subSteps);
      const ms = performance.now() - t0;
      stepEma = stepEma === 0 ? ms : stepEma * 0.9 + ms * 0.1;
      state.stepMsAvg = stepEma;
      state.stepCount =
        typeof wasm.sim_step_count === "function"
          ? wasm.sim_step_count()
          : state.stepCount + 1;
      if (state.paused && state.stepsPending > 0) state.stepsPending -= 1;
      stepped = true;
    }

    if (state.debugFlags !== 0) {
      debugOverlay.update(wasm.sim_debug_draw(state.debugFlags));
    } else {
      debugOverlay.clear();
    }

    frame += 1;
    if (frame % 2 === 0) {
      frameMsEl.textContent = `${lastFrameMs.toFixed(1)} ms`;
      stepCountEl.textContent = `step ${state.stepCount}`;
      const cam = cameraReadout(demo);
      camPivotEl.textContent = `pivot m (${cam.px.toFixed(1)}, ${cam.py.toFixed(1)}, ${cam.pz.toFixed(1)})`;
      camYawEl.textContent = `yaw/pitch (${cam.yaw.toFixed(1)}, ${cam.pitch.toFixed(1)})`;
      camRadiusEl.textContent = `radius m ${cam.radius.toFixed(1)}`;
    }

    return stepped;
  }

  const withTick = state as SimControllerWithTick;
  withTick.tickFrame = tickFrame;

  state.dispose = () => {
    window.removeEventListener("keydown", onKey);
    canvas.removeEventListener("pointerdown", onPointerDown);
    canvas.removeEventListener("pointermove", onPointerMove);
    canvas.removeEventListener("pointerup", onPointerUp);
    canvas.removeEventListener("pointercancel", onPointerUp);
    canvas.removeEventListener("contextmenu", onContext);
    wasm.sim_mouse_up();
    if (wasm.sim_is_recording?.()) wasm.sim_stop_recording?.();
    debugOverlay.dispose();
    paramDispose();
    demo.controls.enabled = true;
  };

  return withTick;
}

function escapeHtml(s: string): string {
  return s
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

export type SimControllerWithTick = SimController & { tickFrame: () => boolean };
