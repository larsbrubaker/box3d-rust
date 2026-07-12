// Shared simulation interaction layer: pause/step/restart, pick-drag, spawn/delete,
// stats overlay, debug-draw toggles, and declarative parameter panels.

import * as THREE from "three";
import {
  createButton,
  createButtonGroup,
  createCheckbox,
  createReadout,
  createSlider,
  updateReadout,
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
  debugFlags: number;
  /** Rolling average step time in ms. */
  stepMsAvg: number;
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
  /** Optional declarative parameter panel (applied before sim controls). */
  params?: ParamDef[];
  onParamsChange?: (values: ParamValues, changedKey: string) => void;
  /** Hide spawn/delete hints when false. Default true. */
  enableSpawnDelete?: boolean;
  /** Base dt passed to wasm (default 1/60). */
  baseDt?: number;
};

const _ndc = new THREE.Vector2();
const _raycaster = new THREE.Raycaster();
const _hitPlane = new THREE.Plane();
const _planeHit = new THREE.Vector3();
const _camDir = new THREE.Vector3();
const _grabPoint = new THREE.Vector3();

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
  // Fallback: project along ray at the original grab depth
  return _raycaster.ray.origin.clone().add(
    _raycaster.ray.direction.clone().multiplyScalar(planePoint.distanceTo(_raycaster.ray.origin)),
  );
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
 * Attach pause/step/restart, hotkeys, time scale, pick-drag, spawn/delete,
 * stats readout, and debug-draw toggles to a dynamics demo.
 */
export function attachInteraction(opts: AttachInteractionOpts): SimControllerWithTick {
  const {
    wasm,
    demo,
    canvas,
    controls,
    onRestart,
    params: paramDefs = [],
    onParamsChange,
    enableSpawnDelete = true,
    baseDt = 1 / 60,
  } = opts;

  const state: SimController = {
    paused: false,
    stepsPending: 0,
    timeScale: 1,
    subSteps: 4,
    debugFlags: 0,
    stepMsAvg: 0,
    params: {},
    setPaused(v) {
      state.paused = v;
      pauseBtn.classList.toggle("active", v);
      pauseBtn.textContent = v ? "Resume (Space)" : "Pause (Space)";
    },
    requestStep(n = 1) {
      state.stepsPending += n;
    },
    restart() {
      wasm.sim_mouse_up();
      onRestart();
    },
    dispose() {
      /* filled below */
    },
  };

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

  // --- Sim controls ---
  const simSection = document.createElement("div");
  simSection.className = "control-section-title";
  simSection.textContent = "Simulation";
  controls.appendChild(simSection);

  const pauseBtn = createButton("Pause (Space)", () => state.setPaused(!state.paused));
  const stepBtn = createButton("Step (S)", () => {
    state.setPaused(true);
    state.requestStep(1);
  });
  const restartBtn = createButton("Restart (R)", () => state.restart());
  const row = document.createElement("div");
  row.className = "control-row";
  row.style.marginBottom = "12px";
  row.append(pauseBtn, stepBtn, restartBtn);
  controls.appendChild(row);

  controls.appendChild(
    createSlider("Time scale", 0, 2, 1, 0.05, (v) => {
      state.timeScale = v;
    }),
  );

  // --- Debug draw toggles ---
  const dbgTitle = document.createElement("div");
  dbgTitle.className = "control-section-title";
  dbgTitle.textContent = "Debug draw";
  controls.appendChild(dbgTitle);

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
    controls.appendChild(
      createCheckbox(f.label, false, (on) => {
        if (on) state.debugFlags |= f.flag;
        else state.debugFlags &= ~f.flag;
      }),
    );
  }

  const hint = document.createElement("div");
  hint.className = "info-box";
  hint.innerHTML = enableSpawnDelete
    ? "Drag body to grab · Shift-click spawn · Ctrl-click delete · Space/S/R"
    : "Drag body to grab · Space pause · S step · R restart";
  controls.appendChild(hint);

  const readout = createReadout();
  controls.appendChild(readout);

  const statsHud = document.createElement("div");
  statsHud.className = "stats-overlay";
  canvas.parentElement?.appendChild(statsHud);

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
  // Prevent context menu on ctrl-click
  const onContext = (e: Event) => {
    if (suppressClick) {
      e.preventDefault();
      suppressClick = false;
    }
  };
  canvas.addEventListener("contextmenu", onContext);

  // --- Hotkeys ---
  const onKey = (e: KeyboardEvent) => {
    const tag = (e.target as HTMLElement)?.tagName;
    if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
    if (e.code === "Space") {
      e.preventDefault();
      state.setPaused(!state.paused);
    } else if (e.code === "KeyS" && !e.ctrlKey && !e.metaKey) {
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

  /** Call once per animation frame from the demo loop. Returns whether a step ran. */
  function tickFrame(): boolean {
    let stepped = false;
    const scale = state.timeScale;
    const shouldStep =
      (!state.paused && scale > 0) || (state.paused && state.stepsPending > 0);

    if (shouldStep) {
      const t0 = performance.now();
      const dt = state.paused ? baseDt : baseDt * scale;
      wasm.sim_step(dt, state.subSteps);
      const ms = performance.now() - t0;
      stepEma = stepEma === 0 ? ms : stepEma * 0.9 + ms * 0.1;
      state.stepMsAvg = stepEma;
      if (state.paused && state.stepsPending > 0) state.stepsPending -= 1;
      stepped = true;
    }

    if (state.debugFlags !== 0) {
      debugOverlay.update(wasm.sim_debug_draw(state.debugFlags));
    } else {
      debugOverlay.clear();
    }

    frame += 1;
    if (frame % 10 === 0) {
      const c = wasm.sim_counters();
      const entries = [
        { label: "step", value: `${state.stepMsAvg.toFixed(2)} ms` },
        { label: "bodies", value: String(c[0] | 0) },
        { label: "shapes", value: String(c[1] | 0) },
        { label: "contacts", value: String(c[2] | 0) },
        { label: "joints", value: String(c[3] | 0) },
        { label: "awake", value: String(c[5] | 0) },
        { label: "sleeping", value: String(c[6] | 0) },
        { label: "paused", value: state.paused ? "yes" : "no" },
      ];
      updateReadout(readout, entries);
      statsHud.textContent =
        `${state.stepMsAvg.toFixed(1)} ms  ·  ` +
        `${c[0] | 0} bodies  ·  ${c[5] | 0} awake / ${c[6] | 0} sleep` +
        (state.paused ? "  ·  PAUSED" : "");
    }

    return stepped;
  }

  // Expose tick on the controller
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
    debugOverlay.dispose();
    statsHud.remove();
    paramDispose();
    demo.controls.enabled = true;
  };

  return withTick;
}

export type SimControllerWithTick = SimController & { tickFrame: () => boolean };
