// Shared simulation interaction layer: Samples App Info panel (C DrawInfoPanel),
// pause/step/restart, pick-drag, spawn/delete, Solver/Recording, debug-draw.

import * as THREE from "three";
import { LineSegments2 } from "three/addons/lines/LineSegments2.js";
import { LineSegmentsGeometry } from "three/addons/lines/LineSegmentsGeometry.js";
import type { LineMaterial } from "three/addons/lines/LineMaterial.js";
import {
  createButton,
  createButtonGroup,
  createCheckbox,
  createCollapsingSection,
  createSlider,
  createTextInput,
} from "./controls.ts";
import type { DemoScene } from "./three-scene.ts";
import { makeFatLineMaterial, updateFatLineResolution } from "./render/lines.ts";
import { demoBus, emitInitialState, viewFlags } from "./bus.ts";
import { VIEW_BITS, OVERLAY_MASK, TEXT_MASK } from "./view-flags.ts";

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
  /** Set the sim's debug-draw flag mask (16-bit; see VIEW_BITS). Added by the wasm agent. */
  sim_set_debug_flags?(mask: number): void;
  /** Set the sim's debug-draw joint / force scales. Added by the wasm agent. */
  sim_set_draw_scales?(joint: number, force: number): void;
  /** JSON array of `{x,y,z,color,text}` debug labels for the current view mask.
   *  Optional: only some demos emit text (body names / mass / sleep / contact
   *  features). Adapters forward the demo's own `*_debug_text` export here. */
  sim_debug_text?: () => string;
};

// The 16-bit view-flag mask (bit order, defaults, labels) lives in the shared
// leaf view-flags.ts. VIEW_BITS/OVERLAY_MASK/TEXT_MASK are imported above; this
// module keys the panel + menu into that one table via the `view.flag` bus.

/**
 * Build an {@link InteractWasm} adapter for a per-demo export family by prefix.
 *
 * Every category's wasm module exposes the same method set under its own prefix
 * (`bodies_step`, `shapes_step`, `joint_step`, …). Hand-writing that mapping in
 * each page duplicated ~15 lines seven times and repeatedly forgot to forward
 * the *global* debug-flag / draw-scale setters (they are not prefixed — one
 * `sim_set_debug_flags` drives whichever demo is mounted). This factory does the
 * mapping once via bracket access, ALWAYS forwards the two global setters, and
 * pulls in the optional per-prefix methods (`*_step_count`, `*_set_enable_*`,
 * `*_debug_text`, recording) only when the export actually exists — so a demo
 * that lacks `set_enable_sleep` simply omits it, exactly like the old hand
 * adapters. `overrides` wins over the generated mapping (used by the World page
 * to route between the Far Pyramid and generic `world_far_*` families, and by
 * Benchmark whose poses export is the odd `bench_body_poses`).
 */
export function makeInteractAdapter(
  wasm: object,
  prefix: string,
  overrides: Partial<InteractWasm> = {},
): InteractWasm {
  // Dynamic export bag: bracket access is intentional here (one factory over
  // every prefix), so an `any`-valued record is the right typing seam.
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const bag = wasm as Record<string, (...args: any[]) => any> & {
    sim_set_debug_flags?: (mask: number) => void;
    sim_set_draw_scales?: (joint: number, force: number) => void;
  };
  const name = (suffix: string) => `${prefix}_${suffix}`;
  const has = (suffix: string) => typeof bag[name(suffix)] === "function";
  const call = (suffix: string) => bag[name(suffix)];

  const adapter: InteractWasm = {
    sim_step: (dt, ss) => call("step")(dt, ss),
    sim_body_poses: () => call("poses")(),
    sim_mouse_down: (ox, oy, oz, tx, ty, tz) => call("mouse_down")(ox, oy, oz, tx, ty, tz),
    sim_mouse_move: (px, py, pz) => call("mouse_move")(px, py, pz),
    sim_mouse_up: () => call("mouse_up")(),
    sim_mouse_active: () => call("mouse_active")(),
    sim_spawn_random: (ox, oy, oz, tx, ty, tz) => call("spawn_random")(ox, oy, oz, tx, ty, tz),
    sim_delete_at_ray: (ox, oy, oz, tx, ty, tz) => call("delete_at_ray")(ox, oy, oz, tx, ty, tz),
    sim_counters: () => call("counters")(),
    sim_debug_draw: (flags) => call("debug_draw")(flags),
  };

  // Global (non-prefixed) setters — one shared pair per wasm module. Always
  // forward them when present so the View menu / panel drive this demo's overlay.
  if (typeof bag.sim_set_debug_flags === "function") {
    adapter.sim_set_debug_flags = (m) => bag.sim_set_debug_flags!(m);
  }
  if (typeof bag.sim_set_draw_scales === "function") {
    adapter.sim_set_draw_scales = (j, f) => bag.sim_set_draw_scales!(j, f);
  }

  // Optional per-prefix methods: included only when the demo exports them.
  if (has("step_count")) adapter.sim_step_count = () => call("step_count")();
  if (has("set_enable_sleep")) adapter.sim_set_enable_sleep = (v) => call("set_enable_sleep")(v);
  if (has("set_enable_warm_starting"))
    adapter.sim_set_enable_warm_starting = (v) => call("set_enable_warm_starting")(v);
  if (has("set_enable_continuous"))
    adapter.sim_set_enable_continuous = (v) => call("set_enable_continuous")(v);
  if (has("set_recycle_distance"))
    adapter.sim_set_recycle_distance = (m) => call("set_recycle_distance")(m);
  if (has("start_recording")) adapter.sim_start_recording = () => call("start_recording")();
  if (has("stop_recording")) adapter.sim_stop_recording = () => call("stop_recording")();
  if (has("is_recording")) adapter.sim_is_recording = () => call("is_recording")();
  if (has("record_start_step")) adapter.sim_record_start_step = () => call("record_start_step")();
  if (has("debug_text")) adapter.sim_debug_text = () => call("debug_text")();

  return { ...adapter, ...overrides };
}

/** A single visibility predicate: the param shows only while `values[key] === equals`. */
export type ParamVisibility = { key: string; equals: boolean | number | string };

/** Fields shared by every param kind. */
type ParamCommon = {
  key: string;
  label: string;
  /** Restart the scene when this value changes (default true). */
  restart?: boolean;
  /**
   * Render this param inside a titled section. Consecutive defs with the same
   * `group` share one section; the section auto-hides when all its members are
   * hidden by `visibleWhen`.
   */
  group?: string;
  /**
   * Show this param only while the predicate(s) hold. Re-evaluated whenever any
   * param changes. An array requires every predicate to pass (logical AND).
   * Hidden params keep their current values.
   */
  visibleWhen?: ParamVisibility | ParamVisibility[];
};

export type ParamDef =
  | (ParamCommon & {
      type: "slider";
      min: number;
      max: number;
      step: number;
      default: number;
    })
  | (ParamCommon & {
      type: "checkbox";
      default: boolean;
    })
  | (ParamCommon & {
      type: "select";
      options: { label: string; value: string }[];
      default: string;
    });

export type ParamValues = Record<string, number | boolean | string>;

export type SimController = {
  paused: boolean;
  /** Remaining single-steps to run while paused. */
  stepsPending: number;
  timeScale: number;
  subSteps: number;
  /** Solver frequency in Hz (C sample Hertz). dt = 1/hertz. */
  hertz: number;
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
  worldOrigin?: [number, number, number];
};

const _ndc = new THREE.Vector2();
const _raycaster = new THREE.Raycaster();
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

// Read yaw/pitch/radius/pivot straight from the camera controller (C samples
// read these off the Camera; no need to reverse-engineer from position).
function cameraReadout(demo: DemoScene): {
  px: number;
  py: number;
  pz: number;
  yaw: number;
  pitch: number;
  radius: number;
} {
  const c = demo.controls;
  const t = c.pivot;
  return { px: t.x, py: t.y, pz: t.z, yaw: c.yawDeg, pitch: c.pitchDeg, radius: c.radius };
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

function paramVisible(vw: ParamVisibility | ParamVisibility[] | undefined, values: ParamValues): boolean {
  if (!vw) return true;
  const conds = Array.isArray(vw) ? vw : [vw];
  return conds.every((c) => values[c.key] === c.equals);
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

  // Track each control's element + predicate so visibility can be re-evaluated on
  // any change. Group sections auto-hide when all their members are hidden.
  type Group = { name: string; container: HTMLElement; members: ParamDef[] };
  const entries: { el: HTMLElement; def: ParamDef }[] = [];
  const groups: Group[] = [];
  let currentGroup: Group | null = null;

  const containerFor = (def: ParamDef): HTMLElement => {
    if (def.group == null) {
      currentGroup = null;
      return section;
    }
    if (currentGroup && currentGroup.name === def.group) {
      currentGroup.members.push(def);
      return currentGroup.container;
    }
    const container = document.createElement("div");
    container.className = "param-group";
    container.style.marginTop = "0.5rem";
    const gt = document.createElement("div");
    gt.className = "control-section-title";
    gt.textContent = def.group;
    container.appendChild(gt);
    section.appendChild(container);
    currentGroup = { name: def.group, container, members: [def] };
    groups.push(currentGroup);
    return container;
  };

  const buildControl = (def: ParamDef, restart: boolean): HTMLElement => {
    if (def.type === "slider") {
      values[def.key] = def.default;
      return createSlider(def.label, def.min, def.max, def.default, def.step, (v) => {
        values[def.key] = def.step >= 1 ? Math.round(v) : v;
        onChange(values, def.key, restart);
        applyVisibility();
      });
    }
    if (def.type === "checkbox") {
      values[def.key] = def.default;
      return createCheckbox(def.label, def.default, (v) => {
        values[def.key] = v;
        onChange(values, def.key, restart);
        applyVisibility();
      });
    }
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
        applyVisibility();
      }),
    );
    return group;
  };

  function applyVisibility() {
    for (const e of entries) {
      e.el.style.display = paramVisible(e.def.visibleWhen, values) ? "" : "none";
    }
    for (const g of groups) {
      const any = g.members.some((m) => paramVisible(m.visibleWhen, values));
      g.container.style.display = any ? "" : "none";
    }
  }

  for (const def of defs) {
    const container = containerFor(def);
    const el = buildControl(def, def.restart !== false);
    container.appendChild(el);
    entries.push({ el, def });
  }

  applyVisibility();
  parent.appendChild(section);
  return {
    values,
    dispose: () => section.remove(),
  };
}

/**
 * Three.js line/point overlay fed by `sim_debug_draw`. Segments render as fat
 * `LineSegments2` at C's 1.5px default (`draw.h`:16-20) so joint frames / contact
 * overlays read with real thickness; plain `THREE.LineSegments` ignores linewidth
 * on most platforms. Per-vertex colors come straight from the engine's packed
 * segment colors (`LineMaterial` vertexColors). Points stay as `THREE.Points`.
 */
export class DebugDrawOverlay {
  private lines: LineSegments2 | null = null;
  private points: THREE.Points | null = null;
  private readonly group: THREE.Group;
  // One shared fat-line material (a ShaderMaterial — compile it once, not per
  // frame). Geometry is rebuilt each frame; its `resolution` uniform is refreshed
  // from the renderer's drawing-buffer size so px thickness stays correct across
  // canvas resizes without a separate observer.
  private readonly lineMaterial: LineMaterial;
  private readonly _res = new THREE.Vector2();

  constructor(private readonly demo: DemoScene) {
    this.group = new THREE.Group();
    this.demo.dynamic.add(this.group);
    this.lineMaterial = makeFatLineMaterial({
      thickness: 1.5, // C debug default (draw.h DrawColorLine / lineWidth)
      vertexColors: true,
      opacity: 0.9,
    });
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
      // Keep the material's pixel->clip resolution current (covers resizes too).
      const size = this.demo.renderer.getDrawingBufferSize(this._res);
      updateFatLineResolution(this.lineMaterial, size.x, size.y);
      const geo = new LineSegmentsGeometry();
      geo.setPositions(positions);
      geo.setColors(colors);
      this.lines = new LineSegments2(geo, this.lineMaterial);
      // Fat lines cover the whole cloud; skip frustum culling on the tight
      // per-segment bounds the addon would otherwise compute.
      this.lines.frustumCulled = false;
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
    // Dispose per-frame geometry (and the Points material), but keep the shared
    // fat-line material alive across frames — it is disposed in dispose().
    if (this.lines) {
      this.group.remove(this.lines);
      this.lines.geometry.dispose();
      this.lines = null;
    }
    if (this.points) {
      this.group.remove(this.points);
      this.points.geometry.dispose();
      (this.points.material as THREE.Material).dispose();
      this.points = null;
    }
  }

  dispose() {
    this.clear();
    this.lineMaterial.dispose();
    this.demo.dynamic.remove(this.group);
  }
}

function floatBitsToRgb(bitsAsF32: number): [number, number, number] {
  const u = new Float32Array([bitsAsF32]);
  const view = new Uint32Array(u.buffer);
  const hex = view[0]! & 0x00ffffff;
  return [((hex >> 16) & 0xff) / 255, ((hex >> 8) & 0xff) / 255, (hex & 0xff) / 255];
}

/** One debug label: a text string anchored at a world point, with a color. */
type DebugLabel = { x: number; y: number; z: number; color: string; text: string };

/** Parse the `sim_debug_text` JSON channel into labels (tolerant of an empty or
 *  malformed payload — the overlay simply clears in that case). Color accepts a
 *  packed 0xRRGGBB number or a CSS string; anything else falls back to white. */
function parseDebugText(json: string | undefined): DebugLabel[] {
  if (!json) return [];
  let raw: unknown;
  try {
    raw = JSON.parse(json);
  } catch {
    return [];
  }
  if (!Array.isArray(raw)) return [];
  const labels: DebugLabel[] = [];
  for (const item of raw) {
    if (!item || typeof item !== "object") continue;
    const o = item as Record<string, unknown>;
    if (typeof o.text !== "string" || o.text.length === 0) continue;
    labels.push({
      x: Number(o.x) || 0,
      y: Number(o.y) || 0,
      z: Number(o.z) || 0,
      color: normDebugColor(o.color),
      text: o.text,
    });
  }
  return labels;
}

function normDebugColor(c: unknown): string {
  if (typeof c === "number") return `#${(c & 0xffffff).toString(16).padStart(6, "0")}`;
  if (typeof c === "string" && c.length > 0) return c;
  return "#ffffff";
}

/**
 * Billboarded text labels for the `sim_debug_text` channel (body names, mass,
 * sleep timers, contact feature ids). Each unique `color|text` is rasterized to
 * a canvas texture once and cached; sprites are pooled and reused across frames
 * so a steady label set costs no per-frame allocation. Sprites always face the
 * camera (THREE.Sprite), matching the C DrawString HUD behavior in 3D.
 */
class TextLabelOverlay {
  private readonly group: THREE.Group;
  private readonly textureCache = new Map<string, THREE.Texture>();
  private readonly pool: THREE.Sprite[] = [];

  constructor(private readonly demo: DemoScene) {
    this.group = new THREE.Group();
    this.demo.dynamic.add(this.group);
  }

  private textureFor(text: string, color: string): THREE.Texture {
    const key = `${color} ${text}`;
    const cached = this.textureCache.get(key);
    if (cached) return cached;
    const canvas = document.createElement("canvas");
    const ctx = canvas.getContext("2d")!;
    const font = "600 32px system-ui, -apple-system, sans-serif";
    ctx.font = font;
    const pad = 8;
    const w = Math.ceil(ctx.measureText(text).width) + pad * 2;
    const h = 32 + pad * 2;
    canvas.width = w;
    canvas.height = h;
    // measureText resets after the canvas resize, so re-apply the font.
    ctx.font = font;
    ctx.textBaseline = "middle";
    ctx.fillStyle = "rgba(0,0,0,0.5)"; // legibility plate behind the glyphs
    ctx.fillRect(0, 0, w, h);
    ctx.fillStyle = color;
    ctx.fillText(text, pad, h / 2);
    const tex = new THREE.CanvasTexture(canvas);
    tex.minFilter = THREE.LinearFilter;
    tex.magFilter = THREE.LinearFilter;
    this.textureCache.set(key, tex);
    return tex;
  }

  update(labels: DebugLabel[]) {
    while (this.pool.length < labels.length) {
      const sprite = new THREE.Sprite(
        new THREE.SpriteMaterial({ transparent: true, depthTest: true }),
      );
      this.pool.push(sprite);
      this.group.add(sprite);
    }
    for (let i = 0; i < this.pool.length; i++) {
      const sprite = this.pool[i]!;
      const l = labels[i];
      if (!l) {
        sprite.visible = false;
        continue;
      }
      const tex = this.textureFor(l.text, l.color);
      const mat = sprite.material as THREE.SpriteMaterial;
      mat.map = tex;
      mat.needsUpdate = true;
      const img = tex.image as HTMLCanvasElement;
      const worldH = 0.5; // ~0.5 m tall labels; scale width to preserve aspect
      sprite.scale.set(worldH * (img.width / img.height), worldH, 1);
      sprite.position.set(l.x, l.y, l.z);
      sprite.visible = true;
    }
  }

  clear() {
    for (const s of this.pool) s.visible = false;
  }

  dispose() {
    for (const s of this.pool) {
      this.group.remove(s);
      (s.material as THREE.SpriteMaterial).dispose();
    }
    this.pool.length = 0;
    for (const tex of this.textureCache.values()) tex.dispose();
    this.textureCache.clear();
    this.demo.dynamic.remove(this.group);
  }
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

  // worldOrigin / sampleName are per-scene on multi-scene pages (e.g. World's Far
  // Pyramid sits at [1e7,0,0] with its own name), so keep them mutable and expose
  // setters on the returned controller rather than baking them in once.
  let worldOrigin: [number, number, number] = opts.worldOrigin ?? [0, 0, 0];

  controls.classList.add("samples-info-panel");

  const state: SimController = {
    paused: false,
    stepsPending: 0,
    timeScale: 1,
    subSteps: 4,
    hertz: 60,
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
      // onParamsChange may clamp values in place; copy after it so state.params
      // (and any restart that reads it) sees the corrected values.
      onParamsChange?.(values, key);
      state.params = { ...values };
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

  // --- Debug-draw state: one 16-bit mask (VIEW_BITS order) is the single source ---
  // The View menu (main.ts) and the checkbox panel below are two UIs over the
  // same flags. Both route through the `view.flag` bus event; the subscriber
  // (further down) folds each change into `viewMask` and pushes it to the sim via
  // the guarded sim_set_debug_flags. tickFrame then fetches the overlay/text
  // channels off that one mask. There is no parallel client-side flag word.
  let viewMask = 0;
  const drawScales = { joint: 1, force: 1 };
  const pushDebugFlags = () => {
    wasm.sim_set_debug_flags?.(viewMask);
  };
  const pushDrawScales = () => {
    wasm.sim_set_draw_scales?.(drawScales.joint, drawScales.force);
  };
  const setViewBit = (key: string, on: boolean) => {
    const bit = VIEW_BITS[key];
    if (bit === undefined) return;
    if (on) viewMask |= bit;
    else viewMask &= ~bit;
  };

  // --- Debug draw toggles ---
  const dbg = createCollapsingSection("Debug draw", false);
  controls.appendChild(dbg.root);

  // A curated subset of the View-menu flags, surfaced as native checkboxes. Each
  // toggle initializes from the shared `viewFlags` state and emits `view.flag`
  // (the same event the menu uses) so the menu tick, the mask, and any other
  // subscriber all stay in lockstep. The `view.flag` subscriber below writes the
  // checkbox back when the change originates in the menu.
  const panelFlagDefs: { label: string; viewKey: string }[] = [
    { label: "Contacts", viewKey: "contacts" },
    { label: "Contact normals", viewKey: "contactNormals" },
    { label: "Contact forces", viewKey: "contactForces" },
    { label: "Joints", viewKey: "joints" },
    { label: "Joint frames", viewKey: "jointExtras" },
    { label: "AABBs", viewKey: "bounds" },
    { label: "Mass axes", viewKey: "mass" },
    { label: "Islands", viewKey: "islands" },
  ];
  const panelCheckboxes = new Map<string, HTMLInputElement>();
  for (const f of panelFlagDefs) {
    const el = createCheckbox(f.label, viewFlags[f.viewKey] ?? false, (on) => {
      viewFlags[f.viewKey] = on;
      demoBus.emit("view.flag", { name: f.viewKey, value: on });
    });
    const input = el.querySelector("input") as HTMLInputElement | null;
    if (input) panelCheckboxes.set(f.viewKey, input);
    dbg.body.appendChild(el);
  }

  // --- Keyboard / mouse legend (C Controls window, adapted) ---
  const keys = createCollapsingSection("Keyboard", false);
  controls.appendChild(keys.root);
  keys.body.innerHTML = `
    <table class="key-legend">
      <tr><td>P</td><td>Pause / resume</td></tr>
      <tr><td>O (Shift+O)</td><td>Single / 5× step</td></tr>
      <tr><td>R</td><td>Restart sample</td></tr>
      <tr><td>[ / ]</td><td>Prev / next sample</td></tr>
      <tr><td>F</td><td>Frame selection</td></tr>
      <tr><td>Tab</td><td>Hide / show UI</td></tr>
      <tr><td>Left-click</td><td>Select body</td></tr>
      <tr><td>Ctrl + click</td><td>Grab body</td></tr>
      ${enableSpawnDelete ? "<tr><td>Shift + click</td><td>Spawn body</td></tr>" : ""}
      <tr><td>Alt + drag</td><td>Orbit / pan / zoom</td></tr>
      <tr><td>Right-drag + WASD</td><td>Fly camera</td></tr>
      <tr><td>Scroll</td><td>Zoom</td></tr>
    </table>
  `;

  const debugOverlay = new DebugDrawOverlay(demo);
  const textOverlay = new TextLabelOverlay(demo);

  // --- Pointer interaction (C Sample::MouseDown/Move, sample.cpp:1136-1289) ---
  //   plain left-click : select the body under the cursor (store for F-frame)
  //   Ctrl + left-drag : grab a dynamic body with the motor-joint mouse spring
  //   Shift + left     : spawn the bullet sphere along the pick ray
  // Camera gestures (Alt+drag orbit/pan/zoom, right-drag fly) live in the camera
  // controller, so nothing here disables it. Grab tracks the drag point along the
  // pick ray at the initial hit fraction, exactly like C MouseMove:1288.
  let dragging = false;
  let grabFraction = 0;
  let suppressClick = false;
  /** Current selection as a world-space box, used by F-frame. null = nothing selected. */
  let selection: THREE.Box3 | null = null;

  // Fraction of the pick ray at which `hit` lies: hit = origin + f·translation.
  const rayFraction = (
    origin: THREE.Vector3,
    translation: THREE.Vector3,
    hit: THREE.Vector3,
  ): number => {
    const tl2 = translation.dot(translation);
    return tl2 > 0 ? _grabPoint.copy(hit).sub(origin).dot(translation) / tl2 : 0;
  };

  const selectAt = (clientX: number, clientY: number) => {
    screenToNdc(canvas, clientX, clientY);
    _raycaster.setFromCamera(_ndc, demo.camera);
    const hits = _raycaster.intersectObjects(demo.content.children, true);
    if (hits.length === 0) {
      selection = null; // C ClearSelection on empty space
      return;
    }
    const hit = hits[0]!;
    const obj = hit.object;
    // InstancedMesh AABBs cover the whole cloud, so frame a unit box at the hit
    // point instead; single meshes use their own world AABB.
    if ((obj as THREE.InstancedMesh).isInstancedMesh) {
      selection = new THREE.Box3().setFromCenterAndSize(hit.point, new THREE.Vector3(1, 1, 1));
    } else {
      const box = new THREE.Box3().setFromObject(obj);
      selection = box.isEmpty() || !isFinite(box.min.x)
        ? new THREE.Box3().setFromCenterAndSize(hit.point, new THREE.Vector3(1, 1, 1))
        : box;
    }
  };

  const onPointerDown = (e: PointerEvent) => {
    if (e.button !== 0 && e.pointerType === "mouse") return;

    // Shift+click spawns the bullet body (C sample.cpp:1211). Shift+Ctrl (cylinder)
    // and Shift+Alt (human) are batch-3: their wasm exports don't exist yet.
    if (e.shiftKey && enableSpawnDelete) {
      const { origin, translation } = pickRay(demo, canvas, e.clientX, e.clientY);
      wasm.sim_spawn_random(origin.x, origin.y, origin.z, translation.x, translation.y, translation.z);
      e.preventDefault();
      e.stopPropagation();
      suppressClick = true;
      return;
    }

    // Ctrl+click grabs a dynamic body (C sample.cpp:1161).
    if (e.ctrlKey) {
      const { origin, translation } = pickRay(demo, canvas, e.clientX, e.clientY);
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
        const hit = new THREE.Vector3(grab[1]!, grab[2]!, grab[3]!);
        grabFraction = rayFraction(origin, translation, hit);
        try {
          canvas.setPointerCapture(e.pointerId);
        } catch {
          /* pointer may not be capturable (e.g. synthetic events) */
        }
        e.preventDefault();
        e.stopPropagation();
      }
      return;
    }

    // Plain left-click selects the body under the cursor (C sample.cpp:1143).
    if (!e.altKey && !e.metaKey) {
      selectAt(e.clientX, e.clientY);
    }
  };

  const onPointerMove = (e: PointerEvent) => {
    if (!dragging || !wasm.sim_mouse_active()) return;
    // Track the drag point along the current pick ray at the initial hit
    // fraction (C MouseMove:1288: m_mousePoint = origin + fraction·translation).
    const { origin, translation } = pickRay(demo, canvas, e.clientX, e.clientY);
    origin.addScaledVector(translation, grabFraction);
    wasm.sim_mouse_move(origin.x, origin.y, origin.z);
    e.preventDefault();
  };

  const onPointerUp = (e: PointerEvent) => {
    if (dragging) {
      wasm.sim_mouse_up();
      dragging = false;
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

  // Frame the current selection, or the whole scene when nothing is selected
  // (C main.cpp:226 F key + Camera::Frame, camera.cpp:153). Keeps the current
  // yaw/pitch and refits the radius so the box fills the view.
  const frameSelection = () => {
    let box = selection;
    if (!box || box.isEmpty()) {
      box = new THREE.Box3().setFromObject(demo.content);
    }
    if (box.isEmpty() || !isFinite(box.min.x)) return;
    const center = box.getCenter(new THREE.Vector3());
    const ext = box.getSize(new THREE.Vector3()).multiplyScalar(0.5); // half extents
    const r = Math.sqrt(ext.x * ext.x + ext.y * ext.y + ext.z * ext.z);
    const c = demo.controls;
    if (r < 1e-6) {
      c.setView(c.yawDeg, c.pitchDeg, c.radius, [center.x, center.y, center.z]);
      return;
    }
    const aspect = demo.camera.aspect > 0 ? demo.camera.aspect : 1;
    const halfFovY = 0.5 * demo.camera.fov * (Math.PI / 180);
    const invTan = 1 / Math.tan(halfFovY);
    const distV = r * invTan;
    const distH = (r * invTan) / aspect;
    const d = Math.max(distV, distH) * 1.5; // C main.cpp:245 padding
    c.setView(c.yawDeg, c.pitchDeg, d, [center.x, center.y, center.z]);
  };

  // --- Event-bus consumers (menu bar + global keys dispatch here) ---
  const unsubscribers: (() => void)[] = [];
  unsubscribers.push(
    demoBus.on("sim.pause", () => state.setPaused(!state.paused)),
    demoBus.on("sim.step", () => {
      state.setPaused(true);
      state.requestStep(1);
    }),
    demoBus.on("sim.restart", () => state.restart()),
    demoBus.on("sim.frame", () => frameSelection()),
    demoBus.on("view.flag", ({ name, value }) => {
      setViewBit(name, value);
      pushDebugFlags();
      // Reflect menu-originated changes back onto the panel checkbox (no-op when
      // the panel itself emitted the event).
      const cb = panelCheckboxes.get(name);
      if (cb && cb.checked !== value) cb.checked = value;
    }),
    demoBus.on("view.scale", ({ name, value }) => {
      if (name === "joint") drawScales.joint = value;
      else if (name === "force") drawScales.force = value;
      else return; // drawDistance et al. are not part of the draw-scale setter
      pushDrawScales();
    }),
  );
  // Sync current menu state into the freshly-attached layer (the module-level
  // emit ran at boot, before this demo mounted).
  emitInitialState();

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

    // Overlay lines/points: the mask is set globally via sim_set_debug_flags, so
    // the legacy `sim_debug_draw(flags)` arg is ignored by the wasm; pass viewMask
    // for clarity. Only fetch when an overlay-relevant bit is set (the solid-mesh
    // bits shapes/transparent alone produce no overlay geometry).
    if ((viewMask & OVERLAY_MASK) !== 0) {
      debugOverlay.update(wasm.sim_debug_draw(viewMask));
    } else {
      debugOverlay.clear();
    }

    // Text labels (body names / mass / sleep / contact features): fetch only when
    // a text-relevant bit is set and the adapter exposes the channel.
    if ((viewMask & TEXT_MASK) !== 0 && wasm.sim_debug_text) {
      textOverlay.update(parseDebugText(wasm.sim_debug_text()));
    } else {
      textOverlay.clear();
    }

    frame += 1;
    if (frame % 2 === 0) {
      frameMsEl.textContent = `${lastFrameMs.toFixed(1)} ms`;
      stepCountEl.textContent = `step ${state.stepCount}`;
      const cam = cameraReadout(demo);
      const px = cam.px + worldOrigin[0];
      const py = cam.py + worldOrigin[1];
      const pz = cam.pz + worldOrigin[2];
      camPivotEl.textContent = `pivot m (${px.toFixed(1)}, ${py.toFixed(1)}, ${pz.toFixed(1)})`;
      camYawEl.textContent = `yaw/pitch (${cam.yaw.toFixed(1)}, ${cam.pitch.toFixed(1)})`;
      camRadiusEl.textContent = `radius m ${cam.radius.toFixed(1)}`;
    }

    return stepped;
  }

  const sampleNameEl = infoHead.querySelector(".sample-name") as HTMLElement;

  const withTick = state as SimControllerWithTick;
  withTick.tickFrame = tickFrame;
  withTick.setWorldOrigin = (o) => {
    worldOrigin = o;
  };
  withTick.setSampleName = (name) => {
    sampleNameEl.textContent = name;
  };

  state.dispose = () => {
    for (const off of unsubscribers) off();
    canvas.removeEventListener("pointerdown", onPointerDown);
    canvas.removeEventListener("pointermove", onPointerMove);
    canvas.removeEventListener("pointerup", onPointerUp);
    canvas.removeEventListener("pointercancel", onPointerUp);
    canvas.removeEventListener("contextmenu", onContext);
    wasm.sim_mouse_up();
    if (wasm.sim_is_recording?.()) wasm.sim_stop_recording?.();
    debugOverlay.dispose();
    textOverlay.dispose();
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

export type SimControllerWithTick = SimController & {
  tickFrame: () => boolean;
  /** Update the camera-readout world origin (per-scene on multi-scene pages). */
  setWorldOrigin: (origin: [number, number, number]) => void;
  /** Update the Info-panel sample name (per-scene on multi-scene pages). */
  setSampleName: (name: string) => void;
};
