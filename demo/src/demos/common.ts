// Shared demo page scaffolding (2D canvas helpers remain for Deterministic Math).

import { SamplePanel, type SamplePanelOpts } from "../sample-panel.ts";

export function fitCanvas(canvas: HTMLCanvasElement) {
  const w = Math.max(1, Math.round(canvas.clientWidth));
  const h = Math.max(1, Math.round(canvas.clientHeight));
  if (canvas.width !== w) canvas.width = w;
  if (canvas.height !== h) canvas.height = h;
}

export type RunLoopOpts = {
  /**
   * Gate for scenes whose reset is asynchronous (e.g. Shapes' Conveyor Mesh
   * fetches conveyor.obj before `shapes_reset_conveyor`). The loop keeps
   * scheduling frames but skips `frame()` until this returns true, so the first
   * synchronous tick never calls into an uninitialized wasm scene (which would
   * panic on `expect("… not initialized")`). Async asset-loading scenes inherit
   * this by returning a promise from their reset and flipping a ready flag when
   * it resolves.
   */
  ready?: () => boolean;
};

export function runLoop(
  frame: () => void,
  readout: HTMLElement,
  opts: RunLoopOpts = {},
): () => void {
  let rafId = 0;
  let stopped = false;

  function tick() {
    if (stopped) return;
    // Not yet initialized (async scene still loading): reschedule without
    // stepping/rendering the scene. Surfacing a loading state is the page's job.
    if (opts.ready && !opts.ready()) {
      rafId = requestAnimationFrame(tick);
      return;
    }
    try {
      frame();
    } catch (e) {
      readout.textContent = `Demo error: ${e}`;
      console.error(e);
      return;
    }
    rafId = requestAnimationFrame(tick);
  }

  tick();
  return () => {
    stopped = true;
    cancelAnimationFrame(rafId);
  };
}

/**
 * Awake-gated cache for the per-demo `*_styles()` export. That call is a full
 * `world_draw` capture, wasteful to run every frame for a settled/static scene.
 * The returned getter refetches only when something is awake, on the first
 * frame, and once on the awake→0 transition (so sleep recoloring is still
 * captured); otherwise it returns the last snapshot. Pass the awake dynamic-body
 * count (counters[5] from `counters_with_sleep`) and a fetch closure.
 */
export function makeStyleGate<T>(): (awake: number, fetch: () => T) => T {
  let cached: T | undefined;
  let prevAwake = -1;
  return (awake, fetch) => {
    const firstFrame = cached === undefined;
    const justSettled = prevAwake > 0 && awake === 0;
    if (firstFrame || awake > 0 || justSettled) cached = fetch();
    prevAwake = awake;
    return cached as T;
  };
}

/**
 * Canvas-hint fragment for camera orbit. With a mouse, plain left-drag does not
 * orbit (C camera: right-drag / Alt+drag); on touch, one-finger drag does. The
 * `.hint-desktop` / `.hint-mobile` spans switch on `(pointer: coarse)`.
 */
export const ORBIT_HINT =
  '<span class="hint-desktop">Right-drag orbit</span><span class="hint-mobile">Drag to orbit</span>';

export type DemoPageOpts = {
  /** C sample category label (Info-panel fallback when no registry entry resolves). */
  category?: string;
  /** Use Samples App dark shell (right Info panel). Default false. */
  samplesShell?: boolean;
  /** Hide the top demo header — name lives in the Info panel. Default follows samplesShell. */
  hideHeader?: boolean;
  /**
   * Description of the sample on screen, shown right under the panel header:
   * one string for a single-sample page, or per-scene text keyed by the page's
   * scene key (the panel shows only the active scene's entry).
   */
  info?: SamplePanelOpts["info"];
  /** Per-scene canvas hints for scenes with their own controls (falls back to `hint`). */
  sceneHints?: Record<string, string>;
  /** Page-wide notes not tied to one sample (collapsed "About this page"). */
  about?: string;
};

/**
 * Build a sample page: canvas area + the shared Info panel (see sample-panel.ts).
 * `controls` is the panel body — everything a page appends lands directly under
 * the active sample's description; the shared sim chrome (transport, scene
 * switcher, Solver, …) lives in the panel tail below it. Multi-scene pages call
 * `panel.setScene(key)` whenever the scene on screen changes.
 */
export function demoPage(
  container: HTMLElement,
  title: string,
  description: string,
  hint: string,
  version: string,
  opts: DemoPageOpts = {},
): { canvas: HTMLCanvasElement; controls: HTMLElement; page: HTMLElement; panel: SamplePanel } {
  const samplesShell = opts.samplesShell === true;
  const hideHeader = opts.hideHeader ?? samplesShell;
  const shellClass = samplesShell ? " samples-shell" : "";

  container.innerHTML = `
    <div class="demo-page${shellClass}">
      ${
        hideHeader
          ? ""
          : `<div class="demo-header">
        <h2>${title} <span class="badge-live">LIVE</span></h2>
        <p>${description}</p>
        <p class="wasm-note">computed by box3d-rust v${version} · WebAssembly</p>
      </div>`
      }
      <div class="demo-body">
        <div class="demo-canvas-area">
          <canvas id="demo-canvas"></canvas>
          <div class="canvas-hint">${hint}</div>
        </div>
        <div class="demo-controls" id="controls"></div>
      </div>
    </div>
  `;
  const panel = new SamplePanel(document.getElementById("controls")!, {
    title,
    category: opts.category,
    hintEl: container.querySelector(".canvas-hint") as HTMLElement | null,
    hint,
    info: opts.info,
    sceneHints: opts.sceneHints,
    about: opts.about,
  });
  return {
    canvas: document.getElementById("demo-canvas") as HTMLCanvasElement,
    controls: panel.body,
    page: container.querySelector(".demo-page") as HTMLElement,
    panel,
  };
}
