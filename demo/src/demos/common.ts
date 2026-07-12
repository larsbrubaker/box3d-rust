// Shared demo page scaffolding (2D canvas helpers remain for Deterministic Math).

export function fitCanvas(canvas: HTMLCanvasElement) {
  const w = Math.max(1, Math.round(canvas.clientWidth));
  const h = Math.max(1, Math.round(canvas.clientHeight));
  if (canvas.width !== w) canvas.width = w;
  if (canvas.height !== h) canvas.height = h;
}

export function runLoop(frame: () => void, readout: HTMLElement): () => void {
  let rafId = 0;
  let stopped = false;

  function tick() {
    if (stopped) return;
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

export type DemoPageOpts = {
  /** C sample category label (shown in Info panel). */
  category?: string;
  /** Use Samples App dark shell (right Info panel). Default false. */
  samplesShell?: boolean;
  /** Hide the top demo header — name lives in the Info panel. Default follows samplesShell. */
  hideHeader?: boolean;
};

export function demoPage(
  container: HTMLElement,
  title: string,
  description: string,
  hint: string,
  version: string,
  opts: DemoPageOpts = {},
): { canvas: HTMLCanvasElement; controls: HTMLElement; page: HTMLElement } {
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
  return {
    canvas: document.getElementById("demo-canvas") as HTMLCanvasElement,
    controls: document.getElementById("controls")!,
    page: container.querySelector(".demo-page") as HTMLElement,
  };
}
