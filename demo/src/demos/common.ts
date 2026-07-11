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

export function demoPage(
  container: HTMLElement,
  title: string,
  description: string,
  hint: string,
  version: string,
): { canvas: HTMLCanvasElement; controls: HTMLElement } {
  container.innerHTML = `
    <div class="demo-page">
      <div class="demo-header">
        <h2>${title} <span class="badge-live">LIVE</span></h2>
        <p>${description}</p>
        <p class="wasm-note">computed by box3d-rust v${version} · WebAssembly</p>
      </div>
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
  };
}
