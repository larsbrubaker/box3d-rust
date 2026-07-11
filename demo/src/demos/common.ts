// Shared demo page scaffolding and a simple orbiting 3D canvas projector.

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

/** Simple yaw/pitch orbit camera projecting world XYZ onto the canvas. */
export class OrbitCamera {
  yaw = 0.55;
  pitch = 0.45;
  distance = 10;
  scale = 55;
  target: [number, number, number] = [0, 0, 0];

  project(canvas: HTMLCanvasElement, x: number, y: number, z: number): [number, number, number] {
    const [tx, ty, tz] = this.target;
    let dx = x - tx;
    let dy = y - ty;
    let dz = z - tz;

    const cy = Math.cos(this.yaw);
    const sy = Math.sin(this.yaw);
    const cp = Math.cos(this.pitch);
    const sp = Math.sin(this.pitch);

    // Yaw around Y, then pitch around X
    const x1 = dx * cy + dz * sy;
    const z1 = -dx * sy + dz * cy;
    const y2 = dy * cp - z1 * sp;
    const z2 = dy * sp + z1 * cp;

    const depth = z2 + this.distance;
    const sx = canvas.width / 2 + x1 * this.scale;
    const sy2 = canvas.height / 2 - y2 * this.scale;
    return [sx, sy2, depth];
  }

  attachDrag(canvas: HTMLCanvasElement) {
    let dragging = false;
    let lastX = 0;
    let lastY = 0;
    const onDown = (e: PointerEvent) => {
      dragging = true;
      lastX = e.clientX;
      lastY = e.clientY;
      canvas.setPointerCapture(e.pointerId);
    };
    const onMove = (e: PointerEvent) => {
      if (!dragging) return;
      this.yaw += (e.clientX - lastX) * 0.01;
      this.pitch += (e.clientY - lastY) * 0.01;
      this.pitch = Math.max(-1.2, Math.min(1.2, this.pitch));
      lastX = e.clientX;
      lastY = e.clientY;
    };
    const onUp = () => {
      dragging = false;
    };
    canvas.addEventListener("pointerdown", onDown);
    canvas.addEventListener("pointermove", onMove);
    canvas.addEventListener("pointerup", onUp);
    canvas.addEventListener("pointercancel", onUp);
    return () => {
      canvas.removeEventListener("pointerdown", onDown);
      canvas.removeEventListener("pointermove", onMove);
      canvas.removeEventListener("pointerup", onUp);
      canvas.removeEventListener("pointercancel", onUp);
    };
  }
}

export function drawSegment(
  ctx: CanvasRenderingContext2D,
  cam: OrbitCamera,
  canvas: HTMLCanvasElement,
  a: [number, number, number],
  b: [number, number, number],
  color: string,
  width = 1.5,
) {
  const [ax, ay] = cam.project(canvas, a[0], a[1], a[2]);
  const [bx, by] = cam.project(canvas, b[0], b[1], b[2]);
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(ax, ay);
  ctx.lineTo(bx, by);
  ctx.stroke();
}

export function drawDot(
  ctx: CanvasRenderingContext2D,
  cam: OrbitCamera,
  canvas: HTMLCanvasElement,
  p: [number, number, number],
  color: string,
  r = 5,
) {
  const [x, y] = cam.project(canvas, p[0], p[1], p[2]);
  ctx.beginPath();
  ctx.arc(x, y, r, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
}

export function drawWireBox(
  ctx: CanvasRenderingContext2D,
  cam: OrbitCamera,
  canvas: HTMLCanvasElement,
  cx: number, cy: number, cz: number,
  hx: number, hy: number, hz: number,
  color: string,
) {
  const corners: [number, number, number][] = [];
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      for (const sz of [-1, 1]) {
        corners.push([cx + sx * hx, cy + sy * hy, cz + sz * hz]);
      }
    }
  }
  // Index order for axis-aligned box edges
  const edges = [
    [0, 1], [2, 3], [4, 5], [6, 7],
    [0, 2], [1, 3], [4, 6], [5, 7],
    [0, 4], [1, 5], [2, 6], [3, 7],
  ];
  // Remap: our loop order is sx,sy,sz nested differently — use explicit corners
  const c: [number, number, number][] = [
    [cx - hx, cy - hy, cz - hz],
    [cx + hx, cy - hy, cz - hz],
    [cx - hx, cy + hy, cz - hz],
    [cx + hx, cy + hy, cz - hz],
    [cx - hx, cy - hy, cz + hz],
    [cx + hx, cy - hy, cz + hz],
    [cx - hx, cy + hy, cz + hz],
    [cx + hx, cy + hy, cz + hz],
  ];
  const e = [
    [0, 1], [1, 3], [3, 2], [2, 0],
    [4, 5], [5, 7], [7, 6], [6, 4],
    [0, 4], [1, 5], [2, 6], [3, 7],
  ];
  for (const [i, j] of e) drawSegment(ctx, cam, canvas, c[i], c[j], color);
  void corners;
  void edges;
}

export function drawWireEdges(
  ctx: CanvasRenderingContext2D,
  cam: OrbitCamera,
  canvas: HTMLCanvasElement,
  edges: ArrayLike<number>,
  color: string,
  offset = 0,
) {
  for (let i = offset; i + 5 < edges.length; i += 6) {
    drawSegment(
      ctx,
      cam,
      canvas,
      [edges[i], edges[i + 1], edges[i + 2]],
      [edges[i + 3], edges[i + 4], edges[i + 5]],
      color,
    );
  }
}

export function drawAxes(
  ctx: CanvasRenderingContext2D,
  cam: OrbitCamera,
  canvas: HTMLCanvasElement,
  len = 1.5,
) {
  drawSegment(ctx, cam, canvas, [0, 0, 0], [len, 0, 0], "#dc2626", 1);
  drawSegment(ctx, cam, canvas, [0, 0, 0], [0, len, 0], "#15803d", 1);
  drawSegment(ctx, cam, canvas, [0, 0, 0], [0, 0, len], "#2563eb", 1);
}
