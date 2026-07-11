// Height Field — wave HF with ray cast.

import { createInfoBox, createReadout, updateReadout } from "../controls.ts";
import { getWasm } from "../wasm.ts";
import {
  OrbitCamera,
  demoPage,
  drawAxes,
  drawDot,
  drawSegment,
  drawWireEdges,
  fitCanvas,
  runLoop,
} from "./common.ts";

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Height Field",
    "A sinusoidal height field from <code>b3CreateWave</code>. Ray casts use the ported " +
      "<code>b3RayCastHeightField</code>; triangle edges come from " +
      "<code>b3GetHeightFieldTriangle</code>.",
    "Drag to orbit · ray sweeps automatically",
    wasm.version(),
  );

  const triCount = wasm.hf_build_wave();
  const wire = wasm.hf_wireframe();

  controls.appendChild(
    createInfoBox(
      `Wave height field with ${triCount} triangles. The sweeping ray reports hit fraction, ` +
        "point, normal, and triangle index — all from Rust wasm.",
    ),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  const cam = new OrbitCamera();
  cam.yaw = 0.9;
  cam.pitch = 0.55;
  cam.distance = 14;
  cam.scale = 28;
  cam.target = [4, 0, 4];
  const detach = cam.attachDrag(canvas);
  const ctx = canvas.getContext("2d")!;
  const start = performance.now();

  const stop = runLoop(() => {
    fitCanvas(canvas);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawAxes(ctx, cam, canvas, 2);
    drawWireEdges(ctx, cam, canvas, wire, "#5a6170", 0);

    const t = (performance.now() - start) / 1000;
    const ox = 4 + Math.cos(t * 0.7) * 5;
    const oz = 4 + Math.sin(t * 0.7) * 5;
    const oy = 6;
    const tx = 0;
    const ty = -12;
    const tz = 0;
    const hit = wasm.hf_ray_cast(ox, oy, oz, tx, ty, tz);
    const frac = hit[0] === 1.0 ? hit[1] : 1.0;
    drawSegment(ctx, cam, canvas, [ox, oy, oz], [ox + tx * frac, oy + ty * frac, oz + tz * frac], "#2563eb", 2);
    drawDot(ctx, cam, canvas, [ox, oy, oz], "#2563eb", 4);
    if (hit[0] === 1.0) {
      drawDot(ctx, cam, canvas, [hit[2], hit[3], hit[4]], "#dc2626", 6);
      drawSegment(
        ctx, cam, canvas,
        [hit[2], hit[3], hit[4]],
        [hit[2] + hit[5], hit[3] + hit[6], hit[4] + hit[7]],
        "#dc2626",
        2,
      );
    }

    updateReadout(readout, [
      { label: "Triangles", value: String(triCount) },
      { label: "Hit", value: hit[0] === 1.0 ? "yes" : "no" },
      { label: "Fraction", value: hit[1].toFixed(4) },
      { label: "Triangle", value: hit[0] === 1.0 ? String(hit[8]) : "—" },
    ]);
  }, readout);

  return () => {
    stop();
    detach();
  };
}
