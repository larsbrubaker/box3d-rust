// Geometry Queries — 3D ray casts and GJK closest points.

import { createInfoBox, createReadout, updateReadout } from "../controls.ts";
import { getWasm } from "../wasm.ts";
import {
  OrbitCamera,
  demoPage,
  drawAxes,
  drawDot,
  drawSegment,
  drawWireBox,
  fitCanvas,
  runLoop,
} from "./common.ts";

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Geometry Queries",
    "A ray tracks the cursor and is cast against a sphere, capsule, box hull, and AABB via " +
      "the ported <code>b3RayCast*</code> functions. The green probe reports GJK closest " +
      "points from <code>b3ShapeDistance</code>.",
    "Drag to orbit · move to aim the ray",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "Red dots are ray hits with surface normals. The dashed green line is the closest-point " +
        "witness between the probe sphere and the scene sphere.",
    ),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  const sphere = wasm.scene_shape(0);
  const capsule = wasm.scene_shape(1);
  const box = wasm.scene_shape(2);
  const aabb = wasm.scene_shape(3);

  const cam = new OrbitCamera();
  cam.distance = 12;
  cam.scale = 48;
  const detach = cam.attachDrag(canvas);

  let aim: [number, number, number] = [2, 0.5, 2];
  canvas.addEventListener("pointermove", (e) => {
    if (e.buttons) return; // orbiting
    const rect = canvas.getBoundingClientRect();
    const nx = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    const ny = 1 - ((e.clientY - rect.top) / rect.height) * 2;
    aim = [nx * 4, ny * 3, 2.5];
  });

  const ctx = canvas.getContext("2d")!;
  const ACCENT = "#2563eb";
  const HIT = "#dc2626";
  const GOOD = "#15803d";
  const SHAPE = "#5a6170";

  const stop = runLoop(() => {
    fitCanvas(canvas);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawAxes(ctx, cam, canvas);

    // Sphere as latitude rings (approx)
    {
      const [cx, cy, cz, r] = [sphere[0], sphere[1], sphere[2], sphere[3]];
      for (let ring = 0; ring < 3; ring++) {
        const yy = cy + (ring - 1) * r * 0.55;
        const rr = Math.sqrt(Math.max(0, r * r - (yy - cy) * (yy - cy)));
        for (let i = 0; i < 24; i++) {
          const a0 = (i / 24) * Math.PI * 2;
          const a1 = ((i + 1) / 24) * Math.PI * 2;
          drawSegment(
            ctx, cam, canvas,
            [cx + Math.cos(a0) * rr, yy, cz + Math.sin(a0) * rr],
            [cx + Math.cos(a1) * rr, yy, cz + Math.sin(a1) * rr],
            SHAPE, 1.2,
          );
        }
      }
    }

    // Capsule spine + ends
    drawSegment(
      ctx, cam, canvas,
      [capsule[0], capsule[1], capsule[2]],
      [capsule[3], capsule[4], capsule[5]],
      SHAPE, 2,
    );
    drawDot(ctx, cam, canvas, [capsule[0], capsule[1], capsule[2]], SHAPE, 4);
    drawDot(ctx, cam, canvas, [capsule[3], capsule[4], capsule[5]], SHAPE, 4);

    drawWireBox(ctx, cam, canvas, box[3], box[4], box[5], box[0], box[1], box[2], SHAPE);
    drawWireBox(
      ctx, cam, canvas,
      (aabb[0] + aabb[3]) / 2, (aabb[1] + aabb[4]) / 2, (aabb[2] + aabb[5]) / 2,
      (aabb[3] - aabb[0]) / 2, (aabb[4] - aabb[1]) / 2, (aabb[5] - aabb[2]) / 2,
      "#8b92a0",
    );

    const origin: [number, number, number] = [-5, 1, 0];
    const dx = aim[0] - origin[0];
    const dy = aim[1] - origin[1];
    const dz = aim[2] - origin[2];
    const len = Math.hypot(dx, dy, dz) || 1;
    const tx = (dx / len) * 14;
    const ty = (dy / len) * 14;
    const tz = (dz / len) * 14;

    const results = wasm.ray_cast_scene(origin[0], origin[1], origin[2], tx, ty, tz);
    let nearest = 1.0;
    let hitCount = 0;
    for (let i = 0; i < 4; i++) {
      if (results[8 * i] === 1.0) {
        hitCount++;
        nearest = Math.min(nearest, results[8 * i + 1]);
      }
    }

    drawSegment(
      ctx, cam, canvas,
      origin,
      [origin[0] + tx * nearest, origin[1] + ty * nearest, origin[2] + tz * nearest],
      ACCENT, 2,
    );
    drawDot(ctx, cam, canvas, origin, ACCENT, 5);

    for (let i = 0; i < 4; i++) {
      if (results[8 * i] !== 1.0) continue;
      const hx = results[8 * i + 2];
      const hy = results[8 * i + 3];
      const hz = results[8 * i + 4];
      const nx = results[8 * i + 5];
      const ny = results[8 * i + 6];
      const nz = results[8 * i + 7];
      drawDot(ctx, cam, canvas, [hx, hy, hz], HIT);
      drawSegment(ctx, cam, canvas, [hx, hy, hz], [hx + nx * 0.6, hy + ny * 0.6, hz + nz * 0.6], HIT, 2);
    }

    const cp = wasm.closest_points(aim[0], aim[1], aim[2]);
    drawDot(ctx, cam, canvas, aim, GOOD, 6);
    if (cp[6] >= 0) {
      ctx.setLineDash([6, 4]);
      drawSegment(ctx, cam, canvas, [cp[0], cp[1], cp[2]], [cp[3], cp[4], cp[5]], GOOD, 1.5);
      ctx.setLineDash([]);
      drawDot(ctx, cam, canvas, [cp[0], cp[1], cp[2]], GOOD, 4);
      drawDot(ctx, cam, canvas, [cp[3], cp[4], cp[5]], GOOD, 4);
    }

    updateReadout(readout, [
      { label: "Ray hits", value: `${hitCount}/4` },
      { label: "Nearest fraction", value: nearest.toFixed(4) },
      { label: "b3ShapeDistance", value: `${cp[6].toFixed(4)} m` },
      { label: "GJK iterations", value: String(cp[7]) },
    ]);
  }, readout);

  return () => {
    stop();
    detach();
  };
}
