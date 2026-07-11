// Mesh — box/grid mesh with ray cast and AABB.

import { createButtonGroup, createInfoBox, createReadout, updateReadout } from "../controls.ts";
import { getWasm } from "../wasm.ts";
import {
  OrbitCamera,
  demoPage,
  drawAxes,
  drawDot,
  drawSegment,
  drawWireBox,
  drawWireEdges,
  fitCanvas,
  runLoop,
} from "./common.ts";

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Mesh",
    "Triangle meshes from <code>b3CreateBoxMesh</code> / <code>b3CreateGridMesh</code>. " +
      "Ray casts use <code>b3RayCastMesh</code>; the AABB comes from " +
      "<code>b3ComputeMeshAABB</code>.",
    "Drag to orbit · ray sweeps automatically",
    wasm.version(),
  );

  let stats = wasm.mesh_build_box();
  let wire = wasm.mesh_wireframe();
  let aabb = wasm.mesh_aabb();

  controls.appendChild(
    createInfoBox(
      "Switch between a box mesh and a flat grid. Hits report triangle index from the " +
        "ported mesh BVH ray cast.",
    ),
  );
  controls.appendChild(
    createButtonGroup(
      [
        { label: "Box", value: "box" },
        { label: "Grid", value: "grid" },
      ],
      "box",
      (v) => {
        stats = v === "grid" ? wasm.mesh_build_grid() : wasm.mesh_build_box();
        wire = wasm.mesh_wireframe();
        aabb = wasm.mesh_aabb();
      },
    ),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  const cam = new OrbitCamera();
  cam.distance = 10;
  cam.scale = 55;
  const detach = cam.attachDrag(canvas);
  const ctx = canvas.getContext("2d")!;
  const start = performance.now();

  const stop = runLoop(() => {
    fitCanvas(canvas);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawAxes(ctx, cam, canvas);
    drawWireEdges(ctx, cam, canvas, wire, "#2563eb");
    drawWireBox(
      ctx, cam, canvas,
      (aabb[0] + aabb[3]) / 2, (aabb[1] + aabb[4]) / 2, (aabb[2] + aabb[5]) / 2,
      (aabb[3] - aabb[0]) / 2, (aabb[4] - aabb[1]) / 2, (aabb[5] - aabb[2]) / 2,
      "#8b92a0",
    );

    const t = (performance.now() - start) / 1000;
    const ox = Math.cos(t * 0.8) * 4;
    const oy = 3;
    const oz = Math.sin(t * 0.8) * 4;
    const tx = -ox * 1.5;
    const ty = -6;
    const tz = -oz * 1.5;
    const hit = wasm.mesh_ray_cast(ox, oy, oz, tx, ty, tz);
    const frac = hit[0] === 1.0 ? hit[1] : 1.0;
    drawSegment(ctx, cam, canvas, [ox, oy, oz], [ox + tx * frac, oy + ty * frac, oz + tz * frac], "#15803d", 2);
    drawDot(ctx, cam, canvas, [ox, oy, oz], "#15803d", 4);
    if (hit[0] === 1.0) {
      drawDot(ctx, cam, canvas, [hit[2], hit[3], hit[4]], "#dc2626", 6);
    }

    updateReadout(readout, [
      { label: "Vertices", value: String(stats[0]) },
      { label: "Triangles", value: String(stats[1]) },
      { label: "Hit", value: hit[0] === 1.0 ? "yes" : "no" },
      { label: "Triangle", value: hit[0] === 1.0 ? String(hit[8]) : "—" },
    ]);
  }, readout);

  return () => {
    stop();
    detach();
  };
}
