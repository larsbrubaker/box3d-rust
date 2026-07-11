// Contact Manifolds — narrow-phase contact points in 3D.

import { createButtonGroup, createInfoBox, createReadout, updateReadout } from "../controls.ts";
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

const KIND_NAMES = ["spheres", "capsules", "hull-sphere", "hull-hull"];

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Contact Manifolds",
    "Contact points and normals from the ported <code>b3CollideSpheres</code> / " +
      "<code>b3CollideCapsules</code> / <code>b3CollideHullAndSphere</code> / " +
      "<code>b3CollideHulls</code>. Red points penetrate; green are speculative.",
    "Drag to orbit · move to position shape B",
    wasm.version(),
  );

  let kind = 0;
  let target: [number, number, number] = [1.8, 0.4, 0.2];

  controls.appendChild(
    createInfoBox(
      "Shape A is fixed at the origin. Shape B follows the cursor in the XZ plane " +
        "(Y from vertical mouse position) and slowly rotates for capsule/hull pairs.",
    ),
  );
  const kindGroup = createButtonGroup(
    [
      { label: "Spheres", value: "0" },
      { label: "Capsules", value: "1" },
      { label: "Hull·Sphere", value: "2" },
      { label: "Hull·Hull", value: "3" },
    ],
    "0",
    (v) => {
      kind = parseInt(v, 10);
    },
  );
  controls.appendChild(kindGroup);
  const readout = createReadout();
  controls.appendChild(readout);

  const cam = new OrbitCamera();
  cam.distance = 9;
  cam.scale = 60;
  const detach = cam.attachDrag(canvas);

  canvas.addEventListener("pointermove", (e) => {
    if (e.buttons) return;
    const rect = canvas.getBoundingClientRect();
    const nx = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    const ny = 1 - ((e.clientY - rect.top) / rect.height) * 2;
    target = [nx * 2.5, ny * 2.0, 0.3];
  });

  const ctx = canvas.getContext("2d")!;
  const start = performance.now();

  const stop = runLoop(() => {
    fitCanvas(canvas);
    const t = (performance.now() - start) / 1000;
    const angle = 0.35 * t;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawAxes(ctx, cam, canvas);

    // Shape A
    if (kind === 0) {
      // sphere A r=1
      for (let i = 0; i < 24; i++) {
        const a0 = (i / 24) * Math.PI * 2;
        const a1 = ((i + 1) / 24) * Math.PI * 2;
        drawSegment(ctx, cam, canvas, [Math.cos(a0), 0, Math.sin(a0)], [Math.cos(a1), 0, Math.sin(a1)], "#5a6170");
      }
    } else if (kind === 1) {
      drawSegment(ctx, cam, canvas, [0, -0.8, 0], [0, 0.8, 0], "#5a6170", 3);
    } else {
      drawWireBox(ctx, cam, canvas, 0, 0, 0, 1, 1, 1, "#5a6170");
    }

    // Shape B outline
    const [bx, by, bz] = target;
    if (kind === 0 || kind === 2) {
      drawDot(ctx, cam, canvas, [bx, by, bz], "#15803d", 8);
    } else if (kind === 1) {
      drawSegment(ctx, cam, canvas, [bx, by - 0.6, bz], [bx, by + 0.6, bz], "#15803d", 3);
    } else {
      drawWireBox(ctx, cam, canvas, bx, by, bz, 0.7, 0.7, 0.7, "#15803d");
    }

    let m: Float32Array;
    if (kind === 0) m = wasm.collide_spheres_demo(bx, by, bz);
    else if (kind === 1) m = wasm.collide_capsules_demo(bx, by, bz, angle);
    else if (kind === 2) m = wasm.collide_hull_sphere_demo(bx, by, bz);
    else m = wasm.collide_hulls_demo(bx, by, bz, angle);

    const pointCount = m[3];
    for (let i = 0; i < pointCount; i++) {
      const px = m[4 + 4 * i];
      const py = m[5 + 4 * i];
      const pz = m[6 + 4 * i];
      const sep = m[7 + 4 * i];
      drawDot(ctx, cam, canvas, [px, py, pz], sep < 0 ? "#dc2626" : "#15803d", 6);
      drawSegment(
        ctx, cam, canvas,
        [px, py, pz],
        [px + m[0] * 0.5, py + m[1] * 0.5, pz + m[2] * 0.5],
        "#2563eb",
        2,
      );
    }

    const entries = [
      { label: "Pair", value: KIND_NAMES[kind] },
      { label: "Points", value: String(pointCount) },
    ];
    if (pointCount > 0) {
      entries.push({
        label: "Normal",
        value: `(${m[0].toFixed(3)}, ${m[1].toFixed(3)}, ${m[2].toFixed(3)})`,
      });
      entries.push({
        label: "Separations",
        value: Array.from({ length: pointCount }, (_, i) => m[7 + 4 * i].toFixed(4)).join(", "),
      });
    }
    updateReadout(readout, entries);
  }, readout);

  return () => {
    stop();
    detach();
  };
}
