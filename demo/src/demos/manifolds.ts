// Contact Manifolds — narrow-phase contact points in 3D (Three.js).

import { createButtonGroup, createInfoBox, createReadout, updateReadout } from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import {
  COLORS,
  DemoScene,
  makeArrow,
  makeAxes,
  makeCapsule,
  makeDot,
  makeSolidBox,
  makeSphere,
  makeWireBox,
} from "../three-scene.ts";

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

  const demo = new DemoScene(canvas, { distance: 10 });
  canvas.addEventListener("pointermove", (e) => {
    if (e.buttons) return;
    const rect = canvas.getBoundingClientRect();
    const nx = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    const ny = 1 - ((e.clientY - rect.top) / rect.height) * 2;
    target = [nx * 2.5, ny * 2.0, 0.3];
  });

  const start = performance.now();

  const stop = runLoop(() => {
    demo.clearContent();
    demo.clearDynamic();
    demo.content.add(makeAxes(1.5));

    const t = (performance.now() - start) / 1000;
    const angle = 0.35 * t;
    const [bx, by, bz] = target;

    // Shape A
    if (kind === 0) {
      demo.content.add(makeSphere(0, 0, 0, 1, COLORS.shape, 0.45));
    } else if (kind === 1) {
      demo.content.add(makeCapsule([0, -0.8, 0], [0, 0.8, 0], 0.35, COLORS.shape, 0.45));
    } else {
      demo.content.add(makeSolidBox(0, 0, 0, 1, 1, 1, COLORS.shape, 0.35));
      demo.content.add(makeWireBox(0, 0, 0, 1, 1, 1, COLORS.shape));
    }

    // Shape B
    if (kind === 0 || kind === 2) {
      demo.content.add(makeSphere(bx, by, bz, kind === 0 ? 0.75 : 0.55, COLORS.good, 0.5));
    } else if (kind === 1) {
      const ca: [number, number, number] = [bx, by - 0.6, bz];
      const cb: [number, number, number] = [bx, by + 0.6, bz];
      // Visual hint of rotation: tilt slightly with angle
      const sx = Math.sin(angle) * 0.25;
      ca[0] += sx;
      cb[0] -= sx;
      demo.content.add(makeCapsule(ca, cb, 0.3, COLORS.good, 0.5));
    } else {
      demo.content.add(makeSolidBox(bx, by, bz, 0.7, 0.7, 0.7, COLORS.good, 0.4));
      demo.content.add(makeWireBox(bx, by, bz, 0.7, 0.7, 0.7, COLORS.good));
    }

    let m: Float32Array;
    if (kind === 0) m = wasm.collide_spheres_demo(bx, by, bz);
    else if (kind === 1) m = wasm.collide_capsules_demo(bx, by, bz, angle);
    else if (kind === 2) m = wasm.collide_hull_sphere_demo(bx, by, bz);
    else m = wasm.collide_hulls_demo(bx, by, bz, angle);

    const pointCount = m[3]!;
    for (let i = 0; i < pointCount; i++) {
      const px = m[4 + 4 * i]!;
      const py = m[5 + 4 * i]!;
      const pz = m[6 + 4 * i]!;
      const sep = m[7 + 4 * i]!;
      demo.dynamic.add(makeDot([px, py, pz], sep < 0 ? COLORS.hit : COLORS.good, 0.08));
      demo.dynamic.add(makeArrow([px, py, pz], [m[0]!, m[1]!, m[2]!], 0.5, COLORS.accent));
    }

    const entries = [
      { label: "Pair", value: KIND_NAMES[kind]! },
      { label: "Points", value: String(pointCount) },
    ];
    if (pointCount > 0) {
      entries.push({
        label: "Normal",
        value: `(${m[0]!.toFixed(3)}, ${m[1]!.toFixed(3)}, ${m[2]!.toFixed(3)})`,
      });
      entries.push({
        label: "Separations",
        value: Array.from({ length: pointCount }, (_, i) => m[7 + 4 * i]!.toFixed(4)).join(", "),
      });
    }
    updateReadout(readout, entries);
    demo.render();
  }, readout);

  return () => {
    stop();
    demo.dispose();
  };
}
