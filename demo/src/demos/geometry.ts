// Geometry Queries — 3D ray casts and GJK closest points (Three.js).

import { createInfoBox, createReadout, updateReadout } from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import {
  COLORS,
  DemoScene,
  makeArrow,
  makeAxes,
  makeCapsule,
  makeDashedSegment,
  makeDot,
  makeSegment,
  makeSolidBox,
  makeSphere,
  makeWireBox,
} from "../three-scene.ts";

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
      "Red markers are ray hits with surface normals. The dashed green line is the closest-point " +
        "witness between the probe sphere and the scene sphere.",
    ),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  const sphere = wasm.scene_shape(0);
  const capsule = wasm.scene_shape(1);
  const box = wasm.scene_shape(2);
  const aabb = wasm.scene_shape(3);

  const demo = new DemoScene(canvas, { distance: 14 });
  demo.content.add(makeAxes(1.5));
  demo.content.add(
    makeSphere(sphere[0]!, sphere[1]!, sphere[2]!, sphere[3]!, COLORS.shape, 0.55),
  );
  demo.content.add(
    makeCapsule(
      [capsule[0]!, capsule[1]!, capsule[2]!],
      [capsule[3]!, capsule[4]!, capsule[5]!],
      capsule[6]!,
      COLORS.shape,
      0.55,
    ),
  );
  demo.content.add(
    makeSolidBox(box[3]!, box[4]!, box[5]!, box[0]!, box[1]!, box[2]!, COLORS.shape, 0.4),
  );
  demo.content.add(
    makeWireBox(box[3]!, box[4]!, box[5]!, box[0]!, box[1]!, box[2]!, COLORS.shape),
  );
  demo.content.add(
    makeWireBox(
      (aabb[0]! + aabb[3]!) / 2,
      (aabb[1]! + aabb[4]!) / 2,
      (aabb[2]! + aabb[5]!) / 2,
      (aabb[3]! - aabb[0]!) / 2,
      (aabb[4]! - aabb[1]!) / 2,
      (aabb[5]! - aabb[2]!) / 2,
      COLORS.muted,
    ),
  );

  let aim: [number, number, number] = [2, 0.5, 2];
  canvas.addEventListener("pointermove", (e) => {
    if (e.buttons) return;
    const rect = canvas.getBoundingClientRect();
    const nx = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    const ny = 1 - ((e.clientY - rect.top) / rect.height) * 2;
    aim = [nx * 4, ny * 3, 2.5];
  });

  const stop = runLoop(() => {
    demo.clearDynamic();

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
        nearest = Math.min(nearest, results[8 * i + 1]!);
      }
    }

    demo.dynamic.add(
      makeSegment(
        origin,
        [origin[0] + tx * nearest, origin[1] + ty * nearest, origin[2] + tz * nearest],
        COLORS.accent,
      ),
    );
    demo.dynamic.add(makeDot(origin, COLORS.accent, 0.1));

    for (let i = 0; i < 4; i++) {
      if (results[8 * i] !== 1.0) continue;
      const hx = results[8 * i + 2]!;
      const hy = results[8 * i + 3]!;
      const hz = results[8 * i + 4]!;
      const nx = results[8 * i + 5]!;
      const ny = results[8 * i + 6]!;
      const nz = results[8 * i + 7]!;
      demo.dynamic.add(makeDot([hx, hy, hz], COLORS.hit, 0.09));
      demo.dynamic.add(makeArrow([hx, hy, hz], [nx, ny, nz], 0.6, COLORS.hit));
    }

    const cp = wasm.closest_points(aim[0], aim[1], aim[2]);
    demo.dynamic.add(makeSphere(aim[0], aim[1], aim[2], 0.15, COLORS.good, 0.7));
    if (cp[6]! >= 0) {
      demo.dynamic.add(
        makeDashedSegment(
          [cp[0]!, cp[1]!, cp[2]!],
          [cp[3]!, cp[4]!, cp[5]!],
          COLORS.good,
        ),
      );
      demo.dynamic.add(makeDot([cp[0]!, cp[1]!, cp[2]!], COLORS.good, 0.07));
      demo.dynamic.add(makeDot([cp[3]!, cp[4]!, cp[5]!], COLORS.good, 0.07));
    }

    updateReadout(readout, [
      { label: "Ray hits", value: `${hitCount}/4` },
      { label: "Nearest fraction", value: nearest.toFixed(4) },
      { label: "b3ShapeDistance", value: `${cp[6]!.toFixed(4)} m` },
      { label: "GJK iterations", value: String(cp[7]) },
    ]);

    demo.render();
  }, readout);

  return () => {
    stop();
    demo.dispose();
  };
}
