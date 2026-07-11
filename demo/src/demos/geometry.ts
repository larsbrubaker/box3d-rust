// Geometry Queries — 3D ray casts and GJK closest points (Three.js).

import * as THREE from "three";
import {
  createInfoBox,
  createReadout,
  createSlider,
  updateReadout,
} from "../controls.ts";
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

const SHAPE_NAMES = ["Sphere", "Capsule", "Hull", "AABB"] as const;

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Geometry Queries",
    "A camera ray follows the cursor and is cast against a sphere, capsule, box hull, and AABB via " +
      "the ported <code>b3RayCast*</code> functions. The green probe reports GJK closest " +
      "points from <code>b3ShapeDistance</code>.",
    "Drag to orbit · move cursor to aim · click to lock aim",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "The ray originates at the camera and passes through the cursor (NDC → world). " +
        "Orbit with drag; aim with move or click. Red markers are wasm ray hits with " +
        "surface normals. The dashed green line is the closest-point witness between the " +
        "probe sphere and the scene sphere.",
    ),
  );

  let rayLength = 40;
  let probeAlong = 8;
  let aimLocked = false;
  controls.appendChild(
    createSlider("Ray length", 5, 80, rayLength, 1, (v) => {
      rayLength = v;
    }),
  );
  controls.appendChild(
    createSlider("GJK probe along ray", 1, 40, probeAlong, 0.5, (v) => {
      probeAlong = v;
    }),
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

  // Screen-space aim (NDC). Reprojected each frame so orbit keeps the ray
  // aimed at the same pixel until the cursor moves again.
  let ndcX = 0;
  let ndcY = 0;
  const pointerDown = { x: 0, y: 0, moved: false };
  const ndc = new THREE.Vector2();
  const raycaster = new THREE.Raycaster();

  function setNdcFromEvent(e: PointerEvent) {
    const rect = canvas.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return;
    ndcX = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    ndcY = 1 - ((e.clientY - rect.top) / rect.height) * 2;
  }

  const onPointerMove = (e: PointerEvent) => {
    if (e.buttons) {
      const dx = e.clientX - pointerDown.x;
      const dy = e.clientY - pointerDown.y;
      if (dx * dx + dy * dy > 16) pointerDown.moved = true;
      return;
    }
    if (aimLocked) return;
    setNdcFromEvent(e);
  };

  const onPointerDown = (e: PointerEvent) => {
    if (e.button !== 0) return;
    pointerDown.x = e.clientX;
    pointerDown.y = e.clientY;
    pointerDown.moved = false;
  };

  const onPointerUp = (e: PointerEvent) => {
    if (e.button !== 0) return;
    if (pointerDown.moved) return;
    // Click: aim through this pixel and lock until unlock click / Escape.
    setNdcFromEvent(e);
    aimLocked = !aimLocked;
  };

  const onKeyDown = (e: KeyboardEvent) => {
    if (e.key === "Escape") aimLocked = false;
  };

  canvas.addEventListener("pointermove", onPointerMove);
  canvas.addEventListener("pointerdown", onPointerDown);
  canvas.addEventListener("pointerup", onPointerUp);
  window.addEventListener("keydown", onKeyDown);

  const stop = runLoop(() => {
    demo.clearDynamic();

    ndc.set(ndcX, ndcY);
    raycaster.setFromCamera(ndc, demo.camera);
    const originVec = raycaster.ray.origin;
    const dir = raycaster.ray.direction;
    const origin: [number, number, number] = [originVec.x, originVec.y, originVec.z];
    const tx = dir.x * rayLength;
    const ty = dir.y * rayLength;
    const tz = dir.z * rayLength;

    const results = wasm.ray_cast_scene(origin[0], origin[1], origin[2], tx, ty, tz);
    let nearest = 1.0;
    let hitCount = 0;
    let nearestHit: {
      name: string;
      fraction: number;
      point: [number, number, number];
      normal: [number, number, number];
    } | null = null;

    for (let i = 0; i < 4; i++) {
      if (results[8 * i] !== 1.0) continue;
      hitCount++;
      const fraction = results[8 * i + 1]!;
      if (fraction < nearest) {
        nearest = fraction;
        nearestHit = {
          name: SHAPE_NAMES[i]!,
          fraction,
          point: [results[8 * i + 2]!, results[8 * i + 3]!, results[8 * i + 4]!],
          normal: [results[8 * i + 5]!, results[8 * i + 6]!, results[8 * i + 7]!],
        };
      }
    }

    const endFrac = nearest < 1.0 ? nearest : 1.0;
    demo.dynamic.add(
      makeSegment(
        origin,
        [origin[0] + tx * endFrac, origin[1] + ty * endFrac, origin[2] + tz * endFrac],
        COLORS.accent,
      ),
    );
    // Faint remainder past nearest hit so the full cast length stays visible.
    if (nearest < 1.0) {
      demo.dynamic.add(
        makeSegment(
          [origin[0] + tx * nearest, origin[1] + ty * nearest, origin[2] + tz * nearest],
          [origin[0] + tx, origin[1] + ty, origin[2] + tz],
          COLORS.muted,
        ),
      );
    }
    demo.dynamic.add(makeDot(origin, COLORS.accent, 0.08));

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

    const probeT = Math.min(probeAlong, rayLength);
    const probe: [number, number, number] = [
      origin[0] + dir.x * probeT,
      origin[1] + dir.y * probeT,
      origin[2] + dir.z * probeT,
    ];
    const cp = wasm.closest_points(probe[0], probe[1], probe[2]);
    demo.dynamic.add(makeSphere(probe[0], probe[1], probe[2], 0.15, COLORS.good, 0.7));
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

    const entries: { label: string; value: string }[] = [
      { label: "Aim", value: aimLocked ? "locked (click/Esc)" : "follow cursor" },
      { label: "Ray hits", value: `${hitCount}/4` },
      {
        label: "Nearest",
        value: nearestHit
          ? `${nearestHit.name} @ t=${nearestHit.fraction.toFixed(4)}`
          : "—",
      },
    ];
    if (nearestHit) {
      entries.push({
        label: "Hit point",
        value: nearestHit.point.map((v) => v.toFixed(3)).join(", "),
      });
      entries.push({
        label: "Hit normal",
        value: nearestHit.normal.map((v) => v.toFixed(3)).join(", "),
      });
    }
    for (let i = 0; i < 4; i++) {
      const hit = results[8 * i] === 1.0;
      entries.push({
        label: SHAPE_NAMES[i]!,
        value: hit ? `hit t=${results[8 * i + 1]!.toFixed(4)}` : "miss",
      });
    }
    entries.push(
      { label: "b3ShapeDistance", value: `${cp[6]!.toFixed(4)} m` },
      { label: "GJK iterations", value: String(cp[7]) },
    );
    updateReadout(readout, entries);

    demo.render();
  }, readout);

  return () => {
    stop();
    canvas.removeEventListener("pointermove", onPointerMove);
    canvas.removeEventListener("pointerdown", onPointerDown);
    canvas.removeEventListener("pointerup", onPointerUp);
    window.removeEventListener("keydown", onKeyDown);
    demo.dispose();
  };
}
