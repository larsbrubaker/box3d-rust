// Regression test for the mouse-anchor depth bug in camera-controls.ts.
//
// The demos use near-horizontal cameras (low pitch, eye a few metres above the
// ground, orbit radius ~30 m). When the cursor is off any mesh, the ground-plane
// (Y=0) fallback anchor lands near the horizon — the raw intersection sits 100+ m
// out while the scene is only ~30 m away. Pan/orbit anchored to that far point
// then move the camera tens of metres for a small drag, so the UX reads as
// "middle pan doesn't work and the gesture isn't at the cursor".
//
// This test poses a camera exactly like the measured failure (fov 50, 740x686,
// pivot (0,5,0), yaw 25°, pitch 10°, radius 30) and casts a grazing ray toward
// the ground near the horizon. It documents the raw far anchor, then asserts the
// exported bounding helper clamps the anchor to <= 2*radius while keeping it in
// front of the camera.

import { test, expect, beforeAll } from "bun:test";
import * as THREE from "three";

// CameraControls' constructor registers window listeners; stub them for the
// headless integration test below. findIntersectionPoint itself needs no DOM.
beforeAll(() => {
  const g = globalThis as unknown as { window?: unknown; document?: unknown };
  g.window ??= { addEventListener() {}, removeEventListener() {} };
  g.document ??= { activeElement: { tagName: "CANVAS" } };
});
import {
  boundAnchorDistance,
  MAX_ANCHOR_FACTOR,
  CameraControls,
} from "../src/render/camera-controls.ts";

const DEG_TO_RAD = Math.PI / 180;

// Reproduce the pose from applyToCamera(): forward points pivot -> eye and
// eye = pivot + forward * radius.
function posedCamera() {
  const fov = 50;
  const width = 740;
  const height = 686;
  const pivot = new THREE.Vector3(0, 5, 0);
  const yaw = 25 * DEG_TO_RAD;
  const pitch = 10 * DEG_TO_RAD;
  const radius = 30;

  const cp = Math.cos(pitch);
  const forward = new THREE.Vector3(Math.sin(yaw) * cp, Math.sin(pitch), Math.cos(yaw) * cp);
  const eye = pivot.clone().addScaledVector(forward, radius);

  const camera = new THREE.PerspectiveCamera(fov, width / height, 0.1, 1000);
  camera.position.copy(eye);
  camera.up.set(0, 1, 0);
  camera.lookAt(pivot);
  camera.updateMatrixWorld();
  camera.updateProjectionMatrix();

  return { camera, pivot, radius, width, height };
}

function groundAnchorRay(camera: THREE.PerspectiveCamera, width: number, height: number) {
  // Pixel above screen centre: this grazing ray looks just below the horizon and
  // strikes the ground far out. clientY 283 (~60 px above centre) reproduces the
  // measured ~110 m failing anchor.
  const clientX = width / 2;
  const clientY = 283;
  const ndc = new THREE.Vector2((clientX / width) * 2 - 1, -((clientY / height) * 2 - 1));
  const raycaster = new THREE.Raycaster();
  raycaster.setFromCamera(ndc, camera);
  return raycaster.ray;
}

test("ground-plane fallback anchor lands far beyond the scene (documents the bug)", () => {
  const { camera, radius, width, height } = posedCamera();
  const ray = groundAnchorRay(camera, width, height);

  const ground = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  const hit = new THREE.Vector3();
  expect(ray.intersectPlane(ground, hit)).not.toBeNull();

  const rawT = hit.clone().sub(ray.origin).dot(ray.direction);
  // The raw anchor is well past 2*radius (measured 100-150 m for a 30 m scene).
  expect(rawT).toBeGreaterThan(MAX_ANCHOR_FACTOR * radius);
});

test("boundAnchorDistance clamps the far anchor to <= 2*radius, still in front", () => {
  const { camera, radius, width, height } = posedCamera();
  const ray = groundAnchorRay(camera, width, height);

  const ground = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  const hit = new THREE.Vector3();
  ray.intersectPlane(ground, hit);

  const boundedT = boundAnchorDistance(ray.origin, ray.direction, hit, radius);
  expect(boundedT).toBeLessThanOrEqual(MAX_ANCHOR_FACTOR * radius);
  // The clamped anchor must stay in the cursor's direction (in front of the eye).
  expect(boundedT).toBeGreaterThan(0);

  const boundedPoint = ray.origin.clone().addScaledVector(ray.direction, boundedT);
  const t = boundedPoint.clone().sub(ray.origin).dot(ray.direction);
  expect(t).toBeGreaterThan(0);
});

test("boundAnchorDistance leaves near anchors untouched", () => {
  const radius = 30;
  const origin = new THREE.Vector3(0, 0, 0);
  const dir = new THREE.Vector3(0, 0, 1);
  const near = new THREE.Vector3(0, 0, 10); // t = 10, well within 2*radius
  expect(boundAnchorDistance(origin, dir, near, radius)).toBeCloseTo(10, 6);
});

test("findIntersectionPoint clamps the far ground fallback through the real class", () => {
  // Drives the private clampFarAnchor fallback selection end-to-end so a
  // regression in the fallback logic (e.g. dropping the pt <= maxT guard) is
  // caught — the pure-helper tests cannot see that path.
  const width = 740;
  const height = 686;
  const radius = 30;
  const dom = {
    addEventListener() {},
    getBoundingClientRect: () => ({ left: 0, top: 0, width, height }),
  } as unknown as HTMLElement;
  const camera = new THREE.PerspectiveCamera(50, width / height, 0.1, 1000);
  const controls = new CameraControls(camera, dom);
  controls.setView(25, 10, radius, [0, 5, 0]);
  camera.updateMatrixWorld();
  camera.updateProjectionMatrix();

  // Same grazing pixel as the reproduction test: its raw ground hit is > 2*radius,
  // with pickRoots empty so the ground/pivot fallback path runs.
  const anchor: THREE.Vector3 = (
    controls as unknown as {
      findIntersectionPoint(x: number, y: number): THREE.Vector3;
    }
  ).findIntersectionPoint(width / 2, 283);

  const dist = anchor.distanceTo(camera.position);
  expect(dist).toBeLessThanOrEqual(MAX_ANCHOR_FACTOR * radius);
  expect(dist).toBeGreaterThan(0);
});
