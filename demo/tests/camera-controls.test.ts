// Camera gesture regression tests. These drive the real CameraControls
// production code with a stubbed DOM (bun has no window/document), dispatching
// the same MouseEvents the browser would, then asserting the ported C camera
// semantics (camera.cpp Update):
//   - plain right-drag = FPS fly-look: eye stays FIXED, radius unchanged, only
//     yaw/pitch rotate (look-in-place). Regression guard for the temp-vector
//     aliasing bug where `eyeBefore` aliased the shared `_forward` scratch and
//     the eye collapsed toward the origin (read as a violent zoom-in).
//   - Alt+right-drag-Y = radial zoom: radius changes, yaw/pitch unchanged.

import { test, expect, beforeAll } from "bun:test";
import * as THREE from "three";

type Listener = (e: unknown) => void;
class FakeTarget {
  listeners: Record<string, Listener[]> = {};
  addEventListener(type: string, cb: Listener) {
    (this.listeners[type] ??= []).push(cb);
  }
  removeEventListener() {}
  dispatch(type: string, e: unknown) {
    for (const cb of this.listeners[type] ?? []) cb(e);
  }
}

beforeAll(() => {
  // CameraControls attaches to window and reads document.activeElement.
  (globalThis as unknown as { window: FakeTarget }).window = new FakeTarget();
  (globalThis as unknown as { document: unknown }).document = {
    activeElement: { tagName: "CANVAS" },
  };
});

async function makeControls(dom: FakeTarget) {
  const { CameraControls } = await import("../src/render/camera-controls.ts");
  const camera = new THREE.PerspectiveCamera(60, 1.5, 0.1, 1000);
  const controls = new CameraControls(camera, dom as unknown as HTMLElement);
  return { camera, controls };
}

test("plain right-drag is look-in-place: eye fixed, radius unchanged", async () => {
  const dom = new FakeTarget();
  const win = (globalThis as unknown as { window: FakeTarget }).window;
  const { camera, controls } = await makeControls(dom);
  controls.setView(0, 15, 50, [0, 20, 0]); // box-stack view

  controls.update(1 / 60);
  const eye0 = camera.position.clone();
  const radius0 = controls.radius;

  // Right mouse down (no Alt), then drag right + down over several frames.
  dom.dispatch("mousedown", { button: 2, buttons: 2, clientX: 600, clientY: 400, altKey: false });
  let x = 600;
  let y = 400;
  for (let i = 0; i < 10; i++) {
    x += 20;
    y += 8;
    win.dispatch("mousemove", { button: 0, buttons: 2, clientX: x, clientY: y, altKey: false });
    controls.update(1 / 60);
  }
  win.dispatch("mouseup", { button: 2, buttons: 0, clientX: x, clientY: y, altKey: false });

  const eyeMoved = camera.position.distanceTo(eye0);
  // Look-in-place: the eye must not translate (allow a hair of float slop).
  expect(eyeMoved).toBeLessThan(1e-3);
  // No zoom: radius unchanged.
  expect(Math.abs(controls.radius - radius0)).toBeLessThan(1e-3);
  // The look direction DID rotate (drag right decreases yaw).
  expect(controls.yawDeg).toBeLessThan(-1);
});

test("Alt+right-drag-Y is radial zoom: radius changes, look direction fixed", async () => {
  const dom = new FakeTarget();
  const win = (globalThis as unknown as { window: FakeTarget }).window;
  const { controls } = await makeControls(dom);
  controls.setView(0, 15, 50, [0, 20, 0]);
  controls.update(1 / 60);

  const yaw0 = controls.yawDeg;
  const pitch0 = controls.pitchDeg;
  const radius0 = controls.radius;

  dom.dispatch("mousedown", { button: 2, buttons: 2, clientX: 600, clientY: 400, altKey: true });
  let y = 400;
  for (let i = 0; i < 10; i++) {
    y -= 10; // drag up -> zoom out (radius grows)
    win.dispatch("mousemove", { button: 0, buttons: 2, clientX: 600, clientY: y, altKey: true });
    controls.update(1 / 60);
  }
  win.dispatch("mouseup", { button: 2, buttons: 0, clientX: 600, clientY: y, altKey: true });

  expect(controls.radius).toBeGreaterThan(radius0 + 0.5); // zoomed out
  expect(Math.abs(controls.yawDeg - yaw0)).toBeLessThan(1e-3); // look dir unchanged
  expect(Math.abs(controls.pitchDeg - pitch0)).toBeLessThan(1e-3);
});

test("arrow keys alias WASD in fly-mode and preventDefault (no page scroll)", async () => {
  const dom = new FakeTarget();
  const win = (globalThis as unknown as { window: FakeTarget }).window;
  const { camera, controls } = await makeControls(dom);
  controls.setView(0, 15, 50, [0, 20, 0]);
  controls.update(1 / 60);
  const eye0 = camera.position.clone();

  // Hold right mouse (enter fly mode), then press ArrowUp — should translate the
  // eye forward exactly like KeyW, and preventDefault the arrow so the page
  // doesn't scroll.
  dom.dispatch("mousedown", { button: 2, buttons: 2, clientX: 600, clientY: 400, altKey: false });
  let prevented = false;
  win.dispatch("keydown", {
    code: "ArrowUp",
    target: { tagName: "CANVAS" },
    preventDefault: () => {
      prevented = true;
    },
  });
  for (let i = 0; i < 10; i++) controls.update(1 / 60);
  win.dispatch("keyup", { code: "ArrowUp", target: { tagName: "CANVAS" } });
  win.dispatch("mouseup", { button: 2, buttons: 0, clientX: 600, clientY: 400, altKey: false });

  expect(prevented).toBe(true); // arrow scroll suppressed
  const eyeMoved = camera.position.distanceTo(eye0);
  expect(eyeMoved).toBeGreaterThan(0.5); // flew forward
});

test("arrow keys are ignored while a text input is focused", async () => {
  const dom = new FakeTarget();
  const win = (globalThis as unknown as { window: FakeTarget }).window;
  // Focus a text input: isTextTarget() reads document.activeElement.tagName.
  (globalThis as unknown as { document: { activeElement: { tagName: string } } }).document.activeElement = {
    tagName: "INPUT",
  };
  const { camera, controls } = await makeControls(dom);
  controls.setView(0, 15, 50, [0, 20, 0]);
  controls.update(1 / 60);
  const eye0 = camera.position.clone();

  dom.dispatch("mousedown", { button: 2, buttons: 2, clientX: 600, clientY: 400, altKey: false });
  let prevented = false;
  win.dispatch("keydown", {
    code: "ArrowUp",
    target: { tagName: "INPUT" },
    preventDefault: () => {
      prevented = true;
    },
  });
  for (let i = 0; i < 10; i++) controls.update(1 / 60);
  win.dispatch("mouseup", { button: 2, buttons: 0, clientX: 600, clientY: 400, altKey: false });

  // Restore for other tests.
  (globalThis as unknown as { document: { activeElement: { tagName: string } } }).document.activeElement = {
    tagName: "CANVAS",
  };

  expect(prevented).toBe(false); // typing not stolen
  expect(camera.position.distanceTo(eye0)).toBeLessThan(1e-6); // no fly
});
