// Camera gesture regression tests for NodeDesigner-style mouse-anchored
// navigation (camera-controls.ts), plus C-compat Alt+ gestures.

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
  getBoundingClientRect() {
    return { left: 0, top: 0, width: 800, height: 600, right: 800, bottom: 600 };
  }
  contains() {
    return true;
  }
  closest() {
    return null;
  }
}

beforeAll(() => {
  (globalThis as unknown as { window: FakeTarget }).window = new FakeTarget();
  (globalThis as unknown as { document: unknown }).document = {
    activeElement: { tagName: "CANVAS" },
  };
});

async function makeControls(dom: FakeTarget) {
  const { CameraControls } = await import("../src/render/camera-controls.ts");
  const camera = new THREE.PerspectiveCamera(60, 800 / 600, 0.1, 1000);
  camera.updateProjectionMatrix();
  const controls = new CameraControls(camera, dom as unknown as HTMLElement);
  return { camera, controls };
}

test("scroll zooms toward cursor: look direction fixed, pivot moves", async () => {
  const dom = new FakeTarget();
  const { camera, controls } = await makeControls(dom);
  controls.setView(0, 0, 50, [0, 0, 0]);
  controls.update(1 / 60);
  const look0 = controls.pivot.clone().sub(camera.position).normalize();
  const pivot0 = controls.pivot.clone();
  const radius0 = controls.radius;
  dom.dispatch("wheel", {
    clientX: 400,
    clientY: 300,
    deltaY: -100,
    deltaMode: 0,
    preventDefault() {},
    target: dom,
  });
  const look1 = controls.pivot.clone().sub(camera.position).normalize();
  expect(look1.dot(look0)).toBeGreaterThan(0.999);
  expect(Math.abs(controls.radius - radius0)).toBeLessThan(1e-3);
  expect(controls.pivot.distanceTo(pivot0)).toBeGreaterThan(0.5);
});

test("right-drag orbits around cursor anchor: eye moves, radius stable", async () => {
  const dom = new FakeTarget();
  const win = (globalThis as unknown as { window: FakeTarget }).window;
  const { camera, controls } = await makeControls(dom);
  controls.setView(0, 15, 50, [0, 20, 0]);
  controls.update(1 / 60);
  const eye0 = camera.position.clone();
  const radius0 = controls.radius;
  const yaw0 = controls.yawDeg;
  dom.dispatch("mousedown", {
    button: 2,
    buttons: 2,
    clientX: 600,
    clientY: 400,
    altKey: false,
    ctrlKey: false,
    shiftKey: false,
    target: dom,
  });
  let x = 600;
  let y = 400;
  for (let i = 0; i < 10; i++) {
    x += 20;
    y += 8;
    win.dispatch("mousemove", { button: 0, buttons: 2, clientX: x, clientY: y, altKey: false });
  }
  win.dispatch("mouseup", { button: 2, buttons: 0, clientX: x, clientY: y, altKey: false });
  expect(camera.position.distanceTo(eye0)).toBeGreaterThan(0.5);
  expect(Math.abs(controls.radius - radius0)).toBeLessThan(0.5);
  expect(controls.yawDeg).not.toBeCloseTo(yaw0, 0);
});

test("middle-drag pans pivot in the view plane", async () => {
  const dom = new FakeTarget();
  const win = (globalThis as unknown as { window: FakeTarget }).window;
  const { controls } = await makeControls(dom);
  controls.setView(0, 15, 50, [0, 20, 0]);
  controls.update(1 / 60);
  const pivot0 = controls.pivot.clone();
  dom.dispatch("mousedown", {
    button: 1,
    buttons: 4,
    clientX: 400,
    clientY: 300,
    altKey: false,
    ctrlKey: false,
    shiftKey: false,
    preventDefault() {},
    target: dom,
  });
  win.dispatch("mousemove", { button: 0, buttons: 4, clientX: 500, clientY: 300, altKey: false });
  win.dispatch("mouseup", { button: 1, buttons: 0, clientX: 500, clientY: 300, altKey: false });
  expect(controls.pivot.distanceTo(pivot0)).toBeGreaterThan(0.1);
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
  dom.dispatch("mousedown", {
    button: 2,
    buttons: 2,
    clientX: 600,
    clientY: 400,
    altKey: true,
    ctrlKey: false,
    shiftKey: false,
    target: dom,
  });
  let y = 400;
  for (let i = 0; i < 10; i++) {
    y -= 10;
    win.dispatch("mousemove", { button: 0, buttons: 2, clientX: 600, clientY: y, altKey: true });
    controls.update(1 / 60);
  }
  win.dispatch("mouseup", { button: 2, buttons: 0, clientX: 600, clientY: y, altKey: true });
  expect(controls.radius).toBeGreaterThan(radius0 + 0.5);
  expect(Math.abs(controls.yawDeg - yaw0)).toBeLessThan(1e-3);
  expect(Math.abs(controls.pitchDeg - pitch0)).toBeLessThan(1e-3);
});

test("arrow keys move the camera and preventDefault (no page scroll)", async () => {
  const dom = new FakeTarget();
  const win = (globalThis as unknown as { window: FakeTarget }).window;
  const { camera, controls } = await makeControls(dom);
  controls.setView(0, 15, 50, [0, 20, 0]);
  controls.update(1 / 60);
  const eye0 = camera.position.clone();
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
  expect(prevented).toBe(true);
  expect(camera.position.distanceTo(eye0)).toBeGreaterThan(0.5);
});

test("arrow keys are ignored while a text input is focused", async () => {
  const dom = new FakeTarget();
  const win = (globalThis as unknown as { window: FakeTarget }).window;
  (globalThis as unknown as { document: { activeElement: { tagName: string } } }).document.activeElement = {
    tagName: "INPUT",
  };
  const { camera, controls } = await makeControls(dom);
  controls.setView(0, 15, 50, [0, 20, 0]);
  controls.update(1 / 60);
  const eye0 = camera.position.clone();
  let prevented = false;
  win.dispatch("keydown", {
    code: "ArrowUp",
    target: { tagName: "INPUT" },
    preventDefault: () => {
      prevented = true;
    },
  });
  for (let i = 0; i < 10; i++) controls.update(1 / 60);
  (globalThis as unknown as { document: { activeElement: { tagName: string } } }).document.activeElement = {
    tagName: "CANVAS",
  };
  expect(prevented).toBe(false);
  expect(camera.position.distanceTo(eye0)).toBeLessThan(1e-6);
});
