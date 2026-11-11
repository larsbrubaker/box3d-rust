// Third-person input + camera rig for the Character scenes that read C
// `Sample::GetMoverInput` (Geometric Mover, Dynamic Mover). It mirrors the C host:
//
// - keys: the `IsKeyDown` table (WASD, Space, Shift, Ctrl) goes to wasm every frame;
//   the Rust `GetMoverInput` port ignores it unless the camera is in third person, so
//   the free camera keeps WASD (C: "active only in third person so the free camera
//   keeps its keys").
// - look: `Sample::MouseMove` third-person deltas (0.1°/px, yaw at twice the pitch
//   rate, pitch clamped to ±85°) from a locked pointer. C calls `sapp_lock_mouse` when
//   third person turns on; browsers only allow pointer lock from a user gesture, so the
//   lock is requested on T / the checkbox and on a canvas click, and Esc releases it.
//   Without a lock, left-drag looks instead.
// - zoom: the C third-person camera's only camera input is the wheel on the radius
//   (`m_radius -= scroll`, floor `THIRD_PERSON_MIN_RADIUS = 0.25`).
// - placement: the page renders from the pivot / radius the scene leaves on the camera.
// Arrow keys alias WASD, and C aliases Ctrl for walking because Ctrl+W closes the tab
// in Windows / Linux browsers.

import type { Box3dWasm } from "../wasm.ts";
import type { DemoScene } from "../three-scene.ts";

export interface RigHost {
  wasm: Box3dWasm;
  demo: DemoScene;
  canvas: HTMLCanvasElement;
  canvasArea: HTMLElement | null;
  keys: Set<string>;
  isActive(): boolean;
  isThirdPerson(): boolean;
  /** Extra prompt text while the mouse is locked (scene-specific bindings). */
  lockedPrompt: string;
  /** Whether a plain left click may (re)capture the mouse right now. */
  canLockOnClick?(): boolean;
}

export interface ThirdPersonRig {
  setVisible(visible: boolean): void;
  /** The wheel-zoom radius (C `m_camera->m_radius`). */
  radius(): number;
  setRadius(r: number): void;
  isLocked(): boolean;
  onThirdPerson(thirdPerson: boolean): void;
  /** Feed keys + camera to wasm; returns the frame dt used. */
  beforeStep(): number;
  /** Render from `pivot` / `radius` in third person; free orbit camera otherwise. */
  place(pivot: [number, number, number] | null, radius: number): void;
  dispose(): void;
}

const DEG = Math.PI / 180;
const LOOK_SENS = 0.1 * DEG;
const PITCH_LIMIT = 85 * DEG;
const MIN_RADIUS = 0.25;

export function createThirdPersonRig(host: RigHost): ThirdPersonRig {
  const { wasm, demo, canvas } = host;
  let savedRadius = 6;
  let pointerLocked = false;
  let lastNow = performance.now();
  let visible = false;

  const prompt = document.createElement("div");
  prompt.className = "sample-draw-text";
  Object.assign(prompt.style, {
    bottom: "0.6em",
    top: "auto",
    left: "50%",
    transform: "translateX(-50%)",
    pointerEvents: "none",
    display: "none",
  });
  host.canvasArea?.appendChild(prompt);

  const active = () => visible && host.isActive();

  function updatePrompt() {
    if (!active()) {
      prompt.style.display = "none";
      return;
    }
    prompt.style.display = "";
    if (!host.isThirdPerson()) {
      prompt.textContent = "Free camera (orbit / WASD fly) — press T to drive the mover";
    } else {
      prompt.textContent = pointerLocked
        ? `Mouse look on — ${host.lockedPrompt}, Esc releases the mouse`
        : "Click the canvas to play (locks the mouse for look; Esc releases)";
    }
  }

  function requestLock() {
    try {
      const r = canvas.requestPointerLock?.() as unknown;
      if (r instanceof Promise) r.catch(() => {});
    } catch {
      /* pointer lock unavailable: left-drag looks instead */
    }
  }

  const onMouseDown = (e: MouseEvent) => {
    if (!active() || !host.isThirdPerson()) return;
    canvas.focus();
    const may = host.canLockOnClick ? host.canLockOnClick() : true;
    if (e.button === 0 && !pointerLocked && may) requestLock();
  };
  const onMouseMove = (e: MouseEvent) => {
    if (!active() || !host.isThirdPerson()) return;
    if (!pointerLocked && !(e.buttons & 1 && e.target === canvas)) return;
    // C Sample::MouseMove: m_yaw -= 2 * sens * dx; m_pitch += sens * dy; clamp ±85°.
    const c = demo.controls;
    c.yaw -= 2 * LOOK_SENS * e.movementX;
    c.pitch += LOOK_SENS * e.movementY;
    if (c.pitch > PITCH_LIMIT) c.pitch = PITCH_LIMIT;
    else if (c.pitch < -PITCH_LIMIT) c.pitch = -PITCH_LIMIT;
  };
  const onWheel = (e: WheelEvent) => {
    if (!active() || !host.isThirdPerson()) return;
    let d = e.deltaY;
    if (e.deltaMode === 1) d *= 33;
    else if (e.deltaMode === 2) d *= 100;
    // One wheel notch (~100 px) is one C scroll tick: m_radius -= scrollY.
    savedRadius = Math.max(MIN_RADIUS, savedRadius + d / 100);
    e.preventDefault();
  };
  const onKeyDown = (e: KeyboardEvent) => {
    if (!active()) return;
    const tag = (e.target as HTMLElement)?.tagName;
    if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
    // Keep Ctrl+S / Ctrl+D / Ctrl+A (walk + move) from triggering browser shortcuts.
    if (e.ctrlKey && ["KeyW", "KeyA", "KeyS", "KeyD", "Space"].includes(e.code)) e.preventDefault();
  };
  const onLockChange = () => {
    pointerLocked = document.pointerLockElement === canvas;
    updatePrompt();
  };

  canvas.addEventListener("mousedown", onMouseDown);
  document.addEventListener("mousemove", onMouseMove);
  canvas.addEventListener("wheel", onWheel, { passive: false, capture: true });
  window.addEventListener("keydown", onKeyDown);
  document.addEventListener("pointerlockchange", onLockChange);

  function setVisible(v: boolean) {
    visible = v;
    if (!v) {
      demo.controls.enabled = true;
      if (pointerLocked) document.exitPointerLock?.();
    }
    lastNow = performance.now();
    updatePrompt();
  }

  function onThirdPerson(thirdPerson: boolean) {
    if (!active()) return;
    if (thirdPerson) {
      // Entering third person keeps the orbit camera's radius, like the C camera.
      savedRadius = Math.max(MIN_RADIUS, demo.controls.radius);
      requestLock();
    } else {
      if (pointerLocked) document.exitPointerLock?.();
      demo.controls.enabled = true;
    }
    updatePrompt();
  }

  function beforeStep(): number {
    const now = performance.now();
    const dt = Math.min(0.1, Math.max(0, (now - lastNow) / 1000));
    lastNow = now;
    const k = host.keys;
    wasm.character_set_keys(
      k.has("KeyW") || k.has("ArrowUp"),
      k.has("KeyA") || k.has("ArrowLeft"),
      k.has("KeyS") || k.has("ArrowDown"),
      k.has("KeyD") || k.has("ArrowRight"),
      k.has("Space"),
      k.has("ShiftLeft") || k.has("ShiftRight"),
      k.has("ControlLeft") || k.has("ControlRight") || k.has("KeyC"),
    );
    const radius = host.isThirdPerson() ? savedRadius : demo.controls.radius;
    wasm.character_set_camera(demo.controls.yaw, demo.controls.pitch, radius, dt);
    return dt;
  }

  function place(pivot: [number, number, number] | null, radius: number) {
    if (host.isThirdPerson() && pivot) {
      demo.controls.enabled = false;
      demo.controls.pivot.set(pivot[0], pivot[1], pivot[2]);
      demo.controls.radius = radius;
    } else {
      demo.controls.enabled = true;
    }
  }

  function dispose() {
    canvas.removeEventListener("mousedown", onMouseDown);
    document.removeEventListener("mousemove", onMouseMove);
    canvas.removeEventListener("wheel", onWheel, { capture: true });
    window.removeEventListener("keydown", onKeyDown);
    document.removeEventListener("pointerlockchange", onLockChange);
    if (pointerLocked) document.exitPointerLock?.();
    demo.controls.enabled = true;
    prompt.remove();
  }

  return {
    setVisible,
    radius: () => savedRadius,
    setRadius: (r: number) => {
      savedRadius = Math.max(MIN_RADIUS, r);
    },
    isLocked: () => pointerLocked,
    onThirdPerson,
    beforeStep,
    place,
    dispose,
  };
}
