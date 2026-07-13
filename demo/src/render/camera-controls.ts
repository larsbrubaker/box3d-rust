// Orbit + fly camera, a faithful port of Box3D's sample camera
// (box3d-cpp-reference/samples/host/camera.cpp/h) adapted to drive a Three.js
// PerspectiveCamera. It replaces three's OrbitControls so the demo's mouse and
// keyboard interaction matches the C samples app exactly.
//
// Two modes share one state (yaw / pitch / radius / pivot). Sticky mouse-button
// + Alt-modifier flags select which mode each gesture drives:
//
//   ORBIT (Alt held)
//     Alt + left-drag    : orbit (yaw/pitch around pivot)      ORBIT_SENS
//     Alt + middle-drag  : pan pivot in view-space XY          PAN_SENS·radius
//     Alt + right-drag-Y : radial zoom (adjust radius)         RADIAL_ZOOM_SENS
//
//   FLY (right mouse held, no Alt)
//     right-drag         : FPS look (yaw/pitch the direction)  FLY_LOOK_SENS
//     WASD / arrows      : translate eye along forward/right   m_speed (m/s)
//     scroll             : tune m_speed                        FLY_SPEED_STEP
//
//   Always
//     bare scroll        : multiplicative zoom on radius       ZOOM_STEP
//
// The demos render in meters, Y-up — the same frame the C camera lives in once
// its sim->display render transform is identity — so this port drops that
// transform (and the third-person follow branch) and keeps the gesture state
// machine, sensitivities, and clamps bit-for-bit. Mouse events accumulate input
// deltas (camera.cpp OnEvent); update() consumes them and folds them into the
// camera state (camera.cpp Update), then positions the Three camera.

import * as THREE from "three";

// --- Constants (camera.cpp:9-21) --------------------------------------------
const DEG_TO_RAD = Math.PI / 180;
const HALF_PI = Math.PI * 0.5;
const ORBIT_SENS = 0.005; // radians per pixel
const FLY_LOOK_SENS = 0.005; // radians per pixel
const PAN_SENS = 0.005; // meters per pixel per meter of radius
const RADIAL_ZOOM_SENS = 0.02; // meters per pixel (alt+right-drag)
const FLY_SPEED_STEP = 1.0; // m/s per scroll tick
const ZOOM_STEP = 0.9; // multiplier per scroll tick
const MIN_DIST = 0.1;
const MIN_SPEED = 0.06; // matches box3d's 0.001*60
const MAX_SPEED = 30000.0; // matches box3d's 500*60
const PITCH_LIM = HALF_PI - 0.01;
// camera.h:50 kViewDistance: projection far plane + maximum orbit radius.
const VIEW_DISTANCE = 1000.0;

// b3ClampFloat (math_functions.h:177).
function clampFloat(a: number, lower: number, upper: number): number {
  return a < lower ? lower : upper < a ? upper : a;
}

// b3UnwindAngle = remainderf(radians, 2*PI) (math_functions.h:213). IEEE
// remainder: subtract 2π·n where n = radians/2π rounded to the nearest integer
// with ties going to the EVEN integer (round-half-to-even), exactly as C's
// remainderf does. JS `Math.round` breaks ties half-up, so a tie is detected and
// corrected below. Result lands in [-π, π]; keeps yaw bounded across long
// sessions.
function unwindAngle(radians: number): number {
  const twoPi = 2.0 * Math.PI;
  const q = radians / twoPi;
  const r = Math.round(q); // nearest, ties toward +∞
  // On an exact tie (r - q === 0.5) round to the even integer instead.
  const n = r - q === 0.5 && r % 2 !== 0 ? r - 1 : r;
  return radians - twoPi * n;
}

// Yaw/pitch -> the "+view-Z" direction (pivot -> camera), camera.cpp:35-39. The
// actual looking direction is -forward.
function forwardFromAngles(out: THREE.Vector3, yaw: number, pitch: number): THREE.Vector3 {
  const cp = Math.cos(pitch);
  return out.set(Math.sin(yaw) * cp, Math.sin(pitch), Math.cos(yaw) * cp);
}

const _forward = new THREE.Vector3();
const _worldUp = new THREE.Vector3(0, 1, 0);
const _eye = new THREE.Vector3();
const _eyeBefore = new THREE.Vector3();
const _off = new THREE.Vector3();
const _tmpR = new THREE.Vector3();

export class CameraControls {
  /** Look-at pivot (meters). Aliased as `target` for OrbitControls-compatible callers. */
  readonly pivot = new THREE.Vector3(0, 0, 0);
  yaw = 35.0 * DEG_TO_RAD; // radians, around Y (camera.cpp:75)
  pitch = -25.0 * DEG_TO_RAD; // radians, around camera-frame X (camera.cpp:76)
  radius = 25.0; // meters from pivot (camera.cpp:77)
  speed = 10.0; // fly-mode m/s (camera.cpp:82)

  /** When false, gestures are ignored but follow (external pivot moves) still tracks. */
  enabled = true;

  // Per-frame input deltas accumulated by the event handlers, zeroed by update
  // (camera.h:236-245).
  private orbitDX = 0;
  private orbitDY = 0;
  private panDX = 0;
  private panDY = 0;
  private radialZoomDY = 0;
  private scrollAccum = 0;
  private speedScrollAccum = 0;
  // Multiplicative radius change from a two-finger pinch (1 = none). Touch only;
  // see the touch handlers below.
  private pinchFactor = 1;

  // Sticky button/key state (camera.h:247-255).
  private leftDown = false;
  private rightDown = false;
  private middleDown = false;
  private altDown = false;
  private wDown = false;
  private aDown = false;
  private sDown = false;
  private dDown = false;

  // Cached basis from the previous frame; pan reads last frame's right/up so it
  // matches the screen the user dragged against (camera.cpp:481-483).
  private readonly right = new THREE.Vector3(1, 0, 0);
  private readonly up = new THREE.Vector3(0, 1, 0);

  // The eye we last wrote to the camera. If the camera moves out from under us
  // (a demo assigns camera.position directly), reconcile our spherical state
  // from it so external framing and follow cams keep working.
  private readonly lastEye = new THREE.Vector3();
  private lastNow = performance.now();

  // Last cursor position, for frame-to-frame pixel deltas (sokol mouse_dx/dy).
  // Computed from clientX/clientY rather than movementX/movementY so it is robust
  // across browsers and works without pointer lock.
  private lastX = 0;
  private lastY = 0;

  // Touch gesture baseline (previous centroid + pinch distance). The C sample app
  // has no touch spec — touch is a browser affordance layered on top, so these
  // fields and the touch handlers below never touch the ported orbit/fly math.
  private touchCount = 0;
  private lastTouchX = 0;
  private lastTouchY = 0;
  private lastPinchDist = 0;

  constructor(
    private readonly camera: THREE.PerspectiveCamera,
    private readonly dom: HTMLElement,
  ) {
    this.applyToCamera();
    this.addListeners();
  }

  /** OrbitControls-compatible alias so existing demos keep working unchanged. */
  get target(): THREE.Vector3 {
    return this.pivot;
  }

  get yawDeg(): number {
    return (this.yaw * 180) / Math.PI;
  }

  get pitchDeg(): number {
    return (this.pitch * 180) / Math.PI;
  }

  /** Box3D's sample signature: angles in degrees, pivot folded in (camera.cpp:195). */
  setView(
    yawDeg: number,
    pitchDeg: number,
    radius: number,
    target: [number, number, number] | THREE.Vector3,
  ): void {
    if (Array.isArray(target)) this.pivot.set(target[0], target[1], target[2]);
    else this.pivot.copy(target);
    this.yaw = yawDeg * DEG_TO_RAD;
    this.pitch = clampFloat(pitchDeg * DEG_TO_RAD, -PITCH_LIM, PITCH_LIM);
    this.radius = clampFloat(radius, MIN_DIST, VIEW_DISTANCE);
    this.applyToCamera();
  }

  /**
   * Consume accumulated input and fold it into the camera state, then reposition
   * the Three camera. Mirrors camera.cpp Update() (minus the third-person and
   * render-transform branches). `dt` defaults to the real frame delta, needed by
   * fly-mode WASD translation.
   */
  update(dt?: number): void {
    const now = performance.now();
    const frameDt = dt ?? clampFloat((now - this.lastNow) / 1000, 0, 0.1);
    this.lastNow = now;

    // Reconcile an external camera move (a demo set camera.position directly,
    // e.g. character/joints follow cams that also nudge the target). OrbitControls
    // derived its spherical state from position-target every frame; do the same
    // only when the camera actually drifted from our last write.
    if (this.camera.position.distanceToSquared(this.lastEye) > 1e-10) {
      _off.copy(this.camera.position).sub(this.pivot);
      const r = _off.length();
      if (r > 1e-6) {
        this.radius = clampFloat(r, MIN_DIST, VIEW_DISTANCE);
        this.pitch = clampFloat(Math.asin(clampFloat(_off.y / r, -1, 1)), -PITCH_LIM, PITCH_LIM);
        this.yaw = Math.atan2(_off.x, _off.z);
      }
    }

    if (this.enabled) {
      const flyMode = this.rightDown && !this.altDown;
      if (flyMode) {
        // Snapshot eye BEFORE rotating so yaw/pitch pivot around the eye (FPS)
        // instead of around the pivot; back-derive the pivot afterward so the eye
        // stays put regardless of look angle (camera.cpp:404-461). This must live
        // in its own scratch (_eyeBefore): forwardFromAngles writes the shared
        // _forward, and the post-rotation forward below reuses _forward — copying
        // out here keeps eyeBefore from being clobbered by that second call.
        const eyeBefore = _eyeBefore
          .copy(forwardFromAngles(_forward, this.yaw, this.pitch))
          .multiplyScalar(this.radius)
          .add(this.pivot);

        if (this.orbitDX !== 0 || this.orbitDY !== 0) {
          this.yaw -= this.orbitDX * FLY_LOOK_SENS;
          this.pitch += this.orbitDY * FLY_LOOK_SENS;
          this.pitch = clampFloat(this.pitch, -PITCH_LIM, PITCH_LIM);
        }

        const forward = forwardFromAngles(_forward, this.yaw, this.pitch);

        let wasdF = 0.0; // +forward = backwards, since forward = pivot->eye
        let wasdR = 0.0;
        if (this.wDown) wasdF -= 1.0;
        if (this.sDown) wasdF += 1.0;
        if (this.dDown) wasdR += 1.0;
        if (this.aDown) wasdR -= 1.0;

        _eye.copy(eyeBefore);
        if (wasdF !== 0.0 || wasdR !== 0.0) {
          // right = normalize(worldUp x forward), matching Box3D's UpdateTransform.
          const right = _tmpR.copy(_worldUp).cross(forward);
          const rlen = right.length();
          if (rlen > 1e-6) right.multiplyScalar(1 / rlen);
          const step = this.speed * frameDt;
          _eye.addScaledVector(forward, wasdF * step).addScaledVector(right, wasdR * step);
        }

        // Back-derive the pivot so a return to orbit preserves the look direction.
        this.pivot.copy(_eye).addScaledVector(forward, -this.radius);

        if (this.speedScrollAccum !== 0.0) {
          this.speed += this.speedScrollAccum * FLY_SPEED_STEP;
          this.speed = clampFloat(this.speed, MIN_SPEED, MAX_SPEED);
        }
      } else {
        // Orbit mode (camera.cpp:469-505).
        if (this.orbitDX !== 0 || this.orbitDY !== 0) {
          this.yaw -= this.orbitDX * ORBIT_SENS;
          this.pitch -= this.orbitDY * ORBIT_SENS;
          this.pitch = clampFloat(this.pitch, -PITCH_LIM, PITCH_LIM);
        }

        if (this.panDX !== 0 || this.panDY !== 0) {
          // Move pivot along last frame's view-space right/up (camera.cpp:479-490).
          const panScale = PAN_SENS * this.radius;
          this.pivot.addScaledVector(this.right, -this.panDX * panScale);
          this.pivot.addScaledVector(this.up, this.panDY * panScale);
        }

        if (this.radialZoomDY !== 0) {
          // Drag down -> zoom in (radius shrinks).
          this.radius -= RADIAL_ZOOM_SENS * this.radialZoomDY;
          this.radius = clampFloat(this.radius, MIN_DIST, VIEW_DISTANCE);
        }

        if (this.scrollAccum !== 0) {
          this.radius *= Math.pow(ZOOM_STEP, this.scrollAccum);
          this.radius = clampFloat(this.radius, MIN_DIST, VIEW_DISTANCE);
        }

        // Two-finger pinch (touch): scale the orbit radius directly by the ratio
        // of the previous to current finger spread, so spreading zooms in.
        if (this.pinchFactor !== 1) {
          this.radius *= this.pinchFactor;
          this.radius = clampFloat(this.radius, MIN_DIST, VIEW_DISTANCE);
        }
      }

      this.yaw = unwindAngle(this.yaw);
    }

    this.orbitDX = 0;
    this.orbitDY = 0;
    this.panDX = 0;
    this.panDY = 0;
    this.radialZoomDY = 0;
    this.scrollAccum = 0;
    this.speedScrollAccum = 0;
    this.pinchFactor = 1;

    this.applyToCamera();
  }

  // Refresh the cached basis and position the Three camera. forward is +view-Z
  // (pivot->eye); the look direction is -forward, so lookAt(pivot) is correct.
  private applyToCamera(): void {
    const forward = forwardFromAngles(_forward, this.yaw, this.pitch);
    // up = worldUp re-orthogonalized against forward; right = up x forward
    // (camera.cpp:50-52). Cached for the next frame's pan.
    this.up.copy(_worldUp).addScaledVector(forward, -_worldUp.dot(forward)).normalize();
    this.right.copy(this.up).cross(forward).normalize();

    _eye.copy(this.pivot).addScaledVector(forward, this.radius);
    this.camera.position.copy(_eye);
    this.camera.up.set(0, 1, 0);
    this.camera.lookAt(this.pivot);
    this.lastEye.copy(_eye);
  }

  // --- Event handling (camera.cpp OnEvent) ----------------------------------
  private addListeners(): void {
    this.dom.addEventListener("mousedown", this.onMouseDown);
    window.addEventListener("mousemove", this.onMouseMove);
    window.addEventListener("mouseup", this.onMouseUp);
    this.dom.addEventListener("wheel", this.onWheel, { passive: false });
    window.addEventListener("keydown", this.onKeyDown);
    window.addEventListener("keyup", this.onKeyUp);
    this.dom.addEventListener("contextmenu", this.onContextMenu);
    window.addEventListener("blur", this.onBlur);
    // Touch (non-passive: we preventDefault to own the gesture and suppress the
    // browser's compatibility mouse events / page scroll on the canvas).
    this.dom.addEventListener("touchstart", this.onTouchStart, { passive: false });
    this.dom.addEventListener("touchmove", this.onTouchMove, { passive: false });
    this.dom.addEventListener("touchend", this.onTouchEnd);
    this.dom.addEventListener("touchcancel", this.onTouchEnd);
  }

  private isTextTarget(): boolean {
    const el = document.activeElement as HTMLElement | null;
    const tag = el?.tagName;
    return tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT";
  }

  private onMouseDown = (e: MouseEvent): void => {
    if (e.button === 0) this.leftDown = true;
    else if (e.button === 2) this.rightDown = true;
    else if (e.button === 1) this.middleDown = true;
    this.altDown = e.altKey;
    this.lastX = e.clientX;
    this.lastY = e.clientY;
  };

  private onMouseUp = (e: MouseEvent): void => {
    if (e.button === 0) this.leftDown = false;
    else if (e.button === 2) this.rightDown = false;
    else if (e.button === 1) this.middleDown = false;
    this.altDown = e.altKey;
  };

  private onMouseMove = (e: MouseEvent): void => {
    this.altDown = e.altKey;
    // Frame-to-frame pixel delta (sokol mouse_dx/dy). Track even when disabled so
    // deltas stay correct across a disabled span.
    const dx = e.clientX - this.lastX;
    const dy = e.clientY - this.lastY;
    this.lastX = e.clientX;
    this.lastY = e.clientY;
    if (!this.enabled) return;
    // C keeps only keyboard mods; canOrbit is Alt held with no Ctrl/Shift
    // (camera.cpp:260-261).
    const canOrbit = e.altKey && !e.ctrlKey && !e.shiftKey;
    if (canOrbit && this.leftDown) {
      this.orbitDX += dx;
      this.orbitDY += dy;
    } else if (canOrbit && this.middleDown) {
      this.panDX += dx;
      this.panDY += dy;
    } else if (canOrbit && this.rightDown) {
      this.radialZoomDY += dy;
    } else if (!canOrbit && this.rightDown) {
      // Fly-look: routed into the orbit accumulators; update() reads them as a
      // look delta when rightDown && !altDown (camera.cpp:296-302).
      this.orbitDX += dx;
      this.orbitDY += dy;
    }
  };

  private onWheel = (e: WheelEvent): void => {
    e.preventDefault();
    // Normalize to ~1 tick per notch across deltaMode (pixel/line/page).
    let d = e.deltaY;
    if (e.deltaMode === 1) d *= 16;
    else if (e.deltaMode === 2) d *= 100;
    const ticks = -d / 100;
    const canOrbit = e.altKey && !e.ctrlKey && !e.shiftKey;
    if (!canOrbit && this.rightDown) this.speedScrollAccum += ticks;
    else this.scrollAccum += ticks;
  };

  private onKeyDown = (e: KeyboardEvent): void => {
    if (this.isTextTarget()) return;
    // Arrow keys alias WASD for fly-mode translation. They also scroll the page,
    // so preventDefault while we own them (the text-input guard above keeps the
    // arrows working normally inside form fields / range sliders).
    switch (e.code) {
      case "KeyW":
      case "ArrowUp":
        this.wDown = true;
        if (e.code === "ArrowUp") e.preventDefault();
        break;
      case "KeyA":
      case "ArrowLeft":
        this.aDown = true;
        if (e.code === "ArrowLeft") e.preventDefault();
        break;
      case "KeyS":
      case "ArrowDown":
        this.sDown = true;
        if (e.code === "ArrowDown") e.preventDefault();
        break;
      case "KeyD":
      case "ArrowRight":
        this.dDown = true;
        if (e.code === "ArrowRight") e.preventDefault();
        break;
      case "AltLeft":
      case "AltRight":
        this.altDown = true;
        break;
      default:
        break;
    }
  };

  private onKeyUp = (e: KeyboardEvent): void => {
    switch (e.code) {
      case "KeyW":
      case "ArrowUp":
        this.wDown = false;
        break;
      case "KeyA":
      case "ArrowLeft":
        this.aDown = false;
        break;
      case "KeyS":
      case "ArrowDown":
        this.sDown = false;
        break;
      case "KeyD":
      case "ArrowRight":
        this.dDown = false;
        break;
      case "AltLeft":
      case "AltRight":
        this.altDown = false;
        break;
      default:
        break;
    }
  };

  // --- Touch (browser affordance; C samples app is mouse/keyboard only) -------
  // Desktop mouse/keyboard behavior is unchanged. Gesture mapping:
  //   one finger drag   -> orbit  (feeds the orbit accumulators, like Alt+drag)
  //   two-finger pinch  -> zoom   (scales the orbit radius by the spread ratio)
  //   two-finger drag   -> pan    (moves the pivot in view-space, like Alt+middle)
  //   tap               -> select (handled by the pointer layer in interaction.ts)
  // Baselines re-sync on finger add/remove so lifting one finger of a pinch never
  // jerks the single-finger orbit.
  private touchCentroidX(touches: TouchList): number {
    return touches.length >= 2 ? (touches[0]!.clientX + touches[1]!.clientX) * 0.5 : touches[0]!.clientX;
  }

  private touchCentroidY(touches: TouchList): number {
    return touches.length >= 2 ? (touches[0]!.clientY + touches[1]!.clientY) * 0.5 : touches[0]!.clientY;
  }

  private pinchDistance(touches: TouchList): number {
    const dx = touches[0]!.clientX - touches[1]!.clientX;
    const dy = touches[0]!.clientY - touches[1]!.clientY;
    return Math.hypot(dx, dy);
  }

  private syncTouchBaseline(touches: TouchList): void {
    this.touchCount = touches.length;
    if (touches.length === 0) return;
    this.lastTouchX = this.touchCentroidX(touches);
    this.lastTouchY = this.touchCentroidY(touches);
    this.lastPinchDist = touches.length >= 2 ? this.pinchDistance(touches) : 0;
  }

  private onTouchStart = (e: TouchEvent): void => {
    // Own the gesture: suppress page scroll/zoom and compatibility mouse events.
    e.preventDefault();
    this.syncTouchBaseline(e.touches);
  };

  private onTouchMove = (e: TouchEvent): void => {
    e.preventDefault();
    const touches = e.touches;
    if (!this.enabled || touches.length === 0) {
      this.syncTouchBaseline(touches);
      return;
    }

    if (touches.length >= 2) {
      const cx = this.touchCentroidX(touches);
      const cy = this.touchCentroidY(touches);
      const dist = this.pinchDistance(touches);
      // Only fold deltas when the previous frame was also a 2-finger gesture,
      // else a 1->2 transition would inject a spurious jump.
      if (this.touchCount >= 2) {
        this.panDX += cx - this.lastTouchX;
        this.panDY += cy - this.lastTouchY;
        if (this.lastPinchDist > 0 && dist > 0) {
          // Spread fingers (dist grows) -> ratio < 1 -> radius shrinks -> zoom in.
          this.pinchFactor *= this.lastPinchDist / dist;
        }
      }
      this.lastTouchX = cx;
      this.lastTouchY = cy;
      this.lastPinchDist = dist;
      this.touchCount = touches.length;
    } else {
      const t = touches[0]!;
      if (this.touchCount === 1) {
        this.orbitDX += t.clientX - this.lastTouchX;
        this.orbitDY += t.clientY - this.lastTouchY;
      }
      this.lastTouchX = t.clientX;
      this.lastTouchY = t.clientY;
      this.lastPinchDist = 0;
      this.touchCount = 1;
    }
  };

  private onTouchEnd = (e: TouchEvent): void => {
    // Re-baseline against the fingers still down (2->1 keeps orbiting smoothly).
    this.syncTouchBaseline(e.touches);
  };

  private onContextMenu = (e: Event): void => {
    // Right-drag drives fly-look, so the browser menu must not pop.
    e.preventDefault();
  };

  // Drop every held input on focus loss (camera.cpp:356-367).
  private onBlur = (): void => {
    this.leftDown = false;
    this.rightDown = false;
    this.middleDown = false;
    this.wDown = false;
    this.aDown = false;
    this.sDown = false;
    this.dDown = false;
    this.altDown = false;
  };

  dispose(): void {
    this.dom.removeEventListener("mousedown", this.onMouseDown);
    window.removeEventListener("mousemove", this.onMouseMove);
    window.removeEventListener("mouseup", this.onMouseUp);
    this.dom.removeEventListener("wheel", this.onWheel);
    window.removeEventListener("keydown", this.onKeyDown);
    window.removeEventListener("keyup", this.onKeyUp);
    this.dom.removeEventListener("contextmenu", this.onContextMenu);
    window.removeEventListener("blur", this.onBlur);
    this.dom.removeEventListener("touchstart", this.onTouchStart);
    this.dom.removeEventListener("touchmove", this.onTouchMove);
    this.dom.removeEventListener("touchend", this.onTouchEnd);
    this.dom.removeEventListener("touchcancel", this.onTouchEnd);
  }
}
