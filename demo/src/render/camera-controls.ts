// Camera for the Box3D demo samples. Spherical state (yaw / pitch / radius /
// pivot) still matches the C sample camera (camera.cpp), but primary mouse
// navigation mirrors NodeDesigner / MatterCAD mouse-anchored controls
// (FDS/NodeDesigner static/js/node-editor/rendering/camera-operations.js +
// 3d-controls.js):
//
//   PRIMARY (NodeDesigner-style, no modifier)
//     scroll             : zoom toward the world point under the cursor
//     right-drag         : orbit around the world point under the cursor
//     middle-drag        : pan in the view plane through that point
//
//   C-compat (Alt held) — same sensitivities as camera.cpp
//     Alt + left-drag    : orbit around the fixed pivot               ORBIT_SENS
//     Alt + middle-drag  : pan pivot in view-space XY                 PAN_SENS·radius
//     Alt + right-drag-Y : radial zoom (adjust radius)                RADIAL_ZOOM_SENS
//
//   KEYBOARD
//     WASD / arrows      : translate eye along forward/right          m_speed (m/s)
//
// Y-up meters (Box3D). NodeDesigner is Z-up; turntable yaw uses world +Y here
// instead of +Z. Mesh picking uses optional pick roots (content/dynamic);
// fallbacks match ND: previous-hit plane → ground (Y=0) → pivot plane.

import * as THREE from "three";

const DEG_TO_RAD = Math.PI / 180;
const HALF_PI = Math.PI / 2;
const ORBIT_SENS = 0.005;
const PAN_SENS = 0.005;
const RADIAL_ZOOM_SENS = 0.02;
const ZOOM_EXP = 0.1; // NodeDesigner onWheel: Math.exp(±zoomSpeed)
const MIN_DIST = 0.1;
const PITCH_LIM = HALF_PI - 0.01;
const VIEW_DISTANCE = 1000.0;

function clampFloat(a: number, lower: number, upper: number): number {
  return a < lower ? lower : upper < a ? upper : a;
}

function unwindAngle(radians: number): number {
  const twoPi = 2.0 * Math.PI;
  const q = radians / twoPi;
  const r = Math.round(q);
  const n = r - q === 0.5 && r % 2 !== 0 ? r - 1 : r;
  return radians - twoPi * n;
}

function forwardFromAngles(out: THREE.Vector3, yaw: number, pitch: number): THREE.Vector3 {
  const cp = Math.cos(pitch);
  return out.set(Math.sin(yaw) * cp, Math.sin(pitch), Math.cos(yaw) * cp);
}

const _forward = new THREE.Vector3();
const _worldUp = new THREE.Vector3(0, 1, 0);
const _eye = new THREE.Vector3();
const _off = new THREE.Vector3();
const _tmpR = new THREE.Vector3();
const _tmpV = new THREE.Vector3();
const _tmpV2 = new THREE.Vector3();
const _tmpV3 = new THREE.Vector3();
const _ndc = new THREE.Vector2();
const _quat = new THREE.Quaternion();
const _quatZ = new THREE.Quaternion();
const _quatX = new THREE.Quaternion();
const _raycaster = new THREE.Raycaster();
const _groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
const _hitPlaneScratch = new THREE.Plane();

type MouseNav = "none" | "orbit_anchor" | "pan_plane" | "alt_orbit" | "alt_pan" | "alt_zoom";

export class CameraControls {
  readonly pivot = new THREE.Vector3(0, 0, 0);
  yaw = 35.0 * DEG_TO_RAD;
  pitch = -25.0 * DEG_TO_RAD;
  radius = 25.0;
  speed = 10.0;
  enabled = true;

  private pickRoots: THREE.Object3D[] = [];
  private orbitDX = 0;
  private orbitDY = 0;
  private panDX = 0;
  private panDY = 0;
  private radialZoomDY = 0;
  private pinchFactor = 1;
  private leftDown = false;
  private rightDown = false;
  private middleDown = false;
  private altDown = false;
  private wDown = false;
  private aDown = false;
  private sDown = false;
  private dDown = false;
  private readonly right = new THREE.Vector3(1, 0, 0);
  private readonly up = new THREE.Vector3(0, 1, 0);
  private readonly lastEye = new THREE.Vector3();
  private lastNow = performance.now();
  private lastX = 0;
  private lastY = 0;
  private mouseNav: MouseNav = "none";
  private readonly mouseDownPos = new THREE.Vector2();
  private readonly curMousePos = new THREE.Vector2();
  private readonly rotationAnchor = new THREE.Vector3();
  private readonly mouseDownWorld = new THREE.Vector3();
  private readonly previousHitPoint = new THREE.Vector3();
  private hasPreviousHit = false;
  private readonly hitPlane = new THREE.Plane();
  private readonly panStartEye = new THREE.Vector3();
  private readonly panStartPivot = new THREE.Vector3();
  private readonly panStartCam = new THREE.PerspectiveCamera();
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

  setPickRoots(roots: THREE.Object3D[]): void {
    this.pickRoots = roots;
  }

  get target(): THREE.Vector3 {
    return this.pivot;
  }

  get yawDeg(): number {
    return (this.yaw * 180) / Math.PI;
  }

  get pitchDeg(): number {
    return (this.pitch * 180) / Math.PI;
  }

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

  update(dt?: number): void {
    const now = performance.now();
    const frameDt = dt ?? clampFloat((now - this.lastNow) / 1000, 0, 0.1);
    this.lastNow = now;

    if (this.camera.position.distanceToSquared(this.lastEye) > 1e-10) {
      this.syncSphericalFromEye();
    }

    if (this.enabled) {
      let wasdF = 0.0;
      let wasdR = 0.0;
      if (this.wDown) wasdF -= 1.0;
      if (this.sDown) wasdF += 1.0;
      if (this.dDown) wasdR += 1.0;
      if (this.aDown) wasdR -= 1.0;

      if (wasdF !== 0.0 || wasdR !== 0.0) {
        const forward = forwardFromAngles(_forward, this.yaw, this.pitch);
        const right = _tmpR.copy(_worldUp).cross(forward);
        const rlen = right.length();
        if (rlen > 1e-6) right.multiplyScalar(1 / rlen);
        const step = this.speed * frameDt;
        _eye.copy(this.camera.position);
        _eye.addScaledVector(forward, wasdF * step).addScaledVector(right, wasdR * step);
        this.pivot.add(_tmpV.copy(_eye).sub(this.camera.position));
        this.camera.position.copy(_eye);
        this.syncSphericalFromEye();
      }

      if (this.orbitDX !== 0 || this.orbitDY !== 0) {
        this.yaw -= this.orbitDX * ORBIT_SENS;
        this.pitch -= this.orbitDY * ORBIT_SENS;
        this.pitch = clampFloat(this.pitch, -PITCH_LIM, PITCH_LIM);
      }

      if (this.panDX !== 0 || this.panDY !== 0) {
        const panScale = PAN_SENS * this.radius;
        this.pivot.addScaledVector(this.right, -this.panDX * panScale);
        this.pivot.addScaledVector(this.up, this.panDY * panScale);
      }

      if (this.radialZoomDY !== 0) {
        this.radius -= RADIAL_ZOOM_SENS * this.radialZoomDY;
        this.radius = clampFloat(this.radius, MIN_DIST, VIEW_DISTANCE);
      }

      if (this.pinchFactor !== 1) {
        this.radius *= this.pinchFactor;
        this.radius = clampFloat(this.radius, MIN_DIST, VIEW_DISTANCE);
      }

      this.yaw = unwindAngle(this.yaw);
    }

    this.orbitDX = 0;
    this.orbitDY = 0;
    this.panDX = 0;
    this.panDY = 0;
    this.radialZoomDY = 0;
    this.pinchFactor = 1;
    this.applyToCamera();
  }

  private applyToCamera(): void {
    const forward = forwardFromAngles(_forward, this.yaw, this.pitch);
    this.up.copy(_worldUp).addScaledVector(forward, -_worldUp.dot(forward)).normalize();
    this.right.copy(this.up).cross(forward).normalize();
    _eye.copy(this.pivot).addScaledVector(forward, this.radius);
    this.camera.position.copy(_eye);
    this.camera.up.set(0, 1, 0);
    this.camera.lookAt(this.pivot);
    this.lastEye.copy(_eye);
  }

  private syncSphericalFromEye(): void {
    _off.copy(this.camera.position).sub(this.pivot);
    const r = _off.length();
    if (r > 1e-6) {
      this.radius = clampFloat(r, MIN_DIST, VIEW_DISTANCE);
      this.pitch = clampFloat(Math.asin(clampFloat(_off.y / r, -1, 1)), -PITCH_LIM, PITCH_LIM);
      this.yaw = Math.atan2(_off.x, _off.z);
    }
  }

  private clientToNdc(clientX: number, clientY: number): void {
    const rect = this.dom.getBoundingClientRect();
    const w = Math.max(rect.width, 1);
    const h = Math.max(rect.height, 1);
    _ndc.set(((clientX - rect.left) / w) * 2 - 1, -((clientY - rect.top) / h) * 2 + 1);
  }

  private setRayFromClient(clientX: number, clientY: number, cam: THREE.Camera = this.camera): void {
    this.clientToNdc(clientX, clientY);
    _raycaster.setFromCamera(_ndc, cam);
  }

  /** NodeDesigner findIntersectionPoint, adapted to Y-up (ground = XZ at Y=0). */
  private findIntersectionPoint(clientX: number, clientY: number): THREE.Vector3 {
    this.setRayFromClient(clientX, clientY);
    const out = new THREE.Vector3();

    if (this.pickRoots.length > 0) {
      const hits = _raycaster.intersectObjects(this.pickRoots, true);
      if (hits.length > 0) {
        out.copy(hits[0]!.point);
        this.previousHitPoint.copy(out);
        this.hasPreviousHit = true;
        return out;
      }
    }

    if (this.hasPreviousHit) {
      const viewDir = _tmpV.copy(this.camera.position).sub(this.previousHitPoint).normalize();
      _hitPlaneScratch.setFromNormalAndCoplanarPoint(viewDir, this.previousHitPoint);
      if (_raycaster.ray.intersectPlane(_hitPlaneScratch, out)) return out;
    }

    // Reject coplanar ground hits (pitch≈0): Three returns ray origin → zoom no-op.
    if (
      Math.abs(_raycaster.ray.direction.y) > 1e-4 &&
      _raycaster.ray.intersectPlane(_groundPlane, out)
    ) {
      const t = _tmpV.copy(out).sub(_raycaster.ray.origin).dot(_raycaster.ray.direction);
      if (t > MIN_DIST) {
        const viewDir = _tmpV.copy(this.camera.position).sub(out).normalize();
        _hitPlaneScratch.setFromNormalAndCoplanarPoint(viewDir, out);
        if (_raycaster.ray.intersectPlane(_hitPlaneScratch, _tmpV2)) return _tmpV2.clone();
        return out.clone();
      }
    }

    const viewDir = _tmpV.copy(this.camera.position).sub(this.pivot).normalize();
    _hitPlaneScratch.setFromNormalAndCoplanarPoint(viewDir, this.pivot);
    if (_raycaster.ray.intersectPlane(_hitPlaneScratch, out)) return out;

    const distance = this.camera.position.distanceTo(this.pivot);
    return _raycaster.ray.origin.clone().addScaledVector(_raycaster.ray.direction, distance);
  }

  /** NodeDesigner zoomToCursor: slide eye along eye→point, keep look vector. */
  private zoomToCursor(worldPosition: THREE.Vector3, zoomFactor: number): void {
    const eye = this.camera.position;
    const toPoint = _tmpV.copy(worldPosition).sub(eye);
    const dist = toPoint.length();
    if (dist < 1e-8) return;
    const newDist = dist * zoomFactor;
    if (newDist < MIN_DIST && zoomFactor < 1) return;
    const move = dist - newDist;
    const dir = toPoint.multiplyScalar(1 / dist);
    const eyeMove = _tmpV2.copy(dir).multiplyScalar(move);
    const look = _tmpV3.copy(this.pivot).sub(eye);
    eye.add(eyeMove);
    this.pivot.copy(eye).add(look);
    this.syncSphericalFromEye();
    this.radius = clampFloat(this.radius, MIN_DIST, VIEW_DISTANCE);
    this.applyToCamera();
  }

  /** NodeDesigner applyOrbitDrag / turntable around cursor anchor (Y-up). */
  private applyOrbitAroundAnchor(deltaX: number, deltaY: number, anchor: THREE.Vector3): void {
    const rotYaw = -deltaX * ORBIT_SENS;
    const rotPitch = -deltaY * ORBIT_SENS;
    _quatZ.setFromAxisAngle(_worldUp, rotYaw);
    const lookDir = _tmpV.copy(this.pivot).sub(this.camera.position).normalize();
    const camRight = _tmpR.copy(lookDir).cross(_worldUp).normalize();
    if (camRight.lengthSq() < 1e-8) return;
    const elevation = Math.asin(clampFloat(lookDir.y, -1, 1));
    const maxEl = HALF_PI - 0.1;
    let clampedPitch = rotPitch;
    if ((elevation > maxEl && rotPitch > 0) || (elevation < -maxEl && rotPitch < 0)) {
      clampedPitch = 0;
    }
    _quatX.setFromAxisAngle(camRight, clampedPitch);
    _quat.copy(_quatZ).multiply(_quatX);
    _tmpV.copy(this.camera.position).sub(anchor).applyQuaternion(_quat);
    this.camera.position.copy(anchor).add(_tmpV);
    _tmpV.copy(this.pivot).sub(anchor).applyQuaternion(_quat);
    this.pivot.copy(anchor).add(_tmpV);
    this.syncSphericalFromEye();
    this.pitch = clampFloat(this.pitch, -PITCH_LIM, PITCH_LIM);
    this.yaw = unwindAngle(this.yaw);
    this.applyToCamera();
  }

  /** NodeDesigner panAlongHitPlane. */
  private panAlongHitPlane(): void {
    this.clientToNdc(this.curMousePos.x, this.curMousePos.y);
    this.panStartCam.position.copy(this.panStartEye);
    this.panStartCam.up.copy(this.camera.up);
    this.panStartCam.lookAt(this.panStartPivot);
    this.panStartCam.aspect = this.camera.aspect;
    this.panStartCam.fov = this.camera.fov;
    this.panStartCam.near = this.camera.near;
    this.panStartCam.far = this.camera.far;
    this.panStartCam.updateProjectionMatrix();
    this.panStartCam.updateMatrixWorld();
    _raycaster.setFromCamera(_ndc, this.panStartCam);
    if (!_raycaster.ray.intersectPlane(this.hitPlane, _tmpV)) return;
    const offset = _tmpV2.copy(_tmpV).sub(this.mouseDownWorld);
    this.camera.position.copy(this.panStartEye).sub(offset);
    this.pivot.copy(this.panStartPivot).sub(offset);
    this.syncSphericalFromEye();
    this.applyToCamera();
  }

  private addListeners(): void {
    this.dom.addEventListener("mousedown", this.onMouseDown);
    window.addEventListener("mousemove", this.onMouseMove);
    window.addEventListener("mouseup", this.onMouseUp);
    this.dom.addEventListener("wheel", this.onWheel, { passive: false });
    window.addEventListener("keydown", this.onKeyDown);
    window.addEventListener("keyup", this.onKeyUp);
    this.dom.addEventListener("contextmenu", this.onContextMenu);
    this.dom.addEventListener("auxclick", this.onAuxClick);
    window.addEventListener("blur", this.onBlur);
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

  private isUiTarget(e: Event): boolean {
    const t = e.target as HTMLElement | null;
    if (!t || typeof t.closest !== "function") return false;
    if (t === this.dom || this.dom.contains(t)) return false;
    return !!t.closest(
      ".demo-controls, .menu-bar, .sample-metrics, .tree-nav, button, input, select, textarea, a",
    );
  }

  private onMouseDown = (e: MouseEvent): void => {
    if (this.isUiTarget(e)) return;
    if (e.button === 0) this.leftDown = true;
    else if (e.button === 2) this.rightDown = true;
    else if (e.button === 1) {
      this.middleDown = true;
      e.preventDefault();
    }
    this.altDown = e.altKey;
    this.lastX = e.clientX;
    this.lastY = e.clientY;
    this.mouseDownPos.set(e.clientX, e.clientY);
    this.curMousePos.copy(this.mouseDownPos);
    if (!this.enabled) return;

    const canAlt = e.altKey && !e.ctrlKey && !e.shiftKey;
    if (canAlt && e.button === 0) {
      this.mouseNav = "alt_orbit";
      return;
    }
    if (canAlt && e.button === 1) {
      this.mouseNav = "alt_pan";
      return;
    }
    if (canAlt && e.button === 2) {
      this.mouseNav = "alt_zoom";
      return;
    }

    if (e.button === 2) {
      this.mouseNav = "orbit_anchor";
      this.rotationAnchor.copy(this.findIntersectionPoint(e.clientX, e.clientY));
    } else if (e.button === 1) {
      this.mouseNav = "pan_plane";
      this.mouseDownWorld.copy(this.findIntersectionPoint(e.clientX, e.clientY));
      const viewDir = _tmpV.copy(this.pivot).sub(this.camera.position).normalize();
      this.hitPlane.setFromNormalAndCoplanarPoint(viewDir, this.mouseDownWorld);
      this.panStartEye.copy(this.camera.position);
      this.panStartPivot.copy(this.pivot);
    } else {
      this.mouseNav = "none";
    }
  };

  private onMouseUp = (e: MouseEvent): void => {
    if (e.button === 0) this.leftDown = false;
    else if (e.button === 2) this.rightDown = false;
    else if (e.button === 1) this.middleDown = false;
    this.altDown = e.altKey;
    if (e.button === 0 || e.button === 1 || e.button === 2) this.mouseNav = "none";
  };

  private onMouseMove = (e: MouseEvent): void => {
    this.altDown = e.altKey;
    const dx = e.clientX - this.lastX;
    const dy = e.clientY - this.lastY;
    this.lastX = e.clientX;
    this.lastY = e.clientY;
    this.curMousePos.set(e.clientX, e.clientY);
    if (!this.enabled) return;

    if (this.mouseNav !== "none" && e.buttons === 0) {
      this.mouseNav = "none";
      this.leftDown = false;
      this.rightDown = false;
      this.middleDown = false;
      return;
    }

    if (this.mouseNav === "orbit_anchor") {
      this.applyOrbitAroundAnchor(dx, dy, this.rotationAnchor);
      return;
    }
    if (this.mouseNav === "pan_plane") {
      this.panAlongHitPlane();
      return;
    }
    if (this.mouseNav === "alt_orbit" && this.leftDown) {
      this.orbitDX += dx;
      this.orbitDY += dy;
    } else if (this.mouseNav === "alt_pan" && this.middleDown) {
      this.panDX += dx;
      this.panDY += dy;
    } else if (this.mouseNav === "alt_zoom" && this.rightDown) {
      this.radialZoomDY += dy;
    }
  };

  private onWheel = (e: WheelEvent): void => {
    if (this.isUiTarget(e)) return;
    e.preventDefault();
    if (!this.enabled) return;
    let d = e.deltaY;
    if (e.deltaMode === 1) d *= 16;
    else if (e.deltaMode === 2) d *= 100;
    const steps = Math.max(1, Math.round(Math.abs(d) / 100));
    const zoomFactor = Math.exp(d > 0 ? ZOOM_EXP * steps : -ZOOM_EXP * steps);
    this.zoomToCursor(this.findIntersectionPoint(e.clientX, e.clientY), zoomFactor);
  };

  private onKeyDown = (e: KeyboardEvent): void => {
    if (this.isTextTarget()) return;
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
    e.preventDefault();
    this.syncTouchBaseline(e.touches);
    if (e.touches.length === 1) {
      const t = e.touches[0]!;
      this.mouseNav = "orbit_anchor";
      this.rotationAnchor.copy(this.findIntersectionPoint(t.clientX, t.clientY));
    } else if (e.touches.length >= 2) {
      this.mouseNav = "pan_plane";
      const cx = this.touchCentroidX(e.touches);
      const cy = this.touchCentroidY(e.touches);
      this.mouseDownWorld.copy(this.findIntersectionPoint(cx, cy));
      const viewDir = _tmpV.copy(this.pivot).sub(this.camera.position).normalize();
      this.hitPlane.setFromNormalAndCoplanarPoint(viewDir, this.mouseDownWorld);
      this.panStartEye.copy(this.camera.position);
      this.panStartPivot.copy(this.pivot);
      this.curMousePos.set(cx, cy);
    }
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
      if (this.touchCount >= 2) {
        this.curMousePos.set(cx, cy);
        this.panAlongHitPlane();
        if (this.lastPinchDist > 0 && dist > 0) {
          this.zoomToCursor(this.findIntersectionPoint(cx, cy), this.lastPinchDist / dist);
        }
      }
      this.lastTouchX = cx;
      this.lastTouchY = cy;
      this.lastPinchDist = dist;
      this.touchCount = touches.length;
    } else {
      const t = touches[0]!;
      if (this.touchCount === 1 && this.mouseNav === "orbit_anchor") {
        this.applyOrbitAroundAnchor(t.clientX - this.lastTouchX, t.clientY - this.lastTouchY, this.rotationAnchor);
      }
      this.lastTouchX = t.clientX;
      this.lastTouchY = t.clientY;
      this.lastPinchDist = 0;
      this.touchCount = 1;
    }
  };

  private onTouchEnd = (e: TouchEvent): void => {
    this.syncTouchBaseline(e.touches);
    if (e.touches.length === 0) this.mouseNav = "none";
    else if (e.touches.length === 1) {
      const t = e.touches[0]!;
      this.mouseNav = "orbit_anchor";
      this.rotationAnchor.copy(this.findIntersectionPoint(t.clientX, t.clientY));
    }
  };

  private onContextMenu = (e: Event): void => {
    e.preventDefault();
  };

  private onAuxClick = (e: MouseEvent): void => {
    if (e.button === 1) e.preventDefault();
  };

  private onBlur = (): void => {
    this.leftDown = false;
    this.rightDown = false;
    this.middleDown = false;
    this.wDown = false;
    this.aDown = false;
    this.sDown = false;
    this.dDown = false;
    this.altDown = false;
    this.mouseNav = "none";
  };

  dispose(): void {
    this.dom.removeEventListener("mousedown", this.onMouseDown);
    window.removeEventListener("mousemove", this.onMouseMove);
    window.removeEventListener("mouseup", this.onMouseUp);
    this.dom.removeEventListener("wheel", this.onWheel);
    window.removeEventListener("keydown", this.onKeyDown);
    window.removeEventListener("keyup", this.onKeyUp);
    this.dom.removeEventListener("contextmenu", this.onContextMenu);
    this.dom.removeEventListener("auxclick", this.onAuxClick);
    window.removeEventListener("blur", this.onBlur);
    this.dom.removeEventListener("touchstart", this.onTouchStart);
    this.dom.removeEventListener("touchmove", this.onTouchMove);
    this.dom.removeEventListener("touchend", this.onTouchEnd);
    this.dom.removeEventListener("touchcancel", this.onTouchEnd);
  }
}
