// Contact Manifolds — a 1:1 port of sample_manifold.cpp (nine narrow-phase
// viewers). There is no world and no stepping: shape A is fixed, shape B is posed
// by the mouse (drag translates in a plane 10 m in front of the camera,
// Shift-drag rotates), and every frame recomputes one manifold via the ported
// b3Collide* entry points. The C samples derive from two bases — Manifold (points
// and normal expressed in frame A) and TriangleManifold (frame B, with a triangle
// drawn in frame A). "Use cache" persists the SimplexCache/SATCache across frames;
// the faceA/faceB/edgePair radios override the SAT axis on the hull pairs.

import * as THREE from "three";
import {
  createButtonGroup,
  createCheckbox,
  createInfoBox,
  createReadout,
  updateReadout,
} from "../controls.ts";
import { pickRay, TextLabelOverlay, type DebugLabel } from "../interaction.ts";
import { getWasm } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { demoPage, runLoop } from "./common.ts";
import {
  DemoScene,
  makeArrow,
  makeAxes,
  makeCapsule,
  makeDot,
  makeSegment,
  makeSolidBox,
  makeSphere,
  makeTriangleMesh,
  makeWireBox,
  setView,
} from "../three-scene.ts";

// Scene keys mirror the registry (C registration order). init() maps each to the
// wasm scene id 0..=8.
export const SCENES = [
  "sphere-sphere",
  "capsule-sphere",
  "hull-sphere",
  "triangle-sphere",
  "capsule-capsule",
  "capsule-hull",
  "triangle-capsule",
  "hull-hull",
  "triangle-hull",
] as const;

const SCENE_LABELS = [
  "Sphere vs Sphere",
  "Capsule vs Sphere",
  "Hull vs Sphere",
  "Triangle vs Sphere",
  "Capsule vs Capsule",
  "Capsule vs Hull",
  "Triangle vs Capsule",
  "Hull vs Hull",
  "Triangle vs Hull",
];

// Debug-draw palette matching the C sample colors.
const CYAN = 0x00b8c4;
const GREEN = 0x15803d;
const NORMAL_WHITE = 0xffffff;
const POINT_PEN = 0xf2c200; // b3_colorYellow — penetrating (separation <= 0)
const POINT_SEP = 0xffffff; // b3_colorWhite — speculative (separation > 0)
const TRI_NORMAL = 0x9370db; // b3_colorMediumPurple

// Per-scene body colors: colorA is body A (or the triangle on triangle scenes),
// colorB is the convex drawn in frame B; bAlpha < 1 makes body B translucent.
const SCENE_COLORS = [
  { a: GREEN, b: CYAN, bAlpha: 1.0 }, // sphere-sphere
  { a: CYAN, b: GREEN, bAlpha: 1.0 }, // capsule-sphere
  { a: CYAN, b: GREEN, bAlpha: 1.0 }, // hull-sphere
  { a: CYAN, b: GREEN, bAlpha: 0.5 }, // triangle-sphere
  { a: GREEN, b: CYAN, bAlpha: 1.0 }, // capsule-capsule
  { a: CYAN, b: GREEN, bAlpha: 1.0 }, // capsule-hull
  { a: CYAN, b: GREEN, bAlpha: 0.5 }, // triangle-capsule
  { a: GREEN, b: CYAN, bAlpha: 1.0 }, // hull-hull
  { a: CYAN, b: GREEN, bAlpha: 1.0 }, // triangle-hull
];

// Scenes with a small local frame drawn on body B (C DrawAxes at transformB).
const B_AXES_SCENES = new Set([6, 8]);

// The manifold normal is drawn 0.5 * lengthUnitsPerMeter long (LUPM = 1).
const NORMAL_LEN = 0.5;

const SHAPE_SPHERE = 0;
const SHAPE_CAPSULE = 1;
const SHAPE_BOX = 2;
const SHAPE_TRIANGLE = 3;

/** A shape read out of the wasm scene descriptor, kept in its own (local) frame. */
type ShapeDesc =
  | { kind: 0; center: [number, number, number]; radius: number }
  | {
      kind: 1;
      c1: [number, number, number];
      c2: [number, number, number];
      radius: number;
    }
  | { kind: 2; h: [number, number, number]; center: [number, number, number] }
  | { kind: 3; verts: [number, number, number][] };

/** Parsed one-time scene descriptor emitted by `manifold_reset`. */
interface SceneInfo {
  baseTriangle: boolean;
  camera: { yaw: number; pitch: number; dist: number; target: [number, number, number] };
  transformA: { p: THREE.Vector3; q: THREE.Quaternion };
  transformB: { p: THREE.Vector3; q: THREE.Quaternion };
  shapeA: ShapeDesc;
  shapeB: ShapeDesc;
}

function readTransform(a: Float32Array, o: number): { p: THREE.Vector3; q: THREE.Quaternion } {
  return {
    p: new THREE.Vector3(a[o]!, a[o + 1]!, a[o + 2]!),
    q: new THREE.Quaternion(a[o + 3]!, a[o + 4]!, a[o + 5]!, a[o + 6]!),
  };
}

function readShape(a: Float32Array, o: number): ShapeDesc {
  const kind = a[o]!;
  const p = o + 1;
  if (kind === SHAPE_SPHERE) {
    return { kind: 0, center: [a[p]!, a[p + 1]!, a[p + 2]!], radius: a[p + 3]! };
  }
  if (kind === SHAPE_CAPSULE) {
    return {
      kind: 1,
      c1: [a[p]!, a[p + 1]!, a[p + 2]!],
      c2: [a[p + 3]!, a[p + 4]!, a[p + 5]!],
      radius: a[p + 6]!,
    };
  }
  if (kind === SHAPE_BOX) {
    return {
      kind: 2,
      h: [a[p]!, a[p + 1]!, a[p + 2]!],
      center: [a[p + 3]!, a[p + 4]!, a[p + 5]!],
    };
  }
  return {
    kind: 3,
    verts: [
      [a[p]!, a[p + 1]!, a[p + 2]!],
      [a[p + 3]!, a[p + 4]!, a[p + 5]!],
      [a[p + 6]!, a[p + 7]!, a[p + 8]!],
    ],
  };
}

function parseSceneInfo(a: Float32Array): SceneInfo {
  return {
    baseTriangle: a[0] === 1,
    camera: {
      yaw: a[1]!,
      pitch: a[2]!,
      dist: a[3]!,
      target: [a[4]!, a[5]!, a[6]!],
    },
    transformA: readTransform(a, 7),
    transformB: readTransform(a, 14),
    shapeA: readShape(a, 21),
    shapeB: readShape(a, 31),
  };
}

/** Build a shape's mesh(es) in its own local frame; the caller parents it under a
 *  group carrying the body transform. Returns the group and, for triangles, the
 *  local vertices (for vertex labels + the face-normal arrow). */
function buildShape(
  shape: ShapeDesc,
  color: number,
  opacity: number,
): { group: THREE.Group; triVerts: [number, number, number][] | null } {
  const group = new THREE.Group();
  let triVerts: [number, number, number][] | null = null;

  if (shape.kind === SHAPE_SPHERE) {
    group.add(makeSphere(...shape.center, shape.radius, color, opacity));
  } else if (shape.kind === SHAPE_CAPSULE) {
    group.add(makeCapsule(shape.c1, shape.c2, shape.radius, color, opacity));
  } else if (shape.kind === SHAPE_BOX) {
    const [cx, cy, cz] = shape.center;
    const [hx, hy, hz] = shape.h;
    group.add(makeSolidBox(cx, cy, cz, hx, hy, hz, color, Math.min(opacity, 0.45)));
    group.add(makeWireBox(cx, cy, cz, hx, hy, hz, color));
  } else {
    triVerts = shape.verts;
    const positions = new Float32Array([
      ...shape.verts[0]!,
      ...shape.verts[1]!,
      ...shape.verts[2]!,
    ]);
    group.add(makeTriangleMesh(positions, color, opacity));
    group.add(makeTriangleMesh(positions, color, 1.0, true)); // wire edges
  }
  return { group, triVerts };
}

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  assertRouteScenes("manifolds", SCENES);

  const { canvas, controls } = demoPage(
    container,
    "Contact Manifolds",
    "Every <code>sample_manifold.cpp</code> viewer — the nine narrow-phase pairs computed by the " +
      "ported <code>b3CollideSpheres</code>, <code>b3CollideCapsules</code>, " +
      "<code>b3CollideHulls</code>, <code>b3CollideHullAndTriangle</code> and friends. " +
      "Yellow points penetrate; white points are speculative.",
    "Drag body B to move it · Shift-drag to rotate · Alt-drag to orbit the camera",
    wasm.version(),
  );

  let sceneId = initialScene ? Math.max(0, SCENES.indexOf(initialScene as (typeof SCENES)[number])) : 0;

  controls.appendChild(
    createInfoBox(
      "Body A is fixed. <strong>Drag</strong> body B to translate it, <strong>Shift-drag</strong> " +
        "to rotate it, and <strong>Alt-drag</strong> to orbit the camera. Contact points show their " +
        "separation and feature-pair ids; enable <em>Use cache</em> to persist the SAT/simplex cache " +
        "across frames (the faceA/faceB/edgePair radios override the SAT axis on the hull pairs).",
    ),
  );

  const sceneGroup = createButtonGroup(
    SCENES.map((s, i) => ({ label: SCENE_LABELS[i]!, value: s })),
    SCENES[sceneId]!,
    (v) => {
      sceneId = SCENES.indexOf(v as (typeof SCENES)[number]);
      loadScene();
    },
  );
  controls.appendChild(sceneGroup);

  let useCache = false;
  let manualFeature = 0;

  controls.appendChild(
    createCheckbox("Use cache", useCache, (v) => {
      useCache = v;
      featureRow.style.display = useCache ? "" : "none";
    }),
  );

  const featureRow = document.createElement("div");
  featureRow.className = "control-group";
  const featureLabel = document.createElement("label");
  featureLabel.textContent = "SAT feature";
  featureRow.appendChild(featureLabel);
  featureRow.appendChild(
    createButtonGroup(
      [
        { label: "auto", value: "0" },
        { label: "faceA", value: "1" },
        { label: "faceB", value: "2" },
        { label: "edgePair", value: "3" },
      ],
      "0",
      (v) => {
        manualFeature = parseInt(v, 10);
      },
    ),
  );
  featureRow.style.display = "none";
  controls.appendChild(featureRow);

  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { distance: 50, target: [0, 5, 0] });
  const textSprites = new TextLabelOverlay(demo);

  // Body-B pose, seeded from the scene and then driven by the mouse.
  const bPos = new THREE.Vector3();
  const bQuat = new THREE.Quaternion();
  let baseTriangle = false;
  // Fixed frame-A triangle vertices in world space (triangle scenes only).
  let triWorld: THREE.Vector3[] = [];
  // The group carrying body B's transform (updated each frame from bPos/bQuat).
  let bGroup: THREE.Group | null = null;

  function loadScene() {
    const info = parseSceneInfo(wasm.manifold_reset(sceneId));
    baseTriangle = info.baseTriangle;
    bPos.copy(info.transformB.p);
    bQuat.copy(info.transformB.q);

    setView(demo, info.camera.yaw, info.camera.pitch, info.camera.dist, info.camera.target);

    demo.clearContent();
    demo.clearDynamic();
    textSprites.clear();
    triWorld = [];

    demo.content.add(makeAxes(1.0));

    const colors = SCENE_COLORS[sceneId]!;

    // Body A (fixed): mount its group at transform A. On triangle scenes body A is
    // the cyan triangle, and we cache its world-space vertices for labels/arrow.
    const a = buildShape(info.shapeA, colors.a, 0.85);
    a.group.position.copy(info.transformA.p);
    a.group.quaternion.copy(info.transformA.q);
    demo.content.add(a.group);
    if (a.triVerts) {
      triWorld = a.triVerts.map((v) =>
        new THREE.Vector3(v[0], v[1], v[2]).applyQuaternion(info.transformA.q).add(info.transformA.p),
      );
    }

    // Body B (mouse-driven): its group transform is refreshed each frame.
    const b = buildShape(info.shapeB, colors.b, colors.bAlpha);
    if (B_AXES_SCENES.has(sceneId)) b.group.add(makeAxes(0.15));
    b.group.position.copy(bPos);
    b.group.quaternion.copy(bQuat);
    demo.content.add(b.group);
    bGroup = b.group;

    // Reflect the persisted feature-radio visibility for the freshly-built panel.
    featureRow.style.display = useCache ? "" : "none";
  }

  loadScene();

  // --- Body-B mouse interaction (C Manifold::MouseDown/Move) ------------------
  //   plain drag  : translate B in a plane 10 m in front of the camera
  //   Shift-drag  : rotate B (yaw about world Y, roll about world Z)
  // Alt-drag is left to the camera controller (it only orbits while Alt is held).
  let tracking = false;
  let rotating = false;
  const originRef = new THREE.Vector3();
  const baseTranslation = new THREE.Vector3();
  const baseQuat = new THREE.Quaternion();
  let baseX = 0;
  let baseY = 0;
  const _dir = new THREE.Vector3();
  const _qx = new THREE.Quaternion();
  const _qz = new THREE.Quaternion();
  const AXIS_Y = new THREE.Vector3(0, 1, 0);
  const AXIS_Z = new THREE.Vector3(0, 0, 1);

  // The pick point 10 m along the ray (C origin = pickRay.origin + 10 * dir).
  function pickPoint(clientX: number, clientY: number): THREE.Vector3 {
    const { origin, translation } = pickRay(demo, canvas, clientX, clientY);
    _dir.copy(translation).normalize();
    return origin.addScaledVector(_dir, 10);
  }

  const onPointerDown = (e: PointerEvent) => {
    if (e.button !== 0 || e.altKey) return;
    if (e.shiftKey) {
      baseX = e.clientX;
      baseY = e.clientY;
      baseQuat.copy(bQuat);
      rotating = true;
    } else {
      originRef.copy(pickPoint(e.clientX, e.clientY));
      baseTranslation.copy(bPos);
      tracking = true;
    }
    if (tracking || rotating) {
      try {
        canvas.setPointerCapture(e.pointerId);
      } catch {
        /* pointer may not be capturable */
      }
      e.preventDefault();
    }
  };

  const onPointerMove = (e: PointerEvent) => {
    if (tracking) {
      const p = pickPoint(e.clientX, e.clientY);
      bPos.copy(baseTranslation).add(p.sub(originRef));
      e.preventDefault();
    } else if (rotating) {
      _qx.setFromAxisAngle(AXIS_Y, 0.01 * (e.clientX - baseX));
      _qz.setFromAxisAngle(AXIS_Z, 0.01 * (e.clientY - baseY));
      bQuat.copy(baseQuat).multiply(_qx.multiply(_qz)).normalize();
      e.preventDefault();
    }
  };

  const onPointerUp = (e: PointerEvent) => {
    if (tracking || rotating) {
      tracking = false;
      rotating = false;
      try {
        canvas.releasePointerCapture(e.pointerId);
      } catch {
        /* already released */
      }
    }
  };

  canvas.addEventListener("pointerdown", onPointerDown);
  canvas.addEventListener("pointermove", onPointerMove);
  canvas.addEventListener("pointerup", onPointerUp);
  canvas.addEventListener("pointercancel", onPointerUp);

  const stop = runLoop(() => {
    // Push the current B pose + cache settings, then recompute the manifold.
    wasm.manifold_set_transform_b(bPos.x, bPos.y, bPos.z, bQuat.x, bQuat.y, bQuat.z, bQuat.w);
    wasm.manifold_set_cache(useCache, manualFeature);
    const m = wasm.manifold_step();

    if (bGroup) {
      bGroup.position.copy(bPos);
      bGroup.quaternion.copy(bQuat);
    }

    demo.clearDynamic();
    const labels: DebugLabel[] = [];

    const count = m[0]! | 0;
    const nx = m[1]!;
    const ny = m[2]!;
    const nz = m[3]!;
    const satType = m[4]! | 0;
    const cacheHit = m[5]! | 0;
    const feature = m[6]! | 0;

    // Triangle face-normal arrow + vertex-index labels (C TriangleManifold::Render).
    if (baseTriangle && triWorld.length === 3) {
      const p1 = triWorld[0]!;
      const p2 = triWorld[1]!;
      const p3 = triWorld[2]!;
      const e1 = new THREE.Vector3().subVectors(p2, p1);
      const e2 = new THREE.Vector3().subVectors(p3, p1);
      const nrm = new THREE.Vector3().crossVectors(e1, e2).normalize();
      const center = p1.clone().addScaledVector(e1, 1 / 3).addScaledVector(e2, 1 / 3);
      demo.dynamic.add(
        makeArrow([center.x, center.y, center.z], [nrm.x, nrm.y, nrm.z], 0.5, TRI_NORMAL),
      );
      labels.push(
        { x: p1.x, y: p1.y, z: p1.z, color: "#ffffff", text: "0" },
        { x: p2.x, y: p2.y, z: p2.z, color: "#ffffff", text: "1" },
        { x: p3.x, y: p3.y, z: p3.z, color: "#ffffff", text: "2" },
      );
    }

    const separations: string[] = [];
    for (let i = 0; i < count; i++) {
      const o = 7 + 8 * i;
      const px = m[o]!;
      const py = m[o + 1]!;
      const pz = m[o + 2]!;
      const sep = m[o + 3]!;
      const o1 = m[o + 4]! | 0;
      const i1 = m[o + 5]! | 0;
      const o2 = m[o + 6]! | 0;
      const i2 = m[o + 7]! | 0;
      separations.push(sep.toFixed(4));

      demo.dynamic.add(makeDot([px, py, pz], sep > 0 ? POINT_SEP : POINT_PEN, 0.06));
      demo.dynamic.add(
        makeSegment([px, py, pz], [px + NORMAL_LEN * nx, py + NORMAL_LEN * ny, pz + NORMAL_LEN * nz], NORMAL_WHITE),
      );
      // Separation + feature-pair strings, drawn where C draws them (DrawString3D).
      const sepText = baseTriangle ? (100 * sep).toFixed(2) : sep.toFixed(3);
      const hex = (v: number) => v.toString(16).toUpperCase();
      labels.push(
        { x: px, y: py, z: pz, color: "#ffffff", text: `  ${sepText}` },
        {
          x: px + 0.025 * nx,
          y: py + 0.025 * ny,
          z: pz + 0.025 * nz,
          color: "#ffdab9", // b3_colorPapayaWhip
          text: `${hex(o1)}:${hex(i1)} ${hex(o2)}:${hex(i2)}`,
        },
      );
    }

    textSprites.update(labels);

    // Readout: the HUD text lines the C bases draw (count / feature / cache / SAT).
    const entries = [
      { label: "Pair", value: SCENE_LABELS[sceneId]! },
      { label: "Points", value: String(count) },
    ];
    if (baseTriangle) {
      entries.push({ label: "Feature", value: String(feature) });
      entries.push({ label: "Cache hit", value: String(cacheHit) });
    }
    if (sceneId === 7 || sceneId === 8) {
      entries.push({ label: "SAT type", value: String(satType) });
    }
    if (count > 0) {
      entries.push({ label: "Normal", value: `(${nx.toFixed(3)}, ${ny.toFixed(3)}, ${nz.toFixed(3)})` });
      entries.push({ label: "Separations", value: separations.join(", ") });
    }
    updateReadout(readout, entries);

    demo.render();
  }, readout);

  return () => {
    stop();
    canvas.removeEventListener("pointerdown", onPointerDown);
    canvas.removeEventListener("pointermove", onPointerMove);
    canvas.removeEventListener("pointerup", onPointerUp);
    canvas.removeEventListener("pointercancel", onPointerUp);
    textSprites.dispose();
    demo.dispose();
  };
}
