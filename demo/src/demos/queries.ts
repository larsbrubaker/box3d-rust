// Queries — Collision / Cast World
// Faithful Samples App UI for `sample_collision.cpp` CastWorld.

import * as THREE from "three";
import {
  createButton,
  createButtonGroup,
  createCheckbox,
  createInfoBox,
  createSlider,
} from "../controls.ts";
import {
  attachInteraction,
  makeInteractAdapter,
  type ParamValues,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm, type Box3dWasm } from "../wasm.ts";

/** Cast World exports (not yet on Box3dWasm until wasm.ts is regenerated). */
type CastWorldWasm = Box3dWasm & {
  query_set_params(cast_type: number, mode: number, radius: number, initial_overlap: number): void;
  query_set_ray(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): void;
  query_add_shapes(shape_type: number, count: number): number;
  query_destroy_shape(): number;
  query_cast(): Float32Array;
  query_ignore_aabbs(): Float32Array;
  query_surface_wireframe(): Float32Array;
  query_mouse_down(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  query_mouse_move(px: number, py: number, pz: number): void;
  query_mouse_up(): void;
  query_mouse_active(): boolean;
  query_spawn_random(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number, variant?: number): Float32Array;
  query_delete_at_ray(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): number;
  query_counters(): Float32Array;
  query_debug_draw(flags: number): Float32Array;
};
import { demoPage, runLoop } from "./common.ts";
import {
  COLORS,
  DemoScene,
  lineMat,
  makeAxes,
  makeCapsule,
  makeSolidBox,
  makeSphere,
  makeWireBox,
  makeWireEdges,
  setView,
  solidMat,
} from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

/** ShapeType discriminant values from box3d geometry. */
const SHAPE_SPHERE = 5;
const SHAPE_CAPSULE = 0;
const SHAPE_HULL = 3;
const SHAPE_MESH = 4;
const SHAPE_HEIGHT = 2;

const HIT_COLORS = [0xdc2626, 0x16a34a, 0x2563eb];

function applyParams(wasm: CastWorldWasm, p: ParamValues) {
  const castType =
    p.castType === "sphere" ? 1 : p.castType === "capsule" ? 2 : p.castType === "box" ? 3 : 0;
  const mode =
    p.mode === "any" ? 0 : p.mode === "multiple" ? 2 : p.mode === "sorted" ? 3 : 1;
  const radius = Number(p.radius) || 0.5;
  const initialOverlap = p.initialOverlap ? 1 : 0;
  wasm.query_set_params(castType, mode, radius, initialOverlap);
}

function pickRay(
  demo: DemoScene,
  canvas: HTMLCanvasElement,
  clientX: number,
  clientY: number,
): { origin: THREE.Vector3; translation: THREE.Vector3 } {
  const rect = canvas.getBoundingClientRect();
  const ndc = new THREE.Vector2(
    ((clientX - rect.left) / rect.width) * 2 - 1,
    -((clientY - rect.top) / rect.height) * 2 + 1,
  );
  const raycaster = new THREE.Raycaster();
  raycaster.setFromCamera(ndc, demo.camera);
  const origin = raycaster.ray.origin.clone();
  const translation = raycaster.ray.direction.clone().normalize().multiplyScalar(100);
  return { origin, translation };
}

function initCastWorld(container: HTMLElement) {
  const wasm = getWasm() as CastWorldWasm;
  const { canvas, controls } = demoPage(
    container,
    "Queries",
    "Official Collision sample <strong>Cast World</strong> from <code>sample_collision.cpp</code> — " +
      "ray / sphere / capsule / box casts with Any / Closest / Multiple / Sorted modes.",
    "Ctrl+click aim · click select · spawn via buttons · P/O/R",
    wasm.version(),
    { category: "Collision", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "<strong>Cast World</strong> — Ctrl + left mouse aims the cast through the cursor. " +
        "Shapes outlined in yellow AABBs are ignored by the cast (ignore user_data). " +
        "Spawn spheres / capsules / hulls / meshes / height fields; gravity scale is 0.",
    ),
  );

  let ctrl!: SimControllerWithTick;
  const demo = new DemoScene(canvas, { target: [0, 0, 0], distance: 20, shadowExtent: 48 });
  demo.camera.far = 400;
  demo.camera.updateProjectionMatrix();
  const pool = createMeshPool();

  // Cast visualization
  const rayGeo = new THREE.BufferGeometry();
  const rayPos = new Float32Array(6);
  rayGeo.setAttribute("position", new THREE.BufferAttribute(rayPos, 3));
  const rayLine = new THREE.Line(rayGeo, lineMat(0x00ffff, 0.95));
  demo.dynamic.add(rayLine);

  const originMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.15, 12, 10),
    new THREE.MeshStandardMaterial({ color: 0x22c55e, roughness: 0.4 }),
  );
  demo.dynamic.add(originMarker);

  const hitGroup = new THREE.Group();
  demo.dynamic.add(hitGroup);
  const hitMeshes: THREE.Object3D[] = [];

  let ignoreBoxes: THREE.LineSegments | null = null;
  let surfaceWire: THREE.LineSegments | null = null;
  let castProxy: THREE.Object3D | null = null;

  function clearHitViz() {
    for (const m of hitMeshes) {
      hitGroup.remove(m);
      if (m instanceof THREE.Mesh) {
        m.geometry.dispose();
        (m.material as THREE.Material).dispose();
      } else if (m instanceof THREE.Line) {
        m.geometry.dispose();
        (m.material as THREE.Material).dispose();
      }
    }
    hitMeshes.length = 0;
  }

  function clearCastProxy() {
    if (castProxy) {
      demo.dynamic.remove(castProxy);
      castProxy.traverse((o) => {
        if (o instanceof THREE.Mesh) {
          o.geometry.dispose();
          (o.material as THREE.Material).dispose();
        }
      });
      castProxy = null;
    }
  }

  function syncIgnoreAabbs() {
    if (ignoreBoxes) {
      demo.dynamic.remove(ignoreBoxes);
      ignoreBoxes.geometry.dispose();
      (ignoreBoxes.material as THREE.Material).dispose();
      ignoreBoxes = null;
    }
    const aabbs = wasm.query_ignore_aabbs();
    const count = aabbs[0] | 0;
    if (!count) return;
    const edges: number[] = [];
    for (let i = 0; i < count; i++) {
      const o = 1 + i * 6;
      const lx = aabbs[o]!,
        ly = aabbs[o + 1]!,
        lz = aabbs[o + 2]!;
      const ux = aabbs[o + 3]!,
        uy = aabbs[o + 4]!,
        uz = aabbs[o + 5]!;
      const corners: [number, number, number][] = [
        [lx, ly, lz],
        [ux, ly, lz],
        [ux, uy, lz],
        [lx, uy, lz],
        [lx, ly, uz],
        [ux, ly, uz],
        [ux, uy, uz],
        [lx, uy, uz],
      ];
      const segs = [
        [0, 1],
        [1, 2],
        [2, 3],
        [3, 0],
        [4, 5],
        [5, 6],
        [6, 7],
        [7, 4],
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
      ];
      for (const [a, b] of segs) {
        edges.push(...corners[a]!, ...corners[b]!);
      }
    }
    ignoreBoxes = makeWireEdges(edges, 0xfacc15);
    demo.dynamic.add(ignoreBoxes);
  }

  function syncSurfaceWire() {
    if (surfaceWire) {
      demo.dynamic.remove(surfaceWire);
      surfaceWire.geometry.dispose();
      (surfaceWire.material as THREE.Material).dispose();
      surfaceWire = null;
    }
    const wire = wasm.query_surface_wireframe();
    if (!wire.length) return;
    surfaceWire = makeWireEdges(wire, 0x94a3b8);
    demo.dynamic.add(surfaceWire);
  }

  function updateCastViz() {
    const data = wasm.query_cast();
    const count = data[0] | 0;
    const ox = data[1]!,
      oy = data[2]!,
      oz = data[3]!;
    const tx = data[4]!,
      ty = data[5]!,
      tz = data[6]!;
    const castType = data[7] | 0;
    const radius = data[8]!;

    rayPos[0] = ox;
    rayPos[1] = oy;
    rayPos[2] = oz;
    rayPos[3] = ox + tx;
    rayPos[4] = oy + ty;
    rayPos[5] = oz + tz;
    rayGeo.attributes.position!.needsUpdate = true;
    rayGeo.computeBoundingSphere();

    originMarker.position.set(ox, oy, oz);

    clearHitViz();
    clearCastProxy();

    const makeProxy = (frac: number, color: number, alpha = 0.5) => {
      const px = ox + frac * tx;
      const py = oy + frac * ty;
      const pz = oz + frac * tz;
      let obj: THREE.Object3D | null = null;
      if (castType === 1) {
        obj = new THREE.Mesh(
          new THREE.SphereGeometry(radius, 20, 14),
          new THREE.MeshStandardMaterial({
            color,
            transparent: true,
            opacity: alpha,
            roughness: 0.45,
          }),
        );
      } else if (castType === 2) {
        obj = new THREE.Mesh(
          new THREE.CapsuleGeometry(radius, 1, 4, 10),
          new THREE.MeshStandardMaterial({ color, roughness: 0.45 }),
        );
      } else if (castType === 3) {
        obj = new THREE.Mesh(
          new THREE.BoxGeometry(2 * radius, radius, 0.5 * radius),
          new THREE.MeshStandardMaterial({ color, roughness: 0.45 }),
        );
      }
      if (obj) {
        obj.position.set(px, py, pz);
        demo.dynamic.add(obj);
        castProxy = obj;
      }
    };

    if (count > 0) {
      for (let i = 0; i < count; i++) {
        const o = 9 + i * 9;
        const px = data[o]!,
          py = data[o + 1]!,
          pz = data[o + 2]!;
        const nx = data[o + 3]!,
          ny = data[o + 4]!,
          nz = data[o + 5]!;
        const frac = data[o + 6]!;
        const color = HIT_COLORS[i % HIT_COLORS.length]!;

        const marker = new THREE.Mesh(
          new THREE.SphereGeometry(0.12, 10, 8),
          new THREE.MeshStandardMaterial({ color, roughness: 0.35 }),
        );
        marker.position.set(px, py, pz);
        hitGroup.add(marker);
        hitMeshes.push(marker);

        const nGeo = new THREE.BufferGeometry();
        nGeo.setAttribute(
          "position",
          new THREE.Float32BufferAttribute(
            [px, py, pz, px + 0.5 * nx, py + 0.5 * ny, pz + 0.5 * nz],
            3,
          ),
        );
        const nLine = new THREE.Line(nGeo, lineMat(castType === 0 ? color : 0xf97316, 1));
        hitGroup.add(nLine);
        hitMeshes.push(nLine);

        if (i === 0 && castType !== 0) {
          makeProxy(frac, color, 0.45);
        }
      }
    } else if (castType !== 0) {
      makeProxy(1, 0x9ca3af, 0.35);
    }
  }

  function reset() {
    wasm.query_reset();
    applyParams(wasm, ctrl.params);
    setView(demo, 45, 30, 20, [0, 0, 0]);
    syncIgnoreAabbs();
    syncSurfaceWire();
  }

  const spawnRow = document.createElement("div");
  spawnRow.className = "control-row";
  const spawners: { label: string; type: number; count: number }[] = [
    { label: "Spheres", type: SHAPE_SPHERE, count: 10 },
    { label: "Capsules", type: SHAPE_CAPSULE, count: 10 },
    { label: "Hulls", type: SHAPE_HULL, count: 10 },
    { label: "Meshes", type: SHAPE_MESH, count: 1 },
    { label: "Height Field", type: SHAPE_HEIGHT, count: 1 },
  ];
  for (const s of spawners) {
    spawnRow.appendChild(
      createButton(s.label, () => {
        wasm.query_add_shapes(s.type, s.count);
        syncIgnoreAabbs();
        syncSurfaceWire();
      }, false),
    );
  }
  spawnRow.appendChild(
    createButton(
      "Destroy Shape",
      () => {
        wasm.query_destroy_shape();
        syncIgnoreAabbs();
        syncSurfaceWire();
      },
      false,
    ),
  );
  controls.appendChild(spawnRow);

  // Radius slider shown when sphere/capsule cast (mirrors C ImGui).
  const radiusSlider = createSlider("Radius", 0.1, 2.0, 0.5, 0.1, (v) => {
    ctrl.params.radius = v;
    applyParams(wasm, ctrl.params);
  });
  controls.appendChild(radiusSlider);

  function updateRadiusVisibility() {
    const t = ctrl.params.castType;
    radiusSlider.style.display = t === "sphere" || t === "capsule" ? "" : "none";
  }

  ctrl = attachInteraction({
    wasm: makeInteractAdapter(wasm, "query"),
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: "Cast World",
    sampleCategory: "Collision",
    // Ctrl is reserved for cast aim (C CastWorld::MouseDown); spawn via buttons.
    enableSpawnDelete: false,
    params: [
      {
        type: "select",
        key: "castType",
        label: "Cast Type",
        options: [
          { label: "Ray", value: "ray" },
          { label: "Sphere", value: "sphere" },
          { label: "Capsule", value: "capsule" },
          { label: "Box", value: "box" },
        ],
        default: "ray",
        restart: false,
      },
      {
        type: "select",
        key: "mode",
        label: "Mode",
        options: [
          { label: "Any", value: "any" },
          { label: "Closest", value: "closest" },
          { label: "Multiple", value: "multiple" },
          { label: "Sorted", value: "sorted" },
        ],
        default: "closest",
        restart: false,
      },
      {
        type: "slider",
        key: "radius",
        label: "Radius",
        min: 0.1,
        max: 2,
        step: 0.1,
        default: 0.5,
        restart: false,
      },
      {
        type: "checkbox",
        key: "initialOverlap",
        label: "Initial Overlap",
        default: false,
        restart: false,
      },
    ],
    onParamsChange: (values, key) => {
      applyParams(wasm, values);
      if (key === "castType") updateRadiusVisibility();
    },
  }) as SimControllerWithTick;

  // Hide duplicate radius param slider from attachInteraction panel — we keep the C-style one.
  // (attachInteraction still tracks params.radius for applyParams.)
  updateRadiusVisibility();

  // Ctrl-click aim: onCtrlClick is not on AttachInteractionOpts yet, so handle locally.
  const onPointerDown = (e: PointerEvent) => {
    if (!e.ctrlKey || e.button !== 0) return;
    const { origin, translation } = pickRay(demo, canvas, e.clientX, e.clientY);
    // C: m_translation = 100 * normalize(pickRay.translation)
    const len = Math.hypot(translation.x, translation.y, translation.z) || 1;
    const s = 100 / len;
    wasm.query_set_ray(
      origin.x,
      origin.y,
      origin.z,
      translation.x * s,
      translation.y * s,
      translation.z * s,
    );
    e.preventDefault();
    e.stopPropagation();
  };
  canvas.addEventListener("pointerdown", onPointerDown, true);

  reset();

  const readout = controls.querySelector(".info-readout") as HTMLElement | null;
  const stop = runLoop(() => {
    ctrl.tickFrame();
    syncMeshesFromPoses(demo.content, pool, wasm.query_poses(), { styles: wasm.query_styles() });
    updateCastViz();
    // Surfaces / ignore AABBs update infrequently enough via spawn buttons; refresh lightly.
    syncIgnoreAabbs();
    syncSurfaceWire();
    demo.render();
  }, readout ?? undefined);

  return () => {
    stop();
    canvas.removeEventListener("pointerdown", onPointerDown, true);
    clearHitViz();
    clearCastProxy();
    if (ignoreBoxes) {
      demo.dynamic.remove(ignoreBoxes);
      ignoreBoxes.geometry.dispose();
      (ignoreBoxes.material as THREE.Material).dispose();
    }
    if (surfaceWire) {
      demo.dynamic.remove(surfaceWire);
      surfaceWire.geometry.dispose();
      (surfaceWire.material as THREE.Material).dispose();
    }
    disposeMeshPool(pool);
    ctrl.dispose();
    demo.dispose();
  };
}

// ===========================================================================
// Multi-scene dispatch — every Collision sample from sample_collision.cpp.
// The registry (route "queries") routes each entry here by its slug scene key.
// ===========================================================================

/** Scene keys hosted by this page (validated against the registry in registry.test). */
export const SCENES = [
  "cast-world",
  "ray-curtain",
  "mesh-scale",
  "shape-cast",
  "overlap-world",
  "long-ray-cast",
  "initial-overlap",
  "shape-cast-debug",
  "distance-debug",
  "shape-distance",
  "time-of-impact",
  "capsule-cast-ray",
] as const;

export function init(container: HTMLElement, scene?: string) {
  switch (scene) {
    case "ray-curtain":
      return initRayCurtain(container);
    case "mesh-scale":
      return initMeshScale(container);
    case "shape-cast":
      return initShapeCast(container);
    case "overlap-world":
      return initOverlapWorld(container);
    case "long-ray-cast":
      return initLongRayCast(container);
    case "initial-overlap":
      return initInitialOverlap(container);
    case "shape-cast-debug":
      return initShapeCastDebug(container);
    case "distance-debug":
      return initDistanceDebug(container);
    case "shape-distance":
      return initShapeDistance(container);
    case "time-of-impact":
      return initTimeOfImpact(container);
    case "capsule-cast-ray":
      return initCapsuleCastRay(container);
    default:
      return initCastWorld(container);
  }
}

// --- Shared viewer scaffolding ---------------------------------------------

// b3_color* → hex approximations used across the collision viewers.
const CC = {
  green: 0x22c55e,
  red: 0xdc2626,
  blue: 0x2563eb,
  yellow: 0xfacc15,
  orange: 0xf97316,
  gray: 0x9ca3af,
  cyan: 0x22d3ee,
  aqua: 0x22d3ee,
  white: 0xffffff,
  aliceBlue: 0xdbeafe,
  lightGreen: 0x86efac,
  lightBlue: 0x93c5fd,
  lightCoral: 0xf08080,
  lightCyan: 0xa5f3fc,
  bisque: 0xffe4c4,
  dimGray: 0x9ca3af,
};

function disposeObj3(o: THREE.Object3D) {
  o.traverse((c) => {
    const m = c as THREE.Mesh;
    if (m.geometry) m.geometry.dispose();
    const mat = (m as { material?: THREE.Material | THREE.Material[] }).material;
    if (Array.isArray(mat)) mat.forEach((x) => x.dispose());
    else if (mat) mat.dispose();
  });
}

/** A per-frame overlay group: rebuild it each frame from wasm-computed geometry. */
class Overlay {
  readonly group = new THREE.Group();
  private items: THREE.Object3D[] = [];
  constructor(parent: THREE.Object3D) {
    parent.add(this.group);
  }
  add<T extends THREE.Object3D>(o: T): T {
    this.group.add(o);
    this.items.push(o);
    return o;
  }
  clear() {
    for (const o of this.items) {
      this.group.remove(o);
      disposeObj3(o);
    }
    this.items.length = 0;
  }
  /** Line segments from a flat [ax,ay,az, bx,by,bz, ...] position buffer. */
  segs(positions: number[], color: number, opacity = 1) {
    if (positions.length === 0) return;
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    this.add(new THREE.LineSegments(geo, lineMat(color, opacity)));
  }
  line(a: number[], b: number[], color: number, opacity = 1) {
    this.segs([a[0]!, a[1]!, a[2]!, b[0]!, b[1]!, b[2]!], color, opacity);
  }
  /** Screen-space points (pixel size, like C DrawPoint). */
  points(positions: number[], color: number, size = 6, opacity = 1) {
    if (positions.length === 0) return;
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    const mat = new THREE.PointsMaterial({
      color,
      size,
      sizeAttenuation: false,
      transparent: opacity < 1,
      opacity,
    });
    this.add(new THREE.Points(geo, mat));
  }
  point(p: number[], color: number, size = 6) {
    this.points([p[0]!, p[1]!, p[2]!], color, size);
  }
  dispose() {
    this.clear();
    this.group.parent?.remove(this.group);
  }
}

/** Apply a (position, quaternion) transform to a local point. `q` = [x,y,z,w]. */
function xfPoint(p: number[], q: number[] | null, local: number[]): number[] {
  const v = new THREE.Vector3(local[0], local[1], local[2]);
  if (q) v.applyQuaternion(new THREE.Quaternion(q[0], q[1], q[2], q[3]));
  return [v.x + p[0]!, v.y + p[1]!, v.z + p[2]!];
}

/** Wire triangle (3 edges) from 3 world verts. */
function triSegs(a: number[], b: number[], c: number[]): number[] {
  return [
    a[0]!, a[1]!, a[2]!, b[0]!, b[1]!, b[2]!,
    b[0]!, b[1]!, b[2]!, c[0]!, c[1]!, c[2]!,
    c[0]!, c[1]!, c[2]!, a[0]!, a[1]!, a[2]!,
  ];
}

/** A rotated solid box mesh (BoxGeometry + quaternion). half = [hx,hy,hz]. */
function boxMesh(
  center: number[],
  half: number[],
  q: number[] | null,
  color: number,
  opacity: number,
): THREE.Mesh {
  const mesh = makeSolidBox(
    center[0]!,
    center[1]!,
    center[2]!,
    half[0]!,
    half[1]!,
    half[2]!,
    color,
    opacity,
  );
  if (q) mesh.quaternion.set(q[0]!, q[1]!, q[2]!, q[3]!);
  return mesh;
}

type Viewer = {
  wasm: Box3dWasm;
  canvas: HTMLCanvasElement;
  controls: HTMLElement;
  demo: DemoScene;
  readout: HTMLElement | undefined;
};

function makeViewer(
  container: HTMLElement,
  opts: {
    name: string;
    desc: string;
    hint: string;
    target: [number, number, number];
    distance: number;
  },
): Viewer {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Queries",
    opts.desc,
    opts.hint,
    wasm.version(),
    { category: "Collision", samplesShell: true },
  );
  const demo = new DemoScene(canvas, { target: opts.target, distance: opts.distance });
  demo.camera.far = 600;
  demo.camera.updateProjectionMatrix();
  const readout = controls.querySelector(".info-readout") as HTMLElement | null;
  return { wasm, canvas, controls, demo, readout: readout ?? undefined };
}

/** World-axis lines drawn at the origin (C DrawLine axisX/Y/Z, length 0.4). */
function addOriginAxes(ov: Overlay, len = 0.4) {
  ov.line([0, 0, 0], [len, 0, 0], CC.red);
  ov.line([0, 0, 0], [0, len, 0], CC.green);
  ov.line([0, 0, 0], [0, 0, len], CC.blue);
}

// --- Ray Curtain -----------------------------------------------------------

function initRayCurtain(container: HTMLElement) {
  const { wasm, canvas, controls, demo, readout } = makeViewer(container, {
    name: "Ray Curtain",
    desc:
      "Official Collision sample <strong>Ray Curtain</strong> — a curtain of downward " +
      "rays sweeps four rotating kinematic bodies; green marks show closest hits.",
    hint: "Rotating bodies · animated ray curtain",
    target: [0, 0, 0],
    distance: 20,
  });
  controls.appendChild(
    createInfoBox(
      "<strong>Ray Curtain</strong> — sphere, capsule, hull and torus mesh spin as " +
        "kinematic bodies. A fan of rays (x = -8..8) sweeps its z-offset back and forth.",
    ),
  );

  const pool = createMeshPool();
  const ov = new Overlay(demo.dynamic);
  let wire: THREE.LineSegments | null = null;

  wasm.rc_reset();
  setView(demo, 45, 30, 20, [0, 0, 0]);

  const stop = runLoop(() => {
    wasm.rc_step(1 / 60, 4);
    syncMeshesFromPoses(demo.content, pool, wasm.rc_poses(), { styles: undefined });

    if (wire) {
      demo.dynamic.remove(wire);
      wire.geometry.dispose();
      (wire.material as THREE.Material).dispose();
      wire = null;
    }
    const w = wasm.rc_surface_wireframe();
    if (w.length) {
      wire = makeWireEdges(w, CC.gray);
      demo.dynamic.add(wire);
    }

    ov.clear();
    addOriginAxes(ov);
    const rays = wasm.rc_rays();
    const rayLines: number[] = [];
    const origins: number[] = [];
    const ends: number[] = [];
    const hitLines: number[] = [];
    for (let i = 0; i + 12 < rays.length; i += 13) {
      const o = [rays[i]!, rays[i + 1]!, rays[i + 2]!];
      const e = [rays[i + 3]!, rays[i + 4]!, rays[i + 5]!];
      rayLines.push(o[0]!, o[1]!, o[2]!, e[0]!, e[1]!, e[2]!);
      origins.push(o[0]!, o[1]!, o[2]!);
      ends.push(e[0]!, e[1]!, e[2]!);
      if (rays[i + 6]! > 0.5) {
        const p = [rays[i + 7]!, rays[i + 8]!, rays[i + 9]!];
        const n = [rays[i + 10]!, rays[i + 11]!, rays[i + 12]!];
        hitLines.push(p[0]!, p[1]!, p[2]!, p[0]! + 0.5 * n[0]!, p[1]! + 0.5 * n[1]!, p[2]! + 0.5 * n[2]!);
      }
    }
    ov.segs(rayLines, CC.yellow, 0.5);
    ov.segs(hitLines, CC.green);
    ov.points(origins, CC.green, 4);
    ov.points(ends, CC.red, 4);
    demo.render();
  }, readout);

  return () => {
    stop();
    ov.dispose();
    if (wire) {
      demo.dynamic.remove(wire);
      wire.geometry.dispose();
      (wire.material as THREE.Material).dispose();
    }
    disposeMeshPool(pool);
    demo.dispose();
  };
}

// --- Mesh Scale ------------------------------------------------------------

function initMeshScale(container: HTMLElement) {
  const { wasm, canvas, controls, demo, readout } = makeViewer(container, {
    name: "Mesh Scale",
    desc:
      "Official Collision sample <strong>Mesh Scale</strong> — a scalable box mesh is " +
      "cast against by a sphere or ray as the scale and start slide.",
    hint: "Scale X/Y/Z · Start Y/Z · sphere/ray cast",
    target: [0, 0, 0],
    distance: 20,
  });
  controls.appendChild(
    createInfoBox(
      "<strong>Mesh Scale</strong> — negative scales mirror the mesh. The sphere cast " +
        "resolves initial overlap; the yellow marker is the swept sphere's stop.",
    ),
  );

  const p = { sx: 1, sy: 1, sz: 1, startY: 0, startZ: 0, sphere: true };
  const apply = () =>
    wasm.msc_set_params(p.sx, p.sy, p.sz, p.startY, p.startZ, p.sphere ? 1 : 0);

  controls.appendChild(createSlider("Scale X", -2, 2, 1, 0.1, (v) => { p.sx = v; apply(); }));
  controls.appendChild(createSlider("Scale Y", -2, 2, 1, 0.1, (v) => { p.sy = v; apply(); }));
  controls.appendChild(createSlider("Scale Z", -2, 2, 1, 0.1, (v) => { p.sz = v; apply(); }));
  controls.appendChild(createSlider("Start Y", -2, 2, 0, 0.1, (v) => { p.startY = v; apply(); }));
  controls.appendChild(createSlider("Start Z", -2, 2, 0, 0.1, (v) => { p.startZ = v; apply(); }));
  controls.appendChild(
    createCheckbox("sphere Cast", true, (v) => { p.sphere = v; apply(); }),
  );

  const ov = new Overlay(demo.dynamic);
  let wire: THREE.LineSegments | null = null;

  wasm.msc_reset();
  apply();
  setView(demo, 45, 30, 20, [0, 0, 0]);

  const stop = runLoop(() => {
    if (wire) {
      demo.dynamic.remove(wire);
      wire.geometry.dispose();
      (wire.material as THREE.Material).dispose();
      wire = null;
    }
    const w = wasm.msc_wireframe();
    if (w.length) {
      wire = makeWireEdges(w, CC.cyan);
      demo.dynamic.add(wire);
    }

    ov.clear();
    const d = wasm.msc_cast();
    const start = [d[0]!, d[1]!, d[2]!];
    const end = [d[3]!, d[4]!, d[5]!];
    const sphereCast = d[6]! > 0.5;
    const hit = d[7]! > 0.5;
    const spherePos = [d[8]!, d[9]!, d[10]!];
    const radius = d[11]!;
    const hasPoint = d[12]! > 0.5;
    const point = [d[13]!, d[14]!, d[15]!];
    const normal = [d[16]!, d[17]!, d[18]!];

    ov.line(start, end, CC.white);
    ov.point(start, CC.green, 8);
    ov.point(end, CC.red, 8);

    if (sphereCast) {
      ov.add(
        makeSphere(spherePos[0]!, spherePos[1]!, spherePos[2]!, radius, hit ? CC.yellow : CC.gray, 0.75),
      );
    }
    if (hasPoint) {
      ov.line(point, [point[0]! + 0.5 * normal[0]!, point[1]! + 0.5 * normal[1]!, point[2]! + 0.5 * normal[2]!], CC.green);
      ov.point(point, CC.yellow, 5);
    }
    demo.render();
  }, readout);

  return () => {
    stop();
    ov.dispose();
    if (wire) {
      demo.dynamic.remove(wire);
      wire.geometry.dispose();
      (wire.material as THREE.Material).dispose();
    }
    demo.dispose();
  };
}

// --- Shape Cast ------------------------------------------------------------

function initShapeCast(container: HTMLElement) {
  const { wasm, canvas, controls, demo, readout } = makeViewer(container, {
    name: "Shape Cast",
    desc:
      "Official Collision sample <strong>Shape Cast</strong> — rows of sphere / capsule / " +
      "hull casts sweep into a grid of static shapes.",
    hint: "Shift/Ctrl + drag to shift cast start · Initial Overlap",
    target: [0, 1.5, 0],
    distance: 20,
  });
  controls.appendChild(
    createInfoBox(
      "<strong>Shape Cast</strong> — green shapes are the cast start, red the resolved " +
        "stop, gray a miss. Shift + drag (horizontal) and Ctrl + drag (vertical) move the start.",
    ),
  );

  let initialOverlap = false;
  controls.appendChild(
    createCheckbox("Initial Overlap", false, (v) => {
      initialOverlap = v;
      wasm.sc_set_initial_overlap(v ? 1 : 0);
    }),
  );

  const pool = createMeshPool();
  const ov = new Overlay(demo.dynamic);
  let wire: THREE.LineSegments | null = null;

  wasm.sc_reset();
  wasm.sc_set_initial_overlap(0);
  setView(demo, 120, 30, 20, [0, 1.5, 0]);

  // Mouse drag → cast offset (C ShapeCast::MouseDown/Move).
  let trackingX = false;
  let trackingY = false;
  let baseX = 0;
  let baseY = 0;
  let offY = 0;
  let offZ = 0;
  const onDown = (e: PointerEvent) => {
    if (e.button !== 0) return;
    if (e.shiftKey) { trackingX = true; baseX = e.clientX; }
    else if (e.ctrlKey) { trackingY = true; baseY = e.clientY; }
  };
  const onMove = (e: PointerEvent) => {
    if (trackingX) offZ = 0.05 * (baseX - e.clientX);
    if (trackingY) offY = 0.05 * (baseY - e.clientY);
    if (trackingX || trackingY) wasm.sc_set_offset(offY, offZ);
  };
  const onUp = () => { trackingX = false; trackingY = false; };
  canvas.addEventListener("pointerdown", onDown, true);
  window.addEventListener("pointermove", onMove);
  window.addEventListener("pointerup", onUp);

  const stop = runLoop(() => {
    wasm.sc_step(1 / 60, 4);
    syncMeshesFromPoses(demo.content, pool, wasm.sc_poses(), { styles: undefined });

    if (wire) {
      demo.dynamic.remove(wire);
      wire.geometry.dispose();
      (wire.material as THREE.Material).dispose();
      wire = null;
    }
    const w = wasm.sc_surface_wireframe();
    if (w.length) {
      wire = makeWireEdges(w, CC.gray);
      demo.dynamic.add(wire);
    }

    ov.clear();
    addOriginAxes(ov, 1.0);
    const d = wasm.sc_casts();
    let k = 0;
    // 4 spheres: offset(3), r, hit, moved(3), point(3), normal(3)
    for (let s = 0; s < 4; s++) {
      const off = [d[k]!, d[k + 1]!, d[k + 2]!];
      const r = d[k + 3]!;
      const hit = d[k + 4]! > 0.5;
      const moved = [d[k + 5]!, d[k + 6]!, d[k + 7]!];
      const pt = [d[k + 8]!, d[k + 9]!, d[k + 10]!];
      const nrm = [d[k + 11]!, d[k + 12]!, d[k + 13]!];
      k += 14;
      ov.add(makeSphere(off[0]!, off[1]!, off[2]!, r, CC.green, 0.5));
      ov.add(makeSphere(moved[0]!, moved[1]!, moved[2]!, r, hit ? CC.red : CC.gray, 0.5));
      if (hit) drawHitMarker(ov, pt, nrm);
    }
    // 4 capsules: c1(3), c2(3), r, hit, movedOff(3), point(3), normal(3)
    for (let s = 0; s < 4; s++) {
      const c1 = [d[k]!, d[k + 1]!, d[k + 2]!];
      const c2 = [d[k + 3]!, d[k + 4]!, d[k + 5]!];
      const r = d[k + 6]!;
      const hit = d[k + 7]! > 0.5;
      const mo = [d[k + 8]!, d[k + 9]!, d[k + 10]!];
      const pt = [d[k + 11]!, d[k + 12]!, d[k + 13]!];
      const nrm = [d[k + 14]!, d[k + 15]!, d[k + 16]!];
      k += 17;
      ov.add(makeCapsule(c1 as [number, number, number], c2 as [number, number, number], r, CC.green, 0.5));
      const mc1 = [c1[0]! + mo[0]!, c1[1]! + mo[1]!, c1[2]! + mo[2]!];
      const mc2 = [c2[0]! + mo[0]!, c2[1]! + mo[1]!, c2[2]! + mo[2]!];
      ov.add(makeCapsule(mc1 as [number, number, number], mc2 as [number, number, number], r, hit ? CC.red : CC.gray, 0.5));
      if (hit) drawHitMarker(ov, pt, nrm);
    }
    // 4 hulls: center(3), q(4), half, hit, movedOff(3), point(3), normal(3)
    for (let s = 0; s < 4; s++) {
      const c = [d[k]!, d[k + 1]!, d[k + 2]!];
      const q = [d[k + 3]!, d[k + 4]!, d[k + 5]!, d[k + 6]!];
      const half = d[k + 7]!;
      const hit = d[k + 8]! > 0.5;
      const mo = [d[k + 9]!, d[k + 10]!, d[k + 11]!];
      const pt = [d[k + 12]!, d[k + 13]!, d[k + 14]!];
      const nrm = [d[k + 15]!, d[k + 16]!, d[k + 17]!];
      k += 18;
      ov.add(boxMesh(c, [half, half, half], q, CC.green, 0.5));
      ov.add(boxMesh([c[0]! + mo[0]!, c[1]! + mo[1]!, c[2]! + mo[2]!], [half, half, half], q, hit ? CC.red : CC.gray, 0.5));
      if (hit) drawHitMarker(ov, pt, nrm);
    }
    demo.render();
  }, readout);

  return () => {
    stop();
    canvas.removeEventListener("pointerdown", onDown, true);
    window.removeEventListener("pointermove", onMove);
    window.removeEventListener("pointerup", onUp);
    ov.dispose();
    if (wire) {
      demo.dynamic.remove(wire);
      wire.geometry.dispose();
      (wire.material as THREE.Material).dispose();
    }
    disposeMeshPool(pool);
    demo.dispose();
  };
}

function drawHitMarker(ov: Overlay, pt: number[], nrm: number[]) {
  ov.point(pt, CC.red, 4);
  ov.line(pt, [pt[0]! + 0.2 * nrm[0]!, pt[1]! + 0.2 * nrm[1]!, pt[2]! + 0.2 * nrm[2]!], CC.yellow);
}

// --- Overlap World ---------------------------------------------------------

function initOverlapWorld(container: HTMLElement) {
  const { wasm, canvas, controls, demo, readout } = makeViewer(container, {
    name: "Overlap World",
    desc:
      "Official Collision sample <strong>Overlap World</strong> — rows of overlap probes " +
      "turn red where they intersect a grid of shapes.",
    hint: "Shift + drag to move the probes",
    target: [0, 1.5, 0],
    distance: 20,
  });
  controls.appendChild(
    createInfoBox(
      "<strong>Overlap World</strong> — sphere, capsule and hull probes query the world; " +
        "green means clear, red means overlapping. Shift + drag moves them in z.",
    ),
  );

  const pool = createMeshPool();
  const ov = new Overlay(demo.dynamic);
  let wire: THREE.LineSegments | null = null;

  wasm.ow_reset();
  setView(demo, 120, 30, 20, [0, 1.5, 0]);

  let tracking = false;
  let baseX = 0;
  let off = 0;
  const onDown = (e: PointerEvent) => {
    if (e.button === 0 && e.shiftKey) { tracking = true; baseX = e.clientX; }
  };
  const onMove = (e: PointerEvent) => {
    if (tracking) { off = 0.05 * (baseX - e.clientX); wasm.ow_set_offset(off); }
  };
  const onUp = () => { tracking = false; };
  canvas.addEventListener("pointerdown", onDown, true);
  window.addEventListener("pointermove", onMove);
  window.addEventListener("pointerup", onUp);

  const stop = runLoop(() => {
    wasm.ow_step(1 / 60, 4);
    syncMeshesFromPoses(demo.content, pool, wasm.ow_poses(), { styles: undefined });

    if (wire) {
      demo.dynamic.remove(wire);
      wire.geometry.dispose();
      (wire.material as THREE.Material).dispose();
      wire = null;
    }
    const w = wasm.ow_surface_wireframe();
    if (w.length) {
      wire = makeWireEdges(w, CC.gray);
      demo.dynamic.add(wire);
    }

    ov.clear();
    addOriginAxes(ov);
    const d = wasm.ow_overlaps();
    let k = 0;
    // 5 spheres: cx,cy,cz, r, overlap
    for (let s = 0; s < 5; s++) {
      const c = [d[k]!, d[k + 1]!, d[k + 2]!];
      const r = d[k + 3]!;
      const hit = d[k + 4]! > 0.5;
      k += 5;
      ov.add(makeSphere(c[0]!, c[1]!, c[2]!, r, hit ? CC.red : CC.green, 0.75));
    }
    // 5 capsules: c1(3), c2(3), r, overlap
    for (let s = 0; s < 5; s++) {
      const c1 = [d[k]!, d[k + 1]!, d[k + 2]!];
      const c2 = [d[k + 3]!, d[k + 4]!, d[k + 5]!];
      const r = d[k + 6]!;
      const hit = d[k + 7]! > 0.5;
      k += 8;
      ov.add(makeCapsule(c1 as [number, number, number], c2 as [number, number, number], r, hit ? CC.red : CC.green, 0.75));
    }
    // 5 hulls: cx,cy,cz, half, overlap
    for (let s = 0; s < 5; s++) {
      const c = [d[k]!, d[k + 1]!, d[k + 2]!];
      const half = d[k + 3]!;
      const hit = d[k + 4]! > 0.5;
      k += 5;
      ov.add(boxMesh(c, [half, half, half], null, hit ? CC.red : CC.green, 0.6));
    }
    demo.render();
  }, readout);

  return () => {
    stop();
    canvas.removeEventListener("pointerdown", onDown, true);
    window.removeEventListener("pointermove", onMove);
    window.removeEventListener("pointerup", onUp);
    ov.dispose();
    if (wire) {
      demo.dynamic.remove(wire);
      wire.geometry.dispose();
      (wire.material as THREE.Material).dispose();
    }
    disposeMeshPool(pool);
    demo.dispose();
  };
}

// --- Long Ray Cast ---------------------------------------------------------

function initLongRayCast(container: HTMLElement) {
  const { wasm, canvas, controls, demo, readout } = makeViewer(container, {
    name: "Long Ray Cast",
    desc:
      "Official Collision sample <strong>Long Ray Cast</strong> — rays fired from kilometers " +
      "away test far-origin single-precision accuracy; a cone sweeps each hit into a loop.",
    hint: "Ray Length (km) · Cone Angle",
    target: [0, 1, 0],
    distance: 34,
  });
  controls.appendChild(
    createInfoBox(
      "<strong>Long Ray Cast</strong> — green trail = accurate, orange = drifting, red = a " +
        "miss the near ground-truth ray would have hit. The rock is drawn from its real convex hull.",
    ),
  );

  const p = { km: 1, cone: 5 };
  const apply = () => wasm.lrc_set_params(p.km, p.cone);
  controls.appendChild(
    createSlider("Ray Length (km)", 1, 10000, 1, 1, (v) => { p.km = v; apply(); }),
  );
  controls.appendChild(
    createSlider("Cone Angle", 0, 12, 5, 0.1, (v) => { p.cone = v; apply(); }),
  );

  const pool = createMeshPool();
  const ov = new Overlay(demo.dynamic);
  let wire: THREE.LineSegments | null = null;
  let rockMesh: THREE.Mesh | null = null;
  let rockWire: THREE.LineSegments | null = null;

  wasm.lrc_reset();
  apply();
  setView(demo, -35, 22, 34, [0, 1, 0]);

  if (wire === null) {
    const w = wasm.lrc_surface_wireframe();
    if (w.length) {
      wire = makeWireEdges(w, CC.gray);
      demo.dynamic.add(wire);
    }
  }

  // Rock: solid faces + wireframe edges from the real create_rock hull (static).
  {
    const g = wasm.lrc_rock_geometry();
    let p = 0;
    const triCount = g[p++]! | 0;
    const tris = g.slice(p, p + triCount);
    p += triCount;
    const edgeCount = g[p++]! | 0;
    const edges = g.slice(p, p + edgeCount);
    if (triCount > 0) {
      const rg = new THREE.BufferGeometry();
      rg.setAttribute("position", new THREE.BufferAttribute(new Float32Array(tris), 3));
      rg.computeVertexNormals();
      rockMesh = new THREE.Mesh(rg, solidMat(0x8a8f98, 1));
      rockMesh.castShadow = true;
      rockMesh.receiveShadow = true;
      demo.dynamic.add(rockMesh);
      rockWire = makeWireEdges(edges, 0x1e293b);
      demo.dynamic.add(rockWire);
    }
  }

  const stop = runLoop(() => {
    syncMeshesFromPoses(demo.content, pool, wasm.lrc_poses(), { styles: undefined });

    ov.clear();
    const d = wasm.lrc_step();
    const coneDir = [d[0]!, d[1]!, d[2]!];
    let k = 3;
    for (let i = 0; i < 5; i++) {
      const state = d[k]!;
      const colorCat = d[k + 1]!;
      const pt = [d[k + 2]!, d[k + 3]!, d[k + 4]!];
      const nrm = [d[k + 5]!, d[k + 6]!, d[k + 7]!];
      const aim = [d[k + 9]!, d[k + 10]!, d[k + 11]!];
      k += 12;
      const trailCount = d[k]! | 0;
      k += 1;
      const trail: number[] = [];
      for (let j = 0; j < trailCount; j++) {
        trail.push(d[k]!, d[k + 1]!, d[k + 2]!);
        k += 4;
      }
      if (state === 0) {
        // Hit: aqua approach line, yellow normal, colored point.
        ov.line([pt[0]! + 3 * coneDir[0]!, pt[1]! + 3 * coneDir[1]!, pt[2]! + 3 * coneDir[2]!], pt, CC.aqua);
        ov.line(pt, [pt[0]! + 1.5 * nrm[0]!, pt[1]! + 1.5 * nrm[1]!, pt[2]! + 1.5 * nrm[2]!], CC.yellow);
        ov.point(pt, colorCat === 0 ? CC.green : CC.orange, 8);
      } else if (state === 1) {
        // Accuracy failure: red expected point + red slash.
        ov.point(pt, CC.red, 14);
        ov.line(
          [pt[0]! + 2 * coneDir[0]!, pt[1]! + 2 * coneDir[1]!, pt[2]! + 2 * coneDir[2]!],
          [pt[0]! - 2 * coneDir[0]!, pt[1]! - 2 * coneDir[1]!, pt[2]! - 2 * coneDir[2]!],
          CC.red,
        );
      } else {
        // Geometric miss: faint gray slash through the aim.
        ov.line(
          [aim[0]! + 2 * coneDir[0]!, aim[1]! + 2 * coneDir[1]!, aim[2]! + 2 * coneDir[2]!],
          [aim[0]! - 4 * coneDir[0]!, aim[1]! - 4 * coneDir[1]!, aim[2]! - 4 * coneDir[2]!],
          CC.gray,
          0.4,
        );
      }
      if (trail.length) ov.points(trail, CC.green, 4, 0.8);
    }
    demo.render();
  }, readout);

  return () => {
    stop();
    ov.dispose();
    if (wire) {
      demo.dynamic.remove(wire);
      wire.geometry.dispose();
      (wire.material as THREE.Material).dispose();
    }
    if (rockMesh) {
      demo.dynamic.remove(rockMesh);
      rockMesh.geometry.dispose();
      (rockMesh.material as THREE.Material).dispose();
    }
    if (rockWire) {
      demo.dynamic.remove(rockWire);
      rockWire.geometry.dispose();
      (rockWire.material as THREE.Material).dispose();
    }
    disposeMeshPool(pool);
    demo.dispose();
  };
}

// --- Initial Overlap -------------------------------------------------------

function initInitialOverlap(container: HTMLElement) {
  const { wasm, canvas, controls, demo, readout } = makeViewer(container, {
    name: "Initial Overlap",
    desc:
      "Official Collision sample <strong>Initial Overlap</strong> — a zero-length capsule " +
      "shape-cast against a mesh; the toggle decides whether a touching start reports a hit.",
    hint: "Toggle initial overlap",
    target: [0, 0, 0],
    distance: 10,
  });
  controls.appendChild(
    createInfoBox(
      "<strong>Initial Overlap</strong> — with the toggle on, an already-overlapping cast " +
        "reports the contact (red capsule + witness point); off, it is ignored.",
    ),
  );

  controls.appendChild(
    createCheckbox("initial overlap", true, (v) => wasm.io_set_initial_overlap(v ? 1 : 0)),
  );

  const ov = new Overlay(demo.dynamic);
  let wire: THREE.LineSegments | null = null;

  wasm.io_reset();
  setView(demo, -140, 10, 10, [0, 0, 0]);

  const stop = runLoop(() => {
    if (!wire) {
      const w = wasm.io_surface_wireframe();
      if (w.length) {
        wire = makeWireEdges(w, CC.cyan);
        demo.dynamic.add(wire);
      }
    }
    ov.clear();
    ov.add(makeAxes(1));
    const d = wasm.io_cast();
    const c1 = [d[0]!, d[1]!, d[2]!];
    const c2 = [d[3]!, d[4]!, d[5]!];
    const r = d[6]!;
    const hit = d[7]! > 0.5;
    // green capsule always; red overlaid when overlapping.
    ov.add(makeCapsule(c1 as [number, number, number], c2 as [number, number, number], r, hit ? CC.red : CC.green, 0.6));
    if (hit) {
      const pt = [d[8]!, d[9]!, d[10]!];
      const nrm = [d[11]!, d[12]!, d[13]!];
      ov.line(pt, [pt[0]! + 0.5 * nrm[0]!, pt[1]! + 0.5 * nrm[1]!, pt[2]! + 0.5 * nrm[2]!], CC.aliceBlue);
      ov.point(pt, CC.aliceBlue, 8);
    }
    demo.render();
  }, readout);

  return () => {
    stop();
    ov.dispose();
    if (wire) {
      demo.dynamic.remove(wire);
      wire.geometry.dispose();
      (wire.material as THREE.Material).dispose();
    }
    demo.dispose();
  };
}

// --- Shape Cast Debug ------------------------------------------------------

function initShapeCastDebug(container: HTMLElement) {
  const { wasm, controls, demo, readout } = makeViewer(container, {
    name: "Shape Cast Debug",
    desc:
      "Official Collision sample <strong>Shape Cast Debug</strong> — a degenerate large-world " +
      "shape cast (triangle vs capsule) reproduced at 0.01 scale.",
    hint: "Static repro",
    target: [0, 1.5, 0],
    distance: 20,
  });
  controls.appendChild(
    createInfoBox(
      "<strong>Shape Cast Debug</strong> — cyan triangle, green start capsule, red the hit " +
        "position, gray the full sweep end.",
    ),
  );

  const ov = new Overlay(demo.dynamic);
  setView(demo, 120, 30, 20, [0, 1.5, 0]);

  const stop = runLoop(() => {
    ov.clear();
    ov.add(makeAxes(1));
    const d = wasm.scd_data();
    const hit = d[0]! > 0.5;
    const t0 = [d[2]!, d[3]!, d[4]!];
    const t1 = [d[5]!, d[6]!, d[7]!];
    const t2 = [d[8]!, d[9]!, d[10]!];
    const c1 = [d[11]!, d[12]!, d[13]!];
    const c2 = [d[14]!, d[15]!, d[16]!];
    const r = d[17]!;
    const greenP = [d[18]!, d[19]!, d[20]!];
    const grayP = [d[21]!, d[22]!, d[23]!];
    const redP = [d[24]!, d[25]!, d[26]!];

    ov.segs(triSegs(t0, t1, t2), CC.cyan);
    const cap = (p: number[], color: number) =>
      ov.add(
        makeCapsule(
          [c1[0]! + p[0]!, c1[1]! + p[1]!, c1[2]! + p[2]!] as [number, number, number],
          [c2[0]! + p[0]!, c2[1]! + p[1]!, c2[2]! + p[2]!] as [number, number, number],
          r,
          color,
          0.6,
        ),
      );
    cap(greenP, CC.green);
    if (hit) cap(redP, CC.red);
    cap(grayP, CC.gray);
    demo.render();
  }, readout);

  return () => {
    stop();
    ov.dispose();
    demo.dispose();
  };
}

// --- Distance Debug --------------------------------------------------------

function initDistanceDebug(container: HTMLElement) {
  const { wasm, controls, demo, readout } = makeViewer(container, {
    name: "Distance Debug",
    desc:
      "Official Collision sample <strong>Distance Debug</strong> — GJK distance between two " +
      "fixed boxes, with a slider to step through the recorded simplex history.",
    hint: "Simplex index slider",
    target: [0, 1.5, 0],
    distance: 20,
  });
  controls.appendChild(
    createInfoBox(
      "<strong>Distance Debug</strong> — green box A, cyan box B; white marks the closest " +
        "points, green the current simplex witness, red the simplex support points.",
    ),
  );

  let simplexIndex = 0;
  let simplexSlider: HTMLElement | null = null;
  let lastCount = -1;
  const mkSlider = (count: number) => {
    if (simplexSlider) simplexSlider.remove();
    simplexSlider = createSlider("simplex index", 0, Math.max(0, count - 1), 0, 1, (v) => {
      simplexIndex = v | 0;
    });
    controls.appendChild(simplexSlider);
  };

  const ov = new Overlay(demo.dynamic);
  const label = document.createElement("div");
  label.className = "info-readout";
  controls.appendChild(label);
  setView(demo, 120, 30, 20, [0, 1.5, 0]);

  const stop = runLoop(() => {
    const d = wasm.dd_data(simplexIndex);
    const bp = [d[0]!, d[1]!, d[2]!];
    const bq = [d[3]!, d[4]!, d[5]!, d[6]!];
    const distance = d[7]!;
    const pA = [d[11]!, d[12]!, d[13]!];
    const pB = [d[14]!, d[15]!, d[16]!];
    const count = d[17]! | 0;
    if (count !== lastCount) { mkSlider(count); lastCount = count; }

    ov.clear();
    ov.add(makeAxes(1));
    // Box A (40,1,40) at identity, Box B (0.5,10,0.5) at transformB.
    ov.add(boxMesh([0, 0, 0], [40, 1, 40], null, CC.green, 0.25));
    ov.add(boxMesh(bp, [0.5, 10, 0.5], bq, CC.cyan, 0.4));
    ov.point(pA, CC.white, 5);
    ov.point(pB, CC.white, 5);

    const hasSel = d[18]! > 0.5;
    if (hasSel) {
      const w1 = [d[19]!, d[20]!, d[21]!];
      const w2 = [d[22]!, d[23]!, d[24]!];
      ov.point(w1, CC.green, 10);
      ov.point(w2, CC.green, 10);
      let k = 26;
      const vcount = d[25]! | 0;
      for (let i = 0; i < vcount; i++) {
        ov.point([d[k]!, d[k + 1]!, d[k + 2]!], CC.red, 5);
        ov.point([d[k + 3]!, d[k + 4]!, d[k + 5]!], CC.red, 5);
        k += 6;
      }
      label.textContent = `distance = ${distance.toFixed(4)} · simplex ${simplexIndex}/${count - 1}`;
    } else {
      label.textContent = `distance = ${distance.toFixed(4)}`;
    }
    demo.render();
  }, readout);

  return () => {
    stop();
    ov.dispose();
    demo.dispose();
  };
}

// --- Shape Distance --------------------------------------------------------

const SD_SHAPES = ["point", "segment", "triangle", "box"];

function initShapeDistance(container: HTMLElement) {
  const { wasm, canvas, controls, demo, readout } = makeViewer(container, {
    name: "Shape Distance",
    desc:
      "Official Collision sample <strong>Shape Distance</strong> — GJK closest points between " +
      "two shapes; drag / rotate shape B with the mouse.",
    hint: "LMB drag · Shift+LMB rotate",
    target: [0, 0, 0],
    distance: 5,
  });
  controls.appendChild(
    createInfoBox(
      "<strong>Shape Distance</strong> — cyan shape A is fixed, bisque shape B follows the " +
        "mouse. Yellow is the separating normal; toggle the simplex viewer to inspect GJK.",
    ),
  );

  const st = {
    typeA: 2,
    typeB: 3,
    radiusA: 0,
    radiusB: 0,
    useCache: false,
    showIndices: false,
    drawSimplex: false,
    simplexIndex: 0,
  };
  const apply = () =>
    wasm.sd_set_params(
      st.typeA,
      st.typeB,
      st.radiusA,
      st.radiusB,
      st.useCache ? 1 : 0,
      st.showIndices ? 1 : 0,
      st.drawSimplex ? 1 : 0,
    );

  controls.appendChild(
    createButtonGroup(
      SD_SHAPES.map((s, i) => ({ label: s, value: String(i) })),
      "2",
      (v) => { st.typeA = Number(v); apply(); },
    ),
  );
  controls.appendChild(
    createButtonGroup(
      SD_SHAPES.map((s, i) => ({ label: s, value: String(i) })),
      "3",
      (v) => { st.typeB = Number(v); apply(); },
    ),
  );
  controls.appendChild(createSlider("radius A", 0, 0.5, 0, 0.01, (v) => { st.radiusA = v; apply(); }));
  controls.appendChild(createSlider("radius B", 0, 0.5, 0, 0.01, (v) => { st.radiusB = v; apply(); }));
  controls.appendChild(createCheckbox("show indices", false, (v) => { st.showIndices = v; apply(); }));
  controls.appendChild(createCheckbox("use cache", false, (v) => { st.useCache = v; apply(); }));
  controls.appendChild(createCheckbox("draw simplex", false, (v) => { st.drawSimplex = v; st.simplexIndex = 0; apply(); }));
  const idxSlider = createSlider("index", 0, 0, 0, 1, (v) => { st.simplexIndex = v | 0; });
  controls.appendChild(idxSlider);

  const label = document.createElement("div");
  label.className = "info-readout";
  controls.appendChild(label);

  const ov = new Overlay(demo.dynamic);
  wasm.sd_reset();
  apply();
  setView(demo, -45, 10, 5, [0, 0, 0]);

  // Transform B state (mouse driven).
  let posB = new THREE.Vector3(0, 1, 0);
  let quatB = new THREE.Quaternion(0, 0, 0, 1);
  let dragging = false;
  let rotating = false;
  let dragStart = new THREE.Vector3();
  let basePos = new THREE.Vector3();
  let baseQuat = new THREE.Quaternion();
  let rotateStartX = 0;

  const pickPlanePoint = (clientX: number, clientY: number): THREE.Vector3 => {
    const rect = canvas.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1,
    );
    const rc = new THREE.Raycaster();
    rc.setFromCamera(ndc, demo.camera);
    const o = rc.ray.origin.clone();
    const dd = rc.ray.direction.clone().normalize();
    // Plane through origin: p = o - dot(o, d) * d.
    return o.sub(dd.multiplyScalar(o.dot(dd)));
  };

  const onDown = (e: PointerEvent) => {
    if (e.button !== 0) return;
    if (!e.shiftKey && !rotating) {
      dragging = true;
      dragStart = pickPlanePoint(e.clientX, e.clientY);
      basePos = posB.clone();
    } else if (e.shiftKey && !dragging) {
      rotating = true;
      rotateStartX = e.clientX;
      baseQuat = quatB.clone();
    }
  };
  const onMove = (e: PointerEvent) => {
    if (dragging) {
      const p = pickPlanePoint(e.clientX, e.clientY);
      posB = basePos.clone().add(p.sub(dragStart));
      wasm.sd_set_transform_b(posB.x, posB.y, posB.z, quatB.x, quatB.y, quatB.z, quatB.w);
    } else if (rotating) {
      const rect = canvas.getBoundingClientRect();
      const dx = (e.clientX - rotateStartX) / rect.width;
      const angle = Math.max(-Math.PI, Math.min(Math.PI, 2 * dx));
      const fwd = new THREE.Vector3();
      demo.camera.getWorldDirection(fwd);
      const dq = new THREE.Quaternion().setFromAxisAngle(fwd, angle);
      quatB = dq.multiply(baseQuat);
      wasm.sd_set_transform_b(posB.x, posB.y, posB.z, quatB.x, quatB.y, quatB.z, quatB.w);
    }
  };
  const onUp = () => { dragging = false; rotating = false; };
  canvas.addEventListener("pointerdown", onDown, true);
  window.addEventListener("pointermove", onMove);
  window.addEventListener("pointerup", onUp);
  wasm.sd_set_transform_b(posB.x, posB.y, posB.z, quatB.x, quatB.y, quatB.z, quatB.w);

  const stop = runLoop(() => {
    const d = wasm.sd_step(st.simplexIndex);
    const typeA = d[0]! | 0;
    const radiusA = d[1]!;
    const typeB = d[2]! | 0;
    const radiusB = d[3]!;
    const bp = [d[4]!, d[5]!, d[6]!];
    const bq = [d[7]!, d[8]!, d[9]!, d[10]!];
    const distance = d[11]!;
    const iterations = d[12]! | 0;
    const drawSimplex = d[13]! > 0.5;
    const simplexCount = d[15]! | 0;
    const pA = [d[16]!, d[17]!, d[18]!];
    const pB = [d[19]!, d[20]!, d[21]!];
    const normal = [d[22]!, d[23]!, d[24]!];

    // Update the index slider range.
    const idxInput = idxSlider.querySelector("input") as HTMLInputElement | null;
    if (idxInput && drawSimplex && simplexCount > 0) {
      idxInput.max = String(simplexCount - 1);
    }

    ov.clear();
    ov.add(makeAxes(0.5));
    drawSDShape(ov, typeA, [0, 0, 0], null, radiusA, CC.cyan);
    drawSDShape(ov, typeB, bp, bq, radiusB, CC.bisque);

    if (drawSimplex) {
      const hasSel = d[25]! > 0.5;
      const witnessValid = d[27]! > 0.5;
      if (witnessValid) {
        const wa = [d[28]!, d[29]!, d[30]!];
        const wb = [d[31]!, d[32]!, d[33]!];
        ov.line(wa, wb, CC.white);
        ov.point(wa, CC.lightGreen, 10);
        ov.point(wb, CC.lightBlue, 10);
      }
      let k = 35;
      const vcount = d[34]! | 0;
      const cols = [CC.red, CC.green, CC.blue];
      for (let i = 0; i < vcount; i++) {
        ov.point([d[k]!, d[k + 1]!, d[k + 2]!], cols[i % 3]!, 10);
        ov.point([d[k + 3]!, d[k + 4]!, d[k + 5]!], cols[i % 3]!, 10);
        k += 6;
      }
      void hasSel;
    } else {
      ov.line(pA, pB, CC.dimGray);
      ov.point(pA, CC.lightGreen, 10);
      ov.point(pB, CC.lightBlue, 10);
      ov.line(pA, [pA[0]! + 0.5 * normal[0]!, pA[1]! + 0.5 * normal[1]!, pA[2]! + 0.5 * normal[2]!], CC.yellow);
    }
    label.textContent = `distance = ${distance.toFixed(4)}, iterations = ${iterations}`;
    demo.render();
  }, readout);

  return () => {
    stop();
    canvas.removeEventListener("pointerdown", onDown, true);
    window.removeEventListener("pointermove", onMove);
    window.removeEventListener("pointerup", onUp);
    ov.dispose();
    demo.dispose();
  };
}

// C geometry for the point / segment / triangle / box shapes (sample_collision.cpp:2069).
const SD_POINT = [0, 0, 0];
const SD_SEGMENT = [
  [-0.5, 0, 0],
  [0.5, 0, 0],
];
const SD_TRIANGLE = [
  [-1.5, 0, 0],
  [1.5, 0, 0],
  [0, 0, 2],
];
const SD_BOX_HALF = [0.125, 0.25, 0.5];

function drawSDShape(
  ov: Overlay,
  type: number,
  p: number[],
  q: number[] | null,
  radius: number,
  color: number,
) {
  switch (type) {
    case 0: {
      const w = xfPoint(p, q, SD_POINT);
      if (radius > 0) ov.add(makeSphere(w[0]!, w[1]!, w[2]!, radius, color, 0.6));
      else ov.point(w, color, 5);
      break;
    }
    case 1: {
      const a = xfPoint(p, q, SD_SEGMENT[0]!);
      const b = xfPoint(p, q, SD_SEGMENT[1]!);
      if (radius > 0) ov.add(makeCapsule(a as [number, number, number], b as [number, number, number], radius, color, 0.6));
      else ov.line(a, b, color);
      break;
    }
    case 2: {
      const a = xfPoint(p, q, SD_TRIANGLE[0]!);
      const b = xfPoint(p, q, SD_TRIANGLE[1]!);
      const c = xfPoint(p, q, SD_TRIANGLE[2]!);
      ov.segs(triSegs(a, b, c), color);
      break;
    }
    default:
      ov.add(boxMesh(p, SD_BOX_HALF, q, color, 0.5));
  }
}

// --- Time of Impact --------------------------------------------------------

const TOI_SHAPES = ["box", "capsule", "triangle"];

function initTimeOfImpact(container: HTMLElement) {
  const { wasm, controls, demo, readout } = makeViewer(container, {
    name: "Time of Impact",
    desc:
      "Official Collision sample <strong>Time of Impact</strong> — the conservative-advancement " +
      "TOI between a fixed shape A and a swept shape B.",
    hint: "Shape A / Shape B",
    target: [0, 0, 0],
    distance: 10,
  });
  controls.appendChild(
    createInfoBox(
      "<strong>Time of Impact</strong> — green = sweep start, coral = sweep end, cyan = the " +
        "computed impact pose. (C's shape combo mislabels its box/capsule/triangle cases; these " +
        "labels are corrected.)",
    ),
  );

  const st = { typeA: 2, typeB: 1 };
  controls.appendChild(
    createButtonGroup(
      TOI_SHAPES.map((s, i) => ({ label: s, value: String(i) })),
      "2",
      (v) => { st.typeA = Number(v); },
    ),
  );
  controls.appendChild(
    createButtonGroup(
      TOI_SHAPES.map((s, i) => ({ label: s, value: String(i) })),
      "1",
      (v) => { st.typeB = Number(v); },
    ),
  );
  const label = document.createElement("div");
  label.className = "info-readout";
  controls.appendChild(label);

  const ov = new Overlay(demo.dynamic);
  setView(demo, -90, 0, 10, [0, 0, 0]);

  const stop = runLoop(() => {
    const d = wasm.toi_data(st.typeA, st.typeB);
    let k = 0;
    const typeA = d[k]! | 0;
    const descA = [d[k + 1]!, d[k + 2]!, d[k + 3]!, d[k + 4]!, d[k + 5]!, d[k + 6]!, d[k + 7]!, d[k + 8]!, d[k + 9]!];
    k += 10;
    const typeB = d[k]! | 0;
    const descB = [d[k + 1]!, d[k + 2]!, d[k + 3]!, d[k + 4]!, d[k + 5]!, d[k + 6]!, d[k + 7]!, d[k + 8]!, d[k + 9]!];
    k += 10;
    const xf1 = readXf(d, k); k += 7;
    const xf2 = readXf(d, k); k += 7;
    const hasHit = d[k]! > 0.5; k += 1;
    const xfHit = readXf(d, k); k += 7;
    const state = d[k]! | 0; k += 1;
    const fraction = d[k]!; k += 1;
    const angle = d[k]!; k += 1;
    const hasPoint = d[k]! > 0.5; k += 1;
    const point = [d[k]!, d[k + 1]!, d[k + 2]!];
    const normal = [d[k + 3]!, d[k + 4]!, d[k + 5]!];

    ov.clear();
    ov.add(makeAxes(0.5));
    drawTOIShape(ov, typeA, descA, [0, 0, 0], [0, 0, 0, 1], CC.cyan);
    drawTOIShape(ov, typeB, descB, xf1.p, xf1.q, CC.lightGreen);
    drawTOIShape(ov, typeB, descB, xf2.p, xf2.q, CC.lightCoral);
    if (hasHit) drawTOIShape(ov, typeB, descB, xfHit.p, xfHit.q, CC.lightCyan);
    if (hasPoint) {
      ov.line(point, [point[0]! + 0.5 * normal[0]!, point[1]! + 0.5 * normal[1]!, point[2]! + 0.5 * normal[2]!], CC.dimGray);
      ov.point(point, CC.lightGreen, 10);
    }
    const states = ["unknown", "failed", "overlapped", "hit", "separated"];
    const stateText = state === 3 ? `hit ${fraction.toFixed(3)}` : states[state]!;
    label.textContent = `angle = ${angle.toFixed(1)}° · ${stateText}`;
    demo.render();
  }, readout);

  return () => {
    stop();
    ov.dispose();
    demo.dispose();
  };
}

function readXf(d: Float32Array, k: number): { p: number[]; q: number[] } {
  return { p: [d[k]!, d[k + 1]!, d[k + 2]!], q: [d[k + 3]!, d[k + 4]!, d[k + 5]!, d[k + 6]!] };
}

function drawTOIShape(
  ov: Overlay,
  type: number,
  desc: number[],
  p: number[],
  q: number[],
  color: number,
) {
  if (type === 0) {
    // box, half extents in desc[0..2]
    ov.add(boxMesh(p, [desc[0]!, desc[1]!, desc[2]!], q, color, 0.5));
  } else if (type === 1) {
    // capsule, local c1/c2/radius
    const a = xfPoint(p, q, [desc[0]!, desc[1]!, desc[2]!]);
    const b = xfPoint(p, q, [desc[3]!, desc[4]!, desc[5]!]);
    ov.add(makeCapsule(a as [number, number, number], b as [number, number, number], desc[6]!, color, 0.6));
  } else {
    const a = xfPoint(p, q, [desc[0]!, desc[1]!, desc[2]!]);
    const b = xfPoint(p, q, [desc[3]!, desc[4]!, desc[5]!]);
    const c = xfPoint(p, q, [desc[6]!, desc[7]!, desc[8]!]);
    ov.segs(triSegs(a, b, c), color);
  }
}

// --- Capsule Cast Ray ------------------------------------------------------

function initCapsuleCastRay(container: HTMLElement) {
  const { wasm, controls, demo, readout } = makeViewer(container, {
    name: "Capsule Cast Ray",
    desc:
      "Official Collision sample <strong>Capsule Cast Ray</strong> — a fixed ray is cast " +
      "against a single kinematic capsule with <code>b3Body_CastRay</code>.",
    hint: "Static viewer",
    target: [0, 1.5, 0],
    distance: 20,
  });
  controls.appendChild(
    createInfoBox("<strong>Capsule Cast Ray</strong> — green/red mark the ray ends; orange is the hit."),
  );

  const pool = createMeshPool();
  const ov = new Overlay(demo.dynamic);
  wasm.ccray_reset();
  setView(demo, 120, 30, 20, [0, 1.5, 0]);

  const stop = runLoop(() => {
    syncMeshesFromPoses(demo.content, pool, wasm.ccray_poses(), { styles: undefined });
    ov.clear();
    addOriginAxes(ov);
    const d = wasm.ccray_cast();
    const o = [d[0]!, d[1]!, d[2]!];
    const e = [d[3]!, d[4]!, d[5]!];
    ov.line(o, e, CC.gray);
    ov.point(o, CC.green, 4);
    ov.point(e, CC.red, 4);
    if (d[6]! > 0.5) ov.point([d[7]!, d[8]!, d[9]!], CC.orange, 4);
    demo.render();
  }, readout);

  return () => {
    stop();
    ov.dispose();
    disposeMeshPool(pool);
    demo.dispose();
  };
}
