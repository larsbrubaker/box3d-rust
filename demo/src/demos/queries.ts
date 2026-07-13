// Queries — Collision / Cast World
// Faithful Samples App UI for `sample_collision.cpp` CastWorld.

import * as THREE from "three";
import {
  createButton,
  createInfoBox,
  createSlider,
} from "../controls.ts";
import {
  attachInteraction,
  type InteractWasm,
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
  query_spawn_random(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  query_delete_at_ray(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): number;
  query_counters(): Float32Array;
  query_debug_draw(flags: number): Float32Array;
};
import { demoPage, runLoop } from "./common.ts";
import { COLORS, DemoScene, lineMat, makeWireEdges, setView } from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

/** ShapeType discriminant values from box3d geometry. */
const SHAPE_SPHERE = 5;
const SHAPE_CAPSULE = 0;
const SHAPE_HULL = 3;
const SHAPE_MESH = 4;
const SHAPE_HEIGHT = 2;

const HIT_COLORS = [0xdc2626, 0x16a34a, 0x2563eb];

function queryAsInteract(wasm: CastWorldWasm): InteractWasm {
  // `query_debug_text` is a per-demo export the Rust agent may add; guard it.
  const queryDebugText = (wasm as unknown as { query_debug_text?: () => string }).query_debug_text;
  return {
    sim_step: (dt, n) => wasm.query_step(dt, n),
    sim_body_poses: () => wasm.query_poses(),
    sim_mouse_down: (ox, oy, oz, tx, ty, tz) => wasm.query_mouse_down(ox, oy, oz, tx, ty, tz),
    sim_mouse_move: (px, py, pz) => wasm.query_mouse_move(px, py, pz),
    sim_mouse_up: () => wasm.query_mouse_up(),
    sim_mouse_active: () => wasm.query_mouse_active(),
    sim_spawn_random: (ox, oy, oz, tx, ty, tz) => wasm.query_spawn_random(ox, oy, oz, tx, ty, tz),
    sim_delete_at_ray: (ox, oy, oz, tx, ty, tz) => wasm.query_delete_at_ray(ox, oy, oz, tx, ty, tz),
    sim_counters: () => wasm.query_counters(),
    sim_debug_draw: (flags) => wasm.query_debug_draw(flags),
    // Debug-flag mask + draw scales are GLOBAL wasm exports; forward them so the
    // View menu / panel drive the Cast World overlay too.
    sim_set_debug_flags: (m) => wasm.sim_set_debug_flags(m),
    sim_set_draw_scales: (j, f) => wasm.sim_set_draw_scales(j, f),
    sim_debug_text: queryDebugText ? () => queryDebugText.call(wasm) : undefined,
  };
}

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

export function init(container: HTMLElement) {
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
    ignoreBoxes = makeWireEdges(edges, 0xfacc15, 0);
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
    surfaceWire = makeWireEdges(wire, 0x94a3b8, 0);
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
    wasm: queryAsInteract(wasm),
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
