// Mesh — the eight sample_mesh.cpp samples hosted by the `mesh` route (Height Field
// has its own page): Grid, Big Box, Box, Reflection, Viewer, Creation Benchmark,
// Voxel (large-world), and Hollow Box. Follows the shapes.ts samples-shell pattern
// (attachInteraction for grab/spawn/pause/camera/debug) with per-scene manual
// controls, plus the async OBJ-loading gate for the mesh-file scenes.

import * as THREE from "three";
import { createButton, createButtonGroup, createInfoBox, createSlider } from "../controls.ts";
import {
  attachInteraction,
  makeInteractAdapter,
  type ParamDef,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { demoPage, makeStyleGate, runLoop } from "./common.ts";
import {
  COLORS,
  DemoScene,
  makeArrow,
  makeTriangleMesh,
  makeWireBox,
  makeWireEdges,
  setView,
  trianglesFromWireframe,
} from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

type Scene =
  | "grid"
  | "big-box"
  | "box"
  | "reflection"
  | "viewer"
  | "creation-benchmark"
  | "voxel"
  | "hollow-box";

export const SCENES: Scene[] = [
  "grid",
  "big-box",
  "box",
  "reflection",
  "viewer",
  "creation-benchmark",
  "voxel",
  "hollow-box",
];

const NAMES: Record<Scene, string> = {
  grid: "Grid",
  "big-box": "Big Box",
  box: "Box",
  reflection: "Reflection",
  viewer: "Viewer",
  "creation-benchmark": "Creation Benchmark",
  voxel: "Voxel",
  "hollow-box": "Hollow Box",
};

// C m_camera->SetView(yaw, pitch, distance, target). Voxel renders in a base frame
// anchored at its origin, so its target is the C target minus the origin: {0,10,0}.
const CAMERAS: Record<Scene, [number, number, number, [number, number, number]]> = {
  grid: [45, 30, 6, [0, 0, 0]],
  "big-box": [45, 30, 6, [0, 0, 0]],
  box: [45, 30, 6, [0, 0, 0]],
  reflection: [45, 30, 40, [0, 0, 0]],
  viewer: [45, 30, 50, [0, 0, 0]],
  "creation-benchmark": [45, 30, 40, [0, 0, 0]],
  voxel: [-115, 5, 5, [0, 10, 0]],
  "hollow-box": [45, 30, 30, [0, 0, 0]],
};

const VOXEL_ORIGIN: [number, number, number] = [5000, 3500, -7000];

const SHAPE_IDS: Record<string, number> = { sphere: 0, capsule: 1, box: 2, cylinder: 3 };
const VIEWER_MESHES = [
  "voxel_mesh_01.obj",
  "voxel_mesh_02.obj",
  "voxel_mesh_03.obj",
  "voxel_mesh_04.obj",
];

// --- OBJ fetch cache (shared across resets) ---
const objCache = new Map<string, Promise<string>>();
function loadObj(name: string): Promise<string> {
  let p = objCache.get(name);
  if (!p) {
    p = fetch(`/public/meshes/${name}`).then((r) => {
      if (!r.ok) throw new Error(`${name} HTTP ${r.status}`);
      return r.text();
    });
    objCache.set(name, p);
  }
  return p;
}

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  assertRouteScenes("mesh", SCENES);
  const { canvas, controls } = demoPage(
    container,
    "Mesh",
    "The eight <code>sample_mesh.cpp</code> Mesh samples: grid / box / big-box mesh grounds " +
      "with a shape picker, the mirrored <code>building.obj</code> Reflection with 20 humans, the " +
      "voxel-mesh BVH Viewer, the Creation Benchmark, the large-world Voxel terrain, and the " +
      "zero-gravity Hollow Box.",
    "Ctrl+click grab · Shift+click spawn · P/O/R",
    wasm.version(),
    { category: "Mesh", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "<strong>Grid / Big Box / Box</strong> — drop a sphere/capsule/box/cylinder onto a mesh ground.<br>" +
        "<strong>Reflection</strong> — a building mesh and its X-mirror rain 20 ragdolls.<br>" +
        "<strong>Viewer</strong> — inspect a voxel mesh's BVH level by level.<br>" +
        "<strong>Creation Benchmark</strong> — time <code>b3CreateMesh</code> over four meshes.<br>" +
        "<strong>Voxel</strong> — a collision-mesh terrain 8&nbsp;km from the origin (large world).<br>" +
        "<strong>Hollow Box</strong> — 14 zero-gravity bodies inside a hollow box mesh.",
    ),
  );

  let scene: Scene =
    initialScene && SCENES.includes(initialScene as Scene) ? (initialScene as Scene) : "grid";
  let ctrl!: SimControllerWithTick;

  // Async gate for the OBJ-backed scenes (Viewer / Creation Benchmark / Voxel).
  let sceneReady = false;
  const hintEl = container.querySelector(".canvas-hint") as HTMLElement | null;
  const baseHint = hintEl?.textContent ?? "";
  const setLoading = (on: boolean) => {
    if (hintEl) hintEl.textContent = on ? "Loading mesh…" : baseHint;
  };

  const demo = new DemoScene(canvas, { target: [0, 0, 0], distance: 20, shadowExtent: 60 });
  demo.camera.far = 2000;
  demo.camera.updateProjectionMatrix();
  const pool = createMeshPool();

  // --- Static mesh-ground wireframe (rebuilt on reset) ---
  const groundGroup = new THREE.Group();
  demo.content.add(groundGroup);
  function clearGround() {
    for (const child of [...groundGroup.children]) {
      groundGroup.remove(child);
      const m = child as THREE.Mesh | THREE.LineSegments;
      m.geometry.dispose();
      const mat = m.material as THREE.Material | THREE.Material[];
      if (Array.isArray(mat)) mat.forEach((x) => x.dispose());
      else mat.dispose();
    }
  }
  function buildGround() {
    clearGround();
    const wire = wasm.mesh_ground_wireframe();
    if (!wire.length) return;
    const tri = makeTriangleMesh(trianglesFromWireframe(wire), 0x8a94a6, 0.9);
    tri.receiveShadow = true;
    groundGroup.add(tri);
    groundGroup.add(makeWireEdges(wire, 0x4a5568, 0.35));
  }

  // --- Voxel hull wireframe (dynamic, rebuilt each frame) ---
  const hullGroup = new THREE.Group();
  demo.dynamic.add(hullGroup);

  // --- Viewer BVH node overlay (rebuilt on draw-level change) ---
  const bvhGroup = new THREE.Group();
  demo.content.add(bvhGroup);
  const BVH_COLORS = [
    0xf0f8ff, 0xfaebd7, 0x00ffff, 0x7fffd4, 0xf0ffff, 0xf5f5dc, 0xffe4c4, 0xffebcd, 0x0000ff,
    0x8a2be2, 0xa52a2a, 0xdeb887, 0x5f9ea0, 0x7fff00, 0xd2691e, 0xff7f50, 0x6495ed, 0xfff8dc,
    0xdc143c, 0x00ffff,
  ];
  function clearBvh() {
    for (const child of [...bvhGroup.children]) {
      bvhGroup.remove(child);
      const anyChild = child as THREE.LineSegments | THREE.ArrowHelper;
      if ((anyChild as THREE.LineSegments).geometry) {
        (anyChild as THREE.LineSegments).geometry.dispose();
        const mat = (anyChild as THREE.LineSegments).material as THREE.Material;
        if (mat && mat.dispose) mat.dispose();
      }
      (anyChild as THREE.ArrowHelper).dispose?.();
    }
  }
  function buildBvh(level: number) {
    clearBvh();
    if (scene !== "viewer" || level < 0) return;
    const nodes = wasm.mesh_viewer_nodes(level); // 7 floats/node: lx,ly,lz,ux,uy,uz,axis
    const color = BVH_COLORS[level % BVH_COLORS.length]!;
    for (let i = 0; i + 6 < nodes.length; i += 7) {
      const lx = nodes[i]!, ly = nodes[i + 1]!, lz = nodes[i + 2]!;
      const ux = nodes[i + 3]!, uy = nodes[i + 4]!, uz = nodes[i + 5]!;
      const axis = nodes[i + 6]! | 0;
      bvhGroup.add(
        makeWireBox((lx + ux) / 2, (ly + uy) / 2, (lz + uz) / 2, (ux - lx) / 2, (uy - ly) / 2, (uz - lz) / 2, color),
      );
      const cx = (lx + ux) / 2, cy = (ly + uy) / 2, cz = (lz + uz) / 2;
      if (axis === 0) bvhGroup.add(makeArrow([cx, cy, cz], [1, 0, 0], 0.1, 0xff0000));
      else if (axis === 1) bvhGroup.add(makeArrow([cx, cy, cz], [0, 1, 0], 0.1, 0x00ff00));
      else if (axis === 2) bvhGroup.add(makeArrow([cx, cy, cz], [0, 0, 1], 0.1, 0x0000ff));
    }
  }

  // ---------------------------------------------------------------------------
  // Per-scene state + manual controls
  // ---------------------------------------------------------------------------
  let shapeId = 3; // default cylinder (Grid/Big Box); Box overrides to box below
  let scaleX = 2;
  let scaleZ = 2;
  const refl = { x: -1, y: 1, z: 1 };
  const viewer = { index: 0, median: true, concave: true, weld: true, tolMm: 1.5, level: -1 };
  let benchStat = "—";

  const sceneControls = document.createElement("div");

  function sceneId(): number {
    return scene === "big-box" ? 1 : scene === "box" ? 2 : 0;
  }

  // Rebuild the manual control block for the current scene.
  function rebuildSceneControls() {
    sceneControls.replaceChildren();

    if (scene === "grid" || scene === "big-box" || scene === "box") {
      const shapeName = Object.keys(SHAPE_IDS).find((k) => SHAPE_IDS[k] === shapeId) ?? "cylinder";
      sceneControls.appendChild(
        createButtonGroup(
          [
            { label: "Sphere", value: "sphere" },
            { label: "Capsule", value: "capsule" },
            { label: "Box", value: "box" },
            { label: "Cylinder", value: "cylinder" },
          ],
          shapeName,
          (v) => {
            shapeId = SHAPE_IDS[v]!;
            wasm.mesh_set_shape(shapeId); // C Spawn(): live re-drop, ground untouched
          },
        ),
      );
      sceneControls.appendChild(
        createSlider("Scale X", -2, 2, scaleX, 0.1, (v) => {
          scaleX = v;
          reset(); // C b3Shape_SetMesh re-scales the ground; the port rebuilds it
        }),
      );
      sceneControls.appendChild(
        createSlider("Scale Z", -2, 2, scaleZ, 0.1, (v) => {
          scaleZ = v;
          reset();
        }),
      );
    } else if (scene === "reflection") {
      // C MeshReflection::DrawControls — six sign radios on the mirrored building.
      const axisRow = (label: string, key: "x" | "y" | "z") =>
        createButtonGroup(
          [
            { label: `Neg ${label}`, value: "neg" },
            { label: `Pos ${label}`, value: "pos" },
          ],
          refl[key] < 0 ? "neg" : "pos",
          (v) => {
            refl[key] = v === "neg" ? -1 : 1;
            reset();
          },
        );
      sceneControls.append(axisRow("X", "x"), axisRow("Y", "y"), axisRow("Z", "z"));
    } else if (scene === "viewer") {
      sceneControls.appendChild(
        createSlider("Index", 0, 3, viewer.index, 1, (v) => {
          viewer.index = Math.round(v);
          reset();
        }),
      );
      sceneControls.appendChild(
        createButtonGroup(
          [
            { label: "Median split", value: "median" },
            { label: "SAH binning", value: "sah" },
          ],
          viewer.median ? "median" : "sah",
          (v) => {
            viewer.median = v === "median";
            reset();
          },
        ),
      );
      sceneControls.appendChild(
        createSlider("Tolerance (mm)", 0, 10, viewer.tolMm, 0.1, (v) => {
          viewer.tolMm = v;
          reset();
        }),
      );
      sceneControls.appendChild(
        createSlider("Draw level", -1, Math.max(0, wasm.mesh_viewer_height()), viewer.level, 1, (v) => {
          viewer.level = Math.round(v);
          buildBvh(viewer.level); // live, no rebuild
        }),
      );
    } else if (scene === "creation-benchmark") {
      sceneControls.appendChild(
        createButton("Run benchmark", () => runBenchmark()),
      );
    }
  }

  // Creation Benchmark: build all four meshes over N iterations, keep the minimum
  // wall time (C's b3MinFloat over b3GetMilliseconds), report total + per-mesh.
  function runBenchmark() {
    if (scene !== "creation-benchmark") return;
    const iterations = 10;
    let best = Infinity;
    let triangles = 0;
    for (let i = 0; i < iterations; i++) {
      const t0 = performance.now();
      triangles = wasm.mesh_benchmark_build();
      best = Math.min(best, performance.now() - t0);
    }
    benchStat = `${best.toFixed(3)} ms total · ${(best / 4).toFixed(3)} ms/mesh · ${triangles} tris`;
  }

  function setCamera() {
    const [yaw, pitch, dist, target] = CAMERAS[scene];
    setView(demo, yaw, pitch, dist, target);
    ctrl?.setWorldOrigin(scene === "voxel" ? VOXEL_ORIGIN : [0, 0, 0]);
  }

  function reset(): Promise<void> {
    clearGround();
    clearBvh();
    for (const child of [...hullGroup.children]) {
      hullGroup.remove(child);
      const m = child as THREE.LineSegments;
      m.geometry.dispose();
      (m.material as THREE.Material).dispose();
    }
    syncMeshesFromPoses(demo.content, pool, []);
    setCamera();

    // Reset per-scene defaults when entering a scene fresh.
    if (scene === "grid") {
      shapeId = 3;
      scaleX = 2;
      scaleZ = 2;
    } else if (scene === "big-box") {
      shapeId = 3;
      scaleX = 1;
      scaleZ = 1;
    } else if (scene === "box") {
      shapeId = 2;
      scaleX = 1;
      scaleZ = 1;
    }

    if (scene === "grid" || scene === "big-box" || scene === "box") {
      wasm.mesh_reset(sceneId(), shapeId, scaleX, scaleZ);
      buildGround();
      sceneReady = true;
      return Promise.resolve();
    }
    if (scene === "reflection") {
      wasm.mesh_reset_reflection(refl.x, refl.y, refl.z);
      buildGround();
      sceneReady = true;
      return Promise.resolve();
    }
    if (scene === "hollow-box") {
      wasm.mesh_reset_hollow_box();
      buildGround();
      sceneReady = true;
      return Promise.resolve();
    }

    // --- Async OBJ scenes ---
    sceneReady = false;
    setLoading(true);
    const activeScene = scene;
    if (scene === "voxel") {
      return loadObj("collision_mesh_01.obj")
        .then((txt) => {
          if (scene !== activeScene) return;
          wasm.mesh_reset_voxel(txt);
          buildGround();
          setCamera();
          sceneReady = true;
          setLoading(false);
        })
        .catch((e) => {
          console.warn("voxel mesh load failed", e);
          setLoading(false);
        });
    }
    if (scene === "viewer") {
      return loadObj(VIEWER_MESHES[viewer.index]!)
        .then((txt) => {
          if (scene !== activeScene) return;
          wasm.mesh_reset_viewer(txt, viewer.median, viewer.concave, viewer.weld, viewer.tolMm);
          buildGround();
          buildBvh(viewer.level);
          rebuildSceneControls(); // refresh the draw-level slider max
          sceneReady = true;
          setLoading(false);
        })
        .catch((e) => {
          console.warn("viewer mesh load failed", e);
          setLoading(false);
        });
    }
    // creation-benchmark
    return Promise.all(VIEWER_MESHES.map(loadObj))
      .then(([o1, o2, o3, o4]) => {
        if (scene !== activeScene) return;
        wasm.mesh_reset_benchmark(o1!, o2!, o3!, o4!);
        runBenchmark();
        sceneReady = true;
        setLoading(false);
      })
      .catch((e) => {
        console.warn("benchmark mesh load failed", e);
        setLoading(false);
      });
  }

  const params: ParamDef[] = [
    {
      type: "select",
      key: "sample",
      label: "Sample",
      options: SCENES.map((s) => ({ label: NAMES[s], value: s })),
      default: scene,
      restart: true,
    },
  ];

  ctrl = attachInteraction({
    wasm: makeInteractAdapter(wasm, "mesh"),
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: NAMES[scene],
    sampleCategory: "Mesh",
    enableSpawnDelete: true,
    params,
    onParamsChange: (values, key) => {
      if (key === "sample") {
        scene = values.sample as Scene;
        ctrl.setSampleName(NAMES[scene]);
        rebuildSceneControls();
        // restart re-applies the active scene via reset()
      }
    },
  }) as SimControllerWithTick;

  // Place the per-scene controls right below the parameter panel.
  const paramPanel = controls.querySelector(".param-panel");
  if (paramPanel && paramPanel.parentElement) {
    paramPanel.parentElement.insertBefore(sceneControls, paramPanel.nextSibling);
  } else {
    controls.appendChild(sceneControls);
  }
  rebuildSceneControls();

  reset();

  // Stats readout appended at the bottom.
  const readout = document.createElement("div");
  readout.className = "sample-stat mesh-stats";
  readout.style.marginTop = "0.5rem";
  controls.appendChild(readout);

  function statsText(): string {
    const s = wasm.mesh_stats();
    switch (scene) {
      case "grid":
      case "big-box":
      case "box":
        return `triangles ${s[0] ?? 0} · bytes ${s[1] ?? 0}`;
      case "reflection":
        return `building triangles ${s[0] ?? 0} · scale (${refl.x}, ${refl.y}, ${refl.z})`;
      case "viewer":
        return `tris ${s[0] ?? 0} · verts ${s[1] ?? 0} · degenerate ${s[2] ?? 0} · height ${s[3] ?? 0} · node area ${(s[4] ?? 0).toFixed(2)}`;
      case "creation-benchmark":
        return benchStat;
      case "voxel":
        return `terrain triangles ${s[0] ?? 0}`;
      case "hollow-box":
        return `mesh triangles ${s[0] ?? 0}`;
    }
  }

  const styleGate = makeStyleGate<Uint32Array>();
  let frame = 0;
  const stop = runLoop(
    () => {
      ctrl.tickFrame();
      const awake = wasm.mesh_counters()[5] ?? 0;
      const styles = styleGate(awake, () => wasm.mesh_styles());
      syncMeshesFromPoses(demo.content, pool, wasm.mesh_poses(), { groundIndex: null, styles });

      // Voxel's dynamic hull renders as a live wireframe overlay.
      if (scene === "voxel") {
        for (const child of [...hullGroup.children]) {
          hullGroup.remove(child);
          const m = child as THREE.LineSegments;
          m.geometry.dispose();
          (m.material as THREE.Material).dispose();
        }
        const hw = wasm.mesh_voxel_hull_wireframe();
        if (hw.length >= 6) hullGroup.add(makeWireEdges(hw, COLORS.accent));
      }

      frame += 1;
      if (frame % 6 === 0) readout.textContent = statsText();
      demo.render();
    },
    controls,
    { ready: () => sceneReady },
  );

  return () => {
    stop();
    clearGround();
    clearBvh();
    demo.content.remove(groundGroup);
    demo.content.remove(bvhGroup);
    demo.dynamic.remove(hullGroup);
    ctrl.dispose();
    disposeMeshPool(pool);
    demo.dispose();
  };
}
