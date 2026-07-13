// Issues — faithful Samples-App ports of sample_issues.cpp.
// Each C `RegisterSample( "Issues", … )` maps to one scene selected by the registry
// slug and dispatched by `init`. Dynamic bodies come from `issues_poses()`
// (box/sphere/capsule); static collision geometry is a gray wireframe; Convex
// Jitter's two arbitrary hulls ride a dedicated geometry channel; Hull Crash is a
// static hull/points render.

import * as THREE from "three";
import { createButton, createInfoBox } from "../controls.ts";
import {
  attachInteraction,
  makeInteractAdapter,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { demoPage, makeStyleGate, runLoop } from "./common.ts";
import {
  DemoScene,
  lineMat,
  makeWireEdges,
  setView,
  solidMat,
} from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

/** Scene keys hosted by this page (validated against the registry in registry.test). */
export const SCENES = [
  "dump-loader",
  "crash",
  "multiple-prismatic",
  "hull-crash",
  "convex-jitter",
  "s-box-mover",
  "capsule-mesh",
] as const;

type Scene = (typeof SCENES)[number];

const GRAY = 0x94a3b8;
const YELLOW = 0xfacc15;
const WHITE = 0xffffff;
const HULL_COLOR = 0x64748b;

interface SceneConfig {
  name: string;
  reset: string; // wasm export name
  camera: { yaw: number; pitch: number; distance: number; target: [number, number, number] };
  desc: string;
  hint: string;
  /** Crash exposes an "Add Joint" button; other scenes do not. */
  addJoint?: boolean;
  /** Hull Crash is a static hull/points render with no simulated bodies. */
  staticHull?: boolean;
}

const CONFIG: Record<Scene, SceneConfig> = {
  "dump-loader": {
    name: "Dump Loader",
    reset: "issues_reset_dump_loader",
    camera: { yaw: 45, pitch: 30, distance: 15, target: [0, 2, 0] },
    desc:
      "Official Issues sample <strong>Dump Loader</strong> — a recorded scene (one rotated " +
      "cube on a large ground box) rebuilt from <code>box3d_dump.inl</code>.",
    hint: "Recorded single_box dump",
  },
  crash: {
    name: "Crash",
    reset: "issues_reset_crash",
    camera: { yaw: 45, pitch: 30, distance: 15, target: [0, 2, 0] },
    desc:
      "Official Issues sample <strong>Crash</strong> — two boxes over a grid mesh; " +
      "press <strong>Add Joint</strong> to weld them together.",
    hint: "Add Joint welds the two boxes",
    addJoint: true,
  },
  "multiple-prismatic": {
    name: "Multiple Prismatic",
    reset: "issues_reset_multiple_prismatic",
    camera: { yaw: 0, pitch: 0, distance: 25, target: [0, 5, 0] },
    desc:
      "Official Issues sample <strong>Multiple Prismatic</strong> — six boxes chained by " +
      "prismatic joints (limit ±6, constraint hertz 240).",
    hint: "Stacked prismatic joints",
  },
  "hull-crash": {
    name: "Hull Crash",
    reset: "issues_reset_hull_crash",
    camera: { yaw: 0, pitch: 15, distance: 5, target: [0, 0, 0] },
    desc:
      "Official Issues sample <strong>Hull Crash</strong> — a nearly-coplanar point set fed " +
      "through <code>b3CreateHull</code>; the raw points show when the builder rejects them.",
    hint: "Degenerate hull robustness",
    staticHull: true,
  },
  "convex-jitter": {
    name: "Convex Jitter",
    reset: "issues_reset_convex_jitter",
    camera: { yaw: 0, pitch: 15, distance: 10, target: [0, 2, 0] },
    desc:
      "Official Issues sample <strong>Convex Jitter</strong> — two precise 16-/18-point hulls " +
      "at scale 0.01 rest on a ground box.",
    hint: "Precision convex resting",
  },
  "s-box-mover": {
    name: "s&box mover",
    reset: "issues_reset_sbox_mover",
    camera: { yaw: 45, pitch: 30, distance: 12, target: [0, 0, 0] },
    desc:
      "Official Issues sample <strong>s&box mover</strong> — an angular-locked box drops onto " +
      "a height-field grid and a platform mesh.",
    hint: "Height field + platform mesh",
  },
  "capsule-mesh": {
    name: "Capsule Mesh",
    reset: "issues_reset_capsule_mesh",
    camera: { yaw: 20, pitch: 10, distance: 30, target: [0, 2, 0] },
    desc:
      "Official Issues sample <strong>Capsule Mesh</strong> — the player-controller repro: a " +
      "locked magenta capsule drops onto <code>building.obj</code>.",
    hint: "Locked capsule vs building mesh",
  },
};

export function init(container: HTMLElement, initialScene?: string) {
  assertRouteScenes("issues", SCENES);
  const scene: Scene =
    initialScene && (SCENES as readonly string[]).includes(initialScene)
      ? (initialScene as Scene)
      : "crash";
  const cfg = CONFIG[scene];
  const wasm = getWasm();

  const { canvas, controls } = demoPage(
    container,
    "Issues",
    cfg.desc,
    cfg.hint,
    wasm.version(),
    { category: "Issues", samplesShell: true },
  );
  controls.appendChild(createInfoBox(cfg.desc));

  const demo = new DemoScene(canvas, { target: cfg.camera.target, distance: cfg.camera.distance });
  demo.camera.far = 800;
  demo.camera.updateProjectionMatrix();
  const pool = createMeshPool();
  const styleGate = makeStyleGate<Uint32Array>();

  // Static collision-mesh / height-field wireframe (rebuilt on reset).
  let staticWire: THREE.LineSegments | null = null;
  function clearStaticWire() {
    if (staticWire) {
      demo.dynamic.remove(staticWire);
      staticWire.geometry.dispose();
      (staticWire.material as THREE.Material).dispose();
      staticWire = null;
    }
  }

  // Convex Jitter arbitrary-hull bodies: solid mesh + wire per body, transformed each frame.
  const hullMeshes: THREE.Mesh[] = [];
  const hullWires: THREE.LineSegments[] = [];
  function clearHulls() {
    for (const m of hullMeshes) {
      demo.dynamic.remove(m);
      m.geometry.dispose();
      (m.material as THREE.Material).dispose();
    }
    for (const w of hullWires) {
      demo.dynamic.remove(w);
      w.geometry.dispose();
      (w.material as THREE.Material).dispose();
    }
    hullMeshes.length = 0;
    hullWires.length = 0;
  }

  // Hull Crash static render group.
  const staticGroup = new THREE.Group();
  demo.dynamic.add(staticGroup);
  function clearStaticGroup() {
    for (let i = staticGroup.children.length - 1; i >= 0; i--) {
      const o = staticGroup.children[i] as THREE.Mesh | THREE.LineSegments | THREE.Points;
      staticGroup.remove(o);
      (o.geometry as THREE.BufferGeometry).dispose();
      (o.material as THREE.Material).dispose();
    }
  }

  function buildStaticWire() {
    clearStaticWire();
    const wire = wasm.issues_static_wireframe();
    if (wire.length) {
      staticWire = makeWireEdges(wire, GRAY, 0.6);
      demo.dynamic.add(staticWire);
    }
  }

  function buildHulls() {
    clearHulls();
    const geo = wasm.issues_hull_geometry();
    let p = 0;
    while (p < geo.length) {
      const triCount = geo[p++]! | 0;
      const tris = geo.slice(p, p + triCount);
      p += triCount;
      const edgeCount = geo[p++]! | 0;
      const edges = geo.slice(p, p + edgeCount);
      p += edgeCount;

      const g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.BufferAttribute(new Float32Array(tris), 3));
      g.computeVertexNormals();
      const mesh = new THREE.Mesh(g, solidMat(HULL_COLOR, 1));
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      demo.dynamic.add(mesh);
      hullMeshes.push(mesh);

      const wire = makeWireEdges(edges, 0x1e293b);
      demo.dynamic.add(wire);
      hullWires.push(wire);
    }
  }

  function buildStaticHull() {
    clearStaticGroup();
    const d = wasm.issues_hull_crash();
    let p = 0;
    const ok = d[p++]! > 0.5;
    const triCount = d[p++]! | 0;
    const tris = d.slice(p, p + triCount);
    p += triCount;
    const edgeCount = d[p++]! | 0;
    const edges = d.slice(p, p + edgeCount);
    p += edgeCount;
    const ptCount = d[p++]! | 0;
    const pts = d.slice(p, p + ptCount);

    if (ok && triCount > 0) {
      const g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.BufferAttribute(new Float32Array(tris), 3));
      g.computeVertexNormals();
      staticGroup.add(new THREE.Mesh(g, solidMat(YELLOW, 0.85)));
      staticGroup.add(makeWireEdges(edges, 0x1e293b));
    } else if (ptCount > 0) {
      const g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.BufferAttribute(new Float32Array(pts), 3));
      staticGroup.add(
        new THREE.Points(
          g,
          new THREE.PointsMaterial({ color: WHITE, size: 6, sizeAttenuation: false }),
        ),
      );
    }
    // World axes at the origin (C DrawAxes identity, length 1).
    staticGroup.add(makeOriginAxes(1));
  }

  function reset() {
    (wasm as unknown as Record<string, () => number>)[cfg.reset]!();
    setView(demo, cfg.camera.yaw, cfg.camera.pitch, cfg.camera.distance, cfg.camera.target);
    buildStaticWire();
    buildHulls();
    if (cfg.staticHull) buildStaticHull();
  }

  if (cfg.addJoint) {
    const row = document.createElement("div");
    row.className = "control-row";
    row.appendChild(createButton("Add Joint", () => wasm.issues_add_joint(), false));
    controls.appendChild(row);
  }

  const ctrl = attachInteraction({
    wasm: makeInteractAdapter(wasm, "issues"),
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: cfg.name,
    sampleCategory: "Issues",
    enableSpawnDelete: !cfg.staticHull,
    params: [],
  }) as SimControllerWithTick;

  reset();

  const readout = controls.querySelector(".info-readout") as HTMLElement | null;
  const stop = runLoop(() => {
    ctrl.tickFrame();
    const awake = wasm.issues_counters()[5] ?? 0;
    syncMeshesFromPoses(demo.content, pool, wasm.issues_poses(), {
      groundIndex: null,
      styles: styleGate(awake, () => wasm.issues_styles()),
    });

    // Update the Convex Jitter hull transforms.
    if (hullMeshes.length) {
      const hp = wasm.issues_hull_poses();
      for (let i = 0; i < hullMeshes.length; i++) {
        const o = i * 7;
        const mesh = hullMeshes[i]!;
        const wire = hullWires[i]!;
        mesh.position.set(hp[o]!, hp[o + 1]!, hp[o + 2]!);
        mesh.quaternion.set(hp[o + 3]!, hp[o + 4]!, hp[o + 5]!, hp[o + 6]!);
        wire.position.copy(mesh.position);
        wire.quaternion.copy(mesh.quaternion);
      }
    }
    demo.render();
  }, readout ?? undefined);

  return () => {
    stop();
    clearStaticWire();
    clearHulls();
    clearStaticGroup();
    staticGroup.parent?.remove(staticGroup);
    disposeMeshPool(pool);
    ctrl.dispose();
    demo.dispose();
  };
}

/** Three colored origin-axis lines (C DrawAxes: red X, green Y, blue Z). */
function makeOriginAxes(len: number): THREE.LineSegments {
  const g = new THREE.BufferGeometry();
  g.setAttribute(
    "position",
    new THREE.Float32BufferAttribute(
      [0, 0, 0, len, 0, 0, 0, 0, 0, 0, len, 0, 0, 0, 0, 0, 0, len],
      3,
    ),
  );
  g.setAttribute(
    "color",
    new THREE.Float32BufferAttribute(
      [1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1],
      3,
    ),
  );
  const mat = lineMat(0xffffff);
  mat.vertexColors = true;
  return new THREE.LineSegments(g, mat);
}
