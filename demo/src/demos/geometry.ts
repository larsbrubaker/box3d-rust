// Geometry — the five static hull / mass viewers from sample_geometry.cpp
// (Box Hull, Hull, Hull Reduction, Hull Transform, Capsule Mass). Every hull,
// plane, mass and inertia value is computed by the ported Rust code in
// demo/wasm/src/geometry_demo.rs; this page only orbits the camera and turns the
// packed hull blocks into solid + wireframe meshes. No stepping world.

import {
  createButton,
  createButtonGroup,
  createCanvasOverlay,
  createInfoBox,
  createSlider,
} from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { demoPage, runLoop } from "./common.ts";
import {
  DemoScene,
  makeAxes,
  makeCapsule,
  makeTriangleMesh,
  makeWireEdges,
  setView,
} from "../three-scene.ts";

// Scene keys mirror the registry slugs for the Geometry category.
export const SCENES = [
  "box-hull",
  "hull",
  "hull-reduction",
  "hull-transform",
  "capsule-mass",
] as const;
type Scene = (typeof SCENES)[number];

// C debug palette (types.h): the exact colors the C sample's DrawHull /
// DrawSolidCapsule use, so the wireframes read the same as Erin's samples app.
const C_YELLOW = 0xffff00;
const C_CYAN = 0x00ffff;
const C_GREEN = 0x008000;
const C_BLUEVIOLET = 0x8a2be2;
const C_AQUA = 0x00ffff;

const SCENE_LABELS: Record<Scene, string> = {
  "box-hull": "Box Hull",
  hull: "Hull",
  "hull-reduction": "Hull Reduction",
  "hull-transform": "Hull Transform",
  "capsule-mass": "Capsule Mass",
};

interface HullBlock {
  surfaceArea: number;
  volume: number;
  innerRadius: number;
  vertexCount: number;
  faceCount: number;
  uniqueEdges: number;
  triangles: Float32Array;
  wire: Float32Array;
  empty: boolean;
}

/** Parse the packed hull-block stream (see geometry_demo.rs module docs). */
function parseHullBlocks(data: Float32Array): { blocks: HullBlock[]; next: number } {
  const count = data[0]!;
  const blocks: HullBlock[] = [];
  let off = 1;
  for (let b = 0; b < count; b++) {
    const surfaceArea = data[off]!;
    const volume = data[off + 1]!;
    const innerRadius = data[off + 2]!;
    const vertexCount = data[off + 3]!;
    const faceCount = data[off + 4]!;
    const uniqueEdges = data[off + 5]!;
    const triLen = data[off + 6]!;
    const wireLen = data[off + 7]!;
    off += 8;
    const triangles = data.subarray(off, off + triLen);
    off += triLen;
    const wire = data.subarray(off, off + wireLen);
    off += wireLen;
    blocks.push({
      surfaceArea,
      volume,
      innerRadius,
      vertexCount,
      faceCount,
      uniqueEdges,
      triangles,
      wire,
      empty: triLen === 0 && wireLen === 0,
    });
  }
  return { blocks, next: off };
}

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  assertRouteScenes("geometry", SCENES);

  let scene: Scene = SCENES.includes(initialScene as Scene)
    ? (initialScene as Scene)
    : "box-hull";

  const { canvas, controls, page } = demoPage(
    container,
    "Geometry",
    "The five static hull &amp; mass viewers from Erin Catto's Geometry samples. " +
      "Convex hulls, plane recomputation, hull reduction, transform/clone, and mass/inertia " +
      "all come from the ported <code>box3d_rust::hull</code> / <code>::geometry</code> code.",
    "Drag to orbit",
    wasm.version(),
  );

  const overlay = createCanvasOverlay(page);
  // `.sample-draw-text` is only styled under the samples-shell page variant;
  // this page uses the plain shell, so pin the C `DrawTextLine` overlay to the
  // canvas top-left inline (multi-line via `pre`).
  Object.assign(overlay.style, {
    position: "absolute",
    top: "12px",
    left: "12px",
    zIndex: "3",
    pointerEvents: "none",
    fontFamily: "var(--font-mono)",
    fontSize: "13px",
    lineHeight: "1.45",
    color: "#ffffff",
    textShadow: "0 1px 2px rgba(0, 0, 0, 0.45)",
    whiteSpace: "pre",
  });

  // --- Per-scene parameters (defaults mirror the C sample constructors) ---
  // Box Hull
  let bh = { h: [1.0, 0.5, 0.25], c: [0, 0, 0], r: [0, 0, 0], s: [1, 1, 1] };
  // Hull Reduction
  let hr = { kind: 1, count: 16 }; // C default: e_sphere, 16
  // Hull Transform
  let ht = { s: [1, 1, 1], r: [0, 0, 0], p: [0, 0, 0] };
  // Capsule Mass
  let cm = { sides: 6 };

  let dirty = true;
  const markDirty = () => {
    dirty = true;
  };

  // --- Controls: scene selector + a per-scene control block rebuilt on switch ---
  const sceneRow = createButtonGroup(
    SCENES.map((s) => ({ label: SCENE_LABELS[s], value: s })),
    scene,
    (v) => {
      scene = v as Scene;
      // Keep the deep link in sync without triggering a full re-navigation.
      history.replaceState(null, "", `#/geometry/${scene}`);
      buildSceneControls();
      markDirty();
    },
  );
  controls.appendChild(sceneRow);

  const sceneControls = document.createElement("div");
  controls.appendChild(sceneControls);

  function slider3(
    labels: [string, string, string],
    arr: number[],
    min: number,
    max: number,
    step: number,
  ) {
    labels.forEach((lbl, i) => {
      sceneControls.appendChild(
        createSlider(lbl, min, max, arr[i]!, step, (v) => {
          arr[i] = v;
          markDirty();
        }),
      );
    });
  }

  function buildSceneControls() {
    sceneControls.innerHTML = "";
    switch (scene) {
      case "box-hull":
        sceneControls.appendChild(
          createInfoBox(
            "Yellow is <code>b3CreateHull</code> of eight transformed + post-scaled box " +
              "corners; cyan is <code>b3MakeScaledBoxHull</code>. They should coincide.",
          ),
        );
        slider3(["h x", "h y", "h z"], bh.h, 0.1, 2.0, 0.1);
        slider3(["c x", "c y", "c z"], bh.c, -2.0, 2.0, 0.1);
        slider3(["r x°", "r y°", "r z°"], bh.r, -180, 180, 1);
        slider3(["s x", "s y", "s z"], bh.s, -2.0, 2.0, 0.1);
        sceneControls.appendChild(createButton("Refresh", markDirty));
        break;
      case "hull":
        sceneControls.appendChild(
          createInfoBox(
            "A fixed 48-point cloud scaled ×0.01 and reduced to at most 16 vertices by " +
              "<code>b3CreateHull</code>. No controls — matches the C sample.",
          ),
        );
        break;
      case "hull-reduction":
        sceneControls.appendChild(
          createInfoBox(
            "128 random points (seed 42) reduced to <code>count</code> vertices. Box clusters " +
              "on a cube; Sphere samples the unit sphere.",
          ),
        );
        sceneControls.appendChild(
          createButtonGroup(
            [
              { label: "Box", value: "0" },
              { label: "Sphere", value: "1" },
            ],
            String(hr.kind),
            (v) => {
              hr.kind = parseInt(v, 10);
              markDirty();
            },
          ),
        );
        sceneControls.appendChild(
          createSlider("count", 4, 128, hr.count, 1, (v) => {
            hr.count = Math.round(v);
            markDirty();
          }),
        );
        break;
      case "hull-transform":
        sceneControls.appendChild(
          createInfoBox(
            "Green is a 9-sided cylinder; yellow is <code>b3CloneAndTransformHull</code> of it " +
              "under the scale / rotation / offset sliders. Negative net scale reflects it.",
          ),
        );
        slider3(["s x", "s y", "s z"], ht.s, -2.0, 2.0, 0.1);
        slider3(["r x°", "r y°", "r z°"], ht.r, -180, 180, 1);
        slider3(["p x", "p y", "p z"], ht.p, -1.0, 1.0, 0.1);
        break;
      case "capsule-mass":
        sceneControls.appendChild(
          createInfoBox(
            "Mass and diagonal inertia of a tessellated capsule hull (yellow) vs the analytic " +
              "capsule (aqua) vs an enclosing box hull (blue-violet).",
          ),
        );
        sceneControls.appendChild(
          createSlider("sides", 3, 6, cm.sides, 1, (v) => {
            cm.sides = Math.round(v);
            markDirty();
          }),
        );
        break;
    }
  }
  buildSceneControls();

  const demo = new DemoScene(canvas, { distance: 5 });
  // C camera: SetView(yaw 0, pitch 15, radius 5, pivot origin) (sample_geometry.cpp:20).
  setView(demo, 0, 15, 5, [0, 0, 0]);

  /** Add a hull block as a faint solid + bright wireframe at an x offset. */
  function addHull(block: HullBlock, color: number, offsetX = 0) {
    if (block.empty) return;
    if (block.triangles.length > 0) {
      const solid = makeTriangleMesh(new Float32Array(block.triangles), color, 0.2);
      solid.position.x = offsetX;
      demo.content.add(solid);
    }
    if (block.wire.length > 0) {
      const wire = makeWireEdges(block.wire, color);
      wire.position.x = offsetX;
      demo.content.add(wire);
    }
  }

  const g = (x: number) => {
    // Compact %g-ish formatting for the in-scene readouts.
    if (x === 0) return "0";
    const a = Math.abs(x);
    if (a >= 1e-3 && a < 1e6) return parseFloat(x.toPrecision(4)).toString();
    return x.toExponential(3);
  };

  function rebuild() {
    demo.clearContent();
    demo.content.add(makeAxes(1.0));
    let text = "";

    if (scene === "box-hull") {
      const data = wasm.geometry_box_hull(
        bh.h[0]!, bh.h[1]!, bh.h[2]!,
        bh.c[0]!, bh.c[1]!, bh.c[2]!,
        bh.r[0]!, bh.r[1]!, bh.r[2]!,
        bh.s[0]!, bh.s[1]!, bh.s[2]!,
      );
      const { blocks } = parseHullBlocks(data);
      addHull(blocks[0]!, C_YELLOW);
      addHull(blocks[1]!, C_CYAN);
    } else if (scene === "hull") {
      const { blocks } = parseHullBlocks(wasm.geometry_hull());
      addHull(blocks[0]!, C_YELLOW);
      if (blocks[0]!.empty) text = "hull reduction produced no hull";
    } else if (scene === "hull-reduction") {
      const { blocks } = parseHullBlocks(wasm.geometry_hull_reduction(hr.kind, hr.count));
      const blk = blocks[0]!;
      addHull(blk, C_YELLOW);
      if (!blk.empty) {
        text = `v/f/e = ${blk.vertexCount}/${blk.faceCount}/${blk.uniqueEdges}`;
      }
    } else if (scene === "hull-transform") {
      const data = wasm.geometry_hull_transform(
        ht.s[0]!, ht.s[1]!, ht.s[2]!,
        ht.r[0]!, ht.r[1]!, ht.r[2]!,
        ht.p[0]!, ht.p[1]!, ht.p[2]!,
      );
      const { blocks } = parseHullBlocks(data);
      const orig = blocks[0]!;
      const clone = blocks[1]!;
      addHull(orig, C_GREEN, -2);
      addHull(clone, C_YELLOW, 2);
      text =
        `hull 1: area = ${g(orig.surfaceArea)}, volume = ${g(orig.volume)}, radius = ${g(orig.innerRadius)}\n` +
        (clone.empty
          ? "hull 2: (degenerate transform)"
          : `hull 2: area = ${g(clone.surfaceArea)}, volume = ${g(clone.volume)}, radius = ${g(clone.innerRadius)}`);
    } else if (scene === "capsule-mass") {
      const data = wasm.geometry_capsule_mass(cm.sides);
      const { blocks, next } = parseHullBlocks(data);
      const hullBlk = blocks[0]!;
      const boxBlk = blocks[1]!;
      const capLen = data[next]!;
      const capRadius = data[next + 1]!;
      const m = next + 2;
      const [
        massHull, massCap, massBox,
        ixxHull, ixxCap, ixxBox,
        iyyHull, iyyCap, iyyBox,
        izzHull, izzCap, izzBox,
      ] = [
        data[m]!, data[m + 1]!, data[m + 2]!,
        data[m + 3]!, data[m + 4]!, data[m + 5]!,
        data[m + 6]!, data[m + 7]!, data[m + 8]!,
        data[m + 9]!, data[m + 10]!, data[m + 11]!,
      ];

      // Analytic capsule (aqua) drawn along X from -capLen/2 to +capLen/2.
      demo.content.add(
        makeCapsule([-0.5 * capLen, 0, 0], [0.5 * capLen, 0, 0], capRadius, C_AQUA, 0.8),
      );
      addHull(boxBlk, C_BLUEVIOLET);
      addHull(hullBlk, C_YELLOW);

      text =
        `mass hull:    ${g(massHull)}\nmass capsule: ${g(massCap)}\nmass box:     ${g(massBox)}\n\n` +
        `Ixx hull:    ${g(ixxHull)}\nIxx capsule: ${g(ixxCap)}\nIxx box:     ${g(ixxBox)}\n\n` +
        `Iyy hull:    ${g(iyyHull)}\nIyy capsule: ${g(iyyCap)}\nIyy box:     ${g(iyyBox)}\n\n` +
        `Izz hull:    ${g(izzHull)}\nIzz capsule: ${g(izzCap)}\nIzz box:     ${g(izzBox)}`;
    }

    overlay.textContent = text;
  }

  const stop = runLoop(() => {
    if (dirty) {
      rebuild();
      dirty = false;
    }
    demo.render();
  }, overlay);

  return () => {
    stop();
    demo.dispose();
  };
}
