// Mesh — box/grid mesh with ray cast and AABB (Three.js BufferGeometry).

import { createButtonGroup, createInfoBox, createReadout, updateReadout } from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { demoPage, runLoop } from "./common.ts";

/** Scene keys for this multi-scene page (registry `scene` values for `mesh`). */
export const SCENES = ["box", "grid"] as const;
import {
  COLORS,
  DemoScene,
  makeAxes,
  makeDot,
  makeSegment,
  makeTriangleMesh,
  makeWireBox,
  makeWireEdges,
  setView,
  trianglesFromWireframe,
} from "../three-scene.ts";

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  // Self-check the scene table against the registry (see registry.ts pattern).
  assertRouteScenes("mesh", SCENES);
  const initialMesh = initialScene === "grid" ? "grid" : "box";
  const { canvas, controls } = demoPage(
    container,
    "Mesh",
    "Triangle meshes from <code>b3CreateBoxMesh</code> / <code>b3CreateGridMesh</code>. " +
      "Ray casts use <code>b3RayCastMesh</code>; the AABB comes from " +
      "<code>b3ComputeMeshAABB</code>.",
    "Drag to orbit · ray sweeps automatically",
    wasm.version(),
  );

  let stats = initialMesh === "grid" ? wasm.mesh_build_grid() : wasm.mesh_build_box();
  let wire = wasm.mesh_wireframe();
  let aabb = wasm.mesh_aabb();

  controls.appendChild(
    createInfoBox(
      "Switch between a box mesh and a flat grid. Hits report triangle index from the " +
        "ported mesh BVH ray cast. Geometry is a Three.js BufferGeometry built from wasm triangles.",
    ),
  );
  controls.appendChild(
    createButtonGroup(
      [
        { label: "Box", value: "box" },
        { label: "Grid", value: "grid" },
      ],
      initialMesh,
      (v) => {
        stats = v === "grid" ? wasm.mesh_build_grid() : wasm.mesh_build_box();
        wire = wasm.mesh_wireframe();
        aabb = wasm.mesh_aabb();
        rebuildMesh();
      },
    ),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { distance: 11 });
  // Preserve this sample's current flatter framing (yaw 35°, pitch 20°) now that
  // the DemoScene default is the C camera (pitch -25°). The C per-sample camera
  // for Mesh lands with this sample's batch-3 rebuild.
  setView(demo, 35, 20, 11, [0, 0, 0]);

  function rebuildMesh() {
    demo.clearContent();
    demo.content.add(makeAxes(1.5));
    const positions = trianglesFromWireframe(wire);
    demo.content.add(makeTriangleMesh(positions, COLORS.accent, 0.55));
    demo.content.add(makeWireEdges(wire, COLORS.accent));
    demo.content.add(
      makeWireBox(
        (aabb[0]! + aabb[3]!) / 2,
        (aabb[1]! + aabb[4]!) / 2,
        (aabb[2]! + aabb[5]!) / 2,
        (aabb[3]! - aabb[0]!) / 2,
        (aabb[4]! - aabb[1]!) / 2,
        (aabb[5]! - aabb[2]!) / 2,
        COLORS.muted,
      ),
    );
  }

  rebuildMesh();
  const start = performance.now();

  const stop = runLoop(() => {
    demo.clearDynamic();

    const t = (performance.now() - start) / 1000;
    const ox = Math.cos(t * 0.8) * 4;
    const oy = 3;
    const oz = Math.sin(t * 0.8) * 4;
    const tx = -ox * 1.5;
    const ty = -6;
    const tz = -oz * 1.5;
    const hit = wasm.mesh_ray_cast(ox, oy, oz, tx, ty, tz);
    const frac = hit[0] === 1.0 ? hit[1]! : 1.0;
    demo.dynamic.add(
      makeSegment(
        [ox, oy, oz],
        [ox + tx * frac, oy + ty * frac, oz + tz * frac],
        COLORS.good,
      ),
    );
    demo.dynamic.add(makeDot([ox, oy, oz], COLORS.good, 0.1));
    if (hit[0] === 1.0) {
      demo.dynamic.add(makeDot([hit[2]!, hit[3]!, hit[4]!], COLORS.hit, 0.1));
    }

    updateReadout(readout, [
      { label: "Vertices", value: String(stats[0]) },
      { label: "Triangles", value: String(stats[1]) },
      { label: "Hit", value: hit[0] === 1.0 ? "yes" : "no" },
      { label: "Triangle", value: hit[0] === 1.0 ? String(hit[8]) : "—" },
    ]);

    demo.render();
  }, readout);

  return () => {
    stop();
    demo.dispose();
  };
}
