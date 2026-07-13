// Height Field — sample_mesh.cpp HeightField (:750). A wave / flat-grid height
// field probed every frame by a ray cast (b3World_CastRayClosest) or, when the
// radius is non-zero, a sphere shape cast (b3World_CastShape). Columns / rows /
// amplitude / holes rebuild the field; the ray x/z, delta x/z, and radius sliders
// drive the probe live. Camera (45, 30, 40) per the C ctor.

import type * as THREE from "three";
import { createCheckbox, createInfoBox, createReadout, createSlider, updateReadout } from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import {
  COLORS,
  DemoScene,
  makeArrow,
  makeAxes,
  makeDot,
  makeSegment,
  makeSphere,
  makeTriangleMesh,
  makeWireEdges,
  setView,
  trianglesFromWireframe,
} from "../three-scene.ts";

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Height Field",
    "A sinusoidal <code>b3CreateWave</code> (or flat <code>b3CreateGrid</code>) height field probed " +
      "by <code>b3World_CastRayClosest</code> and, when the radius is set, a sphere " +
      "<code>b3World_CastShape</code> — the two branches of the C <code>HeightField::Step</code>.",
    "Drag to orbit · tune the field and the ray",
    wasm.version(),
  );

  controls.appendChild(
    createInfoBox(
      "The C sample builds 400×400 in release / 10×10 in debug; rendering a 400×400 wireframe in the " +
        "browser is infeasible, so this serial-wasm build defaults to 40×40 and caps the sliders at 100 " +
        "(a disclosed scaling, not a changed C constant). Set the radius to 0 for a ray cast, or above 0 " +
        "for a sphere shape cast.",
    ),
  );

  // C HeightField defaults.
  const state = {
    columns: 40,
    rows: 40,
    amplitude: 0.75,
    holes: false,
    rayX: 5.5,
    rayZ: 1.01,
    deltaX: 0.0,
    deltaZ: 0.0,
    radius: 0.2,
  };
  const RAY_Y = 4.0; // C m_rayOrigin.y
  const DELTA_Y = -8.0; // C m_rayTranslation.y

  const demo = new DemoScene(canvas, { target: [0, 0, 0], distance: 40, shadowExtent: 120 });
  demo.content.add(makeAxes(2));

  let groundTri: THREE.Mesh | null = null;
  let groundWire: THREE.LineSegments | null = null;
  function clearGround() {
    for (const g of [groundTri, groundWire]) {
      if (!g) continue;
      demo.content.remove(g);
      g.geometry.dispose();
      (g.material as THREE.Material).dispose();
    }
    groundTri = null;
    groundWire = null;
  }
  function rebuildField() {
    clearGround();
    wasm.hf_reset(state.rows, state.columns, state.amplitude, state.holes);
    const wire = wasm.hf_wireframe();
    groundTri = makeTriangleMesh(trianglesFromWireframe(wire), COLORS.shape, 0.7);
    groundTri.receiveShadow = true;
    groundWire = makeWireEdges(wire, COLORS.muted);
    demo.content.add(groundTri);
    demo.content.add(groundWire);
  }

  // --- Controls ---
  controls.appendChild(
    createSlider("Columns", 1, 100, state.columns, 1, (v) => {
      state.columns = Math.round(v);
      rebuildField();
    }),
  );
  controls.appendChild(
    createSlider("Rows", 1, 100, state.rows, 1, (v) => {
      state.rows = Math.round(v);
      rebuildField();
    }),
  );
  controls.appendChild(
    createSlider("Amplitude", 0, 2, state.amplitude, 0.05, (v) => {
      state.amplitude = v;
      rebuildField();
    }),
  );
  controls.appendChild(
    createCheckbox("Holes", state.holes, (v) => {
      state.holes = v;
      rebuildField();
    }),
  );
  // Ray-probe sliders. The C ranges scale with the field extent; use the default
  // 40×40 · scale-2 extent (≈ ±40 m) so the whole field is reachable.
  const extent = 42;
  controls.appendChild(
    createSlider("Ray X", -extent, extent, state.rayX, 0.1, (v) => {
      state.rayX = v;
    }),
  );
  controls.appendChild(
    createSlider("Ray Z", -extent, extent, state.rayZ, 0.1, (v) => {
      state.rayZ = v;
    }),
  );
  controls.appendChild(
    createSlider("Delta X", -2 * extent, 2 * extent, state.deltaX, 0.1, (v) => {
      state.deltaX = v;
    }),
  );
  controls.appendChild(
    createSlider("Delta Z", -2 * extent, 2 * extent, state.deltaZ, 0.1, (v) => {
      state.deltaZ = v;
    }),
  );
  controls.appendChild(
    createSlider("Radius", 0, 1, state.radius, 0.05, (v) => {
      state.radius = v;
    }),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  rebuildField();
  setView(demo, 45, 30, 40, [0, 0, 0]);

  const stop = runLoop(() => {
    demo.clearDynamic();

    const ox = state.rayX;
    const oy = RAY_Y;
    const oz = state.rayZ;
    const tx = state.deltaX;
    const ty = DELTA_Y;
    const tz = state.deltaZ;
    // [hit, fraction, px,py,pz, nx,ny,nz]
    const r = wasm.hf_cast(ox, oy, oz, tx, ty, tz, state.radius);
    const hit = r[0] === 1;
    const frac = r[1]!;
    const end: [number, number, number] = [ox + tx, oy + ty, oz + tz];

    if (state.radius === 0) {
      // Ray-cast visualization (C HeightField::Step ray branch).
      demo.dynamic.add(makeDot([ox, oy, oz], 0xadff2f, 0.15)); // greenYellow origin
      demo.dynamic.add(makeDot(end, COLORS.hit, 0.15)); // red end
      demo.dynamic.add(makeSegment([ox, oy, oz], end, COLORS.muted));
      if (hit) {
        const p: [number, number, number] = [r[2]!, r[3]!, r[4]!];
        demo.dynamic.add(makeDot(p, 0xffa500, 0.2)); // orange hit
        demo.dynamic.add(makeArrow(p, [r[5]!, r[6]!, r[7]!], 0.5, 0x808080));
      }
    } else {
      // Shape-cast visualization (C HeightField::Step shape branch).
      demo.dynamic.add(makeDot([ox, oy, oz], COLORS.good, 0.1)); // green origin
      demo.dynamic.add(makeDot(end, COLORS.hit, 0.1)); // red end
      demo.dynamic.add(makeSegment([ox, oy, oz], end, 0xffd700)); // yellow ray
      const sc: [number, number, number] = [ox + frac * tx, oy + frac * ty, oz + frac * tz];
      demo.dynamic.add(makeSphere(sc[0], sc[1], sc[2], state.radius, 0xffa500, 0.55)); // swept sphere
      if (hit) {
        const p: [number, number, number] = [r[2]!, r[3]!, r[4]!];
        demo.dynamic.add(makeDot(p, 0x9370db, 0.2)); // purple hit
        demo.dynamic.add(makeArrow(p, [r[5]!, r[6]!, r[7]!], 0.5, COLORS.good));
      }
    }

    updateReadout(readout, [
      { label: "mode", value: state.radius === 0 ? "ray" : "shape cast" },
      { label: "hit", value: hit ? "yes" : "no" },
      { label: "fraction", value: frac.toFixed(4) },
    ]);

    demo.render();
  }, readout);

  return () => {
    stop();
    clearGround();
    demo.dispose();
  };
}
