// Hull — Box Hull wireframe (Three.js). Partial port of the C Geometry / Box Hull sample.

import { createInfoBox, createReadout, createSlider, updateReadout } from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import {
  COLORS,
  DemoScene,
  makeAxes,
  makeSolidBox,
  makeWireEdges,
} from "../three-scene.ts";

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Box Hull",
    "Convex box hull from the ported <code>b3MakeBoxHull</code>. Wireframe edges come " +
      "straight from the Rust hull half-edge mesh. Partial port of the C Geometry / Box Hull sample.",
    "Drag to orbit",
    wasm.version(),
  );

  let hx = 1.2;
  let hy = 0.8;
  let hz = 1.0;

  controls.appendChild(
    createInfoBox(
      "Box hulls use the embedded <code>b3BoxHull</code> template. Solid fill + wireframe edges. " +
        "The C sample's c/r/s sliders and dual-hull compare are not yet ported.",
    ),
  );
  controls.appendChild(createSlider("Half X", 0.3, 2.5, hx, 0.1, (v) => { hx = v; }));
  controls.appendChild(createSlider("Half Y", 0.3, 2.5, hy, 0.1, (v) => { hy = v; }));
  controls.appendChild(createSlider("Half Z", 0.3, 2.5, hz, 0.1, (v) => { hz = v; }));
  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { distance: 9 });

  const stop = runLoop(() => {
    demo.clearContent();
    demo.content.add(makeAxes(1.5));

    const edges = wasm.box_hull_edges(hx, hy, hz);
    demo.content.add(makeSolidBox(0, 0, 0, hx, hy, hz, COLORS.accent, 0.35));
    demo.content.add(makeWireEdges(edges, COLORS.accent));
    updateReadout(readout, [
      { label: "API", value: "b3MakeBoxHull" },
      { label: "Edges", value: String(edges.length / 6) },
      { label: "Half-extents", value: `${hx.toFixed(1)}, ${hy.toFixed(1)}, ${hz.toFixed(1)}` },
    ]);

    demo.render();
  }, readout);

  return () => {
    stop();
    demo.dispose();
  };
}
