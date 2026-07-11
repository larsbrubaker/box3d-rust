// Hull — create_hull / make_box_hull wireframes (Three.js).

import { createButtonGroup, createInfoBox, createReadout, createSlider, updateReadout } from "../controls.ts";
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
    "Hull",
    "Convex hulls from the ported <code>b3MakeBoxHull</code> and <code>b3CreateHull</code> " +
      "(quickhull). Wireframe edges come straight from the Rust hull half-edge mesh.",
    "Drag to orbit",
    wasm.version(),
  );

  let mode = "box";
  let hx = 1.2;
  let hy = 0.8;
  let hz = 1.0;
  let sides = 8;

  controls.appendChild(
    createInfoBox(
      "Box hulls use the embedded <code>b3BoxHull</code> template. The pyramid/prism mode " +
        "feeds a ring of points plus poles into quickhull. Solid fill + wireframe edges.",
    ),
  );
  controls.appendChild(
    createButtonGroup(
      [
        { label: "Box", value: "box" },
        { label: "CreateHull", value: "create" },
      ],
      "box",
      (v) => {
        mode = v;
      },
    ),
  );
  controls.appendChild(createSlider("Half X", 0.3, 2.5, hx, 0.1, (v) => { hx = v; }));
  controls.appendChild(createSlider("Half Y", 0.3, 2.5, hy, 0.1, (v) => { hy = v; }));
  controls.appendChild(createSlider("Half Z", 0.3, 2.5, hz, 0.1, (v) => { hz = v; }));
  controls.appendChild(createSlider("Sides", 3, 16, sides, 1, (v) => { sides = v; }));
  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { distance: 9 });

  const stop = runLoop(() => {
    demo.clearContent();
    demo.content.add(makeAxes(1.5));

    if (mode === "box") {
      const edges = wasm.box_hull_edges(hx, hy, hz);
      demo.content.add(makeSolidBox(0, 0, 0, hx, hy, hz, COLORS.accent, 0.35));
      demo.content.add(makeWireEdges(edges, COLORS.accent));
      updateReadout(readout, [
        { label: "API", value: "b3MakeBoxHull" },
        { label: "Edges", value: String(edges.length / 6) },
        { label: "Half-extents", value: `${hx.toFixed(1)}, ${hy.toFixed(1)}, ${hz.toFixed(1)}` },
      ]);
    } else {
      const data = wasm.create_hull_demo(sides, Math.max(hx, hz), hy);
      demo.content.add(makeWireEdges(data, COLORS.good, 3));
      updateReadout(readout, [
        { label: "API", value: "b3CreateHull" },
        { label: "Vertices", value: String(data[0]) },
        { label: "Faces", value: String(data[1]) },
        { label: "Half-edges", value: String(data[2]) },
      ]);
    }

    demo.render();
  }, readout);

  return () => {
    stop();
    demo.dispose();
  };
}
