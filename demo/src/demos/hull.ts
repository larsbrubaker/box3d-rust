// Hull — create_hull / make_box_hull wireframes.

import { createButtonGroup, createInfoBox, createReadout, createSlider, updateReadout } from "../controls.ts";
import { getWasm } from "../wasm.ts";
import {
  OrbitCamera,
  demoPage,
  drawAxes,
  drawWireEdges,
  fitCanvas,
  runLoop,
} from "./common.ts";

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
        "feeds a ring of points plus poles into quickhull.",
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

  const cam = new OrbitCamera();
  cam.distance = 8;
  cam.scale = 70;
  const detach = cam.attachDrag(canvas);
  const ctx = canvas.getContext("2d")!;

  const stop = runLoop(() => {
    fitCanvas(canvas);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawAxes(ctx, cam, canvas);

    if (mode === "box") {
      const edges = wasm.box_hull_edges(hx, hy, hz);
      drawWireEdges(ctx, cam, canvas, edges, "#2563eb");
      updateReadout(readout, [
        { label: "API", value: "b3MakeBoxHull" },
        { label: "Edges", value: String(edges.length / 6) },
        { label: "Half-extents", value: `${hx.toFixed(1)}, ${hy.toFixed(1)}, ${hz.toFixed(1)}` },
      ]);
    } else {
      const data = wasm.create_hull_demo(sides, Math.max(hx, hz), hy);
      drawWireEdges(ctx, cam, canvas, data, "#15803d", 3);
      updateReadout(readout, [
        { label: "API", value: "b3CreateHull" },
        { label: "Vertices", value: String(data[0]) },
        { label: "Faces", value: String(data[1]) },
        { label: "Half-edges", value: String(data[2]) },
      ]);
    }
  }, readout);

  return () => {
    stop();
    detach();
  };
}
