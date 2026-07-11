// Dynamic Tree — AABB proxies and query visualization (Three.js).

import { createInfoBox, createReadout, createSlider, updateReadout } from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import {
  COLORS,
  DemoScene,
  makeAxes,
  makeSolidBox,
  makeWireBox,
} from "../three-scene.ts";

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Dynamic Tree",
    "A dynamic AABB tree from the ported <code>b3DynamicTree</code>. Proxies are inserted " +
      "with <code>b3DynamicTree_CreateProxy</code>; the moving query box uses " +
      "<code>b3DynamicTree_Query</code>.",
    "Drag to orbit · query box orbits the grid",
    wasm.version(),
  );

  let count = 8;
  wasm.tree_reset(count);

  controls.appendChild(
    createInfoBox(
      "Green boxes are query hits. Gray boxes are non-overlapping proxies. Metrics " +
        "(height, area ratio) come from the Rust tree.",
    ),
  );
  controls.appendChild(
    createSlider("Proxies", 1, 27, count, 1, (v) => {
      count = v;
      wasm.tree_reset(count);
    }),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { distance: 16 });
  const start = performance.now();

  const stop = runLoop(() => {
    demo.clearContent();
    demo.clearDynamic();
    demo.content.add(makeAxes(1.5));

    const t = (performance.now() - start) / 1000;
    const qx = Math.cos(t * 0.6) * 2.5;
    const qy = Math.sin(t * 0.4) * 2.0;
    const qz = Math.sin(t * 0.6) * 2.5;
    const qh = 1.1;

    const aabbs = wasm.tree_proxy_aabbs();
    const query = wasm.tree_query(qx, qy, qz, qh);
    const hitSet = new Set<number>();
    for (let i = 3; i < query.length; i++) hitSet.add(query[i]!);

    const n = aabbs[0]!;
    for (let i = 0; i < n; i++) {
      const o = 1 + i * 6;
      const lx = aabbs[o]!;
      const ly = aabbs[o + 1]!;
      const lz = aabbs[o + 2]!;
      const ux = aabbs[o + 3]!;
      const uy = aabbs[o + 4]!;
      const uz = aabbs[o + 5]!;
      const cx = (lx + ux) / 2;
      const cy = (ly + uy) / 2;
      const cz = (lz + uz) / 2;
      const hx = (ux - lx) / 2;
      const hy = (uy - ly) / 2;
      const hz = (uz - lz) / 2;
      const hit = hitSet.has(i);
      const color = hit ? COLORS.good : COLORS.muted;
      demo.content.add(makeSolidBox(cx, cy, cz, hx, hy, hz, color, hit ? 0.35 : 0.15));
      demo.content.add(makeWireBox(cx, cy, cz, hx, hy, hz, color));
    }

    demo.dynamic.add(makeSolidBox(qx, qy, qz, qh, qh, qh, COLORS.accent, 0.2));
    demo.dynamic.add(makeWireBox(qx, qy, qz, qh, qh, qh, COLORS.accent));

    const metrics = wasm.tree_metrics();
    updateReadout(readout, [
      { label: "Proxies", value: String(metrics[0]) },
      { label: "Tree height", value: String(metrics[1]) },
      { label: "Area ratio", value: metrics[2]!.toFixed(3) },
      { label: "Query hits", value: String(query[0]) },
      { label: "Node visits", value: String(query[1]) },
      { label: "Leaf visits", value: String(query[2]) },
    ]);

    demo.render();
  }, readout);

  return () => {
    stop();
    demo.dispose();
  };
}
