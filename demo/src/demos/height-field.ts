// Height Field — wave HF mesh surface with ray cast (Three.js).

import { createInfoBox, createReadout, updateReadout } from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import {
  COLORS,
  DemoScene,
  makeArrow,
  makeAxes,
  makeDot,
  makeSegment,
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
    "A sinusoidal height field from <code>b3CreateWave</code>. Ray casts use the ported " +
      "<code>b3RayCastHeightField</code>; triangle edges come from " +
      "<code>b3GetHeightFieldTriangle</code>.",
    "Drag to orbit · ray sweeps automatically",
    wasm.version(),
  );

  const triCount = wasm.hf_build_wave();
  const wire = wasm.hf_wireframe();
  const positions = trianglesFromWireframe(wire);

  controls.appendChild(
    createInfoBox(
      `Wave height field with ${triCount} triangles rendered as a Three.js mesh. The sweeping ` +
        "ray reports hit fraction, point, normal, and triangle index — all from Rust wasm.",
    ),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, {
    target: [4, 0, 4],
    distance: 16,
  });
  // Preserve this sample's current flatter framing (yaw 35°, pitch 20°) now that
  // the DemoScene default is the C camera (pitch -25°). The C per-sample camera
  // for Height Field lands with this sample's batch-3 rebuild.
  setView(demo, 35, 20, 16, [4, 0, 4]);
  demo.content.add(makeAxes(2));
  demo.content.add(makeTriangleMesh(positions, COLORS.shape, 0.7));
  demo.content.add(makeWireEdges(wire, COLORS.muted));

  const start = performance.now();

  const stop = runLoop(() => {
    demo.clearDynamic();

    const t = (performance.now() - start) / 1000;
    const ox = 4 + Math.cos(t * 0.7) * 5;
    const oz = 4 + Math.sin(t * 0.7) * 5;
    const oy = 6;
    const tx = 0;
    const ty = -12;
    const tz = 0;
    const hit = wasm.hf_ray_cast(ox, oy, oz, tx, ty, tz);
    const frac = hit[0] === 1.0 ? hit[1]! : 1.0;
    demo.dynamic.add(
      makeSegment(
        [ox, oy, oz],
        [ox + tx * frac, oy + ty * frac, oz + tz * frac],
        COLORS.accent,
      ),
    );
    demo.dynamic.add(makeDot([ox, oy, oz], COLORS.accent, 0.1));
    if (hit[0] === 1.0) {
      demo.dynamic.add(makeDot([hit[2]!, hit[3]!, hit[4]!], COLORS.hit, 0.1));
      demo.dynamic.add(
        makeArrow(
          [hit[2]!, hit[3]!, hit[4]!],
          [hit[5]!, hit[6]!, hit[7]!],
          1.0,
          COLORS.hit,
        ),
      );
    }

    updateReadout(readout, [
      { label: "Triangles", value: String(triCount) },
      { label: "Hit", value: hit[0] === 1.0 ? "yes" : "no" },
      { label: "Fraction", value: hit[1]!.toFixed(4) },
      { label: "Triangle", value: hit[0] === 1.0 ? String(hit[8]) : "—" },
    ]);

    demo.render();
  }, readout);

  return () => {
    stop();
    demo.dispose();
  };
}
