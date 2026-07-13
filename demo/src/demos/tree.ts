// Tree — faithful Samples-App port of sample_tree.cpp `TreeBenchmark`.
// A dynamic AABB tree loaded from an AABB record file, probed by 1024 deterministic
// ray / overlap / closest-point queries. This is a query benchmark, not a physics
// sim, so it wires its own controls instead of the stepping samples shell.
//
// Disclosed deviations (see also the registry entry): record files are fetched
// (no browser fopen); build / query timings use performance.now (no wasm std::time).
// Save / Load + Load Scale ARE wired: Save downloads the serialized tree (Blob), Load
// re-reads a picked file and rebuilds the tree (b3DynamicTree_Save/_Load ported in
// tree_demo.rs as a portable leaf format, since C's raw-struct dump isn't portable).
// Distance culling of drawn boxes uses the live camera.

import * as THREE from "three";
import { createButton, createCheckbox, createInfoBox, createSlider } from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { demoPage, runLoop } from "./common.ts";
import { DemoScene, lineMat, setView, solidMat } from "../three-scene.ts";

/** Scene keys hosted by this page (validated against the registry in registry.test). */
export const SCENES = ["benchmark"] as const;

const FILES = ["bounds01", "bounds02", "bounds03"];
const TEST_COUNT = 1024;

const CC = {
  lightBlue: 0x93c5fd,
  lightGray: 0xd1d5db,
  red: 0xdc2626,
  cyan: 0x22d3ee,
  orange: 0xf97316,
};

// C Render level palette (20 named colors); a representative subset cycles by depth.
const LEVEL_COLORS = [
  0xf0f8ff, 0xfaebd7, 0x00ffff, 0x7fffd4, 0xf0ffff, 0xf5f5dc, 0xffe4c4, 0xffebcd,
  0x0000ff, 0x8a2be2, 0xa52a2a, 0xdeb887, 0x5f9ea0, 0x7fff00, 0xd2691e, 0xff7f50,
  0x6495ed, 0xfff8dc, 0xdc143c, 0x00ffff,
];

function pushBoxEdges(out: number[], lx: number, ly: number, lz: number, ux: number, uy: number, uz: number) {
  const c: [number, number, number][] = [
    [lx, ly, lz], [ux, ly, lz], [ux, uy, lz], [lx, uy, lz],
    [lx, ly, uz], [ux, ly, uz], [ux, uy, uz], [lx, uy, uz],
  ];
  const segs = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]];
  for (const [a, b] of segs) out.push(...c[a]!, ...c[b]!);
}

function fetchBounds(index: number): Promise<string> {
  return fetch(`/public/data/trees/${FILES[index]}.txt`).then((r) => {
    if (!r.ok) throw new Error(`${FILES[index]}.txt HTTP ${r.status}`);
    return r.text();
  });
}

export function init(container: HTMLElement) {
  assertRouteScenes("tree", SCENES);
  const wasm = getWasm();

  const { canvas, controls } = demoPage(
    container,
    "Tree",
    "Official Tree sample <strong>Benchmark</strong> from <code>sample_tree.cpp</code> — a dynamic " +
      "AABB tree loaded from an AABB record file, profiled by 1024 ray / overlap / closest queries.",
    "File · Top Down rebuild · Ray/Overlap/Closest · Profile · Test/Level/km sliders",
    wasm.version(),
    { category: "Tree", samplesShell: true },
  );
  controls.appendChild(
    createInfoBox(
      "<strong>Tree Benchmark</strong> — leaves draw light blue, gray when the current test's " +
        "query or ray hits them. Toggle Ray Cast / Overlap / Closet Point and drag Test to move the " +
        "probe; Profile times all 1024 queries. Save downloads the tree; Load re-reads it (Load Scale " +
        "rescales every box).",
    ),
  );

  const demo = new DemoScene(canvas, { target: [0, 0, 0], distance: 250 });
  demo.camera.far = 4000;
  demo.camera.near = 0.5;
  demo.camera.updateProjectionMatrix();
  setView(demo, 45, 45, 250, [0, 0, 0]);

  const readout = controls.querySelector(".info-readout") as HTMLElement | null;

  const state = {
    fileIndex: 0,
    doRay: false,
    doOverlap: false,
    doClosest: false,
    testIndex: 0,
    level: -1,
    km: 1,
    loadScale: 1,
    height: 0,
    buildMs: 0,
    rayMs: 0,
    overlapMs: 0,
    closestMs: 0,
    loaded: false,
  };

  // --- Controls -------------------------------------------------------------

  const fileRow = document.createElement("div");
  fileRow.className = "control-group";
  const fileLabel = document.createElement("label");
  fileLabel.textContent = "File";
  const fileSelect = document.createElement("select");
  fileSelect.className = "control-select";
  FILES.forEach((f, i) => {
    const opt = document.createElement("option");
    opt.value = String(i);
    opt.textContent = f;
    fileSelect.appendChild(opt);
  });
  fileSelect.addEventListener("change", () => {
    state.fileIndex = Number(fileSelect.value);
    void loadFile(state.fileIndex);
  });
  fileRow.append(fileLabel, fileSelect);
  controls.appendChild(fileRow);

  const btnRow = document.createElement("div");
  btnRow.className = "control-row";
  btnRow.appendChild(
    createButton("Top Down", () => {
      const t = performance.now();
      wasm.tree_rebuild();
      state.buildMs = performance.now() - t;
      refreshStats();
      rebuildGeometry();
    }, false),
  );
  btnRow.appendChild(
    createButton("Profile", () => {
      let t = performance.now();
      wasm.tree_profile_ray();
      state.rayMs = performance.now() - t;
      t = performance.now();
      wasm.tree_profile_overlap();
      state.overlapMs = performance.now() - t;
      t = performance.now();
      wasm.tree_profile_closest();
      state.closestMs = performance.now() - t;
    }, false),
  );
  controls.appendChild(btnRow);

  controls.appendChild(createCheckbox("Ray Cast", false, (v) => (state.doRay = v)));
  controls.appendChild(createCheckbox("Overlap", false, (v) => (state.doOverlap = v)));
  controls.appendChild(createCheckbox("Closet Point", false, (v) => (state.doClosest = v)));

  const testSlider = createSlider("Test", 0, TEST_COUNT - 1, 0, 1, (v) => (state.testIndex = Math.round(v)));
  controls.appendChild(testSlider);
  const levelSlider = createSlider("Level", -1, 1, -1, 1, (v) => {
    state.level = Math.round(v);
    rebuildGeometry();
  });
  controls.appendChild(levelSlider);
  controls.appendChild(
    createSlider("Kilometers", 0.5, 20, 1, 0.1, (v) => {
      state.km = v;
      rebuildGeometry();
    }),
  );

  // --- Save / Load / Load Scale (C Save / Load buttons + Load Scale slider) ----
  // Save serializes the live tree (tree_save) and downloads it as a Blob; Load
  // re-reads a picked file through tree_load(bytes, loadScale) and rebuilds. No
  // browser fopen, so the file name is user-chosen at download / pick time.
  const saveLoadRow = document.createElement("div");
  saveLoadRow.className = "control-row";
  saveLoadRow.appendChild(
    createButton("Save", () => {
      const bytes = wasm.tree_save();
      // Copy into a fresh ArrayBuffer so the Blob never aliases wasm memory.
      const blob = new Blob([bytes.slice()], { type: "application/octet-stream" });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = `${FILES[state.fileIndex]}.b3tree`;
      a.click();
      URL.revokeObjectURL(url);
    }, false),
  );
  const loadInput = document.createElement("input");
  loadInput.type = "file";
  loadInput.accept = ".b3tree,application/octet-stream";
  loadInput.style.display = "none";
  loadInput.addEventListener("change", async () => {
    const f = loadInput.files?.[0];
    if (!f) return;
    const buf = await f.arrayBuffer();
    const ok = wasm.tree_load(new Uint8Array(buf), state.loadScale);
    loadInput.value = ""; // allow re-picking the same file
    if (!ok) {
      if (readout) readout.textContent = `Load failed: ${f.name} is not a valid tree file`;
      return;
    }
    state.buildMs = 0;
    state.rayMs = 0;
    state.overlapMs = 0;
    state.closestMs = 0;
    refreshStats();
    rebuildGeometry();
    state.loaded = true;
  });
  saveLoadRow.appendChild(
    createButton("Load", () => loadInput.click(), false),
  );
  controls.appendChild(saveLoadRow);
  controls.appendChild(loadInput);
  controls.appendChild(
    createSlider("Load Scale", 0.01, 1, 1, 0.01, (v) => {
      state.loadScale = v;
    }),
  );

  function rebuildLevelSlider() {
    const input = levelSlider.querySelector("input") as HTMLInputElement | null;
    if (input) {
      input.max = String(state.height);
      if (state.level > state.height) {
        state.level = state.height;
        input.value = String(state.level);
      }
    }
  }

  // --- Visualization --------------------------------------------------------
  // Leaf / level geometry is static: it only changes on File / Top Down / Level /
  // Kilometers. Build it once per such event (`rebuildGeometry`); per frame only
  // the per-test hit COLORING (leaf vertex colors) and the small ray / overlap /
  // closest probe overlays update — no per-frame clearGroup / dispose / rebuild.

  const staticGroup = new THREE.Group();
  const probeGroup = new THREE.Group();
  demo.dynamic.add(staticGroup);
  demo.dynamic.add(probeGroup);
  type Disposable = THREE.LineSegments | THREE.Mesh | THREE.Points;
  const staticDisposables: Disposable[] = [];
  const probeDisposables: Disposable[] = [];

  // Leaf geometry is one LineSegments with a vertex-color attribute, built once
  // per rebuild and recolored (not rebuilt) when the current test's hits change.
  let leafColorAttr: THREE.Float32BufferAttribute | null = null;
  let leafHitMeta: { boxOffset: number; vtxStart: number }[] = [];
  const blueColor = new THREE.Color(CC.lightBlue);
  const grayColor = new THREE.Color(CC.lightGray);
  const VTX_PER_BOX = 24; // 12 edges * 2 endpoints
  let lastQueryToken = "";

  // Hoisted scratch so culling never allocates a Vector3 per box.
  const camPos = new THREE.Vector3();
  const scratch = new THREE.Vector3();

  function disposeAll(list: Disposable[]) {
    for (const o of list) {
      o.parent?.remove(o);
      (o.geometry as THREE.BufferGeometry).dispose();
      (o.material as THREE.Material).dispose();
    }
    list.length = 0;
  }
  function addSegs(g: THREE.Group, disp: Disposable[], positions: number[], color: number, opacity = 1) {
    if (!positions.length) return;
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    const o = new THREE.LineSegments(geo, lineMat(color, opacity));
    g.add(o);
    disp.push(o);
  }

  // Repaint each drawn leaf blue (idle) or gray (hit by the current test). Only
  // the color buffer changes — the positions are untouched.
  function recolorLeaves(boxes: Float32Array) {
    if (!leafColorAttr) return;
    const arr = leafColorAttr.array as Float32Array;
    for (const { boxOffset, vtxStart } of leafHitMeta) {
      const c = (boxes[boxOffset + 6] ?? 0) > 0.5 ? grayColor : blueColor;
      for (let v = 0; v < VTX_PER_BOX; v++) {
        const o = (vtxStart + v) * 3;
        arr[o] = c.r;
        arr[o + 1] = c.g;
        arr[o + 2] = c.b;
      }
    }
    leafColorAttr.needsUpdate = true;
  }

  // Rebuild the static leaf / level geometry + axes. Called only on File / Top
  // Down / Level / Kilometers changes, never per frame.
  function rebuildGeometry() {
    disposeAll(staticDisposables);
    leafColorAttr = null;
    leafHitMeta = [];
    demo.camera.getWorldPosition(camPos);
    const distSq = state.km * state.km * 1000 * 1000;

    if (state.level >= 0) {
      // Per-level internal-node view (flat color; no per-test hit coloring).
      const boxes = wasm.tree_level_boxes(state.level);
      const color = LEVEL_COLORS[state.level % LEVEL_COLORS.length]!;
      const edges: number[] = [];
      for (let i = 0; i + 5 < boxes.length; i += 6) {
        const lx = boxes[i]!, ly = boxes[i + 1]!, lz = boxes[i + 2]!;
        const ux = boxes[i + 3]!, uy = boxes[i + 4]!, uz = boxes[i + 5]!;
        // C culls level draw only when level >= 10.
        if (state.level >= 10) {
          scratch.set(0.5 * (lx + ux), 0.5 * (ly + uy), 0.5 * (lz + uz));
          if (camPos.distanceToSquared(scratch) >= distSq) continue;
        }
        pushBoxEdges(edges, lx, ly, lz, ux, uy, uz);
      }
      addSegs(staticGroup, staticDisposables, edges, color);
    } else {
      // Leaf view: one vertex-colored geometry, distance culled at build time.
      const boxes = wasm.tree_leaf_boxes();
      const positions: number[] = [];
      const colors: number[] = [];
      for (let i = 0; i + 6 < boxes.length; i += 7) {
        const lx = boxes[i]!, ly = boxes[i + 1]!, lz = boxes[i + 2]!;
        const ux = boxes[i + 3]!, uy = boxes[i + 4]!, uz = boxes[i + 5]!;
        scratch.set(0.5 * (lx + ux), 0.5 * (ly + uy), 0.5 * (lz + uz));
        if (camPos.distanceToSquared(scratch) > distSq) continue;
        const vtxStart = positions.length / 3;
        pushBoxEdges(positions, lx, ly, lz, ux, uy, uz);
        for (let v = 0; v < VTX_PER_BOX; v++) colors.push(0, 0, 0);
        leafHitMeta.push({ boxOffset: i, vtxStart });
      }
      if (positions.length) {
        const geo = new THREE.BufferGeometry();
        geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
        const colorAttr = new THREE.Float32BufferAttribute(colors, 3);
        geo.setAttribute("color", colorAttr);
        const o = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ vertexColors: true }));
        staticGroup.add(o);
        staticDisposables.push(o);
        leafColorAttr = colorAttr;
        recolorLeaves(boxes);
      }
    }

    // World axes at the origin (C DrawAxes identity, length 2).
    addSegs(staticGroup, staticDisposables, [0, 0, 0, 2, 0, 0], CC.red);
    addSegs(staticGroup, staticDisposables, [0, 0, 0, 0, 2, 0], 0x22c55e);
    addSegs(staticGroup, staticDisposables, [0, 0, 0, 0, 0, 2], 0x2563eb);

    // Force the hit recolor + probe overlay to refresh on the next frame.
    lastQueryToken = "";
  }

  // The small per-test probe overlays (ray / overlap box / closest sphere+point),
  // rebuilt only when the probe inputs (Test / Ray / Overlap / Closest) change.
  function rebuildProbes(closest: Float32Array, haveClosest: boolean) {
    disposeAll(probeDisposables);
    if (state.doRay) {
      const r = wasm.tree_test_ray(state.testIndex);
      addSegs(probeGroup, probeDisposables, [r[0]!, r[1]!, r[2]!, r[3]!, r[4]!, r[5]!], CC.red);
    }
    if (state.doOverlap) {
      const o = wasm.tree_test_overlap(state.testIndex);
      const edges: number[] = [];
      pushBoxEdges(edges, o[0]!, o[1]!, o[2]!, o[3]!, o[4]!, o[5]!);
      addSegs(probeGroup, probeDisposables, edges, CC.red);
    }
    if (state.doClosest) {
      const sph = wasm.tree_test_sphere(state.testIndex);
      const mesh = new THREE.Mesh(new THREE.SphereGeometry(sph[3]!, 16, 12), solidMat(CC.cyan, 0.5));
      mesh.position.set(sph[0]!, sph[1]!, sph[2]!);
      probeGroup.add(mesh);
      probeDisposables.push(mesh);
      if (haveClosest) {
        const g = new THREE.BufferGeometry();
        g.setAttribute("position", new THREE.Float32BufferAttribute([closest[1]!, closest[2]!, closest[3]!], 3));
        const pts = new THREE.Points(g, new THREE.PointsMaterial({ color: CC.orange, size: 12, sizeAttenuation: false }));
        probeGroup.add(pts);
        probeDisposables.push(pts);
      }
    }
  }

  function refreshStats() {
    const s = wasm.tree_stats();
    const dims = wasm.tree_dims();
    state.height = Math.round(dims[1] ?? 0);
    rebuildLevelSlider();
    updateReadout(s);
  }

  function updateReadout(stats: Float32Array) {
    if (!readout) return;
    const leaves = Math.round(stats[0] ?? 0);
    const height = Math.round(stats[1] ?? 0);
    const area = stats[2] ?? 0;
    const s = 1000 / TEST_COUNT;
    readout.innerHTML =
      `leaves = ${leaves}, height = ${height}, area = ${area.toPrecision(4)}<br>` +
      `build time = ${state.buildMs.toFixed(3)} ms<br>` +
      `total: ray = ${state.rayMs.toFixed(3)} ms, overlap = ${state.overlapMs.toFixed(3)} ms, ` +
      `closest = ${state.closestMs.toFixed(3)} ms<br>` +
      `ave: ray = ${(s * state.rayMs).toFixed(3)} us, overlap = ${(s * state.overlapMs).toFixed(3)} us, ` +
      `closest = ${(s * state.closestMs).toFixed(3)} us`;
  }

  async function loadFile(index: number) {
    state.loaded = false;
    const text = await fetchBounds(index);
    const t = performance.now();
    wasm.tree_reset(index, text);
    state.buildMs = performance.now() - t;
    state.rayMs = 0;
    state.overlapMs = 0;
    state.closestMs = 0;
    refreshStats();
    rebuildGeometry();
    state.loaded = true;
  }

  void loadFile(0).catch((err) => {
    if (readout) readout.textContent = `Failed to load ${FILES[0]}.txt: ${err}`;
    console.error("tree bounds load failed", err);
  });

  const stop = runLoop(() => {
    // C Step: advance the timestamp and run the selected single-test queries. The
    // static leaf / level geometry is already built; per frame we only refresh the
    // per-test hit coloring and probe overlays, and only when a probe input changes.
    const closest = wasm.tree_step_query(state.doRay, state.doOverlap, state.doClosest, state.testIndex);
    const haveClosest = closest[0]! > 0.5;

    const token = `${state.testIndex}|${state.doRay}|${state.doOverlap}|${state.doClosest}`;
    if (token !== lastQueryToken) {
      lastQueryToken = token;
      if (state.level < 0) recolorLeaves(wasm.tree_leaf_boxes());
      rebuildProbes(closest, haveClosest);
    }

    demo.render();
  }, readout ?? undefined, { ready: () => state.loaded });

  return () => {
    stop();
    disposeAll(staticDisposables);
    disposeAll(probeDisposables);
    staticGroup.parent?.remove(staticGroup);
    probeGroup.parent?.remove(probeGroup);
    demo.dispose();
  };
}
