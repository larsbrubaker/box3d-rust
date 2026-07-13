// Replay — .b3rec recording viewer (port of samples/sample_replay.cpp).
//
// The C `ReplayViewer` drives a `b3RecPlayer` one recorded step at a time and
// draws the replayed world through the debug-draw path. This page mirrors the
// core of that viewer for the browser: a transport (play / pause / step / seek),
// a scrubber timeline with a frame readout, and faithful playback rendering of
// every recorded body's transform and shape geometry each frame — colored by the
// engine's own `shape_debug_color` state machine (via `replay_shape_styles`).
//
// Scope vs. the C viewer (honest disclosure, matching the wasm binding's doc):
//   - Shipped & working: load a bundled or user-picked .b3rec, transport +
//     scrubber, and playback rendering (sphere/capsule parametric; hull/mesh/
//     height-field as triangle meshes), plus the divergence readout.
//   - Shipped: the OUTLINE scene tree (bodies grouped by creation ordinal with their
//     shapes) and the SELECTION INSPECTOR (the selected body's live transform /
//     velocity / mass / awake-enabled-bullet state at the current frame, updating as
//     you scrub or play). Click a body in the outline or in the 3D view to select and
//     highlight it (emissive tint). Selection survives backward seeks (stored as an
//     ordinal), matching C.
//   - Not ported (disclosed): the whole-recording query SEARCH INDEX (C replays the
//     entire recording once to index every spatial query — scoped out as too heavy for
//     the browser demo) and the KEYFRAME-POLICY popup (it only tunes a backward-seek
//     keyframe-ring budget our restart-and-replay seek never consumes, so a control
//     would have no observable effect). Compound-shape bodies are not individually
//     tessellated, so they render nothing (but still list + inspect).
//
// Loading a LOCAL .b3rec via the file picker is ordinary in-page behavior; nothing
// is uploaded. The bundled sample was recorded from this port itself (see
// public/recordings/README.md).

import * as THREE from "three";
import { getWasm } from "../wasm.ts";
import { createButton } from "../controls.ts";
import { createCanvasOverlay } from "../controls.ts";
import { demoPage, runLoop } from "./common.ts";
import { applyShapeStyle, DemoScene, makeShapeMaterial, setView } from "../three-scene.ts";

// Single-scene page, not registry-backed (Replay is registered via g_replayIndex
// in the C app). Exported for symmetry with the multi-scene pages.
export const SCENES: readonly string[] = [];

const SAMPLE_URL = "/public/recordings/sample.b3rec";

// Geometry-stream shape kinds — must match replay_demo.rs.
const RKIND_TRI = 0;
const RKIND_SPHERE = 1;
const RKIND_CAPSULE = 2;

const SPEEDS = [
  { label: "0.25x", value: 0.25 },
  { label: "0.5x", value: 0.5 },
  { label: "1x", value: 1 },
  { label: "2x", value: 2 },
  { label: "4x", value: 4 },
];

type ParsedShape = {
  ordinal: number;
  kind: number;
  // Sphere: local center + radius. Capsule: two local centers + radius.
  center: [number, number, number];
  center2: [number, number, number];
  radius: number;
};

export function init(container: HTMLElement) {
  const wasm = getWasm();

  const { canvas, controls, page } = demoPage(
    container,
    "Replay",
    "Recording viewer for <code>.b3rec</code> files (port of <code>sample_replay.cpp</code>): " +
      "transport, scrubber, and faithful playback of a recording captured by the port itself.",
    "Drag to orbit · load a .b3rec to replay",
    wasm.version(),
    { category: "Replay", samplesShell: true },
  );

  const demo = new DemoScene(canvas, {
    target: [0, 2, 0],
    distance: 20,
    fov: 50,
    shadowExtent: 40,
    gridSize: 80,
  });

  const overlay = createCanvasOverlay(page);

  // --- Render resources ---
  const sphereGeo = new THREE.SphereGeometry(1, 20, 14);
  let parsed: ParsedShape[] = [];
  let meshes: THREE.Mesh[] = [];

  // --- Selection (C ReplayViewer m_selBodyOrdinal). Stored as a creation ordinal so
  // it survives the backward-seek world rebuild, exactly like the C viewer. ---
  let selectedOrd: number | null = null;
  const bodyRows = new Map<number, HTMLElement>();
  const HILITE = new THREE.Color(0x2f6bff);

  // --- Transport state ---
  let playing = false;
  let loop = false;
  let speed = 1;
  let accumulator = 0;
  let needsSync = true;
  let status = "loading bundled recording...";

  const _q = new THREE.Quaternion();
  const _p = new THREE.Vector3();
  const _a = new THREE.Vector3();
  const _b = new THREE.Vector3();
  const _dir = new THREE.Vector3();
  const _yUp = new THREE.Vector3(0, 1, 0);

  function disposeScene() {
    for (const m of meshes) {
      demo.content.remove(m);
      if (m.geometry !== sphereGeo) m.geometry.dispose();
      (m.material as THREE.Material).dispose();
    }
    meshes = [];
    parsed = [];
  }

  function parseGeometry(buf: Float32Array): ParsedShape[] {
    const shapes: ParsedShape[] = [];
    let i = 0;
    while (i + 1 < buf.length) {
      const ordinal = buf[i++]!;
      const kind = buf[i++]!;
      if (kind === RKIND_SPHERE) {
        const cx = buf[i++]!, cy = buf[i++]!, cz = buf[i++]!, r = buf[i++]!;
        shapes.push({ ordinal, kind, center: [cx, cy, cz], center2: [0, 0, 0], radius: r });
      } else if (kind === RKIND_CAPSULE) {
        const c1x = buf[i++]!, c1y = buf[i++]!, c1z = buf[i++]!;
        const c2x = buf[i++]!, c2y = buf[i++]!, c2z = buf[i++]!;
        const r = buf[i++]!;
        shapes.push({
          ordinal,
          kind,
          center: [c1x, c1y, c1z],
          center2: [c2x, c2y, c2z],
          radius: r,
        });
      } else {
        // Triangle list: floatCount then that many body-local triangle floats.
        const n = buf[i++]!;
        const tris = buf.slice(i, i + n);
        i += n;
        const g = new THREE.BufferGeometry();
        g.setAttribute("position", new THREE.BufferAttribute(tris, 3));
        g.computeVertexNormals();
        // Stash the built geometry on a synthetic shape entry via a closure below;
        // to keep ParsedShape uniform we build the mesh here and remember it.
        triGeos.push(g);
        shapes.push({ ordinal, kind, center: [0, 0, 0], center2: [0, 0, 0], radius: 0 });
      }
    }
    return shapes;
  }

  // Triangle BufferGeometries built during parse, consumed in the same order by
  // buildScene (only the RKIND_TRI shapes push here).
  let triGeos: THREE.BufferGeometry[] = [];

  function buildScene() {
    disposeScene();
    triGeos = [];
    const geo = wasm.replay_scene_geometry();
    parsed = parseGeometry(geo);
    let triCursor = 0;
    meshes = parsed.map((s) => {
      const mat = makeShapeMaterial();
      let mesh: THREE.Mesh;
      if (s.kind === RKIND_SPHERE) {
        mesh = new THREE.Mesh(sphereGeo, mat);
      } else if (s.kind === RKIND_CAPSULE) {
        _a.set(...s.center);
        _b.set(...s.center2);
        const len = Math.max(1e-4, _a.distanceTo(_b));
        mesh = new THREE.Mesh(new THREE.CapsuleGeometry(s.radius, len, 6, 12), mat);
      } else {
        mesh = new THREE.Mesh(triGeos[triCursor++]!, mat);
      }
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      demo.content.add(mesh);
      return mesh;
    });
    // Fresh materials start with black emissive; rebuild the outline for the new
    // topology and re-apply the current selection highlight.
    buildOutline();
    applyHighlight();
  }

  // Emissive tint on every mesh belonging to the selected body (C's outline
  // highlight). applyShapeStyle never touches emissive, so this persists across
  // frames and only needs re-applying on a selection change or a topology rebuild.
  function applyHighlight() {
    for (let i = 0; i < meshes.length; i++) {
      const mat = meshes[i]!.material as THREE.MeshStandardMaterial;
      if (parsed[i] && parsed[i]!.ordinal === selectedOrd) mat.emissive.copy(HILITE);
      else mat.emissive.setHex(0x000000);
    }
  }

  function escapeHtml(s: string): string {
    return s.replace(/[&<>"]/g, (c) =>
      c === "&" ? "&amp;" : c === "<" ? "&lt;" : c === ">" ? "&gt;" : "&quot;",
    );
  }

  // (Re)populate the Outline panel from the current topology (C DrawOutlineTree):
  // one clickable row per valid body, its shapes listed beneath. A shape row selects
  // its owning body (selection is body-granular; see the module scope note).
  function buildOutline() {
    outlineList.innerHTML = "";
    bodyRows.clear();
    let entries: { ord: number; name: string; type: string; shapes: string[] }[] = [];
    try {
      entries = JSON.parse(wasm.replay_outline());
    } catch {
      entries = [];
    }
    for (const b of entries) {
      const row = document.createElement("div");
      row.style.cursor = "pointer";
      row.style.padding = "1px 4px";
      row.textContent = `Body ${b.ord} · ${b.name || b.type}`;
      row.title = `${b.type}`;
      row.addEventListener("click", () => selectBody(b.ord));
      outlineList.appendChild(row);
      bodyRows.set(b.ord, row);
      for (const s of b.shapes) {
        const srow = document.createElement("div");
        srow.style.cursor = "pointer";
        srow.style.paddingLeft = "18px";
        srow.style.opacity = "0.75";
        srow.style.fontSize = "0.9em";
        srow.textContent = `– ${s}`;
        srow.addEventListener("click", () => selectBody(b.ord));
        outlineList.appendChild(srow);
      }
    }
    updateOutlineSelection();
  }

  function updateOutlineSelection() {
    for (const [ord, row] of bodyRows) {
      row.style.background = ord === selectedOrd ? "rgba(47,107,255,0.35)" : "";
    }
  }

  function selectBody(ord: number | null) {
    selectedOrd = ord;
    applyHighlight();
    updateOutlineSelection();
    updateInspector();
  }

  // Selection inspector for the current frame (C DrawBodyDetail). Called each rendered
  // frame so the readout tracks the body as the recording plays / scrubs.
  function updateInspector() {
    if (selectedOrd == null) {
      inspectorBody.innerHTML = '<span style="opacity:.65">Click a body (outline or view) to inspect.</span>';
      return;
    }
    let d: {
      present: boolean;
      id?: number;
      name?: string;
      type?: string;
      pos?: number[];
      spinDeg?: number;
      vel?: number[];
      omega?: number[];
      speed?: number;
      spinRate?: number;
      mass?: number;
      awake?: boolean;
      enabled?: boolean;
      bullet?: boolean;
      gravityScale?: number;
      shapeCount?: number;
      jointCount?: number;
    };
    try {
      d = JSON.parse(wasm.replay_body_detail(selectedOrd));
    } catch {
      d = { present: false };
    }
    if (!d.present) {
      inspectorBody.innerHTML =
        `Body ordinal ${selectedOrd}<br><span style="opacity:.65">Not present at this frame.</span>`;
      return;
    }
    const f = (n: number) => n.toFixed(3);
    const p = d.pos!, v = d.vel!, w = d.omega!;
    inspectorBody.innerHTML =
      `<b>Body ${d.id}</b> ${escapeHtml(d.name || "(none)")} · ${d.type}<br>` +
      `pos (${f(p[0]!)}, ${f(p[1]!)}, ${f(p[2]!)})<br>` +
      `spin ${d.spinDeg!.toFixed(1)}°<br>` +
      `vel (${f(v[0]!)}, ${f(v[1]!)}, ${f(v[2]!)}) · speed ${f(d.speed!)}<br>` +
      `omega (${f(w[0]!)}, ${f(w[1]!)}, ${f(w[2]!)}) · rate ${f(d.spinRate!)}<br>` +
      `mass ${d.mass!.toPrecision(4)} kg<br>` +
      `awake ${d.awake ? "yes" : "no"} · enabled ${d.enabled ? "yes" : "no"} · bullet ${d.bullet ? "yes" : "no"}<br>` +
      `gravity scale ${d.gravityScale!.toFixed(2)}<br>` +
      `shapes ${d.shapeCount} · joints ${d.jointCount}`;
  }

  function renderFrame() {
    if (!wasm.replay_loaded()) return;
    // Rebuild the mesh set whenever the recorded shape count changes (a body was
    // created or destroyed mid-recording). The style stream is one word per live
    // shape, so a length mismatch is the signal that the cached geometry is stale;
    // rebuilding keeps geometry, transforms, and styles index-aligned.
    let styles = wasm.replay_shape_styles();
    if (styles.length !== meshes.length) {
      buildScene();
      styles = wasm.replay_shape_styles();
    }
    const xf = wasm.replay_body_transforms();
    for (let i = 0; i < meshes.length; i++) {
      const s = parsed[i]!;
      const m = meshes[i]!;
      const o = s.ordinal * 8;
      if (!xf[o]) {
        m.visible = false;
        continue;
      }
      m.visible = true;
      const px = xf[o + 1]!, py = xf[o + 2]!, pz = xf[o + 3]!;
      _q.set(xf[o + 4]!, xf[o + 5]!, xf[o + 6]!, xf[o + 7]!);
      if (s.kind === RKIND_SPHERE) {
        _a.set(...s.center).applyQuaternion(_q).add(_p.set(px, py, pz));
        m.position.copy(_a);
        m.quaternion.copy(_q);
        m.scale.setScalar(s.radius);
      } else if (s.kind === RKIND_CAPSULE) {
        _p.set(px, py, pz);
        _a.set(...s.center).applyQuaternion(_q).add(_p);
        _b.set(...s.center2).applyQuaternion(_q).add(_p);
        m.position.copy(_a).add(_b).multiplyScalar(0.5);
        _dir.subVectors(_b, _a);
        if (_dir.length() > 1e-6) m.quaternion.setFromUnitVectors(_yUp, _dir.normalize());
      } else {
        m.position.set(px, py, pz);
        m.quaternion.copy(_q);
        m.scale.setScalar(1);
      }
      applyShapeStyle(m, styles[i] ?? 0);
    }
    // Re-apply the current selection highlight (applyShapeStyle may have rewritten a
    // recolored material's flags) and refresh the inspector for this frame.
    applyHighlight();
    updateInspector();
  }

  function frameCamera() {
    const b = wasm.replay_bounds();
    if (b.length === 6) {
      const cx = (b[0]! + b[3]!) / 2;
      const cy = (b[1]! + b[4]!) / 2;
      const cz = (b[2]! + b[5]!) / 2;
      const radius = Math.max(b[3]! - b[0]!, b[4]! - b[1]!, b[5]! - b[2]!, 1);
      setView(demo, 35, -20, radius * 1.8, [cx, cy, cz]);
    } else {
      setView(demo, 35, -20, 20, [0, 2, 0]);
    }
  }

  function markDirty() {
    needsSync = true;
    syncScrubber();
  }

  async function loadBytes(bytes: Uint8Array, name: string) {
    const ok = wasm.replay_load(bytes);
    if (ok) {
      wasm.replay_seek(0);
      selectedOrd = null; // a fresh recording clears any prior selection
      buildScene();
      frameCamera();
      playing = false;
      accumulator = 0;
      const total = wasm.replay_frame_count();
      const hz = wasm.replay_time_step() > 0 ? Math.round(1 / wasm.replay_time_step()) : 0;
      status = `loaded ${name} · ${total} frames @ ${hz} Hz`;
    } else {
      status = `failed to load ${name} (bad or unsupported .b3rec)`;
    }
    scrubber.max = String(Math.max(1, wasm.replay_frame_count()));
    updateButtons();
    markDirty();
  }

  // --- Transport controls UI ---
  const fileGroup = document.createElement("div");
  fileGroup.className = "control-group";
  const fileLabel = document.createElement("label");
  fileLabel.textContent = "Recording";
  const fileInput = document.createElement("input");
  fileInput.type = "file";
  fileInput.accept = ".b3rec,application/octet-stream";
  fileInput.addEventListener("change", async () => {
    const f = fileInput.files?.[0];
    if (!f) return;
    const buf = await f.arrayBuffer();
    await loadBytes(new Uint8Array(buf), f.name);
  });
  fileGroup.append(fileLabel, fileInput);

  const transportRow = document.createElement("div");
  transportRow.className = "control-row";

  const btnStart = createButton("|<", () => {
    wasm.replay_seek(0);
    playing = false;
    accumulator = 0;
    updateButtons();
    markDirty();
  });
  const btnBack = createButton("<", () => {
    wasm.replay_seek(wasm.replay_frame() - 1);
    playing = false;
    accumulator = 0;
    updateButtons();
    markDirty();
  });
  const btnPlay = createButton("Play", () => {
    if (wasm.replay_is_at_end()) wasm.replay_restart();
    playing = !playing;
    accumulator = 0;
    updateButtons();
    markDirty();
  });
  const btnFwd = createButton(">", () => {
    wasm.replay_seek(wasm.replay_frame() + 1);
    playing = false;
    accumulator = 0;
    updateButtons();
    markDirty();
  });
  const btnEnd = createButton(">|", () => {
    wasm.replay_seek(wasm.replay_frame_count());
    playing = false;
    accumulator = 0;
    updateButtons();
    markDirty();
  });
  transportRow.append(btnStart, btnBack, btnPlay, btnFwd, btnEnd);

  // Scrubber timeline.
  const scrubGroup = document.createElement("div");
  scrubGroup.className = "control-group";
  const scrubLabel = document.createElement("label");
  scrubLabel.textContent = "Timeline";
  const scrubber = document.createElement("input");
  scrubber.type = "range";
  scrubber.min = "0";
  scrubber.max = "1";
  scrubber.step = "1";
  scrubber.value = "0";
  scrubber.addEventListener("input", () => {
    wasm.replay_seek(parseInt(scrubber.value, 10));
    playing = false;
    accumulator = 0;
    updateButtons();
    needsSync = true;
  });
  scrubGroup.append(scrubLabel, scrubber);

  // Speed + loop.
  const optRow = document.createElement("div");
  optRow.className = "control-row";
  const speedSel = document.createElement("select");
  for (const s of SPEEDS) {
    const opt = document.createElement("option");
    opt.value = String(s.value);
    opt.textContent = s.label;
    if (s.value === speed) opt.selected = true;
    speedSel.appendChild(opt);
  }
  speedSel.addEventListener("change", () => {
    speed = parseFloat(speedSel.value);
  });
  const loopLabel = document.createElement("label");
  loopLabel.style.display = "flex";
  loopLabel.style.alignItems = "center";
  loopLabel.style.gap = "0.4em";
  const loopCb = document.createElement("input");
  loopCb.type = "checkbox";
  loopCb.addEventListener("change", () => {
    loop = loopCb.checked;
  });
  loopLabel.append(loopCb, document.createTextNode("Loop"));
  optRow.append(speedSel, loopLabel);

  // --- Outline scene tree + selection inspector (C left Outline window + Detail pane).
  const outlineGroup = document.createElement("div");
  outlineGroup.className = "control-group";
  const outlineLabel = document.createElement("label");
  outlineLabel.textContent = "Outline";
  const outlineList = document.createElement("div");
  outlineList.style.maxHeight = "180px";
  outlineList.style.overflowY = "auto";
  outlineList.style.fontFamily = "monospace";
  outlineList.style.fontSize = "0.85em";
  outlineList.style.border = "1px solid rgba(128,128,128,0.3)";
  outlineList.style.borderRadius = "4px";
  outlineList.style.padding = "2px";
  outlineGroup.append(outlineLabel, outlineList);

  const inspGroup = document.createElement("div");
  inspGroup.className = "control-group";
  const inspLabel = document.createElement("label");
  inspLabel.textContent = "Inspector";
  const inspectorBody = document.createElement("div");
  inspectorBody.style.fontFamily = "monospace";
  inspectorBody.style.fontSize = "0.85em";
  inspectorBody.style.lineHeight = "1.5";
  inspGroup.append(inspLabel, inspectorBody);

  controls.append(fileGroup, transportRow, scrubGroup, optRow, outlineGroup, inspGroup);

  // 3D-view picking: a genuine (non-drag) left click ray-tests the rendered meshes and
  // selects the hit body's ordinal, or clears the selection on a miss (C MouseDown).
  const raycaster = new THREE.Raycaster();
  const ndc = new THREE.Vector2();
  let downX = 0;
  let downY = 0;
  const onCanvasPointerDown = (e: PointerEvent) => {
    downX = e.clientX;
    downY = e.clientY;
  };
  const onCanvasPointerUp = (e: PointerEvent) => {
    if (e.button !== 0 || e.altKey) return;
    if (Math.hypot(e.clientX - downX, e.clientY - downY) >= 4) return; // was a drag/orbit
    if (!wasm.replay_loaded()) return;
    const rect = canvas.getBoundingClientRect();
    ndc.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    ndc.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(ndc, demo.camera);
    const hits = raycaster.intersectObjects(meshes, false);
    if (hits.length === 0) {
      selectBody(null);
      return;
    }
    const idx = meshes.indexOf(hits[0]!.object as THREE.Mesh);
    selectBody(idx >= 0 ? (parsed[idx]?.ordinal ?? null) : null);
  };
  canvas.addEventListener("pointerdown", onCanvasPointerDown);
  canvas.addEventListener("pointerup", onCanvasPointerUp);

  function updateButtons() {
    btnPlay.textContent = playing ? "Pause" : "Play";
  }

  function syncScrubber() {
    scrubber.value = String(wasm.replay_frame());
  }

  function updateHud() {
    if (!wasm.replay_loaded()) {
      overlay.innerHTML = status;
      return;
    }
    const frame = wasm.replay_frame();
    const total = wasm.replay_frame_count();
    const phase = wasm.replay_is_at_end() ? " (end)" : "";
    let html = `${status}<br>frame ${frame} / ${total}${phase}`;
    const dv = wasm.replay_diverge_frame();
    if (dv >= 0) {
      html += `<br><span style="color:#f87171">diverged at frame ${dv}</span>`;
    }
    overlay.innerHTML = html;
  }

  // --- Main loop ---
  const stop = runLoop(() => {
    if (playing && wasm.replay_loaded()) {
      if (wasm.replay_is_at_end()) {
        if (loop) {
          wasm.replay_restart();
          needsSync = true;
          syncScrubber();
        } else {
          playing = false;
          updateButtons();
        }
      } else {
        accumulator += speed;
        let stepped = false;
        while (accumulator >= 1) {
          accumulator -= 1;
          if (wasm.replay_is_at_end()) {
            if (loop) wasm.replay_restart();
            else break;
          }
          wasm.replay_step();
          stepped = true;
        }
        if (stepped) {
          needsSync = true;
          syncScrubber();
        }
      }
    }
    if (needsSync) {
      renderFrame();
      needsSync = false;
    }
    updateHud();
    demo.render();
  }, controls);

  // Load the bundled sample recording cold.
  fetch(SAMPLE_URL)
    .then((r) => {
      if (!r.ok) throw new Error(`${r.status}`);
      return r.arrayBuffer();
    })
    .then((buf) => loadBytes(new Uint8Array(buf), "sample.b3rec"))
    .catch((e) => {
      status = `no bundled recording (${e}); load a .b3rec to begin`;
      needsSync = true;
    });

  return () => {
    stop();
    canvas.removeEventListener("pointerdown", onCanvasPointerDown);
    canvas.removeEventListener("pointerup", onCanvasPointerUp);
    disposeScene();
    wasm.replay_unload();
    sphereGeo.dispose();
    demo.dispose();
  };
}
