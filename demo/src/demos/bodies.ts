// Bodies — the eleven Bodies-category samples from sample_bodies.cpp:
// Body Type, Spinning Book, Gyroscopic Torque, Gyroscopic Precession, Weeble,
// Disable, Cast, Kinematic, Lock Mixing, Fixed Rotation, Class Ring.

import * as THREE from "three";
import { createButton, createInfoBox, createReadout, updateReadout } from "../controls.ts";
import {
  attachInteraction,
  DebugDrawOverlay,
  makeInteractAdapter,
  type ParamDef,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { demoPage, makeStyleGate, runLoop } from "./common.ts";
import {
  DemoScene,
  makeCapsule,
  makeSphere,
  makeWireEdges,
  setView,
  solidMat,
} from "../three-scene.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

// Fixed hull colors for the Gyroscopic Precession tops (they render through a
// dedicated hull channel, not the styled pose stream): steel solid + slate wire.
const TOP_HULL_COLOR = 0x64748b;
const TOP_WIRE_COLOR = 0x1e293b;

type Scene =
  | "body-type"
  | "spinning-book"
  | "gyroscopic-torque"
  | "weeble"
  | "disable"
  | "cast"
  | "kinematic"
  | "lock-mixing"
  | "fixed-rotation"
  | "gyroscopic-precession"
  | "class-ring";

// Index in this array is the numeric scene id consumed by `bodies_reset`. Precession
// and Class Ring are appended last so the existing scene ids stay stable (matches
// `scenes::build`).
export const SCENES: Scene[] = [
  "body-type",
  "spinning-book",
  "gyroscopic-torque",
  "weeble",
  "disable",
  "cast",
  "kinematic",
  "lock-mixing",
  "fixed-rotation",
  "gyroscopic-precession",
  "class-ring",
];

// Only Weeble / Kinematic / Cast emit the always-on overlay channel; the other
// six return `[0, 0]`, so skip the `bodies_overlay()` wasm call for them.
const OVERLAY_SCENES = new Set<Scene>([
  "weeble",
  "kinematic",
  "cast",
  "gyroscopic-precession",
]);

/** Reinterpret a packed-color f32 (from the Rust cast-shapes buffer) as 0xRRGGBB. */
const _cbuf = new Float32Array(1);
const _cview = new Uint32Array(_cbuf.buffer);
function colorFromBits(bits: number): number {
  _cbuf[0] = bits;
  return _cview[0]! & 0xffffff;
}

const CAMERAS: Record<Scene, [number, number, number, [number, number, number]]> = {
  "body-type": [0, 30, 30, [0, 1.5, 0]],
  "spinning-book": [0, 30, 10, [0, 1, 0]],
  "gyroscopic-torque": [0, 20, 4, [0, 2, 0]],
  weeble: [45, 25, 25, [0, 0, 0]],
  disable: [45, 25, 10, [0, 0, 0]],
  cast: [120, 30, 20, [0, 1.5, 0]],
  kinematic: [0, 30, 10, [0, 1.5, 0]],
  "lock-mixing": [45, 30, 40, [0, 0, 0]],
  "fixed-rotation": [0, 15, 10, [0, 0, 0]],
  // C GyroscopicPrecession::SetView( 40, 30, 75, {0, 2, 0} ).
  "gyroscopic-precession": [40, 30, 75, [0, 2, 0]],
  // C ClassRing::SetView( 40, 30, 15, {0, 2, 0} ).
  "class-ring": [40, 30, 15, [0, 2, 0]],
};

function typeId(v: string): number {
  return v === "static" ? 0 : v === "kinematic" ? 1 : 2;
}

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  assertRouteScenes("bodies", SCENES);
  const { canvas, controls } = demoPage(
    container,
    "Bodies",
    "The eleven Bodies-category samples from <code>sample_bodies.cpp</code> — body types, " +
      "gyroscopic effects, spinning-top precession, explosions, kinematic targets, casts, motion locks.",
    "Ctrl+click grab · Shift+click spawn · Shift+drag moves the Cast target · P/O/R",
    wasm.version(),
    { category: "Bodies", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "<strong>Body Type</strong> — switch platform/attachments between static / kinematic / dynamic.<br>" +
        "<strong>Spinning Book / Gyroscopic Torque</strong> — the Dzhanibekov (tennis-racket) effect.<br>" +
        "<strong>Gyroscopic Precession</strong> — a field of tilted spinning tops; the measured top's " +
        "precession rate is compared to the classical heavy-top solution (Goldstein 5.7).<br>" +
        "<strong>Weeble</strong> — self-righting capsule; Teleport / Explode.<br>" +
        "<strong>Disable</strong> — enable/disable a welded link or the ball.<br>" +
        "<strong>Cast</strong> — ray / sphere / capsule / mover queries against a Shift-drag target " +
        "(cast proxy shapes drawn as translucent solids; the blue target as a wireframe).<br>" +
        "<strong>Kinematic</strong> — a driven Lissajous target. <strong>Lock Mixing / Fixed Rotation</strong> — motion locks.<br>" +
        "<strong>Class Ring</strong> — a ring of 24 capsules with a heavy gem, spun at 100 rad/s; " +
        "the gem flips from bottom to top. Runs at 960 Hz / 8 sub-steps (16 steps per rendered " +
        "frame, as in C), so the Hertz and Sub-steps sliders do not apply to this scene.",
    ),
  );

  let scene: Scene =
    initialScene && SCENES.includes(initialScene as Scene) ? (initialScene as Scene) : "body-type";
  let ctrl!: SimControllerWithTick;

  const demo = new DemoScene(canvas, { target: [0, 1.5, 0], distance: 20, shadowExtent: 40 });
  demo.camera.far = 400;
  demo.camera.updateProjectionMatrix();
  const pool = createMeshPool();
  const styleGate = makeStyleGate<Uint32Array>();

  // --- Always-on per-scene overlay (segments + points): reuse the shared fat-line
  // parser/overlay so the bodies_overlay() channel gets 1.5px lines + one reused
  // material, identical to the debug-draw overlay (no per-frame material alloc). ---
  const bodiesOverlay = new DebugDrawOverlay(demo);

  // --- Cast solid proxy shapes (sphere / capsule): created once and updated in
  // place. The three proxies have stable geometry (fixed radius/endpoints); only
  // the sphere proxy translates along the drag, and colors change per frame, so
  // rebuild a slot's geometry only when its signature changes. ---
  const castGroup = new THREE.Group();
  demo.dynamic.add(castGroup);
  type CastSlot = { mesh: THREE.Mesh; sig: string };
  const castSlots: (CastSlot | undefined)[] = [];

  function disposeSlot(slot: CastSlot) {
    castGroup.remove(slot.mesh);
    slot.mesh.geometry.dispose();
    (slot.mesh.material as THREE.Material).dispose();
  }
  function clearCast() {
    for (const slot of castSlots) if (slot) disposeSlot(slot);
    castSlots.length = 0;
  }
  function updateCastShapes(buf: Float32Array) {
    const count = buf[0]! | 0;
    let o = 1;
    for (let i = 0; i < count; i++) {
      const kind = buf[o]! | 0;
      const c1: [number, number, number] = [buf[o + 1]!, buf[o + 2]!, buf[o + 3]!];
      const c2: [number, number, number] = [buf[o + 4]!, buf[o + 5]!, buf[o + 6]!];
      const radius = buf[o + 7]!;
      const color = colorFromBits(buf[o + 8]!);
      o += 9;
      // Signature keys the baked geometry: spheres depend only on radius (they
      // translate per frame via position), capsules on radius + both endpoints.
      const sig =
        kind === 0
          ? `s:${radius}`
          : `c:${radius}:${c1[0]},${c1[1]},${c1[2]}:${c2[0]},${c2[1]},${c2[2]}`;
      let slot = castSlots[i];
      if (!slot || slot.sig !== sig) {
        if (slot) disposeSlot(slot);
        const mesh =
          kind === 0
            ? makeSphere(c1[0], c1[1], c1[2], radius, color, 0.6)
            : makeCapsule(c1, c2, radius, color, 0.6);
        slot = { mesh, sig };
        castSlots[i] = slot;
        castGroup.add(mesh);
      }
      if (kind === 0) slot.mesh.position.set(c1[0], c1[1], c1[2]); // sphere moves
      (slot.mesh.material as THREE.MeshStandardMaterial).color.setHex(color);
    }
    // Drop any proxies beyond the current count (kept minimal: count is fixed at 3).
    for (let i = count; i < castSlots.length; i++) {
      const slot = castSlots[i];
      if (slot) disposeSlot(slot);
    }
    castSlots.length = count;
  }

  // --- Gyroscopic Precession tops: all tops share one hull geometry (built once
  // from bodies_precession_hull) and render as N meshes + wire outlines driven by
  // the dedicated bodies_precession_poses channel. The measured top's axis line
  // comes through the shared overlay; the heavy-top HUD through bodies_precession_hud. ---
  const topGroup = new THREE.Group();
  demo.dynamic.add(topGroup);
  let topSolidGeo: THREE.BufferGeometry | null = null;
  const topMeshes: THREE.Mesh[] = [];
  const topWires: THREE.LineSegments[] = [];

  function clearTops() {
    for (const m of topMeshes) {
      topGroup.remove(m);
      (m.material as THREE.Material).dispose();
    }
    for (const w of topWires) {
      topGroup.remove(w);
      w.geometry.dispose();
      (w.material as THREE.Material).dispose();
    }
    topMeshes.length = 0;
    topWires.length = 0;
    if (topSolidGeo) {
      topSolidGeo.dispose();
      topSolidGeo = null;
    }
  }

  function buildTops() {
    clearTops();
    const geo = wasm.bodies_precession_hull();
    if (geo.length < 2) return;
    let p = 0;
    const triCount = geo[p++]! | 0;
    const tris = geo.slice(p, p + triCount);
    p += triCount;
    const edgeCount = geo[p++]! | 0;
    const edges = geo.slice(p, p + edgeCount);

    topSolidGeo = new THREE.BufferGeometry();
    topSolidGeo.setAttribute("position", new THREE.BufferAttribute(new Float32Array(tris), 3));
    topSolidGeo.computeVertexNormals();

    const count = wasm.bodies_precession_poses().length / 7;
    for (let i = 0; i < count; i++) {
      const mesh = new THREE.Mesh(topSolidGeo, solidMat(TOP_HULL_COLOR, 1));
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      topGroup.add(mesh);
      topMeshes.push(mesh);

      const wire = makeWireEdges(edges, TOP_WIRE_COLOR);
      topGroup.add(wire);
      topWires.push(wire);
    }
  }

  function updateTops() {
    const hp = wasm.bodies_precession_poses();
    for (let i = 0; i < topMeshes.length; i++) {
      const o = i * 7;
      const mesh = topMeshes[i]!;
      const wire = topWires[i]!;
      mesh.position.set(hp[o]!, hp[o + 1]!, hp[o + 2]!);
      mesh.quaternion.set(hp[o + 3]!, hp[o + 4]!, hp[o + 5]!, hp[o + 6]!);
      wire.position.copy(mesh.position);
      wire.quaternion.copy(mesh.quaternion);
    }
  }

  // --- Weeble action buttons (Teleport / Explode), toggled by scene ---
  const weebleRow = document.createElement("div");
  weebleRow.className = "control-row";
  weebleRow.style.display = "none";
  weebleRow.appendChild(createButton("Teleport", () => wasm.bodies_teleport(), false));
  weebleRow.appendChild(createButton("Explode", () => wasm.bodies_explode(), false));

  const readout = createReadout();

  function applySceneControls() {
    const p = ctrl.params;
    if (scene === "body-type") {
      wasm.bodies_set_type(typeId(p.bodyType as string));
      wasm.bodies_set_enabled(p.enable as boolean);
    } else if (scene === "disable") {
      wasm.bodies_enable_link(p.enableLink as boolean);
      wasm.bodies_enable_ball(p.enableBall as boolean);
    } else if (scene === "weeble") {
      wasm.bodies_set_magnitude(p.magnitude as number);
    }
  }

  function reset() {
    wasm.bodies_reset(SCENES.indexOf(scene));
    applySceneControls();
    const [yaw, pitch, dist, target] = CAMERAS[scene];
    setView(demo, yaw, pitch, dist, target);
    weebleRow.style.display = scene === "weeble" ? "" : "none";
    if (scene !== "gyroscopic-torque" && scene !== "gyroscopic-precession") readout.innerHTML = "";
    if (scene !== "cast") clearCast();
    if (scene === "gyroscopic-precession") buildTops();
    else clearTops();
    if (!OVERLAY_SCENES.has(scene)) bodiesOverlay.clear();
  }

  const onBodyType = { key: "sample", equals: "body-type" };
  const onDisable = { key: "sample", equals: "disable" };
  const onWeeble = { key: "sample", equals: "weeble" };
  const params: ParamDef[] = [
    {
      type: "select",
      key: "sample",
      label: "Sample",
      options: SCENES.map((s) => ({
        label: s.replace(/(^|-)([a-z])/g, (_m, p, c: string) => (p ? " " : "") + c.toUpperCase()),
        value: s,
      })),
      default: scene,
      restart: true,
    },
    // Body Type (C BodyType::DrawControls radios + Enable checkbox).
    { type: "select", key: "bodyType", label: "Body Type", options: [{ label: "Static", value: "static" }, { label: "Kinematic", value: "kinematic" }, { label: "Dynamic", value: "dynamic" }], default: "dynamic", restart: false, group: "Body Type", visibleWhen: onBodyType },
    { type: "checkbox", key: "enable", label: "Enable", default: true, restart: false, group: "Body Type", visibleWhen: onBodyType },
    // Disable (C DisableBody::DrawControls).
    { type: "checkbox", key: "enableLink", label: "Enable Link", default: true, restart: false, group: "Disable", visibleWhen: onDisable },
    { type: "checkbox", key: "enableBall", label: "Enable Ball", default: true, restart: false, group: "Disable", visibleWhen: onDisable },
    // Weeble (C Weeble::DrawControls magnitude slider, -100000..100000).
    { type: "slider", key: "magnitude", label: "Magnitude", min: -100000, max: 100000, step: 1000, default: 20000, restart: false, group: "Weeble", visibleWhen: onWeeble },
  ];

  ctrl = attachInteraction({
    wasm: makeInteractAdapter(wasm, "bodies"),
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: "Bodies",
    sampleCategory: "Bodies",
    params,
    onParamsChange: (values, key) => {
      if (key === "sample") {
        scene = values.sample as Scene;
        return; // restart re-applies the active scene's controls via reset()
      }
      if (key === "bodyType") wasm.bodies_set_type(typeId(values.bodyType as string));
      else if (key === "enable") wasm.bodies_set_enabled(values.enable as boolean);
      else if (key === "enableLink") wasm.bodies_enable_link(values.enableLink as boolean);
      else if (key === "enableBall") wasm.bodies_enable_ball(values.enableBall as boolean);
      else if (key === "magnitude") wasm.bodies_set_magnitude(values.magnitude as number);
    },
  }) as SimControllerWithTick;

  controls.appendChild(weebleRow);
  controls.appendChild(readout);

  // --- Cast target Shift-drag (C BodyCast::MouseDown / MouseMove / MouseUp) ---
  const _ndc = new THREE.Vector2();
  const _ray = new THREE.Raycaster();
  function pickRay(clientX: number, clientY: number): { o: THREE.Vector3; d: THREE.Vector3 } {
    const rect = canvas.getBoundingClientRect();
    _ndc.set(((clientX - rect.left) / rect.width) * 2 - 1, -((clientY - rect.top) / rect.height) * 2 + 1);
    _ray.setFromCamera(_ndc, demo.camera);
    return { o: _ray.ray.origin.clone(), d: _ray.ray.direction.clone() };
  }
  let castTracking = false;
  const onCastDown = (e: PointerEvent) => {
    if (scene !== "cast" || e.button !== 0 || !e.shiftKey) return;
    const { o, d } = pickRay(e.clientX, e.clientY);
    wasm.bodies_cast_track_down(o.x, o.y, o.z, d.x, d.y, d.z);
    castTracking = true;
    e.preventDefault();
    e.stopPropagation();
  };
  const onCastMove = (e: PointerEvent) => {
    if (!castTracking) return;
    const { o, d } = pickRay(e.clientX, e.clientY);
    wasm.bodies_cast_track_move(o.x, o.y, o.z, d.x, d.y, d.z);
  };
  const onCastUp = () => {
    if (castTracking) {
      wasm.bodies_cast_track_up();
      castTracking = false;
    }
  };
  canvas.addEventListener("pointerdown", onCastDown, true);
  window.addEventListener("pointermove", onCastMove);
  window.addEventListener("pointerup", onCastUp);

  reset();

  let frame = 0;
  const stop = runLoop(() => {
    ctrl.tickFrame();
    // Awake-gated style refetch: bodies_styles() is a full world_draw capture, so
    // fetch it only while bodies are awake, on the first frame, and once on the
    // awake→0 settle (to pick up sleep recoloring). counters[5] = awake dynamic.
    const awake = wasm.bodies_counters()[5] ?? 0;
    const styles = styleGate(awake, () => wasm.bodies_styles());
    syncMeshesFromPoses(demo.content, pool, wasm.bodies_poses(), {
      styles,
      groundIndex: null,
    });
    if (OVERLAY_SCENES.has(scene)) bodiesOverlay.update(wasm.bodies_overlay());
    else bodiesOverlay.clear();
    if (scene === "cast") updateCastShapes(wasm.bodies_cast_shapes());
    else clearCast();
    if (scene === "gyroscopic-precession") updateTops();

    frame += 1;
    if (scene === "gyroscopic-torque" && frame % 10 === 0) {
      const hud = wasm.bodies_hud();
      if (hud) updateReadout(readout, [{ label: "center", value: hud.replace("center ", "") }]);
    }
    if (scene === "gyroscopic-precession" && frame % 10 === 0) {
      // Heavy-top diagnostic (C GyroscopicPrecession::Step DrawTextLine); one line each.
      const hud = wasm.bodies_precession_hud();
      readout.innerHTML = hud
        .split("\n")
        .map((line) => `<span class="value">${line}</span>`)
        .join("<br>");
    }
    demo.render();
  }, readout);

  return () => {
    stop();
    canvas.removeEventListener("pointerdown", onCastDown, true);
    window.removeEventListener("pointermove", onCastMove);
    window.removeEventListener("pointerup", onCastUp);
    bodiesOverlay.dispose();
    clearCast();
    clearTops();
    ctrl.dispose();
    disposeMeshPool(pool);
    demo.dispose();
  };
}
