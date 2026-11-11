// Character / Dynamic Mover page mode — the browser host for the wasm port of C
// `DynamicMoverSample` (sample_character.cpp, upstream 3362633). The physics, the
// mover input (`Sample::GetMoverInput`), the aim math, and the follow camera
// (`FollowMover`: aim blend, shoulder offset, camera-collision ray) all run in Rust; this
// module owns the `DrawControls` panel, the aim / shoot mouse buttons, the K snap key,
// the mover fade (`SetBodyAlpha`), and the on-canvas `DrawTextLine` HUD. Keys, mouse
// look, wheel zoom, and pointer lock come from the shared third-person rig
// (character-rig.ts).
//
// Controls (C bindings): WASD move relative to the camera, Space jump, Shift sprint,
// Ctrl walk, T third-person toggle, right mouse aim + left mouse shoot, wheel zoom,
// K align-for-video snap. Browser additions: arrows alias WASD, C aliases Ctrl, the
// mouse locks on a canvas click, and a small dot marks the screen center (the
// "crosshair" C's FollowMover comment aims at) while aiming.

import type * as THREE from "three";
import type { Box3dWasm } from "../wasm.ts";
import { buildRangeInput, createCheckbox } from "../controls.ts";
import { applyShapeStyle, setView, type DemoScene } from "../three-scene.ts";
import type { MeshPool } from "./sim-mesh.ts";
import { createThirdPersonRig } from "./character-rig.ts";

export interface DynamicMoverHost {
  wasm: Box3dWasm;
  demo: DemoScene;
  canvas: HTMLCanvasElement;
  controls: HTMLElement;
  canvasArea: HTMLElement | null;
  keys: Set<string>;
  isActive(): boolean;
  isThirdPerson(): boolean;
}

export interface DynamicMoverMode {
  setVisible(visible: boolean): void;
  onReset(): void;
  /** Called when T / the checkbox toggles the third-person camera. */
  onThirdPerson(thirdPerson: boolean): void;
  beforeStep(): void;
  afterStep(pool: MeshPool, styles: ArrayLike<number>): void;
  readout(): { label: string; value: string }[];
  dispose(): void;
}

/** C `DynamicMoverSample` start position / `SetView( 180, 20, 6, startPosition )`. */
const START: [number, number, number] = [0, 1.5, 0];

/** C `DrawControls` sliders: [label, id, min, max, decimals]. Ids match the Rust
 *  `PARAM_*` constants (character_demo/dynamic/exports.rs). */
const SLIDERS: [string, number, number, number, number][] = [
  ["Jump Speed", 0, 0, 20, 1],
  ["Max Speed", 1, 0, 20, 0],
  ["Accelerate", 2, 0, 100, 0],
  ["Friction", 3, 0, 20, 1],
  ["Air Steer", 4, 0, 1, 2],
  ["Gravity Scale", 5, 0, 4, 1],
  ["Pogo Hertz", 6, 0, 30, 0],
  ["Pogo Damping", 7, 0, 4, 1],
  ["Ground Force", 8, 0, 20000, 0],
  ["Ball Speed", 9, 10, 200, 0],
];
/** C `DrawControls` checkboxes: [label, flag id]. Ids match the Rust `FLAG_*`. */
const CHECKS: [string, number][] = [
  ["Skirt Pogo", 0],
  ["Draw Probes", 1],
  ["Box Mover", 2],
];

// Status layout from `character_status()` for this scene (dynamic/exports.rs `step`).
const S = {
  px: 0, py: 1, pz: 2, vx: 3, vy: 4, vz: 5,
  onGround: 6, walkable: 7, normalY: 8, pogoLength: 9, pogoRest: 10, timeSliding: 11,
  gemsLeft: 12, gemCount: 13, moverIndex: 14, aiming: 15, alpha: 16,
  pivotX: 17, pivotY: 18, pivotZ: 19, viewRadius: 20,
} as const;

export function createDynamicMoverMode(host: DynamicMoverHost): DynamicMoverMode {
  const { wasm, demo, canvas } = host;

  let status: Float32Array = new Float32Array();
  let alphaIndex = -1;

  const rig = createThirdPersonRig({
    wasm,
    demo,
    canvas,
    canvasArea: host.canvasArea,
    keys: host.keys,
    isActive: host.isActive,
    isThirdPerson: host.isThirdPerson,
    lockedPrompt: "RMB aim, LMB shoot",
    // While aiming, a left click shoots instead of capturing the mouse.
    canLockOnClick: () => (status[S.aiming] ?? 0) < 0.5,
  });

  // --- DrawControls panel -------------------------------------------------------
  const panel = document.createElement("div");
  panel.style.display = "none";
  const checkInputs = new Map<number, HTMLInputElement>();
  for (const [label, id] of CHECKS) {
    const row = createCheckbox(label, false, (v) => wasm.character_dynamic_set_flag(id, v));
    checkInputs.set(id, row.querySelector("input") as HTMLInputElement);
    panel.appendChild(row);
  }
  const sliderInputs = new Map<number, { input: HTMLInputElement; span: HTMLElement; d: number }>();
  for (const [label, id, min, max, d] of SLIDERS) {
    if (id === 0 || id === 9) panel.appendChild(document.createElement("hr"));
    const group = document.createElement("div");
    group.className = "control-group";
    const lbl = document.createElement("label");
    lbl.appendChild(document.createTextNode(label));
    const span = document.createElement("span");
    span.className = "slider-value";
    lbl.appendChild(span);
    // ImGui SliderFloat is continuous; the C printf precision only formats the value.
    const step = (max - min) / 2000;
    const input = buildRangeInput(min, max, step, min, span, (v) => v.toFixed(d), (v) =>
      wasm.character_dynamic_set_param(id, v),
    );
    group.appendChild(lbl);
    group.appendChild(input);
    panel.appendChild(group);
    sliderInputs.set(id, { input, span, d });
  }
  host.controls.appendChild(panel);

  // --- On-canvas HUD (C DrawTextLine) + aim point ---------------------------------
  const hud = document.createElement("div");
  hud.className = "sample-draw-text";
  hud.style.pointerEvents = "none";
  hud.style.whiteSpace = "pre";
  hud.style.display = "none";
  host.canvasArea?.appendChild(hud);

  const crosshair = document.createElement("div");
  Object.assign(crosshair.style, {
    position: "absolute",
    left: "50%",
    top: "50%",
    width: "6px",
    height: "6px",
    margin: "-3px 0 0 -3px",
    borderRadius: "50%",
    background: "rgba(255,255,255,0.85)",
    boxShadow: "0 0 2px rgba(0,0,0,0.9)",
    pointerEvents: "none",
    display: "none",
  });
  host.canvasArea?.appendChild(crosshair);

  const active = () => host.isActive();

  // C button numbering (sokol): 0 left, 1 right, 2 middle.
  const toCButton = (b: number) => (b === 2 ? 1 : b === 1 ? 2 : 0);

  const onMouseDown = (e: MouseEvent) => {
    if (!active() || !host.isThirdPerson()) return;
    wasm.character_dynamic_mouse_down(toCButton(e.button));
    e.preventDefault();
  };
  const onMouseUp = (e: MouseEvent) => {
    if (!active()) return;
    wasm.character_dynamic_mouse_up(toCButton(e.button));
  };
  const onKeyDown = (e: KeyboardEvent) => {
    if (!active()) return;
    const tag = (e.target as HTMLElement)?.tagName;
    if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
    if (e.code === "KeyK" && !e.repeat) {
      // C KEY_K: align movement for video.
      const snapped = wasm.character_dynamic_snap();
      demo.controls.yaw = snapped[0] ?? demo.controls.yaw;
      demo.controls.pitch = snapped[1] ?? demo.controls.pitch;
    }
  };
  const onBlur = () => {
    // C ClearKeysDown on focus loss; release the aim too.
    if (active()) wasm.character_dynamic_mouse_up(1);
  };

  canvas.addEventListener("mousedown", onMouseDown);
  window.addEventListener("mouseup", onMouseUp);
  window.addEventListener("keydown", onKeyDown);
  window.addEventListener("blur", onBlur);

  function setVisible(visible: boolean) {
    panel.style.display = visible ? "" : "none";
    hud.style.display = visible ? "" : "none";
    if (!visible) crosshair.style.display = "none";
    rig.setVisible(visible);
  }

  function onReset() {
    const p = wasm.character_dynamic_params();
    for (const [id, s] of sliderInputs) {
      const v = p[id] ?? 0;
      s.input.value = String(v);
      s.span.textContent = v.toFixed(s.d);
    }
    for (const [id, input] of checkInputs) input.checked = (p[10 + id] ?? 0) > 0.5;
    // C: m_camera->SetView( 180.0f, 20.0f, 6.0f, startPosition ).
    setView(demo, 180, 20, 6, START);
    rig.setRadius(6);
    rig.setVisible(true);
    alphaIndex = -1;
    status = new Float32Array();
  }

  function afterStep(pool: MeshPool, styles: ArrayLike<number>) {
    status = wasm.character_status();
    const third = host.isThirdPerson();
    // Render from the pivot / radius C FollowMover left on the camera this frame.
    rig.place(
      status.length > S.viewRadius
        ? [status[S.pivotX]!, status[S.pivotY]!, status[S.pivotZ]!]
        : null,
      status[S.viewRadius] ?? rig.radius(),
    );

    // C SetBodyAlpha( moverId, 1 - 0.7 * blend ): fade the mover for the aim camera.
    const index = Math.round(status[S.moverIndex] ?? -1);
    if (alphaIndex >= 0 && alphaIndex !== index) restoreAlpha(pool, alphaIndex, styles);
    const mesh = pool.meshes[index] as THREE.Mesh | undefined;
    if (mesh) {
      const alpha = status[S.alpha] ?? 1;
      const mat = mesh.material as THREE.MeshStandardMaterial;
      // C: c.w = min( c.w, bodyAlpha ) on top of the transparent-dynamic mode's 0.5.
      const styleAlpha = ((styles[index] ?? 0) >>> 29) & 1 ? 0.5 : 1;
      const a = Math.min(alpha, styleAlpha);
      const want = a < 1;
      if (mat.transparent !== want) {
        mat.transparent = want;
        mat.needsUpdate = true;
      }
      mat.opacity = a;
      mat.depthWrite = !want;
      alphaIndex = index;
    }

    const aiming = (status[S.aiming] ?? 0) > 0.5;
    crosshair.style.display = aiming && third ? "" : "none";

    const f2 = (i: number) => (status[i] ?? 0).toFixed(2);
    const gemsLeft = Math.round(status[S.gemsLeft] ?? 0);
    const gemCount = Math.round(status[S.gemCount] ?? 0);
    hud.textContent =
      "WASD = move, Space = jump, Shift = sprint, Ctrl = walk\n" +
      "T = camera, RMB = aim, LMB = shoot\n" +
      `on ground ${Math.round(status[S.onGround] ?? 0)}, walkable ${Math.round(status[S.walkable] ?? 0)}, normal y ${f2(S.normalY)}\n` +
      `pogo length ${f2(S.pogoLength)} / ${f2(S.pogoRest)}\n` +
      `time sliding ${f2(S.timeSliding)}\n` +
      (gemsLeft === 0
        ? `all ${gemCount} gems collected!`
        : `gems ${gemCount - gemsLeft} / ${gemCount}`);
  }

  function restoreAlpha(pool: MeshPool, i: number, styles: ArrayLike<number>) {
    const mesh = pool.meshes[i] as THREE.Mesh | undefined;
    if (!mesh) return;
    mesh.userData.styleWord = undefined;
    (mesh.material as THREE.MeshStandardMaterial).depthWrite = true;
    applyShapeStyle(mesh, styles[i] ?? 0);
  }

  function readout() {
    const gemsLeft = Math.round(status[S.gemsLeft] ?? 0);
    const gemCount = Math.round(status[S.gemCount] ?? 0);
    const speed = Math.hypot(status[S.vx] ?? 0, status[S.vz] ?? 0);
    return [
      { label: "scene", value: "dynamic-mover" },
      { label: "gems", value: `${gemCount - gemsLeft} / ${gemCount}` },
      { label: "ground", value: (status[S.onGround] ?? 0) > 0.5 ? "yes" : "no" },
      { label: "y", value: (status[S.py] ?? 0).toFixed(2) },
      { label: "speed", value: speed.toFixed(2) },
    ];
  }

  function dispose() {
    canvas.removeEventListener("mousedown", onMouseDown);
    window.removeEventListener("mouseup", onMouseUp);
    window.removeEventListener("keydown", onKeyDown);
    window.removeEventListener("blur", onBlur);
    rig.dispose();
    panel.remove();
    hud.remove();
    crosshair.remove();
  }

  return {
    setVisible,
    onReset,
    onThirdPerson: (v: boolean) => rig.onThirdPerson(v),
    beforeStep: () => {
      rig.beforeStep();
    },
    afterStep,
    readout,
    dispose,
  };
}
