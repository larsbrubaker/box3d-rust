// Character samples — a 1:1 port of the four upstream `sample_character.cpp`
// scenes: CapsulePlane, MoverOverlap, Mover (BasicMover), and Rigid Body.
//
// - CapsulePlane / MoverOverlap: drag a query capsule with the mouse into static
//   shapes; watch the returned collision planes and the b3SolvePlanes push-out.
// - Mover: the real BasicMover level (test_map01 + stairs + torus mesh + wave
//   height field + spring door + enemy/friendly capsules + dynamic sphere) driven
//   by the C-exact CharacterMover. WASD / Space / Shift, Third Person + Clip
//   Velocity controls.
// - Rigid Body: the s&box-style dynamic character over the full obstacle course.

import * as THREE from "three";
import {
  createButton,
  createButtonGroup,
  createCheckbox,
  createInfoBox,
  createReadout,
  updateReadout,
} from "../controls.ts";
import { getWasm } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { demoPage, runLoop } from "./common.ts";
import {
  DemoScene,
  makeTriangleMesh,
  makeWireEdges,
  setView,
  trianglesFromWireframe,
} from "../three-scene.ts";
import { pickRay } from "../interaction.ts";
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

type Scene = "capsule-plane" | "mover-overlap" | "mover" | "rigid-body";

export const SCENES: Scene[] = ["capsule-plane", "mover-overlap", "mover", "rigid-body"];

const isDrag = (s: Scene) => s === "capsule-plane" || s === "mover-overlap";
const isWalker = (s: Scene) => s === "mover" || s === "rigid-body";

// Cached OBJ fetches (mirrors the Mesh Voxel / Shapes Conveyor async pattern).
const objCache = new Map<string, Promise<string>>();
function loadObj(name: string): Promise<string> {
  let p = objCache.get(name);
  if (!p) {
    p = fetch(`/public/meshes/${name}`).then((r) => {
      if (!r.ok) throw new Error(`${name} HTTP ${r.status}`);
      return r.text();
    });
    objCache.set(name, p);
  }
  return p;
}

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  assertRouteScenes("character", SCENES);

  const { canvas, controls } = demoPage(
    container,
    "Character",
    "Erin Catto’s four <code>sample_character.cpp</code> scenes: drag a query capsule into " +
      "shapes (CapsulePlane, MoverOverlap), or walk the real level with the C-exact " +
      "<code>CharacterMover</code> (Mover) and the s&box-style dynamic character (Rigid Body).",
    "drag capsule · WASD/arrows move · Space jump · Shift sprint · drag to orbit",
    wasm.version(),
    { category: "Character", samplesShell: true },
  );

  let scene: Scene =
    initialScene && SCENES.includes(initialScene as Scene) ? (initialScene as Scene) : "capsule-plane";
  let thirdPerson = false;
  let clipVelocity = true;

  const info = createInfoBox("");
  controls.appendChild(info);

  const sceneButtons = createButtonGroup(
    [
      { label: "CapsulePlane", value: "capsule-plane" },
      { label: "MoverOverlap", value: "mover-overlap" },
      { label: "Mover", value: "mover" },
      { label: "Rigid Body", value: "rigid-body" },
      { label: "Respawn", value: "respawn" },
    ],
    scene,
    (v) => {
      if (v === "respawn") {
        void reset();
        return;
      }
      scene = v as Scene;
      void reset();
    },
  );
  controls.appendChild(sceneButtons);

  // Scene-specific controls (shown/hidden per scene).
  const solveBtn = createButton("Solve (push out)", () => wasm.character_solve());
  const clipRow = createCheckbox("Clip Velocity", clipVelocity, (v) => {
    clipVelocity = v;
    wasm.character_set_clip_velocity(v);
  });
  const thirdRow = createCheckbox("Third Person (T)", thirdPerson, (v) => {
    thirdPerson = v;
    wasm.character_set_third_person(v);
    if (!v && pointerLocked) document.exitPointerLock?.();
    updateLookHint();
  });
  controls.appendChild(solveBtn);
  controls.appendChild(clipRow);
  controls.appendChild(thirdRow);

  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { target: [0, 1, 0], distance: 8 });
  demo.camera.far = 800;
  demo.camera.updateProjectionMatrix();

  const pool = createMeshPool();

  // Static ground wireframe (mesh + height field), rebuilt on each reset.
  let groundMesh: THREE.Mesh | null = null;
  let groundWire: THREE.LineSegments | null = null;
  function clearGround() {
    if (groundMesh) {
      demo.content.remove(groundMesh);
      groundMesh.geometry.dispose();
      (groundMesh.material as THREE.Material).dispose();
      groundMesh = null;
    }
    if (groundWire) {
      demo.content.remove(groundWire);
      groundWire.geometry.dispose();
      (groundWire.material as THREE.Material).dispose();
      groundWire = null;
    }
  }
  function buildGround() {
    clearGround();
    const wire = wasm.character_ground_wireframe();
    if (wire.length < 18) return;
    groundMesh = makeTriangleMesh(trianglesFromWireframe(wire), 0x5a7a62, 0.9);
    groundMesh.receiveShadow = true;
    groundWire = makeWireEdges(wire, 0x2f4035, 0.3);
    demo.content.add(groundMesh);
    demo.content.add(groundWire);
  }

  // Colored debug line segments ([x0,y0,z0,x1,y1,z1,color] septuples).
  const segGeo = new THREE.BufferGeometry();
  const segMat = new THREE.LineBasicMaterial({ vertexColors: true });
  const segLines = new THREE.LineSegments(segGeo, segMat);
  segLines.frustumCulled = false;
  demo.dynamic.add(segLines);

  // Colored debug points ([x,y,z,color,size] quintuples).
  const ptGeo = new THREE.BufferGeometry();
  const ptMat = new THREE.PointsMaterial({ vertexColors: true, size: 0.16 });
  const ptPoints = new THREE.Points(ptGeo, ptMat);
  ptPoints.frustumCulled = false;
  demo.dynamic.add(ptPoints);

  const _col = new THREE.Color();
  function updateSegments(data: Float32Array) {
    const n = Math.floor(data.length / 7);
    const pos = new Float32Array(n * 6);
    const col = new Float32Array(n * 6);
    for (let i = 0; i < n; i++) {
      const b = i * 7;
      pos[i * 6 + 0] = data[b]!;
      pos[i * 6 + 1] = data[b + 1]!;
      pos[i * 6 + 2] = data[b + 2]!;
      pos[i * 6 + 3] = data[b + 3]!;
      pos[i * 6 + 4] = data[b + 4]!;
      pos[i * 6 + 5] = data[b + 5]!;
      _col.setHex(data[b + 6]! & 0xffffff);
      for (const off of [0, 3]) {
        col[i * 6 + off] = _col.r;
        col[i * 6 + off + 1] = _col.g;
        col[i * 6 + off + 2] = _col.b;
      }
    }
    segGeo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
    segGeo.setAttribute("color", new THREE.BufferAttribute(col, 3));
    segGeo.computeBoundingSphere();
  }
  function updatePoints(data: Float32Array) {
    const n = Math.floor(data.length / 5);
    const pos = new Float32Array(n * 3);
    const col = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) {
      const b = i * 5;
      pos[i * 3] = data[b]!;
      pos[i * 3 + 1] = data[b + 1]!;
      pos[i * 3 + 2] = data[b + 2]!;
      _col.setHex(data[b + 3]! & 0xffffff);
      col[i * 3] = _col.r;
      col[i * 3 + 1] = _col.g;
      col[i * 3 + 2] = _col.b;
    }
    ptGeo.setAttribute("position", new THREE.BufferAttribute(pos, 3));
    ptGeo.setAttribute("color", new THREE.BufferAttribute(col, 3));
    ptGeo.computeBoundingSphere();
  }

  // --- Keyboard (Mover / Rigid Body) ---
  const keys = new Set<string>();
  // Arrow keys alias WASD; preventDefault on the movement keys so arrows/Space
  // never scroll the page. Skip while a form field is focused (don't steal typing).
  const moveKeys = ["KeyW", "KeyA", "KeyS", "KeyD", "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", "Space"];
  const onKeyDown = (e: KeyboardEvent) => {
    const tag = (e.target as HTMLElement)?.tagName;
    if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
    keys.add(e.code);
    if (e.code === "KeyT") {
      thirdPerson = !thirdPerson;
      wasm.character_set_third_person(thirdPerson);
      if (!thirdPerson && pointerLocked) document.exitPointerLock?.();
      updateLookHint();
    }
    if (moveKeys.includes(e.code)) e.preventDefault();
  };
  const onKeyUp = (e: KeyboardEvent) => keys.delete(e.code);
  window.addEventListener("keydown", onKeyDown);
  window.addEventListener("keyup", onKeyUp);
  canvas.tabIndex = 0;
  canvas.style.outline = "none";

  // --- Mouse drag (CapsulePlane / MoverOverlap) ---
  let lastStatus: Float32Array = new Float32Array();
  let dragging = false;
  const dragOrigin = new THREE.Vector3();
  const dragBase = new THREE.Vector3();
  const _dir = new THREE.Vector3();

  function rayPoint10(clientX: number, clientY: number, out: THREE.Vector3) {
    const { origin, translation } = pickRay(demo, canvas, clientX, clientY);
    _dir.copy(translation).normalize();
    out.copy(origin).addScaledVector(_dir, 10);
  }
  const onPointerDown = (e: PointerEvent) => {
    if (!isDrag(scene) || e.button !== 0 || e.ctrlKey || e.altKey) return;
    rayPoint10(e.clientX, e.clientY, dragOrigin);
    dragBase.set(lastStatus[2] ?? 0, lastStatus[3] ?? 0, lastStatus[4] ?? 0);
    dragging = true;
    canvas.setPointerCapture(e.pointerId);
    e.preventDefault();
  };
  const onPointerMove = (e: PointerEvent) => {
    if (!dragging) return;
    const p = new THREE.Vector3();
    rayPoint10(e.clientX, e.clientY, p);
    p.sub(dragOrigin).add(dragBase);
    wasm.character_set_drag(p.x, p.y, p.z);
  };
  const onPointerUp = (e: PointerEvent) => {
    if (dragging) {
      dragging = false;
      try {
        canvas.releasePointerCapture(e.pointerId);
      } catch {
        /* ignore */
      }
    }
  };
  canvas.addEventListener("pointerdown", onPointerDown);
  canvas.addEventListener("pointermove", onPointerMove);
  canvas.addEventListener("pointerup", onPointerUp);
  canvas.addEventListener("pointercancel", onPointerUp);

  // --- Rigid Body FPS look (Pointer Lock) + third-person camera boom -----------
  // The C RigidBody sample calls sapp_lock_mouse(true) and drives yaw/pitch from the
  // host's relative mouse deltas (sample.cpp MouseMove :1271-1283), running a
  // third-person camera with a boom raycast that clamps the eye distance on a hit
  // (sample_character.cpp :1571-1602). The browser can only lock the pointer from a
  // user gesture, so we lock on canvas click while the Rigid Body scene is in
  // third-person, and Esc releases (standard browser behavior). Orbit / Alt-drag still
  // work as an unlocked fallback since the boom just reads controls.yaw / pitch.
  const LOOK_SENS = 0.1 * (Math.PI / 180); // C sensitivity: 0.1 deg per pixel, in radians
  const PITCH_LIMIT = 85 * (Math.PI / 180); // C clamps pitch to ±85°
  // Third-person boom distance the user chose (wheel / Alt+right-drag); the boom only
  // ever shortens it for a frame and restores, exactly like C's m_radius save/restore.
  let boomRadius = 6;
  let lastBoomApplied = 6;
  let pointerLocked = false;

  const canvasArea = (canvas.closest(".demo-canvas-area") as HTMLElement | null) ?? canvas.parentElement;
  const lookHint = document.createElement("div");
  lookHint.className = "sample-draw-text";
  lookHint.style.bottom = "0.6em";
  lookHint.style.top = "auto";
  lookHint.style.left = "50%";
  lookHint.style.transform = "translateX(-50%)";
  lookHint.style.pointerEvents = "none";
  lookHint.style.display = "none";
  canvasArea?.appendChild(lookHint);

  function updateLookHint() {
    if (scene !== "rigid-body" || !thirdPerson) {
      lookHint.style.display = "none";
      return;
    }
    lookHint.style.display = "";
    lookHint.textContent = pointerLocked
      ? "Mouse look active — press Esc to release"
      : "Click to look around (Esc releases)";
  }

  const onLookMove = (e: MouseEvent) => {
    if (!pointerLocked) return;
    // Raw relative deltas (sample.cpp: m_mouseDelta = { mouseDX, mouseDY }).
    demo.controls.yaw -= 2 * LOOK_SENS * e.movementX;
    demo.controls.pitch += LOOK_SENS * e.movementY;
    if (demo.controls.pitch > PITCH_LIMIT) demo.controls.pitch = PITCH_LIMIT;
    else if (demo.controls.pitch < -PITCH_LIMIT) demo.controls.pitch = -PITCH_LIMIT;
  };
  const onLookClick = () => {
    if (scene === "rigid-body" && thirdPerson && !pointerLocked) {
      canvas.requestPointerLock?.();
    }
  };
  const onPointerLockChange = () => {
    pointerLocked = document.pointerLockElement === canvas;
    updateLookHint();
  };
  canvas.addEventListener("click", onLookClick);
  document.addEventListener("mousemove", onLookMove);
  document.addEventListener("pointerlockchange", onPointerLockChange);

  let sceneReady = false;

  function applyCamera() {
    switch (scene) {
      case "capsule-plane":
        setView(demo, 120, 22, 6, [0, 1, 0.7]);
        break;
      case "mover-overlap":
        setView(demo, 120, 18, 9, [0, 1, 0]);
        break;
      case "mover":
        setView(demo, 35, 20, 14, [7.5, 1, 9]);
        break;
      case "rigid-body":
        setView(demo, 30, 18, 6, [7.5, 2, 9]);
        // Adopt the framed distance as the user's chosen boom length.
        boomRadius = 6;
        lastBoomApplied = 6;
        break;
    }
    demo.camera.far = 800;
    demo.camera.updateProjectionMatrix();
  }

  function updateControlVisibility() {
    solveBtn.style.display = scene === "capsule-plane" ? "" : "none";
    clipRow.style.display = scene === "mover" ? "" : "none";
    thirdRow.style.display = isWalker(scene) ? "" : "none";
    const hints: Record<Scene, string> = {
      "capsule-plane":
        "Drag the green capsule into the box; yellow marks the returned planes. " +
        "<b>Solve</b> pushes it out with <code>b3SolvePlanes</code>.",
      "mover-overlap":
        "Drag the yellow capsule into the sphere / capsule / box. Lime arrows are valid plane " +
        "normals (red = degenerate — must stay 0); the cyan capsule is the solved push-out.",
      mover:
        "Walk the C-exact <code>CharacterMover</code> over the real level. Blue capsule + purple " +
        "velocity + green/gray pogo ray. It shoves the dynamic sphere and swings the spring door. " +
        "Click the canvas to focus keys.",
      "rigid-body":
        "The s&box dynamic character (green feet box + blue capsule) with trace-based step-up over " +
        "the obstacle course. Purple velocity, orange wish, yellow mass center. <b>Click to look</b> " +
        "(FPS mouse-look, Esc releases); the camera boom won't clip walls you back into.",
    };
    info.innerHTML = hints[scene];
    updateLookHint();
  }

  async function reset(): Promise<void> {
    sceneReady = false;
    dragging = false;
    updateControlVisibility();
    applyCamera();

    if (scene === "capsule-plane") {
      wasm.character_reset(0);
      clearGround();
      sceneReady = true;
    } else if (scene === "mover-overlap") {
      wasm.character_reset(1);
      clearGround();
      sceneReady = true;
    } else if (scene === "mover") {
      const activeScene = scene;
      const [map, stairs] = await Promise.all([
        loadObj("test_map01.obj"),
        loadObj("stairs.obj"),
      ]);
      if (scene !== activeScene) return;
      wasm.character_reset_mover(map, stairs);
      wasm.character_set_clip_velocity(clipVelocity);
      wasm.character_set_third_person(thirdPerson);
      buildGround();
      sceneReady = true;
    } else {
      const activeScene = scene;
      const [map, stairs, building, v1, v2] = await Promise.all([
        loadObj("test_map01.obj"),
        loadObj("stairs.obj"),
        loadObj("building.obj"),
        loadObj("voxel_mesh_01.obj"),
        loadObj("voxel_mesh_02.obj"),
      ]);
      if (scene !== activeScene) return;
      wasm.character_reset_rigid_body(map, stairs, building, v1, v2);
      thirdPerson = true;
      thirdRow.querySelector("input")?.setAttribute("checked", "true");
      wasm.character_set_third_person(true);
      // Rigid Body forces third-person on; refresh the look hint (updateControlVisibility
      // ran before this with the old thirdPerson value, so the hint was hidden).
      updateLookHint();
      buildGround();
      sceneReady = true;
    }
  }

  void reset();

  const _target = new THREE.Vector3();
  let frame = 0;

  const stop = runLoop(
    () => {
      // Camera-relative heading for the walkers.
      const cam = demo.camera.position;
      const tgt = demo.controls.target;
      let fwdX = tgt.x - cam.x;
      let fwdZ = tgt.z - cam.z;
      const fl = Math.hypot(fwdX, fwdZ) || 1;
      fwdX /= fl;
      fwdZ /= fl;
      const rightX = -fwdZ;
      const rightZ = fwdX;

      if (isWalker(scene)) {
        let tx = 0;
        let ty = 0;
        if (keys.has("KeyW") || keys.has("ArrowUp")) tx += 1;
        if (keys.has("KeyS") || keys.has("ArrowDown")) tx -= 1;
        if (keys.has("KeyA") || keys.has("ArrowLeft")) ty -= 1;
        if (keys.has("KeyD") || keys.has("ArrowRight")) ty += 1;
        const jump = keys.has("Space");
        const sprint = keys.has("ShiftLeft") || keys.has("ShiftRight");
        wasm.character_set_input(tx, ty, jump, sprint, fwdX, fwdZ, rightX, rightZ);
      }

      wasm.character_step(1 / 60, 4);

      const poses = wasm.character_poses();
      syncMeshesFromPoses(demo.content, pool, poses, {
        groundIndex: null,
        styles: wasm.character_styles(),
      });

      updateSegments(wasm.character_debug_segments());
      updatePoints(wasm.character_debug_points());

      lastStatus = wasm.character_status();

      // Third-person / follow camera for the walkers.
      if (scene === "rigid-body") {
        // C-exact third-person boom (sample_character.cpp :1571-1602): pivot on the
        // character, cast a ray toward the desired eye, and clamp the boom on a hit so
        // the camera never clips a wall the player backs into. Restored each frame so
        // the eye springs back out once the wall is clear.
        const follow = wasm.character_follow_target();
        if (follow.length >= 3) {
          _target.set(follow[0]!, follow[1]!, follow[2]!);
          demo.controls.pivot.copy(_target);
          // Fold wheel / Alt+right-drag zoom (which lands in controls.radius) back into
          // the user's chosen boom length; the boom itself never persists into radius.
          if (Math.abs(demo.controls.radius - lastBoomApplied) > 1e-6) {
            boomRadius = demo.controls.radius;
          }
          // Desired eye at full boom radius, using the mouse-look yaw / pitch. forward
          // is the +view-Z (pivot→eye) direction, matching CameraControls.
          const yaw = demo.controls.yaw;
          const pitch = demo.controls.pitch;
          const cp = Math.cos(pitch);
          const ex = _target.x + Math.sin(yaw) * cp * boomRadius;
          const ey = _target.y + Math.sin(pitch) * boomRadius;
          const ez = _target.z + Math.cos(yaw) * cp * boomRadius;
          let applied = boomRadius;
          if (boomRadius > 0.01) {
            const frac = wasm.character_camera_boom(_target.x, _target.y, _target.z, ex, ey, ez);
            if (frac < 1) {
              // C margins: 0.15 m camera radius, 0.1 m floor; radius = min(saved, clamped).
              applied = Math.min(boomRadius, Math.max(0.1, frac * boomRadius - 0.15));
            }
          }
          // Only set radius; never write camera.position directly, so CameraControls'
          // external-move reconciliation never ratchets the boom inward. render()'s
          // update() positions the eye from pivot + forward * radius.
          demo.controls.radius = applied;
          lastBoomApplied = applied;
        }
      } else if (scene === "mover") {
        const follow = wasm.character_follow_target();
        if (follow.length >= 3) {
          _target.set(follow[0]!, follow[1]! + 0.5, follow[2]!);
          if (thirdPerson) {
            // Chase: translate the camera rigidly with the character so the boom trails.
            const dx = _target.x - tgt.x;
            const dy = _target.y - tgt.y;
            const dz = _target.z - tgt.z;
            demo.camera.position.set(cam.x + dx, cam.y + dy, cam.z + dz);
          }
          demo.controls.target.copy(_target);
          demo.controls.update();
        }
      }

      frame += 1;
      if (frame % 10 === 0) {
        if (isDrag(scene)) {
          updateReadout(readout, [
            { label: "scene", value: scene },
            { label: "planes", value: String(Math.round(lastStatus[0] ?? 0)) },
            { label: "degenerate", value: String(Math.round(lastStatus[1] ?? 0)) },
          ]);
        } else {
          const speed = Math.hypot(lastStatus[3] ?? 0, lastStatus[5] ?? 0);
          updateReadout(readout, [
            { label: "scene", value: scene },
            { label: "ground", value: (lastStatus[6] ?? 0) > 0.5 ? "yes" : "no" },
            { label: "sprint", value: (lastStatus[7] ?? 0) > 0.5 ? "yes" : "no" },
            { label: "y", value: (lastStatus[1] ?? 0).toFixed(2) },
            { label: "speed", value: speed.toFixed(2) },
          ]);
        }
      }
      demo.render();
    },
    readout,
    { ready: () => sceneReady },
  );

  return () => {
    stop();
    window.removeEventListener("keydown", onKeyDown);
    window.removeEventListener("keyup", onKeyUp);
    canvas.removeEventListener("pointerdown", onPointerDown);
    canvas.removeEventListener("pointermove", onPointerMove);
    canvas.removeEventListener("pointerup", onPointerUp);
    canvas.removeEventListener("pointercancel", onPointerUp);
    canvas.removeEventListener("click", onLookClick);
    document.removeEventListener("mousemove", onLookMove);
    document.removeEventListener("pointerlockchange", onPointerLockChange);
    if (pointerLocked) document.exitPointerLock?.();
    lookHint.remove();
    clearGround();
    disposeMeshPool(pool);
    segGeo.dispose();
    segMat.dispose();
    ptGeo.dispose();
    ptMat.dispose();
    demo.dispose();
  };
}
