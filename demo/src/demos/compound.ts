// Compound — Simple / Spheres / Hulls / Tile Floor / Mesh Tile / Village gallery
// (sample_compound.cpp). Village embeds the C CharacterMover (WASD walkthrough)
// plus the sweeping ray/shape/overlap query visualization.

import * as THREE from "three";
import { createButtonGroup, createInfoBox } from "../controls.ts";
import {
  attachInteraction,
  type ParamValues,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm } from "../wasm.ts";
import { assertRouteScenes } from "../registry.ts";
import { demoPage, runLoop } from "./common.ts";
import { applyShapeStyle, DemoScene, makeShapeMaterial, setView } from "../three-scene.ts";
import {
  formatVillageStats,
  loadBuildingGeometry,
  makeBuildingMaterial,
  syncBuildingInstances,
} from "../building-mesh.ts";

/** `[px..qw, hx,hy,hz, kind, bodyType, awake]` */
const STRIDE = 13;

type Mode = "simple" | "spheres" | "hulls" | "tile-floor" | "mesh-tile" | "village";

export const SCENES: Mode[] = [
  "simple",
  "spheres",
  "hulls",
  "tile-floor",
  "mesh-tile",
  "village",
];

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  // Self-check the scene table against the registry (see registry.ts pattern).
  assertRouteScenes("compound", SCENES);
  const { canvas, controls } = demoPage(
    container,
    "Compound",
    "Compound shape gallery from <code>sample_compound.cpp</code>: Simple, Spheres, Hulls, " +
      "Tile Floor (2500-hull compound), Mesh Tile (box-mesh compound), and Village (real " +
      "<code>building.obj</code> compound meshes with the C character mover + query sweep).",
    "WASD/arrows walk (Village) · Ctrl+click grab · Shift+click spawn · click select · P/O/R",
    wasm.version(),
    { category: "Compound", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "Village ports Erin’s Compound / Village: hull tiles, odd-tile props, instanced " +
        "<code>data/meshes/building.obj</code> meshes, the embedded character mover (WASD, " +
        "space to jump, shift to sprint, T for third person) and the sweeping ray / shape-cast / " +
        "overlap-shape query visualization. Grid is fixed at the C debug value 8 (release uses " +
        "200, too heavy for serial wasm). Tile Floor keeps the C release count (2500 hulls).",
    ),
  );

  const statsEl = document.createElement("pre");
  statsEl.className = "village-stats";
  statsEl.style.cssText =
    "margin:0.5rem 0 0;padding:0.5rem 0.65rem;font:12px/1.35 ui-monospace,Consolas,monospace;" +
    "color:#d4d4d4;background:rgba(0,0,0,0.45);border-radius:4px;white-space:pre-wrap;";
  controls.appendChild(statsEl);

  let mode: Mode =
    initialScene && SCENES.includes(initialScene as Mode) ? (initialScene as Mode) : "village";

  const demo = new DemoScene(canvas, { target: [0, 4, 0], distance: 48 });
  demo.camera.far = 800;
  demo.camera.updateProjectionMatrix();

  const meshes: THREE.Object3D[] = [];
  const boxGeo = new THREE.BoxGeometry(2, 2, 2);
  const sphereGeo = new THREE.SphereGeometry(1, 20, 14);
  // Each mesh owns its material; engine style words color every body per-body.
  const buildingMat = makeBuildingMaterial();

  function disposeMesh(m: THREE.Object3D) {
    demo.content.remove(m);
    const mm = m as THREE.Mesh;
    if (mm.geometry && mm.geometry !== boxGeo && mm.geometry !== sphereGeo) {
      mm.geometry.dispose();
    }
    if (mm.material) (mm.material as THREE.Material).dispose();
  }
  let buildingInstanced: THREE.InstancedMesh | null = null;
  let buildingGeo: THREE.BufferGeometry | null = null;

  // ---- Tile Floor: instanced static tiles -------------------------------
  let tileInstanced: THREE.InstancedMesh | null = null;
  let tileGeo: THREE.BoxGeometry | null = null;
  const tileMat = makeShapeMaterial();

  function clearTiles() {
    if (tileInstanced) {
      demo.content.remove(tileInstanced);
      tileInstanced.dispose();
      tileInstanced = null;
    }
    if (tileGeo) {
      tileGeo.dispose();
      tileGeo = null;
    }
  }

  const _tp = new THREE.Vector3();
  const _tq = new THREE.Quaternion();
  const _ts = new THREE.Vector3(1, 1, 1);
  const _tm = new THREE.Matrix4();
  function syncTileFloor() {
    clearTiles();
    if (mode !== "tile-floor") return;
    const half = wasm.sim_tile_half();
    const data = wasm.sim_tile_transforms();
    const n = Math.floor(data.length / 7);
    if (n <= 0 || half.length < 3) return;
    tileGeo = new THREE.BoxGeometry(2 * half[0]!, 2 * half[1]!, 2 * half[2]!);
    tileInstanced = new THREE.InstancedMesh(tileGeo, tileMat, n);
    tileInstanced.castShadow = true;
    tileInstanced.receiveShadow = true;
    for (let i = 0; i < n; i++) {
      const o = i * 7;
      _tp.set(data[o]!, data[o + 1]!, data[o + 2]!);
      _tq.set(data[o + 3]!, data[o + 4]!, data[o + 5]!, data[o + 6]!);
      _tm.compose(_tp, _tq, _ts);
      tileInstanced.setMatrixAt(i, _tm);
    }
    tileInstanced.instanceMatrix.needsUpdate = true;
    demo.content.add(tileInstanced);
  }

  void loadBuildingGeometry()
    .then((geo) => {
      buildingGeo = geo;
      if (mode === "village") reset();
    })
    .catch((err) => console.warn("building.obj load failed", err));

  function clearBuildings() {
    if (buildingInstanced) {
      demo.content.remove(buildingInstanced);
      buildingInstanced.dispose();
      buildingInstanced = null;
    }
  }

  function clearMeshes() {
    for (const m of meshes) disposeMesh(m);
    meshes.length = 0;
    clearBuildings();
    clearTiles();
  }

  function syncVillageBuildings() {
    clearBuildings();
    if (mode !== "village" || !buildingGeo) {
      return;
    }
    const data = wasm.sim_village_buildings();
    const n = Math.floor(data.length / 10);
    if (n <= 0) return;
    buildingInstanced = new THREE.InstancedMesh(buildingGeo, buildingMat, n);
    buildingInstanced.castShadow = true;
    buildingInstanced.receiveShadow = true;
    syncBuildingInstances(buildingInstanced, data);
    demo.content.add(buildingInstanced);
  }

  // ---- Village mover + query-sweep overlays ------------------------------
  const moverMat = new THREE.MeshStandardMaterial({ color: 0x2563eb, roughness: 0.6 });
  const moverGeo = new THREE.CapsuleGeometry(0.3, 1.0, 6, 12);
  const moverMesh = new THREE.Mesh(moverGeo, moverMat);
  moverMesh.castShadow = true;
  moverMesh.visible = false;
  demo.content.add(moverMesh);

  // Cast rays (ray + shape cast) as aliceblue segments; normals as yellow.
  const castGeo = new THREE.BufferGeometry();
  const castPos = new Float32Array(12); // 2 segments
  castGeo.setAttribute("position", new THREE.BufferAttribute(castPos, 3));
  const castLines = new THREE.LineSegments(
    castGeo,
    new THREE.LineBasicMaterial({ color: 0xf0f8ff }),
  );
  castLines.visible = false;
  demo.dynamic.add(castLines);

  const normalGeo = new THREE.BufferGeometry();
  const normalPos = new Float32Array(12); // 2 segments
  normalGeo.setAttribute("position", new THREE.BufferAttribute(normalPos, 3));
  const normalLines = new THREE.LineSegments(
    normalGeo,
    new THREE.LineBasicMaterial({ color: 0xffff00 }),
  );
  normalLines.visible = false;
  demo.dynamic.add(normalLines);

  const hitGeo = new THREE.BufferGeometry();
  const hitPos = new Float32Array(6); // 2 points
  hitGeo.setAttribute("position", new THREE.BufferAttribute(hitPos, 3));
  const hitPoints = new THREE.Points(
    hitGeo,
    new THREE.PointsMaterial({ color: 0xf08080, size: 0.4 }),
  );
  hitPoints.visible = false;
  demo.dynamic.add(hitPoints);

  const querySphereGeo = new THREE.SphereGeometry(1, 16, 12);
  const shapeSphereMat = new THREE.MeshStandardMaterial({
    color: 0xda70d6,
    transparent: true,
    opacity: 0.7,
  });
  const shapeSphere = new THREE.Mesh(querySphereGeo, shapeSphereMat);
  shapeSphere.scale.setScalar(0.25);
  shapeSphere.visible = false;
  demo.content.add(shapeSphere);

  const overlapMat = new THREE.MeshStandardMaterial({ color: 0x8fbc8f });
  const overlapSphere = new THREE.Mesh(querySphereGeo, overlapMat);
  overlapSphere.scale.setScalar(0.3);
  overlapSphere.visible = false;
  demo.content.add(overlapSphere);

  function hideVillageOverlays() {
    moverMesh.visible = false;
    castLines.visible = false;
    normalLines.visible = false;
    hitPoints.visible = false;
    shapeSphere.visible = false;
    overlapSphere.visible = false;
  }

  // ---- Keyboard (Village WASD) ------------------------------------------
  const keys = new Set<string>();
  // Arrow keys alias WASD for the walkthrough; preventDefault on the movement
  // keys so arrows/Space never scroll the page. Skip while a form field is focused.
  const moveKeys = ["KeyW", "KeyA", "KeyS", "KeyD", "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", "Space"];
  const onKeyDown = (e: KeyboardEvent) => {
    const tag = (e.target as HTMLElement)?.tagName;
    if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
    keys.add(e.code);
    if (mode === "village") {
      if (e.code === "KeyT") wasm.sim_village_toggle_third_person();
      if (moveKeys.includes(e.code)) e.preventDefault();
    }
  };
  const onKeyUp = (e: KeyboardEvent) => keys.delete(e.code);
  window.addEventListener("keydown", onKeyDown);
  window.addEventListener("keyup", onKeyUp);
  canvas.tabIndex = 0;
  canvas.style.outline = "none";

  function refreshStats() {
    if (mode === "village") statsEl.textContent = formatVillageStats(wasm.sim_village_stats());
    else if (mode === "tile-floor" || mode === "mesh-tile")
      statsEl.textContent = formatVillageStats(wasm.sim_tile_stats());
    else statsEl.textContent = "";
  }

  function reset() {
    clearMeshes();
    hideVillageOverlays();
    // C SetView(yaw, pitch, distance, target) values from sample_compound.cpp.
    if (mode === "simple") {
      wasm.sim_reset_compound_simple();
      setView(demo, 45, 30, 45, [0, 0, 0]); // SimpleCompound :23
    } else if (mode === "spheres") {
      wasm.sim_reset_compound_spheres();
      setView(demo, 45, 30, 45, [0, 0, 0]); // CompoundSpheres :117
    } else if (mode === "hulls") {
      wasm.sim_reset_compound_hulls();
      setView(demo, 45, 30, 45, [0, 0, 0]); // CompoundHulls :178
    } else if (mode === "tile-floor") {
      wasm.sim_reset_tile_floor();
      setView(demo, 45, 30, 45, [0, 0, 0]); // TileFloor :252
      syncTileFloor();
    } else if (mode === "mesh-tile") {
      wasm.sim_reset_mesh_tile();
      setView(demo, 45, 30, 45, [0, 0, 0]); // MeshTile :370
    } else {
      wasm.sim_reset_village();
      // C Village :499 SetView(45, 10, 5, {0,10,0}) at the mover start.
      setView(demo, 45, 10, 5, [0, 10, 0]);
      syncVillageBuildings();
    }
    refreshStats();
  }

  controls.appendChild(
    createButtonGroup(
      [
        { label: "Simple", value: "simple" },
        { label: "Spheres", value: "spheres" },
        { label: "Hulls", value: "hulls" },
        { label: "Tile Floor", value: "tile-floor" },
        { label: "Mesh Tile", value: "mesh-tile" },
        { label: "Village", value: "village" },
      ],
      mode,
      (v) => {
        mode = v as Mode;
        reset();
      },
    ),
  );

  let subSteps = 4;

  const ctrl = attachInteraction({
    wasm,
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: "Village",
    sampleCategory: "Compound",
    params: [
      {
        type: "slider",
        key: "subSteps",
        label: "Sub-steps",
        min: 1,
        max: 8,
        step: 1,
        default: 4,
        restart: false,
      },
    ],
    onParamsChange: (values: ParamValues) => {
      subSteps = Number(values.subSteps) || 4;
      ctrl.subSteps = subSteps;
    },
  }) as SimControllerWithTick;

  ctrl.subSteps = subSteps;
  reset();

  const quat = new THREE.Quaternion();

  function driveVillage() {
    const dt = 1 / (ctrl.hertz || 60);
    const stepping = !ctrl.paused;

    // WASD throttle in camera-relative axes (as in the Character page).
    let throttleX = 0;
    let throttleY = 0;
    if (keys.has("KeyW") || keys.has("ArrowUp")) throttleX += 1;
    if (keys.has("KeyS") || keys.has("ArrowDown")) throttleX -= 1;
    if (keys.has("KeyA") || keys.has("ArrowLeft")) throttleY -= 1;
    if (keys.has("KeyD") || keys.has("ArrowRight")) throttleY += 1;
    const jump = keys.has("Space");
    const sprint = keys.has("ShiftLeft") || keys.has("ShiftRight");

    const cam = demo.camera.position;
    const target = demo.controls.target;
    let fwdX = target.x - cam.x;
    let fwdZ = target.z - cam.z;
    const fl = Math.hypot(fwdX, fwdZ) || 1;
    fwdX /= fl;
    fwdZ /= fl;
    wasm.sim_village_set_input(throttleX, throttleY, jump, sprint, fwdX, fwdZ, -fwdZ, fwdX);

    if (stepping) {
      wasm.sim_village_mover_step(dt); // C: mover.Step before world.Step
    }
    // Query sweep advances by 2*timeStep (0 while paused, matching C).
    wasm.sim_village_query_step(stepping ? dt : 0);
  }

  function renderVillageOverlays() {
    const pose = wasm.sim_village_mover_pose();
    if (pose.length >= 3) {
      moverMesh.position.set(pose[0]!, pose[1]!, pose[2]!);
      moverMesh.visible = true;
    }

    const q = wasm.sim_village_query();
    if (q.length < 39) {
      hideVillageOverlays();
      moverMesh.visible = pose.length >= 3;
      return;
    }
    // Cast segments: ray (0-2 -> 3-5), shape cast (16-18 -> 19-21).
    castPos[0] = q[0]!;
    castPos[1] = q[1]!;
    castPos[2] = q[2]!;
    castPos[3] = q[3]!;
    castPos[4] = q[4]!;
    castPos[5] = q[5]!;
    castPos[6] = q[16]!;
    castPos[7] = q[17]!;
    castPos[8] = q[18]!;
    castPos[9] = q[19]!;
    castPos[10] = q[20]!;
    castPos[11] = q[21]!;
    castGeo.attributes.position!.needsUpdate = true;
    castGeo.computeBoundingSphere();
    castLines.visible = true;

    // Hit points (ray point 7-9, shape point 26-28) + normals.
    const rayHit = q[6]! > 0.5;
    const shapeHit = q[22]! > 0.5;
    let np = 0;
    let nn = 0;
    if (rayHit) {
      hitPos[np * 3] = q[7]!;
      hitPos[np * 3 + 1] = q[8]!;
      hitPos[np * 3 + 2] = q[9]!;
      np++;
      normalPos[nn * 6] = q[7]!;
      normalPos[nn * 6 + 1] = q[8]!;
      normalPos[nn * 6 + 2] = q[9]!;
      normalPos[nn * 6 + 3] = q[7]! + 0.5 * q[10]!;
      normalPos[nn * 6 + 4] = q[8]! + 0.5 * q[11]!;
      normalPos[nn * 6 + 5] = q[9]! + 0.5 * q[12]!;
      nn++;
    }
    if (shapeHit) {
      hitPos[np * 3] = q[26]!;
      hitPos[np * 3 + 1] = q[27]!;
      hitPos[np * 3 + 2] = q[28]!;
      np++;
      normalPos[nn * 6] = q[26]!;
      normalPos[nn * 6 + 1] = q[27]!;
      normalPos[nn * 6 + 2] = q[28]!;
      normalPos[nn * 6 + 3] = q[26]! + 0.5 * q[29]!;
      normalPos[nn * 6 + 4] = q[27]! + 0.5 * q[30]!;
      normalPos[nn * 6 + 5] = q[28]! + 0.5 * q[31]!;
      nn++;
    }
    // Collapse unused slots onto the first vertex so stale points don't render.
    for (let i = np; i < 2; i++) {
      hitPos[i * 3] = hitPos[0]!;
      hitPos[i * 3 + 1] = hitPos[1]!;
      hitPos[i * 3 + 2] = hitPos[2]!;
    }
    for (let i = nn; i < 2; i++) {
      for (let k = 0; k < 6; k++) normalPos[i * 6 + k] = normalPos[k]!;
    }
    hitGeo.attributes.position!.needsUpdate = true;
    normalGeo.attributes.position!.needsUpdate = true;
    hitPoints.visible = np > 0;
    normalLines.visible = nn > 0;

    // Shape-cast sphere at the hit (23-25).
    if (shapeHit) {
      shapeSphere.position.set(q[23]!, q[24]!, q[25]!);
      shapeSphere.visible = true;
    } else {
      shapeSphere.visible = false;
    }

    // Overlap sphere (35-37) tinted by the overlap flag (38).
    overlapSphere.position.set(q[35]!, q[36]!, q[37]!);
    overlapMat.color.setHex(q[38]! > 0.5 ? 0x8b008b : 0x8fbc8f);
    overlapSphere.visible = true;
  }

  const stop = runLoop(() => {
    if (mode === "village") driveVillage();
    ctrl.tickFrame();
    const poses = wasm.sim_body_poses();
    const styles = wasm.sim_body_styles();
    const n = Math.floor(poses.length / STRIDE);
    while (meshes.length > n) {
      disposeMesh(meshes.pop()!);
    }

    for (let i = 0; i < n; i++) {
      const o = i * STRIDE;
      const kind = poses[o + 10]!;
      const hx = poses[o + 7]!;
      const hy = poses[o + 8]!;
      const hz = poses[o + 9]!;
      let mesh = meshes[i] as THREE.Mesh | undefined;
      const wantSphere = kind === 1;
      const wantCapsule = kind === 2;

      if (wantCapsule) {
        const radius = hx;
        const halfLen = Math.max(1e-4, hy);
        const needNew =
          !mesh || (mesh.geometry as THREE.CapsuleGeometry)?.type !== "CapsuleGeometry";
        if (needNew) {
          if (mesh) disposeMesh(mesh);
          mesh = new THREE.Mesh(
            new THREE.CapsuleGeometry(radius, halfLen * 2, 4, 10),
            makeShapeMaterial(),
          );
          demo.content.add(mesh);
          meshes[i] = mesh;
        }
        applyShapeStyle(mesh, styles[i]!);
        mesh.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
        quat.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
        mesh.quaternion.copy(quat);
        continue;
      }

      if (
        !mesh ||
        (wantSphere && mesh.geometry !== sphereGeo) ||
        (!wantSphere && mesh.geometry !== boxGeo)
      ) {
        if (mesh) disposeMesh(mesh);
        mesh = new THREE.Mesh(wantSphere ? sphereGeo : boxGeo, makeShapeMaterial());
        demo.content.add(mesh);
        meshes[i] = mesh;
      }
      applyShapeStyle(mesh, styles[i]!);
      mesh.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
      quat.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
      mesh.quaternion.copy(quat);
      if (kind === 1) mesh.scale.setScalar(hx);
      else mesh.scale.set(hx, hy, hz);
    }

    if (mode === "village") renderVillageOverlays();
    demo.render();
  }, controls);

  return () => {
    ctrl.dispose();
    stop();
    window.removeEventListener("keydown", onKeyDown);
    window.removeEventListener("keyup", onKeyUp);
    clearMeshes();
    demo.dispose();
    boxGeo.dispose();
    sphereGeo.dispose();
    buildingMat.dispose();
    tileMat.dispose();
    moverGeo.dispose();
    moverMat.dispose();
    castGeo.dispose();
    (castLines.material as THREE.Material).dispose();
    normalGeo.dispose();
    (normalLines.material as THREE.Material).dispose();
    hitGeo.dispose();
    (hitPoints.material as THREE.Material).dispose();
    querySphereGeo.dispose();
    shapeSphereMat.dispose();
    overlapMat.dispose();
  };
}
