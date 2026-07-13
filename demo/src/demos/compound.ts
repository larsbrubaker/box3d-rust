// Compound — Simple / Spheres / Hulls / Village gallery (sample_compound.cpp).

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

type Mode = "simple" | "spheres" | "hulls" | "village";

export const SCENES: Mode[] = ["simple", "spheres", "hulls", "village"];

/** Camera helper matching the C samples' `Camera::SetView(yaw, pitch, distance, target)`. */
export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  // Self-check the scene table against the registry (see registry.ts pattern).
  assertRouteScenes("compound", SCENES);
  const { canvas, controls } = demoPage(
    container,
    "Compound",
    "Compound shape gallery from <code>sample_compound.cpp</code>: Simple, Spheres, Hulls, " +
      "and Village (real <code>building.obj</code> compound meshes from the C samples, MIT).",
    "Ctrl+click grab · Shift+click spawn · click select · P/O/R",
    wasm.version(),
    { category: "Compound", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "Village ports Erin’s Compound / Village: hull tiles, odd-tile props, and instanced " +
        "<code>data/meshes/building.obj</code> meshes. Grid is fixed at the C debug value 8 " +
        "(C release uses 200, too heavy for serial wasm). The C sample also drives a character " +
        "mover + ray/shape/overlap query sweep through the village — not yet ported here " +
        "(the walkthrough lives under Character → Village).",
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
  }

  function syncVillageBuildings() {
    clearBuildings();
    if (mode !== "village" || !buildingGeo) {
      statsEl.textContent = "";
      return;
    }
    const data = wasm.sim_village_buildings();
    const n = Math.floor(data.length / 10);
    if (n <= 0) {
      statsEl.textContent = formatVillageStats(wasm.sim_village_stats());
      return;
    }
    buildingInstanced = new THREE.InstancedMesh(buildingGeo, buildingMat, n);
    buildingInstanced.castShadow = true;
    buildingInstanced.receiveShadow = true;
    syncBuildingInstances(buildingInstanced, data);
    demo.content.add(buildingInstanced);
    statsEl.textContent = formatVillageStats(wasm.sim_village_stats());
  }

  function reset() {
    clearMeshes();
    // C SetView(yaw, pitch, distance, target) values from sample_compound.cpp.
    if (mode === "simple") {
      wasm.sim_reset_compound_simple();
      setView(demo, 45, 30, 45, [0, 0, 0]); // SimpleCompound :23
      statsEl.textContent = "";
    } else if (mode === "spheres") {
      wasm.sim_reset_compound_spheres();
      setView(demo, 45, 30, 45, [0, 0, 0]); // CompoundSpheres :117
      statsEl.textContent = "";
    } else if (mode === "hulls") {
      wasm.sim_reset_compound_hulls();
      setView(demo, 45, 30, 45, [0, 0, 0]); // CompoundHulls :178
      statsEl.textContent = "";
    } else {
      // C Village grid is fixed at the debug value 8 (release 200 is too heavy).
      wasm.sim_reset_village();
      // C Village :499 SetView(45, 10, 5, {0,10,0}) is the mover walkthrough start;
      // without the character mover the user can orbit/zoom out to survey the village.
      setView(demo, 45, 10, 5, [0, 10, 0]);
      syncVillageBuildings();
    }
  }

  controls.appendChild(
    createButtonGroup(
      [
        { label: "Simple", value: "simple" },
        { label: "Spheres", value: "spheres" },
        { label: "Hulls", value: "hulls" },
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
  const stop = runLoop(() => {
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
          !mesh ||
          (mesh.geometry as THREE.CapsuleGeometry)?.type !== "CapsuleGeometry";
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
    demo.render();
  }, controls);

  return () => {
    ctrl.dispose();
    stop();
    clearMeshes();
    demo.dispose();
    boxGeo.dispose();
    sphereGeo.dispose();
    buildingMat.dispose();
  };
}
