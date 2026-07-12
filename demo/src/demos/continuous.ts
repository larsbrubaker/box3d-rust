// Continuous — Thin Wall, Bounce House, Bullet vs Stack (sample_continuous.cpp).

import * as THREE from "three";
import { createButton, createInfoBox } from "../controls.ts";
import {
  attachInteraction,
  type ParamValues,
  type SimControllerWithTick,
} from "../interaction.ts";
import { getWasm } from "../wasm.ts";
import { demoPage, runLoop } from "./common.ts";
import { applyBodyColor, DemoScene, makeBodyMaterial } from "../three-scene.ts";

/** `[px..qw, hx,hy,hz, kind, bodyType, awake]` */
const STRIDE = 13;

type Mode = "thin" | "bounce" | "bullet";

function cameraFromView(
  demo: DemoScene,
  yawDeg: number,
  pitchDeg: number,
  distance: number,
  target: [number, number, number],
) {
  demo.controls.target.set(target[0], target[1], target[2]);
  const yaw = (yawDeg * Math.PI) / 180;
  const pitch = (pitchDeg * Math.PI) / 180;
  demo.camera.position.set(
    target[0] + distance * Math.cos(pitch) * Math.sin(yaw),
    target[1] + distance * Math.sin(pitch),
    target[2] + distance * Math.cos(pitch) * Math.cos(yaw),
  );
  demo.controls.update();
}

export function init(container: HTMLElement) {
  const wasm = getWasm();
  const { canvas, controls } = demoPage(
    container,
    "Continuous",
    "Official Continuous samples from <code>sample_continuous.cpp</code>: Thin Wall, " +
      "Bounce House, and Bullet vs Stack — fast bodies with continuous collision.",
    "Pick a sample · Launch (L) on Bullet vs Stack · drag · P/O/R",
    wasm.version(),
    { category: "Continuous", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "<strong>Thin Wall</strong> — sphere, capsule, and box fly into a thin wall at 180 m/s.<br>" +
        "<strong>Bounce House</strong> — restitution-1 sphere ricochets inside walls (no gravity).<br>" +
        "<strong>Bullet vs Stack</strong> — press Launch or <kbd>L</kbd> to fire an " +
        "<code>is_bullet</code> sphere into a 10-box stack.",
    ),
  );

  let mode: Mode = "thin";
  let launchRow: HTMLElement | null = null;

  const demo = new DemoScene(canvas, {
    target: [0, 10, 0],
    distance: 30,
    shadowExtent: 48,
  });
  const meshes: THREE.Mesh[] = [];
  const boxGeo = new THREE.BoxGeometry(2, 2, 2);
  const sphereGeo = new THREE.SphereGeometry(1, 20, 14);
  const staticMat = makeBodyMaterial(0, true);
  const dynamicMat = makeBodyMaterial(2, true);
  const sleepMat = makeBodyMaterial(2, false);

  function clearMeshes() {
    for (const m of meshes) {
      demo.content.remove(m);
      if (m.geometry !== boxGeo && m.geometry !== sphereGeo) m.geometry.dispose();
    }
    meshes.length = 0;
  }

  function setCameraForMode() {
    if (mode === "thin") {
      cameraFromView(demo, 45, 30, 30, [0, 0, 0]);
    } else if (mode === "bounce") {
      cameraFromView(demo, 45, 45, 50, [0, 0, 0]);
    } else {
      cameraFromView(demo, 15, 20, 30, [0, 2, 0]);
    }
  }

  function pickMat(bodyType: number, awake: boolean): THREE.MeshStandardMaterial {
    const mat = bodyType === 0 ? staticMat : awake ? dynamicMat : sleepMat;
    applyBodyColor(mat, bodyType, awake);
    return mat;
  }

  function ensureMesh(i: number, kind: number, bodyType: number, awake: boolean): THREE.Mesh {
    let mesh = meshes[i];
    const wantSphere = kind === 1;
    const wantCapsule = kind === 2;
    const mat = pickMat(bodyType, awake);

    if (wantCapsule) {
      if (!mesh || mesh.geometry.type !== "CapsuleGeometry") {
        if (mesh) {
          demo.content.remove(mesh);
          if (mesh.geometry !== boxGeo && mesh.geometry !== sphereGeo) mesh.geometry.dispose();
        }
        mesh = new THREE.Mesh(new THREE.CapsuleGeometry(0.2, 0.5, 4, 10), mat);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        demo.content.add(mesh);
        meshes[i] = mesh;
      } else {
        mesh.material = mat;
      }
      return mesh;
    }

    if (
      !mesh ||
      (wantSphere && mesh.geometry !== sphereGeo) ||
      (!wantSphere && mesh.geometry !== boxGeo)
    ) {
      if (mesh) {
        demo.content.remove(mesh);
        if (mesh.geometry !== boxGeo && mesh.geometry !== sphereGeo) mesh.geometry.dispose();
      }
      mesh = new THREE.Mesh(wantSphere ? sphereGeo : boxGeo, mat);
      mesh.castShadow = bodyType !== 0;
      mesh.receiveShadow = true;
      demo.content.add(mesh);
      meshes[i] = mesh;
    } else {
      mesh.material = mat;
    }
    return mesh;
  }

  function syncLaunchControl() {
    if (launchRow) {
      launchRow.remove();
      launchRow = null;
    }
    if (mode !== "bullet") return;
    launchRow = document.createElement("div");
    launchRow.className = "control-group";
    const btn = createButton("Launch (L)", () => {
      wasm.sim_launch_bullet();
    });
    launchRow.appendChild(btn);
    controls.appendChild(launchRow);
  }

  function reset() {
    clearMeshes();
    if (mode === "thin") wasm.sim_reset_thin_wall();
    else if (mode === "bounce") wasm.sim_reset_bounce_house();
    else wasm.sim_reset_bullet_vs_stack();
    syncLaunchControl();
    setCameraForMode();
  }

  const onKeyDown = (e: KeyboardEvent) => {
    if (mode === "bullet" && (e.key === "l" || e.key === "L") && !e.repeat) {
      e.preventDefault();
      wasm.sim_launch_bullet();
    }
  };
  window.addEventListener("keydown", onKeyDown);
  canvas.tabIndex = 0;
  canvas.style.outline = "none";

  const ctrl = attachInteraction({
    wasm,
    demo,
    canvas,
    controls,
    onRestart: reset,
    sampleName: "Thin Wall",
    sampleCategory: "Continuous",
    params: [
      {
        type: "select",
        key: "sample",
        label: "Sample",
        options: [
          { label: "Thin Wall", value: "thin" },
          { label: "Bounce House", value: "bounce" },
          { label: "Bullet vs Stack", value: "bullet" },
        ],
        default: "thin",
        restart: true,
      },
    ],
    onParamsChange: (values: ParamValues, key: string) => {
      if (key !== "sample") return;
      mode = String(values.sample) as Mode;
      const names: Record<Mode, string> = {
        thin: "Thin Wall",
        bounce: "Bounce House",
        bullet: "Bullet vs Stack",
      };
      const nameEl = controls.querySelector(".sample-name");
      if (nameEl) nameEl.textContent = names[mode];
    },
  }) as SimControllerWithTick;

  reset();

  const quat = new THREE.Quaternion();
  const stop = runLoop(() => {
    ctrl.tickFrame();
    const poses = wasm.sim_body_poses();
    const n = Math.floor(poses.length / STRIDE);
    while (meshes.length > n) {
      const m = meshes.pop()!;
      demo.content.remove(m);
      if (m.geometry !== boxGeo && m.geometry !== sphereGeo) m.geometry.dispose();
    }
    for (let i = 0; i < n; i++) {
      const o = i * STRIDE;
      const kind = poses[o + 10]!;
      const bodyType = poses[o + 11]! | 0;
      const awake = poses[o + 12]! > 0.5;
      const mesh = ensureMesh(i, kind, bodyType, awake);
      mesh.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
      quat.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
      mesh.quaternion.copy(quat);
      if (kind === 2) {
        const radius = poses[o + 7]!;
        const halfLen = poses[o + 8]!;
        const geo = mesh.geometry as THREE.CapsuleGeometry;
        if (
          Math.abs(geo.parameters.radius - radius) > 1e-4 ||
          Math.abs(geo.parameters.length - Math.max(1e-4, halfLen * 2)) > 1e-3
        ) {
          geo.dispose();
          mesh.geometry = new THREE.CapsuleGeometry(radius, Math.max(1e-4, halfLen * 2), 4, 10);
        }
      } else if (kind === 1) {
        mesh.scale.setScalar(poses[o + 7]!);
      } else {
        mesh.scale.set(poses[o + 7]!, poses[o + 8]!, poses[o + 9]!);
      }
    }
    demo.render();
  }, controls);

  return () => {
    window.removeEventListener("keydown", onKeyDown);
    ctrl.dispose();
    stop();
    clearMeshes();
    demo.dispose();
    boxGeo.dispose();
    sphereGeo.dispose();
    staticMat.dispose();
    dynamicMat.dispose();
    sleepMat.dispose();
  };
}
