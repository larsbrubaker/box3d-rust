// Ragdolls — the four sample_ragdoll.cpp samples: Box, Mesh, Pile, Incline.
// Capsule-bone humans from the ported create_human API; ground meshes render as a
// baked wireframe. Follows the stacking.ts multi-scene pattern (SCENES export,
// scene selector, deep links, assertRouteScenes self-check).

import type * as THREE from "three";
import {
  createButtonGroup,
  createInfoBox,
  createReadout,
  createSlider,
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
import { createMeshPool, disposeMeshPool, syncMeshesFromPoses } from "./sim-mesh.ts";

type Mode = "box" | "mesh" | "pile" | "incline";

export const SCENES: Mode[] = ["box", "mesh", "pile", "incline"];

const RAGDOLL_NAMES: Record<Mode, string> = {
  box: "Box",
  mesh: "Mesh",
  pile: "Pile",
  incline: "Incline",
};

const SCENE_ID: Record<Mode, number> = { box: 0, mesh: 1, pile: 2, incline: 3 };

// C Ragdoll*::SetView(yaw, pitch, distance, target).
const CAMERAS: Record<Mode, [number, number, number, [number, number, number]]> = {
  box: [45, 30, 6, [0, 0, 0]],
  mesh: [45, 30, 6, [0, 0, 0]],
  pile: [180, 30, 20, [0, 0, 0]],
  incline: [-20, 30, 25, [0, 0, 0]],
};

export function init(container: HTMLElement, initialScene?: string) {
  const wasm = getWasm();
  assertRouteScenes("ragdolls", SCENES);
  const { canvas, controls } = demoPage(
    container,
    "Ragdolls",
    "The four <code>sample_ragdoll.cpp</code> samples driving the ported <code>create_human</code> " +
      "builder: <strong>Box</strong>, <strong>Mesh</strong> (walled grid + parallel anchors), " +
      "<strong>Pile</strong> (20 stacked humans), and <strong>Incline</strong> (slides down two " +
      "tilted grid grounds, de-motorized after 2 s).",
    "Drag to orbit · pick a scene · tweak joint params · P/O/R",
    wasm.version(),
    { category: "Ragdoll", samplesShell: true },
  );

  controls.appendChild(
    createInfoBox(
      "Box / Mesh expose the C <code>DrawControls</code> joint sliders " +
        "(<code>Human_SetJoint*</code>). Pile (groupIndex per human, torque 10 / hertz 0.5) and " +
        "Incline (torque 10 / hertz 2, then friction 0.5 / hertz 0.5 after 2 s) use fixed joint " +
        "parameters, matching the C samples.",
    ),
  );

  let mode: Mode =
    initialScene && SCENES.includes(initialScene as Mode) ? (initialScene as Mode) : "box";

  // C RagdollOnBox / RagdollOnMesh joint defaults: friction 5, damping 0.7,
  // hertz 1 (Box) or 2 (Mesh).
  let friction = 5;
  let hertz = 1;
  let damping = 0.7;

  // Joint sliders (Box / Mesh only); rebuilt on scene change.
  const jointControls = document.createElement("div");

  function rebuildJointControls() {
    jointControls.replaceChildren();
    if (mode !== "box" && mode !== "mesh") {
      jointControls.style.display = "none";
      return;
    }
    jointControls.style.display = "";
    hertz = mode === "mesh" ? 2 : 1;
    friction = 5;
    damping = 0.7;
    // C DrawControls: "Joint Friction" 0–20 (%3.0f), "Hertz" 0–20 (%3.1f),
    // "Damping" 0–4 (%3.1f).
    jointControls.appendChild(
      createSlider("Joint Friction", 0, 20, friction, 1, (v) => {
        friction = v;
        wasm.ragdoll_set_joint_params(friction, hertz, damping);
      }),
    );
    jointControls.appendChild(
      createSlider("Hertz", 0, 20, hertz, 0.1, (v) => {
        hertz = v;
        wasm.ragdoll_set_joint_params(friction, hertz, damping);
      }),
    );
    jointControls.appendChild(
      createSlider("Damping", 0, 4, damping, 0.1, (v) => {
        damping = v;
        wasm.ragdoll_set_joint_params(friction, hertz, damping);
      }),
    );
  }

  controls.appendChild(
    createButtonGroup(
      SCENES.map((s) => ({ label: RAGDOLL_NAMES[s], value: s })),
      mode,
      (v) => {
        mode = v as Mode;
        const nameEl = controls.querySelector(".sample-name");
        if (nameEl) nameEl.textContent = RAGDOLL_NAMES[mode];
        rebuildJointControls();
        reset();
      },
    ),
  );
  controls.appendChild(jointControls);

  controls.appendChild(
    createButtonGroup([{ label: "Respawn", value: "restart" }], "restart", () => reset()),
  );
  const readout = createReadout();
  controls.appendChild(readout);

  const demo = new DemoScene(canvas, { target: [0, 0, 0], distance: 6, shadowExtent: 24 });
  const pool = createMeshPool();

  let groundTri: THREE.Mesh | null = null;
  let groundWire: THREE.LineSegments | null = null;

  function clearGround() {
    if (groundTri) {
      demo.content.remove(groundTri);
      groundTri.geometry.dispose();
      (groundTri.material as THREE.Material).dispose();
      groundTri = null;
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
    const wire = wasm.ragdoll_ground_wireframe();
    if (!wire.length) return;
    groundTri = makeTriangleMesh(trianglesFromWireframe(wire), 0x8a94a6, 0.9);
    groundTri.receiveShadow = true;
    groundWire = makeWireEdges(wire, 0x4a5568, 0.35);
    demo.content.add(groundTri);
    demo.content.add(groundWire);
  }

  function reset() {
    syncMeshesFromPoses(demo.content, pool, []);
    wasm.ragdoll_reset_scene(SCENE_ID[mode]);
    if (mode === "box" || mode === "mesh") {
      wasm.ragdoll_set_joint_params(friction, hertz, damping);
    }
    buildGround();
    const [yaw, pitch, dist, target] = CAMERAS[mode];
    setView(demo, yaw, pitch, dist, target);
  }

  rebuildJointControls();
  reset();

  let frame = 0;
  const stop = runLoop(() => {
    wasm.ragdoll_step(1 / 60, 4);
    const poses = wasm.ragdoll_poses();
    const styles = wasm.ragdoll_styles();
    syncMeshesFromPoses(demo.content, pool, poses, { styles });
    frame += 1;
    if (frame % 20 === 0) {
      // Count only the dynamic bodies (the human bones). The multi-scene
      // conversion renders each static ground / wall as a VisBody too, so
      // poses.length/16 over-counts (e.g. Box read one high for its ground box).
      // Body type rides in bits 27-28 of the packed style word (Dynamic = 2),
      // the same decode applyShapeStyle uses (three-scene.ts).
      let bones = 0;
      for (let i = 0; i < styles.length; i++) if (((styles[i]! >>> 27) & 0x3) === 2) bones += 1;
      updateReadout(readout, [
        { label: "scene", value: RAGDOLL_NAMES[mode] },
        { label: "bodies", value: String(bones) },
        { label: "frame", value: String(frame) },
      ]);
    }
    demo.render();
  }, readout);

  return () => {
    stop();
    clearGround();
    disposeMeshPool(pool);
    demo.dispose();
  };
}
