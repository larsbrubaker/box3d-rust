// Cascaded shadow maps (CSM), mirroring the C samples shadow setup.
//
// C uses a 3-cascade PCF-3x3 shadow with slope-scaled bias (samples/shaders/
// shapes/geom.glsl:141-219, cube.glsl similarly; renderer configures 3 cascades
// at 2048x2048). Three's jsm CSM addon implements the same 3-cascade split and
// PCF filtering. It provides its own directional lights (one per cascade) that
// both light and shadow the scene, so this manager owns those lights.
//
// The addon requires every shadow-receiving material to be registered via
// setupMaterial(); the demos build materials themselves and this module can't
// edit them, so DemoScene registers materials transparently by traversal.

import * as THREE from "three";
import { CSM } from "three/addons/csm/CSM.js";
import { SUN_DIR, SUN_COLOR } from "./constants";
import { gridInjectShader } from "./grid";

export interface CsmManagerOptions {
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  /** Far distance the cascades cover (falls back to a shadow-focused range). */
  maxFar: number;
  /** Total sun intensity; split across cascade lights so it does not stack. */
  intensity: number;
}

/**
 * Wraps a CSM instance plus the bookkeeping to register materials once each.
 * Light direction is the direction light travels = -SUN_DIR (dir TO sun).
 */
export class CsmManager {
  readonly csm: CSM;
  private readonly registered = new WeakSet<THREE.Material>();
  private readonly cascades = 3;

  constructor(opts: CsmManagerOptions) {
    this.csm = new CSM({
      parent: opts.scene,
      camera: opts.camera,
      cascades: this.cascades,
      maxFar: opts.maxFar,
      mode: "practical",
      shadowMapSize: 2048, // C: 2048 shadow maps
      lightDirection: SUN_DIR.clone().negate().normalize(),
      // Three's CSM lights all illuminate the whole scene, so divide the sun
      // intensity across cascades to avoid N-times over-lighting.
      lightIntensity: opts.intensity / this.cascades,
    });
    // Warm the cascade lights to the C sun color (CSM defaults to white).
    for (const light of this.csm.lights) {
      (light as THREE.DirectionalLight).color.setHex(SUN_COLOR);
      light.shadow.bias = -0.0004; // slope-scaled bias analogue
      light.shadow.normalBias = 0.02;
    }
  }

  setIntensity(intensity: number): void {
    this.csm.lightIntensity = intensity / this.cascades;
    for (const light of this.csm.lights) {
      (light as THREE.DirectionalLight).intensity = intensity / this.cascades;
    }
  }

  /** Register a material for cascaded shadows exactly once. */
  registerMaterial(mat: THREE.Material): void {
    if (this.registered.has(mat)) return;
    this.registered.add(mat);
    // The grid material already owns onBeforeCompile; chain CSM then re-apply
    // the grid injection so both shader patches land.
    if ((mat as THREE.MeshStandardMaterial).userData?.isBox3dGrid) {
      const cell = (mat as THREE.MeshStandardMaterial).userData.gridCellSize as number;
      this.csm.setupMaterial(mat);
      const csmHook = mat.onBeforeCompile;
      mat.onBeforeCompile = (shader, renderer) => {
        csmHook.call(mat, shader, renderer);
        gridInjectShader(shader, cell);
      };
    } else {
      this.csm.setupMaterial(mat);
    }
    mat.needsUpdate = true;
  }

  /** Walk a subtree registering every standard material for shadows. */
  registerSubtree(root: THREE.Object3D): void {
    root.traverse((obj) => {
      const mesh = obj as THREE.Mesh;
      const mat = mesh.material;
      if (!mat) return;
      if (Array.isArray(mat)) {
        for (const m of mat) if (isStandard(m)) this.registerMaterial(m);
      } else if (isStandard(mat)) {
        this.registerMaterial(mat);
      }
    });
  }

  update(): void {
    this.csm.update();
  }

  updateFrustums(): void {
    this.csm.updateFrustums();
  }

  dispose(): void {
    this.csm.remove();
    this.csm.dispose();
  }
}

function isStandard(m: THREE.Material): m is THREE.MeshStandardMaterial {
  return (m as THREE.MeshStandardMaterial).isMeshStandardMaterial === true;
}
