// Shared Three.js scene helper for demo SPA routes — Samples App look:
// muted sky, soft shadows, grid floor, flat-ish materials, C debug body colors.

import * as THREE from "three";
import { CameraControls } from "./render/camera-controls";
import { EffectComposer } from "three/addons/postprocessing/EffectComposer.js";
import { RenderPass } from "three/addons/postprocessing/RenderPass.js";
import { OutputPass } from "three/addons/postprocessing/OutputPass.js";
import { GTAOPass } from "three/addons/postprocessing/GTAOPass.js";
import {
  SUN_DIR,
  SUN_COLOR,
  SUN_AMBIENT,
  BODY_TYPE_ROUGHNESS,
  BODY_TYPE_METALLIC,
  MATERIAL_PRESET_ROUGHNESS,
  MATERIAL_PRESET_METALLIC,
  CAMERA_FOV_DEG,
  CAMERA_NEAR,
  CAMERA_FAR,
} from "./render/constants";
import { setupSkyEnvironment, type SkyEnvironment } from "./render/environment";
import { makeGroundGrid } from "./render/grid";
import { CsmManager } from "./render/shadows";
// Render-settings state lives in a three-free leaf so main.ts can drive it
// without importing this module (see render-settings.ts for the why).
import {
  type RenderSettings,
  getRenderSettings,
  setActiveScene,
  clearActiveScene,
} from "./render-settings.ts";
// Re-export the render-settings surface so existing `three-scene.ts` importers
// (and this module's own users) keep a single, unchanged entry point.
export {
  type RenderSettings,
  applyRenderSettings,
  getRenderSettings,
} from "./render-settings.ts";

/** Base directional-light intensity (sun). Tuned against AgX + Preetham IBL. */
const SUN_INTENSITY = 2.6;

/**
 * Base tone-map exposure at 0 EV stops. Three's Sky addon and PMREM env emit
 * radiance in a different magnitude than C's physically-scaled Preetham, so the
 * exposure that reads correctly is not 1.0. C compensates with exposureEv=-2.5
 * (renderer.c:1249); here we fold an equivalent baseline into the scene so the
 * *stops* semantics stay literal: toneMappingExposure = BASE_EXPOSURE * 2^stops,
 * i.e. each +1 stop still doubles brightness, and 0 stops is the calibrated look.
 */
const BASE_EXPOSURE = 0.42;

/** IBL ambient strength (scene.environmentIntensity) when IBL is enabled. */
const IBL_INTENSITY = 0.55;

export const COLORS = {
  accent: 0x2563eb,
  hit: 0xdc2626,
  good: 0x15803d,
  shape: 0x5a6170,
  muted: 0x8b92a0,
  /** Muted grey/blue sky matching the C samples soft environment. */
  sky: 0x6b7a8f,
  bg: 0x6b7a8f,
} as const;

/**
 * Per-mesh material for the engine-driven style path. Each dynamics mesh owns
 * one; `applyShapeStyle` writes color / roughness / metalness / opacity into it
 * from the packed style word emitted by the Rust `*_styles()` exports.
 */
export function makeShapeMaterial(): THREE.MeshStandardMaterial {
  return new THREE.MeshStandardMaterial({
    color: 0xffffff,
    roughness: 0.5,
    metalness: 0,
    flatShading: true,
    side: THREE.DoubleSide,
  });
}

/**
 * Apply one packed engine style word (from a Rust `*_styles()` export) to a
 * mesh's material, reproducing the C debug adapter's material resolution
 * (`debug_adapter.c` DrawShape :832-861):
 *
 * ```text
 *  bits 23..0   0xRRGGBB base color (sRGB)
 *  bits 26..24  b3DebugMaterial preset (0 default … 5 metal)
 *  bits 28..27  body type (0 static, 1 kinematic, 2 dynamic)
 *  bit  29      transparent (alpha 0.5)
 * ```
 *
 * Engine colors change per frame (sleep/wake, fast, bullet, speed-capped), so
 * this is the shared "apply on change" primitive: it caches the last word on
 * `mesh.userData.styleWord` and no-ops when the word is unchanged, so callers
 * can simply call it every frame with `styles[i]`. Each mesh must own its own
 * material (colors differ per body) — pair with `makeShapeMaterial`.
 */
export function applyShapeStyle(mesh: THREE.Mesh, style: number): void {
  if (mesh.userData.styleWord === style) return;
  mesh.userData.styleWord = style;

  const mat = mesh.material as THREE.MeshStandardMaterial;
  const rgb = style & 0xffffff;
  const preset = (style >>> 24) & 0x7;
  const bodyType = (style >>> 27) & 0x3;
  const transparent = (style >>> 29) & 0x1;

  // Base color is sRGB; the renderer's SRGB output space converts to linear.
  mat.color.setHex(rgb);
  let roughness: number;
  let metalness: number;
  if (preset >= 1 && preset <= 5) {
    roughness = MATERIAL_PRESET_ROUGHNESS[preset]!;
    metalness = MATERIAL_PRESET_METALLIC[preset]!;
  } else {
    roughness = BODY_TYPE_ROUGHNESS[bodyType]!;
    metalness = BODY_TYPE_METALLIC[bodyType]!;
  }
  mat.roughness = roughness;
  mat.metalness = metalness;

  const wantTransparent = transparent === 1;
  if (mat.transparent !== wantTransparent) {
    mat.transparent = wantTransparent;
    mat.needsUpdate = true; // toggling transparency recompiles the material
  }
  mat.opacity = wantTransparent ? 0.5 : 1.0;
}

/**
 * Apply packed engine style words to an `InstancedMesh`, one color per instance,
 * via Three's `instanceColor` attribute (`mesh.setColorAt`). This is the instanced
 * analogue of {@link applyShapeStyle} and exists so the Benchmark piles (Large
 * Pyramid, Falling Boxes, …) show per-body sleep/wake recoloring while still
 * rendering as a single draw call.
 *
 * **Compromise (documented):** an `InstancedMesh` shares ONE material, so only the
 * 0xRRGGBB *color* can vary per instance (`instanceColor`). The PBR *material
 * preset* (roughness / metalness / transparency) is resolved once, from the
 * `representative`-th style word, and written to the shared material — every
 * instance therefore shares one preset. In the Benchmark scenes that use this,
 * all instanced bodies are dynamic boxes/cylinders of the same body type, so the
 * preset is identical across instances and the compromise is invisible; only the
 * per-body awake/asleep *color* differs, which is exactly what `instanceColor`
 * carries. The shared material's own `color` is forced to white so `instanceColor`
 * is the final albedo (Three multiplies `material.color * instanceColor`).
 *
 * Colors are decoded from the same style layout as {@link applyShapeStyle} (sRGB
 * 0xRRGGBB in bits 0..24) and converted to the renderer's linear working space via
 * `THREE.Color`, matching `applyShapeStyle`'s `mat.color.setHex` semantics.
 */
export function applyInstancedStyles(
  mesh: THREE.InstancedMesh,
  styles: ArrayLike<number>,
  count: number,
  representative = 0,
  colorsChanged = true,
): void {
  // Resolve the shared material preset from one representative style word.
  const rep = styles[representative] ?? 0;
  const mat = mesh.material as THREE.MeshStandardMaterial;
  const preset = (rep >>> 24) & 0x7;
  const bodyType = (rep >>> 27) & 0x3;
  const transparent = (rep >>> 29) & 0x1;
  let roughness: number;
  let metalness: number;
  if (preset >= 1 && preset <= 5) {
    roughness = MATERIAL_PRESET_ROUGHNESS[preset]!;
    metalness = MATERIAL_PRESET_METALLIC[preset]!;
  } else {
    roughness = BODY_TYPE_ROUGHNESS[bodyType]!;
    metalness = BODY_TYPE_METALLIC[bodyType]!;
  }
  if (mat.roughness !== roughness) mat.roughness = roughness;
  if (mat.metalness !== metalness) mat.metalness = metalness;
  // instanceColor is the albedo; keep the shared material color white so it does
  // not tint the per-instance colors.
  if (mat.color.getHex() !== 0xffffff) mat.color.setHex(0xffffff);
  const wantTransparent = transparent === 1;
  if (mat.transparent !== wantTransparent) {
    mat.transparent = wantTransparent;
    mat.opacity = wantTransparent ? 0.5 : 1.0;
    mat.needsUpdate = true;
  }

  // Per-instance color. setColorAt lazily allocates mesh.instanceColor. When the
  // caller's style buffer is byte-for-byte unchanged (a settled/asleep pile whose
  // awake-gated `*_styles()` snapshot did not refetch), the per-instance colors
  // already live in `instanceColor`, so skip the whole write + GPU upload. A newly
  // (re)allocated mesh has no `instanceColor` yet and must be filled regardless.
  if (!colorsChanged && mesh.instanceColor) return;
  for (let i = 0; i < count; i++) {
    const rgb = (styles[i] ?? 0) & 0xffffff;
    _styleColor.setHex(rgb, THREE.SRGBColorSpace);
    mesh.setColorAt(i, _styleColor);
  }
  if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
}

const _styleColor = new THREE.Color();
const _yUp = new THREE.Vector3(0, 1, 0);
const _tmp = new THREE.Vector3();
const _tmp2 = new THREE.Vector3();

/**
 * Prefer cascaded shadow maps (CSM) over a single directional shadow. CSM
 * matches C's 3-cascade setup; the single-light path is the robust fallback if
 * CSM ever misbehaves with instanced meshes. Flip to false to force the
 * fallback everywhere.
 */
const PREFER_CSM = true;

export class DemoScene {
  readonly scene: THREE.Scene;
  readonly camera: THREE.PerspectiveCamera;
  readonly renderer: THREE.WebGLRenderer;
  readonly controls: CameraControls;
  /** Cleared each frame for dynamic overlays (rays, hits, contacts). */
  readonly dynamic: THREE.Group;
  /** Persistent content (shapes that change infrequently). */
  readonly content: THREE.Group;
  /** The sun key light (also the plain-shadow caster when CSM is off). */
  readonly keyLight: THREE.DirectionalLight;

  private readonly canvas: HTMLCanvasElement;
  private readonly ro: ResizeObserver;
  private disposed = false;

  private readonly ambient: THREE.AmbientLight;
  private readonly ground: THREE.Mesh;
  private shadowExtent: number;

  private readonly env: SkyEnvironment;
  private composer: EffectComposer | null = null;
  private readonly renderPass: RenderPass;
  private readonly gtaoPass: GTAOPass;
  private csm: CsmManager | null = null;

  // CSM material registration is add-time driven, not per-frame. The `content`
  // and `dynamic` groups' `add` is wrapped (see hookCsmDirty) to raise these
  // flags whenever a demo inserts a mesh; render() re-walks a subtree only when
  // its flag is set (and once when a fresh CsmManager is created). Split per group
  // so a demo that rebuilds `dynamic` overlays every frame doesn't force a
  // needless re-walk of stable `content`.
  private contentCsmDirty = true;
  private dynamicCsmDirty = true;

  // Cached debug-view override materials (created once, reused across switches)
  // so applyDebugView never leaks a MeshDepth/MeshNormalMaterial per settings
  // change. Disposed in dispose().
  private depthMaterial: THREE.MeshDepthMaterial | null = null;
  private normalMaterial: THREE.MeshNormalMaterial | null = null;

  private settings: RenderSettings = getRenderSettings();

  constructor(
    canvas: HTMLCanvasElement,
    opts: {
      target?: [number, number, number];
      distance?: number;
      fov?: number;
      /** Shadow camera half-extent (default 28). */
      shadowExtent?: number;
      /** Ground-grid plane size (default 200). */
      gridSize?: number;
      /** Unused now (grid is a shader plane); kept for API compatibility. */
      gridDivisions?: number;
    } = {},
  ) {
    this.canvas = canvas;
    this.scene = new THREE.Scene();

    // Camera projection per C main.cpp:88-89 (fov 50°, near 0.1, far 1000).
    this.camera = new THREE.PerspectiveCamera(
      opts.fov ?? CAMERA_FOV_DEG,
      1,
      CAMERA_NEAR,
      CAMERA_FAR,
    );
    const dist = opts.distance ?? 12;

    this.renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: true,
      alpha: false,
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    // Tone mapping = AgX. C uses Sobotka "Minimal AgX" (samples/shaders/post/
    // tonemap.glsl) with exposure in EV stops and a saturation knob (1.4). Three
    // ships THREE.AgXToneMapping, the closest built-in — not bit-identical to
    // Minimal AgX and with no saturation control, but the same filmic family.
    // Exposure semantics: toneMappingExposure = 2^stops (see applySettings).
    this.renderer.toneMapping = THREE.AgXToneMapping;
    this.renderer.toneMappingExposure = BASE_EXPOSURE * Math.pow(2, this.settings.exposureStops);

    // Preetham sky (background dome) + PMREM environment for IBL ambient.
    this.env = setupSkyEnvironment(this.renderer);
    this.env.sky.frustumCulled = false;
    this.scene.add(this.env.sky);

    // Faithful port of the C sample camera (orbit + fly), replacing OrbitControls.
    // The controller's own defaults are the C camera's (yaw 35°, pitch -25°,
    // radius 25; camera.cpp:73-102). The constructor frames from opts.distance/
    // target using those same C defaults (yaw 35°, pitch -25°) so any future
    // sample that never calls setView inherits the C-correct camera. The handful
    // of batch-2 collision demos that want today's flatter over-the-shoulder tilt
    // call setView(demo, 35, 20, …) explicitly; the dynamics demos call setView in
    // their reset(). No smooth damping — C has none.
    this.controls = new CameraControls(this.camera, canvas);
    this.controls.setView(35, -25, dist, opts.target ?? [0, 0, 0]);

    this.ambient = new THREE.AmbientLight(SUN_COLOR, SUN_AMBIENT);

    const key = new THREE.DirectionalLight(SUN_COLOR, SUN_INTENSITY);
    key.position.copy(SUN_DIR).multiplyScalar(40);
    key.target.position.set(0, 0, 0);
    const extent = opts.shadowExtent ?? 28;
    this.shadowExtent = extent;
    key.shadow.mapSize.set(2048, 2048);
    key.shadow.bias = -0.0004;
    key.shadow.normalBias = 0.02;
    key.shadow.camera.near = 0.5;
    key.shadow.camera.far = extent * 4;
    key.shadow.camera.left = -extent;
    key.shadow.camera.right = extent;
    key.shadow.camera.top = extent;
    key.shadow.camera.bottom = -extent;
    this.keyLight = key;
    this.scene.add(this.ambient, key, key.target);

    // Shader-based ground grid (geom.glsl ground mode) replacing the old lines.
    this.ground = makeGroundGrid((opts.gridSize ?? 200), undefined, 0xa9a9a9);
    this.scene.add(this.ground);

    this.content = new THREE.Group();
    this.dynamic = new THREE.Group();
    // Raise the CSM dirty flag on insertion instead of walking every frame.
    this.hookCsmDirty(this.content, () => {
      this.contentCsmDirty = true;
    });
    this.hookCsmDirty(this.dynamic, () => {
      this.dynamicCsmDirty = true;
    });
    this.scene.add(this.content, this.dynamic);

    // Post-processing chain: scene (offscreen, linear HDR) -> GTAO -> OutputPass
    // (applies AgX tone map + sRGB). RenderPass renders offscreen, so Three
    // skips per-material tone mapping there and OutputPass applies it once.
    this.renderPass = new RenderPass(this.scene, this.camera);
    this.gtaoPass = new GTAOPass(this.scene, this.camera, 1, 1);
    try {
      const composer = new EffectComposer(this.renderer);
      composer.addPass(this.renderPass);
      composer.addPass(this.gtaoPass);
      composer.addPass(new OutputPass());
      this.composer = composer;
    } catch {
      // Fall back to direct rendering (still AgX via renderer.toneMapping).
      this.composer = null;
    }

    this.ro = new ResizeObserver(() => this.resize());
    this.ro.observe(canvas.parentElement ?? canvas);
    this.resize();

    setActiveScene(this);
    this.applySettings(getRenderSettings());
  }

  /**
   * Wrap a group's `add` so inserting a child raises the given CSM dirty flag.
   * Keeps CSM material registration add-time driven: render() re-walks the
   * subtree only when something actually changed, not once per frame.
   */
  private hookCsmDirty(group: THREE.Group, mark: () => void): void {
    const origAdd = group.add.bind(group);
    group.add = ((...objs: THREE.Object3D[]) => {
      mark();
      return origAdd(...objs);
    }) as THREE.Group["add"];
  }

  /** Apply the shared render settings to this scene's pipeline. */
  applySettings(s: RenderSettings) {
    if (this.disposed) return;
    this.settings = { ...s };

    // Exposure: EV stops -> linear multiplier (relative to the calibrated base).
    this.renderer.toneMappingExposure = BASE_EXPOSURE * Math.pow(2, s.exposureStops);

    // IBL ambient: env map when on, small flat ambient fills in when off.
    this.scene.environment = s.ibl ? this.env.envTexture : null;
    this.scene.environmentIntensity = s.ibl ? IBL_INTENSITY * s.sunStrength : 0;
    this.ambient.intensity = s.ibl ? SUN_AMBIENT : 0.5;

    // Shadows.
    this.updateShadowSystem(s.shadows);
    // Sun strength scales the active sun light(s).
    if (this.csm) this.csm.setIntensity(SUN_INTENSITY * s.sunStrength);
    else this.keyLight.intensity = SUN_INTENSITY * s.sunStrength;

    // GTAO.
    this.gtaoPass.enabled = s.gtao && this.composer !== null;
    this.applyGtaoQuality(s.gtaoQuality);

    // Debug view outputs.
    this.applyDebugView(s.debugView);
  }

  /**
   * Resize the directional shadow-camera frustum (world units). Multi-scene
   * pages that share one DemoScene across scenes with very different footprints
   * (e.g. World's 80-unit Far Pyramid vs the 60-unit far scenes) call this on
   * scene switch so shadows stay tight without re-instantiating the scene.
   */
  setShadowExtent(extent: number) {
    if (extent === this.shadowExtent) return;
    this.shadowExtent = extent;
    const cam = this.keyLight.shadow.camera;
    cam.near = 0.5;
    cam.far = extent * 4;
    cam.left = -extent;
    cam.right = extent;
    cam.top = extent;
    cam.bottom = -extent;
    cam.updateProjectionMatrix();
  }

  private updateShadowSystem(shadows: boolean) {
    this.renderer.shadowMap.enabled = shadows;
    if (shadows && PREFER_CSM) {
      if (!this.csm) {
        try {
          this.csm = new CsmManager({
            scene: this.scene,
            camera: this.camera,
            maxFar: Math.max(this.shadowExtent * 4, 120),
            intensity: SUN_INTENSITY * this.settings.sunStrength,
          });
          // A fresh CsmManager has an empty material set, so force a one-time
          // re-registration of the existing content/dynamic subtrees.
          this.contentCsmDirty = true;
          this.dynamicCsmDirty = true;
        } catch {
          this.csm = null;
        }
      }
      // CSM owns the sun; the key light stays as a dormant API handle.
      this.keyLight.castShadow = false;
      this.keyLight.intensity = this.csm ? 0 : SUN_INTENSITY * this.settings.sunStrength;
      if (!this.csm) this.keyLight.castShadow = true; // fallback if CSM failed
    } else {
      if (this.csm) {
        this.csm.dispose();
        this.csm = null;
      }
      this.keyLight.intensity = SUN_INTENSITY * this.settings.sunStrength;
      this.keyLight.castShadow = shadows;
    }
  }

  private applyGtaoQuality(q: 0 | 1 | 2) {
    // XeGTAO (C) has no direct Three equivalent; map quality to sample count +
    // radius. GTAOPass modulates the whole beauty, not just ambient (addon
    // limitation vs C which multiplies only IBL ambient) — softened via blend.
    const presets = [
      { samples: 16, radius: 0.5, distanceExponent: 1, thickness: 1, scale: 1 }, // 0 medium
      { samples: 16, radius: 0.35, distanceExponent: 1, thickness: 1, scale: 1 }, // 1 high
      { samples: 32, radius: 0.25, distanceExponent: 1, thickness: 1, scale: 1 }, // 2 ultra
    ] as const;
    const p = presets[q] ?? presets[0];
    this.gtaoPass.updateGtaoMaterial({ ...p });
    this.gtaoPass.blendIntensity = 0.9;
  }

  private applyDebugView(view: number) {
    // 0 = lit (required). 1 = depth, 3 = view-space normals, 4 = AO isolated.
    // 2 = cascade-index has no cheap Three equivalent, so it clamps to lit.
    this.scene.overrideMaterial = null;
    // Reset GTAO to composite unless AO is explicitly requested.
    this.gtaoPass.output = (GTAOPass as unknown as { OUTPUT: Record<string, number> }).OUTPUT
      .Default;
    switch (view) {
      case 1:
        // Reuse the cached override material (allocate on first use) instead of
        // constructing a new MeshDepthMaterial on every settings change.
        this.scene.overrideMaterial = this.depthMaterial ??= new THREE.MeshDepthMaterial();
        break;
      case 3:
        this.scene.overrideMaterial = this.normalMaterial ??= new THREE.MeshNormalMaterial();
        break;
      case 4:
        this.gtaoPass.enabled = true;
        this.gtaoPass.output = (
          GTAOPass as unknown as { OUTPUT: Record<string, number> }
        ).OUTPUT.AO;
        break;
      default:
        break; // 0 and 2 -> lit
    }
  }

  resize() {
    if (this.disposed) return;
    const parent = this.canvas.parentElement ?? this.canvas;
    const w = Math.max(1, Math.round(parent.clientWidth));
    const h = Math.max(1, Math.round(parent.clientHeight));
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h, false);
    const dpr = this.renderer.getPixelRatio();
    this.composer?.setSize(w * dpr, h * dpr);
    this.csm?.updateFrustums();
  }

  clearGroup(group: THREE.Group) {
    while (group.children.length > 0) {
      const obj = group.children[0]!;
      group.remove(obj);
      disposeObject(obj);
    }
  }

  clearContent() {
    this.clearGroup(this.content);
  }

  clearDynamic() {
    this.clearGroup(this.dynamic);
  }

  render() {
    if (this.disposed) return;
    this.controls.update();
    if (this.csm) {
      // Register newly-added materials for cascaded shadows only when a subtree
      // actually changed (add-time dirty flags), not every frame.
      if (this.contentCsmDirty) {
        this.csm.registerSubtree(this.content);
        this.csm.registerMaterial(this.ground.material as THREE.Material);
        this.contentCsmDirty = false;
      }
      if (this.dynamicCsmDirty) {
        this.csm.registerSubtree(this.dynamic);
        this.dynamicCsmDirty = false;
      }
      this.csm.update();
    }
    if (this.composer) this.composer.render();
    else this.renderer.render(this.scene, this.camera);
  }

  dispose() {
    if (this.disposed) return;
    this.disposed = true;
    clearActiveScene(this);
    this.ro.disconnect();
    this.controls.dispose();
    this.clearContent();
    this.clearDynamic();
    this.csm?.dispose();
    this.scene.remove(this.ground);
    disposeObject(this.ground);
    this.scene.remove(this.env.sky);
    this.env.dispose();
    this.gtaoPass.dispose();
    this.composer?.dispose();
    // Detach and dispose the cached debug-view override materials (they live
    // outside the scene graph, so the traversal below won't reach them).
    this.scene.overrideMaterial = null;
    this.depthMaterial?.dispose();
    this.normalMaterial?.dispose();
    this.scene.traverse((obj) => {
      if (obj !== this.content && obj !== this.dynamic) disposeObject(obj);
    });
    this.renderer.dispose();
    this.renderer.forceContextLoss();
  }
}

function disposeObject(obj: THREE.Object3D) {
  obj.traverse((child) => {
    const mesh = child as THREE.Mesh;
    if (mesh.geometry) mesh.geometry.dispose();
    const mat = mesh.material;
    if (Array.isArray(mat)) mat.forEach((m) => m.dispose());
    else if (mat) mat.dispose();
  });
}

/// C `camera::SetView(yaw, pitch, distance, target)` (camera.cpp:195) — routes to
/// the orbit/fly camera controller so every demo call site keeps working unchanged.
/// eye = target + distance · ForwardFromAngles(yaw, pitch); angles in degrees.
export function setView(
  demo: DemoScene,
  yawDeg: number,
  pitchDeg: number,
  distance: number,
  target: [number, number, number],
) {
  demo.controls.setView(yawDeg, pitchDeg, distance, target);
}

export function makeAxes(len = 1.5): THREE.AxesHelper {
  return new THREE.AxesHelper(len);
}

export function solidMat(color: number, opacity = 0.85): THREE.MeshStandardMaterial {
  return new THREE.MeshStandardMaterial({
    color,
    transparent: opacity < 1,
    opacity,
    roughness: 0.78,
    metalness: 0.04,
    flatShading: true,
    side: THREE.DoubleSide,
  });
}

export function lineMat(color: number, opacity = 1): THREE.LineBasicMaterial {
  return new THREE.LineBasicMaterial({
    color,
    transparent: opacity < 1,
    opacity,
    depthTest: true,
  });
}

export function makeSphere(
  cx: number,
  cy: number,
  cz: number,
  r: number,
  color: number,
  opacity = 0.75,
): THREE.Mesh {
  const mesh = new THREE.Mesh(
    new THREE.SphereGeometry(r, 24, 16),
    solidMat(color, opacity),
  );
  mesh.position.set(cx, cy, cz);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

/** Capsule between two centers (sphere ends included in length). */
export function makeCapsule(
  c1: [number, number, number],
  c2: [number, number, number],
  radius: number,
  color: number,
  opacity = 0.75,
): THREE.Mesh {
  const a = _tmp.set(c1[0], c1[1], c1[2]);
  const b = _tmp2.set(c2[0], c2[1], c2[2]);
  const dir = new THREE.Vector3().subVectors(b, a);
  const len = dir.length();
  const cyl = Math.max(1e-4, len);
  const mesh = new THREE.Mesh(
    new THREE.CapsuleGeometry(radius, cyl, 6, 12),
    solidMat(color, opacity),
  );
  mesh.position.copy(a).add(b).multiplyScalar(0.5);
  if (len > 1e-6) {
    mesh.quaternion.setFromUnitVectors(_yUp, dir.normalize());
  }
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

export function makeWireBox(
  cx: number,
  cy: number,
  cz: number,
  hx: number,
  hy: number,
  hz: number,
  color: number,
): THREE.LineSegments {
  const geo = new THREE.BoxGeometry(hx * 2, hy * 2, hz * 2);
  const edges = new THREE.EdgesGeometry(geo);
  geo.dispose();
  const lines = new THREE.LineSegments(edges, lineMat(color));
  lines.position.set(cx, cy, cz);
  return lines;
}

export function makeSolidBox(
  cx: number,
  cy: number,
  cz: number,
  hx: number,
  hy: number,
  hz: number,
  color: number,
  opacity = 0.35,
): THREE.Mesh {
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(hx * 2, hy * 2, hz * 2),
    solidMat(color, opacity),
  );
  mesh.position.set(cx, cy, cz);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

/**
 * Line segments from a flat `[ax,ay,az, bx,by,bz, ...]` edge buffer. `opacity`
 * feeds the shared {@link lineMat} (default fully opaque; ground/terrain overlays
 * pass a faint value like 0.35).
 *
 * NOTE: the third argument is the line *opacity*, not a buffer index. A previous
 * `offset` index parameter here silently produced all-NaN geometry when callers
 * passed a fractional opacity value (0.35 → `edges[0.35]` is `undefined` →
 * coerced to NaN), which then tripped Three's `computeBoundingSphere(): radius is
 * NaN` warning at render on every route with a faint ground wire. No caller ever
 * needed a real index offset, so the parameter now carries the opacity the call
 * sites always intended.
 */
export function makeWireEdges(
  edges: ArrayLike<number>,
  color: number,
  opacity = 1,
): THREE.LineSegments {
  const positions: number[] = [];
  for (let i = 0; i + 5 < edges.length; i += 6) {
    positions.push(
      edges[i]!,
      edges[i + 1]!,
      edges[i + 2]!,
      edges[i + 3]!,
      edges[i + 4]!,
      edges[i + 5]!,
    );
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  return new THREE.LineSegments(geo, lineMat(color, opacity));
}

/** Reconstruct triangle vertex positions from wireframe edge triplets (3 edges × 6 floats). */
export function trianglesFromWireframe(wire: ArrayLike<number>): Float32Array {
  const out: number[] = [];
  for (let i = 0; i + 17 < wire.length; i += 18) {
    out.push(wire[i]!, wire[i + 1]!, wire[i + 2]!);
    out.push(wire[i + 3]!, wire[i + 4]!, wire[i + 5]!);
    out.push(wire[i + 9]!, wire[i + 10]!, wire[i + 11]!);
  }
  return new Float32Array(out);
}

export function makeTriangleMesh(
  positions: Float32Array,
  color: number,
  opacity = 0.8,
  wireframe = false,
): THREE.Mesh {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geo.computeVertexNormals();
  const mat = solidMat(color, opacity);
  mat.wireframe = wireframe;
  const mesh = new THREE.Mesh(geo, mat);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

export function makeDot(
  p: [number, number, number],
  color: number,
  r = 0.08,
): THREE.Mesh {
  const mesh = new THREE.Mesh(
    new THREE.SphereGeometry(r, 12, 8),
    new THREE.MeshBasicMaterial({ color }),
  );
  mesh.position.set(p[0], p[1], p[2]);
  return mesh;
}

export function makeSegment(
  a: [number, number, number],
  b: [number, number, number],
  color: number,
): THREE.Line {
  const geo = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(a[0], a[1], a[2]),
    new THREE.Vector3(b[0], b[1], b[2]),
  ]);
  return new THREE.Line(geo, lineMat(color));
}

export function makeArrow(
  from: [number, number, number],
  dir: [number, number, number],
  length: number,
  color: number,
): THREE.ArrowHelper {
  const d = new THREE.Vector3(dir[0], dir[1], dir[2]);
  if (d.lengthSq() < 1e-12) d.set(0, 1, 0);
  else d.normalize();
  return new THREE.ArrowHelper(
    d,
    new THREE.Vector3(from[0], from[1], from[2]),
    length,
    color,
    length * 0.28,
    length * 0.16,
  );
}

export function makeDashedSegment(
  a: [number, number, number],
  b: [number, number, number],
  color: number,
): THREE.Line {
  const geo = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(a[0], a[1], a[2]),
    new THREE.Vector3(b[0], b[1], b[2]),
  ]);
  const mat = new THREE.LineDashedMaterial({
    color,
    dashSize: 0.12,
    gapSize: 0.08,
  });
  const line = new THREE.Line(geo, mat);
  line.computeLineDistances();
  return line;
}
