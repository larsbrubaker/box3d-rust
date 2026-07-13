// Preetham sky + image-based lighting, mirroring the C samples renderer.
//
// C renders a Preetham sky (samples/shaders/sky.glsl via samples/gfx/sky.c) as
// the scene background AND bakes it into an irradiance/prefilter environment
// for ambient IBL (samples/gfx/ibl.c, RebuildImageBasedLightingIfDirty). Three's
// jsm Sky addon is the same Preetham model, so we drive both the visible
// background and a PMREM environment map from one Sky instance.

import * as THREE from "three";
import { Sky } from "three/addons/objects/Sky.js";
import { SUN_DIR, SKY_TURBIDITY } from "./constants";

export interface SkyEnvironment {
  /** Sky dome mesh; add to scene.background-equivalent by adding to the scene. */
  readonly sky: Sky;
  /** PMREM-filtered environment texture for scene.environment (IBL ambient). */
  readonly envTexture: THREE.Texture;
  dispose(): void;
}

/**
 * Build the Preetham sky dome and its PMREM environment map.
 *
 * Sun direction and turbidity come from the C renderer defaults
 * (renderer.c:1232 dirToSun, renderer.c:1259 turbidity). rayleigh/mie are the
 * Sky addon's physically-plausible defaults tuned for a clear daytime sky, the
 * regime C's turbidity=2.2 ("Spec turbidity") targets.
 */
export function setupSkyEnvironment(renderer: THREE.WebGLRenderer): SkyEnvironment {
  const sky = new Sky();
  // Large enough to always contain the camera; the Sky shader forces depth to
  // the far plane so the actual scale only needs to enclose the eye.
  sky.scale.setScalar(10000);

  const u = sky.material.uniforms;
  u.turbidity.value = SKY_TURBIDITY; // renderer.c:1259
  u.rayleigh.value = 1.2;
  u.mieCoefficient.value = 0.005;
  u.mieDirectionalG.value = 0.8;
  // sunPosition is a direction; Sky normalizes it. Use the C dir-to-sun.
  u.sunPosition.value.copy(SUN_DIR);

  // Bake the sky into a PMREM env map for ambient IBL. Render a throwaway scene
  // holding an identically-configured Sky so we never remove the visible dome.
  const pmrem = new THREE.PMREMGenerator(renderer);
  pmrem.compileEquirectangularShader();

  const envScene = new THREE.Scene();
  const envSky = new Sky();
  envSky.scale.setScalar(10000);
  const eu = envSky.material.uniforms;
  eu.turbidity.value = u.turbidity.value;
  eu.rayleigh.value = u.rayleigh.value;
  eu.mieCoefficient.value = u.mieCoefficient.value;
  eu.mieDirectionalG.value = u.mieDirectionalG.value;
  eu.sunPosition.value.copy(SUN_DIR);
  envScene.add(envSky);

  const envRT = pmrem.fromScene(envScene, 0.04);
  const envTexture = envRT.texture;

  // The render target/scene are only needed for the one-shot bake.
  envSky.geometry.dispose();
  (envSky.material as THREE.Material).dispose();
  pmrem.dispose();

  return {
    sky,
    envTexture,
    dispose() {
      sky.geometry.dispose();
      (sky.material as THREE.Material).dispose();
      envRT.dispose();
    },
  };
}
