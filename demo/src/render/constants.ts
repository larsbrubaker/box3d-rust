// Rendering constants ported from Erin Catto's Box3D samples app so the
// Three.js demo matches the C reference "look". Every value here is traced to
// a specific line in box3d-cpp-reference/samples so a reader can diff them.

import * as THREE from "three";

/**
 * World-space direction TO the sun.
 * C: renderer.c:1232 `s_gfx.sun.dirToSun = b3Normalize((b3Vec3){0.5, 0.8, 0.4})`.
 * The demos run Y-up (zUp=false), which matches Three's world, so the vector
 * carries over unchanged. normalize(0.5,0.8,0.4) ≈ (0.4879, 0.7807, 0.3904).
 */
export const SUN_DIR = new THREE.Vector3(0.5, 0.8, 0.4).normalize();

/**
 * Sun / key-light color.
 * C: renderer.h:62 "color (1.0, 0.95, 0.85)".
 */
export const SUN_COLOR = 0xfff2d9; // (1.0, 0.95, 0.85) in sRGB hex

/**
 * Flat-ambient fallback strength (used when IBL is disabled).
 * C: renderer.c:1245 `s_gfx.sun.ambient = 0.10f`.
 */
export const SUN_AMBIENT = 0.1;

/**
 * Preetham turbidity, single source of truth for sky + IBL.
 * C: renderer.c:1259 `s_gfx.turbidity = 2.2f` ("Spec turbidity.").
 */
export const SKY_TURBIDITY = 2.2;

/**
 * AgX exposure default, in EV stops. C sets exposureEv = -2.5 (renderer.c:1249)
 * because physically-scaled Preetham radiance is very bright and needs pulling
 * down. Three's Sky addon emits far tamer values and Three's AgXToneMapping is
 * not Sobotka Minimal AgX, so batch-2 spec fixes the default at 0 stops and
 * lets the Render menu adjust from there. toneMappingExposure = 2^stops.
 */
export const EXPOSURE_STOPS_DEFAULT = 0;

// AgX-look saturation. C: renderer.c:1253 `s_gfx.tonemapSaturation = 1.4f`.
// Three's built-in AgXToneMapping has no saturation knob, so there is nothing to
// feed this into — recorded here as a plain comment (not an exported symbol) so
// the C value stays documented without a dead export (see three-scene.ts
// tone-map comment).

/** Ground grid cell size. C: debug_adapter.h:17 `BOX3D_GROUND_GRID_CELL_SIZE 1.0f`. */
export const GROUND_GRID_CELL_SIZE = 1.0;

/**
 * Per-b3BodyType PBR roughness/metallic defaults.
 * C: debug_adapter.c:139-140
 *   kBodyTypeMetallic  = {0.0, 0.0, 0.0}
 *   kBodyTypeRoughness = {0.70, 0.55, 0.40}   // static, kinematic, dynamic
 */
export const BODY_TYPE_ROUGHNESS = [0.7, 0.55, 0.4] as const;
export const BODY_TYPE_METALLIC = [0.0, 0.0, 0.0] as const;

/**
 * b3DebugMaterial presets, indexed [default, matte, soft, dead, glossy, metal].
 * C: debug_adapter.c:145-146
 *   kDebugMaterialMetallic  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.85}
 *   kDebugMaterialRoughness = {0.50, 0.85, 0.65, 0.95, 0.30, 0.35}
 */
export const MATERIAL_PRESET_ROUGHNESS = [0.5, 0.85, 0.65, 0.95, 0.3, 0.35] as const;
export const MATERIAL_PRESET_METALLIC = [0.0, 0.0, 0.0, 0.0, 0.0, 0.85] as const;

/** Camera projection defaults. C: main.cpp:88-89 fov 50°, near 0.1, far kViewDistance. */
export const CAMERA_FOV_DEG = 50; // main.cpp:88 SetFov(50°) overrides camera.cpp:78 default 60°
export const CAMERA_NEAR = 0.1; // main.cpp:89 SetClip near
export const CAMERA_FAR = 1000; // camera.h:50 kViewDistance = 1000.0f
