// Shader-based ground grid, porting the C samples "ground grid" material.
//
// C renders the floor not as lines but as a lit PBR surface whose baseColor is
// replaced by an fwidth-antialiased procedural grid (samples/shaders/shapes/
// geom.glsl:307-320, using proceduralGrid from samples/shaders/common/
// pbr.glsl:216-244). We reproduce that by injecting the same grid math into a
// MeshStandardMaterial's diffuseColor via onBeforeCompile, so the floor still
// receives sun light, shadows (incl. CSM), and IBL ambient exactly like the
// other shapes — matching how geom.glsl feeds the grid into brdf_evaluate.

import * as THREE from "three";
import { GROUND_GRID_CELL_SIZE } from "./constants";

/**
 * proceduralGrid, transcribed from pbr.glsl:216-244.
 * minor lines = base*0.70, major (10× cell) = base*0.35, red +X / blue +Z axes.
 */
const GRID_GLSL = /* glsl */ `
vec3 box3dProceduralGrid( vec2 line_xz, vec2 axis_xz, vec3 base_color, float cell_size ) {
  vec2 coord_minor = line_xz / cell_size;
  vec2 d_minor = max( fwidth( coord_minor ), vec2( 1.0e-6 ) );
  vec2 g_minor = abs( fract( coord_minor - 0.5 ) - 0.5 ) / d_minor;
  float line_minor = 1.0 - clamp( min( g_minor.x, g_minor.y ), 0.0, 1.0 );

  vec2 coord_major = line_xz / ( cell_size * 10.0 );
  vec2 d_major = max( fwidth( coord_major ), vec2( 1.0e-6 ) );
  vec2 g_major = abs( fract( coord_major - 0.5 ) - 0.5 ) / d_major;
  float line_major = 1.0 - clamp( min( g_major.x, g_major.y ), 0.0, 1.0 );

  vec3 minor_color = base_color * 0.70;
  vec3 major_color = base_color * 0.35;

  vec3 result = mix( base_color, minor_color, line_minor );
  result = mix( result, major_color, line_major );

  vec2 d_axis = max( fwidth( axis_xz ), vec2( 1.0e-6 ) );
  float axis_x = ( 1.0 - clamp( abs( axis_xz.y ) / d_axis.y, 0.0, 1.0 ) ) * step( 0.0, axis_xz.x );
  float axis_z = ( 1.0 - clamp( abs( axis_xz.x ) / d_axis.x, 0.0, 1.0 ) ) * step( 0.0, axis_xz.y );
  result = mix( result, vec3( 1.0, 0.0, 0.0 ), axis_x );
  result = mix( result, vec3( 0.0, 0.0, 1.0 ), axis_z );
  return result;
}
`;

/**
 * Inject the procedural-grid shader into a compiled MeshStandardMaterial shader
 * object. Exported so the CSM shadow path can chain this after its own
 * onBeforeCompile (both need to patch the same material's shaders).
 */
export function gridInjectShader(
  shader: { vertexShader: string; fragmentShader: string; uniforms: Record<string, THREE.IUniform> },
  cellSize: number,
): void {
  shader.uniforms.gridCellSize = { value: cellSize };
  shader.vertexShader = shader.vertexShader
    .replace("#include <common>", "#include <common>\nvarying vec3 vGridWorld;")
    .replace(
      "#include <begin_vertex>",
      "#include <begin_vertex>\nvGridWorld = ( modelMatrix * vec4( transformed, 1.0 ) ).xyz;",
    );
  shader.fragmentShader = shader.fragmentShader
    .replace(
      "#include <common>",
      `#include <common>\nvarying vec3 vGridWorld;\nuniform float gridCellSize;\n${GRID_GLSL}`,
    )
    .replace(
      "vec4 diffuseColor = vec4( diffuse, opacity );",
      "vec4 diffuseColor = vec4( diffuse, opacity );\n" +
        "diffuseColor.rgb = box3dProceduralGrid( vGridWorld.xz, vGridWorld.xz, diffuseColor.rgb, gridCellSize );",
    );
}

/** Create the ground grid material (a MeshStandardMaterial with grid injection). */
export function makeGridMaterial(
  baseColor = 0xa9a9a9, // C static-body color (DarkGray); the grid darkens it
  cellSize = GROUND_GRID_CELL_SIZE,
): THREE.MeshStandardMaterial {
  const mat = new THREE.MeshStandardMaterial({
    color: baseColor,
    roughness: 0.7, // static-body roughness (debug_adapter.c:140)
    metalness: 0.0,
  });
  mat.onBeforeCompile = (shader) => gridInjectShader(shader, cellSize);
  // Marker + cell size so the shadow path can recognize the grid material and
  // chain gridInjectShader onto CSM's onBeforeCompile.
  mat.userData.isBox3dGrid = true;
  mat.userData.gridCellSize = cellSize;
  // Distinguish this shader program from a plain MeshStandardMaterial so
  // Three doesn't share a cached program that lacks the grid injection.
  mat.customProgramCacheKey = () => `box3d-grid-${cellSize}`;
  return mat;
}

/**
 * Create the ground plane mesh with the procedural grid material.
 * Lies in the XZ plane at y=0, receiving shadows.
 */
export function makeGroundGrid(
  size = 200,
  cellSize = GROUND_GRID_CELL_SIZE,
  baseColor = 0xa9a9a9,
): THREE.Mesh {
  const geo = new THREE.PlaneGeometry(size, size);
  const mat = makeGridMaterial(baseColor, cellSize);
  const mesh = new THREE.Mesh(geo, mat);
  mesh.rotation.x = -Math.PI / 2; // XY plane -> XZ ground
  mesh.position.y = 0;
  mesh.receiveShadow = true;
  mesh.name = "box3d-ground-grid";
  return mesh;
}
