// Fat (thick) line material for the debug-draw overlay. Plain THREE.Line ignores
// linewidth on most platforms, so the C debug-draw overlay (default 1.5px lines,
// samples draw thin AA edges) needs the LineMaterial shader path to get real
// thickness.
//
// The debug-draw overlay (`DebugDrawOverlay` in interaction.ts) consumes this
// via `makeFatLineMaterial({ vertexColors: true })` + a `LineSegments2` built
// directly, and keeps `material.resolution` in sync with `updateFatLineResolution`
// each frame.

import * as THREE from "three";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";

export interface FatLineOptions {
  color?: THREE.ColorRepresentation;
  /**
   * Line thickness. When `worldUnits` is false (default) this is in *pixels*
   * (C debug default 1.5px). When true, it is in world units.
   */
  thickness?: number;
  worldUnits?: boolean;
  opacity?: number;
  /** Renderer drawing-buffer size; required so px thickness resolves correctly. */
  resolution?: THREE.Vector2;
  /** Colour each vertex from the geometry's `instanceColor` attribute
   *  (`LineSegmentsGeometry.setColors`) instead of the flat `color`. */
  vertexColors?: boolean;
}

/**
 * Per-resize hook: update a fat-line material's `resolution` uniform to the
 * renderer's drawing-buffer size (px). LineMaterial converts pixel thickness to
 * clip space using this uniform, so pixel-width lines only stay correct if the
 * overlay calls this on every canvas resize (and once at creation). The debug
 * overlay drives it from `renderer.getDrawingBufferSize` each frame, which also
 * covers resizes without a separate observer.
 */
export function updateFatLineResolution(
  mat: LineMaterial,
  width: number,
  height: number,
): void {
  mat.resolution.set(width, height);
}

/** Build a LineMaterial from the shared options (px or world-unit thickness). */
export function makeFatLineMaterial(opts: FatLineOptions = {}): LineMaterial {
  const worldUnits = opts.worldUnits ?? false;
  const mat = new LineMaterial({
    color: new THREE.Color(opts.color ?? 0xffffff).getHex(),
    linewidth: opts.thickness ?? 1.5, // C default 1.5px
    worldUnits,
    vertexColors: opts.vertexColors ?? false,
    transparent: (opts.opacity ?? 1) < 1,
    opacity: opts.opacity ?? 1,
    alphaToCoverage: true,
  });
  // LineMaterial needs the drawing-buffer resolution to convert px -> clip.
  if (opts.resolution) mat.resolution.copy(opts.resolution);
  return mat;
}
