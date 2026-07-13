// Fat (thick) line utilities wrapping three/addons Line2 / LineSegments2 /
// LineMaterial. Plain THREE.Line ignores linewidth on most platforms, so the
// C debug-draw overlay (default 1.5px lines, samples draw thin AA edges) needs
// the LineMaterial shader path to get real thickness.
//
// This module only EXPORTS the util; wiring it into the debug-draw overlay is
// done later by the interaction/overlay agent (do not edit interaction.ts here).

import * as THREE from "three";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineSegments2 } from "three/addons/lines/LineSegments2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineSegmentsGeometry } from "three/addons/lines/LineSegmentsGeometry.js";
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
  dashed?: boolean;
}

/**
 * Per-resize hook: update a fat-line material's `resolution` uniform to the
 * renderer's drawing-buffer size (px). LineMaterial converts pixel thickness to
 * clip space using this uniform, so pixel-width lines only stay correct if the
 * overlay calls this on every canvas resize (and once at creation).
 *
 * NOTE: overlay adoption is PENDING. Wiring these fat lines into the debug-draw
 * overlay lives in interaction.ts, owned by the interaction/overlay agent; this
 * helper is exported and ready for that agent to call `dom.getBoundingClientRect`
 * (or `renderer.getDrawingBufferSize`) into it. Until then it is unused by design.
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
    dashed: opts.dashed ?? false,
    transparent: (opts.opacity ?? 1) < 1,
    opacity: opts.opacity ?? 1,
    alphaToCoverage: true,
  });
  // LineMaterial needs the drawing-buffer resolution to convert px -> clip.
  if (opts.resolution) mat.resolution.copy(opts.resolution);
  return mat;
}

/**
 * Polyline through a flat [x,y,z, x,y,z, ...] point list.
 * Update `material.resolution` on resize for correct pixel thickness.
 */
export function makeFatLine(points: ArrayLike<number>, opts: FatLineOptions = {}): Line2 {
  const geo = new LineGeometry();
  geo.setPositions(Array.from(points));
  const mat = makeFatLineMaterial(opts);
  const line = new Line2(geo, mat);
  line.computeLineDistances();
  return line;
}

/**
 * Disjoint segments: pairs of endpoints in a flat [ax,ay,az, bx,by,bz, ...]
 * list (every 6 floats is one segment), matching the debug-draw edge layout.
 */
export function makeFatLineSegments(
  points: ArrayLike<number>,
  opts: FatLineOptions = {},
): LineSegments2 {
  const geo = new LineSegmentsGeometry();
  geo.setPositions(Array.from(points));
  const mat = makeFatLineMaterial(opts);
  const seg = new LineSegments2(geo, mat);
  seg.computeLineDistances();
  return seg;
}
