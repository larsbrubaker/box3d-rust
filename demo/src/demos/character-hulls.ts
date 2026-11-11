// Convex-hull render layer for the Character page (demo/wasm/src/character_demo/hulls.rs).
//
// The shared 16-float pose stream only draws boxes / spheres / capsules / smooth
// cylinders. Character scenes that use real convex hulls (the Dynamic Mover's 12-sided
// pillars and 5-sided launcher, the Geometric Mover's falling tree) publish them on this
// channel instead: hull-local triangles once per reset, then a pose + engine style per
// frame, so the faces drawn are exactly the collision hulls.

import * as THREE from "three";
import type { Box3dWasm } from "../wasm.ts";
import { applyShapeStyle, makeShapeMaterial, type DemoScene } from "../three-scene.ts";

export interface HullLayer {
  /** Re-read the hull geometry for the scene that was just reset. */
  rebuild(): void;
  /** Apply this frame's hull poses / styles. */
  update(): void;
  clear(): void;
}

export function createHullLayer(demo: DemoScene, wasm: Box3dWasm): HullLayer {
  let meshes: THREE.Mesh[] = [];

  function clear() {
    for (const m of meshes) {
      demo.content.remove(m);
      m.geometry.dispose();
      (m.material as THREE.Material).dispose();
    }
    meshes = [];
  }

  function rebuild() {
    clear();
    const g = wasm.character_hull_geometry();
    const count = Math.round(g[0] ?? 0);
    let o = 1;
    for (let i = 0; i < count; i++) {
      o += 1; // color (the style stream carries it per frame)
      const n = Math.round(g[o] ?? 0);
      o += 1;
      const geo = new THREE.BufferGeometry();
      geo.setAttribute("position", new THREE.BufferAttribute(g.slice(o, o + n), 3));
      geo.computeVertexNormals();
      o += n;
      const mesh = new THREE.Mesh(geo, makeShapeMaterial());
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      mesh.visible = false;
      demo.content.add(mesh);
      meshes.push(mesh);
    }
  }

  function update() {
    if (meshes.length === 0) return;
    const poses = wasm.character_hull_poses();
    const styles = wasm.character_hull_styles();
    for (let i = 0; i < meshes.length; i++) {
      const mesh = meshes[i]!;
      const b = i * 8;
      const alpha = poses[b + 7] ?? 0;
      if (alpha <= 0) {
        mesh.visible = false;
        continue;
      }
      mesh.visible = true;
      mesh.position.set(poses[b]!, poses[b + 1]!, poses[b + 2]!);
      mesh.quaternion.set(poses[b + 3]!, poses[b + 4]!, poses[b + 5]!, poses[b + 6]!);
      applyShapeStyle(mesh, styles[i] ?? 0);
      const mat = mesh.material as THREE.MeshStandardMaterial;
      if (alpha < 1 && mat.opacity !== alpha) {
        if (!mat.transparent) {
          mat.transparent = true;
          mat.needsUpdate = true;
        }
        mat.opacity = alpha;
      }
    }
  }

  return { rebuild, update, clear };
}
