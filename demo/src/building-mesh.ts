// Shared Village building rendering — loads MIT `building.obj` from C samples.

import * as THREE from "three";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";

const BUILDING_URL = "/public/meshes/building.obj";

let buildingGeoPromise: Promise<THREE.BufferGeometry> | null = null;

/** Load and cache the C samples `building.obj` as a single BufferGeometry. */
export function loadBuildingGeometry(): Promise<THREE.BufferGeometry> {
  if (!buildingGeoPromise) {
    buildingGeoPromise = new Promise((resolve, reject) => {
      const loader = new OBJLoader();
      loader.load(
        BUILDING_URL,
        (group) => {
          const geos: THREE.BufferGeometry[] = [];
          group.traverse((child) => {
            const mesh = child as THREE.Mesh;
            if (mesh.isMesh && mesh.geometry) {
              geos.push(mesh.geometry as THREE.BufferGeometry);
            }
          });
          if (geos.length === 0) {
            reject(new Error("building.obj has no meshes"));
            return;
          }
          // Sample OBJ is a single object; merge if multiple parts appear.
          const geo =
            geos.length === 1
              ? geos[0]!.clone()
              : mergeGeometries(geos) ?? geos[0]!.clone();
          geo.computeVertexNormals();
          resolve(geo);
        },
        undefined,
        reject,
      );
    });
  }
  return buildingGeoPromise;
}

function mergeGeometries(
  geos: THREE.BufferGeometry[],
): THREE.BufferGeometry | null {
  // Lightweight merge without BufferGeometryUtils dependency.
  const positions: number[] = [];
  const normals: number[] = [];
  let indexOffset = 0;
  const indices: number[] = [];
  for (const g of geos) {
    const pos = g.getAttribute("position");
    const nor = g.getAttribute("normal");
    if (!pos) continue;
    for (let i = 0; i < pos.count; i++) {
      positions.push(pos.getX(i), pos.getY(i), pos.getZ(i));
      if (nor) normals.push(nor.getX(i), nor.getY(i), nor.getZ(i));
      else normals.push(0, 1, 0);
    }
    const idx = g.getIndex();
    if (idx) {
      for (let i = 0; i < idx.count; i++) {
        indices.push(idx.getX(i) + indexOffset);
      }
    } else {
      for (let i = 0; i < pos.count; i++) {
        indices.push(indexOffset + i);
      }
    }
    indexOffset += pos.count;
  }
  if (positions.length === 0) return null;
  const out = new THREE.BufferGeometry();
  out.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  out.setAttribute("normal", new THREE.Float32BufferAttribute(normals, 3));
  out.setIndex(indices);
  return out;
}

/** C-sample-ish light grey building with subtle wireframe edges. */
export function makeBuildingMaterial(): THREE.MeshStandardMaterial {
  return new THREE.MeshStandardMaterial({
    color: 0xc8c8c8,
    roughness: 0.82,
    metalness: 0.05,
    flatShading: true,
  });
}

/**
 * Apply wasm building instance buffer to an InstancedMesh.
 * Layout: `[px,py,pz, qx,qy,qz,qw, sx,sy,sz] * N`
 */
export function syncBuildingInstances(
  mesh: THREE.InstancedMesh,
  data: ArrayLike<number>,
): number {
  const stride = 10;
  const n = Math.floor(data.length / stride);
  const dummy = new THREE.Object3D();
  const quat = new THREE.Quaternion();
  for (let i = 0; i < n; i++) {
    const o = i * stride;
    dummy.position.set(data[o]!, data[o + 1]!, data[o + 2]!);
    quat.set(data[o + 3]!, data[o + 4]!, data[o + 5]!, data[o + 6]!);
    dummy.quaternion.copy(quat);
    // Negative scales (C mirrors) — Three.js supports via scale signs.
    dummy.scale.set(data[o + 7]!, data[o + 8]!, data[o + 9]!);
    dummy.updateMatrix();
    mesh.setMatrixAt(i, dummy.matrix);
  }
  mesh.count = n;
  mesh.instanceMatrix.needsUpdate = true;
  return n;
}

/** Format C Village DrawTextLine-style compound stats. */
export function formatVillageStats(stats: ArrayLike<number>): string {
  if (!stats || stats.length < 7) return "";
  const [caps, hulls, meshes, spheres, bytes, treeBytes, height] = [
    stats[0]!,
    stats[1]!,
    stats[2]!,
    stats[3]!,
    stats[4]!,
    stats[5]!,
    stats[6]!,
  ];
  return [
    `compound capsules/hulls/meshes/sphere = ${caps | 0} / ${hulls | 0} / ${meshes | 0} / ${spheres | 0}`,
    `compound byte count = ${bytes | 0}`,
    `compound tree byte count = ${treeBytes | 0}, height = ${height | 0}`,
  ].join("\n");
}
