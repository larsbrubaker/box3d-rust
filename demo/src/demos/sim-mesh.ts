// Shared mesh sync for dynamics demos using the 16-float pose stride.

import * as THREE from "three";
import { applyShapeStyle, COLORS, makeShapeMaterial } from "../three-scene.ts";

export const POSE_STRIDE = 16;
export const KIND_BOX = 0;
export const KIND_SPHERE = 1;
export const KIND_CAPSULE = 2;
export const KIND_CYLINDER = 3;
export const KIND_ICOSAHEDRON = 4;

const _quat = new THREE.Quaternion();
const _c1 = new THREE.Vector3();
const _c2 = new THREE.Vector3();
const _mid = new THREE.Vector3();
const _dir = new THREE.Vector3();
const _yUp = new THREE.Vector3(0, 1, 0);

export type MeshPool = {
  meshes: THREE.Object3D[];
  boxGeo: THREE.BoxGeometry;
  sphereGeo: THREE.SphereGeometry;
  icoGeo: THREE.IcosahedronGeometry;
  groundMat: THREE.MeshStandardMaterial;
  dynamicMat: THREE.MeshStandardMaterial;
  sensorMat: THREE.MeshStandardMaterial;
  boneMats: THREE.MeshStandardMaterial[];
  colorMats: Map<number, THREE.MeshStandardMaterial>;
  /** Per-mesh materials for the engine-driven style path (index-aligned to `meshes`). */
  styleMats: THREE.MeshStandardMaterial[];
};

export function createMeshPool(): MeshPool {
  const boneColors = [0x4a90d9, 0x2f6fad, 0xe8c39e, 0xd4a574, 0x3db8a0, 0xc45c5c];
  return {
    meshes: [],
    boxGeo: new THREE.BoxGeometry(2, 2, 2),
    sphereGeo: new THREE.SphereGeometry(1, 20, 14),
    icoGeo: new THREE.IcosahedronGeometry(1, 0),
    groundMat: new THREE.MeshStandardMaterial({
      color: 0x9aa3b2,
      roughness: 0.92,
      metalness: 0.05,
    }),
    dynamicMat: new THREE.MeshStandardMaterial({
      color: COLORS.accent,
      roughness: 0.42,
      metalness: 0.12,
    }),
    sensorMat: new THREE.MeshStandardMaterial({
      color: 0x22c55e,
      transparent: true,
      opacity: 0.35,
      roughness: 0.4,
      metalness: 0.05,
      depthWrite: false,
    }),
    boneMats: boneColors.map(
      (c) =>
        new THREE.MeshStandardMaterial({
          color: c,
          roughness: 0.45,
          metalness: 0.1,
        }),
    ),
    colorMats: new Map(),
    styleMats: [],
  };
}

export function disposeMeshPool(pool: MeshPool) {
  pool.boxGeo.dispose();
  pool.sphereGeo.dispose();
  pool.icoGeo.dispose();
  pool.groundMat.dispose();
  pool.dynamicMat.dispose();
  pool.sensorMat.dispose();
  for (const m of pool.boneMats) m.dispose();
  for (const m of pool.colorMats.values()) m.dispose();
  pool.colorMats.clear();
  for (const m of pool.styleMats) m.dispose();
  pool.styleMats.length = 0;
  for (const mesh of pool.meshes) {
    mesh.traverse((child) => {
      const m = child as THREE.Mesh;
      if (
        m.geometry &&
        m.geometry !== pool.boxGeo &&
        m.geometry !== pool.sphereGeo &&
        m.geometry !== pool.icoGeo
      ) {
        m.geometry.dispose();
      }
    });
  }
  pool.meshes.length = 0;
}

function coloredMaterial(pool: MeshPool, color: number): THREE.MeshStandardMaterial {
  let mat = pool.colorMats.get(color);
  if (!mat) {
    const metallic = color === 0x008b8b;
    mat = new THREE.MeshStandardMaterial({
      color,
      roughness: metallic ? 0.35 : 0.55,
      metalness: metallic ? 0.85 : 0.08,
    });
    pool.colorMats.set(color, mat);
  }
  return mat;
}

function materialFor(
  pool: MeshPool,
  index: number,
  kind: number,
  color: number,
  sensorIndex: number | null,
  sensorIndices: Set<number> | null,
  groundIndex: number | null,
): THREE.Material {
  if (color !== 0) return coloredMaterial(pool, color);
  if (sensorIndices !== null && sensorIndices.has(index)) return pool.sensorMat;
  if (sensorIndex !== null && index === sensorIndex) return pool.sensorMat;
  if (groundIndex !== null && index === groundIndex && kind === KIND_BOX) return pool.groundMat;
  if (kind === KIND_CAPSULE) {
    return pool.boneMats[index % pool.boneMats.length]!;
  }
  return pool.dynamicMat;
}

function isSharedGeo(pool: MeshPool, geo: THREE.BufferGeometry | undefined): boolean {
  return geo === pool.boxGeo || geo === pool.sphereGeo || geo === pool.icoGeo;
}

function disposeIfOwned(pool: MeshPool, obj: THREE.Object3D) {
  obj.traverse((child) => {
    const mesh = child as THREE.Mesh;
    if (mesh.geometry && !isSharedGeo(pool, mesh.geometry)) {
      mesh.geometry.dispose();
    }
  });
}

function makeCapsuleMesh(radius: number, length: number, mat: THREE.Material): THREE.Mesh {
  const cyl = Math.max(1e-4, length);
  return new THREE.Mesh(new THREE.CapsuleGeometry(radius, cyl, 4, 10), mat);
}

function makeCylinderMesh(radius: number, halfLength: number, mat: THREE.Material): THREE.Mesh {
  const h = Math.max(1e-4, 2 * halfLength);
  return new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, h, 20), mat);
}

export function syncMeshesFromPoses(
  content: THREE.Group,
  pool: MeshPool,
  poses: ArrayLike<number>,
  opts: {
    sensorIndex?: number | null;
    sensorIndices?: ArrayLike<number>;
    groundIndex?: number | null;
    colors?: ArrayLike<number>;
    /**
     * Packed engine style words parallel to `poses` (one per body). When given,
     * each mesh gets its own material driven by `applyShapeStyle` — the
     * engine-color path — and the legacy `colors`/pool-material selection is
     * bypassed.
     */
    styles?: ArrayLike<number>;
  } = {},
) {
  const n = Math.floor(poses.length / POSE_STRIDE);
  const sensorIndex = opts.sensorIndex ?? null;
  const groundIndex = opts.groundIndex === undefined ? 0 : opts.groundIndex;
  const sensorIndices =
    opts.sensorIndices !== undefined
      ? new Set(Array.from(opts.sensorIndices, (v) => Number(v)))
      : null;
  const styles = opts.styles;

  while (pool.meshes.length > n) {
    const m = pool.meshes.pop()!;
    content.remove(m);
    disposeIfOwned(pool, m);
  }

  for (let i = 0; i < n; i++) {
    const o = i * POSE_STRIDE;
    const kind = poses[o + 14]!;
    let mat: THREE.Material;
    if (styles !== undefined) {
      // Engine-color path: one owned material per mesh slot, written by
      // applyShapeStyle after the mesh is positioned below.
      let sm = pool.styleMats[i];
      if (!sm) {
        sm = makeShapeMaterial();
        pool.styleMats[i] = sm;
      }
      mat = sm;
    } else {
      const poseColor = Math.round(poses[o + 15]!) & 0xffffff;
      const color = opts.colors !== undefined ? (opts.colors[i]! | 0) : poseColor;
      mat = materialFor(pool, i, kind, color, sensorIndex, sensorIndices, groundIndex);
    }
    let obj = pool.meshes[i];

    if (kind === KIND_CAPSULE) {
      const c1x = poses[o + 7]!;
      const c1y = poses[o + 8]!;
      const c1z = poses[o + 9]!;
      const c2x = poses[o + 10]!;
      const c2y = poses[o + 11]!;
      const c2z = poses[o + 12]!;
      const radius = poses[o + 13]!;
      _quat.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
      const bx = poses[o]!;
      const by = poses[o + 1]!;
      const bz = poses[o + 2]!;
      _c1.set(c1x, c1y, c1z).applyQuaternion(_quat).add(_mid.set(bx, by, bz));
      _c2.set(c2x, c2y, c2z).applyQuaternion(_quat).add(_mid.set(bx, by, bz));
      _dir.subVectors(_c2, _c1);
      const len = _dir.length();

      if (!obj || (obj as THREE.Mesh).geometry?.type !== "CapsuleGeometry") {
        if (obj) {
          content.remove(obj);
          disposeIfOwned(pool, obj);
        }
        obj = makeCapsuleMesh(radius, len, mat);
        content.add(obj);
        pool.meshes[i] = obj;
      } else {
        const mesh = obj as THREE.Mesh;
        const geo = mesh.geometry as THREE.CapsuleGeometry;
        const params = geo.parameters;
        if (Math.abs(params.radius - radius) > 1e-4 || Math.abs(params.length - Math.max(1e-4, len)) > 1e-3) {
          geo.dispose();
          mesh.geometry = new THREE.CapsuleGeometry(radius, Math.max(1e-4, len), 4, 10);
        }
        mesh.material = mat;
      }

      obj.position.copy(_c1).add(_c2).multiplyScalar(0.5);
      if (len > 1e-6) {
        obj.quaternion.setFromUnitVectors(_yUp, _dir.normalize());
      }
    } else if (kind === KIND_SPHERE) {
      const radius = poses[o + 7]!;
      if (!obj || (obj as THREE.Mesh).geometry !== pool.sphereGeo) {
        if (obj) {
          content.remove(obj);
          disposeIfOwned(pool, obj);
        }
        obj = new THREE.Mesh(pool.sphereGeo, mat);
        content.add(obj);
        pool.meshes[i] = obj;
      } else {
        (obj as THREE.Mesh).material = mat;
      }
      obj.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
      _quat.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
      obj.quaternion.copy(_quat);
      obj.scale.setScalar(radius);
    } else if (kind === KIND_CYLINDER) {
      const radius = poses[o + 7]!;
      const halfLength = poses[o + 8]!;
      if (!obj || (obj as THREE.Mesh).geometry?.type !== "CylinderGeometry") {
        if (obj) {
          content.remove(obj);
          disposeIfOwned(pool, obj);
        }
        obj = makeCylinderMesh(radius, halfLength, mat);
        content.add(obj);
        pool.meshes[i] = obj;
      } else {
        const mesh = obj as THREE.Mesh;
        const geo = mesh.geometry as THREE.CylinderGeometry;
        const params = geo.parameters;
        const h = Math.max(1e-4, 2 * halfLength);
        if (Math.abs(params.radiusTop - radius) > 1e-4 || Math.abs(params.height - h) > 1e-3) {
          geo.dispose();
          mesh.geometry = new THREE.CylinderGeometry(radius, radius, h, 20);
        }
        mesh.material = mat;
      }
      obj.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
      _quat.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
      obj.quaternion.copy(_quat);
      obj.scale.set(1, 1, 1);
    } else if (kind === KIND_ICOSAHEDRON) {
      const radius = poses[o + 7]!;
      if (!obj || (obj as THREE.Mesh).geometry !== pool.icoGeo) {
        if (obj) {
          content.remove(obj);
          disposeIfOwned(pool, obj);
        }
        obj = new THREE.Mesh(pool.icoGeo, mat);
        content.add(obj);
        pool.meshes[i] = obj;
      } else {
        (obj as THREE.Mesh).material = mat;
      }
      obj.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
      _quat.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
      obj.quaternion.copy(_quat);
      obj.scale.setScalar(radius);
    } else {
      if (!obj || (obj as THREE.Mesh).geometry !== pool.boxGeo) {
        if (obj) {
          content.remove(obj);
          disposeIfOwned(pool, obj);
        }
        obj = new THREE.Mesh(pool.boxGeo, mat);
        content.add(obj);
        pool.meshes[i] = obj;
      } else {
        (obj as THREE.Mesh).material = mat;
      }
      obj.position.set(poses[o]!, poses[o + 1]!, poses[o + 2]!);
      _quat.set(poses[o + 3]!, poses[o + 4]!, poses[o + 5]!, poses[o + 6]!);
      obj.quaternion.copy(_quat);
      obj.scale.set(poses[o + 7]!, poses[o + 8]!, poses[o + 9]!);
    }

    if (styles !== undefined) {
      applyShapeStyle(pool.meshes[i] as THREE.Mesh, styles[i]!);
    }
  }
}
