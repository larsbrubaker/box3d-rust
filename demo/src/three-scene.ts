// Shared Three.js scene helper for demo SPA routes — Samples App look:
// muted sky, soft shadows, grid floor, flat-ish materials, C debug body colors.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

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

/** C physics_world.c debug shape palette (b3HexColor). */
export const DEBUG_BODY_COLORS = {
  static: 0xa9a9a9, // DarkGray
  kinematicAwake: 0x4682b4, // SteelBlue
  kinematicSleep: 0xb0c4de, // LightSteelBlue
  dynamicAwake: 0xd2b48c, // Tan
  dynamicSleep: 0x778899, // LightSlateGray
  bullet: 0x40e0d0, // Turquoise
  sensor: 0xf5deb3, // Wheat
} as const;

/** body_type: 0 static, 1 kinematic, 2 dynamic (matches b3BodyType). */
export function debugBodyColor(bodyType: number, awake: boolean): number {
  if (bodyType === 0) return DEBUG_BODY_COLORS.static;
  if (bodyType === 1) {
    return awake ? DEBUG_BODY_COLORS.kinematicAwake : DEBUG_BODY_COLORS.kinematicSleep;
  }
  return awake ? DEBUG_BODY_COLORS.dynamicAwake : DEBUG_BODY_COLORS.dynamicSleep;
}

/** Roughness / metalness from C debug_adapter kBodyType* + material presets. */
export function debugBodyMaterialProps(
  bodyType: number,
  awake: boolean,
): { roughness: number; metalness: number } {
  if (bodyType === 0) return { roughness: 0.85, metalness: 0.0 }; // matte
  if (bodyType === 1) {
    return awake
      ? { roughness: 0.35, metalness: 0.85 } // metallic
      : { roughness: 0.85, metalness: 0.0 }; // matte
  }
  return awake
    ? { roughness: 0.65, metalness: 0.0 } // soft
    : { roughness: 0.95, metalness: 0.0 }; // dead
}

export function makeBodyMaterial(
  bodyType: number,
  awake: boolean,
  opacity = 1,
): THREE.MeshStandardMaterial {
  const props = debugBodyMaterialProps(bodyType, awake);
  return new THREE.MeshStandardMaterial({
    color: debugBodyColor(bodyType, awake),
    roughness: props.roughness,
    metalness: props.metalness,
    transparent: opacity < 1,
    opacity,
    flatShading: true,
    side: THREE.DoubleSide,
  });
}

/** Apply C debug colorization to an existing standard material. */
export function applyBodyColor(
  mat: THREE.MeshStandardMaterial,
  bodyType: number,
  awake: boolean,
) {
  const props = debugBodyMaterialProps(bodyType, awake);
  mat.color.setHex(debugBodyColor(bodyType, awake));
  mat.roughness = props.roughness;
  mat.metalness = props.metalness;
}

const _yUp = new THREE.Vector3(0, 1, 0);
const _tmp = new THREE.Vector3();
const _tmp2 = new THREE.Vector3();

export class DemoScene {
  readonly scene: THREE.Scene;
  readonly camera: THREE.PerspectiveCamera;
  readonly renderer: THREE.WebGLRenderer;
  readonly controls: OrbitControls;
  /** Cleared each frame for dynamic overlays (rays, hits, contacts). */
  readonly dynamic: THREE.Group;
  /** Persistent content (shapes that change infrequently). */
  readonly content: THREE.Group;
  readonly keyLight: THREE.DirectionalLight;

  private readonly canvas: HTMLCanvasElement;
  private readonly ro: ResizeObserver;
  private disposed = false;
  private readonly grid: THREE.GridHelper;

  constructor(
    canvas: HTMLCanvasElement,
    opts: {
      target?: [number, number, number];
      distance?: number;
      fov?: number;
      /** Shadow camera half-extent (default 28). */
      shadowExtent?: number;
      gridSize?: number;
      gridDivisions?: number;
    } = {},
  ) {
    this.canvas = canvas;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(COLORS.sky);
    this.scene.fog = new THREE.Fog(COLORS.sky, 28, 90);

    this.camera = new THREE.PerspectiveCamera(opts.fov ?? 45, 1, 0.05, 200);
    const dist = opts.distance ?? 12;
    this.camera.position.set(dist * 0.55, dist * 0.35, dist * 0.75);

    this.renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: true,
      alpha: false,
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.05;

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.target.set(...(opts.target ?? [0, 0, 0]));
    this.controls.update();

    const ambient = new THREE.AmbientLight(0xc8d0dc, 0.42);
    const key = new THREE.DirectionalLight(0xfff5e8, 1.05);
    key.position.set(8, 16, 6);
    key.castShadow = true;
    const extent = opts.shadowExtent ?? 28;
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

    const fill = new THREE.DirectionalLight(0xa8b8d0, 0.28);
    fill.position.set(-6, 4, -8);
    this.scene.add(ambient, key, fill);

    const gridSize = opts.gridSize ?? 40;
    const gridDiv = opts.gridDivisions ?? 40;
    this.grid = new THREE.GridHelper(gridSize, gridDiv, 0x7a8494, 0x5c6574);
    this.grid.position.y = 0.001;
    const gridMat = this.grid.material as THREE.LineBasicMaterial | THREE.LineBasicMaterial[];
    if (Array.isArray(gridMat)) {
      for (const m of gridMat) {
        m.transparent = true;
        m.opacity = 0.55;
      }
    } else {
      gridMat.transparent = true;
      gridMat.opacity = 0.55;
    }
    this.scene.add(this.grid);

    this.content = new THREE.Group();
    this.dynamic = new THREE.Group();
    this.scene.add(this.content, this.dynamic);

    this.ro = new ResizeObserver(() => this.resize());
    this.ro.observe(canvas.parentElement ?? canvas);
    this.resize();
  }

  resize() {
    if (this.disposed) return;
    const parent = this.canvas.parentElement ?? this.canvas;
    const w = Math.max(1, Math.round(parent.clientWidth));
    const h = Math.max(1, Math.round(parent.clientHeight));
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h, false);
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
    this.renderer.render(this.scene, this.camera);
  }

  dispose() {
    if (this.disposed) return;
    this.disposed = true;
    this.ro.disconnect();
    this.controls.dispose();
    this.clearContent();
    this.clearDynamic();
    this.scene.remove(this.grid);
    disposeObject(this.grid);
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

/// C `camera::SetView(yaw, pitch, distance, target)` → orbit camera position.
/// x = target.x + d·cos(pitch)·sin(yaw), y = target.y + d·sin(pitch),
/// z = target.z + d·cos(pitch)·cos(yaw); angles in degrees.
export function setView(
  demo: DemoScene,
  yawDeg: number,
  pitchDeg: number,
  distance: number,
  target: [number, number, number],
) {
  demo.controls.target.set(target[0], target[1], target[2]);
  const yaw = (yawDeg * Math.PI) / 180;
  const pitch = (pitchDeg * Math.PI) / 180;
  demo.camera.position.set(
    target[0] + distance * Math.cos(pitch) * Math.sin(yaw),
    target[1] + distance * Math.sin(pitch),
    target[2] + distance * Math.cos(pitch) * Math.cos(yaw),
  );
  demo.controls.update();
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

export function makeWireEdges(
  edges: ArrayLike<number>,
  color: number,
  offset = 0,
): THREE.LineSegments {
  const positions: number[] = [];
  for (let i = offset; i + 5 < edges.length; i += 6) {
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
  return new THREE.LineSegments(geo, lineMat(color));
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
