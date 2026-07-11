// WASM module loader. The pkg is imported at runtime (not bundled) so the
// box3d_wasm_bg.wasm URL resolves relative to public/pkg/ in both the dev
// server and the static GitHub Pages deployment.

export interface Box3dWasm {
  version(): string;
  compute_cos_sin(radians: number): Float32Array;
  atan2(y: number, x: number): number;
  polygon_points(sides: number, radius: number, angle: number, cx: number, cy: number): Float32Array;

  scene_shape(index: number): Float32Array;
  ray_cast_scene(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  closest_points(px: number, py: number, pz: number): Float32Array;

  collide_spheres_demo(bx: number, by: number, bz: number): Float32Array;
  collide_capsules_demo(bx: number, by: number, bz: number, angle: number): Float32Array;
  collide_hull_sphere_demo(bx: number, by: number, bz: number): Float32Array;
  collide_hulls_demo(bx: number, by: number, bz: number, angle: number): Float32Array;

  box_hull_edges(hx: number, hy: number, hz: number): Float32Array;
  create_hull_demo(sides: number, radius: number, height: number): Float32Array;

  hf_build_wave(): number;
  hf_wireframe(): Float32Array;
  hf_ray_cast(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;

  mesh_build_box(): Float32Array;
  mesh_build_grid(): Float32Array;
  mesh_wireframe(): Float32Array;
  mesh_aabb(): Float32Array;
  mesh_ray_cast(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;

  tree_reset(count: number): number;
  tree_proxy_aabbs(): Float32Array;
  tree_query(cx: number, cy: number, cz: number, h: number): Float32Array;
  tree_metrics(): Float32Array;
}

let wasmModule: Box3dWasm | null = null;

export async function loadWasm(): Promise<Box3dWasm> {
  if (!wasmModule) {
    const wasmUrl = new URL("./public/pkg/box3d_wasm.js", window.location.href).href;
    const mod = await import(wasmUrl);
    await mod.default();
    wasmModule = mod as Box3dWasm;
  }
  return wasmModule;
}

export function getWasm(): Box3dWasm {
  if (!wasmModule) throw new Error("WASM not loaded — call loadWasm() first");
  return wasmModule;
}
