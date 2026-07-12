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

  sim_reset_bodies(): number;
  sim_reset_stacking(count: number): number;
  sim_reset_jenga(layers: number): number;
  sim_reset_single_box(): number;
  sim_step(dt: number, sub_steps: number): number;
  sim_body_poses(): Float32Array;
  sim_body_count(): number;

  ragdoll_reset(count: number): number;
  ragdoll_set_joint_params(friction: number, hertz: number, damping: number): void;
  ragdoll_step(dt: number, sub_steps: number): number;
  ragdoll_poses(): Float32Array;
  ragdoll_body_count(): number;

  joint_reset_chain(link_count: number): number;
  joint_reset_hinge(): number;
  joint_reset_gear_lift(): number;
  joint_reset_driving(): number;
  joint_set_motor(enabled: boolean, speed: number, torque: number): void;
  joint_set_revolute_params(
    flags: number,
    lower_deg: number,
    upper_deg: number,
    motor_speed: number,
    motor_torque: number,
    hertz: number,
    damping: number,
    target_deg: number,
  ): void;
  joint_set_drive_input(throttle_x: number, throttle_y: number): void;
  joint_set_drive_params(spin_speed: number, max_spin_torque: number): void;
  joint_step(dt: number, sub_steps: number): number;
  joint_poses(): Float32Array;
  joint_body_count(): number;
  joint_chassis_pose(): Float32Array;
  joint_terrain_wireframe(): Float32Array;
  joint_mouse_down(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  joint_mouse_move(px: number, py: number, pz: number): void;
  joint_mouse_up(): void;
  joint_mouse_active(): boolean;
  joint_spawn_random(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  joint_delete_at_ray(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): number;
  joint_counters(): Float32Array;
  joint_debug_draw(flags: number): Float32Array;

  continuous_reset(continuous: boolean): number;
  continuous_set_enabled(continuous: boolean): void;
  continuous_is_enabled(): boolean;
  continuous_step(dt: number, sub_steps: number): number;
  continuous_poses(): Float32Array;
  continuous_status(): Float32Array;

  sensor_reset(): number;
  sensor_step(dt: number, sub_steps: number): number;
  sensor_poses(): Float32Array;
  sensor_event_stats(): Float32Array;

  query_reset(): number;
  query_step(dt: number, sub_steps: number): number;
  query_poses(): Float32Array;
  query_ray_cast(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;

  sim_reset_compound(): number;
  sim_reset_compound_simple(): number;
  sim_reset_compound_spheres(): number;
  sim_reset_compound_hulls(): number;
  sim_reset_village(grid_count: number): number;
  sim_reset_pyramid(size: number): number;
  sim_reset_sphere_stack(count: number): number;

  sim_mouse_down(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  sim_mouse_move(px: number, py: number, pz: number): void;
  sim_mouse_up(): void;
  sim_mouse_active(): boolean;
  sim_spawn_random(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  sim_delete_at_ray(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): number;
  sim_counters(): Float32Array;
  sim_debug_draw(flags: number): Float32Array;
  sim_step_count(): number;
  sim_set_enable_sleep(flag: boolean): void;
  sim_set_enable_warm_starting(flag: boolean): void;
  sim_set_enable_continuous(flag: boolean): void;
  sim_set_recycle_distance(meters: number): void;
  sim_start_recording(): void;
  sim_stop_recording(): Uint8Array;
  sim_is_recording(): boolean;
  sim_record_start_step(): number;

  bench_reset_large_pyramid(base_count: number): number;
  bench_reset_junkyard(): number;
  bench_reset_trees(): number;
  bench_step(dt: number, sub_steps: number): number;
  bench_body_poses(): Float32Array;
  bench_body_count(): number;
  bench_mesh_wireframe(): Float32Array;
  bench_mouse_down(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  bench_mouse_move(px: number, py: number, pz: number): void;
  bench_mouse_up(): void;
  bench_mouse_active(): boolean;
  bench_spawn_random(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  bench_delete_at_ray(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): number;
  bench_counters(): Float32Array;
  bench_debug_draw(flags: number): Float32Array;

  terrain_reset(mode: number): number;
  terrain_step(dt: number, sub_steps: number): number;
  terrain_poses(): Float32Array;
  terrain_wireframe(): Float32Array;

  character_reset(): number;
  character_reset_ex(mode: number, grid_count: number): number;
  character_set_input(
    throttle_x: number,
    throttle_y: number,
    jump: boolean,
    sprint: boolean,
    fwd_x: number,
    fwd_z: number,
    right_x: number,
    right_z: number,
  ): void;
  character_step(dt: number, sub_steps: number): number;
  character_poses(): Float32Array;
  character_status(): Float32Array;
  character_debug_lines(): Float32Array;
  character_terrain_wireframe(): Float32Array;
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
