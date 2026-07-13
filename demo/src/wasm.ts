// WASM module loader. The pkg is imported at runtime (not bundled) so the
// box3d_wasm_bg.wasm URL resolves relative to public/pkg/ in both the dev
// server and the static GitHub Pages deployment.

export interface Box3dWasm {
  version(): string;
  compute_cos_sin(radians: number): Float32Array;
  atan2(y: number, x: number): number;
  polygon_points(sides: number, radius: number, angle: number, cx: number, cy: number): Float32Array;

  collide_spheres_demo(bx: number, by: number, bz: number): Float32Array;
  collide_capsules_demo(bx: number, by: number, bz: number, angle: number): Float32Array;
  collide_hull_sphere_demo(bx: number, by: number, bz: number): Float32Array;
  collide_hulls_demo(bx: number, by: number, bz: number, angle: number): Float32Array;

  box_hull_edges(hx: number, hy: number, hz: number): Float32Array;

  hf_build_wave(): number;
  hf_wireframe(): Float32Array;
  hf_ray_cast(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;

  mesh_build_box(): Float32Array;
  mesh_build_grid(): Float32Array;
  mesh_wireframe(): Float32Array;
  mesh_aabb(): Float32Array;
  mesh_ray_cast(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;

  sim_reset_stacking(): number;
  /** `shape`: 0 = hull box, 1 = capsule (C JengaStack DrawControls radio). */
  sim_reset_jenga(shape: number): number;
  sim_reset_single_box(): number;
  sim_step(dt: number, sub_steps: number): number;
  sim_body_poses(): Float32Array;
  /** Packed engine style words parallel to `sim_body_poses()`; see `applyShapeStyle`. */
  sim_body_styles(): Uint32Array;
  sim_body_count(): number;

  ragdoll_reset(): number;
  ragdoll_set_joint_params(friction: number, hertz: number, damping: number): void;
  ragdoll_step(dt: number, sub_steps: number): number;
  ragdoll_poses(): Float32Array;
  ragdoll_styles(): Uint32Array;
  ragdoll_body_count(): number;

  joint_reset_chain(): number;
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
  joint_set_driving_suspension(lower: number, upper: number, hertz: number, damping: number): void;
  joint_set_driving_steering(
    hertz: number,
    damping: number,
    torque: number,
    lower_deg: number,
    upper_deg: number,
  ): void;
  joint_drive_telemetry(): Float32Array;
  joint_revolute_energy(): Float32Array;
  joint_step(dt: number, sub_steps: number): number;
  joint_poses(): Float32Array;
  joint_styles(): Uint32Array;
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

  sim_reset_thin_wall(): number;
  sim_reset_bounce_house(): number;
  sim_reset_bullet_vs_stack(): number;
  sim_launch_bullet(): number;

  sensor_reset(scene: number): number;
  sensor_set_bullet(flag: boolean): void;
  sensor_is_bullet(): boolean;
  sensor_launch(): void;
  sensor_step(dt: number, sub_steps: number): number;
  sensor_poses(): Float32Array;
  sensor_colors(): Float32Array;
  sensor_sensor_indices(): Uint32Array;
  sensor_topology_version(): number;
  sensor_event_stats(): Float32Array;

  query_reset(): void;
  query_step(dt: number, sub_steps: number): number;
  query_poses(): Float32Array;
  query_styles(): Uint32Array;
  query_set_params(cast_type: number, mode: number, radius: number, initial_overlap: number): void;
  query_set_ray(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): void;
  query_add_shapes(shape_type: number, count: number): number;
  query_destroy_shape(): number;
  query_cast(): Float32Array;
  query_ignore_aabbs(): Float32Array;
  query_surface_wireframe(): Float32Array;
  query_mouse_down(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  query_mouse_move(px: number, py: number, pz: number): void;
  query_mouse_up(): void;
  query_mouse_active(): boolean;
  query_spawn_random(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  query_delete_at_ray(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): number;
  query_counters(): Float32Array;
  query_debug_draw(flags: number): Float32Array;

  sim_reset_compound(): number;
  sim_reset_compound_simple(): number;
  sim_reset_compound_spheres(): number;
  sim_reset_compound_hulls(): number;
  sim_reset_village(): number;
  sim_village_buildings(): Float32Array;
  sim_village_stats(): Float32Array;
  sim_reset_pyramid(): number;
  sim_reset_sphere_stack(): number;

  sim_mouse_down(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  sim_mouse_move(px: number, py: number, pz: number): void;
  sim_mouse_up(): void;
  sim_mouse_active(): boolean;
  sim_spawn_random(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  sim_delete_at_ray(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): number;
  sim_counters(): Float32Array;
  sim_debug_draw(flags: number): Float32Array;
  /** Global 16-bit view-flag mask (see `VIEW_FLAGS` in view-flags.ts); drives overlays + transparent dynamics. */
  sim_set_debug_flags(mask: number): void;
  /** Global joint/force draw scales for the debug overlay. */
  sim_set_draw_scales(joint_scale: number, force_scale: number): void;
  /** Debug text labels for the current view mask, as a JSON array of
   *  `{x,y,z,color,text}`. The flags come from `sim_set_debug_flags`. */
  sim_debug_text(): string;
  sim_step_count(): number;
  sim_set_enable_sleep(flag: boolean): void;
  sim_set_enable_warm_starting(flag: boolean): void;
  sim_set_enable_continuous(flag: boolean): void;
  sim_set_recycle_distance(meters: number): void;
  sim_start_recording(): void;
  sim_stop_recording(): Uint8Array;
  sim_is_recording(): boolean;
  sim_record_start_step(): number;

  bench_reset_large_pyramid(): number;
  bench_reset_junkyard(): number;
  bench_reset_trees(gridSize: number): number;
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

  is_double_precision_build(): boolean;
  world_far_pyramid_offset_km(): number;
  world_reset_far_pyramid(): number;
  world_far_pyramid_step(dt: number, sub_steps: number): number;
  world_far_pyramid_step_count(): number;
  world_far_pyramid_poses(): Float32Array;
  world_far_pyramid_styles(): Uint32Array;
  /** `[groundStyle, boxStyle]` — the packed engine style words for the Far
   *  Pyramid ground + boxes (consumed by the renderer agent). */
  world_far_pyramid_style_pair(): Uint32Array;
  world_far_pyramid_body_count(): number;
  world_far_pyramid_mouse_down(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  world_far_pyramid_mouse_move(px: number, py: number, pz: number): void;
  world_far_pyramid_mouse_up(): void;
  world_far_pyramid_mouse_active(): boolean;
  world_far_pyramid_spawn_random(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  world_far_pyramid_delete_at_ray(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): number;
  world_far_pyramid_counters(): Float32Array;
  world_far_pyramid_debug_draw(flags: number): Float32Array;
  world_far_pyramid_set_enable_sleep(flag: boolean): void;
  world_far_pyramid_set_enable_warm_starting(flag: boolean): void;
  world_far_pyramid_set_enable_continuous(flag: boolean): void;
  world_far_pyramid_set_recycle_distance(meters: number): void;

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
  character_styles(): Uint32Array;
  character_status(): Float32Array;
  character_debug_lines(): Float32Array;
  character_terrain_wireframe(): Float32Array;
  character_village_buildings(): Float32Array;
  character_village_stats(): Float32Array;
}

// --- Typed telemetry layouts (single source of truth for the positional
// Float32Arrays crossing the wasm boundary). Keep these in lockstep with the
// Rust builders they mirror. ---

/** Layout of `joint_drive_telemetry()` — demo/wasm/src/joint_drive.rs `telemetry()`. */
export const DRIVE_TELEMETRY = {
  length: 9,
  speed: 0,
  spinL: 1,
  spinR: 2,
  spinTorqueL: 3,
  spinTorqueR: 4,
  steerL: 5,
  steerR: 6,
  steerTorqueL: 7,
  steerTorqueR: 8,
} as const;

/** Layout of `joint_revolute_energy()` — demo/wasm/src/joint_demo.rs `joint_revolute_energy()`. */
export const REVOLUTE_ENERGY = {
  length: 3,
  kinetic: 0,
  potential: 1,
  total: 2,
} as const;

/** Layout of `sensor_event_stats()` — demo/wasm/src/sensor_demo.rs `sensor_event_stats()`. */
export const SENSOR_EVENT_STATS = {
  length: 5,
  begin: 0,
  end: 1,
  beginThisStep: 2,
  endThisStep: 3,
  benchmarkFlag: 4,
} as const;

// The global view-flag mask bit positions live in view-flags.ts (`VIEW_FLAGS`),
// the single source of truth shared with the View menu, the panel, and the bus.

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
