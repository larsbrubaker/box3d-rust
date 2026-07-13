// WASM module loader. The pkg is imported at runtime (not bundled) so the
// box3d_wasm_bg.wasm URL resolves relative to public/pkg/ in both the dev
// server and the static GitHub Pages deployment.

export interface Box3dWasm {
  version(): string;
  compute_cos_sin(radians: number): Float32Array;
  atan2(y: number, x: number): number;
  polygon_points(sides: number, radius: number, angle: number, cx: number, cy: number): Float32Array;

  /** Select a manifold viewer (scene 0..=8, C order) and return its scene descriptor. */
  manifold_reset(scene: number): Float32Array;
  /** Push shape B's mouse-driven world transform (position + quaternion x,y,z,w). */
  manifold_set_transform_b(
    px: number, py: number, pz: number,
    qx: number, qy: number, qz: number, qw: number,
  ): void;
  /** Set the "Use cache" checkbox and manual-feature radio (0 auto,1 faceA,2 faceB,3 edgePair). */
  manifold_set_cache(use_cache: boolean, manual_feature: number): void;
  /** Recompute the manifold in the current pose; returns it packed in world space. */
  manifold_step(): Float32Array;

  // --- Geometry category (sample_geometry.cpp; demo/wasm/src/geometry_demo.rs).
  // Each returns packed hull blocks (see geometry_demo.rs module docs): a leading
  // hull-block count, then per block an 8-float header
  // `[surfaceArea, volume, innerRadius, vertexCount, faceCount, uniqueEdges,
  //   triFloatLen, wireFloatLen]` followed by its triangle then wire floats.
  geometry_box_hull(
    hx: number, hy: number, hz: number,
    cx: number, cy: number, cz: number,
    rx: number, ry: number, rz: number,
    sx: number, sy: number, sz: number,
  ): Float32Array;
  geometry_hull(): Float32Array;
  geometry_hull_reduction(kind: number, count: number): Float32Array;
  geometry_hull_transform(
    sx: number, sy: number, sz: number,
    rx: number, ry: number, rz: number,
    px: number, py: number, pz: number,
  ): Float32Array;
  /** Capsule Mass: two hull blocks (capsule hull, box hull) then
   *  `[capLen, capRadius, massHull, massCap, massBox, ixx×3, iyy×3, izz×3]`. */
  geometry_capsule_mass(sides: number): Float32Array;

  // --- Height Field (sample_mesh.cpp HeightField; demo/wasm/src/height_field_demo.rs).
  hf_reset(rowCount: number, columnCount: number, amplitude: number, holes: boolean): number;
  hf_wireframe(): Float32Array;
  hf_info(): Float32Array;
  hf_cast(
    ox: number, oy: number, oz: number, tx: number, ty: number, tz: number, radius: number,
  ): Float32Array;

  // --- Mesh category (sample_mesh.cpp; demo/wasm/src/mesh_demo/). demo_shell +
  // world-toggle exports (mesh_mouse_*, mesh_spawn_random, mesh_delete_at_ray,
  // mesh_counters, mesh_debug_draw/text, mesh_set_enable_*) are reached through
  // makeInteractAdapter's bracket access and are not re-declared here.
  mesh_reset(scene: number, shapeType: number, scaleX: number, scaleZ: number): number;
  mesh_reset_reflection(scaleX: number, scaleY: number, scaleZ: number): number;
  mesh_reset_hollow_box(): number;
  mesh_reset_voxel(objText: string): number;
  mesh_reset_viewer(
    objText: string, medianSplit: boolean, concaveEdges: boolean, weldVertices: boolean,
    weldToleranceMm: number,
  ): number;
  mesh_reset_benchmark(obj1: string, obj2: string, obj3: string, obj4: string): number;
  mesh_set_shape(shapeType: number): void;
  mesh_step(dt: number, subSteps: number): number;
  mesh_poses(): Float32Array;
  mesh_styles(): Uint32Array;
  mesh_counters(): Float32Array;
  mesh_body_count(): number;
  mesh_ground_wireframe(): Float32Array;
  mesh_voxel_hull_wireframe(): Float32Array;
  mesh_stats(): Float32Array;
  mesh_viewer_height(): number;
  mesh_viewer_nodes(level: number): Float32Array;
  mesh_benchmark_build(): number;

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
  /** Reset to a ragdoll scene: 0 Box, 1 Mesh, 2 Pile, 3 Incline. */
  ragdoll_reset_scene(scene: number): number;
  /** Baked world-space ground-mesh wireframe for the current scene (empty for Box). */
  ragdoll_ground_wireframe(): Float32Array;
  ragdoll_set_joint_params(friction: number, hertz: number, damping: number): void;
  ragdoll_step(dt: number, sub_steps: number): number;
  ragdoll_poses(): Float32Array;
  ragdoll_styles(): Uint32Array;
  ragdoll_body_count(): number;

  // Robustness (sample_robustness.cpp): 0 HighMassRatio1, 1 Tiny Pyramid,
  // 2 Overlap Recovery, 3 Overflow Color Pile.
  robustness_reset_scene(scene: number): number;
  robustness_set_overlap_params(
    extent: number,
    base_count: number,
    overlap: number,
    speed: number,
    hertz: number,
    damping_ratio: number,
  ): void;
  robustness_step(dt: number, sub_steps: number): number;
  robustness_poses(): Float32Array;
  robustness_styles(): Uint32Array;
  robustness_body_count(): number;
  robustness_counters(): Float32Array;
  /** [neighbor_count, overflow_contacts, total_contacts] (Overflow Color Pile). */
  robustness_overflow_stats(): Float32Array;
  /** Tiny Pyramid box edge length in centimetres (200 * extent). */
  robustness_tiny_cm(): number;

  joint_reset_chain(): number;
  joint_reset_hinge(): number;
  joint_reset_gear_lift(): number;
  joint_reset_driving(): number;
  // Batch-3b scenes (sample_joint.cpp).
  joint_reset_distance(
    count: number,
    hertz: number,
    damping: number,
    length_v: number,
    tension: number,
    compression: number,
    min_length: number,
    max_length: number,
    enable_spring: boolean,
    enable_limit: boolean,
  ): number;
  joint_set_distance_params(
    length_v: number,
    enable_spring: boolean,
    tension: number,
    compression: number,
    hertz: number,
    damping: number,
    enable_limit: boolean,
    min_length: number,
    max_length: number,
  ): void;
  joint_reset_filter(): number;
  joint_reset_motor(): number;
  joint_motor_set_params(speed: number, max_force: number, max_torque: number): void;
  joint_motor_apply_impulse(): void;
  joint_motor_readout(): Float32Array;
  joint_reset_top_down_friction(): number;
  joint_explode(): void;
  joint_reset_prismatic(): number;
  joint_set_prismatic_params(
    flags: number,
    lower: number,
    upper: number,
    max_force: number,
    speed: number,
    hertz: number,
    damping: number,
    target: number,
  ): void;
  joint_reset_spherical(): number;
  joint_set_spherical_params(
    flags: number,
    cone_deg: number,
    lower_twist_deg: number,
    upper_twist_deg: number,
    max_torque: number,
    vel_x: number,
    vel_y: number,
    vel_z: number,
    hertz: number,
    damping: number,
    rot_x: number,
    rot_y: number,
    rot_z: number,
  ): void;
  joint_reset_parallel(): number;
  joint_set_parallel_params(hertz: number, damping: number): void;
  joint_reset_weld(): number;
  joint_set_weld_params(
    lin_hertz: number,
    lin_damp: number,
    ang_hertz: number,
    ang_damp: number,
  ): void;
  joint_reset_wheel(): number;
  joint_set_wheel_params(
    flags: number,
    susp_min: number,
    susp_max: number,
    max_spin_torque: number,
    spin_speed: number,
    susp_hertz: number,
    susp_damp: number,
    steer_hertz: number,
    steer_damp: number,
    target_deg: number,
    steer_min_deg: number,
    steer_max_deg: number,
  ): void;
  joint_wheel_steering_angle(): number;
  joint_reset_door(
    magnitude: number,
    two_joints: boolean,
    hertz: number,
    damping: number,
  ): number;
  joint_door_impulse(): void;
  joint_door_set_magnitude(magnitude: number): void;
  joint_door_set_limit(enable: boolean): void;
  joint_door_set_two_joints(two_joints: boolean): void;
  joint_door_set_tuning(hertz: number, damping: number): void;
  joint_door_readout(): Float32Array;
  joint_reset_bridge(): number;
  joint_set_bridge_gravity(scale: number): void;
  joint_reset_motion_locks(): number;
  joint_set_motion_locks(
    linear_x: boolean,
    linear_y: boolean,
    linear_z: boolean,
    angular_x: boolean,
    angular_y: boolean,
    angular_z: boolean,
  ): void;
  joint_motion_lock_impulse(): void;
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
  // Continuous batch 2 (continuous_scenes.rs).
  sim_reset_spinning_stick(): number;
  sim_reset_needle_mesh(): number;
  sim_reset_mesh_drop(): number;
  sim_reset_mesh_drop_unit(): number;
  sim_reset_hump_mesh(): number;
  sim_reset_is_fast(): number;
  sim_reset_stall(): number;
  /** Baked world-space ground-mesh wireframe for the current scene (empty for box-only scenes). */
  sim_cont_ground_wireframe(): Float32Array;
  /** Fire the Stall rock bullet (C `Stall::Launch`). */
  sim_cont_stall_launch(): number;
  /** Active CCD stall threshold in ms (C `b3GetStallThreshold` × 1000); Stall sets 1.0. */
  sim_cont_stall_threshold_ms(): number;
  /** Mesh Drop "Type" combo: 0 box, 1 capsule, 2 cylinder, 3 sphere. */
  sim_cont_mesh_drop_set_type(shape: number): number;
  /** Mesh Drop "Amplitude" slider (0..1); rebuilds ground + grid. */
  sim_cont_mesh_drop_set_amplitude(amplitude: number): number;
  /** Mesh Drop "Generate" button; reseeds from `ticks` (C `b3GetTicks()`) and rebuilds. */
  sim_cont_mesh_drop_generate(ticks: number): number;
  /** Bodies that moved on the last step (`b3BodyEvents.moveCount`) — drives Auto Generate. */
  sim_cont_mesh_drop_move_count(): number;
  /** Minimum tracked-body mass-center height (Mesh Drop Unit Test failure readout). */
  sim_cont_min_body_height(): number;

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
  /** Packed overlay `[segCount, ptCount, ...segs(7), ...pts(5)]` (DebugDrawOverlay layout);
   *  Hit + Persistent Contact event markers. `[0, 0]` for the other scenes. */
  sensor_overlay(): Float32Array;
  /** JSON `[{x,y,z,color,text}]` 3D debug-text labels (C `DrawString3D`); `"[]"` if none. */
  sensor_debug_text(): string;
  /** JSON `[{label,value}]` HUD readout rows for the Events scenes; `"[]"` for the sensor scenes. */
  sensor_hud(): string;
  /** Ground mesh triangle edges for Hit / Persistent Contact (empty for box-ground scenes). */
  sensor_ground_wireframe(): Float32Array;

  // --- Shapes category (sample_shapes.cpp) ---
  shapes_reset(scene: number): number;
  shapes_reset_restitution(box_shape: boolean): number;
  shapes_reset_wind(shape_type: number, count: number): number;
  shapes_reset_conveyor(obj_text: string): number;
  shapes_set_wind_live(wind_x: number, drag: number, lift: number): void;
  shapes_set_invoke(invoke: boolean): void;
  shapes_create_static(): void;
  shapes_destroy_static(): void;
  shapes_static_exists(): boolean;
  shapes_step(dt: number, sub_steps: number): number;
  shapes_poses(): Float32Array;
  shapes_styles(): Uint32Array;
  shapes_debug_text(): string;
  shapes_debug_draw(flags: number): Float32Array;
  shapes_counters(): Float32Array;
  shapes_wind_arrow(): Float32Array;
  shapes_conveyor_mesh(): Float32Array;
  shapes_conveyor_colors(): Uint32Array;
  shapes_conveyor_velocity_lines(): Float32Array;
  shapes_body_count(): number;
  shapes_mouse_down(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  shapes_mouse_move(px: number, py: number, pz: number): void;
  shapes_mouse_up(): void;
  shapes_mouse_active(): boolean;
  shapes_spawn_random(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  shapes_delete_at_ray(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): number;

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

  // Collision / Ray Curtain (sample_collision.cpp RayCurtain).
  rc_reset(): void;
  rc_step(dt: number, sub_steps: number): void;
  rc_poses(): Float32Array;
  rc_surface_wireframe(): Float32Array;
  rc_rays(): Float32Array;

  // Collision / Mesh Scale (MeshScale).
  msc_reset(): void;
  msc_set_params(sx: number, sy: number, sz: number, start_y: number, start_z: number, sphere_cast: number): void;
  msc_wireframe(): Float32Array;
  msc_cast(): Float32Array;

  // Collision / Shape Cast (ShapeCast).
  sc_reset(): void;
  sc_step(dt: number, sub_steps: number): void;
  sc_set_offset(y: number, z: number): void;
  sc_set_initial_overlap(flag: number): void;
  sc_poses(): Float32Array;
  sc_surface_wireframe(): Float32Array;
  sc_casts(): Float32Array;

  // Collision / Overlap World (OverlapWorld).
  ow_reset(): void;
  ow_step(dt: number, sub_steps: number): void;
  ow_set_offset(offset: number): void;
  ow_poses(): Float32Array;
  ow_surface_wireframe(): Float32Array;
  ow_overlaps(): Float32Array;

  // Collision / Long Ray Cast (LongRayCast).
  lrc_reset(): void;
  lrc_set_params(ray_length_km: number, cone_angle: number): void;
  lrc_poses(): Float32Array;
  lrc_surface_wireframe(): Float32Array;
  /** Rock hull geometry (world space): `[triCount, tris…, edgeCount, edges…]`. */
  lrc_rock_geometry(): Float32Array;
  lrc_step(): Float32Array;

  // Collision / Initial Overlap (InitialOverlap).
  io_reset(): void;
  io_set_initial_overlap(flag: number): void;
  io_surface_wireframe(): Float32Array;
  io_cast(): Float32Array;

  // Collision / Shape Cast Debug (ShapeCastDebug).
  scd_data(): Float32Array;

  // Collision / Distance Debug (DistanceDebug).
  dd_data(simplex_index: number): Float32Array;

  // Collision / Shape Distance (ShapeDistance).
  sd_reset(): void;
  sd_set_params(type_a: number, type_b: number, radius_a: number, radius_b: number, use_cache: number, show_indices: number, draw_simplex: number): void;
  sd_set_transform_b(px: number, py: number, pz: number, qx: number, qy: number, qz: number, qw: number): void;
  sd_step(simplex_index: number): Float32Array;

  // Collision / Time of Impact (TimeOfImpact).
  toi_data(type_a: number, type_b: number): Float32Array;

  // Collision / Capsule Cast Ray (CapsuleCastRay).
  ccray_reset(): void;
  ccray_poses(): Float32Array;
  ccray_cast(): Float32Array;

  sim_reset_compound(): number;
  sim_reset_compound_simple(): number;
  sim_reset_compound_spheres(): number;
  sim_reset_compound_hulls(): number;
  sim_reset_village(): number;
  sim_village_buildings(): Float32Array;
  sim_village_stats(): Float32Array;
  // Compound / Tile Floor + Mesh Tile (sample_compound.cpp).
  sim_reset_tile_floor(): number;
  sim_reset_mesh_tile(): number;
  sim_tile_transforms(): Float32Array;
  sim_tile_half(): Float32Array;
  sim_tile_stats(): Float32Array;
  // Compound / Village character mover + sweeping query visualization.
  sim_village_set_input(
    throttle_x: number,
    throttle_y: number,
    jump: boolean,
    sprint: boolean,
    fwd_x: number,
    fwd_z: number,
    right_x: number,
    right_z: number,
  ): void;
  sim_village_mover_step(dt: number): void;
  sim_village_query_step(dt: number): void;
  sim_village_mover_pose(): Float32Array;
  sim_village_query(): Float32Array;
  sim_village_toggle_third_person(): boolean;
  sim_reset_pyramid(): number;
  sim_reset_sphere_stack(): number;

  // --- Stacking batch 3b (sample_stacking.cpp) ---
  sim_reset_card_house_thick(): number;
  sim_reset_card_house(): number;
  sim_reset_capsule_stack(): number;
  sim_reset_cylinder(): number;
  sim_reset_cylinder_stack(): number;
  sim_reset_dominoes(): number;
  sim_reset_wedge(): number;
  sim_reset_arch(): number;
  sim_reset_double_domino(): number;
  /** Per-body local hull triangle geometry for the arbitrary-hull stacking scenes
   *  (Cylinder, Cylinder Stack, Wedge, Arch). Layout: for each body in
   *  `sim_body_poses` order, a `floatCount` value then `floatCount` triangle-vertex
   *  floats (`0` for non-hull bodies). */
  sim_hull_geometry(): Float32Array;

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
  /** Override the active scene's projectile launch-speed scale (`m_launchSpeedScale`).
   *  Scene resets restore the base default of 5.0, so call this after a reset when
   *  the matching C sample overrides it (e.g. Compound Village = 2.0). */
  sim_set_launch_speed_scale(scale: number): void;
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

  // --- Replay viewer (sample_replay.cpp): .b3rec recording player ---
  replay_load(data: Uint8Array): boolean;
  replay_unload(): void;
  replay_loaded(): boolean;
  replay_frame_count(): number;
  replay_frame(): number;
  replay_time_step(): number;
  replay_sub_step_count(): number;
  replay_bounds(): Float32Array;
  replay_body_count(): number;
  replay_is_at_end(): boolean;
  replay_has_diverged(): boolean;
  replay_diverge_frame(): number;
  replay_step(): void;
  replay_seek(frame: number): void;
  replay_restart(): void;
  replay_scene_geometry(): Float32Array;
  replay_body_transforms(): Float32Array;
  replay_shape_styles(): Uint32Array;
  /** JSON outline: `[{ord,name,type,shapes:[typeName,...]}]` for bodies valid this frame. */
  replay_outline(): string;
  /** JSON inspector for body `ord` at the current frame: `{present,id,name,type,pos,spinDeg,vel,omega,speed,spinRate,mass,awake,enabled,bullet,gravityScale,shapeCount,jointCount}` or `{present:false}`. */
  replay_body_detail(ord: number): string;

  bench_reset_large_pyramid(): number;
  bench_reset_wide_pyramid(): number;
  bench_reset_many_pyramids(): number;
  bench_reset_rain(): number;
  bench_reset_joint_grid(): number;
  bench_reset_falling_boxes(): number;
  bench_reset_candy_cups(): number;
  bench_reset_explosion(): number;
  bench_reset_height_field(): number;
  bench_reset_trees(gridSize: number): number;
  bench_reset_washer(): number;
  bench_reset_large_world(): number;
  bench_reset_hull(): number;
  bench_reset_chains(): number;
  bench_reset_destruction(): number;
  bench_reset_junkyard(): number;
  bench_step(dt: number, sub_steps: number): number;
  bench_poses(): Float32Array;
  bench_styles(): Uint32Array;
  bench_mesh_wireframe(): Float32Array;
  bench_hull_wireframe_b(): Float32Array;
  bench_hull_info(): Float32Array;
  bench_washer_drum(): Float32Array;
  /** Washer drum geometry (drum-local): `[triCount, tris…, edgeCount, edges…]`. */
  bench_washer_drum_geometry(): Float32Array;
  /** Candy Cups frustum-hull solid faces (cup-local, flat `[x,y,z]` triples). */
  bench_candy_hull(): Float32Array;
  bench_set_explosion_magnitude(impulse: number): void;
  bench_explode(): void;
  bench_set_height_field_radius(radius: number): void;
  bench_height_field_cast(): Float32Array;
  bench_mouse_down(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  bench_mouse_move(px: number, py: number, pz: number): void;
  bench_mouse_up(): void;
  bench_mouse_active(): boolean;
  bench_spawn_random(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  bench_delete_at_ray(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): number;
  bench_counters(): Float32Array;
  bench_debug_draw(flags: number): Float32Array;
  bench_debug_text(): string;
  bench_set_enable_sleep(flag: boolean): void;
  bench_set_enable_warm_starting(flag: boolean): void;
  bench_set_enable_continuous(flag: boolean): void;
  bench_set_recycle_distance(meters: number): void;

  // --- Bodies category (sample_bodies.cpp; demo/wasm/src/bodies_demo.rs) ---
  bodies_reset(scene: number): number;
  bodies_step(dt: number, sub_steps: number): number;
  bodies_poses(): Float32Array;
  bodies_styles(): Uint32Array;
  bodies_counters(): Float32Array;
  bodies_debug_draw(flags: number): Float32Array;
  bodies_debug_text(): string;
  /** HUD readout line for Gyroscopic Torque (world center of mass); "" otherwise. */
  bodies_hud(): string;
  /** Per-scene always-on overlay geometry `[segCount, ptCount, ...segs(7), ...pts(5)]`. */
  bodies_overlay(): Float32Array;
  /** Cast solid proxy shapes `[count, then per shape: kind, c1(3), c2(3), radius, colorBits]`. */
  bodies_cast_shapes(): Float32Array;
  bodies_set_type(t: number): void;
  bodies_set_enabled(flag: boolean): void;
  bodies_enable_link(flag: boolean): void;
  bodies_enable_ball(flag: boolean): void;
  bodies_teleport(): void;
  bodies_explode(): void;
  bodies_set_magnitude(m: number): void;
  bodies_cast_track_down(ox: number, oy: number, oz: number, dx: number, dy: number, dz: number): void;
  bodies_cast_track_move(ox: number, oy: number, oz: number, dx: number, dy: number, dz: number): void;
  bodies_cast_track_up(): void;
  bodies_mouse_down(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  bodies_mouse_move(px: number, py: number, pz: number): void;
  bodies_mouse_up(): void;
  bodies_mouse_active(): boolean;
  bodies_spawn_random(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  bodies_delete_at_ray(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): number;

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

  // --- World / Determinism ---
  // Far Stack / Far Ragdolls / Far Mesh Drop share one generic `world_far_*`
  // scene (only one is active at a time); the page calls the matching reset.
  // Every position crossing the boundary is already shifted into the scene's
  // base frame (16-float `vis` pose stride, ground wireframe, debug draw/text).
  world_far_reset_stack(offset_km: number): number;
  world_far_reset_ragdolls(): number;
  world_far_reset_mesh_drop(): number;
  world_far_step(dt: number, sub_steps: number): number;
  world_far_step_count(): number;
  world_far_poses(): Float32Array;
  world_far_styles(): Uint32Array;
  /** Ground mesh triangle edges in the base frame (empty for Far Stack). */
  world_far_ground_wireframe(): Float32Array;
  world_far_offset_km(): number;
  /** Far Mesh Drop failure latch (`m_failed`); false for the other scenes. */
  world_far_mesh_drop_failed(): boolean;
  world_far_mouse_down(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  world_far_mouse_move(px: number, py: number, pz: number): void;
  world_far_mouse_up(): void;
  world_far_mouse_active(): boolean;
  world_far_spawn_random(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  world_far_delete_at_ray(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): number;
  world_far_counters(): Float32Array;
  world_far_debug_draw(flags: number): Float32Array;
  world_far_debug_text(): string;
  world_far_set_enable_sleep(flag: boolean): void;
  world_far_set_enable_warm_starting(flag: boolean): void;
  world_far_set_enable_continuous(flag: boolean): void;
  world_far_set_recycle_distance(meters: number): void;

  // Falling Ragdolls determinism soak (`sample_determinism.cpp`).
  determinism_reset(): number;
  determinism_step(dt: number, sub_steps: number): number;
  determinism_step_count(): number;
  determinism_poses(): Float32Array;
  determinism_styles(): Uint32Array;
  determinism_ground_wireframe(): Float32Array;
  determinism_done(): boolean;
  determinism_sleep_step(): number;
  determinism_hash(): number;
  determinism_mouse_down(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  determinism_mouse_move(px: number, py: number, pz: number): void;
  determinism_mouse_up(): void;
  determinism_mouse_active(): boolean;
  determinism_spawn_random(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  determinism_delete_at_ray(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): number;
  determinism_counters(): Float32Array;
  determinism_debug_draw(flags: number): Float32Array;
  determinism_debug_text(): string;
  determinism_set_enable_sleep(flag: boolean): void;
  determinism_set_enable_warm_starting(flag: boolean): void;
  determinism_set_enable_continuous(flag: boolean): void;
  determinism_set_recycle_distance(meters: number): void;

  // Asset-free drag scenes: 0 = CapsulePlane, 1 = MoverOverlap.
  character_reset(scene: number): number;
  // Mover / Rigid Body take fetched OBJ text.
  character_reset_mover(test_map_obj: string, stairs_obj: string): number;
  character_reset_rigid_body(
    test_map_obj: string,
    stairs_obj: string,
    building_obj: string,
    voxel1_obj: string,
    voxel2_obj: string,
  ): number;
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
  character_set_clip_velocity(clip: boolean): void;
  character_set_third_person(third_person: boolean): void;
  character_set_drag(x: number, y: number, z: number): void;
  character_solve(): void;
  character_step(dt: number, sub_steps: number): number;
  character_poses(): Float32Array;
  character_styles(): Uint32Array;
  character_status(): Float32Array;
  character_ground_wireframe(): Float32Array;
  character_debug_segments(): Float32Array;
  character_debug_points(): Float32Array;
  character_follow_target(): Float32Array;

  // --- Issues category (sample_issues.cpp; demo/wasm/src/issues_demo/).
  issues_reset_dump_loader(): number;
  issues_reset_crash(): number;
  issues_reset_multiple_prismatic(): number;
  issues_reset_hull_crash(): number;
  issues_reset_convex_jitter(): number;
  issues_reset_sbox_mover(): number;
  issues_reset_capsule_mesh(): number;
  issues_step(dt: number, sub_steps: number): number;
  issues_poses(): Float32Array;
  issues_styles(): Uint32Array;
  issues_body_count(): number;
  issues_static_wireframe(): Float32Array;
  issues_hull_geometry(): Float32Array;
  issues_hull_poses(): Float32Array;
  issues_hull_crash(): Float32Array;
  issues_add_joint(): void;
  issues_mouse_down(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  issues_mouse_move(px: number, py: number, pz: number): void;
  issues_mouse_up(): void;
  issues_mouse_active(): boolean;
  issues_spawn_random(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): Float32Array;
  issues_delete_at_ray(ox: number, oy: number, oz: number, tx: number, ty: number, tz: number): number;
  issues_counters(): Float32Array;
  issues_debug_draw(flags: number): Float32Array;
  issues_debug_text(): string;
  issues_set_enable_sleep(flag: boolean): void;
  issues_set_enable_warm_starting(flag: boolean): void;
  issues_set_enable_continuous(flag: boolean): void;
  issues_set_recycle_distance(meters: number): void;

  // --- Tree category (sample_tree.cpp; demo/wasm/src/tree_demo.rs).
  tree_reset(file_index: number, text: string): void;
  tree_rebuild(): void;
  tree_stats(): Float32Array;
  tree_dims(): Float32Array;
  tree_leaf_boxes(): Float32Array;
  tree_level_boxes(level: number): Float32Array;
  tree_step_query(
    do_ray: boolean,
    do_overlap: boolean,
    do_closest: boolean,
    test_index: number,
  ): Float32Array;
  tree_profile_ray(): void;
  tree_profile_overlap(): void;
  tree_profile_closest(): void;
  tree_test_ray(index: number): Float32Array;
  tree_test_overlap(index: number): Float32Array;
  tree_test_sphere(index: number): Float32Array;
  tree_file_index(): number;
  /** Serialize the current dynamic tree to the portable leaf format (C `b3DynamicTree_Save`). */
  tree_save(): Uint8Array;
  /** Replace the live tree with one loaded from `bytes`, scaling every AABB by `scale`
   *  (C `b3DynamicTree_Load( file, loadScale )`). Returns false on a bad/mismatched file. */
  tree_load(bytes: Uint8Array, scale: number): boolean;

  /** Rigid Body third-person camera boom raycast (C `RigidBodyCharacter::Step`): cast
   *  from the character toward the desired eye; returns the hit fraction in [0,1], or 1
   *  when the eye is reachable unobstructed. JS applies the C margins + radius restore. */
  character_camera_boom(
    fromX: number, fromY: number, fromZ: number,
    toX: number, toY: number, toZ: number,
  ): number;
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

/** Layout of `joint_revolute_energy()` — demo/wasm/src/joint_demo/mod.rs `joint_revolute_energy()`. */
export const REVOLUTE_ENERGY = {
  length: 3,
  kinetic: 0,
  potential: 1,
  total: 2,
} as const;

/** Layout of `joint_motor_readout()` — demo/wasm/src/joint_demo/basic.rs. */
export const MOTOR_READOUT = {
  length: 2,
  force: 0,
  torque: 1,
} as const;

/** Layout of `joint_door_readout()` — demo/wasm/src/joint_demo/structures.rs. */
export const DOOR_READOUT = {
  length: 6,
  error1: 0,
  error2: 1,
  hasTwo: 2,
  pointX: 3,
  pointY: 4,
  pointZ: 5,
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
