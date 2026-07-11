//! The b3World_* public API from physics_world.c: validity, events, enable
//! flags, tuning setters, counters, callbacks, explosions, and the static-tree
//! rebuild. World queries (overlap/cast) live in query.rs.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{default_friction_callback, default_restitution_callback, CustomFilterFcn, PreSolveFcn, Profile, World};
use crate::body::{get_body_transform_quick, wake_body};
use crate::constants::GRAPH_COLOR_COUNT;
use crate::distance::{make_proxy, shape_distance, DistanceInput, SimplexCache};
use crate::events::{BodyMoveEvent, ContactEvents, JointEvent, SensorEvents};
use crate::math_functions::{
    add, clamp_float, cross, inv_transform_world_point, is_valid_float, is_valid_position,
    length_squared, max_int, mul_add, mul_mv, mul_sv, normalize, offset_aabb, rotate_vector, sub,
    Aabb, Pos, Vec3, TRANSFORM_IDENTITY,
};
use crate::shape::{get_shape_centroid, get_shape_projected_area, make_shape_proxy};
use crate::solver_set::{wake_solver_set, AWAKE_SET, FIRST_SLEEPING_SET};
use crate::types::{
    BodyType, Capacity, Counters, ExplosionDef, FrictionCallback, RestitutionCallback,
};

/// World id validity. (b3World_IsValid)
///
/// C validates the id against the global world registry; the registry-less
/// Rust port owns the `World`, so a reachable world is valid unless it has
/// been torn down.
pub fn world_is_valid(world: &World) -> bool {
    world.in_use
}

/// Get the body events for the current time step. The event data is transient.
/// Do not store a reference to this data. (b3World_GetBodyEvents)
pub fn world_get_body_events(world: &World) -> &[BodyMoveEvent] {
    debug_assert!(!world.locked);
    if world.locked {
        return &[];
    }

    &world.body_move_events
}

/// Get sensor events for the current time step. The event data is transient.
/// Do not store a reference to this data. (b3World_GetSensorEvents)
pub fn world_get_sensor_events(world: &World) -> SensorEvents<'_> {
    debug_assert!(!world.locked);
    if world.locked {
        return SensorEvents {
            begin_events: &[],
            end_events: &[],
        };
    }

    // Careful to use previous buffer
    let end_event_array_index = 1 - world.end_event_array_index;

    SensorEvents {
        begin_events: &world.sensor_begin_events,
        end_events: &world.sensor_end_events[end_event_array_index as usize],
    }
}

/// Get contact events for this current time step. The event data is transient.
/// Do not store a reference to this data. (b3World_GetContactEvents)
pub fn world_get_contact_events(world: &World) -> ContactEvents<'_> {
    debug_assert!(!world.locked);
    if world.locked {
        return ContactEvents {
            begin_events: &[],
            end_events: &[],
            hit_events: &[],
        };
    }

    // Careful to use previous buffer
    let end_event_array_index = 1 - world.end_event_array_index;

    ContactEvents {
        begin_events: &world.contact_begin_events,
        end_events: &world.contact_end_events[end_event_array_index as usize],
        hit_events: &world.contact_hit_events,
    }
}

/// Get the joint events for the current time step. The event data is
/// transient. Do not store a reference to this data. (b3World_GetJointEvents)
pub fn world_get_joint_events(world: &World) -> &[JointEvent] {
    debug_assert!(!world.locked);
    if world.locked {
        return &[];
    }

    &world.joint_events
}

/// Enable/disable sleep. (b3World_EnableSleeping)
pub fn world_enable_sleeping(world: &mut World, flag: bool) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    if flag == world.enable_sleep {
        return;
    }

    world.enable_sleep = flag;

    if !flag {
        let set_count = world.solver_sets.len() as i32;
        for i in FIRST_SLEEPING_SET..set_count {
            if !world.solver_sets[i as usize].body_sims.is_empty() {
                wake_solver_set(world, i);
            }
        }
    }
}

/// Is sleeping enabled? (b3World_IsSleepingEnabled)
pub fn world_is_sleeping_enabled(world: &World) -> bool {
    world.enable_sleep
}

/// Enable/disable constraint warm starting. (b3World_EnableWarmStarting)
pub fn world_enable_warm_starting(world: &mut World, flag: bool) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.enable_warm_starting = flag;
}

/// Is constraint warm starting enabled? (b3World_IsWarmStartingEnabled)
pub fn world_is_warm_starting_enabled(world: &World) -> bool {
    world.enable_warm_starting
}

/// Get the number of awake bodies. (b3World_GetAwakeBodyCount)
pub fn world_get_awake_body_count(world: &World) -> i32 {
    world.solver_sets[AWAKE_SET as usize].body_sims.len() as i32
}

/// Enable/disable continuous collision. (b3World_EnableContinuous)
pub fn world_enable_continuous(world: &mut World, flag: bool) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.enable_continuous = flag;
}

/// Is continuous collision enabled? (b3World_IsContinuousEnabled)
pub fn world_is_continuous_enabled(world: &World) -> bool {
    world.enable_continuous
}

/// Enable/disable speculative contacts. (b3World_EnableSpeculative)
pub fn world_enable_speculative(world: &mut World, flag: bool) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.enable_speculative = flag;
}

/// Is speculative contact enabled?
pub fn world_is_speculative_enabled(world: &World) -> bool {
    world.enable_speculative
}

/// Adjust the restitution threshold. (b3World_SetRestitutionThreshold)
pub fn world_set_restitution_threshold(world: &mut World, value: f32) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.restitution_threshold = clamp_float(value, 0.0, f32::MAX);
}

/// Get the restitution speed threshold. (b3World_GetRestitutionThreshold)
pub fn world_get_restitution_threshold(world: &World) -> f32 {
    world.restitution_threshold
}

/// Adjust the hit event threshold. (b3World_SetHitEventThreshold)
pub fn world_set_hit_event_threshold(world: &mut World, value: f32) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.hit_event_threshold = clamp_float(value, 0.0, f32::MAX);
}

/// Get the hit event speed threshold. (b3World_GetHitEventThreshold)
pub fn world_get_hit_event_threshold(world: &World) -> f32 {
    world.hit_event_threshold
}

/// Adjust contact tuning parameters. (b3World_SetContactTuning)
pub fn world_set_contact_tuning(
    world: &mut World,
    hertz: f32,
    damping_ratio: f32,
    contact_speed: f32,
) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.contact_hertz = clamp_float(hertz, 0.0, f32::MAX);
    world.contact_damping_ratio = clamp_float(damping_ratio, 0.0, f32::MAX);
    world.contact_speed = clamp_float(contact_speed, 0.0, f32::MAX);
}

/// Set the contact recycle distance. (b3World_SetContactRecycleDistance)
pub fn world_set_contact_recycle_distance(world: &mut World, recycle_distance: f32) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.contact_recycle_distance = clamp_float(recycle_distance, 0.0, f32::MAX);
}

/// Get the contact recycle distance. (b3World_GetContactRecycleDistance)
pub fn world_get_contact_recycle_distance(world: &World) -> f32 {
    world.contact_recycle_distance
}

/// Set the maximum linear speed. (b3World_SetMaximumLinearSpeed)
pub fn world_set_maximum_linear_speed(world: &mut World, maximum_linear_speed: f32) {
    debug_assert!(is_valid_float(maximum_linear_speed) && maximum_linear_speed > 0.0);

    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.max_linear_speed = maximum_linear_speed;
}

/// Get the maximum linear speed. (b3World_GetMaximumLinearSpeed)
pub fn world_get_maximum_linear_speed(world: &World) -> f32 {
    world.max_linear_speed
}

/// Get the current world performance profile. (b3World_GetProfile)
pub fn world_get_profile(world: &World) -> Profile {
    world.profile
}

/// Get world counters and sizes. (b3World_GetCounters)
pub fn world_get_counters(world: &World) -> Counters {
    let mut s = Counters {
        body_count: world.body_id_pool.id_count(),
        shape_count: world.shape_id_pool.id_count(),
        contact_count: world.contact_id_pool.id_count(),
        joint_count: world.joint_id_pool.id_count(),
        island_count: world.island_id_pool.id_count(),
        sat_call_count: world.sat_call_count,
        sat_cache_hit_count: world.sat_cache_hit_count,
        manifold_counts: world.manifold_counts,
        ..Default::default()
    };

    let static_tree = &world.broad_phase.trees[BodyType::Static as usize];
    s.static_tree_height = static_tree.height();

    let dynamic_tree = &world.broad_phase.trees[BodyType::Dynamic as usize];
    let kinematic_tree = &world.broad_phase.trees[BodyType::Kinematic as usize];
    s.tree_height = max_int(dynamic_tree.height(), kinematic_tree.height());

    // stack_used, byte_count, arena_capacity, and task_count stay zero.

    for i in 0..world.worker_count as usize {
        s.recycled_contact_count += world.task_contexts[i].recycled_contact_count;
        s.distance_iterations =
            max_int(s.distance_iterations, world.task_contexts[i].distance_iterations);
        s.push_back_iterations = max_int(
            s.push_back_iterations,
            world.task_contexts[i].push_back_iterations,
        );
        s.root_iterations =
            max_int(s.root_iterations, world.task_contexts[i].root_iterations);
    }

    for i in 0..GRAPH_COLOR_COUNT as usize {
        let color = &world.constraint_graph.colors[i];
        let color_contact_count =
            (color.convex_contacts.len() + color.contacts.len()) as i32;
        s.color_counts[i] = color_contact_count + color.joint_sims.len() as i32;
        s.awake_contact_count += color_contact_count;
    }
    s.awake_contact_count +=
        world.solver_sets[AWAKE_SET as usize].contact_indices.len() as i32;

    s
}

/// Get the maximum capacity the world has reached. (b3World_GetMaxCapacity)
pub fn world_get_max_capacity(world: &World) -> Capacity {
    world.max_capacity
}

/// Set the user data pointer. (b3World_SetUserData)
pub fn world_set_user_data(world: &mut World, user_data: u64) {
    world.user_data = user_data;
}

/// Get the user data pointer. (b3World_GetUserData)
pub fn world_get_user_data(world: &World) -> u64 {
    world.user_data
}

/// Register the friction callback. Passing `None` restores the default.
/// (b3World_SetFrictionCallback)
pub fn world_set_friction_callback(world: &mut World, callback: Option<FrictionCallback>) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.friction_callback = Some(callback.unwrap_or(default_friction_callback));
}

/// Register the restitution callback. Passing `None` restores the default.
/// (b3World_SetRestitutionCallback)
pub fn world_set_restitution_callback(world: &mut World, callback: Option<RestitutionCallback>) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.restitution_callback = Some(callback.unwrap_or(default_restitution_callback));
}

/// Register the custom filter callback. (b3World_SetCustomFilterCallback)
pub fn world_set_custom_filter_callback(
    world: &mut World,
    fcn: Option<CustomFilterFcn>,
    context: u64,
) {
    world.custom_filter_fcn = fcn;
    world.custom_filter_context = context;
}

/// Register the pre-solve callback. (b3World_SetPreSolveCallback)
pub fn world_set_pre_solve_callback(world: &mut World, fcn: Option<PreSolveFcn>, context: u64) {
    world.pre_solve_fcn = fcn;
    world.pre_solve_context = context;
}

/// Set the gravity vector. (b3World_SetGravity)
pub fn world_set_gravity(world: &mut World, gravity: Vec3) {
    world.gravity = gravity;
}

/// Get the gravity vector. (b3World_GetGravity)
pub fn world_get_gravity(world: &World) -> Vec3 {
    world.gravity
}

/// Rebuild the static broad-phase tree. (b3World_RebuildStaticTree)
pub fn world_rebuild_static_tree(world: &mut World) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.broad_phase.trees[BodyType::Static as usize].rebuild(true);
}

/// Apply a radial explosion. (b3World_Explode + static ExplosionCallback)
pub fn world_explode(world: &mut World, explosion_def: &ExplosionDef) {
    let mask_bits = explosion_def.mask_bits;
    let position = explosion_def.position;
    let radius = explosion_def.radius;
    let falloff = explosion_def.falloff;
    let impulse_per_area = explosion_def.impulse_per_area;

    debug_assert!(is_valid_position(position));
    debug_assert!(is_valid_float(radius) && radius >= 0.0);
    debug_assert!(is_valid_float(falloff) && falloff >= 0.0);
    debug_assert!(is_valid_float(impulse_per_area));

    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    // Locked due to waking
    world.locked = true;

    let extent = radius + falloff;
    let local_box = Aabb {
        lower_bound: Vec3 {
            x: -extent,
            y: -extent,
            z: -extent,
        },
        upper_bound: Vec3 {
            x: extent,
            y: extent,
            z: extent,
        },
    };
    let aabb = offset_aabb(local_box, position);

    // C applies impulses inside the tree traversal, but waking needs &mut World
    // while the tree is borrowed. Collect candidate shape ids first, then apply.
    let mut shape_ids: Vec<i32> = Vec::new();
    world.broad_phase.trees[BodyType::Dynamic as usize].query(
        aabb,
        mask_bits,
        false,
        |_, user_data| {
            shape_ids.push(user_data as i32);
            true
        },
    );

    for shape_id in shape_ids {
        explode_shape(world, shape_id, position, radius, falloff, impulse_per_area);
    }

    world.locked = false;
}

/// Apply the explosion impulse to one candidate shape. (static ExplosionCallback)
fn explode_shape(
    world: &mut World,
    shape_id: i32,
    position: Pos,
    radius: f32,
    falloff: f32,
    impulse_per_area: f32,
) {
    let shape = &world.shapes[shape_id as usize];
    if shape.explosion_scale == 0.0 {
        return;
    }

    let body_id = shape.body_id;
    let body = &world.bodies[body_id as usize];
    debug_assert!(body.type_ == BodyType::Dynamic);

    let xf = get_body_transform_quick(world, body);

    // Re-center the explosion into the shape local frame
    let local_position = inv_transform_world_point(xf, position);

    let input = DistanceInput {
        proxy_a: make_shape_proxy(shape),
        proxy_b: make_proxy(&[local_position], 0.0),
        transform: TRANSFORM_IDENTITY,
        use_radii: true,
    };

    let mut cache = SimplexCache::default();
    let output = shape_distance(&input, &mut cache, None);

    if output.distance > radius + falloff {
        return;
    }

    // Snapshot shape-derived values before waking (wake may reshuffle sims).
    let mut closest_point = output.point_a;
    if output.distance == 0.0 {
        closest_point = get_shape_centroid(shape);
    }

    let mut direction = sub(closest_point, local_position);
    if length_squared(direction) > 100.0 * f32::EPSILON * f32::EPSILON {
        direction = normalize(direction);
    } else {
        direction = Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        };
    }

    let area = get_shape_projected_area(shape, direction);
    let mut scale = 1.0;
    if output.distance > radius && falloff > 0.0 {
        scale = clamp_float((radius + falloff - output.distance) / falloff, 0.0, 1.0);
    }

    let magnitude = impulse_per_area * area * scale * shape.explosion_scale;
    let impulse = mul_sv(magnitude, rotate_vector(xf.q, direction));
    let closest_for_torque = closest_point;
    let direction_q = xf.q;

    wake_body(world, body_id);

    let body = &world.bodies[body_id as usize];
    if body.set_index != AWAKE_SET {
        return;
    }

    let local_index = body.local_index;
    let set = &mut world.solver_sets[AWAKE_SET as usize];
    // Copy inv_mass / local_center / inv_inertia before mutably borrowing body_states.
    let (inv_mass, local_center, inv_inertia_world) = {
        let body_sim = &set.body_sims[local_index as usize];
        (
            body_sim.inv_mass,
            body_sim.local_center,
            body_sim.inv_inertia_world,
        )
    };
    let state = &mut set.body_states[local_index as usize];
    state.linear_velocity = mul_add(state.linear_velocity, inv_mass, impulse);

    // Lever arm from the center of mass to the closest point, rotated to world
    let r = rotate_vector(direction_q, sub(closest_for_torque, local_center));
    state.angular_velocity = add(state.angular_velocity, mul_mv(inv_inertia_world, cross(r, impulse)));
}
