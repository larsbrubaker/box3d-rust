// World creation types and defaults from types.h / types.c.
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::{get_length_units_per_meter, SECRET_COOKIE};
use crate::debug_draw::{CreateDebugShapeCallback, DestroyDebugShapeCallback};
use crate::math_functions::Vec3;

/// Optional world capacities that can be used to avoid run-time allocations.
/// (b3Capacity)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Capacity {
    /// Number of expected static shapes.
    pub static_shape_count: i32,
    /// Number of expected dynamic and kinematic shapes.
    pub dynamic_shape_count: i32,
    /// Number of expected static bodies.
    pub static_body_count: i32,
    /// Number of expected dynamic and kinematic bodies.
    pub dynamic_body_count: i32,
    /// Number of expected contacts.
    pub contact_count: i32,
}

/// Optional friction mixing callback. (b3FrictionCallback)
///
/// Args: `(friction_a, user_material_id_a, friction_b, user_material_id_b)`.
pub type FrictionCallback = fn(f32, u64, f32, u64) -> f32;

/// Optional restitution mixing callback. (b3RestitutionCallback)
pub type RestitutionCallback = fn(f32, u64, f32, u64) -> f32;

/// World definition used to create a simulation world.
/// Must be initialized using [`default_world_def`]. (b3WorldDef)
///
/// Task-system fields from C (`workerCount`, enqueue/finish callbacks) are
/// omitted: the Rust port is serial and never spawns workers.
#[derive(Debug, Clone)]
pub struct WorldDef {
    /// Gravity vector. Box3D has no up-vector defined.
    pub gravity: Vec3,
    /// Restitution speed threshold, usually in m/s.
    pub restitution_threshold: f32,
    /// Hit event speed threshold, usually in m/s.
    pub hit_event_threshold: f32,
    /// Contact stiffness. Cycles per second.
    pub contact_hertz: f32,
    /// Contact bounciness. Non-dimensional.
    pub contact_damping_ratio: f32,
    /// Contact speed cap for overlap resolution, usually m/s.
    pub contact_speed: f32,
    /// Maximum linear speed, usually m/s.
    pub maximum_linear_speed: f32,
    /// Optional friction mixing callback.
    pub friction_callback: Option<FrictionCallback>,
    /// Optional restitution mixing callback.
    pub restitution_callback: Option<RestitutionCallback>,
    /// Can bodies go to sleep?
    pub enable_sleep: bool,
    /// Enable continuous collision.
    pub enable_continuous: bool,
    /// Application-specific world data.
    pub user_data: u64,
    /// Optional capacity hints to avoid run-time allocations.
    pub capacity: Capacity,
    /// Used to create debug draw shapes when a shape is first drawn.
    pub create_debug_shape: Option<CreateDebugShapeCallback>,
    /// Used to destroy debug draw shapes when a shape is modified or destroyed.
    pub destroy_debug_shape: Option<DestroyDebugShapeCallback>,
    /// Passed to the debug shape callbacks.
    pub user_debug_shape_context: u64,
    /// Used internally to detect a valid definition. DO NOT SET.
    pub internal_value: i32,
}

/// Use this to initialize your world definition. (b3DefaultWorldDef)
pub fn default_world_def() -> WorldDef {
    let length_units = get_length_units_per_meter();
    WorldDef {
        gravity: Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        },
        restitution_threshold: 1.0 * length_units,
        hit_event_threshold: 1.0 * length_units,
        contact_hertz: 30.0,
        contact_damping_ratio: 10.0,
        contact_speed: 3.0 * length_units,
        maximum_linear_speed: 400.0 * length_units,
        friction_callback: None,
        restitution_callback: None,
        enable_sleep: true,
        enable_continuous: true,
        user_data: 0,
        capacity: Capacity::default(),
        create_debug_shape: None,
        destroy_debug_shape: None,
        user_debug_shape_context: 0,
        internal_value: SECRET_COOKIE,
    }
}

impl Default for WorldDef {
    fn default() -> Self {
        default_world_def()
    }
}

/// World-space cast output from a single-shape ray cast. (b3WorldCastOutput)
///
/// In single precision this matches [`crate::distance::CastOutput`] field-wise
/// except `point` is [`Pos`] (an alias of [`Vec3`]). With `double-precision`,
/// `point` stays a world [`Pos`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorldCastOutput {
    /// The surface normal at the hit point.
    pub normal: Vec3,
    /// The surface hit point in world space.
    pub point: crate::math_functions::Pos,
    /// The fraction of the input translation at collision.
    pub fraction: f32,
    /// The number of iterations used.
    pub iterations: i32,
    /// The index of the mesh or height field triangle hit.
    pub triangle_index: i32,
    /// The index of the compound child shape.
    pub child_index: i32,
    /// The material index. May be -1 for null.
    pub material_index: i32,
    /// Did the cast hit?
    pub hit: bool,
}

impl Default for WorldCastOutput {
    fn default() -> Self {
        WorldCastOutput {
            normal: crate::math_functions::VEC3_ZERO,
            point: crate::math_functions::POS_ZERO,
            fraction: 0.0,
            iterations: 0,
            triangle_index: crate::core::NULL_INDEX,
            child_index: 0,
            material_index: 0,
            hit: false,
        }
    }
}

/// Result from b3World_CastRayClosest. (b3RayResult)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayResult {
    /// The shape hit.
    pub shape_id: crate::id::ShapeId,
    /// The world point of the hit.
    pub point: crate::math_functions::Pos,
    /// The world normal of the shape surface at the hit point.
    pub normal: Vec3,
    /// The user material id at the hit point.
    pub user_material_id: u64,
    /// The fraction of the input ray.
    pub fraction: f32,
    /// The triangle index if the shape is a mesh, height-field, or compound with child mesh.
    pub triangle_index: i32,
    /// The child index if the shape is a compound.
    pub child_index: i32,
    /// The number of BVH nodes visited. Diagnostic.
    pub node_visits: i32,
    /// The number of BVH leaves visited. Diagnostic.
    pub leaf_visits: i32,
    /// Did the ray hit? If false, all other data is invalid.
    pub hit: bool,
}

impl Default for RayResult {
    fn default() -> Self {
        RayResult {
            shape_id: crate::id::ShapeId::default(),
            point: crate::math_functions::POS_ZERO,
            normal: crate::math_functions::VEC3_ZERO,
            user_material_id: 0,
            fraction: 0.0,
            triangle_index: 0,
            child_index: 0,
            node_visits: 0,
            leaf_visits: 0,
            hit: false,
        }
    }
}

/// Counters that give details of the simulation size. (b3Counters)
///
/// byte_count, stack_used, arena_capacity, and task_count are always
/// zero in this port: there is no global allocation tracker, no arena stack
/// allocator, and no task system in the serial Rust implementation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Counters {
    pub body_count: i32,
    pub shape_count: i32,
    pub contact_count: i32,
    pub joint_count: i32,
    pub island_count: i32,
    pub stack_used: i32,
    pub arena_capacity: i32,
    pub static_tree_height: i32,
    pub tree_height: i32,
    pub sat_call_count: i32,
    pub sat_cache_hit_count: i32,
    pub byte_count: i32,
    pub task_count: i32,
    pub color_counts: [i32; crate::constants::GRAPH_COLOR_COUNT as usize],
    pub manifold_counts: [i32; crate::constants::CONTACT_MANIFOLD_COUNT_BUCKETS],
    /// Number of contacts touched by the collide pass (graph contacts +
    /// awake-set non-touching).
    pub awake_contact_count: i32,
    /// Number of contacts recycled in the most recent step.
    pub recycled_contact_count: i32,
    /// Maximum number of time of impact iterations
    pub distance_iterations: i32,
    pub push_back_iterations: i32,
    pub root_iterations: i32,
}

/// The explosion definition is used to configure options for explosions.
/// Explosions consider shape geometry when computing the impulse.
/// (b3ExplosionDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExplosionDef {
    /// Mask bits to filter shapes
    pub mask_bits: u64,
    /// The center of the explosion in world space
    pub position: crate::math_functions::Pos,
    /// The radius of the explosion
    pub radius: f32,
    /// The falloff distance beyond the radius. Impulse is reduced to zero at
    /// this distance.
    pub falloff: f32,
    /// Impulse per unit area. This applies an impulse according to the shape
    /// area that is facing the explosion. Explosions only apply to spheres,
    /// capsules, and hulls. This may be negative for implosions.
    pub impulse_per_area: f32,
}

/// Use this to initialize your explosion definition. (b3DefaultExplosionDef)
pub fn default_explosion_def() -> ExplosionDef {
    ExplosionDef {
        mask_bits: crate::dynamic_tree::DEFAULT_MASK_BITS,
        position: crate::math_functions::POS_ZERO,
        radius: 0.0,
        falloff: 0.0,
        impulse_per_area: 0.0,
    }
}

impl Default for ExplosionDef {
    fn default() -> Self {
        default_explosion_def()
    }
}
