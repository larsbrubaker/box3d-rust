// World creation types and defaults from types.h / types.c.
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::{get_length_units_per_meter, SECRET_COOKIE};
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
        internal_value: SECRET_COOKIE,
    }
}

impl Default for WorldDef {
    fn default() -> Self {
        default_world_def()
    }
}
