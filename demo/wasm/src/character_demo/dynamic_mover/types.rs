//! The plain data types of the vendored dynamic mover (dynamic_mover.h:8-53) and the
//! pogo ray cast callback (dynamic_mover.cpp:11-44).
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use box3d_rust::geometry::Capsule;
use box3d_rust::id::{BodyId, NULL_BODY_ID};
use box3d_rust::math_functions::{Pos, Vec3, POS_ZERO, VEC3_ZERO};
use box3d_rust::shape::{shape_get_body, shape_is_sensor};
use box3d_rust::types::{default_filter, Filter, QueryFilter};
use box3d_rust::world::{world_cast_ray, World};

/// C `DynamicMoverDef` (dynamic_mover.h:8), with the same member defaults.
#[derive(Clone)]
pub(crate) struct DynamicMoverDef {
    pub position: Pos,
    pub capsule: Capsule,

    pub density: f32,

    pub filter: Filter,
    pub jump_speed: f32,
    pub max_speed: f32,
    pub min_speed: f32,
    pub stop_speed: f32,
    pub accelerate: f32,
    pub air_steer: f32,
    pub friction: f32,
    pub gravity_scale: f32,
    pub max_ground_force: f32,
    pub max_air_force: f32,
    pub pogo_rest_length: f32,
    pub pogo_hertz: f32,
    pub pogo_damping_ratio: f32,
    pub pogo_compression_scale: f32,
    pub pogo_tension_scale: f32,
    pub platform_material_id: u64,
    pub enable_skirt_pogo: bool,
    pub enable_pre_solve_events: bool,
    pub enable_sensor_events: bool,
}

impl Default for DynamicMoverDef {
    fn default() -> Self {
        DynamicMoverDef {
            position: POS_ZERO,
            capsule: Capsule {
                center1: Vec3 {
                    x: 0.0,
                    y: -0.5,
                    z: 0.0,
                },
                center2: Vec3 {
                    x: 0.0,
                    y: 0.5,
                    z: 0.0,
                },
                radius: 0.3,
            },
            density: 200.0,
            filter: default_filter(),
            jump_speed: 7.0,
            max_speed: 6.0,
            min_speed: 0.1,
            stop_speed: 3.0,
            accelerate: 20.0,
            air_steer: 0.25,
            friction: 8.0,
            gravity_scale: 1.5,
            max_ground_force: 6000.0,
            max_air_force: 1500.0,
            pogo_rest_length: 0.9,
            pogo_hertz: 3.0,
            pogo_damping_ratio: 0.8,
            pogo_compression_scale: 100.0,
            pogo_tension_scale: 100.0,
            platform_material_id: 0,
            enable_skirt_pogo: true,
            enable_pre_solve_events: false,
            enable_sensor_events: false,
        }
    }
}

/// C `DynamicMoverCastResult` (dynamic_mover.h:38). Zero-initialized like C's `{}`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct DynamicMoverCastResult {
    pub point: Pos,
    pub normal: Vec3,
    pub body_id: BodyId,
    pub user_material_id: u64,
    pub fraction: f32,
    pub hit: bool,
}

impl Default for DynamicMoverCastResult {
    fn default() -> Self {
        DynamicMoverCastResult {
            point: POS_ZERO,
            normal: VEC3_ZERO,
            body_id: NULL_BODY_ID,
            user_material_id: 0,
            fraction: 0.0,
            hit: false,
        }
    }
}

/// C `DynamicMoverProbe` (dynamic_mover.h:48) — one pogo ray kept for drawing.
#[derive(Clone, Copy, Debug)]
pub(crate) struct DynamicMoverProbe {
    pub origin: Pos,
    pub translation: Vec3,
    pub fraction: f32,
    pub hit: bool,
}

/// C `DynamicMover::m_skirtCount`.
pub(crate) const SKIRT_COUNT: usize = 8;
/// C `DynamicMover::m_probeCapacity`.
pub(crate) const PROBE_CAPACITY: usize = SKIRT_COUNT + 1;

/// C `CastCallback` (dynamic_mover.cpp:17): keep the closest hit that is neither the
/// mover itself nor a sensor.
pub(super) fn cast_closest(
    world: &World,
    mover_id: BodyId,
    origin: Pos,
    translation: Vec3,
    filter: &QueryFilter,
) -> DynamicMoverCastResult {
    let mut result = DynamicMoverCastResult::default();
    world_cast_ray(
        world,
        origin,
        translation,
        filter,
        |shape_id, point, normal, fraction, user_material_id, _triangle_index, _child_index| {
            let body_id = shape_get_body(world, shape_id);

            // The probes start inside or on the capsule. Sensors can't hold up the mover.
            if body_id == mover_id || shape_is_sensor(world, shape_id) {
                return -1.0;
            }

            result.point = point;
            result.normal = normal;
            result.body_id = body_id;
            result.user_material_id = user_material_id;
            result.fraction = fraction;
            result.hit = true;
            fraction
        },
    );
    result
}
