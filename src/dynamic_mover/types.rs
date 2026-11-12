//! The plain data types of the dynamic mover (`samples/dynamic_mover.h:8-53`) and the
//! pogo ray cast callback (`samples/dynamic_mover.cpp:11-44`).
//!
//! [`DynamicMoverDef`] keeps the C member defaults and adds three port fields a game needs
//! and the sample hard-codes: the joint anchor body, the query filter, and the capsule's
//! custom colour. Their defaults reproduce the sample exactly.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::debug_draw::HexColor;
use crate::geometry::Capsule;
use crate::id::{BodyId, NULL_BODY_ID};
use crate::math_functions::{Pos, Vec3, POS_ZERO, VEC3_ZERO};
use crate::shape::{shape_get_body, shape_is_sensor};
use crate::types::{default_filter, Filter, QueryFilter};
use crate::world::{world_cast_ray, World};

/// C `DynamicMoverDef` (dynamic_mover.h:8), with the same member defaults.
#[derive(Clone, Debug)]
pub struct DynamicMoverDef {
    /// World position of the capsule body origin (the capsule's center for the default
    /// capsule).
    pub position: Pos,
    /// The capsule in body space. `center1` is the lower end: the pogo rays start there.
    pub capsule: Capsule,

    /// Capsule density in kg/m^3.
    pub density: f32,

    /// Collision filter of the capsule shape.
    pub filter: Filter,
    /// Vertical speed a jump reaches in m/s.
    pub jump_speed: f32,
    /// Top horizontal speed in m/s at a speed scale of 1.
    pub max_speed: f32,
    /// Below this ground speed in m/s the mover stops dead.
    pub min_speed: f32,
    /// Below this ground speed in m/s friction removes a fixed amount instead of a fraction.
    pub stop_speed: f32,
    /// Ground acceleration as a multiple of the top speed per second.
    pub accelerate: f32,
    /// Fraction of the ground acceleration available in the air.
    pub air_steer: f32,
    /// Ground friction in units of 1/time.
    pub friction: f32,
    /// Gravity scale of the capsule body (a character usually falls faster than the world).
    pub gravity_scale: f32,
    /// Force budget in newtons the mover joint has on the ground.
    pub max_ground_force: f32,
    /// Force budget in newtons the mover joint has in the air.
    pub max_air_force: f32,
    /// Pogo spring rest length in metres, measured down from `capsule.center1`.
    pub pogo_rest_length: f32,
    /// Pogo spring stiffness in hertz.
    pub pogo_hertz: f32,
    /// Pogo spring damping ratio, non-dimensional.
    pub pogo_damping_ratio: f32,
    /// Pogo compression force limit as a multiple of the mover weight.
    pub pogo_compression_scale: f32,
    /// Pogo tension force limit as a multiple of the mover weight.
    pub pogo_tension_scale: f32,
    /// User material id of kinematic platforms the mover rides. 0 disables riding.
    pub platform_material_id: u64,
    /// Cast an octagon skirt of rays when the central ray misses or lands on a steep surface.
    pub enable_skirt_pogo: bool,
    /// Pre-solve events for the capsule shape.
    pub enable_pre_solve_events: bool,
    /// Sensor events for the capsule shape.
    pub enable_sensor_events: bool,

    /// Port addition: the static body the mover joint is anchored to while the mover is not
    /// riding a platform. Many movers can share one anchor, so a crowd adds one body, not
    /// one per mover. [`NULL_BODY_ID`] (the default) makes `create` build a private static
    /// anchor at the origin, as the C sample does, and `destroy` remove it again. A shared
    /// anchor must be a static body and outlive every mover that uses it.
    pub anchor_id: BodyId,
    /// Port addition: the filter of the pogo ray casts. `None` (the default) casts with the
    /// capsule's `filter` category and mask bits, as the C sample does.
    pub query_filter: Option<QueryFilter>,
    /// Port addition: the capsule shape's `base_material.custom_color`. The default is the C
    /// sample's dodger blue.
    pub custom_color: u32,
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
            anchor_id: NULL_BODY_ID,
            query_filter: None,
            custom_color: HexColor::DODGER_BLUE.0,
        }
    }
}

/// C `DynamicMoverCastResult` (dynamic_mover.h:38). Zero-initialized like C's `{}`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicMoverCastResult {
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

/// C `DynamicMoverProbe` (dynamic_mover.h:48): one pogo ray of the last update, kept so a
/// caller can draw it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicMoverProbe {
    pub origin: Pos,
    pub translation: Vec3,
    pub fraction: f32,
    pub hit: bool,
}

impl Default for DynamicMoverProbe {
    fn default() -> Self {
        DynamicMoverProbe {
            origin: POS_ZERO,
            translation: VEC3_ZERO,
            fraction: 1.0,
            hit: false,
        }
    }
}

/// C `DynamicMover::m_skirtCount`.
pub const SKIRT_COUNT: usize = 8;
/// C `DynamicMover::m_probeCapacity`.
pub const PROBE_CAPACITY: usize = SKIRT_COUNT + 1;

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
