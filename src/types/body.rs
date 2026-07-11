// Body creation types and defaults from types.h / types.c.
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::{get_length_units_per_meter, SECRET_COOKIE};
use crate::math_functions::{Pos, Quat, Vec3, POS_ZERO, QUAT_IDENTITY, VEC3_ZERO};

/// The body simulation type. Each body is one of these three types.
/// (b3BodyType)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(i32)]
pub enum BodyType {
    /// zero mass, zero velocity, may be manually moved
    #[default]
    Static = 0,
    /// zero mass, velocity set by user, moved by solver
    Kinematic = 1,
    /// positive mass, velocity determined by forces, moved by solver
    Dynamic = 2,
}

/// Number of body types. (b3_bodyTypeCount)
pub const BODY_TYPE_COUNT: usize = 3;

/// Motion locks to restrict the body movement. (b3MotionLocks)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MotionLocks {
    /// Prevent translation along the x-axis
    pub linear_x: bool,
    /// Prevent translation along the y-axis
    pub linear_y: bool,
    /// Prevent translation along the z-axis
    pub linear_z: bool,
    /// Prevent rotation around the x-axis
    pub angular_x: bool,
    /// Prevent rotation around the y-axis
    pub angular_y: bool,
    /// Prevent rotation around the z-axis
    pub angular_z: bool,
}

/// A body definition holds all the data needed to construct a rigid body.
/// Must be initialized using [`default_body_def`]. (b3BodyDef)
#[derive(Debug, Clone, PartialEq)]
pub struct BodyDef {
    /// The body type: static, kinematic, or dynamic.
    pub type_: BodyType,
    /// The initial world position of the body.
    pub position: Pos,
    /// The initial world rotation of the body.
    pub rotation: Quat,
    /// The initial linear velocity of the body's origin, usually m/s.
    pub linear_velocity: Vec3,
    /// The initial angular velocity of the body, radians per second.
    pub angular_velocity: Vec3,
    /// Linear damping used to reduce the linear velocity.
    pub linear_damping: f32,
    /// Angular damping used to reduce the angular velocity.
    pub angular_damping: f32,
    /// Scale the gravity applied to this body. Non-dimensional.
    pub gravity_scale: f32,
    /// Sleep speed threshold, default is 0.05 meters per second.
    pub sleep_threshold: f32,
    /// Optional body name for debugging.
    pub name: String,
    /// Application specific body data.
    pub user_data: u64,
    /// Motion locks to restrict linear and angular movement.
    pub motion_locks: MotionLocks,
    /// Set to false if this body should never fall asleep.
    pub enable_sleep: bool,
    /// Is this body initially awake or sleeping?
    pub is_awake: bool,
    /// Treat this body as a high speed object for continuous collision.
    pub is_bullet: bool,
    /// Used to disable a body. A disabled body does not move or collide.
    pub is_enabled: bool,
    /// Allow this body to bypass rotational speed limits.
    pub allow_fast_rotation: bool,
    /// Enable contact recycling. True by default.
    pub enable_contact_recycling: bool,
    /// Used internally to detect a valid definition. DO NOT SET.
    pub internal_value: i32,
}

/// Initialize a body definition with the default values. (b3DefaultBodyDef)
pub fn default_body_def() -> BodyDef {
    BodyDef {
        type_: BodyType::Static,
        position: POS_ZERO,
        rotation: QUAT_IDENTITY,
        linear_velocity: VEC3_ZERO,
        angular_velocity: VEC3_ZERO,
        linear_damping: 0.0,
        angular_damping: 0.0,
        gravity_scale: 1.0,
        sleep_threshold: 0.05 * get_length_units_per_meter(),
        name: String::new(),
        user_data: 0,
        motion_locks: MotionLocks::default(),
        enable_sleep: true,
        is_awake: true,
        is_bullet: false,
        is_enabled: true,
        allow_fast_rotation: false,
        enable_contact_recycling: true,
        internal_value: SECRET_COOKIE,
    }
}

impl Default for BodyDef {
    fn default() -> Self {
        default_body_def()
    }
}
