// Joint definition types from include/box3d/types.h and their b3Default*
// constructors from src/joint.c.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::constants::huge;
use crate::core::{get_length_units_per_meter, SECRET_COOKIE};
use crate::id::BodyId;
use crate::math_functions::{
    Quat, Transform, Vec3, QUAT_IDENTITY, TRANSFORM_IDENTITY, VEC3_ZERO,
};

/// Base joint definition used by all joint types. The local frames are measured
/// from the body's origin rather than the center of mass because:
/// 1. You might not know where the center of mass will be.
/// 2. If you add/remove shapes from a body and recompute the mass, the joints
///    will be broken.
/// (b3JointDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointDef {
    /// User data
    pub user_data: u64,

    /// The first attached body
    pub body_id_a: BodyId,

    /// The second attached body
    pub body_id_b: BodyId,

    /// The first local joint frame
    pub local_frame_a: Transform,

    /// The second local joint frame
    pub local_frame_b: Transform,

    /// Force threshold for joint events
    pub force_threshold: f32,

    /// Torque threshold for joint events
    pub torque_threshold: f32,

    /// Constraint hertz (advanced feature)
    pub constraint_hertz: f32,

    /// Constraint damping ratio (advanced feature)
    pub constraint_damping_ratio: f32,

    /// Debug draw scale
    pub draw_scale: f32,

    /// Set this flag to true if the attached bodies should collide
    pub collide_connected: bool,

    /// Used internally to detect a valid definition. DO NOT SET.
    pub internal_value: i32,
}

/// (static b3DefaultJointDef)
pub(crate) fn default_joint_def() -> JointDef {
    JointDef {
        user_data: 0,
        body_id_a: BodyId::default(),
        body_id_b: BodyId::default(),
        local_frame_a: Transform {
            q: QUAT_IDENTITY,
            ..TRANSFORM_IDENTITY
        },
        local_frame_b: Transform {
            q: QUAT_IDENTITY,
            ..TRANSFORM_IDENTITY
        },
        force_threshold: f32::MAX,
        torque_threshold: f32::MAX,
        constraint_hertz: 60.0,
        constraint_damping_ratio: 2.0,
        draw_scale: get_length_units_per_meter(),
        collide_connected: false,
        internal_value: SECRET_COOKIE,
    }
}

/// Distance joint definition.
/// Connects a point on body A with a point on body B by a segment.
/// Useful for ropes and springs. (b3DistanceJointDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DistanceJointDef {
    pub base: JointDef,
    pub length: f32,
    pub enable_spring: bool,
    pub lower_spring_force: f32,
    pub upper_spring_force: f32,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub enable_limit: bool,
    pub min_length: f32,
    pub max_length: f32,
    pub enable_motor: bool,
    pub max_motor_force: f32,
    pub motor_speed: f32,
}

/// (b3DefaultDistanceJointDef)
pub fn default_distance_joint_def() -> DistanceJointDef {
    DistanceJointDef {
        base: default_joint_def(),
        length: 1.0,
        enable_spring: false,
        lower_spring_force: -f32::MAX,
        upper_spring_force: f32::MAX,
        hertz: 0.0,
        damping_ratio: 0.0,
        enable_limit: false,
        min_length: 0.0,
        max_length: huge(),
        enable_motor: false,
        max_motor_force: 0.0,
        motor_speed: 0.0,
    }
}

impl Default for DistanceJointDef {
    fn default() -> Self {
        default_distance_joint_def()
    }
}

/// A motor joint controls relative position and velocity. (b3MotorJointDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorJointDef {
    pub base: JointDef,
    pub linear_velocity: Vec3,
    pub max_velocity_force: f32,
    pub angular_velocity: Vec3,
    pub max_velocity_torque: f32,
    pub linear_hertz: f32,
    pub linear_damping_ratio: f32,
    pub max_spring_force: f32,
    pub angular_hertz: f32,
    pub angular_damping_ratio: f32,
    pub max_spring_torque: f32,
}

/// (b3DefaultMotorJointDef)
pub fn default_motor_joint_def() -> MotorJointDef {
    MotorJointDef {
        base: default_joint_def(),
        linear_velocity: VEC3_ZERO,
        max_velocity_force: 0.0,
        angular_velocity: VEC3_ZERO,
        max_velocity_torque: 0.0,
        linear_hertz: 0.0,
        linear_damping_ratio: 0.0,
        max_spring_force: 0.0,
        angular_hertz: 0.0,
        angular_damping_ratio: 0.0,
        max_spring_torque: 0.0,
    }
}

impl Default for MotorJointDef {
    fn default() -> Self {
        default_motor_joint_def()
    }
}

/// Disables collision between two specific bodies. (b3FilterJointDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FilterJointDef {
    pub base: JointDef,
}

/// (b3DefaultFilterJointDef)
pub fn default_filter_joint_def() -> FilterJointDef {
    FilterJointDef {
        base: default_joint_def(),
    }
}

impl Default for FilterJointDef {
    fn default() -> Self {
        default_filter_joint_def()
    }
}

/// Parallel joint: spring between body A z-axis and body B z-axis.
/// (b3ParallelJointDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParallelJointDef {
    pub base: JointDef,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub max_torque: f32,
}

/// (b3DefaultParallelJointDef)
pub fn default_parallel_joint_def() -> ParallelJointDef {
    ParallelJointDef {
        base: default_joint_def(),
        hertz: 1.0,
        damping_ratio: 1.0,
        max_torque: f32::MAX,
    }
}

impl Default for ParallelJointDef {
    fn default() -> Self {
        default_parallel_joint_def()
    }
}

/// Prismatic joint: body B slides along frame A x-axis. (b3PrismaticJointDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PrismaticJointDef {
    pub base: JointDef,
    pub enable_spring: bool,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub target_translation: f32,
    pub enable_limit: bool,
    pub lower_translation: f32,
    pub upper_translation: f32,
    pub enable_motor: bool,
    pub max_motor_force: f32,
    pub motor_speed: f32,
}

/// (b3DefaultPrismaticJointDef)
pub fn default_prismatic_joint_def() -> PrismaticJointDef {
    PrismaticJointDef {
        base: default_joint_def(),
        enable_spring: false,
        hertz: 0.0,
        damping_ratio: 0.0,
        target_translation: 0.0,
        enable_limit: false,
        lower_translation: 0.0,
        upper_translation: 0.0,
        enable_motor: false,
        max_motor_force: 0.0,
        motor_speed: 0.0,
    }
}

impl Default for PrismaticJointDef {
    fn default() -> Self {
        default_prismatic_joint_def()
    }
}

/// Revolute joint: point constraint with relative rotation about z.
/// (b3RevoluteJointDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RevoluteJointDef {
    pub base: JointDef,
    pub target_angle: f32,
    pub enable_spring: bool,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub enable_limit: bool,
    pub lower_angle: f32,
    pub upper_angle: f32,
    pub enable_motor: bool,
    pub max_motor_torque: f32,
    pub motor_speed: f32,
}

/// (b3DefaultRevoluteJointDef)
pub fn default_revolute_joint_def() -> RevoluteJointDef {
    RevoluteJointDef {
        base: default_joint_def(),
        target_angle: 0.0,
        enable_spring: false,
        hertz: 0.0,
        damping_ratio: 0.0,
        enable_limit: false,
        lower_angle: 0.0,
        upper_angle: 0.0,
        enable_motor: false,
        max_motor_torque: 0.0,
        motor_speed: 0.0,
    }
}

impl Default for RevoluteJointDef {
    fn default() -> Self {
        default_revolute_joint_def()
    }
}

/// Spherical joint: point constraint allowing free rotation. (b3SphericalJointDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SphericalJointDef {
    pub base: JointDef,
    pub enable_spring: bool,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub target_rotation: Quat,
    pub enable_cone_limit: bool,
    pub cone_angle: f32,
    pub enable_twist_limit: bool,
    pub lower_twist_angle: f32,
    pub upper_twist_angle: f32,
    pub enable_motor: bool,
    pub max_motor_torque: f32,
    pub motor_velocity: Vec3,
}

/// (b3DefaultSphericalJointDef)
pub fn default_spherical_joint_def() -> SphericalJointDef {
    SphericalJointDef {
        base: default_joint_def(),
        enable_spring: false,
        hertz: 0.0,
        damping_ratio: 0.0,
        target_rotation: QUAT_IDENTITY,
        enable_cone_limit: false,
        cone_angle: 0.0,
        enable_twist_limit: false,
        lower_twist_angle: 0.0,
        upper_twist_angle: 0.0,
        enable_motor: false,
        max_motor_torque: 0.0,
        motor_velocity: VEC3_ZERO,
    }
}

impl Default for SphericalJointDef {
    fn default() -> Self {
        default_spherical_joint_def()
    }
}

/// Weld joint: rigid connection with optional soft springs. (b3WeldJointDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WeldJointDef {
    pub base: JointDef,
    pub linear_hertz: f32,
    pub angular_hertz: f32,
    pub linear_damping_ratio: f32,
    pub angular_damping_ratio: f32,
}

/// (b3DefaultWeldJointDef)
pub fn default_weld_joint_def() -> WeldJointDef {
    WeldJointDef {
        base: default_joint_def(),
        linear_hertz: 0.0,
        angular_hertz: 0.0,
        linear_damping_ratio: 0.0,
        angular_damping_ratio: 0.0,
    }
}

impl Default for WeldJointDef {
    fn default() -> Self {
        default_weld_joint_def()
    }
}

/// Wheel joint: chassis A + wheel B with suspension and steering.
/// (b3WheelJointDef)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelJointDef {
    pub base: JointDef,
    pub enable_suspension_spring: bool,
    pub suspension_hertz: f32,
    pub suspension_damping_ratio: f32,
    pub enable_suspension_limit: bool,
    pub lower_suspension_limit: f32,
    pub upper_suspension_limit: f32,
    pub enable_spin_motor: bool,
    pub max_spin_torque: f32,
    pub spin_speed: f32,
    pub enable_steering: bool,
    pub steering_hertz: f32,
    pub steering_damping_ratio: f32,
    pub target_steering_angle: f32,
    pub max_steering_torque: f32,
    pub enable_steering_limit: bool,
    pub lower_steering_limit: f32,
    pub upper_steering_limit: f32,
}

/// (b3DefaultWheelJointDef)
pub fn default_wheel_joint_def() -> WheelJointDef {
    WheelJointDef {
        base: default_joint_def(),
        enable_suspension_spring: true,
        suspension_hertz: 1.0,
        suspension_damping_ratio: 0.7,
        enable_suspension_limit: false,
        lower_suspension_limit: 0.0,
        upper_suspension_limit: 0.0,
        enable_spin_motor: false,
        max_spin_torque: 0.0,
        spin_speed: 0.0,
        enable_steering: false,
        steering_hertz: 1.0,
        steering_damping_ratio: 0.7,
        target_steering_angle: 0.0,
        max_steering_torque: 0.0,
        enable_steering_limit: false,
        lower_steering_limit: 0.0,
        upper_steering_limit: 0.0,
    }
}

impl Default for WheelJointDef {
    fn default() -> Self {
        default_wheel_joint_def()
    }
}
