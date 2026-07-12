//! Parallel/distance/motor/prismatic joint setter replay handlers,
//! dispatch.rs to satisfy the file-length limit. One function per op family;
//! returns false when the op belongs to another family.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::recording::dispatch::RecReader;
use crate::recording::ops::RecOp;

#[allow(unused_imports)]
use crate::recording::query_replay;

/// Handle one joint_type op. Returns false if `op` is not in this family.
pub(super) fn dispatch(
    op: RecOp,
    rdr: &mut RecReader<'_>,
    payload_start: i32,
    payload_size: u32,
) -> bool {
    let world = unsafe { &mut *rdr.world };
    let _ = world;
    match op {
        RecOp::ParallelJointSetSpringHertz => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::parallel_joint_set_spring_hertz(
                    world,
                    rdr.make_joint_id(joint),
                    hertz,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::ParallelJointSetSpringDampingRatio => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::parallel_joint_set_spring_damping_ratio(
                    world,
                    rdr.make_joint_id(joint),
                    damping_ratio,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::ParallelJointSetMaxTorque => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let max_torque = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::parallel_joint_set_max_torque(
                    world,
                    rdr.make_joint_id(joint),
                    max_torque,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::DistanceJointSetLength => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let length = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_set_length(world, rdr.make_joint_id(joint), length);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::DistanceJointEnableSpring => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_spring = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_enable_spring(
                    world,
                    rdr.make_joint_id(joint),
                    enable_spring,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::DistanceJointSetSpringForceRange => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let lower_force = s.f32();
            let upper_force = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_set_spring_force_range(
                    world,
                    rdr.make_joint_id(joint),
                    lower_force,
                    upper_force,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::DistanceJointSetSpringHertz => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_set_spring_hertz(
                    world,
                    rdr.make_joint_id(joint),
                    hertz,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::DistanceJointSetSpringDampingRatio => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_set_spring_damping_ratio(
                    world,
                    rdr.make_joint_id(joint),
                    damping_ratio,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::DistanceJointEnableLimit => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_limit = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_enable_limit(
                    world,
                    rdr.make_joint_id(joint),
                    enable_limit,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::DistanceJointSetLengthRange => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let min_length = s.f32();
            let max_length = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_set_length_range(
                    world,
                    rdr.make_joint_id(joint),
                    min_length,
                    max_length,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::DistanceJointEnableMotor => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_motor = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_enable_motor(
                    world,
                    rdr.make_joint_id(joint),
                    enable_motor,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::DistanceJointSetMotorSpeed => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let motor_speed = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_set_motor_speed(
                    world,
                    rdr.make_joint_id(joint),
                    motor_speed,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::DistanceJointSetMaxMotorForce => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let force = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_set_max_motor_force(
                    world,
                    rdr.make_joint_id(joint),
                    force,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::MotorJointSetLinearVelocity => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let velocity = s.vec3();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_linear_velocity(
                    world,
                    rdr.make_joint_id(joint),
                    velocity,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::MotorJointSetAngularVelocity => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let velocity = s.vec3();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_angular_velocity(
                    world,
                    rdr.make_joint_id(joint),
                    velocity,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::MotorJointSetMaxVelocityForce => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let max_force = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_max_velocity_force(
                    world,
                    rdr.make_joint_id(joint),
                    max_force,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::MotorJointSetMaxVelocityTorque => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let max_torque = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_max_velocity_torque(
                    world,
                    rdr.make_joint_id(joint),
                    max_torque,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::MotorJointSetLinearHertz => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_linear_hertz(world, rdr.make_joint_id(joint), hertz);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::MotorJointSetLinearDampingRatio => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_linear_damping_ratio(
                    world,
                    rdr.make_joint_id(joint),
                    damping,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::MotorJointSetAngularHertz => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_angular_hertz(world, rdr.make_joint_id(joint), hertz);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::MotorJointSetAngularDampingRatio => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_angular_damping_ratio(
                    world,
                    rdr.make_joint_id(joint),
                    damping,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::MotorJointSetMaxSpringForce => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let max_force = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_max_spring_force(
                    world,
                    rdr.make_joint_id(joint),
                    max_force,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::MotorJointSetMaxSpringTorque => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let max_torque = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_max_spring_torque(
                    world,
                    rdr.make_joint_id(joint),
                    max_torque,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::PrismaticJointEnableSpring => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_spring = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::prismatic_joint_enable_spring(
                    world,
                    rdr.make_joint_id(joint),
                    enable_spring,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::PrismaticJointSetSpringHertz => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::prismatic_joint_set_spring_hertz(
                    world,
                    rdr.make_joint_id(joint),
                    hertz,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::PrismaticJointSetSpringDampingRatio => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::prismatic_joint_set_spring_damping_ratio(
                    world,
                    rdr.make_joint_id(joint),
                    damping_ratio,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::PrismaticJointSetTargetTranslation => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let translation = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::prismatic_joint_set_target_translation(
                    world,
                    rdr.make_joint_id(joint),
                    translation,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::PrismaticJointEnableLimit => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_limit = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::prismatic_joint_enable_limit(
                    world,
                    rdr.make_joint_id(joint),
                    enable_limit,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::PrismaticJointSetLimits => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let lower = s.f32();
            let upper = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::prismatic_joint_set_limits(
                    world,
                    rdr.make_joint_id(joint),
                    lower,
                    upper,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::PrismaticJointEnableMotor => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_motor = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::prismatic_joint_enable_motor(
                    world,
                    rdr.make_joint_id(joint),
                    enable_motor,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::PrismaticJointSetMotorSpeed => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let motor_speed = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::prismatic_joint_set_motor_speed(
                    world,
                    rdr.make_joint_id(joint),
                    motor_speed,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::PrismaticJointSetMaxMotorForce => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let force = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::prismatic_joint_set_max_motor_force(
                    world,
                    rdr.make_joint_id(joint),
                    force,
                );
            }
            let _ = (payload_start, payload_size);
        }
        _ => return false,
    }
    true
}
