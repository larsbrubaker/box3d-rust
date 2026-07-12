//! Revolute/spherical/weld/wheel joint setter replay handlers,
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
        RecOp::RevoluteJointEnableSpring => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_spring = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::revolute_joint_enable_spring(
                    world,
                    rdr.make_joint_id(joint),
                    enable_spring,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::RevoluteJointSetSpringHertz => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::revolute_joint_set_spring_hertz(
                    world,
                    rdr.make_joint_id(joint),
                    hertz,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::RevoluteJointSetSpringDampingRatio => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::revolute_joint_set_spring_damping_ratio(
                    world,
                    rdr.make_joint_id(joint),
                    damping_ratio,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::RevoluteJointSetTargetAngle => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let angle = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::revolute_joint_set_target_angle(
                    world,
                    rdr.make_joint_id(joint),
                    angle,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::RevoluteJointEnableLimit => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_limit = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::revolute_joint_enable_limit(
                    world,
                    rdr.make_joint_id(joint),
                    enable_limit,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::RevoluteJointSetLimits => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let lower = s.f32();
            let upper = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::revolute_joint_set_limits(
                    world,
                    rdr.make_joint_id(joint),
                    lower,
                    upper,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::RevoluteJointEnableMotor => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_motor = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::revolute_joint_enable_motor(
                    world,
                    rdr.make_joint_id(joint),
                    enable_motor,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::RevoluteJointSetMotorSpeed => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let motor_speed = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::revolute_joint_set_motor_speed(
                    world,
                    rdr.make_joint_id(joint),
                    motor_speed,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::RevoluteJointSetMaxMotorTorque => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let torque = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::revolute_joint_set_max_motor_torque(
                    world,
                    rdr.make_joint_id(joint),
                    torque,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::SphericalJointEnableConeLimit => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_limit = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_enable_cone_limit(
                    world,
                    rdr.make_joint_id(joint),
                    enable_limit,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::SphericalJointSetConeLimit => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let angle_radians = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_set_cone_limit(
                    world,
                    rdr.make_joint_id(joint),
                    angle_radians,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::SphericalJointEnableTwistLimit => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_limit = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_enable_twist_limit(
                    world,
                    rdr.make_joint_id(joint),
                    enable_limit,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::SphericalJointSetTwistLimits => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let lower = s.f32();
            let upper = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_set_twist_limits(
                    world,
                    rdr.make_joint_id(joint),
                    lower,
                    upper,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::SphericalJointEnableSpring => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_spring = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_enable_spring(
                    world,
                    rdr.make_joint_id(joint),
                    enable_spring,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::SphericalJointSetSpringHertz => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_set_spring_hertz(
                    world,
                    rdr.make_joint_id(joint),
                    hertz,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::SphericalJointSetSpringDampingRatio => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_set_spring_damping_ratio(
                    world,
                    rdr.make_joint_id(joint),
                    damping_ratio,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::SphericalJointSetTargetRotation => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let target_rotation = s.quat();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_set_target_rotation(
                    world,
                    rdr.make_joint_id(joint),
                    target_rotation,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::SphericalJointEnableMotor => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_motor = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_enable_motor(
                    world,
                    rdr.make_joint_id(joint),
                    enable_motor,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::SphericalJointSetMotorVelocity => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let motor_velocity = s.vec3();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_set_motor_velocity(
                    world,
                    rdr.make_joint_id(joint),
                    motor_velocity,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::SphericalJointSetMaxMotorTorque => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let torque = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_set_max_motor_torque(
                    world,
                    rdr.make_joint_id(joint),
                    torque,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WeldJointSetLinearHertz => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::weld_joint_set_linear_hertz(world, rdr.make_joint_id(joint), hertz);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WeldJointSetLinearDampingRatio => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::weld_joint_set_linear_damping_ratio(
                    world,
                    rdr.make_joint_id(joint),
                    damping_ratio,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WeldJointSetAngularHertz => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::weld_joint_set_angular_hertz(world, rdr.make_joint_id(joint), hertz);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WeldJointSetAngularDampingRatio => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::weld_joint_set_angular_damping_ratio(
                    world,
                    rdr.make_joint_id(joint),
                    damping_ratio,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WheelJointEnableSuspension => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_enable_suspension(world, rdr.make_joint_id(joint), flag);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WheelJointSetSuspensionHertz => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_suspension_hertz(
                    world,
                    rdr.make_joint_id(joint),
                    hertz,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WheelJointSetSuspensionDampingRatio => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_suspension_damping_ratio(
                    world,
                    rdr.make_joint_id(joint),
                    damping_ratio,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WheelJointEnableSuspensionLimit => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_enable_suspension_limit(
                    world,
                    rdr.make_joint_id(joint),
                    flag,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WheelJointSetSuspensionLimits => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let lower = s.f32();
            let upper = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_suspension_limits(
                    world,
                    rdr.make_joint_id(joint),
                    lower,
                    upper,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WheelJointEnableSpinMotor => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_enable_spin_motor(world, rdr.make_joint_id(joint), flag);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WheelJointSetSpinMotorSpeed => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let speed = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_spin_motor_speed(
                    world,
                    rdr.make_joint_id(joint),
                    speed,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WheelJointSetMaxSpinTorque => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let torque = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_max_spin_torque(
                    world,
                    rdr.make_joint_id(joint),
                    torque,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WheelJointEnableSteering => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_enable_steering(world, rdr.make_joint_id(joint), flag);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WheelJointSetSteeringHertz => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_steering_hertz(
                    world,
                    rdr.make_joint_id(joint),
                    hertz,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WheelJointSetSteeringDampingRatio => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_steering_damping_ratio(
                    world,
                    rdr.make_joint_id(joint),
                    damping_ratio,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WheelJointSetMaxSteeringTorque => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let torque = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_max_steering_torque(
                    world,
                    rdr.make_joint_id(joint),
                    torque,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WheelJointEnableSteeringLimit => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_enable_steering_limit(
                    world,
                    rdr.make_joint_id(joint),
                    flag,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WheelJointSetSteeringLimits => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let lower = s.f32();
            let upper = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_steering_limits(
                    world,
                    rdr.make_joint_id(joint),
                    lower,
                    upper,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::WheelJointSetTargetSteeringAngle => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let radians = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_target_steering_angle(
                    world,
                    rdr.make_joint_id(joint),
                    radians,
                );
            }
            let _ = (payload_start, payload_size);
        }
        _ => return false,
    }
    true
}
