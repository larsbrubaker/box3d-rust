//! Framed joint op writers for `Recording`, split from ops.rs to satisfy
//! the file-length limit. Layouts mirror recording_ops.inl.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::id::JointId;
use crate::math_functions::{Quat, Vec3};

use crate::recording::ops::RecOp;
use crate::recording::session::Recording;

impl Recording {
    /// Write framed `ParallelJointSetSpringHertz` op.
    pub fn write_parallel_joint_set_spring_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::ParallelJointSetSpringHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `ParallelJointSetSpringDampingRatio` op.
    pub fn write_parallel_joint_set_spring_damping_ratio(
        &mut self,
        joint: JointId,
        damping_ratio: f32,
    ) {
        self.begin_record(RecOp::ParallelJointSetSpringDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `ParallelJointSetMaxTorque` op.
    pub fn write_parallel_joint_set_max_torque(&mut self, joint: JointId, max_torque: f32) {
        self.begin_record(RecOp::ParallelJointSetMaxTorque as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(max_torque);
        self.end_record();
    }

    /// Write framed `DistanceJointSetLength` op.
    pub fn write_distance_joint_set_length(&mut self, joint: JointId, length: f32) {
        self.begin_record(RecOp::DistanceJointSetLength as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(length);
        self.end_record();
    }

    /// Write framed `DistanceJointEnableSpring` op.
    pub fn write_distance_joint_enable_spring(&mut self, joint: JointId, enable_spring: bool) {
        self.begin_record(RecOp::DistanceJointEnableSpring as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_spring);
        self.end_record();
    }

    /// Write framed `DistanceJointSetSpringForceRange` op.
    pub fn write_distance_joint_set_spring_force_range(
        &mut self,
        joint: JointId,
        lower_force: f32,
        upper_force: f32,
    ) {
        self.begin_record(RecOp::DistanceJointSetSpringForceRange as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(lower_force);
        self.buffer.append_f32(upper_force);
        self.end_record();
    }

    /// Write framed `DistanceJointSetSpringHertz` op.
    pub fn write_distance_joint_set_spring_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::DistanceJointSetSpringHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `DistanceJointSetSpringDampingRatio` op.
    pub fn write_distance_joint_set_spring_damping_ratio(
        &mut self,
        joint: JointId,
        damping_ratio: f32,
    ) {
        self.begin_record(RecOp::DistanceJointSetSpringDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `DistanceJointEnableLimit` op.
    pub fn write_distance_joint_enable_limit(&mut self, joint: JointId, enable_limit: bool) {
        self.begin_record(RecOp::DistanceJointEnableLimit as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_limit);
        self.end_record();
    }

    /// Write framed `DistanceJointSetLengthRange` op.
    pub fn write_distance_joint_set_length_range(
        &mut self,
        joint: JointId,
        min_length: f32,
        max_length: f32,
    ) {
        self.begin_record(RecOp::DistanceJointSetLengthRange as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(min_length);
        self.buffer.append_f32(max_length);
        self.end_record();
    }

    /// Write framed `DistanceJointEnableMotor` op.
    pub fn write_distance_joint_enable_motor(&mut self, joint: JointId, enable_motor: bool) {
        self.begin_record(RecOp::DistanceJointEnableMotor as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_motor);
        self.end_record();
    }

    /// Write framed `DistanceJointSetMotorSpeed` op.
    pub fn write_distance_joint_set_motor_speed(&mut self, joint: JointId, motor_speed: f32) {
        self.begin_record(RecOp::DistanceJointSetMotorSpeed as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(motor_speed);
        self.end_record();
    }

    /// Write framed `DistanceJointSetMaxMotorForce` op.
    pub fn write_distance_joint_set_max_motor_force(&mut self, joint: JointId, force: f32) {
        self.begin_record(RecOp::DistanceJointSetMaxMotorForce as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(force);
        self.end_record();
    }

    /// Write framed `MotorJointSetLinearVelocity` op.
    pub fn write_motor_joint_set_linear_velocity(&mut self, joint: JointId, velocity: Vec3) {
        self.begin_record(RecOp::MotorJointSetLinearVelocity as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_vec3(velocity);
        self.end_record();
    }

    /// Write framed `MotorJointSetAngularVelocity` op.
    pub fn write_motor_joint_set_angular_velocity(&mut self, joint: JointId, velocity: Vec3) {
        self.begin_record(RecOp::MotorJointSetAngularVelocity as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_vec3(velocity);
        self.end_record();
    }

    /// Write framed `MotorJointSetMaxVelocityForce` op.
    pub fn write_motor_joint_set_max_velocity_force(&mut self, joint: JointId, max_force: f32) {
        self.begin_record(RecOp::MotorJointSetMaxVelocityForce as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(max_force);
        self.end_record();
    }

    /// Write framed `MotorJointSetMaxVelocityTorque` op.
    pub fn write_motor_joint_set_max_velocity_torque(&mut self, joint: JointId, max_torque: f32) {
        self.begin_record(RecOp::MotorJointSetMaxVelocityTorque as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(max_torque);
        self.end_record();
    }

    /// Write framed `MotorJointSetLinearHertz` op.
    pub fn write_motor_joint_set_linear_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::MotorJointSetLinearHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `MotorJointSetLinearDampingRatio` op.
    pub fn write_motor_joint_set_linear_damping_ratio(&mut self, joint: JointId, damping: f32) {
        self.begin_record(RecOp::MotorJointSetLinearDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping);
        self.end_record();
    }

    /// Write framed `MotorJointSetAngularHertz` op.
    pub fn write_motor_joint_set_angular_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::MotorJointSetAngularHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `MotorJointSetAngularDampingRatio` op.
    pub fn write_motor_joint_set_angular_damping_ratio(&mut self, joint: JointId, damping: f32) {
        self.begin_record(RecOp::MotorJointSetAngularDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping);
        self.end_record();
    }

    /// Write framed `MotorJointSetMaxSpringForce` op.
    pub fn write_motor_joint_set_max_spring_force(&mut self, joint: JointId, max_force: f32) {
        self.begin_record(RecOp::MotorJointSetMaxSpringForce as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(max_force);
        self.end_record();
    }

    /// Write framed `MotorJointSetMaxSpringTorque` op.
    pub fn write_motor_joint_set_max_spring_torque(&mut self, joint: JointId, max_torque: f32) {
        self.begin_record(RecOp::MotorJointSetMaxSpringTorque as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(max_torque);
        self.end_record();
    }

    /// Write framed `PrismaticJointEnableSpring` op.
    pub fn write_prismatic_joint_enable_spring(&mut self, joint: JointId, enable_spring: bool) {
        self.begin_record(RecOp::PrismaticJointEnableSpring as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_spring);
        self.end_record();
    }

    /// Write framed `PrismaticJointSetSpringHertz` op.
    pub fn write_prismatic_joint_set_spring_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::PrismaticJointSetSpringHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `PrismaticJointSetSpringDampingRatio` op.
    pub fn write_prismatic_joint_set_spring_damping_ratio(
        &mut self,
        joint: JointId,
        damping_ratio: f32,
    ) {
        self.begin_record(RecOp::PrismaticJointSetSpringDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `PrismaticJointSetTargetTranslation` op.
    pub fn write_prismatic_joint_set_target_translation(
        &mut self,
        joint: JointId,
        translation: f32,
    ) {
        self.begin_record(RecOp::PrismaticJointSetTargetTranslation as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(translation);
        self.end_record();
    }

    /// Write framed `PrismaticJointEnableLimit` op.
    pub fn write_prismatic_joint_enable_limit(&mut self, joint: JointId, enable_limit: bool) {
        self.begin_record(RecOp::PrismaticJointEnableLimit as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_limit);
        self.end_record();
    }

    /// Write framed `PrismaticJointSetLimits` op.
    pub fn write_prismatic_joint_set_limits(&mut self, joint: JointId, lower: f32, upper: f32) {
        self.begin_record(RecOp::PrismaticJointSetLimits as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(lower);
        self.buffer.append_f32(upper);
        self.end_record();
    }

    /// Write framed `PrismaticJointEnableMotor` op.
    pub fn write_prismatic_joint_enable_motor(&mut self, joint: JointId, enable_motor: bool) {
        self.begin_record(RecOp::PrismaticJointEnableMotor as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_motor);
        self.end_record();
    }

    /// Write framed `PrismaticJointSetMotorSpeed` op.
    pub fn write_prismatic_joint_set_motor_speed(&mut self, joint: JointId, motor_speed: f32) {
        self.begin_record(RecOp::PrismaticJointSetMotorSpeed as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(motor_speed);
        self.end_record();
    }

    /// Write framed `PrismaticJointSetMaxMotorForce` op.
    pub fn write_prismatic_joint_set_max_motor_force(&mut self, joint: JointId, force: f32) {
        self.begin_record(RecOp::PrismaticJointSetMaxMotorForce as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(force);
        self.end_record();
    }

    /// Write framed `RevoluteJointEnableSpring` op.
    pub fn write_revolute_joint_enable_spring(&mut self, joint: JointId, enable_spring: bool) {
        self.begin_record(RecOp::RevoluteJointEnableSpring as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_spring);
        self.end_record();
    }

    /// Write framed `RevoluteJointSetSpringHertz` op.
    pub fn write_revolute_joint_set_spring_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::RevoluteJointSetSpringHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `RevoluteJointSetSpringDampingRatio` op.
    pub fn write_revolute_joint_set_spring_damping_ratio(
        &mut self,
        joint: JointId,
        damping_ratio: f32,
    ) {
        self.begin_record(RecOp::RevoluteJointSetSpringDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `RevoluteJointSetTargetAngle` op.
    pub fn write_revolute_joint_set_target_angle(&mut self, joint: JointId, angle: f32) {
        self.begin_record(RecOp::RevoluteJointSetTargetAngle as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(angle);
        self.end_record();
    }

    /// Write framed `RevoluteJointEnableLimit` op.
    pub fn write_revolute_joint_enable_limit(&mut self, joint: JointId, enable_limit: bool) {
        self.begin_record(RecOp::RevoluteJointEnableLimit as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_limit);
        self.end_record();
    }

    /// Write framed `RevoluteJointSetLimits` op.
    pub fn write_revolute_joint_set_limits(&mut self, joint: JointId, lower: f32, upper: f32) {
        self.begin_record(RecOp::RevoluteJointSetLimits as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(lower);
        self.buffer.append_f32(upper);
        self.end_record();
    }

    /// Write framed `RevoluteJointEnableMotor` op.
    pub fn write_revolute_joint_enable_motor(&mut self, joint: JointId, enable_motor: bool) {
        self.begin_record(RecOp::RevoluteJointEnableMotor as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_motor);
        self.end_record();
    }

    /// Write framed `RevoluteJointSetMotorSpeed` op.
    pub fn write_revolute_joint_set_motor_speed(&mut self, joint: JointId, motor_speed: f32) {
        self.begin_record(RecOp::RevoluteJointSetMotorSpeed as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(motor_speed);
        self.end_record();
    }

    /// Write framed `RevoluteJointSetMaxMotorTorque` op.
    pub fn write_revolute_joint_set_max_motor_torque(&mut self, joint: JointId, torque: f32) {
        self.begin_record(RecOp::RevoluteJointSetMaxMotorTorque as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(torque);
        self.end_record();
    }

    /// Write framed `SphericalJointEnableConeLimit` op.
    pub fn write_spherical_joint_enable_cone_limit(&mut self, joint: JointId, enable_limit: bool) {
        self.begin_record(RecOp::SphericalJointEnableConeLimit as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_limit);
        self.end_record();
    }

    /// Write framed `SphericalJointSetConeLimit` op.
    pub fn write_spherical_joint_set_cone_limit(&mut self, joint: JointId, angle_radians: f32) {
        self.begin_record(RecOp::SphericalJointSetConeLimit as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(angle_radians);
        self.end_record();
    }

    /// Write framed `SphericalJointEnableTwistLimit` op.
    pub fn write_spherical_joint_enable_twist_limit(&mut self, joint: JointId, enable_limit: bool) {
        self.begin_record(RecOp::SphericalJointEnableTwistLimit as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_limit);
        self.end_record();
    }

    /// Write framed `SphericalJointSetTwistLimits` op.
    pub fn write_spherical_joint_set_twist_limits(
        &mut self,
        joint: JointId,
        lower: f32,
        upper: f32,
    ) {
        self.begin_record(RecOp::SphericalJointSetTwistLimits as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(lower);
        self.buffer.append_f32(upper);
        self.end_record();
    }

    /// Write framed `SphericalJointEnableSpring` op.
    pub fn write_spherical_joint_enable_spring(&mut self, joint: JointId, enable_spring: bool) {
        self.begin_record(RecOp::SphericalJointEnableSpring as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_spring);
        self.end_record();
    }

    /// Write framed `SphericalJointSetSpringHertz` op.
    pub fn write_spherical_joint_set_spring_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::SphericalJointSetSpringHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `SphericalJointSetSpringDampingRatio` op.
    pub fn write_spherical_joint_set_spring_damping_ratio(
        &mut self,
        joint: JointId,
        damping_ratio: f32,
    ) {
        self.begin_record(RecOp::SphericalJointSetSpringDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `SphericalJointSetTargetRotation` op.
    pub fn write_spherical_joint_set_target_rotation(
        &mut self,
        joint: JointId,
        target_rotation: Quat,
    ) {
        self.begin_record(RecOp::SphericalJointSetTargetRotation as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_quat(target_rotation);
        self.end_record();
    }

    /// Write framed `SphericalJointEnableMotor` op.
    pub fn write_spherical_joint_enable_motor(&mut self, joint: JointId, enable_motor: bool) {
        self.begin_record(RecOp::SphericalJointEnableMotor as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_motor);
        self.end_record();
    }

    /// Write framed `SphericalJointSetMotorVelocity` op.
    pub fn write_spherical_joint_set_motor_velocity(
        &mut self,
        joint: JointId,
        motor_velocity: Vec3,
    ) {
        self.begin_record(RecOp::SphericalJointSetMotorVelocity as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_vec3(motor_velocity);
        self.end_record();
    }

    /// Write framed `SphericalJointSetMaxMotorTorque` op.
    pub fn write_spherical_joint_set_max_motor_torque(&mut self, joint: JointId, torque: f32) {
        self.begin_record(RecOp::SphericalJointSetMaxMotorTorque as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(torque);
        self.end_record();
    }

    /// Write framed `WeldJointSetLinearHertz` op.
    pub fn write_weld_joint_set_linear_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::WeldJointSetLinearHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `WeldJointSetLinearDampingRatio` op.
    pub fn write_weld_joint_set_linear_damping_ratio(
        &mut self,
        joint: JointId,
        damping_ratio: f32,
    ) {
        self.begin_record(RecOp::WeldJointSetLinearDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `WeldJointSetAngularHertz` op.
    pub fn write_weld_joint_set_angular_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::WeldJointSetAngularHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `WeldJointSetAngularDampingRatio` op.
    pub fn write_weld_joint_set_angular_damping_ratio(
        &mut self,
        joint: JointId,
        damping_ratio: f32,
    ) {
        self.begin_record(RecOp::WeldJointSetAngularDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `WheelJointEnableSuspension` op.
    pub fn write_wheel_joint_enable_suspension(&mut self, joint: JointId, flag: bool) {
        self.begin_record(RecOp::WheelJointEnableSuspension as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `WheelJointSetSuspensionHertz` op.
    pub fn write_wheel_joint_set_suspension_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::WheelJointSetSuspensionHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `WheelJointSetSuspensionDampingRatio` op.
    pub fn write_wheel_joint_set_suspension_damping_ratio(
        &mut self,
        joint: JointId,
        damping_ratio: f32,
    ) {
        self.begin_record(RecOp::WheelJointSetSuspensionDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `WheelJointEnableSuspensionLimit` op.
    pub fn write_wheel_joint_enable_suspension_limit(&mut self, joint: JointId, flag: bool) {
        self.begin_record(RecOp::WheelJointEnableSuspensionLimit as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `WheelJointSetSuspensionLimits` op.
    pub fn write_wheel_joint_set_suspension_limits(
        &mut self,
        joint: JointId,
        lower: f32,
        upper: f32,
    ) {
        self.begin_record(RecOp::WheelJointSetSuspensionLimits as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(lower);
        self.buffer.append_f32(upper);
        self.end_record();
    }

    /// Write framed `WheelJointEnableSpinMotor` op.
    pub fn write_wheel_joint_enable_spin_motor(&mut self, joint: JointId, flag: bool) {
        self.begin_record(RecOp::WheelJointEnableSpinMotor as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `WheelJointSetSpinMotorSpeed` op.
    pub fn write_wheel_joint_set_spin_motor_speed(&mut self, joint: JointId, speed: f32) {
        self.begin_record(RecOp::WheelJointSetSpinMotorSpeed as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(speed);
        self.end_record();
    }

    /// Write framed `WheelJointSetMaxSpinTorque` op.
    pub fn write_wheel_joint_set_max_spin_torque(&mut self, joint: JointId, torque: f32) {
        self.begin_record(RecOp::WheelJointSetMaxSpinTorque as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(torque);
        self.end_record();
    }

    /// Write framed `WheelJointEnableSteering` op.
    pub fn write_wheel_joint_enable_steering(&mut self, joint: JointId, flag: bool) {
        self.begin_record(RecOp::WheelJointEnableSteering as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `WheelJointSetSteeringHertz` op.
    pub fn write_wheel_joint_set_steering_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::WheelJointSetSteeringHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `WheelJointSetSteeringDampingRatio` op.
    pub fn write_wheel_joint_set_steering_damping_ratio(
        &mut self,
        joint: JointId,
        damping_ratio: f32,
    ) {
        self.begin_record(RecOp::WheelJointSetSteeringDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `WheelJointSetMaxSteeringTorque` op.
    pub fn write_wheel_joint_set_max_steering_torque(&mut self, joint: JointId, torque: f32) {
        self.begin_record(RecOp::WheelJointSetMaxSteeringTorque as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(torque);
        self.end_record();
    }

    /// Write framed `WheelJointEnableSteeringLimit` op.
    pub fn write_wheel_joint_enable_steering_limit(&mut self, joint: JointId, flag: bool) {
        self.begin_record(RecOp::WheelJointEnableSteeringLimit as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `WheelJointSetSteeringLimits` op.
    pub fn write_wheel_joint_set_steering_limits(
        &mut self,
        joint: JointId,
        lower: f32,
        upper: f32,
    ) {
        self.begin_record(RecOp::WheelJointSetSteeringLimits as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(lower);
        self.buffer.append_f32(upper);
        self.end_record();
    }

    /// Write framed `WheelJointSetTargetSteeringAngle` op.
    pub fn write_wheel_joint_set_target_steering_angle(&mut self, joint: JointId, radians: f32) {
        self.begin_record(RecOp::WheelJointSetTargetSteeringAngle as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(radians);
        self.end_record();
    }
}
