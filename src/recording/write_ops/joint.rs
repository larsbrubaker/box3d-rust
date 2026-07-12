//! Framed joint op writers for `Recording`, split from ops.rs to satisfy
//! the file-length limit. Layouts mirror recording_ops.inl.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::id::{JointId, WorldId};
use crate::math_functions::Transform;
use crate::types::{
    DistanceJointDef, FilterJointDef, MotorJointDef, ParallelJointDef, PrismaticJointDef,
    RevoluteJointDef, SphericalJointDef, WeldJointDef, WheelJointDef,
};

use crate::recording::ops::RecOp;
use crate::recording::session::Recording;

impl Recording {
    /// Write framed `CreateParallelJoint` op.
    pub fn write_create_parallel_joint(
        &mut self,
        world: WorldId,
        def: &ParallelJointDef,
        ret_id: JointId,
    ) {
        self.begin_record(RecOp::CreateParallelJoint as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_parallel_joint_def(def);
        self.buffer.append_joint_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateDistanceJoint` op.
    pub fn write_create_distance_joint(
        &mut self,
        world: WorldId,
        def: &DistanceJointDef,
        ret_id: JointId,
    ) {
        self.begin_record(RecOp::CreateDistanceJoint as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_distance_joint_def(def);
        self.buffer.append_joint_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateFilterJoint` op.
    pub fn write_create_filter_joint(
        &mut self,
        world: WorldId,
        def: &FilterJointDef,
        ret_id: JointId,
    ) {
        self.begin_record(RecOp::CreateFilterJoint as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_filter_joint_def(def);
        self.buffer.append_joint_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateMotorJoint` op.
    pub fn write_create_motor_joint(
        &mut self,
        world: WorldId,
        def: &MotorJointDef,
        ret_id: JointId,
    ) {
        self.begin_record(RecOp::CreateMotorJoint as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_motor_joint_def(def);
        self.buffer.append_joint_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreatePrismaticJoint` op.
    pub fn write_create_prismatic_joint(
        &mut self,
        world: WorldId,
        def: &PrismaticJointDef,
        ret_id: JointId,
    ) {
        self.begin_record(RecOp::CreatePrismaticJoint as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_prismatic_joint_def(def);
        self.buffer.append_joint_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateRevoluteJoint` op.
    pub fn write_create_revolute_joint(
        &mut self,
        world: WorldId,
        def: &RevoluteJointDef,
        ret_id: JointId,
    ) {
        self.begin_record(RecOp::CreateRevoluteJoint as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_revolute_joint_def(def);
        self.buffer.append_joint_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateSphericalJoint` op.
    pub fn write_create_spherical_joint(
        &mut self,
        world: WorldId,
        def: &SphericalJointDef,
        ret_id: JointId,
    ) {
        self.begin_record(RecOp::CreateSphericalJoint as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_spherical_joint_def(def);
        self.buffer.append_joint_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateWeldJoint` op.
    pub fn write_create_weld_joint(&mut self, world: WorldId, def: &WeldJointDef, ret_id: JointId) {
        self.begin_record(RecOp::CreateWeldJoint as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_weld_joint_def(def);
        self.buffer.append_joint_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateWheelJoint` op.
    pub fn write_create_wheel_joint(
        &mut self,
        world: WorldId,
        def: &WheelJointDef,
        ret_id: JointId,
    ) {
        self.begin_record(RecOp::CreateWheelJoint as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_wheel_joint_def(def);
        self.buffer.append_joint_id(ret_id);
        self.end_record();
    }

    /// Write framed `DestroyJoint` op.
    pub fn write_destroy_joint(&mut self, joint: JointId, wake_attached: bool) {
        self.begin_record(RecOp::DestroyJoint as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(wake_attached);
        self.end_record();
    }

    /// Write framed `JointSetLocalFrameA` op.
    pub fn write_joint_set_local_frame_a(&mut self, joint: JointId, local_frame: Transform) {
        self.begin_record(RecOp::JointSetLocalFrameA as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_transform(local_frame);
        self.end_record();
    }

    /// Write framed `JointSetLocalFrameB` op.
    pub fn write_joint_set_local_frame_b(&mut self, joint: JointId, local_frame: Transform) {
        self.begin_record(RecOp::JointSetLocalFrameB as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_transform(local_frame);
        self.end_record();
    }

    /// Write framed `JointSetCollideConnected` op.
    pub fn write_joint_set_collide_connected(&mut self, joint: JointId, should_collide: bool) {
        self.begin_record(RecOp::JointSetCollideConnected as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(should_collide);
        self.end_record();
    }

    /// Write framed `JointWakeBodies` op.
    pub fn write_joint_wake_bodies(&mut self, joint: JointId) {
        self.begin_record(RecOp::JointWakeBodies as u8);
        self.buffer.append_joint_id(joint);
        self.end_record();
    }

    /// Write framed `JointSetConstraintTuning` op.
    pub fn write_joint_set_constraint_tuning(
        &mut self,
        joint: JointId,
        hertz: f32,
        damping_ratio: f32,
    ) {
        self.begin_record(RecOp::JointSetConstraintTuning as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `JointSetForceThreshold` op.
    pub fn write_joint_set_force_threshold(&mut self, joint: JointId, threshold: f32) {
        self.begin_record(RecOp::JointSetForceThreshold as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(threshold);
        self.end_record();
    }

    /// Write framed `JointSetTorqueThreshold` op.
    pub fn write_joint_set_torque_threshold(&mut self, joint: JointId, threshold: f32) {
        self.begin_record(RecOp::JointSetTorqueThreshold as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(threshold);
        self.end_record();
    }
}
