//! Framed body op writers for `Recording`, split from ops.rs to satisfy
//! the file-length limit. Layouts mirror recording_ops.inl.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::geometry::MassData;
use crate::id::{BodyId, WorldId};
use crate::math_functions::{Pos, Quat, Vec3, WorldTransform};
use crate::types::{BodyDef, MotionLocks};

use crate::recording::ops::RecOp;
use crate::recording::session::Recording;

impl Recording {
    /// Write framed `CreateBody` op.
    pub fn write_create_body(&mut self, world: WorldId, def: &BodyDef, ret_id: BodyId) {
        self.begin_record(RecOp::CreateBody as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_body_def(def);
        self.buffer.append_body_id(ret_id);
        self.end_record();
    }

    /// Write framed `DestroyBody` op.
    pub fn write_destroy_body(&mut self, body: BodyId) {
        self.begin_record(RecOp::DestroyBody as u8);
        self.buffer.append_body_id(body);
        self.end_record();
    }

    /// Write framed `BodySetTransform` op.
    pub fn write_body_set_transform(&mut self, body: BodyId, position: Pos, rotation: Quat) {
        self.begin_record(RecOp::BodySetTransform as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_pos(position);
        self.buffer.append_quat(rotation);
        self.end_record();
    }

    /// Write framed `BodySetLinearVelocity` op.
    pub fn write_body_set_linear_velocity(&mut self, body: BodyId, v: Vec3) {
        self.begin_record(RecOp::BodySetLinearVelocity as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_vec3(v);
        self.end_record();
    }

    /// Write framed `BodySetType` op.
    pub fn write_body_set_type(&mut self, body: BodyId, type_: i32) {
        self.begin_record(RecOp::BodySetType as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_i32(type_);
        self.end_record();
    }

    /// Write framed `BodySetName` op.
    pub fn write_body_set_name(&mut self, body: BodyId, name: &str) {
        self.begin_record(RecOp::BodySetName as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_str(name);
        self.end_record();
    }

    /// Write framed `BodySetAngularVelocity` op.
    pub fn write_body_set_angular_velocity(&mut self, body: BodyId, w: Vec3) {
        self.begin_record(RecOp::BodySetAngularVelocity as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_vec3(w);
        self.end_record();
    }

    /// Write framed `BodySetTargetTransform` op.
    pub fn write_body_set_target_transform(
        &mut self,
        body: BodyId,
        target: WorldTransform,
        time_step: f32,
        wake: bool,
    ) {
        self.begin_record(RecOp::BodySetTargetTransform as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_world_xf(target);
        self.buffer.append_f32(time_step);
        self.buffer.append_bool(wake);
        self.end_record();
    }

    /// Write framed `BodyApplyForce` op.
    pub fn write_body_apply_force(&mut self, body: BodyId, force: Vec3, point: Pos, wake: bool) {
        self.begin_record(RecOp::BodyApplyForce as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_vec3(force);
        self.buffer.append_pos(point);
        self.buffer.append_bool(wake);
        self.end_record();
    }

    /// Write framed `BodyApplyForceToCenter` op.
    pub fn write_body_apply_force_to_center(&mut self, body: BodyId, force: Vec3, wake: bool) {
        self.begin_record(RecOp::BodyApplyForceToCenter as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_vec3(force);
        self.buffer.append_bool(wake);
        self.end_record();
    }

    /// Write framed `BodyApplyTorque` op.
    pub fn write_body_apply_torque(&mut self, body: BodyId, torque: Vec3, wake: bool) {
        self.begin_record(RecOp::BodyApplyTorque as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_vec3(torque);
        self.buffer.append_bool(wake);
        self.end_record();
    }

    /// Write framed `BodyApplyLinearImpulse` op.
    pub fn write_body_apply_linear_impulse(
        &mut self,
        body: BodyId,
        impulse: Vec3,
        point: Pos,
        wake: bool,
    ) {
        self.begin_record(RecOp::BodyApplyLinearImpulse as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_vec3(impulse);
        self.buffer.append_pos(point);
        self.buffer.append_bool(wake);
        self.end_record();
    }

    /// Write framed `BodyApplyLinearImpulseToCenter` op.
    pub fn write_body_apply_linear_impulse_to_center(
        &mut self,
        body: BodyId,
        impulse: Vec3,
        wake: bool,
    ) {
        self.begin_record(RecOp::BodyApplyLinearImpulseToCenter as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_vec3(impulse);
        self.buffer.append_bool(wake);
        self.end_record();
    }

    /// Write framed `BodyApplyAngularImpulse` op.
    pub fn write_body_apply_angular_impulse(&mut self, body: BodyId, impulse: Vec3, wake: bool) {
        self.begin_record(RecOp::BodyApplyAngularImpulse as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_vec3(impulse);
        self.buffer.append_bool(wake);
        self.end_record();
    }

    /// Write framed `BodySetMassData` op.
    pub fn write_body_set_mass_data(&mut self, body: BodyId, mass_data: MassData) {
        self.begin_record(RecOp::BodySetMassData as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_mass_data(mass_data);
        self.end_record();
    }

    /// Write framed `BodyApplyMassFromShapes` op.
    pub fn write_body_apply_mass_from_shapes(&mut self, body: BodyId) {
        self.begin_record(RecOp::BodyApplyMassFromShapes as u8);
        self.buffer.append_body_id(body);
        self.end_record();
    }

    /// Write framed `BodySetLinearDamping` op.
    pub fn write_body_set_linear_damping(&mut self, body: BodyId, damping: f32) {
        self.begin_record(RecOp::BodySetLinearDamping as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_f32(damping);
        self.end_record();
    }

    /// Write framed `BodySetAngularDamping` op.
    pub fn write_body_set_angular_damping(&mut self, body: BodyId, damping: f32) {
        self.begin_record(RecOp::BodySetAngularDamping as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_f32(damping);
        self.end_record();
    }

    /// Write framed `BodySetGravityScale` op.
    pub fn write_body_set_gravity_scale(&mut self, body: BodyId, scale: f32) {
        self.begin_record(RecOp::BodySetGravityScale as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_f32(scale);
        self.end_record();
    }

    /// Write framed `BodySetAwake` op.
    pub fn write_body_set_awake(&mut self, body: BodyId, awake: bool) {
        self.begin_record(RecOp::BodySetAwake as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_bool(awake);
        self.end_record();
    }

    /// Write framed `BodyEnableSleep` op.
    pub fn write_body_enable_sleep(&mut self, body: BodyId, flag: bool) {
        self.begin_record(RecOp::BodyEnableSleep as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `BodySetSleepThreshold` op.
    pub fn write_body_set_sleep_threshold(&mut self, body: BodyId, threshold: f32) {
        self.begin_record(RecOp::BodySetSleepThreshold as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_f32(threshold);
        self.end_record();
    }

    /// Write framed `BodyDisable` op.
    pub fn write_body_disable(&mut self, body: BodyId) {
        self.begin_record(RecOp::BodyDisable as u8);
        self.buffer.append_body_id(body);
        self.end_record();
    }

    /// Write framed `BodyEnable` op.
    pub fn write_body_enable(&mut self, body: BodyId) {
        self.begin_record(RecOp::BodyEnable as u8);
        self.buffer.append_body_id(body);
        self.end_record();
    }

    /// Write framed `BodySetMotionLocks` op.
    pub fn write_body_set_motion_locks(&mut self, body: BodyId, locks: MotionLocks) {
        self.begin_record(RecOp::BodySetMotionLocks as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_locks(locks);
        self.end_record();
    }

    /// Write framed `BodySetBullet` op.
    pub fn write_body_set_bullet(&mut self, body: BodyId, flag: bool) {
        self.begin_record(RecOp::BodySetBullet as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `BodyAllowFastRotation` op.
    pub fn write_body_allow_fast_rotation(&mut self, body: BodyId, flag: bool) {
        self.begin_record(RecOp::BodyAllowFastRotation as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `BodyEnableContactRecycling` op.
    pub fn write_body_enable_contact_recycling(&mut self, body: BodyId, flag: bool) {
        self.begin_record(RecOp::BodyEnableContactRecycling as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `BodyEnableHitEvents` op.
    pub fn write_body_enable_hit_events(&mut self, body: BodyId, flag: bool) {
        self.begin_record(RecOp::BodyEnableHitEvents as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_bool(flag);
        self.end_record();
    }
}
