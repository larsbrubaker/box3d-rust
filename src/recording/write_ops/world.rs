//! Framed world op writers for `Recording`, split from ops.rs to satisfy
//! the file-length limit. Layouts mirror recording_ops.inl.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::id::WorldId;
use crate::math_functions::Vec3;
use crate::types::ExplosionDef;

use crate::recording::ops::RecOp;
use crate::recording::session::Recording;

impl Recording {
    /// Write framed `DestroyWorld` op.
    pub fn write_destroy_world(&mut self, world: WorldId) {
        self.begin_record(RecOp::DestroyWorld as u8);
        self.buffer.append_world_id(world);
        self.end_record();
    }

    /// Write framed `Step` op.
    pub fn write_step(&mut self, world: WorldId, dt: f32, sub_step_count: i32) {
        self.begin_record(RecOp::Step as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_f32(dt);
        self.buffer.append_i32(sub_step_count);
        self.end_record();
    }

    /// Write framed `WorldEnableSleeping` op.
    pub fn write_world_enable_sleeping(&mut self, world: WorldId, flag: bool) {
        self.begin_record(RecOp::WorldEnableSleeping as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `WorldEnableContinuous` op.
    pub fn write_world_enable_continuous(&mut self, world: WorldId, flag: bool) {
        self.begin_record(RecOp::WorldEnableContinuous as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `WorldSetRestitutionThreshold` op.
    pub fn write_world_set_restitution_threshold(&mut self, world: WorldId, value: f32) {
        self.begin_record(RecOp::WorldSetRestitutionThreshold as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_f32(value);
        self.end_record();
    }

    /// Write framed `WorldSetHitEventThreshold` op.
    pub fn write_world_set_hit_event_threshold(&mut self, world: WorldId, value: f32) {
        self.begin_record(RecOp::WorldSetHitEventThreshold as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_f32(value);
        self.end_record();
    }

    /// Write framed `WorldSetGravity` op.
    pub fn write_world_set_gravity(&mut self, world: WorldId, gravity: Vec3) {
        self.begin_record(RecOp::WorldSetGravity as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_vec3(gravity);
        self.end_record();
    }

    /// Write framed `WorldExplode` op.
    pub fn write_world_explode(&mut self, world: WorldId, def: ExplosionDef) {
        self.begin_record(RecOp::WorldExplode as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_explosion_def(def);
        self.end_record();
    }

    /// Write framed `WorldSetContactTuning` op.
    pub fn write_world_set_contact_tuning(
        &mut self,
        world: WorldId,
        hertz: f32,
        damping_ratio: f32,
        contact_speed: f32,
    ) {
        self.begin_record(RecOp::WorldSetContactTuning as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_f32(hertz);
        self.buffer.append_f32(damping_ratio);
        self.buffer.append_f32(contact_speed);
        self.end_record();
    }

    /// Write framed `WorldSetContactRecycleDistance` op.
    pub fn write_world_set_contact_recycle_distance(
        &mut self,
        world: WorldId,
        recycle_distance: f32,
    ) {
        self.begin_record(RecOp::WorldSetContactRecycleDistance as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_f32(recycle_distance);
        self.end_record();
    }

    /// Write framed `WorldSetMaximumLinearSpeed` op.
    pub fn write_world_set_maximum_linear_speed(
        &mut self,
        world: WorldId,
        maximum_linear_speed: f32,
    ) {
        self.begin_record(RecOp::WorldSetMaximumLinearSpeed as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_f32(maximum_linear_speed);
        self.end_record();
    }

    /// Write framed `WorldEnableWarmStarting` op.
    pub fn write_world_enable_warm_starting(&mut self, world: WorldId, flag: bool) {
        self.begin_record(RecOp::WorldEnableWarmStarting as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `WorldRebuildStaticTree` op.
    pub fn write_world_rebuild_static_tree(&mut self, world: WorldId) {
        self.begin_record(RecOp::WorldRebuildStaticTree as u8);
        self.buffer.append_world_id(world);
        self.end_record();
    }

    /// Write framed `WorldEnableSpeculative` op.
    pub fn write_world_enable_speculative(&mut self, world: WorldId, flag: bool) {
        self.begin_record(RecOp::WorldEnableSpeculative as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_bool(flag);
        self.end_record();
    }
}
