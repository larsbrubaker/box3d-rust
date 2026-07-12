//! Framed query op writers for `Recording`, split from ops.rs to satisfy
//! the file-length limit. Layouts mirror recording_ops.inl.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::distance::ShapeProxy;
use crate::geometry::Capsule;
use crate::id::WorldId;
use crate::math_functions::{Aabb, Pos, Vec3};
use crate::types::QueryFilter;

use crate::recording::ops::RecOp;
use crate::recording::session::Recording;

impl Recording {
    /// Write framed `QueryOverlapAABB` op.
    pub fn write_query_overlap_aabb(&mut self, world: WorldId, aabb: Aabb, filter: &QueryFilter) {
        self.begin_record(RecOp::QueryOverlapAABB as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_aabb(aabb);
        self.buffer.append_query_filter(filter);
        self.end_record();
    }

    /// Write framed `QueryOverlapShape` op.
    pub fn write_query_overlap_shape(
        &mut self,
        world: WorldId,
        origin: Pos,
        proxy: &ShapeProxy,
        filter: &QueryFilter,
    ) {
        self.begin_record(RecOp::QueryOverlapShape as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_pos(origin);
        self.buffer.append_shape_proxy(proxy);
        self.buffer.append_query_filter(filter);
        self.end_record();
    }

    /// Write framed `QueryCastRay` op.
    pub fn write_query_cast_ray(
        &mut self,
        world: WorldId,
        origin: Pos,
        translation: Vec3,
        filter: &QueryFilter,
    ) {
        self.begin_record(RecOp::QueryCastRay as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_pos(origin);
        self.buffer.append_vec3(translation);
        self.buffer.append_query_filter(filter);
        self.end_record();
    }

    /// Write framed `QueryCastShape` op.
    pub fn write_query_cast_shape(
        &mut self,
        world: WorldId,
        origin: Pos,
        proxy: &ShapeProxy,
        translation: Vec3,
        filter: &QueryFilter,
    ) {
        self.begin_record(RecOp::QueryCastShape as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_pos(origin);
        self.buffer.append_shape_proxy(proxy);
        self.buffer.append_vec3(translation);
        self.buffer.append_query_filter(filter);
        self.end_record();
    }

    /// Write framed `QueryCastRayClosest` op.
    pub fn write_query_cast_ray_closest(
        &mut self,
        world: WorldId,
        origin: Pos,
        translation: Vec3,
        filter: &QueryFilter,
    ) {
        self.begin_record(RecOp::QueryCastRayClosest as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_pos(origin);
        self.buffer.append_vec3(translation);
        self.buffer.append_query_filter(filter);
        self.end_record();
    }

    /// Write framed `QueryCastMover` op.
    pub fn write_query_cast_mover(
        &mut self,
        world: WorldId,
        origin: Pos,
        mover: Capsule,
        translation: Vec3,
        filter: &QueryFilter,
    ) {
        self.begin_record(RecOp::QueryCastMover as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_pos(origin);
        self.buffer.append_capsule(mover);
        self.buffer.append_vec3(translation);
        self.buffer.append_query_filter(filter);
        self.end_record();
    }

    /// Write framed `QueryCollideMover` op.
    pub fn write_query_collide_mover(
        &mut self,
        world: WorldId,
        origin: Pos,
        mover: Capsule,
        filter: &QueryFilter,
    ) {
        self.begin_record(RecOp::QueryCollideMover as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_pos(origin);
        self.buffer.append_capsule(mover);
        self.buffer.append_query_filter(filter);
        self.end_record();
    }

    /// Write framed `QueryTag` op.
    pub fn write_query_tag(&mut self, key: u64) {
        self.begin_record(RecOp::QueryTag as u8);
        self.buffer.append_u64(key);
        self.end_record();
    }

    /// Write framed `StateHash` op.
    pub fn write_state_hash(&mut self, world: WorldId, hash: u64) {
        self.begin_record(RecOp::StateHash as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_u64(hash);
        self.end_record();
    }

    /// Write framed `RecordingBounds` op.
    pub fn write_recording_bounds(&mut self, bounds: Aabb) {
        self.begin_record(RecOp::RecordingBounds as u8);
        self.buffer.append_aabb(bounds);
        self.end_record();
    }
}
