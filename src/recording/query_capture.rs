//! Query recording writer. Port of b3RecQueryWriter / trampolines from recording.c.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::distance::ShapeProxy;
use crate::dynamic_tree::TreeStats;
use crate::geometry::{Capsule, PlaneResult};
use crate::id::{ShapeId, WorldId};
use crate::math_functions::{Aabb, Pos, Vec3};
use crate::recording::buffer::RecBuffer;
use crate::recording::ops::RecOp;
use crate::recording::session::{world_public_id, Recording};
use crate::types::{QueryFilter, RayResult};
use crate::world::World;

/// Local buffer that accumulates one query record then commits under the session. (b3RecQueryWriter)
pub struct QueryWriter {
    buf: RecBuffer,
    hit_count: u32,
    count_offset: i32,
    tag_id: u64,
    tag_name: String,
}

impl QueryWriter {
    pub fn begin(filter: &QueryFilter) -> Self {
        Self {
            buf: RecBuffer::new(),
            hit_count: 0,
            count_offset: 0,
            tag_id: filter.id,
            tag_name: filter.name.clone(),
        }
    }

    fn reserve_u32(&mut self) -> i32 {
        let offset = self.buf.size();
        self.buf.append_u32(0);
        offset
    }

    fn patch_u32(&mut self, offset: i32, v: u32) {
        let p = offset as usize;
        debug_assert!(p + 4 <= self.buf.data.len());
        self.buf.data[p..p + 4].copy_from_slice(&v.to_le_bytes());
    }

    pub fn write_overlap_aabb_header(&mut self, world: WorldId, aabb: Aabb, filter: &QueryFilter) {
        self.buf.append_world_id(world);
        self.buf.append_aabb(aabb);
        self.buf.append_query_filter(filter);
        self.count_offset = self.reserve_u32();
    }

    pub fn write_overlap_shape_header(
        &mut self,
        world: WorldId,
        origin: Pos,
        proxy: &ShapeProxy,
        filter: &QueryFilter,
    ) {
        self.buf.append_world_id(world);
        self.buf.append_pos(origin);
        self.buf.append_shape_proxy(proxy);
        self.buf.append_query_filter(filter);
        self.count_offset = self.reserve_u32();
    }

    pub fn write_cast_ray_header(
        &mut self,
        world: WorldId,
        origin: Pos,
        translation: Vec3,
        filter: &QueryFilter,
    ) {
        self.buf.append_world_id(world);
        self.buf.append_pos(origin);
        self.buf.append_vec3(translation);
        self.buf.append_query_filter(filter);
        self.count_offset = self.reserve_u32();
    }

    pub fn write_cast_shape_header(
        &mut self,
        world: WorldId,
        origin: Pos,
        proxy: &ShapeProxy,
        translation: Vec3,
        filter: &QueryFilter,
    ) {
        self.buf.append_world_id(world);
        self.buf.append_pos(origin);
        self.buf.append_shape_proxy(proxy);
        self.buf.append_vec3(translation);
        self.buf.append_query_filter(filter);
        self.count_offset = self.reserve_u32();
    }

    pub fn write_cast_mover_header(
        &mut self,
        world: WorldId,
        origin: Pos,
        mover: Capsule,
        translation: Vec3,
        filter: &QueryFilter,
    ) {
        self.buf.append_world_id(world);
        self.buf.append_pos(origin);
        self.buf.append_capsule(mover);
        self.buf.append_vec3(translation);
        self.buf.append_query_filter(filter);
        self.count_offset = self.reserve_u32();
    }

    pub fn write_collide_mover_header(
        &mut self,
        world: WorldId,
        origin: Pos,
        mover: Capsule,
        filter: &QueryFilter,
    ) {
        self.buf.append_world_id(world);
        self.buf.append_pos(origin);
        self.buf.append_capsule(mover);
        self.buf.append_query_filter(filter);
        self.count_offset = self.reserve_u32();
    }

    pub fn write_cast_ray_closest_header(
        &mut self,
        world: WorldId,
        origin: Pos,
        translation: Vec3,
        filter: &QueryFilter,
    ) {
        self.buf.append_world_id(world);
        self.buf.append_pos(origin);
        self.buf.append_vec3(translation);
        self.buf.append_query_filter(filter);
    }

    pub fn append_overlap_hit(&mut self, id: ShapeId, user_return: bool) {
        self.buf.append_shape_id(id);
        self.buf.append_bool(user_return);
        self.hit_count += 1;
    }

    pub fn append_cast_hit(
        &mut self,
        id: ShapeId,
        point: Pos,
        normal: Vec3,
        fraction: f32,
        user_material_id: u64,
        triangle_index: i32,
        child_index: i32,
        user_return: f32,
    ) {
        self.buf.append_shape_id(id);
        self.buf.append_pos(point);
        self.buf.append_vec3(normal);
        self.buf.append_f32(fraction);
        self.buf.append_u64(user_material_id);
        self.buf.append_i32(triangle_index);
        self.buf.append_i32(child_index);
        self.buf.append_f32(user_return);
        self.hit_count += 1;
    }

    pub fn append_plane_hit(&mut self, id: ShapeId, planes: &[PlaneResult], user_return: bool) {
        self.buf.append_shape_id(id);
        self.buf.append_i32(planes.len() as i32);
        for p in planes {
            self.buf.append_vec3(p.plane.normal);
            self.buf.append_f32(p.plane.offset);
            self.buf.append_vec3(p.point);
        }
        self.buf.append_bool(user_return);
        self.hit_count += 1;
    }

    pub fn append_tree_stats(&mut self, stats: TreeStats) {
        self.buf.append_i32(stats.node_visits);
        self.buf.append_i32(stats.leaf_visits);
    }

    pub fn append_ray_result(&mut self, result: &RayResult) {
        self.buf.append_shape_id(result.shape_id);
        self.buf.append_pos(result.point);
        self.buf.append_vec3(result.normal);
        self.buf.append_u64(result.user_material_id);
        self.buf.append_f32(result.fraction);
        self.buf.append_i32(result.triangle_index);
        self.buf.append_i32(result.child_index);
        self.buf.append_bool(result.hit);
    }

    pub fn append_f32(&mut self, v: f32) {
        self.buf.append_f32(v);
    }

    pub fn finish_counted(&mut self) {
        self.patch_u32(self.count_offset, self.hit_count);
    }

    pub fn commit(self, rec: &mut Recording, opcode: RecOp) {
        let tagged = self.tag_id != 0 || !self.tag_name.is_empty();
        if tagged {
            let key = Recording::hash_query_tag(self.tag_id, &self.tag_name);
            rec.intern_tag(key, self.tag_id, &self.tag_name);
            rec.write_query_tag(key);
        }
        rec.commit_record(opcode as u8, &self.buf.data);
    }
}

/// Begin a query writer when the world is recording. (physics_world.c query prelude)
pub fn maybe_begin(world: &World, filter: &QueryFilter) -> Option<QueryWriter> {
    if world.recording.is_some() {
        Some(QueryWriter::begin(filter))
    } else {
        None
    }
}

pub fn world_id(world: &World) -> WorldId {
    world_public_id(world)
}

/// Commit a finished query writer into the active recording session.
pub fn commit(world: &World, writer: QueryWriter, opcode: RecOp) {
    let Some(rec_ptr) = world.recording else {
        return;
    };
    // SAFETY: pointer set by start_recording; exclusive while session active.
    let rec = unsafe { &mut *rec_ptr };
    writer.commit(rec, opcode);
}
