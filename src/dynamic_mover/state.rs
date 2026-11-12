//! Save and restore of a [`DynamicMover`] (port addition; the C sample keeps its mover in
//! memory only).
//!
//! The mover keeps state between updates that lives outside the world, so a world
//! snapshot alone does not resume it. [`DynamicMover::serialize`] writes every member
//! (tuning, ids, flags, timers, the pogo warm start, the last cast and the probes) to a
//! [`RecBuffer`] in the same little-endian writers the world snapshot uses, and
//! [`DynamicMover::deserialize`] reads it back from a [`SnapReader`], so several movers
//! can follow one another or a world image in one byte stream. Floats are stored as their
//! bits, so the restored mover is identical, and the world snapshot keeps body and joint
//! ids, so its ids name the restored bodies and joints.
//!
//! The world snapshot does not store the world's `world_id`, so `deserialize` writes the
//! target world's id into the world part of every non-null saved id. [`DynamicMover::validate`]
//! checks that the ids name live bodies and joints of the expected kinds.
//!
//! Image layout: magic, version, flags (bit 0x2 marks a double-precision build, whose
//! positions are `f64`), then the members in declaration order.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{DynamicMover, DynamicMoverCastResult, DynamicMoverProbe, PROBE_CAPACITY};
use crate::body::{body_get_type, body_is_valid};
use crate::id::{BodyId, JointId};
use crate::joint::{joint_get_body_b, joint_get_type, joint_is_valid, JointType};
use crate::recording::snapshot::SNAP_FLAG_DOUBLE_PRECISION;
use crate::recording::{RecBuffer, SnapReader};
use crate::types::{BodyType, QueryFilter};
use crate::world::World;

/// Dynamic mover image magic 'BDM3'.
pub const DYNAMIC_MOVER_STATE_MAGIC: u32 = 0x334D_4442;
/// Dynamic mover image version. Bump when the member layout changes.
pub const DYNAMIC_MOVER_STATE_VERSION: u32 = 1;

/// The flags this build writes and accepts.
fn state_flags() -> u32 {
    if cfg!(feature = "double-precision") {
        SNAP_FLAG_DOUBLE_PRECISION
    } else {
        0
    }
}

fn append_body_id(buf: &mut RecBuffer, id: BodyId) {
    buf.append_i32(id.index1);
    buf.append_u16(id.world0);
    buf.append_u16(id.generation);
}

fn read_body_id(r: &mut SnapReader) -> BodyId {
    BodyId {
        index1: r.i32(),
        world0: r.u16(),
        generation: r.u16(),
    }
}

fn append_joint_id(buf: &mut RecBuffer, id: JointId) {
    buf.append_i32(id.index1);
    buf.append_u16(id.world0);
    buf.append_u16(id.generation);
}

fn read_joint_id(r: &mut SnapReader) -> JointId {
    JointId {
        index1: r.i32(),
        world0: r.u16(),
        generation: r.u16(),
    }
}

fn append_cast_result(buf: &mut RecBuffer, c: &DynamicMoverCastResult) {
    buf.append_pos(c.point);
    buf.append_vec3(c.normal);
    append_body_id(buf, c.body_id);
    buf.append_u64(c.user_material_id);
    buf.append_f32(c.fraction);
    buf.append_bool(c.hit);
}

fn read_cast_result(r: &mut SnapReader) -> DynamicMoverCastResult {
    DynamicMoverCastResult {
        point: r.pos(),
        normal: r.vec3(),
        body_id: read_body_id(r),
        user_material_id: r.u64(),
        fraction: r.f32(),
        hit: r.bool(),
    }
}

fn append_query_filter(buf: &mut RecBuffer, f: &QueryFilter) {
    buf.append_u64(f.category_bits);
    buf.append_u64(f.mask_bits);
    buf.append_u64(f.id);
    buf.append_u32(f.name.len() as u32);
    buf.append(f.name.as_bytes());
}

fn read_query_filter(r: &mut SnapReader) -> Option<QueryFilter> {
    let category_bits = r.u64();
    let mask_bits = r.u64();
    let id = r.u64();
    let name_len = r.u32() as usize;
    let name_bytes = r.bytes(name_len)?;
    let name = String::from_utf8(name_bytes.to_vec()).ok()?;
    Some(QueryFilter {
        category_bits,
        mask_bits,
        id,
        name,
    })
}

impl DynamicMover {
    /// Append this mover's complete state to `buf`. Restore it with
    /// [`DynamicMover::deserialize`] after the world it drives is restored.
    pub fn serialize(&self, buf: &mut RecBuffer) {
        buf.append_u32(DYNAMIC_MOVER_STATE_MAGIC);
        buf.append_u32(DYNAMIC_MOVER_STATE_VERSION);
        buf.append_u32(state_flags());

        buf.append_f32(self.jump_speed);
        buf.append_f32(self.max_speed);
        buf.append_f32(self.min_speed);
        buf.append_f32(self.stop_speed);
        buf.append_f32(self.accelerate);
        buf.append_f32(self.air_steer);
        buf.append_f32(self.friction);
        buf.append_f32(self.gravity_scale);
        buf.append_f32(self.max_ground_force);
        buf.append_f32(self.max_air_force);
        buf.append_f32(self.pogo_rest_length);
        buf.append_f32(self.pogo_hertz);
        buf.append_f32(self.pogo_damping_ratio);
        buf.append_f32(self.pogo_compression_scale);
        buf.append_f32(self.pogo_tension_scale);
        buf.append_bool(self.enable_skirt_pogo);
        buf.append_u64(self.platform_material_id);
        buf.append_f32(self.min_ground_normal_y);

        buf.append_capsule(self.capsule);
        buf.append_filter(self.filter);
        append_query_filter(buf, &self.query_filter);

        append_body_id(buf, self.ground_id);
        buf.append_bool(self.owns_ground);
        append_body_id(buf, self.mover_id);
        append_body_id(buf, self.surface_body_id);
        append_joint_id(buf, self.mover_joint_id);
        append_joint_id(buf, self.pogo_joint_id);

        append_cast_result(buf, &self.cast_result);

        buf.append_vec3(self.velocity);
        buf.append_f32(self.time_sliding);
        buf.append_f32(self.slide_duration);
        buf.append_bool(self.on_ground);
        buf.append_bool(self.walkable);
        buf.append_bool(self.jumping);
        buf.append_i32(self.jump_ticks);

        buf.append_f32(self.pogo_impulse);
        buf.append_f32(self.pogo_velocity);
        buf.append_f32(self.pogo_length);

        for probe in &self.probes {
            buf.append_pos(probe.origin);
            buf.append_vec3(probe.translation);
            buf.append_f32(probe.fraction);
            buf.append_bool(probe.hit);
        }
        buf.append_u32(self.probe_count as u32);
        buf.append_u32(self.selected_probe as u32);
    }

    /// Read a mover written by [`DynamicMover::serialize`] at the reader's cursor, for the
    /// restored `world` it will drive: every non-null id takes `world.world_id` as its world
    /// part. Returns `None` for a truncated image, a wrong magic or version, an image from
    /// a build with the other position precision, or out-of-range probe counts. Check the
    /// result against the world with [`DynamicMover::validate`] before stepping.
    pub fn deserialize(r: &mut SnapReader, world: &World) -> Option<DynamicMover> {
        if r.u32() != DYNAMIC_MOVER_STATE_MAGIC
            || r.u32() != DYNAMIC_MOVER_STATE_VERSION
            || r.u32() != state_flags()
        {
            return None;
        }

        let mut m = DynamicMover::new();
        m.jump_speed = r.f32();
        m.max_speed = r.f32();
        m.min_speed = r.f32();
        m.stop_speed = r.f32();
        m.accelerate = r.f32();
        m.air_steer = r.f32();
        m.friction = r.f32();
        m.gravity_scale = r.f32();
        m.max_ground_force = r.f32();
        m.max_air_force = r.f32();
        m.pogo_rest_length = r.f32();
        m.pogo_hertz = r.f32();
        m.pogo_damping_ratio = r.f32();
        m.pogo_compression_scale = r.f32();
        m.pogo_tension_scale = r.f32();
        m.enable_skirt_pogo = r.bool();
        m.platform_material_id = r.u64();
        m.min_ground_normal_y = r.f32();

        m.capsule = r.capsule();
        m.filter = r.filter();
        m.query_filter = read_query_filter(r)?;

        m.ground_id = read_body_id(r);
        m.owns_ground = r.bool();
        m.mover_id = read_body_id(r);
        m.surface_body_id = read_body_id(r);
        m.mover_joint_id = read_joint_id(r);
        m.pogo_joint_id = read_joint_id(r);

        m.cast_result = read_cast_result(r);

        m.velocity = r.vec3();
        m.time_sliding = r.f32();
        m.slide_duration = r.f32();
        m.on_ground = r.bool();
        m.walkable = r.bool();
        m.jumping = r.bool();
        m.jump_ticks = r.i32();

        m.pogo_impulse = r.f32();
        m.pogo_velocity = r.f32();
        m.pogo_length = r.f32();

        for i in 0..PROBE_CAPACITY {
            m.probes[i] = DynamicMoverProbe {
                origin: r.pos(),
                translation: r.vec3(),
                fraction: r.f32(),
                hit: r.bool(),
            };
        }
        m.probe_count = r.u32() as usize;
        m.selected_probe = r.u32() as usize;

        if !r.ok || m.probe_count > PROBE_CAPACITY || m.selected_probe >= PROBE_CAPACITY {
            return None;
        }

        let world0 = world.world_id;
        for id in [
            &mut m.ground_id,
            &mut m.mover_id,
            &mut m.surface_body_id,
            &mut m.cast_result.body_id,
        ] {
            if id.is_non_null() {
                id.world0 = world0;
            }
        }
        for id in [&mut m.mover_joint_id, &mut m.pogo_joint_id] {
            if id.is_non_null() {
                id.world0 = world0;
            }
        }
        Some(m)
    }

    /// Whether this mover's ids name what it needs in `world`: a live dynamic body, a live
    /// static anchor, a live mover joint on the body, and, when set, a live pogo joint on
    /// the body and a live surface body, all in this world. A game checks a loaded image
    /// with it before stepping.
    pub fn validate(&self, world: &World) -> bool {
        let world0 = world.world_id;
        let body_ok = |id: BodyId| id.world0 == world0 && body_is_valid(world, id);
        let joint_ok = |id: JointId, type_: JointType| {
            id.world0 == world0
                && joint_is_valid(world, id)
                && joint_get_type(world, id) == type_
                && joint_get_body_b(world, id) == self.mover_id
        };

        body_ok(self.mover_id)
            && body_get_type(world, self.mover_id) == BodyType::Dynamic
            && body_ok(self.ground_id)
            && body_get_type(world, self.ground_id) == BodyType::Static
            && joint_ok(self.mover_joint_id, JointType::Mover)
            && (self.pogo_joint_id.is_null() || joint_ok(self.pogo_joint_id, JointType::Pogo))
            && (self.surface_body_id.is_null() || body_ok(self.surface_body_id))
            && self.probe_count <= PROBE_CAPACITY
            && self.selected_probe < PROBE_CAPACITY
    }
}
