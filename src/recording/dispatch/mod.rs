//! Op-stream dispatcher for recording replay.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::id::{BodyId, JointId, ShapeId};
use crate::recording::buffer::SnapReader;
use crate::recording::ops::RecOp;
use crate::recording::registry::RegistrySlot;
use crate::recording::session::RecTag;
use crate::world::World;

use super::player::RecPlayer;

/// Reader state threaded through the replay loop. (b3RecReader)
pub struct RecReader<'a> {
    pub data: &'a [u8],
    pub size: i32,
    pub cursor: i32,
    pub ok: bool,
    pub diverged: bool,
    pub world: *mut World,
    pub owner: Option<*mut RecPlayer>,
    pub slots: Vec<RegistrySlot>,
    pub tags: Vec<RecTag>,
    pub pending_query_key: u64,
    pub pending_body_create: Option<BodyId>,
    pub pending_body_destroy: Option<BodyId>,
}

impl<'a> RecReader<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            size: data.len() as i32,
            cursor: 0,
            ok: true,
            diverged: false,
            world: std::ptr::null_mut(),
            owner: None,
            slots: Vec::new(),
            tags: Vec::new(),
            pending_query_key: 0,
            pending_body_create: None,
            pending_body_destroy: None,
        }
    }

    pub fn snap(&mut self) -> SnapReader<'a> {
        let mut r = SnapReader::new(self.data);
        r.set_cursor(self.cursor as usize);
        r.ok = self.ok;
        r
    }

    pub fn sync_from(&mut self, r: &SnapReader<'_>) {
        self.cursor = r.cursor() as i32;
        self.ok = r.ok;
    }

    pub fn make_body_id(&self, recorded: BodyId) -> BodyId {
        let world = unsafe { &*self.world };
        BodyId {
            index1: recorded.index1,
            world0: world.world_id,
            generation: recorded.generation,
        }
    }

    pub fn make_shape_id(&self, recorded: ShapeId) -> ShapeId {
        let world = unsafe { &*self.world };
        ShapeId {
            index1: recorded.index1,
            world0: world.world_id,
            generation: recorded.generation,
        }
    }

    pub fn make_joint_id(&self, recorded: JointId) -> JointId {
        let world = unsafe { &*self.world };
        JointId {
            index1: recorded.index1,
            world0: world.world_id,
            generation: recorded.generation,
        }
    }

    pub fn check_id(
        ok: &mut bool,
        kind: &str,
        got_index: i32,
        got_gen: u16,
        rec_index: i32,
        rec_gen: u16,
    ) {
        if got_index != rec_index || got_gen != rec_gen {
            eprintln!(
                "b3ReplayFile: {kind} id mismatch (rec index1={rec_index} gen={rec_gen}, got index1={got_index} gen={got_gen})"
            );
            *ok = false;
        }
    }
}

mod body_ops;
mod joint_angular_type_ops;
mod joint_linear_type_ops;
mod joint_ops;
mod query_ops;
mod shape_ops;
mod world_ops;

/// Dispatch one framed op. Returns opcode as i32, or -1 when exhausted/broken.
pub fn dispatch_one(rdr: &mut RecReader<'_>) -> i32 {
    if rdr.cursor >= rdr.size || !rdr.ok {
        return -1;
    }
    let mut snap = rdr.snap();
    let opcode = snap.u8();
    let payload_size = snap.u24();
    rdr.sync_from(&snap);
    if !rdr.ok {
        return -1;
    }
    let payload_start = rdr.cursor;

    let Some(op) = RecOp::from_u8(opcode) else {
        eprintln!("b3ReplayFile: unknown opcode 0x{opcode:02X}, skipping {payload_size} bytes");
        if payload_size > (rdr.size - payload_start) as u32 {
            rdr.ok = false;
        } else {
            rdr.cursor = payload_start + payload_size as i32;
        }
        return opcode as i32;
    };

    let handled = world_ops::dispatch(op, rdr, payload_start, payload_size)
        || body_ops::dispatch(op, rdr, payload_start, payload_size)
        || shape_ops::dispatch(op, rdr, payload_start, payload_size)
        || joint_ops::dispatch(op, rdr, payload_start, payload_size)
        || joint_linear_type_ops::dispatch(op, rdr, payload_start, payload_size)
        || joint_angular_type_ops::dispatch(op, rdr, payload_start, payload_size)
        || query_ops::dispatch(op, rdr, payload_start, payload_size);
    debug_assert!(handled, "op {op:?} not claimed by any dispatch family");

    opcode as i32
}
