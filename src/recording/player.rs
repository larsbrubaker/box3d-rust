//! Recording player and validate-replay. Port of recording_replay.c player API.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::core::{get_length_units_per_meter, set_length_units_per_meter};
use crate::id::BodyId;
use crate::math_functions::{Aabb, Vec3};
use crate::recording::dispatch::{dispatch_one, RecReader};
use crate::recording::ops::RecOp;
use crate::recording::registry::{GeometryKind, RegistrySlot};
use crate::recording::session::{RecHeader, RecTag, REC_MAGIC, REC_VERSION_MAJOR};
use crate::recording::snapshot::deserialize_into_shell;
use crate::types::default_world_def;
use crate::world::World;
use std::collections::HashMap;

const KEYFRAME_INTERVAL_DEFAULT: i32 = 16;

/// Interactive / headless recording player. (b3RecPlayer)
pub struct RecPlayer {
    pub data: Vec<u8>,
    pub header_end: i32,
    pub registry_end: i32,
    pub length_scale: f32,
    pub previous_length_scale: f32,
    pub frame: i32,
    pub frame_count: i32,
    pub recorded_dt: f32,
    pub recorded_sub_step_count: i32,
    pub bounds: Aabb,
    pub at_end: bool,
    pub at_pre_step: bool,
    pub diverge_frame: i32,
    pub body_ids: Vec<BodyId>,
    pub frame0_body_ids: Vec<BodyId>,
    pub keyframe_interval: i32,
    pub keyframe_min_interval: i32,
    pub keyframe_budget: usize,
    pub keyframe_bytes: usize,
    pub last_keyframe_frame: i32,
    pub world: World,
    pub frame_queries: Vec<crate::recording::query_replay::FrameQuery>,
    /// Reader borrows `data` via raw pointer for self-referential layout.
    reader_cursor: i32,
    reader_ok: bool,
    reader_diverged: bool,
    reader_slots: Vec<RegistrySlot>,
    reader_tags: Vec<RecTag>,
    reader_pending_query_key: u64,
    frame0_image_start: usize,
    frame0_image_size: usize,
}

impl RecPlayer {
    /// (b3CreatePlayer)
    pub fn create(data: &[u8], _worker_count: i32) -> Option<Box<Self>> {
        let hdr = RecHeader::from_bytes(data)?;
        if hdr.magic != REC_MAGIC {
            eprintln!("b3RecPlayer_Create: bad magic 0x{:08X}", hdr.magic);
            return None;
        }
        if hdr.version_major != REC_VERSION_MAJOR {
            eprintln!(
                "b3RecPlayer_Create: version mismatch {}.{} vs {}.x",
                hdr.version_major, hdr.version_minor, REC_VERSION_MAJOR
            );
            return None;
        }
        if hdr.pointer_width != std::mem::size_of::<*const ()>() as u8 {
            eprintln!(
                "b3RecPlayer_Create: pointer width mismatch {} vs {}",
                hdr.pointer_width,
                std::mem::size_of::<*const ()>()
            );
            return None;
        }
        if hdr.big_endian != 0 {
            eprintln!("b3RecPlayer_Create: big-endian recording not supported");
            return None;
        }
        if hdr.snapshot_size == 0 {
            eprintln!("b3RecPlayer_Create: missing snapshot seed");
            return None;
        }

        let header_end64 = 48u64 + hdr.snapshot_size;
        let registry_end64 = if hdr.registry_offset != 0 {
            hdr.registry_offset
        } else {
            data.len() as u64
        };
        if header_end64 < 48 || header_end64 > registry_end64 || registry_end64 > data.len() as u64
        {
            eprintln!("b3RecPlayer_Create: corrupt offsets");
            return None;
        }
        let header_end = header_end64 as i32;
        let registry_end = registry_end64 as i32;

        let previous_length_scale = get_length_units_per_meter();
        if hdr.length_scale > 0.0 {
            set_length_units_per_meter(hdr.length_scale);
        }

        let copy = data.to_vec();
        let snap_start = 48usize;
        let snap_size = hdr.snapshot_size as usize;

        let (mut slots, tags) =
            load_registry_block(&copy, hdr.registry_offset as usize, copy.len());

        let mut world = World::new(&default_world_def());
        if !deserialize_into_shell(
            &copy[snap_start..snap_start + snap_size],
            &mut world,
            &mut slots,
        ) {
            eprintln!("b3RecPlayer_Create: snapshot deserialization failed");
            set_length_units_per_meter(previous_length_scale);
            return None;
        }

        let mut player = Box::new(Self {
            data: copy,
            header_end,
            registry_end,
            length_scale: hdr.length_scale,
            previous_length_scale,
            frame: 0,
            frame_count: 0,
            recorded_dt: 0.0,
            recorded_sub_step_count: 0,
            bounds: Aabb::default(),
            at_end: false,
            at_pre_step: false,
            diverge_frame: -1,
            body_ids: Vec::new(),
            frame0_body_ids: Vec::new(),
            keyframe_interval: KEYFRAME_INTERVAL_DEFAULT,
            keyframe_min_interval: KEYFRAME_INTERVAL_DEFAULT,
            keyframe_budget: 0,
            keyframe_bytes: 0,
            last_keyframe_frame: 0,
            world,
            frame_queries: Vec::new(),
            reader_cursor: header_end,
            reader_ok: true,
            reader_diverged: false,
            reader_slots: slots,
            reader_tags: tags,
            reader_pending_query_key: 0,
            frame0_image_start: snap_start,
            frame0_image_size: snap_size,
        });

        player.scan_file();
        player.seed_frame0_body_ids();
        Some(player)
    }

    fn with_reader<R>(&mut self, f: impl FnOnce(&mut RecReader<'_>) -> R) -> R {
        let data_ptr = self.data.as_ptr();
        let data_len = self.data.len();
        let owner = self as *mut RecPlayer;
        let mut rdr = RecReader {
            // SAFETY: data Vec is not resized while the reader lives.
            data: unsafe { std::slice::from_raw_parts(data_ptr, data_len) },
            size: self.registry_end,
            cursor: self.reader_cursor,
            ok: self.reader_ok,
            diverged: self.reader_diverged,
            world: std::ptr::null_mut(),
            owner: Some(owner),
            slots: std::mem::take(&mut self.reader_slots),
            tags: std::mem::take(&mut self.reader_tags),
            pending_query_key: self.reader_pending_query_key,
            pending_body_create: None,
            pending_body_destroy: None,
        };
        rdr.world = &mut self.world as *mut World;
        let result = f(&mut rdr);
        let created = rdr.pending_body_create.take();
        let destroyed = rdr.pending_body_destroy.take();
        self.reader_cursor = rdr.cursor;
        self.reader_ok = rdr.ok;
        self.reader_diverged = rdr.diverged;
        self.reader_slots = rdr.slots;
        self.reader_tags = rdr.tags;
        self.reader_pending_query_key = rdr.pending_query_key;
        if let Some(id) = created {
            self.track_body_create(id);
        }
        if let Some(id) = destroyed {
            self.track_body_destroy(id);
        }
        result
    }

    /// Count Step ops and first dt. (b3RecScanFile)
    fn scan_file(&mut self) {
        let mut cursor = self.header_end as usize;
        let end = self.registry_end as usize;
        let data = &self.data;
        let mut first_step = true;
        while cursor + 4 <= end {
            let opcode = data[cursor];
            let payload = data[cursor + 1] as u32
                | ((data[cursor + 2] as u32) << 8)
                | ((data[cursor + 3] as u32) << 16);
            cursor += 4;
            if cursor + payload as usize > end {
                break;
            }
            if opcode == RecOp::Step as u8 {
                self.frame_count += 1;
                if first_step && payload >= 4 + 4 {
                    // WORLDID u32 + F32 dt + I32 subStep
                    let base = cursor + 4; // skip world id
                    if base + 8 <= cursor + payload as usize {
                        self.recorded_dt =
                            f32::from_le_bytes(data[base..base + 4].try_into().unwrap());
                        self.recorded_sub_step_count =
                            i32::from_le_bytes(data[base + 4..base + 8].try_into().unwrap());
                    }
                    first_step = false;
                }
            } else if opcode == RecOp::RecordingBounds as u8 && payload >= 24 {
                // AABB: lower xyz + upper xyz as f32
                let b = cursor;
                self.bounds = Aabb {
                    lower_bound: Vec3 {
                        x: f32::from_le_bytes(data[b..b + 4].try_into().unwrap()),
                        y: f32::from_le_bytes(data[b + 4..b + 8].try_into().unwrap()),
                        z: f32::from_le_bytes(data[b + 8..b + 12].try_into().unwrap()),
                    },
                    upper_bound: Vec3 {
                        x: f32::from_le_bytes(data[b + 12..b + 16].try_into().unwrap()),
                        y: f32::from_le_bytes(data[b + 16..b + 20].try_into().unwrap()),
                        z: f32::from_le_bytes(data[b + 20..b + 24].try_into().unwrap()),
                    },
                };
            }
            if opcode == RecOp::DestroyWorld as u8 {
                break;
            }
            cursor += payload as usize;
        }
    }

    fn seed_frame0_body_ids(&mut self) {
        self.body_ids.clear();
        for (i, body) in self.world.bodies.iter().enumerate() {
            if body.id == i as i32 && body.set_index != crate::core::NULL_INDEX {
                let id = BodyId {
                    index1: i as i32 + 1,
                    world0: self.world.world_id,
                    generation: body.generation,
                };
                self.body_ids.push(id);
            }
        }
        self.frame0_body_ids = self.body_ids.clone();
    }

    pub fn track_body_create(&mut self, id: BodyId) {
        self.body_ids.push(id);
    }

    pub fn track_body_destroy(&mut self, id: BodyId) {
        for slot in &mut self.body_ids {
            if slot.index1 == id.index1 && slot.generation == id.generation {
                *slot = BodyId::default();
                break;
            }
        }
    }

    /// (b3RecPlayer_StepFrame)
    pub fn step_frame(&mut self) -> bool {
        self.at_pre_step = false;
        self.frame_queries.clear();
        if self.at_end {
            return false;
        }

        let mut stepped = false;
        loop {
            if self.reader_cursor >= self.registry_end || !self.reader_ok {
                self.at_end = true;
                return stepped;
            }

            if stepped {
                let next = self.data[self.reader_cursor as usize];
                if next != RecOp::StateHash as u8 {
                    return true;
                }
            }

            let op = self.with_reader(dispatch_one);
            if op < 0 {
                self.at_end = true;
                return stepped;
            }
            if op == RecOp::DestroyWorld as i32 {
                self.at_end = true;
                return stepped;
            }
            if op == RecOp::Step as i32 {
                self.frame += 1;
                stepped = true;
            } else if op == RecOp::StateHash as i32 {
                if self.diverge_frame < 0 && self.reader_diverged {
                    self.diverge_frame = self.frame;
                }
            }
        }
    }

    /// Park before Step when a CreateBody led the frame. (b3RecPlayer_SubStepFrame)
    pub fn sub_step_frame(&mut self) {
        if self.at_end {
            return;
        }

        if !self.at_pre_step {
            self.frame_queries.clear();
        }

        let mut stepped = false;
        let mut have_create_body = false;
        loop {
            if self.reader_cursor >= self.registry_end || !self.reader_ok {
                self.at_end = true;
                self.at_pre_step = false;
                return;
            }

            let current = self.data[self.reader_cursor as usize];
            if stepped && current != RecOp::StateHash as u8 {
                return;
            }

            if !self.at_pre_step && have_create_body && current == RecOp::Step as u8 {
                self.at_pre_step = true;
                return;
            }

            let op = self.with_reader(dispatch_one);
            if op < 0 {
                self.at_end = true;
                self.at_pre_step = false;
                return;
            }
            if op == RecOp::DestroyWorld as i32 {
                self.at_end = true;
                self.at_pre_step = false;
                return;
            }
            if op == RecOp::CreateBody as i32 {
                have_create_body = true;
            }
            if op == RecOp::Step as i32 {
                self.frame += 1;
                stepped = true;
                self.at_pre_step = false;
            } else if op == RecOp::StateHash as i32 {
                if self.diverge_frame < 0 && self.reader_diverged {
                    self.diverge_frame = self.frame;
                }
            }
        }
    }

    /// (b3RecPlayer_Restart)
    pub fn restart(&mut self) {
        let snap = self.data
            [self.frame0_image_start..self.frame0_image_start + self.frame0_image_size]
            .to_vec();
        let mut slots = self.reader_slots.clone();
        // Clear live compounds so deserialize rebuilds.
        for s in &mut slots {
            s.live_compound = None;
        }
        if !deserialize_into_shell(&snap, &mut self.world, &mut slots) {
            self.reader_ok = false;
            return;
        }
        self.reader_slots = slots;
        self.reader_cursor = self.header_end;
        self.reader_ok = true;
        self.reader_diverged = false;
        self.reader_pending_query_key = 0;
        self.frame = 0;
        self.at_end = false;
        self.at_pre_step = false;
        self.diverge_frame = -1;
        self.body_ids = self.frame0_body_ids.clone();
        self.last_keyframe_frame = 0;
    }

    /// (b3RecPlayer_SeekFrame)
    pub fn seek_frame(&mut self, target_frame: i32) {
        if target_frame <= self.frame {
            self.restart();
        }
        while self.frame < target_frame && self.step_frame() {}
    }

    pub fn has_diverged(&self) -> bool {
        self.reader_diverged
    }

    pub fn get_diverge_frame(&self) -> i32 {
        self.diverge_frame
    }

    pub fn get_frame(&self) -> i32 {
        self.frame
    }

    pub fn get_frame_count(&self) -> i32 {
        self.frame_count
    }

    pub fn is_at_end(&self) -> bool {
        self.at_end
    }

    pub fn is_at_pre_step(&self) -> bool {
        self.at_pre_step
    }

    pub fn get_body_count(&self) -> i32 {
        self.body_ids.len() as i32
    }

    pub fn get_body_id(&self, index: i32) -> BodyId {
        if index < 0 || index as usize >= self.body_ids.len() {
            return BodyId::default();
        }
        self.body_ids[index as usize]
    }

    pub fn world(&self) -> &World {
        &self.world
    }

    pub fn world_mut(&mut self) -> &mut World {
        &mut self.world
    }

    /// (b3RecPlayer_GetInfo)
    pub fn get_info(&self) -> RecPlayerInfo {
        RecPlayerInfo {
            frame_count: self.frame_count,
            time_step: self.recorded_dt,
            sub_step_count: self.recorded_sub_step_count,
            bounds: self.bounds,
        }
    }

    /// (b3RecPlayer_SetKeyframePolicy)
    pub fn set_keyframe_policy(&mut self, budget_bytes: usize, min_interval_frames: i32) {
        self.keyframe_budget = budget_bytes;
        self.keyframe_min_interval = min_interval_frames.max(1);
        self.keyframe_interval = self.keyframe_min_interval;
        self.keyframe_bytes = 0;
    }

    pub fn get_keyframe_min_interval(&self) -> i32 {
        self.keyframe_min_interval
    }

    pub fn get_keyframe_interval(&self) -> i32 {
        self.keyframe_interval
    }

    pub fn get_keyframe_budget(&self) -> usize {
        self.keyframe_budget
    }

    pub fn get_keyframe_bytes(&self) -> usize {
        self.keyframe_bytes
    }

    pub fn get_frame_query_count(&self) -> i32 {
        self.frame_queries.len() as i32
    }

    pub fn get_frame_query(
        &self,
        index: i32,
    ) -> Option<&crate::recording::query_replay::FrameQuery> {
        self.frame_queries.get(index as usize)
    }

    pub fn resolve_tag(&self, key: u64) -> Option<&crate::recording::session::RecTag> {
        self.reader_tags.iter().find(|t| t.key == key)
    }
}

/// Viewer info for a recording. (b3RecPlayerInfo)
#[derive(Debug, Clone, Copy)]
pub struct RecPlayerInfo {
    pub frame_count: i32,
    pub time_step: f32,
    pub sub_step_count: i32,
    pub bounds: Aabb,
}

impl Drop for RecPlayer {
    fn drop(&mut self) {
        set_length_units_per_meter(self.previous_length_scale);
    }
}

/// (b3ValidateReplay)
pub fn validate_replay(data: &[u8], worker_count: i32) -> bool {
    let Some(mut player) = RecPlayer::create(data, worker_count) else {
        return false;
    };
    while player.step_frame() {
        if player.reader_diverged {
            break;
        }
    }
    player.reader_ok && !player.reader_diverged
}

fn load_registry_block(data: &[u8], offset: usize, end: usize) -> (Vec<RegistrySlot>, Vec<RecTag>) {
    if offset == 0 || offset + 4 > end {
        return (Vec::new(), Vec::new());
    }
    let mut cursor = offset;
    let entry_count = u32::from_le_bytes(data[cursor..cursor + 4].try_into().unwrap()) as usize;
    cursor += 4;
    let mut slots = Vec::with_capacity(entry_count);
    for _ in 0..entry_count {
        if cursor + 5 > end {
            break;
        }
        let kind = match data[cursor] {
            0 => GeometryKind::Hull,
            1 => GeometryKind::Mesh,
            2 => GeometryKind::HeightField,
            _ => GeometryKind::Compound,
        };
        cursor += 1;
        let byte_count = u32::from_le_bytes(data[cursor..cursor + 4].try_into().unwrap()) as usize;
        cursor += 4;
        if cursor + byte_count > end {
            break;
        }
        let bytes = data[cursor..cursor + byte_count].to_vec();
        cursor += byte_count;
        slots.push(RegistrySlot {
            kind,
            bytes,
            live_compound: None,
        });
    }

    let mut tags = Vec::new();
    if cursor + 4 <= end {
        let tag_count = u32::from_le_bytes(data[cursor..cursor + 4].try_into().unwrap()) as usize;
        cursor += 4;
        for _ in 0..tag_count {
            if cursor + 16 > end {
                break;
            }
            let key = u64::from_le_bytes(data[cursor..cursor + 8].try_into().unwrap());
            cursor += 8;
            let id = u64::from_le_bytes(data[cursor..cursor + 8].try_into().unwrap());
            cursor += 8;
            if cursor + 2 > end {
                break;
            }
            let len = u16::from_le_bytes(data[cursor..cursor + 2].try_into().unwrap());
            cursor += 2;
            let query_name = if len == 0xFFFF {
                String::new()
            } else {
                let full = len as usize;
                if cursor + full > end {
                    break;
                }
                // C clamps the stored name to B3_MAX_QUERY_NAME_LENGTH but always
                // advances the cursor by the full recorded length.
                let n = full.min(crate::recording::session::MAX_QUERY_NAME_LENGTH);
                let s = String::from_utf8_lossy(&data[cursor..cursor + n]).into_owned();
                cursor += full;
                s
            };
            tags.push(RecTag {
                key,
                id,
                query_name,
            });
        }
    }
    let _ = cursor;
    let _map: HashMap<u64, u32> = tags
        .iter()
        .enumerate()
        .map(|(i, t)| (t.key, i as u32))
        .collect();
    let _ = _map;
    (slots, tags)
}
