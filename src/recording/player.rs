//! Recording player and validate-replay. Port of replay.c (formerly recording_replay.c) player API.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::core::{get_length_units_per_meter, set_length_units_per_meter};
use crate::debug_draw::{CreateDebugShapeCallback, DestroyDebugShapeCallback};
use crate::id::BodyId;
use crate::math_functions::{Aabb, Vec3};
use crate::recording::dispatch::{dispatch_one, RecReader};
use crate::recording::keyframes::{
    RecKeyframe, KEYFRAME_BUDGET_DEFAULT, KEYFRAME_INTERVAL_DEFAULT,
};
use crate::recording::ops::RecOp;
use crate::recording::registry::{GeometryRegistry, RegistrySlot};
use crate::recording::registry_load::load_slots;
use crate::recording::session::{RecHeader, RecTag, REC_MAGIC, REC_VERSION_MAJOR};
use crate::recording::snapshot::deserialize_into_shell;
use crate::types::default_world_def;
use crate::world::{destroy_world, destroy_world_in_place, World};

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
    // Keyframe ring
    pub(crate) keyframes: Vec<RecKeyframe>,
    pub(crate) keyframe_budget: usize,
    pub(crate) keyframe_bytes: usize,
    pub(crate) keyframe_min_interval: i32,
    pub(crate) keyframe_interval: i32,
    pub(crate) last_keyframe_frame: i32,
    /// Pre-populated registry used by `serialize_world` during keyframe capture. Its entries
    /// mirror the reader slots 1:1 so geometry ids stay stable. (keyframeRec's registry)
    pub(crate) keyframe_registry: GeometryRegistry,
    pub world: World,
    pub frame_queries: Vec<crate::recording::query_replay::FrameQuery>,
    /// Reader borrows `data` via raw pointer for self-referential layout.
    pub(crate) reader_cursor: i32,
    pub(crate) reader_ok: bool,
    pub(crate) reader_diverged: bool,
    pub(crate) reader_slots: Vec<RegistrySlot>,
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

        // Divergence from C: on a registry or seed snapshot failure b3CreatePlayer returns NULL
        // without restoring the length scale it just set, leaking the recording's units into the
        // global state. The port restores the previous scale on every failure path.
        let Some((mut slots, tags)) =
            load_slots(&copy, hdr.registry_offset, hdr.registry_byte_count)
        else {
            set_length_units_per_meter(previous_length_scale);
            return None;
        };

        let mut world = World::new(&default_world_def());
        if !deserialize_into_shell(
            &copy[snap_start..snap_start + snap_size],
            &mut world,
            &mut slots,
        ) {
            eprintln!("b3RecPlayer_Create: snapshot deserialization failed");
            destroy_world(world);
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
            keyframes: Vec::new(),
            keyframe_budget: KEYFRAME_BUDGET_DEFAULT,
            keyframe_bytes: 0,
            keyframe_min_interval: KEYFRAME_INTERVAL_DEFAULT,
            keyframe_interval: KEYFRAME_INTERVAL_DEFAULT,
            last_keyframe_frame: 0,
            keyframe_registry: GeometryRegistry::new(),
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
        // Seed the outliner from the restored world (snapshot bodies bypass the create hook) and
        // save the frame-0 restore copy.
        player.seed_frame0_body_ids();

        // Build the keyframe registry pre-seeded to mirror the reader slots, so serialize_world
        // geometry ids stay stable across captures.
        player.seed_keyframe_registry();
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
        // This is never true when full stepping
        self.at_pre_step = false;

        if self.at_end {
            return false;
        }

        // Reset the per-frame query store before this frame's records are dispatched.
        self.frame_queries.clear();

        let mut stepped = false;
        loop {
            if self.reader_cursor >= self.registry_end || !self.reader_ok {
                self.at_end = true;
                return stepped;
            }

            // Once stepped, the StateHash is the only record still belonging to this frame.
            // Anything else begins the next frame, so stop and let the next StepFrame consume it.
            // Capture a keyframe at the boundary.
            if stepped {
                let next = self.data[self.reader_cursor as usize];
                if next != RecOp::StateHash as u8 {
                    if self.frame > self.last_keyframe_frame
                        && self.frame % self.keyframe_interval == 0
                    {
                        self.capture_keyframe();
                    }
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

            // Once stepped, the StateHash is the only record still belonging to this frame.
            // Anything else begins the next frame, so stop and let the next StepFrame consume it.
            // Capture a keyframe at the boundary.
            let current = self.data[self.reader_cursor as usize];
            if stepped && current != RecOp::StateHash as u8 {
                if self.frame > self.last_keyframe_frame && self.frame % self.keyframe_interval == 0
                {
                    self.capture_keyframe();
                }
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
        if !deserialize_into_shell(&snap, &mut self.world, &mut slots) {
            self.reader_ok = false;
            return;
        }
        self.reader_slots = slots;
        self.reader_cursor = self.header_end;
        self.reader_ok = true;
        self.reader_diverged = false;
        // C leaves rdr.pendingQueryKey untouched here, so a QueryTag left dangling by a skipped
        // query op pairs with the first query stashed after the restart.
        self.frame = 0;
        self.diverge_frame = -1;
        self.at_end = false;
        self.at_pre_step = false;

        // Frame 0 is the pre-step snapshot with no recorded queries, so clear the per-frame
        // store. This keeps the last stepped frame's queries from lingering on a restart or a
        // backward scrub to 0.
        self.frame_queries.clear();

        // Roll the outliner body list back to its frame-0 contents.
        self.body_ids = self.frame0_body_ids.clone();
        // The keyframe ring and lastKeyframeFrame survive a restart, so re-stepping frames that
        // were already captured does not capture them again.
    }

    /// Wire the host debug-shape callbacks into the replay world.
    /// (b3RecPlayer_SetDebugShapeCallbacks)
    ///
    /// A world fixes its debug-shape callbacks at creation, so rebuild frame 0 under the new
    /// wiring. The old world is destroyed first, releasing any debug shapes it created.
    /// Geometry slots are byte blobs reused as-is, the same path Restart relies on.
    pub fn set_debug_shape_callbacks(
        &mut self,
        create_debug_shape: Option<CreateDebugShapeCallback>,
        destroy_debug_shape: Option<DestroyDebugShapeCallback>,
        context: u64,
    ) {
        destroy_world_in_place(&mut self.world);

        // (b3RecPlayerCreateWorld)
        let mut def = default_world_def();
        def.create_debug_shape = create_debug_shape;
        def.destroy_debug_shape = destroy_debug_shape;
        def.user_debug_shape_context = context;
        self.world = World::new(&def);

        self.reader_cursor = self.header_end;
        self.reader_ok = true;
        self.reader_diverged = false;
        self.frame = 0;
        self.diverge_frame = -1;
        self.at_end = false;

        // Re-seed the world so its shapes are recreated through the new callbacks.
        let start = self.frame0_image_start;
        let end = start + self.frame0_image_size;
        if !deserialize_into_shell(
            &self.data[start..end],
            &mut self.world,
            &mut self.reader_slots,
        ) {
            self.reader_ok = false;
            return;
        }
        self.reader_cursor = self.header_end;

        // Rebuild the outliner from the frame-0 world that was just stood up under the new
        // callbacks.
        self.seed_frame0_body_ids();
    }

    /// Seek to `target_frame`. A backward seek, or a forward seek that can skip ahead, restores
    /// the nearest keyframe strictly before the target and then re-steps the remaining gap.
    /// (b3RecPlayer_SeekFrame)
    pub fn seek_frame(&mut self, target_frame: i32) {
        self.at_pre_step = false;

        let target_frame = target_frame.max(0);

        // Find the best keyframe strictly before the target.
        let mut best: Option<usize> = None;
        for (i, kf) in self.keyframes.iter().enumerate() {
            if kf.frame < target_frame && best.map_or(true, |b| kf.frame > self.keyframes[b].frame)
            {
                best = Some(i);
            }
        }

        if target_frame < self.frame {
            // Backward seek: restore keyframe or restart from frame 0.
            match best {
                Some(b) => self.restore_keyframe(b),
                None => self.restart(),
            }
        } else if let Some(b) = best.filter(|&b| self.keyframes[b].frame > self.frame) {
            // Forward seek that can skip ahead via a keyframe.
            self.restore_keyframe(b);
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

    pub fn get_frame_query_count(&self) -> i32 {
        self.frame_queries.len() as i32
    }

    pub fn get_frame_query(
        &self,
        index: i32,
    ) -> Option<&crate::recording::query_replay::FrameQuery> {
        self.frame_queries.get(index as usize)
    }

    /// Look up a query tag by key. C's tag map replaces on insert, so when a corrupt table holds a
    /// key more than once the last loaded tag wins.
    pub fn resolve_tag(&self, key: u64) -> Option<&crate::recording::session::RecTag> {
        self.reader_tags.iter().rev().find(|t| t.key == key)
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
    /// (b3DestroyPlayer)
    fn drop(&mut self) {
        // Tear down the replay world first so the host's destroyDebugShape sees every live
        // shape, as b3DestroyPlayer does via b3DestroyWorld. No reader holds a raw pointer to
        // `self.world` here: RecReader only lives inside `with_reader`.
        destroy_world_in_place(&mut self.world);
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
