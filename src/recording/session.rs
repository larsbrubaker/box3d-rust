//! Recording session lifecycle: create, start/stop, framing, registry tail.
//! Port of recording.c lifecycle + framing (b3Recording, b3RecBegin/EndRecord).
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::core::get_length_units_per_meter;
use crate::id::WorldId;
use crate::math_functions::{aabb_union, Aabb};
use crate::recording::buffer::RecBuffer;
use crate::recording::hash::hash_world_state;
use crate::recording::registry::GeometryRegistry;
use crate::recording::snapshot::serialize_world;
use crate::world::World;
use std::collections::HashMap;

/// Magic 'B3RC' little-endian. (B3_REC_MAGIC)
pub const REC_MAGIC: u32 = 0x4352_3342;
/// Major recording version. (B3_REC_VERSION_MAJOR)
/// Major version 4 added b3ShapeDef::enableSpeculativeContact.
pub const REC_VERSION_MAJOR: u16 = 4;
/// Minor recording version — v3 added the name cache. (B3_REC_VERSION_MINOR)
pub const REC_VERSION_MINOR: u16 = 3;

/// Maximum query name length. Query names longer than this probably indicate a
/// bug in user code. (B3_MAX_QUERY_NAME_LENGTH)
pub const MAX_QUERY_NAME_LENGTH: usize = 64;

/// Fixed 48-byte recording header. (b3RecHeader)
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct RecHeader {
    pub magic: u32,
    pub version_major: u16,
    pub version_minor: u16,
    pub pointer_width: u8,
    pub big_endian: u8,
    pub validation_enabled: u8,
    pub reserved: u8,
    pub length_scale: f32,
    pub reserved2: u32,
    pub reserved3: u32,
    pub snapshot_size: u64,
    pub registry_offset: u64,
    pub registry_byte_count: u64,
}

const _: () = assert!(std::mem::size_of::<RecHeader>() == 48);

impl RecHeader {
    pub fn to_bytes(self) -> [u8; 48] {
        let mut b = [0u8; 48];
        b[0..4].copy_from_slice(&self.magic.to_le_bytes());
        b[4..6].copy_from_slice(&self.version_major.to_le_bytes());
        b[6..8].copy_from_slice(&self.version_minor.to_le_bytes());
        b[8] = self.pointer_width;
        b[9] = self.big_endian;
        b[10] = self.validation_enabled;
        b[11] = self.reserved;
        b[12..16].copy_from_slice(&self.length_scale.to_le_bytes());
        b[16..20].copy_from_slice(&self.reserved2.to_le_bytes());
        b[20..24].copy_from_slice(&self.reserved3.to_le_bytes());
        b[24..32].copy_from_slice(&self.snapshot_size.to_le_bytes());
        b[32..40].copy_from_slice(&self.registry_offset.to_le_bytes());
        b[40..48].copy_from_slice(&self.registry_byte_count.to_le_bytes());
        b
    }

    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 48 {
            return None;
        }
        Some(Self {
            magic: u32::from_le_bytes(data[0..4].try_into().ok()?),
            version_major: u16::from_le_bytes(data[4..6].try_into().ok()?),
            version_minor: u16::from_le_bytes(data[6..8].try_into().ok()?),
            pointer_width: data[8],
            big_endian: data[9],
            validation_enabled: data[10],
            reserved: data[11],
            length_scale: f32::from_le_bytes(data[12..16].try_into().ok()?),
            reserved2: u32::from_le_bytes(data[16..20].try_into().ok()?),
            reserved3: u32::from_le_bytes(data[20..24].try_into().ok()?),
            snapshot_size: u64::from_le_bytes(data[24..32].try_into().ok()?),
            registry_offset: u64::from_le_bytes(data[32..40].try_into().ok()?),
            registry_byte_count: u64::from_le_bytes(data[40..48].try_into().ok()?),
        })
    }
}

/// Interned query tag. (b3RecTag)
#[derive(Debug, Clone)]
pub struct RecTag {
    /// hash of (id, query_name)
    pub key: u64,
    pub id: u64,
    pub query_name: String,
}

/// User-owned recording buffer. (b3Recording)
#[derive(Debug)]
pub struct Recording {
    pub buffer: RecBuffer,
    /// Offset of the 3-byte size field for u24 backpatch.
    pub record_start: i32,
    pub registry: GeometryRegistry,
    pub tags: Vec<RecTag>,
    tag_map: HashMap<u64, u32>,
    pub accumulated_bounds: Aabb,
    pub have_bounds: bool,
}

impl Recording {
    /// (b3CreateRecording)
    pub fn new(byte_capacity: i32) -> Self {
        let init_cap = if byte_capacity > 0 {
            byte_capacity as usize
        } else {
            65536
        };
        Self {
            buffer: RecBuffer::with_capacity(init_cap),
            record_start: 0,
            registry: GeometryRegistry::new(),
            tags: Vec::new(),
            tag_map: HashMap::new(),
            accumulated_bounds: Aabb::default(),
            have_bounds: false,
        }
    }

    pub fn data(&self) -> &[u8] {
        &self.buffer.data
    }

    pub fn size(&self) -> i32 {
        self.buffer.size()
    }

    /// (b3RecBeginRecord)
    pub fn begin_record(&mut self, opcode: u8) {
        self.buffer.append_u8(opcode);
        self.record_start = self.buffer.size();
        self.buffer.append(&[0, 0, 0]);
    }

    /// (b3RecEndRecord)
    pub fn end_record(&mut self) {
        let payload_size = self.buffer.size() - self.record_start - 3;
        debug_assert!(payload_size >= 0 && payload_size < (1 << 24));
        let p = self.record_start as usize;
        let sz = payload_size as u32;
        self.buffer.data[p] = sz as u8;
        self.buffer.data[p + 1] = (sz >> 8) as u8;
        self.buffer.data[p + 2] = (sz >> 16) as u8;
    }

    /// (b3RecCommitRecord)
    pub fn commit_record(&mut self, opcode: u8, payload: &[u8]) {
        debug_assert!(payload.len() < (1 << 24));
        self.buffer.append_u8(opcode);
        let sz = payload.len() as u32;
        self.buffer
            .append(&[sz as u8, (sz >> 8) as u8, (sz >> 16) as u8]);
        self.buffer.append(payload);
    }

    /// (b3RecAccumulateBounds)
    pub fn accumulate_bounds(&mut self, bounds: Aabb) {
        self.accumulated_bounds = if self.have_bounds {
            aabb_union(self.accumulated_bounds, bounds)
        } else {
            bounds
        };
        self.have_bounds = true;
    }

    /// (b3HashQueryTag)
    pub fn hash_query_tag(id: u64, name: &str) -> u64 {
        use crate::recording::hash::{SNAP_FNV_INIT, SNAP_FNV_PRIME};
        let mut h = SNAP_FNV_INIT;
        for i in 0..8 {
            h = (h ^ ((id >> (8 * i)) & 0xFF)).wrapping_mul(SNAP_FNV_PRIME);
        }
        for b in name.bytes() {
            h = (h ^ (b as u64)).wrapping_mul(SNAP_FNV_PRIME);
        }
        if h != 0 {
            h
        } else {
            1
        }
    }

    /// (b3RecInternTag)
    pub fn intern_tag(&mut self, key: u64, id: u64, name: &str) {
        if self.tag_map.contains_key(&key) {
            return;
        }
        let index = self.tags.len() as u32;
        let mut clamped = String::new();
        for (i, ch) in name.chars().enumerate() {
            if i >= MAX_QUERY_NAME_LENGTH {
                break;
            }
            clamped.push(ch);
        }
        self.tags.push(RecTag {
            key,
            id,
            query_name: clamped,
        });
        self.tag_map.insert(key, index);
    }

    /// (b3RecWriteRegistry)
    pub fn write_registry(&mut self) {
        self.buffer.append_u32(self.registry.entries.len() as u32);
        for e in &self.registry.entries {
            self.buffer.append_u8(e.kind as u8);
            self.buffer.append_u32(e.bytes.len() as u32);
            self.buffer.append(&e.bytes);
        }
        self.buffer.append_u32(self.tags.len() as u32);
        for t in &self.tags {
            self.buffer.append_u64(t.key);
            self.buffer.append_u64(t.id);
            self.buffer.append_str(&t.query_name);
        }
    }

    fn reset_for_session(&mut self) {
        self.buffer.data.clear();
        self.record_start = 0;
        self.have_bounds = false;
        self.registry = GeometryRegistry::new();
        self.tags.clear();
        self.tag_map.clear();
    }
}

/// Public world id matching C's `{ worldId+1, generation }`.
pub fn world_public_id(world: &World) -> WorldId {
    WorldId {
        index1: world.world_id.wrapping_add(1),
        generation: world.generation,
    }
}

/// Start recording into a user-owned buffer. (b3StartRecordingIntoBuffer /
/// b3World_StartRecording)
pub fn start_recording(world: &mut World, recording: &mut Recording) {
    recording.reset_for_session();

    let mut hdr = RecHeader {
        magic: REC_MAGIC,
        version_major: REC_VERSION_MAJOR,
        version_minor: REC_VERSION_MINOR,
        pointer_width: std::mem::size_of::<*const ()>() as u8,
        big_endian: 0,
        validation_enabled: if cfg!(debug_assertions) { 1 } else { 0 },
        reserved: 0,
        length_scale: get_length_units_per_meter(),
        reserved2: 0,
        reserved3: 0,
        snapshot_size: 0,
        registry_offset: 0,
        registry_byte_count: 0,
    };

    // SAFETY: recording outlives the session; cleared in stop_recording.
    world.recording = Some(recording as *mut Recording);

    let mut snap_buf = RecBuffer::new();
    serialize_world(world, &mut snap_buf, &mut recording.registry);
    hdr.snapshot_size = snap_buf.size() as u64;

    recording.buffer.append(&hdr.to_bytes());
    recording.buffer.append(&snap_buf.data);

    let wid = world_public_id(world);
    let hash = hash_world_state(world);
    recording.write_state_hash(wid, hash);
}

/// Stop recording and finalize the buffer. (b3StopRecordingInternal /
/// b3World_StopRecording)
pub fn stop_recording(world: &mut World) {
    let Some(rec_ptr) = world.recording.take() else {
        return;
    };
    // SAFETY: pointer set by start_recording; exclusive while session active.
    let rec = unsafe { &mut *rec_ptr };

    let bounds = if rec.have_bounds {
        rec.accumulated_bounds
    } else {
        Aabb::default()
    };
    rec.write_recording_bounds(bounds);

    let wid = world_public_id(world);
    rec.write_destroy_world(wid);

    let registry_offset = rec.buffer.size();
    rec.write_registry();
    let registry_byte_count = rec.buffer.size() - registry_offset;

    // Backpatch registryOffset / registryByteCount at header offsets 32 / 40.
    let data = &mut rec.buffer.data;
    debug_assert!(data.len() >= 48);
    data[32..40].copy_from_slice(&(registry_offset as u64).to_le_bytes());
    data[40..48].copy_from_slice(&(registry_byte_count as u64).to_le_bytes());
}

/// Invoke `f` on the active recording, if any.
#[inline]
pub fn with_recording(world: &mut World, f: impl FnOnce(&mut Recording)) {
    if let Some(p) = world.recording {
        // SAFETY: valid for the session; no concurrent access in serial port.
        unsafe { f(&mut *p) }
    }
}
