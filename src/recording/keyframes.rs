//! Replay keyframe ring: periodic world snapshots kept under a byte budget so a seek restores the
//! nearest earlier keyframe and re-steps only the gap. Port of the keyframe half of the
//! `b3RecPlayer` implementation in replay.c (`b3RecSeedKeyframeRegistry`, `b3RecCaptureKeyframe`,
//! `b3RecPlayerRestoreKeyframe`, `b3FreeKeyframe`, and the keyframe policy accessors), split from
//! player.rs to keep both files under the length limit.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::core::hash64_non_zero;
use crate::id::BodyId;
use crate::recording::buffer::RecBuffer;
use crate::recording::player::RecPlayer;
use crate::recording::registry::GeometryKind;
use crate::recording::snapshot::{deserialize_into_shell, serialize_world};

/// (B3_REC_KEYFRAME_INTERVAL_DEFAULT)
pub(crate) const KEYFRAME_INTERVAL_DEFAULT: i32 = 16;
/// (B3_REC_KEYFRAME_BUDGET_DEFAULT)
pub(crate) const KEYFRAME_BUDGET_DEFAULT: usize = 512 * 1024 * 1024;

/// Stored snapshot for fast backward seek. (b3RecKeyframe)
///
/// C keeps the image as a raw allocation plus `imageSize` / `imageCapacity`. The port owns the
/// image bytes as a `Vec` (`image.len()` is `imageSize`) and keeps `image_capacity` as the
/// allocation size C's b3RecBuffer would hold for the same appends, which is what the budget
/// charges. Freeing a keyframe (b3FreeKeyframe) is dropping it.
#[derive(Debug, Clone)]
pub(crate) struct RecKeyframe {
    /// serialized world image at the end of this frame
    pub image: Vec<u8>,
    /// allocation size (may exceed the image size)
    pub image_capacity: i32,
    /// frame index this restores to
    pub frame: i32,
    /// op-stream cursor for the frame AFTER this one
    pub cursor: i32,
    /// divergeFrame state at capture
    pub diverge_frame: i32,
    /// rdr.diverged state at capture
    pub diverged: bool,
    /// Outliner body list as it stood at this frame, restored verbatim so ordinals are stable.
    pub body_ids: Vec<BodyId>,
}

impl RecKeyframe {
    /// The bytes this keyframe charges against the budget: its image allocation plus the
    /// outliner body list, `imageCapacity + bodyIdCount * sizeof( b3BodyId )` in C.
    fn charged_bytes(&self) -> usize {
        self.image_capacity as usize + self.body_ids.len() * std::mem::size_of::<BodyId>()
    }
}

impl RecPlayer {
    /// Pre-populate the keyframe registry to mirror the reader slots so geometry ids stay stable
    /// during `serialize_world`. Each slot becomes one entry with id == slot index, even
    /// byte-identical slots that a hash collision left undeduplicated in an already-recorded file.
    /// `serialize_world` then resolves a live blob back to a valid slot index via the registry's
    /// exact dedup, so capture never grows it. (b3RecSeedKeyframeRegistry)
    ///
    /// C seeds `keyframeRec`, a whole b3Recording made by `b3CreateRecording( 0 )`, but only its
    /// registry is ever used (b3SerializeWorld writes the image into a separate buffer), so the
    /// port keeps just the registry.
    pub(super) fn seed_keyframe_registry(&mut self) {
        let reg = &mut self.keyframe_registry;
        for (i, slot) in self.reader_slots.iter().enumerate() {
            let h = hash64_non_zero(&slot.bytes);
            // C copies the raw kind byte into the entry. The port's entry kind is a
            // `GeometryKind`, so a kind byte outside it is stored as a hull. That is never
            // observable: interning compares bytes only, and the keyframe registry is never
            // written out, so an entry's kind is not read.
            let kind = match slot.kind {
                k if k == GeometryKind::Mesh as u8 => GeometryKind::Mesh,
                k if k == GeometryKind::HeightField as u8 => GeometryKind::HeightField,
                k if k == GeometryKind::Compound as u8 => GeometryKind::Compound,
                _ => GeometryKind::Hull,
            };
            let id = reg.append(kind, h, slot.bytes.clone());
            // Seeding in order without dedup keeps id == slot index.
            debug_assert_eq!(id, i as u32);
        }
    }

    /// Capture a restore-point keyframe for the just-completed frame. The reader cursor already
    /// points to the next frame's Step op. (b3RecCaptureKeyframe)
    pub(super) fn capture_keyframe(&mut self) {
        let mut buf = RecBuffer::new();

        let reg_count_before = self.keyframe_registry.entries.len();

        serialize_world(&self.world, &mut buf, &mut self.keyframe_registry);
        // Registry must not grow: all geometry was pre-seeded and the registry dedups exactly.
        debug_assert_eq!(self.keyframe_registry.entries.len(), reg_count_before);

        let body_bytes = self.body_ids.len() * std::mem::size_of::<BodyId>();
        let new_bytes = buf.capacity as usize + body_bytes;

        // Make room under the budget by doubling the spacing and evicting off-grid keyframes.
        while !self.keyframes.is_empty() && self.keyframe_bytes + new_bytes > self.keyframe_budget {
            self.keyframe_interval *= 2;
            let interval = self.keyframe_interval;
            let count_before = self.keyframes.len();
            // `retain` keeps the survivors in ring order, as C's in-place compaction does.
            self.keyframes.retain(|kf| kf.frame % interval == 0);
            let kept_bytes = self.keyframes.iter().map(RecKeyframe::charged_bytes).sum();
            let progress = self.keyframes.len() < count_before;
            self.keyframe_bytes = kept_bytes;
            if !progress {
                break;
            }
        }

        // C grows the ring array (8, then doubling) before appending. `Vec` grows itself, and the
        // ring array is not charged against the budget, so nothing observable differs.
        self.keyframes.push(RecKeyframe {
            image: buf.data,
            image_capacity: buf.capacity,
            frame: self.frame,
            cursor: self.reader_cursor,
            diverge_frame: self.diverge_frame,
            diverged: self.reader_diverged,
            body_ids: self.body_ids.clone(),
        });

        self.keyframe_bytes += new_bytes;
        self.last_keyframe_frame = self.frame;
    }

    /// Restore the world in-place from the keyframe at `index` in the ring.
    /// (b3RecPlayerRestoreKeyframe)
    pub(super) fn restore_keyframe(&mut self, index: usize) {
        let kf = &self.keyframes[index];
        if !deserialize_into_shell(&kf.image, &mut self.world, &mut self.reader_slots) {
            self.reader_ok = false;
            return;
        }
        self.reader_cursor = kf.cursor;
        self.reader_ok = true;
        self.reader_diverged = kf.diverged;
        self.frame = kf.frame;
        self.diverge_frame = kf.diverge_frame;
        self.at_end = false;
        self.at_pre_step = false;

        // Restore the outliner list verbatim so ordinals match this frame.
        self.body_ids.clear();
        self.body_ids.extend_from_slice(&kf.body_ids);
    }

    /// Tune the keyframe ring used to speed up backward seeking. A zero budget or a non-positive
    /// interval keeps the current value. The ring is dropped so it repopulates under the new
    /// policy on the next replay. (b3RecPlayer_SetKeyframePolicy)
    pub fn set_keyframe_policy(&mut self, budget_bytes: usize, min_interval_frames: i32) {
        if budget_bytes > 0 {
            self.keyframe_budget = budget_bytes;
        }
        if min_interval_frames > 0 {
            self.keyframe_min_interval = min_interval_frames;
        }

        // Drop the ring so it repopulates under the new policy on the next replay.
        self.keyframes.clear();
        self.keyframe_bytes = 0;
        self.keyframe_interval = self.keyframe_min_interval;
        self.last_keyframe_frame = 0;
    }

    /// The keyframe memory budget in bytes. (b3RecPlayer_GetKeyframeBudget)
    pub fn get_keyframe_budget(&self) -> usize {
        self.keyframe_budget
    }

    /// The finest keyframe spacing in frames. (b3RecPlayer_GetKeyframeMinInterval)
    pub fn get_keyframe_min_interval(&self) -> i32 {
        self.keyframe_min_interval
    }

    /// The current keyframe spacing in frames; starts at the min interval and doubles as the
    /// budget fills. (b3RecPlayer_GetKeyframeInterval)
    pub fn get_keyframe_interval(&self) -> i32 {
        self.keyframe_interval
    }

    /// The memory currently held by keyframe snapshots, in bytes. (b3RecPlayer_GetKeyframeBytes)
    ///
    /// Each keyframe is charged its image allocation size plus its outliner body list, as in C.
    /// The port's image is its own field-by-field snapshot format, not C's struct-image format,
    /// so the byte totals differ from C's for the same world; the capture cadence and eviction
    /// rules applied to them are C's.
    pub fn get_keyframe_bytes(&self) -> usize {
        self.keyframe_bytes
    }

    /// The frame indices of the keyframes in the ring, in ring order. Test-only inspection aid
    /// (C tests could read `player->keyframes` directly).
    #[cfg(test)]
    pub(crate) fn get_keyframe_frames(&self) -> Vec<i32> {
        self.keyframes.iter().map(|kf| kf.frame).collect()
    }
}
