//! Recording, replay, and world snapshots.
//!
//! Port of recording.c / recording.h / recording_replay.c / world_snapshot.c.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

mod buffer;
pub(crate) mod hash;
mod registry;
pub mod snapshot;
pub(crate) mod session;
mod writers;
pub(crate) mod capture;
pub mod ops;
pub mod dispatch;
pub mod player;
pub(crate) mod query_capture;
pub mod query_replay;

pub use buffer::{RecBuffer, SnapReader};
pub use hash::{fnv_mix_position, hash_world_state, SNAP_FNV_INIT, SNAP_FNV_PRIME};
pub use registry::{hash64_blob, GeometryEntry, GeometryKind, GeometryRegistry, RegistrySlot};
pub use session::{
    start_recording, stop_recording, with_recording, world_public_id, RecHeader, Recording, RecTag,
    REC_MAGIC, REC_VERSION_MAJOR, REC_VERSION_MINOR,
};
pub use snapshot::{
    clone_world_via_snapshot, compute_layout_hash, deserialize_into_shell, serialize_world,
    SNAP_FLAG_DOUBLE_PRECISION, SNAP_FLAG_VALIDATION, SNAP_LAYOUT_VERSION, SNAP_MAGIC,
    SNAP_VERSION,
};
pub use player::{validate_replay, RecPlayer};
pub use ops::RecOp;
pub use capture::rec as record_op;
