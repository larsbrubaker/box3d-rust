//! Recording, replay, and world snapshots.
//!
//! Port of recording.c / recording.h / recording_replay.c / world_snapshot.c.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

mod buffer;
pub(crate) mod capture;
pub mod dispatch;
pub(crate) mod hash;
pub mod ops;
pub mod player;
pub(crate) mod query_capture;
pub mod query_replay;
mod registry;
pub(crate) mod session;
pub mod snapshot;
mod write_ops;
mod writers;

pub use buffer::{RecBuffer, SnapReader};
pub use capture::rec as record_op;
pub use hash::{fnv_mix_position, hash_world_state, SNAP_FNV_INIT, SNAP_FNV_PRIME};
pub use ops::RecOp;
pub use player::{validate_replay, RecPlayer};
pub use registry::{GeometryEntry, GeometryKind, GeometryRegistry, RegistrySlot};
pub use session::{
    start_recording, stop_recording, with_recording, world_public_id, RecHeader, RecTag, Recording,
    REC_MAGIC, REC_VERSION_MAJOR, REC_VERSION_MINOR,
};
pub use snapshot::{
    clone_world_via_snapshot, compute_layout_hash, deserialize_into_shell, serialize_world,
    SNAP_FLAG_DOUBLE_PRECISION, SNAP_FLAG_VALIDATION, SNAP_LAYOUT_VERSION, SNAP_MAGIC,
    SNAP_VERSION,
};
