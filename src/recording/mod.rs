//! Recording, replay, and world snapshots.
//!
//! Port of recording.c / recording.h / world_snapshot.c from the C reference.
//! Geometry is interned by content hash; world snapshots use field-by-field
//! serialization adapted to Vec-backed pools (not C POD memcpy).
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

mod buffer;
mod hash;
mod registry;
pub mod snapshot;

pub use buffer::{RecBuffer, SnapReader};
pub use hash::{fnv_mix_position, hash_world_state, SNAP_FNV_INIT, SNAP_FNV_PRIME};
pub use registry::{
    hash64_blob, GeometryEntry, GeometryKind, GeometryRegistry, RegistrySlot,
};
pub use snapshot::{
    clone_world_via_snapshot, compute_layout_hash, deserialize_into_shell, serialize_world,
    SNAP_FLAG_DOUBLE_PRECISION, SNAP_FLAG_VALIDATION, SNAP_LAYOUT_VERSION, SNAP_MAGIC,
    SNAP_VERSION,
};
