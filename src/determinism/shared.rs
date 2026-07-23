//! Shared byte-layout helpers for the determinism scenes.
//!
//! The content hash itself (`b3Hash` / [`crate::core::hash`]) already lives in
//! [`crate::core`]; these helpers add the world-transform and vector byte layout
//! used by every scene.

use crate::core::hash;
use crate::math_functions::{Vec3, WorldTransform};

/// Hash a world transform's raw little-endian field bytes in C declaration
/// order, matching `b3Hash(hash, (uint8_t*)&xf, sizeof(b3WorldTransform))` on
/// little-endian hosts. (determinism.c UpdateFallingRagdolls)
pub fn hash_world_transform(h: u32, xf: &WorldTransform) -> u32 {
    let mut bytes = Vec::with_capacity(40);
    bytes.extend_from_slice(&xf.p.x.to_le_bytes());
    bytes.extend_from_slice(&xf.p.y.to_le_bytes());
    bytes.extend_from_slice(&xf.p.z.to_le_bytes());
    bytes.extend_from_slice(&xf.q.v.x.to_le_bytes());
    bytes.extend_from_slice(&xf.q.v.y.to_le_bytes());
    bytes.extend_from_slice(&xf.q.v.z.to_le_bytes());
    bytes.extend_from_slice(&xf.q.s.to_le_bytes());
    hash(h, &bytes)
}

/// Hash a `b3Vec3`'s little-endian field bytes, matching `b3Hash(h, &vec, sizeof(b3Vec3))`.
pub(crate) fn vec3_bytes(v: Vec3) -> [u8; 12] {
    let mut bytes = [0u8; 12];
    bytes[0..4].copy_from_slice(&v.x.to_le_bytes());
    bytes[4..8].copy_from_slice(&v.y.to_le_bytes());
    bytes[8..12].copy_from_slice(&v.z.to_le_bytes());
    bytes
}
