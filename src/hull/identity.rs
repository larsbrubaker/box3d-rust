//! Hull identity: content hash and byte-wise compare.
//!
//! The verstable `b3HullMap` is deferred — only the hash/compare primitives used by
//! identity and serialization are ported here.

use super::types::HullData;

/// Spread the baked 32-bit content hash across 64 bits. (b3HashHullData)
pub fn hash_hull_data(hull: &HullData) -> u64 {
    (hull.hash as u64).wrapping_mul(0x9E3779B97F4A7C15)
}

/// Compare two hulls by C contiguous-blob equality. (b3CompareHullData)
pub fn compare_hull_data(hull1: &HullData, hull2: &HullData) -> bool {
    if core::ptr::eq(hull1, hull2) {
        return true;
    }
    if hull1.byte_count != hull2.byte_count {
        return false;
    }
    hull1.to_bytes() == hull2.to_bytes()
}
