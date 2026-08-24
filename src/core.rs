// Port of the surviving behavior from box3d-cpp-reference/src/core.c, core.h,
// include/box3d/base.h, and src/ctz.h needed by math_functions / constants /
// bitset / table.
//
// core.c is largely an allocator / threading / timing shim. Rust covers that
// natively, so those pieces are not ported. What remains here is the runtime
// length-unit scale, the precision query, and the ctz.h bit helpers.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use core::sync::atomic::{AtomicU32, Ordering};

use crate::rapidhash::rapidhash;

/// Used to indicate an unset or invalid index value. (base.h: B3_NULL_INDEX)
pub const NULL_INDEX: i32 = -1;

/// Use to validate definitions. (core.h: B3_SECRET_COOKIE)
pub const SECRET_COOKIE: i32 = 1152023;

// The length-unit scale is a single global that the user sets once at startup.
// C stores it as a plain `static float`; we store the bit pattern in an atomic
// so the global is sound under Rust's threading rules. The observable value is
// identical. 0x3F80_0000 is the bit pattern of 1.0f32.
static LENGTH_UNITS_PER_METER_BITS: AtomicU32 = AtomicU32::new(0x3F80_0000);

/// Box3D bases all length units on meters. Set this to use different units for
/// all length values passed to and returned from Box3D. Must be set at
/// application startup, before any other Box3D calls.
pub fn set_length_units_per_meter(length_units: f32) {
    debug_assert!(crate::math_functions::is_valid_float(length_units) && length_units > 0.0);
    LENGTH_UNITS_PER_METER_BITS.store(length_units.to_bits(), Ordering::Relaxed);
}

/// Get the current length units per meter.
pub fn get_length_units_per_meter() -> f32 {
    f32::from_bits(LENGTH_UNITS_PER_METER_BITS.load(Ordering::Relaxed))
}

// The CCD stall threshold is a single global in C (`static float
// b3_stallThreshold = FLT_MAX`) used only to log continuous-collision steps that
// exceed it (solver.c / shape.c). Stored as an atomic bit pattern for the same
// soundness reason as the length unit above. 0x7F7F_FFFF is FLT_MAX.
static STALL_THRESHOLD_BITS: AtomicU32 = AtomicU32::new(0x7F7F_FFFF);

/// Set the CCD stall threshold, in seconds. (core.c: b3SetStallThreshold)
pub fn set_stall_threshold(seconds: f32) {
    debug_assert!(crate::math_functions::is_valid_float(seconds) && seconds > 0.0);
    STALL_THRESHOLD_BITS.store(seconds.to_bits(), Ordering::Relaxed);
}

/// Get the CCD stall threshold, in seconds. (core.c: b3GetStallThreshold)
pub fn get_stall_threshold() -> f32 {
    f32::from_bits(STALL_THRESHOLD_BITS.load(Ordering::Relaxed))
}

/// @return true if the library was built with the `double-precision` feature
/// (large world mode), mirroring `BOX3D_DOUBLE_PRECISION`.
pub fn is_double_precision() -> bool {
    cfg!(feature = "double-precision")
}

// ---------------------------------------------------------------------------
// Bit helpers (ctz.h). The C versions are thin wrappers over compiler
// intrinsics (__builtin_ctz / _BitScanForward / __popcnt). The count-leading
// and count-trailing intrinsics are undefined for a zero argument in C; every
// caller guarantees a nonzero argument, and Rust's intrinsics are well defined
// (returning the bit width) even for zero, so the ported callers behave
// identically.
// ---------------------------------------------------------------------------

/// Count trailing zeros of a 32-bit block. (ctz.h: b3CTZ32)
pub fn ctz32(block: u32) -> u32 {
    block.trailing_zeros()
}

/// Count leading zeros of a 32-bit value. (ctz.h: b3CLZ32)
pub fn clz32(value: u32) -> u32 {
    value.leading_zeros()
}

/// Count trailing zeros of a 64-bit block. (ctz.h: b3CTZ64)
pub fn ctz64(block: u64) -> u32 {
    block.trailing_zeros()
}

/// Population count of a 64-bit block. (ctz.h: b3PopCount64)
pub fn pop_count64(block: u64) -> i32 {
    block.count_ones() as i32
}

/// (ctz.h: b3IsPowerOf2)
pub fn is_power_of2(x: i32) -> bool {
    (x & (x - 1)) == 0
}

/// (ctz.h: b3BoundingPowerOf2)
pub fn bounding_power_of2(x: i32) -> i32 {
    if x <= 1 {
        return 1;
    }

    32 - clz32((x as u32) - 1) as i32
}

/// (ctz.h: b3RoundUpPowerOf2)
pub fn round_up_power_of2(x: i32) -> i32 {
    if x <= 1 {
        return 1;
    }

    1 << (32 - clz32((x as u32) - 1))
}

/// Position of the most significant bit = floor(log2(x)). (ctz.h: b3LowerPowerOf2Exponent)
pub fn lower_power_of_2_exponent(x: i32) -> i32 {
    debug_assert!(x > 0);
    let clz = clz32(x as u32) as i32;

    // Position of most significant bit = floor(log2(M))
    31 - clz
}

// ---------------------------------------------------------------------------
// Content hash (base.h / timer.c). Word-oriented djb2 over 8-byte little-endian
// chunks — not the byte-wise recurrence used by Box2D.
// ---------------------------------------------------------------------------

/// Initial value for [`hash`]. (base.h: B3_HASH_INIT)
pub const HASH_INIT: u32 = 5381;

/// Hash `data` into `hash` (djb2-style, 8-byte little-endian words then bytes).
/// (timer.c: b3Hash)
pub fn hash(hash: u32, data: &[u8]) -> u32 {
    let mut result = hash;
    let mut i = 0;
    let count = data.len();

    while i + 8 <= count {
        // Little-endian load; matches memcpy of uint64_t on LE hosts (and the
        // explicit byte-swap path on BE in the C source).
        let word = u64::from_le_bytes(data[i..i + 8].try_into().unwrap());
        result = result
            .wrapping_shl(5)
            .wrapping_add(result)
            .wrapping_add(word as u32);
        result = result
            .wrapping_shl(5)
            .wrapping_add(result)
            .wrapping_add((word >> 32) as u32);
        i += 8;
    }

    while i < count {
        result = result
            .wrapping_shl(5)
            .wrapping_add(result)
            .wrapping_add(data[i] as u32);
        i += 1;
    }

    result
}

/// 64-bit content hash (rapidhash) that reserves zero to mean unhashed.
/// An empty blob has no content to mix, so it takes the reserved value.
/// (core.c: b3Hash64NonZero)
pub fn hash64_non_zero(bytes: &[u8]) -> u64 {
    // C guards on `n <= 0`; the empty slice is the Rust equivalent.
    if bytes.is_empty() {
        return 1;
    }

    let h = rapidhash(bytes);
    if h == 0 {
        1
    } else {
        h
    }
}

/// Geometry content hashes reserve zero to mean unhashed. (core.h: b3NonZeroHash)
pub fn non_zero_hash(hash: u32) -> u32 {
    if hash != 0 {
        hash
    } else {
        1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bit_helpers() {
        assert_eq!(ctz32(0b1000), 3);
        assert_eq!(clz32(1), 31);
        assert_eq!(clz32(9), 31 - 3);
        assert_eq!(ctz64(1u64 << 40), 40);
        assert_eq!(pop_count64(0xFFFF_FFFF_FFFF_FFFF), 64);
        assert!(is_power_of2(8));
        assert!(!is_power_of2(6));
        assert_eq!(round_up_power_of2(5), 8);
        assert_eq!(round_up_power_of2(1), 1);
        assert_eq!(bounding_power_of2(5), 3);
        assert_eq!(lower_power_of_2_exponent(9), 3);
    }

    #[test]
    fn stall_threshold_round_trip() {
        // Default mirrors C's `b3_stallThreshold = FLT_MAX`.
        assert_eq!(get_stall_threshold(), f32::MAX);

        // Matches sample_continuous.cpp Stall: b3SetStallThreshold(0.001f).
        set_stall_threshold(0.001);
        assert_eq!(get_stall_threshold(), 0.001);

        // Restore the default so other tests observe C's initial value.
        set_stall_threshold(f32::MAX);
        assert_eq!(get_stall_threshold(), f32::MAX);
    }
}
