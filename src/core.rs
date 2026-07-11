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

/// Used to indicate an unset or invalid index value. (base.h: B3_NULL_INDEX)
pub const NULL_INDEX: i32 = -1;

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
}
