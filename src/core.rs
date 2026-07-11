// Port of the surviving behavior from box3d-cpp-reference/src/core.c, core.h,
// and include/box3d/base.h needed by math_functions / constants.
//
// core.c is largely an allocator / threading / timing shim. Rust covers that
// natively, so those pieces are not ported. What remains here is the runtime
// length-unit scale and the precision query.
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
