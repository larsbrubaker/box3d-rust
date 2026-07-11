// Port of box3d-cpp-reference/include/box3d/constants.h
//
// The C header defines these as macros. Constants that do not depend on runtime
// state are `pub const`. Those defined in terms of `b3GetLengthUnitsPerMeter()`
// re-read that global on every use, so they are ported as functions to preserve
// that behavior exactly.
//
// For now only the pieces needed by math validators (B3_HUGE) and a few related
// length-scaled constants are included; the rest land with the modules that use them.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::get_length_units_per_meter;

/// Used to detect bad values. In float mode positions greater than about 16km have
/// precision problems, so 100km is a safe limit. Large world mode keeps coordinates
/// accurate much farther from the origin, so the sanity limit widens. (B3_HUGE)
#[cfg(feature = "double-precision")]
pub fn huge() -> f32 {
    1.0e9 * get_length_units_per_meter()
}

/// See [`huge`].
#[cfg(not(feature = "double-precision"))]
pub fn huge() -> f32 {
    1.0e5 * get_length_units_per_meter()
}

/// Bit width reserved for a shape index in [`crate::table::shape_pair_key`]. (B3_SHAPE_POWER)
pub const SHAPE_POWER: u32 = 22;

/// Bit width reserved for a child index in the pair key. (B3_CHILD_POWER)
pub const CHILD_POWER: u32 = 64 - 2 * SHAPE_POWER;

/// Maximum number of shapes. (B3_MAX_SHAPES)
pub const MAX_SHAPES: i32 = 1 << SHAPE_POWER;

/// Maximum number of child shapes. (B3_MAX_CHILD_SHAPES)
pub const MAX_CHILD_SHAPES: i32 = 1 << CHILD_POWER;

/// Mask for a shape index packed into a pair key. (B3_SHAPE_MASK)
pub const SHAPE_MASK: u64 = (MAX_SHAPES as u64) - 1;

/// Mask for a child index packed into a pair key. (B3_CHILD_MASK)
pub const CHILD_MASK: u64 = (MAX_CHILD_SHAPES as u64) - 1;

const _: () = assert!(2 * SHAPE_POWER + CHILD_POWER == 64);
const _: () = assert!(CHILD_POWER > 8);
