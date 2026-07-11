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
