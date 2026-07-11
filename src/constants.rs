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
use crate::math_functions::PI;

/// Maximum body rotation per time step to prevent numerical issues. (B3_MAX_ROTATION)
pub const MAX_ROTATION: f32 = 0.25 * PI;

/// Velocity constraint iterations per sub-step. (solver.c: ITERATIONS)
pub const SOLVER_ITERATIONS: i32 = 1;

/// Relaxation iterations per sub-step (bias off). (solver.c: RELAX_ITERATIONS)
pub const RELAX_ITERATIONS: i32 = 1;

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

/// A small length used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant. In meters.
/// @warning modifying this can have a significant impact on stability
/// (B3_LINEAR_SLOP)
pub fn linear_slop() -> f32 {
    0.005 * get_length_units_per_meter()
}

/// Used to determine if two shapes are overlapping. Typically about 10% of
/// [`linear_slop`]. (B3_OVERLAP_SLOP)
pub fn overlap_slop() -> f32 {
    0.1 * linear_slop()
}

/// The maximum number of points to use for shape cast proxies (swept point cloud).
/// (B3_MAX_SHAPE_CAST_POINTS)
pub const MAX_SHAPE_CAST_POINTS: usize = 64;

/// The maximum number of contact points between two touching shapes.
/// (B3_MAX_MANIFOLD_POINTS)
pub const MAX_MANIFOLD_POINTS: usize = 4;

/// Maximum number of colors in the constraint graph. Constraints that cannot
/// find a color are added to the overflow set. (B3_GRAPH_COLOR_COUNT)
pub const GRAPH_COLOR_COUNT: i32 = 24;

/// Contact-point buckets for reporting manifold counts per pair.
/// (B3_CONTACT_MANIFOLD_COUNT_BUCKETS)
pub const CONTACT_MANIFOLD_COUNT_BUCKETS: usize = 8;

/// Time a body must be still before it will go to sleep, in seconds.
/// (B3_TIME_TO_SLEEP)
pub const TIME_TO_SLEEP: f32 = 0.5;

/// Null name id in the name cache. (B3_NULL_NAME)
pub const NULL_NAME: u32 = 0;

/// Used to determine if two shapes are overlapping. Typically about 4×
/// [`linear_slop`]. (B3_SPECULATIVE_DISTANCE)
pub fn speculative_distance() -> f32 {
    4.0 * linear_slop()
}

/// Rest offset for mesh contact to reduce ghost collisions. Must be at least
/// [`linear_slop`] and less than [`speculative_distance`]. (B3_MESH_REST_OFFSET)
pub fn mesh_rest_offset() -> f32 {
    1.0 * linear_slop()
}

/// Angular distance threshold for contact recycling (cos² of half-angle).
/// (B3_CONTACT_RECYCLE_ANGULAR_DISTANCE)
pub const CONTACT_RECYCLE_ANGULAR_DISTANCE: f32 = 0.99240388;

/// Default contact recycling distance. (B3_CONTACT_RECYCLE_DISTANCE)
pub fn contact_recycle_distance() -> f32 {
    10.0 * linear_slop()
}

/// Minimum capsule segment length. (B3_MIN_CAPSULE_LENGTH)
pub fn min_capsule_length() -> f32 {
    linear_slop()
}

/// Maximum AABB margin used when expanding bounds for casts.
/// (B3_MAX_AABB_MARGIN)
pub fn max_aabb_margin() -> f32 {
    0.05 * get_length_units_per_meter()
}

/// Fraction of shape size used for the AABB movement margin.
/// (B3_AABB_MARGIN_FRACTION)
pub const AABB_MARGIN_FRACTION: f32 = 0.125;
