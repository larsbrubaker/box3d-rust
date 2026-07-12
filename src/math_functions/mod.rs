//! Vectors, quaternions, transforms, and deterministic scalar math.
//!
//! Core types ([`Vec3`], [`Quat`], [`Pos`], [`Transform`], [`Aabb`], …) and the
//! hand-rolled trig used for cross-platform determinism. Many of these are
//! re-exported at the crate root (`box3d_rust::Vec3`, `Pos`, …).
//!
//! With the `double-precision` feature, [`Pos`] (and related world-space
//! scalars) widen to match `BOX3D_DOUBLE_PRECISION`; collision math stays `f32`.
//!
//! Port of `include/box3d/math_functions.h` and `src/math_functions.c`.
//! Submodules form one flat namespace via re-exports here.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

// Float literals are written with the exact digits of the C source, and Pos math casts
// through PosScalar so the same expression compiles in both precision modes (the cast is
// a no-op in single precision). These allows cascade into the submodules below.
#![allow(clippy::excessive_precision)]
#![allow(clippy::unnecessary_cast)]

mod internal;
mod matrix;
mod quat;
mod query;
mod ray_triangle;
mod scalar;
mod transform;
mod types;
mod validate;
mod vector;

pub use internal::*;
pub use matrix::*;
pub use quat::*;
pub use query::*;
pub use ray_triangle::{
    intersect_ray_triangle, test_bounds_overlap, test_bounds_ray_overlap,
    test_bounds_triangle_overlap,
};
pub use scalar::*;
pub use transform::*;
pub use types::*;
pub use validate::*;
pub use vector::*;
