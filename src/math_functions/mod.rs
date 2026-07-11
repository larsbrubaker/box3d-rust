// Port of box3d-cpp-reference/include/box3d/math_functions.h and src/math_functions.c
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
//
// Split into focused submodules to stay under the project file-length limit. The
// submodules form one flat namespace: everything is re-exported here, so callers
// use `crate::math_functions::<name>` exactly as before.

// Float literals are written with the exact digits of the C source, and Pos math casts
// through PosScalar so the same expression compiles in both precision modes (the cast is
// a no-op in single precision). These allows cascade into the submodules below.
#![allow(clippy::excessive_precision)]
#![allow(clippy::unnecessary_cast)]

mod internal;
mod matrix;
mod query;
mod quat;
mod ray_triangle;
mod scalar;
mod transform;
mod types;
mod validate;
mod vector;

pub use internal::*;
pub use matrix::*;
pub use query::*;
pub use quat::*;
pub use ray_triangle::{
    intersect_ray_triangle, test_bounds_overlap, test_bounds_ray_overlap,
    test_bounds_triangle_overlap,
};
pub use scalar::*;
pub use transform::*;
pub use types::*;
pub use validate::*;
pub use vector::*;
