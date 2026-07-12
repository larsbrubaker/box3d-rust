//! Public definition types and C-compatible defaults.
//!
//! Every create API expects a def built from the matching `default_*_def`
//! helper — for example [`default_world_def`], [`default_body_def`],
//! [`default_shape_def`], and the joint defaults. These mirror
//! `b3Default*Def` in the C API (including the internal cookie).
//!
//! Port of `include/box3d/types.h` and `src/types.c`. Split by domain with flat
//! re-exports for callers.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

mod body;
mod joint;
mod shape;
mod world;

pub use body::*;
pub use joint::*;
pub use shape::*;
pub use world::*;
