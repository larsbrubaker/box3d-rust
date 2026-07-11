//! Shape module public create API re-exports.
//!
//! Typed wrappers live in [`lifecycle`]; this file keeps the public surface flat.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

pub use super::lifecycle::{create_capsule_shape, create_sphere_shape, get_shape};
