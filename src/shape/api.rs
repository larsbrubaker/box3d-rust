//! Shape module public create/destroy API re-exports.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

pub use super::lifecycle::{
    create_capsule_shape, create_compound_shape, create_height_field_shape, create_hull_shape,
    create_mesh_shape, create_sphere_shape, destroy_shape, get_shape, shape_get_hull,
    shape_is_valid,
};

// Accessors / geometry_set are re-exported from mod.rs.
