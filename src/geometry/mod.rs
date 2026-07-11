//! Shape geometry queries: sphere, capsule, and hull mass/AABB/overlap/cast.
//!
//! Port of `box3d-cpp-reference/src/sphere.c`, `capsule.c`, and the query APIs
//! at the end of `hull.c`. Types come from `include/box3d/types.h`.
//!
//! Layout (mirrors box2d-rust `geometry/`):
//! - `types.rs`  — MassData, Sphere, Capsule, RayCastInput, ShapeCastInput
//! - `sphere.rs` — sphere mass, AABB, overlap, ray/shape cast
//! - `capsule.rs` — capsule mass, AABB, overlap, ray/shape cast
//!
//! Hull queries live in [`crate::hull::queries`] and are re-exported here.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

mod capsule;
mod sphere;
pub(crate) mod types;

pub use capsule::{
    collide_mover_and_capsule, compute_capsule_aabb, compute_capsule_mass,
    compute_swept_capsule_aabb, overlap_capsule, ray_cast_capsule, shape_cast_capsule,
};
pub use sphere::{
    collide_mover_and_sphere, compute_sphere_aabb, compute_sphere_mass, compute_swept_sphere_aabb,
    overlap_sphere, ray_cast_hollow_sphere, ray_cast_sphere, shape_cast_sphere,
};
pub use types::{
    default_surface_material, Capsule, CollisionPlane, MassData, PlaneResult, PlaneSolverResult,
    RayCastInput, ShapeCastInput, ShapeExtent, ShapeType, Sphere, SurfaceMaterial,
    SURFACE_MATERIAL_SIZE,
};

pub use crate::distance::CastOutput;
pub use crate::hull::{
    collide_mover_and_hull, compute_hull_aabb, compute_hull_mass, compute_swept_hull_aabb,
    overlap_hull, ray_cast_hull, shape_cast_hull,
};

use crate::constants::huge;
use crate::math_functions::{is_valid_float, is_valid_vec3};

/// Validate ray cast input data (NaN, etc). (b3IsValidRay)
pub fn is_valid_ray(input: &RayCastInput) -> bool {
    is_valid_vec3(input.origin)
        && is_valid_vec3(input.translation)
        && is_valid_float(input.max_fraction)
        && 0.0 <= input.max_fraction
        && input.max_fraction < huge()
}
