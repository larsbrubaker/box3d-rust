//! Contact manifold generation for convex primitive pairs and mesh narrow phase.
//!
//! Port of `box3d-cpp-reference/src/convex_manifold.c` (sphere / capsule / hull
//! pairs), `triangle_manifold.c`, plus the clip/edge helpers from
//! `manifold.h` / `manifold.c`. Mesh contact assembly lives in `contact/mesh_contact.rs`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

mod capsules;
mod clip;
mod hull_capsule;
mod hulls;
mod sat;
mod separating_axis;
mod simd_scalar;
mod spheres;
mod triangle;
mod triangle_face;
mod triangle_hull;
mod types;

pub use capsules::collide_capsules;
pub use clip::{find_incident_face, flip_pair};
pub use hull_capsule::collide_hull_and_capsule;
pub use hulls::collide_hulls;
pub use spheres::{collide_capsule_and_sphere, collide_hull_and_sphere, collide_spheres};
pub use triangle::{collide_triangle_and_capsule, collide_triangle_and_sphere};
pub use triangle_hull::collide_triangle_and_hull;
pub use types::{
    make_feature_id, make_feature_pair, FeatureOwner, FeaturePair, LocalManifold,
    LocalManifoldPoint, Manifold, ManifoldPoint, SatCache, SeparatingFeature, TriangleFeature,
    FEATURE_PAIR_SINGLE,
};

#[cfg(test)]
pub(crate) use separating_axis::compute_separating_axis;
#[cfg(test)]
pub(crate) use types::get_best_axis;
