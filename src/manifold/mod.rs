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
mod edge_candidates;
mod hull_capsule;
mod hulls;
mod sat;
mod separating_axis;
mod spheres;
mod triangle;
mod triangle_face;
mod triangle_hull;
mod types;

pub use capsules::collide_capsules;
pub use clip::{find_incident_face, flip_pair};
pub use hull_capsule::collide_hull_and_capsule;
pub use hulls::collide_hulls;
pub(crate) use hulls::collide_hulls_at_width;
pub use spheres::{collide_capsule_and_sphere, collide_hull_and_sphere, collide_spheres};
pub(crate) use triangle::build_triangle_and_capsule_face_contact;
pub use triangle::{collide_triangle_and_capsule, collide_triangle_and_sphere};
pub(crate) use triangle_hull::build_triangle_and_hull_face_contact;
pub use triangle_hull::collide_triangle_and_hull;
pub(crate) use types::MAX_CLIP_POINTS;
pub use types::{
    make_feature_id, make_feature_pair, FeatureOwner, FeaturePair, LocalManifold,
    LocalManifoldPoint, Manifold, ManifoldPoint, SatCache, SeparatingFeature, TriangleFeature,
    FEATURE_PAIR_SINGLE, LOCAL_MANIFOLD_CAPACITY, MESH_CONTACT_DEEP, MESH_CONTACT_HULL_FACE,
    MESH_CONTACT_SNAPPED,
};

#[cfg(test)]
pub(crate) use edge_candidates::test_edge_candidate_sorted;
#[cfg(test)]
pub(crate) use separating_axis::compute_separating_axis;
#[cfg(test)]
pub(crate) use types::get_best_axis;
