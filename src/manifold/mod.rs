//! Contact manifold generation for convex primitive pairs.
//!
//! Port of `box3d-cpp-reference/src/convex_manifold.c` (sphere / capsule / hull
//! pairs) plus the clip/edge helpers from `manifold.h` / `manifold.c`.
//!
//! Deferred: triangle_manifold.c, mesh_contact.c.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

mod capsules;
mod clip;
mod hull_capsule;
mod hulls;
mod sat;
mod spheres;
mod types;

pub use capsules::collide_capsules;
pub use clip::{edge_edge_separation, find_incident_face, flip_pair};
pub use hull_capsule::collide_hull_and_capsule;
pub use hulls::collide_hulls;
pub use spheres::{collide_capsule_and_sphere, collide_hull_and_sphere, collide_spheres};
pub use types::{
    make_feature_id, make_feature_pair, FeatureOwner, FeaturePair, LocalManifold,
    LocalManifoldPoint, SatCache, SeparatingFeature, TriangleFeature, FEATURE_PAIR_SINGLE,
};
