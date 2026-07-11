//! Contact manifold generation for convex primitive pairs.
//!
//! Port of the sphere / capsule / hull-sphere slice of
//! `box3d-cpp-reference/src/convex_manifold.c`, plus the feature-pair helpers
//! from `manifold.h` / `manifold.c` needed by those collide functions.
//!
//! Deferred (later slices): hull-hull SAT/clip, hull-capsule, triangle, mesh,
//! `b3ReduceManifoldPoints`, `b3ClipPolygon`, `b3FlipPair`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

mod capsules;
mod spheres;
mod types;

pub use capsules::collide_capsules;
pub use spheres::{collide_capsule_and_sphere, collide_hull_and_sphere, collide_spheres};
pub use types::{
    make_feature_id, make_feature_pair, FeatureOwner, FeaturePair, LocalManifold,
    LocalManifoldPoint, TriangleFeature, FEATURE_PAIR_SINGLE,
};
