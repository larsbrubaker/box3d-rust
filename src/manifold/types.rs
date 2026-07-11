//! Manifold and feature-pair types from `include/box3d/types.h` and
//! `src/manifold.h`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::constants::MAX_MANIFOLD_POINTS;
use crate::math_functions::{Vec3, VEC3_ZERO};

/// Which shape owns a clipped feature edge. (b3FeatureOwner)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum FeatureOwner {
    #[default]
    ShapeA = 0,
    ShapeB = 1,
}

/// Cached triangle feature for mesh/height-field contacts. (b3TriangleFeature)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum TriangleFeature {
    #[default]
    None = 0,
    TriangleFace = 1,
    HullFace = 2,
    /// v1-v2
    Edge1 = 3,
    /// v2-v3
    Edge2 = 4,
    /// v3-v1
    Edge3 = 5,
    Vertex1 = 6,
    Vertex2 = 7,
    Vertex3 = 8,
}

/// Contact points are always the result of two edges intersecting.
/// It can be two edges of the same shape, which is just a shape vertex.
/// Or a contact point can be the result of two edges crossing from different shapes.
/// (b3FeaturePair)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FeaturePair {
    /// Incoming type (either edge on shape A or shape B)
    pub owner1: u8,
    /// Incoming edge index (into associated shape array)
    pub index1: u8,
    /// Outgoing type (either edge on shape A or shape B)
    pub owner2: u8,
    /// Outgoing edge index (into associated shape array)
    pub index2: u8,
}

/// For single point contact, such as sphere-sphere, sphere-capsule, sphere-triangle.
/// (b3FeaturePair_single)
pub const FEATURE_PAIR_SINGLE: FeaturePair = FeaturePair {
    owner1: 0,
    index1: 0,
    owner2: 0,
    index2: 0,
};

/// Build a feature pair. (b3MakeFeaturePair)
pub fn make_feature_pair(
    owner1: FeatureOwner,
    index1: i32,
    owner2: FeatureOwner,
    index2: i32,
) -> FeaturePair {
    debug_assert!((0..=u8::MAX as i32).contains(&index1));
    debug_assert!((0..=u8::MAX as i32).contains(&index2));
    FeaturePair {
        owner1: owner1 as u8,
        index1: index1 as u8,
        owner2: owner2 as u8,
        index2: index2 as u8,
    }
}

/// Pack a feature pair into a 32-bit id. (b3MakeFeatureId)
pub fn make_feature_id(pair: FeaturePair) -> u32 {
    ((pair.owner1 as u32) << 24)
        | ((pair.index1 as u32) << 16)
        | ((pair.owner2 as u32) << 8)
        | (pair.index2 as u32)
}

/// A local manifold point and normal in frame A. (b3LocalManifoldPoint)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LocalManifoldPoint {
    /// Local point in frame A.
    pub point: Vec3,
    /// The contact point separation. Negative for overlap.
    pub separation: f32,
    /// The feature pair for this point.
    pub pair: FeaturePair,
    /// The triangle index when collide with a mesh or height-field.
    pub triangle_index: i32,
}

/// A local manifold with no dynamic information. Used by collide functions.
///
/// Unlike C (which holds a `b3LocalManifoldPoint*` into an external buffer),
/// this embeds a fixed [`MAX_MANIFOLD_POINTS`] point array. Callers pass
/// `capacity` to the collide functions, matching the C API.
/// (b3LocalManifold)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalManifold {
    /// Local normal in frame A.
    pub normal: Vec3,
    /// The triangle normal.
    pub triangle_normal: Vec3,
    /// The manifold points.
    pub points: [LocalManifoldPoint; MAX_MANIFOLD_POINTS],
    /// The number of manifold points. Only bounded by the buffer capacity.
    pub point_count: i32,
    /// The index of the triangle.
    pub triangle_index: i32,
    /// Vertex 1 index.
    pub i1: i32,
    /// Vertex 2 index.
    pub i2: i32,
    /// Vertex 3 index.
    pub i3: i32,
    /// The squared distance of a sphere from a triangle. For ghost collision reduction.
    pub squared_distance: f32,
    /// The triangle feature involved.
    pub feature: TriangleFeature,
    /// b3MeshEdgeFlags.
    pub triangle_flags: i32,
}

impl Default for LocalManifold {
    fn default() -> Self {
        LocalManifold {
            normal: VEC3_ZERO,
            triangle_normal: VEC3_ZERO,
            points: [LocalManifoldPoint::default(); MAX_MANIFOLD_POINTS],
            point_count: 0,
            triangle_index: 0,
            i1: 0,
            i2: 0,
            i3: 0,
            squared_distance: 0.0,
            feature: TriangleFeature::None,
            triangle_flags: 0,
        }
    }
}

/// Clip vertex used by capsule/hull clipping. (b3ClipVertex)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct ClipVertex {
    pub position: Vec3,
    pub separation: f32,
    pub pair: FeaturePair,
}
