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

// This gives debug information about how a contact point was generated. (b3MeshContactFlags)

/// Normal is a hull face normal. (b3_meshContactHullFace)
pub const MESH_CONTACT_HULL_FACE: u8 = 0x01;

/// The normal is snapped to the triangle normal. (b3_meshContactSnapped)
pub const MESH_CONTACT_SNAPPED: u8 = 0x02;

/// The contact is considered deep (2 * linear slop). (b3_meshContactDeep)
pub const MESH_CONTACT_DEEP: u8 = 0x04;

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
    /// [`TriangleFeature`] as a byte. (b3TriangleFeature)
    pub triangle_feature: u8,
    /// `MESH_CONTACT_*` bits. (b3MeshContactFlags)
    pub mesh_flags: u8,
}

/// The most points a single collide call writes. C points a local manifold into a caller buffer
/// and passes the room left as the capacity. The largest single manifold is a triangle face clip,
/// which C builds in a `2 * B3_MAX_CLIP_POINTS` buffer, so a window of this many points never
/// truncates a clip that C keeps.
pub const LOCAL_MANIFOLD_CAPACITY: usize = 2 * MAX_CLIP_POINTS;

/// A local manifold with no dynamic information. Used by collide functions.
///
/// As in C, the points live in a caller buffer: `points` borrows the buffer and the collide
/// functions take its room as their `capacity`, so a collide call never fills point storage it
/// does not write. Convex callers use a [`MAX_MANIFOLD_POINTS`] buffer; the mesh narrow phase
/// points each triangle manifold at a window of its shared point buffer.
///
/// ```
/// use box3d_rust::constants::MAX_MANIFOLD_POINTS;
/// use box3d_rust::manifold::{LocalManifold, LocalManifoldPoint};
///
/// let mut points = [LocalManifoldPoint::default(); MAX_MANIFOLD_POINTS];
/// let manifold = LocalManifold::new(&mut points);
/// assert_eq!(manifold.point_count, 0);
/// ```
/// (b3LocalManifold)
#[derive(Debug, PartialEq)]
pub struct LocalManifold<'a> {
    /// Local normal in frame A.
    pub normal: Vec3,
    /// The triangle normal.
    pub triangle_normal: Vec3,
    /// The manifold points. From a point buffer.
    pub points: &'a mut [LocalManifoldPoint],
    /// The number of manifold points. Only bounded by the buffer capacity.
    pub point_count: i32,
    /// The index of the triangle.
    pub triangle_index: i32,
    /// The triangle feature involved.
    pub feature: TriangleFeature,
}

impl<'a> LocalManifold<'a> {
    /// An empty manifold that writes its points into `points`. The buffer is not cleared; the
    /// collide functions write every point they report.
    /// (`b3LocalManifold manifold = { 0 }; manifold.points = points;`)
    pub fn new(points: &'a mut [LocalManifoldPoint]) -> Self {
        LocalManifold {
            normal: VEC3_ZERO,
            triangle_normal: VEC3_ZERO,
            points,
            point_count: 0,
            triangle_index: 0,
            feature: TriangleFeature::None,
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

/// Maximum vertices in a clipped polygon buffer. (B3_MAX_CLIP_POINTS)
pub(crate) const MAX_CLIP_POINTS: usize = 64;

/// A single separating axis (face-A, face-B, or edge-pair). (b3SeparatingAxis)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct SeparatingAxis {
    pub normal: Vec3,
    pub separation: f32,
    pub index_a: i32,
    pub index_b: i32,
    /// [`SeparatingFeature`] describing which axis type this is.
    pub type_: SeparatingFeature,
}

/// Separating axis query result from the SIMD hull collision path. Holds the best
/// axis of each of the three feature types plus the feature that separated the hulls
/// (if any). (b3AxisQuery)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct AxisQuery {
    pub face_a: SeparatingAxis,
    pub face_b: SeparatingAxis,
    pub edge: SeparatingAxis,
    /// The feature that separated the hulls, or [`SeparatingFeature::InvalidAxis`] if
    /// the hulls are not separated. (separatedFeature)
    pub separated_feature: SeparatingFeature,
}

/// Return the deepest of the three axes. Used by SAT tests. (b3GetBestAxis)
#[cfg(test)]
pub(crate) fn get_best_axis(query: &AxisQuery) -> SeparatingAxis {
    debug_assert!(query.face_a.type_ == SeparatingFeature::FaceAxisA);
    debug_assert!(query.edge.type_ == SeparatingFeature::EdgePairAxis);
    debug_assert!(query.face_b.type_ == SeparatingFeature::FaceAxisB);

    if query.face_a.separation > query.face_b.separation {
        if query.edge.separation > query.face_a.separation {
            return query.edge;
        }

        return query.face_a;
    }

    if query.edge.separation > query.face_b.separation {
        return query.edge;
    }

    query.face_b
}

/// Cached separating axis feature. (b3SeparatingFeature)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum SeparatingFeature {
    #[default]
    InvalidAxis = 0,
    BacksideAxis = 1,
    FaceAxisA = 2,
    FaceAxisB = 3,
    EdgePairAxis = 4,
    ClosestPointsAxis = 5,
    /// Testing only
    ManualFaceAxisA = 6,
    /// Testing only
    ManualFaceAxisB = 7,
    /// Testing only
    ManualEdgePairAxis = 8,
}

/// Separating axis test cache. Provides temporal acceleration of collision routines.
/// (b3SATCache)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SatCache {
    /// The separation when the cache is populated. Negative for overlap.
    pub separation: f32,
    /// [`SeparatingFeature`].
    pub type_: u8,
    /// Index of the feature on shape A.
    pub index_a: u8,
    /// Index of the feature on shape B.
    pub index_b: u8,
    /// Was the cache re-used?
    pub hit: u8,
}

/// A manifold point is a contact point belonging to a contact manifold.
/// Box3D uses speculative collision so some contact points may be separated.
/// (b3ManifoldPoint)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ManifoldPoint {
    /// Location of the contact point relative to body A center of mass in world space.
    pub anchor_a: Vec3,
    /// Location of the contact point relative to body B center of mass in world space.
    pub anchor_b: Vec3,
    /// Separation of the contact point; negative if penetrating.
    pub separation: f32,
    /// Cached separation used for contact recycling.
    pub base_separation: f32,
    /// Impulse along the manifold normal from the final sub-step.
    pub normal_impulse: f32,
    /// Total normal impulse applied during sub-stepping.
    pub total_normal_impulse: f32,
    /// Relative normal velocity pre-solve. Negative means approaching.
    pub normal_velocity: f32,
    /// Uniquely identifies a contact point between two shapes.
    pub feature_id: u32,
    /// Triangle index if one of the shapes is a mesh or height field.
    pub triangle_index: i32,
    /// Did this contact point exist in the previous step?
    pub persisted: bool,
    /// [`TriangleFeature`] as a byte. (b3TriangleFeature)
    pub triangle_feature: u8,
    /// `MESH_CONTACT_*` bits. (b3MeshContactFlags)
    pub mesh_flags: u8,
}

/// A contact manifold describes the contact points between colliding shapes.
/// (b3Manifold)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Manifold {
    /// The manifold points. There may be 0 to [`MAX_MANIFOLD_POINTS`] valid points.
    pub points: [ManifoldPoint; MAX_MANIFOLD_POINTS],
    /// Unit normal in world space, points from shape A to shape B.
    pub normal: Vec3,
    /// Central friction angular impulse (applied about the normal).
    pub twist_impulse: f32,
    /// Central friction linear impulse.
    pub friction_impulse: Vec3,
    /// Rolling resistance angular impulse.
    pub rolling_impulse: Vec3,
    /// The number of contact points, 0 to 4.
    pub point_count: i32,
}

impl Default for Manifold {
    fn default() -> Self {
        Manifold {
            points: [ManifoldPoint::default(); MAX_MANIFOLD_POINTS],
            normal: VEC3_ZERO,
            twist_impulse: 0.0,
            friction_impulse: VEC3_ZERO,
            rolling_impulse: VEC3_ZERO,
            point_count: 0,
        }
    }
}
