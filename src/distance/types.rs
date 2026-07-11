// Distance-group types from include/box3d/types.h (query group).
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use crate::constants::MAX_SHAPE_CAST_POINTS;
use crate::core::NULL_INDEX;
use crate::math_functions::{Quat, Transform, Vec3, QUAT_IDENTITY, VEC3_ZERO};

/// A shape proxy is used by the GJK algorithm. It can represent a convex shape.
///
/// Unlike C's `b3ShapeProxy` (which holds a `const b3Vec3*` pointer), this owns
/// a fixed-size point buffer so the proxy is self-contained and `Copy`.
/// (b3ShapeProxy)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShapeProxy {
    /// The point cloud.
    pub points: [Vec3; MAX_SHAPE_CAST_POINTS],
    /// The number of points. Do not exceed [`MAX_SHAPE_CAST_POINTS`].
    pub count: i32,
    /// The external radius of the point cloud.
    pub radius: f32,
}

impl Default for ShapeProxy {
    fn default() -> Self {
        ShapeProxy {
            points: [VEC3_ZERO; MAX_SHAPE_CAST_POINTS],
            count: 0,
            radius: 0.0,
        }
    }
}

/// Used to warm start the GJK simplex. If you call this function multiple times
/// with nearby transforms this might improve performance. Otherwise you can
/// zero initialize this. The distance cache must be initialized to zero on the
/// first call. (b3SimplexCache)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SimplexCache {
    /// Value used to compare length, area, volume of two simplexes.
    pub metric: f32,
    /// The number of stored simplex points
    pub count: u16,
    /// The cached simplex indices on shape A
    pub index_a: [u8; 4],
    /// The cached simplex indices on shape B
    pub index_b: [u8; 4],
}

/// Input parameters for [`shape_cast`](crate::distance::shape_cast).
/// (b3ShapeCastPairInput)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShapeCastPairInput {
    /// The proxy for shape A
    pub proxy_a: ShapeProxy,
    /// The proxy for shape B
    pub proxy_b: ShapeProxy,
    /// Transform of shape B in shape A's frame, the relative pose B in A
    pub transform: Transform,
    /// The translation of shape B, in A's frame
    pub translation_b: Vec3,
    /// The fraction of the translation to consider, typically 1
    pub max_fraction: f32,
    /// Allows shapes with a radius to move slightly closer if already touching
    pub can_encroach: bool,
}

/// Input for [`shape_distance`](crate::distance::shape_distance).
/// (b3DistanceInput)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DistanceInput {
    /// The proxy for shape A
    pub proxy_a: ShapeProxy,
    /// The proxy for shape B
    pub proxy_b: ShapeProxy,
    /// Transform of shape B in shape A's frame, the relative pose B in A
    /// (`inv_mul_transforms(world_a, world_b)`). The query is origin
    /// independent and runs in frame A.
    pub transform: Transform,
    /// Should the proxy radius be considered?
    pub use_radii: bool,
}

/// Output for [`shape_distance`](crate::distance::shape_distance).
/// (b3DistanceOutput)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct DistanceOutput {
    /// Closest point on shape A, in shape A's frame
    pub point_a: Vec3,
    /// Closest point on shape B, in shape A's frame
    pub point_b: Vec3,
    /// A to B normal in shape A's frame. Invalid if distance is zero.
    pub normal: Vec3,
    /// The final distance, zero if overlapped
    pub distance: f32,
    /// Number of GJK iterations used
    pub iterations: i32,
    /// The number of simplexes stored in the simplex array
    pub simplex_count: i32,
}

/// Simplex vertex for debugging the GJK algorithm. (b3SimplexVertex)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SimplexVertex {
    /// support point in proxy A
    pub w_a: Vec3,
    /// support point in proxy B
    pub w_b: Vec3,
    /// w_b - w_a
    pub w: Vec3,
    /// barycentric coordinates
    pub a: f32,
    /// w_a index
    pub index_a: i32,
    /// w_b index
    pub index_b: i32,
}

/// Simplex from the GJK algorithm. (b3Simplex)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Simplex {
    /// vertices
    pub vertices: [SimplexVertex; 4],
    /// number of valid vertices
    pub count: i32,
}

/// Low level ray cast or shape-cast output data. (b3CastOutput)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CastOutput {
    /// The surface normal at the hit point.
    pub normal: Vec3,
    /// The surface hit point.
    pub point: Vec3,
    /// The fraction of the input translation at collision.
    pub fraction: f32,
    /// The number of iterations used.
    pub iterations: i32,
    /// The index of the mesh or height field triangle hit.
    pub triangle_index: i32,
    /// The index of the compound child shape.
    pub child_index: i32,
    /// The material index. May be -1 for null.
    pub material_index: i32,
    /// Did the cast hit?
    pub hit: bool,
}

impl Default for CastOutput {
    fn default() -> Self {
        CastOutput {
            normal: VEC3_ZERO,
            point: VEC3_ZERO,
            fraction: 0.0,
            iterations: 0,
            triangle_index: NULL_INDEX,
            child_index: 0,
            material_index: 0,
            hit: false,
        }
    }
}

/// This describes the motion of a body/shape for TOI computation. Shapes are
/// defined with respect to the body origin, which may not coincide with the
/// center of mass. However, to support dynamics we must interpolate the center
/// of mass position. (b3Sweep)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sweep {
    /// Local center of mass position
    pub local_center: Vec3,
    /// Starting center of mass world position
    pub c1: Vec3,
    /// Ending center of mass world position
    pub c2: Vec3,
    /// Starting world rotation
    pub q1: Quat,
    /// Ending world rotation
    pub q2: Quat,
}

impl Default for Sweep {
    fn default() -> Self {
        Sweep {
            local_center: VEC3_ZERO,
            c1: VEC3_ZERO,
            c2: VEC3_ZERO,
            q1: QUAT_IDENTITY,
            q2: QUAT_IDENTITY,
        }
    }
}

/// Time of impact input. (b3TOIInput)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToiInput {
    /// The proxy for shape A
    pub proxy_a: ShapeProxy,
    /// The proxy for shape B
    pub proxy_b: ShapeProxy,
    /// The movement of shape A
    pub sweep_a: Sweep,
    /// The movement of shape B
    pub sweep_b: Sweep,
    /// Defines the sweep interval [0, max_fraction]
    pub max_fraction: f32,
}

/// Describes the TOI output. (b3TOIState)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ToiState {
    #[default]
    Unknown,
    Failed,
    Overlapped,
    Hit,
    Separated,
}

/// Time of impact output. (b3TOIOutput)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ToiOutput {
    /// The type of result
    pub state: ToiState,
    /// The hit point
    pub point: Vec3,
    /// The hit normal
    pub normal: Vec3,
    /// The sweep time of the collision
    pub fraction: f32,
    /// The final distance
    pub distance: f32,
    /// Number of outer iterations
    pub distance_iterations: i32,
    /// Total number of push back iterations
    pub push_back_iterations: i32,
    /// Total number of root iterations
    pub root_iterations: i32,
    /// Indicates that the time of impact detected initial overlap and used a
    /// fallback sphere as a last ditch effort to prevent tunneling.
    /// (Present in the C type; never set by `b3TimeOfImpact` in the pinned source.)
    pub used_fallback: bool,
}
