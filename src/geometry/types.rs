//! Geometry shape types from include/box3d/types.h.
//!
//! `CastOutput` lives in [`crate::distance`] and is re-exported from the
//! geometry module.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::distance::ShapeProxy;
use crate::math_functions::{Matrix3, Plane, Vec3, MAT3_ZERO, VEC3_ZERO};

/// This holds the mass data computed for a shape. (b3MassData)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MassData {
    /// The shape mass
    pub mass: f32,
    /// The local center of mass position.
    pub center: Vec3,
    /// The inertia tensor about the shape center of mass.
    pub inertia: Matrix3,
}

impl Default for MassData {
    fn default() -> Self {
        MassData {
            mass: 0.0,
            center: VEC3_ZERO,
            inertia: MAT3_ZERO,
        }
    }
}

/// A solid sphere. (b3Sphere)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Sphere {
    /// The local center
    pub center: Vec3,
    /// The radius
    pub radius: f32,
}

/// A solid capsule can be viewed as two hemispheres connected by a cylinder.
/// (b3Capsule)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Capsule {
    /// Local center of the first hemisphere
    pub center1: Vec3,
    /// Local center of the second hemisphere
    pub center2: Vec3,
    /// The radius of the hemispheres
    pub radius: f32,
}

/// Low level ray cast input data. (b3RayCastInput)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RayCastInput {
    /// Start point of the ray cast.
    pub origin: Vec3,
    /// Translation of the ray cast. `end = start + translation`.
    pub translation: Vec3,
    /// The maximum fraction of the translation to consider, typically 1
    pub max_fraction: f32,
}

/// Low level shape cast input in generic form. This allows casting an arbitrary
/// point cloud wrapped with a radius. For example, a sphere is a single point
/// with a non-zero radius. A capsule is two points with a non-zero radius. A box
/// is eight points with a zero radius. (b3ShapeCastInput)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ShapeCastInput {
    /// A generic query shape.
    pub proxy: ShapeProxy,
    /// The translation of the shape cast.
    pub translation: Vec3,
    /// The maximum fraction of the translation to consider, typically 1.
    pub max_fraction: f32,
    /// Allow shape cast to encroach when initially touching. This only works if
    /// the radius is greater than zero.
    pub can_encroach: bool,
}

/// The plane between a character mover and a shape. (b3PlaneResult)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlaneResult {
    /// Outward pointing plane.
    pub plane: Plane,
    /// Closest point on the shape. May not be unique.
    pub point: Vec3,
}

impl Default for PlaneResult {
    fn default() -> Self {
        PlaneResult {
            plane: Plane {
                normal: VEC3_ZERO,
                offset: 0.0,
            },
            point: VEC3_ZERO,
        }
    }
}

/// Collision planes that can be fed to [`crate::mover::solve_planes`].
/// Normally assembled by the user from [`PlaneResult`] values. (b3CollisionPlane)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CollisionPlane {
    /// The collision plane between the mover and some shape.
    pub plane: Plane,
    /// Setting this to `f32::MAX` makes the plane as rigid as possible. Lower
    /// values can make the plane collision soft. Usually in meters.
    pub push_limit: f32,
    /// The push on the mover determined by `solve_planes`. Usually in meters.
    pub push: f32,
    /// Indicates if `clip_vector` should clip against this plane. Should be
    /// false for soft collision.
    pub clip_velocity: bool,
}

impl Default for CollisionPlane {
    fn default() -> Self {
        CollisionPlane {
            plane: Plane {
                normal: VEC3_ZERO,
                offset: 0.0,
            },
            push_limit: 0.0,
            push: 0.0,
            clip_velocity: false,
        }
    }
}

/// Result returned by [`crate::mover::solve_planes`]. (b3PlaneSolverResult)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PlaneSolverResult {
    /// The final relative translation.
    pub delta: Vec3,
    /// The number of iterations used by the plane solver. For diagnostics.
    pub iteration_count: i32,
}

/// Material properties supported per triangle on meshes and height fields.
/// (b3SurfaceMaterial)
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct SurfaceMaterial {
    /// The Coulomb (dry) friction coefficient, usually in the range [0,1].
    pub friction: f32,
    /// The coefficient of restitution (bounce) usually in the range [0,1].
    pub restitution: f32,
    /// The rolling resistance usually in the range [0,1].
    pub rolling_resistance: f32,
    /// The tangent velocity for conveyor belts.
    pub tangent_velocity: Vec3,
    /// User material identifier.
    pub user_material_id: u64,
    /// Custom debug draw color. Ignored if 0.
    pub custom_color: u32,
}

impl Default for SurfaceMaterial {
    fn default() -> Self {
        default_surface_material()
    }
}

/// Use this to initialize your surface material. (b3DefaultSurfaceMaterial)
pub fn default_surface_material() -> SurfaceMaterial {
    SurfaceMaterial {
        friction: 0.6,
        restitution: 0.0,
        rolling_resistance: 0.0,
        tangent_velocity: VEC3_ZERO,
        user_material_id: 0,
        custom_color: 0,
    }
}

/// Size of C `b3SurfaceMaterial` including trailing padding to 8-byte alignment.
pub const SURFACE_MATERIAL_SIZE: usize = 40;

impl SurfaceMaterial {
    /// Serialize to the C layout (40 bytes with padding).
    pub fn to_bytes(self) -> [u8; SURFACE_MATERIAL_SIZE] {
        let mut buf = [0u8; SURFACE_MATERIAL_SIZE];
        buf[0..4].copy_from_slice(&self.friction.to_le_bytes());
        buf[4..8].copy_from_slice(&self.restitution.to_le_bytes());
        buf[8..12].copy_from_slice(&self.rolling_resistance.to_le_bytes());
        buf[12..16].copy_from_slice(&self.tangent_velocity.x.to_le_bytes());
        buf[16..20].copy_from_slice(&self.tangent_velocity.y.to_le_bytes());
        buf[20..24].copy_from_slice(&self.tangent_velocity.z.to_le_bytes());
        buf[24..32].copy_from_slice(&self.user_material_id.to_le_bytes());
        buf[32..36].copy_from_slice(&self.custom_color.to_le_bytes());
        buf
    }

    /// Parse from the C layout.
    pub fn from_bytes(buf: &[u8]) -> Self {
        debug_assert!(buf.len() >= SURFACE_MATERIAL_SIZE);
        let read_f32 = |o: usize| f32::from_le_bytes(buf[o..o + 4].try_into().unwrap());
        Self {
            friction: read_f32(0),
            restitution: read_f32(4),
            rolling_resistance: read_f32(8),
            tangent_velocity: Vec3 {
                x: read_f32(12),
                y: read_f32(16),
                z: read_f32(20),
            },
            user_material_id: u64::from_le_bytes(buf[24..32].try_into().unwrap()),
            custom_color: u32::from_le_bytes(buf[32..36].try_into().unwrap()),
        }
    }
}

/// Shape type. (b3ShapeType)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(i32)]
pub enum ShapeType {
    /// A capsule is an extruded sphere.
    #[default]
    Capsule = 0,
    /// A baked compound shape.
    Compound = 1,
    /// A height field useful for terrain.
    Height = 2,
    /// A convex hull.
    Hull = 3,
    /// A triangle soup.
    Mesh = 4,
    /// A sphere with an offset.
    Sphere = 5,
}

/// Minimum and maximum extent of a shape relative to a local origin.
/// (math_internal.h: b3ShapeExtent)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ShapeExtent {
    pub min_extent: f32,
    pub max_extent: Vec3,
}
