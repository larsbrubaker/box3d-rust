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
