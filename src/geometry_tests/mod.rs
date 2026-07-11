//! Port of the geometry half of box3d-cpp-reference/test/test_shape.c.
//!
//! Ported: ShapeMassTest, ShapeAABBTest, RayCastShapeTest, RayCastSphere*,
//! RayCastCapsule*, RayCastOverlapConventionTest, RayCastFarOriginTest.
//!
//! Skipped (need world/shape.c): ShapeNameTest, ShapeFlagsTest.
//!
//! SPDX-FileCopyrightText: 2023 Erin Catto
//! SPDX-License-Identifier: MIT

mod mass_aabb;
mod ray_capsule;
mod ray_misc;
mod ray_sphere;

use crate::geometry::CastOutput;
use crate::hull::{make_box_hull, BoxHull};
use crate::math_functions::{distance, mul_add, Vec3};

fn ensure_small(value: f32, tolerance: f32) {
    assert!(
        !(value < -tolerance || tolerance < value),
        "|{value}| > tolerance {tolerance}"
    );
}

fn v(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

fn capsule() -> crate::geometry::Capsule {
    crate::geometry::Capsule {
        center1: v(-1.0, 0.0, 0.0),
        center2: v(1.0, 0.0, 0.0),
        radius: 1.0,
    }
}

fn sphere() -> crate::geometry::Sphere {
    crate::geometry::Sphere {
        center: v(1.0, 0.0, 0.0),
        radius: 1.0,
    }
}

fn box_hull() -> BoxHull {
    make_box_hull(1.0, 1.0, 1.0)
}

/// Capsule along x from -2 to 2, radius 1. Reused by the capsule ray cast subtests.
fn ray_capsule() -> crate::geometry::Capsule {
    crate::geometry::Capsule {
        center1: v(-2.0, 0.0, 0.0),
        center2: v(2.0, 0.0, 0.0),
        radius: 1.0,
    }
}

/// Shared assertions for a surface hit.
fn check_cast_hit(
    out: CastOutput,
    origin: Vec3,
    translation: Vec3,
    point: Vec3,
    normal: Vec3,
    fraction: f32,
    tol: f32,
) {
    assert!(out.hit);
    ensure_small(out.fraction - fraction, tol);
    ensure_small(out.point.x - point.x, tol);
    ensure_small(out.point.y - point.y, tol);
    ensure_small(out.point.z - point.z, tol);
    ensure_small(out.normal.x - normal.x, tol);
    ensure_small(out.normal.y - normal.y, tol);
    ensure_small(out.normal.z - normal.z, tol);

    let on_ray = mul_add(origin, out.fraction, translation);
    ensure_small(distance(out.point, on_ray), tol);
}

/// The shared initial overlap convention: a ray starting inside a solid reports the origin
/// with zero fraction and no normal.
fn check_initial_overlap(out: CastOutput, origin: Vec3) {
    assert!(out.hit);
    assert_eq!(out.fraction, 0.0);
    ensure_small(distance(out.point, origin), f32::EPSILON);
    assert_eq!(out.normal.x, 0.0);
    assert_eq!(out.normal.y, 0.0);
    assert_eq!(out.normal.z, 0.0);
}

