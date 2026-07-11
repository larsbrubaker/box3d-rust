//! Focused unit tests for the convex primitive manifold slice.
//!
//! Upstream C has almost no dedicated collide unit tests (samples + determinism
//! cover them). These mirror the invent-focused style of box2d-rust's
//! `manifold_tests.rs`: overlap, separation, normals, and point counts.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::constants::MAX_MANIFOLD_POINTS;
use crate::distance::SimplexCache;
use crate::geometry::{Capsule, Sphere};
use crate::hull::make_box_hull;
use crate::manifold::{
    collide_capsule_and_sphere, collide_capsules, collide_hull_and_sphere, collide_spheres,
    LocalManifold,
};
use crate::math_functions::{
    Transform, Vec3, QUAT_IDENTITY, TRANSFORM_IDENTITY, VEC3_ZERO,
};

fn ensure_small(value: f32, tolerance: f32) {
    // Matches the C ENSURE_SMALL macro, which is inclusive: pass when
    // -tol <= value <= tol.
    assert!(
        !(value < -tolerance || tolerance < value),
        "|{value}| > tolerance {tolerance}"
    );
}

fn v(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

fn xf_at(p: Vec3) -> Transform {
    Transform {
        p,
        q: QUAT_IDENTITY,
    }
}

#[test]
fn collide_spheres_overlap_and_separation() {
    let a = Sphere {
        center: VEC3_ZERO,
        radius: 1.0,
    };
    let b = Sphere {
        center: VEC3_ZERO,
        radius: 1.0,
    };

    // Centers 1.5 apart → overlap of 0.5 along +x.
    let mut manifold = LocalManifold::default();
    collide_spheres(
        &mut manifold,
        MAX_MANIFOLD_POINTS as i32,
        &a,
        &b,
        xf_at(v(1.5, 0.0, 0.0)),
    );
    assert_eq!(manifold.point_count, 1);
    ensure_small(manifold.normal.x - 1.0, f32::EPSILON);
    ensure_small(manifold.normal.y, f32::EPSILON);
    ensure_small(manifold.normal.z, f32::EPSILON);
    ensure_small(manifold.points[0].separation + 0.5, 1e-6);
    // Midpoint: 0.5 * ((0 + 1*n) + 1.5 - 1*n) = 0.75 on x
    ensure_small(manifold.points[0].point.x - 0.75, 1e-6);

    // Far apart: no contact (point_count left at 0 from default).
    let mut far = LocalManifold::default();
    collide_spheres(
        &mut far,
        MAX_MANIFOLD_POINTS as i32,
        &a,
        &b,
        xf_at(v(10.0, 0.0, 0.0)),
    );
    assert_eq!(far.point_count, 0);

    // Coincident centers: fallback normal (0,1,0), separation = -2.
    let mut coincident = LocalManifold::default();
    collide_spheres(
        &mut coincident,
        MAX_MANIFOLD_POINTS as i32,
        &a,
        &b,
        TRANSFORM_IDENTITY,
    );
    assert_eq!(coincident.point_count, 1);
    ensure_small(coincident.normal.x, f32::EPSILON);
    ensure_small(coincident.normal.y - 1.0, f32::EPSILON);
    ensure_small(coincident.normal.z, f32::EPSILON);
    ensure_small(coincident.points[0].separation + 2.0, 1e-6);
}

#[test]
fn collide_capsule_and_sphere_overlap_and_separation() {
    let capsule = Capsule {
        center1: v(-1.0, 0.0, 0.0),
        center2: v(1.0, 0.0, 0.0),
        radius: 0.5,
    };
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };

    // Sphere above capsule mid-segment, centers 0.8 apart → overlap 0.2.
    let mut manifold = LocalManifold::default();
    collide_capsule_and_sphere(
        &mut manifold,
        MAX_MANIFOLD_POINTS as i32,
        &capsule,
        &sphere,
        xf_at(v(0.0, 0.8, 0.0)),
    );
    assert_eq!(manifold.point_count, 1);
    ensure_small(manifold.normal.x, 1e-6);
    ensure_small(manifold.normal.y - 1.0, 1e-6);
    ensure_small(manifold.normal.z, 1e-6);
    ensure_small(manifold.points[0].separation + 0.2, 1e-5);

    // Far above: separated.
    let mut far = LocalManifold::default();
    collide_capsule_and_sphere(
        &mut far,
        MAX_MANIFOLD_POINTS as i32,
        &capsule,
        &sphere,
        xf_at(v(0.0, 5.0, 0.0)),
    );
    assert_eq!(far.point_count, 0);

    // Sphere beyond an end-cap (along +x past center2).
    let mut end = LocalManifold::default();
    collide_capsule_and_sphere(
        &mut end,
        MAX_MANIFOLD_POINTS as i32,
        &capsule,
        &sphere,
        xf_at(v(1.8, 0.0, 0.0)),
    );
    assert_eq!(end.point_count, 1);
    ensure_small(end.normal.x - 1.0, 1e-5);
    ensure_small(end.points[0].separation + 0.2, 1e-5);
}

#[test]
fn collide_capsules_parallel_and_skew() {
    // Two horizontal capsules stacked with overlap → two contact points.
    let a = Capsule {
        center1: v(-1.0, 0.0, 0.0),
        center2: v(1.0, 0.0, 0.0),
        radius: 0.5,
    };
    let b = Capsule {
        center1: v(-1.0, 0.0, 0.0),
        center2: v(1.0, 0.0, 0.0),
        radius: 0.5,
    };

    let mut manifold = LocalManifold::default();
    collide_capsules(
        &mut manifold,
        MAX_MANIFOLD_POINTS as i32,
        &a,
        &b,
        xf_at(v(0.0, 0.9, 0.0)),
    );
    assert_eq!(manifold.point_count, 2);
    ensure_small(manifold.normal.x, 1e-5);
    ensure_small(manifold.normal.y - 1.0, 1e-5);
    ensure_small(manifold.normal.z, 1e-5);
    ensure_small(manifold.points[0].separation + 0.1, 1e-4);
    ensure_small(manifold.points[1].separation + 0.1, 1e-4);

    // Far apart.
    let mut far = LocalManifold::default();
    collide_capsules(
        &mut far,
        MAX_MANIFOLD_POINTS as i32,
        &a,
        &b,
        xf_at(v(0.0, 10.0, 0.0)),
    );
    assert_eq!(far.point_count, 0);

    // Skew (non-parallel) capsules: single closest-point contact.
    let skew_b = Capsule {
        center1: v(0.0, 0.0, -1.0),
        center2: v(0.0, 0.0, 1.0),
        radius: 0.5,
    };
    let mut skew = LocalManifold::default();
    collide_capsules(
        &mut skew,
        MAX_MANIFOLD_POINTS as i32,
        &a,
        &skew_b,
        xf_at(v(0.0, 0.8, 0.0)),
    );
    assert_eq!(skew.point_count, 1);
    ensure_small(skew.normal.y - 1.0, 1e-4);
    ensure_small(skew.points[0].separation + 0.2, 1e-4);
}

#[test]
fn collide_hull_and_sphere_via_gjk() {
    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };

    // Sphere resting above the +y face with 0.05 penetration.
    // Box half-extent 1, sphere radius 0.5, center at y=1.45 → separation -0.05.
    let mut manifold = LocalManifold::default();
    let mut cache = SimplexCache::default();
    collide_hull_and_sphere(
        &mut manifold,
        MAX_MANIFOLD_POINTS as i32,
        &box_hull.base,
        &sphere,
        xf_at(v(0.0, 1.45, 0.0)),
        &mut cache,
    );
    assert_eq!(manifold.point_count, 1);
    ensure_small(manifold.normal.x, 1e-5);
    ensure_small(manifold.normal.y - 1.0, 1e-5);
    ensure_small(manifold.normal.z, 1e-5);
    ensure_small(manifold.points[0].separation + 0.05, 1e-4);

    // Far above: separated, cache cleared.
    let mut far = LocalManifold::default();
    let mut far_cache = SimplexCache::default();
    // Seed cache so we can observe the clear-on-separation path.
    far_cache.count = 1;
    collide_hull_and_sphere(
        &mut far,
        MAX_MANIFOLD_POINTS as i32,
        &box_hull.base,
        &sphere,
        xf_at(v(0.0, 10.0, 0.0)),
        &mut far_cache,
    );
    assert_eq!(far.point_count, 0);
    assert_eq!(far_cache.count, 0);

    // Deep penetration: sphere center inside the box → face-support path.
    let mut deep = LocalManifold::default();
    let mut deep_cache = SimplexCache::default();
    collide_hull_and_sphere(
        &mut deep,
        MAX_MANIFOLD_POINTS as i32,
        &box_hull.base,
        &sphere,
        xf_at(v(0.0, 0.25, 0.0)),
        &mut deep_cache,
    );
    assert_eq!(deep.point_count, 1);
    // Closest face is +y: plane sep of center = -0.75, then - radius → -1.25.
    ensure_small(deep.normal.y - 1.0, 1e-5);
    ensure_small(deep.points[0].separation + 1.25, 1e-4);
}
