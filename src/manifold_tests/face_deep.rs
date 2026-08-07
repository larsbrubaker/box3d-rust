//! Deep face-clip tests for the capsule colliders, ported from `test/test_manifold.c`.
//!
//! Sibling of `edge_triangle`, which covers the deep edge-edge paths for the same
//! shape pairs.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{ensure_small, v};
use crate::distance::SimplexCache;
use crate::geometry::Capsule;
use crate::hull::make_box_hull;
use crate::manifold::{
    collide_hull_and_capsule, collide_triangle_and_capsule, LocalManifold, TriangleFeature,
};
use crate::math_functions::{max_float, min_float, mul_add, TRANSFORM_IDENTITY};

// A capsule core straddling a triangle face inside the interior. The core pierces the plane so the
// deep path runs, and with the tilt kept small the face stays the axis of minimum penetration, so the
// clip must return two points on the triangle face. This is the branch that had no coverage.
#[test]
fn capsule_triangle_face_deep_test() {
    // Triangle in the y = 0 plane, normal +y, centroid at the origin
    let v1 = v(-3.0, 0.0, -2.0);
    let v2 = v(0.0, 0.0, 4.0);
    let v3 = v(3.0, 0.0, -2.0);
    let triangle = [v1, v2, v3];

    let yaws = [0.0f32, 0.6, 1.2, 1.8, 2.4];
    let tilts = [0.06f32, 0.1, 0.15];
    let radii = [0.05f32, 0.1, 0.2];

    // Bias the center just above the plane so the back side cull passes while the lower endpoint dips through
    let bias = 0.01;
    let half_length = 1.0;

    let mut face_contacts = 0;

    for &yaw in &yaws {
        for &tilt in &tilts {
            for &radius in &radii {
                // Axis is the in-plane heading tipped up so the segment straddles the plane
                let axis = v(tilt.cos() * yaw.cos(), tilt.sin(), tilt.cos() * yaw.sin());
                let center = v(0.0, bias, 0.0);
                let c1 = mul_add(center, -half_length, axis);
                let c2 = mul_add(center, half_length, axis);
                let capsule = Capsule {
                    center1: c1,
                    center2: c2,
                    radius,
                };

                let mut manifold = LocalManifold::default();
                let mut cache = SimplexCache::default();
                collide_triangle_and_capsule(&mut manifold, 8, &triangle, &capsule, &mut cache);

                // Two points on the triangle face with the plane normal
                assert_eq!(manifold.point_count, 2);
                assert_eq!(manifold.feature, TriangleFeature::TriangleFace);
                ensure_small(manifold.normal.x, 1e-5);
                ensure_small(manifold.normal.y - 1.0, 1e-5);
                ensure_small(manifold.normal.z, 1e-5);

                // Separations are the endpoint heights pulled in by the radius. The lower endpoint is below
                // the plane, so the deepest separation is negative.
                let lower = min_float(c1.y, c2.y) - radius;
                let upper = max_float(c1.y, c2.y) - radius;
                let min_sep =
                    min_float(manifold.points[0].separation, manifold.points[1].separation);
                let max_sep =
                    max_float(manifold.points[0].separation, manifold.points[1].separation);
                ensure_small(min_sep - lower, 1e-5);
                ensure_small(max_sep - upper, 1e-5);
                assert!(min_sep < 0.0);

                face_contacts += 1;
            }
        }
    }

    // Every configuration must reach the face path
    assert_eq!(face_contacts, yaws.len() * tilts.len() * radii.len());
}

// A capsule laid flat with its core below a box top face. The core sits inside the box so the deep
// path runs, and across a sweep of depths, headings and radii the face clip must return two points.
#[test]
fn hull_capsule_face_deep_test() {
    let hull = make_box_hull(0.5, 0.5, 0.5);

    let depths = [0.1f32, 0.2, 0.3, 0.4, 0.45];
    let yaws = [0.0f32, 0.4, 0.8, 1.2];
    let radii = [0.1f32, 0.15, 0.2];
    let offsets = [-0.1f32, 0.0, 0.1];
    let half_length = 0.3;

    let mut face_contacts = 0;

    for &depth in &depths {
        for &yaw in &yaws {
            for &radius in &radii {
                for &offset in &offsets {
                    let y = depth;
                    let dir = v(yaw.cos(), 0.0, yaw.sin());
                    let center = v(offset, y, 0.0);
                    let c1 = mul_add(center, -half_length, dir);
                    let c2 = mul_add(center, half_length, dir);
                    let capsule = Capsule {
                        center1: c1,
                        center2: c2,
                        radius,
                    };

                    let mut manifold = LocalManifold::default();
                    let mut cache = SimplexCache::default();
                    collide_hull_and_capsule(
                        &mut manifold,
                        8,
                        &hull.base,
                        &capsule,
                        TRANSFORM_IDENTITY,
                        &mut cache,
                    );

                    // Two points on the top face. The hull path does not tag a feature, so the face is
                    // identified by the normal and the point count.
                    assert_eq!(manifold.point_count, 2);
                    ensure_small(manifold.normal.x, 1e-5);
                    ensure_small(manifold.normal.y - 1.0, 1e-5);
                    ensure_small(manifold.normal.z, 1e-5);

                    // Flat capsule, so both points sit at the same analytic gap
                    let expected = (y - 0.5) - radius;
                    ensure_small(manifold.points[0].separation - expected, 1e-5);
                    ensure_small(manifold.points[1].separation - expected, 1e-5);
                    assert!(expected < 0.0);

                    face_contacts += 1;
                }
            }
        }
    }

    assert_eq!(
        face_contacts,
        depths.len() * yaws.len() * radii.len() * offsets.len()
    );
}
