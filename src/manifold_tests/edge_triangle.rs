//! Triangle / capsule / sphere edge-edge tests ported from `test/test_manifold.c`
//! (upstream commit aaa795e "Edge edge optimization").
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{
    check_edge_contact, ensure_small, exact_quat, exact_rotation, hull_edge_segment, min_separation,
    v, AXIS_X, AXIS_Y, HALF_DIAGONAL, HALF_ROOT2, TILT_ANGLES, TILT_AXES,
};
use crate::constants::speculative_distance;
use crate::distance::SimplexCache;
use crate::geometry::{Capsule, Sphere};
use crate::hull::{make_box_hull, make_transformed_box_hull};
use crate::manifold::{
    collide_capsule_and_triangle, collide_hull_and_capsule, collide_hull_and_sphere,
    collide_hull_and_triangle, LocalManifold, SatCache, SeparatingFeature, TriangleFeature,
};
use crate::math_functions::{
    add, cross, dot, lerp, mul_add, mul_sv, neg, normalize, rotate_vector, sub, PI,
    TRANSFORM_IDENTITY,
};

fn edge_pair(cache: &SatCache) -> bool {
    cache.type_ == SeparatingFeature::EdgePairAxis as u8
}

// A cube pitched 45 degrees rests on an edge along x at y = -h*root2. The two faces meeting there
// have normals (0,-r,r) and (0,-r,-r), so the arc between them spans the whole lower quadrant.
// A triangle edge crossing under it at an angle picks out an interior point of that arc, which is
// where a wrong lerp parameter would show up.
#[test]
fn triangle_edge_test() {
    let beta = 20.0 * PI / 180.0;
    let gamma = 30.0 * PI / 180.0;

    let hull = make_transformed_box_hull(0.5, 0.5, 0.5, exact_rotation(AXIS_X, 0.25 * PI));

    // Tipping the triangle plane about z keeps its normal off the hull edge, which the Minkowski
    // test needs. Tipping the edge within that plane moves the arc intersection off the midpoint.
    let tri_normal = v(beta.sin(), beta.cos(), 0.0);
    let tri_edge = v(
        gamma.sin() * beta.cos(),
        -gamma.sin() * beta.sin(),
        gamma.cos(),
    );

    // Perpendicular to both edges and pointing out of the hull
    let axis = normalize(cross(AXIS_X, tri_edge));
    let hull_point = v(0.0, -HALF_ROOT2, 0.0);

    let gaps = [0.03f32, 0.01, 0.0, -0.01, -0.1];

    for &gap in &gaps {
        let triangle_point = mul_add(hull_point, gap, axis);

        let v1 = mul_add(triangle_point, -1.0, tri_edge);
        let v2 = mul_add(triangle_point, 1.0, tri_edge);
        let v3 = mul_add(v1, 1.5, cross(tri_normal, tri_edge));

        let mut manifold = LocalManifold::default();
        let mut cache = SatCache {
            type_: SeparatingFeature::ManualEdgePairAxis as u8,
            ..Default::default()
        };
        collide_hull_and_triangle(&mut manifold, 8, &hull.base, v1, v2, v3, 0, &mut cache, true);

        let expected_normal = neg(axis);
        let expected_point = mul_add(hull_point, 0.5 * gap, axis);

        assert_eq!(manifold.point_count, 1);
        assert!(edge_pair(&cache));
        ensure_small(manifold.normal.x - expected_normal.x, 1e-5);
        ensure_small(manifold.normal.y - expected_normal.y, 1e-5);
        ensure_small(manifold.normal.z - expected_normal.z, 1e-5);
        ensure_small(manifold.points[0].separation - gap, 1e-5);
        ensure_small(manifold.points[0].point.x - expected_point.x, 1e-5);
        ensure_small(manifold.points[0].point.y - expected_point.y, 1e-5);
        ensure_small(manifold.points[0].point.z - expected_point.z, 1e-5);
    }

    // The tipped triangle plane buries a corner of the hull, so neither face axis separates and
    // the edge axis has to carry the speculative cull on its own.
    let culled = [0.03f32, 0.05];

    for &gap in &culled {
        let triangle_point = mul_add(hull_point, gap, axis);

        let v1 = mul_add(triangle_point, -1.0, tri_edge);
        let v2 = mul_add(triangle_point, 1.0, tri_edge);
        let v3 = mul_add(v1, 1.5, cross(tri_normal, tri_edge));

        let mut manifold = LocalManifold::default();
        let mut cache = SatCache::default();
        collide_hull_and_triangle(&mut manifold, 8, &hull.base, v1, v2, v3, 0, &mut cache, true);

        assert_eq!(manifold.point_count, 0);
        assert!(edge_pair(&cache));
        ensure_small(cache.separation - gap, 1e-5);
    }
}

// The same cube resting its bottom edge on a triangle whose first edge runs along x. Tipping the
// triangle takes that pair from exactly parallel through the rejection threshold.
#[test]
fn triangle_parallel_edge_test() {
    let hull = make_transformed_box_hull(0.5, 0.5, 0.5, exact_rotation(AXIS_X, 0.25 * PI));

    let overlap = 0.01;
    let y = -HALF_ROOT2 + overlap;

    for &tilt_axis in &TILT_AXES {
        for &angle in &TILT_ANGLES {
            let q = exact_quat(tilt_axis, angle);

            let v1 = rotate_vector(q, v(-2.0, y, -1.0));
            let v2 = rotate_vector(q, v(0.0, y, 2.0));
            let v3 = rotate_vector(q, v(2.0, y, -1.0));

            let mut manifold = LocalManifold::default();
            let mut cache = SatCache::default();
            collide_hull_and_triangle(&mut manifold, 8, &hull.base, v1, v2, v3, 0, &mut cache, true);

            assert_eq!(manifold.point_count, 4);
            assert_eq!(cache.type_, SeparatingFeature::FaceAxisA as u8);
            assert!(dot(manifold.normal, AXIS_Y) > 0.99);

            // The tilt can only sink the contact by the length of the arc it sweeps
            let bound = HALF_DIAGONAL * angle + 1e-5;
            ensure_small(min_separation(&manifold) + overlap, bound);
        }
    }
}

// A thin capsule stabbed through the +x +y edge of a box so the edge pair is the axis of minimum
// penetration. This drives the isolated edge axis (arc versus circle on the Gauss map) that a
// capsule presents.
#[test]
fn hull_capsule_edge_deep_test() {
    let hull = make_box_hull(0.5, 0.5, 0.5);

    // The +x +y edge runs along z between the +x and +y faces
    let edge_point = v(0.5, 0.5, 0.0);
    let edge_dir = v(0.0, 0.0, 1.0);
    let outward = normalize(v(1.0, 1.0, 0.0));

    // Penetrate far enough that the core segment clearly overlaps the box so the deep path runs,
    // but keep the radius small enough that the edge stays the axis of minimum penetration.
    let depths = [0.12f32, 0.18, 0.25];
    let radii = [0.05f32, 0.1, 0.2];
    let tilts = [0.0f32, 0.25, -0.25];

    let mut count = 0;

    for &depth in &depths {
        for &radius in &radii {
            for &tilt in &tilts {
                let capsule_dir = normalize(v(1.0, -1.0, tilt));
                let mid = mul_add(edge_point, -depth, outward);
                let c1 = mul_add(mid, -0.5, capsule_dir);
                let c2 = mul_add(mid, 0.5, capsule_dir);
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

                assert_eq!(manifold.point_count, 1);
                assert!(manifold.points[0].separation < 0.0);

                // Hull edge is e1, capsule axis is e2, normal points out of the hull
                check_edge_contact(
                    &manifold,
                    edge_point,
                    edge_dir,
                    c1,
                    sub(c2, c1),
                    outward,
                    radius,
                    1e-4,
                    1e-4,
                    1e-4,
                );

                count += 1;
            }
        }
    }

    assert_eq!(count, depths.len() * radii.len() * tilts.len());
}

// Force the triangle versus hull edge query over a broad sweep of crossing geometries. The manual
// axis hands the winning pair to the builder, and the recovered axis must match the cross product
// of the chosen triangle and hull edges and point from the triangle into the hull.
#[test]
fn triangle_hull_edge_sweep_test() {
    let hull = make_transformed_box_hull(0.5, 0.5, 0.5, exact_rotation(AXIS_X, 0.25 * PI));
    let hull_edge_point = v(0.0, -HALF_ROOT2, 0.0);

    // Degrees: triangle plane tip about z, and triangle edge yaw
    let betas = [8.0f32, 20.0, 32.0];
    let gammas = [20.0f32, 35.0, 50.0, 70.0];
    let gaps = [0.02f32, 0.0, -0.03, -0.08];

    let mut edge_contacts = 0;

    for &beta_deg in &betas {
        for &gamma_deg in &gammas {
            for &gap in &gaps {
                let beta = beta_deg * PI / 180.0;
                let gamma = gamma_deg * PI / 180.0;

                // Tip the plane off the hull edge so the Minkowski test holds, then yaw the edge
                let tri_normal = v(beta.sin(), beta.cos(), 0.0);
                let tri_edge = v(
                    gamma.sin() * beta.cos(),
                    -gamma.sin() * beta.sin(),
                    gamma.cos(),
                );

                // Perpendicular to both edges and pointing out of the hull
                let axis = normalize(cross(AXIS_X, tri_edge));
                let triangle_point = mul_add(hull_edge_point, gap, axis);

                let v1 = mul_add(triangle_point, -1.0, tri_edge);
                let v2 = mul_add(triangle_point, 1.0, tri_edge);
                let v3 = mul_add(v1, 1.5, cross(tri_normal, tri_edge));

                let triangle_verts = [v1, v2, v3];
                let triangle_edges = [sub(v2, v1), sub(v3, v2), sub(v1, v3)];
                let triangle_center = mul_sv(1.0 / 3.0, add(v1, add(v2, v3)));

                let mut manifold = LocalManifold::default();
                let mut cache = SatCache {
                    type_: SeparatingFeature::ManualEdgePairAxis as u8,
                    ..Default::default()
                };
                collide_hull_and_triangle(
                    &mut manifold,
                    8,
                    &hull.base,
                    v1,
                    v2,
                    v3,
                    0,
                    &mut cache,
                    true,
                );

                if !edge_pair(&cache) || manifold.point_count != 1 {
                    continue;
                }

                let p1 = triangle_verts[cache.index_a as usize];
                let e1 = triangle_edges[cache.index_a as usize];

                let (p2, e2) = hull_edge_segment(&hull.base, cache.index_b as i32, TRANSFORM_IDENTITY);

                // Normal points from the triangle into the hull
                let orient_ref = sub(hull.base.center, triangle_center);

                check_edge_contact(&manifold, p1, e1, p2, e2, orient_ref, 0.0, 1e-4, 1e-4, 1e-3);

                edge_contacts += 1;
            }
        }
    }

    assert!(edge_contacts >= 30);
}

// A capsule laid nearly in a triangle plane and pushed across one edge so the edge pair drives the
// deep contact. This exercises the two sided triangle edge, where the side normal trick chooses
// which half of the arc holds the axis.
#[test]
fn capsule_triangle_edge_deep_test() {
    // Triangle in the y = 0 plane. The v1 v2 edge runs along x at z = 0, the interior lies at z < 0.
    let v1 = v(-2.0, 0.0, 0.0);
    let v2 = v(2.0, 0.0, 0.0);
    let v3 = v(0.0, 0.0, -2.0);
    let triangle = [v1, v2, v3];
    let triangle_edges = [sub(v2, v1), sub(v3, v2), sub(v1, v3)];
    let triangle_center = mul_sv(1.0 / 3.0, add(v1, add(v2, v3)));

    // A nearly in plane core crossing the edge at (0,0,z0) with a small out of plane tilt.
    let z0s = [-0.05f32, -0.03, -0.01];
    let tilts = [0.2f32, 0.3, 0.4];
    let yaws = [0.4f32, 0.6, 0.8];
    let radii = [0.05f32, 0.1];

    let mut edge_contacts = 0;

    for &z0 in &z0s {
        for &tilt in &tilts {
            for &yaw in &yaws {
                for &radius in &radii {
                    let capsule_dir = normalize(v(yaw.sin(), tilt, yaw.cos()));
                    let mid = v(0.0, 0.0, z0);
                    let c1 = mul_add(mid, -0.6, capsule_dir);
                    let c2 = mul_add(mid, 0.6, capsule_dir);
                    let capsule = Capsule {
                        center1: c1,
                        center2: c2,
                        radius,
                    };

                    let mut manifold = LocalManifold::default();
                    let mut cache = SimplexCache::default();
                    collide_capsule_and_triangle(&mut manifold, 8, &capsule, &triangle, &mut cache);

                    // Only the edge contacts exercise the new axis. Face contacts are handled elsewhere.
                    let feature = manifold.feature as u8;
                    if manifold.point_count != 1
                        || feature < TriangleFeature::Edge1 as u8
                        || feature > TriangleFeature::Edge3 as u8
                    {
                        continue;
                    }

                    let edge_index = (feature - TriangleFeature::Edge1 as u8) as usize;
                    let p1 = triangle[edge_index];
                    let e1 = triangle_edges[edge_index];

                    // Normal points from the triangle toward the capsule
                    let capsule_edge = sub(c2, c1);
                    let capsule_center = lerp(c1, c2, 0.5);
                    let orient_ref = sub(capsule_center, triangle_center);

                    check_edge_contact(
                        &manifold,
                        p1,
                        e1,
                        c1,
                        capsule_edge,
                        orient_ref,
                        radius,
                        1e-4,
                        1e-4,
                        2e-4,
                    );

                    edge_contacts += 1;
                }
            }
        }
    }

    // The sweep must actually reach the edge path
    assert!(edge_contacts >= 15);
}

// A sphere driven straight through a box face, from separated, across the surface where the
// collider switches from GJK closest points to the SAT face pick, and on into deep overlap.
#[test]
fn sphere_hull_seam_test() {
    let hull = make_box_hull(0.5, 0.5, 0.5);
    let radius = 0.15;

    let y_start = 0.5 + radius + 0.4 * speculative_distance();
    let y_end = 0.1;
    let steps = 400;
    let dy = (y_start - y_end) / steps as f32;

    let mut previous = 0.0;
    let mut shallow_samples = 0;
    let mut deep_samples = 0;

    for i in 0..=steps {
        let y = y_start - i as f32 * dy;
        let sphere = Sphere {
            center: v(0.0, y, 0.0),
            radius,
        };

        let mut manifold = LocalManifold::default();
        let mut cache = SimplexCache::default();
        collide_hull_and_sphere(
            &mut manifold,
            8,
            &hull.base,
            &sphere,
            TRANSFORM_IDENTITY,
            &mut cache,
        );

        assert_eq!(manifold.point_count, 1);

        let separation = manifold.points[0].separation;
        let expected = (y - 0.5) - radius;

        // Separation is the analytic gap on both sides of the seam
        ensure_small(separation - expected, 1e-5);

        // Normal holds the face direction with no flip
        ensure_small(manifold.normal.x, 1e-5);
        ensure_small(manifold.normal.y - 1.0, 1e-5);
        ensure_small(manifold.normal.z, 1e-5);

        // No jump across the seam: consecutive separations track the step
        if i > 0 {
            ensure_small((previous - separation) - dy, 1e-5);
        }
        previous = separation;

        if y > 0.5 {
            shallow_samples += 1;
        } else {
            deep_samples += 1;
        }
    }

    // The sweep must straddle the surface so both the GJK and the SAT branch run
    assert!(shallow_samples > 0 && deep_samples > 0);
}

// The same seam for a capsule laid parallel to the face.
#[test]
fn capsule_hull_seam_test() {
    let hull = make_box_hull(0.5, 0.5, 0.5);
    let radius = 0.15;
    let half_length = 0.3;

    let y_start = 0.5 + radius + 0.4 * speculative_distance();
    let y_end = 0.1;
    let steps = 400;
    let dy = (y_start - y_end) / steps as f32;

    let mut previous = 0.0;
    let mut shallow_samples = 0;
    let mut deep_samples = 0;

    for i in 0..=steps {
        let y = y_start - i as f32 * dy;
        let capsule = Capsule {
            center1: v(-half_length, y, 0.0),
            center2: v(half_length, y, 0.0),
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

        assert!(manifold.point_count >= 1);

        let expected = (y - 0.5) - radius;

        // Every point sits at the analytic gap
        for k in 0..manifold.point_count {
            ensure_small(manifold.points[k as usize].separation - expected, 1e-5);
        }

        // Normal holds the face direction with no flip
        ensure_small(manifold.normal.x, 1e-5);
        ensure_small(manifold.normal.y - 1.0, 1e-5);
        ensure_small(manifold.normal.z, 1e-5);

        // No jump across the seam
        let min = min_separation(&manifold);
        if i > 0 {
            ensure_small((previous - min) - dy, 1e-5);
        }
        previous = min;

        if y > 0.5 {
            shallow_samples += 1;
        } else {
            deep_samples += 1;
        }
    }

    assert!(shallow_samples > 0 && deep_samples > 0);
}
