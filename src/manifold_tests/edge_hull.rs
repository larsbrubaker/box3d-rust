//! Hull-vs-hull edge-edge tests ported from `test/test_manifold.c`
//! (upstream commit aaa795e "Edge edge optimization").
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{
    check_edge_contact, ensure_small, exact_quat, exact_rotation, hull_edge_segment,
    min_separation, normal_length, slide_x, v, Rng, AXIS_X, AXIS_Y, AXIS_Z, HALF_DIAGONAL,
    HALF_ROOT2, ROOT2, TILT_ANGLES, TILT_AXES,
};
use crate::hull::{make_box_hull, make_transformed_box_hull, BoxHull};
use crate::manifold::{collide_hulls, LocalManifold, SatCache, SeparatingFeature};
use crate::math_functions::{
    dot, mul_sv, transform_point, Transform, PI, QUAT_IDENTITY, TRANSFORM_IDENTITY,
};

fn edge_pair(cache: &SatCache) -> bool {
    cache.type_ == SeparatingFeature::EdgePairAxis as u8
}

// Cube A yawed 45 degrees presents an edge along y at x = +h*root2.
// Cube B rolled 45 degrees presents an edge along z at x = -h*root2.
// Sliding B along x makes those two edges the closest features, so the axis of minimum
// penetration is x, the separation is d - 2*h*root2 and the contact point sits at x = d/2.
// Both hulls are far from a face axis here, which keeps the edge query in charge.
// (MakeCrossedEdgeHulls)
fn make_crossed_edge_hulls(half_width: f32) -> (BoxHull, BoxHull) {
    let hull_a = make_transformed_box_hull(
        half_width,
        half_width,
        half_width,
        exact_rotation(AXIS_Y, 0.25 * PI),
    );
    let hull_b = make_transformed_box_hull(
        half_width,
        half_width,
        half_width,
        exact_rotation(AXIS_Z, 0.25 * PI),
    );
    (hull_a, hull_b)
}

// The edge pair axis is built by intersecting the two Gauss map arcs. Walk the crossed edges from
// speculative contact into deep overlap and check the axis, the separation and the point.
#[test]
fn crossed_edge_test() {
    let (hull_a, hull_b) = make_crossed_edge_hulls(0.5);

    let distances = [1.42f32, ROOT2, 1.41, 1.3];

    for &d in &distances {
        let expected = d - ROOT2;

        let mut manifold = LocalManifold::default();
        let mut cache = SatCache::default();
        collide_hulls(
            &mut manifold,
            8,
            &hull_a.base,
            &hull_b.base,
            slide_x(d),
            &mut cache,
        );

        assert_eq!(manifold.point_count, 1);
        assert!(edge_pair(&cache));
        ensure_small(manifold.normal.x - 1.0, 1e-6);
        ensure_small(manifold.normal.y, 1e-6);
        ensure_small(manifold.normal.z, 1e-6);
        ensure_small(manifold.points[0].separation - expected, 1e-5);
        ensure_small(manifold.points[0].point.x - 0.5 * d, 1e-5);
        ensure_small(manifold.points[0].point.y, 1e-5);
        ensure_small(manifold.points[0].point.z, 1e-5);

        // The forced edge query must agree with what the full solver chose
        let mut manual = LocalManifold::default();
        let mut manual_cache = SatCache {
            type_: SeparatingFeature::ManualEdgePairAxis as u8,
            ..Default::default()
        };
        collide_hulls(
            &mut manual,
            8,
            &hull_a.base,
            &hull_b.base,
            slide_x(d),
            &mut manual_cache,
        );

        assert_eq!(manual.point_count, 1);
        ensure_small(manual.points[0].separation - expected, 1e-5);
    }

    // Beyond the speculative distance the query reports the axis without building a contact.
    // The axis carries its own orientation now, so a sign error here would read as deep overlap.
    let mut manifold = LocalManifold::default();
    let mut cache = SatCache::default();
    collide_hulls(
        &mut manifold,
        8,
        &hull_a.base,
        &hull_b.base,
        slide_x(1.5),
        &mut cache,
    );

    assert_eq!(manifold.point_count, 0);
    assert!(edge_pair(&cache));
    ensure_small(cache.separation - (1.5 - ROOT2), 1e-5);
}

// The parallel edge rejection compares dot products against the edge length, so it is a sine
// threshold and must hold at any size.
#[test]
fn edge_axis_scale_test() {
    let scales = [100.0f32, 1.0, 0.2];

    for &s in &scales {
        let (hull_a, hull_b) = make_crossed_edge_hulls(0.5 * s);

        let expected = -0.002;
        let d = s * ROOT2 + expected;

        let mut manifold = LocalManifold::default();
        let mut cache = SatCache::default();
        collide_hulls(
            &mut manifold,
            8,
            &hull_a.base,
            &hull_b.base,
            slide_x(d),
            &mut cache,
        );

        // Differencing coordinates of magnitude d costs precision proportional to the scale
        let tolerance = 1e-5 * s + 1e-6;

        assert_eq!(manifold.point_count, 1);
        assert!(edge_pair(&cache));
        ensure_small(manifold.normal.x - 1.0, 1e-6);
        ensure_small(manifold.points[0].separation - expected, tolerance);
        ensure_small(manifold.points[0].point.x - 0.5 * d, tolerance);
        ensure_small(manifold.points[0].point.y, tolerance);
        ensure_small(manifold.points[0].point.z, tolerance);
    }
}

// The cached edge pair rebuilds the axis without a fresh query. An untouched cache proves the
// cached branch answered rather than falling through to the full SAT.
#[test]
fn edge_cache_test() {
    let (hull_a, hull_b) = make_crossed_edge_hulls(0.5);

    let mut manifold = LocalManifold::default();
    let mut cache = SatCache::default();

    collide_hulls(
        &mut manifold,
        8,
        &hull_a.base,
        &hull_b.base,
        slide_x(1.41),
        &mut cache,
    );
    assert_eq!(manifold.point_count, 1);
    assert!(edge_pair(&cache));
    // Cached edges are the even half of each twin pair
    assert!((cache.index_a & 1) == 0 && (cache.index_a as i32) < hull_a.base.edge_count);
    assert!((cache.index_b & 1) == 0 && (cache.index_b as i32) < hull_b.base.edge_count);

    let seeded_separation = cache.separation;
    ensure_small(seeded_separation - (1.41 - ROOT2), 1e-5);

    // Small motion, the cached features still describe the contact
    collide_hulls(
        &mut manifold,
        8,
        &hull_a.base,
        &hull_b.base,
        slide_x(1.4105),
        &mut cache,
    );
    assert_eq!(manifold.point_count, 1);
    assert_eq!(cache.separation, seeded_separation);
    ensure_small(manifold.points[0].separation - (1.4105 - ROOT2), 1e-5);
    ensure_small(manifold.normal.x - 1.0, 1e-6);

    // Jump past the speculative distance. The cached axis alone must report the separation.
    collide_hulls(
        &mut manifold,
        8,
        &hull_a.base,
        &hull_b.base,
        slide_x(1.5),
        &mut cache,
    );
    assert_eq!(manifold.point_count, 0);
    assert_eq!(cache.separation, seeded_separation);
}

// Sliding B along the direction of edge A walks the closest point off the end of the segment.
#[test]
fn edge_endpoint_test() {
    let (hull_a, hull_b) = make_crossed_edge_hulls(0.5);

    let d = 1.41;
    let expected = d - ROOT2;

    // Just inside the end of edge A
    {
        let transform = Transform {
            p: v(d, 0.49, 0.0),
            q: QUAT_IDENTITY,
        };
        let mut manifold = LocalManifold::default();
        let mut cache = SatCache {
            type_: SeparatingFeature::ManualEdgePairAxis as u8,
            ..Default::default()
        };
        collide_hulls(
            &mut manifold,
            8,
            &hull_a.base,
            &hull_b.base,
            transform,
            &mut cache,
        );

        assert_eq!(manifold.point_count, 1);
        ensure_small(manifold.points[0].separation - expected, 1e-5);
        ensure_small(manifold.points[0].point.y - 0.49, 1e-5);
    }

    // Off the end. The edge pair no longer describes a contact, so the builder rejects it and
    // clears the cache rather than clamping to a point that is not on the hulls.
    {
        let transform = Transform {
            p: v(d, 0.55, 0.0),
            q: QUAT_IDENTITY,
        };
        let mut manifold = LocalManifold::default();
        let mut cache = SatCache {
            type_: SeparatingFeature::ManualEdgePairAxis as u8,
            ..Default::default()
        };
        collide_hulls(
            &mut manifold,
            8,
            &hull_a.base,
            &hull_b.base,
            transform,
            &mut cache,
        );

        assert_eq!(manifold.point_count, 0);
        assert_eq!(cache.type_, SeparatingFeature::InvalidAxis as u8);

        // The true gap is a vertex to edge distance well past the speculative distance
        let mut fresh_cache = SatCache::default();
        collide_hulls(
            &mut manifold,
            8,
            &hull_a.base,
            &hull_b.base,
            transform,
            &mut fresh_cache,
        );
        assert_eq!(manifold.point_count, 0);
        assert!(fresh_cache.separation > 0.0);
    }
}

// Cubes stacked face to face and tipped by a hair. A third of the edge pairs are then nearly
// parallel, the angle between them is at the noise floor and the arc intersection carries no
// information. The face contact has to survive that untouched.
#[test]
fn parallel_edge_test() {
    let hull_a = make_box_hull(0.5, 0.5, 0.5);
    let hull_b = make_box_hull(0.5, 0.5, 0.5);

    let overlap = 0.01;

    for &tilt_axis in &TILT_AXES {
        for &angle in &TILT_ANGLES {
            let transform = Transform {
                p: v(0.0, 1.0 - overlap, 0.0),
                q: exact_quat(tilt_axis, angle),
            };

            let mut manifold = LocalManifold::default();
            let mut cache = SatCache::default();
            collide_hulls(
                &mut manifold,
                8,
                &hull_a.base,
                &hull_b.base,
                transform,
                &mut cache,
            );

            assert_eq!(manifold.point_count, 4);
            assert!(
                cache.type_ == SeparatingFeature::FaceAxisA as u8
                    || cache.type_ == SeparatingFeature::FaceAxisB as u8
            );
            assert!(dot(manifold.normal, AXIS_Y) > 0.998);

            // The tilt can only lift or sink a face point by the length of the arc it sweeps
            let bound = HALF_DIAGONAL * angle + 1e-5;
            for k in 0..manifold.point_count {
                ensure_small(manifold.points[k as usize].separation + overlap, bound);
            }
        }
    }
}

// Same stack, but force the edge query to answer. With the edges exactly parallel no pair forms a
// Minkowski face at all, and once a pair does form its separation can never be positive because
// the hulls overlap.
#[test]
fn parallel_edge_manual_test() {
    let hull_a = make_box_hull(0.5, 0.5, 0.5);
    let hull_b = make_box_hull(0.5, 0.5, 0.5);

    let overlap = 0.01;

    for &tilt_axis in &TILT_AXES {
        for &angle in &TILT_ANGLES {
            let transform = Transform {
                p: v(0.0, 1.0 - overlap, 0.0),
                q: exact_quat(tilt_axis, angle),
            };

            let mut manifold = LocalManifold::default();
            let mut cache = SatCache {
                type_: SeparatingFeature::ManualEdgePairAxis as u8,
                ..Default::default()
            };
            collide_hulls(
                &mut manifold,
                8,
                &hull_a.base,
                &hull_b.base,
                transform,
                &mut cache,
            );

            if angle == 0.0 {
                // Every pair is parallel so the query finds nothing and leaves the cache alone
                assert_eq!(manifold.point_count, 0);
                assert_eq!(cache.type_, SeparatingFeature::ManualEdgePairAxis as u8);
                continue;
            }

            // The closest points can fall off the ends of the segments
            if manifold.point_count == 0 {
                continue;
            }

            assert_eq!(manifold.point_count, 1);
            assert!(dot(manifold.normal, AXIS_Y) > 0.99);

            let separation = manifold.points[0].separation;
            assert!(separation <= 0.0);
            assert!(separation >= -overlap - HALF_DIAGONAL * angle - 1e-4);
        }
    }
}

// Overlapping hulls admit no separating axis, so an edge separation that comes back positive is
// always noise. It shows up as a manifold with no points, which the solver reads as no contact.
#[test]
fn overlap_never_empty_test() {
    let hull_a = make_box_hull(0.5, 0.5, 0.5);
    let hull_b = make_box_hull(0.4, 0.6, 0.5);

    let mut rng = Rng::new(12345);

    for i in 0..2000 {
        let axis = rng.next_direction();

        // Half the samples are nearly aligned, where the edge cross products are smallest
        let angle = if (i & 1) != 0 {
            rng.next_float(-0.01, 0.01)
        } else {
            rng.next_float(-PI, PI)
        };

        // Shorter than the smallest half width, so the center of B is inside A
        let offset = mul_sv(0.4, rng.next_direction());

        let transform = Transform {
            p: offset,
            q: exact_quat(axis, angle),
        };

        let mut manifold = LocalManifold::default();
        let mut cache = SatCache::default();
        collide_hulls(
            &mut manifold,
            8,
            &hull_a.base,
            &hull_b.base,
            transform,
            &mut cache,
        );

        assert!(manifold.point_count > 0);
        ensure_small(normal_length(&manifold) - 1.0, 1e-5);

        // Clipping keeps points that are separated, but the deepest one must penetrate
        assert!(min_separation(&manifold) < 0.0);
    }
}

// A crossed ridge pair must land on a four point roof face contact. The clipped face
// separation can be no deeper than root2 times the vertical overlap. (CheckRoofFaceContact)
fn check_roof_face_contact(manifold: &LocalManifold, cache: &SatCache, overlap: f32) {
    assert_eq!(manifold.point_count, 4);
    assert!(
        cache.type_ == SeparatingFeature::FaceAxisA as u8
            || cache.type_ == SeparatingFeature::FaceAxisB as u8
    );

    // A roof face of one hull, so 45 degrees off the vertical
    ensure_small(manifold.normal.y - HALF_ROOT2, 1e-4);

    let min_separation = min_separation(manifold);
    assert!(min_separation < -HALF_ROOT2 * overlap + 1e-4);
    assert!(min_separation > -ROOT2 * overlap - 1e-4);
}

// Two long roof ridges laid across each other. The axis of minimum penetration is the edge
// pair, but a one point edge contact is weak for stacking. The collider builds the roof face
// contact first and only switches to the edge contact when the edge axis beats the clipped
// face separation by more than the slop. This pins all three regimes of that policy.
#[test]
fn ridge_crossing_test() {
    let hull_a = make_transformed_box_hull(1.5, 0.1, 0.1, exact_rotation(AXIS_X, 0.25 * PI));
    let hull_b = make_transformed_box_hull(1.5, 0.1, 0.1, exact_rotation(AXIS_X, 0.25 * PI));

    let ridge_y = 0.1 * ROOT2;

    // Shallow overlap. The edge axis is better by only ( root2 - 1 ) * overlap, inside the
    // slop, so the four point face contact carries the crossing at every angle.
    {
        let overlap = 0.01;
        let lift = 2.0 * ridge_y - overlap;
        let crossing_angles = [0.0f32, 1e-3, 0.02, 0.1, 0.5];

        for &angle in &crossing_angles {
            let transform = Transform {
                p: v(0.0, lift, 0.0),
                q: exact_quat(AXIS_Y, angle),
            };

            let mut manifold = LocalManifold::default();
            let mut cache = SatCache::default();
            collide_hulls(
                &mut manifold,
                8,
                &hull_a.base,
                &hull_b.base,
                transform,
                &mut cache,
            );

            check_roof_face_contact(&manifold, &cache, overlap);
        }
    }

    // Deep overlap at a clear crossing. The edge axis now beats the clipped face separation
    // by more than the slop, so the edge contact replaces the face contact.
    {
        let overlap = 0.05;
        let lift = 2.0 * ridge_y - overlap;
        let crossing_angles = [0.05f32, 0.1, 0.2, 0.5];

        for &angle in &crossing_angles {
            let transform = Transform {
                p: v(0.0, lift, 0.0),
                q: exact_quat(AXIS_Y, angle),
            };

            let mut manifold = LocalManifold::default();
            let mut cache = SatCache::default();
            collide_hulls(
                &mut manifold,
                8,
                &hull_a.base,
                &hull_b.base,
                transform,
                &mut cache,
            );

            assert_eq!(manifold.point_count, 1);
            assert!(edge_pair(&cache));
            ensure_small(manifold.normal.x, 1e-4);
            ensure_small(manifold.normal.y - 1.0, 1e-4);
            ensure_small(manifold.normal.z, 1e-4);
            ensure_small(manifold.points[0].separation + overlap, 1e-4);
            ensure_small(manifold.points[0].point.y - (ridge_y - 0.5 * overlap), 1e-4);

            // Only has to land near the crossing, not at the end of a three meter beam
            ensure_small(manifold.points[0].point.x, 0.01);
            ensure_small(manifold.points[0].point.z, 0.01);
        }
    }

    // Deep overlap near parallel. A one point edge contact off a parallel pair would have a
    // normal built from noise, so the roof faces keep the contact.
    {
        let overlap = 0.05;
        let lift = 2.0 * ridge_y - overlap;
        let shallow_angles = [0.0f32, 1e-4, 1e-3, 0.003];

        for &angle in &shallow_angles {
            let transform = Transform {
                p: v(0.0, lift, 0.0),
                q: exact_quat(AXIS_Y, angle),
            };

            let mut manifold = LocalManifold::default();
            let mut cache = SatCache::default();
            collide_hulls(
                &mut manifold,
                8,
                &hull_a.base,
                &hull_b.base,
                transform,
                &mut cache,
            );

            check_roof_face_contact(&manifold, &cache, overlap);
        }
    }
}

// Two boxes crossing edge to edge. A holds a vertical edge, B is rolled to present a crossing edge
// and yawed so the arc intersection walks off the midpoint. For every configuration that resolves
// to an edge pair the recovered axis, separation and point must match the cross product oracle to
// tight tolerance.
#[test]
fn edge_axis_oracle_test() {
    let hull_a = make_transformed_box_hull(0.5, 0.5, 0.5, exact_rotation(AXIS_Y, 0.25 * PI));

    let rolls = [0.18 * PI, 0.25 * PI, 0.32 * PI];
    let yaws = [-0.35f32, -0.15, 0.0, 0.15, 0.35];
    let distances = [1.38f32, 1.40, ROOT2, 1.44];

    let mut edge_contacts = 0;

    for &roll in &rolls {
        let hull_b = make_transformed_box_hull(0.5, 0.5, 0.5, exact_rotation(AXIS_Z, roll));

        for &yaw in &yaws {
            for &d in &distances {
                let transform = Transform {
                    p: v(d, 0.0, 0.0),
                    q: exact_quat(AXIS_Y, yaw),
                };

                let mut manifold = LocalManifold::default();
                let mut cache = SatCache::default();
                collide_hulls(
                    &mut manifold,
                    8,
                    &hull_a.base,
                    &hull_b.base,
                    transform,
                    &mut cache,
                );

                if !edge_pair(&cache) || manifold.point_count != 1 {
                    continue;
                }

                let (p1, e1) =
                    hull_edge_segment(&hull_a.base, cache.index_a as i32, TRANSFORM_IDENTITY);
                let (p2, e2) = hull_edge_segment(&hull_b.base, cache.index_b as i32, transform);

                let center_a = hull_a.base.center;
                let center_b = transform_point(transform, hull_b.base.center);
                let orient_ref = crate::math_functions::sub(center_b, center_a);

                check_edge_contact(&manifold, p1, e1, p2, e2, orient_ref, 0.0, 2e-4, 2e-4, 2e-3);

                edge_contacts += 1;
            }
        }
    }

    // The sweep is only meaningful if it actually drove the edge path
    assert!(edge_contacts >= 15);
}

// The same oracle over randomly oriented box pairs. Whenever the solver reports an edge pair the
// recovered axis must be perpendicular to both edges and match the cross product.
#[test]
fn edge_axis_random_oracle_test() {
    let mut rng = Rng::new(246813579);

    let mut edge_contacts = 0;

    for _ in 0..2000 {
        let angle_a = rng.next_float(0.2, 0.5) * PI;
        let angle_b = rng.next_float(0.2, 0.5) * PI;
        let dir_a = rng.next_direction();
        let hull_a = make_transformed_box_hull(0.5, 0.5, 0.5, exact_rotation(dir_a, angle_a));
        let hull_b = make_box_hull(0.5, 0.5, 0.5);

        let d = rng.next_float(1.2, 1.55);
        let dir_offset = rng.next_direction();
        let dir_rotation = rng.next_direction();
        let transform = Transform {
            p: mul_sv(d, dir_offset),
            q: exact_quat(dir_rotation, angle_b),
        };

        let mut manifold = LocalManifold::default();
        let mut cache = SatCache::default();
        collide_hulls(
            &mut manifold,
            8,
            &hull_a.base,
            &hull_b.base,
            transform,
            &mut cache,
        );

        if !edge_pair(&cache) || manifold.point_count != 1 {
            continue;
        }

        let (p1, e1) = hull_edge_segment(&hull_a.base, cache.index_a as i32, TRANSFORM_IDENTITY);
        let (p2, e2) = hull_edge_segment(&hull_b.base, cache.index_b as i32, transform);

        // Skip crossings near parallel where the closest point solve is ill conditioned. The
        // parallel rejection itself is covered by parallel_edge_test.
        let sine = crate::math_functions::length(crate::math_functions::cross(
            crate::math_functions::normalize(e1),
            crate::math_functions::normalize(e2),
        ));
        if sine < 0.1 {
            continue;
        }

        let orient_ref = crate::math_functions::sub(
            transform_point(transform, hull_b.base.center),
            hull_a.base.center,
        );

        check_edge_contact(&manifold, p1, e1, p2, e2, orient_ref, 0.0, 1e-3, 1e-3, 5e-3);

        edge_contacts += 1;
    }

    assert!(edge_contacts >= 100);
}
