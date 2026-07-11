//! Capsule-vs-capsule contact manifold from `convex_manifold.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::clip::clip_segment;
use super::types::{
    make_feature_pair, ClipVertex, FeatureOwner, LocalManifold, FEATURE_PAIR_SINGLE,
};
use crate::constants::{linear_slop, min_capsule_length, speculative_distance};
use crate::geometry::Capsule;
use crate::math_functions::{
    add, cross, distance, dot, get_length_and_normalize, length_squared, mul_add, mul_sub, mul_sv,
    neg, normalize, point_to_segment_distance, segment_distance, sub, transform_point, Plane,
    Transform,
};

/// Collide two capsules. (b3CollideCapsules)
pub fn collide_capsules(
    manifold: &mut LocalManifold,
    capacity: i32,
    capsule_a: &Capsule,
    capsule_b: &Capsule,
    transform_b_to_a: Transform,
) {
    manifold.point_count = 0;

    if capacity < 2 {
        return;
    }

    // Work in shapeA coordinates
    let center_a1 = capsule_a.center1;
    let center_a2 = capsule_a.center2;
    let center_b1 = transform_point(transform_b_to_a, capsule_b.center1);
    let center_b2 = transform_point(transform_b_to_a, capsule_b.center2);

    let radius = capsule_a.radius + capsule_b.radius;
    let max_distance = radius + speculative_distance();

    let result = segment_distance(center_a1, center_a2, center_b1, center_b2);
    let offset = sub(result.point2, result.point1);
    let distance_squared = length_squared(offset);
    let slop = linear_slop();
    let min_distance = 0.01 * slop;

    if distance_squared > max_distance * max_distance
        || distance_squared < min_distance * min_distance
    {
        // We found a separating axis
        return;
    }

    let mut length_a = 0.0;
    let segment_a = sub(center_a2, center_a1);
    let edge_a = get_length_and_normalize(&mut length_a, segment_a);
    if length_a < min_capsule_length() {
        return;
    }

    let mut length_b = 0.0;
    let segment_b = sub(center_b2, center_b1);
    let edge_b = get_length_and_normalize(&mut length_b, segment_b);
    if length_b < min_capsule_length() {
        return;
    }

    // Parallel edges: |eA x eB| = sin(alpha)
    const ALPHA_TOL: f32 = 0.05;
    let alpha_tol_sqr = ALPHA_TOL * ALPHA_TOL;
    let axis = cross(edge_a, edge_b);

    // Try to create two contact points if the capsules are nearly parallel
    if length_squared(axis) < alpha_tol_sqr {
        // Clip segment B against side planes of segment A

        // Sides planes of A
        let planes_a = [
            Plane {
                normal: neg(edge_a),
                offset: -dot(edge_a, capsule_a.center1),
            },
            Plane {
                normal: edge_a,
                offset: dot(edge_a, capsule_a.center2),
            },
        ];

        // Clip points for B
        let mut vertices_b = [
            ClipVertex {
                position: center_b1,
                separation: 0.0,
                pair: make_feature_pair(FeatureOwner::ShapeA, 0, FeatureOwner::ShapeA, 0),
            },
            ClipVertex {
                position: center_b2,
                separation: 0.0,
                pair: make_feature_pair(FeatureOwner::ShapeA, 1, FeatureOwner::ShapeA, 1),
            },
        ];

        let mut point_count = clip_segment(&mut vertices_b, planes_a[0]);
        if point_count == 2 {
            point_count = clip_segment(&mut vertices_b, planes_a[1]);
        }

        if point_count == 2 {
            // Closest points on A to the clipped points on B.
            let closest_point1 =
                point_to_segment_distance(center_a1, center_a2, vertices_b[0].position);
            let closest_point2 =
                point_to_segment_distance(center_a1, center_a2, vertices_b[1].position);

            let distance1 = distance(closest_point1, vertices_b[0].position);
            let distance2 = distance(closest_point2, vertices_b[1].position);
            if distance1 <= radius && distance2 <= radius {
                if distance1 < min_distance || distance2 < min_distance {
                    // Avoid divide by zero
                    return;
                }

                let normal1 = mul_sv(1.0 / distance1, sub(vertices_b[0].position, closest_point1));
                let normal2 = mul_sv(1.0 / distance2, sub(vertices_b[1].position, closest_point2));
                let normal = normalize(add(normal1, normal2));
                let radius_a = capsule_a.radius;
                let radius_b = capsule_b.radius;

                // Contact is at the midpoint: 0.5 * (((vB.pos + rA*nK) + cP) - rB*n)
                let point1 = mul_sv(
                    0.5,
                    mul_sub(
                        add(
                            mul_add(vertices_b[0].position, radius_a, normal1),
                            closest_point1,
                        ),
                        radius_b,
                        normal,
                    ),
                );
                let point2 = mul_sv(
                    0.5,
                    mul_sub(
                        add(
                            mul_add(vertices_b[1].position, radius_a, normal2),
                            closest_point2,
                        ),
                        radius_b,
                        normal,
                    ),
                );

                // Manifold in frame A
                manifold.normal = normal;
                manifold.point_count = 2;

                let pt1 = &mut manifold.points[0];
                pt1.point = point1;
                pt1.separation = distance1 - radius;
                pt1.pair = vertices_b[0].pair;

                let pt2 = &mut manifold.points[1];
                pt2.point = point2;
                pt2.separation = distance2 - radius;
                pt2.pair = vertices_b[1].pair;

                return;
            }
        }
    }

    let mut distance = 0.0;
    let normal = get_length_and_normalize(&mut distance, offset);
    // Contact at the midpoint 0.5 * (((p1 + rA*n) + p2) - rB*n)
    let point = mul_sv(
        0.5,
        mul_sub(
            add(
                mul_add(result.point1, capsule_a.radius, normal),
                result.point2,
            ),
            capsule_b.radius,
            normal,
        ),
    );

    // Manifold in frame A
    manifold.normal = normal;
    manifold.point_count = 1;

    let pt = &mut manifold.points[0];
    pt.point = point;
    pt.separation = distance - radius;
    pt.pair = FEATURE_PAIR_SINGLE;
}
