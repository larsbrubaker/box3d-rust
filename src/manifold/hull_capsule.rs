//! Hull-vs-capsule contact manifold from `convex_manifold.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::clip::clip_segment_to_hull_face;
use super::sat::{
    deepest_point_separation, query_edge_direction_hull_and_capsule,
    query_face_direction_hull_and_capsule,
};
use super::types::{
    make_feature_pair, ClipVertex, EdgeQuery, FaceQuery, FeatureOwner, LocalManifold,
    FEATURE_PAIR_SINGLE,
};
use crate::constants::{linear_slop, speculative_distance};
use crate::distance::{make_proxy, shape_distance, DistanceInput, SimplexCache};
use crate::geometry::Capsule;
use crate::hull::{
    find_hull_support_face, get_hull_edges, get_hull_planes, get_hull_points, HullData,
};
use crate::math_functions::{
    abs_float, add, cross, dot, is_within_segments, line_distance, mul_sv, mul_sub, neg, normalize,
    plane_separation, sub, transform_point, Transform,
};

/// Build face contact between a hull face and a capsule. (static b3BuildHullFaceAndCapsuleContact)
fn build_hull_face_and_capsule_contact(
    manifold: &mut LocalManifold,
    hull_a: &HullData,
    capsule_b: &Capsule,
    transform_b_to_a: Transform,
    query: FaceQuery,
) -> bool {
    let planes = get_hull_planes(hull_a);
    let ref_face = query.face_index;
    let ref_plane = planes[ref_face as usize];

    let mut segment_b = [
        ClipVertex {
            position: transform_point(transform_b_to_a, capsule_b.center1),
            separation: 0.0,
            pair: make_feature_pair(FeatureOwner::ShapeA, 0, FeatureOwner::ShapeA, 0),
        },
        ClipVertex {
            position: transform_point(transform_b_to_a, capsule_b.center2),
            separation: 0.0,
            pair: make_feature_pair(FeatureOwner::ShapeA, 1, FeatureOwner::ShapeA, 1),
        },
    ];

    let point_count = clip_segment_to_hull_face(&mut segment_b, hull_a, ref_face);
    if point_count < 2 {
        return false;
    }

    let distance1 = plane_separation(ref_plane, segment_b[0].position);
    let distance2 = plane_separation(ref_plane, segment_b[1].position);
    let speculative = speculative_distance();

    if distance1 <= speculative || distance2 <= speculative {
        let normal = ref_plane.normal;
        let point1 = mul_sub(
            segment_b[0].position,
            0.5 * (distance1 + capsule_b.radius),
            normal,
        );
        let point2 = mul_sub(
            segment_b[1].position,
            0.5 * (distance2 + capsule_b.radius),
            normal,
        );

        manifold.normal = normal;
        manifold.point_count = 2;

        let pt1 = &mut manifold.points[0];
        pt1.point = point1;
        pt1.separation = distance1 - capsule_b.radius;
        pt1.pair = segment_b[0].pair;

        let pt2 = &mut manifold.points[1];
        pt2.point = point2;
        pt2.separation = distance2 - capsule_b.radius;
        pt2.pair = segment_b[1].pair;

        return true;
    }

    false
}

/// Build edge contact between a hull edge and a capsule. (static b3BuildHullAndCapsuleEdgeContact)
fn build_hull_and_capsule_edge_contact(
    manifold: &mut LocalManifold,
    capacity: i32,
    hull_a: &HullData,
    capsule_b: &Capsule,
    transform_b_to_a: Transform,
    query: EdgeQuery,
) -> bool {
    if capacity < 1 {
        return false;
    }

    let pc = transform_point(transform_b_to_a, capsule_b.center1);
    let qc = transform_point(transform_b_to_a, capsule_b.center2);
    let ec = sub(qc, pc);

    let edges = get_hull_edges(hull_a);
    let points = get_hull_points(hull_a);

    let edge2 = &edges[query.index_b as usize];
    let twin2 = &edges[edge2.twin as usize];
    let ch = hull_a.center;
    let ph = points[edge2.origin as usize];
    let qh = points[twin2.origin as usize];
    let eh = sub(qh, ph);

    let mut normal = normalize(cross(ec, eh));

    if dot(normal, sub(ph, ch)) < 0.0 {
        normal = neg(normal);
    }

    let result = line_distance(ph, eh, pc, ec);

    if !is_within_segments(&result) {
        return false;
    }

    let point = mul_sv(
        0.5,
        add(mul_sub(result.point1, capsule_b.radius, normal), result.point2),
    );

    let separation = dot(normal, sub(result.point2, result.point1));
    debug_assert!(abs_float(separation - query.separation) < linear_slop());

    manifold.normal = normal;
    manifold.point_count = 1;

    let pt = &mut manifold.points[0];
    pt.point = point;
    pt.separation = separation - capsule_b.radius;
    pt.pair = make_feature_pair(
        FeatureOwner::ShapeA,
        query.index_a,
        FeatureOwner::ShapeB,
        query.index_b,
    );
    true
}

/// Collide a hull and a capsule. (b3CollideHullAndCapsule)
pub fn collide_hull_and_capsule(
    manifold: &mut LocalManifold,
    capacity: i32,
    hull_a: &HullData,
    capsule_b: &Capsule,
    transform_b_to_a: Transform,
    cache: &mut SimplexCache,
) {
    manifold.point_count = 0;

    if capacity < 2 {
        return;
    }

    let distance_input = DistanceInput {
        proxy_a: make_proxy(get_hull_points(hull_a), 0.0),
        proxy_b: make_proxy(&[capsule_b.center1, capsule_b.center2], 0.0),
        transform: transform_b_to_a,
        use_radii: false,
    };

    let distance_output = shape_distance(&distance_input, cache, None);
    let speculative = speculative_distance();

    if distance_output.distance > capsule_b.radius + speculative {
        *cache = SimplexCache::default();
        return;
    }

    if distance_output.distance > 100.0 * f32::EPSILON {
        let planes = get_hull_planes(hull_a);

        let delta = distance_output.normal;
        let ref_face = find_hull_support_face(hull_a, delta);
        let ref_plane = planes[ref_face as usize];

        const K_TOLERANCE: f32 = 0.998;
        if abs_float(dot(ref_plane.normal, delta)) > K_TOLERANCE {
            let mut vertices_b = [
                ClipVertex {
                    position: transform_point(transform_b_to_a, capsule_b.center1),
                    separation: 0.0,
                    pair: make_feature_pair(FeatureOwner::ShapeA, 0, FeatureOwner::ShapeA, 0),
                },
                ClipVertex {
                    position: transform_point(transform_b_to_a, capsule_b.center2),
                    separation: 0.0,
                    pair: make_feature_pair(FeatureOwner::ShapeA, 1, FeatureOwner::ShapeA, 1),
                },
            ];

            let point_count = clip_segment_to_hull_face(&mut vertices_b, hull_a, ref_face);

            if point_count == 2 {
                let distance1 = plane_separation(ref_plane, vertices_b[0].position);
                let distance2 = plane_separation(ref_plane, vertices_b[1].position);
                if distance1 <= capsule_b.radius + speculative
                    || distance2 <= capsule_b.radius + speculative
                {
                    let normal = ref_plane.normal;
                    let point1 = mul_sub(
                        vertices_b[0].position,
                        0.5 * (capsule_b.radius + distance1),
                        normal,
                    );
                    let point2 = mul_sub(
                        vertices_b[1].position,
                        0.5 * (capsule_b.radius + distance2),
                        normal,
                    );

                    manifold.normal = normal;
                    manifold.point_count = 2;

                    let pt1 = &mut manifold.points[0];
                    pt1.point = point1;
                    pt1.separation = distance1 - capsule_b.radius;
                    pt1.pair = vertices_b[0].pair;

                    let pt2 = &mut manifold.points[1];
                    pt2.point = point2;
                    pt2.separation = distance2 - capsule_b.radius;
                    pt2.pair = vertices_b[1].pair;

                    return;
                }
            }
        }

        let point = mul_sv(
            0.5,
            add(
                mul_sub(distance_output.point_a, capsule_b.radius, delta),
                distance_output.point_b,
            ),
        );

        manifold.normal = delta;
        manifold.point_count = 1;

        let pt = &mut manifold.points[0];
        pt.point = point;
        pt.separation = distance_output.distance - capsule_b.radius;
        pt.pair = FEATURE_PAIR_SINGLE;
        return;
    }

    // Deep penetration
    let face_query = query_face_direction_hull_and_capsule(hull_a, capsule_b, transform_b_to_a);
    if face_query.separation > capsule_b.radius {
        return;
    }

    let edge_query = query_edge_direction_hull_and_capsule(hull_a, capsule_b, transform_b_to_a);
    if edge_query.separation > capsule_b.radius {
        return;
    }

    let mut face_separation = face_query.separation - capsule_b.radius;
    build_hull_face_and_capsule_contact(manifold, hull_a, capsule_b, transform_b_to_a, face_query);
    if manifold.point_count > 1 {
        face_separation = deepest_point_separation(manifold);
    }
    debug_assert!(face_separation <= 0.0);

    const K_REL_EDGE_TOLERANCE: f32 = 0.90;
    let k_abs_tolerance = 0.5 * linear_slop();
    let edge_separation = edge_query.separation - capsule_b.radius;
    if manifold.point_count == 0
        || edge_separation > K_REL_EDGE_TOLERANCE * face_separation + k_abs_tolerance
    {
        build_hull_and_capsule_edge_contact(
            manifold,
            capacity,
            hull_a,
            capsule_b,
            transform_b_to_a,
            edge_query,
        );
    }
}
