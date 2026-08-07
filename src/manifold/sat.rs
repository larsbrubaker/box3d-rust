//! SAT face/edge queries and manifold point reduction from `convex_manifold.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{LocalManifold, LocalManifoldPoint, SeparatingAxis, SeparatingFeature};
use crate::constants::speculative_distance;
use crate::core::NULL_INDEX;
use crate::distance::get_point_support;
use crate::geometry::Capsule;
use crate::hull::{get_hull_edges, get_hull_planes, get_hull_points, HullData};
use crate::math_functions::{
    abs_float, arbitrary_perp, cross, dot, length_squared, lerp, max_float, mul_sub, neg,
    normalize, plane_separation, sub, transform_point, Transform, VEC3_ZERO,
};

/// Face directions for hull vs capsule. (static b3QueryFaceDirectionHullAndCapsule)
pub(crate) fn query_face_direction_hull_and_capsule(
    hull: &HullData,
    capsule: &Capsule,
    capsule_transform: Transform,
) -> SeparatingAxis {
    let mut max_face_index = -1;
    let mut max_vertex_index = -1;
    let mut max_face_separation = -f32::MAX;
    let planes = get_hull_planes(hull);

    let capsule_points = [
        transform_point(capsule_transform, capsule.center1),
        transform_point(capsule_transform, capsule.center2),
    ];

    for face_index in 0..hull.face_count {
        let plane = planes[face_index as usize];
        let vertex_index = get_point_support(&capsule_points, neg(plane.normal));
        let support = capsule_points[vertex_index as usize];
        let separation = plane_separation(plane, support);
        if separation > max_face_separation {
            max_vertex_index = vertex_index;
            max_face_index = face_index;
            max_face_separation = separation;
        }
    }

    SeparatingAxis {
        normal: planes[max_face_index as usize].normal,
        separation: max_face_separation,
        // Match C's (uint8_t) cast into the int fields.
        index_a: max_face_index as u8 as i32,
        index_b: max_vertex_index as u8 as i32,
        type_: SeparatingFeature::InvalidAxis,
    }
}

/// Edge directions for hull vs capsule. (static b3QueryEdgeDirectionHullAndCapsule)
pub(crate) fn query_edge_direction_hull_and_capsule(
    hull: &HullData,
    capsule: &Capsule,
    capsule_transform: Transform,
) -> SeparatingAxis {
    // Find axis of minimum penetration
    let mut max_normal = VEC3_ZERO;
    let mut max_separation = -f32::MAX;
    let mut max_index_a = NULL_INDEX;
    let mut max_index_b = NULL_INDEX;

    // We perform all computations in local space of the hull
    let p_a = transform_point(capsule_transform, capsule.center1);
    let q_a = transform_point(capsule_transform, capsule.center2);
    let e_a = sub(q_a, p_a);

    let edges = get_hull_edges(hull);
    let points = get_hull_points(hull);
    let planes = get_hull_planes(hull);
    let squared_tolerance = 0.005 * 0.005;

    let mut index = 0;
    while index < hull.edge_count {
        let edge = &edges[index as usize];
        let twin = &edges[(index + 1) as usize];
        debug_assert!(edge.twin as i32 == index + 1 && twin.twin as i32 == index);

        let q_b = points[twin.origin as usize];
        let u_b = planes[edge.face as usize].normal;
        let v_b = planes[twin.face as usize].normal;

        // An isolated edge (e.g. like in a capsule) defines a circle through the
        // origin on the Gauss map. So testing for overlap between this circle and
        // the arc AB simplifies to a plane test.
        let cba = dot(u_b, e_a);
        let dba = dot(v_b, e_a);

        if cba * dba < 0.0 {
            // Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
            if max_float(cba * cba, dba * dba) < squared_tolerance * length_squared(e_a) {
                index += 2;
                continue;
            }

            // The intersection of the arcs on the Gauss map is the edge pair axis. Cast the
            // arc of hull B (from uB to vB) against the plane containing the arc of hull A:
            // dot(uB + t * (vB - uB), eA) == 0
            // then
            // t = cba / (cba - dba)
            //
            // The signs of cba and dba differ (Minkowski test), so the division is safe.
            //
            // The axis generated points from B to A by construction since it lands between
            // two face normals on B. This removes the need to orient the separation axis
            // using the hull centers.
            //
            // The axis is perpendicular to both edges so I can use qA and qB as arbitrary
            // points on edgeA and edgeB to measure the separation.
            let t = cba / (cba - dba);
            let mut axis = lerp(u_b, v_b, t);
            debug_assert!(length_squared(axis) > 1000.0 * f32::MIN_POSITIVE);
            axis = normalize(axis);
            let separation = dot(axis, sub(q_a, q_b));

            if separation > max_separation {
                // Note: We don't exit early if we find a separating axis here since we want to
                // find the best one for caching and account for the convex radius later.
                max_normal = axis;
                max_separation = separation;
                max_index_a = 0;
                max_index_b = index;
            }
        }

        index += 2;
    }

    // Save result
    SeparatingAxis {
        normal: max_normal,
        separation: max_separation,
        index_a: max_index_a,
        index_b: max_index_b,
        type_: SeparatingFeature::InvalidAxis,
    }
}

/// Reduce manifold points to a maximum of 4. Modifies `points` in place.
/// (static b3ReduceManifoldPoints)
pub(crate) fn reduce_manifold_points(
    manifold: &mut LocalManifold,
    capacity: i32,
    points: &mut [LocalManifoldPoint],
    mut count: i32,
) {
    if capacity < 4 {
        return;
    }

    if count <= 4 {
        for i in 0..count {
            manifold.points[i as usize] = points[i as usize];
        }
        manifold.point_count = count;
        return;
    }

    let normal = manifold.normal;
    let speculative = speculative_distance();
    let tol_sqr = speculative * speculative;
    let bias = 0.95;

    // Step 1: find extreme point that is touching
    let mut best_index = NULL_INDEX;
    let mut best_score = -f32::MAX;
    let search_direction = arbitrary_perp(normal);

    for index in 0..count {
        let pt = &points[index as usize];
        if pt.separation > speculative {
            continue;
        }
        let score = -pt.separation + dot(search_direction, pt.point);
        if bias * score > best_score {
            best_index = index;
            best_score = score;
        }
    }

    debug_assert!(best_index == NULL_INDEX || (0 <= best_index && best_index < count));
    if best_index == NULL_INDEX {
        manifold.point_count = 0;
        return;
    }

    manifold.points[0] = points[best_index as usize];
    manifold.point_count = 1;
    points[best_index as usize] = points[(count - 1) as usize];
    count -= 1;

    let a = manifold.points[0].point;

    // Step 2: farthest point in 2D
    best_score = 0.0;
    best_index = NULL_INDEX;
    let mut max_distance_squared = 0.0;

    for index in 0..count {
        let p = points[index as usize].point;
        let d = crate::math_functions::sub(p, a);
        let v = mul_sub(d, dot(d, normal), normal);
        let distance_squared = length_squared(v);
        max_distance_squared = max_float(max_distance_squared, distance_squared);
        let separation = max_float(0.0, -points[index as usize].separation);
        let score = distance_squared + 4.0 * separation * separation;
        if bias * score > best_score {
            best_score = score;
            best_index = index;
        }
    }

    let _ = max_distance_squared;

    if best_score < tol_sqr {
        return;
    }

    debug_assert!(0 <= best_index && best_index < count);
    manifold.points[1] = points[best_index as usize];
    manifold.point_count = 2;
    points[best_index as usize] = points[(count - 1) as usize];
    count -= 1;

    let b = manifold.points[1].point;

    // Step 3: maximum triangular area
    best_score = tol_sqr;
    best_index = NULL_INDEX;
    let mut best_signed_area = 0.0;
    let ba = crate::math_functions::sub(b, a);
    for index in 0..count {
        let p = points[index as usize].point;
        let signed_area = dot(normal, cross(ba, crate::math_functions::sub(p, a)));
        let score = abs_float(signed_area);
        if bias * score >= best_score {
            best_score = score;
            best_index = index;
            best_signed_area = signed_area;
        }
    }

    if best_index == NULL_INDEX {
        return;
    }

    manifold.points[2] = points[best_index as usize];
    manifold.point_count = 3;
    points[best_index as usize] = points[(count - 1) as usize];
    count -= 1;

    let c = manifold.points[2].point;

    // Step 4: point that adds the most area outside the triangle
    best_score = tol_sqr;
    best_index = NULL_INDEX;
    let sign = if best_signed_area < 0.0 { -1.0 } else { 1.0 };
    for index in 0..count {
        let p = points[index as usize].point;
        let u1 = sign * dot(normal, cross(crate::math_functions::sub(p, a), ba));
        let u2 = sign
            * dot(
                normal,
                cross(
                    crate::math_functions::sub(p, b),
                    crate::math_functions::sub(c, b),
                ),
            );
        let u3 = sign
            * dot(
                normal,
                cross(
                    crate::math_functions::sub(p, c),
                    crate::math_functions::sub(a, c),
                ),
            );
        let score = max_float(u1, max_float(u2, u3));

        if bias * score > best_score {
            best_score = score;
            best_index = index;
        }
    }

    if best_index != NULL_INDEX {
        manifold.points[manifold.point_count as usize] = points[best_index as usize];
        manifold.point_count += 1;
    }
}
