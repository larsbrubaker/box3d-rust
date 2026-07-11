//! SAT face/edge queries and manifold point reduction from `convex_manifold.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::clip::{edge_edge_separation, is_minkowski_face_isolated};
use super::types::{EdgeQuery, FaceQuery, LocalManifold, LocalManifoldPoint};
use crate::constants::speculative_distance;
use crate::core::NULL_INDEX;
use crate::distance::get_point_support;
use crate::geometry::Capsule;
use crate::hull::{
    find_hull_support_vertex, get_hull_edges, get_hull_planes, get_hull_points, HullData,
};
use crate::math_functions::{
    abs_float, add, arbitrary_perp, cross, dot, invert_transform, length_squared,
    make_matrix_from_quat, max_float, min_float, mul_mv, mul_sub, mul_sv, neg, plane_separation,
    transform_plane, transform_point, Transform,
};

/// Face directions for hull vs capsule. (static b3QueryFaceDirectionHullAndCapsule)
pub(crate) fn query_face_direction_hull_and_capsule(
    hull: &HullData,
    capsule: &Capsule,
    capsule_transform: Transform,
) -> FaceQuery {
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

    FaceQuery {
        separation: max_face_separation,
        // Match C's (uint8_t) cast into the int fields.
        face_index: max_face_index as u8 as i32,
        vertex_index: max_vertex_index as u8 as i32,
    }
}

/// Face directions for hull vs hull. (static b3QueryFaceDirections)
pub(crate) fn query_face_directions(
    hull_a: &HullData,
    hull_b: &HullData,
    relative_transform: Transform,
) -> FaceQuery {
    let transform = invert_transform(relative_transform);
    let planes_a = get_hull_planes(hull_a);
    let points_b = get_hull_points(hull_b);

    let mut max_face_index = -1;
    let mut max_vertex_index = -1;
    let mut max_face_separation = -f32::MAX;

    for face_index in 0..hull_a.face_count {
        let plane = transform_plane(transform, planes_a[face_index as usize]);
        let vertex_index = find_hull_support_vertex(hull_b, neg(plane.normal));
        let support = points_b[vertex_index as usize];
        let separation = plane_separation(plane, support);
        if separation > max_face_separation {
            max_face_index = face_index;
            max_vertex_index = vertex_index;
            max_face_separation = separation;
        }
    }

    FaceQuery {
        separation: max_face_separation,
        face_index: max_face_index as u8 as i32,
        vertex_index: max_vertex_index as u8 as i32,
    }
}

/// Edge directions for hull vs capsule. (static b3QueryEdgeDirectionHullAndCapsule)
pub(crate) fn query_edge_direction_hull_and_capsule(
    hull: &HullData,
    capsule: &Capsule,
    capsule_transform: Transform,
) -> EdgeQuery {
    let mut max_separation = -f32::MAX;
    let mut max_index1 = -1;
    let mut max_index2 = -1;

    let p1 = transform_point(capsule_transform, capsule.center1);
    let q1 = transform_point(capsule_transform, capsule.center2);
    let e1 = crate::math_functions::sub(q1, p1);

    let edges = get_hull_edges(hull);
    let points = get_hull_points(hull);
    let planes = get_hull_planes(hull);

    let mut index = 0;
    while index < hull.edge_count {
        let edge = &edges[index as usize];
        let twin = &edges[(index + 1) as usize];
        debug_assert!(edge.twin as i32 == index + 1 && twin.twin as i32 == index);

        let p2 = points[edge.origin as usize];
        let q2 = points[twin.origin as usize];
        let e2 = crate::math_functions::sub(q2, p2);

        let u2 = planes[edge.face as usize].normal;
        let v2 = planes[twin.face as usize].normal;

        if is_minkowski_face_isolated(u2, v2, e1) {
            let c1 = mul_sv(0.5, add(q1, p1));
            let c2 = hull.center;
            let separation = edge_edge_separation(q1, e1, c1, q2, e2, c2);
            if separation > max_separation {
                max_separation = separation;
                max_index1 = 0;
                max_index2 = index;
            }
        }

        index += 2;
    }

    EdgeQuery {
        separation: max_separation,
        index_a: max_index1 as u8 as i32,
        index_b: max_index2 as u8 as i32,
    }
}

/// Edge directions for hull vs hull. (static b3QueryEdgeDirections)
pub(crate) fn query_edge_directions(
    hull_a: &HullData,
    hull_b: &HullData,
    transform_b_to_a: Transform,
) -> EdgeQuery {
    let mut max_separation = -f32::MAX;
    let mut max_index_a = NULL_INDEX;
    let mut max_index_b = NULL_INDEX;

    let edges_a = get_hull_edges(hull_a);
    let points_a = get_hull_points(hull_a);
    let planes_a = get_hull_planes(hull_a);
    let edges_b = get_hull_edges(hull_b);
    let points_b = get_hull_points(hull_b);
    let planes_b = get_hull_planes(hull_b);

    let matrix = make_matrix_from_quat(transform_b_to_a.q);

    let mut index_b = 0;
    while index_b < hull_b.edge_count {
        let edge_b = &edges_b[index_b as usize];
        let twin_b = &edges_b[(index_b + 1) as usize];
        debug_assert!(edge_b.twin as i32 == index_b + 1 && twin_b.twin as i32 == index_b);

        let mut q_b = points_b[twin_b.origin as usize];
        let e_b = mul_mv(
            matrix,
            crate::math_functions::sub(q_b, points_b[edge_b.origin as usize]),
        );
        q_b = add(mul_mv(matrix, q_b), transform_b_to_a.p);

        let u_b = mul_mv(matrix, planes_b[edge_b.face as usize].normal);
        let v_b = mul_mv(matrix, planes_b[twin_b.face as usize].normal);

        let mut index_a = 0;
        while index_a < hull_a.edge_count {
            let edge_a = &edges_a[index_a as usize];
            let twin_a = &edges_a[(index_a + 1) as usize];
            debug_assert!(edge_a.twin as i32 == index_a + 1 && twin_a.twin as i32 == index_a);

            let q_a = points_a[twin_a.origin as usize];
            let e_a = crate::math_functions::sub(q_a, points_a[edge_a.origin as usize]);
            let u_a = planes_a[edge_a.face as usize].normal;
            let v_a = planes_a[twin_a.face as usize].normal;

            // Inlined Minkowski test with the sign flip on eB used by C.
            let cba = dot(u_b, e_a);
            let dba = dot(v_b, e_a);
            let adc = -dot(u_a, e_b);
            let bdc = -dot(v_a, e_b);
            let is_minkowski = cba * dba < 0.0 && adc * bdc < 0.0 && cba * bdc > 0.0;

            if is_minkowski {
                let center_a = hull_a.center;
                let center_b = transform_point(transform_b_to_a, hull_b.center);
                let separation = edge_edge_separation(q_a, e_a, center_a, q_b, e_b, center_b);

                if separation > max_separation {
                    max_separation = separation;
                    max_index_a = index_a;
                    max_index_b = index_b;
                }
            }

            index_a += 2;
        }

        index_b += 2;
    }

    EdgeQuery {
        separation: max_separation,
        index_a: max_index_a,
        index_b: max_index_b,
    }
}

/// Deepest (most negative) separation among manifold points.
pub(crate) fn deepest_point_separation(manifold: &LocalManifold) -> f32 {
    let mut min_separation = f32::MAX;
    for i in 0..manifold.point_count {
        min_separation = min_float(min_separation, manifold.points[i as usize].separation);
    }
    min_separation
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
