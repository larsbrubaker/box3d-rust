//! Hull-vs-hull contact manifold from `convex_manifold.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::clip::{
    build_polygon, clip_polygon, edge_edge_separation, find_incident_face, flip_pair,
    is_minkowski_face,
};
use super::sat::{query_edge_directions, query_face_directions, reduce_manifold_points};
use super::types::{
    make_feature_pair, ClipVertex, EdgeQuery, FaceQuery, FeatureOwner, LocalManifold,
    LocalManifoldPoint, SatCache, SeparatingFeature, MAX_CLIP_POINTS,
};
use crate::constants::{linear_slop, speculative_distance};
use crate::core::NULL_INDEX;
use crate::hull::{
    find_hull_support_vertex, get_hull_edges, get_hull_faces, get_hull_planes, get_hull_points,
    HullData,
};
use crate::math_functions::{
    abs_float, add, cross, dot, inv_rotate_vector, inv_transform_point, invert_transform,
    is_within_segments, line_distance, make_matrix_from_quat, make_plane_from_normal_and_point,
    min_float, min_int, mul_mv, mul_sub, mul_sv, neg, normalize, plane_separation, rotate_vector,
    sub, transform_point, Transform,
};

/// Build face-A contact by clipping the incident face of B. (static b3BuildFaceAContact)
fn build_face_a_contact(
    manifold: &mut LocalManifold,
    capacity: i32,
    hull_a: &HullData,
    hull_b: &HullData,
    transform_b_to_a: Transform,
    query: FaceQuery,
    cache: &mut SatCache,
) -> bool {
    let faces_a = get_hull_faces(hull_a);
    let edges_a = get_hull_edges(hull_a);
    let planes_a = get_hull_planes(hull_a);
    let points_a = get_hull_points(hull_a);

    let ref_face = query.face_index;
    let ref_plane = planes_a[ref_face as usize];

    let ref_normal_in_b = inv_rotate_vector(transform_b_to_a.q, ref_plane.normal);
    let inc_face = find_incident_face(hull_b, ref_normal_in_b, query.vertex_index);

    let mut buffer1 = [ClipVertex::default(); MAX_CLIP_POINTS];
    let mut buffer2 = [ClipVertex::default(); MAX_CLIP_POINTS];
    let mut point_count =
        build_polygon(&mut buffer1, transform_b_to_a, hull_b, inc_face, ref_plane);

    // Clip incident face against side planes of reference face.
    // C swaps input/output pointers; we track which buffer is current with a bool.
    let mut input_is_buffer1 = true;

    let face = &faces_a[ref_face as usize];
    let mut edge_index = face.edge as i32;

    loop {
        let edge = &edges_a[edge_index as usize];
        let next_edge_index = edge.next as i32;
        let next = &edges_a[next_edge_index as usize];
        let vertex1 = points_a[edge.origin as usize];
        let vertex2 = points_a[next.origin as usize];
        let tangent = normalize(sub(vertex2, vertex1));
        let binormal = cross(tangent, ref_plane.normal);
        let clip_plane = make_plane_from_normal_and_point(binormal, vertex1);

        point_count = if input_is_buffer1 {
            clip_polygon(
                &mut buffer2,
                &buffer1,
                point_count,
                clip_plane,
                edge_index,
                ref_plane,
            )
        } else {
            clip_polygon(
                &mut buffer1,
                &buffer2,
                point_count,
                clip_plane,
                edge_index,
                ref_plane,
            )
        };
        debug_assert!(point_count <= MAX_CLIP_POINTS as i32);

        input_is_buffer1 = !input_is_buffer1;

        if point_count < 3 {
            *cache = SatCache::default();
            return false;
        }

        edge_index = next_edge_index;
        if edge_index == face.edge as i32 {
            break;
        }
    }

    point_count = min_int(point_count, MAX_CLIP_POINTS as i32);

    let input = if input_is_buffer1 {
        &buffer1[..]
    } else {
        &buffer2[..]
    };

    let mut points = [LocalManifoldPoint::default(); MAX_CLIP_POINTS];
    let mut min_separation = f32::MAX;

    manifold.normal = ref_plane.normal;

    for i in 0..point_count {
        let clip_point = &input[i as usize];
        let pt = &mut points[i as usize];
        *pt = LocalManifoldPoint::default();

        let point = mul_sub(
            clip_point.position,
            0.5 * clip_point.separation,
            ref_plane.normal,
        );

        pt.point = point;
        pt.separation = clip_point.separation;
        pt.pair = clip_point.pair;

        min_separation = min_float(min_separation, clip_point.separation);
    }

    if min_separation >= speculative_distance() {
        *cache = SatCache::default();
        return false;
    }

    reduce_manifold_points(manifold, capacity, &mut points, point_count);

    cache.separation = min_separation;
    cache.type_ = SeparatingFeature::FaceAxisA as u8;
    cache.index_a = query.face_index as u8;
    cache.index_b = query.vertex_index as u8;

    true
}

/// Build face-B contact (swap roles, then transform into frame A). (static b3BuildFaceBContact)
fn build_face_b_contact(
    manifold: &mut LocalManifold,
    capacity: i32,
    hull_a: &HullData,
    hull_b: &HullData,
    transform_b_to_a: Transform,
    query: FaceQuery,
    cache: &mut SatCache,
) -> bool {
    let transform_a_to_b = invert_transform(transform_b_to_a);
    let touching = build_face_a_contact(
        manifold,
        capacity,
        hull_b,
        hull_a,
        transform_a_to_b,
        query,
        cache,
    );
    if !touching {
        return false;
    }

    let matrix = make_matrix_from_quat(transform_b_to_a.q);

    manifold.normal = neg(mul_mv(matrix, manifold.normal));
    cache.type_ = SeparatingFeature::FaceAxisB as u8;
    cache.index_a = query.vertex_index as u8;
    cache.index_b = query.face_index as u8;

    for i in 0..manifold.point_count {
        let pt = &mut manifold.points[i as usize];
        pt.point = add(mul_mv(matrix, pt.point), transform_b_to_a.p);
        pt.pair = flip_pair(pt.pair);
    }

    true
}

/// Build a single edge-edge contact. (static b3BuildEdgeContact)
fn build_edge_contact(
    manifold: &mut LocalManifold,
    hull_a: &HullData,
    hull_b: &HullData,
    transform_b_to_a: Transform,
    query: EdgeQuery,
    cache: &mut SatCache,
) -> bool {
    let edges_a = get_hull_edges(hull_a);
    let points_a = get_hull_points(hull_a);
    let edges_b = get_hull_edges(hull_b);
    let points_b = get_hull_points(hull_b);

    let edge_a = &edges_a[query.index_a as usize];
    let twin_a = &edges_a[edge_a.twin as usize];
    let center_a = hull_a.center;
    let p_a = points_a[edge_a.origin as usize];
    let q_a = points_a[twin_a.origin as usize];
    let e_a = sub(q_a, p_a);

    let edge_b = &edges_b[query.index_b as usize];
    let twin_b = &edges_b[edge_b.twin as usize];
    let p_b = transform_point(transform_b_to_a, points_b[edge_b.origin as usize]);
    let q_b = transform_point(transform_b_to_a, points_b[twin_b.origin as usize]);
    let e_b = sub(q_b, p_b);

    let mut normal = normalize(cross(e_a, e_b));

    if dot(normal, sub(p_a, center_a)) < 0.0 {
        normal = neg(normal);
    }

    let result = line_distance(p_a, e_a, p_b, e_b);

    if !is_within_segments(&result) {
        *cache = SatCache::default();
        return false;
    }

    let separation = dot(normal, sub(result.point2, result.point1));
    let point = mul_sv(0.5, add(result.point1, result.point2));

    manifold.normal = normal;
    manifold.point_count = 1;

    let pt = &mut manifold.points[0];
    pt.point = point;
    pt.separation = separation;
    pt.pair = make_feature_pair(
        FeatureOwner::ShapeA,
        query.index_a,
        FeatureOwner::ShapeB,
        query.index_b,
    );

    cache.separation = separation;
    cache.type_ = SeparatingFeature::EdgePairAxis as u8;
    cache.index_a = query.index_a as u8;
    cache.index_b = query.index_b as u8;

    true
}

/// Collide two convex hulls. (b3CollideHulls)
pub fn collide_hulls(
    manifold: &mut LocalManifold,
    capacity: i32,
    hull_a: &HullData,
    hull_b: &HullData,
    transform_b_to_a: Transform,
    cache: &mut SatCache,
) {
    manifold.point_count = 0;

    if capacity < 4 {
        return;
    }

    let speculative = speculative_distance();
    let slop = linear_slop();
    let edges_a = get_hull_edges(hull_a);
    let planes_a = get_hull_planes(hull_a);
    let points_a = get_hull_points(hull_a);
    let edges_b = get_hull_edges(hull_b);
    let planes_b = get_hull_planes(hull_b);
    let points_b = get_hull_points(hull_b);

    match cache.type_ {
        t if t == SeparatingFeature::InvalidAxis as u8 => {
            *cache = SatCache::default();
        }
        t if t == SeparatingFeature::FaceAxisA as u8 => {
            debug_assert!((cache.index_a as i32) < hull_a.face_count);

            let plane = planes_a[cache.index_a as usize];
            let search_direction_in_b = neg(inv_rotate_vector(transform_b_to_a.q, plane.normal));
            let vertex_index = find_hull_support_vertex(hull_b, search_direction_in_b);
            let support = transform_point(transform_b_to_a, points_b[vertex_index as usize]);
            let separation = plane_separation(plane, support);

            if separation >= speculative {
                return;
            }

            let face_query = FaceQuery {
                separation: 0.0,
                face_index: cache.index_a as i32,
                vertex_index,
            };

            let mut local_cache = SatCache::default();
            let touching = build_face_a_contact(
                manifold,
                capacity,
                hull_a,
                hull_b,
                transform_b_to_a,
                face_query,
                &mut local_cache,
            );
            if touching && abs_float(cache.separation - local_cache.separation) < slop {
                return;
            }
        }
        t if t == SeparatingFeature::FaceAxisB as u8 => {
            debug_assert!((cache.index_b as i32) < hull_b.face_count);

            let plane = planes_b[cache.index_b as usize];
            let search_direction_in_a = neg(rotate_vector(transform_b_to_a.q, plane.normal));
            let vertex_index = find_hull_support_vertex(hull_a, search_direction_in_a);
            let support = inv_transform_point(transform_b_to_a, points_a[vertex_index as usize]);
            let separation = plane_separation(plane, support);

            if separation >= speculative {
                return;
            }

            let face_query = FaceQuery {
                separation: 0.0,
                face_index: cache.index_b as i32,
                vertex_index,
            };

            let mut local_cache = SatCache::default();
            let touching = build_face_b_contact(
                manifold,
                capacity,
                hull_a,
                hull_b,
                transform_b_to_a,
                face_query,
                &mut local_cache,
            );
            if touching && abs_float(cache.separation - local_cache.separation) < slop {
                return;
            }
        }
        t if t == SeparatingFeature::EdgePairAxis as u8 => {
            let index1 = cache.index_a as i32;
            let edge1 = &edges_a[index1 as usize];
            let twin1 = &edges_a[(index1 + 1) as usize];
            debug_assert!(edge1.twin as i32 == index1 + 1 && twin1.twin as i32 == index1);

            let p1 = points_a[edge1.origin as usize];
            let q1 = points_a[twin1.origin as usize];
            let e1 = sub(q1, p1);
            let u1 = planes_a[edge1.face as usize].normal;
            let v1 = planes_a[twin1.face as usize].normal;

            let index2 = cache.index_b as i32;
            let edge2 = &edges_b[index2 as usize];
            let twin2 = &edges_b[(index2 + 1) as usize];
            debug_assert!(edge2.twin as i32 == index2 + 1 && twin2.twin as i32 == index2);

            let p2 = transform_point(transform_b_to_a, points_b[edge2.origin as usize]);
            let q2 = transform_point(transform_b_to_a, points_b[twin2.origin as usize]);
            let e2 = sub(q2, p2);
            let u2 = rotate_vector(transform_b_to_a.q, planes_b[edge2.face as usize].normal);
            let v2 = rotate_vector(transform_b_to_a.q, planes_b[twin2.face as usize].normal);

            let is_minkowski = is_minkowski_face(u1, v1, e1, neg(u2), neg(v2), e2);
            if is_minkowski {
                let c1 = hull_a.center;
                let c2 = transform_point(transform_b_to_a, hull_b.center);
                let separation = edge_edge_separation(p1, e1, c1, p2, e2, c2);
                if separation > speculative {
                    return;
                }

                let edge_query = EdgeQuery {
                    index_a: cache.index_a as i32,
                    index_b: cache.index_b as i32,
                    separation: 0.0,
                };

                let mut local_cache = SatCache::default();
                let touching = build_edge_contact(
                    manifold,
                    hull_a,
                    hull_b,
                    transform_b_to_a,
                    edge_query,
                    &mut local_cache,
                );
                if touching && abs_float(cache.separation - local_cache.separation) < slop {
                    return;
                }
            }
        }
        t if t == SeparatingFeature::ManualFaceAxisA as u8 => {
            let face_query_a = query_face_directions(hull_a, hull_b, transform_b_to_a);
            build_face_a_contact(
                manifold,
                capacity,
                hull_a,
                hull_b,
                transform_b_to_a,
                face_query_a,
                cache,
            );
            return;
        }
        t if t == SeparatingFeature::ManualFaceAxisB as u8 => {
            let face_query_b =
                query_face_directions(hull_b, hull_a, invert_transform(transform_b_to_a));
            build_face_b_contact(
                manifold,
                capacity,
                hull_a,
                hull_b,
                transform_b_to_a,
                face_query_b,
                cache,
            );
            return;
        }
        t if t == SeparatingFeature::ManualEdgePairAxis as u8 => {
            let edge_query = query_edge_directions(hull_a, hull_b, transform_b_to_a);
            if edge_query.index_a != NULL_INDEX {
                build_edge_contact(
                    manifold,
                    hull_a,
                    hull_b,
                    transform_b_to_a,
                    edge_query,
                    cache,
                );
            }
            return;
        }
        _ => {
            debug_assert!(false, "unexpected SAT cache type");
        }
    }

    manifold.point_count = 0;
    *cache = SatCache::default();

    let face_query_a = query_face_directions(hull_a, hull_b, transform_b_to_a);
    if face_query_a.separation > speculative {
        debug_assert!(face_query_a.face_index < hull_a.face_count);
        debug_assert!(face_query_a.vertex_index < hull_b.vertex_count);

        cache.separation = face_query_a.separation;
        cache.type_ = SeparatingFeature::FaceAxisA as u8;
        cache.index_a = face_query_a.face_index as u8;
        cache.index_b = face_query_a.vertex_index as u8;
        return;
    }

    let face_query_b = query_face_directions(hull_b, hull_a, invert_transform(transform_b_to_a));
    if face_query_b.separation > speculative {
        debug_assert!(face_query_b.face_index < hull_b.face_count);
        debug_assert!(face_query_b.vertex_index < hull_a.vertex_count);

        cache.separation = face_query_b.separation;
        cache.type_ = SeparatingFeature::FaceAxisB as u8;
        cache.index_a = face_query_b.vertex_index as u8;
        cache.index_b = face_query_b.face_index as u8;
        return;
    }

    let edge_query = query_edge_directions(hull_a, hull_b, transform_b_to_a);
    if edge_query.separation > speculative {
        cache.separation = edge_query.separation;
        cache.type_ = SeparatingFeature::EdgePairAxis as u8;
        cache.index_a = edge_query.index_a as u8;
        cache.index_b = edge_query.index_b as u8;
        return;
    }

    let face_separation_a = face_query_a.separation;
    let face_separation_b = face_query_b.separation;
    debug_assert!(face_separation_a <= speculative && face_separation_b <= speculative);

    if face_separation_b > face_separation_a + 0.5 * slop {
        build_face_b_contact(
            manifold,
            capacity,
            hull_a,
            hull_b,
            transform_b_to_a,
            face_query_b,
            cache,
        );
    } else {
        build_face_a_contact(
            manifold,
            capacity,
            hull_a,
            hull_b,
            transform_b_to_a,
            face_query_a,
            cache,
        );
    }

    if edge_query.index_a == NULL_INDEX {
        return;
    }

    let clipped_face_separation = cache.separation;
    debug_assert!(edge_query.separation <= speculative);

    const K_REL_EDGE_TOLERANCE: f32 = 0.90;
    let k_abs_tolerance = 0.5 * slop;

    if manifold.point_count == 0
        || edge_query.separation > K_REL_EDGE_TOLERANCE * clipped_face_separation + k_abs_tolerance
    {
        let mut edge_manifold = LocalManifold::default();
        build_edge_contact(
            &mut edge_manifold,
            hull_a,
            hull_b,
            transform_b_to_a,
            edge_query,
            cache,
        );

        if edge_manifold.point_count == 1 {
            let edge_point = edge_manifold.points[0];
            *manifold = edge_manifold;
            manifold.points[0] = edge_point;
        }
    }
}
