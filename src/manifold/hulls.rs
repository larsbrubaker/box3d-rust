//! Hull-vs-hull contact manifold from `convex_manifold.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::clip::{build_polygon, clip_polygon, find_incident_face, flip_pair};
use super::sat::reduce_manifold_points;
use super::separating_axis::compute_separating_axis;
use super::types::{
    make_feature_pair, ClipVertex, FeatureOwner, LocalManifold, LocalManifoldPoint, SatCache,
    SeparatingAxis, SeparatingFeature, MAX_CLIP_POINTS,
};
use crate::constants::{linear_slop, speculative_distance};
use crate::core::NULL_INDEX;
use crate::hull::{
    find_hull_support_vertex, get_hull_edges, get_hull_faces, get_hull_planes, get_hull_points,
    HullData,
};
use crate::math_functions::{
    abs_float, add, cross, dot, inv_rotate_vector, inv_transform_point, invert_transform,
    is_within_segments, length_squared, lerp, line_distance, make_matrix_from_quat,
    make_plane_from_normal_and_point, max_float, min_float, min_int, mul_mv, mul_sub, mul_sv, neg,
    normalize, plane_separation, rotate_vector, sub, transform_point, Transform,
};

/// Build face-A contact by clipping the incident face of B. (static b3BuildFaceAContact)
fn build_face_a_contact(
    manifold: &mut LocalManifold,
    capacity: i32,
    hull_a: &HullData,
    hull_b: &HullData,
    transform_b_to_a: Transform,
    query: SeparatingAxis,
    cache: &mut SatCache,
) -> bool {
    debug_assert!(query.type_ == SeparatingFeature::FaceAxisA);
    debug_assert!(0 <= query.index_a && query.index_a < hull_a.face_count);
    debug_assert!(0 <= query.index_b && query.index_b < hull_b.vertex_count);

    let faces_a = get_hull_faces(hull_a);
    let edges_a = get_hull_edges(hull_a);
    let planes_a = get_hull_planes(hull_a);
    let points_a = get_hull_points(hull_a);

    let ref_face = query.index_a;
    let ref_plane = planes_a[ref_face as usize];

    let ref_normal_in_b = inv_rotate_vector(transform_b_to_a.q, ref_plane.normal);
    let inc_face = find_incident_face(hull_b, ref_normal_in_b, query.index_b);

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
    cache.index_a = query.index_a as u8;
    cache.index_b = query.index_b as u8;

    true
}

/// Build face-B contact (swap roles, then transform into frame A). (static b3BuildFaceBContact)
fn build_face_b_contact(
    manifold: &mut LocalManifold,
    capacity: i32,
    hull_a: &HullData,
    hull_b: &HullData,
    transform_b_to_a: Transform,
    query: SeparatingAxis,
    cache: &mut SatCache,
) -> bool {
    debug_assert!(query.type_ == SeparatingFeature::FaceAxisB);

    let transform_a_to_b = invert_transform(transform_b_to_a);
    let flipped_query = SeparatingAxis {
        normal: neg(query.normal),
        separation: query.separation,
        index_a: query.index_b,
        index_b: query.index_a,
        type_: SeparatingFeature::FaceAxisA,
    };

    let touching = build_face_a_contact(
        manifold,
        capacity,
        hull_b,
        hull_a,
        transform_a_to_b,
        flipped_query,
        cache,
    );
    if !touching {
        *cache = SatCache::default();
        return false;
    }

    // Results are in frame B, need to transform them into frame A
    let matrix = make_matrix_from_quat(transform_b_to_a.q);

    // Transform and flip normal so it points from A to B, even though B has the reference face.
    manifold.normal = neg(mul_mv(matrix, manifold.normal));

    // Transform points from frame B to frame A. Also flip the pairs to ensure correct matches.
    for i in 0..manifold.point_count {
        let pt = &mut manifold.points[i as usize];
        pt.point = add(mul_mv(matrix, pt.point), transform_b_to_a.p);
        pt.pair = flip_pair(pt.pair);
    }

    cache.type_ = SeparatingFeature::FaceAxisB as u8;
    cache.index_a = query.index_a as u8;
    cache.index_b = query.index_b as u8;

    true
}

/// Build a single edge-edge contact. (static b3BuildEdgeContact)
fn build_edge_contact(
    manifold: &mut LocalManifold,
    hull_a: &HullData,
    hull_b: &HullData,
    transform_b_to_a: Transform,
    query: SeparatingAxis,
    cache: &mut SatCache,
) -> bool {
    debug_assert!(query.type_ == SeparatingFeature::EdgePairAxis);
    debug_assert!(0 <= query.index_a && query.index_a < hull_a.edge_count);
    debug_assert!(0 <= query.index_b && query.index_b < hull_b.edge_count);

    let edges_a = get_hull_edges(hull_a);
    let points_a = get_hull_points(hull_a);
    let edges_b = get_hull_edges(hull_b);
    let points_b = get_hull_points(hull_b);

    let edge_a = &edges_a[query.index_a as usize];
    let twin_a = &edges_a[edge_a.twin as usize];
    let p_a = points_a[edge_a.origin as usize];
    let q_a = points_a[twin_a.origin as usize];
    let e_a = sub(q_a, p_a);

    let edge_b = &edges_b[query.index_b as usize];
    let twin_b = &edges_b[edge_b.twin as usize];
    let p_b = transform_point(transform_b_to_a, points_b[edge_b.origin as usize]);
    let q_b = transform_point(transform_b_to_a, points_b[twin_b.origin as usize]);
    let e_b = sub(q_b, p_b);

    let normal = query.normal;
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

    // Work in shapeA coordinates
    let speculative = speculative_distance();
    let slop = linear_slop();
    let edges_a = get_hull_edges(hull_a);
    let planes_a = get_hull_planes(hull_a);
    let points_a = get_hull_points(hull_a);
    let edges_b = get_hull_edges(hull_b);
    let planes_b = get_hull_planes(hull_b);
    let points_b = get_hull_points(hull_b);

    cache.hit = 0;

    // Attempt to use the cache to speed up collision
    match cache.type_ {
        t if t == SeparatingFeature::InvalidAxis as u8 => {}
        t if t == SeparatingFeature::FaceAxisA as u8 => {
            debug_assert!((cache.index_a as i32) < hull_a.face_count);

            let plane = planes_a[cache.index_a as usize];
            let search_direction_in_b = neg(inv_rotate_vector(transform_b_to_a.q, plane.normal));
            let vertex_index = find_hull_support_vertex(hull_b, search_direction_in_b);
            let support = transform_point(transform_b_to_a, points_b[vertex_index as usize]);
            let separation = plane_separation(plane, support);

            if separation >= speculative {
                // Cache hit, shapes are separated
                cache.hit = 1;
                return;
            }

            let face_query = SeparatingAxis {
                normal: plane.normal,
                separation: 0.0,
                index_a: cache.index_a as i32,
                index_b: vertex_index,
                type_: SeparatingFeature::FaceAxisA,
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
                // Cache hit, contact points generated
                cache.hit = 1;
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
                // Cache hit, shapes are separated
                cache.hit = 1;
                return;
            }

            let face_query = SeparatingAxis {
                normal: neg(plane.normal),
                separation: 0.0,
                index_a: vertex_index,
                index_b: cache.index_b as i32,
                type_: SeparatingFeature::FaceAxisB,
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
                // Cache hit, contact points generated
                cache.hit = 1;
                return;
            }
        }
        t if t == SeparatingFeature::EdgePairAxis as u8 => {
            let index_a = cache.index_a as i32;
            let edge1 = &edges_a[index_a as usize];
            let twin1 = &edges_a[(index_a + 1) as usize];
            debug_assert!(edge1.twin as i32 == index_a + 1 && twin1.twin as i32 == index_a);

            let p_a = points_a[edge1.origin as usize];
            let q_a = points_a[twin1.origin as usize];
            let e_a = sub(q_a, p_a);

            let u_a = planes_a[edge1.face as usize].normal;
            let v_a = planes_a[twin1.face as usize].normal;

            let index_b = cache.index_b as i32;
            let edge2 = &edges_b[index_b as usize];
            let twin2 = &edges_b[(index_b + 1) as usize];
            debug_assert!(edge2.twin as i32 == index_b + 1 && twin2.twin as i32 == index_b);

            let p_b = transform_point(transform_b_to_a, points_b[edge2.origin as usize]);
            let q_b = transform_point(transform_b_to_a, points_b[twin2.origin as usize]);
            let e_b = sub(q_b, p_b);

            let u_b = rotate_vector(transform_b_to_a.q, planes_b[edge2.face as usize].normal);
            let v_b = rotate_vector(transform_b_to_a.q, planes_b[twin2.face as usize].normal);

            // flipping the signs of u2 and v2
            // cross(v2, u2) == cross(-v2, -u2)
            // so we still use -e2
            // but we can also use e1 = cross(u1, v1) and e2 = cross(u2, v2)
            let cba = dot(u_b, e_a);
            let dba = dot(v_b, e_a);
            let adc = -dot(u_a, e_b);
            let bdc = -dot(v_a, e_b);

            if cba * dba < 0.0 && adc * bdc < 0.0 && cba * bdc > 0.0 {
                // Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
                let squared_tolerance = 0.005 * 0.005;
                if max_float(cba * cba, dba * dba) >= squared_tolerance * length_squared(e_a) {
                    // Transform reference center of the first hull into local space of the second hull
                    let t = cba / (cba - dba);
                    let mut axis = lerp(u_b, v_b, t);
                    debug_assert!(length_squared(axis) > 1000.0 * f32::MIN_POSITIVE);
                    axis = normalize(axis);
                    let separation = dot(axis, sub(q_a, q_b));

                    if separation > speculative {
                        // Cache hit, shapes are separated
                        cache.hit = 1;
                        return;
                    }

                    // Try to rebuild contact from last features
                    let edge_query = SeparatingAxis {
                        normal: neg(axis),
                        separation: 0.0,
                        index_a: cache.index_a as i32,
                        index_b: cache.index_b as i32,
                        type_: SeparatingFeature::EdgePairAxis,
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
                    // This separation tolerance may have a big impact on performance in some benchmarks.
                    if touching && abs_float(cache.separation - local_cache.separation) < slop {
                        // Cache hit, contact point generated
                        cache.hit = 1;
                        return;
                    }
                }
            }
        }
        // This case is for testing
        t if t == SeparatingFeature::ManualFaceAxisA as u8 => {
            let axis_query = compute_separating_axis(hull_a, hull_b, transform_b_to_a, false);
            let face_query = axis_query.face_a;
            build_face_a_contact(
                manifold,
                capacity,
                hull_a,
                hull_b,
                transform_b_to_a,
                face_query,
                cache,
            );
            return;
        }
        // This case is for testing
        t if t == SeparatingFeature::ManualFaceAxisB as u8 => {
            let axis_query = compute_separating_axis(hull_a, hull_b, transform_b_to_a, false);
            let face_query = axis_query.face_b;
            build_face_b_contact(
                manifold,
                capacity,
                hull_a,
                hull_b,
                transform_b_to_a,
                face_query,
                cache,
            );
            return;
        }
        // This case is for testing
        t if t == SeparatingFeature::ManualEdgePairAxis as u8 => {
            let axis_query = compute_separating_axis(hull_a, hull_b, transform_b_to_a, false);
            let edge_query = axis_query.edge;

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

    let axis_query = compute_separating_axis(hull_a, hull_b, transform_b_to_a, true);

    if axis_query.separated_feature != SeparatingFeature::InvalidAxis {
        // We found a separating axis
        cache.type_ = axis_query.separated_feature as u8;

        if axis_query.separated_feature == SeparatingFeature::FaceAxisA {
            debug_assert!(axis_query.face_a.separation > speculative);
            cache.separation = axis_query.face_a.separation;
            cache.index_a = axis_query.face_a.index_a as u8;
            cache.index_b = axis_query.face_a.index_b as u8;
        } else if axis_query.separated_feature == SeparatingFeature::FaceAxisB {
            debug_assert!(axis_query.face_b.separation > speculative);
            cache.separation = axis_query.face_b.separation;
            cache.index_a = axis_query.face_b.index_a as u8;
            cache.index_b = axis_query.face_b.index_b as u8;
        } else {
            debug_assert!(axis_query.separated_feature == SeparatingFeature::EdgePairAxis);
            debug_assert!(axis_query.edge.separation > speculative);
            cache.separation = axis_query.edge.separation;
            cache.index_a = axis_query.edge.index_a as u8;
            cache.index_b = axis_query.edge.index_b as u8;
        }
        return;
    }

    debug_assert!(
        axis_query.face_a.separation <= speculative
            || axis_query.face_b.separation <= speculative
            || axis_query.edge.separation <= speculative
    );

    if axis_query.face_a.separation > axis_query.face_b.separation {
        let face_query = axis_query.face_a;
        debug_assert!(0 <= face_query.index_a && face_query.index_a < hull_a.face_count);
        debug_assert!(0 <= face_query.index_b && face_query.index_b < hull_b.vertex_count);

        // Face contact A
        build_face_a_contact(
            manifold,
            capacity,
            hull_a,
            hull_b,
            transform_b_to_a,
            face_query,
            cache,
        );

        debug_assert!((cache.index_a as i32) < hull_a.face_count);
        debug_assert!((cache.index_b as i32) < hull_b.vertex_count);
    } else {
        let face_query = axis_query.face_b;
        debug_assert!(0 <= face_query.index_a && face_query.index_a < hull_a.vertex_count);
        debug_assert!(0 <= face_query.index_b && face_query.index_b < hull_b.face_count);

        // Face contact B
        build_face_b_contact(
            manifold,
            capacity,
            hull_a,
            hull_b,
            transform_b_to_a,
            face_query,
            cache,
        );

        debug_assert!((cache.index_a as i32) < hull_a.vertex_count);
        debug_assert!((cache.index_b as i32) < hull_b.face_count);
    }

    let edge_query = axis_query.edge;

    if edge_query.index_a == NULL_INDEX {
        // There are no valid edge pairs (all edges parallel)
        return;
    }

    let clip_separation = cache.separation;
    let edge_tol = slop;

    // Face contact can be empty if it does not realize the axis of minimum penetration.
    // Create edge contact if face contact fails or edge contact is significantly better!
    if manifold.point_count == 0 || edge_query.separation > clip_separation + edge_tol {
        debug_assert!(0 <= edge_query.index_a && edge_query.index_a < hull_a.edge_count);
        debug_assert!(0 <= edge_query.index_b && edge_query.index_b < hull_b.edge_count);

        // Edge contact
        let mut edge_manifold = LocalManifold::default();

        let mut edge_cache = SatCache::default();
        build_edge_contact(
            &mut edge_manifold,
            hull_a,
            hull_b,
            transform_b_to_a,
            edge_query,
            &mut edge_cache,
        );

        // It is possible with speculation to have vertex-vertex collision that is missed by SAT,
        // so edge contact yields no points. In that case perhaps the face contact has some points.
        if edge_manifold.point_count == 1 {
            // Copy edge manifold out, being careful to preserve manifold point buffer.
            let edge_point = edge_manifold.points[0];
            *manifold = edge_manifold;
            manifold.points[0] = edge_point;
            *cache = edge_cache;
        }
    }
}
