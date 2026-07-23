//! Hull-triangle face/edge contact builders from `triangle_manifold.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::clip::{clip_polygon, find_incident_face, flip_pair};
use super::triangle_hull::TriangleData;
use super::types::{
    make_feature_pair, ClipVertex, EdgeQuery, FaceQuery, FeatureOwner, LocalManifold, SatCache,
    SeparatingFeature, TriangleFeature, MAX_CLIP_POINTS,
};
use crate::constants::speculative_distance;
use crate::hull::{get_hull_edges, get_hull_faces, get_hull_planes, get_hull_points, HullData};
use crate::math_functions::{
    abs_float, add, cross, dot, line_distance, make_plane_from_normal_and_point, min_float,
    min_int, mul_sub, mul_sv, neg, normalize, plane_separation, sub, Vec3,
};
/// Clip triangle against a hull reference face. (static b3CollideHullFace)
pub(crate) fn collide_hull_face(
    manifold: &mut LocalManifold,
    point_capacity: i32,
    triangle: &TriangleData,
    hull: &HullData,
    query: FaceQuery,
    cache: &mut SatCache,
    enable_speculative: bool,
) -> f32 {
    manifold.point_count = 0;

    let hull_faces = get_hull_faces(hull);
    let hull_edges = get_hull_edges(hull);
    let hull_planes = get_hull_planes(hull);
    let hull_points = get_hull_points(hull);

    let ref_face = query.face_index;
    let ref_plane = hull_planes[ref_face as usize];

    let mut buffer1 = [ClipVertex::default(); MAX_CLIP_POINTS];
    let mut buffer2 = [ClipVertex::default(); MAX_CLIP_POINTS];

    let v1 = triangle.v1;
    let v2 = triangle.v2;
    let v3 = triangle.v3;
    buffer1[0].position = v1;
    buffer1[0].separation = plane_separation(ref_plane, v1);
    buffer1[0].pair = make_feature_pair(FeatureOwner::ShapeB, 2, FeatureOwner::ShapeB, 0);
    buffer1[1].position = v2;
    buffer1[1].separation = plane_separation(ref_plane, v2);
    buffer1[1].pair = make_feature_pair(FeatureOwner::ShapeB, 0, FeatureOwner::ShapeB, 1);
    buffer1[2].position = v3;
    buffer1[2].separation = plane_separation(ref_plane, v3);
    buffer1[2].pair = make_feature_pair(FeatureOwner::ShapeB, 1, FeatureOwner::ShapeB, 2);
    let mut point_count = 3;

    let mut input_is_buffer1 = true;

    let face = &hull_faces[ref_face as usize];
    let mut edge_index = face.edge as i32;

    loop {
        let edge = &hull_edges[edge_index as usize];
        let next_edge_index = edge.next as i32;
        let next = &hull_edges[next_edge_index as usize];
        let vertex1 = hull_points[edge.origin as usize];
        let vertex2 = hull_points[next.origin as usize];
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

        if point_count < 3 {
            // Using a stale cache
            *cache = SatCache::default();
            return query.separation;
        }

        input_is_buffer1 = !input_is_buffer1;
        edge_index = next_edge_index;
        if edge_index == face.edge as i32 {
            break;
        }
    }

    point_count = min_int(point_count, point_capacity);
    let mut min_separation = f32::MAX;
    let mut final_point_count = 0;

    let input = if input_is_buffer1 {
        &buffer1[..]
    } else {
        &buffer2[..]
    };

    for i in 0..point_count {
        let clip_point = &input[i as usize];
        min_separation = min_float(min_separation, clip_point.separation);

        if !enable_speculative && clip_point.separation > 0.0 {
            continue;
        }

        // Move point onto hull face improved culling
        let point = mul_sub(clip_point.position, clip_point.separation, ref_plane.normal);

        let pt = &mut manifold.points[final_point_count as usize];
        pt.point = point;
        pt.separation = clip_point.separation;
        pt.pair = flip_pair(clip_point.pair);

        final_point_count += 1;
    }

    let speculative_distance = if enable_speculative {
        speculative_distance()
    } else {
        0.0
    };
    if min_separation > speculative_distance {
        // This can occur with a stale SAT cache
        manifold.point_count = 0;
        *cache = SatCache::default();
        return min_separation;
    }

    manifold.point_count = final_point_count;
    manifold.normal = neg(ref_plane.normal);
    manifold.feature = TriangleFeature::HullFace;

    cache.separation = min_separation;
    cache.type_ = SeparatingFeature::FaceAxisB as u8;
    cache.index_a = query.vertex_index as u8;
    cache.index_b = query.face_index as u8;
    min_separation
}

/// Clip hull incident face against triangle. (static b3CollideTriangleFace)
pub(crate) fn collide_triangle_face(
    manifold: &mut LocalManifold,
    point_capacity: i32,
    triangle: &TriangleData,
    hull: &HullData,
    query: FaceQuery,
    cache: &mut SatCache,
    enable_speculative: bool,
) -> f32 {
    debug_assert!(manifold.point_count == 0);

    let hull_faces = get_hull_faces(hull);
    let hull_edges = get_hull_edges(hull);
    let hull_points = get_hull_points(hull);

    debug_assert!(query.face_index == 0);
    let ref_plane = triangle.plane;

    let inc_face = find_incident_face(hull, ref_plane.normal, query.vertex_index);

    // Build clip polygon from incident face
    let mut buffer1 = [ClipVertex::default(); 2 * MAX_CLIP_POINTS];
    let mut buffer2 = [ClipVertex::default(); 2 * MAX_CLIP_POINTS];
    let mut point_count = 0;
    let face = &hull_faces[inc_face as usize];
    let mut hull_edge_index = face.edge as i32;

    loop {
        let edge = &hull_edges[hull_edge_index as usize];
        let next_edge_index = edge.next as i32;
        let next = &hull_edges[next_edge_index as usize];

        let hull_point = hull_points[next.origin as usize];
        buffer1[point_count as usize].position = hull_point;
        buffer1[point_count as usize].separation = plane_separation(ref_plane, hull_point);
        buffer1[point_count as usize].pair = make_feature_pair(
            FeatureOwner::ShapeB,
            hull_edge_index,
            FeatureOwner::ShapeB,
            next_edge_index,
        );

        point_count += 1;
        hull_edge_index = next_edge_index;
        if hull_edge_index == face.edge as i32 || point_count >= 2 * MAX_CLIP_POINTS as i32 {
            break;
        }
    }

    debug_assert!(point_count >= 3);

    let mut input_is_buffer1 = true;

    let triangle_points = [triangle.v1, triangle.v2, triangle.v3];
    let triangle_edges = [triangle.e1, triangle.e2, triangle.e3];

    for i in 0..3 {
        if point_count <= 0 {
            break;
        }

        let mut side_normal = cross(triangle_edges[i], ref_plane.normal);
        side_normal = normalize(side_normal);
        let clip_plane = make_plane_from_normal_and_point(side_normal, triangle_points[i]);

        point_count = if input_is_buffer1 {
            clip_polygon(
                &mut buffer2,
                &buffer1,
                point_count,
                clip_plane,
                i as i32,
                ref_plane,
            )
        } else {
            clip_polygon(
                &mut buffer1,
                &buffer2,
                point_count,
                clip_plane,
                i as i32,
                ref_plane,
            )
        };
        debug_assert!(point_count <= 2 * MAX_CLIP_POINTS as i32);

        input_is_buffer1 = !input_is_buffer1;
    }

    if point_count == 0 {
        // Triangle face clipped away. Invalidate cache.
        *cache = SatCache::default();
        return f32::MAX;
    }

    point_count = min_int(point_count, point_capacity);

    let mut min_separation = f32::MAX;

    let input = if input_is_buffer1 {
        &buffer1[..]
    } else {
        &buffer2[..]
    };

    let mut final_point_count = 0;
    for i in 0..point_count {
        let clip_point = &input[i as usize];
        min_separation = min_float(min_separation, clip_point.separation);

        if !enable_speculative && clip_point.separation > 0.0 {
            continue;
        }

        // Move point onto triangle surface for improved culling — C leaves position as-is
        let point = clip_point.position;

        let pt = &mut manifold.points[final_point_count as usize];
        pt.point = point;
        pt.separation = clip_point.separation;
        pt.pair = clip_point.pair;

        final_point_count += 1;
    }

    let speculative_distance = if enable_speculative {
        speculative_distance()
    } else {
        0.0
    };
    if min_separation >= speculative_distance {
        // This can happen if the objects move apart while re-using a cached axis
        *cache = SatCache::default();
        return min_separation;
    }

    manifold.point_count = final_point_count;
    manifold.normal = ref_plane.normal;
    manifold.feature = TriangleFeature::TriangleFace;

    cache.separation = min_separation;
    cache.type_ = SeparatingFeature::FaceAxisA as u8;
    cache.index_a = query.face_index as u8;
    cache.index_b = query.vertex_index as u8;
    min_separation
}

/// Build edge-edge contact between hull and triangle. (static b3CollideHullAndTriangleEdges)
pub(crate) fn collide_hull_and_triangle_edges(
    manifold: &mut LocalManifold,
    capacity: i32,
    triangle_point: Vec3,
    triangle_edge: Vec3,
    triangle_center: Vec3,
    hull: &HullData,
    query: EdgeQuery,
    cache: &mut SatCache,
) {
    debug_assert!(query.index_a < 3);

    let c_a = triangle_center;
    let p_a = triangle_point;
    let e_a = triangle_edge;

    let edges_b = get_hull_edges(hull);
    let points_b = get_hull_points(hull);
    let edge_b = &edges_b[query.index_b as usize];
    let twin_b = &edges_b[edge_b.twin as usize];
    let p_b = points_b[edge_b.origin as usize];
    let q_b = points_b[twin_b.origin as usize];
    let e_b = sub(q_b, p_b);

    let mut normal = cross(e_a, e_b);
    normal = normalize(normal);

    // Ensure normal points outward from triangle center
    let outward_a = dot(normal, sub(p_a, c_a));
    // Ensure normal points towards hull center
    let outward_b = dot(normal, sub(hull.center, p_b));

    // Use the largest magnitude. The triangle outward value may be unreliable at some angles.
    if abs_float(outward_a) > abs_float(outward_b) {
        if outward_a < 0.0 {
            normal = neg(normal);
        }
    } else if outward_b < 0.0 {
        normal = neg(normal);
    }

    let result = line_distance(p_a, e_a, p_b, e_b);

    if capacity == 0
        || result.fraction1 < 0.0
        || 1.0 < result.fraction1
        || result.fraction2 < 0.0
        || 1.0 < result.fraction2
    {
        // Invalid edge pair, no points generated
        debug_assert!(manifold.point_count == 0);
        *cache = SatCache::default();
        return;
    }

    // This can slide off the end from caching
    let separation = dot(normal, sub(result.point2, result.point1));

    let point = mul_sv(0.5, add(result.point1, result.point2));

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

    manifold.normal = normal;
    manifold.point_count = 1;

    let edges_features = [
        TriangleFeature::Edge1,
        TriangleFeature::Edge2,
        TriangleFeature::Edge3,
    ];
    manifold.feature = edges_features[query.index_a as usize];
}

/// Minkowski-face test for triangle edge vs hull edge.
/// (static b3IsTriangleMinkowskiFace)
#[inline]
pub(crate) fn is_triangle_minkowski_face(
    tri_normal: Vec3,
    tri_edge: Vec3,
    hull_normal1: Vec3,
    hull_normal2: Vec3,
    hull_edge: Vec3,
) -> bool {
    let cab = dot(hull_normal1, tri_edge);
    let dab = dot(hull_normal2, tri_edge);
    let bcd = dot(tri_normal, hull_edge);
    cab * dab < 0.0 && cab * bcd > 0.0
}
