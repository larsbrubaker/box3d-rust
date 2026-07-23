//! Shared clip / edge helpers from `manifold.c` and `convex_manifold.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{make_feature_pair, ClipVertex, FeatureOwner, FeaturePair};
use crate::hull::{
    get_hull_edges, get_hull_faces, get_hull_planes, get_hull_points, get_hull_vertices, HullData,
};
use crate::math_functions::{
    abs_float, add, cross, dot, make_plane_from_normal_and_point, mul_add, mul_sv, normalize,
    plane_separation, sub, Plane, Vec3,
};

/// Find the incident face given a reference normal and closest vertex.
/// (b3FindIncidentFace)
pub fn find_incident_face(hull: &HullData, ref_normal: Vec3, vertex_index: i32) -> i32 {
    debug_assert!(0 <= vertex_index && vertex_index < hull.vertex_count);

    let vertices = get_hull_vertices(hull);
    let edges = get_hull_edges(hull);
    let planes = get_hull_planes(hull);
    let points = get_hull_points(hull);

    let mut min_edge_index = -1;
    let mut min_edge_projection = f32::MAX;

    let vertex = &vertices[vertex_index as usize];
    let mut edge_index = vertex.edge as i32;
    let edge = &edges[edge_index as usize];
    let edge_origin = points[edge.origin as usize];
    debug_assert!(edge.origin as i32 == vertex_index);

    loop {
        let edge = &edges[edge_index as usize];
        let twin = &edges[edge.twin as usize];
        let twin_origin = points[twin.origin as usize];

        let axis = normalize(sub(twin_origin, edge_origin));
        let edge_projection = abs_float(dot(axis, ref_normal));
        if edge_projection < min_edge_projection {
            min_edge_index = edge_index;
            min_edge_projection = edge_projection;
        }

        edge_index = twin.next as i32;
        let edge = &edges[edge_index as usize];
        debug_assert!(edge.origin as i32 == vertex_index);

        if edge_index == vertex.edge as i32 {
            break;
        }
    }
    debug_assert!(min_edge_index >= 0);

    let min_edge = &edges[min_edge_index as usize];
    let min_face_index1 = min_edge.face as i32;
    let min_plane1 = planes[min_face_index1 as usize];

    let min_twin = &edges[min_edge.twin as usize];
    let min_face_index2 = min_twin.face as i32;
    let min_plane2 = planes[min_face_index2 as usize];

    if dot(min_plane1.normal, ref_normal) < dot(min_plane2.normal, ref_normal) {
        min_face_index1
    } else {
        min_face_index2
    }
}

/// Flip a feature pair so face-A / face-B reference choice does not change ids.
/// (b3FlipPair)
pub fn flip_pair(mut pair: FeaturePair) -> FeaturePair {
    debug_assert!(pair.owner1 == 0 || pair.owner1 == 1);
    debug_assert!(pair.owner2 == 0 || pair.owner2 == 1);
    core::mem::swap(&mut pair.owner1, &mut pair.owner2);
    pair.owner1 = 1 - pair.owner1;
    pair.owner2 = 1 - pair.owner2;
    core::mem::swap(&mut pair.index1, &mut pair.index2);
    pair
}

/// Sutherland-Hodgman clip of a polygon against a plane. (b3ClipPolygon)
pub(crate) fn clip_polygon(
    out: &mut [ClipVertex],
    polygon: &[ClipVertex],
    count: i32,
    clip_plane: Plane,
    edge: i32,
    ref_plane: Plane,
) -> i32 {
    debug_assert!(count >= 3);

    let mut vertex1 = polygon[(count - 1) as usize];
    let mut distance1 = plane_separation(clip_plane, vertex1.position);
    let mut out_count = 0;

    for index in 0..count {
        let vertex2 = polygon[index as usize];
        let distance2 = plane_separation(clip_plane, vertex2.position);

        if distance1 <= 0.0 && distance2 <= 0.0 {
            out[out_count as usize] = vertex2;
            out_count += 1;
        } else if distance1 <= 0.0 && distance2 > 0.0 {
            let fraction = distance1 / (distance1 - distance2);
            let position = mul_add(
                vertex1.position,
                fraction,
                sub(vertex2.position, vertex1.position),
            );

            let mut vertex = ClipVertex {
                position,
                separation: plane_separation(ref_plane, position),
                pair: vertex2.pair,
            };
            vertex.pair.owner2 = FeatureOwner::ShapeA as u8;
            vertex.pair.index2 = edge as u8;
            out[out_count as usize] = vertex;
            out_count += 1;
        } else if distance2 <= 0.0 && distance1 > 0.0 {
            let fraction = distance1 / (distance1 - distance2);
            let position = mul_add(
                vertex1.position,
                fraction,
                sub(vertex2.position, vertex1.position),
            );

            let mut vertex = ClipVertex {
                position,
                separation: plane_separation(ref_plane, position),
                pair: vertex1.pair,
            };
            vertex.pair.owner1 = FeatureOwner::ShapeA as u8;
            vertex.pair.index1 = edge as u8;
            out[out_count as usize] = vertex;
            out_count += 1;

            out[out_count as usize] = vertex2;
            out_count += 1;
        }

        vertex1 = vertex2;
        distance1 = distance2;
    }

    out_count
}

/// Clip a 2-vertex segment against a plane in place. (static b3ClipSegment)
pub(crate) fn clip_segment(segment: &mut [ClipVertex; 2], plane: Plane) -> i32 {
    let mut vertex_count = 0;
    let vertex1 = segment[0];
    let vertex2 = segment[1];

    let distance1 = plane_separation(plane, vertex1.position);
    let distance2 = plane_separation(plane, vertex2.position);

    if distance1 <= 0.0 {
        segment[vertex_count as usize] = vertex1;
        vertex_count += 1;
    }
    if distance2 <= 0.0 {
        segment[vertex_count as usize] = vertex2;
        vertex_count += 1;
    }

    if distance1 * distance2 < 0.0 {
        let t = distance1 / (distance1 - distance2);
        segment[vertex_count as usize].position = add(
            mul_sv(1.0 - t, vertex1.position),
            mul_sv(t, vertex2.position),
        );
        segment[vertex_count as usize].pair = if distance1 > 0.0 {
            vertex1.pair
        } else {
            vertex2.pair
        };
        vertex_count += 1;
    }

    vertex_count
}

/// Clip a segment against the side planes of a hull face. (static b3ClipSegmentToHullFace)
pub(crate) fn clip_segment_to_hull_face(
    segment: &mut [ClipVertex; 2],
    hull: &HullData,
    ref_face: i32,
) -> i32 {
    let faces = get_hull_faces(hull);
    let planes = get_hull_planes(hull);
    let edges = get_hull_edges(hull);
    let points = get_hull_points(hull);

    let ref_plane = planes[ref_face as usize];
    let face = &faces[ref_face as usize];
    let mut edge_index = face.edge as i32;

    loop {
        let edge = &edges[edge_index as usize];
        let next_edge_index = edge.next as i32;
        let next = &edges[next_edge_index as usize];

        let vertex1 = points[edge.origin as usize];
        let vertex2 = points[next.origin as usize];
        let tangent = normalize(sub(vertex2, vertex1));
        let binormal = cross(tangent, ref_plane.normal);

        let point_count =
            clip_segment(segment, make_plane_from_normal_and_point(binormal, vertex1));
        if point_count < 2 {
            return 0;
        }

        edge_index = next_edge_index;
        if edge_index == face.edge as i32 {
            break;
        }
    }

    2
}

/// Build a clip polygon from an incident face in the given frame. (static b3BuildPolygon)
pub(crate) fn build_polygon(
    out: &mut [ClipVertex],
    transform: crate::math_functions::Transform,
    hull: &HullData,
    inc_face: i32,
    ref_plane: Plane,
) -> i32 {
    use crate::math_functions::{make_matrix_from_quat, mul_mv};

    let faces = get_hull_faces(hull);
    let edges = get_hull_edges(hull);
    let points = get_hull_points(hull);

    let face = &faces[inc_face as usize];
    let mut edge_index = face.edge as i32;
    debug_assert!(edges[edge_index as usize].face as i32 == inc_face);

    let mut out_count = 0;
    let matrix = make_matrix_from_quat(transform.q);

    loop {
        let edge = &edges[edge_index as usize];
        let next_edge_index = edge.next as i32;
        let next = &edges[next_edge_index as usize];

        let position = add(mul_mv(matrix, points[next.origin as usize]), transform.p);
        let vertex = ClipVertex {
            position,
            separation: plane_separation(ref_plane, position),
            pair: make_feature_pair(
                FeatureOwner::ShapeB,
                edge_index,
                FeatureOwner::ShapeB,
                next_edge_index,
            ),
        };

        out[out_count as usize] = vertex;
        out_count += 1;

        edge_index = next_edge_index;
        if edge_index == face.edge as i32 || out_count >= super::types::MAX_CLIP_POINTS as i32 {
            break;
        }
    }

    out_count
}
