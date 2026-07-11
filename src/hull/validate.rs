//! Hull validation and support queries.

use super::types::{
    get_hull_edges, get_hull_faces, get_hull_planes, get_hull_points, get_hull_vertices, HullData,
    HULL_VERSION,
};
use crate::core::NULL_INDEX;
use crate::math_functions::{dot, plane_separation, Vec3};

/// Find the support vertex in a direction. (b3FindHullSupportVertex)
pub fn find_hull_support_vertex(hull: &HullData, direction: Vec3) -> i32 {
    let mut best_index = NULL_INDEX;
    let mut best_dot = -f32::MAX;
    let points = get_hull_points(hull);

    for (index, point) in points.iter().enumerate() {
        let d = dot(direction, *point);
        if d > best_dot {
            best_index = index as i32;
            best_dot = d;
        }
    }
    debug_assert!(best_index >= 0);
    best_index
}

/// Find the support face in a direction. (b3FindHullSupportFace)
pub fn find_hull_support_face(hull: &HullData, direction: Vec3) -> i32 {
    let mut best_index = NULL_INDEX;
    let mut best_dot = -f32::MAX;
    let planes = get_hull_planes(hull);

    for (index, plane) in planes.iter().enumerate() {
        let d = dot(plane.normal, direction);
        if d > best_dot {
            best_dot = d;
            best_index = index as i32;
        }
    }
    debug_assert!(best_index >= 0);
    best_index
}

/// Validate hull topology and bulk properties. (b3IsValidHull)
///
/// Always runs the full check (mirrors `B3_ENABLE_VALIDATION` debug builds).
pub fn is_valid_hull(hull: &HullData) -> bool {
    if hull.version != HULL_VERSION {
        return false;
    }

    let v = hull.vertex_count;
    let e = hull.edge_count / 2;
    let f = hull.face_count;

    if v - e + f != 2 {
        return false;
    }

    let vertices = get_hull_vertices(hull);
    let edges = get_hull_edges(hull);
    for (index, vertex) in vertices.iter().enumerate() {
        let edge = &edges[vertex.edge as usize];
        if edge.origin as usize != index {
            return false;
        }
    }

    let mut index = 0;
    while index < hull.edge_count {
        let edge = &edges[index as usize];
        let twin = &edges[(index + 1) as usize];
        if edge.twin as i32 != index + 1 {
            return false;
        }
        if twin.twin as i32 != index {
            return false;
        }
        index += 2;
    }

    let faces = get_hull_faces(hull);
    let planes = get_hull_planes(hull);
    for face_index in 0..hull.face_count {
        let face = &faces[face_index as usize];
        let base_edge_index = face.edge as i32;
        let plane = planes[face_index as usize];
        if plane_separation(plane, hull.center) >= 0.0 {
            return false;
        }

        let mut edge_index = base_edge_index;
        loop {
            let edge = &edges[edge_index as usize];
            let next = &edges[edge.next as usize];
            let twin = &edges[edge.twin as usize];

            if edge.face as i32 != face_index {
                return false;
            }
            if twin.twin as i32 != edge_index {
                return false;
            }
            if next.origin != twin.origin {
                return false;
            }

            edge_index = edge.next as i32;
            if edge_index == base_edge_index {
                break;
            }
        }
    }

    if hull.volume <= 0.0 {
        return false;
    }
    if hull.surface_area <= 0.0 {
        return false;
    }
    if hull.inner_radius <= 0.0 {
        return false;
    }

    true
}
