//! Hull validation and support queries.

use super::types::{
    get_hull_edges, get_hull_faces, get_hull_planes, get_hull_soa_vertices, get_hull_vertices,
    hull_soa_vertex_count, HullData, HULL_VERSION,
};
use crate::core::NULL_INDEX;
use crate::math_functions::{dot, plane_separation, Vec3};
use crate::simd_scalar::{add_w, blend_w, greater_than_w, load_w, mul_w, set_w, splat_w, store_w};

/// Find the support vertex in a direction. (b3FindHullSupportVertex)
///
/// Runs four lanes at a time over the SoA vertices using the scalar `b3FloatW` path. The
/// padded tail lanes repeat vertex 0, so they can only tie with it; the final lane reduction
/// breaks ties toward the lowest index, which always selects a real vertex.
pub fn find_hull_support_vertex(hull: &HullData, direction: Vec3) -> i32 {
    let mut best_index = NULL_INDEX;
    let mut best_dot = -f32::MAX;

    let vertex_count = hull.vertex_count;
    let soa_vertex_count = hull_soa_vertex_count(hull);
    let soa = get_hull_soa_vertices(hull);
    let vx = &soa[0..soa_vertex_count];
    let vy = &soa[soa_vertex_count..2 * soa_vertex_count];
    let vz = &soa[2 * soa_vertex_count..3 * soa_vertex_count];

    let dx = splat_w(direction.x);
    let dy = splat_w(direction.y);
    let dz = splat_w(direction.z);
    let four = splat_w(4.0);
    let mut index = set_w(0.0, 1.0, 2.0, 3.0);
    let mut best_dot_w = splat_w(-f32::MAX);
    let mut best_index_w = splat_w(-1.0);

    // Keep this at 4 wide to avoid regressing 4 wide platforms.
    let loop_count = ((vertex_count + 3) & !3) as usize;
    let mut i = 0;
    while i < loop_count {
        let d = add_w(
            add_w(mul_w(dx, load_w(&vx[i..])), mul_w(dy, load_w(&vy[i..]))),
            mul_w(dz, load_w(&vz[i..])),
        );
        let mask = greater_than_w(d, best_dot_w);
        best_dot_w = blend_w(best_dot_w, d, mask);
        best_index_w = blend_w(best_index_w, index, mask);
        index = add_w(index, four);
        i += 4;
    }

    let mut dots = [0.0f32; 4];
    let mut indices = [0.0f32; 4];
    store_w(&mut dots, best_dot_w);
    store_w(&mut indices, best_index_w);

    for lane in 0..4 {
        let lane_index = indices[lane] as i32;
        if lane_index >= 0
            && (dots[lane] > best_dot || (dots[lane] == best_dot && lane_index < best_index))
        {
            best_index = lane_index;
            best_dot = dots[lane];
        }
    }

    debug_assert!(0 <= best_index && best_index < vertex_count);
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
