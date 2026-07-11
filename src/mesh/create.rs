//! Mesh creation, edge identification, validation, and destroy.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::bvh::{
    build_recursive, collect_primitives, fill_triangles, sort_mesh_triangles, weld_vertices,
};
use super::types::{
    MeshData, MeshDef, MeshNode, MeshTriangle, CONCAVE_EDGE1, CONCAVE_EDGE2, CONCAVE_EDGE3,
    INVERSE_CONCAVE_EDGE1, INVERSE_CONCAVE_EDGE2, INVERSE_CONCAVE_EDGE3, MESH_DATA_SIZE,
    MESH_NODE_SIZE, MESH_TRIANGLE_SIZE, MESH_VERSION,
};
use crate::core::{hash, non_zero_hash, HASH_INIT, NULL_INDEX};
use crate::math_functions::{
    align_up8, cross, dot, max_int, min_int, normalize, signed_volume, sub, Vec3,
};
use std::collections::HashMap;

struct MeshEdge {
    vertex1: i32,
    vertex2: i32,
    triangle1: i32,
    triangle2: i32,
    triangle_count: u16,
    triangle_edge_index1: u8,
    triangle_edge_index2: u8,
}

fn identify_edges(mesh: &mut MeshData) {
    let triangle_count = mesh.triangle_count as usize;
    let edge_count = 3 * triangle_count;
    let mut edges = Vec::with_capacity(edge_count);
    let mut normals = vec![Vec3::default(); triangle_count];

    for i in 0..triangle_count {
        let triangle = mesh.triangles[i];
        let i1 = triangle.index1;
        let i2 = triangle.index2;
        let i3 = triangle.index3;

        edges.push(MeshEdge {
            vertex1: min_int(i1, i2),
            vertex2: max_int(i1, i2),
            triangle1: i as i32,
            triangle2: NULL_INDEX,
            triangle_edge_index1: 0,
            triangle_edge_index2: 0xFF,
            triangle_count: 1,
        });
        edges.push(MeshEdge {
            vertex1: min_int(i2, i3),
            vertex2: max_int(i2, i3),
            triangle1: i as i32,
            triangle2: NULL_INDEX,
            triangle_edge_index1: 1,
            triangle_edge_index2: 0xFF,
            triangle_count: 1,
        });
        edges.push(MeshEdge {
            vertex1: min_int(i3, i1),
            vertex2: max_int(i3, i1),
            triangle1: i as i32,
            triangle2: NULL_INDEX,
            triangle_edge_index1: 2,
            triangle_edge_index2: 0xFF,
            triangle_count: 1,
        });

        let v1 = mesh.vertices[i1 as usize];
        let v2 = mesh.vertices[i2 as usize];
        let v3 = mesh.vertices[i3 as usize];
        let e1 = sub(v2, v1);
        let e2 = sub(v3, v1);
        let n = cross(e1, e2);
        normals[i] = normalize(n);
    }

    let mut map: HashMap<u64, i32> = HashMap::with_capacity(edge_count);
    let key0 = ((edges[0].vertex1 as u64) << 32) | (edges[0].vertex2 as u64);
    map.insert(key0, 0);

    for i in 1..edge_count {
        let key = ((edges[i].vertex1 as u64) << 32) | (edges[i].vertex2 as u64);
        let triangle1 = edges[i].triangle1;
        let triangle_edge_index1 = edges[i].triangle_edge_index1;
        if let Some(&other_index) = map.get(&key) {
            debug_assert!((other_index as usize) < i);
            let base = &mut edges[other_index as usize];
            if base.triangle_count == 1 {
                base.triangle2 = triangle1;
                base.triangle_edge_index2 = triangle_edge_index1;
            }
            base.triangle_count += 1;
        } else {
            map.insert(key, i as i32);
        }
    }
    drop(map);

    let edge_flags_concave = [CONCAVE_EDGE1, CONCAVE_EDGE2, CONCAVE_EDGE3];
    let edge_flags_inverse = [
        INVERSE_CONCAVE_EDGE1,
        INVERSE_CONCAVE_EDGE2,
        INVERSE_CONCAVE_EDGE3,
    ];

    for i in 0..edge_count {
        let edge = &edges[i];
        if edge.triangle_count != 2 {
            continue;
        }

        debug_assert!(edge.triangle_edge_index1 < 3);
        debug_assert!(edge.triangle_edge_index2 < 3);

        let triangle1 = mesh.triangles[edge.triangle1 as usize];
        let triangle2 = mesh.triangles[edge.triangle2 as usize];

        let j1 = triangle2.index1;
        let j2 = triangle2.index2;
        let j3 = triangle2.index3;

        let opposite = match edge.triangle_edge_index2 {
            0 => j3,
            1 => j1,
            2 => j2,
            _ => unreachable!(),
        };

        let i1 = triangle1.index1;
        let i2 = triangle1.index2;
        let i3 = triangle1.index3;

        let v1 = mesh.vertices[i1 as usize];
        let v2 = mesh.vertices[i2 as usize];
        let v3 = mesh.vertices[i3 as usize];
        let p = mesh.vertices[opposite as usize];

        let cos5_deg = 0.9962f32;
        let signed_vol = signed_volume(v1, v2, v3, p);
        let n1 = normals[edge.triangle1 as usize];
        let n2 = normals[edge.triangle2 as usize];
        let cos_angle = dot(n1, n2);

        if signed_vol > 0.0 || cos_angle > cos5_deg {
            mesh.flags[edge.triangle1 as usize] |=
                edge_flags_concave[edge.triangle_edge_index1 as usize] as u8;
            mesh.flags[edge.triangle2 as usize] |=
                edge_flags_concave[edge.triangle_edge_index2 as usize] as u8;
        }

        if signed_vol < 0.0 || cos_angle > cos5_deg {
            mesh.flags[edge.triangle1 as usize] |=
                edge_flags_inverse[edge.triangle_edge_index1 as usize] as u8;
            mesh.flags[edge.triangle2 as usize] |=
                edge_flags_inverse[edge.triangle_edge_index2 as usize] as u8;
        }
    }
}

fn node_height(nodes: &[MeshNode], index: usize) -> i32 {
    let node = &nodes[index];
    if node.is_leaf() {
        return 0;
    }
    let left = node_height(nodes, index + 1);
    let right = node_height(nodes, index + node.child_offset() as usize);
    1 + left.max(right)
}

/// Height of the mesh BVH. (b3GetHeight)
pub fn get_height(mesh: &MeshData) -> i32 {
    if mesh.nodes.is_empty() {
        return 0;
    }
    node_height(&mesh.nodes, 0)
}

/// Validate mesh version and size. (b3IsValidMesh)
pub fn is_valid_mesh(mesh: Option<&MeshData>) -> bool {
    let Some(mesh) = mesh else {
        return false;
    };
    if mesh.version != MESH_VERSION {
        return false;
    }
    if mesh.byte_count < MESH_DATA_SIZE as i32 {
        return false;
    }
    true
}

/// Create a mesh from a definition. (b3CreateMesh)
///
/// Returns `None` on invalid input or BVH overflow. Degenerate triangle indices
/// are written into `degenerate_triangle_indices` when provided (up to its length).
pub fn create_mesh(
    def: &MeshDef,
    degenerate_triangle_indices: Option<&mut [i32]>,
) -> Option<MeshData> {
    let vertex_count_in = def.vertices.len() as i32;
    let triangle_count_in = (def.indices.len() / 3) as i32;

    if vertex_count_in < 3 || triangle_count_in <= 0 || def.indices.len() < 3 {
        return None;
    }

    let mut indices = vec![0i32; (3 * triangle_count_in) as usize];
    let mut vertices: Vec<Vec3>;
    let mut vertex_count = vertex_count_in;

    if def.weld_vertices && def.weld_tolerance > 0.0 {
        vertices = vec![Vec3::default(); vertex_count as usize];
        vertex_count = weld_vertices(
            &def.vertices,
            &def.indices[..(3 * triangle_count_in) as usize],
            &mut vertices,
            &mut indices,
            def.weld_tolerance,
        );
        vertices.truncate(vertex_count as usize);
        debug_assert!(vertex_count <= vertex_count_in);
    } else {
        vertices = def.vertices.clone();
        indices.copy_from_slice(&def.indices[..(3 * triangle_count_in) as usize]);
    }

    let src_materials = if def.material_indices.is_empty() {
        None
    } else {
        Some(def.material_indices.as_slice())
    };

    let (mut primitives, degenerate_count, surface_area, material_count, mesh_bounds) =
        collect_primitives(&vertices, &indices, src_materials, triangle_count_in);

    if let Some(out) = degenerate_triangle_indices {
        let min_area = 0.01 * crate::constants::linear_slop() * crate::constants::linear_slop();
        let mut written = 0usize;
        let mut seen = 0i32;
        for index in 0..triangle_count_in {
            let index1 = indices[(3 * index) as usize];
            let index2 = indices[(3 * index + 1) as usize];
            let index3 = indices[(3 * index + 2) as usize];
            let vertex1 = vertices[index1 as usize];
            let vertex2 = vertices[index2 as usize];
            let vertex3 = vertices[index3 as usize];
            let normal = cross(sub(vertex2, vertex1), sub(vertex3, vertex1));
            let area = 0.5 * crate::math_functions::length(normal);
            if area < min_area && index1 != index2 && index1 != index3 && index2 != index3 {
                seen += 1;
                if written < out.len() {
                    out[written] = index;
                    written += 1;
                }
            }
        }
        debug_assert_eq!(seen, degenerate_count);
        let _ = seen;
    }

    let triangle_count = primitives.len() as i32;
    if !crate::math_functions::is_sane_aabb(mesh_bounds) {
        return None;
    }

    let mut temp_nodes = Vec::with_capacity((2 * triangle_count - 1).max(1) as usize);
    let mut tree_height = 0i32;
    build_recursive(
        &mut temp_nodes,
        triangle_count,
        &mut primitives,
        0,
        def.use_median_split,
        &mut tree_height,
    );

    let mut byte_count = align_up8(MESH_DATA_SIZE);
    let node_offset = byte_count as i32;
    byte_count += align_up8(temp_nodes.len() * MESH_NODE_SIZE);
    let vertex_offset = byte_count as i32;
    byte_count += align_up8(vertex_count as usize * core::mem::size_of::<Vec3>());
    let triangle_offset = byte_count as i32;
    byte_count += align_up8(triangle_count as usize * MESH_TRIANGLE_SIZE);
    let material_indices_offset = byte_count as i32;
    byte_count += align_up8(triangle_count as usize);
    let flags_offset = byte_count as i32;
    byte_count += align_up8(triangle_count as usize);

    let mut mesh = MeshData {
        version: MESH_VERSION,
        byte_count: byte_count as i32,
        hash: 0,
        bounds: mesh_bounds,
        surface_area,
        node_count: temp_nodes.len() as i32,
        tree_height,
        vertex_count,
        triangle_count,
        degenerate_count,
        node_offset,
        vertex_offset,
        triangle_offset,
        material_offset: material_indices_offset,
        material_count,
        flags_offset,
        nodes: temp_nodes,
        vertices,
        triangles: vec![MeshTriangle::default(); triangle_count as usize],
        material_indices: vec![0u8; triangle_count as usize],
        flags: vec![0u8; triangle_count as usize],
    };

    fill_triangles(
        &mut mesh.triangles,
        &mut mesh.material_indices,
        &mut mesh.flags,
        &primitives,
        &indices,
        src_materials,
    );

    if !sort_mesh_triangles(&mut mesh) {
        return None;
    }

    if def.identify_edges {
        identify_edges(&mut mesh);
    }

    let bytes = mesh.to_bytes_with_hash(0);
    mesh.hash = non_zero_hash(hash(HASH_INIT, &bytes));

    Some(mesh)
}

/// Destroy a mesh (no-op drop for owned Rust data). (b3DestroyMesh)
pub fn destroy_mesh(_mesh: MeshData) {}
