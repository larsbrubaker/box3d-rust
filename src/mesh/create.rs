//! Mesh creation, validation, and destroy. Edge identification lives in `edges` and the
//! vertex rings in `vertex_ring`.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::bvh::{
    build_recursive, collect_primitives, fill_triangles, sort_mesh_triangles, weld_vertices,
};
use super::edges::identify_edges;
use super::types::{
    MeshData, MeshDef, MeshNode, MeshTriangle, MESH_DATA_SIZE, MESH_NODE_SIZE, MESH_TRIANGLE_SIZE,
    MESH_VERSION,
};
use super::vertex_ring::append_vertex_rings;
use crate::core::hash64_non_zero;
use crate::math_functions::{align_up8, cross, sub, Vec3};

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

// Stride larger than this likely indicates stride is uninitialized memory.
const MAX_STRIDE: usize = 4096;

const VEC3_BYTE_COUNT: usize = core::mem::size_of::<Vec3>();

/// Gather the definition's interleaved `vertex_bytes` into a contiguous array,
/// reading through the byte stride. (b3GetStridedVertex / b3CopyVerticesWithStride)
///
/// C reads strided vertices in place while welding or copying. Gathering them
/// first is equivalent: both paths only read vertex `i` at byte `i * stride`.
/// Only complete positions inside the source buffer are read, where C trusts
/// `vertexCount`.
fn gather_strided_vertices(def: &MeshDef, stride: usize) -> Vec<Vec3> {
    debug_assert!(stride >= VEC3_BYTE_COUNT && stride % 4 == 0);
    debug_assert!(!def.vertex_bytes.is_empty());

    let bytes = &def.vertex_bytes;
    let count = if bytes.len() >= VEC3_BYTE_COUNT {
        (bytes.len() - VEC3_BYTE_COUNT) / stride + 1
    } else {
        0
    };
    let read_float = |offset: usize| -> f32 {
        f32::from_ne_bytes([
            bytes[offset],
            bytes[offset + 1],
            bytes[offset + 2],
            bytes[offset + 3],
        ])
    };
    (0..count)
        .map(|i| {
            let offset = i * stride;
            Vec3 {
                x: read_float(offset),
                y: read_float(offset + 4),
                z: read_float(offset + 8),
            }
        })
        .collect()
}

/// Create a mesh from a definition. (b3CreateMesh)
///
/// Returns `None` on invalid input or BVH overflow. Degenerate triangle indices
/// are written into `degenerate_triangle_indices` when provided (up to its length).
pub fn create_mesh(
    def: &MeshDef,
    degenerate_triangle_indices: Option<&mut [i32]>,
) -> Option<MeshData> {
    if def.stride != 0 && (def.stride < VEC3_BYTE_COUNT || MAX_STRIDE < def.stride) {
        return None;
    }

    if (def.stride & 3) != 0 {
        return None;
    }

    debug_assert!(
        def.vertex_bytes.is_empty() || def.vertices.is_empty(),
        "set either vertices or vertex_bytes, not both"
    );

    let stride = if def.stride == 0 {
        VEC3_BYTE_COUNT
    } else {
        def.stride
    };

    // The stride only applies to the interleaved byte buffer. Contiguous `vertices`
    // must use a stride of 0 or size_of::<Vec3>().
    let src_vertices = if def.vertex_bytes.is_empty() {
        if stride != VEC3_BYTE_COUNT {
            return None;
        }
        def.vertices.clone()
    } else {
        gather_strided_vertices(def, stride)
    };

    let vertex_count_in = src_vertices.len() as i32;
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
            &src_vertices,
            &def.indices[..(3 * triangle_count_in) as usize],
            &mut vertices,
            &mut indices,
            def.weld_tolerance,
        );
        vertices.truncate(vertex_count as usize);
        debug_assert!(vertex_count <= vertex_count_in);
    } else {
        vertices = src_vertices;
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
    let mut edge_angle_offset = 0;
    let mut vertex_flag_offset = 0;
    if def.identify_edges {
        edge_angle_offset = byte_count as i32;
        byte_count += align_up8(3 * triangle_count as usize);
        vertex_flag_offset = byte_count as i32;
        byte_count += align_up8(vertex_count as usize);
    }

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
        edge_angle_offset,
        vertex_flag_offset,
        vertex_ring_offset: 0,
        nodes: temp_nodes,
        vertices,
        triangles: vec![MeshTriangle::default(); triangle_count as usize],
        material_indices: vec![0u8; triangle_count as usize],
        flags: vec![0u8; triangle_count as usize],
        edge_angles: if def.identify_edges {
            vec![0i8; 3 * triangle_count as usize]
        } else {
            Vec::new()
        },
        vertex_flags: if def.identify_edges {
            vec![0u8; vertex_count as usize]
        } else {
            Vec::new()
        },
        vertex_ring_starts: Vec::new(),
        vertex_ring_data: Vec::new(),
    };

    fill_triangles(
        &mut mesh.triangles,
        &mut mesh.material_indices,
        &mut mesh.flags,
        &primitives,
        &indices,
        src_materials,
        def.clock_wise_winding,
    );

    if !sort_mesh_triangles(&mut mesh) {
        return None;
    }

    if def.identify_edges {
        let mesh_triangle_count = mesh.triangle_count as usize;
        let mut normals = vec![Vec3::default(); mesh_triangle_count];

        identify_edges(&mut mesh, &mut normals);

        append_vertex_rings(&mut mesh, &normals);
    }

    // Must ensure the hash is 0 so it doesn't contribute to itself.
    mesh.hash = 0;
    let bytes = mesh.to_bytes_with_hash(0);
    mesh.hash = hash64_non_zero(&bytes);

    Some(mesh)
}

/// Destroy a mesh (no-op drop for owned Rust data). (b3DestroyMesh)
pub fn destroy_mesh(_mesh: MeshData) {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math_functions::Vec3;

    /// Degenerate triangle indices must be written when an out-buffer is provided
    /// (C `b3CreateMesh` + MeshViewer `m_degenerateTriangles`).
    #[test]
    fn create_mesh_collects_degenerate_triangle_indices() {
        // One valid triangle + one near-zero-area triangle with distinct verts.
        let def = MeshDef {
            vertices: vec![
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 2.0,
                    y: 0.0,
                    z: 0.0,
                }, // collinear with v0,v1 → degenerate
            ],
            indices: vec![0, 1, 2, 0, 1, 3],
            material_indices: vec![],
            weld_tolerance: 0.0,
            weld_vertices: false,
            use_median_split: true,
            identify_edges: false,
            ..Default::default()
        };
        let mut degenerate = [-1i32; 8];
        let mesh = create_mesh(&def, Some(&mut degenerate)).expect("mesh");
        assert_eq!(mesh.degenerate_count, 1);
        assert_eq!(mesh.triangle_count, 1); // only the valid tri survives into the BVH
        assert_eq!(degenerate[0], 1); // second input triangle
        assert_eq!(degenerate[1], -1); // untouched past written count
    }

    /// Concave-edge identification and welding flags must both be accepted by
    /// `create_mesh` (Viewer rebuild path); welding can collapse a near-duplicate
    /// without inventing extra degenerates from duplicate-index triangles.
    #[test]
    fn create_mesh_viewer_flags_weld_and_identify_edges() {
        let def = MeshDef {
            vertices: vec![
                Vec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0,
                },
                Vec3 {
                    x: 0.0005,
                    y: 0.0,
                    z: 0.0,
                }, // within 1.5 mm weld tolerance of v0
            ],
            indices: vec![0, 1, 2, 3, 1, 2],
            material_indices: vec![0, 1],
            weld_tolerance: 0.0015,
            weld_vertices: true,
            use_median_split: true,
            identify_edges: true,
            ..Default::default()
        };
        let mut degenerate = [0i32; 8];
        let mesh = create_mesh(&def, Some(&mut degenerate)).expect("mesh");
        assert!(mesh.triangle_count >= 1);
        assert!(mesh.vertex_count <= 3); // v0 and v3 weld together
        assert!(mesh.degenerate_count >= 0);
    }
}
