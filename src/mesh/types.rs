//! Mesh types and blob serialization.
//!
//! Maps C's `b3MeshData` header + trailing arrays (nodes, vertices, triangles,
//! materials, flags).
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::math_functions::{Aabb, Vec3, VEC3_ONE};

/// 64-bit mesh version. (B3_MESH_VERSION)
pub const MESH_VERSION: u64 = 0xABD11AB62A6E886D;

/// Size of the C `b3MeshData` header.
pub const MESH_DATA_SIZE: usize = 88;

/// Size of the C `b3MeshNode`.
pub const MESH_NODE_SIZE: usize = 32;

/// Size of the C `b3MeshTriangle`.
pub const MESH_TRIANGLE_SIZE: usize = 12;

/// Leaf node type tag in the packed node bitfield. (B3_LEAF_NODE)
pub const LEAF_NODE: u32 = 3;

/// BVH traversal stack size. (B3_MESH_STACK_SIZE)
pub const MESH_STACK_SIZE: usize = 256;

/// Triangle mesh edge flags. (b3MeshEdgeFlags)
pub const CONCAVE_EDGE1: i32 = 0x01;
pub const CONCAVE_EDGE2: i32 = 0x02;
pub const CONCAVE_EDGE3: i32 = 0x04;
pub const INVERSE_CONCAVE_EDGE1: i32 = 0x10;
pub const INVERSE_CONCAVE_EDGE2: i32 = 0x20;
pub const INVERSE_CONCAVE_EDGE3: i32 = 0x40;
pub const ALL_CONCAVE_EDGES: i32 = CONCAVE_EDGE1 | CONCAVE_EDGE2 | CONCAVE_EDGE3;
pub const FLAT_EDGE1: i32 = CONCAVE_EDGE1 | INVERSE_CONCAVE_EDGE1;
pub const FLAT_EDGE2: i32 = CONCAVE_EDGE2 | INVERSE_CONCAVE_EDGE2;
pub const FLAT_EDGE3: i32 = CONCAVE_EDGE3 | INVERSE_CONCAVE_EDGE3;
pub const ALL_FLAT_EDGES: i32 = FLAT_EDGE1 | FLAT_EDGE2 | FLAT_EDGE3;

/// Data used to create a re-usable collision mesh. (b3MeshDef)
#[derive(Debug, Clone, Default)]
pub struct MeshDef {
    /// Triangle vertices.
    pub vertices: Vec<Vec3>,
    /// Triangle vertex indices (3 per triangle).
    pub indices: Vec<i32>,
    /// Triangle material index (1 per triangle). Empty = all zero.
    pub material_indices: Vec<u8>,
    /// Tolerance for vertex welding in length units.
    pub weld_tolerance: f32,
    /// Optionally weld nearby vertices.
    pub weld_vertices: bool,
    /// Use median split instead of SAH (good for grid-like meshes).
    pub use_median_split: bool,
    /// Compute triangle adjacency information using shared edges.
    pub identify_edges: bool,
}

/// A mesh triangle. (b3MeshTriangle)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(C)]
pub struct MeshTriangle {
    pub index1: i32,
    pub index2: i32,
    pub index3: i32,
}

/// A mesh BVH node. (b3MeshNode)
///
/// The C union bitfield is packed into `data`:
/// - bits 0–1: axis (internal) or type (leaf; 3 = leaf)
/// - bits 2–31: childOffset (internal) or triangleCount (leaf)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct MeshNode {
    pub lower_bound: Vec3,
    pub data: u32,
    pub upper_bound: Vec3,
    pub triangle_offset: u32,
}

impl MeshNode {
    #[inline]
    pub fn is_leaf(&self) -> bool {
        (self.data & 0x3) == LEAF_NODE
    }

    #[inline]
    pub fn axis(&self) -> u32 {
        self.data & 0x3
    }

    #[inline]
    pub fn child_offset(&self) -> u32 {
        self.data >> 2
    }

    #[inline]
    pub fn triangle_count(&self) -> u32 {
        self.data >> 2
    }

    #[inline]
    pub fn store_leaf(aabb: Aabb, triangle_count: i32, triangle_offset: i32) -> Self {
        debug_assert!(triangle_count >= 0);
        Self {
            lower_bound: aabb.lower_bound,
            data: LEAF_NODE | ((triangle_count as u32) << 2),
            upper_bound: aabb.upper_bound,
            triangle_offset: triangle_offset as u32,
        }
    }

    #[inline]
    pub fn store_internal(aabb: Aabb, axis: i32, child_offset: i32) -> Self {
        debug_assert!((0..3).contains(&axis));
        debug_assert!(child_offset > 1);
        Self {
            lower_bound: aabb.lower_bound,
            data: (axis as u32) | ((child_offset as u32) << 2),
            upper_bound: aabb.upper_bound,
            triangle_offset: 0,
        }
    }

    #[inline]
    pub fn aabb(&self) -> Aabb {
        Aabb {
            lower_bound: self.lower_bound,
            upper_bound: self.upper_bound,
        }
    }
}

/// Sorted triangle collision bounding volume hierarchy. (b3MeshData)
///
/// Maps to C's header + trailing blob. Offsets and `byte_count` match C so
/// [`MeshData::to_bytes`] reproduces the contiguous layout used by `b3Hash`.
#[derive(Debug, Clone)]
pub struct MeshData {
    pub version: u64,
    pub byte_count: i32,
    pub hash: u32,
    pub bounds: Aabb,
    pub surface_area: f32,
    pub tree_height: i32,
    pub degenerate_count: i32,
    pub node_offset: i32,
    pub node_count: i32,
    pub vertex_offset: i32,
    pub vertex_count: i32,
    pub triangle_offset: i32,
    pub triangle_count: i32,
    pub material_offset: i32,
    pub material_count: i32,
    pub flags_offset: i32,
    pub nodes: Vec<MeshNode>,
    pub vertices: Vec<Vec3>,
    pub triangles: Vec<MeshTriangle>,
    pub material_indices: Vec<u8>,
    pub flags: Vec<u8>,
}

impl Default for MeshData {
    fn default() -> Self {
        Self {
            version: MESH_VERSION,
            byte_count: 0,
            hash: 0,
            bounds: Aabb::default(),
            surface_area: 0.0,
            tree_height: 0,
            degenerate_count: 0,
            node_offset: 0,
            node_count: 0,
            vertex_offset: 0,
            vertex_count: 0,
            triangle_offset: 0,
            triangle_count: 0,
            material_offset: 0,
            material_count: 0,
            flags_offset: 0,
            nodes: Vec::new(),
            vertices: Vec::new(),
            triangles: Vec::new(),
            material_indices: Vec::new(),
            flags: Vec::new(),
        }
    }
}

/// Mesh data re-used with different scales. (b3Mesh)
#[derive(Debug, Clone, Copy)]
pub struct Mesh<'a> {
    pub data: &'a MeshData,
    pub scale: Vec3,
}

impl<'a> Mesh<'a> {
    pub fn new(data: &'a MeshData, scale: Vec3) -> Self {
        Self { data, scale }
    }

    pub fn with_unit_scale(data: &'a MeshData) -> Self {
        Self {
            data,
            scale: VEC3_ONE,
        }
    }
}

/// Mesh nodes. (b3GetMeshNodes)
pub fn get_mesh_nodes(mesh: &MeshData) -> &[MeshNode] {
    &mesh.nodes
}

/// Mesh vertices. (b3GetMeshVertices)
pub fn get_mesh_vertices(mesh: &MeshData) -> &[Vec3] {
    &mesh.vertices
}

/// Mesh triangles. (b3GetMeshTriangles)
pub fn get_mesh_triangles(mesh: &MeshData) -> &[MeshTriangle] {
    &mesh.triangles
}

/// Mesh material indices. (b3GetMeshMaterialIndices)
pub fn get_mesh_material_indices(mesh: &MeshData) -> &[u8] {
    &mesh.material_indices
}

/// Mesh triangle flags. (b3GetMeshFlags)
pub fn get_mesh_flags(mesh: &MeshData) -> &[u8] {
    &mesh.flags
}

fn write_u64_le(buf: &mut Vec<u8>, v: u64) {
    buf.extend_from_slice(&v.to_le_bytes());
}

fn write_u32_le(buf: &mut Vec<u8>, v: u32) {
    buf.extend_from_slice(&v.to_le_bytes());
}

fn write_i32_le(buf: &mut Vec<u8>, v: i32) {
    buf.extend_from_slice(&v.to_le_bytes());
}

fn write_f32_le(buf: &mut Vec<u8>, v: f32) {
    buf.extend_from_slice(&v.to_le_bytes());
}

fn write_vec3(buf: &mut Vec<u8>, v: Vec3) {
    write_f32_le(buf, v.x);
    write_f32_le(buf, v.y);
    write_f32_le(buf, v.z);
}

fn write_aabb(buf: &mut Vec<u8>, a: Aabb) {
    write_vec3(buf, a.lower_bound);
    write_vec3(buf, a.upper_bound);
}

fn pad_to(buf: &mut Vec<u8>, len: usize) {
    if buf.len() < len {
        buf.resize(len, 0);
    }
}

fn write_header(buf: &mut Vec<u8>, m: &MeshData, hash_override: Option<u32>) {
    write_u64_le(buf, m.version);
    write_i32_le(buf, m.byte_count);
    write_u32_le(buf, hash_override.unwrap_or(m.hash));
    write_aabb(buf, m.bounds);
    write_f32_le(buf, m.surface_area);
    write_i32_le(buf, m.tree_height);
    write_i32_le(buf, m.degenerate_count);
    write_i32_le(buf, m.node_offset);
    write_i32_le(buf, m.node_count);
    write_i32_le(buf, m.vertex_offset);
    write_i32_le(buf, m.vertex_count);
    write_i32_le(buf, m.triangle_offset);
    write_i32_le(buf, m.triangle_count);
    write_i32_le(buf, m.material_offset);
    write_i32_le(buf, m.material_count);
    write_i32_le(buf, m.flags_offset);
    debug_assert_eq!(buf.len(), MESH_DATA_SIZE);
}

fn write_node(buf: &mut Vec<u8>, n: &MeshNode) {
    write_vec3(buf, n.lower_bound);
    write_u32_le(buf, n.data);
    write_vec3(buf, n.upper_bound);
    write_u32_le(buf, n.triangle_offset);
}

impl MeshData {
    /// Serialize to the C contiguous trailing-blob layout (for hash parity).
    pub fn to_bytes(&self) -> Vec<u8> {
        self.to_bytes_with_hash(self.hash)
    }

    /// Like [`to_bytes`], but with an explicit hash field (use 0 when computing the hash).
    pub fn to_bytes_with_hash(&self, hash: u32) -> Vec<u8> {
        let mut buf = Vec::with_capacity(self.byte_count as usize);
        write_header(&mut buf, self, Some(hash));
        pad_to(&mut buf, self.node_offset as usize);
        for n in &self.nodes {
            write_node(&mut buf, n);
        }
        pad_to(&mut buf, self.vertex_offset as usize);
        for v in &self.vertices {
            write_vec3(&mut buf, *v);
        }
        pad_to(&mut buf, self.triangle_offset as usize);
        for t in &self.triangles {
            write_i32_le(&mut buf, t.index1);
            write_i32_le(&mut buf, t.index2);
            write_i32_le(&mut buf, t.index3);
        }
        pad_to(&mut buf, self.material_offset as usize);
        buf.extend_from_slice(&self.material_indices);
        pad_to(&mut buf, self.flags_offset as usize);
        buf.extend_from_slice(&self.flags);
        pad_to(&mut buf, self.byte_count as usize);
        debug_assert_eq!(buf.len(), self.byte_count as usize);
        buf
    }
}
