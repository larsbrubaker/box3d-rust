//! Compound blob serialization (convert to/from bytes).
//!
//! Port of `b3ConvertCompoundToBytes` / `b3ConvertBytesToCompound`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{
    CompoundCapsule, CompoundData, CompoundSphere, HullInstance, MeshInstance,
    COMPOUND_CONVEX_SIZE, COMPOUND_DATA_SIZE, COMPOUND_VERSION, DYNAMIC_TREE_SIZE,
    HULL_INSTANCE_SIZE, MAX_COMPOUND_MESH_MATERIALS, MESH_INSTANCE_SIZE, TREE_NODE_SIZE,
};
use crate::core::NULL_INDEX;
use crate::dynamic_tree::{DynamicTree, TreeNode, ALLOCATED_NODE, DYNAMIC_TREE_VERSION, LEAF_NODE};
use crate::geometry::{SurfaceMaterial, SURFACE_MATERIAL_SIZE};
use crate::hull::{
    HullData, HullFace, HullHalfEdge, HullVertex, HULL_DATA_SIZE, HULL_VERSION,
};
use crate::math_functions::{
    Aabb, Matrix3, Plane, Quat, Transform, Vec3, MAT3_ZERO, QUAT_IDENTITY, TRANSFORM_IDENTITY,
    VEC3_ONE, VEC3_ZERO,
};
use crate::mesh::{
    MeshData, MeshNode, MeshTriangle, MESH_DATA_SIZE, MESH_NODE_SIZE, MESH_TRIANGLE_SIZE,
    MESH_VERSION,
};

fn write_u64(buf: &mut Vec<u8>, v: u64) {
    buf.extend_from_slice(&v.to_le_bytes());
}
fn write_u32(buf: &mut Vec<u8>, v: u32) {
    buf.extend_from_slice(&v.to_le_bytes());
}
fn write_i32(buf: &mut Vec<u8>, v: i32) {
    buf.extend_from_slice(&v.to_le_bytes());
}
fn write_f32(buf: &mut Vec<u8>, v: f32) {
    buf.extend_from_slice(&v.to_le_bytes());
}
fn write_vec3(buf: &mut Vec<u8>, v: Vec3) {
    write_f32(buf, v.x);
    write_f32(buf, v.y);
    write_f32(buf, v.z);
}
fn write_quat(buf: &mut Vec<u8>, q: Quat) {
    write_vec3(buf, q.v);
    write_f32(buf, q.s);
}
fn write_transform(buf: &mut Vec<u8>, t: Transform) {
    write_vec3(buf, t.p);
    write_quat(buf, t.q);
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

fn write_tree_node(buf: &mut Vec<u8>, node: &TreeNode) {
    write_aabb(buf, node.aabb);
    write_u64(buf, node.category_bits);
    if node.flags & LEAF_NODE != 0 {
        write_u64(buf, node.user_data);
    } else {
        write_i32(buf, node.child1);
        write_i32(buf, node.child2);
    }
    // parent/next union: free nodes use next; allocated use parent
    if node.flags & ALLOCATED_NODE != 0 {
        write_i32(buf, node.parent);
    } else {
        write_i32(buf, node.next);
    }
    buf.extend_from_slice(&node.height.to_le_bytes());
    buf.extend_from_slice(&node.flags.to_le_bytes());
}

fn write_dynamic_tree_header(buf: &mut Vec<u8>, tree: &DynamicTree, nodes_ptr_zeroed: bool) {
    let start = buf.len();
    write_u64(buf, tree.version());
    // nodes pointer — scrubbed to null for serialization
    if nodes_ptr_zeroed {
        buf.extend_from_slice(&0u64.to_le_bytes());
    } else {
        buf.extend_from_slice(&0u64.to_le_bytes());
    }
    write_i32(buf, tree.root);
    write_i32(buf, tree.node_count());
    write_i32(buf, tree.node_capacity());
    write_i32(buf, tree.proxy_count());
    write_i32(buf, tree.free_list);
    // pad to 8 for leafIndices*
    pad_to(buf, start + 40);
    // leafIndices, leafBoxes, leafCenters, binIndices — all null
    buf.extend_from_slice(&0u64.to_le_bytes());
    buf.extend_from_slice(&0u64.to_le_bytes());
    buf.extend_from_slice(&0u64.to_le_bytes());
    buf.extend_from_slice(&0u64.to_le_bytes());
    write_i32(buf, tree.rebuild_capacity);
    pad_to(buf, start + DYNAMIC_TREE_SIZE);
}

fn write_compound_header(buf: &mut Vec<u8>, c: &CompoundData) {
    write_u64(buf, c.version);
    write_i32(buf, c.byte_count);
    write_i32(buf, c.node_offset);
    write_dynamic_tree_header(buf, &c.tree, true);
    write_i32(buf, c.material_offset);
    write_i32(buf, c.material_count);
    write_i32(buf, c.capsule_offset);
    write_i32(buf, c.capsule_count);
    write_i32(buf, c.hull_offset);
    write_i32(buf, c.hull_count);
    write_i32(buf, c.shared_hull_count);
    write_i32(buf, c.mesh_offset);
    write_i32(buf, c.mesh_count);
    write_i32(buf, c.shared_mesh_count);
    write_i32(buf, c.sphere_offset);
    write_i32(buf, c.sphere_count);
    debug_assert_eq!(buf.len(), COMPOUND_DATA_SIZE);
}

fn write_capsule(buf: &mut Vec<u8>, c: &CompoundCapsule) {
    write_vec3(buf, c.capsule.center1);
    write_vec3(buf, c.capsule.center2);
    write_f32(buf, c.capsule.radius);
    write_i32(buf, c.material_index);
}

fn write_sphere(buf: &mut Vec<u8>, s: &CompoundSphere) {
    write_vec3(buf, s.sphere.center);
    write_f32(buf, s.sphere.radius);
    write_i32(buf, s.material_index);
}

fn write_hull_instance(buf: &mut Vec<u8>, h: &HullInstance) {
    write_transform(buf, h.transform);
    write_u32(buf, h.hull_offset);
    write_u32(buf, h.material_index);
}

fn write_mesh_instance(buf: &mut Vec<u8>, m: &MeshInstance) {
    write_transform(buf, m.transform);
    write_vec3(buf, m.scale);
    write_u32(buf, m.mesh_offset);
    for i in 0..MAX_COMPOUND_MESH_MATERIALS {
        write_u32(buf, m.material_indices[i]);
    }
}

impl CompoundData {
    /// Serialize to the C contiguous blob layout.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(self.byte_count as usize);
        write_compound_header(&mut buf, self);

        pad_to(&mut buf, self.node_offset as usize);
        for node in &self.tree.nodes {
            write_tree_node(&mut buf, node);
        }

        pad_to(&mut buf, self.material_offset as usize);
        for mat in &self.materials {
            buf.extend_from_slice(&mat.to_bytes());
        }

        pad_to(&mut buf, self.capsule_offset as usize);
        for cap in &self.capsules {
            write_capsule(&mut buf, cap);
        }

        pad_to(&mut buf, self.hull_offset as usize);
        for inst in &self.hull_instances {
            write_hull_instance(&mut buf, inst);
        }
        // Shared hull blobs at their recorded offsets
        for (i, hull) in self.shared_hulls.iter().enumerate() {
            let offset = self.hull_instances
                .iter()
                .find(|inst| inst.shared_index as usize == i)
                .map(|inst| inst.hull_offset as usize)
                .unwrap_or(0);
            if offset > 0 {
                pad_to(&mut buf, offset);
                buf.extend_from_slice(&hull.to_bytes());
            }
        }

        pad_to(&mut buf, self.mesh_offset as usize);
        for inst in &self.mesh_instances {
            write_mesh_instance(&mut buf, inst);
        }
        for (i, mesh) in self.shared_meshes.iter().enumerate() {
            let offset = self.mesh_instances
                .iter()
                .find(|inst| inst.shared_index as usize == i)
                .map(|inst| inst.mesh_offset as usize)
                .unwrap_or(0);
            if offset > 0 {
                pad_to(&mut buf, offset);
                buf.extend_from_slice(&mesh.to_bytes());
            }
        }

        pad_to(&mut buf, self.sphere_offset as usize);
        for sph in &self.spheres {
            write_sphere(&mut buf, sph);
        }

        pad_to(&mut buf, self.byte_count as usize);
        debug_assert_eq!(buf.len(), self.byte_count as usize);
        buf
    }
}

/// Scrub and return the compound as a byte buffer. (b3ConvertCompoundToBytes)
pub fn convert_compound_to_bytes(compound: &CompoundData) -> Vec<u8> {
    compound.to_bytes()
}

fn read_u64(buf: &[u8], o: usize) -> u64 {
    u64::from_le_bytes(buf[o..o + 8].try_into().unwrap())
}
fn read_u32(buf: &[u8], o: usize) -> u32 {
    u32::from_le_bytes(buf[o..o + 4].try_into().unwrap())
}
fn read_i32(buf: &[u8], o: usize) -> i32 {
    i32::from_le_bytes(buf[o..o + 4].try_into().unwrap())
}
fn read_f32(buf: &[u8], o: usize) -> f32 {
    f32::from_le_bytes(buf[o..o + 4].try_into().unwrap())
}
fn read_vec3(buf: &[u8], o: usize) -> Vec3 {
    Vec3 {
        x: read_f32(buf, o),
        y: read_f32(buf, o + 4),
        z: read_f32(buf, o + 8),
    }
}
fn read_quat(buf: &[u8], o: usize) -> Quat {
    Quat {
        v: read_vec3(buf, o),
        s: read_f32(buf, o + 12),
    }
}
fn read_transform(buf: &[u8], o: usize) -> Transform {
    Transform {
        p: read_vec3(buf, o),
        q: read_quat(buf, o + 12),
    }
}
fn read_aabb(buf: &[u8], o: usize) -> Aabb {
    Aabb {
        lower_bound: read_vec3(buf, o),
        upper_bound: read_vec3(buf, o + 12),
    }
}

fn read_tree_node(buf: &[u8], o: usize) -> TreeNode {
    let aabb = read_aabb(buf, o);
    let category_bits = read_u64(buf, o + 24);
    let union8 = read_u64(buf, o + 32);
    let parent_or_next = read_i32(buf, o + 40);
    let height = u16::from_le_bytes(buf[o + 44..o + 46].try_into().unwrap());
    let flags = u16::from_le_bytes(buf[o + 46..o + 48].try_into().unwrap());

    let (child1, child2, user_data) = if flags & LEAF_NODE != 0 {
        (NULL_INDEX, NULL_INDEX, union8)
    } else {
        let c1 = read_i32(buf, o + 32);
        let c2 = read_i32(buf, o + 36);
        (c1, c2, 0)
    };

    let (parent, next) = if flags & ALLOCATED_NODE != 0 {
        (parent_or_next, NULL_INDEX)
    } else {
        (NULL_INDEX, parent_or_next)
    };

    TreeNode {
        aabb,
        category_bits,
        child1,
        child2,
        user_data,
        parent,
        next,
        height,
        flags,
    }
}

fn read_plane(buf: &[u8], o: usize) -> Plane {
    Plane {
        normal: read_vec3(buf, o),
        offset: read_f32(buf, o + 12),
    }
}

fn read_matrix3(buf: &[u8], o: usize) -> Matrix3 {
    Matrix3 {
        cx: read_vec3(buf, o),
        cy: read_vec3(buf, o + 12),
        cz: read_vec3(buf, o + 24),
    }
}

fn read_hull_data(buf: &[u8]) -> Option<HullData> {
    if buf.len() < HULL_DATA_SIZE {
        return None;
    }
    let version = read_u64(buf, 0);
    if version != HULL_VERSION {
        return None;
    }
    let byte_count = read_i32(buf, 8);
    if byte_count as usize > buf.len() || byte_count < HULL_DATA_SIZE as i32 {
        return None;
    }
    let hash = read_u32(buf, 12);
    let aabb = read_aabb(buf, 16);
    let surface_area = read_f32(buf, 40);
    let volume = read_f32(buf, 44);
    let inner_radius = read_f32(buf, 48);
    let center = read_vec3(buf, 52);
    let central_inertia = read_matrix3(buf, 64);
    let vertex_count = read_i32(buf, 100);
    let vertex_offset = read_i32(buf, 104);
    let point_offset = read_i32(buf, 108);
    let edge_count = read_i32(buf, 112);
    let edge_offset = read_i32(buf, 116);
    let face_count = read_i32(buf, 120);
    let face_offset = read_i32(buf, 124);
    let plane_offset = read_i32(buf, 128);
    let padding = read_i32(buf, 132);

    let mut vertices = Vec::with_capacity(vertex_count as usize);
    for i in 0..vertex_count as usize {
        let o = vertex_offset as usize + i;
        if o >= buf.len() {
            return None;
        }
        vertices.push(HullVertex { edge: buf[o] });
    }

    let mut points = Vec::with_capacity(vertex_count as usize);
    for i in 0..vertex_count as usize {
        let o = point_offset as usize + i * 12;
        points.push(read_vec3(buf, o));
    }

    let mut edges = Vec::with_capacity(edge_count as usize);
    for i in 0..edge_count as usize {
        let o = edge_offset as usize + i * 4;
        edges.push(HullHalfEdge {
            next: buf[o],
            twin: buf[o + 1],
            origin: buf[o + 2],
            face: buf[o + 3],
        });
    }

    let mut faces = Vec::with_capacity(face_count as usize);
    for i in 0..face_count as usize {
        let o = face_offset as usize + i;
        faces.push(HullFace { edge: buf[o] });
    }

    let mut planes = Vec::with_capacity(face_count as usize);
    for i in 0..face_count as usize {
        let o = plane_offset as usize + i * 16;
        planes.push(read_plane(buf, o));
    }

    Some(HullData {
        version,
        byte_count,
        hash,
        aabb,
        surface_area,
        volume,
        inner_radius,
        center,
        central_inertia,
        vertex_count,
        vertex_offset,
        point_offset,
        edge_count,
        edge_offset,
        face_count,
        face_offset,
        plane_offset,
        padding,
        vertices,
        points,
        edges,
        faces,
        planes,
    })
}

fn read_mesh_data(buf: &[u8]) -> Option<MeshData> {
    if buf.len() < MESH_DATA_SIZE {
        return None;
    }
    let version = read_u64(buf, 0);
    if version != MESH_VERSION {
        return None;
    }
    let byte_count = read_i32(buf, 8);
    if byte_count as usize > buf.len() || byte_count < MESH_DATA_SIZE as i32 {
        return None;
    }
    let hash = read_u32(buf, 12);
    let bounds = read_aabb(buf, 16);
    let surface_area = read_f32(buf, 40);
    let tree_height = read_i32(buf, 44);
    let degenerate_count = read_i32(buf, 48);
    let node_offset = read_i32(buf, 52);
    let node_count = read_i32(buf, 56);
    let vertex_offset = read_i32(buf, 60);
    let vertex_count = read_i32(buf, 64);
    let triangle_offset = read_i32(buf, 68);
    let triangle_count = read_i32(buf, 72);
    let material_offset = read_i32(buf, 76);
    let material_count = read_i32(buf, 80);
    let flags_offset = read_i32(buf, 84);

    let mut nodes = Vec::with_capacity(node_count as usize);
    for i in 0..node_count as usize {
        let o = node_offset as usize + i * MESH_NODE_SIZE;
        nodes.push(MeshNode {
            lower_bound: read_vec3(buf, o),
            data: read_u32(buf, o + 12),
            upper_bound: read_vec3(buf, o + 16),
            triangle_offset: read_u32(buf, o + 28),
        });
    }

    let mut vertices = Vec::with_capacity(vertex_count as usize);
    for i in 0..vertex_count as usize {
        vertices.push(read_vec3(buf, vertex_offset as usize + i * 12));
    }

    let mut triangles = Vec::with_capacity(triangle_count as usize);
    for i in 0..triangle_count as usize {
        let o = triangle_offset as usize + i * MESH_TRIANGLE_SIZE;
        triangles.push(MeshTriangle {
            index1: read_i32(buf, o),
            index2: read_i32(buf, o + 4),
            index3: read_i32(buf, o + 8),
        });
    }

    let mat_start = material_offset as usize;
    let material_indices = buf[mat_start..mat_start + material_count as usize].to_vec();
    let flags_start = flags_offset as usize;
    let flags = if flags_offset > 0 {
        buf[flags_start..flags_start + triangle_count as usize].to_vec()
    } else {
        Vec::new()
    };

    Some(MeshData {
        version,
        byte_count,
        hash,
        bounds,
        surface_area,
        tree_height,
        degenerate_count,
        node_offset,
        node_count,
        vertex_offset,
        vertex_count,
        triangle_offset,
        triangle_count,
        material_offset,
        material_count,
        flags_offset,
        nodes,
        vertices,
        triangles,
        material_indices,
        flags,
    })
}

/// Restore a compound from a byte buffer. (b3ConvertBytesToCompound)
pub fn convert_bytes_to_compound(bytes: &[u8]) -> Option<CompoundData> {
    if bytes.len() < COMPOUND_DATA_SIZE {
        return None;
    }

    let version = read_u64(bytes, 0);
    if version != COMPOUND_VERSION {
        return None;
    }

    let byte_count = read_i32(bytes, 8);
    if byte_count < COMPOUND_DATA_SIZE as i32 {
        return None;
    }
    if bytes.len() != byte_count as usize {
        return None;
    }

    let node_offset = read_i32(bytes, 12);
    if node_offset <= 0 {
        return None;
    }

    // Tree header starts at offset 16
    let tree_version = read_u64(bytes, 16);
    let root = read_i32(bytes, 16 + 16);
    let node_count = read_i32(bytes, 16 + 20);
    let node_capacity = read_i32(bytes, 16 + 24);
    let proxy_count = read_i32(bytes, 16 + 28);
    let free_list = read_i32(bytes, 16 + 32);
    let rebuild_capacity = read_i32(bytes, 16 + 72);

    let material_offset = read_i32(bytes, 16 + DYNAMIC_TREE_SIZE);
    let material_count = read_i32(bytes, 16 + DYNAMIC_TREE_SIZE + 4);
    let capsule_offset = read_i32(bytes, 16 + DYNAMIC_TREE_SIZE + 8);
    let capsule_count = read_i32(bytes, 16 + DYNAMIC_TREE_SIZE + 12);
    let hull_offset = read_i32(bytes, 16 + DYNAMIC_TREE_SIZE + 16);
    let hull_count = read_i32(bytes, 16 + DYNAMIC_TREE_SIZE + 20);
    let shared_hull_count = read_i32(bytes, 16 + DYNAMIC_TREE_SIZE + 24);
    let mesh_offset = read_i32(bytes, 16 + DYNAMIC_TREE_SIZE + 28);
    let mesh_count = read_i32(bytes, 16 + DYNAMIC_TREE_SIZE + 32);
    let shared_mesh_count = read_i32(bytes, 16 + DYNAMIC_TREE_SIZE + 36);
    let sphere_offset = read_i32(bytes, 16 + DYNAMIC_TREE_SIZE + 40);
    let sphere_count = read_i32(bytes, 16 + DYNAMIC_TREE_SIZE + 44);

    let mut nodes = Vec::with_capacity(node_capacity as usize);
    for i in 0..node_capacity as usize {
        let o = node_offset as usize + i * TREE_NODE_SIZE;
        nodes.push(read_tree_node(bytes, o));
    }

    let mut tree = DynamicTree::new(0);
    tree.version = if tree_version != 0 {
        tree_version
    } else {
        DYNAMIC_TREE_VERSION
    };
    tree.nodes = nodes;
    tree.root = root;
    tree.node_count = node_count;
    tree.free_list = free_list;
    tree.proxy_count = proxy_count;
    tree.rebuild_capacity = rebuild_capacity;

    let mut materials = Vec::with_capacity(material_count as usize);
    for i in 0..material_count as usize {
        let o = material_offset as usize + i * SURFACE_MATERIAL_SIZE;
        materials.push(SurfaceMaterial::from_bytes(&bytes[o..o + SURFACE_MATERIAL_SIZE]));
    }

    let mut capsules = Vec::with_capacity(capsule_count as usize);
    for i in 0..capsule_count as usize {
        let o = capsule_offset as usize + i * COMPOUND_CONVEX_SIZE;
        capsules.push(CompoundCapsule {
            capsule: crate::geometry::Capsule {
                center1: read_vec3(bytes, o),
                center2: read_vec3(bytes, o + 12),
                radius: read_f32(bytes, o + 24),
            },
            material_index: read_i32(bytes, o + 28),
        });
    }

    let mut hull_instances = Vec::with_capacity(hull_count as usize);
    for i in 0..hull_count as usize {
        let o = hull_offset as usize + i * HULL_INSTANCE_SIZE;
        hull_instances.push(HullInstance {
            transform: read_transform(bytes, o),
            shared_index: 0, // fixed up below
            material_index: read_u32(bytes, o + 32),
            hull_offset: read_u32(bytes, o + 28),
        });
    }

    // Collect unique hull offsets in first-seen order to rebuild shared_hulls
    let mut shared_hulls = Vec::new();
    let mut offset_to_shared: Vec<(u32, usize)> = Vec::new();
    for inst in &mut hull_instances {
        let off = inst.hull_offset;
        if let Some((_, idx)) = offset_to_shared.iter().find(|(o, _)| *o == off) {
            inst.shared_index = *idx as u32;
        } else {
            let idx = shared_hulls.len();
            let hull_bytes = &bytes[off as usize..];
            // byte_count is at offset 8 of the hull header
            let hull_byte_count = read_i32(hull_bytes, 8) as usize;
            let hull = read_hull_data(&hull_bytes[..hull_byte_count])?;
            shared_hulls.push(hull);
            offset_to_shared.push((off, idx));
            inst.shared_index = idx as u32;
        }
    }
    debug_assert_eq!(shared_hulls.len() as i32, shared_hull_count);

    let mut mesh_instances = Vec::with_capacity(mesh_count as usize);
    for i in 0..mesh_count as usize {
        let o = mesh_offset as usize + i * MESH_INSTANCE_SIZE;
        let mut material_indices = [0u32; MAX_COMPOUND_MESH_MATERIALS];
        for j in 0..MAX_COMPOUND_MESH_MATERIALS {
            material_indices[j] = read_u32(bytes, o + 44 + j * 4);
        }
        mesh_instances.push(MeshInstance {
            transform: read_transform(bytes, o),
            scale: read_vec3(bytes, o + 28),
            shared_index: 0,
            material_indices,
            mesh_offset: read_u32(bytes, o + 40),
        });
    }

    let mut shared_meshes = Vec::new();
    let mut mesh_offset_to_shared: Vec<(u32, usize)> = Vec::new();
    for inst in &mut mesh_instances {
        let off = inst.mesh_offset;
        if let Some((_, idx)) = mesh_offset_to_shared.iter().find(|(o, _)| *o == off) {
            inst.shared_index = *idx as u32;
        } else {
            let idx = shared_meshes.len();
            let mesh_bytes = &bytes[off as usize..];
            let mesh_byte_count = read_i32(mesh_bytes, 8) as usize;
            let mesh = read_mesh_data(&mesh_bytes[..mesh_byte_count])?;
            shared_meshes.push(mesh);
            mesh_offset_to_shared.push((off, idx));
            inst.shared_index = idx as u32;
        }
    }
    debug_assert_eq!(shared_meshes.len() as i32, shared_mesh_count);

    let mut spheres = Vec::with_capacity(sphere_count as usize);
    for i in 0..sphere_count as usize {
        let o = sphere_offset as usize + i * COMPOUND_CONVEX_SIZE;
        spheres.push(CompoundSphere {
            sphere: crate::geometry::Sphere {
                center: read_vec3(bytes, o),
                radius: read_f32(bytes, o + 12),
            },
            material_index: read_i32(bytes, o + 16),
        });
    }

    Some(CompoundData {
        version,
        byte_count,
        node_offset,
        tree,
        material_offset,
        material_count,
        capsule_offset,
        capsule_count,
        hull_offset,
        hull_count,
        shared_hull_count,
        mesh_offset,
        mesh_count,
        shared_mesh_count,
        sphere_offset,
        sphere_count,
        materials,
        capsules,
        hull_instances,
        shared_hulls,
        mesh_instances,
        shared_meshes,
        spheres,
    })
}

// Silence unused import warnings for constants used only in docs/debug.
#[allow(dead_code)]
fn _keep() {
    let _ = (
        TRANSFORM_IDENTITY,
        QUAT_IDENTITY,
        VEC3_ZERO,
        VEC3_ONE,
        MAT3_ZERO,
    );
}
