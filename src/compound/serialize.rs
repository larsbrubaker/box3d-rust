//! Compound blob serialization (convert to/from bytes).
//!
//! Port of `b3ConvertCompoundToBytes` / `b3ConvertBytesToCompound`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{
    CompoundCapsule, CompoundData, CompoundSphere, HullInstance, MeshInstance, COMPOUND_DATA_SIZE,
    DYNAMIC_TREE_SIZE, MAX_COMPOUND_MESH_MATERIALS,
};
use crate::dynamic_tree::{DynamicTree, TreeNode, ALLOCATED_NODE, LEAF_NODE};
use crate::math_functions::{Aabb, Quat, Transform, Vec3};

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
            let offset = self
                .hull_instances
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
            let offset = self
                .mesh_instances
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
