//! Compound blob serialization.
//!
//! In C a compound is a single flat blob with no pointers, so writing `byteCount` bytes from
//! the compound address serializes it (upstream 5d83df8 removed `b3ConvertCompoundToBytes`).
//! The port keeps owned arrays, so [`CompoundData::to_bytes`] lays them out at their recorded
//! offsets to reproduce those bytes exactly.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{
    CompoundCapsule, CompoundData, CompoundSphere, HullInstance, MeshInstance, COMPOUND_DATA_SIZE,
};
use crate::dynamic_tree::write_tree_node;
use crate::math_functions::{Quat, Transform, Vec3};

fn write_u64(buf: &mut Vec<u8>, v: u64) {
    buf.extend_from_slice(&v.to_le_bytes());
}
fn write_u16(buf: &mut Vec<u8>, v: u16) {
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
fn pad_to(buf: &mut Vec<u8>, len: usize) {
    if buf.len() < len {
        buf.resize(len, 0);
    }
}

fn write_compound_header(buf: &mut Vec<u8>, c: &CompoundData) {
    write_u64(buf, c.version);
    write_i32(buf, c.byte_count);
    write_vec3(buf, c.bounds.lower_bound);
    write_vec3(buf, c.bounds.upper_bound);
    write_i32(buf, c.tree_height);
    write_i32(buf, c.node_offset);
    write_i32(buf, c.node_count);
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
    write_u16(buf, c.material_index);
    write_u16(buf, c.padding);
}

fn write_sphere(buf: &mut Vec<u8>, s: &CompoundSphere) {
    write_vec3(buf, s.sphere.center);
    write_f32(buf, s.sphere.radius);
    write_u16(buf, s.material_index);
    write_u16(buf, s.padding);
}

fn write_hull_instance(buf: &mut Vec<u8>, h: &HullInstance) {
    write_transform(buf, h.transform);
    write_u32(buf, h.hull_offset);
    write_u16(buf, h.material_index);
    write_u16(buf, h.padding);
}

fn write_mesh_instance(buf: &mut Vec<u8>, m: &MeshInstance) {
    write_transform(buf, m.transform);
    write_vec3(buf, m.scale);
    write_u32(buf, m.mesh_offset);
    write_u32(buf, m.material_offset);
    write_u32(buf, m.material_count);
}

/// Write one mesh instance's run of the packed material map at its recorded blob offset.
/// Writing in place (rather than appending) keeps the output faithful to the recorded
/// offsets regardless of instance order.
fn write_mesh_materials(buf: &mut Vec<u8>, m: &MeshInstance, map: &[u16]) {
    let start = m.material_start as usize;
    let count = m.material_count as usize;
    let offset = m.material_offset as usize;
    let end = offset + count * core::mem::size_of::<u16>();
    pad_to(buf, end);
    for (k, index) in map[start..start + count].iter().enumerate() {
        let o = offset + k * core::mem::size_of::<u16>();
        buf[o..o + 2].copy_from_slice(&index.to_le_bytes());
    }
}

impl CompoundData {
    /// Serialize to the C contiguous blob layout: the `byteCount` bytes starting at the C
    /// compound address.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(self.byte_count as usize);
        write_compound_header(&mut buf, self);

        pad_to(&mut buf, self.node_offset as usize);
        for node in &self.nodes {
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
        // Packed mesh material map
        for inst in &self.mesh_instances {
            write_mesh_materials(&mut buf, inst, &self.mesh_material_map);
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
