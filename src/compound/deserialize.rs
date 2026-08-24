//! Compound blob deserialization (convert from bytes).
//!
//! Port of `b3ConvertBytesToCompound`, split from `serialize.rs` to satisfy the
//! file-length limit.
//!
//! The hull and mesh blobs nested inside a compound buffer are byte-for-byte copies of the
//! standalone blobs: `b3CreateCompound` memcpy's `hullData->byteCount` / `meshData->byteCount`
//! bytes into the compound buffer (compound.c lines 587 and 606). C then reaches them by
//! casting a pointer into the blob; the Rust port slices them out with [`sub_blob`] and
//! parses them with the standalone converters, so the two readers cannot drift apart.
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
use crate::hull::{convert_bytes_to_hull, HULL_BYTE_COUNT_OFFSET, HULL_DATA_SIZE};
use crate::math_functions::{Aabb, Quat, Transform, Vec3};
use crate::mesh::{convert_bytes_to_mesh, MESH_BYTE_COUNT_OFFSET, MESH_DATA_SIZE};

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

/// Validate one trailing-section offset against the blob and return it as an index.
///
/// Divergence from C: `b3ConvertBytesToCompound` casts the stored offsets straight to
/// pointers and trusts the blob. The Rust port validates instead, so a corrupt
/// recording fails the conversion rather than wrapping the index arithmetic, which
/// panics on the overflow in debug builds and on the resulting slice bounds check in
/// release builds. Well-formed blobs always pass, so the happy path is unchanged.
fn section_offset(offset: i32, count: i32, stride: usize, len: usize) -> Option<usize> {
    if offset < 0 || count < 0 {
        return None;
    }
    let start = offset as usize;
    if start.checked_add((count as usize).checked_mul(stride)?)? > len {
        return None;
    }
    Some(start)
}

/// Slice a nested hull/mesh blob out of the compound buffer.
///
/// Divergence from C: the C reader offsets a pointer and trusts the nested `byteCount`.
/// The Rust port bounds-checks both, so a corrupt blob fails the conversion instead of
/// panicking on the slice. The returned slice is exactly `byteCount` long, which is what
/// `convert_bytes_to_hull` / `convert_bytes_to_mesh` require of a standalone blob.
/// `byte_count_offset` / `header_size` differ between the hull and mesh headers, so the
/// caller supplies them (b3HullData keeps `byteCount` last, b3MeshData keeps it after the
/// 64-bit hash).
fn sub_blob(
    bytes: &[u8],
    offset: u32,
    byte_count_offset: usize,
    header_size: usize,
) -> Option<&[u8]> {
    let start = offset as usize;
    if start.checked_add(header_size)? > bytes.len() {
        return None;
    }
    let byte_count = read_i32(bytes, start + byte_count_offset);
    if byte_count < 0 {
        return None;
    }
    let end = start.checked_add(byte_count as usize)?;
    if end > bytes.len() {
        return None;
    }
    Some(&bytes[start..end])
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

    let noff = section_offset(node_offset, node_capacity, TREE_NODE_SIZE, bytes.len())?;
    let mut nodes = Vec::with_capacity(node_capacity as usize);
    for i in 0..node_capacity as usize {
        let o = noff + i * TREE_NODE_SIZE;
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

    let moff = section_offset(
        material_offset,
        material_count,
        SURFACE_MATERIAL_SIZE,
        bytes.len(),
    )?;
    let mut materials = Vec::with_capacity(material_count as usize);
    for i in 0..material_count as usize {
        let o = moff + i * SURFACE_MATERIAL_SIZE;
        materials.push(SurfaceMaterial::from_bytes(
            &bytes[o..o + SURFACE_MATERIAL_SIZE],
        ));
    }

    let coff = section_offset(
        capsule_offset,
        capsule_count,
        COMPOUND_CONVEX_SIZE,
        bytes.len(),
    )?;
    let mut capsules = Vec::with_capacity(capsule_count as usize);
    for i in 0..capsule_count as usize {
        let o = coff + i * COMPOUND_CONVEX_SIZE;
        capsules.push(CompoundCapsule {
            capsule: crate::geometry::Capsule {
                center1: read_vec3(bytes, o),
                center2: read_vec3(bytes, o + 12),
                radius: read_f32(bytes, o + 24),
            },
            material_index: read_i32(bytes, o + 28),
        });
    }

    let hoff = section_offset(hull_offset, hull_count, HULL_INSTANCE_SIZE, bytes.len())?;
    let mut hull_instances = Vec::with_capacity(hull_count as usize);
    for i in 0..hull_count as usize {
        let o = hoff + i * HULL_INSTANCE_SIZE;
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
            let hull = convert_bytes_to_hull(sub_blob(
                bytes,
                off,
                HULL_BYTE_COUNT_OFFSET,
                HULL_DATA_SIZE,
            )?)?;
            shared_hulls.push(hull);
            offset_to_shared.push((off, idx));
            inst.shared_index = idx as u32;
        }
    }
    // C asserts this; the Rust port turns it into a rejection so a corrupt recording
    // fails the conversion instead of tripping a debug assert.
    if shared_hulls.len() as i32 != shared_hull_count {
        return None;
    }

    let mioff = section_offset(mesh_offset, mesh_count, MESH_INSTANCE_SIZE, bytes.len())?;
    let mut mesh_instances = Vec::with_capacity(mesh_count as usize);
    for i in 0..mesh_count as usize {
        let o = mioff + i * MESH_INSTANCE_SIZE;
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
            let mesh = convert_bytes_to_mesh(sub_blob(
                bytes,
                off,
                MESH_BYTE_COUNT_OFFSET,
                MESH_DATA_SIZE,
            )?)?;
            shared_meshes.push(mesh);
            mesh_offset_to_shared.push((off, idx));
            inst.shared_index = idx as u32;
        }
    }
    // C asserts this; the Rust port turns it into a rejection so a corrupt recording
    // fails the conversion instead of tripping a debug assert.
    if shared_meshes.len() as i32 != shared_mesh_count {
        return None;
    }

    let soff = section_offset(
        sphere_offset,
        sphere_count,
        COMPOUND_CONVEX_SIZE,
        bytes.len(),
    )?;
    let mut spheres = Vec::with_capacity(sphere_count as usize);
    for i in 0..sphere_count as usize {
        let o = soff + i * COMPOUND_CONVEX_SIZE;
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
