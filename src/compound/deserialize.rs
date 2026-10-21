//! Compound blob validation and deserialization.
//!
//! Port of `b3ValidateCompound` (upstream 5d83df8, which replaced
//! `b3ConvertBytesToCompound`), split from `serialize.rs` to satisfy the file-length limit.
//! C validates the header and then uses the bytes in place as the compound. The port runs the
//! same header checks in the same order, then parses the blob into an owned [`CompoundData`].
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
    COMPOUND_CAPSULE_SIZE, COMPOUND_DATA_SIZE, COMPOUND_SPHERE_SIZE, COMPOUND_VERSION,
    HULL_INSTANCE_SIZE, MESH_INSTANCE_SIZE,
};
use crate::dynamic_tree::{read_tree_node, TREE_NODE_SIZE, TREE_STACK_SIZE};

/// The deepest compound tree a validated blob may hold: half the traversal stack. See the
/// node checks in [`validate_compound`].
const MAX_COMPOUND_TREE_DEPTH: usize = TREE_STACK_SIZE / 2;
use crate::geometry::{SurfaceMaterial, SURFACE_MATERIAL_SIZE};
use crate::hull::{convert_bytes_to_hull, HULL_BYTE_COUNT_OFFSET, HULL_DATA_SIZE};
use crate::math_functions::{Aabb, Quat, Transform, Vec3};
use crate::mesh::{convert_bytes_to_mesh, MESH_BYTE_COUNT_OFFSET, MESH_DATA_SIZE};

fn read_u64(buf: &[u8], o: usize) -> u64 {
    u64::from_le_bytes(buf[o..o + 8].try_into().unwrap())
}
fn read_u16(buf: &[u8], o: usize) -> u16 {
    u16::from_le_bytes(buf[o..o + 2].try_into().unwrap())
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

/// Validate one trailing-section offset against the blob and return it as an index.
///
/// Divergence from C: `b3ValidateCompound` checks only the header and the tree node range,
/// and C then reaches the other sections by adding the stored offsets to the blob address.
/// The Rust port validates them too, so a corrupt recording fails the conversion rather than wrapping the index arithmetic, which
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

/// Validate serialized compound bytes and restore the compound. Returns `None` if invalid.
/// (b3ValidateCompound)
///
/// C returns the input address cast to a compound and never modifies the bytes. The port
/// takes the bytes by shared reference, so they cannot be modified either, and returns an
/// owned compound parsed from them. C's checks come first, in C's order: the size, the
/// version, the byte count, the node offset (inside the buffer past the header and 8 byte
/// aligned), the node count (at least the root pair), and the node array end (in 64-bit
/// arithmetic so a huge count cannot overflow).
pub fn validate_compound(bytes: &[u8]) -> Option<CompoundData> {
    // C takes the byte count as an int; a buffer too large for one cannot match a stored
    // byteCount either.
    let byte_count_arg = i32::try_from(bytes.len()).ok()?;
    if byte_count_arg < COMPOUND_DATA_SIZE as i32 {
        return None;
    }

    let version = read_u64(bytes, 0);
    if version != COMPOUND_VERSION {
        return None;
    }

    let byte_count = read_i32(bytes, 8);
    if byte_count != byte_count_arg {
        return None;
    }

    let bounds = Aabb {
        lower_bound: read_vec3(bytes, 12),
        upper_bound: read_vec3(bytes, 24),
    };
    let tree_height = read_i32(bytes, 36);
    let node_offset = read_i32(bytes, 40);
    let node_count = read_i32(bytes, 44);

    if node_offset < COMPOUND_DATA_SIZE as i32 || (node_offset & 7) != 0 {
        return None;
    }

    if node_count < 2 {
        return None;
    }

    let node_end = node_offset as i64 + node_count as i64 * TREE_NODE_SIZE as i64;
    if node_end > byte_count as i64 {
        return None;
    }

    let material_offset = read_i32(bytes, 48);
    let material_count = read_i32(bytes, 52);
    let capsule_offset = read_i32(bytes, 56);
    let capsule_count = read_i32(bytes, 60);
    let hull_offset = read_i32(bytes, 64);
    let hull_count = read_i32(bytes, 68);
    let shared_hull_count = read_i32(bytes, 72);
    let mesh_offset = read_i32(bytes, 76);
    let mesh_count = read_i32(bytes, 80);
    let shared_mesh_count = read_i32(bytes, 84);
    let sphere_offset = read_i32(bytes, 88);
    let sphere_count = read_i32(bytes, 92);

    // C queries the node array in place through b3GetCompoundTreeView; the port reads it out
    // of the blob.
    let mut nodes = Vec::with_capacity(node_count as usize);
    for i in 0..node_count as usize {
        let o = node_offset as usize + i * TREE_NODE_SIZE;
        nodes.push(read_tree_node(bytes, o));
    }

    // Divergence from C: C trusts the node contents. The port requires every internal node to
    // point at a sibling pair after it inside the node array (the full rebuild is DFS ordered,
    // so children always follow their parent) and every node with the leaf bit to name a child
    // of the compound. That includes empty nodes: they carry the leaf bit too, and a traversal
    // reports one as a leaf if its bounds overlap. Real empty nodes have shape index 0 and a
    // compound always has a child, so they pass.
    //
    // It also bounds the tree depth. The query and cast stacks hold TREE_STACK_SIZE entries
    // and grow by at most one entry per level, so a validated tree may be at most
    // MAX_COMPOUND_TREE_DEPTH (half the stack) levels deep. The median split rebuild C bakes
    // compounds with is balanced, so a real tree is about log2 of the child count deep (at
    // most 2^CHILD_POWER children), far below the bound. Depths are computed in one forward
    // pass, since every sibling pair comes after its parent.
    //
    // A corrupt tree then fails the conversion instead of panicking on a node or child index,
    // looping, or overflowing the traversal stack during a query.
    let child_count =
        capsule_count as i64 + hull_count as i64 + mesh_count as i64 + sphere_count as i64;
    let mut depths = vec![0usize; nodes.len()];
    for (i, node) in nodes.iter().enumerate() {
        if node.is_leaf() {
            let child_index = node.shape_index() as i64;
            if child_index < 0 || child_index >= child_count {
                return None;
            }
        } else {
            let pair = node.left_child() as usize;
            if pair <= i || pair + 1 >= nodes.len() {
                return None;
            }
            let depth = depths[i] + 1;
            if depth > MAX_COMPOUND_TREE_DEPTH {
                return None;
            }
            depths[pair] = depths[pair].max(depth);
            depths[pair + 1] = depths[pair + 1].max(depth);
        }
    }

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
        COMPOUND_CAPSULE_SIZE,
        bytes.len(),
    )?;
    let mut capsules = Vec::with_capacity(capsule_count as usize);
    for i in 0..capsule_count as usize {
        let o = coff + i * COMPOUND_CAPSULE_SIZE;
        capsules.push(CompoundCapsule {
            capsule: crate::geometry::Capsule {
                center1: read_vec3(bytes, o),
                center2: read_vec3(bytes, o + 12),
                radius: read_f32(bytes, o + 24),
            },
            material_index: read_u16(bytes, o + 28),
            padding: read_u16(bytes, o + 30),
        });
    }
    // Divergence from C: reject a material index outside the compound material table (see
    // the mesh material map below).
    if capsules
        .iter()
        .any(|c| c.material_index as i32 >= material_count)
    {
        return None;
    }

    let hoff = section_offset(hull_offset, hull_count, HULL_INSTANCE_SIZE, bytes.len())?;
    let mut hull_instances = Vec::with_capacity(hull_count as usize);
    for i in 0..hull_count as usize {
        let o = hoff + i * HULL_INSTANCE_SIZE;
        hull_instances.push(HullInstance {
            transform: read_transform(bytes, o),
            shared_index: 0, // fixed up below
            material_index: read_u16(bytes, o + 32),
            padding: read_u16(bytes, o + 34),
            hull_offset: read_u32(bytes, o + 28),
        });
    }

    // Collect unique hull offsets in first-seen order to rebuild shared_hulls
    if hull_instances
        .iter()
        .any(|h| h.material_index as i32 >= material_count)
    {
        return None;
    }

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
        mesh_instances.push(MeshInstance {
            transform: read_transform(bytes, o),
            scale: read_vec3(bytes, o + 28),
            shared_index: 0,
            material_offset: read_u32(bytes, o + 44),
            material_count: read_u32(bytes, o + 48),
            material_start: 0, // fixed up below
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

    // Packed mesh material map. C points each instance at its run by adding materialOffset
    // to the blob address; the port copies the runs into an owned array.
    //
    // Divergence from C: C trusts the blob. The port requires each run to lie inside the map
    // section, which starts after the mesh instance array and ends at the first shared mesh
    // blob. It also requires each run to cover every triangle material of its mesh (and at
    // least the mesh material count) and each entry to index the compound material table.
    // Those indices are used as slice indexes by the casts, mover, contacts, and
    // get_shape_user_material_id, so a corrupt blob fails the conversion here instead of
    // panicking there. Well-formed blobs always pass.
    let map_section_start = mioff + mesh_count as usize * MESH_INSTANCE_SIZE;
    let map_section_end = mesh_instances
        .iter()
        .map(|inst| inst.mesh_offset as usize)
        .min()
        .unwrap_or(map_section_start);
    let mut mesh_material_map: Vec<u16> = Vec::new();
    for inst in &mut mesh_instances {
        let count = inst.material_count as usize;
        let start = inst.material_offset as usize;
        let end = start.checked_add(count.checked_mul(core::mem::size_of::<u16>())?)?;
        if start < map_section_start || end > map_section_end || end > bytes.len() {
            return None;
        }

        let mesh = &shared_meshes[inst.shared_index as usize];
        if (count as u64) < mesh.material_count as u64 {
            return None;
        }
        if mesh.material_indices.iter().any(|&m| m as usize >= count) {
            return None;
        }

        inst.material_start = mesh_material_map.len() as u32;
        for k in 0..count {
            let index = read_u16(bytes, start + k * core::mem::size_of::<u16>());
            if index as i32 >= material_count {
                return None;
            }
            mesh_material_map.push(index);
        }
    }

    let soff = section_offset(
        sphere_offset,
        sphere_count,
        COMPOUND_SPHERE_SIZE,
        bytes.len(),
    )?;
    let mut spheres = Vec::with_capacity(sphere_count as usize);
    for i in 0..sphere_count as usize {
        let o = soff + i * COMPOUND_SPHERE_SIZE;
        spheres.push(CompoundSphere {
            sphere: crate::geometry::Sphere {
                center: read_vec3(bytes, o),
                radius: read_f32(bytes, o + 12),
            },
            material_index: read_u16(bytes, o + 16),
            padding: read_u16(bytes, o + 18),
        });
    }
    if spheres
        .iter()
        .any(|s| s.material_index as i32 >= material_count)
    {
        return None;
    }

    Some(CompoundData {
        version,
        byte_count,
        bounds,
        tree_height,
        node_offset,
        node_count,
        nodes,
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
        mesh_material_map,
        shared_meshes,
        spheres,
    })
}
