//! Compound creation and destruction.
//!
//! Port of `b3CreateCompound` / `b3DestroyCompound` from compound.c.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{
    CompoundCapsule, CompoundData, CompoundDef, CompoundSphere, HullInstance, MeshInstance,
    COMPOUND_CONVEX_SIZE, COMPOUND_DATA_SIZE, COMPOUND_VERSION, HULL_INSTANCE_SIZE,
    MAX_COMPOUND_MESH_MATERIALS, MESH_INSTANCE_SIZE, TREE_NODE_SIZE,
};
use crate::constants::MAX_CHILD_SHAPES;
use crate::core::NULL_INDEX;
use crate::dynamic_tree::DynamicTree;
use crate::geometry::{
    compute_capsule_aabb, compute_sphere_aabb, SurfaceMaterial, SURFACE_MATERIAL_SIZE,
};
use crate::hull::{compare_hull_data, compute_hull_aabb, HullData};
use crate::math_functions::{align_up8, TRANSFORM_IDENTITY};
use crate::mesh::{compute_mesh_aabb, MeshData};

struct SharedHull {
    hull: HullData,
    hull_offset: i32,
}

struct SharedMesh {
    mesh_data: MeshData,
    mesh_offset: i32,
}

fn materials_equal(a: &SurfaceMaterial, b: &SurfaceMaterial) -> bool {
    a.to_bytes() == b.to_bytes()
}

fn get_or_insert_material(materials: &mut Vec<SurfaceMaterial>, mat: &SurfaceMaterial) -> i32 {
    if let Some(i) = materials.iter().position(|m| materials_equal(m, mat)) {
        return i as i32;
    }
    let index = materials.len() as i32;
    materials.push(*mat);
    index
}

fn meshes_equal(a: &MeshData, b: &MeshData) -> bool {
    if core::ptr::eq(a, b) {
        return true;
    }
    if a.byte_count != b.byte_count {
        return false;
    }
    a.to_bytes() == b.to_bytes()
}

/// Create a baked compound shape. (b3CreateCompound)
pub fn create_compound(def: &CompoundDef<'_>) -> Option<CompoundData> {
    let capsule_count = def.capsules.len() as i32;
    let hull_count = def.hulls.len() as i32;
    let mesh_count = def.meshes.len() as i32;
    let sphere_count = def.spheres.len() as i32;

    let convex_count = capsule_count + hull_count + sphere_count;
    let shape_count = convex_count + mesh_count;

    if shape_count >= MAX_CHILD_SHAPES {
        debug_assert!(false);
        return None;
    }

    let mut tree = DynamicTree::new(shape_count);
    let mut child_index = 0i32;

    let mut capsule_instances = vec![CompoundCapsule::default(); capsule_count as usize];
    let mut hull_instances = vec![
        HullInstance {
            transform: TRANSFORM_IDENTITY,
            shared_index: 0,
            material_index: 0,
            hull_offset: 0,
        };
        hull_count as usize
    ];
    let mut mesh_instances = vec![
        MeshInstance {
            transform: TRANSFORM_IDENTITY,
            scale: crate::math_functions::VEC3_ONE,
            shared_index: 0,
            material_indices: [0; MAX_COMPOUND_MESH_MATERIALS],
            mesh_offset: 0,
        };
        mesh_count as usize
    ];
    let mut sphere_instances = vec![CompoundSphere::default(); sphere_count as usize];

    let mut material_capacity = convex_count;
    for mesh in def.meshes {
        debug_assert!(!mesh.materials.is_empty());
        material_capacity += mesh.materials.len() as i32;
    }

    let mut materials: Vec<SurfaceMaterial> = Vec::with_capacity(material_capacity.max(0) as usize);

    for (i, capsule_def) in def.capsules.iter().enumerate() {
        capsule_instances[i].capsule = capsule_def.capsule;
        let material_index = get_or_insert_material(&mut materials, &capsule_def.material);
        capsule_instances[i].material_index = material_index;

        let aabb = compute_capsule_aabb(&capsule_def.capsule, TRANSFORM_IDENTITY);
        tree.create_proxy(aabb, !0u64, child_index as u64);
        child_index += 1;
    }

    let mut shared_hulls: Vec<SharedHull> = Vec::with_capacity(hull_count as usize);

    for (i, hull_def) in def.hulls.iter().enumerate() {
        let hull = hull_def.hull;
        let aabb = compute_hull_aabb(hull, hull_def.transform);
        tree.create_proxy(aabb, !0u64, child_index as u64);
        child_index += 1;

        let material_index = get_or_insert_material(&mut materials, &hull_def.material);
        hull_instances[i].material_index = material_index as u32;
        hull_instances[i].transform = hull_def.transform;

        let shared_index = if let Some(idx) = shared_hulls
            .iter()
            .position(|s| compare_hull_data(&s.hull, hull))
        {
            idx
        } else {
            let idx = shared_hulls.len();
            shared_hulls.push(SharedHull {
                hull: hull.clone(),
                hull_offset: NULL_INDEX,
            });
            idx
        };
        hull_instances[i].shared_index = shared_index as u32;
        // Temporarily store shared index in hull_offset (C does the same).
        hull_instances[i].hull_offset = shared_index as u32;
    }

    let mut shared_meshes: Vec<SharedMesh> = Vec::with_capacity(mesh_count as usize);

    for (i, mesh_def) in def.meshes.iter().enumerate() {
        let mesh_data = mesh_def.mesh_data;
        let aabb = compute_mesh_aabb(mesh_data, mesh_def.transform, mesh_def.scale);
        tree.create_proxy(aabb, !0u64, child_index as u64);
        child_index += 1;

        debug_assert_eq!(mesh_data.material_count as usize, mesh_def.materials.len());

        for (j, mat) in mesh_def.materials.iter().enumerate() {
            let material_index = get_or_insert_material(&mut materials, mat);
            mesh_instances[i].material_indices[j] = material_index as u32;
        }

        let shared_index = if let Some(idx) = shared_meshes
            .iter()
            .position(|s| meshes_equal(&s.mesh_data, mesh_data))
        {
            idx
        } else {
            let idx = shared_meshes.len();
            shared_meshes.push(SharedMesh {
                mesh_data: mesh_data.clone(),
                mesh_offset: NULL_INDEX,
            });
            idx
        };

        mesh_instances[i].transform = mesh_def.transform;
        mesh_instances[i].scale = mesh_def.scale;
        mesh_instances[i].shared_index = shared_index as u32;
        mesh_instances[i].mesh_offset = shared_index as u32;
    }

    for (i, sphere_def) in def.spheres.iter().enumerate() {
        sphere_instances[i].sphere = sphere_def.sphere;
        let material_index = get_or_insert_material(&mut materials, &sphere_def.material);
        sphere_instances[i].material_index = material_index;

        let aabb = compute_sphere_aabb(&sphere_def.sphere, TRANSFORM_IDENTITY);
        tree.create_proxy(aabb, !0u64, child_index as u64);
        child_index += 1;
    }

    let material_count = materials.len() as i32;
    debug_assert!(material_count <= material_capacity);
    debug_assert!(tree.node_count() > 0);

    tree.rebuild(true);

    let shared_hull_count = shared_hulls.len() as i32;
    let shared_mesh_count = shared_meshes.len() as i32;

    let mut byte_count = align_up8(COMPOUND_DATA_SIZE);

    let node_offset = byte_count as i32;
    byte_count += align_up8(tree.node_capacity() as usize * TREE_NODE_SIZE);

    let material_offset = byte_count as i32;
    byte_count += align_up8(material_count as usize * SURFACE_MATERIAL_SIZE);

    let capsule_offset = byte_count as i32;
    byte_count += align_up8(capsule_count as usize * COMPOUND_CONVEX_SIZE);

    let hull_array_offset = byte_count as i32;
    byte_count += align_up8(hull_count as usize * HULL_INSTANCE_SIZE);

    for shared in &mut shared_hulls {
        shared.hull_offset = byte_count as i32;
        byte_count += align_up8(shared.hull.byte_count as usize);
    }

    let mesh_array_offset = byte_count as i32;
    byte_count += align_up8(mesh_count as usize * MESH_INSTANCE_SIZE);

    for shared in &mut shared_meshes {
        shared.mesh_offset = byte_count as i32;
        byte_count += align_up8(shared.mesh_data.byte_count as usize);
    }

    let sphere_offset = byte_count as i32;
    byte_count += align_up8(sphere_count as usize * COMPOUND_CONVEX_SIZE);

    // Fix up hull/mesh instance offsets to absolute blob offsets (C does this
    // before memcpy into the compound).
    for inst in &mut hull_instances {
        let shared_index = inst.hull_offset as usize;
        debug_assert!(shared_index < shared_hulls.len());
        inst.hull_offset = shared_hulls[shared_index].hull_offset as u32;
    }
    for inst in &mut mesh_instances {
        let shared_index = inst.mesh_offset as usize;
        debug_assert!(shared_index < shared_meshes.len());
        inst.mesh_offset = shared_meshes[shared_index].mesh_offset as u32;
    }

    // Immutable tree: scrub rebuild scratch and force free_list like C.
    tree.free_list = 0;
    tree.leaf_indices.clear();
    tree.leaf_centers.clear();
    tree.rebuild_capacity = 0;

    debug_assert!(material_count > 0);

    Some(CompoundData {
        version: COMPOUND_VERSION,
        byte_count: byte_count as i32,
        node_offset,
        tree,
        material_offset,
        material_count,
        capsule_offset,
        capsule_count,
        hull_offset: hull_array_offset,
        hull_count,
        shared_hull_count,
        mesh_offset: mesh_array_offset,
        mesh_count,
        shared_mesh_count,
        sphere_offset,
        sphere_count,
        materials,
        capsules: capsule_instances,
        hull_instances,
        shared_hulls: shared_hulls.into_iter().map(|s| s.hull).collect(),
        mesh_instances,
        shared_meshes: shared_meshes.into_iter().map(|s| s.mesh_data).collect(),
        spheres: sphere_instances,
    })
}

/// Destroy a compound shape. (b3DestroyCompound)
pub fn destroy_compound(_compound: CompoundData) {}
