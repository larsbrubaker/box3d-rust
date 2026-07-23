//! Compound shape types and accessors.
//!
//! Maps C's compound group in `include/box3d/types.h` and the getter APIs in
//! `compound.c` / `collision.h`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::dynamic_tree::{DynamicTree, DYNAMIC_TREE_VERSION};
use crate::geometry::{Capsule, ShapeType, Sphere, SurfaceMaterial};
use crate::hull::{HullData, HULL_VERSION};
use crate::math_functions::{Transform, Vec3, TRANSFORM_IDENTITY, VEC3_ONE};
use crate::mesh::{Mesh, MeshData, MESH_VERSION};

/// The baked compound version depends on the tree, mesh, and hull versions.
/// (B3_COMPOUND_VERSION)
pub const COMPOUND_VERSION: u64 =
    0x8307_78DB_0708_6EB4u64 ^ DYNAMIC_TREE_VERSION ^ MESH_VERSION ^ HULL_VERSION;

/// Meshes used in compounds have limited space for materials.
/// (B3_MAX_COMPOUND_MESH_MATERIALS)
pub const MAX_COMPOUND_MESH_MATERIALS: usize = 4;

/// Size of the C `b3CompoundData` header on 64-bit (MSVC/GCC).
pub const COMPOUND_DATA_SIZE: usize = 144;

/// Size of the C `b3DynamicTree` on 64-bit.
pub const DYNAMIC_TREE_SIZE: usize = 80;

/// Size of the C `b3TreeNode` (union layout).
pub const TREE_NODE_SIZE: usize = 48;

/// Size of the C `b3HullInstance`.
pub const HULL_INSTANCE_SIZE: usize = 36;

/// Size of the C `b3MeshInstance`.
pub const MESH_INSTANCE_SIZE: usize = 60;

/// Size of the C `b3CompoundCapsule` / `b3CompoundSphere`.
pub const COMPOUND_CONVEX_SIZE: usize = 32;

/// Definition for a capsule in a compound shape. (b3CompoundCapsuleDef)
#[derive(Debug, Clone, Copy)]
pub struct CompoundCapsuleDef {
    pub capsule: Capsule,
    pub material: SurfaceMaterial,
}

/// Definition for a convex hull in a compound shape. (b3CompoundHullDef)
#[derive(Debug, Clone, Copy)]
pub struct CompoundHullDef<'a> {
    pub hull: &'a HullData,
    pub transform: Transform,
    pub material: SurfaceMaterial,
}

/// Definition for a triangle mesh in a compound shape. (b3CompoundMeshDef)
#[derive(Debug, Clone, Copy)]
pub struct CompoundMeshDef<'a> {
    pub mesh_data: &'a MeshData,
    pub transform: Transform,
    pub scale: Vec3,
    pub materials: &'a [SurfaceMaterial],
}

/// Definition for a sphere in a compound shape. (b3CompoundSphereDef)
#[derive(Debug, Clone, Copy)]
pub struct CompoundSphereDef {
    pub sphere: Sphere,
    pub material: SurfaceMaterial,
}

/// Definition for creating a compound shape. (b3CompoundDef)
#[derive(Debug, Clone, Default)]
pub struct CompoundDef<'a> {
    pub capsules: &'a [CompoundCapsuleDef],
    pub hulls: &'a [CompoundHullDef<'a>],
    pub meshes: &'a [CompoundMeshDef<'a>],
    pub spheres: &'a [CompoundSphereDef],
}

/// A capsule that lives in a compound. (b3CompoundCapsule)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CompoundCapsule {
    pub capsule: Capsule,
    pub material_index: i32,
}

/// A hull that lives in a compound. (b3CompoundHull)
#[derive(Debug, Clone, Copy)]
pub struct CompoundHull<'a> {
    pub hull: &'a HullData,
    pub transform: Transform,
    pub material_index: i32,
}

/// A mesh with non-uniform scale that lives in a compound. (b3CompoundMesh)
#[derive(Debug, Clone, Copy)]
pub struct CompoundMesh<'a> {
    pub mesh_data: &'a MeshData,
    pub transform: Transform,
    pub scale: Vec3,
    pub material_indices: [i32; MAX_COMPOUND_MESH_MATERIALS],
}

/// A sphere that lives in a compound. (b3CompoundSphere)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CompoundSphere {
    pub sphere: Sphere,
    pub material_index: i32,
}

/// Internal hull instance stored in the compound (offset resolved at create).
#[derive(Debug, Clone, Copy)]
pub(crate) struct HullInstance {
    pub transform: Transform,
    /// Index into [`CompoundData::shared_hulls`].
    pub shared_index: u32,
    pub material_index: u32,
    /// Byte offset into the serialized blob (for to_bytes parity).
    pub hull_offset: u32,
}

/// Internal mesh instance stored in the compound.
#[derive(Debug, Clone, Copy)]
pub(crate) struct MeshInstance {
    pub transform: Transform,
    pub scale: Vec3,
    /// Index into [`CompoundData::shared_meshes`].
    pub shared_index: u32,
    pub material_indices: [u32; MAX_COMPOUND_MESH_MATERIALS],
    /// Byte offset into the serialized blob (for to_bytes parity).
    pub mesh_offset: u32,
}

/// Child geometry of a compound. (union half of b3ChildShape)
#[derive(Debug, Clone, Copy)]
pub enum ChildGeometry<'a> {
    Capsule(Capsule),
    Hull(&'a HullData),
    Mesh(Mesh<'a>),
    Sphere(Sphere),
}

/// Child shape of a compound. (b3ChildShape)
#[derive(Debug, Clone, Copy)]
pub struct ChildShape<'a> {
    pub geometry: ChildGeometry<'a>,
    pub transform: Transform,
    pub material_indices: [i32; MAX_COMPOUND_MESH_MATERIALS],
    pub shape_type: ShapeType,
}

/// The data for a baked compound shape. (b3CompoundData)
///
/// Owned arrays replace C's trailing-blob offsets. Offsets are retained so
/// [`CompoundData::to_bytes`] / [`from_bytes`](crate::compound::convert_bytes_to_compound)
/// reproduce the contiguous layout.
#[derive(Debug, Clone)]
pub struct CompoundData {
    pub version: u64,
    pub byte_count: i32,
    pub node_offset: i32,
    pub tree: DynamicTree,
    pub material_offset: i32,
    pub material_count: i32,
    pub capsule_offset: i32,
    pub capsule_count: i32,
    pub hull_offset: i32,
    pub hull_count: i32,
    pub shared_hull_count: i32,
    pub mesh_offset: i32,
    pub mesh_count: i32,
    pub shared_mesh_count: i32,
    pub sphere_offset: i32,
    pub sphere_count: i32,
    pub materials: Vec<SurfaceMaterial>,
    pub capsules: Vec<CompoundCapsule>,
    pub(crate) hull_instances: Vec<HullInstance>,
    pub shared_hulls: Vec<HullData>,
    pub(crate) mesh_instances: Vec<MeshInstance>,
    pub shared_meshes: Vec<MeshData>,
    pub spheres: Vec<CompoundSphere>,
}

impl Default for CompoundData {
    fn default() -> Self {
        Self {
            version: COMPOUND_VERSION,
            byte_count: 0,
            node_offset: 0,
            tree: DynamicTree::new(0),
            material_offset: 0,
            material_count: 0,
            capsule_offset: 0,
            capsule_count: 0,
            hull_offset: 0,
            hull_count: 0,
            shared_hull_count: 0,
            mesh_offset: 0,
            mesh_count: 0,
            shared_mesh_count: 0,
            sphere_offset: 0,
            sphere_count: 0,
            materials: Vec::new(),
            capsules: Vec::new(),
            hull_instances: Vec::new(),
            shared_hulls: Vec::new(),
            mesh_instances: Vec::new(),
            shared_meshes: Vec::new(),
            spheres: Vec::new(),
        }
    }
}

/// Compound materials. (b3GetCompoundMaterials)
pub fn get_compound_materials(compound: &CompoundData) -> &[SurfaceMaterial] {
    if compound.material_offset == 0 {
        return &[];
    }
    &compound.materials
}

/// Capsule at index. (b3GetCompoundCapsule)
pub fn get_compound_capsule(compound: &CompoundData, index: i32) -> CompoundCapsule {
    debug_assert!(0 <= index && index < compound.capsule_count && compound.capsule_offset > 0);
    if compound.capsule_offset == 0 {
        return CompoundCapsule::default();
    }
    compound.capsules[index as usize]
}

/// Hull at index. (b3GetCompoundHull)
pub fn get_compound_hull(compound: &CompoundData, index: i32) -> CompoundHull<'_> {
    debug_assert!(0 <= index && index < compound.hull_count && compound.hull_offset > 0);
    if compound.hull_offset == 0 {
        return CompoundHull {
            hull: &compound.shared_hulls[0],
            transform: TRANSFORM_IDENTITY,
            material_index: 0,
        };
    }
    let inst = &compound.hull_instances[index as usize];
    CompoundHull {
        hull: &compound.shared_hulls[inst.shared_index as usize],
        transform: inst.transform,
        material_index: inst.material_index as i32,
    }
}

/// Mesh at index. (b3GetCompoundMesh)
pub fn get_compound_mesh(compound: &CompoundData, index: i32) -> CompoundMesh<'_> {
    debug_assert!(0 <= index && index < compound.mesh_count && compound.mesh_offset > 0);
    if compound.mesh_offset == 0 {
        return CompoundMesh {
            mesh_data: &compound.shared_meshes[0],
            transform: TRANSFORM_IDENTITY,
            scale: VEC3_ONE,
            material_indices: [0; MAX_COMPOUND_MESH_MATERIALS],
        };
    }
    let inst = &compound.mesh_instances[index as usize];
    let mut material_indices = [0i32; MAX_COMPOUND_MESH_MATERIALS];
    for i in 0..MAX_COMPOUND_MESH_MATERIALS {
        material_indices[i] = inst.material_indices[i] as i32;
    }
    CompoundMesh {
        mesh_data: &compound.shared_meshes[inst.shared_index as usize],
        transform: inst.transform,
        scale: inst.scale,
        material_indices,
    }
}

/// Sphere at index. (b3GetCompoundSphere)
pub fn get_compound_sphere(compound: &CompoundData, index: i32) -> CompoundSphere {
    debug_assert!(0 <= index && index < compound.sphere_count && compound.sphere_offset > 0);
    if compound.sphere_offset == 0 {
        return CompoundSphere::default();
    }
    compound.spheres[index as usize]
}

/// Child shape by flat index (capsules → hulls → meshes → spheres).
/// (b3GetCompoundChild)
pub fn get_compound_child(compound: &CompoundData, mut child_index: i32) -> ChildShape<'_> {
    // Capsule?
    if 0 <= child_index && child_index < compound.capsule_count {
        let compound_capsule = get_compound_capsule(compound, child_index);
        return ChildShape {
            geometry: ChildGeometry::Capsule(compound_capsule.capsule),
            transform: TRANSFORM_IDENTITY,
            material_indices: [compound_capsule.material_index, 0, 0, 0],
            shape_type: ShapeType::Capsule,
        };
    }
    child_index -= compound.capsule_count;

    // Hull?
    if 0 <= child_index && child_index < compound.hull_count {
        let compound_hull = get_compound_hull(compound, child_index);
        return ChildShape {
            geometry: ChildGeometry::Hull(compound_hull.hull),
            transform: compound_hull.transform,
            material_indices: [compound_hull.material_index, 0, 0, 0],
            shape_type: ShapeType::Hull,
        };
    }
    child_index -= compound.hull_count;

    // Mesh?
    if 0 <= child_index && child_index < compound.mesh_count {
        let compound_mesh = get_compound_mesh(compound, child_index);
        debug_assert_eq!(MAX_COMPOUND_MESH_MATERIALS, 4);
        return ChildShape {
            geometry: ChildGeometry::Mesh(Mesh {
                data: compound_mesh.mesh_data,
                scale: compound_mesh.scale,
            }),
            transform: compound_mesh.transform,
            material_indices: compound_mesh.material_indices,
            shape_type: ShapeType::Mesh,
        };
    }
    child_index -= compound.mesh_count;

    debug_assert!(0 <= child_index && child_index < compound.sphere_count);

    let compound_sphere = get_compound_sphere(compound, child_index);
    ChildShape {
        geometry: ChildGeometry::Sphere(compound_sphere.sphere),
        transform: TRANSFORM_IDENTITY,
        material_indices: [compound_sphere.material_index, 0, 0, 0],
        shape_type: ShapeType::Sphere,
    }
}
