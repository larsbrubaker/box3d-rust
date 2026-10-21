//! Compound shape types and accessors.
//!
//! Maps C's compound group in `include/box3d/types.h` and the getter APIs in
//! `compound.c` / `collision.h`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::dynamic_tree::{TreeNode, TreeView};
use crate::geometry::{Capsule, ShapeType, Sphere, SurfaceMaterial};
use crate::hull::{HullData, HULL_VERSION};
use crate::math_functions::{Aabb, Transform, Vec3, TRANSFORM_IDENTITY, VEC3_ONE, VEC3_ZERO};
use crate::mesh::{Mesh, MeshData, MESH_VERSION};

/// The baked compound version depends on the mesh and hull versions.
/// (B3_COMPOUND_VERSION)
pub const COMPOUND_VERSION: u64 = 0x9E4B_17D3_A25C_68F1u64 ^ MESH_VERSION ^ HULL_VERSION;

/// Size of the C `b3CompoundData` header: the 64-bit version, byteCount, the 24 byte
/// `b3AABB` bounds, then 15 ints (treeHeight, nodeOffset, nodeCount, and the material,
/// capsule, hull, mesh, and sphere offsets and counts). C asserts
/// `sizeof( b3CompoundData ) == sizeof( uint64_t ) + sizeof( b3AABB ) + 16 * sizeof( int )`.
pub const COMPOUND_DATA_SIZE: usize = 96;

/// Size of the C `b3HullInstance`: transform, hullOffset, then the `uint16_t`
/// materialIndex and padding.
pub const HULL_INSTANCE_SIZE: usize = 36;

/// Size of the C `b3MeshInstance`: transform, scale, meshOffset, materialOffset,
/// materialCount.
pub const MESH_INSTANCE_SIZE: usize = 52;

/// Size of the C `b3CompoundCapsule`: the 28 byte `b3Capsule` plus the `uint16_t`
/// materialIndex and padding.
pub const COMPOUND_CAPSULE_SIZE: usize = 32;

/// Size of the C `b3CompoundSphere`: the 16 byte `b3Sphere` plus the `uint16_t`
/// materialIndex and padding.
pub const COMPOUND_SPHERE_SIZE: usize = 20;

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

    /// Index to a shared material.
    pub material_index: u16,

    /// Padding for determinism.
    pub padding: u16,
}

/// A hull that lives in a compound. (b3CompoundHull)
#[derive(Debug, Clone, Copy)]
pub struct CompoundHull<'a> {
    pub hull: &'a HullData,
    pub transform: Transform,

    /// Index to a shared material.
    pub material_index: u16,

    /// Padding for determinism.
    pub padding: u16,
}

/// A mesh with non-uniform scale that lives in a compound. (b3CompoundMesh)
#[derive(Debug, Clone, Copy)]
pub struct CompoundMesh<'a> {
    pub mesh_data: &'a MeshData,
    pub transform: Transform,
    pub scale: Vec3,

    /// This is used to access the surface material from [`get_compound_materials`].
    /// Requires an extra level of indirection.
    /// `material_index = material_indices[triangle.material_index]`
    pub material_indices: &'a [u16],

    /// The number of materials. 1 for convex hapes.
    pub material_count: i32,
}

/// A sphere that lives in a compound. (b3CompoundSphere)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CompoundSphere {
    pub sphere: Sphere,

    /// Index to a shared material.
    pub material_index: u16,

    /// Padding for determinism.
    pub padding: u16,
}

/// Internal hull instance stored in the compound (offset resolved at create).
#[derive(Debug, Clone, Copy)]
pub(crate) struct HullInstance {
    pub transform: Transform,
    /// Index into [`CompoundData::shared_hulls`].
    pub shared_index: u32,
    pub material_index: u16,
    pub padding: u16,
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
    /// Byte offset of this instance's run of `uint16_t` material indices in the serialized
    /// blob's packed mesh material map (C `materialOffset`).
    pub material_offset: u32,
    pub material_count: u32,
    /// Index of the same run in [`CompoundData::mesh_material_map`]. C reaches the run by
    /// adding `material_offset` to the blob address; the port keeps the map as an owned
    /// array, so it stores the element index alongside the byte offset.
    pub material_start: u32,
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

    /// Material indices. Index 0 is used for convex shapes.
    pub material_indices: &'a [u16],

    /// The number of materials. This is one for convex shapes. For
    /// meshes it is determined by the maximum triangle material index.
    pub material_count: i32,

    pub shape_type: ShapeType,
}

/// The data for a baked compound shape. (b3CompoundData)
///
/// In C the compound is a single flat, pointer-free blob addressed by byte offsets. The port
/// keeps owned arrays instead and retains the offsets so [`CompoundData::to_bytes`] /
/// [`validate_compound`](crate::compound::validate_compound) reproduce the contiguous layout.
#[derive(Debug, Clone)]
pub struct CompoundData {
    pub version: u64,
    /// The total number of bytes for this compound.
    pub byte_count: i32,
    /// Bounds of the tree.
    pub bounds: Aabb,
    /// Tree height for diagnostics.
    pub tree_height: i32,
    /// Offset of the tree node array in bytes from the struct address.
    pub node_offset: i32,
    /// The number of tree nodes.
    pub node_count: i32,
    /// The tree nodes (C: `node_count` nodes at `node_offset` in the blob). Leaves store the
    /// child index in `shapeIndex`; the baked tree carries no proxies.
    pub nodes: Vec<TreeNode>,
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
    /// Packed `uint16_t` compound material indices for all mesh children, stored in the
    /// blob right after the mesh instance array.
    pub(crate) mesh_material_map: Vec<u16>,
    pub shared_meshes: Vec<MeshData>,
    pub spheres: Vec<CompoundSphere>,
}

impl Default for CompoundData {
    fn default() -> Self {
        Self {
            version: COMPOUND_VERSION,
            byte_count: 0,
            bounds: Aabb {
                lower_bound: VEC3_ZERO,
                upper_bound: VEC3_ZERO,
            },
            tree_height: 0,
            node_offset: 0,
            node_count: 0,
            nodes: Vec::new(),
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
            mesh_material_map: Vec::new(),
            shared_meshes: Vec::new(),
            spheres: Vec::new(),
        }
    }
}

/// The compound tree as a view without proxies. (b3GetCompoundTreeView)
#[inline]
pub(crate) fn get_compound_tree_view(compound: &CompoundData) -> TreeView<'_> {
    TreeView {
        nodes: &compound.nodes,
        proxies: None,
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
            padding: 0,
        };
    }
    let inst = &compound.hull_instances[index as usize];
    CompoundHull {
        hull: &compound.shared_hulls[inst.shared_index as usize],
        transform: inst.transform,
        material_index: inst.material_index,
        padding: 0,
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
            material_indices: &[],
            material_count: 0,
        };
    }
    let inst = &compound.mesh_instances[index as usize];
    let start = inst.material_start as usize;
    let count = inst.material_count as usize;
    CompoundMesh {
        mesh_data: &compound.shared_meshes[inst.shared_index as usize],
        transform: inst.transform,
        scale: inst.scale,
        material_indices: &compound.mesh_material_map[start..start + count],
        material_count: inst.material_count as i32,
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
///
/// The returned material indices reference the compound's own storage, like the C
/// pointers into the blob.
pub fn get_compound_child(compound: &CompoundData, mut child_index: i32) -> ChildShape<'_> {
    // Capsule?
    if 0 <= child_index && child_index < compound.capsule_count {
        debug_assert!(compound.capsule_offset > 0);
        let capsule = &compound.capsules[child_index as usize];
        return ChildShape {
            geometry: ChildGeometry::Capsule(capsule.capsule),
            transform: TRANSFORM_IDENTITY,
            material_indices: core::slice::from_ref(&capsule.material_index),
            material_count: 1,
            shape_type: ShapeType::Capsule,
        };
    }
    child_index -= compound.capsule_count;

    // Hull?
    if 0 <= child_index && child_index < compound.hull_count {
        debug_assert!(compound.hull_offset > 0);
        let inst = &compound.hull_instances[child_index as usize];
        return ChildShape {
            geometry: ChildGeometry::Hull(&compound.shared_hulls[inst.shared_index as usize]),
            transform: inst.transform,
            material_indices: core::slice::from_ref(&inst.material_index),
            material_count: 1,
            shape_type: ShapeType::Hull,
        };
    }
    child_index -= compound.hull_count;

    // Mesh?
    if 0 <= child_index && child_index < compound.mesh_count {
        let compound_mesh = get_compound_mesh(compound, child_index);

        return ChildShape {
            geometry: ChildGeometry::Mesh(Mesh {
                data: compound_mesh.mesh_data,
                scale: compound_mesh.scale,
            }),
            transform: compound_mesh.transform,
            material_indices: compound_mesh.material_indices,
            material_count: compound_mesh.material_count,
            shape_type: ShapeType::Mesh,
        };
    }
    child_index -= compound.mesh_count;

    debug_assert!(0 <= child_index && child_index < compound.sphere_count);

    // Sphere?
    debug_assert!(compound.sphere_offset > 0);
    let sphere = &compound.spheres[child_index as usize];
    ChildShape {
        geometry: ChildGeometry::Sphere(sphere.sphere),
        transform: TRANSFORM_IDENTITY,
        material_indices: core::slice::from_ref(&sphere.material_index),
        material_count: 1,
        shape_type: ShapeType::Sphere,
    }
}
