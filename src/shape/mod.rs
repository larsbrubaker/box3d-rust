// Port of the shape module from box3d-cpp-reference/src/shape.h + shape.c.
//
// Split to satisfy the 800-line file limit:
// - dispatch.rs  — per-shape-type dispatch (AABBs, mass, centroid, proxy)
// - lifecycle.rs — shape creation (margin, create_shape_internal, typed wrappers)
// - api.rs       — public create re-exports
//
// This file holds the data model and filter predicates.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::compound::CompoundData;
use crate::core::NULL_INDEX;
use crate::geometry::{Capsule, ShapeType, Sphere, SurfaceMaterial};
use crate::height_field::HeightFieldData;
use crate::hull::HullData;
use crate::math_functions::{Aabb, Vec3, BOUNDS3_EMPTY, VEC3_ONE, VEC3_ZERO};
use crate::mesh::MeshData;
use crate::types::{Filter, QueryFilter};
use std::rc::Rc;

/// Shape flag bits. (enum b3ShapeFlags)
pub mod shape_flags {
    pub const ENABLE_SENSOR_EVENTS: u8 = 0x01;
    pub const ENABLE_CONTACT_EVENTS: u8 = 0x02;
    pub const ENABLE_CUSTOM_FILTERING: u8 = 0x04;
    pub const ENABLE_HIT_EVENTS: u8 = 0x08;
    pub const ENABLE_PRE_SOLVE_EVENTS: u8 = 0x10;
    pub const ENLARGED_AABB: u8 = 0x20;
}

/// Concrete shape geometry. Maps to C's `type` tag + anonymous union.
#[derive(Debug, Clone)]
pub enum ShapeGeometry {
    Capsule(Capsule),
    Sphere(Sphere),
    /// Shared hull from the world hull database. (C: `const b3HullData* hull`)
    Hull(Rc<HullData>),
    /// Mesh data plus per-instance scale (C: `b3Mesh { data*, scale }`).
    Mesh { data: MeshData, scale: Vec3 },
    HeightField(HeightFieldData),
    Compound(CompoundData),
}

impl ShapeGeometry {
    /// The shape type tag. (C: shape->type)
    pub fn shape_type(&self) -> ShapeType {
        match self {
            ShapeGeometry::Capsule(_) => ShapeType::Capsule,
            ShapeGeometry::Sphere(_) => ShapeType::Sphere,
            ShapeGeometry::Hull(_) => ShapeType::Hull,
            ShapeGeometry::Mesh { .. } => ShapeType::Mesh,
            ShapeGeometry::HeightField(_) => ShapeType::Height,
            ShapeGeometry::Compound(_) => ShapeType::Compound,
        }
    }
}

impl Default for ShapeGeometry {
    fn default() -> Self {
        ShapeGeometry::Capsule(Capsule::default())
    }
}

/// Internal shape. (b3Shape)
///
/// A single-material shape keeps its material inline. Multi-material meshes and
/// compounds own a heap array in [`Self::materials`]. Reach materials the same
/// way for both via [`Shape::shape_materials`].
#[derive(Debug, Clone)]
pub struct Shape {
    pub id: i32,
    pub body_id: i32,
    pub prev_shape_id: i32,
    pub next_shape_id: i32,
    pub sensor_index: i32,
    pub proxy_key: i32,
    pub density: f32,
    pub explosion_scale: f32,
    pub aabb_margin: f32,

    pub aabb: Aabb,
    pub fat_aabb: Aabb,
    pub local_centroid: Vec3,

    /// Inline material for single-material shapes.
    pub material: SurfaceMaterial,
    /// Heap materials for multi-material meshes/compounds. Empty means use
    /// [`Self::material`] as a one-element array (C: `materials == NULL`).
    pub materials: Vec<SurfaceMaterial>,

    pub filter: Filter,
    pub user_data: u64,
    /// Application user shape pointer (C: `void* userShape`).
    pub user_shape: u64,

    pub name_id: u32,
    pub generation: u16,

    /// shape_flags bits
    pub flags: u8,

    /// The shape geometry (C: type tag + union).
    pub geometry: ShapeGeometry,
}

impl Shape {
    /// The shape type tag. (C: shape->type)
    pub fn shape_type(&self) -> ShapeType {
        self.geometry.shape_type()
    }

    /// Material count. Single-material shapes report 1. (C: materialCount)
    pub fn material_count(&self) -> i32 {
        if self.materials.is_empty() {
            1
        } else {
            self.materials.len() as i32
        }
    }

    /// Materials slice. Single-material shapes present the inline material as a
    /// one-element view via this helper's return of a temporary isn't possible
    /// without an enum — callers should use [`Self::get_material`] or check
    /// `materials` emptiness. (b3GetShapeMaterials)
    pub fn shape_materials(&self) -> &[SurfaceMaterial] {
        if self.materials.is_empty() {
            core::slice::from_ref(&self.material)
        } else {
            &self.materials
        }
    }

    /// Material at index. (index into the materials array)
    pub fn get_material(&self, index: i32) -> &SurfaceMaterial {
        let mats = self.shape_materials();
        &mats[index as usize]
    }

    /// User material id for a child/triangle of this shape.
    /// (b3GetShapeUserMaterialId)
    ///
    /// C early-returns 0 when materialCount == 0; the Rust shape always
    /// presents at least the inline material, so that guard has no
    /// equivalent here.
    pub fn get_shape_user_material_id(&self, child_index: i32, triangle_index: i32) -> u64 {
        use crate::compound::{get_compound_child, ChildGeometry, MAX_COMPOUND_MESH_MATERIALS};
        use crate::height_field::get_height_field_material;
        use crate::math_functions::clamp_int;
        use crate::mesh::get_mesh_material_indices;

        let mut material_index = 0i32;
        match &self.geometry {
            ShapeGeometry::Mesh { data, .. } => {
                let indices = get_mesh_material_indices(data);
                if !indices.is_empty() {
                    material_index = indices[triangle_index as usize] as i32;
                }
            }
            ShapeGeometry::HeightField(height_field) => {
                material_index = get_height_field_material(height_field, triangle_index);
            }
            ShapeGeometry::Compound(compound) => {
                let child = get_compound_child(compound, child_index);
                if let ChildGeometry::Mesh(mesh) = child.geometry {
                    let indices = get_mesh_material_indices(mesh.data);
                    let mesh_material_index = if !indices.is_empty() {
                        indices[triangle_index as usize] as i32
                    } else {
                        0
                    };
                    let mesh_material_index = clamp_int(
                        mesh_material_index,
                        0,
                        MAX_COMPOUND_MESH_MATERIALS as i32 - 1,
                    );
                    material_index = child.material_indices[mesh_material_index as usize];
                } else {
                    material_index = child.material_indices[0];
                }
            }
            _ => {}
        }

        let material_index = clamp_int(material_index, 0, self.material_count() - 1);
        self.shape_materials()[material_index as usize].user_material_id
    }
}

impl Default for Shape {
    fn default() -> Self {
        Shape {
            id: NULL_INDEX,
            body_id: NULL_INDEX,
            prev_shape_id: NULL_INDEX,
            next_shape_id: NULL_INDEX,
            sensor_index: NULL_INDEX,
            proxy_key: NULL_INDEX,
            density: 0.0,
            explosion_scale: 1.0,
            aabb_margin: 0.0,
            aabb: BOUNDS3_EMPTY,
            fat_aabb: BOUNDS3_EMPTY,
            local_centroid: VEC3_ZERO,
            material: SurfaceMaterial::default(),
            materials: Vec::new(),
            filter: Filter::default(),
            user_data: 0,
            user_shape: 0,
            name_id: 0,
            generation: 0,
            flags: 0,
            geometry: ShapeGeometry::default(),
        }
    }
}

/// (static inline b3ShouldShapesCollide)
pub fn should_shapes_collide(filter_a: Filter, filter_b: Filter) -> bool {
    if filter_a.group_index == filter_b.group_index && filter_a.group_index != 0 {
        return filter_a.group_index > 0;
    }

    (filter_a.mask_bits & filter_b.category_bits) != 0
        && (filter_a.category_bits & filter_b.mask_bits) != 0
}

/// (static inline b3ShouldQueryCollide)
pub fn should_query_collide(shape_filter: &Filter, query_filter: &QueryFilter) -> bool {
    (shape_filter.category_bits & query_filter.mask_bits) != 0
        && (shape_filter.mask_bits & query_filter.category_bits) != 0
}

/// Convenience: unit-scale mesh geometry. (b3Mesh with identity scale)
pub fn mesh_geometry(data: MeshData) -> ShapeGeometry {
    ShapeGeometry::Mesh {
        data,
        scale: VEC3_ONE,
    }
}

mod api;
mod dispatch;
pub(crate) mod lifecycle;

pub use api::*;
pub use dispatch::*;

