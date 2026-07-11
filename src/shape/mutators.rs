//! Shape filter and material public API from shape.c.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::dispatch::update_shape_aabbs;
use super::lifecycle::get_shape;
use crate::body::get_body_transform_quick;
use crate::broad_phase::proxy_type;
use crate::core::NULL_INDEX;
use crate::geometry::{ShapeType, SurfaceMaterial};
use crate::id::ShapeId;
use crate::math_functions::{is_valid_float, is_valid_vec3};
use crate::types::Filter;
use crate::world::World;

/// Destroy contacts on a shape and refresh its broad-phase proxy.
/// (static b3ResetProxy)
pub(crate) fn reset_proxy(
    world: &mut World,
    shape_index: i32,
    wake_bodies: bool,
    destroy_proxy: bool,
) {
    let body_id = world.shapes[shape_index as usize].body_id;
    let shape_id = world.shapes[shape_index as usize].id;

    // Destroy all contacts associated with this shape.
    let mut contact_key = world.bodies[body_id as usize].head_contact_key;
    while contact_key != NULL_INDEX {
        let contact_id = contact_key >> 1;
        let edge_index = contact_key & 1;
        let next_key = world.contacts[contact_id as usize].edges[edge_index as usize].next_key;
        let shape_id_a = world.contacts[contact_id as usize].shape_id_a;
        let shape_id_b = world.contacts[contact_id as usize].shape_id_b;
        contact_key = next_key;

        if shape_id_a == shape_id || shape_id_b == shape_id {
            crate::contact::destroy_contact(world, contact_id, wake_bodies);
        }
    }

    let transform = get_body_transform_quick(world, &world.bodies[body_id as usize]);
    let proxy_key = world.shapes[shape_index as usize].proxy_key;

    if proxy_key != NULL_INDEX {
        let proxy_type_ = proxy_type(proxy_key);
        update_shape_aabbs(
            &mut world.shapes[shape_index as usize],
            transform,
            proxy_type_,
        );

        if destroy_proxy {
            let fat_aabb = world.shapes[shape_index as usize].fat_aabb;
            let category_bits = world.shapes[shape_index as usize].filter.category_bits;
            world.broad_phase.destroy_proxy(proxy_key);

            let force_pair_creation = true;
            world.shapes[shape_index as usize].proxy_key = world.broad_phase.create_proxy(
                proxy_type_,
                fat_aabb,
                category_bits,
                shape_id,
                force_pair_creation,
            );
        } else {
            let fat_aabb = world.shapes[shape_index as usize].fat_aabb;
            world.broad_phase.move_proxy(proxy_key, fat_aabb);
        }
    } else {
        let proxy_type_ = world.bodies[body_id as usize].type_;
        update_shape_aabbs(
            &mut world.shapes[shape_index as usize],
            transform,
            proxy_type_,
        );
    }

    world.validate_solver_sets();
}

/// (b3Shape_SetFriction)
pub fn shape_set_friction(world: &mut World, shape_id: ShapeId, friction: f32) {
    debug_assert!(is_valid_float(friction) && friction >= 0.0);
    let index = get_shape(world, shape_id);
    let shape = &mut world.shapes[index as usize];
    debug_assert!(shape.shape_type() != ShapeType::Compound);
    shape.shape_materials_mut()[0].friction = friction;
}

/// (b3Shape_GetFriction)
pub fn shape_get_friction(world: &World, shape_id: ShapeId) -> f32 {
    let index = get_shape(world, shape_id);
    world.shapes[index as usize].shape_materials()[0].friction
}

/// (b3Shape_SetRestitution)
pub fn shape_set_restitution(world: &mut World, shape_id: ShapeId, restitution: f32) {
    debug_assert!(is_valid_float(restitution) && restitution >= 0.0);
    let index = get_shape(world, shape_id);
    let shape = &mut world.shapes[index as usize];
    debug_assert!(shape.shape_type() != ShapeType::Compound);
    shape.shape_materials_mut()[0].restitution = restitution;
}

/// (b3Shape_GetRestitution)
pub fn shape_get_restitution(world: &World, shape_id: ShapeId) -> f32 {
    let index = get_shape(world, shape_id);
    world.shapes[index as usize].shape_materials()[0].restitution
}

/// (b3Shape_SetSurfaceMaterial)
pub fn shape_set_surface_material(
    world: &mut World,
    shape_id: ShapeId,
    surface_material: SurfaceMaterial,
) {
    debug_assert!(is_valid_float(surface_material.friction) && surface_material.friction >= 0.0);
    debug_assert!(
        is_valid_float(surface_material.restitution) && surface_material.restitution >= 0.0
    );
    debug_assert!(
        is_valid_float(surface_material.rolling_resistance)
            && surface_material.rolling_resistance >= 0.0
    );
    debug_assert!(is_valid_vec3(surface_material.tangent_velocity));

    let index = get_shape(world, shape_id);
    let shape = &mut world.shapes[index as usize];
    debug_assert!(shape.shape_type() != ShapeType::Compound);
    shape.shape_materials_mut()[0] = surface_material;
}

/// (b3Shape_GetSurfaceMaterial)
pub fn shape_get_surface_material(world: &World, shape_id: ShapeId) -> SurfaceMaterial {
    let index = get_shape(world, shape_id);
    world.shapes[index as usize].shape_materials()[0]
}

/// (b3Shape_GetMeshMaterialCount)
pub fn shape_get_mesh_material_count(world: &World, shape_id: ShapeId) -> i32 {
    let index = get_shape(world, shape_id);
    world.shapes[index as usize].material_count()
}

/// (b3Shape_SetMeshMaterial)
pub fn shape_set_mesh_material(
    world: &mut World,
    shape_id: ShapeId,
    surface_material: SurfaceMaterial,
    material_index: i32,
) {
    debug_assert!(is_valid_float(surface_material.friction) && surface_material.friction >= 0.0);
    debug_assert!(
        is_valid_float(surface_material.restitution) && surface_material.restitution >= 0.0
    );
    debug_assert!(
        is_valid_float(surface_material.rolling_resistance)
            && surface_material.rolling_resistance >= 0.0
    );
    debug_assert!(is_valid_vec3(surface_material.tangent_velocity));

    let index = get_shape(world, shape_id);
    let shape = &mut world.shapes[index as usize];
    debug_assert!(0 <= material_index && material_index < shape.material_count());
    debug_assert!(shape.shape_type() != ShapeType::Compound);
    shape.shape_materials_mut()[material_index as usize] = surface_material;
}

/// (b3Shape_GetMeshSurfaceMaterial)
pub fn shape_get_mesh_surface_material(
    world: &World,
    shape_id: ShapeId,
    material_index: i32,
) -> SurfaceMaterial {
    let index = get_shape(world, shape_id);
    let shape = &world.shapes[index as usize];
    debug_assert!(0 <= material_index && material_index < shape.material_count());
    shape.shape_materials()[material_index as usize]
}

/// (b3Shape_GetFilter)
pub fn shape_get_filter(world: &World, shape_id: ShapeId) -> Filter {
    let index = get_shape(world, shape_id);
    world.shapes[index as usize].filter
}

/// (b3Shape_SetFilter)
///
/// When `invoke_contacts` is true, contacts on the shape are destroyed and the
/// broad-phase proxy is refreshed so pairs are recomputed on the next step.
pub fn shape_set_filter(
    world: &mut World,
    shape_id: ShapeId,
    filter: Filter,
    invoke_contacts: bool,
) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let index = get_shape(world, shape_id);
    let shape = &mut world.shapes[index as usize];
    if filter.mask_bits == shape.filter.mask_bits
        && filter.category_bits == shape.filter.category_bits
        && filter.group_index == shape.filter.group_index
    {
        return;
    }

    shape.filter = filter;

    if invoke_contacts {
        world.locked = true;
        let wake_bodies = true;
        // C compares after assignment (`filter.categoryBits == shape->filter.categoryBits`),
        // so destroy_proxy is always true here. Match that exactly.
        let destroy_proxy =
            filter.category_bits == world.shapes[index as usize].filter.category_bits;
        reset_proxy(world, index, wake_bodies, destroy_proxy);
        world.locked = false;
    }

    // Note: this does not immediately update sensor overlaps. Sensor overlaps
    // are updated the next time step (same as C).
}
