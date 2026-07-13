//! Geometry get/set for sphere, capsule, and hull. (shape.c Set*/Get*)
//!
//! Changing geometry destroys contacts and rebuilds the broad-phase proxy.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::lifecycle::{
    compute_shape_margin, destroy_shape_allocation_for_shape_change, get_shape,
};
use super::mutators::reset_proxy;
use super::ShapeGeometry;
use crate::geometry::{Capsule, Sphere};
use crate::hull::{is_valid_hull, HullData};
use crate::id::ShapeId;
use crate::math_functions::{is_valid_vec3, safe_scale, Vec3};
use crate::mesh::{is_valid_mesh, MeshData};
use crate::world::World;
use std::rc::Rc;

/// (b3Shape_GetSphere)
pub fn shape_get_sphere(world: &World, shape_id: ShapeId) -> Sphere {
    let index = get_shape(world, shape_id);
    match &world.shapes[index as usize].geometry {
        ShapeGeometry::Sphere(sphere) => *sphere,
        _ => {
            debug_assert!(false, "shape is not a sphere");
            Sphere::default()
        }
    }
}

/// (b3Shape_GetCapsule)
pub fn shape_get_capsule(world: &World, shape_id: ShapeId) -> Capsule {
    let index = get_shape(world, shape_id);
    match &world.shapes[index as usize].geometry {
        ShapeGeometry::Capsule(capsule) => *capsule,
        _ => {
            debug_assert!(false, "shape is not a capsule");
            Capsule::default()
        }
    }
}

/// (b3Shape_SetSphere)
pub fn shape_set_sphere(world: &mut World, shape_id: ShapeId, sphere: &Sphere) {
    crate::recording::with_recording(world, |rec| {
        rec.write_shape_set_sphere(shape_id, *sphere);
    });
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.locked = true;

    let index = get_shape(world, shape_id);
    destroy_shape_allocation_for_shape_change(world, index);

    {
        let shape = &mut world.shapes[index as usize];
        shape.geometry = ShapeGeometry::Sphere(*sphere);
        shape.aabb_margin = compute_shape_margin(shape);
    }

    // Need to wake bodies so they can react to the shape change.
    let wake_bodies = true;
    let destroy_proxy = true;
    reset_proxy(world, index, wake_bodies, destroy_proxy);

    world.locked = false;
}

/// (b3Shape_SetCapsule)
pub fn shape_set_capsule(world: &mut World, shape_id: ShapeId, capsule: &Capsule) {
    crate::recording::with_recording(world, |rec| {
        rec.write_shape_set_capsule(shape_id, *capsule);
    });
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.locked = true;

    let index = get_shape(world, shape_id);
    destroy_shape_allocation_for_shape_change(world, index);

    {
        let shape = &mut world.shapes[index as usize];
        shape.geometry = ShapeGeometry::Capsule(*capsule);
        shape.aabb_margin = compute_shape_margin(shape);
    }

    let wake_bodies = true;
    let destroy_proxy = true;
    reset_proxy(world, index, wake_bodies, destroy_proxy);

    world.locked = false;
}

/// (b3Shape_SetMesh)
///
/// Retypes the shape to a mesh, swapping in `mesh` (an owned clone, as
/// [`create_mesh_shape`](crate::shape::create_mesh_shape) does — C keeps a
/// borrowed pointer) at the sanitized `scale`, then destroys the shape's
/// contacts and rebuilds the broad-phase proxy. Like C, this does no mass
/// update: mesh shapes carry no mass, and the C source only calls `b3ResetProxy`.
pub fn shape_set_mesh(world: &mut World, shape_id: ShapeId, mesh: &MeshData, scale: Vec3) {
    debug_assert!(is_valid_vec3(scale));
    debug_assert!(is_valid_mesh(Some(mesh)));
    debug_assert!(mesh.hash != 0);

    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.locked = true;

    let index = get_shape(world, shape_id);
    destroy_shape_allocation_for_shape_change(world, index);

    {
        let shape = &mut world.shapes[index as usize];
        shape.geometry = ShapeGeometry::Mesh {
            data: mesh.clone(),
            scale: safe_scale(scale),
        };
        shape.aabb_margin = compute_shape_margin(shape);
    }

    // Need to wake bodies so they can react to the shape change.
    let wake_bodies = true;
    let destroy_proxy = true;
    reset_proxy(world, index, wake_bodies, destroy_proxy);

    world.locked = false;
}

/// (b3Shape_SetHull)
///
/// Acquires the new hull before releasing the old so `hull` may safely alias
/// the shape's current shared data.
pub fn shape_set_hull(world: &mut World, shape_id: ShapeId, hull: &HullData) {
    debug_assert!(is_valid_hull(hull));
    debug_assert!(hull.hash != 0);

    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.locked = true;

    let index = get_shape(world, shape_id);

    // Acquire the new hull before releasing the old so the input may safely
    // alias the shape's current shared data.
    let data = world.hull_database.add(hull);

    // Same shared hull — avoid destroying contacts and recreating the proxy.
    if let ShapeGeometry::Hull(existing) = &world.shapes[index as usize].geometry {
        if Rc::ptr_eq(existing, &data) {
            // Drop the extra Rc from add(); do not release from the database.
            drop(data);
            world.locked = false;
            return;
        }
    }

    destroy_shape_allocation_for_shape_change(world, index);

    {
        let shape = &mut world.shapes[index as usize];
        shape.geometry = ShapeGeometry::Hull(data);
        shape.aabb_margin = compute_shape_margin(shape);
    }

    let wake_bodies = true;
    let destroy_proxy = true;
    reset_proxy(world, index, wake_bodies, destroy_proxy);

    world.locked = false;
}
