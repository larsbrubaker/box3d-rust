// Shape creation from shape.c.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::{
    create_shape_proxy, destroy_shape_proxy, get_shape_centroid, shape_flags, Shape, ShapeGeometry,
};
use crate::compound::{get_compound_materials, CompoundData};
use crate::constants::{linear_slop, max_aabb_margin, AABB_MARGIN_FRACTION, MAX_SHAPES};
use crate::core::{NULL_INDEX, SECRET_COOKIE};
use crate::geometry::{Capsule, ShapeType, Sphere};
use crate::height_field::HeightFieldData;
use crate::hull::{get_hull_points, HullData};
use crate::id::{ShapeId, NULL_SHAPE_ID};
use crate::math_functions::{
    distance, distance_squared, is_valid_float, lerp, min_float, safe_scale, Aabb, Vec3,
    WorldTransform, VEC3_ZERO,
};
use crate::mesh::{is_valid_mesh, MeshData};
use crate::solver_set::DISABLED_SET;
use crate::types::{BodyType, ShapeDef};
use crate::world::World;

/// AABB margin for the broad phase fat AABB, limited by shape size.
/// (static b3ComputeShapeMargin)
pub fn compute_shape_margin(shape: &Shape) -> f32 {
    let margin = match &shape.geometry {
        ShapeGeometry::Sphere(sphere) => sphere.radius,
        ShapeGeometry::Capsule(capsule) => {
            0.5 * distance(capsule.center2, capsule.center1) + capsule.radius
        }
        ShapeGeometry::Hull(hull) => {
            let points = get_hull_points(hull);
            let mut max_extent_sqr = 0.0f32;
            for point in points {
                let dist_sqr = distance_squared(*point, hull.center);
                max_extent_sqr = crate::math_functions::max_float(max_extent_sqr, dist_sqr);
            }
            max_extent_sqr.sqrt()
        }
        ShapeGeometry::Mesh { .. } | ShapeGeometry::HeightField(_) | ShapeGeometry::Compound(_) => {
            // Static-only shapes: return the cap so incidental use is generous.
            return max_aabb_margin();
        }
    };

    min_float(max_aabb_margin(), AABB_MARGIN_FRACTION * margin)
}

/// Create a shape on a body. Returns the raw shape id, or NULL_INDEX on failure.
/// (static b3CreateShapeInternal)
pub(crate) fn create_shape_internal(
    world: &mut World,
    body_index: i32,
    body_transform: WorldTransform,
    def: &ShapeDef,
    geometry: ShapeGeometry,
) -> i32 {
    let shape_type = geometry.shape_type();

    let shape_id = world.shape_id_pool.alloc_id();

    if shape_id == world.shapes.len() as i32 {
        world.shapes.push(Shape::default());
    } else {
        debug_assert!(world.shapes[shape_id as usize].id == NULL_INDEX);
    }

    let (body_raw_id, body_set_index, body_type, head_shape_id) = {
        let body = &world.bodies[body_index as usize];
        (body.id, body.set_index, body.type_, body.head_shape_id)
    };

    let name_id = world.names.add_name(&def.name);

    {
        let shape = &mut world.shapes[shape_id as usize];
        shape.geometry = geometry;

        shape.id = shape_id;
        shape.body_id = body_raw_id;
        shape.density = def.density;
        shape.explosion_scale = def.explosion_scale;
        shape.filter = def.filter;
        shape.user_data = def.user_data;
        shape.user_shape = 0;
        shape.flags = 0;
        if def.enable_sensor_events {
            shape.flags |= shape_flags::ENABLE_SENSOR_EVENTS;
        }
        if def.enable_contact_events {
            shape.flags |= shape_flags::ENABLE_CONTACT_EVENTS;
        }
        if def.enable_custom_filtering {
            shape.flags |= shape_flags::ENABLE_CUSTOM_FILTERING;
        }
        if def.enable_hit_events {
            shape.flags |= shape_flags::ENABLE_HIT_EVENTS;
        }
        if def.enable_pre_solve_events {
            shape.flags |= shape_flags::ENABLE_PRE_SOLVE_EVENTS;
        }
        if def.enable_speculative_contact {
            shape.flags |= shape_flags::ENABLE_SPECULATIVE;
        }
        shape.proxy_key = NULL_INDEX;
        shape.local_centroid = get_shape_centroid(shape);
        shape.aabb_margin = compute_shape_margin(shape);
        shape.aabb = Aabb {
            lower_bound: VEC3_ZERO,
            upper_bound: VEC3_ZERO,
        };
        shape.fat_aabb = Aabb {
            lower_bound: VEC3_ZERO,
            upper_bound: VEC3_ZERO,
        };
        shape.name_id = name_id;
        shape.generation = shape.generation.wrapping_add(1);

        if shape_type == ShapeType::Compound {
            // Own a copy of the compound materials so every shape frees its array
            // the same way. Compounds are few, so the copy is cheap and avoids
            // aliasing the geometry blob. (C: b3CreateShapeInternal)
            let mats = match &shape.geometry {
                ShapeGeometry::Compound(compound) => get_compound_materials(compound).to_vec(),
                _ => unreachable!(),
            };
            if mats.is_empty() {
                shape.material = def.base_material;
                shape.materials.clear();
            } else if mats.len() == 1 {
                shape.material = mats[0];
                shape.materials.clear();
            } else {
                shape.material = def.base_material;
                shape.materials = mats;
            }
        } else if def.materials.len() > 1 {
            shape.materials = def.materials.clone();
            shape.material = def.base_material;
        } else if def.materials.len() == 1 {
            shape.material = def.materials[0];
            shape.materials.clear();
        } else {
            shape.material = def.base_material;
            shape.materials.clear();
        }
    }

    if body_set_index != DISABLED_SET {
        let force_pair_creation = def.invoke_contact_creation && shape_type != ShapeType::Compound;
        let (shapes, broad_phase) = (&mut world.shapes, &mut world.broad_phase);
        create_shape_proxy(
            &mut shapes[shape_id as usize],
            broad_phase,
            body_type,
            body_transform,
            force_pair_creation,
        );
    }

    // Add to shape doubly linked list
    if head_shape_id != NULL_INDEX {
        world.shapes[head_shape_id as usize].prev_shape_id = shape_id;
    }

    world.shapes[shape_id as usize].prev_shape_id = NULL_INDEX;
    world.shapes[shape_id as usize].next_shape_id = head_shape_id;
    world.bodies[body_index as usize].head_shape_id = shape_id;
    world.bodies[body_index as usize].shape_count += 1;

    if def.is_sensor {
        world.shapes[shape_id as usize].sensor_index = world.sensors.len() as i32;
        world.sensors.push(crate::sensor::Sensor::new(shape_id));
    } else {
        world.shapes[shape_id as usize].sensor_index = NULL_INDEX;
    }

    world.validate_solver_sets();

    shape_id
}

/// (static b3CreateShape)
fn create_shape(
    world: &mut World,
    body_id: crate::id::BodyId,
    def: &ShapeDef,
    geometry: ShapeGeometry,
) -> ShapeId {
    use crate::body::{
        body_flags, get_body_full_id, get_body_transform_quick, sync_body_flags,
        update_body_mass_data,
    };

    debug_assert!(def.internal_value == SECRET_COOKIE);
    debug_assert!(is_valid_float(def.density) && def.density >= 0.0);
    debug_assert!(is_valid_float(def.base_material.friction) && def.base_material.friction >= 0.0);
    debug_assert!(
        is_valid_float(def.base_material.restitution) && def.base_material.restitution >= 0.0
    );

    debug_assert!(!world.locked);
    if world.locked {
        return NULL_SHAPE_ID;
    }

    if world.shapes.len() as i32 == MAX_SHAPES && world.shape_id_pool.free_array.is_empty() {
        debug_assert!(false);
        return NULL_SHAPE_ID;
    }

    let body_index = get_body_full_id(world, body_id);
    let shape_type = geometry.shape_type();

    if world.bodies[body_index as usize].type_ != BodyType::Static
        && (shape_type == ShapeType::Compound || shape_type == ShapeType::Height)
    {
        // Compound and height shapes must be on static bodies.
        return NULL_SHAPE_ID;
    }

    world.locked = true;

    let body_transform = get_body_transform_quick(world, &world.bodies[body_index as usize]);

    let shape_id = create_shape_internal(world, body_index, body_transform, def, geometry);

    if shape_id == NULL_INDEX {
        world.locked = false;
        return NULL_SHAPE_ID;
    }

    if def.update_body_mass {
        update_body_mass_data(world, body_index);
    } else if world.bodies[body_index as usize].flags & body_flags::DIRTY_MASS == 0 {
        world.bodies[body_index as usize].flags |= body_flags::DIRTY_MASS;
        sync_body_flags(world, body_index);
    }

    world.validate_solver_sets();

    let id = ShapeId {
        index1: shape_id + 1,
        world0: body_id.world0,
        generation: world.shapes[shape_id as usize].generation,
    };

    world.locked = false;

    id
}

/// (b3CreateSphereShape)
pub fn create_sphere_shape(
    world: &mut World,
    body_id: crate::id::BodyId,
    def: &ShapeDef,
    sphere: &Sphere,
) -> ShapeId {
    let shape_id = create_shape(world, body_id, def, ShapeGeometry::Sphere(*sphere));
    if shape_id.index1 != 0 {
        crate::recording::with_recording(world, |rec| {
            rec.write_create_sphere_shape(body_id, def, *sphere, shape_id);
        });
    }
    shape_id
}

/// (b3CreateCapsuleShape)
pub fn create_capsule_shape(
    world: &mut World,
    body_id: crate::id::BodyId,
    def: &ShapeDef,
    capsule: &Capsule,
) -> ShapeId {
    let length_sqr = distance_squared(capsule.center1, capsule.center2);
    let shape_id = if length_sqr <= linear_slop() * linear_slop() {
        let sphere = Sphere {
            center: lerp(capsule.center1, capsule.center2, 0.5),
            radius: capsule.radius,
        };
        create_shape(world, body_id, def, ShapeGeometry::Sphere(sphere))
    } else {
        create_shape(world, body_id, def, ShapeGeometry::Capsule(*capsule))
    };
    if shape_id.index1 != 0 {
        crate::recording::with_recording(world, |rec| {
            rec.write_create_capsule_shape(body_id, def, *capsule, shape_id);
        });
    }
    shape_id
}

/// (b3CreateHullShape)
pub fn create_hull_shape(
    world: &mut World,
    body_id: crate::id::BodyId,
    def: &ShapeDef,
    hull: &HullData,
) -> ShapeId {
    debug_assert!(crate::hull::is_valid_hull(hull));
    debug_assert!(hull.hash != 0);
    let shared = world.hull_database.add(hull);
    let shape_id = create_shape(world, body_id, def, ShapeGeometry::Hull(shared));
    if shape_id.index1 != 0 {
        crate::recording::with_recording(world, |rec| {
            let geometry_id = rec.registry.intern_hull(hull);
            rec.write_create_hull_shape(body_id, def, geometry_id, shape_id);
        });
    }
    shape_id
}

/// (b3CreateMeshShape)
///
/// The shape stores an owned clone of `mesh` (C keeps a borrowed pointer). Per-instance
/// `scale` is sanitized via [`safe_scale`].
pub fn create_mesh_shape(
    world: &mut World,
    body_id: crate::id::BodyId,
    def: &ShapeDef,
    mesh: &MeshData,
    scale: Vec3,
) -> ShapeId {
    debug_assert!(is_valid_mesh(Some(mesh)));
    debug_assert!(mesh.hash != 0);
    let shape_id = create_shape(
        world,
        body_id,
        def,
        ShapeGeometry::Mesh {
            data: mesh.clone(),
            scale: safe_scale(scale),
        },
    );
    if shape_id.index1 != 0 {
        crate::recording::with_recording(world, |rec| {
            let geometry_id = rec.registry.intern_mesh(mesh);
            rec.write_create_mesh_shape(body_id, def, geometry_id, safe_scale(scale), shape_id);
        });
    }
    shape_id
}

/// (b3CreateHeightFieldShape)
///
/// Height fields must be on static bodies (enforced in [`create_shape`]).
pub fn create_height_field_shape(
    world: &mut World,
    body_id: crate::id::BodyId,
    def: &ShapeDef,
    height_field: &HeightFieldData,
) -> ShapeId {
    debug_assert!(height_field.hash != 0);
    let shape_id = create_shape(
        world,
        body_id,
        def,
        ShapeGeometry::HeightField(height_field.clone()),
    );
    if shape_id.index1 != 0 {
        crate::recording::with_recording(world, |rec| {
            let geometry_id = rec.registry.intern_height_field(height_field);
            rec.write_create_height_field_shape(body_id, def, geometry_id, shape_id);
        });
    }
    shape_id
}

/// Baked compound shapes are only allowed on static bodies.
/// Note: runtime compounds are achieved by adding multiple shapes to a body.
/// Runtime compounds can be dynamic and/or kinematic.
/// (b3CreateBakedCompoundShape)
///
/// Compounds must be on static non-sensor bodies. Materials are copied from the
/// compound geometry into the shape (see [`create_shape_internal`]).
pub fn create_baked_compound_shape(
    world: &mut World,
    body_id: crate::id::BodyId,
    def: &ShapeDef,
    compound: &CompoundData,
) -> ShapeId {
    debug_assert!(!def.is_sensor);
    let shape_id = create_shape(
        world,
        body_id,
        def,
        ShapeGeometry::Compound(compound.clone()),
    );
    if shape_id.index1 != 0 {
        crate::recording::with_recording(world, |rec| {
            let geometry_id = rec.registry.intern_compound(compound);
            rec.write_create_compound_shape(body_id, def, geometry_id, shape_id);
        });
    }
    shape_id
}

/// Resolve a ShapeId to the shape index. (b3GetShape)
pub fn get_shape(world: &World, shape_id: ShapeId) -> i32 {
    let id = shape_id.index1 - 1;
    let shape = &world.shapes[id as usize];
    debug_assert!(shape.id == id && shape.generation == shape_id.generation);
    id
}

/// Shape identifier validation. (b3Shape_IsValid)
pub fn shape_is_valid(world: &World, id: ShapeId) -> bool {
    if id.index1 < 1 || (world.shapes.len() as i32) < id.index1 {
        return false;
    }
    let shape = &world.shapes[(id.index1 - 1) as usize];
    if shape.id == NULL_INDEX {
        return false;
    }
    shape.generation == id.generation
}

/// (b3Shape_GetHull) — returns None for non-hull shapes.
pub fn shape_get_hull(world: &World, shape_id: ShapeId) -> Option<&HullData> {
    let index = get_shape(world, shape_id);
    match &world.shapes[index as usize].geometry {
        ShapeGeometry::Hull(hull) => Some(hull.as_ref()),
        _ => {
            debug_assert!(false, "shape is not a hull");
            None
        }
    }
}

/// Release hull-database ownership when geometry is about to change.
/// Does not clear materials. (`b3DestroyShapeAllocationForShapeChange`)
pub(crate) fn destroy_shape_allocation_for_shape_change(world: &mut World, shape_index: i32) {
    if matches!(
        world.shapes[shape_index as usize].geometry,
        ShapeGeometry::Hull(_)
    ) {
        let geometry = std::mem::replace(
            &mut world.shapes[shape_index as usize].geometry,
            ShapeGeometry::default(),
        );
        if let ShapeGeometry::Hull(rc) = &geometry {
            world.hull_database.release(rc);
        }
    }

    let user_shape = world.shapes[shape_index as usize].user_shape;
    if user_shape != 0 {
        if let Some(destroy) = world.destroy_debug_shape {
            destroy(user_shape, world.user_debug_shape_context);
        }
        world.shapes[shape_index as usize].user_shape = 0;
    }
}

/// Free hull-database and material allocations for a shape.
/// (b3DestroyShapeAllocations)
fn destroy_shape_allocations(world: &mut World, shape_index: i32) {
    destroy_shape_allocation_for_shape_change(world, shape_index);
    world.shapes[shape_index as usize].materials.clear();
}

/// Destroy a shape on a body. (static b3DestroyShapeInternal)
pub(crate) fn destroy_shape_internal(
    world: &mut World,
    shape_index: i32,
    body_index: i32,
    wake_bodies: bool,
) {
    let _ = wake_bodies; // contact destroy uses this; contacts land next

    let (prev_shape_id, next_shape_id, sensor_index) = {
        let shape = &world.shapes[shape_index as usize];
        (shape.prev_shape_id, shape.next_shape_id, shape.sensor_index)
    };

    // Remove the shape from the body's doubly linked list.
    if prev_shape_id != NULL_INDEX {
        world.shapes[prev_shape_id as usize].next_shape_id = next_shape_id;
    }
    if next_shape_id != NULL_INDEX {
        world.shapes[next_shape_id as usize].prev_shape_id = prev_shape_id;
    }
    if shape_index == world.bodies[body_index as usize].head_shape_id {
        world.bodies[body_index as usize].head_shape_id = next_shape_id;
    }
    world.bodies[body_index as usize].shape_count -= 1;

    // Remove from broad-phase.
    {
        let (shapes, broad_phase) = (&mut world.shapes, &mut world.broad_phase);
        destroy_shape_proxy(&mut shapes[shape_index as usize], broad_phase);
    }

    // Destroy any contacts associated with the shape.
    let mut contact_key = world.bodies[body_index as usize].head_contact_key;
    while contact_key != NULL_INDEX {
        let contact_id = contact_key >> 1;
        let edge_index = contact_key & 1;
        let next_key = world.contacts[contact_id as usize].edges[edge_index as usize].next_key;
        let shape_id_a = world.contacts[contact_id as usize].shape_id_a;
        let shape_id_b = world.contacts[contact_id as usize].shape_id_b;
        contact_key = next_key;

        if shape_id_a == shape_index || shape_id_b == shape_index {
            crate::contact::destroy_contact(world, contact_id, wake_bodies);
        }
    }

    if sensor_index != NULL_INDEX {
        crate::sensor::destroy_sensor(world, shape_index);
    }

    destroy_shape_allocations(world, shape_index);

    world.shape_id_pool.free_id(shape_index);
    world.shapes[shape_index as usize].id = NULL_INDEX;

    world.validate_solver_sets();
}

/// Destroy a shape. (b3DestroyShape)
pub fn destroy_shape(world: &mut World, shape_id: ShapeId, update_body_mass: bool) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    crate::recording::with_recording(world, |rec| {
        rec.write_destroy_shape(shape_id, update_body_mass);
    });

    world.locked = true;

    let shape_index = get_shape(world, shape_id);
    let body_index = world.shapes[shape_index as usize].body_id;

    destroy_shape_internal(world, shape_index, body_index, true);

    if update_body_mass {
        crate::body::update_body_mass_data(world, body_index);
    }

    world.locked = false;
}
