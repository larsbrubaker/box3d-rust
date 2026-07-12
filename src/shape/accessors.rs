//! Remaining public shape getters/setters from shape.c: event flags, density,
//! name, user data, AABB, type/body/world/sensor, ray cast, closest point.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::dispatch::{compute_shape_mass, make_shape_proxy};
use super::lifecycle::get_shape;
use super::query::ray_cast_shape;
use super::shape_flags;
use crate::body::{get_body_transform, get_body_transform_quick, make_body_id};
use crate::constants::{NULL_NAME, SHAPE_NAME_LENGTH};
use crate::contact::contact_flags;
use crate::core::NULL_INDEX;
use crate::distance::{shape_distance, DistanceInput, ShapeProxy, SimplexCache};
use crate::events::ContactData;
use crate::geometry::{MassData, RayCastInput, ShapeType};
use crate::id::{BodyId, ContactId, ShapeId, WorldId};
use crate::math_functions::{
    inv_mul_transforms, is_valid_float, is_valid_position, is_valid_vec3, offset_pos,
    to_relative_transform, transform_point, Aabb, Pos, Vec3, POS_ZERO, TRANSFORM_IDENTITY,
    VEC3_ZERO,
};
use crate::types::WorldCastOutput;
use crate::world::World;

fn truncate_shape_name(name: &str) -> &str {
    if name.len() > SHAPE_NAME_LENGTH {
        let mut end = SHAPE_NAME_LENGTH;
        while end > 0 && !name.is_char_boundary(end) {
            end -= 1;
        }
        &name[..end]
    } else {
        name
    }
}

fn set_shape_flag(world: &mut World, shape_id: ShapeId, bit: u8, flag: bool) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }
    let index = get_shape(world, shape_id);
    let shape = &mut world.shapes[index as usize];
    if flag {
        shape.flags |= bit;
    } else {
        shape.flags &= !bit;
    }
}

fn shape_has_flag(world: &World, shape_id: ShapeId, bit: u8) -> bool {
    let index = get_shape(world, shape_id);
    (world.shapes[index as usize].flags & bit) != 0
}

/// (b3Shape_EnableSensorEvents)
pub fn shape_enable_sensor_events(world: &mut World, shape_id: ShapeId, flag: bool) {
    crate::recording::with_recording(world, |rec| {
        rec.write_shape_enable_sensor_events(shape_id, flag);
    });
    set_shape_flag(world, shape_id, shape_flags::ENABLE_SENSOR_EVENTS, flag);
}

/// (b3Shape_AreSensorEventsEnabled)
pub fn shape_are_sensor_events_enabled(world: &World, shape_id: ShapeId) -> bool {
    shape_has_flag(world, shape_id, shape_flags::ENABLE_SENSOR_EVENTS)
}

/// (b3Shape_EnableContactEvents)
pub fn shape_enable_contact_events(world: &mut World, shape_id: ShapeId, flag: bool) {
    crate::recording::with_recording(world, |rec| {
        rec.write_shape_enable_contact_events(shape_id, flag);
    });
    set_shape_flag(world, shape_id, shape_flags::ENABLE_CONTACT_EVENTS, flag);
}

/// (b3Shape_AreContactEventsEnabled)
pub fn shape_are_contact_events_enabled(world: &World, shape_id: ShapeId) -> bool {
    shape_has_flag(world, shape_id, shape_flags::ENABLE_CONTACT_EVENTS)
}

/// (b3Shape_EnablePreSolveEvents)
pub fn shape_enable_pre_solve_events(world: &mut World, shape_id: ShapeId, flag: bool) {
    crate::recording::with_recording(world, |rec| {
        rec.write_shape_enable_pre_solve_events(shape_id, flag);
    });
    set_shape_flag(world, shape_id, shape_flags::ENABLE_PRE_SOLVE_EVENTS, flag);
}

/// (b3Shape_ArePreSolveEventsEnabled)
pub fn shape_are_pre_solve_events_enabled(world: &World, shape_id: ShapeId) -> bool {
    shape_has_flag(world, shape_id, shape_flags::ENABLE_PRE_SOLVE_EVENTS)
}

/// (b3Shape_EnableHitEvents)
pub fn shape_enable_hit_events(world: &mut World, shape_id: ShapeId, flag: bool) {
    crate::recording::with_recording(world, |rec| {
        rec.write_shape_enable_hit_events(shape_id, flag);
    });
    set_shape_flag(world, shape_id, shape_flags::ENABLE_HIT_EVENTS, flag);
}

/// (b3Shape_AreHitEventsEnabled)
pub fn shape_are_hit_events_enabled(world: &World, shape_id: ShapeId) -> bool {
    shape_has_flag(world, shape_id, shape_flags::ENABLE_HIT_EVENTS)
}

/// (b3Shape_SetUserData) — Rust stores `u64` instead of `void*`.
pub fn shape_set_user_data(world: &mut World, shape_id: ShapeId, user_data: u64) {
    let index = get_shape(world, shape_id);
    world.shapes[index as usize].user_data = user_data;
}

/// (b3Shape_GetUserData)
pub fn shape_get_user_data(world: &World, shape_id: ShapeId) -> u64 {
    let index = get_shape(world, shape_id);
    world.shapes[index as usize].user_data
}

/// Set the shape name (truncated to [`SHAPE_NAME_LENGTH`]). Uses the world name
/// cache rather than C's fixed char buffer. (b3Shape_SetName)
pub fn shape_set_name(world: &mut World, shape_id: ShapeId, name: &str) {
    crate::recording::with_recording(world, |rec| {
        rec.write_shape_set_name(shape_id, name);
    });
    let truncated = truncate_shape_name(name);
    let name_id = world.names.add_name(truncated);
    let index = get_shape(world, shape_id);
    world.shapes[index as usize].name_id = name_id;
}

/// (b3Shape_GetName)
pub fn shape_get_name(world: &World, shape_id: ShapeId) -> &str {
    let index = get_shape(world, shape_id);
    let name_id = world.shapes[index as usize].name_id;
    if name_id == NULL_NAME {
        return "";
    }
    world.names.find_name(name_id).unwrap_or("")
}

/// (b3Shape_IsSensor)
pub fn shape_is_sensor(world: &World, shape_id: ShapeId) -> bool {
    let index = get_shape(world, shape_id);
    world.shapes[index as usize].sensor_index != NULL_INDEX
}

/// (b3Shape_GetType)
pub fn shape_get_type(world: &World, shape_id: ShapeId) -> ShapeType {
    let index = get_shape(world, shape_id);
    world.shapes[index as usize].shape_type()
}

/// (b3Shape_GetBody)
pub fn shape_get_body(world: &World, shape_id: ShapeId) -> BodyId {
    let index = get_shape(world, shape_id);
    let body_index = world.shapes[index as usize].body_id;
    make_body_id(world, body_index)
}

/// (b3Shape_GetWorld)
pub fn shape_get_world(world: &World, shape_id: ShapeId) -> WorldId {
    WorldId {
        index1: shape_id.world0.wrapping_add(1),
        generation: world.generation,
    }
}

/// (b3Shape_SetDensity)
pub fn shape_set_density(
    world: &mut World,
    shape_id: ShapeId,
    density: f32,
    update_body_mass: bool,
) {
    crate::recording::with_recording(world, |rec| {
        rec.write_shape_set_density(shape_id, density, update_body_mass);
    });
    debug_assert!(is_valid_float(density) && density >= 0.0);
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let index = get_shape(world, shape_id);
    if density == world.shapes[index as usize].density {
        return;
    }

    world.shapes[index as usize].density = density;

    if update_body_mass {
        let body_index = world.shapes[index as usize].body_id;
        crate::body::update_body_mass_data(world, body_index);
    }
}

/// (b3Shape_GetDensity)
pub fn shape_get_density(world: &World, shape_id: ShapeId) -> f32 {
    let index = get_shape(world, shape_id);
    world.shapes[index as usize].density
}

/// (b3Shape_GetAABB)
pub fn shape_get_aabb(world: &World, shape_id: ShapeId) -> Aabb {
    let index = get_shape(world, shape_id);
    world.shapes[index as usize].aabb
}

/// World-space ray cast against a single shape. (b3Shape_RayCast)
///
/// Re-centers on `origin` so the cast runs in float precision far from the
/// world origin, matching C.
pub fn shape_ray_cast(
    world: &World,
    shape_id: ShapeId,
    origin: Pos,
    translation: Vec3,
) -> WorldCastOutput {
    debug_assert!(is_valid_position(origin));
    debug_assert!(is_valid_vec3(translation));

    let index = get_shape(world, shape_id);
    let body_id = world.shapes[index as usize].body_id;
    let transform = to_relative_transform(get_body_transform(world, body_id), origin);

    let input = RayCastInput {
        origin: VEC3_ZERO,
        translation,
        max_fraction: 1.0,
    };

    let local = ray_cast_shape(&world.shapes[index as usize], transform, &input);
    WorldCastOutput {
        normal: local.normal,
        point: offset_pos(origin, local.point),
        fraction: local.fraction,
        iterations: local.iterations,
        triangle_index: local.triangle_index,
        child_index: local.child_index,
        material_index: local.material_index,
        hit: local.hit,
    }
}

/// Closest point on a shape to a target in the query frame. (b3Shape_GetClosestPoint)
///
/// Low-level closest point is a documented float carve-out far from the origin.
pub fn shape_get_closest_point(world: &World, shape_id: ShapeId, target: Vec3) -> Vec3 {
    let index = get_shape(world, shape_id);
    let body_id = world.shapes[index as usize].body_id;
    let body = &world.bodies[body_id as usize];
    let transform = to_relative_transform(get_body_transform_quick(world, body), POS_ZERO);

    let mut proxy_b = ShapeProxy::default();
    proxy_b.points[0] = target;
    proxy_b.count = 1;
    proxy_b.radius = 0.0;

    let input = DistanceInput {
        proxy_a: make_shape_proxy(&world.shapes[index as usize]),
        proxy_b,
        transform: inv_mul_transforms(transform, TRANSFORM_IDENTITY),
        use_radii: true,
    };

    let mut cache = SimplexCache::default();
    let output = shape_distance(&input, &mut cache, None);

    // Witness point comes back in frame A; lift it back to the query frame.
    transform_point(transform, output.point_a)
}

/// Conservative contact capacity for a non-sensor shape. (b3Shape_GetContactCapacity)
pub fn shape_get_contact_capacity(world: &World, shape_id: ShapeId) -> i32 {
    debug_assert!(!world.locked);
    if world.locked {
        return 0;
    }

    let shape_index = get_shape(world, shape_id);
    let shape = &world.shapes[shape_index as usize];
    if shape.sensor_index != NULL_INDEX {
        return 0;
    }

    // Conservative and fast
    world.bodies[shape.body_id as usize].contact_count
}

/// Touching contact data involving this shape. (b3Shape_GetContactData)
pub fn shape_get_contact_data(
    world: &World,
    shape_id: ShapeId,
    capacity: usize,
) -> Vec<ContactData> {
    debug_assert!(!world.locked);
    if world.locked {
        return Vec::new();
    }

    let shape_index = get_shape(world, shape_id);
    let shape = &world.shapes[shape_index as usize];
    if shape.sensor_index != NULL_INDEX {
        return Vec::new();
    }

    let mut out = Vec::new();
    let mut contact_key = world.bodies[shape.body_id as usize].head_contact_key;
    while contact_key != NULL_INDEX && out.len() < capacity {
        let contact_id = contact_key >> 1;
        let edge_index = contact_key & 1;

        let contact = &world.contacts[contact_id as usize];
        contact_key = contact.edges[edge_index as usize].next_key;

        // Does contact involve this shape and is it touching?
        if (contact.shape_id_a == shape_index || contact.shape_id_b == shape_index)
            && (contact.flags & contact_flags::TOUCHING) != 0
        {
            let shape_a = &world.shapes[contact.shape_id_a as usize];
            let shape_b = &world.shapes[contact.shape_id_b as usize];

            out.push(ContactData {
                contact_id: ContactId {
                    index1: contact.contact_id + 1,
                    world0: shape_id.world0,
                    padding: 0,
                    generation: contact.generation,
                },
                shape_id_a: ShapeId {
                    index1: shape_a.id + 1,
                    world0: shape_id.world0,
                    generation: shape_a.generation,
                },
                shape_id_b: ShapeId {
                    index1: shape_b.id + 1,
                    world0: shape_id.world0,
                    generation: shape_b.generation,
                },
                manifolds: contact.manifolds.clone(),
            });
        }
    }

    debug_assert!(out.len() <= capacity);
    out
}

/// Capacity required for [`shape_get_sensor_data`] / sensor overlaps.
/// Returns 0 if the shape is not a sensor. (b3Shape_GetSensorCapacity)
pub fn shape_get_sensor_capacity(world: &World, shape_id: ShapeId) -> i32 {
    debug_assert!(!world.locked);
    if world.locked {
        return 0;
    }

    let shape_index = get_shape(world, shape_id);
    let shape = &world.shapes[shape_index as usize];
    if shape.sensor_index == NULL_INDEX {
        return 0;
    }

    world.sensors[shape.sensor_index as usize].overlaps2.len() as i32
}

/// Overlapped shapes for a sensor. Overlaps may contain destroyed shapes —
/// use [`super::shape_is_valid`] to confirm each. (b3Shape_GetSensorData;
/// docs also call this GetSensorOverlaps)
pub fn shape_get_sensor_data(world: &World, shape_id: ShapeId, capacity: usize) -> Vec<ShapeId> {
    debug_assert!(!world.locked);
    if world.locked {
        return Vec::new();
    }

    let shape_index = get_shape(world, shape_id);
    let shape = &world.shapes[shape_index as usize];
    if shape.sensor_index == NULL_INDEX {
        return Vec::new();
    }

    let sensor = &world.sensors[shape.sensor_index as usize];
    let count = sensor.overlaps2.len().min(capacity);
    sensor.overlaps2[..count]
        .iter()
        .map(|visitor| ShapeId {
            index1: visitor.shape_id + 1,
            world0: shape_id.world0,
            generation: visitor.generation,
        })
        .collect()
}

/// Alias for [`shape_get_sensor_data`] matching the header doc name
/// `b3Shape_GetSensorOverlaps`.
pub fn shape_get_sensor_overlaps(
    world: &World,
    shape_id: ShapeId,
    capacity: usize,
) -> Vec<ShapeId> {
    shape_get_sensor_data(world, shape_id, capacity)
}

/// Compute the mass data for a shape. (b3Shape_ComputeMassData)
pub fn shape_compute_mass_data(world: &World, shape_id: ShapeId) -> MassData {
    let shape_index = get_shape(world, shape_id);
    compute_shape_mass(&world.shapes[shape_index as usize])
}
