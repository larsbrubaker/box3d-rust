// Contact register table and contact creation/destruction from contact.c.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::{contact_flags, Contact, ContactGeometry, MeshContact};
use crate::body::body_flags;
use crate::core::NULL_INDEX;
use crate::events::ContactEndTouchEvent;
use crate::geometry::ShapeType;
use crate::id::{ContactId, ShapeId};
use crate::math_functions::max_float;
use crate::shape::{shape_flags, ShapeGeometry};
use crate::solver_set::{AWAKE_SET, DISABLED_SET, STATIC_SET};
use crate::table::shape_pair_key;
use crate::types::BodyType;
use crate::world::World;

/// Primary (unflipped) order pairs from b3InitializeContactRegisters / b3AddType.
fn is_primary_pair(type_a: ShapeType, type_b: ShapeType) -> bool {
    use ShapeType::*;
    matches!(
        (type_a, type_b),
        (Sphere, Sphere)
            | (Capsule, Sphere)
            | (Capsule, Capsule)
            | (Compound, Sphere)
            | (Compound, Capsule)
            | (Compound, Hull)
            | (Hull, Sphere)
            | (Hull, Capsule)
            | (Hull, Hull)
            | (Mesh, Sphere)
            | (Mesh, Capsule)
            | (Mesh, Hull)
            | (Height, Sphere)
            | (Height, Capsule)
            | (Height, Hull)
    )
}

/// Register table entry: Some(primary) if supported. (s_registers)
pub(crate) fn contact_register(type_a: ShapeType, type_b: ShapeType) -> Option<bool> {
    if is_primary_pair(type_a, type_b) {
        return Some(true);
    }
    if type_a != type_b && is_primary_pair(type_b, type_a) {
        return Some(false);
    }
    None
}

/// (b3CanCollide equivalent — register supported)
pub fn can_collide(type_a: ShapeType, type_b: ShapeType) -> bool {
    contact_register(type_a, type_b).is_some()
}

/// No-op kept for C API parity; registers are compile-time via match.
/// (b3InitializeContactRegisters)
pub fn initialize_contact_registers() {}

fn shape_radius(world: &World, shape_index: i32) -> f32 {
    match &world.shapes[shape_index as usize].geometry {
        crate::shape::ShapeGeometry::Sphere(s) => s.radius,
        crate::shape::ShapeGeometry::Capsule(c) => c.radius,
        _ => 0.0,
    }
}

/// Create a contact between two shapes. (b3CreateContact)
pub fn create_contact(world: &mut World, shape_id_a: i32, shape_id_b: i32, child_index: i32) {
    let type_a = world.shapes[shape_id_a as usize].shape_type();
    let type_b = world.shapes[shape_id_b as usize].shape_type();

    let Some(primary) = contact_register(type_a, type_b) else {
        return;
    };

    if !primary {
        create_contact(world, shape_id_b, shape_id_a, child_index);
        return;
    }

    let body_id_a = world.shapes[shape_id_a as usize].body_id;
    let body_id_b = world.shapes[shape_id_b as usize].body_id;

    let set_a = world.bodies[body_id_a as usize].set_index;
    let set_b = world.bodies[body_id_b as usize].set_index;
    debug_assert!(set_a != DISABLED_SET && set_b != DISABLED_SET);
    debug_assert!(set_a != STATIC_SET || set_b != STATIC_SET);

    let set_index = if set_a == AWAKE_SET || set_b == AWAKE_SET {
        AWAKE_SET
    } else {
        DISABLED_SET
    };

    let contact_id = world.contact_id_pool.alloc_id();
    if contact_id == world.contacts.len() as i32 {
        world.contacts.push(Contact::default());
    }

    let local_index = world.solver_sets[set_index as usize].contact_indices.len() as i32;

    {
        let contact = &mut world.contacts[contact_id as usize];
        let generation = contact.generation;
        *contact = Contact::default();
        contact.contact_id = contact_id;
        contact.generation = generation.wrapping_add(1);
        contact.set_index = set_index;
        contact.color_index = NULL_INDEX;
        contact.local_index = local_index;
        contact.island_id = NULL_INDEX;
        contact.island_index = NULL_INDEX;
        contact.shape_id_a = shape_id_a;
        contact.shape_id_b = shape_id_b;
        contact.child_index = child_index;
    }

    if (world.bodies[body_id_a as usize].flags & body_flags::BODY_ENABLE_CONTACT_RECYCLING) != 0
        && (world.bodies[body_id_b as usize].flags & body_flags::BODY_ENABLE_CONTACT_RECYCLING) != 0
    {
        world.contacts[contact_id as usize].flags |= contact_flags::RECYCLE;
    }

    if type_a == ShapeType::Mesh || type_a == ShapeType::Height {
        world.contacts[contact_id as usize].flags |= contact_flags::SIM_MESH_CONTACT;
        world.contacts[contact_id as usize].geometry =
            ContactGeometry::Mesh(MeshContact::default());
    } else if type_a == ShapeType::Compound {
        use crate::compound::{get_compound_child, ChildGeometry};
        let ShapeGeometry::Compound(compound) = &world.shapes[shape_id_a as usize].geometry else {
            unreachable!()
        };
        let child = get_compound_child(compound, child_index);
        if matches!(child.geometry, ChildGeometry::Mesh(_)) {
            world.contacts[contact_id as usize].flags |= contact_flags::SIM_MESH_CONTACT;
            world.contacts[contact_id as usize].geometry =
                ContactGeometry::Mesh(MeshContact::default());
        }
    }

    debug_assert!(
        type_b == ShapeType::Sphere || type_b == ShapeType::Capsule || type_b == ShapeType::Hull
    );

    if world.bodies[body_id_a as usize].type_ == BodyType::Static
        || world.bodies[body_id_b as usize].type_ == BodyType::Static
    {
        world.contacts[contact_id as usize].flags |= contact_flags::STATIC_FLAG;
    }

    debug_assert!(
        world.shapes[shape_id_a as usize].sensor_index == NULL_INDEX
            && world.shapes[shape_id_b as usize].sensor_index == NULL_INDEX
    );

    if (world.shapes[shape_id_a as usize].flags & shape_flags::ENABLE_CONTACT_EVENTS) != 0
        || (world.shapes[shape_id_b as usize].flags & shape_flags::ENABLE_CONTACT_EVENTS) != 0
    {
        world.contacts[contact_id as usize].flags |= contact_flags::ENABLE_CONTACT_EVENTS;
    }

    // Connect to body A
    {
        let head_contact_key = world.bodies[body_id_a as usize].head_contact_key;
        {
            let contact = &mut world.contacts[contact_id as usize];
            contact.edges[0].body_id = body_id_a;
            contact.edges[0].prev_key = NULL_INDEX;
            contact.edges[0].next_key = head_contact_key;
        }
        let key_a = contact_id << 1;
        if head_contact_key != NULL_INDEX {
            let head_contact = &mut world.contacts[(head_contact_key >> 1) as usize];
            head_contact.edges[(head_contact_key & 1) as usize].prev_key = key_a;
        }
        let body_a = &mut world.bodies[body_id_a as usize];
        body_a.head_contact_key = key_a;
        body_a.contact_count += 1;
    }

    // Connect to body B
    {
        let head_contact_key = world.bodies[body_id_b as usize].head_contact_key;
        {
            let contact = &mut world.contacts[contact_id as usize];
            contact.edges[1].body_id = body_id_b;
            contact.edges[1].prev_key = NULL_INDEX;
            contact.edges[1].next_key = head_contact_key;
        }
        let key_b = (contact_id << 1) | 1;
        if head_contact_key != NULL_INDEX {
            let head_contact = &mut world.contacts[(head_contact_key >> 1) as usize];
            head_contact.edges[(head_contact_key & 1) as usize].prev_key = key_b;
        }
        let body_b = &mut world.bodies[body_id_b as usize];
        body_b.head_contact_key = key_b;
        body_b.contact_count += 1;
    }

    let pair_key = shape_pair_key(shape_id_a, shape_id_b, child_index);
    world.broad_phase.pair_set.add_key(pair_key);

    world.solver_sets[set_index as usize]
        .contact_indices
        .push(contact_id);

    let radius_a = shape_radius(world, shape_id_a);
    let radius_b = shape_radius(world, shape_id_b);
    let max_radius = max_float(radius_a, radius_b);

    let rolling_a = world.shapes[shape_id_a as usize]
        .get_material(0)
        .rolling_resistance;
    let rolling_b = world.shapes[shape_id_b as usize]
        .get_material(0)
        .rolling_resistance;
    world.contacts[contact_id as usize].rolling_resistance =
        max_float(rolling_a, rolling_b) * max_radius;

    if (world.shapes[shape_id_a as usize].flags & shape_flags::ENABLE_PRE_SOLVE_EVENTS) != 0
        || (world.shapes[shape_id_b as usize].flags & shape_flags::ENABLE_PRE_SOLVE_EVENTS) != 0
    {
        world.contacts[contact_id as usize].flags |= contact_flags::SIM_ENABLE_PRE_SOLVE_EVENTS;
    }
}

/// (b3DestroyContact)
pub fn destroy_contact(world: &mut World, contact_id: i32, wake_bodies: bool) {
    let (shape_id_a, shape_id_b, child_index, edge_a, edge_b, flags, generation) = {
        let contact = &world.contacts[contact_id as usize];
        (
            contact.shape_id_a,
            contact.shape_id_b,
            contact.child_index,
            contact.edges[0],
            contact.edges[1],
            contact.flags,
            contact.generation,
        )
    };

    let pair_key = shape_pair_key(shape_id_a, shape_id_b, child_index);
    world.broad_phase.pair_set.remove_key(pair_key);

    world.contacts[contact_id as usize].manifolds.clear();

    let body_id_a = edge_a.body_id;
    let body_id_b = edge_b.body_id;
    let touching = (flags & contact_flags::TOUCHING) != 0;

    if touching && (flags & contact_flags::ENABLE_CONTACT_EVENTS) != 0 {
        let world_id = world.world_id;
        let shape_a = &world.shapes[shape_id_a as usize];
        let shape_b = &world.shapes[shape_id_b as usize];
        let event = ContactEndTouchEvent {
            shape_id_a: ShapeId {
                index1: shape_a.id + 1,
                world0: world_id,
                generation: shape_a.generation,
            },
            shape_id_b: ShapeId {
                index1: shape_b.id + 1,
                world0: world_id,
                generation: shape_b.generation,
            },
            contact_id: ContactId {
                index1: contact_id + 1,
                world0: world_id,
                padding: 0,
                generation,
            },
        };
        world.contact_end_events[world.end_event_array_index as usize].push(event);
    }

    // Remove from body A
    if edge_a.prev_key != NULL_INDEX {
        let prev = &mut world.contacts[(edge_a.prev_key >> 1) as usize];
        prev.edges[(edge_a.prev_key & 1) as usize].next_key = edge_a.next_key;
    }
    if edge_a.next_key != NULL_INDEX {
        let next = &mut world.contacts[(edge_a.next_key >> 1) as usize];
        next.edges[(edge_a.next_key & 1) as usize].prev_key = edge_a.prev_key;
    }
    let edge_key_a = contact_id << 1;
    {
        let body_a = &mut world.bodies[body_id_a as usize];
        if body_a.head_contact_key == edge_key_a {
            body_a.head_contact_key = edge_a.next_key;
        }
        body_a.contact_count -= 1;
    }

    // Remove from body B
    if edge_b.prev_key != NULL_INDEX {
        let prev = &mut world.contacts[(edge_b.prev_key >> 1) as usize];
        prev.edges[(edge_b.prev_key & 1) as usize].next_key = edge_b.next_key;
    }
    if edge_b.next_key != NULL_INDEX {
        let next = &mut world.contacts[(edge_b.next_key >> 1) as usize];
        next.edges[(edge_b.next_key & 1) as usize].prev_key = edge_b.prev_key;
    }
    let edge_key_b = (contact_id << 1) | 1;
    {
        let body_b = &mut world.bodies[body_id_b as usize];
        if body_b.head_contact_key == edge_key_b {
            body_b.head_contact_key = edge_b.next_key;
        }
        body_b.contact_count -= 1;
    }

    if (flags & contact_flags::SIM_MESH_CONTACT) != 0 {
        if let ContactGeometry::Mesh(mesh) = &mut world.contacts[contact_id as usize].geometry {
            mesh.triangle_cache.clear();
        }
    }

    if world.contacts[contact_id as usize].island_id != NULL_INDEX {
        crate::island::unlink_contact(world, contact_id);
    }

    let (color_index, local_index, set_index, mesh_contact) = {
        let c = &world.contacts[contact_id as usize];
        (
            c.color_index,
            c.local_index,
            c.set_index,
            (c.flags & contact_flags::SIM_MESH_CONTACT) != 0,
        )
    };

    if color_index != NULL_INDEX {
        debug_assert!(set_index == AWAKE_SET);
        crate::constraint_graph::remove_contact_from_graph(
            world,
            body_id_a,
            body_id_b,
            color_index,
            local_index,
            mesh_contact,
        );
    } else {
        debug_assert!(
            set_index != AWAKE_SET
                || (world.contacts[contact_id as usize].flags & contact_flags::TOUCHING) == 0
        );
        let set = &mut world.solver_sets[set_index as usize];
        let moved_index = set.contact_indices.len() as i32 - 1;
        set.contact_indices.swap_remove(local_index as usize);
        if moved_index != local_index {
            let moved_contact_id = set.contact_indices[local_index as usize];
            world.contacts[moved_contact_id as usize].local_index = local_index;
        }
    }

    {
        let contact = &mut world.contacts[contact_id as usize];
        contact.contact_id = NULL_INDEX;
        contact.set_index = NULL_INDEX;
        contact.color_index = NULL_INDEX;
        contact.local_index = NULL_INDEX;
    }
    world.contact_id_pool.free_id(contact_id);

    if wake_bodies && touching {
        crate::body::wake_body(world, body_id_a);
        crate::body::wake_body(world, body_id_b);
    }
}

/// Contact identifier validation. (b3Contact_IsValid — world registry checks
/// collapse to the world argument)
pub fn contact_is_valid(world: &World, id: ContactId) -> bool {
    let contact_id = id.index1 - 1;
    if contact_id < 0 || (world.contacts.len() as i32) <= contact_id {
        return false;
    }

    let contact = &world.contacts[contact_id as usize];
    if contact.contact_id == NULL_INDEX {
        return false;
    }

    debug_assert!(contact.contact_id == contact_id);

    id.generation == contact.generation
}
