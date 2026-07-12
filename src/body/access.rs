//! Body shape/joint/contact list accessors and AABB union from body.c.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::lifecycle::{get_body_full_id, get_body_transform};
use crate::contact::contact_flags;
use crate::core::NULL_INDEX;
use crate::events::ContactData;
use crate::id::{BodyId, ContactId, JointId, ShapeId};
use crate::math_functions::{aabb_union, to_vec3, Aabb, VEC3_ZERO};
use crate::world::World;

/// (b3Body_GetShapeCount)
pub fn body_get_shape_count(world: &World, body_id: BodyId) -> i32 {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].shape_count
}

/// Fills at most `capacity` shape ids. (b3Body_GetShapes)
pub fn body_get_shapes(world: &World, body_id: BodyId, capacity: usize) -> Vec<ShapeId> {
    let body_index = get_body_full_id(world, body_id);
    let mut out = Vec::new();
    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX && out.len() < capacity {
        let shape = &world.shapes[shape_id as usize];
        out.push(ShapeId {
            index1: shape.id + 1,
            world0: body_id.world0,
            generation: shape.generation,
        });
        shape_id = shape.next_shape_id;
    }
    out
}

/// (b3Body_GetJointCount)
pub fn body_get_joint_count(world: &World, body_id: BodyId) -> i32 {
    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].joint_count
}

/// Fills at most `capacity` joint ids. (b3Body_GetJoints)
pub fn body_get_joints(world: &World, body_id: BodyId, capacity: usize) -> Vec<JointId> {
    let body_index = get_body_full_id(world, body_id);
    let mut out = Vec::new();
    let mut joint_key = world.bodies[body_index as usize].head_joint_key;
    while joint_key != NULL_INDEX && out.len() < capacity {
        let joint_id = joint_key >> 1;
        let edge_index = joint_key & 1;

        let joint = &world.joints[joint_id as usize];
        out.push(JointId {
            index1: joint_id + 1,
            world0: body_id.world0,
            generation: joint.generation,
        });

        joint_key = joint.edges[edge_index as usize].next_key;
    }
    out
}

/// Conservative and fast. (b3Body_GetContactCapacity)
pub fn body_get_contact_capacity(world: &World, body_id: BodyId) -> i32 {
    debug_assert!(!world.locked);
    if world.locked {
        return 0;
    }

    let body_index = get_body_full_id(world, body_id);
    world.bodies[body_index as usize].contact_count
}

/// Touching contact data for a body, at most `capacity` entries.
/// (b3Body_GetContactData)
pub fn body_get_contact_data(world: &World, body_id: BodyId, capacity: usize) -> Vec<ContactData> {
    debug_assert!(!world.locked);
    if world.locked {
        return Vec::new();
    }

    let body_index = get_body_full_id(world, body_id);

    let mut out = Vec::new();
    let mut contact_key = world.bodies[body_index as usize].head_contact_key;
    while contact_key != NULL_INDEX && out.len() < capacity {
        let contact_id = contact_key >> 1;
        let edge_index = contact_key & 1;

        let contact = &world.contacts[contact_id as usize];
        contact_key = contact.edges[edge_index as usize].next_key;

        // Is contact touching?
        if (contact.flags & contact_flags::TOUCHING) != 0 {
            let shape_a = &world.shapes[contact.shape_id_a as usize];
            let shape_b = &world.shapes[contact.shape_id_b as usize];

            out.push(ContactData {
                contact_id: ContactId {
                    index1: contact.contact_id + 1,
                    world0: body_id.world0,
                    padding: 0,
                    generation: contact.generation,
                },
                shape_id_a: ShapeId {
                    index1: shape_a.id + 1,
                    world0: body_id.world0,
                    generation: shape_a.generation,
                },
                shape_id_b: ShapeId {
                    index1: shape_b.id + 1,
                    world0: body_id.world0,
                    generation: shape_b.generation,
                },
                manifolds: contact.manifolds.clone(),
            });
        }
    }

    debug_assert!(out.len() <= capacity);
    out
}

/// Union of shape AABBs, or a degenerate AABB at the body position when
/// shapeless. (b3Body_ComputeAABB)
pub fn body_compute_aabb(world: &World, body_id: BodyId) -> Aabb {
    debug_assert!(!world.locked);
    if world.locked {
        return Aabb {
            lower_bound: VEC3_ZERO,
            upper_bound: VEC3_ZERO,
        };
    }

    let body_index = get_body_full_id(world, body_id);
    let body = &world.bodies[body_index as usize];
    if body.head_shape_id == NULL_INDEX {
        let transform = get_body_transform(world, body_index);
        let p = to_vec3(transform.p);
        return Aabb {
            lower_bound: p,
            upper_bound: p,
        };
    }

    let mut shape = &world.shapes[body.head_shape_id as usize];
    let mut aabb = shape.aabb;
    while shape.next_shape_id != NULL_INDEX {
        shape = &world.shapes[shape.next_shape_id as usize];
        aabb = aabb_union(aabb, shape.aabb);
    }

    aabb
}
