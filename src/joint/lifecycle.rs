// Joint creation from joint.c: the shared b3CreateJoint machinery, contact
// filtering helpers, and create_filter_joint (no per-type solve payload).
//
// The C b3CreateJoint returns a (b3Joint*, b3JointSim*) pair; the Rust port
// returns the raw joint index and callers re-fetch the sim through
// get_joint_sim, which also survives the solver-set merge that orphans the C
// pointer.
//
// Unlike Box2D, Box3D does not destroy contacts on joint create when
// collideConnected is false — only b3Joint_SetCollideConnected(false) does.
// New pairs are still blocked by should_bodies_collide.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::{
    get_joint_full_id, get_joint_sim, make_joint_id, Joint, JointSim, JointType, JointUnion,
};
use crate::body::get_body_full_id;
use crate::constants::linear_slop;
use crate::core::NULL_INDEX;
use crate::id::JointId;
use crate::island::link_joint;
use crate::math_functions::{
    clamp_float, is_valid_float, is_valid_transform, max_float, max_int, min_float, PI,
};
use crate::solver_set::{
    merge_solver_sets, wake_solver_set, AWAKE_SET, DISABLED_SET, FIRST_SLEEPING_SET, STATIC_SET,
};
use crate::types::{
    BodyType, DistanceJointDef, FilterJointDef, JointDef, ParallelJointDef, PrismaticJointDef,
    RevoluteJointDef, WeldJointDef,
};
use crate::world::World;

/// (static b3DestroyContactsBetweenBodies)
pub(crate) fn destroy_contacts_between_bodies(world: &mut World, body_id_a: i32, body_id_b: i32) {
    // use the smaller of the two contact lists
    let (mut contact_key, other_body_id) = {
        let body_a = &world.bodies[body_id_a as usize];
        let body_b = &world.bodies[body_id_b as usize];
        if body_a.contact_count < body_b.contact_count {
            (body_a.head_contact_key, body_b.id)
        } else {
            (body_b.head_contact_key, body_a.id)
        }
    };

    // no need to wake bodies when a joint removes collision between them
    let wake_bodies = false;

    // destroy the contacts
    while contact_key != NULL_INDEX {
        let contact_id = contact_key >> 1;
        let edge_index = contact_key & 1;

        contact_key = world.contacts[contact_id as usize].edges[edge_index as usize].next_key;

        let other_edge_index = edge_index ^ 1;
        if world.contacts[contact_id as usize].edges[other_edge_index as usize].body_id
            == other_body_id
        {
            // Careful, this removes the contact from the current doubly linked
            // list
            crate::contact::destroy_contact(world, contact_id, wake_bodies);
        }
    }

    world.validate_solver_sets();
}

/// Shared joint creation. Returns the raw joint index; the per-type
/// constructors fill the payload through get_joint_sim. (static b3CreateJoint)
pub(crate) fn create_joint(world: &mut World, def: &JointDef, joint_type: JointType) -> i32 {
    debug_assert!(is_valid_transform(def.local_frame_a));
    debug_assert!(is_valid_transform(def.local_frame_b));
    debug_assert!(def.internal_value == crate::core::SECRET_COOKIE);

    let body_id_a = get_body_full_id(world, def.body_id_a);
    let body_id_b = get_body_full_id(world, def.body_id_b);
    let max_set_index = max_int(
        world.bodies[body_id_a as usize].set_index,
        world.bodies[body_id_b as usize].set_index,
    );

    // Create joint id and joint
    let joint_id = world.joint_id_pool.alloc_id();
    if joint_id == world.joints.len() as i32 {
        world.joints.push(Joint::default());
    }

    {
        let joint = &mut world.joints[joint_id as usize];
        joint.joint_id = joint_id;
        joint.user_data = def.user_data;
        joint.generation = joint.generation.wrapping_add(1);
        joint.set_index = NULL_INDEX;
        joint.color_index = NULL_INDEX;
        joint.local_index = NULL_INDEX;
        joint.island_id = NULL_INDEX;
        joint.island_index = NULL_INDEX;
        joint.draw_scale = def.draw_scale;
        joint.type_ = joint_type;
        joint.collide_connected = def.collide_connected;
    }

    // Doubly linked list on bodyA
    {
        let head_joint_key = world.bodies[body_id_a as usize].head_joint_key;
        {
            let joint = &mut world.joints[joint_id as usize];
            joint.edges[0].body_id = body_id_a;
            joint.edges[0].prev_key = NULL_INDEX;
            joint.edges[0].next_key = head_joint_key;
        }

        let key_a = joint_id << 1;
        if head_joint_key != NULL_INDEX {
            let head_joint = &mut world.joints[(head_joint_key >> 1) as usize];
            head_joint.edges[(head_joint_key & 1) as usize].prev_key = key_a;
        }
        let body_a = &mut world.bodies[body_id_a as usize];
        body_a.head_joint_key = key_a;
        body_a.joint_count += 1;
    }

    // Doubly linked list on bodyB
    {
        let head_joint_key = world.bodies[body_id_b as usize].head_joint_key;
        {
            let joint = &mut world.joints[joint_id as usize];
            joint.edges[1].body_id = body_id_b;
            joint.edges[1].prev_key = NULL_INDEX;
            joint.edges[1].next_key = head_joint_key;
        }

        let key_b = (joint_id << 1) | 1;
        if head_joint_key != NULL_INDEX {
            let head_joint = &mut world.joints[(head_joint_key >> 1) as usize];
            head_joint.edges[(head_joint_key & 1) as usize].prev_key = key_b;
        }
        let body_b = &mut world.bodies[body_id_b as usize];
        body_b.head_joint_key = key_b;
        body_b.joint_count += 1;
    }

    let set_a = world.bodies[body_id_a as usize].set_index;
    let set_b = world.bodies[body_id_b as usize].set_index;
    let type_a = world.bodies[body_id_a as usize].type_;
    let type_b = world.bodies[body_id_b as usize].type_;

    if set_a == DISABLED_SET || set_b == DISABLED_SET {
        // if either body is disabled, create in disabled set
        let local_index = world.solver_sets[DISABLED_SET as usize].joint_sims.len() as i32;
        {
            let joint = &mut world.joints[joint_id as usize];
            joint.set_index = DISABLED_SET;
            joint.local_index = local_index;
        }

        let joint_sim = JointSim {
            joint_id,
            body_id_a,
            body_id_b,
            type_: joint_type,
            union_: JointUnion::empty(joint_type),
            ..JointSim::default()
        };
        world.solver_sets[DISABLED_SET as usize]
            .joint_sims
            .push(joint_sim);
    } else if type_a != BodyType::Dynamic && type_b != BodyType::Dynamic {
        // joint is not attached to a dynamic body
        let local_index = world.solver_sets[STATIC_SET as usize].joint_sims.len() as i32;
        {
            let joint = &mut world.joints[joint_id as usize];
            joint.set_index = STATIC_SET;
            joint.local_index = local_index;
        }

        let joint_sim = JointSim {
            joint_id,
            body_id_a,
            body_id_b,
            type_: joint_type,
            union_: JointUnion::empty(joint_type),
            ..JointSim::default()
        };
        world.solver_sets[STATIC_SET as usize]
            .joint_sims
            .push(joint_sim);
    } else if set_a == AWAKE_SET || set_b == AWAKE_SET {
        // if either body is sleeping, wake it
        if max_set_index >= FIRST_SLEEPING_SET {
            wake_solver_set(world, max_set_index);
        }

        world.joints[joint_id as usize].set_index = AWAKE_SET;

        let (color_index, local_index) =
            crate::constraint_graph::create_joint_in_graph(world, joint_id);
        {
            let joint_sim = &mut world.constraint_graph.colors[color_index as usize].joint_sims
                [local_index as usize];
            joint_sim.joint_id = joint_id;
            joint_sim.body_id_a = body_id_a;
            joint_sim.body_id_b = body_id_b;
            joint_sim.type_ = joint_type;
            joint_sim.union_ = JointUnion::empty(joint_type);
        }
    } else {
        // joint connected between sleeping and/or static bodies
        debug_assert!(set_a >= FIRST_SLEEPING_SET || set_b >= FIRST_SLEEPING_SET);
        debug_assert!(set_a != STATIC_SET || set_b != STATIC_SET);

        // joint should go into the sleeping set (not static set)
        let set_index = max_set_index;

        let local_index = world.solver_sets[set_index as usize].joint_sims.len() as i32;
        {
            let joint = &mut world.joints[joint_id as usize];
            joint.set_index = set_index;
            joint.local_index = local_index;
        }

        let joint_sim = JointSim {
            joint_id,
            body_id_a,
            body_id_b,
            type_: joint_type,
            union_: JointUnion::empty(joint_type),
            ..JointSim::default()
        };
        world.solver_sets[set_index as usize]
            .joint_sims
            .push(joint_sim);

        if set_a != set_b && set_a >= FIRST_SLEEPING_SET && set_b >= FIRST_SLEEPING_SET {
            // merge sleeping sets. The C jointSim pointer is orphaned here;
            // the Rust port re-fetches through get_joint_sim below.
            merge_solver_sets(world, set_a, set_b);
            debug_assert!(
                world.bodies[body_id_a as usize].set_index
                    == world.bodies[body_id_b as usize].set_index
            );
        }
    }

    debug_assert!(is_valid_float(def.force_threshold) && def.force_threshold >= 0.0);
    debug_assert!(is_valid_float(def.torque_threshold) && def.torque_threshold >= 0.0);

    {
        let joint_sim = get_joint_sim(world, joint_id);
        joint_sim.local_frame_a = def.local_frame_a;
        joint_sim.local_frame_b = def.local_frame_b;
        joint_sim.type_ = joint_type;
        joint_sim.constraint_hertz = def.constraint_hertz;
        joint_sim.constraint_damping_ratio = def.constraint_damping_ratio;
        joint_sim.constraint_softness = crate::solver::Softness {
            bias_rate: 0.0,
            mass_scale: 1.0,
            impulse_scale: 0.0,
        };
        joint_sim.force_threshold = def.force_threshold;
        joint_sim.torque_threshold = def.torque_threshold;

        debug_assert!(joint_sim.joint_id == joint_id);
        debug_assert!(joint_sim.body_id_a == body_id_a);
        debug_assert!(joint_sim.body_id_b == body_id_b);
    }

    if world.joints[joint_id as usize].set_index > DISABLED_SET {
        // Add edge to island graph
        link_joint(world, joint_id);
    }

    // Box3D C does not destroy contacts here (unlike Box2D). Filtering for new
    // pairs is handled by should_bodies_collide; existing contacts are cleared
    // only via joint_set_collide_connected(false).

    world.validate_solver_sets();

    joint_id
}

/// (b3CreateFilterJoint)
pub fn create_filter_joint(world: &mut World, def: &FilterJointDef) -> JointId {
    debug_assert!(def.base.internal_value == crate::core::SECRET_COOKIE);
    debug_assert!(!world.locked);
    if world.locked {
        return crate::id::NULL_JOINT_ID;
    }

    let joint_id = create_joint(world, &def.base, JointType::Filter);
    make_joint_id(world, joint_id)
}

/// (b3CreateDistanceJoint)
pub fn create_distance_joint(world: &mut World, def: &DistanceJointDef) -> JointId {
    debug_assert!(def.base.internal_value == crate::core::SECRET_COOKIE);
    debug_assert!(is_valid_float(def.length) && def.length > 0.0);
    debug_assert!(def.lower_spring_force <= def.upper_spring_force);
    debug_assert!(!world.locked);
    if world.locked {
        return crate::id::NULL_JOINT_ID;
    }

    let joint_id = create_joint(world, &def.base, JointType::Distance);

    let joint_sim = get_joint_sim(world, joint_id);
    let joint = joint_sim.distance_mut();
    *joint = super::DistanceJoint::default();
    joint.length = max_float(def.length, linear_slop());
    joint.hertz = def.hertz;
    joint.damping_ratio = def.damping_ratio;
    joint.lower_spring_force = def.lower_spring_force;
    joint.upper_spring_force = def.upper_spring_force;
    joint.min_length = max_float(def.min_length, linear_slop());
    joint.max_length = max_float(def.min_length, def.max_length);
    joint.max_motor_force = def.max_motor_force;
    joint.motor_speed = def.motor_speed;
    joint.enable_spring = def.enable_spring;
    joint.enable_limit = def.enable_limit;
    joint.enable_motor = def.enable_motor;
    joint.impulse = 0.0;
    joint.lower_impulse = 0.0;
    joint.upper_impulse = 0.0;
    joint.motor_impulse = 0.0;

    make_joint_id(world, joint_id)
}

/// (b3CreateParallelJoint)
pub fn create_parallel_joint(world: &mut World, def: &ParallelJointDef) -> JointId {
    debug_assert!(def.base.internal_value == crate::core::SECRET_COOKIE);
    debug_assert!(is_valid_float(def.hertz) && def.hertz >= 0.0);
    debug_assert!(is_valid_float(def.damping_ratio) && def.damping_ratio >= 0.0);
    debug_assert!(is_valid_float(def.max_torque) && def.max_torque >= 0.0);
    debug_assert!(!world.locked);
    if world.locked {
        return crate::id::NULL_JOINT_ID;
    }

    let joint_id = create_joint(world, &def.base, JointType::Parallel);

    let joint_sim = get_joint_sim(world, joint_id);
    let joint = joint_sim.parallel_mut();
    *joint = super::ParallelJoint::default();
    joint.hertz = def.hertz;
    joint.damping_ratio = def.damping_ratio;
    joint.max_torque = def.max_torque;

    make_joint_id(world, joint_id)
}

/// (b3CreateWeldJoint)
pub fn create_weld_joint(world: &mut World, def: &WeldJointDef) -> JointId {
    debug_assert!(def.base.internal_value == crate::core::SECRET_COOKIE);
    debug_assert!(def.angular_hertz >= 0.0);
    debug_assert!(def.angular_damping_ratio >= 0.0);
    debug_assert!(def.linear_hertz >= 0.0);
    debug_assert!(def.linear_damping_ratio >= 0.0);
    debug_assert!(!world.locked);
    if world.locked {
        return crate::id::NULL_JOINT_ID;
    }

    let joint_id = create_joint(world, &def.base, JointType::Weld);

    let joint_sim = get_joint_sim(world, joint_id);
    let joint = joint_sim.weld_mut();
    *joint = super::WeldJoint::default();
    joint.linear_hertz = def.linear_hertz;
    joint.linear_damping_ratio = def.linear_damping_ratio;
    joint.angular_hertz = def.angular_hertz;
    joint.angular_damping_ratio = def.angular_damping_ratio;

    make_joint_id(world, joint_id)
}

/// (b3CreateRevoluteJoint)
pub fn create_revolute_joint(world: &mut World, def: &RevoluteJointDef) -> JointId {
    debug_assert!(def.base.internal_value == crate::core::SECRET_COOKIE);
    debug_assert!(!world.locked);
    if world.locked {
        return crate::id::NULL_JOINT_ID;
    }

    let joint_id = create_joint(world, &def.base, JointType::Revolute);

    let joint_sim = get_joint_sim(world, joint_id);
    let joint = joint_sim.revolute_mut();
    *joint = super::RevoluteJoint::default();
    joint.hertz = def.hertz;
    joint.damping_ratio = def.damping_ratio;
    joint.target_angle = clamp_float(def.target_angle, -PI, PI);

    let lower_angle = min_float(def.lower_angle, def.upper_angle);
    let upper_angle = max_float(def.lower_angle, def.upper_angle);
    joint.lower_angle = clamp_float(lower_angle, -0.99 * PI, 0.99 * PI);
    joint.upper_angle = clamp_float(upper_angle, -0.99 * PI, 0.99 * PI);

    joint.max_motor_torque = def.max_motor_torque;
    joint.motor_speed = def.motor_speed;
    joint.enable_spring = def.enable_spring;
    joint.enable_limit = def.enable_limit;
    joint.enable_motor = def.enable_motor;

    make_joint_id(world, joint_id)
}

/// (b3CreatePrismaticJoint)
pub fn create_prismatic_joint(world: &mut World, def: &PrismaticJointDef) -> JointId {
    debug_assert!(def.base.internal_value == crate::core::SECRET_COOKIE);
    debug_assert!(def.lower_translation <= def.upper_translation);
    debug_assert!(!world.locked);
    if world.locked {
        return crate::id::NULL_JOINT_ID;
    }

    let joint_id = create_joint(world, &def.base, JointType::Prismatic);

    let joint_sim = get_joint_sim(world, joint_id);
    let joint = joint_sim.prismatic_mut();
    *joint = super::PrismaticJoint::default();
    joint.hertz = def.hertz;
    joint.damping_ratio = def.damping_ratio;
    joint.target_translation = def.target_translation;
    joint.lower_translation = def.lower_translation;
    joint.upper_translation = def.upper_translation;
    joint.max_motor_force = def.max_motor_force;
    joint.motor_speed = def.motor_speed;
    joint.enable_spring = def.enable_spring;
    joint.enable_limit = def.enable_limit;
    joint.enable_motor = def.enable_motor;

    make_joint_id(world, joint_id)
}

/// (b3Joint_SetCollideConnected)
pub fn joint_set_collide_connected(world: &mut World, joint_id: JointId, should_collide: bool) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let id = get_joint_full_id(world, joint_id);
    if world.joints[id as usize].collide_connected == should_collide {
        return;
    }

    world.joints[id as usize].collide_connected = should_collide;

    let body_id_a = world.joints[id as usize].edges[0].body_id;
    let body_id_b = world.joints[id as usize].edges[1].body_id;

    if should_collide {
        // need to tell the broad-phase to look for new pairs for one of the
        // two bodies. Pick the one with the fewest shapes.
        let (shape_count_a, shape_count_b) = {
            let body_a = &world.bodies[body_id_a as usize];
            let body_b = &world.bodies[body_id_b as usize];
            (body_a.shape_count, body_b.shape_count)
        };

        let mut shape_id = if shape_count_a < shape_count_b {
            world.bodies[body_id_a as usize].head_shape_id
        } else {
            world.bodies[body_id_b as usize].head_shape_id
        };

        while shape_id != NULL_INDEX {
            let proxy_key = world.shapes[shape_id as usize].proxy_key;
            let next = world.shapes[shape_id as usize].next_shape_id;
            if proxy_key != NULL_INDEX {
                world.broad_phase.buffer_move(proxy_key);
            }
            shape_id = next;
        }
    } else {
        destroy_contacts_between_bodies(world, body_id_a, body_id_b);
    }
}

/// (b3Joint_GetCollideConnected)
pub fn joint_get_collide_connected(world: &World, joint_id: JointId) -> bool {
    let id = get_joint_full_id(world, joint_id);
    world.joints[id as usize].collide_connected
}

/// (b3Joint_GetType)
pub fn joint_get_type(world: &World, joint_id: JointId) -> JointType {
    let id = get_joint_full_id(world, joint_id);
    world.joints[id as usize].type_
}

/// (b3Joint_GetBodyA)
pub fn joint_get_body_a(world: &World, joint_id: JointId) -> crate::id::BodyId {
    let id = get_joint_full_id(world, joint_id);
    let body_index = world.joints[id as usize].edges[0].body_id;
    crate::body::make_body_id(world, body_index)
}

/// (b3Joint_GetBodyB)
pub fn joint_get_body_b(world: &World, joint_id: JointId) -> crate::id::BodyId {
    let id = get_joint_full_id(world, joint_id);
    let body_index = world.joints[id as usize].edges[1].body_id;
    crate::body::make_body_id(world, body_index)
}


#[cfg(test)]
#[path = "lifecycle_tests.rs"]
mod tests;
