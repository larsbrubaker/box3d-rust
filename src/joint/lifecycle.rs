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
use crate::core::NULL_INDEX;
use crate::id::JointId;
use crate::island::link_joint;
use crate::math_functions::{is_valid_float, is_valid_transform, max_int};
use crate::solver_set::{
    merge_solver_sets, wake_solver_set, AWAKE_SET, DISABLED_SET, FIRST_SLEEPING_SET, STATIC_SET,
};
use crate::types::{BodyType, FilterJointDef, JointDef};
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
mod tests {
    use super::*;
    use crate::body::{create_body, destroy_body, get_body_full_id};
    use crate::broad_phase::update_broad_phase_pairs;
    use crate::constraint_graph::OVERFLOW_INDEX;
    use crate::core::NULL_INDEX;
    use crate::hull::make_cube_hull;
    use crate::joint::{destroy_joint, joint_is_valid};
    use crate::shape::create_hull_shape;
    use crate::solver_set::AWAKE_SET;
    use crate::types::{
        default_body_def, default_filter_joint_def, default_shape_def, default_world_def, BodyType,
    };
    use crate::world::World;

    #[test]
    fn create_and_destroy_filter_joint() {
        let mut world = World::new(&default_world_def());

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        let body_a = create_body(&mut world, &body_def);
        let body_b = create_body(&mut world, &body_def);
        let a_index = get_body_full_id(&world, body_a);
        let b_index = get_body_full_id(&world, body_b);

        let cube = make_cube_hull(0.5);
        let shape_def = default_shape_def();
        let _sa = create_hull_shape(&mut world, body_a, &shape_def, &cube.base);
        let _sb = create_hull_shape(&mut world, body_b, &shape_def, &cube.base);

        update_broad_phase_pairs(&mut world);
        assert_eq!(world.contact_id_pool.id_count(), 1);

        // The two dynamic bodies start in separate islands.
        assert_ne!(
            world.bodies[a_index as usize].island_id,
            world.bodies[b_index as usize].island_id
        );

        // A filter joint merges islands. Box3D C does not destroy the contact
        // on create; SetCollideConnected(false) does (already false here, so
        // toggle true then false to exercise destroy).
        let mut filter_def = default_filter_joint_def();
        filter_def.base.body_id_a = body_a;
        filter_def.base.body_id_b = body_b;
        let filter_id = create_filter_joint(&mut world, &filter_def);

        assert!(joint_is_valid(&world, filter_id));
        assert_eq!(world.joint_id_pool.id_count(), 1);
        assert_eq!(world.bodies[a_index as usize].joint_count, 1);
        assert_eq!(world.bodies[b_index as usize].joint_count, 1);
        assert_eq!(
            world.bodies[a_index as usize].island_id,
            world.bodies[b_index as usize].island_id
        );
        assert_eq!(joint_get_type(&world, filter_id), JointType::Filter);
        assert!(!joint_get_collide_connected(&world, filter_id));
        assert_eq!(joint_get_body_a(&world, filter_id), body_a);
        assert_eq!(joint_get_body_b(&world, filter_id), body_b);

        let raw = get_joint_full_id(&world, filter_id);
        {
            let joint = &world.joints[raw as usize];
            assert_eq!(joint.set_index, AWAKE_SET);
            assert!(joint.color_index != NULL_INDEX && joint.color_index <= OVERFLOW_INDEX);
            assert!(joint.island_id != NULL_INDEX);
            assert_eq!(joint.type_, JointType::Filter);
        }

        // Destroy existing contact via SetCollideConnected(false) after a
        // no-op true→false path is already default; force via true then false.
        assert_eq!(world.contact_id_pool.id_count(), 1);
        joint_set_collide_connected(&mut world, filter_id, true);
        assert!(joint_get_collide_connected(&world, filter_id));
        joint_set_collide_connected(&mut world, filter_id, false);
        assert!(!joint_get_collide_connected(&world, filter_id));
        assert_eq!(world.contact_id_pool.id_count(), 0);

        // Broad-phase pair update does not recreate the filtered contact.
        update_broad_phase_pairs(&mut world);
        assert_eq!(world.contact_id_pool.id_count(), 0);

        // Enabling collision re-buffers shapes so the broad phase can recreate.
        joint_set_collide_connected(&mut world, filter_id, true);
        update_broad_phase_pairs(&mut world);
        assert_eq!(world.contact_id_pool.id_count(), 1);

        destroy_joint(&mut world, filter_id, true);
        assert!(!joint_is_valid(&world, filter_id));
        assert_eq!(world.joint_id_pool.id_count(), 0);
        assert_eq!(world.bodies[a_index as usize].joint_count, 0);
        assert_eq!(world.bodies[b_index as usize].joint_count, 0);

        // Destroying body A with no joints is fine.
        destroy_body(&mut world, body_a);
        world.validate_solver_sets();
    }

    #[test]
    fn destroy_body_destroys_attached_joints() {
        let mut world = World::new(&default_world_def());

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        let body_a = create_body(&mut world, &body_def);
        let body_b = create_body(&mut world, &body_def);

        let mut filter_def = default_filter_joint_def();
        filter_def.base.body_id_a = body_a;
        filter_def.base.body_id_b = body_b;
        let filter_id = create_filter_joint(&mut world, &filter_def);
        assert_eq!(world.joint_id_pool.id_count(), 1);

        destroy_body(&mut world, body_a);
        assert!(!joint_is_valid(&world, filter_id));
        assert_eq!(world.joint_id_pool.id_count(), 0);
        world.validate_solver_sets();
    }

    #[test]
    fn default_joint_defs_match_c_cookies() {
        use crate::core::SECRET_COOKIE;
        use crate::types::{
            default_distance_joint_def, default_motor_joint_def, default_parallel_joint_def,
            default_prismatic_joint_def, default_revolute_joint_def, default_spherical_joint_def,
            default_weld_joint_def, default_wheel_joint_def,
        };

        assert_eq!(default_filter_joint_def().base.internal_value, SECRET_COOKIE);
        assert_eq!(
            default_distance_joint_def().base.internal_value,
            SECRET_COOKIE
        );
        assert_eq!(default_motor_joint_def().base.internal_value, SECRET_COOKIE);
        assert_eq!(
            default_parallel_joint_def().base.internal_value,
            SECRET_COOKIE
        );
        assert_eq!(
            default_prismatic_joint_def().base.internal_value,
            SECRET_COOKIE
        );
        assert_eq!(
            default_revolute_joint_def().base.internal_value,
            SECRET_COOKIE
        );
        assert_eq!(
            default_spherical_joint_def().base.internal_value,
            SECRET_COOKIE
        );
        assert_eq!(default_weld_joint_def().base.internal_value, SECRET_COOKIE);
        assert_eq!(default_wheel_joint_def().base.internal_value, SECRET_COOKIE);

        let wheel = default_wheel_joint_def();
        assert!(wheel.enable_suspension_spring);
        assert_eq!(wheel.suspension_hertz, 1.0);
        assert_eq!(wheel.suspension_damping_ratio, 0.7);

        let parallel = default_parallel_joint_def();
        assert_eq!(parallel.hertz, 1.0);
        assert_eq!(parallel.damping_ratio, 1.0);
        assert_eq!(parallel.max_torque, f32::MAX);

        let distance = default_distance_joint_def();
        assert_eq!(distance.length, 1.0);
        assert_eq!(distance.lower_spring_force, -f32::MAX);
        assert_eq!(distance.upper_spring_force, f32::MAX);
    }
}
