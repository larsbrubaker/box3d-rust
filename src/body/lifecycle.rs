// Body create/destroy and island helpers from body.c.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::{body_flags, Body, BodySim, IDENTITY_BODY_STATE};
use crate::constants::huge;
use crate::core::{NULL_INDEX, SECRET_COOKIE};
use crate::id::{BodyId, NULL_BODY_ID};
use crate::island::{create_island, destroy_island, validate_island};
use crate::math_functions::{
    is_valid_float, is_valid_position, is_valid_quat, is_valid_vec3, length, WorldTransform,
};
use crate::solver_set::{
    destroy_solver_set, wake_solver_set, SolverSet, AWAKE_SET, DISABLED_SET, FIRST_SLEEPING_SET,
    STATIC_SET,
};
use crate::types::BodyType;
use crate::world::World;

/// Resolve a BodyId to the body index. (b3GetBodyFullId)
pub fn get_body_full_id(world: &World, body_id: BodyId) -> i32 {
    debug_assert!(body_is_valid(world, body_id));
    body_id.index1 - 1
}

/// Create a BodyId from a raw body index. (b3MakeBodyId)
pub fn make_body_id(world: &World, body_index: i32) -> BodyId {
    let body = &world.bodies[body_index as usize];
    BodyId {
        index1: body_index + 1,
        world0: world.world_id,
        generation: body.generation,
    }
}

/// Body identifier validation. (b3Body_IsValid — world registry checks collapse
/// to the world argument)
pub fn body_is_valid(world: &World, id: BodyId) -> bool {
    if id.index1 < 1 || (world.bodies.len() as i32) < id.index1 {
        return false;
    }

    let body = &world.bodies[(id.index1 - 1) as usize];
    if body.set_index == NULL_INDEX {
        return false;
    }

    debug_assert!(body.local_index != NULL_INDEX);

    if body.generation != id.generation {
        return false;
    }

    true
}

/// Quick transform lookup. (b3GetBodyTransformQuick)
pub fn get_body_transform_quick(world: &World, body: &Body) -> WorldTransform {
    world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize].transform
}

/// Transform by body index. (b3GetBodyTransform)
pub fn get_body_transform(world: &World, body_index: i32) -> WorldTransform {
    get_body_transform_quick(world, &world.bodies[body_index as usize])
}

/// (b3GetBodySim)
pub fn get_body_sim(world: &World, body_index: i32) -> &BodySim {
    let body = &world.bodies[body_index as usize];
    &world.solver_sets[body.set_index as usize].body_sims[body.local_index as usize]
}

/// (b3GetBodySim) mutable
pub fn get_body_sim_mut(world: &mut World, body_index: i32) -> &mut BodySim {
    let set_index = world.bodies[body_index as usize].set_index;
    let local_index = world.bodies[body_index as usize].local_index;
    &mut world.solver_sets[set_index as usize].body_sims[local_index as usize]
}

/// (b3GetBodyState) — None when the body is not in the awake set.
pub fn get_body_state_index(world: &World, body_index: i32) -> Option<i32> {
    let body = &world.bodies[body_index as usize];
    if body.set_index == AWAKE_SET {
        Some(body.local_index)
    } else {
        None
    }
}

/// Sync non-transient flags from Body onto BodySim / BodyState. (b3SyncBodyFlags)
pub fn sync_body_flags(world: &mut World, body_index: i32) {
    let flags = world.bodies[body_index as usize].flags & !body_flags::BODY_TRANSIENT_FLAGS;
    {
        let sim = get_body_sim_mut(world, body_index);
        sim.flags = flags;
    }
    if let Some(local_index) = get_body_state_index(world, body_index) {
        world.solver_sets[AWAKE_SET as usize].body_states[local_index as usize].flags = flags;
    }
}

/// Joint `collide_connected` override. (b3ShouldBodiesCollide)
pub fn should_bodies_collide(world: &World, body_id_a: i32, body_id_b: i32) -> bool {
    let body_a = &world.bodies[body_id_a as usize];
    let body_b = &world.bodies[body_id_b as usize];

    let (mut joint_key, other_body_id) = if body_a.joint_count < body_b.joint_count {
        (body_a.head_joint_key, body_b.id)
    } else {
        (body_b.head_joint_key, body_a.id)
    };

    while joint_key != NULL_INDEX {
        let joint_id = joint_key >> 1;
        let edge_index = joint_key & 1;
        let other_edge_index = edge_index ^ 1;
        let joint = &world.joints[joint_id as usize];
        if !joint.collide_connected
            && joint.edges[other_edge_index as usize].body_id == other_body_id
        {
            return false;
        }
        joint_key = joint.edges[edge_index as usize].next_key;
    }

    true
}

/// (static b3CreateIslandForBody)
pub(crate) fn create_island_for_body(world: &mut World, set_index: i32, body_index: i32) {
    debug_assert!(world.bodies[body_index as usize].island_id == NULL_INDEX);
    debug_assert!(set_index != DISABLED_SET);

    let island_id = create_island(world, set_index);
    world.islands[island_id as usize].bodies.push(body_index);
    let body = &mut world.bodies[body_index as usize];
    body.island_id = island_id;
    body.island_index = 0;

    validate_island(world, island_id);
}

/// (static b3RemoveBodyFromIsland)
pub(crate) fn remove_body_from_island(world: &mut World, body_index: i32) {
    let (island_id, island_index) = {
        let body = &world.bodies[body_index as usize];
        (body.island_id, body.island_index)
    };
    if island_id == NULL_INDEX {
        debug_assert!(island_index == NULL_INDEX);
        return;
    }

    {
        let local_index = island_index;
        let last = world.islands[island_id as usize].bodies.len() - 1;
        let moved_body_id = world.islands[island_id as usize].bodies[last];
        world.islands[island_id as usize].bodies[local_index as usize] = moved_body_id;
        debug_assert!(world.bodies[moved_body_id as usize].island_index == last as i32);
        world.bodies[moved_body_id as usize].island_index = local_index;
        world.islands[island_id as usize].bodies.pop();
    }

    if world.islands[island_id as usize].bodies.is_empty() {
        debug_assert!(world.islands[island_id as usize].contacts.is_empty());
        debug_assert!(world.islands[island_id as usize].joints.is_empty());
        destroy_island(world, island_id);
    } else {
        validate_island(world, island_id);
    }

    let body = &mut world.bodies[body_index as usize];
    body.island_id = NULL_INDEX;
    body.island_index = NULL_INDEX;
}

/// True when the body is in the awake set. (b3IsBodyAwake)
pub fn is_body_awake(world: &World, body_index: i32) -> bool {
    world.bodies[body_index as usize].set_index == AWAKE_SET
}

/// Wake a sleeping body. (b3WakeBody)
pub fn wake_body(world: &mut World, body_index: i32) -> bool {
    let set_index = world.bodies[body_index as usize].set_index;
    if set_index >= FIRST_SLEEPING_SET {
        wake_solver_set(world, set_index);
        world.validate_solver_sets();
        return true;
    }
    false
}

/// Wake with world lock. (b3WakeBodyWithLock)
pub fn wake_body_with_lock(world: &mut World, body_index: i32) -> bool {
    debug_assert!(!world.locked);
    world.locked = true;
    let woke = wake_body(world, body_index);
    world.locked = false;
    woke
}

/// Create a rigid body given a definition. (b3CreateBody)
///
/// C resolves the world from a WorldId registry; Rust takes `&mut World`.
pub fn create_body(world: &mut World, def: &crate::types::BodyDef) -> BodyId {
    debug_assert!(def.internal_value == SECRET_COOKIE);
    debug_assert!(is_valid_position(def.position));
    debug_assert!(is_valid_quat(def.rotation));
    debug_assert!(is_valid_vec3(def.linear_velocity));
    debug_assert!(is_valid_vec3(def.angular_velocity));
    debug_assert!(is_valid_float(def.linear_damping) && def.linear_damping >= 0.0);
    debug_assert!(is_valid_float(def.angular_damping) && def.angular_damping >= 0.0);
    debug_assert!(is_valid_float(def.sleep_threshold) && def.sleep_threshold >= 0.0);
    debug_assert!(is_valid_float(def.gravity_scale));

    debug_assert!(!world.locked);
    if world.locked {
        return NULL_BODY_ID;
    }

    world.locked = true;

    let is_awake = (def.is_awake || !def.enable_sleep) && def.is_enabled;

    // determine the solver set
    let set_id;
    if !def.is_enabled {
        set_id = DISABLED_SET;
    } else if def.type_ == BodyType::Static {
        set_id = STATIC_SET;
    } else if is_awake {
        set_id = AWAKE_SET;
    } else {
        // new set for a sleeping body in its own island
        set_id = world.solver_set_id_pool.alloc_id();
        if set_id == world.solver_sets.len() as i32 {
            world.solver_sets.push(SolverSet::default());
        } else {
            debug_assert!(world.solver_sets[set_id as usize].set_index == NULL_INDEX);
        }
        world.solver_sets[set_id as usize].set_index = set_id;
    }

    debug_assert!(0 <= set_id && set_id < world.solver_sets.len() as i32);

    let body_id = world.body_id_pool.alloc_id();

    let mut lock_flags = 0u32;
    lock_flags |= if def.motion_locks.linear_x {
        body_flags::LOCK_LINEAR_X
    } else {
        0
    };
    lock_flags |= if def.motion_locks.linear_y {
        body_flags::LOCK_LINEAR_Y
    } else {
        0
    };
    lock_flags |= if def.motion_locks.linear_z {
        body_flags::LOCK_LINEAR_Z
    } else {
        0
    };
    lock_flags |= if def.motion_locks.angular_x {
        body_flags::LOCK_ANGULAR_X
    } else {
        0
    };
    lock_flags |= if def.motion_locks.angular_y {
        body_flags::LOCK_ANGULAR_Y
    } else {
        0
    };
    lock_flags |= if def.motion_locks.angular_z {
        body_flags::LOCK_ANGULAR_Z
    } else {
        0
    };

    let mut body_sim = BodySim {
        transform: WorldTransform {
            p: def.position,
            q: def.rotation,
        },
        center: def.position,
        rotation0: def.rotation,
        center0: def.position,
        min_extent: huge(),
        max_extent: crate::math_functions::VEC3_ZERO,
        linear_damping: def.linear_damping,
        angular_damping: def.angular_damping,
        gravity_scale: def.gravity_scale,
        body_id,
        flags: lock_flags,
        ..Default::default()
    };
    body_sim.flags |= if def.is_bullet {
        body_flags::IS_BULLET
    } else {
        0
    };
    body_sim.flags |= if def.allow_fast_rotation {
        body_flags::ALLOW_FAST_ROTATION
    } else {
        0
    };
    body_sim.flags |= if def.type_ == BodyType::Dynamic {
        body_flags::DYNAMIC_FLAG
    } else {
        0
    };
    body_sim.flags |= if def.enable_sleep {
        body_flags::ENABLE_SLEEP
    } else {
        0
    };
    body_sim.flags |= if def.enable_contact_recycling {
        body_flags::BODY_ENABLE_CONTACT_RECYCLING
    } else {
        0
    };
    let sim_flags = body_sim.flags;

    let local_index = {
        let set = &mut world.solver_sets[set_id as usize];
        set.body_sims.push(body_sim);
        let local_index = set.body_sims.len() as i32 - 1;

        if set_id == AWAKE_SET {
            let mut body_state = IDENTITY_BODY_STATE;
            body_state.linear_velocity = def.linear_velocity;
            body_state.angular_velocity = def.angular_velocity;
            body_state.flags = sim_flags;
            set.body_states.push(body_state);

            set.body_sims[local_index as usize].max_angular_velocity =
                length(def.angular_velocity) + 5.0;
        }
        local_index
    };

    if body_id == world.bodies.len() as i32 {
        world.bodies.push(Body::default());
    } else {
        debug_assert!(world.bodies[body_id as usize].id == NULL_INDEX);
    }

    let name_id = world.names.add_name(&def.name);

    {
        let body = &mut world.bodies[body_id as usize];
        body.user_data = def.user_data;
        body.set_index = set_id;
        body.local_index = local_index;
        body.generation = body.generation.wrapping_add(1);
        body.head_shape_id = NULL_INDEX;
        body.shape_count = 0;
        body.head_chain_id = NULL_INDEX;
        body.head_contact_key = NULL_INDEX;
        body.contact_count = 0;
        body.head_joint_key = NULL_INDEX;
        body.joint_count = 0;
        body.island_id = NULL_INDEX;
        body.island_index = NULL_INDEX;
        body.body_move_index = NULL_INDEX;
        body.id = body_id;
        body.sleep_threshold = def.sleep_threshold;
        body.sleep_time = 0.0;
        body.sleep_velocity = 0.0;
        body.mass = 0.0;
        body.inertia = crate::math_functions::MAT3_ZERO;
        body.name_id = name_id;
        body.type_ = def.type_;
        body.flags = sim_flags;
    }

    // dynamic and kinematic bodies that are enabled need an island
    if set_id >= AWAKE_SET {
        create_island_for_body(world, set_id, body_id);
    }

    world.validate_solver_sets();

    let id = make_body_id(world, body_id);
    world.locked = false;
    id
}

/// Destroy all contacts attached to a body. (static b3DestroyBodyContacts)
pub(crate) fn destroy_body_contacts(world: &mut World, body_index: i32, wake_bodies: bool) {
    let mut edge_key = world.bodies[body_index as usize].head_contact_key;
    while edge_key != NULL_INDEX {
        let contact_id = edge_key >> 1;
        let edge_index = edge_key & 1;
        edge_key = world.contacts[contact_id as usize].edges[edge_index as usize].next_key;
        crate::contact::destroy_contact(world, contact_id, wake_bodies);
    }

    world.validate_solver_sets();
}

/// Destroy a rigid body. (b3DestroyBody)
pub fn destroy_body(world: &mut World, body_id: BodyId) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.locked = true;

    let body_index = get_body_full_id(world, body_id);

    // Wake bodies attached to this body, even if this body is static.
    let wake_bodies = true;

    // Destroy the attached joints
    let mut edge_key = world.bodies[body_index as usize].head_joint_key;
    while edge_key != NULL_INDEX {
        let joint_id = edge_key >> 1;
        let edge_index = edge_key & 1;
        edge_key = world.joints[joint_id as usize].edges[edge_index as usize].next_key;

        // Careful because this modifies the list being traversed
        crate::joint::destroy_joint_internal(world, joint_id, wake_bodies);
    }

    destroy_body_contacts(world, body_index, wake_bodies);

    // Destroy the attached shapes and their broad-phase proxies.
    let mut shape_id = world.bodies[body_index as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        let next = world.shapes[shape_id as usize].next_shape_id;
        crate::shape::lifecycle::destroy_shape_internal(world, shape_id, body_index, true);
        shape_id = next;
    }

    remove_body_from_island(world, body_index);

    let (set_index, local_index) = {
        let body = &world.bodies[body_index as usize];
        (body.set_index, body.local_index)
    };

    let set = &mut world.solver_sets[set_index as usize];
    let moved_index = {
        let last = set.body_sims.len() as i32 - 1;
        set.body_sims.swap_remove(local_index as usize);
        if local_index < last {
            Some(last)
        } else {
            None
        }
    };

    if let Some(moved) = moved_index {
        let moved_sim = &set.body_sims[local_index as usize];
        let moved_id = moved_sim.body_id;
        let moved_body = &mut world.bodies[moved_id as usize];
        debug_assert!(moved_body.local_index == moved);
        moved_body.local_index = local_index;
    }

    if set_index == AWAKE_SET {
        let result = {
            let last = world.solver_sets[AWAKE_SET as usize].body_states.len() as i32 - 1;
            world.solver_sets[AWAKE_SET as usize]
                .body_states
                .swap_remove(local_index as usize);
            if local_index < last {
                Some(last)
            } else {
                None
            }
        };
        debug_assert!(result == moved_index);
        let _ = result;
    } else if set_index >= FIRST_SLEEPING_SET
        && world.solver_sets[set_index as usize].body_sims.is_empty()
    {
        destroy_solver_set(world, set_index);
    }

    world.body_id_pool.free_id(body_index);

    let body = &mut world.bodies[body_index as usize];
    body.set_index = NULL_INDEX;
    body.local_index = NULL_INDEX;
    body.id = NULL_INDEX;

    world.validate_solver_sets();
    world.locked = false;
}
