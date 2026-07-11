// Port of the constraint graph data model from
// box3d-cpp-reference/src/constraint_graph.h. Logic from constraint_graph.c
// lands in the solver bring-up commit.
//
// The C b3GraphColor carries transient pointers into arena scratch
// (wideConstraints / manifoldConstraints / contactConstraints), rebuilt every
// step by the solver. The Rust solver phase owns that scratch as Vecs local to
// the step; the persistent graph state here is the bitset and the sim arrays.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::bitset::BitSet;
use crate::constants::GRAPH_COLOR_COUNT;
use crate::contact::ContactSpec;
use crate::joint::JointSim;
use crate::math_functions::max_int;

/// Constraints that cannot fit the graph color limit. (B3_OVERFLOW_INDEX)
pub const OVERFLOW_INDEX: i32 = GRAPH_COLOR_COUNT - 1;

/// Keeps dyn-dyn constraints at lower solver priority than dyn-static.
/// (B3_DYNAMIC_COLOR_COUNT)
pub const DYNAMIC_COLOR_COUNT: i32 = GRAPH_COLOR_COUNT - 4;

/// (b3GraphColor)
#[derive(Debug, Clone, Default)]
pub struct GraphColor {
    /// Indexed by bodyId; oversized to encompass static bodies. Bits are never
    /// traversed or counted. Unused on the overflow color.
    pub body_set: BitSet,

    /// Cache friendly arrays
    pub joint_sims: Vec<JointSim>,

    pub convex_contacts: Vec<i32>,
    pub contacts: Vec<ContactSpec>,
}

/// (b3ConstraintGraph)
#[derive(Debug, Clone, Default)]
pub struct ConstraintGraph {
    /// Including overflow at the end
    pub colors: Vec<GraphColor>,
}

impl ConstraintGraph {
    /// (b3CreateGraph)
    pub fn new(body_capacity: i32) -> ConstraintGraph {
        const _: () = assert!(GRAPH_COLOR_COUNT >= 2, "must have at least two colors");
        const _: () = assert!(
            OVERFLOW_INDEX == GRAPH_COLOR_COUNT - 1,
            "bad overflow index"
        );

        let body_capacity = max_int(body_capacity, 8);

        let mut colors = Vec::with_capacity(GRAPH_COLOR_COUNT as usize);
        // Initialize graph color bit set. No bitset for overflow color.
        for i in 0..GRAPH_COLOR_COUNT {
            let mut color = GraphColor::default();
            if i < OVERFLOW_INDEX {
                color.body_set = BitSet::new(body_capacity as u32);
                color.body_set.set_bit_count_and_clear(body_capacity as u32);
            }
            colors.push(color);
        }

        ConstraintGraph { colors }
    }
}

/// Contacts are created non-touching; once touching they enter the graph.
/// (b3AddContactToGraph)
pub fn add_contact_to_graph(world: &mut crate::world::World, contact_id: i32) {
    use crate::contact::contact_flags;
    use crate::core::NULL_INDEX;
    use crate::types::BodyType;

    let contact = &world.contacts[contact_id as usize];
    debug_assert!(contact.manifold_count() > 0);
    debug_assert!((contact.flags & contact_flags::TOUCHING) != 0);

    let body_id_a = contact.edges[0].body_id;
    let body_id_b = contact.edges[1].body_id;
    let type_a = world.bodies[body_id_a as usize].type_;
    let type_b = world.bodies[body_id_b as usize].type_;
    debug_assert!(type_a == BodyType::Dynamic || type_b == BodyType::Dynamic);

    let mut color_index = OVERFLOW_INDEX;

    if type_a == BodyType::Dynamic && type_b == BodyType::Dynamic {
        for i in 0..DYNAMIC_COLOR_COUNT {
            let color = &world.constraint_graph.colors[i as usize];
            if color.body_set.get_bit(body_id_a as u32) || color.body_set.get_bit(body_id_b as u32)
            {
                continue;
            }
            world.constraint_graph.colors[i as usize]
                .body_set
                .set_bit_grow(body_id_a as u32);
            world.constraint_graph.colors[i as usize]
                .body_set
                .set_bit_grow(body_id_b as u32);
            color_index = i;
            break;
        }
    } else if type_a == BodyType::Dynamic {
        for i in (1..OVERFLOW_INDEX).rev() {
            if world.constraint_graph.colors[i as usize]
                .body_set
                .get_bit(body_id_a as u32)
            {
                continue;
            }
            world.constraint_graph.colors[i as usize]
                .body_set
                .set_bit_grow(body_id_a as u32);
            color_index = i;
            break;
        }
    } else if type_b == BodyType::Dynamic {
        for i in (1..OVERFLOW_INDEX).rev() {
            if world.constraint_graph.colors[i as usize]
                .body_set
                .get_bit(body_id_b as u32)
            {
                continue;
            }
            world.constraint_graph.colors[i as usize]
                .body_set
                .set_bit_grow(body_id_b as u32);
            color_index = i;
            break;
        }
    }

    let is_mesh = (world.contacts[contact_id as usize].flags & contact_flags::SIM_MESH_CONTACT) != 0;
    let is_scalar = is_mesh || color_index == OVERFLOW_INDEX;
    let local_index = if is_scalar {
        world.constraint_graph.colors[color_index as usize]
            .contacts
            .len() as i32
    } else {
        world.constraint_graph.colors[color_index as usize]
            .convex_contacts
            .len() as i32
    };

    let local_a = world.bodies[body_id_a as usize].local_index;
    let local_b = world.bodies[body_id_b as usize].local_index;
    let manifold_count = world.contacts[contact_id as usize].manifold_count();

    {
        let contact = &mut world.contacts[contact_id as usize];
        contact.color_index = color_index;
        contact.local_index = local_index;
        contact.body_sim_index_a = if type_a == BodyType::Static {
            NULL_INDEX
        } else {
            local_a
        };
        contact.body_sim_index_b = if type_b == BodyType::Static {
            NULL_INDEX
        } else {
            local_b
        };
    }

    let color = &mut world.constraint_graph.colors[color_index as usize];
    if is_scalar {
        debug_assert!(manifold_count < u16::MAX as i32);
        color.contacts.push(ContactSpec {
            contact_id,
            manifold_start: 0,
            manifold_count: manifold_count as u16,
        });
    } else {
        color.convex_contacts.push(contact_id);
    }
}

/// Pick a graph color for a joint between the two bodies. (b3AssignJointColor)
///
/// C compiles the coloring away when B3_FORCE_OVERFLOW is set; the reference
/// pins it to 0, so the coloring always runs.
fn assign_joint_color(
    graph: &mut ConstraintGraph,
    body_id_a: i32,
    body_id_b: i32,
    type_a: crate::types::BodyType,
    type_b: crate::types::BodyType,
) -> i32 {
    use crate::types::BodyType;

    debug_assert!(type_a == BodyType::Dynamic || type_b == BodyType::Dynamic);

    if type_a == BodyType::Dynamic && type_b == BodyType::Dynamic {
        // Dynamic constraint colors cannot encroach on colors reserved for static constraints
        for i in 0..DYNAMIC_COLOR_COUNT {
            let color = &graph.colors[i as usize];
            if color.body_set.get_bit(body_id_a as u32) || color.body_set.get_bit(body_id_b as u32)
            {
                continue;
            }

            let color = &mut graph.colors[i as usize];
            color.body_set.set_bit_grow(body_id_a as u32);
            color.body_set.set_bit_grow(body_id_b as u32);
            return i;
        }
    } else if type_a == BodyType::Dynamic {
        // Static constraint colors build from the end to get higher priority than dyn-dyn constraints
        for i in (1..OVERFLOW_INDEX).rev() {
            let color = &graph.colors[i as usize];
            if color.body_set.get_bit(body_id_a as u32) {
                continue;
            }

            graph.colors[i as usize]
                .body_set
                .set_bit_grow(body_id_a as u32);
            return i;
        }
    } else if type_b == BodyType::Dynamic {
        // Static constraint colors build from the end to get higher priority than dyn-dyn constraints
        for i in (1..OVERFLOW_INDEX).rev() {
            let color = &graph.colors[i as usize];
            if color.body_set.get_bit(body_id_b as u32) {
                continue;
            }

            graph.colors[i as usize]
                .body_set
                .set_bit_grow(body_id_b as u32);
            return i;
        }
    }

    OVERFLOW_INDEX
}

/// Allocate a zeroed joint sim slot in the graph for the joint and set the
/// joint's color/local indices. Returns (color_index, local_index).
/// (b3CreateJointInGraph — C returns the sim pointer; the caller writes it)
pub fn create_joint_in_graph(world: &mut crate::world::World, joint_id: i32) -> (i32, i32) {
    let body_id_a = world.joints[joint_id as usize].edges[0].body_id;
    let body_id_b = world.joints[joint_id as usize].edges[1].body_id;
    let type_a = world.bodies[body_id_a as usize].type_;
    let type_b = world.bodies[body_id_b as usize].type_;

    let color_index = assign_joint_color(
        &mut world.constraint_graph,
        body_id_a,
        body_id_b,
        type_a,
        type_b,
    );

    let color = &mut world.constraint_graph.colors[color_index as usize];
    let local_index = color.joint_sims.len() as i32;
    color.joint_sims.push(crate::joint::JointSim::default());

    let joint = &mut world.joints[joint_id as usize];
    joint.color_index = color_index;
    joint.local_index = local_index;
    (color_index, local_index)
}

/// (b3AddJointToGraph)
pub fn add_joint_to_graph(
    world: &mut crate::world::World,
    joint_sim: crate::joint::JointSim,
    joint_id: i32,
) {
    let (color_index, local_index) = create_joint_in_graph(world, joint_id);
    world.constraint_graph.colors[color_index as usize].joint_sims[local_index as usize] =
        joint_sim;
}

/// (b3RemoveJointFromGraph)
pub fn remove_joint_from_graph(
    world: &mut crate::world::World,
    body_id_a: i32,
    body_id_b: i32,
    color_index: i32,
    local_index: i32,
) {
    debug_assert!(0 <= color_index && color_index < GRAPH_COLOR_COUNT);

    if color_index != OVERFLOW_INDEX {
        // May clear static bodies, no effect
        let color = &mut world.constraint_graph.colors[color_index as usize];
        color.body_set.clear_bit(body_id_a as u32);
        color.body_set.clear_bit(body_id_b as u32);
    }

    let color = &mut world.constraint_graph.colors[color_index as usize];
    let moved_index = color.joint_sims.len() as i32 - 1;
    color.joint_sims.swap_remove(local_index as usize);
    if moved_index != local_index {
        // Fix moved joint
        let moved_id = color.joint_sims[local_index as usize].joint_id;
        let moved_joint = &mut world.joints[moved_id as usize];
        debug_assert!(moved_joint.set_index == crate::solver_set::AWAKE_SET);
        debug_assert!(moved_joint.color_index == color_index);
        debug_assert!(moved_joint.local_index == moved_index);
        moved_joint.local_index = local_index;
    }
}

/// (b3RemoveContactFromGraph)
pub fn remove_contact_from_graph(
    world: &mut crate::world::World,
    body_id_a: i32,
    body_id_b: i32,
    color_index: i32,
    local_index: i32,
    mesh_contact: bool,
) {
    debug_assert!(0 <= color_index && color_index < GRAPH_COLOR_COUNT);

    if color_index != OVERFLOW_INDEX {
        let color = &mut world.constraint_graph.colors[color_index as usize];
        color.body_set.clear_bit(body_id_a as u32);
        color.body_set.clear_bit(body_id_b as u32);
    }

    let color = &mut world.constraint_graph.colors[color_index as usize];
    if mesh_contact || color_index == OVERFLOW_INDEX {
        let moved_index = color.contacts.len() as i32 - 1;
        color.contacts.swap_remove(local_index as usize);
        if moved_index != local_index {
            let moved_contact_id = color.contacts[local_index as usize].contact_id;
            let moved = &mut world.contacts[moved_contact_id as usize];
            debug_assert!(moved.set_index == crate::solver_set::AWAKE_SET);
            debug_assert!(moved.color_index == color_index);
            debug_assert!(moved.local_index == moved_index);
            moved.local_index = local_index;
        }
    } else {
        let moved_index = color.convex_contacts.len() as i32 - 1;
        color.convex_contacts.swap_remove(local_index as usize);
        if moved_index != local_index {
            let moved_contact_id = color.convex_contacts[local_index as usize];
            let moved = &mut world.contacts[moved_contact_id as usize];
            debug_assert!(moved.set_index == crate::solver_set::AWAKE_SET);
            debug_assert!(moved.color_index == color_index);
            debug_assert!(moved.local_index == moved_index);
            debug_assert!(
                (moved.flags & crate::contact::contact_flags::SIM_MESH_CONTACT) == 0
            );
            moved.local_index = local_index;
        }
    }
}
