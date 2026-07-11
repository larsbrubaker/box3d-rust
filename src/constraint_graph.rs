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
