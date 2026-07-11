// Tree validation from dynamic_tree.c. The C versions are compiled in only
// with B3_ENABLE_VALIDATION; here they always run when called (they are only
// invoked explicitly and from rebuild) and assert in debug builds.
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::{DynamicTree, ALLOCATED_NODE, ENLARGED_NODE};
use crate::core::NULL_INDEX;
use crate::math_functions::{aabb_contains, is_valid_aabb, max_int};

impl DynamicTree {
    /// Compute the height of a sub-tree. (static b3ComputeHeightRecurse)
    fn compute_height(&self, node_id: i32) -> i32 {
        debug_assert!(0 <= node_id && node_id < self.node_capacity());
        let node = &self.nodes[node_id as usize];

        if node.is_leaf() {
            return 0;
        }

        let height1 = self.compute_height(node.child1);
        let height2 = self.compute_height(node.child2);
        1 + max_int(height1, height2)
    }

    /// (static b3ValidateStructure)
    fn validate_structure(&self, index: i32) {
        if index == NULL_INDEX {
            return;
        }

        if index == self.root {
            debug_assert!(self.nodes[index as usize].parent == NULL_INDEX);
        }

        let node = &self.nodes[index as usize];

        debug_assert!(node.flags == 0 || (node.flags & ALLOCATED_NODE) != 0);

        if node.is_leaf() {
            debug_assert!(node.height == 0);
            return;
        }

        let child1 = node.child1;
        let child2 = node.child2;

        debug_assert!(0 <= child1 && child1 < self.node_capacity());
        debug_assert!(0 <= child2 && child2 < self.node_capacity());

        debug_assert!(self.nodes[child1 as usize].parent == index);
        debug_assert!(self.nodes[child2 as usize].parent == index);

        if (self.nodes[child1 as usize].flags | self.nodes[child2 as usize].flags) & ENLARGED_NODE
            != 0
        {
            debug_assert!(node.flags & ENLARGED_NODE != 0);
        }

        self.validate_structure(child1);
        self.validate_structure(child2);
    }

    /// (static b3ValidateMetrics)
    fn validate_metrics(&self, index: i32) {
        if index == NULL_INDEX {
            return;
        }

        let node = &self.nodes[index as usize];

        debug_assert!(is_valid_aabb(node.aabb));

        if node.is_leaf() {
            debug_assert!(node.height == 0);
            return;
        }

        let child1 = node.child1;
        let child2 = node.child2;

        debug_assert!(0 <= child1 && child1 < self.node_capacity());
        debug_assert!(0 <= child2 && child2 < self.node_capacity());

        let height1 = self.nodes[child1 as usize].height as i32;
        let height2 = self.nodes[child2 as usize].height as i32;
        let height = 1 + max_int(height1, height2);
        debug_assert!(node.height as i32 == height);

        debug_assert!(aabb_contains(node.aabb, self.nodes[child1 as usize].aabb));
        debug_assert!(aabb_contains(node.aabb, self.nodes[child2 as usize].aabb));

        let category_bits =
            self.nodes[child1 as usize].category_bits | self.nodes[child2 as usize].category_bits;
        debug_assert!(node.category_bits == category_bits);

        self.validate_metrics(child1);
        self.validate_metrics(child2);
    }

    /// Validate this tree. For testing. (b3DynamicTree_Validate)
    pub fn validate(&self) {
        if self.root == NULL_INDEX {
            return;
        }

        self.validate_structure(self.root);
        self.validate_metrics(self.root);

        let mut free_count = 0;
        let mut free_index = self.free_list;
        while free_index != NULL_INDEX {
            debug_assert!(0 <= free_index && free_index < self.node_capacity());
            free_index = self.nodes[free_index as usize].next;
            free_count += 1;
        }

        let height = self.height();
        let computed_height = self.compute_height(self.root);
        debug_assert!(height == computed_height);
        let _ = (height, computed_height);

        debug_assert!(self.node_count + free_count == self.node_capacity());
        let _ = free_count;
    }

    /// Validate this tree has no enlarged AABBs. For testing.
    /// (b3DynamicTree_ValidateNoEnlarged)
    pub fn validate_no_enlarged(&self) {
        for node in &self.nodes {
            if node.flags & ALLOCATED_NODE != 0 {
                debug_assert!(node.flags & ENLARGED_NODE == 0);
            }
        }
    }
}
