// Tree rebuild with the median-split heuristic from dynamic_tree.c.
// The SAH heuristic (B3_TREE_HEURISTIC == 1) is compiled out upstream and is
// not ported.
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::{DynamicTree, ENLARGED_NODE, TREE_STACK_SIZE};
use crate::core::NULL_INDEX;
use crate::math_functions::{aabb_center, aabb_union, add, max, min, mul_sv, sub, Vec3};

fn max_u16(a: u16, b: u16) -> u16 {
    if a > b {
        a
    } else {
        b
    }
}

/// Median split heuristic. (static b3PartitionMid)
fn partition_mid(indices: &mut [i32], centers: &mut [Vec3]) -> i32 {
    let count = indices.len();

    // Handle trivial case
    if count <= 2 {
        return (count / 2) as i32;
    }

    let mut lower_bound = centers[0];
    let mut upper_bound = centers[0];

    for center in centers.iter().skip(1) {
        lower_bound = min(lower_bound, *center);
        upper_bound = max(upper_bound, *center);
    }

    let d = sub(upper_bound, lower_bound);
    let c = mul_sv(0.5, add(lower_bound, upper_bound));

    // Partition longest axis using the Hoare partition scheme
    // https://en.wikipedia.org/wiki/Quicksort
    // https://nicholasvadivelu.com/2021/01/11/array-partition/
    let (mut i1, mut i2) = (0usize, count);
    if d.x >= d.y && d.x >= d.z {
        let pivot = c.x;

        while i1 < i2 {
            while i1 < i2 && centers[i1].x < pivot {
                i1 += 1;
            }

            while i1 < i2 && centers[i2 - 1].x >= pivot {
                i2 -= 1;
            }

            if i1 < i2 {
                indices.swap(i1, i2 - 1);
                centers.swap(i1, i2 - 1);

                i1 += 1;
                i2 -= 1;
            }
        }
    } else if d.y >= d.z {
        let pivot = c.y;

        while i1 < i2 {
            while i1 < i2 && centers[i1].y < pivot {
                i1 += 1;
            }

            while i1 < i2 && centers[i2 - 1].y >= pivot {
                i2 -= 1;
            }

            if i1 < i2 {
                indices.swap(i1, i2 - 1);
                centers.swap(i1, i2 - 1);

                i1 += 1;
                i2 -= 1;
            }
        }
    } else {
        let pivot = c.z;

        while i1 < i2 {
            while i1 < i2 && centers[i1].z < pivot {
                i1 += 1;
            }

            while i1 < i2 && centers[i2 - 1].z >= pivot {
                i2 -= 1;
            }

            if i1 < i2 {
                indices.swap(i1, i2 - 1);
                centers.swap(i1, i2 - 1);

                i1 += 1;
                i2 -= 1;
            }
        }
    }
    debug_assert!(i1 == i2);

    if i1 > 0 && i1 < count {
        i1 as i32
    } else {
        (count / 2) as i32
    }
}

/// Temporary data used to track the rebuild of a tree node.
/// (struct b3RebuildItem)
#[derive(Clone, Copy, Default)]
struct RebuildItem {
    node_index: i32,
    child_count: i32,
    // Leaf indices
    start_index: i32,
    split_index: i32,
    end_index: i32,
}

impl DynamicTree {
    /// Returns the root node index. (static b3BuildTree)
    fn build_tree(&mut self, leaf_count: i32) -> i32 {
        if leaf_count == 1 {
            let leaf = self.leaf_indices[0];
            self.nodes[leaf as usize].parent = NULL_INDEX;
            return leaf;
        }

        let mut stack = vec![RebuildItem::default(); TREE_STACK_SIZE];
        let mut top = 0usize;

        stack[0].node_index = self.allocate_node();
        stack[0].child_count = -1;
        stack[0].start_index = 0;
        stack[0].end_index = leaf_count;
        stack[0].split_index = {
            // Split borrows: partition operates on the rebuild scratch arrays.
            let (indices, centers) = (&mut self.leaf_indices, &mut self.leaf_centers);
            partition_mid(
                &mut indices[..leaf_count as usize],
                &mut centers[..leaf_count as usize],
            )
        };

        loop {
            stack[top].child_count += 1;
            let item = stack[top];

            if item.child_count == 2 {
                // This internal node has both children established

                if top == 0 {
                    // all done
                    break;
                }

                let parent_item = stack[top - 1];

                if parent_item.child_count == 0 {
                    debug_assert!(self.nodes[parent_item.node_index as usize].child1 == NULL_INDEX);
                    self.nodes[parent_item.node_index as usize].child1 = item.node_index;
                } else {
                    debug_assert!(parent_item.child_count == 1);
                    debug_assert!(self.nodes[parent_item.node_index as usize].child2 == NULL_INDEX);
                    self.nodes[parent_item.node_index as usize].child2 = item.node_index;
                }

                let node_index = item.node_index as usize;
                debug_assert!(self.nodes[node_index].parent == NULL_INDEX);
                self.nodes[node_index].parent = parent_item.node_index;

                debug_assert!(self.nodes[node_index].child1 != NULL_INDEX);
                debug_assert!(self.nodes[node_index].child2 != NULL_INDEX);
                let c1 = self.nodes[node_index].child1 as usize;
                let c2 = self.nodes[node_index].child2 as usize;

                self.nodes[node_index].aabb = aabb_union(self.nodes[c1].aabb, self.nodes[c2].aabb);
                self.nodes[node_index].height =
                    1 + max_u16(self.nodes[c1].height, self.nodes[c2].height);
                self.nodes[node_index].category_bits =
                    self.nodes[c1].category_bits | self.nodes[c2].category_bits;

                // Pop stack
                top -= 1;
            } else {
                let (start_index, end_index) = if item.child_count == 0 {
                    (item.start_index, item.split_index)
                } else {
                    debug_assert!(item.child_count == 1);
                    (item.split_index, item.end_index)
                };

                let count = end_index - start_index;

                if count == 1 {
                    let child_index = self.leaf_indices[start_index as usize];
                    let node_index = item.node_index as usize;

                    if item.child_count == 0 {
                        debug_assert!(self.nodes[node_index].child1 == NULL_INDEX);
                        self.nodes[node_index].child1 = child_index;
                    } else {
                        debug_assert!(item.child_count == 1);
                        debug_assert!(self.nodes[node_index].child2 == NULL_INDEX);
                        self.nodes[node_index].child2 = child_index;
                    }

                    debug_assert!(self.nodes[child_index as usize].parent == NULL_INDEX);
                    self.nodes[child_index as usize].parent = item.node_index;
                } else {
                    debug_assert!(count > 0);
                    debug_assert!(top < TREE_STACK_SIZE);

                    top += 1;
                    let node_index = self.allocate_node();
                    let split_index = {
                        let (s, e) = (start_index as usize, end_index as usize);
                        let (indices, centers) = (&mut self.leaf_indices, &mut self.leaf_centers);
                        partition_mid(&mut indices[s..e], &mut centers[s..e])
                    };
                    let new_item = &mut stack[top];
                    new_item.node_index = node_index;
                    new_item.child_count = -1;
                    new_item.start_index = start_index;
                    new_item.end_index = end_index;
                    new_item.split_index = split_index + start_index;
                }
            }
        }

        let root_index = stack[0].node_index as usize;
        debug_assert!(self.nodes[root_index].parent == NULL_INDEX);
        debug_assert!(self.nodes[root_index].child1 != NULL_INDEX);
        debug_assert!(self.nodes[root_index].child2 != NULL_INDEX);

        let c1 = self.nodes[root_index].child1 as usize;
        let c2 = self.nodes[root_index].child2 as usize;

        self.nodes[root_index].aabb = aabb_union(self.nodes[c1].aabb, self.nodes[c2].aabb);
        self.nodes[root_index].height = 1 + max_u16(self.nodes[c1].height, self.nodes[c2].height);
        self.nodes[root_index].category_bits =
            self.nodes[c1].category_bits | self.nodes[c2].category_bits;

        stack[0].node_index
    }

    /// Rebuild the tree while retaining subtrees that haven't changed.
    /// Returns the number of boxes sorted. (b3DynamicTree_Rebuild)
    pub fn rebuild(&mut self, full_build: bool) -> i32 {
        let proxy_count = self.proxy_count;
        if proxy_count == 0 {
            return 0;
        }

        // Ensure capacity for rebuild space
        if proxy_count > self.rebuild_capacity {
            let new_capacity = proxy_count + proxy_count / 2;

            self.leaf_indices = vec![0; new_capacity as usize];
            self.leaf_centers = vec![Vec3::default(); new_capacity as usize];
            self.rebuild_capacity = new_capacity;
        }

        let mut leaf_count = 0usize;
        let mut stack = [0i32; TREE_STACK_SIZE];
        let mut stack_count = 0usize;

        let mut node_index = self.root;

        // Gather all proxy nodes that have grown and all internal nodes that
        // haven't grown. Both are considered leaves in the tree rebuild.
        // Free all internal nodes that have grown.
        loop {
            let node = self.nodes[node_index as usize];
            if node.is_leaf() || (node.flags & ENLARGED_NODE == 0 && !full_build) {
                self.leaf_indices[leaf_count] = node_index;
                self.leaf_centers[leaf_count] = aabb_center(node.aabb);
                leaf_count += 1;

                // Detach
                self.nodes[node_index as usize].parent = NULL_INDEX;
            } else {
                let doomed_node_index = node_index;

                // Handle children
                node_index = node.child1;

                debug_assert!(stack_count < TREE_STACK_SIZE);
                if stack_count < TREE_STACK_SIZE {
                    stack[stack_count] = node.child2;
                    stack_count += 1;
                }

                // Remove doomed node
                self.free_node(doomed_node_index);

                continue;
            }

            if stack_count == 0 {
                break;
            }

            stack_count -= 1;
            node_index = stack[stack_count];
        }

        debug_assert!(leaf_count <= proxy_count as usize);

        self.root = self.build_tree(leaf_count as i32);

        self.validate();

        leaf_count as i32
    }
}
