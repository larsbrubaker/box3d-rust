// Leaf insertion/removal, SAH sibling selection, rotations, and proxy
// operations from dynamic_tree.c.
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::{DynamicTree, ALLOCATED_NODE, ENLARGED_NODE, LEAF_NODE};
use crate::aabb::{enlarge_aabb, perimeter};
use crate::core::NULL_INDEX;
use crate::math_functions::{
    aabb_center, aabb_contains, aabb_union, is_valid_aabb, length_squared, min_float, sub, Aabb,
};

fn max_u16(a: u16, b: u16) -> u16 {
    if a > b {
        a
    } else {
        b
    }
}

// Greedy algorithm for sibling selection using the SAH
// We have three nodes A-(B,C) and want to add a leaf D, there are three choices.
// 1: make a new parent for A and D : E-(A-(B,C), D)
// 2: associate D with B
//   a: B is a leaf : A-(E-(B,D), C)
//   b: B is an internal node: A-(B{D},C)
// 3: associate D with C
//   a: C is a leaf : A-(B, E-(C,D))
//   b: C is an internal node: A-(B, C{D})
// All of these have a clear cost except when B or C is an internal node.
// Hence we need to be greedy.
//
// The cost for cases 1, 2a, and 3a can be computed using the sibling cost
// formula: cost of sibling H = area(union(H, D)) + increased area of ancestors
//
// Suppose B (or C) is an internal node, then the lowest cost would be one of
// two cases:
// case1: D becomes a sibling of B
// case2: D becomes a descendant of B along with a new internal node of area(D).
fn find_best_sibling(tree: &DynamicTree, box_d: Aabb) -> i32 {
    let center_d = aabb_center(box_d);
    let area_d = perimeter(box_d);

    let nodes = &tree.nodes;
    let root_index = tree.root;

    let root_box = nodes[root_index as usize].aabb;

    // Area of current node
    let mut area_base = perimeter(root_box);

    // Area of inflated node
    let mut direct_cost = perimeter(aabb_union(root_box, box_d));
    let mut inherited_cost = 0.0;

    let mut best_sibling = root_index;
    let mut best_cost = direct_cost;

    // Descend the tree from root, following a single greedy path.
    let mut index = root_index;
    while !nodes[index as usize].is_leaf() {
        let child1 = nodes[index as usize].child1;
        let child2 = nodes[index as usize].child2;

        // Cost of creating a new parent for this node and the new leaf
        let cost = direct_cost + inherited_cost;

        // Sometimes there are multiple identical costs within tolerance.
        // This breaks the ties using the centroid distance.
        if cost < best_cost {
            best_sibling = index;
            best_cost = cost;
        }

        // Inheritance cost seen by children
        inherited_cost += direct_cost - area_base;

        let leaf1 = nodes[child1 as usize].is_leaf();
        let leaf2 = nodes[child2 as usize].is_leaf();

        // Cost of descending into child 1
        let mut lower_cost1 = f32::MAX;
        let box1 = nodes[child1 as usize].aabb;
        let direct_cost1 = perimeter(aabb_union(box1, box_d));
        let mut area1 = 0.0;
        if leaf1 {
            // Child 1 is a leaf
            // Cost of creating new node and increasing area of node P
            let cost1 = direct_cost1 + inherited_cost;

            // Need this here due to while condition above
            if cost1 < best_cost {
                best_sibling = child1;
                best_cost = cost1;
            }
        } else {
            // Child 1 is an internal node
            area1 = perimeter(box1);

            // Lower bound cost of inserting under child 1.
            lower_cost1 = inherited_cost + direct_cost1 + min_float(area_d - area1, 0.0);
        }

        // Cost of descending into child 2
        let mut lower_cost2 = f32::MAX;
        let box2 = nodes[child2 as usize].aabb;
        let direct_cost2 = perimeter(aabb_union(box2, box_d));
        let mut area2 = 0.0;
        if leaf2 {
            // Child 2 is a leaf
            // Cost of creating new node and increasing area of node P
            let cost2 = direct_cost2 + inherited_cost;

            // Need this here due to while condition above
            if cost2 < best_cost {
                best_sibling = child2;
                best_cost = cost2;
            }
        } else {
            // Child 2 is an internal node
            area2 = perimeter(box2);

            // Lower bound cost of inserting under child 2.
            lower_cost2 = inherited_cost + direct_cost2 + min_float(area_d - area2, 0.0);
        }

        if leaf1 && leaf2 {
            break;
        }

        // Can the cost possibly be decreased?
        if best_cost <= lower_cost1 && best_cost <= lower_cost2 {
            break;
        }

        if lower_cost1 == lower_cost2 && !leaf1 {
            debug_assert!(lower_cost1 < f32::MAX);
            debug_assert!(lower_cost2 < f32::MAX);

            // No clear choice based on lower bound surface area. This can
            // happen when both children fully contain D. Fall back to node
            // distance.
            let d1 = sub(aabb_center(box1), center_d);
            let d2 = sub(aabb_center(box2), center_d);
            lower_cost1 = length_squared(d1);
            lower_cost2 = length_squared(d2);
        }

        // Descend
        if lower_cost1 < lower_cost2 && !leaf1 {
            index = child1;
            area_base = area1;
            direct_cost = direct_cost1;
        } else {
            index = child2;
            area_base = area2;
            direct_cost = direct_cost2;
        }

        debug_assert!(!nodes[index as usize].is_leaf());
    }

    best_sibling
}

/// (enum b3RotateType)
enum RotateType {
    None,
    Bf,
    Bg,
    Cd,
    Ce,
}

impl DynamicTree {
    /// Perform a left or right rotation if node A is imbalanced.
    /// (static b3RotateNodes)
    fn rotate_nodes(&mut self, i_a: i32) {
        debug_assert!(i_a != NULL_INDEX);

        let nodes = &mut self.nodes;

        if nodes[i_a as usize].is_leaf() {
            return;
        }

        let i_b = nodes[i_a as usize].child1;
        let i_c = nodes[i_a as usize].child2;
        debug_assert!(0 <= i_b && (i_b as usize) < nodes.len());
        debug_assert!(0 <= i_c && (i_c as usize) < nodes.len());

        let (ia, ib, ic) = (i_a as usize, i_b as usize, i_c as usize);

        let is_leaf_b = nodes[ib].is_leaf();
        let is_leaf_c = nodes[ic].is_leaf();

        if is_leaf_b && !is_leaf_c {
            // B is a leaf and C is internal
            let i_f = nodes[ic].child1;
            let i_g = nodes[ic].child2;
            let (if_, ig) = (i_f as usize, i_g as usize);
            debug_assert!(0 <= i_f && (i_f as usize) < nodes.len());
            debug_assert!(0 <= i_g && (i_g as usize) < nodes.len());

            // Base cost
            let cost_base = perimeter(nodes[ic].aabb);

            // Cost of swapping B and F
            let aabb_bg = aabb_union(nodes[ib].aabb, nodes[ig].aabb);
            let cost_bf = perimeter(aabb_bg);

            // Cost of swapping B and G
            let aabb_bf = aabb_union(nodes[ib].aabb, nodes[if_].aabb);
            let cost_bg = perimeter(aabb_bf);

            if cost_base < cost_bf && cost_base < cost_bg {
                // Rotation does not improve cost
                return;
            }

            if cost_bf < cost_bg {
                // Swap B and F
                nodes[ia].child1 = i_f;
                nodes[ic].child1 = i_b;

                nodes[ib].parent = i_c;
                nodes[if_].parent = i_a;

                nodes[ic].aabb = aabb_bg;

                nodes[ic].height = 1 + max_u16(nodes[ib].height, nodes[ig].height);
                nodes[ia].height = 1 + max_u16(nodes[ic].height, nodes[if_].height);
                nodes[ic].category_bits = nodes[ib].category_bits | nodes[ig].category_bits;
                nodes[ia].category_bits = nodes[ic].category_bits | nodes[if_].category_bits;
                nodes[ic].flags |= (nodes[ib].flags | nodes[ig].flags) & ENLARGED_NODE;
                nodes[ia].flags |= (nodes[ic].flags | nodes[if_].flags) & ENLARGED_NODE;
            } else {
                // Swap B and G
                nodes[ia].child1 = i_g;
                nodes[ic].child2 = i_b;

                nodes[ib].parent = i_c;
                nodes[ig].parent = i_a;

                nodes[ic].aabb = aabb_bf;

                nodes[ic].height = 1 + max_u16(nodes[ib].height, nodes[if_].height);
                nodes[ia].height = 1 + max_u16(nodes[ic].height, nodes[ig].height);
                nodes[ic].category_bits = nodes[ib].category_bits | nodes[if_].category_bits;
                nodes[ia].category_bits = nodes[ic].category_bits | nodes[ig].category_bits;
                nodes[ic].flags |= (nodes[ib].flags | nodes[if_].flags) & ENLARGED_NODE;
                nodes[ia].flags |= (nodes[ic].flags | nodes[ig].flags) & ENLARGED_NODE;
            }
        } else if is_leaf_c && !is_leaf_b {
            // C is a leaf and B is internal
            let i_d = nodes[ib].child1;
            let i_e = nodes[ib].child2;
            let (id, ie) = (i_d as usize, i_e as usize);
            debug_assert!(0 <= i_d && (i_d as usize) < nodes.len());
            debug_assert!(0 <= i_e && (i_e as usize) < nodes.len());

            // Base cost
            let cost_base = perimeter(nodes[ib].aabb);

            // Cost of swapping C and D
            let aabb_ce = aabb_union(nodes[ic].aabb, nodes[ie].aabb);
            let cost_cd = perimeter(aabb_ce);

            // Cost of swapping C and E
            let aabb_cd = aabb_union(nodes[ic].aabb, nodes[id].aabb);
            let cost_ce = perimeter(aabb_cd);

            if cost_base < cost_cd && cost_base < cost_ce {
                // Rotation does not improve cost
                return;
            }

            if cost_cd < cost_ce {
                // Swap C and D
                nodes[ia].child2 = i_d;
                nodes[ib].child1 = i_c;

                nodes[ic].parent = i_b;
                nodes[id].parent = i_a;

                nodes[ib].aabb = aabb_ce;

                nodes[ib].height = 1 + max_u16(nodes[ic].height, nodes[ie].height);
                nodes[ia].height = 1 + max_u16(nodes[ib].height, nodes[id].height);
                nodes[ib].category_bits = nodes[ic].category_bits | nodes[ie].category_bits;
                nodes[ia].category_bits = nodes[ib].category_bits | nodes[id].category_bits;
                nodes[ib].flags |= (nodes[ic].flags | nodes[ie].flags) & ENLARGED_NODE;
                nodes[ia].flags |= (nodes[ib].flags | nodes[id].flags) & ENLARGED_NODE;
            } else {
                // Swap C and E
                nodes[ia].child2 = i_e;
                nodes[ib].child2 = i_c;

                nodes[ic].parent = i_b;
                nodes[ie].parent = i_a;

                nodes[ib].aabb = aabb_cd;
                nodes[ib].height = 1 + max_u16(nodes[ic].height, nodes[id].height);
                nodes[ia].height = 1 + max_u16(nodes[ib].height, nodes[ie].height);
                nodes[ib].category_bits = nodes[ic].category_bits | nodes[id].category_bits;
                nodes[ia].category_bits = nodes[ib].category_bits | nodes[ie].category_bits;
                nodes[ib].flags |= (nodes[ic].flags | nodes[id].flags) & ENLARGED_NODE;
                nodes[ia].flags |= (nodes[ib].flags | nodes[ie].flags) & ENLARGED_NODE;
            }
        } else if !is_leaf_b && !is_leaf_c {
            let i_d = nodes[ib].child1;
            let i_e = nodes[ib].child2;
            let i_f = nodes[ic].child1;
            let i_g = nodes[ic].child2;

            let (id, ie, if_, ig) = (i_d as usize, i_e as usize, i_f as usize, i_g as usize);

            debug_assert!(0 <= i_d && (i_d as usize) < nodes.len());
            debug_assert!(0 <= i_e && (i_e as usize) < nodes.len());
            debug_assert!(0 <= i_f && (i_f as usize) < nodes.len());
            debug_assert!(0 <= i_g && (i_g as usize) < nodes.len());

            // Base cost
            let area_b = perimeter(nodes[ib].aabb);
            let area_c = perimeter(nodes[ic].aabb);
            let cost_base = area_b + area_c;
            let mut best_rotation = RotateType::None;
            let mut best_cost = cost_base;

            // Cost of swapping B and F
            let aabb_bg = aabb_union(nodes[ib].aabb, nodes[ig].aabb);
            let cost_bf = area_b + perimeter(aabb_bg);
            if cost_bf < best_cost {
                best_rotation = RotateType::Bf;
                best_cost = cost_bf;
            }

            // Cost of swapping B and G
            let aabb_bf = aabb_union(nodes[ib].aabb, nodes[if_].aabb);
            let cost_bg = area_b + perimeter(aabb_bf);
            if cost_bg < best_cost {
                best_rotation = RotateType::Bg;
                best_cost = cost_bg;
            }

            // Cost of swapping C and D
            let aabb_ce = aabb_union(nodes[ic].aabb, nodes[ie].aabb);
            let cost_cd = area_c + perimeter(aabb_ce);
            if cost_cd < best_cost {
                best_rotation = RotateType::Cd;
                best_cost = cost_cd;
            }

            // Cost of swapping C and E
            let aabb_cd = aabb_union(nodes[ic].aabb, nodes[id].aabb);
            let cost_ce = area_c + perimeter(aabb_cd);
            if cost_ce < best_cost {
                best_rotation = RotateType::Ce;
                // best_cost = cost_ce;
            }

            match best_rotation {
                RotateType::None => {}

                RotateType::Bf => {
                    nodes[ia].child1 = i_f;
                    nodes[ic].child1 = i_b;

                    nodes[ib].parent = i_c;
                    nodes[if_].parent = i_a;

                    nodes[ic].aabb = aabb_bg;
                    nodes[ic].height = 1 + max_u16(nodes[ib].height, nodes[ig].height);
                    nodes[ia].height = 1 + max_u16(nodes[ic].height, nodes[if_].height);
                    nodes[ic].category_bits = nodes[ib].category_bits | nodes[ig].category_bits;
                    nodes[ia].category_bits = nodes[ic].category_bits | nodes[if_].category_bits;
                    nodes[ic].flags |= (nodes[ib].flags | nodes[ig].flags) & ENLARGED_NODE;
                    nodes[ia].flags |= (nodes[ic].flags | nodes[if_].flags) & ENLARGED_NODE;
                }

                RotateType::Bg => {
                    nodes[ia].child1 = i_g;
                    nodes[ic].child2 = i_b;

                    nodes[ib].parent = i_c;
                    nodes[ig].parent = i_a;

                    nodes[ic].aabb = aabb_bf;
                    nodes[ic].height = 1 + max_u16(nodes[ib].height, nodes[if_].height);
                    nodes[ia].height = 1 + max_u16(nodes[ic].height, nodes[ig].height);
                    nodes[ic].category_bits = nodes[ib].category_bits | nodes[if_].category_bits;
                    nodes[ia].category_bits = nodes[ic].category_bits | nodes[ig].category_bits;
                    nodes[ic].flags |= (nodes[ib].flags | nodes[if_].flags) & ENLARGED_NODE;
                    nodes[ia].flags |= (nodes[ic].flags | nodes[ig].flags) & ENLARGED_NODE;
                }

                RotateType::Cd => {
                    nodes[ia].child2 = i_d;
                    nodes[ib].child1 = i_c;

                    nodes[ic].parent = i_b;
                    nodes[id].parent = i_a;

                    nodes[ib].aabb = aabb_ce;
                    nodes[ib].height = 1 + max_u16(nodes[ic].height, nodes[ie].height);
                    nodes[ia].height = 1 + max_u16(nodes[ib].height, nodes[id].height);
                    nodes[ib].category_bits = nodes[ic].category_bits | nodes[ie].category_bits;
                    nodes[ia].category_bits = nodes[ib].category_bits | nodes[id].category_bits;
                    nodes[ib].flags |= (nodes[ic].flags | nodes[ie].flags) & ENLARGED_NODE;
                    nodes[ia].flags |= (nodes[ib].flags | nodes[id].flags) & ENLARGED_NODE;
                }

                RotateType::Ce => {
                    nodes[ia].child2 = i_e;
                    nodes[ib].child2 = i_c;

                    nodes[ic].parent = i_b;
                    nodes[ie].parent = i_a;

                    nodes[ib].aabb = aabb_cd;
                    nodes[ib].height = 1 + max_u16(nodes[ic].height, nodes[id].height);
                    nodes[ia].height = 1 + max_u16(nodes[ib].height, nodes[ie].height);
                    nodes[ib].category_bits = nodes[ic].category_bits | nodes[id].category_bits;
                    nodes[ia].category_bits = nodes[ib].category_bits | nodes[ie].category_bits;
                    nodes[ib].flags |= (nodes[ic].flags | nodes[id].flags) & ENLARGED_NODE;
                    nodes[ia].flags |= (nodes[ib].flags | nodes[ie].flags) & ENLARGED_NODE;
                }
            }
        }
    }

    /// (static b3InsertLeaf)
    fn insert_leaf(&mut self, leaf: i32, should_rotate: bool) {
        if self.root == NULL_INDEX {
            self.root = leaf;
            self.nodes[self.root as usize].parent = NULL_INDEX;
            return;
        }

        // Stage 1: find the best sibling for this node
        let leaf_aabb = self.nodes[leaf as usize].aabb;
        let sibling = find_best_sibling(self, leaf_aabb);

        // Stage 2: create a new parent for the leaf and sibling
        let old_parent = self.nodes[sibling as usize].parent;
        let new_parent = self.allocate_node();

        // warning: node pointer can change after allocation
        let nodes = &mut self.nodes;
        let np = new_parent as usize;
        nodes[np].parent = old_parent;
        nodes[np].user_data = u64::MAX;
        nodes[np].aabb = aabb_union(leaf_aabb, nodes[sibling as usize].aabb);
        nodes[np].category_bits =
            nodes[leaf as usize].category_bits | nodes[sibling as usize].category_bits;
        nodes[np].height = nodes[sibling as usize].height + 1;

        if old_parent != NULL_INDEX {
            // The sibling was not the root.
            if nodes[old_parent as usize].child1 == sibling {
                nodes[old_parent as usize].child1 = new_parent;
            } else {
                nodes[old_parent as usize].child2 = new_parent;
            }

            nodes[np].child1 = sibling;
            nodes[np].child2 = leaf;
            nodes[sibling as usize].parent = new_parent;
            nodes[leaf as usize].parent = new_parent;
        } else {
            // The sibling was the root.
            nodes[np].child1 = sibling;
            nodes[np].child2 = leaf;
            nodes[sibling as usize].parent = new_parent;
            nodes[leaf as usize].parent = new_parent;
            self.root = new_parent;
        }

        // Stage 3: walk back up the tree fixing heights and AABBs
        let mut index = self.nodes[leaf as usize].parent;
        while index != NULL_INDEX {
            let child1 = self.nodes[index as usize].child1;
            let child2 = self.nodes[index as usize].child2;

            debug_assert!(child1 != NULL_INDEX);
            debug_assert!(child2 != NULL_INDEX);

            let (c1, c2) = (child1 as usize, child2 as usize);
            self.nodes[index as usize].aabb = aabb_union(self.nodes[c1].aabb, self.nodes[c2].aabb);
            self.nodes[index as usize].category_bits =
                self.nodes[c1].category_bits | self.nodes[c2].category_bits;
            self.nodes[index as usize].height =
                1 + max_u16(self.nodes[c1].height, self.nodes[c2].height);
            self.nodes[index as usize].flags |=
                (self.nodes[c1].flags | self.nodes[c2].flags) & ENLARGED_NODE;

            if should_rotate {
                self.rotate_nodes(index);
            }

            index = self.nodes[index as usize].parent;
        }
    }

    /// (static b3RemoveLeaf)
    pub(crate) fn remove_leaf(&mut self, leaf: i32) {
        if leaf == self.root {
            self.root = NULL_INDEX;
            return;
        }

        let parent = self.nodes[leaf as usize].parent;
        let grand_parent = self.nodes[parent as usize].parent;
        let sibling = if self.nodes[parent as usize].child1 == leaf {
            self.nodes[parent as usize].child2
        } else {
            self.nodes[parent as usize].child1
        };

        if grand_parent != NULL_INDEX {
            // Destroy parent and connect sibling to grandParent.
            if self.nodes[grand_parent as usize].child1 == parent {
                self.nodes[grand_parent as usize].child1 = sibling;
            } else {
                self.nodes[grand_parent as usize].child2 = sibling;
            }
            self.nodes[sibling as usize].parent = grand_parent;
            self.free_node(parent);

            // Adjust ancestor bounds.
            let mut index = grand_parent;
            while index != NULL_INDEX {
                let child1 = self.nodes[index as usize].child1 as usize;
                let child2 = self.nodes[index as usize].child2 as usize;

                self.nodes[index as usize].aabb =
                    aabb_union(self.nodes[child1].aabb, self.nodes[child2].aabb);
                self.nodes[index as usize].category_bits =
                    self.nodes[child1].category_bits | self.nodes[child2].category_bits;
                self.nodes[index as usize].height =
                    1 + max_u16(self.nodes[child1].height, self.nodes[child2].height);

                index = self.nodes[index as usize].parent;
            }
        } else {
            self.root = sibling;
            self.nodes[sibling as usize].parent = NULL_INDEX;
            self.free_node(parent);
        }
    }

    /// Create a proxy in the tree as a leaf node. Returns the node index.
    /// (b3DynamicTree_CreateProxy)
    pub fn create_proxy(&mut self, aabb: Aabb, category_bits: u64, user_data: u64) -> i32 {
        debug_assert!(is_valid_aabb(aabb));

        let proxy_id = self.allocate_node();
        let node = &mut self.nodes[proxy_id as usize];

        node.aabb = aabb;
        node.user_data = user_data;
        node.category_bits = category_bits;
        node.height = 0;
        node.flags = ALLOCATED_NODE | LEAF_NODE;

        let should_rotate = true;
        self.insert_leaf(proxy_id, should_rotate);

        self.proxy_count += 1;

        proxy_id
    }

    /// Destroy a proxy. This asserts if the id is invalid.
    /// (b3DynamicTree_DestroyProxy)
    pub fn destroy_proxy(&mut self, proxy_id: i32) {
        debug_assert!(0 <= proxy_id && proxy_id < self.node_capacity());
        debug_assert!(self.nodes[proxy_id as usize].is_leaf());

        self.remove_leaf(proxy_id);
        self.free_node(proxy_id);

        debug_assert!(self.proxy_count > 0);
        self.proxy_count -= 1;
    }

    /// Move a proxy to a new AABB by removing and reinserting into the tree.
    /// (b3DynamicTree_MoveProxy)
    pub fn move_proxy(&mut self, proxy_id: i32, aabb: Aabb) {
        debug_assert!(is_valid_aabb(aabb));
        debug_assert!(0 <= proxy_id && proxy_id < self.node_capacity());
        debug_assert!(self.nodes[proxy_id as usize].is_leaf());

        self.remove_leaf(proxy_id);

        self.nodes[proxy_id as usize].aabb = aabb;

        let should_rotate = false;
        self.insert_leaf(proxy_id, should_rotate);
    }

    /// Enlarge a proxy and enlarge ancestors as necessary.
    /// (b3DynamicTree_EnlargeProxy)
    pub fn enlarge_proxy(&mut self, proxy_id: i32, aabb: Aabb) {
        debug_assert!(is_valid_aabb(aabb));
        debug_assert!(0 <= proxy_id && proxy_id < self.node_capacity());
        debug_assert!(self.nodes[proxy_id as usize].is_leaf());

        // Caller must ensure this
        debug_assert!(!aabb_contains(self.nodes[proxy_id as usize].aabb, aabb));

        self.nodes[proxy_id as usize].aabb = aabb;

        let mut parent_index = self.nodes[proxy_id as usize].parent;
        while parent_index != NULL_INDEX {
            let changed = enlarge_aabb(&mut self.nodes[parent_index as usize].aabb, aabb);
            self.nodes[parent_index as usize].flags |= ENLARGED_NODE;
            parent_index = self.nodes[parent_index as usize].parent;

            if !changed {
                break;
            }
        }

        while parent_index != NULL_INDEX {
            if self.nodes[parent_index as usize].flags & ENLARGED_NODE != 0 {
                // early out because this ancestor was previously ascended and
                // marked as enlarged
                break;
            }

            self.nodes[parent_index as usize].flags |= ENLARGED_NODE;
            parent_index = self.nodes[parent_index as usize].parent;
        }
    }

    /// Modify the category bits on a proxy. This is an expensive operation.
    /// (b3DynamicTree_SetCategoryBits)
    pub fn set_category_bits(&mut self, proxy_id: i32, category_bits: u64) {
        debug_assert!(self.nodes[proxy_id as usize].is_leaf());

        self.nodes[proxy_id as usize].category_bits = category_bits;

        // Fix up category bits in ancestor internal nodes
        let mut node_index = self.nodes[proxy_id as usize].parent;
        while node_index != NULL_INDEX {
            let child1 = self.nodes[node_index as usize].child1;
            debug_assert!(child1 != NULL_INDEX);
            let child2 = self.nodes[node_index as usize].child2;
            debug_assert!(child2 != NULL_INDEX);
            self.nodes[node_index as usize].category_bits = self.nodes[child1 as usize]
                .category_bits
                | self.nodes[child2 as usize].category_bits;

            node_index = self.nodes[node_index as usize].parent;
        }
    }
}
