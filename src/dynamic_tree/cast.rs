// Ray cast and box cast from dynamic_tree.c.
//
// The C callbacks take a `void* context`; the Rust versions take closures, which capture
// their context directly.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::aabbv::overlap_node;
use super::query::leaf_user_data;
use super::{
    get_root_pair, make_tree_view, BoxCastInput, DynamicTree, TreeStats, TreeView, ROOT_NODE,
    TREE_STACK_SIZE,
};
use crate::geometry::RayCastInput;
use crate::math_functions::{
    aabb_center, aabb_extents, add, distance_squared, max, min, mul_add, mul_sv, sub,
    test_bounds_ray_overlap, Aabb,
};

/// Ray cast over a tree view. (b3RayCastTreeView)
///
/// A lot of optimization work went into this. The children of a popped pair are both
/// tested before either is followed, then the survivors are ordered so the closer one
/// narrows first.
pub(super) fn ray_cast_tree_view(
    view: &TreeView<'_>,
    input: &RayCastInput,
    mask_bits: u64,
    require_all_bits: bool,
    mut callback: impl FnMut(&RayCastInput, i32, u64) -> f32,
) -> TreeStats {
    let mut result = TreeStats::default();

    if view.nodes[ROOT_NODE as usize].is_empty() {
        return result;
    }

    let p1 = input.origin;
    let d = input.translation;

    let mut max_fraction = input.max_fraction;

    let mut p2 = mul_add(p1, max_fraction, d);

    // Build a bounding box for the segment.
    let mut segment_aabb = Aabb {
        lower_bound: min(p1, p2),
        upper_bound: max(p1, p2),
    };
    let mut boxv = segment_aabb;

    let nodes = view.nodes;

    let mut stack = [0i32; TREE_STACK_SIZE];
    let mut stack_count = 0usize;
    stack[stack_count] = get_root_pair(nodes);
    stack_count += 1;

    let mut sub_input = *input;

    while stack_count > 0 {
        stack_count -= 1;
        let pair = stack[stack_count];
        result.node_visits += 1;

        let mut hit = [0usize; 2];
        let mut is_leaf = [false; 2];
        let mut hit_count = 0usize;
        for i in 0..2 {
            let node_index = pair as usize + i;
            let node = &nodes[node_index];

            if !overlap_node(&boxv, node) {
                continue;
            }

            let node_aabb = node.aabb;
            if !test_bounds_ray_overlap(node_aabb.lower_bound, node_aabb.upper_bound, p1, d) {
                continue;
            }

            is_leaf[hit_count] = node.is_leaf();
            hit[hit_count] = node_index;
            hit_count += 1;
        }

        if hit_count == 2 && !is_leaf[0] && !is_leaf[1] {
            let center1 = aabb_center(nodes[hit[0]].aabb);
            let center2 = aabb_center(nodes[hit[1]].aabb);
            let d1 = distance_squared(center1, p1);
            let d2 = distance_squared(center2, p1);

            // Want to push the closest one last. Both have the same isLeaf, so they
            // don't swap.
            if d1 < d2 {
                hit.swap(0, 1);
            }
        }

        for i in 0..hit_count {
            let node = &nodes[hit[i]];
            if is_leaf[i] {
                let proxy_id = node.proxy_id();
                let Some(user_data) =
                    leaf_user_data(view, node, proxy_id, mask_bits, require_all_bits)
                else {
                    continue;
                };

                sub_input.max_fraction = max_fraction;

                let value = callback(&sub_input, proxy_id, user_data);
                result.leaf_visits += 1;

                // The user may return -1 to indicate this shape should be skipped

                if value == 0.0 {
                    // The client has terminated the ray cast.
                    return result;
                }

                if 0.0 < value && value <= max_fraction {
                    // Update segment bounding box.
                    max_fraction = value;
                    p2 = mul_add(p1, max_fraction, d);
                    segment_aabb.lower_bound = min(p1, p2);
                    segment_aabb.upper_bound = max(p1, p2);
                    boxv = segment_aabb;
                }
            } else if stack_count < TREE_STACK_SIZE - 1 {
                stack[stack_count] = node.left_child();
                stack_count += 1;
            } else {
                debug_assert!(stack_count < TREE_STACK_SIZE - 1);
            }
        }
    }

    result
}

/// Box cast over a tree view. (b3BoxCastTreeView)
///
/// Follows structure of ray cast with small tweaks to handle a swept box.
pub(super) fn box_cast_tree_view(
    view: &TreeView<'_>,
    input: &BoxCastInput,
    mask_bits: u64,
    require_all_bits: bool,
    mut callback: impl FnMut(&BoxCastInput, i32, u64) -> f32,
) -> TreeStats {
    let mut result = TreeStats::default();

    if view.nodes[ROOT_NODE as usize].is_empty() {
        return result;
    }

    // The caller folds the shape radius and the world origin into the box
    let origin_aabb = input.box_;

    let p1 = aabb_center(origin_aabb);
    let extension = aabb_extents(origin_aabb);

    let d = input.translation;

    let mut max_fraction = input.max_fraction;

    // Build total box for the cast
    let mut t = mul_sv(max_fraction, input.translation);
    let mut total_aabb = Aabb {
        lower_bound: min(origin_aabb.lower_bound, add(origin_aabb.lower_bound, t)),
        upper_bound: max(origin_aabb.upper_bound, add(origin_aabb.upper_bound, t)),
    };
    let mut boxv = total_aabb;

    let nodes = view.nodes;

    let mut stack = [0i32; TREE_STACK_SIZE];
    let mut stack_count = 0usize;
    stack[stack_count] = get_root_pair(nodes);
    stack_count += 1;

    let mut sub_input = *input;

    while stack_count > 0 {
        stack_count -= 1;
        let pair = stack[stack_count];
        result.node_visits += 1;

        let mut hit = [0usize; 2];
        let mut is_leaf = [false; 2];
        let mut hit_count = 0usize;
        for i in 0..2 {
            let node_index = pair as usize + i;
            let node = &nodes[node_index];

            if !overlap_node(&boxv, node) {
                continue;
            }

            // radius extension is added to the node in this case
            let node_aabb = node.aabb;
            let lower = sub(node_aabb.lower_bound, extension);
            let upper = add(node_aabb.upper_bound, extension);
            if !test_bounds_ray_overlap(lower, upper, p1, d) {
                continue;
            }

            is_leaf[hit_count] = node.is_leaf();
            hit[hit_count] = node_index;
            hit_count += 1;
        }

        if hit_count == 2 && !is_leaf[0] && !is_leaf[1] {
            let center1 = aabb_center(nodes[hit[0]].aabb);
            let center2 = aabb_center(nodes[hit[1]].aabb);
            let d1 = distance_squared(center1, p1);
            let d2 = distance_squared(center2, p1);

            // Want to push the closest one last. Both have the same isLeaf, so they
            // don't swap.
            if d1 < d2 {
                hit.swap(0, 1);
            }
        }

        for i in 0..hit_count {
            let node = &nodes[hit[i]];
            if is_leaf[i] {
                let proxy_id = node.proxy_id();
                let Some(user_data) =
                    leaf_user_data(view, node, proxy_id, mask_bits, require_all_bits)
                else {
                    continue;
                };

                sub_input.max_fraction = max_fraction;

                let value = callback(&sub_input, proxy_id, user_data);
                result.leaf_visits += 1;

                if value == 0.0 {
                    // The client has terminated the cast.
                    return result;
                }

                if 0.0 < value && value < max_fraction {
                    max_fraction = value;
                    t = mul_sv(max_fraction, input.translation);
                    total_aabb.lower_bound =
                        min(origin_aabb.lower_bound, add(origin_aabb.lower_bound, t));
                    total_aabb.upper_bound =
                        max(origin_aabb.upper_bound, add(origin_aabb.upper_bound, t));
                    boxv = total_aabb;
                }
            } else if stack_count < TREE_STACK_SIZE - 1 {
                stack[stack_count] = node.left_child();
                stack_count += 1;
            } else {
                debug_assert!(stack_count < TREE_STACK_SIZE - 1);
            }
        }
    }

    result
}

impl DynamicTree {
    /// Ray cast against the proxies in the tree. The callback performs an exact ray cast
    /// when the proxy contains a shape, and returns the new ray fraction:
    /// - return 0 to terminate the ray cast
    /// - return a value less than the input max_fraction to clip the ray
    /// - return the input max_fraction to continue without clipping
    ///
    /// (b3DynamicTree_RayCast)
    pub fn ray_cast(
        &self,
        input: &RayCastInput,
        mask_bits: u64,
        require_all_bits: bool,
        callback: impl FnMut(&RayCastInput, i32, u64) -> f32,
    ) -> TreeStats {
        if self.proxy_count == 0 {
            return TreeStats::default();
        }

        let view = make_tree_view(self);
        ray_cast_tree_view(&view, input, mask_bits, require_all_bits, callback)
    }

    /// Cast a swept AABB through the tree. The callback returns the new cast fraction, with
    /// the same semantics as [`DynamicTree::ray_cast`]. (b3DynamicTree_BoxCast)
    pub fn box_cast(
        &self,
        input: &BoxCastInput,
        mask_bits: u64,
        require_all_bits: bool,
        callback: impl FnMut(&BoxCastInput, i32, u64) -> f32,
    ) -> TreeStats {
        if self.proxy_count == 0 {
            return TreeStats::default();
        }

        let view = make_tree_view(self);
        box_cast_tree_view(&view, input, mask_bits, require_all_bits, callback)
    }
}
