// AABB query and closest query from dynamic_tree.c.
//
// The C callbacks take a `void* context`; the Rust versions take closures, which capture
// their context directly.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::aabbv::overlap_node;
use super::{
    get_root_pair, make_tree_view, test_category, DynamicTree, TreeNode, TreeStats, TreeView,
    ROOT_NODE, TREE_STACK_SIZE,
};
use crate::math_functions::{clamp, dot, sub, Aabb, Vec3};

/// Squared distance from a point to a node AABB. (b3DistanceToNodeSqr)
#[inline]
fn distance_to_node_sqr(point: Vec3, node: &TreeNode) -> f32 {
    let r = sub(
        point,
        clamp(point, node.aabb.lower_bound, node.aabb.upper_bound),
    );
    dot(r, r)
}

/// (struct b3QueryClosestItem)
#[derive(Clone, Copy)]
struct QueryClosestItem {
    node_index: i32,
    distance_to_node_sqr: f32,
}

/// The user data of a leaf, or `None` if its proxy fails the category filter.
///
/// C reads `(uint64_t)(uint32_t)node->shapeIndex`, then, when the view has proxies, tests the
/// proxy category bits and takes the proxy user data instead. A view without proxies (a baked
/// compound tree) has no category filter. C passes a `hasProxies` flag that is always
/// `view->proxies != NULL` so the two cases inline separately; the port reads it from the
/// optional proxy slice.
#[inline]
pub(super) fn leaf_user_data(
    view: &TreeView<'_>,
    node: &TreeNode,
    proxy_id: i32,
    mask_bits: u64,
    require_all_bits: bool,
) -> Option<u64> {
    let mut user_data = node.shape_index() as u32 as u64;
    if let Some(proxies) = view.proxies {
        let proxy = &proxies[proxy_id as usize];
        if !test_category(proxy.category_bits, mask_bits, require_all_bits) {
            return None;
        }
        user_data = proxy.user_data;
    }
    Some(user_data)
}

/// AABB query over a tree view. (b3QueryTreeView)
pub(super) fn query_tree_view(
    view: &TreeView<'_>,
    aabb: Aabb,
    mask_bits: u64,
    require_all_bits: bool,
    mut callback: impl FnMut(i32, u64) -> bool,
) -> TreeStats {
    let mut result = TreeStats::default();

    if view.nodes[ROOT_NODE as usize].is_empty() {
        return result;
    }

    let nodes = view.nodes;

    let mut stack = [0i32; TREE_STACK_SIZE];
    let mut stack_count = 0usize;
    stack[stack_count] = get_root_pair(nodes);
    stack_count += 1;

    let boxv = aabb;

    while stack_count > 0 {
        stack_count -= 1;
        let pair = stack[stack_count];
        result.node_visits += 1;

        for i in 0..2 {
            let node = &nodes[pair as usize + i];
            if !overlap_node(&boxv, node) {
                continue;
            }

            if node.is_leaf() {
                let proxy_id = node.proxy_id();
                let Some(user_data) =
                    leaf_user_data(view, node, proxy_id, mask_bits, require_all_bits)
                else {
                    continue;
                };

                let proceed = callback(proxy_id, user_data);
                result.leaf_visits += 1;

                if !proceed {
                    return result;
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

/// Closest query over a tree view. (b3QueryClosestTreeView)
pub(super) fn query_closest_tree_view(
    view: &TreeView<'_>,
    point: Vec3,
    mask_bits: u64,
    require_all_bits: bool,
    mut callback: impl FnMut(f32, i32, u64) -> f32,
    min_distance_sqr: &mut f32,
) -> TreeStats {
    let mut result = TreeStats::default();

    if view.nodes[ROOT_NODE as usize].is_empty() {
        return result;
    }

    let nodes = view.nodes;

    let mut min_sqr = *min_distance_sqr;
    let mut stack = [QueryClosestItem {
        node_index: 0,
        distance_to_node_sqr: 0.0,
    }; TREE_STACK_SIZE];
    let mut stack_count = 0usize;

    let root_pair = get_root_pair(nodes);
    let seed1 = QueryClosestItem {
        node_index: root_pair,
        distance_to_node_sqr: distance_to_node_sqr(point, &nodes[root_pair as usize]),
    };

    if root_pair == ROOT_NODE {
        stack[stack_count] = seed1;
        stack_count += 1;
    } else {
        let seed2 = QueryClosestItem {
            node_index: root_pair + 1,
            distance_to_node_sqr: distance_to_node_sqr(point, &nodes[root_pair as usize + 1]),
        };

        // Ensure we iterate the closest child first as we pop off the stack
        if seed2.distance_to_node_sqr < seed1.distance_to_node_sqr {
            stack[stack_count] = seed1;
            stack_count += 1;
            stack[stack_count] = seed2;
            stack_count += 1;
        } else {
            stack[stack_count] = seed2;
            stack_count += 1;
            stack[stack_count] = seed1;
            stack_count += 1;
        }
    }

    while stack_count > 0 {
        stack_count -= 1;
        let item = stack[stack_count];
        let node = &nodes[item.node_index as usize];
        result.node_visits += 1;

        if item.distance_to_node_sqr >= min_sqr {
            continue;
        }

        if node.is_leaf() {
            let proxy_id = node.proxy_id();
            let Some(user_data) = leaf_user_data(view, node, proxy_id, mask_bits, require_all_bits)
            else {
                continue;
            };

            // callback to user code with minimum distance squared so far and proxy id
            let dd = callback(min_sqr, proxy_id, user_data);

            if dd < min_sqr {
                min_sqr = dd;
            }

            result.leaf_visits += 1;
            continue;
        }

        if stack_count + 2 > TREE_STACK_SIZE {
            debug_assert!(stack_count + 2 <= TREE_STACK_SIZE);
            continue;
        }

        let pair = node.left_child();

        // Store the distance to node in the stack instead of recomputing after pop
        let item1 = QueryClosestItem {
            node_index: pair,
            distance_to_node_sqr: distance_to_node_sqr(point, &nodes[pair as usize]),
        };

        let item2 = QueryClosestItem {
            node_index: pair + 1,
            distance_to_node_sqr: distance_to_node_sqr(point, &nodes[pair as usize + 1]),
        };

        // Ensure we iterate the closest child first as we pop off the stack
        if item2.distance_to_node_sqr < item1.distance_to_node_sqr {
            stack[stack_count] = item1;
            stack_count += 1;
            stack[stack_count] = item2;
            stack_count += 1;
        } else {
            stack[stack_count] = item2;
            stack_count += 1;
            stack[stack_count] = item1;
            stack_count += 1;
        }
    }

    *min_distance_sqr = min_sqr;

    result
}

impl DynamicTree {
    /// Query an AABB for overlapping proxies. The callback is called for each proxy that
    /// overlaps the supplied AABB and passes the mask-bits filter; return false from the
    /// callback to stop. (b3DynamicTree_Query)
    pub fn query(
        &self,
        aabb: Aabb,
        mask_bits: u64,
        require_all_bits: bool,
        callback: impl FnMut(i32, u64) -> bool,
    ) -> TreeStats {
        if self.proxy_count == 0 {
            return TreeStats::default();
        }

        let view = make_tree_view(self);
        query_tree_view(&view, aabb, mask_bits, require_all_bits, callback)
    }

    /// Query for the closest proxy to a point. The callback receives the current minimum
    /// squared distance and returns an updated distance for that proxy. `min_distance_sqr`
    /// is the initial and final minimum squared distance.
    /// (b3DynamicTree_QueryClosest)
    pub fn query_closest(
        &self,
        point: Vec3,
        mask_bits: u64,
        require_all_bits: bool,
        callback: impl FnMut(f32, i32, u64) -> f32,
        min_distance_sqr: &mut f32,
    ) -> TreeStats {
        if self.proxy_count == 0 {
            return TreeStats::default();
        }

        let view = make_tree_view(self);
        query_closest_tree_view(
            &view,
            point,
            mask_bits,
            require_all_bits,
            callback,
            min_distance_sqr,
        )
    }
}
