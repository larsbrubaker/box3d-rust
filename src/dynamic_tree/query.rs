// AABB query, closest query, ray cast, and box cast from dynamic_tree.c.
//
// The C callbacks take a `void* context`; the Rust versions take closures,
// which capture their context directly.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::{category_bits_match, BoxCastInput, DynamicTree, TreeStats, TREE_STACK_SIZE};
use crate::core::NULL_INDEX;
use crate::geometry::RayCastInput;
use crate::math_functions::{
    aabb_center, aabb_extents, aabb_overlaps, add, clamp, distance_squared, dot, max, min, mul_add,
    mul_sv, sub, test_bounds_ray_overlap, Aabb, Vec3,
};

/// Squared distance from a point to a node AABB. (static b3DistanceToNodeSqr)
fn distance_to_node_sqr(point: Vec3, node_aabb: Aabb) -> f32 {
    let r = sub(point, clamp(point, node_aabb.lower_bound, node_aabb.upper_bound));
    dot(r, r)
}

#[derive(Clone, Copy)]
struct QueryClosestItem {
    node_index: i32,
    distance_to_node_sqr: f32,
}

impl DynamicTree {
    /// Query an AABB for overlapping proxies. The callback is called for each
    /// proxy that overlaps the supplied AABB and passes the mask-bits filter;
    /// return false from the callback to stop. (b3DynamicTree_Query)
    pub fn query(
        &self,
        aabb: Aabb,
        mask_bits: u64,
        require_all_bits: bool,
        mut callback: impl FnMut(i32, u64) -> bool,
    ) -> TreeStats {
        let mut result = TreeStats::default();

        if self.node_count == 0 {
            return result;
        }

        let mut stack = [0i32; TREE_STACK_SIZE];
        let mut stack_count = 0usize;
        stack[stack_count] = self.root;
        stack_count += 1;

        while stack_count > 0 {
            stack_count -= 1;
            let node_id = stack[stack_count];
            if node_id == NULL_INDEX {
                debug_assert!(false);
                continue;
            }

            let node = &self.nodes[node_id as usize];
            result.node_visits += 1;

            if category_bits_match(node.category_bits, mask_bits, require_all_bits)
                && aabb_overlaps(node.aabb, aabb)
            {
                if node.is_leaf() {
                    // callback to user code with proxy id
                    let proceed = callback(node_id, node.user_data);
                    result.leaf_visits += 1;

                    if !proceed {
                        return result;
                    }
                } else {
                    debug_assert!(stack_count < TREE_STACK_SIZE - 1);
                    if stack_count < TREE_STACK_SIZE - 1 {
                        stack[stack_count] = node.child1;
                        stack_count += 1;
                        stack[stack_count] = node.child2;
                        stack_count += 1;
                    }
                }
            }
        }

        result
    }

    /// Query for the closest proxy to a point. The callback receives the current
    /// minimum squared distance and returns an updated distance for that proxy.
    /// (b3DynamicTree_QueryClosest)
    pub fn query_closest(
        &self,
        point: Vec3,
        mask_bits: u64,
        require_all_bits: bool,
        mut callback: impl FnMut(f32, i32, u64) -> f32,
        min_distance_sqr: &mut f32,
    ) -> TreeStats {
        let mut result = TreeStats::default();

        if self.node_count == 0 {
            return result;
        }

        let mut min_sqr = *min_distance_sqr;
        let mut stack = [QueryClosestItem {
            node_index: 0,
            distance_to_node_sqr: 0.0,
        }; TREE_STACK_SIZE];
        let mut stack_count = 0usize;

        let root_distance_sqr = distance_to_node_sqr(point, self.nodes[self.root as usize].aabb);
        stack[stack_count] = QueryClosestItem {
            node_index: self.root,
            distance_to_node_sqr: root_distance_sqr,
        };
        stack_count += 1;

        while stack_count > 0 {
            stack_count -= 1;
            let item = stack[stack_count];
            let node = &self.nodes[item.node_index as usize];
            result.node_visits += 1;

            if category_bits_match(node.category_bits, mask_bits, require_all_bits)
                && item.distance_to_node_sqr < min_sqr
            {
                if node.is_leaf() {
                    let dd = callback(min_sqr, item.node_index, node.user_data);

                    if dd < min_sqr {
                        min_sqr = dd;
                    }

                    result.leaf_visits += 1;
                } else {
                    debug_assert!(stack_count < TREE_STACK_SIZE - 1);
                    if stack_count < TREE_STACK_SIZE - 1 {
                        let child1 = node.child1;
                        let child2 = node.child2;

                        let item1 = QueryClosestItem {
                            node_index: child1,
                            distance_to_node_sqr: distance_to_node_sqr(
                                point,
                                self.nodes[child1 as usize].aabb,
                            ),
                        };

                        let item2 = QueryClosestItem {
                            node_index: child2,
                            distance_to_node_sqr: distance_to_node_sqr(
                                point,
                                self.nodes[child2 as usize].aabb,
                            ),
                        };

                        // Ensure we iterate the closest child first as we pop
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
                }
            }
        }

        *min_distance_sqr = min_sqr;

        result
    }

    /// Ray cast against the proxies in the tree. The callback performs an
    /// exact ray cast when the proxy contains a shape, and returns the new
    /// ray fraction:
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
        mut callback: impl FnMut(&RayCastInput, i32, u64) -> f32,
    ) -> TreeStats {
        let mut result = TreeStats::default();

        if self.node_count == 0 {
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

        let mut stack = [0i32; TREE_STACK_SIZE];
        let mut stack_count = 0usize;
        stack[stack_count] = self.root;
        stack_count += 1;

        let mut sub_input = *input;

        while stack_count > 0 {
            stack_count -= 1;
            let node_id = stack[stack_count];
            if node_id == NULL_INDEX {
                debug_assert!(false);
                continue;
            }

            let node = &self.nodes[node_id as usize];
            result.node_visits += 1;

            let node_aabb = node.aabb;

            if !category_bits_match(node.category_bits, mask_bits, require_all_bits)
                || !aabb_overlaps(node_aabb, segment_aabb)
            {
                continue;
            }

            if !test_bounds_ray_overlap(
                node_aabb.lower_bound,
                node_aabb.upper_bound,
                p1,
                d,
            ) {
                continue;
            }

            if node.is_leaf() {
                sub_input.max_fraction = max_fraction;

                let value = callback(&sub_input, node_id, node.user_data);
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
                }
            } else {
                debug_assert!(stack_count < TREE_STACK_SIZE - 1);
                if stack_count < TREE_STACK_SIZE - 1 {
                    let c1 = aabb_center(self.nodes[node.child1 as usize].aabb);
                    let c2 = aabb_center(self.nodes[node.child2 as usize].aabb);
                    if distance_squared(c1, p1) < distance_squared(c2, p1) {
                        stack[stack_count] = node.child2;
                        stack_count += 1;
                        stack[stack_count] = node.child1;
                        stack_count += 1;
                    } else {
                        stack[stack_count] = node.child1;
                        stack_count += 1;
                        stack[stack_count] = node.child2;
                        stack_count += 1;
                    }
                }
            }
        }

        result
    }

    /// Cast a swept AABB through the tree. The callback returns the new cast
    /// fraction, with the same semantics as [`DynamicTree::ray_cast`].
    /// (b3DynamicTree_BoxCast)
    pub fn box_cast(
        &self,
        input: &BoxCastInput,
        mask_bits: u64,
        require_all_bits: bool,
        mut callback: impl FnMut(&BoxCastInput, i32, u64) -> f32,
    ) -> TreeStats {
        let mut stats = TreeStats::default();

        if self.node_count == 0 {
            return stats;
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

        let mut sub_input = *input;

        let mut stack = [0i32; TREE_STACK_SIZE];
        let mut stack_count = 0usize;
        stack[stack_count] = self.root;
        stack_count += 1;

        while stack_count > 0 {
            stack_count -= 1;
            let node_id = stack[stack_count];
            if node_id == NULL_INDEX {
                debug_assert!(false);
                continue;
            }

            let node = &self.nodes[node_id as usize];
            stats.node_visits += 1;

            if !category_bits_match(node.category_bits, mask_bits, require_all_bits)
                || !aabb_overlaps(node.aabb, total_aabb)
            {
                continue;
            }

            // radius extension is added to the node in this case
            let lower = sub(node.aabb.lower_bound, extension);
            let upper = add(node.aabb.upper_bound, extension);
            if !test_bounds_ray_overlap(lower, upper, p1, d) {
                continue;
            }

            if node.is_leaf() {
                sub_input.max_fraction = max_fraction;

                let value = callback(&sub_input, node_id, node.user_data);
                stats.leaf_visits += 1;

                if value == 0.0 {
                    // The client has terminated the cast.
                    return stats;
                }

                if 0.0 < value && value < max_fraction {
                    max_fraction = value;
                    t = mul_sv(max_fraction, input.translation);
                    total_aabb.lower_bound =
                        min(origin_aabb.lower_bound, add(origin_aabb.lower_bound, t));
                    total_aabb.upper_bound =
                        max(origin_aabb.upper_bound, add(origin_aabb.upper_bound, t));
                }
            } else {
                debug_assert!(stack_count < TREE_STACK_SIZE - 1);
                if stack_count < TREE_STACK_SIZE - 1 {
                    let c1 = aabb_center(self.nodes[node.child1 as usize].aabb);
                    let c2 = aabb_center(self.nodes[node.child2 as usize].aabb);
                    if distance_squared(c1, p1) < distance_squared(c2, p1) {
                        stack[stack_count] = node.child2;
                        stack_count += 1;
                        stack[stack_count] = node.child1;
                        stack_count += 1;
                    } else {
                        stack[stack_count] = node.child1;
                        stack_count += 1;
                        stack[stack_count] = node.child2;
                        stack_count += 1;
                    }
                }
            }
        }

        stats
    }
}
