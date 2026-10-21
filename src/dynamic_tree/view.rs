// Read-only tree views from dynamic_tree.h / dynamic_tree.c (upstream 5d83df8).
//
// A view is a node array plus an optional proxy array. A live dynamic tree has both. A baked
// compound stores only its nodes, so its view has no proxies: the leaves carry the child index
// in `shapeIndex` and there is no category filtering.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::cast::{box_cast_tree_view, ray_cast_tree_view};
use super::query::{query_closest_tree_view, query_tree_view};
use super::{BoxCastInput, DynamicTree, TreeNode, TreeProxy, TreeStats};
use crate::geometry::RayCastInput;
use crate::math_functions::{Aabb, Vec3};

/// A read-only view of tree nodes and, optionally, proxies. (b3TreeView)
///
/// C uses a null `proxies` pointer for a view without proxies; the port uses `None`.
#[derive(Debug, Clone, Copy)]
pub struct TreeView<'a> {
    pub nodes: &'a [TreeNode],
    pub proxies: Option<&'a [TreeProxy]>,
}

/// View a dynamic tree. (b3MakeTreeView)
#[inline]
pub fn make_tree_view(tree: &DynamicTree) -> TreeView<'_> {
    TreeView {
        nodes: &tree.nodes,
        proxies: Some(&tree.proxies),
    }
}

impl TreeView<'_> {
    /// Query an AABB for overlapping leaves. Same semantics as [`DynamicTree::query`]; a view
    /// without proxies skips the category filter and reports the leaf shape index as the
    /// user data. (b3TreeView_Query)
    pub fn query(
        &self,
        aabb: Aabb,
        mask_bits: u64,
        require_all_bits: bool,
        callback: impl FnMut(i32, u64) -> bool,
    ) -> TreeStats {
        query_tree_view(self, aabb, mask_bits, require_all_bits, callback)
    }

    /// Query for the closest leaf to a point. Same semantics as
    /// [`DynamicTree::query_closest`]. (b3TreeView_QueryClosest)
    pub fn query_closest(
        &self,
        point: Vec3,
        mask_bits: u64,
        require_all_bits: bool,
        callback: impl FnMut(f32, i32, u64) -> f32,
        min_distance_sqr: &mut f32,
    ) -> TreeStats {
        query_closest_tree_view(
            self,
            point,
            mask_bits,
            require_all_bits,
            callback,
            min_distance_sqr,
        )
    }

    /// Ray cast against the leaves. Same semantics as [`DynamicTree::ray_cast`].
    /// (b3TreeView_RayCast)
    pub fn ray_cast(
        &self,
        input: &RayCastInput,
        mask_bits: u64,
        require_all_bits: bool,
        callback: impl FnMut(&RayCastInput, i32, u64) -> f32,
    ) -> TreeStats {
        ray_cast_tree_view(self, input, mask_bits, require_all_bits, callback)
    }

    /// Cast a swept AABB against the leaves. Same semantics as [`DynamicTree::box_cast`].
    /// (b3TreeView_BoxCast)
    pub fn box_cast(
        &self,
        input: &BoxCastInput,
        mask_bits: u64,
        require_all_bits: bool,
        callback: impl FnMut(&BoxCastInput, i32, u64) -> f32,
    ) -> TreeStats {
        box_cast_tree_view(self, input, mask_bits, require_all_bits, callback)
    }
}
