// Port of box3d-cpp-reference/src/dynamic_tree.c and the tree group of
// include/box3d/collision.h — a dynamic AABB tree broad-phase inspired by
// Nathanael Presson's btDbvt.
//
// Split to satisfy the 800-line file limit:
// - insert.rs   — SAH sibling selection, rotations, insert/remove, proxy ops
// - query.rs    — AABB query, closest query, ray cast, box cast
// - rebuild.rs  — median-split partial/full rebuild
// - validate.rs — structure/metrics validation for tests
//
// The C `#else` SAH build heuristic (B3_TREE_HEURISTIC == 1) is compiled out
// upstream and is not ported; only the median-split heuristic is live.
//
// Save/Load are debug-only file I/O and are not needed by compound (which
// embeds nodes by memcpy); they are deferred.
//
// The C node uses two unions: {children | userData} and {parent | next}. The
// Rust node stores all four fields separately; the code writes them at exactly
// the points the C writes the corresponding union view, so observable behavior
// matches. (The C default node's children view {-1,-1} reads as u64::MAX
// through the userData view, which is reproduced explicitly.)
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

mod insert;
mod query;
mod rebuild;
mod validate;

use crate::core::NULL_INDEX;
use crate::math_functions::{Aabb, Vec3, VEC3_ZERO};

/// types.h: B3_DEFAULT_CATEGORY_BITS
pub const DEFAULT_CATEGORY_BITS: u64 = u64::MAX;
/// types.h: B3_DEFAULT_MASK_BITS
pub const DEFAULT_MASK_BITS: u64 = u64::MAX;

/// types.h: B3_DYNAMIC_TREE_VERSION
pub const DYNAMIC_TREE_VERSION: u64 = 0x93EDAF889FD30B4A;

pub(crate) const TREE_STACK_SIZE: usize = 1024;

// Tree node flags (enum b3TreeNodeFlags)
pub(crate) const ALLOCATED_NODE: u16 = 0x0001;
pub(crate) const ENLARGED_NODE: u16 = 0x0002;
pub(crate) const LEAF_NODE: u16 = 0x0004;

/// A node in the dynamic tree. For internal usage. (b3TreeNode)
#[derive(Debug, Clone, Copy)]
pub struct TreeNode {
    /// The node bounding box
    pub aabb: Aabb,
    /// Category bits for collision filtering
    pub category_bits: u64,
    /// Child node index 1 (internal nodes)
    pub child1: i32,
    /// Child node index 2 (internal nodes)
    pub child2: i32,
    /// User data (leaf nodes)
    pub user_data: u64,
    /// The node parent index (allocated nodes)
    pub parent: i32,
    /// The node freelist next index (free nodes)
    pub next: i32,
    pub height: u16,
    pub flags: u16,
}

impl TreeNode {
    /// static b3_defaultTreeNode
    pub(crate) fn default_node() -> TreeNode {
        TreeNode {
            aabb: Aabb {
                lower_bound: VEC3_ZERO,
                upper_bound: VEC3_ZERO,
            },
            category_bits: DEFAULT_CATEGORY_BITS,
            child1: NULL_INDEX,
            child2: NULL_INDEX,
            // The C children/userData union: {-1, -1} reads as u64::MAX.
            user_data: u64::MAX,
            parent: NULL_INDEX,
            next: NULL_INDEX,
            height: 0,
            flags: ALLOCATED_NODE,
        }
    }

    /// Zeroed storage matching the C memset of the node pool.
    fn zeroed() -> TreeNode {
        TreeNode {
            aabb: Aabb {
                lower_bound: VEC3_ZERO,
                upper_bound: VEC3_ZERO,
            },
            category_bits: 0,
            child1: 0,
            child2: 0,
            user_data: 0,
            parent: 0,
            next: 0,
            height: 0,
            flags: 0,
        }
    }

    /// (static b3IsLeaf)
    pub(crate) fn is_leaf(&self) -> bool {
        self.flags & LEAF_NODE != 0
    }

    /// (static b3IsAllocated)
    pub(crate) fn is_allocated(&self) -> bool {
        self.flags & ALLOCATED_NODE != 0
    }
}

/// These are performance results returned by dynamic tree queries. (b3TreeStats)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TreeStats {
    /// Number of internal nodes visited during the query
    pub node_visits: i32,
    /// Number of leaf nodes visited during the query
    pub leaf_visits: i32,
}

/// Input for casting an AABB through a dynamic tree. (b3BoxCastInput)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BoxCastInput {
    /// The AABB to cast, in the tree's frame.
    pub box_: Aabb,
    /// The sweep translation.
    pub translation: Vec3,
    /// The maximum fraction of the translation to consider, typically 1.
    pub max_fraction: f32,
}

/// The dynamic tree structure. (b3DynamicTree)
///
/// A dynamic AABB tree broad-phase. Leaf nodes are proxies with an AABB, used
/// to hold a user collision object. Nodes are pooled and relocatable, so node
/// indices are used rather than pointers.
#[derive(Debug, Clone)]
pub struct DynamicTree {
    /// Dynamic tree version for serialization compatibility.
    pub(crate) version: u64,
    /// The tree nodes. `nodes.len()` is the node capacity.
    pub(crate) nodes: Vec<TreeNode>,
    /// The root index
    pub(crate) root: i32,
    /// The number of allocated nodes
    pub(crate) node_count: i32,
    /// Node free list
    pub(crate) free_list: i32,
    /// Number of proxies created
    pub(crate) proxy_count: i32,
    /// Leaf indices for rebuild
    pub(crate) leaf_indices: Vec<i32>,
    /// Leaf bounding box centers for rebuild
    pub(crate) leaf_centers: Vec<Vec3>,
    /// Allocated space for rebuilding
    pub(crate) rebuild_capacity: i32,
}

impl Default for DynamicTree {
    fn default() -> Self {
        Self::new(16)
    }
}

impl DynamicTree {
    /// Constructing the tree initializes the node pool. (b3DynamicTree_Create)
    pub fn new(proxy_capacity: i32) -> DynamicTree {
        let capacity = crate::math_functions::max_int(proxy_capacity, 16);

        // maximum node count for a full binary tree is 2 * leafCount - 1
        let node_capacity = (2 * capacity - 1) as usize;
        let mut nodes = vec![TreeNode::zeroed(); node_capacity];

        // Build a linked list for the free list.
        for (i, node) in nodes.iter_mut().enumerate().take(node_capacity - 1) {
            node.next = i as i32 + 1;
        }
        nodes[node_capacity - 1].next = NULL_INDEX;

        DynamicTree {
            version: DYNAMIC_TREE_VERSION,
            nodes,
            root: NULL_INDEX,
            node_count: 0,
            free_list: 0,
            proxy_count: 0,
            leaf_indices: Vec::new(),
            leaf_centers: Vec::new(),
            rebuild_capacity: 0,
        }
    }

    /// Destroy the tree, freeing the node pool. (b3DynamicTree_Destroy)
    ///
    /// Rust would drop the storage automatically; this mirrors the C function
    /// (which leaves a zeroed struct) so ported call sites and tests read the
    /// same.
    pub fn destroy(&mut self) {
        *self = DynamicTree {
            version: 0,
            nodes: Vec::new(),
            root: 0,
            node_count: 0,
            free_list: 0,
            proxy_count: 0,
            leaf_indices: Vec::new(),
            leaf_centers: Vec::new(),
            rebuild_capacity: 0,
        };
    }

    /// Dynamic tree version for serialization compatibility.
    /// (b3DynamicTree::version / B3_DYNAMIC_TREE_VERSION)
    pub fn version(&self) -> u64 {
        self.version
    }

    pub(crate) fn node_capacity(&self) -> i32 {
        self.nodes.len() as i32
    }

    /// The number of allocated nodes.
    pub fn node_count(&self) -> i32 {
        self.node_count
    }

    /// Allocate a node from the pool. Grow the pool if necessary.
    /// (static b3AllocateNode)
    pub(crate) fn allocate_node(&mut self) -> i32 {
        // Expand the node pool as needed.
        if self.free_list == NULL_INDEX {
            debug_assert!(self.node_count == self.node_capacity());

            // The free list is empty. Rebuild a bigger pool.
            let old_capacity = self.nodes.len();
            let new_capacity = old_capacity + (old_capacity >> 1);
            self.nodes.resize(new_capacity, TreeNode::zeroed());

            // Build a linked list for the free list. The parent pointer
            // becomes the "next" pointer.
            for i in self.node_count as usize..new_capacity - 1 {
                self.nodes[i].next = i as i32 + 1;
            }
            self.nodes[new_capacity - 1].next = NULL_INDEX;
            self.free_list = self.node_count;
        }

        // Peel a node off the free list.
        let node_index = self.free_list;
        self.free_list = self.nodes[node_index as usize].next;
        self.nodes[node_index as usize] = TreeNode::default_node();
        self.node_count += 1;
        node_index
    }

    /// Return a node to the pool. (static b3FreeNode)
    pub(crate) fn free_node(&mut self, node_id: i32) {
        debug_assert!(0 <= node_id && node_id < self.node_capacity());
        debug_assert!(0 < self.node_count);
        self.nodes[node_id as usize].next = self.free_list;
        self.nodes[node_id as usize].flags = 0;
        self.free_list = node_id;
        self.node_count -= 1;
    }

    /// Get the number of proxies created. (b3DynamicTree_GetProxyCount)
    pub fn proxy_count(&self) -> i32 {
        self.proxy_count
    }

    /// Get the category bits on a proxy. (b3DynamicTree_GetCategoryBits)
    pub fn category_bits(&self, proxy_id: i32) -> u64 {
        debug_assert!(0 <= proxy_id && proxy_id < self.node_capacity());
        self.nodes[proxy_id as usize].category_bits
    }

    /// Get the height of the binary tree. (b3DynamicTree_GetHeight)
    pub fn height(&self) -> i32 {
        if self.root == NULL_INDEX {
            return 0;
        }

        self.nodes[self.root as usize].height as i32
    }

    /// Get the ratio of the sum of the node areas to the root area.
    /// (b3DynamicTree_GetAreaRatio)
    pub fn area_ratio(&self) -> f32 {
        use crate::aabb::perimeter;

        if self.root == NULL_INDEX {
            return 0.0;
        }

        let root = &self.nodes[self.root as usize];
        let root_area = perimeter(root.aabb);

        let mut total_area = 0.0;
        for (i, node) in self.nodes.iter().enumerate() {
            if !node.is_allocated() || node.is_leaf() || i as i32 == self.root {
                continue;
            }

            total_area += perimeter(node.aabb);
        }

        total_area / root_area
    }

    /// Get the bounding box that contains the entire tree.
    /// (b3DynamicTree_GetRootBounds)
    pub fn root_bounds(&self) -> Aabb {
        if self.root != NULL_INDEX {
            return self.nodes[self.root as usize].aabb;
        }

        Aabb {
            lower_bound: VEC3_ZERO,
            upper_bound: VEC3_ZERO,
        }
    }

    /// Get the number of bytes used by this tree. (b3DynamicTree_GetByteCount)
    ///
    /// Matches the C formula, which always counts leafBoxes/binIndices slots
    /// even when the median-split heuristic leaves those pointers null.
    pub fn byte_count(&self) -> i32 {
        let size = core::mem::size_of::<DynamicTree>()
            + core::mem::size_of::<TreeNode>() * self.nodes.len()
            + self.rebuild_capacity as usize
                * (core::mem::size_of::<i32>()
                    + core::mem::size_of::<Aabb>()
                    + core::mem::size_of::<Vec3>()
                    + core::mem::size_of::<i32>());

        size as i32
    }

    /// Get proxy user data. (b3DynamicTree_GetUserData)
    pub fn user_data(&self, proxy_id: i32) -> u64 {
        debug_assert!(0 <= proxy_id && proxy_id < self.node_capacity());
        self.nodes[proxy_id as usize].user_data
    }

    /// Get the AABB of a proxy. (b3DynamicTree_GetAABB)
    pub fn aabb(&self, proxy_id: i32) -> Aabb {
        debug_assert!(0 <= proxy_id && proxy_id < self.node_capacity());
        self.nodes[proxy_id as usize].aabb
    }
}

/// Category-bit filter matching the C ternary in Query/RayCast/BoxCast.
#[inline]
pub(crate) fn category_bits_match(
    category_bits: u64,
    mask_bits: u64,
    require_all_bits: bool,
) -> bool {
    if require_all_bits {
        (category_bits & mask_bits) == mask_bits
    } else {
        (category_bits & mask_bits) != 0
    }
}
