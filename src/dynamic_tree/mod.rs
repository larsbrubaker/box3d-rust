// Port of box3d-cpp-reference/src/dynamic_tree.c, src/dynamic_tree.h and the tree group
// of include/box3d/collision.h and include/box3d/types.h — a dynamic AABB tree
// broad-phase inspired by Nathanael Presson's btDbvt.
//
// The node layout follows upstream f555ee4 (ported from Box2D): 32-byte nodes where
// siblings sit together at an even index so two nodes fit in a 64 byte cache line. The
// root is at index zero and index one is always empty. Parents live in a separate array
// with the free pair list interleaved, and proxy data (user data, category bits) lives in
// a separate cold proxy array.
//
// Split to satisfy the 800-line file limit, mirroring the sections of dynamic_tree.c:
// - aabbv.rs     — the scalar (B3_SIMD_NONE) b3AABBV helpers from simd.h
// - alloc.rs     — sibling pair and proxy allocation
// - insert.rs    — SAH sibling selection, rotations, insert/remove, proxy ops
// - query.rs     — AABB query and closest query
// - cast.rs      — ray cast and box cast
// - rebuild.rs   — DFS-ordered median-split rebuild retaining unmoved subtrees
// - moved.rs     — moved-flag propagation, gathering, clearing, and refit
// - validate.rs  — structure validation and tree metrics
// - serialize.rs — b3DynamicTree_Save / b3DynamicTree_Load
// - view.rs      — b3TreeView, the read-only node/proxy view the queries and casts run on
//
// The C `#else` SAH build heuristic (B3_TREE_HEURISTIC == 1) is compiled out upstream and
// is not ported; only the median-split heuristic is live. The C allocates `leafBoxes` and
// `binIndices` only for the SAH heuristic, so the port has no storage for them (the
// byte count still counts them, like C).
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

mod aabbv;
mod alloc;
mod cast;
mod insert;
mod moved;
mod query;
mod rebuild;
mod serialize;
mod validate;
mod view;

pub(crate) use aabbv::{overlap_aabb_v, overlap_v, union_pair_v, union_v};
pub(crate) use serialize::{read_tree_node, write_tree_node};
pub use serialize::{DYNAMIC_TREE_SIZE, TREE_NODE_SIZE, TREE_PROXY_SIZE};
pub use view::{make_tree_view, TreeView};

use crate::core::NULL_INDEX;
use crate::math_functions::{Aabb, Vec3, VEC3_ZERO};

/// types.h: B3_DEFAULT_CATEGORY_BITS
pub const DEFAULT_CATEGORY_BITS: u64 = u64::MAX;
/// types.h: B3_DEFAULT_MASK_BITS
pub const DEFAULT_MASK_BITS: u64 = u64::MAX;

/// Dynamic tree version for compatibility testing. (B3_DYNAMIC_TREE_VERSION)
pub const DYNAMIC_TREE_VERSION: u64 = 0x1D6F4C2A73B80E91;

/// (B3_TREE_STACK_SIZE)
pub(crate) const TREE_STACK_SIZE: usize = 512;

// Used to mark a node as being moved such that pairs need to be generated and the tree
// may need to be rebuilt. This mark comes from a few sources:
// - updating the transform and AABB in the solver from movement
// - a joint disabling collision
// - creating a proxy
// - setting the transform on a body

/// (B3_MOVED_NODE)
pub const MOVED_NODE: u32 = 1 << 30;
/// (B3_LEAF_NODE)
pub const LEAF_NODE: u32 = 1 << 31;
/// (B3_NODE_INDEX_MASK: `0xFFFFFFFFu & ~( B3_MOVED_NODE | B3_LEAF_NODE )`)
pub const NODE_INDEX_MASK: u32 = !(MOVED_NODE | LEAF_NODE);

/// Used to indicate empty nodes. It also passes as a leaf so that internal node
/// processing will naturally skip it. (B3_EMPTY_NODE)
pub const EMPTY_NODE: u32 = NODE_INDEX_MASK | LEAF_NODE;

/// (B3_ROOT_NODE)
pub const ROOT_NODE: i32 = 0;

/// A node in the dynamic tree. Siblings sit together at an even index so two nodes fit in
/// a 64 byte cache line. The root is at index zero and index one always empty.
/// (b3TreeNode — 32 bytes, like C)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TreeNode {
    /// The node bounding box
    pub aabb: Aabb,

    /// bit 31 : 1 for leaf node
    /// bit 30 : 1 for moved flag
    /// bits 0-29 : index of the sibling pair node or the proxy id for a leaf
    pub flag_index: u32,

    /// The C union `{ int32_t height; int32_t shapeIndex; }`: the height of an internal
    /// node (a leaf has zero height), or for a leaf the shape index truncated from the
    /// proxy user data. Read the leaf view through [`TreeNode::shape_index`].
    pub height: i32,
}

impl TreeNode {
    /// Zeroed storage matching the C `B3_GROW_ZERO` / `b3AllocZero` of the node arrays.
    pub(crate) const fn zeroed() -> TreeNode {
        TreeNode {
            aabb: Aabb {
                lower_bound: VEC3_ZERO,
                upper_bound: VEC3_ZERO,
            },
            flag_index: 0,
            height: 0,
        }
    }

    /// The shape index view of the height/shapeIndex union. (b3TreeNode::shapeIndex)
    #[inline]
    pub fn shape_index(&self) -> i32 {
        self.height
    }

    /// (b3IsLeaf)
    #[inline]
    pub fn is_leaf(&self) -> bool {
        (self.flag_index & LEAF_NODE) == LEAF_NODE
    }

    /// (b3IsNodeMoved)
    #[inline]
    pub fn is_moved(&self) -> bool {
        (self.flag_index & MOVED_NODE) == MOVED_NODE
    }

    /// (b3IsEmptyNode)
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.flag_index == EMPTY_NODE
    }

    /// (b3GetLeftChild)
    #[inline]
    pub fn left_child(&self) -> i32 {
        (self.flag_index & NODE_INDEX_MASK) as i32
    }

    /// (b3GetProxyId)
    #[inline]
    pub fn proxy_id(&self) -> i32 {
        (self.flag_index & NODE_INDEX_MASK) as i32
    }
}

/// (b3MakeEmptyNode)
#[inline]
pub(crate) fn make_empty_node() -> TreeNode {
    TreeNode {
        aabb: Aabb {
            lower_bound: Vec3 {
                x: f32::INFINITY,
                y: f32::INFINITY,
                z: f32::INFINITY,
            },
            upper_bound: Vec3 {
                x: -f32::INFINITY,
                y: -f32::INFINITY,
                z: -f32::INFINITY,
            },
        },
        flag_index: EMPTY_NODE,
        height: 0,
    }
}

/// (b3GetRootPair)
#[inline]
pub(crate) fn get_root_pair(nodes: &[TreeNode]) -> i32 {
    let root = &nodes[ROOT_NODE as usize];
    if root.is_leaf() {
        ROOT_NODE
    } else {
        root.left_child()
    }
}

/// Separate storage for tree leaves. (b3TreeProxy — 24 bytes, like C)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TreeProxy {
    /// User data is an index instead of void* because it is used internally as a shape index.
    pub user_data: u64,

    /// Category bits for collision filtering.
    pub category_bits: u64,

    /// The leaf node. [`NULL_INDEX`] for a free proxy.
    pub node: i32,

    /// Next free proxy.
    pub next: i32,
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
/// A dynamic AABB tree broad-phase. Leaf nodes are proxies with an AABB, used to hold a
/// user collision object. Nodes are pooled and relocatable, so node indices are used
/// rather than pointers. The C arrays map to `Vec`s whose length is the C capacity; a C
/// null pointer maps to an empty `Vec`.
#[derive(Debug, Clone)]
pub struct DynamicTree {
    /// The tree version. Used to check compatibility if the tree is serialized.
    pub(crate) version: u64,

    /// Array of nodes. The root is at index zero and index 1 is empty.
    /// Otherwise siblings are paired at even indices. Has holes for free node pairs.
    /// `nodes.len()` is the C `nodeCapacity`.
    pub(crate) nodes: Vec<TreeNode>,

    /// Parent index per node. The free list is interweaved. Empty (C null) once destroyed.
    pub(crate) parents: Vec<i32>,

    /// Proxy data split from node array as cold data. `proxies.len()` is the C
    /// `proxyCapacity`.
    pub(crate) proxies: Vec<TreeProxy>,

    /// Every allocated node has a lower index than this.
    pub(crate) node_end: i32,

    /// Free pairs below nodeEnd
    pub(crate) pair_free_list: i32,

    /// Number of proxies created
    pub(crate) proxy_count: i32,

    /// Proxy free list
    pub(crate) proxy_free_list: i32,

    /// Array of nodes for rebuild. Empty (C null) until a rebuild allocates it.
    pub(crate) swap_nodes: Vec<TreeNode>,

    /// Leaf indices for rebuild
    pub(crate) leaf_indices: Vec<i32>,

    /// Leaves for the rebuild. May represent a proxy or a retained subtree.
    pub(crate) leaf_nodes: Vec<TreeNode>,

    /// Leaf bounding box centers for rebuild
    pub(crate) leaf_centers: Vec<Vec3>,

    /// Allocated space for rebuilding
    pub(crate) rebuild_capacity: i32,

    /// Rebuild orders the nodes so the children follow parents. Cache friendly for queries
    /// and refitting. The order can be disrupted by proxy creation.
    pub(crate) dfs_ordered: bool,
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

        // A tree of n proxies has 2n - 1 nodes plus the empty node beside the root
        let node_capacity = (2 * capacity) as usize;
        let mut nodes = vec![TreeNode::zeroed(); node_capacity];
        let mut parents = vec![0i32; node_capacity];

        // The root and the empty node always exist
        nodes[ROOT_NODE as usize] = make_empty_node();
        nodes[ROOT_NODE as usize + 1] = make_empty_node();
        parents[ROOT_NODE as usize] = NULL_INDEX;
        parents[ROOT_NODE as usize + 1] = NULL_INDEX;

        let mut proxies = vec![TreeProxy::default(); capacity as usize];

        // Build a linked list for the free list.
        for (i, proxy) in proxies.iter_mut().enumerate().take(capacity as usize - 1) {
            proxy.node = NULL_INDEX;
            proxy.next = i as i32 + 1;
        }

        proxies[capacity as usize - 1].node = NULL_INDEX;
        proxies[capacity as usize - 1].next = NULL_INDEX;

        DynamicTree {
            version: DYNAMIC_TREE_VERSION,
            nodes,
            parents,
            proxies,
            // This is the bump index.
            node_end: 2,
            pair_free_list: NULL_INDEX,
            proxy_count: 0,
            proxy_free_list: 0,
            swap_nodes: Vec::new(),
            leaf_indices: Vec::new(),
            leaf_nodes: Vec::new(),
            leaf_centers: Vec::new(),
            rebuild_capacity: 0,
            dfs_ordered: true,
        }
    }

    /// The zeroed tree C leaves behind after `memset( tree, 0, sizeof( b3DynamicTree ) )`.
    pub(crate) fn zeroed() -> DynamicTree {
        DynamicTree {
            version: 0,
            nodes: Vec::new(),
            parents: Vec::new(),
            proxies: Vec::new(),
            node_end: 0,
            pair_free_list: 0,
            proxy_count: 0,
            proxy_free_list: 0,
            swap_nodes: Vec::new(),
            leaf_indices: Vec::new(),
            leaf_nodes: Vec::new(),
            leaf_centers: Vec::new(),
            rebuild_capacity: 0,
            dfs_ordered: false,
        }
    }

    /// Destroy the tree, freeing the node pool. (b3DynamicTree_Destroy)
    ///
    /// Rust would drop the storage automatically; this mirrors the C function (which
    /// leaves a zeroed struct) so ported call sites and tests read the same.
    pub fn destroy(&mut self) {
        *self = DynamicTree::zeroed();
    }

    /// Dynamic tree version for serialization compatibility.
    /// (b3DynamicTree::version / B3_DYNAMIC_TREE_VERSION)
    pub fn version(&self) -> u64 {
        self.version
    }

    /// The allocated node space. (b3DynamicTree::nodeCapacity)
    pub fn node_capacity(&self) -> i32 {
        self.nodes.len() as i32
    }

    /// The allocated proxy space. (b3DynamicTree::proxyCapacity)
    pub fn proxy_capacity(&self) -> i32 {
        self.proxies.len() as i32
    }

    /// Every allocated node has a lower index than this. (b3DynamicTree::nodeEnd)
    pub fn node_end(&self) -> i32 {
        self.node_end
    }

    /// Whether the nodes are in depth-first order. (b3DynamicTree::dfsOrdered)
    pub fn dfs_ordered(&self) -> bool {
        self.dfs_ordered
    }

    /// Get the number of proxies created. (b3DynamicTree_GetProxyCount)
    pub fn proxy_count(&self) -> i32 {
        self.proxy_count
    }

    /// Get proxy user data. (b3DynamicTree_GetUserData)
    pub fn user_data(&self, proxy_id: i32) -> u64 {
        self.proxies[proxy_id as usize].user_data
    }

    /// Get the AABB of a proxy. (b3DynamicTree_GetAABB)
    pub fn aabb(&self, proxy_id: i32) -> Aabb {
        self.nodes[self.proxies[proxy_id as usize].node as usize].aabb
    }

    /// Does the root carry the moved mark? (b3HasTreeMoved)
    #[inline]
    pub fn has_tree_moved(&self) -> bool {
        self.nodes[ROOT_NODE as usize].is_moved()
    }

    /// Does the next rebuild have work to do? (b3NeedsRebuild)
    #[inline]
    pub fn needs_rebuild(&self) -> bool {
        self.nodes[ROOT_NODE as usize].is_moved() || !self.dfs_ordered
    }

    /// A read-only view of every node below `node_end` (live nodes, the empty node beside
    /// the root, and free pairs), indexed identically to the C `tree.nodes[i]` array so a
    /// caller can run the Tree Benchmark sample's breadth-first depth walk and per-level
    /// AABB draw. This mirrors the C sample reaching directly into `b3DynamicTree::nodes`
    /// and `b3DynamicTree::proxies`.
    pub fn node_views(&self) -> Vec<TreeNodeView> {
        self.nodes[..self.node_end.max(0) as usize]
            .iter()
            .map(|n| {
                let is_leaf = n.is_leaf();
                let is_empty = n.is_empty();
                let user_data = if is_leaf && !is_empty {
                    self.proxies[n.proxy_id() as usize].user_data
                } else {
                    0
                };
                TreeNodeView {
                    aabb: n.aabb,
                    left_child: if is_leaf { NULL_INDEX } else { n.left_child() },
                    proxy_id: if is_leaf && !is_empty {
                        n.proxy_id()
                    } else {
                        NULL_INDEX
                    },
                    user_data,
                    is_leaf,
                    is_empty,
                    is_moved: n.is_moved(),
                }
            })
            .collect()
    }
}

/// Read-only view of a single dynamic-tree node, for visualization tooling that needs
/// the structural fields the C sample reads off `b3TreeNode` directly.
#[derive(Debug, Clone, Copy)]
pub struct TreeNodeView {
    /// The node bounding box. (b3TreeNode::aabb)
    pub aabb: Aabb,
    /// Index of the children sibling pair for an internal node, else
    /// [`crate::core::NULL_INDEX`]. (b3GetLeftChild)
    pub left_child: i32,
    /// The proxy id for a live leaf, else [`crate::core::NULL_INDEX`]. (b3GetProxyId)
    pub proxy_id: i32,
    /// The proxy user data for a live leaf. (b3TreeProxy::userData)
    pub user_data: u64,
    /// Whether this node is a leaf (`b3IsLeaf`). Empty nodes also pass as leaves.
    pub is_leaf: bool,
    /// Whether this node is empty (`b3IsEmptyNode`).
    pub is_empty: bool,
    /// Whether this node carries the moved mark (`b3IsNodeMoved`).
    pub is_moved: bool,
}

/// Category-bit filter. (b3TestCategory)
#[inline]
pub(crate) fn test_category(category_bits: u64, mask_bits: u64, require_all_bits: bool) -> bool {
    if require_all_bits {
        (category_bits & mask_bits) == mask_bits
    } else {
        (category_bits & mask_bits) != 0
    }
}
