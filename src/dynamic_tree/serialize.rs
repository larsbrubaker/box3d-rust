// b3DynamicTree_Save / b3DynamicTree_Load from dynamic_tree.c.
//
// C `fwrite`s the raw `b3DynamicTree` struct (pointers and rebuild scratch zeroed, the
// node capacity trimmed to the node end) followed by the nodes and parents up to the node
// end and the proxies up to the proxy capacity. The port writes the same bytes in the same
// order using the 64-bit little-endian C layout, so files interchange with the C library:
//
// b3DynamicTree (112 bytes)
//   0 version u64 | 8 nodes* | 16 parents* | 24 proxies*
//   32 nodeEnd | 36 nodeCapacity | 40 pairFreeList | 44 proxyCount | 48 proxyCapacity
//   52 proxyFreeList | 56 swapNodes* | 64 leafIndices* | 72 leafNodes* | 80 leafBoxes*
//   88 leafCenters* | 96 binIndices* | 104 rebuildCapacity | 108 dfsOrdered u8 | 3 pad
// b3TreeNode (32 bytes): aabb 6 x f32 | flagIndex u32 | height/shapeIndex i32
// b3TreeProxy (24 bytes): userData u64 | categoryBits u64 | node i32 | next i32
//
// The byte-level functions are exposed so callers without a file system (the wasm demo)
// can use the same format.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::{DynamicTree, TreeNode, TreeProxy, DYNAMIC_TREE_VERSION};
use crate::math_functions::{mul_sv, Aabb, Vec3};

/// sizeof( b3DynamicTree ) on 64-bit targets.
pub const DYNAMIC_TREE_SIZE: usize = 112;
/// sizeof( b3TreeNode )
pub const TREE_NODE_SIZE: usize = 32;
/// sizeof( b3TreeProxy )
pub const TREE_PROXY_SIZE: usize = 24;

pub(crate) fn write_tree_node(buf: &mut Vec<u8>, node: &TreeNode) {
    let a = node.aabb;
    for v in [
        a.lower_bound.x,
        a.lower_bound.y,
        a.lower_bound.z,
        a.upper_bound.x,
        a.upper_bound.y,
        a.upper_bound.z,
    ] {
        buf.extend_from_slice(&v.to_le_bytes());
    }
    buf.extend_from_slice(&node.flag_index.to_le_bytes());
    buf.extend_from_slice(&node.height.to_le_bytes());
}

pub(crate) fn write_tree_proxy(buf: &mut Vec<u8>, proxy: &TreeProxy) {
    buf.extend_from_slice(&proxy.user_data.to_le_bytes());
    buf.extend_from_slice(&proxy.category_bits.to_le_bytes());
    buf.extend_from_slice(&proxy.node.to_le_bytes());
    buf.extend_from_slice(&proxy.next.to_le_bytes());
}

fn read_u64(buf: &[u8], o: usize) -> u64 {
    u64::from_le_bytes(buf[o..o + 8].try_into().unwrap())
}
fn read_u32(buf: &[u8], o: usize) -> u32 {
    u32::from_le_bytes(buf[o..o + 4].try_into().unwrap())
}
fn read_i32(buf: &[u8], o: usize) -> i32 {
    i32::from_le_bytes(buf[o..o + 4].try_into().unwrap())
}
fn read_f32(buf: &[u8], o: usize) -> f32 {
    f32::from_le_bytes(buf[o..o + 4].try_into().unwrap())
}
fn read_vec3(buf: &[u8], o: usize) -> Vec3 {
    Vec3 {
        x: read_f32(buf, o),
        y: read_f32(buf, o + 4),
        z: read_f32(buf, o + 8),
    }
}

/// Read a `b3TreeNode` at byte offset `o`.
pub(crate) fn read_tree_node(buf: &[u8], o: usize) -> TreeNode {
    TreeNode {
        aabb: Aabb {
            lower_bound: read_vec3(buf, o),
            upper_bound: read_vec3(buf, o + 12),
        },
        flag_index: read_u32(buf, o + 24),
        height: read_i32(buf, o + 28),
    }
}

/// Read a `b3TreeProxy` at byte offset `o`.
pub(crate) fn read_tree_proxy(buf: &[u8], o: usize) -> TreeProxy {
    TreeProxy {
        user_data: read_u64(buf, o),
        category_bits: read_u64(buf, o + 8),
        node: read_i32(buf, o + 16),
        next: read_i32(buf, o + 20),
    }
}

/// Write the `b3DynamicTree` struct image with every pointer and the rebuild scratch
/// zeroed, as C does before `fwrite`.
pub(crate) fn write_tree_header(buf: &mut Vec<u8>, tree: &DynamicTree, node_capacity: i32) {
    let start = buf.len();
    buf.extend_from_slice(&tree.version.to_le_bytes());
    // nodes, parents, proxies
    buf.extend_from_slice(&[0u8; 24]);
    buf.extend_from_slice(&tree.node_end.to_le_bytes());
    buf.extend_from_slice(&node_capacity.to_le_bytes());
    buf.extend_from_slice(&tree.pair_free_list.to_le_bytes());
    buf.extend_from_slice(&tree.proxy_count.to_le_bytes());
    buf.extend_from_slice(&tree.proxy_capacity().to_le_bytes());
    buf.extend_from_slice(&tree.proxy_free_list.to_le_bytes());
    // swapNodes, leafIndices, leafNodes, leafBoxes, leafCenters, binIndices
    buf.extend_from_slice(&[0u8; 48]);
    // rebuildCapacity
    buf.extend_from_slice(&0i32.to_le_bytes());
    buf.push(tree.dfs_ordered as u8);
    // tail padding
    buf.extend_from_slice(&[0u8; 3]);
    debug_assert_eq!(buf.len() - start, DYNAMIC_TREE_SIZE);
}

/// The scalar fields of a `b3DynamicTree` struct image.
pub(crate) struct TreeHeader {
    pub version: u64,
    pub node_end: i32,
    pub node_capacity: i32,
    pub pair_free_list: i32,
    pub proxy_count: i32,
    pub proxy_capacity: i32,
    pub proxy_free_list: i32,
    pub dfs_ordered: bool,
}

/// Read the scalar fields of a `b3DynamicTree` struct image at byte offset `o`.
pub(crate) fn read_tree_header(buf: &[u8], o: usize) -> TreeHeader {
    TreeHeader {
        version: read_u64(buf, o),
        node_end: read_i32(buf, o + 32),
        node_capacity: read_i32(buf, o + 36),
        pair_free_list: read_i32(buf, o + 40),
        proxy_count: read_i32(buf, o + 44),
        proxy_capacity: read_i32(buf, o + 48),
        proxy_free_list: read_i32(buf, o + 52),
        dfs_ordered: buf[o + 108] != 0,
    }
}

impl DynamicTree {
    /// Serialize this tree in the `b3DynamicTree_Save` file format. Returns `None` when
    /// C would write nothing: a tree without nodes, parents, or proxies (a destroyed tree).
    pub fn save_to_bytes(&self) -> Option<Vec<u8>> {
        if self.nodes.is_empty() || self.parents.is_empty() || self.proxies.is_empty() {
            return None;
        }

        let node_end = self.node_end.max(0) as usize;
        let mut buf = Vec::with_capacity(
            DYNAMIC_TREE_SIZE
                + node_end * (TREE_NODE_SIZE + 4)
                + self.proxies.len() * TREE_PROXY_SIZE,
        );

        // Copy to allow setting some fields to zero. The node capacity is trimmed to the
        // node end.
        write_tree_header(&mut buf, self, self.node_end);

        if node_end > 0 {
            for node in &self.nodes[..node_end] {
                write_tree_node(&mut buf, node);
            }
            for parent in &self.parents[..node_end] {
                buf.extend_from_slice(&parent.to_le_bytes());
            }
        }

        for proxy in &self.proxies {
            write_tree_proxy(&mut buf, proxy);
        }

        Some(buf)
    }

    /// Restore a tree from the `b3DynamicTree_Save` file format, scaling every non-empty
    /// node box. A bad image yields the zeroed tree (no nodes), like C.
    pub fn load_from_bytes(bytes: &[u8], scale: f32) -> DynamicTree {
        if bytes.len() < DYNAMIC_TREE_SIZE {
            return DynamicTree::zeroed();
        }

        let header = read_tree_header(bytes, 0);
        if header.version != DYNAMIC_TREE_VERSION {
            return DynamicTree::zeroed();
        }

        let ok = header.node_capacity >= 2
            && header.proxy_capacity >= 1
            && (header.node_end & 1) == 0
            && header.node_end >= 2
            && header.node_end <= header.node_capacity;
        if !ok {
            return DynamicTree::zeroed();
        }

        let node_capacity = header.node_capacity as usize;
        let proxy_capacity = header.proxy_capacity as usize;
        let needed = DYNAMIC_TREE_SIZE
            + node_capacity * (TREE_NODE_SIZE + 4)
            + proxy_capacity * TREE_PROXY_SIZE;
        if bytes.len() < needed {
            return DynamicTree::zeroed();
        }

        let mut o = DYNAMIC_TREE_SIZE;
        let mut nodes = Vec::with_capacity(node_capacity);
        for _ in 0..node_capacity {
            nodes.push(read_tree_node(bytes, o));
            o += TREE_NODE_SIZE;
        }
        let mut parents = Vec::with_capacity(node_capacity);
        for _ in 0..node_capacity {
            parents.push(read_i32(bytes, o));
            o += 4;
        }
        let mut proxies = Vec::with_capacity(proxy_capacity);
        for _ in 0..proxy_capacity {
            proxies.push(read_tree_proxy(bytes, o));
            o += TREE_PROXY_SIZE;
        }

        // An empty node has infinite bounds, so the scale has to skip it.
        for node in &mut nodes[..header.node_end as usize] {
            if node.is_empty() {
                continue;
            }

            node.aabb.lower_bound = mul_sv(scale, node.aabb.lower_bound);
            node.aabb.upper_bound = mul_sv(scale, node.aabb.upper_bound);
        }

        DynamicTree {
            version: header.version,
            nodes,
            parents,
            proxies,
            node_end: header.node_end,
            pair_free_list: header.pair_free_list,
            proxy_count: header.proxy_count,
            proxy_free_list: header.proxy_free_list,
            swap_nodes: Vec::new(),
            leaf_indices: Vec::new(),
            leaf_nodes: Vec::new(),
            leaf_centers: Vec::new(),
            rebuild_capacity: 0,
            dfs_ordered: header.dfs_ordered,
        }
    }

    /// Save this tree to a file for debugging. (b3DynamicTree_Save)
    ///
    /// Like C, failures (no file, nothing to save) are silent.
    pub fn save(&self, file_name: &str) {
        if let Some(bytes) = self.save_to_bytes() {
            let _ = std::fs::write(file_name, bytes);
        }
    }

    /// Load a tree from a file for debugging. (b3DynamicTree_Load)
    ///
    /// A missing or bad file yields the zeroed tree (no nodes), like C.
    pub fn load(file_name: &str, scale: f32) -> DynamicTree {
        match std::fs::read(file_name) {
            Ok(bytes) => DynamicTree::load_from_bytes(&bytes, scale),
            Err(_) => DynamicTree::zeroed(),
        }
    }
}
