//! Tree Save / Load (C `b3DynamicTree_Save` / `b3DynamicTree_Load`), split out of the
//! parent `tree_demo` module to keep each file focused (and under the source-length
//! limit). See the parent module docs for the format rationale: C's raw `b3DynamicTree`
//! / `b3TreeNode` memory dump is not portable across the C/Rust node layouts, so this
//! ports the *behavior* with a small self-describing leaf stream.

#![allow(clippy::unnecessary_cast)]

use super::{compute_depths, generate, with_state, Proxy};
use box3d_rust::dynamic_tree::{DynamicTree, DYNAMIC_TREE_VERSION};
use box3d_rust::math_functions::{max_int, Aabb, Vec3, VEC3_ZERO};
use wasm_bindgen::prelude::*;

// --------------------------------------------------------------------------
// Save / Load (C `b3DynamicTree_Save` / `b3DynamicTree_Load`)
// --------------------------------------------------------------------------

/// Magic tag for the portable tree file. C dumps the raw `b3DynamicTree` struct +
/// `b3TreeNode[]`; that layout is neither portable nor browser-reachable, so the port
/// writes this self-describing leaf stream instead (see the module docs).
const TREE_FILE_MAGIC: [u8; 4] = *b"B3T1";
/// Per-leaf record size: `userData(u64) categoryBits(u64) lower(3×f32) upper(3×f32)`.
const LEAF_RECORD_LEN: usize = 8 + 8 + 6 * 4;

/// One saved leaf: its proxy `userData`, `categoryBits`, and world AABB.
struct SavedLeaf {
    user_data: u64,
    category_bits: u64,
    aabb: Aabb,
}

/// Serialize a tree's leaves to the portable binary format. Mirrors C
/// `b3DynamicTree_Save`: a header (magic + the real [`DYNAMIC_TREE_VERSION`], which
/// Load rejects on mismatch exactly like C's `version` guard) then every leaf. Leaves
/// are written in ascending `userData` order so a reload re-inserts them in the same
/// order the tree was originally built, reproducing its structure.
fn serialize_tree(tree: &DynamicTree) -> Vec<u8> {
    let mut leaves: Vec<SavedLeaf> = tree
        .node_views()
        .iter()
        .enumerate()
        .filter(|(_, n)| n.is_allocated && n.is_leaf)
        .map(|(i, n)| SavedLeaf {
            user_data: n.user_data,
            category_bits: tree.category_bits(i as i32),
            aabb: n.aabb,
        })
        .collect();
    leaves.sort_by_key(|l| l.user_data);

    let mut out = Vec::with_capacity(16 + leaves.len() * LEAF_RECORD_LEN);
    out.extend_from_slice(&TREE_FILE_MAGIC);
    out.extend_from_slice(&DYNAMIC_TREE_VERSION.to_le_bytes());
    out.extend_from_slice(&(leaves.len() as u32).to_le_bytes());
    for l in &leaves {
        out.extend_from_slice(&l.user_data.to_le_bytes());
        out.extend_from_slice(&l.category_bits.to_le_bytes());
        for v in [
            l.aabb.lower_bound.x,
            l.aabb.lower_bound.y,
            l.aabb.lower_bound.z,
            l.aabb.upper_bound.x,
            l.aabb.upper_bound.y,
            l.aabb.upper_bound.z,
        ] {
            out.extend_from_slice(&v.to_le_bytes());
        }
    }
    out
}

/// Little-endian scalar readers over a byte cursor (`None` past the end).
fn read_u32(bytes: &[u8], at: usize) -> Option<u32> {
    bytes
        .get(at..at + 4)
        .map(|s| u32::from_le_bytes(s.try_into().unwrap()))
}
fn read_u64(bytes: &[u8], at: usize) -> Option<u64> {
    bytes
        .get(at..at + 8)
        .map(|s| u64::from_le_bytes(s.try_into().unwrap()))
}
fn read_f32(bytes: &[u8], at: usize) -> Option<f32> {
    bytes
        .get(at..at + 4)
        .map(|s| f32::from_le_bytes(s.try_into().unwrap()))
}

/// Rebuild a tree + its proxy table from a [`serialize_tree`] buffer, applying `scale`
/// to every AABB (C `b3DynamicTree_Load`'s `scale` param / the sample's Load Scale).
/// Returns `None` for a bad magic, a version mismatch (C's `version` guard), or a
/// truncated stream. Leaves are re-inserted in ascending `userData` order.
fn deserialize_tree(bytes: &[u8], scale: f32) -> Option<(DynamicTree, Vec<Proxy>)> {
    if bytes.get(0..4)? != TREE_FILE_MAGIC {
        return None;
    }
    if read_u64(bytes, 4)? != DYNAMIC_TREE_VERSION {
        return None;
    }
    let count = read_u32(bytes, 12)? as usize;

    // Parse every leaf record first so a truncated file aborts before we mutate a tree.
    let mut leaves: Vec<SavedLeaf> = Vec::with_capacity(count);
    let mut max_user_data: u64 = 0;
    for i in 0..count {
        let base = 16 + i * LEAF_RECORD_LEN;
        let user_data = read_u64(bytes, base)?;
        let category_bits = read_u64(bytes, base + 8)?;
        let f = |k: usize| read_f32(bytes, base + 16 + k * 4);
        let aabb = Aabb {
            lower_bound: Vec3 {
                x: scale * f(0)?,
                y: scale * f(1)?,
                z: scale * f(2)?,
            },
            upper_bound: Vec3 {
                x: scale * f(3)?,
                y: scale * f(4)?,
                z: scale * f(5)?,
            },
        };
        max_user_data = max_user_data.max(user_data);
        leaves.push(SavedLeaf {
            user_data,
            category_bits,
            aabb,
        });
    }
    leaves.sort_by_key(|l| l.user_data);

    let mut tree = DynamicTree::new(max_int(512, count as i32));
    // Proxy table keyed by userData (C rebuilds `m_proxies[node->userData]`).
    let mut proxies: Vec<Proxy> = vec![
        Proxy {
            aabb: Aabb {
                lower_bound: VEC3_ZERO,
                upper_bound: VEC3_ZERO,
            },
            query_time_stamp: 0,
            ray_time_stamp: 0,
        };
        (max_user_data as usize + 1).max(count)
    ];
    for l in &leaves {
        tree.create_proxy(l.aabb, l.category_bits, l.user_data);
        proxies[l.user_data as usize] = Proxy {
            aabb: l.aabb,
            query_time_stamp: 0,
            ray_time_stamp: 0,
        };
    }
    tree.validate();
    Some((tree, proxies))
}

/// C "Save" button (`b3DynamicTree_Save`): serialize the current tree to bytes. JS
/// wraps these in a Blob and triggers a download (the browser has no `fopen`).
#[wasm_bindgen]
pub fn tree_save() -> Vec<u8> {
    with_state(|state| serialize_tree(&state.tree))
}

/// C "Load" button (`LoadTree` → `b3DynamicTree_Load( file, m_loadScale )`): replace
/// the live tree with one deserialized from `bytes`, scaling every AABB by `scale`.
/// Recomputes depths / area / height and regenerates the 1024 queries (C `Generate()`),
/// continuing the persistent RNG stream. Returns false on a bad / mismatched file.
#[wasm_bindgen]
pub fn tree_load(bytes: &[u8], scale: f32) -> bool {
    let Some((tree, proxies)) = deserialize_tree(bytes, scale) else {
        return false;
    };
    with_state(|state| {
        let area_ratio = tree.area_ratio();
        let (depths, height) = compute_depths(&tree);
        state.tree = tree;
        state.proxies = proxies;
        state.depths = depths;
        state.height = height;
        state.area_ratio = area_ratio;
        // C LoadTree resets the draw level to -1 and re-generates the query batch.
        generate(state);
    });
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a small deterministic tree via `create_proxy` (the incremental path
    /// `tree_reset` uses), matching the demo's initial state.
    fn build_tree(n: usize) -> (DynamicTree, Vec<Proxy>) {
        let mut tree = DynamicTree::new(512);
        let mut proxies = Vec::with_capacity(n);
        for i in 0..n {
            let base = i as f32;
            let aabb = Aabb {
                lower_bound: Vec3 {
                    x: base,
                    y: 0.5 * base,
                    z: -base,
                },
                upper_bound: Vec3 {
                    x: base + 1.0,
                    y: 0.5 * base + 2.0,
                    z: -base + 1.5,
                },
            };
            tree.create_proxy(aabb, 1, i as u64);
            proxies.push(Proxy {
                aabb,
                query_time_stamp: 0,
                ray_time_stamp: 0,
            });
        }
        tree.validate();
        (tree, proxies)
    }

    /// Sorted (by userData) leaf list for structural comparison.
    fn leaves_sorted(tree: &DynamicTree) -> Vec<(u64, [f32; 6])> {
        let mut v: Vec<(u64, [f32; 6])> = tree
            .node_views()
            .iter()
            .filter(|n| n.is_allocated && n.is_leaf)
            .map(|n| {
                (
                    n.user_data,
                    [
                        n.aabb.lower_bound.x,
                        n.aabb.lower_bound.y,
                        n.aabb.lower_bound.z,
                        n.aabb.upper_bound.x,
                        n.aabb.upper_bound.y,
                        n.aabb.upper_bound.z,
                    ],
                )
            })
            .collect();
        v.sort_by_key(|(u, _)| *u);
        v
    }

    #[test]
    fn save_load_round_trip_reproduces_tree() {
        let (tree, _proxies) = build_tree(64);
        let bytes = serialize_tree(&tree);

        let (loaded, loaded_proxies) =
            deserialize_tree(&bytes, 1.0).expect("round-trip load must succeed");

        // Leaf set, structure (height), area ratio, and bounds all reproduce exactly.
        assert_eq!(leaves_sorted(&tree), leaves_sorted(&loaded));
        assert_eq!(tree.proxy_count(), loaded.proxy_count());
        assert_eq!(compute_depths(&tree).1, compute_depths(&loaded).1);
        assert_eq!(tree.area_ratio(), loaded.area_ratio());
        assert_eq!(tree.root_bounds(), loaded.root_bounds());
        assert_eq!(loaded_proxies.len(), 64);
    }

    #[test]
    fn load_scale_scales_all_bounds() {
        let (tree, _) = build_tree(16);
        let bytes = serialize_tree(&tree);

        let scale = 0.5f32;
        let (loaded, _) = deserialize_tree(&bytes, scale).expect("scaled load must succeed");

        let orig = leaves_sorted(&tree);
        let scaled = leaves_sorted(&loaded);
        assert_eq!(orig.len(), scaled.len());
        for ((u0, a0), (u1, a1)) in orig.iter().zip(scaled.iter()) {
            assert_eq!(u0, u1);
            for k in 0..6 {
                assert_eq!(scale * a0[k], a1[k]);
            }
        }
    }

    #[test]
    fn load_rejects_bad_magic_and_version() {
        let (tree, _) = build_tree(8);
        let mut bytes = serialize_tree(&tree);

        // Corrupt magic.
        let mut bad_magic = bytes.clone();
        bad_magic[0] ^= 0xFF;
        assert!(deserialize_tree(&bad_magic, 1.0).is_none());

        // Corrupt version word (bytes 4..12).
        bytes[4] ^= 0xFF;
        assert!(deserialize_tree(&bytes, 1.0).is_none());
    }

    #[test]
    fn load_rejects_truncated_stream() {
        let (tree, _) = build_tree(8);
        let bytes = serialize_tree(&tree);
        // Drop the tail of the last leaf record.
        let truncated = &bytes[..bytes.len() - 4];
        assert!(deserialize_tree(truncated, 1.0).is_none());
    }
}
