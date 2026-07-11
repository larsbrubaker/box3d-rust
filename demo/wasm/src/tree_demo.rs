//! Dynamic tree demo.

use wasm_bindgen::prelude::*;

use box3d_rust::dynamic_tree::{DynamicTree, DEFAULT_CATEGORY_BITS, DEFAULT_MASK_BITS};
use box3d_rust::math_functions::{Aabb, Vec3};
use std::cell::RefCell;

thread_local! {
    static TREE: RefCell<DynamicTree> = RefCell::new(DynamicTree::new(32));
    static PROXY_IDS: RefCell<Vec<i32>> = const { RefCell::new(Vec::new()) };
}

fn make_aabb(cx: f32, cy: f32, cz: f32, h: f32) -> Aabb {
    Aabb {
        lower_bound: Vec3 {
            x: cx - h,
            y: cy - h,
            z: cz - h,
        },
        upper_bound: Vec3 {
            x: cx + h,
            y: cy + h,
            z: cz + h,
        },
    }
}

/// Reset the tree and insert a grid of proxies. Returns proxy count.
#[wasm_bindgen]
pub fn tree_reset(count: u32) -> i32 {
    TREE.with(|t| {
        PROXY_IDS.with(|ids| {
            let mut tree = DynamicTree::new(32);
            let mut list = Vec::new();
            let n = count.clamp(1, 27) as i32;
            let side = ((n as f32).cbrt().ceil() as i32).max(1);
            let mut placed = 0;
            for iz in 0..side {
                for iy in 0..side {
                    for ix in 0..side {
                        if placed >= n {
                            break;
                        }
                        let cx = (ix as f32 - (side - 1) as f32 * 0.5) * 2.2;
                        let cy = (iy as f32 - (side - 1) as f32 * 0.5) * 2.2;
                        let cz = (iz as f32 - (side - 1) as f32 * 0.5) * 2.2;
                        let id = tree.create_proxy(
                            make_aabb(cx, cy, cz, 0.55),
                            DEFAULT_CATEGORY_BITS,
                            placed as u64,
                        );
                        list.push(id);
                        placed += 1;
                    }
                }
            }
            *t.borrow_mut() = tree;
            *ids.borrow_mut() = list;
            placed
        })
    })
}

/// All proxy AABBs: [count, then count×(lx,ly,lz,ux,uy,uz)].
#[wasm_bindgen]
pub fn tree_proxy_aabbs() -> Vec<f32> {
    TREE.with(|t| {
        PROXY_IDS.with(|ids| {
            let tree = t.borrow();
            let list = ids.borrow();
            let mut out = vec![list.len() as f32];
            for &id in list.iter() {
                let a = tree.aabb(id);
                out.push(a.lower_bound.x);
                out.push(a.lower_bound.y);
                out.push(a.lower_bound.z);
                out.push(a.upper_bound.x);
                out.push(a.upper_bound.y);
                out.push(a.upper_bound.z);
            }
            out
        })
    })
}

/// Query an AABB centered at (cx,cy,cz) with half-extent h.
/// Returns [hit_count, node_visits, leaf_visits, then hit proxy indices as f32].
#[wasm_bindgen]
pub fn tree_query(cx: f32, cy: f32, cz: f32, h: f32) -> Vec<f32> {
    TREE.with(|t| {
        let tree = t.borrow();
        let mut hits = Vec::new();
        let stats = tree.query(
            make_aabb(cx, cy, cz, h),
            DEFAULT_MASK_BITS,
            false,
            |proxy_id, user| {
                let _ = proxy_id;
                hits.push(user as f32);
                true
            },
        );
        let mut out = vec![
            hits.len() as f32,
            stats.node_visits as f32,
            stats.leaf_visits as f32,
        ];
        out.extend(hits);
        out
    })
}

/// Tree metrics: [proxy_count, height, area_ratio].
#[wasm_bindgen]
pub fn tree_metrics() -> Vec<f32> {
    TREE.with(|t| {
        let tree = t.borrow();
        vec![
            tree.proxy_count() as f32,
            tree.height() as f32,
            tree.area_ratio(),
        ]
    })
}
