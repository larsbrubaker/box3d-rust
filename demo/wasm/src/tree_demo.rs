//! Tree Benchmark — a 1:1 port of `box3d-cpp-reference/samples/sample_tree.cpp`
//! (`TreeBenchmark`, registered as `"Tree" / "Benchmark"`).
//!
//! The sample loads an AABB record file (`data/trees/bounds0{1,2,3}.txt`) into a
//! `b3DynamicTree_Create( 512 )`, generates 1024 deterministic ray / overlap-AABB /
//! closest-point-sphere queries, profiles the three query kinds, and visualizes the
//! leaf boxes (blue, or gray when hit by the current test) plus the current test's
//! ray / overlap box / sphere and closest point. A Level slider draws the internal
//! nodes at a chosen BFS depth.
//!
//! # Deviations from C (disclosed)
//!
//! - **File I/O.** The browser has no `fopen`; JS fetches the record file and passes
//!   its text to [`tree_reset`], which parses it with the identical
//!   `lowerX lowerY lowerZ upperX upperY upperZ` grammar (`sscanf( "%f %f %f %f %f %f" )`)
//!   and per-file `scale` / `zUp` config from the C `m_config` table.
//! - **Timing.** wasm has no `std::time`; the JS page times the isolated
//!   [`tree_reset`], [`tree_rebuild`], and `tree_profile_*` calls with
//!   `performance.now()` (the C `b3GetTicks` / `b3GetMilliseconds` reductions).
//! - **Save / Load.** `b3DynamicTree_Save` / `b3DynamicTree_Load` are debug-only file
//!   I/O and are not ported in box3d-rust; the C sample's Save / Load buttons and the
//!   Load Scale slider are therefore dropped (disclosed on the page).
//! - **RNG.** [`Generate`](Self) mirrors the C stream exactly: the C `srand( 42 )` seeds
//!   the C-library `rand()`, which the sample never calls (it uses `RandomVec3` /
//!   `RandomFloatRange` over `g_randomSeed`), so it is a no-op on the query stream. The
//!   stream is `g_randomSeed = RAND_SEED = 12345`, set once at Sample construction and
//!   *not* reset by `Generate`, so a File-change rebuild continues the same stream — as
//!   in C. Distance culling of drawn boxes is a pure view concern and is done JS-side
//!   from the live camera position, not here.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)]

use crate::rng::XorShift32;
use box3d_rust::core::NULL_INDEX;
use box3d_rust::dynamic_tree::{DynamicTree, DEFAULT_MASK_BITS};
use box3d_rust::geometry::RayCastInput;
use box3d_rust::math_functions::{
    aabb_extents, add, closest_point_to_aabb, distance_squared, max, max_int, min, mul_sv, sub,
    Aabb, Vec3, VEC3_ZERO,
};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

/// C `TreeBenchmark::m_testCount` — the query batch size (release build).
const TEST_COUNT: usize = 1024;
/// C `RAND_SEED` (`shared/utils.h`) — the `g_randomSeed` value at Sample construction.
const RAND_SEED: u32 = 12345;

/// C `Proxy` (sample_tree.cpp:32): a leaf AABB plus the timestamps of the last ray /
/// overlap-or-closest query that touched it (used to color a leaf gray when hit).
#[derive(Clone, Copy)]
struct Proxy {
    aabb: Aabb,
    query_time_stamp: i32,
    ray_time_stamp: i32,
}

/// C `Ray` (sample_tree.cpp:40).
#[derive(Clone, Copy)]
struct Ray {
    origin: Vec3,
    translation: Vec3,
}

/// C `b3Sphere` closest-point query (center + radius).
#[derive(Clone, Copy)]
struct SphereQuery {
    center: Vec3,
    radius: f32,
}

/// Per-file `scale` / `zUp` (C `FileConfig` / `m_config`, sample_tree.cpp:59-61).
const FILE_CONFIG: [(f32, bool); 3] = [(1.0, false), (1.0, false), (0.01, false)];

struct TreeState {
    tree: DynamicTree,
    /// C `m_proxies` (an `unordered_map<uint64_t, Proxy>` keyed 0..count-1) — a `Vec`
    /// indexed by the proxy `userData`, which the callbacks use to map back.
    proxies: Vec<Proxy>,
    file_index: usize,
    /// The `g_randomSeed` XorShift stream — persistent across rebuilds (see module docs).
    seed: u32,
    rays: Vec<Ray>,
    overlap_queries: Vec<Aabb>,
    closest_queries: Vec<SphereQuery>,
    /// BFS depth of each node slot (C `m_depths`), indexed like `tree.node_views()`.
    depths: Vec<i32>,
    height: i32,
    area_ratio: f32,
    time_stamp: i32,
    test_index: usize,
    /// Closest-point result of the last per-frame closest query (C `m_closestPoint` /
    /// `m_haveClosest`).
    have_closest: bool,
    closest_point: Vec3,
}

thread_local! {
    static STATE: RefCell<Option<TreeState>> = const { RefCell::new(None) };
}

fn with_state<R>(f: impl FnOnce(&mut TreeState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("tree demo not initialized — call tree_reset first"))
    })
}

/// Breadth-first depth walk (C `TreeBenchmark::ComputeDepths`): assign each node its
/// depth from the root and return the maximum depth (the tree "height" readout).
fn compute_depths(tree: &DynamicTree) -> (Vec<i32>, i32) {
    let nodes = tree.node_views();
    let mut depths = vec![0i32; nodes.len()];
    let root = tree.root_index();
    if root == NULL_INDEX || nodes.is_empty() {
        return (depths, 0);
    }

    // C uses a plain FIFO sized nodeCount; a Vec queue is behaviorally identical.
    let mut queue: Vec<i32> = Vec::with_capacity(nodes.len());
    queue.push(root);
    let mut front = 0usize;
    let mut depth = 0;

    while front < queue.len() {
        let index = queue[front];
        front += 1;
        let node = nodes[index as usize];
        if node.parent == NULL_INDEX {
            depths[index as usize] = 0;
        } else {
            depths[index as usize] = depths[node.parent as usize] + 1;
            depth = max_int(depth, depths[index as usize]);
        }
        if !node.is_leaf {
            queue.push(node.child1);
            queue.push(node.child2);
        }
    }

    (depths, depth)
}

/// C `TreeBenchmark::Generate` — build 1024 deterministic ray / overlap / closest
/// queries from the current tree bounds. The RNG stream persists in `state.seed`.
fn generate(state: &mut TreeState) {
    // C `srand( 42 )`: seeds the unused C-library rand(); a no-op on this stream.
    let mut rng = XorShift32::with_seed(state.seed);

    let bounds = state.tree.root_bounds();
    let extents = aabb_extents(bounds);
    let radius = (extents.x + extents.y + extents.z) / 3.0;

    state.rays.clear();
    state.overlap_queries.clear();
    state.closest_queries.clear();

    for _ in 0..TEST_COUNT {
        let origin = rng.vec3(bounds.lower_bound, bounds.upper_bound);
        let end = rng.vec3(bounds.lower_bound, bounds.upper_bound);
        state.rays.push(Ray {
            origin,
            translation: sub(end, origin),
        });

        let s = rng.range(0.01, 0.2);
        let c = rng.vec3(bounds.lower_bound, bounds.upper_bound);
        let p1 = sub(c, mul_sv(s, extents));
        let p2 = add(c, mul_sv(s, extents));
        state.overlap_queries.push(Aabb {
            lower_bound: min(p1, p2),
            upper_bound: max(p1, p2),
        });
        state.closest_queries.push(SphereQuery {
            center: c,
            radius: s * radius,
        });
    }

    state.seed = rng.seed();
}

/// Parse one record line (C `sscanf( "%f %f %f %f %f %f" )` + trim + `#` comment skip),
/// applying the file's `scale` / `zUp`. Returns `None` for blank / comment / malformed.
fn parse_record(line: &str, scale: f32, z_up: bool) -> Option<Aabb> {
    let trimmed = line.trim_start_matches([' ', '\t']);
    if trimmed.is_empty() || trimmed.starts_with('\n') || trimmed.starts_with('#') {
        return None;
    }
    let mut it = trimmed.split_whitespace();
    let mut v = [0.0f32; 6];
    for slot in v.iter_mut() {
        *slot = it.next()?.parse::<f32>().ok()?;
    }
    let (b1, b2, b3, b4, b5, b6) = (v[0], v[1], v[2], v[3], v[4], v[5]);
    let aabb = if z_up {
        Aabb {
            lower_bound: Vec3 {
                x: scale * b1,
                y: scale * b3,
                z: scale * b2,
            },
            upper_bound: Vec3 {
                x: scale * b4,
                y: scale * b6,
                z: scale * b5,
            },
        }
    } else {
        Aabb {
            lower_bound: Vec3 {
                x: scale * b1,
                y: scale * b2,
                z: scale * b3,
            },
            upper_bound: Vec3 {
                x: scale * b4,
                y: scale * b5,
                z: scale * b6,
            },
        }
    };
    Some(aabb)
}

/// C `TreeBenchmark::CreateTree` — build the tree from a record file's text, insert
/// every record as a proxy (`categoryBits = 1`, `userData = i`), then `Generate`.
///
/// `file_index` selects the C `m_config` scale / axis convention. The record file text
/// is fetched by JS (the browser has no `fopen`).
#[wasm_bindgen]
pub fn tree_reset(file_index: u32, text: &str) {
    let file_index = (file_index as usize).min(FILE_CONFIG.len() - 1);
    let (scale, z_up) = FILE_CONFIG[file_index];

    let mut tree = DynamicTree::new(512);
    let mut aabbs: Vec<Aabb> = Vec::new();
    for line in text.lines() {
        if let Some(aabb) = parse_record(line, scale, z_up) {
            aabbs.push(aabb);
        }
    }

    let mut proxies: Vec<Proxy> = Vec::with_capacity(aabbs.len());
    for (i, aabb) in aabbs.iter().enumerate() {
        // C: b3DynamicTree_CreateProxy( &m_tree, aabb, 1, i ).
        tree.create_proxy(*aabb, 1, i as u64);
        proxies.push(Proxy {
            aabb: *aabb,
            query_time_stamp: 0,
            ray_time_stamp: 0,
        });
    }

    tree.validate();
    let area_ratio = tree.area_ratio();
    let (depths, height) = compute_depths(&tree);

    // Preserve the persistent RNG stream across File-change rebuilds (C behavior),
    // starting from RAND_SEED on the first build.
    let prev_seed = STATE.with(|cell| cell.borrow().as_ref().map(|s| s.seed));
    let mut state = TreeState {
        tree,
        proxies,
        file_index,
        seed: prev_seed.unwrap_or(RAND_SEED),
        rays: Vec::new(),
        overlap_queries: Vec::new(),
        closest_queries: Vec::new(),
        depths,
        height,
        area_ratio,
        time_stamp: 1,
        test_index: 0,
        have_closest: false,
        closest_point: VEC3_ZERO,
    };
    generate(&mut state);

    STATE.with(|cell| *cell.borrow_mut() = Some(state));
}

/// C "Top Down" button: full median-split rebuild, recompute area ratio + depths.
/// (Queries are *not* regenerated, matching C.)
#[wasm_bindgen]
pub fn tree_rebuild() {
    with_state(|state| {
        state.tree.rebuild(true);
        state.area_ratio = state.tree.area_ratio();
        let (depths, height) = compute_depths(&state.tree);
        state.depths = depths;
        state.height = height;
    });
}

/// Readout tuple `[leaves, height, areaRatio]` (C `DrawControls` line 1).
#[wasm_bindgen]
pub fn tree_stats() -> Vec<f32> {
    with_state(|state| {
        vec![
            state.tree.proxy_count() as f32,
            state.height as f32,
            state.area_ratio,
        ]
    })
}

/// Number of query tests (always [`TEST_COUNT`]) and the max Level for the slider.
#[wasm_bindgen]
pub fn tree_dims() -> Vec<f32> {
    with_state(|state| vec![TEST_COUNT as f32, state.height as f32])
}

/// Allocated leaf boxes for the default (Level = -1) view. Per leaf:
/// `[lx,ly,lz, ux,uy,uz, hit]` where `hit = 1` when the current test's query or ray
/// touched this proxy this timestamp (C draws those gray, the rest light blue). The
/// JS page applies the camera-distance cull.
#[wasm_bindgen]
pub fn tree_leaf_boxes() -> Vec<f32> {
    with_state(|state| {
        let nodes = state.tree.node_views();
        let mut out = Vec::new();
        for node in &nodes {
            if !(node.is_allocated && node.is_leaf) {
                continue;
            }
            let proxy = &state.proxies[node.user_data as usize];
            let hit = proxy.query_time_stamp == state.time_stamp
                || proxy.ray_time_stamp == state.time_stamp;
            out.extend_from_slice(&[
                node.aabb.lower_bound.x,
                node.aabb.lower_bound.y,
                node.aabb.lower_bound.z,
                node.aabb.upper_bound.x,
                node.aabb.upper_bound.y,
                node.aabb.upper_bound.z,
                if hit { 1.0 } else { 0.0 },
            ]);
        }
        out
    })
}

/// Allocated node boxes at BFS depth `level` (C Level-slider view). Per node:
/// `[lx,ly,lz, ux,uy,uz]`. The JS page colors by `level` and applies the C
/// distance cull (only for `level >= 10`).
#[wasm_bindgen]
pub fn tree_level_boxes(level: i32) -> Vec<f32> {
    with_state(|state| {
        let nodes = state.tree.node_views();
        let mut out = Vec::new();
        for (i, node) in nodes.iter().enumerate() {
            if state.depths[i] != level || !node.is_allocated {
                continue;
            }
            out.extend_from_slice(&[
                node.aabb.lower_bound.x,
                node.aabb.lower_bound.y,
                node.aabb.lower_bound.z,
                node.aabb.upper_bound.x,
                node.aabb.upper_bound.y,
                node.aabb.upper_bound.z,
            ]);
        }
        out
    })
}

/// C `TreeBenchmark::Step` — advance the timestamp and run the selected single-test
/// queries (ray / overlap / closest) for `test_index`, updating proxy timestamps and
/// the closest-point marker. Returns `[haveClosest, cx, cy, cz]`.
#[wasm_bindgen]
pub fn tree_step_query(
    do_ray: bool,
    do_overlap: bool,
    do_closest: bool,
    test_index: u32,
) -> Vec<f32> {
    with_state(|state| {
        state.test_index = (test_index as usize).min(TEST_COUNT - 1);
        state.time_stamp += 1;
        let ts = state.time_stamp;
        let ti = state.test_index;

        if do_ray {
            let ray = state.rays[ti];
            let input = RayCastInput {
                origin: ray.origin,
                translation: ray.translation,
                max_fraction: 1.0,
            };
            state.tree.ray_cast(
                &input,
                DEFAULT_MASK_BITS,
                false,
                |_input, _proxy_id, user_data| {
                    state.proxies[user_data as usize].ray_time_stamp = ts;
                    1.0
                },
            );
        }

        if do_overlap {
            let aabb = state.overlap_queries[ti];
            state
                .tree
                .query(aabb, DEFAULT_MASK_BITS, false, |_proxy_id, user_data| {
                    state.proxies[user_data as usize].query_time_stamp = ts;
                    true
                });
        }

        state.have_closest = false;
        if do_closest {
            run_closest(state, ti);
        }

        vec![
            if state.have_closest { 1.0 } else { 0.0 },
            state.closest_point.x,
            state.closest_point.y,
            state.closest_point.z,
        ]
    })
}

/// C `ClosetPointCallback` driver for one test index (shared by Step and Profile).
/// The C callback reads the *query point* from `m_closestPointQueries[m_testIndex]`
/// (not the loop index — a C quirk faithfully preserved).
fn run_closest(state: &mut TreeState, query_index: usize) {
    let point = state.closest_queries[query_index].center;
    let radius = state.closest_queries[query_index].radius;
    let mut min_dist_sq = radius * radius;
    state.closest_point = point;
    state.have_closest = false;

    let ts = state.time_stamp;
    let query_point = state.closest_queries[state.test_index].center;
    let proxies = &mut state.proxies;
    let mut closest_point = state.closest_point;
    let mut have_closest = false;

    state.tree.query_closest(
        point,
        DEFAULT_MASK_BITS,
        false,
        |min_distance_squared, _proxy_id, user_data| {
            proxies[user_data as usize].query_time_stamp = ts;
            let closest = closest_point_to_aabb(query_point, proxies[user_data as usize].aabb);
            let d2 = distance_squared(query_point, closest);
            if d2 < min_distance_squared {
                closest_point = closest;
                have_closest = true;
            }
            d2
        },
        &mut min_dist_sq,
    );

    state.closest_point = closest_point;
    state.have_closest = have_closest;
}

/// Profile all 1024 ray casts (C `Profile` ray loop). JS times this call.
#[wasm_bindgen]
pub fn tree_profile_ray() {
    with_state(|state| {
        let ts = state.time_stamp;
        for i in 0..TEST_COUNT {
            let ray = state.rays[i];
            let input = RayCastInput {
                origin: ray.origin,
                translation: ray.translation,
                max_fraction: 1.0,
            };
            state.tree.ray_cast(
                &input,
                DEFAULT_MASK_BITS,
                false,
                |_input, _proxy_id, user_data| {
                    state.proxies[user_data as usize].ray_time_stamp = ts;
                    1.0
                },
            );
        }
    });
}

/// Profile all 1024 overlap queries (C `Profile` overlap loop). JS times this call.
#[wasm_bindgen]
pub fn tree_profile_overlap() {
    with_state(|state| {
        let ts = state.time_stamp;
        for i in 0..TEST_COUNT {
            let aabb = state.overlap_queries[i];
            state
                .tree
                .query(aabb, DEFAULT_MASK_BITS, false, |_proxy_id, user_data| {
                    state.proxies[user_data as usize].query_time_stamp = ts;
                    true
                });
        }
    });
}

/// Profile all 1024 closest-point queries (C `Profile` closest loop). JS times this.
#[wasm_bindgen]
pub fn tree_profile_closest() {
    with_state(|state| {
        for i in 0..TEST_COUNT {
            run_closest(state, i);
        }
    });
}

/// Current test ray as `[ox,oy,oz, ex,ey,ez]` (draw when "Ray Cast" is on).
#[wasm_bindgen]
pub fn tree_test_ray(index: u32) -> Vec<f32> {
    with_state(|state| {
        let ray = state.rays[(index as usize).min(TEST_COUNT - 1)];
        let end = add(ray.origin, ray.translation);
        vec![
            ray.origin.x,
            ray.origin.y,
            ray.origin.z,
            end.x,
            end.y,
            end.z,
        ]
    })
}

/// Current test overlap AABB as `[lx,ly,lz, ux,uy,uz]` (draw when "Overlap" is on).
#[wasm_bindgen]
pub fn tree_test_overlap(index: u32) -> Vec<f32> {
    with_state(|state| {
        let a = state.overlap_queries[(index as usize).min(TEST_COUNT - 1)];
        vec![
            a.lower_bound.x,
            a.lower_bound.y,
            a.lower_bound.z,
            a.upper_bound.x,
            a.upper_bound.y,
            a.upper_bound.z,
        ]
    })
}

/// Current test closest-point sphere as `[cx,cy,cz, r]` (draw when "Closet Point" is on).
#[wasm_bindgen]
pub fn tree_test_sphere(index: u32) -> Vec<f32> {
    with_state(|state| {
        let s = state.closest_queries[(index as usize).min(TEST_COUNT - 1)];
        vec![s.center.x, s.center.y, s.center.z, s.radius]
    })
}

/// The current File index (so JS can keep its combo in sync after a reset).
#[wasm_bindgen]
pub fn tree_file_index() -> u32 {
    with_state(|state| state.file_index as u32)
}
