// Port of box3d-cpp-reference/test/test_dynamic_tree.c.
//
// The C tests share a file-scope LCG seed that each test resets; the port threads an
// explicit generator through each test instead, which yields the same stream.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use crate::aabb::ray_cast_aabb;
use crate::dynamic_tree::{BoxCastInput, DynamicTree, DEFAULT_CATEGORY_BITS, DEFAULT_MASK_BITS};
use crate::geometry::RayCastInput;
use crate::math_functions::{
    aabb_contains, aabb_inflate, aabb_overlaps, add, clamp, dot, max, min, mul_add, sub, Aabb, Vec3,
};

const PROXY_COUNT: usize = 300;

/// The C `s_seed` LCG. (NextRandom / RandomFloat / RandomBox)
struct Lcg {
    seed: u32,
}

impl Lcg {
    fn new(seed: u32) -> Lcg {
        Lcg { seed }
    }

    fn next_random(&mut self) -> u32 {
        self.seed = 1664525u32
            .wrapping_mul(self.seed)
            .wrapping_add(1013904223u32);
        self.seed
    }

    fn random_float(&mut self, lo: f32, hi: f32) -> f32 {
        let t = (self.next_random() >> 8) as f32 / (1u32 << 24) as f32;
        lo + t * (hi - lo)
    }

    fn random_vec3(&mut self, lo: f32, hi: f32) -> Vec3 {
        let x = self.random_float(lo, hi);
        let y = self.random_float(lo, hi);
        let z = self.random_float(lo, hi);
        Vec3 { x, y, z }
    }

    fn random_box(&mut self, extent: f32, size: f32) -> Aabb {
        let center = self.random_vec3(-extent, extent);
        let half = self.random_vec3(0.1, size);
        Aabb {
            lower_bound: sub(center, half),
            upper_bound: add(center, half),
        }
    }
}

/// (struct HitSet) — keyed on user data, the index of the box each proxy came from.
/// Proxy ids recycle, box indices do not.
#[derive(Default)]
struct HitSet {
    ids: Vec<i32>,
    bad: bool,
}

impl HitSet {
    /// (CollectCallback) — a callback cannot fail the test directly, so bad news rides back
    /// on the set.
    fn collect(&mut self, user_data: u64) -> bool {
        if self.ids.len() >= PROXY_COUNT
            || (user_data as i32) < 0
            || (user_data as i32) >= PROXY_COUNT as i32
        {
            self.bad = true;
            return false;
        }

        self.ids.push(user_data as i32);
        true
    }
}

/// (SameSet)
fn assert_same_set(a: &mut HitSet, b: &mut HitSet) {
    assert!(!a.bad);
    assert!(!b.bad);
    assert_eq!(a.ids.len(), b.ids.len());
    a.ids.sort_unstable();
    b.ids.sort_unstable();
    assert_eq!(a.ids, b.ids);
}

fn brute_force_hits(boxes: &[Aabb], query_box: Aabb) -> HitSet {
    let mut hits = HitSet::default();
    for (i, b) in boxes.iter().enumerate() {
        if aabb_overlaps(*b, query_box) {
            hits.ids.push(i as i32);
        }
    }
    hits
}

/// The tree must agree with a linear scan over the stored boxes. This is the gate on the
/// SIMD box helpers, which have three separate lane layouts. (TreeQueryBruteForce)
#[test]
fn tree_query_brute_force() {
    let mut rng = Lcg::new(12345);

    let mut tree = DynamicTree::new(16);

    let mut boxes = Vec::with_capacity(PROXY_COUNT);
    for i in 0..PROXY_COUNT {
        boxes.push(rng.random_box(6.0, 1.5));
        let proxy_id = tree.create_proxy(boxes[i], DEFAULT_CATEGORY_BITS, i as u64);
        assert_eq!(proxy_id, i as i32);
    }

    tree.validate();
    assert_eq!(tree.proxy_count(), PROXY_COUNT as i32);

    let mut total_hits = 0;
    for _round in 0..200 {
        let query_box = rng.random_box(7.0, 3.0);

        let mut tree_hits = HitSet::default();
        tree.query(
            query_box,
            DEFAULT_MASK_BITS,
            false,
            |_proxy_id, user_data| tree_hits.collect(user_data),
        );

        let mut brute_hits = brute_force_hits(&boxes, query_box);

        assert_same_set(&mut tree_hits, &mut brute_hits);
        total_hits += tree_hits.ids.len();
    }

    // Guard against the comparison above agreeing on nothing
    assert!(total_hits > 1000);

    tree.destroy();
}

/// Moving and enlarging every proxy exercises remove, insert, the ordered flag, the sweep
/// refit and the partial rebuild, then the query has to still agree with the scan.
/// (TreeMoveRefitRebuild)
#[test]
fn tree_move_refit_rebuild() {
    let mut rng = Lcg::new(777);

    let mut tree = DynamicTree::new(16);

    let mut boxes = Vec::with_capacity(PROXY_COUNT);
    let mut proxies = Vec::with_capacity(PROXY_COUNT);
    for i in 0..PROXY_COUNT {
        boxes.push(rng.random_box(6.0, 1.0));
        proxies.push(tree.create_proxy(boxes[i], DEFAULT_CATEGORY_BITS, i as u64));
    }

    tree.rebuild(true);
    tree.validate();
    assert!(tree.dfs_ordered());

    let mut total_hits = 0;
    for _round in 0..8 {
        // Half the proxies move, half grow in place
        for i in 0..PROXY_COUNT {
            if (i & 1) == 0 {
                boxes[i] = rng.random_box(6.0, 1.0);
                tree.move_proxy(proxies[i], boxes[i]);
            } else {
                let grown = aabb_inflate(boxes[i], 0.35);
                boxes[i] = grown;
                tree.enlarge_proxy(proxies[i], grown);
            }
        }

        tree.refit();
        tree.validate();

        tree.rebuild(false);
        tree.validate();
        tree.validate_no_moved();
        assert!(tree.dfs_ordered());

        // Every stored box must still be inside its leaf and inside the root
        let root_bounds = tree.root_bounds();
        for i in 0..PROXY_COUNT {
            assert!(aabb_contains(tree.aabb(proxies[i]), boxes[i]));
            assert!(aabb_contains(root_bounds, boxes[i]));
            assert_eq!(tree.user_data(proxies[i]) as i32, i as i32);
        }

        let query_box = rng.random_box(7.0, 3.0);

        let mut tree_hits = HitSet::default();
        tree.query(
            query_box,
            DEFAULT_MASK_BITS,
            false,
            |_proxy_id, user_data| tree_hits.collect(user_data),
        );

        let mut brute_hits = brute_force_hits(&boxes, query_box);

        assert_same_set(&mut tree_hits, &mut brute_hits);
        total_hits += tree_hits.ids.len();
    }

    // Guard against the comparison above agreeing on nothing
    assert!(total_hits > 50);

    tree.destroy();
}

/// Destroying proxies in a scrambled order recycles sibling pairs through the free list and
/// leaves the array unordered, which is the path the fallback refit has to handle.
/// (TreeDestroyScrambled)
#[test]
fn tree_destroy_scrambled() {
    let mut rng = Lcg::new(4242);

    let mut tree = DynamicTree::new(16);

    let mut boxes = Vec::with_capacity(PROXY_COUNT);
    let mut proxies = Vec::with_capacity(PROXY_COUNT);
    let mut alive = vec![false; PROXY_COUNT];

    for i in 0..PROXY_COUNT {
        boxes.push(rng.random_box(10.0, 0.8));
        proxies.push(tree.create_proxy(boxes[i], DEFAULT_CATEGORY_BITS, i as u64));
        alive[i] = true;
    }

    for i in (0..PROXY_COUNT).step_by(3) {
        tree.destroy_proxy(proxies[i]);
        alive[i] = false;
    }

    tree.validate();

    // Re-create into the holes
    for i in (0..PROXY_COUNT).step_by(3) {
        boxes[i] = rng.random_box(10.0, 0.8);
        proxies[i] = tree.create_proxy(boxes[i], DEFAULT_CATEGORY_BITS, i as u64);
        alive[i] = true;
    }

    tree.validate();
    assert_eq!(tree.proxy_count(), PROXY_COUNT as i32);

    // The user data has to have followed each proxy through the churn
    for i in 0..PROXY_COUNT {
        assert!(alive[i]);
        assert_eq!(tree.user_data(proxies[i]) as i32, i as i32);
        assert!(aabb_contains(tree.aabb(proxies[i]), boxes[i]));
    }

    let query_box = Aabb {
        lower_bound: Vec3::new(-10.0, -10.0, -10.0),
        upper_bound: Vec3::new(10.0, 10.0, 10.0),
    };
    let mut tree_hits = HitSet::default();
    tree.query(
        query_box,
        DEFAULT_MASK_BITS,
        false,
        |_proxy_id, user_data| tree_hits.collect(user_data),
    );

    let mut brute_hits = brute_force_hits(&boxes, query_box);

    assert_same_set(&mut tree_hits, &mut brute_hits);
    assert_eq!(tree_hits.ids.len(), PROXY_COUNT);

    // Empty the tree
    for &proxy_id in &proxies {
        tree.destroy_proxy(proxy_id);
    }

    tree.validate();
    assert_eq!(tree.proxy_count(), 0);
    assert_eq!(tree.height(), 0);

    // Queries on an empty tree must be silent
    let mut empty_hits = HitSet::default();
    tree.query(
        query_box,
        DEFAULT_MASK_BITS,
        false,
        |_proxy_id, user_data| empty_hits.collect(user_data),
    );
    assert!(empty_hits.ids.is_empty());

    tree.destroy();
}

/// A one proxy tree is a leaf at the root with an empty node beside it, which every
/// traversal has to survive. (TreeLeafRoot)
#[test]
fn tree_leaf_root() {
    let mut tree = DynamicTree::new(16);

    let box_ = Aabb {
        lower_bound: Vec3::new(-1.0, -1.0, -1.0),
        upper_bound: Vec3::new(1.0, 1.0, 1.0),
    };
    let proxy_id = tree.create_proxy(box_, DEFAULT_CATEGORY_BITS, 7);

    tree.validate();
    assert_eq!(tree.proxy_count(), 1);
    assert_eq!(tree.height(), 0);
    assert_eq!(tree.area_ratio(), 0.0);
    assert_eq!(tree.user_data(proxy_id), 7);

    // (LeafRootHit / LeafRootCallback)
    let mut count = 0;
    let mut hit_proxy_id = -1;
    let mut hit_user_data = 0u64;
    let hit_box = Aabb {
        lower_bound: Vec3::new(0.5, 0.5, 0.5),
        upper_bound: Vec3::new(2.0, 2.0, 2.0),
    };
    tree.query(hit_box, DEFAULT_MASK_BITS, false, |id, user_data| {
        count += 1;
        hit_proxy_id = id;
        hit_user_data = user_data;
        true
    });
    assert_eq!(count, 1);
    assert_eq!(hit_proxy_id, proxy_id);
    assert_eq!(hit_user_data, 7);

    count = 0;
    let miss_box = Aabb {
        lower_bound: Vec3::new(4.0, 4.0, 4.0),
        upper_bound: Vec3::new(5.0, 5.0, 5.0),
    };
    tree.query(miss_box, DEFAULT_MASK_BITS, false, |_id, _user_data| {
        count += 1;
        true
    });
    assert_eq!(count, 0);

    // The rebuild has one build leaf and the refit has nothing above it
    tree.rebuild(true);
    tree.validate();
    tree.refit();
    tree.validate();
    assert_eq!(tree.proxy_count(), 1);

    let grown = aabb_inflate(box_, 1.0);
    tree.enlarge_proxy(proxy_id, grown);
    assert!(aabb_contains(tree.aabb(proxy_id), grown));

    tree.destroy();
}

/// Category bits live on the proxy now, so the filter has to bite at the leaf.
/// (TreeCategoryFilter)
#[test]
fn tree_category_filter() {
    let mut rng = Lcg::new(99);

    let mut tree = DynamicTree::new(16);

    let mut boxes = Vec::with_capacity(PROXY_COUNT);
    let mut bits = Vec::with_capacity(PROXY_COUNT);
    let mut proxies = Vec::with_capacity(PROXY_COUNT);

    for i in 0..PROXY_COUNT {
        boxes.push(rng.random_box(8.0, 0.6));
        bits.push(1u64 << (i % 3));
        proxies.push(tree.create_proxy(boxes[i], bits[i], i as u64));
        assert_eq!(tree.category_bits(proxies[i]), bits[i]);
    }

    let query_box = Aabb {
        lower_bound: Vec3::new(-8.0, -8.0, -8.0),
        upper_bound: Vec3::new(8.0, 8.0, 8.0),
    };

    // A single bit mask makes requireAll equivalent to the any-bit test, so the last pass is
    // the one that proves requireAll accepts rather than just rejects
    let masks: [u64; 3] = [0x1, 0x3, 0x2];
    let require_alls = [false, false, true];

    for pass in 0..3 {
        let mask = masks[pass];
        let require_all = require_alls[pass];

        let mut tree_hits = HitSet::default();
        tree.query(query_box, mask, require_all, |_proxy_id, user_data| {
            tree_hits.collect(user_data)
        });

        let mut brute_hits = HitSet::default();
        for i in 0..PROXY_COUNT {
            let accept = if require_all {
                (bits[i] & mask) == mask
            } else {
                (bits[i] & mask) != 0
            };
            if accept && aabb_overlaps(boxes[i], query_box) {
                brute_hits.ids.push(i as i32);
            }
        }

        assert_same_set(&mut tree_hits, &mut brute_hits);
        assert!(tree_hits.ids.len() > PROXY_COUNT / 4);
    }

    // requireAll with two bits can never match a single bit proxy
    let mut no_hits = HitSet::default();
    tree.query(query_box, 0x5, true, |_proxy_id, user_data| {
        no_hits.collect(user_data)
    });
    assert!(no_hits.ids.is_empty());

    // Changing the bits has to change the filter without touching the structure
    tree.set_category_bits(proxies[0], 0x8);
    assert_eq!(tree.category_bits(proxies[0]), 0x8);
    tree.validate();

    let mut one_hit = HitSet::default();
    tree.query(query_box, 0x8, false, |_proxy_id, user_data| {
        one_hit.collect(user_data)
    });
    assert_eq!(one_hit.ids.len(), 1);
    assert_eq!(one_hit.ids[0], 0);

    tree.destroy();
}

/// Returning -1 skips the proxy without shrinking the fraction, so the traversal keeps
/// going and reports every proxy whose node box the segment crosses.
/// (RayCastCollectCallback / BoxCastCollectCallback)
fn cast_collect(set: &mut HitSet, user_data: u64) -> f32 {
    if set.ids.len() >= PROXY_COUNT
        || (user_data as i32) < 0
        || (user_data as i32) >= PROXY_COUNT as i32
    {
        set.bad = true;
        return 0.0;
    }

    set.ids.push(user_data as i32);
    -1.0
}

/// The casts pop a pair, test both nodes and order the survivors. A ray that skips every
/// hit must still reach every proxy its segment crosses. (TreeCastBruteForce)
#[test]
fn tree_cast_brute_force() {
    let mut rng = Lcg::new(31337);

    let mut tree = DynamicTree::new(16);

    let mut boxes = Vec::with_capacity(PROXY_COUNT);
    for i in 0..PROXY_COUNT {
        boxes.push(rng.random_box(6.0, 1.0));
        tree.create_proxy(boxes[i], DEFAULT_CATEGORY_BITS, i as u64);
    }

    tree.rebuild(true);
    tree.validate();

    let mut total_crossings = 0;
    for _round in 0..100 {
        let origin = rng.random_vec3(-8.0, 8.0);
        let translation = rng.random_vec3(-16.0, 16.0);

        let ray_input = RayCastInput {
            origin,
            translation,
            max_fraction: 1.0,
        };

        let mut tree_hits = HitSet::default();
        tree.ray_cast(
            &ray_input,
            DEFAULT_MASK_BITS,
            false,
            |_input, _proxy_id, user_data| cast_collect(&mut tree_hits, user_data),
        );
        assert!(!tree_hits.bad);

        // The tree may report a superset because it tests the node box, not the shape.
        // Every proxy the segment actually crosses has to be in there.
        let p2 = mul_add(origin, 1.0, translation);
        for (i, b) in boxes.iter().enumerate() {
            let mut min_fraction = 0.0;
            let mut max_fraction = 1.0;
            if !ray_cast_aabb(*b, origin, p2, &mut min_fraction, &mut max_fraction) {
                continue;
            }

            assert!(tree_hits.ids.contains(&(i as i32)));
            total_crossings += 1;
        }

        // Every reported proxy has to at least overlap the segment box
        let segment_box = Aabb {
            lower_bound: min(origin, p2),
            upper_bound: max(origin, p2),
        };
        for &id in &tree_hits.ids {
            assert!(aabb_overlaps(boxes[id as usize], segment_box));
        }

        // A zero extent box cast has to reach the same proxies as the ray
        let cast_box = Aabb {
            lower_bound: origin,
            upper_bound: origin,
        };
        let box_input = BoxCastInput {
            box_: cast_box,
            translation,
            max_fraction: 1.0,
        };

        let mut box_hits = HitSet::default();
        tree.box_cast(
            &box_input,
            DEFAULT_MASK_BITS,
            false,
            |_input, _proxy_id, user_data| cast_collect(&mut box_hits, user_data),
        );
        assert!(!box_hits.bad);

        for id in &tree_hits.ids {
            assert!(box_hits.ids.contains(id));
        }
    }

    // Guard against the comparisons above agreeing on nothing
    assert!(total_crossings > 100);

    tree.destroy();
}

/// (DistanceToBoxSqr)
fn distance_to_box_sqr(point: Vec3, box_: Aabb) -> f32 {
    let r = sub(point, clamp(point, box_.lower_bound, box_.upper_bound));
    dot(r, r)
}

/// The closest query orders by distance and prunes on the running minimum, so seeding it
/// from the root pair has to be right or it prunes away a whole half of the tree.
/// (TreeQueryClosest)
#[test]
fn tree_query_closest() {
    let mut rng = Lcg::new(5150);

    let mut tree = DynamicTree::new(16);

    let mut boxes = Vec::with_capacity(PROXY_COUNT);
    for i in 0..PROXY_COUNT {
        boxes.push(rng.random_box(6.0, 0.9));
        let proxy_id = tree.create_proxy(boxes[i], DEFAULT_CATEGORY_BITS, i as u64);
        assert_eq!(proxy_id, i as i32);
    }

    for _round in 0..100 {
        let point = rng.random_vec3(-8.0, 8.0);

        let mut brute_best = f32::MAX;
        for b in &boxes {
            let d = distance_to_box_sqr(point, *b);
            if d < brute_best {
                brute_best = d;
            }
        }

        // The callback reports the exact box distance, so the pruned search has to land on
        // the same minimum as the scan. Same arithmetic both ways, so this is an exact
        // compare. (ClosestResult / ClosestCollectCallback)
        let mut bad = false;
        let mut min_distance_sqr = f32::MAX;
        tree.query_closest(
            point,
            DEFAULT_MASK_BITS,
            false,
            |distance_sqr_min, proxy_id, user_data| {
                if user_data as i32 != proxy_id {
                    bad = true;
                    return distance_sqr_min;
                }

                let d = distance_to_box_sqr(point, boxes[proxy_id as usize]);
                if d < distance_sqr_min {
                    d
                } else {
                    distance_sqr_min
                }
            },
            &mut min_distance_sqr,
        );

        assert!(!bad);
        assert_eq!(min_distance_sqr, brute_best);
    }

    tree.destroy();
}

fn temp_tree_path(name: &str) -> String {
    let mut path = std::env::temp_dir();
    path.push(format!("{}_{}", std::process::id(), name));
    path.to_string_lossy().into_owned()
}

/// A saved tree carries only its live node range. (TreeSaveLoadRoundtrip)
#[test]
fn tree_save_load_roundtrip() {
    let mut rng = Lcg::new(4242);

    let mut tree = DynamicTree::new(16);

    let mut proxies = Vec::with_capacity(PROXY_COUNT);
    for i in 0..PROXY_COUNT {
        let b = rng.random_box(6.0, 1.0);
        proxies.push(tree.create_proxy(b, DEFAULT_CATEGORY_BITS, i as u64));
    }

    // Destroy a few so the pair free list is populated and the node tail is stale
    for i in (0..PROXY_COUNT).step_by(37) {
        tree.destroy_proxy(proxies[i]);
    }

    tree.rebuild(true);
    tree.validate();

    let tree_path = temp_tree_path("test_dynamic_tree_roundtrip.dat");
    tree.save(&tree_path);

    let loaded = DynamicTree::load(&tree_path, 1.0);
    let _ = std::fs::remove_file(&tree_path);

    assert!(!loaded.nodes.is_empty());
    assert_eq!(loaded.node_end(), tree.node_end());
    assert_eq!(loaded.node_capacity(), tree.node_end());
    assert_eq!(loaded.proxy_count(), tree.proxy_count());
    let node_end = tree.node_end() as usize;
    assert_eq!(loaded.nodes[..node_end], tree.nodes[..node_end]);
    assert_eq!(loaded.parents[..node_end], tree.parents[..node_end]);
    loaded.validate();

    drop(loaded);
    tree.destroy();
}
