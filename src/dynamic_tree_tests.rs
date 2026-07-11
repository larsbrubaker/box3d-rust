// Adapted from box2d-rust/src/dynamic_tree_tests.rs (Box2D test_dynamic_tree.c)
// for Box3D Vec3 AABBs. Box3D has no test_dynamic_tree.c.
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::dynamic_tree::{DynamicTree, DEFAULT_MASK_BITS};
use crate::geometry::RayCastInput;
use crate::math_functions::{add, distance_squared, sub, Aabb, Vec3};

fn v(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

fn aabb(lx: f32, ly: f32, lz: f32, ux: f32, uy: f32, uz: f32) -> Aabb {
    Aabb {
        lower_bound: v(lx, ly, lz),
        upper_bound: v(ux, uy, uz),
    }
}

#[test]
fn tree_create_destroy() {
    let a = aabb(-1.0, -1.0, -1.0, 2.0, 2.0, 2.0);

    let mut tree = DynamicTree::new(16);
    tree.create_proxy(a, 1, 0);

    assert!(tree.node_count() > 0);
    assert!(tree.proxy_count() == 1);

    tree.destroy();

    assert!(tree.node_count() == 0);
    assert!(tree.proxy_count() == 0);
}

/// Ray-cast callback records the proxy id and returns 0 (terminate).
fn ray_cast_hit(tree: &DynamicTree, p1: Vec3, p2: Vec3) -> i32 {
    let input = RayCastInput {
        origin: p1,
        translation: sub(p2, p1),
        max_fraction: 1.0,
    };

    let mut proxy_hit = -1;
    tree.ray_cast(&input, 1, false, |_input, proxy_id, _user_data| {
        proxy_hit = proxy_id;
        0.0
    });
    proxy_hit
}

#[test]
fn tree_ray_cast() {
    // Test AABB centered at origin with bounds [-1, -1, -1] to [1, 1, 1]
    let a = aabb(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
    let mut tree = DynamicTree::new(16);
    let proxy_id = tree.create_proxy(a, 1, 0);

    // Test 1: Ray hits AABB from -x
    assert_eq!(
        ray_cast_hit(&tree, v(-3.0, 0.0, 0.0), v(3.0, 0.0, 0.0)),
        proxy_id
    );

    // Test 2: Ray hits AABB from +x
    assert_eq!(
        ray_cast_hit(&tree, v(3.0, 0.0, 0.0), v(-3.0, 0.0, 0.0)),
        proxy_id
    );

    // Test 3: Ray hits AABB from -y
    assert_eq!(
        ray_cast_hit(&tree, v(0.0, -3.0, 0.0), v(0.0, 3.0, 0.0)),
        proxy_id
    );

    // Test 4: Ray hits AABB from +y
    assert_eq!(
        ray_cast_hit(&tree, v(0.0, 3.0, 0.0), v(0.0, -3.0, 0.0)),
        proxy_id
    );

    // Test 5: Ray hits AABB from -z
    assert_eq!(
        ray_cast_hit(&tree, v(0.0, 0.0, -3.0), v(0.0, 0.0, 3.0)),
        proxy_id
    );

    // Test 6: Ray misses AABB (parallel to x-axis, outside in y)
    assert_eq!(ray_cast_hit(&tree, v(-3.0, 2.0, 0.0), v(3.0, 2.0, 0.0)), -1);

    // Test 7: Ray misses AABB (parallel to y-axis, outside in x)
    assert_eq!(ray_cast_hit(&tree, v(2.0, -3.0, 0.0), v(2.0, 3.0, 0.0)), -1);

    // Test 8: Ray misses AABB (parallel to z-axis, outside in x)
    assert_eq!(ray_cast_hit(&tree, v(2.0, 0.0, -3.0), v(2.0, 0.0, 3.0)), -1);

    // Test 9: Ray starts inside AABB
    assert_eq!(
        ray_cast_hit(&tree, v(0.0, 0.0, 0.0), v(2.0, 0.0, 0.0)),
        proxy_id
    );

    // Test 10: Ray hits corner of AABB (space diagonal)
    assert_eq!(
        ray_cast_hit(&tree, v(-2.0, -2.0, -2.0), v(2.0, 2.0, 2.0)),
        proxy_id
    );

    // Test 11: Ray parallel to AABB face but outside
    assert_eq!(ray_cast_hit(&tree, v(-2.0, 1.5, 0.0), v(2.0, 1.5, 0.0)), -1);

    // Test 12: Ray parallel to AABB face and exactly on boundary
    assert_eq!(
        ray_cast_hit(&tree, v(-2.0, 1.0, 0.0), v(2.0, 1.0, 0.0)),
        proxy_id
    );

    // Test 13: Very short ray that doesn't reach AABB
    assert_eq!(
        ray_cast_hit(&tree, v(-3.0, 0.0, 0.0), v(-2.5, 0.0, 0.0)),
        -1
    );

    // Test 14: Zero-length ray (degenerate case)
    assert_eq!(
        ray_cast_hit(&tree, v(0.0, 0.0, 0.0), v(0.0, 0.0, 0.0)),
        proxy_id
    );

    // Test 15: Ray hits AABB at exact boundary condition (t = 1.0)
    assert_eq!(
        ray_cast_hit(&tree, v(-2.0, 0.0, 0.0), v(-1.0, 0.0, 0.0)),
        proxy_id
    );

    tree.destroy();
}

#[test]
fn tree_create_move_destroy_proxy() {
    let mut tree = DynamicTree::new(16);

    let a = aabb(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    let id = tree.create_proxy(a, 0x1, 100);
    assert_eq!(tree.proxy_count(), 1);
    assert_eq!(tree.user_data(id), 100);

    let moved = aabb(10.0, 10.0, 10.0, 11.0, 11.0, 11.0);
    tree.move_proxy(id, moved);
    let got = tree.aabb(id);
    assert_eq!(got.lower_bound, moved.lower_bound);
    assert_eq!(got.upper_bound, moved.upper_bound);

    tree.destroy_proxy(id);
    assert_eq!(tree.proxy_count(), 0);

    tree.destroy();
}

#[test]
fn tree_multiple_proxies() {
    let mut tree = DynamicTree::new(16);

    let a1 = aabb(-5.0, -1.0, -1.0, -3.0, 1.0, 1.0);
    let a2 = aabb(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
    let a3 = aabb(3.0, -1.0, -1.0, 5.0, 1.0, 1.0);

    let id1 = tree.create_proxy(a1, 0x1, 42);
    let id2 = tree.create_proxy(a2, 0x2, 43);
    let id3 = tree.create_proxy(a3, 0x4, 44);

    assert!(tree.proxy_count() == 3);

    assert!(tree.user_data(id1) == 42);
    assert!(tree.user_data(id2) == 43);
    assert!(tree.user_data(id3) == 44);

    assert!(tree.category_bits(id1) == 0x1);
    assert!(tree.category_bits(id2) == 0x2);
    assert!(tree.category_bits(id3) == 0x4);

    tree.destroy();
}

#[test]
fn tree_query() {
    let mut tree = DynamicTree::new(16);

    let a1 = aabb(-5.0, -1.0, -1.0, -3.0, 1.0, 1.0);
    let a2 = aabb(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
    let a3 = aabb(3.0, -1.0, -1.0, 5.0, 1.0, 1.0);

    let _id1 = tree.create_proxy(a1, 0xFF, 0);
    let id2 = tree.create_proxy(a2, 0xFF, 0);
    let _id3 = tree.create_proxy(a3, 0xFF, 0);

    let query_a = aabb(-2.0, -2.0, -2.0, 2.0, 2.0, 2.0);

    let mut found_flags = [0i32; 32];
    let stats = tree.query(query_a, 0xFFFFFFFF, false, |proxy_id, _user_data| {
        found_flags[proxy_id as usize] = 1;
        true // continue the query
    });

    // We expect at least the middle proxy to be visited.
    assert!(found_flags[id2 as usize] == 1);
    assert!(stats.leaf_visits >= 1);

    tree.destroy();
}

#[test]
fn tree_query_require_all_bits() {
    let mut tree = DynamicTree::new(16);

    let a = aabb(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
    let id = tree.create_proxy(a, 0b011, 0);

    let query_a = aabb(-2.0, -2.0, -2.0, 2.0, 2.0, 2.0);

    // OR filter: any overlapping bit matches
    let mut found = false;
    tree.query(query_a, 0b001, false, |proxy_id, _| {
        found = proxy_id == id;
        true
    });
    assert!(found);

    // AND filter: all mask bits must be present — 0b111 is not subset of 0b011
    found = false;
    tree.query(query_a, 0b111, true, |proxy_id, _| {
        found = proxy_id == id;
        true
    });
    assert!(!found);

    // AND filter: 0b011 is fully present
    found = false;
    tree.query(query_a, 0b011, true, |proxy_id, _| {
        found = proxy_id == id;
        true
    });
    assert!(found);

    tree.destroy();
}

#[test]
fn tree_query_closest() {
    let mut tree = DynamicTree::new(16);

    let a1 = aabb(-5.0, -1.0, -1.0, -3.0, 1.0, 1.0);
    let a2 = aabb(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
    let a3 = aabb(3.0, -1.0, -1.0, 5.0, 1.0, 1.0);

    let id1 = tree.create_proxy(a1, DEFAULT_MASK_BITS, 0);
    let id2 = tree.create_proxy(a2, DEFAULT_MASK_BITS, 0);
    let id3 = tree.create_proxy(a3, DEFAULT_MASK_BITS, 0);

    let point = v(0.0, 0.0, 0.0);
    let centers = [
        crate::math_functions::aabb_center(tree.aabb(id1)),
        crate::math_functions::aabb_center(tree.aabb(id2)),
        crate::math_functions::aabb_center(tree.aabb(id3)),
    ];
    let ids = [id1, id2, id3];

    let mut min_sqr = f32::MAX;
    let mut closest_id = -1;
    let mut closest_dd = f32::MAX;

    tree.query_closest(
        point,
        DEFAULT_MASK_BITS,
        false,
        |_min_sqr, proxy_id, _user_data| {
            let center = ids
                .iter()
                .position(|&id| id == proxy_id)
                .map(|i| centers[i])
                .unwrap_or(point);
            let dd = distance_squared(point, center);
            if dd < closest_dd {
                closest_dd = dd;
                closest_id = proxy_id;
            }
            dd
        },
        &mut min_sqr,
    );

    assert_eq!(closest_id, id2);
    assert!(min_sqr < distance_squared(point, centers[0]));
    assert!(min_sqr < distance_squared(point, centers[2]));

    tree.destroy();
}

#[test]
fn tree_move_and_enlarge() {
    let mut tree = DynamicTree::new(16);

    let a = aabb(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    let id = tree.create_proxy(a, 0x1, 100);

    // Move proxy to a new place
    let moved = aabb(10.0, 10.0, 10.0, 11.0, 11.0, 11.0);
    tree.move_proxy(id, moved);

    let got = tree.aabb(id);
    assert!(got.lower_bound.x == moved.lower_bound.x);
    assert!(got.lower_bound.y == moved.lower_bound.y);
    assert!(got.lower_bound.z == moved.lower_bound.z);
    assert!(got.upper_bound.x == moved.upper_bound.x);
    assert!(got.upper_bound.y == moved.upper_bound.y);
    assert!(got.upper_bound.z == moved.upper_bound.z);

    // Now enlarge the proxy
    let enlarge = aabb(9.5, 9.5, 9.5, 11.5, 11.5, 11.5);
    tree.enlarge_proxy(id, enlarge);

    let got2 = tree.aabb(id);
    assert!(got2.lower_bound.x <= enlarge.lower_bound.x + 1e-6);
    assert!(got2.upper_bound.x >= enlarge.upper_bound.x - 1e-6);
    assert!(got2.lower_bound.z <= enlarge.lower_bound.z + 1e-6);
    assert!(got2.upper_bound.z >= enlarge.upper_bound.z - 1e-6);

    tree.destroy();
}

#[test]
fn tree_rebuild_and_validate() {
    let mut tree = DynamicTree::new(16);

    // Create a number of proxies to make rebuild meaningful
    for i in 0..12 {
        let x = i as f32 * 2.0;
        let a = aabb(x - 0.5, -0.5, -0.5, x + 0.5, 0.5, 0.5);
        tree.create_proxy(a, 0xFF, i as u64);
    }

    let sorted = tree.rebuild(true);

    assert!(sorted >= 0);
    assert!(tree.byte_count() > 0);
    assert!(tree.height() > 0);

    tree.validate();
    tree.validate_no_enlarged();

    tree.destroy();
}

#[test]
fn tree_row_height() {
    let mut tree = DynamicTree::new(16);

    let column_count = 200;
    for i in 0..column_count {
        let x = 1.0 * i as f32;
        let a = aabb(x, 0.0, 0.0, x + 1.0, 1.0, 1.0);
        tree.create_proxy(a, 1, i as u64);
    }

    let min_height = (column_count as f32).log2();

    assert!((tree.height() as f32) < 2.0 * min_height);

    tree.destroy();
}

#[test]
fn tree_grid_height() {
    let mut tree = DynamicTree::new(16);

    let column_count = 20;
    let row_count = 20;
    for i in 0..column_count {
        let x = 1.0 * i as f32;
        for j in 0..row_count {
            let y = 1.0 * j as f32;
            let a = aabb(x, y, 0.0, x + 1.0, y + 1.0, 1.0);
            tree.create_proxy(a, 1, i as u64);
        }
    }

    let min_height = ((row_count * column_count) as f32).log2();

    assert!((tree.height() as f32) < 2.0 * min_height);

    tree.destroy();
}

const GRID_COUNT: usize = 20;

#[test]
fn tree_grid_movement() {
    let mut tree = DynamicTree::new(16);

    let mut proxy_ids = [0i32; GRID_COUNT * GRID_COUNT];
    let mut index = 0usize;
    for i in 0..GRID_COUNT {
        let x = 1.0 * i as f32;
        for j in 0..GRID_COUNT {
            let y = 1.0 * j as f32;
            let a = aabb(x, y, 0.0, x + 1.0, y + 1.0, 1.0);
            proxy_ids[index] = tree.create_proxy(a, 1, i as u64);
            index += 1;
        }
    }

    assert!(index == GRID_COUNT * GRID_COUNT);

    let min_height = ((GRID_COUNT * GRID_COUNT) as f32).log2();

    let height1 = tree.height();
    assert!((height1 as f32) < 2.0 * min_height);

    let offset = v(10.0, 20.0, 5.0);
    index = 0;
    for _i in 0..GRID_COUNT {
        for _j in 0..GRID_COUNT {
            let mut a = tree.aabb(proxy_ids[index]);
            a.lower_bound = add(a.lower_bound, offset);
            a.upper_bound = add(a.upper_bound, offset);
            tree.move_proxy(proxy_ids[index], a);
            index += 1;
        }
    }

    let height2 = tree.height();
    assert!((height2 as f32) < 3.0 * min_height);

    tree.rebuild(true);

    let height3 = tree.height();
    assert!((height3 as f32) < 2.0 * min_height);

    tree.destroy();
}

#[test]
fn tree_set_category_bits() {
    let mut tree = DynamicTree::new(16);

    let a1 = aabb(-2.0, -1.0, -1.0, -1.0, 1.0, 1.0);
    let a2 = aabb(1.0, -1.0, -1.0, 2.0, 1.0, 1.0);
    let id1 = tree.create_proxy(a1, 0x1, 0);
    let id2 = tree.create_proxy(a2, 0x2, 0);

    tree.set_category_bits(id1, 0x4);
    assert_eq!(tree.category_bits(id1), 0x4);
    assert_eq!(tree.category_bits(id2), 0x2);

    // Root should OR the children
    let root = tree.root_bounds();
    assert!(root.lower_bound.x <= -2.0);
    assert!(root.upper_bound.x >= 2.0);

    tree.destroy();
}
