//! Port of the 2D hull tests from box3d-cpp-reference/test/test_hull.c
//! (Hull2DSquareTest, Hull2DCollinearTest, Hull2DWeldTest, Hull2DOrderTest,
//! Hull2DNearCollinearTest, Hull2DKeepCollinearTest, Hull2DExactMergeTest,
//! Hull2DIndexOrderTest, SimplifyHull2DTargetTest, SimplifyHull2DNoopTest). Split from
//! hull_tests.rs to keep files under the length limit.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::constants::linear_slop;
use crate::hull::{
    hull_2d, simplify_hull_2d, Point2D, HULL_2D_EXACT_MERGE, HULL_2D_INDEX_ORDER,
    HULL_2D_KEEP_COLLINEAR,
};
use crate::math_functions::{Vec2, PI};

fn v2(x: f32, y: f32) -> Vec2 {
    Vec2 { x, y }
}

fn fill_points_2d(pts: &mut [Point2D], input: &[Vec2], count: usize) {
    for i in 0..count {
        pts[i].p = input[i];
        pts[i].separation = 0.0;
        pts[i].original_index = i as i32;
    }
}

fn hull_2d_area(hull: &[Point2D], count: i32) -> f32 {
    let mut sum = 0.0f32;
    for i in 0..count as usize {
        let next = (i + 1) % count as usize;
        sum += hull[i].p.x * hull[next].p.y - hull[i].p.y * hull[next].p.x;
    }

    0.5 * sum
}

/// Interior points contribute no area, so they must not survive the hull. (Hull2DSquareTest)
#[test]
fn hull_2d_square_test() {
    let input = [
        v2(-1.0, -1.0),
        v2(1.0, -1.0),
        v2(1.0, 1.0),
        v2(-1.0, 1.0),
        v2(0.0, 0.0),
        v2(0.5, 0.25),
        v2(-0.3, 0.6),
        v2(0.1, -0.4),
    ];

    let mut pts = [Point2D::default(); 8];
    let mut hull = [Point2D::default(); 16];
    fill_points_2d(&mut pts, &input, 8);

    let count = hull_2d(&mut pts, 8, &mut hull, 0);
    assert_eq!(count, 4);

    // The reduction code assumes counter clockwise winding
    assert!(hull_2d_area(&hull, count) > 0.0);

    let mut on_hull = [false; 8];
    for point in hull.iter().take(count as usize) {
        assert!(0 <= point.original_index && point.original_index < 8);
        on_hull[point.original_index as usize] = true;
    }

    for flag in on_hull.iter().take(4) {
        assert!(*flag);
    }

    for flag in on_hull.iter().skip(4) {
        assert!(!*flag);
    }
}

/// (Hull2DCollinearTest)
#[test]
fn hull_2d_collinear_test() {
    let input = [
        v2(0.0, 0.0),
        v2(1.0, 0.0),
        v2(2.0, 0.0),
        v2(3.0, 0.0),
        v2(4.0, 0.0),
    ];

    let mut pts = [Point2D::default(); 5];
    let mut hull = [Point2D::default(); 10];
    fill_points_2d(&mut pts, &input, 5);

    assert_eq!(hull_2d(&mut pts, 5, &mut hull, 0), 2);
}

/// Coincident points weld and the lower input index wins, so a sort order change cannot
/// swap which of the two survives. (Hull2DWeldTest)
#[test]
fn hull_2d_weld_test() {
    let offset = 0.1 * linear_slop();

    let input = [
        v2(-1.0, -1.0),
        v2(1.0, -1.0),
        v2(1.0, 1.0),
        v2(-1.0, 1.0),
        v2(-1.0 + offset, -1.0 + offset),
    ];

    let mut pts = [Point2D::default(); 5];
    let mut hull = [Point2D::default(); 10];
    fill_points_2d(&mut pts, &input, 5);

    let count = hull_2d(&mut pts, 5, &mut hull, 0);
    assert_eq!(count, 4);

    for point in hull.iter().take(count as usize) {
        assert_ne!(point.original_index, 4);
    }
}

/// (SimplifyHull2DTargetTest)
#[test]
fn simplify_hull_2d_target_test() {
    let mut pts = [Point2D::default(); 20];
    let mut hull = [Point2D::default(); 40];

    for (i, point) in pts.iter_mut().enumerate() {
        let angle = 2.0 * PI * i as f32 / 20.0;
        point.p = v2(angle.cos(), angle.sin());
        point.separation = 0.0;
        point.original_index = i as i32;
    }

    let hull_count = hull_2d(&mut pts, 20, &mut hull, 0);
    assert_eq!(hull_count, 20);

    let count = simplify_hull_2d(&mut hull, hull_count, 6);
    assert_eq!(count, 6);
    assert!(hull_2d_area(&hull, count) > 0.0);
}

/// (SimplifyHull2DNoopTest)
#[test]
fn simplify_hull_2d_noop_test() {
    let input = [v2(-1.0, -1.0), v2(1.0, -1.0), v2(1.0, 1.0), v2(-1.0, 1.0)];

    let mut pts = [Point2D::default(); 4];
    let mut hull = [Point2D::default(); 8];
    fill_points_2d(&mut pts, &input, 4);

    let hull_count = hull_2d(&mut pts, 4, &mut hull, 0);
    assert_eq!(hull_count, 4);

    assert_eq!(simplify_hull_2d(&mut hull, hull_count, 8), 4);
    assert_eq!(simplify_hull_2d(&mut hull, hull_count, 4), 4);
}

/// The surviving point set must not depend on the order the points arrive in. The reduction
/// runs inside the solver, so an input order dependence would show up as a determinism bug.
/// (Hull2DOrderTest)
#[test]
fn hull_2d_order_test() {
    let input = [
        v2(-2.0, -1.0),
        v2(0.0, -1.5),
        v2(2.0, -1.0),
        v2(2.5, 0.5),
        v2(1.0, 2.0),
        v2(-1.0, 1.8),
        v2(-2.5, 0.4),
        v2(0.2, 0.1),
    ];

    let permutation = [5usize, 2, 7, 0, 4, 6, 1, 3];

    let mut pts = [Point2D::default(); 8];
    let mut hull = [Point2D::default(); 16];
    fill_points_2d(&mut pts, &input, 8);

    let count1 = hull_2d(&mut pts, 8, &mut hull, 0);
    assert_eq!(count1, 7);

    let mut on_hull1 = [false; 8];
    for point in hull.iter().take(count1 as usize) {
        on_hull1[point.original_index as usize] = true;
    }

    let mut shuffled = [Point2D::default(); 8];
    let mut shuffled_hull = [Point2D::default(); 16];
    for i in 0..8 {
        shuffled[i].p = input[permutation[i]];
        shuffled[i].separation = 0.0;
        shuffled[i].original_index = i as i32;
    }

    let count2 = hull_2d(&mut shuffled, 8, &mut shuffled_hull, 0);
    assert_eq!(count2, count1);

    let mut on_hull2 = [false; 8];
    for point in shuffled_hull.iter().take(count2 as usize) {
        on_hull2[permutation[point.original_index as usize]] = true;
    }

    for i in 0..8 {
        assert_eq!(on_hull1[i], on_hull2[i]);
    }
}

/// Float rounding can call the middle of three nearly collinear points a left turn in both
/// chains. Listing it twice gave the mesh manifold a duplicate point. (Hull2DNearCollinearTest)
#[test]
fn hull_2d_near_collinear_test() {
    let input = [
        v2(0.427_424_28, -0.991_437_26),
        v2(0.858_270_76, -0.791_535_14),
        v2(1.585_600_4, -0.454_072_18),
    ];

    let mut pts = [Point2D::default(); 3];
    let mut hull = [Point2D::default(); 6];
    fill_points_2d(&mut pts, &input, 3);

    let count = hull_2d(&mut pts, 3, &mut hull, 0);
    assert!((2..=3).contains(&count));

    for i in 0..count as usize {
        for j in i + 1..count as usize {
            assert_ne!(hull[i].original_index, hull[j].original_index);
        }
    }
}

/// A 3 by 3 grid has three points on each side. Only the corners are strict hull vertices.
/// (Hull2DKeepCollinearTest)
#[test]
fn hull_2d_keep_collinear_test() {
    let mut input = [v2(0.0, 0.0); 9];
    for (i, point) in input.iter_mut().enumerate() {
        *point = v2((i % 3) as f32, (i / 3) as f32);
    }

    let mut pts = [Point2D::default(); 9];
    let mut hull = [Point2D::default(); 18];
    fill_points_2d(&mut pts, &input, 9);
    assert_eq!(hull_2d(&mut pts, 9, &mut hull, 0), 4);

    fill_points_2d(&mut pts, &input, 9);
    let count = hull_2d(&mut pts, 9, &mut hull, HULL_2D_KEEP_COLLINEAR);
    assert_eq!(count, 8);
    assert!(hull_2d_area(&hull, count) > 0.0);

    for point in hull.iter().take(count as usize) {
        assert_ne!(point.original_index, 4);
    }

    // All collinear keeps every point
    let line = [v2(0.0, 0.0), v2(1.0, 0.0), v2(2.0, 0.0), v2(3.0, 0.0)];
    fill_points_2d(&mut pts, &line, 4);
    assert_eq!(hull_2d(&mut pts, 4, &mut hull, HULL_2D_KEEP_COLLINEAR), 4);
}

/// Coincident points under the exact merge: the earlier point survives unless a later one is
/// deeper by more than the linear slop. The default weld always keeps the earlier point.
/// (Hull2DExactMergeTest)
#[test]
fn hull_2d_exact_merge_test() {
    let input = [
        v2(-1.0, -1.0),
        v2(1.0, -1.0),
        v2(1.0, 1.0),
        v2(-1.0, 1.0),
        v2(-1.0, -1.0),
    ];

    let depths = [2.0 * linear_slop(), 0.5 * linear_slop()];
    let expected = [4, 0];

    for k in 0..2 {
        let mut pts = [Point2D::default(); 5];
        let mut hull = [Point2D::default(); 10];
        fill_points_2d(&mut pts, &input, 5);
        pts[4].separation = -depths[k];

        let count = hull_2d(&mut pts, 5, &mut hull, HULL_2D_EXACT_MERGE);
        assert_eq!(count, 4);

        let mut survivor = -1;
        for point in hull.iter().take(count as usize) {
            if point.original_index == 0 || point.original_index == 4 {
                survivor = point.original_index;
            }
        }
        assert_eq!(survivor, expected[k]);

        fill_points_2d(&mut pts, &input, 5);
        pts[4].separation = -depths[k];
        let count = hull_2d(&mut pts, 5, &mut hull, 0);
        assert_eq!(count, 4);

        for point in hull.iter().take(count as usize) {
            assert_ne!(point.original_index, 4);
        }
    }
}

/// The index order flag returns the same points sorted by input index. (Hull2DIndexOrderTest)
#[test]
fn hull_2d_index_order_test() {
    let input = [
        v2(1.0, 2.0),
        v2(-2.5, 0.4),
        v2(2.0, -1.0),
        v2(0.2, 0.1),
        v2(-2.0, -1.0),
        v2(2.5, 0.5),
        v2(0.0, -1.5),
        v2(-1.0, 1.8),
    ];

    let mut pts = [Point2D::default(); 8];
    let mut hull = [Point2D::default(); 16];
    fill_points_2d(&mut pts, &input, 8);
    let count1 = hull_2d(&mut pts, 8, &mut hull, 0);

    let mut on_hull = [false; 8];
    for point in hull.iter().take(count1 as usize) {
        on_hull[point.original_index as usize] = true;
    }

    fill_points_2d(&mut pts, &input, 8);
    let count2 = hull_2d(&mut pts, 8, &mut hull, HULL_2D_INDEX_ORDER);
    assert_eq!(count2, count1);

    for i in 0..count2 as usize {
        assert!(on_hull[hull[i].original_index as usize]);
        assert!(i == 0 || hull[i - 1].original_index < hull[i].original_index);
    }
}
