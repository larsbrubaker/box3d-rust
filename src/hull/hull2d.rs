//! 2D convex hull and hull simplification used for contact manifold reduction.
//!
//! Port of `b3Hull2D`, `b3SimplifyHull2D`, and their helpers (`b3Cross2D`,
//! `b3IsPoint2DLess`, `b3Weld2D`, `b3MergeExact2D`) from `box3d-cpp-reference/src/hull.c`, plus the
//! public `b3Point2D` type and `b3Hull2DFlags` from `include/box3d/collision.h`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::constants::linear_slop;
use crate::math_functions::{distance_squared2, Vec2};

/// This is used for computing 2D hulls in service of contact manifold simplification.
/// Placed here for testing. Internal. (b3Point2D)
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Point2D {
    /// 2D point
    pub p: Vec2,
    /// separation of the associated contact point
    pub separation: f32,
    /// original index of the associated contact point
    pub original_index: i32,
}

// 2D hull (Andrew's monotone chain)

/// cross(b - a, c - a) (b3Cross2D)
#[inline]
fn cross_2d(a: Vec2, b: Vec2, c: Vec2) -> f32 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

/// Simplify a 2D hull. Used internally for contact manifold simplification. Here for testing.
/// Iteratively remove the vertices contributing the least area. (b3SimplifyHull2D)
pub fn simplify_hull_2d(hull: &mut [Point2D], count1: i32, target: i32) -> i32 {
    debug_assert!(target >= 3);

    if count1 <= 3 {
        return count1;
    }

    let linear_slop = linear_slop();
    let area_tol = 0.25 * linear_slop * linear_slop;
    let mut count2 = count1;

    loop {
        let mut min_area = f32::MAX;
        let mut min_index = 0;

        for i in 0..count2 {
            let prev = (i + count2 - 1) % count2;
            let next = (i + 1) % count2;
            let area = cross_2d(
                hull[prev as usize].p,
                hull[i as usize].p,
                hull[next as usize].p,
            );
            debug_assert!(area >= -f32::EPSILON);

            if area + area_tol < min_area {
                min_area = area;
                min_index = i;
            }
        }

        if count2 <= 3 || (min_area > area_tol && count2 <= target) {
            break;
        }

        count2 -= 1;
        if min_index < count2 {
            // memmove( hull + minIndex, hull + ( minIndex + 1 ), ( count2 - minIndex ) * sizeof( b3Point2D ) )
            hull.copy_within(
                (min_index + 1) as usize..(count2 + 1) as usize,
                min_index as usize,
            );
        }
    }

    count2
}

// Options for how a 2D hull is built. (b3Hull2DFlags)

/// Keep collinear points. (b3_hull2DKeepCollinear)
pub const HULL_2D_KEEP_COLLINEAR: i32 = 0x01;

/// Only merge identical points. (b3_hull2DExactMerge)
pub const HULL_2D_EXACT_MERGE: i32 = 0x02;

/// Order the output points by the input order. (b3_hull2DIndexOrder)
pub const HULL_2D_INDEX_ORDER: i32 = 0x04;

/// Comparator for sorting points for the 2D hull. (b3IsPoint2DLess)
#[inline]
fn is_point_2d_less(a: &Point2D, b: &Point2D) -> bool {
    if a.p.x != b.p.x {
        return a.p.x < b.p.x;
    }

    if a.p.y != b.p.y {
        return a.p.y < b.p.y;
    }

    a.original_index < b.original_index
}

/// Welds points that have been sorted O(n) time (b3Weld2D)
fn weld_2d(pts: &mut [Point2D], count1: i32) -> i32 {
    if count1 <= 1 {
        return count1;
    }

    let linear_slop = 0.25 * linear_slop();
    let tol_sqr = linear_slop * linear_slop;

    let mut count2 = 1;
    let mut base_index = 0;
    for i in 1..count1 {
        let distance_sqr = distance_squared2(pts[i as usize].p, pts[base_index as usize].p);
        if distance_sqr < tol_sqr {
            if pts[i as usize].original_index < pts[base_index as usize].original_index {
                pts[base_index as usize] = pts[i as usize];
            }
        } else {
            pts[count2 as usize] = pts[i as usize];
            base_index = count2;
            count2 += 1;
        }
    }

    count2
}

/// Only merge points that line up exactly. (b3MergeExact2D)
fn merge_exact_2d(pts: &mut [Point2D], count1: i32) -> i32 {
    let separation_tol = linear_slop();

    let mut count2: i32 = 0;
    for i in 0..count1 {
        let point = pts[i as usize];
        if count2 > 0
            && pts[(count2 - 1) as usize].p.x == point.p.x
            && pts[(count2 - 1) as usize].p.y == point.p.y
        {
            if point.separation < pts[(count2 - 1) as usize].separation - separation_tol {
                pts[(count2 - 1) as usize] = point;
            }
        } else {
            pts[count2 as usize] = point;
            count2 += 1;
        }
    }

    count2
}

/// Compute a 2D hull. Used internally for contact manifold simplification. Here for testing.
/// Andrew's monotone chain convex hull algorithm.
/// `hull` must have space for 2 * `count` points. `pts` is sorted and welded in place.
/// `flags` combines the `HULL_2D_*` bits (b3Hull2DFlags). (b3Hull2D)
pub fn hull_2d(pts: &mut [Point2D], count: i32, hull: &mut [Point2D], flags: i32) -> i32 {
    if count <= 0 {
        return 0;
    }

    // C sorts with QSORT on b3IsPoint2DLess.
    crate::qsort::qsort(pts, count as usize, is_point_2d_less);

    let count1 = if (flags & HULL_2D_EXACT_MERGE) != 0 {
        merge_exact_2d(pts, count)
    } else {
        weld_2d(pts, count)
    };

    let keep_collinear = (flags & HULL_2D_KEEP_COLLINEAR) != 0;

    let mut count2: i32 = 0;
    if count1 <= 2 {
        for i in 0..count1 {
            hull[i as usize] = pts[i as usize];
        }

        count2 = count1;
    } else {
        for i in 0..count1 {
            while count2 >= 2 {
                let area = cross_2d(
                    hull[(count2 - 2) as usize].p,
                    hull[(count2 - 1) as usize].p,
                    pts[i as usize].p,
                );
                if area > 0.0 || (keep_collinear && area == 0.0) {
                    break;
                }

                count2 -= 1;
            }

            debug_assert!(count2 < 2 * count1);
            hull[count2 as usize] = pts[i as usize];
            count2 += 1;
        }

        let lower_count = count2 + 1;
        let mut lower_index = count2 - 2;

        let mut i = count1 - 2;
        while i >= 0 {
            if lower_index >= 0
                && hull[lower_index as usize].p.x == pts[i as usize].p.x
                && hull[lower_index as usize].p.y == pts[i as usize].p.y
            {
                lower_index -= 1;
                if i > 0 {
                    i -= 1;
                    continue;
                }
            }

            while count2 >= lower_count {
                let area = cross_2d(
                    hull[(count2 - 2) as usize].p,
                    hull[(count2 - 1) as usize].p,
                    pts[i as usize].p,
                );
                if area > 0.0 || (keep_collinear && area == 0.0) {
                    break;
                }

                count2 -= 1;
            }

            debug_assert!(count2 < 2 * count1);
            hull[count2 as usize] = pts[i as usize];
            count2 += 1;
            i -= 1;
        }

        debug_assert!(hull[0].original_index == hull[(count2 - 1) as usize].original_index);
        count2 -= 1;
    }

    debug_assert!(count2 <= count1);

    if (flags & HULL_2D_INDEX_ORDER) != 0 {
        // C sorts with QSORT on the original index.
        crate::qsort::qsort(hull, count2 as usize, |a, b| {
            a.original_index < b.original_index
        });
    }

    count2
}
