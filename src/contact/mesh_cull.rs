//! Mesh manifold point culling and cluster reduction.
//!
//! Port of `b3IsBetterCullCandidate`, `b3FindFarthestCullPoint`, `b3CullPoints`,
//! `b3DropInteriorPoints`, `b3KeepDeepestPoint`, `b3ReduceCluster`, and `b3Cluster` from
//! `box3d-cpp-reference/src/mesh_contact.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::constants::{linear_slop, MAX_MANIFOLD_POINTS};
use crate::core::NULL_INDEX;
use crate::hull::{hull_2d, simplify_hull_2d, Point2D};
use crate::manifold::LocalManifoldPoint;
use crate::math_functions::{
    abs_float, cross, cross2, distance_squared2, dot, dot2, length2, max_float, min_float, perp,
    sub, sub2, Vec2, Vec3,
};

/// Returns true if (score, separation) should replace (bestScore, bestSeparation).
/// (b3IsBetterCullCandidate)
fn is_better_cull_candidate(
    score: f32,
    separation: f32,
    best_score: f32,
    best_separation: f32,
    score_tol: f32,
    separation_tol: f32,
) -> bool {
    if score > best_score + score_tol {
        return true;
    }
    if score < best_score - score_tol {
        return false;
    }

    // Break the tie using separation
    separation < best_separation - separation_tol
}

/// Find the farthest point from the base point. (b3FindFarthestCullPoint)
fn find_farthest_cull_point(
    points: &[Point2D],
    count: i32,
    index: i32,
    score_tol: f32,
    separation_tol: f32,
) -> i32 {
    let origin = points[index as usize].p;
    let mut best_score = 0.0f32;
    let mut best_separation = f32::MAX;
    let mut best_index = NULL_INDEX;
    for i in 0..count {
        if i == index {
            continue;
        }

        let score = distance_squared2(origin, points[i as usize].p);
        if is_better_cull_candidate(
            score,
            points[i as usize].separation,
            best_score,
            best_separation,
            score_tol,
            separation_tol,
        ) {
            best_index = i;
            best_score = score;
            best_separation = points[i as usize].separation;
        }
    }

    best_index
}

/// Cull a 2D point cloud down to at most 4 points. With `sweep` the first two points come
/// from two linear farthest point searches instead of the O(N^2) pair search. (b3CullPoints)
fn cull_points(points: &mut [Point2D], count: i32, sweep: bool) -> i32 {
    if count <= 1 {
        return count;
    }

    let tol = 0.25 * linear_slop();
    let tol_sqr = tol * tol;
    let separation_tol = linear_slop();

    let mut final_points = [Point2D::default(); 4];

    let mut best_score = 0.0f32;
    let mut best_separation = f32::MAX;
    let mut best_index1 = NULL_INDEX;
    let mut best_index2 = NULL_INDEX;

    if sweep {
        // Arbitrary search direction.
        let axis = Vec2 {
            x: 0.92106099,
            y: 0.38941834,
        };
        let mut start_index = 0;
        let mut start_score = dot2(axis, points[0].p);
        let mut start_separation = points[0].separation;
        for i in 1..count {
            let score = dot2(axis, points[i as usize].p);
            if is_better_cull_candidate(
                score,
                points[i as usize].separation,
                start_score,
                start_separation,
                tol,
                separation_tol,
            ) {
                start_index = i;
                start_score = score;
                start_separation = points[i as usize].separation;
            }
        }

        best_index1 = find_farthest_cull_point(points, count, start_index, tol_sqr, separation_tol);
        debug_assert!(best_index1 != NULL_INDEX);
        best_index2 = find_farthest_cull_point(points, count, best_index1, tol_sqr, separation_tol);
        debug_assert!(best_index2 != NULL_INDEX);
        best_score = distance_squared2(
            points[best_index1 as usize].p,
            points[best_index2 as usize].p,
        );
    } else {
        // Step 1: the two points with the largest distance, ties broken by deepest combined
        // separation
        for i in 0..count {
            let p1 = points[i as usize].p;
            for j in (i + 1)..count {
                let score = distance_squared2(p1, points[j as usize].p);
                // Separation sum heuristic
                let separation = points[i as usize].separation + points[j as usize].separation;

                if is_better_cull_candidate(
                    score,
                    separation,
                    best_score,
                    best_separation,
                    tol_sqr,
                    separation_tol,
                ) {
                    best_index1 = i;
                    best_index2 = j;
                    best_score = score;
                    best_separation = separation;
                }
            }
        }
    }

    if best_score < tol_sqr {
        // Choose deepest point
        let mut deepest_index = 0;
        for i in 1..count {
            if points[i as usize].separation < points[deepest_index as usize].separation {
                deepest_index = i;
            }
        }

        if deepest_index != 0 {
            points[0] = points[deepest_index as usize];
        }
        return 1;
    }

    final_points[0] = points[best_index1 as usize];
    final_points[1] = points[best_index2 as usize];

    if count == 2 {
        points[0] = final_points[0];
        points[1] = final_points[1];
        return 2;
    }

    // First anchor point
    let a = final_points[0].p;

    // Second anchor point
    let mut b = final_points[1].p;
    let mut ba = sub2(b, a);

    // Step 2: find the point with the maximum triangular area, ties broken by deepest separation
    best_score = 0.0;
    best_separation = f32::MAX;
    let mut best_index3 = NULL_INDEX;
    let mut best_signed_area = 0.0f32;
    for i in 0..count {
        // Skip best points 1 and 2 rather than culling them. This preserves order under
        // octogon culling.
        if i == best_index1 || i == best_index2 {
            continue;
        }

        let p = points[i as usize].p;
        let signed_area = cross2(ba, sub2(p, a));
        let score = abs_float(signed_area);

        if is_better_cull_candidate(
            score,
            points[i as usize].separation,
            best_score,
            best_separation,
            tol_sqr,
            separation_tol,
        ) {
            best_signed_area = signed_area;
            best_score = score;
            best_separation = points[i as usize].separation;
            best_index3 = i;
        }
    }

    if best_index3 == NULL_INDEX {
        // All points collinear
        points[0] = final_points[0];
        points[1] = final_points[1];
        return 2;
    }

    // Store best point
    final_points[2] = points[best_index3 as usize];

    if count == 3 {
        points[0] = final_points[0];
        points[1] = final_points[1];
        points[2] = final_points[2];
        return 3;
    }

    // Step 4: get the point that adds the most area outside the current triangle

    // Third anchor
    let mut c = final_points[2].p;

    // Ensure CCW ordering
    if best_signed_area < 0.0 {
        core::mem::swap(&mut b, &mut c);
        ba = sub2(b, a);
    }

    let cb = sub2(c, b);
    let ac = sub2(a, c);

    best_score = 0.0;
    best_separation = f32::MAX;
    let mut best_index4 = NULL_INDEX;
    for i in 0..count {
        if i == best_index1 || i == best_index2 || i == best_index3 {
            continue;
        }

        let p = points[i as usize].p;
        let u1 = cross2(sub2(p, a), ba);
        let u2 = cross2(sub2(p, b), cb);
        let u3 = cross2(sub2(p, c), ac);
        let score = max_float(u1, max_float(u2, u3));

        // Use the area tolerance for collinear points and hysteresis
        if is_better_cull_candidate(
            score,
            points[i as usize].separation,
            best_score,
            best_separation,
            tol_sqr,
            separation_tol,
        ) {
            best_score = score;
            best_separation = points[i as usize].separation;
            best_index4 = i;
        }
    }

    if best_index4 == NULL_INDEX {
        // No additional area
        points[0] = final_points[0];
        points[1] = final_points[1];
        points[2] = final_points[2];
        return 3;
    }

    // Store best point
    final_points[3] = points[best_index4 as usize];

    // Full quad
    points[0] = final_points[0];
    points[1] = final_points[1];
    points[2] = final_points[2];
    points[3] = final_points[3];
    4
}

/// Above this point count it is worth culling interior points prior to the regular culling
/// algorithm. (B3_CULL_FILTER_THRESHOLD)
const CULL_FILTER_THRESHOLD: i32 = 64;

/// If the cull filter leaves this many points then fallback to a linear tests for the first to
/// points. (B3_CULL_SWEEP_THRESHOLD)
const CULL_SWEEP_THRESHOLD: i32 = 256;

/// Drop interior points before computing the farthest two points. This is faster than
/// brute-force O(N^2). (b3DropInteriorPoints)
fn drop_interior_points(points: &mut [Point2D], count: i32, margin: f32) -> i32 {
    let mut extremes = [0usize; 8];
    let p0 = points[0].p;
    let mut bests = [
        p0.x,
        p0.x + p0.y,
        p0.y,
        p0.y - p0.x,
        -p0.x,
        -p0.x - p0.y,
        -p0.y,
        p0.x - p0.y,
    ];

    // Find the maximum offsets in the octogon.
    for i in 1..count as usize {
        let p = points[i].p;
        let values = [
            p.x,
            p.x + p.y,
            p.y,
            p.y - p.x,
            -p.x,
            -p.x - p.y,
            -p.y,
            p.x - p.y,
        ];
        for k in 0..8 {
            if values[k] > bests[k] {
                bests[k] = values[k];
                extremes[k] = i;
            }
        }
    }

    // Build 8 planes as an approximate convex hull for culling.
    let mut normals = [Vec2::default(); 8];
    let mut offsets = [0.0f32; 8];
    let mut edge_count = 0;
    for k in 0..8 {
        let a = points[extremes[k]].p;
        let b = points[extremes[(k + 1) & 7]].p;
        let normal = Vec2 {
            x: a.y - b.y,
            y: b.x - a.x,
        };
        let length = length2(normal);
        if length == 0.0 {
            continue;
        }

        normals[edge_count] = normal;
        offsets[edge_count] = dot2(normal, a) + margin * length;
        edge_count += 1;
    }

    if edge_count < 3 {
        return count;
    }

    // Cull points.
    let mut kept_count = 0;
    for i in 0..count as usize {
        let p = points[i].p;
        let mut interior = true;
        for k in 0..edge_count {
            if dot2(normals[k], p) <= offsets[k] {
                interior = false;
                break;
            }
        }

        if !interior {
            points[kept_count] = points[i];
            kept_count += 1;
        }
    }

    kept_count as i32
}

/// Fallback if culling leads to all points separated. Rare. (b3KeepDeepestPoint)
fn keep_deepest_point(
    kept: &mut [Point2D],
    kept_count: i32,
    points: &[LocalManifoldPoint],
    count: i32,
    origin: Vec3,
    u: Vec3,
    v: Vec3,
) {
    let mut deepest = 0;
    for i in 1..count {
        if points[i as usize].separation < points[deepest as usize].separation {
            deepest = i;
        }
    }

    let mut kept_separation = f32::MAX;
    for point in kept.iter().take(kept_count as usize) {
        if point.original_index == deepest {
            return;
        }

        kept_separation = min_float(
            kept_separation,
            points[point.original_index as usize].separation,
        );
    }

    let separation = points[deepest as usize].separation;
    if separation <= 0.0 || separation >= kept_separation - linear_slop() {
        return;
    }

    let mut replace_index = 0;
    let mut best_area = -1.0f32;
    for candidate in 0..kept_count {
        let mut ring = [Vec2::default(); MAX_MANIFOLD_POINTS];
        let mut ring_count = 0;
        for i in 0..kept_count {
            let index = if i == candidate {
                deepest
            } else {
                kept[i as usize].original_index
            };
            let d = sub(points[index as usize].point, origin);
            ring[ring_count] = Vec2 {
                x: dot(d, u),
                y: dot(d, v),
            };
            ring_count += 1;
        }

        let mut area = 0.0f32;
        for i in 0..ring_count {
            for j in (i + 1)..ring_count {
                for k in (j + 1)..ring_count {
                    let signed_area = cross2(sub2(ring[j], ring[i]), sub2(ring[k], ring[i]));
                    area = max_float(area, abs_float(signed_area));
                }
            }
        }

        if area > best_area {
            best_area = area;
            replace_index = candidate;
        }
    }

    kept[replace_index as usize].original_index = deepest;
}

/// Reduce a cluster of local manifold points to ≤ [`MAX_MANIFOLD_POINTS`].
/// (b3ReduceCluster)
pub(crate) fn reduce_cluster(points: &mut [LocalManifoldPoint], count1: i32, normal: Vec3) -> i32 {
    let target_count = 1;
    if count1 <= target_count {
        return count1;
    }

    let mut pts = vec![Point2D::default(); count1 as usize];
    let u = perp(normal);
    let v = cross(normal, u);
    let origin = points[0].point;

    for i in 0..count1 {
        let d = sub(points[i as usize].point, origin);
        pts[i as usize].p = Vec2 {
            x: dot(d, u),
            y: dot(d, v),
        };
        pts[i as usize].separation = points[i as usize].separation;
        pts[i as usize].original_index = i;
    }

    // C selects between these with `#if B3_MAX_MANIFOLD_POINTS == 4`.
    let mut hull_buffer;
    let (hull, count2): (&mut [Point2D], i32) = if MAX_MANIFOLD_POINTS == 4 {
        let count2 = if count1 <= CULL_FILTER_THRESHOLD {
            cull_points(&mut pts, count1, false)
        } else {
            // Many points. Reduce point count using bounding octogon.
            let kept_count = drop_interior_points(&mut pts, count1, 0.25 * linear_slop());
            cull_points(&mut pts, kept_count, kept_count > CULL_SWEEP_THRESHOLD)
        };
        (&mut pts, count2)
    } else {
        hull_buffer = vec![Point2D::default(); 2 * count1 as usize];
        let hull_count = hull_2d(&mut pts, count1, &mut hull_buffer, 0);
        let count2 = simplify_hull_2d(&mut hull_buffer, hull_count, MAX_MANIFOLD_POINTS as i32);
        (&mut hull_buffer, count2)
    };
    debug_assert!(count2 as usize <= MAX_MANIFOLD_POINTS);

    keep_deepest_point(hull, count2, points, count1, origin, u, v);

    let mut final_points = [LocalManifoldPoint::default(); MAX_MANIFOLD_POINTS];
    for i in 0..count2 {
        let index = hull[i as usize].original_index;
        debug_assert!(0 <= index && index < count1);
        final_points[i as usize] = points[index as usize];
    }

    points[..count2 as usize].copy_from_slice(&final_points[..count2 as usize]);
    count2
}

/// Scratch cluster while assembling mesh manifolds. The points are a range of the cluster
/// point buffer. (b3Cluster)
pub(crate) struct Cluster {
    pub manifold_normal: Vec3,
    pub triangle_normal: Vec3,
    pub point_start: usize,
    pub point_capacity: i32,
    pub point_count: i32,
}
