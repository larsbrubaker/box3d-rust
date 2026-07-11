//! Mesh manifold point culling and ghost-collision edge/vertex tracking.
//!
//! Port of the cull / reduce / found-edge helpers from
//! `box3d-cpp-reference/src/mesh_contact.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::constants::{linear_slop, MAX_MANIFOLD_POINTS};
use crate::core::NULL_INDEX;
use crate::manifold::LocalManifoldPoint;
use crate::math_functions::{
    abs_float, cross, cross2, distance_squared2, dot, max_float, max_int, min_int, perp, sub, sub2,
    Vec2, Vec3,
};

/// Max unique mesh edges tracked while accepting manifolds. (B3_MAX_EDGE_COUNT)
const MAX_EDGE_COUNT: usize = 64;
/// Max unique mesh vertices tracked while accepting manifolds. (B3_MAX_VERTEX_COUNT)
const MAX_VERTEX_COUNT: usize = 64;

/// Edges already claimed by accepted manifolds. (b3FoundEdges)
#[derive(Clone)]
pub(crate) struct FoundEdges {
    keys: [u64; MAX_EDGE_COUNT],
    count: i32,
}

impl FoundEdges {
    pub(crate) fn new() -> Self {
        Self {
            keys: [0; MAX_EDGE_COUNT],
            count: 0,
        }
    }

    /// Returns true if the edge is newly added (or buffer full → treat as new).
    /// (b3AddEdge)
    pub(crate) fn add(&mut self, vertex1: i32, vertex2: i32) -> bool {
        let i1 = min_int(vertex1, vertex2) as u64;
        let i2 = max_int(vertex1, vertex2) as u64;
        let key = (i1 << 32) | i2;

        let count = self.count;
        for i in 0..count {
            if self.keys[i as usize] == key {
                return false;
            }
        }

        if count as usize == MAX_EDGE_COUNT {
            // Potential ghost collision if we cannot track the edge.
            return true;
        }

        self.keys[count as usize] = key;
        self.count += 1;
        true
    }

    /// (b3FindEdge)
    pub(crate) fn find(&self, vertex1: i32, vertex2: i32) -> bool {
        let i1 = min_int(vertex1, vertex2) as u64;
        let i2 = max_int(vertex1, vertex2) as u64;
        let key = (i1 << 32) | i2;

        for i in 0..self.count {
            if self.keys[i as usize] == key {
                return true;
            }
        }
        false
    }
}

/// Vertices already claimed by accepted manifolds. (b3FoundVertices)
#[derive(Clone)]
pub(crate) struct FoundVertices {
    keys: [i32; MAX_VERTEX_COUNT],
    count: i32,
}

impl FoundVertices {
    pub(crate) fn new() -> Self {
        Self {
            keys: [0; MAX_VERTEX_COUNT],
            count: 0,
        }
    }

    /// Returns true if the vertex is newly added (or buffer full → treat as new).
    /// (b3AddVertex)
    pub(crate) fn add(&mut self, vertex: i32) -> bool {
        let count = self.count;
        for i in 0..count {
            if self.keys[i as usize] == vertex {
                return false;
            }
        }

        if count as usize == MAX_VERTEX_COUNT {
            return true;
        }

        self.keys[count as usize] = vertex;
        self.count += 1;
        true
    }
}

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
    separation < best_separation - separation_tol
}

#[derive(Clone, Copy, Default)]
struct Point2D {
    p: Vec2,
    separation: f32,
    original_index: i32,
}

/// Cull a 2D point cloud down to at most 4 points. (b3CullPoints)
fn cull_points(points: &mut [Point2D], count: i32) -> i32 {
    if count <= 1 {
        return count;
    }

    let tol = 0.25 * linear_slop();
    let tol_sqr = tol * tol;
    let separation_tol = linear_slop();

    let mut final_points = [Point2D::default(); 4];
    let mut count1 = count;

    // Step 1: the two points with the largest distance, ties broken by deepest
    // combined separation.
    let mut best_score = 0.0;
    let mut best_separation = f32::MAX;
    let mut best_index1 = NULL_INDEX;
    let mut best_index2 = NULL_INDEX;

    for i in 0..count1 {
        let p1 = points[i as usize].p;
        for j in (i + 1)..count1 {
            let score = distance_squared2(p1, points[j as usize].p);
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

    if best_score < tol_sqr {
        let mut deepest_index = 0;
        for i in 1..count1 {
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

    points[best_index2 as usize] = points[(count1 - 1) as usize];
    points[best_index1 as usize] = points[(count1 - 2) as usize];
    count1 -= 2;

    if count1 == 0 {
        points[0] = final_points[0];
        points[1] = final_points[1];
        return 2;
    }

    let a = final_points[0].p;
    let mut b = final_points[1].p;
    let mut ba = sub2(b, a);

    // Step 2: maximum triangular area, ties broken by deepest separation.
    best_score = 0.0;
    best_separation = f32::MAX;
    let mut best_index = NULL_INDEX;
    let mut best_signed_area = 0.0;
    for i in 0..count1 {
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
            best_index = i;
        }
    }

    if best_index == NULL_INDEX {
        points[0] = final_points[0];
        points[1] = final_points[1];
        return 2;
    }

    final_points[2] = points[best_index as usize];

    if count1 == 1 {
        points[0] = final_points[0];
        points[1] = final_points[1];
        points[2] = final_points[2];
        return 3;
    }

    points[best_index as usize] = points[(count1 - 1) as usize];
    count1 -= 1;

    // Step 4: point that adds the most area outside the current triangle.
    let mut c = final_points[2].p;

    if best_signed_area < 0.0 {
        core::mem::swap(&mut b, &mut c);
        ba = sub2(b, a);
    }

    let cb = sub2(c, b);
    let ac = sub2(a, c);

    best_score = 0.0;
    best_separation = f32::MAX;
    best_index = NULL_INDEX;
    for i in 0..count1 {
        let p = points[i as usize].p;
        let u1 = cross2(sub2(p, a), ba);
        let u2 = cross2(sub2(p, b), cb);
        let u3 = cross2(sub2(p, c), ac);
        let score = max_float(u1, max_float(u2, u3));

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
            best_index = i;
        }
    }

    if best_index == NULL_INDEX {
        points[0] = final_points[0];
        points[1] = final_points[1];
        points[2] = final_points[2];
        return 3;
    }

    final_points[3] = points[best_index as usize];

    points[0] = final_points[0];
    points[1] = final_points[1];
    points[2] = final_points[2];
    points[3] = final_points[3];
    4
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

    let count2 = cull_points(&mut pts, count1);
    debug_assert!(count2 as usize <= MAX_MANIFOLD_POINTS);

    let mut final_points = [LocalManifoldPoint::default(); MAX_MANIFOLD_POINTS];
    for i in 0..count2 {
        let index = pts[i as usize].original_index;
        debug_assert!(0 <= index && index < count1);
        final_points[i as usize] = points[index as usize];
    }

    for i in 0..count2 {
        points[i as usize] = final_points[i as usize];
    }
    count2
}

/// Scratch cluster while assembling mesh manifolds. (b3Cluster)
pub(crate) struct Cluster {
    pub manifold_normal: Vec3,
    pub triangle_normal: Vec3,
    pub point_start: usize,
    pub point_capacity: i32,
    pub point_count: i32,
}

impl Cluster {
    pub(crate) fn new(manifold_normal: Vec3, triangle_normal: Vec3, point_capacity: i32) -> Self {
        Self {
            manifold_normal,
            triangle_normal,
            point_start: 0,
            point_capacity,
            point_count: 0,
        }
    }
}

/// Mark triangle edges/vertices claimed by an accepted face manifold.
pub(crate) fn claim_triangle_features(
    edges: &mut FoundEdges,
    vertices: &mut FoundVertices,
    i1: i32,
    i2: i32,
    i3: i32,
) {
    let _ = edges.add(i1, i2);
    let _ = edges.add(i2, i3);
    let _ = edges.add(i3, i1);
    let _ = vertices.add(i1);
    let _ = vertices.add(i2);
    let _ = vertices.add(i3);
}
