//! Manifold collision unit tests.
//!
//! `primitives` holds the focused Rust-authored coverage for the convex
//! primitive collide slice. `edge_hull` and `edge_triangle` port the C
//! `test/test_manifold.c` suite (upstream commit aaa795e "Edge edge
//! optimization"), which exercises the reworked edge-edge axis construction.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

mod edge_hull;
mod edge_triangle;
mod primitives;
mod sat;

use crate::hull::{get_hull_edges, get_hull_points, HullData};
use crate::manifold::LocalManifold;
use crate::math_functions::{
    add, cross, dot, length, line_distance, min_float, mul_sub, mul_sv, neg, normalize, sub,
    transform_point, Quat, Transform, Vec3, QUAT_IDENTITY, VEC3_ZERO,
};

// b3ComputeCosSin is a rational approximation good to about 1e-3. That is coarse enough to
// shift an edge off the position the analytic result expects, so these fixtures need libm.
pub(super) const ROOT2: f32 = 1.41421356;
pub(super) const HALF_ROOT2: f32 = 0.70710678;

pub(super) const AXIS_X: Vec3 = Vec3 {
    x: 1.0,
    y: 0.0,
    z: 0.0,
};
pub(super) const AXIS_Y: Vec3 = Vec3 {
    x: 0.0,
    y: 1.0,
    z: 0.0,
};
pub(super) const AXIS_Z: Vec3 = Vec3 {
    x: 0.0,
    y: 0.0,
    z: 1.0,
};

pub(super) const TILT_AXES: [Vec3; 4] = [
    Vec3 {
        x: 0.57735027,
        y: 0.57735027,
        z: 0.57735027,
    },
    Vec3 {
        x: 0.70710678,
        y: 0.0,
        z: 0.70710678,
    },
    Vec3 {
        x: 0.26726124,
        y: 0.53452248,
        z: 0.80178373,
    },
    Vec3 {
        x: -0.48507125,
        y: 0.72760688,
        z: -0.48507125,
    },
];

/// Angles that straddle the 0.005 rejection threshold.
pub(super) const TILT_ANGLES: [f32; 11] = [
    0.0, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 0.004, 0.005, 0.006, 0.01, 0.05,
];

/// A cube corner is root3/2 from the center of rotation.
pub(super) const HALF_DIAGONAL: f32 = 0.87;

/// Matches the C ENSURE_SMALL macro, which is inclusive: pass when
/// `-tol <= value <= tol`.
pub(super) fn ensure_small(value: f32, tolerance: f32) {
    assert!(
        !(value < -tolerance || tolerance < value),
        "|{value}| > tolerance {tolerance}"
    );
}

pub(super) fn v(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

/// Exact quaternion from an axis-angle, using libm sin/cos rather than the
/// deterministic `b3ComputeCosSin` approximation. (ExactQuat)
pub(super) fn exact_quat(axis: Vec3, radians: f32) -> Quat {
    let half = 0.5 * radians;
    let s = half.sin();
    Quat {
        v: Vec3 {
            x: s * axis.x,
            y: s * axis.y,
            z: s * axis.z,
        },
        s: half.cos(),
    }
}

/// (ExactRotation)
pub(super) fn exact_rotation(axis: Vec3, radians: f32) -> Transform {
    Transform {
        p: VEC3_ZERO,
        q: exact_quat(axis, radians),
    }
}

/// (SlideX)
pub(super) fn slide_x(x: f32) -> Transform {
    Transform {
        p: v(x, 0.0, 0.0),
        q: QUAT_IDENTITY,
    }
}

/// Deepest (most negative) point separation. (MinSeparation)
pub(super) fn min_separation(manifold: &LocalManifold) -> f32 {
    let mut min = f32::MAX;
    for i in 0..manifold.point_count {
        min = min_float(min, manifold.points[i as usize].separation);
    }
    min
}

/// Deterministic linear congruential generator matching the C test globals.
/// (g_seed / NextFloat / NextDirection)
pub(super) struct Rng {
    seed: u32,
}

impl Rng {
    pub(super) fn new(seed: u32) -> Self {
        Rng { seed }
    }

    pub(super) fn next_float(&mut self, lower: f32, upper: f32) -> f32 {
        self.seed = 1664525u32
            .wrapping_mul(self.seed)
            .wrapping_add(1013904223u32);
        let t = (self.seed >> 8) as f32 / (1u32 << 24) as f32;
        lower + t * (upper - lower)
    }

    pub(super) fn next_direction(&mut self) -> Vec3 {
        // C evaluates the three NextFloat calls left to right in the aggregate
        // initializer; bind them in order so the RNG stream matches.
        let x = self.next_float(-1.0, 1.0);
        let y = self.next_float(-1.0, 1.0);
        let z = self.next_float(-1.0, 1.0);
        normalize(Vec3 { x, y, z })
    }
}

/// Pull the tail point and edge vector of a hull half-edge in a given frame.
/// (HullEdgeSegment)
pub(super) fn hull_edge_segment(
    hull: &HullData,
    edge_index: i32,
    transform: Transform,
) -> (Vec3, Vec3) {
    let edges = get_hull_edges(hull);
    let points = get_hull_points(hull);
    let e = &edges[edge_index as usize];
    let tail = transform_point(transform, points[e.origin as usize]);
    let head = transform_point(transform, points[edges[e.twin as usize].origin as usize]);
    (tail, sub(head, tail))
}

/// The edge pair axis produced by the arc intersection must match the classic edge cross product.
/// Rebuild the axis, the separation and the contact point from the two contributing edges and the
/// convex radius, then compare against the manifold. `orient_ref` fixes the sign of the axis to
/// match the manifold normal convention for the shape pair. `e1` belongs to the shape whose contact
/// point is pulled in by the radius (the hull or triangle when it meets a capsule), `e2` to the
/// other edge. (CheckEdgeContact)
#[allow(clippy::too_many_arguments)]
pub(super) fn check_edge_contact(
    manifold: &LocalManifold,
    p1: Vec3,
    e1: Vec3,
    p2: Vec3,
    e2: Vec3,
    orient_ref: Vec3,
    radius: f32,
    normal_tol: f32,
    sep_tol: f32,
    point_tol: f32,
) {
    let mut axis = normalize(cross(e1, e2));
    if dot(axis, orient_ref) < 0.0 {
        axis = neg(axis);
    }

    // Normal matches the cross product and is perpendicular to both edges
    ensure_small(manifold.normal.x - axis.x, normal_tol);
    ensure_small(manifold.normal.y - axis.y, normal_tol);
    ensure_small(manifold.normal.z - axis.z, normal_tol);
    ensure_small(dot(manifold.normal, normalize(e1)), normal_tol);
    ensure_small(dot(manifold.normal, normalize(e2)), normal_tol);

    let closest = line_distance(p1, e1, p2, e2);

    // Signed gap between the edge lines along the axis, less the capsule radius
    let expected_separation = dot(axis, sub(closest.point2, closest.point1)) - radius;
    ensure_small(manifold.points[0].separation - expected_separation, sep_tol);

    // Midpoint of the closest approach, pulling the first point in by the radius
    let expected_point = mul_sv(
        0.5,
        add(mul_sub(closest.point1, radius, axis), closest.point2),
    );
    ensure_small(manifold.points[0].point.x - expected_point.x, point_tol);
    ensure_small(manifold.points[0].point.y - expected_point.y, point_tol);
    ensure_small(manifold.points[0].point.z - expected_point.z, point_tol);
}

/// Length of a manifold normal (used by the overlap sweep). Re-exported helper.
pub(super) fn normal_length(manifold: &LocalManifold) -> f32 {
    length(manifold.normal)
}
