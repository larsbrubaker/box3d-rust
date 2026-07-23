//! Separating axis test coverage. Port of `box3d-cpp-reference/test/test_sat.c`
//! (upstream commit c37cfe4 "SIMD hull collision").
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{ensure_small, exact_quat, exact_rotation, Rng};
use crate::constants::speculative_distance;
use crate::hull::{
    get_hull_edges, get_hull_planes, get_hull_points, make_box_hull, make_offset_box_hull,
    make_transformed_box_hull, HullData,
};
use crate::manifold::{compute_separating_axis, SeparatingFeature};
use crate::math_functions::{
    cross, dot, length, max_float, min_float, mul_add, mul_sv, neg, normalize, rotate_vector,
    transform_point, Transform, Vec3, PI, QUAT_IDENTITY, VEC3_ZERO,
};

const ROOT2: f32 = 1.41421356;
const AXIS_Y: Vec3 = Vec3 {
    x: 0.0,
    y: 1.0,
    z: 0.0,
};
const AXIS_Z: Vec3 = Vec3 {
    x: 0.0,
    y: 0.0,
    z: 1.0,
};

/// Directed separation along a unit axis n pointing from A to B. Positive means B clears A
/// along n. This is the projection gap min over B minus max over A, which is exactly what a
/// valid separating axis measures, so the winning face or edge axis must reproduce it.
/// (SepAlong)
fn sep_along(hull_a: &HullData, hull_b: &HullData, xf_b: Transform, n: Vec3) -> f32 {
    let points_a = get_hull_points(hull_a);
    let mut max_a = -f32::MAX;
    for i in 0..hull_a.vertex_count {
        max_a = max_float(max_a, dot(n, points_a[i as usize]));
    }

    let points_b = get_hull_points(hull_b);
    let mut min_b = f32::MAX;
    for i in 0..hull_b.vertex_count {
        min_b = min_float(min_b, dot(n, transform_point(xf_b, points_b[i as usize])));
    }

    min_b - max_a
}

/// (TryAxis)
fn try_axis(
    hull_a: &HullData,
    hull_b: &HullData,
    xf_b: Transform,
    n: Vec3,
    best: &mut f32,
    best_normal: &mut Vec3,
) {
    let s = sep_along(hull_a, hull_b, xf_b, n);
    if s > *best {
        *best = s;
        *best_normal = n;
    }
}

/// Brute force SAT. The maximum separation over every face normal and every edge cross product
/// is the true answer the SIMD query has to match. Each undirected axis is tried both ways so
/// the winner comes out oriented from A to B with no sign bookkeeping. (OracleSeparation)
fn oracle_separation(hull_a: &HullData, hull_b: &HullData, xf_b: Transform) -> (f32, Vec3) {
    let mut best = -f32::MAX;
    let mut best_normal = VEC3_ZERO;

    let planes_a = get_hull_planes(hull_a);
    for i in 0..hull_a.face_count {
        try_axis(
            hull_a,
            hull_b,
            xf_b,
            planes_a[i as usize].normal,
            &mut best,
            &mut best_normal,
        );
        try_axis(
            hull_a,
            hull_b,
            xf_b,
            neg(planes_a[i as usize].normal),
            &mut best,
            &mut best_normal,
        );
    }

    let planes_b = get_hull_planes(hull_b);
    for i in 0..hull_b.face_count {
        let n = rotate_vector(xf_b.q, planes_b[i as usize].normal);
        try_axis(hull_a, hull_b, xf_b, n, &mut best, &mut best_normal);
        try_axis(hull_a, hull_b, xf_b, neg(n), &mut best, &mut best_normal);
    }

    let edges_a = get_hull_edges(hull_a);
    let points_a = get_hull_points(hull_a);
    let edges_b = get_hull_edges(hull_b);
    let points_b = get_hull_points(hull_b);

    // Half edges are stored as adjacent twin pairs, so the even index and its successor bound one edge.
    let mut i = 0;
    while i < hull_a.edge_count {
        let dir_a = crate::math_functions::sub(
            points_a[edges_a[(i + 1) as usize].origin as usize],
            points_a[edges_a[i as usize].origin as usize],
        );
        let mut j = 0;
        while j < hull_b.edge_count {
            let dir_b = rotate_vector(
                xf_b.q,
                crate::math_functions::sub(
                    points_b[edges_b[(j + 1) as usize].origin as usize],
                    points_b[edges_b[j as usize].origin as usize],
                ),
            );
            let mut cr = cross(dir_a, dir_b);

            // Near parallel edges give no useful axis and cannot be the true maximum.
            if dot(cr, cr) < 1e-10 {
                j += 2;
                continue;
            }

            cr = normalize(cr);
            try_axis(hull_a, hull_b, xf_b, cr, &mut best, &mut best_normal);
            try_axis(hull_a, hull_b, xf_b, neg(cr), &mut best, &mut best_normal);

            j += 2;
        }

        i += 2;
    }

    (best, best_normal)
}

// Two axis aligned cubes pushed apart along x. A's plus x face is the separating axis, the gap
// is large enough to trip the speculative early out, and the reference vertex on B is its minus
// x side. (FaceAxisASeparatedTest)
#[test]
fn face_axis_a_separated_test() {
    let hull_a = make_box_hull(0.5, 0.5, 0.5);
    let hull_b = make_box_hull(0.5, 0.5, 0.5);

    let xf_b = Transform {
        p: Vec3 {
            x: 1.2,
            y: 0.0,
            z: 0.0,
        },
        q: QUAT_IDENTITY,
    };
    let q = compute_separating_axis(
        &hull_a.base,
        &hull_b.base,
        xf_b,
        SeparatingFeature::InvalidAxis,
    );

    assert_eq!(q.type_, SeparatingFeature::FaceAxisA);
    ensure_small(q.separation - 0.2, 1e-5);
    ensure_small(q.normal.x - 1.0, 1e-6);
    ensure_small(q.normal.y, 1e-6);
    ensure_small(q.normal.z, 1e-6);

    let planes_a = get_hull_planes(&hull_a.base);
    ensure_small(planes_a[q.index_a as usize].normal.x - 1.0, 1e-6);

    let points_b = get_hull_points(&hull_b.base);
    ensure_small(points_b[q.index_b as usize].x + 0.5, 1e-6);

    ensure_small(
        sep_along(&hull_a.base, &hull_b.base, xf_b, q.normal) - q.separation,
        1e-5,
    );
}

// A yawed 45 degrees about z so all of its faces sit oblique to x. Only B still has a face square
// to the gap, so the reference has to be B's minus x face. This is the case that ties when both
// hulls are aligned, and the tie is what a face A bias would hide. (FaceAxisBSeparatedTest)
#[test]
fn face_axis_b_separated_test() {
    let hull_a = make_transformed_box_hull(0.5, 0.5, 0.5, exact_rotation(AXIS_Z, 0.25 * PI));
    let hull_b = make_box_hull(0.5, 0.5, 0.5);

    let a_extent = 0.5 * ROOT2;
    let gap = 0.2;
    let d = a_extent + 0.5 + gap;
    let xf_b = Transform {
        p: Vec3 {
            x: d,
            y: 0.0,
            z: 0.0,
        },
        q: QUAT_IDENTITY,
    };

    let q = compute_separating_axis(
        &hull_a.base,
        &hull_b.base,
        xf_b,
        SeparatingFeature::InvalidAxis,
    );

    assert_eq!(q.type_, SeparatingFeature::FaceAxisB);
    ensure_small(q.separation - gap, 1e-5);
    ensure_small(q.normal.x - 1.0, 1e-5);
    ensure_small(q.normal.y, 1e-5);
    ensure_small(q.normal.z, 1e-5);

    let planes_b = get_hull_planes(&hull_b.base);
    ensure_small(planes_b[q.index_b as usize].normal.x + 1.0, 1e-5);

    let points_a = get_hull_points(&hull_a.base);
    ensure_small(points_a[q.index_a as usize].x - a_extent, 1e-4);

    ensure_small(
        sep_along(&hull_a.base, &hull_b.base, xf_b, q.normal) - q.separation,
        1e-4,
    );
}

// Well beyond the speculative distance the face phase must report the axis and separation
// without running the edge phase. A sign slip in the early out would read as deep overlap.
// (FaceFarSeparatedTest)
#[test]
fn face_far_separated_test() {
    let hull_a = make_box_hull(0.5, 0.5, 0.5);
    let hull_b = make_box_hull(0.5, 0.5, 0.5);

    let xf_b = Transform {
        p: Vec3 {
            x: 3.0,
            y: 0.0,
            z: 0.0,
        },
        q: QUAT_IDENTITY,
    };
    let q = compute_separating_axis(
        &hull_a.base,
        &hull_b.base,
        xf_b,
        SeparatingFeature::InvalidAxis,
    );

    assert_eq!(q.type_, SeparatingFeature::FaceAxisA);
    ensure_small(q.separation - 2.0, 1e-5);
    ensure_small(q.normal.x - 1.0, 1e-6);

    let (oracle_sep, _oracle_normal) = oracle_separation(&hull_a.base, &hull_b.base, xf_b);
    ensure_small(q.separation - oracle_sep, 1e-5);
}

// The support bias is derived from each hull's AABB, not from an origin assumed to sit inside it.
// A box built far from its own local origin is the case the old diagonal bias got wrong. A is
// yawed and pushed out to x = 5, so the face B query has to run getSupport over vertices
// clustered near x = 5 and still pick the right one. (OffsetFaceAxisBTest)
#[test]
fn offset_face_axis_b_test() {
    let cx = 5.0;
    let place_a = Transform {
        p: Vec3 {
            x: cx,
            y: 0.0,
            z: 0.0,
        },
        q: exact_quat(AXIS_Z, 0.25 * PI),
    };
    let hull_a = make_transformed_box_hull(0.5, 0.5, 0.5, place_a);
    let hull_b = make_box_hull(0.5, 0.5, 0.5);

    let a_extent = cx + 0.5 * ROOT2;
    let gap = 0.2;
    let d = a_extent + 0.5 + gap;
    let xf_b = Transform {
        p: Vec3 {
            x: d,
            y: 0.0,
            z: 0.0,
        },
        q: QUAT_IDENTITY,
    };

    let q = compute_separating_axis(
        &hull_a.base,
        &hull_b.base,
        xf_b,
        SeparatingFeature::InvalidAxis,
    );

    assert_eq!(q.type_, SeparatingFeature::FaceAxisB);
    ensure_small(q.separation - gap, 1e-4);
    ensure_small(q.normal.x - 1.0, 1e-5);
    ensure_small(q.normal.y, 1e-5);
    ensure_small(q.normal.z, 1e-5);

    let planes_b = get_hull_planes(&hull_b.base);
    ensure_small(planes_b[q.index_b as usize].normal.x + 1.0, 1e-5);

    let points_a = get_hull_points(&hull_a.base);
    ensure_small(points_a[q.index_a as usize].x - a_extent, 1e-4);

    let (oracle_sep, _oracle_normal) = oracle_separation(&hull_a.base, &hull_b.base, xf_b);
    ensure_small(q.separation - oracle_sep, 1e-4);
    ensure_small(
        sep_along(&hull_a.base, &hull_b.base, xf_b, q.normal) - q.separation,
        1e-4,
    );
}

// Cube A yawed 45 about y presents an edge along y at x = h*root2. Cube B rolled 45 about z
// presents an edge along z at x = -h*root2. Sliding B along x makes those edges the closest
// features, so the axis is x and the separation is d - root2. The sweep straddles contact into
// overlap while staying under the speculative distance, so no face axis can short circuit the
// edge phase. The overlap rows pin the maximum separation, which is where a premature edge early
// out shows up. (EdgePairSweepTest)
#[test]
fn edge_pair_sweep_test() {
    let hull_a = make_transformed_box_hull(0.5, 0.5, 0.5, exact_rotation(AXIS_Y, 0.25 * PI));
    let hull_b = make_transformed_box_hull(0.5, 0.5, 0.5, exact_rotation(AXIS_Z, 0.25 * PI));

    let distances = [1.43, 1.42, ROOT2, 1.40, 1.38, 1.35, 1.30];

    for d in distances {
        let expected = d - ROOT2;

        let xf_b = Transform {
            p: Vec3 {
                x: d,
                y: 0.0,
                z: 0.0,
            },
            q: QUAT_IDENTITY,
        };
        let q = compute_separating_axis(
            &hull_a.base,
            &hull_b.base,
            xf_b,
            SeparatingFeature::InvalidAxis,
        );

        assert_eq!(q.type_, SeparatingFeature::EdgePairAxis);
        ensure_small(q.separation - expected, 1e-4);
        ensure_small(q.normal.x - 1.0, 1e-4);
        ensure_small(q.normal.y, 1e-4);
        ensure_small(q.normal.z, 1e-4);

        // Half edge indices are the even twin of each pair.
        assert_eq!(q.index_a & 1, 0);
        assert_eq!(q.index_b & 1, 0);

        let (oracle_sep, _oracle_normal) = oracle_separation(&hull_a.base, &hull_b.base, xf_b);
        ensure_small(q.separation - oracle_sep, 1e-4);
        ensure_small(
            sep_along(&hull_a.base, &hull_b.base, xf_b, q.normal) - q.separation,
            1e-4,
        );
    }
}

// A wide net over randomly oriented box pairs, from clearly separated to deep overlap. The brute
// force oracle is the source of truth. Whatever axis the query returns it must be a unit vector
// that reproduces its own separation, it must not exceed the true maximum, and it must not fall
// short of it. The short fall bound is what a suboptimal edge pick trips. Thin boxes are mixed in
// because a crossed pair of long edges stays the axis of minimum penetration even deep in overlap,
// which is the hardest case for the edge phase. (SeparatingAxisOracleTest)
#[test]
fn separating_axis_oracle_test() {
    let mut rng = Rng::new(987654321);

    let mut separated = 0;
    let mut penetrating = 0;
    let mut edge_wins = 0;

    for i in 0..4000 {
        let half_a;
        let half_b;
        let reach;

        if i & 1 != 0 {
            // Thin crossed beams driven into deep overlap.
            half_a = Vec3 {
                x: rng.next_float(1.0, 1.6),
                y: rng.next_float(0.08, 0.15),
                z: rng.next_float(0.08, 0.15),
            };
            half_b = Vec3 {
                x: rng.next_float(1.0, 1.6),
                y: rng.next_float(0.08, 0.15),
                z: rng.next_float(0.08, 0.15),
            };
            reach = rng.next_float(0.0, 0.4);
        } else {
            // Chunky boxes across the whole separated to overlapping range.
            half_a = Vec3 {
                x: rng.next_float(0.3, 0.8),
                y: rng.next_float(0.3, 0.8),
                z: rng.next_float(0.3, 0.8),
            };
            half_b = Vec3 {
                x: rng.next_float(0.3, 0.8),
                y: rng.next_float(0.3, 0.8),
                z: rng.next_float(0.3, 0.8),
            };
            reach = rng.next_float(0.0, 1.7);
        }

        // Rotate A in place so it stays centered on the origin, which keeps the support bias valid.
        let angle_a = rng.next_float(0.0, PI);
        let hull_a = make_transformed_box_hull(
            half_a.x,
            half_a.y,
            half_a.z,
            exact_rotation(rng.next_direction(), angle_a),
        );
        let hull_b = make_box_hull(half_b.x, half_b.y, half_b.z);

        let p = mul_sv(reach, rng.next_direction());
        let angle_b = rng.next_float(0.0, PI);
        let xf_b = Transform {
            p,
            q: exact_quat(rng.next_direction(), angle_b),
        };

        let q = compute_separating_axis(
            &hull_a.base,
            &hull_b.base,
            xf_b,
            SeparatingFeature::InvalidAxis,
        );

        let (oracle_sep, _oracle_normal) = oracle_separation(&hull_a.base, &hull_b.base, xf_b);

        ensure_small(length(q.normal) - 1.0, 1e-3);
        ensure_small(
            sep_along(&hull_a.base, &hull_b.base, xf_b, q.normal) - q.separation,
            2e-3,
        );

        // Never overstate the true maximum. This holds on every path since the reported axis is real.
        assert!(q.separation <= oracle_sep + 2e-3);

        // The scan stops at the first axis that clears the speculative band, so a widely separated
        // pair reports some separating axis rather than the deepest one. That is all the collide
        // path needs, since past the band it drops the pair either way. Inside the band nothing
        // stops the scan, so the result there is the exhaustive maximum, and that is what a
        // manifold gets built from.
        if q.separation <= speculative_distance() {
            // A suboptimal axis pick lands a discrete step below the true maximum. The bound sits
            // well under that step yet clear of the edge normalization noise floor near 6e-4.
            assert!(q.separation >= oracle_sep - 4e-3);
        }

        if oracle_sep > 0.0 {
            separated += 1;
        } else {
            penetrating += 1;
        }

        if q.type_ == SeparatingFeature::EdgePairAxis {
            edge_wins += 1;
        }
    }

    // The sweep has to cover both regimes and actually drive the edge path.
    assert!(separated > 100);
    assert!(penetrating > 100);
    assert!(edge_wins > 20);
}

// The oracle over hulls whose vertices sit far from their own local origin, which the AABB based
// bias must handle. Half the pairs offset A in its own frame, half offset B, and B is placed so
// the two centers land near each other across the separated to overlapping range. Any bias that
// assumed the origin was inside the hull would pick the wrong support and blow the separation.
// (OffsetHullOracleTest)
#[test]
fn offset_hull_oracle_test() {
    let mut rng = Rng::new(24681012);

    let mut separated = 0;
    let mut penetrating = 0;
    let mut edge_wins = 0;

    for i in 0..3000 {
        let half_a = Vec3 {
            x: rng.next_float(0.3, 0.8),
            y: rng.next_float(0.3, 0.8),
            z: rng.next_float(0.3, 0.8),
        };
        let half_b = Vec3 {
            x: rng.next_float(0.3, 0.8),
            y: rng.next_float(0.3, 0.8),
            z: rng.next_float(0.3, 0.8),
        };

        let offset = mul_sv(rng.next_float(2.0, 6.0), rng.next_direction());
        let reach = rng.next_float(0.0, 1.7);
        let dir = rng.next_direction();

        let hull_a;
        let hull_b;
        let xf_b;

        if i & 1 != 0 {
            // A carries the offset in its own frame. B is centered and dropped near A's center.
            let angle_a = rng.next_float(0.0, PI);
            let place_a = Transform {
                p: offset,
                q: exact_quat(rng.next_direction(), angle_a),
            };
            hull_a = make_transformed_box_hull(half_a.x, half_a.y, half_a.z, place_a);
            hull_b = make_box_hull(half_b.x, half_b.y, half_b.z);
            let angle_b = rng.next_float(0.0, PI);
            xf_b = Transform {
                p: mul_add(offset, reach, dir),
                q: exact_quat(rng.next_direction(), angle_b),
            };
        } else {
            // B carries the offset in its own frame. Place it so its world center lands near the origin.
            let angle_b = rng.next_float(0.0, PI);
            let q_b = exact_quat(rng.next_direction(), angle_b);
            hull_a = make_box_hull(half_a.x, half_a.y, half_a.z);
            hull_b = make_offset_box_hull(half_b.x, half_b.y, half_b.z, offset);
            xf_b = Transform {
                p: crate::math_functions::sub(mul_sv(reach, dir), rotate_vector(q_b, offset)),
                q: q_b,
            };
        }

        let q = compute_separating_axis(
            &hull_a.base,
            &hull_b.base,
            xf_b,
            SeparatingFeature::InvalidAxis,
        );

        let (oracle_sep, _oracle_normal) = oracle_separation(&hull_a.base, &hull_b.base, xf_b);

        let consistency =
            (sep_along(&hull_a.base, &hull_b.base, xf_b, q.normal) - q.separation).abs();

        // Consistency is the sharp bias check: a wrong support pick on an offset hull would throw
        // the returned normal off its own separation by a vertex spacing, not a noise floor. It
        // holds on every path, early out or not, so it stays unconditional. The oracle bounds are
        // looser since differencing coordinates out at radius six costs a few more digits.
        ensure_small(length(q.normal) - 1.0, 1e-3);
        assert!(consistency < 1e-3);
        assert!(q.separation <= oracle_sep + 3e-3);

        // Only the in band result feeds a manifold, and only there does the scan run to completion.
        if q.separation <= speculative_distance() {
            assert!(q.separation >= oracle_sep - 8e-3);
        }

        if oracle_sep > 0.0 {
            separated += 1;
        } else {
            penetrating += 1;
        }

        if q.type_ == SeparatingFeature::EdgePairAxis {
            edge_wins += 1;
        }
    }

    assert!(separated > 100);
    assert!(penetrating > 100);
    assert!(edge_wins > 20);
}
