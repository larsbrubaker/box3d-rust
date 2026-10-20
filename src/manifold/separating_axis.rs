//! SIMD separating axis test from `convex_manifold.c` (`b3ComputeSeparatingAxis`).
//!
//! This is the scalar (`B3_SIMD_NONE`) behavioral reference. The face phase runs
//! through the width-4 `FloatW` helpers from [`crate::simd_scalar`] exactly as the C
//! code does even in scalar mode (`b3GetSupportWide` / `b3NegativeTransformFromSoA`),
//! while the edge phase uses the purely scalar `#if defined( B3_SIMD_NONE )` branch.
//! The two must stay bit-identical to the wide path for cross platform determinism.
//!
//! SIMD separating axis test based on an implementation developed by Cairn Overturf.
//! See his article: <https://cairnc.github.io/posts/improvements-to-the-separating-axis/>
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{AxisQuery, SeparatingAxis, SeparatingFeature};
use crate::constants::{linear_slop, speculative_distance, PARALLEL_EDGE_TOL};
use crate::core::{NULL_INDEX, SIMD_WIDTH};
use crate::hull::{
    get_hull_edge_cosines, get_hull_edges, get_hull_planes, get_hull_soa_normals,
    get_hull_soa_vertices, hull_soa_normal_count, hull_soa_vertex_count, HullData,
    HULL_SOA_PADDING,
};
use crate::math_functions::{
    aabb_center, aabb_extents, abs, abs_float, add, dot, length, make_matrix_from_quat, max_float,
    min_float, mul_add, mul_mv, mul_sv, neg, sub, transpose, Matrix3, Transform, Vec3, VEC3_ZERO,
};
use crate::simd_scalar::{
    add_vw, dot_w, embed_index_w, load_vw, min_index_w, min_w, neg_vw, splat_vw, splat_w, store_vw,
    store_w, sub_w, zero_w, Vec3W,
};

use super::edge_candidates::{
    filter_edge_candidates, get_face_dots, get_face_plane_separations, round_up_to_simd_width,
    test_edge_candidate_sorted, EDGE_PROBE_MIN_TESTS,
};

const HULL_BIT_COUNT: i32 = 7;

/// Transform a SoA point/normal stream (already split into X/Y/Z) by `out = -(R*v (+t))`.
/// The inputs come straight from the hull's stored SoA arrays, so there's no transpose here.
/// `is_point` gates the translation add so it is only applied for points. (b3NegativeTransformFromSoA)
#[allow(clippy::too_many_arguments)]
fn negative_transform_from_soa(
    r: Matrix3,
    p: Vec3,
    in_x: &[f32],
    in_y: &[f32],
    in_z: &[f32],
    n: usize,
    out_x: &mut [f32],
    out_y: &mut [f32],
    out_z: &mut [f32],
    is_point: bool,
) {
    // row-column
    let r0 = Vec3W {
        x: splat_w(r.cx.x),
        y: splat_w(r.cy.x),
        z: splat_w(r.cz.x),
    };
    let r1 = Vec3W {
        x: splat_w(r.cx.y),
        y: splat_w(r.cy.y),
        z: splat_w(r.cz.y),
    };
    let r2 = Vec3W {
        x: splat_w(r.cx.z),
        y: splat_w(r.cy.z),
        z: splat_w(r.cz.z),
    };

    let mut t = Vec3W {
        x: zero_w(),
        y: zero_w(),
        z: zero_w(),
    };
    if is_point {
        t = splat_vw(p);
    }

    let mut i = 0;
    while i < n {
        let v = load_vw(&in_x[i..], &in_y[i..], &in_z[i..]);

        // Rotate one vector per lane
        let mut out = Vec3W {
            x: dot_w(r0, v),
            y: dot_w(r1, v),
            z: dot_w(r2, v),
        };

        if is_point {
            out = add_vw(out, t);
        }

        store_vw(
            &mut out_x[i..],
            &mut out_y[i..],
            &mut out_z[i..],
            neg_vw(out),
        );

        i += SIMD_WIDTH;
    }
}

/// SIMD support point calculation using a SoA vertex array padded with repeats of the first
/// vertex to a multiple of 8.
///
/// This minimizes `(bias - dot)`, where the caller is expected to provide a bias that makes
/// this always positive. It can be direction dependent. The bias should be just big enough to
/// ensure the value is positive because an excessive bias causes a precision loss in the
/// support calculation.
///
/// The vertex index is embedded in the low `HULL_BIT_COUNT` mantissa bits of the value. By
/// minimizing a value that is always positive, the minimum carries the smallest index so that
/// padded SoA values will never win. This is the purpose of using the bias instead of
/// maximizing the dot directly.
///
/// The support is then recomputed exactly as `dot(normal, vertex)`, without the embedded index.
/// (b3GetSupportWide)
fn get_support_wide(
    normal: Vec3,
    vx: &[f32],
    vy: &[f32],
    vz: &[f32],
    n: usize,
    bias: f32,
) -> (f32, i32) {
    let normal_w = splat_vw(normal);
    let bias_v = splat_w(bias);

    // Start the minimum at a large value.
    let mut min_value = splat_w(f32::INFINITY);

    // Tail lanes hold vertex 0 with index bits >= vertexCount, so they never become the min value.
    let mut i = 0;
    while i < n {
        let v = load_vw(&vx[i..], &vy[i..], &vz[i..]);
        let d = dot_w(normal_w, v);

        // This is always positive.
        let value = sub_w(bias_v, d);
        let augmented_value = embed_index_w(value, i as i32, HULL_BIT_COUNT);
        min_value = min_w(min_value, augmented_value);

        i += SIMD_WIDTH;
    }

    // One horizontal min, the winning lane's value and index bits ride through.
    let vi = min_index_w(min_value, HULL_BIT_COUNT);

    // Exact support for the chosen vertex.
    let support =
        normal.x * vx[vi as usize] + normal.y * vy[vi as usize] + normal.z * vz[vi as usize];
    (support, vi)
}

/// (b3GetFaceSeparation)
#[allow(clippy::too_many_arguments)]
fn get_face_separation(
    direction: Vec3,
    plane_separation: f32,
    vx: &[f32],
    vy: &[f32],
    vz: &[f32],
    n: usize,
    center: Vec3,
    extents: Vec3,
) -> (f32, i32) {
    let bias = dot(direction, center) + 1.0625 * dot(abs(direction), extents);
    let (support, vertex_index) = get_support_wide(direction, vx, vy, vz, n, bias);
    (plane_separation - support, vertex_index)
}

// Temporary abbreviations for convenience.
// `NE`/`NF`/`NV` in C: hull limit padded so a tail store of `SIMD_WIDTH` lanes stays in range.
const NE: usize = crate::constants::MAX_HULL_EDGES as usize + SIMD_WIDTH;
const NF: usize = crate::constants::MAX_HULL_FACES as usize + SIMD_WIDTH;
const NV: usize = crate::constants::MAX_HULL_VERTICES as usize + SIMD_WIDTH;

/// Separating axis test at the requested SIMD width. The scalar port only has the width 4
/// kernel; upstream without `B3_SIMD_HAS_WIDTH_8` ignores the width the same way.
/// (b3ComputeSeparatingAxisAtWidth)
pub(crate) fn compute_separating_axis_at_width(
    hull_a: &HullData,
    hull_b: &HullData,
    xf_b: Transform,
    early_return: bool,
    simd_width: i32,
) -> AxisQuery {
    // B3_UNUSED( simdWidth )
    let _ = simd_width;
    compute_separating_axis_w4(hull_a, hull_b, xf_b, early_return)
}

/// Separating axis test at the process wide SIMD width. (b3ComputeSeparatingAxis)
// Upstream's callers are the tests and the SAT benchmark; the solver path goes through
// `collide_hulls_at_width` with the world's width.
#[cfg_attr(not(test), allow(dead_code))]
pub(crate) fn compute_separating_axis(
    hull_a: &HullData,
    hull_b: &HullData,
    xf_b: Transform,
    early_return: bool,
) -> AxisQuery {
    compute_separating_axis_at_width(
        hull_a,
        hull_b,
        xf_b,
        early_return,
        crate::simd_scalar::get_simd_width(),
    )
}

/// SIMD separating axis test based on an implementation developed by Cairn Overturf.
/// Width 4 kernel (`convex_manifold_wide.inl` compiled by `wide4.c`).
/// (b3ComputeSeparatingAxisW4)
pub(crate) fn compute_separating_axis_w4(
    hull_a: &HullData,
    hull_b: &HullData,
    xf_b: Transform,
    early_return: bool,
) -> AxisQuery {
    let r = make_matrix_from_quat(xf_b.q);
    let inv_r = transpose(r);

    let speculative = speculative_distance();

    let mut res = AxisQuery {
        face_a: SeparatingAxis {
            normal: VEC3_ZERO,
            separation: f32::NEG_INFINITY,
            index_a: NULL_INDEX,
            index_b: NULL_INDEX,
            type_: SeparatingFeature::FaceAxisA,
        },
        face_b: SeparatingAxis {
            normal: VEC3_ZERO,
            separation: f32::NEG_INFINITY,
            index_a: NULL_INDEX,
            index_b: NULL_INDEX,
            type_: SeparatingFeature::FaceAxisB,
        },
        edge: SeparatingAxis {
            normal: VEC3_ZERO,
            separation: f32::NEG_INFINITY,
            index_a: NULL_INDEX,
            index_b: NULL_INDEX,
            type_: SeparatingFeature::EdgePairAxis,
        },
        separated_feature: SeparatingFeature::InvalidAxis,
    };

    let face_count_a = hull_a.face_count;
    let planes_a = get_hull_planes(hull_a);

    let vertex_stride_b = hull_soa_vertex_count(hull_b);
    let soa_b = get_hull_soa_vertices(hull_b);
    let vx_b = &soa_b[0..vertex_stride_b];
    let vy_b = &soa_b[vertex_stride_b..2 * vertex_stride_b];
    let vz_b = &soa_b[2 * vertex_stride_b..3 * vertex_stride_b];
    let wide_vertex_count_b = round_up_to_simd_width(hull_b.vertex_count) as usize;

    let c_b = aabb_center(hull_b.aabb);
    let h_b = aabb_extents(hull_b.aabb);

    // The hulls have a precomputed inner radius and centroid.
    // A given axis cannot achieve a separation larger than:
    // dot(axis, centerB - centerA) - innerRadiusA - innerRadiusB
    // So this is the upper bound for the separation of a candidate axis.
    // An axis can be skipped if it has an upper bound that is less than the current
    // best separation. This lets me skip many of the candidates without computing
    // support points.

    let delta_center = sub(add(mul_mv(r, hull_b.center), xf_b.p), hull_a.center);
    let center_distance = length(delta_center);
    let radius = hull_a.inner_radius + hull_b.inner_radius;

    // Adjust the radius to ensure the best axis isn't skipped.
    let radius_bound = radius - (linear_slop() + 0.001 * (center_distance + radius));

    // Compute dot(normalA, centerDelta) for all face normals of hullA.
    let mut dot_a = [0.0f32; NF];
    get_face_dots(hull_a, delta_center, &mut dot_a);

    // Find the face of hullA that most aligns with centerDelta.
    let mut seed_index_a = 0usize;
    let mut max_dot_a = dot_a[0];
    for (i, &d) in dot_a.iter().enumerate().take(face_count_a as usize).skip(1) {
        if d > max_dot_a {
            max_dot_a = d;
            seed_index_a = i;
        }
    }

    // Use the seed to get a lower bound on the separation for the faces of hullA.
    let mut floor_a = f32::NEG_INFINITY;
    let mut seed_separation_a = f32::NEG_INFINITY;
    let mut seed_vertex_b = 0i32;
    if early_return {
        let plane = planes_a[seed_index_a];
        let direction = neg(mul_mv(inv_r, plane.normal));
        let plane_separation = dot(plane.normal, xf_b.p) - plane.offset;
        (seed_separation_a, seed_vertex_b) = get_face_separation(
            direction,
            plane_separation,
            vx_b,
            vy_b,
            vz_b,
            wide_vertex_count_b,
            c_b,
            h_b,
        );
        floor_a = min_float(seed_separation_a, speculative);
    }

    // Test A's face planes against B's vertices.
    for i in 0..face_count_a {
        // The bound offset ensures the seed will be evaluated.
        if dot_a[i as usize] - radius_bound < max_float(floor_a, res.face_a.separation) {
            continue;
        }

        let plane = planes_a[i as usize];
        let mut vertex_index = seed_vertex_b;
        let mut separation = seed_separation_a;

        // Avoid recomputing the seed face separation.
        if !early_return || i as usize != seed_index_a {
            let direction = neg(mul_mv(inv_r, plane.normal));
            let plane_separation = dot(plane.normal, xf_b.p) - plane.offset;
            (separation, vertex_index) = get_face_separation(
                direction,
                plane_separation,
                vx_b,
                vy_b,
                vz_b,
                wide_vertex_count_b,
                c_b,
                h_b,
            );
        }

        if separation > res.face_a.separation {
            res.face_a.normal = plane.normal;
            res.face_a.separation = separation;
            res.face_a.index_a = i;
            res.face_a.index_b = vertex_index;
            if separation > speculative && early_return {
                res.separated_feature = SeparatingFeature::FaceAxisA;
                return res;
            }
        }
    }

    debug_assert!(res.face_a.index_a != NULL_INDEX);

    let face_count_b = hull_b.face_count;
    let planes_b = get_hull_planes(hull_b);

    let vertex_stride_a = hull_soa_vertex_count(hull_a);
    let soa_a = get_hull_soa_vertices(hull_a);
    let vx_a = &soa_a[0..vertex_stride_a];
    let vy_a = &soa_a[vertex_stride_a..2 * vertex_stride_a];
    let vz_a = &soa_a[2 * vertex_stride_a..3 * vertex_stride_a];
    let wide_vertex_count_a = round_up_to_simd_width(hull_a.vertex_count) as usize;

    let c_a = aabb_center(hull_a.aabb);
    let h_a = aabb_extents(hull_a.aabb);

    // Similarly, find the face of hullB that most aligns with the vector pointing from centerB to centerA.
    let mut dot_b = [0.0f32; NF];
    get_face_dots(hull_b, neg(mul_mv(inv_r, delta_center)), &mut dot_b);

    let mut seed_index_b = 0usize;
    let mut max_dot_b = dot_b[0];
    for (i, &d) in dot_b.iter().enumerate().take(face_count_b as usize).skip(1) {
        if d > max_dot_b {
            max_dot_b = d;
            seed_index_b = i;
        }
    }

    // Get a lower bound on the separation for the faces of hullB.
    let mut floor_b = f32::NEG_INFINITY;
    let mut seed_separation_b = f32::NEG_INFINITY;
    let mut seed_vertex_a = 0i32;
    if early_return {
        let plane = planes_b[seed_index_b];
        let direction = neg(mul_mv(r, plane.normal));
        let plane_separation = dot(direction, xf_b.p) - plane.offset;
        (seed_separation_b, seed_vertex_a) = get_face_separation(
            direction,
            plane_separation,
            vx_a,
            vy_a,
            vz_a,
            wide_vertex_count_a,
            c_a,
            h_a,
        );

        // Include the floor set by hull A faces.
        floor_b = max_float(seed_separation_b, res.face_a.separation);
        floor_b = min_float(floor_b, speculative);
    }

    // Test B's face planes against A's vertices.
    for i in 0..face_count_b {
        if dot_b[i as usize] - radius_bound < max_float(floor_b, res.face_b.separation) {
            continue;
        }

        let plane = planes_b[i as usize];
        let direction = neg(mul_mv(r, plane.normal));
        let mut vertex_index = seed_vertex_a;
        let mut separation = seed_separation_b;

        // Avoid recomputing the seed face separation.
        if !early_return || i as usize != seed_index_b {
            let plane_separation = dot(direction, xf_b.p) - plane.offset;
            (separation, vertex_index) = get_face_separation(
                direction,
                plane_separation,
                vx_a,
                vy_a,
                vz_a,
                wide_vertex_count_a,
                c_a,
                h_a,
            );
        }

        if separation > res.face_b.separation {
            res.face_b.normal = direction;
            res.face_b.separation = separation;
            res.face_b.index_a = vertex_index;
            res.face_b.index_b = i;
            if separation > speculative && early_return {
                res.separated_feature = SeparatingFeature::FaceAxisB;
                return res;
            }
        }
    }

    // Transform B into A's space once, into SoA arrays. Extra space so tail can be set to zero.
    const _: () = assert!(
        (crate::constants::MAX_HULL_EDGES as usize & (SIMD_WIDTH - 1)) == 0,
        "must be multiple of SIMD width"
    );
    const _: () = assert!(
        (crate::constants::MAX_HULL_FACES & (HULL_SOA_PADDING - 1)) == 0,
        "must be multiple of hull SoA padding"
    );
    const _: () = assert!(
        (crate::constants::MAX_HULL_VERTICES & (HULL_SOA_PADDING - 1)) == 0,
        "must be multiple of hull SoA padding"
    );

    debug_assert!(
        !early_return
            || center_distance
                >= max_float(res.face_a.separation, res.face_b.separation) + radius_bound
    );

    // Gather edges of A that can feasibly create a separating axis that beats the maximum face separation.
    let half_edge_count_a = hull_a.edge_count;
    let half_edges_a = get_hull_edges(hull_a);
    let mut edge_indices_a = [0i32; NE];
    let mut na = 0usize;

    let half_edge_count_b = hull_b.edge_count;
    let half_edges_b = get_hull_edges(hull_b);
    let mut edge_indices_b = [0i32; crate::constants::MAX_HULL_EDGES as usize];
    let mut nb = 0usize;

    let max_face_separation = max_float(res.face_a.separation, res.face_b.separation);
    let bound_slack = radius - radius_bound;
    let threshold_a = if early_return {
        max_face_separation + hull_b.inner_radius - bound_slack
    } else {
        f32::NEG_INFINITY
    };
    let threshold_b = if early_return {
        max_face_separation + hull_a.inner_radius - bound_slack
    } else {
        f32::NEG_INFINITY
    };

    let center_b_in_a = add(mul_mv(r, hull_b.center), xf_b.p);
    let center_a_in_b = mul_mv(inv_r, sub(hull_a.center, xf_b.p));

    let mut plane_dot_a = [0.0f32; NF];
    let mut plane_dot_b = [0.0f32; NF];
    get_face_plane_separations(hull_a, center_b_in_a, &mut plane_dot_a);
    get_face_plane_separations(hull_b, center_a_in_b, &mut plane_dot_b);

    let cosines_a = get_hull_edge_cosines(hull_a);
    let cosines_b = get_hull_edge_cosines(hull_b);

    let mut i = 0;
    while i < half_edge_count_a {
        let i1 = half_edges_a[i as usize].face as usize;
        let i2 = half_edges_a[(i + 1) as usize].face as usize;
        let c = cosines_a[(i >> 1) as usize];
        edge_indices_a[na] = i;
        na += test_edge_candidate_sorted(plane_dot_a[i1], plane_dot_a[i2], c, threshold_a);
        i += 2;
    }

    let mut i = 0;
    while i < half_edge_count_b {
        let i1 = half_edges_b[i as usize].face as usize;
        let i2 = half_edges_b[(i + 1) as usize].face as usize;
        let c = cosines_b[(i >> 1) as usize];
        edge_indices_b[nb] = i;
        nb += test_edge_candidate_sorted(plane_dot_b[i1], plane_dot_b[i2], c, threshold_b);
        i += 2;
    }

    // Apply additional culling use the support points for the best face axes.
    // The support vertices of the best faces are points on the other hull, so an edge pair axis cannot
    // have a larger separation than the plane separation of these points over the arc of the edge.
    // This can slow down boxes so skip this if there are not enough edge candidates.
    if early_return && nb * ((na + 3) >> 2) >= EDGE_PROBE_MIN_TESTS {
        let probe_bound = max_face_separation
            - (0.1 * linear_slop() + 0.001 * abs_float(center_distance + radius));

        let vertex_b = if res.face_a.index_b != NULL_INDEX {
            res.face_a.index_b
        } else {
            seed_vertex_b
        } as usize;
        let probe_b = Vec3 {
            x: vx_b[vertex_b],
            y: vy_b[vertex_b],
            z: vz_b[vertex_b],
        };
        get_face_plane_separations(hull_a, add(mul_mv(r, probe_b), xf_b.p), &mut plane_dot_a);
        na = filter_edge_candidates(hull_a, &plane_dot_a, probe_bound, &mut edge_indices_a, na);

        let vertex_a = if res.face_b.index_a != NULL_INDEX {
            res.face_b.index_a
        } else {
            seed_vertex_a
        } as usize;
        let probe_a = Vec3 {
            x: vx_a[vertex_a],
            y: vy_a[vertex_a],
            z: vz_a[vertex_a],
        };
        get_face_plane_separations(
            hull_b,
            mul_mv(inv_r, sub(probe_a, xf_b.p)),
            &mut plane_dot_b,
        );
        nb = filter_edge_candidates(hull_b, &plane_dot_b, probe_bound, &mut edge_indices_b, nb);
    }

    if na == 0 || nb == 0 {
        // No edge candidates found.
        return res;
    }

    // The alignments below are not necessary, but they don't hurt.

    // B face normals in A space, negated.
    let mut b_fnx = [0.0f32; NF];
    let mut b_fny = [0.0f32; NF];
    let mut b_fnz = [0.0f32; NF];

    // B vertices in A space, negated.
    let mut b_wx = [0.0f32; NV];
    let mut b_wy = [0.0f32; NV];
    let mut b_wz = [0.0f32; NV];

    let face_stride_b = hull_soa_normal_count(hull_b);
    let normals_b = get_hull_soa_normals(hull_b);
    let nx_b = &normals_b[0..face_stride_b];
    let ny_b = &normals_b[face_stride_b..2 * face_stride_b];
    let nz_b = &normals_b[2 * face_stride_b..3 * face_stride_b];
    let wide_face_count_b = round_up_to_simd_width(face_count_b) as usize;

    negative_transform_from_soa(
        r,
        xf_b.p,
        nx_b,
        ny_b,
        nz_b,
        wide_face_count_b,
        &mut b_fnx,
        &mut b_fny,
        &mut b_fnz,
        false,
    );
    negative_transform_from_soa(
        r,
        xf_b.p,
        vx_b,
        vy_b,
        vz_b,
        wide_vertex_count_b,
        &mut b_wx,
        &mut b_wy,
        &mut b_wz,
        true,
    );

    // Per A edge data, already in A's space so just gathered. n0 and n1 are the two face
    // normals, d the edge vector av1-av0, v0 the first vertex. Tol is the
    // parallel edge tolerance, scaled by the edge length.
    let mut a_n0x = [0.0f32; NE];
    let mut a_n0y = [0.0f32; NE];
    let mut a_n0z = [0.0f32; NE];
    let mut a_n1x = [0.0f32; NE];
    let mut a_n1y = [0.0f32; NE];
    let mut a_n1z = [0.0f32; NE];
    // dir = av1 - av0
    let mut a_dx = [0.0f32; NE];
    let mut a_dy = [0.0f32; NE];
    let mut a_dz = [0.0f32; NE];
    let mut a_v0x = [0.0f32; NE];
    let mut a_v0y = [0.0f32; NE];
    let mut a_v0z = [0.0f32; NE];
    let mut a_tol = [0.0f32; NE];

    let squared_tol = PARALLEL_EDGE_TOL * PARALLEL_EDGE_TOL;
    for k in 0..na {
        let edge = &half_edges_a[edge_indices_a[k] as usize];
        let twin = &half_edges_a[(edge_indices_a[k] + 1) as usize];

        let a = planes_a[edge.face as usize].normal;
        let b = planes_a[twin.face as usize].normal;
        a_n0x[k] = a.x;
        a_n0y[k] = a.y;
        a_n0z[k] = a.z;
        a_n1x[k] = b.x;
        a_n1y[k] = b.y;
        a_n1z[k] = b.z;

        let v0 = edge.origin as usize;
        let v1 = twin.origin as usize;

        a_dx[k] = vx_a[v1] - vx_a[v0];
        a_dy[k] = vy_a[v1] - vy_a[v0];
        a_dz[k] = vz_a[v1] - vz_a[v0];
        a_v0x[k] = vx_a[v0];
        a_v0y[k] = vy_a[v0];
        a_v0z[k] = vz_a[v0];

        a_tol[k] = squared_tol * (a_dx[k] * a_dx[k] + a_dy[k] * a_dy[k] + a_dz[k] * a_dz[k]);
    }

    // Zero the tail lanes.
    let zero = zero_w();
    let zero_v = Vec3W {
        x: zero,
        y: zero,
        z: zero,
    };
    store_vw(&mut a_n0x[na..], &mut a_n0y[na..], &mut a_n0z[na..], zero_v);
    store_vw(&mut a_n1x[na..], &mut a_n1y[na..], &mut a_n1z[na..], zero_v);
    store_vw(&mut a_dx[na..], &mut a_dy[na..], &mut a_dz[na..], zero_v);
    store_vw(&mut a_v0x[na..], &mut a_v0y[na..], &mut a_v0z[na..], zero_v);
    store_w(&mut a_tol[na..], zero);

    let slop = linear_slop();

    // The SIMD emulated version of this code is very slow. This is a purely scalar version
    // for platforms that don't have SIMD capability. It is much faster than SIMD emulation.
    // WARNING: this math needs to match the SIMD version for cross platform determinism.

    let eps = -slop * slop;

    for j in 0..nb {
        let edge = &half_edges_b[edge_indices_b[j] as usize];
        let twin = &half_edges_b[(edge_indices_b[j] + 1) as usize];
        let f0 = edge.face as usize;
        let f1 = twin.face as usize;
        let v0 = edge.origin as usize;
        let v1 = twin.origin as usize;

        let c = Vec3 {
            x: b_fnx[f0],
            y: b_fny[f0],
            z: b_fnz[f0],
        };
        let d_n = Vec3 {
            x: b_fnx[f1],
            y: b_fny[f1],
            z: b_fnz[f1],
        };
        let bv0 = Vec3 {
            x: b_wx[v0],
            y: b_wy[v0],
            z: b_wz[v0],
        };
        let bv1 = Vec3 {
            x: b_wx[v1],
            y: b_wy[v1],
            z: b_wz[v1],
        };
        let dc = sub(bv1, bv0);

        for i in 0..na {
            let d = Vec3 {
                x: a_dx[i],
                y: a_dy[i],
                z: a_dz[i],
            };

            // CBA = C.dir, DBA = D.dir, where dir = B_x_A
            let cba = dot(c, d);
            let dba = dot(d_n, d);
            if cba * dba >= eps {
                continue;
            }

            let n0 = Vec3 {
                x: a_n0x[i],
                y: a_n0y[i],
                z: a_n0z[i],
            };
            let n1 = Vec3 {
                x: a_n1x[i],
                y: a_n1y[i],
                z: a_n1z[i],
            };

            // ADC = n0.DC, BDC = n1.DC, where DC = D_x_C
            let adc = dot(n0, dc);
            let bdc = dot(n1, dc);
            if adc * bdc >= eps || cba * bdc >= eps {
                continue;
            }

            // Reject near parallel edges
            let max_cd = max_float(cba * cba, dba * dba);
            if max_cd <= a_tol[i] {
                continue;
            }

            // t = -CBA / (DBA - CBA)
            let t = -cba / (dba - cba);

            // normal = lerp(t, C, D) = C + (D-C)*t
            let mut n = mul_add(c, t, sub(d_n, c));
            let len2 = dot(n, n);
            let inv = 1.0 / len2.sqrt();
            n = mul_sv(inv, n);

            // separation = -dot(normal, av0 + bv0)
            let av0 = Vec3 {
                x: a_v0x[i],
                y: a_v0y[i],
                z: a_v0z[i],
            };
            let separation = -dot(add(av0, bv0), n);
            if separation > res.edge.separation {
                res.edge.normal = n;
                res.edge.separation = separation;

                // Half edge index
                res.edge.index_a = edge_indices_a[i];
                res.edge.index_b = edge_indices_b[j];

                if separation > speculative && early_return {
                    res.separated_feature = SeparatingFeature::EdgePairAxis;
                    return res;
                }
            }
        }
    }

    res
}
