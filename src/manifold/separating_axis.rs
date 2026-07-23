//! SIMD separating axis test from `convex_manifold.c` (`b3ComputeSeparatingAxis`).
//!
//! This is the scalar (`B3_SIMD_NONE`) behavioral reference. The face phase runs
//! through the width-4 `FloatW` helpers from [`super::simd_scalar`] exactly as the C
//! code does even in scalar mode (`b3GetSupportWide` / `b3NegativeTransformFromSoA`),
//! while the edge phase uses the purely scalar `#if defined( B3_SIMD_NONE )` branch.
//! The two must stay bit-identical to the wide path for cross platform determinism.
//!
//! SIMD separating axis test based on an implementation developed by Cairn Overturf.
//! See his article: <https://cairno.substack.com/p/improvements-to-the-separating-axis>
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::simd_scalar::{
    add_w, dot3_w, embed_index_w, load_w, min_index_w, min_w, mul_w, neg_w, splat_w, store_w,
    sub_w, zero_w,
};
use super::types::{AxisQuery, SeparatingFeature};
use crate::constants::{huge, linear_slop, speculative_distance, PARALLEL_EDGE_TOL};
use crate::core::NULL_INDEX;
use crate::hull::{
    get_hull_edges, get_hull_planes, get_hull_soa_normals, get_hull_soa_vertices,
    hull_soa_normal_count, hull_soa_vertex_count, HullData,
};
use crate::math_functions::{
    aabb_center, aabb_extents, abs, dot, make_matrix_from_quat, max_float, mul_mv, neg, transpose,
    Matrix3, Transform, Vec3, VEC3_ZERO,
};

const HULL_BIT_COUNT: i32 = 7;

/// `NE`/`NF`/`NV` in C: hull limit padded so a tail store of 4 lanes stays in range.
const NE: usize = crate::constants::MAX_HULL_EDGES as usize + 4;
const NF: usize = crate::constants::MAX_HULL_FACES as usize + 4;
const NV: usize = crate::constants::MAX_HULL_VERTICES as usize + 4;

/// Transform a SoA point/normal stream (already split into X/Y/Z) by `out = -(R*v (+t))`.
/// The inputs come straight from the hull's stored SoA arrays, so there's no transpose here.
/// `is_point` gates the translation add so it is only applied for points. (b3NegativeTransformFromSoA)
#[allow(clippy::too_many_arguments)]
fn negative_transform_from_soa(
    r: &Matrix3,
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
    let r00 = splat_w(r.cx.x);
    let r01 = splat_w(r.cy.x);
    let r02 = splat_w(r.cz.x);
    let r10 = splat_w(r.cx.y);
    let r11 = splat_w(r.cy.y);
    let r12 = splat_w(r.cz.y);
    let r20 = splat_w(r.cx.z);
    let r21 = splat_w(r.cy.z);
    let r22 = splat_w(r.cz.z);

    let mut tx = zero_w();
    let mut ty = zero_w();
    let mut tz = zero_w();

    if is_point {
        tx = splat_w(p.x);
        ty = splat_w(p.y);
        tz = splat_w(p.z);
    }

    let mut i = 0;
    while i < n {
        let x = load_w(&in_x[i..]);
        let y = load_w(&in_y[i..]);
        let z = load_w(&in_z[i..]);

        // Rotate four vectors at a time
        let mut ox = dot3_w(r00, r01, r02, x, y, z);
        let mut oy = dot3_w(r10, r11, r12, x, y, z);
        let mut oz = dot3_w(r20, r21, r22, x, y, z);

        if is_point {
            ox = add_w(ox, tx);
            oy = add_w(oy, ty);
            oz = add_w(oz, tz);
        }

        store_w(&mut out_x[i..], neg_w(ox));
        store_w(&mut out_y[i..], neg_w(oy));
        store_w(&mut out_z[i..], neg_w(oz));

        i += 4;
    }
}

/// SIMD support point calculation using a SoA vertex array padded with repeats of the first
/// vertex to a multiple of 4.
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
    let nx = splat_w(normal.x);
    let ny = splat_w(normal.y);
    let nz = splat_w(normal.z);
    let bias_v = splat_w(bias);

    // Start the minimum at a large value.
    let mut min_value = splat_w(huge());

    // Tail lanes hold vertex 0 with index bits >= vertexCount, so they never become the min value.
    let mut i = 0;
    while i < n {
        let x = load_w(&vx[i..]);
        let y = load_w(&vy[i..]);
        let z = load_w(&vz[i..]);
        let d = add_w(mul_w(nz, z), add_w(mul_w(ny, y), mul_w(nx, x)));

        // This is always positive.
        let value = sub_w(bias_v, d);
        let augmented_value = embed_index_w(value, i as i32, HULL_BIT_COUNT);
        min_value = min_w(min_value, augmented_value);

        i += 4;
    }

    // One horizontal min, the winning lane's value and index bits ride through.
    let vi = min_index_w(min_value, HULL_BIT_COUNT);

    // Exact support for the chosen vertex.
    let support =
        normal.x * vx[vi as usize] + normal.y * vy[vi as usize] + normal.z * vz[vi as usize];
    (support, vi)
}

/// SIMD separating axis test based on an implementation developed by Cairn Overturf.
/// (b3ComputeSeparatingAxis)
pub(crate) fn compute_separating_axis(
    hull_a: &HullData,
    hull_b: &HullData,
    xf_b: Transform,
    axis_override: SeparatingFeature,
) -> AxisQuery {
    debug_assert!(
        axis_override == SeparatingFeature::InvalidAxis
            || axis_override == SeparatingFeature::ManualFaceAxisA
            || axis_override == SeparatingFeature::ManualFaceAxisB
            || axis_override == SeparatingFeature::ManualEdgePairAxis
    );

    let r = make_matrix_from_quat(xf_b.q);
    let inv_r = transpose(r);

    let speculative = speculative_distance();

    let mut res = AxisQuery {
        normal: VEC3_ZERO,
        separation: f32::NEG_INFINITY,
        index_a: NULL_INDEX,
        index_b: NULL_INDEX,
        type_: SeparatingFeature::InvalidAxis,
    };

    let face_count_a = hull_a.face_count;
    let planes_a = get_hull_planes(hull_a);

    let soa_vertex_count_b = hull_soa_vertex_count(hull_b);
    let soa_b = get_hull_soa_vertices(hull_b);
    let vx_b = &soa_b[0..soa_vertex_count_b];
    let vy_b = &soa_b[soa_vertex_count_b..2 * soa_vertex_count_b];
    let vz_b = &soa_b[2 * soa_vertex_count_b..3 * soa_vertex_count_b];

    let c_b = aabb_center(hull_b.aabb);
    let h_b = aabb_extents(hull_b.aabb);

    // Test A's face planes against B's vertices.
    if axis_override != SeparatingFeature::ManualFaceAxisB
        && axis_override != SeparatingFeature::ManualEdgePairAxis
    {
        for i in 0..face_count_a {
            let plane = planes_a[i as usize];
            let direction = neg(mul_mv(inv_r, plane.normal));
            let plane_separation = dot(plane.normal, xf_b.p) - plane.offset;
            let bias_b = dot(direction, c_b) + 1.0625 * dot(abs(direction), h_b);
            let (support, vertex_index) =
                get_support_wide(direction, vx_b, vy_b, vz_b, soa_vertex_count_b, bias_b);
            let separation = plane_separation - support;
            if separation > res.separation {
                res.type_ = SeparatingFeature::FaceAxisA;
                res.separation = separation;
                res.index_a = i;
                res.index_b = vertex_index;
                res.normal = plane.normal;
                if separation > speculative {
                    return res;
                }
            }
        }
    }

    if axis_override == SeparatingFeature::ManualFaceAxisA {
        return res;
    }

    let face_count_b = hull_b.face_count;
    let planes_b = get_hull_planes(hull_b);

    let soa_vertex_count_a = hull_soa_vertex_count(hull_a);
    let soa_a = get_hull_soa_vertices(hull_a);
    let vx_a = &soa_a[0..soa_vertex_count_a];
    let vy_a = &soa_a[soa_vertex_count_a..2 * soa_vertex_count_a];
    let vz_a = &soa_a[2 * soa_vertex_count_a..3 * soa_vertex_count_a];

    let c_a = aabb_center(hull_a.aabb);
    let h_a = aabb_extents(hull_a.aabb);

    // Test B's face planes against A's vertices.
    if axis_override != SeparatingFeature::ManualEdgePairAxis {
        for i in 0..face_count_b {
            let plane = planes_b[i as usize];
            let direction = neg(mul_mv(r, plane.normal));
            let plane_separation = dot(direction, xf_b.p) - plane.offset;
            let bias_a = dot(direction, c_a) + 1.0625 * dot(abs(direction), h_a);
            let (support, vertex_index) =
                get_support_wide(direction, vx_a, vy_a, vz_a, soa_vertex_count_a, bias_a);
            let separation = plane_separation - support;
            if separation > res.separation {
                res.type_ = SeparatingFeature::FaceAxisB;
                res.separation = separation;
                res.index_a = vertex_index;
                res.index_b = i;
                // This points from A to B and is in frame A
                res.normal = direction;
                if separation > speculative {
                    return res;
                }
            }
        }
    }

    if axis_override == SeparatingFeature::ManualFaceAxisB {
        return res;
    }

    // Transform B into A's space once, into SoA arrays. Extra space so
    // tail can be set to zero in all cases.

    // The alignments below are not necessary, but they don't hurt.

    // B face normals in A space, negated.
    let mut b_fnx = [0.0f32; NF];
    let mut b_fny = [0.0f32; NF];
    let mut b_fnz = [0.0f32; NF];

    // B vertices in A space, negated.
    let mut b_wx = [0.0f32; NV];
    let mut b_wy = [0.0f32; NV];
    let mut b_wz = [0.0f32; NV];

    let soa_face_count_b = hull_soa_normal_count(hull_b);
    let normals_b = get_hull_soa_normals(hull_b);
    let nx_b = &normals_b[0..soa_face_count_b];
    let ny_b = &normals_b[soa_face_count_b..2 * soa_face_count_b];
    let nz_b = &normals_b[2 * soa_face_count_b..3 * soa_face_count_b];

    negative_transform_from_soa(
        &r,
        xf_b.p,
        nx_b,
        ny_b,
        nz_b,
        soa_face_count_b,
        &mut b_fnx,
        &mut b_fny,
        &mut b_fnz,
        false,
    );
    negative_transform_from_soa(
        &r,
        xf_b.p,
        vx_b,
        vy_b,
        vz_b,
        soa_vertex_count_b,
        &mut b_wx,
        &mut b_wy,
        &mut b_wz,
        true,
    );

    // Per B edge data. C and D are the two face normals, v0 a vertex, DC the edge vector.
    let mut b_cx = [0.0f32; NE];
    let mut b_cy = [0.0f32; NE];
    let mut b_cz = [0.0f32; NE];
    let mut b_dx = [0.0f32; NE];
    let mut b_dy = [0.0f32; NE];
    let mut b_dz = [0.0f32; NE];
    let mut b_v0x = [0.0f32; NE];
    let mut b_v0y = [0.0f32; NE];
    let mut b_v0z = [0.0f32; NE];
    let mut b_dcx = [0.0f32; NE];
    let mut b_dcy = [0.0f32; NE];
    let mut b_dcz = [0.0f32; NE];

    let half_edge_count_b = hull_b.edge_count;
    let half_edges_b = get_hull_edges(hull_b);
    let mut nb = 0usize;
    let mut i = 0;
    while i < half_edge_count_b {
        let edge = &half_edges_b[i as usize];
        let twin = &half_edges_b[(i + 1) as usize];
        let f0 = edge.face as usize;
        let f1 = twin.face as usize;
        let v0 = edge.origin as usize;
        let v1 = twin.origin as usize;

        b_cx[nb] = b_fnx[f0];
        b_cy[nb] = b_fny[f0];
        b_cz[nb] = b_fnz[f0];
        b_dx[nb] = b_fnx[f1];
        b_dy[nb] = b_fny[f1];
        b_dz[nb] = b_fnz[f1];
        b_v0x[nb] = b_wx[v0];
        b_v0y[nb] = b_wy[v0];
        b_v0z[nb] = b_wz[v0];
        b_dcx[nb] = b_wx[v1] - b_wx[v0];
        b_dcy[nb] = b_wy[v1] - b_wy[v0];
        b_dcz[nb] = b_wz[v1] - b_wz[v0];
        nb += 1;

        i += 2;
    }
    let _ = nb;

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

    let half_edge_count_a = hull_a.edge_count;
    let half_edges_a = get_hull_edges(hull_a);
    let mut na = 0usize;

    let squared_tol = PARALLEL_EDGE_TOL * PARALLEL_EDGE_TOL;
    let mut i = 0;
    while i < half_edge_count_a {
        let edge = &half_edges_a[i as usize];
        let twin = &half_edges_a[(i + 1) as usize];

        let a = planes_a[edge.face as usize].normal;
        let b = planes_a[twin.face as usize].normal;
        a_n0x[na] = a.x;
        a_n0y[na] = a.y;
        a_n0z[na] = a.z;
        a_n1x[na] = b.x;
        a_n1y[na] = b.y;
        a_n1z[na] = b.z;

        let v0 = edge.origin as usize;
        let v1 = twin.origin as usize;

        a_dx[na] = vx_a[v1] - vx_a[v0];
        a_dy[na] = vy_a[v1] - vy_a[v0];
        a_dz[na] = vz_a[v1] - vz_a[v0];
        a_v0x[na] = vx_a[v0];
        a_v0y[na] = vy_a[v0];
        a_v0z[na] = vz_a[v0];

        a_tol[na] =
            squared_tol * (a_dx[na] * a_dx[na] + a_dy[na] * a_dy[na] + a_dz[na] * a_dz[na]);
        na += 1;

        i += 2;
    }

    // Zero the tail lanes.
    let zero = zero_w();
    store_w(&mut a_n0x[na..], zero);
    store_w(&mut a_n0y[na..], zero);
    store_w(&mut a_n0z[na..], zero);
    store_w(&mut a_n1x[na..], zero);
    store_w(&mut a_n1y[na..], zero);
    store_w(&mut a_n1z[na..], zero);
    store_w(&mut a_dx[na..], zero);
    store_w(&mut a_dy[na..], zero);
    store_w(&mut a_dz[na..], zero);
    store_w(&mut a_v0x[na..], zero);
    store_w(&mut a_v0y[na..], zero);
    store_w(&mut a_v0z[na..], zero);
    store_w(&mut a_tol[na..], zero);

    // Prefer face contact for more contact points.
    let mut abs_face_bias = 0.1 * linear_slop();

    let edge_count_b = (half_edge_count_b / 2) as usize;

    // The SIMD emulated version of this code is very slow. This is a purely scalar version
    // for platforms that don't have SIMD capability. It is much faster than SIMD emulation.
    // WARNING: this math needs to match the SIMD version for cross platform determinism.

    const EPS: f32 = -0.0001;

    for j in 0..edge_count_b {
        let cx = b_cx[j];
        let cy = b_cy[j];
        let cz = b_cz[j];
        let dx = b_dx[j];
        let dy = b_dy[j];
        let dz = b_dz[j];
        let dcx = b_dcx[j];
        let dcy = b_dcy[j];
        let dcz = b_dcz[j];
        let bv0x = b_v0x[j];
        let bv0y = b_v0y[j];
        let bv0z = b_v0z[j];

        for i in 0..na {
            // CBA = C.dir, DBA = D.dir, where dir = B_x_A
            let cba = cx * a_dx[i] + (cy * a_dy[i] + cz * a_dz[i]);
            let dba = dx * a_dx[i] + (dy * a_dy[i] + dz * a_dz[i]);
            if cba * dba >= EPS {
                continue;
            }

            // ADC = n0.DC, BDC = n1.DC, where DC = D_x_C
            let adc = a_n0x[i] * dcx + (a_n0y[i] * dcy + a_n0z[i] * dcz);
            let bdc = a_n1x[i] * dcx + (a_n1y[i] * dcy + a_n1z[i] * dcz);
            if adc * bdc >= EPS || cba * bdc >= EPS {
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
            let mut nx = cx + t * (dx - cx);
            let mut ny = cy + t * (dy - cy);
            let mut nz = cz + t * (dz - cz);
            let len2 = nx * nx + (ny * ny + nz * nz);
            let inv = 1.0 / len2.sqrt();
            nx *= inv;
            ny *= inv;
            nz *= inv;

            // separation = -dot(normal, av0 + bv0)
            let sx = a_v0x[i] + bv0x;
            let sy = a_v0y[i] + bv0y;
            let sz = a_v0z[i] + bv0z;

            let separation = -(sx * nx + (sy * ny + sz * nz));
            if separation > res.separation + abs_face_bias {
                res.normal = Vec3 {
                    x: nx,
                    y: ny,
                    z: nz,
                };
                res.separation = separation;
                res.type_ = SeparatingFeature::EdgePairAxis;

                // Half edge index
                res.index_a = 2 * i as i32;
                res.index_b = 2 * j as i32;

                // Edge beats face, remove bias
                abs_face_bias = 0.0;
                if separation > speculative {
                    return res;
                }
            }
        }
    }

    res
}
