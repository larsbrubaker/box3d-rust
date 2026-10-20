//! Edge candidate culling for the separating axis test in `convex_manifold.c`
//! (`b3GetFaceDots`, `b3TestEdgeCandidateSorted`, `b3GetFacePlaneSeparations`,
//! `b3FilterEdgeCandidates`). Split from [`super::separating_axis`] to keep files focused.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::core::SIMD_WIDTH;
use crate::hull::{
    get_hull_edge_cosines, get_hull_edges, get_hull_planes, get_hull_soa_normals,
    hull_soa_normal_count, HullData, HULL_SOA_PADDING,
};
use crate::math_functions::{max_float, min_float, Vec3};
use crate::simd_scalar::{dot_w, load_vw, splat_vw, store_w};

// _Static_assert( B3_SIMD_WIDTH <= B3_HULL_SOA_PADDING, "hull SoA padding must cover the SIMD width" );
const _: () = assert!(
    SIMD_WIDTH as i32 <= HULL_SOA_PADDING,
    "hull SoA padding must cover the SIMD width"
);

/// Round a hull SoA element count up to the SIMD width. Loops over the hull SoA arrays use
/// this count; the arrays themselves are strided by `b3GetHullSoaStride`, which pads further
/// (to 8) for the AVX2 path. (b3RoundUpToSIMDWidth)
#[inline]
pub(super) fn round_up_to_simd_width(count: i32) -> i32 {
    (count + SIMD_WIDTH as i32 - 1) & !(SIMD_WIDTH as i32 - 1)
}

/// Wide dot(n, d) for all face normals n of the hull, padded to the SIMD width.
/// (b3GetFaceDots)
pub(super) fn get_face_dots(hull: &HullData, d: Vec3, dots: &mut [f32]) {
    let face_stride = hull_soa_normal_count(hull);
    let normals = get_hull_soa_normals(hull);
    let nx = &normals[0..face_stride];
    let ny = &normals[face_stride..2 * face_stride];
    let nz = &normals[2 * face_stride..3 * face_stride];

    let d_w = splat_vw(d);

    let wide_face_count = round_up_to_simd_width(hull.face_count) as usize;
    let mut i = 0;
    while i < wide_face_count {
        // dot product per lane
        let m = dot_w(load_vw(&nx[i..], &ny[i..], &nz[i..]), d_w);
        store_w(&mut dots[i..], m);
        i += SIMD_WIDTH;
    }
}

/// (B3_PARALLEL_TOL)
const PARALLEL_TOL: f32 = 1e-4;

/// Inscribed sphere edge test. <https://box2d.org/posts/2026/09/inscribed-spheres/>
/// The implementation here is a mix of the versions from Cairn Overturf and Dirk Gregorius:
/// <https://gist.github.com/cairnc/dee7a2866da0709f2d9a77b6493b57b5>
/// <https://gist.github.com/dgregorius/e6751b5c00937cd21af63cba3c53c861>
/// This benchmarks faster than the version from the blog post.
/// (b3TestEdgeCandidateSorted)
pub(crate) fn test_edge_candidate_sorted(a1: f32, a2: f32, c: f32, bound: f32) -> usize {
    // We want to maximize
    //
    //   f(t) = dot(d, nlerp(n1, n2, t))
    //
    // over t in [0, 1], where
    //
    //   a1 = dot(n1, d)
    //   a2 = dot(n2, d)
    //   c  = dot(n1, n2).
    //
    // The maximum can occur either at an endpoint or at an interior
    // critical point where f'(t) = 0.

    // Since the endpoint values are a1 and a2, only the larger endpoint
    // needs to be tested against the bound.
    let hi = max_float(a1, a2);
    let lo = min_float(a1, a2);
    let exterior = (hi >= bound) as usize;

    // Differentiating f(t) gives
    //
    //           a2 - c*a1 - (1 - c)*(a1 + a2)*t
    //   f'(t) = --------------------------------------
    //           ((1 - t)^2 + t^2 + 2*c*t*(1 - t))^(3/2)
    //
    // The denominator is positive and the numerator is linear in t.
    // An interior maximum therefore requires
    //
    //   f'(0) >= 0  ->  a2 >= c*a1
    //   f'(1) <= 0  ->  a1 >= c*a2.
    //
    // After sorting these become
    //
    //   hi >= c*lo
    //   lo >= c*hi.
    //
    // The second is always the stricter condition since
    //
    //   (hi - c*lo) - (lo - c*hi)
    //       = (1 + c)*(hi - lo) >= 0.
    //
    // Thus both conditions reduce to u >= 0.
    let u = lo - c * hi;
    let max_is_interior = (u >= 0.0) as usize;

    // Solving f'(t) = 0 and evaluating f(t) at tmax gives
    //
    //   f(tmax)^2 =
    //       (a1^2 + a2^2 - 2*c*a1*a2) / (1 - c^2).
    //
    // After sorting,
    //
    //   hi^2 + lo^2 - 2*c*hi*lo
    //       = hi^2*(1 - c^2) + u^2.
    //
    // Let s = 1 - c^2 and rearrange f(tmax) >= bound:
    //
    //   u^2 >= (bound^2 - hi^2)*s.
    let s = 1.0 - c * c;
    let bound_term = (bound - hi) * (bound + hi);
    let lhs = u * u;
    let rhs = bound_term * s;
    let max_beats_bound = (lhs >= rhs) as usize;

    // The interior expression contains 1 - c^2 in its denominator.
    // When this approaches zero the arc is degenerate or ill-conditioned,
    // so conservatively keep the edge as a candidate.
    let nearly_parallel = (s < PARALLEL_TOL) as usize;

    let interior = max_is_interior & (max_beats_bound | nearly_parallel);

    exterior | interior
}

/// dot(n, otherCenter) - offset for all faces of the hull, padded to the SIMD width.
/// (b3GetFacePlaneSeparations)
pub(super) fn get_face_plane_separations(
    hull: &HullData,
    other_center: Vec3,
    separations: &mut [f32],
) {
    get_face_dots(hull, other_center, separations);

    let planes = get_hull_planes(hull);
    let face_count = hull.face_count as usize;
    for i in 0..face_count {
        separations[i] -= planes[i].offset;
    }
}

/// The number of edge pair tests needed to make the extra culling pass worthwhile.
/// (B3_EDGE_PROBE_MIN_TESTS)
pub(super) const EDGE_PROBE_MIN_TESTS: usize = 10;

/// Re-test edge candidates against the plane separations of a probe point on the other hull.
/// (b3FilterEdgeCandidates)
pub(super) fn filter_edge_candidates(
    hull: &HullData,
    plane_separations: &[f32],
    bound: f32,
    edge_indices: &mut [i32],
    count: usize,
) -> usize {
    let half_edges = get_hull_edges(hull);
    let cosines = get_hull_edge_cosines(hull);
    let mut kept_count = 0usize;

    for k in 0..count {
        let i = edge_indices[k];
        let i1 = half_edges[i as usize].face as usize;
        let i2 = half_edges[(i + 1) as usize].face as usize;
        let c = cosines[(i >> 1) as usize];
        edge_indices[kept_count] = i;
        kept_count +=
            test_edge_candidate_sorted(plane_separations[i1], plane_separations[i2], c, bound);
    }

    kept_count
}
