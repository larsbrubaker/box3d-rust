// Simplex solvers and witness helpers from distance.c.
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use super::types::{Simplex, SimplexCache};
use crate::math_functions::{
    add, blend2, blend3, cross, distance, dot, length, scalar_triple_product, sub, Vec3, VEC3_ZERO,
};

/// Last element is the divisor. (b3BarycentricCoordsEdge)
fn barycentric_coords_edge(a: Vec3, b: Vec3) -> [f32; 3] {
    let ab = sub(b, a);
    let divisor = dot(ab, ab);
    [dot(b, ab), -dot(a, ab), divisor]
}

/// Last element is the divisor. (b3BarycentricCoordsTri)
fn barycentric_coords_tri(a: Vec3, b: Vec3, c: Vec3) -> [f32; 4] {
    let ab = sub(b, a);
    let ac = sub(c, a);

    let b_x_c = cross(b, c);
    let c_x_a = cross(c, a);
    let a_x_b = cross(a, b);

    let ab_x_ac = cross(ab, ac);

    // Last element is divisor
    let divisor = dot(ab_x_ac, ab_x_ac);

    [
        dot(b_x_c, ab_x_ac),
        dot(c_x_a, ab_x_ac),
        dot(a_x_b, ab_x_ac),
        divisor,
    ]
}

/// Last element is the divisor (forced to be positive). (b3BarycentricCoordsTet)
fn barycentric_coords_tet(a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> [f32; 5] {
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ad = sub(d, a);

    // Last element is divisor (forced to be positive)
    let divisor = scalar_triple_product(ab, ac, ad);

    let sign = if divisor < 0.0 { -1.0 } else { 1.0 };
    [
        sign * scalar_triple_product(b, c, d),
        sign * scalar_triple_product(a, d, c),
        sign * scalar_triple_product(a, b, d),
        sign * scalar_triple_product(a, c, b),
        sign * divisor,
    ]
}

pub(crate) fn get_metric(simplex: &Simplex) -> f32 {
    let count = simplex.count;
    debug_assert!((1..=4).contains(&count));

    let vertices = &simplex.vertices;

    match count {
        1 => 0.0,
        2 => {
            let a = vertices[0].w;
            let b = vertices[1].w;
            distance(a, b)
        }
        3 => {
            let a = vertices[0].w;
            let b = vertices[1].w;
            let c = vertices[2].w;
            length(cross(sub(b, a), sub(c, a))) / 2.0
        }
        4 => {
            let a = vertices[0].w;
            let b = vertices[1].w;
            let c = vertices[2].w;
            let d = vertices[3].w;
            scalar_triple_product(sub(b, a), sub(c, a), sub(d, a)) / 6.0
        }
        _ => {
            debug_assert!(false, "Should never get here!");
            0.0
        }
    }
}

pub(crate) fn write_cache(cache: &mut SimplexCache, simplex: &Simplex) {
    let count = simplex.count;
    cache.metric = get_metric(simplex);
    cache.count = count as u16;
    for index in 0..count as usize {
        cache.index_a[index] = simplex.vertices[index].index_a as u8;
        cache.index_b[index] = simplex.vertices[index].index_b as u8;
    }
}

pub(crate) fn solve_simplex2(simplex: &mut Simplex) -> bool {
    debug_assert!(simplex.count == 2);
    let vs = &mut simplex.vertices;

    let a = vs[0].w;
    let b = vs[1].w;
    let ab = sub(b, a);

    // Last element is divisor
    let divisor = dot(ab, ab);

    let u = dot(b, ab);
    let v = -dot(a, ab);

    // V( A )
    if v <= 0.0 {
        // Reduce simplex
        simplex.count = 1;
        vs[0].a = 1.0;
        return true;
    }

    // V( B )
    if u <= 0.0 {
        // Reduce simplex
        simplex.count = 1;
        vs[0] = vs[1];
        vs[0].a = 1.0;
        return true;
    }

    // Edge region
    if divisor <= 0.0 {
        return false;
    }

    // VR( AB )
    let denominator = 1.0 / divisor;
    vs[0].a = denominator * u;
    vs[1].a = denominator * v;

    true
}

pub(crate) fn solve_simplex3(simplex: &mut Simplex) -> bool {
    debug_assert!(simplex.count == 3);

    // Get simplex (be aware of aliasing here!)
    let v1 = simplex.vertices[0];
    let v2 = simplex.vertices[1];
    let v3 = simplex.vertices[2];

    // Vertex regions
    let w_ab = barycentric_coords_edge(v1.w, v2.w);
    let w_bc = barycentric_coords_edge(v2.w, v3.w);
    let w_ca = barycentric_coords_edge(v3.w, v1.w);

    // VR( A )
    if w_ab[1] <= 0.0 && w_ca[0] <= 0.0 {
        simplex.count = 1;
        simplex.vertices[0] = v1;
        simplex.vertices[0].a = 1.0;
        return true;
    }

    // VR( B )
    if w_bc[1] <= 0.0 && w_ab[0] <= 0.0 {
        simplex.count = 1;
        simplex.vertices[0] = v2;
        simplex.vertices[0].a = 1.0;
        return true;
    }

    // VR( C )
    if w_ca[1] <= 0.0 && w_bc[0] <= 0.0 {
        simplex.count = 1;
        simplex.vertices[0] = v3;
        simplex.vertices[0].a = 1.0;
        return true;
    }

    // Edge regions
    let w_abc = barycentric_coords_tri(v1.w, v2.w, v3.w);

    // VR( AB )
    if w_abc[2] <= 0.0 && w_ab[0] > 0.0 && w_ab[1] > 0.0 {
        simplex.count = 2;
        simplex.vertices[0] = v1;
        simplex.vertices[1] = v2;

        let divisor = w_ab[2];
        if divisor <= 0.0 {
            return false;
        }

        simplex.vertices[0].a = w_ab[0] / divisor;
        simplex.vertices[1].a = w_ab[1] / divisor;
        return true;
    }

    // VR( BC )
    if w_abc[0] <= 0.0 && w_bc[0] > 0.0 && w_bc[1] > 0.0 {
        simplex.count = 2;
        simplex.vertices[0] = v2;
        simplex.vertices[1] = v3;

        let divisor = w_bc[2];
        if divisor <= 0.0 {
            return false;
        }

        simplex.vertices[0].a = w_bc[0] / divisor;
        simplex.vertices[1].a = w_bc[1] / divisor;
        return true;
    }

    // VR( CA )
    if w_abc[1] <= 0.0 && w_ca[0] > 0.0 && w_ca[1] > 0.0 {
        simplex.count = 2;
        simplex.vertices[0] = v3;
        simplex.vertices[1] = v1;

        let divisor = w_ca[2];
        if divisor <= 0.0 {
            return false;
        }

        simplex.vertices[0].a = w_ca[0] / divisor;
        simplex.vertices[1].a = w_ca[1] / divisor;
        return true;
    }

    // Face region
    let divisor = w_abc[3];
    if divisor <= 0.0 {
        return false;
    }

    // VR( ABC )
    simplex.vertices[0].a = w_abc[0] / divisor;
    simplex.vertices[1].a = w_abc[1] / divisor;
    simplex.vertices[2].a = w_abc[2] / divisor;

    true
}

pub(crate) fn solve_simplex4(simplex: &mut Simplex) -> bool {
    debug_assert!(simplex.count == 4);

    // Get simplex (be aware of aliasing here!)
    let vertex_a = simplex.vertices[0];
    let vertex_b = simplex.vertices[1];
    let vertex_c = simplex.vertices[2];
    let vertex_d = simplex.vertices[3];

    // Vertex region
    let w_ab = barycentric_coords_edge(vertex_a.w, vertex_b.w);
    let w_ac = barycentric_coords_edge(vertex_a.w, vertex_c.w);
    let w_ad = barycentric_coords_edge(vertex_a.w, vertex_d.w);
    let w_bc = barycentric_coords_edge(vertex_b.w, vertex_c.w);
    let w_cd = barycentric_coords_edge(vertex_c.w, vertex_d.w);
    let w_db = barycentric_coords_edge(vertex_d.w, vertex_b.w);

    // VR( A )
    if w_ab[1] <= 0.0 && w_ac[1] <= 0.0 && w_ad[1] <= 0.0 {
        simplex.count = 1;
        simplex.vertices[0] = vertex_a;
        simplex.vertices[0].a = 1.0;
        return true;
    }

    // VR( B )
    if w_ab[0] <= 0.0 && w_db[0] <= 0.0 && w_bc[1] <= 0.0 {
        simplex.count = 1;
        simplex.vertices[0] = vertex_b;
        simplex.vertices[0].a = 1.0;
        return true;
    }

    // VR( C )
    if w_ac[0] <= 0.0 && w_bc[0] <= 0.0 && w_cd[1] <= 0.0 {
        simplex.count = 1;
        simplex.vertices[0] = vertex_c;
        simplex.vertices[0].a = 1.0;
        return true;
    }

    // VR( D )
    if w_ad[0] <= 0.0 && w_cd[0] <= 0.0 && w_db[1] <= 0.0 {
        simplex.count = 1;
        simplex.vertices[0] = vertex_d;
        simplex.vertices[0].a = 1.0;
        return true;
    }

    // Edge region
    let w_acb = barycentric_coords_tri(vertex_a.w, vertex_c.w, vertex_b.w);
    let w_abd = barycentric_coords_tri(vertex_a.w, vertex_b.w, vertex_d.w);
    let w_adc = barycentric_coords_tri(vertex_a.w, vertex_d.w, vertex_c.w);
    let w_bcd = barycentric_coords_tri(vertex_b.w, vertex_c.w, vertex_d.w);

    // VR( AB )
    if w_abd[2] <= 0.0 && w_acb[1] <= 0.0 && w_ab[0] > 0.0 && w_ab[1] > 0.0 {
        simplex.count = 2;
        simplex.vertices[0] = vertex_a;
        simplex.vertices[1] = vertex_b;

        let divisor = w_ab[2];
        if divisor <= 0.0 {
            return false;
        }

        simplex.vertices[0].a = w_ab[0] / divisor;
        simplex.vertices[1].a = w_ab[1] / divisor;
        return true;
    }

    // VR( AC )
    if w_acb[2] <= 0.0 && w_adc[1] <= 0.0 && w_ac[0] > 0.0 && w_ac[1] > 0.0 {
        simplex.count = 2;
        simplex.vertices[0] = vertex_a;
        simplex.vertices[1] = vertex_c;

        let divisor = w_ac[2];
        if divisor <= 0.0 {
            return false;
        }

        simplex.vertices[0].a = w_ac[0] / divisor;
        simplex.vertices[1].a = w_ac[1] / divisor;
        return true;
    }

    // VR( AD )
    if w_adc[2] <= 0.0 && w_abd[1] <= 0.0 && w_ad[0] > 0.0 && w_ad[1] > 0.0 {
        simplex.count = 2;
        simplex.vertices[0] = vertex_a;
        simplex.vertices[1] = vertex_d;

        let divisor = w_ad[2];
        if divisor <= 0.0 {
            return false;
        }

        simplex.vertices[0].a = w_ad[0] / divisor;
        simplex.vertices[1].a = w_ad[1] / divisor;
        return true;
    }

    // VR( BC )
    if w_acb[0] <= 0.0 && w_bcd[2] <= 0.0 && w_bc[0] > 0.0 && w_bc[1] > 0.0 {
        simplex.count = 2;
        simplex.vertices[0] = vertex_b;
        simplex.vertices[1] = vertex_c;

        let divisor = w_bc[2];
        if divisor <= 0.0 {
            return false;
        }

        simplex.vertices[0].a = w_bc[0] / divisor;
        simplex.vertices[1].a = w_bc[1] / divisor;
        return true;
    }

    // VR( CD )
    if w_adc[0] <= 0.0 && w_bcd[0] <= 0.0 && w_cd[0] > 0.0 && w_cd[1] > 0.0 {
        simplex.count = 2;
        simplex.vertices[0] = vertex_c;
        simplex.vertices[1] = vertex_d;

        let divisor = w_cd[2];
        if divisor <= 0.0 {
            return false;
        }

        simplex.vertices[0].a = w_cd[0] / divisor;
        simplex.vertices[1].a = w_cd[1] / divisor;
        return true;
    }

    // VR( DB )
    if w_abd[0] <= 0.0 && w_bcd[1] <= 0.0 && w_db[0] > 0.0 && w_db[1] > 0.0 {
        simplex.count = 2;
        simplex.vertices[0] = vertex_d;
        simplex.vertices[1] = vertex_b;

        let divisor = w_db[2];
        if divisor <= 0.0 {
            return false;
        }

        simplex.vertices[0].a = w_db[0] / divisor;
        simplex.vertices[1].a = w_db[1] / divisor;
        return true;
    }

    // Face regions
    let w_abcd = barycentric_coords_tet(vertex_a.w, vertex_b.w, vertex_c.w, vertex_d.w);

    // VR( ACB )
    if w_abcd[3] < 0.0 && w_acb[0] > 0.0 && w_acb[1] > 0.0 && w_acb[2] > 0.0 {
        simplex.count = 3;
        simplex.vertices[0] = vertex_a;
        simplex.vertices[1] = vertex_c;
        simplex.vertices[2] = vertex_b;

        let divisor = w_acb[3];
        if divisor <= 0.0 {
            return false;
        }

        simplex.vertices[0].a = w_acb[0] / divisor;
        simplex.vertices[1].a = w_acb[1] / divisor;
        simplex.vertices[2].a = w_acb[2] / divisor;
        return true;
    }

    // VR( ABD )
    if w_abcd[2] < 0.0 && w_abd[0] > 0.0 && w_abd[1] > 0.0 && w_abd[2] > 0.0 {
        simplex.count = 3;
        simplex.vertices[0] = vertex_a;
        simplex.vertices[1] = vertex_b;
        simplex.vertices[2] = vertex_d;

        let divisor = w_abd[3];
        if divisor <= 0.0 {
            return false;
        }

        simplex.vertices[0].a = w_abd[0] / divisor;
        simplex.vertices[1].a = w_abd[1] / divisor;
        simplex.vertices[2].a = w_abd[2] / divisor;
        return true;
    }

    // VR( ADC )
    if w_abcd[1] < 0.0 && w_adc[0] > 0.0 && w_adc[1] > 0.0 && w_adc[2] > 0.0 {
        simplex.count = 3;
        simplex.vertices[0] = vertex_a;
        simplex.vertices[1] = vertex_d;
        simplex.vertices[2] = vertex_c;

        let divisor = w_adc[3];
        if divisor <= 0.0 {
            return false;
        }

        simplex.vertices[0].a = w_adc[0] / divisor;
        simplex.vertices[1].a = w_adc[1] / divisor;
        simplex.vertices[2].a = w_adc[2] / divisor;
        return true;
    }

    // VR( BCD )
    if w_abcd[0] < 0.0 && w_bcd[0] > 0.0 && w_bcd[1] > 0.0 && w_bcd[2] > 0.0 {
        simplex.count = 3;
        simplex.vertices[0] = vertex_b;
        simplex.vertices[1] = vertex_c;
        simplex.vertices[2] = vertex_d;

        let divisor = w_bcd[3];
        if divisor <= 0.0 {
            return false;
        }

        simplex.vertices[0].a = w_bcd[0] / divisor;
        simplex.vertices[1].a = w_bcd[1] / divisor;
        simplex.vertices[2].a = w_bcd[2] / divisor;
        return true;
    }

    // *** Inside tetrahedron ***
    let divisor = w_abcd[4];
    if divisor <= 0.0 {
        return false;
    }

    // VR( ABCD )
    simplex.vertices[0].a = w_abcd[0] / divisor;
    simplex.vertices[1].a = w_abcd[1] / divisor;
    simplex.vertices[2].a = w_abcd[2] / divisor;
    simplex.vertices[3].a = w_abcd[3] / divisor;

    true
}

pub(crate) fn compute_witness_points(simplex: &Simplex) -> (Vec3, Vec3) {
    let vs = &simplex.vertices;
    let count = simplex.count;
    debug_assert!((1..=4).contains(&count));

    match count {
        1 => (vs[0].w_a, vs[0].w_b),
        2 => (
            blend2(vs[0].a, vs[0].w_a, vs[1].a, vs[1].w_a),
            blend2(vs[0].a, vs[0].w_b, vs[1].a, vs[1].w_b),
        ),
        3 => (
            blend3(vs[0].a, vs[0].w_a, vs[1].a, vs[1].w_a, vs[2].a, vs[2].w_a),
            blend3(vs[0].a, vs[0].w_b, vs[1].a, vs[1].w_b, vs[2].a, vs[2].w_b),
        ),
        4 => {
            // Force identical points and *zero* distance
            let sum = add(
                blend2(vs[0].a, vs[0].w_a, vs[1].a, vs[1].w_a),
                blend2(vs[2].a, vs[2].w_a, vs[3].a, vs[3].w_a),
            );
            (sum, sum)
        }
        _ => {
            debug_assert!(false, "Should never get here!");
            (VEC3_ZERO, VEC3_ZERO)
        }
    }
}
