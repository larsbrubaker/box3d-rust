//! Collision / Distance Debug — faithful port of `sample_collision.cpp`
//! `DistanceDebug` (line 1793). Two boxes at fixed transforms; `b3ShapeDistance`
//! records its simplex history so a slider can step through the GJK iterations.
//! `transformA` is identity, so all A-frame points are world points as-is.

use box3d_rust::distance::{shape_distance, DistanceInput, ShapeProxy, Simplex, SimplexCache};
use box3d_rust::hull::{make_box_hull, make_transformed_box_hull};
use box3d_rust::math_functions::{
    inv_mul_world_transforms, length, Quat, Transform, Vec3, WorldTransform, VEC3_ZERO,
    WORLD_TRANSFORM_IDENTITY,
};
use wasm_bindgen::prelude::*;

fn transform_b() -> WorldTransform {
    // C: m_transformB (line 1834).
    WorldTransform {
        p: Vec3 {
            x: -1.64657831e-06,
            y: 1.00989532471,
            z: 0.0,
        },
        q: Quat {
            v: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.00494779600,
            },
            s: 0.999987781,
        },
    }
}

fn box_proxy(points: &[Vec3; 8]) -> ShapeProxy {
    let mut p = ShapeProxy {
        count: 8,
        radius: 0.0,
        ..Default::default()
    };
    p.points[..8].copy_from_slice(points);
    p
}

/// C `BuildWitnessPoints` (line 1872): barycentric blend of the simplex vertices.
fn build_witness(simplex: &Simplex) -> (Vec3, Vec3) {
    let vs = &simplex.vertices;
    let blend = |get: fn(&box3d_rust::distance::SimplexVertex) -> Vec3| -> Vec3 {
        match simplex.count {
            1 => get(&vs[0]),
            2 => Vec3 {
                x: vs[0].a * get(&vs[0]).x + vs[1].a * get(&vs[1]).x,
                y: vs[0].a * get(&vs[0]).y + vs[1].a * get(&vs[1]).y,
                z: vs[0].a * get(&vs[0]).z + vs[1].a * get(&vs[1]).z,
            },
            3 => Vec3 {
                x: vs[0].a * get(&vs[0]).x + vs[1].a * get(&vs[1]).x + vs[2].a * get(&vs[2]).x,
                y: vs[0].a * get(&vs[0]).y + vs[1].a * get(&vs[1]).y + vs[2].a * get(&vs[2]).y,
                z: vs[0].a * get(&vs[0]).z + vs[1].a * get(&vs[1]).z + vs[2].a * get(&vs[2]).z,
            },
            _ => Vec3 {
                x: vs[0].a * get(&vs[0]).x
                    + vs[1].a * get(&vs[1]).x
                    + vs[2].a * get(&vs[2]).x
                    + vs[3].a * get(&vs[3]).x,
                y: vs[0].a * get(&vs[0]).y
                    + vs[1].a * get(&vs[1]).y
                    + vs[2].a * get(&vs[2]).y
                    + vs[3].a * get(&vs[3]).y,
                z: vs[0].a * get(&vs[0]).z
                    + vs[1].a * get(&vs[1]).z
                    + vs[2].a * get(&vs[2]).z
                    + vs[3].a * get(&vs[3]).z,
            },
        }
    };
    // count == 4 forces identical points (C line 1897); both use wA blend order,
    // but wA == wB there so either is fine.
    (blend(|v| v.w_a), blend(|v| v.w_b))
}

/// C `GetClosestPoint` (line 1906): barycentric blend of the `w` support points.
fn closest_point(simplex: &Simplex) -> Vec3 {
    let vs = &simplex.vertices;
    let mut p = VEC3_ZERO;
    for i in 0..simplex.count as usize {
        p.x += vs[i].a * vs[i].w.x;
        p.y += vs[i].a * vs[i].w.y;
        p.z += vs[i].a * vs[i].w.z;
    }
    p
}

/// Run the fixed distance query and pack the drawer geometry for `simplex_index`.
/// `[ transformB_p(3), transformB_q(4),
///    distance, normal(3), pointA(3), pointB(3),
///    simplexCount, hasSel, witnessV1(3), witnessV2(3), currentDistance,
///    vcount, vcount×(wA(3), wB(3)) ]`
#[wasm_bindgen]
pub fn dd_data(simplex_index: i32) -> Vec<f32> {
    let box_a = make_box_hull(40.0, 1.0, 40.0);
    let box_b = make_transformed_box_hull(
        0.5,
        10.0,
        0.5,
        Transform {
            p: Vec3 {
                x: 0.0,
                y: 10.0,
                z: 0.0,
            },
            q: box3d_rust::math_functions::QUAT_IDENTITY,
        },
    );

    let transform_a = WORLD_TRANSFORM_IDENTITY;
    let xf_b = transform_b();

    let input = DistanceInput {
        proxy_a: box_proxy(&box_a.box_points),
        proxy_b: box_proxy(&box_b.box_points),
        transform: inv_mul_world_transforms(transform_a, xf_b),
        use_radii: false,
    };

    let mut cache = SimplexCache::default();
    let mut simplexes = [Simplex::default(); 32];
    let output = shape_distance(&input, &mut cache, Some(&mut simplexes[..]));
    let simplex_count = output.simplex_count;

    let mut out = vec![
        xf_b.p.x,
        xf_b.p.y,
        xf_b.p.z,
        xf_b.q.v.x,
        xf_b.q.v.y,
        xf_b.q.v.z,
        xf_b.q.s,
        output.distance,
        output.normal.x,
        output.normal.y,
        output.normal.z,
        output.point_a.x,
        output.point_a.y,
        output.point_a.z,
        output.point_b.x,
        output.point_b.y,
        output.point_b.z,
        simplex_count as f32,
    ];

    if simplex_count > 0 {
        let idx = simplex_index.clamp(0, simplex_count - 1) as usize;
        let simplex = &simplexes[idx];
        let (v1, v2) = build_witness(simplex);
        let current_distance = length(closest_point(simplex));
        out.push(1.0);
        out.extend_from_slice(&[v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, current_distance]);
        out.push(simplex.count as f32);
        for i in 0..simplex.count as usize {
            let v = &simplex.vertices[i];
            out.extend_from_slice(&[v.w_a.x, v.w_a.y, v.w_a.z, v.w_b.x, v.w_b.y, v.w_b.z]);
        }
    } else {
        out.push(0.0);
        out.extend_from_slice(&[0.0; 8]); // witnessV1(3),V2(3),currentDistance, vcount
    }

    out
}
