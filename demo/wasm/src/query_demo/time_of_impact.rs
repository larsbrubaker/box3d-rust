//! Collision / Time of Impact — faithful port of `sample_collision.cpp`
//! `TimeOfImpact` (line 2498). A fixed shape-A vs a swept shape-B; `b3TimeOfImpact`
//! reports the collision fraction. Shape types (box / capsule / triangle) are
//! selectable, matching the C `MakeProxy` cases.

use box3d_rust::distance::{
    get_sweep_transform, time_of_impact, ShapeProxy, Sweep, ToiInput, ToiState,
};
use box3d_rust::geometry::Capsule;
use box3d_rust::hull::{make_box_hull, BoxHull};
use box3d_rust::math_functions::{
    get_axis_angle, inv_mul_quat, Quat, Transform, Vec3, PI, QUAT_IDENTITY, TRANSFORM_IDENTITY,
    VEC3_ZERO,
};
use wasm_bindgen::prelude::*;

// C shape enum: e_box = 0, e_capsule = 1, e_triangle = 2 (line 2501). Box is the
// `_` fallback arm, matching the C `default`/`e_box` proxy path.
const KIND_CAPSULE: i32 = 1;
const KIND_TRIANGLE: i32 = 2;

fn box_hull() -> BoxHull {
    // C: m_box = b3MakeBoxHull(0.02, 0.2, 0.04) (line 2516).
    make_box_hull(0.02, 0.2, 0.04)
}

fn capsule() -> Capsule {
    // C: m_capsule = {{0,-0.2,0},{0,0.2,0},0.02} (line 2517).
    Capsule {
        center1: Vec3 {
            x: 0.0,
            y: -0.2,
            z: 0.0,
        },
        center2: Vec3 {
            x: 0.0,
            y: 0.2,
            z: 0.0,
        },
        radius: 0.02,
    }
}

fn triangle() -> [Vec3; 3] {
    // C: m_triangle (line 2518).
    [
        Vec3 {
            x: -4.0,
            y: 0.0,
            z: -4.0,
        },
        Vec3 {
            x: -4.0,
            y: 0.0,
            z: -8.0,
        },
        Vec3 {
            x: -8.0,
            y: 0.0,
            z: -8.0,
        },
    ]
}

fn make_proxy(kind: i32) -> ShapeProxy {
    match kind {
        KIND_CAPSULE => {
            let c = capsule();
            let mut p = ShapeProxy {
                count: 2,
                radius: c.radius,
                ..Default::default()
            };
            p.points[0] = c.center1;
            p.points[1] = c.center2;
            p
        }
        KIND_TRIANGLE => {
            let t = triangle();
            let mut p = ShapeProxy {
                count: 3,
                radius: 0.0,
                ..Default::default()
            };
            p.points[..3].copy_from_slice(&t);
            p
        }
        _ => {
            let b = box_hull();
            let count = b.base.vertex_count;
            let mut p = ShapeProxy {
                count,
                radius: 0.0,
                ..Default::default()
            };
            for i in 0..count as usize {
                p.points[i] = b.box_points[i];
            }
            p
        }
    }
}

/// Nine-float shape descriptor for the TS drawer: kind then params.
/// box → [hx,hy,hz,..], capsule → [c1(3),c2(3),radius,..], triangle → [v0..v2].
fn descriptor(kind: i32) -> [f32; 9] {
    match kind {
        KIND_CAPSULE => {
            let c = capsule();
            [
                c.center1.x,
                c.center1.y,
                c.center1.z,
                c.center2.x,
                c.center2.y,
                c.center2.z,
                c.radius,
                0.0,
                0.0,
            ]
        }
        KIND_TRIANGLE => {
            let t = triangle();
            [
                t[0].x, t[0].y, t[0].z, t[1].x, t[1].y, t[1].z, t[2].x, t[2].y, t[2].z,
            ]
        }
        // Box half-extents from the constructor.
        _ => [0.02, 0.2, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }
}

fn push_xf(out: &mut Vec<f32>, xf: Transform) {
    out.extend_from_slice(&[xf.p.x, xf.p.y, xf.p.z, xf.q.v.x, xf.q.v.y, xf.q.v.z, xf.q.s]);
}

/// Run the TOI for the selected A/B shape kinds and pack the drawer geometry.
#[wasm_bindgen]
pub fn toi_data(type_a: i32, type_b: i32) -> Vec<f32> {
    // Sweep A: identity (c1=c2=0, q1=q2=identity) (line 2528).
    let sweep_a = Sweep {
        local_center: VEC3_ZERO,
        c1: VEC3_ZERO,
        c2: VEC3_ZERO,
        q1: QUAT_IDENTITY,
        q2: QUAT_IDENTITY,
    };

    // Sweep B: fixed capture values (line 2532).
    let sweep_b = Sweep {
        local_center: VEC3_ZERO,
        c1: Vec3 {
            x: -4.06512070,
            y: 0.101333618,
            z: -7.87591267,
        },
        c2: Vec3 {
            x: -4.15895557,
            y: 0.0356027633,
            z: -7.69682646,
        },
        q1: Quat {
            v: Vec3 {
                x: -0.860495985,
                y: -0.272824734,
                z: 0.0724888667,
            },
            s: 0.424097389,
        },
        q2: Quat {
            v: Vec3 {
                x: -0.604184389,
                y: -0.424355596,
                z: 0.0457959622,
            },
            s: 0.672894001,
        },
    };

    let input = ToiInput {
        proxy_a: make_proxy(type_a),
        proxy_b: make_proxy(type_b),
        sweep_a,
        sweep_b,
        max_fraction: 1.0,
    };

    let output = time_of_impact(&input);

    let transform1 = get_sweep_transform(&sweep_b, 0.0);
    let transform2 = get_sweep_transform(&sweep_b, 1.0);

    // qr = inv(q1) * q2 (line 2655).
    let qr = inv_mul_quat(transform1.q, transform2.q);
    let mut angle = 0.0f32;
    let _ = get_axis_angle(&mut angle, qr);
    let angle_deg = 180.0 * angle / PI;

    let state_code = match output.state {
        ToiState::Unknown => 0.0,
        ToiState::Failed => 1.0,
        ToiState::Overlapped => 2.0,
        ToiState::Hit => 3.0,
        ToiState::Separated => 4.0,
    };

    let mut out = Vec::new();
    out.push(type_a as f32);
    out.extend_from_slice(&descriptor(type_a));
    out.push(type_b as f32);
    out.extend_from_slice(&descriptor(type_b));
    push_xf(&mut out, transform1);
    push_xf(&mut out, transform2);

    if output.fraction < 1.0 {
        let transform_hit = get_sweep_transform(&sweep_b, output.fraction);
        out.push(1.0);
        push_xf(&mut out, transform_hit);
    } else {
        out.push(0.0);
        push_xf(&mut out, TRANSFORM_IDENTITY);
    }

    out.push(state_code);
    out.push(output.fraction);
    out.push(angle_deg);

    // Hit point + normal (drawn for Hit / Failed states, line 2669).
    if output.state == ToiState::Hit || output.state == ToiState::Failed {
        out.push(1.0);
        out.extend_from_slice(&[
            output.point.x,
            output.point.y,
            output.point.z,
            output.normal.x,
            output.normal.y,
            output.normal.z,
        ]);
    } else {
        out.push(0.0);
        out.extend_from_slice(&[0.0; 6]);
    }

    out
}
