//! Collision / Shape Cast Debug — faithful port of `sample_collision.cpp`
//! `ShapeCastDebug` (line 1692). A degenerate large-world shape cast (triangle vs
//! capsule) reproduced at `scale = 0.01`; purely static, no world/stepping.

use box3d_rust::distance::{shape_cast, ShapeCastPairInput, ShapeProxy};
use box3d_rust::math_functions::{Transform, Vec3, QUAT_IDENTITY, VEC3_ZERO};
use wasm_bindgen::prelude::*;

/// Packed geometry for the TS viewer:
/// `[hit, fraction,
///   t0(3), t1(3), t2(3),          // triangle world verts
///   c1(3), c2(3), radius,         // capsule local endpoints + radius
///   greenP(3), grayP(3), redP(3)]`// capsule origins (identity rotation)
#[wasm_bindgen]
pub fn scd_data() -> Vec<f32> {
    // C constructor (line 1704). scale shrinks the far-world coordinates.
    let scale = 0.01f32;
    let s = |x: f32, y: f32, z: f32| Vec3 {
        x: scale * x,
        y: scale * y,
        z: scale * z,
    };

    // Triangle, then shifted so vertex 0 sits at the origin (line 1705-1712).
    let mut triangle = [
        s(0.0, 0.0, 0.0),
        s(0.0, -6400.0, 0.0),
        s(6400.0, 0.0, 22.609375),
    ];
    let origin = triangle[0];
    triangle[0] = VEC3_ZERO;
    triangle[1] = Vec3 {
        x: triangle[1].x - origin.x,
        y: triangle[1].y - origin.y,
        z: triangle[1].z - origin.z,
    };
    triangle[2] = Vec3 {
        x: triangle[2].x - origin.x,
        y: triangle[2].y - origin.y,
        z: triangle[2].z - origin.z,
    };

    // Capsule (line 1723).
    let c1 = s(43616.210937500, -100213.0, 132631.812500000);
    let c2 = s(342231.968750000, 359711.687500000, 132631.812500000);
    let radius = scale * 1.0;

    // Transform of B relative to A: p = scale * {..} - origin, q = identity.
    let transform = Transform {
        p: Vec3 {
            x: scale * -115200.0 - origin.x,
            y: scale * -19200.0 - origin.y,
            z: scale * -202755.0 - origin.z,
        },
        q: QUAT_IDENTITY,
    };

    let translation = s(0.008614914, 0.0, 72267.117187500);

    let mut proxy_a = ShapeProxy {
        count: 3,
        radius: 0.0,
        ..Default::default()
    };
    proxy_a.points[0] = triangle[0];
    proxy_a.points[1] = triangle[1];
    proxy_a.points[2] = triangle[2];

    let mut proxy_b = ShapeProxy {
        count: 2,
        radius,
        ..Default::default()
    };
    proxy_b.points[0] = c1;
    proxy_b.points[1] = c2;

    let input = ShapeCastPairInput {
        proxy_a,
        proxy_b,
        transform,
        translation_b: translation,
        max_fraction: 0.970617533,
        can_encroach: false,
    };

    let output = shape_cast(&input);

    let green_p = transform.p;
    let gray_p = Vec3 {
        x: transform.p.x + translation.x,
        y: transform.p.y + translation.y,
        z: transform.p.z + translation.z,
    };
    let red_p = Vec3 {
        x: transform.p.x + output.fraction * translation.x,
        y: transform.p.y + output.fraction * translation.y,
        z: transform.p.z + output.fraction * translation.z,
    };

    vec![
        if output.hit { 1.0 } else { 0.0 },
        output.fraction,
        triangle[0].x,
        triangle[0].y,
        triangle[0].z,
        triangle[1].x,
        triangle[1].y,
        triangle[1].z,
        triangle[2].x,
        triangle[2].y,
        triangle[2].z,
        c1.x,
        c1.y,
        c1.z,
        c2.x,
        c2.y,
        c2.z,
        radius,
        green_p.x,
        green_p.y,
        green_p.z,
        gray_p.x,
        gray_p.y,
        gray_p.z,
        red_p.x,
        red_p.y,
        red_p.z,
    ]
}
