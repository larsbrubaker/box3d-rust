//! Collision / Shape Cast Debug — faithful port of `sample_collision.cpp`
//! `ShapeCastDebug` (line 1692). A degenerate large-world shape cast (triangle vs
//! capsule) reproduced at `scale = 0.01`; purely static, no world/stepping.

use box3d_rust::distance::{shape_cast, CastOutput, ShapeCastPairInput, ShapeProxy};
use box3d_rust::math_functions::{Transform, Vec3, QUAT_IDENTITY, VEC3_ZERO};
use wasm_bindgen::prelude::*;

/// Geometry + cast result for Shape Cast Debug (C `ShapeCastDebug`).
struct ScdScene {
    hit: bool,
    fraction: f32,
    triangle: [Vec3; 3],
    c1: Vec3,
    c2: Vec3,
    radius: f32,
    green_p: Vec3,
    gray_p: Vec3,
    red_p: Vec3,
}

fn scd_scene() -> ScdScene {
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

    // Capsule (line 1723). Note: C dumps `radiusB = 40` in the header comment,
    // but the live sample uses `scale * 1.0f` (line 1725).
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

    let output: CastOutput = shape_cast(&input);

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

    ScdScene {
        hit: output.hit,
        fraction: output.fraction,
        triangle,
        c1,
        c2,
        radius,
        green_p,
        gray_p,
        red_p,
    }
}

/// Packed geometry for the TS viewer:
/// `[hit, fraction,
///   t0(3), t1(3), t2(3),          // triangle world verts
///   c1(3), c2(3), radius,         // capsule local endpoints + radius
///   greenP(3), grayP(3), redP(3)]`// capsule origins (identity rotation)
#[wasm_bindgen]
pub fn scd_data() -> Vec<f32> {
    let s = scd_scene();
    vec![
        if s.hit { 1.0 } else { 0.0 },
        s.fraction,
        s.triangle[0].x,
        s.triangle[0].y,
        s.triangle[0].z,
        s.triangle[1].x,
        s.triangle[1].y,
        s.triangle[1].z,
        s.triangle[2].x,
        s.triangle[2].y,
        s.triangle[2].z,
        s.c1.x,
        s.c1.y,
        s.c1.z,
        s.c2.x,
        s.c2.y,
        s.c2.z,
        s.radius,
        s.green_p.x,
        s.green_p.y,
        s.green_p.z,
        s.gray_p.x,
        s.gray_p.y,
        s.gray_p.z,
        s.red_p.x,
        s.red_p.y,
        s.red_p.z,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn world_end(local: Vec3, origin: Vec3) -> Vec3 {
        Vec3 {
            x: local.x + origin.x,
            y: local.y + origin.y,
            z: local.z + origin.z,
        }
    }

    #[test]
    fn scd_data_matches_c_geometry_and_produces_hit() {
        let s = scd_scene();
        let d = scd_data();
        assert_eq!(d.len(), 27);

        // Triangle after origin shift (C lines 1705–1712, scale 0.01).
        assert_eq!(s.triangle[0], VEC3_ZERO);
        assert!((s.triangle[1].y - (-64.0)).abs() < 1e-4);
        assert!((s.triangle[2].x - 64.0).abs() < 1e-4);
        assert!((s.triangle[2].z - 0.22609375).abs() < 1e-5);

        // Capsule radius stays the scaled hairline used by the cast math.
        assert!((s.radius - 0.01).abs() < 1e-8);

        // Transform sits thousands of units from the triangle (large-world repro).
        assert!((s.green_p.x - (-1152.0)).abs() < 1e-2);
        assert!((s.green_p.z - (-2027.55)).abs() < 1e-1);

        let g1 = world_end(s.c1, s.green_p);
        let g2 = world_end(s.c2, s.green_p);
        let gray1 = world_end(s.c1, s.gray_p);
        let gray2 = world_end(s.c2, s.gray_p);

        // Green capsule world ends ~(-716,-1194,-701) … (2270,3405,-701).
        assert!((g1.x - (-715.8379)).abs() < 0.01);
        assert!((g1.y - (-1194.13)).abs() < 0.01);
        assert!((g1.z - (-701.2319)).abs() < 0.01);
        assert!((g2.x - 2270.32).abs() < 0.01);
        assert!((g2.y - 3405.12).abs() < 0.01);

        // Gray (full sweep) ends near z ≈ +21 — still far in XY from the origin.
        assert!((gray1.z - 21.439).abs() < 0.02);
        assert!((gray2.z - 21.439).abs() < 0.02);

        // Scene AABB of triangle ∪ capsules spans thousands of units (viewer must
        // raise far / frame on this, not rely on default distance 20 + far 600).
        let mut min = [
            s.triangle[0].x.min(s.triangle[1].x).min(s.triangle[2].x),
            s.triangle[0].y.min(s.triangle[1].y).min(s.triangle[2].y),
            s.triangle[0].z.min(s.triangle[1].z).min(s.triangle[2].z),
        ];
        let mut max = [
            s.triangle[0].x.max(s.triangle[1].x).max(s.triangle[2].x),
            s.triangle[0].y.max(s.triangle[1].y).max(s.triangle[2].y),
            s.triangle[0].z.max(s.triangle[1].z).max(s.triangle[2].z),
        ];
        for p in [g1, g2, gray1, gray2] {
            min[0] = min[0].min(p.x);
            min[1] = min[1].min(p.y);
            min[2] = min[2].min(p.z);
            max[0] = max[0].max(p.x);
            max[1] = max[1].max(p.y);
            max[2] = max[2].max(p.z);
        }
        let extent = (max[0] - min[0]).max(max[1] - min[1]).max(max[2] - min[2]);
        assert!(extent > 2000.0, "expected large-world extent, got {extent}");

        // The cast itself must hit — this is the whole point of the repro.
        assert!(s.hit, "ShapeCastDebug cast should hit");
        assert!(
            s.fraction > 0.0 && s.fraction <= 0.970617533,
            "fraction {} out of range",
            s.fraction
        );

        // Packed buffer mirrors the scene.
        assert!(d[0] > 0.5);
        assert!((d[1] - s.fraction).abs() < 1e-6);
        assert!((d[17] - 0.01).abs() < 1e-8);
    }
}
