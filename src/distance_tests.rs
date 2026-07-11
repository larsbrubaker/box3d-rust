// Port of box3d-cpp-reference/test/test_distance.c
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use crate::distance::{
    make_proxy, shape_cast, shape_distance, time_of_impact, DistanceInput, ShapeCastPairInput,
    SimplexCache, Sweep, ToiInput, ToiState,
};
use crate::math_functions::{
    segment_distance, Vec3, QUAT_IDENTITY, TRANSFORM_IDENTITY, VEC3_ZERO,
};

fn ensure_small(value: f32, tolerance: f32) {
    // Matches the C ENSURE_SMALL macro, which is inclusive: pass when
    // -tol <= value <= tol.
    assert!(
        !(value < -tolerance || tolerance < value),
        "|{value}| > tolerance {tolerance}"
    );
}

fn v(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

#[test]
fn segment_distance_test() {
    // C designated initializers leave unspecified components as zero.
    let p1 = v(-1.0, -1.0, 0.0);
    let q1 = v(-1.0, 1.0, 0.0);
    let p2 = v(2.0, 0.0, 0.0);
    let q2 = v(1.0, 0.0, 0.0);

    let result = segment_distance(p1, q1, p2, q2);

    ensure_small(result.fraction1 - 0.5, f32::EPSILON);
    ensure_small(result.fraction2 - 1.0, f32::EPSILON);
    ensure_small(result.point1.x + 1.0, f32::EPSILON);
    ensure_small(result.point1.y, f32::EPSILON);
    ensure_small(result.point1.z, f32::EPSILON);
    ensure_small(result.point2.x - 1.0, f32::EPSILON);
    ensure_small(result.point2.y, f32::EPSILON);
    ensure_small(result.point2.z, f32::EPSILON);
}

#[test]
fn shape_distance_test() {
    let vas = [
        v(-1.0, -1.0, 0.0),
        v(1.0, -1.0, 0.0),
        v(1.0, 1.0, 0.0),
        v(-1.0, 1.0, 0.0),
    ];
    let vbs = [v(2.0, -1.0, 0.0), v(2.0, 1.0, 0.0)];

    let input = DistanceInput {
        proxy_a: make_proxy(&vas, 0.0),
        proxy_b: make_proxy(&vbs, 0.0),
        transform: TRANSFORM_IDENTITY,
        use_radii: false,
    };

    let mut cache = SimplexCache::default();
    let output = shape_distance(&input, &mut cache, None);

    ensure_small(output.distance - 1.0, f32::EPSILON);
}

#[test]
fn shape_cast_test() {
    let vas = [
        v(-1.0, -1.0, 0.0),
        v(1.0, -1.0, 0.0),
        v(1.0, 1.0, 0.0),
        v(-1.0, 1.0, 0.0),
    ];
    let vbs = [v(2.0, -1.0, 0.0), v(2.0, 1.0, 0.0)];

    let input = ShapeCastPairInput {
        proxy_a: make_proxy(&vas, 0.0),
        proxy_b: make_proxy(&vbs, 0.0),
        transform: TRANSFORM_IDENTITY,
        translation_b: v(-2.0, 0.0, 0.0),
        max_fraction: 1.0,
        can_encroach: false,
    };

    let output = shape_cast(&input);

    assert!(output.hit);
    ensure_small(output.fraction - 0.5, 0.005);
}

#[test]
fn time_of_impact_test() {
    let vas = [
        v(-1.0, -1.0, 0.0),
        v(1.0, -1.0, 0.0),
        v(1.0, 1.0, 0.0),
        v(-1.0, 1.0, 0.0),
    ];
    let vbs = [v(2.0, -1.0, 0.0), v(2.0, 1.0, 0.0)];

    let input = ToiInput {
        proxy_a: make_proxy(&vas, 0.0),
        proxy_b: make_proxy(&vbs, 0.0),
        sweep_a: Sweep {
            local_center: VEC3_ZERO,
            c1: VEC3_ZERO,
            c2: VEC3_ZERO,
            q1: QUAT_IDENTITY,
            q2: QUAT_IDENTITY,
        },
        sweep_b: Sweep {
            local_center: VEC3_ZERO,
            c1: VEC3_ZERO,
            c2: v(-2.0, 0.0, 0.0),
            q1: QUAT_IDENTITY,
            q2: QUAT_IDENTITY,
        },
        max_fraction: 1.0,
    };

    let output = time_of_impact(&input);

    assert!(output.state == ToiState::Hit);
    ensure_small(output.fraction - 0.5, 0.005);
}
