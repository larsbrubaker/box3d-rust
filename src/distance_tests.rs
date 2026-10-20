// Port of box3d-cpp-reference/test/test_distance.c
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use crate::distance::{
    make_proxy, shape_cast, shape_distance, time_of_impact, DistanceInput, ShapeCastPairInput,
    SimplexCache, Sweep, ToiInput, ToiState,
};
use crate::math_functions::{segment_distance, Vec3, QUAT_IDENTITY, TRANSFORM_IDENTITY, VEC3_ZERO};

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

// unit box with a segment resting 2mm from its +x face: inside the cast
// target distance, not overlapped
const ENCROACH_VAS: [Vec3; 4] = [
    Vec3 {
        x: -1.0,
        y: -1.0,
        z: 0.0,
    },
    Vec3 {
        x: 1.0,
        y: -1.0,
        z: 0.0,
    },
    Vec3 {
        x: 1.0,
        y: 1.0,
        z: 0.0,
    },
    Vec3 {
        x: -1.0,
        y: 1.0,
        z: 0.0,
    },
];
const ENCROACH_VBS: [Vec3; 2] = [
    Vec3 {
        x: 1.002,
        y: -1.0,
        z: 0.0,
    },
    Vec3 {
        x: 1.002,
        y: 1.0,
        z: 0.0,
    },
];
// 5mm gap segment
const ENCROACH_VBS_5MM: [Vec3; 2] = [
    Vec3 {
        x: 1.005,
        y: -1.0,
        z: 0.0,
    },
    Vec3 {
        x: 1.005,
        y: 1.0,
        z: 0.0,
    },
];

fn encroach_cast_input(translation_b: Vec3) -> ShapeCastPairInput {
    ShapeCastPairInput {
        proxy_a: make_proxy(&ENCROACH_VAS, 0.0),
        proxy_b: make_proxy(&ENCROACH_VBS, 0.0),
        transform: TRANSFORM_IDENTITY,
        translation_b,
        max_fraction: 1.0,
        can_encroach: true,
    }
}

#[test]
fn shape_cast_encroach_recede_test() {
    let input = encroach_cast_input(v(1.0, 0.0, 0.0));
    let output = shape_cast(&input);
    assert!(!output.hit);
}

#[test]
fn shape_cast_encroach_slide_test() {
    let input = encroach_cast_input(v(0.0, 2.0, 0.0));
    let output = shape_cast(&input);
    assert!(!output.hit);
}

#[test]
fn shape_cast_encroach_closing_test() {
    let input = encroach_cast_input(v(-1.0, 0.0, 0.0));
    let output = shape_cast(&input);
    assert!(output.hit);
    ensure_small(output.fraction, f32::EPSILON);
    ensure_small(output.normal.x - 1.0, 0.001);
}

#[test]
fn shape_cast_encroach_graded_test() {
    // 5mm gap: closing consumes the gap down to the retained rest separation
    let mut input = encroach_cast_input(v(-1.0, 0.0, 0.0));
    input.proxy_b = make_proxy(&ENCROACH_VBS_5MM, 0.0);
    let output = shape_cast(&input);
    assert!(output.hit);
    ensure_small(output.fraction - 0.0025, 0.0005);
    ensure_small(output.normal.x - 1.0, 0.001);
}

#[test]
fn shape_cast_encroach_max_fraction_test() {
    // 5mm gap closes at fraction 0.0025
    let mut input = encroach_cast_input(v(-1.0, 0.0, 0.0));
    input.proxy_b = make_proxy(&ENCROACH_VBS_5MM, 0.0);
    input.max_fraction = 0.002;
    let output = shape_cast(&input);
    assert!(!output.hit);

    input.max_fraction = 0.003;
    let output = shape_cast(&input);
    assert!(output.hit);
    ensure_small(output.fraction - 0.0025, 0.0005);
}

#[test]
fn shape_cast_encroach_graze_test() {
    // mostly tangential slide with a slight closing component: free
    let mut input = encroach_cast_input(v(-0.001, 2.0, 0.0));
    input.proxy_b = make_proxy(&ENCROACH_VBS_5MM, 0.0);
    let output = shape_cast(&input);
    assert!(!output.hit);
}

#[test]
fn shape_cast_start_touching_default_test() {
    let mut input = encroach_cast_input(v(1.0, 0.0, 0.0));
    input.can_encroach = false;
    let output = shape_cast(&input);
    assert!(output.hit);
}

#[test]
fn shape_cast_encroach_overlap_test() {
    let vcs = [v(0.5, -1.0, 0.0), v(0.5, 1.0, 0.0)];

    let mut input = encroach_cast_input(v(1.0, 0.0, 0.0));
    input.proxy_b = make_proxy(&vcs, 0.0);
    let output = shape_cast(&input);
    assert!(output.hit);
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
