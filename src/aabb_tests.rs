// Port of the AABB subtests from box3d-cpp-reference/test/test_collision.c
// (AABBTest and TestRayAABBIntersection). The LargeWorld* subtests depend on
// hull/shape modules not yet ported and will be added with them.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::aabb::ray_cast_aabb;
use crate::math_functions::{
    aabb_contains, aabb_overlaps, abs_float, is_valid_aabb, Aabb, Vec3,
};

fn aabb(lx: f32, ly: f32, lz: f32, ux: f32, uy: f32, uz: f32) -> Aabb {
    Aabb {
        lower_bound: Vec3 {
            x: lx,
            y: ly,
            z: lz,
        },
        upper_bound: Vec3 {
            x: ux,
            y: uy,
            z: uz,
        },
    }
}

fn v(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

/// AABBTest from test_collision.c
#[test]
fn aabb_validity_overlap_contains() {
    let mut a = Aabb {
        lower_bound: v(-1.0, -1.0, -1.0),
        // C: a.upperBound = { -2, -2, -2 }
        upper_bound: v(-2.0, -2.0, -2.0),
    };
    assert!(!is_valid_aabb(a));

    // C: a.upperBound = (b3Vec3){ 1.0f, 1.0f }; — z zero-initialized
    a.upper_bound = v(1.0, 1.0, 0.0);
    assert!(is_valid_aabb(a));

    // C: b3AABB b = { { 2, 2 }, { 4, 4 } }; — z components zero-initialized
    let b = aabb(2.0, 2.0, 0.0, 4.0, 4.0, 0.0);
    assert!(!aabb_overlaps(a, b));
    assert!(!aabb_contains(a, b));
}

/// TestRayAABBIntersection from test_collision.c
#[test]
fn test_ray_aabb_intersection() {
    let mut min_fraction = 0.0;
    let mut max_fraction = 0.0;

    // Test 1: Ray passing through center of AABB
    {
        let a = aabb(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
        let hit = ray_cast_aabb(a, v(-2.0, 0.0, 0.0), v(2.0, 0.0, 0.0), &mut min_fraction, &mut max_fraction);
        assert!(hit);
        assert!(abs_float(min_fraction - 0.25) < 0.001); // Enters at 25% of ray
        assert!(abs_float(max_fraction - 0.75) < 0.001); // Exits at 75% of ray
    }

    // Test 2: Ray starting inside AABB
    {
        let a = aabb(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
        let hit = ray_cast_aabb(a, v(0.0, 0.0, 0.0), v(2.0, 0.0, 0.0), &mut min_fraction, &mut max_fraction);
        assert!(hit);
        assert_eq!(min_fraction, 0.0); // Starts inside
        assert!(abs_float(max_fraction - 0.5) < 0.001); // Exits at 50% of ray
    }

    // Test 3: Ray ending inside AABB
    {
        let a = aabb(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
        let hit = ray_cast_aabb(a, v(-2.0, 0.0, 0.0), v(0.0, 0.0, 0.0), &mut min_fraction, &mut max_fraction);
        assert!(hit);
        assert!(abs_float(min_fraction - 0.5) < 0.001); // Enters at 50% of ray
        assert_eq!(max_fraction, 1.0); // Ends inside
    }

    // Test 4: Ray completely inside AABB
    {
        let a = aabb(-2.0, -2.0, -2.0, 2.0, 2.0, 2.0);
        let hit = ray_cast_aabb(a, v(-1.0, 0.0, 0.0), v(1.0, 0.0, 0.0), &mut min_fraction, &mut max_fraction);
        assert!(hit);
        assert_eq!(min_fraction, 0.0);
        assert_eq!(max_fraction, 1.0);
    }

    // Test 5: Ray missing AABB
    {
        let a = aabb(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        let hit = ray_cast_aabb(a, v(-1.0, 2.0, 0.5), v(2.0, 2.0, 0.5), &mut min_fraction, &mut max_fraction);
        assert!(!hit);
    }

    // Test 6: Ray parallel to AABB face (no intersection)
    {
        let a = aabb(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        let hit = ray_cast_aabb(a, v(-1.0, 2.0, 0.5), v(2.0, 2.0, 0.5), &mut min_fraction, &mut max_fraction);
        assert!(!hit);
    }

    // Test 7: Ray parallel to AABB face (within bounds)
    {
        let a = aabb(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        let hit = ray_cast_aabb(a, v(-1.0, 0.5, 0.5), v(2.0, 0.5, 0.5), &mut min_fraction, &mut max_fraction);
        assert!(hit);
        assert!(abs_float(min_fraction - 1.0 / 3.0) < 0.001);
        assert!(abs_float(max_fraction - 2.0 / 3.0) < 0.001);
    }

    // Test 8: Degenerate ray (point) inside AABB
    {
        let a = aabb(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        let hit = ray_cast_aabb(a, v(0.5, 0.5, 0.5), v(0.5, 0.5, 0.5), &mut min_fraction, &mut max_fraction);
        assert!(hit);
        assert_eq!(min_fraction, 0.0);
        assert_eq!(max_fraction, 0.0);
    }

    // Test 9: Degenerate ray (point) outside AABB
    {
        let a = aabb(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        let hit = ray_cast_aabb(a, v(2.0, 2.0, 2.0), v(2.0, 2.0, 2.0), &mut min_fraction, &mut max_fraction);
        assert!(!hit);
    }

    // Test 10: Ray pointing away from AABB
    {
        let a = aabb(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        let hit = ray_cast_aabb(a, v(-1.0, 0.5, 0.5), v(-2.0, 0.5, 0.5), &mut min_fraction, &mut max_fraction);
        assert!(!hit);
    }

    // Test 11: Ray hitting corner of AABB
    {
        let a = aabb(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        let hit = ray_cast_aabb(a, v(-1.0, -1.0, -1.0), v(2.0, 2.0, 2.0), &mut min_fraction, &mut max_fraction);
        assert!(hit);
        assert!(abs_float(min_fraction - 1.0 / 3.0) < 0.001);
        assert!(abs_float(max_fraction - 2.0 / 3.0) < 0.001);
    }

    // Test 12: Ray grazing edge of AABB
    {
        let a = aabb(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        let hit = ray_cast_aabb(a, v(-1.0, 0.0, 0.5), v(2.0, 0.0, 0.5), &mut min_fraction, &mut max_fraction);
        assert!(hit);
        assert!(abs_float(min_fraction - 1.0 / 3.0) < 0.001);
        assert!(abs_float(max_fraction - 2.0 / 3.0) < 0.001);
    }
}
