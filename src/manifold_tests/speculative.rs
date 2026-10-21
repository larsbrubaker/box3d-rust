//! Speculative margin coverage for every manifold routine, ported from `test/test_manifold.c`
//! (upstream commit a2e4816 "Fixes for #175").
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{ensure_small, min_separation, v};
use crate::constants::{speculative_distance, MAX_MANIFOLD_POINTS};
use crate::distance::SimplexCache;
use crate::geometry::{Capsule, Sphere};
use crate::hull::{make_box_hull, make_transformed_box_hull};
use crate::manifold::{
    collide_capsule_and_sphere, collide_capsules, collide_hull_and_capsule,
    collide_hull_and_sphere, collide_hulls, collide_spheres, collide_triangle_and_capsule,
    collide_triangle_and_hull, collide_triangle_and_sphere, LocalManifold, SatCache,
};
use crate::math_functions::{Transform, QUAT_IDENTITY, VEC3_ZERO};

/// (SlideY)
fn slide_y(y: f32) -> Transform {
    Transform {
        p: v(0.0, y, 0.0),
        q: QUAT_IDENTITY,
    }
}

/// (CheckSpeculativePoints)
fn check_speculative_points(manifold: &LocalManifold, gap: f32) {
    assert!(manifold.point_count > 0, "no points at gap {gap}");
    ensure_small(min_separation(manifold) - gap, 1e-4);
}

// Contact recycling treats a pair without points as separated by at least the speculative distance.
// Every manifold routine must report a pair inside that distance or recycling can skip real overlap.
// Discussion #175.
#[test]
fn speculative_margin_test() {
    let capacity = MAX_MANIFOLD_POINTS as i32;
    let gaps = [
        0.0f32,
        0.25 * speculative_distance(),
        0.75 * speculative_distance(),
    ];

    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.25,
    };
    let big_sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    let capsule_x = Capsule {
        center1: v(-0.5, 0.0, 0.0),
        center2: v(0.5, 0.0, 0.0),
        radius: 0.25,
    };
    let capsule_z = Capsule {
        center1: v(0.0, 0.0, -0.5),
        center2: v(0.0, 0.0, 0.5),
        radius: 0.25,
    };
    let box_hull = make_box_hull(0.5, 0.5, 0.5);

    // Normal is +y
    let triangle = [v(-2.0, 0.0, -2.0), v(0.0, 0.0, 2.0), v(2.0, 0.0, -2.0)];

    let mut manifold = LocalManifold::default();

    for &gap in &gaps {
        manifold.point_count = 0;
        collide_spheres(
            &mut manifold,
            capacity,
            &big_sphere,
            &sphere,
            slide_y(0.75 + gap),
        );
        check_speculative_points(&manifold, gap);

        manifold.point_count = 0;
        collide_capsule_and_sphere(
            &mut manifold,
            capacity,
            &capsule_x,
            &sphere,
            slide_y(0.5 + gap),
        );
        check_speculative_points(&manifold, gap);

        manifold.point_count = 0;
        collide_capsules(
            &mut manifold,
            capacity,
            &capsule_x,
            &capsule_z,
            slide_y(0.5 + gap),
        );
        check_speculative_points(&manifold, gap);

        {
            manifold.point_count = 0;
            let mut cache = SimplexCache::default();
            collide_hull_and_sphere(
                &mut manifold,
                capacity,
                &box_hull.base,
                &sphere,
                slide_y(0.75 + gap),
                &mut cache,
            );
            check_speculative_points(&manifold, gap);
        }

        {
            manifold.point_count = 0;
            let mut cache = SimplexCache::default();
            collide_hull_and_capsule(
                &mut manifold,
                capacity,
                &box_hull.base,
                &capsule_x,
                slide_y(0.75 + gap),
                &mut cache,
            );
            check_speculative_points(&manifold, gap);
        }

        {
            manifold.point_count = 0;
            let mut cache = SatCache::default();
            collide_hulls(
                &mut manifold,
                capacity,
                &box_hull.base,
                &box_hull.base,
                slide_y(1.0 + gap),
                &mut cache,
            );
            check_speculative_points(&manifold, gap);
        }

        {
            manifold.point_count = 0;
            let raised = Sphere {
                center: v(0.0, 0.25 + gap, 0.0),
                radius: 0.25,
            };
            collide_triangle_and_sphere(&mut manifold, capacity, &triangle, &raised);
            check_speculative_points(&manifold, gap);
        }

        {
            manifold.point_count = 0;
            let mut raised = capsule_x;
            raised.center1.y = 0.25 + gap;
            raised.center2.y = 0.25 + gap;
            let mut cache = SimplexCache::default();
            collide_triangle_and_capsule(&mut manifold, capacity, &triangle, &raised, &mut cache);
            check_speculative_points(&manifold, gap);
        }

        {
            manifold.point_count = 0;
            let raised = make_transformed_box_hull(0.5, 0.5, 0.5, slide_y(0.5 + gap));
            let mut cache = SatCache::default();
            collide_triangle_and_hull(
                &mut manifold,
                capacity,
                triangle[0],
                triangle[1],
                triangle[2],
                0,
                &raised.base,
                &mut cache,
                true,
            );
            check_speculative_points(&manifold, gap);
        }
    }

    // Beyond the speculative distance the closed form routines report nothing
    let far_gap = 1.5 * speculative_distance();

    manifold.point_count = 0;
    collide_spheres(
        &mut manifold,
        capacity,
        &big_sphere,
        &sphere,
        slide_y(0.75 + far_gap),
    );
    assert_eq!(manifold.point_count, 0);

    manifold.point_count = 0;
    collide_capsule_and_sphere(
        &mut manifold,
        capacity,
        &capsule_x,
        &sphere,
        slide_y(0.5 + far_gap),
    );
    assert_eq!(manifold.point_count, 0);
}
