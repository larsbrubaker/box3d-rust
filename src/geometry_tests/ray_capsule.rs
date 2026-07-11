use super::{check_cast_hit, ensure_small, ray_capsule, v};
use crate::geometry::{ray_cast_capsule, Capsule, RayCastInput};
use crate::math_functions::{distance, mul_add, point_to_segment_distance};
#[test]
fn ray_cast_capsule_side_test() {
    let ray_cap = ray_capsule();

    // Perpendicular hit on the cylindrical side. Surface at distance 2 over a length 6 ray.
    {
        let input = RayCastInput {
            origin: v(0.0, 3.0, 0.0),
            translation: v(0.0, -6.0, 0.0),
            max_fraction: 1.0,
        };
        let out = ray_cast_capsule(&ray_cap, &input);
        check_cast_hit(
            out,
            input.origin,
            input.translation,
            v(0.0, 1.0, 0.0),
            v(0.0, 1.0, 0.0),
            1.0 / 3.0,
            1e-5,
        );
    }
    // Same from +z to exercise the other transverse direction.
    {
        let input = RayCastInput {
            origin: v(0.0, 0.0, 3.0),
            translation: v(0.0, 0.0, -6.0),
            max_fraction: 1.0,
        };
        let out = ray_cast_capsule(&ray_cap, &input);
        check_cast_hit(
            out,
            input.origin,
            input.translation,
            v(0.0, 0.0, 1.0),
            v(0.0, 0.0, 1.0),
            1.0 / 3.0,
            1e-5,
        );
    }
    // Side hit nearer the c1 end.
    {
        let input = RayCastInput {
            origin: v(-1.0, 3.0, 0.0),
            translation: v(0.0, -6.0, 0.0),
            max_fraction: 1.0,
        };
        let out = ray_cast_capsule(&ray_cap, &input);
        check_cast_hit(
            out,
            input.origin,
            input.translation,
            v(-1.0, 1.0, 0.0),
            v(0.0, 1.0, 0.0),
            1.0 / 3.0,
            1e-5,
        );
    }
}

#[test]
fn ray_cast_capsule_oblique_test() {
    let ray_cap = ray_capsule();
    // Oblique ray in the z=0 plane. It crosses y=1 inside the cylinder span, so the
    // normal stays transverse. Exercises the non perpendicular ray/axis solve where
    // dot(axis, rayAxis) != 0.
    let input = RayCastInput {
        origin: v(-3.0, 3.0, 0.0),
        translation: v(4.0, -4.0, 0.0),
        max_fraction: 1.0,
    };
    let out = ray_cast_capsule(&ray_cap, &input);
    check_cast_hit(
        out,
        input.origin,
        input.translation,
        v(-1.0, 1.0, 0.0),
        v(0.0, 1.0, 0.0),
        0.5,
        1e-4,
    );
}

#[test]
fn ray_cast_capsule_cap_test() {
    let ray_cap = ray_capsule();
    let k = 0.70710678;

    // Collinear ray hits the c2 hemisphere from beyond the end.
    {
        let input = RayCastInput {
            origin: v(5.0, 0.0, 0.0),
            translation: v(-8.0, 0.0, 0.0),
            max_fraction: 1.0,
        };
        let out = ray_cast_capsule(&ray_cap, &input);
        check_cast_hit(
            out,
            input.origin,
            input.translation,
            v(3.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            1.0 / 4.0,
            1e-5,
        );
    }
    // Off-axis ray through the c2 cap center, approaching from outside the cylinder.
    {
        let input = RayCastInput {
            origin: v(4.0, 2.0, 0.0),
            translation: v(-4.0, -4.0, 0.0),
            max_fraction: 1.0,
        };
        let out = ray_cast_capsule(&ray_cap, &input);
        check_cast_hit(
            out,
            input.origin,
            input.translation,
            v(2.0 + k, k, 0.0),
            v(k, k, 0.0),
            0.323223,
            1e-4,
        );
    }
    // Mirror through the c1 cap center.
    {
        let input = RayCastInput {
            origin: v(-4.0, 2.0, 0.0),
            translation: v(4.0, -4.0, 0.0),
            max_fraction: 1.0,
        };
        let out = ray_cast_capsule(&ray_cap, &input);
        check_cast_hit(
            out,
            input.origin,
            input.translation,
            v(-2.0 - k, k, 0.0),
            v(-k, k, 0.0),
            0.323223,
            1e-4,
        );
    }
}

#[test]
fn ray_cast_capsule_miss_test() {
    let ray_cap = ray_capsule();

    // Pointing away.
    {
        let input = RayCastInput {
            origin: v(0.0, 3.0, 0.0),
            translation: v(0.0, 4.0, 0.0),
            max_fraction: 1.0,
        };
        assert!(!ray_cast_capsule(&ray_cap, &input).hit);
    }
    // Crosses above the axis more than a radius away.
    {
        let input = RayCastInput {
            origin: v(0.0, 4.0, 2.0),
            translation: v(0.0, -8.0, 0.0),
            max_fraction: 1.0,
        };
        assert!(!ray_cast_capsule(&ray_cap, &input).hit);
    }
    // Aimed at the side but the translation stops short.
    {
        let input = RayCastInput {
            origin: v(0.0, 5.0, 0.0),
            translation: v(0.0, -1.0, 0.0),
            max_fraction: 1.0,
        };
        assert!(!ray_cast_capsule(&ray_cap, &input).hit);
    }
    // Parallel to the axis and outside the cylinder.
    {
        let input = RayCastInput {
            origin: v(0.0, 3.0, 0.0),
            translation: v(8.0, 0.0, 0.0),
            max_fraction: 1.0,
        };
        assert!(!ray_cast_capsule(&ray_cap, &input).hit);
    }
    // Descends past the rounded c2 end, beyond cap reach.
    {
        let input = RayCastInput {
            origin: v(4.0, 3.0, 0.0),
            translation: v(0.0, -6.0, 0.0),
            max_fraction: 1.0,
        };
        assert!(!ray_cast_capsule(&ray_cap, &input).hit);
    }
}

#[test]
fn ray_cast_capsule_interior_test() {
    let ray_cap = ray_capsule();

    // Origin on the axis between the caps.
    {
        let input = RayCastInput {
            origin: v(0.0, 0.0, 0.0),
            translation: v(0.0, -5.0, 0.0),
            max_fraction: 1.0,
        };
        let out = ray_cast_capsule(&ray_cap, &input);
        assert!(out.hit);
        assert_eq!(out.fraction, 0.0);
        ensure_small(distance(out.point, input.origin), f32::EPSILON);
    }
    // Origin inside the c2 hemisphere, past the cylinder end.
    {
        let input = RayCastInput {
            origin: v(2.5, 0.0, 0.0),
            translation: v(0.0, 0.0, 5.0),
            max_fraction: 1.0,
        };
        let out = ray_cast_capsule(&ray_cap, &input);
        assert!(out.hit);
        assert_eq!(out.fraction, 0.0);
        ensure_small(distance(out.point, input.origin), f32::EPSILON);
    }
    // Zero length ray inside.
    {
        let input = RayCastInput {
            origin: v(0.0, 0.0, 0.0),
            translation: v(0.0, 0.0, 0.0),
            max_fraction: 1.0,
        };
        let out = ray_cast_capsule(&ray_cap, &input);
        assert!(out.hit);
        ensure_small(distance(out.point, input.origin), f32::EPSILON);
    }
    // Zero length ray outside.
    {
        let input = RayCastInput {
            origin: v(0.0, 3.0, 0.0),
            translation: v(0.0, 0.0, 0.0),
            max_fraction: 1.0,
        };
        assert!(!ray_cast_capsule(&ray_cap, &input).hit);
    }
}

#[test]
fn ray_cast_capsule_degenerate_test() {
    // Coincident centers collapse to a sphere.
    let c = Capsule {
        center1: v(0.0, 0.0, 0.0),
        center2: v(0.0, 0.0, 0.0),
        radius: 1.0,
    };
    let input = RayCastInput {
        origin: v(-4.0, 0.0, 0.0),
        translation: v(8.0, 0.0, 0.0),
        max_fraction: 1.0,
    };
    let out = ray_cast_capsule(&c, &input);
    check_cast_hit(
        out,
        input.origin,
        input.translation,
        v(-1.0, 0.0, 0.0),
        v(-1.0, 0.0, 0.0),
        3.0 / 8.0,
        1e-5,
    );
}

#[test]
fn ray_cast_capsule_clip_test() {
    let ray_cap = ray_capsule();

    // The side hit occurs at fraction 1/3. Straddle it with maxFraction.
    {
        let input = RayCastInput {
            origin: v(0.0, 3.0, 0.0),
            translation: v(0.0, -6.0, 0.0),
            max_fraction: 0.3,
        };
        assert!(!ray_cast_capsule(&ray_cap, &input).hit);
    }
    {
        let input = RayCastInput {
            origin: v(0.0, 3.0, 0.0),
            translation: v(0.0, -6.0, 0.0),
            max_fraction: 0.5,
        };
        let out = ray_cast_capsule(&ray_cap, &input);
        assert!(out.hit);
        ensure_small(out.fraction - 1.0 / 3.0, 1e-5);
    }
}

#[test]
fn ray_cast_capsule_parallel_test() {
    let ray_cap = ray_capsule();

    // Capsule along y. A long ray almost parallel to the axis drifts inward from just outside the
    // cylinder and dips through the far endcap. The naive solve loses this hit to a determinant of zero.
    let axis_y = Capsule {
        center1: v(0.0, 0.0, 0.0),
        center2: v(0.0, 10.0, 0.0),
        radius: 1.0,
    };
    {
        let input = RayCastInput {
            origin: v(1.0001, 100.0, 0.0),
            translation: v(-0.001, -200.0, 0.0),
            max_fraction: 1.0,
        };
        let out = ray_cast_capsule(&axis_y, &input);
        assert!(out.hit);

        // The hit lands on the capsule surface and on the ray.
        let on_seg = point_to_segment_distance(axis_y.center1, axis_y.center2, out.point);
        ensure_small(distance(out.point, on_seg) - axis_y.radius, 1e-3);
        let on_ray = mul_add(input.origin, out.fraction, input.translation);
        ensure_small(distance(out.point, on_ray), 1e-3);
    }

    // Near parallel ray converging onto the x-axis capsule from far away.
    {
        let input = RayCastInput {
            origin: v(-1000.0, 1.0001, 0.0),
            translation: v(2000.0, -0.001, 0.0),
            max_fraction: 1.0,
        };
        let out = ray_cast_capsule(&ray_cap, &input);
        assert!(out.hit);
        let on_seg = point_to_segment_distance(ray_cap.center1, ray_cap.center2, out.point);
        ensure_small(distance(out.point, on_seg) - ray_cap.radius, 1e-3);
    }

    // Exactly parallel and outside the cylinder still misses.
    {
        let input = RayCastInput {
            origin: v(0.0, 3.0, 0.0),
            translation: v(8.0, 0.0, 0.0),
            max_fraction: 1.0,
        };
        assert!(!ray_cast_capsule(&ray_cap, &input).hit);
    }
}
