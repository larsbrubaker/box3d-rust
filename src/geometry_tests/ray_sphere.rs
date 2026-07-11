use super::{check_cast_hit, ensure_small, v};
use crate::geometry::{ray_cast_sphere, RayCastInput, Sphere};
use crate::math_functions::distance;
#[test]
fn ray_cast_sphere_hit_test() {
    let s = Sphere {
        center: v(0.0, 0.0, 0.0),
        radius: 1.0,
    };

    // Hit along each principal axis. Surface at distance 3 over a length 8 ray.
    {
        let input = RayCastInput {
            origin: v(-4.0, 0.0, 0.0),
            translation: v(8.0, 0.0, 0.0),
            max_fraction: 1.0,
        };
        let out = ray_cast_sphere(&s, &input);
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
    {
        let input = RayCastInput {
            origin: v(0.0, 4.0, 0.0),
            translation: v(0.0, -8.0, 0.0),
            max_fraction: 1.0,
        };
        let out = ray_cast_sphere(&s, &input);
        check_cast_hit(
            out,
            input.origin,
            input.translation,
            v(0.0, 1.0, 0.0),
            v(0.0, 1.0, 0.0),
            3.0 / 8.0,
            1e-5,
        );
    }
    {
        let input = RayCastInput {
            origin: v(0.0, 0.0, -4.0),
            translation: v(0.0, 0.0, 8.0),
            max_fraction: 1.0,
        };
        let out = ray_cast_sphere(&s, &input);
        check_cast_hit(
            out,
            input.origin,
            input.translation,
            v(0.0, 0.0, -1.0),
            v(0.0, 0.0, -1.0),
            3.0 / 8.0,
            1e-5,
        );
    }

    // Offset center, hit partway along the ray.
    {
        let s2 = Sphere {
            center: v(5.0, 0.0, 0.0),
            radius: 2.0,
        };
        let input = RayCastInput {
            origin: v(0.0, 0.0, 0.0),
            translation: v(10.0, 0.0, 0.0),
            max_fraction: 1.0,
        };
        let out = ray_cast_sphere(&s2, &input);
        check_cast_hit(
            out,
            input.origin,
            input.translation,
            v(3.0, 0.0, 0.0),
            v(-1.0, 0.0, 0.0),
            0.3,
            1e-5,
        );
    }

    // Diagonal ray straight through the center.
    {
        let k = 0.70710678;
        let input = RayCastInput {
            origin: v(-3.0, -3.0, 0.0),
            translation: v(6.0, 6.0, 0.0),
            max_fraction: 1.0,
        };
        let out = ray_cast_sphere(&s, &input);
        check_cast_hit(
            out,
            input.origin,
            input.translation,
            v(-k, -k, 0.0),
            v(-k, -k, 0.0),
            0.382149,
            1e-4,
        );
    }
}

#[test]
fn ray_cast_sphere_miss_test() {
    let s = Sphere {
        center: v(0.0, 0.0, 0.0),
        radius: 1.0,
    };

    // Pointing away.
    {
        let input = RayCastInput {
            origin: v(-4.0, 0.0, 0.0),
            translation: v(-8.0, 0.0, 0.0),
            max_fraction: 1.0,
        };
        assert!(!ray_cast_sphere(&s, &input).hit);
    }
    // Passes wide of the sphere.
    {
        let input = RayCastInput {
            origin: v(-4.0, 3.0, 0.0),
            translation: v(8.0, 0.0, 0.0),
            max_fraction: 1.0,
        };
        assert!(!ray_cast_sphere(&s, &input).hit);
    }
    // Aimed at the sphere but the translation stops short.
    {
        let input = RayCastInput {
            origin: v(-4.0, 0.0, 0.0),
            translation: v(8.0, 0.0, 0.0),
            max_fraction: 0.3,
        };
        assert!(!ray_cast_sphere(&s, &input).hit);
    }
}

#[test]
fn ray_cast_sphere_clip_test() {
    let s = Sphere {
        center: v(0.0, 0.0, 0.0),
        radius: 1.0,
    };

    // The surface is reached at fraction 3/8. Straddle it with maxFraction.
    {
        let input = RayCastInput {
            origin: v(-4.0, 0.0, 0.0),
            translation: v(8.0, 0.0, 0.0),
            max_fraction: 0.374,
        };
        assert!(!ray_cast_sphere(&s, &input).hit);
    }
    {
        let input = RayCastInput {
            origin: v(-4.0, 0.0, 0.0),
            translation: v(8.0, 0.0, 0.0),
            max_fraction: 0.376,
        };
        let out = ray_cast_sphere(&s, &input);
        assert!(out.hit);
        ensure_small(out.fraction - 3.0 / 8.0, 1e-5);
    }
}

#[test]
fn ray_cast_sphere_interior_test() {
    let s = Sphere {
        center: v(0.0, 0.0, 0.0),
        radius: 1.0,
    };

    // Origin inside reports the origin with zero fraction.
    {
        let input = RayCastInput {
            origin: v(0.3, 0.0, 0.0),
            translation: v(8.0, 0.0, 0.0),
            max_fraction: 1.0,
        };
        let out = ray_cast_sphere(&s, &input);
        assert!(out.hit);
        assert_eq!(out.fraction, 0.0);
        ensure_small(distance(out.point, input.origin), f32::EPSILON);
    }
    // Zero length ray inside.
    {
        let input = RayCastInput {
            origin: v(0.5, 0.0, 0.0),
            translation: v(0.0, 0.0, 0.0),
            max_fraction: 1.0,
        };
        let out = ray_cast_sphere(&s, &input);
        assert!(out.hit);
        ensure_small(distance(out.point, input.origin), f32::EPSILON);
    }
    // Zero length ray outside.
    {
        let input = RayCastInput {
            origin: v(3.0, 0.0, 0.0),
            translation: v(0.0, 0.0, 0.0),
            max_fraction: 1.0,
        };
        assert!(!ray_cast_sphere(&s, &input).hit);
    }
}

#[test]
fn ray_cast_sphere_graze_test() {
    let s = Sphere {
        center: v(0.0, 0.0, 0.0),
        radius: 1.0,
    };

    // Just inside the radius grazes a hit, just outside misses.
    {
        let input = RayCastInput {
            origin: v(-4.0, 0.999, 0.0),
            translation: v(8.0, 0.0, 0.0),
            max_fraction: 1.0,
        };
        assert!(ray_cast_sphere(&s, &input).hit);
    }
    {
        let input = RayCastInput {
            origin: v(-4.0, 1.001, 0.0),
            translation: v(8.0, 0.0, 0.0),
            max_fraction: 1.0,
        };
        assert!(!ray_cast_sphere(&s, &input).hit);
    }
}
