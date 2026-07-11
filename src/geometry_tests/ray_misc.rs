use super::{
    box_hull, capsule, check_initial_overlap, ensure_small, ray_capsule, sphere, v,
};
use crate::geometry::{
    ray_cast_capsule, ray_cast_hull, ray_cast_sphere, Capsule, RayCastInput, Sphere,
};
use crate::math_functions::{mul_add, mul_sv, normalize, Vec3, VEC3_ZERO};
#[test]
fn ray_cast_overlap_convention_test() {
    let zero = VEC3_ZERO;
    let ray = v(8.0, 0.0, 0.0);
    let ray_cap = ray_capsule();
    let box_h = box_hull();

    // Sphere
    {
        let s = Sphere {
            center: v(0.0, 0.0, 0.0),
            radius: 1.0,
        };
        let inside = v(0.2, 0.0, 0.0);
        let outside = v(3.0, 0.0, 0.0);
        let moving = RayCastInput {
            origin: inside,
            translation: ray,
            max_fraction: 1.0,
        };
        let point_inside = RayCastInput {
            origin: inside,
            translation: zero,
            max_fraction: 1.0,
        };
        let point_outside = RayCastInput {
            origin: outside,
            translation: zero,
            max_fraction: 1.0,
        };
        check_initial_overlap(ray_cast_sphere(&s, &moving), inside);
        check_initial_overlap(ray_cast_sphere(&s, &point_inside), inside);
        assert!(!ray_cast_sphere(&s, &point_outside).hit);
    }

    // Capsule
    {
        let inside = v(0.0, 0.0, 0.0);
        let outside = v(0.0, 3.0, 0.0);
        let moving = RayCastInput {
            origin: inside,
            translation: ray,
            max_fraction: 1.0,
        };
        let point_inside = RayCastInput {
            origin: inside,
            translation: zero,
            max_fraction: 1.0,
        };
        let point_outside = RayCastInput {
            origin: outside,
            translation: zero,
            max_fraction: 1.0,
        };
        check_initial_overlap(ray_cast_capsule(&ray_cap, &moving), inside);
        check_initial_overlap(ray_cast_capsule(&ray_cap, &point_inside), inside);
        assert!(!ray_cast_capsule(&ray_cap, &point_outside).hit);
    }

    // Hull
    {
        let inside = v(0.3, 0.2, 0.1);
        let outside = v(3.0, 0.0, 0.0);
        let moving = RayCastInput {
            origin: inside,
            translation: ray,
            max_fraction: 1.0,
        };
        let point_inside = RayCastInput {
            origin: inside,
            translation: zero,
            max_fraction: 1.0,
        };
        let point_outside = RayCastInput {
            origin: outside,
            translation: zero,
            max_fraction: 1.0,
        };
        check_initial_overlap(ray_cast_hull(&box_h.base, &moving), inside);
        check_initial_overlap(ray_cast_hull(&box_h.base, &point_inside), inside);
        assert!(!ray_cast_hull(&box_h.base, &point_outside).hit);
    }
}

/// Distance, in double precision, from a single precision hit point to the analytic first
/// ray/sphere intersection of the same float ray.
fn sphere_hit_error(shape: Sphere, input: RayCastInput, point: Vec3) -> f64 {
    let r = shape.radius as f64;
    let sx = input.origin.x as f64 - shape.center.x as f64;
    let sy = input.origin.y as f64 - shape.center.y as f64;
    let sz = input.origin.z as f64 - shape.center.z as f64;
    let tx = input.translation.x as f64;
    let ty = input.translation.y as f64;
    let tz = input.translation.z as f64;
    let len = (tx * tx + ty * ty + tz * tz).sqrt();
    let dx = tx / len;
    let dy = ty / len;
    let dz = tz / len;
    let b = sx * dx + sy * dy + sz * dz;
    let c = sx * sx + sy * sy + sz * sz - r * r;
    let t = -b - (b * b - c).sqrt();
    let ex = point.x as f64 - (input.origin.x as f64 + t * dx);
    let ey = point.y as f64 - (input.origin.y as f64 + t * dy);
    let ez = point.z as f64 - (input.origin.z as f64 + t * dz);
    (ex * ex + ey * ey + ez * ez).sqrt()
}

/// Same idea for the ray/infinite-cylinder intersection, used where the hit lands on the side.
fn capsule_hit_error(shape: Capsule, input: RayCastInput, point: Vec3) -> f64 {
    let mut ax = shape.center2.x as f64 - shape.center1.x as f64;
    let mut ay = shape.center2.y as f64 - shape.center1.y as f64;
    let mut az = shape.center2.z as f64 - shape.center1.z as f64;
    let alen = (ax * ax + ay * ay + az * az).sqrt();
    ax /= alen;
    ay /= alen;
    az /= alen;

    let sx = input.origin.x as f64 - shape.center1.x as f64;
    let sy = input.origin.y as f64 - shape.center1.y as f64;
    let sz = input.origin.z as f64 - shape.center1.z as f64;
    let tx = input.translation.x as f64;
    let ty = input.translation.y as f64;
    let tz = input.translation.z as f64;
    let tlen = (tx * tx + ty * ty + tz * tz).sqrt();
    let dx = tx / tlen;
    let dy = ty / tlen;
    let dz = tz / tlen;

    let sa = sx * ax + sy * ay + sz * az;
    let da = dx * ax + dy * ay + dz * az;
    let spx = sx - sa * ax;
    let spy = sy - sa * ay;
    let spz = sz - sa * az;
    let dpx = dx - da * ax;
    let dpy = dy - da * ay;
    let dpz = dz - da * az;
    let a = dpx * dpx + dpy * dpy + dpz * dpz;
    let b = 2.0 * (spx * dpx + spy * dpy + spz * dpz);
    let r = shape.radius as f64;
    let c = spx * spx + spy * spy + spz * spz - r * r;
    let tau = (-b - (b * b - 4.0 * a * c).sqrt()) / (2.0 * a);
    let ex = point.x as f64 - (input.origin.x as f64 + tau * dx);
    let ey = point.y as f64 - (input.origin.y as f64 + tau * dy);
    let ez = point.z as f64 - (input.origin.z as f64 + tau * dz);
    (ex * ex + ey * ey + ez * ez).sqrt()
}

const RAY_MISS: f64 = 1.0e30;

#[test]
fn ray_cast_far_origin_test() {
    let s = Sphere {
        center: v(0.0, 0.0, 0.0),
        radius: 1.0,
    };
    let c = Capsule {
        center1: v(-2.0, 0.0, 0.0),
        center2: v(2.0, 0.0, 0.0),
        radius: 1.0,
    };

    // (0,0,1) lies on the unit sphere and on the capsule side. The ray dives in from a far origin
    // along H + D*u for a fan of directions u skewed off the surface normal, so the capsule solve
    // sees a real perpendicular gap rather than a free exact cancellation.
    let h = v(0.0, 0.0, 1.0);
    let offsets = [-0.7f32, 0.0, 0.7];
    let distances = [1e1f32, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7];

    let mut worst_sphere = [0.0f64; 7];
    let mut worst_capsule = [0.0f64; 7];

    println!("    worst hit point error over a fan of skew rays, by origin distance:");
    println!("    {:<9} {:<13} {:<13}", "distance", "sphere", "capsule");
    for i in 0..distances.len() {
        let d = distances[i];
        let mut max_s = 0.0f64;
        let mut max_c = 0.0f64;

        for &oa in &offsets {
            for &ob in &offsets {
                let u = normalize(v(oa, ob, 1.0));
                let origin = mul_add(h, d, u);
                let translation = mul_sv(-2.0 * d, u);
                let input = RayCastInput {
                    origin,
                    translation,
                    max_fraction: 1.0,
                };

                let os = ray_cast_sphere(&s, &input);
                let oc = ray_cast_capsule(&c, &input);

                let err_s = if os.hit {
                    sphere_hit_error(s, input, os.point)
                } else {
                    RAY_MISS
                };
                let err_c = if oc.hit {
                    capsule_hit_error(c, input, oc.point)
                } else {
                    RAY_MISS
                };

                if err_s > max_s {
                    max_s = err_s;
                }
                if err_c > max_c {
                    max_c = err_c;
                }
            }
        }

        worst_sphere[i] = max_s;
        worst_capsule[i] = max_c;
        println!("    {:<9.0e} {:<13.3e} {:<13.3e}", d, max_s, max_c);
    }

    // The closest point formulation keeps the error at the single precision floor: it grows only
    // linearly with origin distance, error ~ distance * FLT_EPSILON, with no catastrophic loss. This
    // holds out to a million units. At ten million the origin coordinates carry a meter sized ULP,
    // larger than the unit radius, so the ray genuinely drops the hit. That row is printed for insight
    // but left unasserted rather than baking in the breakdown.
    for i in 0..distances.len() {
        if distances[i] < 1.0e7 {
            let floor = 16.0 * distances[i] as f64 * f64::from(f32::EPSILON) + 2.0e-6;
            assert!(
                worst_sphere[i] < floor,
                "sphere error {} >= floor {} at distance {}",
                worst_sphere[i],
                floor,
                distances[i]
            );
            assert!(
                worst_capsule[i] < floor,
                "capsule error {} >= floor {} at distance {}",
                worst_capsule[i],
                floor,
                distances[i]
            );
        }
    }

    // Still a clean sub-meter hit at a million units out.
    assert!(worst_sphere[5] < 0.5 && worst_capsule[5] < 0.5);
}

#[test]
fn ray_cast_shape_test() {
    let input = RayCastInput {
        origin: v(-4.0, 0.0, 0.0),
        translation: v(8.0, 0.0, 0.0),
        max_fraction: 1.0,
    };
    let box_h = box_hull();

    {
        let output = ray_cast_sphere(&sphere(), &input);
        assert!(output.hit);
        ensure_small(output.normal.x + 1.0, f32::EPSILON);
        ensure_small(output.normal.y, f32::EPSILON);
        ensure_small(output.normal.z, f32::EPSILON);
        ensure_small(output.fraction - 0.5, f32::EPSILON);
    }

    {
        let output = ray_cast_capsule(&capsule(), &input);
        assert!(output.hit);
        ensure_small(output.normal.x + 1.0, f32::EPSILON);
        ensure_small(output.normal.y, f32::EPSILON);
        ensure_small(output.normal.z, f32::EPSILON);
        ensure_small(output.fraction - 1.0 / 4.0, f32::EPSILON);
    }

    {
        let output = ray_cast_hull(&box_h.base, &input);
        assert!(output.hit);
        ensure_small(output.normal.x + 1.0, f32::EPSILON);
        ensure_small(output.normal.y, f32::EPSILON);
        ensure_small(output.normal.z, f32::EPSILON);
        ensure_small(output.fraction - 3.0 / 8.0, f32::EPSILON);
    }
}

#[test]
fn is_valid_ray_rejects_bad_input() {
    use crate::geometry::is_valid_ray;

    let good = RayCastInput {
        origin: v(0.0, 0.0, 0.0),
        translation: v(1.0, 0.0, 0.0),
        max_fraction: 1.0,
    };
    assert!(is_valid_ray(&good));

    let bad_frac = RayCastInput {
        max_fraction: -0.1,
        ..good
    };
    assert!(!is_valid_ray(&bad_frac));
}
