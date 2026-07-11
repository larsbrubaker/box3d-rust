//! Sphere geometry queries. Port of box3d-cpp-reference/src/sphere.c.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{Capsule, MassData, PlaneResult, RayCastInput, ShapeCastInput, Sphere};
use crate::constants::{linear_slop, overlap_slop};
use crate::distance::{
    make_proxy, shape_cast, shape_distance, CastOutput, DistanceInput, ShapeCastPairInput,
    ShapeProxy, SimplexCache,
};
use crate::math_functions::{
    add, get_length_and_normalize, inv_mul_transforms, length_squared, make_diagonal_matrix, max,
    min, mul_add, normalize, perp, point_to_segment_distance, sub, transform_point, Aabb, Plane,
    Transform, Vec3, TRANSFORM_IDENTITY, VEC3_AXIS_Y,
};

/// Compute mass properties of a sphere. (b3ComputeSphereMass)
pub fn compute_sphere_mass(shape: &Sphere, density: f32) -> MassData {
    let center = shape.center;
    let radius = shape.radius;

    let volume = 4.0 / 3.0 * crate::math_functions::PI * radius * radius * radius;
    let mass = volume * density;
    let ixx = 0.4 * mass * radius * radius;

    MassData {
        mass,
        center,
        // Inertia about the center of mass
        inertia: make_diagonal_matrix(ixx, ixx, ixx),
    }
}

/// Compute the AABB of a sphere. (b3ComputeSphereAABB)
pub fn compute_sphere_aabb(shape: &Sphere, transform: Transform) -> Aabb {
    let center = transform_point(transform, shape.center);
    let radius = shape.radius;
    let extent = Vec3 {
        x: radius,
        y: radius,
        z: radius,
    };
    Aabb {
        lower_bound: sub(center, extent),
        upper_bound: add(center, extent),
    }
}

/// Compute the swept AABB of a sphere between two transforms.
/// (b3ComputeSweptSphereAABB)
pub fn compute_swept_sphere_aabb(shape: &Sphere, xf1: Transform, xf2: Transform) -> Aabb {
    let r = Vec3 {
        x: shape.radius,
        y: shape.radius,
        z: shape.radius,
    };
    let center1 = transform_point(xf1, shape.center);
    let center2 = transform_point(xf2, shape.center);
    Aabb {
        lower_bound: sub(min(center1, center2), r),
        upper_bound: add(max(center1, center2), r),
    }
}

/// Test overlap between a sphere and a shape proxy. (b3OverlapSphere)
pub fn overlap_sphere(
    shape: &Sphere,
    shape_transform: Transform,
    proxy: &ShapeProxy,
) -> bool {
    let input = DistanceInput {
        proxy_a: make_proxy(&[shape.center], shape.radius),
        proxy_b: *proxy,
        transform: inv_mul_transforms(shape_transform, TRANSFORM_IDENTITY),
        use_radii: true,
    };

    let mut cache = SimplexCache::default();
    let output = shape_distance(&input, &mut cache, None);
    output.distance < overlap_slop()
}

/// Ray cast versus sphere shape in local space. (b3RayCastSphere)
///
/// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
/// <http://www.codercorner.com/blog/?p=321>
pub fn ray_cast_sphere(shape: &Sphere, input: &RayCastInput) -> CastOutput {
    debug_assert!(super::is_valid_ray(input));
    let mut output = CastOutput::default();

    let p = shape.center;

    // Shift ray so sphere center is the origin
    let s = sub(input.origin, p);

    let r = shape.radius;
    let rr = r * r;

    let mut length = 0.0;
    let d = get_length_and_normalize(&mut length, input.translation);
    if length == 0.0 {
        // zero length ray

        if length_squared(s) < rr {
            // initial overlap
            output.point = input.origin;
            output.hit = true;
        }

        return output;
    }

    // Find closest point on ray to origin

    // solve: dot(s + t * d, d) = 0
    let t = -crate::math_functions::dot(s, d);

    // c is the closest point on the line to the origin
    let c = mul_add(s, t, d);

    let cc = crate::math_functions::dot(c, c);

    if cc > rr {
        // closest point is outside the sphere
        return output;
    }

    // Pythagoras
    let h = (rr - cc).sqrt();

    let fraction = t - h;

    if fraction < 0.0 || input.max_fraction * length < fraction {
        // intersection is point outside the range of the ray segment

        if length_squared(s) < rr {
            // initial overlap
            output.point = input.origin;
            output.hit = true;
        }

        return output;
    }

    let hit_point = mul_add(s, fraction, d);

    output.fraction = fraction / length;

    if output.fraction > input.max_fraction {
        // C calls b3Log here for bad input data; skip logs, keep clamp.
        output.fraction = input.max_fraction;
    }

    output.normal = normalize(hit_point);
    output.point = mul_add(p, shape.radius, output.normal);
    output.hit = true;

    output
}

/// Ray cast versus a hollow sphere (interior hits allowed).
/// (b3RayCastHollowSphere)
///
/// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
/// <http://www.codercorner.com/blog/?p=321>
pub fn ray_cast_hollow_sphere(sphere: &Sphere, input: &RayCastInput) -> CastOutput {
    let p = sphere.center;

    let mut output = CastOutput::default();

    // Shift ray so sphere center is the origin
    let s = sub(input.origin, p);
    let d = normalize(input.translation);

    // Find closest point on ray to origin

    // solve: dot(s + t * d, d) = 0
    let t = -crate::math_functions::dot(s, d);

    // c is the closest point on the line to the origin
    let c = mul_add(s, t, d);

    let cc = crate::math_functions::dot(c, c);
    let r = sphere.radius;
    let rr = r * r;

    if cc > rr {
        // closest point is outside the sphere
        return output;
    }

    // Pythagoras
    let h = (rr - cc).sqrt();

    let mut fraction = t - h;

    if fraction < 0.0 {
        fraction = t + h;
    }

    if fraction < 0.0 {
        // behind the ray
        return output;
    }

    if fraction > input.max_fraction {
        return output;
    }

    let hit_point = mul_add(s, fraction, d);

    output.fraction = fraction;
    output.normal = normalize(hit_point);
    output.point = mul_add(p, sphere.radius, output.normal);
    output.hit = true;

    output
}

/// Shape cast versus a sphere. (b3ShapeCastSphere)
pub fn shape_cast_sphere(sphere: &Sphere, input: &ShapeCastInput) -> CastOutput {
    let pair_input = ShapeCastPairInput {
        proxy_a: make_proxy(&[sphere.center], sphere.radius),
        proxy_b: input.proxy,
        transform: TRANSFORM_IDENTITY,
        translation_b: input.translation,
        max_fraction: input.max_fraction,
        can_encroach: input.can_encroach,
    };

    shape_cast(&pair_input)
}

/// Collide a capsule mover against a sphere. (b3CollideMoverAndSphere)
pub fn collide_mover_and_sphere(
    result: &mut PlaneResult,
    shape: &Sphere,
    mover: &Capsule,
) -> i32 {
    let total_radius = mover.radius + shape.radius;
    let closest = point_to_segment_distance(mover.center1, mover.center2, shape.center);

    // The normal points from the sphere toward the mover.
    let mut distance = 0.0;
    let mut normal = get_length_and_normalize(&mut distance, sub(closest, shape.center));

    if distance > total_radius {
        return 0;
    }

    let linear_slop = linear_slop();
    if distance < linear_slop {
        // Deep overlap: the mover axis passes through the sphere center, so no
        // direction is preferred. Push perpendicular to the mover axis.
        let mut length = 0.0;
        let axis = get_length_and_normalize(&mut length, sub(mover.center2, mover.center1));
        normal = if length > linear_slop {
            perp(axis)
        } else {
            VEC3_AXIS_Y
        };
        distance = 0.0;
    }

    result.plane = Plane {
        normal,
        offset: total_radius - distance,
    };
    result.point = shape.center;
    1
}
