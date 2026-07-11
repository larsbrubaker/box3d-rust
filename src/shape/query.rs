//! Shape query dispatch: projected area, proxy, ray/shape cast, and overlap.
//! Port of the corresponding functions from box3d-cpp-reference/src/shape.c.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{Shape, ShapeGeometry};
use crate::compound::{overlap_compound, ray_cast_compound, shape_cast_compound};
use crate::constants::MAX_SHAPE_CAST_POINTS;
use crate::distance::{make_proxy, CastOutput, ShapeProxy};
use crate::geometry::{
    overlap_capsule, overlap_sphere, ray_cast_capsule, ray_cast_sphere, shape_cast_capsule,
    shape_cast_sphere, RayCastInput, ShapeCastInput,
};
use crate::height_field::{
    overlap_height_field, ray_cast_height_field, shape_cast_height_field,
};
use crate::hull::{
    compute_hull_projected_area, get_hull_points, overlap_hull, ray_cast_hull, shape_cast_hull,
};
use crate::math_functions::{
    cross, inv_rotate_vector, inv_transform_point, length, min_int, rotate_vector, sub,
    transform_point, Transform, Vec3, PI,
};
use crate::mesh::{overlap_mesh, ray_cast_mesh, shape_cast_mesh, Mesh};

/// Projected area of a shape onto a plane with the given normal.
/// Used by explosions. (b3GetShapeProjectedArea)
pub fn get_shape_projected_area(shape: &Shape, plane_normal: Vec3) -> f32 {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            let radius = capsule.radius;
            let axis = sub(capsule.center2, capsule.center1);
            let projected_length = length(cross(axis, plane_normal));
            let cylinder_area = 2.0 * radius * projected_length;
            let sphere_area = PI * radius * radius;
            sphere_area + cylinder_area
        }
        ShapeGeometry::Hull(hull) => compute_hull_projected_area(hull, plane_normal),
        ShapeGeometry::Sphere(sphere) => PI * sphere.radius * sphere.radius,
        _ => 0.0,
    }
}

/// Make a GJK shape proxy for a convex shape. (b3MakeShapeProxy)
pub fn make_shape_proxy(shape: &Shape) -> ShapeProxy {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            make_proxy(&[capsule.center1, capsule.center2], capsule.radius)
        }
        ShapeGeometry::Sphere(sphere) => make_proxy(&[sphere.center], sphere.radius),
        ShapeGeometry::Hull(hull) => {
            let points = get_hull_points(hull);
            make_proxy(points, 0.0)
        }
        _ => {
            debug_assert!(false, "make_shape_proxy only supports capsule/sphere/hull");
            ShapeProxy::default()
        }
    }
}

/// Ray cast a shape in world (or relative) space. Transforms the ray into
/// local space, dispatches, then transforms the hit back. (b3RayCastShape)
pub fn ray_cast_shape(
    shape: &Shape,
    transform: Transform,
    input: &RayCastInput,
) -> CastOutput {
    let local_input = RayCastInput {
        origin: inv_transform_point(transform, input.origin),
        translation: inv_rotate_vector(transform.q, input.translation),
        max_fraction: input.max_fraction,
    };

    let mut output = match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => ray_cast_capsule(capsule, &local_input),
        ShapeGeometry::Compound(compound) => ray_cast_compound(compound, &local_input),
        ShapeGeometry::Sphere(sphere) => ray_cast_sphere(sphere, &local_input),
        ShapeGeometry::Hull(hull) => ray_cast_hull(hull, &local_input),
        ShapeGeometry::Mesh { data, scale } => {
            let mesh = Mesh::new(data, *scale);
            ray_cast_mesh(&mesh, &local_input)
        }
        ShapeGeometry::HeightField(height_field) => {
            ray_cast_height_field(height_field, &local_input)
        }
    };

    output.point = transform_point(transform, output.point);
    output.normal = rotate_vector(transform.q, output.normal);
    output
}

/// Shape cast a shape in world (or relative) space. (b3ShapeCastShape)
pub fn shape_cast_shape(
    shape: &Shape,
    transform: Transform,
    input: &ShapeCastInput,
) -> CastOutput {
    let mut local_points = [Vec3::default(); MAX_SHAPE_CAST_POINTS];
    let count = min_int(input.proxy.count, MAX_SHAPE_CAST_POINTS as i32);
    for i in 0..count {
        local_points[i as usize] =
            inv_transform_point(transform, input.proxy.points[i as usize]);
    }

    let mut local_input = *input;
    local_input.proxy.count = count;
    local_input.proxy.points = local_points;
    local_input.translation = inv_rotate_vector(transform.q, input.translation);

    let mut output = match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => shape_cast_capsule(capsule, &local_input),
        ShapeGeometry::Compound(compound) => shape_cast_compound(compound, &local_input),
        ShapeGeometry::HeightField(height_field) => {
            shape_cast_height_field(height_field, &local_input)
        }
        ShapeGeometry::Hull(hull) => shape_cast_hull(hull, &local_input),
        ShapeGeometry::Mesh { data, scale } => {
            let mesh = Mesh::new(data, *scale);
            shape_cast_mesh(&mesh, &local_input)
        }
        ShapeGeometry::Sphere(sphere) => shape_cast_sphere(sphere, &local_input),
    };

    output.point = transform_point(transform, output.point);
    output.normal = rotate_vector(transform.q, output.normal);
    output
}

/// Test overlap between a shape and a shape proxy. (b3OverlapShape)
pub fn overlap_shape(shape: &Shape, transform: Transform, proxy: &ShapeProxy) -> bool {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => overlap_capsule(capsule, transform, proxy),
        ShapeGeometry::Compound(compound) => overlap_compound(compound, transform, proxy),
        ShapeGeometry::HeightField(height_field) => {
            overlap_height_field(height_field, transform, proxy)
        }
        ShapeGeometry::Hull(hull) => overlap_hull(hull, transform, proxy),
        ShapeGeometry::Mesh { data, scale } => {
            let mesh = Mesh::new(data, *scale);
            overlap_mesh(&mesh, transform, proxy)
        }
        ShapeGeometry::Sphere(sphere) => overlap_sphere(sphere, transform, proxy),
    }
}
