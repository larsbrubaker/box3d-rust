// Per-shape-type dispatch helpers from shape.c (AABB, mass, centroid, proxy).
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::Shape;
use crate::aabb::farthest_point_on_aabb;
use crate::broad_phase::{proxy_type, BroadPhase};
use crate::compound::compute_compound_aabb;
use crate::constants::speculative_distance;
use crate::core::NULL_INDEX;
use crate::geometry::{
    compute_capsule_aabb, compute_capsule_mass, compute_sphere_aabb, compute_sphere_mass, MassData,
    ShapeExtent,
};
use crate::height_field::compute_height_field_aabb;
use crate::hull::{compute_hull_aabb, compute_hull_extent, compute_hull_mass};
use crate::math_functions::{
    aabb_center, aabb_inflate, abs, add, lerp, max, min_float, sub, Aabb, Transform, Vec3,
    WorldTransform, TRANSFORM_IDENTITY, VEC3_ZERO,
};
use crate::mesh::compute_mesh_aabb;
use crate::types::BodyType;
use crate::world::World;

use super::ShapeGeometry;

/// Compute the shape AABB and fat AABB with speculative and movement margins.
/// The fat AABB lives in `World::fat_aabbs`, parallel to the shape array.
/// (static b3UpdateShapeAABBs)
pub(crate) fn update_shape_aabbs(
    shape: &mut Shape,
    fat_aabb: &mut Aabb,
    transform: WorldTransform,
    proxy_type_: BodyType,
) {
    let speculative = speculative_distance();
    let aabb_margin = shape.aabb_margin;

    let aabb = compute_fat_shape_aabb(shape, transform, speculative);
    shape.aabb = aabb;

    // Smaller margin for static bodies. Cannot be zero due to TOI tolerance.
    let margin = if proxy_type_ == BodyType::Static {
        speculative
    } else {
        aabb_margin
    };
    *fat_aabb = aabb_inflate(aabb, margin);
}

/// (b3CreateShapeProxy)
pub fn create_shape_proxy(
    world: &mut World,
    shape_id: i32,
    type_: BodyType,
    transform: WorldTransform,
    force_pair_creation: bool,
) {
    let World {
        shapes,
        fat_aabbs,
        broad_phase,
        ..
    } = world;
    let shape = &mut shapes[shape_id as usize];
    debug_assert!(shape.proxy_key == NULL_INDEX);

    let fat_aabb = &mut fat_aabbs[shape.id as usize];
    update_shape_aabbs(shape, fat_aabb, transform, type_);

    // Create proxies in the broad-phase.
    shape.proxy_key = broad_phase.create_proxy(
        type_,
        *fat_aabb,
        shape.filter.category_bits,
        shape.id,
        force_pair_creation,
    );
    debug_assert!((proxy_type(shape.proxy_key) as usize) < crate::types::BODY_TYPE_COUNT);
}

/// (b3DestroyShapeProxy)
pub fn destroy_shape_proxy(shape: &mut Shape, bp: &mut BroadPhase) {
    if shape.proxy_key != NULL_INDEX {
        bp.destroy_proxy(shape.proxy_key);
        shape.proxy_key = NULL_INDEX;
    }
}

/// (b3ComputeShapeAABB)
pub fn compute_shape_aabb(shape: &Shape, transform: Transform) -> Aabb {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => compute_capsule_aabb(capsule, transform),
        ShapeGeometry::Compound(compound) => compute_compound_aabb(compound, transform),
        ShapeGeometry::HeightField(hf) => compute_height_field_aabb(hf, transform),
        ShapeGeometry::Hull(hull) => compute_hull_aabb(hull, transform),
        ShapeGeometry::Mesh { data, scale } => compute_mesh_aabb(data, transform, *scale),
        ShapeGeometry::Sphere(sphere) => compute_sphere_aabb(sphere, transform),
    }
}

/// (b3ComputeFatShapeAABB)
pub fn compute_fat_shape_aabb(shape: &Shape, transform: WorldTransform, extra: f32) -> Aabb {
    let r = Vec3 {
        x: extra,
        y: extra,
        z: extra,
    };
    #[cfg(feature = "double-precision")]
    {
        use crate::math_functions::{offset_aabb, VEC3_ZERO};
        // Build the box in the body local frame, inflate, then translate by the double
        // origin and round outward.
        let rotation = Transform {
            p: VEC3_ZERO,
            q: transform.q,
        };
        let mut local_box = compute_shape_aabb(shape, rotation);
        local_box.lower_bound = sub(local_box.lower_bound, r);
        local_box.upper_bound = add(local_box.upper_bound, r);
        offset_aabb(local_box, transform.p)
    }
    #[cfg(not(feature = "double-precision"))]
    {
        let mut aabb = compute_shape_aabb(shape, transform);
        aabb.lower_bound = sub(aabb.lower_bound, r);
        aabb.upper_bound = add(aabb.upper_bound, r);
        aabb
    }
}

/// (b3GetShapeCentroid)
pub fn get_shape_centroid(shape: &Shape) -> Vec3 {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => lerp(capsule.center1, capsule.center2, 0.5),
        ShapeGeometry::Compound(compound) => {
            let aabb = compute_compound_aabb(compound, TRANSFORM_IDENTITY);
            aabb_center(aabb)
        }
        ShapeGeometry::Sphere(sphere) => sphere.center,
        ShapeGeometry::Hull(hull) => hull.center,
        ShapeGeometry::Mesh { data, scale } => {
            let aabb = compute_mesh_aabb(data, TRANSFORM_IDENTITY, *scale);
            aabb_center(aabb)
        }
        ShapeGeometry::HeightField(hf) => {
            let aabb = compute_height_field_aabb(hf, TRANSFORM_IDENTITY);
            aabb_center(aabb)
        }
    }
}

/// (b3ComputeShapeMass)
pub fn compute_shape_mass(shape: &Shape) -> MassData {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => compute_capsule_mass(capsule, shape.density),
        ShapeGeometry::Hull(hull) => compute_hull_mass(hull, shape.density),
        ShapeGeometry::Sphere(sphere) => compute_sphere_mass(sphere, shape.density),
        _ => MassData::default(),
    }
}

/// Convex shape proxy for distance / TOI. Mesh, height, and compound need
/// their own query paths. (b3MakeShapeProxy)
pub fn make_shape_proxy(shape: &Shape) -> crate::distance::ShapeProxy {
    use crate::distance::make_proxy;
    use crate::hull::get_hull_points;

    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            make_proxy(&[capsule.center1, capsule.center2], capsule.radius)
        }
        ShapeGeometry::Sphere(sphere) => make_proxy(&[sphere.center], sphere.radius),
        ShapeGeometry::Hull(hull) => make_proxy(get_hull_points(hull), 0.0),
        _ => {
            debug_assert!(false, "make_shape_proxy is for convex shapes only");
            make_proxy(&[VEC3_ZERO], 0.0)
        }
    }
}

/// (b3ComputeShapeExtent)
pub fn compute_shape_extent(shape: &Shape, local_center: Vec3) -> ShapeExtent {
    let mut extent = ShapeExtent::default();

    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            let radius = capsule.radius;
            extent.min_extent = radius;
            let c1 = sub(capsule.center1, local_center);
            let c2 = sub(capsule.center2, local_center);
            let r = Vec3 {
                x: radius,
                y: radius,
                z: radius,
            };
            extent.max_extent = add(max(abs(c1), abs(c2)), r);
        }

        ShapeGeometry::Compound(compound) => {
            // This is shouldn't be needed but here for completeness
            let aabb = compute_compound_aabb(compound, TRANSFORM_IDENTITY);
            let r1 = crate::math_functions::length(sub(aabb.lower_bound, local_center));
            let r2 = crate::math_functions::length(sub(aabb.upper_bound, local_center));
            extent.min_extent = min_float(r1, r2);
            let p = farthest_point_on_aabb(aabb, local_center);
            extent.max_extent = abs(sub(p, local_center));
        }

        ShapeGeometry::Sphere(sphere) => {
            let radius = sphere.radius;
            extent.min_extent = radius;
            let h = abs(sub(sphere.center, local_center));
            let r = Vec3 {
                x: radius,
                y: radius,
                z: radius,
            };
            extent.max_extent = add(h, r);
        }

        ShapeGeometry::Hull(hull) => {
            extent = compute_hull_extent(hull, local_center);
        }

        ShapeGeometry::Mesh { data, scale } => {
            // This is needed for kinematic mesh sleeping
            let aabb = compute_mesh_aabb(data, TRANSFORM_IDENTITY, *scale);
            let r1 = crate::math_functions::length(sub(aabb.lower_bound, local_center));
            let r2 = crate::math_functions::length(sub(aabb.upper_bound, local_center));
            extent.min_extent = min_float(r1, r2);
            let p = farthest_point_on_aabb(aabb, local_center);
            extent.max_extent = abs(sub(p, local_center));
        }

        ShapeGeometry::HeightField(_) => {}
    }

    extent
}

/// The smallest extent of a shape, used for the continuous fallback radius and the fast shape
/// test. Cheaper than [`compute_shape_extent`] for capsules, spheres, and hulls.
/// (b3ComputeShapeMinExtent)
pub fn compute_shape_min_extent(shape: &Shape, local_center: Vec3) -> f32 {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => capsule.radius,

        ShapeGeometry::Sphere(sphere) => sphere.radius,

        ShapeGeometry::Hull(hull) => hull.inner_radius,

        _ => compute_shape_extent(shape, local_center).min_extent,
    }
}
