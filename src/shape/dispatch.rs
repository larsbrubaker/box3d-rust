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
    compute_capsule_aabb, compute_capsule_mass, compute_sphere_aabb, compute_sphere_mass,
    MassData, ShapeExtent,
};
use crate::height_field::compute_height_field_aabb;
use crate::hull::{compute_hull_aabb, compute_hull_extent, compute_hull_mass};
use crate::math_functions::{
    aabb_center, abs, add, lerp, max, min_float, sub, Aabb, Transform, Vec3, WorldTransform,
    TRANSFORM_IDENTITY, VEC3_ZERO,
};
use crate::mesh::compute_mesh_aabb;
use crate::types::BodyType;

use super::ShapeGeometry;

/// Compute the shape AABB and fat AABB with speculative and movement margins.
/// (static b3UpdateShapeAABBs)
pub(crate) fn update_shape_aabbs(
    shape: &mut Shape,
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
    shape.fat_aabb = Aabb {
        lower_bound: Vec3 {
            x: aabb.lower_bound.x - margin,
            y: aabb.lower_bound.y - margin,
            z: aabb.lower_bound.z - margin,
        },
        upper_bound: Vec3 {
            x: aabb.upper_bound.x + margin,
            y: aabb.upper_bound.y + margin,
            z: aabb.upper_bound.z + margin,
        },
    };
}

/// (b3CreateShapeProxy)
pub fn create_shape_proxy(
    shape: &mut Shape,
    bp: &mut BroadPhase,
    type_: BodyType,
    transform: WorldTransform,
    force_pair_creation: bool,
) {
    debug_assert!(shape.proxy_key == NULL_INDEX);

    update_shape_aabbs(shape, transform, type_);

    shape.proxy_key = bp.create_proxy(
        type_,
        shape.fat_aabb,
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

/// Time of impact between two shapes along sweeps. Convex vs convex uses GJK
/// TOI; mesh/height/compound targets are handled later (return no-hit until
/// those query paths land). (b3ShapeTimeOfImpact)
pub fn shape_time_of_impact(
    shape_a: &Shape,
    shape_b: &Shape,
    sweep_a: &crate::distance::Sweep,
    sweep_b: &crate::distance::Sweep,
    max_fraction: f32,
) -> crate::distance::ToiOutput {
    use crate::distance::{time_of_impact, ToiInput, ToiOutput, ToiState};
    use crate::geometry::ShapeType;

    let type_a = shape_a.shape_type();
    if type_a == ShapeType::Compound
        || type_a == ShapeType::Height
        || type_a == ShapeType::Mesh
    {
        // Mesh/height/compound CCD against the fast shape is a separate port
        // (b3MeshTimeOfImpactFcn / b3CompoundTimeOfImpactFcn). Convex walls
        // cover the continuous tests in this track.
        return ToiOutput {
            state: ToiState::Separated,
            fraction: 1.0,
            ..ToiOutput::default()
        };
    }

    debug_assert!(
        shape_b.shape_type() != ShapeType::Compound
            && shape_b.shape_type() != ShapeType::Mesh
            && shape_b.shape_type() != ShapeType::Height
    );

    let input = ToiInput {
        proxy_a: make_shape_proxy(shape_a),
        proxy_b: make_shape_proxy(shape_b),
        sweep_a: *sweep_a,
        sweep_b: *sweep_b,
        max_fraction,
    };
    time_of_impact(&input)
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
            extent.max_extent = add(max(c1, c2), r);
        }

        ShapeGeometry::Compound(compound) => {
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
            let r = Vec3 {
                x: radius,
                y: radius,
                z: radius,
            };
            let p = add(sub(sphere.center, local_center), r);
            extent.max_extent = abs(sub(p, local_center));
        }

        ShapeGeometry::Hull(hull) => {
            extent = compute_hull_extent(hull, local_center);
        }

        ShapeGeometry::Mesh { data, scale } => {
            let aabb = compute_mesh_aabb(data, TRANSFORM_IDENTITY, *scale);
            let r1 = crate::math_functions::length(sub(aabb.lower_bound, local_center));
            let r2 = crate::math_functions::length(sub(aabb.upper_bound, local_center));
            extent.min_extent = min_float(r1, r2);
            let p = farthest_point_on_aabb(aabb, local_center);
            extent.max_extent = abs(p);
        }

        ShapeGeometry::HeightField(_) => {}
    }

    extent
}
