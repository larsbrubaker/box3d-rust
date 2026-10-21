//! Compound AABB, overlap, ray cast, and shape cast.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{get_compound_child, get_compound_tree_view, ChildGeometry, CompoundData};
use crate::constants::MAX_SHAPE_CAST_POINTS;
use crate::distance::{compute_proxy_aabb, make_local_proxy, CastOutput, ShapeProxy, Sweep};
use crate::dynamic_tree::BoxCastInput;
use crate::geometry::{
    overlap_capsule, overlap_sphere, ray_cast_capsule, ray_cast_sphere, shape_cast_capsule,
    shape_cast_sphere, RayCastInput, ShapeCastInput,
};
use crate::hull::{overlap_hull, ray_cast_hull, shape_cast_hull};
use crate::math_functions::{
    aabb_transform, inv_rotate_vector, inv_transform_point, make_aabb, mul_transforms,
    rotate_vector, transform_point, Aabb, Transform, VEC3_ZERO,
};
use crate::mesh::{overlap_mesh, ray_cast_mesh, shape_cast_mesh};

/// Compute the AABB of a compound. (b3ComputeCompoundAABB)
pub fn compute_compound_aabb(shape: &CompoundData, transform: Transform) -> Aabb {
    aabb_transform(transform, shape.bounds)
}

/// Test overlap between a compound and a shape proxy. (b3OverlapCompound)
///
/// This is dealing with multiple transforms:
/// - the compound shape transform
/// - the compound child shape transforms
pub fn overlap_compound(
    shape: &CompoundData,
    shape_transform: Transform,
    proxy: &ShapeProxy,
) -> bool {
    debug_assert!(0 < proxy.count && proxy.count as usize <= MAX_SHAPE_CAST_POINTS);

    let mut overlap = false;

    // Use local proxy (proxy in the compound frame).
    let local_proxy = make_local_proxy(proxy, shape_transform);

    let aabb = compute_proxy_aabb(&local_proxy);

    // This query must be in the compound frame.
    let view = get_compound_tree_view(shape);
    view.query(aabb, !0u64, false, |_proxy_id, user_data| {
        let child_index = user_data as i32;
        let child = get_compound_child(shape, child_index);
        let transform = child.transform;
        let proxy = &local_proxy;

        let child_overlap = match child.geometry {
            ChildGeometry::Capsule(ref capsule) => overlap_capsule(capsule, transform, proxy),
            ChildGeometry::Hull(hull) => overlap_hull(hull, transform, proxy),
            ChildGeometry::Mesh(ref mesh) => overlap_mesh(mesh, transform, proxy),
            ChildGeometry::Sphere(ref sphere) => overlap_sphere(sphere, transform, proxy),
        };

        if child_overlap {
            overlap = true;
            false
        } else {
            true
        }
    });

    overlap
}

/// Ray cast versus a compound. (b3RayCastCompound)
pub fn ray_cast_compound(shape: &CompoundData, input: &RayCastInput) -> CastOutput {
    let mut result = CastOutput::default();

    let view = get_compound_tree_view(shape);
    view.ray_cast(input, !0u64, false, |ray_input, _proxy_id, user_data| {
        let child_index = user_data as i32;
        let child = get_compound_child(shape, child_index);

        // Get the input in child local space.
        let local_input = RayCastInput {
            origin: inv_transform_point(child.transform, ray_input.origin),
            translation: inv_rotate_vector(child.transform.q, ray_input.translation),
            max_fraction: ray_input.max_fraction,
        };

        let mut output = match child.geometry {
            ChildGeometry::Capsule(ref capsule) => {
                let mut o = ray_cast_capsule(capsule, &local_input);
                o.material_index = child.material_indices[0] as i32;
                o
            }
            ChildGeometry::Hull(hull) => {
                let mut o = ray_cast_hull(hull, &local_input);
                o.material_index = child.material_indices[0] as i32;
                o
            }
            ChildGeometry::Mesh(ref mesh) => {
                let mut o = ray_cast_mesh(mesh, &local_input);
                debug_assert!(0 <= o.material_index && o.material_index < child.material_count);
                o.material_index = child.material_indices[o.material_index as usize] as i32;
                o
            }
            ChildGeometry::Sphere(ref sphere) => {
                let mut o = ray_cast_sphere(sphere, &local_input);
                o.material_index = child.material_indices[0] as i32;
                o
            }
        };

        if output.hit {
            output.point = transform_point(child.transform, output.point);
            output.normal = rotate_vector(child.transform.q, output.normal);
            output.child_index = child_index;
            result = output;
            return output.fraction;
        }

        ray_input.max_fraction
    });

    result
}

/// Shape cast versus a compound. (b3ShapeCastCompound)
pub fn shape_cast_compound(shape: &CompoundData, input: &ShapeCastInput) -> CastOutput {
    let mut result = CastOutput::default();

    if input.proxy.count == 0 {
        return result;
    }

    let box_ = make_aabb(
        &input.proxy.points[..input.proxy.count as usize],
        input.proxy.radius,
    );
    let tree_input = BoxCastInput {
        box_,
        translation: input.translation,
        max_fraction: input.max_fraction,
    };

    let view = get_compound_tree_view(shape);
    view.box_cast(
        &tree_input,
        !0u64,
        false,
        |box_input, _proxy_id, user_data| {
            let child_index = user_data as i32;
            let child = get_compound_child(shape, child_index);

            // Get the input in the child local space.
            let local_input = ShapeCastInput {
                proxy: make_local_proxy(&input.proxy, child.transform),
                translation: inv_rotate_vector(child.transform.q, input.translation),
                max_fraction: box_input.max_fraction,
                can_encroach: input.can_encroach,
            };

            let mut output = match child.geometry {
                ChildGeometry::Capsule(ref capsule) => {
                    let mut o = shape_cast_capsule(capsule, &local_input);
                    o.material_index = child.material_indices[0] as i32;
                    o
                }
                ChildGeometry::Hull(hull) => {
                    let mut o = shape_cast_hull(hull, &local_input);
                    o.material_index = child.material_indices[0] as i32;
                    o
                }
                ChildGeometry::Mesh(ref mesh) => {
                    let mut o = shape_cast_mesh(mesh, &local_input);
                    debug_assert!(0 <= o.material_index && o.material_index < child.material_count);
                    o.material_index = child.material_indices[o.material_index as usize] as i32;
                    o
                }
                ChildGeometry::Sphere(ref sphere) => {
                    let mut o = shape_cast_sphere(sphere, &local_input);
                    o.material_index = child.material_indices[0] as i32;
                    o
                }
            };

            if output.hit {
                output.point = transform_point(child.transform, output.point);
                output.normal = rotate_vector(child.transform.q, output.normal);
                output.child_index = child_index;
                result = output;
                return output.fraction;
            }

            box_input.max_fraction
        },
    );

    result
}

/// Static compound-child sweep for TOI (child fixed in compound frame).
/// `xf = compoundTransform * childTransform`. (b3MakeCompoundChildSweep)
pub fn make_compound_child_sweep(
    compound_transform: Transform,
    child_transform: Transform,
) -> Sweep {
    let xf = mul_transforms(compound_transform, child_transform);
    Sweep {
        local_center: VEC3_ZERO,
        c1: xf.p,
        c2: xf.p,
        q1: xf.q,
        q2: xf.q,
    }
}
