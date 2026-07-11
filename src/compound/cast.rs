//! Compound AABB, overlap, ray cast, and shape cast.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{get_compound_child, ChildGeometry, CompoundData, MAX_COMPOUND_MESH_MATERIALS};
use crate::constants::MAX_SHAPE_CAST_POINTS;
use crate::distance::{CastOutput, ShapeProxy};
use crate::dynamic_tree::BoxCastInput;
use crate::geometry::{
    overlap_capsule, overlap_sphere, ray_cast_capsule, ray_cast_sphere, shape_cast_capsule,
    shape_cast_sphere, RayCastInput, ShapeCastInput,
};
use crate::hull::{overlap_hull, ray_cast_hull, shape_cast_hull};
use crate::math_functions::{
    aabb_transform, add, invert_transform, inv_rotate_vector, inv_transform_point,
    make_aabb, make_matrix_from_quat, max, min, min_int, mul_mv, mul_transforms, rotate_vector,
    sub, transform_point, Aabb, Transform, Vec3,
};
use crate::mesh::{overlap_mesh, ray_cast_mesh, shape_cast_mesh};

/// Compute the AABB of a compound. (b3ComputeCompoundAABB)
pub fn compute_compound_aabb(shape: &CompoundData, transform: Transform) -> Aabb {
    debug_assert!(shape.node_offset > 0);
    let aabb = shape.tree.root_bounds();
    aabb_transform(transform, aabb)
}

/// Test overlap between a compound and a shape proxy. (b3OverlapCompound)
pub fn overlap_compound(
    shape: &CompoundData,
    shape_transform: Transform,
    proxy: &ShapeProxy,
) -> bool {
    let mut overlap = false;

    let mut aabb = Aabb {
        lower_bound: proxy.points[0],
        upper_bound: proxy.points[0],
    };
    for i in 1..proxy.count as usize {
        aabb.lower_bound = min(aabb.lower_bound, proxy.points[i]);
        aabb.upper_bound = max(aabb.upper_bound, proxy.points[i]);
    }

    let r = Vec3 {
        x: proxy.radius,
        y: proxy.radius,
        z: proxy.radius,
    };
    aabb.lower_bound = sub(aabb.lower_bound, r);
    aabb.upper_bound = add(aabb.upper_bound, r);

    shape.tree.query(aabb, !0u64, false, |_proxy_id, user_data| {
        let child_index = user_data as i32;
        let child = get_compound_child(shape, child_index);
        let transform = mul_transforms(shape_transform, child.transform);

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

    shape.tree.ray_cast(input, !0u64, false, |ray_input, _proxy_id, user_data| {
        let child_index = user_data as i32;
        let child = get_compound_child(shape, child_index);

        let local_input = RayCastInput {
            origin: inv_transform_point(child.transform, ray_input.origin),
            translation: inv_rotate_vector(child.transform.q, ray_input.translation),
            max_fraction: ray_input.max_fraction,
        };

        let mut output = match child.geometry {
            ChildGeometry::Capsule(ref capsule) => {
                let mut o = ray_cast_capsule(capsule, &local_input);
                o.material_index = child.material_indices[0];
                o
            }
            ChildGeometry::Hull(hull) => {
                let mut o = ray_cast_hull(hull, &local_input);
                o.material_index = child.material_indices[0];
                o
            }
            ChildGeometry::Mesh(ref mesh) => {
                let mut o = ray_cast_mesh(mesh, &local_input);
                debug_assert!(0 <= o.material_index);
                let child_material_index =
                    min_int(o.material_index, MAX_COMPOUND_MESH_MATERIALS as i32 - 1);
                o.material_index = child.material_indices[child_material_index as usize];
                o
            }
            ChildGeometry::Sphere(ref sphere) => {
                let mut o = ray_cast_sphere(sphere, &local_input);
                o.material_index = child.material_indices[0];
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

    let box_ = make_aabb(&input.proxy.points[..input.proxy.count as usize], input.proxy.radius);
    let tree_input = BoxCastInput {
        box_,
        translation: input.translation,
        max_fraction: input.max_fraction,
    };

    shape.tree.box_cast(&tree_input, !0u64, false, |box_input, _proxy_id, user_data| {
        let child_index = user_data as i32;
        let child = get_compound_child(shape, child_index);

        let mut local_input = *input;
        local_input.max_fraction = box_input.max_fraction;

        let mut local_points = [Vec3::default(); MAX_SHAPE_CAST_POINTS];
        local_input.proxy.count = min_int(input.proxy.count, MAX_SHAPE_CAST_POINTS as i32);

        let inv_transform = invert_transform(child.transform);
        let r = make_matrix_from_quat(inv_transform.q);

        for i in 0..local_input.proxy.count as usize {
            local_points[i] = add(mul_mv(r, input.proxy.points[i]), inv_transform.p);
        }
        local_input.proxy.points = local_points;
        local_input.translation = mul_mv(r, input.translation);

        let mut output = match child.geometry {
            ChildGeometry::Capsule(ref capsule) => {
                let mut o = shape_cast_capsule(capsule, &local_input);
                o.material_index = child.material_indices[0];
                o
            }
            ChildGeometry::Hull(hull) => {
                let mut o = shape_cast_hull(hull, &local_input);
                o.material_index = child.material_indices[0];
                o
            }
            ChildGeometry::Mesh(ref mesh) => {
                let mut o = shape_cast_mesh(mesh, &local_input);
                debug_assert!(0 <= o.material_index);
                let child_material_index =
                    min_int(o.material_index, MAX_COMPOUND_MESH_MATERIALS as i32 - 1);
                o.material_index = child.material_indices[child_material_index as usize];
                o
            }
            ChildGeometry::Sphere(ref sphere) => {
                let mut o = shape_cast_sphere(sphere, &local_input);
                o.material_index = child.material_indices[0];
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
    });

    result
}
