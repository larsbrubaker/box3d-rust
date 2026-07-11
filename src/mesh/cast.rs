//! Mesh AABB, ray cast, and shape cast.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{Mesh, MeshData, MESH_STACK_SIZE};
use crate::core::NULL_INDEX;
use crate::distance::{make_proxy, shape_cast, CastOutput, ShapeCastPairInput};
use crate::geometry::{RayCastInput, ShapeCastInput};
use crate::math_functions::{
    aabb_center, aabb_extents, aabb_transform, abs, add, cross, get_by_index,
    intersect_ray_triangle, make_aabb, max, min, mul, mul_sv, neg, normalize, sub,
    test_bounds_overlap, test_bounds_ray_overlap, Aabb, Transform, Vec3, QUAT_IDENTITY, VEC3_ZERO,
};

/// Compute the AABB of a mesh. (b3ComputeMeshAABB)
pub fn compute_mesh_aabb(shape: &MeshData, transform: Transform, scale: Vec3) -> Aabb {
    let scaled_lower = mul(scale, shape.bounds.lower_bound);
    let scaled_upper = mul(scale, shape.bounds.upper_bound);
    let bounds = Aabb {
        lower_bound: min(scaled_lower, scaled_upper),
        upper_bound: max(scaled_lower, scaled_upper),
    };
    aabb_transform(transform, bounds)
}

/// Ray cast versus a mesh in local space. (b3RayCastMesh)
pub fn ray_cast_mesh(mesh: &Mesh<'_>, input: &RayCastInput) -> CastOutput {
    let data = mesh.data;
    let mesh_scale = mesh.scale;

    let mut best_output = CastOutput {
        fraction: input.max_fraction,
        triangle_index: NULL_INDEX,
        ..Default::default()
    };

    let mut lambda = input.max_fraction;
    let ray_start = input.origin;
    let ray_delta = input.translation;

    let inv_scale = Vec3 {
        x: 1.0 / mesh_scale.x,
        y: 1.0 / mesh_scale.y,
        z: 1.0 / mesh_scale.z,
    };
    let clockwise = mesh_scale.x * mesh_scale.y * mesh_scale.z < 0.0;

    let inv_scaled_ray_start = mul(inv_scale, ray_start);
    let inv_scaled_ray_delta = mul(inv_scale, ray_delta);
    let mut inv_scaled_ray_end = add(inv_scaled_ray_start, mul_sv(lambda, inv_scaled_ray_delta));
    let mut inv_scaled_ray_min = min(inv_scaled_ray_start, inv_scaled_ray_end);
    let mut inv_scaled_ray_max = max(inv_scaled_ray_start, inv_scaled_ray_end);

    let mut stack = [0i32; MESH_STACK_SIZE];
    let mut count = 0usize;
    let mut node_index = 0usize;

    let triangles = &data.triangles;
    let vertices = &data.vertices;
    let material_indices = &data.material_indices;

    loop {
        let node = &data.nodes[node_index];
        let node_min = node.lower_bound;
        let node_max = node.upper_bound;

        if test_bounds_overlap(node_min, node_max, inv_scaled_ray_min, inv_scaled_ray_max)
            && test_bounds_ray_overlap(
                node_min,
                node_max,
                inv_scaled_ray_start,
                inv_scaled_ray_delta,
            )
        {
            if node.is_leaf() {
                let triangle_count = node.triangle_count() as i32;
                let triangle_offset = node.triangle_offset as i32;

                for index in 0..triangle_count {
                    let triangle_index = triangle_offset + index;
                    let triangle = triangles[triangle_index as usize];

                    let vertex1 = mul(mesh_scale, vertices[triangle.index1 as usize]);
                    let (vertex2, vertex3) = if clockwise {
                        (
                            mul(mesh_scale, vertices[triangle.index3 as usize]),
                            mul(mesh_scale, vertices[triangle.index2 as usize]),
                        )
                    } else {
                        (
                            mul(mesh_scale, vertices[triangle.index2 as usize]),
                            mul(mesh_scale, vertices[triangle.index3 as usize]),
                        )
                    };

                    let alpha =
                        intersect_ray_triangle(ray_start, ray_delta, vertex1, vertex2, vertex3);
                    debug_assert!((0.0..=1.0).contains(&alpha));

                    if alpha < best_output.fraction {
                        let edge1 = sub(vertex2, vertex1);
                        let edge2 = sub(vertex3, vertex1);
                        best_output.normal = normalize(cross(edge1, edge2));
                        best_output.point = add(input.origin, mul_sv(alpha, input.translation));
                        best_output.fraction = alpha;
                        best_output.triangle_index = triangle_index;
                        best_output.material_index =
                            material_indices[triangle_index as usize] as i32;
                        best_output.hit = true;

                        lambda = alpha;
                        inv_scaled_ray_end =
                            add(inv_scaled_ray_start, mul_sv(lambda, inv_scaled_ray_delta));
                        inv_scaled_ray_min = min(inv_scaled_ray_start, inv_scaled_ray_end);
                        inv_scaled_ray_max = max(inv_scaled_ray_start, inv_scaled_ray_end);
                    }
                }
            } else {
                let axis = node.axis() as i32;
                debug_assert!(count <= MESH_STACK_SIZE - 1);
                if get_by_index(inv_scaled_ray_delta, axis) > 0.0 {
                    stack[count] = node_index as i32 + node.child_offset() as i32;
                    count += 1;
                    node_index += 1;
                } else {
                    stack[count] = node_index as i32 + 1;
                    count += 1;
                    node_index += node.child_offset() as usize;
                }
                continue;
            }
        }

        if count == 0 {
            break;
        }
        count -= 1;
        node_index = stack[count] as usize;
    }

    best_output
}

/// Shape cast versus a mesh in local space. (b3ShapeCastMesh)
pub fn shape_cast_mesh(mesh: &Mesh<'_>, input: &ShapeCastInput) -> CastOutput {
    let data = mesh.data;
    let mesh_scale = mesh.scale;

    let mut best_output = CastOutput {
        fraction: input.max_fraction,
        triangle_index: NULL_INDEX,
        ..Default::default()
    };

    let mut lambda = input.max_fraction;

    let shape_bounds = make_aabb(
        &input.proxy.points[..input.proxy.count as usize],
        input.proxy.radius,
    );
    let center = aabb_center(shape_bounds);
    let extents = aabb_extents(shape_bounds);
    let shape_extent = extents;

    let ray_start = center;
    let ray_delta = input.translation;
    let mut ray_end = add(ray_start, mul_sv(lambda, ray_delta));
    let mut ray_min = min(ray_start, ray_end);
    let mut ray_max = max(ray_start, ray_end);

    let inv_scale = Vec3 {
        x: 1.0 / mesh_scale.x,
        y: 1.0 / mesh_scale.y,
        z: 1.0 / mesh_scale.z,
    };
    let abs_inv_scale = abs(inv_scale);
    let clockwise = mesh_scale.x * mesh_scale.y * mesh_scale.z < 0.0;

    let inv_scaled_ray_start = mul(inv_scale, ray_start);
    let inv_scaled_ray_delta = mul(inv_scale, ray_delta);
    let mut inv_scaled_ray_end = add(inv_scaled_ray_start, mul_sv(lambda, inv_scaled_ray_delta));
    let mut inv_scaled_ray_min = min(inv_scaled_ray_start, inv_scaled_ray_end);
    let mut inv_scaled_ray_max = max(inv_scaled_ray_start, inv_scaled_ray_end);
    let inv_scaled_shape_extent = mul(abs_inv_scale, shape_extent);

    let mut stack = [0i32; MESH_STACK_SIZE];
    let mut count = 0usize;
    let mut node_index = 0usize;

    let triangles = &data.triangles;
    let vertices = &data.vertices;
    let material_indices = &data.material_indices;

    loop {
        let node = &data.nodes[node_index];
        let node_min = sub(node.lower_bound, inv_scaled_shape_extent);
        let node_max = add(node.upper_bound, inv_scaled_shape_extent);

        if test_bounds_overlap(node_min, node_max, inv_scaled_ray_min, inv_scaled_ray_max)
            && test_bounds_ray_overlap(
                node_min,
                node_max,
                inv_scaled_ray_start,
                inv_scaled_ray_delta,
            )
        {
            if node.is_leaf() {
                let triangle_count = node.triangle_count() as i32;
                let triangle_offset = node.triangle_offset as i32;

                for index in 0..triangle_count {
                    let triangle_index = triangle_offset + index;
                    let triangle = triangles[triangle_index as usize];

                    let vertex1 = mul(mesh_scale, vertices[triangle.index1 as usize]);
                    let (vertex2, vertex3) = if clockwise {
                        (
                            mul(mesh_scale, vertices[triangle.index3 as usize]),
                            mul(mesh_scale, vertices[triangle.index2 as usize]),
                        )
                    } else {
                        (
                            mul(mesh_scale, vertices[triangle.index2 as usize]),
                            mul(mesh_scale, vertices[triangle.index3 as usize]),
                        )
                    };

                    let triangle_min = sub(min(vertex1, min(vertex2, vertex3)), shape_extent);
                    let triangle_max = add(max(vertex1, max(vertex2, vertex3)), shape_extent);

                    if test_bounds_overlap(triangle_min, triangle_max, ray_min, ray_max) {
                        let origin = vertex1;
                        let triangle_vertices =
                            [VEC3_ZERO, sub(vertex2, origin), sub(vertex3, origin)];
                        let shifted_origin = Transform {
                            p: neg(origin),
                            q: QUAT_IDENTITY,
                        };

                        let pair_input = ShapeCastPairInput {
                            proxy_a: make_proxy(&triangle_vertices, 0.0),
                            proxy_b: input.proxy,
                            transform: shifted_origin,
                            max_fraction: best_output.fraction,
                            translation_b: input.translation,
                            can_encroach: input.can_encroach,
                        };

                        let mut pair_output = shape_cast(&pair_input);
                        if pair_output.hit {
                            pair_output.point = add(pair_output.point, origin);
                            best_output = pair_output;
                            best_output.triangle_index = triangle_index;
                            best_output.material_index =
                                material_indices[triangle_index as usize] as i32;

                            lambda = best_output.fraction;
                            ray_end = add(ray_start, mul_sv(lambda, ray_delta));
                            ray_min = min(ray_start, ray_end);
                            ray_max = max(ray_start, ray_end);

                            inv_scaled_ray_end =
                                add(inv_scaled_ray_start, mul_sv(lambda, inv_scaled_ray_delta));
                            inv_scaled_ray_min = min(inv_scaled_ray_start, inv_scaled_ray_end);
                            inv_scaled_ray_max = max(inv_scaled_ray_start, inv_scaled_ray_end);
                        }
                    }
                }
            } else {
                let axis = node.axis() as i32;
                debug_assert!(count <= MESH_STACK_SIZE - 1);
                if get_by_index(inv_scaled_ray_delta, axis) > 0.0 {
                    stack[count] = node_index as i32 + node.child_offset() as i32;
                    count += 1;
                    node_index += 1;
                } else {
                    stack[count] = node_index as i32 + 1;
                    count += 1;
                    node_index += node.child_offset() as usize;
                }
                continue;
            }
        }

        if count == 0 {
            break;
        }
        count -= 1;
        node_index = stack[count] as usize;
    }

    best_output
}
