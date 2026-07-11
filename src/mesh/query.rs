//! Mesh overlap, query, mover collide, and triangle accessor.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{
    Mesh, CONCAVE_EDGE1, CONCAVE_EDGE2, CONCAVE_EDGE3, INVERSE_CONCAVE_EDGE1, INVERSE_CONCAVE_EDGE2,
    INVERSE_CONCAVE_EDGE3, MESH_STACK_SIZE,
};
use crate::constants::linear_slop;
use crate::distance::{
    compute_proxy_aabb, make_local_proxy, make_proxy, shape_distance, DistanceInput, ShapeProxy,
    SimplexCache,
};
use crate::geometry::{Capsule, PlaneResult};
use crate::math_functions::{
    add, max, min, mul, mul_sv, sub, test_bounds_overlap, test_bounds_triangle_overlap, Aabb, Plane,
    Transform, Triangle, Vec3, TRANSFORM_IDENTITY,
};

/// Test overlap between a mesh and a shape proxy. (b3OverlapMesh)
pub fn overlap_mesh(shape: &Mesh<'_>, shape_transform: Transform, proxy: &ShapeProxy) -> bool {
    debug_assert!(proxy.count > 0);
    let mut cache = SimplexCache::default();

    let local_proxy = make_local_proxy(proxy, shape_transform);
    let aabb = compute_proxy_aabb(&local_proxy);

    let mesh_scale = shape.scale;
    let inv_scale = Vec3 {
        x: 1.0 / mesh_scale.x,
        y: 1.0 / mesh_scale.y,
        z: 1.0 / mesh_scale.z,
    };
    let temp1 = mul(inv_scale, aabb.lower_bound);
    let temp2 = mul(inv_scale, aabb.upper_bound);
    let inv_scaled_bounds_min = min(temp1, temp2);
    let inv_scaled_bounds_max = max(temp1, temp2);
    let inv_scaled_bounds_center =
        mul_sv(0.5, add(inv_scaled_bounds_min, inv_scaled_bounds_max));
    let inv_scaled_bounds_extent = sub(inv_scaled_bounds_max, inv_scaled_bounds_center);

    let mut input = DistanceInput {
        proxy_a: Default::default(),
        proxy_b: local_proxy,
        transform: TRANSFORM_IDENTITY,
        use_radii: true,
    };

    let mut stack = [0i32; MESH_STACK_SIZE];
    let mut count = 0usize;
    let mut node_index = 0usize;

    let data = shape.data;
    let triangles = &data.triangles;
    let vertices = &data.vertices;

    loop {
        let node = &data.nodes[node_index];
        if test_bounds_overlap(
            node.lower_bound,
            node.upper_bound,
            inv_scaled_bounds_min,
            inv_scaled_bounds_max,
        ) {
            if node.is_leaf() {
                let triangle_count = node.triangle_count() as i32;
                let triangle_offset = node.triangle_offset as i32;

                for index in 0..triangle_count {
                    let triangle_index = triangle_offset + index;
                    let triangle = triangles[triangle_index as usize];

                    let vertex1 = vertices[triangle.index1 as usize];
                    let vertex2 = vertices[triangle.index2 as usize];
                    let vertex3 = vertices[triangle.index3 as usize];

                    if test_bounds_triangle_overlap(
                        inv_scaled_bounds_center,
                        inv_scaled_bounds_extent,
                        vertex1,
                        vertex2,
                        vertex3,
                    ) {
                        let triangle_vertices = [
                            mul(mesh_scale, vertex1),
                            mul(mesh_scale, vertex2),
                            mul(mesh_scale, vertex3),
                        ];
                        input.proxy_a = make_proxy(&triangle_vertices, 0.0);
                        cache.count = 0;

                        let output = shape_distance(&input, &mut cache, None);
                        let tolerance = 0.1 * linear_slop();
                        if output.distance < tolerance {
                            return true;
                        }
                    }
                }
            } else {
                debug_assert!(count <= MESH_STACK_SIZE - 1);
                stack[count] = node_index as i32 + node.child_offset() as i32;
                count += 1;
                node_index += 1;
                continue;
            }
        }

        if count == 0 {
            break;
        }
        count -= 1;
        node_index = stack[count] as usize;
    }

    false
}

/// Get a scaled mesh triangle with edge flags. (b3GetMeshTriangle)
pub fn get_mesh_triangle(mesh: &Mesh<'_>, triangle_index: i32) -> Triangle {
    debug_assert!(0 <= triangle_index && triangle_index < mesh.data.triangle_count);

    let triangles = &mesh.data.triangles;
    let flags = &mesh.data.flags;
    let vertices = &mesh.data.vertices;

    let triangle = triangles[triangle_index as usize];
    let triangle_flags = flags[triangle_index as usize];
    let scale = mesh.scale;

    let mut result = Triangle {
        vertices: [Default::default(); 3],
        i1: triangle.index1,
        i2: 0,
        i3: 0,
        flags: 0,
    };
    result.vertices[0] = mul(scale, vertices[triangle.index1 as usize]);

    if scale.x * scale.y * scale.z < 0.0 {
        result.vertices[1] = mul(scale, vertices[triangle.index3 as usize]);
        result.vertices[2] = mul(scale, vertices[triangle.index2 as usize]);
        result.i2 = triangle.index3;
        result.i3 = triangle.index2;

        result.flags = 0;
        if (triangle_flags as i32) & INVERSE_CONCAVE_EDGE1 != 0 {
            result.flags |= CONCAVE_EDGE1;
        }
        if (triangle_flags as i32) & INVERSE_CONCAVE_EDGE2 != 0 {
            result.flags |= CONCAVE_EDGE2;
        }
        if (triangle_flags as i32) & INVERSE_CONCAVE_EDGE3 != 0 {
            result.flags |= CONCAVE_EDGE3;
        }
    } else {
        result.vertices[1] = mul(scale, vertices[triangle.index2 as usize]);
        result.vertices[2] = mul(scale, vertices[triangle.index3 as usize]);
        result.i2 = triangle.index2;
        result.i3 = triangle.index3;
        result.flags = triangle_flags as i32;
    }

    result
}

/// Collide a capsule mover against a mesh, writing contact planes.
/// (b3CollideMoverAndMesh)
pub fn collide_mover_and_mesh(
    planes: &mut [PlaneResult],
    shape: &Mesh<'_>,
    mover: &Capsule,
) -> i32 {
    let capacity = planes.len() as i32;
    if capacity == 0 {
        return 0;
    }

    let mut distance_input = DistanceInput {
        proxy_a: Default::default(),
        proxy_b: make_proxy(&[mover.center1, mover.center2], 0.0),
        transform: TRANSFORM_IDENTITY,
        use_radii: false,
    };

    let mut cache = SimplexCache::default();
    let radius = mover.radius;

    let r = Vec3 {
        x: radius,
        y: radius,
        z: radius,
    };
    let bounds_min = sub(min(mover.center1, mover.center2), r);
    let bounds_max = add(max(mover.center1, mover.center2), r);

    let mesh_scale = shape.scale;
    let inv_scale = Vec3 {
        x: 1.0 / mesh_scale.x,
        y: 1.0 / mesh_scale.y,
        z: 1.0 / mesh_scale.z,
    };
    let temp1 = mul(inv_scale, bounds_min);
    let temp2 = mul(inv_scale, bounds_max);
    let inv_scaled_bounds_min = min(temp1, temp2);
    let inv_scaled_bounds_max = max(temp1, temp2);
    let inv_scaled_bounds_center =
        mul_sv(0.5, add(inv_scaled_bounds_min, inv_scaled_bounds_max));
    let inv_scaled_bounds_extent = sub(inv_scaled_bounds_max, inv_scaled_bounds_center);

    let mut stack = [0i32; MESH_STACK_SIZE];
    let mut count = 0usize;
    let mut node_index = 0usize;

    let data = shape.data;
    let triangles = &data.triangles;
    let vertices = &data.vertices;

    let mut plane_count = 0i32;
    while plane_count < capacity {
        let node = &data.nodes[node_index];
        if test_bounds_overlap(
            node.lower_bound,
            node.upper_bound,
            inv_scaled_bounds_min,
            inv_scaled_bounds_max,
        ) {
            if node.is_leaf() {
                let triangle_count = node.triangle_count() as i32;
                let triangle_offset = node.triangle_offset as i32;

                for index in 0..triangle_count {
                    let triangle_index = triangle_offset + index;
                    let triangle = triangles[triangle_index as usize];

                    let vertex1 = vertices[triangle.index1 as usize];
                    let vertex2 = vertices[triangle.index2 as usize];
                    let vertex3 = vertices[triangle.index3 as usize];

                    if test_bounds_triangle_overlap(
                        inv_scaled_bounds_center,
                        inv_scaled_bounds_extent,
                        vertex1,
                        vertex2,
                        vertex3,
                    ) {
                        let triangle_vertices = [
                            mul(mesh_scale, vertex1),
                            mul(mesh_scale, vertex2),
                            mul(mesh_scale, vertex3),
                        ];
                        distance_input.proxy_a = make_proxy(&triangle_vertices, 0.0);
                        cache.count = 0;

                        let distance_output = shape_distance(&distance_input, &mut cache, None);

                        if distance_output.distance == 0.0 {
                            // todo SAT
                        } else if distance_output.distance <= mover.radius {
                            let plane = Plane {
                                normal: distance_output.normal,
                                offset: mover.radius - distance_output.distance,
                            };
                            planes[plane_count as usize] = PlaneResult {
                                plane,
                                point: distance_output.point_a,
                            };
                            plane_count += 1;
                            if plane_count == capacity {
                                return plane_count;
                            }
                        }
                    }
                }
            } else {
                debug_assert!(count <= MESH_STACK_SIZE - 1);
                stack[count] = node_index as i32 + node.child_offset() as i32;
                count += 1;
                node_index += 1;
                continue;
            }
        }

        if count == 0 {
            break;
        }
        count -= 1;
        node_index = stack[count] as usize;
    }

    plane_count
}

/// Query mesh triangles overlapping an AABB.
/// Callback receives (a, b, c, triangle_index) and may return `false` to stop early.
/// (b3QueryMesh)
pub fn query_mesh<F>(mesh: &Mesh<'_>, bounds: Aabb, mut fcn: F)
where
    F: FnMut(Vec3, Vec3, Vec3, i32) -> bool,
{
    let mesh_scale = mesh.scale;
    // C names this `clockwise` but the predicate is inverted vs ray cast.
    let clockwise = mesh_scale.x * mesh_scale.y * mesh_scale.z > 0.0;

    let inv_scale = Vec3 {
        x: 1.0 / mesh_scale.x,
        y: 1.0 / mesh_scale.y,
        z: 1.0 / mesh_scale.z,
    };
    let temp1 = mul(inv_scale, bounds.lower_bound);
    let temp2 = mul(inv_scale, bounds.upper_bound);
    let inv_scaled_bounds_min = min(temp1, temp2);
    let inv_scaled_bounds_max = max(temp1, temp2);
    let inv_scaled_bounds_center =
        mul_sv(0.5, add(inv_scaled_bounds_min, inv_scaled_bounds_max));
    let inv_scaled_bounds_extent = sub(inv_scaled_bounds_max, inv_scaled_bounds_center);

    let data = mesh.data;
    let mut stack = [0i32; MESH_STACK_SIZE];
    let mut count = 0usize;
    let mut node_index = 0usize;

    let triangles = &data.triangles;
    let vertices = &data.vertices;

    loop {
        let node = &data.nodes[node_index];
        if test_bounds_overlap(
            node.lower_bound,
            node.upper_bound,
            inv_scaled_bounds_min,
            inv_scaled_bounds_max,
        ) {
            if node.is_leaf() {
                let triangle_count = node.triangle_count() as i32;
                let triangle_offset = node.triangle_offset as i32;

                for index in 0..triangle_count {
                    let triangle_index = triangle_offset + index;
                    let triangle = triangles[triangle_index as usize];

                    let vertex1 = vertices[triangle.index1 as usize];
                    let vertex2 = vertices[triangle.index2 as usize];
                    let vertex3 = vertices[triangle.index3 as usize];

                    if test_bounds_triangle_overlap(
                        inv_scaled_bounds_center,
                        inv_scaled_bounds_extent,
                        vertex1,
                        vertex2,
                        vertex3,
                    ) {
                        let a = mul(mesh_scale, vertex1);
                        let (b, c) = if clockwise {
                            (mul(mesh_scale, vertex2), mul(mesh_scale, vertex3))
                        } else {
                            (mul(mesh_scale, vertex3), mul(mesh_scale, vertex2))
                        };

                        if !fcn(a, b, c, triangle_index) {
                            return;
                        }
                    }
                }
            } else {
                debug_assert!(count <= MESH_STACK_SIZE - 1);
                stack[count] = node_index as i32 + node.child_offset() as i32;
                count += 1;
                node_index += 1;
                continue;
            }
        }

        if count == 0 {
            break;
        }
        count -= 1;
        node_index = stack[count] as usize;
    }
}
