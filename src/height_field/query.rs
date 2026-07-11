//! Height field overlap, query, and mover collision.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::triangle::get_height_field_cell_corners;
use super::types::{get_height_field_material_indices, HeightFieldData, HEIGHT_FIELD_HOLE};
use crate::constants::linear_slop;
use crate::distance::{
    compute_proxy_aabb, make_local_proxy, make_proxy, shape_distance, DistanceInput, ShapeProxy,
    SimplexCache,
};
use crate::geometry::{Capsule, PlaneResult};
use crate::math_functions::{
    aabb_overlaps, add, max, min, mul_sv, sub, test_bounds_triangle_overlap, Aabb, Plane, Transform,
    Vec3, TRANSFORM_IDENTITY,
};

/// Test overlap between a height field and a shape proxy. (b3OverlapHeightField)
pub fn overlap_height_field(
    shape: &HeightFieldData,
    shape_transform: Transform,
    proxy: &ShapeProxy,
) -> bool {
    let local_proxy = make_local_proxy(proxy, shape_transform);
    let aabb = compute_proxy_aabb(&local_proxy);

    let scale = shape.scale;
    let min_row = (aabb.lower_bound.z / scale.z).floor() as i32;
    let max_row = (aabb.upper_bound.z / scale.z).floor() as i32;
    let min_col = (aabb.lower_bound.x / scale.x).floor() as i32;
    let max_col = (aabb.upper_bound.x / scale.x).floor() as i32;

    let bounds_center = mul_sv(0.5, add(aabb.lower_bound, aabb.upper_bound));
    let bounds_extent = sub(aabb.upper_bound, bounds_center);

    let mut input = DistanceInput {
        proxy_a: Default::default(),
        proxy_b: local_proxy,
        transform: TRANSFORM_IDENTITY,
        use_radii: true,
    };

    let mut cache = SimplexCache::default();

    for row in min_row..=max_row {
        if row < 0 || shape.row_count - 1 <= row {
            continue;
        }

        for column in min_col..=max_col {
            if column < 0 || shape.column_count - 1 <= column {
                continue;
            }

            let cell_index = (row * (shape.column_count - 1) + column) as usize;
            let material = get_height_field_material_indices(shape)[cell_index];
            if material == HEIGHT_FIELD_HOLE {
                continue;
            }

            let corners = get_height_field_cell_corners(shape, row, column);
            let point11 = corners[0];
            let point12 = corners[1];
            let point21 = corners[2];
            let point22 = corners[3];

            if test_bounds_triangle_overlap(bounds_center, bounds_extent, point11, point21, point12)
            {
                let triangle_vertices = [point11, point21, point12];
                input.proxy_a = make_proxy(&triangle_vertices, 0.0);
                cache.count = 0;

                let output = shape_distance(&input, &mut cache, None);
                let tolerance = 0.1 * linear_slop();
                if output.distance < tolerance {
                    return true;
                }
            }

            if test_bounds_triangle_overlap(bounds_center, bounds_extent, point21, point22, point12)
            {
                let triangle_vertices = [point22, point12, point21];
                input.proxy_a = make_proxy(&triangle_vertices, 0.0);
                cache.count = 0;

                let output = shape_distance(&input, &mut cache, None);
                let tolerance = 0.1 * linear_slop();
                if output.distance < tolerance {
                    return true;
                }
            }
        }
    }

    false
}

/// Query height field triangles overlapping an AABB.
/// Callback receives (a, b, c, triangle_index) and may return `false` to stop early.
/// (b3QueryHeightField)
pub fn query_height_field<F>(height_field: &HeightFieldData, bounds: Aabb, mut fcn: F)
where
    F: FnMut(Vec3, Vec3, Vec3, i32) -> bool,
{
    let scale = height_field.scale;

    let min_row = (bounds.lower_bound.z / scale.z).floor() as i32;
    let max_row = (bounds.upper_bound.z / scale.z).floor() as i32;
    let min_col = (bounds.lower_bound.x / scale.x).floor() as i32;
    let max_col = (bounds.upper_bound.x / scale.x).floor() as i32;

    for row in min_row..=max_row {
        if row < 0 || height_field.row_count - 1 <= row {
            continue;
        }

        for column in min_col..=max_col {
            if column < 0 || height_field.column_count - 1 <= column {
                continue;
            }

            let cell_index = (row * (height_field.column_count - 1) + column) as usize;
            let material = get_height_field_material_indices(height_field)[cell_index];
            if material == HEIGHT_FIELD_HOLE {
                continue;
            }

            let corners = get_height_field_cell_corners(height_field, row, column);
            let point11 = corners[0];
            let point12 = corners[1];
            let point21 = corners[2];
            let point22 = corners[3];

            let cell_bound = Aabb {
                lower_bound: min(min(point11, point12), min(point21, point22)),
                upper_bound: max(max(point11, point12), max(point21, point22)),
            };

            if aabb_overlaps(bounds, cell_bound) {
                let quad_index = row * (height_field.column_count - 1) + column;
                let triangle_index = 2 * quad_index;

                if height_field.clockwise {
                    if !fcn(point11, point12, point21, triangle_index) {
                        return;
                    }
                    if !fcn(point22, point21, point12, triangle_index + 1) {
                        return;
                    }
                } else if !fcn(point11, point21, point12, triangle_index) {
                    return;
                } else if !fcn(point22, point12, point21, triangle_index + 1) {
                    return;
                }
            }
        }
    }
}

/// Collide a character mover capsule against a height field.
/// (b3CollideMoverAndHeightField)
pub fn collide_mover_and_height_field(
    planes: &mut [PlaneResult],
    shape: &HeightFieldData,
    mover: &Capsule,
) -> i32 {
    let capacity = planes.len() as i32;
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
    let bounds_center = mul_sv(0.5, add(bounds_min, bounds_max));
    let bounds_extent = sub(bounds_max, bounds_center);

    let scale = shape.scale;
    let min_row = (bounds_min.z / scale.z).floor() as i32;
    let max_row = (bounds_max.z / scale.z).floor() as i32;
    let min_col = (bounds_min.x / scale.x).floor() as i32;
    let max_col = (bounds_max.x / scale.x).floor() as i32;

    let mut plane_count = 0i32;

    for row in min_row..=max_row {
        if row < 0 || shape.row_count - 1 <= row {
            continue;
        }

        for column in min_col..=max_col {
            if column < 0 || shape.column_count - 1 <= column {
                continue;
            }

            let cell_index = (row * (shape.column_count - 1) + column) as usize;
            let material = get_height_field_material_indices(shape)[cell_index];
            if material == HEIGHT_FIELD_HOLE {
                continue;
            }

            let corners = get_height_field_cell_corners(shape, row, column);
            let point11 = corners[0];
            let point12 = corners[1];
            let point21 = corners[2];
            let point22 = corners[3];

            if test_bounds_triangle_overlap(bounds_center, bounds_extent, point11, point21, point12)
            {
                let triangle_vertices = [point11, point21, point12];
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

            if test_bounds_triangle_overlap(bounds_center, bounds_extent, point21, point22, point12)
            {
                let triangle_vertices = [point22, point12, point21];
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
    }

    plane_count
}
