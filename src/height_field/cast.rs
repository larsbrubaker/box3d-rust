//! Height field ray and shape casts.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::triangle::get_height_field_cell_corners;
use super::types::{get_height_field_material_indices, HeightFieldData, HEIGHT_FIELD_HOLE};
use crate::aabb::ray_cast_aabb;
use crate::constants::max_aabb_margin;
use crate::distance::{make_proxy, shape_cast, CastOutput, ShapeCastPairInput};
use crate::geometry::{RayCastInput, ShapeCastInput};
use crate::math_functions::{
    aabb_center, aabb_extents, aabb_overlaps, abs, add, cross, intersect_ray_triangle, make_aabb,
    max, min, mul_add, mul_sv, normalize, sub, Aabb, Transform, Vec3, TRANSFORM_IDENTITY,
    VEC3_ZERO,
};

/// Compute the AABB of a height field. (b3ComputeHeightFieldAABB)
pub fn compute_height_field_aabb(shape: &HeightFieldData, transform: Transform) -> Aabb {
    crate::math_functions::aabb_transform(transform, shape.aabb)
}

/// Ray cast versus a height field in local space. (b3RayCastHeightField)
pub fn ray_cast_height_field(height_field: &HeightFieldData, input: &RayCastInput) -> CastOutput {
    let shape_cast_input = ShapeCastInput {
        proxy: make_proxy(&[input.origin], 0.0),
        translation: input.translation,
        max_fraction: input.max_fraction,
        can_encroach: false,
    };
    shape_cast_height_field(height_field, &shape_cast_input)
}

/// Shape cast versus a height field in local space. (b3ShapeCastHeightField)
pub fn shape_cast_height_field(
    height_field: &HeightFieldData,
    input: &ShapeCastInput,
) -> CastOutput {
    let shape_bounds = make_aabb(
        &input.proxy.points[..input.proxy.count as usize],
        input.proxy.radius,
    );
    let shape_translation = input.translation;
    let scale = height_field.scale;

    let shape_start = aabb_center(shape_bounds);
    let shape_delta = mul_sv(input.max_fraction, shape_translation);
    let shape_end = add(shape_start, shape_delta);

    let mut result = CastOutput::default();

    let shape_extents = aabb_extents(shape_bounds);
    let margin = max_aabb_margin();
    let margin_v = Vec3 {
        x: margin,
        y: margin,
        z: margin,
    };
    let combined_bounds = Aabb {
        lower_bound: sub(sub(height_field.aabb.lower_bound, shape_extents), margin_v),
        upper_bound: add(add(height_field.aabb.upper_bound, shape_extents), margin_v),
    };

    let mut min_fraction = 0.0;
    let mut max_fraction = 0.0;
    let intersects = ray_cast_aabb(
        combined_bounds,
        shape_start,
        shape_end,
        &mut min_fraction,
        &mut max_fraction,
    );
    if !intersects {
        return result;
    }

    // These are for walking the grid, not the triangle cast.
    let mut clamped_start = mul_add(shape_start, min_fraction, shape_delta);
    let clamped_delta = mul_sv(max_fraction - min_fraction, shape_delta);
    let mut clamped_end = add(clamped_start, clamped_delta);

    // Preserve the un-shifted center sweep for the swept-volume AABB cull.
    let center_start = clamped_start;
    let center_end = clamped_end;

    // The grid traversal starts from the leading shape bounds corner
    let sign_x;
    if shape_translation.x >= 0.0 {
        clamped_start.x += shape_extents.x;
        sign_x = 1.0;
    } else {
        clamped_start.x -= shape_extents.x;
        sign_x = -1.0;
    }

    let sign_z;
    if shape_translation.z >= 0.0 {
        clamped_start.z += shape_extents.z;
        sign_z = 1.0;
    } else {
        clamped_start.z -= shape_extents.z;
        sign_z = -1.0;
    }

    clamped_end = add(clamped_start, clamped_delta);

    let column_start = (clamped_start.x / scale.x).floor() as i32;
    let column_end = (clamped_end.x / scale.x).floor() as i32;
    let row_start = (clamped_start.z / scale.z).floor() as i32;
    let row_end = (clamped_end.z / scale.z).floor() as i32;

    let abs_clamped_delta = abs(clamped_delta);

    let delta_alpha_x;
    let mut next_fraction_x;
    let delta_column;

    if column_start < column_end {
        debug_assert!(abs_clamped_delta.x > 0.0);
        delta_alpha_x = scale.x / abs_clamped_delta.x;
        next_fraction_x =
            (scale.x * ((column_start + 1) as f32) - clamped_start.x) / abs_clamped_delta.x;
        delta_column = 1;
    } else if column_end < column_start {
        debug_assert!(abs_clamped_delta.x > 0.0);
        delta_alpha_x = scale.x / abs_clamped_delta.x;
        next_fraction_x = (clamped_start.x - scale.x * (column_start as f32)) / abs_clamped_delta.x;
        delta_column = -1;
    } else {
        delta_alpha_x = 0.0;
        next_fraction_x = f32::MAX;
        delta_column = 0;
    }

    let delta_alpha_z;
    let mut next_fraction_z;
    let delta_row;

    if row_start < row_end {
        debug_assert!(abs_clamped_delta.z > 0.0);
        delta_alpha_z = scale.z / abs_clamped_delta.z;
        next_fraction_z =
            (scale.z * ((row_start + 1) as f32) - clamped_start.z) / abs_clamped_delta.z;
        delta_row = 1;
    } else if row_end < row_start {
        debug_assert!(abs_clamped_delta.z > 0.0);
        delta_alpha_z = scale.z / abs_clamped_delta.z;
        next_fraction_z = (clamped_start.z - scale.z * (row_start as f32)) / abs_clamped_delta.z;
        delta_row = -1;
    } else {
        delta_alpha_z = 0.0;
        next_fraction_z = f32::MAX;
        delta_row = 0;
    }

    let mut box_column_head = column_start;
    let mut box_row_head = row_start;

    let mut box_column_tail =
        ((clamped_start.x - 2.0 * sign_x * shape_extents.x) / scale.x).floor() as i32;
    let mut box_row_tail =
        ((clamped_start.z - 2.0 * sign_z * shape_extents.z) / scale.z).floor() as i32;

    let mut best_fraction = input.max_fraction;

    let grid_fraction_scale = input.max_fraction * (max_fraction - min_fraction);
    let grid_fraction_offset = input.max_fraction * min_fraction;

    let row_count = height_field.row_count;
    let column_count = height_field.column_count;

    let mut pair_input = ShapeCastPairInput {
        proxy_a: Default::default(),
        proxy_b: input.proxy,
        transform: TRANSFORM_IDENTITY,
        translation_b: input.translation,
        max_fraction: 0.0,
        can_encroach: input.can_encroach,
    };

    let cast_bounds = Aabb {
        lower_bound: sub(min(center_start, center_end), shape_extents),
        upper_bound: add(max(center_start, center_end), shape_extents),
    };

    let ray_origin = shape_start;
    let ray_translation = shape_translation;

    loop {
        let (column1, column2) = if box_column_tail < box_column_head {
            (box_column_tail, box_column_head)
        } else {
            (box_column_head, box_column_tail)
        };

        let (row1, row2) = if box_row_tail < box_row_head {
            (box_row_tail, box_row_head)
        } else {
            (box_row_head, box_row_tail)
        };

        for row in row1..=row2 {
            if row < 0 || row_count - 1 <= row {
                continue;
            }

            for column in column1..=column2 {
                if column < 0 || column_count - 1 <= column {
                    continue;
                }

                let cell_index = (row * (column_count - 1) + column) as usize;
                let material_index = get_height_field_material_indices(height_field)[cell_index];
                if material_index == HEIGHT_FIELD_HOLE {
                    continue;
                }

                let corners = get_height_field_cell_corners(height_field, row, column);
                let point11 = corners[0];
                let point12 = corners[1];
                let point21 = corners[2];
                let point22 = corners[3];

                let bounds = Aabb {
                    lower_bound: min(min(point11, point12), min(point21, point22)),
                    upper_bound: max(max(point11, point12), max(point21, point22)),
                };

                if !aabb_overlaps(cast_bounds, bounds) {
                    continue;
                }

                let quad_index = row * (column_count - 1) + column;
                let triangle_index1 = 2 * quad_index;
                let triangle_index2 = triangle_index1 + 1;

                if input.proxy.count == 1 && input.proxy.radius == 0.0 {
                    // Ray cast
                    {
                        let vertex1 = point11;
                        let (vertex2, vertex3) = if height_field.clockwise {
                            (point12, point21)
                        } else {
                            (point21, point12)
                        };

                        let alpha = intersect_ray_triangle(
                            ray_origin,
                            ray_translation,
                            vertex1,
                            vertex2,
                            vertex3,
                        );
                        debug_assert!((0.0..=1.0).contains(&alpha));

                        if alpha < best_fraction {
                            let edge1 = sub(point21, point11);
                            let edge2 = sub(point12, point11);
                            let normal = if height_field.clockwise {
                                cross(edge2, edge1)
                            } else {
                                cross(edge1, edge2)
                            };

                            result.point = mul_add(shape_start, alpha, shape_translation);
                            result.normal = normalize(normal);
                            result.fraction = alpha;
                            result.triangle_index = triangle_index1;
                            result.material_index = material_index as i32;
                            result.hit = true;
                            best_fraction = alpha;
                        }
                    }

                    {
                        let vertex1 = point22;
                        let (vertex2, vertex3) = if height_field.clockwise {
                            (point21, point12)
                        } else {
                            (point12, point21)
                        };

                        let alpha = intersect_ray_triangle(
                            ray_origin,
                            ray_translation,
                            vertex1,
                            vertex2,
                            vertex3,
                        );
                        debug_assert!((0.0..=1.0).contains(&alpha));

                        if alpha < best_fraction {
                            let edge1 = sub(point22, point21);
                            let edge2 = sub(point12, point21);
                            let normal = if height_field.clockwise {
                                cross(edge2, edge1)
                            } else {
                                cross(edge1, edge2)
                            };

                            result.point = mul_add(shape_start, alpha, shape_translation);
                            result.normal = normalize(normal);
                            result.fraction = alpha;
                            result.triangle_index = triangle_index2;
                            result.material_index = material_index as i32;
                            result.hit = true;
                            best_fraction = alpha;
                        }
                    }
                } else {
                    // Shape cast
                    {
                        let origin = point11;
                        let triangle_vertices =
                            [VEC3_ZERO, sub(point21, origin), sub(point12, origin)];
                        pair_input.proxy_a = make_proxy(&triangle_vertices, 0.0);
                        pair_input.max_fraction = best_fraction;
                        pair_input.transform = Transform {
                            q: TRANSFORM_IDENTITY.q,
                            p: crate::math_functions::neg(origin),
                        };

                        let pair_output = shape_cast(&pair_input);
                        if pair_output.hit {
                            best_fraction = pair_output.fraction;
                            result = pair_output;
                            result.point = add(result.point, origin);
                            result.triangle_index = triangle_index1;
                            result.material_index = material_index as i32;
                        }
                    }

                    {
                        let origin = point21;
                        let triangle_vertices =
                            [VEC3_ZERO, sub(point22, origin), sub(point12, origin)];
                        pair_input.proxy_a = make_proxy(&triangle_vertices, 0.0);
                        pair_input.max_fraction = best_fraction;
                        pair_input.transform = Transform {
                            q: TRANSFORM_IDENTITY.q,
                            p: crate::math_functions::neg(origin),
                        };

                        let pair_output = shape_cast(&pair_input);
                        if pair_output.hit {
                            best_fraction = pair_output.fraction;
                            result = pair_output;
                            result.point = add(result.point, origin);
                            result.triangle_index = triangle_index2;
                            result.material_index = material_index as i32;
                        }
                    }
                }
            }
        }

        let input_fraction_x = if next_fraction_x == f32::MAX {
            f32::MAX
        } else {
            grid_fraction_offset + next_fraction_x * grid_fraction_scale
        };
        let input_fraction_z = if next_fraction_z == f32::MAX {
            f32::MAX
        } else {
            grid_fraction_offset + next_fraction_z * grid_fraction_scale
        };
        if input_fraction_x > best_fraction && input_fraction_z > best_fraction {
            break;
        }

        if next_fraction_x <= next_fraction_z {
            if box_column_head == column_end {
                break;
            }

            box_column_head += delta_column;
            box_column_tail = box_column_head;

            if shape_extents.z == 0.0 {
                box_row_tail = box_row_head;
            } else {
                let row_intercept = clamped_start.z + next_fraction_x * clamped_delta.z;
                box_row_tail =
                    ((row_intercept - 2.0 * sign_z * shape_extents.z) / scale.z).floor() as i32;
            }

            next_fraction_x += delta_alpha_x;
        } else {
            if box_row_head == row_end {
                break;
            }

            box_row_head += delta_row;
            box_row_tail = box_row_head;

            if shape_extents.x == 0.0 {
                box_column_tail = box_column_head;
            } else {
                let column_intercept = clamped_start.x + next_fraction_z * clamped_delta.x;
                box_column_tail =
                    ((column_intercept - 2.0 * sign_x * shape_extents.x) / scale.x).floor() as i32;
            }

            next_fraction_z += delta_alpha_z;
        }
    }

    result
}
