//! Height field vertex classification for ghost collision handling.
//!
//! Port of `b3GetGridPoint`, `b3GetGridTriangleNormal`, `b3MarkGridEdge`, `b3MarkGridBoundary`,
//! and the vertex flag pass at the end of `b3CreateHeightField` in
//! `box3d-cpp-reference/src/height_field.c`. Each grid point is marked flat or concave by the
//! quantized angle across its edges, with the same rule as the mesh, and the points that are
//! neither get a set bit in the height field's vertex bit array.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{HeightFieldData, HEIGHT_FIELD_HOLE};
use crate::math_functions::{atan2, cross, dot, mul, normalize, sub, Vec3};
use crate::mesh::{quantize_edge_angle, CONCAVE_VERTEX, FLAT_VERTEX};

/// (b3GetGridPoint)
fn get_grid_point(heights: &[f32], column_count: i32, scale: Vec3, row: i32, column: i32) -> Vec3 {
    mul(
        scale,
        Vec3 {
            x: column as f32,
            y: heights[(row * column_count + column) as usize],
            z: row as f32,
        },
    )
}

/// (b3GetGridTriangleNormal)
fn get_grid_triangle_normal(
    heights: &[f32],
    column_count: i32,
    scale: Vec3,
    row: i32,
    column: i32,
    odd: bool,
) -> Vec3 {
    let point11 = get_grid_point(heights, column_count, scale, row, column);
    let point12 = get_grid_point(heights, column_count, scale, row, column + 1);
    let point21 = get_grid_point(heights, column_count, scale, row + 1, column);
    let point22 = get_grid_point(heights, column_count, scale, row + 1, column + 1);

    let v1 = if odd { point22 } else { point11 };
    let v2 = if odd { point12 } else { point21 };
    let v3 = if odd { point21 } else { point12 };
    normalize(cross(sub(v2, v1), sub(v3, v1)))
}

/// Mark vertices as flat or concave. (b3MarkGridEdge)
#[allow(clippy::too_many_arguments)]
fn mark_grid_edge(
    vertex_flags: &mut [u8],
    index1: i32,
    index2: i32,
    point1: Vec3,
    point2: Vec3,
    normal1: Vec3,
    normal2: Vec3,
    clockwise: bool,
) {
    let outward = cross(normalize(sub(point2, point1)), normal1);
    let angle = atan2(dot(normal2, outward), dot(normal2, normal1));

    // Quantize to match the mesh rule.
    let quantized = quantize_edge_angle(if clockwise { -angle } else { angle });

    if quantized != 0 {
        vertex_flags[index1 as usize] &= !FLAT_VERTEX;
        vertex_flags[index2 as usize] &= !FLAT_VERTEX;
    }

    if quantized < 0 {
        vertex_flags[index1 as usize] |= CONCAVE_VERTEX;
        vertex_flags[index2 as usize] |= CONCAVE_VERTEX;
    }
}

/// (b3MarkGridBoundary)
fn mark_grid_boundary(vertex_flags: &mut [u8], index1: i32, index2: i32) {
    vertex_flags[index1 as usize] &= !FLAT_VERTEX;
    vertex_flags[index2 as usize] &= !FLAT_VERTEX;
}

/// Classify the grid points from the decompressed heights and fill the vertex bit array.
/// (b3CreateHeightField, b3MeshVertexFlags pass)
pub(super) fn compute_vertex_flags(hf: &mut HeightFieldData, heights: &[f32]) {
    let column_count = hf.column_count;
    let row_count = hf.row_count;
    let height_count = (column_count * row_count) as usize;
    let scale = hf.scale;
    let material_indices = &hf.material_indices;

    // b3MeshVertexFlags
    let mut vertex_flags = vec![FLAT_VERTEX; height_count];

    let clockwise = hf.clockwise;
    let cell_column_count = column_count - 1;
    let cell_row_count = row_count - 1;
    let is_solid = |cell: i32| material_indices[cell as usize] != HEIGHT_FIELD_HOLE;

    for row in 0..row_count {
        for column in 0..column_count {
            let index = row * column_count + column;
            let point = get_grid_point(heights, column_count, scale, row, column);

            if column < cell_column_count {
                let below = row < cell_row_count && is_solid(row * cell_column_count + column);
                let above = row > 0 && is_solid((row - 1) * cell_column_count + column);
                if below && above {
                    let next = get_grid_point(heights, column_count, scale, row, column + 1);
                    let normal1 =
                        get_grid_triangle_normal(heights, column_count, scale, row, column, false);
                    let normal2 = get_grid_triangle_normal(
                        heights,
                        column_count,
                        scale,
                        row - 1,
                        column,
                        true,
                    );
                    mark_grid_edge(
                        &mut vertex_flags,
                        index + 1,
                        index,
                        next,
                        point,
                        normal1,
                        normal2,
                        clockwise,
                    );
                } else if below || above {
                    mark_grid_boundary(&mut vertex_flags, index, index + 1);
                }
            }

            if row < cell_row_count {
                let right =
                    column < cell_column_count && is_solid(row * cell_column_count + column);
                let left = column > 0 && is_solid(row * cell_column_count + column - 1);
                if right && left {
                    let next = get_grid_point(heights, column_count, scale, row + 1, column);
                    let normal1 =
                        get_grid_triangle_normal(heights, column_count, scale, row, column, false);
                    let normal2 = get_grid_triangle_normal(
                        heights,
                        column_count,
                        scale,
                        row,
                        column - 1,
                        true,
                    );
                    mark_grid_edge(
                        &mut vertex_flags,
                        index,
                        index + column_count,
                        point,
                        next,
                        normal1,
                        normal2,
                        clockwise,
                    );
                } else if right || left {
                    mark_grid_boundary(&mut vertex_flags, index, index + column_count);
                }
            }

            if row < cell_row_count
                && column < cell_column_count
                && is_solid(row * cell_column_count + column)
            {
                let point21 = get_grid_point(heights, column_count, scale, row + 1, column);
                let point12 = get_grid_point(heights, column_count, scale, row, column + 1);
                let normal1 =
                    get_grid_triangle_normal(heights, column_count, scale, row, column, false);
                let normal2 =
                    get_grid_triangle_normal(heights, column_count, scale, row, column, true);
                mark_grid_edge(
                    &mut vertex_flags,
                    index + column_count,
                    index + 1,
                    point21,
                    point12,
                    normal1,
                    normal2,
                    clockwise,
                );
            }
        }
    }

    for (i, &flags) in vertex_flags.iter().enumerate() {
        if flags == 0 {
            hf.vertex_bits[i >> 3] |= 1 << (i & 7);
        }
    }
}
