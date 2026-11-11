//! Height field neighbor lookups for ghost collision handling.
//!
//! Port of `b3GetHeightFieldNeighborNormal`, `b3IsHeightFieldVertexConvex`, and
//! `b3GetHeightFieldVertexTriangles` from `box3d-cpp-reference/src/height_field.c`.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::triangle::get_height_field_triangle;
use super::types::{
    get_height_field_material_indices, get_height_field_triangle_count, HeightFieldData,
    HEIGHT_FIELD_HOLE,
};
use crate::math_functions::{cross, normalize, sub, Vec3};

/// The normal of the triangle across an edge. `edge_index` 0, 1, 2 is the edge from vertex
/// 1, 2, 3 of the triangle. Returns `None` at the grid boundary and next to holes.
/// (b3GetHeightFieldNeighborNormal)
pub(crate) fn get_height_field_neighbor_normal(
    height_field: &HeightFieldData,
    triangle_index: i32,
    edge_index: i32,
) -> Option<Vec3> {
    debug_assert!(
        0 <= triangle_index && triangle_index < get_height_field_triangle_count(height_field)
    );
    debug_assert!((0..3).contains(&edge_index));

    let mut edge_index = edge_index;
    if height_field.clockwise {
        // Reflected
        edge_index = 2 - edge_index;
    }

    let cell_column_count = height_field.column_count - 1;
    let cell_row_count = height_field.row_count - 1;
    let cell_index = triangle_index >> 1;
    let mut row = cell_index / cell_column_count;
    let mut column = cell_index - row * cell_column_count;
    let odd = (triangle_index & 1) == 1;

    // Get the row and column for the neighbor cell.
    if edge_index == 0 {
        column += if odd { 1 } else { -1 };
    } else if edge_index == 2 {
        row += if odd { 1 } else { -1 };
    }

    if row < 0 || row >= cell_row_count || column < 0 || column >= cell_column_count {
        return None;
    }

    let neighbor_cell = row * cell_column_count + column;

    let material_indices = get_height_field_material_indices(height_field);
    if material_indices[neighbor_cell as usize] == HEIGHT_FIELD_HOLE {
        return None;
    }

    let neighbor =
        get_height_field_triangle(height_field, 2 * neighbor_cell + if odd { 0 } else { 1 });
    let e1 = sub(neighbor.vertices[1], neighbor.vertices[0]);
    let e2 = sub(neighbor.vertices[2], neighbor.vertices[0]);
    Some(normalize(cross(e1, e2)))
}

/// Is a corner of the triangle a convex vertex (neither flat nor concave)?
/// (b3IsHeightFieldVertexConvex)
pub(crate) fn is_height_field_vertex_convex(
    height_field: &HeightFieldData,
    triangle_index: i32,
    vertex_index: i32,
) -> bool {
    debug_assert!(
        0 <= triangle_index && triangle_index < get_height_field_triangle_count(height_field)
    );
    debug_assert!((0..3).contains(&vertex_index));

    if height_field.vertex_flag_offset == 0 {
        return false;
    }

    let mut vertex_index = vertex_index;
    if height_field.clockwise {
        vertex_index = (3 - vertex_index) % 3;
    }

    let column_count = height_field.column_count;
    let cell_index = triangle_index >> 1;
    let row = cell_index / (column_count - 1);
    let column = cell_index - row * (column_count - 1);

    let offsets = [[0, column_count, 1], [column_count + 1, 1, column_count]];

    let index =
        row * column_count + column + offsets[(triangle_index & 1) as usize][vertex_index as usize];
    let vertex_bits = &height_field.vertex_bits;
    (vertex_bits[(index >> 3) as usize] & (1 << (index & 7))) != 0
}

/// Collect the triangles around a grid point, skipping holes. Returns the count, at most 6.
/// (b3GetHeightFieldVertexTriangles)
pub(crate) fn get_height_field_vertex_triangles(
    height_field: &HeightFieldData,
    vertex: i32,
    triangles: &mut [i32; 6],
) -> i32 {
    let column_count = height_field.column_count;
    let cell_column_count = column_count - 1;
    let cell_row_count = height_field.row_count - 1;
    let row = vertex / column_count;
    let column = vertex - row * column_count;

    let row_offsets = [0, 0, -1, -1];
    let column_offsets = [0, -1, 0, -1];
    let first_parity = [0, 0, 0, 1];
    let last_parity = [0, 1, 1, 1];

    let material_indices = get_height_field_material_indices(height_field);
    let mut count = 0;
    for i in 0..4 {
        let cell_row = row + row_offsets[i];
        let cell_column = column + column_offsets[i];
        if cell_row < 0
            || cell_row >= cell_row_count
            || cell_column < 0
            || cell_column >= cell_column_count
        {
            continue;
        }

        let cell_index = cell_row * cell_column_count + cell_column;
        if material_indices[cell_index as usize] == HEIGHT_FIELD_HOLE {
            continue;
        }

        for parity in first_parity[i]..=last_parity[i] {
            triangles[count as usize] = 2 * cell_index + parity;
            count += 1;
        }
    }

    count
}
