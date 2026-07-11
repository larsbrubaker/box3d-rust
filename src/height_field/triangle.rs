//! Height field triangle accessors.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{
    get_height_field_flags, get_height_field_material_indices, HeightFieldData, CONCAVE_EDGE1,
    CONCAVE_EDGE3, HEIGHT_FIELD_HOLE, INVERSE_CONCAVE_EDGE1, INVERSE_CONCAVE_EDGE3,
};
use crate::math_functions::{mul, Triangle, Vec3};

/// Decode the four corner vertices of a height field cell into local space.
/// Output order: corners[0]=(column,row), [1]=(column+1,row),
/// [2]=(column,row+1), [3]=(column+1,row+1).
/// (static b3GetHeightFieldCellCorners)
pub(crate) fn get_height_field_cell_corners(
    hf: &HeightFieldData,
    row: i32,
    column: i32,
) -> [Vec3; 4] {
    debug_assert!(0 <= row && row < hf.row_count - 1);
    debug_assert!(0 <= column && column < hf.column_count - 1);

    let column_count = hf.column_count;
    let index11 = (row * column_count + column) as usize;
    let index12 = index11 + 1;
    let index21 = ((row + 1) * column_count + column) as usize;
    let index22 = index21 + 1;

    let min_height = hf.min_height;
    let height_scale = hf.height_scale;
    let heights = &hf.compressed_heights;

    let height11 = min_height + height_scale * (heights[index11] as f32);
    let height12 = min_height + height_scale * (heights[index12] as f32);
    let height21 = min_height + height_scale * (heights[index21] as f32);
    let height22 = min_height + height_scale * (heights[index22] as f32);

    let x1 = column as f32;
    let x2 = (column + 1) as f32;
    let z1 = row as f32;
    let z2 = (row + 1) as f32;

    let scale = hf.scale;
    [
        mul(
            scale,
            Vec3 {
                x: x1,
                y: height11,
                z: z1,
            },
        ),
        mul(
            scale,
            Vec3 {
                x: x2,
                y: height12,
                z: z1,
            },
        ),
        mul(
            scale,
            Vec3 {
                x: x1,
                y: height21,
                z: z2,
            },
        ),
        mul(
            scale,
            Vec3 {
                x: x2,
                y: height22,
                z: z2,
            },
        ),
    ]
}

/// Get a height field triangle by index. (b3GetHeightFieldTriangle)
pub fn get_height_field_triangle(height_field: &HeightFieldData, triangle_index: i32) -> Triangle {
    debug_assert!(0 <= triangle_index);
    debug_assert!(
        triangle_index < 2 * (height_field.column_count - 1) * (height_field.row_count - 1)
    );

    let mut triangle = Triangle {
        flags: get_height_field_flags(height_field)[triangle_index as usize] as i32,
        ..Default::default()
    };

    let column_count = height_field.column_count;
    let quad_index = triangle_index >> 1;
    let row = quad_index / (column_count - 1);
    let column = quad_index - row * (column_count - 1);

    let index11 = row * column_count + column;
    let index12 = index11 + 1;
    let index21 = (row + 1) * column_count + column;
    let index22 = index21 + 1;

    let cell_index = row * (column_count - 1) + column;
    debug_assert!(quad_index == cell_index);
    debug_assert!(
        get_height_field_material_indices(height_field)[cell_index as usize] != HEIGHT_FIELD_HOLE
    );

    let corners = get_height_field_cell_corners(height_field, row, column);

    if (triangle_index & 1) == 0 {
        // Triangle 0 (CCW): {11, 21, 12}
        triangle.vertices[0] = corners[0];
        triangle.vertices[1] = corners[2];
        triangle.vertices[2] = corners[1];
        triangle.i1 = index11;
        triangle.i2 = index21;
        triangle.i3 = index12;
    } else {
        // Triangle 1 (CCW): {22, 12, 21}
        triangle.vertices[0] = corners[3];
        triangle.vertices[1] = corners[1];
        triangle.vertices[2] = corners[2];
        triangle.i1 = index22;
        triangle.i2 = index12;
        triangle.i3 = index21;
    }

    if height_field.clockwise {
        triangle.vertices.swap(1, 2);
        core::mem::swap(&mut triangle.i2, &mut triangle.i3);

        // Reversing winding swaps edge1 and edge3; edge2 (the diagonal) is preserved.
        let mut flags = triangle.flags;
        let edge1_bits = flags & (CONCAVE_EDGE1 | INVERSE_CONCAVE_EDGE1);
        let edge3_bits = flags & (CONCAVE_EDGE3 | INVERSE_CONCAVE_EDGE3);
        flags &= !(CONCAVE_EDGE1 | CONCAVE_EDGE3 | INVERSE_CONCAVE_EDGE1 | INVERSE_CONCAVE_EDGE3);
        flags |= edge1_bits << 2;
        flags |= edge3_bits >> 2;
        triangle.flags = flags;
    }

    triangle
}

/// Get the material index for a triangle. (b3GetHeightFieldMaterial)
pub fn get_height_field_material(height_field: &HeightFieldData, triangle_index: i32) -> i32 {
    debug_assert!(0 <= triangle_index);
    debug_assert!(
        triangle_index < 2 * (height_field.column_count - 1) * (height_field.row_count - 1)
    );

    let cell_index = triangle_index >> 1;
    get_height_field_material_indices(height_field)[cell_index as usize] as i32
}
