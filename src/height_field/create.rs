//! Height field creation and convexity flag computation.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{
    HeightFieldData, HeightFieldDef, CONCAVE_EDGE1, CONCAVE_EDGE2, CONCAVE_EDGE3,
    HEIGHT_FIELD_DATA_SIZE, HEIGHT_FIELD_HOLE, HEIGHT_FIELD_VERSION, INVERSE_CONCAVE_EDGE1,
    INVERSE_CONCAVE_EDGE2, INVERSE_CONCAVE_EDGE3,
};
use crate::constants::linear_slop;
use crate::core::{hash, non_zero_hash, HASH_INIT};
use crate::math_functions::{
    align_up8, clamp_float, cross, dot, make_plane_from_points, max_float, min_float, mul,
    normalize, plane_separation, sub, Aabb, Vec3,
};

const _: () = assert!(CONCAVE_EDGE3 == 4 * CONCAVE_EDGE1);
const _: () = assert!(INVERSE_CONCAVE_EDGE3 == 4 * INVERSE_CONCAVE_EDGE1);

fn finalize_hash(hf: &mut HeightFieldData) {
    let bytes = hf.to_bytes_with_hash(0);
    hf.hash = non_zero_hash(hash(HASH_INIT, &bytes));
}

/// Create a height field from a definition. (b3CreateHeightField)
pub fn create_height_field(data: &HeightFieldDef) -> HeightFieldData {
    let column_count = data.count_x;
    let row_count = data.count_z;

    let height_count = (column_count * row_count) as usize;
    debug_assert!(height_count >= 4);

    let cell_count = ((column_count - 1) * (row_count - 1)) as usize;
    let triangle_count = 2 * cell_count;

    let mut byte_count = align_up8(HEIGHT_FIELD_DATA_SIZE);
    let heights_offset = byte_count as i32;
    byte_count += align_up8(height_count * core::mem::size_of::<u16>());
    let material_offset = byte_count as i32;
    byte_count += align_up8(cell_count * core::mem::size_of::<u8>());
    let flags_offset = byte_count as i32;
    byte_count += align_up8(triangle_count * core::mem::size_of::<u8>());

    let mut hf = HeightFieldData {
        version: HEIGHT_FIELD_VERSION,
        byte_count: byte_count as i32,
        hash: 0,
        aabb: Aabb::default(),
        min_height: 0.0,
        max_height: 0.0,
        height_scale: 0.0,
        scale: data.scale,
        column_count,
        row_count,
        heights_offset,
        material_offset,
        flags_offset,
        clockwise: data.clockwise_winding,
        padding: [0; 3],
        compressed_heights: vec![0; height_count],
        material_indices: vec![0; cell_count],
        flags: vec![0; triangle_count],
    };

    debug_assert!(data.global_minimum_height <= data.global_maximum_height);
    hf.min_height = data.global_minimum_height;
    hf.max_height = data.global_maximum_height;

    let height = max_float(hf.max_height - hf.min_height, linear_slop());
    hf.height_scale = height / (u16::MAX as f32);

    let mut lower_height_bound = hf.max_height;
    let mut upper_height_bound = hf.min_height;

    let inv_height_scale = 1.0 / hf.height_scale;
    for i in 0..height_count {
        let clamped_height = clamp_float(data.heights[i], hf.min_height, hf.max_height);
        let scaled_height = (clamped_height - hf.min_height) * inv_height_scale;
        hf.compressed_heights[i] = min_float(scaled_height, u16::MAX as f32) as u16;

        lower_height_bound = min_float(lower_height_bound, clamped_height);
        upper_height_bound = max_float(upper_height_bound, clamped_height);
    }

    // Use decompressed heights for accurate convexity metrics.
    let mut decompressed_heights = vec![0.0f32; height_count];
    for i in 0..height_count {
        decompressed_heights[i] =
            hf.min_height + hf.height_scale * (hf.compressed_heights[i] as f32);
    }

    if data.material_indices.is_empty() {
        // already zeroed
    } else {
        debug_assert!(data.material_indices.len() >= cell_count);
        hf.material_indices[..cell_count].copy_from_slice(&data.material_indices[..cell_count]);
    }

    hf.aabb.lower_bound = Vec3 {
        x: 0.0,
        y: hf.scale.y * lower_height_bound,
        z: 0.0,
    };
    hf.aabb.upper_bound = Vec3 {
        x: hf.scale.x * ((hf.column_count - 1) as f32),
        y: hf.scale.y * upper_height_bound,
        z: hf.scale.z * ((hf.row_count - 1) as f32),
    };

    compute_convexity_flags(&mut hf, &decompressed_heights);

    finalize_hash(&mut hf);
    hf
}

fn compute_convexity_flags(hf: &mut HeightFieldData, heights: &[f32]) {
    let cos5_deg = 0.9962f32;
    let scale = hf.scale;
    let column_count = hf.column_count;
    let row_count = hf.row_count;
    let cell_count = ((column_count - 1) * (row_count - 1)) as usize;

    let mut triangle_index = 0i32;
    for row in 0..row_count - 1 {
        for column in 0..column_count - 1 {
            let triangle_index1 = triangle_index as usize;
            let triangle_index2 = (triangle_index + 1) as usize;
            triangle_index += 2;

            let cell_index = (row * (column_count - 1) + column) as usize;

            if hf.material_indices[cell_index] == HEIGHT_FIELD_HOLE {
                continue;
            }

            let mut flags1 = 0i32;
            let mut flags2 = 0i32;

            let index11 = (row * column_count + column) as usize;
            let index12 = index11 + 1;
            let index21 = ((row + 1) * column_count + column) as usize;
            let index22 = index21 + 1;

            let height11 = heights[index11];
            let height12 = heights[index12];
            let height21 = heights[index21];
            let height22 = heights[index22];

            let x1 = column as f32;
            let x2 = (column + 1) as f32;
            let z1 = row as f32;
            let z2 = (row + 1) as f32;

            // triangle 0 : 11, 21, 12
            let vs0 = [
                mul(scale, Vec3 {
                    x: x1,
                    y: height11,
                    z: z1,
                }),
                mul(scale, Vec3 {
                    x: x1,
                    y: height21,
                    z: z2,
                }),
                mul(scale, Vec3 {
                    x: x2,
                    y: height12,
                    z: z1,
                }),
            ];
            let plane1 = make_plane_from_points(vs0[0], vs0[1], vs0[2]);

            // triangle 1 : 22, 12, 21
            let vs1 = [
                mul(scale, Vec3 {
                    x: x2,
                    y: height22,
                    z: z2,
                }),
                mul(scale, Vec3 {
                    x: x2,
                    y: height12,
                    z: z1,
                }),
                mul(scale, Vec3 {
                    x: x1,
                    y: height21,
                    z: z2,
                }),
            ];
            let plane2 = make_plane_from_points(vs1[0], vs1[1], vs1[2]);

            let separation = plane_separation(plane1, vs1[0]);
            let cos_angle = dot(plane1.normal, plane2.normal);
            if separation > 0.0 || cos_angle > cos5_deg {
                flags1 |= CONCAVE_EDGE2;
                flags2 |= CONCAVE_EDGE2;
            }
            if separation < 0.0 || cos_angle > cos5_deg {
                flags1 |= INVERSE_CONCAVE_EDGE2;
                flags2 |= INVERSE_CONCAVE_EDGE2;
            }

            // top
            let top_cell_index = ((row - 1) * (column_count - 1) + column) as isize;
            if row > 0
                && top_cell_index >= 0
                && (top_cell_index as usize) < cell_count
                && hf.material_indices[top_cell_index as usize] != HEIGHT_FIELD_HOLE
            {
                let r = row - 1;
                let c = column;

                let i11 = (r * column_count + c) as usize;
                let i12 = i11 + 1;
                let i21 = ((r + 1) * column_count + c) as usize;
                let i22 = i21 + 1;

                debug_assert!(i21 == index11);
                debug_assert!(i22 == index12);
                let _ = i11;

                let h12 = heights[i12];
                let h21 = heights[i21];
                let h22 = heights[i22];

                let x1 = c as f32;
                let x2 = (c + 1) as f32;
                let z1 = r as f32;
                let z2 = (r + 1) as f32;

                let vs = [
                    mul(scale, Vec3 {
                        x: x2,
                        y: h22,
                        z: z2,
                    }),
                    mul(scale, Vec3 {
                        x: x2,
                        y: h12,
                        z: z1,
                    }),
                    mul(scale, Vec3 {
                        x: x1,
                        y: h21,
                        z: z2,
                    }),
                ];

                let n = normalize(cross(sub(vs[1], vs[0]), sub(vs[2], vs[0])));

                let separation = plane_separation(plane1, vs[1]);
                let cos_angle = dot(plane1.normal, n);
                if separation > 0.0 || cos_angle > cos5_deg {
                    flags1 |= CONCAVE_EDGE3;
                }
                if separation < 0.0 || cos_angle > cos5_deg {
                    flags1 |= INVERSE_CONCAVE_EDGE3;
                }
            }

            let bottom_cell_index = ((row + 1) * (column_count - 1) + column) as usize;
            if row + 1 < row_count - 1
                && bottom_cell_index < cell_count
                && hf.material_indices[bottom_cell_index] != HEIGHT_FIELD_HOLE
            {
                let r = row + 1;
                let c = column;

                let i11 = (r * column_count + c) as usize;
                let i12 = i11 + 1;
                let i21 = ((r + 1) * column_count + c) as usize;

                debug_assert!(i11 == index21);
                debug_assert!(i12 == index22);

                let h11 = heights[i11];
                let h12 = heights[i12];
                let h21 = heights[i21];

                let x1 = c as f32;
                let x2 = (c + 1) as f32;
                let z1 = r as f32;
                let z2 = (r + 1) as f32;

                let vs = [
                    mul(scale, Vec3 {
                        x: x1,
                        y: h11,
                        z: z1,
                    }),
                    mul(scale, Vec3 {
                        x: x1,
                        y: h21,
                        z: z2,
                    }),
                    mul(scale, Vec3 {
                        x: x2,
                        y: h12,
                        z: z1,
                    }),
                ];

                let n = normalize(cross(sub(vs[1], vs[0]), sub(vs[2], vs[0])));

                let separation = plane_separation(plane2, vs[1]);
                let cos_angle = dot(plane2.normal, n);
                if separation > 0.0 || cos_angle > cos5_deg {
                    flags2 |= CONCAVE_EDGE3;
                }
                if separation < 0.0 || cos_angle > cos5_deg {
                    flags2 |= INVERSE_CONCAVE_EDGE3;
                }
            }

            let left_cell_index = (row * (column_count - 1) + column - 1) as isize;
            if column - 1 >= 0
                && left_cell_index >= 0
                && (left_cell_index as usize) < cell_count
                && hf.material_indices[left_cell_index as usize] != HEIGHT_FIELD_HOLE
            {
                let r = row;
                let c = column - 1;

                let i11 = (r * column_count + c) as usize;
                let i12 = i11 + 1;
                let i21 = ((r + 1) * column_count + c) as usize;
                let i22 = i21 + 1;

                debug_assert!(i12 == index11);
                debug_assert!(i22 == index21);
                let _ = i11;

                let h12 = heights[i12];
                let h21 = heights[i21];
                let h22 = heights[i22];

                let x1 = c as f32;
                let x2 = (c + 1) as f32;
                let z1 = r as f32;
                let z2 = (r + 1) as f32;

                let vs = [
                    mul(scale, Vec3 {
                        x: x2,
                        y: h22,
                        z: z2,
                    }),
                    mul(scale, Vec3 {
                        x: x2,
                        y: h12,
                        z: z1,
                    }),
                    mul(scale, Vec3 {
                        x: x1,
                        y: h21,
                        z: z2,
                    }),
                ];

                let n = normalize(cross(sub(vs[1], vs[0]), sub(vs[2], vs[0])));

                let separation = plane_separation(plane1, vs[2]);
                let cos_angle = dot(plane1.normal, n);
                if separation > 0.0 || cos_angle > cos5_deg {
                    flags1 |= CONCAVE_EDGE1;
                }
                if separation < 0.0 || cos_angle > cos5_deg {
                    flags1 |= INVERSE_CONCAVE_EDGE1;
                }
            }

            let right_cell_index = (row * (column_count - 1) + column + 1) as usize;
            if column + 1 < column_count - 1
                && right_cell_index < cell_count
                && hf.material_indices[right_cell_index] != HEIGHT_FIELD_HOLE
            {
                let r = row;
                let c = column + 1;

                let i11 = (r * column_count + c) as usize;
                let i12 = i11 + 1;
                let i21 = ((r + 1) * column_count + c) as usize;

                debug_assert!(i11 == index12);
                debug_assert!(i21 == index22);
                let _ = i12;

                let h11 = heights[i11];
                let h12 = heights[i12];
                let h21 = heights[i21];

                let x1 = c as f32;
                let x2 = (c + 1) as f32;
                let z1 = r as f32;
                let z2 = (r + 1) as f32;

                let vs = [
                    mul(scale, Vec3 {
                        x: x1,
                        y: h11,
                        z: z1,
                    }),
                    mul(scale, Vec3 {
                        x: x1,
                        y: h21,
                        z: z2,
                    }),
                    mul(scale, Vec3 {
                        x: x2,
                        y: h12,
                        z: z1,
                    }),
                ];

                let n = normalize(cross(sub(vs[1], vs[0]), sub(vs[2], vs[0])));

                let separation = plane_separation(plane2, vs[2]);
                let cos_angle = dot(plane2.normal, n);
                if separation > 0.0 || cos_angle > cos5_deg {
                    flags2 |= CONCAVE_EDGE1;
                }
                if separation < 0.0 || cos_angle > cos5_deg {
                    flags2 |= INVERSE_CONCAVE_EDGE1;
                }
            }

            debug_assert!((0..=u8::MAX as i32).contains(&flags1));
            debug_assert!((0..=u8::MAX as i32).contains(&flags2));

            hf.flags[triangle_index1] = flags1 as u8;
            hf.flags[triangle_index2] = flags2 as u8;
        }
    }

    debug_assert!(triangle_index == 2 * (row_count - 1) * (column_count - 1));
}

/// Destroy a height field (no-op; Rust drops the owned buffers).
/// (b3DestroyHeightField)
pub fn destroy_height_field(_height_field: HeightFieldData) {}
