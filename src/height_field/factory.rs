//! Height field factory helpers: grid, wave, dump, load.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::create::create_height_field;
use super::types::{HeightFieldData, HeightFieldDef, HEIGHT_FIELD_HOLE};
use crate::math_functions::{compute_cos_sin, Vec3, PI};
use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::path::Path;

/// Create a flat height field grid. (b3CreateGrid)
pub fn create_grid(
    row_count: i32,
    column_count: i32,
    scale: Vec3,
    make_holes: bool,
) -> HeightFieldData {
    let height_count = (row_count * column_count) as usize;
    let heights = vec![0.0f32; height_count];

    let cell_count = ((row_count - 1) * (column_count - 1)) as usize;
    let mut material_indices = vec![0u8; cell_count];

    for i in 0..row_count - 1 {
        for j in 0..column_count - 1 {
            let k = (i * (column_count - 1) + j) as usize;
            if make_holes && k > 0 && k % 16 == 0 {
                material_indices[k] = HEIGHT_FIELD_HOLE;
            }
        }
    }

    let def = HeightFieldDef {
        heights,
        material_indices,
        scale,
        count_x: column_count,
        count_z: row_count,
        global_minimum_height: -256.0,
        global_maximum_height: 256.0,
        clockwise_winding: false,
    };

    create_height_field(&def)
}

/// Create a sinusoidal wave height field. (b3CreateWave)
///
/// Uses Box3D's deterministic `b3ComputeCosSin` for heights, matching the C
/// reference (which switched off `sinf` for cross-platform determinism).
pub fn create_wave(
    row_count: i32,
    column_count: i32,
    scale: Vec3,
    row_frequency: f32,
    column_frequency: f32,
    make_holes: bool,
) -> HeightFieldData {
    let height_count = (row_count * column_count) as usize;
    let mut heights = vec![0.0f32; height_count];

    let omega_z = 2.0 * PI * row_frequency;
    let omega_x = 2.0 * PI * column_frequency;

    for i in 0..row_count {
        let row_height = compute_cos_sin(omega_z * (i as f32)).sine;
        for j in 0..column_count {
            let k = (i * column_count + j) as usize;
            let column_height = compute_cos_sin(omega_x * (j as f32)).sine;
            heights[k] = row_height * column_height;
        }
    }

    let cell_count = ((row_count - 1) * (column_count - 1)) as usize;
    let mut material_indices = vec![0u8; cell_count];

    for i in 0..row_count - 1 {
        for j in 0..column_count - 1 {
            let k = (i * (column_count - 1) + j) as usize;
            if make_holes && k > 0 && k % 16 == 0 {
                material_indices[k] = HEIGHT_FIELD_HOLE;
            }
        }
    }

    let def = HeightFieldDef {
        heights,
        material_indices,
        scale,
        count_x: column_count,
        count_z: row_count,
        global_minimum_height: -256.0,
        global_maximum_height: 256.0,
        clockwise_winding: false,
    };

    create_height_field(&def)
}

/// Dump height field definition to a text file. (b3DumpHeightData)
pub fn dump_height_data(data: &HeightFieldDef, file_name: impl AsRef<Path>) {
    let Ok(mut file) = File::create(file_name) else {
        return;
    };

    let _ = writeln!(file, "{} {}", data.count_x, data.count_z);
    let _ = writeln!(
        file,
        "{:.9} {:.9} {:.9}",
        data.scale.x, data.scale.y, data.scale.z
    );
    let _ = writeln!(
        file,
        "{:.9} {:.9}",
        data.global_minimum_height, data.global_maximum_height
    );
    let _ = writeln!(file, "{}", i32::from(data.clockwise_winding));

    let height_count = (data.count_x * data.count_z) as usize;
    for i in 0..height_count {
        let _ = writeln!(file, "{:.9}", data.heights[i]);
    }

    let material_count = ((data.count_x - 1) * (data.count_z - 1)) as usize;
    for i in 0..material_count {
        let _ = writeln!(file, "{}", data.material_indices[i]);
    }
}

/// Load a height field from a text file written by [`dump_height_data`].
/// (b3LoadHeightField)
pub fn load_height_field(file_name: impl AsRef<Path>) -> Option<HeightFieldData> {
    let file = File::open(file_name).ok()?;
    let mut lines = BufReader::new(file).lines().map_while(Result::ok);

    let dim_line = lines.next()?;
    let mut dim_parts = dim_line.split_whitespace();
    let count_x: i32 = dim_parts.next()?.parse().ok()?;
    let count_z: i32 = dim_parts.next()?.parse().ok()?;

    let scale_line = lines.next()?;
    let mut scale_parts = scale_line.split_whitespace();
    let scale = Vec3 {
        x: scale_parts.next()?.parse().ok()?,
        y: scale_parts.next()?.parse().ok()?,
        z: scale_parts.next()?.parse().ok()?,
    };

    let bounds_line = lines.next()?;
    let mut bounds_parts = bounds_line.split_whitespace();
    let global_minimum_height: f32 = bounds_parts.next()?.parse().ok()?;
    let global_maximum_height: f32 = bounds_parts.next()?.parse().ok()?;

    let clockwise_line = lines.next()?;
    let clockwise: i32 = clockwise_line.trim().parse().ok()?;
    let clockwise_winding = clockwise != 0;

    let height_count = (count_x * count_z) as usize;
    let mut heights = Vec::with_capacity(height_count);
    for _ in 0..height_count {
        let line = lines.next()?;
        heights.push(line.trim().parse().ok()?);
    }

    let material_count = ((count_x - 1) * (count_z - 1)) as usize;
    let mut material_indices = Vec::with_capacity(material_count);
    for _ in 0..material_count {
        let line = lines.next()?;
        let material_index: i32 = line.trim().parse().ok()?;
        material_indices.push(material_index as u8);
    }

    let def = HeightFieldDef {
        heights,
        material_indices,
        scale,
        count_x,
        count_z,
        global_minimum_height,
        global_maximum_height,
        clockwise_winding,
    };

    Some(create_height_field(&def))
}
