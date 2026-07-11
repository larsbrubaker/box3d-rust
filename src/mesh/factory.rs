//! Mesh factory helpers: grid, wave, torus, box, hollow box, platform.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::create::create_mesh;
use super::types::{MeshData, MeshDef};
use crate::math_functions::{add, cos, sin, Vec3, PI, TWO_PI};

/// Create a grid mesh along the x and z axes. (b3CreateGridMesh)
pub fn create_grid_mesh(
    x_count: i32,
    z_count: i32,
    cell_width: f32,
    material_count: i32,
    identify_edges: bool,
) -> Option<MeshData> {
    debug_assert!((0..=u8::MAX as i32).contains(&material_count));

    let vertex_count = (x_count + 1) * (z_count + 1);
    let mut vertices = vec![Vec3::default(); vertex_count as usize];
    let mut index = 0usize;

    let x_width = cell_width * (x_count as f32);
    let z_width = cell_width * (z_count as f32);

    let mut x = -0.5 * x_width;
    for _ix in 0..=x_count {
        let mut z = -0.5 * z_width;
        for _iz in 0..=z_count {
            vertices[index] = Vec3 { x, y: 0.0, z };
            z += cell_width;
            index += 1;
        }
        x += cell_width;
    }
    debug_assert!(index == vertex_count as usize);

    let triangle_count = 2 * x_count * z_count;
    let mut indices = vec![0i32; (3 * triangle_count) as usize];
    let mut material_indices = vec![0u8; triangle_count as usize];

    let mut material_index = 0i32;
    index = 0;
    for ix in 0..x_count {
        for iz in 0..z_count {
            let index1 = iz + (z_count + 1) * ix;
            let index2 = index1 + 1;
            let index3 = index2 + (z_count + 1);
            let index4 = index3 - 1;

            debug_assert!(index1 < vertex_count);
            debug_assert!(index2 < vertex_count);
            debug_assert!(index3 < vertex_count);
            debug_assert!(index4 < vertex_count);

            indices[index] = index1;
            indices[index + 1] = index2;
            indices[index + 2] = index3;
            indices[index + 3] = index3;
            indices[index + 4] = index4;
            indices[index + 5] = index1;

            if material_count > 0 {
                material_indices[(2 * material_index) as usize] =
                    (material_index % material_count) as u8;
                material_indices[(2 * material_index + 1) as usize] =
                    (material_index % material_count) as u8;
            }

            material_index += 1;
            index += 6;
        }
    }
    debug_assert!(index == (3 * triangle_count) as usize);

    let def = MeshDef {
        vertices,
        indices,
        material_indices: if material_count > 0 {
            material_indices
        } else {
            Vec::new()
        },
        use_median_split: true,
        identify_edges,
        ..Default::default()
    };

    create_mesh(&def, None)
}

/// Create a wave mesh along the x and z axes. (b3CreateWaveMesh)
///
/// Uses `f32::sin` (C `sinf`) for heights — not the deterministic `b3Sin`.
pub fn create_wave_mesh(
    x_count: i32,
    z_count: i32,
    cell_width: f32,
    amplitude: f32,
    row_frequency: f32,
    column_frequency: f32,
) -> Option<MeshData> {
    let vertex_count = (x_count + 1) * (z_count + 1);
    let mut vertices = vec![Vec3::default(); vertex_count as usize];
    let mut index = 0usize;

    let x_width = cell_width * (x_count as f32);
    let z_width = cell_width * (z_count as f32);

    let omega_z = 2.0 * PI * row_frequency * cell_width;
    let omega_x = 2.0 * PI * column_frequency * cell_width;

    let mut x = -0.5 * x_width;
    for ix in 0..=x_count {
        let row_height = (omega_x * (ix as f32)).sin();
        let mut z = -0.5 * z_width;
        for iz in 0..=z_count {
            let column_height = (omega_z * (iz as f32)).sin();
            let y = amplitude * row_height * column_height;
            vertices[index] = Vec3 { x, y, z };
            z += cell_width;
            index += 1;
        }
        x += cell_width;
    }
    debug_assert!(index == vertex_count as usize);

    let triangle_count = 2 * x_count * z_count;
    let mut indices = vec![0i32; (3 * triangle_count) as usize];
    index = 0;
    for ix in 0..x_count {
        for iz in 0..z_count {
            let index1 = iz + (z_count + 1) * ix;
            let index2 = index1 + 1;
            let index3 = index2 + (z_count + 1);
            let index4 = index3 - 1;

            indices[index] = index1;
            indices[index + 1] = index2;
            indices[index + 2] = index3;
            indices[index + 3] = index3;
            indices[index + 4] = index4;
            indices[index + 5] = index1;
            index += 6;
        }
    }
    debug_assert!(index == (3 * triangle_count) as usize);

    let def = MeshDef {
        vertices,
        indices,
        use_median_split: true,
        identify_edges: true,
        ..Default::default()
    };

    create_mesh(&def, None)
}

/// Create a torus mesh. (b3CreateTorusMesh)
pub fn create_torus_mesh(
    radial_resolution: i32,
    tubular_resolution: i32,
    radius: f32,
    thickness: f32,
) -> Option<MeshData> {
    let mut vertices = Vec::new();

    for radial_index in 0..radial_resolution {
        for tubular_index in 0..tubular_resolution {
            let u = (tubular_index as f32) / (tubular_resolution as f32) * TWO_PI;
            let v = (radial_index as f32) / (radial_resolution as f32) * TWO_PI;

            let x = (radius + thickness * cos(v)) * cos(u);
            let y = (radius + thickness * cos(v)) * sin(u);
            let z = thickness * sin(v);

            vertices.push(Vec3 { x, y, z });
        }
    }

    let mut indices = Vec::new();
    for radial_index1 in 0..radial_resolution {
        let radial_index2 = (radial_index1 + 1) % radial_resolution;
        for tubular_index1 in 0..tubular_resolution {
            let tubular_index2 = (tubular_index1 + 1) % tubular_resolution;
            let index1 = radial_index1 * tubular_resolution + tubular_index1;
            let index2 = radial_index1 * tubular_resolution + tubular_index2;
            let index3 = radial_index2 * tubular_resolution + tubular_index2;
            let index4 = radial_index2 * tubular_resolution + tubular_index1;

            indices.push(index1);
            indices.push(index2);
            indices.push(index3);
            indices.push(index3);
            indices.push(index4);
            indices.push(index1);
        }
    }

    let def = MeshDef {
        vertices,
        indices,
        use_median_split: false,
        identify_edges: true,
        ..Default::default()
    };

    create_mesh(&def, None)
}

/// Create a box mesh. (b3CreateBoxMesh)
pub fn create_box_mesh(center: Vec3, extent: Vec3, identify_edges: bool) -> Option<MeshData> {
    let x = extent.x;
    let y = extent.y;
    let z = extent.z;
    let mut vertices = [
        Vec3 { x, y, z },
        Vec3 { x: -x, y, z },
        Vec3 { x: -x, y: -y, z },
        Vec3 { x, y: -y, z },
        Vec3 { x, y, z: -z },
        Vec3 { x: -x, y, z: -z },
        Vec3 {
            x: -x,
            y: -y,
            z: -z,
        },
        Vec3 { x, y: -y, z: -z },
    ];

    for v in &mut vertices {
        *v = add(*v, center);
    }

    let indices = [
        0, 1, 3, 1, 2, 3, // front
        0, 4, 1, 1, 4, 5, // top
        0, 3, 7, 4, 0, 7, // right
        4, 7, 5, 6, 5, 7, // back
        1, 5, 2, 6, 2, 5, // left
        3, 2, 7, 6, 7, 2, // bottom
    ];

    let def = MeshDef {
        vertices: vertices.to_vec(),
        indices: indices.to_vec(),
        use_median_split: false,
        identify_edges,
        ..Default::default()
    };

    create_mesh(&def, None)
}

/// Create a hollow (inward-facing) box mesh. (b3CreateHollowBoxMesh)
pub fn create_hollow_box_mesh(center: Vec3, extent: Vec3) -> Option<MeshData> {
    let x = extent.x;
    let y = extent.y;
    let z = extent.z;
    let mut vertices = [
        Vec3 { x, y, z },
        Vec3 { x: -x, y, z },
        Vec3 { x: -x, y: -y, z },
        Vec3 { x, y: -y, z },
        Vec3 { x, y, z: -z },
        Vec3 { x: -x, y, z: -z },
        Vec3 {
            x: -x,
            y: -y,
            z: -z,
        },
        Vec3 { x, y: -y, z: -z },
    ];

    for v in &mut vertices {
        *v = add(*v, center);
    }

    let indices = [
        3, 1, 0, 3, 2, 1, // front
        1, 4, 0, 5, 4, 1, // top
        7, 3, 0, 7, 0, 4, // right
        5, 7, 4, 7, 5, 6, // back
        2, 5, 1, 5, 2, 6, // left
        7, 2, 3, 2, 7, 6, // bottom
    ];

    let def = MeshDef {
        vertices: vertices.to_vec(),
        indices: indices.to_vec(),
        use_median_split: false,
        identify_edges: true,
        ..Default::default()
    };

    create_mesh(&def, None)
}

/// Create a platform mesh (truncated pyramid). (b3CreatePlatformMesh)
pub fn create_platform_mesh(
    center: Vec3,
    height: f32,
    top_width: f32,
    bottom_width: f32,
) -> Option<MeshData> {
    let hb = 0.5 * bottom_width;
    let ht = 0.5 * top_width;
    let hy = 0.5 * height;
    let mut vertices = [
        Vec3 {
            x: ht,
            y: hy,
            z: ht,
        },
        Vec3 {
            x: -ht,
            y: hy,
            z: ht,
        },
        Vec3 {
            x: -hb,
            y: -hy,
            z: hb,
        },
        Vec3 {
            x: hb,
            y: -hy,
            z: hb,
        },
        Vec3 {
            x: ht,
            y: hy,
            z: -ht,
        },
        Vec3 {
            x: -ht,
            y: hy,
            z: -ht,
        },
        Vec3 {
            x: -hb,
            y: -hy,
            z: -hb,
        },
        Vec3 {
            x: hb,
            y: -hy,
            z: -hb,
        },
    ];

    for v in &mut vertices {
        *v = add(*v, center);
    }

    let indices = [
        0, 1, 3, 1, 2, 3, // front
        0, 4, 1, 1, 4, 5, // top
        0, 3, 7, 4, 0, 7, // right
        4, 7, 5, 6, 5, 7, // back
        1, 5, 2, 6, 2, 5, // left
        3, 2, 7, 6, 7, 2, // bottom
    ];

    let def = MeshDef {
        vertices: vertices.to_vec(),
        indices: indices.to_vec(),
        use_median_split: true,
        identify_edges: true,
        ..Default::default()
    };

    create_mesh(&def, None)
}
