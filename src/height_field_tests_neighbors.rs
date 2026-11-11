//! Port of the neighbor lookup tests from box3d-cpp-reference/test/test_height_field.c
//! (HeightFieldNeighborNormals, NeighborNormalDispatch). Split from height_field_tests.rs to
//! keep files under the length limit.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::height_field::{
    create_height_field, get_height_field_material, get_height_field_neighbor_normal,
    get_height_field_triangle, get_height_field_triangle_count, HeightFieldData, HeightFieldDef,
    HEIGHT_FIELD_HOLE,
};
use crate::math_functions::{cross, dot, normalize, sub, Triangle, Vec3};
use crate::mesh::{create_wave_mesh, get_mesh_neighbor_normal, Mesh};
use crate::shape::{get_neighbor_normal, TriangleShape};

const BUMPY_COUNT_X: usize = 9;
const BUMPY_COUNT_Z: usize = 7;

/// A bumpy field with holes, so cells have every mix of interior, border and hole neighbors.
/// (MakeBumpyField)
fn make_bumpy_field(clockwise: bool) -> HeightFieldData {
    let mut heights = vec![0.0f32; BUMPY_COUNT_X * BUMPY_COUNT_Z];
    for i in 0..BUMPY_COUNT_Z {
        for j in 0..BUMPY_COUNT_X {
            heights[i * BUMPY_COUNT_X + j] =
                (0.9 * i as f32).sin() * (0.7 * j as f32).cos() + 0.1 * ((i * j) % 3) as f32;
        }
    }

    let mut materials = vec![0u8; (BUMPY_COUNT_X - 1) * (BUMPY_COUNT_Z - 1)];
    materials[9] = HEIGHT_FIELD_HOLE;
    materials[20] = HEIGHT_FIELD_HOLE;
    materials[47] = HEIGHT_FIELD_HOLE;

    create_height_field(&HeightFieldDef {
        heights,
        material_indices: materials,
        scale: Vec3 {
            x: 1.5,
            y: 0.8,
            z: 0.5,
        },
        count_x: BUMPY_COUNT_X as i32,
        count_z: BUMPY_COUNT_Z as i32,
        global_minimum_height: -2.0,
        global_maximum_height: 2.0,
        clockwise_winding: clockwise,
    })
}

/// (IsHole)
fn is_hole(hf: &HeightFieldData, triangle_index: i32) -> bool {
    get_height_field_material(hf, triangle_index) == HEIGHT_FIELD_HOLE as i32
}

/// (TriangleNormal)
fn triangle_normal(triangle: Triangle) -> Vec3 {
    let e1 = sub(triangle.vertices[1], triangle.vertices[0]);
    let e2 = sub(triangle.vertices[2], triangle.vertices[0]);
    normalize(cross(e1, e2))
}

/// Neighbors found by brute force over shared grid points must match the grid walk.
/// (HeightFieldNeighborNormals)
#[test]
fn height_field_neighbor_normals() {
    for pass in 0..2 {
        let hf = make_bumpy_field(pass == 1);
        let triangle_count = get_height_field_triangle_count(&hf);
        let mut interior_count = 0;
        let mut boundary_count = 0;

        for i in 0..triangle_count {
            if is_hole(&hf, i) {
                continue;
            }

            let triangle = get_height_field_triangle(&hf, i);
            let indices = [triangle.i1, triangle.i2, triangle.i3];

            for k in 0..3 {
                let a = indices[k];
                let b = indices[(k + 1) % 3];

                let mut neighbor = None;
                for j in 0..triangle_count {
                    if neighbor.is_some() {
                        break;
                    }

                    if j == i || is_hole(&hf, j) {
                        continue;
                    }

                    let other = get_height_field_triangle(&hf, j);
                    let other_indices = [other.i1, other.i2, other.i3];
                    for m in 0..3 {
                        if other_indices[m] == b && other_indices[(m + 1) % 3] == a {
                            neighbor = Some(j);
                        }
                    }
                }

                let normal = get_height_field_neighbor_normal(&hf, i, k as i32);
                assert_eq!(normal.is_some(), neighbor.is_some());

                let (Some(normal), Some(neighbor)) = (normal, neighbor) else {
                    boundary_count += 1;
                    continue;
                };

                interior_count += 1;
                let expected = triangle_normal(get_height_field_triangle(&hf, neighbor));
                assert!(dot(normal, expected) > 0.99999);
            }
        }

        assert!(interior_count > 0);
        assert!(boundary_count > 0);
    }
}

/// The shape level lookup dispatches to the mesh and height field versions.
/// (NeighborNormalDispatch)
#[test]
fn neighbor_normal_dispatch() {
    let hf = make_bumpy_field(false);
    let height_shape = TriangleShape::HeightField(&hf);

    let normal1 = get_neighbor_normal(&height_shape, 1, 1).expect("height field neighbor");
    let normal2 = get_height_field_neighbor_normal(&hf, 1, 1).expect("height field neighbor");
    assert!(normal1.x == normal2.x && normal1.y == normal2.y && normal1.z == normal2.z);

    let data = create_wave_mesh(4, 4, 1.0, 0.5, 0.2, 0.15).unwrap();
    let mesh = Mesh::new(
        &data,
        Vec3 {
            x: 2.0,
            y: 0.5,
            z: -1.0,
        },
    );
    let mesh_shape = TriangleShape::Mesh(mesh);

    let found1 = get_neighbor_normal(&mesh_shape, 5, 1);
    let found2 = get_mesh_neighbor_normal(&mesh, 5, 1);
    let (Some(normal1), Some(normal2)) = (found1, found2) else {
        panic!("mesh neighbors not found");
    };
    assert!(normal1.x == normal2.x && normal1.y == normal2.y && normal1.z == normal2.z);
}
