//! Port of the edge identification tests from box3d-cpp-reference/test/test_mesh.c
//! (MeshEdgeAngles, MeshVertexRing), plus blob checks against the scalar C build of 38f2ae1.
//! Split from mesh_tests.rs to keep files under the length limit.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::core::hash64_non_zero;
use crate::height_field::{
    create_height_field, HeightFieldData, HeightFieldDef, HEIGHT_FIELD_HOLE,
};
use crate::math_functions::{
    abs, add, cross, dot, make_normal_from_points, max_float, min_float, mul_add, normalize, sub,
    Vec3, PI, VEC3_ONE, VEC3_ZERO,
};
use crate::mesh::{
    create_box_mesh, create_grid_mesh, create_mesh, create_torus_mesh, create_wave_mesh,
    get_mesh_edge_angles, get_mesh_neighbor_normal, get_mesh_triangle, is_in_mesh_vertex_ring,
    Mesh, MeshData, MeshDef, BOUNDARY_EDGE_ANGLE, MESH_DATA_SIZE,
};
use crate::mesh_tests::{make_valley, make_valley_def};

fn v(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

/// Brute force search for the triangle across edge a-b, walked b to a with consistent winding.
/// Returns the triangle and its edge index. (FindNeighbor)
fn find_neighbor(mesh: &Mesh<'_>, triangle_index: i32, a: i32, b: i32) -> Option<(i32, i32)> {
    for i in 0..mesh.data.triangle_count {
        if i == triangle_index {
            continue;
        }

        let triangle = get_mesh_triangle(mesh, i);
        let indices = [triangle.i1, triangle.i2, triangle.i3];
        for j in 0..3 {
            if indices[j] == b && indices[(j + 1) % 3] == a {
                return Some((i, j as i32));
            }
        }
    }

    None
}

/// Both sides of an edge store the same angle. Returns the boundary edge count, or `None` if an
/// edge is asymmetric. (EdgeAnglesAreSymmetric)
fn edge_angles_are_symmetric(data: &MeshData) -> Option<i32> {
    let mesh = Mesh::new(data, VEC3_ONE);
    let edge_angles = get_mesh_edge_angles(data).expect("edge angles");
    let mut boundary_count = 0;

    for i in 0..data.triangle_count {
        let triangle = get_mesh_triangle(&mesh, i);
        let indices = [triangle.i1, triangle.i2, triangle.i3];

        for k in 0..3 {
            let angle = edge_angles[(3 * i) as usize + k];
            if angle == BOUNDARY_EDGE_ANGLE {
                boundary_count += 1;
                continue;
            }

            let (neighbor, edge_index) = find_neighbor(&mesh, i, indices[k], indices[(k + 1) % 3])?;
            if edge_angles[(3 * neighbor + edge_index) as usize] != angle {
                return None;
            }
        }
    }

    Some(boundary_count)
}

/// The rebuilt neighbor normal must match the real one under any scale. A non-uniform scale
/// stretches the quantization error by up to the ratio of its largest and smallest components.
/// (NeighborNormalsMatch)
fn neighbor_normals_match(data: &MeshData, scale: Vec3) -> bool {
    let mesh = Mesh::new(data, scale);

    let s = abs(scale);
    let spread = max_float(s.x, max_float(s.y, s.z)) / min_float(s.x, min_float(s.y, s.z));
    let tolerance = 0.8 * spread * PI / 180.0;
    let cos_tolerance = tolerance.cos();

    for i in 0..data.triangle_count {
        let triangle = get_mesh_triangle(&mesh, i);
        let indices = [triangle.i1, triangle.i2, triangle.i3];

        for k in 0..3 {
            let neighbor = find_neighbor(&mesh, i, indices[k], indices[(k + 1) % 3]);

            let rebuilt = get_mesh_neighbor_normal(&mesh, i, k as i32);
            if rebuilt.is_some() != neighbor.is_some() {
                return false;
            }

            let (Some(rebuilt), Some((neighbor, _))) = (rebuilt, neighbor) else {
                continue;
            };

            let other = get_mesh_triangle(&mesh, neighbor);
            let e1 = sub(other.vertices[1], other.vertices[0]);
            let e2 = sub(other.vertices[2], other.vertices[0]);
            let actual = normalize(cross(e1, e2));
            if dot(rebuilt, actual) < cos_tolerance {
                return false;
            }
        }
    }

    true
}

/// Positive angles are convex, negative are concave. A right angle sits on a rounding boundary.
/// (HasRightAngle)
fn has_right_angle(data: &MeshData, sign: i32) -> bool {
    let edge_angles = get_mesh_edge_angles(data).expect("edge angles");
    edge_angles
        .iter()
        .take(3 * data.triangle_count as usize)
        .any(|&a| {
            let angle = sign * a as i32;
            angle == 63 || angle == 64
        })
}

const SCALES: [Vec3; 6] = [
    Vec3 {
        x: 1.0,
        y: 1.0,
        z: 1.0,
    },
    Vec3 {
        x: 3.0,
        y: 3.0,
        z: 3.0,
    },
    Vec3 {
        x: -1.0,
        y: 1.0,
        z: 1.0,
    },
    Vec3 {
        x: 2.0,
        y: 0.5,
        z: 1.5,
    },
    Vec3 {
        x: 0.5,
        y: -2.0,
        z: 1.0,
    },
    Vec3 {
        x: -1.0,
        y: -1.0,
        z: -1.0,
    },
];

/// (MeshEdgeAngles)
#[test]
fn mesh_edge_angles() {
    {
        let grid = create_grid_mesh(4, 4, 1.0, 0, true).unwrap();
        let edge_angles = get_mesh_edge_angles(&grid).expect("grid with edge identification");

        assert_eq!(edge_angles_are_symmetric(&grid), Some(16));

        for &angle in edge_angles.iter().take(3 * grid.triangle_count as usize) {
            assert!(angle == 0 || angle == BOUNDARY_EDGE_ANGLE);
        }
    }

    {
        let grid = create_grid_mesh(4, 4, 1.0, 0, false).unwrap();
        assert!(get_mesh_edge_angles(&grid).is_none());
    }

    let (vertices, indices) = make_valley();
    let def = make_valley_def(&vertices, &indices);

    let meshes = [
        create_mesh(&def, None),
        create_box_mesh(VEC3_ZERO, v(1.0, 2.0, 3.0), true),
        create_wave_mesh(8, 8, 1.0, 0.5, 0.2, 0.15),
        create_torus_mesh(8, 6, 2.0, 0.5),
    ];

    let valley = meshes[0].as_ref().unwrap();
    let box_mesh = meshes[1].as_ref().unwrap();
    assert!(has_right_angle(valley, -1));
    assert!(has_right_angle(box_mesh, 1));

    assert_eq!(edge_angles_are_symmetric(box_mesh), Some(0));

    for mesh in &meshes {
        let mesh = mesh.as_ref().unwrap();
        assert!(edge_angles_are_symmetric(mesh).is_some());

        for scale in SCALES {
            assert!(neighbor_normals_match(mesh, scale));
        }
    }

    {
        // Three triangles on edge 0-1 are non-manifold. The last two share edge 5-6 walked the
        // same way, which is inconsistent winding. Neither gets an angle.
        let fin_vertices = vec![
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(0.0, 0.0, 1.0),
            v(0.0, 0.0, -1.0),
            v(0.0, 1.0, 0.0),
            v(5.0, 0.0, 0.0),
            v(5.0, 0.0, 1.0),
            v(5.0, 1.0, 0.0),
        ];

        let fin_indices = vec![0, 1, 2, 1, 0, 3, 0, 1, 4, 5, 6, 7, 5, 6, 2];

        let fin_def = MeshDef {
            vertices: fin_vertices,
            indices: fin_indices,
            identify_edges: true,
            ..Default::default()
        };
        let fin = create_mesh(&fin_def, None).expect("fin mesh");

        assert_eq!(edge_angles_are_symmetric(&fin), Some(15));
    }
}

/// Exact vertex cone from every triangle at the vertex. Returns the largest lean of the normal
/// past an edge at the vertex or past the cliff below a boundary edge. (ExactVertexLean)
fn exact_vertex_lean(mesh: &Mesh<'_>, vertex: i32, normal: Vec3) -> f32 {
    let mut lean = -f32::MAX;
    for i in 0..mesh.data.triangle_count {
        let triangle = get_mesh_triangle(mesh, i);
        let indices = [triangle.i1, triangle.i2, triangle.i3];
        let Some(k) = indices.iter().position(|&index| index == vertex) else {
            continue;
        };

        let next = (k + 1) % 3;
        let previous = (k + 2) % 3;
        let edges = [
            sub(triangle.vertices[next], triangle.vertices[k]),
            sub(triangle.vertices[previous], triangle.vertices[k]),
        ];
        let edge_indices = [k as i32, previous as i32];
        let face_normal = make_normal_from_points(
            triangle.vertices[0],
            triangle.vertices[1],
            triangle.vertices[2],
        );

        for j in 0..2 {
            lean = max_float(lean, dot(normal, normalize(edges[j])));

            if get_mesh_neighbor_normal(mesh, i, edge_indices[j]).is_none() {
                lean = max_float(lean, -dot(normal, face_normal));
            }
        }
    }

    lean
}

/// (NextRandom)
fn next_random(state: &mut u32) -> u32 {
    *state = state.wrapping_mul(1664525).wrapping_add(1013904223);
    *state
}

/// (RandomFloat)
fn random_float(state: &mut u32) -> f32 {
    (next_random(state) >> 8) as f32 / (1 << 24) as f32 * 2.0 - 1.0
}

/// The quantized ring must admit normals well inside the exact cone and reject normals well
/// outside. The band between is quantization noise. A non-uniform scale stretches it. Returns
/// the inside and outside sample counts, or `None` on a mismatch. (VertexRingMatches)
fn vertex_ring_matches(data: &MeshData, scale: Vec3) -> Option<(i32, i32)> {
    let mesh = Mesh::new(data, scale);
    let mut inside_count = 0;
    let mut outside_count = 0;

    let s = abs(scale);
    let spread = max_float(s.x, max_float(s.y, s.z)) / min_float(s.x, min_float(s.y, s.z));
    let tolerance = 0.0175;
    let margin = 0.025 * spread;
    let mut state = 12345u32;

    for i in 0..data.triangle_count {
        let triangle = get_mesh_triangle(&mesh, i);
        let indices = [triangle.i1, triangle.i2, triangle.i3];
        let face_normal = make_normal_from_points(
            triangle.vertices[0],
            triangle.vertices[1],
            triangle.vertices[2],
        );

        for k in 0..3 {
            let mut samples = Vec::with_capacity(24);
            samples.push(face_normal);

            // Fan normals and their blends land near the cone
            let mut j = 0;
            while j < data.triangle_count && samples.len() < 12 {
                let other = get_mesh_triangle(&mesh, j);
                if j == i
                    || (other.i1 != indices[k] && other.i2 != indices[k] && other.i3 != indices[k])
                {
                    j += 1;
                    continue;
                }

                let other_normal = make_normal_from_points(
                    other.vertices[0],
                    other.vertices[1],
                    other.vertices[2],
                );
                samples.push(other_normal);
                samples.push(normalize(add(face_normal, other_normal)));
                j += 1;
            }

            while samples.len() < 24 {
                let x = random_float(&mut state);
                let y = random_float(&mut state);
                let z = random_float(&mut state);
                let tilt = v(x, y, z);
                let amount = if samples.len() % 2 == 0 { 0.1 } else { 1.0 };
                samples.push(normalize(mul_add(face_normal, amount, tilt)));
            }

            for &normal in &samples {
                let lean = exact_vertex_lean(&mesh, indices[k], normal);
                let admit = is_in_mesh_vertex_ring(&mesh, i, k as i32, normal, tolerance);

                if lean <= tolerance - margin {
                    inside_count += 1;
                    if !admit {
                        return None;
                    }
                } else if lean > tolerance + margin {
                    outside_count += 1;
                    if admit {
                        return None;
                    }
                }
            }
        }
    }

    Some((inside_count, outside_count))
}

/// (MeshVertexRing)
#[test]
fn mesh_vertex_ring() {
    let (vertices, indices) = make_valley();
    let def = make_valley_def(&vertices, &indices);

    let meshes = [
        create_mesh(&def, None),
        create_box_mesh(VEC3_ZERO, v(1.0, 2.0, 3.0), true),
        create_wave_mesh(8, 8, 1.0, 0.5, 0.2, 0.15),
        create_torus_mesh(8, 6, 2.0, 0.5),
        create_grid_mesh(4, 4, 1.0, 0, true),
    ];

    for mesh in &meshes {
        let mesh = mesh.as_ref().unwrap();

        for scale in SCALES {
            let (inside_count, outside_count) =
                vertex_ring_matches(mesh, scale).expect("vertex ring matches the exact cone");
            assert!(inside_count > 0 && outside_count > 0);
        }
    }

    // Without edge data every normal passes
    let bare = create_grid_mesh(4, 4, 1.0, 0, false).unwrap();
    let mesh = Mesh::new(&bare, VEC3_ONE);
    assert!(is_in_mesh_vertex_ring(
        &mesh,
        0,
        0,
        v(1.0, 0.0, 0.0),
        0.0175
    ));
}

/// A bumpy field with holes and a deterministic height pattern, for the blob check below.
fn blob_check_field(clockwise: bool) -> HeightFieldData {
    let mut heights = vec![0.0f32; 9 * 7];
    for i in 0..7 {
        for j in 0..9 {
            heights[i * 9 + j] =
                0.37 * ((i * 7 + j * 3) % 11) as f32 - 0.9 + 0.1 * ((i * j) % 3) as f32;
        }
    }

    let mut materials = vec![0u8; 8 * 6];
    materials[9] = HEIGHT_FIELD_HOLE;
    materials[20] = HEIGHT_FIELD_HOLE;
    materials[47] = HEIGHT_FIELD_HOLE;

    create_height_field(&HeightFieldDef {
        heights,
        material_indices: materials,
        scale: v(1.5, 0.8, 0.5),
        count_x: 9,
        count_z: 7,
        global_minimum_height: -2.0,
        global_maximum_height: 2.0,
        clockwise_winding: clockwise,
    })
}

/// The mesh and height-field blobs, with and without edge identification, match the scalar C
/// build of 38f2ae1 (clang, float and double precision alike): byte count, the section offsets,
/// and b3Hash64NonZero of the blob. The full blobs were also compared byte for byte. Covers the
/// edge angles, vertex flags, and vertex rings of the mesh and the height-field vertex bits.
#[test]
fn mesh_blobs_match_c() {
    assert_eq!(MESH_DATA_SIZE, 104);

    let check = |mesh: &MeshData, expected: (i32, i32, i32, i32, u64)| {
        let bytes = mesh.to_bytes();
        assert_eq!(bytes.len(), mesh.byte_count as usize);
        assert_eq!(hash64_non_zero(&mesh.to_bytes_with_hash(0)), mesh.hash);
        assert_eq!(
            (
                mesh.byte_count,
                mesh.edge_angle_offset,
                mesh.vertex_flag_offset,
                mesh.vertex_ring_offset,
                mesh.hash,
            ),
            expected
        );
    };

    check(
        &create_grid_mesh(4, 4, 1.0, 0, true).unwrap(),
        (1784, 1336, 1432, 1464, 0x1ce3_9d6b_b74a_76db),
    );
    check(
        &create_grid_mesh(4, 4, 1.0, 0, false).unwrap(),
        (1336, 0, 0, 0, 0xf5cf_bd4d_6814_59ee),
    );
    check(
        &create_box_mesh(VEC3_ZERO, v(1.0, 2.0, 3.0), true).unwrap(),
        (840, 664, 704, 712, 0x631c_5b3f_3ecc_7558),
    );
    check(
        &create_wave_mesh(8, 8, 1.0, 0.5, 0.2, 0.15).unwrap(),
        (6816, 4888, 5272, 5360, 0xa6a3_ba90_3712_13d6),
    );
    check(
        &create_torus_mesh(8, 6, 2.0, 0.5).unwrap(),
        (4992, 3784, 4072, 4120, 0xd059_da2a_8c02_acee),
    );
    check(
        &create_wave_mesh(40, 40, 0.5, 0.3, 0.2, 0.3).unwrap(),
        (159840, 118296, 127896, 129584, 0x29fd_db5b_b751_7c4e),
    );

    let field0 = blob_check_field(false);
    assert_eq!(
        (field0.byte_count, field0.vertex_flag_offset, field0.hash),
        (376, 368, 0xc525_9746_2892_3e2e)
    );
    let field1 = blob_check_field(true);
    assert_eq!(
        (field1.byte_count, field1.vertex_flag_offset, field1.hash),
        (376, 368, 0x4703_f00b_39d2_700a)
    );
}
