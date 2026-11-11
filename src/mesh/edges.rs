//! Mesh edge identification: triangle adjacency, concave edge flags, quantized neighbor
//! angles, vertex flags, and the neighbor normal lookups used for ghost collision handling.
//!
//! Port of `b3IdentifyEdges`, `b3QuantizeEdgeAngle`, `b3GetQuantizedCosSin`,
//! `b3GetMeshNeighborNormal`, `b3GetMeshVertexIndex`, and `b3GetMeshVertexFlag` from
//! `box3d-cpp-reference/src/mesh.c`.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::quantized_cos_sin::QUANTIZED_COS_SIN;
use super::types::{
    get_mesh_edge_angles, get_mesh_vertex_flags, Mesh, MeshData, MeshTriangle, BOUNDARY_EDGE_ANGLE,
    CONCAVE_EDGE1, CONCAVE_EDGE2, CONCAVE_EDGE3, CONCAVE_VERTEX, FLAT_VERTEX,
    INVERSE_CONCAVE_EDGE1, INVERSE_CONCAVE_EDGE2, INVERSE_CONCAVE_EDGE3,
};
use crate::core::NULL_INDEX;
use crate::math_functions::{
    abs, add, clamp_int, cross, dot, max_int, min_int, mul, mul_sv, normalize, signed_volume, sub,
    CosSin, Vec3, PI,
};
use crate::simd_scalar::{atan2_w, load_w, mul_w, sqrt_w, store_w};

/// (b3MeshEdge)
struct MeshEdge {
    vertex1: i32,
    vertex2: i32,
    triangle1: i32,
    triangle2: i32,
    triangle_count: u16,

    // The index of an edge within the parent triangle: 0, 1, or 2. 0xFF is unset
    triangle_edge_index1: u8,
    triangle_edge_index2: u8,
}

/// (B3_EDGE_ANGLE_STEPS)
const EDGE_ANGLE_STEPS: f32 = 127.0;

/// (b3GetQuantizedCosSin)
fn get_quantized_cos_sin(code: i32) -> CosSin {
    debug_assert!((-254..=254).contains(&code));
    let index = min_int(if code < 0 { -code } else { code }, 254);
    let mut cs = QUANTIZED_COS_SIN[index as usize];
    if code < 0 {
        cs.sine = -cs.sine;
    }

    cs
}

/// Quantize a neighbor angle in radians to an 8-bit code in steps of pi / 127.
/// (b3QuantizeEdgeAngle)
pub(crate) fn quantize_edge_angle(radians: f32) -> i8 {
    let scaled = radians * (EDGE_ANGLE_STEPS / PI);
    let quantized = if scaled >= 0.0 {
        (scaled + 0.5) as i32
    } else {
        -((0.5 - scaled) as i32)
    };
    clamp_int(quantized, -127, 127) as i8
}

/// The normal of the triangle across an edge, rebuilt from the quantized edge angle and
/// accounting for the mesh scale. `edge_index` 0, 1, 2 is the edge from vertex 1, 2, 3 of
/// the (scaled) triangle. Returns `None` if the edge is isolated or the mesh has no edge
/// identification. (b3GetMeshNeighborNormal)
pub(crate) fn get_mesh_neighbor_normal(
    mesh: &Mesh<'_>,
    triangle_index: i32,
    edge_index: i32,
) -> Option<Vec3> {
    let data = mesh.data;
    debug_assert!(0 <= triangle_index && triangle_index < data.triangle_count);
    debug_assert!((0..3).contains(&edge_index));

    let edge_angles = get_mesh_edge_angles(data)?;

    let mut edge_index = edge_index;
    let scale = mesh.scale;
    if scale.x * scale.y * scale.z < 0.0 {
        // Reflected mesh.
        edge_index = 2 - edge_index;
    }

    let quantized = edge_angles[(3 * triangle_index + edge_index) as usize];
    if quantized == BOUNDARY_EDGE_ANGLE {
        return None;
    }

    let triangle = data.triangles[triangle_index as usize];
    let vertices = &data.vertices;
    let indices = [triangle.index1, triangle.index2, triangle.index3];

    let a = vertices[indices[edge_index as usize] as usize];
    let b = vertices[indices[((edge_index + 1) % 3) as usize] as usize];
    let c = vertices[indices[((edge_index + 2) % 3) as usize] as usize];
    let n = normalize(cross(sub(b, a), sub(c, a)));
    let outward = cross(normalize(sub(b, a)), n);

    let cs = get_quantized_cos_sin(2 * quantized as i32);
    let unscaled = add(mul_sv(cs.cosine, n), mul_sv(cs.sine, outward));

    let abs_scale = abs(scale);
    if abs_scale.y == abs_scale.x && abs_scale.z == abs_scale.x {
        // Shortcut for uniform scale. Common case.
        return Some(Vec3 {
            x: if scale.x < 0.0 {
                -unscaled.x
            } else {
                unscaled.x
            },
            y: if scale.y < 0.0 {
                -unscaled.y
            } else {
                unscaled.y
            },
            z: if scale.z < 0.0 {
                -unscaled.z
            } else {
                unscaled.z
            },
        });
    }

    // Handle non-uniform scale.
    let inv_scale = Vec3 {
        x: 1.0 / scale.x,
        y: 1.0 / scale.y,
        z: 1.0 / scale.z,
    };
    Some(normalize(mul(inv_scale, unscaled)))
}

/// The mesh vertex index of a corner of the scaled triangle. (b3GetMeshVertexIndex)
pub(super) fn get_mesh_vertex_index(
    mesh: &Mesh<'_>,
    triangle: MeshTriangle,
    vertex_index: i32,
) -> i32 {
    let mut vertex_index = vertex_index;
    let scale = mesh.scale;
    if scale.x * scale.y * scale.z < 0.0 {
        // Handle reflection.
        vertex_index = (3 - vertex_index) % 3;
    }

    let indices = [triangle.index1, triangle.index2, triangle.index3];
    indices[vertex_index as usize]
}

/// The [`FLAT_VERTEX`] / [`CONCAVE_VERTEX`] flags of a corner of the scaled triangle. A mesh
/// without edge identification reports every vertex as flat. (b3GetMeshVertexFlag)
pub(crate) fn get_mesh_vertex_flag(mesh: &Mesh<'_>, triangle_index: i32, vertex_index: i32) -> i32 {
    let data = mesh.data;
    debug_assert!(0 <= triangle_index && triangle_index < data.triangle_count);
    debug_assert!((0..3).contains(&vertex_index));

    let Some(vertex_flags) = get_mesh_vertex_flags(data) else {
        return FLAT_VERTEX as i32;
    };

    let triangle = data.triangles[triangle_index as usize];
    let index = get_mesh_vertex_index(mesh, triangle, vertex_index);
    vertex_flags[index as usize] as i32
}

/// Find triangle adjacency, mark concave edges, quantize the angle across each shared edge,
/// and classify the vertices. Writes the unit triangle normals to `normals`. The mesh must
/// have the edge angle and vertex flag sections allocated. (b3IdentifyEdges)
pub(super) fn identify_edges(mesh: &mut MeshData, normals: &mut [Vec3]) {
    debug_assert!(mesh.edge_angle_offset != 0);

    let triangle_count = mesh.triangle_count as usize;
    let edge_count = 3 * triangle_count;

    debug_assert!(mesh.edge_angles.len() == edge_count);
    for angle in mesh.edge_angles.iter_mut() {
        *angle = BOUNDARY_EDGE_ANGLE;
    }

    let mut edges: Vec<MeshEdge> = Vec::with_capacity(edge_count);

    for i in 0..triangle_count {
        let triangle = mesh.triangles[i];
        let i1 = triangle.index1;
        let i2 = triangle.index2;
        let i3 = triangle.index3;

        edges.push(MeshEdge {
            vertex1: min_int(i1, i2),
            vertex2: max_int(i1, i2),
            triangle1: i as i32,
            triangle2: NULL_INDEX,
            triangle_edge_index1: 0,
            triangle_edge_index2: 0xFF,
            triangle_count: 1,
        });

        edges.push(MeshEdge {
            vertex1: min_int(i2, i3),
            vertex2: max_int(i2, i3),
            triangle1: i as i32,
            triangle2: NULL_INDEX,
            triangle_edge_index1: 1,
            triangle_edge_index2: 0xFF,
            triangle_count: 1,
        });

        edges.push(MeshEdge {
            vertex1: min_int(i3, i1),
            vertex2: max_int(i3, i1),
            triangle1: i as i32,
            triangle2: NULL_INDEX,
            triangle_edge_index1: 2,
            triangle_edge_index2: 0xFF,
            triangle_count: 1,
        });

        let v1 = mesh.vertices[i1 as usize];
        let v2 = mesh.vertices[i2 as usize];
        let v3 = mesh.vertices[i3 as usize];

        let e1 = sub(v2, v1);
        let e2 = sub(v3, v1);
        let n = cross(e1, e2);

        normals[i] = normalize(n);
    }

    // Counting sort for edges. Faster than a hashtable.
    let vertex_count = mesh.vertex_count as usize;
    let mut edge_starts = vec![0i32; vertex_count + 1];
    let mut sorted_edges = vec![0i32; edge_count];
    let mut stamps = vec![NULL_INDEX; vertex_count];
    let mut first_edges = vec![0i32; vertex_count];

    for edge in &edges {
        debug_assert!(0 <= edge.vertex1 && (edge.vertex2 as usize) < vertex_count);
        edge_starts[edge.vertex1 as usize + 1] += 1;
    }

    for i in 0..vertex_count {
        edge_starts[i + 1] += edge_starts[i];
    }

    for (i, edge) in edges.iter().enumerate() {
        let vertex1 = edge.vertex1 as usize;
        sorted_edges[edge_starts[vertex1] as usize] = i as i32;
        edge_starts[vertex1] += 1;
    }

    // Find unique edges and assign adjacency
    let mut begin = 0usize;
    for vertex1 in 0..vertex_count {
        let end = edge_starts[vertex1] as usize;
        for &sorted in &sorted_edges[begin..end] {
            let i = sorted as usize;
            let vertex2 = edges[i].vertex2 as usize;

            // Establish twin.
            if stamps[vertex2] != vertex1 as i32 {
                stamps[vertex2] = vertex1 as i32;
                first_edges[vertex2] = i as i32;
                continue;
            }

            let other_index = first_edges[vertex2] as usize;
            debug_assert!(other_index < i);

            let triangle1 = edges[i].triangle1;
            let triangle_edge_index1 = edges[i].triangle_edge_index1;
            let base = &mut edges[other_index];
            if base.triangle_count == 1 {
                base.triangle2 = triangle1;
                base.triangle_edge_index2 = triangle_edge_index1;
            }

            base.triangle_count += 1;
        }

        begin = end;
    }

    // C pads the angle arrays to a multiple of four lanes for the wide arctangent.
    let angle_capacity = (edge_count / 2 + 3) & !3;
    let mut angle_ys = vec![0.0f32; angle_capacity];
    let mut angle_xs = vec![0.0f32; angle_capacity];
    let mut angle_length_sqs = vec![0.0f32; angle_capacity];
    let mut angle_slots = vec![0i32; 2 * angle_capacity];
    let mut angle_count = 0usize;

    let edge_flags_concave = [CONCAVE_EDGE1, CONCAVE_EDGE2, CONCAVE_EDGE3];
    let edge_flags_inverse = [
        INVERSE_CONCAVE_EDGE1,
        INVERSE_CONCAVE_EDGE2,
        INVERSE_CONCAVE_EDGE3,
    ];

    for edge in &edges {
        if edge.triangle_count != 2 {
            continue;
        }

        debug_assert!(edge.triangle_edge_index1 < 3);
        debug_assert!(edge.triangle_edge_index2 < 3);

        let triangle1 = mesh.triangles[edge.triangle1 as usize];
        let triangle2 = mesh.triangles[edge.triangle2 as usize];

        let j1 = triangle2.index1;
        let j2 = triangle2.index2;
        let j3 = triangle2.index3;

        let opposite = match edge.triangle_edge_index2 {
            0 => j3,
            1 => j1,
            2 => j2,
            _ => {
                debug_assert!(false);
                NULL_INDEX
            }
        };

        let i1 = triangle1.index1;
        let i2 = triangle1.index2;
        let i3 = triangle1.index3;

        let v1 = mesh.vertices[i1 as usize];
        let v2 = mesh.vertices[i2 as usize];
        let v3 = mesh.vertices[i3 as usize];
        let p = mesh.vertices[opposite as usize];
        let n1 = normals[edge.triangle1 as usize];
        let n2 = normals[edge.triangle2 as usize];

        // Quantize the angles between neighboring normals.
        let indices1 = [i1, i2, i3];
        let indices2 = [j1, j2, j3];
        let k1 = edge.triangle_edge_index1 as usize;
        let k2 = edge.triangle_edge_index2 as usize;
        if indices1[k1] == indices2[(k2 + 1) % 3] {
            let a = mesh.vertices[indices1[k1] as usize];
            let b = mesh.vertices[indices1[(k1 + 1) % 3] as usize];
            let edge_vector = sub(b, a);
            let outward = cross(edge_vector, n1);
            debug_assert!(angle_count < angle_capacity);
            angle_ys[angle_count] = dot(n2, outward);
            angle_xs[angle_count] = dot(n2, n1);
            angle_length_sqs[angle_count] = dot(edge_vector, edge_vector);
            angle_slots[2 * angle_count] = 3 * edge.triangle1 + k1 as i32;
            angle_slots[2 * angle_count + 1] = 3 * edge.triangle2 + k2 as i32;
            angle_count += 1;
        }

        let cos5_deg = 0.9962f32;
        let signed_vol = signed_volume(v1, v2, v3, p);
        let cos_angle = dot(n1, n2);
        if signed_vol > 0.0 || cos_angle > cos5_deg {
            mesh.flags[edge.triangle1 as usize] |=
                edge_flags_concave[edge.triangle_edge_index1 as usize] as u8;
            mesh.flags[edge.triangle2 as usize] |=
                edge_flags_concave[edge.triangle_edge_index2 as usize] as u8;
        }

        if signed_vol < 0.0 || cos_angle > cos5_deg {
            mesh.flags[edge.triangle1 as usize] |=
                edge_flags_inverse[edge.triangle_edge_index1 as usize] as u8;
            mesh.flags[edge.triangle2 as usize] |=
                edge_flags_inverse[edge.triangle_edge_index2 as usize] as u8;
        }
    }

    let padded_angle_count = (angle_count + 3) & !3;
    debug_assert!(padded_angle_count <= angle_capacity);
    for i in angle_count..padded_angle_count {
        angle_ys[i] = 0.0;
        angle_xs[i] = 0.0;
        angle_length_sqs[i] = 0.0;
    }

    for i in (0..angle_count).step_by(4) {
        let x = mul_w(
            load_w(&angle_xs[i..]),
            sqrt_w(load_w(&angle_length_sqs[i..])),
        );
        let angle = atan2_w(load_w(&angle_ys[i..]), x);
        store_w(&mut angle_ys[i..], angle);
    }

    for i in 0..angle_count {
        let quantized = quantize_edge_angle(angle_ys[i]);
        mesh.edge_angles[angle_slots[2 * i] as usize] = quantized;
        mesh.edge_angles[angle_slots[2 * i + 1] as usize] = quantized;
    }

    debug_assert!(mesh.vertex_flag_offset != 0);
    debug_assert!(mesh.vertex_flags.len() == vertex_count);
    for flag in mesh.vertex_flags.iter_mut() {
        *flag = FLAT_VERTEX;
    }

    // Vertex flags for ghost collision handling.
    for i in 0..triangle_count {
        let triangle = mesh.triangles[i];
        let indices = [triangle.index1, triangle.index2, triangle.index3];
        for k in 0..3 {
            let angle = mesh.edge_angles[3 * i + k];
            let a = indices[k] as usize;
            let b = indices[if k < 2 { k + 1 } else { 0 }] as usize;

            // Edge not flat?
            if angle != 0 {
                mesh.vertex_flags[a] &= !FLAT_VERTEX;
                mesh.vertex_flags[b] &= !FLAT_VERTEX;
            }

            // Edge concave?
            if angle < 0 && angle != BOUNDARY_EDGE_ANGLE {
                mesh.vertex_flags[a] |= CONCAVE_VERTEX;
                mesh.vertex_flags[b] |= CONCAVE_VERTEX;
            }
        }
    }
}
