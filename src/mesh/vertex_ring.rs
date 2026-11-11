//! Mesh vertex rings: for each non-flat vertex, the octahedral encoded directions of its edges
//! and the cliffs below its boundary edges. The ring is the exact vertex normal cone used to
//! reject ghost normals at vertices.
//!
//! Port of `b3VertexRing`, `b3FoldOctahedral`, `b3DecodeRingDirection`, `b3RingEncoder`,
//! `b3PushRingEntry`, `b3EncodeRingEntries`, `b3IsInMeshVertexRing`, `b3BuildVertexRing`, and
//! `b3AppendVertexRings` from `box3d-cpp-reference/src/mesh.c`.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::edges::get_mesh_vertex_index;
use super::types::{
    get_mesh_edge_angles, get_mesh_vertex_flags, vertex_ring_record_offset, Mesh, MeshData,
    BOUNDARY_EDGE_ANGLE, FLAT_VERTEX,
};
use crate::core::NULL_INDEX;
use crate::math_functions::{
    abs, abs_float, align_up8, clamp_int, dot, length, make_normal_from_points, mul, neg,
    normalize, sub, Vec3,
};
use crate::simd_scalar::{
    abs_w, add_w, blend_w, div_w, less_than_w, load_w, mul_w, splat_w, store_w, sub_w, zero_w,
};

// Ring of mesh-space edge directions in 16-bit octahedral encoding. In the blob a ring is a
// b3VertexRing header followed by its b3VertexRingEntry codes, both 16-bit.

/// Size of the ring header, which holds the number of boundary edges with 90 degree cliffs.
/// (sizeof( b3VertexRing ))
const VERTEX_RING_SIZE: i32 = 2;

/// Size of a ring entry, an octohedral encoded edge direction.
/// (sizeof( b3VertexRingEntry ))
const VERTEX_RING_ENTRY_SIZE: i32 = 2;

/// (b3FoldOctahedral)
fn fold_octahedral(x: f32, y: f32) -> Vec3 {
    Vec3 {
        x: (1.0 - abs_float(y)) * if x >= 0.0 { 1.0 } else { -1.0 },
        y: (1.0 - abs_float(x)) * if y >= 0.0 { 1.0 } else { -1.0 },
        z: 0.0,
    }
}

/// (b3DecodeRingDirection)
fn decode_ring_direction(code: u16) -> Vec3 {
    let mut x = (code & 0xFF) as f32 * (2.0 / 255.0) - 1.0;
    let mut y = (code >> 8) as f32 * (2.0 / 255.0) - 1.0;
    let z = 1.0 - abs_float(x) - abs_float(y);
    if z < 0.0 {
        let folded = fold_octahedral(x, y);
        x = folded.x;
        y = folded.y;
    }

    normalize(Vec3 { x, y, z })
}

/// Gathers ring directions and encodes them four at a time. (b3RingEncoder)
struct RingEncoder {
    x: Vec<f32>,
    y: Vec<f32>,
    z: Vec<f32>,
    entries: Vec<u16>,
    count: usize,
    capacity: usize,
}

impl RingEncoder {
    fn new(capacity: usize) -> Self {
        Self {
            x: vec![0.0; capacity],
            y: vec![0.0; capacity],
            z: vec![0.0; capacity],
            entries: vec![0; capacity],
            count: 0,
            capacity,
        }
    }

    /// (b3PushRingEntry)
    fn push(&mut self, direction: Vec3) {
        let index = self.count;
        debug_assert!(index < self.capacity);
        self.x[index] = direction.x;
        self.y[index] = direction.y;
        self.z[index] = direction.z;
        self.count = index + 1;
    }

    /// (b3EncodeRingEntries)
    fn encode(&mut self) {
        let count = self.count;

        let padded_count = (count + 3) & !3;
        debug_assert!(padded_count <= self.capacity);
        for i in count..padded_count {
            self.x[i] = 0.0;
            self.y[i] = 0.0;
            self.z[i] = 1.0;
        }

        for i in (0..count).step_by(4) {
            let x = load_w(&self.x[i..]);
            let y = load_w(&self.y[i..]);
            let z = load_w(&self.z[i..]);
            let zero = zero_w();
            let one = splat_w(1.0);
            let minus_one = splat_w(-1.0);
            let sum = add_w(add_w(abs_w(x), abs_w(y)), abs_w(z));
            let mut ox = div_w(x, sum);
            let mut oy = div_w(y, sum);
            let fx = mul_w(
                sub_w(one, abs_w(oy)),
                blend_w(one, minus_one, less_than_w(ox, zero)),
            );
            let fy = mul_w(
                sub_w(one, abs_w(ox)),
                blend_w(one, minus_one, less_than_w(oy, zero)),
            );
            let below = less_than_w(z, zero);
            ox = blend_w(ox, fx, below);
            oy = blend_w(oy, fy, below);
            let scale = splat_w(127.5);
            let half = splat_w(0.5);
            store_w(&mut self.x[i..], add_w(mul_w(scale, add_w(ox, one)), half));
            store_w(&mut self.y[i..], add_w(mul_w(scale, add_w(oy, one)), half));
        }

        for i in 0..count {
            let u = clamp_int(self.x[i] as i32, 0, 255);
            let v = clamp_int(self.y[i] as i32, 0, 255);
            self.entries[i] = (u | (v << 8)) as u16;
        }
    }
}

/// Is the candidate manifold normal in vertex Voronoi region? `normal` is in mesh space under
/// the mesh scale. A mesh without vertex rings admits every normal.
/// (b3IsInMeshVertexRing)
pub(crate) fn is_in_mesh_vertex_ring(
    mesh: &Mesh<'_>,
    triangle_index: i32,
    vertex_index: i32,
    normal: Vec3,
    tolerance: f32,
) -> bool {
    let data = mesh.data;
    debug_assert!(0 <= triangle_index && triangle_index < data.triangle_count);
    debug_assert!((0..3).contains(&vertex_index));

    if data.vertex_ring_offset == 0 {
        return true;
    }

    let triangle = data.triangles[triangle_index as usize];
    let vertex = get_mesh_vertex_index(mesh, triangle, vertex_index) as usize;

    // Account for the mesh scale, including reflection. The cliff scale is separate
    // to maintain the 90 degree angle that all cliffs use.
    let scale = mesh.scale;
    let abs_scale = abs(scale);
    let uniform = abs_scale.y == abs_scale.x && abs_scale.z == abs_scale.x;
    let (edge_scale, cliff_scale) = if uniform {
        let edge_scale = Vec3 {
            x: if scale.x < 0.0 { -1.0 } else { 1.0 },
            y: if scale.y < 0.0 { -1.0 } else { 1.0 },
            z: if scale.z < 0.0 { -1.0 } else { 1.0 },
        };
        (edge_scale, edge_scale)
    } else {
        let cliff_scale = Vec3 {
            x: 1.0 / scale.x,
            y: 1.0 / scale.y,
            z: 1.0 / scale.z,
        };
        (scale, cliff_scale)
    };

    let starts = &data.vertex_ring_starts;
    let byte_count = starts[vertex + 1] - starts[vertex];
    if byte_count == 0 {
        // Flat vertex
        let vertices = &data.vertices;
        let mut face_normal = make_normal_from_points(
            vertices[triangle.index1 as usize],
            vertices[triangle.index2 as usize],
            vertices[triangle.index3 as usize],
        );
        face_normal = normalize(mul(cliff_scale, face_normal));
        let cos_angle = dot(normal, face_normal);
        return cos_angle > 0.0 && cos_angle * cos_angle >= 1.0 - tolerance * tolerance;
    }

    let record_offset = vertex_ring_record_offset(data.vertex_ring_offset, data.vertex_count);
    let ring = (starts[vertex] as usize - record_offset) / 2;
    let cliff_count = data.vertex_ring_data[ring] as i32;
    let entries = &data.vertex_ring_data[ring + 1..];
    let entry_count = (byte_count - VERTEX_RING_SIZE) / VERTEX_RING_ENTRY_SIZE;
    let edge_count = entry_count - cliff_count;

    let edge_normal = mul(edge_scale, normal);
    let cliff_normal = mul(cliff_scale, normal);

    for i in 0..entry_count {
        let is_cliff = i >= edge_count;
        let direction = decode_ring_direction(entries[i as usize]);
        let lean = dot(if is_cliff { cliff_normal } else { edge_normal }, direction);

        let mut limit = tolerance;
        if !uniform {
            limit *= length(mul(
                if is_cliff { cliff_scale } else { edge_scale },
                direction,
            ));
        }

        if lean > limit {
            return false;
        }
    }

    true
}

/// Push the ring entry directions of a vertex to the encoder. Returns the cliff count.
/// (b3BuildVertexRing)
fn build_vertex_ring(
    mesh: &MeshData,
    normals: &[Vec3],
    fan: &[i32],
    vertex: i32,
    stamps: &mut [i32],
    encoder: &mut RingEncoder,
) -> i32 {
    let triangles = &mesh.triangles;
    let vertices = &mesh.vertices;
    let edge_angles = get_mesh_edge_angles(mesh).unwrap_or(&[]);

    for &triangle_index in fan {
        let triangle = triangles[triangle_index as usize];
        let indices = [triangle.index1, triangle.index2, triangle.index3];
        let k = if indices[0] == vertex {
            0
        } else if indices[1] == vertex {
            1
        } else {
            2
        };

        // The other vertex indices on this triangle that are not the input vertex index.
        let others = [
            indices[if k < 2 { k + 1 } else { 0 }],
            indices[if k > 0 { k - 1 } else { 2 }],
        ];

        for other in others {
            if stamps[other as usize] == vertex {
                continue;
            }

            stamps[other as usize] = vertex;

            // This encodes every edge. The intersection of all planes perpendicular to the edge
            // axis builds the Voronoi region. That process can identify edges that don't
            // contribute to the Voronoi region. That culling could save storage but is skipped
            // for simplicity.
            encoder.push(sub(vertices[other as usize], vertices[vertex as usize]));
        }
    }

    let mut cliff_count = 0;
    for &triangle_index in fan {
        let triangle = triangles[triangle_index as usize];
        let indices = [triangle.index1, triangle.index2, triangle.index3];
        let k = if indices[0] == vertex {
            0
        } else if indices[1] == vertex {
            1
        } else {
            2
        };
        let edge_indices = [k, if k > 0 { k - 1 } else { 2 }];

        for edge_index in edge_indices {
            if edge_angles[(3 * triangle_index) as usize + edge_index] != BOUNDARY_EDGE_ANGLE {
                continue;
            }

            // The cliff is a 90 degree drop.
            encoder.push(neg(normals[triangle_index as usize]));
            cliff_count += 1;
        }
    }

    cliff_count
}

/// Build a neighbor angle ring for each vertex. Used to mitigate ghost collisions with minimal
/// tolerances and fudge factors. Grows the blob by the ring section. (b3AppendVertexRings)
pub(super) fn append_vertex_rings(mesh: &mut MeshData, normals: &[Vec3]) {
    let vertex_flags = get_mesh_vertex_flags(mesh)
        .expect("vertex rings require vertex flags")
        .to_vec();

    let vertex_count = mesh.vertex_count as usize;
    let triangle_count = mesh.triangle_count as usize;

    // Triangle fan counts per vertex via counting sort and prefix sum.
    let mut fan_starts = vec![0i32; vertex_count + 1];

    // Sub-arrays of triangle fans for each vertex.
    let mut fans = vec![0i32; 3 * triangle_count];

    // Edge stamps to avoid visiting the same edge twice when
    // iterating triangles.
    let mut stamps = vec![NULL_INDEX; vertex_count];

    // Byte offsets for each sub-array of ring entries per vertex.
    let mut ring_starts = vec![0i32; vertex_count + 1];

    // Counting sort
    for triangle in &mesh.triangles {
        fan_starts[triangle.index1 as usize + 1] += 1;
        fan_starts[triangle.index2 as usize + 1] += 1;
        fan_starts[triangle.index3 as usize + 1] += 1;
    }

    for i in 0..vertex_count {
        fan_starts[i + 1] += fan_starts[i];
    }

    // Cursors for walking fan sub-arrays.
    let mut cursors = fan_starts[..vertex_count].to_vec();
    for (i, triangle) in mesh.triangles.iter().enumerate() {
        for index in [triangle.index1, triangle.index2, triangle.index3] {
            fans[cursors[index as usize] as usize] = i as i32;
            cursors[index as usize] += 1;
        }
    }

    // Hey dude, I got an offset to the array of offsets.
    let starts_offset = mesh.byte_count as usize;

    // After the array of offsets, the ring sub-arrays begin.
    let ring_offset = starts_offset + align_up8((vertex_count + 1) * core::mem::size_of::<i32>());
    debug_assert!(
        ring_offset == vertex_ring_record_offset(starts_offset as i32, vertex_count as i32)
    );

    // Upper bound on entries: two edges per fan triangle and two cliffs per boundary edge.
    let edge_angles = get_mesh_edge_angles(mesh).expect("vertex rings require edge angles");
    let mut entry_bound = 0i32;
    for i in 0..vertex_count {
        if (vertex_flags[i] & FLAT_VERTEX) == 0 {
            entry_bound += 2 * (fan_starts[i + 1] - fan_starts[i]);
        }
    }

    for &angle in edge_angles.iter().take(3 * triangle_count) {
        if angle == BOUNDARY_EDGE_ANGLE {
            entry_bound += 2;
        }
    }

    let mut encoder = RingEncoder::new(((entry_bound + 3) & !3) as usize);

    let mut entry_starts = vec![0usize; vertex_count + 1];
    let mut cliff_counts = vec![0i32; vertex_count];

    // The vertex ring offset for the current vertex.
    let mut offset = ring_offset;

    for i in 0..vertex_count {
        ring_starts[i] = offset as i32;
        entry_starts[i] = encoder.count;
        cliff_counts[i] = 0;
        if (vertex_flags[i] & FLAT_VERTEX) != 0 {
            // Flat vertices don't store ring data.
            continue;
        }

        let fan = &fans[fan_starts[i] as usize..fan_starts[i + 1] as usize];
        cliff_counts[i] =
            build_vertex_ring(mesh, normals, fan, i as i32, &mut stamps, &mut encoder);
        let entry_count = encoder.count - entry_starts[i];
        offset += VERTEX_RING_SIZE as usize + entry_count * VERTEX_RING_ENTRY_SIZE as usize;
    }

    ring_starts[vertex_count] = offset as i32;
    entry_starts[vertex_count] = encoder.count;

    encoder.encode();

    // We now have the required space, grow the blob.
    let byte_count = ring_offset + align_up8(offset - ring_offset);

    // The ring records as 16-bit words from ring_offset, zero padded to the blob end.
    let mut ring_data = vec![0u16; (byte_count - ring_offset) / 2];

    for i in 0..vertex_count {
        if ring_starts[i + 1] == ring_starts[i] {
            continue;
        }

        let ring = (ring_starts[i] as usize - ring_offset) / 2;
        debug_assert!(cliff_counts[i] <= u16::MAX as i32);
        ring_data[ring] = cliff_counts[i] as u16;

        let entry_count = entry_starts[i + 1] - entry_starts[i];
        ring_data[ring + 1..ring + 1 + entry_count]
            .copy_from_slice(&encoder.entries[entry_starts[i]..entry_starts[i + 1]]);
    }

    mesh.byte_count = byte_count as i32;
    mesh.vertex_ring_offset = starts_offset as i32;

    // The vertex_count + 1 entry is the end offset.
    mesh.vertex_ring_starts = ring_starts;
    mesh.vertex_ring_data = ring_data;
}
