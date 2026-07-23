//! s&box Ghost Collisions floor generation — a faithful port of the procedural
//! two-chunk mesh built by `SBoxGhostCollisions::CreateFloorChunk` and its
//! `EmitPatch` / `EmitSlope` / `EmitWall` emitters (`sample_issues.cpp` at c52908c).
//!
//! The tessellation must match C bit-for-bit: the whole point of the sample is
//! reproducing ghost collisions at the hash-picked T-junction seams and the
//! below-plane chamfer/pit facets, so the split diagonals, cell counts, and the
//! integer hash that picks each tile's resolution are ported exactly (same float
//! arithmetic order, same `(int)` truncation, same wrapping integer multiplies).
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use box3d_rust::math_functions::Vec3;

/// s&box works in inches: 1 unit = 0.0254 m (C `SBoxGhostCollisions::SRC`).
pub const SRC: f32 = 0.0254;

// Floor layout (s&box inches) — C static constexpr members.
const HALF_LENGTH_U: i32 = 256; // strip half length along x
const HALF_WIDTH_U: i32 = 64; // strip half width along z
const TILE_SIZE_U: i32 = 32; // slab tile stride

// Beam section geometry.
const BEAM_PITCH_U: f32 = 22.0;
const BEAM_WIDTH_U: f32 = 12.0;
const CHAMFER_WIDTH_U: f32 = 1.5;
const CHAMFER_DROP_U: f32 = 1.0;
const PIT_DEPTH_U: f32 = 24.0;
const BEAM_COUNT: i32 = 9;
const BEAM_REGION0: f32 = -94.0; // first beam start
const BEAM_REGION1: f32 = 94.0; // last beam end

/// Half length of the whole strip along x, in s&box inches — the exact chunk-split
/// bound C passes to `CreateFloorChunk` (`(float)m_halfLengthU`).
pub const HALF_LENGTH_INCHES: f32 = HALF_LENGTH_U as f32;

/// Deterministic integer hash for tile tessellation selection (C `Hash`). Uses
/// wrapping multiplies to match the C unsigned overflow exactly.
fn hash(mut x: u32) -> u32 {
    x ^= x >> 16;
    x = x.wrapping_mul(0x7feb_352d);
    x ^= x >> 15;
    x = x.wrapping_mul(0x846c_a68b);
    x ^= x >> 16;
    x
}

fn emit_triangle(vertices: &mut Vec<Vec3>, indices: &mut Vec<i32>, a: Vec3, b: Vec3, c: Vec3) {
    let base = vertices.len() as i32;
    vertices.push(a);
    vertices.push(b);
    vertices.push(c);
    indices.push(base);
    indices.push(base + 1);
    indices.push(base + 2);
}

/// Horizontal patch at height `y` spanning `[x0,x1] x [z0,z1]` (inches), normal +y
/// (C `EmitPatch`).
#[allow(clippy::too_many_arguments)]
fn emit_patch(
    vertices: &mut Vec<Vec3>,
    indices: &mut Vec<i32>,
    x0: f32,
    x1: f32,
    z0: f32,
    z1: f32,
    y: f32,
    cell: f32,
) {
    if x1 - x0 < 0.01 {
        return;
    }

    let count_x = ((x1 - x0) / cell + 0.99) as i32;
    let count_z = ((z1 - z0) / cell + 0.99) as i32;

    for ix in 0..count_x {
        for iz in 0..count_z {
            let cx0 = x0 + (x1 - x0) * ix as f32 / count_x as f32;
            let cx1 = x0 + (x1 - x0) * (ix + 1) as f32 / count_x as f32;
            let cz0 = z0 + (z1 - z0) * iz as f32 / count_z as f32;
            let cz1 = z0 + (z1 - z0) * (iz + 1) as f32 / count_z as f32;

            let a = Vec3 {
                x: SRC * cx0,
                y: SRC * y,
                z: SRC * cz0,
            };
            let b = Vec3 {
                x: SRC * cx1,
                y: SRC * y,
                z: SRC * cz0,
            };
            let c = Vec3 {
                x: SRC * cx1,
                y: SRC * y,
                z: SRC * cz1,
            };
            let d = Vec3 {
                x: SRC * cx0,
                y: SRC * y,
                z: SRC * cz1,
            };

            // Alternate the split diagonal like typical cooked map data.
            if (ix + iz) & 1 != 0 {
                emit_triangle(vertices, indices, a, d, c);
                emit_triangle(vertices, indices, a, c, b);
            } else {
                emit_triangle(vertices, indices, a, d, b);
                emit_triangle(vertices, indices, b, d, c);
            }
        }
    }
}

/// Sloped strip from edge `(x_low, y_low)` to edge `(x_high, y_high)` spanning the
/// full z width (C `EmitSlope`).
fn emit_slope(
    vertices: &mut Vec<Vec3>,
    indices: &mut Vec<i32>,
    x_low: f32,
    y_low: f32,
    x_high: f32,
    y_high: f32,
    z_cell: f32,
) {
    let count_z = (2.0 * HALF_WIDTH_U as f32 / z_cell + 0.99) as i32;
    for iz in 0..count_z {
        let z0 = -(HALF_WIDTH_U as f32) + 2.0 * HALF_WIDTH_U as f32 * iz as f32 / count_z as f32;
        let z1 =
            -(HALF_WIDTH_U as f32) + 2.0 * HALF_WIDTH_U as f32 * (iz + 1) as f32 / count_z as f32;

        let l0 = Vec3 {
            x: SRC * x_low,
            y: SRC * y_low,
            z: SRC * z0,
        };
        let l1 = Vec3 {
            x: SRC * x_low,
            y: SRC * y_low,
            z: SRC * z1,
        };
        let h0 = Vec3 {
            x: SRC * x_high,
            y: SRC * y_high,
            z: SRC * z0,
        };
        let h1 = Vec3 {
            x: SRC * x_high,
            y: SRC * y_high,
            z: SRC * z1,
        };

        emit_triangle(vertices, indices, l0, l1, h1);
        emit_triangle(vertices, indices, l0, h1, h0);
    }
}

/// Vertical wall at `x` from `y0` (bottom) to `y1` (top). `facing = +1` faces +x,
/// `-1` faces -x (C `EmitWall`).
fn emit_wall(
    vertices: &mut Vec<Vec3>,
    indices: &mut Vec<i32>,
    x: f32,
    y0: f32,
    y1: f32,
    facing: i32,
    z_cell: f32,
) {
    let count_z = (2.0 * HALF_WIDTH_U as f32 / z_cell + 0.99) as i32;
    for iz in 0..count_z {
        let z0 = -(HALF_WIDTH_U as f32) + 2.0 * HALF_WIDTH_U as f32 * iz as f32 / count_z as f32;
        let z1 =
            -(HALF_WIDTH_U as f32) + 2.0 * HALF_WIDTH_U as f32 * (iz + 1) as f32 / count_z as f32;

        let b0 = Vec3 {
            x: SRC * x,
            y: SRC * y0,
            z: SRC * z0,
        };
        let b1 = Vec3 {
            x: SRC * x,
            y: SRC * y0,
            z: SRC * z1,
        };
        let t0 = Vec3 {
            x: SRC * x,
            y: SRC * y1,
            z: SRC * z0,
        };
        let t1 = Vec3 {
            x: SRC * x,
            y: SRC * y1,
            z: SRC * z1,
        };

        if facing > 0 {
            emit_triangle(vertices, indices, b0, b1, t1);
            emit_triangle(vertices, indices, b0, t1, t0);
        } else {
            emit_triangle(vertices, indices, b0, t0, t1);
            emit_triangle(vertices, indices, b0, t1, b1);
        }
    }
}

/// Clip `[a0,a1]` to `[c0,c1]`; returns the clamped span when it is non-empty
/// (C `ClipSpan`).
fn clip_span(a0: f32, a1: f32, c0: f32, c1: f32) -> Option<(f32, f32)> {
    let o0 = if a0 > c0 { a0 } else { c0 };
    let o1 = if a1 < c1 { a1 } else { c1 };
    if o1 - o0 > 0.01 {
        Some((o0, o1))
    } else {
        None
    }
}

/// Build one floor chunk's raw (unwelded) triangle soup spanning `[x0u, x1u]`
/// inches (C `SBoxGhostCollisions::CreateFloorChunk`). The caller feeds the result
/// to `create_mesh` with the same weld/identify settings as C.
pub fn create_floor_chunk(chunk: i32, x0u: f32, x1u: f32) -> (Vec<Vec3>, Vec<i32>) {
    let mut vertices: Vec<Vec3> = Vec::new();
    let mut indices: Vec<i32> = Vec::new();

    // --- Concrete slabs at y = 0 outside the beam region ---
    // Tiles tessellate at a hash-picked resolution so neighbors meet with
    // T-junctions, like cooked s&box map collision.
    let slab_spans: [[f32; 2]; 2] = [
        [-(HALF_LENGTH_U as f32), BEAM_REGION0],
        [BEAM_REGION1, HALF_LENGTH_U as f32],
    ];
    for span in &slab_spans {
        let Some((s0, s1)) = clip_span(span[0], span[1], x0u, x1u) else {
            continue;
        };

        let mut tx = s0;
        while tx < s1 {
            let tx1 = (tx + TILE_SIZE_U as f32).min(s1);
            let mut tz = -HALF_WIDTH_U;
            while tz < HALF_WIDTH_U {
                let a = ((tx as i32).wrapping_mul(73856093)) as u32;
                let b = (tz.wrapping_mul(19349663)) as u32;
                let c = (chunk as u32).wrapping_mul(2654435761);
                let h = hash(a ^ b ^ c);
                let cells = [4.0f32, 8.0f32, 16.0f32];
                emit_patch(
                    &mut vertices,
                    &mut indices,
                    tx,
                    tx1,
                    tz as f32,
                    (tz + TILE_SIZE_U) as f32,
                    0.0,
                    cells[(h % 3) as usize],
                );
                tz += TILE_SIZE_U;
            }
            tx += TILE_SIZE_U as f32;
        }
    }

    // --- Beam section: flat tops at y = 0, chamfers dropping to pits ---
    let pit_top = -CHAMFER_DROP_U;
    let pit_bottom = -PIT_DEPTH_U;

    for k in 0..BEAM_COUNT {
        let bx = BEAM_REGION0 + BEAM_PITCH_U * k as f32;
        let pit_left = k > 0;
        let pit_right = k < BEAM_COUNT - 1;

        // Flat top (flush with the slab on outer sides).
        let top0 = if pit_left { bx + CHAMFER_WIDTH_U } else { bx };
        let top1 = if pit_right {
            bx + BEAM_WIDTH_U - CHAMFER_WIDTH_U
        } else {
            bx + BEAM_WIDTH_U
        };
        if let Some((s0, s1)) = clip_span(top0, top1, x0u, x1u) {
            emit_patch(
                &mut vertices,
                &mut indices,
                s0,
                s1,
                -(HALF_WIDTH_U as f32),
                HALF_WIDTH_U as f32,
                0.0,
                8.0,
            );
        }

        // Chamfers sloping below the walkable plane.
        if pit_left && bx >= x0u && bx < x1u {
            emit_slope(
                &mut vertices,
                &mut indices,
                bx,
                pit_top,
                bx + CHAMFER_WIDTH_U,
                0.0,
                8.0,
            );
        }
        if pit_right && bx + BEAM_WIDTH_U > x0u && bx + BEAM_WIDTH_U <= x1u {
            emit_slope(
                &mut vertices,
                &mut indices,
                bx + BEAM_WIDTH_U,
                pit_top,
                bx + BEAM_WIDTH_U - CHAMFER_WIDTH_U,
                0.0,
                8.0,
            );
        }

        // Pit to the right of this beam.
        if pit_right {
            let pit_l = bx + BEAM_WIDTH_U;
            let pit_r = bx + BEAM_PITCH_U;
            if pit_l >= x0u && pit_l < x1u {
                emit_wall(
                    &mut vertices,
                    &mut indices,
                    pit_l,
                    pit_bottom,
                    pit_top,
                    1,
                    16.0,
                );
            }
            if pit_r > x0u && pit_r <= x1u {
                emit_wall(
                    &mut vertices,
                    &mut indices,
                    pit_r,
                    pit_bottom,
                    pit_top,
                    -1,
                    16.0,
                );
            }
            if let Some((s0, s1)) = clip_span(pit_l, pit_r, x0u, x1u) {
                emit_patch(
                    &mut vertices,
                    &mut indices,
                    s0,
                    s1,
                    -(HALF_WIDTH_U as f32),
                    HALF_WIDTH_U as f32,
                    pit_bottom,
                    16.0,
                );
            }
        }
    }

    (vertices, indices)
}
