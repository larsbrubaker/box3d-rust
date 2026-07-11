//! Port of box3d-cpp-reference/test/test_height_field.c (all 9 subtests).
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::distance::{make_proxy, shape_cast, CastOutput, ShapeCastPairInput};
use crate::geometry::{RayCastInput, ShapeCastInput};
use crate::height_field::{
    create_grid, create_height_field, create_wave, destroy_height_field, dump_height_data,
    get_height_field_compressed_heights, get_height_field_material_indices,
    get_height_field_triangle, get_height_field_triangle_count, load_height_field,
    overlap_height_field, ray_cast_height_field, shape_cast_height_field, HeightFieldData,
    HeightFieldDef, HEIGHT_FIELD_HOLE, HEIGHT_FIELD_VERSION,
};
use crate::math_functions::{
    cross, intersect_ray_triangle, normalize, sub, Vec3, TRANSFORM_IDENTITY,
};
use std::fs;

fn ensure_small(value: f32, tolerance: f32) {
    assert!(
        !(value < -tolerance || tolerance < value),
        "|{value}| > tolerance {tolerance}"
    );
}

fn v(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

#[test]
fn height_field_create() {
    let scale = v(1.0, 1.0, 1.0);
    let hf = create_grid(4, 4, scale, false);

    assert_eq!(hf.row_count, 4);
    assert_eq!(hf.column_count, 4);
    assert!(!hf.clockwise);
    assert_eq!(hf.version, HEIGHT_FIELD_VERSION);

    ensure_small(hf.aabb.lower_bound.x, f32::EPSILON);
    ensure_small(hf.aabb.lower_bound.y, f32::EPSILON);
    ensure_small(hf.aabb.lower_bound.z, f32::EPSILON);

    ensure_small(hf.aabb.upper_bound.x - 3.0, f32::EPSILON);
    ensure_small(hf.aabb.upper_bound.y, f32::EPSILON);
    ensure_small(hf.aabb.upper_bound.z - 3.0, f32::EPSILON);

    destroy_height_field(hf);
}

#[test]
fn height_field_triangle_index() {
    // Asymmetric grid (rows != cols) catches off-by-one errors between
    // vertex stride (column_count) and cell stride (column_count - 1).
    let row_count = 4;
    let column_count = 5;
    let scale = v(1.0, 1.0, 1.0);
    let hf = create_grid(row_count, column_count, scale, false);

    let triangle_count = 2 * (row_count - 1) * (column_count - 1);

    for triangle_index in 0..triangle_count {
        let quad_index = triangle_index >> 1;
        let sub = triangle_index & 1;
        let row = quad_index / (column_count - 1);
        let column = quad_index - row * (column_count - 1);

        let index11 = row * column_count + column;
        let index12 = index11 + 1;
        let index21 = (row + 1) * column_count + column;
        let index22 = index21 + 1;

        let t = get_height_field_triangle(&hf, triangle_index);

        if sub == 0 {
            // Triangle 0 (CCW): {11, 21, 12}
            assert_eq!(t.i1, index11);
            assert_eq!(t.i2, index21);
            assert_eq!(t.i3, index12);
        } else {
            // Triangle 1 (CCW): {22, 12, 21}
            assert_eq!(t.i1, index22);
            assert_eq!(t.i2, index12);
            assert_eq!(t.i3, index21);
        }
    }

    destroy_height_field(hf);
}

#[test]
fn height_field_winding() {
    let heights = [0.0f32; 9];
    let materials = [0u8; 4];

    let mut def = HeightFieldDef {
        heights: heights.to_vec(),
        material_indices: materials.to_vec(),
        scale: v(1.0, 1.0, 1.0),
        count_x: 3,
        count_z: 3,
        global_minimum_height: -1.0,
        global_maximum_height: 1.0,
        clockwise_winding: false,
    };

    let ccw = create_height_field(&def);

    def.clockwise_winding = true;
    let cw = create_height_field(&def);

    let ta = get_height_field_triangle(&ccw, 0);
    let tb = get_height_field_triangle(&cw, 0);

    let na = normalize(cross(
        sub(ta.vertices[1], ta.vertices[0]),
        sub(ta.vertices[2], ta.vertices[0]),
    ));
    let nb = normalize(cross(
        sub(tb.vertices[1], tb.vertices[0]),
        sub(tb.vertices[2], tb.vertices[0]),
    ));

    ensure_small(na.x, f32::EPSILON);
    ensure_small(na.y - 1.0, f32::EPSILON);
    ensure_small(na.z, f32::EPSILON);

    ensure_small(nb.x, f32::EPSILON);
    ensure_small(nb.y + 1.0, f32::EPSILON);
    ensure_small(nb.z, f32::EPSILON);

    destroy_height_field(ccw);
    destroy_height_field(cw);
}

#[test]
fn ray_cast_flat_field() {
    let heights = [0.0f32; 16];
    let materials = [0u8; 9];

    let def = HeightFieldDef {
        heights: heights.to_vec(),
        material_indices: materials.to_vec(),
        scale: v(1.0, 1.0, 1.0),
        count_x: 4,
        count_z: 4,
        global_minimum_height: -1.0,
        global_maximum_height: 1.0,
        clockwise_winding: false,
    };

    let hf = create_height_field(&def);

    let input = RayCastInput {
        origin: v(1.25, 10.0, 1.25),
        translation: v(0.0, -20.0, 0.0),
        max_fraction: 1.0,
    };

    let out = ray_cast_height_field(&hf, &input);

    assert!(out.hit);
    ensure_small(out.fraction - 0.5, 1e-5);
    ensure_small(out.normal.x, 1e-5);
    ensure_small(out.normal.y - 1.0, 1e-5);
    ensure_small(out.normal.z, 1e-5);

    destroy_height_field(hf);
}

#[test]
fn overlap_at_surface() {
    let scale = v(1.0, 1.0, 1.0);
    let hf = create_grid(4, 4, scale, false);

    let above = v(1.5, 1.0, 1.5);
    let proxy_above = make_proxy(&[above], 0.5);
    let hit_above = overlap_height_field(&hf, TRANSFORM_IDENTITY, &proxy_above);
    assert!(!hit_above);

    let through = v(1.5, 0.0, 1.5);
    let proxy_through = make_proxy(&[through], 0.5);
    let hit_through = overlap_height_field(&hf, TRANSFORM_IDENTITY, &proxy_through);
    assert!(hit_through);

    destroy_height_field(hf);
}

#[test]
fn file_roundtrip() {
    let heights = [0.0f32, 0.5, -0.3, 0.1, 0.0, 0.0, 0.0, 0.2, 0.0];
    let materials = [0u8, HEIGHT_FIELD_HOLE, 1, 2];

    let def = HeightFieldDef {
        heights: heights.to_vec(),
        material_indices: materials.to_vec(),
        scale: v(1.5, 2.0, 0.75),
        count_x: 3,
        count_z: 3,
        global_minimum_height: -1.0,
        global_maximum_height: 1.0,
        clockwise_winding: true,
    };

    let path = "test_height_field_roundtrip.dat";
    dump_height_data(&def, path);

    let loaded = load_height_field(path).expect("load height field");
    let _ = fs::remove_file(path);

    assert_eq!(loaded.row_count, def.count_z);
    assert_eq!(loaded.column_count, def.count_x);
    assert_eq!(loaded.clockwise, def.clockwise_winding);

    ensure_small(loaded.scale.x - def.scale.x, f32::EPSILON);
    ensure_small(loaded.scale.y - def.scale.y, f32::EPSILON);
    ensure_small(loaded.scale.z - def.scale.z, f32::EPSILON);
    ensure_small(loaded.min_height - def.global_minimum_height, f32::EPSILON);
    ensure_small(loaded.max_height - def.global_maximum_height, f32::EPSILON);

    let cell_count = ((def.count_x - 1) * (def.count_z - 1)) as usize;
    for i in 0..cell_count {
        assert_eq!(get_height_field_material_indices(&loaded)[i], materials[i]);
    }

    let quantum =
        (def.global_maximum_height - def.global_minimum_height) / (u16::MAX as f32);
    for i in 0..(def.count_x * def.count_z) as usize {
        let recovered = loaded.min_height
            + loaded.height_scale * (get_height_field_compressed_heights(&loaded)[i] as f32);
        ensure_small(recovered - heights[i], 2.0 * quantum);
    }

    destroy_height_field(loaded);
}

#[test]
fn shape_cast_vertical_straddle() {
    let heights = [0.0f32; 9];
    let materials = [
        0u8,
        HEIGHT_FIELD_HOLE,
        HEIGHT_FIELD_HOLE,
        HEIGHT_FIELD_HOLE,
    ];

    let def = HeightFieldDef {
        heights: heights.to_vec(),
        material_indices: materials.to_vec(),
        scale: v(1.0, 1.0, 1.0),
        count_x: 3,
        count_z: 3,
        global_minimum_height: -1.0,
        global_maximum_height: 1.0,
        clockwise_winding: false,
    };

    let hf = create_height_field(&def);
    let radius = 0.3;

    {
        let center = v(1.05, 10.0, 0.5);
        let input = ShapeCastInput {
            proxy: make_proxy(&[center], radius),
            translation: v(0.0, -20.0, 0.0),
            max_fraction: 1.0,
            can_encroach: false,
        };

        let out = shape_cast_height_field(&hf, &input);
        assert!(out.hit);
        ensure_small(out.fraction - 0.4852098, 2e-3);
    }

    {
        let center = v(0.5, 10.0, 1.05);
        let input = ShapeCastInput {
            proxy: make_proxy(&[center], radius),
            translation: v(0.0, -20.0, 0.0),
            max_fraction: 1.0,
            can_encroach: false,
        };

        let out = shape_cast_height_field(&hf, &input);
        assert!(out.hit);
        ensure_small(out.fraction - 0.4852098, 2e-3);
    }

    {
        let center = v(1.05, 10.0, 1.05);
        let input = ShapeCastInput {
            proxy: make_proxy(&[center], radius),
            translation: v(0.0, -20.0, 0.0),
            max_fraction: 1.0,
            can_encroach: false,
        };

        let out = shape_cast_height_field(&hf, &input);
        assert!(out.hit);
        ensure_small(out.fraction - 0.4854226, 2e-3);
    }

    destroy_height_field(hf);
}

fn brute_force_shape_cast(hf: &HeightFieldData, input: &ShapeCastInput) -> CastOutput {
    let mut best = CastOutput::default();
    let mut best_fraction = input.max_fraction;

    let triangle_count = get_height_field_triangle_count(hf);
    for t in 0..triangle_count {
        let cell_index = (t >> 1) as usize;
        if get_height_field_material_indices(hf)[cell_index] == HEIGHT_FIELD_HOLE {
            continue;
        }

        let tri = get_height_field_triangle(hf, t);

        let pair = ShapeCastPairInput {
            proxy_a: make_proxy(&tri.vertices, 0.0),
            proxy_b: input.proxy,
            transform: TRANSFORM_IDENTITY,
            translation_b: input.translation,
            max_fraction: best_fraction,
            can_encroach: input.can_encroach,
        };

        let out = shape_cast(&pair);
        if out.hit && out.fraction < best_fraction {
            best_fraction = out.fraction;
            best = out;
            best.triangle_index = t;
        }
    }

    best
}

#[test]
fn shape_cast_brute_force() {
    let scale = v(2.0, 1.5, 2.0);
    let hf = create_wave(10, 10, scale, 0.1, 0.03333, false);

    {
        let origin = v(14.5, 4.0, 11.913);
        let input = ShapeCastInput {
            proxy: make_proxy(&[origin], 0.2),
            translation: v(0.0, -8.0, 6.397),
            max_fraction: 1.0,
            can_encroach: false,
        };

        let grid = shape_cast_height_field(&hf, &input);
        let brute = brute_force_shape_cast(&hf, &input);
        assert!(brute.hit);
        assert_eq!(grid.hit, brute.hit);
        ensure_small(grid.fraction - brute.fraction, 2e-3);
    }

    let radii = [0.15f32, 0.4, 0.9];
    let deltas = [
        v(0.0, -8.0, 0.0),
        v(0.0, -8.0, 6.4),
        v(5.1, -8.0, 0.0),
        v(0.0, -8.0, -6.4),
        v(-5.1, -8.0, 0.0),
        v(6.0, -8.0, 5.0),
        v(-7.0, -8.0, 4.0),
        v(9.0, -3.0, -9.0),
    ];

    let mut failures = 0;
    for xi in 0..5 {
        for zi in 0..5 {
            let origin = v(1.0 + 4.0 * (xi as f32) + 0.05, 4.0, 1.0 + 4.0 * (zi as f32) + 0.05);

            for delta in &deltas {
                for &radius in &radii {
                    let input = ShapeCastInput {
                        proxy: make_proxy(&[origin], radius),
                        translation: *delta,
                        max_fraction: 1.0,
                        can_encroach: false,
                    };

                    let grid = shape_cast_height_field(&hf, &input);
                    let brute = brute_force_shape_cast(&hf, &input);

                    let mut diff = grid.fraction - brute.fraction;
                    if diff < 0.0 {
                        diff = -diff;
                    }

                    if grid.hit != brute.hit || (brute.hit && diff > 2e-3) {
                        eprintln!(
                            "  mismatch: origin=({:.2},{:.2},{:.2}) delta=({:.2},{:.2},{:.2}) r={:.2} \
                             grid(hit={},f={:.5}) brute(hit={},f={:.5} tri={})",
                            origin.x,
                            origin.y,
                            origin.z,
                            delta.x,
                            delta.y,
                            delta.z,
                            radius,
                            grid.hit,
                            grid.fraction,
                            brute.hit,
                            brute.fraction,
                            brute.triangle_index
                        );
                        failures += 1;
                    }
                }
            }
        }
    }

    assert_eq!(failures, 0);
    destroy_height_field(hf);
}

fn brute_force_ray_cast(hf: &HeightFieldData, input: &RayCastInput) -> CastOutput {
    let mut best = CastOutput::default();
    let mut best_fraction = input.max_fraction;

    let ray_start = input.origin;
    let ray_delta = input.translation;

    let triangle_count = get_height_field_triangle_count(hf);
    for t in 0..triangle_count {
        let cell_index = (t >> 1) as usize;
        if get_height_field_material_indices(hf)[cell_index] == HEIGHT_FIELD_HOLE {
            continue;
        }

        let tri = get_height_field_triangle(hf, t);
        let alpha = intersect_ray_triangle(
            ray_start,
            ray_delta,
            tri.vertices[0],
            tri.vertices[1],
            tri.vertices[2],
        );
        if alpha < best_fraction {
            best_fraction = alpha;
            best.hit = true;
            best.fraction = alpha;
            best.triangle_index = t;
        }
    }

    best
}

#[test]
fn ray_cast_brute_force() {
    let scale = v(2.0, 1.5, 2.0);
    let hf = create_wave(10, 10, scale, 0.1, 0.03333, false);

    let deltas = [
        v(0.0, -8.0, 0.0),
        v(0.0, -8.0, 12.0),
        v(12.0, -8.0, 0.0),
        v(0.0, -8.0, -12.0),
        v(-12.0, -8.0, 0.0),
        v(14.0, -8.0, 11.0),
        v(-13.0, -8.0, 9.0),
        v(16.0, -4.0, -15.0),
    ];

    let mut failures = 0;
    for xi in 0..5 {
        for zi in 0..5 {
            let origin = v(1.0 + 4.0 * (xi as f32) + 0.05, 4.0, 1.0 + 4.0 * (zi as f32) + 0.05);

            for delta in &deltas {
                let input = RayCastInput {
                    origin,
                    translation: *delta,
                    max_fraction: 1.0,
                };

                let grid = ray_cast_height_field(&hf, &input);
                let brute = brute_force_ray_cast(&hf, &input);

                let mut diff = grid.fraction - brute.fraction;
                if diff < 0.0 {
                    diff = -diff;
                }

                if grid.hit != brute.hit || (brute.hit && diff > 1e-4) {
                    eprintln!(
                        "  mismatch: origin=({:.2},{:.2},{:.2}) delta=({:.2},{:.2},{:.2}) \
                         grid(hit={},f={:.6} tri={}) brute(hit={},f={:.6} tri={})",
                        origin.x,
                        origin.y,
                        origin.z,
                        delta.x,
                        delta.y,
                        delta.z,
                        grid.hit,
                        grid.fraction,
                        grid.triangle_index,
                        brute.hit,
                        brute.fraction,
                        brute.triangle_index
                    );
                    failures += 1;
                }
            }
        }
    }

    assert_eq!(failures, 0);
    destroy_height_field(hf);
}
