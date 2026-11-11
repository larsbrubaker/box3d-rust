//! Oracle scene meshes: up facing grids with alternating diagonals over a few height
//! functions, a fan, a saddle ridge, and a conveyor rail.
//!
//! Test-support port of the mesh builders in `box3d-cpp-reference/shared/ghost_oracle.c`
//! (`OracleBuilder`, `AddVertex`, `AddTriangle`, `BuildGrid`, the height functions,
//! `BuildFan`, `BuildSaddleRidge`, `BuildRail`, and `CreateOracleMesh`).
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::OracleScene;
use crate::math_functions::{
    abs_float, compute_cos_sin, cross, dot, sub, Vec3, PI, VEC3_AXIS_Y, VEC3_AXIS_Z, VEC3_ZERO,
};
use crate::mesh::{create_mesh, MeshData, MeshDef};
use crate::test_support::ghost_collisions::{GHOST_RAIL_HEIGHT, GHOST_RAIL_THICKNESS};

/// The wave scene is the largest, 169 vertices and 288 triangles. (ORACLE_MAX_VERTICES)
const ORACLE_MAX_VERTICES: usize = 256;

/// (ORACLE_MAX_INDICES)
const ORACLE_MAX_INDICES: usize = 3 * 512;

/// (OracleBuilder)
struct OracleBuilder {
    vertices: Vec<Vec3>,
    indices: Vec<i32>,
}

impl OracleBuilder {
    /// (AddVertex)
    fn add_vertex(&mut self, vertex: Vec3) -> i32 {
        debug_assert!(self.vertices.len() < ORACLE_MAX_VERTICES);
        if self.vertices.len() == ORACLE_MAX_VERTICES {
            return 0;
        }

        self.vertices.push(vertex);
        self.vertices.len() as i32 - 1
    }

    /// Winds the triangle to face the given direction. (AddTriangle)
    fn add_triangle(&mut self, a: i32, b: i32, c: i32, facing: Vec3) {
        debug_assert!(self.indices.len() + 3 <= ORACLE_MAX_INDICES);
        if self.indices.len() + 3 > ORACLE_MAX_INDICES {
            return;
        }

        let (mut b, mut c) = (b, c);
        let va = self.vertices[a as usize];
        let normal = cross(
            sub(self.vertices[b as usize], va),
            sub(self.vertices[c as usize], va),
        );
        if dot(normal, facing) < 0.0 {
            core::mem::swap(&mut b, &mut c);
        }

        self.indices.push(a);
        self.indices.push(b);
        self.indices.push(c);
    }
}

/// (OracleHeightFcn)
type OracleHeightFcn = fn(f32, f32) -> f32;

/// Up facing grid with alternating diagonals. (BuildGrid)
fn build_grid(
    builder: &mut OracleBuilder,
    count_x: i32,
    count_z: i32,
    cell_size: f32,
    height: OracleHeightFcn,
) {
    let x0 = -0.5 * cell_size * count_x as f32;
    let z0 = -0.5 * cell_size * count_z as f32;
    let base = builder.vertices.len() as i32;
    for k in 0..=count_z {
        for i in 0..=count_x {
            let x = x0 + cell_size * i as f32;
            let z = z0 + cell_size * k as f32;
            builder.add_vertex(Vec3 {
                x,
                y: height(x, z),
                z,
            });
        }
    }

    let stride = count_x + 1;
    for k in 0..count_z {
        for i in 0..count_x {
            let a = base + k * stride + i;
            let b = a + 1;
            let c = a + stride + 1;
            let d = a + stride;
            if ((i + k) & 1) != 0 {
                builder.add_triangle(a, d, b, VEC3_AXIS_Y);
                builder.add_triangle(b, d, c, VEC3_AXIS_Y);
            } else {
                builder.add_triangle(a, d, c, VEC3_AXIS_Y);
                builder.add_triangle(a, c, b, VEC3_AXIS_Y);
            }
        }
    }
}

/// (FlatHeight)
fn flat_height(_x: f32, _z: f32) -> f32 {
    0.0
}

/// (CrestHeight)
fn crest_height(x: f32, _z: f32) -> f32 {
    -0.26795 * abs_float(x)
}

/// (ValleyHeight)
fn valley_height(x: f32, _z: f32) -> f32 {
    0.26795 * abs_float(x)
}

/// (RidgeHeight)
fn ridge_height(x: f32, _z: f32) -> f32 {
    -abs_float(x)
}

/// libm trig differs in the last bit between platforms, which would make each a different
/// mesh. (WaveHeight)
fn wave_height(x: f32, z: f32) -> f32 {
    let s1 = compute_cos_sin(2.3 * x + 0.4).sine;
    let c1 = compute_cos_sin(1.7 * z).cosine;
    let s2 = compute_cos_sin(4.1 * z - 1.3 * x).sine;
    0.12 * s1 * c1 + 0.05 * s2
}

/// (BuildFan)
fn build_fan(builder: &mut OracleBuilder) {
    let center = builder.add_vertex(VEC3_ZERO);
    for i in 0..6 {
        let cs = compute_cos_sin((i as f32 + 0.5) * PI / 3.0);
        builder.add_vertex(Vec3 {
            x: 2.0 * cs.cosine,
            y: 0.0,
            z: 2.0 * cs.sine,
        });
    }

    for i in 0..6 {
        builder.add_triangle(center, 1 + i, 1 + (i + 1) % 6, VEC3_AXIS_Y);
    }
}

/// A ridge along x running into a rising wall, so the ridge end is a saddle vertex.
/// (BuildSaddleRidge)
fn build_saddle_ridge(builder: &mut OracleBuilder) {
    let xs = [-2.0f32, 0.0, 1.0];
    let crests = [-0.004f32, 0.0, 0.3];
    let sides = [-0.7f32, -0.7, -0.4];
    for i in 0..3 {
        builder.add_vertex(Vec3 {
            x: xs[i],
            y: sides[i],
            z: -1.0,
        });
        builder.add_vertex(Vec3 {
            x: xs[i],
            y: crests[i],
            z: 0.0,
        });
        builder.add_vertex(Vec3 {
            x: xs[i],
            y: sides[i],
            z: 1.0,
        });
    }

    for i in 0..2 {
        for k in 0..2 {
            let a = 3 * i + k;
            builder.add_triangle(a, a + 1, a + 3, VEC3_AXIS_Y);
            builder.add_triangle(a + 3, a + 1, a + 4, VEC3_AXIS_Y);
        }
    }
}

/// A wall facing -z meeting a narrow top at a convex edge, like a conveyor side rail.
/// (BuildRail)
fn build_rail(builder: &mut OracleBuilder) {
    let section_z = [0.0, 0.0, GHOST_RAIL_THICKNESS, GHOST_RAIL_THICKNESS];
    let section_y = [0.0, GHOST_RAIL_HEIGHT, GHOST_RAIL_HEIGHT, 0.0];
    let columns = 5;
    for row in 0..4 {
        for column in 0..columns {
            builder.add_vertex(Vec3 {
                x: -0.5 + 0.25 * column as f32,
                y: section_y[row],
                z: section_z[row],
            });
        }
    }

    let facings = [
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: -1.0,
        },
        VEC3_AXIS_Y,
        VEC3_AXIS_Z,
    ];
    for row in 0..3i32 {
        for column in 0..columns - 1 {
            let a = columns * row + column;
            let b = a + 1;
            let c = a + columns + 1;
            let d = a + columns;
            if ((row + column) & 1) != 0 {
                builder.add_triangle(a, d, b, facings[row as usize]);
                builder.add_triangle(d, c, b, facings[row as usize]);
            } else {
                builder.add_triangle(a, d, c, facings[row as usize]);
                builder.add_triangle(c, b, a, facings[row as usize]);
            }
        }
    }
}

/// (CreateOracleMesh)
pub(crate) fn create_oracle_mesh(scene: OracleScene) -> Option<MeshData> {
    let mut builder = OracleBuilder {
        vertices: Vec::with_capacity(ORACLE_MAX_VERTICES),
        indices: Vec::with_capacity(ORACLE_MAX_INDICES),
    };

    match scene {
        OracleScene::Flat => build_grid(&mut builder, 6, 6, 0.5, flat_height),
        OracleScene::Fan => build_fan(&mut builder),
        OracleScene::Crest => build_grid(&mut builder, 8, 4, 0.5, crest_height),
        OracleScene::Valley => build_grid(&mut builder, 8, 4, 0.5, valley_height),
        OracleScene::Ridge => build_grid(&mut builder, 6, 4, 0.5, ridge_height),
        OracleScene::SaddleRidge => build_saddle_ridge(&mut builder),
        OracleScene::Rail => build_rail(&mut builder),
        OracleScene::Wave => build_grid(&mut builder, 12, 12, 0.25, wave_height),
    }

    let def = MeshDef {
        vertices: builder.vertices,
        indices: builder.indices,
        weld_vertices: true,
        weld_tolerance: 0.001,
        identify_edges: true,
        ..Default::default()
    };
    create_mesh(&def, None)
}
