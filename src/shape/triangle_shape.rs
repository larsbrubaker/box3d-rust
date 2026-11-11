//! Mesh and height-field shape dispatch for ghost collision handling.
//!
//! Port of `b3GetNeighborNormal` and `b3IsConvexVertex` from
//! `box3d-cpp-reference/src/shape.h`. C passes the `const b3Shape*` and switches on its type;
//! the port passes a borrowed view of the triangle geometry, [`TriangleShape`], so the mesh
//! collide functions can run on a mesh that is not attached to a world shape (C builds a
//! temporary `b3Shape` for `b3CollideMeshAndHull`).
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::ShapeGeometry;
use crate::height_field::{
    get_height_field_neighbor_normal, is_height_field_vertex_convex, HeightFieldData,
};
use crate::math_functions::Vec3;
use crate::mesh::{
    get_mesh_neighbor_normal, get_mesh_vertex_flag, Mesh, CONCAVE_VERTEX, FLAT_VERTEX,
};

/// A borrowed mesh or height-field shape. (the `b3_meshShape` / `b3_heightShape` cases of
/// `const b3Shape*`)
#[derive(Debug, Clone, Copy)]
pub(crate) enum TriangleShape<'a> {
    Mesh(Mesh<'a>),
    HeightField(&'a HeightFieldData),
}

impl<'a> TriangleShape<'a> {
    /// View a mesh or height-field geometry. `None` for other shape types.
    pub(crate) fn from_geometry(geometry: &'a ShapeGeometry) -> Option<Self> {
        match geometry {
            ShapeGeometry::Mesh { data, scale } => Some(TriangleShape::Mesh(Mesh {
                data,
                scale: *scale,
            })),
            ShapeGeometry::HeightField(height_field) => {
                Some(TriangleShape::HeightField(height_field))
            }
            _ => None,
        }
    }
}

/// The normal of the triangle across an edge. Returns `None` if edge is isolated.
/// (b3GetNeighborNormal)
pub(crate) fn get_neighbor_normal(
    shape: &TriangleShape<'_>,
    triangle_index: i32,
    edge_index: i32,
) -> Option<Vec3> {
    match shape {
        TriangleShape::Mesh(mesh) => get_mesh_neighbor_normal(mesh, triangle_index, edge_index),
        TriangleShape::HeightField(height_field) => {
            get_height_field_neighbor_normal(height_field, triangle_index, edge_index)
        }
    }
}

/// Is a corner of the triangle a convex vertex (neither flat nor concave)?
/// (b3IsConvexVertex)
pub(crate) fn is_convex_vertex(
    shape: &TriangleShape<'_>,
    triangle_index: i32,
    vertex_index: i32,
) -> bool {
    match shape {
        TriangleShape::Mesh(mesh) => {
            let flags = get_mesh_vertex_flag(mesh, triangle_index, vertex_index);
            (flags & (FLAT_VERTEX | CONCAVE_VERTEX) as i32) == 0
        }
        TriangleShape::HeightField(height_field) => {
            is_height_field_vertex_convex(height_field, triangle_index, vertex_index)
        }
    }
}
