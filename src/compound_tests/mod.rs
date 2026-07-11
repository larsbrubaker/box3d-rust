//! Port of `box3d-cpp-reference/test/test_compound.c`.
//!
//! All subtests that do not need world/body are ported.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

mod create_material;
mod query_cast;
mod serialize;
mod sharing;

use crate::geometry::{default_surface_material, SurfaceMaterial};
use crate::math_functions::Vec3;

fn v(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

fn make_material(friction: f32, user_id: u64) -> SurfaceMaterial {
    let mut m = default_surface_material();
    m.friction = friction;
    m.user_material_id = user_id;
    m
}
