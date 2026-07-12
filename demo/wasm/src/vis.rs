//! Shared visualization pose packing for wasm dynamics demos.
//!
//! Each body is 15 floats:
//! `[px, py, pz, qx, qy, qz, qw, a0..a6, kind]`
//! - kind 0 (box): `a0..a2` = half extents
//! - kind 1 (sphere): `a0` = radius
//! - kind 2 (capsule): `a0..a2` = local center1, `a3..a5` = local center2, `a6` = radius

#![allow(dead_code)]

use box3d_rust::body::get_body_transform;
use box3d_rust::geometry::{Capsule, Sphere};
use box3d_rust::math_functions::{Pos, Vec3};
use box3d_rust::world::World;

pub const POSE_STRIDE: usize = 15;
pub const KIND_BOX: u8 = 0;
pub const KIND_SPHERE: u8 = 1;
pub const KIND_CAPSULE: u8 = 2;

pub struct VisBody {
    pub body_index: i32,
    pub kind: u8,
    /// Box half-extents, sphere radius in [0], or capsule centers + radius.
    pub params: [f32; 7],
}

impl VisBody {
    pub fn box_body(body_index: i32, hx: f32, hy: f32, hz: f32) -> Self {
        Self {
            body_index,
            kind: KIND_BOX,
            params: [hx, hy, hz, 0.0, 0.0, 0.0, 0.0],
        }
    }

    pub fn sphere_body(body_index: i32, radius: f32) -> Self {
        Self {
            body_index,
            kind: KIND_SPHERE,
            params: [radius, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
    }

    pub fn capsule_body(body_index: i32, capsule: &Capsule) -> Self {
        Self {
            body_index,
            kind: KIND_CAPSULE,
            params: [
                capsule.center1.x,
                capsule.center1.y,
                capsule.center1.z,
                capsule.center2.x,
                capsule.center2.y,
                capsule.center2.z,
                capsule.radius,
            ],
        }
    }
}

pub fn push_poses(world: &World, bodies: &[VisBody], out: &mut Vec<f32>) {
    out.clear();
    out.reserve(bodies.len() * POSE_STRIDE);
    for b in bodies {
        let xf = get_body_transform(world, b.body_index);
        out.push(xf.p.x as f32);
        out.push(xf.p.y as f32);
        out.push(xf.p.z as f32);
        out.push(xf.q.v.x);
        out.push(xf.q.v.y);
        out.push(xf.q.v.z);
        out.push(xf.q.s);
        out.extend_from_slice(&b.params);
        out.push(b.kind as f32);
    }
}

pub fn pos(x: f32, y: f32, z: f32) -> Pos {
    Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    }
}

pub fn vec3(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

pub fn sphere(radius: f32) -> Sphere {
    Sphere {
        center: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        radius,
    }
}

pub fn capsule_x(extent: f32, radius: f32) -> Capsule {
    Capsule {
        center1: Vec3 {
            x: -extent,
            y: 0.0,
            z: 0.0,
        },
        center2: Vec3 {
            x: extent,
            y: 0.0,
            z: 0.0,
        },
        radius,
    }
}
