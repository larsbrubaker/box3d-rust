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
use box3d_rust::math_functions::{mul_transforms, Pos, Transform, Vec3};
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
    /// Optional local transform for compound children (world = body × local).
    pub local: Option<Transform>,
}

impl VisBody {
    pub fn box_body(body_index: i32, hx: f32, hy: f32, hz: f32) -> Self {
        Self {
            body_index,
            kind: KIND_BOX,
            params: [hx, hy, hz, 0.0, 0.0, 0.0, 0.0],
            local: None,
        }
    }

    pub fn box_local(body_index: i32, hx: f32, hy: f32, hz: f32, local: Transform) -> Self {
        Self {
            body_index,
            kind: KIND_BOX,
            params: [hx, hy, hz, 0.0, 0.0, 0.0, 0.0],
            local: Some(local),
        }
    }

    pub fn sphere_body(body_index: i32, radius: f32) -> Self {
        Self {
            body_index,
            kind: KIND_SPHERE,
            params: [radius, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            local: None,
        }
    }

    pub fn sphere_local(body_index: i32, radius: f32, local: Transform) -> Self {
        Self {
            body_index,
            kind: KIND_SPHERE,
            params: [radius, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            local: Some(local),
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
            local: None,
        }
    }
}

pub fn push_poses(world: &World, bodies: &[VisBody], out: &mut Vec<f32>) {
    out.clear();
    out.reserve(bodies.len() * POSE_STRIDE);
    for b in bodies {
        let xf = get_body_transform(world, b.body_index);
        let (px, py, pz, qx, qy, qz, qw) = if let Some(local) = b.local {
            let parent = Transform {
                p: Vec3 {
                    x: xf.p.x as f32,
                    y: xf.p.y as f32,
                    z: xf.p.z as f32,
                },
                q: xf.q,
            };
            let world_xf = mul_transforms(parent, local);
            (
                world_xf.p.x,
                world_xf.p.y,
                world_xf.p.z,
                world_xf.q.v.x,
                world_xf.q.v.y,
                world_xf.q.v.z,
                world_xf.q.s,
            )
        } else {
            (
                xf.p.x as f32,
                xf.p.y as f32,
                xf.p.z as f32,
                xf.q.v.x,
                xf.q.v.y,
                xf.q.v.z,
                xf.q.s,
            )
        };
        out.push(px);
        out.push(py);
        out.push(pz);
        out.push(qx);
        out.push(qy);
        out.push(qz);
        out.push(qw);
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
