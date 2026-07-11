//! Contact manifold demos using ported collide_* functions.

use wasm_bindgen::prelude::*;

use box3d_rust::constants::MAX_MANIFOLD_POINTS;
use box3d_rust::distance::SimplexCache;
use box3d_rust::geometry::{Capsule, Sphere};
use box3d_rust::hull::make_box_hull;
use box3d_rust::manifold::{
    collide_capsules, collide_hull_and_sphere, collide_hulls, collide_spheres, LocalManifold,
    SatCache,
};
use box3d_rust::math_functions::{make_quat_from_axis_angle, Transform, Vec3, VEC3_AXIS_Y};

fn xf(px: f32, py: f32, pz: f32, angle: f32) -> Transform {
    Transform {
        p: Vec3 {
            x: px,
            y: py,
            z: pz,
        },
        q: make_quat_from_axis_angle(VEC3_AXIS_Y, angle),
    }
}

fn pack_manifold(m: &LocalManifold) -> Vec<f32> {
    let mut out = vec![m.normal.x, m.normal.y, m.normal.z, m.point_count as f32];
    for i in 0..m.point_count as usize {
        let p = &m.points[i];
        out.push(p.point.x);
        out.push(p.point.y);
        out.push(p.point.z);
        out.push(p.separation);
    }
    // Pad to a fixed layout for the TS side (up to 4 points).
    while out.len() < 4 + MAX_MANIFOLD_POINTS * 4 {
        out.push(0.0);
    }
    out
}

/// Collide two spheres. Shape A at origin r=1; shape B at (bx,by,bz) r=0.7.
/// Returns [nx,ny,nz, count, p0x,p0y,p0z,sep0, ...].
#[wasm_bindgen]
pub fn collide_spheres_demo(bx: f32, by: f32, bz: f32) -> Vec<f32> {
    let a = Sphere {
        center: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        radius: 1.0,
    };
    let b = Sphere {
        center: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        radius: 0.7,
    };
    let mut m = LocalManifold::default();
    collide_spheres(
        &mut m,
        MAX_MANIFOLD_POINTS as i32,
        &a,
        &b,
        xf(bx, by, bz, 0.0),
    );
    pack_manifold(&m)
}

/// Collide two capsules. A along Y at origin; B at (bx,by,bz) rotated by angle.
#[wasm_bindgen]
pub fn collide_capsules_demo(bx: f32, by: f32, bz: f32, angle: f32) -> Vec<f32> {
    let a = Capsule {
        center1: Vec3 {
            x: 0.0,
            y: -0.8,
            z: 0.0,
        },
        center2: Vec3 {
            x: 0.0,
            y: 0.8,
            z: 0.0,
        },
        radius: 0.35,
    };
    let b = Capsule {
        center1: Vec3 {
            x: 0.0,
            y: -0.6,
            z: 0.0,
        },
        center2: Vec3 {
            x: 0.0,
            y: 0.6,
            z: 0.0,
        },
        radius: 0.3,
    };
    let mut m = LocalManifold::default();
    collide_capsules(
        &mut m,
        MAX_MANIFOLD_POINTS as i32,
        &a,
        &b,
        xf(bx, by, bz, angle),
    );
    pack_manifold(&m)
}

/// Collide a box hull with a sphere at (bx,by,bz).
#[wasm_bindgen]
pub fn collide_hull_sphere_demo(bx: f32, by: f32, bz: f32) -> Vec<f32> {
    let hull = make_box_hull(1.0, 1.0, 1.0);
    let sphere = Sphere {
        center: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        radius: 0.55,
    };
    let mut m = LocalManifold::default();
    let mut cache = SimplexCache::default();
    collide_hull_and_sphere(
        &mut m,
        MAX_MANIFOLD_POINTS as i32,
        &hull.base,
        &sphere,
        xf(bx, by, bz, 0.0),
        &mut cache,
    );
    pack_manifold(&m)
}

/// Collide two box hulls. Fixed unit box vs moving box at (bx,by,bz) rotated.
#[wasm_bindgen]
pub fn collide_hulls_demo(bx: f32, by: f32, bz: f32, angle: f32) -> Vec<f32> {
    let a = make_box_hull(1.0, 1.0, 1.0);
    let b = make_box_hull(0.7, 0.7, 0.7);
    let mut m = LocalManifold::default();
    let mut cache = SatCache::default();
    collide_hulls(
        &mut m,
        MAX_MANIFOLD_POINTS as i32,
        &a.base,
        &b.base,
        xf(bx, by, bz, angle),
        &mut cache,
    );
    pack_manifold(&m)
}
