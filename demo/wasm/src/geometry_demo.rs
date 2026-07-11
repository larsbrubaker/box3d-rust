//! Geometry demo: ray casts and GJK closest points against a 3D scene.

use wasm_bindgen::prelude::*;

use box3d_rust::aabb::ray_cast_aabb;
use box3d_rust::distance::{make_proxy, shape_distance, DistanceInput, SimplexCache};
use box3d_rust::geometry::{
    ray_cast_capsule, ray_cast_sphere, Capsule, RayCastInput, Sphere,
};
use box3d_rust::hull::{make_box_hull, ray_cast_hull};
use box3d_rust::math_functions::{
    add, mul_add, Aabb, Vec3, TRANSFORM_IDENTITY,
};

fn scene_sphere() -> Sphere {
    Sphere {
        center: Vec3 {
            x: 2.5,
            y: 0.5,
            z: 0.0,
        },
        radius: 0.9,
    }
}

fn scene_capsule() -> Capsule {
    Capsule {
        center1: Vec3 {
            x: -2.8,
            y: -0.4,
            z: -0.3,
        },
        center2: Vec3 {
            x: -2.0,
            y: 1.0,
            z: 0.4,
        },
        radius: 0.45,
    }
}

fn scene_box() -> box3d_rust::hull::BoxHull {
    make_box_hull(0.8, 0.8, 0.8)
}

fn scene_aabb() -> Aabb {
    Aabb {
        lower_bound: Vec3 {
            x: -0.6,
            y: 1.6,
            z: -0.6,
        },
        upper_bound: Vec3 {
            x: 0.6,
            y: 2.8,
            z: 0.6,
        },
    }
}

/// Scene outline geometry for drawing:
/// 0 = sphere [cx,cy,cz,r]
/// 1 = capsule [c1x,c1y,c1z, c2x,c2y,c2z, r]
/// 2 = box half-extents + center [hx,hy,hz, cx,cy,cz]
/// 3 = aabb [lx,ly,lz, ux,uy,uz]
#[wasm_bindgen]
pub fn scene_shape(index: u32) -> Vec<f32> {
    match index {
        0 => {
            let s = scene_sphere();
            vec![s.center.x, s.center.y, s.center.z, s.radius]
        }
        1 => {
            let c = scene_capsule();
            vec![
                c.center1.x,
                c.center1.y,
                c.center1.z,
                c.center2.x,
                c.center2.y,
                c.center2.z,
                c.radius,
            ]
        }
        2 => vec![0.8, 0.8, 0.8, 0.0, 0.0, 0.0],
        _ => {
            let a = scene_aabb();
            vec![
                a.lower_bound.x,
                a.lower_bound.y,
                a.lower_bound.z,
                a.upper_bound.x,
                a.upper_bound.y,
                a.upper_bound.z,
            ]
        }
    }
}

fn push_cast(out: &mut Vec<f32>, hit: bool, fraction: f32, point: Vec3, normal: Vec3) {
    out.push(if hit { 1.0 } else { 0.0 });
    out.push(fraction);
    out.push(point.x);
    out.push(point.y);
    out.push(point.z);
    out.push(normal.x);
    out.push(normal.y);
    out.push(normal.z);
}

fn aabb_hit_normal(aabb: Aabb, point: Vec3) -> Vec3 {
    let eps = 1.0e-4;
    if (point.x - aabb.lower_bound.x).abs() < eps {
        return Vec3 {
            x: -1.0,
            y: 0.0,
            z: 0.0,
        };
    }
    if (point.x - aabb.upper_bound.x).abs() < eps {
        return Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        };
    }
    if (point.y - aabb.lower_bound.y).abs() < eps {
        return Vec3 {
            x: 0.0,
            y: -1.0,
            z: 0.0,
        };
    }
    if (point.y - aabb.upper_bound.y).abs() < eps {
        return Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        };
    }
    if (point.z - aabb.lower_bound.z).abs() < eps {
        return Vec3 {
            x: 0.0,
            y: 0.0,
            z: -1.0,
        };
    }
    Vec3 {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    }
}

/// Cast a ray against sphere, capsule, hull box, and AABB.
/// Returns 4 × [hit, fraction, px, py, pz, nx, ny, nz] (32 floats).
#[wasm_bindgen]
pub fn ray_cast_scene(
    ox: f32,
    oy: f32,
    oz: f32,
    tx: f32,
    ty: f32,
    tz: f32,
) -> Vec<f32> {
    let origin = Vec3 {
        x: ox,
        y: oy,
        z: oz,
    };
    let translation = Vec3 {
        x: tx,
        y: ty,
        z: tz,
    };
    let input = RayCastInput {
        origin,
        translation,
        max_fraction: 1.0,
    };

    let sphere_out = ray_cast_sphere(&scene_sphere(), &input);
    let capsule_out = ray_cast_capsule(&scene_capsule(), &input);
    let box_hull = scene_box();
    let hull_out = ray_cast_hull(&box_hull.base, &input);

    let p2 = add(origin, translation);
    let mut min_f = 0.0;
    let mut max_f = 1.0;
    let aabb = scene_aabb();
    let aabb_hit = ray_cast_aabb(aabb, origin, p2, &mut min_f, &mut max_f);
    let aabb_point = mul_add(origin, min_f, translation);
    let aabb_normal = if aabb_hit {
        aabb_hit_normal(aabb, aabb_point)
    } else {
        Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        }
    };

    let mut out = Vec::with_capacity(32);
    push_cast(
        &mut out,
        sphere_out.hit,
        sphere_out.fraction,
        sphere_out.point,
        sphere_out.normal,
    );
    push_cast(
        &mut out,
        capsule_out.hit,
        capsule_out.fraction,
        capsule_out.point,
        capsule_out.normal,
    );
    push_cast(
        &mut out,
        hull_out.hit,
        hull_out.fraction,
        hull_out.point,
        hull_out.normal,
    );
    push_cast(&mut out, aabb_hit, min_f, aabb_point, aabb_normal);
    out
}

/// GJK closest points between a probe sphere at (px,py,pz) and the scene sphere.
/// Returns [ax,ay,az, bx,by,bz, distance, iterations].
#[wasm_bindgen]
pub fn closest_points(px: f32, py: f32, pz: f32) -> Vec<f32> {
    let probe = make_proxy(
        &[Vec3 {
            x: px,
            y: py,
            z: pz,
        }],
        0.35,
    );
    let target = scene_sphere();
    let target_proxy = make_proxy(&[target.center], target.radius);

    let input = DistanceInput {
        proxy_a: probe,
        proxy_b: target_proxy,
        transform: TRANSFORM_IDENTITY,
        use_radii: true,
    };
    let mut cache = SimplexCache::default();
    let output = shape_distance(&input, &mut cache, None);

    vec![
        output.point_a.x,
        output.point_a.y,
        output.point_a.z,
        output.point_b.x,
        output.point_b.y,
        output.point_b.z,
        output.distance,
        output.iterations as f32,
    ]
}
