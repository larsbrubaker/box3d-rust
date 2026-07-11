//! AABB, cast, overlap, query, and mover compound tests.

use super::{make_material, v};
use crate::compound::{
    collide_mover_and_compound, compute_compound_aabb, create_compound, destroy_compound,
    get_compound_materials, overlap_compound, query_compound, ray_cast_compound,
    shape_cast_compound, CompoundDef, CompoundHullDef, CompoundSphereDef,
};
use crate::distance::make_proxy;
use crate::geometry::{default_surface_material, Capsule, RayCastInput, ShapeCastInput, Sphere};
use crate::hull::make_box_hull;
use crate::math_functions::{
    make_quat_from_axis_angle, Aabb, Transform, PI, QUAT_IDENTITY, TRANSFORM_IDENTITY, VEC3_AXIS_Z,
    VEC3_ZERO,
};

#[test]
fn compound_aabb_contains_children() {
    let mat = default_surface_material();
    let spheres = [
        CompoundSphereDef {
            sphere: Sphere {
                center: v(-3.0, 0.0, 0.0),
                radius: 1.0,
            },
            material: mat,
        },
        CompoundSphereDef {
            sphere: Sphere {
                center: v(4.0, 0.0, 0.0),
                radius: 0.5,
            },
            material: mat,
        },
    ];
    let c = create_compound(&CompoundDef {
        spheres: &spheres,
        ..Default::default()
    })
    .unwrap();
    let local = compute_compound_aabb(&c, TRANSFORM_IDENTITY);
    assert!(local.lower_bound.x <= -4.0 + 1e-5);
    assert!(local.upper_bound.x >= 4.5 - 1e-5);
    assert!(local.lower_bound.y <= -1.0 + 1e-5);
    assert!(local.upper_bound.y >= 1.0 - 1e-5);
    let xf = Transform {
        p: v(10.0, 20.0, 30.0),
        q: QUAT_IDENTITY,
    };
    let world = compute_compound_aabb(&c, xf);
    assert!((world.lower_bound.x - (local.lower_bound.x + 10.0)).abs() < 1e-4);
    assert!((world.upper_bound.y - (local.upper_bound.y + 20.0)).abs() < 1e-4);
    assert!((world.lower_bound.z - (local.lower_bound.z + 30.0)).abs() < 1e-4);
    destroy_compound(c);
}

#[test]
fn compound_ray_cast_miss() {
    let mat = default_surface_material();
    let sph = CompoundSphereDef {
        sphere: Sphere {
            center: VEC3_ZERO,
            radius: 0.5,
        },
        material: mat,
    };
    let c = create_compound(&CompoundDef {
        spheres: &[sph],
        ..Default::default()
    })
    .unwrap();
    let out = ray_cast_compound(
        &c,
        &RayCastInput {
            origin: v(-5.0, 5.0, 0.0),
            translation: v(10.0, 0.0, 0.0),
            max_fraction: 1.0,
        },
    );
    assert!(!out.hit);
    destroy_compound(c);
}

#[test]
fn compound_ray_cast_closest() {
    let spheres = [
        CompoundSphereDef {
            sphere: Sphere {
                center: v(5.0, 0.0, 0.0),
                radius: 1.0,
            },
            material: make_material(0.4, 100),
        },
        CompoundSphereDef {
            sphere: Sphere {
                center: v(10.0, 0.0, 0.0),
                radius: 1.0,
            },
            material: make_material(0.4, 200),
        },
    ];
    let c = create_compound(&CompoundDef {
        spheres: &spheres,
        ..Default::default()
    })
    .unwrap();
    let out = ray_cast_compound(
        &c,
        &RayCastInput {
            origin: VEC3_ZERO,
            translation: v(20.0, 0.0, 0.0),
            max_fraction: 1.0,
        },
    );
    assert!(out.hit);
    assert!((out.fraction - 0.2).abs() < 1e-4);
    assert!((out.normal.x + 1.0).abs() < 1e-4);
    assert_eq!(out.child_index, 0);
    assert_eq!(
        get_compound_materials(&c)[out.material_index as usize].user_material_id,
        100
    );
    destroy_compound(c);
}

#[test]
fn compound_ray_cast_hull_normal_rotation() {
    let mat = default_surface_material();
    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    let hull = CompoundHullDef {
        hull: &box_hull.base,
        transform: Transform {
            p: v(5.0, 0.0, 0.0),
            q: make_quat_from_axis_angle(VEC3_AXIS_Z, 0.5 * PI),
        },
        material: mat,
    };
    let c = create_compound(&CompoundDef {
        hulls: &[hull],
        ..Default::default()
    })
    .unwrap();
    let out = ray_cast_compound(
        &c,
        &RayCastInput {
            origin: VEC3_ZERO,
            translation: v(20.0, 0.0, 0.0),
            max_fraction: 1.0,
        },
    );
    assert!(out.hit);
    assert!((out.fraction - 0.2).abs() < 1e-4);
    assert!((out.normal.x + 1.0).abs() < 1e-3);
    assert!(out.normal.y.abs() < 1e-3);
    assert!(out.normal.z.abs() < 1e-3);
    destroy_compound(c);
}

#[test]
fn compound_shape_cast_closest() {
    let mat = default_surface_material();
    let spheres = [
        CompoundSphereDef {
            sphere: Sphere {
                center: v(5.0, 0.0, 0.0),
                radius: 1.0,
            },
            material: mat,
        },
        CompoundSphereDef {
            sphere: Sphere {
                center: v(10.0, 0.0, 0.0),
                radius: 1.0,
            },
            material: mat,
        },
    ];
    let c = create_compound(&CompoundDef {
        spheres: &spheres,
        ..Default::default()
    })
    .unwrap();
    let out = shape_cast_compound(
        &c,
        &ShapeCastInput {
            proxy: make_proxy(&[VEC3_ZERO], 0.25),
            translation: v(20.0, 0.0, 0.0),
            max_fraction: 1.0,
            can_encroach: false,
        },
    );
    assert!(out.hit);
    assert!((out.fraction - 3.75 / 20.0).abs() < 1e-3);
    assert_eq!(out.child_index, 0);
    destroy_compound(c);
}

#[test]
fn compound_overlap() {
    let mat = default_surface_material();
    let spheres = [
        CompoundSphereDef {
            sphere: Sphere {
                center: v(-3.0, 0.0, 0.0),
                radius: 0.5,
            },
            material: mat,
        },
        CompoundSphereDef {
            sphere: Sphere {
                center: v(3.0, 0.0, 0.0),
                radius: 0.5,
            },
            material: mat,
        },
    ];
    let c = create_compound(&CompoundDef {
        spheres: &spheres,
        ..Default::default()
    })
    .unwrap();
    assert!(!overlap_compound(
        &c,
        TRANSFORM_IDENTITY,
        &make_proxy(&[VEC3_ZERO], 0.25)
    ));
    assert!(overlap_compound(
        &c,
        TRANSFORM_IDENTITY,
        &make_proxy(&[v(3.0, 0.0, 0.0)], 0.1)
    ));
    destroy_compound(c);
}

#[test]
fn compound_query() {
    let mat = default_surface_material();
    let spheres = [
        CompoundSphereDef {
            sphere: Sphere {
                center: v(-10.0, 0.0, 0.0),
                radius: 0.5,
            },
            material: mat,
        },
        CompoundSphereDef {
            sphere: Sphere {
                center: VEC3_ZERO,
                radius: 0.5,
            },
            material: mat,
        },
        CompoundSphereDef {
            sphere: Sphere {
                center: v(10.0, 0.0, 0.0),
                radius: 0.5,
            },
            material: mat,
        },
    ];
    let c = create_compound(&CompoundDef {
        spheres: &spheres,
        ..Default::default()
    })
    .unwrap();
    let middle = Aabb {
        lower_bound: v(-1.0, -1.0, -1.0),
        upper_bound: v(1.0, 1.0, 1.0),
    };
    let mut acc = Vec::new();
    query_compound(&c, middle, |_c, child| {
        acc.push(child);
        true
    });
    assert_eq!(acc, vec![1]);

    let wide = Aabb {
        lower_bound: v(-20.0, -1.0, -1.0),
        upper_bound: v(20.0, 1.0, 1.0),
    };
    let mut stop = Vec::new();
    query_compound(&c, wide, |_c, child| {
        stop.push(child);
        stop.len() < 1
    });
    assert_eq!(stop.len(), 1);

    let mut all = Vec::new();
    query_compound(&c, wide, |_c, child| {
        all.push(child);
        true
    });
    assert_eq!(all.len(), 3);
    destroy_compound(c);
}

#[test]
fn compound_mover() {
    let mat = default_surface_material();
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let hulls = [
        CompoundHullDef {
            hull: &box_hull.base,
            transform: Transform {
                p: v(-1.0, 0.0, 0.0),
                q: QUAT_IDENTITY,
            },
            material: mat,
        },
        CompoundHullDef {
            hull: &box_hull.base,
            transform: Transform {
                p: v(1.0, 0.0, 0.0),
                q: QUAT_IDENTITY,
            },
            material: mat,
        },
    ];
    let c = create_compound(&CompoundDef {
        hulls: &hulls,
        ..Default::default()
    })
    .unwrap();
    let mover = Capsule {
        center1: v(-1.0, 0.6, 0.0),
        center2: v(1.0, 0.6, 0.0),
        radius: 0.2,
    };
    let mut planes = [Default::default(); 8];
    let plane_count = collide_mover_and_compound(&mut planes, &c, &mover);
    assert!(plane_count >= 2);
    let up = (0..plane_count)
        .filter(|&i| planes[i as usize].plane.normal.y > 0.9)
        .count();
    assert!(up >= 2);
    let mut one = [Default::default(); 1];
    assert!(collide_mover_and_compound(&mut one, &c, &mover) <= 1);
    destroy_compound(c);
}
