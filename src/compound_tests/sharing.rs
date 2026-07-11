//! Hull/mesh sharing and child-dispatch compound tests.

use super::v;
use crate::compound::{
    create_compound, destroy_compound, get_compound_child, ChildGeometry, CompoundCapsuleDef,
    CompoundDef, CompoundHullDef, CompoundMeshDef, CompoundSphereDef,
};
use crate::geometry::{default_surface_material, Capsule, ShapeType, Sphere};
use crate::hull::make_box_hull;
use crate::math_functions::{TRANSFORM_IDENTITY, VEC3_ZERO};
use crate::mesh::{create_box_mesh, destroy_mesh};

#[test]
fn compound_hull_sharing_pointer() {
    let mat = default_surface_material();
    let box_hull = make_box_hull(1.0, 1.0, 1.0);
    let hulls: Vec<_> = (0..3)
        .map(|i| {
            let mut t = TRANSFORM_IDENTITY;
            t.p.x = (4 * i) as f32;
            CompoundHullDef {
                hull: &box_hull.base,
                transform: t,
                material: mat,
            }
        })
        .collect();
    let c = create_compound(&CompoundDef {
        hulls: &hulls,
        ..Default::default()
    })
    .unwrap();
    assert_eq!(c.hull_count, 3);
    assert_eq!(c.shared_hull_count, 1);
    destroy_compound(c);
}

#[test]
fn compound_hull_sharing_content() {
    let mat = default_surface_material();
    let box_a = make_box_hull(1.0, 1.0, 1.0);
    let box_b = make_box_hull(1.0, 1.0, 1.0);
    assert!(!core::ptr::eq(&box_a, &box_b));
    let mut hulls = [
        CompoundHullDef {
            hull: &box_a.base,
            transform: TRANSFORM_IDENTITY,
            material: mat,
        },
        CompoundHullDef {
            hull: &box_b.base,
            transform: TRANSFORM_IDENTITY,
            material: mat,
        },
    ];
    hulls[1].transform.p.x = 5.0;
    let c = create_compound(&CompoundDef {
        hulls: &hulls,
        ..Default::default()
    })
    .unwrap();
    assert_eq!(c.shared_hull_count, 1);
    destroy_compound(c);
}

#[test]
fn compound_hull_distinct() {
    let mat = default_surface_material();
    let box_a = make_box_hull(1.0, 1.0, 1.0);
    let box_b = make_box_hull(2.0, 1.0, 1.0);
    let mut hulls = [
        CompoundHullDef {
            hull: &box_a.base,
            transform: TRANSFORM_IDENTITY,
            material: mat,
        },
        CompoundHullDef {
            hull: &box_b.base,
            transform: TRANSFORM_IDENTITY,
            material: mat,
        },
    ];
    hulls[1].transform.p.x = 5.0;
    let c = create_compound(&CompoundDef {
        hulls: &hulls,
        ..Default::default()
    })
    .unwrap();
    assert_eq!(c.shared_hull_count, 2);
    destroy_compound(c);
}

#[test]
fn compound_mesh_sharing_pointer() {
    let mat = default_surface_material();
    let materials = [mat];
    let md = create_box_mesh(VEC3_ZERO, v(1.0, 1.0, 1.0), false).expect("mesh");
    let mut meshes = [
        CompoundMeshDef {
            mesh_data: &md,
            transform: TRANSFORM_IDENTITY,
            scale: v(1.0, 1.0, 1.0),
            materials: &materials,
        },
        CompoundMeshDef {
            mesh_data: &md,
            transform: TRANSFORM_IDENTITY,
            scale: v(1.0, 1.0, 1.0),
            materials: &materials,
        },
        CompoundMeshDef {
            mesh_data: &md,
            transform: TRANSFORM_IDENTITY,
            scale: v(1.0, 1.0, 1.0),
            materials: &materials,
        },
    ];
    meshes[1].transform.p.x = 4.0;
    meshes[2].transform.p.x = 8.0;
    let c = create_compound(&CompoundDef {
        meshes: &meshes,
        ..Default::default()
    })
    .unwrap();
    assert_eq!(c.mesh_count, 3);
    assert_eq!(c.shared_mesh_count, 1);
    destroy_compound(c);
    destroy_mesh(md);
}

#[test]
fn compound_mesh_sharing_content() {
    let mat = default_surface_material();
    let md_a = create_box_mesh(VEC3_ZERO, v(1.0, 1.0, 1.0), false).expect("mesh");
    let md_b = create_box_mesh(VEC3_ZERO, v(1.0, 1.0, 1.0), false).expect("mesh");
    assert!(!core::ptr::eq(&md_a, &md_b));
    let mut meshes = [
        CompoundMeshDef {
            mesh_data: &md_a,
            transform: TRANSFORM_IDENTITY,
            scale: v(1.0, 1.0, 1.0),
            materials: &[mat],
        },
        CompoundMeshDef {
            mesh_data: &md_b,
            transform: TRANSFORM_IDENTITY,
            scale: v(1.0, 1.0, 1.0),
            materials: &[mat],
        },
    ];
    meshes[1].transform.p.x = 5.0;
    let c = create_compound(&CompoundDef {
        meshes: &meshes,
        ..Default::default()
    })
    .unwrap();
    assert_eq!(c.shared_mesh_count, 1);
    destroy_compound(c);
    destroy_mesh(md_a);
    destroy_mesh(md_b);
}

#[test]
fn compound_mesh_distinct() {
    let mat = default_surface_material();
    let md_a = create_box_mesh(VEC3_ZERO, v(1.0, 1.0, 1.0), false).expect("mesh");
    let md_b = create_box_mesh(VEC3_ZERO, v(2.0, 1.0, 1.0), false).expect("mesh");
    let mut meshes = [
        CompoundMeshDef {
            mesh_data: &md_a,
            transform: TRANSFORM_IDENTITY,
            scale: v(1.0, 1.0, 1.0),
            materials: &[mat],
        },
        CompoundMeshDef {
            mesh_data: &md_b,
            transform: TRANSFORM_IDENTITY,
            scale: v(1.0, 1.0, 1.0),
            materials: &[mat],
        },
    ];
    meshes[1].transform.p.x = 5.0;
    let c = create_compound(&CompoundDef {
        meshes: &meshes,
        ..Default::default()
    })
    .unwrap();
    assert_eq!(c.shared_mesh_count, 2);
    destroy_compound(c);
    destroy_mesh(md_a);
    destroy_mesh(md_b);
}

#[test]
fn compound_child_dispatch() {
    let mat = default_surface_material();
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let md = create_box_mesh(VEC3_ZERO, v(0.5, 0.5, 0.5), false).expect("mesh");
    let caps = [
        CompoundCapsuleDef {
            capsule: Capsule {
                center1: VEC3_ZERO,
                center2: v(1.0, 0.0, 0.0),
                radius: 0.2,
            },
            material: mat,
        },
        CompoundCapsuleDef {
            capsule: Capsule {
                center1: v(0.0, 2.0, 0.0),
                center2: v(1.0, 2.0, 0.0),
                radius: 0.2,
            },
            material: mat,
        },
    ];
    let mut hulls = [CompoundHullDef {
        hull: &box_hull.base,
        transform: TRANSFORM_IDENTITY,
        material: mat,
    }];
    hulls[0].transform.p = v(5.0, 0.0, 0.0);
    let mut meshes = [CompoundMeshDef {
        mesh_data: &md,
        transform: TRANSFORM_IDENTITY,
        scale: v(1.0, 1.0, 1.0),
        materials: &[mat],
    }];
    meshes[0].transform.p = v(0.0, 0.0, 5.0);
    let spheres = [CompoundSphereDef {
        sphere: Sphere {
            center: v(-5.0, 0.0, 0.0),
            radius: 0.5,
        },
        material: mat,
    }];
    let c = create_compound(&CompoundDef {
        capsules: &caps,
        hulls: &hulls,
        meshes: &meshes,
        spheres: &spheres,
    })
    .unwrap();

    assert_eq!(get_compound_child(&c, 0).shape_type, ShapeType::Capsule);
    assert_eq!(get_compound_child(&c, 1).shape_type, ShapeType::Capsule);
    assert_eq!(get_compound_child(&c, 2).shape_type, ShapeType::Hull);
    assert_eq!(get_compound_child(&c, 3).shape_type, ShapeType::Mesh);
    assert_eq!(get_compound_child(&c, 4).shape_type, ShapeType::Sphere);

    let cap0 = get_compound_child(&c, 0);
    assert!(cap0.transform.p.x.abs() < f32::EPSILON);
    if let ChildGeometry::Capsule(cap) = cap0.geometry {
        assert!((cap.center2.x - 1.0).abs() < f32::EPSILON);
    } else {
        panic!("expected capsule");
    }
    let sph = get_compound_child(&c, 4);
    assert!(sph.transform.p.x.abs() < f32::EPSILON);
    if let ChildGeometry::Sphere(s) = sph.geometry {
        assert!((s.center.x + 5.0).abs() < f32::EPSILON);
    } else {
        panic!("expected sphere");
    }
    assert!((get_compound_child(&c, 2).transform.p.x - 5.0).abs() < f32::EPSILON);
    let mesh = get_compound_child(&c, 3);
    assert!((mesh.transform.p.z - 5.0).abs() < f32::EPSILON);
    assert!(matches!(mesh.geometry, ChildGeometry::Mesh(_)));
    destroy_compound(c);
    destroy_mesh(md);
}
