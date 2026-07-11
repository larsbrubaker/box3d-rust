//! Create and material-sharing compound tests.

use super::{make_material, v};
use crate::compound::{
    create_compound, destroy_compound, get_compound_capsule, get_compound_hull,
    get_compound_materials, get_compound_sphere, CompoundCapsuleDef, CompoundDef, CompoundHullDef,
    CompoundMeshDef, CompoundSphereDef, COMPOUND_VERSION,
};
use crate::geometry::{default_surface_material, Capsule, Sphere};
use crate::hull::make_box_hull;
use crate::math_functions::{TRANSFORM_IDENTITY, VEC3_ZERO};
use crate::mesh::{create_box_mesh, destroy_mesh};

#[test]
fn compound_create_mixed() {
    let mat = default_surface_material();
    let capsules = [CompoundCapsuleDef {
        capsule: Capsule {
            center1: v(-1.0, 0.0, 0.0),
            center2: v(1.0, 0.0, 0.0),
            radius: 0.25,
        },
        material: mat,
    }];
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let hulls = [CompoundHullDef {
        hull: &box_hull.base,
        transform: TRANSFORM_IDENTITY,
        material: mat,
    }];
    let mesh_data = create_box_mesh(VEC3_ZERO, v(0.5, 0.5, 0.5), false).expect("mesh");
    let meshes = [CompoundMeshDef {
        mesh_data: &mesh_data,
        transform: TRANSFORM_IDENTITY,
        scale: v(1.0, 1.0, 1.0),
        materials: &[mat],
    }];
    let spheres = [
        CompoundSphereDef {
            sphere: Sphere {
                center: v(5.0, 0.0, 0.0),
                radius: 0.5,
            },
            material: mat,
        },
        CompoundSphereDef {
            sphere: Sphere {
                center: v(-5.0, 0.0, 0.0),
                radius: 0.5,
            },
            material: mat,
        },
    ];
    let def = CompoundDef {
        capsules: &capsules,
        hulls: &hulls,
        meshes: &meshes,
        spheres: &spheres,
    };
    let compound = create_compound(&def).expect("create");
    assert_eq!(compound.version, COMPOUND_VERSION);
    assert!(compound.byte_count > 144);
    assert_eq!(compound.capsule_count, 1);
    assert_eq!(compound.hull_count, 1);
    assert_eq!(compound.mesh_count, 1);
    assert_eq!(compound.sphere_count, 2);
    assert_eq!(compound.material_count, 1);
    assert_eq!(compound.shared_hull_count, 1);
    assert_eq!(compound.shared_mesh_count, 1);
    assert!(compound.tree.node_count() > 0);
    assert!(!compound.tree.nodes.is_empty());
    destroy_compound(compound);
    destroy_mesh(mesh_data);
}

#[test]
fn compound_create_single_type() {
    let mat = default_surface_material();
    {
        let cap = CompoundCapsuleDef {
            capsule: Capsule {
                center1: VEC3_ZERO,
                center2: v(1.0, 0.0, 0.0),
                radius: 0.5,
            },
            material: mat,
        };
        let c = create_compound(&CompoundDef {
            capsules: &[cap],
            ..Default::default()
        })
        .unwrap();
        assert_eq!(
            (c.capsule_count, c.hull_count, c.mesh_count, c.sphere_count),
            (1, 0, 0, 0)
        );
        destroy_compound(c);
    }
    {
        let box_hull = make_box_hull(1.0, 1.0, 1.0);
        let h = CompoundHullDef {
            hull: &box_hull.base,
            transform: TRANSFORM_IDENTITY,
            material: mat,
        };
        let c = create_compound(&CompoundDef {
            hulls: &[h],
            ..Default::default()
        })
        .unwrap();
        assert_eq!(c.hull_count, 1);
        assert_eq!(c.shared_hull_count, 1);
        assert_eq!(c.capsule_count, 0);
        destroy_compound(c);
    }
    {
        let md = create_box_mesh(VEC3_ZERO, v(1.0, 1.0, 1.0), false).expect("mesh");
        let m = CompoundMeshDef {
            mesh_data: &md,
            transform: TRANSFORM_IDENTITY,
            scale: v(1.0, 1.0, 1.0),
            materials: &[mat],
        };
        let c = create_compound(&CompoundDef {
            meshes: &[m],
            ..Default::default()
        })
        .unwrap();
        assert_eq!(c.mesh_count, 1);
        assert_eq!(c.shared_mesh_count, 1);
        destroy_compound(c);
        destroy_mesh(md);
    }
    {
        let s = CompoundSphereDef {
            sphere: Sphere {
                center: VEC3_ZERO,
                radius: 1.0,
            },
            material: mat,
        };
        let c = create_compound(&CompoundDef {
            spheres: &[s],
            ..Default::default()
        })
        .unwrap();
        assert_eq!(c.sphere_count, 1);
        destroy_compound(c);
    }
}

#[test]
fn compound_material_dedup() {
    let mat = make_material(0.4, 7);
    let caps: Vec<_> = (0..3)
        .map(|i| CompoundCapsuleDef {
            capsule: Capsule {
                center1: v(i as f32, 0.0, 0.0),
                center2: v(i as f32 + 1.0, 0.0, 0.0),
                radius: 0.25,
            },
            material: mat,
        })
        .collect();
    let c = create_compound(&CompoundDef {
        capsules: &caps,
        ..Default::default()
    })
    .unwrap();
    assert_eq!(c.material_count, 1);
    for i in 0..3 {
        assert_eq!(get_compound_capsule(&c, i).material_index, 0);
    }
    destroy_compound(c);
}

#[test]
fn compound_material_distinct() {
    let caps: Vec<_> = (0..3)
        .map(|i| CompoundCapsuleDef {
            capsule: Capsule {
                center1: v(i as f32, 0.0, 0.0),
                center2: v(i as f32 + 1.0, 0.0, 0.0),
                radius: 0.25,
            },
            material: make_material(0.1 * (i as f32 + 1.0), (i + 1) as u64),
        })
        .collect();
    let c = create_compound(&CompoundDef {
        capsules: &caps,
        ..Default::default()
    })
    .unwrap();
    assert_eq!(c.material_count, 3);
    let mats = get_compound_materials(&c);
    for i in 0..3 {
        let cc = get_compound_capsule(&c, i);
        assert!((0..3).contains(&cc.material_index));
        assert_eq!(
            mats[cc.material_index as usize].user_material_id,
            (i + 1) as u64
        );
    }
    destroy_compound(c);
}

#[test]
fn compound_material_cross_shape() {
    let mat = make_material(0.5, 99);
    let cap = CompoundCapsuleDef {
        capsule: Capsule {
            center1: VEC3_ZERO,
            center2: v(1.0, 0.0, 0.0),
            radius: 0.25,
        },
        material: mat,
    };
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let hull = CompoundHullDef {
        hull: &box_hull.base,
        transform: TRANSFORM_IDENTITY,
        material: mat,
    };
    let sph = CompoundSphereDef {
        sphere: Sphere {
            center: v(5.0, 0.0, 0.0),
            radius: 0.5,
        },
        material: mat,
    };
    let c = create_compound(&CompoundDef {
        capsules: &[cap],
        hulls: &[hull],
        spheres: &[sph],
        ..Default::default()
    })
    .unwrap();
    assert_eq!(c.material_count, 1);
    assert_eq!(get_compound_capsule(&c, 0).material_index, 0);
    assert_eq!(get_compound_hull(&c, 0).material_index, 0);
    assert_eq!(get_compound_sphere(&c, 0).material_index, 0);
    destroy_compound(c);
}

#[test]
fn compound_material_mesh_shared() {
    let mat = make_material(0.3, 11);
    let md = create_box_mesh(VEC3_ZERO, v(1.0, 1.0, 1.0), false).expect("mesh");
    assert_eq!(md.material_count, 1);
    let sph = CompoundSphereDef {
        sphere: Sphere {
            center: v(5.0, 0.0, 0.0),
            radius: 0.5,
        },
        material: mat,
    };
    let mesh = CompoundMeshDef {
        mesh_data: &md,
        transform: TRANSFORM_IDENTITY,
        scale: v(1.0, 1.0, 1.0),
        materials: &[mat],
    };
    let c = create_compound(&CompoundDef {
        meshes: &[mesh],
        spheres: &[sph],
        ..Default::default()
    })
    .unwrap();
    assert_eq!(c.material_count, 1);
    destroy_compound(c);
    destroy_mesh(md);
}
