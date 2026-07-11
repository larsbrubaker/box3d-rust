//! Compound serialization roundtrip tests.

use super::v;
use crate::compound::{
    compute_compound_aabb, convert_bytes_to_compound, convert_compound_to_bytes, create_compound,
    destroy_compound, ray_cast_compound, CompoundCapsuleDef, CompoundData, CompoundDef,
    CompoundHullDef, CompoundMeshDef, CompoundSphereDef,
};
use crate::geometry::{default_surface_material, Capsule, RayCastInput, Sphere};
use crate::hull::make_box_hull;
use crate::math_functions::{Transform, QUAT_IDENTITY, TRANSFORM_IDENTITY, VEC3_ZERO};
use crate::mesh::{create_box_mesh, destroy_mesh, MeshData};

fn build_serializable_compound(md: &MeshData) -> CompoundData {
    let mat = default_surface_material();
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let cap = CompoundCapsuleDef {
        capsule: Capsule {
            center1: v(-2.0, 0.0, 0.0),
            center2: v(-1.0, 0.0, 0.0),
            radius: 0.2,
        },
        material: mat,
    };
    let hull = CompoundHullDef {
        hull: &box_hull.base,
        transform: Transform {
            p: v(5.0, 0.0, 0.0),
            q: QUAT_IDENTITY,
        },
        material: mat,
    };
    let mesh = CompoundMeshDef {
        mesh_data: md,
        transform: Transform {
            p: v(0.0, 0.0, 5.0),
            q: QUAT_IDENTITY,
        },
        scale: v(1.0, 1.0, 1.0),
        materials: &[mat],
    };
    let sph = CompoundSphereDef {
        sphere: Sphere {
            center: v(-5.0, 0.0, 0.0),
            radius: 0.5,
        },
        material: mat,
    };
    create_compound(&CompoundDef {
        capsules: &[cap],
        hulls: &[hull],
        meshes: &[mesh],
        spheres: &[sph],
    })
    .unwrap()
}

#[test]
fn compound_serialize_roundtrip() {
    let md = create_box_mesh(VEC3_ZERO, v(0.5, 0.5, 0.5), false).expect("mesh");
    let a = build_serializable_compound(&md);
    let aabb_a = compute_compound_aabb(&a, TRANSFORM_IDENTITY);
    let ray_input = RayCastInput {
        origin: VEC3_ZERO,
        translation: v(20.0, 0.0, 0.0),
        max_fraction: 1.0,
    };
    let ray_a = ray_cast_compound(&a, &ray_input);
    assert!(ray_a.hit);
    let byte_count = a.byte_count;
    let buffer = convert_compound_to_bytes(&a);
    destroy_compound(a);
    let b = convert_bytes_to_compound(&buffer).expect("deserialize");
    assert_eq!(b.byte_count, byte_count);
    assert_eq!(
        (b.capsule_count, b.hull_count, b.mesh_count, b.sphere_count),
        (1, 1, 1, 1)
    );
    assert!(!b.tree.nodes.is_empty());
    let aabb_b = compute_compound_aabb(&b, TRANSFORM_IDENTITY);
    assert!((aabb_a.lower_bound.x - aabb_b.lower_bound.x).abs() < 1e-5);
    assert!((aabb_a.upper_bound.z - aabb_b.upper_bound.z).abs() < 1e-5);
    let ray_b = ray_cast_compound(&b, &ray_input);
    assert_eq!(ray_b.hit, ray_a.hit);
    assert!((ray_b.fraction - ray_a.fraction).abs() < 1e-5);
    assert_eq!(ray_b.child_index, ray_a.child_index);
    destroy_mesh(md);
}

#[test]
fn compound_serialize_bad_version() {
    let md = create_box_mesh(VEC3_ZERO, v(0.5, 0.5, 0.5), false).expect("mesh");
    let a = build_serializable_compound(&md);
    let mut buffer = convert_compound_to_bytes(&a);
    destroy_compound(a);
    buffer[0] ^= 1;
    assert!(convert_bytes_to_compound(&buffer).is_none());
    destroy_mesh(md);
}

#[test]
fn compound_serialize_wrong_byte_count() {
    let md = create_box_mesh(VEC3_ZERO, v(0.5, 0.5, 0.5), false).expect("mesh");
    let a = build_serializable_compound(&md);
    let mut buffer = convert_compound_to_bytes(&a);
    destroy_compound(a);
    buffer.push(0);
    assert!(convert_bytes_to_compound(&buffer).is_none());
    destroy_mesh(md);
}
