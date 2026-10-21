//! Compound blob layout checks against the scalar C build of upstream 5d83df8 (MSVC,
//! `BOX3D_DISABLE_SIMD`). Every expected value below was printed by that build, in float and
//! double precision alike (the compound blob holds no `b3Pos`), and the full blobs were also
//! compared byte for byte.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::v;
use crate::compound::{
    compute_compound_aabb, create_compound, ray_cast_compound, validate_compound,
    CompoundCapsuleDef, CompoundData, CompoundDef, CompoundHullDef, CompoundMeshDef,
    CompoundSphereDef, COMPOUND_DATA_SIZE, COMPOUND_VERSION,
};
use crate::core::hash64_non_zero;
use crate::geometry::{default_surface_material, Capsule, RayCastInput, Sphere};
use crate::hull::make_box_hull;
use crate::math_functions::{Transform, QUAT_IDENTITY, TRANSFORM_IDENTITY, VEC3_ZERO};
use crate::mesh::{create_box_mesh, MeshData};

/// One capsule and three spheres (the recording tests' compound).
fn rec_compound() -> CompoundData {
    let m = default_surface_material();
    let capsules = [CompoundCapsuleDef {
        capsule: Capsule {
            center1: v(-0.5, 0.0, 0.0),
            center2: v(0.5, 0.0, 0.0),
            radius: 0.25,
        },
        material: m,
    }];
    let sphere = |x: f32, y: f32, z: f32, radius: f32| CompoundSphereDef {
        sphere: Sphere {
            center: v(x, y, z),
            radius,
        },
        material: m,
    };
    let spheres = [
        sphere(0.0, 1.0, 0.0, 0.5),
        sphere(1.0, 0.0, 1.0, 0.3),
        sphere(-1.0, 0.5, -1.0, 0.4),
    ];
    create_compound(&CompoundDef {
        capsules: &capsules,
        spheres: &spheres,
        ..Default::default()
    })
    .unwrap()
}

/// A single unit sphere: the tree is a lone root leaf.
fn sphere_compound() -> CompoundData {
    create_compound(&CompoundDef {
        spheres: &[CompoundSphereDef {
            sphere: Sphere {
                center: VEC3_ZERO,
                radius: 1.0,
            },
            material: default_surface_material(),
        }],
        ..Default::default()
    })
    .unwrap()
}

/// One child of every kind. (BuildSerializableCompound)
fn serializable_compound(md: &MeshData) -> CompoundData {
    let mat = default_surface_material();
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    create_compound(&CompoundDef {
        capsules: &[CompoundCapsuleDef {
            capsule: Capsule {
                center1: v(-2.0, 0.0, 0.0),
                center2: v(-1.0, 0.0, 0.0),
                radius: 0.2,
            },
            material: mat,
        }],
        hulls: &[CompoundHullDef {
            hull: &box_hull.base,
            transform: Transform {
                p: v(5.0, 0.0, 0.0),
                q: QUAT_IDENTITY,
            },
            material: mat,
        }],
        meshes: &[CompoundMeshDef {
            mesh_data: md,
            transform: Transform {
                p: v(0.0, 0.0, 5.0),
                q: QUAT_IDENTITY,
            },
            scale: v(1.0, 1.0, 1.0),
            materials: &[mat],
        }],
        spheres: &[CompoundSphereDef {
            sphere: Sphere {
                center: v(-5.0, 0.0, 0.0),
                radius: 0.5,
            },
            material: mat,
        }],
    })
    .unwrap()
}

/// Forty spheres on a grid, so the tree is several levels deep.
fn grid_compound() -> CompoundData {
    let m = default_surface_material();
    let spheres: Vec<CompoundSphereDef> = (0..40)
        .map(|i| CompoundSphereDef {
            sphere: Sphere {
                center: v(
                    (i % 7) as f32 * 1.5,
                    (i / 7) as f32 * 0.75,
                    ((i * 3) % 5) as f32,
                ),
                radius: 0.25 + 0.01 * i as f32,
            },
            material: m,
        })
        .collect();
    create_compound(&CompoundDef {
        spheres: &spheres,
        ..Default::default()
    })
    .unwrap()
}

/// (byteCount, nodeCount, nodeOffset, treeHeight, b3Hash64NonZero of the blob)
fn summary(c: &CompoundData) -> (i32, i32, i32, i32, u64) {
    let bytes = c.to_bytes();
    assert_eq!(bytes.len(), c.byte_count as usize);
    // Every C-built blob passes validation, including the port's node checks.
    assert!(validate_compound(&bytes).is_some());
    (
        c.byte_count,
        c.node_count,
        c.node_offset,
        c.tree_height,
        hash64_non_zero(&bytes),
    )
}

#[test]
fn compound_version_and_header_size_match_c() {
    assert_eq!(COMPOUND_DATA_SIZE, 96);
    assert_eq!(COMPOUND_VERSION, 0x5ffe_b407_fb88_e7f5);
}

#[test]
fn compound_blobs_match_c() {
    assert_eq!(
        summary(&rec_compound()),
        (488, 8, 96, 3, 0x58b8_26b8_d7e0_7710)
    );
    assert_eq!(
        summary(&sphere_compound()),
        (224, 2, 96, 0, 0x37a2_9941_e13e_6541)
    );
    let md = create_box_mesh(VEC3_ZERO, v(0.5, 0.5, 0.5), false).unwrap();
    assert_eq!(
        summary(&serializable_compound(&md)),
        (1904, 8, 96, 2, 0x4209_07e5_bc18_1f1a)
    );
    assert_eq!(
        summary(&grid_compound()),
        (3496, 80, 96, 7, 0x3786_f126_26a3_4b8f)
    );
}

/// The ray cast and bounds run through the compound tree view (no proxies).
#[test]
fn compound_tree_view_queries_match_c() {
    let grid = grid_compound();
    let output = ray_cast_compound(
        &grid,
        &RayCastInput {
            origin: v(-1.0, 0.5, 2.0),
            translation: v(12.0, 0.0, 0.0),
            max_fraction: 1.0,
        },
    );
    assert!(output.hit);
    assert_eq!(output.fraction.to_bits(), 0x3ea0_d5b5);
    assert_eq!(output.child_index, 9);

    let aabb = compute_compound_aabb(&grid, TRANSFORM_IDENTITY);
    let bits = [
        aabb.lower_bound.x.to_bits(),
        aabb.lower_bound.y.to_bits(),
        aabb.lower_bound.z.to_bits(),
        aabb.upper_bound.x.to_bits(),
        aabb.upper_bound.y.to_bits(),
        aabb.upper_bound.z.to_bits(),
    ];
    assert_eq!(
        bits,
        [0xbf1999a0, 0xbe9eb850, 0xbf199998, 0x411970a4, 0x408c7ae1, 0x409428f6]
    );
}

/// The header checks of b3ValidateCompound, with the outcomes the C build reports.
#[test]
fn validate_compound_header_checks_match_c() {
    let md = create_box_mesh(VEC3_ZERO, v(0.5, 0.5, 0.5), false).unwrap();
    let bytes = serializable_compound(&md).to_bytes();
    let with_i32 = |offset: usize, value: i32| {
        let mut b = bytes.clone();
        b[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        b
    };
    const NODE_OFFSET: usize = 40;
    const NODE_COUNT: usize = 44;

    assert!(validate_compound(&bytes).is_some());

    let mut longer = bytes.clone();
    longer.push(0);
    assert!(validate_compound(&longer).is_none());

    assert!(validate_compound(&with_i32(NODE_COUNT, bytes.len() as i32)).is_none());
    assert!(validate_compound(&with_i32(NODE_COUNT, i32::MAX)).is_none());
    assert!(validate_compound(&with_i32(NODE_COUNT, 1)).is_none());

    // Misaligned, and inside the header
    assert!(validate_compound(&with_i32(NODE_OFFSET, 96 + 4)).is_none());
    assert!(validate_compound(&with_i32(NODE_OFFSET, 88)).is_none());

    // Shorter than the header
    assert!(validate_compound(&bytes[..COMPOUND_DATA_SIZE - 1]).is_none());
}
