//! Corrupt-blob tests for the geometry deserializers.
//!
//! These cover a deliberate Rust-side divergence: the C converters
//! (`b3ConvertBytesToHull` / `...Mesh` / `...HeightField` / `...Compound`) cast the
//! stored section offsets straight to pointers and trust the blob, so a corrupt file
//! makes them read garbage. The Rust port validates the offsets instead and returns
//! `None`, because the unchecked `offset as usize` arithmetic would otherwise wrap:
//! that panics on the overflow itself in debug builds, and on the resulting slice
//! bounds check in release builds. Either way the process dies on a corrupt file.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::compound::{
    convert_bytes_to_compound, convert_compound_to_bytes, create_compound, destroy_compound,
    CompoundCapsuleDef, CompoundDef, CompoundHullDef, CompoundMeshDef, CompoundSphereDef,
};
use crate::geometry::{default_surface_material, Capsule, Sphere};
use crate::height_field::{convert_bytes_to_height_field, create_height_field, HeightFieldDef};
use crate::hull::{convert_bytes_to_hull, make_box_hull};
use crate::math_functions::{Transform, Vec3, QUAT_IDENTITY, VEC3_ZERO};
use crate::mesh::{convert_bytes_to_mesh, create_box_mesh, create_grid_mesh, destroy_mesh};

fn v(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

/// Overwrite a little-endian i32 header field in a blob copy.
fn with_i32(bytes: &[u8], offset: usize, value: i32) -> Vec<u8> {
    let mut buf = bytes.to_vec();
    buf[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    buf
}

/// Every section offset in the header, poisoned negative and then oversized, must be
/// rejected rather than wrapping the index arithmetic.
fn assert_offsets_validated(bytes: &[u8], offsets: &[usize], convert: impl Fn(&[u8]) -> bool) {
    assert!(convert(bytes), "baseline blob must deserialize");
    for &offset in offsets {
        assert!(
            !convert(&with_i32(bytes, offset, -16)),
            "negative offset at header byte {offset} was accepted"
        );
        assert!(
            !convert(&with_i32(bytes, offset, i32::MAX)),
            "oversized offset at header byte {offset} was accepted"
        );
    }
}

#[test]
fn mesh_blob_section_offsets_validated() {
    let mesh = create_grid_mesh(4, 4, 2.0, 2, false).expect("mesh");
    let bytes = mesh.to_bytes();
    // node, vertex, triangle, material, flags section offsets.
    assert_offsets_validated(&bytes, &[52, 60, 68, 76, 84], |b| {
        convert_bytes_to_mesh(b).is_some()
    });
    destroy_mesh(mesh);
}

#[test]
fn mesh_blob_negative_counts_rejected() {
    let mesh = create_grid_mesh(4, 4, 2.0, 2, false).expect("mesh");
    let bytes = mesh.to_bytes();
    // node, vertex, triangle, material counts.
    for offset in [56usize, 64, 72, 80] {
        assert!(convert_bytes_to_mesh(&with_i32(&bytes, offset, -1)).is_none());
    }
    destroy_mesh(mesh);
}

#[test]
fn hull_blob_section_offsets_validated() {
    let hull = make_box_hull(0.5, 0.5, 0.5);
    let bytes = hull.base.to_bytes();
    // vertex, point, edge, plane, face, soaVertex, soaNormal section offsets.
    assert_offsets_validated(&bytes, &[104, 108, 116, 124, 128, 132, 136], |b| {
        convert_bytes_to_hull(b).is_some()
    });
}

#[test]
fn height_field_blob_section_offsets_validated() {
    let def = HeightFieldDef {
        heights: vec![0.0, 0.25, 0.5, 0.25, 0.0, -0.25, 0.5, 0.1, 0.0],
        material_indices: vec![0u8; 4],
        scale: v(1.0, 1.0, 1.0),
        count_x: 3,
        count_z: 3,
        global_minimum_height: -1.0,
        global_maximum_height: 1.0,
        clockwise_winding: false,
    };
    let hf = create_height_field(&def);
    let bytes = hf.to_bytes();
    // heights, material, flags section offsets.
    assert_offsets_validated(&bytes, &[72, 76, 80], |b| {
        convert_bytes_to_height_field(b).is_some()
    });
}

/// A compound carrying one child of every kind, serialized. The hull and mesh children
/// are stored as complete nested blobs inside the buffer, which is what the nested-reader
/// tests below poison.
fn serialized_compound() -> Vec<u8> {
    let md = create_box_mesh(VEC3_ZERO, v(0.5, 0.5, 0.5), false).expect("mesh");
    let mat = default_surface_material();
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let compound = create_compound(&CompoundDef {
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
            mesh_data: &md,
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
    .expect("compound");
    let bytes = convert_compound_to_bytes(&compound);
    destroy_compound(compound);
    destroy_mesh(md);
    bytes
}

#[test]
fn compound_blob_section_offsets_validated() {
    let bytes = serialized_compound();

    // nodeOffset (12), then material/capsule/hull/mesh/sphere offsets in the trailing
    // header block that starts at 16 + DYNAMIC_TREE_SIZE (80) = 96.
    assert_offsets_validated(&bytes, &[12, 96, 104, 112, 124, 136], |b| {
        convert_bytes_to_compound(b).is_some()
    });

    // Negative counts must be rejected before they become huge `with_capacity` requests.
    for offset in [40usize, 100, 108, 116, 128, 140] {
        assert!(
            convert_bytes_to_compound(&with_i32(&bytes, offset, -1)).is_none(),
            "negative count at header byte {offset} was accepted"
        );
    }
}

fn read_i32_at(bytes: &[u8], offset: usize) -> i32 {
    i32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

/// Start of the nested hull blob: hullOffset (header byte 112) locates the hull instance
/// array, and each instance stores its shared blob's offset at instance byte 28.
fn nested_hull_start(bytes: &[u8]) -> usize {
    let instances = read_i32_at(bytes, 112) as usize;
    read_i32_at(bytes, instances + 28) as usize
}

/// Start of the nested mesh blob: meshOffset (header byte 124) locates the mesh instance
/// array, and each instance stores its shared blob's offset at instance byte 40.
fn nested_mesh_start(bytes: &[u8]) -> usize {
    let instances = read_i32_at(bytes, 124) as usize;
    read_i32_at(bytes, instances + 40) as usize
}

/// The hull blob nested inside a compound is sliced out by `sub_blob`, which only bounds its
/// outer extent, and then parsed by `convert_bytes_to_hull`. Its own section offsets and
/// counts have to be validated too, or a corrupt compound panics inside the nested parse.
#[test]
fn compound_nested_hull_blob_validated() {
    let bytes = serialized_compound();
    let base = nested_hull_start(&bytes);
    assert!(convert_bytes_to_compound(&bytes).is_some());

    // vertex, point, edge, plane, face, soaVertex, soaNormal section offsets.
    for offset in [104usize, 108, 116, 124, 128, 132, 136] {
        for poison in [-16, i32::MAX] {
            assert!(
                convert_bytes_to_compound(&with_i32(&bytes, base + offset, poison)).is_none(),
                "nested hull offset at blob byte {offset} accepted poison {poison}"
            );
        }
    }
    // vertex, edge, face counts.
    for offset in [100usize, 112, 120] {
        assert!(
            convert_bytes_to_compound(&with_i32(&bytes, base + offset, -1)).is_none(),
            "nested hull negative count at blob byte {offset} was accepted"
        );
    }
}

/// Same for the mesh blob nested inside a compound, parsed by `convert_bytes_to_mesh`.
#[test]
fn compound_nested_mesh_blob_validated() {
    let bytes = serialized_compound();
    let base = nested_mesh_start(&bytes);
    assert!(convert_bytes_to_compound(&bytes).is_some());

    // node, vertex, triangle, material, flags section offsets.
    for offset in [52usize, 60, 68, 76, 84] {
        for poison in [-16, i32::MAX] {
            assert!(
                convert_bytes_to_compound(&with_i32(&bytes, base + offset, poison)).is_none(),
                "nested mesh offset at blob byte {offset} accepted poison {poison}"
            );
        }
    }
    // node, vertex, triangle, material counts.
    for offset in [56usize, 64, 72, 80] {
        assert!(
            convert_bytes_to_compound(&with_i32(&bytes, base + offset, -1)).is_none(),
            "nested mesh negative count at blob byte {offset} was accepted"
        );
    }
}
