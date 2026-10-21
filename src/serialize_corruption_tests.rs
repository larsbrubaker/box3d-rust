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
    create_compound, destroy_compound, validate_compound, CompoundCapsuleDef, CompoundDef,
    CompoundHullDef, CompoundMeshDef, CompoundSphereDef,
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
    assert_offsets_validated(&bytes, &[56, 64, 72, 80, 88], |b| {
        convert_bytes_to_mesh(b).is_some()
    });
    destroy_mesh(mesh);
}

#[test]
fn mesh_blob_negative_counts_rejected() {
    let mesh = create_grid_mesh(4, 4, 2.0, 2, false).expect("mesh");
    let bytes = mesh.to_bytes();
    // node, vertex, triangle, material counts.
    for offset in [60usize, 68, 76, 84] {
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
    assert_offsets_validated(&bytes, &[76, 80, 84], |b| {
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
    let bytes = compound.to_bytes();
    destroy_compound(compound);
    destroy_mesh(md);
    bytes
}

#[test]
fn compound_blob_section_offsets_validated() {
    let bytes = serialized_compound();

    // nodeOffset (40), then the material/capsule/hull/mesh/sphere offsets (48, 56, 64, 76, 88).
    assert_offsets_validated(&bytes, &[40, 48, 56, 64, 76, 88], |b| {
        validate_compound(b).is_some()
    });

    // Negative counts must be rejected before they become huge `with_capacity` requests.
    // nodeCount (44), then the material/capsule/hull/mesh/sphere counts.
    for offset in [44usize, 52, 60, 68, 80, 92] {
        assert!(
            validate_compound(&with_i32(&bytes, offset, -1)).is_none(),
            "negative count at header byte {offset} was accepted"
        );
    }
}

fn read_i32_at(bytes: &[u8], offset: usize) -> i32 {
    i32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

/// Start of the nested hull blob: hullOffset (header byte 64) locates the hull instance
/// array, and each instance stores its shared blob's offset at instance byte 28.
fn nested_hull_start(bytes: &[u8]) -> usize {
    let instances = read_i32_at(bytes, 64) as usize;
    read_i32_at(bytes, instances + 28) as usize
}

/// Start of the nested mesh blob: meshOffset (header byte 76) locates the mesh instance
/// array, and each instance stores its shared blob's offset at instance byte 40.
fn nested_mesh_start(bytes: &[u8]) -> usize {
    let instances = read_i32_at(bytes, 76) as usize;
    read_i32_at(bytes, instances + 40) as usize
}

/// The hull blob nested inside a compound is sliced out by `sub_blob`, which only bounds its
/// outer extent, and then parsed by `convert_bytes_to_hull`. Its own section offsets and
/// counts have to be validated too, or a corrupt compound panics inside the nested parse.
#[test]
fn compound_nested_hull_blob_validated() {
    let bytes = serialized_compound();
    let base = nested_hull_start(&bytes);
    assert!(validate_compound(&bytes).is_some());

    // vertex, point, edge, plane, face, soaVertex, soaNormal section offsets.
    for offset in [104usize, 108, 116, 124, 128, 132, 136] {
        for poison in [-16, i32::MAX] {
            assert!(
                validate_compound(&with_i32(&bytes, base + offset, poison)).is_none(),
                "nested hull offset at blob byte {offset} accepted poison {poison}"
            );
        }
    }
    // vertex, edge, face counts.
    for offset in [100usize, 112, 120] {
        assert!(
            validate_compound(&with_i32(&bytes, base + offset, -1)).is_none(),
            "nested hull negative count at blob byte {offset} was accepted"
        );
    }
}

/// Same for the mesh blob nested inside a compound, parsed by `convert_bytes_to_mesh`.
#[test]
fn compound_nested_mesh_blob_validated() {
    let bytes = serialized_compound();
    let base = nested_mesh_start(&bytes);
    assert!(validate_compound(&bytes).is_some());

    // node, vertex, triangle, material, flags section offsets.
    for offset in [56usize, 64, 72, 80, 88] {
        for poison in [-16, i32::MAX] {
            assert!(
                validate_compound(&with_i32(&bytes, base + offset, poison)).is_none(),
                "nested mesh offset at blob byte {offset} accepted poison {poison}"
            );
        }
    }
    // node, vertex, triangle, material counts.
    for offset in [60usize, 68, 76, 84] {
        assert!(
            validate_compound(&with_i32(&bytes, base + offset, -1)).is_none(),
            "nested mesh negative count at blob byte {offset} was accepted"
        );
    }
}

/// Byte image of the world fat AABB section: the count followed by each AABB.
fn fat_aabb_section(world: &crate::world::World) -> Vec<u8> {
    let mut buf = crate::recording::RecBuffer::new();
    buf.append_i32(world.fat_aabbs.len() as i32);
    for aabb in &world.fat_aabbs {
        buf.append_aabb(*aabb);
    }
    buf.data
}

/// The fat AABB snapshot section parallels the shape array. An image whose section count does
/// not match the shape count must be rejected, even when the rest of the stream stays aligned.
#[test]
fn snapshot_fat_aabb_count_mismatch_rejected() {
    use crate::body::create_body;
    use crate::recording::{deserialize_into_shell, serialize_world, GeometryRegistry, RecBuffer};
    use crate::shape::{create_hull_shape, create_sphere_shape};
    use crate::types::{default_body_def, default_shape_def, default_world_def, BodyType};
    use crate::world::World;

    let mut world = World::new(&default_world_def());
    let ground = create_body(&mut world, &default_body_def());
    let ground_box = make_box_hull(10.0, 1.0, 10.0);
    create_hull_shape(&mut world, ground, &default_shape_def(), &ground_box.base);
    for i in 0..3 {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position.x = (2.0 * i as f32) as _;
        body_def.position.y = 3.0 as _;
        let body = create_body(&mut world, &body_def);
        let sphere = Sphere {
            center: VEC3_ZERO,
            radius: 0.5,
        };
        create_sphere_shape(&mut world, body, &default_shape_def(), &sphere);
    }
    world.step(1.0 / 60.0, 4);

    let mut registry = GeometryRegistry::new();
    let mut buf = RecBuffer::new();
    serialize_world(&world, &mut buf, &mut registry);
    let bytes = buf.data;

    // Locate the fat AABB section in the image
    let section = fat_aabb_section(&world);
    let start = bytes
        .windows(section.len())
        .position(|w| w == section.as_slice())
        .expect("fat AABB section present");
    assert!(
        bytes[start + section.len()..]
            .windows(section.len())
            .all(|w| w != section.as_slice()),
        "fat AABB section must be unique"
    );

    let restore = |image: &[u8]| {
        let mut slots = registry.to_slots();
        let mut shell = World::new(&default_world_def());
        deserialize_into_shell(image, &mut shell, &mut slots)
    };

    // Baseline image restores
    assert!(restore(&bytes));

    // Drop the last AABB and its count: the stream stays aligned but no longer parallels the
    // shape array
    let shape_count = world.shapes.len();
    let mut short = bytes.clone();
    short[start..start + 4].copy_from_slice(&((shape_count - 1) as i32).to_le_bytes());
    let last = start + section.len() - 24;
    short.drain(last..last + 24);
    assert!(!restore(&short), "short fat AABB section was accepted");

    // One extra AABB
    let mut long = bytes.clone();
    long[start..start + 4].copy_from_slice(&((shape_count + 1) as i32).to_le_bytes());
    let end = start + section.len();
    let extra: Vec<u8> = long[end - 24..end].to_vec();
    long.splice(end..end, extra);
    assert!(!restore(&long), "long fat AABB section was accepted");
}

/// Each mesh instance points at its run of the packed `uint16_t` material map with
/// materialOffset (instance byte 44) and materialCount (instance byte 48). A run that leaves
/// the blob must fail the conversion instead of panicking on the slice.
#[test]
fn compound_mesh_material_map_validated() {
    let bytes = serialized_compound();
    assert!(validate_compound(&bytes).is_some());
    let instances = read_i32_at(&bytes, 76) as usize;

    for poison in [-16, i32::MAX] {
        assert!(
            validate_compound(&with_i32(&bytes, instances + 44, poison)).is_none(),
            "mesh material offset accepted poison {poison}"
        );
    }
    for poison in [-1, i32::MAX] {
        assert!(
            validate_compound(&with_i32(&bytes, instances + 48, poison)).is_none(),
            "mesh material count accepted poison {poison}"
        );
    }
}

/// Overwrite a little-endian u16 field in a blob copy.
fn with_u16(bytes: &[u8], offset: usize, value: u16) -> Vec<u8> {
    let mut buf = bytes.to_vec();
    buf[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
    buf
}

/// A mesh instance's material run must lie inside the packed map section, which runs from
/// the end of the mesh instance array to the first shared mesh blob. A run pointing back into
/// the instance array, into a mesh blob, or overrunning the section is rejected even though
/// it stays inside the buffer.
#[test]
fn compound_mesh_material_run_inside_map_section() {
    let bytes = serialized_compound();
    assert!(validate_compound(&bytes).is_some());
    let instances = read_i32_at(&bytes, 76) as usize;
    let mesh_blob = nested_mesh_start(&bytes);

    for offset in [instances as i32, mesh_blob as i32] {
        assert!(
            validate_compound(&with_i32(&bytes, instances + 44, offset)).is_none(),
            "mesh material run at blob byte {offset} outside the map section was accepted"
        );
    }

    // The map section holds alignUp8( 1 * 2 ) = 8 bytes; five entries run into the mesh blob.
    assert!(
        validate_compound(&with_i32(&bytes, instances + 48, 5)).is_none(),
        "mesh material run overrunning the map section was accepted"
    );
}

/// The material indices the compound hands out are used as slice indexes (mesh triangle
/// material -> map entry -> compound material table, convex child -> compound material
/// table), so a deserialized compound must not carry an index that falls outside its table.
#[test]
fn compound_material_indices_validated() {
    let bytes = serialized_compound();
    assert!(validate_compound(&bytes).is_some());
    // The compound has one shared material and the mesh child one map entry.
    assert_eq!(read_i32_at(&bytes, 52), 1);

    // Mesh instance with fewer map entries than its mesh has materials
    let instances = read_i32_at(&bytes, 76) as usize;
    assert!(
        validate_compound(&with_i32(&bytes, instances + 48, 0)).is_none(),
        "mesh material count below the mesh material count was accepted"
    );

    // Map entry outside the compound material table
    let map = read_i32_at(&bytes, instances + 44) as usize;
    for value in [1u16, u16::MAX] {
        assert!(
            validate_compound(&with_u16(&bytes, map, value)).is_none(),
            "mesh material map entry {value} was accepted"
        );
    }

    // Convex children: capsule (header 56, byte 28), hull instance (header 64, byte 32),
    // sphere (header 88, byte 16).
    for (header, field) in [(56usize, 28usize), (64, 32), (88, 16)] {
        let at = read_i32_at(&bytes, header) as usize + field;
        for value in [1u16, u16::MAX] {
            assert!(
                validate_compound(&with_u16(&bytes, at, value)).is_none(),
                "convex child material index {value} (header byte {header}) was accepted"
            );
        }
    }
}

/// The tree nodes are used as slice indexes during queries (an internal node's sibling pair, a
/// leaf's child index), so a validated compound must not carry an out of range or backward
/// pointing node. C trusts the node contents; see the divergence note in `validate_compound`.
#[test]
fn compound_tree_nodes_validated() {
    use crate::dynamic_tree::{LEAF_NODE, TREE_NODE_SIZE};

    let bytes = serialized_compound();
    assert!(validate_compound(&bytes).is_some());
    let node_offset = read_i32_at(&bytes, 40) as usize;
    let node_count = read_i32_at(&bytes, 44) as usize;
    // b3TreeNode: the 24 byte AABB, then flagIndex, then the height / shapeIndex union.
    let flag = |i: usize| node_offset + i * TREE_NODE_SIZE + 24;
    let shape_index = |i: usize| node_offset + i * TREE_NODE_SIZE + 28;

    // The root of this four child compound is internal.
    let root_flag = read_i32_at(&bytes, flag(0)) as u32;
    assert_eq!(root_flag & LEAF_NODE, 0);

    // Root pointing at itself (a cycle) or past the node array
    for pair in [0u32, node_count as u32, node_count as u32 - 1] {
        assert!(
            validate_compound(&with_i32(&bytes, flag(0), pair as i32)).is_none(),
            "root sibling pair {pair} was accepted"
        );
    }

    // A leaf naming a child the compound does not have
    let leaf = (0..node_count)
        .find(|&i| {
            let f = read_i32_at(&bytes, flag(i)) as u32;
            f & LEAF_NODE != 0 && f != crate::dynamic_tree::EMPTY_NODE
        })
        .expect("leaf");
    for child in [-1, 4, i32::MAX] {
        assert!(
            validate_compound(&with_i32(&bytes, shape_index(leaf), child)).is_none(),
            "leaf child index {child} was accepted"
        );
    }
}

/// An empty node also carries the leaf bit, and the traversals treat it as a leaf when its AABB
/// overlaps, so a node relabeled empty must still name an existing child. A corrupt blob that
/// keeps the leaf's bounds and points it past the children is rejected.
#[test]
fn compound_empty_node_child_validated() {
    use crate::dynamic_tree::{EMPTY_NODE, LEAF_NODE, TREE_NODE_SIZE};

    let bytes = serialized_compound();
    let node_offset = read_i32_at(&bytes, 40) as usize;
    let node_count = read_i32_at(&bytes, 44) as usize;
    let flag = |i: usize| node_offset + i * TREE_NODE_SIZE + 24;
    let shape_index = |i: usize| node_offset + i * TREE_NODE_SIZE + 28;

    let leaf = (0..node_count)
        .find(|&i| {
            let f = read_i32_at(&bytes, flag(i)) as u32;
            f & LEAF_NODE != 0 && f != EMPTY_NODE
        })
        .expect("leaf");
    let corrupt = with_i32(
        &with_i32(&bytes, flag(leaf), EMPTY_NODE as i32),
        shape_index(leaf),
        1000,
    );
    assert!(
        validate_compound(&corrupt).is_none(),
        "empty node naming child 1000 was accepted"
    );

    // An empty node naming an existing child is harmless and still validates.
    let harmless = with_i32(&bytes, flag(leaf), EMPTY_NODE as i32);
    assert!(validate_compound(&harmless).is_some());
}

/// Rewrite a compound's node array into a chain: each pair holds a leaf and the internal node
/// for the next pair, `depth` internal levels below the root. Unused nodes become empty.
fn chain_tree_blob(bytes: &[u8], depth: usize) -> Vec<u8> {
    use crate::dynamic_tree::{EMPTY_NODE, LEAF_NODE, TREE_NODE_SIZE};

    let node_offset = read_i32_at(bytes, 40) as usize;
    let node_count = read_i32_at(bytes, 44) as usize;
    assert!(2 * depth + 2 <= node_count);
    let root_aabb = bytes[node_offset..node_offset + 24].to_vec();
    let mut out = bytes.to_vec();
    let mut set = |i: usize, flag: u32, shape: i32| {
        let o = node_offset + i * TREE_NODE_SIZE;
        out[o..o + 24].copy_from_slice(&root_aabb);
        out[o + 24..o + 28].copy_from_slice(&flag.to_le_bytes());
        out[o + 28..o + 32].copy_from_slice(&shape.to_le_bytes());
    };
    // Root at depth 0 points at pair 2.
    set(0, 2, 1);
    set(1, EMPTY_NODE, 0);
    for k in 1..=depth {
        let pair = 2 * k;
        set(pair, LEAF_NODE | k as u32, 0);
        if k < depth {
            set(pair + 1, (pair + 2) as u32, 1);
        } else {
            set(pair + 1, LEAF_NODE, 0);
        }
    }
    for i in 2 * depth + 2..node_count {
        set(i, EMPTY_NODE, 0);
    }
    out
}

/// The traversal stacks hold TREE_STACK_SIZE entries and grow by about one per tree level, so a
/// corrupt chain deeper than the stack would trip its debug assert. Validated blobs are bounded;
/// a shallower chain still validates and can be queried.
#[test]
fn compound_tree_depth_validated() {
    use crate::compound::{query_compound, ray_cast_compound};
    use crate::geometry::RayCastInput;

    let spheres: Vec<CompoundSphereDef> = (0..400)
        .map(|i| CompoundSphereDef {
            sphere: Sphere {
                center: v(i as f32, 0.0, 0.0),
                radius: 0.25,
            },
            material: default_surface_material(),
        })
        .collect();
    let compound = create_compound(&CompoundDef {
        spheres: &spheres,
        ..Default::default()
    })
    .expect("compound");
    let bytes = compound.to_bytes();
    assert!(validate_compound(&bytes).is_some());

    assert!(
        validate_compound(&chain_tree_blob(&bytes, 300)).is_none(),
        "300 level chain was accepted"
    );

    let shallow = validate_compound(&chain_tree_blob(&bytes, 100)).expect("100 level chain");
    let aabb =
        crate::compound::compute_compound_aabb(&shallow, crate::math_functions::TRANSFORM_IDENTITY);
    let mut visits = 0;
    query_compound(&shallow, aabb, |_, _| {
        visits += 1;
        true
    });
    assert_eq!(visits, 101);
    let output = ray_cast_compound(
        &shallow,
        &RayCastInput {
            origin: v(-1.0, 0.0, 0.0),
            translation: v(2.0, 0.0, 0.0),
            max_fraction: 1.0,
        },
    );
    assert!(output.hit);
}
