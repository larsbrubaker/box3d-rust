// Replay tests for corrupt geometry registry slots.
// `include!`d into `recording_replay_tests.rs`, so it shares that module's imports.
//
// A recording carries each interned geometry as an opaque blob in the trailing registry
// block. C's replay dispatcher casts those bytes straight to a b3HullData* / b3MeshData*
// and would happily run on garbage; the Rust converters return None instead, so the
// dispatch arms have to fail the replay gracefully rather than unwrap.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use crate::recording::{GeometryKind, RecHeader};

/// Walk the trailing registry block and return `(kind, blob start)` per slot.
fn registry_slot_spans(data: &[u8]) -> Vec<(GeometryKind, usize)> {
    let hdr = RecHeader::from_bytes(data).expect("header");
    let mut cursor = hdr.registry_offset as usize;
    assert!(cursor != 0 && cursor + 4 <= data.len(), "no registry block");
    let entry_count = u32::from_le_bytes(data[cursor..cursor + 4].try_into().unwrap()) as usize;
    cursor += 4;
    let mut spans = Vec::with_capacity(entry_count);
    for _ in 0..entry_count {
        assert!(cursor + 5 <= data.len());
        let kind = match data[cursor] {
            0 => GeometryKind::Hull,
            1 => GeometryKind::Mesh,
            2 => GeometryKind::HeightField,
            _ => GeometryKind::Compound,
        };
        cursor += 1;
        let byte_count =
            u32::from_le_bytes(data[cursor..cursor + 4].try_into().unwrap()) as usize;
        cursor += 4;
        assert!(cursor + byte_count <= data.len());
        spans.push((kind, cursor));
        cursor += byte_count;
    }
    spans
}

/// Poison one registry slot's version word so its converter rejects the blob. This is the
/// corruption a truncated or bit-rotted recording file produces in the wild.
fn corrupt_registry_slot(data: &[u8], slot: usize) -> Vec<u8> {
    let start = registry_slot_spans(data)[slot].1;
    let mut copy = data.to_vec();
    copy[start] ^= 0xFF;
    copy
}

/// A recording that exercises every geometry-carrying shape op: one create per geometry
/// kind, plus the hull and mesh setters. Each op interns its own registry slot, so the
/// caller can corrupt exactly one dispatch arm at a time.
fn record_all_geometry_ops() -> Recording {
    let mut rec = Recording::new(0);
    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, &mut rec);

    let mut static_def = default_body_def();
    static_def.type_ = BodyType::Static;

    // Height field (slot 0)
    let hf_body = create_body(&mut world, &static_def);
    let hf = crate::height_field::create_height_field(&crate::height_field::HeightFieldDef {
        heights: vec![0.0, 0.1, 0.0, 0.1, 0.2, 0.1, 0.0, 0.1, 0.0],
        material_indices: vec![0u8; 4],
        scale: Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        },
        count_x: 3,
        count_z: 3,
        global_minimum_height: -1.0,
        global_maximum_height: 1.0,
        clockwise_winding: false,
    });
    crate::shape::create_height_field_shape(&mut world, hf_body, &default_shape_def(), &hf);

    // Compound (slot 1). It carries a hull and a mesh child so the nested blob readers
    // inside b3ConvertBytesToCompound are on the replay path too.
    let compound_body = create_body(&mut world, &static_def);
    let compound_hull = make_box_hull(0.3, 0.3, 0.3);
    let compound_mesh =
        crate::mesh::create_box_mesh(VEC3_ZERO, Vec3 { x: 0.4, y: 0.4, z: 0.4 }, false)
            .expect("compound mesh");
    let compound = crate::compound::create_compound(&crate::compound::CompoundDef {
        capsules: &[],
        hulls: &[crate::compound::CompoundHullDef {
            hull: &compound_hull.base,
            transform: crate::math_functions::TRANSFORM_IDENTITY,
            material: crate::geometry::default_surface_material(),
        }],
        meshes: &[crate::compound::CompoundMeshDef {
            mesh_data: &compound_mesh,
            transform: crate::math_functions::TRANSFORM_IDENTITY,
            scale: Vec3 {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
            materials: &[crate::geometry::default_surface_material()],
        }],
        spheres: &[crate::compound::CompoundSphereDef {
            sphere: Sphere {
                center: Vec3 {
                    x: 0.0,
                    y: -4.0,
                    z: 0.0,
                },
                radius: 0.5,
            },
            material: crate::geometry::default_surface_material(),
        }],
    })
    .expect("compound");
    crate::shape::create_baked_compound_shape(
        &mut world,
        compound_body,
        &default_shape_def(),
        &compound,
    );

    // Mesh (slot 2)
    let mesh_body = create_body(&mut world, &static_def);
    let mesh_a = crate::mesh::create_grid_mesh(4, 4, 2.0, 1, false).expect("mesh a");
    let one = Vec3 {
        x: 1.0,
        y: 1.0,
        z: 1.0,
    };
    let mesh_shape = crate::shape::create_mesh_shape(
        &mut world,
        mesh_body,
        &default_shape_def(),
        &mesh_a,
        one,
    );

    // Hull (slot 3)
    let mut dynamic_def = default_body_def();
    dynamic_def.type_ = BodyType::Dynamic;
    dynamic_def.position = Pos {
        x: 0.0 as _,
        y: 3.0 as _,
        z: 0.0 as _,
    };
    let hull_body = create_body(&mut world, &dynamic_def);
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let mut hull_shape_def = default_shape_def();
    hull_shape_def.density = 1.0;
    let hull_shape =
        create_hull_shape(&mut world, hull_body, &hull_shape_def, &box_hull.base);

    world.step(1.0 / 60.0, 4);

    // Hull setter (slot 4) and mesh setter (slot 5)
    let swap_hull = make_box_hull(0.25, 1.5, 0.25);
    crate::shape::shape_set_hull(&mut world, hull_shape, &swap_hull.base);
    let mesh_b = crate::mesh::create_grid_mesh(6, 6, 1.5, 1, false).expect("mesh b");
    crate::shape::shape_set_mesh(&mut world, mesh_shape, &mesh_b, one);

    world.step(1.0 / 60.0, 4);
    world_stop_recording(&mut world);
    rec
}

/// Every geometry-carrying shape op must fail the replay instead of panicking when its
/// registry slot is corrupt. The pristine recording replays clean, so the six negative
/// cases below are proving the guard and not just a broken scene.
#[test]
fn corrupt_geometry_slot_fails_replay() {
    let rec = record_all_geometry_ops();
    let data = rec.data().to_vec();
    assert!(validate_replay(&data, 1), "pristine recording must replay");

    let spans = registry_slot_spans(&data);
    assert_eq!(
        spans.iter().map(|s| s.0).collect::<Vec<_>>(),
        vec![
            GeometryKind::HeightField,
            GeometryKind::Compound,
            GeometryKind::Mesh,
            GeometryKind::Hull,
            GeometryKind::Hull,
            GeometryKind::Mesh,
        ],
        "one slot per geometry-carrying op, in record order"
    );

    for slot in 0..spans.len() {
        let corrupt = corrupt_registry_slot(&data, slot);
        assert!(
            !validate_replay(&corrupt, 1),
            "corrupt slot {slot} was replayed as if valid"
        );
    }
}

/// Overwrite a little-endian i32 in a recording copy.
fn poison_i32(data: &[u8], offset: usize, value: i32) -> Vec<u8> {
    let mut copy = data.to_vec();
    copy[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    copy
}

fn read_i32_at(data: &[u8], offset: usize) -> i32 {
    i32::from_le_bytes(data[offset..offset + 4].try_into().unwrap())
}

/// Poisoning the version word is caught by the version compare before any offset
/// arithmetic runs, so it never exercises the section-offset guards. These cases poison a
/// *section offset* instead, which is what makes the guards load bearing on the replay
/// path: without them the dispatcher panics inside the converter.
#[test]
fn poisoned_section_offset_fails_replay() {
    let rec = record_all_geometry_ops();
    let data = rec.data().to_vec();
    assert!(validate_replay(&data, 1), "pristine recording must replay");

    let spans = registry_slot_spans(&data);
    // One section offset per slot, by kind: height field heightsOffset (76), mesh
    // nodeOffset (56), hull pointOffset (108). Compounds are covered separately below.
    let field = |kind: GeometryKind| match kind {
        GeometryKind::HeightField => Some(76usize),
        GeometryKind::Mesh => Some(56),
        GeometryKind::Hull => Some(108),
        GeometryKind::Compound => None,
    };

    for (slot, (kind, start)) in spans.iter().enumerate() {
        let Some(offset) = field(*kind) else { continue };
        for poison in [-16, i32::MAX] {
            let corrupt = poison_i32(&data, start + offset, poison);
            assert!(
                !validate_replay(&corrupt, 1),
                "slot {slot} survived section offset poison {poison}"
            );
        }
    }
}

/// The hull and mesh blobs nested inside a compound slot are parsed by a second pair of
/// readers, reached only through the compound path. They need the same guards.
#[test]
fn corrupt_nested_compound_blob_fails_replay() {
    let rec = record_all_geometry_ops();
    let data = rec.data().to_vec();
    let compound = registry_slot_spans(&data)[1].1;

    // hullOffset (112) locates the hull instances; each stores its blob offset at
    // instance byte 28. Poison the nested hull's pointOffset (blob byte 108).
    let hull_instances = compound + read_i32_at(&data, compound + 112) as usize;
    let nested_hull = compound + read_i32_at(&data, hull_instances + 28) as usize;
    assert!(
        !validate_replay(&poison_i32(&data, nested_hull + 108, i32::MAX), 1),
        "nested hull pointOffset poison survived the replay"
    );

    // meshOffset (124) locates the mesh instances; each stores its blob offset at
    // instance byte 40. Poison the nested mesh's vertexCount (blob byte 68).
    let mesh_instances = compound + read_i32_at(&data, compound + 124) as usize;
    let nested_mesh = compound + read_i32_at(&data, mesh_instances + 40) as usize;
    assert!(
        !validate_replay(&poison_i32(&data, nested_mesh + 68, -1), 1),
        "nested mesh negative vertexCount survived the replay"
    );
}

/// The registry records each slot's geometry kind, but the dispatch arms used to ignore
/// it and simply reinterpret the bytes. A kind-mismatched geometryId must be rejected.
#[test]
fn kind_mismatched_geometry_slot_fails_replay() {
    let rec = record_all_geometry_ops();
    let data = rec.data().to_vec();

    for (slot, (kind, start)) in registry_slot_spans(&data).iter().enumerate() {
        // The kind byte sits just before the 4-byte blob length.
        let kind_byte = start - 5;
        let wrong = if *kind == GeometryKind::Hull { 1u8 } else { 0 };
        let mut corrupt = data.clone();
        corrupt[kind_byte] = wrong;
        assert!(
            !validate_replay(&corrupt, 1),
            "slot {slot} was accepted under the wrong geometry kind"
        );
    }
}

/// An out-of-range geometryId in a create op must be rejected the same way the setters
/// already reject one, rather than indexing past the slot array.
#[test]
fn out_of_range_geometry_id_fails_replay() {
    let rec = record_all_geometry_ops();
    let data = rec.data().to_vec();

    // Drop every registry slot: the ops still reference ids 0..5, which no longer exist.
    let hdr = RecHeader::from_bytes(&data).expect("header");
    let mut truncated = data.clone();
    let registry_offset = hdr.registry_offset as usize;
    truncated.truncate(registry_offset + 4);
    truncated[registry_offset..registry_offset + 4].copy_from_slice(&0u32.to_le_bytes());
    // Header bytes 40..48 are registryByteCount; the block is now just the empty count.
    truncated[40..48].copy_from_slice(&4u64.to_le_bytes());

    assert!(
        !validate_replay(&truncated, 1),
        "ops referencing missing registry slots must fail the replay"
    );
}
