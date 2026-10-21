// Recording tests for compound geometry (upstream 003a3f7, "Fix recording crash on compounds").
// `include!`d into `recording_replay_tests.rs`, so it shares that module's imports.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use crate::compound::{
    create_compound, CompoundCapsuleDef, CompoundData, CompoundSphereDef,
};
use crate::geometry::Capsule;
use crate::shape::create_baked_compound_shape;

/// One capsule and three spheres, so the baked tree has interior nodes and proxies.
fn rec_compound() -> CompoundData {
    let m = crate::geometry::default_surface_material();
    let v = |x: f32, y: f32, z: f32| Vec3 { x, y, z };
    let capsules = [CompoundCapsuleDef {
        capsule: Capsule {
            center1: v(-0.5, 0.0, 0.0),
            center2: v(0.5, 0.0, 0.0),
            radius: 0.25,
        },
        material: m,
    }];
    let sphere = |center: Vec3, radius: f32| CompoundSphereDef {
        sphere: Sphere { center, radius },
        material: m,
    };
    let spheres = [
        sphere(v(0.0, 1.0, 0.0), 0.5),
        sphere(v(1.0, 0.0, 1.0), 0.3),
        sphere(v(-1.0, 0.5, -1.0), 0.4),
    ];
    create_compound(&crate::compound::CompoundDef {
        capsules: &capsules,
        spheres: &spheres,
        ..Default::default()
    })
    .expect("compound")
}

/// b3RecInternCompound interns the compound bytes as they are: since upstream 5d83df8 the
/// compound is a pointer-free blob addressed by offsets, so the canonical bytes need no
/// scrubbing. The expected byte count and hash come from the scalar C build of 5d83df8 (MSVC,
/// BOX3D_DISABLE_SIMD, float and double precision alike) interning this same compound; the
/// full blob was also compared byte for byte.
#[test]
fn intern_compound_is_pointer_free() {
    let compound = rec_compound();
    let mut rec = Recording::new(0);
    let id = rec.registry.intern_compound(&compound);
    let entry = &rec.registry.entries[id as usize];
    assert_eq!(entry.kind, GeometryKind::Compound);
    assert_eq!(entry.bytes, compound.to_bytes());

    assert_eq!(entry.bytes.len(), 488);
    assert_eq!(entry.content_hash, 0x58b8_26b8_d7e0_7710);

    // Interning the same compound again dedups to the same id.
    assert_eq!(rec.registry.intern_compound(&compound), id);
}

/// Mirrors the keyframe-policy call upstream added to the C `AllOps` session: set a 4 frame
/// keyframe interval while a baked compound is live, play to the end, seek backward, and replay
/// to the end. In C, the stale `tree.proxies` pointer made keyframe capture re-intern the
/// compound under a new hash, tripping the registry-growth assert in b3RecCaptureKeyframe. Here
/// every capture re-serializes the live compound, which must resolve to its pre-seeded registry
/// slot (the same assert is a `debug_assert` in `capture_keyframe`), and the backward seek restores
/// the frame 4 keyframe.
#[test]
fn compound_keyframe_seek() {
    let compound = rec_compound();

    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, Recording::new(0));

    let ground_id = create_body(&mut world, &default_body_def());
    create_baked_compound_shape(&mut world, ground_id, &default_shape_def(), &compound);

    let mut ball_def = default_body_def();
    ball_def.type_ = BodyType::Dynamic;
    ball_def.position = Pos {
        x: 0.0 as _,
        y: 3.0 as _,
        z: 0.0 as _,
    };
    let ball_id = create_body(&mut world, &ball_def);
    create_sphere_shape(
        &mut world,
        ball_id,
        &default_shape_def(),
        &Sphere {
            center: VEC3_ZERO,
            radius: 0.25,
        },
    );

    for _ in 0..20 {
        world.step(1.0 / 60.0, 4);
    }

    let rec = world_stop_recording(&mut world).expect("recording active");
    drop(world);

    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    // Mirrors C's AllOps keyframe-policy call.
    let budget = player.get_keyframe_budget();
    player.set_keyframe_policy(budget, 4);

    let slot_count = player.keyframe_registry.entries.len();
    while player.step_frame() {}
    assert_eq!(player.get_frame(), 20);
    assert!(!player.has_diverged());
    assert!(player.get_keyframe_bytes() > 0);
    assert_eq!(player.get_keyframe_frames(), vec![4, 8, 12, 16, 20]);
    assert_eq!(player.keyframe_registry.entries.len(), slot_count);

    player.seek_frame(5);
    assert_eq!(player.get_frame(), 5);
    assert!(!player.has_diverged());

    while player.step_frame() {}
    assert_eq!(player.get_frame(), 20);
    assert!(!player.has_diverged());
}

/// The seek addition upstream made to the C `AllOps` session (3d9db79): after playing to the end,
/// seek back to frame 6 and forward to frame 9 with a 4 frame keyframe interval. Each restore
/// rebuilds the compound from the registry, so a ray must still reach its tree and the state
/// hashes must hold through the re-stepped frames. C also asserts the player captured keyframes
/// (`b3RecPlayer_GetKeyframeBytes > 0`, upstream 3d9db79).
#[test]
fn compound_seek_ray_cast() {
    use crate::shape::shape_get_type;
    use crate::geometry::ShapeType;
    use crate::types::default_query_filter;
    use crate::world::world_cast_ray;

    let compound = create_compound(&crate::compound::CompoundDef {
        spheres: &[CompoundSphereDef {
            sphere: Sphere {
                center: VEC3_ZERO,
                radius: 1.0,
            },
            material: crate::geometry::default_surface_material(),
        }],
        ..Default::default()
    })
    .expect("compound");

    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, Recording::new(0));

    let mut compound_body_def = default_body_def();
    compound_body_def.type_ = BodyType::Static;
    compound_body_def.position = Pos {
        x: 30.0 as _,
        y: 0.0 as _,
        z: 0.0 as _,
    };
    let compound_body_id = create_body(&mut world, &compound_body_def);
    create_baked_compound_shape(
        &mut world,
        compound_body_id,
        &default_shape_def(),
        &compound,
    );

    let mut ball_def = default_body_def();
    ball_def.type_ = BodyType::Dynamic;
    ball_def.position = Pos {
        x: 0.0 as _,
        y: 3.0 as _,
        z: 0.0 as _,
    };
    let ball_id = create_body(&mut world, &ball_def);
    create_sphere_shape(
        &mut world,
        ball_id,
        &default_shape_def(),
        &Sphere {
            center: VEC3_ZERO,
            radius: 0.25,
        },
    );

    for _ in 0..12 {
        world.step(1.0 / 60.0, 4);
    }

    let rec = world_stop_recording(&mut world).expect("recording active");
    drop(world);

    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    let budget = player.get_keyframe_budget();
    player.set_keyframe_policy(budget, 4);

    while player.step_frame() {}
    assert_eq!(player.get_frame(), 12);
    assert!(player.is_at_end());
    assert!(!player.has_diverged());
    assert!(player.get_keyframe_bytes() > 0);
    assert_eq!(player.get_keyframe_frames(), vec![4, 8, 12]);

    // Seek back onto the frame 4 keyframe, then skip forward onto the frame 8 keyframe. Each
    // restore rebuilds the compound from the registry, so a ray must still reach its tree and
    // the state hashes must hold through the re-stepped frames.
    for target in [6, 9] {
        player.seek_frame(target);
        assert_eq!(player.get_frame(), target);

        let replay_world = player.world();
        let mut hit_compound = false;
        world_cast_ray(
            replay_world,
            Pos {
                x: 30.0 as _,
                y: 10.0 as _,
                z: 0.0 as _,
            },
            Vec3 {
                x: 0.0,
                y: -20.0,
                z: 0.0,
            },
            &default_query_filter(),
            |shape_id, _p, _n, _f, _m, _t, _c| {
                if shape_get_type(replay_world, shape_id) == ShapeType::Compound {
                    hit_compound = true;
                }
                1.0
            },
        );
        assert!(hit_compound);
    }
    while player.step_frame() {}
    assert_eq!(player.get_frame(), 12);
    assert!(!player.has_diverged());
}

/// A corrupt compound in the geometry registry must fail the replay cleanly. A truncated, empty,
/// or invalid compound entry is rejected at load, and a wrong kind when the shape is created.
/// (CorruptCompound)
#[test]
fn corrupt_compound() {
    use crate::compound::COMPOUND_VERSION;

    let compound = create_compound(&crate::compound::CompoundDef {
        spheres: &[CompoundSphereDef {
            sphere: Sphere {
                center: VEC3_ZERO,
                radius: 1.0,
            },
            material: crate::geometry::default_surface_material(),
        }],
        ..Default::default()
    })
    .expect("compound");
    let compound_byte_count = compound.byte_count;
    // The scalar C build of 5d83df8 (MSVC, BOX3D_DISABLE_SIMD) bakes this compound to 224 bytes.
    assert_eq!(compound_byte_count, 224);

    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, Recording::new(0));

    let mut bd = default_body_def();
    let ground_id = create_body(&mut world, &bd);
    let sd = default_shape_def();
    create_baked_compound_shape(&mut world, ground_id, &sd, &compound);

    bd.type_ = BodyType::Dynamic;
    bd.position = Pos {
        x: 0.0 as _,
        y: 3.0 as _,
        z: 0.0 as _,
    };
    let body_id = create_body(&mut world, &bd);
    let s = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    create_sphere_shape(&mut world, body_id, &sd, &s);

    for _ in 0..10 {
        world.step(1.0 / 60.0, 4);
    }

    let rec = world_stop_recording(&mut world).expect("recording active");
    drop(world);

    let rec_data = rec.data();
    let rec_size = rec_data.len();
    assert!(validate_replay(rec_data, 1));

    // The registry entry is a kind byte and a 4 byte length, then the blob, which leads with its version
    let version = COMPOUND_VERSION.to_le_bytes();
    let blob_offset = (5..=rec_size - version.len())
        .find(|&i| rec_data[i..i + version.len()] == version)
        .expect("compound blob");
    assert_eq!(rec_data[blob_offset - 5], GeometryKind::Compound as u8);
    let stored_byte_count =
        i32::from_le_bytes(rec_data[blob_offset - 4..blob_offset].try_into().unwrap());
    assert_eq!(stored_byte_count, compound_byte_count);

    let mut patched = rec_data.to_vec();
    patched[blob_offset] ^= 0xFF;
    assert!(!validate_replay(&patched, 1));

    let mut patched = rec_data.to_vec();
    let short_byte_count = 8i32;
    patched[blob_offset - 4..blob_offset].copy_from_slice(&short_byte_count.to_le_bytes());
    assert!(RecPlayer::create(&patched, 1).is_none());

    let mut patched = rec_data.to_vec();
    let zero_byte_count = 0i32;
    patched[blob_offset - 4..blob_offset].copy_from_slice(&zero_byte_count.to_le_bytes());
    assert!(RecPlayer::create(&patched, 1).is_none());

    let mut patched = rec_data.to_vec();
    patched[blob_offset - 5] = GeometryKind::Mesh as u8;
    assert!(!validate_replay(&patched, 1));
}
