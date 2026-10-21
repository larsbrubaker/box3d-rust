// Recording tests for compound geometry (upstream 003a3f7, "Fix recording crash on compounds").
// `include!`d into `recording_replay_tests.rs`, so it shares that module's imports.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use crate::compound::{
    convert_compound_to_bytes, create_compound, CompoundCapsuleDef, CompoundData,
    CompoundSphereDef, COMPOUND_TREE_OFFSET,
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

/// b3RecInternCompound scrubs every tree pointer through b3ConvertCompoundToBytes, so the
/// canonical blob (and its content hash) is pointer-free. Before 003a3f7 the C side only nulled
/// `tree.nodes`, leaving the live `tree.proxies` pointer in the recorded bytes. The expected
/// byte count and hash come from the scalar C build of 003a3f7 (MSVC, BOX3D_DISABLE_SIMD)
/// interning this same compound.
#[test]
fn intern_compound_is_pointer_free() {
    let compound = rec_compound();
    let mut rec = Recording::new(0);
    let id = rec.registry.intern_compound(&compound);
    let entry = &rec.registry.entries[id as usize];
    assert_eq!(entry.kind, GeometryKind::Compound);
    assert_eq!(entry.bytes, convert_compound_to_bytes(&compound));

    // The embedded b3DynamicTree sits at COMPOUND_TREE_OFFSET in b3CompoundData; its pointer fields (nodes,
    // parents, proxies) follow the 8 byte version.
    let ptrs = COMPOUND_TREE_OFFSET + 8;
    assert!(entry.bytes[ptrs..ptrs + 24].iter().all(|&b| b == 0));

    assert_eq!(entry.bytes.len(), 672);
    assert_eq!(entry.content_hash, 0x6ec7_dd6f_a6f8_4724);

    // Interning the same compound again dedups to the same id.
    assert_eq!(rec.registry.intern_compound(&compound), id);
}

/// Mirrors the keyframe-policy call upstream added to the C `AllOps` session: set a 4 frame
/// keyframe interval while a baked compound is live, play to the end, seek backward, and replay
/// to the end. In C, the stale `tree.proxies` pointer made keyframe capture re-intern the
/// compound under a new hash, tripping the registry-growth assert in b3RecCaptureKeyframe. The
/// Rust player has no keyframe capture: `seek_frame` restarts from frame 0 and replays, so here
/// the policy call only mirrors C and the test checks the seek and replay stay in sync.
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
    // Mirrors C's AllOps keyframe-policy call. The Rust player stores the policy but seeks by
    // restart-and-replay, so no keyframes are captured.
    let budget = player.get_keyframe_budget();
    player.set_keyframe_policy(budget, 4);

    while player.step_frame() {}
    assert_eq!(player.get_frame(), 20);
    assert!(!player.has_diverged());

    player.seek_frame(5);
    assert_eq!(player.get_frame(), 5);
    assert!(!player.has_diverged());

    while player.step_frame() {}
    assert_eq!(player.get_frame(), 20);
    assert!(!player.has_diverged());
}
