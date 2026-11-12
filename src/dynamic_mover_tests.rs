//! Dynamic mover library tests (port additions; the C sample has no tests of its own).
//!
//! A small scene (ground, a step, a ramp, a kinematic platform, a pushable crate) carries
//! several scripted movers that walk, climb, jump, launch and ride. The tests check that
//! a world snapshot plus the movers' own images resume bit-identically, that a shared
//! joint anchor gives the same motion as the sample's private anchors, and that the def's
//! filters reach the capsule shape and the pogo casts. The demo's Dynamic Mover trace
//! test checks the mover against the C sample itself.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

use crate::body::{
    body_apply_linear_impulse_to_center, body_get_linear_velocity, body_get_position,
    body_get_shapes, body_is_valid, create_body,
};
use crate::dynamic_mover::{DynamicMover, DynamicMoverDef};
use crate::hull::{make_box_hull, make_offset_box_hull};
use crate::id::{BodyId, NULL_BODY_ID};
use crate::math_functions::{make_quat_from_axis_angle, Pos, Vec2, Vec3, VEC3_AXIS_Z};
use crate::recording::{clone_world_via_snapshot, hash_world_state, RecBuffer, SnapReader};
use crate::shape::{create_hull_shape, shape_get_filter};
use crate::types::{
    default_body_def, default_shape_def, default_world_def, BodyType, Filter, QueryFilter,
};
use crate::world::{world_get_counters, World};

const DT: f32 = 1.0 / 60.0;
const SUB_STEPS: i32 = 4;
const PLATFORM_MATERIAL: u64 = 7;
const MOVER_COUNT: usize = 4;

const GROUND_BIT: u64 = 0x1;
const MOVER_BIT: u64 = 0x2;

fn pos(x: f32, y: f32, z: f32) -> Pos {
    Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    }
}

/// The static and kinematic scenery every test walks over.
fn build_scenery(world: &mut World) {
    let mut shape_def = default_shape_def();
    shape_def.filter = Filter {
        category_bits: GROUND_BIT,
        mask_bits: u64::MAX,
        group_index: 0,
    };

    let ground = create_body(world, &default_body_def());
    let floor = make_offset_box_hull(
        30.0,
        0.5,
        30.0,
        Vec3 {
            x: 0.0,
            y: -0.5,
            z: 0.0,
        },
    );
    create_hull_shape(world, ground, &shape_def, &floor.base);
    let step = make_offset_box_hull(
        1.0,
        0.15,
        3.0,
        Vec3 {
            x: 4.0,
            y: 0.15,
            z: 0.0,
        },
    );
    create_hull_shape(world, ground, &shape_def, &step.base);

    let mut ramp_def = default_body_def();
    ramp_def.position = pos(-4.0, 0.3, 0.0);
    ramp_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_Z, 0.35);
    let ramp = create_body(world, &ramp_def);
    create_hull_shape(world, ramp, &shape_def, &make_box_hull(2.0, 0.1, 2.0).base);

    let mut platform_def = default_body_def();
    platform_def.type_ = BodyType::Kinematic;
    platform_def.position = pos(0.0, 0.2, 5.0);
    platform_def.linear_velocity = Vec3 {
        x: 0.5,
        y: 0.0,
        z: 0.0,
    };
    let platform = create_body(world, &platform_def);
    let mut platform_shape = shape_def.clone();
    platform_shape.base_material.user_material_id = PLATFORM_MATERIAL;
    create_hull_shape(
        world,
        platform,
        &platform_shape,
        &make_box_hull(1.5, 0.2, 1.5).base,
    );

    let mut crate_def = default_body_def();
    crate_def.type_ = BodyType::Dynamic;
    crate_def.position = pos(1.5, 0.4, -2.0);
    let crate_body = create_body(world, &crate_def);
    let mut crate_shape = shape_def.clone();
    crate_shape.density = 50.0;
    create_hull_shape(
        world,
        crate_body,
        &crate_shape,
        &make_box_hull(0.4, 0.4, 0.4).base,
    );
}

fn mover_def(i: usize, anchor_id: BodyId) -> DynamicMoverDef {
    let start = [
        pos(0.0, 1.5, 0.0),
        pos(2.0, 1.5, 0.0),
        pos(-2.0, 1.5, 0.0),
        pos(0.0, 1.8, 5.0),
    ];
    DynamicMoverDef {
        position: start[i],
        filter: Filter {
            category_bits: MOVER_BIT,
            mask_bits: u64::MAX,
            group_index: 0,
        },
        platform_material_id: PLATFORM_MATERIAL,
        anchor_id,
        ..DynamicMoverDef::default()
    }
}

/// The scene with private anchors (the C sample) or one shared static anchor.
fn build_scene(shared_anchor: bool) -> (World, Vec<DynamicMover>) {
    let mut world = World::new(&default_world_def());
    build_scenery(&mut world);
    let anchor = if shared_anchor {
        create_body(&mut world, &default_body_def())
    } else {
        NULL_BODY_ID
    };
    let movers = (0..MOVER_COUNT)
        .map(|i| {
            let mut mover = DynamicMover::new();
            mover.create(&mut world, &mover_def(i, anchor));
            mover
        })
        .collect();
    (world, movers)
}

/// One tick of scripted input for every mover, then the world step.
fn tick(world: &mut World, movers: &mut [DynamicMover], step: usize) {
    for (i, mover) in movers.iter_mut().enumerate() {
        let phase = (step + 37 * i) % 240;
        if phase == 90 {
            mover.jump(world);
        }
        if i == 2 && phase == 200 {
            let id = mover.body_id();
            body_apply_linear_impulse_to_center(
                world,
                id,
                Vec3 {
                    x: 0.0,
                    y: 300.0,
                    z: 0.0,
                },
                true,
            );
            mover.launch();
        }
        let angle = 0.02 * (step as f32) + i as f32;
        let forward = Vec3 {
            x: angle.cos(),
            y: 0.0,
            z: angle.sin(),
        };
        let right = Vec3 {
            x: -forward.z,
            y: 0.0,
            z: forward.x,
        };
        let throttle = if phase < 150 {
            Vec2 { x: 1.0, y: 0.3 }
        } else {
            Vec2 { x: 0.0, y: 0.0 }
        };
        let speed_scale = if i == 1 { 1.5 } else { 1.0 };
        mover.update(world, DT, forward, right, throttle, speed_scale);
    }
    world.step(DT, SUB_STEPS);
}

fn mover_bytes(movers: &[DynamicMover]) -> Vec<u8> {
    let mut buf = RecBuffer::new();
    for mover in movers {
        mover.serialize(&mut buf);
    }
    buf.data
}

#[test]
fn snapshot_with_mover_state_resumes_bit_identically() {
    const N: usize = 300;
    let (mut world, mut movers) = build_scene(false);
    for step in 0..N {
        tick(&mut world, &mut movers, step);
    }
    assert!(
        movers.iter().any(|m| m.on_ground),
        "the script must leave someone on the ground"
    );

    // Save: the world snapshot and every mover's image.
    let saved = mover_bytes(&movers);
    let mut restored_world = clone_world_via_snapshot(&world).expect("world snapshot restores");
    let mut reader = SnapReader::new(&saved);
    let mut restored: Vec<DynamicMover> = (0..MOVER_COUNT)
        .map(|_| DynamicMover::deserialize(&mut reader).expect("mover image restores"))
        .collect();
    assert_eq!(
        reader.cursor(),
        saved.len(),
        "every mover image is read to its end"
    );
    assert_eq!(restored, movers);
    assert_eq!(hash_world_state(&restored_world), hash_world_state(&world));

    for step in N..2 * N {
        tick(&mut world, &mut movers, step);
        tick(&mut restored_world, &mut restored, step);
        assert_eq!(
            hash_world_state(&restored_world),
            hash_world_state(&world),
            "world hash diverged at tick {step}"
        );
        assert_eq!(
            mover_bytes(&restored),
            mover_bytes(&movers),
            "mover state diverged at tick {step}"
        );
    }
}

#[test]
fn mover_image_rejects_truncation_and_bad_magic() {
    let (_world, movers) = build_scene(false);
    let bytes = mover_bytes(&movers[..1]);
    assert!(DynamicMover::deserialize(&mut SnapReader::new(&bytes)).is_some());
    assert!(DynamicMover::deserialize(&mut SnapReader::new(&bytes[..bytes.len() - 1])).is_none());
    let mut bad = bytes.clone();
    bad[0] ^= 0xFF;
    assert!(DynamicMover::deserialize(&mut SnapReader::new(&bad)).is_none());
}

/// Every mover's body position and velocity bits plus its controller state, without the
/// ids (which differ by the anchors' slots).
fn motion_trace(world: &World, movers: &[DynamicMover]) -> Vec<u64> {
    let mut out = Vec::new();
    for m in movers {
        let p = body_get_position(world, m.body_id());
        let v = body_get_linear_velocity(world, m.body_id());
        for x in [p.x as f64, p.y as f64, p.z as f64] {
            out.push(x.to_bits());
        }
        for x in [
            v.x,
            v.y,
            v.z,
            m.time_sliding,
            m.pogo_impulse,
            m.pogo_velocity,
            m.pogo_length,
        ] {
            out.push(x.to_bits() as u64);
        }
        out.push(u64::from(m.on_ground) | u64::from(m.walkable) << 1 | u64::from(m.jumping) << 2);
        out.push(m.probe_count as u64);
    }
    out
}

#[test]
fn shared_anchor_moves_exactly_like_private_anchors() {
    let (mut private_world, mut private_movers) = build_scene(false);
    let (mut shared_world, mut shared_movers) = build_scene(true);

    // One anchor for the crowd instead of one per mover.
    let private_bodies = world_get_counters(&private_world).body_count;
    let shared_bodies = world_get_counters(&shared_world).body_count;
    assert_eq!(private_bodies - shared_bodies, MOVER_COUNT as i32 - 1);
    let anchor = shared_movers[0].ground_id;
    assert!(shared_movers
        .iter()
        .all(|m| m.ground_id == anchor && !m.owns_ground));

    let mut rode_platform = false;
    for step in 0..900 {
        tick(&mut private_world, &mut private_movers, step);
        tick(&mut shared_world, &mut shared_movers, step);
        assert_eq!(
            motion_trace(&shared_world, &shared_movers),
            motion_trace(&private_world, &private_movers),
            "a shared anchor changed the motion at tick {step}"
        );
        rode_platform |= shared_movers.iter().any(|m| {
            m.cast_result.body_id != NULL_BODY_ID
                && m.cast_result.user_material_id == PLATFORM_MATERIAL
                && m.on_ground
        });
    }
    assert!(rode_platform, "the script must put a mover on the platform");

    // Destroying a mover leaves the shared anchor for the others.
    shared_movers[0].destroy(&mut shared_world);
    assert!(body_is_valid(&shared_world, anchor));
    let private_anchor = private_movers[0].ground_id;
    private_movers[0].destroy(&mut private_world);
    assert!(!body_is_valid(&private_world, private_anchor));
}

#[test]
fn def_filters_reach_the_capsule_and_the_pogo() {
    let mut world = World::new(&default_world_def());
    build_scenery(&mut world);

    // Default query filter: the capsule's bits, so the pogo finds the ground.
    let mut seeing = DynamicMover::new();
    seeing.create(&mut world, &mover_def(0, NULL_BODY_ID));
    assert_eq!(seeing.query_filter.category_bits, MOVER_BIT);
    assert_eq!(seeing.query_filter.mask_bits, u64::MAX);
    let shapes = body_get_shapes(&world, seeing.body_id(), 1);
    assert_eq!(
        shape_get_filter(&world, shapes[0]),
        mover_def(0, NULL_BODY_ID).filter
    );

    // A query mask without the ground category: the pogo sees nothing and the mover falls.
    let mut blind = DynamicMover::new();
    blind.create(
        &mut world,
        &DynamicMoverDef {
            query_filter: Some(QueryFilter {
                category_bits: MOVER_BIT,
                mask_bits: !GROUND_BIT,
                id: 3,
                name: String::from("blind"),
            }),
            ..mover_def(1, NULL_BODY_ID)
        },
    );

    let still = Vec2 { x: 0.0, y: 0.0 };
    let forward = Vec3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };
    let right = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    };
    for _ in 0..120 {
        seeing.update(&mut world, DT, forward, right, still, 1.0);
        blind.update(&mut world, DT, forward, right, still, 1.0);
        world.step(DT, SUB_STEPS);
    }
    assert!(seeing.on_ground && seeing.cast_result.hit);
    assert!(!blind.on_ground && !blind.cast_result.hit);
    let standing = body_get_position(&world, seeing.body_id()).y as f32;
    let lying = body_get_position(&world, blind.body_id()).y as f32;
    assert!(
        standing > lying + 0.5,
        "the pogo holds the seeing mover up: {standing} vs {lying}"
    );
}
