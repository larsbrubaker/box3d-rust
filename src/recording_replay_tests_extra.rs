/// Issue all seven world queries each frame, then validate. (QueryReplay)
#[test]
fn query_replay() {
    use crate::distance::make_proxy;
    use crate::geometry::Capsule;
    use crate::math_functions::Aabb;
    use crate::recording::query_replay::RecQueryKind;
    use crate::recording::RecPlayer;
    use crate::types::default_query_filter;
    use crate::world::{
        world_cast_mover, world_cast_ray, world_cast_ray_closest, world_cast_shape,
        world_collide_mover, world_overlap_aabb, world_overlap_shape,
    };

    let mut rec = Recording::new(0);
    let mut world = World::new(&default_world_def());
    world_set_gravity(
        &mut world,
        Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        },
    );

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground_id = create_body(&mut world, &ground_def);
    let ground_box = make_box_hull(20.0, 1.0, 20.0);
    create_hull_shape(&mut world, ground_id, &default_shape_def(), &ground_box.base);

    for i in 0..4 {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: (i as f32 - 1.5) as _,
            y: 3.0,
            z: 0.0,
        };
        let body_id = create_body(&mut world, &body_def);
        let sphere = Sphere {
            center: VEC3_ZERO,
            radius: 0.5,
        };
        let mut sphere_def = default_shape_def();
        sphere_def.density = 1.0;
        create_sphere_shape(&mut world, body_id, &sphere_def, &sphere);
    }

    world_start_recording(&mut world, &mut rec);

    let filter = default_query_filter();
    for _ in 0..30 {
        let origin = Pos {
            x: 0.0,
            y: 6.0,
            z: 0.0,
        };
        let translation = Vec3 {
            x: 0.0,
            y: -8.0,
            z: 0.0,
        };
        let aabb = Aabb {
            lower_bound: Vec3 {
                x: -5.0,
                y: -1.0,
                z: -5.0,
            },
            upper_bound: Vec3 {
                x: 5.0,
                y: 6.0,
                z: 5.0,
            },
        };
        let proxy = make_proxy(&[VEC3_ZERO], 0.5);
        let mover = Capsule {
            center1: VEC3_ZERO,
            center2: Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            radius: 0.3,
        };

        world_overlap_aabb(&world, aabb, &filter, |_| true);
        world_overlap_shape(&world, origin, &proxy, &filter, |_| true);
        world_cast_ray(
            &world,
            origin,
            translation,
            &filter,
            |_id, _p, _n, fraction, _m, _t, _c| fraction,
        );
        world_cast_ray_closest(&world, origin, translation, &filter);
        world_cast_shape(
            &world,
            origin,
            &proxy,
            translation,
            &filter,
            |_id, _p, _n, fraction, _m, _t, _c| fraction,
        );
        let mut mover_filter = |_id| true;
        world_cast_mover(
            &world,
            origin,
            &mover,
            translation,
            &filter,
            Some(&mut mover_filter),
        );
        world_collide_mover(&world, origin, &mover, &filter, |_, _| true);

        world.step(1.0 / 60.0, 4);
    }

    world_stop_recording(&mut world);
    assert!(validate_replay(rec.data(), 1));

    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    player.seek_frame(15);
    assert!(!player.has_diverged());
    assert_eq!(player.get_frame_query_count(), 7);
    let first = player.get_frame_query(0).unwrap();
    assert_eq!(first.kind, RecQueryKind::OverlapAabb);

    let mut saw_cast_ray = false;
    for qi in 0..player.get_frame_query_count() {
        let info = player.get_frame_query(qi).unwrap();
        if info.kind == RecQueryKind::CastRay {
            saw_cast_ray = true;
            assert!(info.hit_count > 0);
        }
    }
    assert!(saw_cast_ray);
}

/// Tagged queries round-trip through the tag table. (TaggedQuery)
#[test]
fn tagged_query() {
    use crate::math_functions::Aabb;
    use crate::recording::query_replay::RecQueryKind;
    use crate::recording::RecPlayer;
    use crate::types::default_query_filter;
    use crate::world::{world_cast_ray, world_overlap_aabb};

    let mut rec = Recording::new(0);
    let mut world = World::new(&default_world_def());

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground_id = create_body(&mut world, &ground_def);
    let ground_box = make_box_hull(20.0, 1.0, 20.0);
    create_hull_shape(&mut world, ground_id, &default_shape_def(), &ground_box.base);

    world_start_recording(&mut world, &mut rec);

    let mut bullet53 = default_query_filter();
    bullet53.id = 53;
    bullet53.name = "bullet".into();
    let mut bullet54 = default_query_filter();
    bullet54.id = 54;
    bullet54.name = "bullet".into();
    let untagged = default_query_filter();

    let key53 = Recording::hash_query_tag(53, "bullet");
    let key54 = Recording::hash_query_tag(54, "bullet");
    assert!(key53 != 0 && key54 != 0 && key53 != key54);

    for _ in 0..10 {
        let origin = Pos {
            x: 0.0,
            y: 6.0,
            z: 0.0,
        };
        let translation = Vec3 {
            x: 0.0,
            y: -8.0,
            z: 0.0,
        };
        let aabb = Aabb {
            lower_bound: Vec3 {
                x: -5.0,
                y: -1.0,
                z: -5.0,
            },
            upper_bound: Vec3 {
                x: 5.0,
                y: 6.0,
                z: 5.0,
            },
        };

        world_cast_ray(
            &world,
            origin,
            translation,
            &bullet53,
            |_id, _p, _n, fraction, _m, _t, _c| fraction,
        );
        world_cast_ray(
            &world,
            origin,
            translation,
            &bullet54,
            |_id, _p, _n, fraction, _m, _t, _c| fraction,
        );
        world_overlap_aabb(&world, aabb, &untagged, |_| true);
        world.step(1.0 / 60.0, 4);
    }

    world_stop_recording(&mut world);
    assert!(validate_replay(rec.data(), 1));

    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    player.seek_frame(5);
    assert!(!player.has_diverged());
    assert_eq!(player.get_frame_query_count(), 3);

    let mut saw53 = false;
    let mut saw54 = false;
    let mut saw_untagged = false;
    for qi in 0..player.get_frame_query_count() {
        let info = player.get_frame_query(qi).unwrap();
        if info.key == key53 {
            saw53 = true;
            let tag = player.resolve_tag(info.key).unwrap();
            assert_eq!(tag.id, 53);
            assert_eq!(tag.query_name, "bullet");
            assert_eq!(info.kind, RecQueryKind::CastRay);
        } else if info.key == key54 {
            saw54 = true;
            let tag = player.resolve_tag(info.key).unwrap();
            assert_eq!(tag.id, 54);
            assert_eq!(tag.query_name, "bullet");
        } else {
            saw_untagged = true;
            assert_eq!(info.key, 0);
        }
    }
    assert!(saw53 && saw54 && saw_untagged);
}

/// Scrub backward and verify per-frame hashes. (ScrubBackward)
#[test]
fn scrub_backward() {
    use crate::recording::{hash_world_state, RecPlayer};

    let mut rec = Recording::new(0);
    let mut world = World::new(&default_world_def());
    world_set_gravity(
        &mut world,
        Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        },
    );
    world_start_recording(&mut world, &mut rec);

    {
        let mut ground_def = default_body_def();
        ground_def.type_ = BodyType::Static;
        let ground_id = create_body(&mut world, &ground_def);
        let ground_box = make_box_hull(20.0, 1.0, 20.0);
        create_hull_shape(&mut world, ground_id, &default_shape_def(), &ground_box.base);
    }

    let mut box_shape = default_shape_def();
    box_shape.density = 1.0;
    for i in 0..4 {
        let box_hull = make_box_hull(0.5, 0.5, 0.5);
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: 0.0,
            y: (2.0 + i as f32 * 1.5) as _,
            z: 0.0,
        };
        let body_id = create_body(&mut world, &body_def);
        create_hull_shape(&mut world, body_id, &box_shape, &box_hull.base);
    }

    let total_frames = 80;
    for _ in 0..total_frames {
        world.step(1.0 / 60.0, 4);
    }
    world_stop_recording(&mut world);

    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    assert_eq!(player.get_frame_count(), total_frames);

    let mut hashes = vec![0u64; (total_frames + 1) as usize];
    while !player.is_at_end() {
        player.step_frame();
        let f = player.get_frame();
        if f <= total_frames {
            hashes[f as usize] = hash_world_state(player.world());
        }
    }
    assert_eq!(player.get_frame(), total_frames);
    assert!(!player.has_diverged());

    let seek_targets = [
        total_frames,
        total_frames / 2,
        5,
        total_frames - 1,
        0,
        1,
    ];
    for &target in &seek_targets {
        player.seek_frame(target);
        assert_eq!(player.get_frame(), target);
        assert!(!player.has_diverged());
        if target > 0 {
            assert_eq!(hash_world_state(player.world()), hashes[target as usize]);
        }
    }
}

/// Seek with custom hull geometry. (SeekWithHull)
#[test]
fn seek_with_hull() {
    use crate::recording::RecPlayer;

    let pts = [
        Vec3 {
            x: -1.0,
            y: -1.0,
            z: -1.0,
        },
        Vec3 {
            x: 1.0,
            y: -1.0,
            z: -1.0,
        },
        Vec3 {
            x: 1.0,
            y: 1.0,
            z: -1.0,
        },
        Vec3 {
            x: -1.0,
            y: 1.0,
            z: -1.0,
        },
        Vec3 {
            x: -1.0,
            y: -1.0,
            z: 1.0,
        },
        Vec3 {
            x: 1.0,
            y: -1.0,
            z: 1.0,
        },
        Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        },
        Vec3 {
            x: -1.0,
            y: 1.0,
            z: 1.0,
        },
    ];
    let hull = create_hull(&pts, 8).expect("hull");

    let mut rec = Recording::new(0);
    let mut world = World::new(&default_world_def());
    world_set_gravity(
        &mut world,
        Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        },
    );
    world_start_recording(&mut world, &mut rec);

    {
        let mut ground_def = default_body_def();
        ground_def.type_ = BodyType::Static;
        let ground_id = create_body(&mut world, &ground_def);
        let ground_box = make_box_hull(20.0, 1.0, 20.0);
        create_hull_shape(&mut world, ground_id, &default_shape_def(), &ground_box.base);
    }

    let mut sd = default_shape_def();
    sd.density = 1.0;
    for i in 0..3 {
        let mut bd = default_body_def();
        bd.type_ = BodyType::Dynamic;
        bd.position = Pos {
            x: ((i * 4) as f32 - 4.0) as _,
            y: 5.0,
            z: 0.0,
        };
        let body_id = create_body(&mut world, &bd);
        create_hull_shape(&mut world, body_id, &sd, &hull);
    }

    let total_frames = 40;
    for _ in 0..total_frames {
        world.step(1.0 / 60.0, 4);
    }
    world_stop_recording(&mut world);

    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    while !player.is_at_end() {
        player.step_frame();
    }
    assert!(!player.has_diverged());

    let mid = total_frames / 2;
    player.seek_frame(mid);
    assert_eq!(player.get_frame(), mid);
    assert!(!player.has_diverged());
    player.seek_frame(0);
    assert_eq!(player.get_frame(), 0);
}

/// Player accessors and keyframe policy. (PlayerAccessors)
#[test]
fn player_accessors() {
    use crate::body::body_get_type;
    use crate::recording::RecPlayer;

    let mut world = World::new(&default_world_def());
    world_set_gravity(
        &mut world,
        Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        },
    );

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    let ground_id = create_body(&mut world, &ground_def);
    let ground_box = make_box_hull(20.0, 1.0, 20.0);
    create_hull_shape(&mut world, ground_id, &default_shape_def(), &ground_box.base);

    let dynamic_count = 4;
    let mut box_shape = default_shape_def();
    box_shape.density = 1.0;
    for i in 0..dynamic_count {
        let box_hull = make_box_hull(0.5, 0.5, 0.5);
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: 0.0,
            y: (2.0 + i as f32 * 1.5) as _,
            z: 0.0,
        };
        let body_id = create_body(&mut world, &body_def);
        create_hull_shape(&mut world, body_id, &box_shape, &box_hull.base);
    }

    for _ in 0..10 {
        world.step(1.0 / 60.0, 4);
    }

    let mut rec = Recording::new(0);
    world_start_recording(&mut world, &mut rec);
    let total_frames = 80;
    let sub_step_count = 4;
    for _ in 0..total_frames {
        world.step(1.0 / 60.0, sub_step_count);
    }
    world_stop_recording(&mut world);

    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    let info = player.get_info();
    assert_eq!(info.frame_count, total_frames);
    assert_eq!(info.sub_step_count, sub_step_count);
    assert!(info.time_step > 0.0);
    let extent = Vec3 {
        x: info.bounds.upper_bound.x - info.bounds.lower_bound.x,
        y: info.bounds.upper_bound.y - info.bounds.lower_bound.y,
        z: info.bounds.upper_bound.z - info.bounds.lower_bound.z,
    };
    assert!(extent.x > 0.0 && extent.y > 0.0 && extent.z > 0.0);

    assert_eq!(player.get_body_count(), 1 + dynamic_count);
    let ground = player.get_body_id(0);
    assert!(!ground.is_null());
    assert_eq!(body_get_type(player.world(), ground), BodyType::Static);
    for i in 1..=dynamic_count {
        let id = player.get_body_id(i);
        assert!(!id.is_null());
        assert_eq!(body_get_type(player.world(), id), BodyType::Dynamic);
    }
    assert!(player.get_body_id(1 + dynamic_count).is_null());

    player.seek_frame(total_frames);
    assert!(!player.has_diverged());
    assert_eq!(player.get_diverge_frame(), -1);

    let before = player.get_body_id(2);
    player.seek_frame(total_frames / 2);
    player.seek_frame(total_frames);
    let after = player.get_body_id(2);
    assert_eq!(before.index1, after.index1);
    assert_eq!(before.generation, after.generation);

    assert_eq!(player.get_keyframe_min_interval(), 16);
    player.set_keyframe_policy(256 * 1024 * 1024, 8);
    assert_eq!(player.get_keyframe_min_interval(), 8);
    assert_eq!(player.get_keyframe_interval(), 8);
    assert_eq!(player.get_keyframe_budget(), 256 * 1024 * 1024);
    assert_eq!(player.get_keyframe_bytes(), 0);
}

/// Reserved header bytes are ignored by validate. (ReservedHeaderBytes)
#[test]
fn reserved_header_bytes() {
    let mut rec = Recording::new(0);
    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, &mut rec);
    world_set_gravity(
        &mut world,
        Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        },
    );
    for _ in 0..5 {
        world.step(1.0 / 60.0, 4);
    }
    world_stop_recording(&mut world);

    let mut patched = rec.data().to_vec();
    patched[11] = 0xAB;
    patched[16] = 0xCD;
    patched[20] = 0xEF;
    assert!(validate_replay(&patched, 1));
}
/// Geometry hash collision / chain intern. (GeometryHashCollision)
#[test]
fn geometry_hash_collision() {
    use crate::recording::{hash64_blob, GeometryKind, GeometryRegistry};

    let n = 16;
    let shared_hash = 0xABCD1234u64;

    {
        let p = vec![0x11u8; 16];
        let mut q = vec![0x11u8; 16];
        q[7] = 0x12;
        let hp = hash64_blob(&p);
        let hq = hash64_blob(&q);
        assert_ne!(hp, hq);
        assert_ne!((hp >> 32) as u32, (hq >> 32) as u32);
        let _ = (p, q);
    }

    let mut reg = GeometryRegistry::new();
    let blob_a = vec![0xAAu8; n];
    let blob_b = vec![0xBBu8; n];
    let id_a = reg.intern(GeometryKind::Hull, shared_hash, blob_a.clone());
    let id_b = reg.intern(GeometryKind::Hull, shared_hash, blob_b.clone());
    assert_ne!(id_a, id_b);
    assert_eq!(reg.entries.len(), 2);

    assert_eq!(
        reg.intern(GeometryKind::Hull, shared_hash, blob_a.clone()),
        id_a
    );
    assert_eq!(reg.entries.len(), 2);
    assert_eq!(
        reg.intern(GeometryKind::Hull, shared_hash, blob_b.clone()),
        id_b
    );
    assert_eq!(reg.entries.len(), 2);

    let mut seeded = GeometryRegistry::new();
    let slot0 = vec![0xAAu8; n];
    let slot1 = vec![0xBBu8; n];
    let slot2 = vec![0xAAu8; n];
    assert_eq!(seeded.append(GeometryKind::Hull, shared_hash, slot0.clone()), 0);
    assert_eq!(seeded.append(GeometryKind::Hull, shared_hash, slot1), 1);
    assert_eq!(seeded.append(GeometryKind::Hull, shared_hash, slot2), 2);

    let live = vec![0xAAu8; n];
    let resolved = seeded.intern(GeometryKind::Hull, shared_hash, live);
    assert_eq!(seeded.entries.len(), 3);
    assert!(resolved == 0 || resolved == 2);
    assert_eq!(seeded.entries[resolved as usize].bytes, slot0);
}

/// Shape names survive create + SetName through replay. (ShapeNameReplay)
#[test]
fn shape_name_replay() {
    use crate::recording::RecPlayer;
    use crate::shape::{shape_get_name, shape_set_name};

    let names = [
        "def",
        "set",
        "abcdefghijklmnopqrstuvwxyz",
    ];

    let mut rec = Recording::new(0);
    let mut world = World::new(&default_world_def());
    world_start_recording(&mut world, &mut rec);

    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    let mut shape_ids = Vec::new();
    for (i, name) in names.iter().enumerate() {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: i as _,
            y: 1.0,
            z: 0.0,
        };
        let body_id = create_body(&mut world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.density = 1.0;
        if i == 0 {
            shape_def.name = (*name).into();
        }
        let sid = create_sphere_shape(&mut world, body_id, &shape_def, &sphere);
        if i == 1 {
            shape_set_name(&mut world, sid, name);
        } else if i == 2 {
            shape_set_name(&mut world, sid, name);
        }
        shape_ids.push(sid);
    }

    for _ in 0..5 {
        world.step(1.0 / 60.0, 4);
    }
    world_stop_recording(&mut world);
    assert!(validate_replay(rec.data(), 1));

    let mut player = RecPlayer::create(rec.data(), 1).expect("player");
    while !player.is_at_end() {
        player.step_frame();
    }
    assert!(!player.has_diverged());

    for (i, name) in names.iter().enumerate() {
        // Names now round-trip at full length through the name cache.
        let expected: String = (*name).to_string();
        let replay_id = crate::id::ShapeId {
            index1: shape_ids[i].index1,
            world0: player.world().world_id,
            generation: shape_ids[i].generation,
        };
        let got = shape_get_name(player.world(), replay_id);
        assert_eq!(got, expected, "shape {i}");
    }
}
