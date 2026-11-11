//! Headless Dynamic Mover tests: build the real scene and drive the real
//! `DynamicMoverSample::step` with scripted keys / camera / mouse — walk, jump, sprint,
//! walk-slow, aim + shoot, ride the horizontal platform, swap to the box mover and back,
//! raise the gravity scale.
//!
//! The same script runs in a C++ driver compiled against the C reference at upstream
//! 38f2ae1, whose trace is unchanged from 3362633 (`BOX3D_DISABLE_SIMD`,
//! `-ffp-contract=off`, the `DynamicMoverSample` scene code copied verbatim from
//! `samples/sample_character.cpp` + the vendored
//! `samples/dynamic_mover.cpp`). Its per-step trace (mover position / velocity as raw
//! float bits, ground / walkable / jumping flags, probe count, gems left, plate state,
//! first ball position, and every 30 steps an order-sensitive hash of every body
//! transform in broad-phase query order) hashes to [`C_TRACE_FNV1A_F32`], and to
//! [`C_TRACE_FNV1A_F64`] with the C reference built in large world mode. The Rust trace
//! matches both byte-for-byte.

use super::camera::KeysDown;
use super::*;
use box3d_rust::body::{body_get_linear_velocity, body_get_transform};
use box3d_rust::math_functions::DEG_TO_RAD;

/// FNV-1a 64 of the C driver's 900-step trace text (`f32`, see module docs).
const C_TRACE_FNV1A_F32: u64 = 0x087c_fac3_1140_c399;
/// The same trace from the C reference built with `BOX3D_DOUBLE_PRECISION` (large world
/// mode, the `double-precision` feature of `box3d-rust`).
const C_TRACE_FNV1A_F64: u64 = 0x9941_7400_f149_ba1e;

/// `b3Pos` is `f64` in large world mode.
const LARGE_WORLD: bool = std::mem::size_of::<Pos>() == 24;

const SCRIPT_STEPS: usize = 900;

/// The scripted keys + camera yaw for `step` (identical to the C driver's script).
fn script_input(step: usize) -> (KeysDown, f32) {
    let mut keys = KeysDown::default();
    let mut yaw_deg = 180.0f32;
    if (30..150).contains(&step) {
        keys.w = true;
        keys.d = true;
    }
    if (160..170).contains(&step) || (200..206).contains(&step) {
        keys.space = true;
    }
    if (220..300).contains(&step) {
        yaw_deg = 90.0;
        keys.w = true;
        keys.shift = true;
    }
    if (300..360).contains(&step) {
        yaw_deg = 90.0;
        keys.s = true;
        keys.control = true;
    }
    if (600..700).contains(&step) {
        yaw_deg = 0.0;
        keys.w = true;
    }
    if (720..725).contains(&step) {
        keys.space = true;
    }
    if step >= 740 {
        yaw_deg = 270.0;
        keys.w = true;
        keys.d = true;
    }
    (keys, yaw_deg)
}

/// Apply the script's one-shot events before `step` (mouse buttons, the platform
/// teleport, the shape swaps, the gravity change).
fn script_events(world: &mut World, sample: &mut DynamicMoverSample, step: usize) {
    match step {
        360 => sample.mouse_down(1),
        372 | 384 | 396 => sample.mouse_down(0),
        400 => sample.mouse_up(1),
        420 => {
            let mover_id = sample.mover.body_id();
            let p = body_get_position(world, sample.platform_ids[1]);
            let target = Pos {
                x: p.x,
                y: 5.4f32 as _,
                z: p.z,
            };
            let rotation = body_get_transform(world, mover_id).q;
            body_set_transform(world, mover_id, target, rotation);
            body_set_linear_velocity(world, mover_id, VEC3_ZERO);
        }
        450 | 520 => {
            sample.use_box = !sample.use_box;
            sample.swap_mover_shape(world);
        }
        700 => sample.mover.set_gravity_scale(world, 3.0),
        _ => {}
    }
}

/// Run the script, returning the trace text in the C driver's format.
fn run_script(steps: usize) -> (String, World, DynamicMoverSample) {
    let (mut world, mut sample) = build_dynamic_mover();
    let mut trace = String::new();
    for step in 0..steps {
        let (keys, yaw_deg) = script_input(step);
        sample.keys = keys;
        sample.camera.yaw = yaw_deg * DEG_TO_RAD;
        sample.camera.pitch = 20.0 * DEG_TO_RAD;
        sample.camera.update_transform();

        script_events(&mut world, &mut sample, step);

        sample.step(&mut world, 1.0 / 60.0, 4);

        let mover_id = sample.mover.body_id();
        let p = body_get_position(&world, mover_id);
        let v = body_get_linear_velocity(&world, mover_id);
        let m = &sample.mover;
        trace.push_str(&format!(
            "{} {:08x} {:08x} {:08x} {:08x} {:08x} {:08x} g{} w{} j{} n{} gems{} plate{}",
            step,
            (p.x as f32).to_bits(),
            (p.y as f32).to_bits(),
            (p.z as f32).to_bits(),
            v.x.to_bits(),
            v.y.to_bits(),
            v.z.to_bits(),
            m.on_ground as i32,
            m.walkable as i32,
            m.jumping as i32,
            m.probe_count,
            sample.gems_remaining(),
            sample.plate_armed as i32,
        ));
        let ball = sample.ball_ids[0];
        if body_is_valid(&world, ball) {
            let bp = body_get_position(&world, ball);
            trace.push_str(&format!(
                " ball {:08x} {:08x} {:08x}",
                (bp.x as f32).to_bits(),
                (bp.y as f32).to_bits(),
                (bp.z as f32).to_bits()
            ));
        }
        if step % 30 == 29 {
            let (count, hash) = world_hash(&world);
            trace.push_str(&format!(" world {} {:016x}", count, hash));
        }
        trace.push('\n');
    }
    (trace, world, sample)
}

/// Every shape in the broad-phase, in tree query order: FNV-1a over its body transform
/// bits (the C driver's `HashShape`). Order-sensitive, so it also checks the trees.
fn world_hash(world: &World) -> (usize, u64) {
    use box3d_rust::math_functions::{Aabb, Vec3};
    use box3d_rust::types::default_query_filter;
    use box3d_rust::world::world_overlap_aabb;
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut count = 0usize;
    let hash_u32 = |h: &mut u64, u: u32| {
        for i in 0..4 {
            *h ^= ((u >> (8 * i)) & 0xff) as u64;
            *h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
    };
    let bounds = Aabb {
        lower_bound: Vec3 {
            x: -1000.0,
            y: -1000.0,
            z: -1000.0,
        },
        upper_bound: Vec3 {
            x: 1000.0,
            y: 1000.0,
            z: 1000.0,
        },
    };
    world_overlap_aabb(world, bounds, &default_query_filter(), |shape_id| {
        let xf = body_get_transform(world, shape_get_body(world, shape_id));
        for f in [
            xf.p.x as f32,
            xf.p.y as f32,
            xf.p.z as f32,
            xf.q.v.x,
            xf.q.v.y,
            xf.q.v.z,
            xf.q.s,
        ] {
            hash_u32(&mut h, f.to_bits());
        }
        count += 1;
        true
    });
    (count, h)
}

fn fnv1a(text: &str) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in text.bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// Dump the trace for diffing against the C driver:
/// `DYNAMIC_MOVER_TRACE=/path/out.txt cargo test dump_dynamic_mover_trace -- --ignored`.
#[test]
#[ignore]
fn dump_dynamic_mover_trace() {
    let (trace, _, _) = run_script(SCRIPT_STEPS);
    let path = std::env::var("DYNAMIC_MOVER_TRACE").expect("set DYNAMIC_MOVER_TRACE");
    std::fs::write(path, trace).expect("write trace");
}

#[test]
fn scripted_run_matches_c_trace() {
    let (trace, _, _) = run_script(SCRIPT_STEPS);
    let expected = if LARGE_WORLD {
        C_TRACE_FNV1A_F64
    } else {
        C_TRACE_FNV1A_F32
    };
    assert_eq!(fnv1a(&trace), expected, "Rust trace diverged from C");
}

/// Step with the given keys and camera yaw (pitch 20°, the sample's default view).
fn step_with(world: &mut World, sample: &mut DynamicMoverSample, keys: KeysDown, yaw_deg: f32) {
    sample.keys = keys;
    sample.camera.yaw = yaw_deg * DEG_TO_RAD;
    sample.camera.pitch = 20.0 * DEG_TO_RAD;
    sample.camera.update_transform();
    sample.step(world, 1.0 / 60.0, 4);
}

fn teleport(world: &mut World, sample: &DynamicMoverSample, p: Pos) {
    let mover_id = sample.mover.body_id();
    let rotation = body_get_transform(world, mover_id).q;
    body_set_transform(world, mover_id, p, rotation);
    body_set_linear_velocity(world, mover_id, VEC3_ZERO);
}

#[test]
fn walk_collects_the_first_gem_and_jump_lifts_off() {
    let (mut world, mut sample) = build_dynamic_mover();
    let idle = KeysDown::default();
    for _ in 0..30 {
        step_with(&mut world, &mut sample, idle, 180.0);
    }
    assert!(
        sample.mover.on_ground && sample.mover.walkable,
        "settles on the ground"
    );
    let start = body_get_position(&world, sample.mover.body_id());
    assert!(
        (start.y as f32 - 1.4).abs() < 0.05,
        "pogo holds the capsule center ~1.4 m up, got {}",
        start.y
    );

    // Yaw 180° looks down +z, so W walks +z through the gem at { 0, 1.2, 5 }.
    let walk = KeysDown {
        w: true,
        ..KeysDown::default()
    };
    for _ in 0..70 {
        step_with(&mut world, &mut sample, walk, 180.0);
    }
    let walked = body_get_position(&world, sample.mover.body_id());
    assert!(walked.z as f32 > 5.5, "walked forward, z = {}", walked.z);
    assert!(
        (walked.x as f32).abs() < 0.01,
        "kept heading, x = {}",
        walked.x
    );
    let speed = body_get_linear_velocity(&world, sample.mover.body_id()).z;
    assert!((speed - 6.0).abs() < 0.2, "reached max speed, vz = {speed}");
    assert_eq!(sample.gems_remaining(), GEM_COUNT - 1, "collected the gem");

    // Stop, then jump: a fresh press launches at the jump speed.
    for _ in 0..40 {
        step_with(&mut world, &mut sample, idle, 180.0);
    }
    let jump = KeysDown {
        space: true,
        ..KeysDown::default()
    };
    step_with(&mut world, &mut sample, jump, 180.0);
    assert!(sample.mover.jumping, "jump started");
    let vy = body_get_linear_velocity(&world, sample.mover.body_id()).y;
    assert!(vy > 5.0, "launched upward, vy = {vy}");
    let mut peak = 0.0f32;
    for _ in 0..20 {
        step_with(&mut world, &mut sample, jump, 180.0);
        peak = peak.max(body_get_position(&world, sample.mover.body_id()).y as f32);
    }
    assert!(peak > 2.5, "rose well above standing height, peak {peak}");
    assert!(!sample.jump_released, "holding Space does not re-jump");
    for _ in 0..90 {
        step_with(&mut world, &mut sample, idle, 180.0);
    }
    assert!(sample.mover.on_ground && !sample.mover.jumping, "landed");
}

#[test]
fn mover_rides_the_horizontal_platform() {
    let (mut world, mut sample) = build_dynamic_mover();
    let idle = KeysDown::default();
    for _ in 0..10 {
        step_with(&mut world, &mut sample, idle, 180.0);
    }
    let p = body_get_position(&world, sample.platform_ids[1]);
    teleport(
        &mut world,
        &sample,
        Pos {
            x: p.x,
            y: 5.4f32 as _,
            z: p.z,
        },
    );
    for _ in 0..180 {
        step_with(&mut world, &mut sample, idle, 180.0);
    }
    // The pogo found the platform material, so the mover joint moved onto it.
    assert_eq!(
        box3d_rust::joint::joint_get_body_a(&world, sample.mover.mover_joint_id),
        sample.platform_ids[1]
    );
    let mover = body_get_position(&world, sample.mover.body_id());
    let platform = body_get_position(&world, sample.platform_ids[1]);
    let moved = (platform.x - p.x) as f32;
    assert!(moved.abs() > 1.0, "the platform travelled, dx = {moved}");
    assert!(
        ((mover.x - platform.x) as f32).abs() < 0.3,
        "mover stayed on the platform: mover x {} platform x {}",
        mover.x,
        platform.x
    );
    assert!(sample.mover.on_ground);
}

#[test]
fn aim_and_shoot_launches_a_ball_along_the_view() {
    let (mut world, mut sample) = build_dynamic_mover();
    let idle = KeysDown::default();
    for _ in 0..20 {
        step_with(&mut world, &mut sample, idle, 180.0);
    }
    sample.mouse_down(1);
    for _ in 0..15 {
        step_with(&mut world, &mut sample, idle, 180.0);
    }
    assert!(sample.aiming && sample.aim_blend > 0.99, "aim blend ran in");
    assert!(sample.body_alpha < 0.35, "mover fades for the aim camera");
    assert!(sample.view_radius <= 0.8 + 1e-6, "aim camera zoomed in");
    sample.mouse_down(0);
    step_with(&mut world, &mut sample, idle, 180.0);
    let ball = sample.ball_ids[0];
    assert!(body_is_valid(&world, ball), "a ball was fired");
    let v = body_get_linear_velocity(&world, ball);
    let dir = -sample.camera.get_forward();
    let along = v.x * dir.x + v.y * dir.y + v.z * dir.z;
    assert!(
        along > 45.0,
        "ball flies along the view at ~50 m/s, got {along}"
    );
    sample.mouse_up(1);
    for _ in 0..15 {
        step_with(&mut world, &mut sample, idle, 180.0);
    }
    assert!(!sample.aiming && sample.aim_blend == 0.0 && sample.body_alpha == 1.0);
}

#[test]
fn pressure_plate_explodes_and_launches_the_mover() {
    let (mut world, mut sample) = build_dynamic_mover();
    let idle = KeysDown::default();
    teleport(
        &mut world,
        &sample,
        Pos {
            x: PLATE_CENTER.x,
            y: 1.6f32 as _,
            z: PLATE_CENTER.z,
        },
    );
    let mut fired = false;
    for _ in 0..120 {
        step_with(&mut world, &mut sample, idle, 180.0);
        if !sample.plate_armed {
            fired = true;
            break;
        }
    }
    assert!(
        fired,
        "the mover's weight pressed the plate through its travel"
    );
    assert!(sample.mover.jumping, "the plate launched the mover");
}

#[test]
fn box_mover_swap_round_trips() {
    use box3d_rust::geometry::ShapeType;
    use box3d_rust::shape::shape_get_type;
    let (mut world, mut sample) = build_dynamic_mover();
    let mover_id = sample.mover.body_id();
    sample.use_box = true;
    sample.swap_mover_shape(&mut world);
    let shape = body_get_shapes(&world, mover_id, 1)[0];
    assert_eq!(shape_get_type(&world, shape), ShapeType::Hull);
    let (bodies, mover_index) = sample.render_bodies(&world);
    assert_eq!(bodies[mover_index].kind, crate::vis::KIND_BOX);
    for _ in 0..60 {
        step_with(&mut world, &mut sample, KeysDown::default(), 180.0);
    }
    assert!(
        sample.mover.on_ground,
        "the box mover stands on the pogo too"
    );
    sample.use_box = false;
    sample.swap_mover_shape(&mut world);
    let shape = body_get_shapes(&world, mover_id, 1)[0];
    assert_eq!(shape_get_type(&world, shape), ShapeType::Capsule);
}
