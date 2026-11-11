//! Geometric Mover C-trace test: the real level + the real `mover::step`, driven by the
//! scripted keys / camera of a C++ driver compiled against the C reference at upstream
//! 3362633 (`BOX3D_DISABLE_SIMD`, `-ffp-contract=off`; the `GeometricMoverSample` scene
//! and methods copied from `samples/sample_character.cpp`, the vendored
//! `samples/geometric_mover.cpp`, and `samples/mesh_loader.cpp` for the OBJ levels). The
//! per-step trace (mover position / velocity / pogo velocity bits, ground flag, plane
//! count, box / tree impact speeds, and every 30 steps a hash of every body transform in
//! broad-phase order) must match the C trace byte-for-byte in both precision modes.

use super::super::dynamic::camera::KeysDown;
use super::super::{CharacterState, SceneState};
use super::{build_mover, step};
use box3d_rust::body::body_get_transform;
use box3d_rust::math_functions::{Aabb, Pos, Vec3, DEG_TO_RAD};
use box3d_rust::shape::shape_get_body;
use box3d_rust::types::default_query_filter;
use box3d_rust::world::{world_overlap_aabb, World};

const TEST_MAP_OBJ: &str =
    include_str!("../../../../../box3d-cpp-reference/data/meshes/test_map01.obj");
const STAIRS_OBJ: &str = include_str!("../../../../../box3d-cpp-reference/data/meshes/stairs.obj");

/// FNV-1a 64 of the C driver's 900-step trace (`f32` / large world `f64`).
const C_TRACE_FNV1A_F32: u64 = 0xb616_a904_325c_38d3;
const C_TRACE_FNV1A_F64: u64 = 0x7ba7_88af_d99d_54d5;

const LARGE_WORLD: bool = std::mem::size_of::<Pos>() == 24;

fn script_input(step: usize) -> (KeysDown, f32) {
    let mut keys = KeysDown::default();
    let mut yaw_deg = 120.0f32;
    if (30..200).contains(&step) {
        keys.w = true;
    }
    if (200..210).contains(&step) {
        keys.space = true;
    }
    if (220..320).contains(&step) {
        yaw_deg = 200.0;
        keys.w = true;
        keys.shift = true;
    }
    if (330..400).contains(&step) {
        yaw_deg = 200.0;
        keys.s = true;
        keys.control = true;
    }
    if (400..600).contains(&step) {
        yaw_deg = 300.0;
        keys.w = true;
        keys.d = true;
    }
    if (450..455).contains(&step) {
        keys.space = true;
    }
    if step >= 600 {
        yaw_deg = 30.0;
        keys.w = true;
    }
    (keys, yaw_deg)
}

fn world_hash(world: &World, dump: bool) -> (usize, u64) {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut count = 0usize;
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
        let body_id = shape_get_body(world, shape_id);
        let xf = body_get_transform(world, body_id);
        if dump {
            println!(
                "  shape {} body {} {:08x} {:08x} {:08x} {:08x} {:08x} {:08x} {:08x}",
                shape_id.index1,
                body_id.index1,
                (xf.p.x as f32).to_bits(),
                (xf.p.y as f32).to_bits(),
                (xf.p.z as f32).to_bits(),
                xf.q.v.x.to_bits(),
                xf.q.v.y.to_bits(),
                xf.q.v.z.to_bits(),
                xf.q.s.to_bits()
            );
        }
        for f in [
            xf.p.x as f32,
            xf.p.y as f32,
            xf.p.z as f32,
            xf.q.v.x,
            xf.q.v.y,
            xf.q.v.z,
            xf.q.s,
        ] {
            for i in 0..4 {
                h ^= ((f.to_bits() >> (8 * i)) & 0xff) as u64;
                h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
        count += 1;
        true
    });
    (count, h)
}

fn run_script(steps: usize) -> String {
    let mut state: CharacterState = build_mover(TEST_MAP_OBJ, STAIRS_OBJ);
    state.third_person = true;
    let mut trace = String::new();
    for i in 0..steps {
        let (keys, yaw_deg) = script_input(i);
        if let SceneState::Mover(m) = &mut state.state {
            m.keys = keys;
            m.camera.yaw = yaw_deg * DEG_TO_RAD;
            m.camera.pitch = 30.0 * DEG_TO_RAD;
            m.camera.update_transform();
        }
        if i == 500 {
            state.clip_velocity = false;
        }
        if i == 700 {
            state.clip_velocity = true;
        }

        state.render_capsules.clear();
        state.debug_segs.clear();
        state.debug_pts.clear();
        state.debug_text.clear();
        step(&mut state, 1.0 / 60.0, 4);

        let SceneState::Mover(scene) = &state.state else {
            unreachable!("the Geometric Mover scene")
        };
        let m = &scene.mover;
        trace.push_str(&format!(
            "{} {:08x} {:08x} {:08x} {:08x} {:08x} {:08x} {:08x} g{} p{}",
            i,
            (m.position.x as f32).to_bits(),
            (m.position.y as f32).to_bits(),
            (m.position.z as f32).to_bits(),
            m.velocity.x.to_bits(),
            m.velocity.y.to_bits(),
            m.velocity.z.to_bits(),
            m.pogo_velocity.to_bits(),
            m.on_ground as i32,
            m.planes.len(),
        ));
        if scene.box_impact.valid {
            trace.push_str(&format!(" box {:08x}", scene.box_impact.speed.to_bits()));
        }
        if scene.tree_impact.valid {
            trace.push_str(&format!(" tree {:08x}", scene.tree_impact.speed.to_bits()));
        }
        let dump = std::env::var("GM_DUMP_STEP")
            .ok()
            .and_then(|v| v.parse::<usize>().ok())
            .is_some_and(|d| i >= d && i <= d + 2);
        if dump {
            println!("{}", trace.lines().last().unwrap_or(""));
        }
        if i % 30 == 29 || dump {
            let (count, hash) = world_hash(&state.world, dump);
            trace.push_str(&format!(" world {} {:016x}", count, hash));
        }
        trace.push('\n');
    }
    trace
}

fn fnv1a(text: &str) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in text.bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// `GEOMETRIC_MOVER_TRACE=/path/out.txt cargo test dump_geometric_mover_trace -- --ignored`.
#[test]
#[ignore]
fn dump_geometric_mover_trace() {
    let path = std::env::var("GEOMETRIC_MOVER_TRACE").expect("set GEOMETRIC_MOVER_TRACE");
    std::fs::write(path, run_script(900)).expect("write trace");
}

#[test]
fn scripted_run_matches_c_trace() {
    let expected = if LARGE_WORLD {
        C_TRACE_FNV1A_F64
    } else {
        C_TRACE_FNV1A_F32
    };
    assert_eq!(
        fnv1a(&run_script(900)),
        expected,
        "Rust trace diverged from C"
    );
}
