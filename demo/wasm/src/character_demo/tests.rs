//! Character-scene finiteness regression tests (`#[path]`-included by
//! `character_demo/mod.rs`). These drive the real production step paths the wasm
//! `character_step` runs — `mover::step`, `rigid_body::step`, `overlap::step` — and
//! assert every float the renderer reads (poses, debug segments/points, status)
//! stays finite, guarding the reported `computeBoundingSphere: NaN`. See also the
//! library-level mover integration test in `src/mover_tests.rs`, which runs under
//! the root `cargo test` (and `--features double-precision`).

use super::*;

// The real Character level assets (pinned C submodule), embedded so the tests
// drive the true scene the browser loads.
const TEST_MAP_OBJ: &str =
    include_str!("../../../../box3d-cpp-reference/data/meshes/test_map01.obj");
const STAIRS_OBJ: &str = include_str!("../../../../box3d-cpp-reference/data/meshes/stairs.obj");
const BUILDING_OBJ: &str = include_str!("../../../../box3d-cpp-reference/data/meshes/building.obj");
const VOXEL1_OBJ: &str =
    include_str!("../../../../box3d-cpp-reference/data/meshes/voxel_mesh_01.obj");
const VOXEL2_OBJ: &str =
    include_str!("../../../../box3d-cpp-reference/data/meshes/voxel_mesh_02.obj");

/// Assert every renderer-visible float in the scene is finite.
fn assert_scene_finite(state: &CharacterState, step: i32) {
    for (i, v) in state.debug_segs.iter().enumerate() {
        assert!(
            v.is_finite(),
            "non-finite debug_seg[{i}] = {v} at step {step}"
        );
    }
    for (i, v) in state.debug_pts.iter().enumerate() {
        assert!(
            v.is_finite(),
            "non-finite debug_pt[{i}] = {v} at step {step}"
        );
    }
    for (i, v) in state.status.iter().enumerate() {
        assert!(v.is_finite(), "non-finite status[{i}] = {v} at step {step}");
    }
    for rc in &state.render_capsules {
        let t = rc.transform;
        for v in [t.p.x as f32, t.p.y as f32, t.p.z as f32, t.q.s] {
            assert!(
                v.is_finite(),
                "non-finite render capsule transform at step {step}"
            );
        }
    }
}

fn set_heading(state: &mut CharacterState, fwd_x: f32, fwd_z: f32) {
    state.input.forward = Vec3 {
        x: fwd_x,
        y: 0.0,
        z: fwd_z,
    };
    state.input.right = Vec3 {
        x: fwd_z,
        y: 0.0,
        z: -fwd_x,
    };
}

/// Drive the Geometric Mover the way the page does: third person on, the C key table
/// (`GetMoverInput`) and a camera yaw whose look direction is the requested heading.
#[allow(clippy::too_many_arguments)]
fn drive_mover(
    state: &mut CharacterState,
    fwd_x: f32,
    fwd_z: f32,
    throttle_x: f32,
    throttle_y: f32,
    jump: bool,
    sprint: bool,
    walk: bool,
) {
    state.third_person = true;
    if let SceneState::Mover(m) = &mut state.state {
        m.keys = dynamic::camera::KeysDown {
            w: throttle_x > 0.33,
            s: throttle_x < -0.33,
            d: throttle_y > 0.33,
            a: throttle_y < -0.33,
            space: jump,
            shift: sprint,
            control: walk,
        };
        // forward = -camera forward (horizontal): -(sin yaw, cos yaw).
        m.camera.yaw = (-fwd_x).atan2(-fwd_z);
        m.camera.pitch = 0.3;
        m.camera.update_transform();
    }
}

#[test]
fn mover_scene_stays_finite_walking_the_level() {
    let dt = 1.0 / 60.0;
    let mut state = mover::build_mover(TEST_MAP_OBJ, STAIRS_OBJ);

    // Walk the mover around the real level, jumping and sprinting periodically so
    // the airborne launch / apex / landing (ground re-acquisition) transitions and
    // the dynamic-body push loop (door, sphere) are all exercised.
    for step in 0..900 {
        let angle = (step as f32) * 0.03;
        let strafe = if (step / 60) % 2 == 0 { 0.5 } else { -0.5 };
        let walk = step % 200 > 170;
        drive_mover(
            &mut state,
            angle.cos(),
            angle.sin(),
            1.0,
            strafe,
            step % 75 == 0,
            step % 40 < 18,
            walk,
        );
        state.clip_velocity = step % 120 < 60; // toggle the Clip Velocity control

        mover::step(&mut state, dt, 4);
        assert_scene_finite(&state, step);
    }
}

#[test]
fn mover_scene_stays_finite_ramming_obstacles() {
    let dt = 1.0 / 60.0;
    let mut state = mover::build_mover(TEST_MAP_OBJ, STAIRS_OBJ);

    // Ram the mover straight into each scene feature (enemy/friendly capsules, the
    // ignore box, the spring door, off the terrain edge into freefall).
    let targets = [
        (0.0f32, 6.0f32), // enemy capsule
        (0.0, 5.0),       // friendly capsule (~1 unit apart)
        (7.0, -3.0),      // ignore box
        (-2.0, 0.0),      // spring door
        (7.0, 0.0),       // dynamic sphere landing
        (100.0, 100.0),   // off the terrain edge → freefall
        (-100.0, -100.0), // off the far edge
    ];

    let mut step = 0i32;
    for &(tx, tz) in &targets {
        for _ in 0..200 {
            let (px, pz) = if let SceneState::Mover(m) = &state.state {
                (m.mover.position.x as f32, m.mover.position.z as f32)
            } else {
                (0.0, 0.0)
            };
            let dx = tx - px;
            let dz = tz - pz;
            let len = (dx * dx + dz * dz).sqrt().max(1e-4);
            drive_mover(
                &mut state,
                dx / len,
                dz / len,
                1.0,
                0.0,
                step % 50 == 0,
                true,
                false,
            );
            mover::step(&mut state, dt, 4);
            assert_scene_finite(&state, step);
            step += 1;
        }
    }
}

#[test]
fn mover_scene_stays_finite_under_random_input() {
    let dt = 1.0 / 60.0;
    let mut state = mover::build_mover(TEST_MAP_OBJ, STAIRS_OBJ);

    // Deterministic pseudo-random walk (fixed LCG seed).
    let mut rng: u32 = 0x1234_5678;
    let mut next = || {
        rng = rng.wrapping_mul(1664525).wrapping_add(1013904223);
        (rng >> 8) as f32 / (1u32 << 24) as f32 // [0,1)
    };

    for step in 0..2000 {
        let angle = next() * std::f32::consts::TAU;
        let throttle_x = 2.0 * next() - 1.0;
        let throttle_y = 2.0 * next() - 1.0;
        let sprint = next() > 0.4;
        let jump = next() > 0.85;
        drive_mover(
            &mut state,
            angle.cos(),
            angle.sin(),
            throttle_x,
            throttle_y,
            jump,
            sprint,
            false,
        );
        state.clip_velocity = next() > 0.5;
        mover::step(&mut state, dt, 4);
        assert_scene_finite(&state, step);
    }
}

#[test]
fn rigid_body_scene_stays_finite() {
    let dt = 1.0 / 60.0;
    let mut state = rigid_body::build_rigid_body(
        TEST_MAP_OBJ,
        STAIRS_OBJ,
        BUILDING_OBJ,
        VOXEL1_OBJ,
        VOXEL2_OBJ,
    );

    // Drive the s&box character over the obstacle course: walk, sprint, jump, and
    // steer so the trace step-up (TryStep), Reground, and CategorizeGround traces
    // all run against real geometry.
    for step in 0..600 {
        let angle = (step as f32) * 0.05;
        set_heading(&mut state, angle.cos(), angle.sin());
        state.input.throttle_x = 1.0;
        state.input.throttle_y = if (step / 30) % 2 == 0 { 0.4 } else { -0.4 };
        state.input.want_sprint = step % 50 < 25;
        state.input.jump = step % 90 == 0;

        rigid_body::step(&mut state, dt, 4);
        assert_scene_finite(&state, step);

        if let SceneState::RigidBody(c) = &state.state {
            let p = c.position(&state.world);
            for v in [p.x as f32, p.y as f32, p.z as f32] {
                assert!(
                    v.is_finite(),
                    "non-finite rigid-body position at step {step}"
                );
            }
        }
    }
}

#[test]
fn overlap_scenes_have_no_degenerate_normals() {
    // CapsulePlane: bury the query capsule inside the box; plane data stays finite.
    {
        let mut state = overlap::build_capsule_plane();
        if let SceneState::Drag(s) = &mut state.state {
            s.transform.p = Pos {
                x: 0.0,
                y: 1.0,
                z: 1.0,
            };
        }
        overlap::step(&mut state);
        assert_scene_finite(&state, 0);
        // status = [plane_count, degenerate_count]; degenerate must be 0.
        assert_eq!(state.status.get(1).copied(), Some(0.0));
    }

    // MoverOverlap: bury the mover in each primitive; degenerate normals stay 0.
    for (x, y, z) in [(-3.0f32, 1.0, 0.0), (0.0, 1.0, 0.0), (3.0, 1.0, 0.0)] {
        let mut state = overlap::build_mover_overlap();
        if let SceneState::Drag(s) = &mut state.state {
            s.transform.p = Pos {
                x: x as _,
                y: y as _,
                z: z as _,
            };
        }
        overlap::step(&mut state);
        assert_scene_finite(&state, 0);
        assert_eq!(
            state.status.get(1).copied(),
            Some(0.0),
            "degenerate normal at overlap centre ({x},{y},{z})"
        );
    }
}
