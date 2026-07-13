//! Character-mover finiteness regression tests (`#[path]`-included by
//! `character_demo.rs`). These drive the real `solve_move` + `World::step` loop
//! the wasm `character_step` runs and assert every float the renderer reads
//! (`character_debug_lines` / the mover pose) stays finite — guarding the
//! reported `computeBoundingSphere: NaN`. See also the library-level mover
//! integration test in `src/mover_tests.rs`, which runs under the root
//! `cargo test` (and `--features double-precision`).

use super::*;

/// Build a BasicMover-scene `CharacterState` directly (the same setup
/// `character_reset_ex(0, _)` performs), so tests can drive the real
/// `solve_move` + `World::step` loop the wasm `character_step` runs.
fn basic_mover_state() -> CharacterState {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let (hf, hf_origin, start) = build_basic_mover(&mut world, &mut bodies);
    CharacterState {
        world,
        bodies,
        hf: Some(hf),
        hf_origin,
        mover_pos: start,
        velocity: VEC3_ZERO,
        capsule: mover_capsule(),
        pogo_velocity: 0.0,
        on_ground: false,
        sprint: false,
        throttle_x: 0.0,
        throttle_y: 0.0,
        jump: false,
        want_sprint: false,
        forward: Vec3 {
            x: 0.0,
            y: 0.0,
            z: -1.0,
        },
        right: Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        pogo_origin: VEC3_ZERO,
        pogo_end: VEC3_ZERO,
        pogo_hit: false,
        village_buildings: Vec::new(),
        village_stats: [0.0; 7],
        village_ground_index: -1,
    }
}

/// Every float `character_debug_lines` (and the mover pose) exposes must stay
/// finite; a single NaN reaches the renderer's `computeBoundingSphere`.
fn assert_finite(state: &CharacterState, step: i32) {
    let vals = [
        ("mover_pos.x", state.mover_pos.x as f32),
        ("mover_pos.y", state.mover_pos.y as f32),
        ("mover_pos.z", state.mover_pos.z as f32),
        ("velocity.x", state.velocity.x),
        ("velocity.y", state.velocity.y),
        ("velocity.z", state.velocity.z),
        ("pogo_origin.x", state.pogo_origin.x),
        ("pogo_origin.y", state.pogo_origin.y),
        ("pogo_origin.z", state.pogo_origin.z),
        ("pogo_end.x", state.pogo_end.x),
        ("pogo_end.y", state.pogo_end.y),
        ("pogo_end.z", state.pogo_end.z),
        ("pogo_velocity", state.pogo_velocity),
    ];
    for (name, v) in vals {
        assert!(
            v.is_finite(),
            "non-finite {name} = {v} at step {step} (mover_pos={:?}, velocity={:?})",
            state.mover_pos,
            state.velocity
        );
    }
}

/// Point the mover's camera-relative axes at a world-space target (unit XZ),
/// mirroring `character_set_input`'s normalization.
fn aim_at(state: &mut CharacterState, tx: f32, tz: f32) {
    let dx = tx - state.mover_pos.x as f32;
    let dz = tz - state.mover_pos.z as f32;
    let mut len = 0.0;
    let fwd = get_length_and_normalize(
        &mut len,
        Vec3 {
            x: dx,
            y: 0.0,
            z: dz,
        },
    );
    if len >= 1e-4 {
        state.forward = fwd;
        state.right = Vec3 {
            x: fwd.z,
            y: 0.0,
            z: -fwd.x,
        };
    }
}

#[test]
fn mover_debug_lines_stay_finite() {
    let dt = 1.0 / 60.0;
    let mut state = basic_mover_state();

    // Walk the mover around the wave terrain and obstacles, jumping and
    // sprinting periodically so the airborne launch, apex, and landing
    // (ground re-acquisition) transitions are all exercised.
    for step in 0..900 {
        state.throttle_x = 1.0;
        state.throttle_y = if (step / 60) % 2 == 0 { 0.5 } else { -0.5 };
        if step % 75 == 0 {
            state.jump = true;
        }
        state.want_sprint = step % 40 < 18;

        solve_move(&mut state, dt);
        state.world.step(dt, 4);

        assert_finite(&state, step);
    }
}

#[test]
fn mover_stays_finite_driving_into_obstacles() {
    let dt = 1.0 / 60.0;
    let mut state = basic_mover_state();

    // The BasicMover scene's obstacle / capsule / box coordinates. Ram the
    // mover straight into each (wedging it into corners and between the two
    // 1-unit-apart static capsules), sprinting, so degenerate contact-plane
    // sets and stuck solves are exercised.
    let targets = [
        (0.0f32, 6.0f32), // enemy capsule
        (0.0, 5.0),       // friendly capsule (both, ~1 unit apart)
        (4.0, 14.0),      // overlapping box pair
        (5.8, 13.7),      // adjacent box
        (-3.0, 2.0),      // lone box
        (2.0, -5.0),      // flat slab
        (7.0, -3.0),      // low slab
        (7.0, 0.0),       // dynamic sphere start (x,z)
        (100.0, 100.0),   // off the terrain edge → freefall
        (-100.0, -100.0), // off the far edge
    ];

    let mut step = 0i32;
    for &(tx, tz) in &targets {
        for _ in 0..240 {
            aim_at(&mut state, tx, tz);
            state.throttle_x = 1.0;
            state.throttle_y = 0.0;
            state.want_sprint = true;
            if step % 50 == 0 {
                state.jump = true;
            }
            solve_move(&mut state, dt);
            state.world.step(dt, 4);
            assert_finite(&state, step);
            step += 1;
        }
    }
}

#[test]
fn mover_stays_finite_in_village() {
    let dt = 1.0 / 60.0;
    let mut world = new_world();
    let mut bodies = Vec::new();
    let walk = build_village_ground(&mut world, &mut bodies, 10);
    let mut state = CharacterState {
        world,
        bodies,
        hf: None,
        hf_origin: VEC3_ZERO,
        mover_pos: walk.start,
        velocity: VEC3_ZERO,
        capsule: mover_capsule(),
        pogo_velocity: 0.0,
        on_ground: false,
        sprint: false,
        throttle_x: 0.0,
        throttle_y: 0.0,
        jump: false,
        want_sprint: false,
        forward: Vec3 {
            x: 0.0,
            y: 0.0,
            z: -1.0,
        },
        right: Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        pogo_origin: VEC3_ZERO,
        pogo_end: VEC3_ZERO,
        pogo_hit: false,
        village_buildings: walk.buildings,
        village_stats: walk.stats,
        village_ground_index: walk.ground_index,
    };

    for step in 0..900 {
        let angle = (step as f32) * 0.05;
        state.forward = Vec3 {
            x: angle.cos(),
            y: 0.0,
            z: angle.sin(),
        };
        state.right = Vec3 {
            x: state.forward.z,
            y: 0.0,
            z: -state.forward.x,
        };
        state.throttle_x = 1.0;
        state.want_sprint = step % 40 < 18;
        if step % 75 == 0 {
            state.jump = true;
        }
        solve_move(&mut state, dt);
        state.world.step(dt, 4);
        assert_finite(&state, step);
    }
}

#[test]
fn mover_stays_finite_under_random_input() {
    let dt = 1.0 / 60.0;
    let mut state = basic_mover_state();

    // Deterministic pseudo-random walk (fixed LCG seed): jitter throttle,
    // heading, jump and sprint every frame to blanket the state space.
    let mut rng: u32 = 0x1234_5678;
    let mut next = || {
        rng = rng.wrapping_mul(1664525).wrapping_add(1013904223);
        (rng >> 8) as f32 / (1u32 << 24) as f32 // [0,1)
    };

    for step in 0..3000 {
        let angle = next() * std::f32::consts::TAU;
        state.forward = Vec3 {
            x: angle.cos(),
            y: 0.0,
            z: angle.sin(),
        };
        state.right = Vec3 {
            x: state.forward.z,
            y: 0.0,
            z: -state.forward.x,
        };
        state.throttle_x = 2.0 * next() - 1.0;
        state.throttle_y = 2.0 * next() - 1.0;
        state.want_sprint = next() > 0.4;
        if next() > 0.85 {
            state.jump = true;
        }
        solve_move(&mut state, dt);
        state.world.step(dt, 4);
        assert_finite(&state, step);
    }
}
