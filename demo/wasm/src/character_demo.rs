//! Character mover demo — capsule mover on height-field terrain (sample BasicMover, simplified).

use crate::vis::{pos, push_poses, sphere, VisBody, KIND_CAPSULE, POSE_STRIDE};
use box3d_rust::body::create_body;
use box3d_rust::geometry::{Capsule, CollisionPlane};
use box3d_rust::height_field::{
    create_wave, get_height_field_triangle, get_height_field_triangle_count, HeightFieldData,
};
use box3d_rust::hull::make_box_hull;
use box3d_rust::math_functions::{
    dot, get_length_and_normalize, length, length_squared, max_float, mul_sv, offset_pos, sub_pos,
    Vec3, VEC3_AXIS_Y, VEC3_ZERO,
};
use box3d_rust::mover::{clip_vector, solve_planes};
use box3d_rust::shape::{create_height_field_shape, create_hull_shape, create_sphere_shape};
use box3d_rust::types::{
    default_body_def, default_query_filter, default_shape_def, default_world_def, BodyType,
};
use box3d_rust::world::{world_cast_mover, world_cast_ray_closest, world_collide_mover, World};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

const PLANE_CAPACITY: usize = 8;
const JUMP_SPEED: f32 = 5.0;
const MAX_SPEED: f32 = 6.0;
const MIN_SPEED: f32 = 0.01;
const STOP_SPEED: f32 = 1.0;
const ACCELERATE: f32 = 30.0;
const FRICTION: f32 = 4.0;
const MOVER_GRAVITY: f32 = 15.0;

thread_local! {
    static STATE: RefCell<Option<CharacterState>> = const { RefCell::new(None) };
}

struct CharacterState {
    world: World,
    /// Static scenery (boxes) + optional dynamic sphere for poses.
    bodies: Vec<VisBody>,
    hf: HeightFieldData,
    hf_origin: Vec3,
    mover_pos: box3d_rust::math_functions::Pos,
    velocity: Vec3,
    capsule: Capsule,
    pogo_velocity: f32,
    on_ground: bool,
    sprint: bool,
    /// Throttle: x = forward/back, y = strafe
    throttle_x: f32,
    throttle_y: f32,
    jump: bool,
    want_sprint: bool,
    /// Camera-relative forward/right on XZ (unit-ish).
    forward: Vec3,
    right: Vec3,
}

fn with_state<R>(f: impl FnOnce(&mut CharacterState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("character not initialized — call character_reset first"))
    })
}

fn new_world() -> World {
    let mut def = default_world_def();
    def.gravity = Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    World::new(&def)
}

fn solve_move(state: &mut CharacterState, time_step: f32) {
    if time_step <= 0.0 {
        return;
    }

    // Friction
    let speed = length(state.velocity);
    if speed < MIN_SPEED {
        state.velocity.x = 0.0;
        state.velocity.z = 0.0;
    } else {
        let control = if speed < STOP_SPEED {
            STOP_SPEED
        } else {
            speed
        };
        let drop = control * FRICTION * time_step;
        let new_speed = max_float(0.0, speed - drop);
        state.velocity *= new_speed / speed;
    }

    let max_speed = if state.sprint {
        1.5 * MAX_SPEED
    } else {
        MAX_SPEED
    };

    let desired_velocity = mul_sv(max_speed * state.throttle_x, state.forward)
        + mul_sv(max_speed * state.throttle_y, state.right);
    let mut desired_speed = 0.0;
    let desired_direction = get_length_and_normalize(&mut desired_speed, desired_velocity);
    let mut desired_velocity = desired_velocity;
    if desired_speed > max_speed {
        desired_velocity *= max_speed / desired_speed;
        desired_speed = max_speed;
    }

    if state.on_ground {
        state.velocity.y = 0.0;
    }

    let current_speed = dot(state.velocity, desired_direction);
    let add_speed = desired_speed - current_speed;
    if add_speed > 0.0 {
        let mut accel_speed = ACCELERATE * max_speed * time_step;
        if accel_speed > add_speed {
            accel_speed = add_speed;
        }
        state.velocity += mul_sv(accel_speed, desired_direction);
    }

    state.velocity.y -= MOVER_GRAVITY * time_step;

    // Pogo ground probe
    let pogo_rest_length = 3.0 * state.capsule.radius;
    let ray_length = pogo_rest_length + state.capsule.radius;
    let ray_origin = offset_pos(state.mover_pos, state.capsule.center1);
    let ray_translation = mul_sv(-ray_length, VEC3_AXIS_Y);
    let filter = default_query_filter();
    let ray_result = world_cast_ray_closest(&state.world, ray_origin, ray_translation, &filter);

    if !ray_result.hit {
        state.on_ground = false;
        state.pogo_velocity = 0.0;
    } else {
        state.on_ground = true;
        let pogo_current_length = ray_result.fraction * ray_length;
        let zeta = 0.7f32;
        let hertz = 4.0f32;
        let omega = 2.0 * std::f32::consts::PI * hertz;
        let omega_h = omega * time_step;
        state.pogo_velocity = (state.pogo_velocity
            - omega * omega_h * (pogo_current_length - pogo_rest_length))
            / (1.0 + 2.0 * zeta * omega_h + omega_h * omega_h);
    }

    if state.jump && state.on_ground {
        state.velocity.y = JUMP_SPEED;
        state.on_ground = false;
        state.jump = false;
    }
    state.sprint = state.on_ground && state.want_sprint;

    let target = offset_pos(
        state.mover_pos,
        mul_sv(time_step, state.velocity) + mul_sv(time_step * state.pogo_velocity, VEC3_AXIS_Y),
    );

    let mover_filter = default_query_filter();
    let cast_filter = default_query_filter();
    let tolerance = 0.01f32;

    for _ in 0..5 {
        let mut planes = [CollisionPlane::default(); PLANE_CAPACITY];
        let mut plane_count = 0usize;

        world_collide_mover(
            &state.world,
            state.mover_pos,
            &state.capsule,
            &mover_filter,
            |_shape_id, results| {
                for r in results {
                    if plane_count >= PLANE_CAPACITY {
                        break;
                    }
                    planes[plane_count] = CollisionPlane {
                        plane: r.plane,
                        push_limit: f32::MAX,
                        push: 0.0,
                        clip_velocity: true,
                    };
                    plane_count += 1;
                }
                true
            },
        );

        let target_delta = sub_pos(target, state.mover_pos);
        let result = solve_planes(target_delta, &mut planes[..plane_count]);
        let mut delta = result.delta;

        let fraction = world_cast_mover(
            &state.world,
            state.mover_pos,
            &state.capsule,
            delta,
            &cast_filter,
            None,
        );
        delta *= fraction;
        state.mover_pos = offset_pos(state.mover_pos, delta);

        if length_squared(delta) < tolerance * tolerance {
            break;
        }
    }

    // Clip velocity against planes from final position
    let mut planes = [CollisionPlane::default(); PLANE_CAPACITY];
    let mut plane_count = 0usize;
    world_collide_mover(
        &state.world,
        state.mover_pos,
        &state.capsule,
        &mover_filter,
        |_shape_id, results| {
            for r in results {
                if plane_count >= PLANE_CAPACITY {
                    break;
                }
                planes[plane_count] = CollisionPlane {
                    plane: r.plane,
                    push_limit: f32::MAX,
                    push: 0.0,
                    clip_velocity: true,
                };
                plane_count += 1;
            }
            true
        },
    );
    // Re-solve once so push flags are set for clipping
    let _ = solve_planes(VEC3_ZERO, &mut planes[..plane_count]);
    state.velocity = clip_vector(state.velocity, &planes[..plane_count]);
}

/// Reset character scene on a wave height field with a few obstacles.
#[wasm_bindgen]
pub fn character_reset() -> u32 {
    STATE.with(|cell| {
        let mut world = new_world();
        let rows = 33;
        let cols = 33;
        let scale = Vec3 {
            x: 1.0,
            y: 1.2,
            z: 1.0,
        };
        let hf = create_wave(rows, cols, scale, 0.08, 0.04, false);
        let hf_origin = Vec3 {
            x: -0.5 * scale.x * (cols - 1) as f32,
            y: 0.0,
            z: -0.5 * scale.z * (rows - 1) as f32,
        };

        let mut ground_def = default_body_def();
        ground_def.type_ = BodyType::Static;
        ground_def.position = pos(hf_origin.x, hf_origin.y, hf_origin.z);
        let ground = create_body(&mut world, &ground_def);
        create_height_field_shape(&mut world, ground, &default_shape_def(), &hf);

        let mut bodies = Vec::new();

        // Obstacle boxes
        for (bx, by, bz, hx, hy, hz) in [
            (4.0f32, 1.0, 4.0, 1.0, 1.0, 1.0),
            (-3.0, 0.8, 2.0, 1.2, 0.8, 0.6),
            (2.0, 0.5, -5.0, 2.0, 0.5, 0.5),
            (-6.0, 1.0, -3.0, 0.8, 1.0, 0.8),
        ] {
            let mut body_def = default_body_def();
            body_def.type_ = BodyType::Static;
            body_def.position = pos(bx, by, bz);
            let body = create_body(&mut world, &body_def);
            let hull = make_box_hull(hx, hy, hz);
            create_hull_shape(&mut world, body, &default_shape_def(), &hull.base);
            bodies.push(VisBody::box_body(body.index1 - 1, hx, hy, hz));
        }

        // Dynamic sphere to shove
        {
            let mut body_def = default_body_def();
            body_def.type_ = BodyType::Dynamic;
            body_def.position = pos(1.0, 3.0, 1.0);
            let body = create_body(&mut world, &body_def);
            let mut shape_def = default_shape_def();
            shape_def.density = 1.0;
            create_sphere_shape(&mut world, body, &shape_def, &sphere(0.4));
            bodies.push(VisBody::sphere_body(body.index1 - 1, 0.4));
        }

        let capsule = Capsule {
            center1: Vec3 {
                x: 0.0,
                y: -0.5,
                z: 0.0,
            },
            center2: Vec3 {
                x: 0.0,
                y: 0.5,
                z: 0.0,
            },
            radius: 0.3,
        };

        let state = CharacterState {
            world,
            bodies,
            hf,
            hf_origin,
            mover_pos: pos(0.0, 4.0, 0.0),
            velocity: VEC3_ZERO,
            capsule,
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
        };
        let n = state.bodies.len() as u32 + 1; // + mover
        *cell.borrow_mut() = Some(state);
        n
    })
}

/// Set WASD throttle, jump edge, sprint, and camera-relative axes (XZ).
#[wasm_bindgen]
pub fn character_set_input(
    throttle_x: f32,
    throttle_y: f32,
    jump: bool,
    sprint: bool,
    fwd_x: f32,
    fwd_z: f32,
    right_x: f32,
    right_z: f32,
) {
    with_state(|state| {
        state.throttle_x = throttle_x;
        state.throttle_y = throttle_y;
        if jump {
            state.jump = true;
        }
        state.want_sprint = sprint;
        let mut fwd = Vec3 {
            x: fwd_x,
            y: 0.0,
            z: fwd_z,
        };
        let mut len = 0.0;
        fwd = get_length_and_normalize(&mut len, fwd);
        if len < 1e-4 {
            fwd = Vec3 {
                x: 0.0,
                y: 0.0,
                z: -1.0,
            };
        }
        let mut right = Vec3 {
            x: right_x,
            y: 0.0,
            z: right_z,
        };
        right = get_length_and_normalize(&mut len, right);
        if len < 1e-4 {
            right = Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            };
        }
        state.forward = fwd;
        state.right = right;
    });
}

#[wasm_bindgen]
pub fn character_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        solve_move(state, dt);
        state.world.step(dt, sub_steps);
        (state.bodies.len() + 1) as u32
    })
}

/// Poses: static boxes + dynamic sphere, then the mover capsule (kind 2).
#[wasm_bindgen]
pub fn character_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.bodies, &mut out);
        // Mover capsule — kinematic visual (not a rigid body)
        out.push(state.mover_pos.x as f32);
        out.push(state.mover_pos.y as f32);
        out.push(state.mover_pos.z as f32);
        out.push(0.0);
        out.push(0.0);
        out.push(0.0);
        out.push(1.0);
        out.push(state.capsule.center1.x);
        out.push(state.capsule.center1.y);
        out.push(state.capsule.center1.z);
        out.push(state.capsule.center2.x);
        out.push(state.capsule.center2.y);
        out.push(state.capsule.center2.z);
        out.push(state.capsule.radius);
        out.push(KIND_CAPSULE as f32);
        debug_assert_eq!(out.len() % POSE_STRIDE, 0);
        out
    })
}

#[wasm_bindgen]
pub fn character_status() -> Vec<f32> {
    with_state(|state| {
        vec![
            state.mover_pos.x as f32,
            state.mover_pos.y as f32,
            state.mover_pos.z as f32,
            state.velocity.x,
            state.velocity.y,
            state.velocity.z,
            if state.on_ground { 1.0 } else { 0.0 },
            if state.sprint { 1.0 } else { 0.0 },
        ]
    })
}

#[wasm_bindgen]
pub fn character_terrain_wireframe() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        let count = get_height_field_triangle_count(&state.hf);
        for i in 0..count {
            let tri = get_height_field_triangle(&state.hf, i);
            let verts = tri.vertices;
            for e in 0..3 {
                let a = verts[e];
                let b = verts[(e + 1) % 3];
                out.push(a.x + state.hf_origin.x);
                out.push(a.y + state.hf_origin.y);
                out.push(a.z + state.hf_origin.z);
                out.push(b.x + state.hf_origin.x);
                out.push(b.y + state.hf_origin.y);
                out.push(b.z + state.hf_origin.z);
            }
        }
        out
    })
}
