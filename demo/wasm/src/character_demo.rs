//! Character / Mover — capsule mover toward BasicMover fidelity (`sample_character.cpp`).
//!
//! Scene modes:
//! - `0` BasicMover-style: wave height field, static capsules, boxes, dynamic sphere
//! - `1` Village walk: real compound Village (building.obj meshes) + mover (C embeds a mover)

use crate::village::{self, BuildingInstance};
use crate::vis::{pos, push_poses, sphere, VisBody, KIND_CAPSULE, POSE_STRIDE};
use box3d_rust::body::{create_body, get_body_transform};
use box3d_rust::geometry::{Capsule, CollisionPlane};
use box3d_rust::height_field::{
    create_wave, get_height_field_triangle, get_height_field_triangle_count, HeightFieldData,
};
use box3d_rust::hull::make_box_hull;
use box3d_rust::math_functions::{
    dot, get_length_and_normalize, length, length_squared, max_float, mul_sv, mul_transforms,
    offset_pos, sub_pos, Pos, Transform, Vec3, QUAT_IDENTITY, VEC3_AXIS_Y, VEC3_ZERO,
};
use box3d_rust::mover::{clip_vector, solve_planes};
use box3d_rust::shape::{
    create_capsule_shape, create_height_field_shape, create_hull_shape, create_sphere_shape,
};
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
    /// Static scenery + dynamic props for poses.
    bodies: Vec<VisBody>,
    hf: Option<HeightFieldData>,
    hf_origin: Vec3,
    mover_pos: Pos,
    velocity: Vec3,
    capsule: Capsule,
    pogo_velocity: f32,
    on_ground: bool,
    sprint: bool,
    throttle_x: f32,
    throttle_y: f32,
    jump: bool,
    want_sprint: bool,
    forward: Vec3,
    right: Vec3,
    /// Last pogo ray segment for debug draw: origin → end (or hit).
    pogo_origin: Vec3,
    pogo_end: Vec3,
    pogo_hit: bool,
    village_buildings: Vec<BuildingInstance>,
    village_stats: [f32; 7],
    village_ground_index: i32,
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

fn mover_capsule() -> Capsule {
    Capsule {
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
    }
}

fn solve_move(state: &mut CharacterState, time_step: f32) {
    if time_step <= 0.0 {
        return;
    }

    // Friction (XZ plane)
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

    // Pogo ground probe (visualized as the ground ray)
    let pogo_rest_length = 3.0 * state.capsule.radius;
    let ray_length = pogo_rest_length + state.capsule.radius;
    let ray_origin = offset_pos(state.mover_pos, state.capsule.center1);
    let ray_translation = mul_sv(-ray_length, VEC3_AXIS_Y);
    let filter = default_query_filter();
    let ray_result = world_cast_ray_closest(&state.world, ray_origin, ray_translation, &filter);

    state.pogo_origin = Vec3 {
        x: ray_origin.x as f32,
        y: ray_origin.y as f32,
        z: ray_origin.z as f32,
    };
    if !ray_result.hit {
        state.on_ground = false;
        state.pogo_velocity = 0.0;
        state.pogo_hit = false;
        state.pogo_end = state.pogo_origin + ray_translation;
    } else {
        state.on_ground = true;
        state.pogo_hit = true;
        let pogo_current_length = ray_result.fraction * ray_length;
        state.pogo_end = state.pogo_origin + mul_sv(ray_result.fraction, ray_translation);
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
    let _ = solve_planes(VEC3_ZERO, &mut planes[..plane_count]);
    state.velocity = clip_vector(state.velocity, &planes[..plane_count]);
}

fn build_basic_mover(world: &mut World, bodies: &mut Vec<VisBody>) -> (HeightFieldData, Vec3, Pos) {
    // Height field offset like BasicMover (`position = {20,0,0}`), smaller for the browser.
    let rows = 41;
    let cols = 41;
    let scale = Vec3 {
        x: 1.0,
        y: 1.0,
        z: 1.0,
    };
    let hf = create_wave(rows, cols, scale, 0.02, 0.04, true);
    let hf_origin = Vec3 {
        x: 20.0 - 0.5 * scale.x * (cols - 1) as f32,
        y: 0.0,
        z: -0.5 * scale.z * (rows - 1) as f32,
    };

    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    ground_def.position = pos(hf_origin.x, hf_origin.y, hf_origin.z);
    let ground = create_body(world, &ground_def);
    create_height_field_shape(world, ground, &default_shape_def(), &hf);

    // Obstacle boxes (stand-in for the C test_map01 mesh corners)
    for (bx, by, bz, hx, hy, hz) in [
        (4.0f32, 1.0, 14.0, 1.0, 1.0, 1.0),
        (4.0, 1.0, 13.95, 1.0, 1.0, 1.0),
        (5.8, 1.0, 13.7, 1.0, 1.0, 1.0),
        (-3.0, 0.8, 2.0, 1.2, 0.8, 0.6),
        (2.0, 0.5, -5.0, 2.0, 0.5, 0.5),
        (7.0, 0.25, -3.0, 0.5, 0.25, 0.5),
    ] {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Static;
        body_def.position = pos(bx, by, bz);
        let body = create_body(world, &body_def);
        let hull = make_box_hull(hx, hy, hz);
        create_hull_shape(world, body, &default_shape_def(), &hull.base);
        bodies.push(VisBody::box_body(body.index1 - 1, hx, hy, hz));
    }

    // Enemy capsule (pushable wall feel) — BasicMover violet-red
    {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Static;
        body_def.position = pos(0.0, 1.4, 6.0);
        let body = create_body(world, &body_def);
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
        create_capsule_shape(world, body, &default_shape_def(), &capsule);
        bodies.push(VisBody::capsule_body(body.index1 - 1, &capsule));
    }

    // Friendly capsule (filter group in C; here still collides, green visual)
    {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Static;
        body_def.position = pos(0.0, 1.4, 5.0);
        let body = create_body(world, &body_def);
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
        create_capsule_shape(world, body, &default_shape_def(), &capsule);
        bodies.push(VisBody::capsule_body(body.index1 - 1, &capsule));
    }

    // Dynamic sphere to shove
    {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = pos(7.0, 5.0, 0.0);
        let body = create_body(world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.density = 1.0;
        create_sphere_shape(world, body, &shape_def, &sphere(0.5));
        bodies.push(VisBody::sphere_body(body.index1 - 1, 0.5));
    }

    let start = pos(7.5, 0.75, 9.0);
    (hf, hf_origin, start)
}

fn build_village_ground(world: &mut World, bodies: &mut Vec<VisBody>, grid: i32) -> VillageWalk {
    let village = village::build_village(world, grid);
    let a = village.tile_half;
    let parent_index = village.ground_body_index;

    for xf in &village.hull_transforms {
        bodies.push(VisBody::box_local(parent_index, a, 0.5 * a, a, *xf));
    }
    for s in &village.spheres {
        bodies.push(VisBody::sphere_local(
            parent_index,
            s.sphere.radius,
            Transform {
                p: s.sphere.center,
                q: QUAT_IDENTITY,
            },
        ));
    }
    for c in &village.capsules {
        bodies.push(VisBody {
            body_index: parent_index,
            kind: KIND_CAPSULE,
            params: [
                c.capsule.center1.x,
                c.capsule.center1.y,
                c.capsule.center1.z,
                c.capsule.center2.x,
                c.capsule.center2.y,
                c.capsule.center2.z,
                c.capsule.radius,
            ],
            local: None,
            color: 0,
        });
    }

    VillageWalk {
        start: pos(0.0, 10.0, 0.0),
        buildings: village.buildings,
        stats: village.stats,
        ground_index: parent_index,
    }
}

struct VillageWalk {
    start: Pos,
    buildings: Vec<BuildingInstance>,
    stats: [f32; 7],
    ground_index: i32,
}

/// Reset character scene.
/// `mode`: 0 = BasicMover-style, 1 = Village walk (`grid_count` used only for village).
#[wasm_bindgen]
pub fn character_reset_ex(mode: u32, grid_count: u32) -> u32 {
    STATE.with(|cell| {
        let mut world = new_world();
        let mut bodies = Vec::new();
        let capsule = mover_capsule();

        let mut village_buildings = Vec::new();
        let mut village_stats = [0.0f32; 7];
        let mut village_ground_index = -1;

        let (hf, hf_origin, start) = if mode == 1 {
            let grid = grid_count.clamp(8, 40) as i32;
            let walk = build_village_ground(&mut world, &mut bodies, grid);
            village_buildings = walk.buildings;
            village_stats = walk.stats;
            village_ground_index = walk.ground_index;
            (None, VEC3_ZERO, walk.start)
        } else {
            let (hf, origin, start) = build_basic_mover(&mut world, &mut bodies);
            (Some(hf), origin, start)
        };

        let state = CharacterState {
            world,
            bodies,
            hf,
            hf_origin,
            mover_pos: start,
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
            pogo_origin: VEC3_ZERO,
            pogo_end: VEC3_ZERO,
            pogo_hit: false,
            village_buildings,
            village_stats,
            village_ground_index,
        };
        let n = state.bodies.len() as u32 + 1;
        *cell.borrow_mut() = Some(state);
        n
    })
}

/// Reset BasicMover-style scene (default).
#[wasm_bindgen]
pub fn character_reset() -> u32 {
    character_reset_ex(0, 10)
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

/// Poses: scenery bodies, then the mover capsule (kind 2).
#[wasm_bindgen]
pub fn character_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.bodies, &mut out);
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
        out.push(0.0); // color
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

/// Debug overlay segments: pogo ray (6 floats) + velocity (6 floats) + hit flag.
/// Layout: `[ox,oy,oz, ex,ey,ez,  vx0,vy0,vz0, vx1,vy1,vz1, hit]`
#[wasm_bindgen]
pub fn character_debug_lines() -> Vec<f32> {
    with_state(|state| {
        let p = Vec3 {
            x: state.mover_pos.x as f32,
            y: state.mover_pos.y as f32,
            z: state.mover_pos.z as f32,
        };
        vec![
            state.pogo_origin.x,
            state.pogo_origin.y,
            state.pogo_origin.z,
            state.pogo_end.x,
            state.pogo_end.y,
            state.pogo_end.z,
            p.x,
            p.y,
            p.z,
            p.x + state.velocity.x,
            p.y + state.velocity.y,
            p.z + state.velocity.z,
            if state.pogo_hit { 1.0 } else { 0.0 },
        ]
    })
}

#[wasm_bindgen]
pub fn character_terrain_wireframe() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        let Some(hf) = state.hf.as_ref() else {
            return out;
        };
        let count = get_height_field_triangle_count(hf);
        for i in 0..count {
            let tri = get_height_field_triangle(hf, i);
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

/// Village building instances in world space (same layout as `sim_village_buildings`).
#[wasm_bindgen]
pub fn character_village_buildings() -> Vec<f32> {
    with_state(|state| {
        if state.village_ground_index < 0 || state.village_buildings.is_empty() {
            return Vec::new();
        }
        let parent = get_body_transform(&state.world, state.village_ground_index);
        let parent_xf = Transform {
            p: Vec3 {
                x: parent.p.x as f32,
                y: parent.p.y as f32,
                z: parent.p.z as f32,
            },
            q: parent.q,
        };
        let mut out = Vec::with_capacity(state.village_buildings.len() * 10);
        for b in &state.village_buildings {
            let world_xf = mul_transforms(parent_xf, b.transform);
            out.push(world_xf.p.x);
            out.push(world_xf.p.y);
            out.push(world_xf.p.z);
            out.push(world_xf.q.v.x);
            out.push(world_xf.q.v.y);
            out.push(world_xf.q.v.z);
            out.push(world_xf.q.s);
            out.push(b.scale.x);
            out.push(b.scale.y);
            out.push(b.scale.z);
        }
        out
    })
}

/// Village compound stats (same layout as `sim_village_stats`).
#[wasm_bindgen]
pub fn character_village_stats() -> Vec<f32> {
    with_state(|state| state.village_stats.to_vec())
}
