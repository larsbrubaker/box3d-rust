//! Shared demo-side kinematic mover integration — the one true port of C
//! `CharacterMover::SolveMove` (`sample.cpp:2154`).
//!
//! Both the Character page's `MoverController` (`character_demo::mover`) and the
//! Compound Village's `VillageMover` (`village`) are thin wrappers around this
//! single integration: friction, accelerate, the pogo ground probe/spring, the
//! 5-iteration slide over `world_collide_mover` + `b3SolvePlanes` +
//! `world_cast_mover`, the dynamic-body push loop, and the final clip / position-
//! delta velocity recovery. The scene-specific differences (team-based query
//! filters, ignored shapes, and the Clip Velocity toggle) are passed in via
//! [`MoverParams`]; the numeric behavior is identical for every caller.

use box3d_rust::body::{
    body_apply_linear_impulse, body_get_angular_velocity, body_get_inverse_mass,
    body_get_linear_velocity, body_get_type, body_get_world_center,
    body_get_world_inverse_rotational_inertia,
};
use box3d_rust::geometry::{Capsule, CollisionPlane};
use box3d_rust::id::ShapeId;
use box3d_rust::math_functions::{
    cross, dot, get_length_and_normalize, length, length_squared, max_float, mul_mv, mul_sub,
    mul_sv, neg, offset_pos, sub_pos, Pos, Vec3, VEC3_AXIS_Y,
};
use box3d_rust::mover::{clip_vector, solve_planes};
use box3d_rust::shape::{shape_get_body, shape_get_user_data};
use box3d_rust::types::{BodyType, QueryFilter};
use box3d_rust::world::{world_cast_mover, world_cast_ray_closest, world_collide_mover, World};
use std::f32::consts::PI;

// CharacterMover tuning constants (C `struct CharacterMover`, sample.h:296-303).
pub const PLANE_CAPACITY: usize = 8;
pub const JUMP_SPEED: f32 = 5.0;
pub const MAX_SPEED: f32 = 6.0;
pub const MIN_SPEED: f32 = 0.01;
pub const STOP_SPEED: f32 = 1.0;
pub const ACCELERATE: f32 = 30.0;
pub const FRICTION: f32 = 4.0;
pub const GRAVITY: f32 = 15.0;

/// The shared `CharacterMover` capsule (C `{ {0,-0.5,0}, {0,0.5,0}, 0.3 }`).
pub fn mover_capsule() -> Capsule {
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

/// s&box-style `MoverShapeUserData` (C sample.h:282) packed into the shape's u64
/// user data: sentinel bit 40, `max_push` in bits 1..33, `clip_velocity` in bit 0.
pub fn pack_mover_user_data(max_push: f32, clip_velocity: bool) -> u64 {
    (1u64 << 40) | ((max_push.to_bits() as u64) << 1) | (clip_velocity as u64)
}

fn unpack_mover_user_data(ud: u64) -> Option<(f32, bool)> {
    if ud & (1u64 << 40) == 0 {
        return None;
    }
    Some((f32::from_bits((ud >> 1) as u32), ud & 1 != 0))
}

/// The kinematic mover's physical state, read and mutated by [`solve_move`]. The
/// capsule never rotates (C keeps `m_transform.q = identity`), so only its world
/// position is tracked.
pub struct MoverBody {
    pub position: Pos,
    pub velocity: Vec3,
    pub capsule: Capsule,
    pub pogo_velocity: f32,
    pub on_ground: bool,
    pub sprint: bool,
}

/// Per-step inputs plus the scene-specific query behavior (the only things that
/// differ between the Character mover and the Village mover).
pub struct MoverParams<'a> {
    pub forward: Vec3,
    pub right: Vec3,
    pub throttle_x: f32,
    pub throttle_y: f32,
    /// C `clipVelocity` (`DrawControls` "Clip Velocity"): clip against the contact
    /// planes when true, else recover velocity from the realized position delta.
    pub clip_velocity: bool,
    /// Shapes the collide/cast queries skip (e.g. the Character mover's `{7,2,-3}`
    /// ignore box). Empty for scenes with nothing to ignore.
    pub ignore_shapes: &'a [ShapeId],
    /// Pogo ground-probe ray filter (C `skipTeamFilter`).
    pub pogo_filter: QueryFilter,
    /// Plane-gathering filter (C `moverFilter`, collides with allies).
    pub mover_filter: QueryFilter,
    /// Shape-cast filter (C `castFilter`, ignores allies).
    pub cast_filter: QueryFilter,
}

/// Debug/plane output filled by [`solve_move`]. The planes drive the push loop and
/// the clip step; callers that draw overlays (the Character mover) read the planes,
/// contact points, and pogo segment back out. The Village mover ignores them.
#[derive(Default)]
pub struct MoverDraw {
    pub planes: Vec<CollisionPlane>,
    pub plane_points: Vec<Pos>,
    pub plane_shapes: Vec<ShapeId>,
    pub pogo_origin: Pos,
    pub pogo_end: Pos,
    pub pogo_hit: bool,
}

fn shape_is_ignored(ignore: &[ShapeId], shape_id: ShapeId) -> bool {
    ignore
        .iter()
        .any(|s| s.index1 == shape_id.index1 && s.generation == shape_id.generation)
}

/// C `CharacterMover::SolveMove` (`sample.cpp:2154`). Mutates the world (dynamic
/// push impulses), `body`, and the plane/pogo `draw` output. `jump`/`sprint` are
/// applied by the caller before this runs, mirroring C's `Step` order.
pub fn solve_move(
    world: &mut World,
    body: &mut MoverBody,
    params: &MoverParams,
    draw: &mut MoverDraw,
    time_step: f32,
) {
    // --- Friction ---
    let speed = length(body.velocity);
    if speed < MIN_SPEED {
        // C zeroes x and y here (sample.cpp:2160-2161); replicated verbatim.
        body.velocity.x = 0.0;
        body.velocity.y = 0.0;
    } else {
        let control = if speed < STOP_SPEED {
            STOP_SPEED
        } else {
            speed
        };
        let drop = control * FRICTION * time_step;
        let new_speed = max_float(0.0, speed - drop);
        body.velocity *= new_speed / speed;
    }

    let max_speed = if body.sprint {
        1.5 * MAX_SPEED
    } else {
        MAX_SPEED
    };

    let mut desired_velocity = mul_sv(max_speed * params.throttle_x, params.forward)
        + mul_sv(max_speed * params.throttle_y, params.right);
    let mut desired_speed = 0.0;
    let desired_direction = get_length_and_normalize(&mut desired_speed, desired_velocity);
    if desired_speed > max_speed {
        desired_velocity *= max_speed / desired_speed;
        desired_speed = max_speed;
    }

    if body.on_ground {
        body.velocity.y = 0.0;
    }

    // --- Accelerate ---
    let current_speed = dot(body.velocity, desired_direction);
    let add_speed = desired_speed - current_speed;
    if add_speed > 0.0 {
        let mut accel_speed = ACCELERATE * max_speed * time_step;
        if accel_speed > add_speed {
            accel_speed = add_speed;
        }
        body.velocity += mul_sv(accel_speed, desired_direction);
    }

    body.velocity.y -= GRAVITY * time_step;

    // --- Pogo ground probe + spring ---
    let pogo_rest_length = 3.0 * body.capsule.radius;
    let ray_length = pogo_rest_length + body.capsule.radius;
    // The capsule never rotates, so the world ray origin is a plain offset (C uses
    // b3TransformWorldPoint with the identity rotation, which is bit-identical).
    let ray_origin = offset_pos(body.position, body.capsule.center1);
    let ray_translation = mul_sv(-ray_length, VEC3_AXIS_Y);
    let ray_result =
        world_cast_ray_closest(world, ray_origin, ray_translation, &params.pogo_filter);

    draw.pogo_origin = ray_origin;
    if !ray_result.hit {
        body.on_ground = false;
        body.pogo_velocity = 0.0;
        draw.pogo_hit = false;
        draw.pogo_end = offset_pos(ray_origin, ray_translation);
    } else {
        body.on_ground = true;
        draw.pogo_hit = true;
        draw.pogo_end = ray_result.point;
        let pogo_current_length = ray_result.fraction * ray_length;
        let zeta = 0.7f32;
        let hertz = 4.0f32;
        let omega = 2.0 * PI * hertz;
        let omega_h = omega * time_step;
        body.pogo_velocity = (body.pogo_velocity
            - omega * omega_h * (pogo_current_length - pogo_rest_length))
            / (1.0 + 2.0 * zeta * omega_h + omega_h * omega_h);
    }

    let start_position = body.position;
    let target = offset_pos(
        body.position,
        mul_sv(time_step, body.velocity) + mul_sv(time_step * body.pogo_velocity, VEC3_AXIS_Y),
    );

    let tolerance = 0.01f32;
    for _iteration in 0..5 {
        draw.planes.clear();
        draw.plane_points.clear();
        draw.plane_shapes.clear();

        let mover = body.capsule;
        {
            // Immutable world borrow shared by the query and the userData reads.
            let w: &World = world;
            let origin = body.position;
            let ignore = params.ignore_shapes;
            let planes = &mut draw.planes;
            let points = &mut draw.plane_points;
            let shapes = &mut draw.plane_shapes;
            world_collide_mover(
                w,
                origin,
                &mover,
                &params.mover_filter,
                |shape_id, results| {
                    // C PlaneResultFcn: skip ignored shapes' planes but keep looking.
                    if shape_is_ignored(ignore, shape_id) {
                        return true;
                    }
                    let (max_push, clip) = unpack_mover_user_data(shape_get_user_data(w, shape_id))
                        .unwrap_or((f32::MAX, true));
                    for r in results {
                        if planes.len() >= PLANE_CAPACITY {
                            break;
                        }
                        planes.push(CollisionPlane {
                            plane: r.plane,
                            push_limit: max_push,
                            push: 0.0,
                            clip_velocity: clip,
                        });
                        points.push(offset_pos(origin, r.point));
                        shapes.push(shape_id);
                    }
                    true
                },
            );
        }

        let target_delta = sub_pos(target, body.position);
        let result = solve_planes(target_delta, &mut draw.planes);
        let mut delta = result.delta;

        let ignore = params.ignore_shapes.to_vec();
        let mut filter_fcn = move |sid: ShapeId| !shape_is_ignored(&ignore, sid);
        let fraction = world_cast_mover(
            world,
            body.position,
            &mover,
            delta,
            &params.cast_filter,
            Some(&mut filter_fcn),
        );
        delta *= fraction;
        body.position = offset_pos(body.position, delta);

        if length_squared(delta) < tolerance * tolerance {
            break;
        }
    }

    // --- Push dynamic bodies the mover leans on (sample.cpp:2280-2313) ---
    for i in 0..draw.planes.len() {
        let shape_id = draw.plane_shapes[i];
        let body_id = shape_get_body(world, shape_id);
        if body_get_type(world, body_id) != BodyType::Dynamic {
            continue;
        }

        let point = draw.plane_points[i];
        let normal = neg(draw.planes[i].plane.normal);

        let inv_mass_a = 0.0f32;
        let inv_mass_b = body_get_inverse_mass(world, body_id);
        let inv_ib = body_get_world_inverse_rotational_inertia(world, body_id);

        let p_b = body_get_world_center(world, body_id);
        let r_b = sub_pos(point, p_b);

        let rn_b = cross(r_b, normal);
        let k_normal = inv_mass_a + inv_mass_b + dot(rn_b, mul_mv(inv_ib, rn_b));
        let normal_mass = if k_normal > 0.0 { 1.0 / k_normal } else { 0.0 };

        let v_b = body_get_linear_velocity(world, body_id);
        let omega_b = body_get_angular_velocity(world, body_id);
        let vr_b = v_b + cross(omega_b, r_b);
        let vn = dot(vr_b - body.velocity, normal);
        let impulse = max_float(-normal_mass * vn, 0.0);

        let p_impulse = mul_sv(impulse, normal);
        body.velocity = mul_sub(body.velocity, inv_mass_a, p_impulse);
        body_apply_linear_impulse(world, body_id, p_impulse, point, true);
    }

    if params.clip_velocity {
        body.velocity = clip_vector(body.velocity, &draw.planes);
    } else if time_step > 0.0 {
        body.velocity = mul_sv(1.0 / time_step, sub_pos(body.position, start_position));
    }
}
