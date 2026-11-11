//! Geometric character mover — a 1:1 port of the vendored sample class
//! `box3d-cpp-reference/samples/geometric_mover.{h,cpp}` (upstream 3362633, which
//! replaced the older `mover.{h,cpp}` `CharacterMover`).
//!
//! The capsule is moved by casting and by solving collision planes, so the game keeps
//! full control of the motion. A pogo spring holds the capsule above the ground so it
//! can move up steps.
//!
//! Shared by the Character page's Geometric Mover scene (`character_demo::mover`) and
//! the Compound Village walkthrough (`village`), exactly as both C samples embed one
//! `GeometricMover`. `Draw` writes into debug segment / point buffers.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

use box3d_rust::body::{
    body_apply_linear_impulse, body_get_angular_velocity, body_get_inverse_mass,
    body_get_linear_velocity, body_get_type, body_get_world_center,
    body_get_world_inverse_rotational_inertia,
};
use box3d_rust::geometry::{Capsule, CollisionPlane};
use box3d_rust::id::ShapeId;
use box3d_rust::math_functions::{
    cross, dot, get_length_and_normalize, length, length_squared, max_float, mul_mv, mul_sub,
    mul_sv, neg, offset_pos, spring_damper, sub_pos, Pos, Vec2, Vec3, POS_ZERO, VEC3_AXIS_Y,
    VEC3_ZERO,
};
use box3d_rust::mover::{clip_vector, solve_planes};
use box3d_rust::shape::{shape_get_body, shape_get_user_data};
use box3d_rust::types::{BodyType, QueryFilter};
use box3d_rust::world::{world_cast_mover, world_cast_ray_closest, world_collide_mover, World};

/// C `GeometricMoverDef` (geometric_mover.h:9), with the same member defaults.
#[derive(Clone, Copy)]
pub struct GeometricMoverDef {
    pub position: Pos,
    pub capsule: Capsule,
    pub jump_speed: f32,
    pub max_speed: f32,
    pub min_speed: f32,
    pub stop_speed: f32,
    pub accelerate: f32,
    pub friction: f32,
    pub gravity: f32,
    pub pogo_rest_length: f32,
    pub pogo_hertz: f32,
    pub pogo_damping_ratio: f32,
}

impl Default for GeometricMoverDef {
    fn default() -> Self {
        GeometricMoverDef {
            position: POS_ZERO,
            capsule: Capsule {
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
            },
            jump_speed: 5.0,
            max_speed: 6.0,
            min_speed: 0.01,
            stop_speed: 1.0,
            accelerate: 30.0,
            friction: 4.0,
            gravity: 15.0,
            pogo_rest_length: 0.9,
            pogo_hertz: 4.0,
            pogo_damping_ratio: 0.7,
        }
    }
}

/// C `MoverShapeUserData` (geometric_mover.h:27) — optional shape user data read when
/// the mover gathers collision planes. Shapes without user data are treated as rigid,
/// fully clipping, and pushable.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MoverShapeUserData {
    /// Distance limit on the push this surface may apply to the mover. FLT_MAX is rigid.
    pub max_push: f32,
    /// False keeps the velocity along this surface, which is what soft collision needs.
    pub clip_velocity: bool,
    /// False stops the mover from shoving this body.
    pub can_mover_push: bool,
}

/// Sentinel bit marking a packed [`MoverShapeUserData`] in a shape's `u64` user data
/// (C stores a pointer; the port packs the value: sentinel bit 40, `can_mover_push` bit
/// 41, `max_push` bits in 1..33, `clip_velocity` bit 0).
const USER_DATA_SENTINEL: u64 = 1 << 40;
const USER_DATA_CAN_PUSH: u64 = 1 << 41;

impl MoverShapeUserData {
    /// Pack into a shape `user_data` word.
    pub fn pack(self) -> u64 {
        USER_DATA_SENTINEL
            | if self.can_mover_push {
                USER_DATA_CAN_PUSH
            } else {
                0
            }
            | ((self.max_push.to_bits() as u64) << 1)
            | (self.clip_velocity as u64)
    }

    /// Unpack a shape `user_data` word; `None` when the shape carries no mover data
    /// (C `userData == nullptr`).
    pub fn unpack(user_data: u64) -> Option<Self> {
        if user_data & USER_DATA_SENTINEL == 0 {
            return None;
        }
        Some(MoverShapeUserData {
            max_push: f32::from_bits((user_data >> 1) as u32),
            clip_velocity: user_data & 1 != 0,
            can_mover_push: user_data & USER_DATA_CAN_PUSH != 0,
        })
    }
}

/// C `PlaneExtra` (geometric_mover.h:39) — extra data kept alongside each collision
/// plane so the mover can push rigid bodies.
#[derive(Clone, Copy, Debug)]
pub struct PlaneExtra {
    pub point: Pos,
    pub shape_id: ShapeId,
}

/// C `GeometricMover::m_planeCapacity`.
pub const PLANE_CAPACITY: usize = 8;

/// C `class GeometricMover` (geometric_mover.h:49). Member names drop the C `m_` prefix.
pub struct GeometricMover {
    pub jump_speed: f32,
    pub max_speed: f32,
    pub min_speed: f32,
    pub stop_speed: f32,
    pub accelerate: f32,

    /// Friction in units of 1/time.
    pub friction: f32,

    /// The mover is not simulated so it needs its own gravity.
    pub gravity: f32,

    pub pogo_rest_length: f32,
    pub pogo_hertz: f32,
    pub pogo_damping_ratio: f32,

    /// The clipper avoids picking up velocity from soft collision depenetration. Otherwise
    /// the velocity comes from the position delta, which is more intuitive in some cases.
    pub clip_velocity: bool,

    /// Shapes the mover passes through. Set before calling Update.
    pub ignore_shape_ids: Vec<ShapeId>,

    /// The capsule is relative to the mover position.
    pub capsule: Capsule,

    pub position: Pos,
    pub velocity: Vec3,
    pub pogo_velocity: f32,
    pub on_ground: bool,

    /// Planes from the last solve, kept for debug drawing.
    pub planes: Vec<CollisionPlane>,
    pub plane_extras: Vec<PlaneExtra>,
    pub total_iterations: i32,

    pub pogo_origin: Pos,
    pub pogo_translation: Vec3,
    pub pogo_point: Pos,
    pub pogo_hit: bool,
}

impl Default for GeometricMover {
    fn default() -> Self {
        Self::new()
    }
}

impl GeometricMover {
    /// C `GeometricMover::GeometricMover` (geometric_mover.cpp:69).
    pub fn new() -> Self {
        let def = GeometricMoverDef::default();
        GeometricMover {
            jump_speed: def.jump_speed,
            max_speed: def.max_speed,
            min_speed: def.min_speed,
            stop_speed: def.stop_speed,
            accelerate: def.accelerate,
            friction: def.friction,
            gravity: def.gravity,
            pogo_rest_length: def.pogo_rest_length,
            pogo_hertz: def.pogo_hertz,
            pogo_damping_ratio: def.pogo_damping_ratio,
            clip_velocity: true,
            ignore_shape_ids: Vec::new(),
            capsule: def.capsule,
            position: def.position,
            velocity: VEC3_ZERO,
            pogo_velocity: 0.0,
            on_ground: false,
            planes: Vec::with_capacity(PLANE_CAPACITY),
            plane_extras: Vec::with_capacity(PLANE_CAPACITY),
            total_iterations: 0,
            pogo_origin: POS_ZERO,
            pogo_translation: VEC3_ZERO,
            pogo_point: POS_ZERO,
            pogo_hit: false,
        }
    }

    /// C `GeometricMover::Create` (geometric_mover.cpp:101). The world is passed to each
    /// `update` instead of being stored.
    pub fn create(&mut self, def: &GeometricMoverDef) {
        self.capsule = def.capsule;
        self.jump_speed = def.jump_speed;
        self.max_speed = def.max_speed;
        self.min_speed = def.min_speed;
        self.stop_speed = def.stop_speed;
        self.accelerate = def.accelerate;
        self.friction = def.friction;
        self.gravity = def.gravity;
        self.pogo_rest_length = def.pogo_rest_length;
        self.pogo_hertz = def.pogo_hertz;
        self.pogo_damping_ratio = def.pogo_damping_ratio;

        self.position = def.position;
        self.velocity = VEC3_ZERO;
        self.pogo_velocity = 0.0;
        self.on_ground = false;
        self.planes.clear();
        self.plane_extras.clear();
        self.total_iterations = 0;
        self.pogo_hit = false;
    }

    /// C `GeometricMover::Jump` — jump if grounded. Returns false when airborne.
    pub fn jump(&mut self) -> bool {
        if !self.on_ground {
            return false;
        }

        self.velocity.y = self.jump_speed;
        self.on_ground = false;
        true
    }

    /// C `MoverFilterCallback` (geometric_mover.cpp:14): false for the ignored shapes.
    fn passes_filter(ignore: &[ShapeId], shape_id: ShapeId) -> bool {
        !ignore.contains(&shape_id)
    }

    /// C `GeometricMover::PushBodies` (geometric_mover.cpp:140) — solve a normal
    /// constraint between the mover and each touched rigid body. This is the same
    /// velocity constraint the solver uses for a contact, except the mover is treated as
    /// having infinite mass, so all of the impulse lands on the body.
    fn push_bodies(&mut self, world: &mut World) {
        for i in 0..self.planes.len() {
            let shape_id = self.plane_extras[i].shape_id;
            let body_id = shape_get_body(world, shape_id);
            if body_get_type(world, body_id) != BodyType::Dynamic {
                continue;
            }

            if let Some(user_data) =
                MoverShapeUserData::unpack(shape_get_user_data(world, shape_id))
            {
                if !user_data.can_mover_push {
                    continue;
                }
            }

            // Plane normals point at the mover
            let point = self.plane_extras[i].point;
            let normal = neg(self.planes[i].plane.normal);

            let inv_mass_a = 0.0f32;
            let inv_mass_b = body_get_inverse_mass(world, body_id);
            let inv_ib = body_get_world_inverse_rotational_inertia(world, body_id);

            let center_b = body_get_world_center(world, body_id);
            let r_b = sub_pos(point, center_b);

            let rn_b = cross(r_b, normal);
            let k_normal = inv_mass_a + inv_mass_b + dot(rn_b, mul_mv(inv_ib, rn_b));
            let normal_mass = if k_normal > 0.0 { 1.0 / k_normal } else { 0.0 };

            let v_b = body_get_linear_velocity(world, body_id);
            let omega_b = body_get_angular_velocity(world, body_id);
            let vr_b = v_b + cross(omega_b, r_b);
            let vn = dot(vr_b - self.velocity, normal);

            // Push only, a separating body is left alone
            let impulse = max_float(-normal_mass * vn, 0.0);

            let p = mul_sv(impulse, normal);
            self.velocity = mul_sub(self.velocity, inv_mass_a, p);

            body_apply_linear_impulse(world, body_id, p, point, true);
        }
    }

    /// C `GeometricMover::Update` (geometric_mover.cpp:190) — advance the mover. Throttle
    /// is the input along the forward and right directions on [-1, 1].
    ///
    /// Movement follows the Quake ground and air model:
    /// <https://github.com/id-Software/Quake/blob/master/QW/client/pmove.c#L390>
    pub fn update(
        &mut self,
        world: &mut World,
        time_step: f32,
        forward: Vec3,
        right: Vec3,
        throttle: Vec2,
        speed_scale: f32,
    ) {
        // Friction
        let speed = length(self.velocity);
        if speed < self.min_speed {
            self.velocity.x = 0.0;
            self.velocity.z = 0.0;
        } else {
            // Linear damping above stopSpeed and fixed reduction below stopSpeed
            let control = if speed < self.stop_speed {
                self.stop_speed
            } else {
                speed
            };

            // friction has units of 1/time
            let drop = control * self.friction * time_step;
            let new_speed = max_float(0.0, speed - drop);
            let ratio = new_speed / speed;
            self.velocity.x *= ratio;
            self.velocity.z *= ratio;
        }

        let max_speed = speed_scale * self.max_speed;

        let desired_velocity = max_speed * throttle.x * forward + max_speed * throttle.y * right;
        let mut desired_speed = 0.0f32;
        let desired_direction = get_length_and_normalize(&mut desired_speed, desired_velocity);

        if desired_speed > max_speed {
            desired_speed = max_speed;
        }

        if self.on_ground {
            self.velocity.y = 0.0;
        }

        // Accelerate
        let current_speed = dot(self.velocity, desired_direction);
        let add_speed = desired_speed - current_speed;
        if add_speed > 0.0 {
            let mut accel_speed = self.accelerate * max_speed * time_step;
            if accel_speed > add_speed {
                accel_speed = add_speed;
            }

            self.velocity += accel_speed * desired_direction;
        }

        self.velocity.y -= self.gravity * time_step;

        // The probe reaches past the rest length so the spring can be stretched
        let ray_length = self.pogo_rest_length + self.capsule.radius;
        let ray_origin = offset_pos(self.position, self.capsule.center1);
        let ray_translation = -ray_length * VEC3_AXIS_Y;
        let pogo_filter = QueryFilter {
            category_bits: 1,
            mask_bits: !2u64,
            id: 0,
            name: String::from("pogo"),
        };
        let ray_result = world_cast_ray_closest(world, ray_origin, ray_translation, &pogo_filter);

        self.pogo_origin = ray_origin;
        self.pogo_translation = ray_translation;
        self.pogo_point = ray_result.point;

        // After gravity was applied, disable pogo when still moving up.
        // Avoids getting pulled back to the ground when jumping.
        let suppress_pogo = self.velocity.y > 0.0;

        if !ray_result.hit || suppress_pogo {
            self.on_ground = false;
            self.pogo_velocity = 0.0;
        } else {
            self.on_ground = true;
            let pogo_current_length = ray_result.fraction * ray_length;
            let offset = pogo_current_length - self.pogo_rest_length;
            self.pogo_velocity = spring_damper(
                self.pogo_hertz,
                self.pogo_damping_ratio,
                offset,
                self.pogo_velocity,
                time_step,
            );
        }

        self.pogo_hit = self.on_ground;

        let start_position = self.position;
        let target = offset_pos(
            offset_pos(self.position, time_step * self.velocity),
            time_step * self.pogo_velocity * VEC3_AXIS_Y,
        );

        // Want the mover to collide with allies
        let mover_filter = QueryFilter {
            category_bits: 1,
            mask_bits: !0u64,
            id: 1,
            name: String::from("mover_collide"),
        };

        // The cast should ignore allies
        let cast_filter = QueryFilter {
            category_bits: 1,
            mask_bits: !2u64,
            id: 1,
            name: String::from("mover_cast"),
        };

        self.total_iterations = 0;
        let tolerance = 0.01f32;

        for _iteration in 0..5 {
            self.planes.clear();
            self.plane_extras.clear();

            self.collect_planes(world, &mover_filter);

            let target_delta = sub_pos(target, self.position);
            let result = solve_planes(target_delta, &mut self.planes);

            self.total_iterations += result.iteration_count;

            let ignore = &self.ignore_shape_ids;
            let mut filter_fcn = |shape_id: ShapeId| Self::passes_filter(ignore, shape_id);
            let fraction = world_cast_mover(
                world,
                self.position,
                &self.capsule,
                result.delta,
                &cast_filter,
                Some(&mut filter_fcn),
            );

            let delta = fraction * result.delta;
            self.position = offset_pos(self.position, delta);

            if length_squared(delta) < tolerance * tolerance {
                break;
            }
        }

        self.push_bodies(world);

        if self.clip_velocity {
            // Stop pushing into surfaces so speed can't accumulate against a wall
            self.velocity = clip_vector(self.velocity, &self.planes);
        } else if time_step > 0.0 {
            self.velocity = (1.0 / time_step) * sub_pos(self.position, start_position);
        }
    }

    /// `b3World_CollideMover( ..., PlaneResultFcn, this )` (geometric_mover.cpp:36):
    /// gather up to [`PLANE_CAPACITY`] planes, skipping the ignored shapes' planes but
    /// continuing to look for more.
    fn collect_planes(&mut self, world: &World, filter: &QueryFilter) {
        let origin = self.position;
        let ignore = &self.ignore_shape_ids;
        let planes = &mut self.planes;
        let extras = &mut self.plane_extras;
        world_collide_mover(world, origin, &self.capsule, filter, |shape_id, results| {
            if !Self::passes_filter(ignore, shape_id) {
                // ignore these planes but continue looking for more
                return true;
            }

            let mut max_push = f32::MAX;
            let mut clip_velocity = true;
            if let Some(user_data) =
                MoverShapeUserData::unpack(shape_get_user_data(world, shape_id))
            {
                max_push = user_data.max_push;
                clip_velocity = user_data.clip_velocity;
            }

            for r in results {
                if planes.len() >= PLANE_CAPACITY {
                    break;
                }
                debug_assert!(box3d_rust::math_functions::is_valid_plane(r.plane));
                planes.push(CollisionPlane {
                    plane: r.plane,
                    push_limit: max_push,
                    push: 0.0,
                    clip_velocity,
                });

                // The plane point is relative to the origin passed to the collide call
                extras.push(PlaneExtra {
                    point: offset_pos(origin, r.point),
                    shape_id,
                });
            }

            true
        });
    }

    /// C `GeometricMover::Draw` (geometric_mover.cpp:325) minus the solid capsule, which
    /// the caller renders: the pogo ray (green on ground, gray in air), the yellow plane
    /// points + normals, and the purple velocity line. Appends `[x0,y0,z0,x1,y1,z1,color]`
    /// segments and `[x,y,z,color,size]` points.
    pub fn draw(&self, segs: &mut Vec<f32>, pts: &mut Vec<f32>) {
        const GREEN: u32 = 0x008000;
        const GRAY: u32 = 0x808080;
        const YELLOW: u32 = 0xFFFF00;
        const PURPLE: u32 = 0x800080;

        if self.pogo_hit {
            push_line(segs, self.pogo_origin, self.pogo_point, GREEN);
        } else {
            push_line(
                segs,
                self.pogo_origin,
                offset_pos(self.pogo_origin, self.pogo_translation),
                GRAY,
            );
        }

        for cp in &self.planes {
            let plane = cp.plane;
            let p1 = offset_pos(
                self.position,
                (plane.offset - self.capsule.radius) * plane.normal,
            );
            let p2 = offset_pos(p1, 0.1 * plane.normal);
            pts.extend_from_slice(&[p1.x as f32, p1.y as f32, p1.z as f32, YELLOW as f32, 5.0]);
            push_line(segs, p1, p2, YELLOW);
        }

        push_line(
            segs,
            self.position,
            offset_pos(self.position, self.velocity),
            PURPLE,
        );
    }
}

fn push_line(segs: &mut Vec<f32>, a: Pos, b: Pos, color: u32) {
    segs.extend_from_slice(&[
        a.x as f32,
        a.y as f32,
        a.z as f32,
        b.x as f32,
        b.y as f32,
        b.z as f32,
        color as f32,
    ]);
}
