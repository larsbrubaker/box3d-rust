//! Dynamic character mover — a 1:1 port of the vendored sample class
//! `box3d-cpp-reference/samples/dynamic_mover.{h,cpp}` (upstream 3362633).
//!
//! The capsule is a rigid body with locked rotation, so it collides and is pushed
//! like everything else in the world. A mover joint drives it to the target velocity
//! and a pogo joint holds it above the ground so it can climb steps without touching
//! the capsule.
//!
//! The dynamic mover should be vendored into your project so you can alter it to meet
//! your game's specific needs.
//!
//! This system is inspired by Tom Waterson's presentation:
//! <https://www.youtube.com/watch?v=_jRLlTDqoGI>
//!
//! Arithmetic and call order follow the C++ line by line (the C++ operator overloads
//! map to the same component-wise `f32` ops), so the mover body matches the C sample
//! bit-for-bit given the same inputs. `Draw` writes into the Character page's debug
//! segment / point buffers instead of the sample renderer.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

mod pogo;
mod types;

use types::cast_closest;
pub(crate) use types::{DynamicMoverCastResult, DynamicMoverDef, DynamicMoverProbe};
#[allow(unused_imports)]
pub(crate) use types::{PROBE_CAPACITY, SKIRT_COUNT};

use box3d_rust::body::{
    body_apply_linear_impulse_to_center, body_get_linear_velocity, body_get_mass,
    body_get_position, body_get_type, body_get_world_point_velocity, body_is_valid,
    body_set_gravity_scale, create_body, destroy_body,
};
use box3d_rust::debug_draw::HexColor;
use box3d_rust::geometry::Capsule;
use box3d_rust::id::{BodyId, JointId, NULL_BODY_ID, NULL_JOINT_ID};
use box3d_rust::joint::{
    create_mover_joint, destroy_joint, joint_get_body_a, joint_is_valid,
    mover_joint_set_linear_velocity, mover_joint_set_max_velocity_force,
};
use box3d_rust::math_functions::{
    abs_float, clamp_float, compute_cos_sin, dot, get_length_and_normalize, is_normalized,
    length_squared, max_float, min_float, normalize, offset_pos, Vec2, Vec3, PI, POS_ZERO,
    VEC3_AXIS_X, VEC3_ZERO,
};
use box3d_rust::shape::create_capsule_shape;
use box3d_rust::types::{
    default_body_def, default_mover_joint_def, default_shape_def, BodyType, Filter, QueryFilter,
};
use box3d_rust::world::World;

/// C `class DynamicMover` (dynamic_mover.h:66). Member names drop the C `m_` prefix.
pub(crate) struct DynamicMover {
    pub jump_speed: f32,
    pub max_speed: f32,
    pub min_speed: f32,
    pub stop_speed: f32,
    pub accelerate: f32,

    /// Fraction of the ground acceleration available in the air.
    pub air_steer: f32,

    /// Ground friction in units of 1/time.
    pub friction: f32,

    /// A character usually wants to fall faster than the rest of the world.
    pub gravity_scale: f32,

    /// Force budget the mover joint has for reaching the target velocity. The air value
    /// is small so the character keeps its momentum while jumping.
    pub max_ground_force: f32,
    pub max_air_force: f32,

    pub pogo_rest_length: f32,
    pub pogo_hertz: f32,
    pub pogo_damping_ratio: f32,

    /// Pogo force limits as a multiple of the mover weight.
    pub pogo_compression_scale: f32,
    pub pogo_tension_scale: f32,

    /// Casts an octagon skirt of rays around the capsule when the central ray misses or
    /// lands on a surface too steep to walk, so the mover can stand on a ledge or a thin
    /// rail, or step off the foot of a steep slope.
    pub enable_skirt_pogo: bool,

    /// Used for the pogo to detect moving platforms.
    pub platform_material_id: u64,

    /// Surfaces steeper than this are not ground.
    pub min_ground_normal_y: f32,

    pub capsule: Capsule,
    pub filter: Filter,

    pub ground_id: BodyId,
    pub mover_id: BodyId,
    pub surface_body_id: BodyId,
    pub mover_joint_id: JointId,
    pub pogo_joint_id: JointId,

    pub cast_result: DynamicMoverCastResult,

    pub velocity: Vec3,
    pub time_sliding: f32,
    pub slide_duration: f32,
    pub on_ground: bool,
    pub walkable: bool,
    pub jumping: bool,
    pub jump_ticks: i32,

    /// Pogo joint state, carried across the joint being recreated each update.
    pub pogo_impulse: f32,
    pub pogo_velocity: f32,
    pub pogo_length: f32,

    /// Pogo ray cast results for drawing.
    pub probes: [DynamicMoverProbe; PROBE_CAPACITY],
    pub probe_count: usize,
    pub selected_probe: usize,
}

impl DynamicMover {
    /// C `DynamicMover::DynamicMover` (dynamic_mover.cpp:46).
    pub(crate) fn new() -> Self {
        let def = DynamicMoverDef::default();
        DynamicMover {
            jump_speed: def.jump_speed,
            max_speed: def.max_speed,
            min_speed: def.min_speed,
            stop_speed: def.stop_speed,
            accelerate: def.accelerate,
            air_steer: def.air_steer,
            friction: def.friction,
            gravity_scale: def.gravity_scale,
            max_ground_force: def.max_ground_force,
            max_air_force: def.max_air_force,
            pogo_rest_length: def.pogo_rest_length,
            pogo_hertz: def.pogo_hertz,
            pogo_damping_ratio: def.pogo_damping_ratio,
            pogo_compression_scale: def.pogo_compression_scale,
            pogo_tension_scale: def.pogo_tension_scale,
            enable_skirt_pogo: def.enable_skirt_pogo,
            platform_material_id: def.platform_material_id,
            min_ground_normal_y: 0.7,

            capsule: def.capsule,
            filter: def.filter,

            ground_id: NULL_BODY_ID,
            mover_id: NULL_BODY_ID,
            surface_body_id: NULL_BODY_ID,
            mover_joint_id: NULL_JOINT_ID,
            pogo_joint_id: NULL_JOINT_ID,

            cast_result: DynamicMoverCastResult::default(),

            velocity: VEC3_ZERO,
            time_sliding: 0.0,
            slide_duration: 0.2,
            on_ground: false,
            walkable: false,
            jumping: false,
            jump_ticks: 0,

            pogo_impulse: 0.0,
            pogo_velocity: 0.0,
            pogo_length: 0.0,

            probes: [DynamicMoverProbe {
                origin: POS_ZERO,
                translation: VEC3_ZERO,
                fraction: 1.0,
                hit: false,
            }; PROBE_CAPACITY],
            probe_count: 0,
            selected_probe: 0,
        }
    }

    /// C `DynamicMover::Create` (dynamic_mover.cpp:96).
    pub(crate) fn create(&mut self, world: &mut World, def: &DynamicMoverDef) {
        self.capsule = def.capsule;
        self.filter = def.filter;
        self.jump_speed = def.jump_speed;
        self.max_speed = def.max_speed;
        self.min_speed = def.min_speed;
        self.stop_speed = def.stop_speed;
        self.accelerate = def.accelerate;
        self.air_steer = def.air_steer;
        self.friction = def.friction;
        self.gravity_scale = def.gravity_scale;
        self.max_ground_force = def.max_ground_force;
        self.max_air_force = def.max_air_force;
        self.pogo_rest_length = def.pogo_rest_length;
        self.pogo_hertz = def.pogo_hertz;
        self.pogo_damping_ratio = def.pogo_damping_ratio;
        self.pogo_compression_scale = def.pogo_compression_scale;
        self.pogo_tension_scale = def.pogo_tension_scale;
        self.enable_skirt_pogo = def.enable_skirt_pogo;
        self.platform_material_id = def.platform_material_id;

        self.velocity = VEC3_ZERO;
        self.on_ground = false;
        self.walkable = false;
        self.jumping = false;
        self.jump_ticks = 0;
        self.pogo_impulse = 0.0;
        self.pogo_velocity = 0.0;
        self.pogo_length = 0.0;
        self.pogo_joint_id = NULL_JOINT_ID;

        let mut body_def = default_body_def();
        self.ground_id = create_body(world, &body_def);

        // Position is the center of the capsule.
        body_def.type_ = BodyType::Dynamic;
        body_def.position = def.position;
        body_def.gravity_scale = self.gravity_scale;
        body_def.motion_locks.angular_x = true;
        body_def.motion_locks.angular_y = true;
        body_def.motion_locks.angular_z = true;
        body_def.enable_sleep = false;
        body_def.name = String::from("mover");

        self.mover_id = create_body(world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density = def.density;

        // The mover joint and the pogo joint handle all the movement, surface friction would
        // fight them.
        shape_def.base_material.friction = 0.0;
        shape_def.base_material.restitution = 0.0;
        shape_def.base_material.custom_color = HexColor::DODGER_BLUE.0;
        shape_def.filter = def.filter;
        shape_def.enable_pre_solve_events = def.enable_pre_solve_events;
        shape_def.enable_sensor_events = def.enable_sensor_events;

        create_capsule_shape(world, self.mover_id, &shape_def, &self.capsule);

        self.create_mover_joint(world, self.ground_id);
    }

    /// C `DynamicMover::CreateMoverJoint` (dynamic_mover.cpp:158).
    fn create_mover_joint(&mut self, world: &mut World, base_id: BodyId) {
        let mut mover_def = default_mover_joint_def();
        mover_def.linear_velocity = self.velocity;
        mover_def.max_force = self.max_ground_force;
        mover_def.base.body_id_a = base_id;
        mover_def.base.body_id_b = self.mover_id;
        mover_def.base.collide_connected = true;

        self.mover_joint_id = create_mover_joint(world, &mover_def);
    }

    /// C `DynamicMover::Destroy` (dynamic_mover.cpp:170).
    #[allow(dead_code)] // C API surface; the sample tears the world down instead.
    pub(crate) fn destroy(&mut self, world: &mut World) {
        if self.mover_id.is_non_null() {
            destroy_body(world, self.mover_id);
        }

        if self.ground_id.is_non_null() {
            destroy_body(world, self.ground_id);
        }

        self.ground_id = NULL_BODY_ID;
        self.mover_id = NULL_BODY_ID;

        // Joints are implicitly destroyed.
        self.mover_joint_id = NULL_JOINT_ID;
        self.pogo_joint_id = NULL_JOINT_ID;
    }

    /// C `DynamicMover::SetGravityScale` — gravity scale is used to size the pogo
    /// forces, so change it here.
    pub(crate) fn set_gravity_scale(&mut self, world: &mut World, gravity_scale: f32) {
        self.gravity_scale = gravity_scale;

        if self.mover_id.is_non_null() {
            body_set_gravity_scale(world, self.mover_id, gravity_scale);
        }
    }

    /// C `DynamicMover::GetBodyId`.
    pub(crate) fn body_id(&self) -> BodyId {
        self.mover_id
    }

    /// C `DynamicMover::Jump` — jump if grounded. Returns false when airborne.
    pub(crate) fn jump(&mut self, world: &mut World) -> bool {
        if !self.on_ground || !self.walkable {
            return false;
        }

        let mut surface_velocity = 0.0f32;
        if body_is_valid(world, self.cast_result.body_id) {
            let v = body_get_world_point_velocity(
                world,
                self.cast_result.body_id,
                self.cast_result.point,
            );
            surface_velocity = v.y;
        }

        let mass = body_get_mass(world, self.mover_id);
        let vy = body_get_linear_velocity(world, self.mover_id).y;

        // Remove the pogo constraint velocity but add the surface velocity.
        let dv = max_float(0.0, self.jump_speed - vy) + surface_velocity;
        body_apply_linear_impulse_to_center(
            world,
            self.mover_id,
            Vec3 {
                x: 0.0,
                y: mass * dv,
                z: 0.0,
            },
            true,
        );

        self.launch();
        true
    }

    /// C `DynamicMover::Launch` — release the ground so an external impulse can carry the
    /// mover into the air. Otherwise the pogo pulls it back down.
    pub(crate) fn launch(&mut self) {
        // This removes the pogo step down on the next update.
        self.on_ground = false;
        self.walkable = false;
        self.jumping = true;
        self.jump_ticks = 0;
    }

    /// C `DynamicMover::Update` (dynamic_mover.cpp:256) — advance the mover. Throttle is
    /// the input along the forward and right directions on [-1, 1].
    ///
    /// Movement follows the Quake ground and air model:
    /// <https://github.com/id-Software/Quake/blob/master/QW/client/pmove.c#L390>
    ///
    /// - onGround - controls friction, mover max force, and pogo step-down extension
    /// - walkable - controls air steer
    /// - jumping - blocks the pogo from pulling down
    pub(crate) fn update(
        &mut self,
        world: &mut World,
        time_step: f32,
        forward: Vec3,
        right: Vec3,
        throttle: Vec2,
        speed_scale: f32,
    ) {
        // Reach further while grounded so the spring can find the ground over a step
        let step_down_length = self.pogo_rest_length;
        let ray_length = if self.on_ground {
            self.pogo_rest_length + step_down_length
        } else {
            self.pogo_rest_length
        };
        let translation = Vec3 {
            x: 0.0,
            y: -ray_length,
            z: 0.0,
        };

        let position = body_get_position(world, self.mover_id);
        let origin = offset_pos(position, self.capsule.center1);

        let query_filter = QueryFilter {
            category_bits: self.filter.category_bits,
            mask_bits: self.filter.mask_bits,
            id: 0,
            name: String::from("dynamic_mover_pogo"),
        };

        self.cast_result = cast_closest(world, self.mover_id, origin, translation, &query_filter);

        self.probes[0] = DynamicMoverProbe {
            origin,
            translation,
            fraction: if self.cast_result.hit {
                self.cast_result.fraction
            } else {
                1.0
            },
            hit: self.cast_result.hit,
        };
        self.probe_count = 1;
        self.selected_probe = 0;

        let mut pogo_offset = VEC3_ZERO;

        let center_walkable =
            self.cast_result.hit && self.cast_result.normal.y >= self.min_ground_normal_y;
        if !center_walkable && self.enable_skirt_pogo {
            // The skirt turns with the direction of travel so ties go to the leading probe
            let mover_velocity = body_get_linear_velocity(world, self.mover_id);
            let mut heading = Vec3 {
                x: mover_velocity.x,
                y: 0.0,
                z: mover_velocity.z,
            };
            if length_squared(heading) < f32::EPSILON {
                heading = VEC3_AXIS_X;
            }

            heading = normalize(heading);
            let side = Vec3 {
                x: -heading.z,
                y: 0.0,
                z: heading.x,
            };

            let mut best_displacement = f32::MAX;
            for i in 0..SKIRT_COUNT {
                // C: 2.0f * B3_PI * i / m_skirtCount, evaluated left to right in float.
                let cs = compute_cos_sin(2.0 * PI * i as f32 / SKIRT_COUNT as f32);
                let offset = self.capsule.radius * (cs.cosine * heading + cs.sine * side);

                let skirt_origin = offset_pos(origin, offset);
                let result = cast_closest(
                    world,
                    self.mover_id,
                    skirt_origin,
                    translation,
                    &query_filter,
                );

                self.probes[self.probe_count] = DynamicMoverProbe {
                    origin: skirt_origin,
                    translation,
                    fraction: if result.hit { result.fraction } else { 1.0 },
                    hit: result.hit,
                };

                // Use the walkable result that causes the least movement. With none the mover
                // keeps the central hit and slides off the steep surface.
                if result.hit && result.normal.y >= self.min_ground_normal_y {
                    let displacement =
                        abs_float(result.fraction * ray_length - self.pogo_rest_length);
                    if displacement < best_displacement {
                        best_displacement = displacement;
                        self.cast_result = result;
                        self.selected_probe = self.probe_count;
                        pogo_offset = offset;
                    }
                }

                self.probe_count += 1;
            }
        }

        // Should jumping end?
        if self.jumping {
            self.jump_ticks += 1;

            // Jump ticks allow time for the jump to leave the ground. Otherwise jumping ends
            // when the character approaches the current ground.
            if self.jump_ticks > 2
                && self.cast_result.hit
                && dot(self.velocity, self.cast_result.normal) <= 0.0
            {
                self.jumping = false;
            }
        }

        if self.cast_result.hit {
            // Is the ground too steep to walk?
            self.walkable = self.cast_result.normal.y >= self.min_ground_normal_y;

            // The cast can still hit while jumping.
            self.on_ground = !self.jumping;

            if self.on_ground {
                if self.walkable {
                    self.time_sliding = max_float(self.time_sliding - time_step, 0.0);
                } else {
                    self.time_sliding =
                        min_float(self.time_sliding + time_step, self.slide_duration);
                }
            }
        } else {
            self.on_ground = false;
            self.walkable = false;
            self.time_sliding = max_float(self.time_sliding - time_step, 0.0);
        }

        // Connect the mover joint to platform. Acts like friction.
        let mut base_id = self.ground_id;
        if self.cast_result.hit
            && self.platform_material_id != 0
            && self.cast_result.user_material_id == self.platform_material_id
            && body_get_type(world, self.cast_result.body_id) == BodyType::Kinematic
        {
            base_id = self.cast_result.body_id;
        }

        // The mover joint can be destroyed if attached to a platform that gets destroyed.
        let mover_joint_valid = joint_is_valid(world, self.mover_joint_id);

        if !mover_joint_valid || joint_get_body_a(world, self.mover_joint_id) != base_id {
            if mover_joint_valid {
                destroy_joint(world, self.mover_joint_id, false);
            }

            self.create_mover_joint(world, base_id);
        }

        self.velocity = body_get_linear_velocity(world, self.mover_id)
            - body_get_linear_velocity(world, base_id);

        // Friction acts in the horizontal plane. The pogo owns the vertical motion.
        if self.on_ground {
            let speed =
                (self.velocity.x * self.velocity.x + self.velocity.z * self.velocity.z).sqrt();
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
        }

        // Mover force
        let max_force = if self.on_ground {
            if self.slide_duration > 0.0 {
                let scale = clamp_float(1.0 - self.time_sliding / self.slide_duration, 0.0, 1.0);
                scale * self.max_ground_force
            } else if self.walkable {
                self.max_ground_force
            } else {
                0.0
            }
        } else {
            self.max_air_force
        };

        mover_joint_set_max_velocity_force(world, self.mover_joint_id, max_force);

        let max_speed = speed_scale * self.max_speed;
        let mut desired_velocity =
            max_speed * throttle.x * forward + max_speed * throttle.y * right;
        desired_velocity.y = 0.0;

        let mut desired_speed = 0.0f32;
        let desired_direction = get_length_and_normalize(&mut desired_speed, desired_velocity);

        if desired_speed > max_speed {
            desired_speed = max_speed;
        }

        // Accelerate
        let current_speed = dot(self.velocity, desired_direction);
        let add_speed = desired_speed - current_speed;
        if add_speed > 0.0 {
            let steer = if self.walkable { 1.0 } else { self.air_steer };
            let mut accel_speed = steer * self.accelerate * max_speed * time_step;
            if accel_speed > add_speed {
                accel_speed = add_speed;
            }

            self.velocity.x += accel_speed * desired_direction.x;
            self.velocity.z += accel_speed * desired_direction.z;
        }

        if self.cast_result.hit && is_normalized(self.cast_result.normal) {
            self.update_pogo(world, ray_length, pogo_offset);
        } else {
            if joint_is_valid(world, self.pogo_joint_id) {
                destroy_joint(world, self.pogo_joint_id, false);
                self.pogo_joint_id = NULL_JOINT_ID;
            }

            self.surface_body_id = NULL_BODY_ID;
            self.pogo_impulse = 0.0;
            self.pogo_velocity = 0.0;
            self.pogo_length = 0.0;
        }

        mover_joint_set_linear_velocity(world, self.mover_joint_id, self.velocity);
    }
}
