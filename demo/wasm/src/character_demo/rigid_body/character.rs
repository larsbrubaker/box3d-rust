//! The `RigidbodyCharacter` velocity model + trace-based step-up
//! (`sample_character.cpp` `RigidbodyCharacter`, `:649`). A dynamic body with a
//! lower feet box and an upper capsule, all rotation locked. Movement is a
//! velocity model driven by `AddVelocity` + `UpdateBody`, ground is categorized
//! with a box shape cast (`b3World_CastShape`), and obstacles are surmounted with
//! a 4-phase trace-based step-up (`TryStep`) plus a `Reground` snap. Ported
//! faithfully; the s&box unit conversion (1 unit = 1 inch = 0.0254 m) is preserved.

#![allow(clippy::unnecessary_cast)]

use super::super::colors;
use box3d_rust::body::{
    body_get_linear_velocity, body_get_mass_data, body_get_position, body_get_transform,
    body_get_world_center, body_set_gravity_scale, body_set_linear_damping,
    body_set_linear_velocity, body_set_mass_data, body_set_transform,
};
use box3d_rust::distance::ShapeProxy;
use box3d_rust::id::{BodyId, ShapeId};
use box3d_rust::math_functions::{
    clamp_float, dot, length, mul_sv, offset_pos, sub_pos, Pos, Vec3, VEC3_AXIS_Y, VEC3_ZERO,
};
use box3d_rust::shape::shape_set_friction;
use box3d_rust::types::default_query_filter;
use box3d_rust::world::{world_cast_shape, World};
use std::f32::consts::PI;

// s&box unit conversion: 1 unit = 1 inch = 0.0254 m (40 units/m).
pub(super) const SRC: f32 = 0.0254;

const WALK_SPEED: f32 = 230.0 * SRC;
const RUN_SPEED: f32 = 350.0 * SRC;
const JUMP_SPEED: f32 = 300.0 * SRC;
const MAX_SLOPE_ANGLE: f32 = 45.0;
pub(super) const CHARACTER_GRAVITY: f32 = 15.0;
pub(super) const CHARACTER_MASS: f32 = 500.0;
const JUMP_COOLDOWN_TIME: f32 = 0.2;

const STEP_UP_HEIGHT: f32 = 18.0 * SRC;
const STEP_DOWN_HEIGHT: f32 = 18.0 * SRC;
const SKIN: f32 = 0.095 * SRC;
const BRAKE_POWER: f32 = 0.2;
const SURFACE_FRICTION: f32 = 0.6;
const AIR_FRICTION: f32 = 0.1;

pub(super) const BODY_RADIUS: f32 = 16.0 * SRC;
pub(super) const TOTAL_HEIGHT: f32 = 72.0 * SRC;
pub(super) const FEET_HEIGHT: f32 = TOTAL_HEIGHT * 0.5;

/// Offset a `Pos` along Y by `dy` metres (feature-agnostic Pos scalar handling).
fn y_off(p: Pos, dy: f32) -> Pos {
    offset_pos(
        p,
        Vec3 {
            x: 0.0,
            y: dy,
            z: 0.0,
        },
    )
}

struct TraceResult {
    end_position: Pos,
    normal: Vec3,
    hit_point: Pos,
    fraction: f32,
    hit: bool,
    started_solid: bool,
}

pub(crate) struct RigidbodyCharacter {
    pub(super) body_id: BodyId,
    pub(super) feet_box_id: ShapeId,
    pub(super) own_shapes: Vec<ShapeId>,

    pub(super) ground_normal: Vec3,
    pub(super) ground_velocity: Vec3,
    pub(super) jump_cooldown: f32,
    pub on_ground: bool,
    pub sprint: bool,

    pub(super) did_step: bool,
    pub(super) step_position: Pos,

    pub(super) last_wish_velocity: Vec3,
    pub(super) mass_center_world: Pos,

    // Debug draw collected during the step.
    pub debug_segs: Vec<f32>,
    pub debug_pts: Vec<f32>,
}

impl RigidbodyCharacter {
    pub(crate) fn position(&self, world: &World) -> Pos {
        body_get_position(world, self.body_id)
    }

    fn draw_line(&mut self, a: Pos, b: Pos, color: u32) {
        self.debug_segs.extend_from_slice(&[
            a.x as f32,
            a.y as f32,
            a.z as f32,
            b.x as f32,
            b.y as f32,
            b.z as f32,
            color as f32,
        ]);
    }

    fn draw_point(&mut self, p: Pos, size: f32, color: u32) {
        self.debug_pts
            .extend_from_slice(&[p.x as f32, p.y as f32, p.z as f32, color as f32, size]);
    }

    /// C `TraceBody` (sample_character.cpp:786): cast the body box from `from` to
    /// `to` with a radius/height scale, ignoring the character's own shapes.
    fn trace_body(
        &self,
        world: &World,
        from: Pos,
        to: Pos,
        radius_scale: f32,
        height_scale: f32,
    ) -> TraceResult {
        let mut result = TraceResult {
            end_position: to,
            normal: VEC3_AXIS_Y,
            hit_point: to,
            fraction: 1.0,
            hit: false,
            started_solid: false,
        };

        let translation = sub_pos(to, from);
        let translation_len = length(translation);
        if translation_len < 1e-6 {
            return result;
        }

        let half_w = BODY_RADIUS * 0.5 * radius_scale;
        let half_h = TOTAL_HEIGHT * height_scale * 0.5;
        let half_d = BODY_RADIUS * 0.5 * radius_scale;

        let box_min_y = 0.0f32;
        let box_max_y = TOTAL_HEIGHT * height_scale;
        let box_center_y = (box_min_y + box_max_y) * 0.5;

        let mut proxy = ShapeProxy {
            points: [VEC3_ZERO; box3d_rust::constants::MAX_SHAPE_CAST_POINTS],
            count: 8,
            radius: 0.0,
        };
        for i in 0..8 {
            let sx = if i & 1 != 0 { half_w } else { -half_w };
            let sy = if i & 2 != 0 { half_h } else { -half_h };
            let sz = if i & 4 != 0 { half_d } else { -half_d };
            proxy.points[i] = Vec3 {
                x: sx,
                y: box_center_y + sy,
                z: sz,
            };
        }

        // ClosestShapeCastCallback state (sample_character.cpp:604).
        let ignore = &self.own_shapes;
        let mut closest_fraction = 1.0f32;
        let mut closest_normal = VEC3_AXIS_Y;
        let mut closest_point = to;
        let mut hit = false;
        let mut started_solid = false;

        let filter = default_query_filter();
        world_cast_shape(
            world,
            from,
            &proxy,
            translation,
            &filter,
            |shape_id, point, normal, fraction, _um, _ti, _ci| {
                for s in ignore {
                    if s.index1 == shape_id.index1 && s.generation == shape_id.generation {
                        return -1.0;
                    }
                }
                if fraction == 0.0 {
                    started_solid = true;
                    return -1.0;
                }
                if fraction < closest_fraction {
                    closest_fraction = fraction;
                    closest_normal = normal;
                    closest_point = point;
                    hit = true;
                }
                closest_fraction
            },
        );

        result.started_solid = started_solid;
        if hit {
            result.hit = true;
            result.fraction = closest_fraction;
            result.normal = closest_normal;
            result.hit_point = closest_point;
            result.end_position = offset_pos(from, mul_sv(closest_fraction, translation));
        }
        result
    }

    fn is_standable_surface(&self, normal: Vec3) -> bool {
        let max_slope_cos = (MAX_SLOPE_ANGLE * PI / 180.0).cos();
        dot(normal, VEC3_AXIS_Y) >= max_slope_cos
    }

    fn feet_position(&self, world: &World) -> Pos {
        let p = body_get_position(world, self.body_id);
        y_off(p, -TOTAL_HEIGHT * 0.5)
    }

    /// C `CategorizeGround` (sample_character.cpp:870).
    fn categorize_ground(&mut self, world: &World) {
        let feet = self.feet_position(world);
        let from = y_off(feet, 4.0 * SRC);
        let to = y_off(feet, -2.0 * SRC);

        let mut radius_scale = 1.0f32;
        let mut tr = self.trace_body(world, from, to, radius_scale, 0.5);

        while tr.started_solid || (tr.hit && !self.is_standable_surface(tr.normal)) {
            radius_scale -= 0.1;
            if radius_scale < 0.7 {
                self.update_ground(false, VEC3_AXIS_Y);
                self.draw_line(from, to, colors::RED);
                return;
            }
            tr = self.trace_body(world, from, to, radius_scale, 0.5);
        }

        if !tr.started_solid
            && tr.hit
            && self.is_standable_surface(tr.normal)
            && self.jump_cooldown <= 0.0
        {
            self.update_ground(true, tr.normal);
            self.draw_line(from, tr.hit_point, colors::GREEN);
            self.draw_point(tr.hit_point, 5.0, colors::GREEN);
        } else {
            self.update_ground(false, VEC3_AXIS_Y);
            self.draw_line(from, to, colors::GRAY);
        }
    }

    fn update_ground(&mut self, on_ground: bool, normal: Vec3) {
        self.on_ground = on_ground;
        self.ground_normal = normal;
        if !on_ground {
            self.ground_velocity = VEC3_ZERO;
        }
    }

    /// C `Reground` (sample_character.cpp:918): snap to the surface when on ground.
    fn reground(&mut self, world: &mut World, step_size: f32) {
        if !self.on_ground {
            return;
        }
        let pos = body_get_position(world, self.body_id);
        let from = Pos {
            x: pos.x,
            y: pos.y + 0.05,
            z: pos.z,
        };
        let to = y_off(pos, -step_size);

        let mut radius_scale = 1.0f32;
        let mut tr = self.trace_body(world, from, to, radius_scale, 0.5);
        while tr.started_solid {
            radius_scale -= 0.1;
            if radius_scale < 0.7 {
                return;
            }
            tr = self.trace_body(world, from, to, radius_scale, 0.5);
        }

        if tr.hit {
            let target_pos = Pos {
                x: tr.end_position.x,
                y: tr.end_position.y + 0.01,
                z: tr.end_position.z,
            };
            let delta_y = target_pos.y - pos.y;
            let rot = body_get_transform(world, self.body_id).q;
            body_set_transform(world, self.body_id, target_pos, rot);

            if delta_y > 0.01 {
                let mut vel = body_get_linear_velocity(world, self.body_id);
                vel.y = 0.0;
                body_set_linear_velocity(world, self.body_id, vel);
            }
            self.draw_line(from, tr.end_position, colors::CYAN);
        }
    }

    /// C `TryStep` (sample_character.cpp:965): 4-phase trace-based step-up.
    fn try_step(&mut self, world: &mut World, max_step_height: f32) -> bool {
        let pos = body_get_position(world, self.body_id);
        let vel = body_get_linear_velocity(world, self.body_id);

        if !self.on_ground {
            return false;
        }

        let h_vel = Vec3 {
            x: vel.x,
            y: 0.0,
            z: vel.z,
        };
        let h_speed = length(h_vel);
        if h_speed < 0.01 {
            return false;
        }
        let move_dir = Vec3 {
            x: h_vel.x / h_speed,
            y: 0.0,
            z: h_vel.z / h_speed,
        };

        // Phase 1 — FORWARD.
        let forward_dist = h_speed * (1.0 / 60.0) + BODY_RADIUS;
        let forward_from = offset_pos(pos, mul_sv(-SKIN, move_dir));
        let forward_to = offset_pos(pos, mul_sv(forward_dist, move_dir));

        let mut radius_scale = 1.0f32;
        let mut tr_forward = self.trace_body(world, forward_from, forward_to, radius_scale, 1.0);
        while tr_forward.started_solid {
            radius_scale -= 0.1;
            if radius_scale < 0.6 {
                self.draw_line(forward_from, forward_to, colors::RED);
                return false;
            }
            tr_forward = self.trace_body(world, forward_from, forward_to, radius_scale, 1.0);
        }
        if !tr_forward.hit {
            return false;
        }
        self.draw_line(forward_from, tr_forward.end_position, colors::YELLOW);

        // Phase 2 — UP.
        let up_from = tr_forward.end_position;
        let up_to = y_off(up_from, max_step_height);
        let tr_up = self.trace_body(world, up_from, up_to, radius_scale, 1.0);
        if tr_up.started_solid {
            self.draw_line(up_from, up_to, colors::RED);
            return false;
        }
        let top_pos = if tr_up.hit { tr_up.end_position } else { up_to };
        let up_distance = top_pos.y - up_from.y;
        if up_distance < 0.005 {
            self.draw_line(up_from, top_pos, colors::RED);
            return false;
        }
        self.draw_line(up_from, top_pos, colors::YELLOW);

        // Phase 3 — ACROSS.
        let across_dist = forward_dist * (1.0 - tr_forward.fraction) + BODY_RADIUS * 0.5;
        let across_from = top_pos;
        let across_to = offset_pos(top_pos, mul_sv(across_dist, move_dir));
        let tr_across = self.trace_body(world, across_from, across_to, radius_scale, 1.0);
        if tr_across.started_solid {
            self.draw_line(across_from, across_to, colors::RED);
            return false;
        }
        let across_pos = if tr_across.hit {
            tr_across.end_position
        } else {
            across_to
        };
        self.draw_line(across_from, across_pos, colors::YELLOW);

        // Phase 4 — DOWN.
        let down_from = across_pos;
        let down_to = y_off(across_pos, -max_step_height);
        let tr_down = self.trace_body(world, down_from, down_to, radius_scale, 1.0);
        if !tr_down.hit {
            self.draw_line(down_from, down_to, colors::RED);
            return false;
        }
        if !self.is_standable_surface(tr_down.normal) {
            self.draw_line(down_from, tr_down.end_position, colors::RED);
            return false;
        }
        let step_height = tr_down.end_position.y - pos.y;
        if step_height < 0.01 {
            return false;
        }
        self.draw_line(down_from, tr_down.end_position, colors::YELLOW);
        self.draw_point(tr_down.end_position, 8.0, colors::YELLOW);

        let step_pos = Pos {
            x: tr_down.end_position.x,
            y: tr_down.end_position.y + 0.01,
            z: tr_down.end_position.z,
        };
        let rot = body_get_transform(world, self.body_id).q;
        body_set_transform(world, self.body_id, step_pos, rot);

        let mut new_vel = body_get_linear_velocity(world, self.body_id);
        new_vel.x *= 0.9;
        new_vel.y = 0.0;
        new_vel.z *= 0.9;
        body_set_linear_velocity(world, self.body_id, new_vel);

        self.step_position = step_pos;
        true
    }

    fn restore_step(&mut self, world: &mut World) {
        if !self.did_step {
            return;
        }
        let rot = body_get_transform(world, self.body_id).q;
        body_set_transform(world, self.body_id, self.step_position, rot);
        self.did_step = false;
    }

    fn add_clamped(current: Vec3, add: Vec3, max_add_length: f32) -> Vec3 {
        let add_len = length(add);
        let add = if add_len > max_add_length && add_len > 0.0 {
            mul_sv(max_add_length / add_len, add)
        } else {
            add
        };
        current + add
    }

    /// C `UpdateMassCenter` (sample_character.cpp:1121).
    pub(super) fn update_mass_center(&mut self, world: &mut World, wish_speed: f32) {
        let mut mass_data = body_get_mass_data(world, self.body_id);
        let half_height = TOTAL_HEIGHT * 0.5;
        if self.on_ground {
            let center_offset = clamp_float(wish_speed, 0.0, half_height);
            mass_data.center = Vec3 {
                x: 0.0,
                y: center_offset - half_height,
                z: 0.0,
            };
        } else {
            mass_data.center = VEC3_ZERO;
        }
        body_set_mass_data(world, self.body_id, mass_data);
    }

    /// C `UpdateBody` (sample_character.cpp:1141).
    fn update_body(&mut self, world: &mut World, wish_velocity: Vec3) {
        let wish_len = length(wish_velocity);
        let vel = body_get_linear_velocity(world, self.body_id);
        let vel_len = length(vel);

        let mut feet_friction = 0.0f32;
        if self.on_ground {
            let wants_brakes = wish_len < (5.0 * SRC) || wish_len < vel_len * 0.9;
            if wants_brakes {
                feet_friction = 1.0 + 100.0 * BRAKE_POWER * SURFACE_FRICTION;
            }
        }
        shape_set_friction(world, self.feet_box_id, feet_friction);

        self.update_mass_center(world, wish_len);

        let mut wants_gravity = false;
        if !self.on_ground {
            wants_gravity = true;
        }
        if vel_len > (1.0 * SRC) {
            wants_gravity = true;
        }
        if length(self.ground_velocity) > (1.0 * SRC) {
            wants_gravity = true;
        }
        body_set_gravity_scale(
            world,
            self.body_id,
            if wants_gravity {
                CHARACTER_GRAVITY / 10.0
            } else {
                0.0
            },
        );

        let wants_damping =
            self.on_ground && wish_len < (1.0 * SRC) && length(self.ground_velocity) < (1.0 * SRC);
        body_set_linear_damping(
            world,
            self.body_id,
            if wants_damping {
                10.0 * BRAKE_POWER
            } else {
                AIR_FRICTION
            },
        );
    }

    /// C `AddVelocity` (sample_character.cpp:1180): s&box MoveMode.Walk model.
    fn add_velocity(&mut self, world: &mut World, wish_velocity: Vec3) {
        let wish = Vec3 {
            x: wish_velocity.x,
            y: 0.0,
            z: wish_velocity.z,
        };
        let wish_len = length(wish);
        if wish_len < 0.001 {
            return;
        }

        let ground_friction_factor = 0.25 + SURFACE_FRICTION * 10.0;
        let vel = body_get_linear_velocity(world, self.body_id);
        let saved_y = vel.y;

        let mut velocity = vel - self.ground_velocity;
        let speed = length(velocity);
        let max_speed = wish_len.max(speed);

        if self.on_ground {
            let amount = 1.0 * ground_friction_factor;
            velocity = Self::add_clamped(velocity, mul_sv(amount, wish), wish_len * amount);
        } else {
            let amount = 0.05f32;
            velocity = Self::add_clamped(velocity, mul_sv(amount, wish), wish_len);
        }

        let new_speed = length(velocity);
        if new_speed > max_speed && new_speed > 0.0 {
            velocity = mul_sv(max_speed / new_speed, velocity);
        }

        velocity += self.ground_velocity;
        if self.on_ground {
            velocity.y = saved_y;
        }
        body_set_linear_velocity(world, self.body_id, velocity);
    }

    /// C `PreStep` (sample_character.cpp:1226).
    #[allow(clippy::too_many_arguments)]
    pub(super) fn pre_step(
        &mut self,
        world: &mut World,
        time_step: f32,
        forward: Vec3,
        right: Vec3,
        throttle_x: f32,
        throttle_y: f32,
    ) {
        if self.jump_cooldown > 0.0 {
            self.jump_cooldown -= time_step;
        }

        let max_speed = if self.sprint { RUN_SPEED } else { WALK_SPEED };
        let mut wish_velocity =
            mul_sv(max_speed * throttle_x, forward) + mul_sv(max_speed * throttle_y, right);
        let wish_speed = length(wish_velocity);
        if wish_speed > max_speed {
            wish_velocity = mul_sv(max_speed / wish_speed, wish_velocity);
        }
        self.last_wish_velocity = wish_velocity;

        self.update_body(world, wish_velocity);
        self.add_velocity(world, wish_velocity);
        self.did_step = self.try_step(world, STEP_UP_HEIGHT);

        self.mass_center_world = body_get_world_center(world, self.body_id);
    }

    /// C `PostStep` (sample_character.cpp:1256).
    pub(super) fn post_step(&mut self, world: &mut World, _time_step: f32) {
        self.restore_step(world);
        self.reground(world, STEP_DOWN_HEIGHT);
        self.categorize_ground(world);
    }

    /// C `Jump` (sample_character.cpp:1268).
    pub(super) fn jump(&mut self, world: &mut World) {
        if self.on_ground && self.jump_cooldown <= 0.0 {
            let mut velocity = body_get_linear_velocity(world, self.body_id);
            velocity.y = JUMP_SPEED;
            body_set_linear_velocity(world, self.body_id, velocity);
            self.on_ground = false;
            self.jump_cooldown = JUMP_COOLDOWN_TIME;
        }
    }

    /// C `DrawDebug` (sample_character.cpp:1290).
    pub(super) fn draw_debug(&mut self, world: &World) {
        let pos = body_get_position(world, self.body_id);
        let vel = body_get_linear_velocity(world, self.body_id);
        self.draw_line(pos, offset_pos(pos, vel), colors::PURPLE);
        self.draw_line(
            pos,
            offset_pos(pos, self.last_wish_velocity),
            colors::ORANGE,
        );
        self.draw_point(self.mass_center_world, 8.0, colors::YELLOW);
        if self.on_ground {
            let bottom = y_off(pos, -TOTAL_HEIGHT * 0.5);
            self.draw_line(
                bottom,
                offset_pos(bottom, mul_sv(0.3, self.ground_normal)),
                colors::GREEN,
            );
        }
    }
}
