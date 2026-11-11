//! Character / Dynamic Mover — a 1:1 port of `DynamicMoverSample`
//! (`sample_character.cpp:848`, upstream 3362633), the playground for the vendored
//! [`DynamicMover`]: stairs, a suspension bridge, ramps / a wall / a steep slope / a curb,
//! ledges, a vertical and a horizontal moving platform (ridden through
//! `platformMaterialId`), a seesaw, a box pyramid, a spring door, a weld-sprung ball pit,
//! an exploding pressure plate on a prismatic joint, a big ball, a motorized sweeper, a
//! pillar jump puzzle, 17 spinning gems to collect, the capsule ↔ box mover swap, and the
//! aim-and-shoot ball launcher with the over-the-shoulder aim camera.
//!
//! - [`scene`] — the constructor (C body / shape / joint creation order).
//! - [`camera`] — the host camera basis + `GetMoverInput` slice the sample relies on.
//! - this module — `Step` / `FollowMover` / `ShootBall` / `CollectGems` /
//!   `UpdatePressurePlate` / `SwapMoverShape` and the render list.
//! - [`exports`] — the wasm surface the Character page drives.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

pub(crate) mod camera;
mod exports;
mod scene;
mod scene_parts;

pub(crate) use exports::step;
pub(crate) use scene::build_dynamic_mover;
use scene::get_platform_position;

use super::dynamic_mover::DynamicMover;
use super::hulls::HullVis;
use crate::vis::{hull_triangles, VisBody};
use box3d_rust::body::{
    body_get_position, body_get_shapes, body_get_transform, body_is_valid,
    body_set_linear_velocity, body_set_target_transform, body_set_transform, create_body,
    destroy_body,
};
use box3d_rust::debug_draw::HexColor;
use box3d_rust::geometry::{Capsule, Sphere};
use box3d_rust::hull::{make_offset_box_hull, HullData};
use box3d_rust::id::{BodyId, JointId, NULL_BODY_ID};
use box3d_rust::joint::prismatic_joint_get_translation;
use box3d_rust::math_functions::{
    cross, length, lerp, make_quat_from_matrix, max_float, min_float, normalize, offset_pos,
    sub_pos, Matrix3, Pos, Vec3, WorldTransform, PI, QUAT_IDENTITY, VEC3_AXIS_Y, VEC3_ZERO,
};
use box3d_rust::shape::{
    create_capsule_shape, create_hull_shape, create_sphere_shape, destroy_shape, shape_get_body,
    shape_get_density, shape_is_valid,
};
use box3d_rust::types::{
    default_body_def, default_explosion_def, default_shape_def, BodyType, Filter, QueryFilter,
};
use box3d_rust::world::{world_cast_ray_closest, world_explode, world_get_sensor_events, World};
use camera::{get_mover_input, KeysDown, SampleCamera};

// Collision filter bits (C `DynamicMoverSample` statics, :1832).
pub(crate) const STATIC_BIT: u64 = 0x1;
pub(crate) const MOVER_BIT: u64 = 0x2;
pub(crate) const GEM_BIT: u64 = 0x4;
pub(crate) const DYNAMIC_BIT: u64 = 0x10;
pub(crate) const PROJECTILE_BIT: u64 = 0x20;
pub(crate) const DEBRIS_BIT: u64 = 0x40;
pub(crate) const ALL_BITS: u64 = !0u64;

pub(crate) const BALL_CAPACITY: usize = 50;

pub(crate) const LAUNCHER_RADIUS: f32 = 0.03;
pub(crate) const LAUNCHER_BACK: f32 = 0.05;
pub(crate) const LAUNCHER_FRONT: f32 = 0.55;
pub(crate) const BALL_RADIUS: f32 = 0.1;

pub(crate) const PLATFORM_MATERIAL: u64 = 7;
pub(crate) const PLATFORM_COUNT: usize = 2;
pub(crate) const PILLAR_COUNT: usize = 6;
pub(crate) const GEM_COUNT: usize = 17;
pub(crate) const SWEEPER_CENTER: Pos = Pos {
    x: 0.0,
    y: 0.0,
    z: -16.0,
};
pub(crate) const PLATE_CENTER: Pos = Pos {
    x: 0.0,
    y: 0.05,
    z: 26.5,
};
pub(crate) const PLATE_TRAVEL: f32 = 0.1;

/// C `DynamicMoverSample::Aim` (:1523).
#[derive(Clone, Copy, Debug)]
pub(crate) struct Aim {
    pub shoulder: Pos,
    pub muzzle: Pos,
    pub direction: Vec3,
    pub right: Vec3,
    pub velocity: Vec3,
}

/// The sample's members (C `DynamicMoverSample`, :1855). The world lives beside it in
/// the Character page state; every method that touches physics takes `&mut World`.
pub(crate) struct DynamicMoverSample {
    pub mover: DynamicMover,
    pub camera: SampleCamera,
    /// C `IsKeyDown` table (page-fed each frame).
    pub keys: KeysDown,
    /// C `ImGui::GetIO().DeltaTime` — the frame time the aim blend runs on.
    pub frame_dt: f32,

    pub plate_joint_id: JointId,
    pub plate_armed: bool,
    pub platform_ids: [BodyId; PLATFORM_COUNT],
    pub gem_ids: [BodyId; GEM_COUNT],
    pub time: f32,
    pub jump_released: bool,
    pub draw_probes: bool,
    pub use_box: bool,

    pub ball_ids: [BodyId; BALL_CAPACITY],
    pub ball_index: usize,
    pub ball_speed: f32,
    pub aiming: bool,
    pub aim_blend: f32,
    pub shoot_requested: bool,
    #[allow(dead_code)] // kept like C's m_launcherHull; the render mesh is built from it
    pub launcher_hull: Option<HullData>,

    /// C `SetBodyAlpha( moverId, alpha )` — the mover fades while the aim camera zooms in.
    pub body_alpha: f32,
    /// The radius the camera renders from this frame (C leaves `m_radius` restored to
    /// the wheel zoom after `FollowMover`, but the view was built from this value).
    pub view_radius: f32,
    /// C `m_didStep`.
    pub did_step: bool,

    /// Persistent render bodies (everything but gems, balls, and the mover).
    pub static_vis: Vec<VisBody>,
    /// Hull channel entries built with the scene (pillars + launcher, last).
    pub hulls: Vec<HullVis>,
}

impl DynamicMoverSample {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        mover: DynamicMover,
        camera: SampleCamera,
        plate_joint_id: JointId,
        platform_ids: [BodyId; PLATFORM_COUNT],
        gem_ids: [BodyId; GEM_COUNT],
        static_vis: Vec<VisBody>,
        mut hulls: Vec<HullVis>,
        launcher_hull: Option<HullData>,
    ) -> Self {
        if let Some(hull) = &launcher_hull {
            hulls.push(HullVis::render_only(
                hull_triangles(hull),
                HexColor::DIM_GRAY.0,
            ));
        }
        let view_radius = camera.radius;
        DynamicMoverSample {
            mover,
            camera,
            keys: KeysDown::default(),
            frame_dt: 1.0 / 60.0,
            plate_joint_id,
            plate_armed: true,
            platform_ids,
            gem_ids,
            time: 0.0,
            jump_released: true,
            draw_probes: true,
            use_box: false,
            ball_ids: [NULL_BODY_ID; BALL_CAPACITY],
            ball_index: 0,
            ball_speed: 50.0,
            aiming: false,
            aim_blend: 0.0,
            shoot_requested: false,
            launcher_hull,
            body_alpha: 1.0,
            view_radius,
            did_step: false,
            static_vis,
            hulls,
        }
    }

    /// C `DynamicMoverSample::Step` (:1742) through `Sample::Step` and the post-step
    /// gem / plate logic. Drawing is separate ([`Self::draw`]).
    pub(crate) fn step(&mut self, world: &mut World, time_step: f32, sub_step_count: i32) {
        // The Character page has no pause, so `pause` is always false here.
        let pause = false;

        if !pause && time_step > 0.0 {
            self.time += time_step;

            for i in 0..PLATFORM_COUNT {
                let target = WorldTransform {
                    p: get_platform_position(i, self.time),
                    q: QUAT_IDENTITY,
                };
                body_set_target_transform(world, self.platform_ids[i], target, time_step, true);
            }

            let input = get_mover_input(&self.camera, &self.keys);

            // Jumping again needs a fresh press
            if input.jump {
                if self.jump_released && self.mover.jump(world) {
                    self.jump_released = false;
                }
            } else {
                self.jump_released = true;
            }

            let mut speed_scale = 1.0f32;
            if input.walk || self.aiming {
                speed_scale = 0.25;
            } else if input.sprint {
                speed_scale = 1.5;
            }
            self.mover.update(
                world,
                time_step,
                input.forward,
                input.right,
                input.throttle,
                speed_scale,
            );
        }

        // The base step draws the world, so the camera has to be placed before it.
        // Otherwise the world renders from the previous eye and the overlay from the new one.
        let mover_position = body_get_position(world, self.mover.body_id());
        self.follow_mover(world, mover_position);

        // Shoot before the step so the ball is in the world when it is drawn
        if self.aiming && self.shoot_requested {
            let lead_time = if !pause { time_step } else { 0.0 };
            let aim = self.compute_aim(world);
            self.shoot_ball(world, &aim, lead_time);
        }

        self.shoot_requested = false;

        // Sample::Step: the world settings it re-applies each step are the world
        // defaults here (sleep / warm starting / continuous on, 2 restitution iterations).
        self.did_step = false;
        if time_step > 0.0 {
            world.step(time_step, sub_step_count);
            self.did_step = true;
        }

        if self.did_step {
            self.collect_gems(world);
            self.update_pressure_plate(world);
        }
    }

    /// C `DynamicMoverSample::UpdatePressurePlate` (:1438).
    fn update_pressure_plate(&mut self, world: &mut World) {
        let translation = prismatic_joint_get_translation(world, self.plate_joint_id);

        if self.plate_armed && translation < -PLATE_TRAVEL + 0.01 {
            let mut def = default_explosion_def();
            def.position = offset_pos(
                PLATE_CENTER,
                Vec3 {
                    x: 0.0,
                    y: -0.1,
                    z: 0.0,
                },
            );
            def.radius = 10.0;
            def.falloff = 10.0;
            def.impulse_per_area = 3000.0;
            world_explode(world, &def);

            self.mover.launch();
            self.plate_armed = false;
        } else if !self.plate_armed && translation > -0.01 {
            self.plate_armed = true;
        }
    }

    /// C `DynamicMoverSample::CollectGems` (:1460).
    fn collect_gems(&mut self, world: &mut World) {
        // Copy the begin events out: destroying a gem mutates the world.
        let begin_events: Vec<_> = world_get_sensor_events(world).begin_events.to_vec();
        for event in begin_events {
            if !shape_is_valid(world, event.visitor_shape_id)
                || !shape_is_valid(world, event.sensor_shape_id)
            {
                continue;
            }

            let visitor_id = shape_get_body(world, event.visitor_shape_id);
            if visitor_id != self.mover.body_id() {
                continue;
            }

            let gem_id = shape_get_body(world, event.sensor_shape_id);
            for id in self.gem_ids.iter_mut() {
                if *id == gem_id {
                    destroy_body(world, *id);
                    *id = NULL_BODY_ID;
                    break;
                }
            }
        }
    }

    /// C `DynamicMoverSample::SwapMoverShape` (:1491) — the "Box Mover" checkbox.
    pub(crate) fn swap_mover_shape(&mut self, world: &mut World) {
        let body_id = self.mover.body_id();

        let shape_id = body_get_shapes(world, body_id, 1)[0];
        let density = shape_get_density(world, shape_id);
        destroy_shape(world, shape_id, false);

        let mut shape_def = default_shape_def();
        shape_def.base_material.friction = 0.0;
        shape_def.base_material.restitution = 0.0;
        shape_def.filter = Filter {
            category_bits: MOVER_BIT,
            mask_bits: ALL_BITS,
            group_index: 0,
        };
        shape_def.enable_sensor_events = true;

        let capsule = self.mover.capsule;
        if self.use_box {
            let (r, hy, c) = box_mover_extents(&capsule);
            let box_hull = make_offset_box_hull(r, hy, r, c);
            shape_def.density = density;
            create_hull_shape(world, body_id, &shape_def, &box_hull.base);
        } else {
            shape_def.density = density;
            create_capsule_shape(world, body_id, &shape_def, &capsule);
        }
    }

    /// C `DynamicMoverSample::ComputeAim` (:1532).
    pub(crate) fn compute_aim(&self, world: &World) -> Aim {
        let capsule = &self.mover.capsule;

        let mut right = self.camera.get_right();
        right.y = 0.0;
        right = normalize(right);

        let shoulder_offset = capsule.center2 + (capsule.radius + LAUNCHER_RADIUS) * right;
        let shoulder = offset_pos(
            body_get_position(world, self.mover.body_id()),
            shoulder_offset,
        );
        let direction = -self.camera.get_forward();
        Aim {
            shoulder,
            muzzle: offset_pos(shoulder, (LAUNCHER_FRONT + LAUNCHER_RADIUS) * direction),
            direction,
            right,
            velocity: self.ball_speed * direction,
        }
    }

    /// C `DynamicMoverSample::ShootBall` (:1552).
    pub(crate) fn shoot_ball(&mut self, world: &mut World, aim: &Aim, lead_time: f32) {
        let ball_id = self.ball_ids[self.ball_index];
        if body_is_valid(world, ball_id) {
            destroy_body(world, ball_id);
        }

        // Back up the spawn point so we see it drawn at the muzzle.
        let back_distance = min_float(
            self.ball_speed * lead_time,
            LAUNCHER_FRONT + LAUNCHER_RADIUS,
        );

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        // C `aim.muzzle - backDistance * aim.direction` (b3Pos minus b3Vec3).
        body_def.position = offset_pos(aim.muzzle, -(back_distance * aim.direction));
        body_def.linear_velocity = aim.velocity;

        // This is good use case for the bullet setting because this is a fast projectile.
        body_def.is_bullet = true;
        body_def.name = String::from("ball");
        let new_id = create_body(world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density = 4000.0;
        shape_def.base_material.friction = 0.2;
        shape_def.base_material.restitution = 0.5;
        shape_def.base_material.rolling_resistance = 0.1;
        shape_def.base_material.custom_color = HexColor::ORANGE_RED.0;
        shape_def.filter = Filter {
            category_bits: PROJECTILE_BIT,
            mask_bits: ALL_BITS & !MOVER_BIT,
            group_index: 0,
        };
        let sphere = Sphere {
            center: VEC3_ZERO,
            radius: BALL_RADIUS,
        };
        create_sphere_shape(world, new_id, &shape_def, &sphere);

        self.ball_ids[self.ball_index] = new_id;
        self.ball_index = (self.ball_index + 1) % BALL_CAPACITY;
    }

    /// C `DynamicMoverSample::DrawWeapon` transform (:1587): the cylinder axis is local
    /// y; roll with the camera to keep it steady.
    pub(crate) fn weapon_transform(aim: &Aim) -> WorldTransform {
        let basis = Matrix3 {
            cx: aim.right,
            cy: aim.direction,
            cz: cross(aim.right, aim.direction),
        };
        WorldTransform {
            p: aim.shoulder,
            q: make_quat_from_matrix(&basis),
        }
    }

    /// C `DynamicMoverSample::MouseDown` (:1597), third-person branch. The free-camera
    /// branch (`Sample::MouseDown`: selection / mouse joint) is the page's orbit camera.
    pub(crate) fn mouse_down(&mut self, button: i32) {
        if self.camera.third_person {
            if button == 1 {
                self.aiming = true;
            } else if button == 0 && self.aiming {
                self.shoot_requested = true;
            }
        }
    }

    /// C `DynamicMoverSample::MouseUp` (:1616).
    pub(crate) fn mouse_up(&mut self, button: i32) {
        if button == 1 {
            self.aiming = false;
        }
    }

    /// C `DynamicMoverSample::FollowMover` (:1627) — pull the boom in when the view to
    /// the mover is blocked. Leaves `camera.pivot` and [`Self::view_radius`] as the view
    /// to render, and `camera.radius` restored to the wheel zoom.
    fn follow_mover(&mut self, world: &World, position: Pos) {
        if !self.camera.third_person {
            self.aiming = false;
            self.aim_blend = 0.0;
            self.body_alpha = 1.0;
            self.view_radius = self.camera.radius;
            return;
        }

        // Frame time so the blend still runs while paused
        let blend_time = 0.2f32;
        let blend_step = self.frame_dt / blend_time;
        let delta = if self.aiming { blend_step } else { -blend_step };
        self.aim_blend = (self.aim_blend + delta).clamp(0.0, 1.0);
        let blend = self.aim_blend * self.aim_blend * (3.0 - 2.0 * self.aim_blend);

        // The zoomed camera sits right behind the mover, so fade it to see past
        self.body_alpha = 1.0 - 0.7 * blend;

        // The wheel zoom stays in the camera radius, aiming only overrides it for this frame
        let saved_radius = self.camera.radius;
        let aim_radius = min_float(saved_radius, 0.8);
        self.camera.radius = saved_radius + blend * (aim_radius - saved_radius);

        self.camera.pivot = position;
        self.camera.update_transform();

        // Over the shoulder so the mover and launcher sit low and left of the crosshair
        if blend > 0.0 {
            let shoulder = 0.75 * VEC3_AXIS_Y + 0.6 * self.camera.get_right();
            self.camera.pivot = offset_pos(position, blend * shoulder);
            self.camera.update_transform();
        }

        let translation = sub_pos(self.camera.world_eye, position);
        let distance = length(translation);
        if distance > 0.01 {
            let filter = QueryFilter {
                category_bits: STATIC_BIT,
                mask_bits: STATIC_BIT,
                id: 0,
                name: String::from("camera"),
            };
            let result = world_cast_ray_closest(world, position, translation, &filter);
            if result.hit {
                self.camera.radius = max_float(0.1, result.fraction * distance - 0.15);
                self.camera.update_transform();
            }
        }

        self.view_radius = self.camera.radius;
        self.camera.radius = saved_radius;
    }

    /// C `DynamicMoverSample::Keyboard` KEY_K (:1686) — align movement for video: snap the
    /// yaw to a quarter turn, pitch to 15°, and the mover to the integer x / z grid.
    pub(crate) fn snap_for_video(&mut self, world: &mut World) {
        let quarter_turn = 0.5 * PI;
        self.camera.yaw = quarter_turn * (self.camera.yaw / quarter_turn).round();
        self.camera.pitch = 15.0 * box3d_rust::math_functions::DEG_TO_RAD;

        let body_id = self.mover.body_id();
        let mut position = body_get_position(world, body_id);
        position.x = (position.x as f32).round() as _;
        position.z = (position.z as f32).round() as _;
        let rotation = body_get_transform(world, body_id).q;
        body_set_transform(world, body_id, position, rotation);
        body_set_linear_velocity(world, body_id, VEC3_ZERO);
    }

    /// The gems still in the world.
    pub(crate) fn gems_remaining(&self) -> usize {
        self.gem_ids.iter().filter(|id| id.is_non_null()).count()
    }

    /// The per-frame render list: persistent bodies, live gems, live balls, then the
    /// mover (capsule or box). Returns the list and the mover's index in it.
    pub(crate) fn render_bodies(&self, world: &World) -> (Vec<VisBody>, usize) {
        let mut out: Vec<VisBody> = Vec::with_capacity(self.static_vis.len() + 70);
        for v in &self.static_vis {
            out.push(VisBody {
                body_index: v.body_index,
                kind: v.kind,
                params: v.params,
                local: v.local,
                color: v.color,
            });
        }
        let gem_local = scene::gem_local();
        for id in self.gem_ids.iter().filter(|id| id.is_non_null()) {
            out.push(VisBody::box_local(id.index1 - 1, 0.2, 0.2, 0.2, gem_local));
        }
        for id in self.ball_ids.iter() {
            if id.is_non_null() && body_is_valid(world, *id) {
                out.push(VisBody::sphere_body(id.index1 - 1, BALL_RADIUS));
            }
        }
        let mover_index = out.len();
        let body_index = self.mover.body_id().index1 - 1;
        if self.use_box {
            let (r, hy, c) = box_mover_extents(&self.mover.capsule);
            out.push(VisBody::box_local(
                body_index,
                r,
                hy,
                r,
                box3d_rust::math_functions::Transform {
                    p: c,
                    q: QUAT_IDENTITY,
                },
            ));
        } else {
            out.push(VisBody::capsule_body(body_index, &self.mover.capsule));
        }
        (out, mover_index)
    }

    /// The post-step draw block of C `Step` (:1805): probes and the launcher. Fills the
    /// debug buffers and places / hides the launcher hull (the last hull entry).
    pub(crate) fn draw(
        &self,
        world: &World,
        segs: &mut Vec<f32>,
        pts: &mut Vec<f32>,
        hulls: &mut [HullVis],
    ) {
        if self.draw_probes {
            self.mover.draw(segs, pts);
        }

        if let Some(launcher) = hulls.last_mut().filter(|h| h.body_id.is_null()) {
            if self.aiming {
                let aim = self.compute_aim(world);
                launcher.transform = Self::weapon_transform(&aim);
                launcher.visible = true;
            } else {
                launcher.visible = false;
            }
        }
    }
}

/// The box mover hull extents from the capsule (C `SwapMoverShape`):
/// `r`, `hy = 0.5 * |c2 - c1| + r`, center `c = lerp( c1, c2, 0.5 )`.
fn box_mover_extents(capsule: &Capsule) -> (f32, f32, Vec3) {
    let r = capsule.radius;
    let hy = 0.5 * length(capsule.center2 - capsule.center1) + r;
    let c = lerp(capsule.center1, capsule.center2, 0.5);
    (r, hy, c)
}

#[cfg(test)]
#[path = "tests.rs"]
mod tests;
