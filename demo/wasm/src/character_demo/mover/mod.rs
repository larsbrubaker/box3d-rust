//! Character / Geometric Mover — a 1:1 port of `GeometricMoverSample`
//! (`sample_character.cpp:334`, upstream 3362633; registered as "Geometric Mover" since
//! upstream 47d7f7c) driving the shared [`GeometricMover`] port over the real level.
//!
//! The mover is a query-driven capsule (not a rigid body): each step it damps and
//! accelerates its own velocity, probes the ground with a pogo ray / spring, then slides
//! against the world with `b3World_CollideMover` + `b3SolvePlanes` + `b3World_CastMover`,
//! pushes the dynamic bodies it leans on, and either clips its velocity or recovers it
//! from the position delta (the "Clip Velocity" control). A falling box and a toppling
//! tree sweep through the level; `b3Body_TimeOfImpactMover` reports where and how fast
//! each first strikes the mover (yellow / cyan impact arrows, the yellow ghost box).
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)]

mod scene;

pub(crate) use scene::build_mover;

use super::colors;
use super::dynamic::camera::{get_mover_input, KeysDown, SampleCamera};
use super::{capsule_style, CharacterState, DebugText, RenderCapsule, SceneState};
use crate::geometric_mover::{GeometricMover, MoverShapeUserData};
use box3d_rust::body::{
    body_get_transform, body_get_world_point_velocity, body_is_awake, body_set_angular_velocity,
    body_set_awake, body_set_linear_velocity, body_set_transform, body_time_of_impact_mover,
    destroy_body, BodyToiResult,
};
use box3d_rust::id::BodyId;
use box3d_rust::math_functions::{
    dot, length, make_matrix_from_quat, mul_sv, nlerp, offset_pos, perp, sub_pos, Pos, Vec3,
    WorldTransform, QUAT_IDENTITY, VEC3_ZERO,
};
use box3d_rust::types::default_query_filter;
use box3d_rust::world::World;

/// C `MoverImpact` (:326).
#[derive(Clone, Copy, Debug)]
pub(crate) struct MoverImpact {
    pub point: Pos,
    pub normal: Vec3,
    pub speed: f32,
    pub valid: bool,
}

impl Default for MoverImpact {
    fn default() -> Self {
        MoverImpact {
            point: Pos {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            normal: VEC3_ZERO,
            speed: 0.0,
            valid: false,
        }
    }
}

/// The `GeometricMoverSample` members (:822). The world lives in the page state.
pub(crate) struct GeometricMoverScene {
    pub mover: GeometricMover,
    pub camera: SampleCamera,
    pub keys: KeysDown,
    pub tree_id: BodyId,
    pub tree_spawn: Pos,
    pub tree_spin: Vec3,
    pub tree_timer: f32,
    pub tree_shape_user_data: MoverShapeUserData,
    pub tree_impact: MoverImpact,
    pub falling_box_id: BodyId,
    pub falling_spawn: Pos,
    #[allow(dead_code)] // C m_fallingExtent; the ghost hull is built from it at construction
    pub falling_extent: Vec3,
    pub falling_spin: Vec3,
    pub respawn_timer: f32,
    pub impact_transform: WorldTransform,
    pub box_impact: MoverImpact,
}

impl GeometricMoverScene {
    /// C `ResetFallingTree` (:658): spin about the base so the trunk sweeps down through
    /// anything standing in the fall line.
    fn reset_falling_tree(&mut self, world: &mut World) -> BodyId {
        destroy_body(world, self.tree_id);
        let (tree_id, _) = scene::create_falling_tree(
            world,
            self.tree_spawn,
            self.tree_spin,
            self.tree_shape_user_data,
        );
        self.tree_id = tree_id;
        tree_id
    }

    /// C `MakeImpact` (:665): the normal points from the body to the mover, so approach
    /// gives a positive speed.
    fn make_impact(&self, world: &World, body_id: BodyId, toi: &BodyToiResult) -> MoverImpact {
        let point_velocity = body_get_world_point_velocity(world, body_id, toi.point);
        let speed = dot(toi.normal, point_velocity - self.mover.velocity);
        MoverImpact {
            point: toi.point,
            normal: toi.normal,
            speed,
            valid: true,
        }
    }

    /// C `SettledForOneSecond` (:689): true once the body has held still for a second.
    fn settled_for_one_second(
        world: &World,
        body_id: BodyId,
        timer: &mut f32,
        time_step: f32,
    ) -> bool {
        if body_is_awake(world, body_id) {
            *timer = 0.0;
            return false;
        }

        // C: m_context->hertz > 0 ? 1 / hertz : 0, i.e. the step's time step.
        *timer += time_step;
        if *timer < 1.0 {
            return false;
        }

        *timer = 0.0;
        true
    }

    /// C `UpdateFallingTree` (:708): the same sweep as the box, but the tree reports
    /// what the character was struck with. Returns the new tree body after a reset.
    fn update_falling_tree(
        &mut self,
        world: &mut World,
        mover_start: Pos,
        tree_start: WorldTransform,
        time_step: f32,
    ) -> Option<BodyId> {
        let tree_end = body_get_transform(world, self.tree_id);
        let mover_translation = sub_pos(self.mover.position, mover_start);

        let toi = body_time_of_impact_mover(
            world,
            self.tree_id,
            mover_start,
            &self.mover.capsule,
            mover_translation,
            &default_query_filter(),
            tree_start,
            tree_end,
        );

        if toi.fraction > 0.0 {
            self.tree_impact = self.make_impact(world, self.tree_id, &toi);
        }

        if Self::settled_for_one_second(world, self.tree_id, &mut self.tree_timer, time_step) {
            return Some(self.reset_falling_tree(world));
        }
        None
    }

    /// C `UpdateFallingBox` (:731): sweep the mover capsule against the box motion from
    /// this step and keep the box pose at first touch. Both sweeps cover the same
    /// interval, so the mover translation comes from this step too.
    fn update_falling_box(
        &mut self,
        world: &mut World,
        mover_start: Pos,
        box_start: WorldTransform,
        time_step: f32,
    ) {
        let box_end = body_get_transform(world, self.falling_box_id);
        let mover_translation = sub_pos(self.mover.position, mover_start);

        let toi = body_time_of_impact_mover(
            world,
            self.falling_box_id,
            mover_start,
            &self.mover.capsule,
            mover_translation,
            &default_query_filter(),
            box_start,
            box_end,
        );

        // A zero fraction means the box was already resting against the mover, so keep the
        // pose from the step that first reached it.
        if toi.fraction > 0.0 {
            self.impact_transform.p =
                offset_pos(box_start.p, toi.fraction * sub_pos(box_end.p, box_start.p));
            self.impact_transform.q = nlerp(box_start.q, box_end.q, toi.fraction);
            self.box_impact = self.make_impact(world, self.falling_box_id, &toi);
        }

        if Self::settled_for_one_second(
            world,
            self.falling_box_id,
            &mut self.respawn_timer,
            time_step,
        ) {
            self.respawn_falling_box(world);
        }
    }

    /// C `RespawnFallingBox` (:755).
    fn respawn_falling_box(&mut self, world: &mut World) {
        body_set_transform(
            world,
            self.falling_box_id,
            self.falling_spawn,
            QUAT_IDENTITY,
        );
        body_set_linear_velocity(world, self.falling_box_id, VEC3_ZERO);
        body_set_angular_velocity(world, self.falling_box_id, self.falling_spin);
        body_set_awake(world, self.falling_box_id, true);
    }
}

/// C `GeometricMoverSample::Step` (:763): mover input + update, the base step, then the
/// time-of-impact sweeps; followed by the `Render` / `Step` draw calls.
pub(crate) fn step(state: &mut CharacterState, dt: f32, sub_steps: i32) {
    let CharacterState {
        world,
        bodies: _,
        hulls,
        render_capsules,
        debug_segs,
        debug_pts,
        debug_text,
        status,
        third_person,
        clip_velocity,
        state: scene_state,
        ..
    } = state;
    let SceneState::Mover(scene) = scene_state else {
        return;
    };
    scene.camera.third_person = *third_person;
    scene.mover.clip_velocity = *clip_velocity;

    let mover_start = scene.mover.position;
    let box_start = body_get_transform(world, scene.falling_box_id);
    let tree_start = body_get_transform(world, scene.tree_id);

    let input = get_mover_input(&scene.camera, &scene.keys);
    if input.jump {
        scene.mover.jump();
    }

    let mut speed_scale = 1.0f32;
    if input.walk {
        speed_scale = 0.25;
    } else if scene.mover.on_ground && input.sprint {
        speed_scale = 1.5;
    }
    scene.mover.update(
        world,
        dt,
        input.forward,
        input.right,
        input.throttle,
        speed_scale,
    );

    // The page pivots the third-person camera on the mover (C `m_camera->m_pivot =
    // m_mover.m_position` + `UpdateTransform`) from `character_follow_target`.
    scene.mover.draw(debug_segs, debug_pts);
    render_capsules.push(RenderCapsule {
        transform: WorldTransform {
            p: scene.mover.position,
            q: QUAT_IDENTITY,
        },
        capsule: scene.mover.capsule,
        style: capsule_style(colors::BLUE, 2),
    });

    // Sample::Step
    let did_step = dt > 0.0;
    if did_step {
        world.step(dt, sub_steps);
    }

    if did_step {
        scene.update_falling_box(world, mover_start, box_start, dt);
        if let Some(tree_id) = scene.update_falling_tree(world, mover_start, tree_start, dt) {
            if let Some(h) = hulls.first_mut() {
                h.body_id = tree_id;
            }
        }
    }

    // C Render: DrawAxes( { { 0, 0, 0.02 }, identity }, 2 ).
    let origin = Pos {
        x: 0.0,
        y: 0.0,
        z: 0.02f32 as _,
    };
    let basis = make_matrix_from_quat(QUAT_IDENTITY);
    for (axis, color) in [
        (basis.cx, 0xFF0000u32),
        (basis.cy, 0x00FF00),
        (basis.cz, 0x0000FF),
    ] {
        push_line(
            debug_segs,
            origin,
            offset_pos(origin, mul_sv(2.0, axis)),
            color,
        );
    }

    // The ghost box at first touch (hull channel entry 1).
    if let Some(ghost) = hulls.get_mut(1) {
        ghost.visible = scene.box_impact.valid;
        ghost.transform = scene.impact_transform;
    }

    draw_impact(
        debug_segs,
        debug_pts,
        debug_text,
        &scene.box_impact,
        colors::YELLOW,
    );
    draw_impact(
        debug_segs,
        debug_pts,
        debug_text,
        &scene.tree_impact,
        colors::CYAN,
    );

    let m = &scene.mover;
    *status = vec![
        m.position.x as f32,
        m.position.y as f32,
        m.position.z as f32,
        m.velocity.x,
        m.velocity.y,
        m.velocity.z,
        m.on_ground as i32 as f32,
        (m.on_ground && input.sprint && !input.walk) as i32 as f32,
    ];
}

/// C `DrawImpact` (:673): a 6 px point, an arrow along the normal, and the speed label.
fn draw_impact(
    segs: &mut Vec<f32>,
    pts: &mut Vec<f32>,
    text: &mut Vec<DebugText>,
    impact: &MoverImpact,
    color: u32,
) {
    if !impact.valid {
        return;
    }

    let tip = offset_pos(impact.point, 0.5 * impact.normal);

    pts.extend_from_slice(&[
        impact.point.x as f32,
        impact.point.y as f32,
        impact.point.z as f32,
        color as f32,
        6.0,
    ]);
    draw_arrow(segs, impact.point, tip, color);
    text.push(DebugText {
        p: tip,
        color,
        text: format!(" {:.1} m/s", impact.speed),
    });
}

/// C `DrawArrowEx` (gfx/draw.c:158) with `DEFAULT_ARROW_HEAD_FRAC = 0.15`.
fn draw_arrow(segs: &mut Vec<f32>, a: Pos, b: Pos, color: u32) {
    push_line(segs, a, b, color);
    let shaft = sub_pos(b, a);
    let shaft_len = length(shaft);
    if shaft_len < 1e-6 {
        return;
    }
    let dir = Vec3 {
        x: shaft.x / shaft_len,
        y: shaft.y / shaft_len,
        z: shaft.z / shaft_len,
    };
    let side = perp(dir);
    let head_len = shaft_len * 0.15;
    let back_from_tip = mul_sv(-head_len, dir);
    let side_step = mul_sv(head_len * 0.5, side);
    let tip1 = offset_pos(b, back_from_tip + side_step);
    let tip2 = offset_pos(b, back_from_tip - side_step);
    push_line(segs, b, tip1, color);
    push_line(segs, b, tip2, color);
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

#[cfg(test)]
mod tests;
