//! The pogo half of C `DynamicMover::Update` and `DynamicMover::Draw`
//! (dynamic_mover.cpp:474-584), split from [`super`] for file length.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

use super::DynamicMover;
use crate::character_demo::colors;
use box3d_rust::body::{body_get_local_point, body_get_mass};
use box3d_rust::id::NULL_JOINT_ID;
use box3d_rust::joint::{
    create_pogo_joint, destroy_joint, joint_is_valid, joint_set_local_frame_a,
    joint_set_local_frame_b, pogo_joint_get_current_speed, pogo_joint_get_impulse,
    pogo_joint_set_max_compression_force, pogo_joint_set_max_tension_force, pogo_joint_set_normal,
    pogo_joint_set_spring_damping_ratio, pogo_joint_set_spring_hertz,
};
use box3d_rust::math_functions::{length, offset_pos, Pos, Transform, Vec3, QUAT_IDENTITY};
use box3d_rust::types::default_pogo_joint_def;
use box3d_rust::world::{world_get_gravity, World};

impl DynamicMover {
    /// The `m_castResult.hit && b3IsNormalized(normal)` branch of C `DynamicMover::Update`
    /// (dynamic_mover.cpp:474-562): create, recreate, or retune the pogo joint. Split out
    /// only to keep `update` readable; the statements are in C order.
    pub(super) fn update_pogo(&mut self, world: &mut World, ray_length: f32, pogo_offset: Vec3) {
        self.pogo_length = self.cast_result.fraction * ray_length;

        let mover_mass = body_get_mass(world, self.mover_id);
        let gravity = length(world_get_gravity(world));
        let mover_weight = self.gravity_scale * gravity * mover_mass;

        let pogo_joint_valid = joint_is_valid(world, self.pogo_joint_id);

        // Create or recreate the pogo joint as needed.
        if self.cast_result.body_id != self.surface_body_id || !pogo_joint_valid {
            if pogo_joint_valid {
                self.pogo_impulse = pogo_joint_get_impulse(world, self.pogo_joint_id);
                self.pogo_velocity = pogo_joint_get_current_speed(world, self.pogo_joint_id);

                destroy_joint(world, self.pogo_joint_id, false);
                self.pogo_joint_id = NULL_JOINT_ID;
            }

            self.surface_body_id = self.cast_result.body_id;

            let mut pogo_def = default_pogo_joint_def();
            pogo_def.base.local_frame_a.p =
                body_get_local_point(world, self.surface_body_id, self.cast_result.point);
            pogo_def.base.local_frame_b.p = self.capsule.center1 + pogo_offset;
            pogo_def.normal = self.cast_result.normal;
            pogo_def.base.body_id_a = self.surface_body_id;
            pogo_def.base.body_id_b = self.mover_id;
            pogo_def.base.collide_connected = true;
            pogo_def.rest_length = self.pogo_rest_length;
            pogo_def.hertz = self.pogo_hertz;
            pogo_def.damping_ratio = self.pogo_damping_ratio;
            pogo_def.max_compression_force = self.pogo_compression_scale * mover_weight;

            // Warm start from the joint that was just destroyed
            pogo_def.impulse = self.pogo_impulse;
            pogo_def.velocity = self.pogo_velocity;

            if self.jumping {
                // Don't allow the pogo to pull down
                pogo_def.max_tension_force = 0.0;
            } else {
                // The pogo can pull down at a multiple of the gravity force.
                pogo_def.max_tension_force = self.pogo_tension_scale * mover_weight;
            }

            self.pogo_joint_id = create_pogo_joint(world, &pogo_def);
        } else {
            // Preserve the joint, improving performance.

            let local_frame_a = Transform {
                p: body_get_local_point(world, self.surface_body_id, self.cast_result.point),
                q: QUAT_IDENTITY,
            };
            joint_set_local_frame_a(world, self.pogo_joint_id, local_frame_a);

            let local_frame_b = Transform {
                p: self.capsule.center1 + pogo_offset,
                q: QUAT_IDENTITY,
            };
            joint_set_local_frame_b(world, self.pogo_joint_id, local_frame_b);

            pogo_joint_set_normal(world, self.pogo_joint_id, self.cast_result.normal);

            if self.jumping {
                // Don't allow the pogo to pull down
                pogo_joint_set_max_tension_force(world, self.pogo_joint_id, 0.0);
            } else {
                // The pogo can pull down at a multiple of the gravity force.
                pogo_joint_set_max_tension_force(
                    world,
                    self.pogo_joint_id,
                    self.pogo_tension_scale * mover_weight,
                );
            }

            pogo_joint_set_max_compression_force(
                world,
                self.pogo_joint_id,
                self.pogo_compression_scale * mover_weight,
            );

            pogo_joint_set_spring_hertz(world, self.pogo_joint_id, self.pogo_hertz);
            pogo_joint_set_spring_damping_ratio(world, self.pogo_joint_id, self.pogo_damping_ratio);
        }
    }

    /// C `DynamicMover::Draw` (dynamic_mover.cpp:567): gray misses, the selected probe
    /// green, the other hits yellow, each hit capped with a 6 px point. Appends
    /// `[x0,y0,z0,x1,y1,z1,color]` segments and `[x,y,z,color,size]` points.
    pub(crate) fn draw(&self, segs: &mut Vec<f32>, pts: &mut Vec<f32>) {
        for i in 0..self.probe_count {
            let probe = &self.probes[i];
            let end = offset_pos(probe.origin, probe.fraction * probe.translation);

            if !probe.hit {
                push_line(segs, probe.origin, end, colors::GRAY);
                continue;
            }

            let color = if i == self.selected_probe {
                colors::GREEN
            } else {
                colors::YELLOW
            };
            push_line(segs, probe.origin, end, color);
            pts.extend_from_slice(&[end.x as f32, end.y as f32, end.z as f32, color as f32, 6.0]);
        }
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
