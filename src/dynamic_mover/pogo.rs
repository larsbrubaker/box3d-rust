//! The pogo half of C `DynamicMover::Update` (dynamic_mover.cpp:474-562), split from
//! [`super`] for file length.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::DynamicMover;
use crate::body::{body_get_local_point, body_get_mass};
use crate::id::NULL_JOINT_ID;
use crate::joint::{
    create_pogo_joint, destroy_joint, joint_is_valid, joint_set_local_frame_a,
    joint_set_local_frame_b, pogo_joint_get_current_speed, pogo_joint_get_impulse,
    pogo_joint_set_max_compression_force, pogo_joint_set_max_tension_force, pogo_joint_set_normal,
    pogo_joint_set_spring_damping_ratio, pogo_joint_set_spring_hertz,
};
use crate::math_functions::{length, Transform, Vec3, QUAT_IDENTITY};
use crate::types::default_pogo_joint_def;
use crate::world::{world_get_gravity, World};

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
}
