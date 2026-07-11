// b3World_Step from physics_world.c, split out of world/mod.rs to keep
// modules focused.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::world::{Profile, World};

impl World {
    /// Advance the world simulation by one time step. (b3World_Step)
    pub fn step(&mut self, time_step: f32, sub_step_count: i32) {
        use crate::broad_phase::update_broad_phase_pairs;
        use crate::math_functions::{is_valid_float, max_int, min_float};
        use crate::solver::{make_soft, solve, StepContext};

        debug_assert!(is_valid_float(time_step) && time_step >= 0.0);
        debug_assert!(!self.locked);
        if self.locked {
            return;
        }

        self.locked = true;

        self.body_move_events.clear();
        self.sensor_begin_events.clear();
        self.contact_begin_events.clear();
        self.contact_hit_events.clear();
        self.joint_events.clear();
        self.profile = Profile::default();

        {
            let c = &mut self.max_capacity;
            c.static_shape_count = max_int(
                c.static_shape_count,
                self.broad_phase.trees[crate::types::BodyType::Static as usize].proxy_count(),
            );
            c.dynamic_shape_count = max_int(
                c.dynamic_shape_count,
                self.broad_phase.trees[crate::types::BodyType::Dynamic as usize].proxy_count(),
            );
            let static_body_count =
                self.solver_sets[crate::solver_set::STATIC_SET as usize]
                    .body_sims
                    .len() as i32;
            c.static_body_count = max_int(c.static_body_count, static_body_count);
            let total_body_count = self.body_id_pool.id_count();
            c.dynamic_body_count =
                max_int(c.dynamic_body_count, total_body_count - static_body_count);
            c.contact_count = max_int(c.contact_count, self.contact_id_pool.id_count());
        }

        update_broad_phase_pairs(self);

        let sub_steps = max_int(1, sub_step_count);
        let mut context = StepContext {
            dt: time_step,
            sub_step_count: sub_steps,
            ..StepContext::default()
        };

        if time_step > 0.0 {
            context.inv_dt = 1.0 / time_step;
            context.h = time_step / sub_steps as f32;
            context.inv_h = sub_steps as f32 * context.inv_dt;
        } else {
            context.inv_dt = 0.0;
            context.h = 0.0;
            context.inv_h = 0.0;
        }

        self.inv_dt = context.inv_dt;
        self.inv_h = context.inv_h;

        // Hertz values get reduced for large time steps
        let contact_hertz = min_float(self.contact_hertz, 0.125 * context.inv_h);
        context.contact_softness = make_soft(contact_hertz, self.contact_damping_ratio, context.h);
        context.static_softness =
            make_soft(2.0 * contact_hertz, 0.5 * self.contact_damping_ratio, context.h);
        context.restitution_threshold = self.restitution_threshold;
        context.max_linear_velocity = self.max_linear_speed;
        context.contact_speed = self.contact_speed;
        context.enable_warm_starting = self.enable_warm_starting;

        crate::contact::collide(self, time_step);

        if time_step > 0.0 {
            solve(self, &context);
        }

        self.step_index = self.step_index.wrapping_add(1);
        self.validate_solver_sets();
        self.locked = false;
    }
}
