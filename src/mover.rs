//! Character mover plane solver from box3d-cpp-reference/src/mover.c.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::constants::linear_slop;
use crate::geometry::{CollisionPlane, PlaneSolverResult};
use crate::math_functions::{
    abs_float, clamp_float, dot, min_float, mul_add, mul_sub, plane_separation, Vec3,
};

/// Iteratively solve a character mover against collected collision planes.
/// (b3SolvePlanes)
pub fn solve_planes(target_delta: Vec3, planes: &mut [CollisionPlane]) -> PlaneSolverResult {
    for plane in planes.iter_mut() {
        plane.push = 0.0;
    }

    let mut delta = target_delta;
    let tolerance = linear_slop();

    let mut iteration = 0;
    while iteration < 20 {
        let mut total_push = 0.0;
        for plane in planes.iter_mut() {
            // Add slop to prevent jitter
            let separation = plane_separation(plane.plane, delta) + linear_slop();

            let mut push = -separation;

            // Clamp accumulated push
            let accumulated_push = plane.push;
            plane.push = clamp_float(plane.push + push, 0.0, plane.push_limit);
            push = plane.push - accumulated_push;
            delta = mul_add(delta, push, plane.plane.normal);

            // Track maximum push for convergence
            total_push += abs_float(push);
        }

        if total_push < tolerance {
            break;
        }

        iteration += 1;
    }

    PlaneSolverResult {
        delta,
        iteration_count: iteration,
    }
}

/// Clip a velocity (or other) vector against solved collision planes.
/// (b3ClipVector)
pub fn clip_vector(vector: Vec3, planes: &[CollisionPlane]) -> Vec3 {
    let mut v = vector;

    for plane in planes {
        if plane.push == 0.0 || !plane.clip_velocity {
            continue;
        }

        v = mul_sub(
            v,
            min_float(0.0, dot(v, plane.plane.normal)),
            plane.plane.normal,
        );
    }

    v
}
