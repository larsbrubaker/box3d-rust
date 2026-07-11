// Linear shape cast and sweep evaluation from distance.c.
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use super::gjk::shape_distance;
use super::types::{CastOutput, DistanceInput, ShapeCastPairInput, SimplexCache, Sweep};
use crate::constants::linear_slop;
use crate::math_functions::{
    dot, is_normalized, lerp, max_float, mul_add, nlerp, rotate_vector, sub, Transform, VEC3_ZERO,
};

/// Evaluate the transform sweep at a specific time. (b3GetSweepTransform)
pub fn get_sweep_transform(sweep: &Sweep, time: f32) -> Transform {
    let mut transform = Transform {
        q: nlerp(sweep.q1, sweep.q2, time),
        p: VEC3_ZERO,
    };
    transform.p = sub(
        lerp(sweep.c1, sweep.c2, time),
        rotate_vector(transform.q, sweep.local_center),
    );
    transform
}

/// Final sweep transform at t = 1 without nlerp. (b3GetFinalSweepTransform)
pub(crate) fn get_final_sweep_transform(sweep: &Sweep) -> Transform {
    let mut transform = Transform {
        q: sweep.q2,
        p: VEC3_ZERO,
    };
    transform.p = sub(sweep.c2, rotate_vector(transform.q, sweep.local_center));
    transform
}

/// Perform a linear shape cast of shape B moving and shape A fixed. Determines
/// the hit point, normal, and translation fraction. The query runs in frame A,
/// so the hit point and normal are returned in frame A. Initially touching
/// shapes are a miss unless `can_encroach` allows it. (b3ShapeCast)
///
/// Shape cast using conservative advancement.
pub fn shape_cast(input: &ShapeCastPairInput) -> CastOutput {
    // Compute tolerance
    let linear_slop = linear_slop();
    let total_radius = input.proxy_a.radius + input.proxy_b.radius;
    let mut target = max_float(linear_slop, total_radius - linear_slop);
    let tolerance = 0.25 * linear_slop;

    debug_assert!(target > tolerance);

    // Prepare input for distance query
    let mut cache = SimplexCache::default();

    let mut alpha = 0.0;

    let mut distance_input = DistanceInput {
        proxy_a: input.proxy_a,
        proxy_b: input.proxy_b,
        // The whole cast runs in frame A. Advance the relative pose of B in
        // float each iteration, which keeps the math near the local origin and
        // avoids re-relativizing world poses.
        transform: input.transform,
        use_radii: false,
    };

    let delta2 = input.translation_b;
    let mut output = CastOutput::default();

    let max_iterations = 20;

    for iteration in 0..max_iterations {
        output.iterations += 1;

        let distance_output = shape_distance(&distance_input, &mut cache, None);

        if distance_output.distance < target + tolerance {
            if iteration == 0 {
                if input.can_encroach && distance_output.distance > 2.0 * linear_slop {
                    target = distance_output.distance - linear_slop;
                } else {
                    // Initial overlap
                    output.hit = true;

                    // Compute a common point
                    let c1 = mul_add(
                        distance_output.point_a,
                        input.proxy_a.radius,
                        distance_output.normal,
                    );
                    let c2 = mul_add(
                        distance_output.point_b,
                        -input.proxy_b.radius,
                        distance_output.normal,
                    );
                    output.point = lerp(c1, c2, 0.5);
                    return output;
                }
            } else {
                // Logging for bad input data (C calls b3Log); skip logs, keep early return.
                if distance_output.distance > 0.0 && !is_normalized(distance_output.normal) {
                    // Numerical problem. Likely extreme input.
                    return output;
                }

                // Hitting this assert implies that the algorithm brought the shapes too close.
                // debug_assert!(distance_output.distance > 0.0 && is_normalized(distance_output.normal));

                output.fraction = alpha;
                output.point = mul_add(
                    distance_output.point_a,
                    input.proxy_a.radius,
                    distance_output.normal,
                );
                output.normal = distance_output.normal;
                output.hit = true;
                return output;
            }
        }

        debug_assert!(distance_output.distance > 0.0);
        debug_assert!(is_normalized(distance_output.normal));

        // Check if shapes are approaching each other
        let denominator = dot(delta2, distance_output.normal);
        if denominator >= 0.0 {
            // Miss
            return output;
        }

        // Advance sweep
        alpha += (target - distance_output.distance) / denominator;
        if alpha >= input.max_fraction {
            // Success!
            return output;
        }

        distance_input.transform.p = mul_add(input.transform.p, alpha, delta2);
    }

    // Failure!
    output
}
