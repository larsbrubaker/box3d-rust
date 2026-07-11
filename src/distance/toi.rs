// Time of impact (b3TimeOfImpact) via local separating axes, from distance.c.
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use super::cast::{get_final_sweep_transform, get_sweep_transform};
use super::gjk::{get_point_support, shape_distance};
use super::types::{DistanceInput, ShapeProxy, SimplexCache, Sweep, ToiInput, ToiOutput, ToiState};
use crate::constants::linear_slop;
use crate::math_functions::{
    abs_float, add, cross, dot, inv_mul_transforms, inv_rotate_vector, length_squared, lerp,
    max_float, mul_add, mul_sv, neg, normalize, rotate_vector, sub, transform_point, Vec3,
    VEC3_ZERO,
};

#[derive(Clone, Copy, PartialEq, Eq)]
enum SeparationType {
    Unknown = 0,
    Vertices,
    Edges,
    FaceA,
    FaceB,
}

struct SeparationFunction<'a> {
    proxy_a: &'a ShapeProxy,
    proxy_b: &'a ShapeProxy,
    sweep_a: Sweep,
    sweep_b: Sweep,
    // These are associated with different bodies depending on the separation
    // function type. It could be two local vectors/points on the same body.
    witness1: Vec3,
    witness2: Vec3,
    kind: SeparationType,
}

fn unique_count(vertex_count: i32, vertices: [i32; 3]) -> i32 {
    debug_assert!((1..=3).contains(&vertex_count));

    match vertex_count {
        1 => 1,
        2 => {
            if vertices[0] != vertices[1] {
                2
            } else {
                1
            }
        }
        3 => {
            if vertices[0] != vertices[1]
                && vertices[0] != vertices[2]
                && vertices[1] != vertices[2]
            {
                // All different
                3
            } else if vertices[0] == vertices[1]
                && vertices[0] == vertices[2]
                && vertices[1] == vertices[2]
            {
                // All equal
                1
            } else {
                2
            }
        }
        _ => {
            debug_assert!(false, "Should never get here!");
            0
        }
    }
}

/// This checks if the cross product of two edges switches direction.
fn check_fast_edges(
    xf_a: crate::math_functions::Transform,
    local_edge_a: Vec3,
    xf_b: crate::math_functions::Transform,
    local_edge_b: Vec3,
    axis0: Vec3,
) -> bool {
    // By taking the local witness axes we make sure that we
    // get the correct orientations (e.g. if one axis was flipped)!
    let edge_a = rotate_vector(xf_a.q, local_edge_a);
    let edge_b = rotate_vector(xf_b.q, local_edge_b);
    let axis = cross(edge_a, edge_b);
    dot(axis, axis0) < 0.0
}

fn make_separation_function<'a>(
    cache: SimplexCache,
    proxy_a: &'a ShapeProxy,
    sweep_a: &Sweep,
    proxy_b: &'a ShapeProxy,
    sweep_b: &Sweep,
    world_normal: Vec3,
    t1: f32,
) -> SeparationFunction<'a> {
    debug_assert!((1..=3).contains(&(cache.count as i32)));
    debug_assert!(crate::math_functions::is_normalized(world_normal));

    let mut fcn = SeparationFunction {
        proxy_a,
        proxy_b,
        sweep_a: *sweep_a,
        sweep_b: *sweep_b,
        witness1: VEC3_ZERO,
        witness2: VEC3_ZERO,
        kind: SeparationType::Unknown,
    };

    let mut index_a = [
        cache.index_a[0] as i32,
        cache.index_a[1] as i32,
        cache.index_a[2] as i32,
    ];
    let mut index_b = [
        cache.index_b[0] as i32,
        cache.index_b[1] as i32,
        cache.index_b[2] as i32,
    ];

    let unique_count_a = unique_count(cache.count as i32, index_a);
    let unique_count_b = unique_count(cache.count as i32, index_b);

    let xf_a1 = get_sweep_transform(sweep_a, t1);
    let xf_b1 = get_sweep_transform(sweep_b, t1);

    let q_a = xf_a1.q;
    let q_b = xf_b1.q;

    // Minimize round-off
    let delta_p = sub(xf_b1.p, xf_a1.p);

    match cache.count {
        1 => {
            // Witness is the world space direction
            fcn.kind = SeparationType::Vertices;
            fcn.witness1 = world_normal;
        }
        2 => {
            if unique_count_a == 2 && unique_count_b == 2 {
                // Edge/Edge
                let v_a1 = proxy_a.points[index_a[0] as usize];
                let mut local_edge_a = sub(proxy_a.points[index_a[1] as usize], v_a1);
                local_edge_a = normalize(local_edge_a);
                let edge_a = rotate_vector(q_a, local_edge_a);

                let v_b1 = proxy_b.points[index_b[0] as usize];
                let mut local_edge_b = sub(proxy_b.points[index_b[1] as usize], v_b1);
                local_edge_b = normalize(local_edge_b);
                let edge_b = rotate_vector(q_b, local_edge_b);

                let mut axis = cross(edge_a, edge_b);
                let length_sq = length_squared(axis);

                // Skip near parallel edges: |e1 x e1| = sin(alpha) * |e1| * |e2|
                let k_tolerance_squared = 0.05 * 0.05;
                if length_sq < k_tolerance_squared {
                    // The axis is not safe to normalize so we use a world axis instead!
                    fcn.kind = SeparationType::Vertices;
                    fcn.witness1 = world_normal;
                } else {
                    let delta = add(
                        sub(rotate_vector(q_b, v_b1), rotate_vector(q_a, v_a1)),
                        delta_p,
                    );
                    if dot(delta, axis) < 0.0 {
                        // Make axis point from A to B
                        axis = neg(axis);
                        local_edge_b = neg(local_edge_b);
                    }

                    // Check for possible sign flip in edge/edge cross product
                    let xf_a2 = get_final_sweep_transform(sweep_a);
                    let xf_b2 = get_final_sweep_transform(sweep_b);
                    let fast_edges =
                        check_fast_edges(xf_a2, local_edge_a, xf_b2, local_edge_b, axis);
                    if fast_edges {
                        // Not safe to use local edges, fall back to initial world space axis instead
                        fcn.kind = SeparationType::Vertices;
                        fcn.witness1 = normalize(axis);
                    } else {
                        // Edge cross product is safe. This converges faster than a fixed axis.
                        fcn.kind = SeparationType::Edges;
                        fcn.witness1 = local_edge_a;
                        fcn.witness2 = local_edge_b;
                    }
                }
            } else {
                debug_assert!(crate::math_functions::is_normalized(world_normal));

                // Vertex versus edge, use world axis witness
                fcn.kind = SeparationType::Vertices;
                fcn.witness1 = world_normal;
            }
        }
        3 => {
            if unique_count_a == 3 {
                let v_a1 = proxy_a.points[index_a[0] as usize];
                let v_a2 = proxy_a.points[index_a[1] as usize];
                let v_a3 = proxy_a.points[index_a[2] as usize];
                let mut local_axis_a = cross(sub(v_a2, v_a1), sub(v_a3, v_a1));
                local_axis_a = normalize(local_axis_a);
                let axis_a = rotate_vector(q_a, local_axis_a);

                let local_point_a = mul_sv(1.0 / 3.0, add(add(v_a1, v_a2), v_a3));
                let local_point_b = proxy_b.points[index_b[0] as usize];
                let delta = add(
                    sub(
                        rotate_vector(q_b, local_point_b),
                        rotate_vector(q_a, local_point_a),
                    ),
                    delta_p,
                );

                if dot(delta, axis_a) < 0.0 {
                    // Make axis point from A to B
                    local_axis_a = neg(local_axis_a);
                }

                // Witness is the local plane of faceA
                fcn.kind = SeparationType::FaceA;
                fcn.witness1 = local_axis_a;
                fcn.witness2 = local_point_a;
            } else if unique_count_b == 3 {
                let v_b1 = proxy_b.points[index_b[0] as usize];
                let v_b2 = proxy_b.points[index_b[1] as usize];
                let v_b3 = proxy_b.points[index_b[2] as usize];
                let mut local_axis_b = cross(sub(v_b2, v_b1), sub(v_b3, v_b1));
                local_axis_b = normalize(local_axis_b);
                let axis_b = rotate_vector(q_b, local_axis_b);

                let local_point_a = proxy_a.points[index_a[0] as usize];
                let local_point_b = mul_sv(1.0 / 3.0, add(add(v_b1, v_b2), v_b3));
                let delta = sub(
                    sub(
                        rotate_vector(q_a, local_point_a),
                        rotate_vector(q_b, local_point_b),
                    ),
                    delta_p,
                );

                if dot(delta, axis_b) < 0.0 {
                    // Make axis point from B to A
                    local_axis_b = neg(local_axis_b);
                }

                // Witness is the local plane of faceB
                fcn.kind = SeparationType::FaceB;
                fcn.witness1 = local_axis_b;
                fcn.witness2 = local_point_b;
            } else {
                debug_assert!(unique_count_a == 2 && unique_count_b == 2);

                if index_a[0] == index_a[1] {
                    // Make first two indices are unique
                    index_a[1] = index_a[2];
                    debug_assert!(index_a[0] != index_a[1]);
                }

                let v_a1 = proxy_a.points[index_a[0] as usize];
                let v_a2 = proxy_a.points[index_a[1] as usize];
                let local_edge_a = normalize(sub(v_a2, v_a1));
                let edge_a = rotate_vector(q_a, local_edge_a);

                if index_b[0] == index_b[1] {
                    // Make first two indices are unique
                    index_b[1] = index_b[2];
                    debug_assert!(index_b[0] != index_b[1]);
                }

                let v_b1 = proxy_b.points[index_b[0] as usize];
                let v_b2 = proxy_b.points[index_b[1] as usize];
                let mut local_edge_b = normalize(sub(v_b2, v_b1));
                let edge_b = rotate_vector(q_b, local_edge_b);

                let mut axis = cross(edge_a, edge_b);
                let length_sq = length_squared(axis);

                // Skip near parallel edges: |e1 x e1| = sin(alpha) * |e1| * |e2|
                let k_tolerance_squared = 0.005 * 0.005;
                if length_sq < k_tolerance_squared {
                    // The axis is not safe to normalize so we use a world axis instead!
                    fcn.kind = SeparationType::Vertices;
                    fcn.witness1 = world_normal;
                } else {
                    let delta = add(
                        sub(rotate_vector(q_b, v_b1), rotate_vector(q_a, v_a1)),
                        delta_p,
                    );
                    if dot(delta, axis) < 0.0 {
                        // Make axis point from A to B
                        axis = neg(axis);
                        local_edge_b = neg(local_edge_b);
                    }

                    // Check for possible sign flip in edge/edge cross product
                    let xf_a2 = get_final_sweep_transform(sweep_a);
                    let xf_b2 = get_final_sweep_transform(sweep_b);
                    let fast_edges =
                        check_fast_edges(xf_a2, local_edge_a, xf_b2, local_edge_b, axis);
                    if fast_edges {
                        // Not safe to use local edges, fall back to initial world space axis instead
                        fcn.kind = SeparationType::Vertices;
                        fcn.witness1 = normalize(axis);
                    } else {
                        // Edge cross product is safe. This converges faster than a fixed axis.
                        fcn.kind = SeparationType::Edges;
                        fcn.witness1 = local_edge_a;
                        fcn.witness2 = local_edge_b;
                    }
                }
            }
        }
        _ => {
            debug_assert!(false, "Should never get here!");
        }
    }

    fcn
}

/// Returns (separation, index_a, index_b). (b3FindMinSeparation)
fn find_min_separation(fcn: &SeparationFunction, t: f32) -> (f32, i32, i32) {
    let xf_a = get_sweep_transform(&fcn.sweep_a, t);
    let xf_b = get_sweep_transform(&fcn.sweep_b, t);

    match fcn.kind {
        SeparationType::Vertices => {
            let axis = fcn.witness1;

            let local_axis_a = inv_rotate_vector(xf_a.q, axis);
            let local_axis_b = inv_rotate_vector(xf_b.q, neg(axis));

            let index_a = get_point_support(
                &fcn.proxy_a.points[..fcn.proxy_a.count as usize],
                local_axis_a,
            );
            let index_b = get_point_support(
                &fcn.proxy_b.points[..fcn.proxy_b.count as usize],
                local_axis_b,
            );

            let delta_p = sub(xf_b.p, xf_a.p);
            let local_point_a = fcn.proxy_a.points[index_a as usize];
            let local_point_b = fcn.proxy_b.points[index_b as usize];
            let delta = add(
                sub(
                    rotate_vector(xf_b.q, local_point_b),
                    rotate_vector(xf_a.q, local_point_a),
                ),
                delta_p,
            );
            (dot(delta, axis), index_a, index_b)
        }

        SeparationType::Edges => {
            let edge_a = rotate_vector(xf_a.q, fcn.witness1);
            let edge_b = rotate_vector(xf_b.q, fcn.witness2);
            let mut axis = cross(edge_a, edge_b);
            debug_assert!(axis.x != 0.0 || axis.y != 0.0 || axis.z != 0.0);
            axis = normalize(axis);

            let axis_a = inv_rotate_vector(xf_a.q, axis);
            let index_a =
                get_point_support(&fcn.proxy_a.points[..fcn.proxy_a.count as usize], axis_a);

            let axis_b = inv_rotate_vector(xf_b.q, axis);
            let index_b = get_point_support(
                &fcn.proxy_b.points[..fcn.proxy_b.count as usize],
                neg(axis_b),
            );

            let delta_p = sub(xf_b.p, xf_a.p);
            let local_point_a = fcn.proxy_a.points[index_a as usize];
            let local_point_b = fcn.proxy_b.points[index_b as usize];
            let delta = add(
                sub(
                    rotate_vector(xf_b.q, local_point_b),
                    rotate_vector(xf_a.q, local_point_a),
                ),
                delta_p,
            );

            (dot(delta, axis), index_a, index_b)
        }

        SeparationType::FaceA => {
            let normal = rotate_vector(xf_a.q, fcn.witness1);
            let index_a = -1;
            let point_a = transform_point(xf_a, fcn.witness2);

            let axis_b = inv_rotate_vector(xf_b.q, normal);
            let index_b = get_point_support(
                &fcn.proxy_b.points[..fcn.proxy_b.count as usize],
                neg(axis_b),
            );
            let point_b = transform_point(xf_b, fcn.proxy_b.points[index_b as usize]);

            (dot(sub(point_b, point_a), normal), index_a, index_b)
        }

        SeparationType::FaceB => {
            let normal = rotate_vector(xf_b.q, fcn.witness1);

            let axis_a = inv_rotate_vector(xf_a.q, normal);
            let index_a = get_point_support(
                &fcn.proxy_a.points[..fcn.proxy_a.count as usize],
                neg(axis_a),
            );
            let point_a = transform_point(xf_a, fcn.proxy_a.points[index_a as usize]);

            let index_b = -1;
            let point_b = transform_point(xf_b, fcn.witness2);

            (dot(sub(point_a, point_b), normal), index_a, index_b)
        }

        SeparationType::Unknown => {
            debug_assert!(false, "Should never get here!");
            (0.0, 0, 0)
        }
    }
}

/// (b3EvaluateSeparation)
fn evaluate_separation(fcn: &SeparationFunction, index1: i32, index2: i32, beta: f32) -> f32 {
    let transform1 = get_sweep_transform(&fcn.sweep_a, beta);
    let transform2 = get_sweep_transform(&fcn.sweep_b, beta);

    match fcn.kind {
        SeparationType::Vertices => {
            let axis = fcn.witness1;

            let point1 = transform_point(transform1, fcn.proxy_a.points[index1 as usize]);
            let point2 = transform_point(transform2, fcn.proxy_b.points[index2 as usize]);

            dot(sub(point2, point1), axis)
        }

        SeparationType::Edges => {
            let edge1 = rotate_vector(transform1.q, fcn.witness1);
            let edge2 = rotate_vector(transform2.q, fcn.witness2);
            let mut axis = cross(edge1, edge2);
            axis = normalize(axis);

            let point1 = transform_point(transform1, fcn.proxy_a.points[index1 as usize]);
            let point2 = transform_point(transform2, fcn.proxy_b.points[index2 as usize]);

            dot(sub(point2, point1), axis)
        }

        SeparationType::FaceA => {
            let axis = rotate_vector(transform1.q, fcn.witness1);

            let point1 = transform_point(transform1, fcn.witness2);
            let point2 = transform_point(transform2, fcn.proxy_b.points[index2 as usize]);

            dot(sub(point2, point1), axis)
        }

        SeparationType::FaceB => {
            let axis = rotate_vector(transform2.q, fcn.witness1);

            let point1 = transform_point(transform1, fcn.proxy_a.points[index1 as usize]);
            let point2 = transform_point(transform2, fcn.witness2);

            dot(sub(point1, point2), axis)
        }

        SeparationType::Unknown => {
            debug_assert!(false, "Should never get here!");
            0.0
        }
    }
}

fn force_fixed_axis(fcn: &mut SeparationFunction, beta: f32) {
    debug_assert!(fcn.kind == SeparationType::Edges);

    let transform1 = get_sweep_transform(&fcn.sweep_a, beta);
    let transform2 = get_sweep_transform(&fcn.sweep_b, beta);

    let edge1 = rotate_vector(transform1.q, fcn.witness1);
    let edge2 = rotate_vector(transform2.q, fcn.witness2);
    let mut axis = cross(edge1, edge2);
    axis = normalize(axis);

    fcn.kind = SeparationType::Vertices;
    fcn.witness1 = axis;
    fcn.witness2 = VEC3_ZERO;
}

/// Compute the upper bound on time before two shapes penetrate. Time is
/// represented as a fraction between [0, max_fraction]. This uses a swept
/// separating axis and may miss some intermediate, non-tunneling collisions.
/// If you change the time interval, you should call this function again.
/// (b3TimeOfImpact)
pub fn time_of_impact(input: &ToiInput) -> ToiOutput {
    let mut output = ToiOutput {
        // Set these to invalid values so they can be validated on exit
        state: ToiState::Unknown,
        fraction: -1.0,
        ..Default::default()
    };

    let mut sweep_a = input.sweep_a;
    let mut sweep_b = input.sweep_b;

    // Shift to origin
    let origin = sweep_a.c1;
    sweep_a.c1 = VEC3_ZERO;
    sweep_a.c2 = sub(sweep_a.c2, origin);
    sweep_b.c1 = sub(sweep_b.c1, origin);
    sweep_b.c2 = sub(sweep_b.c2, origin);

    let proxy_a = input.proxy_a;
    let proxy_b = input.proxy_b;

    let max_push_back_iterations = proxy_a.count + proxy_b.count;
    let t_max = input.max_fraction;

    // Setup target distance and tolerance
    let linear_slop = linear_slop();
    let total_radius = proxy_a.radius + proxy_b.radius;
    let target = max_float(linear_slop, total_radius - linear_slop);
    let tolerance = 0.25 * linear_slop;
    debug_assert!(target > tolerance);

    let mut t1 = 0.0f32;
    let max_iterations = 25;
    let mut distance_iterations = 0;

    // Prepare input for distance query.
    let mut cache = SimplexCache::default();
    let mut distance_input = DistanceInput {
        proxy_a,
        proxy_b,
        transform: crate::math_functions::TRANSFORM_IDENTITY,
        use_radii: false,
    };

    // The outer loop progressively attempts to compute new separating axes.
    // This loop terminates when an axis is repeated (no progress is made).
    loop {
        // Get the distance between shapes. We can also use the results to get a separating axis.
        let xf_a = get_sweep_transform(&sweep_a, t1);
        let xf_b = get_sweep_transform(&sweep_b, t1);
        distance_input.transform = inv_mul_transforms(xf_a, xf_b);
        let distance_output = shape_distance(&distance_input, &mut cache, None);
        output.distance = distance_output.distance;

        // The distance query runs in frame A, project the witness data back to the shifted world
        let world_normal = rotate_vector(xf_a.q, distance_output.normal);
        let world_point_a = transform_point(xf_a, distance_output.point_a);
        let world_point_b = transform_point(xf_a, distance_output.point_b);

        output.distance_iterations += 1;
        distance_iterations += 1;

        // If the shapes are overlapped, we give up on continuous collision.
        if distance_output.distance <= 0.0 {
            output.state = ToiState::Overlapped;
            output.fraction = 0.0;
            break;
        }

        if distance_output.distance <= target + tolerance {
            // Success!
            output.state = ToiState::Hit;

            // Averaged hit point
            let p_a = mul_add(world_point_a, proxy_a.radius, world_normal);
            let p_b = mul_add(world_point_b, -proxy_b.radius, world_normal);
            output.point = lerp(p_a, p_b, 0.5);
            output.point = add(output.point, origin);
            output.normal = world_normal;
            output.fraction = t1;
            break;
        }

        if distance_iterations == max_iterations {
            // Progress too slow. This can happen when a capsule rotates around a
            // triangle vertex.
            output.state = ToiState::Failed;
            output.fraction = t1;

            // Averaged hit point
            let p_a = mul_add(world_point_a, input.proxy_a.radius, world_normal);
            let p_b = mul_add(world_point_b, -input.proxy_b.radius, world_normal);
            output.point = lerp(p_a, p_b, 0.5);
            output.point = add(output.point, origin);
            output.normal = world_normal;
            break;
        }

        // Initialize the separating axis.
        let mut function = make_separation_function(
            cache,
            &proxy_a,
            &sweep_a,
            &proxy_b,
            &sweep_b,
            world_normal,
            t1,
        );

        // Compute the TOI on the separating axis. We do this by successively resolving the deepest point.
        let mut done = false;
        let mut t2 = t_max;
        let mut push_back_iterations = 0;
        loop {
            let (mut s2, index_a, index_b) = find_min_separation(&function, t2);

            // Is the final configuration separated?
            if s2 - target > tolerance {
                // Success!
                output.state = ToiState::Separated;
                output.fraction = input.max_fraction;
                done = true;
                break;
            }

            // Has the separation reached tolerance?
            if s2 >= target - tolerance {
                // Advance the sweeps
                t1 = t2;
                break;
            }

            // Compute the initial separation of the witness points
            let mut s1 = evaluate_separation(&function, index_a, index_b, t1);

            // Check for overlap. This might happen if the root finder runs out of iterations.
            if s1 < target - tolerance {
                // Failed!
                debug_assert!(false);
                output.state = ToiState::Failed;
                output.fraction = t1;
                done = true;
                break;
            }

            // Has the separation reached tolerance?
            if s1 <= target + tolerance {
                // Success! t1 should hold the TOI (could be 0.0)
                output.state = ToiState::Hit;
                output.fraction = t1;
                done = true;
                break;
            }

            // Compute 1D root of: f(x) - target = 0
            let mut root_iteration_count = 0;
            let max_root_iterations = 50;
            let mut a1 = t1;
            let mut a2 = t2;
            loop {
                // Use a mix of false position and bisection.
                let t = if root_iteration_count & 1 != 0 {
                    // False position to improve convergence.
                    a1 + (target - s1) * (a2 - a1) / (s2 - s1)
                } else {
                    // Bisection to guarantee progress.
                    0.5 * (a1 + a2)
                };

                output.root_iterations += 1;
                root_iteration_count += 1;

                let s = evaluate_separation(&function, index_a, index_b, t);

                // Has the separation reached tolerance?
                if abs_float(s - target) <= tolerance {
                    // t2 holds a tentative value for t1
                    t2 = t;
                    break;
                }

                // Ensure we continue to bracket the root.
                if s > target {
                    a1 = t;
                    s1 = s;
                } else {
                    a2 = t;
                    s2 = s;
                }

                if root_iteration_count == max_root_iterations {
                    debug_assert!(false);
                    break;
                }
            }

            // Restart the inner loop if we have a failing edge case.
            if root_iteration_count == max_root_iterations - 1
                && function.kind == SeparationType::Edges
            {
                debug_assert!(false);

                // C resets rootIterationCount here; the next push-back iteration
                // creates a fresh counter, so the assignment is a no-op.
                t2 = input.max_fraction;
                force_fixed_axis(&mut function, t1);
                debug_assert!(function.kind != SeparationType::Edges);
            }

            output.push_back_iterations += 1;
            push_back_iterations += 1;

            if push_back_iterations == max_push_back_iterations {
                break;
            }
        }

        if done {
            // Averaged hit point
            let p_a = mul_add(world_point_a, input.proxy_a.radius, world_normal);
            let p_b = mul_add(world_point_b, -input.proxy_b.radius, world_normal);
            output.point = lerp(p_a, p_b, 0.5);
            output.point = add(output.point, origin);
            output.normal = world_normal;
            break;
        }
    }

    // It is expected that the state and fraction are set before reaching this
    debug_assert!(output.state != ToiState::Unknown);
    debug_assert!(output.fraction >= 0.0);

    output
}
