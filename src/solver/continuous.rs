//! Continuous collision (CCD) from solver.c: query callback + solve pass.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::aabb::farthest_point_on_aabb;
use crate::body::{body_flags, should_bodies_collide, BodySim};
use crate::core::NULL_INDEX;
use crate::distance::Sweep;
use crate::dynamic_tree::DEFAULT_MASK_BITS;
use crate::geometry::ShapeType;
use crate::id::ShapeId;
use crate::math_functions::{
    aabb_contains, aabb_inflate, aabb_union, add, distance, inv_mul_quat, length, lerp, max_int,
    mul_sv, nlerp, offset_aabb, offset_pos, rotate_vector, sub, sub_pos, transform_point, Aabb,
    Pos, Transform, Vec3, WorldTransform,
};
use crate::sensor::SensorHit;
use crate::shape::{
    compute_fat_shape_aabb, compute_shape_aabb, compute_shape_min_extent, shape_flags,
    shape_time_of_impact, should_shapes_collide, Shape, ShapeGeometry,
};
use crate::solver_set::AWAKE_SET;
use crate::types::BodyType;
use crate::world::World;

const MAX_CONTINUOUS_SENSOR_HITS: usize = 8;

/// Scratch for one fast-body continuous solve. (b3ContinuousContext)
struct ContinuousContext {
    fast_body_sim_index: usize,
    fast_shape_id: i32,
    sweep: Sweep,
    base: Pos,
    fraction: f32,
    sensor_hits: [SensorHit; MAX_CONTINUOUS_SENSOR_HITS],
    sensor_fractions: [f32; MAX_CONTINUOUS_SENSOR_HITS],
    sensor_count: i32,
    distance_iterations: i32,
    push_back_iterations: i32,
    root_iterations: i32,
}

/// Make a sweep relative to a base position to keep TOI in float precision.
/// (b3MakeRelativeSweep)
pub(super) fn make_relative_sweep(body_sim: &BodySim, base: Pos) -> Sweep {
    Sweep {
        c1: sub_pos(body_sim.center0, base),
        c2: sub_pos(body_sim.center, base),
        q1: body_sim.rotation0,
        q2: body_sim.transform.q,
        local_center: body_sim.local_center,
    }
}

/// Process one broad-phase leaf against the fast shape. (b3ContinuousQueryCallback)
fn continuous_query_callback(world: &mut World, ctx: &mut ContinuousContext, shape_id: i32) {
    let fast_shape_id = ctx.fast_shape_id;
    debug_assert!(world.shapes[fast_shape_id as usize].sensor_index == NULL_INDEX);

    if shape_id == fast_shape_id {
        return;
    }

    let fast_body_id = world.shapes[fast_shape_id as usize].body_id;
    let other_body_id = world.shapes[shape_id as usize].body_id;
    if other_body_id == fast_body_id {
        return;
    }

    let is_sensor = world.shapes[shape_id as usize].sensor_index != NULL_INDEX;
    if is_sensor
        && ((world.shapes[shape_id as usize].flags & shape_flags::ENABLE_SENSOR_EVENTS) == 0
            || (world.shapes[fast_shape_id as usize].flags & shape_flags::ENABLE_SENSOR_EVENTS)
                == 0)
    {
        return;
    }

    let filter_fast = world.shapes[fast_shape_id as usize].filter;
    let filter_other = world.shapes[shape_id as usize].filter;
    if !should_shapes_collide(filter_fast, filter_other) {
        return;
    }

    let fast_sim = &world.solver_sets[AWAKE_SET as usize].body_sims[ctx.fast_body_sim_index];
    debug_assert!(
        world.bodies[other_body_id as usize].type_ == BodyType::Static
            || (fast_sim.flags & body_flags::IS_BULLET) != 0
    );

    // Skip bullets
    if (world.bodies[other_body_id as usize].flags & body_flags::IS_BULLET) != 0 {
        return;
    }

    if !should_bodies_collide(world, fast_body_id, other_body_id) {
        return;
    }

    let custom_a =
        (world.shapes[shape_id as usize].flags & shape_flags::ENABLE_CUSTOM_FILTERING) != 0;
    let custom_b =
        (world.shapes[fast_shape_id as usize].flags & shape_flags::ENABLE_CUSTOM_FILTERING) != 0;
    if custom_a || custom_b {
        if let Some(fcn) = world.custom_filter_fcn {
            let id_a = ShapeId {
                index1: shape_id + 1,
                world0: world.world_id,
                generation: world.shapes[shape_id as usize].generation,
            };
            let id_b = ShapeId {
                index1: fast_shape_id + 1,
                world0: world.world_id,
                generation: world.shapes[fast_shape_id as usize].generation,
            };
            if !fcn(&*world, id_a, id_b, world.custom_filter_context) {
                return;
            }
        }
    }

    let other_set = world.bodies[other_body_id as usize].set_index;
    let other_local = world.bodies[other_body_id as usize].local_index;
    let other_sim = world.solver_sets[other_set as usize].body_sims[other_local as usize];
    let sweep_a = make_relative_sweep(&other_sim, ctx.base);
    let sweep_b = ctx.sweep;
    let max_fraction = ctx.fraction;

    let output = shape_time_of_impact(
        &world.shapes[shape_id as usize],
        &world.shapes[fast_shape_id as usize],
        &sweep_a,
        &sweep_b,
        max_fraction,
    );

    if is_sensor {
        if output.fraction <= ctx.fraction && ctx.sensor_count < MAX_CONTINUOUS_SENSOR_HITS as i32 {
            let index = ctx.sensor_count as usize;
            ctx.sensor_hits[index] = SensorHit {
                sensor_id: shape_id,
                visitor_id: fast_shape_id,
            };
            ctx.sensor_fractions[index] = output.fraction;
            ctx.sensor_count += 1;
        }
    } else if 0.0 < output.fraction && output.fraction < ctx.fraction {
        let mut did_hit = true;
        let pre_a =
            (world.shapes[shape_id as usize].flags & shape_flags::ENABLE_PRE_SOLVE_EVENTS) != 0;
        let pre_b = (world.shapes[fast_shape_id as usize].flags
            & shape_flags::ENABLE_PRE_SOLVE_EVENTS)
            != 0;
        if did_hit && (pre_a || pre_b) {
            if let Some(pre_solve) = world.pre_solve_fcn {
                let shape_id_a = ShapeId {
                    index1: shape_id + 1,
                    world0: world.world_id,
                    generation: world.shapes[shape_id as usize].generation,
                };
                let shape_id_b = ShapeId {
                    index1: fast_shape_id + 1,
                    world0: world.world_id,
                    generation: world.shapes[fast_shape_id as usize].generation,
                };
                let point = offset_pos(ctx.base, output.point);
                did_hit = pre_solve(
                    shape_id_a,
                    shape_id_b,
                    point,
                    output.normal,
                    world.pre_solve_context,
                );
            }
        }

        if did_hit {
            ctx.fraction = output.fraction;
            ctx.distance_iterations = max_int(ctx.distance_iterations, output.distance_iterations);
            ctx.push_back_iterations =
                max_int(ctx.push_back_iterations, output.push_back_iterations);
            ctx.root_iterations = max_int(ctx.root_iterations, output.root_iterations);
        }
    }
}

/// (b3IsShapeFast)
fn is_shape_fast(
    shape: &Shape,
    centroid1: Vec3,
    centroid2: Vec3,
    rotation_chord: f32,
    safety_factor: f32,
) -> bool {
    let radius = match &shape.geometry {
        ShapeGeometry::Sphere(sphere) => sphere.radius,

        ShapeGeometry::Capsule(capsule) => {
            0.5 * distance(capsule.center1, capsule.center2) + capsule.radius
        }

        ShapeGeometry::Hull(hull) => {
            let farthest_point = farthest_point_on_aabb(hull.aabb, shape.local_centroid);
            distance(farthest_point, shape.local_centroid)
        }

        _ => return true,
    };

    let min_extent = compute_shape_min_extent(shape, shape.local_centroid);
    let max_motion = distance(centroid1, centroid2) + rotation_chord * radius;
    max_motion > safety_factor * min_extent
}

fn query_tree_shape_ids(world: &World, tree_type: BodyType, swept_box: Aabb) -> Vec<i32> {
    let mut ids = Vec::new();
    world.broad_phase.trees[tree_type as usize].query(
        swept_box,
        DEFAULT_MASK_BITS,
        false,
        |_proxy_id, user_data| {
            ids.push(user_data as i32);
            true
        },
    );
    ids
}

/// Continuous collision of a fast dynamic body. (b3SolveContinuous)
pub(super) fn solve_continuous(world: &mut World, body_sim_index: i32, dt: f32) {
    let speculative = crate::constants::speculative_distance();
    let sim_index = body_sim_index as usize;

    {
        let fast = &world.solver_sets[AWAKE_SET as usize].body_sims[sim_index];
        debug_assert!((fast.flags & body_flags::IS_FAST) != 0);
    }

    let base = world.solver_sets[AWAKE_SET as usize].body_sims[sim_index].center0;
    let sweep = make_relative_sweep(
        &world.solver_sets[AWAKE_SET as usize].body_sims[sim_index],
        base,
    );

    let xf1 = Transform {
        q: sweep.q1,
        p: sub(sweep.c1, rotate_vector(sweep.q1, sweep.local_center)),
    };
    let xf2 = Transform {
        q: sweep.q2,
        p: sub(sweep.c2, rotate_vector(sweep.q2, sweep.local_center)),
    };

    let fast_body_id = world.solver_sets[AWAKE_SET as usize].body_sims[sim_index].body_id;
    let is_bullet = (world.solver_sets[AWAKE_SET as usize].body_sims[sim_index].flags
        & body_flags::IS_BULLET)
        != 0;
    let safety_factor = world.bodies[fast_body_id as usize].safety_factor;
    let rotation_chord = 2.0 * length(inv_mul_quat(sweep.q1, sweep.q2).v);

    let mut ctx = ContinuousContext {
        fast_body_sim_index: sim_index,
        fast_shape_id: NULL_INDEX,
        sweep,
        base,
        fraction: 1.0,
        sensor_hits: [SensorHit::default(); MAX_CONTINUOUS_SENSOR_HITS],
        sensor_fractions: [0.0; MAX_CONTINUOUS_SENSOR_HITS],
        sensor_count: 0,
        distance_iterations: 0,
        push_back_iterations: 0,
        root_iterations: 0,
    };

    let mut shape_id = world.bodies[fast_body_id as usize].head_shape_id;
    while shape_id != NULL_INDEX {
        let next_shape_id = world.shapes[shape_id as usize].next_shape_id;
        let local_centroid = world.shapes[shape_id as usize].local_centroid;
        let centroid1 = transform_point(xf1, local_centroid);
        let centroid2 = transform_point(xf2, local_centroid);

        let box1 = world.shapes[shape_id as usize].aabb;
        let box2 = offset_aabb(
            compute_shape_aabb(&world.shapes[shape_id as usize], xf2),
            base,
        );
        world.shapes[shape_id as usize].aabb = box2;

        let shape_type = world.shapes[shape_id as usize].shape_type();
        if shape_type == ShapeType::Mesh || shape_type == ShapeType::Height {
            shape_id = next_shape_id;
            continue;
        }
        if world.shapes[shape_id as usize].sensor_index != NULL_INDEX {
            shape_id = next_shape_id;
            continue;
        }

        if !is_bullet {
            let is_shape_fast = is_shape_fast(
                &world.shapes[shape_id as usize],
                centroid1,
                centroid2,
                rotation_chord,
                safety_factor,
            );
            if !is_shape_fast {
                shape_id = next_shape_id;
                continue;
            }
        }

        ctx.fast_shape_id = shape_id;
        let swept_box = aabb_union(box1, box2);

        let static_ids = query_tree_shape_ids(world, BodyType::Static, swept_box);
        for id in static_ids {
            continuous_query_callback(world, &mut ctx, id);
        }

        if is_bullet {
            let kinematic_ids = query_tree_shape_ids(world, BodyType::Kinematic, swept_box);
            for id in kinematic_ids {
                continuous_query_callback(world, &mut ctx, id);
            }
            let dynamic_ids = query_tree_shape_ids(world, BodyType::Dynamic, swept_box);
            for id in dynamic_ids {
                continuous_query_callback(world, &mut ctx, id);
            }
        }

        shape_id = next_shape_id;
    }

    if ctx.fraction < 1.0 {
        let q = nlerp(sweep.q1, sweep.q2, ctx.fraction);
        let c = lerp(sweep.c1, sweep.c2, ctx.fraction);
        let origin = sub(c, rotate_vector(q, sweep.local_center));

        let transform = WorldTransform {
            p: offset_pos(base, origin),
            q,
        };
        let center = offset_pos(base, c);
        {
            let sim = &mut world.solver_sets[AWAKE_SET as usize].body_sims[sim_index];
            sim.transform = transform;
            sim.center = center;
            sim.rotation0 = q;
            sim.center0 = center;
            sim.flags |= body_flags::HAD_TIME_OF_IMPACT;
        }

        // Timeloss means there is a lost gravity contribution. Other forces and torques are ignored for now.
        {
            let gravity_scale =
                world.solver_sets[AWAKE_SET as usize].body_sims[sim_index].gravity_scale;
            let gravity = world.gravity;
            let fast_body_state = &mut world.solver_sets[AWAKE_SET as usize].body_states[sim_index];
            let v = fast_body_state.linear_velocity;
            let time_loss = (1.0 - ctx.fraction) * dt;
            let mut dv = mul_sv(-time_loss * gravity_scale, gravity);
            let flags = fast_body_state.flags;
            dv.x = if (flags & body_flags::LOCK_LINEAR_X) != 0 {
                0.0
            } else {
                dv.x
            };
            dv.y = if (flags & body_flags::LOCK_LINEAR_Y) != 0 {
                0.0
            } else {
                dv.y
            };
            dv.z = if (flags & body_flags::LOCK_LINEAR_Z) != 0 {
                0.0
            } else {
                dv.z
            };
            fast_body_state.linear_velocity = add(v, dv);
        }

        // The move event was written before CCD, so correct it with the impact pose
        world.body_move_events[sim_index].transform = transform;

        // Prepare AABBs for broad-phase.
        // Even though a body is fast, it may not move much. So the AABB may not need enlargement.
        let mut shape_id = world.bodies[fast_body_id as usize].head_shape_id;
        while shape_id != NULL_INDEX {
            // Must recompute aabb at the interpolated transform
            let aabb =
                compute_fat_shape_aabb(&world.shapes[shape_id as usize], transform, speculative);
            world.shapes[shape_id as usize].aabb = aabb;

            if !aabb_contains(world.fat_aabbs[shape_id as usize], aabb) {
                let fat_aabb = aabb_inflate(aabb, world.shapes[shape_id as usize].aabb_margin);
                world.fat_aabbs[shape_id as usize] = fat_aabb;

                // Regular bodies mark the hierarchy as moved using atomic operations.
                // Bullets are handled separately at a later stage.
                if !is_bullet {
                    let proxy_key = world.shapes[shape_id as usize].proxy_key;
                    world.broad_phase.mark_proxy_moved(proxy_key, fat_aabb);
                } else {
                    world.solver_sets[AWAKE_SET as usize].body_sims[sim_index].flags |=
                        body_flags::ENLARGE_BULLET_BOUNDS;
                }
            }
            shape_id = world.shapes[shape_id as usize].next_shape_id;
        }
    } else {
        // No time of impact event

        // Advance body
        {
            let sim = &mut world.solver_sets[AWAKE_SET as usize].body_sims[sim_index];
            sim.rotation0 = sim.transform.q;
            sim.center0 = sim.center;
        }

        // Prepare AABBs for broad-phase
        let mut shape_id = world.bodies[fast_body_id as usize].head_shape_id;
        while shape_id != NULL_INDEX {
            // shape->aabb is still valid from above
            let shape_aabb = world.shapes[shape_id as usize].aabb;
            if !aabb_contains(world.fat_aabbs[shape_id as usize], shape_aabb) {
                let fat_aabb =
                    aabb_inflate(shape_aabb, world.shapes[shape_id as usize].aabb_margin);
                world.fat_aabbs[shape_id as usize] = fat_aabb;

                if !is_bullet {
                    let proxy_key = world.shapes[shape_id as usize].proxy_key;
                    world.broad_phase.mark_proxy_moved(proxy_key, fat_aabb);
                } else {
                    world.solver_sets[AWAKE_SET as usize].body_sims[sim_index].flags |=
                        body_flags::ENLARGE_BULLET_BOUNDS;
                }
            }
            shape_id = world.shapes[shape_id as usize].next_shape_id;
        }
    }

    for i in 0..ctx.sensor_count as usize {
        if ctx.sensor_fractions[i] < ctx.fraction {
            world.task_contexts[0].sensor_hits.push(ctx.sensor_hits[i]);
        }
    }

    let task = &mut world.task_contexts[0];
    task.distance_iterations = max_int(task.distance_iterations, ctx.distance_iterations);
    task.push_back_iterations = max_int(task.push_back_iterations, ctx.push_back_iterations);
    task.root_iterations = max_int(task.root_iterations, ctx.root_iterations);
}
