// Port of parallel_joint.c: public accessors, torque reporting, and the
// prepare/warm-start/solve simulation functions.
//
// The parallel joint keeps body A's and body B's z-axes collinear (a 2-DOF
// angular constraint about the perpendicular x/y axes) via a soft spring.
// Unlike distance/weld, `b3SolveParallelJoint` takes no `useBias` parameter ╬ô├ç├╢
// the joint is always driven through its own softness.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::{get_joint_sim_check_type, get_joint_sim_check_type_ref, JointSim, JointType};
use crate::body::{body_flags, BodyState, IDENTITY_BODY_STATE};
use crate::core::NULL_INDEX;
use crate::id::JointId;
use crate::math_functions::{
    add, add_mm, blend2, cross, det, dot, dot_quat, inv_mul_quat, length2, length_squared2, mul_mv,
    mul_quat, mul_sv, negate_quat, rotate_vector, solve2, sub, sub2, Mat2, Vec2, Vec3, VEC3_AXIS_X,
    VEC3_AXIS_Y,
};
use crate::solver::{make_soft, StepContext};
use crate::solver_set::AWAKE_SET;
use crate::world::World;

/// (b3ParallelJoint_SetSpringHertz)
pub fn parallel_joint_set_spring_hertz(world: &mut World, joint_id: JointId, hertz: f32) {
    crate::recording::with_recording(world, |rec| {
        rec.write_parallel_joint_set_spring_hertz(joint_id, hertz);
    });
    debug_assert!(hertz >= 0.0);
    get_joint_sim_check_type(world, joint_id, JointType::Parallel)
        .parallel_mut()
        .hertz = hertz;
}

/// (b3ParallelJoint_GetSpringHertz)
pub fn parallel_joint_get_spring_hertz(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Parallel)
        .parallel()
        .hertz
}

/// (b3ParallelJoint_SetSpringDampingRatio)
pub fn parallel_joint_set_spring_damping_ratio(
    world: &mut World,
    joint_id: JointId,
    damping_ratio: f32,
) {
    crate::recording::with_recording(world, |rec| {
        rec.write_parallel_joint_set_spring_damping_ratio(joint_id, damping_ratio);
    });
    debug_assert!(damping_ratio >= 0.0);
    get_joint_sim_check_type(world, joint_id, JointType::Parallel)
        .parallel_mut()
        .damping_ratio = damping_ratio;
}

/// (b3ParallelJoint_GetSpringDampingRatio)
pub fn parallel_joint_get_spring_damping_ratio(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Parallel)
        .parallel()
        .damping_ratio
}

/// (b3ParallelJoint_SetMaxTorque)
pub fn parallel_joint_set_max_torque(world: &mut World, joint_id: JointId, max_force: f32) {
    crate::recording::with_recording(world, |rec| {
        rec.write_parallel_joint_set_max_torque(joint_id, max_force);
    });
    debug_assert!(max_force >= 0.0);
    get_joint_sim_check_type(world, joint_id, JointType::Parallel)
        .parallel_mut()
        .max_torque = max_force;
}

/// (b3ParallelJoint_GetMaxTorque)
pub fn parallel_joint_get_max_torque(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Parallel)
        .parallel()
        .max_torque
}

/// (b3GetParallelJointTorque)
///
/// C takes `(b3World* world, b3JointSim* base)`; the port only needs
/// `world->inv_h`, taken by value so callers that already hold `base` as a
/// mutable borrow of `world` (e.g. a future joint-type dispatcher) don't hit
/// a double-borrow of `world`.
pub fn get_parallel_joint_torque(inv_h: f32, base: &mut JointSim) -> Vec3 {
    let joint = base.parallel_mut();

    let rel_q = inv_mul_quat(joint.quat_a, joint.quat_b);
    joint.perp_axis_x = mul_sv(
        0.5,
        rotate_vector(
            joint.quat_a,
            add(mul_sv(rel_q.s, VEC3_AXIS_X), cross(rel_q.v, VEC3_AXIS_X)),
        ),
    );
    joint.perp_axis_y = mul_sv(
        0.5,
        rotate_vector(
            joint.quat_a,
            add(mul_sv(rel_q.s, VEC3_AXIS_Y), cross(rel_q.v, VEC3_AXIS_Y)),
        ),
    );

    let angular_impulse = blend2(
        joint.perp_impulse.x,
        joint.perp_axis_x,
        joint.perp_impulse.y,
        joint.perp_axis_y,
    );
    mul_sv(inv_h, angular_impulse)
}

/// (b3PrepareParallelJoint)
pub fn prepare_parallel_joint(world: &World, base: &mut JointSim, context: &StepContext) {
    debug_assert!(base.type_ == JointType::Parallel);

    let id_a = base.body_id_a;
    let id_b = base.body_id_b;

    let body_a = &world.bodies[id_a as usize];
    let body_b = &world.bodies[id_b as usize];

    debug_assert!(body_a.set_index == AWAKE_SET || body_b.set_index == AWAKE_SET);

    let body_sim_a =
        &world.solver_sets[body_a.set_index as usize].body_sims[body_a.local_index as usize];
    let body_sim_b =
        &world.solver_sets[body_b.set_index as usize].body_sims[body_b.local_index as usize];

    base.inv_mass_a = body_sim_a.inv_mass;
    base.inv_mass_b = body_sim_b.inv_mass;
    base.inv_i_a = body_sim_a.inv_inertia_world;
    base.inv_i_b = body_sim_b.inv_inertia_world;

    let inv_inertia_sum = add_mm(base.inv_i_a, base.inv_i_b);
    base.fixed_rotation = det(inv_inertia_sum) < 1000.0 * f32::MIN_POSITIVE;

    let local_frame_a_q = base.local_frame_a.q;
    let local_frame_b_q = base.local_frame_b.q;

    let index_a = if body_a.set_index == AWAKE_SET {
        body_a.local_index
    } else {
        NULL_INDEX
    };
    let index_b = if body_b.set_index == AWAKE_SET {
        body_b.local_index
    } else {
        NULL_INDEX
    };

    let quat_a = mul_quat(body_sim_a.transform.q, local_frame_a_q);
    let quat_b = mul_quat(body_sim_b.transform.q, local_frame_b_q);
    let rel_q = inv_mul_quat(quat_a, quat_b);

    // These are needed for warm starting
    let perp_axis_x = mul_sv(
        0.5,
        rotate_vector(
            quat_a,
            add(mul_sv(rel_q.s, VEC3_AXIS_X), cross(rel_q.v, VEC3_AXIS_X)),
        ),
    );
    let perp_axis_y = mul_sv(
        0.5,
        rotate_vector(
            quat_a,
            add(mul_sv(rel_q.s, VEC3_AXIS_Y), cross(rel_q.v, VEC3_AXIS_Y)),
        ),
    );

    let joint = base.parallel_mut();
    joint.index_a = index_a;
    joint.index_b = index_b;
    joint.quat_a = quat_a;
    joint.quat_b = quat_b;
    joint.perp_axis_x = perp_axis_x;
    joint.perp_axis_y = perp_axis_y;

    joint.softness = make_soft(joint.hertz, joint.damping_ratio, context.h);

    if !context.enable_warm_starting {
        joint.perp_impulse = Vec2 { x: 0.0, y: 0.0 };
    }
}

/// (b3WarmStartParallelJoint)
pub fn warm_start_parallel_joint(base: &mut JointSim, states: &mut [BodyState]) {
    debug_assert!(base.type_ == JointType::Parallel);

    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;

    let joint = base.parallel_mut();

    let mut state_a = if joint.index_a == NULL_INDEX {
        IDENTITY_BODY_STATE
    } else {
        states[joint.index_a as usize]
    };
    let mut state_b = if joint.index_b == NULL_INDEX {
        IDENTITY_BODY_STATE
    } else {
        states[joint.index_b as usize]
    };

    let mut w_a = state_a.angular_velocity;
    let mut w_b = state_b.angular_velocity;

    let angular_impulse = blend2(
        joint.perp_impulse.x,
        joint.perp_axis_x,
        joint.perp_impulse.y,
        joint.perp_axis_y,
    );

    w_a = sub(w_a, mul_mv(i_a, angular_impulse));
    w_b = add(w_b, mul_mv(i_b, angular_impulse));

    if state_a.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_a.angular_velocity = w_a;
        states[joint.index_a as usize] = state_a;
    }

    if state_b.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_b.angular_velocity = w_b;
        states[joint.index_b as usize] = state_b;
    }
}

/// (b3SolveParallelJoint)
pub fn solve_parallel_joint(base: &mut JointSim, context: &StepContext, states: &mut [BodyState]) {
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;
    let fixed_rotation = base.fixed_rotation;

    let joint = base.parallel_mut();

    let mut state_a = if joint.index_a == NULL_INDEX {
        IDENTITY_BODY_STATE
    } else {
        states[joint.index_a as usize]
    };
    let mut state_b = if joint.index_b == NULL_INDEX {
        IDENTITY_BODY_STATE
    } else {
        states[joint.index_b as usize]
    };

    let mut w_a = state_a.angular_velocity;
    let mut w_b = state_b.angular_velocity;

    let quat_a = mul_quat(state_a.delta_rotation, joint.quat_a);
    let mut quat_b = mul_quat(state_b.delta_rotation, joint.quat_b);

    if dot_quat(quat_a, quat_b) < 0.0 {
        // this keeps the rotation angle in the range [-pi, pi]
        quat_b = negate_quat(quat_b);
    }

    let rel_q = inv_mul_quat(quat_a, quat_b);

    if !fixed_rotation && joint.max_torque > 0.0 {
        let c = Vec2 {
            x: rel_q.v.x,
            y: rel_q.v.y,
        };
        let bias = Vec2 {
            x: joint.softness.bias_rate * c.x,
            y: joint.softness.bias_rate * c.y,
        };
        let mass_scale = joint.softness.mass_scale;
        let impulse_scale = joint.softness.impulse_scale;

        // Collinearity constraint as 2-by-2
        let perp_axis_x = mul_sv(
            0.5,
            rotate_vector(
                quat_a,
                add(mul_sv(rel_q.s, VEC3_AXIS_X), cross(rel_q.v, VEC3_AXIS_X)),
            ),
        );
        let perp_axis_y = mul_sv(
            0.5,
            rotate_vector(
                quat_a,
                add(mul_sv(rel_q.s, VEC3_AXIS_Y), cross(rel_q.v, VEC3_AXIS_Y)),
            ),
        );
        joint.perp_axis_x = perp_axis_x;
        joint.perp_axis_y = perp_axis_y;

        let inv_inertia_sum = add_mm(i_a, i_b);
        let kxx = dot(perp_axis_x, mul_mv(inv_inertia_sum, perp_axis_x));
        let kyy = dot(perp_axis_y, mul_mv(inv_inertia_sum, perp_axis_y));
        let kxy = dot(perp_axis_x, mul_mv(inv_inertia_sum, perp_axis_y));

        let k = Mat2 {
            cx: Vec2 { x: kxx, y: kxy },
            cy: Vec2 { x: kxy, y: kyy },
        };

        let w_rel = sub(w_b, w_a);
        let cdot = Vec2 {
            x: dot(w_rel, perp_axis_x),
            y: dot(w_rel, perp_axis_y),
        };

        let max_impulse = context.h * joint.max_torque;
        let old_impulse = joint.perp_impulse;
        let cdot_plus_bias = Vec2 {
            x: cdot.x + bias.x,
            y: cdot.y + bias.y,
        };
        let sol = solve2(k, cdot_plus_bias);
        let mut delta_impulse = Vec2 {
            x: -mass_scale * sol.x - impulse_scale * old_impulse.x,
            y: -mass_scale * sol.y - impulse_scale * old_impulse.y,
        };
        joint.perp_impulse = Vec2 {
            x: old_impulse.x + delta_impulse.x,
            y: old_impulse.y + delta_impulse.y,
        };
        if length_squared2(joint.perp_impulse) > max_impulse * max_impulse {
            let s = max_impulse / length2(joint.perp_impulse);
            joint.perp_impulse = Vec2 {
                x: s * joint.perp_impulse.x,
                y: s * joint.perp_impulse.y,
            };
        }

        delta_impulse = sub2(joint.perp_impulse, old_impulse);

        let angular_impulse = blend2(delta_impulse.x, perp_axis_x, delta_impulse.y, perp_axis_y);
        w_a = sub(w_a, mul_mv(i_a, angular_impulse));
        w_b = add(w_b, mul_mv(i_b, angular_impulse));
    }

    if state_a.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_a.angular_velocity = w_a;
        states[joint.index_a as usize] = state_a;
    }

    if state_b.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_b.angular_velocity = w_b;
        states[joint.index_b as usize] = state_b;
    }
}
