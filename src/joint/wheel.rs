// Port of wheel_joint.c prepare / warm-start / solve.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use super::{JointSim, JointType};
use crate::body::{body_flags, BodyState, IDENTITY_BODY_STATE};
use crate::core::NULL_INDEX;
use crate::math_functions::{
    add, add2, add_mm, atan2, blend2, blend3, clamp_float, cross, det, dot, dot_quat, inv_mul_quat,
    make_matrix_from_quat, max_float, mul_add, mul_mv, mul_quat, mul_sub, mul_sv, mul_sv2,
    negate_quat, rotate_vector, solve2, sub, sub2, sub_pos, Mat2, Transform, Vec2, VEC2_ZERO,
    VEC3_AXIS_X, VEC3_AXIS_Y,
};
use crate::solver::{make_soft, StepContext};
use crate::solver_set::AWAKE_SET;
use crate::world::World;

/// (b3PrepareWheelJoint)
pub fn prepare_wheel_joint(world: &World, base: &mut JointSim, context: &StepContext) {
    debug_assert!(base.type_ == JointType::Wheel);

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

    let local_frame_a = base.local_frame_a;
    let local_frame_b = base.local_frame_b;
    let inv_mass_a = base.inv_mass_a;
    let inv_mass_b = base.inv_mass_b;
    let inv_i_a = base.inv_i_a;
    let inv_i_b = base.inv_i_b;

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

    // Compute joint anchor frames with world space rotation, relative to center of mass
    let frame_a_q = mul_quat(body_sim_a.transform.q, local_frame_a.q);
    let frame_a_p = rotate_vector(
        body_sim_a.transform.q,
        sub(local_frame_a.p, body_sim_a.local_center),
    );
    let frame_b_q = mul_quat(body_sim_b.transform.q, local_frame_b.q);
    let frame_b_p = rotate_vector(
        body_sim_b.transform.q,
        sub(local_frame_b.p, body_sim_b.local_center),
    );
    let delta_center = sub_pos(body_sim_b.center, body_sim_a.center);

    let joint = base.wheel_mut();
    joint.index_a = index_a;
    joint.index_b = index_b;
    joint.frame_a = Transform {
        p: frame_a_p,
        q: frame_a_q,
    };
    joint.frame_b = Transform {
        p: frame_b_p,
        q: frame_b_q,
    };
    joint.delta_center = delta_center;

    let r_a = joint.frame_a.p;
    let r_b = joint.frame_b.p;

    let matrix_a = make_matrix_from_quat(joint.frame_a.q);
    let matrix_b = make_matrix_from_quat(joint.frame_b.q);

    // todo use fresh effective masses in the sub-step to avoid divergence like I saw for the prismatic joint

    {
        let suspension_axis = matrix_a.cx;
        let r_an = cross(r_a, suspension_axis);
        let r_bn = cross(r_b, suspension_axis);

        let k = inv_mass_a
            + inv_mass_b
            + dot(r_an, mul_mv(inv_i_a, r_an))
            + dot(r_bn, mul_mv(inv_i_b, r_bn));
        joint.suspension_mass = if k > 0.0 { 1.0 / k } else { 0.0 };
    }

    joint.suspension_softness = make_soft(
        joint.suspension_hertz,
        joint.suspension_damping_ratio,
        context.h,
    );
    joint.steering_softness = make_soft(
        joint.steering_hertz,
        joint.steering_damping_ratio,
        context.h,
    );

    {
        // Rotation axis is the z-axis of body A.
        let spin_axis = matrix_b.cz;
        let k = dot(spin_axis, mul_mv(inv_inertia_sum, spin_axis));
        joint.spin_mass = if k > 0.0 { 1.0 / k } else { 0.0 };
    }

    {
        // Twist constraint around x-axis
        let cs = dot(matrix_b.cz, matrix_a.cz);
        let ss = -dot(matrix_b.cz, matrix_a.cy);
        let mut den = cs * cs + ss * ss;
        den = if den > 0.0 { 1.0 / den } else { 0.0 };
        let steering_axis = mul_sv(
            den,
            cross(
                matrix_b.cz,
                sub(mul_sv(-cs, matrix_a.cy), mul_sv(ss, matrix_a.cz)),
            ),
        );

        let k = dot(steering_axis, mul_mv(inv_inertia_sum, steering_axis));
        joint.steering_mass = if k > 0.0 { 1.0 / k } else { 0.0 };
    }

    if !context.enable_warm_starting {
        joint.linear_impulse = VEC2_ZERO;
        joint.angular_impulse = VEC2_ZERO;
        joint.spin_impulse = 0.0;
        joint.suspension_spring_impulse = 0.0;
        joint.lower_suspension_impulse = 0.0;
        joint.upper_suspension_impulse = 0.0;
        joint.steering_spring_impulse = 0.0;
        joint.lower_steering_impulse = 0.0;
        joint.upper_steering_impulse = 0.0;
    }
}

/// (b3WarmStartWheelJoint)
pub fn warm_start_wheel_joint(base: &mut JointSim, states: &mut [BodyState]) {
    debug_assert!(base.type_ == JointType::Wheel);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;

    let joint = base.wheel_mut();

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

    let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
    let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

    let d = add(
        add(
            sub(state_b.delta_position, state_a.delta_position),
            joint.delta_center,
        ),
        sub(r_b, r_a),
    );

    let quat_a = mul_quat(state_a.delta_rotation, joint.frame_a.q);
    let mut quat_b = mul_quat(state_b.delta_rotation, joint.frame_b.q);
    if dot_quat(quat_a, quat_b) < 0.0 {
        // this keeps the rotation angle in the range [-pi, pi]
        quat_b = negate_quat(quat_b);
    }

    let matrix_a = make_matrix_from_quat(quat_a);
    let matrix_b = make_matrix_from_quat(quat_b);

    let s_ax = cross(add(d, r_a), matrix_a.cx);
    let s_bx = cross(r_b, matrix_a.cx);
    let s_ay = cross(add(d, r_a), matrix_a.cy);
    let s_by = cross(r_b, matrix_a.cy);
    let s_az = cross(add(d, r_a), matrix_a.cz);
    let s_bz = cross(r_b, matrix_a.cz);

    let suspension_impulse = joint.suspension_spring_impulse + joint.lower_suspension_impulse
        - joint.upper_suspension_impulse;

    let linear_impulse_y = joint.linear_impulse.x;
    let linear_impulse_z = joint.linear_impulse.y;
    let angular_impulse_x = joint.angular_impulse.x;
    let angular_impulse_y = joint.angular_impulse.y;

    let linear_impulse = blend3(
        suspension_impulse,
        matrix_a.cx,
        linear_impulse_y,
        matrix_a.cy,
        linear_impulse_z,
        matrix_a.cz,
    );
    let angular_impulse_a = blend3(
        suspension_impulse,
        s_ax,
        linear_impulse_y,
        s_ay,
        linear_impulse_z,
        s_az,
    );
    let angular_impulse_b = blend3(
        suspension_impulse,
        s_bx,
        linear_impulse_y,
        s_by,
        linear_impulse_z,
        s_bz,
    );
    let mut angular_impulse = mul_sv(joint.spin_impulse, matrix_a.cz);

    let spin_axis = matrix_b.cz;

    if joint.enable_steering {
        // Twist constraint around x-axis
        let cs = dot(matrix_b.cz, matrix_a.cz);
        let ss = -dot(matrix_b.cz, matrix_a.cy);
        let mut den = cs * cs + ss * ss;
        den = if den > 0.0 { 1.0 / den } else { 0.0 };
        let steering_axis = mul_sv(
            den,
            cross(
                matrix_b.cz,
                sub(mul_sv(-cs, matrix_a.cy), mul_sv(ss, matrix_a.cz)),
            ),
        );

        let perp_axis = cross(spin_axis, matrix_a.cx);
        let steering_impulse = joint.steering_spring_impulse + joint.lower_steering_impulse
            - joint.upper_steering_impulse;
        angular_impulse = blend3(
            angular_impulse_x,
            perp_axis,
            joint.spin_impulse,
            spin_axis,
            steering_impulse,
            steering_axis,
        );
    } else {
        let rel_q = inv_mul_quat(quat_a, quat_b);
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
        angular_impulse = add(
            angular_impulse,
            blend3(
                angular_impulse_x,
                perp_axis_x,
                angular_impulse_y,
                perp_axis_y,
                joint.spin_impulse,
                spin_axis,
            ),
        );
    }

    if state_a.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_a.linear_velocity = mul_sub(state_a.linear_velocity, m_a, linear_impulse);
        state_a.angular_velocity = sub(
            state_a.angular_velocity,
            mul_mv(i_a, add(angular_impulse_a, angular_impulse)),
        );
        states[joint.index_a as usize] = state_a;
    }

    if state_b.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_b.linear_velocity = mul_add(state_b.linear_velocity, m_b, linear_impulse);
        state_b.angular_velocity = add(
            state_b.angular_velocity,
            mul_mv(i_b, add(angular_impulse_b, angular_impulse)),
        );
        states[joint.index_b as usize] = state_b;
    }
}

/// (b3SolveWheelJoint)
pub fn solve_wheel_joint(
    base: &mut JointSim,
    context: &StepContext,
    states: &mut [BodyState],
    use_bias: bool,
) {
    debug_assert!(base.type_ == JointType::Wheel);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;
    let fixed_rotation = base.fixed_rotation;
    let constraint_softness = base.constraint_softness;

    let joint = base.wheel_mut();

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

    let mut v_a = state_a.linear_velocity;
    let mut w_a = state_a.angular_velocity;
    let mut v_b = state_b.linear_velocity;
    let mut w_b = state_b.angular_velocity;

    // current anchors
    let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
    let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

    let quat_a = mul_quat(state_a.delta_rotation, joint.frame_a.q);
    let mut quat_b = mul_quat(state_b.delta_rotation, joint.frame_b.q);

    if dot_quat(quat_a, quat_b) < 0.0 {
        // this keeps the rotation angle in the range [-pi, pi]
        quat_b = negate_quat(quat_b);
    }

    let rel_q = inv_mul_quat(quat_a, quat_b);
    let matrix_a = make_matrix_from_quat(quat_a);
    let matrix_b = make_matrix_from_quat(quat_b);

    let d = add(
        add(
            sub(state_b.delta_position, state_a.delta_position),
            joint.delta_center,
        ),
        sub(r_b, r_a),
    );
    let s_ax = cross(add(d, r_a), matrix_a.cx);
    let s_bx = cross(r_b, matrix_a.cx);
    let s_ay = cross(add(d, r_a), matrix_a.cy);
    let s_by = cross(r_b, matrix_a.cy);
    let s_az = cross(add(d, r_a), matrix_a.cz);
    let s_bz = cross(r_b, matrix_a.cz);

    let translation = dot(matrix_a.cx, d);

    // Steering param ib = cz_b, ia = cz_a, ja = -cy_a
    let cs = dot(matrix_b.cz, matrix_a.cz);
    let ss = -dot(matrix_b.cz, matrix_a.cy);
    let mut den = cs * cs + ss * ss;
    den = if den > 0.0 { 1.0 / den } else { 0.0 };
    let steering_axis = mul_sv(
        den,
        cross(
            matrix_b.cz,
            sub(mul_sv(-cs, matrix_a.cy), mul_sv(ss, matrix_a.cz)),
        ),
    );

    // motor constraint
    if joint.enable_spin_motor && !fixed_rotation {
        let spin_axis = matrix_b.cz;
        let cdot = dot(sub(w_b, w_a), spin_axis) - joint.spin_speed;
        let mut impulse = -joint.spin_mass * cdot;
        let old_impulse = joint.spin_impulse;
        let max_impulse = context.h * joint.max_spin_torque;
        joint.spin_impulse = clamp_float(joint.spin_impulse + impulse, -max_impulse, max_impulse);
        impulse = joint.spin_impulse - old_impulse;

        w_a = sub(w_a, mul_mv(i_a, mul_sv(impulse, spin_axis)));
        w_b = add(w_b, mul_mv(i_b, mul_sv(impulse, spin_axis)));
    }

    // suspension
    if joint.enable_suspension_spring {
        // This is a real spring and should be applied even during relax
        let c = translation;
        let bias = joint.suspension_softness.bias_rate * c;
        let mass_scale = joint.suspension_softness.mass_scale;
        let impulse_scale = joint.suspension_softness.impulse_scale;

        let cdot = dot(matrix_a.cx, sub(v_b, v_a)) + dot(s_bx, w_b) - dot(s_ax, w_a);
        let impulse = -mass_scale * joint.suspension_mass * (cdot + bias)
            - impulse_scale * joint.suspension_spring_impulse;
        joint.suspension_spring_impulse += impulse;

        let linear_impulse = mul_sv(impulse, matrix_a.cx);
        let angular_impulse_a = mul_sv(impulse, s_ax);
        let angular_impulse_b = mul_sv(impulse, s_bx);

        v_a = mul_sub(v_a, m_a, linear_impulse);
        w_a = sub(w_a, mul_mv(i_a, angular_impulse_a));
        v_b = mul_add(v_b, m_b, linear_impulse);
        w_b = add(w_b, mul_mv(i_b, angular_impulse_b));
    }

    // steering
    if joint.enable_steering && !fixed_rotation {
        let steering_angle = atan2(ss, cs);

        {
            // This is a real spring and should be applied even during relax
            let c = steering_angle - joint.target_steering_angle;
            let bias = joint.steering_softness.bias_rate * c;
            let mass_scale = joint.steering_softness.mass_scale;
            let impulse_scale = joint.steering_softness.impulse_scale;

            let cdot = dot(steering_axis, sub(w_b, w_a));
            let old_impulse = joint.steering_spring_impulse;
            let mut impulse =
                -mass_scale * joint.steering_mass * (cdot + bias) - impulse_scale * old_impulse;
            let max_impulse = context.h * joint.max_steering_torque;
            joint.steering_spring_impulse =
                clamp_float(old_impulse + impulse, -max_impulse, max_impulse);
            impulse = joint.steering_spring_impulse - old_impulse;

            w_a = sub(w_a, mul_mv(i_a, mul_sv(impulse, steering_axis)));
            w_b = add(w_b, mul_mv(i_b, mul_sv(impulse, steering_axis)));
        }

        if joint.enable_steering_limit {
            // Lower limit
            {
                let c = steering_angle - joint.lower_steering_limit;
                let mut bias = 0.0;
                let mut mass_scale = 1.0;
                let mut impulse_scale = 0.0;

                if c > 0.0 {
                    // speculation
                    bias = c * context.inv_h;
                } else if use_bias {
                    bias = constraint_softness.bias_rate * c;
                    mass_scale = constraint_softness.mass_scale;
                    impulse_scale = constraint_softness.impulse_scale;
                }

                let cdot = dot(steering_axis, sub(w_b, w_a));
                let old_impulse = joint.lower_steering_impulse;
                let mut impulse =
                    -mass_scale * joint.steering_mass * (cdot + bias) - impulse_scale * old_impulse;
                joint.lower_steering_impulse = max_float(old_impulse + impulse, 0.0);
                impulse = joint.lower_steering_impulse - old_impulse;

                w_a = sub(w_a, mul_mv(i_a, mul_sv(impulse, steering_axis)));
                w_b = add(w_b, mul_mv(i_b, mul_sv(impulse, steering_axis)));
            }

            // Upper limit
            // Note: signs are flipped to keep c positive when the constraint is satisfied.
            // This also keeps the impulse positive when the limit is active.
            {
                // sign flipped
                let c = joint.upper_steering_limit - steering_angle;
                let mut bias = 0.0;
                let mut mass_scale = 1.0;
                let mut impulse_scale = 0.0;

                if c > 0.0 {
                    // speculation
                    bias = c * context.inv_h;
                } else if use_bias {
                    bias = constraint_softness.bias_rate * c;
                    mass_scale = constraint_softness.mass_scale;
                    impulse_scale = constraint_softness.impulse_scale;
                }

                // sign flipped on cdot
                let cdot = dot(steering_axis, sub(w_a, w_b));
                let old_impulse = joint.upper_steering_impulse;
                let mut impulse =
                    -mass_scale * joint.steering_mass * (cdot + bias) - impulse_scale * old_impulse;
                joint.upper_steering_impulse = max_float(old_impulse + impulse, 0.0);
                impulse = joint.upper_steering_impulse - old_impulse;

                // sign flipped on applied impulse
                w_a = add(w_a, mul_mv(i_a, mul_sv(impulse, steering_axis)));
                w_b = sub(w_b, mul_mv(i_b, mul_sv(impulse, steering_axis)));
            }
        }
    }

    if joint.enable_suspension_limit {
        // Lower limit
        {
            let c = translation - joint.lower_suspension_limit;
            let mut bias = 0.0;
            let mut mass_scale = 1.0;
            let mut impulse_scale = 0.0;

            if c > 0.0 {
                // speculation
                bias = c * context.inv_h;
            } else if use_bias {
                bias = constraint_softness.bias_rate * c;
                mass_scale = constraint_softness.mass_scale;
                impulse_scale = constraint_softness.impulse_scale;
            }

            let cdot = dot(matrix_a.cx, sub(v_b, v_a)) + dot(s_bx, w_b) - dot(s_ax, w_a);
            let mut impulse = -mass_scale * joint.suspension_mass * (cdot + bias)
                - impulse_scale * joint.lower_suspension_impulse;
            let old_impulse = joint.lower_suspension_impulse;
            joint.lower_suspension_impulse = max_float(old_impulse + impulse, 0.0);
            impulse = joint.lower_suspension_impulse - old_impulse;

            let linear_impulse = mul_sv(impulse, matrix_a.cx);
            let angular_impulse_a = mul_sv(impulse, s_ax);
            let angular_impulse_b = mul_sv(impulse, s_bx);

            v_a = mul_sub(v_a, m_a, linear_impulse);
            w_a = sub(w_a, mul_mv(i_a, angular_impulse_a));
            v_b = mul_add(v_b, m_b, linear_impulse);
            w_b = add(w_b, mul_mv(i_b, angular_impulse_b));
        }

        // Upper limit
        // Note: signs are flipped to keep c positive when the constraint is satisfied.
        // This also keeps the impulse positive when the limit is active.
        {
            // sign flipped
            let c = joint.upper_suspension_limit - translation;
            let mut bias = 0.0;
            let mut mass_scale = 1.0;
            let mut impulse_scale = 0.0;

            if c > 0.0 {
                // speculation
                bias = c * context.inv_h;
            } else if use_bias {
                bias = constraint_softness.bias_rate * c;
                mass_scale = constraint_softness.mass_scale;
                impulse_scale = constraint_softness.impulse_scale;
            }

            // sign flipped on cdot
            let cdot = dot(matrix_a.cx, sub(v_a, v_b)) + dot(s_ax, w_a) - dot(s_bx, w_b);
            let mut impulse = -mass_scale * joint.suspension_mass * (cdot + bias)
                - impulse_scale * joint.upper_suspension_impulse;
            let old_impulse = joint.upper_suspension_impulse;
            joint.upper_suspension_impulse = max_float(old_impulse + impulse, 0.0);
            impulse = joint.upper_suspension_impulse - old_impulse;

            let linear_impulse = mul_sv(impulse, matrix_a.cx);
            let angular_impulse_a = mul_sv(impulse, s_ax);
            let angular_impulse_b = mul_sv(impulse, s_bx);

            // sign flipped on applied impulse
            v_a = mul_add(v_a, m_a, linear_impulse);
            w_a = add(w_a, mul_mv(i_a, angular_impulse_a));
            v_b = mul_sub(v_b, m_b, linear_impulse);
            w_b = sub(w_b, mul_mv(i_b, angular_impulse_b));
        }
    }

    // Collinearity constraint
    if !fixed_rotation {
        if joint.enable_steering {
            let mut bias = 0.0;
            let mut mass_scale = 1.0;
            let mut impulse_scale = 0.0;
            if use_bias {
                let c = dot(matrix_a.cx, matrix_b.cz);

                bias = constraint_softness.bias_rate * c;
                mass_scale = constraint_softness.mass_scale;
                impulse_scale = constraint_softness.impulse_scale;
            }

            let u = cross(matrix_b.cz, matrix_a.cx);
            let cdot = dot(sub(w_b, w_a), u);

            let inv_inertia_sum = add_mm(i_a, i_b);
            let k = dot(u, mul_mv(inv_inertia_sum, u));
            let perp_mass = if k > 0.0 { 1.0 / k } else { 0.0 };

            let delta_impulse =
                -mass_scale * perp_mass * (cdot + bias) - impulse_scale * joint.angular_impulse.x;
            joint.angular_impulse.x += delta_impulse;

            w_a = mul_sub(w_a, delta_impulse, mul_mv(i_a, u));
            w_b = mul_add(w_b, delta_impulse, mul_mv(i_b, u));
        } else {
            let mut bias = VEC2_ZERO;
            let mut mass_scale = 1.0;
            let mut impulse_scale = 0.0;

            if use_bias {
                let c = Vec2 {
                    x: rel_q.v.x,
                    y: rel_q.v.y,
                };
                bias = Vec2 {
                    x: constraint_softness.bias_rate * c.x,
                    y: constraint_softness.bias_rate * c.y,
                };
                mass_scale = constraint_softness.mass_scale;
                impulse_scale = constraint_softness.impulse_scale;
            }

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
            let old_impulse = joint.angular_impulse;
            let cdot_plus_bias = add2(cdot, bias);
            let sol = solve2(k, cdot_plus_bias);
            let delta_impulse = Vec2 {
                x: -mass_scale * sol.x - impulse_scale * old_impulse.x,
                y: -mass_scale * sol.y - impulse_scale * old_impulse.y,
            };
            joint.angular_impulse = add2(old_impulse, delta_impulse);

            let angular_impulse =
                blend2(delta_impulse.x, perp_axis_x, delta_impulse.y, perp_axis_y);
            w_a = sub(w_a, mul_mv(i_a, angular_impulse));
            w_b = add(w_b, mul_mv(i_b, angular_impulse));
        }
    }

    // Solve point-to-line constraint
    {
        let perp_y = matrix_a.cy;
        let perp_z = matrix_a.cz;

        let mut bias = VEC2_ZERO;
        let mut mass_scale = 1.0;
        let mut impulse_scale = 0.0;
        if use_bias {
            let c = Vec2 {
                x: dot(perp_y, d),
                y: dot(perp_z, d),
            };
            bias = Vec2 {
                x: constraint_softness.bias_rate * c.x,
                y: constraint_softness.bias_rate * c.y,
            };
            mass_scale = constraint_softness.mass_scale;
            impulse_scale = constraint_softness.impulse_scale;
        }

        let v_rel = sub(sub(add(v_b, cross(w_b, r_b)), v_a), cross(w_a, add(r_a, d)));
        let cdot = Vec2 {
            x: dot(perp_y, v_rel),
            y: dot(perp_z, v_rel),
        };

        //// K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
        ///// Jx = [-perpX, -cross(d + rA, perpX), perpX, cross(rB, perpX)]

        let kyy = m_a + m_b + dot(s_ay, mul_mv(i_a, s_ay)) + dot(s_by, mul_mv(i_b, s_by));
        let kyz = dot(s_ay, mul_mv(i_a, s_az)) + dot(s_by, mul_mv(i_b, s_bz));
        let kzz = m_a + m_b + dot(s_az, mul_mv(i_a, s_az)) + dot(s_bz, mul_mv(i_b, s_bz));

        let k = Mat2 {
            cx: Vec2 { x: kyy, y: kyz },
            cy: Vec2 { x: kyz, y: kzz },
        };

        let old_impulse = joint.linear_impulse;
        let cdot_plus_bias = add2(cdot, bias);
        let sol = solve2(k, cdot_plus_bias);
        let delta_impulse = sub2(
            mul_sv2(-mass_scale, sol),
            mul_sv2(impulse_scale, old_impulse),
        );
        joint.linear_impulse = add2(old_impulse, delta_impulse);

        let linear_impulse = blend2(delta_impulse.x, perp_y, delta_impulse.y, perp_z);

        v_a = mul_sub(v_a, m_a, linear_impulse);
        w_a = sub(
            w_a,
            mul_mv(i_a, blend2(delta_impulse.x, s_ay, delta_impulse.y, s_az)),
        );
        v_b = mul_add(v_b, m_b, linear_impulse);
        w_b = add(
            w_b,
            mul_mv(i_b, blend2(delta_impulse.x, s_by, delta_impulse.y, s_bz)),
        );
    }

    if state_a.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_a.linear_velocity = v_a;
        state_a.angular_velocity = w_a;
        states[joint.index_a as usize] = state_a;
    }

    if state_b.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_b.linear_velocity = v_b;
        state_b.angular_velocity = w_b;
        states[joint.index_b as usize] = state_b;
    }
}
