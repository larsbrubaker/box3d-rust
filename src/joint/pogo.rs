// Port of pogo_joint.c: public accessors, force reporting, the prepare/warm-start/solve
// simulation functions, and debug drawing.
//
// The pogo joint is a spring along the ground normal with separate tension and compression force
// limits, used to hold up a dynamic character mover. The spring is solved with an implicit
// mass-spring-damper (spring_damper) on the pogo axis, the frame B y-axis.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use super::{get_joint_sim_check_type, get_joint_sim_check_type_ref, JointSim, JointType};
use crate::body::{body_flags, get_body_transform, BodyState, IDENTITY_BODY_STATE};
use crate::constants::linear_slop;
use crate::core::NULL_INDEX;
use crate::debug_draw::{DebugDraw, HexColor};
use crate::id::JointId;
use crate::math_functions::{
    add, clamp_float, cross, dot, is_normalized, is_valid_float, is_valid_vec3, length, mul_add,
    mul_mv, mul_quat, mul_sub, mul_sv, offset_pos, perp, rotate_vector, spring_damper, sub,
    sub_pos, to_relative_transform, transform_point, transform_world_point, Vec3, WorldTransform,
    VEC3_AXIS_Y,
};
use crate::solver::StepContext;
use crate::solver_set::AWAKE_SET;
use crate::world::World;

/// Set the rest length of a pogo joint. (b3PogoJoint_SetRestLength)
pub fn pogo_joint_set_rest_length(world: &mut World, joint_id: JointId, length: f32) {
    debug_assert!(is_valid_float(length) && length >= 0.0);
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Pogo).pogo();
    if joint.rest_length == length {
        return;
    }

    crate::recording::with_recording(world, |rec| {
        rec.write_pogo_joint_set_rest_length(joint_id, length);
    });
    get_joint_sim_check_type(world, joint_id, JointType::Pogo)
        .pogo_mut()
        .rest_length = length;
}

/// Get the rest length of a pogo joint. (b3PogoJoint_GetRestLength)
pub fn pogo_joint_get_rest_length(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Pogo)
        .pogo()
        .rest_length
}

/// Set the spring stiffness in Hertz. (b3PogoJoint_SetSpringHertz)
pub fn pogo_joint_set_spring_hertz(world: &mut World, joint_id: JointId, hertz: f32) {
    debug_assert!(is_valid_float(hertz) && hertz >= 0.0);
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Pogo).pogo();
    if joint.hertz == hertz {
        return;
    }

    crate::recording::with_recording(world, |rec| {
        rec.write_pogo_joint_set_spring_hertz(joint_id, hertz);
    });
    get_joint_sim_check_type(world, joint_id, JointType::Pogo)
        .pogo_mut()
        .hertz = hertz;
}

/// Get the spring Hertz. (b3PogoJoint_GetSpringHertz)
pub fn pogo_joint_get_spring_hertz(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Pogo)
        .pogo()
        .hertz
}

/// Set the spring damping ratio, non-dimensional. (b3PogoJoint_SetSpringDampingRatio)
pub fn pogo_joint_set_spring_damping_ratio(
    world: &mut World,
    joint_id: JointId,
    damping_ratio: f32,
) {
    debug_assert!(is_valid_float(damping_ratio) && damping_ratio >= 0.0);
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Pogo).pogo();
    if joint.damping_ratio == damping_ratio {
        return;
    }

    crate::recording::with_recording(world, |rec| {
        rec.write_pogo_joint_set_spring_damping_ratio(joint_id, damping_ratio);
    });
    get_joint_sim_check_type(world, joint_id, JointType::Pogo)
        .pogo_mut()
        .damping_ratio = damping_ratio;
}

/// Get the spring damping ratio. (b3PogoJoint_GetSpringDampingRatio)
pub fn pogo_joint_get_spring_damping_ratio(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Pogo)
        .pogo()
        .damping_ratio
}

/// Set the max tension force, usually in Newtons. (b3PogoJoint_SetMaxTensionForce)
pub fn pogo_joint_set_max_tension_force(world: &mut World, joint_id: JointId, force: f32) {
    debug_assert!(is_valid_float(force) && force >= 0.0);
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Pogo).pogo();
    if joint.max_tension_force == force {
        return;
    }

    crate::recording::with_recording(world, |rec| {
        rec.write_pogo_joint_set_max_tension_force(joint_id, force);
    });
    get_joint_sim_check_type(world, joint_id, JointType::Pogo)
        .pogo_mut()
        .max_tension_force = force;
}

/// Get the max tension force, usually in Newtons. (b3PogoJoint_GetMaxTensionForce)
pub fn pogo_joint_get_max_tension_force(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Pogo)
        .pogo()
        .max_tension_force
}

/// Set the max compression force, usually in Newtons. (b3PogoJoint_SetMaxCompressionForce)
pub fn pogo_joint_set_max_compression_force(world: &mut World, joint_id: JointId, force: f32) {
    debug_assert!(is_valid_float(force) && force >= 0.0);
    let joint = get_joint_sim_check_type_ref(world, joint_id, JointType::Pogo).pogo();
    if joint.max_compression_force == force {
        return;
    }

    crate::recording::with_recording(world, |rec| {
        rec.write_pogo_joint_set_max_compression_force(joint_id, force);
    });
    get_joint_sim_check_type(world, joint_id, JointType::Pogo)
        .pogo_mut()
        .max_compression_force = force;
}

/// Get the max compression force, usually in Newtons. (b3PogoJoint_GetMaxCompressionForce)
pub fn pogo_joint_get_max_compression_force(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Pogo)
        .pogo()
        .max_compression_force
}

/// Set the unit ground normal. (b3PogoJoint_SetNormal)
pub fn pogo_joint_set_normal(world: &mut World, joint_id: JointId, normal: Vec3) {
    debug_assert!(is_valid_vec3(normal) && is_normalized(normal));
    let current = get_joint_sim_check_type_ref(world, joint_id, JointType::Pogo)
        .pogo()
        .normal;
    if current.x == normal.x && current.y == normal.y && current.z == normal.z {
        return;
    }

    crate::recording::with_recording(world, |rec| {
        rec.write_pogo_joint_set_normal(joint_id, normal);
    });
    get_joint_sim_check_type(world, joint_id, JointType::Pogo)
        .pogo_mut()
        .normal = normal;
}

/// Get the unit ground normal. (b3PogoJoint_GetNormal)
pub fn pogo_joint_get_normal(world: &World, joint_id: JointId) -> Vec3 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Pogo)
        .pogo()
        .normal
}

/// Get the current length of the pogo. (b3PogoJoint_GetCurrentLength)
pub fn pogo_joint_get_current_length(world: &World, joint_id: JointId) -> f32 {
    let base = get_joint_sim_check_type_ref(world, joint_id, JointType::Pogo);

    let world_transform_a = get_body_transform(world, base.body_id_a);
    let transform_a = to_relative_transform(world_transform_a, world_transform_a.p);
    let transform_b = to_relative_transform(
        get_body_transform(world, base.body_id_b),
        world_transform_a.p,
    );

    let axis = rotate_vector(mul_quat(transform_b.q, base.local_frame_b.q), VEC3_AXIS_Y);
    let p_a = transform_point(transform_a, base.local_frame_a.p);
    let p_b = transform_point(transform_b, base.local_frame_b.p);
    dot(sub(p_b, p_a), axis)
}

/// Get the internal pogo impulse. (b3PogoJoint_GetImpulse)
pub fn pogo_joint_get_impulse(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Pogo)
        .pogo()
        .impulse
}

/// Get the internal pogo velocity. (b3PogoJoint_GetCurrentSpeed)
pub fn pogo_joint_get_current_speed(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Pogo)
        .pogo()
        .velocity
}

/// (b3GetPogoJointForce)
pub fn get_pogo_joint_force(world: &World, base: &JointSim) -> Vec3 {
    mul_sv(world.inv_h * base.pogo().impulse, base.pogo().normal)
}

/// (b3PreparePogoJoint)
pub fn prepare_pogo_joint(world: &World, base: &mut JointSim, context: &StepContext) {
    debug_assert!(base.type_ == JointType::Pogo);

    // chase body id to the solver set where the body lives
    let id_a = base.body_id_a;
    let id_b = base.body_id_b;

    let body_a = &world.bodies[id_a as usize];
    let body_b = &world.bodies[id_b as usize];

    debug_assert!(body_a.set_index == AWAKE_SET || body_b.set_index == AWAKE_SET);

    let local_index_a = body_a.local_index;
    let local_index_b = body_b.local_index;

    let body_sim_a =
        &world.solver_sets[body_a.set_index as usize].body_sims[local_index_a as usize];
    let body_sim_b =
        &world.solver_sets[body_b.set_index as usize].body_sims[local_index_b as usize];

    let m_a = body_sim_a.inv_mass;
    let m_b = body_sim_b.inv_mass;
    let i_a = body_sim_a.inv_inertia_world;
    let i_b = body_sim_b.inv_inertia_world;

    base.inv_mass_a = m_a;
    base.inv_mass_b = m_b;
    base.inv_i_a = i_a;
    base.inv_i_b = i_b;

    let local_frame_a = base.local_frame_a;
    let local_frame_b = base.local_frame_b;

    let joint = base.pogo_mut();
    joint.index_a = if body_a.set_index == AWAKE_SET {
        local_index_a
    } else {
        NULL_INDEX
    };
    joint.index_b = if body_b.set_index == AWAKE_SET {
        local_index_b
    } else {
        NULL_INDEX
    };

    // Compute joint anchor frames with world space rotation, relative to center of mass
    joint.frame_a.q = mul_quat(body_sim_a.transform.q, local_frame_a.q);
    joint.frame_a.p = rotate_vector(
        body_sim_a.transform.q,
        sub(local_frame_a.p, body_sim_a.local_center),
    );
    joint.frame_b.q = mul_quat(body_sim_b.transform.q, local_frame_b.q);
    joint.frame_b.p = rotate_vector(
        body_sim_b.transform.q,
        sub(local_frame_b.p, body_sim_b.local_center),
    );

    // Compute the initial center delta. Incremental position updates are relative to this.
    joint.delta_center = sub_pos(body_sim_b.center, body_sim_a.center);

    let rn_a = cross(joint.frame_a.p, joint.normal);
    let rn_b = cross(joint.frame_b.p, joint.normal);
    let k = m_a + m_b + dot(rn_a, mul_mv(i_a, rn_a)) + dot(rn_b, mul_mv(i_b, rn_b));
    joint.linear_mass = if k > 0.0 { 1.0 / k } else { 0.0 };

    if !context.enable_warm_starting {
        joint.velocity = 0.0;
        joint.impulse = 0.0;
    }
}

/// (b3WarmStartPogoJoint)
pub fn warm_start_pogo_joint(base: &mut JointSim, states: &mut [BodyState]) {
    debug_assert!(base.type_ == JointType::Pogo);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;

    let joint = base.pogo_mut();

    if joint.hertz == 0.0 {
        joint.impulse = 0.0;
        return;
    }

    // dummy state for static bodies
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

    let linear_impulse = mul_sv(joint.impulse, joint.normal);

    if state_a.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_a.linear_velocity = mul_sub(state_a.linear_velocity, m_a, linear_impulse);
        state_a.angular_velocity = sub(
            state_a.angular_velocity,
            mul_mv(i_a, cross(r_a, linear_impulse)),
        );
        states[joint.index_a as usize] = state_a;
    }

    if state_b.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_b.linear_velocity = mul_add(state_b.linear_velocity, m_b, linear_impulse);
        state_b.angular_velocity = add(
            state_b.angular_velocity,
            mul_mv(i_b, cross(r_b, linear_impulse)),
        );
        states[joint.index_b as usize] = state_b;
    }
}

/// (b3SolvePogoJoint)
pub fn solve_pogo_joint(
    base: &mut JointSim,
    context: &StepContext,
    states: &mut [BodyState],
    use_bias: bool,
) {
    debug_assert!(base.type_ == JointType::Pogo);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;
    let i_a = base.inv_i_a;
    let i_b = base.inv_i_b;

    let joint = base.pogo_mut();
    if joint.hertz == 0.0 {
        joint.impulse = 0.0;
        joint.velocity = 0.0;
        return;
    }

    // dummy state for static bodies
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

    let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
    let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

    let mut bias = 0.0f32;
    if use_bias {
        // This is a custom pogo position correction designed to avoid feeding velocity into
        // the mover body. Otherwise the mover can make huge jumps going up stairs. This still
        // works with continuous collision.

        let dc_a = state_a.delta_position;
        let dc_b = state_b.delta_position;
        let d = add(add(sub(dc_b, dc_a), sub(r_b, r_a)), joint.delta_center);

        // The pogo axis is usually fixed as the world y-axis. Different than the ground normal.
        let pogo_axis = rotate_vector(joint.frame_b.q, VEC3_AXIS_Y);
        let c = dot(pogo_axis, d) - joint.rest_length;

        // This could be divided by dot(joint->normal, pogoAxis) to account for the constraint
        // direction, but I'd rather diminish the pogo recovery rate than risk making it huge.
        joint.velocity = spring_damper(
            joint.hertz,
            joint.damping_ratio,
            c,
            joint.velocity,
            context.h,
        );
        bias = -joint.velocity;
    }

    let vr = sub(add(v_b, cross(w_b, r_b)), add(v_a, cross(w_a, r_a)));
    let cdot = dot(joint.normal, vr);

    let max_tension_impulse = context.h * joint.max_tension_force;
    let max_compression_impulse = context.h * joint.max_compression_force;
    let old_impulse = joint.impulse;
    let mut impulse = -joint.linear_mass * (cdot + bias);
    joint.impulse = clamp_float(
        joint.impulse + impulse,
        -max_tension_impulse,
        max_compression_impulse,
    );
    impulse = joint.impulse - old_impulse;

    let p = mul_sv(impulse, joint.normal);
    v_a = mul_sub(v_a, m_a, p);
    w_a = sub(w_a, mul_mv(i_a, cross(r_a, p)));
    v_b = mul_add(v_b, m_b, p);
    w_b = add(w_b, mul_mv(i_b, cross(r_b, p)));

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

/// (b3DrawPogoJoint)
pub(super) fn draw_pogo_joint(
    draw: &mut dyn DebugDraw,
    world: &World,
    base: &JointSim,
    transform_a: WorldTransform,
    transform_b: WorldTransform,
    scale: f32,
) {
    debug_assert!(base.type_ == JointType::Pogo);

    let joint = base.pogo();

    let p_a = transform_world_point(transform_a, base.local_frame_a.p);
    let p_b = transform_world_point(transform_b, base.local_frame_b.p);
    let axis = rotate_vector(mul_quat(transform_b.q, base.local_frame_b.q), VEC3_AXIS_Y);
    let perp1 = perp(axis);
    let perp2 = cross(axis, perp1);

    let length_ = dot(sub_pos(p_b, p_a), axis);
    let foot = offset_pos(p_b, mul_sv(-length_, axis));

    let force = world.inv_h * joint.impulse;

    // Tolerance for reaching max force.
    let saturation = 0.99f32;

    let color = if joint.hertz == 0.0 {
        HexColor::GRAY
    } else if force > 0.0 && force >= saturation * joint.max_compression_force {
        HexColor::RED
    } else if force < 0.0 && -force >= saturation * joint.max_tension_force {
        HexColor::RED
    } else if force > 0.0 {
        HexColor::LIME_GREEN
    } else if force < 0.0 {
        HexColor::DEEP_SKY_BLUE
    } else {
        HexColor::LIGHT_GRAY
    };

    // Spring
    let segment_count = 12;
    let width = 0.1 * scale;
    let mut p1 = p_b;
    for i in 1..=segment_count {
        let side = if i == segment_count {
            0.0
        } else if (i & 1) != 0 {
            width
        } else {
            -width
        };
        let t = i as f32 / segment_count as f32;
        let p2 = offset_pos(p_b, mul_add(mul_sv(-t * length_, axis), side, perp2));
        draw.draw_segment(p1, p2, color);
        p1 = p2;
    }

    // Ground plane
    let rest = offset_pos(p_b, mul_sv(-joint.rest_length, axis));
    let tick = 1.5 * width;
    draw.draw_segment(
        offset_pos(rest, mul_sv(-tick, perp1)),
        offset_pos(rest, mul_sv(tick, perp1)),
        HexColor::WHITE,
    );
    draw.draw_segment(
        offset_pos(rest, mul_sv(-tick, perp2)),
        offset_pos(rest, mul_sv(tick, perp2)),
        HexColor::WHITE,
    );

    if length(sub_pos(p_a, foot)) > linear_slop() {
        draw.draw_segment(foot, p_a, HexColor::DIM_GRAY);
    }

    draw.draw_point(p_a, 6.0, color);
    draw.draw_segment(
        p_a,
        offset_pos(p_a, mul_sv(0.5 * scale, joint.normal)),
        HexColor::ORANGE,
    );

    if draw.draw_joint_extras() {
        let buffer = format!(
            "length = {:.3}, rest = {:.3}, f = {:.1}, v = {:.3}",
            length_, joint.rest_length, force, joint.velocity
        );
        draw.draw_string(p_b, &buffer, color);
    }
}
