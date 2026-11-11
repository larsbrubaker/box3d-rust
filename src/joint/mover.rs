// Port of mover_joint.c: public accessors, force reporting, the prepare/warm-start/solve
// simulation functions, and debug drawing.
//
// The mover joint drives the relative linear velocity in the xz plane of joint frame B with a
// force budget. It does not affect rotation, so the prepare zeroes the inverse inertias.
// Like `b3SolveMotorJoint`, `b3SolveMoverJoint` takes no `useBias` parameter.
//
// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

use super::{get_joint_sim_check_type, get_joint_sim_check_type_ref, JointSim, JointType};
use crate::body::{body_flags, get_body_state_index, BodyState, IDENTITY_BODY_STATE};
use crate::core::NULL_INDEX;
use crate::debug_draw::{DebugDraw, HexColor};
use crate::id::JointId;
use crate::math_functions::{
    add, blend2, clamp_float, compute_cos_sin, cross, dot, dot2, is_valid_float, is_valid_vec3,
    length, length2, lerp_float, make_matrix_from_quat, mul_add, mul_quat, mul_sub, mul_sv,
    mul_sv2, mul_world_transforms, normalize, offset_pos, sub, sub2, sub_pos, Pos, Vec2, Vec3,
    WorldTransform, MAT3_ZERO, PI, VEC2_ZERO, VEC3_ZERO,
};
use crate::solver::StepContext;
use crate::solver_set::AWAKE_SET;
use crate::world::World;

/// Set the desired relative linear velocity, usually in meters per second.
/// (b3MoverJoint_SetLinearVelocity)
pub fn mover_joint_set_linear_velocity(world: &mut World, joint_id: JointId, velocity: Vec3) {
    debug_assert!(is_valid_vec3(velocity));
    crate::recording::with_recording(world, |rec| {
        rec.write_mover_joint_set_linear_velocity(joint_id, velocity);
    });
    get_joint_sim_check_type(world, joint_id, JointType::Mover)
        .mover_mut()
        .linear_velocity = velocity;
}

/// Get the desired relative linear velocity, usually in meters per second.
/// (b3MoverJoint_GetLinearVelocity)
pub fn mover_joint_get_linear_velocity(world: &World, joint_id: JointId) -> Vec3 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Mover)
        .mover()
        .linear_velocity
}

/// Set the joint maximum force, usually in newtons. (b3MoverJoint_SetMaxVelocityForce)
pub fn mover_joint_set_max_velocity_force(world: &mut World, joint_id: JointId, max_force: f32) {
    debug_assert!(is_valid_float(max_force) && max_force >= 0.0);
    crate::recording::with_recording(world, |rec| {
        rec.write_mover_joint_set_max_velocity_force(joint_id, max_force);
    });
    get_joint_sim_check_type(world, joint_id, JointType::Mover)
        .mover_mut()
        .max_force = max_force;
}

/// Get the joint maximum force, usually in newtons. (b3MoverJoint_GetMaxVelocityForce)
pub fn mover_joint_get_max_velocity_force(world: &World, joint_id: JointId) -> f32 {
    get_joint_sim_check_type_ref(world, joint_id, JointType::Mover)
        .mover()
        .max_force
}

/// (b3GetMoverJointForce)
pub fn get_mover_joint_force(world: &World, base: &JointSim) -> Vec3 {
    let joint = base.mover();
    let p = blend2(joint.impulse.x, joint.axis_x, joint.impulse.y, joint.axis_z);
    mul_sv(world.inv_h, p)
}

/// (b3PrepareMoverJoint)
pub fn prepare_mover_joint(world: &World, base: &mut JointSim, context: &StepContext) {
    debug_assert!(base.type_ == JointType::Mover);

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

    base.inv_mass_a = m_a;
    base.inv_mass_b = m_b;
    base.inv_i_a = MAT3_ZERO;
    base.inv_i_b = MAT3_ZERO;

    let quat_b = mul_quat(body_sim_b.transform.q, base.local_frame_b.q);
    let matrix_b = make_matrix_from_quat(quat_b);

    let joint = base.mover_mut();
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

    joint.axis_x = matrix_b.cx;
    joint.axis_z = matrix_b.cz;

    let k = m_a + m_b;
    joint.linear_mass = if k > 0.0 { 1.0 / k } else { 0.0 };

    if !context.enable_warm_starting {
        joint.impulse = VEC2_ZERO;
    }
}

/// (b3WarmStartMoverJoint)
pub fn warm_start_mover_joint(base: &mut JointSim, states: &mut [BodyState]) {
    debug_assert!(base.type_ == JointType::Mover);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;

    let joint = base.mover();

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

    let p = blend2(joint.impulse.x, joint.axis_x, joint.impulse.y, joint.axis_z);
    if state_a.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_a.linear_velocity = mul_sub(state_a.linear_velocity, m_a, p);
        states[joint.index_a as usize] = state_a;
    }

    if state_b.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_b.linear_velocity = mul_add(state_b.linear_velocity, m_b, p);
        states[joint.index_b as usize] = state_b;
    }
}

/// (b3SolveMoverJoint)
pub fn solve_mover_joint(base: &mut JointSim, context: &StepContext, states: &mut [BodyState]) {
    debug_assert!(base.type_ == JointType::Mover);

    let m_a = base.inv_mass_a;
    let m_b = base.inv_mass_b;

    let joint = base.mover_mut();

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
    let mut v_b = state_b.linear_velocity;

    let max_force = joint.max_force;
    let vr = sub(sub(v_b, v_a), joint.linear_velocity);
    let cdot = Vec2 {
        x: dot(vr, joint.axis_x),
        y: dot(vr, joint.axis_z),
    };
    let mut delta_impulse = mul_sv2(-joint.linear_mass, cdot);
    let mut new_impulse = Vec2 {
        x: joint.impulse.x + delta_impulse.x,
        y: joint.impulse.y + delta_impulse.y,
    };

    let max_impulse = context.h * max_force;

    let length_squared = dot2(new_impulse, new_impulse);
    if length_squared > max_impulse * max_impulse {
        let scale = max_impulse / length_squared.sqrt();
        new_impulse.x *= scale;
        new_impulse.y *= scale;
    }

    delta_impulse = sub2(new_impulse, joint.impulse);
    joint.impulse = new_impulse;

    let p = blend2(delta_impulse.x, joint.axis_x, delta_impulse.y, joint.axis_z);
    v_a = mul_sub(v_a, m_a, p);
    v_b = mul_add(v_b, m_b, p);

    if state_a.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_a.linear_velocity = v_a;
        states[joint.index_a as usize] = state_a;
    }

    if state_b.flags & body_flags::DYNAMIC_FLAG != 0 {
        state_b.linear_velocity = v_b;
        states[joint.index_b as usize] = state_b;
    }
}

/// (static b3LerpColor)
fn lerp_color(a: HexColor, b: HexColor, t: f32) -> HexColor {
    let mut result = 0u32;
    let mut shift = 0;
    while shift < 24 {
        let ca = ((a.0 >> shift) & 0xFF) as f32;
        let cb = ((b.0 >> shift) & 0xFF) as f32;
        let c = (lerp_float(ca, cb, t) + 0.5) as u32;
        result |= c << shift;
        shift += 8;
    }
    HexColor(result)
}

/// (static b3DrawArrow)
fn draw_arrow(
    draw: &mut dyn DebugDraw,
    a: Pos,
    b: Pos,
    plane_normal: Vec3,
    size: f32,
    color: HexColor,
) {
    draw.draw_segment(a, b, color);

    let shaft = sub_pos(b, a);
    let shaft_len = length(shaft);
    if shaft_len < 1e-6 {
        return;
    }

    let dir = Vec3 {
        x: shaft.x / shaft_len,
        y: shaft.y / shaft_len,
        z: shaft.z / shaft_len,
    };
    let head_len = size;

    let perp = normalize(cross(plane_normal, dir));
    let back_from_tip = mul_sv(-head_len, dir);
    let side_step = mul_sv(head_len * 0.5, perp);
    let tip1 = offset_pos(b, add(back_from_tip, side_step));
    let tip2 = offset_pos(b, sub(back_from_tip, side_step));
    draw.draw_segment(b, tip1, color);
    draw.draw_segment(b, tip2, color);
}

/// (b3DrawMoverJoint)
pub(super) fn draw_mover_joint(
    draw: &mut dyn DebugDraw,
    world: &World,
    base: &JointSim,
    transform_a: WorldTransform,
    transform_b: WorldTransform,
) {
    debug_assert!(base.type_ == JointType::Mover);
    let _ = transform_a;

    let joint = base.mover();

    let frame_b = mul_world_transforms(transform_b, base.local_frame_b);
    let matrix_b = make_matrix_from_quat(frame_b.q);
    let axis_x = matrix_b.cx;
    let axis_z = matrix_b.cz;

    let awake_states = &world.solver_sets[AWAKE_SET as usize].body_states;
    let v_a = match get_body_state_index(world, base.body_id_a) {
        Some(index) => awake_states[index as usize].linear_velocity,
        None => VEC3_ZERO,
    };
    let v_b = match get_body_state_index(world, base.body_id_b) {
        Some(index) => awake_states[index as usize].linear_velocity,
        None => VEC3_ZERO,
    };

    let velocity_error = sub(sub(v_b, v_a), joint.linear_velocity);
    let planar_velocity_error = Vec2 {
        x: dot(velocity_error, axis_x),
        y: dot(velocity_error, axis_z),
    };
    let target_velocity = Vec2 {
        x: dot(joint.linear_velocity, axis_x),
        y: dot(joint.linear_velocity, axis_z),
    };

    // Colorize ring by ratio of velocity error magnitude to target speed.
    let mut color = HexColor::WHITE;
    let target_speed = length2(target_velocity);
    if target_speed > 0.0 {
        let t = clamp_float(length2(planar_velocity_error) / target_speed, 0.0, 1.0);
        color = lerp_color(HexColor::WHITE, HexColor::BLUE, t);
    }

    let radius = draw.joint_scale();
    let rx = mul_sv(radius, axis_x);
    let rz = mul_sv(radius, axis_z);
    let segment_count = 32;
    let cs = compute_cos_sin(2.0 * PI / segment_count as f32);
    let mut x1 = 1.0f32;
    let mut y1 = 0.0f32;
    let mut p1 = offset_pos(frame_b.p, rx);
    for _ in 0..segment_count {
        let x2 = cs.cosine * x1 - cs.sine * y1;
        let y2 = cs.sine * x1 + cs.cosine * y1;
        let p2 = offset_pos(frame_b.p, blend2(x2, rx, y2, rz));
        draw.draw_segment(p1, p2, color);
        x1 = x2;
        y1 = y2;
        p1 = p2;
    }

    if joint.max_force > 0.0 {
        let force = blend2(joint.impulse.x, rx, joint.impulse.y, rz);
        let dir = normalize(force);
        let origin = offset_pos(frame_b.p, mul_sv(-radius, dir));
        let scale = world.inv_h / joint.max_force;
        let tip = offset_pos(origin, mul_sv(scale, force));
        let joint_scale = draw.joint_scale();
        draw_arrow(
            draw,
            origin,
            tip,
            matrix_b.cy,
            0.1 * joint_scale,
            HexColor::WHITE,
        );
    }
}
