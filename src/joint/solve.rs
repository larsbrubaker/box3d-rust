//! Joint solver stage dispatch: prepare / warm-start / solve / reaction.
//! Port of b3PrepareJoint, b3WarmStartJoint, b3SolveJoint, b3GetJointReaction
//! from box3d-cpp-reference/src/joint.c.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{
    prepare_distance_joint, prepare_motor_joint, prepare_parallel_joint, prepare_prismatic_joint,
    prepare_revolute_joint, prepare_spherical_joint, prepare_weld_joint, prepare_wheel_joint,
    solve_distance_joint, solve_motor_joint, solve_parallel_joint, solve_prismatic_joint,
    solve_revolute_joint, solve_spherical_joint, solve_weld_joint, solve_wheel_joint,
    warm_start_distance_joint, warm_start_motor_joint, warm_start_parallel_joint,
    warm_start_prismatic_joint, warm_start_revolute_joint, warm_start_spherical_joint,
    warm_start_weld_joint, warm_start_wheel_joint, JointSim, JointType,
};
use crate::body::{get_body_transform, BodyState};
use crate::math_functions::{
    abs_float, add, cross, length, min_float, mul_add, mul_quat, mul_sv, normalize, rotate_vector,
    sub, Matrix3, Vec3, VEC3_AXIS_Z, VEC3_ZERO,
};
use crate::solver::{make_soft, StepContext};
use crate::world::World;

/// Clamp constraint hertz and dispatch prepare. (b3PrepareJoint)
pub fn prepare_joint(world: &World, base: &mut JointSim, context: &StepContext) {
    // Clamp joint hertz based on the time step to reduce jitter.
    let hertz = min_float(base.constraint_hertz, 0.25 * context.inv_h);
    base.constraint_softness = make_soft(hertz, base.constraint_damping_ratio, context.h);

    match base.type_ {
        JointType::Parallel => prepare_parallel_joint(world, base, context),
        JointType::Distance => prepare_distance_joint(world, base, context),
        JointType::Filter => {}
        JointType::Motor => prepare_motor_joint(world, base, context),
        JointType::Prismatic => prepare_prismatic_joint(world, base, context),
        JointType::Revolute => prepare_revolute_joint(world, base, context),
        JointType::Spherical => prepare_spherical_joint(world, base, context),
        JointType::Weld => prepare_weld_joint(world, base, context),
        JointType::Wheel => prepare_wheel_joint(world, base, context),
    }
}

/// (b3WarmStartJoint)
pub fn warm_start_joint(base: &mut JointSim, states: &mut [BodyState]) {
    match base.type_ {
        JointType::Parallel => warm_start_parallel_joint(base, states),
        JointType::Distance => warm_start_distance_joint(base, states),
        JointType::Filter => {}
        JointType::Motor => warm_start_motor_joint(base, states),
        JointType::Prismatic => warm_start_prismatic_joint(base, states),
        JointType::Revolute => warm_start_revolute_joint(base, states),
        JointType::Spherical => warm_start_spherical_joint(base, states),
        JointType::Weld => warm_start_weld_joint(base, states),
        JointType::Wheel => warm_start_wheel_joint(base, states),
    }
}

/// (b3SolveJoint)
///
/// Motor and parallel ignore `use_bias` like C (`B3_UNUSED(useBias)` at the
/// top of b3SolveJoint; those callees have no useBias parameter).
pub fn solve_joint(
    base: &mut JointSim,
    context: &StepContext,
    states: &mut [BodyState],
    use_bias: bool,
) {
    let _ = use_bias;

    match base.type_ {
        JointType::Parallel => solve_parallel_joint(base, context, states),
        JointType::Distance => solve_distance_joint(base, context, states, use_bias),
        JointType::Filter => {}
        JointType::Motor => solve_motor_joint(base, context, states),
        JointType::Prismatic => solve_prismatic_joint(base, context, states, use_bias),
        JointType::Revolute => solve_revolute_joint(base, context, states, use_bias),
        JointType::Spherical => solve_spherical_joint(base, context, states, use_bias),
        JointType::Weld => solve_weld_joint(base, context, states, use_bias),
        JointType::Wheel => solve_wheel_joint(base, context, states, use_bias),
    }
}

/// Scalar force/torque magnitudes from accumulated impulses. (b3GetJointReaction)
pub fn get_joint_reaction(world: &World, sim: &JointSim, inv_time_step: f32) -> (f32, f32) {
    let mut linear_impulse = 0.0f32;
    let mut angular_impulse = 0.0f32;

    match sim.type_ {
        JointType::Parallel => {
            let joint = sim.parallel();
            let impulse = Vec3 {
                x: joint.perp_impulse.x,
                y: joint.perp_impulse.y,
                z: 0.0,
            };
            angular_impulse = length(impulse);
        }
        JointType::Distance => {
            let joint = sim.distance();
            linear_impulse = abs_float(
                joint.impulse + joint.lower_impulse - joint.upper_impulse + joint.motor_impulse,
            );
        }
        JointType::Motor => {
            let joint = sim.motor();
            linear_impulse = length(add(
                joint.linear_velocity_impulse,
                joint.linear_spring_impulse,
            ));
            angular_impulse = length(add(
                joint.angular_velocity_impulse,
                joint.angular_spring_impulse,
            ));
        }
        JointType::Prismatic => {
            let joint = sim.prismatic();
            let impulse = Vec3 {
                x: joint.motor_impulse + joint.lower_impulse - joint.upper_impulse,
                y: joint.perp_impulse.x,
                z: joint.perp_impulse.y,
            };
            linear_impulse = length(impulse);
            angular_impulse = length(joint.angular_impulse);
        }
        JointType::Revolute => {
            let joint = sim.revolute();
            linear_impulse = length(joint.linear_impulse);
            let impulse = Vec3 {
                x: joint.perp_impulse.x,
                y: joint.perp_impulse.y,
                z: joint.motor_impulse + joint.lower_impulse - joint.upper_impulse,
            };
            angular_impulse = length(impulse);
        }
        JointType::Spherical => {
            let joint = sim.spherical();
            linear_impulse = length(joint.linear_impulse);

            let xf_a = get_body_transform(world, sim.body_id_a);
            let xf_b = get_body_transform(world, sim.body_id_b);
            let q_a = mul_quat(xf_a.q, sim.local_frame_a.q);
            let q_b = mul_quat(xf_b.q, sim.local_frame_b.q);

            // Cone axis is the z-axis of body A.
            let cone_axis = rotate_vector(q_a, VEC3_AXIS_Z);
            let twist_axis = rotate_vector(q_b, VEC3_AXIS_Z);
            let swing_axis = normalize(cross(cone_axis, twist_axis));

            let mut impulse = add(joint.spring_impulse, joint.motor_impulse);
            impulse = mul_add(
                impulse,
                joint.lower_twist_impulse - joint.upper_twist_impulse,
                twist_axis,
            );
            impulse = mul_add(impulse, joint.swing_impulse, swing_axis);

            angular_impulse = length(impulse);
        }
        JointType::Weld => {
            let joint = sim.weld();
            linear_impulse = length(joint.linear_impulse);
            angular_impulse = length(joint.angular_impulse);
        }
        JointType::Wheel => {
            // todo probably wrong (matches C comment)
            let joint = sim.wheel();
            let perp_impulse = joint.linear_impulse;
            let axial_impulse = joint.suspension_spring_impulse + joint.lower_suspension_impulse
                - joint.upper_suspension_impulse;
            linear_impulse = (perp_impulse.x * perp_impulse.x
                + perp_impulse.y * perp_impulse.y
                + axial_impulse * axial_impulse)
                .sqrt();
            angular_impulse = abs_float(joint.spin_impulse);
        }
        JointType::Filter => {}
    }

    (
        linear_impulse * inv_time_step,
        angular_impulse * inv_time_step,
    )
}

/// Solve the 3x3 point-to-point constraint `K * b = rhs` without forming the
/// effective mass matrix through skew products.
/// K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
/// Returns zero if K is singular. (b3SolvePointConstraint, an inline in joint.h)
#[allow(clippy::too_many_arguments)]
#[inline]
pub fn solve_point_constraint(
    m_a: f32,
    m_b: f32,
    i_a: Matrix3,
    i_b: Matrix3,
    r_a: Vec3,
    r_b: Vec3,
    rhs: Vec3,
) -> Vec3 {
    let axy = r_a.z * i_a.cy.y - r_a.y * i_a.cz.y;
    let axz = r_a.z * i_a.cy.z - r_a.y * i_a.cz.z;
    let ay = sub(mul_sv(r_a.x, i_a.cz), mul_sv(r_a.z, i_a.cx));
    let az = sub(mul_sv(r_a.y, i_a.cx), mul_sv(r_a.x, i_a.cy));

    let bxy = r_b.z * i_b.cy.y - r_b.y * i_b.cz.y;
    let bxz = r_b.z * i_b.cy.z - r_b.y * i_b.cz.z;
    let by = sub(mul_sv(r_b.x, i_b.cz), mul_sv(r_b.z, i_b.cx));
    let bz = sub(mul_sv(r_b.y, i_b.cx), mul_sv(r_b.x, i_b.cy));

    let m = m_a + m_b;
    let kxx = (axy * r_a.z - axz * r_a.y) + (bxy * r_b.z - bxz * r_b.y) + m;
    let kyy = (ay.z * r_a.x - ay.x * r_a.z) + (by.z * r_b.x - by.x * r_b.z) + m;
    let kzz = (az.x * r_a.y - az.y * r_a.x) + (bz.x * r_b.y - bz.y * r_b.x) + m;
    let kxy = (ay.y * r_a.z - ay.z * r_a.y) + (by.y * r_b.z - by.z * r_b.y);
    let kxz = (az.y * r_a.z - az.z * r_a.y) + (bz.y * r_b.z - bz.z * r_b.y);
    let kyz = (az.z * r_a.x - az.x * r_a.z) + (bz.z * r_b.x - bz.x * r_b.z);

    let cxx = kyy * kzz - kyz * kyz;
    let cyy = kxx * kzz - kxz * kxz;
    let czz = kxx * kyy - kxy * kxy;
    let cxy = kxz * kyz - kxy * kzz;
    let cxz = kxy * kyz - kyy * kxz;
    let cyz = kxy * kxz - kxx * kyz;

    let det = kxx * cxx + kxy * cxy + kxz * cxz;
    if abs_float(det) > 1000.0 * f32::MIN_POSITIVE {
        let inv_det = 1.0 / det;
        return Vec3 {
            x: inv_det * (cxx * rhs.x + cxy * rhs.y + cxz * rhs.z),
            y: inv_det * (cxy * rhs.x + cyy * rhs.y + cyz * rhs.z),
            z: inv_det * (cxz * rhs.x + cyz * rhs.y + czz * rhs.z),
        };
    }

    VEC3_ZERO
}
