//! Generic `b3Joint_*` public API from joint.c (tuning, frames, userdata,
//! wake, constraint force/torque, separation).
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{
    get_distance_joint_force, get_joint_full_id, get_joint_sim, get_joint_sim_ref,
    get_motor_joint_force, get_motor_joint_torque, get_parallel_joint_torque,
    get_prismatic_joint_force, get_prismatic_joint_torque, get_revolute_joint_force,
    get_revolute_joint_torque, get_spherical_joint_force, get_spherical_joint_torque,
    get_weld_joint_force, get_weld_joint_torque, get_wheel_joint_force, get_wheel_joint_torque,
    JointSim, JointType,
};
use crate::body::{get_body_transform, wake_body};
use crate::core::NULL_INDEX;
use crate::id::{JointId, WorldId};
use crate::math_functions::{
    abs_float, dot, get_quat_angle, get_swing_angle, get_twist_angle, inv_mul_quat, is_valid_float,
    is_valid_transform, length, max_float, perp, rotate_vector, sub_pos, transform_world_point,
    Transform, Vec3, VEC3_AXIS_X, VEC3_ZERO,
};
use crate::world::World;

/// (b3Joint_SetConstraintTuning)
pub fn joint_set_constraint_tuning(
    world: &mut World,
    joint_id: JointId,
    hertz: f32,
    damping_ratio: f32,
) {
    debug_assert!(is_valid_float(hertz) && hertz >= 0.0);
    debug_assert!(is_valid_float(damping_ratio) && damping_ratio >= 0.0);

    let id = get_joint_full_id(world, joint_id);
    let base = get_joint_sim(world, id);
    base.constraint_hertz = hertz;
    base.constraint_damping_ratio = damping_ratio;
}

/// (b3Joint_GetConstraintTuning)
pub fn joint_get_constraint_tuning(world: &World, joint_id: JointId) -> (f32, f32) {
    let id = get_joint_full_id(world, joint_id);
    let base = get_joint_sim_ref(world, id);
    (base.constraint_hertz, base.constraint_damping_ratio)
}

/// (b3Joint_SetForceThreshold)
pub fn joint_set_force_threshold(world: &mut World, joint_id: JointId, threshold: f32) {
    debug_assert!(is_valid_float(threshold) && threshold >= 0.0);

    let id = get_joint_full_id(world, joint_id);
    get_joint_sim(world, id).force_threshold = threshold;
}

/// (b3Joint_GetForceThreshold)
pub fn joint_get_force_threshold(world: &World, joint_id: JointId) -> f32 {
    let id = get_joint_full_id(world, joint_id);
    get_joint_sim_ref(world, id).force_threshold
}

/// (b3Joint_SetTorqueThreshold)
pub fn joint_set_torque_threshold(world: &mut World, joint_id: JointId, threshold: f32) {
    debug_assert!(is_valid_float(threshold) && threshold >= 0.0);

    let id = get_joint_full_id(world, joint_id);
    get_joint_sim(world, id).torque_threshold = threshold;
}

/// (b3Joint_GetTorqueThreshold)
pub fn joint_get_torque_threshold(world: &World, joint_id: JointId) -> f32 {
    let id = get_joint_full_id(world, joint_id);
    get_joint_sim_ref(world, id).torque_threshold
}

/// (b3Joint_GetWorld)
pub fn joint_get_world(world: &World, joint_id: JointId) -> WorldId {
    let _ = get_joint_full_id(world, joint_id);
    WorldId {
        index1: joint_id.world0.wrapping_add(1),
        generation: world.generation,
    }
}

/// (b3Joint_SetLocalFrameA)
pub fn joint_set_local_frame_a(world: &mut World, joint_id: JointId, local_frame: Transform) {
    debug_assert!(is_valid_transform(local_frame));

    let id = get_joint_full_id(world, joint_id);
    get_joint_sim(world, id).local_frame_a = local_frame;
}

/// (b3Joint_GetLocalFrameA)
pub fn joint_get_local_frame_a(world: &World, joint_id: JointId) -> Transform {
    let id = get_joint_full_id(world, joint_id);
    get_joint_sim_ref(world, id).local_frame_a
}

/// (b3Joint_SetLocalFrameB)
pub fn joint_set_local_frame_b(world: &mut World, joint_id: JointId, local_frame: Transform) {
    debug_assert!(is_valid_transform(local_frame));

    let id = get_joint_full_id(world, joint_id);
    get_joint_sim(world, id).local_frame_b = local_frame;
}

/// (b3Joint_GetLocalFrameB)
pub fn joint_get_local_frame_b(world: &World, joint_id: JointId) -> Transform {
    let id = get_joint_full_id(world, joint_id);
    get_joint_sim_ref(world, id).local_frame_b
}

/// (b3Joint_SetUserData) — Rust stores `u64` instead of `void*`.
pub fn joint_set_user_data(world: &mut World, joint_id: JointId, user_data: u64) {
    let id = get_joint_full_id(world, joint_id);
    world.joints[id as usize].user_data = user_data;
}

/// (b3Joint_GetUserData)
pub fn joint_get_user_data(world: &World, joint_id: JointId) -> u64 {
    let id = get_joint_full_id(world, joint_id);
    world.joints[id as usize].user_data
}

/// (b3Joint_WakeBodies)
pub fn joint_wake_bodies(world: &mut World, joint_id: JointId) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    world.locked = true;

    let id = get_joint_full_id(world, joint_id);
    let body_id_a = world.joints[id as usize].edges[0].body_id;
    let body_id_b = world.joints[id as usize].edges[1].body_id;

    wake_body(world, body_id_a);
    wake_body(world, body_id_b);

    world.locked = false;
}

/// Copy of the joint sim so force helpers can also borrow `&World`.
fn joint_sim_copy(world: &World, joint_id: i32) -> JointSim {
    *get_joint_sim_ref(world, joint_id)
}

/// (b3GetJointConstraintForce / b3Joint_GetConstraintForce)
pub fn joint_get_constraint_force(world: &World, joint_id: JointId) -> Vec3 {
    let id = get_joint_full_id(world, joint_id);
    let type_ = world.joints[id as usize].type_;
    let base = joint_sim_copy(world, id);

    match type_ {
        JointType::Parallel => VEC3_ZERO,
        JointType::Distance => get_distance_joint_force(world, &base),
        JointType::Filter => VEC3_ZERO,
        JointType::Motor => get_motor_joint_force(world, &base),
        JointType::Prismatic => get_prismatic_joint_force(world, &base),
        JointType::Revolute => get_revolute_joint_force(world, &base),
        JointType::Spherical => get_spherical_joint_force(world, &base),
        JointType::Weld => get_weld_joint_force(world, &base),
        JointType::Wheel => get_wheel_joint_force(world, &base),
    }
}

/// (b3GetJointConstraintTorque / b3Joint_GetConstraintTorque)
pub fn joint_get_constraint_torque(world: &mut World, joint_id: JointId) -> Vec3 {
    let id = get_joint_full_id(world, joint_id);
    let type_ = world.joints[id as usize].type_;

    match type_ {
        JointType::Parallel => {
            let inv_h = world.inv_h;
            let mut base = joint_sim_copy(world, id);
            get_parallel_joint_torque(inv_h, &mut base)
        }
        JointType::Distance | JointType::Filter => VEC3_ZERO,
        JointType::Motor => {
            let base = joint_sim_copy(world, id);
            get_motor_joint_torque(world, &base)
        }
        JointType::Prismatic => {
            let base = joint_sim_copy(world, id);
            get_prismatic_joint_torque(world, &base)
        }
        JointType::Revolute => {
            // C recomputes perp axes as a side effect — write back.
            with_joint_sim_mut(world, id, |world, base| {
                get_revolute_joint_torque(world, base)
            })
        }
        JointType::Spherical => {
            let base = joint_sim_copy(world, id);
            get_spherical_joint_torque(world, &base)
        }
        JointType::Weld => {
            let base = joint_sim_copy(world, id);
            get_weld_joint_torque(world, &base)
        }
        JointType::Wheel => {
            let base = joint_sim_copy(world, id);
            get_wheel_joint_torque(world, &base)
        }
    }
}

fn with_joint_sim_mut<R>(
    world: &mut World,
    joint_id: i32,
    f: impl FnOnce(&World, &mut JointSim) -> R,
) -> R {
    let set_index = world.joints[joint_id as usize].set_index;
    let color_index = world.joints[joint_id as usize].color_index;
    let local_index = world.joints[joint_id as usize].local_index as usize;

    if color_index != NULL_INDEX {
        let mut sim = std::mem::take(
            &mut world.constraint_graph.colors[color_index as usize].joint_sims[local_index],
        );
        let result = f(world, &mut sim);
        world.constraint_graph.colors[color_index as usize].joint_sims[local_index] = sim;
        result
    } else {
        let mut sim =
            std::mem::take(&mut world.solver_sets[set_index as usize].joint_sims[local_index]);
        let result = f(world, &mut sim);
        world.solver_sets[set_index as usize].joint_sims[local_index] = sim;
        result
    }
}

/// (b3Joint_GetLinearSeparation)
pub fn joint_get_linear_separation(world: &World, joint_id: JointId) -> f32 {
    let id = get_joint_full_id(world, joint_id);
    let type_ = world.joints[id as usize].type_;
    let body_id_a = world.joints[id as usize].edges[0].body_id;
    let body_id_b = world.joints[id as usize].edges[1].body_id;
    let base = joint_sim_copy(world, id);

    let xf_a = get_body_transform(world, body_id_a);
    let xf_b = get_body_transform(world, body_id_b);

    let p_a = transform_world_point(xf_a, base.local_frame_a.p);
    let p_b = transform_world_point(xf_b, base.local_frame_b.p);
    let dp = sub_pos(p_b, p_a);

    match type_ {
        JointType::Parallel | JointType::Motor | JointType::Filter => 0.0,
        JointType::Distance => {
            let distance_joint = base.distance();
            let len = length(dp);
            if distance_joint.enable_spring {
                if distance_joint.enable_limit {
                    if len < distance_joint.min_length {
                        return distance_joint.min_length - len;
                    }
                    if len > distance_joint.max_length {
                        return len - distance_joint.max_length;
                    }
                    return 0.0;
                }
                return 0.0;
            }
            abs_float(len - distance_joint.length)
        }
        JointType::Prismatic => {
            let prismatic = base.prismatic();
            let axis_a = rotate_vector(xf_a.q, VEC3_AXIS_X);
            let perp_a = perp(axis_a);
            let perpendicular_separation = abs_float(dot(perp_a, dp));
            let mut limit_separation = 0.0f32;

            if prismatic.enable_limit {
                let translation = dot(axis_a, dp);
                if translation < prismatic.lower_translation {
                    limit_separation = prismatic.lower_translation - translation;
                }
                if prismatic.upper_translation < translation {
                    limit_separation = translation - prismatic.upper_translation;
                }
            }

            (perpendicular_separation * perpendicular_separation
                + limit_separation * limit_separation)
                .sqrt()
        }
        JointType::Revolute | JointType::Spherical => length(dp),
        JointType::Weld => {
            if base.weld().linear_hertz == 0.0 {
                length(dp)
            } else {
                0.0
            }
        }
        JointType::Wheel => {
            let wheel = base.wheel();
            let axis_a = rotate_vector(xf_a.q, VEC3_AXIS_X);
            let perp_a = perp(axis_a);
            let perpendicular_separation = abs_float(dot(perp_a, dp));
            let mut limit_separation = 0.0f32;

            if wheel.enable_suspension_limit {
                let translation = dot(axis_a, dp);
                if translation < wheel.lower_suspension_limit {
                    limit_separation = wheel.lower_suspension_limit - translation;
                }
                if wheel.upper_suspension_limit < translation {
                    limit_separation = translation - wheel.upper_suspension_limit;
                }
            }

            (perpendicular_separation * perpendicular_separation
                + limit_separation * limit_separation)
                .sqrt()
        }
    }
}

/// (b3Joint_GetAngularSeparation)
pub fn joint_get_angular_separation(world: &World, joint_id: JointId) -> f32 {
    let id = get_joint_full_id(world, joint_id);
    let type_ = world.joints[id as usize].type_;
    let body_id_a = world.joints[id as usize].edges[0].body_id;
    let body_id_b = world.joints[id as usize].edges[1].body_id;
    let base = joint_sim_copy(world, id);

    let xf_a = get_body_transform(world, body_id_a);
    let xf_b = get_body_transform(world, body_id_b);

    let mut rel_q = inv_mul_quat(xf_a.q, xf_b.q);

    match type_ {
        JointType::Parallel => {
            // Remove hinge angle
            rel_q.v.z = 0.0;
            get_quat_angle(rel_q)
        }
        JointType::Distance | JointType::Motor | JointType::Filter => 0.0,
        JointType::Prismatic => get_quat_angle(rel_q),
        JointType::Revolute => {
            let revolute = base.revolute();
            if revolute.enable_limit {
                let angle = get_twist_angle(rel_q);
                if angle < revolute.lower_angle || revolute.upper_angle < angle {
                    return get_quat_angle(rel_q);
                }
            }
            // Remove hinge angle
            rel_q.v.z = 0.0;
            get_quat_angle(rel_q)
        }
        JointType::Spherical => {
            let spherical = base.spherical();
            let mut sum = 0.0f32;
            if spherical.enable_cone_limit {
                let swing_angle = get_swing_angle(rel_q);
                sum += max_float(0.0, swing_angle - spherical.cone_angle);
            }
            if spherical.enable_twist_limit {
                let twist_angle = get_twist_angle(rel_q);
                sum += max_float(0.0, spherical.lower_twist_angle - twist_angle);
                sum += max_float(0.0, twist_angle - spherical.upper_twist_angle);
            }
            sum
        }
        JointType::Weld => {
            if base.weld().angular_hertz == 0.0 {
                get_quat_angle(rel_q)
            } else {
                0.0
            }
        }
        JointType::Wheel => {
            // todo in C — asserts; skip for wheel in callers
            debug_assert!(false, "wheel angular separation unimplemented in C");
            0.0
        }
    }
}
