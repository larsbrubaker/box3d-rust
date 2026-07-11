//! Human ragdoll runtime API. Port of DestroyHuman / Human_* from `shared/human.c`.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::random::random_vec3;
use super::types::{BoneId, Human, BONE_COUNT};
use crate::body::{
    body_apply_angular_impulse, body_get_transform, body_set_bullet, body_set_linear_velocity,
    create_body, destroy_body,
};
use crate::id::{NULL_BODY_ID, NULL_JOINT_ID};
use crate::joint::{
    create_motor_joint, create_parallel_joint, destroy_joint, revolute_joint_set_max_motor_torque,
    revolute_joint_set_spring_damping_ratio, revolute_joint_set_spring_hertz,
    spherical_joint_set_max_motor_torque, spherical_joint_set_spring_damping_ratio,
    spherical_joint_set_spring_hertz, JointType,
};
use crate::math_functions::{
    compute_quat_between_unit_vectors, inv_mul_quat, neg, Vec3, VEC3_AXIS_Y, VEC3_AXIS_Z,
};
use crate::types::{
    default_body_def, default_motor_joint_def, default_parallel_joint_def, BodyType,
};
use crate::world::World;
use crate::BodyId;

/// Destroy joints then bodies. Safe to skip when the world itself is destroyed.
/// (DestroyHuman)
pub fn destroy_human(human: &mut Human, world: &mut World) {
    debug_assert!(human.is_spawned);

    for i in 0..human.filter_joint_count as usize {
        destroy_joint(world, human.filter_joints[i], false);
        human.filter_joints[i] = NULL_JOINT_ID;
    }

    for i in 0..BONE_COUNT {
        if human.bones[i].joint_id.is_null() {
            continue;
        }
        destroy_joint(world, human.bones[i].joint_id, false);
        human.bones[i].joint_id = NULL_JOINT_ID;
    }

    for i in 0..BONE_COUNT {
        if human.bones[i].body_id.is_null() {
            continue;
        }
        destroy_body(world, human.bones[i].body_id);
        human.bones[i].body_id = NULL_BODY_ID;
    }

    human.is_spawned = false;
}

/// Set linear velocity on every bone body. (Human_SetVelocity)
pub fn human_set_velocity(human: &Human, world: &mut World, velocity: Vec3) {
    for i in 0..BONE_COUNT {
        let body_id = human.bones[i].body_id;
        if body_id.is_null() {
            continue;
        }
        body_set_linear_velocity(world, body_id, velocity);
    }
}

/// Apply a random angular impulse to spine_01. (Human_ApplyRandomAngularImpulse)
pub fn human_apply_random_angular_impulse(human: &Human, world: &mut World, magnitude: f32) {
    debug_assert!(human.is_spawned);
    let range = Vec3 {
        x: magnitude,
        y: magnitude,
        z: magnitude,
    };
    let impulse = random_vec3(neg(range), range);
    body_apply_angular_impulse(
        world,
        human.bones[BoneId::Spine01 as usize].body_id,
        impulse,
        true,
    );
}

/// Scale max motor torque on every bone joint. (Human_SetJointFrictionTorque)
pub fn human_set_joint_friction_torque(human: &mut Human, world: &mut World, torque: f32) {
    debug_assert!(human.is_spawned);
    human.friction_torque = torque;

    for i in 1..BONE_COUNT {
        let bone = &human.bones[i];
        if bone.joint_type == JointType::Revolute {
            revolute_joint_set_max_motor_torque(world, bone.joint_id, bone.joint_friction * torque);
        } else {
            spherical_joint_set_max_motor_torque(
                world,
                bone.joint_id,
                bone.joint_friction * torque,
            );
        }
    }
}

/// Set spring hertz on every bone joint. (Human_SetJointSpringHertz)
pub fn human_set_joint_spring_hertz(human: &Human, world: &mut World, hertz: f32) {
    debug_assert!(human.is_spawned);
    for i in 1..BONE_COUNT {
        let bone = &human.bones[i];
        if bone.joint_type == JointType::Revolute {
            revolute_joint_set_spring_hertz(world, bone.joint_id, hertz);
        } else {
            spherical_joint_set_spring_hertz(world, bone.joint_id, hertz);
        }
    }
}

/// Set spring damping ratio on every bone joint. (Human_SetJointDampingRatio)
pub fn human_set_joint_damping_ratio(human: &Human, world: &mut World, damping_ratio: f32) {
    debug_assert!(human.is_spawned);
    for i in 1..BONE_COUNT {
        let bone = &human.bones[i];
        if bone.joint_type == JointType::Revolute {
            revolute_joint_set_spring_damping_ratio(world, bone.joint_id, damping_ratio);
        } else {
            spherical_joint_set_spring_damping_ratio(world, bone.joint_id, damping_ratio);
        }
    }
}

/// Attach a parallel spring from ground to the pelvis. (Human_AlignSpring)
pub fn human_align_spring(
    human: &mut Human,
    world: &mut World,
    ground_id: BodyId,
    hertz: f32,
    damping_ratio: f32,
) {
    debug_assert!(human.is_spawned);

    let bone = &mut human.bones[BoneId::Pelvis as usize];
    debug_assert!(bone.joint_id.is_null());
    let q = compute_quat_between_unit_vectors(VEC3_AXIS_Z, VEC3_AXIS_Y);
    let qb = body_get_transform(world, bone.body_id).q;

    let mut joint_def = default_parallel_joint_def();
    joint_def.base.body_id_a = ground_id;
    joint_def.base.body_id_b = bone.body_id;
    joint_def.base.local_frame_a.q = q;
    joint_def.base.local_frame_b.q = inv_mul_quat(qb, q);
    joint_def.base.draw_scale = 2.0;
    joint_def.base.collide_connected = true;
    joint_def.hertz = hertz;
    joint_def.damping_ratio = damping_ratio;

    bone.joint_id = create_parallel_joint(world, &joint_def);
}

/// Create kinematic motor anchors for every bone. (Human_CreateMotorAnchors)
pub fn human_create_motor_anchors(human: &mut Human, world: &mut World) {
    let mut anchor_def = default_body_def();
    anchor_def.type_ = BodyType::Kinematic;

    let mut motor_def = default_motor_joint_def();
    motor_def.angular_hertz = 5.0;
    motor_def.angular_damping_ratio = 1.0;
    motor_def.linear_hertz = 5.0;
    motor_def.linear_damping_ratio = 1.0;
    motor_def.max_spring_force = f32::MAX;
    motor_def.max_spring_torque = f32::MAX;

    for i in 0..BONE_COUNT {
        let bone = &mut human.bones[i];
        let body_transform = body_get_transform(world, bone.body_id);
        anchor_def.position = body_transform.p;
        anchor_def.rotation = body_transform.q;
        bone.anchor_id = create_body(world, &anchor_def);

        motor_def.base.body_id_a = bone.anchor_id;
        motor_def.base.body_id_b = bone.body_id;
        bone.anchor_joint_id = create_motor_joint(world, &motor_def);
    }
}

/// Create kinematic parallel-joint anchors for every bone. (Human_CreateParallelAnchors)
pub fn human_create_parallel_anchors(human: &mut Human, world: &mut World) {
    let mut anchor_def = default_body_def();
    anchor_def.type_ = BodyType::Kinematic;

    let q_frame_world = compute_quat_between_unit_vectors(VEC3_AXIS_Z, VEC3_AXIS_Y);
    let mut joint_def = default_parallel_joint_def();
    joint_def.hertz = 8.0;
    joint_def.damping_ratio = 1.0;
    joint_def.max_torque = 800.0;

    for i in 0..BONE_COUNT {
        let bone = &mut human.bones[i];
        let body_transform = body_get_transform(world, bone.body_id);
        anchor_def.position = body_transform.p;
        anchor_def.rotation = body_transform.q;
        bone.anchor_id = create_body(world, &anchor_def);

        joint_def.base.body_id_a = bone.anchor_id;
        joint_def.base.body_id_b = bone.body_id;

        let frame_quat = inv_mul_quat(body_transform.q, q_frame_world);
        joint_def.base.local_frame_a.q = frame_quat;
        joint_def.base.local_frame_b.q = frame_quat;

        bone.anchor_joint_id = create_parallel_joint(world, &joint_def);
    }
}

/// Toggle bullet CCD on every bone body. (Human_SetBullet)
pub fn human_set_bullet(human: &Human, world: &mut World, flag: bool) {
    for i in 0..BONE_COUNT {
        body_set_bullet(world, human.bones[i].body_id, flag);
    }
}
