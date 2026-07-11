//! Ragdoll bone/human types from `shared/human.h`.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::id::{BodyId, JointId, NULL_BODY_ID, NULL_JOINT_ID};
use crate::joint::JointType;
use crate::math_functions::{Transform, Vec2, TRANSFORM_IDENTITY, VEC2_ZERO};

/// Bone indices in the ragdoll hierarchy. (BoneId)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(usize)]
pub enum BoneId {
    Pelvis = 0,
    Spine01 = 1,
    Spine02 = 2,
    Spine03 = 3,
    Neck = 4,
    Head = 5,
    ThighL = 6,
    CalfL = 7,
    ThighR = 8,
    CalfR = 9,
    UpperArmL = 10,
    LowerArmL = 11,
    UpperArmR = 12,
    LowerArmR = 13,
}

/// Number of bones in a human ragdoll. (bone_count)
pub const BONE_COUNT: usize = 14;

/// Max filter joints that disable selected bone-bone collisions. (FILTER_JOINT_COUNT)
pub const FILTER_JOINT_COUNT: usize = 8;

/// Hex colors used when `colorize` is true. (b3HexColor / types.h)
pub const COLOR_MEDIUM_TURQUOISE: u32 = 0x48D1CC;
pub const COLOR_DODGER_BLUE: u32 = 0x1E90FF;
pub const COLOR_NAVAJO_WHITE: u32 = 0xFFDEAD;
pub const COLOR_LIGHT_YELLOW: u32 = 0xFFFFE0;
pub const COLOR_PERU: u32 = 0xCD853F;
pub const COLOR_TAN: u32 = 0xD2B48C;

/// One bone of a human ragdoll. (Bone)
#[derive(Debug, Clone, Copy)]
pub struct Bone {
    pub body_id: BodyId,
    pub joint_id: JointId,
    pub anchor_id: BodyId,
    pub anchor_joint_id: JointId,
    pub local_frame_a: Transform,
    pub local_frame_b: Transform,
    pub reference_frame: Transform,
    pub joint_type: JointType,
    pub swing_limit: f32,
    pub twist_limit: Vec2,
    pub joint_friction: f32,
    pub parent_index: i32,
}

impl Default for Bone {
    fn default() -> Self {
        Bone {
            body_id: NULL_BODY_ID,
            joint_id: NULL_JOINT_ID,
            anchor_id: NULL_BODY_ID,
            anchor_joint_id: NULL_JOINT_ID,
            local_frame_a: TRANSFORM_IDENTITY,
            local_frame_b: TRANSFORM_IDENTITY,
            reference_frame: TRANSFORM_IDENTITY,
            joint_type: JointType::Parallel,
            swing_limit: 0.0,
            twist_limit: VEC2_ZERO,
            joint_friction: 1.0,
            parent_index: -1,
        }
    }
}

/// Zero-initialized human ragdoll. Must call [`super::create_human`] to spawn.
/// (Human)
#[derive(Debug, Clone)]
pub struct Human {
    pub bones: [Bone; BONE_COUNT],
    pub filter_joints: [JointId; FILTER_JOINT_COUNT],
    pub filter_joint_count: i32,
    pub friction_torque: f32,
    pub is_spawned: bool,
}

impl Default for Human {
    fn default() -> Self {
        Human {
            bones: [Bone::default(); BONE_COUNT],
            filter_joints: [NULL_JOINT_ID; FILTER_JOINT_COUNT],
            filter_joint_count: 0,
            friction_torque: 0.0,
            is_spawned: false,
        }
    }
}
