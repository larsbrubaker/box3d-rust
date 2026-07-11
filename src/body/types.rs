// Port of the body data model from box3d-cpp-reference/src/body.h.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::NULL_INDEX;
use crate::math_functions::{
    Matrix3, Pos, Quat, Vec3, WorldTransform, MAT3_ZERO, POS_ZERO, QUAT_IDENTITY, VEC3_ZERO,
    WORLD_TRANSFORM_IDENTITY,
};
use crate::types::BodyType;

/// Body flag bits. (enum b3BodyFlags)
pub mod body_flags {
    /// Fixed translation along the x-axis
    pub const LOCK_LINEAR_X: u32 = 0x0000_0001;
    /// Fixed translation along the y-axis
    pub const LOCK_LINEAR_Y: u32 = 0x0000_0002;
    /// Fixed translation along the z-axis
    pub const LOCK_LINEAR_Z: u32 = 0x0000_0004;
    /// Fixed rotation around the x-axis
    pub const LOCK_ANGULAR_X: u32 = 0x0000_0008;
    /// Fixed rotation around the y-axis
    pub const LOCK_ANGULAR_Y: u32 = 0x0000_0010;
    /// Fixed rotation around the z-axis
    pub const LOCK_ANGULAR_Z: u32 = 0x0000_0020;
    /// Used for debug draw
    pub const IS_FAST: u32 = 0x0000_0040;
    /// Dynamic body does a final CCD pass against all body types, but not other bullets
    pub const IS_BULLET: u32 = 0x0000_0080;
    /// Speed capped in the current time step
    pub const IS_SPEED_CAPPED: u32 = 0x0000_0100;
    /// Had a time of impact event in the current time step
    pub const HAD_TIME_OF_IMPACT: u32 = 0x0000_0200;
    /// No limit on angular velocity
    pub const ALLOW_FAST_ROTATION: u32 = 0x0000_0400;
    /// Needs AABB increased
    pub const ENLARGE_BOUNDS: u32 = 0x0000_0800;
    /// Dynamic so the solver should write to it. Used for BodyState flags.
    pub const DYNAMIC_FLAG: u32 = 0x0000_1000;
    pub const ENABLE_SLEEP: u32 = 0x0000_2000;
    pub const BODY_ENABLE_CONTACT_RECYCLING: u32 = 0x0000_4000;
    /// User deferred mass computation and mass data still hasn't been set.
    pub const DIRTY_MASS: u32 = 0x0000_8000;

    /// All lock flags
    pub const ALL_LOCKS: u32 = LOCK_LINEAR_X
        | LOCK_LINEAR_Y
        | LOCK_LINEAR_Z
        | LOCK_ANGULAR_X
        | LOCK_ANGULAR_Y
        | LOCK_ANGULAR_Z;

    /// If all these flags are set then the body has fixed rotation
    pub const FIXED_ROTATION: u32 = LOCK_ANGULAR_X | LOCK_ANGULAR_Y | LOCK_ANGULAR_Z;

    /// Transient per time step. May differ across Body, BodySim, and BodyState.
    pub const BODY_TRANSIENT_FLAGS: u32 = IS_FAST | IS_SPEED_CAPPED | HAD_TIME_OF_IMPACT;
}

/// Body organizational details that are not used in the solver. (b3Body)
#[derive(Debug, Clone)]
pub struct Body {
    pub user_data: u64,

    /// Index of solver set stored in World. May be NULL_INDEX.
    pub set_index: i32,

    /// Body sim and state index within set. May be NULL_INDEX.
    pub local_index: i32,

    /// [31 : contactId | 1 : edgeIndex]
    pub head_contact_key: i32,
    pub contact_count: i32,

    pub head_shape_id: i32,
    pub shape_count: i32,

    pub head_chain_id: i32,

    /// [31 : jointId | 1 : edgeIndex]
    pub head_joint_key: i32,
    pub joint_count: i32,

    /// All enabled dynamic and kinematic bodies are in an island.
    pub island_id: i32,

    /// Index into the island's bodies array for O(1) swap-removal.
    /// NULL_INDEX when not in an island.
    pub island_index: i32,

    pub sleep_threshold: f32,
    pub sleep_time: f32,
    pub sleep_velocity: f32,
    pub mass: f32,

    /// Local space inertia
    pub inertia: Matrix3,

    /// Adjusts the fellAsleep flag in the body move array
    pub body_move_index: i32,

    pub id: i32,

    /// body_flags bits
    pub flags: u32,
    pub name_id: u32,

    pub type_: BodyType,

    /// Monotonically advanced when a body is allocated in this slot.
    /// Used to check for invalid BodyId.
    pub generation: u16,
}

impl Default for Body {
    fn default() -> Self {
        Body {
            user_data: 0,
            set_index: NULL_INDEX,
            local_index: NULL_INDEX,
            head_contact_key: NULL_INDEX,
            contact_count: 0,
            head_shape_id: NULL_INDEX,
            shape_count: 0,
            head_chain_id: NULL_INDEX,
            head_joint_key: NULL_INDEX,
            joint_count: 0,
            island_id: NULL_INDEX,
            island_index: NULL_INDEX,
            sleep_threshold: 0.0,
            sleep_time: 0.0,
            sleep_velocity: 0.0,
            mass: 0.0,
            inertia: MAT3_ZERO,
            body_move_index: NULL_INDEX,
            id: NULL_INDEX,
            flags: 0,
            name_id: 0,
            type_: BodyType::Static,
            generation: 0,
        }
    }
}

/// Body state designed for fast conversion to and from SIMD via scatter-gather.
/// Only awake dynamic and kinematic bodies have a body state. Used in the
/// performance critical constraint solver. (b3BodyState, 56 bytes)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyState {
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,

    /// Using delta position reduces round-off error far from the origin
    pub delta_position: Vec3,

    /// Delta rotation; identity for static bodies via a dummy state
    pub delta_rotation: Quat,

    /// body_flags bits — important: locking, dynamic
    pub flags: u32,
}

/// Identity body state; notice delta_rotation is identity. (b3_identityBodyState)
pub const IDENTITY_BODY_STATE: BodyState = BodyState {
    linear_velocity: VEC3_ZERO,
    angular_velocity: VEC3_ZERO,
    delta_position: VEC3_ZERO,
    delta_rotation: QUAT_IDENTITY,
    flags: 0,
};

impl Default for BodyState {
    fn default() -> Self {
        IDENTITY_BODY_STATE
    }
}

/// Body simulation data used for integration of position and velocity.
/// Transform data used for collision and solver preparation. (b3BodySim)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodySim {
    /// Transform for body origin (double translation in large world mode)
    pub transform: WorldTransform,

    /// Center of mass position in world space
    pub center: Pos,

    /// Previous rotation and COM for TOI
    pub rotation0: Quat,
    pub center0: Pos,

    /// Location of center of mass relative to the body origin
    pub local_center: Vec3,

    pub force: Vec3,
    pub torque: Vec3,

    pub inv_mass: f32,

    /// Rotational inertia about the center of mass. World inverse inertia
    /// must be updated whenever the body rotation is modified.
    pub inv_inertia_local: Matrix3,
    pub inv_inertia_world: Matrix3,

    pub min_extent: f32,
    pub max_extent: Vec3,
    pub max_angular_velocity: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub gravity_scale: f32,

    /// Index of Body
    pub body_id: i32,

    /// body_flags bits
    pub flags: u32,
}

impl Default for BodySim {
    fn default() -> Self {
        BodySim {
            transform: WORLD_TRANSFORM_IDENTITY,
            center: POS_ZERO,
            rotation0: QUAT_IDENTITY,
            center0: POS_ZERO,
            local_center: VEC3_ZERO,
            force: VEC3_ZERO,
            torque: VEC3_ZERO,
            inv_mass: 0.0,
            inv_inertia_local: MAT3_ZERO,
            inv_inertia_world: MAT3_ZERO,
            min_extent: 0.0,
            max_extent: VEC3_ZERO,
            max_angular_velocity: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            gravity_scale: 1.0,
            body_id: NULL_INDEX,
            flags: 0,
        }
    }
}

/// Body plane result for movers. (b3BodyPlaneResult)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyPlaneResult {
    /// The shape id on the body.
    pub shape_id: crate::id::ShapeId,
    /// The plane result.
    pub result: crate::geometry::PlaneResult,
}

impl Default for BodyPlaneResult {
    fn default() -> Self {
        BodyPlaneResult {
            shape_id: crate::id::NULL_SHAPE_ID,
            result: crate::geometry::PlaneResult::default(),
        }
    }
}
