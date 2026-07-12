//! Recording op table generated from recording_ops.inl.
//! Opcode constants, arg structs, and framed write helpers.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::distance::ShapeProxy;
use crate::geometry::{Capsule, MassData, Sphere, SurfaceMaterial};
use crate::id::{BodyId, JointId, ShapeId, WorldId};
use crate::math_functions::{Aabb, Pos, Quat, Transform, Vec3, WorldTransform};
use crate::types::{
    BodyDef, DistanceJointDef, ExplosionDef, Filter, FilterJointDef, MotionLocks, MotorJointDef,
    ParallelJointDef, PrismaticJointDef, QueryFilter, RevoluteJointDef, ShapeDef, SphericalJointDef,
    WeldJointDef, WheelJointDef,
};

use super::session::Recording;

/// Opcode constants from recording_ops.inl.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecOp {
    DestroyWorld = 0x01,
    Step = 0x80,
    WorldEnableSleeping = 0x02,
    WorldEnableContinuous = 0x03,
    WorldSetRestitutionThreshold = 0x04,
    WorldSetHitEventThreshold = 0x05,
    WorldSetGravity = 0x06,
    WorldExplode = 0x07,
    WorldSetContactTuning = 0x08,
    WorldSetContactRecycleDistance = 0x09,
    WorldSetMaximumLinearSpeed = 0x0A,
    WorldEnableWarmStarting = 0x0B,
    WorldRebuildStaticTree = 0x0C,
    WorldEnableSpeculative = 0x0D,
    CreateBody = 0x10,
    DestroyBody = 0x11,
    BodySetTransform = 0x20,
    BodySetLinearVelocity = 0x21,
    BodySetType = 0x22,
    BodySetName = 0x23,
    BodySetAngularVelocity = 0x24,
    BodySetTargetTransform = 0x25,
    BodyApplyForce = 0x26,
    BodyApplyForceToCenter = 0x27,
    BodyApplyTorque = 0x28,
    BodyApplyLinearImpulse = 0x29,
    BodyApplyLinearImpulseToCenter = 0x2A,
    BodyApplyAngularImpulse = 0x2B,
    BodySetMassData = 0x2C,
    BodyApplyMassFromShapes = 0x2D,
    BodySetLinearDamping = 0x2E,
    BodySetAngularDamping = 0x2F,
    BodySetGravityScale = 0x30,
    BodySetAwake = 0x31,
    BodyEnableSleep = 0x32,
    BodySetSleepThreshold = 0x33,
    BodyDisable = 0x34,
    BodyEnable = 0x35,
    BodySetMotionLocks = 0x36,
    BodySetBullet = 0x37,
    BodyEnableContactRecycling = 0x38,
    BodyEnableHitEvents = 0x39,
    CreateSphereShape = 0x40,
    CreateCapsuleShape = 0x41,
    CreateHullShape = 0x42,
    CreateMeshShape = 0x43,
    CreateHeightFieldShape = 0x44,
    CreateCompoundShape = 0x45,
    DestroyShape = 0x46,
    ShapeSetDensity = 0x50,
    ShapeSetFriction = 0x51,
    ShapeSetRestitution = 0x52,
    ShapeSetSurfaceMaterial = 0x53,
    ShapeSetFilter = 0x54,
    ShapeEnableSensorEvents = 0x55,
    ShapeEnableContactEvents = 0x56,
    ShapeEnablePreSolveEvents = 0x57,
    ShapeEnableHitEvents = 0x58,
    ShapeSetSphere = 0x59,
    ShapeSetCapsule = 0x5A,
    ShapeApplyWind = 0x5B,
    ShapeSetName = 0x5C,
    CreateParallelJoint = 0x90,
    CreateDistanceJoint = 0x91,
    CreateFilterJoint = 0x92,
    CreateMotorJoint = 0x93,
    CreatePrismaticJoint = 0x94,
    CreateRevoluteJoint = 0x95,
    CreateSphericalJoint = 0x96,
    CreateWeldJoint = 0x97,
    CreateWheelJoint = 0x98,
    DestroyJoint = 0x99,
    JointSetLocalFrameA = 0x9A,
    JointSetLocalFrameB = 0x9B,
    JointSetCollideConnected = 0x9C,
    JointWakeBodies = 0x9D,
    JointSetConstraintTuning = 0x9E,
    JointSetForceThreshold = 0x9F,
    JointSetTorqueThreshold = 0xA0,
    ParallelJointSetSpringHertz = 0xA1,
    ParallelJointSetSpringDampingRatio = 0xA2,
    ParallelJointSetMaxTorque = 0xA3,
    DistanceJointSetLength = 0xA4,
    DistanceJointEnableSpring = 0xA5,
    DistanceJointSetSpringForceRange = 0xA6,
    DistanceJointSetSpringHertz = 0xA7,
    DistanceJointSetSpringDampingRatio = 0xA8,
    DistanceJointEnableLimit = 0xA9,
    DistanceJointSetLengthRange = 0xAA,
    DistanceJointEnableMotor = 0xAB,
    DistanceJointSetMotorSpeed = 0xAC,
    DistanceJointSetMaxMotorForce = 0xAD,
    MotorJointSetLinearVelocity = 0xAE,
    MotorJointSetAngularVelocity = 0xAF,
    MotorJointSetMaxVelocityForce = 0xB0,
    MotorJointSetMaxVelocityTorque = 0xB1,
    MotorJointSetLinearHertz = 0xB2,
    MotorJointSetLinearDampingRatio = 0xB3,
    MotorJointSetAngularHertz = 0xB4,
    MotorJointSetAngularDampingRatio = 0xB5,
    MotorJointSetMaxSpringForce = 0xB6,
    MotorJointSetMaxSpringTorque = 0xB7,
    PrismaticJointEnableSpring = 0xB8,
    PrismaticJointSetSpringHertz = 0xB9,
    PrismaticJointSetSpringDampingRatio = 0xBA,
    PrismaticJointSetTargetTranslation = 0xBB,
    PrismaticJointEnableLimit = 0xBC,
    PrismaticJointSetLimits = 0xBD,
    PrismaticJointEnableMotor = 0xBE,
    PrismaticJointSetMotorSpeed = 0xBF,
    PrismaticJointSetMaxMotorForce = 0xC0,
    RevoluteJointEnableSpring = 0xC1,
    RevoluteJointSetSpringHertz = 0xC2,
    RevoluteJointSetSpringDampingRatio = 0xC3,
    RevoluteJointSetTargetAngle = 0xC4,
    RevoluteJointEnableLimit = 0xC5,
    RevoluteJointSetLimits = 0xC6,
    RevoluteJointEnableMotor = 0xC7,
    RevoluteJointSetMotorSpeed = 0xC8,
    RevoluteJointSetMaxMotorTorque = 0xC9,
    SphericalJointEnableConeLimit = 0xCA,
    SphericalJointSetConeLimit = 0xCB,
    SphericalJointEnableTwistLimit = 0xCC,
    SphericalJointSetTwistLimits = 0xCD,
    SphericalJointEnableSpring = 0xCE,
    SphericalJointSetSpringHertz = 0xCF,
    SphericalJointSetSpringDampingRatio = 0xD0,
    SphericalJointSetTargetRotation = 0xD1,
    SphericalJointEnableMotor = 0xD2,
    SphericalJointSetMotorVelocity = 0xD3,
    SphericalJointSetMaxMotorTorque = 0xD4,
    WeldJointSetLinearHertz = 0xD5,
    WeldJointSetLinearDampingRatio = 0xD6,
    WeldJointSetAngularHertz = 0xD7,
    WeldJointSetAngularDampingRatio = 0xD8,
    WheelJointEnableSuspension = 0xD9,
    WheelJointSetSuspensionHertz = 0xDA,
    WheelJointSetSuspensionDampingRatio = 0xDB,
    WheelJointEnableSuspensionLimit = 0xDC,
    WheelJointSetSuspensionLimits = 0xDD,
    WheelJointEnableSpinMotor = 0xDE,
    WheelJointSetSpinMotorSpeed = 0xDF,
    WheelJointSetMaxSpinTorque = 0xE0,
    WheelJointEnableSteering = 0xE1,
    WheelJointSetSteeringHertz = 0xE2,
    WheelJointSetSteeringDampingRatio = 0xE3,
    WheelJointSetMaxSteeringTorque = 0xE4,
    WheelJointEnableSteeringLimit = 0xE5,
    WheelJointSetSteeringLimits = 0xE6,
    WheelJointSetTargetSteeringAngle = 0xE7,
    QueryOverlapAABB = 0xE8,
    QueryOverlapShape = 0xE9,
    QueryCastRay = 0xEA,
    QueryCastShape = 0xEB,
    QueryCastRayClosest = 0xEC,
    QueryCastMover = 0xED,
    QueryCollideMover = 0xEE,
    QueryTag = 0xEF,
    StateHash = 0xF1,
    RecordingBounds = 0xF2,
}

impl RecOp {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x01 => Some(Self::DestroyWorld),
            0x80 => Some(Self::Step),
            0x02 => Some(Self::WorldEnableSleeping),
            0x03 => Some(Self::WorldEnableContinuous),
            0x04 => Some(Self::WorldSetRestitutionThreshold),
            0x05 => Some(Self::WorldSetHitEventThreshold),
            0x06 => Some(Self::WorldSetGravity),
            0x07 => Some(Self::WorldExplode),
            0x08 => Some(Self::WorldSetContactTuning),
            0x09 => Some(Self::WorldSetContactRecycleDistance),
            0x0A => Some(Self::WorldSetMaximumLinearSpeed),
            0x0B => Some(Self::WorldEnableWarmStarting),
            0x0C => Some(Self::WorldRebuildStaticTree),
            0x0D => Some(Self::WorldEnableSpeculative),
            0x10 => Some(Self::CreateBody),
            0x11 => Some(Self::DestroyBody),
            0x20 => Some(Self::BodySetTransform),
            0x21 => Some(Self::BodySetLinearVelocity),
            0x22 => Some(Self::BodySetType),
            0x23 => Some(Self::BodySetName),
            0x24 => Some(Self::BodySetAngularVelocity),
            0x25 => Some(Self::BodySetTargetTransform),
            0x26 => Some(Self::BodyApplyForce),
            0x27 => Some(Self::BodyApplyForceToCenter),
            0x28 => Some(Self::BodyApplyTorque),
            0x29 => Some(Self::BodyApplyLinearImpulse),
            0x2A => Some(Self::BodyApplyLinearImpulseToCenter),
            0x2B => Some(Self::BodyApplyAngularImpulse),
            0x2C => Some(Self::BodySetMassData),
            0x2D => Some(Self::BodyApplyMassFromShapes),
            0x2E => Some(Self::BodySetLinearDamping),
            0x2F => Some(Self::BodySetAngularDamping),
            0x30 => Some(Self::BodySetGravityScale),
            0x31 => Some(Self::BodySetAwake),
            0x32 => Some(Self::BodyEnableSleep),
            0x33 => Some(Self::BodySetSleepThreshold),
            0x34 => Some(Self::BodyDisable),
            0x35 => Some(Self::BodyEnable),
            0x36 => Some(Self::BodySetMotionLocks),
            0x37 => Some(Self::BodySetBullet),
            0x38 => Some(Self::BodyEnableContactRecycling),
            0x39 => Some(Self::BodyEnableHitEvents),
            0x40 => Some(Self::CreateSphereShape),
            0x41 => Some(Self::CreateCapsuleShape),
            0x42 => Some(Self::CreateHullShape),
            0x43 => Some(Self::CreateMeshShape),
            0x44 => Some(Self::CreateHeightFieldShape),
            0x45 => Some(Self::CreateCompoundShape),
            0x46 => Some(Self::DestroyShape),
            0x50 => Some(Self::ShapeSetDensity),
            0x51 => Some(Self::ShapeSetFriction),
            0x52 => Some(Self::ShapeSetRestitution),
            0x53 => Some(Self::ShapeSetSurfaceMaterial),
            0x54 => Some(Self::ShapeSetFilter),
            0x55 => Some(Self::ShapeEnableSensorEvents),
            0x56 => Some(Self::ShapeEnableContactEvents),
            0x57 => Some(Self::ShapeEnablePreSolveEvents),
            0x58 => Some(Self::ShapeEnableHitEvents),
            0x59 => Some(Self::ShapeSetSphere),
            0x5A => Some(Self::ShapeSetCapsule),
            0x5B => Some(Self::ShapeApplyWind),
            0x5C => Some(Self::ShapeSetName),
            0x90 => Some(Self::CreateParallelJoint),
            0x91 => Some(Self::CreateDistanceJoint),
            0x92 => Some(Self::CreateFilterJoint),
            0x93 => Some(Self::CreateMotorJoint),
            0x94 => Some(Self::CreatePrismaticJoint),
            0x95 => Some(Self::CreateRevoluteJoint),
            0x96 => Some(Self::CreateSphericalJoint),
            0x97 => Some(Self::CreateWeldJoint),
            0x98 => Some(Self::CreateWheelJoint),
            0x99 => Some(Self::DestroyJoint),
            0x9A => Some(Self::JointSetLocalFrameA),
            0x9B => Some(Self::JointSetLocalFrameB),
            0x9C => Some(Self::JointSetCollideConnected),
            0x9D => Some(Self::JointWakeBodies),
            0x9E => Some(Self::JointSetConstraintTuning),
            0x9F => Some(Self::JointSetForceThreshold),
            0xA0 => Some(Self::JointSetTorqueThreshold),
            0xA1 => Some(Self::ParallelJointSetSpringHertz),
            0xA2 => Some(Self::ParallelJointSetSpringDampingRatio),
            0xA3 => Some(Self::ParallelJointSetMaxTorque),
            0xA4 => Some(Self::DistanceJointSetLength),
            0xA5 => Some(Self::DistanceJointEnableSpring),
            0xA6 => Some(Self::DistanceJointSetSpringForceRange),
            0xA7 => Some(Self::DistanceJointSetSpringHertz),
            0xA8 => Some(Self::DistanceJointSetSpringDampingRatio),
            0xA9 => Some(Self::DistanceJointEnableLimit),
            0xAA => Some(Self::DistanceJointSetLengthRange),
            0xAB => Some(Self::DistanceJointEnableMotor),
            0xAC => Some(Self::DistanceJointSetMotorSpeed),
            0xAD => Some(Self::DistanceJointSetMaxMotorForce),
            0xAE => Some(Self::MotorJointSetLinearVelocity),
            0xAF => Some(Self::MotorJointSetAngularVelocity),
            0xB0 => Some(Self::MotorJointSetMaxVelocityForce),
            0xB1 => Some(Self::MotorJointSetMaxVelocityTorque),
            0xB2 => Some(Self::MotorJointSetLinearHertz),
            0xB3 => Some(Self::MotorJointSetLinearDampingRatio),
            0xB4 => Some(Self::MotorJointSetAngularHertz),
            0xB5 => Some(Self::MotorJointSetAngularDampingRatio),
            0xB6 => Some(Self::MotorJointSetMaxSpringForce),
            0xB7 => Some(Self::MotorJointSetMaxSpringTorque),
            0xB8 => Some(Self::PrismaticJointEnableSpring),
            0xB9 => Some(Self::PrismaticJointSetSpringHertz),
            0xBA => Some(Self::PrismaticJointSetSpringDampingRatio),
            0xBB => Some(Self::PrismaticJointSetTargetTranslation),
            0xBC => Some(Self::PrismaticJointEnableLimit),
            0xBD => Some(Self::PrismaticJointSetLimits),
            0xBE => Some(Self::PrismaticJointEnableMotor),
            0xBF => Some(Self::PrismaticJointSetMotorSpeed),
            0xC0 => Some(Self::PrismaticJointSetMaxMotorForce),
            0xC1 => Some(Self::RevoluteJointEnableSpring),
            0xC2 => Some(Self::RevoluteJointSetSpringHertz),
            0xC3 => Some(Self::RevoluteJointSetSpringDampingRatio),
            0xC4 => Some(Self::RevoluteJointSetTargetAngle),
            0xC5 => Some(Self::RevoluteJointEnableLimit),
            0xC6 => Some(Self::RevoluteJointSetLimits),
            0xC7 => Some(Self::RevoluteJointEnableMotor),
            0xC8 => Some(Self::RevoluteJointSetMotorSpeed),
            0xC9 => Some(Self::RevoluteJointSetMaxMotorTorque),
            0xCA => Some(Self::SphericalJointEnableConeLimit),
            0xCB => Some(Self::SphericalJointSetConeLimit),
            0xCC => Some(Self::SphericalJointEnableTwistLimit),
            0xCD => Some(Self::SphericalJointSetTwistLimits),
            0xCE => Some(Self::SphericalJointEnableSpring),
            0xCF => Some(Self::SphericalJointSetSpringHertz),
            0xD0 => Some(Self::SphericalJointSetSpringDampingRatio),
            0xD1 => Some(Self::SphericalJointSetTargetRotation),
            0xD2 => Some(Self::SphericalJointEnableMotor),
            0xD3 => Some(Self::SphericalJointSetMotorVelocity),
            0xD4 => Some(Self::SphericalJointSetMaxMotorTorque),
            0xD5 => Some(Self::WeldJointSetLinearHertz),
            0xD6 => Some(Self::WeldJointSetLinearDampingRatio),
            0xD7 => Some(Self::WeldJointSetAngularHertz),
            0xD8 => Some(Self::WeldJointSetAngularDampingRatio),
            0xD9 => Some(Self::WheelJointEnableSuspension),
            0xDA => Some(Self::WheelJointSetSuspensionHertz),
            0xDB => Some(Self::WheelJointSetSuspensionDampingRatio),
            0xDC => Some(Self::WheelJointEnableSuspensionLimit),
            0xDD => Some(Self::WheelJointSetSuspensionLimits),
            0xDE => Some(Self::WheelJointEnableSpinMotor),
            0xDF => Some(Self::WheelJointSetSpinMotorSpeed),
            0xE0 => Some(Self::WheelJointSetMaxSpinTorque),
            0xE1 => Some(Self::WheelJointEnableSteering),
            0xE2 => Some(Self::WheelJointSetSteeringHertz),
            0xE3 => Some(Self::WheelJointSetSteeringDampingRatio),
            0xE4 => Some(Self::WheelJointSetMaxSteeringTorque),
            0xE5 => Some(Self::WheelJointEnableSteeringLimit),
            0xE6 => Some(Self::WheelJointSetSteeringLimits),
            0xE7 => Some(Self::WheelJointSetTargetSteeringAngle),
            0xE8 => Some(Self::QueryOverlapAABB),
            0xE9 => Some(Self::QueryOverlapShape),
            0xEA => Some(Self::QueryCastRay),
            0xEB => Some(Self::QueryCastShape),
            0xEC => Some(Self::QueryCastRayClosest),
            0xED => Some(Self::QueryCastMover),
            0xEE => Some(Self::QueryCollideMover),
            0xEF => Some(Self::QueryTag),
            0xF1 => Some(Self::StateHash),
            0xF2 => Some(Self::RecordingBounds),
            _ => None,
        }
    }
}

/// Args for `DestroyWorld` (op 0x01).
#[derive(Debug, Clone)]
pub struct ArgsDestroyWorld {
    pub world: WorldId,
}

/// Args for `Step` (op 0x80).
#[derive(Debug, Clone)]
pub struct ArgsStep {
    pub world: WorldId,
    pub dt: f32,
    pub sub_step_count: i32,
}

/// Args for `WorldEnableSleeping` (op 0x02).
#[derive(Debug, Clone)]
pub struct ArgsWorldEnableSleeping {
    pub world: WorldId,
    pub flag: bool,
}

/// Args for `WorldEnableContinuous` (op 0x03).
#[derive(Debug, Clone)]
pub struct ArgsWorldEnableContinuous {
    pub world: WorldId,
    pub flag: bool,
}

/// Args for `WorldSetRestitutionThreshold` (op 0x04).
#[derive(Debug, Clone)]
pub struct ArgsWorldSetRestitutionThreshold {
    pub world: WorldId,
    pub value: f32,
}

/// Args for `WorldSetHitEventThreshold` (op 0x05).
#[derive(Debug, Clone)]
pub struct ArgsWorldSetHitEventThreshold {
    pub world: WorldId,
    pub value: f32,
}

/// Args for `WorldSetGravity` (op 0x06).
#[derive(Debug, Clone)]
pub struct ArgsWorldSetGravity {
    pub world: WorldId,
    pub gravity: Vec3,
}

/// Args for `WorldExplode` (op 0x07).
#[derive(Debug, Clone)]
pub struct ArgsWorldExplode {
    pub world: WorldId,
    pub def: ExplosionDef,
}

/// Args for `WorldSetContactTuning` (op 0x08).
#[derive(Debug, Clone)]
pub struct ArgsWorldSetContactTuning {
    pub world: WorldId,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub contact_speed: f32,
}

/// Args for `WorldSetContactRecycleDistance` (op 0x09).
#[derive(Debug, Clone)]
pub struct ArgsWorldSetContactRecycleDistance {
    pub world: WorldId,
    pub recycle_distance: f32,
}

/// Args for `WorldSetMaximumLinearSpeed` (op 0x0A).
#[derive(Debug, Clone)]
pub struct ArgsWorldSetMaximumLinearSpeed {
    pub world: WorldId,
    pub maximum_linear_speed: f32,
}

/// Args for `WorldEnableWarmStarting` (op 0x0B).
#[derive(Debug, Clone)]
pub struct ArgsWorldEnableWarmStarting {
    pub world: WorldId,
    pub flag: bool,
}

/// Args for `WorldRebuildStaticTree` (op 0x0C).
#[derive(Debug, Clone)]
pub struct ArgsWorldRebuildStaticTree {
    pub world: WorldId,
}

/// Args for `WorldEnableSpeculative` (op 0x0D).
#[derive(Debug, Clone)]
pub struct ArgsWorldEnableSpeculative {
    pub world: WorldId,
    pub flag: bool,
}

/// Args for `CreateBody` (op 0x10).
#[derive(Debug, Clone)]
pub struct ArgsCreateBody {
    pub world: WorldId,
    pub def: BodyDef,
}

/// Args for `DestroyBody` (op 0x11).
#[derive(Debug, Clone)]
pub struct ArgsDestroyBody {
    pub body: BodyId,
}

/// Args for `BodySetTransform` (op 0x20).
#[derive(Debug, Clone)]
pub struct ArgsBodySetTransform {
    pub body: BodyId,
    pub position: Pos,
    pub rotation: Quat,
}

/// Args for `BodySetLinearVelocity` (op 0x21).
#[derive(Debug, Clone)]
pub struct ArgsBodySetLinearVelocity {
    pub body: BodyId,
    pub v: Vec3,
}

/// Args for `BodySetType` (op 0x22).
#[derive(Debug, Clone)]
pub struct ArgsBodySetType {
    pub body: BodyId,
    pub type_: i32,
}

/// Args for `BodySetName` (op 0x23).
#[derive(Debug, Clone)]
pub struct ArgsBodySetName {
    pub body: BodyId,
    pub name: String,
}

/// Args for `BodySetAngularVelocity` (op 0x24).
#[derive(Debug, Clone)]
pub struct ArgsBodySetAngularVelocity {
    pub body: BodyId,
    pub w: Vec3,
}

/// Args for `BodySetTargetTransform` (op 0x25).
#[derive(Debug, Clone)]
pub struct ArgsBodySetTargetTransform {
    pub body: BodyId,
    pub target: WorldTransform,
    pub time_step: f32,
    pub wake: bool,
}

/// Args for `BodyApplyForce` (op 0x26).
#[derive(Debug, Clone)]
pub struct ArgsBodyApplyForce {
    pub body: BodyId,
    pub force: Vec3,
    pub point: Pos,
    pub wake: bool,
}

/// Args for `BodyApplyForceToCenter` (op 0x27).
#[derive(Debug, Clone)]
pub struct ArgsBodyApplyForceToCenter {
    pub body: BodyId,
    pub force: Vec3,
    pub wake: bool,
}

/// Args for `BodyApplyTorque` (op 0x28).
#[derive(Debug, Clone)]
pub struct ArgsBodyApplyTorque {
    pub body: BodyId,
    pub torque: Vec3,
    pub wake: bool,
}

/// Args for `BodyApplyLinearImpulse` (op 0x29).
#[derive(Debug, Clone)]
pub struct ArgsBodyApplyLinearImpulse {
    pub body: BodyId,
    pub impulse: Vec3,
    pub point: Pos,
    pub wake: bool,
}

/// Args for `BodyApplyLinearImpulseToCenter` (op 0x2A).
#[derive(Debug, Clone)]
pub struct ArgsBodyApplyLinearImpulseToCenter {
    pub body: BodyId,
    pub impulse: Vec3,
    pub wake: bool,
}

/// Args for `BodyApplyAngularImpulse` (op 0x2B).
#[derive(Debug, Clone)]
pub struct ArgsBodyApplyAngularImpulse {
    pub body: BodyId,
    pub impulse: Vec3,
    pub wake: bool,
}

/// Args for `BodySetMassData` (op 0x2C).
#[derive(Debug, Clone)]
pub struct ArgsBodySetMassData {
    pub body: BodyId,
    pub mass_data: MassData,
}

/// Args for `BodyApplyMassFromShapes` (op 0x2D).
#[derive(Debug, Clone)]
pub struct ArgsBodyApplyMassFromShapes {
    pub body: BodyId,
}

/// Args for `BodySetLinearDamping` (op 0x2E).
#[derive(Debug, Clone)]
pub struct ArgsBodySetLinearDamping {
    pub body: BodyId,
    pub damping: f32,
}

/// Args for `BodySetAngularDamping` (op 0x2F).
#[derive(Debug, Clone)]
pub struct ArgsBodySetAngularDamping {
    pub body: BodyId,
    pub damping: f32,
}

/// Args for `BodySetGravityScale` (op 0x30).
#[derive(Debug, Clone)]
pub struct ArgsBodySetGravityScale {
    pub body: BodyId,
    pub scale: f32,
}

/// Args for `BodySetAwake` (op 0x31).
#[derive(Debug, Clone)]
pub struct ArgsBodySetAwake {
    pub body: BodyId,
    pub awake: bool,
}

/// Args for `BodyEnableSleep` (op 0x32).
#[derive(Debug, Clone)]
pub struct ArgsBodyEnableSleep {
    pub body: BodyId,
    pub flag: bool,
}

/// Args for `BodySetSleepThreshold` (op 0x33).
#[derive(Debug, Clone)]
pub struct ArgsBodySetSleepThreshold {
    pub body: BodyId,
    pub threshold: f32,
}

/// Args for `BodyDisable` (op 0x34).
#[derive(Debug, Clone)]
pub struct ArgsBodyDisable {
    pub body: BodyId,
}

/// Args for `BodyEnable` (op 0x35).
#[derive(Debug, Clone)]
pub struct ArgsBodyEnable {
    pub body: BodyId,
}

/// Args for `BodySetMotionLocks` (op 0x36).
#[derive(Debug, Clone)]
pub struct ArgsBodySetMotionLocks {
    pub body: BodyId,
    pub locks: MotionLocks,
}

/// Args for `BodySetBullet` (op 0x37).
#[derive(Debug, Clone)]
pub struct ArgsBodySetBullet {
    pub body: BodyId,
    pub flag: bool,
}

/// Args for `BodyEnableContactRecycling` (op 0x38).
#[derive(Debug, Clone)]
pub struct ArgsBodyEnableContactRecycling {
    pub body: BodyId,
    pub flag: bool,
}

/// Args for `BodyEnableHitEvents` (op 0x39).
#[derive(Debug, Clone)]
pub struct ArgsBodyEnableHitEvents {
    pub body: BodyId,
    pub flag: bool,
}

/// Args for `CreateSphereShape` (op 0x40).
#[derive(Debug, Clone)]
pub struct ArgsCreateSphereShape {
    pub body: BodyId,
    pub def: ShapeDef,
    pub sphere: Sphere,
}

/// Args for `CreateCapsuleShape` (op 0x41).
#[derive(Debug, Clone)]
pub struct ArgsCreateCapsuleShape {
    pub body: BodyId,
    pub def: ShapeDef,
    pub capsule: Capsule,
}

/// Args for `CreateHullShape` (op 0x42).
#[derive(Debug, Clone)]
pub struct ArgsCreateHullShape {
    pub body: BodyId,
    pub def: ShapeDef,
    pub geometry_id: u32,
}

/// Args for `CreateMeshShape` (op 0x43).
#[derive(Debug, Clone)]
pub struct ArgsCreateMeshShape {
    pub body: BodyId,
    pub def: ShapeDef,
    pub geometry_id: u32,
    pub scale: Vec3,
}

/// Args for `CreateHeightFieldShape` (op 0x44).
#[derive(Debug, Clone)]
pub struct ArgsCreateHeightFieldShape {
    pub body: BodyId,
    pub def: ShapeDef,
    pub geometry_id: u32,
}

/// Args for `CreateCompoundShape` (op 0x45).
#[derive(Debug, Clone)]
pub struct ArgsCreateCompoundShape {
    pub body: BodyId,
    pub def: ShapeDef,
    pub geometry_id: u32,
}

/// Args for `DestroyShape` (op 0x46).
#[derive(Debug, Clone)]
pub struct ArgsDestroyShape {
    pub shape: ShapeId,
    pub update_body_mass: bool,
}

/// Args for `ShapeSetDensity` (op 0x50).
#[derive(Debug, Clone)]
pub struct ArgsShapeSetDensity {
    pub shape: ShapeId,
    pub density: f32,
    pub update_body_mass: bool,
}

/// Args for `ShapeSetFriction` (op 0x51).
#[derive(Debug, Clone)]
pub struct ArgsShapeSetFriction {
    pub shape: ShapeId,
    pub friction: f32,
}

/// Args for `ShapeSetRestitution` (op 0x52).
#[derive(Debug, Clone)]
pub struct ArgsShapeSetRestitution {
    pub shape: ShapeId,
    pub restitution: f32,
}

/// Args for `ShapeSetSurfaceMaterial` (op 0x53).
#[derive(Debug, Clone)]
pub struct ArgsShapeSetSurfaceMaterial {
    pub shape: ShapeId,
    pub material: SurfaceMaterial,
}

/// Args for `ShapeSetFilter` (op 0x54).
#[derive(Debug, Clone)]
pub struct ArgsShapeSetFilter {
    pub shape: ShapeId,
    pub filter: Filter,
    pub invoke_contacts: bool,
}

/// Args for `ShapeEnableSensorEvents` (op 0x55).
#[derive(Debug, Clone)]
pub struct ArgsShapeEnableSensorEvents {
    pub shape: ShapeId,
    pub flag: bool,
}

/// Args for `ShapeEnableContactEvents` (op 0x56).
#[derive(Debug, Clone)]
pub struct ArgsShapeEnableContactEvents {
    pub shape: ShapeId,
    pub flag: bool,
}

/// Args for `ShapeEnablePreSolveEvents` (op 0x57).
#[derive(Debug, Clone)]
pub struct ArgsShapeEnablePreSolveEvents {
    pub shape: ShapeId,
    pub flag: bool,
}

/// Args for `ShapeEnableHitEvents` (op 0x58).
#[derive(Debug, Clone)]
pub struct ArgsShapeEnableHitEvents {
    pub shape: ShapeId,
    pub flag: bool,
}

/// Args for `ShapeSetSphere` (op 0x59).
#[derive(Debug, Clone)]
pub struct ArgsShapeSetSphere {
    pub shape: ShapeId,
    pub sphere: Sphere,
}

/// Args for `ShapeSetCapsule` (op 0x5A).
#[derive(Debug, Clone)]
pub struct ArgsShapeSetCapsule {
    pub shape: ShapeId,
    pub capsule: Capsule,
}

/// Args for `ShapeApplyWind` (op 0x5B).
#[derive(Debug, Clone)]
pub struct ArgsShapeApplyWind {
    pub shape: ShapeId,
    pub wind: Vec3,
    pub drag: f32,
    pub lift: f32,
    pub max_speed: f32,
    pub wake: bool,
}

/// Args for `ShapeSetName` (op 0x5C).
#[derive(Debug, Clone)]
pub struct ArgsShapeSetName {
    pub shape: ShapeId,
    pub name: String,
}

/// Args for `CreateParallelJoint` (op 0x90).
#[derive(Debug, Clone)]
pub struct ArgsCreateParallelJoint {
    pub world: WorldId,
    pub def: ParallelJointDef,
}

/// Args for `CreateDistanceJoint` (op 0x91).
#[derive(Debug, Clone)]
pub struct ArgsCreateDistanceJoint {
    pub world: WorldId,
    pub def: DistanceJointDef,
}

/// Args for `CreateFilterJoint` (op 0x92).
#[derive(Debug, Clone)]
pub struct ArgsCreateFilterJoint {
    pub world: WorldId,
    pub def: FilterJointDef,
}

/// Args for `CreateMotorJoint` (op 0x93).
#[derive(Debug, Clone)]
pub struct ArgsCreateMotorJoint {
    pub world: WorldId,
    pub def: MotorJointDef,
}

/// Args for `CreatePrismaticJoint` (op 0x94).
#[derive(Debug, Clone)]
pub struct ArgsCreatePrismaticJoint {
    pub world: WorldId,
    pub def: PrismaticJointDef,
}

/// Args for `CreateRevoluteJoint` (op 0x95).
#[derive(Debug, Clone)]
pub struct ArgsCreateRevoluteJoint {
    pub world: WorldId,
    pub def: RevoluteJointDef,
}

/// Args for `CreateSphericalJoint` (op 0x96).
#[derive(Debug, Clone)]
pub struct ArgsCreateSphericalJoint {
    pub world: WorldId,
    pub def: SphericalJointDef,
}

/// Args for `CreateWeldJoint` (op 0x97).
#[derive(Debug, Clone)]
pub struct ArgsCreateWeldJoint {
    pub world: WorldId,
    pub def: WeldJointDef,
}

/// Args for `CreateWheelJoint` (op 0x98).
#[derive(Debug, Clone)]
pub struct ArgsCreateWheelJoint {
    pub world: WorldId,
    pub def: WheelJointDef,
}

/// Args for `DestroyJoint` (op 0x99).
#[derive(Debug, Clone)]
pub struct ArgsDestroyJoint {
    pub joint: JointId,
    pub wake_attached: bool,
}

/// Args for `JointSetLocalFrameA` (op 0x9A).
#[derive(Debug, Clone)]
pub struct ArgsJointSetLocalFrameA {
    pub joint: JointId,
    pub local_frame: Transform,
}

/// Args for `JointSetLocalFrameB` (op 0x9B).
#[derive(Debug, Clone)]
pub struct ArgsJointSetLocalFrameB {
    pub joint: JointId,
    pub local_frame: Transform,
}

/// Args for `JointSetCollideConnected` (op 0x9C).
#[derive(Debug, Clone)]
pub struct ArgsJointSetCollideConnected {
    pub joint: JointId,
    pub should_collide: bool,
}

/// Args for `JointWakeBodies` (op 0x9D).
#[derive(Debug, Clone)]
pub struct ArgsJointWakeBodies {
    pub joint: JointId,
}

/// Args for `JointSetConstraintTuning` (op 0x9E).
#[derive(Debug, Clone)]
pub struct ArgsJointSetConstraintTuning {
    pub joint: JointId,
    pub hertz: f32,
    pub damping_ratio: f32,
}

/// Args for `JointSetForceThreshold` (op 0x9F).
#[derive(Debug, Clone)]
pub struct ArgsJointSetForceThreshold {
    pub joint: JointId,
    pub threshold: f32,
}

/// Args for `JointSetTorqueThreshold` (op 0xA0).
#[derive(Debug, Clone)]
pub struct ArgsJointSetTorqueThreshold {
    pub joint: JointId,
    pub threshold: f32,
}

/// Args for `ParallelJointSetSpringHertz` (op 0xA1).
#[derive(Debug, Clone)]
pub struct ArgsParallelJointSetSpringHertz {
    pub joint: JointId,
    pub hertz: f32,
}

/// Args for `ParallelJointSetSpringDampingRatio` (op 0xA2).
#[derive(Debug, Clone)]
pub struct ArgsParallelJointSetSpringDampingRatio {
    pub joint: JointId,
    pub damping_ratio: f32,
}

/// Args for `ParallelJointSetMaxTorque` (op 0xA3).
#[derive(Debug, Clone)]
pub struct ArgsParallelJointSetMaxTorque {
    pub joint: JointId,
    pub max_torque: f32,
}

/// Args for `DistanceJointSetLength` (op 0xA4).
#[derive(Debug, Clone)]
pub struct ArgsDistanceJointSetLength {
    pub joint: JointId,
    pub length: f32,
}

/// Args for `DistanceJointEnableSpring` (op 0xA5).
#[derive(Debug, Clone)]
pub struct ArgsDistanceJointEnableSpring {
    pub joint: JointId,
    pub enable_spring: bool,
}

/// Args for `DistanceJointSetSpringForceRange` (op 0xA6).
#[derive(Debug, Clone)]
pub struct ArgsDistanceJointSetSpringForceRange {
    pub joint: JointId,
    pub lower_force: f32,
    pub upper_force: f32,
}

/// Args for `DistanceJointSetSpringHertz` (op 0xA7).
#[derive(Debug, Clone)]
pub struct ArgsDistanceJointSetSpringHertz {
    pub joint: JointId,
    pub hertz: f32,
}

/// Args for `DistanceJointSetSpringDampingRatio` (op 0xA8).
#[derive(Debug, Clone)]
pub struct ArgsDistanceJointSetSpringDampingRatio {
    pub joint: JointId,
    pub damping_ratio: f32,
}

/// Args for `DistanceJointEnableLimit` (op 0xA9).
#[derive(Debug, Clone)]
pub struct ArgsDistanceJointEnableLimit {
    pub joint: JointId,
    pub enable_limit: bool,
}

/// Args for `DistanceJointSetLengthRange` (op 0xAA).
#[derive(Debug, Clone)]
pub struct ArgsDistanceJointSetLengthRange {
    pub joint: JointId,
    pub min_length: f32,
    pub max_length: f32,
}

/// Args for `DistanceJointEnableMotor` (op 0xAB).
#[derive(Debug, Clone)]
pub struct ArgsDistanceJointEnableMotor {
    pub joint: JointId,
    pub enable_motor: bool,
}

/// Args for `DistanceJointSetMotorSpeed` (op 0xAC).
#[derive(Debug, Clone)]
pub struct ArgsDistanceJointSetMotorSpeed {
    pub joint: JointId,
    pub motor_speed: f32,
}

/// Args for `DistanceJointSetMaxMotorForce` (op 0xAD).
#[derive(Debug, Clone)]
pub struct ArgsDistanceJointSetMaxMotorForce {
    pub joint: JointId,
    pub force: f32,
}

/// Args for `MotorJointSetLinearVelocity` (op 0xAE).
#[derive(Debug, Clone)]
pub struct ArgsMotorJointSetLinearVelocity {
    pub joint: JointId,
    pub velocity: Vec3,
}

/// Args for `MotorJointSetAngularVelocity` (op 0xAF).
#[derive(Debug, Clone)]
pub struct ArgsMotorJointSetAngularVelocity {
    pub joint: JointId,
    pub velocity: Vec3,
}

/// Args for `MotorJointSetMaxVelocityForce` (op 0xB0).
#[derive(Debug, Clone)]
pub struct ArgsMotorJointSetMaxVelocityForce {
    pub joint: JointId,
    pub max_force: f32,
}

/// Args for `MotorJointSetMaxVelocityTorque` (op 0xB1).
#[derive(Debug, Clone)]
pub struct ArgsMotorJointSetMaxVelocityTorque {
    pub joint: JointId,
    pub max_torque: f32,
}

/// Args for `MotorJointSetLinearHertz` (op 0xB2).
#[derive(Debug, Clone)]
pub struct ArgsMotorJointSetLinearHertz {
    pub joint: JointId,
    pub hertz: f32,
}

/// Args for `MotorJointSetLinearDampingRatio` (op 0xB3).
#[derive(Debug, Clone)]
pub struct ArgsMotorJointSetLinearDampingRatio {
    pub joint: JointId,
    pub damping: f32,
}

/// Args for `MotorJointSetAngularHertz` (op 0xB4).
#[derive(Debug, Clone)]
pub struct ArgsMotorJointSetAngularHertz {
    pub joint: JointId,
    pub hertz: f32,
}

/// Args for `MotorJointSetAngularDampingRatio` (op 0xB5).
#[derive(Debug, Clone)]
pub struct ArgsMotorJointSetAngularDampingRatio {
    pub joint: JointId,
    pub damping: f32,
}

/// Args for `MotorJointSetMaxSpringForce` (op 0xB6).
#[derive(Debug, Clone)]
pub struct ArgsMotorJointSetMaxSpringForce {
    pub joint: JointId,
    pub max_force: f32,
}

/// Args for `MotorJointSetMaxSpringTorque` (op 0xB7).
#[derive(Debug, Clone)]
pub struct ArgsMotorJointSetMaxSpringTorque {
    pub joint: JointId,
    pub max_torque: f32,
}

/// Args for `PrismaticJointEnableSpring` (op 0xB8).
#[derive(Debug, Clone)]
pub struct ArgsPrismaticJointEnableSpring {
    pub joint: JointId,
    pub enable_spring: bool,
}

/// Args for `PrismaticJointSetSpringHertz` (op 0xB9).
#[derive(Debug, Clone)]
pub struct ArgsPrismaticJointSetSpringHertz {
    pub joint: JointId,
    pub hertz: f32,
}

/// Args for `PrismaticJointSetSpringDampingRatio` (op 0xBA).
#[derive(Debug, Clone)]
pub struct ArgsPrismaticJointSetSpringDampingRatio {
    pub joint: JointId,
    pub damping_ratio: f32,
}

/// Args for `PrismaticJointSetTargetTranslation` (op 0xBB).
#[derive(Debug, Clone)]
pub struct ArgsPrismaticJointSetTargetTranslation {
    pub joint: JointId,
    pub translation: f32,
}

/// Args for `PrismaticJointEnableLimit` (op 0xBC).
#[derive(Debug, Clone)]
pub struct ArgsPrismaticJointEnableLimit {
    pub joint: JointId,
    pub enable_limit: bool,
}

/// Args for `PrismaticJointSetLimits` (op 0xBD).
#[derive(Debug, Clone)]
pub struct ArgsPrismaticJointSetLimits {
    pub joint: JointId,
    pub lower: f32,
    pub upper: f32,
}

/// Args for `PrismaticJointEnableMotor` (op 0xBE).
#[derive(Debug, Clone)]
pub struct ArgsPrismaticJointEnableMotor {
    pub joint: JointId,
    pub enable_motor: bool,
}

/// Args for `PrismaticJointSetMotorSpeed` (op 0xBF).
#[derive(Debug, Clone)]
pub struct ArgsPrismaticJointSetMotorSpeed {
    pub joint: JointId,
    pub motor_speed: f32,
}

/// Args for `PrismaticJointSetMaxMotorForce` (op 0xC0).
#[derive(Debug, Clone)]
pub struct ArgsPrismaticJointSetMaxMotorForce {
    pub joint: JointId,
    pub force: f32,
}

/// Args for `RevoluteJointEnableSpring` (op 0xC1).
#[derive(Debug, Clone)]
pub struct ArgsRevoluteJointEnableSpring {
    pub joint: JointId,
    pub enable_spring: bool,
}

/// Args for `RevoluteJointSetSpringHertz` (op 0xC2).
#[derive(Debug, Clone)]
pub struct ArgsRevoluteJointSetSpringHertz {
    pub joint: JointId,
    pub hertz: f32,
}

/// Args for `RevoluteJointSetSpringDampingRatio` (op 0xC3).
#[derive(Debug, Clone)]
pub struct ArgsRevoluteJointSetSpringDampingRatio {
    pub joint: JointId,
    pub damping_ratio: f32,
}

/// Args for `RevoluteJointSetTargetAngle` (op 0xC4).
#[derive(Debug, Clone)]
pub struct ArgsRevoluteJointSetTargetAngle {
    pub joint: JointId,
    pub angle: f32,
}

/// Args for `RevoluteJointEnableLimit` (op 0xC5).
#[derive(Debug, Clone)]
pub struct ArgsRevoluteJointEnableLimit {
    pub joint: JointId,
    pub enable_limit: bool,
}

/// Args for `RevoluteJointSetLimits` (op 0xC6).
#[derive(Debug, Clone)]
pub struct ArgsRevoluteJointSetLimits {
    pub joint: JointId,
    pub lower: f32,
    pub upper: f32,
}

/// Args for `RevoluteJointEnableMotor` (op 0xC7).
#[derive(Debug, Clone)]
pub struct ArgsRevoluteJointEnableMotor {
    pub joint: JointId,
    pub enable_motor: bool,
}

/// Args for `RevoluteJointSetMotorSpeed` (op 0xC8).
#[derive(Debug, Clone)]
pub struct ArgsRevoluteJointSetMotorSpeed {
    pub joint: JointId,
    pub motor_speed: f32,
}

/// Args for `RevoluteJointSetMaxMotorTorque` (op 0xC9).
#[derive(Debug, Clone)]
pub struct ArgsRevoluteJointSetMaxMotorTorque {
    pub joint: JointId,
    pub torque: f32,
}

/// Args for `SphericalJointEnableConeLimit` (op 0xCA).
#[derive(Debug, Clone)]
pub struct ArgsSphericalJointEnableConeLimit {
    pub joint: JointId,
    pub enable_limit: bool,
}

/// Args for `SphericalJointSetConeLimit` (op 0xCB).
#[derive(Debug, Clone)]
pub struct ArgsSphericalJointSetConeLimit {
    pub joint: JointId,
    pub angle_radians: f32,
}

/// Args for `SphericalJointEnableTwistLimit` (op 0xCC).
#[derive(Debug, Clone)]
pub struct ArgsSphericalJointEnableTwistLimit {
    pub joint: JointId,
    pub enable_limit: bool,
}

/// Args for `SphericalJointSetTwistLimits` (op 0xCD).
#[derive(Debug, Clone)]
pub struct ArgsSphericalJointSetTwistLimits {
    pub joint: JointId,
    pub lower: f32,
    pub upper: f32,
}

/// Args for `SphericalJointEnableSpring` (op 0xCE).
#[derive(Debug, Clone)]
pub struct ArgsSphericalJointEnableSpring {
    pub joint: JointId,
    pub enable_spring: bool,
}

/// Args for `SphericalJointSetSpringHertz` (op 0xCF).
#[derive(Debug, Clone)]
pub struct ArgsSphericalJointSetSpringHertz {
    pub joint: JointId,
    pub hertz: f32,
}

/// Args for `SphericalJointSetSpringDampingRatio` (op 0xD0).
#[derive(Debug, Clone)]
pub struct ArgsSphericalJointSetSpringDampingRatio {
    pub joint: JointId,
    pub damping_ratio: f32,
}

/// Args for `SphericalJointSetTargetRotation` (op 0xD1).
#[derive(Debug, Clone)]
pub struct ArgsSphericalJointSetTargetRotation {
    pub joint: JointId,
    pub target_rotation: Quat,
}

/// Args for `SphericalJointEnableMotor` (op 0xD2).
#[derive(Debug, Clone)]
pub struct ArgsSphericalJointEnableMotor {
    pub joint: JointId,
    pub enable_motor: bool,
}

/// Args for `SphericalJointSetMotorVelocity` (op 0xD3).
#[derive(Debug, Clone)]
pub struct ArgsSphericalJointSetMotorVelocity {
    pub joint: JointId,
    pub motor_velocity: Vec3,
}

/// Args for `SphericalJointSetMaxMotorTorque` (op 0xD4).
#[derive(Debug, Clone)]
pub struct ArgsSphericalJointSetMaxMotorTorque {
    pub joint: JointId,
    pub torque: f32,
}

/// Args for `WeldJointSetLinearHertz` (op 0xD5).
#[derive(Debug, Clone)]
pub struct ArgsWeldJointSetLinearHertz {
    pub joint: JointId,
    pub hertz: f32,
}

/// Args for `WeldJointSetLinearDampingRatio` (op 0xD6).
#[derive(Debug, Clone)]
pub struct ArgsWeldJointSetLinearDampingRatio {
    pub joint: JointId,
    pub damping_ratio: f32,
}

/// Args for `WeldJointSetAngularHertz` (op 0xD7).
#[derive(Debug, Clone)]
pub struct ArgsWeldJointSetAngularHertz {
    pub joint: JointId,
    pub hertz: f32,
}

/// Args for `WeldJointSetAngularDampingRatio` (op 0xD8).
#[derive(Debug, Clone)]
pub struct ArgsWeldJointSetAngularDampingRatio {
    pub joint: JointId,
    pub damping_ratio: f32,
}

/// Args for `WheelJointEnableSuspension` (op 0xD9).
#[derive(Debug, Clone)]
pub struct ArgsWheelJointEnableSuspension {
    pub joint: JointId,
    pub flag: bool,
}

/// Args for `WheelJointSetSuspensionHertz` (op 0xDA).
#[derive(Debug, Clone)]
pub struct ArgsWheelJointSetSuspensionHertz {
    pub joint: JointId,
    pub hertz: f32,
}

/// Args for `WheelJointSetSuspensionDampingRatio` (op 0xDB).
#[derive(Debug, Clone)]
pub struct ArgsWheelJointSetSuspensionDampingRatio {
    pub joint: JointId,
    pub damping_ratio: f32,
}

/// Args for `WheelJointEnableSuspensionLimit` (op 0xDC).
#[derive(Debug, Clone)]
pub struct ArgsWheelJointEnableSuspensionLimit {
    pub joint: JointId,
    pub flag: bool,
}

/// Args for `WheelJointSetSuspensionLimits` (op 0xDD).
#[derive(Debug, Clone)]
pub struct ArgsWheelJointSetSuspensionLimits {
    pub joint: JointId,
    pub lower: f32,
    pub upper: f32,
}

/// Args for `WheelJointEnableSpinMotor` (op 0xDE).
#[derive(Debug, Clone)]
pub struct ArgsWheelJointEnableSpinMotor {
    pub joint: JointId,
    pub flag: bool,
}

/// Args for `WheelJointSetSpinMotorSpeed` (op 0xDF).
#[derive(Debug, Clone)]
pub struct ArgsWheelJointSetSpinMotorSpeed {
    pub joint: JointId,
    pub speed: f32,
}

/// Args for `WheelJointSetMaxSpinTorque` (op 0xE0).
#[derive(Debug, Clone)]
pub struct ArgsWheelJointSetMaxSpinTorque {
    pub joint: JointId,
    pub torque: f32,
}

/// Args for `WheelJointEnableSteering` (op 0xE1).
#[derive(Debug, Clone)]
pub struct ArgsWheelJointEnableSteering {
    pub joint: JointId,
    pub flag: bool,
}

/// Args for `WheelJointSetSteeringHertz` (op 0xE2).
#[derive(Debug, Clone)]
pub struct ArgsWheelJointSetSteeringHertz {
    pub joint: JointId,
    pub hertz: f32,
}

/// Args for `WheelJointSetSteeringDampingRatio` (op 0xE3).
#[derive(Debug, Clone)]
pub struct ArgsWheelJointSetSteeringDampingRatio {
    pub joint: JointId,
    pub damping_ratio: f32,
}

/// Args for `WheelJointSetMaxSteeringTorque` (op 0xE4).
#[derive(Debug, Clone)]
pub struct ArgsWheelJointSetMaxSteeringTorque {
    pub joint: JointId,
    pub torque: f32,
}

/// Args for `WheelJointEnableSteeringLimit` (op 0xE5).
#[derive(Debug, Clone)]
pub struct ArgsWheelJointEnableSteeringLimit {
    pub joint: JointId,
    pub flag: bool,
}

/// Args for `WheelJointSetSteeringLimits` (op 0xE6).
#[derive(Debug, Clone)]
pub struct ArgsWheelJointSetSteeringLimits {
    pub joint: JointId,
    pub lower: f32,
    pub upper: f32,
}

/// Args for `WheelJointSetTargetSteeringAngle` (op 0xE7).
#[derive(Debug, Clone)]
pub struct ArgsWheelJointSetTargetSteeringAngle {
    pub joint: JointId,
    pub radians: f32,
}

/// Args for `QueryOverlapAABB` (op 0xE8).
#[derive(Debug, Clone)]
pub struct ArgsQueryOverlapAABB {
    pub world: WorldId,
    pub aabb: Aabb,
    pub filter: QueryFilter,
}

/// Args for `QueryOverlapShape` (op 0xE9).
#[derive(Debug, Clone)]
pub struct ArgsQueryOverlapShape {
    pub world: WorldId,
    pub origin: Pos,
    pub proxy: ShapeProxy,
    pub filter: QueryFilter,
}

/// Args for `QueryCastRay` (op 0xEA).
#[derive(Debug, Clone)]
pub struct ArgsQueryCastRay {
    pub world: WorldId,
    pub origin: Pos,
    pub translation: Vec3,
    pub filter: QueryFilter,
}

/// Args for `QueryCastShape` (op 0xEB).
#[derive(Debug, Clone)]
pub struct ArgsQueryCastShape {
    pub world: WorldId,
    pub origin: Pos,
    pub proxy: ShapeProxy,
    pub translation: Vec3,
    pub filter: QueryFilter,
}

/// Args for `QueryCastRayClosest` (op 0xEC).
#[derive(Debug, Clone)]
pub struct ArgsQueryCastRayClosest {
    pub world: WorldId,
    pub origin: Pos,
    pub translation: Vec3,
    pub filter: QueryFilter,
}

/// Args for `QueryCastMover` (op 0xED).
#[derive(Debug, Clone)]
pub struct ArgsQueryCastMover {
    pub world: WorldId,
    pub origin: Pos,
    pub mover: Capsule,
    pub translation: Vec3,
    pub filter: QueryFilter,
}

/// Args for `QueryCollideMover` (op 0xEE).
#[derive(Debug, Clone)]
pub struct ArgsQueryCollideMover {
    pub world: WorldId,
    pub origin: Pos,
    pub mover: Capsule,
    pub filter: QueryFilter,
}

/// Args for `QueryTag` (op 0xEF).
#[derive(Debug, Clone)]
pub struct ArgsQueryTag {
    pub key: u64,
}

/// Args for `StateHash` (op 0xF1).
#[derive(Debug, Clone)]
pub struct ArgsStateHash {
    pub world: WorldId,
    pub hash: u64,
}

/// Args for `RecordingBounds` (op 0xF2).
#[derive(Debug, Clone)]
pub struct ArgsRecordingBounds {
    pub bounds: Aabb,
}

impl Recording {
    /// Write framed `DestroyWorld` op.
    pub fn write_destroy_world(&mut self, world: WorldId) {
        self.begin_record(RecOp::DestroyWorld as u8);
        self.buffer.append_world_id(world);
        self.end_record();
    }

    /// Write framed `Step` op.
    pub fn write_step(&mut self, world: WorldId, dt: f32, sub_step_count: i32) {
        self.begin_record(RecOp::Step as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_f32(dt);
        self.buffer.append_i32(sub_step_count);
        self.end_record();
    }

    /// Write framed `WorldEnableSleeping` op.
    pub fn write_world_enable_sleeping(&mut self, world: WorldId, flag: bool) {
        self.begin_record(RecOp::WorldEnableSleeping as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `WorldEnableContinuous` op.
    pub fn write_world_enable_continuous(&mut self, world: WorldId, flag: bool) {
        self.begin_record(RecOp::WorldEnableContinuous as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `WorldSetRestitutionThreshold` op.
    pub fn write_world_set_restitution_threshold(&mut self, world: WorldId, value: f32) {
        self.begin_record(RecOp::WorldSetRestitutionThreshold as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_f32(value);
        self.end_record();
    }

    /// Write framed `WorldSetHitEventThreshold` op.
    pub fn write_world_set_hit_event_threshold(&mut self, world: WorldId, value: f32) {
        self.begin_record(RecOp::WorldSetHitEventThreshold as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_f32(value);
        self.end_record();
    }

    /// Write framed `WorldSetGravity` op.
    pub fn write_world_set_gravity(&mut self, world: WorldId, gravity: Vec3) {
        self.begin_record(RecOp::WorldSetGravity as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_vec3(gravity);
        self.end_record();
    }

    /// Write framed `WorldExplode` op.
    pub fn write_world_explode(&mut self, world: WorldId, def: ExplosionDef) {
        self.begin_record(RecOp::WorldExplode as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_explosion_def(def);
        self.end_record();
    }

    /// Write framed `WorldSetContactTuning` op.
    pub fn write_world_set_contact_tuning(&mut self, world: WorldId, hertz: f32, damping_ratio: f32, contact_speed: f32) {
        self.begin_record(RecOp::WorldSetContactTuning as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_f32(hertz);
        self.buffer.append_f32(damping_ratio);
        self.buffer.append_f32(contact_speed);
        self.end_record();
    }

    /// Write framed `WorldSetContactRecycleDistance` op.
    pub fn write_world_set_contact_recycle_distance(&mut self, world: WorldId, recycle_distance: f32) {
        self.begin_record(RecOp::WorldSetContactRecycleDistance as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_f32(recycle_distance);
        self.end_record();
    }

    /// Write framed `WorldSetMaximumLinearSpeed` op.
    pub fn write_world_set_maximum_linear_speed(&mut self, world: WorldId, maximum_linear_speed: f32) {
        self.begin_record(RecOp::WorldSetMaximumLinearSpeed as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_f32(maximum_linear_speed);
        self.end_record();
    }

    /// Write framed `WorldEnableWarmStarting` op.
    pub fn write_world_enable_warm_starting(&mut self, world: WorldId, flag: bool) {
        self.begin_record(RecOp::WorldEnableWarmStarting as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `WorldRebuildStaticTree` op.
    pub fn write_world_rebuild_static_tree(&mut self, world: WorldId) {
        self.begin_record(RecOp::WorldRebuildStaticTree as u8);
        self.buffer.append_world_id(world);
        self.end_record();
    }

    /// Write framed `WorldEnableSpeculative` op.
    pub fn write_world_enable_speculative(&mut self, world: WorldId, flag: bool) {
        self.begin_record(RecOp::WorldEnableSpeculative as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `CreateBody` op.
    pub fn write_create_body(&mut self, world: WorldId, def: &BodyDef, ret_id: BodyId) {
        self.begin_record(RecOp::CreateBody as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_body_def(def);
        self.buffer.append_body_id(ret_id);
        self.end_record();
    }

    /// Write framed `DestroyBody` op.
    pub fn write_destroy_body(&mut self, body: BodyId) {
        self.begin_record(RecOp::DestroyBody as u8);
        self.buffer.append_body_id(body);
        self.end_record();
    }

    /// Write framed `BodySetTransform` op.
    pub fn write_body_set_transform(&mut self, body: BodyId, position: Pos, rotation: Quat) {
        self.begin_record(RecOp::BodySetTransform as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_pos(position);
        self.buffer.append_quat(rotation);
        self.end_record();
    }

    /// Write framed `BodySetLinearVelocity` op.
    pub fn write_body_set_linear_velocity(&mut self, body: BodyId, v: Vec3) {
        self.begin_record(RecOp::BodySetLinearVelocity as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_vec3(v);
        self.end_record();
    }

    /// Write framed `BodySetType` op.
    pub fn write_body_set_type(&mut self, body: BodyId, type_: i32) {
        self.begin_record(RecOp::BodySetType as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_i32(type_);
        self.end_record();
    }

    /// Write framed `BodySetName` op.
    pub fn write_body_set_name(&mut self, body: BodyId, name: &str) {
        self.begin_record(RecOp::BodySetName as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_str(name);
        self.end_record();
    }

    /// Write framed `BodySetAngularVelocity` op.
    pub fn write_body_set_angular_velocity(&mut self, body: BodyId, w: Vec3) {
        self.begin_record(RecOp::BodySetAngularVelocity as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_vec3(w);
        self.end_record();
    }

    /// Write framed `BodySetTargetTransform` op.
    pub fn write_body_set_target_transform(&mut self, body: BodyId, target: WorldTransform, time_step: f32, wake: bool) {
        self.begin_record(RecOp::BodySetTargetTransform as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_world_xf(target);
        self.buffer.append_f32(time_step);
        self.buffer.append_bool(wake);
        self.end_record();
    }

    /// Write framed `BodyApplyForce` op.
    pub fn write_body_apply_force(&mut self, body: BodyId, force: Vec3, point: Pos, wake: bool) {
        self.begin_record(RecOp::BodyApplyForce as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_vec3(force);
        self.buffer.append_pos(point);
        self.buffer.append_bool(wake);
        self.end_record();
    }

    /// Write framed `BodyApplyForceToCenter` op.
    pub fn write_body_apply_force_to_center(&mut self, body: BodyId, force: Vec3, wake: bool) {
        self.begin_record(RecOp::BodyApplyForceToCenter as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_vec3(force);
        self.buffer.append_bool(wake);
        self.end_record();
    }

    /// Write framed `BodyApplyTorque` op.
    pub fn write_body_apply_torque(&mut self, body: BodyId, torque: Vec3, wake: bool) {
        self.begin_record(RecOp::BodyApplyTorque as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_vec3(torque);
        self.buffer.append_bool(wake);
        self.end_record();
    }

    /// Write framed `BodyApplyLinearImpulse` op.
    pub fn write_body_apply_linear_impulse(&mut self, body: BodyId, impulse: Vec3, point: Pos, wake: bool) {
        self.begin_record(RecOp::BodyApplyLinearImpulse as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_vec3(impulse);
        self.buffer.append_pos(point);
        self.buffer.append_bool(wake);
        self.end_record();
    }

    /// Write framed `BodyApplyLinearImpulseToCenter` op.
    pub fn write_body_apply_linear_impulse_to_center(&mut self, body: BodyId, impulse: Vec3, wake: bool) {
        self.begin_record(RecOp::BodyApplyLinearImpulseToCenter as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_vec3(impulse);
        self.buffer.append_bool(wake);
        self.end_record();
    }

    /// Write framed `BodyApplyAngularImpulse` op.
    pub fn write_body_apply_angular_impulse(&mut self, body: BodyId, impulse: Vec3, wake: bool) {
        self.begin_record(RecOp::BodyApplyAngularImpulse as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_vec3(impulse);
        self.buffer.append_bool(wake);
        self.end_record();
    }

    /// Write framed `BodySetMassData` op.
    pub fn write_body_set_mass_data(&mut self, body: BodyId, mass_data: MassData) {
        self.begin_record(RecOp::BodySetMassData as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_mass_data(mass_data);
        self.end_record();
    }

    /// Write framed `BodyApplyMassFromShapes` op.
    pub fn write_body_apply_mass_from_shapes(&mut self, body: BodyId) {
        self.begin_record(RecOp::BodyApplyMassFromShapes as u8);
        self.buffer.append_body_id(body);
        self.end_record();
    }

    /// Write framed `BodySetLinearDamping` op.
    pub fn write_body_set_linear_damping(&mut self, body: BodyId, damping: f32) {
        self.begin_record(RecOp::BodySetLinearDamping as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_f32(damping);
        self.end_record();
    }

    /// Write framed `BodySetAngularDamping` op.
    pub fn write_body_set_angular_damping(&mut self, body: BodyId, damping: f32) {
        self.begin_record(RecOp::BodySetAngularDamping as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_f32(damping);
        self.end_record();
    }

    /// Write framed `BodySetGravityScale` op.
    pub fn write_body_set_gravity_scale(&mut self, body: BodyId, scale: f32) {
        self.begin_record(RecOp::BodySetGravityScale as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_f32(scale);
        self.end_record();
    }

    /// Write framed `BodySetAwake` op.
    pub fn write_body_set_awake(&mut self, body: BodyId, awake: bool) {
        self.begin_record(RecOp::BodySetAwake as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_bool(awake);
        self.end_record();
    }

    /// Write framed `BodyEnableSleep` op.
    pub fn write_body_enable_sleep(&mut self, body: BodyId, flag: bool) {
        self.begin_record(RecOp::BodyEnableSleep as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `BodySetSleepThreshold` op.
    pub fn write_body_set_sleep_threshold(&mut self, body: BodyId, threshold: f32) {
        self.begin_record(RecOp::BodySetSleepThreshold as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_f32(threshold);
        self.end_record();
    }

    /// Write framed `BodyDisable` op.
    pub fn write_body_disable(&mut self, body: BodyId) {
        self.begin_record(RecOp::BodyDisable as u8);
        self.buffer.append_body_id(body);
        self.end_record();
    }

    /// Write framed `BodyEnable` op.
    pub fn write_body_enable(&mut self, body: BodyId) {
        self.begin_record(RecOp::BodyEnable as u8);
        self.buffer.append_body_id(body);
        self.end_record();
    }

    /// Write framed `BodySetMotionLocks` op.
    pub fn write_body_set_motion_locks(&mut self, body: BodyId, locks: MotionLocks) {
        self.begin_record(RecOp::BodySetMotionLocks as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_locks(locks);
        self.end_record();
    }

    /// Write framed `BodySetBullet` op.
    pub fn write_body_set_bullet(&mut self, body: BodyId, flag: bool) {
        self.begin_record(RecOp::BodySetBullet as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `BodyEnableContactRecycling` op.
    pub fn write_body_enable_contact_recycling(&mut self, body: BodyId, flag: bool) {
        self.begin_record(RecOp::BodyEnableContactRecycling as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `BodyEnableHitEvents` op.
    pub fn write_body_enable_hit_events(&mut self, body: BodyId, flag: bool) {
        self.begin_record(RecOp::BodyEnableHitEvents as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `CreateSphereShape` op.
    pub fn write_create_sphere_shape(&mut self, body: BodyId, def: &ShapeDef, sphere: Sphere, ret_id: ShapeId) {
        self.begin_record(RecOp::CreateSphereShape as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_shape_def(def);
        self.buffer.append_sphere(sphere);
        self.buffer.append_shape_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateCapsuleShape` op.
    pub fn write_create_capsule_shape(&mut self, body: BodyId, def: &ShapeDef, capsule: Capsule, ret_id: ShapeId) {
        self.begin_record(RecOp::CreateCapsuleShape as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_shape_def(def);
        self.buffer.append_capsule(capsule);
        self.buffer.append_shape_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateHullShape` op.
    pub fn write_create_hull_shape(&mut self, body: BodyId, def: &ShapeDef, geometry_id: u32, ret_id: ShapeId) {
        self.begin_record(RecOp::CreateHullShape as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_shape_def(def);
        self.buffer.append_u32(geometry_id);
        self.buffer.append_shape_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateMeshShape` op.
    pub fn write_create_mesh_shape(&mut self, body: BodyId, def: &ShapeDef, geometry_id: u32, scale: Vec3, ret_id: ShapeId) {
        self.begin_record(RecOp::CreateMeshShape as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_shape_def(def);
        self.buffer.append_u32(geometry_id);
        self.buffer.append_vec3(scale);
        self.buffer.append_shape_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateHeightFieldShape` op.
    pub fn write_create_height_field_shape(&mut self, body: BodyId, def: &ShapeDef, geometry_id: u32, ret_id: ShapeId) {
        self.begin_record(RecOp::CreateHeightFieldShape as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_shape_def(def);
        self.buffer.append_u32(geometry_id);
        self.buffer.append_shape_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateCompoundShape` op.
    pub fn write_create_compound_shape(&mut self, body: BodyId, def: &ShapeDef, geometry_id: u32, ret_id: ShapeId) {
        self.begin_record(RecOp::CreateCompoundShape as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_shape_def(def);
        self.buffer.append_u32(geometry_id);
        self.buffer.append_shape_id(ret_id);
        self.end_record();
    }

    /// Write framed `DestroyShape` op.
    pub fn write_destroy_shape(&mut self, shape: ShapeId, update_body_mass: bool) {
        self.begin_record(RecOp::DestroyShape as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_bool(update_body_mass);
        self.end_record();
    }

    /// Write framed `ShapeSetDensity` op.
    pub fn write_shape_set_density(&mut self, shape: ShapeId, density: f32, update_body_mass: bool) {
        self.begin_record(RecOp::ShapeSetDensity as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_f32(density);
        self.buffer.append_bool(update_body_mass);
        self.end_record();
    }

    /// Write framed `ShapeSetFriction` op.
    pub fn write_shape_set_friction(&mut self, shape: ShapeId, friction: f32) {
        self.begin_record(RecOp::ShapeSetFriction as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_f32(friction);
        self.end_record();
    }

    /// Write framed `ShapeSetRestitution` op.
    pub fn write_shape_set_restitution(&mut self, shape: ShapeId, restitution: f32) {
        self.begin_record(RecOp::ShapeSetRestitution as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_f32(restitution);
        self.end_record();
    }

    /// Write framed `ShapeSetSurfaceMaterial` op.
    pub fn write_shape_set_surface_material(&mut self, shape: ShapeId, material: SurfaceMaterial) {
        self.begin_record(RecOp::ShapeSetSurfaceMaterial as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_material(material);
        self.end_record();
    }

    /// Write framed `ShapeSetFilter` op.
    pub fn write_shape_set_filter(&mut self, shape: ShapeId, filter: Filter, invoke_contacts: bool) {
        self.begin_record(RecOp::ShapeSetFilter as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_filter(filter);
        self.buffer.append_bool(invoke_contacts);
        self.end_record();
    }

    /// Write framed `ShapeEnableSensorEvents` op.
    pub fn write_shape_enable_sensor_events(&mut self, shape: ShapeId, flag: bool) {
        self.begin_record(RecOp::ShapeEnableSensorEvents as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `ShapeEnableContactEvents` op.
    pub fn write_shape_enable_contact_events(&mut self, shape: ShapeId, flag: bool) {
        self.begin_record(RecOp::ShapeEnableContactEvents as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `ShapeEnablePreSolveEvents` op.
    pub fn write_shape_enable_pre_solve_events(&mut self, shape: ShapeId, flag: bool) {
        self.begin_record(RecOp::ShapeEnablePreSolveEvents as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `ShapeEnableHitEvents` op.
    pub fn write_shape_enable_hit_events(&mut self, shape: ShapeId, flag: bool) {
        self.begin_record(RecOp::ShapeEnableHitEvents as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `ShapeSetSphere` op.
    pub fn write_shape_set_sphere(&mut self, shape: ShapeId, sphere: Sphere) {
        self.begin_record(RecOp::ShapeSetSphere as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_sphere(sphere);
        self.end_record();
    }

    /// Write framed `ShapeSetCapsule` op.
    pub fn write_shape_set_capsule(&mut self, shape: ShapeId, capsule: Capsule) {
        self.begin_record(RecOp::ShapeSetCapsule as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_capsule(capsule);
        self.end_record();
    }

    /// Write framed `ShapeApplyWind` op.
    pub fn write_shape_apply_wind(&mut self, shape: ShapeId, wind: Vec3, drag: f32, lift: f32, max_speed: f32, wake: bool) {
        self.begin_record(RecOp::ShapeApplyWind as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_vec3(wind);
        self.buffer.append_f32(drag);
        self.buffer.append_f32(lift);
        self.buffer.append_f32(max_speed);
        self.buffer.append_bool(wake);
        self.end_record();
    }

    /// Write framed `ShapeSetName` op.
    pub fn write_shape_set_name(&mut self, shape: ShapeId, name: &str) {
        self.begin_record(RecOp::ShapeSetName as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_str(name);
        self.end_record();
    }

    /// Write framed `CreateParallelJoint` op.
    pub fn write_create_parallel_joint(&mut self, world: WorldId, def: &ParallelJointDef, ret_id: JointId) {
        self.begin_record(RecOp::CreateParallelJoint as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_parallel_joint_def(def);
        self.buffer.append_joint_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateDistanceJoint` op.
    pub fn write_create_distance_joint(&mut self, world: WorldId, def: &DistanceJointDef, ret_id: JointId) {
        self.begin_record(RecOp::CreateDistanceJoint as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_distance_joint_def(def);
        self.buffer.append_joint_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateFilterJoint` op.
    pub fn write_create_filter_joint(&mut self, world: WorldId, def: &FilterJointDef, ret_id: JointId) {
        self.begin_record(RecOp::CreateFilterJoint as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_filter_joint_def(def);
        self.buffer.append_joint_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateMotorJoint` op.
    pub fn write_create_motor_joint(&mut self, world: WorldId, def: &MotorJointDef, ret_id: JointId) {
        self.begin_record(RecOp::CreateMotorJoint as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_motor_joint_def(def);
        self.buffer.append_joint_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreatePrismaticJoint` op.
    pub fn write_create_prismatic_joint(&mut self, world: WorldId, def: &PrismaticJointDef, ret_id: JointId) {
        self.begin_record(RecOp::CreatePrismaticJoint as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_prismatic_joint_def(def);
        self.buffer.append_joint_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateRevoluteJoint` op.
    pub fn write_create_revolute_joint(&mut self, world: WorldId, def: &RevoluteJointDef, ret_id: JointId) {
        self.begin_record(RecOp::CreateRevoluteJoint as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_revolute_joint_def(def);
        self.buffer.append_joint_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateSphericalJoint` op.
    pub fn write_create_spherical_joint(&mut self, world: WorldId, def: &SphericalJointDef, ret_id: JointId) {
        self.begin_record(RecOp::CreateSphericalJoint as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_spherical_joint_def(def);
        self.buffer.append_joint_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateWeldJoint` op.
    pub fn write_create_weld_joint(&mut self, world: WorldId, def: &WeldJointDef, ret_id: JointId) {
        self.begin_record(RecOp::CreateWeldJoint as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_weld_joint_def(def);
        self.buffer.append_joint_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateWheelJoint` op.
    pub fn write_create_wheel_joint(&mut self, world: WorldId, def: &WheelJointDef, ret_id: JointId) {
        self.begin_record(RecOp::CreateWheelJoint as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_wheel_joint_def(def);
        self.buffer.append_joint_id(ret_id);
        self.end_record();
    }

    /// Write framed `DestroyJoint` op.
    pub fn write_destroy_joint(&mut self, joint: JointId, wake_attached: bool) {
        self.begin_record(RecOp::DestroyJoint as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(wake_attached);
        self.end_record();
    }

    /// Write framed `JointSetLocalFrameA` op.
    pub fn write_joint_set_local_frame_a(&mut self, joint: JointId, local_frame: Transform) {
        self.begin_record(RecOp::JointSetLocalFrameA as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_transform(local_frame);
        self.end_record();
    }

    /// Write framed `JointSetLocalFrameB` op.
    pub fn write_joint_set_local_frame_b(&mut self, joint: JointId, local_frame: Transform) {
        self.begin_record(RecOp::JointSetLocalFrameB as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_transform(local_frame);
        self.end_record();
    }

    /// Write framed `JointSetCollideConnected` op.
    pub fn write_joint_set_collide_connected(&mut self, joint: JointId, should_collide: bool) {
        self.begin_record(RecOp::JointSetCollideConnected as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(should_collide);
        self.end_record();
    }

    /// Write framed `JointWakeBodies` op.
    pub fn write_joint_wake_bodies(&mut self, joint: JointId) {
        self.begin_record(RecOp::JointWakeBodies as u8);
        self.buffer.append_joint_id(joint);
        self.end_record();
    }

    /// Write framed `JointSetConstraintTuning` op.
    pub fn write_joint_set_constraint_tuning(&mut self, joint: JointId, hertz: f32, damping_ratio: f32) {
        self.begin_record(RecOp::JointSetConstraintTuning as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `JointSetForceThreshold` op.
    pub fn write_joint_set_force_threshold(&mut self, joint: JointId, threshold: f32) {
        self.begin_record(RecOp::JointSetForceThreshold as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(threshold);
        self.end_record();
    }

    /// Write framed `JointSetTorqueThreshold` op.
    pub fn write_joint_set_torque_threshold(&mut self, joint: JointId, threshold: f32) {
        self.begin_record(RecOp::JointSetTorqueThreshold as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(threshold);
        self.end_record();
    }

    /// Write framed `ParallelJointSetSpringHertz` op.
    pub fn write_parallel_joint_set_spring_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::ParallelJointSetSpringHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `ParallelJointSetSpringDampingRatio` op.
    pub fn write_parallel_joint_set_spring_damping_ratio(&mut self, joint: JointId, damping_ratio: f32) {
        self.begin_record(RecOp::ParallelJointSetSpringDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `ParallelJointSetMaxTorque` op.
    pub fn write_parallel_joint_set_max_torque(&mut self, joint: JointId, max_torque: f32) {
        self.begin_record(RecOp::ParallelJointSetMaxTorque as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(max_torque);
        self.end_record();
    }

    /// Write framed `DistanceJointSetLength` op.
    pub fn write_distance_joint_set_length(&mut self, joint: JointId, length: f32) {
        self.begin_record(RecOp::DistanceJointSetLength as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(length);
        self.end_record();
    }

    /// Write framed `DistanceJointEnableSpring` op.
    pub fn write_distance_joint_enable_spring(&mut self, joint: JointId, enable_spring: bool) {
        self.begin_record(RecOp::DistanceJointEnableSpring as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_spring);
        self.end_record();
    }

    /// Write framed `DistanceJointSetSpringForceRange` op.
    pub fn write_distance_joint_set_spring_force_range(&mut self, joint: JointId, lower_force: f32, upper_force: f32) {
        self.begin_record(RecOp::DistanceJointSetSpringForceRange as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(lower_force);
        self.buffer.append_f32(upper_force);
        self.end_record();
    }

    /// Write framed `DistanceJointSetSpringHertz` op.
    pub fn write_distance_joint_set_spring_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::DistanceJointSetSpringHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `DistanceJointSetSpringDampingRatio` op.
    pub fn write_distance_joint_set_spring_damping_ratio(&mut self, joint: JointId, damping_ratio: f32) {
        self.begin_record(RecOp::DistanceJointSetSpringDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `DistanceJointEnableLimit` op.
    pub fn write_distance_joint_enable_limit(&mut self, joint: JointId, enable_limit: bool) {
        self.begin_record(RecOp::DistanceJointEnableLimit as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_limit);
        self.end_record();
    }

    /// Write framed `DistanceJointSetLengthRange` op.
    pub fn write_distance_joint_set_length_range(&mut self, joint: JointId, min_length: f32, max_length: f32) {
        self.begin_record(RecOp::DistanceJointSetLengthRange as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(min_length);
        self.buffer.append_f32(max_length);
        self.end_record();
    }

    /// Write framed `DistanceJointEnableMotor` op.
    pub fn write_distance_joint_enable_motor(&mut self, joint: JointId, enable_motor: bool) {
        self.begin_record(RecOp::DistanceJointEnableMotor as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_motor);
        self.end_record();
    }

    /// Write framed `DistanceJointSetMotorSpeed` op.
    pub fn write_distance_joint_set_motor_speed(&mut self, joint: JointId, motor_speed: f32) {
        self.begin_record(RecOp::DistanceJointSetMotorSpeed as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(motor_speed);
        self.end_record();
    }

    /// Write framed `DistanceJointSetMaxMotorForce` op.
    pub fn write_distance_joint_set_max_motor_force(&mut self, joint: JointId, force: f32) {
        self.begin_record(RecOp::DistanceJointSetMaxMotorForce as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(force);
        self.end_record();
    }

    /// Write framed `MotorJointSetLinearVelocity` op.
    pub fn write_motor_joint_set_linear_velocity(&mut self, joint: JointId, velocity: Vec3) {
        self.begin_record(RecOp::MotorJointSetLinearVelocity as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_vec3(velocity);
        self.end_record();
    }

    /// Write framed `MotorJointSetAngularVelocity` op.
    pub fn write_motor_joint_set_angular_velocity(&mut self, joint: JointId, velocity: Vec3) {
        self.begin_record(RecOp::MotorJointSetAngularVelocity as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_vec3(velocity);
        self.end_record();
    }

    /// Write framed `MotorJointSetMaxVelocityForce` op.
    pub fn write_motor_joint_set_max_velocity_force(&mut self, joint: JointId, max_force: f32) {
        self.begin_record(RecOp::MotorJointSetMaxVelocityForce as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(max_force);
        self.end_record();
    }

    /// Write framed `MotorJointSetMaxVelocityTorque` op.
    pub fn write_motor_joint_set_max_velocity_torque(&mut self, joint: JointId, max_torque: f32) {
        self.begin_record(RecOp::MotorJointSetMaxVelocityTorque as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(max_torque);
        self.end_record();
    }

    /// Write framed `MotorJointSetLinearHertz` op.
    pub fn write_motor_joint_set_linear_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::MotorJointSetLinearHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `MotorJointSetLinearDampingRatio` op.
    pub fn write_motor_joint_set_linear_damping_ratio(&mut self, joint: JointId, damping: f32) {
        self.begin_record(RecOp::MotorJointSetLinearDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping);
        self.end_record();
    }

    /// Write framed `MotorJointSetAngularHertz` op.
    pub fn write_motor_joint_set_angular_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::MotorJointSetAngularHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `MotorJointSetAngularDampingRatio` op.
    pub fn write_motor_joint_set_angular_damping_ratio(&mut self, joint: JointId, damping: f32) {
        self.begin_record(RecOp::MotorJointSetAngularDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping);
        self.end_record();
    }

    /// Write framed `MotorJointSetMaxSpringForce` op.
    pub fn write_motor_joint_set_max_spring_force(&mut self, joint: JointId, max_force: f32) {
        self.begin_record(RecOp::MotorJointSetMaxSpringForce as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(max_force);
        self.end_record();
    }

    /// Write framed `MotorJointSetMaxSpringTorque` op.
    pub fn write_motor_joint_set_max_spring_torque(&mut self, joint: JointId, max_torque: f32) {
        self.begin_record(RecOp::MotorJointSetMaxSpringTorque as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(max_torque);
        self.end_record();
    }

    /// Write framed `PrismaticJointEnableSpring` op.
    pub fn write_prismatic_joint_enable_spring(&mut self, joint: JointId, enable_spring: bool) {
        self.begin_record(RecOp::PrismaticJointEnableSpring as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_spring);
        self.end_record();
    }

    /// Write framed `PrismaticJointSetSpringHertz` op.
    pub fn write_prismatic_joint_set_spring_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::PrismaticJointSetSpringHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `PrismaticJointSetSpringDampingRatio` op.
    pub fn write_prismatic_joint_set_spring_damping_ratio(&mut self, joint: JointId, damping_ratio: f32) {
        self.begin_record(RecOp::PrismaticJointSetSpringDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `PrismaticJointSetTargetTranslation` op.
    pub fn write_prismatic_joint_set_target_translation(&mut self, joint: JointId, translation: f32) {
        self.begin_record(RecOp::PrismaticJointSetTargetTranslation as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(translation);
        self.end_record();
    }

    /// Write framed `PrismaticJointEnableLimit` op.
    pub fn write_prismatic_joint_enable_limit(&mut self, joint: JointId, enable_limit: bool) {
        self.begin_record(RecOp::PrismaticJointEnableLimit as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_limit);
        self.end_record();
    }

    /// Write framed `PrismaticJointSetLimits` op.
    pub fn write_prismatic_joint_set_limits(&mut self, joint: JointId, lower: f32, upper: f32) {
        self.begin_record(RecOp::PrismaticJointSetLimits as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(lower);
        self.buffer.append_f32(upper);
        self.end_record();
    }

    /// Write framed `PrismaticJointEnableMotor` op.
    pub fn write_prismatic_joint_enable_motor(&mut self, joint: JointId, enable_motor: bool) {
        self.begin_record(RecOp::PrismaticJointEnableMotor as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_motor);
        self.end_record();
    }

    /// Write framed `PrismaticJointSetMotorSpeed` op.
    pub fn write_prismatic_joint_set_motor_speed(&mut self, joint: JointId, motor_speed: f32) {
        self.begin_record(RecOp::PrismaticJointSetMotorSpeed as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(motor_speed);
        self.end_record();
    }

    /// Write framed `PrismaticJointSetMaxMotorForce` op.
    pub fn write_prismatic_joint_set_max_motor_force(&mut self, joint: JointId, force: f32) {
        self.begin_record(RecOp::PrismaticJointSetMaxMotorForce as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(force);
        self.end_record();
    }

    /// Write framed `RevoluteJointEnableSpring` op.
    pub fn write_revolute_joint_enable_spring(&mut self, joint: JointId, enable_spring: bool) {
        self.begin_record(RecOp::RevoluteJointEnableSpring as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_spring);
        self.end_record();
    }

    /// Write framed `RevoluteJointSetSpringHertz` op.
    pub fn write_revolute_joint_set_spring_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::RevoluteJointSetSpringHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `RevoluteJointSetSpringDampingRatio` op.
    pub fn write_revolute_joint_set_spring_damping_ratio(&mut self, joint: JointId, damping_ratio: f32) {
        self.begin_record(RecOp::RevoluteJointSetSpringDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `RevoluteJointSetTargetAngle` op.
    pub fn write_revolute_joint_set_target_angle(&mut self, joint: JointId, angle: f32) {
        self.begin_record(RecOp::RevoluteJointSetTargetAngle as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(angle);
        self.end_record();
    }

    /// Write framed `RevoluteJointEnableLimit` op.
    pub fn write_revolute_joint_enable_limit(&mut self, joint: JointId, enable_limit: bool) {
        self.begin_record(RecOp::RevoluteJointEnableLimit as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_limit);
        self.end_record();
    }

    /// Write framed `RevoluteJointSetLimits` op.
    pub fn write_revolute_joint_set_limits(&mut self, joint: JointId, lower: f32, upper: f32) {
        self.begin_record(RecOp::RevoluteJointSetLimits as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(lower);
        self.buffer.append_f32(upper);
        self.end_record();
    }

    /// Write framed `RevoluteJointEnableMotor` op.
    pub fn write_revolute_joint_enable_motor(&mut self, joint: JointId, enable_motor: bool) {
        self.begin_record(RecOp::RevoluteJointEnableMotor as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_motor);
        self.end_record();
    }

    /// Write framed `RevoluteJointSetMotorSpeed` op.
    pub fn write_revolute_joint_set_motor_speed(&mut self, joint: JointId, motor_speed: f32) {
        self.begin_record(RecOp::RevoluteJointSetMotorSpeed as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(motor_speed);
        self.end_record();
    }

    /// Write framed `RevoluteJointSetMaxMotorTorque` op.
    pub fn write_revolute_joint_set_max_motor_torque(&mut self, joint: JointId, torque: f32) {
        self.begin_record(RecOp::RevoluteJointSetMaxMotorTorque as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(torque);
        self.end_record();
    }

    /// Write framed `SphericalJointEnableConeLimit` op.
    pub fn write_spherical_joint_enable_cone_limit(&mut self, joint: JointId, enable_limit: bool) {
        self.begin_record(RecOp::SphericalJointEnableConeLimit as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_limit);
        self.end_record();
    }

    /// Write framed `SphericalJointSetConeLimit` op.
    pub fn write_spherical_joint_set_cone_limit(&mut self, joint: JointId, angle_radians: f32) {
        self.begin_record(RecOp::SphericalJointSetConeLimit as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(angle_radians);
        self.end_record();
    }

    /// Write framed `SphericalJointEnableTwistLimit` op.
    pub fn write_spherical_joint_enable_twist_limit(&mut self, joint: JointId, enable_limit: bool) {
        self.begin_record(RecOp::SphericalJointEnableTwistLimit as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_limit);
        self.end_record();
    }

    /// Write framed `SphericalJointSetTwistLimits` op.
    pub fn write_spherical_joint_set_twist_limits(&mut self, joint: JointId, lower: f32, upper: f32) {
        self.begin_record(RecOp::SphericalJointSetTwistLimits as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(lower);
        self.buffer.append_f32(upper);
        self.end_record();
    }

    /// Write framed `SphericalJointEnableSpring` op.
    pub fn write_spherical_joint_enable_spring(&mut self, joint: JointId, enable_spring: bool) {
        self.begin_record(RecOp::SphericalJointEnableSpring as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_spring);
        self.end_record();
    }

    /// Write framed `SphericalJointSetSpringHertz` op.
    pub fn write_spherical_joint_set_spring_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::SphericalJointSetSpringHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `SphericalJointSetSpringDampingRatio` op.
    pub fn write_spherical_joint_set_spring_damping_ratio(&mut self, joint: JointId, damping_ratio: f32) {
        self.begin_record(RecOp::SphericalJointSetSpringDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `SphericalJointSetTargetRotation` op.
    pub fn write_spherical_joint_set_target_rotation(&mut self, joint: JointId, target_rotation: Quat) {
        self.begin_record(RecOp::SphericalJointSetTargetRotation as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_quat(target_rotation);
        self.end_record();
    }

    /// Write framed `SphericalJointEnableMotor` op.
    pub fn write_spherical_joint_enable_motor(&mut self, joint: JointId, enable_motor: bool) {
        self.begin_record(RecOp::SphericalJointEnableMotor as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(enable_motor);
        self.end_record();
    }

    /// Write framed `SphericalJointSetMotorVelocity` op.
    pub fn write_spherical_joint_set_motor_velocity(&mut self, joint: JointId, motor_velocity: Vec3) {
        self.begin_record(RecOp::SphericalJointSetMotorVelocity as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_vec3(motor_velocity);
        self.end_record();
    }

    /// Write framed `SphericalJointSetMaxMotorTorque` op.
    pub fn write_spherical_joint_set_max_motor_torque(&mut self, joint: JointId, torque: f32) {
        self.begin_record(RecOp::SphericalJointSetMaxMotorTorque as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(torque);
        self.end_record();
    }

    /// Write framed `WeldJointSetLinearHertz` op.
    pub fn write_weld_joint_set_linear_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::WeldJointSetLinearHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `WeldJointSetLinearDampingRatio` op.
    pub fn write_weld_joint_set_linear_damping_ratio(&mut self, joint: JointId, damping_ratio: f32) {
        self.begin_record(RecOp::WeldJointSetLinearDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `WeldJointSetAngularHertz` op.
    pub fn write_weld_joint_set_angular_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::WeldJointSetAngularHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `WeldJointSetAngularDampingRatio` op.
    pub fn write_weld_joint_set_angular_damping_ratio(&mut self, joint: JointId, damping_ratio: f32) {
        self.begin_record(RecOp::WeldJointSetAngularDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `WheelJointEnableSuspension` op.
    pub fn write_wheel_joint_enable_suspension(&mut self, joint: JointId, flag: bool) {
        self.begin_record(RecOp::WheelJointEnableSuspension as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `WheelJointSetSuspensionHertz` op.
    pub fn write_wheel_joint_set_suspension_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::WheelJointSetSuspensionHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `WheelJointSetSuspensionDampingRatio` op.
    pub fn write_wheel_joint_set_suspension_damping_ratio(&mut self, joint: JointId, damping_ratio: f32) {
        self.begin_record(RecOp::WheelJointSetSuspensionDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `WheelJointEnableSuspensionLimit` op.
    pub fn write_wheel_joint_enable_suspension_limit(&mut self, joint: JointId, flag: bool) {
        self.begin_record(RecOp::WheelJointEnableSuspensionLimit as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `WheelJointSetSuspensionLimits` op.
    pub fn write_wheel_joint_set_suspension_limits(&mut self, joint: JointId, lower: f32, upper: f32) {
        self.begin_record(RecOp::WheelJointSetSuspensionLimits as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(lower);
        self.buffer.append_f32(upper);
        self.end_record();
    }

    /// Write framed `WheelJointEnableSpinMotor` op.
    pub fn write_wheel_joint_enable_spin_motor(&mut self, joint: JointId, flag: bool) {
        self.begin_record(RecOp::WheelJointEnableSpinMotor as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `WheelJointSetSpinMotorSpeed` op.
    pub fn write_wheel_joint_set_spin_motor_speed(&mut self, joint: JointId, speed: f32) {
        self.begin_record(RecOp::WheelJointSetSpinMotorSpeed as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(speed);
        self.end_record();
    }

    /// Write framed `WheelJointSetMaxSpinTorque` op.
    pub fn write_wheel_joint_set_max_spin_torque(&mut self, joint: JointId, torque: f32) {
        self.begin_record(RecOp::WheelJointSetMaxSpinTorque as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(torque);
        self.end_record();
    }

    /// Write framed `WheelJointEnableSteering` op.
    pub fn write_wheel_joint_enable_steering(&mut self, joint: JointId, flag: bool) {
        self.begin_record(RecOp::WheelJointEnableSteering as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `WheelJointSetSteeringHertz` op.
    pub fn write_wheel_joint_set_steering_hertz(&mut self, joint: JointId, hertz: f32) {
        self.begin_record(RecOp::WheelJointSetSteeringHertz as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(hertz);
        self.end_record();
    }

    /// Write framed `WheelJointSetSteeringDampingRatio` op.
    pub fn write_wheel_joint_set_steering_damping_ratio(&mut self, joint: JointId, damping_ratio: f32) {
        self.begin_record(RecOp::WheelJointSetSteeringDampingRatio as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(damping_ratio);
        self.end_record();
    }

    /// Write framed `WheelJointSetMaxSteeringTorque` op.
    pub fn write_wheel_joint_set_max_steering_torque(&mut self, joint: JointId, torque: f32) {
        self.begin_record(RecOp::WheelJointSetMaxSteeringTorque as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(torque);
        self.end_record();
    }

    /// Write framed `WheelJointEnableSteeringLimit` op.
    pub fn write_wheel_joint_enable_steering_limit(&mut self, joint: JointId, flag: bool) {
        self.begin_record(RecOp::WheelJointEnableSteeringLimit as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `WheelJointSetSteeringLimits` op.
    pub fn write_wheel_joint_set_steering_limits(&mut self, joint: JointId, lower: f32, upper: f32) {
        self.begin_record(RecOp::WheelJointSetSteeringLimits as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(lower);
        self.buffer.append_f32(upper);
        self.end_record();
    }

    /// Write framed `WheelJointSetTargetSteeringAngle` op.
    pub fn write_wheel_joint_set_target_steering_angle(&mut self, joint: JointId, radians: f32) {
        self.begin_record(RecOp::WheelJointSetTargetSteeringAngle as u8);
        self.buffer.append_joint_id(joint);
        self.buffer.append_f32(radians);
        self.end_record();
    }

    /// Write framed `QueryOverlapAABB` op.
    pub fn write_query_overlap_aabb(&mut self, world: WorldId, aabb: Aabb, filter: &QueryFilter) {
        self.begin_record(RecOp::QueryOverlapAABB as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_aabb(aabb);
        self.buffer.append_query_filter(filter);
        self.end_record();
    }

    /// Write framed `QueryOverlapShape` op.
    pub fn write_query_overlap_shape(&mut self, world: WorldId, origin: Pos, proxy: &ShapeProxy, filter: &QueryFilter) {
        self.begin_record(RecOp::QueryOverlapShape as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_pos(origin);
        self.buffer.append_shape_proxy(proxy);
        self.buffer.append_query_filter(filter);
        self.end_record();
    }

    /// Write framed `QueryCastRay` op.
    pub fn write_query_cast_ray(&mut self, world: WorldId, origin: Pos, translation: Vec3, filter: &QueryFilter) {
        self.begin_record(RecOp::QueryCastRay as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_pos(origin);
        self.buffer.append_vec3(translation);
        self.buffer.append_query_filter(filter);
        self.end_record();
    }

    /// Write framed `QueryCastShape` op.
    pub fn write_query_cast_shape(&mut self, world: WorldId, origin: Pos, proxy: &ShapeProxy, translation: Vec3, filter: &QueryFilter) {
        self.begin_record(RecOp::QueryCastShape as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_pos(origin);
        self.buffer.append_shape_proxy(proxy);
        self.buffer.append_vec3(translation);
        self.buffer.append_query_filter(filter);
        self.end_record();
    }

    /// Write framed `QueryCastRayClosest` op.
    pub fn write_query_cast_ray_closest(&mut self, world: WorldId, origin: Pos, translation: Vec3, filter: &QueryFilter) {
        self.begin_record(RecOp::QueryCastRayClosest as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_pos(origin);
        self.buffer.append_vec3(translation);
        self.buffer.append_query_filter(filter);
        self.end_record();
    }

    /// Write framed `QueryCastMover` op.
    pub fn write_query_cast_mover(&mut self, world: WorldId, origin: Pos, mover: Capsule, translation: Vec3, filter: &QueryFilter) {
        self.begin_record(RecOp::QueryCastMover as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_pos(origin);
        self.buffer.append_capsule(mover);
        self.buffer.append_vec3(translation);
        self.buffer.append_query_filter(filter);
        self.end_record();
    }

    /// Write framed `QueryCollideMover` op.
    pub fn write_query_collide_mover(&mut self, world: WorldId, origin: Pos, mover: Capsule, filter: &QueryFilter) {
        self.begin_record(RecOp::QueryCollideMover as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_pos(origin);
        self.buffer.append_capsule(mover);
        self.buffer.append_query_filter(filter);
        self.end_record();
    }

    /// Write framed `QueryTag` op.
    pub fn write_query_tag(&mut self, key: u64) {
        self.begin_record(RecOp::QueryTag as u8);
        self.buffer.append_u64(key);
        self.end_record();
    }

    /// Write framed `StateHash` op.
    pub fn write_state_hash(&mut self, world: WorldId, hash: u64) {
        self.begin_record(RecOp::StateHash as u8);
        self.buffer.append_world_id(world);
        self.buffer.append_u64(hash);
        self.end_record();
    }

    /// Write framed `RecordingBounds` op.
    pub fn write_recording_bounds(&mut self, bounds: Aabb) {
        self.begin_record(RecOp::RecordingBounds as u8);
        self.buffer.append_aabb(bounds);
        self.end_record();
    }

}
