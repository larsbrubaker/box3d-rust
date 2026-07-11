// Port of the event types from include/box3d/types.h (events group).
//
// The C aggregate views (b3SensorEvents, b3ContactEvents, b3BodyEvents,
// b3JointEvents) are pointer+count pairs over world-owned arrays. The Rust
// world API returns slices; BodyEvents/JointEvents collapse to a single
// slice while SensorEvents/ContactEvents keep grouping structs because they
// bundle several arrays.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::id::{BodyId, ContactId, JointId, ShapeId};
use crate::math_functions::{Pos, Vec3, WorldTransform};

/// A begin-touch event is generated when a shape starts to overlap a sensor
/// shape. (b3SensorBeginTouchEvent)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorBeginTouchEvent {
    /// The id of the sensor shape
    pub sensor_shape_id: ShapeId,
    /// The id of the shape that began touching the sensor shape
    pub visitor_shape_id: ShapeId,
}

/// An end touch event is generated when a shape stops overlapping a sensor
/// shape. Always confirm the shape id is valid before use. (b3SensorEndTouchEvent)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorEndTouchEvent {
    /// The id of the sensor shape. @warning may have been destroyed
    pub sensor_shape_id: ShapeId,
    /// The id of the shape that stopped touching. @warning may have been destroyed
    pub visitor_shape_id: ShapeId,
}

/// A begin-touch event is generated when two shapes begin touching.
/// (b3ContactBeginTouchEvent)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ContactBeginTouchEvent {
    /// Id of the first shape
    pub shape_id_a: ShapeId,
    /// Id of the second shape
    pub shape_id_b: ShapeId,
    /// The transient contact id. May be destroyed automatically when the world
    /// is modified or simulated.
    pub contact_id: ContactId,
}

/// An end touch event is generated when two shapes stop touching.
/// (b3ContactEndTouchEvent)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ContactEndTouchEvent {
    /// Id of the first shape. @warning may have been destroyed
    pub shape_id_a: ShapeId,
    /// Id of the second shape. @warning may have been destroyed
    pub shape_id_b: ShapeId,
    /// Id of the contact. @warning may have been destroyed
    pub contact_id: ContactId,
}

/// A hit touch event is generated when two shapes collide faster than the hit
/// speed threshold. (b3ContactHitEvent)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactHitEvent {
    /// Id of the first shape
    pub shape_id_a: ShapeId,
    /// Id of the second shape
    pub shape_id_b: ShapeId,
    /// Id of the contact. @warning may have been destroyed
    pub contact_id: ContactId,
    /// Point where the shapes hit, a mid-point between the two surfaces.
    pub point: Pos,
    /// Normal vector pointing from shape A to shape B
    pub normal: Vec3,
    /// The speed the shapes are approaching. Always positive.
    pub approach_speed: f32,
    /// User material on shape A
    pub user_material_id_a: u64,
    /// User material on shape B
    pub user_material_id_b: u64,
}

/// Body move event, triggered when a body moves due to simulation. Not
/// reported for bodies moved by the user. (b3BodyMoveEvent)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyMoveEvent {
    pub user_data: u64,
    pub transform: WorldTransform,
    pub body_id: BodyId,
    pub fell_asleep: bool,
}

/// Joint event, reported for awake joints whose force and/or torque exceed the
/// threshold. (b3JointEvent)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct JointEvent {
    /// The joint id
    pub joint_id: JointId,
    /// The user data from the joint for convenience
    pub user_data: u64,
}
