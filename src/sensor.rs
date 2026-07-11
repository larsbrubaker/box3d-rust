// Port of the sensor data model from box3d-cpp-reference/src/sensor.h.
// Overlap update logic lands in a later bring-up commit.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::bitset::BitSet;
use crate::core::NULL_INDEX;

/// Used to track shapes that hit sensors using time of impact. (b3SensorHit)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorHit {
    pub sensor_id: i32,
    pub visitor_id: i32,
}

impl Default for SensorHit {
    fn default() -> Self {
        SensorHit {
            sensor_id: NULL_INDEX,
            visitor_id: NULL_INDEX,
        }
    }
}

/// (b3Visitor)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Visitor {
    pub shape_id: i32,
    pub generation: u16,
}

impl Default for Visitor {
    fn default() -> Self {
        Visitor {
            shape_id: NULL_INDEX,
            generation: 0,
        }
    }
}

/// Sensors are shapes that live in the broad-phase but never have contacts.
/// (b3Sensor)
#[derive(Debug, Clone, Default)]
pub struct Sensor {
    pub hits: Vec<Visitor>,
    pub overlaps1: Vec<Visitor>,
    pub overlaps2: Vec<Visitor>,
    pub shape_id: i32,
}

impl Sensor {
    pub fn new(shape_id: i32) -> Sensor {
        Sensor {
            hits: Vec::new(),
            overlaps1: Vec::new(),
            overlaps2: Vec::new(),
            shape_id,
        }
    }
}

/// (b3SensorTaskContext)
#[derive(Debug, Clone, Default)]
pub struct SensorTaskContext {
    pub event_bits: BitSet,
}
