//! Softness / make_soft and the step context from solver.h.
//! Integration and the serial solve driver live in submodules.
//!
//! The C b3StepContext carries multithreading scratch. The single-threaded port
//! keeps only the step parameters here; solver scratch is owned locally by the
//! solve pass and world data is passed as separate arguments.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

mod integrate;
mod solve;

use crate::math_functions::PI;

pub use solve::solve;

/// Soft constraint coefficients. (b3Softness)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Softness {
    pub bias_rate: f32,
    pub mass_scale: f32,
    pub impulse_scale: f32,
}

/// (static inline b3MakeSoft)
pub fn make_soft(hertz: f32, zeta: f32, h: f32) -> Softness {
    if hertz == 0.0 {
        return Softness {
            bias_rate: 0.0,
            mass_scale: 0.0,
            impulse_scale: 0.0,
        };
    }

    let omega = 2.0 * PI * hertz;
    let a1 = 2.0 * zeta + h * omega;
    let a2 = h * omega * a1;
    let a3 = 1.0 / (1.0 + a2);

    // bias = w / (2 * z + hw)
    // massScale = hw * (2 * z + hw) / (1 + hw * (2 * z + hw))
    // impulseScale = 1 / (1 + hw * (2 * z + hw))
    Softness {
        bias_rate: omega / a1,
        mass_scale: a2 * a3,
        impulse_scale: a3,
    }
}

/// Solver stage type. (b3SolverStageType)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum SolverStageType {
    PrepareJoints = 0,
    PrepareWideContacts = 1,
    PrepareContacts = 2,
    IntegrateVelocities = 3,
    WarmStart = 4,
    Solve = 5,
    IntegratePositions = 6,
    Relax = 7,
    Restitution = 8,
    StoreWideImpulses = 9,
    StoreImpulses = 10,
}

/// Solver block type. (b3SolverBlockType)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SolverBlockType {
    BodyBlock = 0,
    JointBlock = 1,
    WideContactBlock = 2,
    ContactBlock = 3,
    GraphJointBlock = 4,
    GraphWideContactBlock = 5,
    GraphContactBlock = 6,
    OverflowBlock = 7,
}

/// Solver block describes a unit of work. (b3SolverBlock)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SolverBlock {
    pub start_index: i32,
    pub count: u16,
    /// SolverBlockType
    pub block_type: u8,
    pub color_index: u8,
}

/// Context for a time step. Recreated each time step. (b3StepContext — step
/// parameters only; see the module header for what the serial port drops)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct StepContext {
    /// time step
    pub dt: f32,

    /// inverse time step (0 if dt == 0).
    pub inv_dt: f32,

    /// sub-step
    pub h: f32,
    pub inv_h: f32,

    pub sub_step_count: i32,

    pub contact_softness: Softness,
    pub static_softness: Softness,

    pub restitution_threshold: f32,
    pub max_linear_velocity: f32,
    pub contact_speed: f32,

    pub enable_warm_starting: bool,
}
