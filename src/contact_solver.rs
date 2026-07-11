// Port of contact_solver.h data model. Constraint kernels land later.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::constants::MAX_MANIFOLD_POINTS;
use crate::core::NULL_INDEX;
use crate::math_functions::{Mat2, Matrix3, Vec2, Vec3, MAT2_ZERO, MAT3_ZERO, VEC2_ZERO, VEC3_ZERO};
use crate::solver::Softness;

/// (b3ManifoldConstraintPoint)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ManifoldConstraintPoint {
    pub r_a: Vec3,
    pub r_b: Vec3,
    pub base_separation: f32,
    pub relative_velocity: f32,
    pub normal_impulse: f32,
    pub total_normal_impulse: f32,
    pub normal_mass: f32,
    pub lever_arm: f32,
}

/// (b3ManifoldConstraint)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ManifoldConstraint {
    pub points: [ManifoldConstraintPoint; MAX_MANIFOLD_POINTS],
    pub point_count: i32,
    pub normal: Vec3,
    pub tangent1: Vec3,
    pub tangent2: Vec3,
    pub origin_a: Vec3,
    pub origin_b: Vec3,
    pub twist_mass: f32,
    pub twist_impulse: f32,
    pub tangent_mass: Mat2,
    pub friction_impulse: Vec2,
    pub rolling_impulse: Vec3,
    pub tangent_velocity1: f32,
    pub tangent_velocity2: f32,
}

impl Default for ManifoldConstraint {
    fn default() -> Self {
        ManifoldConstraint {
            points: [ManifoldConstraintPoint::default(); MAX_MANIFOLD_POINTS],
            point_count: 0,
            normal: VEC3_ZERO,
            tangent1: VEC3_ZERO,
            tangent2: VEC3_ZERO,
            origin_a: VEC3_ZERO,
            origin_b: VEC3_ZERO,
            twist_mass: 0.0,
            twist_impulse: 0.0,
            tangent_mass: MAT2_ZERO,
            friction_impulse: VEC2_ZERO,
            rolling_impulse: VEC3_ZERO,
            tangent_velocity1: 0.0,
            tangent_velocity2: 0.0,
        }
    }
}

/// (b3ContactConstraint)
///
/// C holds `b3ManifoldConstraint* constraints` and `b3Contact* contact`. The
/// Rust data model stores the manifold constraints inline and the contact id;
/// the solver wires slices at step time.
#[derive(Debug, Clone, PartialEq)]
pub struct ContactConstraint {
    pub constraints: Vec<ManifoldConstraint>,
    pub contact_id: i32,
    pub index_a: i32,
    pub index_b: i32,
    pub inv_mass_a: f32,
    pub inv_mass_b: f32,
    pub inv_i_a: Matrix3,
    pub inv_i_b: Matrix3,
    pub softness: Softness,
    pub rolling_mass: Matrix3,
    pub friction: f32,
    pub restitution: f32,
    pub rolling_resistance: f32,
    pub manifold_count: i32,
}

impl Default for ContactConstraint {
    fn default() -> Self {
        ContactConstraint {
            constraints: Vec::new(),
            contact_id: NULL_INDEX,
            index_a: NULL_INDEX,
            index_b: NULL_INDEX,
            inv_mass_a: 0.0,
            inv_mass_b: 0.0,
            inv_i_a: MAT3_ZERO,
            inv_i_b: MAT3_ZERO,
            softness: Softness::default(),
            rolling_mass: MAT3_ZERO,
            friction: 0.0,
            restitution: 0.0,
            rolling_resistance: 0.0,
            manifold_count: 0,
        }
    }
}
