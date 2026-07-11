// Port of the joint data model from box3d-cpp-reference/src/joint.h.
// Lifecycle and plumbing from joint.c; per-type solve in sibling modules.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::NULL_INDEX;
use crate::math_functions::{
    Matrix3, Quat, Transform, Vec2, Vec3, MAT3_ZERO, QUAT_IDENTITY, TRANSFORM_IDENTITY, VEC2_ZERO,
    VEC3_ZERO,
};
use crate::solver::Softness;

/// Joint type enumeration. (types.h: b3JointType)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(i32)]
pub enum JointType {
    #[default]
    Parallel = 0,
    Distance = 1,
    Filter = 2,
    Motor = 3,
    Prismatic = 4,
    Revolute = 5,
    Spherical = 6,
    Weld = 7,
    Wheel = 8,
}

/// A joint edge connects bodies and joints in a joint graph. (b3JointEdge)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct JointEdge {
    pub body_id: i32,
    pub prev_key: i32,
    pub next_key: i32,
}

impl Default for JointEdge {
    fn default() -> Self {
        JointEdge {
            body_id: NULL_INDEX,
            prev_key: NULL_INDEX,
            next_key: NULL_INDEX,
        }
    }
}

/// Map from JointId to joint data in the solver sets. (b3Joint)
#[derive(Debug, Clone)]
pub struct Joint {
    pub user_data: u64,

    /// Index of simulation set stored in World. NULL_INDEX when slot is free.
    pub set_index: i32,

    /// Index into the constraint graph color array; may be NULL_INDEX for
    /// sleeping/disabled joints. NULL_INDEX when slot is free.
    pub color_index: i32,

    /// Joint index within set or graph color. NULL_INDEX when slot is free.
    pub local_index: i32,

    pub edges: [JointEdge; 2],

    pub joint_id: i32,
    pub island_id: i32,

    /// Index into the island's joints array for O(1) swap-removal.
    /// NULL_INDEX when not in an island.
    pub island_index: i32,

    pub draw_scale: f32,

    pub type_: JointType,

    /// Monotonically advanced when a joint is allocated in this slot.
    pub generation: u16,

    pub collide_connected: bool,
}

impl Default for Joint {
    fn default() -> Self {
        Joint {
            user_data: 0,
            set_index: NULL_INDEX,
            color_index: NULL_INDEX,
            local_index: NULL_INDEX,
            edges: [JointEdge::default(); 2],
            joint_id: NULL_INDEX,
            island_id: NULL_INDEX,
            island_index: NULL_INDEX,
            draw_scale: 1.0,
            type_: JointType::Distance,
            generation: 0,
            collide_connected: false,
        }
    }
}

/// (b3DistanceJoint)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct DistanceJoint {
    pub length: f32,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub lower_spring_force: f32,
    pub upper_spring_force: f32,
    pub min_length: f32,
    pub max_length: f32,

    pub max_motor_force: f32,
    pub motor_speed: f32,

    pub impulse: f32,
    pub lower_impulse: f32,
    pub upper_impulse: f32,
    pub motor_impulse: f32,

    pub index_a: i32,
    pub index_b: i32,
    pub anchor_a: Vec3,
    pub anchor_b: Vec3,
    pub delta_center: Vec3,
    pub distance_softness: Softness,
    pub axial_mass: f32,

    pub enable_spring: bool,
    pub enable_limit: bool,
    pub enable_motor: bool,
}

/// (b3MotorJoint)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorJoint {
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub max_velocity_force: f32,
    pub max_velocity_torque: f32,
    pub linear_hertz: f32,
    pub linear_damping_ratio: f32,
    pub max_spring_force: f32,
    pub angular_hertz: f32,
    pub angular_damping_ratio: f32,
    pub max_spring_torque: f32,

    pub linear_velocity_impulse: Vec3,
    pub angular_velocity_impulse: Vec3,
    pub linear_spring_impulse: Vec3,
    pub angular_spring_impulse: Vec3,

    pub linear_spring: Softness,
    pub angular_spring: Softness,

    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub delta_center: Vec3,
    pub angular_mass: Matrix3,
}

impl Default for MotorJoint {
    fn default() -> Self {
        MotorJoint {
            linear_velocity: VEC3_ZERO,
            angular_velocity: VEC3_ZERO,
            max_velocity_force: 0.0,
            max_velocity_torque: 0.0,
            linear_hertz: 0.0,
            linear_damping_ratio: 0.0,
            max_spring_force: 0.0,
            angular_hertz: 0.0,
            angular_damping_ratio: 0.0,
            max_spring_torque: 0.0,
            linear_velocity_impulse: VEC3_ZERO,
            angular_velocity_impulse: VEC3_ZERO,
            linear_spring_impulse: VEC3_ZERO,
            angular_spring_impulse: VEC3_ZERO,
            linear_spring: Softness::default(),
            angular_spring: Softness::default(),
            index_a: NULL_INDEX,
            index_b: NULL_INDEX,
            frame_a: TRANSFORM_IDENTITY,
            frame_b: TRANSFORM_IDENTITY,
            delta_center: VEC3_ZERO,
            angular_mass: MAT3_ZERO,
        }
    }
}

/// (b3ParallelJoint)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParallelJoint {
    pub hertz: f32,
    pub damping_ratio: f32,
    pub max_torque: f32,

    pub perp_impulse: Vec2,
    pub perp_axis_x: Vec3,
    pub perp_axis_y: Vec3,

    pub quat_a: Quat,
    pub quat_b: Quat,
    pub index_a: i32,
    pub index_b: i32,
    pub softness: Softness,
}

impl Default for ParallelJoint {
    fn default() -> Self {
        ParallelJoint {
            hertz: 0.0,
            damping_ratio: 0.0,
            max_torque: 0.0,
            perp_impulse: VEC2_ZERO,
            perp_axis_x: VEC3_ZERO,
            perp_axis_y: VEC3_ZERO,
            quat_a: QUAT_IDENTITY,
            quat_b: QUAT_IDENTITY,
            index_a: NULL_INDEX,
            index_b: NULL_INDEX,
            softness: Softness::default(),
        }
    }
}

/// (b3PrismaticJoint)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PrismaticJoint {
    pub perp_impulse: Vec2,
    pub angular_impulse: Vec3,
    pub spring_impulse: f32,
    pub motor_impulse: f32,
    pub lower_impulse: f32,
    pub upper_impulse: f32,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub max_motor_force: f32,
    pub motor_speed: f32,
    pub target_translation: f32,
    pub lower_translation: f32,
    pub upper_translation: f32,

    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub joint_axis: Vec3,
    pub perp_axis_y: Vec3,
    pub perp_axis_z: Vec3,
    pub delta_center: Vec3,
    pub delta_angle: f32,
    pub rotation_mass: Matrix3,
    pub spring_softness: Softness,

    pub enable_spring: bool,
    pub enable_limit: bool,
    pub enable_motor: bool,
}

impl Default for PrismaticJoint {
    fn default() -> Self {
        PrismaticJoint {
            perp_impulse: VEC2_ZERO,
            angular_impulse: VEC3_ZERO,
            spring_impulse: 0.0,
            motor_impulse: 0.0,
            lower_impulse: 0.0,
            upper_impulse: 0.0,
            hertz: 0.0,
            damping_ratio: 0.0,
            max_motor_force: 0.0,
            motor_speed: 0.0,
            target_translation: 0.0,
            lower_translation: 0.0,
            upper_translation: 0.0,
            index_a: NULL_INDEX,
            index_b: NULL_INDEX,
            frame_a: TRANSFORM_IDENTITY,
            frame_b: TRANSFORM_IDENTITY,
            joint_axis: VEC3_ZERO,
            perp_axis_y: VEC3_ZERO,
            perp_axis_z: VEC3_ZERO,
            delta_center: VEC3_ZERO,
            delta_angle: 0.0,
            rotation_mass: MAT3_ZERO,
            spring_softness: Softness::default(),
            enable_spring: false,
            enable_limit: false,
            enable_motor: false,
        }
    }
}

/// (b3RevoluteJoint)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RevoluteJoint {
    pub linear_impulse: Vec3,
    pub perp_impulse: Vec2,
    pub spring_impulse: f32,
    pub motor_impulse: f32,
    pub lower_impulse: f32,
    pub upper_impulse: f32,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub max_motor_torque: f32,
    pub motor_speed: f32,
    pub target_angle: f32,
    pub lower_angle: f32,
    pub upper_angle: f32,

    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub rotation_axis_z: Vec3,
    pub perp_axis_x: Vec3,
    pub perp_axis_y: Vec3,
    pub delta_center: Vec3,
    pub delta_angle: f32,
    pub axial_mass: f32,
    pub spring_softness: Softness,

    pub enable_spring: bool,
    pub enable_motor: bool,
    pub enable_limit: bool,
}

impl Default for RevoluteJoint {
    fn default() -> Self {
        RevoluteJoint {
            linear_impulse: VEC3_ZERO,
            perp_impulse: VEC2_ZERO,
            spring_impulse: 0.0,
            motor_impulse: 0.0,
            lower_impulse: 0.0,
            upper_impulse: 0.0,
            hertz: 0.0,
            damping_ratio: 0.0,
            max_motor_torque: 0.0,
            motor_speed: 0.0,
            target_angle: 0.0,
            lower_angle: 0.0,
            upper_angle: 0.0,
            index_a: NULL_INDEX,
            index_b: NULL_INDEX,
            frame_a: TRANSFORM_IDENTITY,
            frame_b: TRANSFORM_IDENTITY,
            rotation_axis_z: VEC3_ZERO,
            perp_axis_x: VEC3_ZERO,
            perp_axis_y: VEC3_ZERO,
            delta_center: VEC3_ZERO,
            delta_angle: 0.0,
            axial_mass: 0.0,
            spring_softness: Softness::default(),
            enable_spring: false,
            enable_motor: false,
            enable_limit: false,
        }
    }
}

/// (b3SphericalJoint)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SphericalJoint {
    pub linear_impulse: Vec3,
    pub spring_impulse: Vec3,
    pub motor_impulse: Vec3,
    pub lower_twist_impulse: f32,
    pub upper_twist_impulse: f32,
    pub swing_impulse: f32,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub max_motor_torque: f32,
    pub motor_velocity: Vec3,
    pub lower_twist_angle: f32,
    pub upper_twist_angle: f32,
    pub cone_angle: f32,
    pub target_rotation: Quat,

    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub delta_center: Vec3,
    pub swing_axis: Vec3,
    pub twist_jacobian: Vec3,

    pub rotation_mass: Matrix3,
    pub swing_mass: f32,
    pub twist_mass: f32,
    pub spring_softness: Softness,

    pub enable_spring: bool,
    pub enable_motor: bool,
    pub enable_cone_limit: bool,
    pub enable_twist_limit: bool,
}

impl Default for SphericalJoint {
    fn default() -> Self {
        SphericalJoint {
            linear_impulse: VEC3_ZERO,
            spring_impulse: VEC3_ZERO,
            motor_impulse: VEC3_ZERO,
            lower_twist_impulse: 0.0,
            upper_twist_impulse: 0.0,
            swing_impulse: 0.0,
            hertz: 0.0,
            damping_ratio: 0.0,
            max_motor_torque: 0.0,
            motor_velocity: VEC3_ZERO,
            lower_twist_angle: 0.0,
            upper_twist_angle: 0.0,
            cone_angle: 0.0,
            target_rotation: QUAT_IDENTITY,
            index_a: NULL_INDEX,
            index_b: NULL_INDEX,
            frame_a: TRANSFORM_IDENTITY,
            frame_b: TRANSFORM_IDENTITY,
            delta_center: VEC3_ZERO,
            swing_axis: VEC3_ZERO,
            twist_jacobian: VEC3_ZERO,
            rotation_mass: MAT3_ZERO,
            swing_mass: 0.0,
            twist_mass: 0.0,
            spring_softness: Softness::default(),
            enable_spring: false,
            enable_motor: false,
            enable_cone_limit: false,
            enable_twist_limit: false,
        }
    }
}

/// (b3WeldJoint)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WeldJoint {
    pub linear_hertz: f32,
    pub linear_damping_ratio: f32,
    pub angular_hertz: f32,
    pub angular_damping_ratio: f32,

    pub linear_spring: Softness,
    pub angular_spring: Softness,
    pub linear_impulse: Vec3,
    pub angular_impulse: Vec3,

    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub delta_center: Vec3,

    pub angular_mass: Matrix3,
}

impl Default for WeldJoint {
    fn default() -> Self {
        WeldJoint {
            linear_hertz: 0.0,
            linear_damping_ratio: 0.0,
            angular_hertz: 0.0,
            angular_damping_ratio: 0.0,
            linear_spring: Softness::default(),
            angular_spring: Softness::default(),
            linear_impulse: VEC3_ZERO,
            angular_impulse: VEC3_ZERO,
            index_a: NULL_INDEX,
            index_b: NULL_INDEX,
            frame_a: TRANSFORM_IDENTITY,
            frame_b: TRANSFORM_IDENTITY,
            delta_center: VEC3_ZERO,
            angular_mass: MAT3_ZERO,
        }
    }
}

/// (b3WheelJoint)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelJoint {
    pub linear_impulse: Vec2,
    pub angular_impulse: Vec2,
    pub spin_impulse: f32,
    pub max_spin_torque: f32,
    pub spin_speed: f32,
    pub suspension_spring_impulse: f32,
    pub lower_suspension_impulse: f32,
    pub upper_suspension_impulse: f32,
    pub lower_suspension_limit: f32,
    pub upper_suspension_limit: f32,
    pub suspension_hertz: f32,
    pub suspension_damping_ratio: f32,
    pub steering_spring_impulse: f32,
    pub lower_steering_impulse: f32,
    pub upper_steering_impulse: f32,
    pub lower_steering_limit: f32,
    pub upper_steering_limit: f32,
    pub target_steering_angle: f32,
    pub max_steering_torque: f32,
    pub steering_hertz: f32,
    pub steering_damping_ratio: f32,

    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub delta_center: Vec3,
    pub spin_mass: f32,
    pub suspension_mass: f32,
    pub steering_mass: f32,
    pub suspension_softness: Softness,
    pub steering_softness: Softness,

    pub enable_spin_motor: bool,
    pub enable_suspension_spring: bool,
    pub enable_suspension_limit: bool,
    pub enable_steering: bool,
    pub enable_steering_limit: bool,
    pub enable_steering_motor: bool,
}

impl Default for WheelJoint {
    fn default() -> Self {
        WheelJoint {
            linear_impulse: VEC2_ZERO,
            angular_impulse: VEC2_ZERO,
            spin_impulse: 0.0,
            max_spin_torque: 0.0,
            spin_speed: 0.0,
            suspension_spring_impulse: 0.0,
            lower_suspension_impulse: 0.0,
            upper_suspension_impulse: 0.0,
            lower_suspension_limit: 0.0,
            upper_suspension_limit: 0.0,
            suspension_hertz: 0.0,
            suspension_damping_ratio: 0.0,
            steering_spring_impulse: 0.0,
            lower_steering_impulse: 0.0,
            upper_steering_impulse: 0.0,
            lower_steering_limit: 0.0,
            upper_steering_limit: 0.0,
            target_steering_angle: 0.0,
            max_steering_torque: 0.0,
            steering_hertz: 0.0,
            steering_damping_ratio: 0.0,
            index_a: NULL_INDEX,
            index_b: NULL_INDEX,
            frame_a: TRANSFORM_IDENTITY,
            frame_b: TRANSFORM_IDENTITY,
            delta_center: VEC3_ZERO,
            spin_mass: 0.0,
            suspension_mass: 0.0,
            steering_mass: 0.0,
            suspension_softness: Softness::default(),
            steering_softness: Softness::default(),
            enable_spin_motor: false,
            enable_suspension_spring: false,
            enable_suspension_limit: false,
            enable_steering: false,
            enable_steering_limit: false,
            enable_steering_motor: false,
        }
    }
}

/// Joint-specific simulation union. (C anonymous union in b3JointSim)
/// Filter joints have no simulation payload.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum JointUnion {
    Distance(DistanceJoint),
    Filter,
    Motor(MotorJoint),
    Parallel(ParallelJoint),
    Revolute(RevoluteJoint),
    Spherical(SphericalJoint),
    Prismatic(PrismaticJoint),
    Weld(WeldJoint),
    Wheel(WheelJoint),
}

impl JointUnion {
    /// Empty payload for the given type (C memset of the union).
    pub fn empty(joint_type: JointType) -> Self {
        match joint_type {
            JointType::Distance => JointUnion::Distance(DistanceJoint::default()),
            JointType::Filter => JointUnion::Filter,
            JointType::Motor => JointUnion::Motor(MotorJoint::default()),
            JointType::Parallel => JointUnion::Parallel(ParallelJoint::default()),
            JointType::Prismatic => JointUnion::Prismatic(PrismaticJoint::default()),
            JointType::Revolute => JointUnion::Revolute(RevoluteJoint::default()),
            JointType::Spherical => JointUnion::Spherical(SphericalJoint::default()),
            JointType::Weld => JointUnion::Weld(WeldJoint::default()),
            JointType::Wheel => JointUnion::Wheel(WheelJoint::default()),
        }
    }
}

impl Default for JointUnion {
    fn default() -> Self {
        JointUnion::Distance(DistanceJoint::default())
    }
}

impl JointSim {
    /// (C: &base->distanceJoint)
    pub fn distance(&self) -> &DistanceJoint {
        match &self.union_ {
            JointUnion::Distance(joint) => joint,
            _ => unreachable!("joint union is not a distance joint"),
        }
    }

    pub fn distance_mut(&mut self) -> &mut DistanceJoint {
        match &mut self.union_ {
            JointUnion::Distance(joint) => joint,
            _ => unreachable!("joint union is not a distance joint"),
        }
    }

    /// (C: &base->parallelJoint)
    pub fn parallel(&self) -> &ParallelJoint {
        match &self.union_ {
            JointUnion::Parallel(joint) => joint,
            _ => unreachable!("joint union is not a parallel joint"),
        }
    }

    pub fn parallel_mut(&mut self) -> &mut ParallelJoint {
        match &mut self.union_ {
            JointUnion::Parallel(joint) => joint,
            _ => unreachable!("joint union is not a parallel joint"),
        }
    }

    /// (C: &base->weldJoint)
    pub fn weld(&self) -> &WeldJoint {
        match &self.union_ {
            JointUnion::Weld(joint) => joint,
            _ => unreachable!("joint union is not a weld joint"),
        }
    }

    pub fn weld_mut(&mut self) -> &mut WeldJoint {
        match &mut self.union_ {
            JointUnion::Weld(joint) => joint,
            _ => unreachable!("joint union is not a weld joint"),
        }
    }

    /// (C: &base->revoluteJoint)
    pub fn revolute(&self) -> &RevoluteJoint {
        match &self.union_ {
            JointUnion::Revolute(joint) => joint,
            _ => unreachable!("joint union is not a revolute joint"),
        }
    }

    pub fn revolute_mut(&mut self) -> &mut RevoluteJoint {
        match &mut self.union_ {
            JointUnion::Revolute(joint) => joint,
            _ => unreachable!("joint union is not a revolute joint"),
        }
    }

    /// (C: &base->prismaticJoint)
    pub fn prismatic(&self) -> &PrismaticJoint {
        match &self.union_ {
            JointUnion::Prismatic(joint) => joint,
            _ => unreachable!("joint union is not a prismatic joint"),
        }
    }

    pub fn prismatic_mut(&mut self) -> &mut PrismaticJoint {
        match &mut self.union_ {
            JointUnion::Prismatic(joint) => joint,
            _ => unreachable!("joint union is not a prismatic joint"),
        }
    }

    /// (C: &base->motorJoint)
    pub fn motor(&self) -> &MotorJoint {
        match &self.union_ {
            JointUnion::Motor(joint) => joint,
            _ => unreachable!("joint union is not a motor joint"),
        }
    }

    pub fn motor_mut(&mut self) -> &mut MotorJoint {
        match &mut self.union_ {
            JointUnion::Motor(joint) => joint,
            _ => unreachable!("joint union is not a motor joint"),
        }
    }

    /// (C: &base->sphericalJoint)
    pub fn spherical(&self) -> &SphericalJoint {
        match &self.union_ {
            JointUnion::Spherical(joint) => joint,
            _ => unreachable!("joint union is not a spherical joint"),
        }
    }

    pub fn spherical_mut(&mut self) -> &mut SphericalJoint {
        match &mut self.union_ {
            JointUnion::Spherical(joint) => joint,
            _ => unreachable!("joint union is not a spherical joint"),
        }
    }

    /// (C: &base->wheelJoint)
    pub fn wheel(&self) -> &WheelJoint {
        match &self.union_ {
            JointUnion::Wheel(joint) => joint,
            _ => unreachable!("joint union is not a wheel joint"),
        }
    }

    pub fn wheel_mut(&mut self) -> &mut WheelJoint {
        match &mut self.union_ {
            JointUnion::Wheel(joint) => joint,
            _ => unreachable!("joint union is not a wheel joint"),
        }
    }
}

/// The base joint simulation class. (b3JointSim)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointSim {
    pub joint_id: i32,

    pub body_id_a: i32,
    pub body_id_b: i32,

    pub type_: JointType,

    /// Joint frames local to body origin
    pub local_frame_a: Transform,
    pub local_frame_b: Transform,

    pub inv_mass_a: f32,
    pub inv_mass_b: f32,
    pub inv_i_a: Matrix3,
    pub inv_i_b: Matrix3,

    pub constraint_hertz: f32,
    pub constraint_damping_ratio: f32,

    pub constraint_softness: Softness,

    pub force_threshold: f32,
    pub torque_threshold: f32,

    pub fixed_rotation: bool,

    pub union_: JointUnion,
}

impl Default for JointSim {
    fn default() -> Self {
        JointSim {
            joint_id: NULL_INDEX,
            body_id_a: NULL_INDEX,
            body_id_b: NULL_INDEX,
            type_: JointType::Distance,
            local_frame_a: TRANSFORM_IDENTITY,
            local_frame_b: TRANSFORM_IDENTITY,
            inv_mass_a: 0.0,
            inv_mass_b: 0.0,
            inv_i_a: MAT3_ZERO,
            inv_i_b: MAT3_ZERO,
            constraint_hertz: 0.0,
            constraint_damping_ratio: 0.0,
            constraint_softness: Softness::default(),
            force_threshold: 0.0,
            torque_threshold: 0.0,
            fixed_rotation: false,
            union_: JointUnion::default(),
        }
    }
}
