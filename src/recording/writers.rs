//! Additional RecBuffer / SnapReader writers and readers for recording wire types.
//! Port of b3RecW_* / b3RecR_* def and id helpers from recording.c / recording_replay.c.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::constants::MAX_SHAPE_CAST_POINTS;
use crate::distance::ShapeProxy;
use crate::geometry::MassData;
use crate::id::{BodyId, JointId, ShapeId, WorldId};
use crate::math_functions::min_int;
use crate::recording::buffer::{RecBuffer, SnapReader};
use crate::types::{
    default_body_def, default_distance_joint_def, default_explosion_def, default_filter_joint_def,
    default_joint_def, default_motor_joint_def, default_parallel_joint_def,
    default_prismatic_joint_def, default_query_filter, default_revolute_joint_def,
    default_shape_def, default_spherical_joint_def, default_weld_joint_def,
    default_wheel_joint_def, BodyDef, BodyType, DistanceJointDef, ExplosionDef, FilterJointDef,
    JointDef, MotionLocks, MotorJointDef, ParallelJointDef, PrismaticJointDef, QueryFilter,
    RevoluteJointDef, ShapeDef, SphericalJointDef, WeldJointDef, WheelJointDef,
};

/// Maximum name length restored by the STR reader. Simplifies the scratch buffer
/// lifetime; names longer than this probably indicate a bug. (B3_MAX_NAME_LENGTH)
pub const MAX_NAME_LENGTH: usize = 256;

impl RecBuffer {
    /// (b3RecW_STR)
    pub fn append_str(&mut self, s: &str) {
        let bytes = s.as_bytes();
        let len = bytes.len().min(65534);
        self.append_u16(len as u16);
        if len > 0 {
            self.append(&bytes[..len]);
        }
    }

    /// Null string sentinel used when C passes NULL. (b3RecW_STR NULL path)
    pub fn append_str_null(&mut self) {
        self.append_u16(0xFFFF);
    }

    pub fn append_world_id(&mut self, v: WorldId) {
        self.append_u32(v.store());
    }

    pub fn append_body_id(&mut self, v: BodyId) {
        self.append_u64(v.store());
    }

    pub fn append_shape_id(&mut self, v: ShapeId) {
        self.append_u64(v.store());
    }

    pub fn append_joint_id(&mut self, v: JointId) {
        self.append_u64(v.store());
    }

    pub fn append_mass_data(&mut self, v: MassData) {
        self.append_f32(v.mass);
        self.append_vec3(v.center);
        self.append_matrix3(v.inertia);
    }

    pub fn append_locks(&mut self, v: MotionLocks) {
        self.append_bool(v.linear_x);
        self.append_bool(v.linear_y);
        self.append_bool(v.linear_z);
        self.append_bool(v.angular_x);
        self.append_bool(v.angular_y);
        self.append_bool(v.angular_z);
    }

    pub fn append_query_filter(&mut self, v: &QueryFilter) {
        self.append_u64(v.category_bits);
        self.append_u64(v.mask_bits);
    }

    pub fn append_shape_proxy(&mut self, v: &ShapeProxy) {
        let mut count = v.count;
        if count < 0 {
            count = 0;
        }
        if count > MAX_SHAPE_CAST_POINTS as i32 {
            count = MAX_SHAPE_CAST_POINTS as i32;
        }
        self.append_i32(count);
        for i in 0..count {
            self.append_vec3(v.points[i as usize]);
        }
        self.append_f32(v.radius);
    }

    pub fn append_explosion_def(&mut self, v: ExplosionDef) {
        self.append_u64(v.mask_bits);
        self.append_pos(v.position);
        self.append_f32(v.radius);
        self.append_f32(v.falloff);
        self.append_f32(v.impulse_per_area);
    }

    /// (b3RecW_BODYDEF)
    pub fn append_body_def(&mut self, v: &BodyDef) {
        self.append_i32(v.type_ as i32);
        self.append_pos(v.position);
        self.append_quat(v.rotation);
        self.append_vec3(v.linear_velocity);
        self.append_vec3(v.angular_velocity);
        self.append_f32(v.linear_damping);
        self.append_f32(v.angular_damping);
        self.append_f32(v.gravity_scale);
        self.append_f32(v.sleep_threshold);
        self.append_str(&v.name);
        self.append_u64(0); // userData not preserved
        self.append_locks(v.motion_locks);
        self.append_bool(v.enable_sleep);
        self.append_bool(v.is_awake);
        self.append_bool(v.is_bullet);
        self.append_bool(v.is_enabled);
        self.append_bool(v.allow_fast_rotation);
        self.append_bool(v.enable_contact_recycling);
    }

    /// (b3RecW_SHAPEDEF)
    pub fn append_shape_def(&mut self, v: &ShapeDef) {
        self.append_str(&v.name);
        self.append_u64(0); // userData not preserved
        let mat_count = v.materials.len() as i32;
        self.append_i32(mat_count);
        for m in &v.materials {
            self.append_material(*m);
        }
        self.append_material(v.base_material);
        self.append_f32(v.density);
        self.append_f32(v.explosion_scale);
        self.append_filter(v.filter);
        self.append_bool(v.enable_custom_filtering);
        self.append_bool(v.is_sensor);
        self.append_bool(v.enable_sensor_events);
        self.append_bool(v.enable_contact_events);
        self.append_bool(v.enable_hit_events);
        self.append_bool(v.enable_pre_solve_events);
        self.append_bool(v.invoke_contact_creation);
        self.append_bool(v.update_body_mass);
    }

    fn append_joint_base(&mut self, base: &JointDef) {
        self.append_u64(0); // userData
        self.append_body_id(base.body_id_a);
        self.append_body_id(base.body_id_b);
        self.append_transform(base.local_frame_a);
        self.append_transform(base.local_frame_b);
        self.append_f32(base.force_threshold);
        self.append_f32(base.torque_threshold);
        self.append_f32(base.constraint_hertz);
        self.append_f32(base.constraint_damping_ratio);
        self.append_f32(base.draw_scale);
        self.append_bool(base.collide_connected);
    }

    pub fn append_parallel_joint_def(&mut self, v: &ParallelJointDef) {
        self.append_joint_base(&v.base);
        self.append_f32(v.hertz);
        self.append_f32(v.damping_ratio);
        self.append_f32(v.max_torque);
    }

    pub fn append_distance_joint_def(&mut self, v: &DistanceJointDef) {
        self.append_joint_base(&v.base);
        self.append_f32(v.length);
        self.append_bool(v.enable_spring);
        self.append_f32(v.lower_spring_force);
        self.append_f32(v.upper_spring_force);
        self.append_f32(v.hertz);
        self.append_f32(v.damping_ratio);
        self.append_bool(v.enable_limit);
        self.append_f32(v.min_length);
        self.append_f32(v.max_length);
        self.append_bool(v.enable_motor);
        self.append_f32(v.max_motor_force);
        self.append_f32(v.motor_speed);
    }

    pub fn append_filter_joint_def(&mut self, v: &FilterJointDef) {
        self.append_joint_base(&v.base);
    }

    pub fn append_motor_joint_def(&mut self, v: &MotorJointDef) {
        self.append_joint_base(&v.base);
        self.append_vec3(v.linear_velocity);
        self.append_f32(v.max_velocity_force);
        self.append_vec3(v.angular_velocity);
        self.append_f32(v.max_velocity_torque);
        self.append_f32(v.linear_hertz);
        self.append_f32(v.linear_damping_ratio);
        self.append_f32(v.max_spring_force);
        self.append_f32(v.angular_hertz);
        self.append_f32(v.angular_damping_ratio);
        self.append_f32(v.max_spring_torque);
    }

    pub fn append_prismatic_joint_def(&mut self, v: &PrismaticJointDef) {
        self.append_joint_base(&v.base);
        self.append_bool(v.enable_spring);
        self.append_f32(v.hertz);
        self.append_f32(v.damping_ratio);
        self.append_f32(v.target_translation);
        self.append_bool(v.enable_limit);
        self.append_f32(v.lower_translation);
        self.append_f32(v.upper_translation);
        self.append_bool(v.enable_motor);
        self.append_f32(v.max_motor_force);
        self.append_f32(v.motor_speed);
    }

    pub fn append_revolute_joint_def(&mut self, v: &RevoluteJointDef) {
        self.append_joint_base(&v.base);
        self.append_f32(v.target_angle);
        self.append_bool(v.enable_spring);
        self.append_f32(v.hertz);
        self.append_f32(v.damping_ratio);
        self.append_bool(v.enable_limit);
        self.append_f32(v.lower_angle);
        self.append_f32(v.upper_angle);
        self.append_bool(v.enable_motor);
        self.append_f32(v.max_motor_torque);
        self.append_f32(v.motor_speed);
    }

    pub fn append_spherical_joint_def(&mut self, v: &SphericalJointDef) {
        self.append_joint_base(&v.base);
        self.append_bool(v.enable_spring);
        self.append_f32(v.hertz);
        self.append_f32(v.damping_ratio);
        self.append_quat(v.target_rotation);
        self.append_bool(v.enable_cone_limit);
        self.append_f32(v.cone_angle);
        self.append_bool(v.enable_twist_limit);
        self.append_f32(v.lower_twist_angle);
        self.append_f32(v.upper_twist_angle);
        self.append_bool(v.enable_motor);
        self.append_f32(v.max_motor_torque);
        self.append_vec3(v.motor_velocity);
    }

    pub fn append_weld_joint_def(&mut self, v: &WeldJointDef) {
        self.append_joint_base(&v.base);
        self.append_f32(v.linear_hertz);
        self.append_f32(v.angular_hertz);
        self.append_f32(v.linear_damping_ratio);
        self.append_f32(v.angular_damping_ratio);
    }

    pub fn append_wheel_joint_def(&mut self, v: &WheelJointDef) {
        self.append_joint_base(&v.base);
        self.append_bool(v.enable_suspension_spring);
        self.append_f32(v.suspension_hertz);
        self.append_f32(v.suspension_damping_ratio);
        self.append_bool(v.enable_suspension_limit);
        self.append_f32(v.lower_suspension_limit);
        self.append_f32(v.upper_suspension_limit);
        self.append_bool(v.enable_spin_motor);
        self.append_f32(v.max_spin_torque);
        self.append_f32(v.spin_speed);
        self.append_bool(v.enable_steering);
        self.append_f32(v.steering_hertz);
        self.append_f32(v.steering_damping_ratio);
        self.append_f32(v.target_steering_angle);
        self.append_f32(v.max_steering_torque);
        self.append_bool(v.enable_steering_limit);
        self.append_f32(v.lower_steering_limit);
        self.append_f32(v.upper_steering_limit);
    }
}

impl<'a> SnapReader<'a> {
    pub fn u24(&mut self) -> u32 {
        let b0 = self.u8() as u32;
        let b1 = self.u8() as u32;
        let b2 = self.u8() as u32;
        b0 | (b1 << 8) | (b2 << 16)
    }

    /// (b3RecR_STR) — empty string for the 0xFFFF null sentinel. Names are
    /// clamped to [`MAX_NAME_LENGTH`] like C's rotating scratch buffers, but the
    /// cursor still advances by the full recorded length.
    pub fn str_owned(&mut self) -> String {
        let len = self.u16();
        if len == 0xFFFF {
            return String::new();
        }
        let full = len as usize;
        let n = full.min(MAX_NAME_LENGTH);
        let result = if let Some(bytes) = self.bytes(n) {
            String::from_utf8_lossy(bytes).into_owned()
        } else {
            String::new()
        };
        if full > n {
            // Skip the over-length remainder so the cursor advances by len.
            let _ = self.bytes(full - n);
        }
        result
    }

    pub fn world_id(&mut self) -> WorldId {
        WorldId::load(self.u32())
    }

    pub fn body_id(&mut self) -> BodyId {
        BodyId::load(self.u64())
    }

    pub fn shape_id(&mut self) -> ShapeId {
        ShapeId::load(self.u64())
    }

    pub fn joint_id(&mut self) -> JointId {
        JointId::load(self.u64())
    }

    pub fn mass_data(&mut self) -> MassData {
        MassData {
            mass: self.f32(),
            center: self.vec3(),
            inertia: self.matrix3(),
        }
    }

    pub fn locks(&mut self) -> MotionLocks {
        MotionLocks {
            linear_x: self.bool(),
            linear_y: self.bool(),
            linear_z: self.bool(),
            angular_x: self.bool(),
            angular_y: self.bool(),
            angular_z: self.bool(),
        }
    }

    pub fn query_filter(&mut self) -> QueryFilter {
        let mut f = default_query_filter();
        f.category_bits = self.u64();
        f.mask_bits = self.u64();
        f
    }

    pub fn shape_proxy(&mut self) -> ShapeProxy {
        let mut proxy = ShapeProxy::default();
        let mut count = self.i32();
        if count < 0 {
            count = 0;
        }
        count = min_int(count, MAX_SHAPE_CAST_POINTS as i32);
        proxy.count = count;
        for i in 0..count {
            proxy.points[i as usize] = self.vec3();
        }
        proxy.radius = self.f32();
        proxy
    }

    pub fn explosion_def(&mut self) -> ExplosionDef {
        let mut d = default_explosion_def();
        d.mask_bits = self.u64();
        d.position = self.pos();
        d.radius = self.f32();
        d.falloff = self.f32();
        d.impulse_per_area = self.f32();
        d
    }

    pub fn body_def(&mut self) -> BodyDef {
        let mut d = default_body_def();
        d.type_ = match self.i32() {
            1 => BodyType::Kinematic,
            2 => BodyType::Dynamic,
            _ => BodyType::Static,
        };
        d.position = self.pos();
        d.rotation = self.quat();
        d.linear_velocity = self.vec3();
        d.angular_velocity = self.vec3();
        d.linear_damping = self.f32();
        d.angular_damping = self.f32();
        d.gravity_scale = self.f32();
        d.sleep_threshold = self.f32();
        d.name = self.str_owned();
        let _user = self.u64();
        d.motion_locks = self.locks();
        d.enable_sleep = self.bool();
        d.is_awake = self.bool();
        d.is_bullet = self.bool();
        d.is_enabled = self.bool();
        d.allow_fast_rotation = self.bool();
        d.enable_contact_recycling = self.bool();
        d
    }

    pub fn shape_def(&mut self) -> ShapeDef {
        let mut d = default_shape_def();
        d.name = self.str_owned();
        let _user = self.u64();
        let mat_count = self.i32();
        d.materials.clear();
        if mat_count > 0 && self.ok {
            d.materials.reserve(mat_count as usize);
            for _ in 0..mat_count {
                d.materials.push(self.material());
            }
        }
        d.base_material = self.material();
        d.density = self.f32();
        d.explosion_scale = self.f32();
        d.filter = self.filter();
        d.enable_custom_filtering = self.bool();
        d.is_sensor = self.bool();
        d.enable_sensor_events = self.bool();
        d.enable_contact_events = self.bool();
        d.enable_hit_events = self.bool();
        d.enable_pre_solve_events = self.bool();
        d.invoke_contact_creation = self.bool();
        d.update_body_mass = self.bool();
        d
    }

    fn joint_base(&mut self) -> JointDef {
        let mut base = default_joint_def();
        let _user = self.u64();
        base.body_id_a = self.body_id();
        base.body_id_b = self.body_id();
        base.local_frame_a = self.transform();
        base.local_frame_b = self.transform();
        base.force_threshold = self.f32();
        base.torque_threshold = self.f32();
        base.constraint_hertz = self.f32();
        base.constraint_damping_ratio = self.f32();
        base.draw_scale = self.f32();
        base.collide_connected = self.bool();
        base
    }

    pub fn parallel_joint_def(&mut self) -> ParallelJointDef {
        let mut d = default_parallel_joint_def();
        d.base = self.joint_base();
        d.hertz = self.f32();
        d.damping_ratio = self.f32();
        d.max_torque = self.f32();
        d
    }

    pub fn distance_joint_def(&mut self) -> DistanceJointDef {
        let mut d = default_distance_joint_def();
        d.base = self.joint_base();
        d.length = self.f32();
        d.enable_spring = self.bool();
        d.lower_spring_force = self.f32();
        d.upper_spring_force = self.f32();
        d.hertz = self.f32();
        d.damping_ratio = self.f32();
        d.enable_limit = self.bool();
        d.min_length = self.f32();
        d.max_length = self.f32();
        d.enable_motor = self.bool();
        d.max_motor_force = self.f32();
        d.motor_speed = self.f32();
        d
    }

    pub fn filter_joint_def(&mut self) -> FilterJointDef {
        let mut d = default_filter_joint_def();
        d.base = self.joint_base();
        d
    }

    pub fn motor_joint_def(&mut self) -> MotorJointDef {
        let mut d = default_motor_joint_def();
        d.base = self.joint_base();
        d.linear_velocity = self.vec3();
        d.max_velocity_force = self.f32();
        d.angular_velocity = self.vec3();
        d.max_velocity_torque = self.f32();
        d.linear_hertz = self.f32();
        d.linear_damping_ratio = self.f32();
        d.max_spring_force = self.f32();
        d.angular_hertz = self.f32();
        d.angular_damping_ratio = self.f32();
        d.max_spring_torque = self.f32();
        d
    }

    pub fn prismatic_joint_def(&mut self) -> PrismaticJointDef {
        let mut d = default_prismatic_joint_def();
        d.base = self.joint_base();
        d.enable_spring = self.bool();
        d.hertz = self.f32();
        d.damping_ratio = self.f32();
        d.target_translation = self.f32();
        d.enable_limit = self.bool();
        d.lower_translation = self.f32();
        d.upper_translation = self.f32();
        d.enable_motor = self.bool();
        d.max_motor_force = self.f32();
        d.motor_speed = self.f32();
        d
    }

    pub fn revolute_joint_def(&mut self) -> RevoluteJointDef {
        let mut d = default_revolute_joint_def();
        d.base = self.joint_base();
        d.target_angle = self.f32();
        d.enable_spring = self.bool();
        d.hertz = self.f32();
        d.damping_ratio = self.f32();
        d.enable_limit = self.bool();
        d.lower_angle = self.f32();
        d.upper_angle = self.f32();
        d.enable_motor = self.bool();
        d.max_motor_torque = self.f32();
        d.motor_speed = self.f32();
        d
    }

    pub fn spherical_joint_def(&mut self) -> SphericalJointDef {
        let mut d = default_spherical_joint_def();
        d.base = self.joint_base();
        d.enable_spring = self.bool();
        d.hertz = self.f32();
        d.damping_ratio = self.f32();
        d.target_rotation = self.quat();
        d.enable_cone_limit = self.bool();
        d.cone_angle = self.f32();
        d.enable_twist_limit = self.bool();
        d.lower_twist_angle = self.f32();
        d.upper_twist_angle = self.f32();
        d.enable_motor = self.bool();
        d.max_motor_torque = self.f32();
        d.motor_velocity = self.vec3();
        d
    }

    pub fn weld_joint_def(&mut self) -> WeldJointDef {
        let mut d = default_weld_joint_def();
        d.base = self.joint_base();
        d.linear_hertz = self.f32();
        d.angular_hertz = self.f32();
        d.linear_damping_ratio = self.f32();
        d.angular_damping_ratio = self.f32();
        d
    }

    pub fn wheel_joint_def(&mut self) -> WheelJointDef {
        let mut d = default_wheel_joint_def();
        d.base = self.joint_base();
        d.enable_suspension_spring = self.bool();
        d.suspension_hertz = self.f32();
        d.suspension_damping_ratio = self.f32();
        d.enable_suspension_limit = self.bool();
        d.lower_suspension_limit = self.f32();
        d.upper_suspension_limit = self.f32();
        d.enable_spin_motor = self.bool();
        d.max_spin_torque = self.f32();
        d.spin_speed = self.f32();
        d.enable_steering = self.bool();
        d.steering_hertz = self.f32();
        d.steering_damping_ratio = self.f32();
        d.target_steering_angle = self.f32();
        d.max_steering_torque = self.f32();
        d.enable_steering_limit = self.bool();
        d.lower_steering_limit = self.f32();
        d.upper_steering_limit = self.f32();
        d
    }
}
