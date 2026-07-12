//! JointSim field-by-field snapshot ser/de.
use crate::joint::{
    DistanceJoint, Joint, JointEdge, JointSim, JointType, JointUnion, MotorJoint,
    ParallelJoint, PrismaticJoint, RevoluteJoint, SphericalJoint, WeldJoint, WheelJoint,
};
use crate::math_functions::Vec2;
use crate::recording::buffer::{RecBuffer, SnapReader};
use crate::solver::Softness;

impl RecBuffer {
    pub fn append_vec2(&mut self, v: Vec2) { self.append_f32(v.x); self.append_f32(v.y); }
    pub fn append_softness(&mut self, v: Softness) { self.append_f32(v.bias_rate); self.append_f32(v.mass_scale); self.append_f32(v.impulse_scale); }
}
impl SnapReader<'_> {
    pub fn vec2(&mut self) -> Vec2 { Vec2 { x: self.f32(), y: self.f32() } }
    pub fn softness(&mut self) -> Softness { Softness { bias_rate: self.f32(), mass_scale: self.f32(), impulse_scale: self.f32() } }
}

pub fn ser_joint(buf: &mut RecBuffer, j: &Joint) {
    buf.append_u64(0); // user_data scrubbed
    buf.append_i32(j.set_index);
    buf.append_i32(j.color_index);
    buf.append_i32(j.local_index);
    for e in &j.edges { buf.append_i32(e.body_id); buf.append_i32(e.prev_key); buf.append_i32(e.next_key); }
    buf.append_i32(j.joint_id);
    buf.append_i32(j.island_id);
    buf.append_i32(j.island_index);
    buf.append_f32(j.draw_scale);
    buf.append_i32(j.type_ as i32);
    buf.append_u16(j.generation);
    buf.append_bool(j.collide_connected);
}

pub fn des_joint(r: &mut SnapReader<'_>) -> Joint {
    let _user = r.u64();
    Joint {
        user_data: 0,
        set_index: r.i32(),
        color_index: r.i32(),
        local_index: r.i32(),
        edges: [
            JointEdge { body_id: r.i32(), prev_key: r.i32(), next_key: r.i32() },
            JointEdge { body_id: r.i32(), prev_key: r.i32(), next_key: r.i32() },
        ],
        joint_id: r.i32(),
        island_id: r.i32(),
        island_index: r.i32(),
        draw_scale: r.f32(),
        type_: match r.i32() {
            0 => JointType::Parallel,
            1 => JointType::Distance,
            2 => JointType::Filter,
            3 => JointType::Motor,
            4 => JointType::Prismatic,
            5 => JointType::Revolute,
            6 => JointType::Spherical,
            7 => JointType::Weld,
            _ => JointType::Wheel,
        },
        generation: r.u16(),
        collide_connected: r.bool(),
    }
}

pub fn ser_joint_sim(buf: &mut RecBuffer, s: &JointSim) {
    buf.append_i32(s.joint_id);
    buf.append_i32(s.body_id_a);
    buf.append_i32(s.body_id_b);
    buf.append_i32(s.type_ as i32);
    buf.append_transform(s.local_frame_a);
    buf.append_transform(s.local_frame_b);
    buf.append_f32(s.inv_mass_a);
    buf.append_f32(s.inv_mass_b);
    buf.append_matrix3(s.inv_i_a);
    buf.append_matrix3(s.inv_i_b);
    buf.append_f32(s.constraint_hertz);
    buf.append_f32(s.constraint_damping_ratio);
    buf.append_softness(s.constraint_softness);
    buf.append_f32(s.force_threshold);
    buf.append_f32(s.torque_threshold);
    buf.append_bool(s.fixed_rotation);
    match &s.union_ {
        JointUnion::Distance(j) => {
            buf.append_i32(s.type_ as i32);
            buf.append_f32(j.length);
            buf.append_f32(j.hertz);
            buf.append_f32(j.damping_ratio);
            buf.append_f32(j.lower_spring_force);
            buf.append_f32(j.upper_spring_force);
            buf.append_f32(j.min_length);
            buf.append_f32(j.max_length);
            buf.append_f32(j.max_motor_force);
            buf.append_f32(j.motor_speed);
            buf.append_f32(j.impulse);
            buf.append_f32(j.lower_impulse);
            buf.append_f32(j.upper_impulse);
            buf.append_f32(j.motor_impulse);
            buf.append_i32(j.index_a);
            buf.append_i32(j.index_b);
            buf.append_vec3(j.anchor_a);
            buf.append_vec3(j.anchor_b);
            buf.append_vec3(j.delta_center);
            buf.append_softness(j.distance_softness);
            buf.append_f32(j.axial_mass);
            buf.append_bool(j.enable_spring);
            buf.append_bool(j.enable_limit);
            buf.append_bool(j.enable_motor);
        }
        JointUnion::Motor(j) => {
            buf.append_i32(s.type_ as i32);
            buf.append_vec3(j.linear_velocity);
            buf.append_vec3(j.angular_velocity);
            buf.append_f32(j.max_velocity_force);
            buf.append_f32(j.max_velocity_torque);
            buf.append_f32(j.linear_hertz);
            buf.append_f32(j.linear_damping_ratio);
            buf.append_f32(j.max_spring_force);
            buf.append_f32(j.angular_hertz);
            buf.append_f32(j.angular_damping_ratio);
            buf.append_f32(j.max_spring_torque);
            buf.append_vec3(j.linear_velocity_impulse);
            buf.append_vec3(j.angular_velocity_impulse);
            buf.append_vec3(j.linear_spring_impulse);
            buf.append_vec3(j.angular_spring_impulse);
            buf.append_softness(j.linear_spring);
            buf.append_softness(j.angular_spring);
            buf.append_i32(j.index_a);
            buf.append_i32(j.index_b);
            buf.append_transform(j.frame_a);
            buf.append_transform(j.frame_b);
            buf.append_vec3(j.delta_center);
            buf.append_matrix3(j.angular_mass);
        }
        JointUnion::Parallel(j) => {
            buf.append_i32(s.type_ as i32);
            buf.append_f32(j.hertz);
            buf.append_f32(j.damping_ratio);
            buf.append_f32(j.max_torque);
            buf.append_vec2(j.perp_impulse);
            buf.append_vec3(j.perp_axis_x);
            buf.append_vec3(j.perp_axis_y);
            buf.append_quat(j.quat_a);
            buf.append_quat(j.quat_b);
            buf.append_i32(j.index_a);
            buf.append_i32(j.index_b);
            buf.append_softness(j.softness);
        }
        JointUnion::Prismatic(j) => {
            buf.append_i32(s.type_ as i32);
            buf.append_vec2(j.perp_impulse);
            buf.append_vec3(j.angular_impulse);
            buf.append_f32(j.spring_impulse);
            buf.append_f32(j.motor_impulse);
            buf.append_f32(j.lower_impulse);
            buf.append_f32(j.upper_impulse);
            buf.append_f32(j.hertz);
            buf.append_f32(j.damping_ratio);
            buf.append_f32(j.max_motor_force);
            buf.append_f32(j.motor_speed);
            buf.append_f32(j.target_translation);
            buf.append_f32(j.lower_translation);
            buf.append_f32(j.upper_translation);
            buf.append_i32(j.index_a);
            buf.append_i32(j.index_b);
            buf.append_transform(j.frame_a);
            buf.append_transform(j.frame_b);
            buf.append_vec3(j.joint_axis);
            buf.append_vec3(j.perp_axis_y);
            buf.append_vec3(j.perp_axis_z);
            buf.append_vec3(j.delta_center);
            buf.append_f32(j.delta_angle);
            buf.append_matrix3(j.rotation_mass);
            buf.append_softness(j.spring_softness);
            buf.append_bool(j.enable_spring);
            buf.append_bool(j.enable_limit);
            buf.append_bool(j.enable_motor);
        }
        JointUnion::Revolute(j) => {
            buf.append_i32(s.type_ as i32);
            buf.append_vec3(j.linear_impulse);
            buf.append_vec2(j.perp_impulse);
            buf.append_f32(j.spring_impulse);
            buf.append_f32(j.motor_impulse);
            buf.append_f32(j.lower_impulse);
            buf.append_f32(j.upper_impulse);
            buf.append_f32(j.hertz);
            buf.append_f32(j.damping_ratio);
            buf.append_f32(j.max_motor_torque);
            buf.append_f32(j.motor_speed);
            buf.append_f32(j.target_angle);
            buf.append_f32(j.lower_angle);
            buf.append_f32(j.upper_angle);
            buf.append_i32(j.index_a);
            buf.append_i32(j.index_b);
            buf.append_transform(j.frame_a);
            buf.append_transform(j.frame_b);
            buf.append_vec3(j.rotation_axis_z);
            buf.append_vec3(j.perp_axis_x);
            buf.append_vec3(j.perp_axis_y);
            buf.append_vec3(j.delta_center);
            buf.append_f32(j.delta_angle);
            buf.append_f32(j.axial_mass);
            buf.append_softness(j.spring_softness);
            buf.append_bool(j.enable_spring);
            buf.append_bool(j.enable_motor);
            buf.append_bool(j.enable_limit);
        }
        JointUnion::Spherical(j) => {
            buf.append_i32(s.type_ as i32);
            buf.append_vec3(j.linear_impulse);
            buf.append_vec3(j.spring_impulse);
            buf.append_vec3(j.motor_impulse);
            buf.append_f32(j.lower_twist_impulse);
            buf.append_f32(j.upper_twist_impulse);
            buf.append_f32(j.swing_impulse);
            buf.append_f32(j.hertz);
            buf.append_f32(j.damping_ratio);
            buf.append_f32(j.max_motor_torque);
            buf.append_vec3(j.motor_velocity);
            buf.append_f32(j.lower_twist_angle);
            buf.append_f32(j.upper_twist_angle);
            buf.append_f32(j.cone_angle);
            buf.append_quat(j.target_rotation);
            buf.append_i32(j.index_a);
            buf.append_i32(j.index_b);
            buf.append_transform(j.frame_a);
            buf.append_transform(j.frame_b);
            buf.append_vec3(j.delta_center);
            buf.append_vec3(j.swing_axis);
            buf.append_vec3(j.twist_jacobian);
            buf.append_matrix3(j.rotation_mass);
            buf.append_f32(j.swing_mass);
            buf.append_f32(j.twist_mass);
            buf.append_softness(j.spring_softness);
            buf.append_bool(j.enable_spring);
            buf.append_bool(j.enable_motor);
            buf.append_bool(j.enable_cone_limit);
            buf.append_bool(j.enable_twist_limit);
        }
        JointUnion::Weld(j) => {
            buf.append_i32(s.type_ as i32);
            buf.append_f32(j.linear_hertz);
            buf.append_f32(j.linear_damping_ratio);
            buf.append_f32(j.angular_hertz);
            buf.append_f32(j.angular_damping_ratio);
            buf.append_softness(j.linear_spring);
            buf.append_softness(j.angular_spring);
            buf.append_vec3(j.linear_impulse);
            buf.append_vec3(j.angular_impulse);
            buf.append_i32(j.index_a);
            buf.append_i32(j.index_b);
            buf.append_transform(j.frame_a);
            buf.append_transform(j.frame_b);
            buf.append_vec3(j.delta_center);
            buf.append_matrix3(j.angular_mass);
        }
        JointUnion::Wheel(j) => {
            buf.append_i32(s.type_ as i32);
            buf.append_vec2(j.linear_impulse);
            buf.append_vec2(j.angular_impulse);
            buf.append_f32(j.spin_impulse);
            buf.append_f32(j.max_spin_torque);
            buf.append_f32(j.spin_speed);
            buf.append_f32(j.suspension_spring_impulse);
            buf.append_f32(j.lower_suspension_impulse);
            buf.append_f32(j.upper_suspension_impulse);
            buf.append_f32(j.lower_suspension_limit);
            buf.append_f32(j.upper_suspension_limit);
            buf.append_f32(j.suspension_hertz);
            buf.append_f32(j.suspension_damping_ratio);
            buf.append_f32(j.steering_spring_impulse);
            buf.append_f32(j.lower_steering_impulse);
            buf.append_f32(j.upper_steering_impulse);
            buf.append_f32(j.lower_steering_limit);
            buf.append_f32(j.upper_steering_limit);
            buf.append_f32(j.target_steering_angle);
            buf.append_f32(j.max_steering_torque);
            buf.append_f32(j.steering_hertz);
            buf.append_f32(j.steering_damping_ratio);
            buf.append_i32(j.index_a);
            buf.append_i32(j.index_b);
            buf.append_transform(j.frame_a);
            buf.append_transform(j.frame_b);
            buf.append_vec3(j.delta_center);
            buf.append_f32(j.spin_mass);
            buf.append_f32(j.suspension_mass);
            buf.append_f32(j.steering_mass);
            buf.append_softness(j.suspension_softness);
            buf.append_softness(j.steering_softness);
            buf.append_bool(j.enable_spin_motor);
            buf.append_bool(j.enable_suspension_spring);
            buf.append_bool(j.enable_suspension_limit);
            buf.append_bool(j.enable_steering);
            buf.append_bool(j.enable_steering_limit);
            buf.append_bool(j.enable_steering_motor);
        }
        JointUnion::Filter => {
            buf.append_i32(JointType::Filter as i32);
        }
    }
}

pub fn des_joint_sim(r: &mut SnapReader<'_>) -> JointSim {
    let joint_id = r.i32();
    let body_id_a = r.i32();
    let body_id_b = r.i32();
    let type_i = r.i32();
    let type_ = match type_i {
        0 => JointType::Parallel, 1 => JointType::Distance, 2 => JointType::Filter,
        3 => JointType::Motor, 4 => JointType::Prismatic, 5 => JointType::Revolute,
        6 => JointType::Spherical, 7 => JointType::Weld, _ => JointType::Wheel,
    };
    let local_frame_a = r.transform();
    let local_frame_b = r.transform();
    let inv_mass_a = r.f32();
    let inv_mass_b = r.f32();
    let inv_i_a = r.matrix3();
    let inv_i_b = r.matrix3();
    let constraint_hertz = r.f32();
    let constraint_damping_ratio = r.f32();
    let constraint_softness = r.softness();
    let force_threshold = r.f32();
    let torque_threshold = r.f32();
    let fixed_rotation = r.bool();
    let union_tag = r.i32();
    let union_ = match union_tag {
        0 => JointUnion::Parallel(ParallelJoint {
            hertz: r.f32(),
            damping_ratio: r.f32(),
            max_torque: r.f32(),
            perp_impulse: r.vec2(),
            perp_axis_x: r.vec3(),
            perp_axis_y: r.vec3(),
            quat_a: r.quat(),
            quat_b: r.quat(),
            index_a: r.i32(),
            index_b: r.i32(),
            softness: r.softness(),
        }),
        1 => JointUnion::Distance(DistanceJoint {
            length: r.f32(),
            hertz: r.f32(),
            damping_ratio: r.f32(),
            lower_spring_force: r.f32(),
            upper_spring_force: r.f32(),
            min_length: r.f32(),
            max_length: r.f32(),
            max_motor_force: r.f32(),
            motor_speed: r.f32(),
            impulse: r.f32(),
            lower_impulse: r.f32(),
            upper_impulse: r.f32(),
            motor_impulse: r.f32(),
            index_a: r.i32(),
            index_b: r.i32(),
            anchor_a: r.vec3(),
            anchor_b: r.vec3(),
            delta_center: r.vec3(),
            distance_softness: r.softness(),
            axial_mass: r.f32(),
            enable_spring: r.bool(),
            enable_limit: r.bool(),
            enable_motor: r.bool(),
        }),
        2 => JointUnion::Filter,
        3 => JointUnion::Motor(MotorJoint {
            linear_velocity: r.vec3(),
            angular_velocity: r.vec3(),
            max_velocity_force: r.f32(),
            max_velocity_torque: r.f32(),
            linear_hertz: r.f32(),
            linear_damping_ratio: r.f32(),
            max_spring_force: r.f32(),
            angular_hertz: r.f32(),
            angular_damping_ratio: r.f32(),
            max_spring_torque: r.f32(),
            linear_velocity_impulse: r.vec3(),
            angular_velocity_impulse: r.vec3(),
            linear_spring_impulse: r.vec3(),
            angular_spring_impulse: r.vec3(),
            linear_spring: r.softness(),
            angular_spring: r.softness(),
            index_a: r.i32(),
            index_b: r.i32(),
            frame_a: r.transform(),
            frame_b: r.transform(),
            delta_center: r.vec3(),
            angular_mass: r.matrix3(),
        }),
        4 => JointUnion::Prismatic(PrismaticJoint {
            perp_impulse: r.vec2(),
            angular_impulse: r.vec3(),
            spring_impulse: r.f32(),
            motor_impulse: r.f32(),
            lower_impulse: r.f32(),
            upper_impulse: r.f32(),
            hertz: r.f32(),
            damping_ratio: r.f32(),
            max_motor_force: r.f32(),
            motor_speed: r.f32(),
            target_translation: r.f32(),
            lower_translation: r.f32(),
            upper_translation: r.f32(),
            index_a: r.i32(),
            index_b: r.i32(),
            frame_a: r.transform(),
            frame_b: r.transform(),
            joint_axis: r.vec3(),
            perp_axis_y: r.vec3(),
            perp_axis_z: r.vec3(),
            delta_center: r.vec3(),
            delta_angle: r.f32(),
            rotation_mass: r.matrix3(),
            spring_softness: r.softness(),
            enable_spring: r.bool(),
            enable_limit: r.bool(),
            enable_motor: r.bool(),
        }),
        5 => JointUnion::Revolute(RevoluteJoint {
            linear_impulse: r.vec3(),
            perp_impulse: r.vec2(),
            spring_impulse: r.f32(),
            motor_impulse: r.f32(),
            lower_impulse: r.f32(),
            upper_impulse: r.f32(),
            hertz: r.f32(),
            damping_ratio: r.f32(),
            max_motor_torque: r.f32(),
            motor_speed: r.f32(),
            target_angle: r.f32(),
            lower_angle: r.f32(),
            upper_angle: r.f32(),
            index_a: r.i32(),
            index_b: r.i32(),
            frame_a: r.transform(),
            frame_b: r.transform(),
            rotation_axis_z: r.vec3(),
            perp_axis_x: r.vec3(),
            perp_axis_y: r.vec3(),
            delta_center: r.vec3(),
            delta_angle: r.f32(),
            axial_mass: r.f32(),
            spring_softness: r.softness(),
            enable_spring: r.bool(),
            enable_motor: r.bool(),
            enable_limit: r.bool(),
        }),
        6 => JointUnion::Spherical(SphericalJoint {
            linear_impulse: r.vec3(),
            spring_impulse: r.vec3(),
            motor_impulse: r.vec3(),
            lower_twist_impulse: r.f32(),
            upper_twist_impulse: r.f32(),
            swing_impulse: r.f32(),
            hertz: r.f32(),
            damping_ratio: r.f32(),
            max_motor_torque: r.f32(),
            motor_velocity: r.vec3(),
            lower_twist_angle: r.f32(),
            upper_twist_angle: r.f32(),
            cone_angle: r.f32(),
            target_rotation: r.quat(),
            index_a: r.i32(),
            index_b: r.i32(),
            frame_a: r.transform(),
            frame_b: r.transform(),
            delta_center: r.vec3(),
            swing_axis: r.vec3(),
            twist_jacobian: r.vec3(),
            rotation_mass: r.matrix3(),
            swing_mass: r.f32(),
            twist_mass: r.f32(),
            spring_softness: r.softness(),
            enable_spring: r.bool(),
            enable_motor: r.bool(),
            enable_cone_limit: r.bool(),
            enable_twist_limit: r.bool(),
        }),
        7 => JointUnion::Weld(WeldJoint {
            linear_hertz: r.f32(),
            linear_damping_ratio: r.f32(),
            angular_hertz: r.f32(),
            angular_damping_ratio: r.f32(),
            linear_spring: r.softness(),
            angular_spring: r.softness(),
            linear_impulse: r.vec3(),
            angular_impulse: r.vec3(),
            index_a: r.i32(),
            index_b: r.i32(),
            frame_a: r.transform(),
            frame_b: r.transform(),
            delta_center: r.vec3(),
            angular_mass: r.matrix3(),
        }),
        8 => JointUnion::Wheel(WheelJoint {
            linear_impulse: r.vec2(),
            angular_impulse: r.vec2(),
            spin_impulse: r.f32(),
            max_spin_torque: r.f32(),
            spin_speed: r.f32(),
            suspension_spring_impulse: r.f32(),
            lower_suspension_impulse: r.f32(),
            upper_suspension_impulse: r.f32(),
            lower_suspension_limit: r.f32(),
            upper_suspension_limit: r.f32(),
            suspension_hertz: r.f32(),
            suspension_damping_ratio: r.f32(),
            steering_spring_impulse: r.f32(),
            lower_steering_impulse: r.f32(),
            upper_steering_impulse: r.f32(),
            lower_steering_limit: r.f32(),
            upper_steering_limit: r.f32(),
            target_steering_angle: r.f32(),
            max_steering_torque: r.f32(),
            steering_hertz: r.f32(),
            steering_damping_ratio: r.f32(),
            index_a: r.i32(),
            index_b: r.i32(),
            frame_a: r.transform(),
            frame_b: r.transform(),
            delta_center: r.vec3(),
            spin_mass: r.f32(),
            suspension_mass: r.f32(),
            steering_mass: r.f32(),
            suspension_softness: r.softness(),
            steering_softness: r.softness(),
            enable_spin_motor: r.bool(),
            enable_suspension_spring: r.bool(),
            enable_suspension_limit: r.bool(),
            enable_steering: r.bool(),
            enable_steering_limit: r.bool(),
            enable_steering_motor: r.bool(),
        }),
        _ => JointUnion::Filter,
    };
    JointSim { joint_id, body_id_a, body_id_b, type_, local_frame_a, local_frame_b,
        inv_mass_a, inv_mass_b, inv_i_a, inv_i_b, constraint_hertz, constraint_damping_ratio,
        constraint_softness, force_threshold, torque_threshold, fixed_rotation, union_ }
}
