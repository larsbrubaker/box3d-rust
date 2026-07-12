//! joint ops handlers for the replay dispatcher, split from
//! dispatch.rs to satisfy the file-length limit. One function per op family;
//! returns false when the op belongs to another family.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::joint::{
    create_distance_joint, create_filter_joint, create_motor_joint, create_parallel_joint,
    create_prismatic_joint, create_revolute_joint, create_spherical_joint, create_weld_joint,
    create_wheel_joint,
};
use crate::recording::dispatch::RecReader;
use crate::recording::ops::RecOp;

#[allow(unused_imports)]
use crate::recording::query_replay;

/// Handle one joint op. Returns false if `op` is not in this family.
pub(super) fn dispatch(
    op: RecOp,
    rdr: &mut RecReader<'_>,
    payload_start: i32,
    payload_size: u32,
) -> bool {
    let world = unsafe { &mut *rdr.world };
    let _ = world;
    match op {
        RecOp::CreateParallelJoint => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let def = s.parallel_joint_def();
            rdr.sync_from(&s);
            if rdr.ok {
                let mut s2 = rdr.snap();
                let rec_id = s2.joint_id();
                rdr.sync_from(&s2);
                let mut def = def;
                def.base.body_id_a = rdr.make_body_id(def.base.body_id_a);
                def.base.body_id_b = rdr.make_body_id(def.base.body_id_b);
                let got = create_parallel_joint(world, &def);
                RecReader::check_id(
                    &mut rdr.ok,
                    "joint",
                    got.index1,
                    got.generation,
                    rec_id.index1,
                    rec_id.generation,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::CreateDistanceJoint => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let def = s.distance_joint_def();
            rdr.sync_from(&s);
            if rdr.ok {
                let mut s2 = rdr.snap();
                let rec_id = s2.joint_id();
                rdr.sync_from(&s2);
                let mut def = def;
                def.base.body_id_a = rdr.make_body_id(def.base.body_id_a);
                def.base.body_id_b = rdr.make_body_id(def.base.body_id_b);
                let got = create_distance_joint(world, &def);
                RecReader::check_id(
                    &mut rdr.ok,
                    "joint",
                    got.index1,
                    got.generation,
                    rec_id.index1,
                    rec_id.generation,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::CreateFilterJoint => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let def = s.filter_joint_def();
            rdr.sync_from(&s);
            if rdr.ok {
                let mut s2 = rdr.snap();
                let rec_id = s2.joint_id();
                rdr.sync_from(&s2);
                let mut def = def;
                def.base.body_id_a = rdr.make_body_id(def.base.body_id_a);
                def.base.body_id_b = rdr.make_body_id(def.base.body_id_b);
                let got = create_filter_joint(world, &def);
                RecReader::check_id(
                    &mut rdr.ok,
                    "joint",
                    got.index1,
                    got.generation,
                    rec_id.index1,
                    rec_id.generation,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::CreateMotorJoint => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let def = s.motor_joint_def();
            rdr.sync_from(&s);
            if rdr.ok {
                let mut s2 = rdr.snap();
                let rec_id = s2.joint_id();
                rdr.sync_from(&s2);
                let mut def = def;
                def.base.body_id_a = rdr.make_body_id(def.base.body_id_a);
                def.base.body_id_b = rdr.make_body_id(def.base.body_id_b);
                let got = create_motor_joint(world, &def);
                RecReader::check_id(
                    &mut rdr.ok,
                    "joint",
                    got.index1,
                    got.generation,
                    rec_id.index1,
                    rec_id.generation,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::CreatePrismaticJoint => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let def = s.prismatic_joint_def();
            rdr.sync_from(&s);
            if rdr.ok {
                let mut s2 = rdr.snap();
                let rec_id = s2.joint_id();
                rdr.sync_from(&s2);
                let mut def = def;
                def.base.body_id_a = rdr.make_body_id(def.base.body_id_a);
                def.base.body_id_b = rdr.make_body_id(def.base.body_id_b);
                let got = create_prismatic_joint(world, &def);
                RecReader::check_id(
                    &mut rdr.ok,
                    "joint",
                    got.index1,
                    got.generation,
                    rec_id.index1,
                    rec_id.generation,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::CreateRevoluteJoint => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let def = s.revolute_joint_def();
            rdr.sync_from(&s);
            if rdr.ok {
                let mut s2 = rdr.snap();
                let rec_id = s2.joint_id();
                rdr.sync_from(&s2);
                let mut def = def;
                def.base.body_id_a = rdr.make_body_id(def.base.body_id_a);
                def.base.body_id_b = rdr.make_body_id(def.base.body_id_b);
                let got = create_revolute_joint(world, &def);
                RecReader::check_id(
                    &mut rdr.ok,
                    "joint",
                    got.index1,
                    got.generation,
                    rec_id.index1,
                    rec_id.generation,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::CreateSphericalJoint => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let def = s.spherical_joint_def();
            rdr.sync_from(&s);
            if rdr.ok {
                let mut s2 = rdr.snap();
                let rec_id = s2.joint_id();
                rdr.sync_from(&s2);
                let mut def = def;
                def.base.body_id_a = rdr.make_body_id(def.base.body_id_a);
                def.base.body_id_b = rdr.make_body_id(def.base.body_id_b);
                let got = create_spherical_joint(world, &def);
                RecReader::check_id(
                    &mut rdr.ok,
                    "joint",
                    got.index1,
                    got.generation,
                    rec_id.index1,
                    rec_id.generation,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::CreateWeldJoint => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let def = s.weld_joint_def();
            rdr.sync_from(&s);
            if rdr.ok {
                let mut s2 = rdr.snap();
                let rec_id = s2.joint_id();
                rdr.sync_from(&s2);
                let mut def = def;
                def.base.body_id_a = rdr.make_body_id(def.base.body_id_a);
                def.base.body_id_b = rdr.make_body_id(def.base.body_id_b);
                let got = create_weld_joint(world, &def);
                RecReader::check_id(
                    &mut rdr.ok,
                    "joint",
                    got.index1,
                    got.generation,
                    rec_id.index1,
                    rec_id.generation,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::CreateWheelJoint => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let def = s.wheel_joint_def();
            rdr.sync_from(&s);
            if rdr.ok {
                let mut s2 = rdr.snap();
                let rec_id = s2.joint_id();
                rdr.sync_from(&s2);
                let mut def = def;
                def.base.body_id_a = rdr.make_body_id(def.base.body_id_a);
                def.base.body_id_b = rdr.make_body_id(def.base.body_id_b);
                let got = create_wheel_joint(world, &def);
                RecReader::check_id(
                    &mut rdr.ok,
                    "joint",
                    got.index1,
                    got.generation,
                    rec_id.index1,
                    rec_id.generation,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::DestroyJoint => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let wake_attached = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::destroy_joint(world, rdr.make_joint_id(joint), wake_attached);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::JointSetLocalFrameA => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let local_frame = s.transform();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::joint_set_local_frame_a(world, rdr.make_joint_id(joint), local_frame);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::JointSetLocalFrameB => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let local_frame = s.transform();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::joint_set_local_frame_b(world, rdr.make_joint_id(joint), local_frame);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::JointSetCollideConnected => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let should_collide = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::joint_set_collide_connected(
                    world,
                    rdr.make_joint_id(joint),
                    should_collide,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::JointWakeBodies => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::joint_wake_bodies(world, rdr.make_joint_id(joint));
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::JointSetConstraintTuning => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::joint_set_constraint_tuning(
                    world,
                    rdr.make_joint_id(joint),
                    hertz,
                    damping_ratio,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::JointSetForceThreshold => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let threshold = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::joint_set_force_threshold(world, rdr.make_joint_id(joint), threshold);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::JointSetTorqueThreshold => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let threshold = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::joint_set_torque_threshold(
                    world,
                    rdr.make_joint_id(joint),
                    threshold,
                );
            }
            let _ = (payload_start, payload_size);
        }
        _ => return false,
    }
    true
}
