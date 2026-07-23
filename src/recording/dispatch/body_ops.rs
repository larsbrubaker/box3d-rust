//! body ops handlers for the replay dispatcher, split from
//! dispatch.rs to satisfy the file-length limit. One function per op family;
//! returns false when the op belongs to another family.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::body::create_body;
use crate::recording::dispatch::RecReader;
use crate::recording::ops::RecOp;
use crate::types::BodyType;

#[allow(unused_imports)]
use crate::recording::query_replay;

/// Handle one body op. Returns false if `op` is not in this family.
pub(super) fn dispatch(
    op: RecOp,
    rdr: &mut RecReader<'_>,
    payload_start: i32,
    payload_size: u32,
) -> bool {
    let world = unsafe { &mut *rdr.world };
    let _ = world;
    match op {
        RecOp::CreateBody => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let def = s.body_def();
            rdr.sync_from(&s);
            if rdr.ok {
                let mut s2 = rdr.snap();
                let rec_id = s2.body_id();
                rdr.sync_from(&s2);
                let got = create_body(world, &def);
                RecReader::check_id(
                    &mut rdr.ok,
                    "body",
                    got.index1,
                    got.generation,
                    rec_id.index1,
                    rec_id.generation,
                );
                rdr.pending_body_create = Some(got);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::DestroyBody => {
            let mut s = rdr.snap();
            let body = s.body_id();
            rdr.sync_from(&s);
            if rdr.ok {
                let id = rdr.make_body_id(body);
                rdr.pending_body_destroy = Some(id);
                crate::body::destroy_body(world, id);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodySetTransform => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let position = s.pos();
            let rotation = s.quat();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_transform(world, rdr.make_body_id(body), position, rotation);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodySetLinearVelocity => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let v = s.vec3();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_linear_velocity(world, rdr.make_body_id(body), v);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodySetType => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let type_ = s.i32();
            rdr.sync_from(&s);
            if rdr.ok {
                let id = rdr.make_body_id(body);
                let ty = match type_ {
                    1 => BodyType::Kinematic,
                    2 => BodyType::Dynamic,
                    _ => BodyType::Static,
                };
                crate::body::body_set_type(world, id, ty);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodySetName => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let name = s.str_owned();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_name(world, rdr.make_body_id(body), &name);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodySetAngularVelocity => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let w = s.vec3();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_angular_velocity(world, rdr.make_body_id(body), w);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodySetTargetTransform => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let target = s.world_xf();
            let time_step = s.f32();
            let wake = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_target_transform(
                    world,
                    rdr.make_body_id(body),
                    target,
                    time_step,
                    wake,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodyApplyForce => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let force = s.vec3();
            let point = s.pos();
            let wake = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_apply_force(world, rdr.make_body_id(body), force, point, wake);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodyApplyForceToCenter => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let force = s.vec3();
            let wake = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_apply_force_to_center(world, rdr.make_body_id(body), force, wake);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodyApplyTorque => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let torque = s.vec3();
            let wake = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_apply_torque(world, rdr.make_body_id(body), torque, wake);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodyApplyLinearImpulse => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let impulse = s.vec3();
            let point = s.pos();
            let wake = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_apply_linear_impulse(
                    world,
                    rdr.make_body_id(body),
                    impulse,
                    point,
                    wake,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodyApplyLinearImpulseToCenter => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let impulse = s.vec3();
            let wake = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_apply_linear_impulse_to_center(
                    world,
                    rdr.make_body_id(body),
                    impulse,
                    wake,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodyApplyAngularImpulse => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let impulse = s.vec3();
            let wake = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_apply_angular_impulse(
                    world,
                    rdr.make_body_id(body),
                    impulse,
                    wake,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodySetMassData => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let mass_data = s.mass_data();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_mass_data(world, rdr.make_body_id(body), mass_data);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodyApplyMassFromShapes => {
            let mut s = rdr.snap();
            let body = s.body_id();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_apply_mass_from_shapes(world, rdr.make_body_id(body));
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodySetLinearDamping => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let damping = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_linear_damping(world, rdr.make_body_id(body), damping);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodySetAngularDamping => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let damping = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_angular_damping(world, rdr.make_body_id(body), damping);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodySetGravityScale => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let scale = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_gravity_scale(world, rdr.make_body_id(body), scale);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodySetAwake => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let awake = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_awake(world, rdr.make_body_id(body), awake);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodyEnableSleep => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_enable_sleep(world, rdr.make_body_id(body), flag);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodySetSleepThreshold => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let threshold = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_sleep_threshold(world, rdr.make_body_id(body), threshold);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodyDisable => {
            let mut s = rdr.snap();
            let body = s.body_id();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_disable(world, rdr.make_body_id(body));
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodyEnable => {
            let mut s = rdr.snap();
            let body = s.body_id();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_enable(world, rdr.make_body_id(body));
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodySetMotionLocks => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let locks = s.locks();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_motion_locks(world, rdr.make_body_id(body), locks);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodySetBullet => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_bullet(world, rdr.make_body_id(body), flag);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodyAllowFastRotation => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_allow_fast_rotation(world, rdr.make_body_id(body), flag);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodyEnableContactRecycling => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_enable_contact_recycling(world, rdr.make_body_id(body), flag);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::BodyEnableHitEvents => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_enable_hit_events(world, rdr.make_body_id(body), flag);
            }
            let _ = (payload_start, payload_size);
        }
        _ => return false,
    }
    true
}
