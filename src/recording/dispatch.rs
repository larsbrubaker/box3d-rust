//! Op-stream dispatcher for recording replay.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::body::create_body;
use crate::height_field::convert_bytes_to_height_field;
use crate::hull::convert_bytes_to_hull;
use crate::id::{BodyId, JointId, ShapeId};
use crate::joint::{
    create_distance_joint, create_filter_joint, create_motor_joint, create_parallel_joint,
    create_prismatic_joint, create_revolute_joint, create_spherical_joint, create_weld_joint,
    create_wheel_joint,
};
use crate::mesh::convert_bytes_to_mesh;
use crate::recording::buffer::SnapReader;
use crate::recording::hash::hash_world_state;
use crate::recording::ops::RecOp;
use crate::recording::registry::RegistrySlot;
use crate::recording::session::RecTag;
use crate::shape::{
    create_capsule_shape, create_compound_shape, create_height_field_shape, create_hull_shape,
    create_mesh_shape, create_sphere_shape,
};
use crate::types::BodyType;
use crate::world::World;

use super::player::RecPlayer;

/// Reader state threaded through the replay loop. (b3RecReader)
pub struct RecReader<'a> {
    pub data: &'a [u8],
    pub size: i32,
    pub cursor: i32,
    pub ok: bool,
    pub diverged: bool,
    pub world: *mut World,
    pub owner: Option<*mut RecPlayer>,
    pub slots: Vec<RegistrySlot>,
    pub tags: Vec<RecTag>,
    pub pending_query_key: u64,
    pub pending_body_create: Option<BodyId>,
    pub pending_body_destroy: Option<BodyId>,
}

impl<'a> RecReader<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            size: data.len() as i32,
            cursor: 0,
            ok: true,
            diverged: false,
            world: std::ptr::null_mut(),
            owner: None,
            slots: Vec::new(),
            tags: Vec::new(),
            pending_query_key: 0,
            pending_body_create: None,
            pending_body_destroy: None,
        }
    }

    pub fn snap(&mut self) -> SnapReader<'a> {
        let mut r = SnapReader::new(self.data);
        r.set_cursor(self.cursor as usize);
        r.ok = self.ok;
        r
    }

    pub fn sync_from(&mut self, r: &SnapReader<'_>) {
        self.cursor = r.cursor() as i32;
        self.ok = r.ok;
    }

    pub fn make_body_id(&self, recorded: BodyId) -> BodyId {
        let world = unsafe { &*self.world };
        BodyId { index1: recorded.index1, world0: world.world_id, generation: recorded.generation }
    }

    pub fn make_shape_id(&self, recorded: ShapeId) -> ShapeId {
        let world = unsafe { &*self.world };
        ShapeId { index1: recorded.index1, world0: world.world_id, generation: recorded.generation }
    }

    pub fn make_joint_id(&self, recorded: JointId) -> JointId {
        let world = unsafe { &*self.world };
        JointId { index1: recorded.index1, world0: world.world_id, generation: recorded.generation }
    }

    pub fn check_id(ok: &mut bool, kind: &str, got_index: i32, got_gen: u16, rec_index: i32, rec_gen: u16) {
        if got_index != rec_index || got_gen != rec_gen {
            eprintln!(
                "b3ReplayFile: {kind} id mismatch (rec index1={rec_index} gen={rec_gen}, got index1={got_index} gen={got_gen})"
            );
            *ok = false;
        }
    }
}

fn unimplemented_op(name: &str) {
    eprintln!("recording replay: unhandled op {name}");
}

/// Dispatch one framed op. Returns opcode as i32, or -1 when exhausted/broken.
pub fn dispatch_one(rdr: &mut RecReader<'_>) -> i32 {
    if rdr.cursor >= rdr.size || !rdr.ok {
        return -1;
    }
    let mut snap = rdr.snap();
    let opcode = snap.u8();
    let payload_size = snap.u24();
    rdr.sync_from(&snap);
    if !rdr.ok {
        return -1;
    }
    let payload_start = rdr.cursor;
    let world = unsafe { &mut *rdr.world };

    match RecOp::from_u8(opcode) {
        Some(RecOp::DestroyWorld) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            rdr.sync_from(&s);
            if rdr.ok {
                // end-of-session marker
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::Step) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let dt = s.f32();
            let sub_step_count = s.i32();
            rdr.sync_from(&s);
            if rdr.ok {
                world.step(dt, sub_step_count);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WorldEnableSleeping) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_enable_sleeping(world, flag);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WorldEnableContinuous) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_enable_continuous(world, flag);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WorldSetRestitutionThreshold) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let value = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_set_restitution_threshold(world, value);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WorldSetHitEventThreshold) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let value = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_set_hit_event_threshold(world, value);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WorldSetGravity) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let gravity = s.vec3();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_set_gravity(world, gravity);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WorldExplode) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let def = s.explosion_def();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_explode(world, &def);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WorldSetContactTuning) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let hertz = s.f32();
            let damping_ratio = s.f32();
            let contact_speed = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_set_contact_tuning(world, hertz, damping_ratio, contact_speed);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WorldSetContactRecycleDistance) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let recycle_distance = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_set_contact_recycle_distance(world, recycle_distance);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WorldSetMaximumLinearSpeed) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let maximum_linear_speed = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_set_maximum_linear_speed(world, maximum_linear_speed);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WorldEnableWarmStarting) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_enable_warm_starting(world, flag);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WorldRebuildStaticTree) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_rebuild_static_tree(world, );
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WorldEnableSpeculative) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::world::world_enable_speculative(world, flag);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::CreateBody) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let def = s.body_def();
            rdr.sync_from(&s);
            if rdr.ok {
                let mut s2 = rdr.snap();
                let rec_id = s2.body_id();
                rdr.sync_from(&s2);
                let got = create_body(world, &def);
                RecReader::check_id(&mut rdr.ok, "body", got.index1, got.generation, rec_id.index1, rec_id.generation);
                rdr.pending_body_create = Some(got);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::DestroyBody) => {
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
        Some(RecOp::BodySetTransform) => {
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
        Some(RecOp::BodySetLinearVelocity) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let v = s.vec3();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_linear_velocity(world, rdr.make_body_id(body), v);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodySetType) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let type_ = s.i32();
            rdr.sync_from(&s);
            if rdr.ok {
                let id = rdr.make_body_id(body);
                let ty = match type_ { 1 => BodyType::Kinematic, 2 => BodyType::Dynamic, _ => BodyType::Static };
                crate::body::body_set_type(world, id, ty);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodySetName) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let name = s.body_str();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_name(world, rdr.make_body_id(body), &name);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodySetAngularVelocity) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let w = s.vec3();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_angular_velocity(world, rdr.make_body_id(body), w);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodySetTargetTransform) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let target = s.world_xf();
            let time_step = s.f32();
            let wake = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_target_transform(world, rdr.make_body_id(body), target, time_step, wake);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodyApplyForce) => {
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
        Some(RecOp::BodyApplyForceToCenter) => {
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
        Some(RecOp::BodyApplyTorque) => {
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
        Some(RecOp::BodyApplyLinearImpulse) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let impulse = s.vec3();
            let point = s.pos();
            let wake = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_apply_linear_impulse(world, rdr.make_body_id(body), impulse, point, wake);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodyApplyLinearImpulseToCenter) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let impulse = s.vec3();
            let wake = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_apply_linear_impulse_to_center(world, rdr.make_body_id(body), impulse, wake);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodyApplyAngularImpulse) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let impulse = s.vec3();
            let wake = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_apply_angular_impulse(world, rdr.make_body_id(body), impulse, wake);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodySetMassData) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let mass_data = s.mass_data();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_mass_data(world, rdr.make_body_id(body), mass_data);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodyApplyMassFromShapes) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_apply_mass_from_shapes(world, rdr.make_body_id(body));
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodySetLinearDamping) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let damping = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_linear_damping(world, rdr.make_body_id(body), damping);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodySetAngularDamping) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let damping = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_angular_damping(world, rdr.make_body_id(body), damping);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodySetGravityScale) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let scale = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_gravity_scale(world, rdr.make_body_id(body), scale);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodySetAwake) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let awake = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_awake(world, rdr.make_body_id(body), awake);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodyEnableSleep) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_enable_sleep(world, rdr.make_body_id(body), flag);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodySetSleepThreshold) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let threshold = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_sleep_threshold(world, rdr.make_body_id(body), threshold);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodyDisable) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_disable(world, rdr.make_body_id(body));
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodyEnable) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_enable(world, rdr.make_body_id(body));
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodySetMotionLocks) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let locks = s.locks();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_motion_locks(world, rdr.make_body_id(body), locks);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodySetBullet) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_set_bullet(world, rdr.make_body_id(body), flag);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodyEnableContactRecycling) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_enable_contact_recycling(world, rdr.make_body_id(body), flag);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::BodyEnableHitEvents) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::body::body_enable_hit_events(world, rdr.make_body_id(body), flag);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::CreateSphereShape) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let def = s.shape_def();
            let sphere = s.sphere();
            rdr.sync_from(&s);
            if rdr.ok {
                let mut s2 = rdr.snap();
                let rec_id = s2.shape_id();
                rdr.sync_from(&s2);
                let body_id = rdr.make_body_id(body);
                let got = create_sphere_shape(world, body_id, &def, &sphere);
                RecReader::check_id(&mut rdr.ok, "shape", got.index1, got.generation, rec_id.index1, rec_id.generation);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::CreateCapsuleShape) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let def = s.shape_def();
            let capsule = s.capsule();
            rdr.sync_from(&s);
            if rdr.ok {
                let mut s2 = rdr.snap();
                let rec_id = s2.shape_id();
                rdr.sync_from(&s2);
                let body_id = rdr.make_body_id(body);
                let got = create_capsule_shape(world, body_id, &def, &capsule);
                RecReader::check_id(&mut rdr.ok, "shape", got.index1, got.generation, rec_id.index1, rec_id.generation);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::CreateHullShape) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let def = s.shape_def();
            let geometry_id = s.u32();
            rdr.sync_from(&s);
            if rdr.ok {
                let mut s2 = rdr.snap();
                let rec_id = s2.shape_id();
                rdr.sync_from(&s2);
                let body_id = rdr.make_body_id(body);
                let got = {
                    let hull = convert_bytes_to_hull(&rdr.slots[geometry_id as usize].bytes).expect("hull");
                    create_hull_shape(world, body_id, &def, &hull)
                };
                RecReader::check_id(&mut rdr.ok, "shape", got.index1, got.generation, rec_id.index1, rec_id.generation);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::CreateMeshShape) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let def = s.shape_def();
            let geometry_id = s.u32();
            let scale = s.vec3();
            rdr.sync_from(&s);
            if rdr.ok {
                let mut s2 = rdr.snap();
                let rec_id = s2.shape_id();
                rdr.sync_from(&s2);
                let body_id = rdr.make_body_id(body);
                let got = {
                    let mesh = convert_bytes_to_mesh(&rdr.slots[geometry_id as usize].bytes).expect("mesh");
                    create_mesh_shape(world, body_id, &def, &mesh, scale)
                };
                RecReader::check_id(&mut rdr.ok, "shape", got.index1, got.generation, rec_id.index1, rec_id.generation);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::CreateHeightFieldShape) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let def = s.shape_def();
            let geometry_id = s.u32();
            rdr.sync_from(&s);
            if rdr.ok {
                let mut s2 = rdr.snap();
                let rec_id = s2.shape_id();
                rdr.sync_from(&s2);
                let body_id = rdr.make_body_id(body);
                let got = {
                    let hf = convert_bytes_to_height_field(&rdr.slots[geometry_id as usize].bytes).expect("hf");
                    create_height_field_shape(world, body_id, &def, &hf)
                };
                RecReader::check_id(&mut rdr.ok, "shape", got.index1, got.generation, rec_id.index1, rec_id.generation);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::CreateCompoundShape) => {
            let mut s = rdr.snap();
            let body = s.body_id();
            let def = s.shape_def();
            let geometry_id = s.u32();
            rdr.sync_from(&s);
            if rdr.ok {
                let mut s2 = rdr.snap();
                let rec_id = s2.shape_id();
                rdr.sync_from(&s2);
                let body_id = rdr.make_body_id(body);
                let got = {
                    let compound = rdr.slots[geometry_id as usize].ensure_compound().cloned().expect("compound");
                    create_compound_shape(world, body_id, &def, &compound)
                };
                RecReader::check_id(&mut rdr.ok, "shape", got.index1, got.generation, rec_id.index1, rec_id.generation);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::DestroyShape) => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let update_body_mass = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::destroy_shape(world, rdr.make_shape_id(shape), update_body_mass);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::ShapeSetDensity) => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let density = s.f32();
            let update_body_mass = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_set_density(world, rdr.make_shape_id(shape), density, update_body_mass);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::ShapeSetFriction) => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let friction = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_set_friction(world, rdr.make_shape_id(shape), friction);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::ShapeSetRestitution) => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let restitution = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_set_restitution(world, rdr.make_shape_id(shape), restitution);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::ShapeSetSurfaceMaterial) => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let material = s.material();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_set_surface_material(world, rdr.make_shape_id(shape), material);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::ShapeSetFilter) => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let filter = s.filter();
            let invoke_contacts = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_set_filter(world, rdr.make_shape_id(shape), filter, invoke_contacts);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::ShapeEnableSensorEvents) => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_enable_sensor_events(world, rdr.make_shape_id(shape), flag);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::ShapeEnableContactEvents) => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_enable_contact_events(world, rdr.make_shape_id(shape), flag);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::ShapeEnablePreSolveEvents) => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_enable_pre_solve_events(world, rdr.make_shape_id(shape), flag);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::ShapeEnableHitEvents) => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_enable_hit_events(world, rdr.make_shape_id(shape), flag);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::ShapeSetSphere) => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let sphere = s.sphere();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_set_sphere(world, rdr.make_shape_id(shape), &sphere);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::ShapeSetCapsule) => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let capsule = s.capsule();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_set_capsule(world, rdr.make_shape_id(shape), &capsule);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::ShapeApplyWind) => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let wind = s.vec3();
            let drag = s.f32();
            let lift = s.f32();
            let max_speed = s.f32();
            let wake = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_apply_wind(world, rdr.make_shape_id(shape), wind, drag, lift, max_speed, wake);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::ShapeSetName) => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let name = s.shape_str();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_set_name(world, rdr.make_shape_id(shape), &name);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::CreateParallelJoint) => {
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
                RecReader::check_id(&mut rdr.ok, "joint", got.index1, got.generation, rec_id.index1, rec_id.generation);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::CreateDistanceJoint) => {
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
                RecReader::check_id(&mut rdr.ok, "joint", got.index1, got.generation, rec_id.index1, rec_id.generation);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::CreateFilterJoint) => {
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
                RecReader::check_id(&mut rdr.ok, "joint", got.index1, got.generation, rec_id.index1, rec_id.generation);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::CreateMotorJoint) => {
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
                RecReader::check_id(&mut rdr.ok, "joint", got.index1, got.generation, rec_id.index1, rec_id.generation);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::CreatePrismaticJoint) => {
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
                RecReader::check_id(&mut rdr.ok, "joint", got.index1, got.generation, rec_id.index1, rec_id.generation);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::CreateRevoluteJoint) => {
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
                RecReader::check_id(&mut rdr.ok, "joint", got.index1, got.generation, rec_id.index1, rec_id.generation);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::CreateSphericalJoint) => {
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
                RecReader::check_id(&mut rdr.ok, "joint", got.index1, got.generation, rec_id.index1, rec_id.generation);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::CreateWeldJoint) => {
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
                RecReader::check_id(&mut rdr.ok, "joint", got.index1, got.generation, rec_id.index1, rec_id.generation);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::CreateWheelJoint) => {
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
                RecReader::check_id(&mut rdr.ok, "joint", got.index1, got.generation, rec_id.index1, rec_id.generation);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::DestroyJoint) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let wake_attached = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::destroy_joint(world, rdr.make_joint_id(joint), wake_attached);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::JointSetLocalFrameA) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let local_frame = s.transform();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::joint_set_local_frame_a(world, rdr.make_joint_id(joint), local_frame);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::JointSetLocalFrameB) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let local_frame = s.transform();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::joint_set_local_frame_b(world, rdr.make_joint_id(joint), local_frame);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::JointSetCollideConnected) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let should_collide = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::joint_set_collide_connected(world, rdr.make_joint_id(joint), should_collide);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::JointWakeBodies) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::joint_wake_bodies(world, rdr.make_joint_id(joint));
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::JointSetConstraintTuning) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::joint_set_constraint_tuning(world, rdr.make_joint_id(joint), hertz, damping_ratio);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::JointSetForceThreshold) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let threshold = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::joint_set_force_threshold(world, rdr.make_joint_id(joint), threshold);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::JointSetTorqueThreshold) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let threshold = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::joint_set_torque_threshold(world, rdr.make_joint_id(joint), threshold);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::ParallelJointSetSpringHertz) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::parallel_joint_set_spring_hertz(world, rdr.make_joint_id(joint), hertz);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::ParallelJointSetSpringDampingRatio) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::parallel_joint_set_spring_damping_ratio(world, rdr.make_joint_id(joint), damping_ratio);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::ParallelJointSetMaxTorque) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let max_torque = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::parallel_joint_set_max_torque(world, rdr.make_joint_id(joint), max_torque);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::DistanceJointSetLength) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let length = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_set_length(world, rdr.make_joint_id(joint), length);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::DistanceJointEnableSpring) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_spring = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_enable_spring(world, rdr.make_joint_id(joint), enable_spring);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::DistanceJointSetSpringForceRange) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let lower_force = s.f32();
            let upper_force = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_set_spring_force_range(world, rdr.make_joint_id(joint), lower_force, upper_force);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::DistanceJointSetSpringHertz) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_set_spring_hertz(world, rdr.make_joint_id(joint), hertz);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::DistanceJointSetSpringDampingRatio) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_set_spring_damping_ratio(world, rdr.make_joint_id(joint), damping_ratio);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::DistanceJointEnableLimit) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_limit = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_enable_limit(world, rdr.make_joint_id(joint), enable_limit);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::DistanceJointSetLengthRange) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let min_length = s.f32();
            let max_length = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_set_length_range(world, rdr.make_joint_id(joint), min_length, max_length);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::DistanceJointEnableMotor) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_motor = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_enable_motor(world, rdr.make_joint_id(joint), enable_motor);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::DistanceJointSetMotorSpeed) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let motor_speed = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_set_motor_speed(world, rdr.make_joint_id(joint), motor_speed);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::DistanceJointSetMaxMotorForce) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let force = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::distance_joint_set_max_motor_force(world, rdr.make_joint_id(joint), force);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::MotorJointSetLinearVelocity) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let velocity = s.vec3();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_linear_velocity(world, rdr.make_joint_id(joint), velocity);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::MotorJointSetAngularVelocity) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let velocity = s.vec3();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_angular_velocity(world, rdr.make_joint_id(joint), velocity);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::MotorJointSetMaxVelocityForce) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let max_force = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_max_velocity_force(world, rdr.make_joint_id(joint), max_force);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::MotorJointSetMaxVelocityTorque) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let max_torque = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_max_velocity_torque(world, rdr.make_joint_id(joint), max_torque);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::MotorJointSetLinearHertz) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_linear_hertz(world, rdr.make_joint_id(joint), hertz);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::MotorJointSetLinearDampingRatio) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_linear_damping_ratio(world, rdr.make_joint_id(joint), damping);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::MotorJointSetAngularHertz) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_angular_hertz(world, rdr.make_joint_id(joint), hertz);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::MotorJointSetAngularDampingRatio) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_angular_damping_ratio(world, rdr.make_joint_id(joint), damping);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::MotorJointSetMaxSpringForce) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let max_force = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_max_spring_force(world, rdr.make_joint_id(joint), max_force);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::MotorJointSetMaxSpringTorque) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let max_torque = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::motor_joint_set_max_spring_torque(world, rdr.make_joint_id(joint), max_torque);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::PrismaticJointEnableSpring) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_spring = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::prismatic_joint_enable_spring(world, rdr.make_joint_id(joint), enable_spring);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::PrismaticJointSetSpringHertz) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::prismatic_joint_set_spring_hertz(world, rdr.make_joint_id(joint), hertz);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::PrismaticJointSetSpringDampingRatio) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::prismatic_joint_set_spring_damping_ratio(world, rdr.make_joint_id(joint), damping_ratio);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::PrismaticJointSetTargetTranslation) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let translation = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::prismatic_joint_set_target_translation(world, rdr.make_joint_id(joint), translation);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::PrismaticJointEnableLimit) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_limit = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::prismatic_joint_enable_limit(world, rdr.make_joint_id(joint), enable_limit);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::PrismaticJointSetLimits) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let lower = s.f32();
            let upper = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::prismatic_joint_set_limits(world, rdr.make_joint_id(joint), lower, upper);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::PrismaticJointEnableMotor) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_motor = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::prismatic_joint_enable_motor(world, rdr.make_joint_id(joint), enable_motor);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::PrismaticJointSetMotorSpeed) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let motor_speed = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::prismatic_joint_set_motor_speed(world, rdr.make_joint_id(joint), motor_speed);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::PrismaticJointSetMaxMotorForce) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let force = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::prismatic_joint_set_max_motor_force(world, rdr.make_joint_id(joint), force);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::RevoluteJointEnableSpring) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_spring = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::revolute_joint_enable_spring(world, rdr.make_joint_id(joint), enable_spring);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::RevoluteJointSetSpringHertz) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::revolute_joint_set_spring_hertz(world, rdr.make_joint_id(joint), hertz);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::RevoluteJointSetSpringDampingRatio) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::revolute_joint_set_spring_damping_ratio(world, rdr.make_joint_id(joint), damping_ratio);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::RevoluteJointSetTargetAngle) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let angle = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::revolute_joint_set_target_angle(world, rdr.make_joint_id(joint), angle);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::RevoluteJointEnableLimit) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_limit = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::revolute_joint_enable_limit(world, rdr.make_joint_id(joint), enable_limit);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::RevoluteJointSetLimits) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let lower = s.f32();
            let upper = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::revolute_joint_set_limits(world, rdr.make_joint_id(joint), lower, upper);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::RevoluteJointEnableMotor) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_motor = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::revolute_joint_enable_motor(world, rdr.make_joint_id(joint), enable_motor);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::RevoluteJointSetMotorSpeed) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let motor_speed = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::revolute_joint_set_motor_speed(world, rdr.make_joint_id(joint), motor_speed);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::RevoluteJointSetMaxMotorTorque) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let torque = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::revolute_joint_set_max_motor_torque(world, rdr.make_joint_id(joint), torque);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::SphericalJointEnableConeLimit) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_limit = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_enable_cone_limit(world, rdr.make_joint_id(joint), enable_limit);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::SphericalJointSetConeLimit) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let angle_radians = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_set_cone_limit(world, rdr.make_joint_id(joint), angle_radians);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::SphericalJointEnableTwistLimit) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_limit = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_enable_twist_limit(world, rdr.make_joint_id(joint), enable_limit);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::SphericalJointSetTwistLimits) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let lower = s.f32();
            let upper = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_set_twist_limits(world, rdr.make_joint_id(joint), lower, upper);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::SphericalJointEnableSpring) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_spring = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_enable_spring(world, rdr.make_joint_id(joint), enable_spring);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::SphericalJointSetSpringHertz) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_set_spring_hertz(world, rdr.make_joint_id(joint), hertz);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::SphericalJointSetSpringDampingRatio) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_set_spring_damping_ratio(world, rdr.make_joint_id(joint), damping_ratio);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::SphericalJointSetTargetRotation) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let target_rotation = s.quat();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_set_target_rotation(world, rdr.make_joint_id(joint), target_rotation);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::SphericalJointEnableMotor) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let enable_motor = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_enable_motor(world, rdr.make_joint_id(joint), enable_motor);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::SphericalJointSetMotorVelocity) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let motor_velocity = s.vec3();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_set_motor_velocity(world, rdr.make_joint_id(joint), motor_velocity);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::SphericalJointSetMaxMotorTorque) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let torque = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::spherical_joint_set_max_motor_torque(world, rdr.make_joint_id(joint), torque);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WeldJointSetLinearHertz) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::weld_joint_set_linear_hertz(world, rdr.make_joint_id(joint), hertz);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WeldJointSetLinearDampingRatio) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::weld_joint_set_linear_damping_ratio(world, rdr.make_joint_id(joint), damping_ratio);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WeldJointSetAngularHertz) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::weld_joint_set_angular_hertz(world, rdr.make_joint_id(joint), hertz);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WeldJointSetAngularDampingRatio) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::weld_joint_set_angular_damping_ratio(world, rdr.make_joint_id(joint), damping_ratio);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WheelJointEnableSuspension) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_enable_suspension(world, rdr.make_joint_id(joint), flag);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WheelJointSetSuspensionHertz) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_suspension_hertz(world, rdr.make_joint_id(joint), hertz);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WheelJointSetSuspensionDampingRatio) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_suspension_damping_ratio(world, rdr.make_joint_id(joint), damping_ratio);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WheelJointEnableSuspensionLimit) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_enable_suspension_limit(world, rdr.make_joint_id(joint), flag);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WheelJointSetSuspensionLimits) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let lower = s.f32();
            let upper = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_suspension_limits(world, rdr.make_joint_id(joint), lower, upper);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WheelJointEnableSpinMotor) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_enable_spin_motor(world, rdr.make_joint_id(joint), flag);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WheelJointSetSpinMotorSpeed) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let speed = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_spin_motor_speed(world, rdr.make_joint_id(joint), speed);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WheelJointSetMaxSpinTorque) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let torque = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_max_spin_torque(world, rdr.make_joint_id(joint), torque);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WheelJointEnableSteering) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_enable_steering(world, rdr.make_joint_id(joint), flag);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WheelJointSetSteeringHertz) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let hertz = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_steering_hertz(world, rdr.make_joint_id(joint), hertz);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WheelJointSetSteeringDampingRatio) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let damping_ratio = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_steering_damping_ratio(world, rdr.make_joint_id(joint), damping_ratio);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WheelJointSetMaxSteeringTorque) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let torque = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_max_steering_torque(world, rdr.make_joint_id(joint), torque);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WheelJointEnableSteeringLimit) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_enable_steering_limit(world, rdr.make_joint_id(joint), flag);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WheelJointSetSteeringLimits) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let lower = s.f32();
            let upper = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_steering_limits(world, rdr.make_joint_id(joint), lower, upper);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::WheelJointSetTargetSteeringAngle) => {
            let mut s = rdr.snap();
            let joint = s.joint_id();
            let radians = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::joint::wheel_joint_set_target_steering_angle(world, rdr.make_joint_id(joint), radians);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::QueryOverlapAABB) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let aabb = s.aabb();
            let filter = s.query_filter();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::recording::query_replay::dispatch_query_overlap_aabb(rdr, world, aabb, filter);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::QueryOverlapShape) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let origin = s.pos();
            let proxy = s.shape_proxy();
            let filter = s.query_filter();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::recording::query_replay::dispatch_query_overlap_shape(rdr, world, origin, proxy, filter);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::QueryCastRay) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let origin = s.pos();
            let translation = s.vec3();
            let filter = s.query_filter();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::recording::query_replay::dispatch_query_cast_ray(rdr, world, origin, translation, filter);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::QueryCastShape) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let origin = s.pos();
            let proxy = s.shape_proxy();
            let translation = s.vec3();
            let filter = s.query_filter();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::recording::query_replay::dispatch_query_cast_shape(rdr, world, origin, proxy, translation, filter);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::QueryCastRayClosest) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let origin = s.pos();
            let translation = s.vec3();
            let filter = s.query_filter();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::recording::query_replay::dispatch_query_cast_ray_closest(rdr, world, origin, translation, filter);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::QueryCastMover) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let origin = s.pos();
            let mover = s.capsule();
            let translation = s.vec3();
            let filter = s.query_filter();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::recording::query_replay::dispatch_query_cast_mover(rdr, world, origin, mover, translation, filter);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::QueryCollideMover) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let origin = s.pos();
            let mover = s.capsule();
            let filter = s.query_filter();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::recording::query_replay::dispatch_query_collide_mover(rdr, world, origin, mover, filter);
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::QueryTag) => {
            let mut s = rdr.snap();
            let key = s.u64();
            rdr.sync_from(&s);
            if rdr.ok {
                rdr.pending_query_key = key;
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::StateHash) => {
            let mut s = rdr.snap();
            let _world_id = s.world_id();
            let hash = s.u64();
            rdr.sync_from(&s);
            if rdr.ok {
                if hash_world_state(world) != hash { rdr.diverged = true; }
            }
            let _ = (payload_start, payload_size);
        }
        Some(RecOp::RecordingBounds) => {
            let mut s = rdr.snap();
            let bounds = s.aabb();
            rdr.sync_from(&s);
            if rdr.ok {
                if let Some(owner) = rdr.owner {
                    // SAFETY: owner is the RecPlayer for this dispatch.
                    unsafe { (*owner).bounds = bounds };
                }
            }
            let _ = (payload_start, payload_size);
        }
        None => {
            eprintln!("b3ReplayFile: unknown opcode 0x{opcode:02X}, skipping {payload_size} bytes");
            if payload_size > (rdr.size - payload_start) as u32 {
                rdr.ok = false;
            } else {
                rdr.cursor = payload_start + payload_size as i32;
            }
        }
    }
    opcode as i32
}
