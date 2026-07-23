//! Body, body-sim/state, solver-set, and island-link snapshot POD codecs.
//! Split from pods.rs to satisfy the file-length limit.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

//! Shared POD helpers for world snapshots.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::pods::{des_i32_array, ser_i32_array};
use crate::body::{Body, BodySim, BodyState};
use crate::island::{ContactLink, JointLink};
use crate::recording::buffer::{RecBuffer, SnapReader};
use crate::recording::snapshot::joints::{des_joint_sim, ser_joint_sim};
use crate::sensor::Visitor;
use crate::solver_set::SolverSet;
use crate::types::BodyType;

pub use super::joints::{des_joint, ser_joint};

pub fn ser_body(buf: &mut RecBuffer, b: &Body) {
    buf.append_u64(0); // user_data scrubbed
    buf.append_i32(b.set_index);
    buf.append_i32(b.local_index);
    buf.append_i32(b.head_contact_key);
    buf.append_i32(b.contact_count);
    buf.append_i32(b.head_shape_id);
    buf.append_i32(b.shape_count);
    buf.append_i32(b.head_chain_id);
    buf.append_i32(b.head_joint_key);
    buf.append_i32(b.joint_count);
    buf.append_i32(b.island_id);
    buf.append_i32(b.island_index);
    buf.append_f32(b.sleep_threshold);
    buf.append_f32(b.sleep_time);
    buf.append_f32(b.sleep_velocity);
    buf.append_f32(b.mass);
    buf.append_matrix3(b.inertia);
    buf.append_i32(b.body_move_index);
    buf.append_i32(b.id);
    buf.append_u32(b.flags);
    buf.append_u32(b.name_id);
    buf.append_i32(b.type_ as i32);
    buf.append_u16(b.generation);
}

pub fn des_body(r: &mut SnapReader<'_>) -> Body {
    let _user = r.u64();
    Body {
        user_data: 0,
        set_index: r.i32(),
        local_index: r.i32(),
        head_contact_key: r.i32(),
        contact_count: r.i32(),
        head_shape_id: r.i32(),
        shape_count: r.i32(),
        head_chain_id: r.i32(),
        head_joint_key: r.i32(),
        joint_count: r.i32(),
        island_id: r.i32(),
        island_index: r.i32(),
        sleep_threshold: r.f32(),
        sleep_time: r.f32(),
        sleep_velocity: r.f32(),
        mass: r.f32(),
        inertia: r.matrix3(),
        body_move_index: r.i32(),
        id: r.i32(),
        flags: r.u32(),
        name_id: r.u32(),
        type_: match r.i32() {
            1 => BodyType::Kinematic,
            2 => BodyType::Dynamic,
            _ => BodyType::Static,
        },
        generation: r.u16(),
    }
}

pub fn ser_body_sim(buf: &mut RecBuffer, s: &BodySim) {
    buf.append_world_xf(s.transform);
    buf.append_pos(s.center);
    buf.append_quat(s.rotation0);
    buf.append_pos(s.center0);
    buf.append_vec3(s.local_center);
    buf.append_vec3(s.force);
    buf.append_vec3(s.torque);
    buf.append_f32(s.inv_mass);
    buf.append_matrix3(s.inv_inertia_local);
    buf.append_matrix3(s.inv_inertia_world);
    buf.append_f32(s.min_extent);
    buf.append_vec3(s.max_extent);
    buf.append_f32(s.linear_damping);
    buf.append_f32(s.angular_damping);
    buf.append_f32(s.gravity_scale);
    buf.append_i32(s.body_id);
    buf.append_u32(s.flags);
}

pub fn des_body_sim(r: &mut SnapReader<'_>) -> BodySim {
    BodySim {
        transform: r.world_xf(),
        center: r.pos(),
        rotation0: r.quat(),
        center0: r.pos(),
        local_center: r.vec3(),
        force: r.vec3(),
        torque: r.vec3(),
        inv_mass: r.f32(),
        inv_inertia_local: r.matrix3(),
        inv_inertia_world: r.matrix3(),
        min_extent: r.f32(),
        max_extent: r.vec3(),
        linear_damping: r.f32(),
        angular_damping: r.f32(),
        gravity_scale: r.f32(),
        body_id: r.i32(),
        flags: r.u32(),
    }
}

pub fn ser_body_state(buf: &mut RecBuffer, s: &BodyState) {
    buf.append_vec3(s.linear_velocity);
    buf.append_vec3(s.angular_velocity);
    buf.append_vec3(s.delta_position);
    buf.append_quat(s.delta_rotation);
    buf.append_u32(s.flags);
}

pub fn des_body_state(r: &mut SnapReader<'_>) -> BodyState {
    BodyState {
        linear_velocity: r.vec3(),
        angular_velocity: r.vec3(),
        delta_position: r.vec3(),
        delta_rotation: r.quat(),
        flags: r.u32(),
    }
}

pub fn ser_solver_set(buf: &mut RecBuffer, set: &SolverSet) {
    buf.append_i32(set.set_index);
    buf.append_i32(set.body_sims.len() as i32);
    for s in &set.body_sims {
        ser_body_sim(buf, s);
    }
    buf.append_i32(set.body_states.len() as i32);
    for s in &set.body_states {
        ser_body_state(buf, s);
    }
    buf.append_i32(set.joint_sims.len() as i32);
    for s in &set.joint_sims {
        ser_joint_sim(buf, s);
    }
    ser_i32_array(buf, &set.contact_indices);
    buf.append_i32(set.island_sims.len() as i32);
    for s in &set.island_sims {
        buf.append_i32(s.island_id);
    }
}

pub fn des_solver_set(r: &mut SnapReader<'_>) -> SolverSet {
    let set_index = r.i32();
    let n = r.i32();
    let mut body_sims = Vec::with_capacity(n.max(0) as usize);
    for _ in 0..n.max(0) {
        body_sims.push(des_body_sim(r));
    }
    let n = r.i32();
    let mut body_states = Vec::with_capacity(n.max(0) as usize);
    for _ in 0..n.max(0) {
        body_states.push(des_body_state(r));
    }
    let n = r.i32();
    let mut joint_sims = Vec::with_capacity(n.max(0) as usize);
    for _ in 0..n.max(0) {
        joint_sims.push(des_joint_sim(r));
    }
    let contact_indices = des_i32_array(r);
    let n = r.i32();
    let mut island_sims = Vec::with_capacity(n.max(0) as usize);
    for _ in 0..n.max(0) {
        island_sims.push(crate::island::IslandSim { island_id: r.i32() });
    }
    SolverSet {
        body_sims,
        body_states,
        joint_sims,
        contact_indices,
        island_sims,
        set_index,
    }
}

pub fn ser_visitors(buf: &mut RecBuffer, arr: &[Visitor]) {
    buf.append_i32(arr.len() as i32);
    for v in arr {
        buf.append_i32(v.shape_id);
        buf.append_u16(v.generation);
    }
}

pub fn des_visitors(r: &mut SnapReader<'_>) -> Vec<Visitor> {
    let cnt = r.i32();
    let mut out = Vec::with_capacity(cnt.max(0) as usize);
    for _ in 0..cnt.max(0) {
        out.push(Visitor {
            shape_id: r.i32(),
            generation: r.u16(),
        });
    }
    out
}

pub fn ser_contact_links(buf: &mut RecBuffer, arr: &[ContactLink]) {
    buf.append_i32(arr.len() as i32);
    for l in arr {
        buf.append_i32(l.contact_id);
        buf.append_i32(l.body_id_a);
        buf.append_i32(l.body_id_b);
    }
}

pub fn des_contact_links(r: &mut SnapReader<'_>) -> Vec<ContactLink> {
    let cnt = r.i32();
    let mut out = Vec::with_capacity(cnt.max(0) as usize);
    for _ in 0..cnt.max(0) {
        out.push(ContactLink {
            contact_id: r.i32(),
            body_id_a: r.i32(),
            body_id_b: r.i32(),
        });
    }
    out
}

pub fn ser_joint_links(buf: &mut RecBuffer, arr: &[JointLink]) {
    buf.append_i32(arr.len() as i32);
    for l in arr {
        buf.append_i32(l.joint_id);
        buf.append_i32(l.body_id_a);
        buf.append_i32(l.body_id_b);
    }
}

pub fn des_joint_links(r: &mut SnapReader<'_>) -> Vec<JointLink> {
    let cnt = r.i32();
    let mut out = Vec::with_capacity(cnt.max(0) as usize);
    for _ in 0..cnt.max(0) {
        out.push(JointLink {
            joint_id: r.i32(),
            body_id_a: r.i32(),
            body_id_b: r.i32(),
        });
    }
    out
}
