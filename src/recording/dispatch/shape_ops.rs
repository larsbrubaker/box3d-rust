//! shape ops handlers for the replay dispatcher, split from
//! dispatch.rs to satisfy the file-length limit. One function per op family;
//! returns false when the op belongs to another family.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::compound::CompoundData;
use crate::height_field::convert_bytes_to_height_field;
use crate::hull::convert_bytes_to_hull;
use crate::mesh::convert_bytes_to_mesh;
use crate::recording::dispatch::RecReader;
use crate::recording::ops::RecOp;
use crate::recording::registry::GeometryKind;
use crate::shape::{
    create_baked_compound_shape, create_capsule_shape, create_height_field_shape,
    create_hull_shape, create_mesh_shape, create_sphere_shape,
};

#[allow(unused_imports)]
use crate::recording::query_replay;

fn kind_name(kind: GeometryKind) -> &'static str {
    match kind {
        GeometryKind::Hull => "hull",
        GeometryKind::Mesh => "mesh",
        GeometryKind::HeightField => "height field",
        GeometryKind::Compound => "compound",
    }
}

/// Range- and kind-check a geometry registry slot, or fail the replay.
///
/// Divergence from C: `b3ReplayFile` casts the slot bytes straight to a `b3HullData*` /
/// `b3MeshData*` / `b3HeightFieldData*` and trusts the recording, so a corrupt or
/// kind-mismatched geometryId makes it read garbage. The Rust port validates instead and
/// fails the replay with a "b3ReplayFile: ..." message, matching the graceful path the
/// out-of-range check already used. Well-formed recordings always resolve, so the happy
/// path is unchanged.
fn check_slot(rdr: &mut RecReader<'_>, kind: GeometryKind, geometry_id: u32) -> bool {
    let name = kind_name(kind);
    if geometry_id as usize >= rdr.slots.len() {
        eprintln!("b3ReplayFile: {name} geometryId {geometry_id} out of range");
        rdr.ok = false;
        return false;
    }
    let actual = rdr.slots[geometry_id as usize].kind;
    if actual != kind {
        let actual = kind_name(actual);
        eprintln!("b3ReplayFile: {name} geometryId {geometry_id} is a {actual} slot");
        rdr.ok = false;
        return false;
    }
    true
}

/// Resolve a geometry registry slot for a shape op, or fail the replay. See
/// [`check_slot`] for the divergence from C.
fn slot_geometry<T>(
    rdr: &mut RecReader<'_>,
    kind: GeometryKind,
    geometry_id: u32,
    convert: fn(&[u8]) -> Option<T>,
) -> Option<T> {
    if !check_slot(rdr, kind, geometry_id) {
        return None;
    }
    match convert(&rdr.slots[geometry_id as usize].bytes) {
        Some(value) => Some(value),
        None => {
            let name = kind_name(kind);
            eprintln!("b3ReplayFile: {name} geometryId {geometry_id} has a corrupt blob");
            rdr.ok = false;
            None
        }
    }
}

/// [`slot_geometry`] for compounds, which convert lazily through the slot's cache.
fn slot_compound(rdr: &mut RecReader<'_>, geometry_id: u32) -> Option<CompoundData> {
    if !check_slot(rdr, GeometryKind::Compound, geometry_id) {
        return None;
    }
    match rdr.slots[geometry_id as usize].ensure_compound().cloned() {
        Some(compound) => Some(compound),
        None => {
            eprintln!("b3ReplayFile: compound geometryId {geometry_id} has a corrupt blob");
            rdr.ok = false;
            None
        }
    }
}

/// Handle one shape op. Returns false if `op` is not in this family.
pub(super) fn dispatch(
    op: RecOp,
    rdr: &mut RecReader<'_>,
    payload_start: i32,
    payload_size: u32,
) -> bool {
    let world = unsafe { &mut *rdr.world };
    let _ = world;
    match op {
        RecOp::CreateSphereShape => {
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
                RecReader::check_id(
                    &mut rdr.ok,
                    "shape",
                    got.index1,
                    got.generation,
                    rec_id.index1,
                    rec_id.generation,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::CreateCapsuleShape => {
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
                RecReader::check_id(
                    &mut rdr.ok,
                    "shape",
                    got.index1,
                    got.generation,
                    rec_id.index1,
                    rec_id.generation,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::CreateHullShape => {
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
                let Some(hull) =
                    slot_geometry(rdr, GeometryKind::Hull, geometry_id, convert_bytes_to_hull)
                else {
                    return true;
                };
                let got = create_hull_shape(world, body_id, &def, &hull);
                RecReader::check_id(
                    &mut rdr.ok,
                    "shape",
                    got.index1,
                    got.generation,
                    rec_id.index1,
                    rec_id.generation,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::CreateMeshShape => {
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
                let Some(mesh) =
                    slot_geometry(rdr, GeometryKind::Mesh, geometry_id, convert_bytes_to_mesh)
                else {
                    return true;
                };
                let got = create_mesh_shape(world, body_id, &def, &mesh, scale);
                RecReader::check_id(
                    &mut rdr.ok,
                    "shape",
                    got.index1,
                    got.generation,
                    rec_id.index1,
                    rec_id.generation,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::CreateHeightFieldShape => {
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
                let Some(hf) = slot_geometry(
                    rdr,
                    GeometryKind::HeightField,
                    geometry_id,
                    convert_bytes_to_height_field,
                ) else {
                    return true;
                };
                let got = create_height_field_shape(world, body_id, &def, &hf);
                RecReader::check_id(
                    &mut rdr.ok,
                    "shape",
                    got.index1,
                    got.generation,
                    rec_id.index1,
                    rec_id.generation,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::CreateCompoundShape => {
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
                let Some(compound) = slot_compound(rdr, geometry_id) else {
                    return true;
                };
                let got = create_baked_compound_shape(world, body_id, &def, &compound);
                RecReader::check_id(
                    &mut rdr.ok,
                    "shape",
                    got.index1,
                    got.generation,
                    rec_id.index1,
                    rec_id.generation,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::DestroyShape => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let update_body_mass = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::destroy_shape(world, rdr.make_shape_id(shape), update_body_mass);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::ShapeSetDensity => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let density = s.f32();
            let update_body_mass = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_set_density(
                    world,
                    rdr.make_shape_id(shape),
                    density,
                    update_body_mass,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::ShapeSetFriction => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let friction = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_set_friction(world, rdr.make_shape_id(shape), friction);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::ShapeSetRestitution => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let restitution = s.f32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_set_restitution(world, rdr.make_shape_id(shape), restitution);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::ShapeSetSurfaceMaterial => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let material = s.material();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_set_surface_material(world, rdr.make_shape_id(shape), material);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::ShapeSetFilter => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let filter = s.filter();
            let invoke_contacts = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_set_filter(
                    world,
                    rdr.make_shape_id(shape),
                    filter,
                    invoke_contacts,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::ShapeEnableSensorEvents => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_enable_sensor_events(world, rdr.make_shape_id(shape), flag);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::ShapeEnableContactEvents => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_enable_contact_events(world, rdr.make_shape_id(shape), flag);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::ShapeEnablePreSolveEvents => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_enable_pre_solve_events(world, rdr.make_shape_id(shape), flag);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::ShapeEnableHitEvents => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let flag = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_enable_hit_events(world, rdr.make_shape_id(shape), flag);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::ShapeSetSphere => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let sphere = s.sphere();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_set_sphere(world, rdr.make_shape_id(shape), &sphere);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::ShapeSetCapsule => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let capsule = s.capsule();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_set_capsule(world, rdr.make_shape_id(shape), &capsule);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::ShapeApplyWind => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let wind = s.vec3();
            let drag = s.f32();
            let lift = s.f32();
            let max_speed = s.f32();
            let wake = s.bool();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_apply_wind(
                    world,
                    rdr.make_shape_id(shape),
                    wind,
                    drag,
                    lift,
                    max_speed,
                    wake,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::ShapeSetName => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let name = s.str_owned();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_set_name(world, rdr.make_shape_id(shape), &name);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::ShapeSetMeshMaterial => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let material = s.material();
            let index = s.i32();
            rdr.sync_from(&s);
            if rdr.ok {
                crate::shape::shape_set_mesh_material(
                    world,
                    rdr.make_shape_id(shape),
                    material,
                    index,
                );
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::ShapeSetHull => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let geometry_id = s.u32();
            rdr.sync_from(&s);
            if rdr.ok {
                let shape_id = rdr.make_shape_id(shape);
                let Some(hull) =
                    slot_geometry(rdr, GeometryKind::Hull, geometry_id, convert_bytes_to_hull)
                else {
                    return true;
                };
                crate::shape::shape_set_hull(world, shape_id, &hull);
            }
            let _ = (payload_start, payload_size);
        }
        RecOp::ShapeSetMesh => {
            let mut s = rdr.snap();
            let shape = s.shape_id();
            let geometry_id = s.u32();
            let scale = s.vec3();
            rdr.sync_from(&s);
            if rdr.ok {
                let shape_id = rdr.make_shape_id(shape);
                let Some(mesh) =
                    slot_geometry(rdr, GeometryKind::Mesh, geometry_id, convert_bytes_to_mesh)
                else {
                    return true;
                };
                crate::shape::shape_set_mesh(world, shape_id, &mesh, scale);
            }
            let _ = (payload_start, payload_size);
        }
        _ => return false,
    }
    true
}
