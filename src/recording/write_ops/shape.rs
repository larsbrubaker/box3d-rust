//! Framed shape op writers for `Recording`, split from ops.rs to satisfy
//! the file-length limit. Layouts mirror recording_ops.inl.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::geometry::{Capsule, Sphere, SurfaceMaterial};
use crate::id::{BodyId, ShapeId};
use crate::math_functions::Vec3;
use crate::types::{Filter, ShapeDef};

use crate::recording::ops::RecOp;
use crate::recording::session::Recording;

impl Recording {
    /// Write framed `CreateSphereShape` op.
    pub fn write_create_sphere_shape(
        &mut self,
        body: BodyId,
        def: &ShapeDef,
        sphere: Sphere,
        ret_id: ShapeId,
    ) {
        self.begin_record(RecOp::CreateSphereShape as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_shape_def(def);
        self.buffer.append_sphere(sphere);
        self.buffer.append_shape_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateCapsuleShape` op.
    pub fn write_create_capsule_shape(
        &mut self,
        body: BodyId,
        def: &ShapeDef,
        capsule: Capsule,
        ret_id: ShapeId,
    ) {
        self.begin_record(RecOp::CreateCapsuleShape as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_shape_def(def);
        self.buffer.append_capsule(capsule);
        self.buffer.append_shape_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateHullShape` op.
    pub fn write_create_hull_shape(
        &mut self,
        body: BodyId,
        def: &ShapeDef,
        geometry_id: u32,
        ret_id: ShapeId,
    ) {
        self.begin_record(RecOp::CreateHullShape as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_shape_def(def);
        self.buffer.append_u32(geometry_id);
        self.buffer.append_shape_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateMeshShape` op.
    pub fn write_create_mesh_shape(
        &mut self,
        body: BodyId,
        def: &ShapeDef,
        geometry_id: u32,
        scale: Vec3,
        ret_id: ShapeId,
    ) {
        self.begin_record(RecOp::CreateMeshShape as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_shape_def(def);
        self.buffer.append_u32(geometry_id);
        self.buffer.append_vec3(scale);
        self.buffer.append_shape_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateHeightFieldShape` op.
    pub fn write_create_height_field_shape(
        &mut self,
        body: BodyId,
        def: &ShapeDef,
        geometry_id: u32,
        ret_id: ShapeId,
    ) {
        self.begin_record(RecOp::CreateHeightFieldShape as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_shape_def(def);
        self.buffer.append_u32(geometry_id);
        self.buffer.append_shape_id(ret_id);
        self.end_record();
    }

    /// Write framed `CreateCompoundShape` op.
    pub fn write_create_compound_shape(
        &mut self,
        body: BodyId,
        def: &ShapeDef,
        geometry_id: u32,
        ret_id: ShapeId,
    ) {
        self.begin_record(RecOp::CreateCompoundShape as u8);
        self.buffer.append_body_id(body);
        self.buffer.append_shape_def(def);
        self.buffer.append_u32(geometry_id);
        self.buffer.append_shape_id(ret_id);
        self.end_record();
    }

    /// Write framed `DestroyShape` op.
    pub fn write_destroy_shape(&mut self, shape: ShapeId, update_body_mass: bool) {
        self.begin_record(RecOp::DestroyShape as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_bool(update_body_mass);
        self.end_record();
    }

    /// Write framed `ShapeSetDensity` op.
    pub fn write_shape_set_density(
        &mut self,
        shape: ShapeId,
        density: f32,
        update_body_mass: bool,
    ) {
        self.begin_record(RecOp::ShapeSetDensity as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_f32(density);
        self.buffer.append_bool(update_body_mass);
        self.end_record();
    }

    /// Write framed `ShapeSetFriction` op.
    pub fn write_shape_set_friction(&mut self, shape: ShapeId, friction: f32) {
        self.begin_record(RecOp::ShapeSetFriction as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_f32(friction);
        self.end_record();
    }

    /// Write framed `ShapeSetRestitution` op.
    pub fn write_shape_set_restitution(&mut self, shape: ShapeId, restitution: f32) {
        self.begin_record(RecOp::ShapeSetRestitution as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_f32(restitution);
        self.end_record();
    }

    /// Write framed `ShapeSetSurfaceMaterial` op.
    pub fn write_shape_set_surface_material(&mut self, shape: ShapeId, material: SurfaceMaterial) {
        self.begin_record(RecOp::ShapeSetSurfaceMaterial as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_material(material);
        self.end_record();
    }

    /// Write framed `ShapeSetFilter` op.
    pub fn write_shape_set_filter(
        &mut self,
        shape: ShapeId,
        filter: Filter,
        invoke_contacts: bool,
    ) {
        self.begin_record(RecOp::ShapeSetFilter as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_filter(filter);
        self.buffer.append_bool(invoke_contacts);
        self.end_record();
    }

    /// Write framed `ShapeEnableSensorEvents` op.
    pub fn write_shape_enable_sensor_events(&mut self, shape: ShapeId, flag: bool) {
        self.begin_record(RecOp::ShapeEnableSensorEvents as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `ShapeEnableContactEvents` op.
    pub fn write_shape_enable_contact_events(&mut self, shape: ShapeId, flag: bool) {
        self.begin_record(RecOp::ShapeEnableContactEvents as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `ShapeEnablePreSolveEvents` op.
    pub fn write_shape_enable_pre_solve_events(&mut self, shape: ShapeId, flag: bool) {
        self.begin_record(RecOp::ShapeEnablePreSolveEvents as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `ShapeEnableHitEvents` op.
    pub fn write_shape_enable_hit_events(&mut self, shape: ShapeId, flag: bool) {
        self.begin_record(RecOp::ShapeEnableHitEvents as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_bool(flag);
        self.end_record();
    }

    /// Write framed `ShapeSetSphere` op.
    pub fn write_shape_set_sphere(&mut self, shape: ShapeId, sphere: Sphere) {
        self.begin_record(RecOp::ShapeSetSphere as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_sphere(sphere);
        self.end_record();
    }

    /// Write framed `ShapeSetCapsule` op.
    pub fn write_shape_set_capsule(&mut self, shape: ShapeId, capsule: Capsule) {
        self.begin_record(RecOp::ShapeSetCapsule as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_capsule(capsule);
        self.end_record();
    }

    /// Write framed `ShapeApplyWind` op.
    pub fn write_shape_apply_wind(
        &mut self,
        shape: ShapeId,
        wind: Vec3,
        drag: f32,
        lift: f32,
        max_speed: f32,
        wake: bool,
    ) {
        self.begin_record(RecOp::ShapeApplyWind as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_vec3(wind);
        self.buffer.append_f32(drag);
        self.buffer.append_f32(lift);
        self.buffer.append_f32(max_speed);
        self.buffer.append_bool(wake);
        self.end_record();
    }

    /// Write framed `ShapeSetName` op.
    pub fn write_shape_set_name(&mut self, shape: ShapeId, name: &str) {
        self.begin_record(RecOp::ShapeSetName as u8);
        self.buffer.append_shape_id(shape);
        self.buffer.append_str(name);
        self.end_record();
    }
}
