#!/usr/bin/env python3
"""Generate a clean src/recording/dispatch.rs."""

from __future__ import annotations

import re
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
INL = ROOT / "box3d-cpp-reference" / "src" / "recording_ops.inl"
OUT = ROOT / "src" / "recording" / "dispatch.rs"

TAG_READ = {
    "BOOL": "bool", "I32": "i32", "U8": "u8", "U16": "u16", "U32": "u32", "U64": "u64",
    "F32": "f32", "F64": "f64", "VEC3": "vec3", "QUAT": "quat", "TRANSFORM": "transform",
    "POSITION": "pos", "WORLDXF": "world_xf", "MATRIX3": "matrix3", "AABB": "aabb",
    "SPHERE": "sphere", "CAPSULE": "capsule", "QUERYFILTER": "query_filter",
    "SHAPEPROXY": "shape_proxy", "GEOMID": "u32", "FILTER": "filter", "MATERIAL": "material",
    "MASSDATA": "mass_data", "LOCKS": "locks", "BODYSTR": "body_str", "SHAPESTR": "shape_str",
    "WORLDID": "world_id", "BODYID": "body_id", "SHAPEID": "shape_id", "JOINTID": "joint_id",
    "BODYDEF": "body_def", "SHAPEDEF": "shape_def", "EXPLOSIONDEF": "explosion_def",
    "PARALLELJOINTDEF": "parallel_joint_def", "DISTANCEJOINTDEF": "distance_joint_def",
    "FILTERJOINTDEF": "filter_joint_def", "MOTORJOINTDEF": "motor_joint_def",
    "PRISMATICJOINTDEF": "prismatic_joint_def", "REVOLUTEJOINTDEF": "revolute_joint_def",
    "SPHERICALJOINTDEF": "spherical_joint_def", "WELDJOINTDEF": "weld_joint_def",
    "WHEELJOINTDEF": "wheel_joint_def",
}


def to_snake(f: str) -> str:
    s = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", f)
    s = re.sub(r"([A-Z]+)([A-Z][a-z])", r"\1_\2", s)
    s = s.lower()
    return "type_" if s == "type" else s


def parse_ops():
    text = INL.read_text(encoding="utf-8")
    flat = re.sub(r"\\\n", " ", text)
    flat = re.sub(r"[ \t]*\n[ \t]*", " ", flat)
    ops = []
    for m in re.finditer(r"B3_REC_OP\(\s*(0x[0-9A-Fa-f]+)\s*,\s*(\w+)\s*,\s*(RET_\w+)\s*,", flat):
        i = flat.index("(", m.start()) + 1
        depth = 1
        while i < len(flat) and depth:
            if flat[i] == "(":
                depth += 1
            elif flat[i] == ")":
                depth -= 1
            i += 1
        inner = flat[m.start() : i]
        mm = re.match(
            r"B3_REC_OP\(\s*(0x[0-9A-Fa-f]+)\s*,\s*(\w+)\s*,\s*(RET_\w+)\s*,(.*)\)$",
            inner,
            re.S,
        )
        if not mm:
            continue
        opcode, name, ret, args = mm.groups()
        fields = re.findall(r"ARG\(\s*(\w+)\s*,\s*(\w+)\s*\)", args)
        ops.append((int(opcode, 16), name, ret, fields))
    return ops


def emit_body(name, fields):
    """Emit the dispatch body after locals are read."""
    lines = []

    def args_for_call(skip_world=True):
        out = []
        for tag, field in fields:
            if skip_world and tag == "WORLDID":
                continue
            rf = to_snake(field)
            if tag == "BODYID":
                out.append(f"rdr.make_body_id({rf})")
            elif tag == "SHAPEID":
                out.append(f"rdr.make_shape_id({rf})")
            elif tag == "JOINTID":
                out.append(f"rdr.make_joint_id({rf})")
            elif tag in ("BODYSTR", "SHAPESTR"):
                out.append(f"&{rf}")
            elif tag in ("BODYDEF", "SHAPEDEF") or tag.endswith("JOINTDEF"):
                out.append(f"&{rf}")
            elif tag == "EXPLOSIONDEF":
                out.append(f"&{rf}")
            elif tag in ("SPHERE", "CAPSULE") and name.startswith("ShapeSet"):
                out.append(f"&{rf}")
            else:
                out.append(rf)
        return out

    if name == "DestroyWorld":
        lines.append("                // end-of-session marker\n")
    elif name == "Step":
        lines.append("                world.step(dt, sub_step_count);\n")
    elif name == "StateHash":
        lines.append(
            "                if hash_world_state(world) != hash { rdr.diverged = true; }\n"
        )
    elif name == "RecordingBounds":
        lines.append(
            "                if let Some(o) = rdr.owner { unsafe { (*o).bounds = bounds; } }\n"
        )
    elif name == "QueryTag":
        lines.append("                rdr.pending_query_key = key;\n")
    elif name.startswith("Query"):
        qfn = to_snake(name)
        call_args = ", ".join(args_for_call())
        lines.append(
            f"                crate::recording::query_replay::dispatch_{qfn}(rdr, world, {call_args});\n"
        )
    elif name == "CreateBody":
        lines.append(
            """                let mut s2 = rdr.snap();
                let rec_id = s2.body_id();
                rdr.sync_from(&s2);
                let got = create_body(world, &def);
                RecReader::check_id(&mut rdr.ok, "body", got.index1, got.generation, rec_id.index1, rec_id.generation);
                if let Some(o) = rdr.owner { unsafe { (*o).track_body_create(got); } }
"""
        )
    elif name == "DestroyBody":
        lines.append(
            """                let id = rdr.make_body_id(body);
                if let Some(o) = rdr.owner { unsafe { (*o).track_body_destroy(id); } }
                crate::body::destroy_body(world, id);
"""
        )
    elif name == "BodySetType":
        lines.append(
            """                let id = rdr.make_body_id(body);
                let ty = match type_ { 1 => BodyType::Kinematic, 2 => BodyType::Dynamic, _ => BodyType::Static };
                crate::body::body_set_type(world, id, ty);
"""
        )
    elif name == "CreateSphereShape":
        lines.append(create_shape("create_sphere_shape(world, body_id, &def, &sphere)"))
    elif name == "CreateCapsuleShape":
        lines.append(create_shape("create_capsule_shape(world, body_id, &def, &capsule)"))
    elif name == "CreateHullShape":
        lines.append(
            create_shape(
                """{
                    let hull = convert_bytes_to_hull(&rdr.slots[geometry_id as usize].bytes).expect("hull");
                    create_hull_shape(world, body_id, &def, &hull)
                }"""
            )
        )
    elif name == "CreateMeshShape":
        lines.append(
            create_shape(
                """{
                    let mesh = convert_bytes_to_mesh(&rdr.slots[geometry_id as usize].bytes).expect("mesh");
                    create_mesh_shape(world, body_id, &def, &mesh, scale)
                }"""
            )
        )
    elif name == "CreateHeightFieldShape":
        lines.append(
            create_shape(
                """{
                    let hf = convert_bytes_to_height_field(&rdr.slots[geometry_id as usize].bytes).expect("hf");
                    create_height_field_shape(world, body_id, &def, &hf)
                }"""
            )
        )
    elif name == "CreateCompoundShape":
        lines.append(
            create_shape(
                """{
                    let compound = rdr.slots[geometry_id as usize].ensure_compound().cloned().expect("compound");
                    create_compound_shape(world, body_id, &def, &compound)
                }"""
            )
        )
    elif name.startswith("Create") and "Joint" in name:
        lines.append(
            f"""                let mut s2 = rdr.snap();
                let rec_id = s2.joint_id();
                rdr.sync_from(&s2);
                let mut def = def;
                def.base.body_id_a = rdr.make_body_id(def.base.body_id_a);
                def.base.body_id_b = rdr.make_body_id(def.base.body_id_b);
                let got = {to_snake(name)}(world, &def);
                RecReader::check_id(&mut rdr.ok, "joint", got.index1, got.generation, rec_id.index1, rec_id.generation);
"""
        )
    elif name == "DestroyShape":
        ca = ", ".join(args_for_call())
        lines.append(f"                crate::shape::destroy_shape(world, {ca});\n")
    elif name == "DestroyJoint":
        ca = ", ".join(args_for_call())
        lines.append(f"                crate::joint::destroy_joint(world, {ca});\n")
    elif name.startswith("World"):
        ca = ", ".join(args_for_call())
        lines.append(f"                crate::world::{to_snake(name)}(world, {ca});\n")
    elif name.startswith("Body"):
        ca = ", ".join(args_for_call())
        lines.append(f"                crate::body::{to_snake(name)}(world, {ca});\n")
    elif name.startswith("Shape"):
        ca = ", ".join(args_for_call())
        # shape_apply_wind / enable* may be in mutators
        lines.append(f"                crate::shape::{to_snake(name)}(world, {ca});\n")
    elif name.startswith("Joint") or any(
        name.startswith(p)
        for p in (
            "Parallel", "Distance", "Motor", "Prismatic", "Revolute",
            "Spherical", "Weld", "Wheel",
        )
    ):
        ca = ", ".join(args_for_call())
        lines.append(f"                crate::joint::{to_snake(name)}(world, {ca});\n")
    else:
        lines.append(f"                unimplemented_op(\"{name}\");\n")
    return "".join(lines)


def create_shape(expr: str) -> str:
    return f"""                let mut s2 = rdr.snap();
                let rec_id = s2.shape_id();
                rdr.sync_from(&s2);
                let body_id = rdr.make_body_id(body);
                let got = {expr};
                RecReader::check_id(&mut rdr.ok, "shape", got.index1, got.generation, rec_id.index1, rec_id.generation);
"""


def main():
    ops = parse_ops()
    parts = []
    parts.append('''//! Op-stream dispatcher for recording replay.
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
''')

    for _opcode, name, _ret, fields in ops:
        parts.append(f"        Some(RecOp::{name}) => {{\n")
        parts.append("            let mut s = rdr.snap();\n")
        for tag, field in fields:
            local = '_world_id' if tag == 'WORLDID' else to_snake(field)
            parts.append(f"            let {local} = s.{TAG_READ[tag]}();\n")
        parts.append("            rdr.sync_from(&s);\n")
        parts.append("            if rdr.ok {\n")
        parts.append(emit_body(name, fields))
        parts.append("            }\n")
        parts.append("            let _ = (payload_start, payload_size);\n")
        parts.append("        }\n")

    parts.append('''        None => {
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
''')
    OUT.write_text("".join(parts), encoding="utf-8")
    print(f"wrote {OUT} ({OUT.stat().st_size})")


if __name__ == "__main__":
    main()
