#!/usr/bin/env python3
"""Generate src/recording/ops.rs from box3d-cpp-reference/src/recording_ops.inl."""

from __future__ import annotations

import re
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
INL = ROOT / "box3d-cpp-reference" / "src" / "recording_ops.inl"
OUT = ROOT / "src" / "recording" / "ops.rs"

TAG_TY = {
    "BOOL": "bool",
    "I32": "i32",
    "U8": "u8",
    "U16": "u16",
    "U32": "u32",
    "U64": "u64",
    "F32": "f32",
    "F64": "f64",
    "VEC3": "Vec3",
    "QUAT": "Quat",
    "TRANSFORM": "Transform",
    "POSITION": "Pos",
    "WORLDXF": "WorldTransform",
    "MATRIX3": "Matrix3",
    "AABB": "Aabb",
    "SPHERE": "Sphere",
    "CAPSULE": "Capsule",
    "QUERYFILTER": "QueryFilter",
    "SHAPEPROXY": "ShapeProxy",
    "GEOMID": "u32",
    "FILTER": "Filter",
    "MATERIAL": "SurfaceMaterial",
    "MASSDATA": "MassData",
    "LOCKS": "MotionLocks",
    "BODYSTR": "String",
    "SHAPESTR": "String",
    "WORLDID": "WorldId",
    "BODYID": "BodyId",
    "SHAPEID": "ShapeId",
    "JOINTID": "JointId",
    "BODYDEF": "BodyDef",
    "SHAPEDEF": "ShapeDef",
    "EXPLOSIONDEF": "ExplosionDef",
    "PARALLELJOINTDEF": "ParallelJointDef",
    "DISTANCEJOINTDEF": "DistanceJointDef",
    "FILTERJOINTDEF": "FilterJointDef",
    "MOTORJOINTDEF": "MotorJointDef",
    "PRISMATICJOINTDEF": "PrismaticJointDef",
    "REVOLUTEJOINTDEF": "RevoluteJointDef",
    "SPHERICALJOINTDEF": "SphericalJointDef",
    "WELDJOINTDEF": "WeldJointDef",
    "WHEELJOINTDEF": "WheelJointDef",
}

TAG_WRITE = {
    "BOOL": "append_bool",
    "I32": "append_i32",
    "U8": "append_u8",
    "U16": "append_u16",
    "U32": "append_u32",
    "U64": "append_u64",
    "F32": "append_f32",
    "F64": "append_f64",
    "VEC3": "append_vec3",
    "QUAT": "append_quat",
    "TRANSFORM": "append_transform",
    "POSITION": "append_pos",
    "WORLDXF": "append_world_xf",
    "MATRIX3": "append_matrix3",
    "AABB": "append_aabb",
    "SPHERE": "append_sphere",
    "CAPSULE": "append_capsule",
    "QUERYFILTER": "append_query_filter",
    "SHAPEPROXY": "append_shape_proxy",
    "GEOMID": "append_u32",
    "FILTER": "append_filter",
    "MATERIAL": "append_material",
    "MASSDATA": "append_mass_data",
    "LOCKS": "append_locks",
    "BODYSTR": "append_str",
    "SHAPESTR": "append_str",
    "WORLDID": "append_world_id",
    "BODYID": "append_body_id",
    "SHAPEID": "append_shape_id",
    "JOINTID": "append_joint_id",
    "BODYDEF": "append_body_def",
    "SHAPEDEF": "append_shape_def",
    "EXPLOSIONDEF": "append_explosion_def",
    "PARALLELJOINTDEF": "append_parallel_joint_def",
    "DISTANCEJOINTDEF": "append_distance_joint_def",
    "FILTERJOINTDEF": "append_filter_joint_def",
    "MOTORJOINTDEF": "append_motor_joint_def",
    "PRISMATICJOINTDEF": "append_prismatic_joint_def",
    "REVOLUTEJOINTDEF": "append_revolute_joint_def",
    "SPHERICALJOINTDEF": "append_spherical_joint_def",
    "WELDJOINTDEF": "append_weld_joint_def",
    "WHEELJOINTDEF": "append_wheel_joint_def",
}

REF_TAGS = {
    "BODYSTR",
    "SHAPESTR",
    "BODYDEF",
    "SHAPEDEF",
    "QUERYFILTER",
    "SHAPEPROXY",
    "PARALLELJOINTDEF",
    "DISTANCEJOINTDEF",
    "FILTERJOINTDEF",
    "MOTORJOINTDEF",
    "PRISMATICJOINTDEF",
    "REVOLUTEJOINTDEF",
    "SPHERICALJOINTDEF",
    "WELDJOINTDEF",
    "WHEELJOINTDEF",
}


def to_snake(f: str) -> str:
    s = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", f)
    s = re.sub(r"([A-Z]+)([A-Z][a-z])", r"\1_\2", s)
    s = s.lower()
    if s == "type":
        return "type_"
    return s


def parse_ops() -> list[tuple[int, str, str, list[tuple[str, str]]]]:
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


def main() -> None:
    ops = parse_ops()
    lines: list[str] = []
    lines.append(
        """//! Recording op table generated from recording_ops.inl.
//! Opcode constants, arg structs, and framed write helpers.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::distance::ShapeProxy;
use crate::geometry::{Capsule, MassData, Sphere, SurfaceMaterial};
use crate::id::{BodyId, JointId, ShapeId, WorldId};
use crate::math_functions::{Aabb, Pos, Quat, Transform, Vec3, WorldTransform};
use crate::types::{
    BodyDef, DistanceJointDef, ExplosionDef, Filter, FilterJointDef, MotionLocks, MotorJointDef,
    ParallelJointDef, PrismaticJointDef, QueryFilter, RevoluteJointDef, ShapeDef, SphericalJointDef,
    WeldJointDef, WheelJointDef,
};

use super::session::Recording;

"""
    )

    lines.append("/// Opcode constants from recording_ops.inl.\n")
    lines.append("#[repr(u8)]\n#[derive(Debug, Clone, Copy, PartialEq, Eq)]\n")
    lines.append("pub enum RecOp {\n")
    for opcode, name, _ret, _fields in ops:
        lines.append(f"    {name} = 0x{opcode:02X},\n")
    lines.append("}\n\n")

    lines.append("impl RecOp {\n")
    lines.append("    pub fn from_u8(v: u8) -> Option<Self> {\n")
    lines.append("        match v {\n")
    for opcode, name, _ret, _fields in ops:
        lines.append(f"            0x{opcode:02X} => Some(Self::{name}),\n")
    lines.append("            _ => None,\n")
    lines.append("        }\n")
    lines.append("    }\n")
    lines.append("}\n\n")

    for opcode, name, _ret, fields in ops:
        lines.append(f"/// Args for `{name}` (op 0x{opcode:02X}).\n")
        lines.append("#[derive(Debug, Clone)]\n")
        lines.append(f"pub struct Args{name} {{\n")
        for tag, field in fields:
            lines.append(f"    pub {to_snake(field)}: {TAG_TY[tag]},\n")
        lines.append("}\n\n")

    lines.append("impl Recording {\n")
    for opcode, name, ret, fields in ops:
        fn = to_snake(name)
        params: list[str] = []
        for tag, field in fields:
            rf = to_snake(field)
            ty = TAG_TY[tag]
            if tag in REF_TAGS:
                if tag in ("BODYSTR", "SHAPESTR"):
                    params.append(f"{rf}: &str")
                else:
                    params.append(f"{rf}: &{ty}")
            else:
                params.append(f"{rf}: {ty}")

        ret_param = ""
        if ret == "RET_BODYID":
            ret_param = ", ret_id: BodyId"
        elif ret == "RET_SHAPEID":
            ret_param = ", ret_id: ShapeId"
        elif ret == "RET_JOINTID":
            ret_param = ", ret_id: JointId"

        param_str = (", " + ", ".join(params)) if params else ""
        lines.append(f"    /// Write framed `{name}` op.\n")
        lines.append(f"    pub fn write_{fn}(&mut self{param_str}{ret_param}) {{\n")
        lines.append(f"        self.begin_record(RecOp::{name} as u8);\n")
        for tag, field in fields:
            rf = to_snake(field)
            w = TAG_WRITE[tag]
            lines.append(f"        self.buffer.{w}({rf});\n")
        if ret == "RET_BODYID":
            lines.append("        self.buffer.append_body_id(ret_id);\n")
        elif ret == "RET_SHAPEID":
            lines.append("        self.buffer.append_shape_id(ret_id);\n")
        elif ret == "RET_JOINTID":
            lines.append("        self.buffer.append_joint_id(ret_id);\n")
        lines.append("        self.end_record();\n")
        lines.append("    }\n\n")
    lines.append("}\n")

    OUT.write_text("".join(lines), encoding="utf-8")
    print(f"wrote {OUT} ({OUT.stat().st_size} bytes, {len(ops)} ops)")


if __name__ == "__main__":
    main()
