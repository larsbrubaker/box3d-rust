#!/usr/bin/env python3
"""Generate the recording op table and framed writers from recording_ops.inl.

Reads box3d-cpp-reference/src/recording_ops.inl (the B3_REC_OP x-macro manifest) and
writes, under OUT_DIR (default: src/recording):

  ops.rs                 RecOp enum + RecOp::from_u8
  write_ops/mod.rs       family module list
  write_ops/<family>.rs  `impl Recording` framed writers, one file per op family
                         (world, body, shape, joint, joint_setters, query)

The files are written with LF line endings and then formatted with rustfmt (edition
2021), which is how the checked-in files are kept. Ops appear in manifest order.

Usage:
  python scripts/gen_recording_ops.py                  # regenerate in src/recording
  python scripts/gen_recording_ops.py --out-dir DIR    # write elsewhere (e.g. to diff)
  python scripts/gen_recording_ops.py --no-rustfmt     # skip the rustfmt pass

To check for drift without touching src, generate into a scratch directory and diff
(ignoring CR, since git may check the files out with CRLF):
  diff -r --strip-trailing-cr DIR/write_ops src/recording/write_ops
"""

from __future__ import annotations

import argparse
import re
import pathlib
import subprocess

from recording_manifest import parse_ops

ROOT = pathlib.Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "src" / "recording"

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
    "STR": "String",
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
    "MOVERJOINTDEF": "MoverJointDef",
    "POGOJOINTDEF": "PogoJointDef",
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
    "STR": "append_str",
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
    "MOVERJOINTDEF": "append_mover_joint_def",
    "POGOJOINTDEF": "append_pogo_joint_def",
}

REF_TAGS = {
    "STR",
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
    "MOVERJOINTDEF",
    "POGOJOINTDEF",
}


def to_snake(f: str) -> str:
    s = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", f)
    s = re.sub(r"([A-Z]+)([A-Z][a-z])", r"\1_\2", s)
    s = s.lower()
    if s == "type":
        return "type_"
    return s



# Module path for every Rust type a writer signature can mention.
TYPE_MODULE = {
    "ShapeProxy": "crate::distance",
    "Capsule": "crate::geometry",
    "MassData": "crate::geometry",
    "Sphere": "crate::geometry",
    "SurfaceMaterial": "crate::geometry",
    "BodyId": "crate::id",
    "JointId": "crate::id",
    "ShapeId": "crate::id",
    "WorldId": "crate::id",
    "Aabb": "crate::math_functions",
    "Matrix3": "crate::math_functions",
    "Pos": "crate::math_functions",
    "Quat": "crate::math_functions",
    "Transform": "crate::math_functions",
    "Vec3": "crate::math_functions",
    "WorldTransform": "crate::math_functions",
}
for _ty in TAG_TY.values():
    if _ty.endswith("Def") or _ty in ("Filter", "MotionLocks", "QueryFilter"):
        TYPE_MODULE[_ty] = "crate::types"

PRIMITIVES = {"bool", "i32", "u8", "u16", "u32", "u64", "f32", "f64"}

RET_TY = {"RET_BODYID": "BodyId", "RET_SHAPEID": "ShapeId", "RET_JOINTID": "JointId"}
RET_WRITE = {
    "RET_BODYID": "append_body_id",
    "RET_SHAPEID": "append_shape_id",
    "RET_JOINTID": "append_joint_id",
}

JOINT_TYPE_PREFIXES = (
    "Parallel", "Distance", "Motor", "Prismatic", "Revolute", "Spherical", "Weld", "Wheel",
    "Mover", "Pogo",
)

# Writer families: (module, doc noun). List order is the write_ops/mod.rs order.
FAMILIES = [
    ("body", "body"),
    ("joint", "joint"),
    ("joint_setters", "joint"),
    ("query", "query"),
    ("shape", "shape"),
    ("world", "world"),
]


def family_of(name: str) -> str:
    if name in ("DestroyWorld", "Step") or name.startswith("World"):
        return "world"
    if name in ("CreateBody", "DestroyBody") or name.startswith("Body"):
        return "body"
    if name == "DestroyShape" or name.startswith("Shape") or (
        name.startswith("Create") and name.endswith("Shape")
    ):
        return "shape"
    if name == "DestroyJoint" or name.startswith("Joint") or (
        name.startswith("Create") and name.endswith("Joint")
    ):
        return "joint"
    if name.startswith(JOINT_TYPE_PREFIXES) and "Joint" in name:
        return "joint_setters"
    if name.startswith("Query") or name in ("StateHash", "RecordingBounds"):
        return "query"
    raise SystemExit(f"gen_recording_ops: no writer family for op {name}")


def check_tags(ops) -> None:
    for _opcode, name, ret, fields in ops:
        for tag, field in fields:
            if tag not in TAG_TY or tag not in TAG_WRITE:
                raise SystemExit(
                    f"gen_recording_ops: unknown arg tag {tag} ({name}.{field}); "
                    "add it to TAG_TY and TAG_WRITE"
                )
        if ret != "RET_NONE" and ret not in RET_TY:
            raise SystemExit(f"gen_recording_ops: unknown return kind {ret} ({name})")


def emit_writer(name: str, ret: str, fields) -> tuple[str, set[str]]:
    """One framed writer method plus the set of Rust types it mentions."""
    used: set[str] = set()
    fn = to_snake(name)
    params: list[str] = []
    for tag, field in fields:
        rf = to_snake(field)
        ty = TAG_TY[tag]
        if tag == "STR":
            params.append(f"{rf}: &str")
            continue
        used.add(ty)
        if tag in REF_TAGS:
            params.append(f"{rf}: &{ty}")
        else:
            params.append(f"{rf}: {ty}")

    ret_param = ""
    if ret in RET_TY:
        ret_param = f", ret_id: {RET_TY[ret]}"
        used.add(RET_TY[ret])

    param_str = (", " + ", ".join(params)) if params else ""
    out = []
    out.append(f"    /// Write framed `{name}` op.\n")
    out.append(f"    pub fn write_{fn}(&mut self{param_str}{ret_param}) {{\n")
    out.append(f"        self.begin_record(RecOp::{name} as u8);\n")
    for tag, field in fields:
        out.append(f"        self.buffer.{TAG_WRITE[tag]}({to_snake(field)});\n")
    if ret in RET_WRITE:
        out.append(f"        self.buffer.{RET_WRITE[ret]}(ret_id);\n")
    out.append("        self.end_record();\n")
    out.append("    }\n")
    return "".join(out), used


def use_lines(types: set[str]) -> str:
    by_mod: dict[str, list[str]] = {}
    for ty in types:
        if ty in PRIMITIVES:
            continue
        if ty not in TYPE_MODULE:
            raise SystemExit(f"gen_recording_ops: no module known for type {ty}")
        by_mod.setdefault(TYPE_MODULE[ty], []).append(ty)
    lines = []
    for mod in sorted(by_mod):
        names = sorted(by_mod[mod])
        if len(names) == 1:
            lines.append(f"use {mod}::{names[0]};\n")
        else:
            lines.append(f"use {mod}::{{{', '.join(names)}}};\n")
    return "".join(lines)


def gen_ops_rs(ops) -> str:
    lines: list[str] = []
    lines.append(
        """//! Recording op table generated from recording_ops.inl.
//! Opcode constants; the framed write helpers live in write_ops/.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

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
    lines.append("}\n")
    return "".join(lines)


def gen_write_ops(ops) -> dict[str, str]:
    files: dict[str, str] = {}
    mod_lines = [
        """//! Framed op writers for `Recording`, split from ops.rs by op family.
//! Layouts mirror recording_ops.inl.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

"""
    ]
    for module, noun in FAMILIES:
        mod_lines.append(f"mod {module};\n")
        methods: list[str] = []
        used: set[str] = set()
        for _opcode, name, ret, fields in ops:
            if family_of(name) != module:
                continue
            text, tys = emit_writer(name, ret, fields)
            methods.append(text)
            used |= tys
        body = (
            f"""//! Framed {noun} op writers for `Recording`, split from ops.rs to satisfy
//! the file-length limit. Layouts mirror recording_ops.inl.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

"""
            + use_lines(used)
            + "\nuse crate::recording::ops::RecOp;\nuse crate::recording::session::Recording;\n\n"
            + "impl Recording {\n"
            + "\n".join(methods)
            + "}\n"
        )
        files[f"write_ops/{module}.rs"] = body
    files["write_ops/mod.rs"] = "".join(mod_lines)
    return files


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out-dir", type=pathlib.Path, default=OUT_DIR)
    ap.add_argument("--no-rustfmt", action="store_true")
    args = ap.parse_args()

    ops = parse_ops("gen_recording_ops")
    check_tags(ops)
    files = {"ops.rs": gen_ops_rs(ops)}
    files.update(gen_write_ops(ops))

    written = []
    for rel, text in files.items():
        path = args.out_dir / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8", newline="\n")
        written.append(path)
    if not args.no_rustfmt:
        subprocess.run(
            ["rustfmt", "--edition", "2021", *[str(p) for p in written]], check=True
        )
    for path in written:
        print(f"wrote {path} ({path.stat().st_size} bytes)")
    print(f"{len(ops)} ops")


if __name__ == "__main__":
    main()
