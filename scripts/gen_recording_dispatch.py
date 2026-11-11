#!/usr/bin/env python3
"""Generate the recording replay dispatcher from recording_ops.inl.

Reads box3d-cpp-reference/src/recording_ops.inl (the B3_REC_OP x-macro manifest) and
writes, under OUT_DIR (default: src/recording/dispatch):

  mod.rs                         RecReader + dispatch_one (tries each family in turn)
  world_ops.rs, body_ops.rs,     one `dispatch(op, rdr, ..) -> bool` per op family;
  shape_ops.rs, joint_ops.rs,    each arm reads the op payload with SnapReader and
  joint_linear_type_ops.rs,      calls the matching public API function, mirroring the
  joint_angular_type_ops.rs,     per-op cases of b3ReplayFile in replay.c
  query_ops.rs

The files are written with LF line endings and then formatted with rustfmt (edition
2021), which is how the checked-in files are kept. Ops appear in manifest order.

Ops whose replay is not a plain "read args, call API" (create ops, geometry-registry
lookups, markers) are spelled out in emit_body(); an op that no rule covers is an error,
so a new op in the manifest has to be routed here deliberately.

Usage:
  python scripts/gen_recording_dispatch.py                  # regenerate in src/recording/dispatch
  python scripts/gen_recording_dispatch.py --out-dir DIR    # write elsewhere (e.g. to diff)
  python scripts/gen_recording_dispatch.py --no-rustfmt     # skip the rustfmt pass

To check for drift without touching src, generate into a scratch directory and diff
(ignoring CR, since git may check the files out with CRLF):
  diff -r --strip-trailing-cr DIR src/recording/dispatch
"""

from __future__ import annotations

import argparse
import re
import pathlib
import subprocess

from recording_manifest import parse_ops

ROOT = pathlib.Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "src" / "recording" / "dispatch"

TAG_READ = {
    "BOOL": "bool", "I32": "i32", "U8": "u8", "U16": "u16", "U32": "u32", "U64": "u64",
    "F32": "f32", "F64": "f64", "VEC3": "vec3", "QUAT": "quat", "TRANSFORM": "transform",
    "POSITION": "pos", "WORLDXF": "world_xf", "MATRIX3": "matrix3", "AABB": "aabb",
    "SPHERE": "sphere", "CAPSULE": "capsule", "QUERYFILTER": "query_filter",
    "SHAPEPROXY": "shape_proxy", "GEOMID": "u32", "FILTER": "filter", "MATERIAL": "material",
    "MASSDATA": "mass_data", "LOCKS": "locks", "STR": "str_owned",
    "WORLDID": "world_id", "BODYID": "body_id", "SHAPEID": "shape_id", "JOINTID": "joint_id",
    "BODYDEF": "body_def", "SHAPEDEF": "shape_def", "EXPLOSIONDEF": "explosion_def",
    "PARALLELJOINTDEF": "parallel_joint_def", "DISTANCEJOINTDEF": "distance_joint_def",
    "FILTERJOINTDEF": "filter_joint_def", "MOTORJOINTDEF": "motor_joint_def",
    "PRISMATICJOINTDEF": "prismatic_joint_def", "REVOLUTEJOINTDEF": "revolute_joint_def",
    "SPHERICALJOINTDEF": "spherical_joint_def", "WELDJOINTDEF": "weld_joint_def",
    "WHEELJOINTDEF": "wheel_joint_def", "MOVERJOINTDEF": "mover_joint_def",
    "POGOJOINTDEF": "pogo_joint_def",
}

LINEAR_JOINT_PREFIXES = ("Parallel", "Distance", "Motor", "Prismatic", "Mover", "Pogo")
ANGULAR_JOINT_PREFIXES = ("Revolute", "Spherical", "Weld", "Wheel")


def to_snake(f: str) -> str:
    s = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", f)
    s = re.sub(r"([A-Z]+)([A-Z][a-z])", r"\1_\2", s)
    s = s.lower()
    return "type_" if s == "type" else s



def check_tags(ops) -> None:
    for _opcode, name, _ret, fields in ops:
        for tag, field in fields:
            if tag not in TAG_READ:
                raise SystemExit(
                    f"gen_recording_dispatch: unknown arg tag {tag} ({name}.{field}); "
                    "add it to TAG_READ"
                )


def family_of(name: str) -> str:
    if name in ("DestroyWorld", "Step") or name.startswith("World"):
        return "world_ops"
    if name in ("CreateBody", "DestroyBody") or name.startswith("Body"):
        return "body_ops"
    if name == "DestroyShape" or name.startswith("Shape") or (
        name.startswith("Create") and name.endswith("Shape")
    ):
        return "shape_ops"
    if name == "DestroyJoint" or name.startswith("Joint") or (
        name.startswith("Create") and name.endswith("Joint")
    ):
        return "joint_ops"
    if name.startswith(LINEAR_JOINT_PREFIXES) and "Joint" in name:
        return "joint_linear_type_ops"
    if name.startswith(ANGULAR_JOINT_PREFIXES) and "Joint" in name:
        return "joint_angular_type_ops"
    if name.startswith("Query") or name in ("StateHash", "RecordingBounds"):
        return "query_ops"
    raise SystemExit(f"gen_recording_dispatch: no dispatch family for op {name}")


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
            elif tag == "STR":
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
            """                if let Some(owner) = rdr.owner {
                    // SAFETY: owner is the RecPlayer for this dispatch.
                    unsafe { (*owner).bounds = bounds };
                }
"""
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
                rdr.pending_body_create = Some(got);
"""
        )
    elif name == "DestroyBody":
        lines.append(
            """                let id = rdr.make_body_id(body);
                rdr.pending_body_destroy = Some(id);
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
        lines.append(create_shape("", "create_sphere_shape(world, body_id, &def, &sphere)"))
    elif name == "CreateCapsuleShape":
        lines.append(create_shape("", "create_capsule_shape(world, body_id, &def, &capsule)"))
    elif name == "CreateHullShape":
        lines.append(
            create_shape(
                slot_lookup("hull", "Hull", "convert_bytes_to_hull"),
                "create_hull_shape(world, body_id, &def, &hull)",
            )
        )
    elif name == "CreateMeshShape":
        lines.append(
            create_shape(
                slot_lookup("mesh", "Mesh", "convert_bytes_to_mesh"),
                "create_mesh_shape(world, body_id, &def, &mesh, scale)",
            )
        )
    elif name == "CreateHeightFieldShape":
        lines.append(
            create_shape(
                slot_lookup("hf", "HeightField", "convert_bytes_to_height_field"),
                "create_height_field_shape(world, body_id, &def, &hf)",
            )
        )
    elif name == "CreateCompoundShape":
        lines.append(
            create_shape(
                """                let Some(compound) = slot_compound(rdr, geometry_id) else {
                    return true;
                };
""",
                "create_baked_compound_shape(world, body_id, &def, &compound)",
            )
        )
    elif name == "ShapeSetHull":
        lines.append(
            "                let shape_id = rdr.make_shape_id(shape);\n"
            + slot_lookup("hull", "Hull", "convert_bytes_to_hull")
            + "                crate::shape::shape_set_hull(world, shape_id, &hull);\n"
        )
    elif name == "ShapeSetMesh":
        lines.append(
            "                let shape_id = rdr.make_shape_id(shape);\n"
            + slot_lookup("mesh", "Mesh", "convert_bytes_to_mesh")
            + "                crate::shape::shape_set_mesh(world, shape_id, &mesh, scale);\n"
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
        lines.append(f"                crate::shape::{to_snake(name)}(world, {ca});\n")
    elif name.startswith("Joint") or name.startswith(
        LINEAR_JOINT_PREFIXES + ANGULAR_JOINT_PREFIXES
    ):
        ca = ", ".join(args_for_call())
        lines.append(f"                crate::joint::{to_snake(name)}(world, {ca});\n")
    else:
        raise SystemExit(f"gen_recording_dispatch: no replay rule for op {name}")
    return "".join(lines)


def slot_lookup(local: str, kind: str, convert: str) -> str:
    """Resolve a geometry registry slot (shape_ops.rs slot_geometry) or end the arm."""
    return f"""                let Some({local}) = slot_geometry(rdr, GeometryKind::{kind}, geometry_id, {convert}) else {{
                    return true;
                }};
"""


def create_shape(lookup: str, expr: str) -> str:
    return f"""                let mut s2 = rdr.snap();
                let rec_id = s2.shape_id();
                rdr.sync_from(&s2);
                let body_id = rdr.make_body_id(body);
{lookup}                let got = {expr};
                RecReader::check_id(&mut rdr.ok, "shape", got.index1, got.generation, rec_id.index1, rec_id.generation);
"""


MOD_RS = """//! Op-stream dispatcher for recording replay.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::id::{BodyId, JointId, ShapeId};
use crate::recording::buffer::SnapReader;
use crate::recording::ops::RecOp;
use crate::recording::registry::RegistrySlot;
use crate::recording::session::RecTag;
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

@@MODS@@
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

    let Some(op) = RecOp::from_u8(opcode) else {
        eprintln!("b3ReplayFile: unknown opcode 0x{opcode:02X}, skipping {payload_size} bytes");
        if payload_size > (rdr.size - payload_start) as u32 {
            rdr.ok = false;
        } else {
            rdr.cursor = payload_start + payload_size as i32;
        }
        return opcode as i32;
    };

    let handled = @@CHAIN@@;
    debug_assert!(handled, "op {op:?} not claimed by any dispatch family");

    opcode as i32
}
"""

GENERIC_HEADER = """//! {noun} ops handlers for the replay dispatcher, split from
//! dispatch.rs to satisfy the file-length limit. One function per op family;
//! returns false when the op belongs to another family.
"""

SHAPE_HELPERS = """/// The geometry name used in C's replay messages.
fn kind_name(kind: GeometryKind) -> &'static str {
    match kind {
        GeometryKind::Hull => "hull",
        GeometryKind::Mesh => "mesh",
        GeometryKind::HeightField => "heightfield",
        GeometryKind::Compound => "compound",
    }
}

/// Range- and kind-check a geometry registry slot via [`get_slot`], or fail the replay
/// (the `b3RecGetSlot` + "is invalid" pattern every geometry dispatch arm uses).
fn check_slot(rdr: &mut RecReader<'_>, kind: GeometryKind, geometry_id: u32) -> bool {
    if get_slot(&mut rdr.slots, geometry_id, kind).is_none() {
        let name = kind_name(kind);
        eprintln!("b3ReplayFile: {name} geometryId {geometry_id} is invalid");
        rdr.ok = false;
        return false;
    }
    true
}

/// Resolve a geometry registry slot for a shape op, or fail the replay.
///
/// Divergence from C: after `b3RecGetSlot`, `b3ReplayFile` casts hull, mesh, and height field
/// slot bytes straight to a `b3HullData*` / `b3MeshData*` / `b3HeightFieldData*` and trusts the
/// recording, so a corrupt blob makes it read garbage. The Rust converters validate instead,
/// and a rejected blob fails the replay with a "geometry %u is corrupt" message (C printed it
/// for height fields and compounds before upstream 5d83df8). Well-formed recordings always
/// resolve, so the happy path is unchanged.
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
            eprintln!("b3ReplayFile: {name} geometry {geometry_id} is corrupt");
            rdr.ok = false;
            None
        }
    }
}

/// [`slot_geometry`] for compounds. The registry loader already validated and parsed every
/// compound slot ([`RegistrySlot::compound`](crate::recording::RegistrySlot::compound)), so
/// like C this only needs the range and kind check.
fn slot_compound(rdr: &mut RecReader<'_>, geometry_id: u32) -> Option<CompoundData> {
    if !check_slot(rdr, GeometryKind::Compound, geometry_id) {
        return None;
    }
    let compound = rdr.slots[geometry_id as usize].compound.clone();
    debug_assert!(compound.is_some());
    if compound.is_none() {
        rdr.ok = false;
    }
    compound
}

"""

# Dispatch families in dispatch_one try order:
# (module, module doc header, `Handle one {noun} op` noun, extra `use` lines, helpers).
FAMILIES = [
    (
        "world_ops",
        GENERIC_HEADER.format(noun="world"),
        "world",
        [],
        "",
    ),
    (
        "body_ops",
        GENERIC_HEADER.format(noun="body"),
        "body",
        ["crate::body::create_body", "crate::types::BodyType"],
        "",
    ),
    (
        "shape_ops",
        GENERIC_HEADER.format(noun="shape"),
        "shape",
        [
            "crate::compound::CompoundData",
            "crate::height_field::convert_bytes_to_height_field",
            "crate::hull::convert_bytes_to_hull",
            "crate::mesh::convert_bytes_to_mesh",
            "crate::recording::registry::{get_slot, GeometryKind}",
            "crate::shape::{create_baked_compound_shape, create_capsule_shape, "
            "create_height_field_shape, create_hull_shape, create_mesh_shape, "
            "create_sphere_shape}",
        ],
        SHAPE_HELPERS,
    ),
    (
        "joint_ops",
        GENERIC_HEADER.format(noun="joint"),
        "joint",
        [
            "crate::joint::{create_distance_joint, create_filter_joint, create_motor_joint, "
            "create_mover_joint, create_parallel_joint, create_pogo_joint, "
            "create_prismatic_joint, create_revolute_joint, create_spherical_joint, "
            "create_weld_joint, create_wheel_joint}",
        ],
        "",
    ),
    (
        "joint_linear_type_ops",
        """//! Parallel/distance/motor/prismatic/mover/pogo joint setter replay handlers,
//! dispatch.rs to satisfy the file-length limit. One function per op family;
//! returns false when the op belongs to another family.
""",
        "joint_type",
        [],
        "",
    ),
    (
        "joint_angular_type_ops",
        """//! Revolute/spherical/weld/wheel joint setter replay handlers,
//! dispatch.rs to satisfy the file-length limit. One function per op family;
//! returns false when the op belongs to another family.
""",
        "joint_type",
        [],
        "",
    ),
    (
        "query_ops",
        GENERIC_HEADER.format(noun="query"),
        "query",
        ["crate::recording::hash::hash_world_state"],
        "",
    ),
]


def emit_arm(name, fields) -> str:
    parts = [f"        RecOp::{name} => {{\n", "            let mut s = rdr.snap();\n"]
    for tag, field in fields:
        local = "_world_id" if tag == "WORLDID" else to_snake(field)
        parts.append(f"            let {local} = s.{TAG_READ[tag]}();\n")
    parts.append("            rdr.sync_from(&s);\n")
    parts.append("            if rdr.ok {\n")
    parts.append(emit_body(name, fields))
    parts.append("            }\n")
    parts.append("            let _ = (payload_start, payload_size);\n")
    parts.append("        }\n")
    return "".join(parts)


def gen_family(ops, module, header, noun, uses, helpers) -> str:
    use_items = sorted(
        uses + ["crate::recording::dispatch::RecReader", "crate::recording::ops::RecOp"]
    )
    arms = [emit_arm(name, fields) for _o, name, _r, fields in ops if family_of(name) == module]
    return (
        header
        + "//!\n//! SPDX-FileCopyrightText: 2026 Erin Catto\n//! SPDX-License-Identifier: MIT\n\n"
        + "".join(f"use {u};\n" for u in use_items)
        + "\n#[allow(unused_imports)]\nuse crate::recording::query_replay;\n\n"
        + helpers
        + f"/// Handle one {noun} op. Returns false if `op` is not in this family.\n"
        + "pub(super) fn dispatch(op: RecOp, rdr: &mut RecReader<'_>, payload_start: i32, "
        "payload_size: u32) -> bool {\n"
        + "    let world = unsafe { &mut *rdr.world };\n"
        + "    let _ = world;\n"
        + "    match op {\n"
        + "".join(arms)
        + "        _ => return false,\n"
        + "    }\n"
        + "    true\n"
        + "}\n"
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out-dir", type=pathlib.Path, default=OUT_DIR)
    ap.add_argument("--no-rustfmt", action="store_true")
    args = ap.parse_args()

    ops = parse_ops("gen_recording_dispatch")
    check_tags(ops)
    for _o, name, _r, _f in ops:
        family_of(name)

    files = {}
    for module, header, noun, uses, helpers in FAMILIES:
        files[f"{module}.rs"] = gen_family(ops, module, header, noun, uses, helpers)
    mods = "".join(f"mod {m};\n" for m in sorted(f[0] for f in FAMILIES))
    chain = " || ".join(
        f"{f[0]}::dispatch(op, rdr, payload_start, payload_size)" for f in FAMILIES
    )
    files["mod.rs"] = MOD_RS.replace("@@MODS@@", mods).replace("@@CHAIN@@", chain)

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
