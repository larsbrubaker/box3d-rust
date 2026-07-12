import re
import pathlib

src = pathlib.Path("src/joint/types.rs").read_text(encoding="utf-8")
structs = {}
for m in re.finditer(r"pub struct (\w+)\s*\{([^}]+)\}", src, re.S):
    name, body = m.group(1), m.group(2)
    fields = []
    for line in body.splitlines():
        line = line.strip()
        mm = re.match(r"pub (\w+):\s*([^,]+),?", line)
        if mm:
            fields.append((mm.group(1), mm.group(2).strip()))
    if fields:
        structs[name] = fields

structs["Softness"] = [
    ("bias_rate", "f32"),
    ("mass_scale", "f32"),
    ("impulse_scale", "f32"),
]
structs["Vec2"] = [("x", "f32"), ("y", "f32")]

type_writers = {
    "f32": "append_f32",
    "i32": "append_i32",
    "bool": "append_bool",
    "Vec3": "append_vec3",
    "Quat": "append_quat",
    "Transform": "append_transform",
    "Matrix3": "append_matrix3",
    "Vec2": "append_vec2",
    "Softness": "append_softness",
}
type_readers = {
    "f32": "f32",
    "i32": "i32",
    "bool": "bool",
    "Vec3": "vec3",
    "Quat": "quat",
    "Transform": "transform",
    "Matrix3": "matrix3",
    "Vec2": "vec2",
    "Softness": "softness",
}

joint_types = [
    "DistanceJoint",
    "MotorJoint",
    "ParallelJoint",
    "PrismaticJoint",
    "RevoluteJoint",
    "SphericalJoint",
    "WeldJoint",
    "WheelJoint",
]


def ser_fields(var, fields, indent="    "):
    lines = []
    for fname, ftype in fields:
        w = type_writers.get(ftype)
        if not w:
            raise SystemExit(f"unknown type {ftype} in {var}")
        lines.append(f"{indent}buf.{w}({var}.{fname});")
    return lines


def des_fields(fields, indent="        "):
    lines = []
    for fname, ftype in fields:
        r = type_readers.get(ftype)
        lines.append(f"{indent}{fname}: r.{r}(),")
    return lines


out = []
out.append("//! JointSim field-by-field snapshot ser/de.")
out.append("use crate::joint::{")
out.append("    DistanceJoint, Joint, JointEdge, JointSim, JointType, JointUnion, MotorJoint,")
out.append("    ParallelJoint, PrismaticJoint, RevoluteJoint, SphericalJoint, WeldJoint, WheelJoint,")
out.append("};")
out.append("use crate::math_functions::Vec2;")
out.append("use crate::recording::buffer::{RecBuffer, SnapReader};")
out.append("use crate::solver::Softness;")
out.append("")
out.append("impl RecBuffer {")
out.append(
    "    pub fn append_vec2(&mut self, v: Vec2) { self.append_f32(v.x); self.append_f32(v.y); }"
)
out.append(
    "    pub fn append_softness(&mut self, v: Softness) { self.append_f32(v.bias_rate); self.append_f32(v.mass_scale); self.append_f32(v.impulse_scale); }"
)
out.append("}")
out.append("impl SnapReader<'_> {")
out.append('    pub fn vec2(&mut self) -> Vec2 { Vec2 { x: self.f32(), y: self.f32() } }')
out.append(
    "    pub fn softness(&mut self) -> Softness { Softness { bias_rate: self.f32(), mass_scale: self.f32(), impulse_scale: self.f32() } }"
)
out.append("}")
out.append("")
out.append("pub fn ser_joint(buf: &mut RecBuffer, j: &Joint) {")
out.append("    buf.append_u64(0); // user_data scrubbed")
out.append("    buf.append_i32(j.set_index);")
out.append("    buf.append_i32(j.color_index);")
out.append("    buf.append_i32(j.local_index);")
out.append(
    "    for e in &j.edges { buf.append_i32(e.body_id); buf.append_i32(e.prev_key); buf.append_i32(e.next_key); }"
)
out.append("    buf.append_i32(j.joint_id);")
out.append("    buf.append_i32(j.island_id);")
out.append("    buf.append_i32(j.island_index);")
out.append("    buf.append_f32(j.draw_scale);")
out.append("    buf.append_i32(j.type_ as i32);")
out.append("    buf.append_u16(j.generation);")
out.append("    buf.append_bool(j.collide_connected);")
out.append("}")
out.append("")
out.append("pub fn des_joint(r: &mut SnapReader<'_>) -> Joint {")
out.append("    let _user = r.u64();")
out.append("    Joint {")
out.append("        user_data: 0,")
out.append("        set_index: r.i32(),")
out.append("        color_index: r.i32(),")
out.append("        local_index: r.i32(),")
out.append("        edges: [")
out.append(
    "            JointEdge { body_id: r.i32(), prev_key: r.i32(), next_key: r.i32() },"
)
out.append(
    "            JointEdge { body_id: r.i32(), prev_key: r.i32(), next_key: r.i32() },"
)
out.append("        ],")
out.append("        joint_id: r.i32(),")
out.append("        island_id: r.i32(),")
out.append("        island_index: r.i32(),")
out.append("        draw_scale: r.f32(),")
out.append("        type_: match r.i32() {")
out.append("            0 => JointType::Parallel,")
out.append("            1 => JointType::Distance,")
out.append("            2 => JointType::Filter,")
out.append("            3 => JointType::Motor,")
out.append("            4 => JointType::Prismatic,")
out.append("            5 => JointType::Revolute,")
out.append("            6 => JointType::Spherical,")
out.append("            7 => JointType::Weld,")
out.append("            _ => JointType::Wheel,")
out.append("        },")
out.append("        generation: r.u16(),")
out.append("        collide_connected: r.bool(),")
out.append("    }")
out.append("}")
out.append("")
out.append("pub fn ser_joint_sim(buf: &mut RecBuffer, s: &JointSim) {")
out.append("    buf.append_i32(s.joint_id);")
out.append("    buf.append_i32(s.body_id_a);")
out.append("    buf.append_i32(s.body_id_b);")
out.append("    buf.append_i32(s.type_ as i32);")
out.append("    buf.append_transform(s.local_frame_a);")
out.append("    buf.append_transform(s.local_frame_b);")
out.append("    buf.append_f32(s.inv_mass_a);")
out.append("    buf.append_f32(s.inv_mass_b);")
out.append("    buf.append_matrix3(s.inv_i_a);")
out.append("    buf.append_matrix3(s.inv_i_b);")
out.append("    buf.append_f32(s.constraint_hertz);")
out.append("    buf.append_f32(s.constraint_damping_ratio);")
out.append("    buf.append_softness(s.constraint_softness);")
out.append("    buf.append_f32(s.force_threshold);")
out.append("    buf.append_f32(s.torque_threshold);")
out.append("    buf.append_bool(s.fixed_rotation);")
out.append("    match &s.union_ {")
for jt in joint_types:
    short = jt.replace("Joint", "")
    out.append(f"        JointUnion::{short}(j) => {{")
    out.append("            buf.append_i32(s.type_ as i32);")
    out.extend(ser_fields("j", structs[jt], "            "))
    out.append("        }")
out.append("        JointUnion::Filter => {")
out.append("            buf.append_i32(JointType::Filter as i32);")
out.append("        }")
out.append("    }")
out.append("}")
out.append("")
out.append("pub fn des_joint_sim(r: &mut SnapReader<'_>) -> JointSim {")
out.append("    let joint_id = r.i32();")
out.append("    let body_id_a = r.i32();")
out.append("    let body_id_b = r.i32();")
out.append("    let type_i = r.i32();")
out.append("    let type_ = match type_i {")
out.append(
    "        0 => JointType::Parallel, 1 => JointType::Distance, 2 => JointType::Filter,"
)
out.append(
    "        3 => JointType::Motor, 4 => JointType::Prismatic, 5 => JointType::Revolute,"
)
out.append(
    "        6 => JointType::Spherical, 7 => JointType::Weld, _ => JointType::Wheel,"
)
out.append("    };")
out.append("    let local_frame_a = r.transform();")
out.append("    let local_frame_b = r.transform();")
out.append("    let inv_mass_a = r.f32();")
out.append("    let inv_mass_b = r.f32();")
out.append("    let inv_i_a = r.matrix3();")
out.append("    let inv_i_b = r.matrix3();")
out.append("    let constraint_hertz = r.f32();")
out.append("    let constraint_damping_ratio = r.f32();")
out.append("    let constraint_softness = r.softness();")
out.append("    let force_threshold = r.f32();")
out.append("    let torque_threshold = r.f32();")
out.append("    let fixed_rotation = r.bool();")
out.append("    let union_tag = r.i32();")
out.append("    let union_ = match union_tag {")
mapping = {
    0: "Parallel",
    1: "Distance",
    2: "Filter",
    3: "Motor",
    4: "Prismatic",
    5: "Revolute",
    6: "Spherical",
    7: "Weld",
    8: "Wheel",
}
for tag, short in mapping.items():
    if short == "Filter":
        out.append(f"        {tag} => JointUnion::Filter,")
        continue
    jt = short + "Joint"
    out.append(f"        {tag} => JointUnion::{short}({jt} {{")
    out.extend(des_fields(structs[jt], "            "))
    out.append("        }),")
out.append("        _ => JointUnion::Filter,")
out.append("    };")
out.append(
    "    JointSim { joint_id, body_id_a, body_id_b, type_, local_frame_a, local_frame_b,"
)
out.append(
    "        inv_mass_a, inv_mass_b, inv_i_a, inv_i_b, constraint_hertz, constraint_damping_ratio,"
)
out.append(
    "        constraint_softness, force_threshold, torque_threshold, fixed_rotation, union_ }"
)
out.append("}")

path = pathlib.Path("src/recording/snapshot/joints.rs")
path.write_text("\n".join(out) + "\n", encoding="utf-8")
print("wrote", path, "lines", len(out))
for jt in joint_types:
    print(jt, len(structs.get(jt, [])))
