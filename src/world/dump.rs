//! World dump helpers from physics_world.c (`b3World_DumpAwake`,
//! `b3World_DumpShapeBounds`). Writes debug files like the C helpers.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::body::{body_flags, get_body_sim, get_body_state_index};
use crate::constants::NULL_NAME;
use crate::contact::contact_flags;
use crate::core::NULL_INDEX;
use crate::dynamic_tree::{ALLOCATED_NODE, LEAF_NODE};
use crate::hull::{get_hull_points, HullData};
use crate::math_functions::{to_vec3, VEC3_ZERO};
use crate::shape::ShapeGeometry;
use crate::solver_set::{AWAKE_SET, STATIC_SET};
use crate::types::BodyType;
use crate::world::World;
use std::collections::BTreeSet;
use std::fs::File;
use std::io::Write;

/// Dump leaf AABB bounds for one broad-phase tree to `box3d_bounds.txt`.
/// (b3World_DumpShapeBounds)
pub fn world_dump_shape_bounds(world: &World, type_: BodyType) {
    debug_assert!((BodyType::Static as i32..=BodyType::Dynamic as i32).contains(&(type_ as i32)));
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    let Ok(mut file) = File::create("box3d_bounds.txt") else {
        return;
    };

    let tree = &world.broad_phase.trees[type_ as usize];
    let required_flags = ALLOCATED_NODE | LEAF_NODE;
    let capacity = tree.node_capacity();
    for i in 0..capacity {
        let node = &tree.nodes[i as usize];
        if (node.flags & required_flags) != required_flags {
            // skip internal and free nodes
            continue;
        }

        let a = node.aabb.lower_bound;
        let b = node.aabb.upper_bound;
        let _ = writeln!(
            file,
            "{:.9} {:.9} {:.9} {:.9} {:.9} {:.9}",
            a.x, a.y, a.z, b.x, b.y, b.z
        );
    }
}

/// Dump awake bodies (and touching statics) as a C++ recreation snippet to
/// `box3d_dump.inl`. (b3World_DumpAwake)
pub fn world_dump_awake(world: &World) {
    if world.locked {
        return;
    }

    let Ok(mut file) = File::create("box3d_dump.inl") else {
        return;
    };

    let g = world.gravity;
    let _ = writeln!(
        file,
        "b3Vec3 gravity = {{{:.9}, {:.9}, {:.9}}};",
        g.x, g.y, g.z
    );
    let _ = writeln!(file, "b3World_SetGravity(m_worldId, gravity);");
    let _ = writeln!(file, "std::vector<b3BodyId> bodies;");
    let _ = writeln!(file, "std::vector<b3JointId> joints;\n");

    let mut static_bodies = BTreeSet::new();

    for i in 0..world.bodies.len() as i32 {
        let body = &world.bodies[i as usize];
        if body.id == NULL_INDEX {
            continue;
        }
        if body.set_index != AWAKE_SET {
            continue;
        }

        dump_body(world, &mut file, body.id);

        let mut edge_key = body.head_contact_key;
        while edge_key != NULL_INDEX {
            let contact_id = edge_key >> 1;
            let edge_index = edge_key & 1;

            let contact = &world.contacts[contact_id as usize];
            edge_key = contact.edges[edge_index as usize].next_key;

            if (contact.flags & contact_flags::TOUCHING) == 0 {
                continue;
            }

            let other_index = 1 - edge_index;
            let other_body = &world.bodies[contact.edges[other_index as usize].body_id as usize];
            if other_body.set_index == STATIC_SET {
                static_bodies.insert(other_body.id);
            }
        }
    }

    for &body_id in &static_bodies {
        dump_body(world, &mut file, body_id);
    }
}

/// (b3DumpBody)
fn dump_body(world: &World, file: &mut File, body_index: i32) {
    let body = &world.bodies[body_index as usize];
    let sim = get_body_sim(world, body_index);
    let p = to_vec3(sim.transform.p);
    let q = sim.transform.q;

    let (v, w) = if let Some(state_local) = get_body_state_index(world, body_index) {
        let state = &world.solver_sets[body.set_index as usize].body_states[state_local as usize];
        (state.linear_velocity, state.angular_velocity)
    } else {
        (VEC3_ZERO, VEC3_ZERO)
    };

    let _ = writeln!(file, "{{");
    let _ = writeln!(file, "  b3BodyDef bd = b3DefaultBodyDef();");
    let name = if body.name_id != NULL_NAME {
        world.names.find_name(body.name_id).unwrap_or("")
    } else {
        ""
    };
    if !name.is_empty() {
        let _ = writeln!(file, "  bd.name = \"{}\";", name);
    }
    let _ = writeln!(file, "  bd.type = b3BodyType({});", body.type_ as i32);
    let _ = writeln!(
        file,
        "  bd.position = {{{:.9}, {:.9}, {:.9}}};",
        p.x, p.y, p.z
    );
    let _ = writeln!(
        file,
        "  bd.rotation = {{{{{:.9}, {:.9}, {:.9}}}, {:.9}}};",
        q.v.x, q.v.y, q.v.z, q.s
    );
    let _ = writeln!(
        file,
        "  bd.linearVelocity = {{{:.9}, {:.9}, {:.9}}};",
        v.x, v.y, v.z
    );
    let _ = writeln!(
        file,
        "  bd.angularVelocity = {{{:.9}, {:.9}, {:.9}}};",
        w.x, w.y, w.z
    );
    let _ = writeln!(file, "  bd.linearDamping = {:.9};", sim.linear_damping);
    let _ = writeln!(file, "  bd.angularDamping = {:.9};", sim.angular_damping);
    let _ = writeln!(
        file,
        "  bd.enableSleep = bool({});",
        (body.flags & body_flags::ENABLE_SLEEP) != 0
    );
    let _ = writeln!(
        file,
        "  bd.isAwake = bool({});",
        body.set_index == AWAKE_SET
    );
    let _ = writeln!(file, "  bd.gravityScale = {:.9};", sim.gravity_scale);
    let _ = writeln!(file, "  b3BodyId bodyId = b3CreateBody(m_worldId, &bd);");
    let _ = writeln!(file, "  bodies.push_back(bodyId);");
    let _ = writeln!(file);

    let mut shape_index = body.head_shape_id;
    while shape_index != NULL_INDEX {
        let _ = writeln!(file, "  {{");
        dump_shape(world, file, shape_index);
        let _ = writeln!(file, "  }}");
        shape_index = world.shapes[shape_index as usize].next_shape_id;
    }
    let _ = writeln!(file, "}}");
}

/// (b3DumpShape) — covers capsule/sphere/hull; mesh/compound/height are stubs like C.
fn dump_shape(world: &World, file: &mut File, shape_index: i32) {
    let shape = &world.shapes[shape_index as usize];

    let _ = writeln!(file, "    b3ShapeDef sd = b3DefaultShapeDef();");
    let _ = writeln!(file, "    sd.density = {:.9};", shape.density);
    let _ = writeln!(
        file,
        "    sd.isSensor = bool({});",
        shape.sensor_index != NULL_INDEX
    );
    let _ = writeln!(
        file,
        "    sd.filter.categoryBits = 0x{:x};",
        shape.filter.category_bits
    );
    let _ = writeln!(
        file,
        "    sd.filter.maskBits = 0x{:x};",
        shape.filter.mask_bits
    );
    let _ = writeln!(
        file,
        "    sd.filter.groupIndex = {};",
        shape.filter.group_index
    );

    debug_assert!(shape.material_count() >= 1);
    let m = shape.get_material(0);
    let _ = writeln!(file, "    sd.baseMaterial.friction = {:.9};", m.friction);
    let _ = writeln!(
        file,
        "    sd.baseMaterial.restitution = {:.9};",
        m.restitution
    );
    let _ = writeln!(
        file,
        "    sd.baseMaterial.rollingResistance = {:.9};",
        m.rolling_resistance
    );

    match &shape.geometry {
        ShapeGeometry::Capsule(s) => {
            let _ = writeln!(file, "    b3CapsuleShape shape;");
            let _ = writeln!(
                file,
                "    shape.center1 = {{{:.9}, {:.9}, {:.9}}};",
                s.center1.x, s.center1.y, s.center1.z
            );
            let _ = writeln!(
                file,
                "    shape.center2 = {{{:.9}, {:.9}, {:.9}}};",
                s.center2.x, s.center2.y, s.center2.z
            );
            let _ = writeln!(file, "    shape.radius = {:.9};", s.radius);
            let _ = writeln!(file, "    b3CreateCapsuleShape(bodyId, &sd, &shape);");
        }
        ShapeGeometry::Sphere(s) => {
            let _ = writeln!(file, "    b3SphereShape shape;");
            let _ = writeln!(
                file,
                "    shape.center = {{{:.9}, {:.9}, {:.9}}};",
                s.center.x, s.center.y, s.center.z
            );
            let _ = writeln!(file, "    shape.radius = {:.9};", s.radius);
            let _ = writeln!(file, "    b3CreateSphereShape(bodyId, &sd, &shape);");
        }
        ShapeGeometry::Hull(hull) => {
            dump_hull_shape(file, hull);
        }
        ShapeGeometry::Compound(_) | ShapeGeometry::HeightField(_) | ShapeGeometry::Mesh { .. } => {
            // C leaves compound/height as todo; mesh writes a binary dump that
            // the Rust port does not need until a dump consumer lands.
            let _ = writeln!(file, "    // dump: shape type not emitted");
        }
    }
}

fn dump_hull_shape(file: &mut File, hull: &HullData) {
    let vertex_count = hull.vertex_count;
    let vs = get_hull_points(hull);
    let _ = writeln!(file, "    b3Vec3 vs[{}];", vertex_count);
    for i in 0..vertex_count as usize {
        let _ = writeln!(
            file,
            "    vs[{}] = {{{:.9}, {:.9}, {:.9}}};",
            i, vs[i].x, vs[i].y, vs[i].z
        );
    }
    let _ = writeln!(
        file,
        "    b3HullData* hullData = b3CreateHull(vs, {}, {});",
        vertex_count, vertex_count
    );
    let _ = writeln!(file, "    b3CreateHullShape(bodyId, &sd, hullData);");
    let _ = writeln!(file, "    b3DestroyHull(hullData);");
}
