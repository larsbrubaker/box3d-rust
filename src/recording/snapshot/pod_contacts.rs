//! Manifold, shape, contact, and constraint-graph snapshot POD codecs.
//! Split from pods.rs to satisfy the file-length limit.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

//! Shared POD helpers for world snapshots.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use super::pods::{des_bit_set, des_i32_array, ser_bit_set, ser_i32_array};
use crate::bitset::BitSet;
use crate::constants::MAX_MANIFOLD_POINTS;
use crate::contact::{
    contact_flags, Contact, ContactCache, ContactEdge, ContactGeometry, ContactSpec, ConvexContact,
    MeshContact, TriangleCache,
};
use crate::core::NULL_INDEX;
use crate::distance::SimplexCache;
use crate::geometry::ShapeType;
use crate::height_field::convert_bytes_to_height_field;
use crate::hull::convert_bytes_to_hull;
use crate::manifold::{Manifold, ManifoldPoint, SatCache};
use crate::math_functions::Aabb;
use crate::mesh::convert_bytes_to_mesh;
use crate::recording::buffer::{RecBuffer, SnapReader};
use crate::recording::registry::{GeometryRegistry, RegistrySlot};
use crate::recording::snapshot::joints::{des_joint_sim, ser_joint_sim};
use crate::shape::{Shape, ShapeGeometry};
use crate::world::World;

pub fn ser_manifold(buf: &mut RecBuffer, m: &Manifold) {
    for p in &m.points {
        ser_manifold_point(buf, p);
    }
    buf.append_vec3(m.normal);
    buf.append_f32(m.twist_impulse);
    buf.append_vec3(m.friction_impulse);
    buf.append_vec3(m.rolling_impulse);
    buf.append_i32(m.point_count);
}

fn ser_manifold_point(buf: &mut RecBuffer, p: &ManifoldPoint) {
    buf.append_vec3(p.anchor_a);
    buf.append_vec3(p.anchor_b);
    buf.append_f32(p.separation);
    buf.append_f32(p.base_separation);
    buf.append_f32(p.normal_impulse);
    buf.append_f32(p.total_normal_impulse);
    buf.append_f32(p.normal_velocity);
    buf.append_u32(p.feature_id);
    buf.append_i32(p.triangle_index);
    buf.append_bool(p.persisted);
}

pub fn des_manifold(r: &mut SnapReader<'_>) -> Manifold {
    let mut points = [ManifoldPoint::default(); MAX_MANIFOLD_POINTS];
    for p in &mut points {
        *p = des_manifold_point(r);
    }
    Manifold {
        points,
        normal: r.vec3(),
        twist_impulse: r.f32(),
        friction_impulse: r.vec3(),
        rolling_impulse: r.vec3(),
        point_count: r.i32(),
    }
}

fn des_manifold_point(r: &mut SnapReader<'_>) -> ManifoldPoint {
    ManifoldPoint {
        anchor_a: r.vec3(),
        anchor_b: r.vec3(),
        separation: r.f32(),
        base_separation: r.f32(),
        normal_impulse: r.f32(),
        total_normal_impulse: r.f32(),
        normal_velocity: r.f32(),
        feature_id: r.u32(),
        triangle_index: r.i32(),
        persisted: r.bool(),
    }
}

fn ser_contact_cache(buf: &mut RecBuffer, c: &ContactCache) {
    match c {
        ContactCache::Sat(s) => {
            buf.append_u8(0);
            buf.append_f32(s.separation);
            buf.append_u8(s.type_);
            buf.append_u8(s.index_a);
            buf.append_u8(s.index_b);
            buf.append_u8(s.hit);
        }
        ContactCache::Simplex(s) => {
            buf.append_u8(1);
            buf.append_f32(s.metric);
            buf.append_u16(s.count);
            buf.append(&s.index_a);
            buf.append(&s.index_b);
        }
    }
}

fn des_contact_cache(r: &mut SnapReader<'_>) -> ContactCache {
    match r.u8() {
        0 => ContactCache::Sat(SatCache {
            separation: r.f32(),
            type_: r.u8(),
            index_a: r.u8(),
            index_b: r.u8(),
            hit: r.u8(),
        }),
        _ => {
            let metric = r.f32();
            let count = r.u16();
            let mut index_a = [0u8; 4];
            let mut index_b = [0u8; 4];
            r.copy_bytes(&mut index_a);
            r.copy_bytes(&mut index_b);
            ContactCache::Simplex(SimplexCache {
                metric,
                count,
                index_a,
                index_b,
            })
        }
    }
}

pub fn ser_shapes(buf: &mut RecBuffer, world: &World, registry: &mut GeometryRegistry) {
    buf.append_i32(world.shapes.len() as i32);
    for (i, shape) in world.shapes.iter().enumerate() {
        let is_live = shape.id == i as i32;
        ser_shape_scalars(buf, shape);
        if !is_live {
            buf.append_i32(0); // materialCount
            buf.append_i32(-1); // geo kind sentinel
            continue;
        }
        if shape.materials.is_empty() {
            buf.append_i32(0);
        } else {
            buf.append_i32(shape.materials.len() as i32);
            for m in &shape.materials {
                buf.append_material(*m);
            }
        }
        match &shape.geometry {
            ShapeGeometry::Sphere(s) => {
                buf.append_i32(ShapeType::Sphere as i32);
                buf.append_sphere(*s);
            }
            ShapeGeometry::Capsule(c) => {
                buf.append_i32(ShapeType::Capsule as i32);
                buf.append_capsule(*c);
            }
            ShapeGeometry::Hull(h) => {
                buf.append_i32(ShapeType::Hull as i32);
                let gid = registry.intern_hull(h);
                buf.append_u32(gid);
            }
            ShapeGeometry::Mesh { data, scale } => {
                buf.append_i32(ShapeType::Mesh as i32);
                let gid = registry.intern_mesh(data);
                buf.append_u32(gid);
                buf.append_vec3(*scale);
            }
            ShapeGeometry::HeightField(hf) => {
                buf.append_i32(ShapeType::Height as i32);
                let gid = registry.intern_height_field(hf);
                buf.append_u32(gid);
            }
            ShapeGeometry::Compound(c) => {
                buf.append_i32(ShapeType::Compound as i32);
                let gid = registry.intern_compound(c);
                buf.append_u32(gid);
            }
        }
    }
}

fn ser_shape_scalars(buf: &mut RecBuffer, s: &Shape) {
    buf.append_i32(s.id);
    buf.append_i32(s.body_id);
    buf.append_i32(s.prev_shape_id);
    buf.append_i32(s.next_shape_id);
    buf.append_i32(s.sensor_index);
    buf.append_i32(s.proxy_key);
    buf.append_f32(s.density);
    buf.append_f32(s.explosion_scale);
    buf.append_f32(s.aabb_margin);
    buf.append_aabb(s.aabb);
    buf.append_aabb(s.fat_aabb);
    buf.append_vec3(s.local_centroid);
    buf.append_material(s.material);
    buf.append_filter(s.filter);
    buf.append_u64(0); // user_data scrubbed
    buf.append_u64(0); // user_shape scrubbed
    buf.append_u32(s.name_id);
    buf.append_u16(s.generation);
    buf.append_u8(s.flags);
    buf.append_i32(s.shape_type() as i32);
}

pub fn des_shapes(r: &mut SnapReader<'_>, world: &mut World, slots: &mut [RegistrySlot]) {
    let count = r.i32();
    if r.ok && !r.check_count(count, 64, 64) {
        r.ok = false;
    }
    if !r.ok {
        return;
    }
    // Release hulls before overwrite
    free_live_shapes(world);

    world.shapes.clear();
    world.shapes.reserve(count.max(0) as usize);

    for i in 0..count.max(0) {
        let mut shape = des_shape_scalars(r);
        let is_live = shape.id == i;
        let mat_count = r.i32();
        if !r.ok {
            break;
        }
        if !is_live {
            let _ = r.i32(); // geo sentinel
            world.shapes.push(shape);
            continue;
        }
        if mat_count > 0 {
            let mut mats = Vec::with_capacity(mat_count as usize);
            for _ in 0..mat_count {
                mats.push(r.material());
            }
            shape.materials = mats;
        }
        let geo_kind = r.i32();
        shape.geometry = match geo_kind {
            x if x == ShapeType::Sphere as i32 => ShapeGeometry::Sphere(r.sphere()),
            x if x == ShapeType::Capsule as i32 => ShapeGeometry::Capsule(r.capsule()),
            x if x == ShapeType::Hull as i32 => {
                let gid = r.u32() as usize;
                if gid >= slots.len() {
                    r.ok = false;
                    ShapeGeometry::default()
                } else if let Some(hull) = convert_bytes_to_hull(&slots[gid].bytes) {
                    ShapeGeometry::Hull(world.hull_database.add(&hull))
                } else {
                    r.ok = false;
                    ShapeGeometry::default()
                }
            }
            x if x == ShapeType::Mesh as i32 => {
                let gid = r.u32() as usize;
                let scale = r.vec3();
                if gid >= slots.len() {
                    r.ok = false;
                    ShapeGeometry::default()
                } else if let Some(mesh) = convert_bytes_to_mesh(&slots[gid].bytes) {
                    ShapeGeometry::Mesh { data: mesh, scale }
                } else {
                    r.ok = false;
                    ShapeGeometry::default()
                }
            }
            x if x == ShapeType::Height as i32 => {
                let gid = r.u32() as usize;
                if gid >= slots.len() {
                    r.ok = false;
                    ShapeGeometry::default()
                } else if let Some(hf) = convert_bytes_to_height_field(&slots[gid].bytes) {
                    ShapeGeometry::HeightField(hf)
                } else {
                    r.ok = false;
                    ShapeGeometry::default()
                }
            }
            x if x == ShapeType::Compound as i32 => {
                let gid = r.u32() as usize;
                if gid >= slots.len() {
                    r.ok = false;
                    ShapeGeometry::default()
                } else if let Some(c) = slots[gid].ensure_compound().cloned() {
                    ShapeGeometry::Compound(c)
                } else {
                    r.ok = false;
                    ShapeGeometry::default()
                }
            }
            _ => {
                r.ok = false;
                ShapeGeometry::default()
            }
        };
        world.shapes.push(shape);
    }
}

fn des_shape_scalars(r: &mut SnapReader<'_>) -> Shape {
    Shape {
        id: r.i32(),
        body_id: r.i32(),
        prev_shape_id: r.i32(),
        next_shape_id: r.i32(),
        sensor_index: r.i32(),
        proxy_key: r.i32(),
        density: r.f32(),
        explosion_scale: r.f32(),
        aabb_margin: r.f32(),
        aabb: r.aabb(),
        fat_aabb: r.aabb(),
        local_centroid: r.vec3(),
        material: r.material(),
        materials: Vec::new(),
        filter: r.filter(),
        user_data: {
            let _ = r.u64();
            0
        },
        user_shape: {
            let _ = r.u64();
            0
        },
        name_id: r.u32(),
        generation: r.u16(),
        flags: r.u8(),
        geometry: {
            let _type = r.i32();
            ShapeGeometry::default()
        },
    }
}

pub fn free_live_shapes(world: &mut World) {
    for i in 0..world.shapes.len() {
        if world.shapes[i].id != i as i32 {
            continue;
        }
        world.shapes[i].materials.clear();
        if let ShapeGeometry::Hull(ref hull) = world.shapes[i].geometry {
            let hull = hull.clone();
            world.hull_database.release(&hull);
        }
    }
}

pub fn ser_contacts(buf: &mut RecBuffer, world: &World) {
    buf.append_i32(world.contacts.len() as i32);
    for (i, c) in world.contacts.iter().enumerate() {
        let is_live = c.contact_id == i as i32;
        ser_contact_scalars(buf, c);
        if !is_live {
            buf.append_i32(0);
            continue;
        }
        buf.append_i32(c.manifolds.len() as i32);
        for m in &c.manifolds {
            ser_manifold(buf, m);
        }
        if c.flags & contact_flags::SIM_MESH_CONTACT != 0 {
            if let ContactGeometry::Mesh(ref mesh) = c.geometry {
                buf.append_i32(mesh.triangle_cache.len() as i32);
                for t in &mesh.triangle_cache {
                    buf.append_i32(t.triangle_index);
                    ser_contact_cache(buf, &t.cache);
                }
            } else {
                buf.append_i32(0);
            }
        }
    }
}

fn ser_contact_scalars(buf: &mut RecBuffer, c: &Contact) {
    buf.append_i32(c.set_index);
    buf.append_i32(c.color_index);
    buf.append_i32(c.local_index);
    for e in &c.edges {
        buf.append_i32(e.body_id);
        buf.append_i32(e.prev_key);
        buf.append_i32(e.next_key);
    }
    buf.append_i32(c.shape_id_a);
    buf.append_i32(c.shape_id_b);
    buf.append_i32(c.child_index);
    buf.append_i32(c.island_id);
    buf.append_i32(c.island_index);
    buf.append_i32(c.contact_id);
    buf.append_i32(NULL_INDEX); // body_sim_index_a scrubbed
    buf.append_i32(NULL_INDEX); // body_sim_index_b scrubbed
    buf.append_u32(c.flags);
    buf.append_quat(c.cached_rotation_a);
    buf.append_quat(c.cached_rotation_b);
    buf.append_transform(c.cached_relative_pose);
    buf.append_f32(c.friction);
    buf.append_f32(c.restitution);
    buf.append_f32(c.rolling_resistance);
    buf.append_vec3(c.tangent_velocity);
    // geometry tag + convex cache / mesh query bounds
    match &c.geometry {
        ContactGeometry::Convex(cv) => {
            buf.append_u8(0);
            ser_contact_cache(buf, &cv.cache);
        }
        ContactGeometry::Mesh(m) => {
            buf.append_u8(1);
            buf.append_aabb(m.query_bounds);
        }
    }
    buf.append_u32(c.generation);
}

pub fn des_contacts(r: &mut SnapReader<'_>) -> Vec<Contact> {
    let count = r.i32();
    if r.ok && !r.check_count(count, 64, 64) {
        r.ok = false;
    }
    if !r.ok {
        return Vec::new();
    }
    let mut out = Vec::with_capacity(count.max(0) as usize);
    for i in 0..count.max(0) {
        let mut c = des_contact_scalars(r);
        let is_live = c.contact_id == i;
        let manifold_count = r.i32();
        if !r.ok {
            break;
        }
        if is_live && manifold_count > 0 {
            let mut mans = Vec::with_capacity(manifold_count as usize);
            for _ in 0..manifold_count {
                mans.push(des_manifold(r));
            }
            c.manifolds = mans;
        } else {
            c.manifolds.clear();
        }
        if is_live && (c.flags & contact_flags::SIM_MESH_CONTACT) != 0 {
            let cache_count = r.i32();
            let mut triangle_cache = Vec::with_capacity(cache_count.max(0) as usize);
            for _ in 0..cache_count.max(0) {
                triangle_cache.push(TriangleCache {
                    triangle_index: r.i32(),
                    cache: des_contact_cache(r),
                });
            }
            let query_bounds = match &c.geometry {
                ContactGeometry::Mesh(m) => m.query_bounds,
                _ => Aabb::default(),
            };
            c.geometry = ContactGeometry::Mesh(MeshContact {
                triangle_cache,
                query_bounds,
            });
        }
        out.push(c);
    }
    out
}

fn des_contact_scalars(r: &mut SnapReader<'_>) -> Contact {
    let set_index = r.i32();
    let color_index = r.i32();
    let local_index = r.i32();
    let edges = [
        ContactEdge {
            body_id: r.i32(),
            prev_key: r.i32(),
            next_key: r.i32(),
        },
        ContactEdge {
            body_id: r.i32(),
            prev_key: r.i32(),
            next_key: r.i32(),
        },
    ];
    let shape_id_a = r.i32();
    let shape_id_b = r.i32();
    let child_index = r.i32();
    let island_id = r.i32();
    let island_index = r.i32();
    let contact_id = r.i32();
    let _bsa = r.i32();
    let _bsb = r.i32();
    let flags = r.u32();
    let cached_rotation_a = r.quat();
    let cached_rotation_b = r.quat();
    let cached_relative_pose = r.transform();
    let friction = r.f32();
    let restitution = r.f32();
    let rolling_resistance = r.f32();
    let tangent_velocity = r.vec3();
    let geometry = match r.u8() {
        0 => ContactGeometry::Convex(ConvexContact {
            cache: des_contact_cache(r),
        }),
        _ => ContactGeometry::Mesh(MeshContact {
            triangle_cache: Vec::new(),
            query_bounds: r.aabb(),
        }),
    };
    let generation = r.u32();
    Contact {
        set_index,
        color_index,
        local_index,
        edges,
        shape_id_a,
        shape_id_b,
        child_index,
        island_id,
        island_index,
        contact_id,
        body_sim_index_a: NULL_INDEX,
        body_sim_index_b: NULL_INDEX,
        flags,
        manifolds: Vec::new(),
        cached_rotation_a,
        cached_rotation_b,
        cached_relative_pose,
        friction,
        restitution,
        rolling_resistance,
        tangent_velocity,
        geometry,
        generation,
    }
}

pub fn ser_graph_color(
    buf: &mut RecBuffer,
    color: &crate::constraint_graph::GraphColor,
    is_overflow: bool,
) {
    if !is_overflow {
        ser_bit_set(buf, &color.body_set);
    }
    buf.append_i32(color.joint_sims.len() as i32);
    for j in &color.joint_sims {
        ser_joint_sim(buf, j);
    }
    ser_i32_array(buf, &color.convex_contacts);
    buf.append_i32(color.contacts.len() as i32);
    for c in &color.contacts {
        buf.append_i32(c.contact_id);
        buf.append_i32(c.manifold_start);
        buf.append_u16(c.manifold_count);
    }
}

pub fn des_graph_color(
    r: &mut SnapReader<'_>,
    is_overflow: bool,
) -> crate::constraint_graph::GraphColor {
    let body_set = if !is_overflow {
        des_bit_set(r)
    } else {
        BitSet::new(0)
    };
    let n = r.i32();
    let mut joint_sims = Vec::with_capacity(n.max(0) as usize);
    for _ in 0..n.max(0) {
        joint_sims.push(des_joint_sim(r));
    }
    let convex_contacts = des_i32_array(r);
    let n = r.i32();
    let mut contacts = Vec::with_capacity(n.max(0) as usize);
    for _ in 0..n.max(0) {
        contacts.push(ContactSpec {
            contact_id: r.i32(),
            manifold_start: r.i32(),
            manifold_count: r.u16(),
        });
    }
    crate::constraint_graph::GraphColor {
        body_set,
        joint_sims,
        convex_contacts,
        contacts,
    }
}
