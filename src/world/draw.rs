//! Debug draw traversal for the world (`b3World_Draw` in physics_world.c).
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::body::body_flags;
use crate::body::{get_body_sim, get_body_state_index};
use crate::constants::{speculative_distance, GRAPH_COLOR_COUNT, MIN_FRICTION_WEIGHT};
use crate::constraint_graph::OVERFLOW_INDEX;
use crate::core::{get_length_units_per_meter, NULL_INDEX};
use crate::debug_draw::{make_debug_color, DebugDraw, DebugMaterial, DebugShape, HexColor};
use crate::id::ShapeId;
use crate::joint::draw_joint;
use crate::math_functions::{
    aabb_union, clamp_float, clamp_int, is_valid_aabb, length, mul_add, mul_sv, offset_pos,
    transform_world_point, Aabb, Vec3, WorldTransform,
};
use crate::name_cache::NameCache;
use crate::solver_set::{AWAKE_SET, DISABLED_SET};
use crate::types::{BodyType, BODY_TYPE_COUNT};
use crate::world::World;

const GRAPH_COLORS: [HexColor; GRAPH_COLOR_COUNT as usize] = [
    HexColor::RED,
    HexColor::ORANGE,
    HexColor::YELLOW,
    HexColor::GREEN,
    HexColor::CYAN,
    HexColor::BLUE,
    HexColor::VIOLET,
    HexColor::PINK,
    HexColor::CHOCOLATE,
    HexColor::GOLDEN_ROD,
    HexColor::CORAL,
    HexColor::ROSY_BROWN,
    HexColor::AQUA,
    HexColor::PERU,
    HexColor::LIME,
    HexColor::GOLD,
    HexColor::PLUM,
    HexColor::SNOW,
    HexColor::TEAL,
    HexColor::KHAKI,
    HexColor::SALMON,
    HexColor::PEACH_PUFF,
    HexColor::HONEY_DEW,
    HexColor::BLACK,
];

fn body_name(names: &NameCache, name_id: u32) -> &str {
    if name_id == crate::constants::NULL_NAME {
        ""
    } else {
        names.find_name(name_id).unwrap_or("")
    }
}

/// Per-shape body of the C `DrawQueryCallback`: colors the shape by body state
/// and marks its body for the second pass.
fn draw_query_callback(world: &mut World, draw: &mut dyn DebugDraw, shape_id: i32) {
    debug_assert!(world.shapes[shape_id as usize].id == shape_id);
    let body_id = world.shapes[shape_id as usize].body_id;
    world.debug_body_set.set_bit(body_id as u32);

    if draw.draw_shapes() {
        let color = shape_debug_color(world, shape_id);

        let create = world.create_debug_shape;
        let ctx = world.user_debug_shape_context;
        let world0 = world.world_id;

        if world.shapes[shape_id as usize].user_shape == 0 {
            if let Some(create) = create {
                let created = {
                    let shape = &world.shapes[shape_id as usize];
                    let debug_shape = DebugShape {
                        shape_id: ShapeId {
                            index1: shape_id + 1,
                            world0,
                            generation: shape.generation,
                        },
                        shape_type: shape.shape_type(),
                        geometry: &shape.geometry,
                    };
                    create(&debug_shape, ctx)
                };
                world.shapes[shape_id as usize].user_shape = created;
            }
        }

        let user_shape = world.shapes[shape_id as usize].user_shape;
        if user_shape != 0 {
            let transform = get_body_sim(world, body_id).transform;
            draw.draw_shape(user_shape, transform, color);
        }
    }

    if draw.draw_bounds_boxes() {
        let fat_aabb = world.shapes[shape_id as usize].fat_aabb;
        draw.draw_bounds(fat_aabb, HexColor::GOLD);
    }
}

fn shape_debug_color(world: &World, shape_id: i32) -> HexColor {
    let shape = &world.shapes[shape_id as usize];
    let body = &world.bodies[shape.body_id as usize];
    let body_sim = get_body_sim(world, shape.body_id);

    let materials = shape.shape_materials();
    if materials[0].custom_color != 0 {
        return HexColor(materials[0].custom_color);
    }

    let mut material = DebugMaterial::Default;
    let rgb = if body.type_ == BodyType::Dynamic && body.mass == 0.0 {
        HexColor::RED
    } else if body.set_index == DISABLED_SET {
        HexColor::SLATE_GRAY
    } else if shape.sensor_index != NULL_INDEX {
        HexColor::WHEAT
    } else if body.flags & body_flags::HAD_TIME_OF_IMPACT != 0 {
        HexColor::LIME
    } else if (body_sim.flags & body_flags::IS_BULLET != 0) && body.set_index == AWAKE_SET {
        HexColor::TURQUOISE
    } else if body.flags & body_flags::IS_SPEED_CAPPED != 0 {
        HexColor::YELLOW
    } else if body_sim.flags & body_flags::IS_FAST != 0 {
        material = DebugMaterial::Glossy;
        HexColor::ORANGE
    } else if body.type_ == BodyType::Static {
        material = DebugMaterial::Matte;
        HexColor::DARK_GRAY
    } else if body.type_ == BodyType::Kinematic {
        if body.set_index == AWAKE_SET {
            material = DebugMaterial::Metallic;
            HexColor::STEEL_BLUE
        } else {
            material = DebugMaterial::Matte;
            HexColor::LIGHT_STEEL_BLUE
        }
    } else if body.set_index == AWAKE_SET {
        material = DebugMaterial::Soft;
        HexColor::TAN
    } else {
        material = DebugMaterial::Dead;
        HexColor::LIGHT_SLATE_GRAY
    };

    make_debug_color(rgb, material)
}

fn draw_body_joints(world: &mut World, draw: &mut dyn DebugDraw, body_id: i32) {
    let mut joint_key = world.bodies[body_id as usize].head_joint_key;
    while joint_key != NULL_INDEX {
        let joint_id = joint_key >> 1;
        let edge_index = joint_key & 1;

        if !world.debug_joint_set.get_bit(joint_id as u32) {
            // Copy joint so draw_joint can mutably borrow world.
            let joint = world.joints[joint_id as usize].clone();
            draw_joint(draw, world, &joint);
            world.debug_joint_set.set_bit(joint_id as u32);
        }

        joint_key = world.joints[joint_id as usize].edges[edge_index as usize].next_key;
    }
}

fn draw_body_contacts(
    world: &mut World,
    draw: &mut dyn DebugDraw,
    body_id: i32,
    axis_scale: f32,
    length_scale: f32,
    far_color: HexColor,
    speculative_color: HexColor,
    add_color: HexColor,
    persist_color: HexColor,
    impulse_color: HexColor,
    friction_color: HexColor,
) {
    let speculative_distance = speculative_distance();

    let mut contact_key = world.bodies[body_id as usize].head_contact_key;
    while contact_key != NULL_INDEX {
        let contact_id = contact_key >> 1;
        let edge_index = contact_key & 1;
        let contact = &world.contacts[contact_id as usize];
        contact_key = contact.edges[edge_index as usize].next_key;

        if contact.set_index != AWAKE_SET || contact.color_index == NULL_INDEX {
            continue;
        }

        if world.debug_contact_set.get_bit(contact_id as u32) {
            continue;
        }

        let body_a_id = contact.edges[0].body_id;
        let body_b_id = contact.edges[1].body_id;
        let center_a = get_body_sim(world, body_a_id).center;
        let center_b = get_body_sim(world, body_b_id).center;
        let color_index = contact.color_index;
        let manifold_count = contact.manifold_count() as usize;

        for manifold_index in 0..manifold_count {
            let manifold = world.contacts[contact_id as usize].manifolds[manifold_index];
            debug_assert!(manifold.point_count > 0);

            let normal = manifold.normal;
            // Average the anchors not the world points so the friction center stays exact far from the origin
            let contact_center = if draw.draw_anchor_a() {
                center_a
            } else {
                center_b
            };
            let mut friction_anchor = Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            };
            let mut total_weight = 0.0;
            let inv_tau = 1.0 / speculative_distance;

            for point_index in 0..manifold.point_count as usize {
                let mp = &manifold.points[point_index];
                let anchor = if draw.draw_anchor_a() {
                    mp.anchor_a
                } else {
                    mp.anchor_b
                };
                let p = offset_pos(contact_center, anchor);

                // See similar friction anchor weights in b3PrepareContacts_Mesh.
                let weight = clamp_float(2.0 - mp.separation * inv_tau, MIN_FRICTION_WEIGHT, 1.0);
                friction_anchor = mul_add(friction_anchor, weight, anchor);
                total_weight += weight;

                if draw.draw_contact_normals() {
                    let p1 = p;
                    let p2 = offset_pos(p1, mul_sv(axis_scale, normal));
                    draw.draw_segment(p1, p2, HexColor::LIGHT_GRAY);
                    let buffer = format!("   {:.2}", 100.0 * mp.separation);
                    draw.draw_string(p, &buffer, HexColor::WHITE);
                } else if draw.draw_contact_forces() {
                    let inv_dt = if world.inv_dt > 0.0 {
                        world.inv_dt
                    } else {
                        60.0
                    };
                    let force = 0.5 * mp.total_normal_impulse * inv_dt;
                    let p1 = p;
                    let p2 = offset_pos(p1, mul_sv(draw.force_scale() * force, normal));
                    draw.draw_segment(p1, p2, impulse_color);
                    let buffer = format!("  {:.1}", force);
                    draw.draw_string(p1, &buffer, HexColor::WHITE);
                } else if draw.draw_contact_features() {
                    let buffer = format!("   {:#X}", mp.feature_id);
                    draw.draw_string(p, &buffer, HexColor::WHITE);
                }

                if draw.draw_graph_colors() {
                    let point_size = if color_index == OVERFLOW_INDEX {
                        15.0
                    } else {
                        10.0
                    };
                    draw.draw_point(p, point_size, GRAPH_COLORS[color_index as usize]);
                } else if !mp.persisted {
                    draw.draw_point(p, 20.0, add_color);
                } else if mp.separation > speculative_distance {
                    draw.draw_point(p, 10.0, far_color);
                } else if mp.separation > 0.0 {
                    draw.draw_point(p, 10.0, speculative_color);
                } else {
                    draw.draw_point(p, 10.0, persist_color);
                }
            }

            if draw.draw_contact_forces() {
                let inv_dt = if world.inv_dt > 0.0 {
                    world.inv_dt
                } else {
                    60.0
                };
                friction_anchor = mul_sv(1.0 / total_weight, friction_anchor);
                let mut p1 = offset_pos(contact_center, friction_anchor);
                let friction_force = mul_sv(0.5 * inv_dt, manifold.friction_impulse);
                let p2 = offset_pos(p1, mul_sv(draw.force_scale(), friction_force));
                draw.draw_segment(p1, p2, friction_color);
                draw.draw_point(p1, 5.0, friction_color);

                p1 = offset_pos(p1, mul_sv(0.05 * length_scale, normal));
                let buffer = format!("{:.2}", length(friction_force));
                draw.draw_string(p1, &buffer, HexColor::WHITE);
            }
        }

        world.debug_contact_set.set_bit(contact_id as u32);
    }
}

fn draw_body_island(world: &mut World, draw: &mut dyn DebugDraw, body_id: i32) {
    let island_id = world.bodies[body_id as usize].island_id;
    if island_id == NULL_INDEX || world.debug_island_set.get_bit(island_id as u32) {
        return;
    }

    let island = &world.islands[island_id as usize];
    // A valid islandId never has a null set index; skip without infinite-looping
    // (C `continue`s past the body-bit clear).
    if island.set_index == NULL_INDEX {
        return;
    }

    let mut shape_count = 0;
    let mut aabb = Aabb {
        lower_bound: Vec3 {
            x: f32::MAX,
            y: f32::MAX,
            z: f32::MAX,
        },
        upper_bound: Vec3 {
            x: -f32::MAX,
            y: -f32::MAX,
            z: -f32::MAX,
        },
    };

    let body_ids: Vec<i32> = island.bodies.clone();
    for island_body_id in body_ids {
        let mut shape_id = world.bodies[island_body_id as usize].head_shape_id;
        while shape_id != NULL_INDEX {
            let shape = &world.shapes[shape_id as usize];
            aabb = aabb_union(aabb, shape.fat_aabb);
            shape_count += 1;
            shape_id = shape.next_shape_id;
        }
    }

    if shape_count > 0 {
        draw.draw_bounds(aabb, HexColor::ORANGE_RED);
    }

    world.debug_island_set.set_bit(island_id as u32);
}

/// (`b3World_Draw`)
pub fn world_draw(world: &mut World, draw: &mut dyn DebugDraw, mask_bits: u64) {
    debug_assert!(!world.locked);
    if world.locked {
        return;
    }

    debug_assert!(is_valid_aabb(draw.drawing_bounds()));

    let length_scale = get_length_units_per_meter();
    let axis_scale = 0.3 * length_scale;
    let far_color = HexColor::DARK_BLUE;
    let speculative_color = HexColor::BLUE;
    let add_color = HexColor::LIME_GREEN;
    let persist_color = HexColor::LIGHT_BLUE;
    let impulse_color = HexColor::MAGENTA;
    let friction_color = HexColor::YELLOW;

    let body_capacity = world.body_id_pool.id_capacity();
    world
        .debug_body_set
        .set_bit_count_and_clear(body_capacity as u32);

    let joint_capacity = world.joint_id_pool.id_capacity();
    world
        .debug_joint_set
        .set_bit_count_and_clear(joint_capacity as u32);

    let contact_capacity = world.contact_id_pool.id_capacity();
    world
        .debug_contact_set
        .set_bit_count_and_clear(contact_capacity as u32);

    let island_capacity = world.island_id_pool.id_capacity();
    world
        .debug_island_set
        .set_bit_count_and_clear(island_capacity as u32);

    let bounds = draw.drawing_bounds();
    for i in 0..BODY_TYPE_COUNT {
        let mut shape_ids = Vec::new();
        world.broad_phase.trees[i].query(bounds, mask_bits, false, |_, user_data| {
            shape_ids.push(user_data as i32);
            true
        });

        for shape_id in shape_ids {
            // Mark body bit here (also done inside) — DrawQueryCallback sets it first.
            draw_query_callback(world, draw, shape_id);
        }
    }

    let word_count = world.debug_body_set.block_count() as usize;
    let words: Vec<u64> = (0..word_count)
        .map(|k| world.debug_body_set.block(k as u32))
        .collect();

    for (word_index, word) in words.into_iter().enumerate() {
        let mut word = word;
        while word != 0 {
            let ctz = word.trailing_zeros();
            let body_id = (64 * word_index as u32 + ctz) as i32;

            let body = &world.bodies[body_id as usize];
            let body_sim = get_body_sim(world, body_id);

            if draw.draw_body_names() {
                let name = body_name(&world.names, body.name_id);
                if !name.is_empty() {
                    let offset = Vec3 {
                        x: 0.05,
                        y: 0.05,
                        z: 0.05,
                    };
                    let transform = WorldTransform {
                        p: body_sim.center,
                        q: body_sim.transform.q,
                    };
                    let p = transform_world_point(transform, offset);
                    let name = name.to_string();
                    draw.draw_string(p, &name, HexColor::WHITE);
                }
            }

            if draw.draw_mass() {
                let body_sim = get_body_sim(world, body_id);
                let transform = WorldTransform {
                    p: body_sim.center,
                    q: body_sim.transform.q,
                };
                draw.draw_transform(transform);

                if world.bodies[body_id as usize].type_ == BodyType::Dynamic {
                    let offset = Vec3 {
                        x: 0.05,
                        y: 0.05,
                        z: 0.05,
                    };
                    let p = transform_world_point(transform, offset);
                    let mass = world.bodies[body_id as usize].mass;
                    let buffer = format!("{:.2}", mass);
                    draw.draw_string(p, &buffer, HexColor::WHITE);
                }
            }

            if draw.draw_sleep() {
                if let Some(local_index) = get_body_state_index(world, body_id) {
                    let _ = local_index;
                    let colors = [
                        HexColor::BLUE,
                        HexColor::SKY_BLUE,
                        HexColor::ORANGE,
                        HexColor::RED,
                    ];

                    let sleep_threshold = world.bodies[body_id as usize].sleep_threshold;
                    let sleep_velocity = world.bodies[body_id as usize].sleep_velocity;
                    let color = if sleep_threshold > 0.0 {
                        let ratio = sleep_velocity / sleep_threshold;
                        let index = clamp_int(ratio as i32, 0, 3);
                        colors[index as usize]
                    } else {
                        HexColor::BLACK
                    };

                    let center = get_body_sim(world, body_id).center;
                    draw.draw_point(center, 10.0, color);

                    let offset = Vec3 {
                        x: 0.1,
                        y: 0.1,
                        z: 0.1,
                    };
                    let p = offset_pos(center, offset);
                    let buffer = format!("  {:.3}", sleep_velocity);
                    draw.draw_string(p, &buffer, color);
                }
            }

            if draw.draw_joints() {
                draw_body_joints(world, draw, body_id);
            }

            if draw.draw_contacts()
                && world.bodies[body_id as usize].type_ == BodyType::Dynamic
                && world.bodies[body_id as usize].set_index == AWAKE_SET
            {
                draw_body_contacts(
                    world,
                    draw,
                    body_id,
                    axis_scale,
                    length_scale,
                    far_color,
                    speculative_color,
                    add_color,
                    persist_color,
                    impulse_color,
                    friction_color,
                );
            }

            if draw.draw_islands() {
                draw_body_island(world, draw, body_id);
            }

            word &= word - 1;
        }
    }

    let length_units = get_length_units_per_meter();
    let offset = Vec3 {
        x: 0.002 * length_units,
        y: 0.002 * length_units,
        z: 0.002 * length_units,
    };
    for i in 0..world.worker_count as usize {
        let point_count = world.task_contexts[i]
            .point_count
            .min(world.task_contexts[i].points.len());
        for j in 0..point_count {
            let point = world.task_contexts[i].points[j];
            draw.draw_point(point.p, 5.0, point.color);
            let buffer = format!("   {}, {:.2}", point.label, point.value);
            let ps = offset_pos(point.p, offset);
            draw.draw_string(ps, &buffer, point.color);
        }

        let line_count = world.task_contexts[i]
            .line_count
            .min(world.task_contexts[i].lines.len());
        for j in 0..line_count {
            let line = world.task_contexts[i].lines[j];
            draw.draw_segment(line.p1, line.p2, line.color);
            draw.draw_point(line.p1, 10.0, line.color);
            let buffer = format!("{}", line.label);
            let ps = offset_pos(line.p1, offset);
            draw.draw_string(ps, &buffer, line.color);
        }
    }
}
