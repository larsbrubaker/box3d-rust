//! Joint debug drawing (`b3DrawJoint` and per-type helpers).
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{
    get_joint_sim_ref, joint_get_constraint_force, joint_get_constraint_torque, Joint, JointSim,
    JointType,
};
use crate::constants::{huge, linear_slop, GRAPH_COLOR_COUNT};
use crate::core::NULL_INDEX;
use crate::debug_draw::{DebugDraw, HexColor};
use crate::id::JointId;
use crate::math_functions::{
    compute_cos_sin, cos, dot_quat, get_twist_angle, inv_mul_quat, length, lerp_float,
    lerp_position, make_matrix_from_quat, max_float, mul_sv, mul_world_transforms, negate_quat,
    normalize, offset_pos, rotate_vector, sin, sub_pos, transform_world_point, Vec3,
    WorldTransform, PI, VEC3_AXIS_X, VEC3_AXIS_Y, VEC3_AXIS_Z,
};
use crate::solver_set::DISABLED_SET;
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

/// (`b3DrawDistanceJoint`)
fn draw_distance_joint(
    draw: &mut dyn DebugDraw,
    base: &JointSim,
    transform_a: WorldTransform,
    transform_b: WorldTransform,
) {
    debug_assert!(base.type_ == JointType::Distance);
    let joint = base.distance();

    let p_a = transform_world_point(transform_a, base.local_frame_a.p);
    let p_b = transform_world_point(transform_b, base.local_frame_b.p);

    let axis = normalize(sub_pos(p_b, p_a));

    if joint.min_length < joint.max_length && joint.enable_limit {
        let p_min = offset_pos(p_a, mul_sv(joint.min_length, axis));
        let p_max = offset_pos(p_a, mul_sv(joint.max_length, axis));

        if joint.min_length > linear_slop() {
            draw.draw_point(p_min, 6.0, HexColor::LIGHT_GREEN);
        }

        if joint.max_length < huge() {
            draw.draw_point(p_max, 6.0, HexColor::RED);
        }

        if joint.min_length > linear_slop() && joint.max_length < huge() {
            draw.draw_segment(p_min, p_max, HexColor::GRAY);
        }
    }

    draw.draw_segment(p_a, p_b, HexColor::WHITE);
    draw.draw_point(p_a, 4.0, HexColor::WHITE);
    draw.draw_point(p_b, 4.0, HexColor::WHITE);

    if joint.hertz > 0.0 && joint.enable_spring {
        let p_rest = offset_pos(p_a, mul_sv(joint.length, axis));
        draw.draw_point(p_rest, 4.0, HexColor::BLUE);
    }
}

/// (`b3DrawParallelJoint`)
fn draw_parallel_joint(
    draw: &mut dyn DebugDraw,
    base: &JointSim,
    transform_a: WorldTransform,
    transform_b: WorldTransform,
    scale: f32,
) {
    let length1 = 0.1 * scale;

    let frame_a = mul_world_transforms(transform_a, base.local_frame_a);
    draw.draw_segment(
        frame_a.p,
        offset_pos(
            frame_a.p,
            mul_sv(length1, rotate_vector(frame_a.q, VEC3_AXIS_Z)),
        ),
        HexColor::GREEN,
    );

    let frame_b = mul_world_transforms(transform_b, base.local_frame_b);
    draw.draw_segment(
        frame_b.p,
        offset_pos(
            frame_b.p,
            mul_sv(length1, rotate_vector(frame_b.q, VEC3_AXIS_Z)),
        ),
        HexColor::BLUE,
    );
}

/// (`b3DrawPrismaticJoint`)
fn draw_prismatic_joint(
    draw: &mut dyn DebugDraw,
    base: &JointSim,
    transform_a: WorldTransform,
    transform_b: WorldTransform,
    scale: f32,
) {
    let frame_a = mul_world_transforms(transform_a, base.local_frame_a);
    let frame_b = mul_world_transforms(transform_b, base.local_frame_b);

    let r = make_matrix_from_quat(frame_a.q);
    let axis = r.cx;
    let perp_y = r.cy;
    let perp_z = r.cz;

    let s = 0.2 * scale;
    draw.draw_segment(
        frame_a.p,
        offset_pos(frame_a.p, mul_sv(s, perp_y)),
        HexColor::GREEN,
    );
    draw.draw_segment(
        frame_a.p,
        offset_pos(frame_a.p, mul_sv(s, perp_z)),
        HexColor::BLUE,
    );

    let joint = base.prismatic();
    if joint.enable_limit {
        let p1 = offset_pos(frame_a.p, mul_sv(joint.lower_translation, axis));
        let p2 = offset_pos(frame_a.p, mul_sv(joint.upper_translation, axis));
        draw.draw_segment(p1, p2, HexColor::ORANGE);
        draw.draw_point(p1, 10.0, HexColor::GREEN);
        draw.draw_point(p2, 10.0, HexColor::RED);
    } else {
        let p1 = offset_pos(frame_a.p, mul_sv(-0.5 * scale, axis));
        let p2 = offset_pos(frame_a.p, mul_sv(0.5 * scale, axis));
        draw.draw_segment(p1, p2, HexColor::ORANGE);
    }

    draw.draw_point(frame_b.p, 8.0, HexColor::VIOLET);
}

/// (`b3DrawRevoluteJoint`)
fn draw_revolute_joint(
    draw: &mut dyn DebugDraw,
    base: &JointSim,
    transform_a: WorldTransform,
    transform_b: WorldTransform,
    scale: f32,
) {
    let frame_a = mul_world_transforms(transform_a, base.local_frame_a);

    let length1 = 0.1 * scale;
    draw.draw_segment(
        frame_a.p,
        offset_pos(
            frame_a.p,
            mul_sv(length1, rotate_vector(frame_a.q, VEC3_AXIS_X)),
        ),
        HexColor::RED,
    );
    draw.draw_segment(
        frame_a.p,
        offset_pos(
            frame_a.p,
            mul_sv(length1, rotate_vector(frame_a.q, VEC3_AXIS_Y)),
        ),
        HexColor::GREEN,
    );
    draw.draw_segment(
        frame_a.p,
        offset_pos(
            frame_a.p,
            mul_sv(length1, rotate_vector(frame_a.q, VEC3_AXIS_Z)),
        ),
        HexColor::BLUE,
    );

    let frame_b = mul_world_transforms(transform_b, base.local_frame_b);
    let joint = base.revolute();
    const SLICE_COUNT: i32 = 16;

    if joint.enable_limit {
        let quat_a = frame_a.q;
        let mut quat_b = frame_b.q;

        if dot_quat(quat_a, quat_b) < 0.0 {
            quat_b = negate_quat(quat_b);
        }

        let rel_q = inv_mul_quat(quat_a, quat_b);
        let wedge_radius = 0.2 * scale;
        for index in 0..SLICE_COUNT {
            let t1 = index as f32 / SLICE_COUNT as f32;
            let alpha1 = (1.0 - t1) * joint.lower_angle + t1 * joint.upper_angle;
            let t2 = (index + 1) as f32 / SLICE_COUNT as f32;
            let alpha2 = (1.0 - t2) * joint.lower_angle + t2 * joint.upper_angle;

            let vertex1 = Vec3 {
                x: wedge_radius * cos(alpha1),
                y: wedge_radius * sin(alpha1),
                z: 0.0,
            };
            let vertex2 = Vec3 {
                x: wedge_radius * cos(alpha2),
                y: wedge_radius * sin(alpha2),
                z: 0.0,
            };

            if index == 0 {
                draw.draw_segment(
                    frame_a.p,
                    transform_world_point(frame_a, vertex1),
                    HexColor::CYAN,
                );
            }

            if index == SLICE_COUNT - 1 {
                draw.draw_segment(
                    transform_world_point(frame_a, vertex2),
                    frame_a.p,
                    HexColor::CYAN,
                );
            }
            draw.draw_segment(
                transform_world_point(frame_a, vertex1),
                transform_world_point(frame_a, vertex2),
                HexColor::CYAN,
            );
        }

        let twist_angle = get_twist_angle(rel_q);
        let p2 = Vec3 {
            x: wedge_radius * cos(twist_angle),
            y: wedge_radius * sin(twist_angle),
            z: 0.0,
        };
        draw.draw_segment(
            frame_a.p,
            transform_world_point(frame_a, p2),
            HexColor::YELLOW,
        );
    }
}

/// (`b3DrawSphericalJoint`)
fn draw_spherical_joint(
    draw: &mut dyn DebugDraw,
    base: &JointSim,
    transform_a: WorldTransform,
    transform_b: WorldTransform,
    scale: f32,
) {
    let frame_a = mul_world_transforms(transform_a, base.local_frame_a);

    let length1 = 0.1 * scale;
    draw.draw_segment(
        frame_a.p,
        offset_pos(
            frame_a.p,
            mul_sv(length1, rotate_vector(frame_a.q, VEC3_AXIS_X)),
        ),
        HexColor::RED,
    );
    draw.draw_segment(
        frame_a.p,
        offset_pos(
            frame_a.p,
            mul_sv(length1, rotate_vector(frame_a.q, VEC3_AXIS_Y)),
        ),
        HexColor::GREEN,
    );
    draw.draw_segment(
        frame_a.p,
        offset_pos(
            frame_a.p,
            mul_sv(length1, rotate_vector(frame_a.q, VEC3_AXIS_Z)),
        ),
        HexColor::BLUE,
    );

    let frame_b = mul_world_transforms(transform_b, base.local_frame_b);

    let length2 = 0.2 * scale;
    draw.draw_segment(
        frame_b.p,
        offset_pos(
            frame_b.p,
            mul_sv(length2, rotate_vector(frame_b.q, VEC3_AXIS_Z)),
        ),
        HexColor::ORANGE,
    );

    let joint = base.spherical();
    const SLICE_COUNT: i32 = 16;

    if joint.enable_twist_limit {
        let quat_a = frame_a.q;
        let mut quat_b = frame_b.q;

        if dot_quat(quat_a, quat_b) < 0.0 {
            quat_b = negate_quat(quat_b);
        }

        let rel_q = inv_mul_quat(quat_a, quat_b);
        let wedge_radius = 0.1 * scale;
        for index in 0..SLICE_COUNT {
            let t1 = index as f32 / SLICE_COUNT as f32;
            let alpha1 = (1.0 - t1) * joint.lower_twist_angle + t1 * joint.upper_twist_angle;
            let t2 = (index + 1) as f32 / SLICE_COUNT as f32;
            let alpha2 = (1.0 - t2) * joint.lower_twist_angle + t2 * joint.upper_twist_angle;

            let vertex1 = Vec3 {
                x: wedge_radius * cos(alpha1),
                y: wedge_radius * sin(alpha1),
                z: 0.0,
            };
            let vertex2 = Vec3 {
                x: wedge_radius * cos(alpha2),
                y: wedge_radius * sin(alpha2),
                z: 0.0,
            };

            if index == 0 {
                draw.draw_segment(
                    frame_a.p,
                    transform_world_point(frame_a, vertex1),
                    HexColor::CYAN,
                );
            }

            if index == SLICE_COUNT - 1 {
                draw.draw_segment(
                    transform_world_point(frame_a, vertex2),
                    frame_a.p,
                    HexColor::CYAN,
                );
            }
            draw.draw_segment(
                transform_world_point(frame_a, vertex1),
                transform_world_point(frame_a, vertex2),
                HexColor::CYAN,
            );
        }

        let twist_angle = get_twist_angle(rel_q);
        let p2 = Vec3 {
            x: wedge_radius * cos(twist_angle),
            y: wedge_radius * sin(twist_angle),
            z: 0.0,
        };
        draw.draw_segment(
            frame_a.p,
            transform_world_point(frame_a, p2),
            HexColor::YELLOW,
        );
    }

    if joint.enable_cone_limit {
        let radius = 0.1 * scale;
        let cone_radius = radius * sin(joint.cone_angle);
        let cone_height = radius * cos(joint.cone_angle);

        for index in 0..SLICE_COUNT {
            let phi1 = 2.0 * (index as f32) / SLICE_COUNT as f32 * PI;
            let phi2 = 2.0 * ((index + 1) as f32) / SLICE_COUNT as f32 * PI;

            let vertex1 = Vec3 {
                x: cone_radius * cos(phi1),
                y: cone_radius * sin(phi1),
                z: cone_height,
            };
            let vertex2 = Vec3 {
                x: cone_radius * cos(phi2),
                y: cone_radius * sin(phi2),
                z: cone_height,
            };

            draw.draw_segment(
                frame_a.p,
                transform_world_point(frame_a, vertex1),
                HexColor::CYAN,
            );
            draw.draw_segment(
                transform_world_point(frame_a, vertex1),
                transform_world_point(frame_a, vertex2),
                HexColor::CYAN,
            );
        }
    }
}

/// (`b3DrawWeldJoint`)
fn draw_weld_joint(
    draw: &mut dyn DebugDraw,
    base: &JointSim,
    transform_a: WorldTransform,
    transform_b: WorldTransform,
    scale: f32,
) {
    let frame_a = mul_world_transforms(transform_a, base.local_frame_a);
    let frame_b = mul_world_transforms(transform_b, base.local_frame_b);

    let extents = Vec3 {
        x: 0.1 * scale,
        y: 0.05 * scale,
        z: 0.025 * scale,
    };
    draw.draw_box(extents, frame_a, HexColor::DARK_ORANGE);
    draw.draw_box(extents, frame_b, HexColor::DARK_CYAN);
}

/// (`b3DrawWheelJoint`)
fn draw_wheel_joint(
    draw: &mut dyn DebugDraw,
    base: &JointSim,
    transform_a: WorldTransform,
    transform_b: WorldTransform,
    scale: f32,
) {
    debug_assert!(base.type_ == JointType::Wheel);
    let joint = base.wheel();

    let frame_a = mul_world_transforms(transform_a, base.local_frame_a);
    let frame_b = mul_world_transforms(transform_b, base.local_frame_b);

    let matrix_a = make_matrix_from_quat(frame_a.q);
    let matrix_b = make_matrix_from_quat(frame_b.q);

    draw.draw_segment(frame_a.p, frame_b.p, HexColor::BLUE);

    if joint.enable_suspension_limit {
        let lower = offset_pos(frame_a.p, mul_sv(joint.lower_suspension_limit, matrix_a.cx));
        let upper = offset_pos(frame_a.p, mul_sv(joint.upper_suspension_limit, matrix_a.cx));
        let perp = matrix_a.cy;
        draw.draw_segment(lower, upper, HexColor::GRAY);
        draw.draw_segment(
            offset_pos(lower, mul_sv(-0.1 * scale, perp)),
            offset_pos(lower, mul_sv(0.1 * scale, perp)),
            HexColor::GREEN,
        );
        draw.draw_segment(
            offset_pos(upper, mul_sv(-0.1 * scale, perp)),
            offset_pos(upper, mul_sv(0.1 * scale, perp)),
            HexColor::RED,
        );
    } else {
        draw.draw_segment(
            offset_pos(frame_a.p, mul_sv(-scale, matrix_a.cx)),
            offset_pos(frame_a.p, mul_sv(1.0 * scale, matrix_a.cx)),
            HexColor::GRAY,
        );
    }

    if joint.enable_steering && joint.enable_steering_limit {
        let frame = WorldTransform {
            p: frame_b.p,
            q: frame_a.q,
        };

        let radius = 0.5 * scale;
        let slice_count = 16;
        let lower = joint.lower_steering_limit;
        let upper = joint.upper_steering_limit;

        let cs = compute_cos_sin(lower);
        let mut vertex1 = transform_world_point(
            frame,
            Vec3 {
                x: 0.0,
                y: -radius * cs.sine,
                z: radius * cs.cosine,
            },
        );

        for index in 0..slice_count {
            let t2 = (index as f32 + 1.0) / slice_count as f32;
            let phi = lerp_float(lower, upper, t2);

            let cs = compute_cos_sin(phi);
            let vertex2 = transform_world_point(
                frame,
                Vec3 {
                    x: 0.0,
                    y: -radius * cs.sine,
                    z: radius * cs.cosine,
                },
            );

            if index == 0 {
                draw.draw_segment(frame.p, vertex1, HexColor::CYAN);
            }

            if index == slice_count - 1 {
                draw.draw_segment(vertex2, frame.p, HexColor::CYAN);
            }
            draw.draw_segment(vertex1, vertex2, HexColor::CYAN);

            vertex1 = vertex2;
        }
    }

    draw.draw_segment(
        offset_pos(frame_b.p, mul_sv(-0.5 * scale, matrix_b.cz)),
        offset_pos(frame_b.p, mul_sv(0.5 * scale, matrix_b.cz)),
        HexColor::MAGENTA,
    );

    draw.draw_point(frame_a.p, 5.0, HexColor::GRAY);
    draw.draw_point(frame_b.p, 5.0, HexColor::DIM_GRAY);
}

fn joint_full_id(world: &World, joint: &Joint) -> JointId {
    JointId {
        index1: joint.joint_id + 1,
        world0: world.world_id,
        generation: joint.generation,
    }
}

/// (`b3DrawJoint`)
pub fn draw_joint(draw: &mut dyn DebugDraw, world: &mut World, joint: &Joint) {
    let body_a = &world.bodies[joint.edges[0].body_id as usize];
    let body_b = &world.bodies[joint.edges[1].body_id as usize];
    if body_a.set_index == DISABLED_SET || body_b.set_index == DISABLED_SET {
        return;
    }

    let joint_sim = *get_joint_sim_ref(world, joint.joint_id);

    let transform_a = crate::body::get_body_transform_quick(world, body_a);
    let transform_b = crate::body::get_body_transform_quick(world, body_b);
    let p_a = transform_world_point(transform_a, joint_sim.local_frame_a.p);
    let p_b = transform_world_point(transform_b, joint_sim.local_frame_b.p);

    let scale = max_float(0.0001, draw.joint_scale() * joint.draw_scale);

    match joint.type_ {
        JointType::Parallel => {
            draw_parallel_joint(draw, &joint_sim, transform_a, transform_b, scale);
        }
        JointType::Distance => {
            draw_distance_joint(draw, &joint_sim, transform_a, transform_b);
        }
        JointType::Filter => {
            draw.draw_segment(p_a, p_b, HexColor::GOLD);
        }
        JointType::Motor => {
            draw.draw_segment(p_a, p_b, HexColor::PLUM);
            draw.draw_point(p_a, 8.0, HexColor::YELLOW_GREEN);
            draw.draw_point(p_b, 8.0, HexColor::PLUM);
        }
        JointType::Prismatic => {
            draw_prismatic_joint(draw, &joint_sim, transform_a, transform_b, scale);
        }
        JointType::Revolute => {
            draw_revolute_joint(draw, &joint_sim, transform_a, transform_b, scale);
        }
        JointType::Spherical => {
            draw_spherical_joint(draw, &joint_sim, transform_a, transform_b, scale);
        }
        JointType::Weld => {
            draw_weld_joint(draw, &joint_sim, transform_a, transform_b, scale);
        }
        JointType::Wheel => {
            draw_wheel_joint(draw, &joint_sim, transform_a, transform_b, scale);
        }
    }

    if draw.draw_graph_colors() {
        let color_index = joint.color_index;
        if color_index != NULL_INDEX {
            let p = lerp_position(p_a, p_b, 0.5);
            draw.draw_point(p, 5.0, GRAPH_COLORS[color_index as usize]);
        }
    }

    if draw.draw_joint_extras() {
        let jid = joint_full_id(world, joint);
        let force = joint_get_constraint_force(world, jid);
        let torque = joint_get_constraint_torque(world, jid);
        let p = lerp_position(p_a, p_b, 0.5);

        draw.draw_segment(p, offset_pos(p, mul_sv(0.001, force)), HexColor::AZURE);

        let buffer = format!("f = {}, t = {}", length(force), length(torque));
        draw.draw_string(p, &buffer, HexColor::AZURE);
    }
}
