//! Wind force application on shapes from shape.c (`b3Shape_ApplyWind`).
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::lifecycle::get_shape;
use super::ShapeGeometry;
use crate::body::wake_body_with_lock;
use crate::core::get_length_units_per_meter;
use crate::geometry::ShapeType;
use crate::hull::{get_hull_edges, get_hull_faces, get_hull_planes, get_hull_points};
use crate::id::ShapeId;
use crate::math_functions::{
    add, cross, dot, get_length_and_normalize, length, make_matrix_from_quat, min_float, mul_add,
    mul_mv, mul_sub, mul_sv, normalize, rotate_vector, sub, to_relative_transform, Vec3, PI,
    POS_ZERO, VEC3_ZERO,
};
use crate::solver_set::{AWAKE_SET, DISABLED_SET, FIRST_SLEEPING_SET};
use crate::types::BodyType;
use crate::world::World;

// https://en.wikipedia.org/wiki/Density_of_air
// https://www.engineeringtoolbox.com/wind-load-d_1775.html
// force = 0.5 * air_density * velocity^2 * area
// https://en.wikipedia.org/wiki/Lift_(force)
/// Apply a wind force to the body for this shape using the density of air.
/// (b3Shape_ApplyWind)
pub fn shape_apply_wind(
    world: &mut World,
    shape_id: ShapeId,
    wind: Vec3,
    drag: f32,
    lift: f32,
    max_speed: f32,
    wake: bool,
) {
    crate::recording::with_recording(world, |rec| {
        rec.write_shape_apply_wind(shape_id, wind, drag, lift, max_speed, wake);
    });
    let shape_index = get_shape(world, shape_id);

    let shape_type = world.shapes[shape_index as usize].shape_type();
    if shape_type != ShapeType::Sphere
        && shape_type != ShapeType::Capsule
        && shape_type != ShapeType::Hull
    {
        return;
    }

    let body_id = world.shapes[shape_index as usize].body_id;

    if world.bodies[body_id as usize].type_ != BodyType::Dynamic {
        return;
    }

    if world.bodies[body_id as usize].set_index == DISABLED_SET {
        return;
    }

    if world.bodies[body_id as usize].set_index >= FIRST_SLEEPING_SET && !wake {
        return;
    }

    if world.bodies[body_id as usize].set_index != AWAKE_SET {
        // Must wake for state to exist
        wake_body_with_lock(world, body_id);
    }

    debug_assert!(world.bodies[body_id as usize].set_index == AWAKE_SET);

    let local_index = world.bodies[body_id as usize].local_index;
    let (transform, local_center_of_mass) = {
        let sim = &world.solver_sets[AWAKE_SET as usize].body_sims[local_index as usize];
        // Only the rotation is used below, so the demoted world transform is exact
        (
            to_relative_transform(sim.transform, POS_ZERO),
            sim.local_center,
        )
    };
    let (linear_velocity, angular_velocity) = {
        let state = &world.solver_sets[AWAKE_SET as usize].body_states[local_index as usize];
        (state.linear_velocity, state.angular_velocity)
    };

    let length_units = get_length_units_per_meter();
    let volume_units = length_units * length_units * length_units;
    let air_density = 1.2250 / volume_units;

    let mut force = VEC3_ZERO;
    let mut torque = VEC3_ZERO;

    let shape = &world.shapes[shape_index as usize];
    match &shape.geometry {
        ShapeGeometry::Sphere(sphere) => {
            let radius = sphere.radius;
            let centroid = shape.local_centroid;
            let lever = rotate_vector(transform.q, sub(centroid, local_center_of_mass));
            let shape_velocity = add(linear_velocity, cross(angular_velocity, lever));
            let relative_velocity = mul_sub(wind, drag, shape_velocity);
            let mut speed = 0.0;
            let direction = get_length_and_normalize(&mut speed, relative_velocity);
            let speed = min_float(speed, max_speed);
            let projected_area = PI * radius * radius;
            force = mul_sv(
                0.5 * air_density * projected_area * speed * speed,
                direction,
            );
            torque = cross(lever, force);
        }

        ShapeGeometry::Capsule(capsule) => {
            let centroid = shape.local_centroid;
            let lever = rotate_vector(transform.q, sub(centroid, local_center_of_mass));
            let shape_velocity = add(linear_velocity, cross(angular_velocity, lever));
            let relative_velocity = mul_sub(wind, drag, shape_velocity);
            let mut speed = 0.0;
            let direction = get_length_and_normalize(&mut speed, relative_velocity);
            let speed = min_float(speed, max_speed);

            let mut d = sub(capsule.center2, capsule.center1);
            d = rotate_vector(transform.q, d);

            let radius = capsule.radius;
            let projected_area = PI * radius * radius + 2.0 * radius * length(cross(d, direction));

            // Normal that opposes the wind
            let e = normalize(d);
            let normal = sub(mul_sv(dot(direction, e), e), direction);

            // portion of wind that is perpendicular to surface
            let lift_direction = cross(cross(normal, direction), direction);

            let force_magnitude = 0.5 * air_density * projected_area * speed * speed;
            force = mul_sv(force_magnitude, mul_add(direction, lift, lift_direction));

            let edge_lever = mul_add(lever, radius, normal);
            torque = cross(edge_lever, force);
        }

        ShapeGeometry::Hull(hull) => {
            let matrix = make_matrix_from_quat(transform.q);

            let face_count = hull.face_count;
            let points = get_hull_points(hull);
            let faces = get_hull_faces(hull);
            let edges = get_hull_edges(hull);
            let planes = get_hull_planes(hull);

            for i in 0..face_count {
                let face = &faces[i as usize];
                let edge1_index = face.edge;
                let edge2_index = edges[edge1_index as usize].next;
                let mut edge3_index = edges[edge2_index as usize].next;

                debug_assert!(edge1_index != edge3_index);
                debug_assert!((edges[edge1_index as usize].origin as i32) < hull.vertex_count);
                debug_assert!((edges[edge2_index as usize].origin as i32) < hull.vertex_count);

                let local_point1 = points[edges[edge1_index as usize].origin as usize];
                let mut local_point2 = points[edges[edge2_index as usize].origin as usize];
                let v1 = mul_mv(matrix, local_point1);
                let mut v2 = mul_mv(matrix, local_point2);
                let normal = mul_mv(matrix, planes[i as usize].normal);

                loop {
                    debug_assert!((edges[edge3_index as usize].origin as i32) < hull.vertex_count);
                    let local_point3 = points[edges[edge3_index as usize].origin as usize];
                    let v3 = mul_mv(matrix, local_point3);

                    // Triangle center
                    let triangle_local_center =
                        mul_sv(0.333333, add(local_point1, add(local_point2, local_point3)));

                    // Lever arm from center of mass to triangle center in world space
                    let lever = mul_mv(matrix, sub(triangle_local_center, local_center_of_mass));

                    // Velocity of the triangle center in world space
                    let center_velocity = add(linear_velocity, cross(angular_velocity, lever));

                    let relative_velocity = mul_sub(wind, drag, center_velocity);
                    let mut speed = 0.0;
                    let direction = get_length_and_normalize(&mut speed, relative_velocity);

                    // Check for back-side
                    if dot(normal, direction) < -f32::EPSILON {
                        let projected_area = -0.5 * dot(cross(sub(v2, v1), sub(v3, v1)), direction);
                        debug_assert!(projected_area >= -f32::EPSILON);

                        let lift_direction = cross(cross(normal, direction), direction);

                        let speed = min_float(speed, max_speed);

                        let force_magnitude = 0.5 * air_density * projected_area * speed * speed;
                        let delta_force =
                            mul_sv(force_magnitude, mul_add(direction, lift, lift_direction));
                        let delta_torque = cross(lever, delta_force);

                        force = add(force, delta_force);
                        torque = add(torque, delta_torque);
                    }

                    // Advance the fan: C sets edge2 = edge3 then edge3 = next(edge3).
                    edge3_index = edges[edge3_index as usize].next;
                    v2 = v3;
                    local_point2 = local_point3;

                    if edge1_index == edge3_index {
                        break;
                    }
                }
            }
        }

        _ => {}
    }

    let sim = &mut world.solver_sets[AWAKE_SET as usize].body_sims[local_index as usize];
    sim.force = add(sim.force, force);
    sim.torque = add(sim.torque, torque);
}
