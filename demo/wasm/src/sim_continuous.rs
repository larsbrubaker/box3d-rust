//! Continuous collision demos (Thin Wall, Bounce House, Bullet vs Stack).
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::sim_demo::{add_ground, new_sim, stop_recording_if_any, with_sim, SimBody, SIM};
use box3d_rust::body::{create_body, destroy_body, make_body_id};
use box3d_rust::geometry::{Capsule, Sphere};
use box3d_rust::hull::{make_box_hull, make_transformed_box_hull};
use box3d_rust::math_functions::{
    compute_quat_between_unit_vectors, make_quat_from_axis_angle, Pos, Transform, Vec3, DEG_TO_RAD,
    QUAT_IDENTITY, VEC3_AXIS_X, VEC3_AXIS_Y, VEC3_ZERO,
};
use box3d_rust::shape::{create_capsule_shape, create_hull_shape, create_sphere_shape};
use box3d_rust::types::{default_body_def, default_shape_def, BodyType};
use wasm_bindgen::prelude::*;

/// Continuous / Thin Wall â€” `sample_continuous.cpp` ThinWall.
#[wasm_bindgen]
pub fn sim_reset_thin_wall() -> u32 {
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();
        add_ground(&mut sim, 40.0);

        let mut body_def = default_body_def();
        let mut shape_def = default_shape_def();

        body_def.position = Pos {
            x: 0.0 as _,
            y: 10.0 as _,
            z: 0.0 as _,
        };
        body_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_X, 90.0 * DEG_TO_RAD);
        let wall = create_body(&mut sim.world, &body_def);
        let wall_hull = make_box_hull(10.0, 0.1, 10.0);
        create_hull_shape(&mut sim.world, wall, &shape_def, &wall_hull.base);
        sim.bodies.push(SimBody {
            body_index: wall.index1 - 1,
            half_extents: [10.0, 0.1, 10.0],
            kind: 0,
            local: None,
        });

        body_def.type_ = BodyType::Dynamic;
        body_def.rotation = QUAT_IDENTITY;
        shape_def.base_material.rolling_resistance = 0.1;

        body_def.position = Pos {
            x: (-5.0) as _,
            y: 10.0 as _,
            z: 20.0 as _,
        };
        body_def.linear_velocity = Vec3 {
            x: 0.0,
            y: 0.0,
            z: -180.0,
        };
        body_def.angular_velocity = Vec3 {
            x: 20.0,
            y: 0.0,
            z: 0.0,
        };
        let sphere_body = create_body(&mut sim.world, &body_def);
        let sphere = Sphere {
            center: VEC3_ZERO,
            radius: 0.1,
        };
        create_sphere_shape(&mut sim.world, sphere_body, &shape_def, &sphere);
        sim.bodies.push(SimBody {
            body_index: sphere_body.index1 - 1,
            half_extents: [0.1, 0.1, 0.1],
            kind: 1,
            local: None,
        });

        body_def.position = Pos {
            x: 0.0 as _,
            y: 10.0 as _,
            z: 20.0 as _,
        };
        body_def.linear_velocity = Vec3 {
            x: 0.0,
            y: 0.0,
            z: -180.0,
        };
        body_def.angular_velocity = Vec3 {
            x: 20.0,
            y: -5.0,
            z: 0.0,
        };
        let capsule_body = create_body(&mut sim.world, &body_def);
        let capsule = Capsule {
            center1: Vec3 {
                x: -0.3,
                y: 0.0,
                z: 0.0,
            },
            center2: Vec3 {
                x: 0.3,
                y: 0.0,
                z: 0.0,
            },
            radius: 0.1,
        };
        create_capsule_shape(&mut sim.world, capsule_body, &shape_def, &capsule);
        let capsule_viz = Transform {
            p: VEC3_ZERO,
            q: compute_quat_between_unit_vectors(VEC3_AXIS_Y, VEC3_AXIS_X),
        };
        sim.bodies.push(SimBody {
            body_index: capsule_body.index1 - 1,
            half_extents: [0.1, 0.3, 0.1],
            kind: 2,
            local: Some(capsule_viz),
        });

        body_def.position = Pos {
            x: 5.0 as _,
            y: 10.0 as _,
            z: 20.0 as _,
        };
        body_def.linear_velocity = Vec3 {
            x: 0.0,
            y: 0.0,
            z: -180.0,
        };
        body_def.angular_velocity = Vec3 {
            x: 20.0,
            y: 5.0,
            z: 0.0,
        };
        let box_body = create_body(&mut sim.world, &body_def);
        let box_proj = make_box_hull(0.4, 0.1, 0.1);
        create_hull_shape(&mut sim.world, box_body, &shape_def, &box_proj.base);
        sim.bodies.push(SimBody {
            body_index: box_body.index1 - 1,
            half_extents: [0.4, 0.1, 0.1],
            kind: 0,
            local: None,
        });

        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

/// Continuous / Bounce House â€” `sample_continuous.cpp` BounceHouse.
#[wasm_bindgen]
pub fn sim_reset_bounce_house() -> u32 {
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();
        add_ground(&mut sim, 10.0);

        let mut body_def = default_body_def();
        body_def.position = Pos {
            x: 0.0 as _,
            y: (-1.0) as _,
            z: 0.0 as _,
        };
        let walls_body = create_body(&mut sim.world, &body_def);
        let shape_def = default_shape_def();
        let parent_index = walls_body.index1 - 1;

        let wall_specs: [(Vec3, f32, f32, f32); 4] = [
            (
                Vec3 {
                    x: 10.0,
                    y: 5.0,
                    z: 0.0,
                },
                0.1,
                5.0,
                10.0,
            ),
            (
                Vec3 {
                    x: -10.0,
                    y: 5.0,
                    z: 0.0,
                },
                0.1,
                5.0,
                10.0,
            ),
            (
                Vec3 {
                    x: 0.0,
                    y: 5.0,
                    z: -10.0,
                },
                10.0,
                5.0,
                0.1,
            ),
            (
                Vec3 {
                    x: 0.0,
                    y: 5.0,
                    z: 10.0,
                },
                10.0,
                5.0,
                0.1,
            ),
        ];
        for (p, hx, hy, hz) in wall_specs {
            let transform = Transform {
                p,
                q: QUAT_IDENTITY,
            };
            let wall_box = make_transformed_box_hull(hx, hy, hz, transform);
            create_hull_shape(&mut sim.world, walls_body, &shape_def, &wall_box.base);
            sim.bodies.push(SimBody {
                body_index: parent_index,
                half_extents: [hx, hy, hz],
                kind: 0,
                local: Some(transform),
            });
        }

        body_def.type_ = BodyType::Dynamic;
        body_def.gravity_scale = 0.0;
        body_def.position = Pos {
            x: (-8.0) as _,
            y: 4.0 as _,
            z: 0.0 as _,
        };
        body_def.linear_velocity = Vec3 {
            x: 120.0,
            y: 0.0,
            z: 120.0,
        };
        let ball = create_body(&mut sim.world, &body_def);
        let mut ball_shape = default_shape_def();
        ball_shape.base_material.friction = 0.0;
        ball_shape.base_material.restitution = 1.0;
        let sphere = Sphere {
            center: VEC3_ZERO,
            radius: 0.5,
        };
        create_sphere_shape(&mut sim.world, ball, &ball_shape, &sphere);
        sim.bodies.push(SimBody {
            body_index: ball.index1 - 1,
            half_extents: [0.5, 0.5, 0.5],
            kind: 1,
            local: None,
        });

        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

/// Continuous / Bullet vs Stack â€” `sample_continuous.cpp` BulletVersusStack.
#[wasm_bindgen]
pub fn sim_reset_bullet_vs_stack() -> u32 {
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();
        add_ground(&mut sim, 50.0);

        {
            let mut body_def = default_body_def();
            body_def.position = Pos {
                x: 0.0 as _,
                y: (-1.0) as _,
                z: 0.0 as _,
            };
            let ground_body = create_body(&mut sim.world, &body_def);
            let shape_def = default_shape_def();
            let transform = Transform {
                p: Vec3 {
                    x: -1.0,
                    y: 5.0,
                    z: 0.0,
                },
                q: QUAT_IDENTITY,
            };
            let wall_box = make_transformed_box_hull(0.1, 5.0, 10.0, transform);
            create_hull_shape(&mut sim.world, ground_body, &shape_def, &wall_box.base);
            sim.bodies.push(SimBody {
                body_index: ground_body.index1 - 1,
                half_extents: [0.1, 5.0, 10.0],
                kind: 0,
                local: Some(transform),
            });
        }

        let box_hull = make_box_hull(0.5, 0.5, 0.5);
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        let shape_def = default_shape_def();
        for row in 0..10 {
            body_def.position = Pos {
                x: 0.0 as _,
                y: (0.5 + 1.1 * row as f32) as _,
                z: 0.0 as _,
            };
            let body_id = create_body(&mut sim.world, &body_def);
            create_hull_shape(&mut sim.world, body_id, &shape_def, &box_hull.base);
            sim.bodies.push(SimBody {
                body_index: body_id.index1 - 1,
                half_extents: [0.5, 0.5, 0.5],
                kind: 0,
                local: None,
            });
        }

        sim.bullet_body_index = -1;
        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

/// Launch / re-launch the Bullet vs Stack projectile (C `BulletVersusStack::Launch`).
#[wasm_bindgen]
pub fn sim_launch_bullet() -> u32 {
    with_sim(|sim| {
        sim.grab.end(&mut sim.world);
        if sim.bullet_body_index >= 0 {
            let index = sim.bullet_body_index;
            let id = make_body_id(&sim.world, index);
            destroy_body(&mut sim.world, id);
            sim.bodies.retain(|b| b.body_index != index);
            sim.bullet_body_index = -1;
        }

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.is_bullet = true;
        body_def.position = Pos {
            x: 20.5 as _,
            y: 5.5 as _,
            z: 0.0 as _,
        };
        body_def.linear_velocity = Vec3 {
            x: -500.0,
            y: 0.0,
            z: 0.0,
        };
        let bullet = create_body(&mut sim.world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density *= 10.0;
        let sphere = Sphere {
            center: VEC3_ZERO,
            radius: 0.25,
        };
        create_sphere_shape(&mut sim.world, bullet, &shape_def, &sphere);
        sim.bullet_body_index = bullet.index1 - 1;
        sim.bodies.push(SimBody {
            body_index: sim.bullet_body_index,
            half_extents: [0.25, 0.25, 0.25],
            kind: 1,
            local: None,
        });
        sim.bodies.len() as u32
    })
}
