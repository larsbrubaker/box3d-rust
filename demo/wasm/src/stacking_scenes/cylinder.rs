//! Convex-hull Stacking scenes: Cylinder (sample_stacking.cpp:315), Cylinder
//! Stack (:366), Wedge (:641), and Arch (:734). Shared body builders live in the
//! parent module.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{install, pos, push_box_rot, push_hull};
use crate::sim_demo::add_ground;
use box3d_rust::hull::{clone_and_transform_hull, create_cylinder, create_hull};
use box3d_rust::math_functions::{Vec3, QUAT_IDENTITY, TRANSFORM_IDENTITY};
use wasm_bindgen::prelude::*;

// --------------------------------------------------------------------------
// Cylinder (sample_stacking.cpp:315)
// --------------------------------------------------------------------------

/// Cylinder (sample_stacking.cpp:315). Single 12-sided cylinder hull, rolling
/// resistance 0.05. C also sets `GetGuiDraw()->forceScale = 0.01` — the page maps
/// that onto the debug force-draw scale.
#[wasm_bindgen]
pub fn sim_reset_cylinder() -> u32 {
    install(|sim| {
        add_ground(sim, 10.0);
        let hull = create_cylinder(1.0, 0.25, 0.0, 12).expect("cylinder hull");
        // rollingResistance 0.05, otherwise default material (sample_stacking.cpp:339).
        push_hull(
            sim,
            pos(0.0, 2.0, 0.0),
            QUAT_IDENTITY,
            &hull,
            1000.0,
            0.6,
            0.05,
        );
    })
}

// --------------------------------------------------------------------------
// Cylinder Stack (sample_stacking.cpp:366)
// --------------------------------------------------------------------------

/// Cylinder Stack (sample_stacking.cpp:366). 10 15-sided cylinders with the four
/// per-instance scales cycled by `i % 4`. C sets `forceScale = 0.001`.
///
/// C builds one base cylinder hull and installs each body via
/// `b3CreateTransformedHullShape( bodyId, &shapeDef, m_hull, b3Transform_identity,
/// scales[i % 4] )` (sample_stacking.cpp:398). Internally that bakes the identity
/// transform + non-uniform scale into fresh hull data with `b3CloneAndTransformHull`
/// (shape.c:142), so the port clones the base hull the same way per instance.
#[wasm_bindgen]
pub fn sim_reset_cylinder_stack() -> u32 {
    install(|sim| {
        add_ground(sim, 10.0);
        let hull = create_cylinder(1.0, 0.5, 0.0, 15).expect("cylinder hull");
        let scales = [
            Vec3 {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
            Vec3 {
                x: -0.75,
                y: 1.0,
                z: 1.0,
            },
            Vec3 {
                x: 1.2,
                y: 1.0,
                z: -0.9,
            },
            Vec3 {
                x: 0.9,
                y: 0.9,
                z: 0.9,
            },
        ];
        for i in 0..10 {
            let baked =
                clone_and_transform_hull(&hull, TRANSFORM_IDENTITY, scales[(i % 4) as usize])
                    .expect("transformed cylinder hull");
            push_hull(
                sim,
                pos(0.0, 0.0 + 1.1 * i as f32, 0.0),
                QUAT_IDENTITY,
                &baked,
                1000.0,
                0.6,
                0.0,
            );
        }
    })
}

// --------------------------------------------------------------------------
// Wedge (sample_stacking.cpp:641)
// --------------------------------------------------------------------------

/// Wedge (sample_stacking.cpp:641). A custom 6-vertex hull that can produce an
/// incorrect manifold if narrow-phase is mishandled.
#[wasm_bindgen]
pub fn sim_reset_wedge() -> u32 {
    install(|sim| {
        add_ground(sim, 20.0);
        let vertices = [
            Vec3 {
                x: -1.0,
                y: 1.0,
                z: -0.1,
            },
            Vec3 {
                x: 1.0,
                y: 1.0,
                z: -0.1,
            },
            Vec3 {
                x: -1.0,
                y: 1.0,
                z: 0.1,
            },
            Vec3 {
                x: 1.0,
                y: 1.0,
                z: 0.1,
            },
            Vec3 {
                x: -0.5,
                y: 0.5,
                z: 0.0,
            },
            Vec3 {
                x: 0.5,
                y: 0.5,
                z: 0.0,
            },
        ];
        let hull = create_hull(&vertices, 6).expect("wedge hull");
        push_hull(
            sim,
            pos(0.0, 1.0, 0.0),
            QUAT_IDENTITY,
            &hull,
            1000.0,
            0.6,
            0.0,
        );
    })
}

// --------------------------------------------------------------------------
// Arch (sample_stacking.cpp:734)
// --------------------------------------------------------------------------

/// Arch (sample_stacking.cpp:734). Two symmetric columns of voussoir hulls plus a
/// keystone, topped with four boxes; density 200 throughout. Each voussoir body
/// sits at the origin with its hull carrying the world-space arch coordinates.
#[wasm_bindgen]
pub fn sim_reset_arch() -> u32 {
    install(|sim| {
        add_ground(sim, 40.0);

        let mut ps1 = [
            Vec3 {
                x: 16.0,
                y: 0.0,
                z: 0.0,
            },
            Vec3 {
                x: 14.93803712795643,
                y: 5.133601056842984,
                z: 0.0,
            },
            Vec3 {
                x: 13.79871746027416,
                y: 10.24928069555078,
                z: 0.0,
            },
            Vec3 {
                x: 12.56252963284711,
                y: 15.34107019122473,
                z: 0.0,
            },
            Vec3 {
                x: 11.20040987372525,
                y: 20.39856541571217,
                z: 0.0,
            },
            Vec3 {
                x: 9.66521217819836,
                y: 25.40369899225096,
                z: 0.0,
            },
            Vec3 {
                x: 7.87179930638133,
                y: 30.3179337000085,
                z: 0.0,
            },
            Vec3 {
                x: 5.635199558196225,
                y: 35.03820717801641,
                z: 0.0,
            },
            Vec3 {
                x: 2.405937953536585,
                y: 39.09554102558315,
                z: 0.0,
            },
        ];
        let mut ps2 = [
            Vec3 {
                x: 24.0,
                y: 0.0,
                z: 0.0,
            },
            Vec3 {
                x: 22.33619528222415,
                y: 6.02299846205841,
                z: 0.0,
            },
            Vec3 {
                x: 20.54936888969905,
                y: 12.00964361211476,
                z: 0.0,
            },
            Vec3 {
                x: 18.60854610798073,
                y: 17.9470321677465,
                z: 0.0,
            },
            Vec3 {
                x: 16.46769273811807,
                y: 23.81367936585418,
                z: 0.0,
            },
            Vec3 {
                x: 14.05325025774858,
                y: 29.57079353071012,
                z: 0.0,
            },
            Vec3 {
                x: 11.23551045834022,
                y: 35.13775818285372,
                z: 0.0,
            },
            Vec3 {
                x: 7.752568160730571,
                y: 40.30450679009583,
                z: 0.0,
            },
            Vec3 {
                x: 3.016931552701656,
                y: 44.28891593799322,
                z: 0.0,
            },
        ];

        let scale = 0.25f32;
        for i in 0..9 {
            ps1[i].x *= scale;
            ps1[i].y *= scale;
            ps2[i].x *= scale;
            ps2[i].y *= scale;
        }

        let half_depth = 0.5f32;
        // density 200 (sample_stacking.cpp:779).
        for i in 0..8 {
            let ps = [
                Vec3 {
                    x: ps1[i].x,
                    y: ps1[i].y,
                    z: -half_depth,
                },
                Vec3 {
                    x: ps2[i].x,
                    y: ps2[i].y,
                    z: -half_depth,
                },
                Vec3 {
                    x: ps2[i + 1].x,
                    y: ps2[i + 1].y,
                    z: -half_depth,
                },
                Vec3 {
                    x: ps1[i + 1].x,
                    y: ps1[i + 1].y,
                    z: -half_depth,
                },
                Vec3 {
                    x: ps1[i].x,
                    y: ps1[i].y,
                    z: half_depth,
                },
                Vec3 {
                    x: ps2[i].x,
                    y: ps2[i].y,
                    z: half_depth,
                },
                Vec3 {
                    x: ps2[i + 1].x,
                    y: ps2[i + 1].y,
                    z: half_depth,
                },
                Vec3 {
                    x: ps1[i + 1].x,
                    y: ps1[i + 1].y,
                    z: half_depth,
                },
            ];
            let hull = create_hull(&ps, 8).expect("arch voussoir hull");
            push_hull(
                sim,
                pos(0.0, 0.0, 0.0),
                QUAT_IDENTITY,
                &hull,
                200.0,
                0.6,
                0.0,
            );
        }

        for i in 0..8 {
            let ps = [
                Vec3 {
                    x: -ps2[i].x,
                    y: ps2[i].y,
                    z: -half_depth,
                },
                Vec3 {
                    x: -ps1[i].x,
                    y: ps1[i].y,
                    z: -half_depth,
                },
                Vec3 {
                    x: -ps1[i + 1].x,
                    y: ps1[i + 1].y,
                    z: -half_depth,
                },
                Vec3 {
                    x: -ps2[i + 1].x,
                    y: ps2[i + 1].y,
                    z: -half_depth,
                },
                Vec3 {
                    x: -ps2[i].x,
                    y: ps2[i].y,
                    z: half_depth,
                },
                Vec3 {
                    x: -ps1[i].x,
                    y: ps1[i].y,
                    z: half_depth,
                },
                Vec3 {
                    x: -ps1[i + 1].x,
                    y: ps1[i + 1].y,
                    z: half_depth,
                },
                Vec3 {
                    x: -ps2[i + 1].x,
                    y: ps2[i + 1].y,
                    z: half_depth,
                },
            ];
            let hull = create_hull(&ps, 8).expect("arch voussoir hull (mirror)");
            push_hull(
                sim,
                pos(0.0, 0.0, 0.0),
                QUAT_IDENTITY,
                &hull,
                200.0,
                0.6,
                0.0,
            );
        }

        {
            let ps = [
                Vec3 {
                    x: ps1[8].x,
                    y: ps1[8].y,
                    z: -half_depth,
                },
                Vec3 {
                    x: ps2[8].x,
                    y: ps2[8].y,
                    z: -half_depth,
                },
                Vec3 {
                    x: -ps2[8].x,
                    y: ps2[8].y,
                    z: -half_depth,
                },
                Vec3 {
                    x: -ps1[8].x,
                    y: ps1[8].y,
                    z: -half_depth,
                },
                Vec3 {
                    x: ps1[8].x,
                    y: ps1[8].y,
                    z: half_depth,
                },
                Vec3 {
                    x: ps2[8].x,
                    y: ps2[8].y,
                    z: half_depth,
                },
                Vec3 {
                    x: -ps2[8].x,
                    y: ps2[8].y,
                    z: half_depth,
                },
                Vec3 {
                    x: -ps1[8].x,
                    y: ps1[8].y,
                    z: half_depth,
                },
            ];
            let hull = create_hull(&ps, 8).expect("arch keystone hull");
            push_hull(
                sim,
                pos(0.0, 0.0, 0.0),
                QUAT_IDENTITY,
                &hull,
                200.0,
                0.6,
                0.0,
            );
        }

        for i in 0..4 {
            let y = 0.5 + ps2[8].y + 1.0 * i as f32;
            push_box_rot(
                sim,
                pos(0.0, y, 0.0),
                QUAT_IDENTITY,
                2.0,
                0.5,
                half_depth,
                200.0,
                0.6,
                0.0,
            );
        }
    })
}
