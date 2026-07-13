//! Port of box3d-cpp-reference/test/test_mover.c.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::geometry::{
    collide_mover_and_capsule, collide_mover_and_hull, collide_mover_and_sphere, Capsule,
    CollisionPlane, PlaneResult, Sphere,
};
use crate::hull::make_box_hull;
use crate::math_functions::{dot, is_normalized, Plane, Vec3, VEC3_ZERO};
use crate::mover::solve_planes;

fn ensure_small(value: f32, tolerance: f32) {
    assert!(
        !(value < -tolerance || tolerance < value),
        "|{value}| > tolerance {tolerance}"
    );
}

#[test]
fn parallel_planes() {
    let mut planes = [CollisionPlane::default(); 3];
    planes[0].plane = Plane {
        normal: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        },
        offset: 0.5,
    };
    planes[0].push_limit = f32::MAX;
    planes[1].plane = Plane {
        normal: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        },
        offset: 1.0,
    };
    planes[1].push_limit = f32::MAX;

    let target = VEC3_ZERO;
    let result = solve_planes(target, &mut planes[..2]);

    assert_eq!(result.iteration_count, 2);
    ensure_small(result.delta.z - 1.0, 0.0055);
}

#[test]
fn game_planes() {
    // This scenario takes many iterations because the target is deep into the plane.
    let mut planes = [CollisionPlane::default(); 3];
    planes[0].plane = Plane {
        normal: Vec3 {
            x: 0.0,
            y: -0.23941046,
            z: 0.970918416,
        },
        offset: 0.390724182,
    };
    planes[0].push_limit = f32::MAX;
    planes[1].plane = Plane {
        normal: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        },
        offset: 1.49998093,
    };
    planes[1].push_limit = f32::MAX;

    let mut target = Vec3 {
        x: -2.5390625,
        y: 0.0,
        z: -73.6880798,
    };

    planes[0].plane.offset -= dot(planes[0].plane.normal, target);
    planes[1].plane.offset -= dot(planes[1].plane.normal, target);
    target = VEC3_ZERO;

    let result = solve_planes(target, &mut planes[..2]);

    assert_eq!(result.iteration_count, 20);
}

// ---------------------------------------------------------------------------
// Mover-collide overlap handling
//
// collide_mover_and_sphere / capsule / hull must never emit a plane with a
// degenerate (zero) normal, even when the mover deeply penetrates the shape.
// ---------------------------------------------------------------------------

#[test]
fn mover_sphere_separated() {
    let shape = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    let mover = Capsule {
        center1: Vec3 {
            x: 4.0,
            y: 3.0,
            z: 0.0,
        },
        center2: Vec3 {
            x: 6.0,
            y: 3.0,
            z: 0.0,
        },
        radius: 0.2,
    };

    let mut result = PlaneResult::default();
    let count = collide_mover_and_sphere(&mut result, &shape, &mover);
    assert_eq!(count, 0);
}

#[test]
fn mover_sphere_touching() {
    let shape = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };

    // Mover core segment runs along X at y = 0.6, leaving it 0.1 inside the
    // 0.7 combined radius.
    let mover = Capsule {
        center1: Vec3 {
            x: -1.0,
            y: 0.6,
            z: 0.0,
        },
        center2: Vec3 {
            x: 1.0,
            y: 0.6,
            z: 0.0,
        },
        radius: 0.2,
    };

    let mut result = PlaneResult::default();
    let count = collide_mover_and_sphere(&mut result, &shape, &mover);
    assert_eq!(count, 1);
    assert!(is_normalized(result.plane.normal));

    // Push-out points from the sphere straight up toward the mover.
    assert!(result.plane.normal.y > 0.99);
    ensure_small(result.plane.offset - 0.1, 1e-5);
}

#[test]
fn mover_sphere_deep_overlap() {
    let shape = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };

    // Mover axis runs straight through the sphere center: the bug case where
    // GJK reports a zero normal.
    let mover = Capsule {
        center1: Vec3 {
            x: -1.0,
            y: 0.0,
            z: 0.0,
        },
        center2: Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        radius: 0.2,
    };

    let mut result = PlaneResult::default();
    let count = collide_mover_and_sphere(&mut result, &shape, &mover);
    assert_eq!(count, 1);

    // The normal must still be a valid unit vector.
    assert!(is_normalized(result.plane.normal));

    // The fallback axis is perpendicular to the mover axis (X).
    ensure_small(result.plane.normal.x, 1e-5);

    // Deepest possible penetration: the full combined radius.
    ensure_small(result.plane.offset - 0.7, 1e-5);
}

#[test]
fn mover_capsule_separated() {
    let shape = Capsule {
        center1: Vec3 {
            x: -1.0,
            y: 0.0,
            z: 0.0,
        },
        center2: Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        radius: 0.3,
    };
    let mover = Capsule {
        center1: Vec3 {
            x: -1.0,
            y: 5.0,
            z: 0.0,
        },
        center2: Vec3 {
            x: 1.0,
            y: 5.0,
            z: 0.0,
        },
        radius: 0.2,
    };

    let mut result = PlaneResult::default();
    let count = collide_mover_and_capsule(&mut result, &shape, &mover);
    assert_eq!(count, 0);
}

#[test]
fn mover_capsule_touching() {
    let shape = Capsule {
        center1: Vec3 {
            x: -1.0,
            y: 0.0,
            z: 0.0,
        },
        center2: Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        radius: 0.3,
    };

    // Parallel mover 0.4 above, leaving it 0.1 inside the 0.5 combined radius.
    let mover = Capsule {
        center1: Vec3 {
            x: -1.0,
            y: 0.4,
            z: 0.0,
        },
        center2: Vec3 {
            x: 1.0,
            y: 0.4,
            z: 0.0,
        },
        radius: 0.2,
    };

    let mut result = PlaneResult::default();
    let count = collide_mover_and_capsule(&mut result, &shape, &mover);
    assert_eq!(count, 1);
    assert!(is_normalized(result.plane.normal));
    assert!(result.plane.normal.y > 0.99);
    ensure_small(result.plane.offset - 0.1, 1e-5);
}

#[test]
fn mover_capsule_deep_overlap() {
    // Shape capsule along X, mover capsule along Z; their core segments cross
    // exactly at the origin, so GJK reports a zero normal.
    let shape = Capsule {
        center1: Vec3 {
            x: -1.0,
            y: 0.0,
            z: 0.0,
        },
        center2: Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        radius: 0.3,
    };
    let mover = Capsule {
        center1: Vec3 {
            x: 0.0,
            y: 0.0,
            z: -1.0,
        },
        center2: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        },
        radius: 0.2,
    };

    let mut result = PlaneResult::default();
    let count = collide_mover_and_capsule(&mut result, &shape, &mover);
    assert_eq!(count, 1);
    assert!(is_normalized(result.plane.normal));

    // The separating axis of two crossing segments is perpendicular to both.
    ensure_small(result.plane.normal.x, 1e-5);
    ensure_small(result.plane.normal.z, 1e-5);
    ensure_small(result.plane.offset - 0.5, 1e-5);
}

#[test]
fn mover_capsule_parallel_overlap() {
    // Mover core segment coincides with the shape core segment: the cross-product
    // axis degenerates, so a perpendicular of the mover axis is used instead.
    let shape = Capsule {
        center1: Vec3 {
            x: -1.0,
            y: 0.0,
            z: 0.0,
        },
        center2: Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        radius: 0.3,
    };
    let mover = Capsule {
        center1: Vec3 {
            x: -1.0,
            y: 0.0,
            z: 0.0,
        },
        center2: Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        radius: 0.2,
    };

    let mut result = PlaneResult::default();
    let count = collide_mover_and_capsule(&mut result, &shape, &mover);
    assert_eq!(count, 1);
    assert!(is_normalized(result.plane.normal));

    // The fallback axis is perpendicular to the mover axis (X).
    ensure_small(result.plane.normal.x, 1e-5);
    ensure_small(result.plane.offset - 0.5, 1e-5);
}

#[test]
fn mover_hull_separated() {
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let mover = Capsule {
        center1: Vec3 {
            x: -0.3,
            y: 5.0,
            z: 0.0,
        },
        center2: Vec3 {
            x: 0.3,
            y: 5.0,
            z: 0.0,
        },
        radius: 0.2,
    };

    let mut result = PlaneResult::default();
    let count = collide_mover_and_hull(&mut result, &box_hull.base, &mover);
    assert_eq!(count, 0);
}

#[test]
fn mover_hull_touching() {
    let box_hull = make_box_hull(0.5, 0.5, 0.5);

    // Mover core segment above the +Y face; the 0.2 radius reaches 0.1 into it.
    let mover = Capsule {
        center1: Vec3 {
            x: -0.3,
            y: 0.6,
            z: 0.0,
        },
        center2: Vec3 {
            x: 0.3,
            y: 0.6,
            z: 0.0,
        },
        radius: 0.2,
    };

    let mut result = PlaneResult::default();
    let count = collide_mover_and_hull(&mut result, &box_hull.base, &mover);
    assert_eq!(count, 1);
    assert!(is_normalized(result.plane.normal));
    assert!(result.plane.normal.y > 0.99);
    ensure_small(result.plane.offset - 0.1, 1e-4);
}

#[test]
fn mover_hull_deep_overlap() {
    let box_hull = make_box_hull(0.5, 0.5, 0.5);

    // Mover core segment lies entirely inside the box, so GJK reports overlap.
    let mover = Capsule {
        center1: Vec3 {
            x: -0.2,
            y: 0.0,
            z: 0.0,
        },
        center2: Vec3 {
            x: 0.2,
            y: 0.0,
            z: 0.0,
        },
        radius: 0.1,
    };

    let mut result = PlaneResult::default();
    let count = collide_mover_and_hull(&mut result, &box_hull.base, &mover);

    // The overlap guard drops the plane rather than emit a zero normal.
    // todo replace with SAT once collide_mover_and_hull resolves overlaps.
    assert_eq!(count, 0);
}

// ---------------------------------------------------------------------------
// Mover integration loop
//
// Drive the full character-mover query/solve/cast sequence against a live world
// (world_collide_mover -> solve_planes -> clip_vector -> world_cast_mover, the
// same path sample_character's BasicMover controller runs each frame) across a
// wave height field with obstacles, and assert every collision-plane normal is
// finite + unit length and every produced position/velocity stays finite for
// many steps, including airborne launch and landing. Guards the reported
// character-demo `computeBoundingSphere: NaN`. Runs under both the default and
// `double-precision` builds. (The demo-side controller equivalent lives in
// `demo/wasm/src/character_demo_tests.rs`.)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod integration {
    use crate::body::create_body;
    use crate::geometry::{Capsule, CollisionPlane, Sphere};
    use crate::height_field::create_wave;
    use crate::hull::make_box_hull;
    use crate::math_functions::{
        length_squared, mul_sv, offset_pos, sub_pos, Pos, Vec3, VEC3_ZERO,
    };
    use crate::mover::{clip_vector, solve_planes};
    use crate::shape::{
        create_capsule_shape, create_height_field_shape, create_hull_shape, create_sphere_shape,
    };
    use crate::types::{
        default_body_def, default_query_filter, default_shape_def, default_world_def, BodyType,
    };
    use crate::world::{world_cast_mover, world_collide_mover, World};

    const PLANE_CAPACITY: usize = 8;
    const MOVER_GRAVITY: f32 = 15.0;

    fn mover_capsule() -> Capsule {
        Capsule {
            center1: Vec3 {
                x: 0.0,
                y: -0.5,
                z: 0.0,
            },
            center2: Vec3 {
                x: 0.0,
                y: 0.5,
                z: 0.0,
            },
            radius: 0.3,
        }
    }

    /// A wave height field with a few static boxes/capsules and a dynamic sphere,
    /// mirroring the BasicMover scene the character demo builds. Returns the world
    /// and the mover start position.
    fn build_scene() -> (World, Pos) {
        let mut def = default_world_def();
        def.gravity = Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        };
        let mut world = World::new(&def);

        let rows = 21;
        let cols = 21;
        let scale = Vec3 {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        };
        let hf = create_wave(rows, cols, scale, 0.02, 0.04, true);
        let hf_origin = Vec3 {
            x: -0.5 * scale.x * (cols - 1) as f32,
            y: 0.0,
            z: -0.5 * scale.z * (rows - 1) as f32,
        };
        let mut ground_def = default_body_def();
        ground_def.type_ = BodyType::Static;
        ground_def.position = Pos {
            x: hf_origin.x as _,
            y: hf_origin.y as _,
            z: hf_origin.z as _,
        };
        let ground = create_body(&mut world, &ground_def);
        create_height_field_shape(&mut world, ground, &default_shape_def(), &hf);

        for (bx, by, bz, hx, hy, hz) in [
            (2.0f32, 1.0, 3.0, 1.0, 1.0, 1.0),
            (-3.0, 0.8, 2.0, 1.2, 0.8, 0.6),
            (2.0, 0.5, -5.0, 2.0, 0.5, 0.5),
        ] {
            let mut body_def = default_body_def();
            body_def.type_ = BodyType::Static;
            body_def.position = Pos {
                x: bx as _,
                y: by as _,
                z: bz as _,
            };
            let body = create_body(&mut world, &body_def);
            let hull = make_box_hull(hx, hy, hz);
            create_hull_shape(&mut world, body, &default_shape_def(), &hull.base);
        }

        for (cx, cz) in [(0.0f32, 6.0f32), (0.0, 5.0)] {
            let mut body_def = default_body_def();
            body_def.type_ = BodyType::Static;
            body_def.position = Pos {
                x: cx as _,
                y: 1.4 as _,
                z: cz as _,
            };
            let body = create_body(&mut world, &body_def);
            create_capsule_shape(&mut world, body, &default_shape_def(), &mover_capsule());
        }

        {
            let mut body_def = default_body_def();
            body_def.type_ = BodyType::Dynamic;
            body_def.position = Pos {
                x: 5.0 as _,
                y: 5.0 as _,
                z: 0.0 as _,
            };
            let body = create_body(&mut world, &body_def);
            let sphere = Sphere {
                center: VEC3_ZERO,
                radius: 0.5,
            };
            create_sphere_shape(&mut world, body, &default_shape_def(), &sphere);
        }

        (
            world,
            Pos {
                x: 5.0 as _,
                y: 0.75 as _,
                z: 7.0 as _,
            },
        )
    }

    /// Assert a collected plane set carries only finite, unit-length normals —
    /// the invariant a degenerate mover contact would violate before poisoning
    /// the solve/clip with NaN.
    fn assert_planes_ok(planes: &[CollisionPlane], step: i32) {
        for p in planes {
            let n = p.plane.normal;
            assert!(
                n.x.is_finite() && n.y.is_finite() && n.z.is_finite(),
                "non-finite plane normal {n:?} at step {step}"
            );
            let mag2 = n.x * n.x + n.y * n.y + n.z * n.z;
            assert!(
                (mag2 - 1.0).abs() < 1e-3,
                "non-unit mover plane normal |n|^2={mag2} at step {step}"
            );
        }
    }

    /// Collect the mover's contact planes at `origin` into `planes`, returning the
    /// count. Shared by the iterate and final-clip phases.
    fn collect_planes(
        world: &World,
        origin: Pos,
        capsule: &Capsule,
        planes: &mut [CollisionPlane; PLANE_CAPACITY],
    ) -> usize {
        let filter = default_query_filter();
        let mut plane_count = 0usize;
        world_collide_mover(world, origin, capsule, &filter, |_id, results| {
            for r in results {
                if plane_count >= PLANE_CAPACITY {
                    break;
                }
                planes[plane_count] = CollisionPlane {
                    plane: r.plane,
                    push_limit: f32::MAX,
                    push: 0.0,
                    clip_velocity: true,
                };
                plane_count += 1;
            }
            true
        });
        plane_count
    }

    #[test]
    fn mover_solve_loop_stays_finite() {
        let dt = 1.0 / 60.0;
        let (mut world, mut mover_pos) = build_scene();
        let capsule = mover_capsule();
        let filter = default_query_filter();
        let mut velocity = VEC3_ZERO;

        for step in 0..600 {
            // Rotating horizontal drive + a periodic upward launch so the airborne
            // and landing transitions are exercised.
            let angle = step as f32 * 0.07;
            velocity = Vec3 {
                x: 5.0 * angle.cos(),
                y: velocity.y - MOVER_GRAVITY * dt,
                z: 5.0 * angle.sin(),
            };
            if step % 70 == 0 {
                velocity.y = 5.0;
            }

            let target = offset_pos(mover_pos, mul_sv(dt, velocity));

            // collide -> solve -> cast, iterated, exactly like the controller.
            for _ in 0..5 {
                let mut planes = [CollisionPlane::default(); PLANE_CAPACITY];
                let plane_count = collect_planes(&world, mover_pos, &capsule, &mut planes);
                assert_planes_ok(&planes[..plane_count], step);

                let target_delta = sub_pos(target, mover_pos);
                let result = solve_planes(target_delta, &mut planes[..plane_count]);
                let mut delta = result.delta;
                assert!(
                    delta.x.is_finite() && delta.y.is_finite() && delta.z.is_finite(),
                    "solve_planes produced non-finite delta {delta:?} at step {step}"
                );

                let fraction = world_cast_mover(&world, mover_pos, &capsule, delta, &filter, None);
                assert!(
                    fraction.is_finite(),
                    "world_cast_mover returned non-finite fraction {fraction} at step {step}"
                );
                delta *= fraction;
                mover_pos = offset_pos(mover_pos, delta);

                if length_squared(delta) < 0.01 * 0.01 {
                    break;
                }
            }

            // Final clip against the contact planes, like the controller.
            let mut planes = [CollisionPlane::default(); PLANE_CAPACITY];
            let plane_count = collect_planes(&world, mover_pos, &capsule, &mut planes);
            assert_planes_ok(&planes[..plane_count], step);
            let _ = solve_planes(VEC3_ZERO, &mut planes[..plane_count]);
            velocity = clip_vector(velocity, &planes[..plane_count]);

            world.step(dt, 4);

            // Everything the demo would hand the renderer must be finite. Check
            // the position (Scalar: f32 or f64 under double-precision) and the
            // velocity (always f32) fields directly to stay cast-free in both
            // build configurations.
            assert!(
                mover_pos.x.is_finite() && mover_pos.y.is_finite() && mover_pos.z.is_finite(),
                "non-finite mover position {mover_pos:?} at step {step}"
            );
            assert!(
                velocity.x.is_finite() && velocity.y.is_finite() && velocity.z.is_finite(),
                "non-finite mover velocity {velocity:?} at step {step}"
            );
        }
    }
}
