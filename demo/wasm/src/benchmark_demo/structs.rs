//! Joint/structure benchmarks: Joint Grid, Chains, Hull
//! (`benchmarks.c` CreateJointGrid :33; `sample_benchmark.cpp` Chains :1112,
//! Hull :1024).

use super::{empty_scene, new_world, BenchKind, BenchScene};
use crate::rng::XorShift32;
use crate::vis::mesh_triangle_edges;
use crate::vis::VisBody;
use box3d_rust::body::create_body;
use box3d_rust::geometry::{Capsule, Sphere};
use box3d_rust::hull::{clone_and_transform_hull, create_hull};
use box3d_rust::id::{BodyId, ShapeId};
use box3d_rust::joint::create_spherical_joint;
use box3d_rust::math_functions::{
    add, get_length_and_normalize, lerp, mul_sv, Pos, Transform, Vec3, QUAT_IDENTITY,
    TRANSFORM_IDENTITY, VEC3_ONE, VEC3_ZERO,
};
use box3d_rust::mesh::create_wave_mesh;
use box3d_rust::shape::{
    create_capsule_shape, create_mesh_shape, create_sphere_shape, shape_apply_wind,
};
use box3d_rust::types::{
    default_body_def, default_shape_def, default_spherical_joint_def, BodyType,
};

/// `CreateJointGrid` (`benchmarks.c` :33). Sleep disabled. `n = DEBUG 10 : 100`;
/// the browser uses **10** (release 100² = 10 000 jointed spheres does not hold
/// interactively). Spheres self-collision-filtered (category 2 / mask ~2).
pub(crate) fn build_joint_grid() -> BenchScene {
    let n = 10i32;
    let world = new_world();
    let mut scene = empty_scene(world, Vec::new(), BenchKind::JointGrid);
    box3d_rust::world::world_enable_sleeping(&mut scene.world, false);

    let mut shape_def = default_shape_def();
    shape_def.filter.category_bits = 2;
    shape_def.filter.mask_bits = !2u64;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.4,
    };

    let mut joint_def = default_spherical_joint_def();
    let mut body_def = default_body_def();
    body_def.enable_sleep = false;

    let mut bodies: Vec<BodyId> = Vec::with_capacity((n * n) as usize);
    for k in 0..n {
        for i in 0..n {
            let fk = k as f32;
            let fi = i as f32;
            body_def.type_ = if i == 0 {
                BodyType::Static
            } else {
                BodyType::Dynamic
            };
            body_def.position = Pos {
                x: fk,
                y: -fi,
                z: 0.0,
            };
            let body = create_body(&mut scene.world, &body_def);
            create_sphere_shape(&mut scene.world, body, &shape_def, &sphere);
            scene
                .bodies
                .push(VisBody::sphere_body(body.index1 - 1, 0.4));

            let index = bodies.len();
            if i > 0 {
                joint_def.base.body_id_a = bodies[index - 1];
                joint_def.base.body_id_b = body;
                joint_def.base.local_frame_a.p = Vec3 {
                    x: 0.0,
                    y: -0.5,
                    z: 0.0,
                };
                joint_def.base.local_frame_b.p = Vec3 {
                    x: 0.0,
                    y: 0.5,
                    z: 0.0,
                };
                create_spherical_joint(&mut scene.world, &joint_def);
            }
            if k > 0 {
                joint_def.base.body_id_a = bodies[index - n as usize];
                joint_def.base.body_id_b = body;
                joint_def.base.local_frame_a.p = Vec3 {
                    x: 0.5,
                    y: 0.0,
                    z: 0.0,
                };
                joint_def.base.local_frame_b.p = Vec3 {
                    x: -0.5,
                    y: 0.0,
                    z: 0.0,
                };
                create_spherical_joint(&mut scene.world, &joint_def);
            }
            bodies.push(body);
        }
    }
    scene
}

// ---------------------------------------------------------------------------
// Chains
// ---------------------------------------------------------------------------

/// Live Chains state (`BenchmarkChains`).
pub(crate) struct ChainsState {
    /// Last-link shape id per grid cell (the wind is applied to these).
    pub shape_ids: Vec<ShapeId>,
    pub noise: Vec3,
    pub rng: XorShift32,
}

/// `BenchmarkChains` (`sample_benchmark.cpp` :1112). `gridCount = DEBUG 10 : 25`;
/// the browser uses **10** (release 25² × 4 = 2500 jointed capsules does not hold
/// interactively). Spherical-joint chains over a wave mesh, driven by wind.
pub(crate) fn build_chains() -> BenchScene {
    let grid_count = 10i32;
    let world = new_world();
    let mut scene = empty_scene(world, Vec::new(), BenchKind::Chains);

    // Wave mesh ground.
    let mut ground_def = default_body_def();
    let ground = create_body(&mut scene.world, &ground_def);
    let mesh = create_wave_mesh(80, 80, 1.0, 0.5, 0.05, 0.01).expect("chains wave mesh");
    create_mesh_shape(
        &mut scene.world,
        ground,
        &default_shape_def(),
        &mesh,
        VEC3_ONE,
    );
    scene.ground_edges = mesh_triangle_edges(&mesh, VEC3_ONE);
    ground_def.type_ = BodyType::Dynamic;

    let link_radius = 0.125f32;
    let link_extent = 0.25f32;
    let capsule = Capsule {
        center1: Vec3 {
            x: 0.0,
            y: -link_extent,
            z: 0.0,
        },
        center2: Vec3 {
            x: 0.0,
            y: link_extent,
            z: 0.0,
        },
        radius: link_radius,
    };
    let shape_def = default_shape_def();

    let mut body_def = default_body_def();
    body_def.enable_sleep = false;
    let link_count = 4;

    let mut joint_def = default_spherical_joint_def();
    joint_def.base.local_frame_a = Transform {
        p: Vec3 {
            x: 0.0,
            y: -link_extent,
            z: 0.0,
        },
        q: QUAT_IDENTITY,
    };
    joint_def.base.local_frame_b = Transform {
        p: Vec3 {
            x: 0.0,
            y: link_extent,
            z: 0.0,
        },
        q: QUAT_IDENTITY,
    };
    joint_def.enable_spring = true;
    joint_def.hertz = 1.0;
    joint_def.damping_ratio = 0.7;
    joint_def.enable_motor = true;
    joint_def.max_motor_torque = 1.0;

    let mut shape_ids: Vec<ShapeId> = Vec::with_capacity((grid_count * grid_count) as usize);

    let mut x = -(grid_count as f32);
    for _row in 0..grid_count {
        let mut z = -(grid_count as f32);
        for _col in 0..grid_count {
            for i in 0..link_count {
                body_def.position = Pos {
                    x,
                    y: (1.0 - 2.0 * i as f32) * link_extent + 3.0,
                    z,
                };
                body_def.type_ = if i == 0 {
                    BodyType::Static
                } else {
                    BodyType::Dynamic
                };
                let body = create_body(&mut scene.world, &body_def);
                let shape_id = create_capsule_shape(&mut scene.world, body, &shape_def, &capsule);
                scene
                    .bodies
                    .push(VisBody::capsule_body(body.index1 - 1, &capsule));
                if i == link_count - 1 {
                    shape_ids.push(shape_id);
                }
                if i > 0 {
                    joint_def.base.body_id_b = body;
                    create_spherical_joint(&mut scene.world, &joint_def);
                }
                joint_def.base.body_id_a = body;
            }
            z += 2.0;
        }
        x += 2.0;
    }

    scene.chains = Some(ChainsState {
        shape_ids,
        noise: VEC3_ZERO,
        // Sample ctor reseeds g_randomSeed = RAND_SEED (12345).
        rng: XorShift32::with_seed(12345),
    });
    scene
}

/// `BenchmarkChains::Step` (`sample_benchmark.cpp` :1193) — apply a noisy wind to
/// every chain's last link, then evolve the noise.
pub(crate) fn step_chains(scene: &mut BenchScene) {
    let Some(mut chains) = scene.chains.take() else {
        return;
    };
    let base_wind = Vec3 {
        x: 20.0,
        y: 0.0,
        z: 0.0,
    };
    let mut speed = 0.0f32;
    let direction = get_length_and_normalize(&mut speed, base_wind);
    let wind = mul_sv(speed, add(direction, chains.noise));

    for &shape_id in &chains.shape_ids {
        shape_apply_wind(&mut scene.world, shape_id, wind, 1.0, 1.0, 20.0, false);
    }

    let rand = chains.rng.vec3(
        Vec3 {
            x: -0.3,
            y: -0.3,
            z: -0.3,
        },
        Vec3 {
            x: 0.3,
            y: 0.3,
            z: 0.3,
        },
    );
    chains.noise = lerp(chains.noise, rand, 0.05);
    scene.chains = Some(chains);
}

// ---------------------------------------------------------------------------
// Hull
// ---------------------------------------------------------------------------

/// Twin-deduped wireframe edges of a hull (shared [`crate::vis::hull_edges`]),
/// with every vertex shifted by `offset` for side-by-side placement.
fn hull_edges(hull: &box3d_rust::hull::HullData, offset: Vec3) -> Vec<f32> {
    let mut out = crate::vis::hull_edges(hull);
    for chunk in out.chunks_mut(3) {
        chunk[0] += offset.x;
        chunk[1] += offset.y;
        chunk[2] += offset.z;
    }
    out
}

/// `BenchmarkHull` (`sample_benchmark.cpp` :1024). A pure geometry demo (no
/// bodies): 64 random points hulled, and a mirror-scaled (`{-1,1,1}`) clone.
///
/// **Port note:** the mirror hull is produced by `b3CloneAndTransformHull`
/// (`clone_and_transform_hull`) with identity transform and scale `{-1,1,1}`,
/// exactly as C does — this reflects the source hull (reversing edge winding) and
/// recomputes its geometry. Timing is not reported (serial wasm has no
/// `b3GetTicks`); the readout shows trial count + surface areas instead.
pub(crate) fn build_hull() -> BenchScene {
    let world = new_world();
    let mut scene = empty_scene(world, Vec::new(), BenchKind::Hull);

    // g_randomSeed = 42 (BenchmarkHull ctor).
    let mut rng = XorShift32::with_seed(42);
    let count = 64;
    let lo = Vec3 {
        x: -1.0,
        y: -1.0,
        z: -1.0,
    };
    let hi = Vec3 {
        x: 1.0,
        y: 1.0,
        z: 1.0,
    };
    let mut points = Vec::with_capacity(count);
    for _ in 0..count {
        points.push(rng.vec3(lo, hi));
    }

    let hull = create_hull(&points, count as i32).expect("hull sample hull");
    scene.hull_area = hull.surface_area;
    scene.ground_edges = hull_edges(
        &hull,
        Vec3 {
            x: -2.0,
            y: 0.0,
            z: 0.0,
        },
    );

    // Mirror scale {-1,1,1} via b3CloneAndTransformHull (identity transform),
    // exactly as C's BenchmarkHull does.
    let scale = Vec3 {
        x: -1.0,
        y: 1.0,
        z: 1.0,
    };
    let transformed = clone_and_transform_hull(&hull, TRANSFORM_IDENTITY, scale)
        .expect("hull sample transformed");
    scene.hull_clone_area = transformed.surface_area;
    scene.hull_edges_b = hull_edges(
        &transformed,
        Vec3 {
            x: 2.0,
            y: 0.0,
            z: 0.0,
        },
    );

    scene.hull_trials = 2000; // C release trial count (informational).
    scene
}
