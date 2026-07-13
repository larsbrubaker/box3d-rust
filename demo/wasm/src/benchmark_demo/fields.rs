//! Field benchmarks: Explosion, Height Field, Large World
//! (`sample_benchmark.cpp` Explosion :373, Height Field :490, Large World :992;
//! `benchmarks.c` CreateLargeWorld :449 / StepLargeWorld :478).

use super::{empty_scene, new_world, BenchKind, BenchScene};
use crate::vis::{hf_triangle_edges, mesh_triangle_edges, VisBody};
use box3d_rust::body::create_body;
use box3d_rust::distance::ShapeProxy;
use box3d_rust::geometry::Sphere;
use box3d_rust::height_field::create_wave;
use box3d_rust::hull::{create_cylinder, make_box_hull, make_transformed_box_hull};
use box3d_rust::math_functions::{Pos, Transform, Vec3, QUAT_IDENTITY, VEC3_ONE, VEC3_ZERO};
use box3d_rust::mesh::create_grid_mesh;
use box3d_rust::shape::{
    create_height_field_shape, create_hull_shape, create_mesh_shape, create_sphere_shape,
};
use box3d_rust::types::{
    default_body_def, default_explosion_def, default_query_filter, default_shape_def, BodyType,
};
use box3d_rust::world::{world_cast_ray_closest, world_cast_shape, world_explode};

// ---------------------------------------------------------------------------
// Explosion
// ---------------------------------------------------------------------------

/// `BenchmarkExplosion` (`sample_benchmark.cpp` :373). A walled arena of upright
/// cylinders with `explosionScale = 2`; the Explode button applies a radial
/// impulse. `n = DEBUG 3 : 16`; the browser uses **16** (release), `(2·16+1)² =
/// 1089` cylinders — they settle and sleep, so serial wasm holds it.
pub(crate) fn build_explosion() -> BenchScene {
    let world = new_world();
    let mut scene = empty_scene(world, Vec::new(), BenchKind::Explosion);

    let shape_def = default_shape_def();
    let grid_mesh = create_grid_mesh(40, 40, 1.0, 0, true).expect("explosion grid mesh");

    let mut body_def = default_body_def();
    let ground = create_body(&mut scene.world, &body_def);
    create_mesh_shape(&mut scene.world, ground, &shape_def, &grid_mesh, VEC3_ONE);
    scene.ground_edges = mesh_triangle_edges(&grid_mesh, VEC3_ONE);

    let hy = 1.0f32;
    let walls = [
        (
            20.0f32,
            hy,
            0.1f32,
            Vec3 {
                x: 0.0,
                y: hy,
                z: -20.0,
            },
        ),
        (
            20.0,
            hy,
            0.1,
            Vec3 {
                x: 0.0,
                y: hy,
                z: 20.0,
            },
        ),
        (
            0.1,
            hy,
            20.0,
            Vec3 {
                x: -20.0,
                y: hy,
                z: 0.0,
            },
        ),
        (
            0.1,
            hy,
            20.0,
            Vec3 {
                x: 20.0,
                y: hy,
                z: 0.0,
            },
        ),
    ];
    for (hx, why, hz, p) in walls {
        let transform = Transform {
            p,
            q: QUAT_IDENTITY,
        };
        let wall = make_transformed_box_hull(hx, why, hz, transform);
        create_hull_shape(&mut scene.world, ground, &shape_def, &wall.base);
        scene.bodies.push(VisBody::box_local(
            ground.index1 - 1,
            hx,
            why,
            hz,
            transform,
        ));
    }

    // 15-sided cylinder (avoids manifold degeneracies), height 0.5, radius 0.2.
    let cylinder = create_cylinder(0.5, 0.2, 0.0, 15).expect("explosion cylinder");
    let n = 16i32;
    body_def.type_ = BodyType::Dynamic;
    let mut cyl_shape = default_shape_def();
    cyl_shape.explosion_scale = 2.0;

    for i in -n..=n {
        for k in -n..=n {
            body_def.position = Pos {
                x: 1.0 * i as f32,
                y: 0.0,
                z: 1.0 * k as f32,
            };
            let body = create_body(&mut scene.world, &body_def);
            create_hull_shape(&mut scene.world, body, &cyl_shape, &cylinder);
            // Cylinder renders along local Y; create_cylinder(height 0.5) spans
            // y∈[0,0.5] with y_offset 0, so center at y=0.25, half-length 0.25.
            scene.bodies.push(VisBody::cylinder_local(
                body.index1 - 1,
                0.2,
                0.25,
                Transform {
                    p: Vec3 {
                        x: 0.0,
                        y: 0.25,
                        z: 0.0,
                    },
                    q: QUAT_IDENTITY,
                },
                0,
            ));
        }
    }

    scene.explosion_impulse = 1000.0;
    scene
}

/// `BenchmarkExplosion::Explode` (`sample_benchmark.cpp` :456).
pub(crate) fn explode(scene: &mut BenchScene) {
    let mut def = default_explosion_def();
    def.radius = 16.0;
    def.position = Pos {
        x: 0.0,
        y: -4.0,
        z: 0.0,
    };
    def.impulse_per_area = scene.explosion_impulse;
    world_explode(&mut scene.world, &def);
}

// ---------------------------------------------------------------------------
// Height Field
// ---------------------------------------------------------------------------

/// `BenchmarkHeightField` (`sample_benchmark.cpp` :490). A 50×50 wave height field;
/// each frame casts a dense grid of rays (radius 0) or sphere shapes (radius > 0)
/// straight down and reports the hit count + cast time.
pub(crate) fn build_height_field() -> BenchScene {
    let world = new_world();
    let mut scene = empty_scene(world, Vec::new(), BenchKind::HeightField);

    scene.hf_columns = 50;
    scene.hf_rows = 50;
    scene.hf_radius = 0.1;

    let mut body_def = default_body_def();
    body_def.position = Pos {
        x: -0.5 * scene.hf_columns as f32,
        y: 0.0,
        z: -0.5 * scene.hf_rows as f32,
    };
    let body = create_body(&mut scene.world, &body_def);

    let hf = create_wave(50, 50, VEC3_ONE, 0.02, 0.04, true);
    create_height_field_shape(&mut scene.world, body, &default_shape_def(), &hf);
    scene.ground_edges = hf_triangle_edges(
        &hf,
        Vec3 {
            x: -0.5 * scene.hf_columns as f32,
            y: 0.0,
            z: -0.5 * scene.hf_rows as f32,
        },
    );
    scene.hf = Some(hf);
    scene
}

/// Run the cast grid once (`BenchmarkHeightField::Render` :557-632) and return
/// `[castCount, hitCount]`. Uses the C release cast spacing (`delta = 0.4`).
pub(crate) fn height_field_cast(scene: &mut BenchScene) -> Vec<f32> {
    let delta = 0.4f32;
    let span_x = 0.94 * 0.5 * scene.hf_columns as f32;
    let span_z = 0.96 * 0.5 * scene.hf_rows as f32;
    let ray_translation = Vec3 {
        x: 80000.0,
        y: -80000.0,
        z: 8.0,
    };
    let filter = default_query_filter();
    let radius = scene.hf_radius;

    let mut cast_count = 0i32;
    let mut hit_count = 0i32;

    let mut x = -span_x;
    while x <= span_x {
        let mut z = -span_z;
        while z <= span_z {
            let ray_origin = Pos { x, y: 2.0, z };
            let hit = if radius == 0.0 {
                world_cast_ray_closest(&scene.world, ray_origin, ray_translation, &filter).hit
            } else {
                let mut proxy = ShapeProxy {
                    count: 1,
                    radius,
                    ..Default::default()
                };
                proxy.points[0] = VEC3_ZERO;
                let mut got = false;
                world_cast_shape(
                    &scene.world,
                    ray_origin,
                    &proxy,
                    ray_translation,
                    &filter,
                    |_shape, _point, _normal, fraction, _mat, _tri, _child| {
                        got = true;
                        fraction
                    },
                );
                got
            };
            cast_count += 1;
            if hit {
                hit_count += 1;
            }
            z += delta;
        }
        x += delta;
    }

    vec![cast_count as f32, hit_count as f32]
}

// ---------------------------------------------------------------------------
// Large World
// ---------------------------------------------------------------------------

/// Live Large World state (`g_staticFloorData`).
pub(crate) struct LargeWorldState {
    pub spheres_dropped: i32,
    pub sphere_target: i32,
    pub drop_interval: u32,
    pub grid: i32,
    pub cell: f32,
}

/// `CreateLargeWorld` (`benchmarks.c` :449). A huge grid of static floor boxes
/// (each `invokeContactCreation = true`) with a handful of spheres dropped onto it
/// — a broad-phase move-buffer stress test built **at the origin** (not a
/// far-offset scene). `STATIC_FLOOR_GRID = DEBUG 32 : 1000`; the browser uses the
/// DEBUG grid **32** (1024 static boxes; release 1000² = 1M is infeasible in wasm)
/// with **16** dropped spheres (release 100).
pub(crate) fn build_large_world() -> BenchScene {
    let world = new_world();
    let mut scene = empty_scene(world, Vec::new(), BenchKind::LargeWorld);

    let cell = 10.0f32;
    let grid = 32i32;
    let half_span = 0.5 * cell * grid as f32;

    let boxh = make_box_hull(0.5 * cell, 0.25, 0.5 * cell);
    let mut body_def = default_body_def();
    let mut shape_def = default_shape_def();
    shape_def.invoke_contact_creation = true;

    for i in 0..grid {
        let x = -half_span + (i as f32 + 0.5) * cell;
        for j in 0..grid {
            let z = -half_span + (j as f32 + 0.5) * cell;
            body_def.position = Pos { x, y: 0.0, z };
            let body = create_body(&mut scene.world, &body_def);
            create_hull_shape(&mut scene.world, body, &shape_def, &boxh.base);
            scene.bodies.push(VisBody::box_body(
                body.index1 - 1,
                0.5 * cell,
                0.25,
                0.5 * cell,
            ));
        }
    }

    scene.large_world = Some(LargeWorldState {
        spheres_dropped: 0,
        sphere_target: 16,
        drop_interval: 8,
        grid,
        cell,
    });
    scene
}

/// `StepLargeWorld` (`benchmarks.c` :478) — drop one sphere every `interval` steps
/// onto a coarse grid across the floor, until the sphere target is met.
pub(crate) fn step_large_world(scene: &mut BenchScene) {
    let Some(mut lw) = scene.large_world.take() else {
        return;
    };
    let should_drop = lw.spheres_dropped < lw.sphere_target
        && scene.step_count != 0
        && scene.step_count.is_multiple_of(lw.drop_interval);
    if should_drop {
        // Spread spheres across a coarse grid so they don't all pile on one box.
        let mut side = 1i32;
        while side * side < lw.sphere_target {
            side += 1;
        }
        let idx = lw.spheres_dropped;
        let gi = idx % side;
        let gj = idx / side;

        let half_span = 0.5 * lw.cell * lw.grid as f32;
        let inset = 0.1 * 2.0 * half_span;
        let usable = 2.0 * half_span - 2.0 * inset;
        let x = -half_span + inset + (gi as f32 + 0.5) * (usable / side as f32);
        let z = -half_span + inset + (gj as f32 + 0.5) * (usable / side as f32);

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos { x, y: 1.5, z };
        let body = create_body(&mut scene.world, &body_def);
        let sphere = Sphere {
            center: VEC3_ZERO,
            radius: 0.5,
        };
        create_sphere_shape(&mut scene.world, body, &default_shape_def(), &sphere);
        scene
            .bodies
            .push(VisBody::sphere_body(body.index1 - 1, 0.5));

        lw.spheres_dropped += 1;
    }
    scene.large_world = Some(lw);
}
