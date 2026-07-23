//! Pile benchmarks: Falling Boxes, Candy Cups, Washer, Destruction
//! (`sample_benchmark.cpp` :250 / :295 / :965; `benchmarks.c` CreateWasher :535).

use super::{add_ground_box, empty_scene, new_world, BenchKind, BenchScene, WasherState};
use crate::rng::XorShift32;
use crate::vis::{hull_edges, hull_triangles, mesh_triangle_edges, VisBody};
use box3d_rust::body::{create_body, destroy_body};
use box3d_rust::hull::{create_hull, destroy_hull, make_box_hull, make_cube_hull};
use box3d_rust::id::BodyId;
use box3d_rust::math_functions::{
    compute_cos_sin, dot, inv_rotate_vector, make_quat_from_axis_angle, mul_add, normalize,
    rotate_vector, Pos, Transform, Vec3, PI, QUAT_IDENTITY, VEC3_AXIS_Z, VEC3_ONE,
};
use box3d_rust::mesh::{create_grid_mesh, MeshData};
use box3d_rust::shape::{create_hull_shape, create_mesh_shape};
use box3d_rust::types::{default_body_def, default_explosion_def, default_shape_def, BodyType};
use box3d_rust::world::world_explode;

/// `FallingBoxes` (`sample_benchmark.cpp` :250). `n = DEBUG 4 : 50`; the browser
/// uses **50** (the C release value) — 50×8×8 = 3200 unit cubes, which serial wasm
/// holds interactively.
pub(crate) fn build_falling_boxes() -> BenchScene {
    let n = 50i32;
    let world = new_world();
    let mut scene = empty_scene(world, Vec::new(), BenchKind::FallingBoxes);

    add_ground_box(&mut scene, 100.0);

    let a = 0.5f32;
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let shape_def = default_shape_def();
    let box_hull = make_cube_hull(a);

    for i in 0..n {
        for j in 0..8 {
            for k in 0..8 {
                body_def.position = Pos {
                    x: -16.0 * a + 4.0 * a * j as f32,
                    y: 4.0 * a * i as f32 + 5.0 * a,
                    z: -16.0 * a + 4.0 * a * k as f32,
                };
                let body = create_body(&mut scene.world, &body_def);
                create_hull_shape(&mut scene.world, body, &shape_def, &box_hull.base);
                scene
                    .bodies
                    .push(VisBody::box_body(body.index1 - 1, a, a, a));
            }
        }
    }
    scene
}

/// `CandyCups::CreateConvex` (`sample_benchmark.cpp` :337) — an 8-sided frustum
/// hull (bottom ring `radius1`@`height1`, top ring `radius2`@`height2`).
fn create_convex(
    radius1: f32,
    height1: f32,
    radius2: f32,
    height2: f32,
) -> box3d_rust::hull::HullData {
    const SIDE_COUNT: i32 = 8;
    let delta_alpha = 2.0 * PI / SIDE_COUNT as f32;
    let mut verts = Vec::with_capacity((2 * SIDE_COUNT) as usize);
    let mut alpha = 0.0f32;
    for _ in 0..SIDE_COUNT {
        let cs = compute_cos_sin(alpha);
        verts.push(Vec3 {
            x: radius1 * cs.cosine,
            y: height1,
            z: radius1 * cs.sine,
        });
        verts.push(Vec3 {
            x: radius2 * cs.cosine,
            y: height2,
            z: radius2 * cs.sine,
        });
        alpha += delta_alpha;
    }
    create_hull(&verts, verts.len() as i32).expect("candy cup hull")
}

/// `CandyCups` (`sample_benchmark.cpp` :295). `n = m = DEBUG 4 : 16`; the browser
/// uses **4** (release 16×16×16 = 4096 convex hulls does not hold interactively).
/// Cups render from the exact frustum hull (`vis::hull_triangles`), the same hull
/// the physics uses. Each cup is a unit-scaled kind-3 (cylinder) render slot whose
/// per-instance geometry the browser swaps for the frustum via [`bench_candy_hull`].
pub(crate) fn build_candy_cups() -> BenchScene {
    let n = 4i32;
    let m = 4i32;
    let world = new_world();
    let mut scene = empty_scene(world, Vec::new(), BenchKind::CandyCups);

    add_ground_box(&mut scene, 60.0);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let shape_def = default_shape_def();
    let convex = create_convex(0.6, 0.0, 0.95, 1.0);
    // Bake the frustum solid faces once (cup-local space, y∈[0,1]); every cup shares
    // this geometry, rendered at each cup's raw body pose (no local offset).
    scene.candy_hull = hull_triangles(&convex);

    for i in 0..n {
        for j in 0..m {
            for k in 0..m {
                body_def.position = Pos {
                    x: -10.0 + 2.5 * j as f32,
                    y: 1.0 * i as f32,
                    z: -10.0 + 2.5 * k as f32,
                };
                let body = create_body(&mut scene.world, &body_def);
                create_hull_shape(&mut scene.world, body, &shape_def, &convex);
                // Unit kind-3 render slot at the raw body pose: the browser draws the
                // frustum hull (bench_candy_hull) here at scale 1, so it matches the
                // collision geometry exactly.
                scene.bodies.push(VisBody::cylinder_local(
                    body.index1 - 1,
                    1.0,
                    1.0,
                    Transform {
                        p: Vec3 {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                        },
                        q: QUAT_IDENTITY,
                    },
                    0,
                ));
            }
        }
    }
    destroy_hull(convex);
    scene
}

// ---------------------------------------------------------------------------
// Washer
// ---------------------------------------------------------------------------

/// `CreateWasher` (`benchmarks.c` :535). Kinematic drum (36 wall segments + 4
/// inner ribs) spun about Z, filled with a grid of small cubes. `gridCount =
/// DEBUG 8 : 20`; the browser uses **8** (release 20³ = 8000 cubes does not hold
/// interactively).
pub(crate) fn build_washer() -> BenchScene {
    let world = new_world();
    let mut scene = empty_scene(world, Vec::new(), BenchKind::Washer);

    // Ground box(60,1,60) at y=-1.
    {
        let mut body_def = default_body_def();
        body_def.position = Pos {
            x: 0.0,
            y: -1.0,
            z: 0.0,
        };
        let ground = create_body(&mut scene.world, &body_def);
        let hull = make_box_hull(60.0, 1.0, 60.0);
        create_hull_shape(&mut scene.world, ground, &default_shape_def(), &hull.base);
        scene
            .bodies
            .push(VisBody::box_body(ground.index1 - 1, 60.0, 1.0, 60.0));
    }

    // Kinematic drum (returns the body plus its baked child-hull geometry).
    let (drum_id, drum_tris, drum_edges) = build_washer_drum(&mut scene);

    // Cube fill.
    let grid_count = 8i32;
    let a = 0.2f32;
    let cube = make_box_hull(a, a, a);
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let shape_def = default_shape_def();

    let mut x = -2.0 * a * grid_count as f32;
    for _i in 0..grid_count {
        let mut y = -2.0 * a * grid_count as f32 + 21.0;
        for _j in 0..grid_count {
            let mut z = -2.0 * a * grid_count as f32;
            for _k in 0..grid_count {
                body_def.position = Pos { x, y, z };
                let body = create_body(&mut scene.world, &body_def);
                create_hull_shape(&mut scene.world, body, &shape_def, &cube.base);
                scene
                    .bodies
                    .push(VisBody::box_body(body.index1 - 1, a, a, a));
                z += 4.0 * a;
            }
            y += 4.0 * a;
        }
        x += 4.0 * a;
    }

    scene.washer = Some(WasherState {
        drum_id,
        drum_radius: 17.0,
        drum_half_len: 10.0,
        drum_tris,
        drum_edges,
    });
    scene
}

/// Build the kinematic drum body (36 wall segments + ribs). Returns the drum body
/// plus its child-hull geometry flattened to drum-local space (`(tris, edges)`) so
/// the browser can render the real drum instead of a cylinder outline.
fn build_washer_drum(scene: &mut BenchScene) -> (BodyId, Vec<f32>, Vec<f32>) {
    let motor_speed = 25.0f32;
    let mut body_def = default_body_def();
    body_def.position = Pos {
        x: 0.0,
        y: 21.0,
        z: 0.0,
    };
    body_def.type_ = BodyType::Kinematic;
    body_def.angular_velocity = Vec3 {
        x: 0.0,
        y: 0.0,
        z: (PI / 180.0) * motor_speed,
    };
    body_def.linear_velocity = Vec3 {
        x: 0.001,
        y: -0.002,
        z: 0.0,
    };
    let drum = create_body(&mut scene.world, &body_def);
    let shape_def = default_shape_def();

    let r0 = 14.0f32;
    let r1 = 16.0f32;
    let r2 = 18.0f32;
    let nd = Vec3 {
        x: 0.0,
        y: 0.0,
        z: -10.0,
    };
    let pd = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 10.0,
    };
    let angle = PI / 18.0;
    let q = make_quat_from_axis_angle(VEC3_AXIS_Z, angle);
    let qo = make_quat_from_axis_angle(VEC3_AXIS_Z, 0.1 * angle);
    let mut drum_tris: Vec<f32> = Vec::new();
    let mut drum_edges: Vec<f32> = Vec::new();
    let mut u1 = Vec3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };
    for i in 0..36 {
        let u2 = if i == 35 {
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            }
        } else {
            rotate_vector(q, u1)
        };

        {
            let a1 = inv_rotate_vector(qo, u1);
            let a2 = rotate_vector(qo, u2);
            let points = [
                mul_add(nd, r1, a1),
                mul_add(nd, r2, a1),
                mul_add(nd, r1, a2),
                mul_add(nd, r2, a2),
                mul_add(pd, r1, a1),
                mul_add(pd, r2, a1),
                mul_add(pd, r1, a2),
                mul_add(pd, r2, a2),
            ];
            let hull = create_hull(&points, 8).expect("washer wall hull");
            create_hull_shape(&mut scene.world, drum, &shape_def, &hull);
            drum_tris.extend_from_slice(&hull_triangles(&hull));
            drum_edges.extend_from_slice(&hull_edges(&hull));
            destroy_hull(hull);
        }

        if i % 9 == 0 {
            let points = [
                mul_add(nd, r0, u1),
                mul_add(nd, r1, u1),
                mul_add(nd, r0, u2),
                mul_add(nd, r1, u2),
                mul_add(pd, r0, u1),
                mul_add(pd, r1, u1),
                mul_add(pd, r0, u2),
                mul_add(pd, r1, u2),
            ];
            let hull = create_hull(&points, 8).expect("washer rib hull");
            create_hull_shape(&mut scene.world, drum, &shape_def, &hull);
            drum_tris.extend_from_slice(&hull_triangles(&hull));
            drum_edges.extend_from_slice(&hull_edges(&hull));
            destroy_hull(hull);
        }

        u1 = u2;
    }
    (drum, drum_tris, drum_edges)
}

// ---------------------------------------------------------------------------
// Destruction
// ---------------------------------------------------------------------------

/// Live state for the Destruction respawn cycle (`BenchmarkDestruction`).
pub(crate) struct DestructionState {
    pub grid_count: i32,
    pub extent: f32,
    pub random_range: i32,
    pub rng: XorShift32,
    pub explosion_radius: f32,
    pub explosion_falloff: f32,
    pub explosion_position: Pos,
    pub explosion_impulse: f32,
    pub spawn_step: u32,
    pub body_ids: Vec<BodyId>,
    /// Kept alive for the ground wireframe; unused after `ground_edges` is built.
    pub _grid_mesh: MeshData,
}

/// `BenchmarkDestruction` (`sample_benchmark.cpp` :1224). `m_small = m_isDebug`,
/// so the browser uses the DEBUG configuration: gridCount 6, extent 0.75, impulse
/// 200 (release: gridCount 20, extent 2.5, impulse 1000). The block grid is
/// re-spawned + re-exploded every 80 steps (release: 140).
pub(crate) fn build_destruction() -> BenchScene {
    let world = new_world();
    let mut scene = empty_scene(world, Vec::new(), BenchKind::Destruction);

    let small = true;
    let grid_count = 6i32;
    let extent = 0.75f32;

    // Ground grid mesh (b3CreateGridMesh(40, 40, 1, 0, true)).
    let grid_mesh = create_grid_mesh(40, 40, 1.0, 0, true).expect("destruction grid mesh");
    let mut ground_def = default_body_def();
    let ground = create_body(&mut scene.world, &ground_def);
    create_mesh_shape(
        &mut scene.world,
        ground,
        &default_shape_def(),
        &grid_mesh,
        VEC3_ONE,
    );
    scene.ground_edges = mesh_triangle_edges(&grid_mesh, VEC3_ONE);
    ground_def.type_ = BodyType::Dynamic;

    let mut ex = default_explosion_def();
    ex.radius = extent;
    ex.falloff = 0.5 * extent;
    ex.position = Pos {
        x: 0.0,
        y: 2.0 * extent,
        z: 0.0,
    };
    ex.impulse_per_area = if small { 200.0 } else { 1000.0 };

    let mut dstate = DestructionState {
        grid_count,
        extent,
        random_range: if small { 3 } else { 2 },
        // Sample ctor reseeds g_randomSeed = RAND_SEED (12345); Destruction never
        // reseeds, so the stream continues from there across respawns.
        rng: XorShift32::with_seed(12345),
        explosion_radius: ex.radius,
        explosion_falloff: ex.falloff,
        explosion_position: ex.position,
        explosion_impulse: ex.impulse_per_area,
        spawn_step: if small { 80 } else { 140 },
        body_ids: Vec::new(),
        _grid_mesh: grid_mesh,
    };

    spawn_destruction(&mut scene, &mut dstate);
    scene.destruction = Some(dstate);
    scene
}

/// `BenchmarkDestruction::Spawn` (`sample_benchmark.cpp` :1329) — fill the grid
/// (skipping cells at random), then explode.
fn spawn_destruction(scene: &mut BenchScene, d: &mut DestructionState) {
    let a = d.extent / d.grid_count as f32;
    let box_hull = make_box_hull(0.8 * a, 0.8 * a, 0.8 * a);
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let shape_def = default_shape_def();

    d.body_ids.clear();
    for i in 0..d.grid_count {
        for j in 0..d.grid_count {
            for k in 0..d.grid_count {
                if d.rng.range_int(1, d.random_range) == 1 {
                    continue;
                }
                body_def.position = Pos {
                    x: (2.0 * i as f32 - d.grid_count as f32 + 1.0) * a,
                    y: (2.0 * j as f32 + 1.0) * a,
                    z: (2.0 * k as f32 - d.grid_count as f32 + 1.0) * a,
                };
                let body = create_body(&mut scene.world, &body_def);
                create_hull_shape(&mut scene.world, body, &shape_def, &box_hull.base);
                d.body_ids.push(body);
                scene.bodies.push(VisBody::box_body(
                    body.index1 - 1,
                    0.8 * a,
                    0.8 * a,
                    0.8 * a,
                ));
            }
        }
    }

    let mut ex = default_explosion_def();
    ex.radius = d.explosion_radius;
    ex.falloff = d.explosion_falloff;
    ex.position = d.explosion_position;
    ex.impulse_per_area = d.explosion_impulse;
    world_explode(&mut scene.world, &ex);
}

/// `BenchmarkDestruction::Step` (`sample_benchmark.cpp` :1387) — every `spawnStep`
/// steps destroy the whole block grid and spawn a fresh one.
pub(crate) fn step_destruction(scene: &mut BenchScene) {
    let Some(mut d) = scene.destruction.take() else {
        return;
    };
    if scene.step_count > 0 && scene.step_count.is_multiple_of(d.spawn_step) {
        for &id in &d.body_ids {
            destroy_body(&mut scene.world, id);
        }
        scene.bodies.clear();
        spawn_destruction(scene, &mut d);
    }
    scene.destruction = Some(d);
}

// ---------------------------------------------------------------------------
// Convex Pile
// ---------------------------------------------------------------------------

/// PEEL's `BasicRandom` LCG, ported verbatim from `benchmarks.c` so the 32-point
/// hull is bit-identical to the original (`ConvexPileRandom`). This is NOT the
/// shared `g_randomSeed` XorShift stream — it is its own fixed-seed generator.
struct ConvexPileRandom {
    state: u32,
}

impl ConvexPileRandom {
    /// `NextConvexPileRandom` (`benchmarks.c` :921).
    fn next(&mut self) -> u32 {
        self.state = self.state.wrapping_mul(2147001325).wrapping_add(715136305);
        self.state
    }

    /// `ConvexPileRandomFloat` — a float in `[-0.5, 0.5]` (`benchmarks.c` :928).
    fn next_float(&mut self) -> f32 {
        (self.next() & 0xffff) as f32 / 65535.0 - 0.5
    }

    /// `UnitRandomPoint` — a uniform direction, rejection sampled inside the unit
    /// sphere then pushed to the surface (`benchmarks.c` :934).
    fn unit_random_point(&mut self) -> Vec3 {
        loop {
            let point = Vec3 {
                x: self.next_float(),
                y: self.next_float(),
                z: self.next_float(),
            };
            if dot(point, point) <= 0.25 {
                return normalize(point);
            }
        }
    }
}

/// `CreateConvexPile` (`benchmarks.c` :949) — a huge pile of large convexes ported
/// from PEEL. Each convex is the hull of 32 random points on a sphere of radius
/// `amplitude`, seeded so the shape is identical across runs. `layers = DEBUG 10 :
/// 80`; the browser uses **10** (release 80 = 5120 hulls does not hold interactively
/// in the serial wasm build). Every hull is the same shared convex, rendered from
/// its baked triangles ([`super::bench_convex_pile_hull`]) at each body's pose via a
/// unit kind-3 render slot (the same swap path as Candy Cups).
pub(crate) fn build_convex_pile() -> BenchScene {
    let world = new_world();
    let mut scene = empty_scene(world, Vec::new(), BenchKind::ConvexPile);

    // Ground box(250,1,250) at y=-1.
    {
        let mut body_def = default_body_def();
        body_def.position = Pos {
            x: 0.0,
            y: -1.0,
            z: 0.0,
        };
        let ground = create_body(&mut scene.world, &body_def);
        let box_hull = make_box_hull(250.0, 1.0, 250.0);
        create_hull_shape(
            &mut scene.world,
            ground,
            &default_shape_def(),
            &box_hull.base,
        );
        scene
            .bodies
            .push(VisBody::box_body(ground.index1 - 1, 250.0, 1.0, 250.0));
    }

    let count_x = 8i32;
    let count_z = 8i32;
    let layers = 10i32; // BENCHMARK_DEBUG (C release 80)
    let amplitude = 2.0f32;
    let point_count = 32usize;
    let scatter = 2.0 * amplitude;

    // Hull around 32 random points on a sphere of radius amplitude.
    let mut rng = ConvexPileRandom { state: 42 };
    let mut points = Vec::with_capacity(point_count);
    for _ in 0..point_count {
        let p = rng.unit_random_point();
        points.push(Vec3 {
            x: amplitude * p.x,
            y: amplitude * p.y,
            z: amplitude * p.z,
        });
    }
    let convex = create_hull(&points, point_count as i32).expect("convex pile hull");
    // Bake the shared hull solid faces once (hull-local space); every body draws it
    // at its raw body pose (no local offset), matching the collision geometry.
    scene.convex_pile_hull = hull_triangles(&convex);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let shape_def = default_shape_def();

    // Grid tall enough to collapse into a pile.
    for layer in 0..layers {
        for z in 0..count_z {
            for x in 0..count_x {
                let pos_x = (x as f32 - 0.5 * count_x as f32) * scatter;
                let pos_z = (z as f32 - 0.5 * count_z as f32) * scatter;
                let pos_y = amplitude + 2.0 * amplitude * layer as f32;
                body_def.position = Pos {
                    x: pos_x,
                    y: pos_y,
                    z: pos_z,
                };
                let body = create_body(&mut scene.world, &body_def);
                create_hull_shape(&mut scene.world, body, &shape_def, &convex);
                scene.bodies.push(VisBody::cylinder_local(
                    body.index1 - 1,
                    1.0,
                    1.0,
                    Transform {
                        p: Vec3 {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                        },
                        q: QUAT_IDENTITY,
                    },
                    0,
                ));
            }
        }
    }
    destroy_hull(convex);
    scene
}
