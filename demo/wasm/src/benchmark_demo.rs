//! Benchmark samples: Large Pyramid, Junkyard, Falling Trees.
//!
//! Ports `CreateLargePyramid` / `CreateJunkyard`+`StepJunkyard` / `CreateTrees` from
//! `box3d-cpp-reference/shared/benchmarks.c` with browser-scaled counts.
//!
//! Counts vs C (both are Erin's — C release when it runs interactively in serial
//! wasm, else the C DEBUG value; never a third invented number):
//! - Large Pyramid: `baseCount=20` (C DEBUG; C release 90 is a full 3D pyramid of
//!   hundreds of thousands of bodies, far beyond serial wasm).
//! - Junkyard: full C geometry (ground half 120, walls 1×8×50 at ±50, pusher radius
//!   35, cylinder 24×4). Rocks = C DEBUG 2 layers × 21×21 (882) at 4 m spacing from
//!   -40, height 24 (C release uses 24 layers).
//! - Falling Trees: wave mesh `scale*150 × scale*200` (default scale 1 = 150×200,
//!   matching CreateTrees100), 10 trees × 22 tapering hulls (bodyCount = C DEBUG;
//!   22 hulls is fixed in C, not a debug/release split). C release uses 50 trees.

use crate::interact::{self, MouseGrab};
use box3d_rust::body::{
    body_apply_mass_from_shapes, body_get_world_center, body_set_angular_velocity,
    body_set_linear_velocity, body_set_target_transform, create_body, destroy_body,
    get_body_transform, make_body_id,
};
use box3d_rust::hull::{create_cylinder, create_rock, make_box_hull, make_offset_box_hull};
use box3d_rust::id::BodyId;
use box3d_rust::math_functions::{
    compute_cos_sin, cross, mul_transforms, sub_pos, Pos, Transform, Vec3, WorldTransform,
    QUAT_IDENTITY, VEC3_ONE,
};
use box3d_rust::mesh::{create_wave_mesh, MeshData};
use box3d_rust::shape::{create_hull_shape, create_mesh_shape};
use box3d_rust::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use box3d_rust::world::{world_enable_sleeping, World};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

thread_local! {
    static BENCH: RefCell<Option<BenchState>> = const { RefCell::new(None) };
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum BenchMode {
    LargePyramid,
    Junkyard,
    Trees,
}

struct BenchBody {
    body_index: i32,
    /// Half extents (box/rock approx) or [radius, half_height, radius] for cylinders.
    half_extents: [f32; 3],
    /// 0 = box/rock, 1 = sphere unused, 2 = capsule unused, 3 = cylinder
    kind: u8,
    local: Option<Transform>,
}

struct JunkyardAnim {
    pusher_id: BodyId,
    degrees: f32,
    radius: f32,
}

struct BenchState {
    world: World,
    bodies: Vec<BenchBody>,
    grab: MouseGrab,
    mode: BenchMode,
    junkyard: Option<JunkyardAnim>,
    /// Wave mesh for Falling Trees (kept for wireframe export).
    tree_mesh: Option<MeshData>,
}

fn with_bench<R>(f: impl FnOnce(&mut BenchState) -> R) -> R {
    BENCH.with(|cell| {
        let mut slot = cell.borrow_mut();
        let bench = slot
            .as_mut()
            .expect("benchmark not initialized — call bench_reset_* first");
        f(bench)
    })
}

fn new_world() -> World {
    // Restore the base Sample launch-speed scale (5.0) on every scene reset (each
    // bench reset builds one world through here); overrides re-apply after reset.
    crate::interact::reset_scene_scales();
    let mut def = default_world_def();
    def.gravity = Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    World::new(&def)
}

fn push_vis(
    bench: &mut BenchState,
    body_index: i32,
    hx: f32,
    hy: f32,
    hz: f32,
    kind: u8,
    local: Option<Transform>,
) {
    bench.bodies.push(BenchBody {
        body_index,
        half_extents: [hx, hy, hz],
        kind,
        local,
    });
}

/// `CreateLargePyramid` — sleep disabled, density 100, half-size 0.5.
/// C fixes `baseCount = BENCHMARK_DEBUG ? 20 : 90`; the browser uses the DEBUG
/// value 20 (release 90 builds hundreds of thousands of bodies). C has no runtime
/// baseCount control, so the count is pinned to 20 here.
#[wasm_bindgen]
pub fn bench_reset_large_pyramid() -> u32 {
    let base = 20i32;
    BENCH.with(|cell| {
        let mut world = new_world();
        world_enable_sleeping(&mut world, false);

        let mut bench = BenchState {
            world,
            bodies: Vec::new(),
            grab: MouseGrab::default(),
            mode: BenchMode::LargePyramid,
            junkyard: None,
            tree_mesh: None,
        };

        {
            let mut body_def = default_body_def();
            body_def.position = Pos {
                x: 0.0 as _,
                y: (-1.0) as _,
                z: 0.0 as _,
            };
            let ground = create_body(&mut bench.world, &body_def);
            let shape_def = default_shape_def();
            let box_hull = make_box_hull(400.0, 1.0, 400.0);
            create_hull_shape(&mut bench.world, ground, &shape_def, &box_hull.base);
            push_vis(&mut bench, ground.index1 - 1, 400.0, 1.0, 400.0, 0, None);
        }

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        let mut shape_def = default_shape_def();
        shape_def.density = 100.0;
        let h = 0.5f32;
        let box_hull = make_box_hull(h, h, h);
        let shift = 1.0 * h;

        for i in 0..base {
            let y = (2.0 * i as f32 + 1.0) * shift;
            for j in i..base {
                let x = (i as f32 + 1.0) * shift + 2.0 * (j - i) as f32 * shift - h * base as f32;
                body_def.position = Pos {
                    x: x as _,
                    y: y as _,
                    z: 0.0 as _,
                };
                let body_id = create_body(&mut bench.world, &body_def);
                create_hull_shape(&mut bench.world, body_id, &shape_def, &box_hull.base);
                push_vis(&mut bench, body_id.index1 - 1, h, h, h, 0, None);
            }
        }

        let n = bench.bodies.len() as u32;
        *cell.borrow_mut() = Some(bench);
        n
    })
}

/// Browser-scaled `CreateJunkyard`: DEBUG-like layer count, smaller rock grid.
/// Uses `create_rock` + kinematic cylinder pusher orbiting via `SetTargetTransform`.
#[wasm_bindgen]
pub fn bench_reset_junkyard() -> u32 {
    BENCH.with(|cell| {
        let world = new_world();
        let mut bench = BenchState {
            world,
            bodies: Vec::new(),
            grab: MouseGrab::default(),
            mode: BenchMode::Junkyard,
            junkyard: None,
            tree_mesh: None,
        };

        // Arena: C ground half 120, walls MakeOffsetBoxHull(1,8,50) at ±50.
        let wall = 50.0f32;
        let ground_half = 120.0f32;
        let wall_h = 8.0f32;

        let mut ground_def = default_body_def();
        ground_def.position = Pos {
            x: 0.0 as _,
            y: (-1.0) as _,
            z: 0.0 as _,
        };
        let ground_id = create_body(&mut bench.world, &ground_def);
        let shape_def = default_shape_def();

        let floor = make_box_hull(ground_half, 1.0, ground_half);
        create_hull_shape(&mut bench.world, ground_id, &shape_def, &floor.base);
        push_vis(
            &mut bench,
            ground_id.index1 - 1,
            ground_half,
            1.0,
            ground_half,
            0,
            None,
        );

        let walls = [
            (
                1.0f32,
                wall_h,
                wall,
                Vec3 {
                    x: -wall,
                    y: wall_h,
                    z: 0.0,
                },
            ),
            (
                1.0,
                wall_h,
                wall,
                Vec3 {
                    x: wall,
                    y: wall_h,
                    z: 0.0,
                },
            ),
            (
                wall,
                wall_h,
                1.0,
                Vec3 {
                    x: 0.0,
                    y: wall_h,
                    z: -wall,
                },
            ),
            (
                wall,
                wall_h,
                1.0,
                Vec3 {
                    x: 0.0,
                    y: wall_h,
                    z: wall,
                },
            ),
        ];
        for (hx, hy, hz, offset) in walls {
            let hull = make_offset_box_hull(hx, hy, hz, offset);
            create_hull_shape(&mut bench.world, ground_id, &shape_def, &hull.base);
            // Visual: child of ground with local offset (half extents at center).
            push_vis(
                &mut bench,
                ground_id.index1 - 1,
                hx,
                hy,
                hz,
                0,
                Some(Transform {
                    p: offset,
                    q: QUAT_IDENTITY,
                }),
            );
        }

        // Rocks: C DEBUG 2 layers × 21×21 (X,Z in 0..=20) at 4 m spacing from -40,
        // stacked at height 24 (C release uses 24 layers). 2×21×21 = 882 rocks.
        let rock_hull = create_rock(1.5).expect("rock hull");
        let layer_count = 2i32;
        let height = 24.0f32;

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        let rock_shape = default_shape_def();
        for y_i in 0..layer_count {
            for x_i in 0..=20 {
                for z_i in 0..=20 {
                    body_def.position = Pos {
                        x: (-40.0 + 4.0 * x_i as f32) as _,
                        y: (4.0 * y_i as f32 + height + 1.0) as _,
                        z: (-40.0 + 4.0 * z_i as f32) as _,
                    };
                    let body_id = create_body(&mut bench.world, &body_def);
                    create_hull_shape(&mut bench.world, body_id, &rock_shape, &rock_hull);
                    // Approximate rock as a box for rendering (radius 1.5).
                    push_vis(&mut bench, body_id.index1 - 1, 1.2, 1.2, 1.2, 0, None);
                }
            }
        }

        // Pusher: C radius 35, CreateCylinder(24, 4, 0, 16), kinematic at {35,0,0}.
        let pusher_radius = 35.0f32;
        let pusher_height = 24.0f32;
        let pusher_cyl_r = 4.0f32;
        let cyl = create_cylinder(pusher_height, pusher_cyl_r, 0.0, 16).expect("pusher cylinder");
        let mut kin_def = default_body_def();
        kin_def.type_ = BodyType::Kinematic;
        kin_def.position = Pos {
            x: pusher_radius as _,
            y: 0.0 as _,
            z: 0.0 as _,
        };
        let pusher_id = create_body(&mut bench.world, &kin_def);
        create_hull_shape(&mut bench.world, pusher_id, &default_shape_def(), &cyl);
        push_vis(
            &mut bench,
            pusher_id.index1 - 1,
            pusher_cyl_r,
            pusher_height * 0.5,
            pusher_cyl_r,
            3,
            Some(Transform {
                p: Vec3 {
                    x: 0.0,
                    y: pusher_height * 0.5,
                    z: 0.0,
                },
                q: QUAT_IDENTITY,
            }),
        );

        bench.junkyard = Some(JunkyardAnim {
            pusher_id,
            degrees: 0.0,
            radius: pusher_radius,
        });

        let n = bench.bodies.len() as u32;
        *cell.borrow_mut() = Some(bench);
        n
    })
}

fn step_junkyard(bench: &mut BenchState, dt: f32) {
    let Some(anim) = bench.junkyard.as_mut() else {
        return;
    };
    const OMEGA: f32 = -6.0;
    anim.degrees += OMEGA * dt;
    let cs = compute_cos_sin(anim.degrees * std::f32::consts::PI / 180.0);
    let r = anim.radius;
    let target = WorldTransform {
        p: Pos {
            x: (r * cs.cosine) as _,
            y: 0.0 as _,
            z: (r * cs.sine) as _,
        },
        q: QUAT_IDENTITY,
    };
    body_set_target_transform(&mut bench.world, anim.pusher_id, target, dt, false);
}

/// `CreateTrees` — wave-mesh ground + tapering cylinder trees.
/// `grid_size` mirrors the C DrawControls radio (100/50/25 cm cell width) mapping
/// to `scale` 1/2/4: mesh is `scale*150 × scale*200`, `cellWidth = 1/scale`.
/// Default (100 cm → scale 1) is CreateTrees100 = 150×200. Trees: bodyCount = 10
/// (C DEBUG; release 50), 22 tapering hulls each (fixed in C), z start -15 (C DEBUG).
#[wasm_bindgen]
pub fn bench_reset_trees(grid_size: u32) -> u32 {
    BENCH.with(|cell| {
        let world = new_world();
        let mut bench = BenchState {
            world,
            bodies: Vec::new(),
            grab: MouseGrab::default(),
            mode: BenchMode::Trees,
            junkyard: None,
            tree_mesh: None,
        };

        let scale = match grid_size {
            25 => 4i32,
            50 => 2i32,
            _ => 1i32, // 100 cm (and any stale 0-arg binding) → default scale 1
        };
        let x_count = scale * 150;
        let z_count = scale * 200;
        let cell_width = 1.0f32 / scale as f32;
        let mesh =
            create_wave_mesh(x_count, z_count, cell_width, 0.4, 0.05, 0.1).expect("wave mesh");

        let mut ground_def = default_body_def();
        ground_def.position = Pos {
            x: 0.0 as _,
            y: 0.0 as _,
            z: 0.0 as _,
        };
        let ground = create_body(&mut bench.world, &ground_def);
        create_mesh_shape(
            &mut bench.world,
            ground,
            &default_shape_def(),
            &mesh,
            VEC3_ONE,
        );
        // Mesh is drawn via wireframe; no box proxy for ground.
        bench.tree_mesh = Some(mesh);

        let body_count = 10i32;
        let hull_count = 22i32;
        let mut hulls = Vec::with_capacity(hull_count as usize);
        let mut segs = Vec::with_capacity(hull_count as usize);
        let mut y = 1.0f32;
        let mut r = 0.75f32;
        let l = 1.5f32;
        for _ in 0..hull_count {
            let y_offset = y - r;
            let height = l + 2.0 * r;
            hulls.push(create_cylinder(height, r, y_offset, 6).expect("tree cylinder"));
            segs.push((r, y_offset + height * 0.5, height * 0.5));
            y += l + 2.0 * r;
            r *= 0.95;
        }

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.sleep_threshold = 0.2;
        body_def.rotation = QUAT_IDENTITY;

        let mut shape_def = default_shape_def();
        shape_def.base_material.friction = 0.9;
        shape_def.base_material.rolling_resistance = 0.05;
        shape_def.update_body_mass = false;
        shape_def.density = 1.0;

        let mut angular_velocity = -0.5f32;
        let mut z = -15.0f32;
        for body_index in 0..body_count {
            body_def.position = Pos {
                x: 0.0 as _,
                y: 1.0 as _,
                z: z as _,
            };
            let body_id = create_body(&mut bench.world, &body_def);
            for hull in &hulls {
                create_hull_shape(&mut bench.world, body_id, &shape_def, hull);
            }
            for &(radius, y_center, half_h) in &segs {
                push_vis(
                    &mut bench,
                    body_id.index1 - 1,
                    radius,
                    half_h,
                    radius,
                    3,
                    Some(Transform {
                        p: Vec3 {
                            x: 0.0,
                            y: y_center,
                            z: 0.0,
                        },
                        q: QUAT_IDENTITY,
                    }),
                );
            }

            let velocity_scale = 0.5 + (0.5 * body_index as f32) / body_count as f32;
            body_apply_mass_from_shapes(&mut bench.world, body_id);
            let center = body_get_world_center(&bench.world, body_id);
            let omega = Vec3 {
                x: 0.0,
                y: 0.0,
                z: velocity_scale * angular_velocity,
            };
            let v = cross(omega, sub_pos(center, body_def.position));
            body_set_angular_velocity(&mut bench.world, body_id, omega);
            body_set_linear_velocity(&mut bench.world, body_id, v);

            z += 3.0;
            angular_velocity = -angular_velocity;
        }

        let n = bench.bodies.len() as u32;
        *cell.borrow_mut() = Some(bench);
        n
    })
}

/// Advance the benchmark simulation. Returns body count.
#[wasm_bindgen]
pub fn bench_step(dt: f32, sub_steps: i32) -> u32 {
    with_bench(|bench| {
        if bench.mode == BenchMode::Junkyard {
            step_junkyard(bench, dt);
        }
        bench.grab.pre_step(&mut bench.world, dt);
        bench.world.step(dt, sub_steps);
        bench.bodies.len() as u32
    })
}

/// Interleaved poses: `[px,py,pz, qx,qy,qz,qw, hx,hy,hz, kind]` per visual part.
#[wasm_bindgen]
pub fn bench_body_poses() -> Vec<f32> {
    with_bench(|bench| {
        let mut out = Vec::with_capacity(bench.bodies.len() * 11);
        for b in &bench.bodies {
            let xf = get_body_transform(&bench.world, b.body_index);
            let (px, py, pz, qx, qy, qz, qw) = if let Some(local) = b.local {
                let parent = Transform {
                    p: Vec3 {
                        x: xf.p.x as f32,
                        y: xf.p.y as f32,
                        z: xf.p.z as f32,
                    },
                    q: xf.q,
                };
                let world = mul_transforms(parent, local);
                (
                    world.p.x,
                    world.p.y,
                    world.p.z,
                    world.q.v.x,
                    world.q.v.y,
                    world.q.v.z,
                    world.q.s,
                )
            } else {
                (
                    xf.p.x as f32,
                    xf.p.y as f32,
                    xf.p.z as f32,
                    xf.q.v.x,
                    xf.q.v.y,
                    xf.q.v.z,
                    xf.q.s,
                )
            };
            out.push(px);
            out.push(py);
            out.push(pz);
            out.push(qx);
            out.push(qy);
            out.push(qz);
            out.push(qw);
            out.push(b.half_extents[0]);
            out.push(b.half_extents[1]);
            out.push(b.half_extents[2]);
            out.push(b.kind as f32);
        }
        out
    })
}

#[wasm_bindgen]
pub fn bench_body_count() -> u32 {
    with_bench(|bench| bench.bodies.len() as u32)
}

/// Falling Trees wave-mesh wireframe edges: `[x0,y0,z0, x1,y1,z1, ...]`.
#[wasm_bindgen]
pub fn bench_mesh_wireframe() -> Vec<f32> {
    with_bench(|bench| {
        let Some(mesh) = &bench.tree_mesh else {
            return Vec::new();
        };
        crate::vis::mesh_triangle_edges(mesh, VEC3_ONE)
    })
}

#[wasm_bindgen]
pub fn bench_mouse_down(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> Vec<f32> {
    with_bench(|bench| {
        if bench.grab.begin(
            &mut bench.world,
            interact::pos(ox, oy, oz),
            interact::vec3(tx, ty, tz),
        ) {
            let p = bench.grab.mouse_point;
            vec![1.0, p.x as f32, p.y as f32, p.z as f32]
        } else {
            vec![0.0, 0.0, 0.0, 0.0]
        }
    })
}

#[wasm_bindgen]
pub fn bench_mouse_move(px: f32, py: f32, pz: f32) {
    with_bench(|bench| {
        bench.grab.move_to(interact::pos(px, py, pz));
    })
}

#[wasm_bindgen]
pub fn bench_mouse_up() {
    with_bench(|bench| {
        bench.grab.end(&mut bench.world);
    })
}

#[wasm_bindgen]
pub fn bench_mouse_active() -> bool {
    with_bench(|bench| bench.grab.is_active())
}

#[wasm_bindgen]
pub fn bench_spawn_random(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> Vec<f32> {
    with_bench(|bench| {
        match interact::spawn_random(
            &mut bench.world,
            interact::pos(ox, oy, oz),
            interact::vec3(tx, ty, tz),
        ) {
            Some(spawned) => {
                bench.bodies.push(BenchBody {
                    body_index: spawned.body_index,
                    half_extents: spawned.half_extents,
                    kind: spawned.kind,
                    local: None,
                });
                vec![
                    1.0,
                    spawned.body_index as f32,
                    spawned.half_extents[0],
                    spawned.half_extents[1],
                    spawned.half_extents[2],
                    spawned.kind as f32,
                ]
            }
            None => vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
    })
}

#[wasm_bindgen]
pub fn bench_delete_at_ray(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> u32 {
    with_bench(|bench| {
        let index = interact::delete_at_ray(
            &mut bench.world,
            &mut bench.grab,
            interact::pos(ox, oy, oz),
            interact::vec3(tx, ty, tz),
        );
        if index < 0 {
            return 0;
        }
        bench.bodies.retain(|b| b.body_index != index);
        1
    })
}

#[wasm_bindgen]
pub fn bench_counters() -> Vec<f32> {
    with_bench(|bench| interact::counters_with_sleep(&bench.world).to_vec())
}

#[wasm_bindgen]
pub fn bench_debug_draw(_flags: u32) -> Vec<f32> {
    with_bench(|bench| interact::collect_debug_draw(&mut bench.world))
}

#[wasm_bindgen]
pub fn bench_destroy_body_index(body_index: i32) -> u32 {
    with_bench(|bench| {
        bench.grab.end(&mut bench.world);
        if !bench.bodies.iter().any(|b| b.body_index == body_index) {
            return 0;
        }
        let id = make_body_id(&bench.world, body_index);
        destroy_body(&mut bench.world, id);
        bench.bodies.retain(|b| b.body_index != body_index);
        1
    })
}
