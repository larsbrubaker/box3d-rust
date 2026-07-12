//! Shared scene builders ported from `box3d-cpp-reference/shared/benchmarks.c`.
//!
//! Release-sized layouts (`NDEBUG` / non-debug) match the C benchmark app.
//! Criterion configs keep sample counts and measurement time modest so default
//! and `--quick` runs stay practical.
//!
//! Included via `#[path]` into each `[[bench]]` crate, so only a subset of
//! scenes is used per target — allow unused items.

#![allow(dead_code)]

use std::time::Duration;

use box3d_rust::body::{body_set_target_transform, create_body};
use box3d_rust::geometry::Sphere;
use box3d_rust::hull::{create_cylinder, create_rock, make_box_hull, make_offset_box_hull};
use box3d_rust::id::BodyId;
use box3d_rust::joint::create_spherical_joint;
use box3d_rust::math_functions::{
    compute_cos_sin, Pos, Transform, Vec3, WorldTransform, PI, QUAT_IDENTITY,
};
use box3d_rust::shape::{create_hull_shape, create_sphere_shape};
use box3d_rust::types::{
    default_body_def, default_shape_def, default_spherical_joint_def, default_world_def, BodyType,
};
use box3d_rust::world::{world_enable_sleeping, World};
use criterion::Criterion;

pub const TIME_STEP: f32 = 1.0 / 60.0;
pub const SUB_STEP_COUNT: i32 = 4;

/// Modest defaults so `cargo bench` is useful without multi-minute runs.
pub fn configure_group<'a>(
    c: &'a mut Criterion,
    name: &str,
) -> criterion::BenchmarkGroup<'a, criterion::measurement::WallTime> {
    let mut group = c.benchmark_group(name);
    group.sample_size(10);
    group.warm_up_time(Duration::from_millis(500));
    group.measurement_time(Duration::from_secs(2));
    group
}

fn new_world() -> World {
    World::new(&default_world_def())
}

/// (CreateJointGrid) — n×n spherical-joint grid, sleeping disabled.
pub fn create_joint_grid() -> World {
    let mut world = new_world();
    world_enable_sleeping(&mut world, false);

    // C: BENCHMARK_DEBUG ? 10 : 100
    let n = if cfg!(debug_assertions) { 10 } else { 100 };

    let mut bodies = Vec::with_capacity((n * n) as usize);
    let mut shape_def = default_shape_def();
    shape_def.filter.category_bits = 2;
    // C assigns `~2u` (uint32) into uint64_t maskBits — zero-extend, not !2u64.
    shape_def.filter.mask_bits = (!2u32) as u64;

    let sphere = Sphere {
        center: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        radius: 0.4,
    };

    let mut joint_def = default_spherical_joint_def();
    let mut body_def = default_body_def();
    body_def.enable_sleep = false;

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
                x: fk as _,
                y: (-fi) as _,
                z: 0.0 as _,
            };

            let body = create_body(&mut world, &body_def);
            create_sphere_shape(&mut world, body, &shape_def, &sphere);

            let index = bodies.len();
            if i > 0 {
                joint_def.base.body_id_a = bodies[index - 1];
                joint_def.base.body_id_b = body;
                joint_def.base.local_frame_a = Transform {
                    p: Vec3 {
                        x: 0.0,
                        y: -0.5,
                        z: 0.0,
                    },
                    q: QUAT_IDENTITY,
                };
                joint_def.base.local_frame_b = Transform {
                    p: Vec3 {
                        x: 0.0,
                        y: 0.5,
                        z: 0.0,
                    },
                    q: QUAT_IDENTITY,
                };
                create_spherical_joint(&mut world, &joint_def);
            }

            if k > 0 {
                joint_def.base.body_id_a = bodies[index - n as usize];
                joint_def.base.body_id_b = body;
                joint_def.base.local_frame_a = Transform {
                    p: Vec3 {
                        x: 0.5,
                        y: 0.0,
                        z: 0.0,
                    },
                    q: QUAT_IDENTITY,
                };
                joint_def.base.local_frame_b = Transform {
                    p: Vec3 {
                        x: -0.5,
                        y: 0.0,
                        z: 0.0,
                    },
                    q: QUAT_IDENTITY,
                };
                create_spherical_joint(&mut world, &joint_def);
            }

            bodies.push(body);
        }
    }

    world
}

/// (CreateLargePyramid)
pub fn create_large_pyramid() -> World {
    let mut world = new_world();
    world_enable_sleeping(&mut world, false);

    // C: BENCHMARK_DEBUG ? 20 : 90
    let base_count = if cfg!(debug_assertions) { 20 } else { 90 };

    {
        let mut body_def = default_body_def();
        body_def.position = Pos {
            x: 0.0 as _,
            y: (-1.0) as _,
            z: 0.0 as _,
        };
        let ground_id = create_body(&mut world, &body_def);
        let box_hull = make_box_hull(400.0, 1.0, 400.0);
        let shape_def = default_shape_def();
        create_hull_shape(&mut world, ground_id, &shape_def, &box_hull.base);
    }

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;

    let mut shape_def = default_shape_def();
    shape_def.density = 100.0;

    let h = 0.5f32;
    let box_hull = make_box_hull(h, h, h);
    let shift = 1.0 * h;

    for i in 0..base_count {
        let y = (2.0 * i as f32 + 1.0) * shift;
        for j in i..base_count {
            let x = (i as f32 + 1.0) * shift + 2.0 * (j - i) as f32 * shift - h * base_count as f32;
            body_def.position = Pos {
                x: x as _,
                y: y as _,
                z: 0.0 as _,
            };
            let body_id = create_body(&mut world, &body_def);
            create_hull_shape(&mut world, body_id, &shape_def, &box_hull.base);
        }
    }

    world
}

fn create_small_pyramid(
    world: &mut World,
    base_count: i32,
    extent: f32,
    center_x: f32,
    base_z: f32,
) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.enable_sleep = false;

    let mut shape_def = default_shape_def();
    shape_def.density = 100.0;

    let box_hull = make_box_hull(extent, extent, extent);

    for i in 0..base_count {
        let y = (2.0 * i as f32 + 1.0) * extent;
        for j in i..base_count {
            let x = (i as f32 + 1.0) * extent + 2.0 * (j - i) as f32 * extent + center_x - 0.5;
            body_def.position = Pos {
                x: x as _,
                y: y as _,
                z: base_z as _,
            };
            let body_id = create_body(world, &body_def);
            create_hull_shape(world, body_id, &shape_def, &box_hull.base);
        }
    }
}

/// (CreateManyPyramids)
pub fn create_many_pyramids() -> World {
    let mut world = new_world();

    let base_count = 10;
    let extent = 0.5f32;
    // C: BENCHMARK_DEBUG ? 3 : 14
    let row_count = if cfg!(debug_assertions) { 3 } else { 14 };
    let column_count = if cfg!(debug_assertions) { 3 } else { 14 };
    let ground_extent = extent * column_count as f32 * (base_count as f32 + 1.0);

    {
        let mut body_def = default_body_def();
        body_def.position = Pos {
            x: 0.0 as _,
            y: (-1.0) as _,
            z: 0.0 as _,
        };
        let ground_id = create_body(&mut world, &body_def);
        let shape_def = default_shape_def();
        let box_hull = make_box_hull(ground_extent, 1.0, ground_extent);
        create_hull_shape(&mut world, ground_id, &shape_def, &box_hull.base);
    }

    let base_width = 2.0 * extent * base_count as f32;
    let mut base_z = -ground_extent + 2.0 * extent;
    let delta_z = 2.0 * (ground_extent - 2.0 * extent) / (row_count as f32 - 1.0);

    for _i in 0..row_count {
        for j in 0..column_count {
            let center_x =
                -ground_extent + j as f32 * (base_width + 2.0 * extent) + 2.0 * extent;
            create_small_pyramid(&mut world, base_count, extent, center_x, base_z);
        }
        base_z += delta_z;
    }

    world
}

/// State for the junkyard kinematic pusher (StepJunkyard).
pub struct Junkyard {
    pub world: World,
    pusher_id: BodyId,
    degrees: f32,
    radius: f32,
}

impl Junkyard {
    /// (CreateJunkyard)
    pub fn create() -> Self {
        let mut world = new_world();

        let ground_id = {
            let mut body_def = default_body_def();
            body_def.position.y = (-1.0) as _;
            create_body(&mut world, &body_def)
        };

        {
            let shape_def = default_shape_def();
            {
                let box_hull = make_box_hull(120.0, 1.0, 120.0);
                create_hull_shape(&mut world, ground_id, &shape_def, &box_hull.base);
            }
            {
                let offset = Vec3 {
                    x: -50.0,
                    y: 8.0,
                    z: 0.0,
                };
                let box_hull = make_offset_box_hull(1.0, 8.0, 50.0, offset);
                create_hull_shape(&mut world, ground_id, &shape_def, &box_hull.base);
            }
            {
                let offset = Vec3 {
                    x: 50.0,
                    y: 8.0,
                    z: 0.0,
                };
                let box_hull = make_offset_box_hull(1.0, 8.0, 50.0, offset);
                create_hull_shape(&mut world, ground_id, &shape_def, &box_hull.base);
            }
            {
                let offset = Vec3 {
                    x: 0.0,
                    y: 8.0,
                    z: -50.0,
                };
                let box_hull = make_offset_box_hull(50.0, 8.0, 1.0, offset);
                create_hull_shape(&mut world, ground_id, &shape_def, &box_hull.base);
            }
            {
                let offset = Vec3 {
                    x: 0.0,
                    y: 8.0,
                    z: 50.0,
                };
                let box_hull = make_offset_box_hull(50.0, 8.0, 1.0, offset);
                create_hull_shape(&mut world, ground_id, &shape_def, &box_hull.base);
            }
        }

        {
            let rock_hull = create_rock(1.5).expect("create_rock");
            // C: BENCHMARK_DEBUG ? 2 : 24
            let count = if cfg!(debug_assertions) { 2 } else { 24 };
            let height = 24.0f32;
            let mut body_def = default_body_def();
            body_def.type_ = BodyType::Dynamic;
            let shape_def = default_shape_def();
            for y in 0..count {
                for x in 0..=20 {
                    for z in 0..=20 {
                        body_def.position.x = (-40.0 + 4.0 * x as f32) as _;
                        body_def.position.y = (4.0 * y as f32 + height + 1.0) as _;
                        body_def.position.z = (-40.0 + 4.0 * z as f32) as _;
                        let body_id = create_body(&mut world, &body_def);
                        create_hull_shape(&mut world, body_id, &shape_def, &rock_hull);
                    }
                }
            }
        }

        let radius = 35.0f32;
        let m_height = 24.0f32;
        let hull = create_cylinder(m_height, 4.0, 0.0, 16).expect("create_cylinder");
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Kinematic;
        body_def.position = Pos {
            x: radius as _,
            y: 0.0 as _,
            z: 0.0 as _,
        };
        let pusher_id = create_body(&mut world, &body_def);
        let shape_def = default_shape_def();
        create_hull_shape(&mut world, pusher_id, &shape_def, &hull);

        Self {
            world,
            pusher_id,
            degrees: 0.0,
            radius,
        }
    }

    /// (StepJunkyard) then `world.step`.
    pub fn step(&mut self) {
        let omega = -6.0f32;
        self.degrees += omega * TIME_STEP;
        let cs = compute_cos_sin(self.degrees * PI / 180.0);
        let r = self.radius;
        let target_pos = Pos {
            x: (r * cs.cosine) as _,
            y: 0.0 as _,
            z: (r * cs.sine) as _,
        };
        let target = WorldTransform {
            p: target_pos,
            q: QUAT_IDENTITY,
        };
        body_set_target_transform(&mut self.world, self.pusher_id, target, TIME_STEP, false);
        self.world.step(TIME_STEP, SUB_STEP_COUNT);
    }
}

/// Warm-up step matching the C benchmark (initial step is expensive / skewed).
pub fn warm_up(world: &mut World) {
    world.step(TIME_STEP, SUB_STEP_COUNT);
}
