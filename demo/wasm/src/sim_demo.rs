//! Live World::step demos: Bodies, Stacking, Pyramid, and Compound scenes.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::interact::{self, MouseGrab};
use box3d_rust::body::{create_body, destroy_body, get_body_transform, make_body_id};
use box3d_rust::compound::{
    create_compound, CompoundCapsuleDef, CompoundDef, CompoundHullDef, CompoundSphereDef,
};
use box3d_rust::geometry::{default_surface_material, Capsule, Sphere};
use box3d_rust::hull::make_box_hull;
use box3d_rust::math_functions::{
    compute_quat_between_unit_vectors, get_length_and_normalize, make_quat_from_axis_angle,
    mul_transforms, Pos, Transform, Vec3, QUAT_IDENTITY, VEC3_AXIS_Y, VEC3_ZERO,
};
use box3d_rust::shape::{create_compound_shape, create_hull_shape, create_sphere_shape};
use box3d_rust::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use box3d_rust::world::World;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

/// Tiny LCG for village prop placement (demo-only; not the C Random* stream).
struct DemoRng(u32);

impl DemoRng {
    fn next_u32(&mut self) -> u32 {
        self.0 = self.0.wrapping_mul(1664525).wrapping_add(1013904223);
        self.0
    }

    fn next_f32(&mut self) -> f32 {
        (self.next_u32() >> 8) as f32 / (1u32 << 24) as f32
    }

    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.next_f32()
    }

    fn vec3_range(&mut self, lo: Vec3, hi: Vec3) -> Vec3 {
        Vec3 {
            x: self.range(lo.x, hi.x),
            y: self.range(lo.y, hi.y),
            z: self.range(lo.z, hi.z),
        }
    }
}

fn capsule_local_from_centers(c1: Vec3, c2: Vec3, radius: f32) -> (Transform, [f32; 3]) {
    let mut dir = Vec3 {
        x: c2.x - c1.x,
        y: c2.y - c1.y,
        z: c2.z - c1.z,
    };
    let mut len = 0.0;
    dir = get_length_and_normalize(&mut len, dir);
    let mid = Vec3 {
        x: 0.5 * (c1.x + c2.x),
        y: 0.5 * (c1.y + c2.y),
        z: 0.5 * (c1.z + c2.z),
    };
    let q = if len > 1e-6 {
        compute_quat_between_unit_vectors(VEC3_AXIS_Y, dir)
    } else {
        QUAT_IDENTITY
    };
    (
        Transform { p: mid, q },
        [radius, 0.5 * len, radius],
    )
}

thread_local! {
    static SIM: RefCell<Option<SimState>> = const { RefCell::new(None) };
}

struct SimBody {
    body_index: i32,
    half_extents: [f32; 3],
    /// 0 = hull box, 1 = sphere (radius in half_extents[0]), 2 = capsule
    kind: u8,
    /// Optional local transform relative to the body (compound children).
    local: Option<Transform>,
}

struct SimState {
    world: World,
    bodies: Vec<SimBody>,
    grab: MouseGrab,
}

fn with_sim<R>(f: impl FnOnce(&mut SimState) -> R) -> R {
    SIM.with(|cell| {
        let mut slot = cell.borrow_mut();
        let sim = slot
            .as_mut()
            .expect("sim not initialized — call sim_reset_* first");
        f(sim)
    })
}

fn push_dynamic_box(
    sim: &mut SimState,
    x: f32,
    y: f32,
    z: f32,
    hx: f32,
    hy: f32,
    hz: f32,
    density: f32,
    friction: f32,
) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    };
    let body_id = create_body(&mut sim.world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = density;
    shape_def.base_material.friction = friction;
    let hull = make_box_hull(hx, hy, hz);
    create_hull_shape(&mut sim.world, body_id, &shape_def, &hull.base);
    sim.bodies.push(SimBody {
        body_index: body_id.index1 - 1,
        half_extents: [hx, hy, hz],
        kind: 0,
        local: None,
    });
}

fn push_dynamic_box_locked(sim: &mut SimState, x: f32, y: f32, z: f32, hx: f32, hy: f32, hz: f32) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    };
    body_def.motion_locks.linear_z = true;
    body_def.motion_locks.angular_x = true;
    body_def.motion_locks.angular_y = true;
    let body_id = create_body(&mut sim.world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;
    let hull = make_box_hull(hx, hy, hz);
    create_hull_shape(&mut sim.world, body_id, &shape_def, &hull.base);
    sim.bodies.push(SimBody {
        body_index: body_id.index1 - 1,
        half_extents: [hx, hy, hz],
        kind: 0,
        local: None,
    });
}

fn push_dynamic_sphere(sim: &mut SimState, x: f32, y: f32, z: f32, radius: f32, density: f32) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    };
    let body_id = create_body(&mut sim.world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.density = density;
    shape_def.base_material.rolling_resistance = 0.1;
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius,
    };
    create_sphere_shape(&mut sim.world, body_id, &shape_def, &sphere);
    sim.bodies.push(SimBody {
        body_index: body_id.index1 - 1,
        half_extents: [radius, radius, radius],
        kind: 1,
        local: None,
    });
}

fn add_ground(sim: &mut SimState, half_extent: f32) {
    let mut ground_def = default_body_def();
    ground_def.type_ = BodyType::Static;
    ground_def.position = Pos {
        x: 0.0 as _,
        y: (-half_extent) as _,
        z: 0.0 as _,
    };
    let ground = create_body(&mut sim.world, &ground_def);
    let shape_def = default_shape_def();
    let hull = make_box_hull(half_extent * 2.0, half_extent, half_extent * 2.0);
    create_hull_shape(&mut sim.world, ground, &shape_def, &hull.base);
    sim.bodies.push(SimBody {
        body_index: ground.index1 - 1,
        half_extents: [half_extent * 2.0, half_extent, half_extent * 2.0],
        kind: 0,
        local: None,
    });
}

fn new_world() -> World {
    let mut def = default_world_def();
    def.gravity = Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    World::new(&def)
}

/// Bodies demo: ground + falling cube (HelloWorld) + a few companions.
#[wasm_bindgen]
pub fn sim_reset_bodies() -> u32 {
    SIM.with(|cell| {
        let mut sim = SimState {
            world: new_world(),
            bodies: Vec::new(),
            grab: MouseGrab::default(),
        };
        add_ground(&mut sim, 10.0);
        push_dynamic_box(&mut sim, 0.0, 4.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.3);
        push_dynamic_box(&mut sim, -3.0, 6.0, 0.0, 0.5, 0.5, 0.5, 1.0, 0.4);
        push_dynamic_box(&mut sim, 3.0, 8.0, 1.0, 0.75, 0.4, 0.75, 1.5, 0.5);
        push_dynamic_sphere(&mut sim, 1.5, 10.0, -1.0, 0.6, 1.0);
        push_dynamic_sphere(&mut sim, -1.5, 12.0, 0.5, 0.4, 0.8);
        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

/// Compound / Simple — matches `sample_compound.cpp` SimpleCompound (single hull + sphere).
#[wasm_bindgen]
pub fn sim_reset_compound() -> u32 {
    sim_reset_compound_simple()
}

/// Named Simple compound gallery sample (C: Compound / Simple).
#[wasm_bindgen]
pub fn sim_reset_compound_simple() -> u32 {
    SIM.with(|cell| {
        let mut sim = SimState {
            world: new_world(),
            bodies: Vec::new(),
            grab: MouseGrab::default(),
        };

        let a = 4.0f32;
        let box_hull = make_box_hull(a, 0.125 * a, a);
        let material = default_surface_material();
        let hull_transform = Transform {
            p: Vec3 {
                x: 1.0,
                y: -0.125 * a,
                z: 0.0,
            },
            q: QUAT_IDENTITY,
        };
        let hulls = [CompoundHullDef {
            hull: &box_hull.base,
            transform: hull_transform,
            material,
        }];

        let compound = create_compound(&CompoundDef {
            hulls: &hulls,
            ..Default::default()
        })
        .expect("compound");

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Static;
        body_def.position = Pos {
            x: 2.0 as _,
            y: (-1.0) as _,
            z: 0.0 as _,
        };
        body_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_Y, 0.25 * std::f32::consts::PI);
        let ground = create_body(&mut sim.world, &body_def);
        create_compound_shape(&mut sim.world, ground, &default_shape_def(), &compound);

        let parent_index = ground.index1 - 1;
        sim.bodies.push(SimBody {
            body_index: parent_index,
            half_extents: [a, 0.125 * a, a],
            kind: 0,
            local: Some(hull_transform),
        });

        push_dynamic_sphere(&mut sim, 0.0, 2.0, 0.0, 0.25, 1.0);

        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

/// Compound / Spheres — cloud of compound spheres (C: Compound / Spheres, count 20).
#[wasm_bindgen]
pub fn sim_reset_compound_spheres() -> u32 {
    SIM.with(|cell| {
        let mut sim = SimState {
            world: new_world(),
            bodies: Vec::new(),
            grab: MouseGrab::default(),
        };

        let mut rng = DemoRng(0xC0FF_EE42);
        let h = 10.0f32;
        let material = default_surface_material();
        let mut spheres = Vec::with_capacity(20);
        for _ in 0..20 {
            let center = rng.vec3_range(
                Vec3 {
                    x: -h,
                    y: -h,
                    z: -h,
                },
                Vec3 {
                    x: h,
                    y: h,
                    z: h,
                },
            );
            let radius = rng.range(0.01 * h, 0.05 * h);
            spheres.push(CompoundSphereDef {
                sphere: Sphere { center, radius },
                material,
            });
        }

        let compound = create_compound(&CompoundDef {
            spheres: &spheres,
            ..Default::default()
        })
        .expect("compound spheres");

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Static;
        let ground = create_body(&mut sim.world, &body_def);
        create_compound_shape(&mut sim.world, ground, &default_shape_def(), &compound);

        let parent_index = ground.index1 - 1;
        for s in &spheres {
            sim.bodies.push(SimBody {
                body_index: parent_index,
                half_extents: [s.sphere.radius, s.sphere.radius, s.sphere.radius],
                kind: 1,
                local: Some(Transform {
                    p: s.sphere.center,
                    q: QUAT_IDENTITY,
                }),
            });
        }

        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

/// Compound / Hulls — cloud of compound box hulls (C: Compound / Hulls, count 20).
#[wasm_bindgen]
pub fn sim_reset_compound_hulls() -> u32 {
    SIM.with(|cell| {
        let mut sim = SimState {
            world: new_world(),
            bodies: Vec::new(),
            grab: MouseGrab::default(),
        };

        let mut rng = DemoRng(0xA011_C0DE);
        let h = 10.0f32;
        let material = default_surface_material();
        // Keep owned hulls alive for create_compound.
        let mut box_hulls = Vec::with_capacity(20);
        let mut extents = Vec::with_capacity(20);
        let mut transforms = Vec::with_capacity(20);
        for _ in 0..20 {
            let e = Vec3 {
                x: rng.range(0.01 * h, 0.05 * h),
                y: rng.range(0.01 * h, 0.05 * h),
                z: rng.range(0.01 * h, 0.05 * h),
            };
            extents.push(e);
            box_hulls.push(make_box_hull(e.x, e.y, e.z));
            let mut axis = rng.vec3_range(
                Vec3 {
                    x: -1.0,
                    y: -1.0,
                    z: -1.0,
                },
                Vec3 {
                    x: 1.0,
                    y: 1.0,
                    z: 1.0,
                },
            );
            let mut axis_len = 0.0;
            axis = get_length_and_normalize(&mut axis_len, axis);
            if axis_len < 1e-4 {
                axis = VEC3_AXIS_Y;
            }
            transforms.push(Transform {
                p: rng.vec3_range(
                    Vec3 {
                        x: -h,
                        y: -h,
                        z: -h,
                    },
                    Vec3 {
                        x: h,
                        y: h,
                        z: h,
                    },
                ),
                q: make_quat_from_axis_angle(axis, rng.range(0.0, std::f32::consts::TAU)),
            });
        }

        let hulls: Vec<CompoundHullDef<'_>> = box_hulls
            .iter()
            .zip(transforms.iter())
            .map(|(bh, xf)| CompoundHullDef {
                hull: &bh.base,
                transform: *xf,
                material,
            })
            .collect();

        let compound = create_compound(&CompoundDef {
            hulls: &hulls,
            ..Default::default()
        })
        .expect("compound hulls");

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Static;
        let ground = create_body(&mut sim.world, &body_def);
        create_compound_shape(&mut sim.world, ground, &default_shape_def(), &compound);

        let parent_index = ground.index1 - 1;
        for (i, xf) in transforms.iter().enumerate() {
            let e = extents[i];
            sim.bodies.push(SimBody {
                body_index: parent_index,
                half_extents: [e.x, e.y, e.z],
                kind: 0,
                local: Some(*xf),
            });
        }

        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

/// Compound / Village — tiled compound hull ground + odd-tile sphere/capsule props.
///
/// C uses `gridCount = 8` (debug) / `200` (release) plus building meshes. Browser scale is
/// `grid_count` clamped to 8..=16 (default 10); building meshes are omitted (no OBJ loader).
#[wasm_bindgen]
pub fn sim_reset_village(grid_count: u32) -> u32 {
    let grid = grid_count.clamp(8, 16) as i32;
    SIM.with(|cell| {
        let mut sim = SimState {
            world: new_world(),
            bodies: Vec::new(),
            grab: MouseGrab::default(),
        };

        let a = 4.0f32;
        let mut rng = DemoRng(0xB111_A6E7);
        let material = default_surface_material();
        let box_hull = make_box_hull(a, 0.5 * a, a);

        let hull_count = (grid * grid) as usize;
        let prop_capacity = hull_count / 8 + 1;

        let mut capsules: Vec<CompoundCapsuleDef> = Vec::with_capacity(prop_capacity);
        let mut spheres: Vec<CompoundSphereDef> = Vec::with_capacity(prop_capacity);
        let mut hull_transforms: Vec<Transform> = Vec::with_capacity(hull_count);

        let mut transform = Transform {
            p: VEC3_ZERO,
            q: QUAT_IDENTITY,
        };

        for i in 0..grid {
            transform.p.x = (2.0 * i as f32 - grid as f32) * a;
            for j in 0..grid {
                transform.p.z = (2.0 * j as f32 - grid as f32) * a;
                transform.p.y = rng.range(-0.25, 0.125) * a;

                if (i & 1) != 0 && (j & 1) != 0 {
                    let p1 = Vec3 {
                        x: transform.p.x,
                        y: transform.p.y,
                        z: transform.p.z,
                    } + rng.vec3_range(
                        Vec3 {
                            x: -a,
                            y: a,
                            z: -a,
                        },
                        Vec3 {
                            x: a,
                            y: 2.0 * a,
                            z: a,
                        },
                    );
                    let p2 = Vec3 {
                        x: transform.p.x,
                        y: transform.p.y,
                        z: transform.p.z,
                    } + rng.vec3_range(
                        Vec3 {
                            x: -a,
                            y: a,
                            z: -a,
                        },
                        Vec3 {
                            x: a,
                            y: 2.0 * a,
                            z: a,
                        },
                    );
                    let radius = rng.range(0.1, 0.5);
                    if capsules.len() < spheres.len() {
                        if capsules.len() < prop_capacity {
                            capsules.push(CompoundCapsuleDef {
                                capsule: Capsule {
                                    center1: p1,
                                    center2: p2,
                                    radius,
                                },
                                material,
                            });
                        }
                    } else if spheres.len() < prop_capacity {
                        spheres.push(CompoundSphereDef {
                            sphere: Sphere {
                                center: p1,
                                radius,
                            },
                            material,
                        });
                    }
                }

                hull_transforms.push(transform);
            }
        }

        let hulls: Vec<CompoundHullDef<'_>> = hull_transforms
            .iter()
            .map(|xf| CompoundHullDef {
                hull: &box_hull.base,
                transform: *xf,
                material,
            })
            .collect();

        let compound = create_compound(&CompoundDef {
            capsules: &capsules,
            hulls: &hulls,
            spheres: &spheres,
            ..Default::default()
        })
        .expect("village compound");

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Static;
        body_def.position = Pos {
            x: (-1.0) as _,
            y: (-0.5) as _,
            z: 2.0 as _,
        };
        body_def.rotation = make_quat_from_axis_angle(VEC3_AXIS_Y, -1.15 * std::f32::consts::PI);
        let ground = create_body(&mut sim.world, &body_def);
        create_compound_shape(&mut sim.world, ground, &default_shape_def(), &compound);

        let parent_index = ground.index1 - 1;
        for xf in &hull_transforms {
            sim.bodies.push(SimBody {
                body_index: parent_index,
                half_extents: [a, 0.5 * a, a],
                kind: 0,
                local: Some(*xf),
            });
        }
        for s in &spheres {
            sim.bodies.push(SimBody {
                body_index: parent_index,
                half_extents: [s.sphere.radius, s.sphere.radius, s.sphere.radius],
                kind: 1,
                local: Some(Transform {
                    p: s.sphere.center,
                    q: QUAT_IDENTITY,
                }),
            });
        }
        for c in &capsules {
            let (local, half) =
                capsule_local_from_centers(c.capsule.center1, c.capsule.center2, c.capsule.radius);
            sim.bodies.push(SimBody {
                body_index: parent_index,
                half_extents: half,
                kind: 2,
                local: Some(local),
            });
        }

        // A few dynamic drop-ins so the village is interactive like other dynamics demos.
        push_dynamic_sphere(&mut sim, 0.0, 12.0, 0.0, 0.4, 1.0);
        push_dynamic_sphere(&mut sim, 3.0, 14.0, -2.0, 0.35, 1.0);
        push_dynamic_box(&mut sim, -2.0, 13.0, 1.0, 0.4, 0.4, 0.4, 1.0, 0.4);

        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

/// Stacking demo: vertical box stack (mirrors sample BoxStack, fewer boxes for the browser).
#[wasm_bindgen]
pub fn sim_reset_stacking(count: u32) -> u32 {
    let n = count.clamp(1, 24);
    SIM.with(|cell| {
        let mut sim = SimState {
            world: new_world(),
            bodies: Vec::new(),
            grab: MouseGrab::default(),
        };
        add_ground(&mut sim, 20.0);
        let a = 0.5f32;
        for i in 0..n {
            let y = 1.5 * a + 2.5 * a * i as f32;
            push_dynamic_box(&mut sim, 0.0, y, 0.0, a, a, a, 1.0, 0.5);
        }
        let total = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        total
    })
}

/// Pyramid2D stacking (motion-locked to XY plane). `size` is base row length (2–10).
#[wasm_bindgen]
pub fn sim_reset_pyramid(size: u32) -> u32 {
    let n = size.clamp(2, 10) as i32;
    SIM.with(|cell| {
        let mut sim = SimState {
            world: new_world(),
            bodies: Vec::new(),
            grab: MouseGrab::default(),
        };
        add_ground(&mut sim, 30.0);
        let a = 0.75f32;
        for row in 0..n {
            for column in 0..(n - row) {
                let x = (-0.5 * (n - row - 1) as f32 + column as f32) * 2.0 * a;
                let y = (1.5 + 2.5 * row as f32) * a;
                push_dynamic_box_locked(&mut sim, x, y, 0.0, a, a, a);
            }
        }
        let total = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        total
    })
}

/// Sphere stack (sample SphereStack, capped for the browser).
#[wasm_bindgen]
pub fn sim_reset_sphere_stack(count: u32) -> u32 {
    let n = count.clamp(1, 20);
    SIM.with(|cell| {
        let mut sim = SimState {
            world: new_world(),
            bodies: Vec::new(),
            grab: MouseGrab::default(),
        };
        add_ground(&mut sim, 15.0);
        let r = 0.45f32;
        let mut y = 1.5 * r;
        for _ in 0..n {
            push_dynamic_sphere(&mut sim, 0.0, y, 0.0, r, 1.0);
            y += 2.0 * r + 0.05;
        }
        let total = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        total
    })
}

/// Advance the simulation. Returns body count.
#[wasm_bindgen]
pub fn sim_step(dt: f32, sub_steps: i32) -> u32 {
    with_sim(|sim| {
        sim.grab.pre_step(&mut sim.world, dt);
        sim.world.step(dt, sub_steps);
        sim.bodies.len() as u32
    })
}

/// Interleaved body poses: for each body
/// `[px, py, pz, qx, qy, qz, qw, hx, hy, hz, kind]`.
#[wasm_bindgen]
pub fn sim_body_poses() -> Vec<f32> {
    with_sim(|sim| {
        let mut out = Vec::with_capacity(sim.bodies.len() * 11);
        for b in &sim.bodies {
            let xf = get_body_transform(&sim.world, b.body_index);
            let (px, py, pz, qx, qy, qz, qw) = if let Some(local) = b.local {
                // WorldTransform × local Transform for compound children.
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

/// Body count in the current scene.
#[wasm_bindgen]
pub fn sim_body_count() -> u32 {
    with_sim(|sim| sim.bodies.len() as u32)
}

/// Begin mouse grab along a pick ray.
/// Returns `[grabbed, px, py, pz]` (grabbed is 0/1; point is the hit when grabbed).
#[wasm_bindgen]
pub fn sim_mouse_down(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> Vec<f32> {
    with_sim(|sim| {
        if sim
            .grab
            .begin(&mut sim.world, interact::pos(ox, oy, oz), interact::vec3(tx, ty, tz))
        {
            let p = sim.grab.mouse_point;
            vec![1.0, p.x as f32, p.y as f32, p.z as f32]
        } else {
            vec![0.0, 0.0, 0.0, 0.0]
        }
    })
}

/// Move the grab target to a world-space point (camera-facing plane from JS).
#[wasm_bindgen]
pub fn sim_mouse_move(px: f32, py: f32, pz: f32) {
    with_sim(|sim| {
        sim.grab.move_to(interact::pos(px, py, pz));
    })
}

/// Release the mouse grab (body keeps velocity → fling).
#[wasm_bindgen]
pub fn sim_mouse_up() {
    with_sim(|sim| {
        sim.grab.end(&mut sim.world);
    })
}

/// True if a grab is active.
#[wasm_bindgen]
pub fn sim_mouse_active() -> bool {
    with_sim(|sim| sim.grab.is_active())
}

/// Shift-click spawn: random sphere/box/capsule along the pick ray.
/// Returns `[ok, body_index, hx, hy, hz, kind]` (ok is 0/1).
#[wasm_bindgen]
pub fn sim_spawn_random(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> Vec<f32> {
    with_sim(|sim| {
        match interact::spawn_random(
            &mut sim.world,
            interact::pos(ox, oy, oz),
            interact::vec3(tx, ty, tz),
        ) {
            Some(spawned) => {
                sim.bodies.push(SimBody {
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

/// Ctrl-click delete: destroy the dynamic body under the pick ray. Returns 1 on success.
#[wasm_bindgen]
pub fn sim_delete_at_ray(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> u32 {
    with_sim(|sim| {
        let index = interact::delete_at_ray(
            &mut sim.world,
            &mut sim.grab,
            interact::pos(ox, oy, oz),
            interact::vec3(tx, ty, tz),
        );
        if index < 0 {
            return 0;
        }
        sim.bodies.retain(|b| b.body_index != index);
        1
    })
}

/// Counters: `[body, shape, contact, joint, island, awake, sleeping]`
#[wasm_bindgen]
pub fn sim_counters() -> Vec<f32> {
    with_sim(|sim| interact::counters_with_sleep(&sim.world).to_vec())
}

/// Debug-draw geometry for the current flags bitmask. See `interact::DRAW_*`.
#[wasm_bindgen]
pub fn sim_debug_draw(flags: u32) -> Vec<f32> {
    with_sim(|sim| interact::collect_debug_draw(&mut sim.world, flags))
}

/// Destroy a tracked body by render-list index (unused by UI; available for tests).
#[wasm_bindgen]
pub fn sim_destroy_body_index(body_index: i32) -> u32 {
    with_sim(|sim| {
        sim.grab.end(&mut sim.world);
        if !sim.bodies.iter().any(|b| b.body_index == body_index) {
            return 0;
        }
        let id = make_body_id(&sim.world, body_index);
        destroy_body(&mut sim.world, id);
        sim.bodies.retain(|b| b.body_index != body_index);
        1
    })
}
