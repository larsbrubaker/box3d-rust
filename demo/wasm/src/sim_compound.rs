//! Compound gallery scenes for the sim demos (Simple / Spheres / Hulls / Village).
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::sim_demo::{
    capsule_local_from_centers, new_sim, push_dynamic_sphere, stop_recording_if_any, DemoRng,
    SimBody, SIM,
};
use box3d_rust::body::{create_body, get_body_transform};
use box3d_rust::compound::{create_compound, CompoundDef, CompoundHullDef, CompoundSphereDef};
use box3d_rust::geometry::{default_surface_material, Sphere};
use box3d_rust::hull::make_box_hull;
use box3d_rust::math_functions::{
    make_quat_from_axis_angle, mul_transforms, Pos, Quat, Transform, Vec3, QUAT_IDENTITY,
    VEC3_AXIS_Y,
};
use box3d_rust::shape::create_compound_shape;
use box3d_rust::types::{default_body_def, default_shape_def, BodyType};
use box3d_rust::world::world_set_contact_recycle_distance;
use wasm_bindgen::prelude::*;

/// Uniform random quaternion (Shoemake), fed by the demo-local LCG rather than the C
/// `Random*` stream. The Shoemake math lives in `vis::random_quat_from`.
fn random_quat(rng: &mut DemoRng) -> Quat {
    let u1 = rng.range(0.0, 1.0);
    let u2 = rng.range(0.0, 2.0 * std::f32::consts::PI);
    let u3 = rng.range(0.0, 2.0 * std::f32::consts::PI);
    crate::vis::random_quat_from(u1, u2, u3)
}

/// Compound / Simple â€” matches `sample_compound.cpp` SimpleCompound (single hull + sphere).
#[wasm_bindgen]
pub fn sim_reset_compound() -> u32 {
    sim_reset_compound_simple()
}

/// Named Simple compound gallery sample (C: Compound / Simple).
#[wasm_bindgen]
pub fn sim_reset_compound_simple() -> u32 {
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();

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

        // C SimpleCompound :57 disables contact recycling so the dropped sphere
        // settles cleanly on the tilted compound hull.
        world_set_contact_recycle_distance(&mut sim.world, 0.0);

        // C :65 uses b3DefaultShapeDef() for the sphere (no rolling resistance).
        push_dynamic_sphere(&mut sim, 0.0, 2.0, 0.0, 0.25, 0.0);

        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

/// Compound / Spheres â€” cloud of compound spheres (C: Compound / Spheres, count 20).
#[wasm_bindgen]
pub fn sim_reset_compound_spheres() -> u32 {
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();

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
                Vec3 { x: h, y: h, z: h },
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

/// Compound / Hulls â€” cloud of compound box hulls (C: Compound / Hulls, count 20).
#[wasm_bindgen]
pub fn sim_reset_compound_hulls() -> u32 {
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();

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
            // C CompoundHulls :196-197 uses RandomVec3 for position and RandomQuat()
            // (Shoemake uniform random rotation) for orientation.
            let p = rng.vec3_range(
                Vec3 {
                    x: -h,
                    y: -h,
                    z: -h,
                },
                Vec3 { x: h, y: h, z: h },
            );
            transforms.push(Transform {
                p,
                q: random_quat(&mut rng),
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

/// Compound / Village â€” C `sample_compound.cpp` Village with real `building.obj` meshes.
///
/// C uses `gridCount = 8` (debug) / `200` (release). 200 buildings Ã— compound is
/// far too heavy for the serial wasm build, so this fixes the grid at the C debug
/// value 8. The C sample has no grid control, so this takes no argument.
#[wasm_bindgen]
pub fn sim_reset_village() -> u32 {
    // C Village debug build uses gridCount = 8 (release uses 200).
    let grid = 8i32;
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();
        let village = crate::village::build_village(&mut sim.world, grid);
        let a = village.tile_half;
        let parent_index = village.ground_body_index;

        for xf in &village.hull_transforms {
            sim.bodies.push(SimBody {
                body_index: parent_index,
                half_extents: [a, 0.5 * a, a],
                kind: 0,
                local: Some(*xf),
            });
        }
        for s in &village.spheres {
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
        for c in &village.capsules {
            let (local, half) =
                capsule_local_from_centers(c.capsule.center1, c.capsule.center2, c.capsule.radius);
            sim.bodies.push(SimBody {
                body_index: parent_index,
                half_extents: half,
                kind: 2,
                local: Some(local),
            });
        }

        sim.village_buildings = village.buildings;
        sim.village_stats = village.stats;
        sim.village_ground_index = parent_index;

        // C Village is fully static (a character mover + query sweep visualization
        // walks it). No dynamic drop-ins here.

        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

/// Village building instances in world space:
/// `[px,py,pz, qx,qy,qz,qw, sx,sy,sz] * N` (matches C compound mesh children).
#[wasm_bindgen]
pub fn sim_village_buildings() -> Vec<f32> {
    SIM.with(|cell| {
        let slot = cell.borrow();
        let Some(sim) = slot.as_ref() else {
            return Vec::new();
        };
        if sim.village_ground_index < 0 || sim.village_buildings.is_empty() {
            return Vec::new();
        }
        let parent = get_body_transform(&sim.world, sim.village_ground_index);
        let parent_xf = Transform {
            p: Vec3 {
                x: parent.p.x as f32,
                y: parent.p.y as f32,
                z: parent.p.z as f32,
            },
            q: parent.q,
        };
        let mut out = Vec::with_capacity(sim.village_buildings.len() * 10);
        for b in &sim.village_buildings {
            let world_xf = mul_transforms(parent_xf, b.transform);
            out.push(world_xf.p.x);
            out.push(world_xf.p.y);
            out.push(world_xf.p.z);
            out.push(world_xf.q.v.x);
            out.push(world_xf.q.v.y);
            out.push(world_xf.q.v.z);
            out.push(world_xf.q.s);
            out.push(b.scale.x);
            out.push(b.scale.y);
            out.push(b.scale.z);
        }
        out
    })
}

/// Village compound stats: `[capsules, hulls, meshes, spheres, byte_count, tree_bytes, tree_height]`.
#[wasm_bindgen]
pub fn sim_village_stats() -> Vec<f32> {
    SIM.with(|cell| {
        cell.borrow()
            .as_ref()
            .map(|s| s.village_stats.to_vec())
            .unwrap_or_default()
    })
}
