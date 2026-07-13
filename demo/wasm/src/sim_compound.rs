//! Compound gallery scenes for the sim demos (Simple / Spheres / Hulls / Tile
//! Floor / Mesh Tile / Village).
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

use crate::rng::XorShift32;
use crate::sim_demo::{
    capsule_local_from_centers, new_sim, push_dynamic_sphere, stop_recording_if_any, DemoRng,
    SimBody, SIM,
};
use box3d_rust::body::{create_body, get_body_transform};
use box3d_rust::compound::{
    create_compound, CompoundDef, CompoundHullDef, CompoundMeshDef, CompoundSphereDef,
};
use box3d_rust::geometry::{default_surface_material, Sphere};
use box3d_rust::hull::make_box_hull;
use box3d_rust::math_functions::{
    make_quat_from_axis_angle, mul_transforms, Pos, Quat, Transform, Vec3, QUAT_IDENTITY,
    VEC3_AXIS_Y, VEC3_ZERO,
};
use box3d_rust::mesh::create_box_mesh;
use box3d_rust::shape::create_compound_shape;
use box3d_rust::types::{default_body_def, default_shape_def, BodyType};
use box3d_rust::world::world_set_contact_recycle_distance;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

/// Static compound-tile scene data (Tile Floor / Mesh Tile). Tile Floor renders
/// its 2500 static child hulls as a single Three.js `InstancedMesh` (the ground
/// is static, so the world-space child transforms are baked once at reset);
/// Mesh Tile's four tiles flow through the normal pose loop as `SimBody` boxes and
/// leave `transforms` empty. `stats` mirrors the C `DrawTextLine` overlay in the
/// village layout `[capsules, hulls, meshes, spheres, byte_count, tree_bytes,
/// tree_height]` so the browser reuses `formatVillageStats`.
#[derive(Default)]
struct TileSceneData {
    transforms: Vec<Transform>,
    half: [f32; 3],
    stats: [f32; 7],
}

thread_local! {
    static TILE_SCENE: RefCell<TileSceneData> = RefCell::new(TileSceneData::default());
    /// Compound Village character mover + sweeping query visualization (C Village
    /// embeds a `CharacterMover`). `None` unless the Village scene is active.
    static VILLAGE_MOVER: RefCell<Option<crate::village::VillageMover>> =
        const { RefCell::new(None) };
}

/// Uniform random quaternion (Shoemake), a bit-for-bit port of C `RandomQuat`
/// (`utils.h`:113): consume `u1 ∈ [0,1]`, `u2`/`u3 ∈ [0, 2π)` from the shared
/// `g_randomSeed` XorShift stream in that order. The Shoemake math lives in
/// `vis::random_quat_from`.
fn random_quat(rng: &mut XorShift32) -> Quat {
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

        // C `CompoundSpheres` (:112) resets `g_randomSeed = 12345` in the Sample
        // ctor, then per sphere consumes `RandomVec3` (3 floats) for the center and
        // `RandomFloatRange` (1 float) for the radius, in that order (:126-127).
        let mut rng = XorShift32::with_seed(12345);
        let h = 10.0f32;
        let material = default_surface_material();
        let mut spheres = Vec::with_capacity(20);
        for _ in 0..20 {
            let center = rng.vec3(
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

        // C `CompoundHulls` (:173) resets `g_randomSeed = 12345` in the Sample ctor,
        // then per hull consumes, in order (:189-197): three `RandomFloatRange` for
        // the extents (x,y,z), `RandomVec3` (3 floats) for the position, and
        // `RandomQuat` (3 floats) for the orientation.
        let mut rng = XorShift32::with_seed(12345);
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
            let p = rng.vec3(
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

/// Compound / Tile Floor â€” C `sample_compound.cpp` TileFloor (:244). A single
/// compound of `gridCount² = 2500` box hulls (extents `{4,2,4}`, random y jitter)
/// on a tilted static ground, with one dynamic sphere dropped from `{3,12,0}`.
///
/// Faithful to the C **release** count (gridCount 50 → 2500 hulls). The 2500
/// static tiles are baked to world space once and rendered as an instanced mesh
/// (see [`sim_tile_transforms`]); only the sphere is a live `SimBody`.
#[wasm_bindgen]
pub fn sim_reset_tile_floor() -> u32 {
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();

        const GRID_COUNT: i32 = 50;
        let a = 4.0f32;
        let box_hull = make_box_hull(a, 0.5 * a, a);
        let material = default_surface_material();

        let mut rng = DemoRng(0x7113_F100);
        let mut transforms: Vec<Transform> = Vec::with_capacity((GRID_COUNT * GRID_COUNT) as usize);
        for i in 0..GRID_COUNT {
            let x = (2.0 * i as f32 - GRID_COUNT as f32) * a;
            for j in 0..GRID_COUNT {
                let z = (2.0 * j as f32 - GRID_COUNT as f32) * a;
                let y = rng.range(-0.5, 0.25) * a;
                transforms.push(Transform {
                    p: Vec3 { x, y, z },
                    q: QUAT_IDENTITY,
                });
            }
        }

        let hulls: Vec<CompoundHullDef<'_>> = transforms
            .iter()
            .map(|xf| CompoundHullDef {
                hull: &box_hull.base,
                transform: *xf,
                material,
            })
            .collect();

        let compound = create_compound(&CompoundDef {
            hulls: &hulls,
            ..Default::default()
        })
        .expect("tile floor compound");

        let stats = [
            compound.capsule_count as f32,
            compound.hull_count as f32,
            compound.mesh_count as f32,
            compound.sphere_count as f32,
            compound.byte_count as f32,
            compound.tree.byte_count() as f32,
            compound.tree.height() as f32,
        ];

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Static;
        body_def.position = Pos {
            x: (-2.0) as _,
            y: 1.0 as _,
            z: (-3.0) as _,
        };
        // C rotates the ground about normalize({1,-1,0.5}) by angle 0 â€” identity.
        body_def.rotation = QUAT_IDENTITY;
        let ground = create_body(&mut sim.world, &body_def);
        create_compound_shape(&mut sim.world, ground, &default_shape_def(), &compound);

        // Bake tile world transforms (parent Ã— local); the ground is static.
        let parent = get_body_transform(&sim.world, ground.index1 - 1);
        let parent_xf = Transform {
            p: Vec3 {
                x: parent.p.x as f32,
                y: parent.p.y as f32,
                z: parent.p.z as f32,
            },
            q: parent.q,
        };
        let world_tiles: Vec<Transform> = transforms
            .iter()
            .map(|local| mul_transforms(parent_xf, *local))
            .collect();
        TILE_SCENE.with(|t| {
            *t.borrow_mut() = TileSceneData {
                transforms: world_tiles,
                half: [a, 0.5 * a, a],
                stats,
            };
        });

        // C TileFloor :311 drops one dynamic sphere from {3, 12, 0}, r 0.25.
        push_dynamic_sphere(&mut sim, 3.0, 12.0, 0.0, 0.25, 0.0);

        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

/// Compound / Mesh Tile â€” C `sample_compound.cpp` MeshTile (:362). A compound of
/// `gridCount² = 4` box **meshes** (`b3CreateBoxMesh`, extents `{4,2,4}`) on a
/// ground body at the origin. The C dynamic sphere drop is `#if 0`'d, so the scene
/// is fully static. The four tiles render through the normal pose loop as boxes.
#[wasm_bindgen]
pub fn sim_reset_mesh_tile() -> u32 {
    SIM.with(|cell| {
        if let Some(prev) = cell.borrow_mut().as_mut() {
            stop_recording_if_any(prev);
        }
        let mut sim = new_sim();

        const GRID_COUNT: i32 = 2;
        let a = 4.0f32;
        let extents = Vec3 {
            x: a,
            y: 0.5 * a,
            z: a,
        };
        let material = default_surface_material();
        let box_mesh = create_box_mesh(VEC3_ZERO, extents, true).expect("box mesh");

        let mut rng = DemoRng(0x1E5A_7113);
        let materials = [material];
        let mut local_transforms: Vec<Transform> =
            Vec::with_capacity((GRID_COUNT * GRID_COUNT) as usize);
        for i in 0..GRID_COUNT {
            let x = (2.0 * i as f32 - GRID_COUNT as f32) * a;
            for j in 0..GRID_COUNT {
                let z = (2.0 * j as f32 - GRID_COUNT as f32) * a;
                let y = rng.range(-0.5, 0.25) * a;
                local_transforms.push(Transform {
                    p: Vec3 { x, y, z },
                    q: QUAT_IDENTITY,
                });
            }
        }

        let meshes: Vec<CompoundMeshDef<'_>> = local_transforms
            .iter()
            .map(|xf| CompoundMeshDef {
                mesh_data: &box_mesh,
                transform: *xf,
                scale: Vec3 {
                    x: 1.0,
                    y: 1.0,
                    z: 1.0,
                },
                materials: &materials,
            })
            .collect();

        let compound = create_compound(&CompoundDef {
            meshes: &meshes,
            ..Default::default()
        })
        .expect("mesh tile compound");

        let stats = [
            compound.capsule_count as f32,
            compound.hull_count as f32,
            compound.mesh_count as f32,
            compound.sphere_count as f32,
            compound.byte_count as f32,
            compound.tree.byte_count() as f32,
            compound.tree.height() as f32,
        ];

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Static;
        let ground = create_body(&mut sim.world, &body_def);
        create_compound_shape(&mut sim.world, ground, &default_shape_def(), &compound);

        let parent_index = ground.index1 - 1;
        for xf in &local_transforms {
            sim.bodies.push(SimBody {
                body_index: parent_index,
                half_extents: [a, 0.5 * a, a],
                kind: 0,
                local: Some(*xf),
            });
        }

        // Mesh Tile renders its tiles through the pose loop; no instanced tiles.
        TILE_SCENE.with(|t| {
            *t.borrow_mut() = TileSceneData {
                transforms: Vec::new(),
                half: [a, 0.5 * a, a],
                stats,
            };
        });

        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

/// Baked world-space Tile Floor child transforms `[px,py,pz, qx,qy,qz,qw] * N`
/// (empty for Mesh Tile, which renders through the pose loop). All tiles share the
/// half-extents from [`sim_tile_half`].
#[wasm_bindgen]
pub fn sim_tile_transforms() -> Vec<f32> {
    TILE_SCENE.with(|t| {
        let data = t.borrow();
        let mut out = Vec::with_capacity(data.transforms.len() * 7);
        for xf in &data.transforms {
            out.push(xf.p.x);
            out.push(xf.p.y);
            out.push(xf.p.z);
            out.push(xf.q.v.x);
            out.push(xf.q.v.y);
            out.push(xf.q.v.z);
            out.push(xf.q.s);
        }
        out
    })
}

/// Uniform tile half-extents `[hx, hy, hz]` for the instanced Tile Floor mesh.
#[wasm_bindgen]
pub fn sim_tile_half() -> Vec<f32> {
    TILE_SCENE.with(|t| t.borrow().half.to_vec())
}

/// Compound tile-scene stats `[capsules, hulls, meshes, spheres, byte_count,
/// tree_bytes, tree_height]` (same layout as [`sim_village_stats`]).
#[wasm_bindgen]
pub fn sim_tile_stats() -> Vec<f32> {
    TILE_SCENE.with(|t| t.borrow().stats.to_vec())
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

        // C Village :502 sets launchSpeedScale = 2 (bullets spawn at half speed).
        crate::interact::set_launch_speed_scale(2.0);

        // C Village embeds a CharacterMover (start {0,10,0}) plus a sweeping
        // ray/shape/overlap query visualization; `m_worldWidth = 2*gridCount*a`.
        let world_width = 2.0 * grid as f32 * a;
        VILLAGE_MOVER.with(|m| {
            *m.borrow_mut() = Some(crate::village::VillageMover::new(
                Pos {
                    x: 0.0 as _,
                    y: 10.0 as _,
                    z: 0.0 as _,
                },
                world_width,
            ));
        });

        let count = sim.bodies.len() as u32;
        *cell.borrow_mut() = Some(sim);
        count
    })
}

/// Feed the Village mover WASD throttle / jump / sprint and camera-relative axes.
#[wasm_bindgen]
#[allow(clippy::too_many_arguments)]
pub fn sim_village_set_input(
    throttle_x: f32,
    throttle_y: f32,
    jump: bool,
    sprint: bool,
    fwd_x: f32,
    fwd_z: f32,
    right_x: f32,
    right_z: f32,
) {
    VILLAGE_MOVER.with(|m| {
        if let Some(mover) = m.borrow_mut().as_mut() {
            mover.set_input(
                throttle_x, throttle_y, jump, sprint, fwd_x, fwd_z, right_x, right_z,
            );
        }
    });
}

/// Integrate the Village mover one step against the sim world (does not step the
/// world — call before `sim_step`, mirroring C's `mover.Step` before `Sample::Step`).
#[wasm_bindgen]
pub fn sim_village_mover_step(dt: f32) {
    VILLAGE_MOVER.with(|m| {
        let mut slot = m.borrow_mut();
        let Some(mover) = slot.as_mut() else {
            return;
        };
        SIM.with(|cell| {
            if let Some(sim) = cell.borrow_mut().as_mut() {
                mover.solve_move(&mut sim.world, dt);
            }
        });
    });
}

/// Advance the Village sweeping query one step (ray/shape/overlap visualization).
/// Pass `dt = 0` when paused so the sweep freezes (matching C).
#[wasm_bindgen]
pub fn sim_village_query_step(dt: f32) {
    VILLAGE_MOVER.with(|m| {
        let mut slot = m.borrow_mut();
        let Some(mover) = slot.as_mut() else {
            return;
        };
        SIM.with(|cell| {
            if let Some(sim) = cell.borrow().as_ref() {
                mover.query(&sim.world, dt);
            }
        });
    });
}

/// Village mover capsule pose: `[px,py,pz, c1x,c1y,c1z, c2x,c2y,c2z, radius]`
/// (empty when the Village scene is not active).
#[wasm_bindgen]
pub fn sim_village_mover_pose() -> Vec<f32> {
    VILLAGE_MOVER.with(|m| {
        m.borrow()
            .as_ref()
            .map(|mover| {
                vec![
                    mover.mover_pos.x as f32,
                    mover.mover_pos.y as f32,
                    mover.mover_pos.z as f32,
                    mover.capsule.center1.x,
                    mover.capsule.center1.y,
                    mover.capsule.center1.z,
                    mover.capsule.center2.x,
                    mover.capsule.center2.y,
                    mover.capsule.center2.z,
                    mover.capsule.radius,
                ]
            })
            .unwrap_or_default()
    })
}

/// Village sweeping-query visualization (see `village::VillageMover::query` for the
/// 39-float layout: ray, sphere shape cast, and overlap-shape data).
#[wasm_bindgen]
pub fn sim_village_query() -> Vec<f32> {
    VILLAGE_MOVER.with(|m| {
        m.borrow()
            .as_ref()
            .map(|mover| mover.query_viz().to_vec())
            .unwrap_or_default()
    })
}

/// Toggle the Village third-person camera flag (C 'T' key). Returns the new state.
#[wasm_bindgen]
pub fn sim_village_toggle_third_person() -> bool {
    VILLAGE_MOVER.with(|m| {
        if let Some(mover) = m.borrow_mut().as_mut() {
            mover.toggle_third_person();
            mover.third_person
        } else {
            false
        }
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
