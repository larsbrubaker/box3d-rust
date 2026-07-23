//! Determinism scene helpers from `shared/determinism.c`.
//!
//! Builds the falling-ragdoll grid over mesh grounds, the convex wave pile, and the
//! query-driven spawn scene, hashing settled transforms. These feed the
//! `test_determinism.c` cross-platform gate.
//!
//! The content hash itself (`b3Hash` / [`crate::core::hash`]) already lives in
//! [`crate::core`]; this module adds the world-transform byte layout used by
//! the scenes.
//!
//! SPDX-FileCopyrightText: 2022 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)] // Pos is f64 under double-precision

use crate::body::{body_get_transform, create_body};
use crate::core::{hash, HASH_INIT};
use crate::height_field::{create_wave, HeightFieldData};
use crate::hull::{create_rock, make_box_hull};
use crate::human::{
    create_human, random_float_range, random_pos, random_quat, random_unit_vector,
    random_vec3_uniform, set_random_seed, Human, BONE_COUNT,
};
use crate::id::{BodyId, ShapeId};
use crate::geometry::{Capsule, Sphere};
use crate::math_functions::{
    mul_sv, offset_pos, Aabb, Pos, Vec3, WorldTransform, POS_ZERO, VEC3_ONE, VEC3_ZERO,
};
use crate::mesh::{create_grid_mesh, create_torus_mesh, create_wave_mesh, destroy_mesh, MeshData};
use crate::shape::{
    create_capsule_shape, create_height_field_shape, create_hull_shape, create_mesh_shape,
    create_sphere_shape,
};
use crate::types::{default_body_def, default_query_filter, default_shape_def, BodyType, Filter};
use crate::world::{
    world_cast_ray_closest, world_cast_shape, world_get_awake_body_count, world_get_body_events,
    world_overlap_aabb, world_set_gravity, World,
};

/// Humans per ground cell. (RAGDOLL_GROUP_SIZE)
pub const RAGDOLL_GROUP_SIZE: usize = 2;

/// Grid dimension (NxN cells). (RAGDOLL_GRID_COUNT)
pub const RAGDOLL_GRID_COUNT: usize = 2;

const GRID_SIZE: f32 = 15.0;

/// One cell of ragdolls. (RagdollGroup)
#[derive(Debug, Clone, Default)]
pub struct RagdollGroup {
    pub humans: [Human; RAGDOLL_GROUP_SIZE],
}

/// Falling-ragdoll scene state. (FallingRagdollData)
#[derive(Debug)]
pub struct FallingRagdollData {
    pub groups: [RagdollGroup; RAGDOLL_GRID_COUNT * RAGDOLL_GRID_COUNT],
    pub grid_mesh: Option<MeshData>,
    pub torus_mesh: Option<MeshData>,
    pub column_count: i32,
    pub column_index: i32,
    pub step_count: i32,
    pub sleep_step: i32,
    pub hash: u32,
}

impl Default for FallingRagdollData {
    fn default() -> Self {
        FallingRagdollData {
            groups: std::array::from_fn(|_| RagdollGroup::default()),
            grid_mesh: None,
            torus_mesh: None,
            column_count: 0,
            column_index: 0,
            step_count: 0,
            sleep_step: 0,
            hash: 0,
        }
    }
}

/// Hash a world transform's raw little-endian field bytes in C declaration
/// order, matching `b3Hash(hash, (uint8_t*)&xf, sizeof(b3WorldTransform))` on
/// little-endian hosts. (determinism.c UpdateFallingRagdolls)
pub fn hash_world_transform(h: u32, xf: &WorldTransform) -> u32 {
    let mut bytes = Vec::with_capacity(40);
    bytes.extend_from_slice(&xf.p.x.to_le_bytes());
    bytes.extend_from_slice(&xf.p.y.to_le_bytes());
    bytes.extend_from_slice(&xf.p.z.to_le_bytes());
    bytes.extend_from_slice(&xf.q.v.x.to_le_bytes());
    bytes.extend_from_slice(&xf.q.v.y.to_le_bytes());
    bytes.extend_from_slice(&xf.q.v.z.to_le_bytes());
    bytes.extend_from_slice(&xf.q.s.to_le_bytes());
    hash(h, &bytes)
}

fn create_group(
    data: &mut FallingRagdollData,
    world: &mut World,
    row_index: usize,
    column_index: usize,
) {
    debug_assert!(row_index < RAGDOLL_GRID_COUNT && column_index < RAGDOLL_GRID_COUNT);

    let group_index = row_index * RAGDOLL_GRID_COUNT + column_index;

    let span = RAGDOLL_GRID_COUNT as f32 * GRID_SIZE;
    let group_distance = 1.0 * span / RAGDOLL_GRID_COUNT as f32;

    let mut position = Pos {
        x: (-0.5 * span + group_distance * (column_index as f32 + 0.5)) as _,
        y: 15.0 as _,
        z: (-0.5 * span + group_distance * (row_index as f32 + 0.5)) as _,
    };

    let friction_torque = 5.0;
    let hertz = 1.0;
    let damping_ratio = 0.7;
    let colorize = false;

    for i in 0..RAGDOLL_GROUP_SIZE {
        let human = &mut data.groups[group_index].humans[i];
        create_human(
            human,
            world,
            position,
            friction_torque,
            hertz,
            damping_ratio,
            group_index as i32,
            0,
            colorize,
        );
        position.x = (position.x as f32 + 0.75) as _;
    }
}

/// Build mesh grounds and spawn the ragdoll grid. (CreateFallingRagdolls)
pub fn create_falling_ragdolls(world: &mut World) -> FallingRagdollData {
    let mut data = FallingRagdollData::default();

    let half_mesh_grid_rows = 4;
    let mesh_grid_cell_width = GRID_SIZE / (2.0 * half_mesh_grid_rows as f32);
    data.grid_mesh = create_grid_mesh(
        2 * half_mesh_grid_rows,
        2 * half_mesh_grid_rows,
        mesh_grid_cell_width,
        0,
        true,
    );
    data.torus_mesh = create_torus_mesh(16, 16, 0.25 * GRID_SIZE, 1.0);

    let span = GRID_SIZE * RAGDOLL_GRID_COUNT as f32;
    let mut body_def = default_body_def();
    let shape_def = default_shape_def();

    // Clone mesh data for shape attach; FallingRagdollData keeps the originals
    // for DestroyFallingRagdolls (C keeps borrowed pointers).
    let grid_mesh = data.grid_mesh.as_ref().expect("grid mesh").clone();
    let torus_mesh = data.torus_mesh.as_ref().expect("torus mesh").clone();

    body_def.position.x = (-0.5 * span + 0.5 * GRID_SIZE) as _;
    for i in 0..RAGDOLL_GRID_COUNT {
        body_def.position.z = (-0.5 * span + 0.5 * GRID_SIZE) as _;
        for j in 0..RAGDOLL_GRID_COUNT {
            let body = create_body(world, &body_def);
            create_mesh_shape(world, body, &shape_def, &grid_mesh, VEC3_ONE);
            create_mesh_shape(world, body, &shape_def, &torus_mesh, VEC3_ONE);

            create_group(&mut data, world, i, j);

            body_def.position.z = (body_def.position.z as f32 + GRID_SIZE) as _;
        }

        body_def.position.x = (body_def.position.x as f32 + GRID_SIZE) as _;
    }

    data
}

/// Advance sleep/hash tracking after a world step. Returns true once settled.
/// (UpdateFallingRagdolls)
pub fn update_falling_ragdolls(world: &World, data: &mut FallingRagdollData) -> bool {
    if data.hash == 0 {
        let body_events = world_get_body_events(world);

        if body_events.is_empty() {
            let awake_count = world_get_awake_body_count(world);
            debug_assert!(awake_count == 0);

            data.hash = HASH_INIT;
            for i in 0..RAGDOLL_GRID_COUNT {
                for j in 0..RAGDOLL_GRID_COUNT {
                    for k in 0..RAGDOLL_GROUP_SIZE {
                        let group_index = i * RAGDOLL_GRID_COUNT + j;
                        let human = &data.groups[group_index].humans[k];

                        for b in 0..BONE_COUNT {
                            let body_id = human.bones[b].body_id;
                            let xf = body_get_transform(world, body_id);
                            data.hash = hash_world_transform(data.hash, &xf);
                        }
                    }
                }
            }

            data.sleep_step = data.step_count;
        }
    }

    data.step_count += 1;

    data.hash != 0
}

/// Release owned mesh data. (DestroyFallingRagdolls)
pub fn destroy_falling_ragdolls(data: &mut FallingRagdollData) {
    if let Some(mesh) = data.grid_mesh.take() {
        destroy_mesh(mesh);
    }
    if let Some(mesh) = data.torus_mesh.take() {
        destroy_mesh(mesh);
    }
}

// -------------------------------------------------------------------------------------------------
// Wave pile scene
// -------------------------------------------------------------------------------------------------

/// Convex bodies dropped on a wave height field. (WAVE_PILE_BODY_COUNT)
pub const WAVE_PILE_BODY_COUNT: usize = 100;

const WAVE_PILE_GRID: i32 = 5;
const WAVE_PILE_LAYERS: i32 = 4;

/// Convex-pile-on-wave scene state. (WavePileData)
#[derive(Debug)]
pub struct WavePileData {
    pub bodies: [BodyId; WAVE_PILE_BODY_COUNT],
    pub height_field: Option<HeightFieldData>,
    pub step_count: i32,
    pub sleep_step: i32,
    pub hash: u32,
}

impl Default for WavePileData {
    fn default() -> Self {
        WavePileData {
            bodies: [BodyId::default(); WAVE_PILE_BODY_COUNT],
            height_field: None,
            step_count: 0,
            sleep_step: 0,
            hash: 0,
        }
    }
}

/// Drop a mixed convex pile on a wave height field. (CreateWavePile)
pub fn create_wave_pile(world: &mut World) -> WavePileData {
    let mut data = WavePileData::default();

    set_random_seed(52977);

    // Height fields grow from a corner, offset the body to center the patch.
    let field_count = 21;
    let field_scale = Vec3 {
        x: 1.0,
        y: 0.6,
        z: 1.0,
    };
    let height_field = create_wave(field_count, field_count, field_scale, 0.08, 0.06, false);

    {
        let extent = field_scale.x * (field_count - 1) as f32;
        let mut body_def = default_body_def();
        body_def.position.x = (-0.5 * extent) as _;
        body_def.position.z = (-0.5 * extent) as _;
        let ground_id = create_body(world, &body_def);

        let shape_def = default_shape_def();
        create_height_field_shape(world, ground_id, &shape_def, &height_field);
    }

    data.height_field = Some(height_field);

    let rock = create_rock(0.55).expect("rock hull");
    let box_hull = make_box_hull(0.45, 0.3, 0.55);
    let sphere = Sphere {
        center: VEC3_ZERO,
        radius: 0.5,
    };
    let capsule = Capsule {
        center1: Vec3 {
            x: 0.0,
            y: -0.3,
            z: 0.0,
        },
        center2: Vec3 {
            x: 0.0,
            y: 0.3,
            z: 0.0,
        },
        radius: 0.35,
    };

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;

    // Rolling resistance so the pile sleeps quickly.
    let mut shape_def = default_shape_def();
    shape_def.base_material.rolling_resistance = 0.3;

    let spacing = 1.7f32;
    let mut index = 0usize;
    for layer in 0..WAVE_PILE_LAYERS {
        for i in 0..WAVE_PILE_GRID {
            for j in 0..WAVE_PILE_GRID {
                let jitter = random_vec3_uniform(-0.3, 0.3);
                body_def.position.x =
                    (spacing * (i as f32 - 0.5 * (WAVE_PILE_GRID - 1) as f32) + jitter.x) as _;
                body_def.position.y = (2.5 + 1.6 * layer as f32 + 0.3 * jitter.y) as _;
                body_def.position.z =
                    (spacing * (j as f32 - 0.5 * (WAVE_PILE_GRID - 1) as f32) + jitter.z) as _;
                body_def.rotation = random_quat();

                let body_id = create_body(world, &body_def);
                data.bodies[index] = body_id;

                match index % 4 {
                    0 => {
                        create_sphere_shape(world, body_id, &shape_def, &sphere);
                    }
                    1 => {
                        create_capsule_shape(world, body_id, &shape_def, &capsule);
                    }
                    2 => {
                        create_hull_shape(world, body_id, &shape_def, &box_hull.base);
                    }
                    _ => {
                        create_hull_shape(world, body_id, &shape_def, &rock);
                    }
                }

                index += 1;
            }
        }
    }

    debug_assert!(index == WAVE_PILE_BODY_COUNT);

    // The world keeps its own copy (C calls b3DestroyHull on the rock here).
    data
}

/// Advance sleep/hash tracking for the wave pile. (UpdateWavePile)
pub fn update_wave_pile(world: &World, data: &mut WavePileData) -> bool {
    if data.hash == 0 && world_get_awake_body_count(world) == 0 {
        data.hash = HASH_INIT;
        for i in 0..WAVE_PILE_BODY_COUNT {
            let xf = body_get_transform(world, data.bodies[i]);
            data.hash = hash_world_transform(data.hash, &xf);
        }

        data.sleep_step = data.step_count;
    }

    data.step_count += 1;

    data.hash != 0
}

/// Release the owned height field. (DestroyWavePile)
pub fn destroy_wave_pile(data: &mut WavePileData) {
    data.height_field = None;
}

// -------------------------------------------------------------------------------------------------
// Query spawn scene
// -------------------------------------------------------------------------------------------------

/// Bodies spawned by query-driven feedback. (QUERY_SPAWN_COUNT)
pub const QUERY_SPAWN_COUNT: usize = 50;

/// Sphere-cast proxy radius. (QUERY_SPAWN_CAST_RADIUS)
pub const QUERY_SPAWN_CAST_RADIUS: f32 = 0.5;

/// Query-driven spawn scene state. (QuerySpawnData)
#[derive(Debug)]
pub struct QuerySpawnData {
    pub bodies: [BodyId; QUERY_SPAWN_COUNT],
    pub spawn_count: i32,
    pub query_hit_count: i32,
    pub query_hash: u32,
    pub step_count: i32,
    pub sleep_step: i32,
    pub hash: u32,

    // Last query cycle, recorded for visualization.
    pub ray_origin: Pos,
    pub ray_translation: Vec3,
    pub ray_point: Pos,
    pub ray_normal: Vec3,
    pub ray_did_hit: bool,
    pub overlap_bounds: Aabb,
    pub overlap_count: i32,
    pub cast_fraction: f32,
    pub last_spawn_position: Pos,
}

impl Default for QuerySpawnData {
    fn default() -> Self {
        QuerySpawnData {
            bodies: [BodyId::default(); QUERY_SPAWN_COUNT],
            spawn_count: 0,
            query_hit_count: 0,
            query_hash: 0,
            step_count: 0,
            sleep_step: 0,
            hash: 0,
            ray_origin: POS_ZERO,
            ray_translation: VEC3_ZERO,
            ray_point: POS_ZERO,
            ray_normal: VEC3_ZERO,
            ray_did_hit: false,
            overlap_bounds: Aabb {
                lower_bound: VEC3_ZERO,
                upper_bound: VEC3_ZERO,
            },
            overlap_count: 0,
            cast_fraction: 0.0,
            last_spawn_position: POS_ZERO,
        }
    }
}

/// Run one query cycle and spawn a shape whose position, type, and size depend on the
/// query results. (QuerySpawnOnce)
fn query_spawn_once(world: &mut World, data: &mut QuerySpawnData) {
    let filter = default_query_filter();

    // Ray result decides the spawn position, a miss seeds the cloud near the origin.
    let ray_origin = random_pos(
        Vec3 {
            x: -12.0,
            y: -12.0,
            z: -12.0,
        },
        Vec3 {
            x: 12.0,
            y: 12.0,
            z: 12.0,
        },
    );
    let ray_translation = mul_sv(30.0, random_unit_vector());

    let ray = world_cast_ray_closest(world, ray_origin, ray_translation, &filter);
    let spawn_position;
    if ray.hit {
        data.query_hit_count += 1;
        data.query_hash = hash(data.query_hash, &ray.fraction.to_le_bytes());
        data.query_hash = hash(data.query_hash, &vec3_bytes(ray.normal));
        spawn_position = offset_pos(ray.point, mul_sv(1.2, ray.normal));
    } else {
        spawn_position = random_pos(
            Vec3 {
                x: -6.0,
                y: -6.0,
                z: -6.0,
            },
            Vec3 {
                x: 6.0,
                y: 6.0,
                z: 6.0,
            },
        );
    }

    data.ray_origin = ray_origin;
    data.ray_translation = ray_translation;
    data.ray_did_hit = ray.hit;
    data.ray_point = if ray.hit {
        ray.point
    } else {
        offset_pos(ray_origin, ray_translation)
    };
    data.ray_normal = if ray.hit { ray.normal } else { VEC3_ZERO };

    // Overlap count picks the shape type.
    let center = random_vec3_uniform(-10.0, 10.0);
    let extent = random_float_range(1.0, 4.0);
    let aabb = Aabb {
        lower_bound: Vec3 {
            x: center.x - extent,
            y: center.y - extent,
            z: center.z - extent,
        },
        upper_bound: Vec3 {
            x: center.x + extent,
            y: center.y + extent,
            z: center.z + extent,
        },
    };

    let mut overlap_count = 0i32;
    world_overlap_aabb(world, aabb, &filter, |shape_id: ShapeId| {
        overlap_count += 1;
        data.query_hit_count += 1;
        data.query_hash = hash(data.query_hash, &shape_id.index1.to_le_bytes());
        true
    });

    data.overlap_bounds = aabb;
    data.overlap_count = overlap_count;

    // Sphere cast fraction sets the spawn size.
    let mut fraction = 1.0f32;
    let mut proxy = crate::distance::ShapeProxy::default();
    proxy.points[0] = VEC3_ZERO;
    proxy.count = 1;
    proxy.radius = QUERY_SPAWN_CAST_RADIUS;
    world_cast_shape(
        world,
        ray_origin,
        &proxy,
        ray_translation,
        &filter,
        |_shape_id, _point, _normal, cast_fraction, _user_material_id, _triangle_index, _child_index| {
            fraction = cast_fraction;
            cast_fraction
        },
    );
    if fraction < 1.0 {
        data.query_hit_count += 1;
        data.query_hash = hash(data.query_hash, &fraction.to_le_bytes());
    }

    data.cast_fraction = fraction;

    let size = 0.3 + 0.2 * fraction;

    // Damping guarantees everything comes to rest.
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = spawn_position;
    body_def.rotation = random_quat();
    body_def.linear_velocity = random_vec3_uniform(-0.2, 0.2);
    body_def.angular_velocity = random_vec3_uniform(-0.5, 0.5);
    body_def.linear_damping = 1.0;
    body_def.angular_damping = 1.0;

    let body_id = create_body(world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.base_material.rolling_resistance = 0.2;

    match (data.spawn_count + overlap_count) % 3 {
        0 => {
            let sphere = Sphere {
                center: VEC3_ZERO,
                radius: size,
            };
            create_sphere_shape(world, body_id, &shape_def, &sphere);
        }
        1 => {
            let capsule = Capsule {
                center1: Vec3 {
                    x: 0.0,
                    y: -size,
                    z: 0.0,
                },
                center2: Vec3 {
                    x: 0.0,
                    y: size,
                    z: 0.0,
                },
                radius: 0.7 * size,
            };
            create_capsule_shape(world, body_id, &shape_def, &capsule);
        }
        _ => {
            let box_hull = make_box_hull(size, 0.7 * size, 0.5 * size);
            create_hull_shape(world, body_id, &shape_def, &box_hull.base);
        }
    }

    data.bodies[data.spawn_count as usize] = body_id;
    data.spawn_count += 1;
    data.last_spawn_position = spawn_position;
}

/// Set up an empty zero-gravity world for the query spawn scene. (CreateQuerySpawn)
pub fn create_query_spawn(world: &mut World) -> QuerySpawnData {
    let data = QuerySpawnData::default();

    set_random_seed(71689);

    // Empty space, motion comes only from spawn velocities and overlap pushes.
    world_set_gravity(world, VEC3_ZERO);

    data
}

/// Spawn a body per step until the cloud is full, then settle and hash. (UpdateQuerySpawn)
pub fn update_query_spawn(world: &mut World, data: &mut QuerySpawnData) -> bool {
    if (data.spawn_count as usize) < QUERY_SPAWN_COUNT {
        query_spawn_once(world, data);
    } else if data.hash == 0 && world_get_awake_body_count(world) == 0 {
        data.hash = HASH_INIT;
        for i in 0..QUERY_SPAWN_COUNT {
            let xf = body_get_transform(world, data.bodies[i]);
            data.hash = hash_world_transform(data.hash, &xf);
        }

        data.sleep_step = data.step_count;
    }

    data.step_count += 1;

    data.hash != 0
}

/// No owned resources to release. (DestroyQuerySpawn)
pub fn destroy_query_spawn(_data: &mut QuerySpawnData) {}

// -------------------------------------------------------------------------------------------------
// Mesh drop scene (shared/stability.c)
// -------------------------------------------------------------------------------------------------

/// Grid dimension for the thin-box drop. (MESH_DROP_GRID_COUNT)
pub const MESH_DROP_GRID_COUNT: usize = 20;

/// Total thin boxes dropped on the wave mesh.
pub const MESH_DROP_BODY_COUNT: usize = MESH_DROP_GRID_COUNT * MESH_DROP_GRID_COUNT;

/// Thin fast boxes dropped on a wave mesh. Stresses continuous collision and mesh contact
/// stability, and doubles as a determinism scenario via the sleep hash. (MeshDropData)
#[derive(Debug)]
pub struct MeshDropData {
    pub mesh: Option<MeshData>,
    pub bodies: [BodyId; MESH_DROP_BODY_COUNT],
    pub step_count: i32,
    pub sleep_step: i32,
    pub hash: u32,
}

impl Default for MeshDropData {
    fn default() -> Self {
        MeshDropData {
            mesh: None,
            bodies: [BodyId::default(); MESH_DROP_BODY_COUNT],
            step_count: 0,
            sleep_step: 0,
            hash: 0,
        }
    }
}

/// Drop a grid of thin fast boxes onto a wave mesh. (CreateMeshDrop)
pub fn create_mesh_drop(world: &mut World, origin: Pos) -> MeshDropData {
    let mut data = MeshDropData::default();

    {
        let mut body_def = default_body_def();
        body_def.position = origin;
        let ground_id = create_body(world, &body_def);

        let grid_count = 40;
        let cell_width = 1.0;
        let row_hz = 0.1;
        let column_hz = 0.2;
        let ground_amplitude = 0.5;

        let mesh = create_wave_mesh(
            grid_count,
            grid_count,
            cell_width,
            ground_amplitude,
            row_hz,
            column_hz,
        )
        .expect("wave mesh");
        let mut shape_def = default_shape_def();
        shape_def.filter.category_bits = 1;
        create_mesh_shape(world, ground_id, &shape_def, &mesh, VEC3_ONE);
        data.mesh = Some(mesh);
    }

    {
        let box_hull = make_box_hull(0.02, 0.2, 0.04);

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;

        let mut shape_def = default_shape_def();
        shape_def.base_material.rolling_resistance = 0.1;

        // Don't allow shapes to collide with each other.
        shape_def.filter = Filter {
            category_bits: 2,
            mask_bits: 1,
            group_index: 0,
        };

        set_random_seed(3963634789);

        let grid_count = MESH_DROP_GRID_COUNT;

        for i in 0..grid_count {
            for j in 0..grid_count {
                let linear_velocity = random_vec3_uniform(-1.0, 1.0);
                let angular_velocity = random_vec3_uniform(-5.0, 5.0);

                body_def.position = offset_pos(
                    origin,
                    Vec3 {
                        x: 0.5 * (i as f32 - 0.5 * grid_count as f32),
                        y: 5.0,
                        z: 0.5 * (j as f32 - 0.5 * grid_count as f32),
                    },
                );
                body_def.linear_velocity = linear_velocity;
                body_def.angular_velocity = angular_velocity;
                let body_id = create_body(world, &body_def);
                data.bodies[i * grid_count + j] = body_id;

                create_hull_shape(world, body_id, &shape_def, &box_hull.base);
            }
        }
    }

    data
}

/// Advance sleep/hash tracking for the mesh drop. (UpdateMeshDrop)
pub fn update_mesh_drop(world: &World, data: &mut MeshDropData) -> bool {
    if data.hash == 0 && world_get_awake_body_count(world) == 0 {
        data.hash = HASH_INIT;
        for i in 0..MESH_DROP_BODY_COUNT {
            let xf = body_get_transform(world, data.bodies[i]);
            data.hash = hash_world_transform(data.hash, &xf);
        }

        data.sleep_step = data.step_count;
    }

    data.step_count += 1;

    data.hash != 0
}

/// Release the owned wave mesh. (DestroyMeshDrop)
pub fn destroy_mesh_drop(data: &mut MeshDropData) {
    if let Some(mesh) = data.mesh.take() {
        destroy_mesh(mesh);
    }
}

/// Hash a `b3Vec3`'s little-endian field bytes, matching `b3Hash(h, &vec, sizeof(b3Vec3))`.
fn vec3_bytes(v: Vec3) -> [u8; 12] {
    let mut bytes = [0u8; 12];
    bytes[0..4].copy_from_slice(&v.x.to_le_bytes());
    bytes[4..8].copy_from_slice(&v.y.to_le_bytes());
    bytes[8..12].copy_from_slice(&v.z.to_le_bytes());
    bytes
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::default_world_def;

    #[test]
    fn create_and_destroy_falling_ragdolls() {
        let mut world = World::new(&default_world_def());
        let mut data = create_falling_ragdolls(&mut world);
        assert!(data.grid_mesh.is_some());
        assert!(data.torus_mesh.is_some());
        assert!(!data.groups[0].humans[0].bones[0].body_id.is_null());
        destroy_falling_ragdolls(&mut data);
        assert!(data.grid_mesh.is_none());
    }

    #[test]
    fn hash_world_transform_is_stable() {
        let xf = WorldTransform {
            p: Pos {
                x: 1.0 as _,
                y: 2.0 as _,
                z: 3.0 as _,
            },
            q: crate::math_functions::QUAT_IDENTITY,
        };
        let h1 = hash_world_transform(HASH_INIT, &xf);
        let h2 = hash_world_transform(HASH_INIT, &xf);
        assert_eq!(h1, h2);
        assert_ne!(h1, HASH_INIT);
    }
}
