//! Query-driven spawn scene (`shared/determinism.c`).
//!
//! Runs one ray/overlap/shape-cast query cycle per step and spawns a shape whose
//! position, type, and size depend on the query results, then hashes the settled
//! transforms.

use super::shared::{hash_world_transform, vec3_bytes};
use crate::body::{body_get_transform, create_body};
use crate::core::{hash, HASH_INIT};
use crate::geometry::{Capsule, Sphere};
use crate::hull::make_box_hull;
use crate::human::{
    random_float_range, random_pos, random_quat, random_unit_vector, random_vec3_uniform,
    set_random_seed,
};
use crate::id::{BodyId, ShapeId};
use crate::math_functions::{mul_sv, offset_pos, Aabb, Pos, Vec3, POS_ZERO, VEC3_ZERO};
use crate::shape::{create_capsule_shape, create_hull_shape, create_sphere_shape};
use crate::types::{default_body_def, default_query_filter, default_shape_def, BodyType};
use crate::world::{
    world_cast_ray_closest, world_cast_shape, world_get_awake_body_count, world_overlap_aabb,
    world_set_gravity, World,
};

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
        |_shape_id,
         _point,
         _normal,
         cast_fraction,
         _user_material_id,
         _triangle_index,
         _child_index| {
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
