//! Collision / Cast World - faithful port of `sample_collision.cpp` CastWorld.

use crate::interact::MouseGrab;
use crate::rng::XorShift32;
use crate::vis::{hf_triangle_edges, mesh_triangle_edges, vec3, VisBody};
use box3d_rust::body::{create_body, destroy_body, get_body_transform};
use box3d_rust::distance::ShapeProxy;
use box3d_rust::geometry::{Capsule, ShapeType, Sphere, SurfaceMaterial};
use box3d_rust::height_field::HeightFieldData;
use box3d_rust::hull::{make_box_hull, make_transformed_box_hull, BoxHull};
use box3d_rust::id::{BodyId, ShapeId, NULL_BODY_ID};
use box3d_rust::math_functions::{
    make_quat_from_axis_angle, normalize, offset_pos, transform_point, Pos, Transform, Vec3, PI,
    POS_ZERO, TRANSFORM_IDENTITY, VEC3_ZERO,
};
use box3d_rust::mesh::MeshData;
use box3d_rust::shape::{
    create_capsule_shape, create_height_field_shape, create_hull_shape, create_mesh_shape,
    create_sphere_shape, shape_get_user_data,
};
use box3d_rust::types::{default_body_def, default_query_filter, default_shape_def, BodyType};
use box3d_rust::world::{world_cast_ray, world_cast_shape, World};
use std::cell::{Cell, RefCell};

const MAX_COUNT: usize = 64;
const IGNORE_BASE: usize = 0x7;

const MODE_ANY: i32 = 0;
const MODE_CLOSEST: i32 = 1;
const MODE_MULTIPLE: i32 = 2;
const MODE_SORTED: i32 = 3;

const CAST_RAY: i32 = 0;
const CAST_SPHERE: i32 = 1;
const CAST_CAPSULE: i32 = 2;
const CAST_BOX: i32 = 3;

const HIT_STRIDE: usize = 9;

thread_local! {
    static STATE: RefCell<Option<QueryState>> = const { RefCell::new(None) };
    static RAND_SEED: Cell<u32> = const { Cell::new(12345) };
}

struct CastContext {
    points: [Pos; 3],
    normals: [Vec3; 3],
    fractions: [f32; 3],
    material_ids: [u64; 3],
    triangle_indices: [i32; 3],
    count: i32,
    initial_overlap: bool,
}

impl Default for CastContext {
    fn default() -> Self {
        Self {
            points: [POS_ZERO; 3],
            normals: [VEC3_ZERO; 3],
            fractions: [f32::MAX; 3],
            material_ids: [0; 3],
            triangle_indices: [0; 3],
            count: 0,
            initial_overlap: false,
        }
    }
}

struct SurfaceVis {
    body_id: BodyId,
    local_edges: Vec<f32>,
}

struct QueryState {
    world: World,
    grab: MouseGrab,
    bodies: [BodyId; MAX_COUNT],
    vis: Vec<VisBody>,
    surfaces: Vec<SurfaceVis>,
    body_index: usize,
    mode: i32,
    cast_type: i32,
    cast_radius: f32,
    initial_overlap: bool,
    origin: Pos,
    translation: Vec3,
    sphere: Sphere,
    capsule: Capsule,
    box_hull: BoxHull,
    mesh: MeshData,
    height_field: HeightFieldData,
    cast_context: CastContext,
}

fn with_state<R>(f: impl FnOnce(&mut QueryState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("query not initialized - call query_reset first"))
    })
}

/// XorShift like C `RandomFloatRange` / `shared/utils.h` (not `box3d_rust::human`).
/// Draws from the shared [`XorShift32`] over this module's thread-local seed cell.
fn random_float_range(lo: f32, hi: f32) -> f32 {
    RAND_SEED.with(|seed| {
        let mut rng = XorShift32::with_seed(seed.get());
        let v = rng.range(lo, hi);
        seed.set(rng.seed());
        v
    })
}

fn random_vec3_uniform(lo: f32, hi: f32) -> Vec3 {
    Vec3 {
        x: random_float_range(lo, hi),
        y: random_float_range(lo, hi),
        z: random_float_range(lo, hi),
    }
}

fn remove_body_vis(state: &mut QueryState, body_id: BodyId) {
    let index = body_id.index1 - 1;
    state.vis.retain(|v| v.body_index != index);
    state.surfaces.retain(|s| s.body_id != body_id);
}

fn cast_callback(
    world: &World,
    ctx: &mut CastContext,
    mode: i32,
    shape_id: ShapeId,
    point: Pos,
    normal: Vec3,
    fraction: f32,
    material_id: u64,
    triangle_index: i32,
) -> f32 {
    if !ctx.initial_overlap && fraction == 0.0 {
        return -1.0;
    }
    if shape_get_user_data(world, shape_id) == 1 {
        return -1.0;
    }

    match mode {
        MODE_ANY => {
            ctx.points[0] = point;
            ctx.normals[0] = normal;
            ctx.fractions[0] = fraction;
            ctx.material_ids[0] = material_id;
            ctx.triangle_indices[0] = triangle_index;
            ctx.count = 1;
            0.0
        }
        MODE_MULTIPLE => {
            let count = ctx.count as usize;
            debug_assert!(count < 3);
            ctx.points[count] = point;
            ctx.normals[count] = normal;
            ctx.fractions[count] = fraction;
            ctx.material_ids[count] = material_id;
            ctx.triangle_indices[count] = triangle_index;
            ctx.count = count as i32 + 1;
            if ctx.count == 3 {
                0.0
            } else {
                1.0
            }
        }
        MODE_SORTED => {
            let count = ctx.count;
            debug_assert!(count <= 3);
            let mut index = 3;
            while fraction < ctx.fractions[index - 1] {
                index -= 1;
                if index == 0 {
                    break;
                }
            }
            if index == 3 {
                return ctx.fractions[2];
            }
            for j in (index + 1..=2).rev() {
                ctx.points[j] = ctx.points[j - 1];
                ctx.normals[j] = ctx.normals[j - 1];
                ctx.fractions[j] = ctx.fractions[j - 1];
                ctx.material_ids[j] = ctx.material_ids[j - 1];
                ctx.triangle_indices[j] = ctx.triangle_indices[j - 1];
            }
            ctx.points[index] = point;
            ctx.normals[index] = normal;
            ctx.fractions[index] = fraction;
            ctx.material_ids[index] = material_id;
            ctx.triangle_indices[index] = triangle_index;
            ctx.count = if count < 3 { count + 1 } else { 3 };
            if ctx.count == 3 {
                ctx.fractions[2]
            } else {
                1.0
            }
        }
        _ => {
            // MODE_CLOSEST (and fallback)
            ctx.points[0] = point;
            ctx.normals[0] = normal;
            ctx.fractions[0] = fraction;
            ctx.material_ids[0] = material_id;
            ctx.triangle_indices[0] = triangle_index;
            ctx.count = 1;
            fraction
        }
    }
}

fn run_cast(state: &mut QueryState) {
    let mut ctx = CastContext {
        initial_overlap: state.initial_overlap,
        fractions: [f32::MAX; 3],
        ..Default::default()
    };

    let radius = state.cast_radius;
    let mut proxy = ShapeProxy::default();
    let mut _box_keep: Option<BoxHull> = None;

    match state.cast_type {
        CAST_RAY => {
            proxy.count = 0;
        }
        CAST_SPHERE => {
            proxy.count = 1;
            proxy.radius = radius;
            proxy.points[0] = VEC3_ZERO;
        }
        CAST_CAPSULE => {
            proxy.count = 2;
            proxy.radius = radius;
            proxy.points[0] = VEC3_ZERO;
            proxy.points[1] = Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            };
        }
        CAST_BOX => {
            let extent = Vec3 {
                x: radius,
                y: 0.5 * radius,
                z: 0.25 * radius,
            };
            let box_hull =
                make_transformed_box_hull(extent.x, extent.y, extent.z, TRANSFORM_IDENTITY);
            proxy.count = box_hull.base.vertex_count;
            for i in 0..box_hull.base.vertex_count as usize {
                proxy.points[i] = box_hull.box_points[i];
            }
            proxy.radius = 0.0;
            _box_keep = Some(box_hull);
        }
        _ => {}
    }

    let mut filter = default_query_filter();
    filter.name = "cast_world".into();
    let mode = state.mode;
    let origin = state.origin;
    let translation = state.translation;
    let cast_type = state.cast_type;

    {
        let world = &state.world;
        let callback = |shape_id, point, normal, fraction, material_id, triangle_index, _child| {
            cast_callback(
                world,
                &mut ctx,
                mode,
                shape_id,
                point,
                normal,
                fraction,
                material_id,
                triangle_index,
            )
        };

        if cast_type == CAST_RAY {
            world_cast_ray(world, origin, translation, &filter, callback);
        } else {
            world_cast_shape(world, origin, &proxy, translation, &filter, callback);
        }
    }

    state.cast_context = ctx;
}

fn create_shapes(state: &mut QueryState, shape_type: ShapeType, count: i32) {
    let mut shape_def = default_shape_def();
    let mut body_def = default_body_def();
    body_def.gravity_scale = 0.0;

    for _ in 0..count {
        let idx = state.body_index;
        if state.bodies[idx].is_non_null() {
            let old = state.bodies[idx];
            destroy_body(&mut state.world, old);
            remove_body_vis(state, old);
            state.bodies[idx] = NULL_BODY_ID;
        }

        body_def.type_ = if idx % 3 == 0 {
            BodyType::Kinematic
        } else if idx % 2 == 0 {
            BodyType::Dynamic
        } else {
            BodyType::Static
        };

        if shape_type == ShapeType::Height {
            body_def.type_ = BodyType::Static;
        }

        body_def.position = offset_pos(POS_ZERO, random_vec3_uniform(-20.0, 20.0));
        let axis = normalize(random_vec3_uniform(-1.0, 1.0));
        let angle = random_float_range(-PI, PI);
        body_def.rotation = make_quat_from_axis_angle(axis, angle);

        let body_id = create_body(&mut state.world, &body_def);
        let flag: u64 = if (idx & IGNORE_BASE) == IGNORE_BASE {
            1
        } else {
            0
        };
        shape_def.user_data = flag;
        shape_def.materials.clear();

        match shape_type {
            ShapeType::Sphere => {
                shape_def.base_material.user_material_id = 11;
                create_sphere_shape(&mut state.world, body_id, &shape_def, &state.sphere);
                state.vis.push(VisBody::sphere_body(
                    body_id.index1 - 1,
                    state.sphere.radius,
                ));
            }
            ShapeType::Capsule => {
                shape_def.base_material.user_material_id = 22;
                create_capsule_shape(&mut state.world, body_id, &shape_def, &state.capsule);
                state
                    .vis
                    .push(VisBody::capsule_body(body_id.index1 - 1, &state.capsule));
            }
            ShapeType::Hull => {
                shape_def.base_material.user_material_id = 33;
                create_hull_shape(&mut state.world, body_id, &shape_def, &state.box_hull.base);
                state
                    .vis
                    .push(VisBody::box_body(body_id.index1 - 1, 0.6, 0.6, 0.6));
            }
            ShapeType::Mesh => {
                shape_def.base_material.user_material_id = 44;
                let scale = Vec3 {
                    x: 4.0,
                    y: 3.0,
                    z: -2.0,
                };
                create_mesh_shape(&mut state.world, body_id, &shape_def, &state.mesh, scale);
                let edges = mesh_triangle_edges(&state.mesh, scale);
                state.surfaces.push(SurfaceVis {
                    body_id,
                    local_edges: edges,
                });
            }
            ShapeType::Height => {
                shape_def.base_material.user_material_id = 55;
                shape_def.materials = vec![
                    SurfaceMaterial {
                        user_material_id: 111,
                        ..Default::default()
                    },
                    SurfaceMaterial {
                        user_material_id: 222,
                        ..Default::default()
                    },
                    SurfaceMaterial {
                        user_material_id: 333,
                        ..Default::default()
                    },
                ];
                create_height_field_shape(
                    &mut state.world,
                    body_id,
                    &shape_def,
                    &state.height_field,
                );
                let edges = hf_triangle_edges(&state.height_field, VEC3_ZERO);
                state.surfaces.push(SurfaceVis {
                    body_id,
                    local_edges: edges,
                });
            }
            _ => {}
        }

        state.bodies[idx] = body_id;
        state.body_index = (idx + 1) % MAX_COUNT;
    }
}

fn destroy_one_body(state: &mut QueryState) {
    for i in 0..MAX_COUNT {
        if state.bodies[i].is_non_null() {
            let body_id = state.bodies[i];
            destroy_body(&mut state.world, body_id);
            remove_body_vis(state, body_id);
            state.bodies[i] = NULL_BODY_ID;
            return;
        }
    }
}

mod api;
#[allow(unused_imports)] // re-export for unit tests + wasm_bindgen visibility
pub use api::*;

#[cfg(test)]
mod tests;
