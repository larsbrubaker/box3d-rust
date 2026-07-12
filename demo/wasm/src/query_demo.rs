//! Interactive raycast query demo.

use crate::vis::{pos, push_poses, sphere, VisBody};
use box3d_rust::body::create_body;
use box3d_rust::hull::make_box_hull;
use box3d_rust::math_functions::Vec3;
use box3d_rust::shape::{create_hull_shape, create_sphere_shape};
use box3d_rust::types::{
    default_body_def, default_query_filter, default_shape_def, default_world_def, BodyType,
};
use box3d_rust::world::{world_cast_ray_closest, World};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

thread_local! {
    static STATE: RefCell<Option<QueryState>> = const { RefCell::new(None) };
}

struct QueryState {
    world: World,
    bodies: Vec<VisBody>,
}

fn with_state<R>(f: impl FnOnce(&mut QueryState) -> R) -> R {
    STATE.with(|cell| {
        let mut slot = cell.borrow_mut();
        f(slot
            .as_mut()
            .expect("query not initialized — call query_reset first"))
    })
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

/// Static props for interactive ray casts.
#[wasm_bindgen]
pub fn query_reset() -> u32 {
    STATE.with(|cell| {
        let mut world = new_world();
        let mut bodies = Vec::new();

        let mut ground_def = default_body_def();
        ground_def.type_ = BodyType::Static;
        ground_def.position = pos(0.0, -0.5, 0.0);
        let ground = create_body(&mut world, &ground_def);
        let ground_hull = make_box_hull(10.0, 0.5, 10.0);
        create_hull_shape(&mut world, ground, &default_shape_def(), &ground_hull.base);
        bodies.push(VisBody::box_body(ground.index1 - 1, 10.0, 0.5, 10.0));

        let props: &[(f32, f32, f32, f32, f32, f32)] = &[
            (-2.0, 1.0, 0.0, 1.0, 1.0, 1.0),
            (2.0, 1.5, -1.0, 0.75, 1.5, 0.75),
            (0.0, 0.75, 2.5, 1.5, 0.75, 0.5),
            (-1.5, 0.6, -2.0, 0.6, 0.6, 0.6),
        ];
        for &(x, y, z, hx, hy, hz) in props {
            let mut body_def = default_body_def();
            body_def.type_ = BodyType::Static;
            body_def.position = pos(x, y, z);
            let body = create_body(&mut world, &body_def);
            let hull = make_box_hull(hx, hy, hz);
            create_hull_shape(&mut world, body, &default_shape_def(), &hull.base);
            bodies.push(VisBody::box_body(body.index1 - 1, hx, hy, hz));
        }

        // One dynamic sphere so the scene has motion
        let mut dyn_def = default_body_def();
        dyn_def.type_ = BodyType::Dynamic;
        dyn_def.position = pos(0.5, 4.0, 0.0);
        let ball = create_body(&mut world, &dyn_def);
        let mut shape_def = default_shape_def();
        shape_def.density = 1.0;
        let sph = sphere(0.4);
        create_sphere_shape(&mut world, ball, &shape_def, &sph);
        bodies.push(VisBody::sphere_body(ball.index1 - 1, 0.4));

        let count = bodies.len() as u32;
        *cell.borrow_mut() = Some(QueryState { world, bodies });
        count
    })
}

#[wasm_bindgen]
pub fn query_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        state.world.step(dt, sub_steps);
        state.bodies.len() as u32
    })
}

#[wasm_bindgen]
pub fn query_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.bodies, &mut out);
        out
    })
}

/// Closest ray cast. Returns
/// `[hit, px, py, pz, nx, ny, nz, fraction]` (hit is 1/0).
#[wasm_bindgen]
pub fn query_ray_cast(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> Vec<f32> {
    with_state(|state| {
        let origin = pos(ox, oy, oz);
        let translation = Vec3 {
            x: tx - ox,
            y: ty - oy,
            z: tz - oz,
        };
        let filter = default_query_filter();
        let result = world_cast_ray_closest(&state.world, origin, translation, &filter);
        if result.hit {
            vec![
                1.0,
                result.point.x as f32,
                result.point.y as f32,
                result.point.z as f32,
                result.normal.x,
                result.normal.y,
                result.normal.z,
                result.fraction,
            ]
        } else {
            vec![0.0, tx, ty, tz, 0.0, 1.0, 0.0, 1.0]
        }
    })
}
