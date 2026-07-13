//! Collision / Capsule Cast Ray — faithful port of `sample_collision.cpp`
//! `CapsuleCastRay` (line 2726). A single kinematic capsule body is cast against
//! by `b3Body_CastRay` along a fixed segment; the sample is a static viewer.

use crate::vis::{push_poses, VisBody};
use box3d_rust::body::{body_cast_ray, create_body};
use box3d_rust::geometry::Capsule;
use box3d_rust::id::BodyId;
use box3d_rust::math_functions::{Pos, Vec3, WORLD_TRANSFORM_IDENTITY};
use box3d_rust::shape::create_capsule_shape;
use box3d_rust::types::{default_body_def, default_query_filter, default_shape_def, BodyType};
use box3d_rust::world::World;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

struct State {
    world: World,
    vis: Vec<VisBody>,
    /// Cached ray + cast result. The ray, translation, and (kinematic, never-moved)
    /// capsule body are all fixed, so C computes this once on input; we cache it at
    /// reset instead of recasting every render frame.
    cast: Vec<f32>,
}

thread_local! {
    static STATE: RefCell<Option<State>> = const { RefCell::new(None) };
}

fn with_state<R>(f: impl FnOnce(&mut State) -> R) -> R {
    STATE.with(|cell| {
        f(cell
            .borrow_mut()
            .as_mut()
            .expect("ccray not initialized - call ccray_reset first"))
    })
}

#[wasm_bindgen]
pub fn ccray_reset() {
    crate::interact::reset_scene_scales();
    let mut world = World::new(&box3d_rust::types::default_world_def());

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Kinematic;
    let body_id = create_body(&mut world, &body_def);

    let shape_def = default_shape_def();

    // C: capsule center1={0,0,0} center2={0,1,0} radius=0.5 (line 2754).
    let capsule = Capsule {
        center1: Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        center2: Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        },
        radius: 0.5,
    };
    create_capsule_shape(&mut world, body_id, &shape_def, &capsule);

    let vis = vec![VisBody::capsule_body(body_id.index1 - 1, &capsule)];

    let cast = compute_cast(&world, body_id);

    STATE.with(|cell| *cell.borrow_mut() = Some(State { world, vis, cast }));
}

/// The fixed ray + the body cast result.
/// `[ox,oy,oz, ex,ey,ez, hit, hx,hy,hz]`. (C `Render`, line 2765.)
fn compute_cast(world: &World, body_id: BodyId) -> Vec<f32> {
    // C: origin={-1,0.5,0}, translation={2,0,0}, maxFraction=1, xf=identity.
    let origin = Pos {
        x: -1.0,
        y: 0.5,
        z: 0.0,
    };
    let translation = Vec3 {
        x: 2.0,
        y: 0.0,
        z: 0.0,
    };
    let filter = default_query_filter();
    let result = body_cast_ray(
        world,
        body_id,
        origin,
        translation,
        &filter,
        1.0,
        WORLD_TRANSFORM_IDENTITY,
    );
    let end = Pos {
        x: origin.x + translation.x,
        y: origin.y + translation.y,
        z: origin.z + translation.z,
    };
    let mut out = vec![
        origin.x as f32,
        origin.y as f32,
        origin.z as f32,
        end.x as f32,
        end.y as f32,
        end.z as f32,
    ];
    if result.hit {
        out.push(1.0);
        out.push(result.point.x as f32);
        out.push(result.point.y as f32);
        out.push(result.point.z as f32);
    } else {
        out.extend_from_slice(&[0.0, 0.0, 0.0, 0.0]);
    }
    out
}

#[wasm_bindgen]
pub fn ccray_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.vis, &mut out);
        out
    })
}

/// The cached fixed ray + body cast result (computed once at reset).
#[wasm_bindgen]
pub fn ccray_cast() -> Vec<f32> {
    with_state(|state| state.cast.clone())
}
