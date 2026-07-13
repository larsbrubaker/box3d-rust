//! World samples: Far Pyramid (`sample_world.cpp` FarPyramid).

use crate::interact::{self, MouseGrab};
use box3d_rust::body::{create_body, destroy_body, get_body_transform, make_body_id};
use box3d_rust::core::is_double_precision;
use box3d_rust::hull::make_box_hull;
use box3d_rust::math_functions::{offset_pos, sub_pos, Pos, Vec3};
use box3d_rust::shape::create_hull_shape;
use box3d_rust::types::{default_body_def, default_shape_def, default_world_def, BodyType};
use box3d_rust::world::{
    world_enable_continuous, world_enable_sleeping, world_enable_warm_starting,
    world_set_contact_recycle_distance, World,
};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

const OFFSET_KM: f32 = 10000.0;
const BASE_COUNT: i32 = 40;

thread_local! {
    static FAR: RefCell<Option<FarState>> = const { RefCell::new(None) };
}

struct FarBody {
    body_index: i32,
    half_extents: [f32; 3],
}
struct FarState {
    world: World,
    bodies: Vec<FarBody>,
    grab: MouseGrab,
    base: Pos,
    step_count: u32,
}

fn with_far<R>(f: impl FnOnce(&mut FarState) -> R) -> R {
    FAR.with(|cell| {
        let mut slot = cell.borrow_mut();
        let state = slot.as_mut().expect("far pyramid not initialized");
        f(state)
    })
}

fn new_world() -> World {
    // Restore the base Sample launch-speed scale (5.0) on every scene reset (the
    // far-world reset builds its world through here); overrides re-apply after.
    crate::interact::reset_scene_scales();
    let mut def = default_world_def();
    def.gravity = Vec3 {
        x: 0.0,
        y: -10.0,
        z: 0.0,
    };
    World::new(&def)
}

fn content_base() -> Pos {
    Pos {
        x: (1000.0 * OFFSET_KM) as _,
        y: 0.0 as _,
        z: 0.0 as _,
    }
}
fn to_world(base: Pos, rx: f32, ry: f32, rz: f32) -> Pos {
    offset_pos(
        base,
        Vec3 {
            x: rx,
            y: ry,
            z: rz,
        },
    )
}

#[wasm_bindgen]
pub fn is_double_precision_build() -> bool {
    is_double_precision()
}
#[wasm_bindgen]
pub fn world_far_pyramid_offset_km() -> f32 {
    OFFSET_KM
}

#[wasm_bindgen]
pub fn world_reset_far_pyramid() -> u32 {
    FAR.with(|cell| {
        let base = content_base();
        let mut world = new_world();
        let mut bodies = Vec::new();
        {
            let mut body_def = default_body_def();
            body_def.position = offset_pos(
                base,
                Vec3 {
                    x: 0.0,
                    y: -1.0,
                    z: 0.0,
                },
            );
            let ground = create_body(&mut world, &body_def);
            let shape_def = default_shape_def();
            let ground_hull = make_box_hull(400.0, 1.0, 400.0);
            create_hull_shape(&mut world, ground, &shape_def, &ground_hull.base);
            bodies.push(FarBody {
                body_index: ground.index1 - 1,
                half_extents: [400.0, 1.0, 400.0],
            });
        }
        let h = 0.5f32;
        let shift = h;
        let box_hull = make_box_hull(h, h, h);
        let mut shape_def = default_shape_def();
        shape_def.density = 100.0;
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        for i in 0..BASE_COUNT {
            let y = (2.0 * i as f32 + 1.0) * shift;
            for j in i..BASE_COUNT {
                let x =
                    (i as f32 + 1.0) * shift + 2.0 * (j - i) as f32 * shift - h * BASE_COUNT as f32;
                body_def.position = offset_pos(base, Vec3 { x, y, z: 0.0 });
                let body_id = create_body(&mut world, &body_def);
                create_hull_shape(&mut world, body_id, &shape_def, &box_hull.base);
                bodies.push(FarBody {
                    body_index: body_id.index1 - 1,
                    half_extents: [h, h, h],
                });
            }
        }
        let n = bodies.len() as u32;
        *cell.borrow_mut() = Some(FarState {
            world,
            bodies,
            grab: MouseGrab::default(),
            base,
            step_count: 0,
        });
        n
    })
}

#[wasm_bindgen]
pub fn world_far_pyramid_step(dt: f32, sub_steps: i32) -> u32 {
    with_far(|state| {
        state.grab.pre_step(&mut state.world, dt);
        state.world.step(dt, sub_steps);
        state.step_count = state.step_count.saturating_add(1);
        state.bodies.len() as u32
    })
}
#[wasm_bindgen]
pub fn world_far_pyramid_step_count() -> u32 {
    with_far(|s| s.step_count)
}
#[wasm_bindgen]
pub fn world_far_pyramid_poses() -> Vec<f32> {
    with_far(|state| {
        let mut out = Vec::with_capacity(state.bodies.len() * 11);
        for b in &state.bodies {
            let xf = get_body_transform(&state.world, b.body_index);
            let rel = sub_pos(xf.p, state.base);
            out.extend_from_slice(&[
                rel.x,
                rel.y,
                rel.z,
                xf.q.v.x,
                xf.q.v.y,
                xf.q.v.z,
                xf.q.s,
                b.half_extents[0],
                b.half_extents[1],
                b.half_extents[2],
                0.0,
            ]);
        }
        out
    })
}
/// Packed engine-driven style words parallel to [`world_far_pyramid_poses`].
#[wasm_bindgen]
pub fn world_far_pyramid_styles() -> Vec<u32> {
    with_far(|state| {
        crate::draw_data::shape_styles_indexed(
            &mut state.world,
            state.bodies.iter().map(|b| b.body_index),
        )
    })
}

/// Two representative style words for the Far Pyramid: `[ground, first box]`.
///
/// `bodies[0]` is the static ground slab and `bodies[1]` is the first (bottom
/// row) dynamic box. The renderer uses these to color the instanced pyramid
/// without polling the full per-body style array every frame. Only those two
/// bodies are resolved (a 2-entry index slice), so this stays cheap even at the
/// pyramid's ~820-body count. Returns `[]` if the scene has fewer than 2 bodies.
#[wasm_bindgen]
pub fn world_far_pyramid_style_pair() -> Vec<u32> {
    with_far(|state| {
        if state.bodies.len() < 2 {
            return Vec::new();
        }
        crate::draw_data::shape_styles_indexed(
            &mut state.world,
            [state.bodies[0].body_index, state.bodies[1].body_index],
        )
    })
}
#[wasm_bindgen]
pub fn world_far_pyramid_mouse_down(
    ox: f32,
    oy: f32,
    oz: f32,
    tx: f32,
    ty: f32,
    tz: f32,
) -> Vec<f32> {
    with_far(|state| {
        let origin = to_world(state.base, ox, oy, oz);
        if state
            .grab
            .begin(&mut state.world, origin, interact::vec3(tx, ty, tz))
        {
            let rel = sub_pos(state.grab.mouse_point, state.base);
            vec![1.0, rel.x, rel.y, rel.z]
        } else {
            vec![0.0, 0.0, 0.0, 0.0]
        }
    })
}
#[wasm_bindgen]
pub fn world_far_pyramid_mouse_move(px: f32, py: f32, pz: f32) {
    with_far(|s| s.grab.move_to(to_world(s.base, px, py, pz)));
}
#[wasm_bindgen]
pub fn world_far_pyramid_mouse_up() {
    with_far(|s| s.grab.end(&mut s.world));
}
#[wasm_bindgen]
pub fn world_far_pyramid_mouse_active() -> bool {
    with_far(|s| s.grab.is_active())
}
#[wasm_bindgen]
pub fn world_far_pyramid_spawn_random(
    ox: f32,
    oy: f32,
    oz: f32,
    tx: f32,
    ty: f32,
    tz: f32,
) -> Vec<f32> {
    with_far(|state| {
        match interact::spawn_random(
            &mut state.world,
            to_world(state.base, ox, oy, oz),
            interact::vec3(tx, ty, tz),
        ) {
            Some(spawned) => {
                state.bodies.push(FarBody {
                    body_index: spawned.body_index,
                    half_extents: spawned.half_extents,
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
pub fn world_far_pyramid_delete_at_ray(
    ox: f32,
    oy: f32,
    oz: f32,
    tx: f32,
    ty: f32,
    tz: f32,
) -> u32 {
    with_far(|state| {
        let index = interact::delete_at_ray(
            &mut state.world,
            &mut state.grab,
            to_world(state.base, ox, oy, oz),
            interact::vec3(tx, ty, tz),
        );
        if index < 0 {
            return 0;
        }
        state.bodies.retain(|b| b.body_index != index);
        1
    })
}
#[wasm_bindgen]
pub fn world_far_pyramid_counters() -> Vec<f32> {
    with_far(|s| interact::counters_with_sleep(&s.world).to_vec())
}
#[wasm_bindgen]
pub fn world_far_pyramid_debug_draw(_flags: u32) -> Vec<f32> {
    with_far(|state| {
        // Subtract the scene base at collection (in `b3Pos` space) via the shared
        // large-world draw-origin mechanism, instead of reconstructing from the
        // already-truncated f32 buffer.
        let base = state.base;
        interact::with_draw_base(base, || interact::collect_debug_draw(&mut state.world))
    })
}
#[wasm_bindgen]
pub fn world_far_pyramid_set_enable_sleep(flag: bool) {
    with_far(|s| world_enable_sleeping(&mut s.world, flag));
}
#[wasm_bindgen]
pub fn world_far_pyramid_set_enable_warm_starting(flag: bool) {
    with_far(|s| world_enable_warm_starting(&mut s.world, flag));
}
#[wasm_bindgen]
pub fn world_far_pyramid_set_enable_continuous(flag: bool) {
    with_far(|s| world_enable_continuous(&mut s.world, flag));
}
#[wasm_bindgen]
pub fn world_far_pyramid_set_recycle_distance(meters: f32) {
    with_far(|s| world_set_contact_recycle_distance(&mut s.world, meters));
}
#[wasm_bindgen]
pub fn world_far_pyramid_destroy_body_index(body_index: i32) -> u32 {
    with_far(|state| {
        state.grab.end(&mut state.world);
        if !state.bodies.iter().any(|b| b.body_index == body_index) {
            return 0;
        }
        let id = make_body_id(&state.world, body_index);
        destroy_body(&mut state.world, id);
        state.bodies.retain(|b| b.body_index != body_index);
        1
    })
}
