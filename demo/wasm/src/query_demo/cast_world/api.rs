//! Cast World WASM bindings (query_* exports).

use super::*;
use crate::interact::{self, MouseGrab};
use crate::vis::{pos, push_poses};
use box3d_rust::body::body_compute_aabb;
use box3d_rust::height_field::create_wave;
use box3d_rust::id::NULL_BODY_ID;
use box3d_rust::math_functions::{Vec3, VEC3_ONE};
use box3d_rust::mesh::create_torus_mesh;
use box3d_rust::types::default_world_def;
use box3d_rust::world::World;
use wasm_bindgen::prelude::*;
#[wasm_bindgen]
pub fn query_reset() {
    RAND_SEED.with(|s| s.set(12345));
    // Restore the base Sample launch-speed scale (5.0) on scene reset; a scene
    // that overrides it re-applies its value after the reset returns.
    crate::interact::reset_scene_scales();
    STATE.with(|cell| {
        let mut def = default_world_def();
        def.gravity = Vec3 {
            x: 0.0,
            y: -10.0,
            z: 0.0,
        };
        let world = World::new(&def);
        let mesh = create_torus_mesh(10, 12, 0.65, 0.35).expect("torus mesh");
        let scale = Vec3 {
            x: 0.5 * VEC3_ONE.x,
            y: 0.5 * VEC3_ONE.y,
            z: 0.5 * VEC3_ONE.z,
        };
        let height_field = create_wave(10, 10, scale, 0.03, 0.09, false);

        let state = QueryState {
            world,
            grab: MouseGrab::default(),
            bodies: [NULL_BODY_ID; MAX_COUNT],
            vis: Vec::new(),
            surfaces: Vec::new(),
            body_index: 0,
            mode: MODE_CLOSEST,
            cast_type: CAST_RAY,
            cast_radius: 0.5,
            initial_overlap: false,
            origin: pos(-20.0, 10.0, 0.0),
            translation: Vec3 {
                x: 20.0,
                y: 10.0,
                z: 0.0,
            },
            sphere: Sphere {
                center: VEC3_ZERO,
                radius: 0.9,
            },
            capsule: Capsule {
                center1: Vec3 {
                    x: -0.5,
                    y: 0.0,
                    z: 0.0,
                },
                center2: Vec3 {
                    x: 0.5,
                    y: 0.0,
                    z: 0.0,
                },
                radius: 0.8,
            },
            box_hull: make_box_hull(0.6, 0.6, 0.6),
            mesh,
            height_field,
            cast_context: CastContext::default(),
        };

        // C CastWorld constructor starts with an EMPTY world ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â shapes are added
        // only via CreateShapes() from the UI (sample_collision.cpp:352-384) ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â so there
        // is no body count to report here.
        *cell.borrow_mut() = Some(state);
    })
}

#[wasm_bindgen]
pub fn query_step(dt: f32, sub_steps: i32) -> u32 {
    with_state(|state| {
        state.grab.pre_step(&mut state.world, dt);
        state.world.step(dt, sub_steps);
        state.bodies.iter().filter(|b| b.is_non_null()).count() as u32
    })
}

#[wasm_bindgen]
pub fn query_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.vis, &mut out);
        out
    })
}

/// Packed engine-driven style words parallel to [`query_poses`].
#[wasm_bindgen]
pub fn query_styles() -> Vec<u32> {
    with_state(|state| crate::draw_data::shape_styles(&mut state.world, &state.vis))
}

/// Overlay text labels (mass / sleep / body names / contact + joint labels) as a
/// JSON array. Schema documented on [`crate::interact::collect_debug_text`].
/// Empty (`"[]"`) when no text-relevant view flag is set.
#[wasm_bindgen]
pub fn query_debug_text() -> String {
    with_state(|state| crate::interact::collect_debug_text(&mut state.world))
}

#[wasm_bindgen]
pub fn query_set_params(cast_type: i32, mode: i32, radius: f32, initial_overlap: i32) {
    with_state(|state| {
        state.cast_type = cast_type.clamp(0, 3);
        state.mode = mode.clamp(0, 3);
        state.cast_radius = radius.clamp(0.1, 2.0);
        state.initial_overlap = initial_overlap != 0;
    });
}

#[wasm_bindgen]
pub fn query_set_ray(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) {
    with_state(|state| {
        state.origin = pos(ox, oy, oz);
        state.translation = vec3(tx, ty, tz);
    });
}

#[wasm_bindgen]
pub fn query_add_shapes(shape_type: i32, count: i32) -> u32 {
    with_state(|state| {
        let st = match shape_type {
            0 => ShapeType::Capsule,
            2 => ShapeType::Height,
            3 => ShapeType::Hull,
            4 => ShapeType::Mesh,
            5 => ShapeType::Sphere,
            _ => return 0,
        };
        create_shapes(state, st, count.max(1));
        state.bodies.iter().filter(|b| b.is_non_null()).count() as u32
    })
}

#[wasm_bindgen]
pub fn query_destroy_shape() -> u32 {
    with_state(|state| {
        destroy_one_body(state);
        state.bodies.iter().filter(|b| b.is_non_null()).count() as u32
    })
}

/// `[count, ox,oy,oz, tx,ty,tz, cast_type, radius, hits...]`
/// each hit: px,py,pz, nx,ny,nz, fraction, material, triangle
#[wasm_bindgen]
pub fn query_cast() -> Vec<f32> {
    with_state(|state| {
        run_cast(state);
        let ctx = &state.cast_context;
        let mut out = Vec::with_capacity(9 + ctx.count as usize * HIT_STRIDE);
        out.push(ctx.count as f32);
        out.push(state.origin.x as f32);
        out.push(state.origin.y as f32);
        out.push(state.origin.z as f32);
        out.push(state.translation.x);
        out.push(state.translation.y);
        out.push(state.translation.z);
        out.push(state.cast_type as f32);
        out.push(state.cast_radius);
        for i in 0..ctx.count as usize {
            out.push(ctx.points[i].x as f32);
            out.push(ctx.points[i].y as f32);
            out.push(ctx.points[i].z as f32);
            out.push(ctx.normals[i].x);
            out.push(ctx.normals[i].y);
            out.push(ctx.normals[i].z);
            out.push(ctx.fractions[i]);
            out.push(ctx.material_ids[i] as f32);
            out.push(ctx.triangle_indices[i] as f32);
        }
        out
    })
}

#[wasm_bindgen]
pub fn query_ignore_aabbs() -> Vec<f32> {
    with_state(|state| {
        let mut out = vec![0.0];
        let mut count = 0i32;
        for (i, &body_id) in state.bodies.iter().enumerate() {
            if (i & IGNORE_BASE) == IGNORE_BASE && body_id.is_non_null() {
                let aabb = body_compute_aabb(&state.world, body_id);
                out.push(aabb.lower_bound.x);
                out.push(aabb.lower_bound.y);
                out.push(aabb.lower_bound.z);
                out.push(aabb.upper_bound.x);
                out.push(aabb.upper_bound.y);
                out.push(aabb.upper_bound.z);
                count += 1;
            }
        }
        out[0] = count as f32;
        out
    })
}

#[wasm_bindgen]
pub fn query_surface_wireframe() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        for surf in &state.surfaces {
            if surf.body_id.is_null() {
                continue;
            }
            let xf = get_body_transform(&state.world, surf.body_id.index1 - 1);
            let local_xf = Transform {
                p: Vec3 {
                    x: xf.p.x as f32,
                    y: xf.p.y as f32,
                    z: xf.p.z as f32,
                },
                q: xf.q,
            };
            let edges = &surf.local_edges;
            for i in (0..edges.len()).step_by(6) {
                if i + 5 >= edges.len() {
                    break;
                }
                let a = transform_point(
                    local_xf,
                    Vec3 {
                        x: edges[i],
                        y: edges[i + 1],
                        z: edges[i + 2],
                    },
                );
                let b = transform_point(
                    local_xf,
                    Vec3 {
                        x: edges[i + 3],
                        y: edges[i + 4],
                        z: edges[i + 5],
                    },
                );
                out.extend_from_slice(&[a.x, a.y, a.z, b.x, b.y, b.z]);
            }
        }
        out
    })
}

#[wasm_bindgen]
pub fn query_mouse_down(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> Vec<f32> {
    with_state(|state| {
        let origin = pos(ox, oy, oz);
        let translation = vec3(tx, ty, tz);
        if state.grab.begin(&mut state.world, origin, translation) {
            vec![
                1.0,
                state.grab.mouse_point.x as f32,
                state.grab.mouse_point.y as f32,
                state.grab.mouse_point.z as f32,
            ]
        } else {
            vec![0.0, 0.0, 0.0, 0.0]
        }
    })
}

#[wasm_bindgen]
pub fn query_mouse_move(px: f32, py: f32, pz: f32) {
    with_state(|state| {
        state.grab.move_to(pos(px, py, pz));
    });
}

#[wasm_bindgen]
pub fn query_mouse_up() {
    with_state(|state| {
        state.grab.end(&mut state.world);
    });
}

#[wasm_bindgen]
pub fn query_mouse_active() -> bool {
    with_state(|state| state.grab.is_active())
}

#[wasm_bindgen]
pub fn query_spawn_random(
    ox: f32,
    oy: f32,
    oz: f32,
    tx: f32,
    ty: f32,
    tz: f32,
    variant: u8,
) -> Vec<f32> {
    with_state(|state| {
        let spawned = interact::spawn_projectile(
            &mut state.world,
            pos(ox, oy, oz),
            vec3(tx, ty, tz),
            interact::LaunchVariant::from_u8(variant),
        );
        interact::append_spawned_vis(&state.world, &mut state.vis, &spawned);
        interact::spawn_ok_payload(&spawned)
    })
}

#[wasm_bindgen]
pub fn query_delete_at_ray(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> i32 {
    with_state(|state| {
        let index = interact::delete_at_ray(
            &mut state.world,
            &mut state.grab,
            pos(ox, oy, oz),
            vec3(tx, ty, tz),
        );
        if index >= 0 {
            state.vis.retain(|b| b.body_index != index);
            state.surfaces.retain(|s| s.body_id.index1 - 1 != index);
            for slot in state.bodies.iter_mut() {
                if slot.is_non_null() && slot.index1 - 1 == index {
                    *slot = NULL_BODY_ID;
                    break;
                }
            }
        }
        index
    })
}

#[wasm_bindgen]
pub fn query_counters() -> Vec<f32> {
    with_state(|state| interact::counters_with_sleep(&state.world).to_vec())
}

#[wasm_bindgen]
pub fn query_debug_draw(_flags: u32) -> Vec<f32> {
    with_state(|state| interact::collect_debug_draw(&mut state.world))
}
