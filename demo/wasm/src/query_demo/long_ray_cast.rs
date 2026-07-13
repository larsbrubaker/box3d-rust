//! Collision / Long Ray Cast — faithful port of `sample_collision.cpp`
//! `LongRayCast` (line 1334). Very long rays are cast at a row of shapes from
//! kilometers away; a slowly precessing cone sweeps the hit across each surface so
//! single-precision drift shows up as a broken trail. The rock hull is drawn as an
//! icosahedron stand-in (matching the other demos); its collision uses the real
//! `create_rock` hull.

use crate::vis::{hf_triangle_edges, mesh_triangle_edges, push_poses, VisBody};
use box3d_rust::body::{create_body, get_body_transform};
use box3d_rust::geometry::{Capsule, Sphere};
use box3d_rust::height_field::create_wave;
use box3d_rust::id::BodyId;
use box3d_rust::math_functions::{
    length, make_quat_from_axis_angle, mul_sv, offset_pos, rotate_vector, transform_point, Pos,
    Transform, Vec3, PI, VEC3_AXIS_X, VEC3_AXIS_Y, VEC3_ONE, VEC3_ZERO,
};
use box3d_rust::mesh::create_wave_mesh;
use box3d_rust::shape::{
    create_capsule_shape, create_height_field_shape, create_hull_shape, create_mesh_shape,
    create_sphere_shape,
};
use box3d_rust::types::{default_body_def, default_query_filter, default_shape_def};
use box3d_rust::world::{world_cast_ray, World};
use std::cell::RefCell;
use wasm_bindgen::prelude::*;

use super::shared::{cast_closest, CastContext};

const SHAPE_COUNT: usize = 5;
const TRAIL_COUNT: usize = 180;

struct Surface {
    body_id: BodyId,
    local_edges: Vec<f32>,
}

struct State {
    world: World,
    vis: Vec<VisBody>,
    surfaces: Vec<Surface>,
    targets: [Pos; SHAPE_COUNT],
    trail: [Vec<Pos>; SHAPE_COUNT],
    fail_rate: [f32; SHAPE_COUNT],
    ray_length_km: f32,
    cone_angle: f32,
    phase: f32,
}

thread_local! {
    static STATE: RefCell<Option<State>> = const { RefCell::new(None) };
}

fn with_state<R>(f: impl FnOnce(&mut State) -> R) -> R {
    STATE.with(|cell| {
        f(cell
            .borrow_mut()
            .as_mut()
            .expect("lrc not initialized - call lrc_reset first"))
    })
}

struct CastHit {
    point: Pos,
    normal: Vec3,
    hit: bool,
}

/// C `CastAlong` (line 1537): cast a ray through `aim` along `cone_dir`, starting
/// `distance` above it; returns the closest hit.
fn cast_along(world: &World, aim: Pos, cone_dir: Vec3, distance: f32, reach: f32) -> CastHit {
    let origin = offset_pos(aim, mul_sv(distance, cone_dir));
    let translation = mul_sv(-(distance + reach), cone_dir);

    let mut ctx = CastContext::default();
    let filter = default_query_filter();
    world_cast_ray(
        world,
        origin,
        translation,
        &filter,
        |sid, p, n, f, m, t, _c| cast_closest(world, &mut ctx, sid, p, n, f, m, t),
    );

    CastHit {
        point: if ctx.count > 0 {
            ctx.points[0]
        } else {
            Pos::default()
        },
        normal: if ctx.count > 0 {
            ctx.normals[0]
        } else {
            VEC3_ZERO
        },
        hit: ctx.count > 0,
    }
}

#[wasm_bindgen]
pub fn lrc_reset() {
    crate::interact::reset_scene_scales();
    let mut def = box3d_rust::types::default_world_def();
    def.gravity = VEC3_ZERO;
    let mut world = World::new(&def);

    let hull = box3d_rust::hull::create_rock(1.0).expect("rock hull");
    let mesh = create_wave_mesh(8, 8, 0.5, 0.25, 0.2, 0.2).expect("wave mesh");

    let hf_count = 9;
    let hf_scale = Vec3 {
        x: 0.5,
        y: 0.5,
        z: 0.5,
    };
    let height_field = create_wave(hf_count, hf_count, hf_scale, 0.08, 0.16, false);

    let spacing = 5.0f32;
    let aim_height = 2.5f32;
    let mut targets = [Pos::default(); SHAPE_COUNT];
    for (i, t) in targets.iter_mut().enumerate() {
        *t = Pos {
            x: (i as f32 - 2.0) * spacing,
            y: aim_height,
            z: 0.0,
        };
    }

    let mut body_def = default_body_def();
    body_def.type_ = box3d_rust::types::BodyType::Static;
    let shape_def = default_shape_def();
    let mut vis = Vec::new();
    let mut surfaces = Vec::new();

    // Sphere r=1 at targets[0].x.
    body_def.position = Pos {
        x: targets[0].x,
        y: 0.0,
        z: 0.0,
    };
    let b = create_body(&mut world, &body_def);
    create_sphere_shape(
        &mut world,
        b,
        &shape_def,
        &Sphere {
            center: VEC3_ZERO,
            radius: 1.0,
        },
    );
    vis.push(VisBody::sphere_body(b.index1 - 1, 1.0));

    // Capsule along x at targets[1].x.
    body_def.position = Pos {
        x: targets[1].x,
        y: 0.0,
        z: 0.0,
    };
    let b = create_body(&mut world, &body_def);
    let capsule = Capsule {
        center1: Vec3 {
            x: -1.0,
            y: 0.0,
            z: 0.0,
        },
        center2: Vec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        radius: 0.7,
    };
    create_capsule_shape(&mut world, b, &shape_def, &capsule);
    vis.push(VisBody::capsule_body(b.index1 - 1, &capsule));

    // Rock hull at targets[2].x (drawn as an icosahedron stand-in).
    body_def.position = Pos {
        x: targets[2].x,
        y: 0.0,
        z: 0.0,
    };
    let b = create_body(&mut world, &body_def);
    create_hull_shape(&mut world, b, &shape_def, &hull);
    vis.push(VisBody::icosahedron_colored(b.index1 - 1, 1.0, 0));

    // Wave mesh at targets[3].x.
    body_def.position = Pos {
        x: targets[3].x,
        y: 0.0,
        z: 0.0,
    };
    let mesh_body = create_body(&mut world, &body_def);
    create_mesh_shape(&mut world, mesh_body, &shape_def, &mesh, VEC3_ONE);
    surfaces.push(Surface {
        body_id: mesh_body,
        local_edges: mesh_triangle_edges(&mesh, VEC3_ONE),
    });

    // Height field, offset so the patch centers under the ray (line 1414).
    let extent_x = hf_scale.x * (hf_count - 1) as f32;
    let extent_z = hf_scale.z * (hf_count - 1) as f32;
    body_def.position = Pos {
        x: targets[4].x - 0.5 * extent_x,
        y: 0.0,
        z: -0.5 * extent_z,
    };
    let hf_body = create_body(&mut world, &body_def);
    create_height_field_shape(&mut world, hf_body, &shape_def, &height_field);
    surfaces.push(Surface {
        body_id: hf_body,
        local_edges: hf_triangle_edges(&height_field, VEC3_ZERO),
    });

    STATE.with(|cell| {
        *cell.borrow_mut() = Some(State {
            world,
            vis,
            surfaces,
            targets,
            trail: Default::default(),
            fail_rate: [0.0; SHAPE_COUNT],
            ray_length_km: 1.0,
            cone_angle: 5.0,
            phase: 0.0,
        });
    });
}

#[wasm_bindgen]
pub fn lrc_set_params(ray_length_km: f32, cone_angle: f32) {
    with_state(|state| {
        state.ray_length_km = ray_length_km.clamp(1.0, 10000.0);
        state.cone_angle = cone_angle.clamp(0.0, 12.0);
    });
}

#[wasm_bindgen]
pub fn lrc_poses() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        push_poses(&state.world, &state.vis, &mut out);
        out
    })
}

#[wasm_bindgen]
pub fn lrc_surface_wireframe() -> Vec<f32> {
    with_state(|state| {
        let mut out = Vec::new();
        for surf in &state.surfaces {
            let xf = get_body_transform(&state.world, surf.body_id.index1 - 1);
            let local = Transform {
                p: Vec3 {
                    x: xf.p.x as f32,
                    y: xf.p.y as f32,
                    z: xf.p.z as f32,
                },
                q: xf.q,
            };
            let e = &surf.local_edges;
            for i in (0..e.len()).step_by(6) {
                let a = transform_point(
                    local,
                    Vec3 {
                        x: e[i],
                        y: e[i + 1],
                        z: e[i + 2],
                    },
                );
                let b = transform_point(
                    local,
                    Vec3 {
                        x: e[i + 3],
                        y: e[i + 4],
                        z: e[i + 5],
                    },
                );
                out.extend_from_slice(&[a.x, a.y, a.z, b.x, b.y, b.z]);
            }
        }
        out
    })
}

/// Advance the cone one step, cast the long + ground-truth rays, update the trails,
/// and pack the drawer geometry.
/// `[ coneDirX,coneDirY,coneDirZ,
///    per shape: state(0 hit,1 failMiss,2 geomMiss), colorCat(0 green,1 orange),
///               point(3), normal(3), failRate, aim(3),
///               trailCount, trailCount×(px,py,pz, alpha) ]`
#[wasm_bindgen]
pub fn lrc_step() -> Vec<f32> {
    with_state(|state| {
        // Advance the cone (line 1448).
        state.phase += 2.0 * PI / TRAIL_COUNT as f32;
        if state.phase > 2.0 * PI {
            state.phase -= 2.0 * PI;
        }

        let half_angle = state.cone_angle * (PI / 180.0);
        let tilted = rotate_vector(
            make_quat_from_axis_angle(VEC3_AXIS_X, half_angle),
            VEC3_AXIS_Y,
        );
        let cone_dir = rotate_vector(make_quat_from_axis_angle(VEC3_AXIS_Y, state.phase), tilted);

        let reach = 5.0f32;
        let far_distance = 1000.0 * state.ray_length_km;

        let mut out = vec![cone_dir.x, cone_dir.y, cone_dir.z];

        for i in 0..SHAPE_COUNT {
            let aim = state.targets[i];
            let truth = cast_along(&state.world, aim, cone_dir, 50.0, reach);
            let cast = cast_along(&state.world, aim, cone_dir, far_distance, reach);

            let mut fail = 0.0f32;
            let shape_state;
            let mut color_cat = 0.0f32;
            let mut point = Pos::default();
            let mut normal = VEC3_ZERO;

            if cast.hit {
                shape_state = 0.0;
                let error = if truth.hit {
                    length(Vec3 {
                        x: (cast.point.x - truth.point.x) as f32,
                        y: (cast.point.y - truth.point.y) as f32,
                        z: (cast.point.z - truth.point.z) as f32,
                    })
                } else {
                    0.0
                };
                color_cat = if error < 0.05 { 0.0 } else { 1.0 };
                point = cast.point;
                normal = cast.normal;

                // Push into the trail ring buffer.
                let trail = &mut state.trail[i];
                trail.push(cast.point);
                if trail.len() > TRAIL_COUNT {
                    trail.remove(0);
                }
            } else if truth.hit {
                // Accuracy failure (line 1490).
                shape_state = 1.0;
                fail = 1.0;
                point = truth.point;
            } else {
                // Geometric miss (line 1500).
                shape_state = 2.0;
            }

            state.fail_rate[i] = 0.95 * state.fail_rate[i] + 0.05 * fail;

            out.extend_from_slice(&[
                shape_state,
                color_cat,
                point.x as f32,
                point.y as f32,
                point.z as f32,
                normal.x,
                normal.y,
                normal.z,
                state.fail_rate[i],
                aim.x as f32,
                aim.y as f32,
                aim.z as f32,
            ]);

            // Trail (oldest→newest fade).
            let trail = &state.trail[i];
            out.push(trail.len() as f32);
            let n = trail.len();
            for (j, p) in trail.iter().enumerate() {
                let alpha = (j + 1) as f32 / n as f32;
                out.extend_from_slice(&[p.x as f32, p.y as f32, p.z as f32, alpha]);
            }
        }

        out
    })
}
