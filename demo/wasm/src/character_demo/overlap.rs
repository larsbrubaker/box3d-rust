//! CapsulePlane (`sample_character.cpp:13`) and MoverOverlap (`:153`) — the two
//! draggable-capsule query samples. Both exercise `b3World_CollideMover` against
//! static primitives and visualize the returned planes and the `b3SolvePlanes`
//! push-out; MoverOverlap additionally counts degenerate (zero) plane normals,
//! which must stay 0 even with the mover buried inside a shape.

#![allow(clippy::unnecessary_cast)]

use super::colors;
use super::{
    capsule_style, mover_capsule, new_world, CharacterState, RenderCapsule, SceneKind, SceneState,
};
use crate::vis::VisBody;
use box3d_rust::body::create_body;
use box3d_rust::geometry::{Capsule, CollisionPlane, PlaneResult, Sphere};
use box3d_rust::hull::make_box_hull;
use box3d_rust::math_functions::{
    is_normalized, mul_sv, offset_pos, Pos, Transform, Vec3, QUAT_IDENTITY,
};
use box3d_rust::mover::solve_planes;
use box3d_rust::shape::{create_capsule_shape, create_hull_shape, create_sphere_shape};
use box3d_rust::types::{default_body_def, default_query_filter, default_shape_def};
use box3d_rust::world::world_collide_mover;

const PLANE_CAPACITY: usize = 32;

/// Draggable-capsule query scene shared by CapsulePlane and MoverOverlap.
pub(crate) struct DragScene {
    pub kind: SceneKind,
    pub transform: Transform,
    pub capsule: Capsule,
    /// Solver planes gathered on the last step (used by the Solve button).
    pub planes: Vec<CollisionPlane>,
    /// Raw plane results (`plane` + contact `point`) for the MoverOverlap arrows.
    pub results: Vec<PlaneResult>,
    /// C `m_planeCapacity`: 3 for CapsulePlane, 32 for MoverOverlap.
    pub capacity: usize,
}

impl DragScene {
    /// C `CapsulePlane::Solve` / the MoverOverlap solve — push the capsule out of
    /// the stored planes with a zero target delta.
    pub(crate) fn solve(&mut self) {
        let result = solve_planes(Vec3::default(), &mut self.planes);
        self.transform.p = offset_pos(self.transform.p, result.delta);
    }
}

/// CapsulePlane (`:21`): a green draggable capsule vs a static 0.5³ box hull at
/// `{0,1,1}`. Query capsule radius 0.25, plane capacity 3.
pub(crate) fn build_capsule_plane() -> CharacterState {
    let mut world = new_world();

    let mut body_def = default_body_def();
    body_def.position = Pos {
        x: 0.0,
        y: 1.0,
        z: 1.0,
    };
    let body = create_body(&mut world, &body_def);
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    create_hull_shape(&mut world, body, &default_shape_def(), &box_hull.base);

    let scene = DragScene {
        kind: SceneKind::CapsulePlane,
        transform: Transform {
            p: Pos {
                x: 0.0,
                y: 1.0,
                z: 0.4,
            },
            q: QUAT_IDENTITY,
        },
        capsule: Capsule {
            center1: Vec3 {
                x: 0.0,
                y: -0.5,
                z: 0.0,
            },
            center2: Vec3 {
                x: 0.0,
                y: 0.5,
                z: 0.0,
            },
            radius: 0.25,
        },
        planes: Vec::new(),
        results: Vec::new(),
        capacity: 3,
    };

    let mut state = CharacterState::new(SceneKind::CapsulePlane, world, SceneState::Drag(scene));
    state
        .bodies
        .push(VisBody::box_body(body.index1 - 1, 0.5, 0.5, 0.5));
    state
}

/// MoverOverlap (`:161`): a yellow draggable capsule (radius 0.35) vs a static
/// sphere (x=-3), capsule (x=0), and box hull (x=3). Plane capacity 32.
pub(crate) fn build_mover_overlap() -> CharacterState {
    let mut world = new_world();
    let mut bodies = Vec::new();
    let shape_def = default_shape_def();

    // Static sphere at {-3,1,0}, radius 0.6.
    {
        let mut body_def = default_body_def();
        body_def.position = Pos {
            x: -3.0,
            y: 1.0,
            z: 0.0,
        };
        let body = create_body(&mut world, &body_def);
        let sphere = Sphere {
            center: Vec3::default(),
            radius: 0.6,
        };
        create_sphere_shape(&mut world, body, &shape_def, &sphere);
        bodies.push(VisBody::sphere_body(body.index1 - 1, 0.6));
    }

    // Static capsule at {0,1,0}, axis along Z, radius 0.4.
    {
        let mut body_def = default_body_def();
        body_def.position = Pos {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        };
        let body = create_body(&mut world, &body_def);
        let capsule = Capsule {
            center1: Vec3 {
                x: 0.0,
                y: 0.0,
                z: -0.7,
            },
            center2: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 0.7,
            },
            radius: 0.4,
        };
        create_capsule_shape(&mut world, body, &shape_def, &capsule);
        bodies.push(VisBody::capsule_body(body.index1 - 1, &capsule));
    }

    // Static box hull at {3,1,0}, half-extents 0.6.
    {
        let mut body_def = default_body_def();
        body_def.position = Pos {
            x: 3.0,
            y: 1.0,
            z: 0.0,
        };
        let body = create_body(&mut world, &body_def);
        let box_hull = make_box_hull(0.6, 0.6, 0.6);
        create_hull_shape(&mut world, body, &shape_def, &box_hull.base);
        bodies.push(VisBody::box_body(body.index1 - 1, 0.6, 0.6, 0.6));
    }

    let scene = DragScene {
        kind: SceneKind::MoverOverlap,
        transform: Transform {
            p: Pos {
                x: 0.0,
                y: 3.5,
                z: 0.0,
            },
            q: QUAT_IDENTITY,
        },
        capsule: mover_capsule_035(),
        planes: Vec::new(),
        results: Vec::new(),
        capacity: PLANE_CAPACITY,
    };

    let mut state = CharacterState::new(SceneKind::MoverOverlap, world, SceneState::Drag(scene));
    state.bodies = bodies;
    state
}

fn mover_capsule_035() -> Capsule {
    let mut c = mover_capsule();
    c.radius = 0.35;
    c
}

/// Gather overlap planes at the current query pose (C `PlaneResultFcn`).
fn collide(state: &CharacterState, scene: &DragScene) -> (Vec<CollisionPlane>, Vec<PlaneResult>) {
    let mut planes: Vec<CollisionPlane> = Vec::new();
    let mut results: Vec<PlaneResult> = Vec::new();
    let filter = default_query_filter();
    world_collide_mover(
        &state.world,
        scene.transform.p,
        &scene.capsule,
        &filter,
        |_shape_id, plane_results| {
            for r in plane_results {
                if planes.len() >= scene.capacity {
                    break;
                }
                planes.push(CollisionPlane {
                    plane: r.plane,
                    push_limit: f32::MAX,
                    push: 0.0,
                    clip_velocity: true,
                });
                results.push(*r);
            }
            true
        },
    );
    (planes, results)
}

pub(crate) fn step(state: &mut CharacterState) {
    let (kind, transform, capsule) = match &state.state {
        SceneState::Drag(s) => (s.kind, s.transform, s.capsule),
        _ => return,
    };

    let (planes, results) = {
        let scene = match &state.state {
            SceneState::Drag(s) => s,
            _ => return,
        };
        collide(state, scene)
    };

    // Store for the Solve button (CapsulePlane) and re-render.
    if let SceneState::Drag(s) = &mut state.state {
        s.planes = planes.clone();
        s.results = results.clone();
    }

    match kind {
        SceneKind::CapsulePlane => {
            // Green queried capsule.
            state.render_capsules.push(RenderCapsule {
                transform,
                capsule,
                style: capsule_style(colors::GREEN, 2),
            });

            // Ground grid + world axis lines (C Render()).
            draw_axes(state);

            // One yellow point + short normal line per plane.
            for plane in &planes {
                let p1 = offset_pos(
                    transform.p,
                    mul_sv(plane.plane.offset - capsule.radius, plane.plane.normal),
                );
                let p1v = pos_to_vec3(p1);
                let p2v = p1v + mul_sv(0.1, plane.plane.normal);
                state.draw_point(p1v, 5.0, colors::YELLOW);
                state.draw_line(p1v, p2v, colors::YELLOW);
            }

            state.status = vec![
                planes.len() as f32,
                0.0,
                transform.p.x as f32,
                transform.p.y as f32,
                transform.p.z as f32,
            ];
        }
        SceneKind::MoverOverlap => {
            // Yellow queried capsule.
            state.render_capsules.push(RenderCapsule {
                transform,
                capsule,
                style: capsule_style(colors::YELLOW, 2),
            });

            // One arrow per returned plane, from the contact point along the
            // normal; a degenerate (zero) normal is drawn red.
            let mut zero_normal_count = 0i32;
            for r in &results {
                let valid = is_normalized(r.plane.normal);
                let color = if valid {
                    colors::LIME_GREEN
                } else {
                    colors::RED
                };
                let rp = pos_to_vec3(offset_pos(transform.p, r.point));
                state.draw_point(rp, 6.0, color);
                state.draw_line(rp, rp + mul_sv(0.5, r.plane.normal), color);
                if !valid {
                    zero_normal_count += 1;
                }
            }

            // Solve the planes and show the pushed-out (cyan) capsule.
            let mut solver_planes = planes.clone();
            let solved = solve_planes(Vec3::default(), &mut solver_planes);
            let pushed = Transform {
                p: offset_pos(transform.p, solved.delta),
                q: transform.q,
            };
            state.render_capsules.push(RenderCapsule {
                transform: pushed,
                capsule,
                style: capsule_style(colors::CYAN, 2),
            });

            state.status = vec![
                planes.len() as f32,
                zero_normal_count as f32,
                transform.p.x as f32,
                transform.p.y as f32,
                transform.p.z as f32,
            ];
        }
        _ => {}
    }
}

/// C `CapsulePlane::Render` axis lines: X red, Y green, Z blue (length 2 from origin).
fn draw_axes(state: &mut CharacterState) {
    let o = Vec3::default();
    state.draw_line(
        o,
        Vec3 {
            x: 2.0,
            y: 0.0,
            z: 0.0,
        },
        colors::RED,
    );
    state.draw_line(
        o,
        Vec3 {
            x: 0.0,
            y: 2.0,
            z: 0.0,
        },
        colors::GREEN,
    );
    state.draw_line(
        o,
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 2.0,
        },
        colors::BLUE,
    );
}

fn pos_to_vec3(p: Pos) -> Vec3 {
    Vec3 {
        x: p.x as f32,
        y: p.y as f32,
        z: p.z as f32,
    }
}
