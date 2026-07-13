//! BasicMover (`sample_character.cpp:314`) — the real Character / Mover level and
//! the C-exact `CharacterMover` kinematic algorithm (`sample.cpp:2099-2432`).
//!
//! The mover is a query-driven capsule (not a rigid body): each step it damps and
//! accelerates its own velocity, probes the ground with a pogo ray/spring, then
//! slides against the world with `b3World_CollideMover` + `b3SolvePlanes` +
//! `b3World_CastMover` over 5 iterations, pushes any dynamic bodies it leans on,
//! and finally either clips its velocity against the contact planes or recovers it
//! from the realized position delta (the "Clip Velocity" toggle).

#![allow(clippy::unnecessary_cast)]

use super::colors;
use super::{
    capsule_style, load_level_mesh, mover_capsule, new_world, CharacterState, RenderCapsule,
    SceneKind, SceneState,
};
use crate::mover_shared::{
    self, pack_mover_user_data, MoverBody, MoverDraw, MoverParams, JUMP_SPEED,
};
use crate::vis::{mesh_triangle_edges_transform, VisBody};
use box3d_rust::body::create_body;
use box3d_rust::geometry::{Capsule, Sphere};
use box3d_rust::height_field::create_wave;
use box3d_rust::hull::{make_box_hull, make_transformed_box_hull};
use box3d_rust::id::ShapeId;
use box3d_rust::joint::create_revolute_joint;
use box3d_rust::math_functions::{
    get_length_and_normalize, mul_sv, offset_pos, Pos, Transform, Vec3, WorldTransform,
    QUAT_IDENTITY, VEC3_AXIS_Y, VEC3_AXIS_Z, VEC3_ONE, VEC3_ZERO,
};
use box3d_rust::mesh::create_torus_mesh;
use box3d_rust::shape::{
    create_capsule_shape, create_height_field_shape, create_hull_shape, create_mesh_shape,
    create_sphere_shape,
};
use box3d_rust::types::{
    default_body_def, default_revolute_joint_def, default_shape_def, BodyType, QueryFilter,
};
use box3d_rust::world::World;
use std::f32::consts::PI;

/// The kinematic `CharacterMover` (C sample.h:294 + sample.cpp:2099). A thin
/// wrapper that owns the physical state + ignore list and delegates the actual
/// integration to the shared [`mover_shared::solve_move`].
pub(crate) struct MoverController {
    pub transform: WorldTransform,
    pub velocity: Vec3,
    pub capsule: Capsule,
    pub pogo_velocity: f32,
    pub on_ground: bool,
    pub sprint: bool,
    /// Shapes the cast/plane query ignores (the `{7,2,-3}` box).
    pub ignore_shapes: Vec<ShapeId>,
    /// Planes + pogo segment gathered by the last SolveMove, for debug draw.
    draw: MoverDraw,
}

impl MoverController {
    fn new(position: Pos) -> Self {
        MoverController {
            transform: WorldTransform {
                p: position,
                q: QUAT_IDENTITY,
            },
            velocity: VEC3_ZERO,
            capsule: mover_capsule(),
            pogo_velocity: 0.0,
            on_ground: false,
            sprint: false,
            ignore_shapes: Vec::new(),
            draw: MoverDraw::default(),
        }
    }

    /// Run the shared C `CharacterMover::SolveMove` (sample.cpp:2154) over this
    /// controller's state. The team-based query filters are the mover level's
    /// (collide with allies, cast ignores them; the pogo skips the ally team).
    fn solve_move(
        &mut self,
        world: &mut World,
        time_step: f32,
        forward: Vec3,
        right: Vec3,
        throttle_x: f32,
        throttle_y: f32,
        clip_velocity: bool,
    ) {
        let mut body = MoverBody {
            position: self.transform.p,
            velocity: self.velocity,
            capsule: self.capsule,
            pogo_velocity: self.pogo_velocity,
            on_ground: self.on_ground,
            sprint: self.sprint,
        };
        let params = MoverParams {
            forward,
            right,
            throttle_x,
            throttle_y,
            clip_velocity,
            ignore_shapes: &self.ignore_shapes,
            // C `skipTeamFilter` — the pogo ray skips the ally team (category 2).
            pogo_filter: QueryFilter {
                category_bits: 1,
                mask_bits: 0xFFFF_FFFD, // ~2u
                id: 0,
                name: String::from("pogo"),
            },
            // C `moverFilter` — collide with allies.
            mover_filter: QueryFilter {
                category_bits: 1,
                mask_bits: 0xFFFF_FFFF, // ~0u
                id: 1,
                name: String::from("mover_collide"),
            },
            // C `castFilter` — the cast ignores allies.
            cast_filter: QueryFilter {
                category_bits: 1,
                mask_bits: 0xFFFF_FFFD, // ~2u
                id: 1,
                name: String::from("mover_cast"),
            },
        };
        mover_shared::solve_move(world, &mut body, &params, &mut self.draw, time_step);
        self.transform.p = body.position;
        self.velocity = body.velocity;
        self.pogo_velocity = body.pogo_velocity;
        self.on_ground = body.on_ground;
        self.sprint = body.sprint;
    }
}

/// Build the BasicMover level (`sample_character.cpp:314`).
pub(crate) fn build_mover(test_map_obj: &str, stairs_obj: &str) -> CharacterState {
    let mut world = new_world();
    let mut bodies: Vec<VisBody> = Vec::new();
    let mut ground_edges: Vec<f32> = Vec::new();

    // --- Level mesh (test_map01) + 3 transformed box hulls on the same body ---
    {
        let mut shape_def = default_shape_def();
        shape_def.materials = super::ground_materials();
        let body_def = default_body_def();
        let body = create_body(&mut world, &body_def);

        if let Some(mesh) = load_level_mesh(test_map_obj) {
            create_mesh_shape(&mut world, body, &shape_def, &mesh, VEC3_ONE);
            ground_edges.extend(mesh_triangle_edges_transform(
                &mesh,
                VEC3_ONE,
                Transform {
                    p: VEC3_ZERO,
                    q: QUAT_IDENTITY,
                },
            ));
        }

        let boxes = [
            (
                Vec3 {
                    x: 4.0,
                    y: 1.0,
                    z: 14.0,
                },
                QUAT_IDENTITY,
            ),
            (
                Vec3 {
                    x: 4.0,
                    y: 1.0,
                    z: 13.95,
                },
                QUAT_IDENTITY,
            ),
            (
                Vec3 {
                    x: 5.8,
                    y: 1.0,
                    z: 13.7,
                },
                box3d_rust::math_functions::make_quat_from_axis_angle(VEC3_AXIS_Y, 0.1 * PI),
            ),
        ];
        let plain_def = default_shape_def();
        for (p, q) in boxes {
            let xf = Transform { p, q };
            let hull = make_transformed_box_hull(1.0, 1.0, 1.0, xf);
            create_hull_shape(&mut world, body, &plain_def, &hull.base);
            bodies.push(VisBody::box_local(body.index1 - 1, 1.0, 1.0, 1.0, xf));
        }
    }

    // --- Stairs mesh at {-10,0,0}, scale {0.75,0.75,-1.5} ---
    {
        let mut body_def = default_body_def();
        body_def.position = Pos {
            x: -10.0,
            y: 0.0,
            z: 0.0,
        };
        let body = create_body(&mut world, &body_def);
        let scale = Vec3 {
            x: 0.75,
            y: 0.75,
            z: -1.5,
        };
        if let Some(mesh) = load_level_mesh(stairs_obj) {
            create_mesh_shape(&mut world, body, &default_shape_def(), &mesh, scale);
            ground_edges.extend(mesh_triangle_edges_transform(
                &mesh,
                scale,
                Transform {
                    p: Vec3 {
                        x: -10.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    q: QUAT_IDENTITY,
                },
            ));
        }
    }

    // --- Torus mesh at {-10,1,-8}, rot axisY 0.5π, scale {-0.75,1.5,0.5} ---
    {
        if let Some(torus) = create_torus_mesh(10, 12, 2.0, 1.0) {
            let q = box3d_rust::math_functions::make_quat_from_axis_angle(VEC3_AXIS_Y, 0.5 * PI);
            let mut body_def = default_body_def();
            body_def.position = Pos {
                x: -10.0,
                y: 1.0,
                z: -8.0,
            };
            body_def.rotation = q;
            let body = create_body(&mut world, &body_def);
            let scale = Vec3 {
                x: -0.75,
                y: 1.5,
                z: 0.5,
            };
            create_mesh_shape(&mut world, body, &default_shape_def(), &torus, scale);
            ground_edges.extend(mesh_triangle_edges_transform(
                &torus,
                scale,
                Transform {
                    p: Vec3 {
                        x: -10.0,
                        y: 1.0,
                        z: -8.0,
                    },
                    q,
                },
            ));
        }
    }

    // --- Wave height field 50×50 at {20,0,0} ---
    {
        let mut shape_def = default_shape_def();
        shape_def.materials = super::ground_materials();
        let mut body_def = default_body_def();
        body_def.position = Pos {
            x: 20.0,
            y: 0.0,
            z: 0.0,
        };
        let body = create_body(&mut world, &body_def);
        let hf = create_wave(50, 50, VEC3_ONE, 0.02, 0.04, true);
        create_height_field_shape(&mut world, body, &shape_def, &hf);
        ground_edges.extend(crate::vis::hf_triangle_edges(
            &hf,
            Vec3 {
                x: 20.0,
                y: 0.0,
                z: 0.0,
            },
        ));
    }

    // --- Enemy capsule at {0,1.4,6}: maxPush 1.0, clip true, mediumVioletRed ---
    {
        let mut body_def = default_body_def();
        body_def.position = Pos {
            x: 0.0,
            y: 1.4,
            z: 6.0,
        };
        let body = create_body(&mut world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.user_data = pack_mover_user_data(1.0, true);
        shape_def.base_material.custom_color = colors::MEDIUM_VIOLET_RED;
        let capsule = enemy_capsule();
        create_capsule_shape(&mut world, body, &shape_def, &capsule);
        bodies.push(VisBody::capsule_body(body.index1 - 1, &capsule));
    }

    // --- Friendly capsule at {0,1.4,5}: maxPush 0.01, clip false, team 2, lime ---
    {
        let mut body_def = default_body_def();
        body_def.position = Pos {
            x: 0.0,
            y: 1.4,
            z: 5.0,
        };
        let body = create_body(&mut world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.user_data = pack_mover_user_data(0.01, false);
        shape_def.filter.category_bits = 2;
        shape_def.filter.mask_bits = 0xFFFF_FFFF;
        shape_def.base_material.custom_color = colors::LIME_GREEN;
        let capsule = enemy_capsule();
        create_capsule_shape(&mut world, body, &shape_def, &capsule);
        bodies.push(VisBody::capsule_body(body.index1 - 1, &capsule));
    }

    // --- Dynamic sphere at {7,5,0} radius 0.5 ---
    {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = Pos {
            x: 7.0,
            y: 5.0,
            z: 0.0,
        };
        let body = create_body(&mut world, &body_def);
        create_sphere_shape(
            &mut world,
            body,
            &default_shape_def(),
            &Sphere {
                center: VEC3_ZERO,
                radius: 0.5,
            },
        );
        bodies.push(VisBody::sphere_body(body.index1 - 1, 0.5));
    }

    // --- Ignore box at {7,2,-3} half {0.5,0.25,0.5} floralWhite ---
    let ignore_shape = {
        let mut body_def = default_body_def();
        body_def.position = Pos {
            x: 7.0,
            y: 2.0,
            z: -3.0,
        };
        let body = create_body(&mut world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.base_material.custom_color = colors::FLORAL_WHITE;
        let hull = make_box_hull(0.5, 0.25, 0.5);
        let sid = create_hull_shape(&mut world, body, &shape_def, &hull.base);
        bodies.push(VisBody::box_body(body.index1 - 1, 0.5, 0.25, 0.5));
        sid
    };

    // --- Spring revolute door at {-2,1.6,0} (sample_character.cpp:469-506) ---
    {
        let ground_id = create_body(&mut world, &default_body_def());

        let door_pos = Pos {
            x: -2.0,
            y: 1.6,
            z: 0.0,
        };
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = door_pos;
        body_def.gravity_scale = 2.0;
        let door_id = create_body(&mut world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.density = 1000.0;
        let hull = make_box_hull(0.75, 1.5, 0.1);
        create_hull_shape(&mut world, door_id, &shape_def, &hull.base);
        bodies.push(VisBody::box_body(door_id.index1 - 1, 0.75, 1.5, 0.1));

        let axis_quat =
            box3d_rust::math_functions::compute_quat_between_unit_vectors(VEC3_AXIS_Z, VEC3_AXIS_Y);
        let offset = Vec3 {
            x: -0.75,
            y: 0.0,
            z: 0.0,
        };

        let mut joint = default_revolute_joint_def();
        joint.base.body_id_a = ground_id;
        joint.base.body_id_b = door_id;
        joint.base.local_frame_a.p = Vec3 {
            x: door_pos.x as f32 + offset.x,
            y: door_pos.y as f32 + offset.y,
            z: door_pos.z as f32 + offset.z,
        };
        joint.base.local_frame_a.q = axis_quat;
        joint.base.local_frame_b.p = offset;
        joint.base.local_frame_b.q = axis_quat;
        joint.enable_limit = true;
        joint.lower_angle = -0.5 * PI; // -90°
        joint.upper_angle = 0.5 * PI; // +90°
        joint.enable_spring = true;
        joint.hertz = 1.0;
        joint.damping_ratio = 0.5;
        joint.enable_motor = false;
        joint.max_motor_torque = 100.0;
        joint.base.draw_scale = 2.0;
        create_revolute_joint(&mut world, &joint);
    }

    let mut mover = MoverController::new(Pos {
        x: 7.5,
        y: 0.75,
        z: 9.0,
    });
    mover.ignore_shapes.push(ignore_shape);

    let mut state = CharacterState::new(SceneKind::Mover, world, SceneState::Mover(mover));
    state.bodies = bodies;
    state.ground_edges = ground_edges;
    state
}

fn enemy_capsule() -> Capsule {
    Capsule {
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
        radius: 0.3,
    }
}

pub(crate) fn step(state: &mut CharacterState, dt: f32, sub_steps: i32) {
    let (jump, want_sprint, throttle_x, throttle_y, forward, right) = {
        let i = &state.input;
        (
            i.jump,
            i.want_sprint,
            i.throttle_x,
            i.throttle_y,
            normalize_horizontal(i.forward),
            normalize_horizontal(i.right),
        )
    };
    let clip = state.clip_velocity;

    // C `CharacterMover::Step`: jump / sprint set before SolveMove, using the
    // previous frame's on-ground state.
    if let SceneState::Mover(m) = &mut state.state {
        if jump && m.on_ground {
            m.velocity.y = JUMP_SPEED;
            m.on_ground = false;
        }
        m.sprint = m.on_ground && want_sprint;
    }

    // SolveMove needs &mut world and &mut mover simultaneously; move the mover out.
    if let SceneState::Mover(mut m) = std::mem::replace(&mut state.state, SceneState::placeholder())
    {
        m.solve_move(
            &mut state.world,
            dt,
            forward,
            right,
            throttle_x,
            throttle_y,
            clip,
        );
        state.state = SceneState::Mover(m);
    }

    state.world.step(dt, sub_steps);

    // --- Render (C BasicMover Step draw block) ---
    // Snapshot the mover before drawing so the immutable borrow of `state.state`
    // does not clash with the mutable `state.draw_*` / `render_capsules` writes.
    let snap = if let SceneState::Mover(m) = &state.state {
        Some((
            m.transform,
            m.capsule,
            m.draw.planes.clone(),
            m.draw.pogo_origin,
            m.draw.pogo_end,
            m.draw.pogo_hit,
            m.velocity,
            m.on_ground,
            m.sprint,
        ))
    } else {
        None
    };

    if let Some((
        transform,
        capsule,
        planes,
        pogo_o,
        pogo_e,
        pogo_hit,
        velocity,
        on_ground,
        sprint,
    )) = snap
    {
        let position = transform.p;

        // Yellow contact-plane points + short normal lines.
        for plane in &planes {
            let p1 = offset_pos(
                position,
                mul_sv(plane.plane.offset - capsule.radius, plane.plane.normal),
            );
            let p1v = pos_vec3(p1);
            state.draw_point(p1v, 5.0, colors::YELLOW);
            state.draw_line(p1v, p1v + mul_sv(0.1, plane.plane.normal), colors::YELLOW);
        }

        // Pogo ray: green on ground, gray in air.
        let pogo_color = if pogo_hit {
            colors::GREEN
        } else {
            colors::GRAY
        };
        state.draw_line(pos_vec3(pogo_o), pos_vec3(pogo_e), pogo_color);

        // Purple velocity vector.
        let pv = pos_vec3(position);
        state.draw_line(pv, pv + velocity, colors::PURPLE);

        // Blue solid mover capsule (C DrawSolidCapsule blue).
        state.render_capsules.push(RenderCapsule {
            transform,
            capsule,
            style: capsule_style(colors::BLUE, 2),
        });

        state.status = vec![
            position.x as f32,
            position.y as f32,
            position.z as f32,
            velocity.x,
            velocity.y,
            velocity.z,
            if on_ground { 1.0 } else { 0.0 },
            if sprint { 1.0 } else { 0.0 },
        ];
    }
}

fn normalize_horizontal(v: Vec3) -> Vec3 {
    let mut len = 0.0;
    let n = get_length_and_normalize(
        &mut len,
        Vec3 {
            x: v.x,
            y: 0.0,
            z: v.z,
        },
    );
    if len < 1e-4 {
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: -1.0,
        }
    } else {
        n
    }
}

fn pos_vec3(p: Pos) -> Vec3 {
    Vec3 {
        x: p.x as f32,
        y: p.y as f32,
        z: p.z as f32,
    }
}

impl SceneState {
    /// A cheap placeholder used while the Mover controller is temporarily moved
    /// out for the `&mut world` + `&mut mover` step.
    fn placeholder() -> SceneState {
        SceneState::Mover(MoverController::new(Pos {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }))
    }
}
