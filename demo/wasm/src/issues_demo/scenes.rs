//! Scene builders for the Issues samples (`sample_issues.cpp`). Each `build_*`
//! reproduces one C constructor's body/shape/joint creation calls exactly and
//! returns a fully-populated [`IssuesState`].
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::ghost_mesh::{self, SRC};
use super::wheel_data::{WHEEL1_HULLS, WHEEL1_VERTS};
use super::{GhostTrack, HullBody, IssuesState, RestitutionTrack};
use crate::vis::{
    hf_triangle_edges, hull_edges, hull_triangles, mesh_triangle_edges_offset, VisBody,
};
use box3d_rust::body::{body_set_angular_velocity, create_body};
use box3d_rust::geometry::Capsule;
use box3d_rust::height_field::create_grid;
use box3d_rust::hull::{create_hull, get_hull_points, make_box_hull, make_offset_box_hull};
use box3d_rust::math_functions::{
    make_quat_from_axis_angle, rotate_vector, Pos, Quat, Vec3, DEG_TO_RAD, VEC3_AXIS_X,
    VEC3_AXIS_Y, VEC3_ONE,
};
use box3d_rust::mesh::{create_grid_mesh, create_mesh, create_platform_mesh, MeshDef};
use box3d_rust::shape::{
    create_capsule_shape, create_height_field_shape, create_hull_shape, create_mesh_shape,
};
use box3d_rust::types::{default_body_def, default_shape_def, BodyType, MotionLocks};
use box3d_rust::world::world_set_contact_tuning;

/// b3_colorMagenta (draw.h) — the Capsule Mesh player capsule's custom color.
const COLOR_MAGENTA: u32 = 0x00FF_00FF;

fn pos(x: f32, y: f32, z: f32) -> Pos {
    Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    }
}

fn vec3(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

/// C `Sample::AddGroundBox( extent )` — ground body at `(0,-1,0)` with an
/// `extent × 1 × extent` box hull, pushed as the first render body (index 0).
fn add_ground_box(state: &mut IssuesState, extent: f32) {
    let mut def = default_body_def();
    def.position = pos(0.0, -1.0, 0.0);
    let ground = create_body(&mut state.world, &def);
    let hull = make_box_hull(extent, 1.0, extent);
    create_hull_shape(&mut state.world, ground, &default_shape_def(), &hull.base);
    state
        .bodies
        .push(VisBody::box_body(ground.index1 - 1, extent, 1.0, extent));
}

/// Crash (:52) — a 20×20 grid mesh ground with two dynamic boxes above it. The
/// "Add Joint" button welds the two boxes (see [`super::issues_add_joint`]).
pub(super) fn build_crash() -> IssuesState {
    let mut state = IssuesState::new();

    // Ground: grid mesh at (0,-1,0).
    let mut bd = default_body_def();
    bd.position = pos(0.0, -1.0, 0.0);
    let ground = create_body(&mut state.world, &bd);
    let grid = create_grid_mesh(20, 20, 2.0, 0, true).expect("grid mesh");
    create_mesh_shape(
        &mut state.world,
        ground,
        &default_shape_def(),
        &grid,
        VEC3_ONE,
    );
    state.static_wire = mesh_triangle_edges_offset(&grid, VEC3_ONE, vec3(0.0, -1.0, 0.0));

    let sd = default_shape_def();
    let box_hull = make_box_hull(0.5, 0.5, 0.5);

    let mut bd = default_body_def();
    bd.type_ = BodyType::Dynamic;
    bd.position = pos(2.0, 4.0, 0.0);
    let body1 = create_body(&mut state.world, &bd);
    create_hull_shape(&mut state.world, body1, &sd, &box_hull.base);
    state
        .bodies
        .push(VisBody::box_body(body1.index1 - 1, 0.5, 0.5, 0.5));

    bd.position = pos(-2.0, 4.0, 0.0);
    let body2 = create_body(&mut state.world, &bd);
    create_hull_shape(&mut state.world, body2, &sd, &box_hull.base);
    state
        .bodies
        .push(VisBody::box_body(body2.index1 - 1, 0.5, 0.5, 0.5));

    state.crash_body1 = body1;
    state.crash_body2 = body2;
    state
}

/// Multiple Prismatic (:118) — six dynamic boxes stacked and chained by prismatic
/// joints (limit ±6, `constraintHertz = 240`, `drawScale = 2`).
pub(super) fn build_multiple_prismatic() -> IssuesState {
    use box3d_rust::joint::create_prismatic_joint;
    use box3d_rust::types::default_prismatic_joint_def;

    let mut state = IssuesState::new();

    let mut bd = default_body_def();
    let ground = create_body(&mut state.world, &bd);

    let sd = default_shape_def();
    let box_hull = make_box_hull(0.5, 0.5, 0.5);
    let mut joint_def = default_prismatic_joint_def();
    joint_def.base.body_id_a = ground;
    joint_def.base.local_frame_a.p = vec3(0.0, 0.0, 0.0);
    joint_def.base.local_frame_b.p = vec3(0.0, -0.6, 0.0);
    joint_def.base.draw_scale = 2.0;
    joint_def.base.constraint_hertz = 240.0;
    joint_def.lower_translation = -6.0;
    joint_def.upper_translation = 6.0;
    joint_def.enable_limit = true;

    for i in 0..6 {
        bd.position = pos(0.0, 0.6 + 1.2 * i as f32, 0.0);
        bd.type_ = BodyType::Dynamic;
        let body = create_body(&mut state.world, &bd);
        create_hull_shape(&mut state.world, body, &sd, &box_hull.base);
        state
            .bodies
            .push(VisBody::box_body(body.index1 - 1, 0.5, 0.5, 0.5));

        joint_def.base.body_id_b = body;
        create_prismatic_joint(&mut state.world, &joint_def);

        joint_def.base.body_id_a = body;
        joint_def.base.local_frame_a.p = vec3(0.0, 0.6, 0.0);
    }

    state
}

/// Hull Crash (:174) — a fixed, nearly-coplanar point set fed through `b3CreateHull`
/// (the active `#elif 1` block, 5 points, scaled by 0.01). When the hull builder
/// rejects the degenerate set the sample draws the raw points instead of a hull —
/// this reproduces that robustness path.
pub(super) fn build_hull_crash() -> IssuesState {
    let mut state = IssuesState::new();

    // C `#elif 1` point set (sample_issues.cpp:200-204).
    let raw = [
        vec3(100.000000, -142.292389, 130.826111),
        vec3(99.5354385, -71.3011093, 130.826111),
        vec3(99.5930862, -80.1112213, -100.000000),
        vec3(100.000000, -142.292389, -100.000000),
        vec3(99.5930862, -80.1112213, 130.826111),
    ];
    let points: Vec<Vec3> = raw
        .iter()
        .map(|p| Vec3 {
            x: 0.01 * p.x,
            y: 0.01 * p.y,
            z: 0.01 * p.z,
        })
        .collect();

    match create_hull(&points, points.len() as i32) {
        Some(hull) => {
            state.hull_crash_ok = true;
            state.hull_crash_tris = hull_triangles(&hull);
            state.hull_crash_edges = hull_edges(&hull);
        }
        None => {
            state.hull_crash_ok = false;
            for p in &points {
                state.hull_crash_points.extend_from_slice(&[p.x, p.y, p.z]);
            }
        }
    }
    state
}

/// One arbitrary-hull body for Convex Jitter: create the hull, attach it, and record
/// its render geometry (fan triangles + wire edges) keyed by body index.
fn add_hull_body(
    state: &mut IssuesState,
    body_def: &box3d_rust::types::BodyDef,
    shape_def: &box3d_rust::types::ShapeDef,
    points: &[Vec3],
) {
    let body = create_body(&mut state.world, body_def);
    let hull = create_hull(points, points.len() as i32).expect("convex-jitter hull");
    create_hull_shape(&mut state.world, body, shape_def, &hull);
    state.hull_bodies.push(HullBody {
        body_index: body.index1 - 1,
        tris: hull_triangles(&hull),
        edges: hull_edges(&hull),
    });
}

/// Convex Jitter (:275) — two precise 16-/18-point hulls at scale 0.01 (a static
/// pad and a dynamic slab with rolling resistance) over a ground box.
pub(super) fn build_convex_jitter() -> IssuesState {
    let mut state = IssuesState::new();
    add_ground_box(&mut state, 10.0);

    let s = 0.01f32;

    // --- Hull 1 (static, 16 points) ---
    {
        let b = vec3(-459.292877, 217.398331, 1.00115335);
        let mut bd = default_body_def();
        bd.position = pos(s * b.x, s * b.z + 2.0, s * b.y);
        bd.rotation = Quat {
            v: vec3(0.0, -0.707106769, 0.0),
            s: 0.707106769,
        };
        let mut raw = [
            vec3(-44.8770714, -91.6598053, -1.92012548),
            vec3(-92.5001831, 51.0151291, 15.8006573),
            vec3(-91.0282211, -9.44371605, 15.6148796),
            vec3(90.2375641, 77.3870087, 15.9356089),
            vec3(-85.5353241, 91.3750992, -1.36629653),
            vec3(88.9092178, -87.2975464, -1.86754704),
            vec3(83.7932816, -89.8572235, 15.4168339),
            vec3(87.0243988, 88.9776535, -1.32423306),
            vec3(-91.6564941, -85.4949493, 15.3782759),
            vec3(-90.2922516, -87.2074127, -1.92012548),
            vec3(-87.2944870, 89.9510498, 15.9215889),
            vec3(79.2338104, 89.9690781, 15.9724140),
            vec3(-91.6744461, 81.0823212, -1.39959598),
            vec3(90.3452759, -76.4459610, 15.4588966),
            vec3(-87.4021912, -89.2263107, 15.3677588),
            vec3(76.3258057, 92.0059967, 1.82873762),
        ];
        for p in raw.iter_mut() {
            *p = vec3(s * p.x, s * p.z, s * p.y);
        }
        add_hull_body(&mut state, &bd, &default_shape_def(), &raw);
    }

    // --- Hull 2 (dynamic, 18 points, rolling resistance 0.1) ---
    {
        let b = vec3(-402.321838, 157.310364, 16.8169250);
        let mut bd = default_body_def();
        bd.position = pos(s * b.x, s * b.z + 2.0, s * b.y);
        bd.rotation = Quat {
            v: vec3(0.0, -0.00152086187, 0.0),
            s: 0.999998868,
        };
        bd.type_ = BodyType::Dynamic;
        let mut sd = default_shape_def();
        sd.base_material.rolling_resistance = 0.1;
        let mut raw = [
            vec3(29.5000000, 17.1488495, 0.175081104),
            vec3(29.5000000, -17.2990532, 0.125000000),
            vec3(29.4840164, -17.3057766, 24.0200863),
            vec3(29.4840164, 17.1648350, 24.1781254),
            vec3(-29.1345520, 17.5529804, 0.125000000),
            vec3(-29.1345520, 17.5529804, 23.7899799),
            vec3(-29.1441040, 16.9679585, 24.3750000),
            vec3(-29.1345520, -17.2990532, 24.3750000),
            vec3(-29.1345520, -17.2990532, 0.175081253),
            vec3(29.0720215, 17.5529785, 0.125000000),
            vec3(29.0859070, 17.5629406, 23.8120594),
            vec3(29.1401348, -17.2990532, 24.3750000),
            vec3(29.1123581, 16.9722290, 24.4027710),
            vec3(29.3944912, 17.2543602, 24.1206398),
            vec3(-29.1345520, -17.2990532, 24.0759430),
            vec3(-29.1345520, -16.9722252, 24.4027710),
            vec3(29.1123619, -16.9722271, 24.4027729),
            vec3(29.5000000, 17.3429642, 24.0000000),
        ];
        for p in raw.iter_mut() {
            *p = vec3(s * p.x, s * p.z, s * p.y);
        }
        add_hull_body(&mut state, &bd, &sd, &raw);
    }

    state
}

/// s&box mover (:385) — a 40×40 height-field grid plus a `b3CreatePlatformMesh`
/// under an angular-locked box that drops onto them.
pub(super) fn build_sbox_mover() -> IssuesState {
    let mut state = IssuesState::new();

    // Ground 1: height field at (-10, 0, -10).
    let hf_origin = vec3(-10.0, 0.0, -10.0);
    let mut bd = default_body_def();
    bd.position = pos(hf_origin.x, hf_origin.y, hf_origin.z);
    let ground1 = create_body(&mut state.world, &bd);
    let hf = create_grid(40, 40, vec3(0.5, 1.0, 0.5), false);
    create_height_field_shape(&mut state.world, ground1, &default_shape_def(), &hf);
    state.static_wire = hf_triangle_edges(&hf, hf_origin);

    // Ground 2: platform mesh at the origin.
    let bd = default_body_def();
    let ground2 = create_body(&mut state.world, &bd);
    let platform = create_platform_mesh(vec3(0.0, 0.5, 0.0), 1.0, 2.0, 5.0).expect("platform mesh");
    create_mesh_shape(
        &mut state.world,
        ground2,
        &default_shape_def(),
        &platform,
        VEC3_ONE,
    );
    state.static_wire.extend(mesh_triangle_edges_offset(
        &platform,
        VEC3_ONE,
        vec3(0.0, 0.0, 0.0),
    ));

    // Angular-locked dynamic box.
    let mut bd = default_body_def();
    bd.type_ = BodyType::Dynamic;
    bd.position = pos(0.0, 3.5, 0.0);
    bd.motion_locks = MotionLocks {
        angular_x: true,
        angular_y: true,
        angular_z: true,
        ..MotionLocks::default()
    };
    bd.enable_contact_recycling = false;
    let body = create_body(&mut state.world, &bd);
    let box_hull = make_box_hull(0.25, 1.0, 0.25);
    create_hull_shape(&mut state.world, body, &default_shape_def(), &box_hull.base);
    state
        .bodies
        .push(VisBody::box_body(body.index1 - 1, 0.25, 1.0, 0.25));

    state
}

/// Capsule Mesh (:463) — the player-controller repro: `building.obj` on a big ground
/// box with a locked, sleepless magenta capsule dropped above it.
pub(super) fn build_capsule_mesh() -> IssuesState {
    let mut state = IssuesState::new();

    // Ground plane (box hull 50 × 0.1 × 50).
    let bd = default_body_def();
    let ground = create_body(&mut state.world, &bd);
    let ground_hull = make_box_hull(50.0, 0.1, 50.0);
    create_hull_shape(
        &mut state.world,
        ground,
        &default_shape_def(),
        &ground_hull.base,
    );
    state
        .bodies
        .push(VisBody::box_body(ground.index1 - 1, 50.0, 0.1, 50.0));

    // Building mesh at (0, 0.1, 0) — the shipped building.obj (embedded in wasm).
    let building = crate::obj_loader::load_building_mesh();
    let mut bd = default_body_def();
    bd.position = pos(0.0, 0.1, 0.0);
    let building_body = create_body(&mut state.world, &bd);
    create_mesh_shape(
        &mut state.world,
        building_body,
        &default_shape_def(),
        &building,
        VEC3_ONE,
    );
    state.static_wire = mesh_triangle_edges_offset(&building, VEC3_ONE, vec3(0.0, 0.1, 0.0));

    // Locked, sleepless capsule (player-controller setup).
    let mut bd = default_body_def();
    bd.type_ = BodyType::Dynamic;
    bd.position = pos(0.0, 4.0, 10.0);
    bd.motion_locks = MotionLocks {
        angular_x: true,
        angular_y: true,
        angular_z: true,
        ..MotionLocks::default()
    };
    bd.enable_sleep = false;
    bd.enable_contact_recycling = false;
    let body = create_body(&mut state.world, &bd);
    let mut sd = default_shape_def();
    sd.base_material.friction = 0.3;
    sd.base_material.custom_color = COLOR_MAGENTA;
    let capsule = Capsule {
        center1: vec3(0.0, -0.5, 0.0),
        center2: vec3(0.0, 0.5, 0.0),
        radius: 0.3,
    };
    create_capsule_shape(&mut state.world, body, &sd, &capsule);
    state.bodies.push(VisBody::capsule_colored(
        body.index1 - 1,
        &capsule,
        COLOR_MAGENTA,
    ));

    state
}

/// Restitution Overshoot (:1149) — a restitution-1.0 box dropped 10 m onto a small
/// static floor. The HUD tracks the bounce height and flags PASS/FAIL against the
/// drop height (a perfectly elastic bounce must not exceed it). All values match C.
pub(super) fn build_restitution_overshoot() -> IssuesState {
    let mut state = IssuesState::new();

    // RestitutionOvershoot constants (sample_issues.cpp:1152-1156).
    let box_half = 0.5f32;
    let floor_half_xz = 0.375f32;
    let floor_half_y = 0.25f32;
    let drop_height = 10.0f32;
    let tolerance = 0.05f32;

    // Static floor box at (0, -floorHalfY, 0).
    let mut floor_def = default_body_def();
    floor_def.position = pos(0.0, -floor_half_y, 0.0);
    let floor_body = create_body(&mut state.world, &floor_def);
    let floor = make_box_hull(floor_half_xz, floor_half_y, floor_half_xz);
    create_hull_shape(
        &mut state.world,
        floor_body,
        &default_shape_def(),
        &floor.base,
    );
    state.bodies.push(VisBody::box_body(
        floor_body.index1 - 1,
        floor_half_xz,
        floor_half_y,
        floor_half_xz,
    ));

    // Dynamic box with restitution 1.0, dropped from y = dropHeight.
    let mut box_def = default_body_def();
    box_def.type_ = BodyType::Dynamic;
    box_def.position = pos(0.0, drop_height, 0.0);
    let box_body = create_body(&mut state.world, &box_def);
    let box_hull = make_box_hull(box_half, box_half, box_half);
    let mut box_shape = default_shape_def();
    box_shape.base_material.restitution = 1.0;
    create_hull_shape(&mut state.world, box_body, &box_shape, &box_hull.base);
    state.bodies.push(VisBody::box_body(
        box_body.index1 - 1,
        box_half,
        box_half,
        box_half,
    ));

    state.resti = Some(RestitutionTrack {
        box_body,
        drop_height,
        box_half,
        tolerance,
        current_y: drop_height,
        max_bounce_y: 0.0,
        bounced: false,
        failed: false,
    });
    state
}

/// Slide Twist Off Center Shape (:1256) — an off-center box hull spun about the
/// tilted Y of a 20° inclined plane. The box is created from `b3MakeOffsetBoxHull`
/// (local center `{1, 0.5, 1}`) and given 25 rad/s about the tilted Y at spawn; it
/// rides the Issues arbitrary-hull render channel so the offset geometry draws
/// correctly. All values match C.
pub(super) fn build_slide_twist_off_center() -> IssuesState {
    let mut state = IssuesState::new();
    add_ground_box(&mut state, 50.0);

    // orientation = quat about +X by 20 degrees (sample_issues.cpp:1268).
    let orientation = make_quat_from_axis_angle(VEC3_AXIS_X, 20.0 * DEG_TO_RAD);

    // Static inclined plane at (0, 4, 0), 10 × 0.5 × 10 box hull, friction 0.6.
    let mut plane_def = default_body_def();
    plane_def.position = pos(0.0, 4.0, 0.0);
    plane_def.rotation = orientation;
    let plane_body = create_body(&mut state.world, &plane_def);
    let plane = make_box_hull(10.0, 0.5, 10.0);
    let mut plane_shape = default_shape_def();
    plane_shape.base_material.friction = 0.6;
    create_hull_shape(&mut state.world, plane_body, &plane_shape, &plane.base);
    // The plane is a centered box, so a box render body draws it correctly.
    state
        .bodies
        .push(VisBody::box_body(plane_body.index1 - 1, 10.0, 0.5, 10.0));

    // Dynamic off-center box: local center {1, 0.5, 1}, placed so its center sits
    // above the plane, spun at 25 rad/s about the tilted Y.
    let box_local_center = vec3(1.0, 0.5, 1.0);
    let box_offset = rotate_vector(orientation, box_local_center);

    let mut box_def = default_body_def();
    box_def.type_ = BodyType::Dynamic;
    box_def.position = pos(-box_offset.x, 5.0 - box_offset.y, -box_offset.z);
    box_def.rotation = orientation;
    let box_body = create_body(&mut state.world, &box_def);
    let m_box = make_offset_box_hull(1.0, 0.5, 1.0, box_local_center);
    let mut box_shape = default_shape_def();
    box_shape.base_material.friction = 0.3;
    create_hull_shape(&mut state.world, box_body, &box_shape, &m_box.base);
    state.hull_bodies.push(HullBody {
        body_index: box_body.index1 - 1,
        tris: hull_triangles(&m_box.base),
        edges: hull_edges(&m_box.base),
    });

    let spin_axis = rotate_vector(orientation, VEC3_AXIS_Y);
    body_set_angular_velocity(
        &mut state.world,
        box_body,
        vec3(25.0 * spin_axis.x, 25.0 * spin_axis.y, 25.0 * spin_axis.z),
    );

    state
}

/// GMod Wheel Stack (:1051) — 30 stacked metal_wheel1 props. Each wheel's 37-piece
/// convex decomposition is rebuilt into a single wrapping convex hull (the union of
/// every piece hull's output points, matching C's `buffer` accumulation), which is
/// what actually improves the simulation. `b3World_SetContactTuning(240, 10, 3)` lets
/// body-pair contact merging settle and sleep the stack. All values match C.
pub(super) fn build_wheel_stack() -> IssuesState {
    let mut state = IssuesState::new();
    add_ground_box(&mut state, 10.0);

    // Rebuild each decomposition piece hull and accumulate its output points, exactly
    // like C: `b3CreateHull` per span, then gather `b3GetHullPoints` up to N = 512.
    const N: usize = 512;
    let mut buffer: Vec<Vec3> = Vec::with_capacity(N);
    for &(offset, count) in WHEEL1_HULLS.iter() {
        let piece = create_hull(&WHEEL1_VERTS[offset..offset + count], count as i32)
            .expect("wheel piece hull");
        for &p in get_hull_points(&piece) {
            if buffer.len() >= N {
                break;
            }
            buffer.push(p);
        }
    }

    // A single hull that wraps the input hulls.
    let wheel_hull = create_hull(&buffer, buffer.len() as i32).expect("wheel wrapping hull");
    let tris = hull_triangles(&wheel_hull);
    let edges = hull_edges(&wheel_hull);

    let height = 0.171f32;
    let spacing = height + 0.006;
    let start_y = 0.5 * height + 0.004;

    let mut sd = default_shape_def();
    sd.base_material.friction = 0.6;

    let wheel_count = 30;
    for i in 0..wheel_count {
        let mut bd = default_body_def();
        bd.type_ = BodyType::Dynamic;
        bd.name = "wheel".to_string();
        bd.position = pos(0.0, start_y + i as f32 * spacing, 0.0);
        let body = create_body(&mut state.world, &bd);
        create_hull_shape(&mut state.world, body, &sd, &wheel_hull);
        state.hull_bodies.push(HullBody {
            body_index: body.index1 - 1,
            tris: tris.clone(),
            edges: edges.clone(),
        });
    }

    world_set_contact_tuning(&mut state.world, 240.0, 10.0, 3.0);
    state
}

/// s&box Ghost Collisions (:508) — a procedural two-chunk mesh floor with a
/// velocity-driven, fixed-rotation character. The two chunks meet at x = 0, each its
/// own body and mesh shape so seam contacts live in separate contact pairs like s&box
/// world chunks. The character body and its ghost-launch tracking match C exactly;
/// the per-step velocity control + launch detection live in [`super`].
pub(super) fn build_sbox_ghost() -> IssuesState {
    let mut state = IssuesState::new();

    // Two chunks meeting at x = 0 (C `CreateFloorChunk`), bounds in s&box inches.
    create_floor_chunk(&mut state, 0, -ghost_mesh::HALF_LENGTH_INCHES, 0.0);
    create_floor_chunk(&mut state, 1, 0.0, ghost_mesh::HALF_LENGTH_INCHES);

    // Character: s&box player — 16-wide zero-radius box hull, 72 tall, mass 500.
    let body_half_width = 16.0 * SRC;
    let body_half_height = 36.0 * SRC;
    let character_mass = 500.0f32;
    let walk_range_x = 3.5f32;

    let mut bd = default_body_def();
    bd.type_ = BodyType::Dynamic;
    bd.position = pos(-walk_range_x, body_half_height + 0.1, 0.0);
    bd.motion_locks = MotionLocks {
        angular_x: true,
        angular_y: true,
        angular_z: true,
        ..MotionLocks::default()
    };
    bd.enable_sleep = false;
    bd.enable_contact_recycling = false;
    bd.gravity_scale = 2.03; // s&box gravity: 800 inch/s^2
    bd.name = "character".to_string();
    let character = create_body(&mut state.world, &bd);

    let mut sd = default_shape_def();
    sd.base_material.friction = 0.0;
    sd.base_material.restitution = 0.0;
    let volume = 8.0 * body_half_width * body_half_height * body_half_width;
    sd.density = character_mass / volume;
    sd.enable_speculative_contact = false;

    let box_hull = make_box_hull(body_half_width, body_half_height, body_half_width);
    create_hull_shape(&mut state.world, character, &sd, &box_hull.base);
    state.bodies.push(VisBody::box_body(
        character.index1 - 1,
        body_half_width,
        body_half_height,
        body_half_width,
    ));

    state.ghost = Some(GhostTrack {
        character,
        body_half_height,
        walk_direction_x: 1.0,
        walk_direction_z: 1.0,
        walk_speed_x: 350.0 * SRC, // s&box run speed
        walk_speed_z: 20.0 * SRC,
        launch_count: 0,
        max_launch_speed: 0.0,
        was_launched: false,
        launch_markers: Vec::new(),
        vertical_velocity: 0.0,
    });
    state
}

/// One ghost-collision floor chunk: build the procedural triangle soup, cook it into a
/// mesh (weld tolerance == `B3_LINEAR_SLOP`, identify edges, same as s&box), attach it
/// as a static mesh shape, and add its welded triangle edges to the static wireframe.
fn create_floor_chunk(state: &mut IssuesState, chunk: i32, x0u: f32, x1u: f32) {
    let (vertices, indices) = ghost_mesh::create_floor_chunk(chunk, x0u, x1u);

    let mesh_def = MeshDef {
        vertices,
        indices,
        weld_vertices: true,
        weld_tolerance: 0.005, // == B3_LINEAR_SLOP, same as s&box
        identify_edges: true,
        ..MeshDef::default()
    };
    let mesh = create_mesh(&mesh_def, None).expect("ghost floor chunk mesh");

    let bd = default_body_def();
    let body = create_body(&mut state.world, &bd);
    create_mesh_shape(&mut state.world, body, &default_shape_def(), &mesh, VEC3_ONE);

    state
        .static_wire
        .extend(mesh_triangle_edges_offset(&mesh, VEC3_ONE, vec3(0.0, 0.0, 0.0)));
}
