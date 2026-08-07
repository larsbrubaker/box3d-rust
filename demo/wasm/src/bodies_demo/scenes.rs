//! Scene constructors for the Bodies category (`sample_bodies.cpp`). Each returns
//! a fully-built [`BodiesState`]; every constructor value is taken verbatim from
//! the C sample it ports (line references in comments). A submodule of
//! `bodies_demo`, split off to keep each file under the 800-line module gate.

use super::{new_world, BodiesState, SceneKind};
use crate::vis::{hull_edges, hull_triangles, VisBody};
use box3d_rust::body::{
    body_apply_mass_from_shapes, body_get_local_point, body_get_local_rotational_inertia,
    body_get_mass, body_get_mass_data, body_set_angular_velocity, body_set_mass_data, create_body,
};
use box3d_rust::geometry::{Capsule, MassData, Sphere};
use box3d_rust::hull::{create_cylinder, create_hull, make_box_hull, make_transformed_box_hull};
use box3d_rust::id::BodyId;
use box3d_rust::joint::{create_prismatic_joint, create_revolute_joint, create_weld_joint};
use box3d_rust::math_functions::{
    add_mm, compute_cos_sin, length, make_quat_from_axis_angle, normalize, rotate_vector, steiner,
    Pos, Transform, Vec3, WorldTransform, PI, QUAT_IDENTITY, VEC3_AXIS_X, VEC3_AXIS_Z, VEC3_ZERO,
};
use box3d_rust::shape::{create_capsule_shape, create_hull_shape, create_sphere_shape};
use box3d_rust::types::{
    default_body_def, default_prismatic_joint_def, default_revolute_joint_def, default_shape_def,
    default_weld_joint_def, BodyType,
};
use box3d_rust::world::world_get_gravity;

fn pos(x: f32, y: f32, z: f32) -> Pos {
    Pos {
        x: x as _,
        y: y as _,
        z: z as _,
    }
}

fn v3(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3 { x, y, z }
}

/// C `Sample::AddGroundBox` (sample.cpp:543): static body at (0,-1,0), box hull
/// (extent, 1, extent). Pushes the render body and returns the ground id.
fn add_ground_box(state: &mut BodiesState, extent: f32) -> BodyId {
    let mut body_def = default_body_def();
    body_def.position = pos(0.0, -1.0, 0.0);
    let ground = create_body(&mut state.world, &body_def);
    let hull = make_box_hull(extent, 1.0, extent);
    create_hull_shape(&mut state.world, ground, &default_shape_def(), &hull.base);
    state
        .vis
        .push(VisBody::box_body(ground.index1 - 1, extent, 1.0, extent));
    ground
}

/// Dispatch a scene id (see `bodies_reset`) to its constructor.
pub fn build(scene: u32) -> BodiesState {
    match scene {
        1 => spinning_book(),
        2 => gyroscopic(),
        3 => weeble(),
        4 => disable(),
        5 => cast(),
        6 => kinematic(),
        7 => lock_mixing(),
        8 => fixed_rotation(),
        9 => gyroscopic_precession(),
        10 => class_ring(),
        _ => body_type(),
    }
}

/// Body Type (sample_bodies.cpp:11-267).
fn body_type() -> BodiesState {
    let mut st = BodiesState::base(new_world(), SceneKind::BodyType);
    st.body_type = BodyType::Dynamic;
    st.is_enabled = true;
    st.speed = 3.0;

    let ground = add_ground_box(&mut st, 20.0);

    // attach1: dynamic box (0.5, 2, 0.5) at (-2, 3, 0).
    let mut bd = default_body_def();
    bd.type_ = BodyType::Dynamic;
    bd.position = pos(-2.0, 3.0, 0.0);
    st.bt_attach1 = create_body(&mut st.world, &bd);
    let box_a = make_box_hull(0.5, 2.0, 0.5);
    let mut sd = default_shape_def();
    sd.density = 1.0;
    create_hull_shape(&mut st.world, st.bt_attach1, &sd, &box_a.base);
    st.vis
        .push(VisBody::box_body(st.bt_attach1.index1 - 1, 0.5, 2.0, 0.5));

    // attach2: type = m_type, box (0.5, 2, 0.5) at (3, 3, 0).
    let mut bd = default_body_def();
    bd.type_ = st.body_type;
    bd.is_enabled = st.is_enabled;
    bd.position = pos(3.0, 3.0, 0.0);
    st.bt_attach2 = create_body(&mut st.world, &bd);
    create_hull_shape(&mut st.world, st.bt_attach2, &sd, &box_a.base);
    st.vis
        .push(VisBody::box_body(st.bt_attach2.index1 - 1, 0.5, 2.0, 0.5));

    // platform: type = m_type, transformed box at (-4, 5, 0), density 2.
    let mut bd = default_body_def();
    bd.type_ = st.body_type;
    bd.is_enabled = st.is_enabled;
    bd.position = pos(-4.0, 5.0, 0.0);
    st.bt_platform = create_body(&mut st.world, &bd);
    let plat_local = Transform {
        p: v3(4.0, 0.0, 0.0),
        q: make_quat_from_axis_angle(VEC3_AXIS_Z, 0.5 * PI),
    };
    let plat_hull = make_transformed_box_hull(0.5, 4.0, 0.5, plat_local);
    let mut sd2 = default_shape_def();
    sd2.density = 2.0;
    create_hull_shape(&mut st.world, st.bt_platform, &sd2, &plat_hull.base);
    st.vis.push(VisBody::box_local(
        st.bt_platform.index1 - 1,
        0.5,
        4.0,
        0.5,
        plat_local,
    ));

    // Revolute joints attach1↔platform (pivot -2,5,0) and attach2↔platform (3,5,0).
    let mut rev = default_revolute_joint_def();
    let pivot = pos(-2.0, 5.0, 0.0);
    rev.base.body_id_a = st.bt_attach1;
    rev.base.body_id_b = st.bt_platform;
    rev.base.local_frame_a.p = body_get_local_point(&st.world, st.bt_attach1, pivot);
    rev.base.local_frame_b.p = body_get_local_point(&st.world, st.bt_platform, pivot);
    rev.max_motor_torque = 50.0;
    rev.enable_motor = true;
    create_revolute_joint(&mut st.world, &rev);

    let mut rev2 = default_revolute_joint_def();
    let pivot2 = pos(3.0, 5.0, 0.0);
    rev2.base.body_id_a = st.bt_attach2;
    rev2.base.body_id_b = st.bt_platform;
    rev2.base.local_frame_a.p = body_get_local_point(&st.world, st.bt_attach2, pivot2);
    rev2.base.local_frame_b.p = body_get_local_point(&st.world, st.bt_platform, pivot2);
    rev2.max_motor_torque = 50.0;
    rev2.enable_motor = true;
    create_revolute_joint(&mut st.world, &rev2);

    // Prismatic joint ground↔platform (anchor 0,5,0), horizontal limit ±10.
    let mut pri = default_prismatic_joint_def();
    let anchor = pos(0.0, 5.0, 0.0);
    pri.base.body_id_a = ground;
    pri.base.body_id_b = st.bt_platform;
    pri.base.local_frame_a.p = body_get_local_point(&st.world, ground, anchor);
    pri.base.local_frame_b.p = body_get_local_point(&st.world, st.bt_platform, anchor);
    pri.max_motor_force = 1000.0;
    pri.motor_speed = 0.0;
    pri.enable_motor = true;
    pri.lower_translation = -10.0;
    pri.upper_translation = 10.0;
    pri.enable_limit = true;
    create_prismatic_joint(&mut st.world, &pri);

    // crate1: dynamic box (0.75³) at (-3, 8, 0), density 2.
    let cube = make_box_hull(0.75, 0.75, 0.75);
    let mut bd = default_body_def();
    bd.type_ = BodyType::Dynamic;
    bd.position = pos(-3.0, 8.0, 0.0);
    let crate1 = create_body(&mut st.world, &bd);
    create_hull_shape(&mut st.world, crate1, &sd2, &cube.base);
    st.vis
        .push(VisBody::box_body(crate1.index1 - 1, 0.75, 0.75, 0.75));

    // crate2 (second payload): type = m_type, box (0.75³) at (2, 8, 0).
    let mut bd = default_body_def();
    bd.type_ = st.body_type;
    bd.is_enabled = st.is_enabled;
    bd.position = pos(2.0, 8.0, 0.0);
    st.bt_payload2 = create_body(&mut st.world, &bd);
    create_hull_shape(&mut st.world, st.bt_payload2, &sd2, &cube.base);
    st.vis.push(VisBody::box_body(
        st.bt_payload2.index1 - 1,
        0.75,
        0.75,
        0.75,
    ));

    // debris (touching): type = m_type, capsule at (8, 0.2, 0), density 2.
    let mut bd = default_body_def();
    bd.type_ = st.body_type;
    bd.is_enabled = st.is_enabled;
    bd.position = pos(8.0, 0.2, 0.0);
    st.bt_touching = create_body(&mut st.world, &bd);
    let capsule = Capsule {
        center1: v3(0.0, 0.0, 0.0),
        center2: v3(1.0, 0.0, 0.0),
        radius: 0.25,
    };
    create_capsule_shape(&mut st.world, st.bt_touching, &sd2, &capsule);
    st.vis
        .push(VisBody::capsule_body(st.bt_touching.index1 - 1, &capsule));

    // floater: type = m_type, sphere at (-8, 12, 0), gravityScale 0.
    let mut bd = default_body_def();
    bd.type_ = st.body_type;
    bd.is_enabled = st.is_enabled;
    bd.position = pos(-8.0, 12.0, 0.0);
    bd.gravity_scale = 0.0;
    st.bt_floating = create_body(&mut st.world, &bd);
    let sphere = Sphere {
        center: v3(0.0, 0.5, 0.0),
        radius: 0.25,
    };
    create_sphere_shape(&mut st.world, st.bt_floating, &sd2, &sphere);
    st.vis.push(VisBody::sphere_local(
        st.bt_floating.index1 - 1,
        0.25,
        Transform {
            p: v3(0.0, 0.5, 0.0),
            q: QUAT_IDENTITY,
        },
    ));

    st
}

/// Spinning Book (sample_bodies.cpp:271-315): three gravity-free books spinning
/// about x / y / z respectively.
fn spinning_book() -> BodiesState {
    let mut st = BodiesState::base(new_world(), SceneKind::SpinningBook);
    add_ground_box(&mut st, 10.0);

    let book = make_box_hull(0.35, 0.08, 0.5);
    let sd = default_shape_def();
    let specs = [
        (-2.0f32, v3(5.0, 0.01, 0.01)),
        (0.0, v3(0.01, 5.0, 0.01)),
        (2.0, v3(0.01, 0.01, -5.0)),
    ];
    for (x, av) in specs {
        let mut bd = default_body_def();
        bd.type_ = BodyType::Dynamic;
        bd.gravity_scale = 0.0;
        bd.position = pos(x, 2.0, 0.0);
        bd.angular_velocity = av;
        let id = create_body(&mut st.world, &bd);
        create_hull_shape(&mut st.world, id, &sd, &book.base);
        st.vis
            .push(VisBody::box_body(id.index1 - 1, 0.35, 0.08, 0.5));
    }
    st
}

/// Gyroscopic Torque / Dzhanibekov effect (sample_bodies.cpp:320-368).
fn gyroscopic() -> BodiesState {
    let mut st = BodiesState::base(new_world(), SceneKind::Gyroscopic);
    add_ground_box(&mut st, 20.0);

    let mut bd = default_body_def();
    bd.type_ = BodyType::Dynamic;
    bd.position = pos(0.0, 2.0, 0.0);
    bd.rotation = make_quat_from_axis_angle(VEC3_AXIS_X, -0.5 * PI);
    bd.gravity_scale = 0.0;
    st.gyro_body = create_body(&mut st.world, &bd);

    let mut sd = default_shape_def();
    sd.update_body_mass = false;
    let cylinder = create_cylinder(0.6, 0.15, 0.0, 32).expect("gyro cylinder");
    let plate = make_box_hull(1.0, 0.05, 0.1);
    create_hull_shape(&mut st.world, st.gyro_body, &sd, &cylinder);
    create_hull_shape(&mut st.world, st.gyro_body, &sd, &plate.base);
    body_apply_mass_from_shapes(&mut st.world, st.gyro_body);
    body_set_angular_velocity(&mut st.world, st.gyro_body, v3(0.01, 0.01, 10.0));

    let idx = st.gyro_body.index1 - 1;
    st.vis.push(VisBody::cylinder_local(
        idx,
        0.15,
        0.3,
        Transform {
            p: v3(0.0, 0.3, 0.0),
            q: QUAT_IDENTITY,
        },
        0,
    ));
    st.vis.push(VisBody::box_body(idx, 1.0, 0.05, 0.1));
    st
}

/// Weeble (sample_bodies.cpp:442-539): a bottom-heavy capsule that rights itself.
fn weeble() -> BodiesState {
    let mut st = BodiesState::base(new_world(), SceneKind::Weeble);
    add_ground_box(&mut st, 30.0);

    let mut bd = default_body_def();
    bd.type_ = BodyType::Dynamic;
    bd.position = pos(0.0, 3.0, 0.0);
    st.weeble = create_body(&mut st.world, &bd);

    let capsule = Capsule {
        center1: v3(0.0, -1.0, 0.0),
        center2: v3(0.0, 1.0, 0.0),
        radius: 1.0,
    };
    let mut sd = default_shape_def();
    sd.base_material.rolling_resistance = 0.1;
    create_capsule_shape(&mut st.world, st.weeble, &sd, &capsule);
    st.vis
        .push(VisBody::capsule_body(st.weeble.index1 - 1, &capsule));

    // Shift the center of mass down 1.5 m via the parallel-axis theorem.
    let mass = body_get_mass(&st.world, st.weeble);
    let inertia = body_get_local_rotational_inertia(&st.world, st.weeble);
    let offset = v3(0.0, -1.5, 0.0);
    let inertia = add_mm(inertia, steiner(mass, offset));
    body_set_mass_data(
        &mut st.world,
        st.weeble,
        MassData {
            mass,
            center: offset,
            inertia,
        },
    );

    st.explosion_position = pos(0.0, -0.1, 0.0);
    st.explosion_radius = 8.0;
    st.explosion_magnitude = 20000.0;
    st
}

/// Disable (sample_bodies.cpp:543-648): a welded chain plus a ball, both
/// enable/disable toggleable at runtime.
fn disable() -> BodiesState {
    let mut st = BodiesState::base(new_world(), SceneKind::Disable);
    add_ground_box(&mut st, 20.0);

    let e_count = 4;
    let link_radius = 0.1f32;
    let link_length = 5.0 * link_radius;
    let capsule = Capsule {
        center1: v3(0.0, 0.0, 0.0),
        center2: v3(0.0, -link_length, 0.0),
        radius: link_radius,
    };
    let sd = default_shape_def();

    let mut parent: BodyId = BodyId::default();
    for link in 0..e_count {
        let mut bd = default_body_def();
        bd.position = pos(0.0, (e_count as f32 - link as f32) * link_length + 1.0, 0.0);
        bd.type_ = if parent.is_null() {
            BodyType::Kinematic
        } else {
            BodyType::Dynamic
        };
        let child = create_body(&mut st.world, &bd);
        create_capsule_shape(&mut st.world, child, &sd, &capsule);
        st.vis
            .push(VisBody::capsule_body(child.index1 - 1, &capsule));
        st.disable_ids[link as usize] = child;

        if parent.is_non_null() {
            let mut jd = default_weld_joint_def();
            jd.base.body_id_a = parent;
            jd.base.body_id_b = child;
            jd.base.local_frame_a.p = v3(0.0, -link_length, 0.0);
            jd.angular_hertz = 10.0;
            jd.angular_damping_ratio = 1.0;
            create_weld_joint(&mut st.world, &jd);
        }
        parent = child;
    }

    let mut bd = default_body_def();
    bd.type_ = BodyType::Dynamic;
    bd.position = pos(3.0, 3.0, 0.0);
    st.ball = create_body(&mut st.world, &bd);
    let sphere = Sphere {
        center: v3(0.0, 0.0, 0.0),
        radius: 0.5,
    };
    create_sphere_shape(&mut st.world, st.ball, &sd, &sphere);
    st.vis.push(VisBody::sphere_body(st.ball.index1 - 1, 0.5));
    st
}

/// Cast (sample_bodies.cpp:652-835): a spinning kinematic cylinder plus a
/// mouse-draggable cast target; the Step recomputes ray / shape / mover queries.
fn cast() -> BodiesState {
    let mut st = BodiesState::base(new_world(), SceneKind::Cast);

    let mut bd = default_body_def();
    bd.type_ = BodyType::Kinematic;
    bd.position = pos(5.0, 5.0, 0.0);
    bd.angular_velocity = v3(0.1, -0.1, 0.1);
    st.cast_body = create_body(&mut st.world, &bd);

    st.cast_cyl_height = 2.0;
    st.cast_cyl_radius = 0.5;
    let cylinder = create_cylinder(2.0, 0.5, 0.0, 16).expect("cast cylinder");
    create_hull_shape(&mut st.world, st.cast_body, &default_shape_def(), &cylinder);
    st.vis.push(VisBody::cylinder_local(
        st.cast_body.index1 - 1,
        0.5,
        1.0,
        Transform {
            p: v3(0.0, 1.0, 0.0),
            q: QUAT_IDENTITY,
        },
        0,
    ));

    st.cast_transform = WorldTransform {
        p: pos(-10.0, 2.0, 0.0),
        q: make_quat_from_axis_angle(normalize(v3(1.0, -2.0, 3.0)), 0.75 * PI),
    };
    st
}

/// Kinematic (sample_bodies.cpp:840-913): drive a kinematic box along a target
/// Lissajous path via SetTargetTransform.
fn kinematic() -> BodiesState {
    let mut st = BodiesState::base(new_world(), SceneKind::Kinematic);
    add_ground_box(&mut st, 20.0);

    st.kin_amplitude = 2.0;
    let mut bd = default_body_def();
    bd.type_ = BodyType::Kinematic;
    bd.position = pos(2.0 * st.kin_amplitude, st.kin_amplitude + 1.0, 0.0);
    st.kin_body = create_body(&mut st.world, &bd);
    let box_hull = make_box_hull(0.1, 1.0, 0.2);
    create_hull_shape(
        &mut st.world,
        st.kin_body,
        &default_shape_def(),
        &box_hull.base,
    );
    st.vis
        .push(VisBody::box_body(st.kin_body.index1 - 1, 0.1, 1.0, 0.2));
    st.kin_time = 0.0;
    st
}

/// Lock Mixing (sample_bodies.cpp:915-995): five cubes with different motion locks.
fn lock_mixing() -> BodiesState {
    let mut st = BodiesState::base(new_world(), SceneKind::LockMixing);
    add_ground_box(&mut st, 20.0);

    let cube = make_box_hull(1.0, 1.0, 1.0);
    let sd = default_shape_def();

    let add = |st: &mut BodiesState, bd: box3d_rust::types::BodyDef| {
        let id = create_body(&mut st.world, &bd);
        create_hull_shape(&mut st.world, id, &sd, &cube.base);
        st.vis.push(VisBody::box_body(id.index1 - 1, 1.0, 1.0, 1.0));
    };

    // free
    let mut bd = default_body_def();
    bd.type_ = BodyType::Dynamic;
    bd.position = pos(0.0, 2.0, 0.0);
    add(&mut st, bd);

    // angular xz
    let mut bd = default_body_def();
    bd.type_ = BodyType::Dynamic;
    bd.position = pos(2.0, 2.0, 0.0);
    bd.motion_locks.angular_x = true;
    bd.motion_locks.angular_z = true;
    add(&mut st, bd);

    // linear xyz
    let mut bd = default_body_def();
    bd.type_ = BodyType::Dynamic;
    bd.position = pos(-2.0, 2.0, 0.0);
    bd.motion_locks.linear_x = true;
    bd.motion_locks.linear_y = true;
    bd.motion_locks.linear_z = true;
    add(&mut st, bd);

    // full
    let mut bd = default_body_def();
    bd.type_ = BodyType::Dynamic;
    bd.position = pos(0.0, 1.0, 2.0);
    bd.motion_locks.linear_x = true;
    bd.motion_locks.linear_y = true;
    bd.motion_locks.linear_z = true;
    bd.motion_locks.angular_x = true;
    bd.motion_locks.angular_y = true;
    bd.motion_locks.angular_z = true;
    add(&mut st, bd);

    // static
    let mut bd = default_body_def();
    bd.position = pos(0.0, 1.0, -3.0);
    add(&mut st, bd);
    st
}

/// Fixed Rotation (sample_bodies.cpp:998-1044): a static capsule and a dynamic
/// fully rotation-locked capsule (zero inverse inertia tensor).
fn fixed_rotation() -> BodiesState {
    let mut st = BodiesState::base(new_world(), SceneKind::FixedRotation);
    add_ground_box(&mut st, 20.0);

    let sd = default_shape_def();
    let mut capsule = Capsule {
        center1: v3(0.0, 0.0, 0.0),
        center2: v3(0.0, 1.0, 0.0),
        radius: 0.3,
    };

    let mut bd = default_body_def();
    bd.position = pos(0.0, 0.5, 0.0);
    let a = create_body(&mut st.world, &bd);
    create_capsule_shape(&mut st.world, a, &sd, &capsule);
    st.vis.push(VisBody::capsule_body(a.index1 - 1, &capsule));

    let mut bd = default_body_def();
    bd.position = pos(0.3, 0.5, 0.0);
    bd.type_ = BodyType::Dynamic;
    bd.gravity_scale = 0.0;
    bd.enable_sleep = false;
    bd.motion_locks.angular_x = true;
    bd.motion_locks.angular_y = true;
    bd.motion_locks.angular_z = true;
    capsule.radius = 0.2;
    let b = create_body(&mut st.world, &bd);
    create_capsule_shape(&mut st.world, b, &sd, &capsule);
    st.vis.push(VisBody::capsule_body(b.index1 - 1, &capsule));
    st
}

/// Gyroscopic Precession (sample_bodies.cpp:376-578): spinning tops (ported from
/// PEEL). Each top is tilted and spun about its symmetry axis; the gravity torque
/// about the tip makes it precess instead of toppling. The first top carries the
/// heavy-top precession diagnostic (see `precession.rs`). All tops share one hull
/// geometry and render through the dedicated precession pose channel; only the
/// ground box is a `VisBody`.
fn gyroscopic_precession() -> BodiesState {
    let mut st = BodiesState::base(new_world(), SceneKind::GyroscopicPrecession);
    add_ground_box(&mut st, 40.0);

    // Top shape: a wide n-gon rim up top and a point at the origin, so it balances on its tip.
    const NUM_SEGS: usize = 7;
    const R: f32 = 2.0;
    const H: f32 = 2.0;
    let mut hull_points = [VEC3_ZERO; NUM_SEGS + 1];
    let dphi = 2.0 * PI / NUM_SEGS as f32;
    for (i, point) in hull_points.iter_mut().take(NUM_SEGS).enumerate() {
        *point = v3(R * (i as f32 * dphi).cos(), H, R * (i as f32 * dphi).sin());
    }
    hull_points[NUM_SEGS] = VEC3_ZERO;
    let hull = create_hull(&hull_points, (NUM_SEGS + 1) as i32).expect("precession top hull");

    // Shared, hull-local render geometry for every top.
    st.prec.hull_tris = hull_triangles(&hull);
    st.prec.hull_edges = hull_edges(&hull);

    let sd = default_shape_def();

    // Tilt the top, then spin it about its own symmetry axis. Gravity does the rest.
    let rotation = make_quat_from_axis_angle(VEC3_AXIS_Z, 15.0 * PI / 180.0);
    let angular_velocity = rotate_vector(rotation, v3(0.0, 75.0, 0.0));

    const COUNT: i32 = 8;
    const SEPARATION: f32 = 6.0;
    for x in 0..COUNT {
        for z in 0..COUNT {
            let mut bd = default_body_def();
            bd.type_ = BodyType::Dynamic;
            bd.position = pos(
                (x - COUNT / 2) as f32 * SEPARATION,
                H,
                (z - COUNT / 2) as f32 * SEPARATION,
            );
            bd.rotation = rotation;

            // The spin rate exceeds the default cap, so bypass it as the test intends.
            bd.allow_fast_rotation = true;

            let body_id = create_body(&mut st.world, &bd);
            create_hull_shape(&mut st.world, body_id, &sd, &hull);
            body_set_angular_velocity(&mut st.world, body_id, angular_velocity);
            st.prec.top_indices.push(body_id.index1 - 1);

            if x == 0 && z == 0 {
                st.prec.top_id = body_id;
            }
        }
    }

    // Mass properties of the measured top. The tip sits at the body origin, so the pivot
    // distance is just the height of the center of mass, and the symmetry axis is the local
    // up axis.
    let mass_data = body_get_mass_data(&st.world, st.prec.top_id);
    st.prec.mass = mass_data.mass;
    st.prec.pivot_distance = mass_data.center.y;
    st.prec.spin_inertia = mass_data.inertia.cy.y;

    // Transverse inertia belongs about the pivot, not the center of mass.
    let transverse = 0.5 * (mass_data.inertia.cx.x + mass_data.inertia.cz.z);
    st.prec.transverse_inertia =
        transverse + st.prec.mass * st.prec.pivot_distance * st.prec.pivot_distance;

    st.prec.gravity = length(world_get_gravity(&st.world));
    st
}

/// Class Ring (sample_bodies.cpp:1184-1278): a spinning class ring flips its heavy
/// gem from bottom to top (https://www.youtube.com/watch?v=_up0BiLCliA). A band of
/// 24 capsules plus an off-center gem sphere on one dynamic body, tilted 13° and
/// spun at 100 rad/s about its own up axis. "This is a fiddley test and requires
/// careful tuning" — it also needs the 960 Hz stepping done in `bodies_step`.
fn class_ring() -> BodiesState {
    let mut st = BodiesState::base(new_world(), SceneKind::ClassRing);
    add_ground_box(&mut st, 100.0);

    const N: usize = 24;
    const R: f32 = 1.0;
    const TUBE_RADIUS: f32 = 0.1 * R;
    const AXIS_RADIUS: f32 = R - TUBE_RADIUS;

    let mut bd = default_body_def();
    bd.type_ = BodyType::Dynamic;
    bd.position = pos(0.0, R, 0.0);
    bd.rotation = make_quat_from_axis_angle(VEC3_AXIS_X, 13.0 * PI / 180.0);
    bd.allow_fast_rotation = true;
    bd.enable_contact_recycling = false;
    let body_id = create_body(&mut st.world, &bd);
    let idx = body_id.index1 - 1;

    let mut sd = default_shape_def();
    sd.density = 1.0;

    // Band built from a loop of capsules. The ring vertices come from repeatedly
    // rotating (x, y) by the segment angle, using the deterministic cos/sin.
    let mut vertices = [VEC3_ZERO; N];
    let delta_angle = 2.0 * PI / N as f32;
    let cs = compute_cos_sin(delta_angle);
    let (mut x, mut y) = (AXIS_RADIUS, 0.0f32);
    for vertex in vertices.iter_mut() {
        *vertex = v3(x, y, 0.0);
        let x2 = cs.cosine * x - cs.sine * y;
        let y2 = cs.sine * x + cs.cosine * y;
        x = x2;
        y = y2;
    }

    for i in 0..N {
        let capsule = Capsule {
            center1: vertices[i],
            center2: vertices[(i + 1) % N],
            radius: TUBE_RADIUS,
        };
        create_capsule_shape(&mut st.world, body_id, &sd, &capsule);
        st.vis.push(VisBody::capsule_body(idx, &capsule));
    }

    // Heavy gem provides the mass asymmetry that drives the inversion
    sd.density = 2.0;
    let sphere = Sphere {
        center: v3(0.0, -0.65 * R, 0.0),
        radius: 0.3,
    };
    create_sphere_shape(&mut st.world, body_id, &sd, &sphere);
    st.vis.push(VisBody::sphere_local(
        idx,
        sphere.radius,
        Transform {
            p: sphere.center,
            q: QUAT_IDENTITY,
        },
    ));

    let angular_velocity = rotate_vector(bd.rotation, v3(0.0, 100.0, 0.0));
    body_set_angular_velocity(&mut st.world, body_id, angular_velocity);

    // (C keeps `m_ringId` but never reads it, so no handle is stored here.)
    st
}

#[cfg(test)]
mod tests {
    use super::super::{bodies_poses, bodies_reset, bodies_step};
    use crate::vis::{KIND_CAPSULE, KIND_SPHERE, POSE_STRIDE};

    /// Class Ring builds the ground + 24 band capsules + the gem sphere on one
    /// body, and its 960 Hz step override keeps the ring on the ground box.
    #[test]
    fn class_ring_builds_and_steps() {
        let count = bodies_reset(10);
        assert_eq!(count, 1 + 24 + 1);

        let poses = bodies_poses();
        assert_eq!(poses.len(), 26 * POSE_STRIDE);
        // Entries 1..25 are the band capsules, entry 25 the gem sphere.
        let kind = |i: usize| poses[i * POSE_STRIDE + 14] as u8;
        for i in 1..25 {
            assert_eq!(kind(i), KIND_CAPSULE, "band entry {i}");
        }
        assert_eq!(kind(25), KIND_SPHERE);

        // Ten rendered frames = 160 world steps at 1/960 s.
        for _ in 0..10 {
            bodies_step(1.0 / 60.0, 4);
        }
        let poses = bodies_poses();
        let gem_y = poses[25 * POSE_STRIDE + 1];
        assert!(gem_y.is_finite());
        // The ring rolls on the ground box (top face at y = 0); the gem hangs below
        // the ring center but must stay above the ground.
        assert!(gem_y > 0.0 && gem_y < 2.0, "gem y = {gem_y}");
    }
}
