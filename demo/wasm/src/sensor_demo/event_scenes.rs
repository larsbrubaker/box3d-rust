//! The four Events samples from sample_events.cpp ported 1:1:
//! Hit (:83), Move (:247), Joint (:331), Persistent Contact (:573).
//!
//! Each mirrors the C constructor's exact values, the Step event-processing, and
//! the Render/DrawString output. Event visualization goes through the shared
//! overlay (segments/points) and `sensor_debug_text` channels; per-frame HUD
//! strings go through `sensor_hud`.

use super::{
    add_ground_box, hud_json, labels_json, new_world, Label, OverlayBuf, SceneKind, SensorState,
    VisSet, MAX_HIT_EVENTS,
};
use crate::vis::{mesh_triangle_edges, pos, sphere, vec3, VisBody};
use box3d_rust::body::{
    body_apply_mass_from_shapes, body_get_angular_velocity, body_get_linear_velocity,
    body_get_local_point, body_get_local_point_velocity, body_get_name, body_get_world_center,
    body_set_angular_velocity, body_set_linear_velocity, create_body, make_body_id,
};
use box3d_rust::contact::contact_is_valid;
use box3d_rust::debug_draw::HexColor;
use box3d_rust::geometry::{default_surface_material, Capsule};
use box3d_rust::hull::{make_box_hull, make_transformed_box_hull};
use box3d_rust::id::{JointId, NULL_BODY_ID, NULL_CONTACT_ID, NULL_JOINT_ID};
use box3d_rust::joint::{
    create_distance_joint, create_prismatic_joint, create_revolute_joint, create_weld_joint,
    destroy_joint, joint_is_valid,
};
use box3d_rust::math_functions::{
    add, cross, length_squared, mul_sv, offset_pos, sub_pos, Pos, Transform, Vec3, POS_ZERO,
    QUAT_IDENTITY, VEC3_ONE,
};
use box3d_rust::mesh::create_grid_mesh;
use box3d_rust::shape::{
    create_capsule_shape, create_hull_shape, create_mesh_shape, create_sphere_shape,
};
use box3d_rust::types::{
    default_body_def, default_distance_joint_def, default_prismatic_joint_def,
    default_revolute_joint_def, default_shape_def, default_weld_joint_def, BodyType,
};
use box3d_rust::world::{world_get_body_events, world_get_contact_events, world_get_joint_events};

fn pos_to_v3(p: Pos) -> Vec3 {
    Vec3 {
        x: p.x as f32,
        y: p.y as f32,
        z: p.z as f32,
    }
}

/// Create one dynamic box (half extents 1×1×0.5) at `position`, register its
/// VisBody, and return the body id — shared by the four Joint-scene slots.
fn spawn_joint_box(
    world: &mut box3d_rust::world::World,
    vis: &mut VisSet,
    body_def: &mut box3d_rust::types::BodyDef,
    shape_def: &box3d_rust::types::ShapeDef,
    box_hull: &box3d_rust::hull::BoxHull,
    position: Vec3,
) -> box3d_rust::id::BodyId {
    body_def.position = offset_pos(POS_ZERO, position);
    let body_id = create_body(world, body_def);
    create_hull_shape(world, body_id, shape_def, &box_hull.base);
    vis.push(
        VisBody::box_body(body_id.index1 - 1, 1.0, 1.0, 0.5),
        0,
        false,
    );
    body_id
}

// --- Hit (sample_events.cpp:83) ----------------------------------------------

/// Grid-mesh ground with 6 per-triangle materials, plus a welded chain of
/// spinning capsules whose `enableHitEvents` shapes fire contact-hit events on
/// impact. (HitEvent ctor, :96-198)
// `prev_body_id`'s null init mirrors C's `b3BodyId prevBodyId = {}`; it is always
// overwritten before the `is_non_null` check, so the init read is dead.
#[allow(unused_assignments)]
pub(super) fn reset_hit() -> SensorState {
    let mut world = new_world();
    let mut vis = VisSet::default();

    // Ground: 20×20 grid mesh, cell 8, 6 materials (userMaterialId = i + 1).
    let material_count = 6i32;
    let grid_mesh = create_grid_mesh(20, 20, 8.0, material_count, true).expect("hit grid mesh");
    {
        let ground = create_body(&mut world, &default_body_def());
        let mut shape_def = default_shape_def();
        let mut materials = Vec::with_capacity(material_count as usize);
        for i in 0..material_count {
            let mut m = default_surface_material();
            m.user_material_id = (i + 1) as u64;
            materials.push(m);
        }
        shape_def.materials = materials;
        create_mesh_shape(&mut world, ground, &shape_def, &grid_mesh, VEC3_ONE);
    }
    let ground_wire = mesh_triangle_edges(&grid_mesh, VEC3_ONE);

    let mut joint_def = default_weld_joint_def();
    joint_def.angular_hertz = 10.0;
    joint_def.angular_damping_ratio = 2.0;

    let mut r = 0.75f32;
    let mut y = r;
    let l = 1.5f32;
    let mut offset = 0.05f32;

    let mut shape_def = default_shape_def();
    shape_def.enable_hit_events = true;
    shape_def.base_material.rolling_resistance = 0.2;
    shape_def.base_material.user_material_id = 42;
    shape_def.update_body_mass = false;

    let origin = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = offset_pos(POS_ZERO, origin);

    let mut prev_body_id = NULL_BODY_ID;
    let mut body_id = create_body(&mut world, &body_def);
    let shape_count = 22i32;
    let mut velocity_scale = 0.5f32;
    let shapes_per_body = 3i32;

    for i in 0..shape_count {
        let capsule = Capsule {
            center1: Vec3 {
                x: offset,
                y,
                z: 0.0,
            },
            center2: Vec3 {
                x: 0.0,
                y: y + l,
                z: -offset,
            },
            radius: r,
        };
        create_capsule_shape(&mut world, body_id, &shape_def, &capsule);
        vis.push(
            VisBody::capsule_body(body_id.index1 - 1, &capsule),
            0,
            false,
        );

        if (i + 1) % shapes_per_body == 0 || i == shape_count - 1 {
            body_apply_mass_from_shapes(&mut world, body_id);

            let center = body_get_world_center(&world, body_id);
            let omega = Vec3 {
                x: 0.0,
                y: 0.0,
                z: -velocity_scale,
            };
            let v = cross(omega, sub_pos(center, offset_pos(POS_ZERO, origin)));
            body_set_angular_velocity(&mut world, body_id, omega);
            body_set_linear_velocity(&mut world, body_id, v);

            if i < shape_count - 1 {
                prev_body_id = body_id;

                if i < shape_count - 1 {
                    body_id = create_body(&mut world, &body_def);

                    if prev_body_id.is_non_null() {
                        joint_def.base.body_id_a = prev_body_id;
                        joint_def.base.body_id_b = body_id;
                        joint_def.base.local_frame_a.p = Vec3 {
                            x: 0.0,
                            y: y + l + r,
                            z: 0.0,
                        };
                        joint_def.base.local_frame_b.p = Vec3 {
                            x: 0.0,
                            y: y + l + r,
                            z: 0.0,
                        };
                        create_weld_joint(&mut world, &joint_def);
                    }

                    velocity_scale *= 0.75;
                }
            }
        }

        y += l + 2.0 * r;
        r *= 0.95;
        offset = -offset;
    }

    let mut state = SensorState::blank(world, vis, SceneKind::Hit);
    state.grid_mesh = Some(grid_mesh);
    state.ground_wire = ground_wire;
    state
}

/// HitEvent::Step (:227) accumulates hit events (capped at 32); HitEvent::Render
/// (:205) draws each as a yellow point, a yellow approach-speed ray, and a white
/// `"%.1f, %d"` (speed, userMaterialIdA) label. (:211-224)
pub(super) fn process_hit(state: &mut SensorState) {
    // Accumulate from the world's hit-event slice in place: the loop pushes to the
    // disjoint `state.hit_events` field, not `state.world`, so no clone is needed.
    let hits = world_get_contact_events(&state.world).hit_events;
    for &h in hits {
        if state.hit_events.len() >= MAX_HIT_EVENTS {
            break;
        }
        state.hit_events.push(h);
    }

    let mut ov = OverlayBuf::default();
    let mut labels: Vec<Label> = Vec::new();
    for e in &state.hit_events {
        let p1 = pos_to_v3(e.point);
        // p2 = p1 - approachSpeed * normal
        let p2 = add(p1, mul_sv(-e.approach_speed, e.normal));
        ov.point(p1, 10.0, HexColor::YELLOW.0);
        ov.seg(p1, p2, HexColor::YELLOW.0);
        labels.push(Label {
            p: p1,
            color: HexColor::WHITE.0,
            text: format!("{:.1}, {}", e.approach_speed, e.user_material_id_a),
        });
    }
    state.overlay = ov.pack();
    state.labels = labels_json(&labels);
    state.hud = hud_json(&[(
        "event count".to_string(),
        state.hit_events.len().to_string(),
    )]);
}

// --- Move (sample_events.cpp:247) --------------------------------------------

/// A tall dynamic box spun about a pivot; body move/sleep events are read via
/// b3World_GetBodyEvents. (MoveEvent ctor, :250-286)
pub(super) fn reset_move() -> SensorState {
    let mut world = new_world();
    let mut vis = VisSet::default();

    let ground = add_ground_box(&mut world, 40.0);
    vis.push(
        VisBody::box_body(ground.index1 - 1, 40.0, 1.0, 40.0),
        0,
        false,
    );

    let mut body_def = default_body_def();
    let pivot = pos(0.0, 1.0, 0.0);
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pivot;
    body_def.name = "big box".to_string();
    let body_id = create_body(&mut world, &body_def);

    let move_local_pivot = body_get_local_point(&world, body_id, pivot);

    let mut shape_def = default_shape_def();
    shape_def.enable_hit_events = true;
    let box_xf = Transform {
        p: Vec3 {
            x: 0.0,
            y: 10.0,
            z: 0.0,
        },
        q: QUAT_IDENTITY,
    };
    let dynamic_box = make_transformed_box_hull(0.5, 10.0, 0.5, box_xf);
    create_hull_shape(&mut world, body_id, &shape_def, &dynamic_box.base);
    vis.push(
        VisBody::box_local(body_id.index1 - 1, 0.5, 10.0, 0.5, box_xf),
        0,
        false,
    );

    let center = body_get_world_center(&world, body_id);
    let r = sub_pos(pivot, center);
    let rr = length_squared(r);
    if rr > 0.0 {
        let v = Vec3 {
            x: -10.0,
            y: 0.0,
            z: 0.0,
        };
        let omega = mul_sv(1.0 / rr, cross(v, r));
        body_set_angular_velocity(&mut world, body_id, omega);
        body_set_linear_velocity(&mut world, body_id, v);
    }

    let mut state = SensorState::blank(world, vis, SceneKind::Move);
    state.event_body = body_id;
    state.move_local_pivot = move_local_pivot;
    state
}

/// MoveEvent::Render (:288) shows the pivot/linear/angular velocities;
/// MoveEvent::Step (:300) reports each moving body as moved / fell asleep.
pub(super) fn process_move(state: &mut SensorState) {
    let vp = body_get_local_point_velocity(&state.world, state.event_body, state.move_local_pivot);
    let v = body_get_linear_velocity(&state.world, state.event_body);
    let omega = body_get_angular_velocity(&state.world, state.event_body);

    let mut rows: Vec<(String, String)> = vec![
        (
            "vp".to_string(),
            format!("[{:.2}, {:.2}, {:.2}]", vp.x, vp.y, vp.z),
        ),
        (
            "v".to_string(),
            format!("[{:.2}, {:.2}, {:.2}]", v.x, v.y, v.z),
        ),
        (
            "w".to_string(),
            format!("[{:.2}, {:.2}, {:.2}]", omega.x, omega.y, omega.z),
        ),
    ];

    // Iterate the world's move events in place: the loop only reads `state.world`
    // (via `body_get_name`) and pushes to the local `rows`, so no clone is needed.
    for e in world_get_body_events(&state.world) {
        let name = body_get_name(&state.world, e.body_id).to_string();
        let msg = if e.fell_asleep {
            format!("{} fell asleep", name)
        } else {
            format!("{} moved", name)
        };
        rows.push(("event".to_string(), msg));
    }
    state.hud = hud_json(&rows);
}

// --- Joint (sample_events.cpp:331) -------------------------------------------

/// Four boxes held to the ground body by distance / prismatic / revolute / weld
/// joints, each with force/torque thresholds. When a joint's reaction exceeds a
/// threshold, b3World_GetJointEvents reports it and JointEvent::Step destroys it.
/// (JointEvent ctor, :339-540; the motor + wheel slots are `#if 0` in C.)
pub(super) fn reset_joint() -> SensorState {
    let mut world = new_world();
    let mut vis = VisSet::default();

    let ground_box = add_ground_box(&mut world, 20.0);
    vis.push(
        VisBody::box_body(ground_box.index1 - 1, 20.0, 1.0, 20.0),
        0,
        false,
    );

    // Separate empty ground body at the origin — the joint anchor A.
    let ground_id = create_body(&mut world, &default_body_def());

    const E_COUNT: usize = 6;
    let mut joint_ids: Vec<JointId> = vec![NULL_JOINT_ID; E_COUNT];

    let mut position = Vec3 {
        x: -12.5,
        y: 10.0,
        z: 0.0,
    };
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.enable_sleep = false;

    let box_hull = make_box_hull(1.0, 1.0, 0.5);

    let mut index = 0usize;
    let force_threshold = 3000.0f32;
    let torque_threshold = 10000.0f32;

    let mut shape_def = default_shape_def();
    shape_def.density = 1.0;

    // distance joint
    {
        let body_id = spawn_joint_box(
            &mut world,
            &mut vis,
            &mut body_def,
            &shape_def,
            &box_hull,
            position,
        );
        let length = 2.0f32;
        let pivot1 = pos(position.x, position.y + 1.0 + length, 0.0);
        let pivot2 = pos(position.x, position.y + 1.0, 0.0);
        let mut jd = default_distance_joint_def();
        jd.base.body_id_a = ground_id;
        jd.base.body_id_b = body_id;
        jd.base.local_frame_a.p = body_get_local_point(&world, ground_id, pivot1);
        jd.base.local_frame_b.p = body_get_local_point(&world, body_id, pivot2);
        jd.length = length;
        jd.base.force_threshold = force_threshold;
        jd.base.torque_threshold = torque_threshold;
        jd.base.collide_connected = true;
        jd.base.user_data = index as u64;
        joint_ids[index] = create_distance_joint(&mut world, &jd);
    }
    position.x += 5.0;
    index += 1;

    // motor joint — `#if 0` in C, slot stays null.
    position.x += 5.0;
    index += 1;

    // prismatic joint
    {
        let body_id = spawn_joint_box(
            &mut world,
            &mut vis,
            &mut body_def,
            &shape_def,
            &box_hull,
            position,
        );
        let pivot = pos(position.x - 1.0, position.y, 0.0);
        let mut jd = default_prismatic_joint_def();
        jd.base.body_id_a = ground_id;
        jd.base.body_id_b = body_id;
        jd.base.local_frame_a.p = body_get_local_point(&world, ground_id, pivot);
        jd.base.local_frame_b.p = body_get_local_point(&world, body_id, pivot);
        jd.base.force_threshold = force_threshold;
        jd.base.torque_threshold = torque_threshold;
        jd.base.collide_connected = true;
        jd.base.user_data = index as u64;
        joint_ids[index] = create_prismatic_joint(&mut world, &jd);
    }
    position.x += 5.0;
    index += 1;

    // revolute joint
    {
        let body_id = spawn_joint_box(
            &mut world,
            &mut vis,
            &mut body_def,
            &shape_def,
            &box_hull,
            position,
        );
        let pivot = pos(position.x - 1.0, position.y, 0.0);
        let mut jd = default_revolute_joint_def();
        jd.base.body_id_a = ground_id;
        jd.base.body_id_b = body_id;
        jd.base.local_frame_a.p = body_get_local_point(&world, ground_id, pivot);
        jd.base.local_frame_b.p = body_get_local_point(&world, body_id, pivot);
        jd.base.force_threshold = force_threshold;
        jd.base.torque_threshold = torque_threshold;
        jd.base.collide_connected = true;
        jd.base.user_data = index as u64;
        joint_ids[index] = create_revolute_joint(&mut world, &jd);
    }
    position.x += 5.0;
    index += 1;

    // weld joint
    {
        let body_id = spawn_joint_box(
            &mut world,
            &mut vis,
            &mut body_def,
            &shape_def,
            &box_hull,
            position,
        );
        let pivot = pos(position.x - 1.0, position.y, 0.0);
        let mut jd = default_weld_joint_def();
        jd.base.body_id_a = ground_id;
        jd.base.body_id_b = body_id;
        jd.base.local_frame_a.p = body_get_local_point(&world, ground_id, pivot);
        jd.base.local_frame_b.p = body_get_local_point(&world, body_id, pivot);
        jd.angular_hertz = 2.0;
        jd.angular_damping_ratio = 0.5;
        jd.base.force_threshold = force_threshold;
        jd.base.torque_threshold = torque_threshold;
        jd.base.collide_connected = true;
        jd.base.user_data = index as u64;
        joint_ids[index] = create_weld_joint(&mut world, &jd);
    }
    // (wheel joint — `#if 0` in C — and the trailing position/index advances are
    // dead once the loop is unrolled, so they are omitted.)

    let mut state = SensorState::blank(world, vis, SceneKind::Joint);
    state.joint_ids = joint_ids;
    state.hud = hud_json(&[("joints remaining".to_string(), E_COUNT_ACTIVE.to_string())]);
    state
}

/// Number of live joints created (distance/prismatic/revolute/weld); the motor
/// and wheel slots are `#if 0` in C.
const E_COUNT_ACTIVE: usize = 4;

/// JointEvent::Step (:542) destroys every joint the world reports over threshold.
pub(super) fn process_joint(state: &mut SensorState) {
    // The clone is borrow-required here: the loop calls `destroy_joint(&mut
    // state.world)`, which cannot run while a `&state.world` event slice is still
    // borrowed, so the events are snapshotted before iterating.
    let events = world_get_joint_events(&state.world).to_vec();
    for e in &events {
        if joint_is_valid(&state.world, e.joint_id) {
            let index = e.user_data as usize;
            debug_assert!(index < state.joint_ids.len());
            destroy_joint(&mut state.world, e.joint_id, true);
            if index < state.joint_ids.len() {
                state.joint_ids[index] = NULL_JOINT_ID;
            }
        }
    }

    let remaining = state.joint_ids.iter().filter(|j| j.is_non_null()).count();
    state.hud = hud_json(&[("joints remaining".to_string(), remaining.to_string())]);
}

// --- Persistent Contact (sample_events.cpp:573) ------------------------------

/// A sphere rolls across a grid mesh; one contact id is tracked across steps and
/// its manifold points/impulses are drawn. (PersistentContact ctor, :576-609)
pub(super) fn reset_persistent() -> SensorState {
    let mut world = new_world();
    let mut vis = VisSet::default();

    let mesh_data = create_grid_mesh(20, 20, 2.0, 2, true).expect("persistent grid mesh");
    {
        let ground = create_body(&mut world, &default_body_def());
        create_mesh_shape(
            &mut world,
            ground,
            &default_shape_def(),
            &mesh_data,
            VEC3_ONE,
        );
    }
    let ground_wire = mesh_triangle_edges(&mesh_data, VEC3_ONE);

    {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = pos(-18.0, 1.0, 0.5);
        body_def.linear_velocity = vec3(4.0, 0.0, 0.0);
        let body = create_body(&mut world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.density = 20.0;
        shape_def.enable_contact_events = true;
        shape_def.base_material.rolling_resistance = 0.01;
        let sph = sphere(0.5);
        create_sphere_shape(&mut world, body, &shape_def, &sph);
        vis.push(VisBody::sphere_body(body.index1 - 1, 0.5), 0, false);
    }

    let mut state = SensorState::blank(world, vis, SceneKind::PersistentContact);
    state.grid_mesh = Some(mesh_data);
    state.ground_wire = ground_wire;
    state
}

/// PersistentContact::Step (:616) latches the first begin-touch contact id, drops
/// it on end-touch, and while valid draws each manifold point + total normal
/// impulse ray (crimson) with a gray `"%.2f"` impulse label.
pub(super) fn process_persistent(state: &mut SensorState) {
    {
        let events = world_get_contact_events(&state.world);
        if let Some(b) = events.begin_events.first() {
            state.contact_id = b.contact_id;
        }
        // Scan the end-event slice in place: the loop mutates only the disjoint
        // `state.contact_id`, not `state.world`, so no clone is needed.
        for e in events.end_events {
            if state.contact_id == e.contact_id {
                state.contact_id = NULL_CONTACT_ID;
                break;
            }
        }
    }

    let mut ov = OverlayBuf::default();
    let mut labels: Vec<Label> = Vec::new();

    if state.contact_id.is_non_null() && contact_is_valid(&state.world, state.contact_id) {
        let cidx = (state.contact_id.index1 - 1) as usize;
        let shape_a = state.world.contacts[cidx].shape_id_a;
        let body_a_index = state.world.shapes[shape_a as usize].body_id;
        let body_a = make_body_id(&state.world, body_a_index);
        let com = pos_to_v3(body_get_world_center(&state.world, body_a));

        // Borrow the manifolds directly: `com` is already computed and the loop
        // only reads `m` while pushing to local overlay/label buffers (no
        // `state.world` mutation), so no clone is needed.
        for m in &state.world.contacts[cidx].manifolds {
            let normal = m.normal;
            for j in 0..(m.point_count as usize) {
                let mp = &m.points[j];
                let p1 = add(com, mp.anchor_a);
                let p2 = add(p1, mul_sv(mp.total_normal_impulse, normal));
                ov.seg(p1, p2, HexColor::CRIMSON.0);
                ov.point(p1, 6.0, HexColor::CRIMSON.0);
                labels.push(Label {
                    p: p1,
                    color: HexColor::GRAY.0,
                    text: format!("{:.2}", mp.total_normal_impulse),
                });
            }
        }
    } else {
        state.contact_id = NULL_CONTACT_ID;
    }

    state.overlay = ov.pack();
    state.labels = labels_json(&labels);
    state.hud = hud_json(&[(
        "tracking contact".to_string(),
        if state.contact_id.is_non_null() {
            "yes".to_string()
        } else {
            "no".to_string()
        },
    )]);
}
