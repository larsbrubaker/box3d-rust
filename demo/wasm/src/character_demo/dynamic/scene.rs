//! The `DynamicMoverSample` constructor (`sample_character.cpp:851`, upstream 3362633)
//! and its scene helpers (`AddRamp`, `GetPlatformPosition`, `GetPillarTop`,
//! `SpawnGems`). Every body / shape / joint is created in the C order so ids, the
//! broad-phase, and contact ordering match the C sample.
//!
//! The scene reproduces the C **debug** build (`Sample::m_isDebug == true`): the large
//! pyramid uses `baseCount = 20` and the 3072 sleeping skittles over the pressure plate
//! are not created (`if constexpr ( m_isDebug == false )`), exactly as the C debug
//! sample does. The release counts (pyramid 100, skittles 32×32×3) are too heavy for the
//! serial wasm port; this is the same disclosed DEBUG-count convention the Benchmark and
//! Village samples follow.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

use super::camera::SampleCamera;
use super::scene_parts::{build_ball_pit, build_door, build_pressure_plate, build_sweeper};
use super::{
    DynamicMoverSample, ALL_BITS, DEBRIS_BIT, DYNAMIC_BIT, GEM_BIT, GEM_COUNT, MOVER_BIT,
    PILLAR_COUNT, PLATFORM_COUNT, PLATFORM_MATERIAL,
};
use crate::character_demo::hulls::HullVis;
use crate::vis::{hull_triangles, VisBody};
use box3d_rust::body::{body_get_local_point, create_body};
use box3d_rust::debug_draw::HexColor;
use box3d_rust::dynamic_mover::{DynamicMover, DynamicMoverDef};
use box3d_rust::geometry::Sphere;
use box3d_rust::hull::{
    create_cylinder, make_box_hull, make_cube_hull, make_transformed_box_hull, HullData,
};
use box3d_rust::id::{BodyId, NULL_BODY_ID};
use box3d_rust::joint::create_revolute_joint;
use box3d_rust::math_functions::{
    compute_quat_between_unit_vectors, length, lerp, make_quat_from_axis_angle, mul_add, mul_sv,
    normalize, offset_pos, rotate_vector, sub, Pos, Quat, Transform, Vec3, QUAT_IDENTITY,
    VEC3_AXIS_Y, VEC3_AXIS_Z, VEC3_ZERO,
};
use box3d_rust::shape::{create_hull_shape, create_sphere_shape};
use box3d_rust::types::{
    default_body_def, default_revolute_joint_def, default_shape_def, BodyType, Filter,
};
use box3d_rust::world::World;

/// C `DynamicMoverSample` start position.
pub(crate) const START_POSITION: Pos = Pos {
    x: 0.0,
    y: 1.5,
    z: 0.0,
};

/// C `Sample::AddGroundBox( extent )` (sample.cpp:612): an `extent` × 1 × `extent` box at
/// `y = -1`, drawn with the procedural grid (the page's ground plane here).
fn add_ground_box(world: &mut World, extent: f32) -> BodyId {
    let mut body_def = default_body_def();
    body_def.name = String::from("ground");
    body_def.position = Pos {
        x: 0.0,
        y: -1.0,
        z: 0.0,
    };
    let ground_id = create_body(world, &body_def);

    let shape_def = default_shape_def();
    let hull = make_box_hull(extent, 1.0, extent);
    create_hull_shape(world, ground_id, &shape_def, &hull.base);

    ground_id
}

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

fn index_of(id: BodyId) -> i32 {
    id.index1 - 1
}

/// C `DynamicMoverSample::GetPlatformPosition` (:1385).
pub(crate) fn get_platform_position(index: usize, time: f32) -> Pos {
    if index == 0 {
        // Vertical
        let y = 1.95 - 1.95 * (0.6 * time).cos();
        return pos(14.0, y, 0.0);
    }

    // Horizontal
    let x = 25.0 - 3.5 * (0.5 * time).cos();
    pos(x, 3.9, 0.0)
}

/// C `DynamicMoverSample::GetPillarTop` (:1399).
pub(crate) fn get_pillar_top(index: usize) -> Pos {
    let i = index as i32;
    pos(-20.0, 0.5 + 0.6 * i as f32, -3.0 - 5.0 * i as f32)
}

/// C `DynamicMoverSample::AddRamp` (:1363) — a 0.1 m thick plank whose top face spans
/// `low` → `high`.
fn add_ramp(
    world: &mut World,
    vis: &mut Vec<VisBody>,
    high: Vec3,
    low: Vec3,
    half_width: f32,
    color: HexColor,
) {
    let mut axis = sub(high, low);
    let length_ = length(axis);
    axis = mul_sv(1.0 / length_, axis);

    let rotation = compute_quat_between_unit_vectors(VEC3_AXIS_Z, axis);
    let normal = rotate_vector(rotation, VEC3_AXIS_Y);

    let half_thickness = 0.1f32;
    let center = mul_add(lerp(low, high, 0.5), -half_thickness, normal);

    let body_def = default_body_def();
    let ramp_id = create_body(world, &body_def);

    let mut shape_def = default_shape_def();
    shape_def.base_material.custom_color = color.0;
    let local = Transform {
        p: center,
        q: rotation,
    };
    let box_hull = make_transformed_box_hull(half_width, half_thickness, 0.5 * length_, local);
    create_hull_shape(world, ramp_id, &shape_def, &box_hull.base);
    vis.push(VisBody::box_local(
        index_of(ramp_id),
        half_width,
        half_thickness,
        0.5 * length_,
        local,
    ));
}

/// C `DynamicMoverSample::SpawnGems` (:1404): 17 spinning kinematic sensor gems.
fn spawn_gems(world: &mut World, gem_ids: &mut [BodyId; GEM_COUNT]) {
    let mut positions: [Pos; GEM_COUNT] = [
        pos(0.0, 1.2, 5.0),
        pos(-6.0, 2.2, 6.0),
        pos(-6.0, 3.2, 11.0),
        pos(0.0, 3.2, 11.0),
        pos(6.0, 3.2, 11.0),
        pos(6.0, 2.2, 6.0),
        pos(18.0, 5.2, 0.0),
        pos(25.0, 5.2, 0.0),
        pos(32.0, 5.2, 0.0),
        pos(-17.0, 2.2, 0.0),
        pos(-10.0, 4.4, -8.0),
        pos(6.0, 1.2, -9.5),
        pos(3.0, 1.2, -16.0),
        pos(-3.0, 1.2, -16.0),
        pos(0.0, 1.2, -19.0),
        pos(0.0, 1.2, -13.0),
        pos(-20.0, 0.0, 0.0),
    ];

    positions[GEM_COUNT - 1] = get_pillar_top(PILLAR_COUNT - 1);
    // C `positions[...].y += 1.2f`; the x / z adds are +0, which leaves them unchanged.
    positions[GEM_COUNT - 1] = offset_pos(positions[GEM_COUNT - 1], v3(0.0, 1.2, 0.0));

    let mut shape_def = default_shape_def();
    shape_def.is_sensor = true;
    shape_def.enable_sensor_events = true;
    shape_def.filter = Filter {
        category_bits: GEM_BIT,
        mask_bits: MOVER_BIT,
        group_index: 0,
    };
    shape_def.base_material.custom_color = HexColor::GOLD.0;

    let tilt = gem_tilt();
    let gem = make_transformed_box_hull(
        0.2,
        0.2,
        0.2,
        Transform {
            p: VEC3_ZERO,
            q: tilt,
        },
    );

    for i in 0..GEM_COUNT {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Kinematic;
        body_def.position = positions[i];
        body_def.angular_velocity = v3(0.0, 2.0, 0.0);
        gem_ids[i] = create_body(world, &body_def);
        create_hull_shape(world, gem_ids[i], &shape_def, &gem.base);
    }
}

/// The gem box tilt, `b3MakeQuatFromAxisAngle( b3Normalize( { 1, 0, -1 } ), 0.9553f )`
/// (a cube standing on a corner).
pub(crate) fn gem_tilt() -> Quat {
    make_quat_from_axis_angle(normalize(v3(1.0, 0.0, -1.0)), 0.9553)
}

/// Build the whole sample (C `DynamicMoverSample::DynamicMoverSample`).
pub(crate) fn build_dynamic_mover() -> (World, DynamicMoverSample) {
    let mut world = crate::character_demo::new_world();
    let mut vis: Vec<VisBody> = Vec::new();
    let mut hulls: Vec<HullVis> = Vec::new();

    let start_position = START_POSITION;

    // C: m_camera->SetView( 180.0f, 20.0f, 6.0f, startPosition ) on a fresh load.
    let camera = SampleCamera::new_view(180.0, 20.0, 6.0, start_position);

    let ground_id = add_ground_box(&mut world, 80.0);

    let mut mover = DynamicMover::new();
    {
        let def = DynamicMoverDef {
            position: start_position,
            filter: Filter {
                category_bits: MOVER_BIT,
                mask_bits: ALL_BITS & !DEBRIS_BIT,
                group_index: 0,
            },
            platform_material_id: PLATFORM_MATERIAL,
            enable_sensor_events: true,
            ..DynamicMoverDef::default()
        };
        mover.create(&mut world, &def);
    }

    // Stairs up to the bridge
    {
        let body_def = default_body_def();
        let stairs_id = create_body(&mut world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.base_material.custom_color = HexColor::OLIVE_DRAB.0;

        for i in 0..10 {
            let half_height = 0.1 * (i + 1) as f32;
            let transform = Transform {
                p: v3(-6.0, half_height, 3.3 + 0.6 * i as f32),
                q: QUAT_IDENTITY,
            };
            let box_hull = make_transformed_box_hull(1.5, half_height, 0.3, transform);
            create_hull_shape(&mut world, stairs_id, &shape_def, &box_hull.base);
            vis.push(VisBody::box_local(
                index_of(stairs_id),
                1.5,
                half_height,
                0.3,
                transform,
            ));
        }
    }

    // Platforms next to the bridge
    let mut tower_ids = [NULL_BODY_ID; 2];
    for (i, tower_id) in tower_ids.iter_mut().enumerate() {
        let mut body_def = default_body_def();
        body_def.position = pos(if i == 0 { -6.0 } else { 6.0 }, 1.0, 11.0);
        *tower_id = create_body(&mut world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.base_material.custom_color = HexColor::SLATE_GRAY.0;
        let box_hull = make_box_hull(1.5, 1.0, 2.0);
        create_hull_shape(&mut world, *tower_id, &shape_def, &box_hull.base);
        vis.push(VisBody::box_body(index_of(*tower_id), 1.5, 1.0, 2.0));
    }

    // Suspension bridge
    {
        let plank_count = 18;
        let plank_length = 9.0 / plank_count as f32;
        let x_base = -4.5f32;
        let y = 1.95f32;
        let z = 11.0f32;

        let mut plank_shape_def = default_shape_def();
        plank_shape_def.density = 300.0;
        plank_shape_def.base_material.custom_color = HexColor::BURLYWOOD.0;
        let plank = make_box_hull(0.5 * plank_length - 0.01, 0.05, 0.75);

        // The hinge is the local z-axis, across the bridge
        let mut joint_def = default_revolute_joint_def();
        joint_def.enable_motor = true;
        joint_def.max_motor_torque = 50.0;

        let mut prev_id = tower_ids[0];
        for i in 0..plank_count {
            let mut body_def = default_body_def();
            body_def.type_ = BodyType::Dynamic;
            body_def.position = pos(x_base + plank_length * (i as f32 + 0.5), y, z);
            body_def.angular_damping = 0.5;
            let plank_id = create_body(&mut world, &body_def);
            create_hull_shape(&mut world, plank_id, &plank_shape_def, &plank.base);
            vis.push(VisBody::box_body(
                index_of(plank_id),
                0.5 * plank_length - 0.01,
                0.05,
                0.75,
            ));

            let pivot = pos(x_base + plank_length * i as f32, y, z);
            joint_def.base.body_id_a = prev_id;
            joint_def.base.body_id_b = plank_id;
            joint_def.base.local_frame_a.p = body_get_local_point(&world, prev_id, pivot);
            joint_def.base.local_frame_b.p = body_get_local_point(&world, plank_id, pivot);
            create_revolute_joint(&mut world, &joint_def);

            prev_id = plank_id;
        }

        let pivot = pos(x_base + plank_length * plank_count as f32, y, z);
        joint_def.base.body_id_a = prev_id;
        joint_def.base.body_id_b = tower_ids[1];
        joint_def.base.local_frame_a.p = body_get_local_point(&world, prev_id, pivot);
        joint_def.base.local_frame_b.p = body_get_local_point(&world, tower_ids[1], pivot);
        create_revolute_joint(&mut world, &joint_def);
    }

    // Ramp to bridge.
    let steel = HexColor::LIGHT_STEEL_BLUE;
    add_ramp(
        &mut world,
        &mut vis,
        v3(6.0, 2.0, 9.0),
        v3(6.0, 0.0, 3.5),
        1.5,
        HexColor::OLIVE_DRAB,
    );

    // Wall
    add_ramp(
        &mut world,
        &mut vis,
        v3(20.0, 3.0, -40.0),
        v3(20.0, -0.5, -40.0),
        5.0,
        steel,
    );

    // Ramp
    add_ramp(
        &mut world,
        &mut vis,
        v3(35.0, 3.0, -46.0),
        v3(35.0, -0.5, -39.5),
        5.0,
        steel,
    );

    // Steep
    add_ramp(
        &mut world,
        &mut vis,
        v3(50.0, 3.0, -42.5),
        v3(50.0, -0.5, -39.5),
        5.0,
        steel,
    );

    // Curb
    add_ramp(
        &mut world,
        &mut vis,
        v3(65.0, 0.25, -40.0),
        v3(65.0, -0.5, -40.0),
        5.0,
        steel,
    );

    // Ledges
    for i in 0..2 {
        let mut body_def = default_body_def();
        body_def.position = pos(if i == 0 { 18.0 } else { 32.0 }, 2.0, 0.0);
        let ledge_id = create_body(&mut world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.base_material.custom_color = HexColor::DARK_SLATE_GRAY.0;
        let box_hull = make_box_hull(2.0, 2.0, 2.0);
        create_hull_shape(&mut world, ledge_id, &shape_def, &box_hull.base);
        vis.push(VisBody::box_body(index_of(ledge_id), 2.0, 2.0, 2.0));
    }

    // Moving platforms
    let mut platform_ids = [NULL_BODY_ID; PLATFORM_COUNT];
    for (i, platform_id) in platform_ids.iter_mut().enumerate() {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Kinematic;
        body_def.position = get_platform_position(i, 0.0);
        *platform_id = create_body(&mut world, &body_def);

        let mut platform_shape_def = default_shape_def();
        platform_shape_def.filter.category_bits = DYNAMIC_BIT;
        platform_shape_def.base_material.user_material_id = PLATFORM_MATERIAL;
        platform_shape_def.base_material.custom_color = HexColor::GOLDEN_ROD.0;
        let box_hull = make_box_hull(1.5, 0.1, 1.5);
        create_hull_shape(
            &mut world,
            *platform_id,
            &platform_shape_def,
            &box_hull.base,
        );
        vis.push(VisBody::box_body(index_of(*platform_id), 1.5, 0.1, 1.5));
    }

    // Seesaw
    {
        let mut body_def = default_body_def();
        body_def.position = pos(-14.0, 0.4, 0.0);
        let fulcrum_id = create_body(&mut world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.base_material.custom_color = HexColor::SLATE_GRAY.0;
        let fulcrum = make_box_hull(0.25, 0.4, 0.75);
        create_hull_shape(&mut world, fulcrum_id, &shape_def, &fulcrum.base);
        vis.push(VisBody::box_body(index_of(fulcrum_id), 0.25, 0.4, 0.75));

        body_def.type_ = BodyType::Dynamic;
        body_def.position = pos(-14.0, 0.9, 0.0);
        let plank_id = create_body(&mut world, &body_def);

        let mut plank_shape_def = default_shape_def();
        plank_shape_def.filter.category_bits = DYNAMIC_BIT;
        plank_shape_def.density = 200.0;
        plank_shape_def.base_material.custom_color = HexColor::BURLYWOOD.0;
        let plank = make_box_hull(3.5, 0.1, 0.8);
        create_hull_shape(&mut world, plank_id, &plank_shape_def, &plank.base);
        vis.push(VisBody::box_body(index_of(plank_id), 3.5, 0.1, 0.8));

        let pivot = pos(-14.0, 0.8, 0.0);
        let mut joint_def = default_revolute_joint_def();
        joint_def.base.body_id_a = fulcrum_id;
        joint_def.base.body_id_b = plank_id;
        joint_def.base.local_frame_a.p = body_get_local_point(&world, fulcrum_id, pivot);
        joint_def.base.local_frame_b.p = body_get_local_point(&world, plank_id, pivot);
        joint_def.enable_limit = true;
        joint_def.lower_angle = -0.3;
        joint_def.upper_angle = 0.3;
        create_revolute_joint(&mut world, &joint_def);
    }

    // Large pyramid past the seesaw, standing in the yz plane
    {
        // C: m_isDebug ? 20 : 100. The port reproduces the C debug build (see module docs).
        let base_count = 20i32;

        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;

        let mut box_shape_def = default_shape_def();
        box_shape_def.filter.category_bits = DYNAMIC_BIT;
        box_shape_def.density = 50.0;

        let h = 0.25f32;
        let cube = make_cube_hull(h);

        for i in 0..base_count {
            let y = (2.0 * i as f32 + 1.0) * h;

            for j in i..base_count {
                let z = (i as f32 + 1.0) * h + 2.0 * (j - i) as f32 * h - h * base_count as f32;

                body_def.position = pos(-50.0, y, z);
                let body_id = create_body(&mut world, &body_def);
                create_hull_shape(&mut world, body_id, &box_shape_def, &cube.base);
                vis.push(VisBody::box_body(index_of(body_id), h, h, h));
            }
        }
    }

    build_door(&mut world, &mut vis);
    build_ball_pit(&mut world, &mut vis, ground_id);
    let plate_joint_id = build_pressure_plate(&mut world, &mut vis, ground_id, &mover);

    // Skittles: `if constexpr ( m_isDebug == false )` — not part of the C debug build the
    // port reproduces (see module docs).

    // Big sphere
    {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = pos(6.0, 1.0, -6.0);
        let ball_id = create_body(&mut world, &body_def);

        let mut ball_shape_def = default_shape_def();
        ball_shape_def.filter.category_bits = DYNAMIC_BIT;
        ball_shape_def.density = 30.0;
        ball_shape_def.base_material.rolling_resistance = 0.05;
        ball_shape_def.base_material.custom_color = HexColor::TOMATO.0;
        let sphere = Sphere {
            center: VEC3_ZERO,
            radius: 1.0,
        };
        create_sphere_shape(&mut world, ball_id, &ball_shape_def, &sphere);
        vis.push(VisBody::sphere_body(index_of(ball_id), 1.0));
    }

    build_sweeper(&mut world, &mut vis, ground_id);

    // Jump puzzle
    for i in 0..PILLAR_COUNT {
        let top = get_pillar_top(i);

        let mut body_def = default_body_def();
        body_def.position = pos(top.x as f32, 0.0, top.z as f32);
        let pillar_id = create_body(&mut world, &body_def);

        let mut shape_def = default_shape_def();
        shape_def.base_material.custom_color = HexColor::MEDIUM_PURPLE.0;
        if let Some(pillar) = create_cylinder(top.y as f32, 0.75, 0.0, 12) {
            create_hull_shape(&mut world, pillar_id, &shape_def, &pillar);
            hulls.push(HullVis::on_body(pillar_id, hull_triangles(&pillar)));
        }
    }

    let mut gem_ids = [NULL_BODY_ID; GEM_COUNT];
    spawn_gems(&mut world, &mut gem_ids);

    // Flat ended so the muzzle sits at the front face
    let launcher_length = super::LAUNCHER_BACK + super::LAUNCHER_FRONT + super::LAUNCHER_RADIUS;
    let launcher_hull: Option<HullData> = create_cylinder(
        launcher_length,
        super::LAUNCHER_RADIUS,
        -super::LAUNCHER_BACK,
        5,
    );

    let mut sample = DynamicMoverSample::new(
        mover,
        camera,
        plate_joint_id,
        platform_ids,
        gem_ids,
        vis,
        hulls,
        launcher_hull,
    );
    // C: m_camera->m_thirdPerson = true; sapp_lock_mouse( true ) (the page locks on click).
    sample.camera.third_person = true;
    (world, sample)
}

/// The local gem box render transform (matches the gem hull's `{ zero, tilt }`).
pub(crate) fn gem_local() -> Transform {
    Transform {
        p: VEC3_ZERO,
        q: gem_tilt(),
    }
}
