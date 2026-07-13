//! The three sensor scenes hosted on the `sensors` route (verified exact — do
//! not alter their values): Sensor Visit / Sensor Hits (sample_events.cpp) and
//! Benchmark Sensor (sample_benchmark.cpp, 40×40 — the C value; the sample has
//! no debug split for `m_columnCount`/`m_rowCount`).

use super::{add_ground_box, new_world, SceneKind, SensorState, VisSet};
use crate::rng::XorShift32;
use crate::vis::{pos, sphere, vec3, VisBody};
use box3d_rust::body::{
    body_get_local_point, body_get_position, body_set_linear_velocity, create_body, destroy_body,
    make_body_id,
};
use box3d_rust::debug_draw::HexColor;
use box3d_rust::geometry::Capsule;
use box3d_rust::hull::{make_box_hull, make_cube_hull, make_transformed_box_hull};
use box3d_rust::id::{ShapeId, NULL_BODY_ID};
use box3d_rust::joint::{
    create_prismatic_joint, prismatic_joint_get_translation, prismatic_joint_set_motor_speed,
};
use box3d_rust::math_functions::{
    make_quat_from_axis_angle, offset_pos, Transform, Vec3, PI, QUAT_IDENTITY, VEC3_AXIS_Z,
    VEC3_ONE,
};
use box3d_rust::mesh::create_grid_mesh;
use box3d_rust::shape::{
    create_capsule_shape, create_hull_shape, create_mesh_shape, create_sphere_shape,
    shape_get_body, shape_get_user_data, shape_is_sensor, shape_is_valid,
};
use box3d_rust::types::{default_body_def, default_shape_def, BodyType};
use box3d_rust::world::{world_set_custom_filter_callback, World};

const ACTIVE_SENSOR_COLOR: u32 = 0x505050;
const ACTIVE_USER_DATA: u64 = 1;
// C sample_benchmark.cpp: m_columnCount = m_rowCount = 40 (no BENCHMARK_DEBUG split).
const BENCH_COLUMNS: i32 = 40;
const BENCH_ROWS: i32 = 40;

/// Ports `BenchmarkSensor::Filter`/`FilterFcn` (sample_benchmark.cpp:922-946).
/// C reads the sensor shape's userData through `b3Shape_GetUserData(shapeId)`; the
/// ported callback does the same via the `&World` it receives, decoding the demo's
/// userData encoding (bit 0 = active, `>>1 - 1` = row) exactly as
/// [`reset_benchmark`] writes it. C's `this` context (which carries `m_filterRow`)
/// maps to the `u64` filter-row passed at registration. Returns
/// `userData->active || userData->row != filterRow`, suppressing the filter-row
/// sensor touches. `enableCustomFiltering` is set only on the filter row, so the
/// callback is invoked solely for pairs involving one of those sensors.
fn bench_sensor_filter(world: &World, shape_a: ShapeId, shape_b: ShapeId, context: u64) -> bool {
    let filter_row = context as i32;
    // C picks idA's userData if idA is the sensor, else idB's (else no data).
    let user_data = if shape_is_sensor(world, shape_a) {
        Some(shape_get_user_data(world, shape_a))
    } else if shape_is_sensor(world, shape_b) {
        Some(shape_get_user_data(world, shape_b))
    } else {
        None
    };
    match user_data {
        Some(ud) => {
            let active = (ud & 1) != 0;
            let row = (ud >> 1) as i32 - 1;
            active || row != filter_row
        }
        None => true,
    }
}

pub(super) fn reset_visit() -> SensorState {
    let mut world = new_world();
    let mut vis = VisSet::default();

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(0.0, 12.5, 0.0);
    let visitor = create_body(&mut world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.enable_sensor_events = true;
    let dynamic_box = make_box_hull(0.5, 0.5, 0.5);
    create_hull_shape(&mut world, visitor, &shape_def, &dynamic_box.base);
    vis.push(
        VisBody::box_body(visitor.index1 - 1, 0.5, 0.5, 0.5),
        0,
        false,
    );

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Kinematic;
    body_def.position = pos(0.0, 2.0, 0.0);
    let sensor_body = create_body(&mut world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.is_sensor = true;
    shape_def.enable_sensor_events = true;
    let sensor_box = make_box_hull(2.0, 2.0, 2.0);
    let visit_sensor = create_hull_shape(&mut world, sensor_body, &shape_def, &sensor_box.base);
    vis.push(
        VisBody::box_body(sensor_body.index1 - 1, 2.0, 2.0, 2.0),
        0,
        true,
    );

    let mut state = SensorState::blank(world, vis, SceneKind::Visit);
    state.visit_sensor = visit_sensor;
    state
}

pub(super) fn reset_hits(is_bullet: bool) -> SensorState {
    let mut world = new_world();
    let mut vis = VisSet::default();

    let ground = add_ground_box(&mut world, 10.0);
    vis.push(
        VisBody::box_body(ground.index1 - 1, 10.0, 1.0, 10.0),
        0,
        false,
    );

    let body_def = default_body_def();
    let wall_body = create_body(&mut world, &body_def);
    let wall_xf = Transform {
        p: Vec3 {
            x: 10.0,
            y: 5.0,
            z: 0.0,
        },
        q: QUAT_IDENTITY,
    };
    let wall_box = make_transformed_box_hull(0.1, 5.0, 5.0, wall_xf);
    create_hull_shape(&mut world, wall_body, &default_shape_def(), &wall_box.base);
    vis.push(
        VisBody::box_local(wall_body.index1 - 1, 0.1, 5.0, 5.0, wall_xf),
        0,
        false,
    );

    let grid_mesh = create_grid_mesh(2, 2, 5.0, 0, true).expect("hits grid mesh");
    let mesh_rot = make_quat_from_axis_angle(VEC3_AXIS_Z, 0.5 * PI);
    let (mesh_hx, mesh_hy, mesh_hz) = (5.0, 0.05, 5.0);

    {
        let mut body_def = default_body_def();
        body_def.position = pos(-4.0, 6.0, 0.0);
        body_def.rotation = mesh_rot;
        let body = create_body(&mut world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.is_sensor = true;
        shape_def.enable_sensor_events = true;
        create_mesh_shape(&mut world, body, &shape_def, &grid_mesh, VEC3_ONE);
        vis.push(
            VisBody::box_body(body.index1 - 1, mesh_hx, mesh_hy, mesh_hz),
            0,
            true,
        );
    }

    let kinematic_body = {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Kinematic;
        body_def.position = pos(0.0, 6.0, 0.0);
        body_def.rotation = mesh_rot;
        body_def.linear_velocity = vec3(0.5, 0.0, 0.0);
        let body = create_body(&mut world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.is_sensor = true;
        shape_def.enable_sensor_events = true;
        create_mesh_shape(&mut world, body, &shape_def, &grid_mesh, VEC3_ONE);
        vis.push(
            VisBody::box_body(body.index1 - 1, mesh_hx, mesh_hy, mesh_hz),
            0,
            true,
        );
        body
    };

    let joint_id = {
        let mut body_def = default_body_def();
        body_def.type_ = BodyType::Dynamic;
        body_def.position = pos(4.0, 1.0, 0.0);
        let dynamic_body = create_body(&mut world, &body_def);
        let mut shape_def = default_shape_def();
        shape_def.is_sensor = true;
        shape_def.enable_sensor_events = true;
        let capsule = Capsule {
            center1: Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            center2: Vec3 {
                x: 0.0,
                y: 9.0,
                z: 0.0,
            },
            radius: 0.1,
        };
        create_capsule_shape(&mut world, dynamic_body, &shape_def, &capsule);
        vis.push(
            VisBody::capsule_body(dynamic_body.index1 - 1, &capsule),
            0,
            true,
        );

        let pivot = offset_pos(
            body_def.position,
            Vec3 {
                x: 0.0,
                y: 6.0,
                z: 0.0,
            },
        );
        let mut joint_def = box3d_rust::types::default_prismatic_joint_def();
        joint_def.base.body_id_a = wall_body;
        joint_def.base.body_id_b = dynamic_body;
        joint_def.base.local_frame_a.p = body_get_local_point(&world, wall_body, pivot);
        joint_def.base.local_frame_b.p = body_get_local_point(&world, dynamic_body, pivot);
        joint_def.enable_motor = true;
        joint_def.max_motor_force = 1000.0;
        joint_def.motor_speed = 0.5;
        create_prismatic_joint(&mut world, &joint_def)
    };

    let mut state = SensorState::blank(world, vis, SceneKind::Hits);
    state.grid_mesh = Some(grid_mesh);
    state.kinematic_body = kinematic_body;
    state.joint_id = joint_id;
    state.is_bullet = is_bullet;
    launch_bullet(&mut state);
    state
}

pub(super) fn launch_bullet(state: &mut SensorState) {
    if state.kind != SceneKind::Hits {
        return;
    }
    if !state.bullet_body.is_null() {
        let idx = state.bullet_body.index1 - 1;
        destroy_body(&mut state.world, state.bullet_body);
        state.vis.remove_body(idx);
        state.bullet_body = NULL_BODY_ID;
    }
    state.begin_total = 0;
    state.end_total = 0;
    state.last_begin = 0;
    state.last_end = 0;

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.position = pos(-26.7, 6.0, 0.0);
    let speed = state.rng.range(200.0, 300.0);
    body_def.linear_velocity = vec3(speed, 0.0, 0.0);
    body_def.is_bullet = state.is_bullet;
    let body = create_body(&mut state.world, &body_def);
    let mut shape_def = default_shape_def();
    shape_def.enable_sensor_events = true;
    shape_def.base_material.friction = 0.8;
    shape_def.base_material.rolling_resistance = 0.01;
    let sph = sphere(0.25);
    create_sphere_shape(&mut state.world, body, &shape_def, &sph);
    state
        .vis
        .push(VisBody::sphere_body(body.index1 - 1, 0.25), 0, false);
    state.bullet_body = body;
}

pub(super) fn reset_benchmark() -> SensorState {
    let mut world = new_world();
    let mut vis = VisSet::default();
    // Seed 42 is exact, not a typo for the global RAND_SEED (12345): the C
    // BenchmarkSensor ctor overrides the RNG with `g_randomSeed = 42;`
    // (sample_benchmark.cpp:779) before CreateRow draws its jitter. Matches the
    // pre-split module (`sensor_demo.rs` reset_benchmark).
    let rng = XorShift32::with_seed(42);

    // b3World_SetCustomFilterCallback(m_worldId, FilterFcn, this): C passes `this`
    // (holding m_filterRow) as the context. We pass the filter row itself so the
    // callback can compare against it, matching C's `m_filterRow` access.
    let filter_row = BENCH_ROWS >> 1;
    world_set_custom_filter_callback(&mut world, Some(bench_sensor_filter), filter_row as u64);

    let grid_size = 3.0;
    let half = 0.48 * grid_size;
    let box_hull = make_cube_hull(half);
    {
        let mut body_def = default_body_def();
        let mut shape_def = default_shape_def();
        shape_def.is_sensor = true;
        shape_def.enable_sensor_events = true;
        shape_def.user_data = ACTIVE_USER_DATA;
        let count = BENCH_COLUMNS * 2 + 1;
        let mut x = -(BENCH_COLUMNS as f32) * grid_size;
        for _ in 0..count {
            body_def.position = pos(x, 0.0, 0.0);
            let id = create_body(&mut world, &body_def);
            create_hull_shape(&mut world, id, &shape_def, &box_hull.base);
            vis.push(
                VisBody::box_body(id.index1 - 1, half, half, half),
                ACTIVE_SENSOR_COLOR,
                true,
            );
            x += grid_size;
        }
    }

    let shift = 5.0;
    let x_center = 0.5 * shift * BENCH_COLUMNS as f32;
    let cube = make_cube_hull(0.5);
    let y_start = 10.0;
    {
        let mut body_def = default_body_def();
        let mut shape_def = default_shape_def();
        shape_def.is_sensor = true;
        shape_def.enable_sensor_events = true;
        for j in 0..BENCH_ROWS {
            shape_def.user_data = ((j as u64) + 1) << 1;
            // C enables custom filtering + fuchsia only on the filter row.
            shape_def.enable_custom_filtering = j == filter_row;
            let color = if j == filter_row {
                HexColor::FUCHSIA.0
            } else {
                0
            };
            let y = j as f32 * shift + y_start;
            for i in 0..BENCH_COLUMNS {
                let x = i as f32 * shift - x_center;
                body_def.position = pos(x, y, 0.0);
                let id = create_body(&mut world, &body_def);
                create_hull_shape(&mut world, id, &shape_def, &cube.base);
                vis.push(VisBody::box_body(id.index1 - 1, 0.5, 0.5, 0.5), color, true);
            }
        }
    }

    let mut state = SensorState::blank(world, vis, SceneKind::Benchmark);
    state.filter_row = filter_row;
    state.rng = rng;
    state
}

fn create_bench_row(state: &mut SensorState, y: f32) {
    let shift = 5.0;
    let x_center = 0.5 * shift * BENCH_COLUMNS as f32;
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.gravity_scale = 0.0;
    body_def.linear_velocity = vec3(0.0, -5.0, 0.0);
    let mut shape_def = default_shape_def();
    shape_def.enable_sensor_events = true;
    let sph = sphere(0.5);
    for i in 0..BENCH_COLUMNS {
        let y_offset = state.rng.range(-1.0, 1.0);
        body_def.position = pos(shift * i as f32 - x_center, y + y_offset, 0.0);
        let body = create_body(&mut state.world, &body_def);
        create_sphere_shape(&mut state.world, body, &shape_def, &sph);
        state
            .vis
            .push(VisBody::sphere_body(body.index1 - 1, 0.5), 0, false);
    }
}

pub(super) fn process_visit_events(state: &mut SensorState) {
    let begins: Vec<_> = state.world.get_sensor_events().begin_events.to_vec();
    let ends: Vec<_> = state.world.get_sensor_events().end_events.to_vec();
    state.last_begin = begins.len() as u32;
    state.last_end = ends.len() as u32;
    state.begin_total = state.begin_total.wrapping_add(state.last_begin);
    state.end_total = state.end_total.wrapping_add(state.last_end);
    for event in &begins {
        if event.sensor_shape_id == state.visit_sensor {
            let visitor = shape_get_body(&state.world, event.visitor_shape_id);
            let idx = visitor.index1 - 1;
            destroy_body(&mut state.world, visitor);
            state.vis.remove_body(idx);
            break;
        }
    }
}

pub(super) fn process_hits_pre_step(state: &mut SensorState) {
    let p = body_get_position(&state.world, state.kinematic_body);
    if (p.x as f32) > 1.0 {
        body_set_linear_velocity(&mut state.world, state.kinematic_body, vec3(-0.5, 0.0, 0.0));
    } else if (p.x as f32) < -1.0 {
        body_set_linear_velocity(&mut state.world, state.kinematic_body, vec3(0.5, 0.0, 0.0));
    }
    let x = prismatic_joint_get_translation(&state.world, state.joint_id);
    if x > 1.0 {
        prismatic_joint_set_motor_speed(&mut state.world, state.joint_id, -0.5);
    } else if x < -1.0 {
        prismatic_joint_set_motor_speed(&mut state.world, state.joint_id, 0.5);
    }
}

pub(super) fn process_hits_events(state: &mut SensorState) {
    let begin_count = state.world.get_sensor_events().begin_events.len() as u32;
    let end_count = state.world.get_sensor_events().end_events.len() as u32;
    state.last_begin = begin_count;
    state.last_end = end_count;
    state.begin_total = state.begin_total.wrapping_add(begin_count);
    state.end_total = state.end_total.wrapping_add(end_count);
}

pub(super) fn process_benchmark_events(state: &mut SensorState) {
    if state.step_count == state.last_step_count {
        return;
    }
    let begins: Vec<_> = state.world.get_sensor_events().begin_events.to_vec();
    let ends: Vec<_> = state.world.get_sensor_events().end_events.to_vec();
    state.last_begin = begins.len() as u32;
    state.last_end = ends.len() as u32;
    state.begin_total = state.begin_total.wrapping_add(state.last_begin);
    state.end_total = state.end_total.wrapping_add(state.last_end);
    state.max_begin = state.max_begin.max(state.last_begin);
    state.max_end = state.max_end.max(state.last_end);

    let mut zombie_indices: Vec<i32> = Vec::new();
    for event in &begins {
        let user_data = shape_get_user_data(&state.world, event.sensor_shape_id);
        if user_data == ACTIVE_USER_DATA {
            zombie_indices.push(shape_get_body(&state.world, event.visitor_shape_id).index1 - 1);
        } else {
            // Custom filter suppresses filter-row touches, so a passive sensor that
            // begins a touch here is never on the filter row (C: assert row != filter).
            let row = (user_data >> 1) as i32 - 1;
            debug_assert!(row != state.filter_row);
            if let Some(i) = state
                .vis
                .find(shape_get_body(&state.world, event.visitor_shape_id).index1 - 1)
            {
                state.vis.set_color(i, HexColor::LIME.0);
            }
        }
    }
    for event in &ends {
        if !shape_is_valid(&state.world, event.visitor_shape_id) {
            continue;
        }
        let visitor = shape_get_body(&state.world, event.visitor_shape_id);
        if let Some(i) = state.vis.find(visitor.index1 - 1) {
            state.vis.set_color(i, 0);
        }
    }
    zombie_indices.sort_unstable();
    zombie_indices.dedup();
    for idx in zombie_indices {
        let body_id = make_body_id(&state.world, idx);
        destroy_body(&mut state.world, body_id);
        state.vis.remove_body(idx);
    }
    if (state.step_count & 0x1F) == 0 {
        create_bench_row(state, 10.0 + BENCH_ROWS as f32 * 5.0);
    }
    state.last_step_count = state.step_count;
}
