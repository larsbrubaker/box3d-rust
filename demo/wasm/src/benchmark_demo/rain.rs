//! Rain benchmark (`sample_benchmark.cpp` BenchmarkRain :103; `benchmarks.c`
//! CreateRain :301 / StepRain :384). A grid of static mesh platforms onto which
//! groups of library ragdolls ("humans") rain down, column by column, recycling
//! once the field is full.

use super::{empty_scene, new_world, BenchKind, BenchScene};
use crate::vis::{capsule_from_body, mesh_triangle_edges_offset, VisBody};
use box3d_rust::body::create_body;
use box3d_rust::human::{create_human, destroy_human, Human, BONE_COUNT};
use box3d_rust::math_functions::{Pos, Vec3, VEC3_ONE};
use box3d_rust::mesh::{create_grid_mesh, create_torus_mesh};
use box3d_rust::shape::create_mesh_shape;
use box3d_rust::types::{default_body_def, default_shape_def};

const RAIN_GRID_SIZE: f32 = 15.0;

/// Live Rain state (`g_rainData`). Uses the C DEBUG configuration — `GRID_COUNT 3`,
/// `GROUP_SIZE 2` (release: 10 / 3, i.e. 100 groups × 3 = 300 ragdolls, which
/// serial wasm does not hold interactively).
pub(crate) struct RainState {
    pub grid_count: i32,
    pub group_size: i32,
    pub delay: u32,
    /// `grid_count²·group_size` humans, indexed `groupIndex*group_size + i`.
    pub humans: Vec<Human>,
    pub column_count: i32,
    pub column_index: i32,
    /// True after the last churn changed the roster, so the render list rebuilds.
    pub dirty: bool,
}

/// `CreateRain` (`benchmarks.c` :301). Builds the static mesh platforms and seeds
/// the empty human roster; `StepRain` fills it over time.
pub(crate) fn build_rain() -> BenchScene {
    let grid_count = 3i32;
    let group_size = 2i32;
    let delay = 0x7Fu32; // BENCHMARK_DEBUG delay

    let world = new_world();
    let mut scene = empty_scene(world, Vec::new(), BenchKind::Rain);

    // Platform meshes: grid (8×8 cells) + torus, shared across every platform body.
    let half_mesh_grid_rows = 4;
    let cell_width = RAIN_GRID_SIZE / (2.0 * half_mesh_grid_rows as f32);
    let grid_mesh = create_grid_mesh(
        2 * half_mesh_grid_rows,
        2 * half_mesh_grid_rows,
        cell_width,
        1,
        true,
    )
    .expect("rain grid mesh");
    let torus_mesh =
        create_torus_mesh(16, 16, 0.25 * RAIN_GRID_SIZE, 1.0).expect("rain torus mesh");

    let span = RAIN_GRID_SIZE * grid_count as f32;
    let shape_def = default_shape_def();
    let mut edges: Vec<f32> = Vec::new();

    let mut x = -0.5 * span + 0.5 * RAIN_GRID_SIZE;
    for _i in 0..grid_count {
        let mut z = -0.5 * span + 0.5 * RAIN_GRID_SIZE;
        for _j in 0..grid_count {
            let mut body_def = default_body_def();
            body_def.position = Pos { x, y: 0.0, z };
            let body = create_body(&mut scene.world, &body_def);
            create_mesh_shape(&mut scene.world, body, &shape_def, &grid_mesh, VEC3_ONE);
            create_mesh_shape(&mut scene.world, body, &shape_def, &torus_mesh, VEC3_ONE);
            let offset = Vec3 { x, y: 0.0, z };
            edges.extend(mesh_triangle_edges_offset(&grid_mesh, VEC3_ONE, offset));
            edges.extend(mesh_triangle_edges_offset(&torus_mesh, VEC3_ONE, offset));
            z += RAIN_GRID_SIZE;
        }
        x += RAIN_GRID_SIZE;
    }
    scene.ground_edges = edges;

    let count = (grid_count * grid_count * group_size) as usize;
    let mut humans = Vec::with_capacity(count);
    for _ in 0..count {
        humans.push(Human::default());
    }

    scene.rain = Some(RainState {
        grid_count,
        group_size,
        delay,
        humans,
        column_count: 0,
        column_index: 0,
        dirty: false,
    });
    scene
}

/// `CreateGroup` (`benchmarks.c` :345) — spawn one group of ragdolls at grid cell
/// `(rowIndex, columnIndex)`.
fn create_group(scene: &mut BenchScene, rain: &mut RainState, row_index: i32, column_index: i32) {
    let group_index = row_index * rain.grid_count + column_index;
    let span = rain.grid_count as f32 * RAIN_GRID_SIZE;
    let group_distance = span / rain.grid_count as f32;

    let mut position = Pos {
        x: -0.5 * span + group_distance * (column_index as f32 + 0.5),
        y: 20.0,
        z: -0.5 * span + group_distance * (row_index as f32 + 0.5),
    };

    for i in 0..rain.group_size {
        let idx = (group_index * rain.group_size + i) as usize;
        create_human(
            &mut rain.humans[idx],
            &mut scene.world,
            position,
            5.0,
            1.0,
            0.7,
            group_index,
            0,
            false,
        );
        position.x += 0.75;
    }
}

/// `DestroyGroup` (`benchmarks.c` :372).
fn destroy_group(scene: &mut BenchScene, rain: &mut RainState, row_index: i32, column_index: i32) {
    let group_index = row_index * rain.grid_count + column_index;
    for i in 0..rain.group_size {
        let idx = (group_index * rain.group_size + i) as usize;
        destroy_human(&mut rain.humans[idx], &mut scene.world);
    }
}

/// `StepRain` (`benchmarks.c` :384). Fills columns until the field is full, then
/// recycles one column per churn. Non-large-world increment is 1.
pub(crate) fn step_rain(scene: &mut BenchScene) {
    let Some(mut rain) = scene.rain.take() else {
        return;
    };

    if (scene.step_count & rain.delay) == 0 {
        if rain.column_count < rain.grid_count {
            let col = rain.column_count;
            for i in 0..rain.grid_count {
                create_group(scene, &mut rain, i, col);
            }
            rain.column_count = (rain.column_count + 1).min(rain.grid_count);
        } else {
            let col = rain.column_index;
            for i in 0..rain.grid_count {
                destroy_group(scene, &mut rain, i, col);
                create_group(scene, &mut rain, i, col);
            }
            rain.column_index += 1;
            if rain.column_index >= rain.grid_count {
                rain.column_index = 0;
            }
        }
        rain.dirty = true;
    }

    if rain.dirty {
        rebuild_render_list(scene, &rain);
        rain.dirty = false;
    }

    scene.rain = Some(rain);
}

/// Rebuild the capsule render list from every spawned human's bones.
fn rebuild_render_list(scene: &mut BenchScene, rain: &RainState) {
    scene.bodies.clear();
    for human in &rain.humans {
        if !human.is_spawned {
            continue;
        }
        for b in 0..BONE_COUNT {
            let body_id = human.bones[b].body_id;
            if body_id.index1 == 0 {
                continue;
            }
            let body_index = body_id.index1 - 1;
            if let Some(cap) = capsule_from_body(&scene.world, body_index) {
                scene.bodies.push(VisBody::capsule_body(body_index, &cap));
            }
        }
    }
}
