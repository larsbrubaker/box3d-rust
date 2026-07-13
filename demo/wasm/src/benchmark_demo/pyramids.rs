//! Pyramid benchmarks: Large Pyramid, Wide Pyramid, Many Pyramids
//! (`shared/benchmarks.c` :101 / :144 / :212).

use super::{add_ground_box, empty_scene, new_world, BenchKind, BenchScene};
use crate::vis::VisBody;
use box3d_rust::body::create_body;
use box3d_rust::hull::make_box_hull;
use box3d_rust::math_functions::Pos;
use box3d_rust::shape::create_hull_shape;
use box3d_rust::types::{default_body_def, default_shape_def, BodyType};
use box3d_rust::world::world_enable_sleeping;

/// `CreateLargePyramid` (`benchmarks.c` :101). Sleep disabled, density 100,
/// half-size 0.5. `baseCount = DEBUG 20 : release 90`; the browser uses **20**
/// (release 90 is a 3D pyramid of hundreds of thousands of bodies, far beyond
/// serial wasm). C exposes no runtime baseCount control, so 20 is pinned.
pub(crate) fn build_large_pyramid() -> BenchScene {
    let base = 20i32;
    let world = new_world();
    let mut scene = empty_scene(world, Vec::new(), BenchKind::LargePyramid);
    world_enable_sleeping(&mut scene.world, false);

    add_ground_box(&mut scene, 400.0);

    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let mut shape_def = default_shape_def();
    shape_def.density = 100.0;
    let h = 0.5f32;
    let box_hull = make_box_hull(h, h, h);
    let shift = 1.0 * h;

    for i in 0..base {
        let y = (2.0 * i as f32 + 1.0) * shift;
        for j in i..base {
            let x = (i as f32 + 1.0) * shift + 2.0 * (j - i) as f32 * shift - h * base as f32;
            body_def.position = Pos { x, y, z: 0.0 };
            let body = create_body(&mut scene.world, &body_def);
            create_hull_shape(&mut scene.world, body, &shape_def, &box_hull.base);
            scene
                .bodies
                .push(VisBody::box_body(body.index1 - 1, h, h, h));
        }
    }
    scene
}

/// `CreateWidePyramid` (`benchmarks.c` :144). `pyramidHeight = DEBUG 5 : release 15`.
/// The browser uses **15** (the C release value): a full 3D wide pyramid of 1240
/// boxes, which serial wasm holds interactively.
pub(crate) fn build_wide_pyramid() -> BenchScene {
    let world = new_world();
    let mut scene = empty_scene(world, Vec::new(), BenchKind::WidePyramid);

    add_ground_box(&mut scene, 100.0);

    let box_size = 2.0f32;
    let box_separation = 0.5f32;
    let half_box_size = 0.5 * box_size;
    let pyramid_height = 15i32;

    let h = half_box_size - 0.025;
    let box_hull = make_box_hull(h, h, h);
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    let shape_def = default_shape_def();

    for i in 0..pyramid_height {
        let j_start = i / 2;
        let j_end = pyramid_height - (i + 1) / 2;
        for j in j_start..j_end {
            for k in j_start..j_end {
                let odd = if i & 1 != 0 { half_box_size } else { 0.0 };
                let x = -(pyramid_height as f32) + box_size * j as f32 + odd;
                let y = 1.0 + (box_size + box_separation) * i as f32;
                let z = -(pyramid_height as f32) + box_size * k as f32 + odd;
                body_def.position = Pos { x, y, z };
                let body = create_body(&mut scene.world, &body_def);
                create_hull_shape(&mut scene.world, body, &shape_def, &box_hull.base);
                scene
                    .bodies
                    .push(VisBody::box_body(body.index1 - 1, h, h, h));
            }
        }
    }
    scene
}

/// `CreateSmallPyramid` (`benchmarks.c` :186) — one density-100 pyramid, sleep
/// disabled per body.
fn create_small_pyramid(
    scene: &mut BenchScene,
    base_count: i32,
    extent: f32,
    center_x: f32,
    base_z: f32,
) {
    let mut body_def = default_body_def();
    body_def.type_ = BodyType::Dynamic;
    body_def.enable_sleep = false;
    let mut shape_def = default_shape_def();
    shape_def.density = 100.0;
    let box_hull = make_box_hull(extent, extent, extent);

    for i in 0..base_count {
        let y = (2.0 * i as f32 + 1.0) * extent;
        for j in i..base_count {
            let x = (i as f32 + 1.0) * extent + 2.0 * (j - i) as f32 * extent + center_x - 0.5;
            body_def.position = Pos { x, y, z: base_z };
            let body = create_body(&mut scene.world, &body_def);
            create_hull_shape(&mut scene.world, body, &shape_def, &box_hull.base);
            scene
                .bodies
                .push(VisBody::box_body(body.index1 - 1, extent, extent, extent));
        }
    }
}

/// `CreateManyPyramids` (`benchmarks.c` :212). `rowCount = columnCount = DEBUG 3 :
/// release 14`; the browser uses **3** (release 14×14 = 196 pyramids × 55 boxes =
/// 10 780 dynamic bodies, which serial wasm does not hold interactively).
pub(crate) fn build_many_pyramids() -> BenchScene {
    let base_count = 10i32;
    let extent = 0.5f32;
    let row_count = 3i32;
    let column_count = 3i32;
    let ground_extent = extent * column_count as f32 * (base_count as f32 + 1.0);

    let world = new_world();
    let mut scene = empty_scene(world, Vec::new(), BenchKind::ManyPyramids);

    add_ground_box(&mut scene, ground_extent);

    let base_width = 2.0 * extent * base_count as f32;
    let mut base_z = -ground_extent + 2.0 * extent;
    let delta_z = 2.0 * (ground_extent - 2.0 * extent) / (row_count as f32 - 1.0);

    for _i in 0..row_count {
        for j in 0..column_count {
            let center_x = -ground_extent + j as f32 * (base_width + 2.0 * extent) + 2.0 * extent;
            create_small_pyramid(&mut scene, base_count, extent, center_x, base_z);
        }
        base_z += delta_z;
    }
    scene
}
