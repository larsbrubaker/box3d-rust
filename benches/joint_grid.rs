//! Criterion bench for CreateJointGrid / world.step.
//!
//! Run: `cargo bench --bench joint_grid -- --quick`

use criterion::{black_box, criterion_group, criterion_main, Criterion};

#[path = "support.rs"]
mod support;

use support::{configure_group, create_joint_grid, warm_up, SUB_STEP_COUNT, TIME_STEP};

fn joint_grid(c: &mut Criterion) {
    let mut group = configure_group(c, "joint_grid");

    let mut world = create_joint_grid();
    warm_up(&mut world);

    group.bench_function("step", |b| {
        b.iter(|| {
            world.step(black_box(TIME_STEP), black_box(SUB_STEP_COUNT));
        });
    });

    group.finish();
}

criterion_group!(benches, joint_grid);
criterion_main!(benches);
