//! Criterion bench for CreateManyPyramids / world.step.
//!
//! Run: `cargo bench --bench many_pyramids -- --quick`

use criterion::{black_box, criterion_group, criterion_main, Criterion};

#[path = "support.rs"]
mod support;

use support::{configure_group, create_many_pyramids, warm_up, SUB_STEP_COUNT, TIME_STEP};

fn many_pyramids(c: &mut Criterion) {
    let mut group = configure_group(c, "many_pyramids");

    let mut world = create_many_pyramids();
    warm_up(&mut world);

    group.bench_function("step", |b| {
        b.iter(|| {
            world.step(black_box(TIME_STEP), black_box(SUB_STEP_COUNT));
        });
    });

    group.finish();
}

criterion_group!(benches, many_pyramids);
criterion_main!(benches);
