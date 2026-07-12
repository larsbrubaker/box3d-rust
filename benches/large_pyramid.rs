//! Criterion bench for CreateLargePyramid / world.step.
//!
//! Run: `cargo bench --bench large_pyramid -- --quick`

use criterion::{black_box, criterion_group, criterion_main, Criterion};

#[path = "support.rs"]
mod support;

use support::{configure_group, create_large_pyramid, warm_up, SUB_STEP_COUNT, TIME_STEP};

fn large_pyramid(c: &mut Criterion) {
    let mut group = configure_group(c, "large_pyramid");

    let mut world = create_large_pyramid();
    warm_up(&mut world);

    group.bench_function("step", |b| {
        b.iter(|| {
            world.step(black_box(TIME_STEP), black_box(SUB_STEP_COUNT));
        });
    });

    group.finish();
}

criterion_group!(benches, large_pyramid);
criterion_main!(benches);
