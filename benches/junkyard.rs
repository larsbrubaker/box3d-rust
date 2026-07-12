//! Criterion bench for CreateJunkyard / StepJunkyard + world.step.
//!
//! Run: `cargo bench --bench junkyard -- --quick`

use criterion::{criterion_group, criterion_main, Criterion};

#[path = "support.rs"]
mod support;

use support::{configure_group, Junkyard};

fn junkyard(c: &mut Criterion) {
    let mut group = configure_group(c, "junkyard");

    let mut scene = Junkyard::create();
    // C warm-up: StepJunkyard(world, 0) then one World_Step.
    scene.step();

    group.bench_function("step", |b| {
        b.iter(|| {
            scene.step();
        });
    });

    group.finish();
}

criterion_group!(benches, junkyard);
criterion_main!(benches);
