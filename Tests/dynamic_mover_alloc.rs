//! A walking dynamic mover's update allocates nothing: a game runs one per character every
//! tick, so a heap allocation there is a per-character, per-tick cost.
//!
//! This is its own test binary because it installs a counting global allocator. The count
//! is per thread, so the harness's other threads never reach it.

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

use box3d_rust::body::create_body;
use box3d_rust::dynamic_mover::{DynamicMover, DynamicMoverDef};
use box3d_rust::hull::make_offset_box_hull;
use box3d_rust::math_functions::{Pos, Vec2, Vec3};
use box3d_rust::shape::create_hull_shape;
use box3d_rust::types::{default_body_def, default_shape_def, default_world_def};
use box3d_rust::world::World;

struct CountingAlloc;

thread_local! {
    static COUNTING: Cell<bool> = const { Cell::new(false) };
    static ALLOCATIONS: Cell<usize> = const { Cell::new(0) };
}

fn note_allocation() {
    if COUNTING.with(|c| c.get()) {
        ALLOCATIONS.with(|a| a.set(a.get() + 1));
    }
}

unsafe impl GlobalAlloc for CountingAlloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        note_allocation();
        System.alloc(layout)
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        note_allocation();
        System.alloc_zeroed(layout)
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        note_allocation();
        System.realloc(ptr, layout, new_size)
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        System.dealloc(ptr, layout)
    }
}

#[global_allocator]
static GLOBAL: CountingAlloc = CountingAlloc;

#[test]
fn walking_update_does_not_allocate() {
    let dt = 1.0 / 60.0;
    let mut world = World::new(&default_world_def());
    let ground = create_body(&mut world, &default_body_def());
    let floor = make_offset_box_hull(
        50.0,
        0.5,
        50.0,
        Vec3 {
            x: 0.0,
            y: -0.5,
            z: 0.0,
        },
    );
    create_hull_shape(&mut world, ground, &default_shape_def(), &floor.base);

    let mut mover = DynamicMover::new();
    let def = DynamicMoverDef {
        position: Pos {
            x: 0.0 as _,
            y: 1.5 as _,
            z: 0.0 as _,
        },
        ..DynamicMoverDef::default()
    };
    mover.create(&mut world, &def);

    let forward = Vec3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };
    let right = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    };
    let throttle = Vec2 { x: 1.0, y: 0.0 };

    // Settle onto the floor and get walking: the pogo joint exists from here on.
    for _ in 0..120 {
        mover.update(&mut world, dt, forward, right, throttle, 1.0);
        world.step(dt, 4);
    }
    assert!(mover.on_ground && mover.walkable);

    let mut allocations = 0;
    for _ in 0..120 {
        COUNTING.with(|c| c.set(true));
        mover.update(&mut world, dt, forward, right, throttle, 1.0);
        COUNTING.with(|c| c.set(false));
        allocations += ALLOCATIONS.with(|a| a.replace(0));
        world.step(dt, 4);
    }
    assert!(mover.on_ground && mover.walkable);
    assert_eq!(
        allocations, 0,
        "update allocated {allocations} times over 120 ticks"
    );
}
