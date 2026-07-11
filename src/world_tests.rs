//! World step skeleton tests. EmptyWorld from test_world.c; HelloWorld waits
//! for collide/solve.

use crate::types::default_world_def;
use crate::world::World;

/// (EmptyWorld)
#[test]
fn empty_world() {
    let mut world = World::new(&default_world_def());
    let time_step = 1.0 / 60.0;
    let sub_step_count = 1;

    for _ in 0..60 {
        world.step(time_step, sub_step_count);
    }

    assert_eq!(world.step_index, 60);
}
