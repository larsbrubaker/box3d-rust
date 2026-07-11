// Focused unit tests for broad_phase proxy ops (C has no dedicated test file;
// update_pairs is deferred until World/contact exist).
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::broad_phase::{proxy_id, proxy_key, proxy_type, BroadPhase};
use crate::dynamic_tree::DEFAULT_CATEGORY_BITS;
use crate::math_functions::{Aabb, Vec3};
use crate::types::{BodyType, Capacity};

fn unit_aabb(x: f32, y: f32, z: f32) -> Aabb {
    Aabb {
        lower_bound: Vec3 {
            x: x - 0.5,
            y: y - 0.5,
            z: z - 0.5,
        },
        upper_bound: Vec3 {
            x: x + 0.5,
            y: y + 0.5,
            z: z + 0.5,
        },
    }
}

#[test]
fn proxy_key_round_trip() {
    for type_ in [BodyType::Static, BodyType::Kinematic, BodyType::Dynamic] {
        for id in [0, 1, 7, 100] {
            let key = proxy_key(id, type_);
            assert_eq!(proxy_type(key), type_);
            assert_eq!(proxy_id(key), id);
        }
    }
}

#[test]
fn create_destroy_and_move_buffer() {
    let capacity = Capacity {
        static_shape_count: 8,
        dynamic_shape_count: 8,
        static_body_count: 0,
        dynamic_body_count: 0,
        contact_count: 8,
    };
    let mut bp = BroadPhase::new(&capacity);

    // Static without forcePairCreation does not buffer a move.
    let static_key = bp.create_proxy(
        BodyType::Static,
        unit_aabb(0.0, 0.0, 0.0),
        DEFAULT_CATEGORY_BITS,
        10,
        false,
    );
    assert!(bp.move_array.is_empty());
    assert_eq!(bp.shape_index(static_key), 10);

    // Dynamic always buffers.
    let dyn_key = bp.create_proxy(
        BodyType::Dynamic,
        unit_aabb(2.0, 0.0, 0.0),
        DEFAULT_CATEGORY_BITS,
        20,
        false,
    );
    assert_eq!(bp.move_array, vec![dyn_key]);
    assert_eq!(bp.shape_index(dyn_key), 20);

    // Static with forcePairCreation buffers.
    let static_forced = bp.create_proxy(
        BodyType::Static,
        unit_aabb(-2.0, 0.0, 0.0),
        DEFAULT_CATEGORY_BITS,
        11,
        true,
    );
    assert!(bp.move_array.contains(&static_forced));

    // Destroy unbuffers.
    bp.destroy_proxy(dyn_key);
    assert!(!bp.move_array.contains(&dyn_key));

    bp.destroy();
}

#[test]
fn move_and_overlap() {
    let capacity = Capacity {
        static_shape_count: 4,
        dynamic_shape_count: 4,
        ..Capacity::default()
    };
    let mut bp = BroadPhase::new(&capacity);

    let a = bp.create_proxy(
        BodyType::Dynamic,
        unit_aabb(0.0, 0.0, 0.0),
        DEFAULT_CATEGORY_BITS,
        1,
        false,
    );
    let b = bp.create_proxy(
        BodyType::Dynamic,
        unit_aabb(0.25, 0.0, 0.0),
        DEFAULT_CATEGORY_BITS,
        2,
        false,
    );
    assert!(bp.test_overlap(a, b));

    bp.move_proxy(b, unit_aabb(10.0, 0.0, 0.0));
    assert!(!bp.test_overlap(a, b));
    assert!(bp.move_array.contains(&b));

    bp.validate();
    bp.validate_no_enlarged();
}

#[test]
fn defaults_have_secret_cookie() {
    use crate::core::SECRET_COOKIE;
    use crate::types::{default_body_def, default_filter, default_world_def};

    assert_eq!(default_body_def().internal_value, SECRET_COOKIE);
    assert_eq!(default_world_def().internal_value, SECRET_COOKIE);
    assert_eq!(default_filter().group_index, 0);
}
