//! Port of box3d-cpp-reference/test/test_id.c
//! SPDX-FileCopyrightText: 2023 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::id::*;

#[test]
fn id_round_trips() {
    let x: u64 = 0x0123_4567_89AB_CDEF;

    {
        let id = BodyId::load(x);
        let y = id.store();
        assert_eq!(x, y);
    }

    {
        let id = ShapeId::load(x);
        let y = id.store();
        assert_eq!(x, y);
    }

    {
        let id = JointId::load(x);
        let y = id.store();
        assert_eq!(x, y);
    }
}

// Not in test_id.c, but locks in world/contact packing and null semantics.
#[test]
fn world_null_and_contact_round_trip() {
    let a: u32 = 0x0123_4567;
    let id = WorldId::load(a);
    assert_eq!(id.store(), a);

    assert!(NULL_WORLD_ID.is_null());
    assert!(NULL_BODY_ID.is_null());
    assert!(NULL_SHAPE_ID.is_null());
    assert!(NULL_JOINT_ID.is_null());
    assert!(NULL_CONTACT_ID.is_null());
    assert!(WorldId::default().is_null());
    assert!(BodyId::default().is_null());
    assert!(!BodyId {
        index1: 1,
        ..Default::default()
    }
    .is_null());
    assert!(BodyId {
        index1: 1,
        ..Default::default()
    }
    .is_non_null());

    let a = BodyId {
        index1: 1,
        world0: 2,
        generation: 3,
    };
    let b = BodyId {
        index1: 1,
        world0: 2,
        generation: 3,
    };
    let c = BodyId {
        index1: 1,
        world0: 2,
        generation: 4,
    };
    assert!(a.id_equals(b));
    assert!(!a.id_equals(c));

    let values = [0x0123_4567u32, 0x0000_89AB, 0xCDEF_0011];
    let id = ContactId::load(values);
    assert_eq!(id.store(), values);
    assert_eq!(id.padding, 0);
    assert!(id.id_equals(ContactId::load(values)));
}
