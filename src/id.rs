//! Port of box3d-cpp-reference/include/box3d/id.h
//!
//! These ids are opaque handles to internal Box3D objects, passed by value. All
//! ids are null when zero-initialized. The store/load helpers pack and unpack a
//! handle into a plain integer; the bit layout is reproduced exactly so handles
//! round-trip identically to the C library.
//!
//! Box3D has no chain id (unlike Box2D). Contact ids pack into three `u32`
//! values rather than a single `u64`.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

/// World id references a world instance. Treat as an opaque handle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct WorldId {
    pub index1: u16,
    pub generation: u16,
}

/// Body id references a body instance. Treat as an opaque handle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BodyId {
    pub index1: i32,
    pub world0: u16,
    pub generation: u16,
}

/// Shape id references a shape instance. Treat as an opaque handle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ShapeId {
    pub index1: i32,
    pub world0: u16,
    pub generation: u16,
}

/// Joint id references a joint instance. Treat as an opaque handle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct JointId {
    pub index1: i32,
    pub world0: u16,
    pub generation: u16,
}

/// Contact id references a contact instance. Treat as an opaque handle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ContactId {
    pub index1: i32,
    pub world0: u16,
    pub padding: i16,
    pub generation: u32,
}

/// A null world id. (`b3_nullWorldId`)
pub const NULL_WORLD_ID: WorldId = WorldId {
    index1: 0,
    generation: 0,
};

/// A null body id. (`b3_nullBodyId`)
pub const NULL_BODY_ID: BodyId = BodyId {
    index1: 0,
    world0: 0,
    generation: 0,
};

/// A null shape id. (`b3_nullShapeId`)
pub const NULL_SHAPE_ID: ShapeId = ShapeId {
    index1: 0,
    world0: 0,
    generation: 0,
};

/// A null joint id. (`b3_nullJointId`)
pub const NULL_JOINT_ID: JointId = JointId {
    index1: 0,
    world0: 0,
    generation: 0,
};

/// A null contact id. (`b3_nullContactId`)
pub const NULL_CONTACT_ID: ContactId = ContactId {
    index1: 0,
    world0: 0,
    padding: 0,
    generation: 0,
};

impl WorldId {
    /// Store a world id into a u32. (`b3StoreWorldId`)
    pub fn store(self) -> u32 {
        ((self.index1 as u32) << 16) | (self.generation as u32)
    }

    /// Load a u32 into a world id. (`b3LoadWorldId`)
    pub fn load(x: u32) -> WorldId {
        WorldId {
            index1: (x >> 16) as u16,
            generation: x as u16,
        }
    }

    /// True if this id is null (`index1 == 0`). (`B3_IS_NULL`)
    pub fn is_null(self) -> bool {
        self.index1 == 0
    }

    /// True if this id is non-null. (`B3_IS_NON_NULL`)
    pub fn is_non_null(self) -> bool {
        self.index1 != 0
    }
}

// Body, shape, and joint ids share the same 64-bit layout, so a macro generates
// their identical store/load/null helpers.
macro_rules! impl_u64_id {
    ($ty:ident) => {
        impl $ty {
            /// Store this id into a u64.
            pub fn store(self) -> u64 {
                ((self.index1 as u64) << 32)
                    | ((self.world0 as u64) << 16)
                    | (self.generation as u64)
            }

            /// Load a u64 into this id type.
            pub fn load(x: u64) -> $ty {
                $ty {
                    index1: (x >> 32) as i32,
                    world0: (x >> 16) as u16,
                    generation: x as u16,
                }
            }

            /// True if this id is null (`index1 == 0`). (`B3_IS_NULL`)
            pub fn is_null(self) -> bool {
                self.index1 == 0
            }

            /// True if this id is non-null. (`B3_IS_NON_NULL`)
            pub fn is_non_null(self) -> bool {
                self.index1 != 0
            }

            /// Compare two ids for equality. (`B3_ID_EQUALS`)
            pub fn id_equals(self, other: Self) -> bool {
                self.index1 == other.index1
                    && self.world0 == other.world0
                    && self.generation == other.generation
            }
        }
    };
}

impl_u64_id!(BodyId);
impl_u64_id!(ShapeId);
impl_u64_id!(JointId);

impl ContactId {
    /// Store a contact id into three u32 values. (`b3StoreContactId`)
    pub fn store(self) -> [u32; 3] {
        [self.index1 as u32, self.world0 as u32, self.generation]
    }

    /// Load three u32 values into a contact id. (`b3LoadContactId`)
    pub fn load(values: [u32; 3]) -> ContactId {
        ContactId {
            index1: values[0] as i32,
            world0: values[1] as u16,
            padding: 0,
            generation: values[2],
        }
    }

    /// True if this id is null (`index1 == 0`). (`B3_IS_NULL`)
    pub fn is_null(self) -> bool {
        self.index1 == 0
    }

    /// True if this id is non-null. (`B3_IS_NON_NULL`)
    pub fn is_non_null(self) -> bool {
        self.index1 != 0
    }

    /// Compare two contact ids for equality. (`B3_ID_EQUALS`)
    pub fn id_equals(self, other: Self) -> bool {
        self.index1 == other.index1
            && self.world0 == other.world0
            && self.generation == other.generation
    }
}
