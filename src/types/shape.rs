// Shape filter types and defaults from types.h / types.c.
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::dynamic_tree::{DEFAULT_CATEGORY_BITS, DEFAULT_MASK_BITS};

/// This is used to filter collisions. (b3Filter)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Filter {
    /// The collision category bits. Normally you would just set one bit.
    pub category_bits: u64,
    /// The collision mask bits. Categories this shape accepts for collision.
    pub mask_bits: u64,
    /// Collision groups: negative never collide, positive always collide.
    /// Zero has no effect. Non-zero group filtering always wins against masks.
    pub group_index: i32,
}

/// Use this to initialize your filter. (b3DefaultFilter)
pub fn default_filter() -> Filter {
    Filter {
        category_bits: DEFAULT_CATEGORY_BITS,
        mask_bits: DEFAULT_MASK_BITS,
        group_index: 0,
    }
}

impl Default for Filter {
    fn default() -> Self {
        default_filter()
    }
}

/// The query filter is used to filter collisions between queries and shapes.
/// (b3QueryFilter)
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QueryFilter {
    /// The collision category bits of this query.
    pub category_bits: u64,
    /// The collision mask bits. Shape categories this query accepts.
    pub mask_bits: u64,
    /// Optional id combined with [`Self::name`] to identify this query in a recording.
    pub id: u64,
    /// Optional label combined with [`Self::id`] for recording. Empty means none.
    pub name: String,
}

/// Use this to initialize your query filter. (b3DefaultQueryFilter)
pub fn default_query_filter() -> QueryFilter {
    QueryFilter {
        category_bits: DEFAULT_CATEGORY_BITS,
        mask_bits: DEFAULT_MASK_BITS,
        id: 0,
        name: String::new(),
    }
}

impl Default for QueryFilter {
    fn default() -> Self {
        default_query_filter()
    }
}
