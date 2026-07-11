// Shape filter types and defaults from types.h / types.c.
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::core::{get_length_units_per_meter, SECRET_COOKIE};
use crate::dynamic_tree::{DEFAULT_CATEGORY_BITS, DEFAULT_MASK_BITS};
use crate::geometry::{default_surface_material, SurfaceMaterial};

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

/// Used to create a shape. (b3ShapeDef)
#[derive(Debug, Clone)]
pub struct ShapeDef {
    /// Optional shape name for debugging.
    pub name: String,
    /// Application-specific shape data.
    pub user_data: u64,
    /// Per-triangle materials for meshes. Empty means use [`Self::base_material`].
    /// Ignored for convex shapes and compounds.
    pub materials: Vec<SurfaceMaterial>,
    /// The base surface material. Ignored for compound shapes.
    pub base_material: SurfaceMaterial,
    /// The density, usually in kg/m^3.
    pub density: f32,
    /// Explosion scale for `World::explode`. Non-dimensional.
    pub explosion_scale: f32,
    /// Contact filtering data.
    pub filter: Filter,
    /// Enable custom filtering. Only one of the two shapes needs to enable it.
    pub enable_custom_filtering: bool,
    /// A sensor shape generates overlap events but never a collision response.
    pub is_sensor: bool,
    /// Enable sensor events for this shape. False by default, even for sensors.
    pub enable_sensor_events: bool,
    /// Enable contact events. Only kinematic/dynamic; ignored for sensors.
    pub enable_contact_events: bool,
    /// Enable hit events. Only kinematic/dynamic; ignored for sensors.
    pub enable_hit_events: bool,
    /// Enable pre-solve contact events. Only dynamic; ignored for sensors.
    pub enable_pre_solve_events: bool,
    /// When true, static shapes scan for contacts on the next step.
    pub invoke_contact_creation: bool,
    /// Should the body update mass properties when this shape is created.
    pub update_body_mass: bool,
    /// Used internally to detect a valid definition. DO NOT SET.
    pub internal_value: i32,
}

/// Use this to initialize your shape definition. (b3DefaultShapeDef)
pub fn default_shape_def() -> ShapeDef {
    let length_units = get_length_units_per_meter();
    ShapeDef {
        name: String::new(),
        user_data: 0,
        materials: Vec::new(),
        base_material: default_surface_material(),
        // density of water
        density: 1000.0 / (length_units * length_units * length_units),
        explosion_scale: 1.0,
        filter: default_filter(),
        enable_custom_filtering: false,
        is_sensor: false,
        enable_sensor_events: false,
        enable_contact_events: false,
        enable_hit_events: false,
        enable_pre_solve_events: false,
        invoke_contact_creation: true,
        update_body_mass: true,
        internal_value: SECRET_COOKIE,
    }
}

impl Default for ShapeDef {
    fn default() -> Self {
        default_shape_def()
    }
}
