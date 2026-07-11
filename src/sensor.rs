//! Port of box3d-cpp-reference/src/sensor.c + sensor.h.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::bitset::BitSet;
use crate::body::get_body_transform;
use crate::compound::overlap_compound;
use crate::constants::MAX_SHAPE_CAST_POINTS;
use crate::core::NULL_INDEX;
use crate::distance::{make_proxy, ShapeProxy};
use crate::events::{SensorBeginTouchEvent, SensorEndTouchEvent};
use crate::geometry::{overlap_capsule, overlap_sphere, ShapeType};
use crate::height_field::overlap_height_field;
use crate::hull::{get_hull_points, overlap_hull};
use crate::id::ShapeId;
use crate::math_functions::{
    inv_mul_transforms, min_int, to_relative_transform, transform_point, Transform, POS_ZERO,
    TRANSFORM_IDENTITY,
};
use crate::mesh::{overlap_mesh, Mesh};
use crate::shape::{should_shapes_collide, shape_flags, Shape, ShapeGeometry};
use crate::solver_set::DISABLED_SET;
use crate::types::BodyType;
use crate::world::World;

/// Used to track shapes that hit sensors using time of impact. (b3SensorHit)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorHit {
    pub sensor_id: i32,
    pub visitor_id: i32,
}

impl Default for SensorHit {
    fn default() -> Self {
        SensorHit {
            sensor_id: NULL_INDEX,
            visitor_id: NULL_INDEX,
        }
    }
}

/// (b3Visitor)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Visitor {
    pub shape_id: i32,
    pub generation: u16,
}

impl Default for Visitor {
    fn default() -> Self {
        Visitor {
            shape_id: NULL_INDEX,
            generation: 0,
        }
    }
}

/// Sensors are shapes that live in the broad-phase but never have contacts.
/// (b3Sensor)
#[derive(Debug, Clone, Default)]
pub struct Sensor {
    pub hits: Vec<Visitor>,
    pub overlaps1: Vec<Visitor>,
    pub overlaps2: Vec<Visitor>,
    pub shape_id: i32,
}

impl Sensor {
    pub fn new(shape_id: i32) -> Sensor {
        Sensor {
            hits: Vec::with_capacity(4),
            overlaps1: Vec::with_capacity(16),
            overlaps2: Vec::with_capacity(16),
            shape_id,
        }
    }
}

/// (b3SensorTaskContext)
#[derive(Debug, Clone, Default)]
pub struct SensorTaskContext {
    pub event_bits: BitSet,
}

/// Shape proxy for a convex visitor. (b3MakeShapeProxy)
fn make_shape_proxy(shape: &Shape) -> ShapeProxy {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            make_proxy(&[capsule.center1, capsule.center2], capsule.radius)
        }
        ShapeGeometry::Sphere(sphere) => make_proxy(&[sphere.center], sphere.radius),
        ShapeGeometry::Hull(hull) => {
            let points = get_hull_points(hull);
            make_proxy(points, 0.0)
        }
        _ => {
            debug_assert!(false, "make_shape_proxy: unsupported visitor shape type");
            ShapeProxy::default()
        }
    }
}

/// Overlap test with the visitor expressed in the sensor's local frame.
/// (static b3OverlapSensor)
fn overlap_sensor(
    sensor_shape: &Shape,
    sensor_transform: Transform,
    visitor_shape: &Shape,
    visitor_transform: Transform,
) -> bool {
    let proxy = make_shape_proxy(visitor_shape);
    let relative_transform = inv_mul_transforms(sensor_transform, visitor_transform);

    let mut local_proxy = ShapeProxy {
        count: min_int(proxy.count, MAX_SHAPE_CAST_POINTS as i32),
        radius: proxy.radius,
        ..ShapeProxy::default()
    };
    for i in 0..local_proxy.count as usize {
        local_proxy.points[i] = transform_point(relative_transform, proxy.points[i]);
    }

    match &sensor_shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            overlap_capsule(capsule, TRANSFORM_IDENTITY, &local_proxy)
        }
        ShapeGeometry::Compound(compound) => {
            overlap_compound(compound, TRANSFORM_IDENTITY, &local_proxy)
        }
        ShapeGeometry::HeightField(hf) => {
            overlap_height_field(hf, TRANSFORM_IDENTITY, &local_proxy)
        }
        ShapeGeometry::Hull(hull) => overlap_hull(hull, TRANSFORM_IDENTITY, &local_proxy),
        ShapeGeometry::Mesh { data, scale } => {
            let mesh = Mesh::new(data, *scale);
            overlap_mesh(&mesh, TRANSFORM_IDENTITY, &local_proxy)
        }
        ShapeGeometry::Sphere(sphere) => overlap_sphere(sphere, TRANSFORM_IDENTITY, &local_proxy),
    }
}

/// Broad-phase visitor filter + exact overlap for one candidate.
fn sensor_accepts_visitor(
    world: &World,
    sensor_shape: &Shape,
    sensor_transform: Transform,
    visitor_shape_id: i32,
) -> bool {
    let sensor_shape_id = sensor_shape.id;
    if visitor_shape_id == sensor_shape_id {
        return false;
    }

    let other_shape = &world.shapes[visitor_shape_id as usize];
    let other_type = other_shape.shape_type();
    let sensor_type = sensor_shape.shape_type();
    if (other_type == ShapeType::Mesh || other_type == ShapeType::Height)
        && (sensor_type == ShapeType::Mesh || sensor_type == ShapeType::Height)
    {
        return false;
    }

    if (other_shape.flags & shape_flags::ENABLE_SENSOR_EVENTS) == 0 {
        return false;
    }
    if other_shape.body_id == sensor_shape.body_id {
        return false;
    }
    if !should_shapes_collide(sensor_shape.filter, other_shape.filter) {
        return false;
    }

    if (sensor_shape.flags & shape_flags::ENABLE_CUSTOM_FILTERING) != 0
        || (other_shape.flags & shape_flags::ENABLE_CUSTOM_FILTERING) != 0
    {
        if let Some(custom_filter_fcn) = world.custom_filter_fcn {
            let id_a = ShapeId {
                index1: sensor_shape_id + 1,
                world0: world.world_id,
                generation: sensor_shape.generation,
            };
            let id_b = ShapeId {
                index1: visitor_shape_id + 1,
                world0: world.world_id,
                generation: other_shape.generation,
            };
            if !custom_filter_fcn(id_a, id_b, world.custom_filter_context) {
                return false;
            }
        }
    }

    let other_transform =
        to_relative_transform(get_body_transform(world, other_shape.body_id), POS_ZERO);
    overlap_sensor(sensor_shape, sensor_transform, other_shape, other_transform)
}

/// Serial sensor overlap pass for `[start, end)`. (static b3SensorTask)
fn sensor_task(world: &mut World, start_index: usize, end_index: usize) {
    debug_assert!(start_index < end_index);

    for sensor_index in start_index..end_index {
        {
            let sensor = &mut world.sensors[sensor_index];
            std::mem::swap(&mut sensor.overlaps1, &mut sensor.overlaps2);
            sensor.overlaps2.clear();
            sensor.overlaps2.append(&mut sensor.hits);
        }

        let shape_id = world.sensors[sensor_index].shape_id;
        let body_id = world.shapes[shape_id as usize].body_id;
        let body_set_index = world.bodies[body_id as usize].set_index;
        let sensor_events_enabled =
            (world.shapes[shape_id as usize].flags & shape_flags::ENABLE_SENSOR_EVENTS) != 0;

        if body_set_index == DISABLED_SET || !sensor_events_enabled {
            if !world.sensors[sensor_index].overlaps1.is_empty() {
                world.sensor_task_contexts[0]
                    .event_bits
                    .set_bit(sensor_index as u32);
            }
            continue;
        }

        let transform = to_relative_transform(get_body_transform(world, body_id), POS_ZERO);
        debug_assert!(world.shapes[shape_id as usize].sensor_index == sensor_index as i32);

        let query_bounds = world.shapes[shape_id as usize].aabb;
        let mask_bits = world.shapes[shape_id as usize].filter.mask_bits;

        let mut candidates = Vec::new();
        for tree_index in 0..BodyType::Dynamic as usize + 1 {
            world.broad_phase.trees[tree_index].query(
                query_bounds,
                mask_bits,
                false,
                |_proxy_id, user_data| {
                    candidates.push(user_data as i32);
                    true
                },
            );
        }

        for visitor_shape_id in candidates {
            let accepted = {
                let sensor_shape = &world.shapes[shape_id as usize];
                sensor_accepts_visitor(world, sensor_shape, transform, visitor_shape_id)
            };
            if accepted {
                let generation = world.shapes[visitor_shape_id as usize].generation;
                world.sensors[sensor_index].overlaps2.push(Visitor {
                    shape_id: visitor_shape_id,
                    generation,
                });
            }
        }

        world.sensors[sensor_index]
            .overlaps2
            .sort_unstable_by(|a, b| a.shape_id.cmp(&b.shape_id));

        {
            let overlaps = &mut world.sensors[sensor_index].overlaps2;
            let mut unique_count = 0usize;
            for i in 0..overlaps.len() {
                if unique_count == 0 || overlaps[i].shape_id != overlaps[unique_count - 1].shape_id {
                    overlaps[unique_count] = overlaps[i];
                    unique_count += 1;
                }
            }
            overlaps.truncate(unique_count);
        }

        let count1 = world.sensors[sensor_index].overlaps1.len();
        let count2 = world.sensors[sensor_index].overlaps2.len();
        if count1 != count2 {
            world.sensor_task_contexts[0]
                .event_bits
                .set_bit(sensor_index as u32);
        } else {
            let changed = (0..count1).any(|i| {
                let s1 = world.sensors[sensor_index].overlaps1[i];
                let s2 = world.sensors[sensor_index].overlaps2[i];
                s1.shape_id != s2.shape_id || s1.generation != s2.generation
            });
            if changed {
                world.sensor_task_contexts[0]
                    .event_bits
                    .set_bit(sensor_index as u32);
            }
        }
    }
}

fn publish_sensor_events(world: &mut World) {
    let bits: Vec<u64> = {
        let bit_set = &world.sensor_task_contexts[0].event_bits;
        (0..bit_set.block_count())
            .map(|k| bit_set.block(k))
            .collect()
    };
    let world_id = world.world_id;
    let end_index = world.end_event_array_index as usize;

    for (k, mut word) in bits.into_iter().enumerate() {
        while word != 0 {
            let ctz = word.trailing_zeros();
            let sensor_index = (64 * k as u32 + ctz) as usize;

            let shape_id = world.sensors[sensor_index].shape_id;
            let sensor_generation = world.shapes[shape_id as usize].generation;
            let sensor_id = ShapeId {
                index1: shape_id + 1,
                world0: world_id,
                generation: sensor_generation,
            };

            let count1 = world.sensors[sensor_index].overlaps1.len();
            let count2 = world.sensors[sensor_index].overlaps2.len();

            let mut index1 = 0usize;
            let mut index2 = 0usize;
            while index1 < count1 && index2 < count2 {
                let r1 = world.sensors[sensor_index].overlaps1[index1];
                let r2 = world.sensors[sensor_index].overlaps2[index2];
                if r1.shape_id == r2.shape_id {
                    if r1.generation < r2.generation {
                        world.sensor_end_events[end_index].push(SensorEndTouchEvent {
                            sensor_shape_id: sensor_id,
                            visitor_shape_id: ShapeId {
                                index1: r1.shape_id + 1,
                                world0: world_id,
                                generation: r1.generation,
                            },
                        });
                        index1 += 1;
                    } else if r1.generation > r2.generation {
                        world.sensor_begin_events.push(SensorBeginTouchEvent {
                            sensor_shape_id: sensor_id,
                            visitor_shape_id: ShapeId {
                                index1: r2.shape_id + 1,
                                world0: world_id,
                                generation: r2.generation,
                            },
                        });
                        index2 += 1;
                    } else {
                        index1 += 1;
                        index2 += 1;
                    }
                } else if r1.shape_id < r2.shape_id {
                    world.sensor_end_events[end_index].push(SensorEndTouchEvent {
                        sensor_shape_id: sensor_id,
                        visitor_shape_id: ShapeId {
                            index1: r1.shape_id + 1,
                            world0: world_id,
                            generation: r1.generation,
                        },
                    });
                    index1 += 1;
                } else {
                    world.sensor_begin_events.push(SensorBeginTouchEvent {
                        sensor_shape_id: sensor_id,
                        visitor_shape_id: ShapeId {
                            index1: r2.shape_id + 1,
                            world0: world_id,
                            generation: r2.generation,
                        },
                    });
                    index2 += 1;
                }
            }

            while index1 < count1 {
                let r1 = world.sensors[sensor_index].overlaps1[index1];
                world.sensor_end_events[end_index].push(SensorEndTouchEvent {
                    sensor_shape_id: sensor_id,
                    visitor_shape_id: ShapeId {
                        index1: r1.shape_id + 1,
                        world0: world_id,
                        generation: r1.generation,
                    },
                });
                index1 += 1;
            }

            while index2 < count2 {
                let r2 = world.sensors[sensor_index].overlaps2[index2];
                world.sensor_begin_events.push(SensorBeginTouchEvent {
                    sensor_shape_id: sensor_id,
                    visitor_shape_id: ShapeId {
                        index1: r2.shape_id + 1,
                        world0: world_id,
                        generation: r2.generation,
                    },
                });
                index2 += 1;
            }

            word &= word - 1;
        }
    }
}

/// Query all sensors for overlaps and emit begin/end events. (b3OverlapSensors)
pub fn overlap_sensors(world: &mut World) {
    let sensor_count = world.sensors.len();
    if sensor_count == 0 {
        return;
    }

    debug_assert!(!world.sensor_task_contexts.is_empty());
    world.sensor_task_contexts[0]
        .event_bits
        .set_bit_count_and_clear(sensor_count as u32);

    sensor_task(world, 0, sensor_count);
    publish_sensor_events(world);
}

/// Flush pending end events and remove a sensor from the dense array.
/// (b3DestroySensor)
pub fn destroy_sensor(world: &mut World, sensor_shape_id: i32) {
    let sensor_index = world.shapes[sensor_shape_id as usize].sensor_index;
    debug_assert!(sensor_index != NULL_INDEX);

    let world_id = world.world_id;
    let generation = world.shapes[sensor_shape_id as usize].generation;
    let end_index = world.end_event_array_index as usize;

    let overlaps: Vec<_> = world.sensors[sensor_index as usize].overlaps2.clone();
    for visitor in overlaps {
        world.sensor_end_events[end_index].push(SensorEndTouchEvent {
            sensor_shape_id: ShapeId {
                index1: sensor_shape_id + 1,
                world0: world_id,
                generation,
            },
            visitor_shape_id: ShapeId {
                index1: visitor.shape_id + 1,
                world0: world_id,
                generation: visitor.generation,
            },
        });
    }

    world.sensors[sensor_index as usize].hits.clear();
    world.sensors[sensor_index as usize].overlaps1.clear();
    world.sensors[sensor_index as usize].overlaps2.clear();

    let last = world.sensors.len() as i32 - 1;
    world.sensors.swap_remove(sensor_index as usize);
    if sensor_index < last {
        let moved_shape_id = world.sensors[sensor_index as usize].shape_id;
        world.shapes[moved_shape_id as usize].sensor_index = sensor_index;
    }
}
