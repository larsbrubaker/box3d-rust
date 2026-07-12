//! Shape time-of-impact dispatch, including mesh / height / compound CCD.
//!
//! Port of `b3ShapeTimeOfImpact` and helpers from
//! `box3d-cpp-reference/src/shape.c`.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::{make_shape_proxy, Shape, ShapeGeometry};
use crate::compound::{
    get_compound_child, make_compound_child_sweep, query_compound, ChildGeometry,
};
use crate::constants::{linear_slop, speculative_distance};
use crate::core::NULL_INDEX;
use crate::distance::{
    get_sweep_transform, make_proxy, time_of_impact, ShapeProxy, Sweep, ToiInput, ToiOutput,
};
use crate::geometry::{compute_swept_capsule_aabb, compute_swept_sphere_aabb, ShapeType};
use crate::height_field::query_height_field;
use crate::hull::{compute_swept_hull_aabb, get_hull_points};
use crate::math_functions::{
    aabb_transform, cross, dot, inv_transform_point, invert_transform, max_float, mul_transforms,
    normalize, rotate_vector, sub, transform_point, Aabb, Transform, Vec3, VEC3_ZERO,
};
use crate::mesh::{query_mesh, Mesh};

/// Swept AABB of a convex shape along a sweep at time in [0, 1].
/// (b3ComputeSweptShapeAABB)
pub fn compute_swept_shape_aabb(shape: &Shape, sweep: &Sweep, time: f32) -> Aabb {
    debug_assert!((0.0..=1.0).contains(&time));
    let xf1 = Transform {
        p: sub(sweep.c1, rotate_vector(sweep.q1, sweep.local_center)),
        q: sweep.q1,
    };
    let xf2 = get_sweep_transform(sweep, time);

    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => compute_swept_capsule_aabb(capsule, xf1, xf2),
        ShapeGeometry::Hull(hull) => compute_swept_hull_aabb(hull, xf1, xf2),
        ShapeGeometry::Sphere(sphere) => compute_swept_sphere_aabb(sphere, xf1, xf2),
        _ => {
            debug_assert!(false, "compute_swept_shape_aabb is for convex shapes only");
            Aabb {
                lower_bound: xf1.p,
                upper_bound: xf1.p,
            }
        }
    }
}

/// Context for mesh / height-field triangle TOI queries. (b3MeshImpactContext)
struct MeshImpactContext {
    toi_input: ToiInput,
    toi_output: ToiOutput,
    /// Centroid of shape B in body B local space
    local_centroid_b: Vec3,
    /// Centroid of shape B at start of sweep, in mesh local space
    mesh_local_centroid_b1: Vec3,
    /// Centroid of shape B at end of sweep, in mesh local space
    mesh_local_centroid_b2: Vec3,
    fallback_radius: f32,
    is_sensor: bool,
}

/// Per-triangle callback for mesh / height TOI. (b3MeshTimeOfImpactFcn)
fn mesh_time_of_impact_fcn(a: Vec3, b: Vec3, c: Vec3, context: &mut MeshImpactContext) -> bool {
    // Early out for parallel movement
    let c1 = context.mesh_local_centroid_b1;
    let c2 = context.mesh_local_centroid_b2;

    let n = normalize(cross(sub(b, a), sub(c, a)));
    let offset1 = dot(n, sub(c1, a));
    let offset2 = dot(n, sub(c2, a));

    if offset1 < 0.0 {
        // Started behind or finished in front
        return true;
    }

    if !context.is_sensor
        && offset1 - offset2 < context.fallback_radius
        && offset2 > context.fallback_radius
    {
        // Finished in front
        return true;
    }

    let triangle = [a, b, c];
    context.toi_input.proxy_a = make_proxy(&triangle, 0.0);

    let mut output = time_of_impact(&context.toi_input);

    // It is possible for a hit at fraction == 0

    if 0.0 < output.fraction && output.fraction < context.toi_input.max_fraction {
        context.toi_output = output;
        context.toi_input.max_fraction = output.fraction;
    } else if output.fraction == 0.0 {
        // Fallback to TOI of a small circle around the fast shape centroid
        let mut fallback_input = context.toi_input;
        fallback_input.proxy_b = make_proxy(
            &[context.local_centroid_b],
            context.fallback_radius + linear_slop(),
        );
        output = time_of_impact(&fallback_input);

        if 0.0 < output.fraction && output.fraction < context.toi_input.max_fraction {
            context.toi_output = output;
            context.toi_input.max_fraction = output.fraction;
            context.toi_output.used_fallback = true;
        }
    }

    // Continue the query
    true
}

/// Context for compound child TOI queries. (b3CompoundImpactContext)
struct CompoundImpactContext {
    toi_input: ToiInput,
    toi_output: ToiOutput,
    compound_transform: Transform,
    /// Bounds local to compound
    local_sweep_bounds_b: Aabb,
    /// Centroid of shape B in body B local space
    local_centroid_b: Vec3,
    fallback_radius: f32,
}

/// Per-child callback for compound TOI. (b3CompoundTimeOfImpactFcn)
fn compound_time_of_impact_fcn(
    compound: &crate::compound::CompoundData,
    child_index: i32,
    context: &mut CompoundImpactContext,
) -> bool {
    let child = get_compound_child(compound, child_index);

    context.toi_input.sweep_a =
        make_compound_child_sweep(context.compound_transform, child.transform);

    let output = match child.geometry {
        ChildGeometry::Capsule(capsule) => {
            context.toi_input.proxy_a =
                make_proxy(&[capsule.center1, capsule.center2], capsule.radius);
            time_of_impact(&context.toi_input)
        }
        ChildGeometry::Hull(hull) => {
            context.toi_input.proxy_a = make_proxy(get_hull_points(hull), 0.0);
            time_of_impact(&context.toi_input)
        }
        ChildGeometry::Mesh(mesh) => {
            let mut mesh_context = MeshImpactContext {
                toi_input: context.toi_input,
                toi_output: ToiOutput::default(),
                local_centroid_b: context.local_centroid_b,
                mesh_local_centroid_b1: VEC3_ZERO,
                mesh_local_centroid_b2: VEC3_ZERO,
                fallback_radius: context.fallback_radius,
                is_sensor: false,
            };

            let mesh_world_transform = mul_transforms(context.compound_transform, child.transform);

            let sweep_b = &context.toi_input.sweep_b;
            let xf_b1 = Transform {
                p: sub(sweep_b.c1, rotate_vector(sweep_b.q1, sweep_b.local_center)),
                q: sweep_b.q1,
            };
            let xf_b2 = Transform {
                p: sub(sweep_b.c2, rotate_vector(sweep_b.q2, sweep_b.local_center)),
                q: sweep_b.q2,
            };

            mesh_context.mesh_local_centroid_b1 = inv_transform_point(
                mesh_world_transform,
                transform_point(xf_b1, mesh_context.local_centroid_b),
            );
            mesh_context.mesh_local_centroid_b2 = inv_transform_point(
                mesh_world_transform,
                transform_point(xf_b2, mesh_context.local_centroid_b),
            );

            // Bounds local to mesh
            let local_bounds = aabb_transform(
                invert_transform(child.transform),
                context.local_sweep_bounds_b,
            );

            query_mesh(&mesh, local_bounds, |a, b, c, _triangle_index| {
                mesh_time_of_impact_fcn(a, b, c, &mut mesh_context)
            });

            mesh_context.toi_output
        }
        ChildGeometry::Sphere(sphere) => {
            context.toi_input.proxy_a = make_proxy(&[sphere.center], sphere.radius);
            time_of_impact(&context.toi_input)
        }
    };

    if 0.0 < output.fraction && output.fraction < context.toi_input.max_fraction {
        context.toi_output = output;
        context.toi_input.max_fraction = output.fraction;
    }

    // Clear this to be safe (C zeroes proxyA)
    context.toi_input.proxy_a = ShapeProxy::default();

    // Continue the query
    true
}

/// Time of impact between two shapes along sweeps.
/// Convex vs convex uses GJK TOI; mesh/height/compound use triangle/child queries.
/// (b3ShapeTimeOfImpact)
pub fn shape_time_of_impact(
    shape_a: &Shape,
    shape_b: &Shape,
    sweep_a: &Sweep,
    sweep_b: &Sweep,
    max_fraction: f32,
) -> ToiOutput {
    use super::{compute_shape_extent, get_shape_centroid};

    let is_sensor = shape_a.sensor_index != NULL_INDEX;
    let type_a = shape_a.shape_type();

    if type_a == ShapeType::Compound {
        let ShapeGeometry::Compound(compound) = &shape_a.geometry else {
            unreachable!();
        };

        let local_centroid_b = get_shape_centroid(shape_b);
        let extents = compute_shape_extent(shape_b, local_centroid_b);
        let fallback_radius = max_float(0.75 * extents.min_extent, speculative_distance());

        let compound_transform = Transform {
            p: sweep_a.c1,
            q: sweep_a.q1,
        };

        // Swept bounds of shapeB, then local to compound
        let bounds = compute_swept_shape_aabb(shape_b, sweep_b, max_fraction);
        let local_bounds = aabb_transform(invert_transform(compound_transform), bounds);

        let mut context = CompoundImpactContext {
            toi_input: ToiInput {
                proxy_a: ShapeProxy::default(),
                proxy_b: make_shape_proxy(shape_b),
                sweep_a: Sweep::default(),
                sweep_b: *sweep_b,
                max_fraction,
            },
            toi_output: ToiOutput::default(),
            compound_transform,
            local_sweep_bounds_b: local_bounds,
            local_centroid_b,
            fallback_radius,
        };

        query_compound(compound, local_bounds, |compound, child_index| {
            compound_time_of_impact_fcn(compound, child_index, &mut context)
        });

        return context.toi_output;
    }

    if type_a == ShapeType::Height || type_a == ShapeType::Mesh {
        // Note: assuming mesh is static
        let local_centroid_b = get_shape_centroid(shape_b);

        // Assume mesh is static
        let xf_a = Transform {
            p: sub(sweep_a.c1, rotate_vector(sweep_a.q1, sweep_a.local_center)),
            q: sweep_a.q1,
        };
        let xf_b1 = Transform {
            p: sub(sweep_b.c1, rotate_vector(sweep_b.q1, sweep_b.local_center)),
            q: sweep_b.q1,
        };
        let xf_b2 = Transform {
            p: sub(sweep_b.c2, rotate_vector(sweep_b.q2, sweep_b.local_center)),
            q: sweep_b.q2,
        };

        let extents = compute_shape_extent(shape_b, local_centroid_b);
        let fallback_radius = max_float(0.5 * extents.min_extent, linear_slop());

        // Swept bounds of shapeB, then local to mesh
        let bounds = compute_swept_shape_aabb(shape_b, sweep_b, max_fraction);
        let local_bounds = aabb_transform(invert_transform(xf_a), bounds);

        let mut context = MeshImpactContext {
            toi_input: ToiInput {
                proxy_a: ShapeProxy {
                    count: 3,
                    ..ShapeProxy::default()
                },
                proxy_b: make_shape_proxy(shape_b),
                sweep_a: *sweep_a,
                sweep_b: *sweep_b,
                max_fraction,
            },
            toi_output: ToiOutput::default(),
            local_centroid_b,
            mesh_local_centroid_b1: inv_transform_point(
                xf_a,
                transform_point(xf_b1, local_centroid_b),
            ),
            mesh_local_centroid_b2: inv_transform_point(
                xf_a,
                transform_point(xf_b2, local_centroid_b),
            ),
            fallback_radius,
            is_sensor,
        };

        match &shape_a.geometry {
            ShapeGeometry::Mesh { data, scale } => {
                let mesh = Mesh {
                    data,
                    scale: *scale,
                };
                query_mesh(&mesh, local_bounds, |a, b, c, _triangle_index| {
                    mesh_time_of_impact_fcn(a, b, c, &mut context)
                });
            }
            ShapeGeometry::HeightField(hf) => {
                query_height_field(hf, local_bounds, |a, b, c, _triangle_index| {
                    mesh_time_of_impact_fcn(a, b, c, &mut context)
                });
            }
            _ => unreachable!(),
        }

        return context.toi_output;
    }

    debug_assert!(
        shape_b.shape_type() != ShapeType::Compound
            && shape_b.shape_type() != ShapeType::Mesh
            && shape_b.shape_type() != ShapeType::Height
    );

    let input = ToiInput {
        proxy_a: make_shape_proxy(shape_a),
        proxy_b: make_shape_proxy(shape_b),
        sweep_a: *sweep_a,
        sweep_b: *sweep_b,
        max_fraction,
    };
    time_of_impact(&input)
}
