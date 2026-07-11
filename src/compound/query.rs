//! Compound AABB query and character-mover collision.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use super::types::{get_compound_child, ChildGeometry, CompoundData};
use crate::dynamic_tree::DEFAULT_MASK_BITS;
use crate::geometry::{collide_mover_and_capsule, collide_mover_and_sphere, Capsule, PlaneResult};
use crate::hull::collide_mover_and_hull;
use crate::math_functions::{
    add, inv_transform_point, max, min, rotate_vector, sub, transform_point, Aabb, Vec3,
};
use crate::mesh::collide_mover_and_mesh;

/// Query an AABB against compound children. (b3QueryCompound)
///
/// The callback receives `(compound, child_index)` and returns `false` to stop.
pub fn query_compound(
    compound: &CompoundData,
    aabb: Aabb,
    mut fcn: impl FnMut(&CompoundData, i32) -> bool,
) {
    compound
        .tree
        .query(aabb, DEFAULT_MASK_BITS, false, |_proxy_id, user_data| {
            fcn(compound, user_data as i32)
        });
}

/// Collide a capsule mover against a compound. (b3CollideMoverAndCompound)
pub fn collide_mover_and_compound(
    planes: &mut [PlaneResult],
    shape: &CompoundData,
    mover: &Capsule,
) -> i32 {
    let capacity = planes.len() as i32;
    if capacity == 0 {
        return 0;
    }

    let mut plane_count = 0i32;

    let mut aabb = Aabb {
        lower_bound: min(mover.center1, mover.center2),
        upper_bound: max(mover.center1, mover.center2),
    };
    let r = Vec3 {
        x: mover.radius,
        y: mover.radius,
        z: mover.radius,
    };
    aabb.lower_bound = sub(aabb.lower_bound, r);
    aabb.upper_bound = add(aabb.upper_bound, r);

    shape
        .tree
        .query(aabb, !0u64, false, |_proxy_id, user_data| {
            let child_index = user_data as i32;
            let child = get_compound_child(shape, child_index);

            let local_mover = Capsule {
                center1: inv_transform_point(child.transform, mover.center1),
                center2: inv_transform_point(child.transform, mover.center2),
                radius: mover.radius,
            };

            let remaining = capacity - plane_count;
            debug_assert!(remaining > 0);
            let slot = &mut planes[plane_count as usize..];

            let added = match child.geometry {
                ChildGeometry::Capsule(ref capsule) => {
                    collide_mover_and_capsule(&mut slot[0], capsule, &local_mover)
                }
                ChildGeometry::Hull(hull) => {
                    collide_mover_and_hull(&mut slot[0], hull, &local_mover)
                }
                ChildGeometry::Mesh(ref mesh) => collide_mover_and_mesh(slot, mesh, &local_mover),
                ChildGeometry::Sphere(ref sphere) => {
                    collide_mover_and_sphere(&mut slot[0], sphere, &local_mover)
                }
            };

            for i in 0..added {
                let p = &mut planes[(plane_count + i) as usize];
                p.plane.normal = rotate_vector(child.transform.q, p.plane.normal);
                p.point = transform_point(child.transform, p.point);
            }

            plane_count += added;
            plane_count < capacity
        });

    plane_count
}
