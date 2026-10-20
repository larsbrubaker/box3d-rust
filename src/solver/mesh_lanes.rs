//! Mesh contact lane grouping from b3Solve (solver.c).
//!
//! Graph-color mesh contacts are solved four at a time by the wide contact kernels. Each group
//! of four lanes is padded to the largest manifold count in the group, so the solver sorts the
//! contacts of each color by manifold count first.
//!
//! SPDX-FileCopyrightText: 2025 Erin Catto
//! SPDX-License-Identifier: MIT

use crate::contact::ContactSpec;
use crate::math_functions::max_int;

/// The lane order and group manifold starts of one color's mesh contacts.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct MeshLanes {
    /// Contact index (into the color's mesh contact array) of each lane slot.
    pub order: Vec<i32>,
    /// Manifold slot start of each group of lanes plus a sentinel. Relative to the color; C
    /// keeps one array across all colors, and only the differences are used.
    pub manifold_starts: Vec<i32>,
}

/// Sort mesh contacts by manifold count so that contacts with more manifolds are solved
/// together keeping the SIMD lanes fuller and less ragged. (inline block of b3Solve)
///
/// `color_specs` holds the mesh contact arrays of the active colors. The sort is a stable
/// counting sort by descending manifold count. `simd_shift` is 2 for width 4.
pub fn compute_mesh_lanes(color_specs: &[&[ContactSpec]], simd_shift: i32) -> Vec<MeshLanes> {
    let mut max_manifold_count = 0;
    for specs in color_specs {
        for spec in specs.iter() {
            max_manifold_count = max_int(max_manifold_count, spec.manifold_count as i32);
        }
    }

    let mut bucket_starts = vec![0i32; (max_manifold_count + 1) as usize];

    let mut result = Vec::with_capacity(color_specs.len());
    for specs in color_specs {
        let color_contact_count = specs.len();
        if color_contact_count == 0 {
            result.push(MeshLanes::default());
            continue;
        }

        let mut order = vec![0i32; color_contact_count];

        for bucket in bucket_starts.iter_mut() {
            *bucket = 0;
        }

        for spec in specs.iter() {
            bucket_starts[spec.manifold_count as usize] += 1;
        }

        let mut start = 0;
        for k in (0..=max_manifold_count as usize).rev() {
            let count = bucket_starts[k];
            bucket_starts[k] = start;
            start += count;
        }

        for (j, spec) in specs.iter().enumerate() {
            let bucket = &mut bucket_starts[spec.manifold_count as usize];
            order[*bucket as usize] = j as i32;
            *bucket += 1;
        }

        let color_mesh_group_count = ((color_contact_count - 1) >> simd_shift) + 1;
        let mut manifold_starts = Vec::with_capacity(color_mesh_group_count + 1);
        let mut mesh_slot_count = 0;
        for g in 0..color_mesh_group_count {
            let slot_count = specs[order[g << simd_shift] as usize].manifold_count as i32;
            manifold_starts.push(mesh_slot_count);
            mesh_slot_count += slot_count;
        }
        manifold_starts.push(mesh_slot_count);

        result.push(MeshLanes {
            order,
            manifold_starts,
        });
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec(contact_id: i32, manifold_count: u16) -> ContactSpec {
        ContactSpec {
            contact_id,
            manifold_start: 0,
            manifold_count,
        }
    }

    #[test]
    fn groups_by_descending_manifold_count() {
        let color0 = [
            spec(10, 1),
            spec(11, 3),
            spec(12, 2),
            spec(13, 3),
            spec(14, 1),
            spec(15, 2),
        ];
        let color1: [ContactSpec; 0] = [];
        let color2 = [spec(20, 2)];

        let lanes = compute_mesh_lanes(&[&color0, &color1, &color2], 2);
        assert_eq!(lanes.len(), 3);

        // Stable within each manifold count, largest counts first.
        assert_eq!(lanes[0].order, vec![1, 3, 2, 5, 0, 4]);
        // Group 0 is padded to 3 manifolds, group 1 (lanes 4 and 5) to 1 manifold.
        assert_eq!(lanes[0].manifold_starts, vec![0, 3, 4]);

        assert!(lanes[1].order.is_empty());
        assert!(lanes[1].manifold_starts.is_empty());

        assert_eq!(lanes[2].order, vec![0]);
        assert_eq!(lanes[2].manifold_starts, vec![0, 2]);
    }
}
