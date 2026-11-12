//! C `DynamicMover::Draw` (`samples/dynamic_mover.cpp:567`): the pogo probes of the
//! library's [`DynamicMover`], drawn into the Character page's debug buffers. The mover
//! itself lives in `box3d_rust::dynamic_mover`; drawing stays with the demo.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

#![allow(clippy::unnecessary_cast)] // Pos is f64 under the double-precision feature

use crate::character_demo::colors;
use box3d_rust::dynamic_mover::DynamicMover;
use box3d_rust::math_functions::{offset_pos, Pos};

/// Gray misses, the selected probe green, the other hits yellow, each hit capped with a
/// 6 px point. Appends `[x0,y0,z0,x1,y1,z1,color]` segments and `[x,y,z,color,size]`
/// points.
pub(crate) fn draw_mover_probes(mover: &DynamicMover, segs: &mut Vec<f32>, pts: &mut Vec<f32>) {
    for i in 0..mover.probe_count {
        let probe = &mover.probes[i];
        let end = offset_pos(probe.origin, probe.fraction * probe.translation);

        if !probe.hit {
            push_line(segs, probe.origin, end, colors::GRAY);
            continue;
        }

        let color = if i == mover.selected_probe {
            colors::GREEN
        } else {
            colors::YELLOW
        };
        push_line(segs, probe.origin, end, color);
        pts.extend_from_slice(&[end.x as f32, end.y as f32, end.z as f32, color as f32, 6.0]);
    }
}

fn push_line(segs: &mut Vec<f32>, a: Pos, b: Pos, color: u32) {
    segs.extend_from_slice(&[
        a.x as f32,
        a.y as f32,
        a.z as f32,
        b.x as f32,
        b.y as f32,
        b.z as f32,
        color as f32,
    ]);
}
