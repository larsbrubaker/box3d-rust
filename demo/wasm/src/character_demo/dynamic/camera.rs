//! The slice of the C samples host camera + `Sample` input plumbing the Dynamic Mover
//! sample depends on, ported so the mover input, the launcher aim, and the follow-cam
//! boom run in Rust with the C `f32` math:
//!
//! - [`SampleCamera::update_transform`] — `RebuildBasisAndView` (host/camera.cpp:43):
//!   the world-space basis and eye from yaw / pitch / pivot / radius.
//! - [`get_mover_input`] — `Sample::GetMoverInput` (sample.cpp:2325, new at 3362633):
//!   camera-relative WASD throttle, Space jump, Shift sprint, Ctrl walk, active only in
//!   third person so the free camera keeps its keys.
//!
//! The browser page owns the live yaw / pitch (pointer-lock mouse look, the C
//! `Sample::MouseMove` sensitivities) and feeds them in each frame; the page then
//! renders from the pivot / radius [`super::DynamicMoverSample::follow_mover`] leaves
//! here, exactly like the C sample places `m_camera` before the base step draws.
//!
//! SPDX-FileCopyrightText: 2026 Erin Catto
//! SPDX-License-Identifier: MIT

use box3d_rust::math_functions::{
    cross, dot, normalize, offset_pos, Pos, Vec2, Vec3, POS_ZERO, VEC2_ZERO,
};

/// Raw key-down state the page feeds each frame (C `IsKeyDown` table).
#[derive(Clone, Copy, Default)]
pub(crate) struct KeysDown {
    pub w: bool,
    pub a: bool,
    pub s: bool,
    pub d: bool,
    pub space: bool,
    /// `KEY_LEFT_SHIFT || KEY_RIGHT_SHIFT`.
    pub shift: bool,
    /// `KEY_LEFT_CONTROL || KEY_RIGHT_CONTROL`.
    pub control: bool,
}

/// C `MoverInput` (sample.h:25) — character input relative to the camera. Throttle x is
/// forward and y is right.
#[derive(Clone, Copy)]
pub(crate) struct MoverInput {
    pub forward: Vec3,
    pub right: Vec3,
    pub throttle: Vec2,
    pub jump: bool,
    pub sprint: bool,
    pub walk: bool,
}

/// The `Camera` members the sample reads and writes (host/camera.h): spherical state
/// plus the cached basis / eye `RebuildBasisAndView` derives from it.
#[derive(Clone, Copy)]
pub(crate) struct SampleCamera {
    pub pivot: Pos,
    pub yaw: f32,
    pub pitch: f32,
    pub radius: f32,
    pub third_person: bool,
    pub world_eye: Pos,
    pub right: Vec3,
    pub up: Vec3,
    /// The "+view-Z" direction (pivot → camera). The look direction is `-forward`.
    pub forward: Vec3,
}

/// Camera.cpp `ForwardFromAngles`: yaw/pitch → the "+view-Z" direction (pivot →
/// camera). The actual looking direction is `-ForwardFromAngles`.
fn forward_from_angles(yaw: f32, pitch: f32) -> Vec3 {
    let cp = pitch.cos();
    Vec3 {
        x: yaw.sin() * cp,
        y: pitch.sin(),
        z: yaw.cos() * cp,
    }
}

impl SampleCamera {
    /// C `Camera::SetView( yawDegrees, pitchDegrees, radius, pivot )` with the camera
    /// constants (camera.cpp: `PITCH_LIM = HALF_PI - 0.01`, radius clamp at
    /// `MIN_DIST = 0.1`).
    pub(crate) fn new_view(yaw_degrees: f32, pitch_degrees: f32, radius: f32, pivot: Pos) -> Self {
        const DEG_TO_RAD: f32 = box3d_rust::math_functions::PI / 180.0;
        const PITCH_LIM: f32 = box3d_rust::math_functions::PI * 0.5 - 0.01;
        let mut camera = SampleCamera {
            pivot,
            yaw: yaw_degrees * DEG_TO_RAD,
            pitch: (pitch_degrees * DEG_TO_RAD).clamp(-PITCH_LIM, PITCH_LIM),
            radius: radius.max(0.1),
            third_person: false,
            world_eye: POS_ZERO,
            right: Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            up: Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            forward: Vec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
        };
        camera.update_transform();
        camera
    }

    /// C `Camera::UpdateTransform` → `RebuildBasisAndView` (camera.cpp:43): refresh the
    /// world-space basis and the eye from yaw / pitch / pivot / radius.
    pub(crate) fn update_transform(&mut self) {
        let forward = forward_from_angles(self.yaw, self.pitch);
        let world_up = Vec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        };
        // up = worldUp re-orthogonalized against forward. right = up x forward.
        // Matches Box3D's UpdateTransform; produces a right-handed view basis.
        let up_unnorm = world_up - dot(world_up, forward) * forward;
        let up = normalize(up_unnorm);
        let right = normalize(cross(up, forward));

        self.world_eye = offset_pos(self.pivot, self.radius * forward);
        self.right = right;
        self.up = up;
        self.forward = forward;
    }

    /// C `Camera::GetForward` — the "+view-Z" (pivot → eye) direction.
    pub(crate) fn get_forward(&self) -> Vec3 {
        self.forward
    }

    /// C `Camera::GetRight`.
    pub(crate) fn get_right(&self) -> Vec3 {
        self.right
    }
}

/// C `Sample::GetMoverInput` (sample.cpp:2325): WASD, space, shift and ctrl, active only
/// in third person so the free camera keeps its keys.
pub(crate) fn get_mover_input(camera: &SampleCamera, keys: &KeysDown) -> MoverInput {
    let mut input = MoverInput {
        forward: -camera.get_forward(),
        right: camera.get_right(),
        throttle: VEC2_ZERO,
        jump: false,
        sprint: false,
        walk: false,
    };

    input.forward.y = 0.0;
    input.forward = normalize(input.forward);

    if !camera.third_person {
        return input;
    }

    if keys.w {
        input.throttle.x += 1.0;
    }

    if keys.s {
        input.throttle.x -= 1.0;
    }

    if keys.a {
        input.throttle.y -= 1.0;
    }

    if keys.d {
        input.throttle.y += 1.0;
    }

    input.jump = keys.space;
    input.sprint = keys.shift;
    input.walk = keys.control;
    input
}
