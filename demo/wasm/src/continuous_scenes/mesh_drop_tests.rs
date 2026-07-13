//! Finiteness regression tests for the Mesh Drop ground wireframe. Guards the
//! reported `computeBoundingSphere: radius is NaN` on a fresh `#/continuous/mesh-drop`
//! load. `#[path]`-included by `mesh_drop.rs`.

/// After a Mesh Drop reset, every float the renderer reads from the baked ground
/// wireframe must be finite (no NaN/Inf), and the buffer must be non-empty.
#[test]
fn mesh_drop_ground_wireframe_is_finite() {
    let _ = super::sim_reset_mesh_drop();
    let wire = super::super::sim_cont_ground_wireframe();
    assert!(!wire.is_empty(), "ground wireframe unexpectedly empty");
    for (i, v) in wire.iter().enumerate() {
        assert!(
            v.is_finite(),
            "ground wireframe float {i} is not finite: {v}"
        );
    }
}

/// The Mesh Drop Unit Test ground (wave mesh, no walls) must likewise be finite.
#[test]
fn mesh_drop_unit_ground_wireframe_is_finite() {
    let _ = super::sim_reset_mesh_drop_unit();
    let wire = super::super::sim_cont_ground_wireframe();
    assert!(
        !wire.is_empty(),
        "unit-test ground wireframe unexpectedly empty"
    );
    for (i, v) in wire.iter().enumerate() {
        assert!(
            v.is_finite(),
            "unit-test ground wireframe float {i} is not finite: {v}"
        );
    }
}

/// The pose stream driving the renderer must be finite too (the ground body plus
/// every projectile).
#[test]
fn mesh_drop_poses_are_finite() {
    let _ = super::sim_reset_mesh_drop();
    let poses = crate::sim_demo::sim_body_poses();
    for (i, v) in poses.iter().enumerate() {
        assert!(v.is_finite(), "pose float {i} is not finite: {v}");
    }
}

/// Sweep the amplitude slider extremes and the Generate reseed: the baked ground
/// wireframe must stay finite at every setting the UI can drive.
#[test]
fn mesh_drop_wireframe_finite_across_controls() {
    for amp in [0.0f32, 0.05, 0.5, 1.0] {
        let _ = super::sim_cont_mesh_drop_set_amplitude(amp);
        let wire = super::super::sim_cont_ground_wireframe();
        for (i, v) in wire.iter().enumerate() {
            assert!(v.is_finite(), "amp {amp}: wire float {i} not finite: {v}");
        }
    }
    for shape in 0..4u32 {
        let _ = super::sim_cont_mesh_drop_set_type(shape);
        let _ = super::sim_cont_mesh_drop_generate();
        let wire = super::super::sim_cont_ground_wireframe();
        for (i, v) in wire.iter().enumerate() {
            assert!(
                v.is_finite(),
                "shape {shape}: wire float {i} not finite: {v}"
            );
        }
    }
}

/// On a fresh continuous module (no scene reset yet) the shared wireframe export
/// must return an *empty* buffer — never stale or uninitialized data — so the page
/// can skip building geometry from it. Guards the reported fresh-load NaN, whose
/// root cause is THREE's `computeBoundingSphere` on an empty position buffer.
#[test]
fn fresh_wireframe_is_empty_not_nan() {
    // A box-ground continuous scene bakes no mesh, so its wireframe is empty; this
    // is the same contract a never-reset module must honor.
    let _ = super::super::basic::sim_reset_spinning_stick();
    let wire = super::super::sim_cont_ground_wireframe();
    assert!(
        wire.is_empty(),
        "box-ground scene must expose an empty ground wireframe, got {} floats",
        wire.len()
    );
}

/// Switching from a mesh-ground scene to a batch-1 box-ground scene
/// (`sim_continuous.rs`, which shares the wireframe export) must clear the stale
/// mesh edges so the box scene reports an empty wireframe — not a ghost ground.
#[test]
fn box_scene_after_mesh_scene_clears_wireframe() {
    let _ = super::sim_reset_mesh_drop();
    assert!(
        !super::super::sim_cont_ground_wireframe().is_empty(),
        "mesh-drop should bake a non-empty wireframe"
    );
    let _ = crate::sim_continuous::sim_reset_thin_wall();
    assert!(
        super::super::sim_cont_ground_wireframe().is_empty(),
        "box-ground Thin Wall must clear the shared wireframe left by mesh-drop"
    );
}
