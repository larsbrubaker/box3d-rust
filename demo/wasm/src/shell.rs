//! `demo_shell!` — the standard wasm export set every interactive demo category
//! shares: mouse grab (down/move/up/active), shift-click spawn, ray-delete,
//! counters, the debug-draw / debug-text overlay channels, and the four
//! world-tunable toggles (sleep, warm-starting, continuous, contact recycle).
//!
//! `wasm_bindgen` cannot be applied to generic functions, so this is a
//! `macro_rules!` that *stamps* the concrete, byte-identical export bodies for a
//! given demo module. Before this existed the same ~20 functions were copy-pasted
//! into every category (`bodies`, `shapes`, `determinism`, `world_far`, …); a bug
//! fixed in one had to be chased through all of them. Now the logic lives here
//! once and each category is a short configuration table.
//!
//! # Large-world base frame
//!
//! Every position that crosses the wasm boundary is expressed relative to a
//! per-scene `base` (a `b3Pos`): the picker ray origin is offset *into* world
//! space by `base`, and returned points are taken back with `sub_pos`, both in
//! `b3Pos` space (f64 under `double-precision`) before the `f32` truncation. The
//! debug collectors subtract the same base at emission time (see
//! [`crate::interact::with_draw_base`]). Near-origin categories pass a zero base,
//! for which every one of these operations is a bit-identical no-op.
//!
//! # Configuration
//!
//! ```ignore
//! demo_shell! {
//!     with_state: my_mod::with_state,   // fn(FnOnce(&mut State) -> R) -> R
//!     state: MyState,                    // the state type (for closure params)
//!     world: world,                      // World field
//!     bodies: bodies,                    // Vec<VisBody> render list field
//!     grab: grab,                        // interact::MouseGrab field
//!     base: |s| ZERO_POS,                // |&State| -> Pos  (scene base frame)
//!     mouse_down: my_mouse_down, mouse_move: my_mouse_move,
//!     mouse_up: my_mouse_up, mouse_active: my_mouse_active,
//!     spawn_random: my_spawn = |state, spawned| { /* -> Vec<f32> */ },
//!     delete_at_ray: my_delete = |state, index| { /* post-delete cleanup */ },
//!     counters: my_counters,
//!     debug_draw: my_debug_draw, debug_text: my_debug_text,
//! }
//! ```
//!
//! The two `= |..| {..}` hooks (`spawn_random`, `delete_at_ray`) let a category
//! customize the render-list bookkeeping — the spawn hook builds the `VisBody`
//! and the return payload (arity is category-specific); the delete hook prunes any
//! auxiliary handles (static bodies, wind ids). Both run with the shared
//! grab/ray/base plumbing already applied.
//!
//! Categories that expose the four world-tunable toggles (sleep, warm-starting,
//! continuous, contact recycle) additionally invoke [`demo_world_toggles!`] with
//! the same `with_state` / `world` config; Bodies and Shapes omit it.

/// Zero large-world base for near-origin demo categories.
pub const ZERO_POS: box3d_rust::math_functions::Pos = box3d_rust::math_functions::Pos {
    x: 0.0,
    y: 0.0,
    z: 0.0,
};

#[macro_export]
macro_rules! demo_shell {
    (
        with_state: $with:path,
        state: $state:ty,
        world: $world:ident,
        bodies: $bodies:ident,
        grab: $grab:ident,
        base: $base:expr,
        mouse_down: $mouse_down:ident,
        mouse_move: $mouse_move:ident,
        mouse_up: $mouse_up:ident,
        mouse_active: $mouse_active:ident,
        spawn_random: $spawn:ident = $spawn_fn:expr,
        delete_at_ray: $delete:ident = $delete_extra:expr,
        counters: $counters:ident,
        debug_draw: $debug_draw:ident,
        debug_text: $debug_text:ident,
    ) => {
        /// Begin a ctrl-drag grab: raycast from `origin` along the ray, and if a
        /// dynamic body is hit create the kinematic mouse body + motor joint.
        /// Returns `[hit, px, py, pz]` (base-relative mouse point), or `[0,0,0,0]`.
        #[wasm_bindgen::prelude::wasm_bindgen]
        pub fn $mouse_down(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> Vec<f32> {
            $with(|state| {
                let base_fn: &dyn Fn(&$state) -> box3d_rust::math_functions::Pos = &$base;
                let base: box3d_rust::math_functions::Pos = base_fn(&*state);
                let origin = box3d_rust::math_functions::offset_pos(
                    base,
                    box3d_rust::math_functions::Vec3 {
                        x: ox,
                        y: oy,
                        z: oz,
                    },
                );
                if state.$grab.begin(
                    &mut state.$world,
                    origin,
                    box3d_rust::math_functions::Vec3 {
                        x: tx,
                        y: ty,
                        z: tz,
                    },
                ) {
                    let rel = box3d_rust::math_functions::sub_pos(state.$grab.mouse_point, base);
                    vec![1.0, rel.x, rel.y, rel.z]
                } else {
                    vec![0.0, 0.0, 0.0, 0.0]
                }
            })
        }

        /// Update the grab target to a base-relative world point.
        #[wasm_bindgen::prelude::wasm_bindgen]
        pub fn $mouse_move(px: f32, py: f32, pz: f32) {
            $with(|state| {
                let base_fn: &dyn Fn(&$state) -> box3d_rust::math_functions::Pos = &$base;
                let base: box3d_rust::math_functions::Pos = base_fn(&*state);
                let point = box3d_rust::math_functions::offset_pos(
                    base,
                    box3d_rust::math_functions::Vec3 {
                        x: px,
                        y: py,
                        z: pz,
                    },
                );
                state.$grab.move_to(point);
            })
        }

        /// Release the grab (the body keeps its velocity → fling).
        #[wasm_bindgen::prelude::wasm_bindgen]
        pub fn $mouse_up() {
            $with(|state| state.$grab.end(&mut state.$world));
        }

        /// Whether a grab joint is currently active.
        #[wasm_bindgen::prelude::wasm_bindgen]
        pub fn $mouse_active() -> bool {
            $with(|state| state.$grab.is_active())
        }

        /// Shift-click spawn the C bullet sphere along the pick ray. The render
        /// bookkeeping + return payload are the category's `spawn_random` hook.
        #[wasm_bindgen::prelude::wasm_bindgen]
        pub fn $spawn(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> Vec<f32> {
            $with(|state| {
                let base_fn: &dyn Fn(&$state) -> box3d_rust::math_functions::Pos = &$base;
                let base: box3d_rust::math_functions::Pos = base_fn(&*state);
                let origin = box3d_rust::math_functions::offset_pos(
                    base,
                    box3d_rust::math_functions::Vec3 {
                        x: ox,
                        y: oy,
                        z: oz,
                    },
                );
                let spawned = $crate::interact::spawn_random(
                    &mut state.$world,
                    origin,
                    box3d_rust::math_functions::Vec3 {
                        x: tx,
                        y: ty,
                        z: tz,
                    },
                );
                let f: &dyn Fn(&mut $state, Option<$crate::interact::SpawnedBody>) -> Vec<f32> =
                    &$spawn_fn;
                f(state, spawned)
            })
        }

        /// Destroy the dynamic body under the pick ray, prune it from the render
        /// list, then run the category's post-delete cleanup hook. Returns 1 on a
        /// hit, 0 otherwise.
        #[wasm_bindgen::prelude::wasm_bindgen]
        pub fn $delete(ox: f32, oy: f32, oz: f32, tx: f32, ty: f32, tz: f32) -> u32 {
            $with(|state| {
                let base_fn: &dyn Fn(&$state) -> box3d_rust::math_functions::Pos = &$base;
                let base: box3d_rust::math_functions::Pos = base_fn(&*state);
                let origin = box3d_rust::math_functions::offset_pos(
                    base,
                    box3d_rust::math_functions::Vec3 {
                        x: ox,
                        y: oy,
                        z: oz,
                    },
                );
                let index = $crate::interact::delete_at_ray(
                    &mut state.$world,
                    &mut state.$grab,
                    origin,
                    box3d_rust::math_functions::Vec3 {
                        x: tx,
                        y: ty,
                        z: tz,
                    },
                );
                if index < 0 {
                    return 0;
                }
                state.$bodies.retain(|b| b.body_index != index);
                let extra: &dyn Fn(&mut $state, i32) = &$delete_extra;
                extra(state, index);
                1
            })
        }

        /// Counters + awake/sleeping dynamic body counts (`counters_with_sleep`).
        #[wasm_bindgen::prelude::wasm_bindgen]
        pub fn $counters() -> Vec<f32> {
            $with(|state| $crate::interact::counters_with_sleep(&state.$world).to_vec())
        }

        /// Debug-draw overlay segments/points for the active view flags, in the
        /// scene's base frame (subtracted at emission).
        #[wasm_bindgen::prelude::wasm_bindgen]
        pub fn $debug_draw(_flags: u32) -> Vec<f32> {
            $with(|state| {
                let base_fn: &dyn Fn(&$state) -> box3d_rust::math_functions::Pos = &$base;
                let base: box3d_rust::math_functions::Pos = base_fn(&*state);
                $crate::interact::with_draw_base(base, || {
                    $crate::interact::collect_debug_draw(&mut state.$world)
                })
            })
        }

        /// Debug overlay text (JSON), in the scene's base frame.
        #[wasm_bindgen::prelude::wasm_bindgen]
        pub fn $debug_text() -> String {
            $with(|state| {
                let base_fn: &dyn Fn(&$state) -> box3d_rust::math_functions::Pos = &$base;
                let base: box3d_rust::math_functions::Pos = base_fn(&*state);
                $crate::interact::with_draw_base(base, || {
                    $crate::interact::collect_debug_text(&mut state.$world)
                })
            })
        }
    };
}

/// The four world-tunable toggles (sleep, warm-starting, continuous, contact
/// recycle distance). These are a separate opt-in from [`demo_shell!`] because
/// not every category surfaces them in its UI (Bodies and Shapes do not); a
/// category that does invokes this alongside `demo_shell!` with the same
/// `with_state` / `world` config.
#[macro_export]
macro_rules! demo_world_toggles {
    (
        with_state: $with:path,
        world: $world:ident,
        set_enable_sleep: $set_sleep:ident,
        set_enable_warm_starting: $set_warm:ident,
        set_enable_continuous: $set_cont:ident,
        set_recycle_distance: $set_recycle:ident,
    ) => {
        /// `b3World_EnableSleeping`.
        #[wasm_bindgen::prelude::wasm_bindgen]
        pub fn $set_sleep(flag: bool) {
            $with(|state| box3d_rust::world::world_enable_sleeping(&mut state.$world, flag));
        }

        /// `b3World_EnableWarmStarting`.
        #[wasm_bindgen::prelude::wasm_bindgen]
        pub fn $set_warm(flag: bool) {
            $with(|state| box3d_rust::world::world_enable_warm_starting(&mut state.$world, flag));
        }

        /// `b3World_EnableContinuous`.
        #[wasm_bindgen::prelude::wasm_bindgen]
        pub fn $set_cont(flag: bool) {
            $with(|state| box3d_rust::world::world_enable_continuous(&mut state.$world, flag));
        }

        /// `b3World_SetContactRecycleDistance` (contact recycle distance).
        #[wasm_bindgen::prelude::wasm_bindgen]
        pub fn $set_recycle(meters: f32) {
            $with(|state| {
                box3d_rust::world::world_set_contact_recycle_distance(&mut state.$world, meters)
            });
        }
    };
}
