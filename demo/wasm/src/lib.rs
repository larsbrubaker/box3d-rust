// WASM bindings for the browser demos. Every value shown on the demo site is computed
// by the ported Rust code, never re-implemented in JavaScript.

use wasm_bindgen::prelude::*;

/// Version of the box3d-rust port this wasm build was compiled from.
#[wasm_bindgen]
pub fn version() -> String {
    box3d_rust::VERSION.to_string()
}
