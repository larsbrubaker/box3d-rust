# Bundled replay recordings

`sample.b3rec` is a Box3D recording (`.b3rec`) played by the Replay viewer
(`#/replay`, port of `samples/sample_replay.cpp`).

## Provenance

It was **recorded from this port itself**, in-browser, through the same WebAssembly
build and recording exports the demo's Recording panel uses — not imported from the
C samples app. Concretely, it was captured by driving the wasm module directly:
reset the Stacking **Box Stack** scene (`sim_reset_stacking`), then
`sim_start_recording()` → 300 × `sim_step(1/60, 4)` → `sim_stop_recording()`, i.e.
**5 seconds at 60 Hz with 4 sub-steps** of a 41-body box stack settling under
gravity.

Recording it through the wasm build (rather than a native 64-bit binary) matters:
the `.b3rec` snapshot layout is pointer-width dependent, and `RecPlayer` rejects a
recording whose pointer width differs from the host's. The bundled file is
therefore 32-bit, matching the wasm player it ships with.

You can also load any `.b3rec` you record yourself: use the **Record** buttons on
any dynamics demo (e.g. Stacking) to capture and download a recording, then load it
into the Replay viewer with the **Choose File** picker. Loading a local file via
the picker is ordinary in-page behavior — nothing is uploaded anywhere.
