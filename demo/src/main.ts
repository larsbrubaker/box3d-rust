import { loadWasm } from "./wasm.ts";

const statusEl = document.getElementById("wasm-status")!;
const versionEl = document.getElementById("port-version")!;

loadWasm()
  .then((wasm) => {
    statusEl.textContent = "wasm build running";
    statusEl.classList.add("badge-ok");
    versionEl.textContent = `box3d-rust v${wasm.version()}`;
  })
  .catch((err) => {
    statusEl.textContent = "wasm failed to load";
    statusEl.classList.add("badge-err");
    console.error("Failed to initialize wasm:", err);
  });
