#!/bin/sh
# Build the WASM demo and serve it at http://localhost:3000.
# Run from anywhere: ./run_demo.sh   (Ctrl+C to stop)
set -e

cd "$(dirname "$0")/demo"

command -v bun >/dev/null 2>&1 || {
  echo "[run_demo] Bun is required but was not found on PATH."
  echo "          Install it from https://bun.sh and try again."
  exit 1
}

command -v wasm-pack >/dev/null 2>&1 || {
  echo "[run_demo] wasm-pack is required but was not found on PATH."
  echo "          Install it with: cargo install wasm-pack"
  exit 1
}

if [ ! -d node_modules ]; then
  echo "[run_demo] Installing demo dependencies..."
  bun install
fi

echo "[run_demo] Building WASM from the Rust port..."
( cd wasm && wasm-pack build --target web --out-dir ../public/pkg --no-typescript )

# Open the browser shortly after the server has had time to start.
( sleep 2
  if command -v xdg-open >/dev/null 2>&1; then xdg-open http://localhost:3000
  elif command -v open >/dev/null 2>&1; then open http://localhost:3000
  fi ) &

echo "[run_demo] Serving at http://localhost:3000  (Ctrl+C to stop)"
bun run dev
