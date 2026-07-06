#!/bin/sh
cd "$(dirname "$0")/wasm"
wasm-pack build --target web --out-dir ../public/pkg --no-typescript
