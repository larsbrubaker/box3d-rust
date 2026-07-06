Push-Location $PSScriptRoot/wasm
wasm-pack build --target web --out-dir ../public/pkg --no-typescript
Pop-Location
