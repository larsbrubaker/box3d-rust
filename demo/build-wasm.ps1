Push-Location $PSScriptRoot/wasm

# wasm-pack runs binaryen `wasm-opt` when enabled in Cargo.toml. Local Windows
# often lacks it on PATH; fall back so `bun run build:wasm` still succeeds.
$extra = @()
if (-not (Get-Command wasm-opt -ErrorAction SilentlyContinue)) {
  Write-Host "wasm-opt not found on PATH — building without Binaryen optimize (CI still uses -O3)."
  $extra += "--no-opt"
}

wasm-pack build --target web --out-dir ../public/pkg --no-typescript @extra
Pop-Location
