#!/bin/sh
# Build the C reference at upstream 3362633 (scalar path, -ffp-contract=off, f32 and
# BOX3D_DOUBLE_PRECISION), generate the Character mover trace drivers from the pinned
# samples/sample_character.cpp, run the scripted Dynamic Mover and Geometric Mover
# sessions, and print the FNV-1a 64 of each trace. Those hashes are the golden values
# in demo/wasm/src/character_demo/{dynamic,mover}/tests.rs; dump the Rust traces with
#   DYNAMIC_MOVER_TRACE=out.txt cargo test dump_dynamic_mover_trace -- --ignored
#   GEOMETRIC_MOVER_TRACE=out.txt cargo test dump_geometric_mover_trace -- --ignored
# (from demo/wasm, `--features box3d-rust/double-precision` for f64) to diff line by line.
#
# Usage: scripts/c_trace/run.sh <box3d git checkout> <scratch dir> [commit]
set -eu
REF=$(cd "$1" && pwd)
OUT=$(mkdir -p "$2" && cd "$2" && pwd)
COMMIT=${3:-3362633}
HERE=$(cd "$(dirname "$0")" && pwd)

mkdir -p "$OUT/c" "$OUT/drivers/gfx"
git -C "$REF" archive "$COMMIT" include src samples/dynamic_mover.cpp samples/dynamic_mover.h \
	samples/geometric_mover.cpp samples/geometric_mover.h samples/mesh_loader.cpp \
	samples/mesh_loader.h samples/tiny_obj_loader.h samples/earcut.h \
	samples/sample_character.cpp | tar -x -C "$OUT/c"

for mode in f32 f64; do
	defs="-DBOX3D_DISABLE_SIMD -DNDEBUG"
	[ "$mode" = f64 ] && defs="$defs -DBOX3D_DOUBLE_PRECISION"
	mkdir -p "$OUT/build_$mode"
	for f in "$OUT"/c/src/*.c; do
		clang -c -O2 -std=c17 -ffp-contract=off $defs -I"$OUT/c/include" -I"$OUT/c/src" \
			"$f" -o "$OUT/build_$mode/$(basename "$f" .c).o"
	done
	ar rcs "$OUT/build_$mode/libbox3d.a" "$OUT"/build_$mode/*.o
done

cp "$HERE/gfx/draw.h" "$OUT/drivers/gfx/draw.h"
cp "$HERE/dynamic_prefix.h" "$HERE/dynamic_suffix.h" "$OUT/drivers/"
python3 -I "$HERE/gen_drivers.py" "$OUT/c/samples/sample_character.cpp" "$OUT/drivers"

S="$OUT/c/samples"
for mode in f32 f64; do
	defs="-DBOX3D_DISABLE_SIMD -DNDEBUG"
	[ "$mode" = f64 ] && defs="$defs -DBOX3D_DOUBLE_PRECISION"
	clang++ -std=c++20 -O2 -ffp-contract=off $defs -I"$OUT/drivers" -I"$OUT/c/include" -I"$S" \
		"$HERE/trace_dynamic.cpp" "$S/dynamic_mover.cpp" "$OUT/build_$mode/libbox3d.a" \
		-o "$OUT/trace_dynamic_$mode"
	clang++ -std=c++20 -O2 -ffp-contract=off $defs -I"$OUT/drivers" -I"$OUT/c/include" -I"$S" \
		"$HERE/trace_geometric.cpp" "$S/geometric_mover.cpp" "$S/mesh_loader.cpp" \
		"$OUT/build_$mode/libbox3d.a" -o "$OUT/trace_geometric_$mode"
	# The Geometric Mover loads data/meshes/*.obj relative to the working directory.
	(cd "$REF" && "$OUT/trace_dynamic_$mode" 900 > "$OUT/c_dynamic_$mode.txt")
	(cd "$REF" && "$OUT/trace_geometric_$mode" 900 > "$OUT/c_geometric_$mode.txt")
done

for t in "$OUT"/c_*.txt; do
	python3 -I -c 'import sys
h = 0xcbf29ce484222325
for b in open(sys.argv[1], "rb").read():
    h = ((h ^ b) * 0x100000001b3) & 0xffffffffffffffff
print(f"{h:016x}  {sys.argv[1]}")' "$t"
done
