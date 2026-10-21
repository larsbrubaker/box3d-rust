"""Shared parser for the B3_REC_OP x-macro manifest (recording_ops.inl).

Used by gen_recording_ops.py and gen_recording_dispatch.py so both generators see the
same op list, in manifest order. Malformed entries are errors, never silently skipped:
a dropped op would make the generated op table and dispatcher disagree with C.
"""

from __future__ import annotations

import pathlib
import re

ROOT = pathlib.Path(__file__).resolve().parents[1]
INL = ROOT / "box3d-cpp-reference" / "src" / "recording_ops.inl"

Op = tuple[int, str, str, list[tuple[str, str]]]

_OP_START = re.compile(r"B3_REC_OP\(\s*(0x[0-9A-Fa-f]+)\s*,\s*(\w+)\s*,\s*(RET_\w+)\s*,")
_OP_FULL = re.compile(
    r"B3_REC_OP\(\s*(0x[0-9A-Fa-f]+)\s*,\s*(\w+)\s*,\s*(RET_\w+)\s*,(.*)\)$", re.S
)
_ARG = re.compile(r"ARG\(\s*(\w+)\s*,\s*(\w+)\s*\)")


def parse_ops(prog: str, inl: pathlib.Path = INL) -> list[Op]:
    """Parse every B3_REC_OP entry as (opcode, name, ret, [(tag, field), ...]).

    `prog` prefixes error messages. Raises SystemExit on an unbalanced entry or on an
    ARG( whose fields the parser could not read.
    """
    text = inl.read_text(encoding="utf-8")
    flat = re.sub(r"\\\n", " ", text)
    flat = re.sub(r"[ \t]*\n[ \t]*", " ", flat)
    ops: list[Op] = []
    for m in _OP_START.finditer(flat):
        i = flat.index("(", m.start()) + 1
        depth = 1
        while i < len(flat) and depth:
            if flat[i] == "(":
                depth += 1
            elif flat[i] == ")":
                depth -= 1
            i += 1
        inner = flat[m.start() : i]
        mm = _OP_FULL.match(inner) if depth == 0 else None
        if not mm:
            snippet = inner[:120]
            raise SystemExit(
                f"{prog}: unbalanced B3_REC_OP entry for op {m.group(2)} in {inl}: {snippet}"
            )
        opcode, name, ret, args = mm.groups()
        fields = _ARG.findall(args)
        arg_count = args.count("ARG(")
        if arg_count != len(fields):
            raise SystemExit(
                f"{prog}: op {name} has {arg_count} ARG( entries but {len(fields)} parsed "
                f"fields; fix the manifest parser in recording_manifest.py"
            )
        ops.append((int(opcode, 16), name, ret, fields))
    return ops
