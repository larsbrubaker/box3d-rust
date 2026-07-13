# Tree Benchmark AABB record files

`bounds01.txt`, `bounds02.txt`, `bounds03.txt` are the AABB record files used by
the Box3D **Tree / Benchmark** sample (`box3d-cpp-reference/samples/sample_tree.cpp`).

Each line is one axis-aligned bounding box, six whitespace-separated floats:

```
lowerX lowerY lowerZ upperX upperY upperZ
```

The Tree Benchmark demo (`demo/src/demos/tree.ts`) fetches these at scene load and
feeds each record to `b3DynamicTree_CreateProxy`, exactly as the C sample does with
`fopen`/`fgets`/`sscanf`. Per the C `m_config` table the demo applies a per-file
scale (`bounds03` at 0.01, the others at 1.0) and no axis swap (`zUp = false`).

## Provenance / attribution

Copied verbatim from the pinned Box3D C reference submodule
`box3d-cpp-reference/data/trees/` (Box3D, Erin Catto).

- SPDX-FileCopyrightText: 2025 Erin Catto
- SPDX-License-Identifier: MIT
