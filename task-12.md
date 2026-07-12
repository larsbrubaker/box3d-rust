# Task 12 — Demo excellence: port Erin’s Samples App 1:1

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

## Goal (non-negotiable)

**Port Erin Catto’s Box3D Samples App demos 1:1.** Do **not** invent new demos.
Every interactive scene must map to a `RegisterSample(category, name, …)` entry
in `box3d-cpp-reference/samples/`. Rendering is Three.js + Rust/WASM with the
best visual match we can do; physics comes from the ported engine. Godot/Unity
ports are inspiration only — the pinned C samples tree is authority.

Assets (meshes under `box3d-cpp-reference/data/meshes/`, including
`building.obj`) are **MIT** (Copyright 2026 Erin Catto) — copy/adapt with
attribution; see `demo/public/meshes/README.md`.

C reference: **153** `RegisterSample` entries across 19 categories.

## Inventory — C sample → our status

Status key: **done** (faithful scene), **partial** (same category/name but
scaled/stub/missing pieces), **missing**, **invented** (must replace/remove).

### Compound (6) — `sample_compound.cpp`
| C sample | Status | Our route / notes |
|---|---|---|
| Simple | done | `#/compound` Simple |
| Spheres | done | `#/compound` Spheres |
| Hulls | done | `#/compound` Hulls |
| Tile Floor | missing | |
| Mesh Tile | missing | |
| Village | done* | `#/compound` Village + Character Village walk — real `building.obj` compound meshes; browser grid 8–40 (C debug 8 / release 200) |

### Bodies (10) — `sample_bodies.cpp`
| C sample | Status | Notes |
|---|---|---|
| Body Type | partial | `#/bodies` gallery (not 1:1 per sample) |
| Spinning Book | missing | |
| Gyroscopic Torque | missing | |
| Gyroscopic Precession | missing | |
| Weeble | missing | |
| Disable | missing | |
| Cast | missing | |
| Kinematic | missing | |
| Lock Mixing | missing | |
| Fixed Rotation | missing | |

### Stacking (14) — `sample_stacking.cpp`
| C sample | Status | Notes |
|---|---|---|
| Single Box | done | `#/stacking` |
| Box Stack | partial | `#/stacking` |
| Sphere Stack | partial | `#/stacking` |
| Pyramid2D | partial | `#/stacking` |
| Jenga Stack | partial | `#/stacking` |
| Card House / Thick, Capsule Stack, Cylinder*, Dominoes, Wedge, Arch, Double Domino | missing | |

### Joints (16) — `sample_joint.cpp`
| C sample | Status | Notes |
|---|---|---|
| Revolute | done | `#/joints` |
| Gear Lift | partial | `#/joints` (sibling agents refining) |
| Driving | partial | `#/joints` |
| Distance, Filter, Motor, Top Down Friction, Prismatic, Spherical, Parallel Spring, Weld, Wheel, Ball and Chain, Door, Bridge, Motion Locks | missing | |

### Character (4) — `sample_character.cpp`
| C sample | Status | Notes |
|---|---|---|
| Mover | partial | `#/character` BasicMover-style |
| CapsulePlane, MoverOverlap, Rigid Body | missing | |
| (Village mover) | done* | Character → Village uses Compound Village scene (C embeds mover in Village) |

### Benchmark (18) — `sample_benchmark.cpp`
| C sample | Status | Notes |
|---|---|---|
| Large Pyramid | partial | `#/benchmark` |
| Junkyard | partial | `#/benchmark` |
| Falling Trees | partial | `#/benchmark` |
| Wide/Many Pyramids, Rain, Large World, Joint Grid, Falling Boxes, Candy Cups, Explosion, Height Field, Sensor, Washer, Hull, Chains, Destruction, … | missing | |

### Continuous (10) — `sample_continuous.cpp`
| Status | Notes |
|---|---|
| partial / in flight | `#/continuous` — sibling agents; Thin Wall etc. |

### Events (6) / Sensors
| Status | Notes |
|---|---|
| partial | `#/sensors` — not full Events gallery |

### Collision / Queries (12) — `sample_collision.cpp`
| Status | Notes |
|---|---|
| partial | `#/queries` + geometry/hull demos |

### Mesh (9) / Manifold (9) / Geometry (5) / Tree (1)
| Status | Notes |
|---|---|
| partial | `#/mesh`, `#/manifolds`, `#/geometry`, `#/tree`, `#/terrain`, `#/height-field`, `#/hull` |

### Ragdoll (5)
| Status | Notes |
|---|---|
| partial | `#/ragdolls` |

### Shapes (12), World (4), Robustness (4), Issues (7), Determinism (1), Replay
| Status | Notes |
|---|---|
| missing / invented stubs | Roadmap cards that point at unrelated routes are **invented UX** — replace with real sample pages |

### Invented / stub pages to replace
- Roadmap “Shapes” → `#/bodies`, “Events” → `#/sensors`, “World” → `#/queries`, “Determinism” → `#/math` are **category placeholders**, not Erin samples — mark PLANNED until real samples land; do not advertise as live Samples App demos.
- Any “tiny grid Village without buildings” stub is **replaced** by the real Village port (this track).

## 1. Core interaction layer (shared by all demos)

**Done (2026-07-12):** shared TS layer + Samples App Info panel shell.

**Remaining:** opt remaining dynamics demos (ragdolls, continuous, sensors,
queries, terrain) into `attachInteraction` / Samples shell.

## 2. Visual quality

- [ ] Instanced meshes for high-body-count stacks/benchmarks (Village buildings already instanced)
- [ ] Camera polish
- [ ] Samples shell + body colorization on remaining dynamics demos

## 3. Coverage — port remaining RegisterSample entries

Priority (highest gap, avoid duplicating sibling WIP on stacking/joints/continuous/sensors/queries):

- [ ] Shapes gallery (`sample_shapes.cpp`) — real samples only
- [ ] Events (`sample_events.cpp`)
- [ ] World far scenes (`sample_world.cpp`)
- [ ] Robustness (`sample_robustness.cpp`)
- [ ] Remaining Bodies / Stacking / Joints / Benchmark / Mesh / Manifold / Collision
- [ ] Compound Tile Floor + Mesh Tile
- [ ] Replay viewer (`sample_replay.cpp`)
- [ ] Determinism readout on Falling Ragdolls

## 4. Site polish

- [ ] Landing page category cards with real sample counts (not invented demos)
- [ ] Deep links + prev/next between **real** samples
- [ ] Per-demo link to matching C sample source
- [ ] Mobile touch controls
- [ ] Footer version + git hash
- [ ] Refresh `readme_hero.jpg`
