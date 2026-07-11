# Task 3 — Sensors

**Remaining work only; delete items as they complete and delete this file when
the track is done (also remove its row from todo.md).**

Port `sensor.c`. The data model (`Sensor`, `SensorHit`, `SensorTaskContext` in
`src/sensor.rs`) and the world arrays/events exist; the overlap logic and all
wiring do not ("Overlap update logic lands in a later bring-up commit").

Touches the tail of `World::step` (`src/world/step.rs`) and adds the
sensor-hits report pass to `src/solver/solve.rs` — coordinate with task-1 and
task-2.

## Creation and destruction

- [ ] Sensor shape create path: `shape_def.is_sensor` allocates a
      `Sensor` entry, sets `shape.sensor_index`, registers in `world.sensors`
- [ ] Destroy path: remove sensor, fix moved `sensor_index`, flush pending
      events for the destroyed sensor

## Overlap sweep (sensor.c)

- [ ] `b3OverlapSensors`: per-sensor broad-phase overlap query into the
      double-buffered overlap arrays (`overlaps1`/`overlaps2` swap),
      visitor filtering (sensor-vs-sensor rules, self-body exclusion,
      filter bits, enable flag)
- [ ] Begin/end event generation by diffing the two overlap buffers
      (deterministic order), honoring shape `enable_sensor_events`
- [ ] Wire the "Update sensors" block at the end of `World::step`
      (C physics_world.c runs `b3OverlapSensors` after solve)
- [ ] Double-buffered `sensor_end_events` rotation with
      `end_event_array_index` (matches the contact end-event pattern)

## Solver integration

- [ ] Sensor-hits report pass in `solve()` (the C block after bullets):
      drain `task_context.sensor_hits` into per-sensor `hits` arrays —
      the producer is CCD sweeps (task-2), but the pass and its wiring
      belong here and must not depend on task-2 landing first
      (`sensor_hits` is simply empty until then)

## Tests

- [ ] Port `TestSensor` from `test_world.c`
- [ ] Test: sensor begin/end events across sleep — a sensor overlapping a body
      that falls asleep must not spuriously end/begin
