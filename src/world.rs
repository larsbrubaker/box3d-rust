// Port of the world data model from box3d-cpp-reference/src/physics_world.h
// (b3World, b3TaskContext) plus b3Profile from types.h. Logic from
// physics_world.c lands across the remaining bring-up commits.
//
// Porting decisions (mirroring box2d-rust):
// - C keeps a global world registry. The Rust `World` is an owned object;
//   `world_id`/`generation` are kept so body/shape/joint ids remain
//   bit-compatible with C.
// - The arena allocator (b3Stack) is per-step scratch; the Rust solver
//   allocates scratch as Vecs in the step, so there is no stack field.
// - Manifold block allocators become Vec-owned manifolds on Contact for now;
//   a pooled allocator can return later if profiling warrants it.
// - Scheduler/task-system fields are deferred with the single-threaded port;
//   worker_count is kept and clamped like C.
// - b3Recording lands with its slice; placeholders omit opaque C pointers.
// - Hull database is a content-keyed Rc store (verstable map in C).
// - Pre-solve and custom-filter callbacks keep the C shape as Option<fn> with
//   a u64 context.
//
// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

use crate::bitset::BitSet;
use crate::body::Body;
use crate::broad_phase::BroadPhase;
use crate::constants::CONTACT_MANIFOLD_COUNT_BUCKETS;
use crate::constraint_graph::ConstraintGraph;
use crate::contact::Contact;
use crate::events::{
    BodyMoveEvent, ContactBeginTouchEvent, ContactEndTouchEvent, ContactHitEvent, JointEvent,
    SensorBeginTouchEvent, SensorEndTouchEvent,
};
use crate::hull::HullDatabase;
use crate::id::ShapeId;
use crate::id_pool::IdPool;
use crate::island::Island;
use crate::joint::Joint;
use crate::math_functions::Vec3;
use crate::name_cache::NameCache;
use crate::sensor::{Sensor, SensorHit, SensorTaskContext};
use crate::shape::Shape;
use crate::solver_set::SolverSet;
use crate::types::{Capacity, FrictionCallback, RestitutionCallback};

/// Prototype for a contact filter callback. (b3CustomFilterFcn)
pub type CustomFilterFcn = fn(ShapeId, ShapeId, u64) -> bool;

/// Prototype for a pre-solve callback. (b3PreSolveFcn)
pub type PreSolveFcn = fn(ShapeId, ShapeId, crate::math_functions::Pos, crate::math_functions::Vec3, u64) -> bool;

/// Profiling data. Times are in milliseconds. (b3Profile)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Profile {
    pub step: f32,
    pub pairs: f32,
    pub collide: f32,
    pub solve: f32,
    pub solver_setup: f32,
    pub constraints: f32,
    pub prepare_constraints: f32,
    pub integrate_velocities: f32,
    pub warm_start: f32,
    pub solve_impulses: f32,
    pub integrate_positions: f32,
    pub relax_impulses: f32,
    pub apply_restitution: f32,
    pub store_impulses: f32,
    pub split_islands: f32,
    pub transforms: f32,
    pub sensor_hits: f32,
    pub joint_events: f32,
    pub hit_events: f32,
    pub refit: f32,
    pub bullets: f32,
    pub sleep_islands: f32,
    pub sensors: f32,
}

/// Per thread task storage. (b3TaskContext)
///
/// Arena, debug draw buffers, and false-sharing padding from C are omitted;
/// the serial port owns step scratch locally.
#[derive(Debug, Clone, Default)]
pub struct TaskContext {
    /// Collect per thread sensor continuous hit events.
    pub sensor_hits: Vec<SensorHit>,

    /// Align with contact id capacity; signal a change in contact status.
    pub contact_state_bit_set: BitSet,

    /// Align with joint id capacity; signal a change in joint status.
    pub joint_state_bit_set: BitSet,

    /// Align with contact id capacity; signal a hit event.
    pub hit_event_bit_set: BitSet,

    /// Fast-path flag: true when this worker set at least one bit in
    /// hit_event_bit_set this step.
    pub has_hit_events: bool,

    /// Track bodies with shapes that have enlarged AABBs.
    pub enlarged_sim_bit_set: BitSet,

    /// Used to put islands to sleep.
    pub awake_island_bit_set: BitSet,

    /// Per worker split island candidate.
    pub split_sleep_time: f32,
    pub split_island_id: i32,

    /// Profiling
    pub sat_call_count: i32,
    pub sat_cache_hit_count: i32,
    pub distance_iterations: i32,
    pub push_back_iterations: i32,
    pub root_iterations: i32,

    /// Number of contacts recycled this step (collide pass).
    pub recycled_contact_count: i32,

    pub manifold_counts: [i32; CONTACT_MANIFOLD_COUNT_BUCKETS],
}

/// The world struct manages all physics entities, dynamic simulation, and
/// asynchronous queries. (b3World)
#[derive(Debug)]
pub struct World {
    pub broad_phase: BroadPhase,
    pub constraint_graph: ConstraintGraph,

    /// The body id pool allocates and recycles body ids. Aligns with `bodies`.
    pub body_id_pool: IdPool,

    /// Sparse array mapping body ids to body data stored in solver sets.
    pub bodies: Vec<Body>,

    /// Provides free list for solver sets.
    pub solver_set_id_pool: IdPool,

    /// Solver sets store sims in contiguous arrays. Set 0 is static, set 1 is
    /// disabled, set 2 is awake; the rest are sleeping islands.
    pub solver_sets: Vec<SolverSet>,

    /// Used to create stable ids for joints.
    pub joint_id_pool: IdPool,

    /// Sparse array mapping joint ids to joints in the constraint graph or
    /// solver sets.
    pub joints: Vec<Joint>,

    /// Used to create stable ids for contacts.
    pub contact_id_pool: IdPool,

    /// Sparse array mapping contact ids to contacts.
    pub contacts: Vec<Contact>,

    /// Used to create stable ids for islands.
    pub island_id_pool: IdPool,

    /// Persistent islands.
    pub islands: Vec<Island>,

    pub shape_id_pool: IdPool,

    /// Sparse array of shapes.
    pub shapes: Vec<Shape>,

    /// Content-keyed shared hull store. (world->hullDatabase)
    pub hull_database: HullDatabase,

    /// Name cache for shape and body names.
    pub names: NameCache,

    /// Dense array of sensor data.
    pub sensors: Vec<Sensor>,

    /// Per thread storage (one entry in the single-threaded port).
    pub task_contexts: Vec<TaskContext>,
    pub sensor_task_contexts: Vec<SensorTaskContext>,

    pub body_move_events: Vec<BodyMoveEvent>,
    pub sensor_begin_events: Vec<SensorBeginTouchEvent>,
    pub contact_begin_events: Vec<ContactBeginTouchEvent>,

    /// End events are double buffered so the user doesn't need to flush events.
    pub sensor_end_events: [Vec<SensorEndTouchEvent>; 2],
    pub contact_end_events: [Vec<ContactEndTouchEvent>; 2],
    pub end_event_array_index: i32,

    pub contact_hit_events: Vec<ContactHitEvent>,
    pub joint_events: Vec<JointEvent>,

    /// Used to track debug draw.
    pub debug_body_set: BitSet,
    pub debug_joint_set: BitSet,
    pub debug_contact_set: BitSet,
    pub debug_island_set: BitSet,

    /// Id that is incremented every time step.
    pub step_index: u64,

    /// Identify islands for splitting.
    pub split_island_id: i32,

    pub gravity: Vec3,
    pub hit_event_threshold: f32,
    pub restitution_threshold: f32,
    pub max_linear_speed: f32,
    pub contact_speed: f32,
    pub contact_hertz: f32,
    pub contact_damping_ratio: f32,
    pub contact_recycle_distance: f32,

    pub friction_callback: Option<FrictionCallback>,
    pub restitution_callback: Option<RestitutionCallback>,

    pub generation: u16,

    pub profile: Profile,
    pub sat_call_count: i32,
    pub sat_cache_hit_count: i32,
    pub manifold_counts: [i32; CONTACT_MANIFOLD_COUNT_BUCKETS],

    pub max_capacity: Capacity,

    pub pre_solve_fcn: Option<PreSolveFcn>,
    pub pre_solve_context: u64,

    pub custom_filter_fcn: Option<CustomFilterFcn>,
    pub custom_filter_context: u64,

    pub worker_count: i32,

    pub user_data: u64,

    /// Inverse sub-step, remembered for reporting forces and torques.
    pub inv_h: f32,

    /// Inverse full-step.
    pub inv_dt: f32,

    pub world_id: u16,

    pub enable_sleep: bool,
    pub locked: bool,
    pub enable_warm_starting: bool,
    pub enable_continuous: bool,
    pub enable_speculative: bool,
    pub in_use: bool,
}

/// Default friction mixing: `sqrt(frictionA * frictionB)`.
/// (static b3DefaultFrictionCallback — from types.c / physics_world.c)
pub fn default_friction_callback(
    friction_a: f32,
    _material_a: u64,
    friction_b: f32,
    _material_b: u64,
) -> f32 {
    (friction_a * friction_b).sqrt()
}

/// Default restitution mixing: `max(restitutionA, restitutionB)`.
pub fn default_restitution_callback(
    restitution_a: f32,
    _material_a: u64,
    restitution_b: f32,
    _material_b: u64,
) -> f32 {
    crate::math_functions::max_float(restitution_a, restitution_b)
}

impl World {
    /// Create a world. (b3CreateWorld)
    ///
    /// Differences from C, all documented in the module header: there is no
    /// global world registry (the returned World is owned; `world_id` stays 0
    /// unless the embedder assigns one), no arena stack / manifold block
    /// allocator yet, and the serial task path is always used
    /// (worker_count = 1 with one task context), which is the C fallback when
    /// no task system is supplied.
    pub fn new(def: &crate::types::WorldDef) -> World {
        use crate::constants::{
            contact_recycle_distance, linear_slop, mesh_rest_offset, speculative_distance,
        };
        use crate::core::{NULL_INDEX, SECRET_COOKIE};
        use crate::math_functions::max_int;
        use crate::solver_set::{AWAKE_SET, DISABLED_SET, STATIC_SET};

        debug_assert!(def.internal_value == SECRET_COOKIE);
        debug_assert!(linear_slop() <= mesh_rest_offset());
        debug_assert!(mesh_rest_offset() < speculative_distance());

        use crate::contact::initialize_contact_registers;
        initialize_contact_registers();

        let body_capacity = max_int(
            16,
            def.capacity.static_body_count + def.capacity.dynamic_body_count,
        ) as usize;
        let shape_capacity = max_int(
            16,
            def.capacity.static_shape_count + def.capacity.dynamic_shape_count,
        ) as usize;
        let contact_capacity = max_int(16, def.capacity.contact_count) as usize;

        let mut solver_set_id_pool = IdPool::new();
        let mut solver_sets: Vec<SolverSet> = Vec::with_capacity(8);

        // add empty static, disabled, and awake body sets
        // static set
        let mut set = SolverSet {
            set_index: solver_set_id_pool.alloc_id(),
            ..Default::default()
        };
        set.body_sims
            .reserve(max_int(16, def.capacity.static_body_count) as usize);
        solver_sets.push(set);
        debug_assert!(solver_sets[STATIC_SET as usize].set_index == STATIC_SET);

        // disabled set
        solver_sets.push(SolverSet {
            set_index: solver_set_id_pool.alloc_id(),
            ..Default::default()
        });
        debug_assert!(solver_sets[DISABLED_SET as usize].set_index == DISABLED_SET);

        // awake set
        let mut awake = SolverSet {
            set_index: solver_set_id_pool.alloc_id(),
            ..Default::default()
        };
        awake
            .body_sims
            .reserve(max_int(16, def.capacity.dynamic_body_count) as usize);
        awake
            .body_states
            .reserve(max_int(16, def.capacity.dynamic_body_count) as usize);
        awake.contact_indices.reserve(contact_capacity);
        solver_sets.push(awake);
        debug_assert!(solver_sets[AWAKE_SET as usize].set_index == AWAKE_SET);

        World {
            broad_phase: BroadPhase::new(&def.capacity),
            constraint_graph: ConstraintGraph::new(16),
            body_id_pool: IdPool::new(),
            bodies: Vec::with_capacity(body_capacity),
            solver_set_id_pool,
            solver_sets,
            joint_id_pool: IdPool::new(),
            joints: Vec::with_capacity(16),
            contact_id_pool: IdPool::new(),
            contacts: Vec::with_capacity(contact_capacity),
            island_id_pool: IdPool::new(),
            islands: Vec::with_capacity(max_int(16, def.capacity.dynamic_body_count) as usize),
            shape_id_pool: IdPool::new(),
            shapes: Vec::with_capacity(shape_capacity),
            hull_database: HullDatabase::new(),
            names: NameCache::new(),
            sensors: Vec::with_capacity(4),
            // Serial fallback: one worker context. (b3CreateWorkerContexts)
            task_contexts: vec![TaskContext::default()],
            sensor_task_contexts: vec![SensorTaskContext::default()],
            body_move_events: Vec::with_capacity(4),
            sensor_begin_events: Vec::with_capacity(4),
            contact_begin_events: Vec::with_capacity(4),
            sensor_end_events: [Vec::with_capacity(4), Vec::with_capacity(4)],
            contact_end_events: [Vec::with_capacity(4), Vec::with_capacity(4)],
            end_event_array_index: 0,
            contact_hit_events: Vec::with_capacity(4),
            joint_events: Vec::with_capacity(4),
            debug_body_set: BitSet::new(256),
            debug_joint_set: BitSet::new(256),
            debug_contact_set: BitSet::new(256),
            debug_island_set: BitSet::new(256),
            step_index: 0,
            split_island_id: NULL_INDEX,
            gravity: def.gravity,
            hit_event_threshold: def.hit_event_threshold,
            restitution_threshold: def.restitution_threshold,
            max_linear_speed: def.maximum_linear_speed,
            contact_speed: def.contact_speed,
            contact_hertz: def.contact_hertz,
            contact_damping_ratio: def.contact_damping_ratio,
            contact_recycle_distance: contact_recycle_distance(),
            friction_callback: Some(def.friction_callback.unwrap_or(default_friction_callback)),
            restitution_callback: Some(
                def.restitution_callback
                    .unwrap_or(default_restitution_callback),
            ),
            generation: 0,
            profile: Profile::default(),
            sat_call_count: 0,
            sat_cache_hit_count: 0,
            manifold_counts: [0; CONTACT_MANIFOLD_COUNT_BUCKETS],
            max_capacity: def.capacity,
            pre_solve_fcn: None,
            pre_solve_context: 0,
            custom_filter_fcn: None,
            custom_filter_context: 0,
            worker_count: 1,
            user_data: def.user_data,
            inv_h: 0.0,
            inv_dt: 0.0,
            world_id: 0,
            enable_sleep: def.enable_sleep,
            locked: false,
            enable_warm_starting: true,
            enable_continuous: def.enable_continuous,
            enable_speculative: true,
            in_use: true,
        }
    }

    /// Debug validation of solver set bookkeeping. (b3ValidateSolverSets)
    ///
    /// C gates this behind B3_VALIDATE; here it always runs and asserts in
    /// debug. Contact/joint set checks widen as those slices land.
    pub fn validate_solver_sets(&self) {
        use crate::core::NULL_INDEX;
        use crate::solver_set::AWAKE_SET;

        let mut active_body_count = 0;
        for (set_index, set) in self.solver_sets.iter().enumerate() {
            if set.set_index == NULL_INDEX {
                debug_assert!(set.body_sims.is_empty());
                debug_assert!(set.body_states.is_empty());
                debug_assert!(set.island_sims.is_empty());
                continue;
            }

            debug_assert!(set.set_index == set_index as i32);

            if set_index == AWAKE_SET as usize {
                debug_assert!(set.body_sims.len() == set.body_states.len());
            } else {
                debug_assert!(set.body_states.is_empty());
            }

            for (local_index, sim) in set.body_sims.iter().enumerate() {
                let body = &self.bodies[sim.body_id as usize];
                debug_assert!(body.set_index == set_index as i32);
                debug_assert!(body.local_index == local_index as i32);
                debug_assert!(body.id == sim.body_id);
                let _ = (body, local_index);
            }
            active_body_count += set.body_sims.len() as i32;

            for (local_index, island_sim) in set.island_sims.iter().enumerate() {
                let island = &self.islands[island_sim.island_id as usize];
                debug_assert!(island.set_index == set_index as i32);
                debug_assert!(island.local_index == local_index as i32);
                let _ = (island, local_index);
            }
        }

        debug_assert!(active_body_count == self.body_id_pool.id_count());
        let _ = active_body_count;
    }

    /// Advance the world simulation by one time step. (b3World_Step)
    ///
    /// Pairs phase is live. Collide/solve/sleep land in follow-on commits;
    /// an empty world (no bodies / no moves) is a full no-op after pair update.
    pub fn step(&mut self, time_step: f32, sub_step_count: i32) {
        use crate::broad_phase::update_broad_phase_pairs;
        use crate::math_functions::{is_valid_float, max_int};

        debug_assert!(is_valid_float(time_step) && time_step >= 0.0);
        debug_assert!(!self.locked);
        if self.locked {
            return;
        }

        self.locked = true;

        self.body_move_events.clear();
        self.sensor_begin_events.clear();
        self.contact_begin_events.clear();
        self.contact_hit_events.clear();
        self.joint_events.clear();
        self.profile = Profile::default();

        {
            let c = &mut self.max_capacity;
            c.static_shape_count = max_int(
                c.static_shape_count,
                self.broad_phase.trees[crate::types::BodyType::Static as usize].proxy_count(),
            );
            c.dynamic_shape_count = max_int(
                c.dynamic_shape_count,
                self.broad_phase.trees[crate::types::BodyType::Dynamic as usize].proxy_count(),
            );
            let static_body_count =
                self.solver_sets[crate::solver_set::STATIC_SET as usize]
                    .body_sims
                    .len() as i32;
            c.static_body_count = max_int(c.static_body_count, static_body_count);
            let total_body_count = self.body_id_pool.id_count();
            c.dynamic_body_count =
                max_int(c.dynamic_body_count, total_body_count - static_body_count);
            c.contact_count = max_int(c.contact_count, self.contact_id_pool.id_count());
        }

        update_broad_phase_pairs(self);

        let sub_steps = max_int(1, sub_step_count);
        let dt = time_step;
        if time_step > 0.0 {
            self.inv_dt = 1.0 / time_step;
            self.inv_h = sub_steps as f32 * self.inv_dt;
        } else {
            self.inv_dt = 0.0;
            self.inv_h = 0.0;
        }

        crate::contact::collide(self, dt);

        // Solve / sleep deferred — collide + pairs are live; empty and
        // overlapping-but-unsolved scenes advance contact state correctly.

        self.step_index = self.step_index.wrapping_add(1);
        self.validate_solver_sets();
        self.locked = false;
    }
}
