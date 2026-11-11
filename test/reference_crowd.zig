//! Declarations for the C shim over upstream DetourCrowd, test/shim/crowd_shim.cpp.
//!
//! The pure-Zig crowd is tested against upstream through these functions, byte
//! for byte. Names are the shim's: `ref_` + area + operation, one function per
//! upstream function or method, object pointer first, upstream's argument
//! order. Statuses are `u32`. Positions and velocities are `*const [3]f32` in
//! and `*[3]f32` out.
//!
//! A pointer a getter returns points into the object asked, and goes stale when
//! that object is freed, re-initialised or changed. A pointer an object stores
//! (the navigation mesh given to `ref_crowd_init` and `ref_path_queue_init`, the
//! filter given to `ref_path_queue_request`) must outlive that object.

const core = @import("reference_detour.zig");

/// `dtPathQueueRef`: 32 bits in every build configuration; 0 is invalid.
pub const PathQueueRef = u32;

pub const Crowd = opaque {};
/// One slot of a crowd's agent pool. Read it with `ref_crowd_agent_snapshot`.
pub const CrowdAgent = opaque {};
pub const ProximityGrid = opaque {};
pub const ObstacleAvoidanceQuery = opaque {};
pub const ObstacleAvoidanceDebugData = opaque {};
pub const LocalBoundary = opaque {};
pub const PathCorridor = opaque {};
pub const PathQueue = opaque {};

/// `DT_CROWDAGENT_MAX_NEIGHBOURS`.
pub const max_neighbours = 6;
/// `DT_CROWDAGENT_MAX_CORNERS`.
pub const max_corners = 4;
/// `DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS`.
pub const max_obstacle_avoidance_params = 8;
/// `DT_CROWD_MAX_QUERY_FILTER_TYPE`.
pub const max_query_filter_types = 16;
/// The room `dtCrowd::init` gives every agent's corridor.
pub const max_corridor_path = 256;
/// `dtLocalBoundary`'s private `MAX_LOCAL_SEGS`.
pub const max_boundary_segments = 8;

/// `dtObstacleAvoidanceParams`.
pub const ObstacleAvoidanceParams = extern struct {
    vel_bias: f32,
    weight_des_vel: f32,
    weight_cur_vel: f32,
    weight_side: f32,
    weight_toi: f32,
    horiz_time: f32,
    grid_size: u8,
    adaptive_divs: u8,
    adaptive_rings: u8,
    adaptive_depth: u8,
};

/// `dtCrowdAgentParams`.
pub const CrowdAgentParams = extern struct {
    radius: f32,
    height: f32,
    max_acceleration: f32,
    max_speed: f32,
    collision_query_range: f32,
    path_optimization_range: f32,
    separation_weight: f32,
    update_flags: u8,
    obstacle_avoidance_type: u8,
    query_filter_type: u8,
    user_data: ?*anyopaque,
};

/// `dtCrowdNeighbour`.
pub const CrowdNeighbour = extern struct {
    idx: i32,
    dist: f32,
};

/// `RefObstacleCircle`: a copy of `dtObstacleCircle`.
pub const ObstacleCircle = extern struct {
    p: [3]f32,
    vel: [3]f32,
    dvel: [3]f32,
    rad: f32,
    dp: [3]f32,
    np: [3]f32,
};

/// `RefObstacleSegment`: a copy of `dtObstacleSegment`; `touch` is 0 or 1.
pub const ObstacleSegment = extern struct {
    p: [3]f32,
    q: [3]f32,
    touch: i32,
};

/// `RefCrowdDebugInfo`. `vod` goes in, the other two come out.
pub const CrowdDebugInfo = extern struct {
    /// Out: the traced agent's optimisation segment, zero if it was skipped.
    opt_start: [3]f32,
    opt_end: [3]f32,
    /// In: where the traced agent's velocity sampling is recorded, or null.
    vod: ?*ObstacleAvoidanceDebugData,
};

/// `RefAgentSnapshot`: everything observable about one agent. The shim zeroes
/// it before filling it.
pub const AgentSnapshot = extern struct {
    params: CrowdAgentParams,
    /// The first `min(corridor_npath, max_corridor_path)` polygons.
    corridor_path: [max_corridor_path]core.PolyRef,
    corner_polys: [max_corners]core.PolyRef,
    target_ref: core.PolyRef,
    anim_poly_ref: core.PolyRef,
    corridor_pos: [3]f32,
    corridor_target: [3]f32,
    boundary_center: [3]f32,
    /// The first `min(boundary_nsegments, max_boundary_segments)` segments,
    /// start then end.
    boundary_segments: [max_boundary_segments][6]f32,
    topology_opt_time: f32,
    neis: [max_neighbours]CrowdNeighbour,
    desired_speed: f32,
    npos: [3]f32,
    disp: [3]f32,
    dvel: [3]f32,
    nvel: [3]f32,
    vel: [3]f32,
    corner_verts: [max_corners * 3]f32,
    target_pos: [3]f32,
    target_replan_time: f32,
    /// The `anim_*` fields other than `anim_active` are filled only while
    /// `anim_active`: upstream leaves an animation slot uninitialised beyond
    /// its flag until an animation starts.
    anim_init_pos: [3]f32,
    anim_start_pos: [3]f32,
    anim_end_pos: [3]f32,
    anim_t: f32,
    anim_tmax: f32,
    target_pathq_ref: PathQueueRef,
    corridor_npath: i32,
    boundary_nsegments: i32,
    nneis: i32,
    ncorners: i32,
    corner_flags: [max_corners]u8,
    active: bool,
    state: u8,
    partial: bool,
    target_state: u8,
    target_replan: bool,
    anim_active: bool,
    reserved: [6]u8,
};

// ---------------------------------------------------------------------------
// DetourProximityGrid.h
// ---------------------------------------------------------------------------

pub extern fn ref_prox_grid_alloc() ?*ProximityGrid;
pub extern fn ref_prox_grid_free(grid: ?*ProximityGrid) void;
pub extern fn ref_prox_grid_init(grid: *ProximityGrid, pool_size: i32, cell_size: f32) bool;
pub extern fn ref_prox_grid_clear(grid: *ProximityGrid) void;
pub extern fn ref_prox_grid_add_item(
    grid: *ProximityGrid,
    id: u16,
    minx: f32,
    miny: f32,
    maxx: f32,
    maxy: f32,
) void;
pub extern fn ref_prox_grid_query_items(
    grid: *const ProximityGrid,
    minx: f32,
    miny: f32,
    maxx: f32,
    maxy: f32,
    ids: [*]u16,
    max_ids: i32,
) i32;
pub extern fn ref_prox_grid_get_item_count_at(grid: *const ProximityGrid, x: i32, y: i32) i32;
/// minx, miny, maxx, maxy in cells.
pub extern fn ref_prox_grid_get_bounds(grid: *const ProximityGrid) *const [4]i32;
pub extern fn ref_prox_grid_get_cell_size(grid: *const ProximityGrid) f32;

// ---------------------------------------------------------------------------
// DetourObstacleAvoidance.h: dtObstacleAvoidanceDebugData
// ---------------------------------------------------------------------------

pub extern fn ref_oa_debug_alloc() ?*ObstacleAvoidanceDebugData;
pub extern fn ref_oa_debug_free(data: ?*ObstacleAvoidanceDebugData) void;
/// `max_samples` must not be 0.
pub extern fn ref_oa_debug_init(data: *ObstacleAvoidanceDebugData, max_samples: i32) bool;
pub extern fn ref_oa_debug_reset(data: *ObstacleAvoidanceDebugData) void;
pub extern fn ref_oa_debug_add_sample(
    data: *ObstacleAvoidanceDebugData,
    vel: *const [3]f32,
    ssize: f32,
    pen: f32,
    vpen: f32,
    vcpen: f32,
    spen: f32,
    tpen: f32,
) void;
pub extern fn ref_oa_debug_normalize_samples(data: *ObstacleAvoidanceDebugData) void;
pub extern fn ref_oa_debug_get_sample_count(data: *const ObstacleAvoidanceDebugData) i32;
pub extern fn ref_oa_debug_get_sample_velocity(
    data: *const ObstacleAvoidanceDebugData,
    i: i32,
) *const [3]f32;
pub extern fn ref_oa_debug_get_sample_size(data: *const ObstacleAvoidanceDebugData, i: i32) f32;
pub extern fn ref_oa_debug_get_sample_penalty(data: *const ObstacleAvoidanceDebugData, i: i32) f32;
pub extern fn ref_oa_debug_get_sample_desired_velocity_penalty(
    data: *const ObstacleAvoidanceDebugData,
    i: i32,
) f32;
pub extern fn ref_oa_debug_get_sample_current_velocity_penalty(
    data: *const ObstacleAvoidanceDebugData,
    i: i32,
) f32;
pub extern fn ref_oa_debug_get_sample_preferred_side_penalty(
    data: *const ObstacleAvoidanceDebugData,
    i: i32,
) f32;
pub extern fn ref_oa_debug_get_sample_collision_time_penalty(
    data: *const ObstacleAvoidanceDebugData,
    i: i32,
) f32;

// ---------------------------------------------------------------------------
// DetourObstacleAvoidance.h: dtObstacleAvoidanceQuery
// ---------------------------------------------------------------------------

pub extern fn ref_oa_query_alloc() ?*ObstacleAvoidanceQuery;
pub extern fn ref_oa_query_free(query: ?*ObstacleAvoidanceQuery) void;
pub extern fn ref_oa_query_init(
    query: *ObstacleAvoidanceQuery,
    max_circles: i32,
    max_segments: i32,
) bool;
pub extern fn ref_oa_query_reset(query: *ObstacleAvoidanceQuery) void;
pub extern fn ref_oa_query_add_circle(
    query: *ObstacleAvoidanceQuery,
    pos: *const [3]f32,
    rad: f32,
    vel: *const [3]f32,
    dvel: *const [3]f32,
) void;
pub extern fn ref_oa_query_add_segment(
    query: *ObstacleAvoidanceQuery,
    p: *const [3]f32,
    q: *const [3]f32,
) void;
/// `debug` records every sample taken, or is null.
pub extern fn ref_oa_query_sample_velocity_grid(
    query: *ObstacleAvoidanceQuery,
    pos: *const [3]f32,
    rad: f32,
    vmax: f32,
    vel: *const [3]f32,
    dvel: *const [3]f32,
    nvel: *[3]f32,
    params: *const ObstacleAvoidanceParams,
    debug: ?*ObstacleAvoidanceDebugData,
) i32;
/// `debug` records every sample taken, or is null.
pub extern fn ref_oa_query_sample_velocity_adaptive(
    query: *ObstacleAvoidanceQuery,
    pos: *const [3]f32,
    rad: f32,
    vmax: f32,
    vel: *const [3]f32,
    dvel: *const [3]f32,
    nvel: *[3]f32,
    params: *const ObstacleAvoidanceParams,
    debug: ?*ObstacleAvoidanceDebugData,
) i32;
pub extern fn ref_oa_query_get_obstacle_circle_count(query: *const ObstacleAvoidanceQuery) i32;
/// Copies circle `i` into `out`. Returns false, leaving `out` alone, when `i`
/// is not in `0..count`; upstream does not check.
pub extern fn ref_oa_query_get_obstacle_circle(
    query: *ObstacleAvoidanceQuery,
    i: i32,
    out: *ObstacleCircle,
) bool;
pub extern fn ref_oa_query_get_obstacle_segment_count(query: *const ObstacleAvoidanceQuery) i32;
/// Copies segment `i` into `out`. Returns false, leaving `out` alone, when `i`
/// is not in `0..count`; upstream does not check.
pub extern fn ref_oa_query_get_obstacle_segment(
    query: *ObstacleAvoidanceQuery,
    i: i32,
    out: *ObstacleSegment,
) bool;

// ---------------------------------------------------------------------------
// DetourLocalBoundary.h
// ---------------------------------------------------------------------------

/// Upstream has no alloc function; the shim builds one with `dtAlloc` and
/// placement new. Null if memory runs out.
pub extern fn ref_local_boundary_create() ?*LocalBoundary;
pub extern fn ref_local_boundary_destroy(boundary: ?*LocalBoundary) void;
pub extern fn ref_local_boundary_reset(boundary: *LocalBoundary) void;
pub extern fn ref_local_boundary_update(
    boundary: *LocalBoundary,
    poly_ref: core.PolyRef,
    pos: *const [3]f32,
    collision_query_range: f32,
    navquery: *core.NavMeshQuery,
    filter: *const core.QueryFilter,
) void;
pub extern fn ref_local_boundary_is_valid(
    boundary: *LocalBoundary,
    navquery: *core.NavMeshQuery,
    filter: *const core.QueryFilter,
) bool;
pub extern fn ref_local_boundary_get_center(boundary: *const LocalBoundary) *const [3]f32;
pub extern fn ref_local_boundary_get_segment_count(boundary: *const LocalBoundary) i32;
/// The segment's start then end. `i` is not checked; the pointer goes stale on
/// the next `update` or `reset`.
pub extern fn ref_local_boundary_get_segment(boundary: *const LocalBoundary, i: i32) *const [6]f32;

// ---------------------------------------------------------------------------
// DetourPathCorridor.h
// ---------------------------------------------------------------------------

/// Upstream has no alloc function; the shim builds one with `dtAlloc` and
/// placement new. Null if memory runs out.
pub extern fn ref_corridor_create() ?*PathCorridor;
pub extern fn ref_corridor_destroy(corridor: ?*PathCorridor) void;
pub extern fn ref_corridor_init(corridor: *PathCorridor, max_path: i32) bool;
pub extern fn ref_corridor_reset(
    corridor: *PathCorridor,
    poly_ref: core.PolyRef,
    pos: *const [3]f32,
) void;
/// `corner_verts` takes three floats per corner.
pub extern fn ref_corridor_find_corners(
    corridor: *PathCorridor,
    corner_verts: [*]f32,
    corner_flags: [*]u8,
    corner_polys: [*]core.PolyRef,
    max_corners: i32,
    navquery: *core.NavMeshQuery,
    filter: *const core.QueryFilter,
) i32;
pub extern fn ref_corridor_optimize_path_visibility(
    corridor: *PathCorridor,
    next: *const [3]f32,
    path_optimization_range: f32,
    navquery: *core.NavMeshQuery,
    filter: *const core.QueryFilter,
) void;
pub extern fn ref_corridor_optimize_path_topology(
    corridor: *PathCorridor,
    navquery: *core.NavMeshQuery,
    filter: *const core.QueryFilter,
) bool;
pub extern fn ref_corridor_move_over_offmesh_connection(
    corridor: *PathCorridor,
    off_mesh_con_ref: core.PolyRef,
    refs: *[2]core.PolyRef,
    start_pos: *[3]f32,
    end_pos: *[3]f32,
    navquery: *core.NavMeshQuery,
) bool;
pub extern fn ref_corridor_fix_path_start(
    corridor: *PathCorridor,
    safe_ref: core.PolyRef,
    safe_pos: *const [3]f32,
) bool;
pub extern fn ref_corridor_trim_invalid_path(
    corridor: *PathCorridor,
    safe_ref: core.PolyRef,
    safe_pos: *const [3]f32,
    navquery: *core.NavMeshQuery,
    filter: *const core.QueryFilter,
) bool;
pub extern fn ref_corridor_is_valid(
    corridor: *PathCorridor,
    max_look_ahead: i32,
    navquery: *core.NavMeshQuery,
    filter: *const core.QueryFilter,
) bool;
pub extern fn ref_corridor_move_position(
    corridor: *PathCorridor,
    npos: *const [3]f32,
    navquery: *core.NavMeshQuery,
    filter: *const core.QueryFilter,
) bool;
pub extern fn ref_corridor_move_target_position(
    corridor: *PathCorridor,
    npos: *const [3]f32,
    navquery: *core.NavMeshQuery,
    filter: *const core.QueryFilter,
) bool;
pub extern fn ref_corridor_set_corridor(
    corridor: *PathCorridor,
    target: *const [3]f32,
    polys: [*]const core.PolyRef,
    npath: i32,
) void;
pub extern fn ref_corridor_get_pos(corridor: *const PathCorridor) *const [3]f32;
pub extern fn ref_corridor_get_target(corridor: *const PathCorridor) *const [3]f32;
pub extern fn ref_corridor_get_first_poly(corridor: *const PathCorridor) core.PolyRef;
pub extern fn ref_corridor_get_last_poly(corridor: *const PathCorridor) core.PolyRef;
/// `ref_corridor_get_path_count` polygons; null until `ref_corridor_init`. The
/// pointer goes stale when the corridor changes.
pub extern fn ref_corridor_get_path(corridor: *const PathCorridor) ?[*]const core.PolyRef;
pub extern fn ref_corridor_get_path_count(corridor: *const PathCorridor) i32;

pub extern fn ref_merge_corridor_start_moved(
    path: [*]core.PolyRef,
    npath: i32,
    max_path: i32,
    visited: [*]const core.PolyRef,
    nvisited: i32,
) i32;
pub extern fn ref_merge_corridor_end_moved(
    path: [*]core.PolyRef,
    npath: i32,
    max_path: i32,
    visited: [*]const core.PolyRef,
    nvisited: i32,
) i32;
pub extern fn ref_merge_corridor_start_shortcut(
    path: [*]core.PolyRef,
    npath: i32,
    max_path: i32,
    visited: [*]const core.PolyRef,
    nvisited: i32,
) i32;

// ---------------------------------------------------------------------------
// DetourPathQueue.h
// ---------------------------------------------------------------------------

/// Upstream has no alloc function; the shim builds one with `dtAlloc` and
/// placement new. Null if memory runs out.
pub extern fn ref_path_queue_create() ?*PathQueue;
pub extern fn ref_path_queue_destroy(queue: ?*PathQueue) void;
pub extern fn ref_path_queue_init(
    queue: *PathQueue,
    max_path_size: i32,
    max_search_node_count: i32,
    nav: *core.NavMesh,
) bool;
pub extern fn ref_path_queue_update(queue: *PathQueue, max_iters: i32) void;
/// The queue keeps `filter` until the request is consumed or expires.
pub extern fn ref_path_queue_request(
    queue: *PathQueue,
    start_ref: core.PolyRef,
    end_ref: core.PolyRef,
    start_pos: *const [3]f32,
    end_pos: *const [3]f32,
    filter: *const core.QueryFilter,
) PathQueueRef;
pub extern fn ref_path_queue_get_request_status(queue: *const PathQueue, request: PathQueueRef) u32;
pub extern fn ref_path_queue_get_path_result(
    queue: *PathQueue,
    request: PathQueueRef,
    path: [*]core.PolyRef,
    path_size: *i32,
    max_path: i32,
) u32;
pub extern fn ref_path_queue_get_nav_query(queue: *const PathQueue) ?*const core.NavMeshQuery;

// ---------------------------------------------------------------------------
// DetourCrowd.h
// ---------------------------------------------------------------------------

/// The values of `DetourCrowd.h`'s constants and enums, by name.
pub extern fn ref_const_crowdagent_max_neighbours() i32;
pub extern fn ref_const_crowdagent_max_corners() i32;
pub extern fn ref_const_crowd_max_obstavoidance_params() i32;
pub extern fn ref_const_crowd_max_query_filter_type() i32;
pub extern fn ref_const_crowdagent_state_invalid() i32;
pub extern fn ref_const_crowdagent_state_walking() i32;
pub extern fn ref_const_crowdagent_state_offmesh() i32;
pub extern fn ref_const_crowdagent_target_none() i32;
pub extern fn ref_const_crowdagent_target_failed() i32;
pub extern fn ref_const_crowdagent_target_valid() i32;
pub extern fn ref_const_crowdagent_target_requesting() i32;
pub extern fn ref_const_crowdagent_target_waiting_for_queue() i32;
pub extern fn ref_const_crowdagent_target_waiting_for_path() i32;
pub extern fn ref_const_crowdagent_target_velocity() i32;
pub extern fn ref_const_crowd_anticipate_turns() i32;
pub extern fn ref_const_crowd_obstacle_avoidance() i32;
pub extern fn ref_const_crowd_separation() i32;
pub extern fn ref_const_crowd_optimize_vis() i32;
pub extern fn ref_const_crowd_optimize_topo() i32;

/// `dtAllocCrowd`.
pub extern fn ref_crowd_alloc() ?*Crowd;
/// `dtFreeCrowd`.
pub extern fn ref_crowd_free(crowd: ?*Crowd) void;
/// The crowd keeps `nav` for as long as it lives.
pub extern fn ref_crowd_init(
    crowd: *Crowd,
    max_agents: i32,
    max_agent_radius: f32,
    nav: *core.NavMesh,
) bool;
/// `idx` is in `0..max_obstacle_avoidance_params`; others are ignored.
pub extern fn ref_crowd_set_obstacle_avoidance_params(
    crowd: *Crowd,
    idx: i32,
    params: *const ObstacleAvoidanceParams,
) void;
/// Null when `idx` is not in `0..max_obstacle_avoidance_params`.
pub extern fn ref_crowd_get_obstacle_avoidance_params(
    crowd: *const Crowd,
    idx: i32,
) ?*const ObstacleAvoidanceParams;
/// Null when `idx` is not in `0..agent count`. The pool does not move, but an
/// agent's contents change with every update and `ref_crowd_init` frees it.
pub extern fn ref_crowd_get_agent(crowd: *Crowd, idx: i32) ?*const CrowdAgent;
pub extern fn ref_crowd_get_editable_agent(crowd: *Crowd, idx: i32) ?*CrowdAgent;
pub extern fn ref_crowd_get_agent_count(crowd: *const Crowd) i32;
pub extern fn ref_crowd_add_agent(
    crowd: *Crowd,
    pos: *const [3]f32,
    params: *const CrowdAgentParams,
) i32;
pub extern fn ref_crowd_update_agent_parameters(
    crowd: *Crowd,
    idx: i32,
    params: *const CrowdAgentParams,
) void;
pub extern fn ref_crowd_remove_agent(crowd: *Crowd, idx: i32) void;
pub extern fn ref_crowd_request_move_target(
    crowd: *Crowd,
    idx: i32,
    poly_ref: core.PolyRef,
    pos: *const [3]f32,
) bool;
pub extern fn ref_crowd_request_move_velocity(crowd: *Crowd, idx: i32, vel: *const [3]f32) bool;
pub extern fn ref_crowd_reset_move_target(crowd: *Crowd, idx: i32) bool;
/// Differs from upstream, which fills an array of agent pointers: this writes
/// each active agent's index in the pool, in pool order, and returns how many
/// (at most `max_agents`). Returns -1 if the shim's temporary buffer cannot be
/// allocated.
pub extern fn ref_crowd_get_active_agents(crowd: *Crowd, out_indices: [*]i32, max_agents: i32) i32;
/// Runs one update. `debug_idx` is the position in the list of active agents
/// that upstream traces (the order `ref_crowd_get_active_agents` gives), not a
/// pool index. With a negative `debug_idx` and null `out` no debug info is
/// passed at all. Otherwise `out.vod` is handed to upstream as the traced
/// agent's debug data, and the optimisation segment is copied into `out`.
pub extern fn ref_crowd_update(crowd: *Crowd, dt: f32, debug_idx: i32, out: ?*CrowdDebugInfo) void;
/// Null when `i` is not in `0..max_query_filter_types`.
pub extern fn ref_crowd_get_filter(crowd: *const Crowd, i: i32) ?*const core.QueryFilter;
/// Null when `i` is not in `0..max_query_filter_types`.
pub extern fn ref_crowd_get_editable_filter(crowd: *Crowd, i: i32) ?*core.QueryFilter;
pub extern fn ref_crowd_get_query_half_extents(crowd: *const Crowd) *const [3]f32;
/// Same as `ref_crowd_get_query_half_extents`.
pub extern fn ref_crowd_get_query_extents(crowd: *const Crowd) *const [3]f32;
pub extern fn ref_crowd_get_velocity_sample_count(crowd: *const Crowd) i32;
/// Null before `ref_crowd_init`.
pub extern fn ref_crowd_get_grid(crowd: *const Crowd) ?*const ProximityGrid;
pub extern fn ref_crowd_get_path_queue(crowd: *const Crowd) ?*const PathQueue;
/// Null before `ref_crowd_init`.
pub extern fn ref_crowd_get_nav_mesh_query(crowd: *const Crowd) ?*const core.NavMeshQuery;
/// The agent's own corridor and boundary, to drive with the `ref_corridor_*`
/// and `ref_local_boundary_*` functions. They live and die with the crowd.
pub extern fn ref_crowd_agent_corridor(agent: *const CrowdAgent) *PathCorridor;
pub extern fn ref_crowd_agent_boundary(agent: *const CrowdAgent) *LocalBoundary;
/// Zeroes `out`, then copies every observable field of agent `idx` into it.
/// Returns false, leaving `out` zeroed, when `idx` is not in `0..agent count`.
pub extern fn ref_crowd_agent_snapshot(crowd: *const Crowd, idx: i32, out: *AgentSnapshot) bool;

// ---------------------------------------------------------------------------
// Layout checks: compare with `@sizeOf` and `@offsetOf` of the mirrors above.
// ---------------------------------------------------------------------------

pub extern fn ref_sizeof_obstacle_avoidance_params() usize;
pub extern fn ref_offsetof_obstacle_avoidance_params_vel_bias() usize;
pub extern fn ref_offsetof_obstacle_avoidance_params_weight_des_vel() usize;
pub extern fn ref_offsetof_obstacle_avoidance_params_weight_cur_vel() usize;
pub extern fn ref_offsetof_obstacle_avoidance_params_weight_side() usize;
pub extern fn ref_offsetof_obstacle_avoidance_params_weight_toi() usize;
pub extern fn ref_offsetof_obstacle_avoidance_params_horiz_time() usize;
pub extern fn ref_offsetof_obstacle_avoidance_params_grid_size() usize;
pub extern fn ref_offsetof_obstacle_avoidance_params_adaptive_divs() usize;
pub extern fn ref_offsetof_obstacle_avoidance_params_adaptive_rings() usize;
pub extern fn ref_offsetof_obstacle_avoidance_params_adaptive_depth() usize;

pub extern fn ref_sizeof_crowd_agent_params() usize;
pub extern fn ref_offsetof_crowd_agent_params_radius() usize;
pub extern fn ref_offsetof_crowd_agent_params_height() usize;
pub extern fn ref_offsetof_crowd_agent_params_max_acceleration() usize;
pub extern fn ref_offsetof_crowd_agent_params_max_speed() usize;
pub extern fn ref_offsetof_crowd_agent_params_collision_query_range() usize;
pub extern fn ref_offsetof_crowd_agent_params_path_optimization_range() usize;
pub extern fn ref_offsetof_crowd_agent_params_separation_weight() usize;
pub extern fn ref_offsetof_crowd_agent_params_update_flags() usize;
pub extern fn ref_offsetof_crowd_agent_params_obstacle_avoidance_type() usize;
pub extern fn ref_offsetof_crowd_agent_params_query_filter_type() usize;
pub extern fn ref_offsetof_crowd_agent_params_user_data() usize;

pub extern fn ref_sizeof_crowd_neighbour() usize;
pub extern fn ref_sizeof_obstacle_circle() usize;
pub extern fn ref_sizeof_obstacle_segment() usize;
pub extern fn ref_sizeof_crowd_debug_info() usize;
pub extern fn ref_sizeof_agent_snapshot() usize;
