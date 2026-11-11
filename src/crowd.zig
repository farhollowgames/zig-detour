//! The crowd: a pool of agents that each follow a path corridor across the
//! navigation mesh, steer toward the next corner of it, avoid each other
//! and the mesh's walls, and cross off-mesh connections.
//!
//! One `Crowd.update` call moves every active agent by one time step. The
//! steps run in a fixed order over the agents in pool order, so the same
//! calls on the same mesh give the same floats every time.

const std = @import("std");
const common = @import("common.zig");
const Status = @import("status.zig").Status;

const nav_mesh = @import("nav_mesh.zig");
const NavMesh = nav_mesh.NavMesh;
const PolyRef = nav_mesh.PolyRef;
const StraightPathFlags = nav_mesh.StraightPathFlags;

const nav_mesh_query = @import("nav_mesh_query.zig");
const NavMeshQuery = nav_mesh_query.NavMeshQuery;
const QueryFilter = nav_mesh_query.QueryFilter;

const obstacle_avoidance = @import("obstacle_avoidance.zig");
const ObstacleAvoidanceQuery = obstacle_avoidance.ObstacleAvoidanceQuery;
const ObstacleAvoidanceParams = obstacle_avoidance.ObstacleAvoidanceParams;
const ObstacleAvoidanceDebug = obstacle_avoidance.ObstacleAvoidanceDebug;

const ProximityGrid = @import("proximity_grid.zig").ProximityGrid;
const LocalBoundary = @import("local_boundary.zig").LocalBoundary;
const PathCorridor = @import("path_corridor.zig").PathCorridor;

const path_queue = @import("path_queue.zig");
const PathQueue = path_queue.PathQueue;
const PathQueueRef = path_queue.PathQueueRef;
const path_queue_invalid = path_queue.path_queue_invalid;

/// The most neighbours an agent takes into account when steering.
pub const agent_neighbour_count_max = 6;

/// The most corners of its path an agent looks ahead. The crowd steers by
/// the first two, so one fewer than this is of use.
pub const agent_corner_count_max = 4;

/// The number of obstacle avoidance configurations a crowd holds.
pub const obstacle_avoidance_params_count_max = 8;

/// The number of query filters a crowd holds.
pub const query_filter_type_count_max = 16;

/// The most path search iterations the path queue runs in one update.
const iterations_per_update_max: i32 = 100;

/// The search node pool of the path queue's query, for full path searches.
const path_queue_node_count_max: i32 = 4096;

/// The search node pool of the crowd's own query. It only runs local
/// searches, which need few nodes.
const common_node_count_max: i32 = 512;

/// The most polygons a path handed back by the path queue holds, and the
/// length of every agent's corridor.
const path_result_count_max: i32 = 256;

/// The most agents per update that are handed to the path queue.
const path_queue_agent_count_max = 8;

/// The most agents per update whose corridor topology is optimized.
const topology_optimization_agent_count_max = 1;

/// The seconds an agent waits between topology optimizations.
const topology_optimization_time_threshold: f32 = 0.5;

/// The most agents the proximity grid is asked for around one agent.
const neighbour_query_count_max = 32;

/// How much of the overlap between two agents one collision pass removes.
const collision_resolve_factor: f32 = 0.7;

/// How many corridor polygons ahead an agent's path is checked each update.
const check_lookahead: i32 = 10;

/// The seconds after a replan before a path that ends short of its target
/// is replanned again.
const target_replan_delay: f32 = 1.0;

/// The polygons of the quick search a new move request starts with.
const request_path_count_max = 32;

/// The iterations of that quick search.
const request_iteration_count_max: i32 = 20;

/// Whether velocities are sampled on the adaptive pattern rather than the
/// grid.
const adaptive_sampling = true;

/// One of an agent's neighbours.
pub const CrowdNeighbour = struct {
    /// The neighbour's index in the crowd's agent pool.
    idx: i32,
    /// The squared xz distance between the agent and the neighbour.
    dist_sqr: f32,
};

/// The kind of navigation mesh polygon an agent is on.
pub const CrowdAgentTraversal = enum(u8) {
    /// The agent is not on the navigation mesh, and is left out of updates.
    invalid = 0,
    /// The agent is walking a ground polygon.
    walking = 1,
    /// The agent is crossing an off-mesh connection.
    off_mesh = 2,
};

/// Which parts of the steering an agent uses, in the bit order of upstream
/// Detour's `UpdateFlags`.
pub const UpdateFlags = packed struct(u8) {
    /// Cut corners by steering toward the corner after the next one too.
    anticipate_turns: bool = false,
    /// Sample a velocity that avoids neighbours and walls.
    obstacle_avoidance: bool = false,
    /// Push away from neighbours within the collision query range.
    separation: bool = false,
    /// Shortcut the corridor to the corner after the next when it is
    /// visible, by `PathCorridor.optimizePathVisibility`.
    optimize_visibility: bool = false,
    /// Now and then replace the corridor's start with a shorter way round,
    /// by `PathCorridor.optimizePathTopology`.
    optimize_topology: bool = false,
    reserved: u3 = 0,
};

/// How an agent moves and steers.
pub const CrowdAgentParams = struct {
    /// At least 0.
    radius: f32,
    /// Above 0.
    height: f32,
    /// The greatest change of velocity per second, at least 0.
    acceleration_max: f32,
    /// At least 0.
    speed_max: f32,
    /// How near another agent or a wall must be to count for steering,
    /// above 0. Often set from the radius, such as 8 radii.
    collision_query_range: f32,
    /// How far ahead `UpdateFlags.optimize_visibility` looks, above 0.
    /// Often set from the radius, such as 30 radii.
    path_optimization_range: f32,
    /// How hard `UpdateFlags.separation` pushes away from neighbours, at
    /// least 0.
    separation_weight: f32,
    update_flags: UpdateFlags,
    /// Which of the crowd's avoidance configurations the agent uses, below
    /// `obstacle_avoidance_params_count_max`.
    obstacle_avoidance_type: u8,
    /// Which of the crowd's query filters the agent uses, below
    /// `query_filter_type_count_max`.
    query_filter_type: u8,
    /// The caller's own pointer, never read by the crowd.
    user_data: ?*anyopaque = null,
};

/// How far an agent's request to move has got.
pub const MoveRequestStage = enum(u8) {
    /// The agent has no target.
    none = 0,
    /// No path to the target was found.
    failed = 1,
    /// The corridor leads toward the target.
    valid = 2,
    /// A target was set and no path has been looked for yet.
    requesting = 3,
    /// The quick search fell short and the agent waits for the path queue
    /// to take its request.
    waiting_for_queue = 4,
    /// The path queue is searching for the agent's path.
    waiting_for_path = 5,
    /// The agent moves at the velocity in `CrowdAgent.target_pos` and has
    /// no path.
    velocity = 6,
};

/// One agent of a crowd's pool.
pub const CrowdAgent = struct {
    /// False when this pool entry is unused; the other fields are then
    /// stale.
    active: bool,
    state: CrowdAgentTraversal,
    /// True when the corridor is valid and does not reach the target.
    partial: bool,
    /// The polygons the agent walks through to its target.
    corridor: PathCorridor,
    /// The walls near the agent.
    boundary: LocalBoundary,
    /// The seconds since the corridor's topology was last optimized.
    topology_optimization_time: f32,
    /// The agent's neighbours, nearest first; `neighbour_count` are in use.
    neighbours: [agent_neighbour_count_max]CrowdNeighbour,
    neighbour_count: i32,
    desired_speed: f32,
    position: [3]f32,
    /// Scratch for the collision passes: the push away from overlapping
    /// neighbours.
    displacement: [3]f32,
    /// The velocity the agent wants, worked out from its path each update.
    desired_velocity: [3]f32,
    /// The desired velocity after obstacle avoidance, worked out each
    /// update.
    new_velocity: [3]f32,
    /// The agent's velocity, which follows `new_velocity` as fast as the
    /// greatest acceleration allows.
    velocity: [3]f32,
    params: CrowdAgentParams,
    /// The next corners of the agent's path; `corner_count` of each of the
    /// three arrays are in use.
    corner_verts: [agent_corner_count_max][3]f32,
    corner_flags: [agent_corner_count_max]StraightPathFlags,
    /// The polygon entered at each corner.
    corner_polys: [agent_corner_count_max]PolyRef,
    corner_count: i32,
    target_state: MoveRequestStage,
    /// The polygon of the requested target.
    target_ref: PolyRef,
    /// The requested target, or the requested velocity when `target_state`
    /// is `velocity`.
    target_pos: [3]f32,
    /// The agent's request in the path queue.
    target_path_queue_ref: PathQueueRef,
    /// True when the request replans toward a target the agent already had.
    target_replan: bool,
    /// The seconds since the target was last planned for.
    target_replan_time: f32,

    /// An unused pool entry: every field zero, with no corridor memory.
    const unused: CrowdAgent = .{
        .active = false,
        .state = .invalid,
        .partial = false,
        .corridor = .empty,
        .boundary = .empty,
        .topology_optimization_time = 0,
        .neighbours = @splat(.{ .idx = 0, .dist_sqr = 0 }),
        .neighbour_count = 0,
        .desired_speed = 0,
        .position = .{ 0, 0, 0 },
        .displacement = .{ 0, 0, 0 },
        .desired_velocity = .{ 0, 0, 0 },
        .new_velocity = .{ 0, 0, 0 },
        .velocity = .{ 0, 0, 0 },
        .params = .{
            .radius = 0,
            .height = 0,
            .acceleration_max = 0,
            .speed_max = 0,
            .collision_query_range = 0,
            .path_optimization_range = 0,
            .separation_weight = 0,
            .update_flags = .{},
            .obstacle_avoidance_type = 0,
            .query_filter_type = 0,
            .user_data = null,
        },
        .corner_verts = @splat(.{ 0, 0, 0 }),
        .corner_flags = @splat(.{}),
        .corner_polys = @splat(0),
        .corner_count = 0,
        .target_state = .none,
        .target_ref = 0,
        .target_pos = .{ 0, 0, 0 },
        .target_path_queue_ref = path_queue_invalid,
        .target_replan = false,
        .target_replan_time = 0,
    };
};

/// An agent's crossing of an off-mesh connection: it walks from where it
/// was to the connection's start, then to its end.
pub const CrowdAgentAnimation = struct {
    /// True while the agent is crossing; the other fields are stale when
    /// false.
    active: bool,
    /// Where the agent was when the crossing began.
    init_pos: [3]f32,
    /// The connection's end points, in the order crossed.
    start_pos: [3]f32,
    end_pos: [3]f32,
    /// The polygon the agent lands on.
    poly_ref: PolyRef,
    /// The seconds since the crossing began.
    t: f32,
    /// The seconds the crossing takes.
    t_max: f32,

    const unused: CrowdAgentAnimation = .{
        .active = false,
        .init_pos = .{ 0, 0, 0 },
        .start_pos = .{ 0, 0, 0 },
        .end_pos = .{ 0, 0, 0 },
        .poly_ref = 0,
        .t = 0,
        .t_max = 0,
    };
};

/// What `Crowd.update` records about one agent, for drawing its choices.
pub const CrowdAgentDebugInfo = struct {
    /// Which agent to record, as its place among the active agents in pool
    /// order, which is its pool index only while no earlier entry is
    /// unused.
    idx: i32,
    /// The ends of the agent's visibility shortcut, zero when it took none.
    opt_start: [3]f32,
    opt_end: [3]f32,
    /// Takes the velocities sampled for the agent, when set.
    vod: ?*ObstacleAvoidanceDebug,
};

/// How far `t` has got from `t0` to `t1`, limited to 0..1.
inline fn tween(t: f32, t0: f32, t1: f32) f32 {
    return common.clamp(f32, (t - t0) / (t1 - t0), 0.0, 1.0);
}

/// Moves the agent's velocity toward its new velocity, by no more than its
/// acceleration allows in `dt` seconds, and its position by that velocity.
fn integrate(ag: *CrowdAgent, dt: f32) void {
    // Fake dynamic constraint.
    const delta_max = ag.params.acceleration_max * dt;
    var dv = common.vsub(ag.new_velocity, ag.velocity);
    const ds = common.vlen(dv);
    if (ds > delta_max) dv = common.vscale(dv, delta_max / ds);
    ag.velocity = common.vadd(ag.velocity, dv);

    // Integrate.
    if (common.vlen(ag.velocity) > 0.0001) {
        ag.position = common.vmad(ag.position, ag.velocity, dt);
    } else {
        ag.velocity = .{ 0, 0, 0 };
    }
}

/// Whether the agent's last corner starts an off-mesh connection and lies
/// within `radius` of the agent.
fn overOffmeshConnection(ag: *const CrowdAgent, radius: f32) bool {
    if (ag.corner_count == 0) return false;

    const last: usize = @intCast(ag.corner_count - 1);
    if (ag.corner_flags[last].off_mesh_connection) {
        const dist_sqr = common.vdist2dSqr(ag.position, ag.corner_verts[last]);
        if (dist_sqr < radius * radius) return true;
    }

    return false;
}

/// The xz distance to the path's end when the agent's last corner is that
/// end and nearer than `range`; else `range`.
fn getDistanceToGoal(ag: *const CrowdAgent, range: f32) f32 {
    if (ag.corner_count == 0) return range;

    const last: usize = @intCast(ag.corner_count - 1);
    if (ag.corner_flags[last].end) {
        return common.min(f32, common.vdist2d(ag.position, ag.corner_verts[last]), range);
    }

    return range;
}

/// The unit xz direction toward the agent's next corner, bent toward the
/// corner after it so the turn starts early. Zero without corners.
fn calcSmoothSteerDirection(ag: *const CrowdAgent) [3]f32 {
    if (ag.corner_count == 0) return .{ 0, 0, 0 };

    const ip0: usize = 0;
    const ip1: usize = @intCast(@min(1, ag.corner_count - 1));
    const p0 = ag.corner_verts[ip0];
    const p1 = ag.corner_verts[ip1];

    var dir0 = common.vsub(p0, ag.position);
    var dir1 = common.vsub(p1, ag.position);
    dir0[1] = 0;
    dir1[1] = 0;

    const len0 = common.vlen(dir0);
    const len1 = common.vlen(dir1);
    if (len1 > 0.001) dir1 = common.vscale(dir1, 1.0 / len1);

    const dir: [3]f32 = .{
        dir0[0] - dir1[0] * len0 * 0.5,
        0,
        dir0[2] - dir1[2] * len0 * 0.5,
    };

    return common.vnormalize(dir);
}

/// The unit xz direction toward the agent's next corner. Zero without
/// corners.
fn calcStraightSteerDirection(ag: *const CrowdAgent) [3]f32 {
    if (ag.corner_count == 0) return .{ 0, 0, 0 };

    var dir = common.vsub(ag.corner_verts[0], ag.position);
    dir[1] = 0;
    return common.vnormalize(dir);
}

/// Puts neighbour `idx` into the first `count` entries of `neighbours`,
/// which are ordered nearest first, ahead of the entries as far or farther.
/// When the list is full the farthest entry drops out. Returns the new
/// count.
fn addNeighbour(idx: i32, dist_sqr: f32, neighbours: []CrowdNeighbour, count: i32) i32 {
    const count_max: i32 = @intCast(neighbours.len);

    // Insert the neighbour based on the distance.
    var entry: usize = undefined;
    if (count == 0) {
        entry = 0;
    } else if (dist_sqr >= neighbours[@intCast(count - 1)].dist_sqr) {
        if (count >= count_max) return count;
        entry = @intCast(count);
    } else {
        var i: i32 = 0;
        while (i < count) : (i += 1) {
            if (dist_sqr <= neighbours[@intCast(i)].dist_sqr) break;
        }

        const tgt = i + 1;
        const n = @min(count - i, count_max - tgt);

        std.debug.assert(tgt + n <= count_max);

        if (n > 0) {
            const from: usize = @intCast(i);
            const moved: usize = @intCast(n);
            @memmove(neighbours[from + 1 ..][0..moved], neighbours[from..][0..moved]);
        }
        entry = @intCast(i);
    }

    neighbours[entry] = .{ .idx = idx, .dist_sqr = dist_sqr };

    return @min(count + 1, count_max);
}

/// Fills `result` with the agents of `agents` near `pos`, nearest first:
/// those the grid finds within `range` in xz whose heights overlap, leaving
/// out `skip`. Each entry's `idx` is the agent's place in `agents`. Returns
/// how many were written.
fn getNeighbours(
    pos: [3]f32,
    height: f32,
    range: f32,
    skip: *const CrowdAgent,
    result: []CrowdNeighbour,
    agents: []const *CrowdAgent,
    grid: *const ProximityGrid,
) i32 {
    var n: i32 = 0;

    var ids: [neighbour_query_count_max]u16 = undefined;
    const id_count = grid.queryItems(
        pos[0] - range,
        pos[2] - range,
        pos[0] + range,
        pos[2] + range,
        &ids,
    );

    for (ids[0..@intCast(id_count)]) |id| {
        const ag: *const CrowdAgent = agents[id];

        if (ag == skip) continue;

        // Check for overlap.
        var diff = common.vsub(pos, ag.position);
        if (@abs(diff[1]) >= (height + ag.params.height) / 2.0) continue;
        diff[1] = 0;
        const dist_sqr = common.vlenSqr(diff);
        if (dist_sqr > common.sqr(f32, range)) continue;

        n = addNeighbour(id, dist_sqr, result, n);
    }
    return n;
}

/// Puts `new_agent` into the first `count` entries of `agents`, which are
/// ordered by the longest time since a topology optimization first, ahead
/// of the entries that waited as long or less. When the list is full the
/// last entry drops out. Returns the new count.
fn addToOptQueue(new_agent: *CrowdAgent, agents: []*CrowdAgent, count: i32) i32 {
    const count_max: i32 = @intCast(agents.len);
    const time = new_agent.topology_optimization_time;

    // Insert the agent based on the greatest time.
    var entry: usize = undefined;
    if (count == 0) {
        entry = 0;
    } else if (time <= agents[@intCast(count - 1)].topology_optimization_time) {
        if (count >= count_max) return count;
        entry = @intCast(count);
    } else {
        var i: i32 = 0;
        while (i < count) : (i += 1) {
            if (time >= agents[@intCast(i)].topology_optimization_time) break;
        }

        const tgt = i + 1;
        const n = @min(count - i, count_max - tgt);

        std.debug.assert(tgt + n <= count_max);

        if (n > 0) {
            const from: usize = @intCast(i);
            const moved: usize = @intCast(n);
            @memmove(agents[from + 1 ..][0..moved], agents[from..][0..moved]);
        }
        entry = @intCast(i);
    }

    agents[entry] = new_agent;

    return @min(count + 1, count_max);
}

/// Puts `new_agent` into the first `count` entries of `agents`, which are
/// ordered by the longest time since a replan first, ahead of the entries
/// that waited as long or less. When the list is full the last entry drops
/// out. Returns the new count.
fn addToPathQueue(new_agent: *CrowdAgent, agents: []*CrowdAgent, count: i32) i32 {
    const count_max: i32 = @intCast(agents.len);
    const time = new_agent.target_replan_time;

    // Insert the agent based on the greatest time.
    var entry: usize = undefined;
    if (count == 0) {
        entry = 0;
    } else if (time <= agents[@intCast(count - 1)].target_replan_time) {
        if (count >= count_max) return count;
        entry = @intCast(count);
    } else {
        var i: i32 = 0;
        while (i < count) : (i += 1) {
            if (time >= agents[@intCast(i)].target_replan_time) break;
        }

        const tgt = i + 1;
        const n = @min(count - i, count_max - tgt);

        std.debug.assert(tgt + n <= count_max);

        if (n > 0) {
            const from: usize = @intCast(i);
            const moved: usize = @intCast(n);
            @memmove(agents[from + 1 ..][0..moved], agents[from..][0..moved]);
        }
        entry = @intCast(i);
    }

    agents[entry] = new_agent;

    return @min(count + 1, count_max);
}

/// A pool of agents moved across one navigation mesh, each along its own
/// path, steering clear of the others and of the mesh's walls.
///
/// A crowd starts as `empty`; `init` readies it and `deinit` ends it. Then
/// set the avoidance configurations with `setObstacleAvoidanceParams`, add
/// agents with `addAgent`, give them targets with `requestMoveTarget`, and
/// call `update` every frame.
///
/// The crowd owns each agent's position: to place an agent elsewhere,
/// remove it and add it again. A corridor holds 256 polygons at most, so
/// the crowd plans local movement, and a far target is reached through
/// targets on the way.
///
/// A crowd with agents stays at one address: the path requests under way
/// hold pointers to its filters.
pub const Crowd = struct {
    agent_count_max: i32,
    /// The agent pool, `agent_count_max` long.
    agents: []CrowdAgent,
    /// Scratch for `update`: the active agents in pool order.
    active_agents: []*CrowdAgent,
    /// Each pool entry's off-mesh crossing.
    agent_animations: []CrowdAgentAnimation,

    path_queue: PathQueue,

    obstacle_query_params: [obstacle_avoidance_params_count_max]ObstacleAvoidanceParams,
    obstacle_query: ?*ObstacleAvoidanceQuery,

    grid: ?*ProximityGrid,

    /// Scratch for merging a path queue result into a corridor.
    path_result: []PolyRef,
    path_result_count_max: i32,

    /// The half extents of the box searched for the polygon nearest an
    /// agent or a target.
    agent_placement_half_extents: [3]f32,

    filters: [query_filter_type_count_max]QueryFilter,

    agent_radius_max: f32,

    /// The velocities sampled by the latest `update`.
    velocity_sample_count: i32,

    nav_query: ?*NavMeshQuery,

    /// A crowd that holds no memory and no agents, for a field to hold
    /// before `init`. Its filters are the default filter.
    pub const empty: Crowd = .{
        .agent_count_max = 0,
        .agents = &.{},
        .active_agents = &.{},
        .agent_animations = &.{},
        .path_queue = .empty,
        .obstacle_query_params = @splat(std.mem.zeroes(ObstacleAvoidanceParams)),
        .obstacle_query = null,
        .grid = null,
        .path_result = &.{},
        .path_result_count_max = 0,
        .agent_placement_half_extents = .{ 0, 0, 0 },
        .filters = @splat(.{}),
        .agent_radius_max = 0,
        .velocity_sample_count = 0,
        .nav_query = null,
    };

    /// Readies the crowd for up to `agent_count_max` agents, at least 1, of
    /// a radius up to `agent_radius_max`, above 0, on `nav`. Returns false
    /// when memory runs out, leaving the crowd without memory or agents.
    ///
    /// The crowd is `empty` or was readied before; one readied before is
    /// freed first, and `gpa` must then be the allocator of that call. The
    /// filters are left as they are, so those set earlier stay.
    ///
    /// The crowd keeps `nav`'s address: the mesh must outlive the crowd and
    /// stay where it is.
    pub fn init(
        self: *Crowd,
        gpa: std.mem.Allocator,
        agent_count_max: i32,
        agent_radius_max: f32,
        nav: *NavMesh,
    ) bool {
        self.purge(gpa);

        if (!self.allocate(gpa, agent_count_max, agent_radius_max, nav)) {
            self.purge(gpa);
            return false;
        }
        return true;
    }

    /// The work of `init` on a purged crowd. Everything is put in the crowd
    /// before it is set up, and a part that fails to set up is left
    /// holding nothing, so on failure `purge` frees what there is.
    fn allocate(
        self: *Crowd,
        gpa: std.mem.Allocator,
        agent_count_max: i32,
        agent_radius_max: f32,
        nav: *NavMesh,
    ) bool {
        // A negative count asks for more memory than there is.
        const agent_len = std.math.cast(usize, agent_count_max) orelse return false;
        const grid_pool_size = std.math.mul(i32, agent_count_max, 4) catch return false;

        self.agent_radius_max = agent_radius_max;

        // Larger than the agent radius because it is also used for agent
        // recovery.
        self.agent_placement_half_extents = .{
            self.agent_radius_max * 2.0,
            self.agent_radius_max * 1.5,
            self.agent_radius_max * 2.0,
        };

        const grid = gpa.create(ProximityGrid) catch return false;
        grid.* = .empty;
        self.grid = grid;
        if (!grid.init(gpa, grid_pool_size, agent_radius_max * 3)) return false;

        const obstacle_query = gpa.create(ObstacleAvoidanceQuery) catch return false;
        obstacle_query.* = .empty;
        self.obstacle_query = obstacle_query;
        if (!obstacle_query.init(gpa, 6, 8)) return false;

        // Init the obstacle query params.
        for (&self.obstacle_query_params) |*params| {
            params.* = std.mem.zeroes(ObstacleAvoidanceParams);
            params.vel_bias = 0.4;
            params.weight_desired_velocity = 2.0;
            params.weight_current_velocity = 0.75;
            params.weight_side = 0.75;
            params.weight_time_of_impact = 2.5;
            params.horizon_time = 2.5;
            params.grid_size = 33;
            params.adaptive_divisions = 7;
            params.adaptive_rings = 2;
            params.adaptive_depth = 5;
        }

        // Allocate the temp buffer for merging paths.
        self.path_result = gpa.alloc(PolyRef, path_result_count_max) catch return false;
        self.path_result_count_max = path_result_count_max;

        if (!self.path_queue.init(gpa, path_result_count_max, path_queue_node_count_max, nav)) {
            return false;
        }

        self.agents = gpa.alloc(CrowdAgent, agent_len) catch return false;
        // Every entry is whole before any corridor is allocated, so `purge`
        // can go over all of them after a failure part way.
        @memset(self.agents, CrowdAgent.unused);
        self.agent_count_max = agent_count_max;

        self.active_agents = gpa.alloc(*CrowdAgent, agent_len) catch return false;

        self.agent_animations = gpa.alloc(CrowdAgentAnimation, agent_len) catch return false;
        @memset(self.agent_animations, CrowdAgentAnimation.unused);

        for (self.agents) |*ag| {
            if (!ag.corridor.init(gpa, path_result_count_max)) return false;
        }

        // The query is mostly used for local searches, so it needs no large
        // node pool.
        const nav_query = gpa.create(NavMeshQuery) catch return false;
        nav_query.* = .empty;
        self.nav_query = nav_query;
        if (nav_query.init(gpa, nav, common_node_count_max).failed) return false;

        return true;
    }

    /// Frees everything `init` allocated but the path queue and leaves the
    /// crowd without agents, its filters and avoidance configurations kept.
    /// The path queue frees its own memory when `init` readies it again,
    /// and keeps counting its request handles on from where it was.
    fn purge(self: *Crowd, gpa: std.mem.Allocator) void {
        for (self.agents) |*ag| ag.corridor.deinit(gpa);
        gpa.free(self.agents);
        self.agents = &.{};
        self.agent_count_max = 0;

        gpa.free(self.active_agents);
        self.active_agents = &.{};

        gpa.free(self.agent_animations);
        self.agent_animations = &.{};

        gpa.free(self.path_result);
        self.path_result = &.{};

        if (self.grid) |grid| {
            grid.deinit(gpa);
            gpa.destroy(grid);
        }
        self.grid = null;

        if (self.obstacle_query) |obstacle_query| {
            obstacle_query.deinit(gpa);
            gpa.destroy(obstacle_query);
        }
        self.obstacle_query = null;

        if (self.nav_query) |nav_query| {
            nav_query.deinit(gpa);
            gpa.destroy(nav_query);
        }
        self.nav_query = null;
    }

    /// Frees the crowd's memory. `gpa` is the allocator `init` was given.
    /// Every pointer the crowd handed out ends with it.
    pub fn deinit(self: *Crowd, gpa: std.mem.Allocator) void {
        self.purge(gpa);
        self.path_queue.deinit(gpa);
        self.* = undefined;
    }

    /// The index of `agent`, a pointer into the pool, in the pool.
    inline fn getAgentIndex(self: *const Crowd, agent: *const CrowdAgent) i32 {
        const offset = @intFromPtr(agent) - @intFromPtr(self.agents.ptr);
        return @intCast(offset / @sizeOf(CrowdAgent));
    }

    /// Sets avoidance configuration `idx`, which agents pick by
    /// `CrowdAgentParams.obstacle_avoidance_type`. An `idx` outside
    /// 0..`obstacle_avoidance_params_count_max` is ignored.
    pub fn setObstacleAvoidanceParams(
        self: *Crowd,
        idx: i32,
        params: *const ObstacleAvoidanceParams,
    ) void {
        if (idx >= 0 and idx < obstacle_avoidance_params_count_max) {
            self.obstacle_query_params[@intCast(idx)] = params.*;
        }
    }

    /// Avoidance configuration `idx`, or null for an `idx` outside
    /// 0..`obstacle_avoidance_params_count_max`. The pointer is into the
    /// crowd: it holds while the crowd stays where it is and is not
    /// deinitialized, and `setObstacleAvoidanceParams` and `init` change
    /// what it shows.
    pub fn getObstacleAvoidanceParams(
        self: *const Crowd,
        idx: i32,
    ) ?*const ObstacleAvoidanceParams {
        if (idx >= 0 and idx < obstacle_avoidance_params_count_max) {
            return &self.obstacle_query_params[@intCast(idx)];
        }
        return null;
    }

    /// The size of the agent pool: the most agents the crowd holds.
    pub fn getAgentCount(self: *const Crowd) i32 {
        return self.agent_count_max;
    }

    /// Pool entry `idx`, or null for an `idx` outside the pool. The entry
    /// may be unused: check `CrowdAgent.active`. The pointer is into the
    /// pool: `init` and `deinit` end it, and an entry freed by
    /// `removeAgent` is reused by a later `addAgent`.
    pub fn getAgent(self: *const Crowd, idx: i32) ?*const CrowdAgent {
        if (idx < 0 or idx >= self.agent_count_max) return null;
        return &self.agents[@intCast(idx)];
    }

    /// Pool entry `idx` for changing, as `getAgent` gives it for reading.
    pub fn getEditableAgent(self: *Crowd, idx: i32) ?*CrowdAgent {
        if (idx < 0 or idx >= self.agent_count_max) return null;
        return &self.agents[@intCast(idx)];
    }

    /// Replaces agent `idx`'s configuration. An `idx` outside the pool is
    /// ignored.
    pub fn updateAgentParameters(
        self: *Crowd,
        idx: i32,
        params: *const CrowdAgentParams,
    ) void {
        if (idx < 0 or idx >= self.agent_count_max) return;
        self.agents[@intCast(idx)].params = params.*;
    }

    /// Adds an agent at the point of the navigation mesh nearest `pos`,
    /// in the first unused pool entry. Returns the entry's index, or -1
    /// when the pool is full. An agent with no mesh near `pos` is added
    /// where asked, in the `invalid` state, and stays out of updates.
    ///
    /// Its corners and neighbours are filled in by the next `update`.
    pub fn addAgent(self: *Crowd, pos: [3]f32, params: *const CrowdAgentParams) i32 {
        // Find an empty entry.
        var idx: i32 = -1;
        for (self.agents, 0..) |*agent, i| {
            if (!agent.active) {
                idx = @intCast(i);
                break;
            }
        }
        if (idx == -1) return -1;

        const ag = &self.agents[@intCast(idx)];

        self.updateAgentParameters(idx, params);

        // Find the nearest position on the navmesh and place the agent
        // there.
        var nearest = pos;
        var ref: PolyRef = 0;
        const status = self.nav_query.?.findNearestPoly(
            pos,
            self.agent_placement_half_extents,
            &self.filters[ag.params.query_filter_type],
            &ref,
            &nearest,
            null,
        );
        if (status.failed) {
            nearest = pos;
            ref = 0;
        }

        ag.corridor.reset(ref, nearest);
        ag.boundary.reset();
        ag.partial = false;

        ag.topology_optimization_time = 0;
        ag.target_replan_time = 0;
        ag.neighbour_count = 0;

        ag.desired_velocity = .{ 0, 0, 0 };
        ag.new_velocity = .{ 0, 0, 0 };
        ag.velocity = .{ 0, 0, 0 };
        ag.position = nearest;

        ag.desired_speed = 0;

        ag.state = if (ref != 0) .walking else .invalid;

        ag.target_state = .none;

        ag.active = true;

        return idx;
    }

    /// Takes agent `idx` out of the crowd. Its pool entry stays, marked
    /// unused, for a later `addAgent` to take. An `idx` outside the pool is
    /// ignored.
    pub fn removeAgent(self: *Crowd, idx: i32) void {
        if (idx >= 0 and idx < self.agent_count_max) {
            self.agents[@intCast(idx)].active = false;
        }
    }

    /// Asks again for a path to a target agent `idx` already had, keeping
    /// what it can of the corridor. A zero `ref` fails the request.
    fn requestMoveTargetReplan(self: *Crowd, idx: i32, ref: PolyRef, pos: [3]f32) bool {
        if (idx < 0 or idx >= self.agent_count_max) return false;

        const ag = &self.agents[@intCast(idx)];

        // Initialize the request.
        ag.target_ref = ref;
        ag.target_pos = pos;
        ag.target_path_queue_ref = path_queue_invalid;
        ag.target_replan = true;
        ag.target_state = if (ag.target_ref != 0) .requesting else .failed;

        return true;
    }

    /// Sets agent `idx`'s target to `pos` on polygon `ref`. The next
    /// `update` starts looking for the path. Returns false for an `idx`
    /// outside the pool or a zero `ref`.
    pub fn requestMoveTarget(self: *Crowd, idx: i32, ref: PolyRef, pos: [3]f32) bool {
        if (idx < 0 or idx >= self.agent_count_max) return false;
        if (ref == 0) return false;

        const ag = &self.agents[@intCast(idx)];

        // Initialize the request.
        ag.target_ref = ref;
        ag.target_pos = pos;
        ag.target_path_queue_ref = path_queue_invalid;
        ag.target_replan = false;
        ag.target_state = if (ag.target_ref != 0) .requesting else .failed;

        return true;
    }

    /// Makes agent `idx` move at `vel` with no path, still held to the
    /// navigation mesh. Returns false for an `idx` outside the pool.
    pub fn requestMoveVelocity(self: *Crowd, idx: i32, vel: [3]f32) bool {
        if (idx < 0 or idx >= self.agent_count_max) return false;

        const ag = &self.agents[@intCast(idx)];

        // Initialize the request.
        ag.target_ref = 0;
        ag.target_pos = vel;
        ag.target_path_queue_ref = path_queue_invalid;
        ag.target_replan = false;
        ag.target_state = .velocity;

        return true;
    }

    /// Drops agent `idx`'s target or velocity request, so it stands.
    /// Returns false for an `idx` outside the pool.
    pub fn resetMoveTarget(self: *Crowd, idx: i32) bool {
        if (idx < 0 or idx >= self.agent_count_max) return false;

        const ag = &self.agents[@intCast(idx)];

        // Initialize the request.
        ag.target_ref = 0;
        ag.target_pos = .{ 0, 0, 0 };
        ag.desired_velocity = .{ 0, 0, 0 };
        ag.target_path_queue_ref = path_queue_invalid;
        ag.target_replan = false;
        ag.target_state = .none;

        return true;
    }

    /// Writes the active agents to `agents` in pool order, as many as fit,
    /// and returns how many it wrote. The pointers are into the pool, as
    /// `getAgent`'s are.
    pub fn getActiveAgents(self: *Crowd, agents: []*CrowdAgent) i32 {
        var n: usize = 0;
        for (self.agents) |*agent| {
            if (!agent.active) continue;
            if (n < agents.len) {
                agents[n] = agent;
                n += 1;
            }
        }
        return @intCast(n);
    }

    /// Moves every move request one stage on: runs the quick search of the
    /// new ones, hands those it did not satisfy to the path queue, runs
    /// the queue, and merges the paths it finished into the corridors.
    fn updateMoveRequest(self: *Crowd) void {
        const nav_query = self.nav_query.?;

        var queue: [path_queue_agent_count_max]*CrowdAgent = undefined;
        var queue_count: i32 = 0;

        // Fire off new requests.
        for (self.agents) |*ag| {
            if (!ag.active) continue;
            if (ag.state == .invalid) continue;
            if (ag.target_state == .none or ag.target_state == .velocity) continue;

            if (ag.target_state == .requesting) {
                // A view of the corridor, read only before `setCorridor`
                // below overwrites it.
                const path = ag.corridor.getPath();
                std.debug.assert(path.len != 0);

                const filter = &self.filters[ag.params.query_filter_type];

                var req_pos: [3]f32 = undefined;
                // The path to the request location.
                var req_path: [request_path_count_max]PolyRef = undefined;
                var req_path_count: usize = 0;

                // Quick search towards the goal.
                _ = nav_query.initSlicedFindPath(
                    path[0],
                    ag.target_ref,
                    ag.position,
                    ag.target_pos,
                    filter,
                    .{},
                );
                _ = nav_query.updateSlicedFindPath(request_iteration_count_max, null);
                var status: Status = .{};
                if (ag.target_replan) {
                    // Try to use the existing steady path during a replan
                    // if possible.
                    status = nav_query.finalizeSlicedFindPathPartial(
                        path,
                        &req_path,
                        &req_path_count,
                    );
                } else {
                    // Try to move towards the target when the goal changes.
                    status = nav_query.finalizeSlicedFindPath(&req_path, &req_path_count);
                }

                if (!status.failed and req_path_count > 0) {
                    // In progress or succeeded.
                    if (req_path[req_path_count - 1] != ag.target_ref) {
                        // Partial path, constrain the target position
                        // inside the last polygon.
                        status = nav_query.closestPointOnPoly(
                            req_path[req_path_count - 1],
                            ag.target_pos,
                            &req_pos,
                            null,
                        );
                        if (status.failed) req_path_count = 0;
                    } else {
                        req_pos = ag.target_pos;
                    }
                } else {
                    req_path_count = 0;
                }

                if (req_path_count == 0) {
                    // Could not find a path, start the request from the
                    // current location.
                    req_pos = ag.position;
                    req_path[0] = path[0];
                    req_path_count = 1;
                }

                ag.corridor.setCorridor(req_pos, req_path[0..req_path_count]);
                ag.boundary.reset();
                ag.partial = false;

                if (req_path[req_path_count - 1] == ag.target_ref) {
                    ag.target_state = .valid;
                    ag.target_replan_time = 0.0;
                } else {
                    // The path is longer or potentially unreachable, full
                    // plan.
                    ag.target_state = .waiting_for_queue;
                }
            }

            if (ag.target_state == .waiting_for_queue) {
                queue_count = addToPathQueue(ag, &queue, queue_count);
            }
        }

        for (queue[0..@intCast(queue_count)]) |ag| {
            ag.target_path_queue_ref = self.path_queue.request(
                ag.corridor.getLastPoly(),
                ag.target_ref,
                ag.corridor.getTarget(),
                ag.target_pos,
                &self.filters[ag.params.query_filter_type],
            );
            if (ag.target_path_queue_ref != path_queue_invalid) {
                ag.target_state = .waiting_for_path;
            }
        }

        // Update requests.
        self.path_queue.update(iterations_per_update_max);

        // Process path results.
        for (self.agents) |*ag| {
            if (!ag.active) continue;
            if (ag.target_state == .none or ag.target_state == .velocity) continue;

            if (ag.target_state == .waiting_for_path) {
                // Poll the path queue.
                var status = self.path_queue.getRequestStatus(ag.target_path_queue_ref);
                if (status.failed) {
                    // Path find failed, retry if the target location is
                    // still valid.
                    ag.target_path_queue_ref = path_queue_invalid;
                    ag.target_state = if (ag.target_ref != 0) .requesting else .failed;
                    ag.target_replan_time = 0.0;
                } else if (status.succeeded) {
                    // A view of the corridor, read only before
                    // `setCorridor` below overwrites it.
                    const path = ag.corridor.getPath();
                    const npath: i32 = @intCast(path.len);
                    std.debug.assert(npath != 0);

                    // Apply results.
                    var target_pos = ag.target_pos;

                    const res = self.path_result;
                    var valid = true;
                    var result_count: usize = 0;
                    status = self.path_queue.getPathResult(
                        ag.target_path_queue_ref,
                        res,
                        &result_count,
                    );
                    var nres: i32 = @intCast(result_count);
                    if (status.failed or nres == 0) valid = false;

                    ag.partial = status.partial_result;

                    // Merge the result and the existing path. The agent
                    // might have moved whilst the request was being
                    // processed, so the path may have changed. We assume
                    // that the end of the path is at the same location
                    // where the request was issued.

                    // The last ref in the old path should be the same as
                    // the location where the request was issued.
                    if (valid and path[@intCast(npath - 1)] != res[0]) valid = false;

                    if (valid) {
                        // Put the old path in front of the new path.
                        if (npath > 1) {
                            // Make space for the old path.
                            if ((npath - 1) + nres > self.path_result_count_max) {
                                nres = self.path_result_count_max - (npath - 1);
                            }

                            const kept: usize = @intCast(npath - 1);
                            const moved: usize = @intCast(nres);
                            @memmove(res[kept..][0..moved], res[0..moved]);
                            // Copy the old path in the beginning.
                            @memcpy(res[0..kept], path[0..kept]);
                            nres += npath - 1;

                            // Remove trackbacks.
                            var j: i32 = 0;
                            while (j < nres) : (j += 1) {
                                if (j - 1 >= 0 and j + 1 < nres) {
                                    const before: usize = @intCast(j - 1);
                                    const after: usize = @intCast(j + 1);
                                    if (res[before] == res[after]) {
                                        const tail: usize = @intCast(nres - (j + 1));
                                        @memmove(res[before..][0..tail], res[after..][0..tail]);
                                        nres -= 2;
                                        j -= 2;
                                    }
                                }
                            }
                        }

                        // Check for a partial path.
                        const last = res[@intCast(nres - 1)];
                        if (last != ag.target_ref) {
                            // Partial path, constrain the target position
                            // inside the last polygon.
                            var nearest: [3]f32 = undefined;
                            status = nav_query.closestPointOnPoly(
                                last,
                                target_pos,
                                &nearest,
                                null,
                            );
                            if (status.succeeded) {
                                target_pos = nearest;
                            } else {
                                valid = false;
                            }
                        }
                    }

                    if (valid) {
                        // Set the current corridor.
                        ag.corridor.setCorridor(target_pos, res[0..@intCast(nres)]);
                        // Force the boundary to update.
                        ag.boundary.reset();
                        ag.target_state = .valid;
                    } else {
                        // Something went wrong.
                        ag.target_state = .failed;
                    }

                    ag.target_replan_time = 0.0;
                }
            }
        }
    }

    /// Counts up the time since each walking agent's corridor topology was
    /// optimized, and optimizes the corridors that waited longest past the
    /// threshold.
    fn updateTopologyOptimization(self: *Crowd, agents: []const *CrowdAgent, dt: f32) void {
        if (agents.len == 0) return;

        var queue: [topology_optimization_agent_count_max]*CrowdAgent = undefined;
        var queue_count: i32 = 0;

        for (agents) |ag| {
            if (ag.state != .walking) continue;
            if (ag.target_state == .none or ag.target_state == .velocity) continue;
            if (!ag.params.update_flags.optimize_topology) continue;
            ag.topology_optimization_time += dt;
            if (ag.topology_optimization_time >= topology_optimization_time_threshold) {
                queue_count = addToOptQueue(ag, &queue, queue_count);
            }
        }

        for (queue[0..@intCast(queue_count)]) |ag| {
            _ = ag.corridor.optimizePathTopology(
                self.nav_query.?,
                &self.filters[ag.params.query_filter_type],
            );
            ag.topology_optimization_time = 0;
        }
    }

    /// Checks that each walking agent still stands on a valid polygon, has
    /// a valid target and a valid corridor ahead, puts it back on the mesh
    /// where it can, and asks for a replan where the path no longer holds.
    fn checkPathValidity(self: *Crowd, agents: []const *CrowdAgent, dt: f32) void {
        const nav_query = self.nav_query.?;

        for (agents) |ag| {
            if (ag.state != .walking) continue;

            ag.target_replan_time += dt;

            var replan = false;

            const filter = &self.filters[ag.params.query_filter_type];

            // First check that the current location is valid.
            const idx = self.getAgentIndex(ag);
            var agent_ref = ag.corridor.getFirstPoly();
            var agent_pos = ag.position;
            if (!nav_query.isValidPolyRef(agent_ref, filter)) {
                // The current location is not valid, try to reposition.
                // This can snap the agent.
                var nearest = agent_pos;
                agent_ref = 0;
                _ = nav_query.findNearestPoly(
                    ag.position,
                    self.agent_placement_half_extents,
                    filter,
                    &agent_ref,
                    &nearest,
                    null,
                );
                agent_pos = nearest;

                if (agent_ref == 0) {
                    // Could not find a location in the navmesh, set the
                    // state to invalid.
                    ag.corridor.reset(0, agent_pos);
                    ag.partial = false;
                    ag.boundary.reset();
                    ag.state = .invalid;
                    continue;
                }

                // Make sure the first polygon is valid, but leave the other
                // valid polygons in the path so that the replanner can
                // adjust the path better.
                _ = ag.corridor.fixPathStart(agent_ref, agent_pos);
                ag.boundary.reset();
                ag.position = agent_pos;

                replan = true;
            }

            // If the agent does not have a move target or is controlled by
            // velocity, there is no need to recover the target nor replan.
            if (ag.target_state == .none or ag.target_state == .velocity) continue;

            // Try to recover the move request position.
            if (ag.target_state != .none and ag.target_state != .failed) {
                if (!nav_query.isValidPolyRef(ag.target_ref, filter)) {
                    // The current target is not valid, try to reposition.
                    var nearest = ag.target_pos;
                    ag.target_ref = 0;
                    _ = nav_query.findNearestPoly(
                        ag.target_pos,
                        self.agent_placement_half_extents,
                        filter,
                        &ag.target_ref,
                        &nearest,
                        null,
                    );
                    ag.target_pos = nearest;
                    replan = true;
                }
                if (ag.target_ref == 0) {
                    // Failed to reposition the target, fail the move
                    // request.
                    ag.corridor.reset(agent_ref, agent_pos);
                    ag.partial = false;
                    ag.target_state = .none;
                }
            }

            // If the nearby corridor is not valid, replan.
            if (!ag.corridor.isValid(check_lookahead, nav_query, filter)) {
                replan = true;
            }

            // If the end of the path is near and it is not the requested
            // location, replan.
            if (ag.target_state == .valid) {
                if (ag.target_replan_time > target_replan_delay and
                    ag.corridor.getPathCount() < check_lookahead and
                    ag.corridor.getLastPoly() != ag.target_ref)
                {
                    replan = true;
                }
            }

            // Try to replan the path to the goal.
            if (replan) {
                if (ag.target_state != .none) {
                    _ = self.requestMoveTargetReplan(idx, ag.target_ref, ag.target_pos);
                }
            }
        }
    }

    /// Advances the crowd by `dt` seconds: checks the agents' paths, serves
    /// their move requests, finds each walking agent's walls, neighbours
    /// and corners, steers it, plans a velocity clear of the others, moves
    /// it, pushes overlapping agents apart, keeps it on the mesh, and
    /// carries the agents crossing off-mesh connections along.
    ///
    /// `debug`, when given, is filled for one agent: the one at place
    /// `debug.idx` among the active agents, counted in pool order, which is
    /// the pool index only while every agent before it is active.
    pub fn update(self: *Crowd, dt: f32, debug: ?*CrowdAgentDebugInfo) void {
        self.velocity_sample_count = 0;

        const debug_idx: i32 = if (debug) |info| info.idx else -1;

        const nav_query = self.nav_query.?;
        const grid = self.grid.?;
        const obstacle_query = self.obstacle_query.?;

        const agent_count = self.getActiveAgents(self.active_agents);
        const agents = self.active_agents[0..@intCast(agent_count)];

        // Check that all agents still have valid paths.
        self.checkPathValidity(agents, dt);

        // Update the async move requests and the path finder.
        self.updateMoveRequest();

        // Optimize path topology.
        self.updateTopologyOptimization(agents, dt);

        // Register the agents to the proximity grid.
        grid.clear();
        for (agents, 0..) |ag, i| {
            const p = ag.position;
            const r = ag.params.radius;
            grid.addItem(@truncate(i), p[0] - r, p[2] - r, p[0] + r, p[2] + r);
        }

        // Get the nearby navmesh segments and agents to collide with.
        for (agents) |ag| {
            if (ag.state != .walking) continue;

            const filter = &self.filters[ag.params.query_filter_type];

            // Update the collision boundary after a certain distance has
            // been passed or if it has become invalid.
            const update_threshold = ag.params.collision_query_range * 0.25;
            if (common.vdist2dSqr(ag.position, ag.boundary.getCenter()) >
                common.sqr(f32, update_threshold) or
                !ag.boundary.isValid(nav_query, filter))
            {
                ag.boundary.update(
                    ag.corridor.getFirstPoly(),
                    ag.position,
                    ag.params.collision_query_range,
                    nav_query,
                    filter,
                );
            }
            // Query the neighbour agents.
            ag.neighbour_count = getNeighbours(
                ag.position,
                ag.params.height,
                ag.params.collision_query_range,
                ag,
                &ag.neighbours,
                agents,
                grid,
            );
            // The grid names agents by their place among the active ones;
            // the neighbour list keeps pool indices.
            for (ag.neighbours[0..@intCast(ag.neighbour_count)]) |*neighbour| {
                neighbour.idx = self.getAgentIndex(agents[@intCast(neighbour.idx)]);
            }
        }

        // Find the next corner to steer to.
        for (agents, 0..) |ag, i| {
            if (ag.state != .walking) continue;
            if (ag.target_state == .none or ag.target_state == .velocity) continue;

            const filter = &self.filters[ag.params.query_filter_type];

            // Find the corners for steering.
            ag.corner_count = ag.corridor.findCorners(
                &ag.corner_verts,
                &ag.corner_flags,
                &ag.corner_polys,
                nav_query,
                filter,
            );

            // Check to see if the corner after the next corner is directly
            // visible, and short cut to there.
            if (ag.params.update_flags.optimize_visibility and ag.corner_count > 0) {
                const target_corner: usize = @intCast(@min(1, ag.corner_count - 1));
                const target = ag.corner_verts[target_corner];
                ag.corridor.optimizePathVisibility(
                    target,
                    ag.params.path_optimization_range,
                    nav_query,
                    filter,
                );

                if (debug_idx == @as(i32, @intCast(i))) {
                    debug.?.opt_start = ag.corridor.getPos();
                    debug.?.opt_end = target;
                }
            } else {
                if (debug_idx == @as(i32, @intCast(i))) {
                    debug.?.opt_start = .{ 0, 0, 0 };
                    debug.?.opt_end = .{ 0, 0, 0 };
                }
            }
        }

        // Trigger off-mesh connections (depends on the corners).
        for (agents) |ag| {
            if (ag.state != .walking) continue;
            if (ag.target_state == .none or ag.target_state == .velocity) continue;

            const trigger_radius = ag.params.radius * 2.25;
            if (overOffmeshConnection(ag, trigger_radius)) {
                // Prepare to cross the off-mesh connection.
                const anim = &self.agent_animations[@intCast(self.getAgentIndex(ag))];

                // Adjust the path over the off-mesh connection.
                var refs: [2]PolyRef = undefined;
                const moved = ag.corridor.moveOverOffmeshConnection(
                    ag.corner_polys[@intCast(ag.corner_count - 1)],
                    &refs,
                    &anim.start_pos,
                    &anim.end_pos,
                    nav_query,
                );
                if (moved) {
                    anim.init_pos = ag.position;
                    anim.poly_ref = refs[1];
                    anim.active = true;
                    anim.t = 0.0;
                    anim.t_max = (common.vdist2d(anim.start_pos, anim.end_pos) /
                        ag.params.speed_max) * 0.5;

                    ag.state = .off_mesh;
                    ag.corner_count = 0;
                    ag.neighbour_count = 0;
                    continue;
                }
                // Otherwise the path validity check sees to it that a bad
                // or blocked connection is replanned.
            }
        }

        // Calculate steering.
        for (agents) |ag| {
            if (ag.state != .walking) continue;
            if (ag.target_state == .none) continue;

            var dvel: [3]f32 = .{ 0, 0, 0 };

            if (ag.target_state == .velocity) {
                dvel = ag.target_pos;
                ag.desired_speed = common.vlen(ag.target_pos);
            } else {
                // Calculate the steering direction.
                if (ag.params.update_flags.anticipate_turns) {
                    dvel = calcSmoothSteerDirection(ag);
                } else {
                    dvel = calcStraightSteerDirection(ag);
                }

                // Calculate the speed scale, which tells the agent to slow
                // down at the end of the path.
                const slow_down_radius = ag.params.radius * 2;
                const speed_scale = getDistanceToGoal(ag, slow_down_radius) / slow_down_radius;

                ag.desired_speed = ag.params.speed_max;
                dvel = common.vscale(dvel, ag.desired_speed * speed_scale);
            }

            // Separation.
            if (ag.params.update_flags.separation) {
                const separation_dist = ag.params.collision_query_range;
                const separation_dist_inverse = 1.0 / separation_dist;
                const separation_weight = ag.params.separation_weight;

                var w: f32 = 0;
                var disp: [3]f32 = .{ 0, 0, 0 };

                for (ag.neighbours[0..@intCast(ag.neighbour_count)]) |neighbour| {
                    const nei = &self.agents[@intCast(neighbour.idx)];

                    var diff = common.vsub(ag.position, nei.position);
                    diff[1] = 0;

                    const dist_sqr = common.vlenSqr(diff);
                    if (dist_sqr < 0.00001) continue;
                    if (dist_sqr > common.sqr(f32, separation_dist)) continue;
                    const dist = @sqrt(dist_sqr);
                    const weight = separation_weight *
                        (1.0 - common.sqr(f32, dist * separation_dist_inverse));

                    disp = common.vmad(disp, diff, weight / dist);
                    w += 1.0;
                }

                if (w > 0.0001) {
                    // Adjust the desired velocity.
                    dvel = common.vmad(dvel, disp, 1.0 / w);
                    // Clamp the desired velocity to the desired speed.
                    const speed_sqr = common.vlenSqr(dvel);
                    const desired_sqr = common.sqr(f32, ag.desired_speed);
                    if (speed_sqr > desired_sqr) {
                        dvel = common.vscale(dvel, desired_sqr / speed_sqr);
                    }
                }
            }

            // Set the desired velocity.
            ag.desired_velocity = dvel;
        }

        // Velocity planning.
        for (agents, 0..) |ag, i| {
            if (ag.state != .walking) continue;

            if (ag.params.update_flags.obstacle_avoidance) {
                obstacle_query.reset();

                // Add the neighbours as obstacles.
                for (ag.neighbours[0..@intCast(ag.neighbour_count)]) |neighbour| {
                    const nei = &self.agents[@intCast(neighbour.idx)];
                    obstacle_query.addCircle(
                        nei.position,
                        nei.params.radius,
                        nei.velocity,
                        nei.desired_velocity,
                    );
                }

                // Append the neighbour segments as obstacles, leaving out
                // those the agent is behind.
                var j: i32 = 0;
                while (j < ag.boundary.getSegmentCount()) : (j += 1) {
                    const s = ag.boundary.getSegment(j);
                    if (common.triArea2d(ag.position, s[0], s[1]) < 0.0) continue;
                    obstacle_query.addSegment(s[0], s[1]);
                }

                var vod: ?*ObstacleAvoidanceDebug = null;
                if (debug_idx == @as(i32, @intCast(i))) vod = debug.?.vod;

                // Sample a new safe velocity.
                var ns: i32 = 0;

                const params = &self.obstacle_query_params[ag.params.obstacle_avoidance_type];

                if (adaptive_sampling) {
                    ns = obstacle_query.sampleVelocityAdaptive(
                        ag.position,
                        ag.params.radius,
                        ag.desired_speed,
                        ag.velocity,
                        ag.desired_velocity,
                        &ag.new_velocity,
                        params,
                        vod,
                    );
                } else {
                    ns = obstacle_query.sampleVelocityGrid(
                        ag.position,
                        ag.params.radius,
                        ag.desired_speed,
                        ag.velocity,
                        ag.desired_velocity,
                        &ag.new_velocity,
                        params,
                        vod,
                    );
                }
                self.velocity_sample_count += ns;
            } else {
                // Without velocity planning, the new velocity is directly
                // the desired velocity.
                ag.new_velocity = ag.desired_velocity;
            }
        }

        // Integrate.
        for (agents) |ag| {
            if (ag.state != .walking) continue;
            integrate(ag, dt);
        }

        // Handle collisions.
        for (0..4) |_| {
            for (agents) |ag| {
                const idx0 = self.getAgentIndex(ag);

                if (ag.state != .walking) continue;

                ag.displacement = .{ 0, 0, 0 };

                var w: f32 = 0;

                for (ag.neighbours[0..@intCast(ag.neighbour_count)]) |neighbour| {
                    const nei = &self.agents[@intCast(neighbour.idx)];
                    const idx1 = neighbour.idx;

                    var diff = common.vsub(ag.position, nei.position);
                    diff[1] = 0;

                    var dist = common.vlenSqr(diff);
                    if (dist > common.sqr(f32, ag.params.radius + nei.params.radius)) continue;
                    dist = @sqrt(dist);
                    var pen = (ag.params.radius + nei.params.radius) - dist;
                    if (dist < 0.0001) {
                        // Agents on top of each other, try to choose
                        // diverging separation directions.
                        if (idx0 > idx1) {
                            diff = .{ -ag.desired_velocity[2], 0, ag.desired_velocity[0] };
                        } else {
                            diff = .{ ag.desired_velocity[2], 0, -ag.desired_velocity[0] };
                        }
                        pen = 0.01;
                    } else {
                        pen = (1.0 / dist) * (pen * 0.5) * collision_resolve_factor;
                    }

                    ag.displacement = common.vmad(ag.displacement, diff, pen);

                    w += 1.0;
                }

                if (w > 0.0001) {
                    const iw = 1.0 / w;
                    ag.displacement = common.vscale(ag.displacement, iw);
                }
            }

            for (agents) |ag| {
                if (ag.state != .walking) continue;

                ag.position = common.vadd(ag.position, ag.displacement);
            }
        }

        for (agents) |ag| {
            if (ag.state != .walking) continue;

            // Move along the navmesh.
            _ = ag.corridor.movePosition(
                ag.position,
                nav_query,
                &self.filters[ag.params.query_filter_type],
            );
            // Get the valid constrained position back.
            ag.position = ag.corridor.getPos();

            // If not using a path, truncate the corridor to just one poly.
            if (ag.target_state == .none or ag.target_state == .velocity) {
                ag.corridor.reset(ag.corridor.getFirstPoly(), ag.position);
                ag.partial = false;
            }
        }

        // Update the agents using an off-mesh connection.
        for (agents) |ag| {
            const anim = &self.agent_animations[@intCast(self.getAgentIndex(ag))];
            if (!anim.active) continue;

            anim.t += dt;
            if (anim.t > anim.t_max) {
                // Reset the animation.
                anim.active = false;
                // Prepare the agent for walking.
                ag.state = .walking;
                continue;
            }

            // Update the position: first to the connection's start, then
            // across it.
            const ta = anim.t_max * 0.15;
            const tb = anim.t_max;
            if (anim.t < ta) {
                const u = tween(anim.t, 0.0, ta);
                ag.position = common.vlerp(anim.init_pos, anim.start_pos, u);
            } else {
                const u = tween(anim.t, ta, tb);
                ag.position = common.vlerp(anim.start_pos, anim.end_pos, u);
            }

            // Update the velocity.
            ag.velocity = .{ 0, 0, 0 };
            ag.desired_velocity = .{ 0, 0, 0 };
        }
    }

    /// Filter `i`, the one agents pick with
    /// `CrowdAgentParams.query_filter_type`, or null when `i` is outside
    /// 0..`query_filter_type_count_max`. The pointer is into the crowd
    /// itself and holds while the crowd stays where it is.
    pub fn getFilter(self: *const Crowd, i: i32) ?*const QueryFilter {
        if (i < 0 or i >= query_filter_type_count_max) return null;
        return &self.filters[@intCast(i)];
    }

    /// Filter `i` to change, or null when `i` is outside
    /// 0..`query_filter_type_count_max`. The pointer is into the crowd
    /// itself and holds while the crowd stays where it is. Searches under
    /// way read the filter as it is at each `update`.
    pub fn getEditableFilter(self: *Crowd, i: i32) ?*QueryFilter {
        if (i < 0 or i >= query_filter_type_count_max) return null;
        return &self.filters[@intCast(i)];
    }

    /// The half extents of the box the crowd searches for the polygon
    /// nearest an agent or a target, set from the largest agent radius.
    pub fn getQueryHalfExtents(self: *const Crowd) [3]f32 {
        return self.agent_placement_half_extents;
    }

    /// The same as `getQueryHalfExtents`, under its older name.
    pub fn getQueryExtents(self: *const Crowd) [3]f32 {
        return self.agent_placement_half_extents;
    }

    /// The number of velocities the latest `update` sampled over all
    /// agents.
    pub fn getVelocitySampleCount(self: *const Crowd) i32 {
        return self.velocity_sample_count;
    }

    /// The grid the latest `update` put the active agents in, by their
    /// place among the active agents. The pointer holds until `deinit` or
    /// another `init`. The crowd must be readied.
    pub fn getGrid(self: *const Crowd) *const ProximityGrid {
        return self.grid.?;
    }

    /// The queue serving the agents' long path requests. The pointer is
    /// into the crowd itself and holds while the crowd stays where it is.
    pub fn getPathQueue(self: *const Crowd) *const PathQueue {
        return &self.path_queue;
    }

    /// The query the crowd uses for its local searches. The pointer holds
    /// until `deinit` or another `init`. The crowd must be readied.
    pub fn getNavMeshQuery(self: *const Crowd) *const NavMeshQuery {
        return self.nav_query.?;
    }

    /// The off-mesh crossing of pool entry `idx`, which is in
    /// 0..`getAgentCount()`. It is under way while its `active` is set.
    /// The pointer is into a pool and holds until `deinit` or another
    /// `init`.
    pub fn getAgentAnimation(self: *const Crowd, idx: i32) *const CrowdAgentAnimation {
        return &self.agent_animations[@intCast(idx)];
    }
};
