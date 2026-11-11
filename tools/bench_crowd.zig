//! Benchmark rows for the crowd: its update and agent calls, and the
//! velocity sampling.

const std = @import("std");
const support = @import("support");
const Bench = @import("bench.zig").Bench;

const detour = support.detour;
const reference = support.reference;
const reference_crowd = support.reference_crowd;

const PolyRef = detour.PolyRef;

/// How many places the rows draw their points from.
const place_count = 256;
/// How many points of the level are tried as places.
const candidate_count = 1024;
/// How many candidates are tried as the place every other is reached from.
const anchor_try_count = 8;
/// The seed of the places.
const place_seed = 0x63726f77647362;
/// How many nodes the queries of the set-up and of the corridor, boundary
/// and queue rows search with: what a crowd gives its path queue.
const node_count_max = 4096;
/// How many polygons a path or a corridor holds: what a crowd gives its
/// agents' corridors.
const path_count_max = 256;
/// The radius of every agent, and of the largest agent of every crowd.
const agent_radius: f32 = 0.6;
/// How far every agent looks for walls and neighbours.
const collision_query_range: f32 = 7.2;
/// How far along its path an agent looks for a shortcut.
const path_optimization_range: f32 = 18.0;
/// Half the box a crowd searches for the polygon under a new agent of
/// `agent_radius`.
const placement_half_extents: [3]f32 = .{
    agent_radius * 2.0,
    agent_radius * 1.5,
    agent_radius * 2.0,
};

/// A status as a checksum takes it.
inline fn statusSum(status: detour.Status) u64 {
    return @as(u32, @bitCast(status));
}

/// The bits of a float as a checksum takes them.
inline fn floatSum(value: f32) u64 {
    return @as(u32, @bitCast(value));
}

/// The bits of a point's three floats, added up.
inline fn pointSum(point: [3]f32) u64 {
    return floatSum(point[0]) +% floatSum(point[1]) +% floatSum(point[2]);
}

/// What a call that wrote `count` references to `refs` adds to a checksum:
/// the count and the last reference written.
inline fn listSum(refs: []const PolyRef, count: usize) u64 {
    return count +% if (count == 0) 0 else @as(u64, refs[count - 1]);
}

/// An `i32` count or index as a checksum takes it.
inline fn countSum(count: i32) u64 {
    return @as(u32, @bitCast(count));
}

/// The parameters of every agent: a walker with the parts of the update
/// that `update_flags` names switched on.
fn agentParams(update_flags: detour.crowd.UpdateFlags) detour.CrowdAgentParams {
    return .{
        .radius = agent_radius,
        .height = 2.0,
        .acceleration_max = 8.0,
        .speed_max = 3.5,
        .collision_query_range = collision_query_range,
        .path_optimization_range = path_optimization_range,
        .separation_weight = 2.0,
        .update_flags = update_flags,
        .obstacle_avoidance_type = 3,
        .query_filter_type = 0,
    };
}

/// The original's layout of the port's agent parameters.
fn referenceParams(params: *const detour.CrowdAgentParams) reference_crowd.CrowdAgentParams {
    return .{
        .radius = params.radius,
        .height = params.height,
        .max_acceleration = params.acceleration_max,
        .max_speed = params.speed_max,
        .collision_query_range = params.collision_query_range,
        .path_optimization_range = params.path_optimization_range,
        .separation_weight = params.separation_weight,
        .update_flags = @bitCast(params.update_flags),
        .obstacle_avoidance_type = params.obstacle_avoidance_type,
        .query_filter_type = params.query_filter_type,
        .user_data = params.user_data,
    };
}

/// The avoidance parameters a new crowd holds in every one of its entries.
const avoidance_params: detour.ObstacleAvoidanceParams = .{
    .vel_bias = 0.4,
    .weight_desired_velocity = 2.0,
    .weight_current_velocity = 0.75,
    .weight_side = 0.75,
    .weight_time_of_impact = 2.5,
    .horizon_time = 2.5,
    .grid_size = 33,
    .adaptive_divisions = 7,
    .adaptive_rings = 2,
    .adaptive_depth = 5,
};

/// `avoidance_params` in the original's layout.
const reference_avoidance_params: reference_crowd.ObstacleAvoidanceParams = .{
    .vel_bias = avoidance_params.vel_bias,
    .weight_des_vel = avoidance_params.weight_desired_velocity,
    .weight_cur_vel = avoidance_params.weight_current_velocity,
    .weight_side = avoidance_params.weight_side,
    .weight_toi = avoidance_params.weight_time_of_impact,
    .horiz_time = avoidance_params.horizon_time,
    .grid_size = avoidance_params.grid_size,
    .adaptive_divs = avoidance_params.adaptive_divisions,
    .adaptive_rings = avoidance_params.adaptive_rings,
    .adaptive_depth = avoidance_params.adaptive_depth,
};

/// Places on the level's mesh that paths join: where agents start, where
/// they go, and the ends of the paths the corridor and queue rows use.
const Places = struct {
    /// Points of the level's ground, each moved onto the polygon nearest it
    /// in the box a crowd searches for a new agent.
    points: [place_count][3]f32,
    /// The polygon of each point.
    refs: [place_count]PolyRef,

    /// Draws the places with a query of the original: seeded points of the
    /// ground that have a polygon in reach, kept when a whole path joins
    /// them to one of them.
    fn init(self: *Places, world: *const support.World) !void {
        const query = reference.ref_query_alloc() orelse return error.OutOfMemory;
        defer reference.ref_query_free(query);
        const filter = reference.ref_filter_create() orelse return error.OutOfMemory;
        defer reference.ref_filter_destroy(filter);
        const attached: detour.Status = @bitCast(
            reference.ref_query_init(query, world.reference_nav, node_count_max),
        );
        if (attached.failed) return error.QueryInitFailed;

        var prng: std.Random.DefaultPrng = .init(place_seed);
        const random = prng.random();
        var candidates: [candidate_count][3]f32 = undefined;
        var candidate_refs: [candidate_count]PolyRef = undefined;
        var found: usize = 0;
        for (0..candidate_count * 64) |_| {
            if (found == candidate_count) break;
            const center = world.level.groundPoint(random, 0.3);
            var ref: PolyRef = 0;
            var point = center;
            _ = reference.ref_query_find_nearest_poly(
                query,
                &center,
                &placement_half_extents,
                filter,
                &ref,
                &point,
            );
            if (ref == 0) continue;
            candidates[found] = point;
            candidate_refs[found] = ref;
            found += 1;
        }
        if (found != candidate_count) return error.TooFewPoints;

        // A level may have ground no path reaches. The places are the
        // candidates joined to one anchor, and an anchor on a small island
        // gives way to the next candidate.
        var path: [path_count_max]PolyRef = undefined;
        for (0..anchor_try_count) |anchor| {
            var kept: usize = 0;
            for (&candidates, candidate_refs) |*point, ref| {
                if (kept == place_count) break;
                var path_count: i32 = 0;
                const status: detour.Status = @bitCast(reference.ref_query_find_path(
                    query,
                    candidate_refs[anchor],
                    ref,
                    &candidates[anchor],
                    point,
                    filter,
                    &path,
                    &path_count,
                    path_count_max,
                ));
                if (!status.succeeded or status.partial_result) continue;
                if (status.out_of_nodes or status.buffer_too_small) continue;
                self.points[kept] = point.*;
                self.refs[kept] = ref;
                kept += 1;
            }
            if (kept == place_count) return;
        }
        return error.TooFewPoints;
    }
};

/// How many agents the largest crowd of the rows holds.
const crowd_agent_count_max = 128;
/// How many updates one timed call of an update row runs.
const update_count = 64;
/// The seconds one update steps a crowd by.
const update_step: f32 = 1.0 / 20.0;

/// Every part of an agent's update switched on.
const all_update_flags: detour.crowd.UpdateFlags = .{
    .anticipate_turns = true,
    .obstacle_avoidance = true,
    .separation = true,
    .optimize_visibility = true,
    .optimize_topology = true,
};

/// The update with nothing that looks at the other agents: no velocity
/// sampling and no separation.
const solitary_update_flags: detour.crowd.UpdateFlags = .{
    .anticipate_turns = true,
    .optimize_visibility = true,
    .optimize_topology = true,
};

/// A crowd of each side that walks: `prepare` builds both anew, so every
/// timed call steps the same agents from the same start.
const CrowdContext = struct {
    world: *support.World,
    places: *const Places,
    /// How many agents each crowd holds, which is also the size of its pool.
    agent_count: usize,
    params: detour.CrowdAgentParams,
    original_params: reference_crowd.CrowdAgentParams,
    crowd: detour.Crowd,
    original_crowd: *reference_crowd.Crowd,

    /// Leaves both crowds without agents until `prepare`. `self` must not
    /// move afterwards. Free with `deinit`.
    fn init(
        self: *CrowdContext,
        world: *support.World,
        places: *const Places,
        agent_count: usize,
        update_flags: detour.crowd.UpdateFlags,
    ) !void {
        const params = agentParams(update_flags);
        self.* = .{
            .world = world,
            .places = places,
            .agent_count = agent_count,
            .params = params,
            .original_params = referenceParams(&params),
            .crowd = .empty,
            .original_crowd = reference_crowd.ref_crowd_alloc() orelse return error.OutOfMemory,
        };
    }

    fn deinit(self: *CrowdContext) void {
        reference_crowd.ref_crowd_free(self.original_crowd);
        self.crowd.deinit(std.heap.c_allocator);
        self.* = undefined;
    }

    /// Frees both crowds and builds them again from nothing: agent `i`
    /// stands at place `i` and is sent to place `place_count - 1 - i`, so
    /// no agent of an earlier call leaves a path request, a velocity or a
    /// neighbour behind.
    pub fn prepare(self: *CrowdContext) void {
        const agent_count_max: i32 = @intCast(self.agent_count);

        self.crowd.deinit(std.heap.c_allocator);
        self.crowd = .empty;
        if (!self.crowd.init(
            std.heap.c_allocator,
            agent_count_max,
            agent_radius,
            &self.world.nav,
        )) @panic("no memory for the port's crowd");

        reference_crowd.ref_crowd_free(self.original_crowd);
        self.original_crowd = reference_crowd.ref_crowd_alloc() orelse
            @panic("no memory for the original's crowd");
        if (!reference_crowd.ref_crowd_init(
            self.original_crowd,
            agent_count_max,
            agent_radius,
            self.world.reference_nav,
        )) @panic("no memory for the original's crowd");

        const places = self.places;
        for (0..self.agent_count) |i| {
            const idx: i32 = @intCast(i);
            const target = place_count - 1 - i;

            _ = self.crowd.addAgent(places.points[i], &self.params);
            _ = self.crowd.requestMoveTarget(idx, places.refs[target], places.points[target]);

            _ = reference_crowd.ref_crowd_add_agent(
                self.original_crowd,
                &places.points[i],
                &self.original_params,
            );
            _ = reference_crowd.ref_crowd_request_move_target(
                self.original_crowd,
                idx,
                places.refs[target],
                &places.points[target],
            );
        }
    }

    fn baselineUpdate(context: *CrowdContext) u64 {
        const crowd = context.original_crowd;
        var sum: u64 = 0;
        for (0..update_count) |_| {
            reference_crowd.ref_crowd_update(crowd, update_step, -1, null);
            sum +%= countSum(reference_crowd.ref_crowd_get_velocity_sample_count(crowd));
        }
        // Where every agent ended up says what all the updates did to it.
        for (0..context.agent_count) |i| {
            const agent = reference_crowd.ref_crowd_get_agent(crowd, @intCast(i)).?;
            const corridor = reference_crowd.ref_crowd_agent_corridor(agent);
            sum +%= pointSum(reference_crowd.ref_corridor_get_pos(corridor).*);
            sum +%= pointSum(reference_crowd.ref_corridor_get_target(corridor).*);
            sum +%= countSum(reference_crowd.ref_corridor_get_path_count(corridor));
            sum +%= reference_crowd.ref_corridor_get_first_poly(corridor);
        }
        return sum;
    }

    fn portUpdate(context: *CrowdContext) u64 {
        const crowd = &context.crowd;
        var sum: u64 = 0;
        for (0..update_count) |_| {
            crowd.update(update_step, null);
            sum +%= countSum(crowd.getVelocitySampleCount());
        }
        // Where every agent ended up says what all the updates did to it.
        for (0..context.agent_count) |i| {
            const corridor = &crowd.getAgent(@intCast(i)).?.corridor;
            sum +%= pointSum(corridor.getPos());
            sum +%= pointSum(corridor.getTarget());
            sum +%= countSum(corridor.getPathCount());
            sum +%= corridor.getFirstPoly();
        }
        return sum;
    }
};

/// How many times one timed call fills and empties a crowd's pool.
const agent_batch_count = 4;
/// How many targets one timed call sets.
const request_count = 4096;
/// The seed of the targets.
const request_seed = 0x74617267657473;

/// A crowd of each side that is never updated, for the calls that add,
/// remove and direct agents.
const AgentContext = struct {
    places: *const Places,
    params: detour.CrowdAgentParams,
    original_params: reference_crowd.CrowdAgentParams,
    crowd: detour.Crowd,
    original_crowd: *reference_crowd.Crowd,
    /// The corridor of each entry of the original's pool, which stays where
    /// it is for as long as the crowd does.
    original_corridors: [crowd_agent_count_max]*reference_crowd.PathCorridor,

    /// Room for every active agent of each side.
    active: [crowd_agent_count_max]*detour.CrowdAgent,
    original_active: [crowd_agent_count_max]i32,

    /// The agent each target is set for: the pool's entries in turn.
    request_agents: [request_count]i32,
    /// The place each target is, drawn from all the places.
    request_places: [request_count]usize,
    /// Room for everything the original shows of one agent.
    snapshot: reference_crowd.AgentSnapshot,

    /// Readies both crowds with empty pools of `crowd_agent_count_max`
    /// entries. `self` must not move afterwards. Free with `deinit`.
    fn init(self: *AgentContext, world: *support.World, places: *const Places) !void {
        self.places = places;
        self.params = agentParams(all_update_flags);
        self.original_params = referenceParams(&self.params);

        self.crowd = .empty;
        if (!self.crowd.init(
            std.heap.c_allocator,
            crowd_agent_count_max,
            agent_radius,
            &world.nav,
        )) return error.OutOfMemory;
        errdefer self.crowd.deinit(std.heap.c_allocator);

        self.original_crowd = reference_crowd.ref_crowd_alloc() orelse return error.OutOfMemory;
        errdefer reference_crowd.ref_crowd_free(self.original_crowd);
        if (!reference_crowd.ref_crowd_init(
            self.original_crowd,
            crowd_agent_count_max,
            agent_radius,
            world.reference_nav,
        )) return error.OutOfMemory;
        for (&self.original_corridors, 0..) |*corridor, i| {
            const agent = reference_crowd.ref_crowd_get_agent(
                self.original_crowd,
                @intCast(i),
            ) orelse return error.AgentMissing;
            corridor.* = reference_crowd.ref_crowd_agent_corridor(agent);
        }

        self.active = undefined;
        self.original_active = @splat(0);

        var prng: std.Random.DefaultPrng = .init(request_seed);
        const random = prng.random();
        for (&self.request_agents, &self.request_places, 0..) |*agent, *place, i| {
            agent.* = @intCast(i % crowd_agent_count_max);
            place.* = random.uintLessThan(usize, place_count);
        }
        self.snapshot = std.mem.zeroes(reference_crowd.AgentSnapshot);
    }

    fn deinit(self: *AgentContext) void {
        reference_crowd.ref_crowd_free(self.original_crowd);
        self.crowd.deinit(std.heap.c_allocator);
        self.* = undefined;
    }

    /// Adds an agent to every entry of both pools, at the first places.
    fn fill(self: *AgentContext) void {
        for (self.places.points[0..crowd_agent_count_max]) |*point| {
            _ = self.crowd.addAgent(point.*, &self.params);
            _ = reference_crowd.ref_crowd_add_agent(
                self.original_crowd,
                point,
                &self.original_params,
            );
        }
    }

    fn baselineAddRemove(context: *AgentContext) u64 {
        const crowd = context.original_crowd;
        var sum: u64 = 0;
        for (0..agent_batch_count) |_| {
            for (context.places.points[0..crowd_agent_count_max]) |*point| {
                const idx = reference_crowd.ref_crowd_add_agent(
                    crowd,
                    point,
                    &context.original_params,
                );
                const corridor = context.original_corridors[@intCast(idx)];
                sum +%= countSum(idx);
                sum +%= reference_crowd.ref_corridor_get_first_poly(corridor);
                sum +%= pointSum(reference_crowd.ref_corridor_get_pos(corridor).*);
            }
            for (0..crowd_agent_count_max) |i| {
                reference_crowd.ref_crowd_remove_agent(crowd, @intCast(i));
            }
        }
        // No agent is left when every one added was removed.
        sum +%= countSum(reference_crowd.ref_crowd_get_active_agents(
            crowd,
            &context.original_active,
            crowd_agent_count_max,
        ));
        return sum;
    }

    fn portAddRemove(context: *AgentContext) u64 {
        const crowd = &context.crowd;
        var sum: u64 = 0;
        for (0..agent_batch_count) |_| {
            for (context.places.points[0..crowd_agent_count_max]) |*point| {
                const idx = crowd.addAgent(point.*, &context.params);
                const corridor = &crowd.getAgent(idx).?.corridor;
                sum +%= countSum(idx);
                sum +%= corridor.getFirstPoly();
                sum +%= pointSum(corridor.getPos());
            }
            for (0..crowd_agent_count_max) |i| {
                crowd.removeAgent(@intCast(i));
            }
        }
        // No agent is left when every one added was removed.
        sum +%= countSum(crowd.getActiveAgents(&context.active));
        return sum;
    }

    fn baselineRequestMoveTarget(context: *AgentContext) u64 {
        const crowd = context.original_crowd;
        const places = context.places;
        var sum: u64 = 0;
        for (&context.request_agents, &context.request_places) |*agent, *place| {
            sum +%= @intFromBool(reference_crowd.ref_crowd_request_move_target(
                crowd,
                agent.*,
                places.refs[place.*],
                &places.points[place.*],
            ));
        }
        // The original shows an agent's target only in a snapshot, which
        // copies the whole agent, so one agent is read: the last one set.
        const last = context.request_agents[request_count - 1];
        _ = reference_crowd.ref_crowd_agent_snapshot(crowd, last, &context.snapshot);
        sum +%= context.snapshot.target_ref;
        sum +%= pointSum(context.snapshot.target_pos);
        sum +%= context.snapshot.target_state;
        return sum;
    }

    fn portRequestMoveTarget(context: *AgentContext) u64 {
        const crowd = &context.crowd;
        const places = context.places;
        var sum: u64 = 0;
        for (&context.request_agents, &context.request_places) |*agent, *place| {
            sum +%= @intFromBool(crowd.requestMoveTarget(
                agent.*,
                places.refs[place.*],
                places.points[place.*],
            ));
        }
        const last = crowd.getAgent(context.request_agents[request_count - 1]).?;
        sum +%= last.target_ref;
        sum +%= pointSum(last.target_pos);
        sum +%= @backingInt(last.target_state);
        return sum;
    }
};

/// How many situations the sampling rows run over.
const situation_count = 1000;
/// How many other agents stand around the agent of a situation.
const situation_circle_count = 6;
/// How many walls stand around the agent of a situation.
const situation_segment_count = 4;
/// The seed of the situations.
const situation_seed = 0x61766f6964616e;
/// The room a crowd gives its avoidance query.
const crowd_circle_count_max = 6;
const crowd_segment_count_max = 8;
/// How many situations' obstacles the add row puts in a query before it
/// resets the query.
const add_batch_size = 20;
/// The room of the add row's queries: more than a batch fills, so a query
/// that was not reset shows in its counts.
const add_circle_count_max = 128;
const add_segment_count_max = 128;

comptime {
    // The add row takes the situations in whole batches that fit its query.
    std.debug.assert(situation_count % add_batch_size == 0);
    std.debug.assert(add_batch_size * situation_circle_count < add_circle_count_max);
    std.debug.assert(add_batch_size * situation_segment_count < add_segment_count_max);
}

/// Another agent near the agent of a situation.
const Circle = struct {
    position: [3]f32,
    velocity: [3]f32,
    desired_velocity: [3]f32,
};

/// What an agent picking a velocity has around it.
const Situation = struct {
    position: [3]f32,
    velocity: [3]f32,
    desired_velocity: [3]f32,
    /// The length of `desired_velocity`, the greatest speed sampled, as a
    /// crowd passes it.
    desired_speed: f32,
    circles: [situation_circle_count]Circle,
    /// Each wall's two ends.
    segments: [situation_segment_count][2][3]f32,
};

/// The vector of `length` along the ground in the direction of `angle`.
fn groundVector(angle: f32, length: f32) [3]f32 {
    return .{ @cos(angle) * length, 0.0, @sin(angle) * length };
}

/// Twice the area of the triangle `a`, `b`, `c` seen from above, negative
/// for one of the two turning directions: the test a crowd puts a wall to.
fn triArea2D(a: [3]f32, b: [3]f32, c: [3]f32) f32 {
    const abx = b[0] - a[0];
    const abz = b[2] - a[2];
    const acx = c[0] - a[0];
    const acz = c[2] - a[2];
    return acx * abz - abx * acz;
}

/// Draws the surroundings of an agent at `position`: a velocity near the
/// one it wants, agents 1.5 to 6.5 units away that move their own ways, and
/// walls 1 to 5 units away. No circle is on the agent, which would leave
/// the direction to it without a length, and no wall's line passes through
/// the agent.
fn drawSituation(random: std.Random, position: [3]f32) Situation {
    var situation: Situation = undefined;
    situation.position = position;
    const heading = random.float(f32) * std.math.tau;
    situation.desired_speed = 1.5 + random.float(f32) * 2.0;
    situation.desired_velocity = groundVector(heading, situation.desired_speed);
    const drift = random.float(f32) - 0.5;
    situation.velocity = groundVector(heading + drift, 0.5 + random.float(f32) * 3.0);

    for (&situation.circles) |*circle| {
        const bearing = random.float(f32) * std.math.tau;
        const offset = groundVector(bearing, 1.5 + random.float(f32) * 5.0);
        const height = (random.float(f32) - 0.5) * 0.4;
        circle.position = .{
            position[0] + offset[0],
            position[1] + height,
            position[2] + offset[2],
        };
        const course = random.float(f32) * std.math.tau;
        circle.velocity = groundVector(course, random.float(f32) * 3.5);
        const turn = random.float(f32) - 0.5;
        circle.desired_velocity = groundVector(course + turn, 1.5 + random.float(f32) * 2.0);
    }

    for (&situation.segments) |*segment| {
        const bearing = random.float(f32) * std.math.tau;
        const middle = groundVector(bearing, 1.0 + random.float(f32) * 4.0);
        // The wall lies across the line from the agent to its middle,
        // turned by up to 0.6 radians, so its line stays clear of the agent.
        const tilt = (random.float(f32) * 2.0 - 1.0) * 0.6;
        const half = groundVector(
            bearing + std.math.pi / 2.0 + tilt,
            0.5 + random.float(f32) * 2.5,
        );
        var p: [3]f32 = .{
            position[0] + middle[0] - half[0],
            position[1],
            position[2] + middle[2] - half[2],
        };
        var q: [3]f32 = .{
            position[0] + middle[0] + half[0],
            position[1],
            position[2] + middle[2] + half[2],
        };
        // A crowd passes on only the walls that face the agent.
        if (triArea2D(position, p, q) < 0.0) std.mem.swap([3]f32, &p, &q);
        segment.* = .{ p, q };
    }
    return situation;
}

/// The avoidance queries of both sides: one for each situation, filled
/// with its obstacles, and one more of each side for the add row.
const AvoidanceContext = struct {
    situations: []Situation,
    /// The query of each situation, with a crowd's room, holding the
    /// situation's circles and segments. Sampling leaves them as they are.
    queries: []detour.ObstacleAvoidanceQuery,
    original_queries: []*reference_crowd.ObstacleAvoidanceQuery,
    /// The queries the add row fills.
    add_query: detour.ObstacleAvoidanceQuery,
    original_add_query: *reference_crowd.ObstacleAvoidanceQuery,
    /// Room for one circle and one segment of the original's query.
    original_circle: reference_crowd.ObstacleCircle,
    original_segment: reference_crowd.ObstacleSegment,

    /// Draws the situations around the places and fills the queries. `gpa`
    /// holds the situations and the lists of queries. Free with `deinit`.
    fn init(self: *AvoidanceContext, gpa: std.mem.Allocator, places: *const Places) !void {
        const situations = try gpa.alloc(Situation, situation_count);
        errdefer gpa.free(situations);
        var prng: std.Random.DefaultPrng = .init(situation_seed);
        const random = prng.random();
        for (situations, 0..) |*situation, i| {
            situation.* = drawSituation(random, places.points[i % place_count]);
        }

        const queries = try gpa.alloc(detour.ObstacleAvoidanceQuery, situation_count);
        errdefer gpa.free(queries);
        @memset(queries, detour.ObstacleAvoidanceQuery.empty);
        errdefer {
            for (queries) |*query| query.deinit(std.heap.c_allocator);
        }

        const original_queries = try gpa.alloc(
            *reference_crowd.ObstacleAvoidanceQuery,
            situation_count,
        );
        errdefer gpa.free(original_queries);
        var original_count: usize = 0;
        errdefer {
            for (original_queries[0..original_count]) |query| {
                reference_crowd.ref_oa_query_free(query);
            }
        }

        for (situations, queries, original_queries) |*situation, *query, *original_query| {
            if (!query.init(
                std.heap.c_allocator,
                crowd_circle_count_max,
                crowd_segment_count_max,
            )) return error.OutOfMemory;
            const original = reference_crowd.ref_oa_query_alloc() orelse return error.OutOfMemory;
            original_query.* = original;
            original_count += 1;
            if (!reference_crowd.ref_oa_query_init(
                original,
                crowd_circle_count_max,
                crowd_segment_count_max,
            )) return error.OutOfMemory;

            for (&situation.circles) |*circle| {
                query.addCircle(
                    circle.position,
                    agent_radius,
                    circle.velocity,
                    circle.desired_velocity,
                );
                reference_crowd.ref_oa_query_add_circle(
                    original,
                    &circle.position,
                    agent_radius,
                    &circle.velocity,
                    &circle.desired_velocity,
                );
            }
            for (&situation.segments) |*segment| {
                query.addSegment(segment[0], segment[1]);
                reference_crowd.ref_oa_query_add_segment(original, &segment[0], &segment[1]);
            }
        }

        var add_query: detour.ObstacleAvoidanceQuery = .empty;
        if (!add_query.init(
            std.heap.c_allocator,
            add_circle_count_max,
            add_segment_count_max,
        )) return error.OutOfMemory;
        errdefer add_query.deinit(std.heap.c_allocator);
        const original_add_query = reference_crowd.ref_oa_query_alloc() orelse {
            return error.OutOfMemory;
        };
        errdefer reference_crowd.ref_oa_query_free(original_add_query);
        if (!reference_crowd.ref_oa_query_init(
            original_add_query,
            add_circle_count_max,
            add_segment_count_max,
        )) return error.OutOfMemory;

        self.* = .{
            .situations = situations,
            .queries = queries,
            .original_queries = original_queries,
            .add_query = add_query,
            .original_add_query = original_add_query,
            .original_circle = std.mem.zeroes(reference_crowd.ObstacleCircle),
            .original_segment = std.mem.zeroes(reference_crowd.ObstacleSegment),
        };
    }

    fn deinit(self: *AvoidanceContext, gpa: std.mem.Allocator) void {
        reference_crowd.ref_oa_query_free(self.original_add_query);
        self.add_query.deinit(std.heap.c_allocator);
        for (self.original_queries) |query| reference_crowd.ref_oa_query_free(query);
        gpa.free(self.original_queries);
        for (self.queries) |*query| query.deinit(std.heap.c_allocator);
        gpa.free(self.queries);
        gpa.free(self.situations);
        self.* = undefined;
    }

    fn baselineSampleVelocityGrid(context: *AvoidanceContext) u64 {
        var sum: u64 = 0;
        var nvel: [3]f32 = @splat(0);
        for (context.original_queries, context.situations) |query, *situation| {
            sum +%= countSum(reference_crowd.ref_oa_query_sample_velocity_grid(
                query,
                &situation.position,
                agent_radius,
                situation.desired_speed,
                &situation.velocity,
                &situation.desired_velocity,
                &nvel,
                &reference_avoidance_params,
                null,
            ));
            sum +%= pointSum(nvel);
        }
        return sum;
    }

    fn portSampleVelocityGrid(context: *AvoidanceContext) u64 {
        var sum: u64 = 0;
        var nvel: [3]f32 = @splat(0);
        for (context.queries, context.situations) |*query, *situation| {
            sum +%= countSum(query.sampleVelocityGrid(
                situation.position,
                agent_radius,
                situation.desired_speed,
                situation.velocity,
                situation.desired_velocity,
                &nvel,
                &avoidance_params,
                null,
            ));
            sum +%= pointSum(nvel);
        }
        return sum;
    }

    fn baselineSampleVelocityAdaptive(context: *AvoidanceContext) u64 {
        var sum: u64 = 0;
        var nvel: [3]f32 = @splat(0);
        for (context.original_queries, context.situations) |query, *situation| {
            sum +%= countSum(reference_crowd.ref_oa_query_sample_velocity_adaptive(
                query,
                &situation.position,
                agent_radius,
                situation.desired_speed,
                &situation.velocity,
                &situation.desired_velocity,
                &nvel,
                &reference_avoidance_params,
                null,
            ));
            sum +%= pointSum(nvel);
        }
        return sum;
    }

    fn portSampleVelocityAdaptive(context: *AvoidanceContext) u64 {
        var sum: u64 = 0;
        var nvel: [3]f32 = @splat(0);
        for (context.queries, context.situations) |*query, *situation| {
            sum +%= countSum(query.sampleVelocityAdaptive(
                situation.position,
                agent_radius,
                situation.desired_speed,
                situation.velocity,
                situation.desired_velocity,
                &nvel,
                &avoidance_params,
                null,
            ));
            sum +%= pointSum(nvel);
        }
        return sum;
    }

    fn baselineAdd(context: *AvoidanceContext) u64 {
        const query = context.original_add_query;
        var sum: u64 = 0;
        var first: usize = 0;
        while (first < situation_count) : (first += add_batch_size) {
            reference_crowd.ref_oa_query_reset(query);
            for (context.situations[first..][0..add_batch_size]) |*situation| {
                for (&situation.circles) |*circle| {
                    reference_crowd.ref_oa_query_add_circle(
                        query,
                        &circle.position,
                        agent_radius,
                        &circle.velocity,
                        &circle.desired_velocity,
                    );
                }
                for (&situation.segments) |*segment| {
                    reference_crowd.ref_oa_query_add_segment(query, &segment[0], &segment[1]);
                }
            }
            // The counts are right only when the reset emptied the query
            // and every obstacle of the batch went in.
            sum +%= countSum(reference_crowd.ref_oa_query_get_obstacle_circle_count(query));
            sum +%= countSum(reference_crowd.ref_oa_query_get_obstacle_segment_count(query));
        }
        // The last obstacles added, as the query holds them.
        const circle = &context.original_circle;
        const segment = &context.original_segment;
        const last_circle = add_batch_size * situation_circle_count - 1;
        const last_segment = add_batch_size * situation_segment_count - 1;
        _ = reference_crowd.ref_oa_query_get_obstacle_circle(query, last_circle, circle);
        _ = reference_crowd.ref_oa_query_get_obstacle_segment(query, last_segment, segment);
        sum +%= pointSum(circle.p) +% pointSum(circle.vel) +% pointSum(circle.dvel);
        sum +%= floatSum(circle.rad) +% pointSum(segment.p) +% pointSum(segment.q);
        return sum;
    }

    fn portAdd(context: *AvoidanceContext) u64 {
        const query = &context.add_query;
        var sum: u64 = 0;
        var first: usize = 0;
        while (first < situation_count) : (first += add_batch_size) {
            query.reset();
            for (context.situations[first..][0..add_batch_size]) |*situation| {
                for (&situation.circles) |*circle| {
                    query.addCircle(
                        circle.position,
                        agent_radius,
                        circle.velocity,
                        circle.desired_velocity,
                    );
                }
                for (&situation.segments) |*segment| {
                    query.addSegment(segment[0], segment[1]);
                }
            }
            // The counts are right only when the reset emptied the query
            // and every obstacle of the batch went in.
            sum +%= countSum(query.getObstacleCircleCount());
            sum +%= countSum(query.getObstacleSegmentCount());
        }
        // The last obstacles added, as the query holds them.
        const circle = query.getObstacleCircle(add_batch_size * situation_circle_count - 1);
        const segment = query.getObstacleSegment(add_batch_size * situation_segment_count - 1);
        sum +%= pointSum(circle.position) +% pointSum(circle.velocity);
        sum +%= pointSum(circle.desired_velocity);
        sum +%= floatSum(circle.radius) +% pointSum(segment.p) +% pointSum(segment.q);
        return sum;
    }
};

pub fn run(bench: *Bench) !void {
    try bench.group("Crowd");

    // The tiled level: its mesh on both sides, built once for every row.
    var world: support.World = undefined;
    try world.init(std.heap.c_allocator, support.nav_test_obj, support.tiled_config, .{});
    defer world.deinit();
    var places: Places = undefined;
    try places.init(&world);

    // 128 agents walking across the level, every part of the update on,
    // 64 updates of 1/20 s a call; then 16 agents; then 128 that steer by
    // their paths alone.
    const CrowdRow = struct {
        name: []const u8,
        agents: usize,
        flags: detour.crowd.UpdateFlags,
    };
    const crowd_rows = [_]CrowdRow{
        .{ .name = "Crowd.update, 128 agents", .agents = 128, .flags = all_update_flags },
        .{ .name = "Crowd.update, 16 agents", .agents = 16, .flags = all_update_flags },
        .{
            .name = "Crowd.update, 128 agents, no avoidance or separation",
            .agents = 128,
            .flags = solitary_update_flags,
        },
    };
    for (crowd_rows) |crowd_row| {
        var context: CrowdContext = undefined;
        try context.init(&world, &places, crowd_row.agents, crowd_row.flags);
        defer context.deinit();
        try bench.compare(
            .{
                .name = crowd_row.name,
                .per = "agent step",
                .count = crowd_row.agents * update_count,
            },
            &context,
            CrowdContext.baselineUpdate,
            CrowdContext.portUpdate,
        );
    }

    {
        var context: AgentContext = undefined;
        try context.init(&world, &places);
        defer context.deinit();
        // Every entry of a pool of 128 filled and emptied, four times.
        try bench.compare(
            .{
                .name = "Crowd.addAgent + removeAgent",
                .per = "agent",
                .count = crowd_agent_count_max * agent_batch_count,
            },
            &context,
            AgentContext.baselineAddRemove,
            AgentContext.portAddRemove,
        );
        // Targets drawn from the places, set for the 128 agents in turn.
        try bench.compare(
            .{ .name = "Crowd.requestMoveTarget", .per = "call", .count = request_count },
            &context,
            AgentContext.baselineRequestMoveTarget,
            AgentContext.portRequestMoveTarget,
        );
    }

    {
        var context: AvoidanceContext = undefined;
        try context.init(bench.gpa, &places);
        defer context.deinit(bench.gpa);
        // 1,000 situations of 6 agents and 4 walls around an agent, the
        // default parameters.
        try bench.compare(
            .{
                .name = "ObstacleAvoidanceQuery.sampleVelocityGrid",
                .per = "situation",
                .count = situation_count,
            },
            &context,
            AvoidanceContext.baselineSampleVelocityGrid,
            AvoidanceContext.portSampleVelocityGrid,
        );
        try bench.compare(
            .{
                .name = "ObstacleAvoidanceQuery.sampleVelocityAdaptive",
                .per = "situation",
                .count = situation_count,
            },
            &context,
            AvoidanceContext.baselineSampleVelocityAdaptive,
            AvoidanceContext.portSampleVelocityAdaptive,
        );
        // The same situations' obstacles added, 20 situations between
        // resets.
        try bench.compare(
            .{
                .name = "ObstacleAvoidanceQuery.addCircle + addSegment + reset",
                .per = "situation",
                .count = situation_count,
            },
            &context,
            AvoidanceContext.baselineAdd,
            AvoidanceContext.portAdd,
        );
    }
}
