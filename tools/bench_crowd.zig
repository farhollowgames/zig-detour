//! Benchmark rows for the crowd: its update and agent calls, the velocity
//! sampling, the path corridor, the local boundary, the proximity grid and
//! the path queue.

const std = @import("std");
const support = @import("support");
const Bench = @import("bench.zig").Bench;
const opaqueFn = @import("bench.zig").opaqueFn;

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
        const update = opaqueFn(reference_crowd.ref_crowd_update);
        const getVelocitySampleCount = opaqueFn(
            reference_crowd.ref_crowd_get_velocity_sample_count,
        );
        const getAgent = opaqueFn(reference_crowd.ref_crowd_get_agent);
        const agentCorridor = opaqueFn(reference_crowd.ref_crowd_agent_corridor);
        const getPos = opaqueFn(reference_crowd.ref_corridor_get_pos);
        const getTarget = opaqueFn(reference_crowd.ref_corridor_get_target);
        const getPathCount = opaqueFn(reference_crowd.ref_corridor_get_path_count);
        const getFirstPoly = opaqueFn(reference_crowd.ref_corridor_get_first_poly);
        const crowd = context.original_crowd;
        var sum: u64 = 0;
        for (0..update_count) |_| {
            update(crowd, update_step, -1, null);
            sum +%= countSum(getVelocitySampleCount(crowd));
        }
        // Where every agent ended up says what all the updates did to it.
        for (0..context.agent_count) |i| {
            const agent = getAgent(crowd, @intCast(i)).?;
            const corridor = agentCorridor(agent);
            sum +%= pointSum(getPos(corridor).*);
            sum +%= pointSum(getTarget(corridor).*);
            sum +%= countSum(getPathCount(corridor));
            sum +%= getFirstPoly(corridor);
        }
        return sum;
    }

    fn portUpdate(context: *CrowdContext) u64 {
        const update = opaqueFn(detour.Crowd.update);
        const getVelocitySampleCount = opaqueFn(detour.Crowd.getVelocitySampleCount);
        const getAgent = opaqueFn(detour.Crowd.getAgent);
        const getPos = opaqueFn(detour.PathCorridor.getPos);
        const getTarget = opaqueFn(detour.PathCorridor.getTarget);
        const getPathCount = opaqueFn(detour.PathCorridor.getPathCount);
        const getFirstPoly = opaqueFn(detour.PathCorridor.getFirstPoly);
        const crowd = &context.crowd;
        var sum: u64 = 0;
        for (0..update_count) |_| {
            update(crowd, update_step, null);
            sum +%= countSum(getVelocitySampleCount(crowd));
        }
        // Where every agent ended up says what all the updates did to it.
        for (0..context.agent_count) |i| {
            const corridor = &getAgent(crowd, @intCast(i)).?.corridor;
            sum +%= pointSum(getPos(corridor));
            sum +%= pointSum(getTarget(corridor));
            sum +%= countSum(getPathCount(corridor));
            sum +%= getFirstPoly(corridor);
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
        const addAgent = opaqueFn(reference_crowd.ref_crowd_add_agent);
        const getFirstPoly = opaqueFn(reference_crowd.ref_corridor_get_first_poly);
        const getPos = opaqueFn(reference_crowd.ref_corridor_get_pos);
        const removeAgent = opaqueFn(reference_crowd.ref_crowd_remove_agent);
        const getActiveAgents = opaqueFn(reference_crowd.ref_crowd_get_active_agents);
        const crowd = context.original_crowd;
        var sum: u64 = 0;
        for (0..agent_batch_count) |_| {
            for (context.places.points[0..crowd_agent_count_max]) |*point| {
                const idx = addAgent(
                    crowd,
                    point,
                    &context.original_params,
                );
                const corridor = context.original_corridors[@intCast(idx)];
                sum +%= countSum(idx);
                sum +%= getFirstPoly(corridor);
                sum +%= pointSum(getPos(corridor).*);
            }
            for (0..crowd_agent_count_max) |i| {
                removeAgent(crowd, @intCast(i));
            }
        }
        // No agent is left when every one added was removed.
        sum +%= countSum(getActiveAgents(
            crowd,
            &context.original_active,
            crowd_agent_count_max,
        ));
        return sum;
    }

    fn portAddRemove(context: *AgentContext) u64 {
        const addAgent = opaqueFn(detour.Crowd.addAgent);
        const getAgent = opaqueFn(detour.Crowd.getAgent);
        const removeAgent = opaqueFn(detour.Crowd.removeAgent);
        const getActiveAgents = opaqueFn(detour.Crowd.getActiveAgents);
        const getFirstPoly = opaqueFn(detour.PathCorridor.getFirstPoly);
        const getPos = opaqueFn(detour.PathCorridor.getPos);
        const crowd = &context.crowd;
        var sum: u64 = 0;
        for (0..agent_batch_count) |_| {
            for (context.places.points[0..crowd_agent_count_max]) |*point| {
                const idx = addAgent(crowd, point.*, &context.params);
                const corridor = &getAgent(crowd, idx).?.corridor;
                sum +%= countSum(idx);
                sum +%= getFirstPoly(corridor);
                sum +%= pointSum(getPos(corridor));
            }
            for (0..crowd_agent_count_max) |i| {
                removeAgent(crowd, @intCast(i));
            }
        }
        // No agent is left when every one added was removed.
        sum +%= countSum(getActiveAgents(crowd, &context.active));
        return sum;
    }

    fn baselineRequestMoveTarget(context: *AgentContext) u64 {
        const requestMoveTarget = opaqueFn(reference_crowd.ref_crowd_request_move_target);
        const agentSnapshot = opaqueFn(reference_crowd.ref_crowd_agent_snapshot);
        const crowd = context.original_crowd;
        const places = context.places;
        var sum: u64 = 0;
        for (&context.request_agents, &context.request_places) |*agent, *place| {
            sum +%= @intFromBool(requestMoveTarget(
                crowd,
                agent.*,
                places.refs[place.*],
                &places.points[place.*],
            ));
        }
        // The original shows an agent's target only in a snapshot, which
        // copies the whole agent, so one agent is read: the last one set.
        const last = context.request_agents[request_count - 1];
        _ = agentSnapshot(crowd, last, &context.snapshot);
        sum +%= context.snapshot.target_ref;
        sum +%= pointSum(context.snapshot.target_pos);
        sum +%= context.snapshot.target_state;
        return sum;
    }

    fn portRequestMoveTarget(context: *AgentContext) u64 {
        const requestMoveTarget = opaqueFn(detour.Crowd.requestMoveTarget);
        const getAgent = opaqueFn(detour.Crowd.getAgent);
        const crowd = &context.crowd;
        const places = context.places;
        var sum: u64 = 0;
        for (&context.request_agents, &context.request_places) |*agent, *place| {
            sum +%= @intFromBool(requestMoveTarget(
                crowd,
                agent.*,
                places.refs[place.*],
                places.points[place.*],
            ));
        }
        const last = getAgent(crowd, context.request_agents[request_count - 1]).?;
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
        const sampleVelocityGrid = opaqueFn(reference_crowd.ref_oa_query_sample_velocity_grid);
        var sum: u64 = 0;
        var nvel: [3]f32 = @splat(0);
        for (context.original_queries, context.situations) |query, *situation| {
            sum +%= countSum(sampleVelocityGrid(
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
        const sampleVelocityGrid = opaqueFn(detour.ObstacleAvoidanceQuery.sampleVelocityGrid);
        var sum: u64 = 0;
        var nvel: [3]f32 = @splat(0);
        for (context.queries, context.situations) |*query, *situation| {
            sum +%= countSum(sampleVelocityGrid(
                query,
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
        const sampleVelocityAdaptive = opaqueFn(
            reference_crowd.ref_oa_query_sample_velocity_adaptive,
        );
        var sum: u64 = 0;
        var nvel: [3]f32 = @splat(0);
        for (context.original_queries, context.situations) |query, *situation| {
            sum +%= countSum(sampleVelocityAdaptive(
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
        const sampleVelocityAdaptive = opaqueFn(
            detour.ObstacleAvoidanceQuery.sampleVelocityAdaptive,
        );
        var sum: u64 = 0;
        var nvel: [3]f32 = @splat(0);
        for (context.queries, context.situations) |*query, *situation| {
            sum +%= countSum(sampleVelocityAdaptive(
                query,
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
        const reset = opaqueFn(reference_crowd.ref_oa_query_reset);
        const addCircle = opaqueFn(reference_crowd.ref_oa_query_add_circle);
        const addSegment = opaqueFn(reference_crowd.ref_oa_query_add_segment);
        const getObstacleCircleCount = opaqueFn(
            reference_crowd.ref_oa_query_get_obstacle_circle_count,
        );
        const getObstacleSegmentCount = opaqueFn(
            reference_crowd.ref_oa_query_get_obstacle_segment_count,
        );
        const getObstacleCircle = opaqueFn(reference_crowd.ref_oa_query_get_obstacle_circle);
        const getObstacleSegment = opaqueFn(reference_crowd.ref_oa_query_get_obstacle_segment);
        const query = context.original_add_query;
        var sum: u64 = 0;
        var first: usize = 0;
        while (first < situation_count) : (first += add_batch_size) {
            reset(query);
            for (context.situations[first..][0..add_batch_size]) |*situation| {
                for (&situation.circles) |*circle| {
                    addCircle(
                        query,
                        &circle.position,
                        agent_radius,
                        &circle.velocity,
                        &circle.desired_velocity,
                    );
                }
                for (&situation.segments) |*segment| {
                    addSegment(query, &segment[0], &segment[1]);
                }
            }
            // The counts are right only when the reset emptied the query
            // and every obstacle of the batch went in.
            sum +%= countSum(getObstacleCircleCount(query));
            sum +%= countSum(getObstacleSegmentCount(query));
        }
        // The last obstacles added, as the query holds them.
        const circle = &context.original_circle;
        const segment = &context.original_segment;
        const last_circle = add_batch_size * situation_circle_count - 1;
        const last_segment = add_batch_size * situation_segment_count - 1;
        _ = getObstacleCircle(query, last_circle, circle);
        _ = getObstacleSegment(query, last_segment, segment);
        sum +%= pointSum(circle.p) +% pointSum(circle.vel) +% pointSum(circle.dvel);
        sum +%= floatSum(circle.rad) +% pointSum(segment.p) +% pointSum(segment.q);
        return sum;
    }

    fn portAdd(context: *AvoidanceContext) u64 {
        const reset = opaqueFn(detour.ObstacleAvoidanceQuery.reset);
        const addCircle = opaqueFn(detour.ObstacleAvoidanceQuery.addCircle);
        const addSegment = opaqueFn(detour.ObstacleAvoidanceQuery.addSegment);
        const getObstacleCircleCount = opaqueFn(
            detour.ObstacleAvoidanceQuery.getObstacleCircleCount,
        );
        const getObstacleSegmentCount = opaqueFn(
            detour.ObstacleAvoidanceQuery.getObstacleSegmentCount,
        );
        const getObstacleCircle = opaqueFn(detour.ObstacleAvoidanceQuery.getObstacleCircle);
        const getObstacleSegment = opaqueFn(detour.ObstacleAvoidanceQuery.getObstacleSegment);
        const query = &context.add_query;
        var sum: u64 = 0;
        var first: usize = 0;
        while (first < situation_count) : (first += add_batch_size) {
            reset(query);
            for (context.situations[first..][0..add_batch_size]) |*situation| {
                for (&situation.circles) |*circle| {
                    addCircle(
                        query,
                        circle.position,
                        agent_radius,
                        circle.velocity,
                        circle.desired_velocity,
                    );
                }
                for (&situation.segments) |*segment| {
                    addSegment(query, segment[0], segment[1]);
                }
            }
            // The counts are right only when the reset emptied the query
            // and every obstacle of the batch went in.
            sum +%= countSum(getObstacleCircleCount(query));
            sum +%= countSum(getObstacleSegmentCount(query));
        }
        // The last obstacles added, as the query holds them.
        const circle = getObstacleCircle(query, add_batch_size * situation_circle_count - 1);
        const segment = getObstacleSegment(query, add_batch_size * situation_segment_count - 1);
        sum +%= pointSum(circle.position) +% pointSum(circle.velocity);
        sum +%= pointSum(circle.desired_velocity);
        sum +%= floatSum(circle.radius) +% pointSum(segment.p) +% pointSum(segment.q);
        return sum;
    }
};

/// How many paths the corridor and queue rows go along: from each place of
/// the first half to the place half the list on.
const pair_count = place_count / 2;
/// How many corners a corridor is asked for: what a crowd asks for.
const corner_count_max = 4;
/// How many neighbours a grid query returns: what a crowd asks for.
const neighbour_count_max = 32;
/// The search iterations of one path queue update: what a crowd gives it.
const queue_iteration_count = 100;
/// How many requests the path queue holds at once.
const queue_request_count = detour.PathQueue.request_count_max;
/// The updates a batch of queue requests may take before the row moves on.
const queue_update_count_max = 1024;

/// A query of each side, the paths between pairs of places, and a corridor,
/// a boundary, a proximity grid and a path queue of each side, for the rows
/// of the crowd's parts.
const PartsContext = struct {
    places: *const Places,
    query: detour.NavMeshQuery,
    filter: detour.QueryFilter,
    corridor: detour.PathCorridor,
    boundary: detour.LocalBoundary,
    grid: detour.ProximityGrid,
    queue: detour.PathQueue,
    original_query: *reference.NavMeshQuery,
    original_filter: *reference.QueryFilter,
    original_corridor: *reference_crowd.PathCorridor,
    original_boundary: *reference_crowd.LocalBoundary,
    original_grid: *reference_crowd.ProximityGrid,
    original_queue: *reference_crowd.PathQueue,

    /// The polygons of each pair's path, from place `i` to place
    /// `i + pair_count`.
    paths: [pair_count][path_count_max]PolyRef,
    path_counts: [pair_count]usize,
    /// For each pair, a point up to a metre from the start toward the
    /// path's first corner: where an agent moves in a step.
    steps: [pair_count][3]f32,
    /// For each pair, the last of the path's first `corner_count_max`
    /// corners: where an agent looks for a shortcut.
    lookaheads: [pair_count][3]f32,

    /// Room for what the rows ask of each side.
    corner_points: [corner_count_max][3]f32,
    corner_flags: [corner_count_max]detour.StraightPathFlags,
    original_corner_flags: [corner_count_max]u8,
    corner_refs: [corner_count_max]PolyRef,
    neighbours: [neighbour_count_max]u16,
    result_path: [path_count_max]PolyRef,

    /// Attaches every part of each side to `world` and finds the paths.
    /// `self` must not move afterwards. Free with `deinit`.
    fn init(self: *PartsContext, world: *support.World, places: *const Places) !void {
        const gpa = std.heap.c_allocator;
        self.places = places;

        self.query = .empty;
        if (self.query.init(gpa, &world.nav, node_count_max).failed) {
            return error.QueryInitFailed;
        }
        errdefer self.query.deinit(gpa);
        self.filter = .{};
        self.corridor = .empty;
        if (!self.corridor.init(gpa, path_count_max)) return error.OutOfMemory;
        errdefer self.corridor.deinit(gpa);
        self.boundary = .empty;
        self.grid = .empty;
        const grid_pool_size = crowd_agent_count_max * 4;
        if (!self.grid.init(gpa, grid_pool_size, agent_radius * 3)) return error.OutOfMemory;
        errdefer self.grid.deinit(gpa);
        self.queue = .empty;
        if (!self.queue.init(gpa, path_count_max, node_count_max, &world.nav)) {
            return error.OutOfMemory;
        }
        errdefer self.queue.deinit(gpa);

        self.original_query = reference.ref_query_alloc() orelse return error.OutOfMemory;
        errdefer reference.ref_query_free(self.original_query);
        const attached: detour.Status = @bitCast(
            reference.ref_query_init(self.original_query, world.reference_nav, node_count_max),
        );
        if (attached.failed) return error.QueryInitFailed;
        self.original_filter = reference.ref_filter_create() orelse return error.OutOfMemory;
        errdefer reference.ref_filter_destroy(self.original_filter);
        self.original_corridor = reference_crowd.ref_corridor_create() orelse
            return error.OutOfMemory;
        errdefer reference_crowd.ref_corridor_destroy(self.original_corridor);
        if (!reference_crowd.ref_corridor_init(self.original_corridor, path_count_max)) {
            return error.OutOfMemory;
        }
        self.original_boundary = reference_crowd.ref_local_boundary_create() orelse
            return error.OutOfMemory;
        errdefer reference_crowd.ref_local_boundary_destroy(self.original_boundary);
        self.original_grid = reference_crowd.ref_prox_grid_alloc() orelse return error.OutOfMemory;
        errdefer reference_crowd.ref_prox_grid_free(self.original_grid);
        if (!reference_crowd.ref_prox_grid_init(
            self.original_grid,
            grid_pool_size,
            agent_radius * 3,
        )) return error.OutOfMemory;
        self.original_queue = reference_crowd.ref_path_queue_create() orelse
            return error.OutOfMemory;
        errdefer reference_crowd.ref_path_queue_destroy(self.original_queue);
        if (!reference_crowd.ref_path_queue_init(
            self.original_queue,
            path_count_max,
            node_count_max,
            world.reference_nav,
        )) return error.OutOfMemory;

        self.corner_points = @splat(@splat(0));
        self.corner_flags = @splat(.{});
        self.original_corner_flags = @splat(0);
        self.corner_refs = @splat(0);
        self.neighbours = @splat(0);
        self.result_path = @splat(0);

        // The paths, and where an agent on each would step and look, from
        // the original's query and corridor.
        for (0..pair_count) |i| {
            var path_count: i32 = 0;
            const status: detour.Status = @bitCast(reference.ref_query_find_path(
                self.original_query,
                places.refs[i],
                places.refs[i + pair_count],
                &places.points[i],
                &places.points[i + pair_count],
                self.original_filter,
                &self.paths[i],
                &path_count,
                path_count_max,
            ));
            if (status.failed) return error.NoPath;
            self.path_counts[i] = @intCast(path_count);

            self.originalSetCorridor(i);
            const corner_count = reference_crowd.ref_corridor_find_corners(
                self.original_corridor,
                @ptrCast(&self.corner_points),
                &self.original_corner_flags,
                &self.corner_refs,
                corner_count_max,
                self.original_query,
                self.original_filter,
            );
            if (corner_count == 0) return error.NoCorner;
            const start = places.points[i];
            const corner = self.corner_points[0];
            const offset: [3]f32 = .{
                corner[0] - start[0],
                corner[1] - start[1],
                corner[2] - start[2],
            };
            const length = @sqrt(offset[0] * offset[0] + offset[2] * offset[2]);
            const scale: f32 = if (length > 1.0) 1.0 / length else 1.0;
            self.steps[i] = .{
                start[0] + offset[0] * scale,
                start[1] + offset[1] * scale,
                start[2] + offset[2] * scale,
            };
            self.lookaheads[i] = self.corner_points[@intCast(corner_count - 1)];
        }
    }

    fn deinit(self: *PartsContext) void {
        const gpa = std.heap.c_allocator;
        reference_crowd.ref_path_queue_destroy(self.original_queue);
        reference_crowd.ref_prox_grid_free(self.original_grid);
        reference_crowd.ref_local_boundary_destroy(self.original_boundary);
        reference_crowd.ref_corridor_destroy(self.original_corridor);
        reference.ref_filter_destroy(self.original_filter);
        reference.ref_query_free(self.original_query);
        self.queue.deinit(gpa);
        self.grid.deinit(gpa);
        self.corridor.deinit(gpa);
        self.query.deinit(gpa);
        self.* = undefined;
    }

    /// Sets the original's corridor to pair `i`'s path, at its start.
    fn originalSetCorridor(self: *PartsContext, i: usize) void {
        const reset = opaqueFn(reference_crowd.ref_corridor_reset);
        const setCorridor = opaqueFn(reference_crowd.ref_corridor_set_corridor);
        const places = self.places;
        reset(
            self.original_corridor,
            places.refs[i],
            &places.points[i],
        );
        setCorridor(
            self.original_corridor,
            &places.points[i + pair_count],
            &self.paths[i],
            @intCast(self.path_counts[i]),
        );
    }

    /// Sets the port's corridor to pair `i`'s path, at its start.
    fn portSetCorridor(self: *PartsContext, i: usize) void {
        const reset = opaqueFn(detour.PathCorridor.reset);
        const setCorridor = opaqueFn(detour.PathCorridor.setCorridor);
        const places = self.places;
        reset(&self.corridor, places.refs[i], places.points[i]);
        const path = self.paths[i][0..self.path_counts[i]];
        setCorridor(&self.corridor, places.points[i + pair_count], path);
    }

    /// What the original's corridor adds to a checksum: its position and
    /// the length and ends of its path.
    fn originalCorridorSum(self: *const PartsContext) u64 {
        const getPos = opaqueFn(reference_crowd.ref_corridor_get_pos);
        const getPathCount = opaqueFn(reference_crowd.ref_corridor_get_path_count);
        const getFirstPoly = opaqueFn(reference_crowd.ref_corridor_get_first_poly);
        const getLastPoly = opaqueFn(reference_crowd.ref_corridor_get_last_poly);
        const corridor = self.original_corridor;
        return pointSum(getPos(corridor).*) +%
            countSum(getPathCount(corridor)) +%
            getFirstPoly(corridor) +%
            getLastPoly(corridor);
    }

    fn portCorridorSum(self: *const PartsContext) u64 {
        const getPos = opaqueFn(detour.PathCorridor.getPos);
        const getPathCount = opaqueFn(detour.PathCorridor.getPathCount);
        const getFirstPoly = opaqueFn(detour.PathCorridor.getFirstPoly);
        const getLastPoly = opaqueFn(detour.PathCorridor.getLastPoly);
        const corridor = &self.corridor;
        return pointSum(getPos(corridor)) +% countSum(getPathCount(corridor)) +%
            getFirstPoly(corridor) +% getLastPoly(corridor);
    }

    fn baselineFindCorners(context: *PartsContext) u64 {
        const findCorners = opaqueFn(reference_crowd.ref_corridor_find_corners);
        var sum: u64 = 0;
        for (0..pair_count) |i| {
            context.originalSetCorridor(i);
            const n = findCorners(
                context.original_corridor,
                @ptrCast(&context.corner_points),
                &context.original_corner_flags,
                &context.corner_refs,
                corner_count_max,
                context.original_query,
                context.original_filter,
            );
            sum +%= countSum(n);
            if (n > 0) {
                const last: usize = @intCast(n - 1);
                sum +%= pointSum(context.corner_points[last]) +% context.corner_refs[last];
            }
        }
        return sum;
    }

    fn portFindCorners(context: *PartsContext) u64 {
        const findCorners = opaqueFn(detour.PathCorridor.findCorners);
        var sum: u64 = 0;
        for (0..pair_count) |i| {
            context.portSetCorridor(i);
            const n = findCorners(
                &context.corridor,
                &context.corner_points,
                &context.corner_flags,
                &context.corner_refs,
                &context.query,
                &context.filter,
            );
            sum +%= countSum(n);
            if (n > 0) {
                const last: usize = @intCast(n - 1);
                sum +%= pointSum(context.corner_points[last]) +% context.corner_refs[last];
            }
        }
        return sum;
    }

    fn baselineMovePosition(context: *PartsContext) u64 {
        const movePosition = opaqueFn(reference_crowd.ref_corridor_move_position);
        var sum: u64 = 0;
        for (0..pair_count) |i| {
            context.originalSetCorridor(i);
            const moved = movePosition(
                context.original_corridor,
                &context.steps[i],
                context.original_query,
                context.original_filter,
            );
            sum +%= @intFromBool(moved);
            sum +%= context.originalCorridorSum();
        }
        return sum;
    }

    fn portMovePosition(context: *PartsContext) u64 {
        const movePosition = opaqueFn(detour.PathCorridor.movePosition);
        var sum: u64 = 0;
        for (0..pair_count) |i| {
            context.portSetCorridor(i);
            const moved = movePosition(
                &context.corridor,
                context.steps[i],
                &context.query,
                &context.filter,
            );
            sum +%= @intFromBool(moved);
            sum +%= context.portCorridorSum();
        }
        return sum;
    }

    fn baselineOptimizeVisibility(context: *PartsContext) u64 {
        const optimizePathVisibility = opaqueFn(
            reference_crowd.ref_corridor_optimize_path_visibility,
        );
        var sum: u64 = 0;
        for (0..pair_count) |i| {
            context.originalSetCorridor(i);
            optimizePathVisibility(
                context.original_corridor,
                &context.lookaheads[i],
                path_optimization_range,
                context.original_query,
                context.original_filter,
            );
            sum +%= context.originalCorridorSum();
        }
        return sum;
    }

    fn portOptimizeVisibility(context: *PartsContext) u64 {
        const optimizePathVisibility = opaqueFn(detour.PathCorridor.optimizePathVisibility);
        var sum: u64 = 0;
        for (0..pair_count) |i| {
            context.portSetCorridor(i);
            optimizePathVisibility(
                &context.corridor,
                context.lookaheads[i],
                path_optimization_range,
                &context.query,
                &context.filter,
            );
            sum +%= context.portCorridorSum();
        }
        return sum;
    }

    fn baselineOptimizeTopology(context: *PartsContext) u64 {
        const optimizePathTopology = opaqueFn(reference_crowd.ref_corridor_optimize_path_topology);
        var sum: u64 = 0;
        for (0..pair_count) |i| {
            context.originalSetCorridor(i);
            const changed = optimizePathTopology(
                context.original_corridor,
                context.original_query,
                context.original_filter,
            );
            sum +%= @intFromBool(changed);
            sum +%= context.originalCorridorSum();
        }
        return sum;
    }

    fn portOptimizeTopology(context: *PartsContext) u64 {
        const optimizePathTopology = opaqueFn(detour.PathCorridor.optimizePathTopology);
        var sum: u64 = 0;
        for (0..pair_count) |i| {
            context.portSetCorridor(i);
            const changed = optimizePathTopology(
                &context.corridor,
                &context.query,
                &context.filter,
            );
            sum +%= @intFromBool(changed);
            sum +%= context.portCorridorSum();
        }
        return sum;
    }

    fn baselineBoundary(context: *PartsContext) u64 {
        const update = opaqueFn(reference_crowd.ref_local_boundary_update);
        const getSegmentCount = opaqueFn(reference_crowd.ref_local_boundary_get_segment_count);
        const getSegment = opaqueFn(reference_crowd.ref_local_boundary_get_segment);
        const boundary = context.original_boundary;
        var sum: u64 = 0;
        for (&context.places.points, context.places.refs) |*point, ref| {
            update(
                boundary,
                ref,
                point,
                collision_query_range,
                context.original_query,
                context.original_filter,
            );
            const n = getSegmentCount(boundary);
            sum +%= countSum(n);
            if (n > 0) {
                const segment = getSegment(boundary, n - 1);
                for (segment) |value| sum +%= floatSum(value);
            }
        }
        return sum;
    }

    fn portBoundary(context: *PartsContext) u64 {
        const update = opaqueFn(detour.LocalBoundary.update);
        const getSegmentCount = opaqueFn(detour.LocalBoundary.getSegmentCount);
        const getSegment = opaqueFn(detour.LocalBoundary.getSegment);
        const boundary = &context.boundary;
        var sum: u64 = 0;
        for (&context.places.points, context.places.refs) |*point, ref| {
            update(
                boundary,
                ref,
                point.*,
                collision_query_range,
                &context.query,
                &context.filter,
            );
            const n = getSegmentCount(boundary);
            sum +%= countSum(n);
            if (n > 0) {
                const segment = getSegment(boundary, n - 1);
                sum +%= pointSum(segment[0]) +% pointSum(segment[1]);
            }
        }
        return sum;
    }

    fn baselineGrid(context: *PartsContext) u64 {
        const clear = opaqueFn(reference_crowd.ref_prox_grid_clear);
        const addItem = opaqueFn(reference_crowd.ref_prox_grid_add_item);
        const queryItems = opaqueFn(reference_crowd.ref_prox_grid_query_items);
        const grid = context.original_grid;
        const points = context.places.points[0..crowd_agent_count_max];
        var sum: u64 = 0;
        clear(grid);
        for (points, 0..) |point, i| {
            addItem(
                grid,
                @intCast(i),
                point[0] - agent_radius,
                point[2] - agent_radius,
                point[0] + agent_radius,
                point[2] + agent_radius,
            );
        }
        for (points) |point| {
            const n = queryItems(
                grid,
                point[0] - collision_query_range,
                point[2] - collision_query_range,
                point[0] + collision_query_range,
                point[2] + collision_query_range,
                &context.neighbours,
                neighbour_count_max,
            );
            sum +%= countSum(n);
            if (n > 0) sum +%= context.neighbours[@intCast(n - 1)];
        }
        return sum;
    }

    fn portGrid(context: *PartsContext) u64 {
        const clear = opaqueFn(detour.ProximityGrid.clear);
        const addItem = opaqueFn(detour.ProximityGrid.addItem);
        const queryItems = opaqueFn(detour.ProximityGrid.queryItems);
        const grid = &context.grid;
        const points = context.places.points[0..crowd_agent_count_max];
        var sum: u64 = 0;
        clear(grid);
        for (points, 0..) |point, i| {
            addItem(
                grid,
                @intCast(i),
                point[0] - agent_radius,
                point[2] - agent_radius,
                point[0] + agent_radius,
                point[2] + agent_radius,
            );
        }
        for (points) |point| {
            const n = queryItems(
                grid,
                point[0] - collision_query_range,
                point[2] - collision_query_range,
                point[0] + collision_query_range,
                point[2] + collision_query_range,
                &context.neighbours,
            );
            sum +%= countSum(n);
            if (n > 0) sum +%= context.neighbours[@intCast(n - 1)];
        }
        return sum;
    }

    fn baselineQueue(context: *PartsContext) u64 {
        const request = opaqueFn(reference_crowd.ref_path_queue_request);
        const update = opaqueFn(reference_crowd.ref_path_queue_update);
        const getRequestStatus = opaqueFn(reference_crowd.ref_path_queue_get_request_status);
        const getPathResult = opaqueFn(reference_crowd.ref_path_queue_get_path_result);
        const queue = context.original_queue;
        const places = context.places;
        var sum: u64 = 0;
        var first: usize = 0;
        while (first < pair_count) : (first += queue_request_count) {
            var handles: [queue_request_count]u32 = undefined;
            for (&handles, first..) |*handle, i| {
                handle.* = request(
                    queue,
                    places.refs[i],
                    places.refs[i + pair_count],
                    &places.points[i],
                    &places.points[i + pair_count],
                    context.original_filter,
                );
            }
            var waiting: usize = queue_request_count;
            var read: [queue_request_count]bool = @splat(false);
            for (0..queue_update_count_max) |_| {
                if (waiting == 0) break;
                update(queue, queue_iteration_count);
                for (handles, &read) |handle, *done| {
                    if (done.*) continue;
                    const status: detour.Status = @bitCast(
                        getRequestStatus(queue, handle),
                    );
                    if (status.in_progress) continue;
                    var size: i32 = 0;
                    sum +%= getPathResult(
                        queue,
                        handle,
                        &context.result_path,
                        &size,
                        path_count_max,
                    );
                    sum +%= countSum(size);
                    if (size > 0) sum +%= context.result_path[@intCast(size - 1)];
                    done.* = true;
                    waiting -= 1;
                }
            }
        }
        return sum;
    }

    fn portQueue(context: *PartsContext) u64 {
        const request = opaqueFn(detour.PathQueue.request);
        const update = opaqueFn(detour.PathQueue.update);
        const getRequestStatus = opaqueFn(detour.PathQueue.getRequestStatus);
        const getPathResult = opaqueFn(detour.PathQueue.getPathResult);
        const queue = &context.queue;
        const places = context.places;
        var sum: u64 = 0;
        var first: usize = 0;
        while (first < pair_count) : (first += queue_request_count) {
            var handles: [queue_request_count]u32 = undefined;
            for (&handles, first..) |*handle, i| {
                handle.* = request(
                    queue,
                    places.refs[i],
                    places.refs[i + pair_count],
                    places.points[i],
                    places.points[i + pair_count],
                    &context.filter,
                );
            }
            var waiting: usize = queue_request_count;
            var read: [queue_request_count]bool = @splat(false);
            for (0..queue_update_count_max) |_| {
                if (waiting == 0) break;
                update(queue, queue_iteration_count);
                for (handles, &read) |handle, *done| {
                    if (done.*) continue;
                    if (getRequestStatus(queue, handle).in_progress) continue;
                    var size: usize = 0;
                    sum +%= statusSum(getPathResult(queue, handle, &context.result_path, &size));
                    sum +%= size;
                    if (size > 0) sum +%= context.result_path[size - 1];
                    done.* = true;
                    waiting -= 1;
                }
            }
        }
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

    {
        // The 128 paths between pairs of places, each set into a corridor
        // at its start before the call.
        const context = try bench.gpa.create(PartsContext);
        defer bench.gpa.destroy(context);
        try context.init(&world, &places);
        defer context.deinit();
        const PartRow = struct {
            name: []const u8,
            per: []const u8,
            count: usize,
            baseline: fn (*PartsContext) u64,
            port: fn (*PartsContext) u64,
        };
        const part_rows = [_]PartRow{
            .{
                .name = "PathCorridor.setCorridor + findCorners",
                .per = "path",
                .count = pair_count,
                .baseline = PartsContext.baselineFindCorners,
                .port = PartsContext.portFindCorners,
            },
            // A step of up to a metre toward the first corner.
            .{
                .name = "PathCorridor.setCorridor + movePosition",
                .per = "path",
                .count = pair_count,
                .baseline = PartsContext.baselineMovePosition,
                .port = PartsContext.portMovePosition,
            },
            // A shortcut toward the fourth corner, or the last.
            .{
                .name = "PathCorridor.setCorridor + optimizePathVisibility",
                .per = "path",
                .count = pair_count,
                .baseline = PartsContext.baselineOptimizeVisibility,
                .port = PartsContext.portOptimizeVisibility,
            },
            .{
                .name = "PathCorridor.setCorridor + optimizePathTopology",
                .per = "path",
                .count = pair_count,
                .baseline = PartsContext.baselineOptimizeTopology,
                .port = PartsContext.portOptimizeTopology,
            },
            // The walls within an agent's collision range, at every place.
            .{
                .name = "LocalBoundary.update",
                .per = "call",
                .count = place_count,
                .baseline = PartsContext.baselineBoundary,
                .port = PartsContext.portBoundary,
            },
            // 128 agents added, then each one's neighbours asked for.
            .{
                .name = "ProximityGrid.clear + addItem + queryItems",
                .per = "agent",
                .count = crowd_agent_count_max,
                .baseline = PartsContext.baselineGrid,
                .port = PartsContext.portGrid,
            },
            // The 128 paths asked for eight at a time, each update searching
            // 100 iterations, as a crowd does.
            .{
                .name = "PathQueue.request + update + getPathResult",
                .per = "path",
                .count = pair_count,
                .baseline = PartsContext.baselineQueue,
                .port = PartsContext.portQueue,
            },
        };
        inline for (part_rows) |part_row| {
            try bench.compare(
                .{ .name = part_row.name, .per = part_row.per, .count = part_row.count },
                context,
                part_row.baseline,
                part_row.port,
            );
        }
    }
}
