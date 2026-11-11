//! Differential tests of the crowd: creating one, adding and removing
//! agents, changing their parameters, asking them to move and stepping the
//! whole crowd. After every call the port and the original must hold the
//! same agents, bit for bit, and the same grid, filters, avoidance
//! parameters and path requests.

const std = @import("std");
const detour = @import("detour");
const support = @import("support.zig");
const reference = support.reference;
const reference_crowd = support.reference_crowd;

const testing = std.testing;

const Crowd = detour.Crowd;
const CrowdAgent = detour.CrowdAgent;
const CrowdAgentParams = detour.CrowdAgentParams;
const CrowdAgentDebugInfo = detour.CrowdAgentDebugInfo;
const ObstacleAvoidanceParams = detour.ObstacleAvoidanceParams;
const ObstacleAvoidanceDebug = detour.ObstacleAvoidanceDebug;
const PolyRef = detour.PolyRef;
const TileRef = detour.TileRef;

/// Whether the neighbour and corner entries beyond an agent's counts are
/// compared too. The original leaves the entries of earlier updates there,
/// and a port that moves its entries the same way leaves the same ones. Set
/// this to false to compare only the entries in use.
const compare_unused_entries = true;

/// Something for an agent's user data to point at. Both sides are given
/// the same pointer and must hand the same one back.
var user_tag: u8 = 0;

/// The update steps the fuzz picks from. A step of zero must change nothing
/// that depends on elapsed time.
const fuzz_steps = [_]f32{ 0.0, 1.0 / 60.0, 1.0 / 30.0, 0.1, 0.5 };

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------

/// The original's layout of the port's agent parameters.
fn referenceParams(params: *const CrowdAgentParams) reference_crowd.CrowdAgentParams {
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

/// The original's layout of the port's avoidance parameters.
fn referenceAvoidanceParams(
    params: *const ObstacleAvoidanceParams,
) reference_crowd.ObstacleAvoidanceParams {
    return .{
        .vel_bias = params.vel_bias,
        .weight_des_vel = params.weight_desired_velocity,
        .weight_cur_vel = params.weight_current_velocity,
        .weight_side = params.weight_side,
        .weight_toi = params.weight_time_of_impact,
        .horiz_time = params.horizon_time,
        .grid_size = params.grid_size,
        .adaptive_divs = params.adaptive_divisions,
        .adaptive_rings = params.adaptive_rings,
        .adaptive_depth = params.adaptive_depth,
    };
}

/// The parameters most agents of these tests start with: every part of the
/// update switched on.
fn defaultParams() CrowdAgentParams {
    return .{
        .radius = 0.6,
        .height = 2.0,
        .acceleration_max = 8.0,
        .speed_max = 3.5,
        .collision_query_range = 7.2,
        .path_optimization_range = 18.0,
        .separation_weight = 2.0,
        .update_flags = @bitCast(@as(u8, 0b11111)),
        .obstacle_avoidance_type = 3,
        .query_filter_type = 0,
    };
}

/// Parameters drawn at random from the ranges both sides accept: sizes,
/// ranges and the top speed above zero, the two types inside their tables.
fn randomParams(random: std.Random) CrowdAgentParams {
    return .{
        .radius = 0.2 + random.float(f32) * 0.5,
        .height = 1.0 + random.float(f32) * 2.0,
        .acceleration_max = random.float(f32) * 20.0,
        .speed_max = 0.5 + random.float(f32) * 6.0,
        .collision_query_range = 1.0 + random.float(f32) * 11.0,
        .path_optimization_range = 2.0 + random.float(f32) * 28.0,
        .separation_weight = random.float(f32) * 4.0,
        .update_flags = @bitCast(random.uintLessThan(u8, 32)),
        .obstacle_avoidance_type = random.uintLessThan(u8, 8),
        .query_filter_type = random.uintLessThan(u8, 16),
    };
}

/// The four avoidance presets the original's demo uses, from coarse to
/// fine. Their division counts are odd: an even count makes the original
/// read a sample it never wrote.
const avoidance_presets = [_]ObstacleAvoidanceParams{
    avoidancePreset(5, 2, 1),
    avoidancePreset(5, 2, 2),
    avoidancePreset(7, 2, 3),
    avoidancePreset(7, 3, 3),
};

fn avoidancePreset(divisions: u8, rings: u8, depth: u8) ObstacleAvoidanceParams {
    return .{
        .vel_bias = 0.5,
        .weight_desired_velocity = 2.0,
        .weight_current_velocity = 0.75,
        .weight_side = 0.75,
        .weight_time_of_impact = 2.5,
        .horizon_time = 2.5,
        .grid_size = 33,
        .adaptive_divisions = divisions,
        .adaptive_rings = rings,
        .adaptive_depth = depth,
    };
}

/// Fails unless the two floats have the same bits or are both not a number.
/// Only the avoidance trace needs this: two agents on one spot give a
/// direction that is not a number, and its sign differs between compilers.
fn expectFloatOrNan(expected: f32, actual: f32) !void {
    if (std.math.isNan(expected) and std.math.isNan(actual)) return;
    try support.expectFloat(expected, actual);
}

/// Fails unless the original's parameters and the port's hold the same bits.
fn expectSameParams(
    expected: *const reference_crowd.CrowdAgentParams,
    actual: *const CrowdAgentParams,
) !void {
    errdefer std.debug.print("  agent parameters\n", .{});
    try support.expectFloat(expected.radius, actual.radius);
    try support.expectFloat(expected.height, actual.height);
    try support.expectFloat(expected.max_acceleration, actual.acceleration_max);
    try support.expectFloat(expected.max_speed, actual.speed_max);
    try support.expectFloat(expected.collision_query_range, actual.collision_query_range);
    try support.expectFloat(expected.path_optimization_range, actual.path_optimization_range);
    try support.expectFloat(expected.separation_weight, actual.separation_weight);
    try testing.expectEqual(expected.update_flags, @as(u8, @bitCast(actual.update_flags)));
    try testing.expectEqual(expected.obstacle_avoidance_type, actual.obstacle_avoidance_type);
    try testing.expectEqual(expected.query_filter_type, actual.query_filter_type);
    try testing.expectEqual(expected.user_data, actual.user_data);
}

/// Fails unless the original's avoidance parameters and the port's hold the
/// same bits.
fn expectSameAvoidanceParams(
    expected: *const reference_crowd.ObstacleAvoidanceParams,
    actual: *const ObstacleAvoidanceParams,
) !void {
    errdefer std.debug.print("  avoidance parameters\n", .{});
    try support.expectFloat(expected.vel_bias, actual.vel_bias);
    try support.expectFloat(expected.weight_des_vel, actual.weight_desired_velocity);
    try support.expectFloat(expected.weight_cur_vel, actual.weight_current_velocity);
    try support.expectFloat(expected.weight_side, actual.weight_side);
    try support.expectFloat(expected.weight_toi, actual.weight_time_of_impact);
    try support.expectFloat(expected.horiz_time, actual.horizon_time);
    try testing.expectEqual(expected.grid_size, actual.grid_size);
    try testing.expectEqual(expected.adaptive_divs, actual.adaptive_divisions);
    try testing.expectEqual(expected.adaptive_rings, actual.adaptive_rings);
    try testing.expectEqual(expected.adaptive_depth, actual.adaptive_depth);
}

// ---------------------------------------------------------------------------
// The avoidance trace of one agent, on both sides
// ---------------------------------------------------------------------------

/// The buffer both sides write one agent's avoidance samples to.
const DebugPair = struct {
    port: ObstacleAvoidanceDebug,
    original: *reference_crowd.ObstacleAvoidanceDebugData,

    /// Two buffers of `sample_count_max` samples. Free with `deinit`.
    fn init(self: *DebugPair, gpa: std.mem.Allocator, sample_count_max: i32) !void {
        const original = reference_crowd.ref_oa_debug_alloc() orelse return error.OutOfMemory;
        errdefer reference_crowd.ref_oa_debug_free(original);
        try testing.expect(reference_crowd.ref_oa_debug_init(original, sample_count_max));
        self.* = .{ .port = .empty, .original = original };
        errdefer self.port.deinit(gpa);
        try testing.expect(self.port.init(gpa, sample_count_max));
    }

    fn deinit(self: *DebugPair, gpa: std.mem.Allocator) void {
        self.port.deinit(gpa);
        reference_crowd.ref_oa_debug_free(self.original);
        self.* = undefined;
    }

    /// Forgets the samples of both sides.
    fn reset(self: *DebugPair) void {
        reference_crowd.ref_oa_debug_reset(self.original);
        self.port.reset();
    }

    /// Fails unless both sides hold the same samples in the same order.
    fn expectSame(self: *DebugPair) !void {
        const expected_count = reference_crowd.ref_oa_debug_get_sample_count(self.original);
        const actual_count = self.port.getSampleCount();
        if (expected_count != actual_count) {
            std.debug.print(
                "avoidance samples: original {d}, port {d}\n",
                .{ expected_count, actual_count },
            );
            return error.TestExpectedEqual;
        }
        var i: i32 = 0;
        while (i < actual_count) : (i += 1) {
            errdefer std.debug.print("  avoidance sample {d} of {d}\n", .{ i, actual_count });
            try support.expectPoint(
                reference_crowd.ref_oa_debug_get_sample_velocity(self.original, i).*,
                self.port.getSampleVelocity(i),
            );
            try support.expectFloat(
                reference_crowd.ref_oa_debug_get_sample_size(self.original, i),
                self.port.getSampleSize(i),
            );
            try expectFloatOrNan(
                reference_crowd.ref_oa_debug_get_sample_penalty(self.original, i),
                self.port.getSamplePenalty(i),
            );
            try support.expectFloat(
                reference_crowd.ref_oa_debug_get_sample_desired_velocity_penalty(self.original, i),
                self.port.getSampleDesiredVelocityPenalty(i),
            );
            try support.expectFloat(
                reference_crowd.ref_oa_debug_get_sample_current_velocity_penalty(self.original, i),
                self.port.getSampleCurrentVelocityPenalty(i),
            );
            try expectFloatOrNan(
                reference_crowd.ref_oa_debug_get_sample_preferred_side_penalty(self.original, i),
                self.port.getSamplePreferredSidePenalty(i),
            );
            try support.expectFloat(
                reference_crowd.ref_oa_debug_get_sample_collision_time_penalty(self.original, i),
                self.port.getSampleCollisionTimePenalty(i),
            );
        }
    }
};

// ---------------------------------------------------------------------------
// A crowd on both sides
// ---------------------------------------------------------------------------

/// A polygon and a point on it, to send an agent to.
const Target = struct {
    ref: PolyRef,
    pos: [3]f32,
};

/// A crowd of the port and one of the original, over the two sides of one
/// world. Every call goes to both and their answers are compared. The pair
/// must stay at one address: the port's crowd points into itself.
const CrowdPair = struct {
    gpa: std.mem.Allocator,
    world: *support.World,
    port: Crowd,
    original: *reference_crowd.Crowd,
    /// A query of the original that finds the polygons targets lie on, so
    /// both crowds are always given the same target.
    locator: *reference.NavMeshQuery,
    locator_filter: *reference.QueryFilter,
    /// Room for every active agent of each side and three more.
    port_active: []*CrowdAgent,
    original_active: []i32,

    /// Creates both crowds with `init`. Free with `deinit`.
    fn init(
        self: *CrowdPair,
        gpa: std.mem.Allocator,
        world: *support.World,
        agent_count_max: i32,
        agent_radius_max: f32,
    ) !void {
        const original = reference_crowd.ref_crowd_alloc() orelse return error.OutOfMemory;
        errdefer reference_crowd.ref_crowd_free(original);
        const locator = reference.ref_query_alloc() orelse return error.OutOfMemory;
        errdefer reference.ref_query_free(locator);
        const locator_filter = reference.ref_filter_create() orelse return error.OutOfMemory;
        errdefer reference.ref_filter_destroy(locator_filter);
        const attached = reference.ref_query_init(locator, world.reference_nav, 2048);
        try testing.expect(reference.ref_status_succeeded(attached));

        self.* = .{
            .gpa = gpa,
            .world = world,
            .port = .empty,
            .original = original,
            .locator = locator,
            .locator_filter = locator_filter,
            .port_active = &.{},
            .original_active = &.{},
        };
        errdefer {
            gpa.free(self.port_active);
            gpa.free(self.original_active);
            self.port.deinit(gpa);
        }
        try self.reinit(agent_count_max, agent_radius_max);
    }

    /// Runs `init` on both crowds as they are, new or used.
    fn reinit(self: *CrowdPair, agent_count_max: i32, agent_radius_max: f32) !void {
        const expected = reference_crowd.ref_crowd_init(
            self.original,
            agent_count_max,
            agent_radius_max,
            self.world.reference_nav,
        );
        const actual = self.port.init(
            self.gpa,
            agent_count_max,
            agent_radius_max,
            &self.world.nav,
        );
        try testing.expectEqual(expected, actual);
        try testing.expect(actual);
        const capacity: usize = @intCast(agent_count_max + 3);
        self.port_active = try self.gpa.realloc(self.port_active, capacity);
        self.original_active = try self.gpa.realloc(self.original_active, capacity);
    }

    fn deinit(self: *CrowdPair) void {
        self.gpa.free(self.port_active);
        self.gpa.free(self.original_active);
        self.port.deinit(self.gpa);
        reference_crowd.ref_crowd_free(self.original);
        reference.ref_filter_destroy(self.locator_filter);
        reference.ref_query_free(self.locator);
        self.* = undefined;
    }

    /// The polygon nearest `pos` and the nearest point on it, as the
    /// original finds them; reference 0 when no polygon is near.
    fn findTarget(self: *CrowdPair, pos: [3]f32) Target {
        const half_extents = [3]f32{ 2.0, 4.0, 2.0 };
        var target: Target = .{ .ref = 0, .pos = pos };
        _ = reference.ref_query_find_nearest_poly(
            self.locator,
            &pos,
            &half_extents,
            self.locator_filter,
            &target.ref,
            &target.pos,
        );
        return target;
    }

    /// A target on the ground of the level, drawn at random.
    fn randomTarget(self: *CrowdPair, random: std.Random) Target {
        return self.findTarget(self.world.level.groundPoint(random, 0.3));
    }

    fn addAgent(self: *CrowdPair, pos: [3]f32, params: *const CrowdAgentParams) !i32 {
        const original_params = referenceParams(params);
        const expected = reference_crowd.ref_crowd_add_agent(self.original, &pos, &original_params);
        const actual = self.port.addAgent(pos, params);
        if (expected != actual) {
            std.debug.print("addAgent: original {d}, port {d}\n", .{ expected, actual });
            return error.TestExpectedEqual;
        }
        return actual;
    }

    fn removeAgent(self: *CrowdPair, idx: i32) void {
        reference_crowd.ref_crowd_remove_agent(self.original, idx);
        self.port.removeAgent(idx);
    }

    fn updateAgentParameters(self: *CrowdPair, idx: i32, params: *const CrowdAgentParams) void {
        const original_params = referenceParams(params);
        reference_crowd.ref_crowd_update_agent_parameters(self.original, idx, &original_params);
        self.port.updateAgentParameters(idx, params);
    }

    fn setObstacleAvoidanceParams(
        self: *CrowdPair,
        idx: i32,
        params: *const ObstacleAvoidanceParams,
    ) void {
        const original_params = referenceAvoidanceParams(params);
        reference_crowd.ref_crowd_set_obstacle_avoidance_params(
            self.original,
            idx,
            &original_params,
        );
        self.port.setObstacleAvoidanceParams(idx, params);
    }

    fn requestMoveTarget(self: *CrowdPair, idx: i32, ref: PolyRef, pos: [3]f32) !bool {
        const expected = reference_crowd.ref_crowd_request_move_target(
            self.original,
            idx,
            ref,
            &pos,
        );
        const actual = self.port.requestMoveTarget(idx, ref, pos);
        if (expected != actual) {
            std.debug.print("requestMoveTarget: original {}, port {}\n", .{ expected, actual });
            return error.TestExpectedEqual;
        }
        return actual;
    }

    fn requestMoveVelocity(self: *CrowdPair, idx: i32, velocity: [3]f32) !bool {
        const expected = reference_crowd.ref_crowd_request_move_velocity(
            self.original,
            idx,
            &velocity,
        );
        const actual = self.port.requestMoveVelocity(idx, velocity);
        if (expected != actual) {
            std.debug.print("requestMoveVelocity: original {}, port {}\n", .{ expected, actual });
            return error.TestExpectedEqual;
        }
        return actual;
    }

    fn resetMoveTarget(self: *CrowdPair, idx: i32) !bool {
        const expected = reference_crowd.ref_crowd_reset_move_target(self.original, idx);
        const actual = self.port.resetMoveTarget(idx);
        if (expected != actual) {
            std.debug.print("resetMoveTarget: original {}, port {}\n", .{ expected, actual });
            return error.TestExpectedEqual;
        }
        return actual;
    }

    /// Sets one area cost of filter `filter` on both sides.
    fn setFilterAreaCost(self: *CrowdPair, filter: i32, area: i32, cost: f32) !void {
        const original = reference_crowd.ref_crowd_get_editable_filter(self.original, filter) orelse
            return error.TestUnexpectedResult;
        const port = self.port.getEditableFilter(filter) orelse return error.TestUnexpectedResult;
        reference.ref_filter_set_area_cost(original, area, cost);
        port.area_cost[@intCast(area)] = cost;
    }

    /// Sets the flags filter `filter` asks for and refuses, on both sides.
    fn setFilterFlags(self: *CrowdPair, filter: i32, include: u16, exclude: u16) !void {
        const original = reference_crowd.ref_crowd_get_editable_filter(self.original, filter) orelse
            return error.TestUnexpectedResult;
        const port = self.port.getEditableFilter(filter) orelse return error.TestUnexpectedResult;
        reference.ref_filter_set_include_flags(original, include);
        reference.ref_filter_set_exclude_flags(original, exclude);
        port.include_flags = include;
        port.exclude_flags = exclude;
    }

    /// Steps both crowds by `dt` with no trace.
    fn update(self: *CrowdPair, dt: f32) void {
        reference_crowd.ref_crowd_update(self.original, dt, -1, null);
        self.port.update(dt, null);
    }

    /// Steps both crowds by `dt` and has both trace the agent at place
    /// `trace_idx` among the active ones, which may be no agent's place.
    /// The trace's two points and, with `debug`, its avoidance samples must
    /// come out the same.
    fn updateTraced(self: *CrowdPair, dt: f32, trace_idx: i32, debug: ?*DebugPair) !void {
        var original_info: reference_crowd.CrowdDebugInfo = .{
            .opt_start = @splat(0.0),
            .opt_end = @splat(0.0),
            .vod = if (debug) |pair| pair.original else null,
        };
        var port_info: CrowdAgentDebugInfo = .{
            .idx = trace_idx,
            .opt_start = @splat(0.0),
            .opt_end = @splat(0.0),
            .vod = if (debug) |pair| &pair.port else null,
        };
        reference_crowd.ref_crowd_update(self.original, dt, trace_idx, &original_info);
        self.port.update(dt, &port_info);

        errdefer std.debug.print("  trace of the agent at place {d}\n", .{trace_idx});
        try testing.expectEqual(trace_idx, port_info.idx);
        try support.expectPoint(original_info.opt_start, port_info.opt_start);
        try support.expectPoint(original_info.opt_end, port_info.opt_end);
        if (debug) |pair| try pair.expectSame();
    }

    /// Fails unless agent slot `idx` holds the same on both sides: every
    /// field of a slot, in use or not, its corridor, its boundary and its
    /// off-mesh animation.
    fn expectSameAgent(self: *CrowdPair, idx: i32) !void {
        errdefer std.debug.print("  agent slot {d}\n", .{idx});
        var snapshot: reference_crowd.AgentSnapshot = undefined;
        try testing.expect(reference_crowd.ref_crowd_agent_snapshot(self.original, idx, &snapshot));
        const agent = self.port.getAgent(idx) orelse {
            std.debug.print("getAgent: the port has no slot {d}\n", .{idx});
            return error.TestExpectedEqual;
        };

        try expectField("active", snapshot.active, agent.active);
        try expectField("state", snapshot.state, @backingInt(agent.state));
        try expectField("partial", snapshot.partial, agent.partial);
        try expectSameParams(&snapshot.params, &agent.params);

        try expectNamedFloat(
            "topology time",
            snapshot.topology_opt_time,
            agent.topology_optimization_time,
        );
        try expectNamedFloat("desired speed", snapshot.desired_speed, agent.desired_speed);
        try expectNamedPoint("position", snapshot.npos, agent.position);
        try expectNamedPoint("displacement", snapshot.disp, agent.displacement);
        try expectNamedPoint("desired velocity", snapshot.dvel, agent.desired_velocity);
        try expectNamedPoint("new velocity", snapshot.nvel, agent.new_velocity);
        try expectNamedPoint("velocity", snapshot.vel, agent.velocity);

        try expectField("target state", snapshot.target_state, @backingInt(agent.target_state));
        try expectField("target polygon", snapshot.target_ref, agent.target_ref);
        try expectNamedPoint("target position", snapshot.target_pos, agent.target_pos);
        try expectField("path request", snapshot.target_pathq_ref, agent.target_path_queue_ref);
        try expectField("replanning", snapshot.target_replan, agent.target_replan);
        try expectNamedFloat("replan time", snapshot.target_replan_time, agent.target_replan_time);

        try self.expectSameCorridor(&snapshot, agent);
        try expectSameBoundary(&snapshot, agent);
        try expectSameNeighbours(&snapshot, agent);
        try expectSameCorners(&snapshot, agent);

        const animation = self.port.getAgentAnimation(idx);
        try expectField("animation active", snapshot.anim_active, animation.active);
        if (animation.active) {
            try expectNamedPoint("animation origin", snapshot.anim_init_pos, animation.init_pos);
            try expectNamedPoint("animation start", snapshot.anim_start_pos, animation.start_pos);
            try expectNamedPoint("animation end", snapshot.anim_end_pos, animation.end_pos);
            try expectField("animation polygon", snapshot.anim_poly_ref, animation.poly_ref);
            try expectNamedFloat("animation time", snapshot.anim_t, animation.t);
            try expectNamedFloat("animation length", snapshot.anim_tmax, animation.t_max);
        }
    }

    /// Fails unless the corridor of the port's `agent` holds the path, the
    /// position and the target of the original's.
    fn expectSameCorridor(
        self: *CrowdPair,
        snapshot: *const reference_crowd.AgentSnapshot,
        agent: *const CrowdAgent,
    ) !void {
        errdefer std.debug.print("  corridor\n", .{});
        try expectField("path length", snapshot.corridor_npath, agent.corridor.getPathCount());
        const count: usize = @intCast(snapshot.corridor_npath);
        const path = agent.corridor.getPath();
        try testing.expectEqual(count, path.len);
        for (snapshot.corridor_path[0..count], path, 0..) |expected, actual, i| {
            if (expected == actual) continue;
            std.debug.print(
                "path polygon {d} of {d}: original {d}, port {d}\n",
                .{ i, count, expected, actual },
            );
            return error.TestExpectedEqual;
        }
        try expectNamedPoint("position", snapshot.corridor_pos, agent.corridor.getPos());
        try expectNamedPoint("target", snapshot.corridor_target, agent.corridor.getTarget());

        // The same answers through the original's own corridor getters.
        const offset = @intFromPtr(agent) - @intFromPtr(self.port.agents.ptr);
        const slot: i32 = @intCast(offset / @sizeOf(CrowdAgent));
        const original_agent = reference_crowd.ref_crowd_get_agent(self.original, slot) orelse
            return error.TestUnexpectedResult;
        const corridor = reference_crowd.ref_crowd_agent_corridor(original_agent);
        try expectField(
            "first polygon",
            reference_crowd.ref_corridor_get_first_poly(corridor),
            agent.corridor.getFirstPoly(),
        );
        try expectField(
            "last polygon",
            reference_crowd.ref_corridor_get_last_poly(corridor),
            agent.corridor.getLastPoly(),
        );
        try expectField(
            "path count",
            reference_crowd.ref_corridor_get_path_count(corridor),
            agent.corridor.getPathCount(),
        );
        try expectNamedPoint(
            "corridor position getter",
            reference_crowd.ref_corridor_get_pos(corridor).*,
            agent.corridor.getPos(),
        );
        try expectNamedPoint(
            "corridor target getter",
            reference_crowd.ref_corridor_get_target(corridor).*,
            agent.corridor.getTarget(),
        );
    }

    /// Fails unless the boundary of the port's `agent` holds the centre and
    /// the wall segments of the original's.
    fn expectSameBoundary(
        snapshot: *const reference_crowd.AgentSnapshot,
        agent: *const CrowdAgent,
    ) !void {
        errdefer std.debug.print("  boundary\n", .{});
        try expectNamedPoint("centre", snapshot.boundary_center, agent.boundary.getCenter());
        try expectField(
            "segment count",
            snapshot.boundary_nsegments,
            agent.boundary.getSegmentCount(),
        );
        var i: i32 = 0;
        while (i < snapshot.boundary_nsegments) : (i += 1) {
            errdefer std.debug.print("  segment {d}\n", .{i});
            const expected = &snapshot.boundary_segments[@intCast(i)];
            const actual = agent.boundary.getSegment(i);
            try support.expectPoint(expected[0..3].*, actual[0]);
            try support.expectPoint(expected[3..6].*, actual[1]);
        }
    }

    /// Fails unless the port's `agent` lists the neighbours of the
    /// original's, with the same squared distances.
    fn expectSameNeighbours(
        snapshot: *const reference_crowd.AgentSnapshot,
        agent: *const CrowdAgent,
    ) !void {
        try expectField("neighbour count", snapshot.nneis, agent.neighbour_count);
        const used: usize = @intCast(snapshot.nneis);
        const end: usize = if (compare_unused_entries) snapshot.neis.len else used;
        for (snapshot.neis[0..end], agent.neighbours[0..end], 0..) |expected, actual, i| {
            errdefer std.debug.print(
                "  neighbour {d}, {d} in use{s}\n",
                .{ i, used, unusedNote(i >= used) },
            );
            try testing.expectEqual(expected.idx, actual.idx);
            try support.expectFloat(expected.dist, actual.dist_sqr);
        }
    }

    /// Fails unless the port's `agent` steers to the corners of the
    /// original's.
    fn expectSameCorners(
        snapshot: *const reference_crowd.AgentSnapshot,
        agent: *const CrowdAgent,
    ) !void {
        try expectField("corner count", snapshot.ncorners, agent.corner_count);
        const used: usize = @intCast(snapshot.ncorners);
        const end: usize = if (compare_unused_entries) snapshot.corner_polys.len else used;
        for (0..end) |c| {
            errdefer std.debug.print(
                "  corner {d}, {d} in use{s}\n",
                .{ c, used, unusedNote(c >= used) },
            );
            try support.expectPoint(snapshot.corner_verts[c * 3 ..][0..3].*, agent.corner_verts[c]);
            const flags: u8 = @bitCast(agent.corner_flags[c]);
            try testing.expectEqual(snapshot.corner_flags[c], flags);
            try testing.expectEqual(snapshot.corner_polys[c], agent.corner_polys[c]);
        }
    }

    /// Fails unless both sides list the same active agents, in the same
    /// order, when given room for `capacity` of them.
    fn expectSameActiveAgents(self: *CrowdPair, capacity: usize) !void {
        errdefer std.debug.print("  active agents, room for {d}\n", .{capacity});
        const expected = reference_crowd.ref_crowd_get_active_agents(
            self.original,
            self.original_active.ptr,
            @intCast(capacity),
        );
        const actual = self.port.getActiveAgents(self.port_active[0..capacity]);
        try testing.expectEqual(expected, actual);
        const count: usize = @intCast(actual);
        for (self.original_active[0..count], self.port_active[0..count]) |original_slot, agent| {
            const offset = @intFromPtr(agent) - @intFromPtr(self.port.agents.ptr);
            const slot: i32 = @intCast(offset / @sizeOf(CrowdAgent));
            try testing.expectEqual(original_slot, slot);
            try testing.expect(agent.active);
        }
    }

    /// The number of agents both sides call active.
    fn activeCount(self: *CrowdPair) !i32 {
        const expected = reference_crowd.ref_crowd_get_active_agents(
            self.original,
            self.original_active.ptr,
            @intCast(self.original_active.len),
        );
        const actual = self.port.getActiveAgents(self.port_active);
        try testing.expectEqual(expected, actual);
        return actual;
    }

    /// Fails unless the two grids hold the same items: the same bounds and
    /// cell size, the same count in every cell when the bounds are small,
    /// and the same answers around every active agent.
    fn expectSameGrid(self: *CrowdPair) !void {
        errdefer std.debug.print("  proximity grid\n", .{});
        const original = reference_crowd.ref_crowd_get_grid(self.original) orelse
            return error.TestUnexpectedResult;
        const port = self.port.getGrid();
        const cell_size = port.getCellSize();
        try support.expectFloat(reference_crowd.ref_prox_grid_get_cell_size(original), cell_size);
        const bounds = port.getBounds().*;
        try testing.expectEqualSlices(
            i32,
            reference_crowd.ref_prox_grid_get_bounds(original),
            &bounds,
        );

        // An empty grid has its bounds inside out, and nothing to walk.
        const width: i64 = @as(i64, bounds[2]) - bounds[0] + 1;
        const height: i64 = @as(i64, bounds[3]) - bounds[1] + 1;
        if (width > 0 and height > 0 and width * height <= 4096) {
            var y = bounds[1];
            while (y <= bounds[3]) : (y += 1) {
                var x = bounds[0];
                while (x <= bounds[2]) : (x += 1) try expectSameCell(original, port, x, y);
            }
        }

        const count: usize = @intCast(try self.activeCount());
        for (self.port_active[0..count]) |agent| {
            const scaled_x: f32 = @floor(agent.position[0] / cell_size);
            const scaled_y: f32 = @floor(agent.position[2] / cell_size);
            const cell_x: i32 = @trunc(scaled_x);
            const cell_y: i32 = @trunc(scaled_y);
            var y = cell_y - 1;
            while (y <= cell_y + 1) : (y += 1) {
                var x = cell_x - 1;
                while (x <= cell_x + 1) : (x += 1) try expectSameCell(original, port, x, y);
            }

            const reach = cell_size * 1.5;
            var expected_ids: [64]u16 = @splat(0xffff);
            var actual_ids: [64]u16 = @splat(0xffff);
            const expected = reference_crowd.ref_prox_grid_query_items(
                original,
                agent.position[0] - reach,
                agent.position[2] - reach,
                agent.position[0] + reach,
                agent.position[2] + reach,
                &expected_ids,
                expected_ids.len,
            );
            const actual = port.queryItems(
                agent.position[0] - reach,
                agent.position[2] - reach,
                agent.position[0] + reach,
                agent.position[2] + reach,
                &actual_ids,
            );
            try testing.expectEqual(expected, actual);
            try testing.expectEqualSlices(u16, &expected_ids, &actual_ids);
        }
    }

    fn expectSameCell(
        original: *const reference_crowd.ProximityGrid,
        port: *const detour.ProximityGrid,
        x: i32,
        y: i32,
    ) !void {
        const expected = reference_crowd.ref_prox_grid_get_item_count_at(original, x, y);
        const actual = port.getItemCountAt(x, y);
        if (expected == actual) return;
        std.debug.print(
            "getItemCountAt({d}, {d}): original {d}, port {d}\n",
            .{ x, y, expected, actual },
        );
        return error.TestExpectedEqual;
    }

    /// Fails unless both sides hold the same sixteen filters, and neither
    /// has one outside the table.
    fn expectSameFilters(self: *CrowdPair) !void {
        var i: i32 = -1;
        while (i <= reference_crowd.max_query_filter_types) : (i += 1) {
            errdefer std.debug.print("  filter {d}\n", .{i});
            const expected = reference_crowd.ref_crowd_get_filter(self.original, i);
            const actual = self.port.getFilter(i);
            try testing.expectEqual(expected == null, actual == null);
            const original = expected orelse continue;
            const port = actual orelse continue;
            const editable: ?*const detour.QueryFilter = self.port.getEditableFilter(i);
            try testing.expect(editable == port);
            try testing.expectEqual(
                reference.ref_filter_get_include_flags(original),
                port.include_flags,
            );
            try testing.expectEqual(
                reference.ref_filter_get_exclude_flags(original),
                port.exclude_flags,
            );
            for (port.area_cost, 0..) |cost, area| {
                errdefer std.debug.print("  area {d}\n", .{area});
                const original_cost = reference.ref_filter_get_area_cost(original, @intCast(area));
                try support.expectFloat(original_cost, cost);
            }
        }
        for ([_]i32{ -1, reference_crowd.max_query_filter_types }) |outside| {
            try testing.expect(
                reference_crowd.ref_crowd_get_editable_filter(self.original, outside) == null,
            );
            try testing.expect(self.port.getEditableFilter(outside) == null);
        }
    }

    /// Fails unless both sides hold the same eight sets of avoidance
    /// parameters, and neither has one outside the table.
    fn expectSameAvoidanceTable(self: *CrowdPair) !void {
        var i: i32 = -1;
        while (i <= reference_crowd.max_obstacle_avoidance_params) : (i += 1) {
            errdefer std.debug.print("  avoidance parameters {d}\n", .{i});
            const expected = reference_crowd.ref_crowd_get_obstacle_avoidance_params(
                self.original,
                i,
            );
            const actual = self.port.getObstacleAvoidanceParams(i);
            try testing.expectEqual(expected == null, actual == null);
            try expectSameAvoidanceParams(expected orelse continue, actual orelse continue);
        }
    }

    /// Fails unless the path queues of both sides say the same of every
    /// request an agent holds, and of the handles next to it. Handle 0 is
    /// left out: it matches a free entry of the queue, whose status the
    /// original never set.
    fn expectSamePathRequests(self: *CrowdPair) !void {
        const original = reference_crowd.ref_crowd_get_path_queue(self.original) orelse
            return error.TestUnexpectedResult;
        const port = self.port.getPathQueue();
        for (self.port.agents, 0..) |*agent, slot| {
            const request = agent.target_path_queue_ref;
            if (request == 0) continue;
            for ([_]u32{ request -% 1, request, request +% 1 }) |handle| {
                if (handle == 0) continue;
                errdefer std.debug.print(
                    "  path request {d}, near the request of agent slot {d}\n",
                    .{ handle, slot },
                );
                try support.expectStatus(
                    reference_crowd.ref_path_queue_get_request_status(original, handle),
                    port.getRequestStatus(handle),
                );
            }
        }
    }

    /// Fails unless the crowd's query and the path queue's query are
    /// attached to the world's mesh on both sides.
    fn expectSameQueries(self: *CrowdPair) !void {
        errdefer std.debug.print("  queries\n", .{});
        const original_query = reference_crowd.ref_crowd_get_nav_mesh_query(self.original) orelse
            return error.TestUnexpectedResult;
        const original_queue = reference_crowd.ref_crowd_get_path_queue(self.original) orelse
            return error.TestUnexpectedResult;
        const queue_query = reference_crowd.ref_path_queue_get_nav_query(original_queue) orelse
            return error.TestUnexpectedResult;
        const original_mesh: *const reference.NavMesh = self.world.reference_nav;
        try testing.expect(
            reference.ref_query_get_attached_nav_mesh(original_query) == original_mesh,
        );
        try testing.expect(reference.ref_query_get_attached_nav_mesh(queue_query) == original_mesh);

        const port_mesh: *const detour.NavMesh = &self.world.nav;
        try testing.expect(self.port.getNavMeshQuery().getAttachedNavMesh() == port_mesh);
        try testing.expect(self.port.path_queue.getNavQuery().getAttachedNavMesh() == port_mesh);
    }

    /// Fails unless the two crowds are in the same state: every agent slot,
    /// the active agents, the grid, the filters, the avoidance parameters,
    /// the path requests and the plain getters.
    fn expectSameState(self: *CrowdPair) !void {
        const count = reference_crowd.ref_crowd_get_agent_count(self.original);
        try expectField("agent count", count, self.port.getAgentCount());
        try testing.expectEqual(@as(usize, @intCast(count)), self.port.agents.len);

        var idx: i32 = 0;
        while (idx < count) : (idx += 1) try self.expectSameAgent(idx);
        for ([_]i32{ -1, count, count + 7 }) |outside| {
            errdefer std.debug.print("  agent slot {d}, outside the crowd\n", .{outside});
            var snapshot: reference_crowd.AgentSnapshot = undefined;
            try testing.expect(
                !reference_crowd.ref_crowd_agent_snapshot(self.original, outside, &snapshot),
            );
            try testing.expect(reference_crowd.ref_crowd_get_agent(self.original, outside) == null);
            try testing.expect(
                reference_crowd.ref_crowd_get_editable_agent(self.original, outside) == null,
            );
            try testing.expect(self.port.getAgent(outside) == null);
            try testing.expect(self.port.getEditableAgent(outside) == null);
        }

        const slots: usize = @intCast(count);
        for ([_]usize{ 0, 1, slots / 2, slots, slots + 3 }) |capacity| {
            try self.expectSameActiveAgents(capacity);
        }

        try expectNamedPoint(
            "query half extents",
            reference_crowd.ref_crowd_get_query_half_extents(self.original).*,
            self.port.getQueryHalfExtents(),
        );
        try expectNamedPoint(
            "query extents",
            reference_crowd.ref_crowd_get_query_extents(self.original).*,
            self.port.getQueryExtents(),
        );
        try expectField(
            "velocity sample count",
            reference_crowd.ref_crowd_get_velocity_sample_count(self.original),
            self.port.getVelocitySampleCount(),
        );

        try self.expectSameGrid();
        try self.expectSameFilters();
        try self.expectSameAvoidanceTable();
        try self.expectSamePathRequests();
        try self.expectSameQueries();
    }
};

fn unusedNote(unused: bool) []const u8 {
    return if (unused) " (not in use: left by an earlier update)" else "";
}

fn expectField(comptime name: []const u8, expected: anytype, actual: anytype) !void {
    errdefer std.debug.print("  " ++ name ++ "\n", .{});
    try testing.expectEqual(expected, actual);
}

fn expectNamedFloat(comptime name: []const u8, expected: f32, actual: f32) !void {
    errdefer std.debug.print("  " ++ name ++ "\n", .{});
    try support.expectFloat(expected, actual);
}

fn expectNamedPoint(comptime name: []const u8, expected: [3]f32, actual: [3]f32) !void {
    errdefer std.debug.print("  " ++ name ++ "\n", .{});
    try support.expectPoint(expected, actual);
}

// ---------------------------------------------------------------------------
// Worlds
// ---------------------------------------------------------------------------

/// What the tiled worlds with one kind of polygon are built with.
const connected_options: support.LevelOptions = .{ .off_mesh_connection_count = 24, .seed = 71 };

/// What the tiled worlds whose polygons differ in area and flags are built
/// with, so the crowd's filters have something to tell apart.
const varied_options: support.LevelOptions = .{
    .off_mesh_connection_count = 12,
    .vary_polygons = true,
    .seed = 72,
};

/// Tiles of four cells and triangles for polygons: as many polygons along
/// a path as these levels give.
const small_tile_config: support.RecastConfig = blk: {
    var config = support.tiled_config;
    config.tile_size = 4;
    config.verts_per_poly = 3;
    break :blk config;
};

/// Fails when a scenario did not reach the case it was written for. That is
/// a fault of the test's inputs, not a difference between the two sides.
fn expectReached(reached: bool, comptime what: []const u8) !void {
    if (reached) return;
    std.debug.print("the scenario never reached its case: " ++ what ++ "\n", .{});
    return error.ScenarioMissedItsCase;
}

/// The two ends of an off-mesh connection.
const Connection = [2][3]f32;

/// Copies the off-mesh connections of the world's tiles to `connections`,
/// as many as fit, and returns how many that is.
fn collectConnections(world: *const support.World, connections: []Connection) usize {
    var count: usize = 0;
    for (world.nav.tiles) |*tile| {
        if (tile.header == null) continue;
        for (tile.off_mesh_connections) |*connection| {
            if (count == connections.len) return count;
            connections[count] = connection.pos;
            count += 1;
        }
    }
    return count;
}

/// Whether some off-mesh connection of the world starts and ends at one
/// spot of the map. Crossing such a connection takes no time, and an update
/// of zero seconds then divides zero by zero on both sides.
fn hasDegenerateConnection(world: *const support.World) bool {
    for (world.nav.tiles) |*tile| {
        if (tile.header == null) continue;
        for (tile.off_mesh_connections) |*connection| {
            const poly = &tile.polys[connection.poly];
            const start = tile.verts[poly.verts[0]];
            const end = tile.verts[poly.verts[1]];
            const dx = end[0] - start[0];
            const dz = end[2] - start[2];
            if (dx * dx + dz * dz < 1e-6) return true;
        }
    }
    return false;
}

/// Removes tile (`x`, `y`) from both meshes of `world`. Returns the tile's
/// reference, or 0 when there was no such tile.
fn removeTileAt(world: *support.World, x: i32, y: i32) !TileRef {
    const expected_ref = reference.ref_nav_mesh_get_tile_ref_at(world.reference_nav, x, y, 0);
    const actual_ref = world.nav.getTileRefAt(x, y, 0);
    try testing.expectEqual(expected_ref, actual_ref);
    if (actual_ref == 0) return 0;
    const expected = reference.ref_nav_mesh_remove_tile(
        world.reference_nav,
        expected_ref,
        null,
        null,
    );
    const actual = world.nav.removeTile(actual_ref, null);
    try support.expectStatus(expected, actual);
    return actual_ref;
}

/// The column and the row of the tile under `pos`.
fn tileUnder(world: *const support.World, pos: [3]f32) [2]i32 {
    const tile_size: f32 = @floatFromInt(world.level.config.tile_size);
    const tile_width = tile_size * world.level.config.cell_size;
    const column: f32 = @floor((pos[0] - world.level.mesh.bounds_min[0]) / tile_width);
    const row: f32 = @floor((pos[2] - world.level.mesh.bounds_min[2]) / tile_width);
    return .{ @trunc(column), @trunc(row) };
}

/// Whether any agent slot of the port's crowd is crossing an off-mesh
/// connection.
fn anyAnimation(pair: *CrowdPair) bool {
    var idx: i32 = 0;
    while (idx < pair.port.getAgentCount()) : (idx += 1) {
        if (pair.port.getAgentAnimation(idx).active) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------

test "init: new crowds of several sizes hold the same defaults" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, support.nav_test_obj, support.solo_config, .{ .seed = 61 });
    defer world.deinit();

    const Size = struct { agent_count: i32, radius: f32 };
    const sizes = [_]Size{
        .{ .agent_count = 1, .radius = 0.6 },
        .{ .agent_count = 2, .radius = 0.3 },
        .{ .agent_count = 25, .radius = 0.6 },
        .{ .agent_count = 300, .radius = 2.0 },
        .{ .agent_count = 2000, .radius = 0.5 },
    };
    for (sizes) |size| {
        errdefer std.debug.print(
            "  a crowd of {d} agents, radius up to {d}\n",
            .{ size.agent_count, size.radius },
        );
        var pair: CrowdPair = undefined;
        try pair.init(gpa, &world, size.agent_count, size.radius);
        defer pair.deinit();
        try pair.expectSameState();

        // What the original documents of a new crowd.
        try testing.expectEqual(size.agent_count, pair.port.getAgentCount());
        try testing.expectEqual(@as(i32, 0), try pair.activeCount());
        try testing.expectEqual(@as(i32, 0), pair.port.getVelocitySampleCount());
        const avoidance = pair.port.getObstacleAvoidanceParams(7) orelse
            return error.TestUnexpectedResult;
        try support.expectFloat(0.4, avoidance.vel_bias);
        try support.expectFloat(2.0, avoidance.weight_desired_velocity);
        try support.expectFloat(0.75, avoidance.weight_current_velocity);
        try support.expectFloat(0.75, avoidance.weight_side);
        try support.expectFloat(2.5, avoidance.weight_time_of_impact);
        try support.expectFloat(2.5, avoidance.horizon_time);
        try testing.expectEqual(@as(u8, 33), avoidance.grid_size);
        try testing.expectEqual(@as(u8, 7), avoidance.adaptive_divisions);
        try testing.expectEqual(@as(u8, 2), avoidance.adaptive_rings);
        try testing.expectEqual(@as(u8, 5), avoidance.adaptive_depth);

        // Stepping an empty crowd changes nothing on either side.
        pair.update(1.0 / 30.0);
        try pair.expectSameState();
        try pair.updateTraced(0.0, 0, null);
        try pair.expectSameState();
    }
}

/// Adds `count` agents on the ground and sends each to a place drawn at
/// random. The agents are a little off the ground points, so none stands
/// exactly on the end of an off-mesh connection.
fn populate(pair: *CrowdPair, random: std.Random, count: usize) !void {
    for (0..count) |i| {
        var params = defaultParams();
        params.obstacle_avoidance_type = @intCast(i % 4);
        const idx = try pair.addAgent(pair.world.level.groundPoint(random, 0.3), &params);
        if (idx < 0) continue;
        const target = pair.randomTarget(random);
        _ = try pair.requestMoveTarget(idx, target.ref, target.pos);
    }
}

/// Steps both crowds `count` times by a thirtieth of a second and compares
/// them after every step.
fn stepAndCompare(pair: *CrowdPair, count: usize) !void {
    for (0..count) |step| {
        errdefer std.debug.print("  update {d} of {d}\n", .{ step, count });
        pair.update(1.0 / 30.0);
        try pair.expectSameState();
    }
}

/// Whether any agent slot of the port's crowd holds the handle of a queued
/// path request, answered or not.
fn anyPathRequest(pair: *CrowdPair) bool {
    for (pair.port.agents) |*agent| {
        if (agent.target_path_queue_ref != 0) return true;
    }
    return false;
}

test "init: a used crowd starts over and keeps its filters" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, support.nav_test_obj, support.tiled_config, connected_options);
    defer world.deinit();

    var prng: std.Random.DefaultPrng = .init(62);
    const random = prng.random();
    var pair: CrowdPair = undefined;
    try pair.init(gpa, &world, 8, 0.6);
    defer pair.deinit();

    // Use everything a second init has to clear or to keep: agents, queued
    // path requests, a filter and a set of avoidance parameters.
    try pair.setFilterAreaCost(2, 4, 3.5);
    try pair.setFilterFlags(2, 0x00ff, 0x0100);
    pair.setObstacleAvoidanceParams(1, &avoidance_presets[0]);
    try populate(&pair, random, 8);
    try stepAndCompare(&pair, 5);
    try expectReached(anyPathRequest(&pair), "an agent that queued a path request");

    const Size = struct { agent_count: i32, radius: f32 };
    const sizes = [_]Size{
        .{ .agent_count = 8, .radius = 0.6 },
        .{ .agent_count = 3, .radius = 0.9 },
        .{ .agent_count = 20, .radius = 0.4 },
    };
    for (sizes, 0..) |size, round| {
        errdefer std.debug.print(
            "  init {d} of a used crowd, {d} agents\n",
            .{ round + 2, size.agent_count },
        );
        try pair.reinit(size.agent_count, size.radius);
        try pair.expectSameState();
        try testing.expectEqual(@as(i32, 0), try pair.activeCount());

        // The original keeps the filters and puts the avoidance parameters
        // back to their defaults.
        const filter = reference_crowd.ref_crowd_get_filter(pair.original, 2) orelse
            return error.TestUnexpectedResult;
        try support.expectFloat(3.5, reference.ref_filter_get_area_cost(filter, 4));
        try testing.expectEqual(@as(u16, 0x0100), reference.ref_filter_get_exclude_flags(filter));
        const avoidance = reference_crowd.ref_crowd_get_obstacle_avoidance_params(
            pair.original,
            1,
        ) orelse return error.TestUnexpectedResult;
        try support.expectFloat(0.4, avoidance.vel_bias);
        try testing.expectEqual(@as(u8, 7), avoidance.adaptive_divs);

        // The path requests of the new agents go on from the handles the
        // crowd handed out before.
        try populate(&pair, random, @intCast(size.agent_count));
        try stepAndCompare(&pair, 5);
    }
}

// ---------------------------------------------------------------------------
// addAgent, removeAgent
// ---------------------------------------------------------------------------

test "addAgent and removeAgent: slots fill, free and fill again lowest first" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, support.nav_test_obj, support.solo_config, .{ .seed = 63 });
    defer world.deinit();

    var prng: std.Random.DefaultPrng = .init(63);
    const random = prng.random();
    var pair: CrowdPair = undefined;
    try pair.init(gpa, &world, 12, 0.6);
    defer pair.deinit();

    var slot: i32 = 0;
    while (slot < 12) : (slot += 1) {
        const params = randomParams(random);
        const idx = try pair.addAgent(world.level.groundPoint(random, 0.3), &params);
        try testing.expectEqual(slot, idx);
        try pair.expectSameState();
    }

    // A full crowd takes no more.
    const extra = defaultParams();
    const refused = try pair.addAgent(world.level.groundPoint(random, 0.3), &extra);
    try testing.expectEqual(@as(i32, -1), refused);
    try pair.expectSameState();

    // Indexes outside the crowd remove nothing.
    const outside = [_]i32{ -1, 12, 13, 4096, std.math.minInt(i32), std.math.maxInt(i32) };
    for (outside) |idx| {
        pair.removeAgent(idx);
        try pair.expectSameState();
    }
    try testing.expectEqual(@as(i32, 12), try pair.activeCount());

    // Freed slots are taken lowest first, whatever order they were freed
    // in. Slot 3 is freed twice.
    for ([_]i32{ 7, 3, 0, 3 }) |freed| {
        pair.removeAgent(freed);
        try pair.expectSameState();
    }
    try stepAndCompare(&pair, 3);
    for ([_]i32{ 0, 3, 7 }) |free_slot| {
        const params = randomParams(random);
        const idx = try pair.addAgent(world.level.groundPoint(random, 0.3), &params);
        try testing.expectEqual(free_slot, idx);
        try pair.expectSameState();
    }
    const full = try pair.addAgent(world.level.groundPoint(random, 0.3), &extra);
    try testing.expectEqual(@as(i32, -1), full);

    // Removing every agent leaves an empty crowd that still steps.
    slot = 0;
    while (slot < 12) : (slot += 1) pair.removeAgent(slot);
    try pair.expectSameState();
    try testing.expectEqual(@as(i32, 0), try pair.activeCount());
    try stepAndCompare(&pair, 2);
}

test "addAgent: agents put off the mesh are held the same" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, support.nav_test_obj, support.solo_config, .{ .seed = 64 });
    defer world.deinit();

    var prng: std.Random.DefaultPrng = .init(64);
    const random = prng.random();
    var pair: CrowdPair = undefined;
    try pair.init(gpa, &world, 10, 0.6);
    defer pair.deinit();

    // Far above, far below and far to the side of the ground, at the edge
    // of the box an agent is placed with, and on the ground itself.
    const ground = world.level.groundPoint(random, 0.0);
    const places = [8][3]f32{
        .{ ground[0], ground[1] + 500.0, ground[2] },
        .{ ground[0], ground[1] - 500.0, ground[2] },
        .{ ground[0] + 1.0e4, ground[1], ground[2] },
        .{ -1.0e4, 50.0, 1.0e4 },
        .{ ground[0], ground[1] + 0.85, ground[2] },
        .{ ground[0], ground[1] + 0.95, ground[2] },
        .{ ground[0] + 1.1, ground[1], ground[2] - 1.1 },
        .{ ground[0] + 0.2, ground[1] + 0.1, ground[2] + 0.2 },
    };
    const params = defaultParams();
    for (places, 0..) |place, i| {
        errdefer std.debug.print("  place {d}\n", .{i});
        const idx = try pair.addAgent(place, &params);
        try testing.expectEqual(@as(i32, @intCast(i)), idx);
        try pair.expectSameState();
    }
    const far = pair.port.getAgent(2) orelse return error.TestUnexpectedResult;
    try expectReached(far.state == .invalid, "an agent with no polygon under it");

    // Agents off the mesh take requests and are stepped like the others.
    var idx: i32 = 0;
    while (idx < 8) : (idx += 1) {
        const target = pair.randomTarget(random);
        _ = try pair.requestMoveTarget(idx, target.ref, target.pos);
    }
    try pair.expectSameState();
    try stepAndCompare(&pair, 20);
    _ = try pair.requestMoveVelocity(2, .{ 1.0, 0.0, -2.0 });
    _ = try pair.resetMoveTarget(3);
    try stepAndCompare(&pair, 5);
}

// ---------------------------------------------------------------------------
// updateAgentParameters, setObstacleAvoidanceParams, filters
// ---------------------------------------------------------------------------

test "updateAgentParameters: every set of update flags steers the same" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, support.nav_test_obj, support.tiled_config, varied_options);
    defer world.deinit();

    var prng: std.Random.DefaultPrng = .init(65);
    const random = prng.random();
    var pair: CrowdPair = undefined;
    try pair.init(gpa, &world, 8, 0.6);
    defer pair.deinit();
    for (&avoidance_presets, 0..) |*preset, i| pair.setObstacleAvoidanceParams(@intCast(i), preset);
    try populate(&pair, random, 6);
    try stepAndCompare(&pair, 10);

    // All thirty-two sets of the five flags; two of them also carry bits
    // the crowd gives no meaning to, which both sides must keep.
    var flags: u8 = 0;
    while (flags < 32) : (flags += 1) {
        errdefer std.debug.print("  update flags {b:0>5}\n", .{flags});
        var idx: i32 = 0;
        while (idx < 6) : (idx += 1) {
            var params = randomParams(random);
            params.update_flags = @bitCast(flags);
            if ((flags == 21 or flags == 31) and idx == 0) {
                params.update_flags = @bitCast(flags | 0xe0);
            }
            params.obstacle_avoidance_type = @intCast(@mod(idx + flags, 4));
            params.query_filter_type = 0;
            params.user_data = if (idx == 1) &user_tag else null;
            pair.updateAgentParameters(idx, &params);
        }
        if (flags % 8 == 0) {
            idx = 0;
            while (idx < 6) : (idx += 1) {
                const target = pair.randomTarget(random);
                _ = try pair.requestMoveTarget(idx, target.ref, target.pos);
            }
        }
        try pair.expectSameState();
        try stepAndCompare(&pair, 4);
    }

    // Indexes outside the crowd change nothing; a free slot takes the
    // parameters and holds them.
    const params = defaultParams();
    for ([_]i32{ -1, 8, 9, 70000, std.math.minInt(i32), 7 }) |idx| {
        pair.updateAgentParameters(idx, &params);
        try pair.expectSameState();
    }
    try stepAndCompare(&pair, 3);
}
