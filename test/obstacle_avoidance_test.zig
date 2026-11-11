//! Differential tests of obstacle avoidance: the same samples go into the
//! port's recorder and into the original's, the same obstacles into both
//! queries, and every getter, every stored obstacle field, each sampling
//! call's count and chosen velocity and every sample it records must then
//! have the same bits.

const std = @import("std");
const detour = @import("detour");
const support = @import("support.zig");
const reference = support.reference_crowd;

const testing = std.testing;

const ObstacleAvoidanceDebug = detour.ObstacleAvoidanceDebug;
const ObstacleAvoidanceParams = detour.ObstacleAvoidanceParams;
const ObstacleAvoidanceQuery = detour.ObstacleAvoidanceQuery;

/// The room the original's crowd gives its query.
const crowd_circle_count_max = 6;
const crowd_segment_count_max = 8;

/// The most divisions and rings the original gives an adaptive pattern
/// (`DT_MAX_PATTERN_DIVS`, `DT_MAX_PATTERN_RINGS`).
const reference_division_count_max = 32;
const reference_ring_count_max = 4;

/// What `nvel` holds before a sampling call, to show the call wrote all
/// three floats.
const untouched_velocity: [3]f32 = .{ 12345.0, -54321.0, 0.03125 };

/// The parameters the original's crowd starts every avoidance type with.
const default_params: ObstacleAvoidanceParams = .{
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

/// One recorded candidate, as `addSample` takes one.
const Sample = struct {
    velocity: [3]f32,
    size: f32,
    penalty: f32,
    desired_velocity_penalty: f32,
    current_velocity_penalty: f32,
    preferred_side_penalty: f32,
    collision_time_penalty: f32,
};

/// The agent a sampling call picks a velocity for.
const Agent = struct {
    position: [3]f32,
    radius: f32,
    velocity_max: f32,
    velocity: [3]f32,
    desired_velocity: [3]f32,
};

/// A moving circle, as `addCircle` takes one.
const Circle = struct {
    position: [3]f32,
    radius: f32,
    velocity: [3]f32,
    desired_velocity: [3]f32,
};

/// A wall segment, as `addSegment` takes one.
const Segment = struct {
    p: [3]f32,
    q: [3]f32,
};

/// Which of the two sampling calls to make.
const Sampler = enum { grid, adaptive };

/// How a sample's total penalty and its preferred side penalty compare.
const SideCompare = enum {
    /// Bit for bit, as every other float.
    exact,
    /// Bit for bit, but any NaN matches any NaN. A circle exactly on the
    /// agent has no direction: both sides store the same NaNs for it, but
    /// the side penalty then adds two NaNs of opposite sign, and which sign
    /// the sum keeps is the compiler's choice of operand order, not the
    /// code's.
    any_nan,
};

/// Fails unless the two floats named `name` compare as `compare` says.
fn expectNamed(
    comptime name: []const u8,
    expected: f32,
    actual: f32,
    compare: SideCompare,
) !void {
    errdefer std.debug.print("  " ++ name ++ "\n", .{});
    if (compare == .any_nan and std.math.isNan(expected) and std.math.isNan(actual)) return;
    try support.expectFloat(expected, actual);
}

/// Fails unless the two points named `name` have the same bits.
fn expectNamedPoint(comptime name: []const u8, expected: [3]f32, actual: [3]f32) !void {
    errdefer std.debug.print("  " ++ name ++ "\n", .{});
    try support.expectPoint(expected, actual);
}

/// `value` as the compiler cannot see through. Arithmetic the compiler does
/// itself gives an invalid operation another NaN than the processor does, so
/// the port's inputs must be as unknown to it as the original's are.
fn runtimeValue(comptime T: type, value: T) T {
    var copy = value;
    std.mem.doNotOptimizeAway(&copy);
    return copy;
}

/// One sample recorder on each side, set up alike and given the same calls.
const DebugPair = struct {
    debug: ObstacleAvoidanceDebug,
    reference_debug: *reference.ObstacleAvoidanceDebugData,

    /// Both recorders with room for `sample_count_max` samples, which is
    /// above zero.
    fn init(gpa: std.mem.Allocator, sample_count_max: i32) !DebugPair {
        var pair = try initEmpty();
        errdefer pair.deinit(gpa);
        try testing.expect(reference.ref_oa_debug_init(pair.reference_debug, sample_count_max));
        try testing.expect(pair.debug.init(gpa, sample_count_max));
        return pair;
    }

    /// Both recorders as they are before `init`: the port's `empty`, the
    /// original's as constructed.
    fn initEmpty() !DebugPair {
        const reference_debug = reference.ref_oa_debug_alloc() orelse return error.OutOfMemory;
        return .{ .debug = .empty, .reference_debug = reference_debug };
    }

    fn deinit(pair: *DebugPair, gpa: std.mem.Allocator) void {
        pair.debug.deinit(gpa);
        reference.ref_oa_debug_free(pair.reference_debug);
    }

    fn reset(pair: *DebugPair) void {
        pair.debug.reset();
        reference.ref_oa_debug_reset(pair.reference_debug);
    }

    /// Records `sample` on both sides.
    fn addSample(pair: *DebugPair, sample: Sample) void {
        // Hidden from the compiler, so `normalizeSamples` works on the
        // port's samples at run time as it does on the original's.
        const hidden = runtimeValue(Sample, sample);
        pair.debug.addSample(
            hidden.velocity,
            hidden.size,
            hidden.penalty,
            hidden.desired_velocity_penalty,
            hidden.current_velocity_penalty,
            hidden.preferred_side_penalty,
            hidden.collision_time_penalty,
        );
        reference.ref_oa_debug_add_sample(
            pair.reference_debug,
            &hidden.velocity,
            hidden.size,
            hidden.penalty,
            hidden.desired_velocity_penalty,
            hidden.current_velocity_penalty,
            hidden.preferred_side_penalty,
            hidden.collision_time_penalty,
        );
    }

    fn normalizeSamples(pair: *DebugPair) void {
        pair.debug.normalizeSamples();
        reference.ref_oa_debug_normalize_samples(pair.reference_debug);
    }

    /// Fails unless both recorders hold the same number of samples and
    /// every getter gives the same bits for each of them. Returns that
    /// number.
    fn expectSame(pair: *const DebugPair, compare: SideCompare) !i32 {
        const expected_count = reference.ref_oa_debug_get_sample_count(pair.reference_debug);
        const actual_count = pair.debug.getSampleCount();
        if (expected_count != actual_count) {
            std.debug.print(
                "getSampleCount: original {d}, port {d}\n",
                .{ expected_count, actual_count },
            );
            return error.TestExpectedEqual;
        }
        var i: i32 = 0;
        while (i < actual_count) : (i += 1) {
            errdefer std.debug.print("  sample {d} of {d}\n", .{ i, actual_count });
            try pair.expectSameSample(i, compare);
        }
        return actual_count;
    }

    /// Fails unless every getter gives the same bits for sample `i`, which
    /// both recorders hold.
    fn expectSameSample(pair: *const DebugPair, i: i32, compare: SideCompare) !void {
        const data = pair.reference_debug;
        const debug = &pair.debug;
        try expectNamedPoint(
            "getSampleVelocity",
            reference.ref_oa_debug_get_sample_velocity(data, i).*,
            debug.getSampleVelocity(i),
        );
        try expectNamed(
            "getSampleSize",
            reference.ref_oa_debug_get_sample_size(data, i),
            debug.getSampleSize(i),
            .exact,
        );
        try expectNamed(
            "getSamplePenalty",
            reference.ref_oa_debug_get_sample_penalty(data, i),
            debug.getSamplePenalty(i),
            compare,
        );
        try expectNamed(
            "getSampleDesiredVelocityPenalty",
            reference.ref_oa_debug_get_sample_desired_velocity_penalty(data, i),
            debug.getSampleDesiredVelocityPenalty(i),
            .exact,
        );
        try expectNamed(
            "getSampleCurrentVelocityPenalty",
            reference.ref_oa_debug_get_sample_current_velocity_penalty(data, i),
            debug.getSampleCurrentVelocityPenalty(i),
            .exact,
        );
        try expectNamed(
            "getSamplePreferredSidePenalty",
            reference.ref_oa_debug_get_sample_preferred_side_penalty(data, i),
            debug.getSamplePreferredSidePenalty(i),
            compare,
        );
        try expectNamed(
            "getSampleCollisionTimePenalty",
            reference.ref_oa_debug_get_sample_collision_time_penalty(data, i),
            debug.getSampleCollisionTimePenalty(i),
            .exact,
        );
    }

    /// Fails unless the port's recorder gives back `sample` as sample `i`,
    /// bit for bit.
    fn expectHolds(pair: *const DebugPair, i: i32, sample: Sample) !void {
        errdefer std.debug.print("  sample {d} read back\n", .{i});
        const debug = &pair.debug;
        try support.expectPoint(sample.velocity, debug.getSampleVelocity(i));
        try support.expectFloat(sample.size, debug.getSampleSize(i));
        try support.expectFloat(sample.penalty, debug.getSamplePenalty(i));
        try support.expectFloat(
            sample.desired_velocity_penalty,
            debug.getSampleDesiredVelocityPenalty(i),
        );
        try support.expectFloat(
            sample.current_velocity_penalty,
            debug.getSampleCurrentVelocityPenalty(i),
        );
        try support.expectFloat(
            sample.preferred_side_penalty,
            debug.getSamplePreferredSidePenalty(i),
        );
        try support.expectFloat(
            sample.collision_time_penalty,
            debug.getSampleCollisionTimePenalty(i),
        );
    }
};

/// What a sampling call gave, the same on both sides.
const Sampling = struct {
    /// The candidates scored.
    count: i32,
    /// The velocity chosen.
    velocity: [3]f32,
};

/// One query on each side, set up alike and given the same calls.
const QueryPair = struct {
    query: ObstacleAvoidanceQuery,
    reference_query: *reference.ObstacleAvoidanceQuery,

    /// Both queries with room for `circle_count_max` circles and
    /// `segment_count_max` segments, neither negative.
    fn init(gpa: std.mem.Allocator, circle_count_max: i32, segment_count_max: i32) !QueryPair {
        var pair = try initEmpty();
        errdefer pair.deinit(gpa);
        try testing.expect(reference.ref_oa_query_init(
            pair.reference_query,
            circle_count_max,
            segment_count_max,
        ));
        try testing.expect(pair.query.init(gpa, circle_count_max, segment_count_max));
        return pair;
    }

    /// Both queries as they are before `init`: the port's `empty`, the
    /// original's as constructed.
    fn initEmpty() !QueryPair {
        const reference_query = reference.ref_oa_query_alloc() orelse return error.OutOfMemory;
        return .{ .query = .empty, .reference_query = reference_query };
    }

    fn deinit(pair: *QueryPair, gpa: std.mem.Allocator) void {
        pair.query.deinit(gpa);
        reference.ref_oa_query_free(pair.reference_query);
    }

    fn reset(pair: *QueryPair) void {
        pair.query.reset();
        reference.ref_oa_query_reset(pair.reference_query);
    }

    fn addCircle(pair: *QueryPair, circle: Circle) void {
        // Hidden from the compiler, as the samples of a recorder are.
        const hidden = runtimeValue(Circle, circle);
        pair.query.addCircle(
            hidden.position,
            hidden.radius,
            hidden.velocity,
            hidden.desired_velocity,
        );
        reference.ref_oa_query_add_circle(
            pair.reference_query,
            &hidden.position,
            hidden.radius,
            &hidden.velocity,
            &hidden.desired_velocity,
        );
    }

    fn addSegment(pair: *QueryPair, segment: Segment) void {
        const hidden = runtimeValue(Segment, segment);
        pair.query.addSegment(hidden.p, hidden.q);
        reference.ref_oa_query_add_segment(pair.reference_query, &hidden.p, &hidden.q);
    }

    /// Fails unless both queries hold the same number of circles and of
    /// segments, and every stored field of each has the same bits: what was
    /// added, and what the latest sampling call worked out.
    fn expectSameObstacles(pair: *const QueryPair) !void {
        const reference_query = pair.reference_query;
        const circle_count = reference.ref_oa_query_get_obstacle_circle_count(reference_query);
        const segment_count = reference.ref_oa_query_get_obstacle_segment_count(reference_query);
        if (circle_count != pair.query.getObstacleCircleCount() or
            segment_count != pair.query.getObstacleSegmentCount())
        {
            std.debug.print(
                "obstacles: original {d} circles and {d} segments, port {d} and {d}\n",
                .{
                    circle_count,
                    segment_count,
                    pair.query.getObstacleCircleCount(),
                    pair.query.getObstacleSegmentCount(),
                },
            );
            return error.TestExpectedEqual;
        }

        var i: i32 = 0;
        while (i < circle_count) : (i += 1) {
            errdefer std.debug.print("  circle {d} of {d}\n", .{ i, circle_count });
            var expected: reference.ObstacleCircle = undefined;
            try testing.expect(
                reference.ref_oa_query_get_obstacle_circle(reference_query, i, &expected),
            );
            const actual = pair.query.getObstacleCircle(i);
            try expectNamedPoint("position", expected.p, actual.position);
            try expectNamedPoint("velocity", expected.vel, actual.velocity);
            try expectNamedPoint("desired_velocity", expected.dvel, actual.desired_velocity);
            try expectNamed("radius", expected.rad, actual.radius, .exact);
            try expectNamedPoint("direction", expected.dp, actual.direction);
            try expectNamedPoint("side_normal", expected.np, actual.side_normal);
        }

        i = 0;
        while (i < segment_count) : (i += 1) {
            errdefer std.debug.print("  segment {d} of {d}\n", .{ i, segment_count });
            var expected: reference.ObstacleSegment = undefined;
            try testing.expect(
                reference.ref_oa_query_get_obstacle_segment(reference_query, i, &expected),
            );
            const actual = pair.query.getObstacleSegment(i);
            try expectNamedPoint("p", expected.p, actual.p);
            try expectNamedPoint("q", expected.q, actual.q);
            try testing.expectEqual(expected.touch, @as(i32, @intFromBool(actual.touch)));
        }
    }

    /// How the samples of the sampling call just made compare: loosely when
    /// a circle's direction came out as NaNs, bit for bit otherwise.
    fn sideCompare(pair: *const QueryPair) SideCompare {
        var circle: reference.ObstacleCircle = undefined;
        var i: i32 = 0;
        while (reference.ref_oa_query_get_obstacle_circle(pair.reference_query, i, &circle)) {
            for (circle.dp) |coordinate| {
                if (std.math.isNan(coordinate)) return .any_nan;
            }
            i += 1;
        }
        return .exact;
    }

    /// The segments the sampling call just made found the agent touching.
    fn touchCount(pair: *const QueryPair) usize {
        var count: usize = 0;
        var segment: reference.ObstacleSegment = undefined;
        var i: i32 = 0;
        while (reference.ref_oa_query_get_obstacle_segment(pair.reference_query, i, &segment)) {
            if (segment.touch != 0) count += 1;
            i += 1;
        }
        return count;
    }

    /// Prints the obstacles the original's query holds, to replay a failure.
    fn printObstacles(pair: *const QueryPair) void {
        var circle: reference.ObstacleCircle = undefined;
        var i: i32 = 0;
        while (reference.ref_oa_query_get_obstacle_circle(pair.reference_query, i, &circle)) {
            std.debug.print("  circle {d}: {any}\n", .{ i, circle });
            i += 1;
        }
        var segment: reference.ObstacleSegment = undefined;
        i = 0;
        while (reference.ref_oa_query_get_obstacle_segment(pair.reference_query, i, &segment)) {
            std.debug.print("  segment {d}: {any}\n", .{ i, segment });
            i += 1;
        }
    }

    /// Makes the `sampler` call for `agent` on both sides. Fails unless both
    /// score the same number of candidates, choose the same velocity, leave
    /// the same obstacles behind and, given `debug`, record the same
    /// samples. Returns the count and the velocity.
    fn expectSameSampling(
        pair: *QueryPair,
        sampler: Sampler,
        agent: Agent,
        params: ObstacleAvoidanceParams,
        debug: ?*DebugPair,
    ) !Sampling {
        errdefer {
            std.debug.print(
                "{s} sampling\n  {any}\n  {any}\n",
                .{ @tagName(sampler), agent, params },
            );
            pair.printObstacles();
        }

        // Hidden from the compiler, so the port samples at run time as the
        // original does.
        const input = runtimeValue(Agent, agent);
        const port_params = runtimeValue(ObstacleAvoidanceParams, params);
        const reference_params = referenceParams(port_params);
        const port_debug: ?*ObstacleAvoidanceDebug = if (debug) |both| &both.debug else null;
        const reference_debug: ?*reference.ObstacleAvoidanceDebugData =
            if (debug) |both| both.reference_debug else null;

        var expected_velocity = untouched_velocity;
        const expected = switch (sampler) {
            .grid => reference.ref_oa_query_sample_velocity_grid(
                pair.reference_query,
                &input.position,
                input.radius,
                input.velocity_max,
                &input.velocity,
                &input.desired_velocity,
                &expected_velocity,
                &reference_params,
                reference_debug,
            ),
            .adaptive => reference.ref_oa_query_sample_velocity_adaptive(
                pair.reference_query,
                &input.position,
                input.radius,
                input.velocity_max,
                &input.velocity,
                &input.desired_velocity,
                &expected_velocity,
                &reference_params,
                reference_debug,
            ),
        };

        var actual_velocity = untouched_velocity;
        const actual = switch (sampler) {
            .grid => pair.query.sampleVelocityGrid(
                input.position,
                input.radius,
                input.velocity_max,
                input.velocity,
                input.desired_velocity,
                &actual_velocity,
                &port_params,
                port_debug,
            ),
            .adaptive => pair.query.sampleVelocityAdaptive(
                input.position,
                input.radius,
                input.velocity_max,
                input.velocity,
                input.desired_velocity,
                &actual_velocity,
                &port_params,
                port_debug,
            ),
        };

        if (expected != actual) {
            std.debug.print("candidates scored: original {d}, port {d}\n", .{ expected, actual });
            return error.TestExpectedEqual;
        }
        try expectNamedPoint("nvel", expected_velocity, actual_velocity);
        try pair.expectSameObstacles();
        if (debug) |both| _ = try both.expectSame(pair.sideCompare());
        return .{ .count = actual, .velocity = actual_velocity };
    }
};

/// `params` as the original takes them.
fn referenceParams(params: ObstacleAvoidanceParams) reference.ObstacleAvoidanceParams {
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

/// The most candidates a `sampler` call with `params` scores: every point
/// of the grid, or the pattern's centre and rings once for each depth. An
/// odd division count is assumed, as every pattern here has one.
fn candidateCountMax(sampler: Sampler, params: ObstacleAvoidanceParams) i32 {
    switch (sampler) {
        .grid => {
            const side: i32 = params.grid_size;
            return side * side;
        },
        .adaptive => {
            const divisions = std.math.clamp(
                @as(i32, params.adaptive_divisions),
                1,
                reference_division_count_max,
            );
            const rings = std.math.clamp(
                @as(i32, params.adaptive_rings),
                1,
                reference_ring_count_max,
            );
            return @as(i32, params.adaptive_depth) * (1 + rings * divisions);
        },
    }
}

/// A recorder with room for every candidate a `sampler` call with `params`
/// can score.
fn ampleDebug(
    gpa: std.mem.Allocator,
    sampler: Sampler,
    params: ObstacleAvoidanceParams,
) !DebugPair {
    return DebugPair.init(gpa, @max(1, candidateCountMax(sampler, params)));
}

/// A float in `[-reach, reach)`.
fn randomSigned(random: std.Random, reach: f32) f32 {
    return (random.float(f32) * 2.0 - 1.0) * reach;
}

/// A vector within `reach` of zero along x and z and a tenth of that along
/// y: avoidance works in the xz plane, and y counts only in the direction
/// from the agent to a circle.
fn randomVector(random: std.Random, reach: f32) [3]f32 {
    return .{
        randomSigned(random, reach),
        randomSigned(random, reach * 0.1),
        randomSigned(random, reach),
    };
}

fn offsetPoint(point: [3]f32, offset: [3]f32) [3]f32 {
    return .{ point[0] + offset[0], point[1] + offset[1], point[2] + offset[2] };
}

fn scaledVector(vector: [3]f32, scale: f32) [3]f32 {
    return .{ vector[0] * scale, vector[1] * scale, vector[2] * scale };
}

/// A random agent, now and then at the origin.
fn randomAgent(random: std.Random) Agent {
    const position: [3]f32 = if (random.uintLessThan(u32, 8) == 0)
        .{ 0, 0, 0 }
    else
        randomVector(random, 20.0);
    return randomAgentAt(random, position);
}

/// A random agent at `position`: now and then standing still, wanting no
/// velocity or the one it has, without a radius, or with a greatest speed of
/// zero or below it.
fn randomAgentAt(random: std.Random, position: [3]f32) Agent {
    const velocity_max: f32 = switch (random.uintLessThan(u32, 20)) {
        0 => 0.0,
        1 => -1.5,
        else => 0.5 + random.float(f32) * 5.0,
    };
    // Up to a fifth over the greatest speed, so that some candidates near
    // the desired velocity are too fast to be scored.
    const reach = @max(@abs(velocity_max), 0.5) * 1.2;
    const velocity: [3]f32 = if (random.uintLessThan(u32, 8) == 0)
        .{ 0, 0, 0 }
    else
        randomVector(random, reach);
    const desired_velocity: [3]f32 = switch (random.uintLessThan(u32, 10)) {
        0 => .{ 0, 0, 0 },
        1 => velocity,
        else => randomVector(random, reach),
    };
    const radius: f32 = if (random.uintLessThan(u32, 16) == 0)
        0.0
    else
        0.1 + random.float(f32) * 0.9;
    return .{
        .position = position,
        .radius = radius,
        .velocity_max = velocity_max,
        .velocity = velocity,
        .desired_velocity = desired_velocity,
    };
}

/// A random circle around `agent`: mostly clear of it, often overlapping
/// it, now and then exactly on it or straight above it, without a radius,
/// standing still, or wanting the velocity the agent wants.
fn randomCircle(random: std.Random, agent: Agent) Circle {
    const radius: f32 = if (random.uintLessThan(u32, 12) == 0)
        0.0
    else
        0.1 + random.float(f32) * 1.2;
    const offset: [3]f32 = switch (random.uintLessThan(u32, 24)) {
        // Exactly on the agent, which leaves the circle without a direction.
        0 => .{ 0, 0, 0 },
        // Straight above or below the agent.
        1 => .{ 0, randomSigned(random, 2.0), 0 },
        // Close enough to overlap the agent.
        2, 3, 4, 5, 6 => randomVector(random, (agent.radius + radius) * 0.7),
        else => randomVector(random, 8.0),
    };
    const velocity: [3]f32 = if (random.uintLessThan(u32, 6) == 0)
        .{ 0, 0, 0 }
    else
        randomVector(random, 3.0);
    const desired_velocity: [3]f32 = switch (random.uintLessThan(u32, 8)) {
        0 => .{ 0, 0, 0 },
        // The agent's own, which leaves no side to prefer.
        1 => agent.desired_velocity,
        2 => velocity,
        else => randomVector(random, 3.0),
    };
    return .{
        .position = offsetPoint(agent.position, offset),
        .radius = radius,
        .velocity = velocity,
        .desired_velocity = desired_velocity,
    };
}

/// The distances from a wall, on either side of it, around the 0.01 below
/// which an agent counts as touching the wall.
const touch_distances = [_]f32{ 0.0099, 0.01, 0.0101, -0.0099, -0.01, -0.0101, 0.005, 0.0 };

/// A random wall segment around `agent`: mostly anywhere near it, now and
/// then through its position or ending on it, without length, or about as
/// far from the agent as touching reaches.
fn randomSegment(random: std.Random, agent: Agent) Segment {
    const at = agent.position;
    const segment: Segment = switch (random.uintLessThan(u32, 16)) {
        // Through the agent's position.
        0 => through: {
            const half = randomVector(random, 4.0);
            break :through .{
                .p = offsetPoint(at, half),
                .q = offsetPoint(at, scaledVector(half, -random.float(f32))),
            };
        },
        // Ending on the agent.
        1 => .{ .p = at, .q = offsetPoint(at, randomVector(random, 4.0)) },
        // Without length, near the agent.
        2 => point: {
            const p = offsetPoint(at, randomVector(random, 3.0));
            break :point .{ .p = p, .q = p };
        },
        // Without length, on the agent.
        3 => .{ .p = at, .q = at },
        // Along x, passing the agent at one of the distances around the
        // reach of touching.
        4, 5 => beside: {
            const z = at[2] + touch_distances[random.uintLessThan(usize, touch_distances.len)];
            const length = 0.5 + random.float(f32) * 3.0;
            break :beside .{
                .p = .{ at[0] - length, at[1], z },
                .q = .{ at[0] + length * random.float(f32), at[1] + 0.25, z },
            };
        },
        else => .{
            .p = offsetPoint(at, randomVector(random, 6.0)),
            .q = offsetPoint(at, randomVector(random, 6.0)),
        },
    };
    // Either way round: the side of a touched wall depends on it.
    return if (random.boolean()) segment else .{ .p = segment.q, .q = segment.p };
}

/// A weight: mostly positive, now and then zero or negative.
fn randomWeight(random: std.Random) f32 {
    return switch (random.uintLessThan(u32, 16)) {
        0, 1 => 0.0,
        2 => -random.float(f32) * 2.0,
        else => random.float(f32) * 4.0,
    };
}

/// A grid size, small enough to keep a sampling call quick.
fn randomGridSize(random: std.Random) u8 {
    return switch (random.uintLessThan(u32, 32)) {
        // No candidates at all.
        0, 1 => 0,
        // One candidate, and a grid step of something over zero.
        2, 3 => 1,
        // The crowd's own.
        4 => 33,
        else => random.intRangeAtMost(u8, 2, 12),
    };
}

/// A division count for the adaptive pattern. Only odd counts, and now and
/// then 0, which the sampler takes as 1: with an even count the original
/// reads a pattern entry it never wrote, where the port reads a zero, so the
/// two cannot be compared. A count above the limit of 32 is taken as 32,
/// which is even too, and with four rings writes past the original's pattern.
fn randomDivisionCount(random: std.Random) u8 {
    if (random.uintLessThan(u32, 24) == 0) return 0;
    return 1 + 2 * random.uintLessThan(u8, 16);
}

/// Random sampling parameters: the crowd's own weights or random ones, with
/// a random grid and a random pattern.
fn randomParams(random: std.Random) ObstacleAvoidanceParams {
    var params = default_params;
    if (random.uintLessThan(u32, 4) != 0) {
        params.vel_bias = switch (random.uintLessThan(u32, 12)) {
            0 => 0.0,
            1 => 1.0,
            2 => 1.5,
            3 => -0.5,
            else => random.float(f32),
        };
        params.weight_desired_velocity = randomWeight(random);
        params.weight_current_velocity = randomWeight(random);
        params.weight_side = randomWeight(random);
        params.weight_time_of_impact = randomWeight(random);
        params.horizon_time = switch (random.uintLessThan(u32, 16)) {
            0 => 0.0,
            else => 0.1 + random.float(f32) * 5.0,
        };
    }
    params.grid_size = randomGridSize(random);
    params.adaptive_divisions = randomDivisionCount(random);
    // The sampler takes any ring count as one of 1 to 4.
    params.adaptive_rings = switch (random.uintLessThan(u32, 12)) {
        0 => 255,
        else => random.uintLessThan(u8, 7),
    };
    params.adaptive_depth = random.uintLessThan(u8, 7);
    return params;
}

/// Empties both queries of `pair` and fills them with random obstacles
/// around `agent`: any number from none to two more than there is room for.
fn fillRandomScene(
    pair: *QueryPair,
    random: std.Random,
    agent: Agent,
    circle_count_max: i32,
    segment_count_max: i32,
) void {
    pair.reset();
    const circle_room: u32 = @intCast(circle_count_max);
    const circle_count = random.uintAtMost(u32, circle_room + 2);
    for (0..circle_count) |_| pair.addCircle(randomCircle(random, agent));
    const segment_room: u32 = @intCast(segment_count_max);
    const segment_count = random.uintAtMost(u32, segment_room + 2);
    for (0..segment_count) |_| pair.addSegment(randomSegment(random, agent));
}

test "ObstacleAvoidanceParams: the original's size and field offsets" {
    const Params = ObstacleAvoidanceParams;
    try testing.expectEqual(reference.ref_sizeof_obstacle_avoidance_params(), @sizeOf(Params));
    try testing.expectEqual(
        reference.ref_offsetof_obstacle_avoidance_params_vel_bias(),
        @offsetOf(Params, "vel_bias"),
    );
    try testing.expectEqual(
        reference.ref_offsetof_obstacle_avoidance_params_weight_des_vel(),
        @offsetOf(Params, "weight_desired_velocity"),
    );
    try testing.expectEqual(
        reference.ref_offsetof_obstacle_avoidance_params_weight_cur_vel(),
        @offsetOf(Params, "weight_current_velocity"),
    );
    try testing.expectEqual(
        reference.ref_offsetof_obstacle_avoidance_params_weight_side(),
        @offsetOf(Params, "weight_side"),
    );
    try testing.expectEqual(
        reference.ref_offsetof_obstacle_avoidance_params_weight_toi(),
        @offsetOf(Params, "weight_time_of_impact"),
    );
    try testing.expectEqual(
        reference.ref_offsetof_obstacle_avoidance_params_horiz_time(),
        @offsetOf(Params, "horizon_time"),
    );
    try testing.expectEqual(
        reference.ref_offsetof_obstacle_avoidance_params_grid_size(),
        @offsetOf(Params, "grid_size"),
    );
    try testing.expectEqual(
        reference.ref_offsetof_obstacle_avoidance_params_adaptive_divs(),
        @offsetOf(Params, "adaptive_divisions"),
    );
    try testing.expectEqual(
        reference.ref_offsetof_obstacle_avoidance_params_adaptive_rings(),
        @offsetOf(Params, "adaptive_rings"),
    );
    try testing.expectEqual(
        reference.ref_offsetof_obstacle_avoidance_params_adaptive_depth(),
        @offsetOf(Params, "adaptive_depth"),
    );
}

test "the shim's obstacle copies: the sizes the shim writes" {
    // Every stored obstacle is read back into one of these. Were one smaller
    // than the shim's, the shim would write past it; were one larger, its
    // tail would compare as whatever the test left there.
    try testing.expectEqual(
        reference.ref_sizeof_obstacle_circle(),
        @sizeOf(reference.ObstacleCircle),
    );
    try testing.expectEqual(
        reference.ref_sizeof_obstacle_segment(),
        @sizeOf(reference.ObstacleSegment),
    );
}

test "pattern_division_count_max and pattern_ring_count_max: the original's limits" {
    // The original's limits have no accessor; these are its header's values.
    const limits = detour.obstacle_avoidance;
    try testing.expectEqual(reference_division_count_max, limits.pattern_division_count_max);
    try testing.expectEqual(reference_ring_count_max, limits.pattern_ring_count_max);
}

/// A sample no float of which is shared with another `number`'s or with
/// another field's.
fn numberedSample(number: usize) Sample {
    const n: f32 = @floatFromInt(number);
    return .{
        .velocity = .{ n + 0.125, -n - 0.25, n * 2.0 + 0.375 },
        .size = n * 0.5 + 0.0625,
        .penalty = n * 3.0 + 1.0,
        .desired_velocity_penalty = 700.0 - n,
        .current_velocity_penalty = n * n * 0.25 + 0.75,
        .preferred_side_penalty = -n * 0.5 - 20.0,
        .collision_time_penalty = 100.0 - n * 1.5,
    };
}

test "ObstacleAvoidanceDebug.init: a new recorder holds no samples" {
    const gpa = testing.allocator;
    for ([_]i32{ 1, 2, 7, 64, 4096 }) |sample_count_max| {
        var pair = try DebugPair.init(gpa, sample_count_max);
        defer pair.deinit(gpa);
        try testing.expectEqual(0, try pair.expectSame(.exact));

        // Nothing to drop or to rescale yet.
        pair.reset();
        pair.normalizeSamples();
        try testing.expectEqual(0, try pair.expectSame(.exact));
    }
}

test "ObstacleAvoidanceDebug.empty: takes no samples, as the original before init" {
    const gpa = testing.allocator;
    var pair = try DebugPair.initEmpty();
    defer pair.deinit(gpa);
    try testing.expectEqual(0, try pair.expectSame(.exact));

    pair.addSample(numberedSample(1));
    try testing.expectEqual(0, try pair.expectSame(.exact));
    pair.normalizeSamples();
    pair.reset();
    pair.addSample(numberedSample(2));
    try testing.expectEqual(0, try pair.expectSame(.exact));
}

test "ObstacleAvoidanceDebug.init: a negative capacity fails and leaves no room" {
    const gpa = testing.allocator;
    for ([_]i32{ -1, -2, -1000, std.math.minInt(i32) }) |sample_count_max| {
        errdefer std.debug.print("  capacity {d}\n", .{sample_count_max});
        var pair = try DebugPair.initEmpty();
        defer pair.deinit(gpa);

        // The original asks for more memory than there is and gives up.
        const expected = reference.ref_oa_debug_init(pair.reference_debug, sample_count_max);
        const actual = pair.debug.init(gpa, sample_count_max);
        try testing.expectEqual(expected, actual);
        try testing.expect(!actual);

        pair.addSample(numberedSample(1));
        try testing.expectEqual(0, try pair.expectSame(.exact));
    }
}

test "ObstacleAvoidanceDebug.addSample: one sample fills a recorder of one" {
    const gpa = testing.allocator;
    var pair = try DebugPair.init(gpa, 1);
    defer pair.deinit(gpa);

    pair.addSample(numberedSample(1));
    try testing.expectEqual(1, try pair.expectSame(.exact));
    try pair.expectHolds(0, numberedSample(1));

    // A second one is dropped, and the first stays as it was.
    pair.addSample(numberedSample(2));
    try testing.expectEqual(1, try pair.expectSame(.exact));
    try pair.expectHolds(0, numberedSample(1));
}

test "ObstacleAvoidanceDebug.addSample: samples beyond the capacity are dropped" {
    const gpa = testing.allocator;
    for ([_]i32{ 1, 2, 3, 8, 33 }) |sample_count_max| {
        errdefer std.debug.print("  capacity {d}\n", .{sample_count_max});
        var pair = try DebugPair.init(gpa, sample_count_max);
        defer pair.deinit(gpa);

        const capacity: usize = @intCast(sample_count_max);
        for (0..capacity + 3) |number| {
            pair.addSample(numberedSample(number));
            const kept: i32 = @intCast(@min(number + 1, capacity));
            try testing.expectEqual(kept, try pair.expectSame(.exact));
        }
        // The samples kept are the first ones, in the order they came.
        for (0..capacity) |number| {
            try pair.expectHolds(@intCast(number), numberedSample(number));
        }
    }
}

test "ObstacleAvoidanceDebug.reset: an emptied recorder fills again from the start" {
    const gpa = testing.allocator;
    var pair = try DebugPair.init(gpa, 5);
    defer pair.deinit(gpa);

    // Emptied when part full, when full, and twice in a row.
    for ([_]usize{ 3, 5, 9, 0, 1 }, 0..) |sample_count, round| {
        errdefer std.debug.print("  round {d}\n", .{round});
        for (0..sample_count) |number| pair.addSample(numberedSample(round * 16 + number));
        const kept: i32 = @intCast(@min(sample_count, 5));
        try testing.expectEqual(kept, try pair.expectSame(.exact));
        if (kept > 0) try pair.expectHolds(0, numberedSample(round * 16));

        pair.reset();
        if (round == 3) pair.reset();
        try testing.expectEqual(0, try pair.expectSame(.exact));
    }
}

/// A sample with the five `penalties`, in the order `addSample` takes them,
/// and a velocity and a size that tell which sample it is.
fn penaltySample(number: usize, penalties: [5]f32) Sample {
    const n: f32 = @floatFromInt(number);
    return .{
        .velocity = .{ n + 0.5, -n - 1.0, n * 4.0 + 2.0 },
        .size = n + 0.25,
        .penalty = penalties[0],
        .desired_velocity_penalty = penalties[1],
        .current_velocity_penalty = penalties[2],
        .preferred_side_penalty = penalties[3],
        .collision_time_penalty = penalties[4],
    };
}

test "ObstacleAvoidanceDebug.normalizeSamples: the penalties of one sample all become 0" {
    const gpa = testing.allocator;
    for ([_][5]f32{
        .{ 7.5, -2.0, 0.0, 1000.0, 0.25 },
        .{ 1.0, 1.0, 1.0, 1.0, 1.0 },
        .{ -0.001, 0.001, 3.0e38, -3.0e38, 1.0e-40 },
    }) |penalties| {
        errdefer std.debug.print("  penalties {any}\n", .{penalties});
        var pair = try DebugPair.init(gpa, 4);
        defer pair.deinit(gpa);

        pair.addSample(penaltySample(3, penalties));
        pair.normalizeSamples();
        try testing.expectEqual(1, try pair.expectSame(.exact));
        // The velocity and the size are no penalties and stay as they were.
        try pair.expectHolds(0, penaltySample(3, @splat(0.0)));
    }
}

test "ObstacleAvoidanceDebug.normalizeSamples: equal penalties all become 0" {
    const gpa = testing.allocator;
    var pair = try DebugPair.init(gpa, 8);
    defer pair.deinit(gpa);

    const penalties: [5]f32 = .{ 4.0, -4.0, 0.0, 0.125, 1.0e10 };
    for (0..6) |number| pair.addSample(penaltySample(number, penalties));
    pair.normalizeSamples();
    try testing.expectEqual(6, try pair.expectSame(.exact));
    for (0..6) |number| {
        try pair.expectHolds(@intCast(number), penaltySample(number, @splat(0.0)));
    }
}

test "ObstacleAvoidanceDebug.normalizeSamples: a range up to 0.001 is shifted, not stretched" {
    const gpa = testing.allocator;
    var pair = try DebugPair.init(gpa, 2);
    defer pair.deinit(gpa);

    // Each of the five penalties has a range of its own: half the limit,
    // twice the limit, the limit itself, a hair over it, and none.
    pair.addSample(penaltySample(0, .{ 1.0, 5.0, 0.0, 0.0, 3.0 }));
    pair.addSample(penaltySample(1, .{ 1.0005, 5.002, 0.001, 0.0010001, 3.0 }));
    pair.normalizeSamples();
    try testing.expectEqual(2, try pair.expectSame(.exact));

    // The lower sample of each penalty lands on 0.
    try pair.expectHolds(0, penaltySample(0, @splat(0.0)));
    // A range of the limit itself is kept as it is, and one a hair over is
    // stretched to 1.
    try support.expectFloat(0.001, pair.debug.getSampleCurrentVelocityPenalty(1));
    try testing.expect(pair.debug.getSamplePenalty(1) < 0.001);
    try testing.expect(pair.debug.getSampleDesiredVelocityPenalty(1) > 0.99);
    try testing.expect(pair.debug.getSamplePreferredSidePenalty(1) > 0.99);
    try support.expectFloat(0.0, pair.debug.getSampleCollisionTimePenalty(1));
}

test "ObstacleAvoidanceDebug.normalizeSamples: twice, and again after more samples" {
    const gpa = testing.allocator;
    var pair = try DebugPair.init(gpa, 12);
    defer pair.deinit(gpa);

    for (0..5) |number| pair.addSample(numberedSample(number));
    pair.normalizeSamples();
    try testing.expectEqual(5, try pair.expectSame(.exact));
    pair.normalizeSamples();
    try testing.expectEqual(5, try pair.expectSame(.exact));

    // Samples not yet rescaled, among those that were.
    for (5..9) |number| pair.addSample(numberedSample(number));
    try testing.expectEqual(9, try pair.expectSame(.exact));
    pair.normalizeSamples();
    try testing.expectEqual(9, try pair.expectSame(.exact));

    // More than there is room for: only what was kept is rescaled.
    for (9..20) |number| pair.addSample(numberedSample(number));
    pair.normalizeSamples();
    try testing.expectEqual(12, try pair.expectSame(.exact));

    // A reset drops the rescaled samples as any others.
    pair.reset();
    pair.normalizeSamples();
    try testing.expectEqual(0, try pair.expectSame(.exact));
    pair.addSample(numberedSample(40));
    pair.addSample(numberedSample(41));
    pair.normalizeSamples();
    try testing.expectEqual(2, try pair.expectSame(.exact));
}

/// Penalties the rescaling turns on: zeros of both signs, values 0.001 apart
/// and a little under and over it, the largest floats, the infinities and a
/// NaN.
const awkward_penalties = [_]f32{
    0.0,
    @bitCast(@as(u32, 0x80000000)),
    1.0,
    1.0005,
    1.001,
    1.0011,
    -1.0,
    2.5,
    1.0e-30,
    std.math.floatMax(f32),
    -std.math.floatMax(f32),
    std.math.inf(f32),
    -std.math.inf(f32),
    // The one NaN the processor gives an invalid operation, so that every
    // NaN the rescaling meets has the same bits: which of two different
    // NaNs an operation keeps is the compiler's choice of operand order.
    @bitCast(@as(u32, 0xffc00000)),
};

test "ObstacleAvoidanceDebug.normalizeSamples: every ordered triple of awkward penalties" {
    const gpa = testing.allocator;
    var pair = try DebugPair.init(gpa, 3);
    defer pair.deinit(gpa);

    for (awkward_penalties) |a| {
        for (awkward_penalties) |b| {
            for (awkward_penalties) |c| {
                errdefer std.debug.print("  penalties {e}, {e}, {e}\n", .{ a, b, c });
                const triple: [3]f32 = .{ a, b, c };
                pair.reset();
                for (0..3) |number| {
                    // Each of the five penalties starts the triple at
                    // another place.
                    var penalties: [5]f32 = undefined;
                    for (&penalties, 0..) |*penalty, field| {
                        penalty.* = triple[(number + field) % 3];
                    }
                    pair.addSample(penaltySample(number, penalties));
                }
                pair.normalizeSamples();
                try testing.expectEqual(3, try pair.expectSame(.exact));
            }
        }
    }
}

/// A float of any bits: every NaN, the infinities and the denormals among
/// them.
fn randomBits(random: std.Random) f32 {
    return @bitCast(random.int(u32));
}

/// A sample every float of which has any bits.
fn randomBitsSample(random: std.Random) Sample {
    return .{
        .velocity = .{ randomBits(random), randomBits(random), randomBits(random) },
        .size = randomBits(random),
        .penalty = randomBits(random),
        .desired_velocity_penalty = randomBits(random),
        .current_velocity_penalty = randomBits(random),
        .preferred_side_penalty = randomBits(random),
        .collision_time_penalty = randomBits(random),
    };
}

test "ObstacleAvoidanceDebug: random samples of any bits, added, dropped and read back" {
    const gpa = testing.allocator;
    const seed: u64 = 0x73616d706c6573;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    const sample_room = 40;
    var full_count: usize = 0;
    for (0..2000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const sample_count_max = random.intRangeAtMost(i32, 1, sample_room);
        const capacity: usize = @intCast(sample_count_max);
        var pair = try DebugPair.init(gpa, sample_count_max);
        defer pair.deinit(gpa);

        // What the recorder holds, kept apart from both recorders.
        var kept: [sample_room]Sample = undefined;
        var kept_count: usize = 0;

        const step_count = random.uintAtMost(u32, 60);
        for (0..step_count) |_| {
            if (random.uintLessThan(u32, 16) == 0) {
                pair.reset();
                kept_count = 0;
            } else {
                const sample = randomBitsSample(random);
                pair.addSample(sample);
                if (kept_count < capacity) {
                    kept[kept_count] = sample;
                    kept_count += 1;
                } else {
                    full_count += 1;
                }
            }
            if (random.uintLessThan(u32, 8) == 0) {
                const count: i32 = @intCast(kept_count);
                try testing.expectEqual(count, try pair.expectSame(.exact));
            }
        }

        const count: i32 = @intCast(kept_count);
        try testing.expectEqual(count, try pair.expectSame(.exact));
        for (kept[0..kept_count], 0..) |sample, i| try pair.expectHolds(@intCast(i), sample);
    }
    // Samples were dropped for want of room, too.
    try testing.expect(full_count > 0);
}

/// How the penalties of one rescaling case are spread.
const PenaltySpread = enum {
    /// All within 0.002 of one value, around the range that is shifted and
    /// not stretched.
    close,
    /// Anywhere within 100 of zero.
    wide,
    /// Small values, a third of them from `awkward_penalties`.
    awkward,
};

/// A penalty spread as `spread` says, the close ones from `base` upward.
fn randomPenalty(random: std.Random, spread: PenaltySpread, base: f32) f32 {
    switch (spread) {
        .close => return base + random.float(f32) * 0.002,
        .wide => return randomSigned(random, 100.0),
        .awkward => {
            if (random.uintLessThan(u32, 3) != 0) return randomSigned(random, 4.0);
            return awkward_penalties[random.uintLessThan(usize, awkward_penalties.len)];
        },
    }
}

test "ObstacleAvoidanceDebug.normalizeSamples: random penalties, close, wide and awkward" {
    const gpa = testing.allocator;
    const seed: u64 = 0x6e6f726d616c697a;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var rescaled_count: usize = 0;
    for (0..3000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const sample_count_max = random.intRangeAtMost(i32, 1, 24);
        var pair = try DebugPair.init(gpa, sample_count_max);
        defer pair.deinit(gpa);

        const spread = random.enumValue(PenaltySpread);
        var bases: [5]f32 = undefined;
        for (&bases) |*base| base.* = randomSigned(random, 2.0);

        // Now and then more samples go in after a rescaling, and all are
        // rescaled again.
        const round_count = random.intRangeAtMost(u32, 1, 3);
        for (0..round_count) |_| {
            const add_count = random.uintAtMost(u32, 30);
            for (0..add_count) |_| {
                // The velocity and the size take any bits: the rescaling
                // must leave them alone.
                var sample = randomBitsSample(random);
                sample.penalty = randomPenalty(random, spread, bases[0]);
                sample.desired_velocity_penalty = randomPenalty(random, spread, bases[1]);
                sample.current_velocity_penalty = randomPenalty(random, spread, bases[2]);
                sample.preferred_side_penalty = randomPenalty(random, spread, bases[3]);
                sample.collision_time_penalty = randomPenalty(random, spread, bases[4]);
                pair.addSample(sample);
            }
            _ = try pair.expectSame(.exact);
            pair.normalizeSamples();
            const count = try pair.expectSame(.exact);
            if (count > 1) rescaled_count += 1;
            if (random.uintLessThan(u32, 6) == 0) pair.reset();
        }
    }
    try testing.expect(rescaled_count > 1000);
}

/// An agent at the origin, moving along x and wanting to veer a little
/// toward z, at well under its greatest speed.
const plain_agent: Agent = .{
    .position = .{ 0, 0, 0 },
    .radius = 0.5,
    .velocity_max = 2.0,
    .velocity = .{ 1.0, 0, 0 },
    .desired_velocity = .{ 1.5, 0, 0.5 },
};

/// A circle no float of which is shared with another `number`'s or with
/// another field's.
fn numberedCircle(number: usize) Circle {
    const n: f32 = @floatFromInt(number);
    return .{
        .position = .{ n + 0.125, n * 0.5 + 0.25, -n - 0.375 },
        .radius = n * 0.25 + 0.0625,
        .velocity = .{ -n * 2.0 - 0.5, n + 0.75, n * 3.0 + 0.875 },
        .desired_velocity = .{ n * 5.0 + 0.5625, -n * 7.0 - 0.6875, n * 11.0 + 0.8125 },
    };
}

/// A segment no float of which is shared with another `number`'s or with
/// the other end's.
fn numberedSegment(number: usize) Segment {
    const n: f32 = @floatFromInt(number);
    return .{
        .p = .{ n * 13.0 + 0.03125, -n - 0.09375, n * 17.0 + 0.15625 },
        .q = .{ -n * 19.0 - 0.21875, n * 23.0 + 0.28125, n + 0.34375 },
    };
}

/// Fails unless both queries of `pair` hold `circle_count` circles and
/// `segment_count` segments, with the same bits in every stored field.
fn expectObstacleCounts(pair: *const QueryPair, circle_count: i32, segment_count: i32) !void {
    try pair.expectSameObstacles();
    try testing.expectEqual(circle_count, pair.query.getObstacleCircleCount());
    try testing.expectEqual(segment_count, pair.query.getObstacleSegmentCount());
}

/// Fails unless the port's query gives back `circle` as circle `i`, bit for
/// bit.
fn expectHoldsCircle(pair: *const QueryPair, i: i32, circle: Circle) !void {
    errdefer std.debug.print("  circle {d} read back\n", .{i});
    const stored = pair.query.getObstacleCircle(i);
    try support.expectPoint(circle.position, stored.position);
    try support.expectFloat(circle.radius, stored.radius);
    try support.expectPoint(circle.velocity, stored.velocity);
    try support.expectPoint(circle.desired_velocity, stored.desired_velocity);
}

/// Fails unless the port's query gives back `segment` as segment `i`, bit
/// for bit.
fn expectHoldsSegment(pair: *const QueryPair, i: i32, segment: Segment) !void {
    errdefer std.debug.print("  segment {d} read back\n", .{i});
    const stored = pair.query.getObstacleSegment(i);
    try support.expectPoint(segment.p, stored.p);
    try support.expectPoint(segment.q, stored.q);
}

test "ObstacleAvoidanceQuery.init: a new query holds no obstacles" {
    const gpa = testing.allocator;
    const rooms = [_][2]i32{ .{ 6, 8 }, .{ 1, 1 }, .{ 0, 0 }, .{ 0, 3 }, .{ 3, 0 }, .{ 64, 64 } };
    for (rooms) |room| {
        errdefer std.debug.print("  room for {d} circles, {d} segments\n", .{ room[0], room[1] });
        var pair = try QueryPair.init(gpa, room[0], room[1]);
        defer pair.deinit(gpa);
        try expectObstacleCounts(&pair, 0, 0);

        // Nothing to remove yet.
        pair.reset();
        try expectObstacleCounts(&pair, 0, 0);
    }
}

test "ObstacleAvoidanceQuery.empty: takes no obstacles, as the original before init" {
    const gpa = testing.allocator;
    var pair = try QueryPair.initEmpty();
    defer pair.deinit(gpa);
    try expectObstacleCounts(&pair, 0, 0);

    pair.addCircle(numberedCircle(1));
    pair.addSegment(numberedSegment(1));
    try expectObstacleCounts(&pair, 0, 0);
    pair.reset();
    pair.addCircle(numberedCircle(2));
    pair.addSegment(numberedSegment(2));
    try expectObstacleCounts(&pair, 0, 0);

    // Sampling among no obstacles needs none of the memory `init` makes.
    for ([_]Sampler{ .grid, .adaptive }) |sampler| {
        const sampling = try pair.expectSameSampling(sampler, plain_agent, default_params, null);
        try testing.expect(sampling.count > 0);
        try expectObstacleCounts(&pair, 0, 0);
    }
}

test "ObstacleAvoidanceQuery.init: a negative capacity fails" {
    const gpa = testing.allocator;
    const rooms = [_][2]i32{
        .{ -1, 8 },
        .{ 6, -1 },
        .{ -1, -1 },
        .{ std.math.minInt(i32), 4 },
        .{ 4, -1000 },
    };
    for (rooms) |room| {
        errdefer std.debug.print("  room for {d} circles, {d} segments\n", .{ room[0], room[1] });
        var pair = try QueryPair.initEmpty();
        defer pair.deinit(gpa);

        // The original asks for more memory than there is and gives up.
        const expected = reference.ref_oa_query_init(pair.reference_query, room[0], room[1]);
        const actual = pair.query.init(gpa, room[0], room[1]);
        try testing.expectEqual(expected, actual);
        try testing.expect(!actual);
        try expectObstacleCounts(&pair, 0, 0);

        // Neither side has room for a segment after that.
        pair.addSegment(numberedSegment(1));
        try expectObstacleCounts(&pair, 0, 0);
        // Circles are tried only where their own room failed: where the
        // segments' room failed, the original still has the circles' room it
        // made first, and the port has none.
        if (room[0] < 0) {
            pair.addCircle(numberedCircle(1));
            try expectObstacleCounts(&pair, 0, 0);
        }
    }
}

test "ObstacleAvoidanceQuery.addCircle, addSegment: one of each, every field read back" {
    const gpa = testing.allocator;
    var pair = try QueryPair.init(gpa, crowd_circle_count_max, crowd_segment_count_max);
    defer pair.deinit(gpa);

    pair.addCircle(numberedCircle(1));
    try expectObstacleCounts(&pair, 1, 0);
    try expectHoldsCircle(&pair, 0, numberedCircle(1));
    // The direction and the side are the zeros `init` left, until a sampling
    // call works them out.
    try support.expectPoint(.{ 0, 0, 0 }, pair.query.getObstacleCircle(0).direction);
    try support.expectPoint(.{ 0, 0, 0 }, pair.query.getObstacleCircle(0).side_normal);

    pair.addSegment(numberedSegment(1));
    try expectObstacleCounts(&pair, 1, 1);
    try expectHoldsSegment(&pair, 0, numberedSegment(1));
    try testing.expect(!pair.query.getObstacleSegment(0).touch);
    // The segment took nothing from the circle.
    try expectHoldsCircle(&pair, 0, numberedCircle(1));

    pair.addCircle(numberedCircle(2));
    pair.addSegment(numberedSegment(2));
    try expectObstacleCounts(&pair, 2, 2);
    try expectHoldsCircle(&pair, 0, numberedCircle(1));
    try expectHoldsCircle(&pair, 1, numberedCircle(2));
    try expectHoldsSegment(&pair, 0, numberedSegment(1));
    try expectHoldsSegment(&pair, 1, numberedSegment(2));
}

test "ObstacleAvoidanceQuery.addCircle, addSegment: obstacles beyond the room are dropped" {
    const gpa = testing.allocator;
    const rooms = [_][2]i32{ .{ 6, 8 }, .{ 1, 1 }, .{ 0, 2 }, .{ 2, 0 }, .{ 0, 0 }, .{ 13, 5 } };
    for (rooms) |room| {
        errdefer std.debug.print("  room for {d} circles, {d} segments\n", .{ room[0], room[1] });
        var pair = try QueryPair.init(gpa, room[0], room[1]);
        defer pair.deinit(gpa);
        const circle_room: usize = @intCast(room[0]);
        const segment_room: usize = @intCast(room[1]);

        // The two kinds fill apart: one being full leaves the other its room.
        for (0..@max(circle_room, segment_room) + 3) |number| {
            pair.addCircle(numberedCircle(number));
            pair.addSegment(numberedSegment(number));
            const circle_count: i32 = @intCast(@min(number + 1, circle_room));
            const segment_count: i32 = @intCast(@min(number + 1, segment_room));
            try expectObstacleCounts(&pair, circle_count, segment_count);
        }
        // What is kept is the first of each kind, in the order they came.
        for (0..circle_room) |number| {
            try expectHoldsCircle(&pair, @intCast(number), numberedCircle(number));
        }
        for (0..segment_room) |number| {
            try expectHoldsSegment(&pair, @intCast(number), numberedSegment(number));
        }
    }
}

test "ObstacleAvoidanceQuery.reset: an emptied query fills again from the start" {
    const gpa = testing.allocator;
    var pair = try QueryPair.init(gpa, 3, 2);
    defer pair.deinit(gpa);

    // Emptied when part full, when over full, and twice in a row.
    for ([_]usize{ 1, 5, 0, 2 }, 0..) |obstacle_count, round| {
        errdefer std.debug.print("  round {d}\n", .{round});
        for (0..obstacle_count) |number| {
            pair.addCircle(numberedCircle(round * 16 + number));
            pair.addSegment(numberedSegment(round * 16 + number));
        }
        const circle_count: i32 = @intCast(@min(obstacle_count, 3));
        const segment_count: i32 = @intCast(@min(obstacle_count, 2));
        try expectObstacleCounts(&pair, circle_count, segment_count);
        if (obstacle_count > 0) {
            try expectHoldsCircle(&pair, 0, numberedCircle(round * 16));
            try expectHoldsSegment(&pair, 0, numberedSegment(round * 16));
        }

        pair.reset();
        if (round == 1) pair.reset();
        try expectObstacleCounts(&pair, 0, 0);
    }
}

test "ObstacleAvoidanceQuery.reset: a refilled slot keeps what the last sampling worked out" {
    const gpa = testing.allocator;
    var pair = try QueryPair.init(gpa, 2, 2);
    defer pair.deinit(gpa);

    // A circle ahead of the agent, and a wall the agent stands on.
    pair.addCircle(.{
        .position = .{ 3, 0, 1 },
        .radius = 0.5,
        .velocity = .{ -1, 0, 0 },
        .desired_velocity = .{ -1, 0, 0 },
    });
    pair.addSegment(.{ .p = .{ -1, 0, 0.005 }, .q = .{ 1, 0, 0.005 } });
    for ([_]Sampler{ .grid, .adaptive }) |sampler| {
        _ = try pair.expectSameSampling(sampler, plain_agent, default_params, null);
        try testing.expectEqual(1, pair.touchCount());
        try testing.expect(pair.query.getObstacleSegment(0).touch);
        try testing.expect(pair.query.getObstacleCircle(0).direction[0] > 0.9);

        // Adding writes only what it is given: the slots' new obstacles show
        // the old direction, side and touch until the next sampling call.
        pair.reset();
        pair.addCircle(numberedCircle(7));
        pair.addSegment(numberedSegment(7));
        try expectObstacleCounts(&pair, 1, 1);
        try expectHoldsCircle(&pair, 0, numberedCircle(7));
        try expectHoldsSegment(&pair, 0, numberedSegment(7));
        try testing.expect(pair.query.getObstacleSegment(0).touch);
        try testing.expect(pair.query.getObstacleCircle(0).direction[0] > 0.9);

        // The next call works all three out afresh.
        _ = try pair.expectSameSampling(sampler, plain_agent, default_params, null);
        try testing.expectEqual(0, pair.touchCount());
        try testing.expect(!pair.query.getObstacleSegment(0).touch);

        pair.reset();
        pair.addCircle(.{
            .position = .{ 3, 0, 1 },
            .radius = 0.5,
            .velocity = .{ -1, 0, 0 },
            .desired_velocity = .{ -1, 0, 0 },
        });
        pair.addSegment(.{ .p = .{ -1, 0, 0.005 }, .q = .{ 1, 0, 0.005 } });
    }
}

/// A point every float of which has any bits.
fn randomBitsPoint(random: std.Random) [3]f32 {
    return .{ randomBits(random), randomBits(random), randomBits(random) };
}

test "ObstacleAvoidanceQuery: random obstacles of any bits, added, dropped and read back" {
    const gpa = testing.allocator;
    const seed: u64 = 0x6f62737461636c65;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    const room_max = 12;
    var dropped_count: usize = 0;
    for (0..2000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const circle_count_max = random.intRangeAtMost(i32, 0, room_max);
        const segment_count_max = random.intRangeAtMost(i32, 0, room_max);
        var pair = try QueryPair.init(gpa, circle_count_max, segment_count_max);
        defer pair.deinit(gpa);
        const circle_room: usize = @intCast(circle_count_max);
        const segment_room: usize = @intCast(segment_count_max);

        // What the query holds, kept apart from both queries.
        var circles: [room_max]Circle = undefined;
        var circle_count: usize = 0;
        var segments: [room_max]Segment = undefined;
        var segment_count: usize = 0;

        const step_count = random.uintAtMost(u32, 40);
        for (0..step_count) |_| {
            switch (random.uintLessThan(u32, 16)) {
                0 => {
                    pair.reset();
                    circle_count = 0;
                    segment_count = 0;
                },
                1, 2, 3, 4, 5, 6, 7, 8 => {
                    const circle: Circle = .{
                        .position = randomBitsPoint(random),
                        .radius = randomBits(random),
                        .velocity = randomBitsPoint(random),
                        .desired_velocity = randomBitsPoint(random),
                    };
                    pair.addCircle(circle);
                    if (circle_count < circle_room) {
                        circles[circle_count] = circle;
                        circle_count += 1;
                    } else {
                        dropped_count += 1;
                    }
                },
                else => {
                    const segment: Segment = .{
                        .p = randomBitsPoint(random),
                        .q = randomBitsPoint(random),
                    };
                    pair.addSegment(segment);
                    if (segment_count < segment_room) {
                        segments[segment_count] = segment;
                        segment_count += 1;
                    } else {
                        dropped_count += 1;
                    }
                },
            }
            if (random.uintLessThan(u32, 8) == 0) {
                try expectObstacleCounts(&pair, @intCast(circle_count), @intCast(segment_count));
            }
        }

        try expectObstacleCounts(&pair, @intCast(circle_count), @intCast(segment_count));
        for (circles[0..circle_count], 0..) |circle, i| {
            try expectHoldsCircle(&pair, @intCast(i), circle);
        }
        for (segments[0..segment_count], 0..) |segment, i| {
            try expectHoldsSegment(&pair, @intCast(i), segment);
        }
    }
    // Obstacles were dropped for want of room, too.
    try testing.expect(dropped_count > 0);
}

/// Makes the `sampler` call for `agent` on both sides of `pair` twice: with
/// a recorder that has room for every candidate, then with none. Fails
/// unless the two sides agree on everything both times, the recorded
/// samples rescale alike, and the recorder leaves the count and the chosen
/// velocity as they are without it. Returns the two.
fn expectSameRecorded(
    pair: *QueryPair,
    sampler: Sampler,
    agent: Agent,
    params: ObstacleAvoidanceParams,
) !Sampling {
    const gpa = testing.allocator;
    var debug = try ampleDebug(gpa, sampler, params);
    defer debug.deinit(gpa);

    const recorded = try pair.expectSameSampling(sampler, agent, params, &debug);
    try testing.expect(recorded.count >= 0);
    try testing.expect(recorded.count <= candidateCountMax(sampler, params));
    // Only a candidate scored to the end is recorded.
    try testing.expect(debug.debug.getSampleCount() <= recorded.count);
    debug.normalizeSamples();
    _ = try debug.expectSame(pair.sideCompare());

    const plain = try pair.expectSameSampling(sampler, agent, params, null);
    try testing.expectEqual(recorded.count, plain.count);
    try support.expectPoint(recorded.velocity, plain.velocity);
    return recorded;
}

/// What the two samplers gave for one scene.
const SceneSampling = struct {
    grid: Sampling,
    adaptive: Sampling,
};

/// Both samplers, each as `expectSameRecorded` calls it, for `agent` among
/// `circles` and `segments` in queries with the crowd's room.
fn expectSameScene(
    agent: Agent,
    circles: []const Circle,
    segments: []const Segment,
    params: ObstacleAvoidanceParams,
) !SceneSampling {
    const gpa = testing.allocator;
    var pair = try QueryPair.init(gpa, crowd_circle_count_max, crowd_segment_count_max);
    defer pair.deinit(gpa);
    for (circles) |circle| pair.addCircle(circle);
    for (segments) |segment| pair.addSegment(segment);

    const grid = try expectSameRecorded(&pair, .grid, agent, params);
    const adaptive = try expectSameRecorded(&pair, .adaptive, agent, params);
    return .{ .grid = grid, .adaptive = adaptive };
}

test "sampleVelocityGrid, sampleVelocityAdaptive: no obstacles" {
    const sampling = try expectSameScene(plain_agent, &.{}, &.{}, default_params);
    try testing.expect(sampling.grid.count > 0);
    try testing.expect(sampling.adaptive.count > 0);
    // With nothing in the way the choice lies near the desired velocity,
    // which heads along x.
    try testing.expect(sampling.grid.velocity[0] > 0.5);
    try testing.expect(sampling.adaptive.velocity[0] > 0.5);
    // Avoidance works in the xz plane.
    try support.expectFloat(0.0, sampling.grid.velocity[1]);
    try support.expectFloat(0.0, sampling.adaptive.velocity[1]);
}

/// One circle each, around `plain_agent`.
const one_circle_scenes = [_]Circle{
    // Ahead and coming closer, a little to one side.
    .{
        .position = .{ 3, 0, 0.2 },
        .radius = 0.5,
        .velocity = .{ -1, 0, 0 },
        .desired_velocity = .{ -1, 0, 0 },
    },
    // Dead ahead and standing still.
    .{
        .position = .{ 2, 0, 0 },
        .radius = 0.5,
        .velocity = .{ 0, 0, 0 },
        .desired_velocity = .{ 0, 0, 0 },
    },
    // Behind and catching up.
    .{
        .position = .{ -3, 0, 0 },
        .radius = 0.5,
        .velocity = .{ 1.75, 0, 0 },
        .desired_velocity = .{ 2, 0, 0 },
    },
    // Beside, moving as the agent does and wanting what it wants.
    .{
        .position = .{ 0, 0, 2 },
        .radius = 0.5,
        .velocity = .{ 1.0, 0, 0 },
        .desired_velocity = .{ 1.5, 0, 0.5 },
    },
    // Overlapping the agent.
    .{
        .position = .{ 0.3, 0, 0.2 },
        .radius = 0.5,
        .velocity = .{ 0.5, 0, -0.5 },
        .desired_velocity = .{ 0, 0, -1 },
    },
    // Straight above the agent: a direction with nothing in the plane.
    .{
        .position = .{ 0, 1.5, 0 },
        .radius = 0.5,
        .velocity = .{ 0, 0, 1 },
        .desired_velocity = .{ 1, 0, 1 },
    },
    // Without a radius.
    .{
        .position = .{ 2, 0, 0.5 },
        .radius = 0.0,
        .velocity = .{ -0.5, 0, 0 },
        .desired_velocity = .{ -0.5, 0, 0 },
    },
    // Far out of reach.
    .{
        .position = .{ 50, 2, -50 },
        .radius = 1.0,
        .velocity = .{ -2, 0, 2 },
        .desired_velocity = .{ -2, 0, 2 },
    },
};

test "sampleVelocityGrid, sampleVelocityAdaptive: one circle" {
    for (one_circle_scenes, 0..) |circle, scene| {
        errdefer std.debug.print("  scene {d}\n", .{scene});
        const sampling = try expectSameScene(plain_agent, &.{circle}, &.{}, default_params);
        try testing.expect(sampling.grid.count > 0);
        try testing.expect(sampling.adaptive.count > 0);
    }
}

test "sampleVelocityGrid, sampleVelocityAdaptive: a circle exactly on the agent" {
    const gpa = testing.allocator;
    var pair = try QueryPair.init(gpa, crowd_circle_count_max, crowd_segment_count_max);
    defer pair.deinit(gpa);
    pair.addCircle(.{
        .position = plain_agent.position,
        .radius = 0.5,
        .velocity = .{ 0, 0, 0 },
        .desired_velocity = .{ 0, 0, 0 },
    });

    for ([_]Sampler{ .grid, .adaptive }) |sampler| {
        errdefer std.debug.print("  {s}\n", .{@tagName(sampler)});
        const sampling = try expectSameRecorded(&pair, sampler, plain_agent, default_params);
        // The circle has no direction, so no candidate's penalty is a number
        // and none is chosen.
        try testing.expectEqual(SideCompare.any_nan, pair.sideCompare());
        try testing.expect(sampling.count > 0);
        try support.expectPoint(.{ 0, 0, 0 }, sampling.velocity);
    }

    // The same among ordinary circles and a wall.
    pair.addCircle(one_circle_scenes[0]);
    pair.addCircle(one_circle_scenes[4]);
    pair.addSegment(.{ .p = .{ 2, 0, -2 }, .q = .{ 2, 0, 2 } });
    for ([_]Sampler{ .grid, .adaptive }) |sampler| {
        errdefer std.debug.print("  {s}, among others\n", .{@tagName(sampler)});
        const sampling = try expectSameRecorded(&pair, sampler, plain_agent, default_params);
        try testing.expectEqual(SideCompare.any_nan, pair.sideCompare());
        try support.expectPoint(.{ 0, 0, 0 }, sampling.velocity);
    }
}

/// One wall segment each, around `plain_agent`.
const one_segment_scenes = [_]Segment{
    // Across the agent's way.
    .{ .p = .{ 2, 0, -2 }, .q = .{ 2, 0, 2 } },
    // The same wall the other way round.
    .{ .p = .{ 2, 0, 2 }, .q = .{ 2, 0, -2 } },
    // Along the agent's way, beside it.
    .{ .p = .{ -2, 0, 1 }, .q = .{ 4, 0, 1 } },
    // Behind the agent.
    .{ .p = .{ -2, 0, -2 }, .q = .{ -2, 0, 2 } },
    // Slanted, and at another height at each end.
    .{ .p = .{ 1, -3, -1 }, .q = .{ 3, 5, 2 } },
    // Without length, ahead of the agent.
    .{ .p = .{ 1, 0, 0.25 }, .q = .{ 1, 0, 0.25 } },
    // Without length, on the agent.
    .{ .p = .{ 0, 0, 0 }, .q = .{ 0, 0, 0 } },
    // Starting on the agent.
    .{ .p = .{ 0, 0, 0 }, .q = .{ 3, 0, 1 } },
    // Ending on the agent.
    .{ .p = .{ -1, 0, 3 }, .q = .{ 0, 0, 0 } },
    // Through the agent.
    .{ .p = .{ -1, 0, -2 }, .q = .{ 0.5, 0, 1 } },
    // Far out of reach.
    .{ .p = .{ 40, 0, 40 }, .q = .{ 60, 0, 45 } },
};

test "sampleVelocityGrid, sampleVelocityAdaptive: one segment" {
    for (one_segment_scenes, 0..) |segment, scene| {
        errdefer std.debug.print("  scene {d}\n", .{scene});
        const sampling = try expectSameScene(plain_agent, &.{}, &.{segment}, default_params);
        try testing.expect(sampling.grid.count > 0);
        try testing.expect(sampling.adaptive.count > 0);
    }
}

test "sampleVelocityGrid, sampleVelocityAdaptive: a wall about as near as touching reaches" {
    const gpa = testing.allocator;
    // Below this distance from a wall the agent counts as touching it.
    const reach: f32 = 0.01;
    for (touch_distances) |distance| {
        for ([_]bool{ false, true }) |reversed| {
            errdefer std.debug.print("  distance {d}, reversed {}\n", .{ distance, reversed });
            var pair = try QueryPair.init(gpa, 1, 1);
            defer pair.deinit(gpa);

            // Along x, so that the agent's distance from it is `distance`
            // to the last bit.
            const wall: Segment = .{ .p = .{ -1, 0, distance }, .q = .{ 1, 0, distance } };
            pair.addSegment(if (reversed) .{ .p = wall.q, .q = wall.p } else wall);

            const touching = distance * distance < reach * reach;
            for ([_]Sampler{ .grid, .adaptive }) |sampler| {
                const sampling =
                    try expectSameRecorded(&pair, sampler, plain_agent, default_params);
                try testing.expect(sampling.count > 0);
                try testing.expectEqual(touching, pair.query.getObstacleSegment(0).touch);
                try testing.expectEqual(@intFromBool(touching), pair.touchCount());
            }
        }
    }
}

test "sampleVelocityGrid, sampleVelocityAdaptive: a full query, and obstacles it had no room for" {
    const gpa = testing.allocator;
    var pair = try QueryPair.init(gpa, crowd_circle_count_max, crowd_segment_count_max);
    defer pair.deinit(gpa);

    // Three more of each kind than there is room for.
    for (0..crowd_circle_count_max + 3) |i| {
        pair.addCircle(one_circle_scenes[i % one_circle_scenes.len]);
    }
    for (0..crowd_segment_count_max + 3) |i| {
        pair.addSegment(one_segment_scenes[i % one_segment_scenes.len]);
    }
    try expectObstacleCounts(&pair, crowd_circle_count_max, crowd_segment_count_max);

    for ([_]Sampler{ .grid, .adaptive }) |sampler| {
        errdefer std.debug.print("  {s}\n", .{@tagName(sampler)});
        const sampling = try expectSameRecorded(&pair, sampler, plain_agent, default_params);
        try testing.expect(sampling.count > 0);

        // The obstacles sampled among are the first added.
        try expectObstacleCounts(&pair, crowd_circle_count_max, crowd_segment_count_max);
        for (0..crowd_circle_count_max) |i| {
            const circle = one_circle_scenes[i % one_circle_scenes.len];
            try expectHoldsCircle(&pair, @intCast(i), circle);
        }
        for (0..crowd_segment_count_max) |i| {
            const segment = one_segment_scenes[i % one_segment_scenes.len];
            try expectHoldsSegment(&pair, @intCast(i), segment);
        }
    }
}

/// A few obstacles around `plain_agent`: circles ahead of it, behind it and
/// overlapping it, a wall across its way and one beside it.
const busy_circles = [_]Circle{
    one_circle_scenes[0],
    one_circle_scenes[2],
    one_circle_scenes[4],
};
const busy_segments = [_]Segment{ one_segment_scenes[0], one_segment_scenes[2] };

/// Both queries with the crowd's room, holding the busy scene.
fn initBusy(gpa: std.mem.Allocator) !QueryPair {
    var pair = try QueryPair.init(gpa, crowd_circle_count_max, crowd_segment_count_max);
    for (busy_circles) |circle| pair.addCircle(circle);
    for (busy_segments) |segment| pair.addSegment(segment);
    return pair;
}

test "sampleVelocityGrid: grid sizes from none to the largest" {
    const gpa = testing.allocator;
    var pair = try initBusy(gpa);
    defer pair.deinit(gpa);

    for ([_]u8{ 0, 1, 2, 3, 4, 5, 33, 255 }) |grid_size| {
        errdefer std.debug.print("  grid size {d}\n", .{grid_size});
        var params = default_params;
        params.grid_size = grid_size;
        const sampling = try expectSameRecorded(&pair, .grid, plain_agent, params);
        switch (grid_size) {
            // No candidates at all.
            0 => {
                try testing.expectEqual(0, sampling.count);
                try support.expectPoint(.{ 0, 0, 0 }, sampling.velocity);
            },
            // One candidate, which a grid step of a division by zero leaves
            // without a number for a velocity: scored, never chosen.
            1 => {
                try testing.expectEqual(1, sampling.count);
                try support.expectPoint(.{ 0, 0, 0 }, sampling.velocity);
            },
            else => try testing.expect(sampling.count > 0),
        }
    }
}

test "sampleVelocityAdaptive: every odd division count, any ring count, depths from none" {
    const gpa = testing.allocator;
    var pair = try initBusy(gpa);
    defer pair.deinit(gpa);

    // No divisions, which the sampler takes as one, then every odd count.
    var divisions: u8 = 0;
    while (divisions <= 31) : (divisions += if (divisions == 0) 1 else 2) {
        for ([_]u8{ 0, 1, 2, 3, 4, 5, 255 }) |rings| {
            for ([_]u8{ 0, 1, 2, 3 }) |depth| {
                errdefer std.debug.print(
                    "  {d} divisions, {d} rings, depth {d}\n",
                    .{ divisions, rings, depth },
                );
                var params = default_params;
                params.adaptive_divisions = divisions;
                params.adaptive_rings = rings;
                params.adaptive_depth = depth;
                const sampling = try expectSameRecorded(&pair, .adaptive, plain_agent, params);
                switch (depth) {
                    // No pattern is tried: the velocity the search would
                    // have started from comes back.
                    0 => {
                        try testing.expectEqual(0, sampling.count);
                        try support.expectPoint(.{
                            plain_agent.desired_velocity[0] * params.vel_bias,
                            0,
                            plain_agent.desired_velocity[2] * params.vel_bias,
                        }, sampling.velocity);
                    },
                    // The first pattern lies well within the agent's
                    // greatest speed, so every point of it is scored: the
                    // centre and each ring's divisions.
                    1 => try testing.expectEqual(
                        candidateCountMax(.adaptive, params),
                        sampling.count,
                    ),
                    else => try testing.expect(sampling.count > 0),
                }
            }
        }
    }
}

test "sampleVelocityAdaptive: the deepest search of the largest pattern" {
    const gpa = testing.allocator;
    var pair = try initBusy(gpa);
    defer pair.deinit(gpa);

    var params = default_params;
    params.adaptive_divisions = 31;
    params.adaptive_rings = reference_ring_count_max;
    params.adaptive_depth = 255;
    // The pattern halves at every depth, and long before the last it has
    // no size left: every candidate is then the best one so far.
    const sampling = try expectSameRecorded(&pair, .adaptive, plain_agent, params);
    try testing.expect(sampling.count > 0);
}

/// The crowd's parameters with the float named `field` set to `value`.
fn changedParams(comptime field: []const u8, value: f32) ObstacleAvoidanceParams {
    var params = default_params;
    @field(params, field) = value;
    return params;
}

/// The crowd's parameters with other weights: of straying from the desired
/// velocity, of straying from the current one, of the side and of the time
/// of impact.
fn weightedParams(weights: [4]f32) ObstacleAvoidanceParams {
    var params = default_params;
    params.weight_desired_velocity = weights[0];
    params.weight_current_velocity = weights[1];
    params.weight_side = weights[2];
    params.weight_time_of_impact = weights[3];
    return params;
}

/// Parameters at and past the ends of what makes sense.
const awkward_params = [_]ObstacleAvoidanceParams{
    // Candidates spread around standing still.
    changedParams("vel_bias", 0.0),
    // Every candidate is the desired velocity.
    changedParams("vel_bias", 1.0),
    // Past the desired velocity, which turns the spread inside out.
    changedParams("vel_bias", 1.5),
    // Centred behind standing still.
    changedParams("vel_bias", -0.5),
    // One weight missing at a time.
    weightedParams(.{ 0.0, 0.75, 0.75, 2.5 }),
    weightedParams(.{ 2.0, 0.0, 0.75, 2.5 }),
    weightedParams(.{ 2.0, 0.75, 0.0, 2.5 }),
    weightedParams(.{ 2.0, 0.75, 0.75, 0.0 }),
    // One weight alone.
    weightedParams(.{ 2.0, 0.0, 0.0, 0.0 }),
    weightedParams(.{ 0.0, 0.75, 0.0, 0.0 }),
    weightedParams(.{ 0.0, 0.0, 0.75, 0.0 }),
    weightedParams(.{ 0.0, 0.0, 0.0, 2.5 }),
    // No weights: every candidate scores nothing, and the first is kept.
    weightedParams(.{ 0.0, 0.0, 0.0, 0.0 }),
    // Rewards in place of penalties.
    weightedParams(.{ -2.0, 0.75, 0.75, 2.5 }),
    weightedParams(.{ 2.0, -0.75, 0.75, 2.5 }),
    weightedParams(.{ 2.0, 0.75, -0.75, 2.5 }),
    weightedParams(.{ 2.0, 0.75, 0.75, -2.5 }),
    weightedParams(.{ -2.0, -0.75, -0.75, -2.5 }),
    // No time to look ahead, less than none, hardly any and far too much.
    changedParams("horizon_time", 0.0),
    changedParams("horizon_time", -1.0),
    changedParams("horizon_time", 0.001),
    changedParams("horizon_time", 1.0e6),
};

/// Agents at the origin, each awkward in one way.
const awkward_agents = [_]Agent{
    plain_agent,
    // Unable to move, and with a greatest speed below zero.
    .{
        .position = .{ 0, 0, 0 },
        .radius = 0.5,
        .velocity_max = 0.0,
        .velocity = .{ 1.0, 0, 0 },
        .desired_velocity = .{ 1.5, 0, 0.5 },
    },
    .{
        .position = .{ 0, 0, 0 },
        .radius = 0.5,
        .velocity_max = -1.5,
        .velocity = .{ 1.0, 0, 0 },
        .desired_velocity = .{ 1.5, 0, 0.5 },
    },
    // Standing still.
    .{
        .position = .{ 0, 0, 0 },
        .radius = 0.5,
        .velocity_max = 2.0,
        .velocity = .{ 0, 0, 0 },
        .desired_velocity = .{ 1.5, 0, 0.5 },
    },
    // Wanting to stand still, which leaves the pattern without a direction.
    .{
        .position = .{ 0, 0, 0 },
        .radius = 0.5,
        .velocity_max = 2.0,
        .velocity = .{ 1.0, 0, 0 },
        .desired_velocity = .{ 0, 0, 0 },
    },
    // Standing still and wanting to.
    .{
        .position = .{ 0, 0, 0 },
        .radius = 0.5,
        .velocity_max = 2.0,
        .velocity = .{ 0, 0, 0 },
        .desired_velocity = .{ 0, 0, 0 },
    },
    // Moving just as it wants to.
    .{
        .position = .{ 0, 0, 0 },
        .radius = 0.5,
        .velocity_max = 2.0,
        .velocity = .{ 1.5, 0, 0.5 },
        .desired_velocity = .{ 1.5, 0, 0.5 },
    },
    // Without a radius.
    .{
        .position = .{ 0, 0, 0 },
        .radius = 0.0,
        .velocity_max = 2.0,
        .velocity = .{ 1.0, 0, 0 },
        .desired_velocity = .{ 1.5, 0, 0.5 },
    },
    // Wanting more than its greatest speed, and moving faster than it too.
    .{
        .position = .{ 0, 0, 0 },
        .radius = 0.5,
        .velocity_max = 1.0,
        .velocity = .{ -2.0, 0, 2.0 },
        .desired_velocity = .{ 3.0, 0, 4.0 },
    },
    // Moving and wanting to move out of the plane as well.
    .{
        .position = .{ 0, 0, 0 },
        .radius = 0.5,
        .velocity_max = 2.0,
        .velocity = .{ 1.0, 3.0, 0 },
        .desired_velocity = .{ 1.5, -2.0, 0.5 },
    },
};

test "sampleVelocityGrid, sampleVelocityAdaptive: awkward parameters for awkward agents" {
    const gpa = testing.allocator;
    var bare = try QueryPair.init(gpa, crowd_circle_count_max, crowd_segment_count_max);
    defer bare.deinit(gpa);
    var busy = try initBusy(gpa);
    defer busy.deinit(gpa);

    for ([_]*QueryPair{ &bare, &busy }, 0..) |pair, scene| {
        for (awkward_agents, 0..) |agent, agent_index| {
            for (awkward_params, 0..) |params, params_index| {
                errdefer std.debug.print(
                    "  scene {d}, agent {d}, parameters {d}\n",
                    .{ scene, agent_index, params_index },
                );
                _ = try expectSameRecorded(pair, .grid, agent, params);
                _ = try expectSameRecorded(pair, .adaptive, agent, params);
            }
        }
    }
}

test "sampleVelocityGrid, sampleVelocityAdaptive: no time to look ahead gives up every candidate" {
    const gpa = testing.allocator;
    var pair = try initBusy(gpa);
    defer pair.deinit(gpa);

    // None, and less than none.
    for ([_]f32{ 0.0, -1.0 }) |horizon_time| {
        const params = changedParams("horizon_time", horizon_time);
        for ([_]Sampler{ .grid, .adaptive }) |sampler| {
            errdefer std.debug.print(
                "  horizon {d}, {s}\n",
                .{ horizon_time, @tagName(sampler) },
            );
            var debug = try ampleDebug(gpa, sampler, params);
            defer debug.deinit(gpa);
            const sampling = try pair.expectSameSampling(sampler, plain_agent, params, &debug);
            // Each candidate slow enough is counted, but none is scored to
            // the end: nothing is recorded and nothing chosen.
            try testing.expect(sampling.count > 0);
            try testing.expectEqual(0, debug.debug.getSampleCount());
            try support.expectPoint(.{ 0, 0, 0 }, sampling.velocity);
        }
    }
}

/// Sample `i` of the port's recorder, read through every getter.
fn sampleAt(debug: *const ObstacleAvoidanceDebug, i: i32) Sample {
    return .{
        .velocity = debug.getSampleVelocity(i),
        .size = debug.getSampleSize(i),
        .penalty = debug.getSamplePenalty(i),
        .desired_velocity_penalty = debug.getSampleDesiredVelocityPenalty(i),
        .current_velocity_penalty = debug.getSampleCurrentVelocityPenalty(i),
        .preferred_side_penalty = debug.getSamplePreferredSidePenalty(i),
        .collision_time_penalty = debug.getSampleCollisionTimePenalty(i),
    };
}

/// Fails unless the two samples have the same bits, the total penalty and
/// the side penalty compared as `compare` says.
fn expectSample(expected: Sample, actual: Sample, compare: SideCompare) !void {
    try expectNamedPoint("velocity", expected.velocity, actual.velocity);
    try expectNamed("size", expected.size, actual.size, .exact);
    try expectNamed("penalty", expected.penalty, actual.penalty, compare);
    try expectNamed(
        "desired velocity penalty",
        expected.desired_velocity_penalty,
        actual.desired_velocity_penalty,
        .exact,
    );
    try expectNamed(
        "current velocity penalty",
        expected.current_velocity_penalty,
        actual.current_velocity_penalty,
        .exact,
    );
    try expectNamed(
        "preferred side penalty",
        expected.preferred_side_penalty,
        actual.preferred_side_penalty,
        compare,
    );
    try expectNamed(
        "collision time penalty",
        expected.collision_time_penalty,
        actual.collision_time_penalty,
        .exact,
    );
}

/// Makes the `sampler` call on both sides of `pair` with a recorder that
/// has room for `room` samples, right after the same call with the ample
/// recorder `ample`, which gave `whole`. Fails unless the two sides agree,
/// the small recorder changes neither the count nor the chosen velocity,
/// and it holds the first samples of the ample one, as many as it has room
/// for. Returns how many that is.
fn expectSameTruncated(
    pair: *QueryPair,
    sampler: Sampler,
    agent: Agent,
    params: ObstacleAvoidanceParams,
    ample: *const DebugPair,
    whole: Sampling,
    room: i32,
) !i32 {
    const gpa = testing.allocator;
    errdefer std.debug.print("  a recorder with room for {d}\n", .{room});
    var small = try DebugPair.init(gpa, room);
    defer small.deinit(gpa);

    const sampling = try pair.expectSameSampling(sampler, agent, params, &small);
    try testing.expectEqual(whole.count, sampling.count);
    try support.expectPoint(whole.velocity, sampling.velocity);

    const kept = @min(room, ample.debug.getSampleCount());
    try testing.expectEqual(kept, small.debug.getSampleCount());
    // The two calls may run through differently compiled copies of the
    // sampler, so a NaN's sign is no more settled here than between the
    // port and the original.
    const compare = pair.sideCompare();
    var i: i32 = 0;
    while (i < kept) : (i += 1) {
        errdefer std.debug.print("  sample {d} of the {d} kept\n", .{ i, kept });
        try expectSample(sampleAt(&ample.debug, i), sampleAt(&small.debug, i), compare);
    }
    return kept;
}

test "sampleVelocityGrid, sampleVelocityAdaptive: a recorder one short, exact and one over" {
    const gpa = testing.allocator;
    var pair = try initBusy(gpa);
    defer pair.deinit(gpa);
    // A grid small enough to read, should a sample differ.
    var params = default_params;
    params.grid_size = 9;

    for ([_]Sampler{ .grid, .adaptive }) |sampler| {
        errdefer std.debug.print("  {s}\n", .{@tagName(sampler)});
        var ample = try ampleDebug(gpa, sampler, params);
        defer ample.deinit(gpa);
        const whole = try pair.expectSameSampling(sampler, plain_agent, params, &ample);
        // The first candidate is scored to the end, and so is each better
        // one after it.
        const full = ample.debug.getSampleCount();
        try testing.expect(full >= 2);

        for ([_]i32{ 1, full - 1, full, full + 1 }) |room| {
            const kept = try expectSameTruncated(
                &pair,
                sampler,
                plain_agent,
                params,
                &ample,
                whole,
                room,
            );
            try testing.expectEqual(@min(room, full), kept);
        }
    }
}

/// What the sampling calls of one random test reached, to show that the
/// scenes were not all of one easy kind.
const Reach = struct {
    /// Calls that scored a candidate.
    scored: usize = 0,
    /// Calls that scored some candidates and passed over others as too
    /// fast.
    passed_over: usize = 0,
    /// Calls that gave a candidate up before scoring it to the end.
    gave_up: usize = 0,
    /// Calls that chose a velocity other than standing still.
    chose: usize = 0,
    /// Calls for an agent touching a wall.
    touching: usize = 0,
    /// Calls among circles of which one, exactly on the agent, had no
    /// direction.
    undirected: usize = 0,
    /// Recorders with less room than the candidates scored to the end.
    truncated: usize = 0,
};

/// Room for obstacles of one kind: mostly the crowd's `usual`, now and then
/// none, one, or anything up to twice the usual.
fn randomRoom(random: std.Random, usual: i32) i32 {
    return switch (random.uintLessThan(u32, 8)) {
        0 => 0,
        1 => 1,
        2 => random.intRangeAtMost(i32, 2, usual * 2),
        else => usual,
    };
}

/// One sampling of a random scene: the `sampler` call for `agent` on both
/// sides of `pair` with a recorder that has room for every candidate, then,
/// each as often as not, with a smaller recorder, with none, and the
/// recorded samples rescaled. Notes in `reach` what the call came to.
fn expectSameRound(
    pair: *QueryPair,
    random: std.Random,
    sampler: Sampler,
    agent: Agent,
    params: ObstacleAvoidanceParams,
    reach: *Reach,
) !void {
    const gpa = testing.allocator;
    var ample = try ampleDebug(gpa, sampler, params);
    defer ample.deinit(gpa);

    const whole = try pair.expectSameSampling(sampler, agent, params, &ample);
    const count_max = candidateCountMax(sampler, params);
    const full = ample.debug.getSampleCount();
    try testing.expect(whole.count >= 0);
    try testing.expect(whole.count <= count_max);
    // Only a candidate scored to the end is recorded.
    try testing.expect(full <= whole.count);

    if (whole.count > 0) reach.scored += 1;
    if (whole.count > 0 and whole.count < count_max) reach.passed_over += 1;
    if (full < whole.count) reach.gave_up += 1;
    if (whole.velocity[0] != 0 or whole.velocity[2] != 0) reach.chose += 1;
    if (pair.touchCount() > 0) reach.touching += 1;
    if (pair.sideCompare() == .any_nan) reach.undirected += 1;

    if (random.boolean()) {
        const room = random.intRangeAtMost(i32, 1, full + 1);
        const kept =
            try expectSameTruncated(pair, sampler, agent, params, &ample, whole, room);
        if (kept < full) reach.truncated += 1;
    }
    if (random.boolean()) {
        // A recorder leaves the count and the chosen velocity as they are
        // without one.
        const plain = try pair.expectSameSampling(sampler, agent, params, null);
        try testing.expectEqual(whole.count, plain.count);
        try support.expectPoint(whole.velocity, plain.velocity);
    }
    if (random.boolean()) {
        ample.normalizeSamples();
        _ = try ample.expectSame(pair.sideCompare());
    }
}

/// `case_count` random scenes from `seed`, each in queries of random room
/// and sampled for one to three rounds. Every round after the first either
/// builds a new scene in the same queries or keeps the obstacles, as the
/// last sampling left them, for another agent in the same place. `only`,
/// when given, is the sampler every round calls; without it each round
/// picks one, so the two samplers follow each other on the same queries.
fn expectSameRandomScenes(only: ?Sampler, seed: u64, case_count: usize) !void {
    const gpa = testing.allocator;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();
    var reach: Reach = .{};

    for (0..case_count) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const circle_count_max = randomRoom(random, crowd_circle_count_max);
        const segment_count_max = randomRoom(random, crowd_segment_count_max);
        var pair = try QueryPair.init(gpa, circle_count_max, segment_count_max);
        defer pair.deinit(gpa);

        var agent = randomAgent(random);
        fillRandomScene(&pair, random, agent, circle_count_max, segment_count_max);
        const round_count = random.intRangeAtMost(u32, 1, 3);
        for (0..round_count) |round| {
            errdefer std.debug.print("  round {d} of {d}\n", .{ round, round_count });
            if (round > 0) {
                if (random.boolean()) {
                    agent = randomAgent(random);
                    fillRandomScene(&pair, random, agent, circle_count_max, segment_count_max);
                } else {
                    agent = randomAgentAt(random, agent.position);
                }
            }
            const sampler = only orelse random.enumValue(Sampler);
            try expectSameRound(&pair, random, sampler, agent, randomParams(random), &reach);
        }
    }

    // Every kind of sampling there is came up.
    try testing.expect(reach.scored > 0);
    try testing.expect(reach.passed_over > 0);
    try testing.expect(reach.gave_up > 0);
    try testing.expect(reach.chose > 0);
    try testing.expect(reach.touching > 0);
    try testing.expect(reach.undirected > 0);
    try testing.expect(reach.truncated > 0);
}

test "sampleVelocityGrid: random agents among random obstacles" {
    try expectSameRandomScenes(.grid, 0x67726964, 1500);
}

test "sampleVelocityAdaptive: random agents among random obstacles" {
    try expectSameRandomScenes(.adaptive, 0x6164617074697665, 1500);
}

test "sampleVelocityGrid, sampleVelocityAdaptive: random scenes, the two in turn" {
    try expectSameRandomScenes(null, 0x696e7475726e, 1000);
}
