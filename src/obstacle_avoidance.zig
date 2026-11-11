//! Local obstacle avoidance: picks the velocity for an agent that keeps it
//! clear of moving circles (other agents) and wall segments while staying
//! close to the velocity it wants.
//!
//! A query holds the obstacles around one agent, samples candidate
//! velocities on a grid or an adaptive pattern, and gives each a penalty for
//! straying from the desired and current velocities, for passing obstacles
//! on the far side, and for a short time to impact. The candidate with the
//! lowest penalty wins. Everything works in the xz plane.

const std = @import("std");
const common = @import("common.zig");

const pi: f32 = 3.14159265;

/// A moving circle to avoid, as another agent is.
pub const ObstacleCircle = struct {
    position: [3]f32,
    velocity: [3]f32,
    /// The velocity the obstacle wants.
    desired_velocity: [3]f32,
    radius: f32,
    /// The unit vector from the sampling agent to the obstacle, set by each
    /// sampling call and stale until then.
    direction: [3]f32,
    /// The side of `direction` the agent should pass on, set with it; its
    /// y stays 0.
    side_normal: [3]f32,
};

/// A wall segment to avoid.
pub const ObstacleSegment = struct {
    /// The segment's end points.
    p: [3]f32,
    q: [3]f32,
    /// Whether the sampling agent is all but on the segment, set by each
    /// sampling call and stale until then.
    touch: bool,
};

const CircleSweep = struct {
    /// When the circles start to overlap.
    t_min: f32,
    /// When the circles stop overlapping.
    t_max: f32,
};

/// The times at which a circle at `c0` of radius `r0`, moving by `v` each
/// unit of time, starts and stops overlapping the circle at `c1` of radius
/// `r1`; null when it hardly moves or never reaches it.
fn sweepCircleCircle(c0: [3]f32, r0: f32, v: [3]f32, c1: [3]f32, r1: f32) ?CircleSweep {
    const eps: f32 = 0.0001;
    const s = common.vsub(c1, c0);
    const r = r0 + r1;
    const c = common.vdot2d(s, s) - r * r;
    var a = common.vdot2d(v, v);
    if (a < eps) return null; // Not moving.

    // Overlap, calculate the time to exit.
    const b = common.vdot2d(v, s);
    const d = b * b - a * c;
    if (d < 0.0) return null; // No intersection.
    a = 1.0 / a;
    const rd = @sqrt(d);
    return .{
        .t_min = (b - rd) * a,
        .t_max = (b + rd) * a,
    };
}

/// How far along `u`, as a fraction of its length, the ray from `ap`
/// crosses the segment `bp`..`bq`; null when it does not cross it within
/// `u`'s length.
fn intersectRaySegment(ap: [3]f32, u: [3]f32, bp: [3]f32, bq: [3]f32) ?f32 {
    const v = common.vsub(bq, bp);
    const w = common.vsub(ap, bp);
    var d = common.vperp2d(u, v);
    if (@abs(d) < 1e-6) return null;
    d = 1.0 / d;
    const t = common.vperp2d(v, w) * d;
    if (t < 0 or t > 1) return null;
    const s = common.vperp2d(u, w) * d;
    if (s < 0 or s > 1) return null;
    return t;
}

/// Rescales `arr` so its smallest entry is 0 and its largest 1. A range
/// of 0.001 or less is shifted to start at 0 and not stretched.
fn normalizeArray(arr: []f32) void {
    // Normalize the penalty range.
    var penalty_min: f32 = std.math.floatMax(f32);
    var penalty_max: f32 = -std.math.floatMax(f32);
    for (arr) |penalty| {
        penalty_min = common.min(f32, penalty_min, penalty);
        penalty_max = common.max(f32, penalty_max, penalty);
    }
    const penalty_range = penalty_max - penalty_min;
    const s: f32 = if (penalty_range > 0.001) (1.0 / penalty_range) else 1;
    for (arr) |*penalty| {
        penalty.* = common.clamp(f32, (penalty.* - penalty_min) * s, 0.0, 1.0);
    }
}

/// The candidate velocities one sampling call tried and the penalties each
/// got, for drawing how an agent chose its velocity. A sampling call handed
/// one empties it and adds every candidate it scores, up to the capacity.
pub const ObstacleAvoidanceDebug = struct {
    sample_count: i32,
    sample_count_max: i32,
    velocities: [][3]f32,
    /// The size to draw each sample at, in velocity units.
    sizes: []f32,
    /// Each sample's total penalty: the sum of the four below.
    penalties: []f32,
    desired_velocity_penalties: []f32,
    current_velocity_penalties: []f32,
    preferred_side_penalties: []f32,
    collision_time_penalties: []f32,

    /// A sample store that holds no memory, for a field to hold before
    /// `init`. It takes no samples.
    pub const empty: ObstacleAvoidanceDebug = .{
        .sample_count = 0,
        .sample_count_max = 0,
        .velocities = &.{},
        .sizes = &.{},
        .penalties = &.{},
        .desired_velocity_penalties = &.{},
        .current_velocity_penalties = &.{},
        .preferred_side_penalties = &.{},
        .collision_time_penalties = &.{},
    };

    /// Makes room for `sample_count_max` samples, which is not zero.
    /// Returns false when memory runs out, leaving the store `empty`.
    pub fn init(
        self: *ObstacleAvoidanceDebug,
        gpa: std.mem.Allocator,
        sample_count_max: i32,
    ) bool {
        std.debug.assert(sample_count_max != 0);
        self.* = allocate(gpa, sample_count_max) catch {
            self.* = empty;
            return false;
        };
        return true;
    }

    fn allocate(
        gpa: std.mem.Allocator,
        sample_count_max: i32,
    ) error{OutOfMemory}!ObstacleAvoidanceDebug {
        // A negative capacity asks for more memory than there is.
        const len = std.math.cast(usize, sample_count_max) orelse return error.OutOfMemory;

        const velocities = try gpa.alloc([3]f32, len);
        errdefer gpa.free(velocities);
        const penalties = try gpa.alloc(f32, len);
        errdefer gpa.free(penalties);
        const sizes = try gpa.alloc(f32, len);
        errdefer gpa.free(sizes);
        const desired_velocity_penalties = try gpa.alloc(f32, len);
        errdefer gpa.free(desired_velocity_penalties);
        const current_velocity_penalties = try gpa.alloc(f32, len);
        errdefer gpa.free(current_velocity_penalties);
        const preferred_side_penalties = try gpa.alloc(f32, len);
        errdefer gpa.free(preferred_side_penalties);
        const collision_time_penalties = try gpa.alloc(f32, len);

        return .{
            .sample_count = 0,
            .sample_count_max = sample_count_max,
            .velocities = velocities,
            .sizes = sizes,
            .penalties = penalties,
            .desired_velocity_penalties = desired_velocity_penalties,
            .current_velocity_penalties = current_velocity_penalties,
            .preferred_side_penalties = preferred_side_penalties,
            .collision_time_penalties = collision_time_penalties,
        };
    }

    /// Frees the samples' memory. `gpa` is the allocator `init` was given.
    pub fn deinit(self: *ObstacleAvoidanceDebug, gpa: std.mem.Allocator) void {
        gpa.free(self.velocities);
        gpa.free(self.sizes);
        gpa.free(self.penalties);
        gpa.free(self.desired_velocity_penalties);
        gpa.free(self.current_velocity_penalties);
        gpa.free(self.preferred_side_penalties);
        gpa.free(self.collision_time_penalties);
        self.* = undefined;
    }

    /// Drops every sample, keeping the memory.
    pub fn reset(self: *ObstacleAvoidanceDebug) void {
        self.sample_count = 0;
    }

    /// Records one candidate velocity with the size to draw it at, its
    /// total penalty and the four parts of that penalty. A sample beyond
    /// the capacity is dropped.
    pub fn addSample(
        self: *ObstacleAvoidanceDebug,
        velocity: [3]f32,
        size: f32,
        penalty: f32,
        desired_velocity_penalty: f32,
        current_velocity_penalty: f32,
        preferred_side_penalty: f32,
        collision_time_penalty: f32,
    ) void {
        if (self.sample_count >= self.sample_count_max) return;
        const i: usize = @intCast(self.sample_count);
        self.velocities[i] = velocity;
        self.sizes[i] = size;
        self.penalties[i] = penalty;
        self.desired_velocity_penalties[i] = desired_velocity_penalty;
        self.current_velocity_penalties[i] = current_velocity_penalty;
        self.preferred_side_penalties[i] = preferred_side_penalty;
        self.collision_time_penalties[i] = collision_time_penalty;
        self.sample_count += 1;
    }

    /// Rescales the total penalties, and each of the four parts on its own,
    /// to run from 0 to 1 across the samples.
    pub fn normalizeSamples(self: *ObstacleAvoidanceDebug) void {
        const n: usize = @intCast(self.sample_count);
        normalizeArray(self.penalties[0..n]);
        normalizeArray(self.desired_velocity_penalties[0..n]);
        normalizeArray(self.current_velocity_penalties[0..n]);
        normalizeArray(self.preferred_side_penalties[0..n]);
        normalizeArray(self.collision_time_penalties[0..n]);
    }

    /// The number of samples recorded since the last `reset`.
    pub fn getSampleCount(self: *const ObstacleAvoidanceDebug) i32 {
        return self.sample_count;
    }

    /// The velocity of sample `i`, which is below `getSampleCount`, as are
    /// the indices the getters below take.
    pub fn getSampleVelocity(self: *const ObstacleAvoidanceDebug, i: i32) [3]f32 {
        return self.velocities[@intCast(i)];
    }

    /// The size to draw sample `i` at, in velocity units.
    pub fn getSampleSize(self: *const ObstacleAvoidanceDebug, i: i32) f32 {
        return self.sizes[@intCast(i)];
    }

    /// The total penalty of sample `i`.
    pub fn getSamplePenalty(self: *const ObstacleAvoidanceDebug, i: i32) f32 {
        return self.penalties[@intCast(i)];
    }

    /// The penalty sample `i` got for straying from the desired velocity.
    pub fn getSampleDesiredVelocityPenalty(
        self: *const ObstacleAvoidanceDebug,
        i: i32,
    ) f32 {
        return self.desired_velocity_penalties[@intCast(i)];
    }

    /// The penalty sample `i` got for straying from the current velocity.
    pub fn getSampleCurrentVelocityPenalty(
        self: *const ObstacleAvoidanceDebug,
        i: i32,
    ) f32 {
        return self.current_velocity_penalties[@intCast(i)];
    }

    /// The penalty sample `i` got for passing obstacles on the far side.
    pub fn getSamplePreferredSidePenalty(
        self: *const ObstacleAvoidanceDebug,
        i: i32,
    ) f32 {
        return self.preferred_side_penalties[@intCast(i)];
    }

    /// The penalty sample `i` got for its time to impact.
    pub fn getSampleCollisionTimePenalty(
        self: *const ObstacleAvoidanceDebug,
        i: i32,
    ) f32 {
        return self.collision_time_penalties[@intCast(i)];
    }
};

/// The most divisions an adaptive sampling ring is given.
pub const pattern_division_count_max: i32 = 32;
/// The most rings an adaptive sampling pattern is given.
pub const pattern_ring_count_max: i32 = 4;

/// How a sampling call spreads its candidates and weighs their penalties.
pub const ObstacleAvoidanceParams = extern struct {
    /// How far the sampled area is pulled toward the desired velocity: 0
    /// centres it on standing still and spans every speed up to the
    /// greatest, 1 shrinks it onto the desired velocity.
    vel_bias: f32,
    /// The weight of straying from the desired velocity.
    weight_desired_velocity: f32,
    /// The weight of straying from the current velocity.
    weight_current_velocity: f32,
    /// The weight of passing an obstacle on the far side.
    weight_side: f32,
    /// The weight of a short time to impact.
    weight_time_of_impact: f32,
    /// How far ahead, in time, impacts are looked for.
    horizon_time: f32,
    /// The candidates along each side of the grid `sampleVelocityGrid`
    /// tries.
    grid_size: u8,
    /// The candidates on each ring of the pattern `sampleVelocityAdaptive`
    /// tries, limited to 1..`pattern_division_count_max`.
    adaptive_divisions: u8,
    /// The rings of that pattern, limited to 1..`pattern_ring_count_max`.
    adaptive_rings: u8,
    /// How many times `sampleVelocityAdaptive` tries the pattern, each time
    /// at half the size around the best candidate so far.
    adaptive_depth: u8,
};

/// `v` scaled to unit length in the xz plane, its y kept. A vector with no
/// xz length is returned as it is.
inline fn normalize2d(v: [3]f32) [3]f32 {
    var d = @sqrt(v[0] * v[0] + v[2] * v[2]);
    if (d == 0) return v;
    d = 1.0 / d;
    return .{ v[0] * d, v[1], v[2] * d };
}

/// `v` turned by `angle` radians in the xz plane, its y kept.
inline fn rotate2d(v: [3]f32, angle: f32) [3]f32 {
    const c = @cos(angle);
    const s = @sin(angle);
    return .{ v[0] * c - v[2] * s, v[1], v[0] * s + v[2] * c };
}

/// Room for the largest adaptive pattern, the sample at zero, and one entry
/// more: a ring with an even division count writes its last point one entry
/// past the pattern's end.
const pattern_len = pattern_division_count_max * pattern_ring_count_max + 2;

/// The obstacles around one agent, and the sampling that picks its velocity
/// among them. Fill it with `addCircle` and `addSegment`, then sample;
/// `reset` empties it for the next agent.
pub const ObstacleAvoidanceQuery = struct {
    /// The parameters of the latest sampling call.
    params: ObstacleAvoidanceParams,
    horizon_time_inverse: f32,
    velocity_max: f32,
    velocity_max_inverse: f32,

    circle_count_max: i32,
    circles: []ObstacleCircle,
    circle_count: i32,

    segment_count_max: i32,
    segments: []ObstacleSegment,
    segment_count: i32,

    /// A query that holds no memory, for a field to hold before `init`. It
    /// takes no obstacles.
    pub const empty: ObstacleAvoidanceQuery = .{
        .params = .{
            .vel_bias = 0,
            .weight_desired_velocity = 0,
            .weight_current_velocity = 0,
            .weight_side = 0,
            .weight_time_of_impact = 0,
            .horizon_time = 0,
            .grid_size = 0,
            .adaptive_divisions = 0,
            .adaptive_rings = 0,
            .adaptive_depth = 0,
        },
        .horizon_time_inverse = 0,
        .velocity_max = 0,
        .velocity_max_inverse = 0,
        .circle_count_max = 0,
        .circles = &.{},
        .circle_count = 0,
        .segment_count_max = 0,
        .segments = &.{},
        .segment_count = 0,
    };

    /// Makes room for `circle_count_max` circles and `segment_count_max`
    /// segments. Returns false when memory runs out, leaving the query
    /// `empty`.
    pub fn init(
        self: *ObstacleAvoidanceQuery,
        gpa: std.mem.Allocator,
        circle_count_max: i32,
        segment_count_max: i32,
    ) bool {
        self.* = empty;

        // A negative capacity asks for more memory than there is.
        const circle_len = std.math.cast(usize, circle_count_max) orelse return false;
        const circles = gpa.alloc(ObstacleCircle, circle_len) catch return false;
        @memset(circles, std.mem.zeroes(ObstacleCircle));

        const segment_len = std.math.cast(usize, segment_count_max) orelse {
            gpa.free(circles);
            return false;
        };
        const segments = gpa.alloc(ObstacleSegment, segment_len) catch {
            gpa.free(circles);
            return false;
        };
        @memset(segments, std.mem.zeroes(ObstacleSegment));

        self.circle_count_max = circle_count_max;
        self.circles = circles;
        self.segment_count_max = segment_count_max;
        self.segments = segments;

        return true;
    }

    /// Frees the obstacles' memory. `gpa` is the allocator `init` was given.
    pub fn deinit(self: *ObstacleAvoidanceQuery, gpa: std.mem.Allocator) void {
        gpa.free(self.circles);
        gpa.free(self.segments);
        self.* = undefined;
    }

    /// Removes every obstacle, keeping the memory.
    pub fn reset(self: *ObstacleAvoidanceQuery) void {
        self.circle_count = 0;
        self.segment_count = 0;
    }

    /// Adds a moving circle to avoid. One beyond the capacity is dropped.
    pub fn addCircle(
        self: *ObstacleAvoidanceQuery,
        position: [3]f32,
        radius: f32,
        velocity: [3]f32,
        desired_velocity: [3]f32,
    ) void {
        if (self.circle_count >= self.circle_count_max) return;

        const circle = &self.circles[@intCast(self.circle_count)];
        self.circle_count += 1;
        circle.position = position;
        circle.radius = radius;
        circle.velocity = velocity;
        circle.desired_velocity = desired_velocity;
    }

    /// Adds the wall segment `p`..`q` to avoid. One beyond the capacity is
    /// dropped.
    pub fn addSegment(self: *ObstacleAvoidanceQuery, p: [3]f32, q: [3]f32) void {
        if (self.segment_count >= self.segment_count_max) return;

        const segment = &self.segments[@intCast(self.segment_count)];
        self.segment_count += 1;
        segment.p = p;
        segment.q = q;
    }

    /// Works out, for an agent at `position` wanting `desired_velocity`,
    /// the side to pass each circle on and which segments it touches.
    fn prepare(
        self: *ObstacleAvoidanceQuery,
        position: [3]f32,
        desired_velocity: [3]f32,
    ) void {
        // Prepare the obstacles.
        for (self.circles[0..@intCast(self.circle_count)]) |*circle| {
            // Side
            const pa = position;
            const pb = circle.position;

            const origin: [3]f32 = .{ 0, 0, 0 };
            circle.direction = common.vnormalize(common.vsub(pb, pa));
            const dv = common.vsub(circle.desired_velocity, desired_velocity);

            const a = common.triArea2d(origin, circle.direction, dv);
            if (a < 0.01) {
                circle.side_normal[0] = -circle.direction[2];
                circle.side_normal[2] = circle.direction[0];
            } else {
                circle.side_normal[0] = circle.direction[2];
                circle.side_normal[2] = -circle.direction[0];
            }
        }

        for (self.segments[0..@intCast(self.segment_count)]) |*segment| {
            // Precalculate if the agent is really close to the segment.
            const r: f32 = 0.01;
            const distance = common.distancePointSegmentSqr2d(position, segment.p, segment.q);
            segment.touch = distance.dist_sqr < common.sqr(f32, r);
        }
    }

    /// The penalty of the candidate velocity `vcand` for an agent at
    /// `position` of `radius`, moving at `velocity` and wanting
    /// `desired_velocity`. Once the penalty is sure to reach `penalty_min`,
    /// the best so far, the candidate is given up and `penalty_min`
    /// returned. `cs` is the size `debug` draws the candidate at.
    fn processSample(
        self: *const ObstacleAvoidanceQuery,
        vcand: [3]f32,
        cs: f32,
        position: [3]f32,
        radius: f32,
        velocity: [3]f32,
        desired_velocity: [3]f32,
        penalty_min: f32,
        debug: ?*ObstacleAvoidanceDebug,
    ) f32 {
        const params = &self.params;

        // The penalty for straying away from the desired and current
        // velocities.
        const vpen = params.weight_desired_velocity *
            (common.vdist2d(vcand, desired_velocity) * self.velocity_max_inverse);
        const vcpen = params.weight_current_velocity *
            (common.vdist2d(vcand, velocity) * self.velocity_max_inverse);

        // Find the threshold hit time to bail out based on the early out
        // penalty (see how the penalty is calculated below to understand).
        const penalty_left = penalty_min - vpen - vcpen;
        const t_threshold =
            (params.weight_time_of_impact / penalty_left - 0.1) * params.horizon_time;
        // Already too much.
        if (t_threshold - params.horizon_time > -std.math.floatEps(f32)) return penalty_min;

        // Find the min time of impact and exit amongst all obstacles.
        var t_min = params.horizon_time;
        var side: f32 = 0;
        var side_count: i32 = 0;

        for (self.circles[0..@intCast(self.circle_count)]) |*circle| {
            // The velocity relative to the obstacle, the obstacle taken to
            // do half of the avoiding (RVO).
            var vab = common.vscale(vcand, 2);
            vab = common.vsub(vab, velocity);
            vab = common.vsub(vab, circle.velocity);

            // Side: least for a candidate that leaves the obstacle behind
            // or on its preferred side.
            const ahead = common.vdot2d(circle.direction, vab) * 0.5 + 0.5;
            const aside = common.vdot2d(circle.side_normal, vab) * 2;
            side += common.clamp(f32, common.min(f32, ahead, aside), 0.0, 1.0);
            side_count += 1;

            const sweep = sweepCircleCircle(
                position,
                radius,
                vab,
                circle.position,
                circle.radius,
            ) orelse continue;
            var ht_min = sweep.t_min;

            // Handle overlapping obstacles.
            if (ht_min < 0.0 and sweep.t_max > 0.0) {
                // Avoid more when overlapped.
                ht_min = -ht_min * 0.5;
            }

            if (ht_min >= 0.0) {
                // The closest obstacle is somewhere ahead of us, keep track
                // of the nearest obstacle.
                if (ht_min < t_min) {
                    t_min = ht_min;
                    if (t_min < t_threshold) return penalty_min;
                }
            }
        }

        for (self.segments[0..@intCast(self.segment_count)]) |*segment| {
            var ht_min: f32 = 0;

            if (segment.touch) {
                // Special case when the agent is very close to the segment.
                const sdir = common.vsub(segment.q, segment.p);
                const snorm: [3]f32 = .{ -sdir[2], 0, sdir[0] };
                // If the velocity is pointing towards the segment, no
                // collision.
                if (common.vdot2d(snorm, vcand) < 0.0) continue;
                // Else immediate collision.
                ht_min = 0.0;
            } else {
                ht_min = intersectRaySegment(
                    position,
                    vcand,
                    segment.p,
                    segment.q,
                ) orelse continue;
            }

            // Avoid less when facing walls.
            ht_min *= 2.0;

            // The closest obstacle is somewhere ahead of us, keep track of
            // the nearest obstacle.
            if (ht_min < t_min) {
                t_min = ht_min;
                if (t_min < t_threshold) return penalty_min;
            }
        }

        // Normalize the side bias, to prevent it dominating too much.
        if (side_count != 0) side /= @as(f32, @floatFromInt(side_count));

        const spen = params.weight_side * side;
        const tpen = params.weight_time_of_impact *
            (1.0 / (0.1 + t_min * self.horizon_time_inverse));

        const penalty = vpen + vcpen + spen + tpen;

        // Store the different penalties for debug viewing.
        if (debug) |samples| samples.addSample(vcand, cs, penalty, vpen, vcpen, spen, tpen);

        return penalty;
    }

    /// Picks the velocity for an agent at `position` of `radius`, moving at
    /// `velocity` and wanting `desired_velocity`, from a square grid of
    /// `params.grid_size` candidates a side. Candidates faster than
    /// `velocity_max` by over half a grid step are skipped. Writes the
    /// candidate with the lowest penalty to `nvel`, zero when none was
    /// scored, and returns how many were scored. `debug`, when given, is
    /// emptied and then takes each candidate scored to the end rather than
    /// given up early.
    pub fn sampleVelocityGrid(
        self: *ObstacleAvoidanceQuery,
        position: [3]f32,
        radius: f32,
        velocity_max: f32,
        velocity: [3]f32,
        desired_velocity: [3]f32,
        nvel: *[3]f32,
        params: *const ObstacleAvoidanceParams,
        debug: ?*ObstacleAvoidanceDebug,
    ) i32 {
        self.prepare(position, desired_velocity);

        self.params = params.*;
        self.horizon_time_inverse = 1.0 / self.params.horizon_time;
        self.velocity_max = velocity_max;
        self.velocity_max_inverse =
            if (velocity_max > 0) 1.0 / velocity_max else std.math.floatMax(f32);

        nvel.* = .{ 0, 0, 0 };

        if (debug) |samples| samples.reset();

        const grid_size: i32 = self.params.grid_size;
        const step_count: f32 = @floatFromInt(grid_size - 1);
        const cvx = desired_velocity[0] * self.params.vel_bias;
        const cvz = desired_velocity[2] * self.params.vel_bias;
        const cs = velocity_max * 2 * (1 - self.params.vel_bias) / step_count;
        const half = step_count * cs * 0.5;

        var penalty_min: f32 = std.math.floatMax(f32);
        var ns: i32 = 0;

        var y: i32 = 0;
        while (y < grid_size) : (y += 1) {
            var x: i32 = 0;
            while (x < grid_size) : (x += 1) {
                const vcand: [3]f32 = .{
                    cvx + @as(f32, @floatFromInt(x)) * cs - half,
                    0,
                    cvz + @as(f32, @floatFromInt(y)) * cs - half,
                };

                const speed_sqr = common.sqr(f32, vcand[0]) + common.sqr(f32, vcand[2]);
                if (speed_sqr > common.sqr(f32, velocity_max + cs / 2)) continue;

                const penalty = self.processSample(
                    vcand,
                    cs,
                    position,
                    radius,
                    velocity,
                    desired_velocity,
                    penalty_min,
                    debug,
                );
                ns += 1;
                if (penalty < penalty_min) {
                    penalty_min = penalty;
                    nvel.* = vcand;
                }
            }
        }

        return ns;
    }

    /// Picks the velocity as `sampleVelocityGrid` does, from a pattern of
    /// `params.adaptive_rings` rings of `params.adaptive_divisions`
    /// candidates each plus the centre, aligned to the desired velocity.
    /// The pattern is tried `params.adaptive_depth` times, each time at
    /// half the size around the best candidate of the time before. Writes
    /// the last best candidate to `nvel`, zero when the last pattern scored
    /// none or the depth is 0, and returns how many candidates were scored.
    ///
    /// With an even `params.adaptive_divisions`, each ring's last candidate
    /// is the pattern's centre instead of the point opposite the ring's
    /// first.
    pub fn sampleVelocityAdaptive(
        self: *ObstacleAvoidanceQuery,
        position: [3]f32,
        radius: f32,
        velocity_max: f32,
        velocity: [3]f32,
        desired_velocity: [3]f32,
        nvel: *[3]f32,
        params: *const ObstacleAvoidanceParams,
        debug: ?*ObstacleAvoidanceDebug,
    ) i32 {
        self.prepare(position, desired_velocity);

        self.params = params.*;
        self.horizon_time_inverse = 1.0 / self.params.horizon_time;
        self.velocity_max = velocity_max;
        self.velocity_max_inverse =
            if (velocity_max > 0) 1.0 / velocity_max else std.math.floatMax(f32);

        nvel.* = .{ 0, 0, 0 };

        if (debug) |samples| samples.reset();

        // Build the sampling pattern aligned to the desired velocity: x and
        // z offsets in units of the pattern's radius. It starts zeroed, for
        // the entry an even division count skips.
        var pattern: [pattern_len][2]f32 = @splat(.{ 0, 0 });
        var pattern_count: usize = 0;

        const division_count: i32 = self.params.adaptive_divisions;
        const ring_count: i32 = self.params.adaptive_rings;
        const depth: i32 = self.params.adaptive_depth;

        const nd = common.clamp(i32, division_count, 1, pattern_division_count_max);
        const nr = common.clamp(i32, ring_count, 1, pattern_ring_count_max);
        const da: f32 = (1.0 / @as(f32, @floatFromInt(nd))) * pi * 2;
        const ca = @cos(da);
        const sa = @sin(da);

        // The desired direction, and the same rotated by half a division.
        var ddir: [2][3]f32 = undefined;
        ddir[0] = normalize2d(desired_velocity);
        ddir[1] = rotate2d(ddir[0], da * 0.5);

        // Always add a sample at zero.
        pattern[pattern_count] = .{ 0, 0 };
        pattern_count += 1;

        var j: i32 = 0;
        while (j < nr) : (j += 1) {
            // Odd rings start half a division round from even ones.
            const r = @as(f32, @floatFromInt(nr - j)) / @as(f32, @floatFromInt(nr));
            const start = ddir[@intCast(@rem(j, 2))];
            pattern[pattern_count] = .{ start[0] * r, start[2] * r };
            var last1 = pattern_count;
            var last2 = last1;
            pattern_count += 1;

            var i: i32 = 1;
            while (i < nd - 1) : (i += 2) {
                // The next point on the "right" (rotate clockwise).
                pattern[pattern_count] = .{
                    pattern[last1][0] * ca + pattern[last1][1] * sa,
                    -pattern[last1][0] * sa + pattern[last1][1] * ca,
                };
                // The next point on the "left" (rotate counter-clockwise).
                pattern[pattern_count + 1] = .{
                    pattern[last2][0] * ca - pattern[last2][1] * sa,
                    pattern[last2][0] * sa + pattern[last2][1] * ca,
                };

                last1 = pattern_count;
                last2 = last1 + 1;
                pattern_count += 2;
            }

            if ((nd & 1) == 0) {
                // The last point on the left goes one entry past the next
                // free one, which is counted in its place and stays zero.
                // The next ring's first point overwrites the point written
                // here, and after the last ring nothing reads it.
                pattern[pattern_count + 1] = .{
                    pattern[last2][0] * ca - pattern[last2][1] * sa,
                    pattern[last2][0] * sa + pattern[last2][1] * ca,
                };
                pattern_count += 1;
            }
        }

        // Start sampling.
        var cr = velocity_max * (1.0 - self.params.vel_bias);
        var res: [3]f32 = .{
            desired_velocity[0] * self.params.vel_bias,
            0,
            desired_velocity[2] * self.params.vel_bias,
        };
        var ns: i32 = 0;

        var k: i32 = 0;
        while (k < depth) : (k += 1) {
            var penalty_min: f32 = std.math.floatMax(f32);
            var bvel: [3]f32 = .{ 0, 0, 0 };

            for (pattern[0..pattern_count]) |offset| {
                const vcand: [3]f32 = .{
                    res[0] + offset[0] * cr,
                    0,
                    res[2] + offset[1] * cr,
                };

                const speed_sqr = common.sqr(f32, vcand[0]) + common.sqr(f32, vcand[2]);
                if (speed_sqr > common.sqr(f32, velocity_max + 0.001)) continue;

                const penalty = self.processSample(
                    vcand,
                    cr / 10,
                    position,
                    radius,
                    velocity,
                    desired_velocity,
                    penalty_min,
                    debug,
                );
                ns += 1;
                if (penalty < penalty_min) {
                    penalty_min = penalty;
                    bvel = vcand;
                }
            }

            res = bvel;

            cr *= 0.5;
        }

        nvel.* = res;

        return ns;
    }

    /// The number of circles added since the last `reset`.
    pub fn getObstacleCircleCount(self: *const ObstacleAvoidanceQuery) i32 {
        return self.circle_count;
    }

    /// Circle `i`, which is below `getObstacleCircleCount`. The pointer is
    /// into the query's memory: `deinit` ends it, and `reset` followed by
    /// `addCircle`, and each sampling call, change what it shows.
    pub fn getObstacleCircle(self: *const ObstacleAvoidanceQuery, i: i32) *const ObstacleCircle {
        return &self.circles[@intCast(i)];
    }

    /// The number of segments added since the last `reset`.
    pub fn getObstacleSegmentCount(self: *const ObstacleAvoidanceQuery) i32 {
        return self.segment_count;
    }

    /// Segment `i`, which is below `getObstacleSegmentCount`. The pointer
    /// is into the query's memory: `deinit` ends it, and `reset` followed
    /// by `addSegment`, and each sampling call, change what it shows.
    pub fn getObstacleSegment(
        self: *const ObstacleAvoidanceQuery,
        i: i32,
    ) *const ObstacleSegment {
        return &self.segments[@intCast(i)];
    }
};
