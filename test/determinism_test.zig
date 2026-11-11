//! The port gives the same bytes on every run and from any number of
//! threads: queries with their own `NavMeshQuery` over one shared
//! `NavMesh`, the way the original documents its use from threads.

const std = @import("std");
const support = @import("support.zig");
const detour = support.detour;

const PolyRef = detour.PolyRef;

const point_count = 1001;
const result_count_max = 64;

/// Everything the queries of one run returned, as raw bytes in call order.
const Sink = struct {
    bytes: []u8,
    len: usize = 0,
    overflowed: bool = false,

    fn put(self: *Sink, value: anytype) void {
        self.putBytes(std.mem.asBytes(&value));
    }

    fn putSlice(self: *Sink, comptime T: type, items: []const T) void {
        self.putBytes(std.mem.sliceAsBytes(items));
    }

    fn putBytes(self: *Sink, raw: []const u8) void {
        if (raw.len > self.bytes.len - self.len) {
            self.overflowed = true;
            return;
        }
        @memcpy(self.bytes[self.len..][0..raw.len], raw);
        self.len += raw.len;
    }

    fn written(self: *const Sink) []const u8 {
        return self.bytes[0..self.len];
    }
};

fn statusBits(status: detour.Status) u32 {
    return @bitCast(status);
}

/// Runs every kind of query from each point to the next and records all
/// they return.
fn runQueries(query: *detour.NavMeshQuery, points: []const [3]f32, sink: *Sink) void {
    const filter: detour.QueryFilter = .{};
    const reach: [3]f32 = .{ 2.0, 4.0, 2.0 };

    for (points[0 .. points.len - 1], points[1..]) |start, end| {
        var start_ref: PolyRef = 0;
        var end_ref: PolyRef = 0;
        var from = start;
        var to = end;
        var over = false;
        const nearest = query.findNearestPoly(start, reach, &filter, &start_ref, &from, &over);
        sink.put(statusBits(nearest));
        sink.put(statusBits(query.findNearestPoly(end, reach, &filter, &end_ref, &to, null)));
        sink.put(start_ref);
        sink.put(end_ref);
        sink.put(from);
        sink.put(to);
        sink.put(@as(u8, @intFromBool(over)));
        if (start_ref == 0 or end_ref == 0) continue;

        var closest: [3]f32 = @splat(0.0);
        sink.put(statusBits(query.closestPointOnPoly(start_ref, end, &closest, &over)));
        sink.put(closest);
        var height: f32 = 0.0;
        sink.put(statusBits(query.getPolyHeight(start_ref, from, &height)));
        sink.put(height);

        var path: [result_count_max]PolyRef = @splat(0);
        var path_count: usize = 0;
        sink.put(statusBits(query.findPath(
            start_ref,
            end_ref,
            from,
            to,
            &filter,
            &path,
            &path_count,
        )));
        sink.putSlice(PolyRef, path[0..path_count]);

        var corners: [result_count_max][3]f32 = undefined;
        var corner_flags: [result_count_max]detour.StraightPathFlags = undefined;
        var corner_refs: [result_count_max]PolyRef = undefined;
        var corner_count: usize = 0;
        sink.put(statusBits(query.findStraightPath(
            from,
            to,
            path[0..path_count],
            &corners,
            &corner_flags,
            &corner_refs,
            &corner_count,
            .{ .all_crossings = true },
        )));
        sink.putSlice([3]f32, corners[0..corner_count]);
        sink.putSlice(detour.StraightPathFlags, corner_flags[0..corner_count]);
        sink.putSlice(PolyRef, corner_refs[0..corner_count]);

        sink.put(statusBits(query.initSlicedFindPath(start_ref, end_ref, from, to, &filter, .{
            .any_angle = true,
        })));
        var done_iterations: i32 = 0;
        var sliced = query.updateSlicedFindPath(7, &done_iterations);
        while (sliced.in_progress) sliced = query.updateSlicedFindPath(7, &done_iterations);
        sink.put(statusBits(sliced));
        sink.put(statusBits(query.finalizeSlicedFindPath(&path, &path_count)));
        sink.putSlice(PolyRef, path[0..path_count]);

        var moved: [3]f32 = @splat(0.0);
        var visited: [result_count_max]PolyRef = undefined;
        var visited_count: usize = 0;
        sink.put(statusBits(query.moveAlongSurface(
            start_ref,
            from,
            to,
            &filter,
            &moved,
            &visited,
            &visited_count,
        )));
        sink.put(moved);
        sink.putSlice(PolyRef, visited[0..visited_count]);

        var ray_path: [result_count_max]PolyRef = undefined;
        var hit: detour.RaycastHit = .{
            .t = 0.0,
            .hit_normal = @splat(0.0),
            .hit_edge_index = 0,
            .path = &ray_path,
            .path_count = 0,
            .path_cost = 0.0,
        };
        sink.put(statusBits(query.raycast(start_ref, from, to, &filter, .{
            .use_costs = true,
        }, &hit, 0)));
        sink.put(hit.t);
        sink.put(hit.hit_normal);
        sink.put(hit.hit_edge_index);
        sink.put(hit.path_cost);
        sink.putSlice(PolyRef, ray_path[0..hit.path_count]);

        var around_refs: [result_count_max]PolyRef = undefined;
        var around_parents: [result_count_max]PolyRef = undefined;
        var around_costs: [result_count_max]f32 = undefined;
        var around_count: usize = 0;
        sink.put(statusBits(query.findPolysAroundCircle(
            start_ref,
            from,
            8.0,
            &filter,
            &around_refs,
            &around_parents,
            &around_costs,
            &around_count,
            result_count_max,
        )));
        sink.putSlice(PolyRef, around_refs[0..around_count]);
        sink.putSlice(PolyRef, around_parents[0..around_count]);
        sink.putSlice(f32, around_costs[0..around_count]);

        sink.put(statusBits(query.findLocalNeighbourhood(
            start_ref,
            from,
            4.0,
            &filter,
            &around_refs,
            &around_parents,
            &around_count,
        )));
        sink.putSlice(PolyRef, around_refs[0..around_count]);
        sink.putSlice(PolyRef, around_parents[0..around_count]);

        var segments: [result_count_max][2][3]f32 = undefined;
        var segment_refs: [result_count_max]PolyRef = undefined;
        var segment_count: usize = 0;
        sink.put(statusBits(query.getPolyWallSegments(
            start_ref,
            &filter,
            &segments,
            &segment_refs,
            &segment_count,
        )));
        sink.putSlice([2][3]f32, segments[0..segment_count]);
        sink.putSlice(PolyRef, segment_refs[0..segment_count]);

        var wall_distance: f32 = 0.0;
        var wall_point: [3]f32 = @splat(0.0);
        var wall_normal: [3]f32 = @splat(0.0);
        sink.put(statusBits(query.findDistanceToWall(
            start_ref,
            from,
            10.0,
            &filter,
            &wall_distance,
            &wall_point,
            &wall_normal,
        )));
        sink.put(wall_distance);
        sink.put(wall_point);
        sink.put(wall_normal);
    }
}

/// One thread's query object and what its run wrote.
const Worker = struct {
    query: detour.NavMeshQuery,
    sink: Sink,

    fn init(self: *Worker, gpa: std.mem.Allocator, nav: *const detour.NavMesh) !void {
        self.query = .empty;
        if (self.query.init(gpa, nav, 512).failed) return error.OutOfMemory;
        errdefer self.query.deinit(gpa);
        self.sink = .{ .bytes = try gpa.alloc(u8, sink_size) };
    }

    fn deinit(self: *Worker, gpa: std.mem.Allocator) void {
        gpa.free(self.sink.bytes);
        self.query.deinit(gpa);
        self.* = undefined;
    }

    fn run(self: *Worker, points: []const [3]f32) void {
        self.sink.len = 0;
        runQueries(&self.query, points, &self.sink);
    }
};

/// Room for everything one run writes: each pair of points writes less
/// than 8 KiB.
const sink_size = point_count * 8 * 1024;

fn expectSameRun(expected: *const Sink, actual: *const Sink, thread_count: usize) !void {
    try std.testing.expect(!actual.overflowed);
    if (std.mem.eql(u8, expected.written(), actual.written())) return;
    const first = std.mem.findDiff(u8, expected.written(), actual.written()) orelse 0;
    std.debug.print("a run on {d} threads differs from the first run at byte {d}\n", .{
        thread_count,
        first,
    });
    return error.TestExpectedEqual;
}

test "queries give the same bytes on every run and on 1, 2 and many threads" {
    const gpa = std.testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, support.nav_test_obj, support.tiled_config, .{
        .off_mesh_connection_count = 8,
        .vary_polygons = true,
    });
    defer world.deinit();

    var prng: std.Random.DefaultPrng = .init(0x64657465726d);
    const random = prng.random();
    const points = try gpa.alloc([3]f32, point_count);
    defer gpa.free(points);
    for (points) |*point| point.* = world.level.groundPoint(random, 0.5);

    var first: Worker = undefined;
    try first.init(gpa, &world.nav);
    defer first.deinit(gpa);
    first.run(points);
    try std.testing.expect(!first.sink.overflowed);
    try std.testing.expect(first.sink.len > point_count * 64);

    // The same query object again, its node pool as the first run left it.
    {
        var again: Worker = undefined;
        try again.init(gpa, &world.nav);
        defer again.deinit(gpa);
        const kept = again.sink;
        again.sink = first.sink;
        again.sink.bytes = kept.bytes;
        again.sink.len = 0;
        runQueries(&first.query, points, &again.sink);
        try expectSameRun(&first.sink, &again.sink, 1);
    }

    const cpu_count = std.Thread.getCpuCount() catch 4;
    const thread_counts = [_]usize{ 1, 2, std.math.clamp(cpu_count, 3, 8) };
    for (thread_counts) |thread_count| {
        const workers = try gpa.alloc(Worker, thread_count);
        defer gpa.free(workers);
        var ready: usize = 0;
        defer for (workers[0..ready]) |*worker| worker.deinit(gpa);
        for (workers) |*worker| {
            try worker.init(gpa, &world.nav);
            ready += 1;
        }

        const threads = try gpa.alloc(std.Thread, thread_count);
        defer gpa.free(threads);
        var started: usize = 0;
        errdefer for (threads[0..started]) |thread| thread.join();
        for (threads, workers) |*thread, *worker| {
            thread.* = try std.Thread.spawn(.{}, Worker.run, .{ worker, points });
            started += 1;
        }
        for (threads) |thread| thread.join();
        started = 0;

        for (workers) |*worker| try expectSameRun(&first.sink, &worker.sink, thread_count);
    }
}
