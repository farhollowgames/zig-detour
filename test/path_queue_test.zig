//! Differential tests of the path queue: `PathQueue` against the original's
//! `dtPathQueue`, driven with the same calls over the same navigation mesh.
//! After every call both must return the same value, give every request
//! they handed out the same status, and hand over the same path, entry for
//! entry, with the rest of the caller's buffer left alone.

const std = @import("std");
const detour = @import("detour");
const support = @import("support.zig");
const reference = support.reference;
const reference_crowd = support.reference_crowd;

const testing = std.testing;

const PathQueue = detour.PathQueue;
const PathQueueRef = detour.path_queue.PathQueueRef;
const PolyRef = detour.PolyRef;
const QueryFilter = detour.QueryFilter;

const path_queue_invalid = detour.path_queue.path_queue_invalid;

/// What a result buffer holds where nothing was written.
const ref_sentinel: PolyRef = std.math.maxInt(PolyRef) - 0x55;
const count_sentinel: i32 = -55;
const port_count_sentinel: usize = std.math.maxInt(usize) - 55;

/// The longest buffer a test reads a path into.
const result_capacity = 300;

/// A queue on each side over the same world, and the filters requests
/// name: the default one, and one that keeps out polygons of flag 2.
const QueuePair = struct {
    gpa: std.mem.Allocator,
    world: *support.World,
    port: PathQueue,
    original: *reference_crowd.PathQueue,
    filters: [2]QueryFilter,
    original_filters: [2]*reference.QueryFilter,
    /// A query of the original's over the world, to find start and end
    /// polygons for requests.
    finder: *reference.NavMeshQuery,
    /// Every handle the two queues handed out, oldest first.
    handles: std.ArrayList(PathQueueRef),

    fn init(self: *QueuePair, gpa: std.mem.Allocator, world: *support.World) !void {
        const original = reference_crowd.ref_path_queue_create() orelse return error.OutOfMemory;
        errdefer reference_crowd.ref_path_queue_destroy(original);
        const default_filter = reference.ref_filter_create() orelse return error.OutOfMemory;
        errdefer reference.ref_filter_destroy(default_filter);
        const narrow_filter = reference.ref_filter_create() orelse return error.OutOfMemory;
        errdefer reference.ref_filter_destroy(narrow_filter);
        reference.ref_filter_set_exclude_flags(narrow_filter, 2);
        const finder = reference.ref_query_alloc() orelse return error.OutOfMemory;
        errdefer reference.ref_query_free(finder);
        try testing.expect(reference.ref_status_succeeded(
            reference.ref_query_init(finder, world.reference_nav, 2048),
        ));
        self.* = .{
            .gpa = gpa,
            .world = world,
            .port = .empty,
            .original = original,
            .filters = .{ .{}, .{ .exclude_flags = 2 } },
            .original_filters = .{ default_filter, narrow_filter },
            .finder = finder,
            .handles = .empty,
        };
    }

    fn deinit(self: *QueuePair) void {
        self.port.deinit(self.gpa);
        reference_crowd.ref_path_queue_destroy(self.original);
        for (self.original_filters) |filter| reference.ref_filter_destroy(filter);
        reference.ref_query_free(self.finder);
        self.handles.deinit(self.gpa);
        self.* = undefined;
    }

    /// Readies both queues; both must say the same.
    fn initQueues(self: *QueuePair, path_size_max: i32, node_count_max: i32) !bool {
        const expected = reference_crowd.ref_path_queue_init(
            self.original,
            path_size_max,
            node_count_max,
            self.world.reference_nav,
        );
        const actual = self.port.init(self.gpa, path_size_max, node_count_max, &self.world.nav);
        try testing.expectEqual(expected, actual);
        if (actual) try self.expectSameQueries();
        return actual;
    }

    /// Asks both queues for a path; both must hand out the same handle.
    fn request(self: *QueuePair, task: Task) !PathQueueRef {
        const expected = reference_crowd.ref_path_queue_request(
            self.original,
            task.start_ref,
            task.end_ref,
            &task.start_pos,
            &task.end_pos,
            self.original_filters[task.filter],
        );
        const actual = self.port.request(
            task.start_ref,
            task.end_ref,
            task.start_pos,
            task.end_pos,
            &self.filters[task.filter],
        );
        try testing.expectEqual(expected, actual);
        if (actual != path_queue_invalid) try self.handles.append(self.gpa, actual);
        return actual;
    }

    fn update(self: *QueuePair, iteration_count_max: i32) void {
        reference_crowd.ref_path_queue_update(self.original, iteration_count_max);
        self.port.update(iteration_count_max);
    }

    /// Takes the result of `handle` from both, into buffers of `capacity`,
    /// and compares the status, the count and every entry of the buffers.
    fn takeResult(self: *QueuePair, handle: PathQueueRef, capacity: usize) !detour.Status {
        var expected_path: [result_capacity]PolyRef = @splat(ref_sentinel);
        var actual_path: [result_capacity]PolyRef = @splat(ref_sentinel);
        var expected_count: i32 = count_sentinel;
        var actual_count: usize = port_count_sentinel;
        const expected = reference_crowd.ref_path_queue_get_path_result(
            self.original,
            handle,
            &expected_path,
            &expected_count,
            @intCast(capacity),
        );
        const actual = self.port.getPathResult(handle, actual_path[0..capacity], &actual_count);
        try support.expectStatus(expected, actual);
        if (expected_count == count_sentinel) {
            try testing.expectEqual(port_count_sentinel, actual_count);
        } else {
            try testing.expectEqual(@as(usize, @intCast(expected_count)), actual_count);
        }
        try testing.expectEqualSlices(PolyRef, &expected_path, &actual_path);
        return actual;
    }

    /// Fails unless both queues give every handle they handed out, and the
    /// next one past it, the same status.
    fn expectSameStatuses(self: *const QueuePair) !void {
        const issued = self.handles.items;
        const next = if (issued.len == 0) 1 else issued[issued.len - 1] +% 1;
        for (issued) |handle| try self.expectSameStatus(handle);
        try self.expectSameStatus(next);
        try self.expectSameStatus(next +% 1000);
    }

    fn expectSameStatus(self: *const QueuePair, handle: PathQueueRef) !void {
        errdefer std.debug.print("  status of request {d}\n", .{handle});
        try support.expectStatus(
            reference_crowd.ref_path_queue_get_request_status(self.original, handle),
            self.port.getRequestStatus(handle),
        );
    }

    /// Fails unless both queues search with a query attached to their
    /// side's mesh, and the two queries find the same polygon.
    fn expectSameQueries(self: *QueuePair) !void {
        const original_query = reference_crowd.ref_path_queue_get_nav_query(self.original) orelse
            return error.TestUnexpectedResult;
        const port_query = self.port.getNavQuery();
        try testing.expect(
            reference.ref_query_get_attached_nav_mesh(original_query) == self.world.reference_nav,
        );
        try testing.expect(port_query.getAttachedNavMesh() == &self.world.nav);

        const center = self.world.level.mesh.triangleCenter(self.world.level.ground_triangles[0]);
        const half_extents: [3]f32 = .{ 2, 4, 2 };
        var expected_ref: PolyRef = 0;
        var expected_point: [3]f32 = .{ 0, 0, 0 };
        const expected = reference.ref_query_find_nearest_poly(
            original_query,
            &center,
            &half_extents,
            self.original_filters[0],
            &expected_ref,
            &expected_point,
        );
        var actual_ref: PolyRef = 0;
        var actual_point: [3]f32 = .{ 0, 0, 0 };
        const actual = port_query.findNearestPoly(
            center,
            half_extents,
            &self.filters[0],
            &actual_ref,
            &actual_point,
            null,
        );
        try support.expectStatus(expected, actual);
        try testing.expectEqual(expected_ref, actual_ref);
        try support.expectPoint(expected_point, actual_point);
    }

    /// A request between two random places on the ground, found with the
    /// original's query; now and then a reference that names nothing, or
    /// the same polygon at both ends.
    fn randomTask(self: *QueuePair, random: std.Random) Task {
        var task: Task = .{
            .start_ref = 0,
            .end_ref = 0,
            .start_pos = self.world.level.groundPoint(random, 1.0),
            .end_pos = self.world.level.groundPoint(random, 1.0),
            .filter = random.uintLessThan(usize, 2),
        };
        const half_extents: [3]f32 = .{ 2, 4, 2 };
        _ = reference.ref_query_find_nearest_poly(
            self.finder,
            &task.start_pos,
            &half_extents,
            self.original_filters[0],
            &task.start_ref,
            &task.start_pos,
        );
        _ = reference.ref_query_find_nearest_poly(
            self.finder,
            &task.end_pos,
            &half_extents,
            self.original_filters[0],
            &task.end_ref,
            &task.end_pos,
        );
        switch (random.uintLessThan(u8, 16)) {
            0 => task.start_ref = 0,
            1 => task.end_ref = std.math.maxInt(PolyRef) - 7,
            2 => {
                task.end_ref = task.start_ref;
                task.end_pos = task.start_pos;
            },
            else => {},
        }
        return task;
    }
};

/// What one request asks for.
const Task = struct {
    start_ref: PolyRef,
    end_ref: PolyRef,
    start_pos: [3]f32,
    end_pos: [3]f32,
    /// Which of the pair's filters.
    filter: usize,
};

fn initWorld(world: *support.World, gpa: std.mem.Allocator) !void {
    try world.init(gpa, support.nav_test_obj, support.tiled_config, .{
        .off_mesh_connection_count = 12,
        .vary_polygons = true,
        .seed = 71,
    });
}

/// Updates both queues until neither has a request in progress, at most
/// `update_count_max` times, comparing statuses after each.
fn runUntilSettled(pair: *QueuePair, iteration_count_max: i32, update_count_max: usize) !void {
    for (0..update_count_max) |_| {
        pair.update(iteration_count_max);
        try pair.expectSameStatuses();
        var busy = false;
        for (pair.handles.items) |handle| {
            const status = pair.port.getRequestStatus(handle);
            if (status.in_progress or @as(u32, @bitCast(status)) == 0) busy = true;
        }
        if (!busy) return;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

test "init: sizes both sides accept and refuse, and a second init of a used queue" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try initWorld(&world, gpa);
    defer world.deinit();
    var pair: QueuePair = undefined;
    try pair.init(gpa, &world);
    defer pair.deinit();
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(72));
    const random = prng.random();

    // The original needs at least four nodes, or its node table has no bucket.
    for ([_][2]i32{ .{ 1, 4 }, .{ 4, 16 }, .{ 256, 4096 }, .{ 32, 2048 }, .{ -1, 64 } }) |size| {
        errdefer std.debug.print("  path size {d}, nodes {d}\n", .{ size[0], size[1] });
        const ready = try pair.initQueues(size[0], size[1]);
        // Requests from before are dropped, and handles count on. An init
        // that fails for want of memory frees the entries it reached.
        try pair.expectSameStatuses();
        if (!ready) continue;
        for (0..3) |_| _ = try pair.request(pair.randomTask(random));
        try runUntilSettled(&pair, 50, 200);
    }
}

test "request: a full queue refuses, and freed entries take new requests" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try initWorld(&world, gpa);
    defer world.deinit();
    var pair: QueuePair = undefined;
    try pair.init(gpa, &world);
    defer pair.deinit();
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(73));
    const random = prng.random();
    try testing.expect(try pair.initQueues(64, 512));

    for (0..PathQueue.request_count_max) |_| {
        try testing.expect(try pair.request(pair.randomTask(random)) != path_queue_invalid);
    }
    try testing.expectEqual(path_queue_invalid, try pair.request(pair.randomTask(random)));
    try pair.expectSameStatuses();

    // Read two results, and the entries they free take two more requests.
    try runUntilSettled(&pair, 1000, 100);
    const issued = pair.handles.items;
    _ = try pair.takeResult(issued[0], result_capacity);
    _ = try pair.takeResult(issued[3], 2);
    try testing.expect(try pair.request(pair.randomTask(random)) != path_queue_invalid);
    try testing.expect(try pair.request(pair.randomTask(random)) != path_queue_invalid);
    try testing.expectEqual(path_queue_invalid, try pair.request(pair.randomTask(random)));
    try pair.expectSameStatuses();
}

test "update: every iteration budget, and results left unread expire" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try initWorld(&world, gpa);
    defer world.deinit();
    var pair: QueuePair = undefined;
    try pair.init(gpa, &world);
    defer pair.deinit();
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(74));
    const random = prng.random();
    try testing.expect(try pair.initQueues(128, 1024));

    for ([_]i32{ 0, 1, 10, 100, std.math.maxInt(i32), -5 }) |budget| {
        errdefer std.debug.print("  budget {d}\n", .{budget});
        for (0..5) |_| _ = try pair.request(pair.randomTask(random));
        for (0..12) |_| {
            pair.update(budget);
            try pair.expectSameStatuses();
        }
        // Finished requests nobody read have expired by now; those still
        // searching are read as they are.
        for (pair.handles.items) |handle| {
            const status = pair.port.getRequestStatus(handle);
            if (!status.failed) _ = try pair.takeResult(handle, result_capacity);
        }
        try pair.expectSameStatuses();
    }
}

test "getPathResult: buffers shorter, as long as and longer than the path" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try initWorld(&world, gpa);
    defer world.deinit();
    var pair: QueuePair = undefined;
    try pair.init(gpa, &world);
    defer pair.deinit();
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(75));
    const random = prng.random();

    // A path size of 6 cuts long paths short; a pool of 24 nodes runs out.
    for ([_][2]i32{ .{ 256, 2048 }, .{ 6, 2048 }, .{ 256, 24 } }) |size| {
        errdefer std.debug.print("  path size {d}, nodes {d}\n", .{ size[0], size[1] });
        try testing.expect(try pair.initQueues(size[0], size[1]));
        var lengths_seen: usize = 0;
        for (0..40) |_| {
            const handle = try pair.request(pair.randomTask(random));
            if (handle == path_queue_invalid) continue;
            try runUntilSettled(&pair, 4096, 50);
            const status = pair.port.getRequestStatus(handle);
            if (status.failed) {
                _ = try pair.takeResult(handle, result_capacity);
                continue;
            }
            // The port's entry tells the path's length, to size the
            // buffers around it.
            const length = pair.port.queue[
                for (pair.port.queue, 0..) |q, i| {
                    if (q.ref == handle) break i;
                } else unreachable
            ].path_count;
            const n: usize = @intCast(length);
            const capacity = switch (random.uintLessThan(u8, 5)) {
                0 => 0,
                1 => n -| 1,
                2 => n,
                3 => n + 1,
                else => result_capacity,
            };
            const taken = try pair.takeResult(handle, capacity);
            try testing.expect(taken.succeeded);
            lengths_seen += n;
            // The handle is stale now on both sides.
            try pair.expectSameStatus(handle);
            _ = try pair.takeResult(handle, result_capacity);
        }
        try testing.expect(lengths_seen > 0);
    }
}

test "path queue: seeded runs of requests, updates, status reads and results" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try initWorld(&world, gpa);
    defer world.deinit();

    var partial_count: usize = 0;
    var refused_count: usize = 0;
    for ([_]u64{ 0x70617468_71, 0x70617468_72 }) |seed| {
        var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(seed));
        const random = prng.random();
        var pair: QueuePair = undefined;
        try pair.init(gpa, &world);
        defer pair.deinit();
        try testing.expect(try pair.initQueues(
            random.intRangeAtMost(i32, 8, 128),
            random.intRangeAtMost(i32, 32, 2048),
        ));

        for (0..1500) |step| {
            errdefer std.debug.print("  seed {d}, step {d}\n", .{ seed, step });
            switch (random.uintLessThan(u8, 10)) {
                0...3 => {
                    if (try pair.request(pair.randomTask(random)) == path_queue_invalid) {
                        refused_count += 1;
                    }
                },
                4...6 => {
                    const budgets = [_]i32{ 0, 1, 3, 10, 50, 100, 1000, std.math.maxInt(i32) };
                    pair.update(budgets[random.uintLessThan(usize, budgets.len)]);
                },
                7 => {
                    const issued = pair.handles.items;
                    const handle = if (issued.len > 0 and random.uintLessThan(u8, 4) != 0)
                        issued[issued.len - 1 - random.uintLessThan(usize, @min(issued.len, 10))]
                    else
                        random.int(PathQueueRef) | 1;
                    try pair.expectSameStatus(handle);
                },
                else => {
                    const issued = pair.handles.items;
                    if (issued.len == 0) continue;
                    const handle =
                        issued[issued.len - 1 - random.uintLessThan(usize, @min(issued.len, 10))];
                    const status = pair.port.getRequestStatus(handle);
                    if (status.partial_result) partial_count += 1;
                    _ = try pair.takeResult(handle, random.uintLessThan(usize, result_capacity));
                },
            }
            try pair.expectSameStatuses();
            // Old handles are stale on both sides; keep the list short.
            if (pair.handles.items.len > 40) {
                const items = pair.handles.items;
                std.mem.copyForwards(PathQueueRef, items[0..20], items[items.len - 20 ..]);
                pair.handles.shrinkRetainingCapacity(20);
            }
        }
        try pair.expectSameQueries();
    }
    try testing.expect(refused_count > 0);
    try testing.expect(partial_count > 0);
}
