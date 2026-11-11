//! Differential tests of the path corridor: the three corridor merges and
//! every `PathCorridor` method, run on both sides with the same input.
//!
//! A merge gets a buffer on each side, filled with the same sentinel and
//! then the same path, and the two buffers are compared whole. A corridor
//! on each side is driven by the same calls over the same world; after every
//! call the return value, the out-parameters and the whole corridor are
//! compared: count, first and last polygon, position, target and every path
//! entry a call has written, in use or not.
//!
//! The original leaves a new corridor's position and target unset, so the
//! two are compared from the first call that writes them. Its shortcut merge
//! reads and writes outside the buffer when more visited polygons go in
//! front of the path than the buffer holds, where the port stops at the
//! buffer's end. The tests stay where the original is defined: a shortcut
//! merge always has room for those polygons, and the two optimizing calls,
//! which put up to 32 polygons in front, run on corridors with room for 32.

const std = @import("std");
const detour = @import("detour");
const support = @import("support.zig");
const reference = support.reference;
const reference_crowd = support.reference_crowd;
const testing = std.testing;

const path_corridor = detour.path_corridor;
const PathCorridor = detour.PathCorridor;
const NavMeshQuery = detour.NavMeshQuery;
const QueryFilter = detour.QueryFilter;
const MeshTile = detour.MeshTile;
const Poly = detour.Poly;
const PolyRef = detour.PolyRef;
const TileRef = detour.TileRef;
const StraightPathFlags = detour.StraightPathFlags;

/// A reference no mesh holds: its salt matches no tile's.
const sentinel_ref: PolyRef = std.math.maxInt(PolyRef) - 0x1234;
/// A point no call writes.
const sentinel_point: [3]f32 = @splat(-98765.25);
/// Corner flags no call writes.
const sentinel_flag_bits: u8 = 0xee;
const sentinel_flag: StraightPathFlags = @bitCast(sentinel_flag_bits);

// ---------------------------------------------------------------------------
// The merges
// ---------------------------------------------------------------------------

/// Entries in the buffers the merge tests hand to both sides. The capacity a
/// call is told is anything up to it; the rest must keep its sentinel.
const merge_room = 96;
/// The longest path or visited list a merge fuzz case makes.
const merge_list_max = 40;
/// Cases each merge's fuzz test runs.
const merge_fuzz_case_count = 4000;

const Merge = enum {
    start_moved,
    end_moved,
    start_shortcut,
};

/// What a merge left on the port's side: the count it returned and its whole
/// buffer, sentinels included.
const Merged = struct {
    count: usize,
    buffer: [merge_room]PolyRef,
};

/// Runs `merge` on both sides and compares the returned count, the whole
/// buffer and the visited list, which neither side may change.
///
/// Both buffers start as sentinels with `initial` at the front, of which the
/// first `path_count` entries are the path and the rest what an older,
/// longer path left behind. Returns the port's result.
fn expectSameMerge(
    merge: Merge,
    initial: []const PolyRef,
    path_count: usize,
    capacity: usize,
    visited: []const PolyRef,
) !Merged {
    // The original reads `path_count` entries and writes up to `capacity`.
    std.debug.assert(path_count <= initial.len);
    std.debug.assert(initial.len <= capacity);
    std.debug.assert(capacity <= merge_room);
    std.debug.assert(visited.len <= merge_room);

    errdefer std.debug.print(
        "  merge {s}: buffer {any}, path count {d}, capacity {d}, visited {any}\n",
        .{ @tagName(merge), initial, path_count, capacity, visited },
    );

    var original_buffer: [merge_room]PolyRef = @splat(sentinel_ref);
    var port_buffer: [merge_room]PolyRef = @splat(sentinel_ref);
    @memcpy(original_buffer[0..initial.len], initial);
    @memcpy(port_buffer[0..initial.len], initial);

    var original_visited: [merge_room]PolyRef = @splat(sentinel_ref);
    var port_visited: [merge_room]PolyRef = @splat(sentinel_ref);
    @memcpy(original_visited[0..visited.len], visited);
    @memcpy(port_visited[0..visited.len], visited);

    const path_count_c: i32 = @intCast(path_count);
    const capacity_c: i32 = @intCast(capacity);
    const visited_count_c: i32 = @intCast(visited.len);
    const original_count: i32 = switch (merge) {
        .start_moved => reference_crowd.ref_merge_corridor_start_moved(
            &original_buffer,
            path_count_c,
            capacity_c,
            &original_visited,
            visited_count_c,
        ),
        .end_moved => reference_crowd.ref_merge_corridor_end_moved(
            &original_buffer,
            path_count_c,
            capacity_c,
            &original_visited,
            visited_count_c,
        ),
        .start_shortcut => reference_crowd.ref_merge_corridor_start_shortcut(
            &original_buffer,
            path_count_c,
            capacity_c,
            &original_visited,
            visited_count_c,
        ),
    };

    const port_path: []PolyRef = port_buffer[0..capacity];
    const port_seen: []const PolyRef = port_visited[0..visited.len];
    const port_count: usize = switch (merge) {
        .start_moved => path_corridor.mergeCorridorStartMoved(port_path, path_count, port_seen),
        .end_moved => path_corridor.mergeCorridorEndMoved(port_path, path_count, port_seen),
        .start_shortcut => path_corridor.mergeCorridorStartShortcut(
            port_path,
            path_count,
            port_seen,
        ),
    };

    try testing.expectEqual(@as(i64, original_count), @as(i64, @intCast(port_count)));
    try testing.expectEqualSlices(PolyRef, &original_buffer, &port_buffer);
    try testing.expectEqualSlices(PolyRef, &original_visited, &port_visited);
    try testing.expectEqualSlices(PolyRef, visited, port_seen);
    return .{ .count = port_count, .buffer = port_buffer };
}

/// Where a merge joins a path and a visited list.
const Splice = struct {
    path_index: usize,
    visited_index: usize,
};

/// The join `merge` looks for: the path entry nearest the path's end (its
/// start, for an end merge) that the visited list holds, with the first
/// visited entry equal to it. Null when the lists share nothing.
///
/// This and `mergedLength` only steer the tests' input, sizing buffers and
/// counting what the cases reached. Every expectation comes from the
/// original.
fn splicePoint(merge: Merge, path: []const PolyRef, visited: []const PolyRef) ?Splice {
    for (0..path.len) |step| {
        const path_index = switch (merge) {
            .start_moved, .start_shortcut => path.len - 1 - step,
            .end_moved => step,
        };
        if (std.mem.indexOfScalar(PolyRef, visited, path[path_index])) |visited_index| {
            return .{ .path_index = path_index, .visited_index = visited_index };
        }
    }
    return null;
}

/// How long `merge` makes the path when the buffer has room for all of it.
fn mergedLength(merge: Merge, path: []const PolyRef, visited: []const PolyRef) usize {
    const splice = splicePoint(merge, path, visited) orelse return path.len;
    const path_before = splice.path_index;
    const path_after = path.len - 1 - splice.path_index;
    const visited_before = splice.visited_index;
    const visited_after = visited.len - 1 - splice.visited_index;
    return switch (merge) {
        .start_moved => visited_after + 1 + path_after,
        .end_moved => path_before + 1 + visited_after,
        .start_shortcut => if (visited_before == 0) path.len else visited_before + 1 + path_after,
    };
}

/// `capacity`, raised where the original needs more. Its shortcut merge is
/// defined only while the visited polygons that go in front of the path fit
/// the buffer; the other two merges take any capacity from the path's length.
fn definedCapacity(
    merge: Merge,
    path: []const PolyRef,
    visited: []const PolyRef,
    capacity: usize,
) usize {
    if (merge != .start_shortcut) return capacity;
    const splice = splicePoint(merge, path, visited) orelse return capacity;
    return @max(capacity, splice.visited_index);
}

/// One section of the original's `dtMergeCorridorStartMoved` test: the
/// buffer it starts from, the call, and the count and buffer it expects.
const UpstreamCase = struct {
    name: []const u8,
    initial: []const PolyRef,
    path_count: usize,
    capacity: usize,
    visited: []const PolyRef,
    count: usize,
    expected: []const PolyRef,
};

const upstream_cases = [_]UpstreamCase{
    .{
        .name = "Should handle empty input",
        .initial = &.{},
        .path_count = 0,
        .capacity = 0,
        .visited = &.{},
        .count = 0,
        .expected = &.{},
    },
    .{
        .name = "Should handle empty visited",
        .initial = &.{1},
        .path_count = 1,
        .capacity = 1,
        .visited = &.{},
        .count = 1,
        .expected = &.{1},
    },
    .{
        .name = "Should handle empty path",
        .initial = &.{},
        .path_count = 0,
        .capacity = 0,
        .visited = &.{1},
        .count = 0,
        .expected = &.{},
    },
    .{
        .name = "Should strip visited points from path except last",
        .initial = &.{ 1, 2 },
        .path_count = 2,
        .capacity = 2,
        .visited = &.{ 1, 2 },
        .count = 1,
        .expected = &.{ 2, 2 },
    },
    .{
        .name = "Should add visited points not present in path in reverse order",
        .initial = &.{ 1, 2, 0 },
        .path_count = 2,
        .capacity = 3,
        .visited = &.{ 1, 2, 3, 4 },
        .count = 3,
        .expected = &.{ 4, 3, 2 },
    },
    .{
        .name = "Should add visited points not present in path up to the path capacity",
        .initial = &.{ 1, 2, 0 },
        .path_count = 2,
        .capacity = 3,
        .visited = &.{ 1, 2, 3, 4, 5 },
        .count = 3,
        .expected = &.{ 5, 4, 3 },
    },
    .{
        .name = "Should not change path if there is no intersection with visited",
        .initial = &.{ 1, 2 },
        .path_count = 2,
        .capacity = 2,
        .visited = &.{ 3, 4 },
        .count = 2,
        .expected = &.{ 1, 2 },
    },
    .{
        .name = "Should save unvisited path points",
        .initial = &.{ 1, 2, 0 },
        .path_count = 2,
        .capacity = 3,
        .visited = &.{ 1, 3 },
        .count = 3,
        .expected = &.{ 3, 1, 2 },
    },
    .{
        .name = "Should save unvisited path points up to the path capacity",
        .initial = &.{ 1, 2 },
        .path_count = 2,
        .capacity = 2,
        .visited = &.{ 1, 3 },
        .count = 2,
        .expected = &.{ 3, 1 },
    },
};

test "mergeCorridorStartMoved: the cases of the original's own test" {
    for (upstream_cases) |case| {
        errdefer std.debug.print("  in case \"{s}\"\n", .{case.name});

        const merged = try expectSameMerge(
            .start_moved,
            case.initial,
            case.path_count,
            case.capacity,
            case.visited,
        );
        // What the original's test expects of the original holds for both.
        try testing.expectEqual(case.count, merged.count);
        try testing.expectEqualSlices(PolyRef, case.expected, merged.buffer[0..case.capacity]);

        // The same lists through the two merges the original leaves untested.
        for ([_]Merge{ .end_moved, .start_shortcut }) |merge| {
            _ = try expectSameMerge(
                merge,
                case.initial,
                case.path_count,
                case.capacity,
                case.visited,
            );
        }
    }
}

/// A merge whose result is worked out by hand. The path fills `path`, and
/// `expected` is the buffer up to its capacity afterwards.
const HandCase = struct {
    merge: Merge,
    path: []const PolyRef,
    capacity: usize,
    visited: []const PolyRef,
    count: usize,
    expected: []const PolyRef,
};

const hand_cases = [_]HandCase{
    // The position walked off the path's start and on: the visited list
    // goes in front, last polygon first, and pushes the path's end out.
    .{
        .merge = .start_moved,
        .path = &.{ 1, 2 },
        .capacity = 2,
        .visited = &.{ 1, 3, 4, 5 },
        .count = 2,
        .expected = &.{ 5, 4 },
    },
    // The position walked along the path: the polygons behind it go.
    .{
        .merge = .start_moved,
        .path = &.{ 1, 2, 3, 4, 5 },
        .capacity = 5,
        .visited = &.{ 1, 2, 3 },
        .count = 3,
        .expected = &.{ 3, 4, 5, 4, 5 },
    },
    // A visited list that comes back to where it started.
    .{
        .merge = .start_moved,
        .path = &.{ 1, 2 },
        .capacity = 4,
        .visited = &.{ 1, 6, 1 },
        .count = 4,
        .expected = &.{ 1, 6, 1, 2 },
    },
    // Room to spare stays untouched.
    .{
        .merge = .start_moved,
        .path = &.{ 1, 2, 3 },
        .capacity = 6,
        .visited = &.{ 2, 7 },
        .count = 3,
        .expected = &.{ 7, 2, 3, sentinel_ref, sentinel_ref, sentinel_ref },
    },
    // The target walked on from the path's end.
    .{
        .merge = .end_moved,
        .path = &.{ 1, 2, 3 },
        .capacity = 5,
        .visited = &.{ 3, 4, 5 },
        .count = 5,
        .expected = &.{ 1, 2, 3, 4, 5 },
    },
    // One entry short for it.
    .{
        .merge = .end_moved,
        .path = &.{ 1, 2, 3 },
        .capacity = 4,
        .visited = &.{ 3, 4, 5 },
        .count = 4,
        .expected = &.{ 1, 2, 3, 4 },
    },
    // The target left the path in its middle.
    .{
        .merge = .end_moved,
        .path = &.{ 1, 2, 3 },
        .capacity = 3,
        .visited = &.{ 2, 9 },
        .count = 3,
        .expected = &.{ 1, 2, 9 },
    },
    // The target stayed in the last polygon.
    .{
        .merge = .end_moved,
        .path = &.{ 1, 2, 3 },
        .capacity = 3,
        .visited = &.{3},
        .count = 3,
        .expected = &.{ 1, 2, 3 },
    },
    .{
        .merge = .end_moved,
        .path = &.{ 1, 2, 3 },
        .capacity = 3,
        .visited = &.{ 7, 8 },
        .count = 3,
        .expected = &.{ 1, 2, 3 },
    },
    // The target walked back to the path's start: only the count shrinks.
    .{
        .merge = .end_moved,
        .path = &.{ 1, 2, 3 },
        .capacity = 3,
        .visited = &.{ 3, 2, 1 },
        .count = 1,
        .expected = &.{ 1, 2, 3 },
    },
    .{
        .merge = .end_moved,
        .path = &.{ 1, 2, 3 },
        .capacity = 6,
        .visited = &.{ 3, 4 },
        .count = 4,
        .expected = &.{ 1, 2, 3, 4, sentinel_ref, sentinel_ref },
    },
    // A shortcut that skips two polygons of the path.
    .{
        .merge = .start_shortcut,
        .path = &.{ 1, 2, 3, 4 },
        .capacity = 4,
        .visited = &.{ 1, 9, 4 },
        .count = 3,
        .expected = &.{ 1, 9, 4, 4 },
    },
    // Lists that meet only at the first visited polygon change nothing.
    .{
        .merge = .start_shortcut,
        .path = &.{ 1, 2, 3, 4 },
        .capacity = 4,
        .visited = &.{1},
        .count = 4,
        .expected = &.{ 1, 2, 3, 4 },
    },
    .{
        .merge = .start_shortcut,
        .path = &.{ 1, 2, 3, 4 },
        .capacity = 4,
        .visited = &.{ 7, 8 },
        .count = 4,
        .expected = &.{ 1, 2, 3, 4 },
    },
    // The polygons going in front fill the buffer exactly: nothing of the
    // path is kept, the most the original is defined for.
    .{
        .merge = .start_shortcut,
        .path = &.{ 1, 2, 3 },
        .capacity = 3,
        .visited = &.{ 5, 6, 7, 3 },
        .count = 3,
        .expected = &.{ 5, 6, 7 },
    },
    .{
        .merge = .start_shortcut,
        .path = &.{ 1, 2 },
        .capacity = 2,
        .visited = &.{ 8, 9, 2 },
        .count = 2,
        .expected = &.{ 8, 9 },
    },
};

/// Runs the hand-made cases of `merge` and checks them against what was
/// worked out by hand as well.
fn expectHandCases(merge: Merge) !void {
    for (hand_cases) |case| {
        if (case.merge != merge) continue;
        const merged = try expectSameMerge(
            merge,
            case.path,
            case.path.len,
            case.capacity,
            case.visited,
        );
        errdefer std.debug.print(
            "  hand-made case: path {any}, capacity {d}, visited {any}\n",
            .{ case.path, case.capacity, case.visited },
        );
        try testing.expectEqual(case.count, merged.count);
        try testing.expectEqualSlices(PolyRef, case.expected, merged.buffer[0..case.capacity]);
    }
}

/// How many lists `tinyList` tells apart: one empty, 3 of one reference, 9
/// of two and 27 of three.
const tiny_list_count = 1 + 3 + 9 + 27;

/// The `index`th list of at most three references from 1 to 3; every such
/// list comes up once as `index` runs to `tiny_list_count`.
fn tinyList(index: usize, storage: *[3]PolyRef) []const PolyRef {
    var rest = index;
    var len: usize = 0;
    var lists_of_len: usize = 1;
    while (rest >= lists_of_len) {
        rest -= lists_of_len;
        lists_of_len *= 3;
        len += 1;
    }
    for (storage[0..len]) |*ref| {
        ref.* = @intCast(1 + rest % 3);
        rest /= 3;
    }
    return storage[0..len];
}

/// Runs `merge` on every pair of tiny lists, at each capacity from the
/// path's length to three above it.
fn expectTinyMerges(merge: Merge) !void {
    var path_storage: [3]PolyRef = undefined;
    var visited_storage: [3]PolyRef = undefined;
    for (0..tiny_list_count) |path_index| {
        const path = tinyList(path_index, &path_storage);
        for (0..tiny_list_count) |visited_index| {
            const visited = tinyList(visited_index, &visited_storage);
            for (0..4) |spare| {
                const capacity = definedCapacity(merge, path, visited, path.len + spare);
                _ = try expectSameMerge(merge, path, path.len, capacity, visited);
            }
        }
    }
}

test "mergeCorridorStartMoved: hand-made lists and every pair of tiny lists" {
    try expectHandCases(.start_moved);
    try expectTinyMerges(.start_moved);
}

test "mergeCorridorEndMoved: hand-made lists and every pair of tiny lists" {
    try expectHandCases(.end_moved);
    try expectTinyMerges(.end_moved);
}

test "mergeCorridorStartShortcut: hand-made lists and every pair of tiny lists" {
    try expectHandCases(.start_shortcut);
    try expectTinyMerges(.start_shortcut);
}

/// How a merge fuzz case relates the visited list to the path.
const MergeShape = enum {
    /// Both lists drawn from the same few references: overlaps anywhere and
    /// repeats in either list.
    mixed,
    /// Nothing in common.
    apart,
    /// The visited list starts at the path's first polygon and leaves it.
    from_first,
    /// The visited list starts at the path's last polygon and leaves it.
    from_last,
    /// The visited list follows a stretch of the path, either way along it.
    along,
    /// The visited list leaves the path at one polygon and meets it again
    /// at another.
    rejoin,
};

const merge_shapes = [_]MergeShape{ .mixed, .apart, .from_first, .from_last, .along, .rejoin };

/// How a merge fuzz case sizes the buffer against the merged path.
const MergeFit = enum {
    /// As long as the path: whatever the merge adds pushes something out.
    exact,
    /// As long as the merged path.
    whole,
    /// Longer than the merged path.
    roomy,
    /// Anything from the path's length to the merged path's.
    between,
    /// One entry short of the merged path.
    one_short,
};

const merge_fits = [_]MergeFit{ .exact, .whole, .roomy, .between, .one_short };

/// The path and the visited list of one merge fuzz case.
const MergeLists = struct {
    path: [merge_list_max]PolyRef,
    path_count: usize,
    visited: [merge_list_max]PolyRef,
    visited_count: usize,
};

/// A list length: mostly anything up to the longest, now and then empty,
/// one entry or the longest.
fn randomListLength(random: std.Random) usize {
    return switch (random.uintLessThan(u32, 10)) {
        0 => 0,
        1 => 1,
        2 => merge_list_max,
        else => random.uintAtMost(usize, merge_list_max),
    };
}

/// Two lists of `shape`. A path's references are all different, as a
/// corridor's are, except in the mixed shape; references from 101 on are in
/// no path.
fn randomMergeLists(random: std.Random, shape: MergeShape) MergeLists {
    var lists: MergeLists = .{
        .path = @splat(0),
        .path_count = randomListLength(random),
        .visited = @splat(0),
        .visited_count = randomListLength(random),
    };
    for (&lists.path, 1..) |*ref, value| ref.* = @intCast(value);
    random.shuffle(PolyRef, &lists.path);
    for (&lists.visited, 101..) |*ref, value| ref.* = @intCast(value);

    const path: []PolyRef = lists.path[0..lists.path_count];
    const visited: []PolyRef = lists.visited[0..lists.visited_count];
    switch (shape) {
        .mixed => {
            const set_sizes = [_]PolyRef{ 2, 4, 8, 16 };
            const set_size = set_sizes[random.uintLessThan(usize, set_sizes.len)];
            for (path) |*ref| ref.* = 1 + random.uintLessThan(PolyRef, set_size);
            for (visited) |*ref| ref.* = 1 + random.uintLessThan(PolyRef, set_size);
        },
        .apart => {},
        .from_first => if (path.len > 0 and visited.len > 0) {
            visited[0] = path[0];
        },
        .from_last => if (path.len > 0 and visited.len > 0) {
            visited[0] = path[path.len - 1];
        },
        .along => if (path.len > 0 and visited.len > 0) {
            const stretch = 1 + random.uintLessThan(usize, @min(path.len, visited.len));
            const path_start = random.uintAtMost(usize, path.len - stretch);
            const visited_start = random.uintAtMost(usize, visited.len - stretch);
            const backward = random.boolean();
            for (0..stretch) |k| {
                const from = if (backward) path_start + stretch - 1 - k else path_start + k;
                visited[visited_start + k] = path[from];
            }
        },
        .rejoin => if (path.len > 1 and visited.len > 1) {
            const leave = random.uintLessThan(usize, path.len);
            var meet = random.uintLessThan(usize, path.len - 1);
            if (meet >= leave) meet += 1;
            visited[0] = path[leave];
            visited[1 + random.uintLessThan(usize, visited.len - 1)] = path[meet];
        },
    }
    return lists;
}

/// The capacity a fuzz case gives a path of `path_count` polygons that a
/// merge with room would make `merged` long.
fn fitCapacity(random: std.Random, fit: MergeFit, path_count: usize, merged: usize) usize {
    const whole = @max(path_count, merged);
    return switch (fit) {
        .exact => path_count,
        .whole => whole,
        .roomy => whole + random.intRangeAtMost(usize, 1, 8),
        .between => random.intRangeAtMost(usize, path_count, whole),
        .one_short => @max(path_count, whole -| 1),
    };
}

/// What a merge fuzz run met.
const MergeTally = struct {
    apart: u32 = 0,
    joined: u32 = 0,
    cut_short: u32 = 0,
    exact_fit: u32 = 0,
    spare_room: u32 = 0,
    stale_tail: u32 = 0,
};

/// Runs `merge` on `merge_fuzz_case_count` random pairs of lists, every
/// shape with every fit.
fn fuzzMerge(merge: Merge, seed: u64) !void {
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();
    var tally: MergeTally = .{};

    for (0..merge_fuzz_case_count) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });

        const shape = merge_shapes[case % merge_shapes.len];
        const fit = merge_fits[(case / merge_shapes.len) % merge_fits.len];
        const lists = randomMergeLists(random, shape);
        const path = lists.path[0..lists.path_count];
        const visited = lists.visited[0..lists.visited_count];

        const merged_length = mergedLength(merge, path, visited);
        const capacity = definedCapacity(
            merge,
            path,
            visited,
            fitCapacity(random, fit, path.len, merged_length),
        );

        // Half the cases have what an older, longer path left behind the
        // path, which a merge may move over but never reads as path.
        var initial: [merge_room]PolyRef = @splat(0);
        @memcpy(initial[0..path.len], path);
        var initial_len = path.len;
        if (random.boolean()) {
            initial_len = random.intRangeAtMost(usize, path.len, capacity);
            for (initial[path.len..initial_len], 201..) |*ref, value| ref.* = @intCast(value);
        }

        _ = try expectSameMerge(merge, initial[0..initial_len], path.len, capacity, visited);

        if (splicePoint(merge, path, visited) == null) tally.apart += 1 else tally.joined += 1;
        if (merged_length > capacity) {
            tally.cut_short += 1;
        } else if (merged_length == capacity) {
            tally.exact_fit += 1;
        } else {
            tally.spare_room += 1;
        }
        if (initial_len > path.len) tally.stale_tail += 1;
    }

    // The cases must have reached every kind of result.
    errdefer std.debug.print("  seed {d}: {any}\n", .{ seed, tally });
    try testing.expect(tally.apart > 0);
    try testing.expect(tally.joined > 0);
    try testing.expect(tally.cut_short > 0);
    try testing.expect(tally.exact_fit > 0);
    try testing.expect(tally.spare_room > 0);
    try testing.expect(tally.stale_tail > 0);
}

test "mergeCorridorStartMoved: random lists against the original" {
    try fuzzMerge(.start_moved, 0x51a7);
}

test "mergeCorridorEndMoved: random lists against the original" {
    try fuzzMerge(.end_moved, 0xe2d0);
}

test "mergeCorridorStartShortcut: random lists against the original" {
    try fuzzMerge(.start_shortcut, 0x5c07);
}

// ---------------------------------------------------------------------------
// Worlds
// ---------------------------------------------------------------------------

/// Nodes each query searches with.
const node_count = 2048;
/// The longest route a stage finds.
const route_capacity = 256;
/// Entries in the corner buffers handed to both sides.
const corner_room = 40;

const WorldCase = struct {
    name: []const u8,
    obj_text: []const u8,
    config: support.RecastConfig,
    options: support.LevelOptions,
};

const tiled_case: WorldCase = .{
    .name = "nav_test, tiled, off-mesh connections",
    .obj_text = support.nav_test_obj,
    .config = support.tiled_config,
    .options = .{ .off_mesh_connection_count = 64, .vary_polygons = true, .seed = 41 },
};

const solo_case: WorldCase = .{
    .name = "dungeon, one tile, off-mesh connections",
    .obj_text = support.dungeon_obj,
    .config = support.solo_config,
    .options = .{ .off_mesh_connection_count = 12, .vary_polygons = true, .seed = 42 },
};

/// A point on the mesh and the polygon it is on.
const Place = struct {
    ref: PolyRef,
    pos: [3]f32,
};

/// A polygon path between two places and the point it ends at.
const Route = struct {
    polys: [route_capacity]PolyRef,
    count: usize,
    target: [3]f32,

    fn slice(self: *const Route) []const PolyRef {
        return self.polys[0..self.count];
    }
};

/// An off-mesh connection whose two ends both link to ground polygons.
const Connection = struct {
    ref: PolyRef,
    start_ref: PolyRef,
    end_ref: PolyRef,
    start_pos: [3]f32,
    end_pos: [3]f32,
};

/// A world with a query and a filter on each side for the corridors under
/// test, and a scout: a second query on the original with a filter that
/// passes everything. The scout only prepares input, finding places and
/// routes that both sides are then given alike, so preparing input never
/// touches the two queries whose state the corridor calls leave behind.
const Stage = struct {
    gpa: std.mem.Allocator,
    world: *support.World,
    query: NavMeshQuery,
    reference_query: *reference.NavMeshQuery,
    filter: QueryFilter,
    reference_filter: *reference.QueryFilter,
    scout: *reference.NavMeshQuery,
    scout_filter: *reference.QueryFilter,
    /// The world's connections with both ends linked, in tile order.
    connections: std.ArrayList(Connection),

    /// Readies the stage in place and compares what the two sides' query
    /// `init` return. The world must outlive the stage and not move.
    fn init(self: *Stage, gpa: std.mem.Allocator, world: *support.World) !void {
        const reference_query = reference.ref_query_alloc() orelse return error.OutOfMemory;
        errdefer reference.ref_query_free(reference_query);
        const scout = reference.ref_query_alloc() orelse return error.OutOfMemory;
        errdefer reference.ref_query_free(scout);
        const reference_filter = reference.ref_filter_create() orelse return error.OutOfMemory;
        errdefer reference.ref_filter_destroy(reference_filter);
        const scout_filter = reference.ref_filter_create() orelse return error.OutOfMemory;
        errdefer reference.ref_filter_destroy(scout_filter);

        self.* = .{
            .gpa = gpa,
            .world = world,
            .query = .empty,
            .reference_query = reference_query,
            .filter = .{},
            .reference_filter = reference_filter,
            .scout = scout,
            .scout_filter = scout_filter,
            .connections = .empty,
        };
        errdefer self.query.deinit(gpa);
        try support.expectStatus(
            reference.ref_query_init(reference_query, world.reference_nav, node_count),
            self.query.init(gpa, &world.nav, node_count),
        );
        const scout_status = reference.ref_query_init(scout, world.reference_nav, node_count);
        if (reference.ref_status_failed(scout_status)) return error.ScoutInitFailed;

        errdefer self.connections.deinit(gpa);
        try self.collectConnections();
    }

    fn deinit(self: *Stage) void {
        self.connections.deinit(self.gpa);
        self.query.deinit(self.gpa);
        reference.ref_query_free(self.reference_query);
        reference.ref_query_free(self.scout);
        reference.ref_filter_destroy(self.reference_filter);
        reference.ref_filter_destroy(self.scout_filter);
    }

    /// Sets the polygon flags both sides' filter includes and excludes. The
    /// scout's filter keeps passing everything.
    fn setFilterFlags(self: *Stage, include: u16, exclude: u16) void {
        self.filter.include_flags = include;
        self.filter.exclude_flags = exclude;
        reference.ref_filter_set_include_flags(self.reference_filter, include);
        reference.ref_filter_set_exclude_flags(self.reference_filter, exclude);
    }

    /// Sets a polygon's flags in both meshes, which must agree on the
    /// status.
    fn setPolyFlags(self: *Stage, ref: PolyRef, flags: u16) !void {
        try support.expectStatus(
            reference.ref_nav_mesh_set_poly_flags(self.world.reference_nav, ref, flags),
            self.world.nav.setPolyFlags(ref, flags),
        );
    }

    /// A polygon's flags, which the two meshes must agree on.
    fn polyFlags(self: *const Stage, ref: PolyRef) !u16 {
        var original_flags: u16 = 0;
        var port_flags: u16 = 0;
        try support.expectStatus(
            reference.ref_nav_mesh_get_poly_flags(self.world.reference_nav, ref, &original_flags),
            self.world.nav.getPolyFlags(ref, &port_flags),
        );
        try testing.expectEqual(original_flags, port_flags);
        return port_flags;
    }

    /// The ground polygon nearest `point` and the point on it, or null when
    /// none is near.
    fn placeNear(self: *const Stage, point: [3]f32) ?Place {
        const half_extents: [3]f32 = .{ 2.0, 4.0, 2.0 };
        var ref: PolyRef = 0;
        var nearest: [3]f32 = point;
        const status = reference.ref_query_find_nearest_poly(
            self.scout,
            &point,
            &half_extents,
            self.scout_filter,
            &ref,
            &nearest,
        );
        if (reference.ref_status_failed(status) or ref == 0) return null;
        return .{ .ref = ref, .pos = nearest };
    }

    /// A random place on the world's ground.
    fn randomPlace(self: *const Stage, random: std.Random) !Place {
        for (0..64) |_| {
            const point = self.world.level.groundPoint(random, 0.5);
            if (self.placeNear(point)) |place| return place;
        }
        return error.NoGround;
    }

    /// The scout's path from one place to another, at most `count_max`
    /// polygons of it; null when the search fails. A path that stops short
    /// of `to` ends at the point of its last polygon nearest `to`.
    fn findRoute(self: *const Stage, from: Place, to: Place, count_max: usize) ?Route {
        var route: Route = .{ .polys = @splat(0), .count = 0, .target = to.pos };
        var count: i32 = 0;
        const status = reference.ref_query_find_path(
            self.scout,
            from.ref,
            to.ref,
            &from.pos,
            &to.pos,
            self.scout_filter,
            &route.polys,
            &count,
            @intCast(@min(count_max, route_capacity)),
        );
        if (reference.ref_status_failed(status) or count <= 0) return null;
        route.count = @intCast(count);

        const last = route.polys[route.count - 1];
        if (last != to.ref) {
            var closest: [3]f32 = to.pos;
            const closest_status = reference.ref_query_closest_point_on_poly(
                self.scout,
                last,
                &to.pos,
                &closest,
                null,
            );
            if (reference.ref_status_failed(closest_status)) return null;
            route.target = closest;
        }
        return route;
    }

    /// A route between two random places with at least `count_min` polygons
    /// and at most `count_max`, with the place it starts from.
    fn randomRoute(
        self: *const Stage,
        random: std.Random,
        count_min: usize,
        count_max: usize,
    ) !StartedRoute {
        for (0..256) |_| {
            const from = try self.randomPlace(random);
            const to = try self.randomPlace(random);
            const route = self.findRoute(from, to, count_max) orelse continue;
            if (route.count < count_min) continue;
            return .{ .from = from, .route = route };
        }
        return error.NoRoute;
    }

    /// Fills `connections` from the port's mesh, which the mesh tests hold
    /// equal to the original's.
    fn collectConnections(self: *Stage) !void {
        const nav = &self.world.nav;
        var tile_index: i32 = 0;
        while (tile_index < nav.getMaxTiles()) : (tile_index += 1) {
            const tile = nav.getTile(tile_index);
            const header = tile.header orelse continue;
            const base = nav.getPolyRefBase(tile);
            var k: i32 = 0;
            while (k < header.off_mesh_connection_count) : (k += 1) {
                const poly_index: usize = @intCast(header.off_mesh_base + k);
                const poly = &tile.polys[poly_index];
                var connection: Connection = .{
                    .ref = base | @as(PolyRef, @intCast(poly_index)),
                    .start_ref = 0,
                    .end_ref = 0,
                    .start_pos = tile.verts[poly.verts[0]],
                    .end_pos = tile.verts[poly.verts[1]],
                };
                var link_index = poly.first_link;
                while (link_index != detour.nav_mesh.null_link) {
                    const link = &tile.links[link_index];
                    if (link.edge == 0) connection.start_ref = link.ref;
                    if (link.edge == 1) connection.end_ref = link.ref;
                    link_index = link.next;
                }
                if (connection.start_ref == 0 or connection.end_ref == 0) continue;
                try self.connections.append(self.gpa, connection);
            }
        }
    }
};

/// A route and the place its first polygon was found from.
const StartedRoute = struct {
    from: Place,
    route: Route,
};

/// Builds the world of `case` in place.
fn initWorld(world: *support.World, gpa: std.mem.Allocator, case: WorldCase) !void {
    try world.init(gpa, case.obj_text, case.config, case.options);
}

/// Whether both sides refuse a position that is not finite. A build that
/// trades that check for speed makes such input undefined, and the cases
/// that feed it are left out there.
fn rejectsNonFinite() bool {
    const nan = std.math.nan(f32);
    return !reference.ref_math_isfinite(nan) and !detour.common.isFinite(nan);
}

// ---------------------------------------------------------------------------
// Both sides of a corridor
// ---------------------------------------------------------------------------

/// Polygons either optimizing call may put in front of a corridor's path.
/// The original is defined only when the corridor has room for them.
const shortcut_room = 32;

/// The corners `findCorners` gave the port, checked equal to the original's
/// over the whole buffers.
const Corners = struct {
    count: usize,
    verts: [corner_room][3]f32,
    flags: [corner_room]StraightPathFlags,
    polys: [corner_room]PolyRef,
};

/// What `moveOverOffmeshConnection` gave the port, checked equal to the
/// original's. An out-parameter the call left alone keeps its sentinel.
const Crossing = struct {
    crossed: bool,
    refs: [2]PolyRef,
    start_pos: [3]f32,
    end_pos: [3]f32,
};

/// A corridor on each side, driven by the same calls. Each method makes
/// one call on both sides, compares what it returns and writes, and then
/// compares the two corridors whole.
const Pair = struct {
    port: PathCorridor,
    original: *reference_crowd.PathCorridor,
    /// The capacity `init` gave both corridors; zero before.
    capacity: usize,
    /// How many entries of the path buffer some call has written. The two
    /// buffers are compared that far, in use or not; beyond it the
    /// original's memory is whatever its allocator handed out.
    written: usize,
    /// The original leaves a new corridor's position and target unset, so
    /// each is compared from the first call that writes it.
    pos_known: bool,
    target_known: bool,

    fn create() !Pair {
        const original = reference_crowd.ref_corridor_create() orelse return error.OutOfMemory;
        return .{
            .port = .empty,
            .original = original,
            .capacity = 0,
            .written = 0,
            .pos_known = false,
            .target_known = false,
        };
    }

    fn deinit(self: *Pair, gpa: std.mem.Allocator) void {
        self.port.deinit(gpa);
        reference_crowd.ref_corridor_destroy(self.original);
    }

    /// A corridor on each side with room for `capacity` polygons, reset to
    /// `from` and then given `route`.
    fn start(gpa: std.mem.Allocator, capacity: usize, from: Place, route: *const Route) !Pair {
        var pair = try Pair.create();
        errdefer pair.deinit(gpa);
        try testing.expect(try pair.init(gpa, @intCast(capacity)));
        try pair.reset(from.ref, from.pos);
        try pair.setCorridor(route.target, route.slice());
        return pair;
    }

    /// The polygons in the port's corridor.
    fn count(self: *const Pair) usize {
        return @intCast(self.port.getPathCount());
    }

    /// Compares everything the two corridors hold, through the getters and
    /// in the path buffers.
    fn expectSame(self: *const Pair) !void {
        const original = self.original;
        const port = &self.port;
        try testing.expectEqual(
            reference_crowd.ref_corridor_get_path_count(original),
            port.getPathCount(),
        );
        try testing.expectEqual(
            reference_crowd.ref_corridor_get_first_poly(original),
            port.getFirstPoly(),
        );
        try testing.expectEqual(
            reference_crowd.ref_corridor_get_last_poly(original),
            port.getLastPoly(),
        );
        if (self.pos_known) {
            try support.expectPoint(
                reference_crowd.ref_corridor_get_pos(original).*,
                port.getPos(),
            );
        }
        if (self.target_known) {
            try support.expectPoint(
                reference_crowd.ref_corridor_get_target(original).*,
                port.getTarget(),
            );
        }

        const original_path = reference_crowd.ref_corridor_get_path(original);
        try testing.expectEqual(original_path == null, port.path == null);
        const in_use = port.getPath();
        try testing.expectEqual(self.count(), in_use.len);
        if (original_path) |original_entries| {
            const port_entries = port.path orelse return error.TestExpectedEqual;
            try testing.expectEqual(self.capacity, port_entries.len);
            try testing.expectEqualSlices(PolyRef, original_entries[0..in_use.len], in_use);
            try testing.expectEqualSlices(
                PolyRef,
                original_entries[0..self.written],
                port_entries[0..self.written],
            );
        }
    }

    /// Notes how far the path buffer is written now and compares the two
    /// corridors.
    fn settle(self: *Pair) !void {
        self.written = @max(self.written, self.count());
        try self.expectSame();
    }

    fn init(self: *Pair, gpa: std.mem.Allocator, capacity: i32) !bool {
        errdefer std.debug.print("  in init with capacity {d}\n", .{capacity});
        const expected = reference_crowd.ref_corridor_init(self.original, capacity);
        const actual = self.port.init(gpa, capacity);
        try testing.expectEqual(expected, actual);
        if (actual) self.capacity = @intCast(capacity);
        try self.settle();
        return actual;
    }

    fn reset(self: *Pair, ref: PolyRef, pos: [3]f32) !void {
        errdefer std.debug.print("  in reset to {d} at {any}\n", .{ ref, pos });
        // The original writes the first entry of the buffer.
        std.debug.assert(self.capacity >= 1);
        reference_crowd.ref_corridor_reset(self.original, ref, &pos);
        self.port.reset(ref, pos);
        self.pos_known = true;
        self.target_known = true;
        try self.settle();
    }

    fn setCorridor(self: *Pair, target: [3]f32, polys: []const PolyRef) !void {
        errdefer std.debug.print("  in setCorridor with {d} polygons\n", .{polys.len});
        // The original copies all of `polys` into the buffer.
        std.debug.assert(polys.len > 0 and polys.len <= self.capacity);
        reference_crowd.ref_corridor_set_corridor(
            self.original,
            &target,
            polys.ptr,
            @intCast(polys.len),
        );
        self.port.setCorridor(target, polys);
        self.target_known = true;
        try self.settle();
    }

    /// `findCorners` into buffers of `room` entries. All `corner_room`
    /// entries are compared, so a write past `room` shows.
    fn findCorners(self: *Pair, stage: *Stage, room: usize) !Corners {
        errdefer std.debug.print("  in findCorners with room for {d}\n", .{room});
        std.debug.assert(room <= corner_room);
        // The original reads the first polygon, the position and the target.
        std.debug.assert(self.count() != 0 and self.pos_known and self.target_known);

        var original_verts: [corner_room][3]f32 = @splat(sentinel_point);
        var original_flags: [corner_room]u8 = @splat(sentinel_flag_bits);
        var original_polys: [corner_room]PolyRef = @splat(sentinel_ref);
        var corners: Corners = .{
            .count = 0,
            .verts = @splat(sentinel_point),
            .flags = @splat(sentinel_flag),
            .polys = @splat(sentinel_ref),
        };

        const original_points: [][3]f32 = &original_verts;
        const expected = reference_crowd.ref_corridor_find_corners(
            self.original,
            @ptrCast(original_points.ptr),
            &original_flags,
            &original_polys,
            @intCast(room),
            stage.reference_query,
            stage.reference_filter,
        );
        const actual = self.port.findCorners(
            corners.verts[0..room],
            corners.flags[0..room],
            corners.polys[0..room],
            &stage.query,
            &stage.filter,
        );
        try testing.expectEqual(expected, actual);
        for (original_verts, corners.verts) |original_vert, port_vert| {
            try support.expectPoint(original_vert, port_vert);
        }
        for (original_flags, corners.flags) |original_bits, port_flag| {
            try testing.expectEqual(original_bits, @as(u8, @bitCast(port_flag)));
        }
        try testing.expectEqualSlices(PolyRef, &original_polys, &corners.polys);
        try self.settle();

        corners.count = @intCast(actual);
        return corners;
    }

    fn optimizePathVisibility(self: *Pair, stage: *Stage, next: [3]f32, range: f32) !void {
        errdefer std.debug.print(
            "  in optimizePathVisibility toward {any}, range {d}\n",
            .{ next, range },
        );
        // The original reads the first polygon and the position, and its
        // shortcut needs room unless the path is one polygon, which no
        // shortcut changes.
        std.debug.assert(self.count() != 0 and self.pos_known);
        std.debug.assert(self.capacity >= shortcut_room or self.count() == 1);
        reference_crowd.ref_corridor_optimize_path_visibility(
            self.original,
            &next,
            range,
            stage.reference_query,
            stage.reference_filter,
        );
        self.port.optimizePathVisibility(next, range, &stage.query, &stage.filter);
        try self.settle();
    }

    fn optimizePathTopology(self: *Pair, stage: *Stage) !bool {
        errdefer std.debug.print("  in optimizePathTopology\n", .{});
        // A path of fewer than three polygons is turned down before anything
        // is read. A longer one has its position and target read, and its
        // shortcut needs room.
        std.debug.assert(self.count() < 3 or (self.pos_known and self.target_known));
        std.debug.assert(self.count() < 3 or self.capacity >= shortcut_room);
        const expected = reference_crowd.ref_corridor_optimize_path_topology(
            self.original,
            stage.reference_query,
            stage.reference_filter,
        );
        const actual = self.port.optimizePathTopology(&stage.query, &stage.filter);
        try testing.expectEqual(expected, actual);
        try self.settle();
        return actual;
    }

    fn moveOverOffmeshConnection(self: *Pair, stage: *Stage, ref: PolyRef) !Crossing {
        errdefer std.debug.print("  in moveOverOffmeshConnection over {d}\n", .{ref});
        std.debug.assert(self.count() != 0);

        var original_refs: [2]PolyRef = @splat(sentinel_ref);
        var original_start = sentinel_point;
        var original_end = sentinel_point;
        var crossing: Crossing = .{
            .crossed = false,
            .refs = @splat(sentinel_ref),
            .start_pos = sentinel_point,
            .end_pos = sentinel_point,
        };

        const expected = reference_crowd.ref_corridor_move_over_offmesh_connection(
            self.original,
            ref,
            &original_refs,
            &original_start,
            &original_end,
            stage.reference_query,
        );
        crossing.crossed = self.port.moveOverOffmeshConnection(
            ref,
            &crossing.refs,
            &crossing.start_pos,
            &crossing.end_pos,
            &stage.query,
        );
        try testing.expectEqual(expected, crossing.crossed);
        try testing.expectEqualSlices(PolyRef, &original_refs, &crossing.refs);
        try support.expectPoint(original_start, crossing.start_pos);
        try support.expectPoint(original_end, crossing.end_pos);
        // A crossing puts the position at the connection's far end.
        if (crossing.crossed) self.pos_known = true;
        try self.settle();
        return crossing;
    }

    fn fixPathStart(self: *Pair, safe_ref: PolyRef, safe_pos: [3]f32) !void {
        errdefer std.debug.print("  in fixPathStart to {d} at {any}\n", .{ safe_ref, safe_pos });
        // The original writes the first two entries, and the third too when
        // the path has one or two polygons.
        const path_count = self.count();
        const touched: usize = if (path_count == 1 or path_count == 2) 3 else 2;
        std.debug.assert(touched <= self.capacity);

        const expected = reference_crowd.ref_corridor_fix_path_start(
            self.original,
            safe_ref,
            &safe_pos,
        );
        const actual = self.port.fixPathStart(safe_ref, safe_pos);
        try testing.expectEqual(expected, actual);
        try testing.expect(actual);
        self.pos_known = true;
        self.written = @max(self.written, touched);
        try self.settle();
    }

    fn trimInvalidPath(self: *Pair, stage: *Stage, safe_ref: PolyRef, safe_pos: [3]f32) !void {
        errdefer std.debug.print("  in trimInvalidPath to {d} at {any}\n", .{ safe_ref, safe_pos });
        // An empty path is left alone before anything is read. Otherwise
        // the original may write the first entry and reads the target; the
        // position it writes only when the first polygon is bad, which
        // these tests reach with the position known.
        std.debug.assert(self.count() == 0 or (self.pos_known and self.target_known));

        const expected = reference_crowd.ref_corridor_trim_invalid_path(
            self.original,
            safe_ref,
            &safe_pos,
            stage.reference_query,
            stage.reference_filter,
        );
        const actual = self.port.trimInvalidPath(safe_ref, safe_pos, &stage.query, &stage.filter);
        try testing.expectEqual(expected, actual);
        try testing.expect(actual);
        try self.settle();
    }

    fn isValid(self: *Pair, stage: *Stage, look_ahead: i32) !bool {
        errdefer std.debug.print("  in isValid looking {d} ahead\n", .{look_ahead});
        const expected = reference_crowd.ref_corridor_is_valid(
            self.original,
            look_ahead,
            stage.reference_query,
            stage.reference_filter,
        );
        const actual = self.port.isValid(look_ahead, &stage.query, &stage.filter);
        try testing.expectEqual(expected, actual);
        try self.settle();
        return actual;
    }

    fn movePosition(self: *Pair, stage: *Stage, new_pos: [3]f32) !bool {
        errdefer std.debug.print("  in movePosition to {any}\n", .{new_pos});
        // The original reads the first polygon and the position.
        std.debug.assert(self.count() != 0 and self.pos_known);
        const expected = reference_crowd.ref_corridor_move_position(
            self.original,
            &new_pos,
            stage.reference_query,
            stage.reference_filter,
        );
        const actual = self.port.movePosition(new_pos, &stage.query, &stage.filter);
        try testing.expectEqual(expected, actual);
        try self.settle();
        return actual;
    }

    fn moveTargetPosition(self: *Pair, stage: *Stage, new_pos: [3]f32) !bool {
        errdefer std.debug.print("  in moveTargetPosition to {any}\n", .{new_pos});
        // The original reads the last polygon and the target.
        std.debug.assert(self.count() != 0 and self.target_known);
        const expected = reference_crowd.ref_corridor_move_target_position(
            self.original,
            &new_pos,
            stage.reference_query,
            stage.reference_filter,
        );
        const actual = self.port.moveTargetPosition(new_pos, &stage.query, &stage.filter);
        try testing.expectEqual(expected, actual);
        try self.settle();
        return actual;
    }
};

// ---------------------------------------------------------------------------
// Corridors without a mesh
// ---------------------------------------------------------------------------

/// An allocator for the original that has no memory left.
fn noMemory(_: usize, _: reference.AllocHint) callconv(.c) ?*anyopaque {
    return null;
}

test "PathCorridor.init: a capacity below zero, no memory left, and a second try" {
    const gpa = testing.allocator;
    var pair = try Pair.create();
    defer pair.deinit(gpa);

    // A new corridor has no buffer and no polygons.
    try pair.settle();
    try testing.expectEqual(0, pair.count());
    try testing.expectEqual(@as(PolyRef, 0), pair.port.getFirstPoly());
    try testing.expectEqual(@as(PolyRef, 0), pair.port.getLastPoly());
    try testing.expect(pair.port.path == null);

    // A capacity below zero asks for more memory than there is.
    for ([_]i32{ -1, -2, -4096, std.math.minInt(i32) }) |capacity| {
        try testing.expect(!try pair.init(gpa, capacity));
        try testing.expect(pair.port.path == null);
    }

    // Neither side gets memory from its allocator. The original's allocator
    // is shared by everything upstream, so it is put back at once.
    {
        reference.ref_alloc_set_custom(&noMemory, null);
        defer reference.ref_alloc_set_custom(null, null);
        try testing.expect(!try pair.init(testing.failing_allocator, 8));
    }
    try testing.expect(pair.port.path == null);

    // A failed init leaves the corridor as new, and the next init works.
    try testing.expect(try pair.init(gpa, 8));
    try testing.expectEqual(0, pair.count());
    try testing.expectEqual(@as(PolyRef, 0), pair.port.getFirstPoly());
    try testing.expectEqual(@as(PolyRef, 0), pair.port.getLastPoly());
    try testing.expectEqual(8, pair.capacity);

    try pair.reset(41, .{ 1.0, 2.0, 3.0 });
    try testing.expectEqualSlices(PolyRef, &.{41}, pair.port.getPath());
}

test "reset, setCorridor, fixPathStart and the getters: paths of made-up polygons" {
    const gpa = testing.allocator;
    const safe_ref: PolyRef = 0x77;
    const start: [3]f32 = .{ 4.0, -1.5, 0.25 };
    const target: [3]f32 = .{ -8.0, 2.5, 16.0 };
    const safe_pos: [3]f32 = .{ 1.5, -2.25, 3.0 };

    // No call here looks a polygon up, so any references do.
    var refs: [64]PolyRef = undefined;
    for (&refs, 0..) |*ref, i| ref.* = @intCast(0x400 + i);

    for ([_]usize{ 1, 2, 3, 4, 5, 8, 64 }) |capacity| {
        errdefer std.debug.print("  capacity {d}\n", .{capacity});
        var pair = try Pair.create();
        defer pair.deinit(gpa);
        try testing.expect(try pair.init(gpa, @intCast(capacity)));
        try testing.expectEqual(0, pair.count());

        // Fixing the start of an empty corridor writes two entries that
        // stay out of use.
        if (capacity >= 2) {
            try pair.fixPathStart(safe_ref, safe_pos);
            try testing.expectEqual(0, pair.count());
            try testing.expectEqual(@as(PolyRef, 0), pair.port.getFirstPoly());
            try testing.expectEqual(@as(PolyRef, 0), pair.port.getLastPoly());
            try support.expectPoint(safe_pos, pair.port.getPos());
        }

        for (1..capacity + 1) |count| {
            errdefer std.debug.print("  {d} polygons\n", .{count});
            try pair.reset(refs[0], start);
            try testing.expectEqualSlices(PolyRef, refs[0..1], pair.port.getPath());
            try support.expectPoint(start, pair.port.getPos());
            try support.expectPoint(start, pair.port.getTarget());

            try pair.setCorridor(target, refs[0..count]);
            try testing.expectEqualSlices(PolyRef, refs[0..count], pair.port.getPath());
            try testing.expectEqual(refs[0], pair.port.getFirstPoly());
            try testing.expectEqual(refs[count - 1], pair.port.getLastPoly());
            try support.expectPoint(start, pair.port.getPos());
            try support.expectPoint(target, pair.port.getTarget());

            // One or two polygons become three: the safe one, a gap and
            // the old last one. A longer path has its first two replaced.
            const fixed_count: usize = @max(count, 3);
            if (fixed_count > capacity) continue;
            try pair.fixPathStart(safe_ref, safe_pos);
            const fixed = pair.port.getPath();
            try testing.expectEqual(fixed_count, fixed.len);
            try testing.expectEqual(safe_ref, fixed[0]);
            try testing.expectEqual(@as(PolyRef, 0), fixed[1]);
            try testing.expectEqual(refs[count - 1], fixed[fixed.len - 1]);
            if (count > 3) try testing.expectEqualSlices(PolyRef, refs[2..count], fixed[2..]);
            try support.expectPoint(safe_pos, pair.port.getPos());
            try support.expectPoint(target, pair.port.getTarget());
        }

        // Shorter and shorter paths over the longest one: what is left
        // behind them in the buffer stays equal too.
        var count = capacity;
        while (count > 0) : (count -= 1) {
            try pair.setCorridor(start, refs[capacity - count ..][0..count]);
            try testing.expectEqual(refs[capacity - count], pair.port.getFirstPoly());
            try testing.expectEqual(refs[capacity - 1], pair.port.getLastPoly());
        }
        try pair.reset(safe_ref, target);
        try testing.expectEqualSlices(PolyRef, &.{safe_ref}, pair.port.getPath());
    }
}

// ---------------------------------------------------------------------------
// Corridors on a mesh
// ---------------------------------------------------------------------------

/// The path of `first` followed by the path of `second`, which starts on
/// the polygon `first` ends on: a corridor with a detour in it. Null when
/// the two do not join or the whole does not fit in `count_max` polygons.
fn joinRoutes(first: *const Route, second: *const Route, count_max: usize) ?Route {
    if (first.count == 0 or second.count == 0) return null;
    if (first.polys[first.count - 1] != second.polys[0]) return null;
    const count = first.count + second.count - 1;
    if (count > @min(count_max, route_capacity)) return null;
    var joined: Route = .{ .polys = @splat(0), .count = count, .target = second.target };
    @memcpy(joined.polys[0..first.count], first.slice());
    @memcpy(joined.polys[first.count..count], second.polys[1..second.count]);
    return joined;
}

/// A route from `from` to the start of `connection`, over the connection
/// and onto the polygon at its far end, where the target is. Null when no
/// route reaches the connection's start.
fn routeOver(stage: *const Stage, from: Place, connection: Connection) ?Route {
    const entry: Place = .{ .ref = connection.start_ref, .pos = connection.start_pos };
    var route = stage.findRoute(from, entry, route_capacity - 2) orelse return null;
    if (route.polys[route.count - 1] != connection.start_ref) return null;
    route.polys[route.count] = connection.ref;
    route.polys[route.count + 1] = connection.end_ref;
    route.count += 2;
    route.target = connection.end_pos;
    return route;
}

test "findCorners: every buffer size on routes through the tiled world" {
    const gpa = testing.allocator;
    const case = tiled_case;
    var world: support.World = undefined;
    try initWorld(&world, gpa, case);
    defer world.deinit();
    var stage: Stage = undefined;
    try stage.init(gpa, &world);
    defer stage.deinit();
    errdefer std.debug.print("  in world \"{s}\"\n", .{case.name});

    const seed: u64 = 0xc02e;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();
    const rooms = [_]usize{ 0, 1, 2, 3, 4, 8, 32, corner_room };
    const few_rooms = [_]usize{ 0, 1, 2, corner_room };

    var corner_total: usize = 0;
    var bent: usize = 0;
    for (0..1000) |case_index| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case_index });
        const started = try stage.randomRoute(random, 1, route_capacity);
        const from = started.from;
        var pair = try Pair.start(gpa, route_capacity, from, &started.route);
        defer pair.deinit(gpa);

        for (rooms) |room| {
            const corners = try pair.findCorners(&stage, room);
            try testing.expect(corners.count <= room);
            corner_total += corners.count;
            if (corners.count >= 2) bent += 1;
        }

        // A fixed start leaves a gap after the first polygon, where the
        // corners stop. A first polygon that is no polygon gives none.
        const safe_ref: PolyRef = switch (case_index % 3) {
            0 => from.ref,
            1 => 0,
            else => sentinel_ref,
        };
        try pair.fixPathStart(safe_ref, from.pos);
        for (few_rooms) |room| {
            const corners = try pair.findCorners(&stage, room);
            try testing.expect(corners.count <= room);
            if (safe_ref != from.ref) try testing.expectEqual(0, corners.count);
        }

        // One polygon with the target at the position: nothing to walk to.
        try pair.reset(from.ref, from.pos);
        for (few_rooms) |room| {
            const corners = try pair.findCorners(&stage, room);
            try testing.expectEqual(0, corners.count);
        }
    }
    try testing.expect(corner_total > 0);
    try testing.expect(bent > 0);

    // Routes that end by crossing an off-mesh connection: the corners stop
    // at the connection's start.
    var over_connection: usize = 0;
    for (stage.connections.items, 0..) |connection, connection_index| {
        errdefer std.debug.print("  seed {d}, connection {d}\n", .{ seed, connection_index });
        const from = try stage.randomPlace(random);
        const route = routeOver(&stage, from, connection) orelse continue;
        var pair = try Pair.start(gpa, route_capacity, from, &route);
        defer pair.deinit(gpa);
        for (rooms) |room| {
            const corners = try pair.findCorners(&stage, room);
            if (corners.count == 0) continue;
            if (corners.flags[corners.count - 1].off_mesh_connection) over_connection += 1;
        }
    }
    if (stage.connections.items.len != 0) try testing.expect(over_connection > 0);
}

/// The nearest ground to a point up to `reach` from `origin` along each
/// level axis, or null when there is none.
fn placeAround(stage: *const Stage, random: std.Random, origin: [3]f32, reach: f32) ?Place {
    return stage.placeNear(.{
        origin[0] + (random.float(f32) * 2.0 - 1.0) * reach,
        origin[1],
        origin[2] + (random.float(f32) * 2.0 - 1.0) * reach,
    });
}

/// A route of at most `count_max` polygons from a random place out to a
/// place nearby and on to one nearer the start: a corridor with a detour
/// that a straight look or a fresh search may cut.
fn randomDetour(stage: *const Stage, random: std.Random, count_max: usize) !StartedRoute {
    for (0..256) |_| {
        const from = try stage.randomPlace(random);
        const middle = placeAround(stage, random, from.pos, 8.0) orelse continue;
        const to = placeAround(stage, random, from.pos, 4.0) orelse continue;
        const out = stage.findRoute(from, middle, count_max) orelse continue;
        if (out.polys[out.count - 1] != middle.ref) continue;
        const back = stage.findRoute(middle, to, count_max) orelse continue;
        const joined = joinRoutes(&out, &back, count_max) orelse continue;
        return .{ .from = from, .route = joined };
    }
    return error.NoRoute;
}

/// A copy of the port's path, to tell whether a call rewrote it.
const PathCopy = struct {
    polys: [route_capacity]PolyRef,
    count: usize,

    fn of(pair: *const Pair) PathCopy {
        const path = pair.port.getPath();
        var copy: PathCopy = .{ .polys = @splat(0), .count = path.len };
        @memcpy(copy.polys[0..path.len], path);
        return copy;
    }

    fn matches(self: *const PathCopy, pair: *const Pair) bool {
        return std.mem.eql(PolyRef, self.polys[0..self.count], pair.port.getPath());
    }
};

/// `point` with one coordinate infinite or not a number.
fn oddPoint(random: std.Random, point: [3]f32) [3]f32 {
    const odd_values = [_]f32{ std.math.inf(f32), -std.math.inf(f32), std.math.nan(f32) };
    var odd = point;
    odd[random.uintLessThan(usize, 3)] = odd_values[random.uintLessThan(usize, odd_values.len)];
    return odd;
}

test "optimizePathVisibility: looks toward corners, the target and stray points" {
    const gpa = testing.allocator;
    const case = tiled_case;
    var world: support.World = undefined;
    try initWorld(&world, gpa, case);
    defer world.deinit();
    var stage: Stage = undefined;
    try stage.init(gpa, &world);
    defer stage.deinit();
    errdefer std.debug.print("  in world \"{s}\"\n", .{case.name});

    const seed: u64 = 0x71b5;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();
    // A range at or under the distance to the point looks exactly that far,
    // a longer one overshoots, and zero leaves the look without a
    // direction.
    const ranges = [_]f32{ 0.001, 0.5, -1.0, 6.0, 30.0, 0.0, -0.0, 1.0e6, 1.0e30 };
    const odd_ranges = [_]f32{ std.math.inf(f32), -std.math.inf(f32), std.math.nan(f32) };
    const odd = rejectsNonFinite();

    var shortened: usize = 0;
    for (0..1500) |case_index| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case_index });
        // The smallest capacity the original's shortcut is defined for,
        // where the merged path may lose its tail, and a roomy one.
        const capacity: usize = if (case_index % 4 == 0) shortcut_room else route_capacity;
        const started = if (case_index % 2 == 0)
            try randomDetour(&stage, random, capacity)
        else
            try stage.randomRoute(random, 1, capacity);
        var pair = try Pair.start(gpa, capacity, started.from, &started.route);
        defer pair.deinit(gpa);

        for (0..3) |_| {
            const corners = try pair.findCorners(&stage, corner_room);
            const pos = pair.port.getPos();
            const target = pair.port.getTarget();
            const next: [3]f32 = switch (random.uintLessThan(u8, 14)) {
                // The corner a crowd agent looks toward.
                0...3 => if (corners.count != 0)
                    corners.verts[@min(1, corners.count - 1)]
                else
                    target,
                4...6 => target,
                7, 8 => if (corners.count != 0)
                    corners.verts[random.uintLessThan(usize, corners.count)]
                else
                    pos,
                9 => (try stage.randomPlace(random)).pos,
                10 => world.level.groundPoint(random, 3.0),
                // Too near, level with the position, to be worth a look.
                11 => .{ pos[0] + 0.005, pos[1] + 3.0, pos[2] - 0.005 },
                12 => pos,
                else => if (odd) oddPoint(random, pos) else target,
            };
            var range = ranges[random.uintLessThan(usize, ranges.len)];
            if (odd and random.uintLessThan(u8, 16) == 0) {
                range = odd_ranges[random.uintLessThan(usize, odd_ranges.len)];
            }

            const before: PathCopy = .of(&pair);
            try pair.optimizePathVisibility(&stage, next, range);
            if (!before.matches(&pair)) shortened += 1;
        }
    }
    try testing.expect(shortened > 0);
}

test "optimizePathTopology: a short search along detours, plain routes and broken starts" {
    const gpa = testing.allocator;
    const case = tiled_case;
    var world: support.World = undefined;
    try initWorld(&world, gpa, case);
    defer world.deinit();
    var stage: Stage = undefined;
    try stage.init(gpa, &world);
    defer stage.deinit();
    errdefer std.debug.print("  in world \"{s}\"\n", .{case.name});

    const seed: u64 = 0x7090;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var rewritten: usize = 0;
    var shortened: usize = 0;
    var refused: usize = 0;
    for (0..1500) |case_index| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case_index });
        const capacity: usize = if (case_index % 4 == 0) shortcut_room else route_capacity;
        const started = if (case_index % 3 != 0)
            try randomDetour(&stage, random, capacity)
        else
            try stage.randomRoute(random, 1, capacity);
        const from = started.from;
        var pair = try Pair.start(gpa, capacity, from, &started.route);
        defer pair.deinit(gpa);

        const before: PathCopy = .of(&pair);
        const optimized = try pair.optimizePathTopology(&stage);
        // A path under three polygons is turned down, and a call that
        // turns the path down leaves it alone.
        if (before.count < 3) try testing.expect(!optimized);
        if (!optimized) try testing.expect(before.matches(&pair));
        if (optimized) rewritten += 1 else refused += 1;
        if (!before.matches(&pair)) shortened += 1;

        // Again, on the path the first call left.
        _ = try pair.optimizePathTopology(&stage);

        // A fixed start: a gap after the first polygon, which the search
        // may close, or a first polygon the search cannot start from.
        const safe_ref: PolyRef = switch (case_index % 3) {
            0 => from.ref,
            1 => 0,
            else => sentinel_ref,
        };
        try pair.fixPathStart(safe_ref, from.pos);
        const fixed: PathCopy = .of(&pair);
        const fixed_optimized = try pair.optimizePathTopology(&stage);
        if (safe_ref != from.ref) {
            try testing.expect(!fixed_optimized);
            try testing.expect(fixed.matches(&pair));
            refused += 1;
        }

        // One polygon, then the route's first two.
        try pair.reset(from.ref, from.pos);
        try testing.expect(!try pair.optimizePathTopology(&stage));
        if (started.route.count >= 2) {
            try pair.setCorridor(started.route.target, started.route.polys[0..2]);
            try testing.expect(!try pair.optimizePathTopology(&stage));
        }
    }
    try testing.expect(rewritten > 0);
    try testing.expect(shortened > 0);
    try testing.expect(refused > 0);

    // Corridors too small for a path the search would touch.
    for ([_]usize{ 1, 2 }) |capacity| {
        errdefer std.debug.print("  seed {d}, capacity {d}\n", .{ seed, capacity });
        const started = try stage.randomRoute(random, capacity, route_capacity);
        var pair = try Pair.create();
        defer pair.deinit(gpa);
        try testing.expect(try pair.init(gpa, @intCast(capacity)));
        try testing.expect(!try pair.optimizePathTopology(&stage));
        try pair.reset(started.from.ref, started.from.pos);
        try pair.setCorridor(started.route.target, started.route.polys[0..capacity]);
        try testing.expect(!try pair.optimizePathTopology(&stage));
    }
}

/// A corridor on each side with room for `capacity` polygons, holding the
/// hand-made path `polys` from `pos` to `target`.
fn startOn(
    gpa: std.mem.Allocator,
    capacity: usize,
    pos: [3]f32,
    target: [3]f32,
    polys: []const PolyRef,
) !Pair {
    var pair = try Pair.create();
    errdefer pair.deinit(gpa);
    try testing.expect(try pair.init(gpa, @intCast(capacity)));
    try pair.reset(polys[0], pos);
    try pair.setCorridor(target, polys);
    return pair;
}

/// Fails unless a crossing left every out-parameter alone.
fn expectUntouched(crossing: Crossing) !void {
    try testing.expect(!crossing.crossed);
    try testing.expectEqualSlices(PolyRef, &.{ sentinel_ref, sentinel_ref }, &crossing.refs);
    try support.expectPoint(sentinel_point, crossing.start_pos);
    try support.expectPoint(sentinel_point, crossing.end_pos);
}

/// Crosses `connection` in three-polygon paths: forward, backward, and
/// from a path that holds it twice.
fn expectCrossingsInShortPaths(
    gpa: std.mem.Allocator,
    stage: *Stage,
    connection: Connection,
) !void {
    const start = connection.start_ref;
    const end = connection.end_ref;
    const over = connection.ref;
    const one_polygon = start == end;

    // Forward, in a buffer the path fills and in a larger one.
    for ([_]usize{ 3, 8 }) |capacity| {
        const polys = [_]PolyRef{ start, over, end };
        var pair = try startOn(gpa, capacity, connection.start_pos, connection.end_pos, &polys);
        defer pair.deinit(gpa);
        const crossing = try pair.moveOverOffmeshConnection(stage, over);
        try testing.expect(crossing.crossed);
        try testing.expectEqualSlices(PolyRef, &.{ start, over }, &crossing.refs);
        try support.expectPoint(connection.start_pos, crossing.start_pos);
        try support.expectPoint(connection.end_pos, crossing.end_pos);
        try testing.expectEqualSlices(PolyRef, &.{end}, pair.port.getPath());
        try support.expectPoint(connection.end_pos, pair.port.getPos());
        // The corridor goes on from the far end.
        _ = try pair.findCorners(stage, 8);
        _ = try pair.movePosition(stage, connection.start_pos);
        _ = try pair.moveTargetPosition(stage, connection.start_pos);
    }

    // Backward: the ends come out in walking order, whichever way the
    // connection allows.
    {
        const polys = [_]PolyRef{ end, over, start };
        var pair = try startOn(gpa, 3, connection.end_pos, connection.start_pos, &polys);
        defer pair.deinit(gpa);
        const crossing = try pair.moveOverOffmeshConnection(stage, over);
        try testing.expect(crossing.crossed);
        try testing.expectEqualSlices(PolyRef, &.{ end, over }, &crossing.refs);
        if (!one_polygon) {
            try support.expectPoint(connection.end_pos, crossing.start_pos);
            try support.expectPoint(connection.start_pos, crossing.end_pos);
        }
        try testing.expectEqualSlices(PolyRef, &.{start}, pair.port.getPath());
        _ = try pair.findCorners(stage, 8);
    }

    // There and back: each crossing takes the first of the two.
    {
        const polys = [_]PolyRef{ start, over, end, over, start };
        var pair = try startOn(gpa, 5, connection.start_pos, connection.start_pos, &polys);
        defer pair.deinit(gpa);
        const there = try pair.moveOverOffmeshConnection(stage, over);
        try testing.expect(there.crossed);
        try testing.expectEqualSlices(PolyRef, &.{ end, over, start }, pair.port.getPath());
        const back = try pair.moveOverOffmeshConnection(stage, over);
        try testing.expect(back.crossed);
        try testing.expectEqualSlices(PolyRef, &.{start}, pair.port.getPath());
        // The connection is gone from the path.
        try expectUntouched(try pair.moveOverOffmeshConnection(stage, over));
        try testing.expectEqualSlices(PolyRef, &.{start}, pair.port.getPath());
    }
}
