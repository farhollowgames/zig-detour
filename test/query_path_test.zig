//! Path queries against the original: `findPath`, the sliced search,
//! `findStraightPath`, `raycast`, `raycastPath` and `moveAlongSurface`, with
//! the node pool a search leaves behind.
//!
//! Each call runs on both sides with the same input. Every output buffer is
//! filled with the same sentinel on both sides first and compared whole, so
//! a write past the reported count shows as a difference.
//!
//! The original writes past a full buffer in a few places; those calls give
//! the original spare room behind its buffer and expect the port to stop at
//! the buffer's end with the same status.

const std = @import("std");
const detour = @import("detour");
const support = @import("support.zig");
const reference = support.reference;
const testing = std.testing;

const PolyRef = detour.PolyRef;
const Status = detour.Status;
const NavMesh = detour.NavMesh;
const NavMeshQuery = detour.NavMeshQuery;
const QueryFilter = detour.QueryFilter;
const RaycastHit = detour.RaycastHit;
const MeshTile = detour.MeshTile;
const Poly = detour.Poly;
const StraightPathFlags = detour.StraightPathFlags;
const StraightPathOptions = detour.StraightPathOptions;
const FindPathOptions = detour.FindPathOptions;
const RaycastOptions = detour.RaycastOptions;

/// Cases each fuzz test runs per world.
const fuzz_case_count = 1000;
/// The longest polygon path any test asks for.
const path_capacity_max = 512;
/// The longest straight path any test asks for.
const straight_capacity_max = 256;
/// The longest visited list any test asks for.
const visited_capacity_max = 80;

/// A reference no mesh holds: its salt matches no tile's.
const sentinel_ref: PolyRef = std.math.maxInt(PolyRef) - 0x1234;
/// A coordinate no call writes.
const sentinel_coordinate: f32 = -98765.25;
const sentinel_point: [3]f32 = @splat(sentinel_coordinate);
const sentinel_flags: u8 = 0xa5;

// ---------------------------------------------------------------------------
// Worlds
// ---------------------------------------------------------------------------

const WorldCase = struct {
    name: []const u8,
    obj_text: []const u8,
    config: support.RecastConfig,
    options: support.LevelOptions,
};

/// The tiled configuration with a narrower agent, which gives the any-angle
/// search another ray length limit.
const narrow_tiled_config: support.RecastConfig = blk: {
    var config = support.tiled_config;
    config.agent_radius = 0.3;
    break :blk config;
};

const world_cases = [_]WorldCase{
    .{
        .name = "dungeon, one tile, off-mesh connections",
        .obj_text = support.dungeon_obj,
        .config = support.solo_config,
        .options = .{ .off_mesh_connection_count = 12, .vary_polygons = true, .seed = 21 },
    },
    .{
        .name = "nav_test, tiled",
        .obj_text = support.nav_test_obj,
        .config = support.tiled_config,
        .options = .{ .vary_polygons = true, .seed = 22 },
    },
    .{
        .name = "dungeon, tiled, narrow agent",
        .obj_text = support.dungeon_obj,
        .config = narrow_tiled_config,
        .options = .{ .off_mesh_connection_count = 12, .vary_polygons = true, .seed = 23 },
    },
    .{
        .name = "nav_test, tiled, two cell sizes",
        .obj_text = support.nav_test_obj,
        .config = support.tiled_config,
        .options = .{
            .off_mesh_connection_count = 12,
            .vary_polygons = true,
            .seed = 24,
            .odd_tile_cell_size = 0.2,
        },
    },
};

// ---------------------------------------------------------------------------
// Both sides of a query
// ---------------------------------------------------------------------------

/// The port's query and the original's over the same world, with the same
/// node count.
const Queries = struct {
    gpa: std.mem.Allocator,
    nav: *const NavMesh,
    reference_nav: *reference.NavMesh,
    port: NavMeshQuery,
    original: *reference.NavMeshQuery,

    /// Readies both queries in place and compares what the two `init` calls
    /// return. The world must outlive the queries.
    fn init(
        self: *Queries,
        gpa: std.mem.Allocator,
        world: *const support.World,
        node_count: i32,
    ) !void {
        const original = reference.ref_query_alloc() orelse return error.OutOfMemory;
        errdefer reference.ref_query_free(original);
        self.* = .{
            .gpa = gpa,
            .nav = &world.nav,
            .reference_nav = world.reference_nav,
            .port = .empty,
            .original = original,
        };
        errdefer self.port.deinit(gpa);
        const expected = reference.ref_query_init(original, world.reference_nav, node_count);
        try support.expectStatus(expected, self.port.init(gpa, &world.nav, node_count));
    }

    fn deinit(self: *Queries) void {
        self.port.deinit(self.gpa);
        reference.ref_query_free(self.original);
    }
};

/// The same filter on both sides.
const Filters = struct {
    port: QueryFilter,
    original: *reference.QueryFilter,

    fn init() !Filters {
        const original = reference.ref_filter_create() orelse return error.OutOfMemory;
        return .{ .port = .{}, .original = original };
    }

    fn deinit(self: *Filters) void {
        reference.ref_filter_destroy(self.original);
    }

    fn setAreaCost(self: *Filters, area: usize, cost: f32) void {
        self.port.area_cost[area] = cost;
        reference.ref_filter_set_area_cost(self.original, @intCast(area), cost);
    }

    fn setFlags(self: *Filters, include: u16, exclude: u16) void {
        self.port.include_flags = include;
        self.port.exclude_flags = exclude;
        reference.ref_filter_set_include_flags(self.original, include);
        reference.ref_filter_set_exclude_flags(self.original, exclude);
    }

    /// Back to the filter that passes every polygon at cost one.
    fn reset(self: *Filters) void {
        for (0..self.port.area_cost.len) |area| self.setAreaCost(area, 1.0);
        self.setFlags(0xffff, 0);
    }

    /// Mostly the default filter; otherwise other area costs, one area far
    /// dearer than the rest, or flags that cut polygons out of the mesh.
    fn randomize(self: *Filters, random: std.Random) void {
        self.reset();
        switch (random.uintLessThan(u32, 12)) {
            0, 1 => for (0..self.port.area_cost.len) |area| {
                self.setAreaCost(area, 0.5 + random.float(f32) * 9.5);
            },
            2 => self.setAreaCost(random.uintLessThan(usize, 6), 500.0),
            3 => self.setFlags(0xffff, @as(u16, 2) << random.uintLessThan(u4, 3)),
            4 => self.setFlags(@as(u16, 2) << random.uintLessThan(u4, 3), 0),
            5 => self.setFlags(0x8000, 0),
            else => {},
        }
    }
};

/// A point on the mesh and the polygon under it.
const Place = struct {
    ref: PolyRef,
    pos: [3]f32,
};

/// The polygon the original finds nearest to `point`, for both sides to
/// start from. The reference is zero where no polygon is in reach.
fn nearestPlace(queries: *const Queries, pass_all: *const Filters, point: [3]f32) Place {
    const half_extents: [3]f32 = .{ 2.0, 4.0, 2.0 };
    var place: Place = .{ .ref = 0, .pos = point };
    var nearest: [3]f32 = point;
    const status: Status = @bitCast(reference.ref_query_find_nearest_poly(
        queries.original,
        &point,
        &half_extents,
        pass_all.original,
        &place.ref,
        &nearest,
    ));
    if (status.succeeded and place.ref != 0) place.pos = nearest;
    return place;
}

/// A place near a random point of the level's ground.
fn randomPlace(
    queries: *const Queries,
    pass_all: *const Filters,
    world: *const support.World,
    random: std.Random,
) Place {
    return nearestPlace(queries, pass_all, world.level.groundPoint(random, 0.3));
}

/// A place on the mesh; an error when a world gives none in a few tries.
fn somePlace(
    queries: *const Queries,
    pass_all: *const Filters,
    world: *const support.World,
    random: std.Random,
) !Place {
    for (0..64) |_| {
        const place = randomPlace(queries, pass_all, world, random);
        if (place.ref != 0) return place;
    }
    return error.TestUnexpectedResult;
}

// ---------------------------------------------------------------------------
// Polygons of a mesh
// ---------------------------------------------------------------------------

/// One polygon of a mesh.
const PolyEntry = struct {
    ref: PolyRef,
    tile: *const MeshTile,
    poly: *const Poly,
};

/// Every polygon of the mesh's loaded tiles, in tile then polygon order.
fn collectPolys(gpa: std.mem.Allocator, nav: *const NavMesh) ![]PolyEntry {
    var list: std.ArrayList(PolyEntry) = .empty;
    errdefer list.deinit(gpa);
    var tile_index: i32 = 0;
    while (tile_index < nav.getMaxTiles()) : (tile_index += 1) {
        const tile = nav.getTile(tile_index);
        if (tile.header == null) continue;
        const base = nav.getPolyRefBase(tile);
        for (tile.polys, 0..) |*poly, poly_index| {
            try list.append(gpa, .{
                .ref = base | @as(PolyRef, @intCast(poly_index)),
                .tile = tile,
                .poly = poly,
            });
        }
    }
    return list.toOwnedSlice(gpa);
}

/// The polygon behind a reference, or null for a reference the mesh refuses.
fn entryOf(nav: *const NavMesh, ref: PolyRef) ?PolyEntry {
    if (!nav.isValidPolyRef(ref)) return null;
    const found = nav.getTileAndPolyByRefUnsafe(ref);
    return .{ .ref = ref, .tile = found.tile, .poly = found.poly };
}

/// The mean of the polygon's vertices.
fn polyCenter(entry: PolyEntry) [3]f32 {
    var center: [3]f32 = @splat(0.0);
    const count: usize = entry.poly.vert_count;
    for (entry.poly.verts[0..count]) |vert_index| {
        const vert = entry.tile.verts[vert_index];
        for (&center, vert) |*sum, value| sum.* += value;
    }
    const scale = 1.0 / @as(f32, @floatFromInt(count));
    for (&center) |*value| value.* *= scale;
    return center;
}

/// One of the polygon's vertices.
fn polyVertex(entry: PolyEntry, index: usize) [3]f32 {
    const count: usize = entry.poly.vert_count;
    return entry.tile.verts[entry.poly.verts[index % count]];
}

/// The point between two of the polygon's neighbouring vertices.
fn polyEdgeMiddle(entry: PolyEntry, index: usize) [3]f32 {
    const a = polyVertex(entry, index);
    const b = polyVertex(entry, index + 1);
    var middle: [3]f32 = undefined;
    for (&middle, a, b) |*value, from, to| value.* = (from + to) * 0.5;
    return middle;
}

/// A polygon `entry` links to, chosen evenly; zero when it links to none.
fn randomNeighbour(entry: PolyEntry, random: std.Random) PolyRef {
    var chosen: PolyRef = 0;
    var seen: u32 = 0;
    var link_index = entry.poly.first_link;
    while (link_index != detour.nav_mesh.null_link) {
        const link = entry.tile.links[link_index];
        seen += 1;
        if (random.uintLessThan(u32, seen) == 0) chosen = link.ref;
        link_index = link.next;
    }
    return chosen;
}

/// The same polygon under another salt, which no mesh accepts.
fn staleRef(nav: *const NavMesh, ref: PolyRef) PolyRef {
    const parts = nav.decodePolyId(ref);
    return nav.encodePolyId(parts.salt + 1, parts.tile_index, parts.poly_index);
}

fn lerpPoint(a: [3]f32, b: [3]f32, t: f32) [3]f32 {
    var result: [3]f32 = undefined;
    for (&result, a, b) |*value, from, to| value.* = from + (to - from) * t;
    return result;
}

fn isFinitePoint(point: [3]f32) bool {
    for (point) |value| {
        if (!std.math.isFinite(value)) return false;
    }
    return true;
}

/// Whole point buffers, bit for bit.
fn expectPoints(expected: []const [3]f32, actual: []const [3]f32) !void {
    try testing.expectEqual(expected.len, actual.len);
    if (std.mem.eql(u8, std.mem.sliceAsBytes(expected), std.mem.sliceAsBytes(actual))) return;
    for (expected, actual, 0..) |expected_point, actual_point, index| {
        errdefer std.debug.print("  at point {d}\n", .{index});
        try support.expectPoint(expected_point, actual_point);
    }
    return error.TestExpectedEqual;
}

// ---------------------------------------------------------------------------
// The node pool a search leaves behind
// ---------------------------------------------------------------------------

/// The fields of one of the original's nodes.
const NodeFields = struct {
    pos: [3]f32,
    cost: f32,
    total: f32,
    parent_index: u32,
    state: u32,
    flags: u32,
    id: PolyRef,
};

/// Compares the two main node pools: their sizes, every node in use in index
/// order, every bucket's first node and every node's next link.
fn expectSamePool(queries: *Queries) !void {
    const port = queries.port.getNodePool();
    const original = reference.ref_query_get_node_pool(queries.original) orelse
        return error.TestUnexpectedResult;

    const max_nodes = reference.ref_node_pool_get_max_nodes(original);
    const hash_size = reference.ref_node_pool_get_hash_size(original);
    const node_count = reference.ref_node_pool_get_node_count(original);
    try testing.expectEqual(max_nodes, port.getMaxNodes());
    try testing.expectEqual(hash_size, port.getHashSize());
    try testing.expectEqual(node_count, port.getNodeCount());

    var index: i32 = 0;
    while (index < node_count) : (index += 1) {
        errdefer std.debug.print("  at node {d} of {d}\n", .{ index, node_count });
        const idx: u32 = @intCast(index + 1);
        const expected_node = reference.ref_node_pool_get_node_at_idx(original, idx) orelse
            return error.TestUnexpectedResult;
        const actual = port.getNodeAtIdx(idx) orelse return error.TestUnexpectedResult;
        var expected: NodeFields = undefined;
        reference.ref_node_fields(
            expected_node,
            &expected.pos,
            &expected.cost,
            &expected.total,
            &expected.parent_index,
            &expected.state,
            &expected.flags,
            &expected.id,
        );
        try support.expectPoint(expected.pos, actual.pos);
        try support.expectFloat(expected.cost, actual.cost);
        try support.expectFloat(expected.total, actual.total);
        try testing.expectEqual(expected.parent_index, @as(u32, actual.bits.parent_index));
        try testing.expectEqual(expected.state, @as(u32, actual.bits.state));
        const actual_flags: u3 = @bitCast(actual.bits.flags);
        try testing.expectEqual(expected.flags, @as(u32, actual_flags));
        try testing.expectEqual(expected.id, actual.id);
    }

    var bucket: i32 = 0;
    while (bucket < hash_size) : (bucket += 1) {
        errdefer std.debug.print("  at bucket {d} of {d}\n", .{ bucket, hash_size });
        const expected = reference.ref_node_pool_get_first(original, bucket);
        try testing.expectEqual(expected, port.getFirst(bucket));
    }

    var link: i32 = 0;
    while (link < max_nodes) : (link += 1) {
        errdefer std.debug.print("  at next link {d} of {d}\n", .{ link, max_nodes });
        const expected = reference.ref_node_pool_get_next(original, link);
        try testing.expectEqual(expected, port.getNext(link));
    }
}

/// Whether each of `refs` is in the closed list, on both sides.
fn expectSameClosedList(queries: *const Queries, refs: []const PolyRef) !void {
    for (refs) |ref| {
        errdefer std.debug.print("  closed list of {x}\n", .{ref});
        try testing.expectEqual(
            reference.ref_query_is_in_closed_list(queries.original, ref),
            queries.port.isInClosedList(ref),
        );
    }
}

/// The pool and the closed list of a path's polygons, of the two ends and of
/// references no search visits.
fn expectSameSearchState(
    queries: *Queries,
    path: []const PolyRef,
    start_ref: PolyRef,
    end_ref: PolyRef,
) !void {
    try expectSamePool(queries);
    try expectSameClosedList(queries, path);
    const others = [_]PolyRef{ start_ref, end_ref, 0, sentinel_ref };
    try expectSameClosedList(queries, &others);
}

// ---------------------------------------------------------------------------
// findPath on both sides
// ---------------------------------------------------------------------------

/// Whether the build refuses infinite and NaN positions. A build without
/// those checks runs a search over them, which has no bound on either side,
/// so the tests feed such positions only where they are refused.
const non_finite_checked = !detour.common.isFinite(std.math.inf(f32));

/// What a path call gave: the status, the count and the whole buffer.
const PathResult = struct {
    status: Status,
    count: usize,
    path: [path_capacity_max]PolyRef,

    fn slice(self: *const PathResult) []const PolyRef {
        return self.path[0..@min(self.count, path_capacity_max)];
    }
};

/// Runs `findPath` on both sides into buffers of `capacity` references and
/// compares the status, the count, the whole buffers and the search state.
fn expectSameFindPath(
    queries: *Queries,
    filters: *const Filters,
    start: Place,
    end: Place,
    capacity: usize,
    result: *PathResult,
) !void {
    var expected: [path_capacity_max]PolyRef = @splat(sentinel_ref);
    var expected_count: i32 = -7;
    const expected_status = reference.ref_query_find_path(
        queries.original,
        start.ref,
        end.ref,
        &start.pos,
        &end.pos,
        filters.original,
        &expected,
        &expected_count,
        @intCast(capacity),
    );

    result.path = @splat(sentinel_ref);
    result.count = 77;
    result.status = queries.port.findPath(
        start.ref,
        end.ref,
        start.pos,
        end.pos,
        &filters.port,
        result.path[0..capacity],
        &result.count,
    );

    try support.expectStatus(expected_status, result.status);
    try testing.expect(expected_count >= 0);
    try testing.expectEqual(@as(usize, @intCast(expected_count)), result.count);
    try testing.expect(result.count <= capacity);
    try testing.expectEqualSlices(PolyRef, &expected, &result.path);
    try expectSameSearchState(queries, result.slice(), start.ref, end.ref);
}

// ---------------------------------------------------------------------------
// The sliced search on both sides
// ---------------------------------------------------------------------------

/// References the original gets in front of its path buffer. It reads the
/// one before the buffer when the path's first ray stores nothing.
const path_guard = 1;
/// References the original gets behind the largest buffer. It writes one
/// past a buffer that a ray has just filled.
const path_slack = 2;

fn expectSameSlicedInit(
    queries: *Queries,
    filters: *const Filters,
    start: Place,
    end: Place,
    options: FindPathOptions,
) !Status {
    const expected = reference.ref_query_init_sliced_find_path(
        queries.original,
        start.ref,
        end.ref,
        &start.pos,
        &end.pos,
        filters.original,
        @bitCast(options),
    );
    const actual = queries.port.initSlicedFindPath(
        start.ref,
        end.ref,
        start.pos,
        end.pos,
        &filters.port,
        options,
    );
    try support.expectStatus(expected, actual);
    return actual;
}

/// One update with the same budget on both sides; compares the status and
/// the iterations done, which stay untouched where the caller passes none.
fn expectSameSlicedUpdate(queries: *Queries, budget: i32, with_done: bool) !Status {
    var expected_done: i32 = -7;
    var actual_done: i32 = -7;
    const expected = reference.ref_query_update_sliced_find_path(
        queries.original,
        budget,
        if (with_done) &expected_done else null,
    );
    const actual = queries.port.updateSlicedFindPath(
        budget,
        if (with_done) &actual_done else null,
    );
    try support.expectStatus(expected, actual);
    try testing.expectEqual(expected_done, actual_done);
    return actual;
}

/// Finalizes on both sides into buffers of `capacity` references: the whole
/// path when `existing` is null, else the partial one along `existing`.
fn expectSameSlicedFinalize(
    queries: *Queries,
    existing: ?[]const PolyRef,
    capacity: usize,
    result: *PathResult,
) !void {
    var expected: [path_guard + path_capacity_max + path_slack]PolyRef = @splat(sentinel_ref);
    var expected_count: i32 = -7;
    const expected_path: [*]PolyRef = expected[path_guard..];
    const expected_status = if (existing) |refs|
        reference.ref_query_finalize_sliced_find_path_partial(
            queries.original,
            refs.ptr,
            @intCast(refs.len),
            expected_path,
            &expected_count,
            @intCast(capacity),
        )
    else
        reference.ref_query_finalize_sliced_find_path(
            queries.original,
            expected_path,
            &expected_count,
            @intCast(capacity),
        );

    result.path = @splat(sentinel_ref);
    result.count = 77;
    result.status = if (existing) |refs|
        queries.port.finalizeSlicedFindPathPartial(refs, result.path[0..capacity], &result.count)
    else
        queries.port.finalizeSlicedFindPath(result.path[0..capacity], &result.count);

    try support.expectStatus(expected_status, result.status);
    try testing.expect(expected_count >= 0);
    var count: usize = @intCast(expected_count);
    if (count == capacity + 1) {
        // The original wrote one reference behind a buffer a ray had filled;
        // the port stops at the buffer's end with the same status.
        expected[path_guard + capacity] = sentinel_ref;
        count = capacity;
    }
    try testing.expectEqual(count, result.count);
    try testing.expectEqualSlices(
        PolyRef,
        expected[path_guard..][0..path_capacity_max],
        &result.path,
    );
    try testing.expectEqual(sentinel_ref, expected[0]);
    for (expected[path_guard + path_capacity_max ..]) |ref| {
        try testing.expectEqual(sentinel_ref, ref);
    }
}

// ---------------------------------------------------------------------------
// findStraightPath on both sides
// ---------------------------------------------------------------------------

/// What a straight path call gave: the status, the count and the whole
/// buffers.
const StraightResult = struct {
    status: Status,
    count: usize,
    /// Whether the original wrote a point past its buffer's end.
    overran: bool,
    points: [straight_capacity_max][3]f32,
    flags: [straight_capacity_max]StraightPathFlags,
    refs: [straight_capacity_max]PolyRef,
};

/// Which of the optional buffers a straight path call passes.
const StraightBuffers = struct {
    flags: bool = true,
    refs: bool = true,

    const all = [_]StraightBuffers{
        .{ .flags = true, .refs = true },
        .{ .flags = true, .refs = false },
        .{ .flags = false, .refs = true },
        .{ .flags = false, .refs = false },
    };
};

const straight_options = [_]StraightPathOptions{
    .{},
    .{ .area_crossings = true },
    .{ .all_crossings = true },
    .{ .area_crossings = true, .all_crossings = true },
};

/// Runs `findStraightPath` on both sides into buffers of `capacity` points
/// and compares the status, the count and the whole buffers.
///
/// The original has one more point of room than it is told: a corridor that
/// breaks while crossings are asked for makes it write one point past a full
/// buffer, where the port stops at the end with the same status.
fn expectSameStraightPath(
    queries: *const Queries,
    start_pos: [3]f32,
    end_pos: [3]f32,
    path: []const PolyRef,
    capacity: usize,
    buffers: StraightBuffers,
    options: StraightPathOptions,
    result: *StraightResult,
) !void {
    const no_path = [1]PolyRef{0};
    const path_pointer: [*]const PolyRef = if (path.len == 0) &no_path else path.ptr;

    var expected_points: [straight_capacity_max + 1][3]f32 = @splat(sentinel_point);
    var expected_flags: [straight_capacity_max + 1]u8 = @splat(sentinel_flags);
    var expected_refs: [straight_capacity_max + 1]PolyRef = @splat(sentinel_ref);
    var expected_count: i32 = -7;
    const expected_status = reference.ref_query_find_straight_path(
        queries.original,
        &start_pos,
        &end_pos,
        path_pointer,
        @intCast(path.len),
        @ptrCast(&expected_points),
        if (buffers.flags) &expected_flags else null,
        if (buffers.refs) &expected_refs else null,
        &expected_count,
        @intCast(capacity),
        @bitCast(options),
    );

    result.points = @splat(sentinel_point);
    result.flags = @splat(@bitCast(sentinel_flags));
    result.refs = @splat(sentinel_ref);
    result.count = 77;
    result.status = queries.port.findStraightPath(
        start_pos,
        end_pos,
        path,
        result.points[0..capacity],
        if (buffers.flags) result.flags[0..capacity] else null,
        if (buffers.refs) result.refs[0..capacity] else null,
        &result.count,
        options,
    );

    try support.expectStatus(expected_status, result.status);
    try testing.expect(expected_count >= 0);
    var count: usize = @intCast(expected_count);
    result.overran = count == capacity + 1;
    if (result.overran) {
        expected_points[capacity] = sentinel_point;
        expected_flags[capacity] = sentinel_flags;
        expected_refs[capacity] = sentinel_ref;
        count = capacity;
    }
    try testing.expectEqual(count, result.count);
    try expectPoints(expected_points[0..straight_capacity_max], &result.points);
    try testing.expectEqualSlices(
        u8,
        expected_flags[0..straight_capacity_max],
        std.mem.sliceAsBytes(&result.flags),
    );
    try testing.expectEqualSlices(PolyRef, expected_refs[0..straight_capacity_max], &result.refs);
    try support.expectPoint(sentinel_point, expected_points[straight_capacity_max]);
    try testing.expectEqual(sentinel_flags, expected_flags[straight_capacity_max]);
    try testing.expectEqual(sentinel_ref, expected_refs[straight_capacity_max]);
}

// ---------------------------------------------------------------------------
// raycast, raycastPath and moveAlongSurface on both sides
// ---------------------------------------------------------------------------

/// What a ray cast gave: every field of the hit and the whole path buffer.
const RayResult = struct {
    status: Status,
    t: f32,
    normal: [3]f32,
    edge_index: i32,
    path_cost: f32,
    count: usize,
    path: [path_capacity_max]PolyRef,
};

/// Runs `raycast` on both sides with a path buffer of `capacity` references
/// and compares the status and every field of the hit.
fn expectSameRay(
    queries: *const Queries,
    filters: *const Filters,
    start: Place,
    end_pos: [3]f32,
    options: RaycastOptions,
    prev_ref: PolyRef,
    capacity: usize,
    result: *RayResult,
) !void {
    var expected_t: f32 = sentinel_coordinate;
    var expected_normal: [3]f32 = sentinel_point;
    var expected_edge_index: i32 = -7;
    var expected_cost: f32 = sentinel_coordinate;
    var expected_count: i32 = -7;
    var expected_path: [path_capacity_max]PolyRef = @splat(sentinel_ref);
    const expected_status = reference.ref_query_raycast_hit(
        queries.original,
        start.ref,
        &start.pos,
        &end_pos,
        filters.original,
        @bitCast(options),
        prev_ref,
        &expected_t,
        &expected_normal,
        &expected_edge_index,
        &expected_path,
        &expected_count,
        @intCast(capacity),
        &expected_cost,
    );

    // The original's hit starts zeroed, and neither side writes the normal
    // or the edge of a ray it refuses.
    result.path = @splat(sentinel_ref);
    var hit: RaycastHit = .{
        .t = sentinel_coordinate,
        .hit_normal = @splat(0.0),
        .hit_edge_index = 0,
        .path = result.path[0..capacity],
        .path_count = 77,
        .path_cost = sentinel_coordinate,
    };
    result.status = queries.port.raycast(
        start.ref,
        start.pos,
        end_pos,
        &filters.port,
        options,
        &hit,
        prev_ref,
    );
    result.t = hit.t;
    result.normal = hit.hit_normal;
    result.edge_index = hit.hit_edge_index;
    result.path_cost = hit.path_cost;
    result.count = hit.path_count;

    try support.expectStatus(expected_status, result.status);
    try support.expectFloat(expected_t, result.t);
    try support.expectPoint(expected_normal, result.normal);
    try testing.expectEqual(expected_edge_index, result.edge_index);
    try support.expectFloat(expected_cost, result.path_cost);
    try testing.expect(expected_count >= 0);
    try testing.expectEqual(@as(usize, @intCast(expected_count)), result.count);
    try testing.expect(result.count <= capacity);
    try testing.expectEqualSlices(PolyRef, &expected_path, &result.path);
}

/// Runs `raycastPath` on both sides, with or without a normal, and with a
/// path buffer of `capacity` references or with none.
///
/// The original copies a normal it never set when it refuses the input, so
/// it gets no normal there, and the port's must come back zero.
fn expectSameRayPath(
    queries: *const Queries,
    filters: *const Filters,
    start: Place,
    end_pos: [3]f32,
    with_normal: bool,
    capacity: ?usize,
    result: *RayResult,
) !void {
    const accepted = isFinitePoint(start.pos) and isFinitePoint(end_pos) and
        reference.ref_nav_mesh_is_valid_poly_ref(queries.reference_nav, start.ref);

    var expected_t: f32 = sentinel_coordinate;
    var expected_normal: [3]f32 = @splat(0.0);
    var expected_count: i32 = 77;
    var expected_path: [path_capacity_max]PolyRef = @splat(sentinel_ref);
    const expected_status = reference.ref_query_raycast(
        queries.original,
        start.ref,
        &start.pos,
        &end_pos,
        filters.original,
        &expected_t,
        if (with_normal and accepted) &expected_normal else null,
        if (capacity != null) &expected_path else null,
        if (capacity != null) &expected_count else null,
        @intCast(capacity orelse 0),
    );

    result.path = @splat(sentinel_ref);
    result.t = sentinel_coordinate;
    result.normal = sentinel_point;
    result.edge_index = 0;
    result.path_cost = 0.0;
    result.count = 77;
    result.status = queries.port.raycastPath(
        start.ref,
        start.pos,
        end_pos,
        &filters.port,
        &result.t,
        if (with_normal) &result.normal else null,
        if (capacity) |room| result.path[0..room] else null,
        if (capacity != null) &result.count else null,
    );

    try support.expectStatus(expected_status, result.status);
    try support.expectFloat(expected_t, result.t);
    try support.expectPoint(if (with_normal) expected_normal else sentinel_point, result.normal);
    try testing.expect(expected_count >= 0);
    try testing.expectEqual(@as(usize, @intCast(expected_count)), result.count);
    try testing.expectEqualSlices(PolyRef, &expected_path, &result.path);
}

/// What a move gave: the status, the end point and the whole visited buffer.
const MoveResult = struct {
    status: Status,
    pos: [3]f32,
    count: usize,
    visited: [visited_capacity_max]PolyRef,
};

/// Runs `moveAlongSurface` on both sides with a visited buffer of `capacity`
/// references and compares the status, the end point and the whole buffers.
fn expectSameMove(
    queries: *Queries,
    filters: *const Filters,
    start: Place,
    end_pos: [3]f32,
    capacity: usize,
    result: *MoveResult,
) !void {
    var expected_pos: [3]f32 = sentinel_point;
    var expected_visited: [visited_capacity_max]PolyRef = @splat(sentinel_ref);
    var expected_count: i32 = -7;
    const expected_status = reference.ref_query_move_along_surface(
        queries.original,
        start.ref,
        &start.pos,
        &end_pos,
        filters.original,
        &expected_pos,
        &expected_visited,
        &expected_count,
        @intCast(capacity),
    );

    result.pos = sentinel_point;
    result.visited = @splat(sentinel_ref);
    result.count = 77;
    result.status = queries.port.moveAlongSurface(
        start.ref,
        start.pos,
        end_pos,
        &filters.port,
        &result.pos,
        result.visited[0..capacity],
        &result.count,
    );

    try support.expectStatus(expected_status, result.status);
    try support.expectPoint(expected_pos, result.pos);
    try testing.expect(expected_count >= 0);
    try testing.expectEqual(@as(usize, @intCast(expected_count)), result.count);
    try testing.expect(result.count <= capacity);
    try testing.expectEqualSlices(PolyRef, &expected_visited, &result.visited);
}

// ---------------------------------------------------------------------------
// Random input
// ---------------------------------------------------------------------------

/// Node counts the random tests search with: room for the worlds' paths, and
/// too few nodes for most of them.
const roomy_node_count = 2048;
const tight_node_count = 48;

fn nonFinite(random: std.Random) f32 {
    return switch (random.uintLessThan(u32, 3)) {
        0 => std.math.nan(f32),
        1 => std.math.inf(f32),
        else => -std.math.inf(f32),
    };
}

/// A reference the mesh refuses: zero, a stale salt or one no tile holds.
fn refusedRef(nav: *const NavMesh, ref: PolyRef, random: std.Random) PolyRef {
    return switch (random.uintLessThan(u32, 3)) {
        0 => 0,
        1 => if (ref != 0) staleRef(nav, ref) else sentinel_ref,
        else => sentinel_ref,
    };
}

/// A buffer size around `count`: none, one, two, shorter, exact or longer.
fn randomCapacity(random: std.Random, count: usize, capacity_max: usize) usize {
    const capacity: usize = switch (random.uintLessThan(u32, 8)) {
        0 => 0,
        1 => 1,
        2 => 2,
        3 => count / 2,
        4 => count -| 1,
        5 => count,
        6 => count + 1,
        else => random.uintLessThan(usize, count + 2),
    };
    return @min(capacity, capacity_max);
}

/// The two ends of a search.
const Ends = struct {
    start: Place,
    end: Place,
};

/// Mostly two places anywhere in the world; otherwise the same polygon at
/// both ends, neighbouring polygons, any polygon of the mesh (off-mesh
/// connections among them), references the mesh refuses, and positions away
/// from their polygons or not finite.
fn randomEnds(
    queries: *const Queries,
    pass_all: *const Filters,
    world: *const support.World,
    polys: []const PolyEntry,
    random: std.Random,
) Ends {
    var ends: Ends = .{
        .start = randomPlace(queries, pass_all, world, random),
        .end = randomPlace(queries, pass_all, world, random),
    };
    switch (random.uintLessThan(u32, 24)) {
        0 => ends.end.ref = ends.start.ref,
        1 => ends.end = ends.start,
        2, 3 => if (entryOf(queries.nav, ends.start.ref)) |entry| {
            const neighbour = randomNeighbour(entry, random);
            if (entryOf(queries.nav, neighbour)) |next| {
                ends.end = .{ .ref = neighbour, .pos = polyCenter(next) };
            }
        },
        4 => ends.start.ref = refusedRef(queries.nav, ends.start.ref, random),
        5 => ends.end.ref = refusedRef(queries.nav, ends.end.ref, random),
        6 => ends.start.pos = polyCenter(polys[random.uintLessThan(usize, polys.len)]),
        7 => ends.end.pos[1] += 40.0,
        8 => if (non_finite_checked) {
            ends.start.pos[random.uintLessThan(usize, 3)] = nonFinite(random);
        },
        9 => if (non_finite_checked) {
            ends.end.pos[random.uintLessThan(usize, 3)] = nonFinite(random);
        },
        10 => {
            const entry = polys[random.uintLessThan(usize, polys.len)];
            ends.start = .{ .ref = entry.ref, .pos = polyCenter(entry) };
        },
        11 => {
            const entry = polys[random.uintLessThan(usize, polys.len)];
            ends.end = .{ .ref = entry.ref, .pos = polyCenter(entry) };
        },
        else => {},
    }
    return ends;
}

test "findPath matches the original on random pairs" {
    const gpa = testing.allocator;
    for (world_cases, 0..) |case, case_index| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{case.name});
        var world: support.World = undefined;
        try world.init(gpa, case.obj_text, case.config, case.options);
        defer world.deinit();
        const polys = try collectPolys(gpa, &world.nav);
        defer gpa.free(polys);

        var roomy: Queries = undefined;
        try roomy.init(gpa, &world, roomy_node_count);
        defer roomy.deinit();
        var tight: Queries = undefined;
        try tight.init(gpa, &world, tight_node_count);
        defer tight.deinit();
        var pass_all = try Filters.init();
        defer pass_all.deinit();
        var filters = try Filters.init();
        defer filters.deinit();

        const seed: u64 = 0x9a7b_0100 + @as(u64, case_index);
        var prng: std.Random.DefaultPrng = .init(seed);
        const random = prng.random();
        var result: PathResult = undefined;
        var reached: u32 = 0;
        for (0..fuzz_case_count) |index| {
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, index });
            const ends = randomEnds(&roomy, &pass_all, &world, polys, random);
            filters.randomize(random);

            const full = path_capacity_max;
            try expectSameFindPath(&roomy, &filters, ends.start, ends.end, full, &result);
            const complete = result.status.succeeded and !result.status.partial_result;
            if (complete and result.count > 1) reached += 1;
            const short = randomCapacity(random, result.count, full);
            try expectSameFindPath(&roomy, &filters, ends.start, ends.end, short, &result);

            try expectSameFindPath(&tight, &filters, ends.start, ends.end, full, &result);
            const shorter = randomCapacity(random, result.count, full);
            try expectSameFindPath(&tight, &filters, ends.start, ends.end, shorter, &result);
        }
        // Without whole paths over several polygons the cases compare little.
        try testing.expect(reached > fuzz_case_count / 8);
    }
}

/// Iterations one update may run: a few, many, and no limit.
const sliced_budgets = [_]i32{ 1, 2, 7, 64, std.math.maxInt(i32) };
/// Updates after which a search that still runs counts as stuck.
const sliced_update_limit = 1 << 20;

/// An existing path for the partial finalize, written into `buffer`: the
/// path an earlier search found, parts of it, that path with a tail no
/// search visits, references no search visits, or nothing.
fn randomExisting(
    random: std.Random,
    found: []const PolyRef,
    polys: []const PolyEntry,
    buffer: *[path_capacity_max + 16]PolyRef,
) []const PolyRef {
    var count: usize = 0;
    switch (random.uintLessThan(u32, 8)) {
        0, 1, 2 => {
            @memcpy(buffer[0..found.len], found);
            count = found.len;
        },
        3 => {
            count = random.uintLessThan(usize, found.len + 1);
            @memcpy(buffer[0..count], found[0..count]);
        },
        4 => {
            count = 1 + random.uintLessThan(usize, 6);
            for (buffer[0..count]) |*ref| ref.* = if (random.boolean()) sentinel_ref else 0;
        },
        5 => {
            count = 1 + random.uintLessThan(usize, 12);
            for (buffer[0..count]) |*ref| {
                ref.* = polys[random.uintLessThan(usize, polys.len)].ref;
            }
        },
        6 => {
            @memcpy(buffer[0..found.len], found);
            buffer[found.len..][0..3].* = .{ sentinel_ref, 0, sentinel_ref };
            count = found.len + 3;
        },
        else => {},
    }
    return buffer[0..count];
}

test "the sliced search matches the original on random pairs" {
    const gpa = testing.allocator;
    for (world_cases, 0..) |case, case_index| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{case.name});
        var world: support.World = undefined;
        try world.init(gpa, case.obj_text, case.config, case.options);
        defer world.deinit();
        const polys = try collectPolys(gpa, &world.nav);
        defer gpa.free(polys);

        var roomy: Queries = undefined;
        try roomy.init(gpa, &world, roomy_node_count);
        defer roomy.deinit();
        var tight: Queries = undefined;
        try tight.init(gpa, &world, tight_node_count);
        defer tight.deinit();
        var pass_all = try Filters.init();
        defer pass_all.deinit();
        var filters = try Filters.init();
        defer filters.deinit();

        const seed: u64 = 0x9a7b_0200 + @as(u64, case_index);
        var prng: std.Random.DefaultPrng = .init(seed);
        const random = prng.random();
        var found: PathResult = undefined;
        var result: PathResult = undefined;
        var existing_buffer: [path_capacity_max + 16]PolyRef = undefined;
        var finished: u32 = 0;
        for (0..fuzz_case_count) |index| {
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, index });
            const queries = if (index % 3 == 2) &tight else &roomy;
            const ends = randomEnds(queries, &pass_all, &world, polys, random);
            filters.randomize(random);
            const options: FindPathOptions = .{ .any_angle = random.boolean() };

            // A whole search first: its path is the one the partial finalize
            // follows, and its length sets the short buffers.
            const full = path_capacity_max;
            try expectSameFindPath(queries, &filters, ends.start, ends.end, full, &found);

            var status = try expectSameSlicedInit(queries, &filters, ends.start, ends.end, options);
            try expectSamePool(queries);

            const budget = sliced_budgets[random.uintLessThan(usize, sliced_budgets.len)];
            const with_done = random.boolean();
            const updates_max: u32 = if (random.uintLessThan(u32, 5) == 0)
                1 + random.uintLessThan(u32, 4)
            else
                sliced_update_limit;
            var updates: u32 = 0;
            while (status.in_progress and updates < updates_max) : (updates += 1) {
                if (random.uintLessThan(u32, 16) == 0) {
                    const idle = try expectSameSlicedUpdate(queries, 0, !with_done);
                    try testing.expect(idle.in_progress);
                }
                status = try expectSameSlicedUpdate(queries, budget, with_done);
            }
            try testing.expect(updates < sliced_update_limit);
            if (!status.in_progress and status.succeeded) finished += 1;
            try expectSameSearchState(queries, found.slice(), ends.start.ref, ends.end.ref);

            const existing: ?[]const PolyRef = if (random.boolean())
                randomExisting(random, found.slice(), polys, &existing_buffer)
            else
                null;
            // No room for a path: refused, and the search stays as it is.
            if (random.uintLessThan(u32, 4) == 0) {
                try expectSameSlicedFinalize(queries, existing, 0, &result);
            }
            // A search left unfinished now and then, for the next to replace.
            if (random.uintLessThan(u32, 20) == 0) continue;

            const capacity = if (random.boolean())
                full
            else
                randomCapacity(random, found.count, full);
            try expectSameSlicedFinalize(queries, existing, capacity, &result);
            try expectSameSearchState(queries, result.slice(), ends.start.ref, ends.end.ref);

            // The search is over: a further update and finalize find none.
            _ = try expectSameSlicedUpdate(queries, 4, true);
            if (random.uintLessThan(u32, 8) == 0) {
                try expectSameSlicedFinalize(queries, existing, full, &result);
            }
        }
        try testing.expect(finished > fuzz_case_count / 8);
    }
}

/// A corridor for a straight path, written into `buffer`: mostly the path a
/// search found; otherwise a part of it, one polygon, none, the path
/// backwards, or the path with a polygon the mesh refuses or one that does
/// not link to its neighbours.
fn randomCorridor(
    random: std.Random,
    nav: *const NavMesh,
    found: []const PolyRef,
    polys: []const PolyEntry,
    buffer: *[path_capacity_max]PolyRef,
) []const PolyRef {
    @memcpy(buffer[0..found.len], found);
    var corridor: []PolyRef = buffer[0..found.len];
    if (corridor.len == 0) return corridor;
    switch (random.uintLessThan(u32, 16)) {
        0 => corridor = corridor[0 .. 1 + random.uintLessThan(usize, corridor.len)],
        1 => corridor = corridor[random.uintLessThan(usize, corridor.len)..],
        2 => corridor = corridor[0..1],
        3 => corridor = corridor[0..0],
        4 => std.mem.reverse(PolyRef, corridor),
        5 => corridor[random.uintLessThan(usize, corridor.len)] = sentinel_ref,
        6 => corridor[random.uintLessThan(usize, corridor.len)] = 0,
        7 => {
            const at = random.uintLessThan(usize, corridor.len);
            corridor[at] = staleRef(nav, corridor[at]);
        },
        8 => {
            const at = random.uintLessThan(usize, corridor.len);
            corridor[at] = polys[random.uintLessThan(usize, polys.len)].ref;
        },
        else => {},
    }
    return corridor;
}

test "findStraightPath matches the original over found paths" {
    const gpa = testing.allocator;
    for (world_cases, 0..) |case, case_index| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{case.name});
        var world: support.World = undefined;
        try world.init(gpa, case.obj_text, case.config, case.options);
        defer world.deinit();
        const polys = try collectPolys(gpa, &world.nav);
        defer gpa.free(polys);

        var queries: Queries = undefined;
        try queries.init(gpa, &world, roomy_node_count);
        defer queries.deinit();
        var pass_all = try Filters.init();
        defer pass_all.deinit();

        const seed: u64 = 0x9a7b_0300 + @as(u64, case_index);
        var prng: std.Random.DefaultPrng = .init(seed);
        const random = prng.random();
        var found: PathResult = undefined;
        var corridor_buffer: [path_capacity_max]PolyRef = undefined;
        var result: StraightResult = undefined;
        var walked: u32 = 0;
        for (0..fuzz_case_count) |index| {
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, index });
            const ends = randomEnds(&queries, &pass_all, &world, polys, random);
            const full = path_capacity_max;
            try expectSameFindPath(&queries, &pass_all, ends.start, ends.end, full, &found);
            const corridor = randomCorridor(
                random,
                &world.nav,
                found.slice(),
                polys,
                &corridor_buffer,
            );

            // Mostly the ends the path was found for; otherwise ends away
            // from the path, above it, or not finite.
            var start_pos = ends.start.pos;
            var end_pos = ends.end.pos;
            switch (random.uintLessThan(u32, 16)) {
                0 => start_pos = world.level.groundPoint(random, 0.3),
                1 => end_pos = world.level.groundPoint(random, 0.3),
                2 => start_pos[1] += 40.0,
                3 => if (non_finite_checked) {
                    start_pos[random.uintLessThan(usize, 3)] = nonFinite(random);
                },
                4 => if (non_finite_checked) {
                    end_pos[random.uintLessThan(usize, 3)] = nonFinite(random);
                },
                else => {},
            }

            for (straight_options) |options| {
                const large = straight_capacity_max;
                const buffers = StraightBuffers.all[random.uintLessThan(usize, 4)];
                try expectSameStraightPath(
                    &queries,
                    start_pos,
                    end_pos,
                    corridor,
                    large,
                    buffers,
                    options,
                    &result,
                );
                if (result.status.succeeded and result.count >= 2) walked += 1;

                const small = randomCapacity(random, result.count, large);
                const others = StraightBuffers.all[random.uintLessThan(usize, 4)];
                try expectSameStraightPath(
                    &queries,
                    start_pos,
                    end_pos,
                    corridor,
                    small,
                    others,
                    options,
                    &result,
                );
            }
        }
        try testing.expect(walked > fuzz_case_count / 8);
    }
}

/// A ray or a move: where it starts and the point it heads for.
const Segment = struct {
    start: Place,
    end_pos: [3]f32,
};

/// Mostly from a place towards another anywhere in the world, across walls
/// and tile borders; otherwise no way at all, a short way, ends on and past
/// the start polygon's vertices and edges, ways along its edges, far out of
/// the world, any polygon of the mesh as the start, and starts the mesh
/// refuses or positions that are not finite.
fn randomSegment(
    queries: *const Queries,
    pass_all: *const Filters,
    world: *const support.World,
    polys: []const PolyEntry,
    random: std.Random,
) Segment {
    var segment: Segment = .{
        .start = randomPlace(queries, pass_all, world, random),
        .end_pos = world.level.groundPoint(random, 0.3),
    };
    const start = &segment.start;
    const entry = entryOf(queries.nav, start.ref) orelse return segment;
    const corner = random.uintLessThan(usize, 6);
    switch (random.uintLessThan(u32, 32)) {
        0, 1 => segment.end_pos = start.pos,
        2, 3, 4, 5 => {
            segment.end_pos = start.pos;
            segment.end_pos[0] += (random.float(f32) - 0.5) * 3.0;
            segment.end_pos[1] += (random.float(f32) - 0.5) * 0.4;
            segment.end_pos[2] += (random.float(f32) - 0.5) * 3.0;
        },
        6 => segment.end_pos = polyVertex(entry, corner),
        7 => segment.end_pos = polyEdgeMiddle(entry, corner),
        8 => segment.end_pos = lerpPoint(start.pos, polyVertex(entry, corner), 3.0),
        9 => segment.end_pos = lerpPoint(start.pos, polyEdgeMiddle(entry, corner), 3.0),
        10, 11 => {
            const angle = random.float(f32) * std.math.tau;
            segment.end_pos = start.pos;
            segment.end_pos[0] += @cos(angle) * 300.0;
            segment.end_pos[2] += @sin(angle) * 300.0;
        },
        12 => start.pos = polyVertex(entry, corner),
        13 => {
            start.pos = polyEdgeMiddle(entry, corner);
            segment.end_pos = polyEdgeMiddle(entry, corner + 2);
        },
        14 => {
            start.pos = polyVertex(entry, corner);
            segment.end_pos = polyVertex(entry, corner + 1);
        },
        15 => if (entryOf(queries.nav, randomNeighbour(entry, random))) |next| {
            segment.end_pos = polyCenter(next);
        },
        16 => {
            const other = polys[random.uintLessThan(usize, polys.len)];
            start.* = .{ .ref = other.ref, .pos = polyCenter(other) };
        },
        17 => start.ref = refusedRef(queries.nav, start.ref, random),
        18 => if (non_finite_checked) {
            start.pos[random.uintLessThan(usize, 3)] = nonFinite(random);
        },
        19 => if (non_finite_checked) {
            segment.end_pos[random.uintLessThan(usize, 3)] = nonFinite(random);
        },
        else => {},
    }
    return segment;
}

test "raycast and raycastPath match the original on random rays" {
    const gpa = testing.allocator;
    for (world_cases, 0..) |case, case_index| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{case.name});
        var world: support.World = undefined;
        try world.init(gpa, case.obj_text, case.config, case.options);
        defer world.deinit();
        const polys = try collectPolys(gpa, &world.nav);
        defer gpa.free(polys);

        var queries: Queries = undefined;
        try queries.init(gpa, &world, roomy_node_count);
        defer queries.deinit();
        var pass_all = try Filters.init();
        defer pass_all.deinit();
        var filters = try Filters.init();
        defer filters.deinit();

        const seed: u64 = 0x9a7b_0400 + @as(u64, case_index);
        var prng: std.Random.DefaultPrng = .init(seed);
        const random = prng.random();
        var result: RayResult = undefined;
        var stopped: u32 = 0;
        var crossed: u32 = 0;
        for (0..fuzz_case_count) |index| {
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, index });
            const segment = randomSegment(&queries, &pass_all, &world, polys, random);
            const start = segment.start;
            const end_pos = segment.end_pos;
            filters.randomize(random);
            const options: RaycastOptions = .{ .use_costs = random.boolean() };

            // The polygon the ray's start was entered from: none, one the
            // start links to, or now and then one the mesh refuses.
            var prev_ref: PolyRef = 0;
            if (entryOf(&world.nav, start.ref)) |entry| {
                switch (random.uintLessThan(u32, 16)) {
                    0, 1, 2, 3, 4 => prev_ref = randomNeighbour(entry, random),
                    5 => prev_ref = refusedRef(&world.nav, entry.ref, random),
                    else => {},
                }
            }

            const full = path_capacity_max;
            try expectSameRay(&queries, &filters, start, end_pos, options, prev_ref, full, &result);
            if (result.status.succeeded) {
                if (result.t <= 1.0) stopped += 1;
                if (result.count > 1) crossed += 1;
            }
            const count = result.count;
            const less = randomCapacity(random, count, full);
            try expectSameRay(&queries, &filters, start, end_pos, options, prev_ref, less, &result);

            const with_normal = random.boolean();
            const capacity: ?usize = switch (random.uintLessThan(u32, 8)) {
                0 => null,
                1 => 0,
                2 => 1,
                3 => count / 2,
                4 => count,
                else => full,
            };
            try expectSameRayPath(
                &queries,
                &filters,
                start,
                end_pos,
                with_normal,
                capacity,
                &result,
            );
        }
        // Rays that stop at walls and rays over several polygons both run.
        try testing.expect(stopped > fuzz_case_count / 16);
        try testing.expect(crossed > fuzz_case_count / 8);
    }
}

/// A target that sends a move into a wall and along it: the point where the
/// original's ray along `segment` meets a wall, shifted along the wall and a
/// little into it. Null when the ray meets no wall.
fn alongWallTarget(
    queries: *const Queries,
    pass_all: *const Filters,
    segment: Segment,
    random: std.Random,
) ?[3]f32 {
    const start = segment.start;
    if (!isFinitePoint(start.pos) or !isFinitePoint(segment.end_pos)) return null;
    if (!reference.ref_nav_mesh_is_valid_poly_ref(queries.reference_nav, start.ref)) return null;

    var t: f32 = 0.0;
    var normal: [3]f32 = @splat(0.0);
    var path: [path_capacity_max]PolyRef = undefined;
    var path_count: i32 = 0;
    const status: Status = @bitCast(reference.ref_query_raycast(
        queries.original,
        start.ref,
        &start.pos,
        &segment.end_pos,
        pass_all.original,
        &t,
        &normal,
        &path,
        &path_count,
        path.len,
    ));
    if (!status.succeeded or status.buffer_too_small) return null;
    if (!(t > 0.0 and t < 1.0)) return null;

    var target = lerpPoint(start.pos, segment.end_pos, t);
    const along = (random.float(f32) - 0.5) * 6.0;
    const into = random.float(f32) * 0.5;
    target[0] += -normal[2] * along - normal[0] * into;
    target[2] += normal[0] * along - normal[2] * into;
    return target;
}

test "moveAlongSurface matches the original on random moves" {
    const gpa = testing.allocator;
    for (world_cases, 0..) |case, case_index| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{case.name});
        var world: support.World = undefined;
        try world.init(gpa, case.obj_text, case.config, case.options);
        defer world.deinit();
        const polys = try collectPolys(gpa, &world.nav);
        defer gpa.free(polys);

        var queries: Queries = undefined;
        try queries.init(gpa, &world, roomy_node_count);
        defer queries.deinit();
        var pass_all = try Filters.init();
        defer pass_all.deinit();
        var filters = try Filters.init();
        defer filters.deinit();

        const seed: u64 = 0x9a7b_0500 + @as(u64, case_index);
        var prng: std.Random.DefaultPrng = .init(seed);
        const random = prng.random();
        var result: MoveResult = undefined;
        var moved: u32 = 0;
        var slid: u32 = 0;
        for (0..fuzz_case_count) |index| {
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, index });
            var segment = randomSegment(&queries, &pass_all, &world, polys, random);
            if (random.uintLessThan(u32, 6) == 0) {
                if (alongWallTarget(&queries, &pass_all, segment, random)) |target| {
                    segment.end_pos = target;
                    slid += 1;
                }
            }
            filters.randomize(random);

            const full = visited_capacity_max;
            try expectSameMove(&queries, &filters, segment.start, segment.end_pos, full, &result);
            if (result.status.succeeded and result.count > 1) moved += 1;
            const short = randomCapacity(random, result.count, full);
            try expectSameMove(&queries, &filters, segment.start, segment.end_pos, short, &result);
        }
        try testing.expect(moved > fuzz_case_count / 8);
        try testing.expect(slid > 0);
        // A move searches in its own small pool and leaves the main one be.
        try expectSamePool(&queries);
    }
}

// ---------------------------------------------------------------------------
// findPath by hand
// ---------------------------------------------------------------------------

/// The world the hand-made cases run in: one tile with off-mesh connections.
const hand_world = world_cases[0];

/// Two places with a whole path of at least `count_min` polygons between
/// them, and that path in `found`.
fn findLongPath(
    queries: *Queries,
    pass_all: *const Filters,
    world: *const support.World,
    random: std.Random,
    count_min: usize,
    found: *PathResult,
) !Ends {
    for (0..512) |_| {
        const ends: Ends = .{
            .start = randomPlace(queries, pass_all, world, random),
            .end = randomPlace(queries, pass_all, world, random),
        };
        if (ends.start.ref == 0 or ends.end.ref == 0) continue;
        try expectSameFindPath(queries, pass_all, ends.start, ends.end, path_capacity_max, found);
        if (!found.status.succeeded or found.status.partial_result) continue;
        if (found.count >= count_min and found.count < path_capacity_max) return ends;
    }
    return error.TestUnexpectedResult;
}

test "findPath refuses the input the original refuses" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, hand_world.obj_text, hand_world.config, hand_world.options);
    defer world.deinit();
    var queries: Queries = undefined;
    try queries.init(gpa, &world, roomy_node_count);
    defer queries.deinit();
    var pass_all = try Filters.init();
    defer pass_all.deinit();

    var prng: std.Random.DefaultPrng = .init(0x9a7b_0600);
    const random = prng.random();
    var found: PathResult = undefined;
    const ends = try findLongPath(&queries, &pass_all, &world, random, 6, &found);
    var result: PathResult = undefined;

    // References the mesh does not hold, at either end and at both.
    const refused_refs = [_]PolyRef{
        0,
        sentinel_ref,
        staleRef(&world.nav, ends.start.ref),
        staleRef(&world.nav, ends.end.ref),
    };
    for (refused_refs) |ref| {
        for ([_]u2{ 1, 2, 3 }) |which| {
            var start = ends.start;
            var end = ends.end;
            if (which & 1 != 0) start.ref = ref;
            if (which & 2 != 0) end.ref = ref;
            for ([_]usize{ 0, 1, 8 }) |capacity| {
                try expectSameFindPath(&queries, &pass_all, start, end, capacity, &result);
                try testing.expect(result.status.failed and result.status.invalid_param);
                try testing.expectEqual(@as(usize, 0), result.count);
            }
        }
    }

    // No room for a path.
    try expectSameFindPath(&queries, &pass_all, ends.start, ends.end, 0, &result);
    try testing.expect(result.status.failed and result.status.invalid_param);
    try expectSameFindPath(&queries, &pass_all, ends.start, ends.start, 0, &result);
    try testing.expect(result.status.failed and result.status.invalid_param);

    // Positions that are not finite, in each coordinate of either end.
    if (non_finite_checked) {
        const bad = [_]f32{ std.math.nan(f32), std.math.inf(f32), -std.math.inf(f32) };
        for (bad) |value| {
            for (0..3) |axis| {
                var start = ends.start;
                start.pos[axis] = value;
                try expectSameFindPath(&queries, &pass_all, start, ends.end, 8, &result);
                try testing.expect(result.status.failed and result.status.invalid_param);
                var end = ends.end;
                end.pos[axis] = value;
                try expectSameFindPath(&queries, &pass_all, ends.start, end, 8, &result);
                try testing.expect(result.status.failed and result.status.invalid_param);
                try expectSameFindPath(&queries, &pass_all, start, start, 8, &result);
                try testing.expect(result.status.failed and result.status.invalid_param);
            }
        }
    }
}

test "findPath fills buffers of every size as the original does" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, hand_world.obj_text, hand_world.config, hand_world.options);
    defer world.deinit();
    var queries: Queries = undefined;
    try queries.init(gpa, &world, roomy_node_count);
    defer queries.deinit();
    var pass_all = try Filters.init();
    defer pass_all.deinit();

    var prng: std.Random.DefaultPrng = .init(0x9a7b_0601);
    const random = prng.random();
    var found: PathResult = undefined;
    var result: PathResult = undefined;
    for (0..8) |_| {
        const ends = try findLongPath(&queries, &pass_all, &world, random, 6, &found);
        const count = found.count;

        // A buffer shorter than the path holds the path's first polygons.
        for ([_]usize{ 1, 2, 3, count / 2, count - 1 }) |capacity| {
            try expectSameFindPath(&queries, &pass_all, ends.start, ends.end, capacity, &result);
            try testing.expect(result.status.succeeded and result.status.buffer_too_small);
            try testing.expectEqual(capacity, result.count);
            try testing.expectEqualSlices(PolyRef, found.path[0..capacity], result.slice());
        }
        // A buffer with room holds all of it.
        for ([_]usize{ count, count + 1, path_capacity_max }) |capacity| {
            try expectSameFindPath(&queries, &pass_all, ends.start, ends.end, capacity, &result);
            try testing.expect(result.status.succeeded and !result.status.buffer_too_small);
            try testing.expectEqualSlices(PolyRef, found.slice(), result.slice());
        }

        // The same polygon at both ends is a path of that polygon, whatever
        // the positions.
        for ([_]usize{ 1, 2, path_capacity_max }) |capacity| {
            var end = ends.end;
            end.ref = ends.start.ref;
            try expectSameFindPath(&queries, &pass_all, ends.start, end, capacity, &result);
            try support.expectStatus(@bitCast(Status.success), result.status);
            try testing.expectEqualSlices(PolyRef, &.{ends.start.ref}, result.slice());
        }
    }
}

/// One sliced search from start to finish on both sides, `budget` iterations
/// an update, compared after every call.
fn expectSameSlicedRun(
    queries: *Queries,
    filters: *const Filters,
    ends: Ends,
    options: FindPathOptions,
    budget: i32,
    with_done: bool,
    capacity: usize,
    result: *PathResult,
) !void {
    var status = try expectSameSlicedInit(queries, filters, ends.start, ends.end, options);
    try expectSamePool(queries);
    var updates: u32 = 0;
    while (status.in_progress) : (updates += 1) {
        try testing.expect(updates < sliced_update_limit);
        status = try expectSameSlicedUpdate(queries, budget, with_done);
    }
    try expectSamePool(queries);
    try expectSameSlicedFinalize(queries, null, capacity, result);
    try expectSameSearchState(queries, result.slice(), ends.start.ref, ends.end.ref);
}

test "searches with few nodes match the original" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, hand_world.obj_text, hand_world.config, hand_world.options);
    defer world.deinit();
    const polys = try collectPolys(gpa, &world.nav);
    defer gpa.free(polys);
    var roomy: Queries = undefined;
    try roomy.init(gpa, &world, roomy_node_count);
    defer roomy.deinit();
    var pass_all = try Filters.init();
    defer pass_all.deinit();

    var prng: std.Random.DefaultPrng = .init(0x9a7b_0610);
    const random = prng.random();
    var result: PathResult = undefined;
    const full = path_capacity_max;

    for ([_]i32{ 4, 5, 8, 16 }) |node_count| {
        errdefer std.debug.print("  with {d} nodes\n", .{node_count});
        var queries: Queries = undefined;
        try queries.init(gpa, &world, node_count);
        defer queries.deinit();
        var out_of_nodes: u32 = 0;
        for (0..96) |index| {
            errdefer std.debug.print("  case {d}\n", .{index});
            const ends = randomEnds(&queries, &pass_all, &world, polys, random);
            try expectSameFindPath(&queries, &pass_all, ends.start, ends.end, full, &result);
            if (result.status.out_of_nodes) out_of_nodes += 1;
            const less = randomCapacity(random, result.count, full);
            try expectSameFindPath(&queries, &pass_all, ends.start, ends.end, less, &result);

            const options: FindPathOptions = .{ .any_angle = index % 2 == 1 };
            try expectSameSlicedRun(&queries, &pass_all, ends, options, 3, true, full, &result);
        }
        try testing.expect(out_of_nodes > 0);
    }

    // With fewer than four nodes the original's pool has no hash buckets and
    // reads outside its table, so the port runs alone: it must come back
    // with a path that starts at the start and has no more polygons than
    // the pool has nodes.
    for ([_]i32{ 1, 2, 3 }) |node_count| {
        errdefer std.debug.print("  with {d} nodes\n", .{node_count});
        var port: NavMeshQuery = .empty;
        defer port.deinit(gpa);
        try testing.expect(port.init(gpa, &world.nav, node_count).succeeded);
        for (0..96) |_| {
            const start = try somePlace(&roomy, &pass_all, &world, random);
            const end = try somePlace(&roomy, &pass_all, &world, random);
            const filter = &pass_all.port;

            var path: [16]PolyRef = @splat(sentinel_ref);
            var count: usize = 77;
            const found = port.findPath(
                start.ref,
                end.ref,
                start.pos,
                end.pos,
                filter,
                &path,
                &count,
            );
            try testing.expect(found.succeeded);
            try testing.expect(count >= 1 and count <= @as(usize, @intCast(node_count)));
            try testing.expectEqual(start.ref, path[0]);
            for (path[0..count]) |ref| try testing.expect(world.nav.isValidPolyRef(ref));
            for (path[count..]) |ref| try testing.expectEqual(sentinel_ref, ref);

            var status = port.initSlicedFindPath(
                start.ref,
                end.ref,
                start.pos,
                end.pos,
                filter,
                .{},
            );
            var updates: u32 = 0;
            while (status.in_progress) : (updates += 1) {
                try testing.expect(updates < sliced_update_limit);
                status = port.updateSlicedFindPath(2, null);
            }
            path = @splat(sentinel_ref);
            count = 77;
            const finalized = port.finalizeSlicedFindPath(&path, &count);
            try testing.expect(finalized.succeeded);
            try testing.expect(count >= 1 and count <= @as(usize, @intCast(node_count)));
            try testing.expectEqual(start.ref, path[0]);
            for (path[count..]) |ref| try testing.expectEqual(sentinel_ref, ref);
        }
    }
}

fn containsRef(refs: []const PolyRef, ref: PolyRef) bool {
    for (refs) |other| {
        if (other == ref) return true;
    }
    return false;
}

/// `count` searches between random places under `filters`: `findPath` into
/// a large buffer and a smaller one, and a sliced search.
fn expectSameRandomPaths(
    queries: *Queries,
    pass_all: *const Filters,
    filters: *const Filters,
    world: *const support.World,
    random: std.Random,
    count: usize,
) !void {
    var result: PathResult = undefined;
    const full = path_capacity_max;
    for (0..count) |index| {
        errdefer std.debug.print("  pair {d}\n", .{index});
        const ends: Ends = .{
            .start = randomPlace(queries, pass_all, world, random),
            .end = randomPlace(queries, pass_all, world, random),
        };
        try expectSameFindPath(queries, filters, ends.start, ends.end, full, &result);
        const less = randomCapacity(random, result.count, full);
        try expectSameFindPath(queries, filters, ends.start, ends.end, less, &result);
        const options: FindPathOptions = .{ .any_angle = index % 2 == 1 };
        try expectSameSlicedRun(queries, filters, ends, options, 64, false, full, &result);
    }
}

test "findPath under filters matches the original" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, hand_world.obj_text, hand_world.config, hand_world.options);
    defer world.deinit();
    var queries: Queries = undefined;
    try queries.init(gpa, &world, roomy_node_count);
    defer queries.deinit();
    var pass_all = try Filters.init();
    defer pass_all.deinit();
    var filters = try Filters.init();
    defer filters.deinit();

    var prng: std.Random.DefaultPrng = .init(0x9a7b_0620);
    const random = prng.random();
    var found: PathResult = undefined;
    var result: PathResult = undefined;
    const full = path_capacity_max;

    // Every area at its own cost.
    for (0..filters.port.area_cost.len) |area| {
        filters.setAreaCost(area, 1.0 + @as(f32, @floatFromInt(area % 7)) * 1.5);
    }
    try expectSameRandomPaths(&queries, &pass_all, &filters, &world, random, 64);

    // One area far dearer than the rest, and one almost free.
    for (0..6) |area| {
        filters.reset();
        filters.setAreaCost(area, 500.0);
        try expectSameRandomPaths(&queries, &pass_all, &filters, &world, random, 24);
        filters.setAreaCost(area, 0.001);
        try expectSameRandomPaths(&queries, &pass_all, &filters, &world, random, 8);
    }

    // Flags that cut polygons out of the mesh.
    for ([_]u16{ 2, 4, 8, 2 | 4, 2 | 4 | 8 }) |exclude| {
        filters.reset();
        filters.setFlags(0xffff, exclude);
        try expectSameRandomPaths(&queries, &pass_all, &filters, &world, random, 24);
    }
    // Flags that pass only some of the polygons.
    for ([_]u16{ 1, 2, 4, 8, 2 | 8 }) |include| {
        filters.reset();
        filters.setFlags(include, 0);
        try expectSameRandomPaths(&queries, &pass_all, &filters, &world, random, 24);
    }

    // A filter that passes nothing: the path is the start polygon alone.
    filters.reset();
    filters.setFlags(0x8000, 0);
    for (0..32) |_| {
        const start = try somePlace(&queries, &pass_all, &world, random);
        const end = try somePlace(&queries, &pass_all, &world, random);
        if (start.ref == end.ref) continue;
        try expectSameFindPath(&queries, &filters, start, end, full, &result);
        try testing.expect(result.status.succeeded and result.status.partial_result);
        try testing.expectEqualSlices(PolyRef, &.{start.ref}, result.slice());
    }

    // A polygon in the middle of a found path, cut out by its flags on both
    // meshes: no path under the filter holds it.
    const ends = try findLongPath(&queries, &pass_all, &world, random, 6, &found);
    const cut = found.path[found.count / 2];
    try support.expectStatus(
        reference.ref_nav_mesh_set_poly_flags(world.reference_nav, cut, 0x4000),
        world.nav.setPolyFlags(cut, 0x4000),
    );
    filters.reset();
    filters.setFlags(0xffff, 0x4000);
    try expectSameFindPath(&queries, &filters, ends.start, ends.end, full, &result);
    try testing.expect(!containsRef(result.slice(), cut));
    try expectSameRandomPaths(&queries, &pass_all, &filters, &world, random, 24);

    // The same polygon passable again, in an area of its own that costs far
    // more than the rest.
    try support.expectStatus(
        reference.ref_nav_mesh_set_poly_flags(world.reference_nav, cut, 1),
        world.nav.setPolyFlags(cut, 1),
    );
    try support.expectStatus(
        reference.ref_nav_mesh_set_poly_area(world.reference_nav, cut, 63),
        world.nav.setPolyArea(cut, 63),
    );
    filters.reset();
    filters.setAreaCost(63, 500.0);
    try expectSameFindPath(&queries, &filters, ends.start, ends.end, full, &result);
    try expectSameRandomPaths(&queries, &pass_all, &filters, &world, random, 24);
}

/// The polygons an off-mesh connection's two ends link to; zero where an
/// end found no polygon.
fn connectionGrounds(entry: PolyEntry) [2]PolyRef {
    var grounds: [2]PolyRef = .{ 0, 0 };
    var link_index = entry.poly.first_link;
    while (link_index != detour.nav_mesh.null_link) {
        const link = entry.tile.links[link_index];
        if (link.edge < 2) grounds[link.edge] = link.ref;
        link_index = link.next;
    }
    return grounds;
}

test "paths over off-mesh connections match the original in both directions" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, hand_world.obj_text, hand_world.config, hand_world.options);
    defer world.deinit();
    const polys = try collectPolys(gpa, &world.nav);
    defer gpa.free(polys);
    var queries: Queries = undefined;
    try queries.init(gpa, &world, roomy_node_count);
    defer queries.deinit();
    var pass_all = try Filters.init();
    defer pass_all.deinit();

    var found: PathResult = undefined;
    var sliced: PathResult = undefined;
    var straight: StraightResult = undefined;
    var ray: RayResult = undefined;
    var move: MoveResult = undefined;
    const full = path_capacity_max;
    var linked: u32 = 0;
    var crossed: u32 = 0;
    var flagged: u32 = 0;
    for (polys) |entry| {
        if (entry.poly.getType() != .off_mesh_connection) continue;
        errdefer std.debug.print("  at connection polygon {x}\n", .{entry.ref});
        const grounds = connectionGrounds(entry);
        const first = entryOf(&world.nav, grounds[0]) orelse continue;
        const second = entryOf(&world.nav, grounds[1]) orelse continue;
        linked += 1;

        // From and to the middles of the polygons at the two ends, and from
        // and to the connection's own end points, each in both directions.
        // A one-way connection gives a path over the ground on the way back.
        const middles: Ends = .{
            .start = .{ .ref = first.ref, .pos = polyCenter(first) },
            .end = .{ .ref = second.ref, .pos = polyCenter(second) },
        };
        const points: Ends = .{
            .start = .{ .ref = first.ref, .pos = polyVertex(entry, 0) },
            .end = .{ .ref = second.ref, .pos = polyVertex(entry, 1) },
        };
        const ways = [_]Ends{
            middles,
            .{ .start = middles.end, .end = middles.start },
            points,
            .{ .start = points.end, .end = points.start },
        };
        for (ways) |ends| {
            try expectSameFindPath(&queries, &pass_all, ends.start, ends.end, full, &found);
            try testing.expect(found.status.succeeded);
            if (containsRef(found.slice(), entry.ref)) crossed += 1;

            for (straight_options) |options| {
                for (StraightBuffers.all) |buffers| {
                    for ([_]usize{ straight_capacity_max, 1, 2, 3 }) |capacity| {
                        try expectSameStraightPath(
                            &queries,
                            ends.start.pos,
                            ends.end.pos,
                            found.slice(),
                            capacity,
                            buffers,
                            options,
                            &straight,
                        );
                        if (!buffers.flags) continue;
                        for (straight.flags[0..straight.count]) |flags| {
                            if (flags.off_mesh_connection) flagged += 1;
                        }
                    }
                }
            }

            for ([_]bool{ false, true }) |any_angle| {
                const options: FindPathOptions = .{ .any_angle = any_angle };
                try expectSameSlicedRun(&queries, &pass_all, ends, options, 7, true, full, &sliced);
            }

            // The connection's own polygon as an end of a search, and as the
            // start of a ray and of a move.
            const on: Place = .{ .ref = entry.ref, .pos = polyEdgeMiddle(entry, 0) };
            try expectSameFindPath(&queries, &pass_all, on, ends.end, full, &found);
            try expectSameFindPath(&queries, &pass_all, ends.start, on, full, &found);
            for ([_]RaycastOptions{ .{}, .{ .use_costs = true } }) |options| {
                const target = ends.end.pos;
                try expectSameRay(&queries, &pass_all, on, target, options, 0, full, &ray);
                try expectSameRay(&queries, &pass_all, ends.start, on.pos, options, 0, full, &ray);
            }
            const room = visited_capacity_max;
            try expectSameMove(&queries, &pass_all, on, ends.end.pos, room, &move);
            try expectSameMove(&queries, &pass_all, ends.start, on.pos, room, &move);
        }
    }
    // Without a linked connection, a path over one and a straight path
    // point that marks one, the test compared nothing it is here for.
    if (linked == 0 or crossed == 0 or flagged == 0) {
        std.debug.print(
            "  {d} connections linked, {d} paths over one, {d} marked points\n",
            .{ linked, crossed, flagged },
        );
        return error.TestUnexpectedResult;
    }
}

// ---------------------------------------------------------------------------
// The sliced search by hand
// ---------------------------------------------------------------------------

test "the sliced search matches the original at every budget" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, hand_world.obj_text, hand_world.config, hand_world.options);
    defer world.deinit();
    var queries: Queries = undefined;
    try queries.init(gpa, &world, roomy_node_count);
    defer queries.deinit();
    var pass_all = try Filters.init();
    defer pass_all.deinit();

    var prng: std.Random.DefaultPrng = .init(0x9a7b_0630);
    const random = prng.random();
    var found: PathResult = undefined;
    var whole: PathResult = undefined;
    var result: PathResult = undefined;
    const full = path_capacity_max;
    const filters = &pass_all;
    for (0..4) |_| {
        const ends = try findLongPath(&queries, filters, &world, random, 6, &found);
        for ([_]bool{ false, true }) |any_angle| {
            errdefer std.debug.print("  any angle: {}\n", .{any_angle});
            const options: FindPathOptions = .{ .any_angle = any_angle };
            try expectSameSlicedRun(&queries, filters, ends, options, 64, true, full, &whole);
            if (!any_angle) {
                // Polygon by polygon, the search gets from end to end.
                try testing.expect(whole.status.succeeded and !whole.status.partial_result);
                try testing.expect(whole.count >= 2 and whole.count < full);
                try testing.expectEqual(ends.start.ref, whole.path[0]);
                try testing.expectEqual(ends.end.ref, whole.path[whole.count - 1]);
            }

            // Few iterations an update, many, and no limit, with and
            // without the count of iterations done: the path is the same
            // however the search is cut into updates.
            for (sliced_budgets) |budget| {
                errdefer std.debug.print("  budget {d}\n", .{budget});
                for ([_]bool{ false, true }) |with_done| {
                    try expectSameSlicedRun(
                        &queries,
                        filters,
                        ends,
                        options,
                        budget,
                        with_done,
                        full,
                        &result,
                    );
                    try support.expectStatus(@bitCast(whole.status), result.status);
                    try testing.expectEqualSlices(PolyRef, whole.slice(), result.slice());
                }
            }

            // An update with no iterations to spend leaves the search as it
            // is, still in progress.
            var status = try expectSameSlicedInit(&queries, filters, ends.start, ends.end, options);
            try testing.expect(status.in_progress);
            for ([_]i32{ 0, -1, 0 }) |budget| {
                status = try expectSameSlicedUpdate(&queries, budget, true);
                try testing.expect(status.in_progress);
                status = try expectSameSlicedUpdate(&queries, budget, false);
                try testing.expect(status.in_progress);
            }
            try expectSamePool(&queries);

            // The finished path into buffers of every size.
            const count = whole.count;
            const capacities = [_]usize{ 1, 2, 3, count / 2, count -| 1, count, count + 1, full };
            for (capacities) |capacity| {
                errdefer std.debug.print("  buffer of {d}\n", .{capacity});
                try expectSameSlicedRun(
                    &queries,
                    filters,
                    ends,
                    options,
                    64,
                    true,
                    capacity,
                    &result,
                );
                if (any_angle) continue;
                // The path's first part fills the room there is.
                try testing.expectEqual(@min(capacity, count), result.count);
                try testing.expectEqual(capacity <= count, result.status.buffer_too_small);
                try testing.expectEqualSlices(PolyRef, whole.path[0..result.count], result.slice());
            }
        }
    }
}

/// Starts a search on both sides and runs up to `updates` updates of
/// `budget` iterations each. Gives the last status.
fn expectSameSlicedStart(
    queries: *Queries,
    filters: *const Filters,
    ends: Ends,
    options: FindPathOptions,
    budget: i32,
    updates: u32,
) !Status {
    var status = try expectSameSlicedInit(queries, filters, ends.start, ends.end, options);
    var done: u32 = 0;
    while (status.in_progress and done < updates) : (done += 1) {
        status = try expectSameSlicedUpdate(queries, budget, true);
    }
    try expectSamePool(queries);
    return status;
}

/// The updates of one iteration each that a search takes on both sides from
/// its start to its end. Leaves no search behind.
fn countSlicedUpdates(
    queries: *Queries,
    filters: *const Filters,
    ends: Ends,
    options: FindPathOptions,
) !u32 {
    var status = try expectSameSlicedInit(queries, filters, ends.start, ends.end, options);
    var updates: u32 = 0;
    while (status.in_progress) : (updates += 1) {
        try testing.expect(updates < sliced_update_limit);
        status = try expectSameSlicedUpdate(queries, 1, true);
    }
    var result: PathResult = undefined;
    try expectSameSlicedFinalize(queries, null, path_capacity_max, &result);
    return updates;
}

test "sliced search calls in any order match the original" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, hand_world.obj_text, hand_world.config, hand_world.options);
    defer world.deinit();
    var queries: Queries = undefined;
    try queries.init(gpa, &world, roomy_node_count);
    defer queries.deinit();
    var pass_all = try Filters.init();
    defer pass_all.deinit();
    const filters = &pass_all;

    var found: PathResult = undefined;
    var result: PathResult = undefined;
    const full = path_capacity_max;
    const untouched = [_]PolyRef{ sentinel_ref, 0, sentinel_ref };
    const nothing: []const PolyRef = untouched[0..0];

    // No search started: an update does nothing and a finalize finds no
    // search, whatever it is given.
    _ = try expectSameSlicedUpdate(&queries, 8, true);
    _ = try expectSameSlicedUpdate(&queries, 8, false);
    try expectSameSlicedFinalize(&queries, null, 0, &result);
    try expectSameSlicedFinalize(&queries, null, full, &result);
    try expectSameSlicedFinalize(&queries, &untouched, full, &result);
    try expectSameSlicedFinalize(&queries, nothing, full, &result);
    try expectSameSlicedFinalize(&queries, &untouched, 0, &result);
    try expectSamePool(&queries);

    // Two ends so far apart that the search still runs after the most
    // updates the cases below give it before they finalize.
    var prng: std.Random.DefaultPrng = .init(0x9a7b_0640);
    const random = prng.random();
    const early_updates = [_]u32{ 0, 1, 2, 5, 12 };
    var ends: Ends = undefined;
    var tries: u32 = 0;
    while (true) : (tries += 1) {
        try testing.expect(tries < 64);
        ends = try findLongPath(&queries, filters, &world, random, 8, &found);
        const plain = try countSlicedUpdates(&queries, filters, ends, .{});
        const shortcut = try countSlicedUpdates(&queries, filters, ends, .{ .any_angle = true });
        if (@min(plain, shortcut) > early_updates[early_updates.len - 1]) break;
    }
    const back: Ends = .{ .start = ends.end, .end = ends.start };
    const whole: []const PolyRef = found.slice();

    // A search refused at its start stays failed through updates, and the
    // finalize that reports it leaves no search behind.
    var refused = [_]Ends{ ends, ends, ends, ends, ends };
    refused[0].start.ref = 0;
    refused[1].end.ref = staleRef(&world.nav, ends.end.ref);
    refused[2].start.ref = sentinel_ref;
    refused[2].end.ref = sentinel_ref;
    if (non_finite_checked) {
        refused[3].start.pos[1] = std.math.nan(f32);
        refused[4].end.pos[0] = std.math.inf(f32);
    } else {
        refused[3].end.ref = 0;
        refused[4].start.ref = staleRef(&world.nav, ends.start.ref);
    }
    for (refused, 0..) |bad, index| {
        errdefer std.debug.print("  refused search {d}\n", .{index});
        for ([_]?[]const PolyRef{ null, whole, &untouched }) |existing| {
            const options: FindPathOptions = .{ .any_angle = index % 2 == 1 };
            const status = try expectSameSlicedStart(&queries, filters, bad, options, 8, 2);
            try testing.expect(status.failed);
            try expectSameSlicedFinalize(&queries, existing, full, &result);
            try testing.expect(result.status.failed);
            _ = try expectSameSlicedUpdate(&queries, 8, true);
            try expectSameSlicedFinalize(&queries, existing, full, &result);
        }
    }

    // The same polygon at both ends: over at the start.
    var same = ends;
    same.end.ref = ends.start.ref;
    for ([_]?[]const PolyRef{ null, whole, &untouched }) |existing| {
        const status = try expectSameSlicedStart(&queries, filters, same, .{}, 8, 2);
        try support.expectStatus(@bitCast(Status.success), status);
        try expectSameSlicedFinalize(&queries, existing, full, &result);
        try testing.expectEqualSlices(PolyRef, &.{ends.start.ref}, result.slice());
    }

    for ([_]bool{ false, true }) |any_angle| {
        errdefer std.debug.print("  any angle: {}\n", .{any_angle});
        const options: FindPathOptions = .{ .any_angle = any_angle };

        // Finalized before the search ends, after no update and after a
        // few: the path so far, whole or along an existing path.
        for (early_updates) |updates| {
            errdefer std.debug.print("  finalized after {d} updates\n", .{updates});
            for ([_]?[]const PolyRef{ null, whole, whole[0..3], &untouched }) |existing| {
                for ([_]usize{ full, 1, 2 }) |capacity| {
                    const status = try expectSameSlicedStart(
                        &queries,
                        filters,
                        ends,
                        options,
                        1,
                        updates,
                    );
                    try testing.expect(status.in_progress);
                    try expectSameSlicedFinalize(&queries, existing, capacity, &result);
                    try testing.expect(result.status.succeeded);
                    const so_far = result.slice();
                    try expectSameSearchState(&queries, so_far, ends.start.ref, ends.end.ref);
                    // The finalize ended the search.
                    _ = try expectSameSlicedUpdate(&queries, 8, true);
                }
            }
        }

        // A second start while a search runs replaces it.
        _ = try expectSameSlicedStart(&queries, filters, ends, options, 3, 2);
        try expectSameSlicedRun(&queries, filters, back, options, 7, true, full, &result);
        // A refused start while a search runs leaves a failed search.
        _ = try expectSameSlicedStart(&queries, filters, ends, options, 3, 2);
        const status = try expectSameSlicedStart(&queries, filters, refused[0], options, 3, 2);
        try testing.expect(status.failed);
        try expectSameSlicedFinalize(&queries, null, full, &result);
        try testing.expect(result.status.failed);

        // A finalize refused for its buffers leaves the search running; the
        // next one ends it.
        const unlimited = std.math.maxInt(i32);
        _ = try expectSameSlicedStart(&queries, filters, ends, options, unlimited, 1);
        try expectSameSlicedFinalize(&queries, null, 0, &result);
        try expectSameSlicedFinalize(&queries, nothing, full, &result);
        try expectSameSlicedFinalize(&queries, whole, 0, &result);
        try testing.expect(result.status.failed and result.status.invalid_param);
        try expectSameSlicedFinalize(&queries, null, full, &result);
        try testing.expect(result.status.succeeded and result.count >= 2);

        // A finished search finalized along existing paths: the whole path
        // it found, parts of it, polygons it never saw, and paths longer
        // than the room for the result.
        const existing_paths = [_][]const PolyRef{
            whole,
            whole[0 .. whole.len / 2],
            whole[whole.len / 2 ..],
            whole[0..1],
            whole[whole.len - 1 ..],
            &untouched,
            untouched[1..2],
        };
        for (existing_paths, 0..) |existing, index| {
            errdefer std.debug.print("  existing path {d}\n", .{index});
            for ([_]usize{ full, whole.len, whole.len - 1, 3, 2, 1 }) |capacity| {
                _ = try expectSameSlicedStart(&queries, filters, ends, options, unlimited, 1);
                try expectSameSlicedFinalize(&queries, existing, capacity, &result);
                try testing.expect(result.status.succeeded);
                try expectSameSearchState(&queries, result.slice(), ends.start.ref, ends.end.ref);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// findStraightPath by hand
// ---------------------------------------------------------------------------

/// One straight path into a large buffer, left in `whole`, then into every
/// buffer size from none to one more than its points, with each set of
/// optional buffers. Gives how often the original wrote past its buffer.
fn expectSameStraightPathSizes(
    queries: *const Queries,
    start_pos: [3]f32,
    end_pos: [3]f32,
    path: []const PolyRef,
    options: StraightPathOptions,
    whole: *StraightResult,
) !u32 {
    var result: StraightResult = undefined;
    const large = straight_capacity_max;
    try expectSameStraightPath(queries, start_pos, end_pos, path, large, .{}, options, whole);
    // A whole path that fits says what every other buffer must hold.
    const fits = whole.status.succeeded and !whole.status.buffer_too_small;
    var overruns: u32 = 0;
    for (StraightBuffers.all) |buffers| {
        for (0..@min(whole.count + 2, large + 1)) |capacity| {
            errdefer std.debug.print("  buffer of {d} points\n", .{capacity});
            try expectSameStraightPath(
                queries,
                start_pos,
                end_pos,
                path,
                capacity,
                buffers,
                options,
                &result,
            );
            if (result.overran) overruns += 1;
            if (capacity == 0) {
                try testing.expect(result.status.failed and result.status.invalid_param);
                continue;
            }
            if (!fits) continue;
            const count = @min(capacity, whole.count);
            try testing.expectEqual(count, result.count);
            try testing.expectEqual(capacity <= whole.count, result.status.buffer_too_small);
            try expectPoints(whole.points[0..count], result.points[0..count]);
        }
    }
    return overruns;
}

test "findStraightPath fills buffers of every size as the original does" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, hand_world.obj_text, hand_world.config, hand_world.options);
    defer world.deinit();
    var queries: Queries = undefined;
    try queries.init(gpa, &world, roomy_node_count);
    defer queries.deinit();
    var pass_all = try Filters.init();
    defer pass_all.deinit();

    var prng: std.Random.DefaultPrng = .init(0x9a7b_0650);
    const random = prng.random();
    var found: PathResult = undefined;
    var whole: StraightResult = undefined;
    var cornered: u32 = 0;
    for (0..4) |_| {
        const ends = try findLongPath(&queries, &pass_all, &world, random, 8, &found);
        const path = found.slice();
        const middle = entryOf(&world.nav, path[path.len / 2]) orelse
            return error.TestUnexpectedResult;
        var above = ends.start.pos;
        above[1] += 40.0;
        var below = ends.end.pos;
        below[1] -= 40.0;

        // The ends the path was found for, then ends away from them: the
        // two swapped, far above and below, both at the start, both in the
        // middle of the path, and anywhere in the world.
        const position_pairs = [_][2][3]f32{
            .{ ends.start.pos, ends.end.pos },
            .{ ends.end.pos, ends.start.pos },
            .{ above, below },
            .{ ends.start.pos, ends.start.pos },
            .{ polyCenter(middle), polyCenter(middle) },
            .{ world.level.groundPoint(random, 0.3), world.level.groundPoint(random, 0.3) },
        };
        for (position_pairs, 0..) |pair, pair_index| {
            errdefer std.debug.print("  position pair {d}\n", .{pair_index});
            for (straight_options) |options| {
                _ = try expectSameStraightPathSizes(
                    &queries,
                    pair[0],
                    pair[1],
                    path,
                    options,
                    &whole,
                );
                // Every polygon of the path is in the mesh and links to the
                // next, so there is a path, with its start point at least.
                try testing.expect(whole.status.succeeded and !whole.status.partial_result);
                try testing.expect(whole.count >= 1);
                if (pair_index == 0 and whole.count > 2) cornered += 1;
            }
        }
    }
    // Paths with points between their two ends ran.
    try testing.expect(cornered > 0);
}

test "findStraightPath over short and odd corridors matches the original" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, hand_world.obj_text, hand_world.config, hand_world.options);
    defer world.deinit();
    var queries: Queries = undefined;
    try queries.init(gpa, &world, roomy_node_count);
    defer queries.deinit();
    var pass_all = try Filters.init();
    defer pass_all.deinit();

    var prng: std.Random.DefaultPrng = .init(0x9a7b_0651);
    const random = prng.random();
    var found: PathResult = undefined;
    var whole: StraightResult = undefined;
    var reversed: [path_capacity_max]PolyRef = undefined;
    for (0..4) |_| {
        const ends = try findLongPath(&queries, &pass_all, &world, random, 8, &found);
        const path = found.slice();
        const start_pos = ends.start.pos;
        const end_pos = ends.end.pos;

        // No polygon, one, two, the same one twice, two walked back and
        // forth, and the whole path backwards.
        const twice = [_]PolyRef{ path[0], path[0] };
        const back_and_forth = [_]PolyRef{ path[0], path[1], path[0], path[1], path[0] };
        @memcpy(reversed[0..path.len], path);
        std.mem.reverse(PolyRef, reversed[0..path.len]);
        const corridors = [_][]const PolyRef{
            path[0..0],
            path[0..1],
            path[path.len - 1 ..],
            path[0..2],
            path[path.len - 2 ..],
            &twice,
            &back_and_forth,
            reversed[0..path.len],
        };
        for (corridors, 0..) |corridor, corridor_index| {
            errdefer std.debug.print("  corridor {d}\n", .{corridor_index});
            for (straight_options) |options| {
                _ = try expectSameStraightPathSizes(
                    &queries,
                    start_pos,
                    end_pos,
                    corridor,
                    options,
                    &whole,
                );
                if (corridor.len == 0) {
                    try testing.expect(whole.status.failed and whole.status.invalid_param);
                    try testing.expectEqual(@as(usize, 0), whole.count);
                } else {
                    try testing.expect(whole.status.succeeded and whole.count >= 1);
                }
            }
        }

        // One polygon with both ends at one point inside it: the start
        // point becomes the end point.
        const first = entryOf(&world.nav, path[0]) orelse return error.TestUnexpectedResult;
        const center = polyCenter(first);
        for (straight_options) |options| {
            _ = try expectSameStraightPathSizes(
                &queries,
                center,
                center,
                path[0..1],
                options,
                &whole,
            );
            try testing.expectEqual(@as(usize, 1), whole.count);
            try testing.expect(whole.flags[0].end);
            try testing.expectEqual(@as(PolyRef, 0), whole.refs[0]);
        }

        // Positions that are not finite, at either end.
        if (non_finite_checked) {
            const bad = [_]f32{ std.math.nan(f32), std.math.inf(f32), -std.math.inf(f32) };
            for (bad, 0..) |value, index| {
                var start_bad = start_pos;
                start_bad[index] = value;
                var end_bad = end_pos;
                end_bad[2 - index] = value;
                const pairs = [_][2][3]f32{
                    .{ start_bad, end_pos },
                    .{ start_pos, end_bad },
                    .{ start_bad, end_bad },
                };
                for (pairs) |pair| {
                    _ = try expectSameStraightPathSizes(
                        &queries,
                        pair[0],
                        pair[1],
                        path,
                        .{ .all_crossings = true },
                        &whole,
                    );
                    try testing.expect(whole.status.failed and whole.status.invalid_param);
                    try testing.expectEqual(@as(usize, 0), whole.count);
                }
            }
        }
    }
}

test "findStraightPath over broken corridors matches the original" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, hand_world.obj_text, hand_world.config, hand_world.options);
    defer world.deinit();
    var queries: Queries = undefined;
    try queries.init(gpa, &world, roomy_node_count);
    defer queries.deinit();
    var pass_all = try Filters.init();
    defer pass_all.deinit();

    var prng: std.Random.DefaultPrng = .init(0x9a7b_0652);
    const random = prng.random();
    var found: PathResult = undefined;
    var whole: StraightResult = undefined;
    var buffer: [path_capacity_max]PolyRef = undefined;
    var stopped_short: u32 = 0;
    var overruns: u32 = 0;
    for (0..4) |_| {
        const ends = try findLongPath(&queries, &pass_all, &world, random, 8, &found);
        const path = found.slice();
        const last = path.len - 1;

        // Where the corridor breaks: at its first polygon, early, in the
        // middle, late and at its last polygon.
        for ([_]usize{ 0, 1, 2, path.len / 2, last - 1, last }) |at| {
            // What stands in the polygon's place: nothing, a reference no
            // mesh holds, the polygon under a stale salt, and a polygon from
            // further along the path, which links to neither neighbour.
            const replacements = [_]PolyRef{
                0,
                sentinel_ref,
                staleRef(&world.nav, path[at]),
                path[(at + path.len / 2) % path.len],
            };
            for (replacements, 0..) |replacement, kind| {
                errdefer std.debug.print(
                    "  polygon {d} of {d} replaced, kind {d}\n",
                    .{ at, path.len, kind },
                );
                @memcpy(buffer[0..path.len], path);
                buffer[at] = replacement;
                const corridor: []const PolyRef = buffer[0..path.len];
                const refused = kind < 3;

                for (straight_options) |options| {
                    overruns += try expectSameStraightPathSizes(
                        &queries,
                        ends.start.pos,
                        ends.end.pos,
                        corridor,
                        options,
                        &whole,
                    );
                    if (!refused) continue;
                    if (at == 0 or at == last) {
                        // Without its first or its last polygon a corridor
                        // has no start or no end to clamp to.
                        try testing.expect(whole.status.failed and whole.status.invalid_param);
                        try testing.expectEqual(@as(usize, 0), whole.count);
                    } else if (!whole.status.buffer_too_small) {
                        // The path stops at the last polygon before the
                        // break.
                        try testing.expect(whole.status.succeeded);
                        try testing.expect(whole.status.partial_result);
                        try testing.expect(whole.count >= 1);
                        stopped_short += 1;
                    }
                }
            }
        }
    }
    try testing.expect(stopped_short > 0);
    // The original's write past a full buffer is among the cases: a corridor
    // that breaks right after its crossing points filled the buffer.
    try testing.expect(overruns > 0);
}

// ---------------------------------------------------------------------------
// raycast and moveAlongSurface by hand
// ---------------------------------------------------------------------------

const polygon_target_count_max = detour.nav_mesh.verts_per_polygon * 6 + 1;

/// The points a ray or a move from a polygon's middle heads for, written
/// into `buffer`: each vertex and each edge's middle, the points half way to
/// them and the points far past them, and the polygon's middle itself.
fn polygonTargets(
    entry: PolyEntry,
    buffer: *[polygon_target_count_max][3]f32,
) []const [3]f32 {
    const center = polyCenter(entry);
    var count: usize = 0;
    for (0..entry.poly.vert_count) |corner| {
        const vertex = polyVertex(entry, corner);
        const middle = polyEdgeMiddle(entry, corner);
        const points = [_][3]f32{
            vertex,
            middle,
            lerpPoint(center, vertex, 0.5),
            lerpPoint(center, middle, 0.5),
            lerpPoint(center, vertex, 3.0),
            lerpPoint(center, middle, 3.0),
        };
        @memcpy(buffer[count..][0..points.len], &points);
        count += points.len;
    }
    buffer[count] = center;
    return buffer[0 .. count + 1];
}

/// Polygons the tests of every vertex and edge start from, at most.
const sampled_polygon_count = 32;

test "rays and moves from polygon middles match the original at every vertex and edge" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, hand_world.obj_text, hand_world.config, hand_world.options);
    defer world.deinit();
    const polys = try collectPolys(gpa, &world.nav);
    defer gpa.free(polys);
    var queries: Queries = undefined;
    try queries.init(gpa, &world, roomy_node_count);
    defer queries.deinit();
    var pass_all = try Filters.init();
    defer pass_all.deinit();

    var prng: std.Random.DefaultPrng = .init(0x9a7b_0660);
    const random = prng.random();
    var targets_buffer: [polygon_target_count_max][3]f32 = undefined;
    var ray: RayResult = undefined;
    var move: MoveResult = undefined;
    var reached: u32 = 0;
    var stopped: u32 = 0;
    var crossed: u32 = 0;
    var moved: u32 = 0;
    const stride = @max(polys.len / sampled_polygon_count, 1);
    var poly_index: usize = 0;
    while (poly_index < polys.len) : (poly_index += stride) {
        const entry = polys[poly_index];
        if (entry.poly.getType() != .ground) continue;
        errdefer std.debug.print("  from polygon {x}\n", .{entry.ref});
        const start: Place = .{ .ref = entry.ref, .pos = polyCenter(entry) };
        const neighbour = randomNeighbour(entry, random);

        for (polygonTargets(entry, &targets_buffer), 0..) |target, target_index| {
            errdefer std.debug.print("  target {d}\n", .{target_index});
            const filters = &pass_all;
            for ([_]RaycastOptions{ .{}, .{ .use_costs = true } }) |options| {
                const full = path_capacity_max;
                try expectSameRay(&queries, filters, start, target, options, 0, full, &ray);
                try testing.expect(ray.status.succeeded);
                const count = ray.count;
                if (ray.t == std.math.floatMax(f32)) {
                    reached += 1;
                } else if (ray.t < 1.0) {
                    stopped += 1;
                }
                if (count > 1) crossed += 1;

                // The same ray with its start entered from a neighbour, and
                // into path buffers with no room, too little and just enough.
                try expectSameRay(&queries, filters, start, target, options, neighbour, full, &ray);
                for ([_]usize{ 0, 1, count -| 1, count }) |capacity| {
                    try expectSameRay(
                        &queries,
                        filters,
                        start,
                        target,
                        options,
                        0,
                        capacity,
                        &ray,
                    );
                }
            }
            for ([_]bool{ false, true }) |with_normal| {
                for ([_]?usize{ null, 0, 1, path_capacity_max }) |capacity| {
                    try expectSameRayPath(
                        &queries,
                        filters,
                        start,
                        target,
                        with_normal,
                        capacity,
                        &ray,
                    );
                }
            }

            const room = visited_capacity_max;
            try expectSameMove(&queries, filters, start, target, room, &move);
            try testing.expect(move.status.succeeded);
            const visited_count = move.count;
            if (visited_count > 1) moved += 1;
            for ([_]usize{ 0, 1, visited_count -| 1, visited_count }) |capacity| {
                try expectSameMove(&queries, filters, start, target, capacity, &move);
            }
        }
    }
    // Rays that reach their end, rays a wall stops, and rays and moves over
    // several polygons all ran.
    try testing.expect(reached > 0 and stopped > 0);
    try testing.expect(crossed > 0 and moved > 0);
}

const ray_options = [_]RaycastOptions{ .{}, .{ .use_costs = true } };

/// A ray both sides must refuse, through `raycast` with and without costs
/// and with path buffers of several sizes.
fn expectRefusedRay(
    queries: *const Queries,
    filters: *const Filters,
    start: Place,
    end_pos: [3]f32,
    prev_ref: PolyRef,
) !void {
    var ray: RayResult = undefined;
    for ([_]usize{ 0, 1, 8 }) |capacity| {
        for (ray_options) |options| {
            try expectSameRay(queries, filters, start, end_pos, options, prev_ref, capacity, &ray);
            try testing.expect(ray.status.failed and ray.status.invalid_param);
            try testing.expectEqual(@as(usize, 0), ray.count);
            try testing.expectEqual(@as(f32, 0.0), ray.t);
            try testing.expectEqual(@as(f32, 0.0), ray.path_cost);
        }
    }
}

/// A ray both sides must refuse, through `raycastPath` with every set of
/// optional buffers.
fn expectRefusedRayPath(
    queries: *const Queries,
    filters: *const Filters,
    start: Place,
    end_pos: [3]f32,
) !void {
    var ray: RayResult = undefined;
    for ([_]bool{ false, true }) |with_normal| {
        for ([_]?usize{ null, 0, 1, 8 }) |capacity| {
            try expectSameRayPath(queries, filters, start, end_pos, with_normal, capacity, &ray);
            try testing.expect(ray.status.failed and ray.status.invalid_param);
            try testing.expectEqual(@as(f32, 0.0), ray.t);
            if (capacity != null) try testing.expectEqual(@as(usize, 0), ray.count);
        }
    }
}

/// A move both sides must refuse, with visited buffers of several sizes.
fn expectRefusedMove(
    queries: *Queries,
    filters: *const Filters,
    start: Place,
    end_pos: [3]f32,
) !void {
    var move: MoveResult = undefined;
    for ([_]usize{ 0, 1, 8 }) |capacity| {
        try expectSameMove(queries, filters, start, end_pos, capacity, &move);
        try testing.expect(move.status.failed and move.status.invalid_param);
        try testing.expectEqual(@as(usize, 0), move.count);
    }
}

test "rays and moves refuse the input the original refuses" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, hand_world.obj_text, hand_world.config, hand_world.options);
    defer world.deinit();
    var queries: Queries = undefined;
    try queries.init(gpa, &world, roomy_node_count);
    defer queries.deinit();
    var pass_all = try Filters.init();
    defer pass_all.deinit();

    var prng: std.Random.DefaultPrng = .init(0x9a7b_0661);
    const random = prng.random();
    const filters = &pass_all;
    const start = try somePlace(&queries, filters, &world, random);
    const far = try somePlace(&queries, filters, &world, random);
    const entry = entryOf(&world.nav, start.ref) orelse return error.TestUnexpectedResult;
    const neighbour = randomNeighbour(entry, random);

    // A start polygon the mesh does not hold, whatever the ray came from.
    for ([_]PolyRef{ 0, sentinel_ref, staleRef(&world.nav, start.ref) }) |ref| {
        errdefer std.debug.print("  start polygon {x}\n", .{ref});
        const refused: Place = .{ .ref = ref, .pos = start.pos };
        for ([_]PolyRef{ 0, neighbour, start.ref, ref }) |prev_ref| {
            try expectRefusedRay(&queries, filters, refused, far.pos, prev_ref);
        }
        try expectRefusedRayPath(&queries, filters, refused, far.pos);
        try expectRefusedMove(&queries, filters, refused, far.pos);
        try expectRefusedMove(&queries, filters, refused, start.pos);
    }

    // A polygon the ray came from that the mesh does not hold.
    var stale_previous = [_]PolyRef{
        sentinel_ref,
        sentinel_ref,
        staleRef(&world.nav, start.ref),
        staleRef(&world.nav, far.ref),
    };
    if (neighbour != 0) stale_previous[0] = staleRef(&world.nav, neighbour);
    for (stale_previous) |prev_ref| {
        errdefer std.debug.print("  previous polygon {x}\n", .{prev_ref});
        try expectRefusedRay(&queries, filters, start, far.pos, prev_ref);
        try expectRefusedRay(&queries, filters, start, start.pos, prev_ref);
    }

    // Any polygon the mesh holds passes as the one the ray came from: the
    // start itself and one far from it.
    var ray: RayResult = undefined;
    for ([_]PolyRef{ start.ref, far.ref }) |prev_ref| {
        for (ray_options) |options| {
            const full = path_capacity_max;
            try expectSameRay(&queries, filters, start, far.pos, options, prev_ref, full, &ray);
            try testing.expect(ray.status.succeeded);
            try testing.expect(ray.count >= 1);
            try testing.expectEqual(start.ref, ray.path[0]);
        }
    }

    // A move with no room for the polygons it visits is refused; the same
    // move with room for one is not.
    var move: MoveResult = undefined;
    for ([_][3]f32{ far.pos, start.pos }) |target| {
        try expectSameMove(&queries, filters, start, target, 0, &move);
        try testing.expect(move.status.failed and move.status.invalid_param);
        try testing.expectEqual(@as(usize, 0), move.count);
        try expectSameMove(&queries, filters, start, target, 1, &move);
        try testing.expect(move.status.succeeded);
    }

    // Positions that are not finite, in each coordinate of either end.
    if (non_finite_checked) {
        const bad = [_]f32{ std.math.nan(f32), std.math.inf(f32), -std.math.inf(f32) };
        for (bad) |value| {
            for (0..3) |axis| {
                errdefer std.debug.print("  {d} on axis {d}\n", .{ value, axis });
                var bad_start = start;
                bad_start.pos[axis] = value;
                var bad_end = far.pos;
                bad_end[axis] = value;
                try expectRefusedRay(&queries, filters, bad_start, far.pos, 0);
                try expectRefusedRay(&queries, filters, start, bad_end, neighbour);
                try expectRefusedRay(&queries, filters, bad_start, bad_end, 0);
                try expectRefusedRayPath(&queries, filters, bad_start, far.pos);
                try expectRefusedRayPath(&queries, filters, start, bad_end);
                try expectRefusedMove(&queries, filters, bad_start, far.pos);
                try expectRefusedMove(&queries, filters, start, bad_end);
                try expectRefusedMove(&queries, filters, bad_start, bad_start.pos);
            }
        }
    }
}
