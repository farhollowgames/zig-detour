//! Differential tests of the area queries of a navigation mesh query: random
//! points on the mesh and around a circle, the polygons a circle or a convex
//! shape reaches, the path back from such a search, the local neighbourhood
//! of a point, the wall segments of one polygon, and the distance to the
//! nearest wall. The port and the original must return the same statuses and
//! the same bits, leave the same outputs alone, and leave the same nodes in
//! their node pools.

const std = @import("std");
const build_options = @import("build_options");
const detour = @import("detour");
const support = @import("support.zig");
const reference = support.reference;

const testing = std.testing;

const NavMesh = detour.NavMesh;
const NavMeshQuery = detour.NavMeshQuery;
const QueryFilter = detour.QueryFilter;
const PolyRef = detour.PolyRef;
const Status = detour.Status;

// ---------------------------------------------------------------------------
// Sentinels
// ---------------------------------------------------------------------------

// Every output holds one of these before a call, on both sides, so a slot a
// function leaves alone still has to compare equal.
const ref_sentinel: PolyRef = 0x5a5a5a5a;
const cost_sentinel: f32 = -12345.0;
const count_sentinel = 77777;
const point_sentinel: [3]f32 = .{ -901.5, 902.25, -903.125 };

/// The original counts in `i32`, the port in `usize`.
fn expectCount(expected: i32, actual: usize) !void {
    try testing.expectEqual(@as(i64, expected), @as(i64, @intCast(actual)));
}

// ---------------------------------------------------------------------------
// Worlds
// ---------------------------------------------------------------------------

/// A level and how its mesh is built.
const WorldCase = struct {
    name: []const u8,
    obj_text: []const u8,
    config: support.RecastConfig,
    options: support.LevelOptions,
};

const dungeon_one_tile: WorldCase = .{
    .name = "dungeon, one tile",
    .obj_text = support.dungeon_obj,
    .config = support.solo_config,
    .options = .{ .off_mesh_connection_count = 8, .vary_polygons = true, .seed = 21 },
};

const nav_test_tiled: WorldCase = .{
    .name = "nav_test, tiled",
    .obj_text = support.nav_test_obj,
    .config = support.tiled_config,
    .options = .{ .off_mesh_connection_count = 24, .vary_polygons = true, .seed = 22 },
};

const dungeon_tiled_plain: WorldCase = .{
    .name = "dungeon, tiled, one area",
    .obj_text = support.dungeon_obj,
    .config = support.tiled_config,
    .options = .{ .off_mesh_connection_count = 16, .seed = 23 },
};

/// Tiles of two cell sizes in a checkerboard, so border edges meet more
/// than one polygon and links cover part of an edge.
const nav_test_checkered: WorldCase = .{
    .name = "nav_test, tiled, two cell sizes",
    .obj_text = support.nav_test_obj,
    .config = support.tiled_config,
    .options = .{
        .off_mesh_connection_count = 12,
        .vary_polygons = true,
        .seed = 24,
        .odd_tile_cell_size = 0.2,
    },
};

const world_cases = [_]WorldCase{
    dungeon_one_tile,
    nav_test_tiled,
    dungeon_tiled_plain,
    nav_test_checkered,
};

/// Four worlds of this many cases put every fuzz test past a thousand.
const cases_per_world = 260;

/// The port's mesh and the original's, built from the same tiles.
const MeshPair = struct {
    port: *const NavMesh,
    original: *const reference.NavMesh,

    fn of(world: *const support.World) MeshPair {
        return .{ .port = &world.nav, .original = world.reference_nav };
    }
};

/// The port's query and the original's.
const QueryPair = struct {
    port: NavMeshQuery,
    original: *reference.NavMeshQuery,

    /// Two queries attached to nothing. Free with `deinit`.
    fn create() !QueryPair {
        const original = reference.ref_query_alloc() orelse return error.OutOfMemory;
        return .{ .port = .empty, .original = original };
    }

    fn deinit(self: *QueryPair, gpa: std.mem.Allocator) void {
        self.port.deinit(gpa);
        reference.ref_query_free(self.original);
        self.* = undefined;
    }

    /// Attaches both queries to a mesh with room for `node_count_max` nodes.
    fn attach(
        self: *QueryPair,
        gpa: std.mem.Allocator,
        mesh: MeshPair,
        node_count_max: usize,
    ) !void {
        const count: i32 = @intCast(node_count_max);
        const expected = reference.ref_query_init(self.original, mesh.original, count);
        const actual = self.port.init(gpa, mesh.port, count);
        try support.expectStatus(expected, actual);
        try testing.expect(actual.succeeded);
    }
};

/// The port's filter and the original's, always set alike.
const FilterPair = struct {
    port: QueryFilter,
    original: *reference.QueryFilter,

    /// Two filters that pass every polygon at a cost of 1. Free with `deinit`.
    fn create() !FilterPair {
        const original = reference.ref_filter_create() orelse return error.OutOfMemory;
        return .{ .port = .{}, .original = original };
    }

    fn deinit(self: *FilterPair) void {
        reference.ref_filter_destroy(self.original);
        self.* = undefined;
    }

    fn setFlags(self: *FilterPair, include: u16, exclude: u16) void {
        self.port.include_flags = include;
        self.port.exclude_flags = exclude;
        reference.ref_filter_set_include_flags(self.original, include);
        reference.ref_filter_set_exclude_flags(self.original, exclude);
    }

    fn setAreaCost(self: *FilterPair, area: usize, cost: f32) void {
        self.port.area_cost[area] = cost;
        reference.ref_filter_set_area_cost(self.original, @intCast(area), cost);
    }

    fn setEveryAreaCost(self: *FilterPair, cost: f32) void {
        for (0..self.port.area_cost.len) |area| self.setAreaCost(area, cost);
    }

    /// Back to what a new filter holds.
    fn reset(self: *FilterPair) void {
        self.setEveryAreaCost(1.0);
        self.setFlags(0xffff, 0);
    }

    /// Lets no polygon through: every polygon of the test worlds has flag 1.
    fn passNothing(self: *FilterPair) void {
        self.setFlags(0xffff, 1);
    }

    /// A cost in `[low, low + span)` for every area.
    fn randomizeCosts(self: *FilterPair, random: std.Random, low: f32, span: f32) void {
        for (0..self.port.area_cost.len) |area| {
            self.setAreaCost(area, low + span * random.float(f32));
        }
    }

    /// One of the kinds of filter a search has to cope with. The varied
    /// polygons carry flag 1 and one of the next few flags, so a mask over
    /// the low bits splits them into passing and refused.
    fn randomize(self: *FilterPair, random: std.Random) void {
        self.reset();
        const low_flags: u16 = 0x1e;
        switch (random.uintLessThan(u32, 10)) {
            0, 1 => {},
            2, 3 => self.randomizeCosts(random, 0.25, 8.0),
            4 => self.setFlags(random.int(u16) & low_flags, 0),
            5 => self.setFlags(0xffff, random.int(u16) & low_flags),
            6 => {
                self.randomizeCosts(random, 0.25, 8.0);
                self.setFlags(0xffff, random.int(u16) & low_flags);
            },
            7 => self.passNothing(),
            // Every step is free, so every node ties with every other.
            8 => self.setEveryAreaCost(0.0),
            else => self.setFlags(random.int(u16), random.int(u16) & low_flags),
        }
    }
};

// ---------------------------------------------------------------------------
// The polygons of a mesh
// ---------------------------------------------------------------------------

/// The edge of a polygon that leads out of its tile.
const border_edge: u16 = 0x8000;

/// The references of a mesh's polygons, sorted by what they are, read from
/// the original's tiles.
const Polygons = struct {
    all: std.ArrayList(PolyRef),
    ground: std.ArrayList(PolyRef),
    off_mesh: std.ArrayList(PolyRef),
    /// Ground polygons with an edge on a tile border.
    border: std.ArrayList(PolyRef),
    /// One reference per tile, just past the tile's last polygon.
    past_end: std.ArrayList(PolyRef),

    const empty: Polygons = .{
        .all = .empty,
        .ground = .empty,
        .off_mesh = .empty,
        .border = .empty,
        .past_end = .empty,
    };

    fn deinit(self: *Polygons, gpa: std.mem.Allocator) void {
        self.all.deinit(gpa);
        self.ground.deinit(gpa);
        self.off_mesh.deinit(gpa);
        self.border.deinit(gpa);
        self.past_end.deinit(gpa);
        self.* = undefined;
    }

    /// Adds every polygon of every tile the mesh holds. A mesh with no tiles
    /// adds nothing.
    fn read(self: *Polygons, gpa: std.mem.Allocator, nav: *const reference.NavMesh) !void {
        const tile_count_max = reference.ref_nav_mesh_get_max_tiles(nav);
        var tile_index: i32 = 0;
        while (tile_index < tile_count_max) : (tile_index += 1) {
            const tile = reference.ref_nav_mesh_get_tile(nav, tile_index) orelse continue;
            const header = reference.ref_tile_header(tile) orelse continue;
            const polys = reference.ref_tile_polys(tile) orelse continue;
            const base = reference.ref_nav_mesh_get_poly_ref_base(nav, tile);
            const poly_count: usize = @intCast(header.poly_count);
            for (0..poly_count) |poly_index| {
                const poly = &polys[poly_index];
                const ref: PolyRef = base | @as(PolyRef, @intCast(poly_index));
                try self.all.append(gpa, ref);
                if (reference.ref_poly_get_type(poly) != 0) {
                    try self.off_mesh.append(gpa, ref);
                    continue;
                }
                try self.ground.append(gpa, ref);
                const vert_count: usize = poly.vert_count;
                for (poly.neis[0..vert_count]) |neighbour| {
                    if (neighbour & border_edge == 0) continue;
                    try self.border.append(gpa, ref);
                    break;
                }
            }
            try self.past_end.append(gpa, base | @as(PolyRef, @intCast(poly_count)));
        }
    }

    /// A polygon to start from: mostly ground, now and then an off-mesh
    /// connection or a polygon on a tile border.
    fn pick(self: *const Polygons, random: std.Random) PolyRef {
        const kind = random.uintLessThan(u32, 8);
        if (kind == 0 and self.off_mesh.items.len > 0) return pickFrom(self.off_mesh.items, random);
        if (kind == 1 and self.border.items.len > 0) return pickFrom(self.border.items, random);
        return pickFrom(self.ground.items, random);
    }

    /// A reference that is almost surely no polygon's: none, any bits, a real
    /// one with its top bit flipped, or one past the end of a tile. Both
    /// sides decode it the same way, so either may be handed it.
    fn randomBadRef(self: *const Polygons, random: std.Random) PolyRef {
        const top_bit: PolyRef = @as(PolyRef, 1) << (@bitSizeOf(PolyRef) - 1);
        return switch (random.uintLessThan(u32, 4)) {
            0 => 0,
            1 => random.int(PolyRef),
            2 => pickFrom(self.all.items, random) ^ top_bit,
            else => pickFrom(self.past_end.items, random),
        };
    }
};

fn pickFrom(refs: []const PolyRef, random: std.Random) PolyRef {
    return refs[random.uintLessThan(usize, refs.len)];
}

/// The corners of a polygon, read from the original's tile. Returns how many
/// there are.
fn polygonCorners(nav: *const reference.NavMesh, ref: PolyRef, corners: *[6][3]f32) usize {
    var tile: ?*const reference.MeshTile = null;
    var poly: ?*const reference.Poly = null;
    reference.ref_nav_mesh_get_tile_and_poly_by_ref_unsafe(nav, ref, &tile, &poly);
    const verts = reference.ref_tile_verts(tile.?).?;
    const corner_count: usize = poly.?.vert_count;
    for (poly.?.verts[0..corner_count], corners[0..corner_count]) |vert, *corner| {
        const first: usize = @as(usize, vert) * 3;
        corner.* = .{ verts[first], verts[first + 1], verts[first + 2] };
    }
    return corner_count;
}

/// The mean of a polygon's corners. For an off-mesh connection that is the
/// middle of its two ends.
fn polygonCenter(nav: *const reference.NavMesh, ref: PolyRef) [3]f32 {
    var corners: [6][3]f32 = undefined;
    const corner_count = polygonCorners(nav, ref, &corners);
    var sum: [3]f32 = .{ 0, 0, 0 };
    for (corners[0..corner_count]) |corner| {
        for (&sum, corner) |*total, value| total.* += value;
    }
    const scale: f32 = 1.0 / @as(f32, @floatFromInt(corner_count));
    return .{ sum[0] * scale, sum[1] * scale, sum[2] * scale };
}

/// Where a search starts.
const Start = struct {
    ref: PolyRef,
    pos: [3]f32,
};

// ---------------------------------------------------------------------------
// Search outputs and node pools
// ---------------------------------------------------------------------------

/// Which of a search's optional outputs a call asks for.
const Asked = packed struct(u3) {
    refs: bool,
    parents: bool,
    costs: bool,

    const all: Asked = .{ .refs = true, .parents = true, .costs = true };
};

/// One side's output buffers.
const Side = struct {
    refs: []PolyRef,
    parents: []PolyRef,
    costs: []f32,
};

/// The output buffers of both sides, each `capacity` long.
const SearchOutputs = struct {
    capacity: usize,
    refs: []PolyRef,
    costs: []f32,

    fn init(gpa: std.mem.Allocator, capacity: usize) !SearchOutputs {
        const refs = try gpa.alloc(PolyRef, capacity * 4);
        errdefer gpa.free(refs);
        const costs = try gpa.alloc(f32, capacity * 2);
        return .{ .capacity = capacity, .refs = refs, .costs = costs };
    }

    fn deinit(self: *SearchOutputs, gpa: std.mem.Allocator) void {
        gpa.free(self.refs);
        gpa.free(self.costs);
        self.* = undefined;
    }

    /// What the original writes to.
    fn expected(self: *const SearchOutputs) Side {
        const n = self.capacity;
        return .{
            .refs = self.refs[0..n],
            .parents = self.refs[n .. n * 2],
            .costs = self.costs[0..n],
        };
    }

    /// What the port writes to.
    fn actual(self: *const SearchOutputs) Side {
        const n = self.capacity;
        return .{
            .refs = self.refs[n * 2 .. n * 3],
            .parents = self.refs[n * 3 .. n * 4],
            .costs = self.costs[n .. n * 2],
        };
    }

    fn fill(self: *const SearchOutputs) void {
        @memset(self.refs, ref_sentinel);
        @memset(self.costs, cost_sentinel);
    }

    /// Both sides hold the same bits from end to end, written or not.
    fn expectSame(self: *const SearchOutputs) !void {
        const original = self.expected();
        const port = self.actual();
        try testing.expectEqualSlices(PolyRef, original.refs, port.refs);
        try testing.expectEqualSlices(PolyRef, original.parents, port.parents);
        try support.expectFloats(original.costs, port.costs);
    }
};

/// The node pools of both queries hold the same nodes in the same slots,
/// linked into the same hash buckets. A search that ran on both sides has
/// written every field of every node it took.
fn expectSameNodePools(queries: *QueryPair) !void {
    const original = reference.ref_query_get_node_pool(queries.original) orelse
        return error.TestUnexpectedResult;
    const port = queries.port.getNodePool();

    const hash_size = port.getHashSize();
    try testing.expectEqual(reference.ref_node_pool_get_hash_size(original), hash_size);
    var bucket: i32 = 0;
    while (bucket < hash_size) : (bucket += 1) {
        try testing.expectEqual(
            reference.ref_node_pool_get_first(original, bucket),
            port.getFirst(bucket),
        );
    }

    const node_count = port.getNodeCount();
    try testing.expectEqual(reference.ref_node_pool_get_node_count(original), node_count);
    var index: i32 = 0;
    while (index < node_count) : (index += 1) {
        errdefer std.debug.print("  at node {d} of {d}\n", .{ index, node_count });
        try testing.expectEqual(
            reference.ref_node_pool_get_next(original, index),
            port.getNext(index),
        );

        // A node's index counts from one.
        const node_index: u32 = @intCast(index + 1);
        const expected = reference.ref_node_pool_get_node_at_idx(original, node_index) orelse
            return error.TestUnexpectedResult;
        const actual = port.getNodeAtIdx(node_index) orelse return error.TestUnexpectedResult;
        var pos: [3]f32 = undefined;
        var cost: f32 = undefined;
        var total: f32 = undefined;
        var parent_index: u32 = undefined;
        var state: u32 = undefined;
        var flags: u32 = undefined;
        var id: PolyRef = undefined;
        reference.ref_node_fields(
            expected,
            &pos,
            &cost,
            &total,
            &parent_index,
            &state,
            &flags,
            &id,
        );
        try support.expectPoint(pos, actual.pos);
        try support.expectFloat(cost, actual.cost);
        try support.expectFloat(total, actual.total);
        try testing.expectEqual(parent_index, @as(u32, actual.bits.parent_index));
        try testing.expectEqual(state, @as(u32, actual.bits.state));
        try testing.expectEqual(flags, @as(u32, @as(u3, @bitCast(actual.bits.flags))));
        try testing.expectEqual(id, actual.id);
    }
}

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------

/// The node count of the query that runs out of nodes on most searches.
const cramped_node_count = 24;

/// What every test of a world needs: the mesh's polygons, a filter, a query
/// with a node for every polygon, one with far too few, and output buffers
/// that hold any result.
const Fixture = struct {
    gpa: std.mem.Allocator,
    mesh: MeshPair,
    polygons: Polygons,
    filter: FilterPair,
    roomy: QueryPair,
    cramped: QueryPair,
    outputs: SearchOutputs,

    /// Fills `self` in place: a sliced search keeps a pointer to the filter.
    fn init(self: *Fixture, gpa: std.mem.Allocator, mesh: MeshPair) !void {
        self.gpa = gpa;
        self.mesh = mesh;

        self.polygons = .empty;
        errdefer self.polygons.deinit(gpa);
        try self.polygons.read(gpa, mesh.original);
        const poly_count = self.polygons.all.items.len;

        self.filter = try .create();
        errdefer self.filter.deinit();

        self.roomy = try .create();
        errdefer self.roomy.deinit(gpa);
        try self.roomy.attach(gpa, mesh, @min(poly_count + 16, 0xffff));

        self.cramped = try .create();
        errdefer self.cramped.deinit(gpa);
        try self.cramped.attach(gpa, mesh, cramped_node_count);

        self.outputs = try .init(gpa, poly_count + 16);
    }

    fn deinit(self: *Fixture) void {
        self.outputs.deinit(self.gpa);
        self.cramped.deinit(self.gpa);
        self.roomy.deinit(self.gpa);
        self.filter.deinit();
        self.polygons.deinit(self.gpa);
        self.* = undefined;
    }

    /// The roomy query three times in four, else the cramped one.
    fn pickQueries(self: *Fixture, random: std.Random) *QueryPair {
        return if (random.uintLessThan(u32, 4) == 0) &self.cramped else &self.roomy;
    }

    /// The centre of a polygon.
    fn startAt(self: *const Fixture, ref: PolyRef) Start {
        return .{ .ref = ref, .pos = polygonCenter(self.mesh.original, ref) };
    }

    /// A polygon and a point at its centre or a short way off, which may lie
    /// outside the polygon. One start in sixteen has a reference no polygon
    /// answers to.
    fn randomStart(self: *const Fixture, random: std.Random) Start {
        var start = self.startAt(self.polygons.pick(random));
        if (random.boolean()) {
            start.pos[0] += (random.float(f32) - 0.5) * 1.5;
            start.pos[2] += (random.float(f32) - 0.5) * 1.5;
        }
        if (random.uintLessThan(u32, 16) == 0) start.ref = self.polygons.randomBadRef(random);
        return start;
    }
};

/// A radius of one of the kinds a search has to cope with: none, tiny,
/// around a few polygons, past the whole mesh, and negative.
fn randomRadius(random: std.Random) f32 {
    return switch (random.uintLessThan(u32, 16)) {
        0 => 0.0,
        1 => 1e-4 * random.float(f32),
        2 => 100.0 + 400.0 * random.float(f32),
        3 => -0.5 - random.float(f32),
        4, 5, 6, 7, 8 => 0.25 + 3.0 * random.float(f32),
        else => 2.0 + 12.0 * random.float(f32),
    };
}

/// A buffer size around the size a result needs: none, one, one short,
/// exact, one over, or anything up to the capacity.
fn randomCountMax(random: std.Random, count: usize, capacity: usize) usize {
    const choice: usize = switch (random.uintLessThan(u32, 8)) {
        0 => 0,
        1 => 1,
        2 => count -| 1,
        3 => count,
        4 => count + 1,
        5 => random.uintAtMost(usize, count),
        else => random.uintAtMost(usize, capacity),
    };
    return @min(choice, capacity);
}

/// What a search returned and how many results it reported.
const SearchResult = struct {
    status: Status,
    count: usize,
};

// ---------------------------------------------------------------------------
// One call on both sides
// ---------------------------------------------------------------------------

/// The polygons around a circle.
const CircleCall = struct {
    start: Start,
    radius: f32,
    asked: Asked = .all,
    count_max: usize,
};

/// Runs `findPolysAroundCircle` on both sides and expects the same status,
/// count, outputs and node pools. The outputs stay in `fixture.outputs`.
fn expectSameCircle(fixture: *Fixture, queries: *QueryPair, call: CircleCall) !SearchResult {
    const expected = fixture.outputs.expected();
    const actual = fixture.outputs.actual();
    fixture.outputs.fill();
    const count_max = call.count_max;
    const asked = call.asked;

    var expected_count: i32 = count_sentinel;
    const expected_status = reference.ref_query_find_polys_around_circle(
        queries.original,
        call.start.ref,
        &call.start.pos,
        call.radius,
        fixture.filter.original,
        if (asked.refs) expected.refs.ptr else null,
        if (asked.parents) expected.parents.ptr else null,
        if (asked.costs) expected.costs.ptr else null,
        &expected_count,
        @intCast(count_max),
    );
    var actual_count: usize = count_sentinel;
    const actual_status = queries.port.findPolysAroundCircle(
        call.start.ref,
        call.start.pos,
        call.radius,
        &fixture.filter.port,
        if (asked.refs) actual.refs[0..count_max] else null,
        if (asked.parents) actual.parents[0..count_max] else null,
        if (asked.costs) actual.costs[0..count_max] else null,
        &actual_count,
        count_max,
    );

    try support.expectStatus(expected_status, actual_status);
    try expectCount(expected_count, actual_count);
    try fixture.outputs.expectSame();
    try expectSameNodePools(queries);
    return .{ .status = actual_status, .count = actual_count };
}

/// The polygons a convex shape touches.
const ShapeCall = struct {
    start_ref: PolyRef,
    /// Cut from an array, so that even no corners at all have an address.
    verts: []const [3]f32,
    asked: Asked = .all,
    count_max: usize,
};

/// Runs `findPolysAroundShape` on both sides and expects the same status,
/// count, outputs and node pools. The outputs stay in `fixture.outputs`.
fn expectSameShape(fixture: *Fixture, queries: *QueryPair, call: ShapeCall) !SearchResult {
    const expected = fixture.outputs.expected();
    const actual = fixture.outputs.actual();
    fixture.outputs.fill();
    const count_max = call.count_max;
    const asked = call.asked;

    var expected_count: i32 = count_sentinel;
    const expected_status = reference.ref_query_find_polys_around_shape(
        queries.original,
        call.start_ref,
        @ptrCast(call.verts.ptr),
        @intCast(call.verts.len),
        fixture.filter.original,
        if (asked.refs) expected.refs.ptr else null,
        if (asked.parents) expected.parents.ptr else null,
        if (asked.costs) expected.costs.ptr else null,
        &expected_count,
        @intCast(count_max),
    );
    var actual_count: usize = count_sentinel;
    const actual_status = queries.port.findPolysAroundShape(
        call.start_ref,
        call.verts,
        &fixture.filter.port,
        if (asked.refs) actual.refs[0..count_max] else null,
        if (asked.parents) actual.parents[0..count_max] else null,
        if (asked.costs) actual.costs[0..count_max] else null,
        &actual_count,
        count_max,
    );

    try support.expectStatus(expected_status, actual_status);
    try expectCount(expected_count, actual_count);
    try fixture.outputs.expectSame();
    try expectSameNodePools(queries);
    return .{ .status = actual_status, .count = actual_count };
}

/// Runs `getPathFromDijkstraSearch` on both sides with room for `path_max`
/// references and expects the same status, count and path. The path stays
/// in the `refs` of `fixture.outputs`.
fn expectSameDijkstraPath(
    fixture: *Fixture,
    queries: *QueryPair,
    end_ref: PolyRef,
    path_max: usize,
) !SearchResult {
    const expected = fixture.outputs.expected();
    const actual = fixture.outputs.actual();
    fixture.outputs.fill();

    var expected_count: i32 = count_sentinel;
    const expected_status = reference.ref_query_get_path_from_dijkstra_search(
        queries.original,
        end_ref,
        expected.refs.ptr,
        &expected_count,
        @intCast(path_max),
    );
    var actual_count: usize = count_sentinel;
    const actual_status = queries.port.getPathFromDijkstraSearch(
        end_ref,
        actual.refs[0..path_max],
        &actual_count,
    );

    try support.expectStatus(expected_status, actual_status);
    try expectCount(expected_count, actual_count);
    try fixture.outputs.expectSame();
    // Reading a path back changes no node.
    try expectSameNodePools(queries);
    return .{ .status = actual_status, .count = actual_count };
}

/// The polygons near a point, none overlapping another.
const LocalCall = struct {
    start: Start,
    radius: f32,
    with_parents: bool = true,
    count_max: usize,
};

/// Runs `findLocalNeighbourhood` on both sides and expects the same status,
/// count and outputs. The outputs stay in `fixture.outputs`.
fn expectSameLocal(fixture: *Fixture, queries: *QueryPair, call: LocalCall) !SearchResult {
    const expected = fixture.outputs.expected();
    const actual = fixture.outputs.actual();
    fixture.outputs.fill();
    const count_max = call.count_max;

    var expected_count: i32 = count_sentinel;
    const expected_status = reference.ref_query_find_local_neighbourhood(
        queries.original,
        call.start.ref,
        &call.start.pos,
        call.radius,
        fixture.filter.original,
        expected.refs.ptr,
        if (call.with_parents) expected.parents.ptr else null,
        &expected_count,
        @intCast(count_max),
    );
    var actual_count: usize = count_sentinel;
    const actual_status = queries.port.findLocalNeighbourhood(
        call.start.ref,
        call.start.pos,
        call.radius,
        &fixture.filter.port,
        actual.refs[0..count_max],
        if (call.with_parents) actual.parents[0..count_max] else null,
        &actual_count,
    );

    try support.expectStatus(expected_status, actual_status);
    try expectCount(expected_count, actual_count);
    try fixture.outputs.expectSame();
    // The neighbourhood has a pool of its own and leaves the search's alone.
    try expectSameNodePools(queries);
    return .{ .status = actual_status, .count = actual_count };
}

/// More segments than any polygon of the test worlds has.
const segment_capacity = 64;

/// The wall and portal segments of one polygon.
const WallSegmentsCall = struct {
    ref: PolyRef,
    with_refs: bool = true,
    count_max: usize = segment_capacity,
};

/// Runs `getPolyWallSegments` on both sides and expects the same status,
/// count, segments and neighbours, written or not.
fn expectSameWallSegments(
    fixture: *Fixture,
    queries: *QueryPair,
    call: WallSegmentsCall,
) !SearchResult {
    const count_max = call.count_max;
    var expected_verts: [segment_capacity][2][3]f32 = @splat(@splat(point_sentinel));
    var actual_verts: [segment_capacity][2][3]f32 = @splat(@splat(point_sentinel));
    var expected_refs: [segment_capacity]PolyRef = @splat(ref_sentinel);
    var actual_refs: [segment_capacity]PolyRef = @splat(ref_sentinel);

    var expected_count: i32 = count_sentinel;
    const expected_refs_out: ?[*]PolyRef = if (call.with_refs) &expected_refs else null;
    const expected_status = reference.ref_query_get_poly_wall_segments(
        queries.original,
        call.ref,
        fixture.filter.original,
        @ptrCast(&expected_verts),
        expected_refs_out,
        &expected_count,
        @intCast(count_max),
    );
    var actual_count: usize = count_sentinel;
    const actual_refs_out: ?[]PolyRef = if (call.with_refs) actual_refs[0..count_max] else null;
    const actual_status = queries.port.getPolyWallSegments(
        call.ref,
        &fixture.filter.port,
        actual_verts[0..count_max],
        actual_refs_out,
        &actual_count,
    );

    try support.expectStatus(expected_status, actual_status);
    try expectCount(expected_count, actual_count);
    for (expected_verts, actual_verts) |expected_segment, actual_segment| {
        try support.expectPoint(expected_segment[0], actual_segment[0]);
        try support.expectPoint(expected_segment[1], actual_segment[1]);
    }
    try testing.expectEqualSlices(PolyRef, &expected_refs, &actual_refs);
    return .{ .status = actual_status, .count = actual_count };
}

/// What `findDistanceToWall` returned on both sides.
const WallDistance = struct {
    status: Status,
    hit_dist: f32,
    hit_pos: [3]f32,
    hit_normal: [3]f32,
};

/// Runs `findDistanceToWall` on both sides and expects the same status,
/// distance, hit position, normal and node pools. A search that reaches no
/// wall leaves the hit position alone and takes the normal from it, so both
/// sides start from the same position.
fn expectSameDistanceToWall(
    fixture: *Fixture,
    queries: *QueryPair,
    start: Start,
    radius: f32,
) !WallDistance {
    var expected_dist: f32 = cost_sentinel;
    var expected_pos: [3]f32 = point_sentinel;
    var expected_normal: [3]f32 = point_sentinel;
    const expected_status = reference.ref_query_find_distance_to_wall(
        queries.original,
        start.ref,
        &start.pos,
        radius,
        fixture.filter.original,
        &expected_dist,
        &expected_pos,
        &expected_normal,
    );
    var actual_dist: f32 = cost_sentinel;
    var actual_pos: [3]f32 = point_sentinel;
    var actual_normal: [3]f32 = point_sentinel;
    const actual_status = queries.port.findDistanceToWall(
        start.ref,
        start.pos,
        radius,
        &fixture.filter.port,
        &actual_dist,
        &actual_pos,
        &actual_normal,
    );

    try support.expectStatus(expected_status, actual_status);
    try support.expectFloat(expected_dist, actual_dist);
    try support.expectPoint(expected_pos, actual_pos);
    try support.expectPoint(expected_normal, actual_normal);
    try expectSameNodePools(queries);
    return .{
        .status = actual_status,
        .hit_dist = actual_dist,
        .hit_pos = actual_pos,
        .hit_normal = actual_normal,
    };
}

/// What a random point function returned on both sides.
const RandomPoint = struct {
    status: Status,
    ref: PolyRef,
    point: [3]f32,
    /// The value both sides drew right after the call.
    next: f32,
};

/// The first value a generator started from `seed` draws.
fn firstDraw(seed: u64) f32 {
    var prng: std.Random.DefaultPrng = .init(seed);
    return prng.random().float(f32);
}

/// Runs `findRandomPoint` on both sides with the same random sequence and
/// expects the same status, polygon and point, and as many draws: the value
/// either side draws next is the same.
fn expectSameRandomPoint(fixture: *Fixture, queries: *QueryPair, seed: u64) !RandomPoint {
    support.seedReferenceRandom(seed);
    var expected_ref: PolyRef = ref_sentinel;
    var expected_point: [3]f32 = point_sentinel;
    const expected_status = reference.ref_query_find_random_point(
        queries.original,
        fixture.filter.original,
        support.referenceRandom,
        &expected_ref,
        &expected_point,
    );
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();
    var actual_ref: PolyRef = ref_sentinel;
    var actual_point: [3]f32 = point_sentinel;
    const actual_status = queries.port.findRandomPoint(
        &fixture.filter.port,
        random,
        &actual_ref,
        &actual_point,
    );

    try support.expectStatus(expected_status, actual_status);
    try testing.expectEqual(expected_ref, actual_ref);
    try support.expectPoint(expected_point, actual_point);
    const next = random.float(f32);
    try support.expectFloat(support.referenceRandom(), next);
    return .{ .status = actual_status, .ref = actual_ref, .point = actual_point, .next = next };
}

/// Runs `findRandomPointAroundCircle` on both sides with the same random
/// sequence and expects the same status, polygon, point, node pools and
/// number of draws.
fn expectSameRandomPointAroundCircle(
    fixture: *Fixture,
    queries: *QueryPair,
    start: Start,
    radius: f32,
    seed: u64,
) !RandomPoint {
    support.seedReferenceRandom(seed);
    var expected_ref: PolyRef = ref_sentinel;
    var expected_point: [3]f32 = point_sentinel;
    const expected_status = reference.ref_query_find_random_point_around_circle(
        queries.original,
        start.ref,
        &start.pos,
        radius,
        fixture.filter.original,
        support.referenceRandom,
        &expected_ref,
        &expected_point,
    );
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();
    var actual_ref: PolyRef = ref_sentinel;
    var actual_point: [3]f32 = point_sentinel;
    const actual_status = queries.port.findRandomPointAroundCircle(
        start.ref,
        start.pos,
        radius,
        &fixture.filter.port,
        random,
        &actual_ref,
        &actual_point,
    );

    try support.expectStatus(expected_status, actual_status);
    try testing.expectEqual(expected_ref, actual_ref);
    try support.expectPoint(expected_point, actual_point);
    const next = random.float(f32);
    try support.expectFloat(support.referenceRandom(), next);
    try expectSameNodePools(queries);
    return .{ .status = actual_status, .ref = actual_ref, .point = actual_point, .next = next };
}

/// Writes a regular polygon in the ground plane around `center`, one corner
/// per element of `corners`. The corners go round from the angle `turn`,
/// measured from the x axis towards the z axis; `clockwise` mirrors them in
/// the x axis, so they go round the other way.
fn regularShape(
    corners: [][3]f32,
    center: [3]f32,
    radius: f32,
    turn: f32,
    clockwise: bool,
) void {
    const corner_count: f32 = @floatFromInt(corners.len);
    for (corners, 0..) |*corner, i| {
        const step: f32 = @floatFromInt(i);
        var angle = turn + step * 2.0 * std.math.pi / corner_count;
        if (clockwise) angle = -angle;
        corner.* = .{
            center[0] + @cos(angle) * radius,
            center[1],
            center[2] + @sin(angle) * radius,
        };
    }
}

/// Starts by hand: the first, middle and last ground polygon, an off-mesh
/// connection and a polygon on a tile border, where the world has them.
fn edgeStarts(fixture: *const Fixture) [5]Start {
    const ground = fixture.polygons.ground.items;
    const off_mesh = fixture.polygons.off_mesh.items;
    const border = fixture.polygons.border.items;
    return .{
        fixture.startAt(ground[0]),
        fixture.startAt(ground[ground.len / 2]),
        fixture.startAt(ground[ground.len - 1]),
        fixture.startAt(if (off_mesh.len > 0) off_mesh[0] else ground[0]),
        fixture.startAt(if (border.len > 0) border[0] else ground[0]),
    };
}

/// Radii asked for by hand: none, tiny, typical, past the whole mesh, and
/// the ones the functions refuse.
const edge_radii = [_]f32{
    0.0,
    -0.0,
    1e-6,
    0.5,
    3.0,
    12.0,
    1000.0,
    std.math.floatMax(f32),
    -1.0,
    -std.math.inf(f32),
    std.math.inf(f32),
    std.math.nan(f32),
};

/// Centres no search accepts unless the build skips its finite checks.
const non_finite_centers = [_][3]f32{
    .{ std.math.nan(f32), 0.0, 0.0 },
    .{ 0.0, std.math.nan(f32), 0.0 },
    .{ 0.0, 0.0, std.math.nan(f32) },
    .{ std.math.inf(f32), 0.0, 0.0 },
    .{ 0.0, -std.math.inf(f32), 0.0 },
    .{ 0.0, 0.0, std.math.inf(f32) },
};

/// A centre so far off that squared distances to it overflow.
const far_center: [3]f32 = .{ 1e30, 0.0, -1e30 };

/// Node counts of queries that run out almost at once.
const tiny_node_counts = [_]usize{ 4, 5, 8, 16 };

/// The edge cases run on the tiled test world: it has several tiles,
/// off-mesh connections, and polygons of differing areas and flags.
const edge_world = nav_test_tiled;

/// The port's mesh and the original's, set up alike and holding no tile.
const BareMeshes = struct {
    port: NavMesh,
    original: *reference.NavMesh,

    /// Fills `self` in place: queries keep a pointer to the port's mesh.
    fn init(self: *BareMeshes, gpa: std.mem.Allocator) !void {
        const original = reference.ref_nav_mesh_alloc() orelse return error.OutOfMemory;
        errdefer reference.ref_nav_mesh_free(original);
        const expected = reference.ref_nav_mesh_init(original, &.{
            .orig = .{ 0, 0, 0 },
            .tile_width = 32,
            .tile_height = 32,
            .max_tiles = 8,
            .max_polys = 256,
        });

        self.port = .empty;
        const actual = self.port.init(gpa, &.{
            .origin = .{ 0, 0, 0 },
            .tile_width = 32,
            .tile_height = 32,
            .tile_count_max = 8,
            .poly_count_max = 256,
        });
        errdefer if (actual.succeeded) self.port.deinit();
        try support.expectStatus(expected, actual);
        try testing.expect(actual.succeeded);
        self.original = original;
    }

    fn deinit(self: *BareMeshes) void {
        self.port.deinit();
        reference.ref_nav_mesh_free(self.original);
        self.* = undefined;
    }

    fn pair(self: *const BareMeshes) MeshPair {
        return .{ .port = &self.port, .original = self.original };
    }
};

// ---------------------------------------------------------------------------
// findRandomPoint
// ---------------------------------------------------------------------------

test "findRandomPoint: random seeds and filters match" {
    const gpa = testing.allocator;
    const seed: u64 = 0x61726561_0001;
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(seed));
    const random = prng.random();

    var found_count: usize = 0;
    for (world_cases, 0..) |world_case, world_index| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{world_case.name});
        var world: support.World = undefined;
        try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        defer world.deinit();
        var fixture: Fixture = undefined;
        try fixture.init(gpa, .of(&world));
        defer fixture.deinit();

        for (0..cases_per_world) |i| {
            const case = world_index * cases_per_world + i;
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });

            fixture.filter.randomize(random);
            const queries = fixture.pickQueries(random);
            const result = try expectSameRandomPoint(&fixture, queries, random.int(u64));
            if (result.status.succeeded) found_count += 1;
        }
    }
    try testing.expect(found_count > 0);
}

test "findRandomPoint: filters that pass everything, a little and nothing" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, edge_world.obj_text, edge_world.config, edge_world.options);
    defer world.deinit();
    var fixture: Fixture = undefined;
    try fixture.init(gpa, .of(&world));
    defer fixture.deinit();

    var seed: u64 = 0;
    while (seed < 16) : (seed += 1) {
        errdefer std.debug.print("  seed {d}\n", .{seed});

        // Every tile has ground the plain filter passes.
        fixture.filter.reset();
        const plain = try expectSameRandomPoint(&fixture, &fixture.roomy, seed);
        try testing.expect(plain.status.succeeded);
        try testing.expect(plain.ref != ref_sentinel);

        // Polygons of one flag, of all flags but one, and of costly areas.
        fixture.filter.setFlags(2, 0);
        _ = try expectSameRandomPoint(&fixture, &fixture.roomy, seed);
        fixture.filter.setFlags(0xffff, 4);
        _ = try expectSameRandomPoint(&fixture, &fixture.cramped, seed);
        fixture.filter.reset();
        fixture.filter.setAreaCost(0, 0.0);
        fixture.filter.setAreaCost(1, 100.0);
        _ = try expectSameRandomPoint(&fixture, &fixture.roomy, seed);

        // No polygon passes: a plain failure that leaves the outputs alone.
        fixture.filter.reset();
        fixture.filter.passNothing();
        const none = try expectSameRandomPoint(&fixture, &fixture.roomy, seed);
        try testing.expect(none.status.failed);
        try testing.expect(!none.status.invalid_param);
        try testing.expectEqual(ref_sentinel, none.ref);
        try support.expectPoint(point_sentinel, none.point);

        fixture.filter.setFlags(0, 0);
        const unasked = try expectSameRandomPoint(&fixture, &fixture.roomy, seed);
        try testing.expect(unasked.status.failed);
        try testing.expectEqual(ref_sentinel, unasked.ref);
    }
}

test "findRandomPoint: a mesh with no tiles fails without drawing" {
    const gpa = testing.allocator;
    var meshes: BareMeshes = undefined;
    try meshes.init(gpa);
    defer meshes.deinit();
    var fixture: Fixture = undefined;
    try fixture.init(gpa, meshes.pair());
    defer fixture.deinit();
    try testing.expectEqual(0, fixture.polygons.all.items.len);

    var seed: u64 = 0;
    while (seed < 8) : (seed += 1) {
        errdefer std.debug.print("  seed {d}\n", .{seed});
        const result = try expectSameRandomPoint(&fixture, &fixture.roomy, seed);
        try testing.expect(result.status.failed);
        try testing.expect(!result.status.invalid_param);
        try testing.expectEqual(ref_sentinel, result.ref);
        try support.expectPoint(point_sentinel, result.point);
        // Nothing was drawn: the next value is the sequence's first.
        try support.expectFloat(firstDraw(seed), result.next);
    }
}

// ---------------------------------------------------------------------------
// findRandomPointAroundCircle
// ---------------------------------------------------------------------------

test "findRandomPointAroundCircle: random starts, radii and filters match" {
    const gpa = testing.allocator;
    const seed: u64 = 0x61726561_0002;
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(seed));
    const random = prng.random();

    var found_count: usize = 0;
    for (world_cases, 0..) |world_case, world_index| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{world_case.name});
        var world: support.World = undefined;
        try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        defer world.deinit();
        var fixture: Fixture = undefined;
        try fixture.init(gpa, .of(&world));
        defer fixture.deinit();

        for (0..cases_per_world) |i| {
            const case = world_index * cases_per_world + i;
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });

            fixture.filter.randomize(random);
            const queries = fixture.pickQueries(random);
            const start = fixture.randomStart(random);
            const radius = randomRadius(random);
            const result = try expectSameRandomPointAroundCircle(
                &fixture,
                queries,
                start,
                radius,
                random.int(u64),
            );
            if (result.status.succeeded) found_count += 1;
        }
    }
    try testing.expect(found_count > 0);
}

test "findRandomPointAroundCircle: radii, starts, filters and small pools by hand" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, edge_world.obj_text, edge_world.config, edge_world.options);
    defer world.deinit();
    var fixture: Fixture = undefined;
    try fixture.init(gpa, .of(&world));
    defer fixture.deinit();
    const starts = edgeStarts(&fixture);

    // Every radius from every start, with nodes to spare and with too few.
    for (starts, 0..) |start, start_index| {
        for (edge_radii, 0..) |radius, radius_index| {
            errdefer std.debug.print("  start {d}, radius {d}\n", .{ start_index, radius_index });
            const seed: u64 = start_index * 100 + radius_index;
            fixture.filter.reset();
            const roomy = try expectSameRandomPointAroundCircle(
                &fixture,
                &fixture.roomy,
                start,
                radius,
                seed,
            );
            if (radius < 0.0) {
                // Refused before anything is drawn or written.
                try testing.expect(roomy.status.failed and roomy.status.invalid_param);
                try testing.expectEqual(ref_sentinel, roomy.ref);
                try support.expectPoint(point_sentinel, roomy.point);
                try support.expectFloat(firstDraw(seed), roomy.next);
            }
            _ = try expectSameRandomPointAroundCircle(
                &fixture,
                &fixture.cramped,
                start,
                radius,
                seed,
            );

            // Areas of differing costs change the order polygons are met in.
            fixture.filter.setAreaCost(0, 7.5);
            fixture.filter.setAreaCost(1, 0.125);
            fixture.filter.setAreaCost(2, 0.0);
            fixture.filter.setFlags(0xffff, 8);
            _ = try expectSameRandomPointAroundCircle(
                &fixture,
                &fixture.roomy,
                start,
                radius,
                seed,
            );
        }
    }

    // A start the filter refuses is refused whole, before anything is drawn.
    for (starts, 0..) |start, start_index| {
        errdefer std.debug.print("  refused start {d}\n", .{start_index});
        const seed: u64 = 500 + start_index;
        fixture.filter.reset();
        fixture.filter.passNothing();
        const result = try expectSameRandomPointAroundCircle(
            &fixture,
            &fixture.roomy,
            start,
            3.0,
            seed,
        );
        try testing.expect(result.status.failed and result.status.invalid_param);
        try testing.expectEqual(ref_sentinel, result.ref);
        try support.expectFloat(firstDraw(seed), result.next);
    }

    // No polygon at all.
    fixture.filter.reset();
    const no_ref: Start = .{ .ref = 0, .pos = starts[0].pos };
    const refused = try expectSameRandomPointAroundCircle(&fixture, &fixture.roomy, no_ref, 3.0, 7);
    try testing.expect(refused.status.failed and refused.status.invalid_param);
    try support.expectFloat(firstDraw(7), refused.next);

    // A centre far from its start polygon, default costs.
    for (starts) |start| {
        const far: Start = .{ .ref = start.ref, .pos = far_center };
        _ = try expectSameRandomPointAroundCircle(&fixture, &fixture.roomy, far, 2.0, 8);
        _ = try expectSameRandomPointAroundCircle(&fixture, &fixture.roomy, far, 1e30, 9);
    }

    // A centre that is no point at all.
    if (!build_options.fast_math) {
        for (non_finite_centers, 0..) |center, center_index| {
            errdefer std.debug.print("  centre {d}\n", .{center_index});
            const start: Start = .{ .ref = starts[1].ref, .pos = center };
            const result = try expectSameRandomPointAroundCircle(
                &fixture,
                &fixture.roomy,
                start,
                3.0,
                10,
            );
            try testing.expect(result.status.failed and result.status.invalid_param);
            try support.expectFloat(firstDraw(10), result.next);
        }
    }

    // Pools of a handful of nodes.
    fixture.filter.reset();
    for (tiny_node_counts) |node_count| {
        errdefer std.debug.print("  pool of {d} nodes\n", .{node_count});
        var queries: QueryPair = try .create();
        defer queries.deinit(gpa);
        try queries.attach(gpa, fixture.mesh, node_count);
        for (starts, 0..) |start, start_index| {
            for ([_]f32{ 0.0, 2.0, 1000.0 }) |radius| {
                _ = try expectSameRandomPointAroundCircle(
                    &fixture,
                    &queries,
                    start,
                    radius,
                    600 + start_index,
                );
            }
        }
    }
}

test "findRandomPointAroundCircle: a mesh with no tiles has no start" {
    const gpa = testing.allocator;
    var meshes: BareMeshes = undefined;
    try meshes.init(gpa);
    defer meshes.deinit();
    var fixture: Fixture = undefined;
    try fixture.init(gpa, meshes.pair());
    defer fixture.deinit();

    const refs = [_]PolyRef{ 0, 1, 0x1234, ref_sentinel };
    for (refs, 0..) |ref, seed| {
        errdefer std.debug.print("  reference {d}\n", .{seed});
        const start: Start = .{ .ref = ref, .pos = .{ 1.0, 2.0, 3.0 } };
        const result = try expectSameRandomPointAroundCircle(
            &fixture,
            &fixture.roomy,
            start,
            5.0,
            seed,
        );
        try testing.expect(result.status.failed and result.status.invalid_param);
        try testing.expectEqual(ref_sentinel, result.ref);
        try support.expectPoint(point_sentinel, result.point);
        try support.expectFloat(firstDraw(seed), result.next);
    }
}

// ---------------------------------------------------------------------------
// findPolysAroundCircle
// ---------------------------------------------------------------------------

test "findPolysAroundCircle: random starts, radii, filters and buffers match" {
    const gpa = testing.allocator;
    const seed: u64 = 0x61726561_0003;
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(seed));
    const random = prng.random();

    var found_count: usize = 0;
    var out_of_nodes_count: usize = 0;
    for (world_cases, 0..) |world_case, world_index| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{world_case.name});
        var world: support.World = undefined;
        try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        defer world.deinit();
        var fixture: Fixture = undefined;
        try fixture.init(gpa, .of(&world));
        defer fixture.deinit();
        const capacity = fixture.outputs.capacity;

        for (0..cases_per_world) |i| {
            const case = world_index * cases_per_world + i;
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });

            fixture.filter.randomize(random);
            const queries = fixture.pickQueries(random);
            const start = fixture.randomStart(random);
            const radius = randomRadius(random);

            // Once with room for everything, then with a buffer sized around
            // the result and any mix of outputs.
            const full = try expectSameCircle(&fixture, queries, .{
                .start = start,
                .radius = radius,
                .count_max = capacity,
            });
            const sized_max = randomCountMax(random, full.count, capacity);
            const sized = try expectSameCircle(&fixture, queries, .{
                .start = start,
                .radius = radius,
                .asked = @bitCast(random.int(u3)),
                .count_max = sized_max,
            });
            if (!full.status.succeeded) continue;

            // The search itself does not depend on the buffer.
            try testing.expect(!full.status.buffer_too_small);
            try testing.expect(sized.status.succeeded);
            try testing.expectEqual(@min(full.count, sized_max), sized.count);
            try testing.expectEqual(sized_max < full.count, sized.status.buffer_too_small);
            try testing.expectEqual(full.status.out_of_nodes, sized.status.out_of_nodes);
            found_count += 1;
            if (full.status.out_of_nodes) out_of_nodes_count += 1;
        }
    }
    try testing.expect(found_count > 0);
    try testing.expect(out_of_nodes_count > 0);
}

test "findPolysAroundCircle: every radius and mix of outputs from starts by hand" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, edge_world.obj_text, edge_world.config, edge_world.options);
    defer world.deinit();
    var fixture: Fixture = undefined;
    try fixture.init(gpa, .of(&world));
    defer fixture.deinit();
    const capacity = fixture.outputs.capacity;
    const starts = edgeStarts(&fixture);

    for (starts, 0..) |start, start_index| {
        for (edge_radii, 0..) |radius, radius_index| {
            errdefer std.debug.print("  start {d}, radius {d}\n", .{ start_index, radius_index });
            fixture.filter.reset();
            var asked_bits: u3 = 0;
            while (true) : (asked_bits += 1) {
                const result = try expectSameCircle(&fixture, &fixture.roomy, .{
                    .start = start,
                    .radius = radius,
                    .asked = @bitCast(asked_bits),
                    .count_max = capacity,
                });
                if (radius < 0.0) {
                    try testing.expect(result.status.failed and result.status.invalid_param);
                    try testing.expectEqual(0, result.count);
                } else if (std.math.isFinite(radius)) {
                    // The start polygon is always found, whatever the radius.
                    try testing.expect(result.status.succeeded);
                    try testing.expect(result.count >= 1);
                    try testing.expect(!result.status.out_of_nodes);
                }
                if (asked_bits == std.math.maxInt(u3)) break;
            }

            // Too few nodes, then costs that reorder the search.
            _ = try expectSameCircle(&fixture, &fixture.cramped, .{
                .start = start,
                .radius = radius,
                .count_max = capacity,
            });
            fixture.filter.setAreaCost(0, 7.5);
            fixture.filter.setAreaCost(1, 0.125);
            fixture.filter.setAreaCost(2, 0.0);
            fixture.filter.setFlags(0xffff, 8);
            _ = try expectSameCircle(&fixture, &fixture.roomy, .{
                .start = start,
                .radius = radius,
                .count_max = capacity,
            });
        }
    }
}

test "findPolysAroundCircle: buffers, filters, pools and refused inputs by hand" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, edge_world.obj_text, edge_world.config, edge_world.options);
    defer world.deinit();
    var fixture: Fixture = undefined;
    try fixture.init(gpa, .of(&world));
    defer fixture.deinit();
    const capacity = fixture.outputs.capacity;
    const starts = edgeStarts(&fixture);

    // Buffers of no room, one slot, one short, exact and one over.
    for (starts, 0..) |start, start_index| {
        errdefer std.debug.print("  start {d}\n", .{start_index});
        fixture.filter.reset();
        const full = try expectSameCircle(&fixture, &fixture.roomy, .{
            .start = start,
            .radius = 12.0,
            .count_max = capacity,
        });
        try testing.expect(full.status.succeeded);
        const n = full.count;
        // The first result is the start itself, with no parent and no cost.
        const found = fixture.outputs.actual();
        try testing.expectEqual(start.ref, found.refs[0]);
        try testing.expectEqual(0, found.parents[0]);
        try support.expectFloat(0.0, found.costs[0]);

        for ([_]usize{ 0, 1, n -| 1, n, n + 1 }) |count_max| {
            var asked_bits: u3 = 0;
            while (true) : (asked_bits += 1) {
                const sized = try expectSameCircle(&fixture, &fixture.roomy, .{
                    .start = start,
                    .radius = 12.0,
                    .asked = @bitCast(asked_bits),
                    .count_max = count_max,
                });
                try testing.expect(sized.status.succeeded);
                try testing.expectEqual(@min(n, count_max), sized.count);
                try testing.expectEqual(count_max < n, sized.status.buffer_too_small);
                if (asked_bits == std.math.maxInt(u3)) break;
            }
        }

        // A filter that passes nothing still reports the start: the start
        // polygon is never put to the filter.
        fixture.filter.passNothing();
        const alone = try expectSameCircle(&fixture, &fixture.roomy, .{
            .start = start,
            .radius = 1000.0,
            .count_max = capacity,
        });
        try testing.expect(alone.status.succeeded);
        try testing.expectEqual(1, alone.count);

        // Every step free: all nodes tie.
        fixture.filter.reset();
        fixture.filter.setEveryAreaCost(0.0);
        _ = try expectSameCircle(&fixture, &fixture.roomy, .{
            .start = start,
            .radius = 12.0,
            .count_max = capacity,
        });
    }

    // Pools of a handful of nodes run out on a wide search.
    fixture.filter.reset();
    var out_of_nodes_seen = false;
    for (tiny_node_counts) |node_count| {
        errdefer std.debug.print("  pool of {d} nodes\n", .{node_count});
        var queries: QueryPair = try .create();
        defer queries.deinit(gpa);
        try queries.attach(gpa, fixture.mesh, node_count);
        for (starts) |start| {
            for ([_]f32{ 0.0, 2.0, 1000.0 }) |radius| {
                for ([_]usize{ 0, 2, capacity }) |count_max| {
                    const result = try expectSameCircle(&fixture, &queries, .{
                        .start = start,
                        .radius = radius,
                        .count_max = count_max,
                    });
                    try testing.expect(result.count <= node_count);
                    if (result.status.out_of_nodes) out_of_nodes_seen = true;
                }
            }
        }
    }
    try testing.expect(out_of_nodes_seen);

    // References no polygon answers to: refused, the count cleared, the
    // buffers and the pool left alone.
    const no_ref: Start = .{ .ref = 0, .pos = starts[0].pos };
    const refused = try expectSameCircle(&fixture, &fixture.roomy, .{
        .start = no_ref,
        .radius = 3.0,
        .count_max = capacity,
    });
    try testing.expect(refused.status.failed and refused.status.invalid_param);
    try testing.expectEqual(0, refused.count);
    const top_bit: PolyRef = @as(PolyRef, 1) << (@bitSizeOf(PolyRef) - 1);
    for (fixture.polygons.past_end.items) |ref| {
        const past_end: Start = .{ .ref = ref, .pos = starts[0].pos };
        _ = try expectSameCircle(&fixture, &fixture.roomy, .{
            .start = past_end,
            .radius = 3.0,
            .count_max = capacity,
        });
        const flipped: Start = .{ .ref = (ref - 1) ^ top_bit, .pos = starts[0].pos };
        _ = try expectSameCircle(&fixture, &fixture.roomy, .{
            .start = flipped,
            .radius = 3.0,
            .count_max = capacity,
        });
    }

    // A centre far from its start polygon, default costs.
    for (starts) |start| {
        const far: Start = .{ .ref = start.ref, .pos = far_center };
        for ([_]f32{ 0.0, 2.0, 1e30, std.math.floatMax(f32) }) |radius| {
            _ = try expectSameCircle(&fixture, &fixture.roomy, .{
                .start = far,
                .radius = radius,
                .count_max = capacity,
            });
        }
    }

    // A centre that is no point at all.
    if (!build_options.fast_math) {
        for (non_finite_centers, 0..) |center, center_index| {
            errdefer std.debug.print("  centre {d}\n", .{center_index});
            const result = try expectSameCircle(&fixture, &fixture.roomy, .{
                .start = .{ .ref = starts[1].ref, .pos = center },
                .radius = 3.0,
                .count_max = capacity,
            });
            try testing.expect(result.status.failed and result.status.invalid_param);
            try testing.expectEqual(0, result.count);
        }
    }
}

// ---------------------------------------------------------------------------
// findPolysAroundShape
// ---------------------------------------------------------------------------

/// The most corners a test shape has.
const shape_corner_count_max = 12;

/// Puts every corner on one line through `center`.
fn lineShape(corners: [][3]f32, center: [3]f32) void {
    for (corners, 0..) |*corner, i| {
        const step: f32 = @floatFromInt(i);
        corner.* = .{ center[0] + step - 1.0, center[1], center[2] + (step - 1.0) * 0.5 };
    }
}

/// A shape around `center` of one of the kinds a search has to cope with:
/// too few corners, every corner in one place or on one line, a ring of
/// corners at uneven distances, a regular polygon wider than the mesh, and
/// regular polygons of three to twelve corners in either winding. The shape
/// is cut from `corners`.
fn randomShape(
    random: std.Random,
    corners: *[shape_corner_count_max][3]f32,
    center: [3]f32,
) []const [3]f32 {
    const kind = random.uintLessThan(u32, 16);
    const count: usize = switch (kind) {
        0 => random.uintLessThan(usize, 3),
        1, 2 => random.intRangeAtMost(usize, 3, 6),
        else => random.intRangeAtMost(usize, 3, shape_corner_count_max),
    };
    const shape = corners[0..count];
    const turn = random.float(f32) * 6.0;
    const clockwise = random.boolean();
    switch (kind) {
        0 => regularShape(shape, center, 2.0, turn, clockwise),
        1 => @memset(shape, center),
        2 => lineShape(shape, center),
        3 => {
            regularShape(shape, center, 6.0, turn, clockwise);
            for (shape) |*corner| {
                corner[0] += (random.float(f32) - 0.5) * 8.0;
                corner[1] += (random.float(f32) - 0.5) * 2.0;
                corner[2] += (random.float(f32) - 0.5) * 8.0;
            }
        },
        4 => regularShape(shape, center, 100.0 + 400.0 * random.float(f32), turn, clockwise),
        else => regularShape(shape, center, 0.25 + 10.0 * random.float(f32), turn, clockwise),
    }
    return shape;
}

test "findPolysAroundShape: random starts, shapes, filters and buffers match" {
    const gpa = testing.allocator;
    const seed: u64 = 0x61726561_0004;
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(seed));
    const random = prng.random();

    var found_count: usize = 0;
    var out_of_nodes_count: usize = 0;
    for (world_cases, 0..) |world_case, world_index| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{world_case.name});
        var world: support.World = undefined;
        try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        defer world.deinit();
        var fixture: Fixture = undefined;
        try fixture.init(gpa, .of(&world));
        defer fixture.deinit();
        const capacity = fixture.outputs.capacity;

        for (0..cases_per_world) |i| {
            const case = world_index * cases_per_world + i;
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });

            fixture.filter.randomize(random);
            const queries = fixture.pickQueries(random);
            const start = fixture.randomStart(random);
            var corners: [shape_corner_count_max][3]f32 = undefined;
            const shape = randomShape(random, &corners, start.pos);

            // Once with room for everything, then with a buffer sized around
            // the result and any mix of outputs.
            const full = try expectSameShape(&fixture, queries, .{
                .start_ref = start.ref,
                .verts = shape,
                .count_max = capacity,
            });
            const sized_max = randomCountMax(random, full.count, capacity);
            const sized = try expectSameShape(&fixture, queries, .{
                .start_ref = start.ref,
                .verts = shape,
                .asked = @bitCast(random.int(u3)),
                .count_max = sized_max,
            });
            if (!full.status.succeeded) continue;

            // The search itself does not depend on the buffer.
            try testing.expect(!full.status.buffer_too_small);
            try testing.expect(sized.status.succeeded);
            try testing.expectEqual(@min(full.count, sized_max), sized.count);
            try testing.expectEqual(sized_max < full.count, sized.status.buffer_too_small);
            try testing.expectEqual(full.status.out_of_nodes, sized.status.out_of_nodes);
            found_count += 1;
            if (full.status.out_of_nodes) out_of_nodes_count += 1;
        }
    }
    try testing.expect(found_count > 0);
    try testing.expect(out_of_nodes_count > 0);
}

test "findPolysAroundShape: corner counts, windings and flat shapes by hand" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, edge_world.obj_text, edge_world.config, edge_world.options);
    defer world.deinit();
    var fixture: Fixture = undefined;
    try fixture.init(gpa, .of(&world));
    defer fixture.deinit();
    const capacity = fixture.outputs.capacity;
    const starts = edgeStarts(&fixture);
    var corners: [shape_corner_count_max][3]f32 = undefined;

    for (starts, 0..) |start, start_index| {
        errdefer std.debug.print("  start {d}\n", .{start_index});
        fixture.filter.reset();

        // Fewer than three corners are no shape: refused, the count cleared.
        for (0..3) |corner_count| {
            const shape = corners[0..corner_count];
            regularShape(shape, start.pos, 4.0, 0.0, false);
            const refused = try expectSameShape(&fixture, &fixture.roomy, .{
                .start_ref = start.ref,
                .verts = shape,
                .count_max = capacity,
            });
            try testing.expect(refused.status.failed and refused.status.invalid_param);
            try testing.expectEqual(0, refused.count);
        }

        // Three corners to twelve, both windings, from within one polygon to
        // past the whole mesh.
        for ([_]usize{ 3, 4, 5, 6, 8, shape_corner_count_max }) |corner_count| {
            const shape = corners[0..corner_count];
            for ([_]f32{ 0.0, 1e-4, 0.5, 4.0, 1000.0 }) |radius| {
                for ([_]bool{ false, true }) |clockwise| {
                    errdefer std.debug.print(
                        "  {d} corners, radius {d}, clockwise {}\n",
                        .{ corner_count, radius, clockwise },
                    );
                    regularShape(shape, start.pos, radius, 0.3, clockwise);
                    const result = try expectSameShape(&fixture, &fixture.roomy, .{
                        .start_ref = start.ref,
                        .verts = shape,
                        .count_max = capacity,
                    });
                    // The start polygon is always found, whatever the shape.
                    try testing.expect(result.status.succeeded);
                    try testing.expect(result.count >= 1);
                    try testing.expectEqual(start.ref, fixture.outputs.actual().refs[0]);
                    _ = try expectSameShape(&fixture, &fixture.cramped, .{
                        .start_ref = start.ref,
                        .verts = shape,
                        .count_max = capacity,
                    });
                }
            }
        }

        // Every corner in one place, and every corner on one line.
        for (3..7) |corner_count| {
            const shape = corners[0..corner_count];
            @memset(shape, start.pos);
            const point = try expectSameShape(&fixture, &fixture.roomy, .{
                .start_ref = start.ref,
                .verts = shape,
                .count_max = capacity,
            });
            try testing.expect(point.status.succeeded);
            lineShape(shape, start.pos);
            const line = try expectSameShape(&fixture, &fixture.roomy, .{
                .start_ref = start.ref,
                .verts = shape,
                .count_max = capacity,
            });
            try testing.expect(line.status.succeeded);
        }
    }

    // A shape that is nowhere near its start polygon.
    fixture.filter.reset();
    const shape = corners[0..5];
    regularShape(shape, starts[2].pos, 6.0, 0.0, false);
    _ = try expectSameShape(&fixture, &fixture.roomy, .{
        .start_ref = starts[0].ref,
        .verts = shape,
        .count_max = capacity,
    });

    // No polygon at all, with a good shape.
    const refused = try expectSameShape(&fixture, &fixture.roomy, .{
        .start_ref = 0,
        .verts = shape,
        .count_max = capacity,
    });
    try testing.expect(refused.status.failed and refused.status.invalid_param);
    try testing.expectEqual(0, refused.count);
    const top_bit: PolyRef = @as(PolyRef, 1) << (@bitSizeOf(PolyRef) - 1);
    for (fixture.polygons.past_end.items) |ref| {
        for ([_]PolyRef{ ref, (ref - 1) ^ top_bit }) |bad_ref| {
            _ = try expectSameShape(&fixture, &fixture.roomy, .{
                .start_ref = bad_ref,
                .verts = shape,
                .count_max = capacity,
            });
        }
    }
}

test "findPolysAroundShape: buffers, filters and small pools by hand" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, edge_world.obj_text, edge_world.config, edge_world.options);
    defer world.deinit();
    var fixture: Fixture = undefined;
    try fixture.init(gpa, .of(&world));
    defer fixture.deinit();
    const capacity = fixture.outputs.capacity;
    const starts = edgeStarts(&fixture);
    var corners: [5][3]f32 = undefined;

    for (starts, 0..) |start, start_index| {
        errdefer std.debug.print("  start {d}\n", .{start_index});
        regularShape(&corners, start.pos, 9.0, 0.7, start_index % 2 == 0);

        // Buffers of no room, one slot, one short, exact and one over.
        fixture.filter.reset();
        const full = try expectSameShape(&fixture, &fixture.roomy, .{
            .start_ref = start.ref,
            .verts = &corners,
            .count_max = capacity,
        });
        try testing.expect(full.status.succeeded);
        const n = full.count;
        const found = fixture.outputs.actual();
        try testing.expectEqual(start.ref, found.refs[0]);
        try testing.expectEqual(0, found.parents[0]);
        try support.expectFloat(0.0, found.costs[0]);

        for ([_]usize{ 0, 1, n -| 1, n, n + 1 }) |count_max| {
            var asked_bits: u3 = 0;
            while (true) : (asked_bits += 1) {
                const sized = try expectSameShape(&fixture, &fixture.roomy, .{
                    .start_ref = start.ref,
                    .verts = &corners,
                    .asked = @bitCast(asked_bits),
                    .count_max = count_max,
                });
                try testing.expect(sized.status.succeeded);
                try testing.expectEqual(@min(n, count_max), sized.count);
                try testing.expectEqual(count_max < n, sized.status.buffer_too_small);
                if (asked_bits == std.math.maxInt(u3)) break;
            }
        }

        // A filter that passes nothing still reports the start.
        fixture.filter.passNothing();
        const alone = try expectSameShape(&fixture, &fixture.roomy, .{
            .start_ref = start.ref,
            .verts = &corners,
            .count_max = capacity,
        });
        try testing.expect(alone.status.succeeded);
        try testing.expectEqual(1, alone.count);

        // Costly, cheap and free areas, one flag shut out; then every step
        // free, so that all nodes tie.
        fixture.filter.reset();
        fixture.filter.setAreaCost(0, 7.5);
        fixture.filter.setAreaCost(1, 0.125);
        fixture.filter.setAreaCost(2, 0.0);
        fixture.filter.setFlags(0xffff, 8);
        _ = try expectSameShape(&fixture, &fixture.roomy, .{
            .start_ref = start.ref,
            .verts = &corners,
            .count_max = capacity,
        });
        fixture.filter.reset();
        fixture.filter.setEveryAreaCost(0.0);
        _ = try expectSameShape(&fixture, &fixture.roomy, .{
            .start_ref = start.ref,
            .verts = &corners,
            .count_max = capacity,
        });
    }

    // Pools of a handful of nodes run out under a shape wider than the mesh.
    // Only one winding reaches past the start polygon, so both are asked.
    fixture.filter.reset();
    var out_of_nodes_seen = false;
    for (tiny_node_counts) |node_count| {
        errdefer std.debug.print("  pool of {d} nodes\n", .{node_count});
        var queries: QueryPair = try .create();
        defer queries.deinit(gpa);
        try queries.attach(gpa, fixture.mesh, node_count);
        for (starts) |start| {
            for ([_]f32{ 0.5, 4.0, 1000.0 }) |radius| {
                for ([_]bool{ false, true }) |clockwise| {
                    regularShape(&corners, start.pos, radius, 0.0, clockwise);
                    for ([_]usize{ 0, 2, capacity }) |count_max| {
                        const result = try expectSameShape(&fixture, &queries, .{
                            .start_ref = start.ref,
                            .verts = &corners,
                            .count_max = count_max,
                        });
                        try testing.expect(result.count <= node_count);
                        if (result.status.out_of_nodes) out_of_nodes_seen = true;
                    }
                }
            }
        }
    }
    try testing.expect(out_of_nodes_seen);
}

// ---------------------------------------------------------------------------
// getPathFromDijkstraSearch
// ---------------------------------------------------------------------------

test "getPathFromDijkstraSearch: random searches, ends and buffers match" {
    const gpa = testing.allocator;
    const seed: u64 = 0x61726561_0005;
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(seed));
    const random = prng.random();

    var visited: std.ArrayList(PolyRef) = .empty;
    defer visited.deinit(gpa);
    var path_count: usize = 0;
    for (world_cases, 0..) |world_case, world_index| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{world_case.name});
        var world: support.World = undefined;
        try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        defer world.deinit();
        var fixture: Fixture = undefined;
        try fixture.init(gpa, .of(&world));
        defer fixture.deinit();
        const capacity = fixture.outputs.capacity;
        visited.clearRetainingCapacity();

        for (0..cases_per_world) |i| {
            const case = world_index * cases_per_world + i;
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });

            // One of the two searches. A refused one leaves the pool as the
            // search before it did, and the paths are read from that.
            fixture.filter.randomize(random);
            const queries = fixture.pickQueries(random);
            const start = fixture.randomStart(random);
            var corners: [shape_corner_count_max][3]f32 = undefined;
            const search = if (random.boolean())
                try expectSameCircle(&fixture, queries, .{
                    .start = start,
                    .radius = randomRadius(random),
                    .count_max = capacity,
                })
            else
                try expectSameShape(&fixture, queries, .{
                    .start_ref = start.ref,
                    .verts = randomShape(random, &corners, start.pos),
                    .count_max = capacity,
                });
            if (search.status.succeeded) {
                visited.clearRetainingCapacity();
                try visited.appendSlice(gpa, fixture.outputs.actual().refs[0..search.count]);
            }

            for (0..3) |_| {
                // Mostly a polygon the search reached, else any polygon or a
                // reference that is none.
                const end_ref = switch (random.uintLessThan(u32, 8)) {
                    0 => fixture.polygons.randomBadRef(random),
                    1, 2 => pickFrom(fixture.polygons.all.items, random),
                    else => if (visited.items.len > 0)
                        pickFrom(visited.items, random)
                    else
                        start.ref,
                };
                const full = try expectSameDijkstraPath(&fixture, queries, end_ref, capacity);
                const path = fixture.outputs.actual().refs;
                const path_start = path[0];
                const path_end = if (full.status.succeeded) path[full.count - 1] else 0;
                const path_max = randomCountMax(random, full.count, capacity);
                const sized = try expectSameDijkstraPath(&fixture, queries, end_ref, path_max);
                if (!full.status.succeeded) continue;

                // A path runs from the search's start to the polygon asked
                // for, and a short buffer keeps its beginning.
                try testing.expect(full.count >= 1);
                try testing.expect(!full.status.buffer_too_small);
                try testing.expectEqual(end_ref, path_end);
                if (search.status.succeeded) try testing.expectEqual(start.ref, path_start);
                try testing.expect(sized.status.succeeded);
                try testing.expectEqual(@min(full.count, path_max), sized.count);
                try testing.expectEqual(path_max < full.count, sized.status.buffer_too_small);
                if (path_max > 0) try testing.expectEqual(path_start, path[0]);
                path_count += 1;
            }
        }
    }
    try testing.expect(path_count > 0);
}

/// Starts a sliced path search on both sides. It leaves the start polygon's
/// node open and not yet closed.
fn startSlicedSearch(fixture: *Fixture, queries: *QueryPair, start: Start, end: Start) !void {
    const expected = reference.ref_query_init_sliced_find_path(
        queries.original,
        start.ref,
        end.ref,
        &start.pos,
        &end.pos,
        fixture.filter.original,
        0,
    );
    const actual = queries.port.initSlicedFindPath(
        start.ref,
        end.ref,
        start.pos,
        end.pos,
        &fixture.filter.port,
        .{},
    );
    try support.expectStatus(expected, actual);
    try testing.expect(actual.in_progress);
    try expectSameNodePools(queries);
}

/// Runs a sliced path search on for up to `iteration_count_max` polygons on
/// both sides.
fn stepSlicedSearch(queries: *QueryPair, iteration_count_max: i32) !void {
    var expected_done: i32 = -1;
    const expected = reference.ref_query_update_sliced_find_path(
        queries.original,
        iteration_count_max,
        &expected_done,
    );
    var actual_done: i32 = -1;
    const actual = queries.port.updateSlicedFindPath(iteration_count_max, &actual_done);
    try support.expectStatus(expected, actual);
    try testing.expectEqual(expected_done, actual_done);
    try expectSameNodePools(queries);
}

/// Reads the path to `end_ref` with buffers of no room, one slot, one short,
/// exact, one over and plenty, and expects what a path of that length gives.
/// Returns the path's length, or null when there is no path.
fn expectSamePathInEveryBuffer(
    fixture: *Fixture,
    queries: *QueryPair,
    end_ref: PolyRef,
) !?usize {
    const capacity = fixture.outputs.capacity;
    const full = try expectSameDijkstraPath(fixture, queries, end_ref, capacity);
    if (!full.status.succeeded) {
        for ([_]usize{ 0, 1, 2 }) |path_max| {
            const again = try expectSameDijkstraPath(fixture, queries, end_ref, path_max);
            try testing.expect(again.status.failed);
            try testing.expectEqual(full.count, again.count);
        }
        return null;
    }

    const n = full.count;
    const path = fixture.outputs.actual().refs;
    const path_start = path[0];
    try testing.expectEqual(end_ref, path[n - 1]);
    for ([_]usize{ 0, 1, n -| 1, n, n + 1 }) |path_max| {
        const sized = try expectSameDijkstraPath(fixture, queries, end_ref, path_max);
        try testing.expect(sized.status.succeeded);
        try testing.expectEqual(@min(n, path_max), sized.count);
        try testing.expectEqual(path_max < n, sized.status.buffer_too_small);
        // A short buffer keeps the beginning of the path.
        if (path_max > 0) try testing.expectEqual(path_start, path[0]);
        if (path_max >= n) try testing.expectEqual(end_ref, path[n - 1]);
    }
    return n;
}

test "getPathFromDijkstraSearch: before a search, after each search and outside it" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, edge_world.obj_text, edge_world.config, edge_world.options);
    defer world.deinit();
    var fixture: Fixture = undefined;
    try fixture.init(gpa, .of(&world));
    defer fixture.deinit();
    const capacity = fixture.outputs.capacity;
    const starts = edgeStarts(&fixture);
    const all = fixture.polygons.all.items;
    const top_bit: PolyRef = @as(PolyRef, 1) << (@bitSizeOf(PolyRef) - 1);

    var visited: std.ArrayList(PolyRef) = .empty;
    defer visited.deinit(gpa);

    // No search has run: no polygon has a node, so none has a path. A
    // reference that is no polygon is refused before the count is cleared.
    {
        var queries: QueryPair = try .create();
        defer queries.deinit(gpa);
        try queries.attach(gpa, fixture.mesh, 64);
        for (starts) |start| {
            const result = try expectSameDijkstraPath(&fixture, &queries, start.ref, capacity);
            try testing.expect(result.status.failed and result.status.invalid_param);
            try testing.expectEqual(0, result.count);
            const length = try expectSamePathInEveryBuffer(&fixture, &queries, start.ref);
            try testing.expect(length == null);
        }
        const refused = try expectSameDijkstraPath(&fixture, &queries, 0, capacity);
        try testing.expect(refused.status.failed and refused.status.invalid_param);
        try testing.expectEqual(count_sentinel, refused.count);
    }

    var corners: [5][3]f32 = undefined;
    for (starts, 0..) |start, start_index| {
        for ([_]bool{ false, true }) |around_shape| {
            for ([_]*QueryPair{ &fixture.roomy, &fixture.cramped }) |queries| {
                errdefer std.debug.print(
                    "  start {d}, around a shape {}\n",
                    .{ start_index, around_shape },
                );
                fixture.filter.reset();
                // The winding that reaches past the start polygon.
                regularShape(&corners, start.pos, 10.0, 0.0, true);
                const search = if (around_shape)
                    try expectSameShape(&fixture, queries, .{
                        .start_ref = start.ref,
                        .verts = &corners,
                        .count_max = capacity,
                    })
                else
                    try expectSameCircle(&fixture, queries, .{
                        .start = start,
                        .radius = 10.0,
                        .count_max = capacity,
                    });
                try testing.expect(search.status.succeeded);
                visited.clearRetainingCapacity();
                try visited.appendSlice(gpa, fixture.outputs.actual().refs[0..search.count]);

                // The polygons the search reached each have a path from the
                // start, no longer than the polygons reached. The start's
                // path is the start alone.
                const visit_stride = @max(1, visited.items.len / 48);
                var visit_index: usize = 0;
                while (visit_index < visited.items.len) : (visit_index += visit_stride) {
                    const end_ref = visited.items[visit_index];
                    const length = try expectSamePathInEveryBuffer(&fixture, queries, end_ref);
                    try testing.expect(length != null);
                    try testing.expect(length.? <= visited.items.len);
                    try testing.expectEqual(start.ref, fixture.outputs.actual().refs[0]);
                    if (visit_index == 0) try testing.expectEqual(1, length.?);
                }

                // Polygons outside the search have none.
                const outside_stride = @max(1, all.len / 48);
                var outside_index: usize = start_index % outside_stride;
                while (outside_index < all.len) : (outside_index += outside_stride) {
                    const end_ref = all[outside_index];
                    if (std.mem.indexOfScalar(PolyRef, visited.items, end_ref) != null) continue;
                    const length = try expectSamePathInEveryBuffer(&fixture, queries, end_ref);
                    try testing.expect(length == null);
                    const outside = try expectSameDijkstraPath(&fixture, queries, end_ref, 4);
                    try testing.expect(outside.status.failed and outside.status.invalid_param);
                    try testing.expectEqual(0, outside.count);
                }

                // References that are no polygon.
                const refused = try expectSameDijkstraPath(&fixture, queries, 0, capacity);
                try testing.expect(refused.status.failed and refused.status.invalid_param);
                try testing.expectEqual(count_sentinel, refused.count);
                for (fixture.polygons.past_end.items) |ref| {
                    for ([_]PolyRef{ ref, (ref - 1) ^ top_bit }) |bad_ref| {
                        _ = try expectSameDijkstraPath(&fixture, queries, bad_ref, capacity);
                        _ = try expectSameDijkstraPath(&fixture, queries, bad_ref, 0);
                    }
                }
            }
        }
    }
}

/// A polygon with a node in a pool, and whether that node is closed.
const PoolNode = struct {
    ref: PolyRef,
    closed: bool,
};

/// Lists the nodes of the port's pool. The original's pool holds the same:
/// every call on both sides ends by comparing the two.
fn readPoolNodes(
    gpa: std.mem.Allocator,
    queries: *QueryPair,
    nodes: *std.ArrayList(PoolNode),
) !void {
    nodes.clearRetainingCapacity();
    const pool = queries.port.getNodePool();
    const node_count: u32 = @intCast(pool.getNodeCount());
    var node_index: u32 = 1;
    while (node_index <= node_count) : (node_index += 1) {
        const node = pool.getNodeAtIdx(node_index) orelse return error.TestUnexpectedResult;
        try nodes.append(gpa, .{ .ref = node.id, .closed = node.bits.flags.closed });
    }
}

/// Reads the path to some sixty of the pool's nodes: a closed node has one,
/// an open one is refused with the count cleared. Returns how many of the
/// nodes read were open.
fn expectPathsFollowNodes(
    fixture: *Fixture,
    queries: *QueryPair,
    nodes: []const PoolNode,
) !usize {
    var open_count: usize = 0;
    const stride = @max(1, nodes.len / 64);
    var node_index: usize = 0;
    while (node_index < nodes.len) : (node_index += stride) {
        const node = nodes[node_index];
        errdefer std.debug.print("  at node {d}, closed {}\n", .{ node_index, node.closed });
        const length = try expectSamePathInEveryBuffer(fixture, queries, node.ref);
        try testing.expectEqual(node.closed, length != null);
        if (node.closed) continue;

        open_count += 1;
        const refused = try expectSameDijkstraPath(fixture, queries, node.ref, 8);
        try testing.expect(refused.status.failed and refused.status.invalid_param);
        try testing.expectEqual(0, refused.count);
    }
    return open_count;
}

test "getPathFromDijkstraSearch: an open node of a sliced search has no path" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, edge_world.obj_text, edge_world.config, edge_world.options);
    defer world.deinit();
    var fixture: Fixture = undefined;
    try fixture.init(gpa, .of(&world));
    defer fixture.deinit();
    const starts = edgeStarts(&fixture);
    const start = starts[0];
    const end = starts[2];
    try testing.expect(start.ref != end.ref);

    var nodes: std.ArrayList(PoolNode) = .empty;
    defer nodes.deinit(gpa);

    fixture.filter.reset();
    for ([_]*QueryPair{ &fixture.roomy, &fixture.cramped }) |queries| {
        // Starting the search opens the start polygon's node and no other.
        try startSlicedSearch(&fixture, queries, start, end);
        try readPoolNodes(gpa, queries, &nodes);
        try testing.expectEqual(1, nodes.items.len);
        try testing.expectEqual(start.ref, nodes.items[0].ref);
        try testing.expect(!nodes.items[0].closed);
        try testing.expectEqual(1, try expectPathsFollowNodes(&fixture, queries, nodes.items));

        // The end polygon has no node at all.
        const unseen = try expectSamePathInEveryBuffer(&fixture, queries, end.ref);
        try testing.expect(unseen == null);

        // One step closes the start, whose path is then itself alone, and
        // opens its neighbours. Further steps close some and open more.
        for ([_]i32{ 1, 1, 2, 5, 40, 2000 }, 0..) |iteration_count, step_index| {
            errdefer std.debug.print("  at step {d}\n", .{step_index});
            try stepSlicedSearch(queries, iteration_count);
            try readPoolNodes(gpa, queries, &nodes);
            try testing.expect(nodes.items[0].closed);
            _ = try expectPathsFollowNodes(&fixture, queries, nodes.items);
            const length = try expectSamePathInEveryBuffer(&fixture, queries, start.ref);
            try testing.expect(length != null and length.? == 1);
        }
    }
}

// ---------------------------------------------------------------------------
// findLocalNeighbourhood
// ---------------------------------------------------------------------------

test "findLocalNeighbourhood: random starts, radii, filters and buffers match" {
    const gpa = testing.allocator;
    const seed: u64 = 0x61726561_0006;
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(seed));
    const random = prng.random();

    var found_count: usize = 0;
    var neighbour_count: usize = 0;
    for (world_cases, 0..) |world_case, world_index| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{world_case.name});
        var world: support.World = undefined;
        try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        defer world.deinit();
        var fixture: Fixture = undefined;
        try fixture.init(gpa, .of(&world));
        defer fixture.deinit();
        const capacity = fixture.outputs.capacity;

        for (0..cases_per_world) |i| {
            const case = world_index * cases_per_world + i;
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });

            fixture.filter.randomize(random);
            const queries = fixture.pickQueries(random);
            const start = fixture.randomStart(random);
            const radius = randomRadius(random);

            // Once with room for everything, then with a buffer sized around
            // the result, with or without the parents.
            const full = try expectSameLocal(&fixture, queries, .{
                .start = start,
                .radius = radius,
                .count_max = capacity,
            });
            const sized_max = randomCountMax(random, full.count, capacity);
            const sized = try expectSameLocal(&fixture, queries, .{
                .start = start,
                .radius = radius,
                .with_parents = random.boolean(),
                .count_max = sized_max,
            });
            if (!full.status.succeeded) continue;

            // A short buffer holds the beginning of the same neighbourhood.
            try testing.expect(!full.status.buffer_too_small);
            try testing.expect(sized.status.succeeded);
            try testing.expectEqual(@min(full.count, sized_max), sized.count);
            try testing.expectEqual(sized_max < full.count, sized.status.buffer_too_small);
            found_count += 1;
            if (full.count > 1) neighbour_count += 1;
        }
    }
    try testing.expect(found_count > 0);
    try testing.expect(neighbour_count > 0);
}

test "findLocalNeighbourhood: every radius, with and without parents, by hand" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, edge_world.obj_text, edge_world.config, edge_world.options);
    defer world.deinit();
    var fixture: Fixture = undefined;
    try fixture.init(gpa, .of(&world));
    defer fixture.deinit();
    const capacity = fixture.outputs.capacity;
    const starts = edgeStarts(&fixture);

    for (starts, 0..) |start, start_index| {
        for (edge_radii, 0..) |radius, radius_index| {
            errdefer std.debug.print("  start {d}, radius {d}\n", .{ start_index, radius_index });
            // A build without finite checks takes an endless radius, and one
            // that is no number, as it finds them.
            const checked = !build_options.fast_math;
            const refused = radius < 0.0 or (checked and !std.math.isFinite(radius));

            fixture.filter.reset();
            for ([_]bool{ true, false }) |with_parents| {
                const result = try expectSameLocal(&fixture, &fixture.roomy, .{
                    .start = start,
                    .radius = radius,
                    .with_parents = with_parents,
                    .count_max = capacity,
                });
                if (refused) {
                    try testing.expect(result.status.failed and result.status.invalid_param);
                    try testing.expectEqual(0, result.count);
                    continue;
                }
                // The start polygon comes first, whatever the radius.
                const found = fixture.outputs.actual();
                try testing.expect(result.status.succeeded);
                try testing.expect(!result.status.buffer_too_small);
                try testing.expect(result.count >= 1);
                try testing.expectEqual(start.ref, found.refs[0]);
                if (with_parents) try testing.expectEqual(0, found.parents[0]);
            }

            // Flags that keep some neighbours out, then a query short of
            // nodes for the other searches: this one has nodes of its own.
            fixture.filter.setFlags(0xffff, 8);
            _ = try expectSameLocal(&fixture, &fixture.roomy, .{
                .start = start,
                .radius = radius,
                .count_max = capacity,
            });
            fixture.filter.setFlags(0x0006, 0);
            _ = try expectSameLocal(&fixture, &fixture.cramped, .{
                .start = start,
                .radius = radius,
                .count_max = capacity,
            });
        }
    }
}

test "findLocalNeighbourhood: buffers, filters and refused inputs by hand" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, edge_world.obj_text, edge_world.config, edge_world.options);
    defer world.deinit();
    var fixture: Fixture = undefined;
    try fixture.init(gpa, .of(&world));
    defer fixture.deinit();
    const capacity = fixture.outputs.capacity;
    const starts = edgeStarts(&fixture);

    var neighbour_seen = false;
    for (starts, 0..) |start, start_index| {
        errdefer std.debug.print("  start {d}\n", .{start_index});
        fixture.filter.reset();
        const full = try expectSameLocal(&fixture, &fixture.roomy, .{
            .start = start,
            .radius = 12.0,
            .count_max = capacity,
        });
        try testing.expect(full.status.succeeded);
        const n = full.count;
        try testing.expect(n >= 1);
        if (n > 1) neighbour_seen = true;

        // No polygon comes twice, and each after the start was reached from
        // one found before it.
        const found = fixture.outputs.actual();
        for (1..n) |i| {
            const before = found.refs[0..i];
            try testing.expect(std.mem.indexOfScalar(PolyRef, before, found.refs[i]) == null);
            try testing.expect(std.mem.indexOfScalar(PolyRef, before, found.parents[i]) != null);
        }

        // Buffers of no room, one slot, one short, exact and one over.
        for ([_]usize{ 0, 1, n -| 1, n, n + 1 }) |count_max| {
            for ([_]bool{ true, false }) |with_parents| {
                const sized = try expectSameLocal(&fixture, &fixture.roomy, .{
                    .start = start,
                    .radius = 12.0,
                    .with_parents = with_parents,
                    .count_max = count_max,
                });
                try testing.expect(sized.status.succeeded);
                try testing.expectEqual(@min(n, count_max), sized.count);
                try testing.expectEqual(count_max < n, sized.status.buffer_too_small);
            }
        }

        // A filter that passes nothing leaves the start alone in its
        // neighbourhood: the start polygon is never put to the filter.
        fixture.filter.passNothing();
        const alone = try expectSameLocal(&fixture, &fixture.roomy, .{
            .start = start,
            .radius = 1000.0,
            .count_max = capacity,
        });
        try testing.expect(alone.status.succeeded);
        try testing.expectEqual(1, alone.count);
    }
    try testing.expect(neighbour_seen);

    // The search has nodes of its own: a query with a handful of nodes for
    // the other searches finds the same neighbourhood as one with plenty.
    fixture.filter.reset();
    for (tiny_node_counts) |node_count| {
        errdefer std.debug.print("  pool of {d} nodes\n", .{node_count});
        var queries: QueryPair = try .create();
        defer queries.deinit(gpa);
        try queries.attach(gpa, fixture.mesh, node_count);
        for (starts) |start| {
            for ([_]f32{ 0.0, 2.0, 1000.0 }) |radius| {
                const roomy = try expectSameLocal(&fixture, &fixture.roomy, .{
                    .start = start,
                    .radius = radius,
                    .count_max = capacity,
                });
                const tiny = try expectSameLocal(&fixture, &queries, .{
                    .start = start,
                    .radius = radius,
                    .count_max = capacity,
                });
                try testing.expect(tiny.status.succeeded);
                try testing.expectEqual(roomy.count, tiny.count);
            }
        }
    }

    // References no polygon answers to: refused, the count cleared and the
    // buffers left alone.
    const no_ref: Start = .{ .ref = 0, .pos = starts[0].pos };
    const refused = try expectSameLocal(&fixture, &fixture.roomy, .{
        .start = no_ref,
        .radius = 3.0,
        .count_max = capacity,
    });
    try testing.expect(refused.status.failed and refused.status.invalid_param);
    try testing.expectEqual(0, refused.count);
    const top_bit: PolyRef = @as(PolyRef, 1) << (@bitSizeOf(PolyRef) - 1);
    for (fixture.polygons.past_end.items) |ref| {
        for ([_]PolyRef{ ref, (ref - 1) ^ top_bit }) |bad_ref| {
            for ([_]usize{ 0, capacity }) |count_max| {
                _ = try expectSameLocal(&fixture, &fixture.roomy, .{
                    .start = .{ .ref = bad_ref, .pos = starts[0].pos },
                    .radius = 3.0,
                    .count_max = count_max,
                });
            }
        }
    }

    // A centre far from its start polygon.
    for (starts) |start| {
        const far: Start = .{ .ref = start.ref, .pos = far_center };
        for ([_]f32{ 0.0, 2.0, 1e30, std.math.floatMax(f32) }) |radius| {
            _ = try expectSameLocal(&fixture, &fixture.roomy, .{
                .start = far,
                .radius = radius,
                .count_max = capacity,
            });
        }
    }

    // A centre that is no point at all.
    if (!build_options.fast_math) {
        for (non_finite_centers, 0..) |center, center_index| {
            errdefer std.debug.print("  centre {d}\n", .{center_index});
            const result = try expectSameLocal(&fixture, &fixture.roomy, .{
                .start = .{ .ref = starts[1].ref, .pos = center },
                .radius = 3.0,
                .count_max = capacity,
            });
            try testing.expect(result.status.failed and result.status.invalid_param);
            try testing.expectEqual(0, result.count);
        }
    }
}

// ---------------------------------------------------------------------------
// getPolyWallSegments
// ---------------------------------------------------------------------------

test "getPolyWallSegments: every polygon of every world, in every buffer" {
    const gpa = testing.allocator;
    var portal_count: usize = 0;
    var split_edge_count: usize = 0;
    for (world_cases) |world_case| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{world_case.name});
        var world: support.World = undefined;
        try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        defer world.deinit();
        var fixture: Fixture = undefined;
        try fixture.init(gpa, .of(&world));
        defer fixture.deinit();
        const queries = &fixture.roomy;

        for (fixture.polygons.all.items, 0..) |ref, poly_index| {
            errdefer std.debug.print("  polygon {d}\n", .{poly_index});
            var corners: [6][3]f32 = undefined;
            const corner_count = polygonCorners(fixture.mesh.original, ref, &corners);

            // Every neighbour passes: each edge gives its portals and the
            // walls between them, or the walls alone when no neighbours are
            // asked for.
            fixture.filter.reset();
            var counts: [2]usize = undefined;
            for ([_]bool{ true, false }, &counts) |with_refs, *count| {
                const full = try expectSameWallSegments(&fixture, queries, .{
                    .ref = ref,
                    .with_refs = with_refs,
                });
                try testing.expect(full.status.succeeded);
                try testing.expect(!full.status.buffer_too_small);
                const n = full.count;
                count.* = n;

                // Buffers of no room, one slot, one short, exact and one over.
                const over = @min(n + 1, segment_capacity);
                for ([_]usize{ 0, 1, n -| 1, n, over }) |count_max| {
                    const sized = try expectSameWallSegments(&fixture, queries, .{
                        .ref = ref,
                        .with_refs = with_refs,
                        .count_max = count_max,
                    });
                    try testing.expect(sized.status.succeeded);
                    try testing.expectEqual(@min(n, count_max), sized.count);
                    try testing.expectEqual(count_max < n, sized.status.buffer_too_small);
                }
            }
            // With neighbours every edge gives at least one segment, and the
            // portals are what the walls alone leave out.
            try testing.expect(counts[0] >= corner_count);
            try testing.expect(counts[1] <= counts[0]);
            portal_count += counts[0] - counts[1];
            if (counts[0] > corner_count) split_edge_count += 1;

            // Flags that pass some neighbours and not others.
            for ([_][2]u16{ .{ 0xffff, 8 }, .{ 0x0006, 0 }, .{ 0x0008, 0 } }) |flags| {
                fixture.filter.setFlags(flags[0], flags[1]);
                for ([_]bool{ true, false }) |with_refs| {
                    const some = try expectSameWallSegments(&fixture, queries, .{
                        .ref = ref,
                        .with_refs = with_refs,
                    });
                    try testing.expect(some.status.succeeded);
                }
            }

            // No neighbour passes: every edge is one whole wall.
            fixture.filter.passNothing();
            for ([_]bool{ true, false }) |with_refs| {
                const walled = try expectSameWallSegments(&fixture, queries, .{
                    .ref = ref,
                    .with_refs = with_refs,
                });
                try testing.expect(walled.status.succeeded);
                try testing.expectEqual(corner_count, walled.count);
            }
        }
    }
    // The worlds have portals, and edges on tile borders that more than one
    // segment shares.
    errdefer std.debug.print(
        "  {d} portals, {d} split edges\n",
        .{ portal_count, split_edge_count },
    );
    try testing.expect(portal_count > 0);
    try testing.expect(split_edge_count > 0);
}

test "getPolyWallSegments: random polygons, filters and buffers match" {
    const gpa = testing.allocator;
    const seed: u64 = 0x61726561_0007;
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(seed));
    const random = prng.random();

    var found_count: usize = 0;
    var refused_count: usize = 0;
    for (world_cases, 0..) |world_case, world_index| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{world_case.name});
        var world: support.World = undefined;
        try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        defer world.deinit();
        var fixture: Fixture = undefined;
        try fixture.init(gpa, .of(&world));
        defer fixture.deinit();
        const border = fixture.polygons.border.items;

        for (0..cases_per_world) |i| {
            const case = world_index * cases_per_world + i;
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });

            // Tile borders are where an edge splits into several segments,
            // so half the polygons come from there when the world has any.
            fixture.filter.randomize(random);
            const queries = fixture.pickQueries(random);
            var ref = fixture.polygons.pick(random);
            if (border.len > 0 and random.boolean()) ref = pickFrom(border, random);
            if (random.uintLessThan(u32, 16) == 0) ref = fixture.polygons.randomBadRef(random);
            const with_refs = random.boolean();

            const full = try expectSameWallSegments(&fixture, queries, .{
                .ref = ref,
                .with_refs = with_refs,
            });
            const sized_max = randomCountMax(random, full.count, segment_capacity);
            const sized = try expectSameWallSegments(&fixture, queries, .{
                .ref = ref,
                .with_refs = with_refs,
                .count_max = sized_max,
            });
            if (!full.status.succeeded) {
                // A reference that is no polygon: the count is cleared.
                try testing.expect(full.status.invalid_param);
                try testing.expectEqual(0, full.count);
                try testing.expectEqual(0, sized.count);
                refused_count += 1;
                continue;
            }

            try testing.expect(!full.status.buffer_too_small);
            try testing.expect(sized.status.succeeded);
            try testing.expectEqual(@min(full.count, sized_max), sized.count);
            try testing.expectEqual(sized_max < full.count, sized.status.buffer_too_small);
            found_count += 1;
        }
    }
    try testing.expect(found_count > 0);
    try testing.expect(refused_count > 0);
}

test "getPolyWallSegments: off-mesh connections and refused references by hand" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, edge_world.obj_text, edge_world.config, edge_world.options);
    defer world.deinit();
    var fixture: Fixture = undefined;
    try fixture.init(gpa, .of(&world));
    defer fixture.deinit();
    const queries = &fixture.roomy;
    const off_mesh = fixture.polygons.off_mesh.items;
    try testing.expect(off_mesh.len > 0);
    try testing.expect(fixture.polygons.border.items.len > 0);

    // An off-mesh connection is two corners with no neighbour along either
    // edge between them: two walls, whatever the filter passes.
    for (off_mesh, 0..) |ref, connection_index| {
        errdefer std.debug.print("  connection {d}\n", .{connection_index});
        for ([_][2]u16{ .{ 0xffff, 0 }, .{ 0xffff, 1 }, .{ 0x0006, 0 } }) |flags| {
            fixture.filter.setFlags(flags[0], flags[1]);
            for ([_]bool{ true, false }) |with_refs| {
                const both = try expectSameWallSegments(&fixture, queries, .{
                    .ref = ref,
                    .with_refs = with_refs,
                });
                try testing.expect(both.status.succeeded);
                try testing.expect(!both.status.buffer_too_small);
                try testing.expectEqual(2, both.count);

                const one = try expectSameWallSegments(&fixture, queries, .{
                    .ref = ref,
                    .with_refs = with_refs,
                    .count_max = 1,
                });
                try testing.expect(one.status.succeeded and one.status.buffer_too_small);
                try testing.expectEqual(1, one.count);
            }
        }
    }

    // References no polygon answers to: refused, the count cleared and the
    // buffers left alone, with room or without.
    fixture.filter.reset();
    const top_bit: PolyRef = @as(PolyRef, 1) << (@bitSizeOf(PolyRef) - 1);
    for ([_]bool{ true, false }) |with_refs| {
        for ([_]usize{ 0, 1, segment_capacity }) |count_max| {
            const refused = try expectSameWallSegments(&fixture, queries, .{
                .ref = 0,
                .with_refs = with_refs,
                .count_max = count_max,
            });
            try testing.expect(refused.status.failed and refused.status.invalid_param);
            try testing.expectEqual(0, refused.count);

            for (fixture.polygons.past_end.items) |ref| {
                for ([_]PolyRef{ ref, (ref - 1) ^ top_bit }) |bad_ref| {
                    const result = try expectSameWallSegments(&fixture, queries, .{
                        .ref = bad_ref,
                        .with_refs = with_refs,
                        .count_max = count_max,
                    });
                    if (result.status.failed) try testing.expectEqual(0, result.count);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// findDistanceToWall
// ---------------------------------------------------------------------------

/// How a search for the nearest wall ended.
const WallOutcome = enum { refused, clear, hit };

/// Checks what `findDistanceToWall` returned against what it promises,
/// whatever the original did: a refused call writes nothing; a search that
/// reaches no wall reports the radius and leaves the hit position alone; a
/// hit lies as far from the centre, over the ground, as the distance says,
/// and no farther than the radius. For an accepted call the square of
/// `radius` has to be finite.
fn expectSoundWallDistance(start: Start, radius: f32, result: WallDistance) !WallOutcome {
    if (!result.status.succeeded) {
        try testing.expect(result.status.failed and result.status.invalid_param);
        try testing.expect(result.hit_dist == cost_sentinel);
        try testing.expect(std.meta.eql(point_sentinel, result.hit_pos));
        try testing.expect(std.meta.eql(point_sentinel, result.hit_normal));
        return .refused;
    }
    if (std.meta.eql(point_sentinel, result.hit_pos)) {
        try testing.expect(result.hit_dist == radius);
        return .clear;
    }
    const dx = result.hit_pos[0] - start.pos[0];
    const dz = result.hit_pos[2] - start.pos[2];
    const planar = @sqrt(dx * dx + dz * dz);
    try testing.expect(result.hit_dist <= radius);
    try testing.expectApproxEqAbs(planar, result.hit_dist, 1e-3 * @max(1.0, planar));
    return .hit;
}

test "findDistanceToWall: random starts, radii and filters match" {
    const gpa = testing.allocator;
    const seed: u64 = 0x61726561_0008;
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(seed));
    const random = prng.random();

    var outcome_counts: std.EnumArray(WallOutcome, usize) = .initFill(0);
    for (world_cases, 0..) |world_case, world_index| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{world_case.name});
        var world: support.World = undefined;
        try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        defer world.deinit();
        var fixture: Fixture = undefined;
        try fixture.init(gpa, .of(&world));
        defer fixture.deinit();

        for (0..cases_per_world) |i| {
            const case = world_index * cases_per_world + i;
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });

            fixture.filter.randomize(random);
            const queries = fixture.pickQueries(random);
            const start = fixture.randomStart(random);
            const radius = randomRadius(random);

            const result = try expectSameDistanceToWall(&fixture, queries, start, radius);
            const outcome = try expectSoundWallDistance(start, radius, result);
            outcome_counts.getPtr(outcome).* += 1;
            if (radius < 0.0) try testing.expectEqual(.refused, outcome);
        }
    }
    try testing.expect(outcome_counts.get(.refused) > 0);
    try testing.expect(outcome_counts.get(.clear) > 0);
    try testing.expect(outcome_counts.get(.hit) > 0);
}

test "findDistanceToWall: every radius, pool and kind of filter from starts by hand" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, edge_world.obj_text, edge_world.config, edge_world.options);
    defer world.deinit();
    var fixture: Fixture = undefined;
    try fixture.init(gpa, .of(&world));
    defer fixture.deinit();
    const starts = edgeStarts(&fixture);

    for (starts, 0..) |start, start_index| {
        for (edge_radii, 0..) |radius, radius_index| {
            errdefer std.debug.print("  start {d}, radius {d}\n", .{ start_index, radius_index });
            // A build without finite checks takes an endless radius, and one
            // that is no number, as it finds them.
            const checked = !build_options.fast_math;
            const refused = radius < 0.0 or (checked and !std.math.isFinite(radius));
            // What the function promises can be checked where the radius
            // squared is still a number.
            const sound = refused or std.math.isFinite(radius * radius);

            fixture.filter.reset();
            const roomy = try expectSameDistanceToWall(&fixture, &fixture.roomy, start, radius);
            try testing.expectEqual(refused, !roomy.status.succeeded);
            if (sound) {
                const outcome = try expectSoundWallDistance(start, radius, roomy);
                // With a node for every polygon, a reach past the whole mesh
                // ends at a wall.
                if (radius == 1000.0) try testing.expectEqual(.hit, outcome);
            }

            // Too few nodes.
            const cramped = try expectSameDistanceToWall(&fixture, &fixture.cramped, start, radius);
            try testing.expectEqual(refused, !cramped.status.succeeded);
            if (sound) _ = try expectSoundWallDistance(start, radius, cramped);

            // No neighbour passes: every edge of the start polygon is a
            // wall, and none of them is far from its centre.
            fixture.filter.passNothing();
            const walled = try expectSameDistanceToWall(&fixture, &fixture.roomy, start, radius);
            try testing.expect(!walled.status.out_of_nodes);
            if (sound) {
                const outcome = try expectSoundWallDistance(start, radius, walled);
                if (!refused and radius >= 12.0) try testing.expectEqual(.hit, outcome);
            }

            // Some neighbours pass and others are walls.
            fixture.filter.setFlags(0xffff, 8);
            const some = try expectSameDistanceToWall(&fixture, &fixture.roomy, start, radius);
            if (sound) _ = try expectSoundWallDistance(start, radius, some);
            fixture.filter.setFlags(0x0006, 0);
            const few = try expectSameDistanceToWall(&fixture, &fixture.cramped, start, radius);
            if (sound) _ = try expectSoundWallDistance(start, radius, few);
        }
    }
}

test "findDistanceToWall: small pools, refused inputs and awkward centres by hand" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, edge_world.obj_text, edge_world.config, edge_world.options);
    defer world.deinit();
    var fixture: Fixture = undefined;
    try fixture.init(gpa, .of(&world));
    defer fixture.deinit();
    const starts = edgeStarts(&fixture);
    const reach_max = std.math.floatMax(f32);

    // Pools of a handful of nodes. From a centre so far off that every
    // distance overflows, with a reach that does too, no wall is nearer
    // than another and the search spreads until the nodes run out.
    fixture.filter.reset();
    var out_of_nodes_seen = false;
    for (tiny_node_counts) |node_count| {
        errdefer std.debug.print("  pool of {d} nodes\n", .{node_count});
        var queries: QueryPair = try .create();
        defer queries.deinit(gpa);
        try queries.attach(gpa, fixture.mesh, node_count);
        for (starts) |start| {
            for ([_]f32{ 0.0, 2.0, 1000.0 }) |radius| {
                const result = try expectSameDistanceToWall(&fixture, &queries, start, radius);
                try testing.expect(result.status.succeeded);
                _ = try expectSoundWallDistance(start, radius, result);
            }
            const far: Start = .{ .ref = start.ref, .pos = far_center };
            const spread = try expectSameDistanceToWall(&fixture, &queries, far, reach_max);
            try testing.expect(spread.status.succeeded);
            if (spread.status.out_of_nodes) out_of_nodes_seen = true;
        }
    }
    try testing.expect(out_of_nodes_seen);

    // References no polygon answers to: refused, with nothing written.
    const no_ref: Start = .{ .ref = 0, .pos = starts[0].pos };
    const refused = try expectSameDistanceToWall(&fixture, &fixture.roomy, no_ref, 3.0);
    try testing.expectEqual(.refused, try expectSoundWallDistance(no_ref, 3.0, refused));
    const top_bit: PolyRef = @as(PolyRef, 1) << (@bitSizeOf(PolyRef) - 1);
    for (fixture.polygons.past_end.items) |ref| {
        for ([_]PolyRef{ ref, (ref - 1) ^ top_bit }) |bad_ref| {
            const bad: Start = .{ .ref = bad_ref, .pos = starts[0].pos };
            const result = try expectSameDistanceToWall(&fixture, &fixture.roomy, bad, 3.0);
            _ = try expectSoundWallDistance(bad, 3.0, result);
        }
    }

    // A centre far from its start polygon: a short reach finds no wall, and
    // one that overflows takes whichever wall the search meets last.
    for (starts, 0..) |start, start_index| {
        errdefer std.debug.print("  far from start {d}\n", .{start_index});
        const far: Start = .{ .ref = start.ref, .pos = far_center };
        for ([_]f32{ 0.0, 2.0 }) |radius| {
            const result = try expectSameDistanceToWall(&fixture, &fixture.roomy, far, radius);
            try testing.expectEqual(.clear, try expectSoundWallDistance(far, radius, result));
        }
        for ([_]f32{ 1e30, reach_max }) |radius| {
            for ([_]*QueryPair{ &fixture.roomy, &fixture.cramped }) |queries| {
                const result = try expectSameDistanceToWall(&fixture, queries, far, radius);
                try testing.expect(result.status.succeeded);
            }
        }
    }

    // A centre that is no point at all.
    if (!build_options.fast_math) {
        for (non_finite_centers, 0..) |center, center_index| {
            errdefer std.debug.print("  centre {d}\n", .{center_index});
            const start: Start = .{ .ref = starts[1].ref, .pos = center };
            const result = try expectSameDistanceToWall(&fixture, &fixture.roomy, start, 3.0);
            try testing.expectEqual(.refused, try expectSoundWallDistance(start, 3.0, result));
        }
    }

    // A centre on a corner of its start polygon. One of the two edges that
    // meet there starts at the corner, and the distance to that edge comes
    // out as exactly nothing. So with every edge a wall, the nearest wall
    // is no distance away, whatever the reach.
    for (starts, 0..) |start, start_index| {
        var corners: [6][3]f32 = undefined;
        const corner_count = polygonCorners(fixture.mesh.original, start.ref, &corners);
        for (corners[0..corner_count], 0..) |corner, corner_index| {
            errdefer std.debug.print("  start {d}, corner {d}\n", .{ start_index, corner_index });
            const on_corner: Start = .{ .ref = start.ref, .pos = corner };
            for ([_]f32{ 0.0, 0.5, 12.0 }) |radius| {
                fixture.filter.reset();
                for ([_]*QueryPair{ &fixture.roomy, &fixture.cramped }) |queries| {
                    const result = try expectSameDistanceToWall(
                        &fixture,
                        queries,
                        on_corner,
                        radius,
                    );
                    _ = try expectSoundWallDistance(on_corner, radius, result);
                }

                fixture.filter.passNothing();
                const walled = try expectSameDistanceToWall(
                    &fixture,
                    &fixture.roomy,
                    on_corner,
                    radius,
                );
                const outcome = try expectSoundWallDistance(on_corner, radius, walled);
                try testing.expectEqual(.hit, outcome);
                try testing.expect(walled.hit_dist == 0.0);
            }
        }
    }
}
