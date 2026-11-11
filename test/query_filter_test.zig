//! Differential tests of the query filter: the flags and area costs a new
//! filter has, which polygons a filter passes and what it charges for a walk
//! across one, and filters that put their own functions in place of those
//! two. The port and the original must answer with the same bits, and a
//! query must ask a custom filter the same questions in the same order.

const std = @import("std");
const build_options = @import("build_options");
const detour = @import("detour");
const support = @import("support.zig");
const reference = support.reference;

const testing = std.testing;

const NavMesh = detour.NavMesh;
const NavMeshQuery = detour.NavMeshQuery;
const QueryFilter = detour.QueryFilter;
const RaycastHit = detour.RaycastHit;
const RaycastOptions = detour.RaycastOptions;
const MeshTile = detour.MeshTile;
const Poly = detour.Poly;
const PolyRef = detour.PolyRef;

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

// Every world has varied polygons: flag 0x1 with one of 0x1, 0x2, 0x4 and
// 0x8, and an area from 0 to 5, so that filters have polygons to tell apart.

const nav_test_tiled: WorldCase = .{
    .name = "nav_test, tiled",
    .obj_text = support.nav_test_obj,
    .config = support.tiled_config,
    .options = .{ .off_mesh_connection_count = 24, .vary_polygons = true, .seed = 51 },
};

const dungeon_one_tile: WorldCase = .{
    .name = "dungeon, one tile",
    .obj_text = support.dungeon_obj,
    .config = support.solo_config,
    .options = .{ .off_mesh_connection_count = 12, .vary_polygons = true, .seed = 52 },
};

const dungeon_tiled_no_tree: WorldCase = .{
    .name = "dungeon, tiled, no tree",
    .obj_text = support.dungeon_obj,
    .config = support.tiled_config,
    .options = .{
        .off_mesh_connection_count = 16,
        .vary_polygons = true,
        .build_bv_tree = false,
        .seed = 53,
    },
};

// ---------------------------------------------------------------------------
// The two sides of a query and a filter
// ---------------------------------------------------------------------------

/// A query of the port and one of the original, attached to the two sides
/// of one world.
const QueryPair = struct {
    port: NavMeshQuery,
    original: *reference.NavMeshQuery,

    /// Enough nodes for most searches to end, few enough that some run out.
    const node_count_max = 2048;

    /// Two queries attached to `world`, which must outlive them. Free with
    /// `deinit`.
    fn create(gpa: std.mem.Allocator, world: *const support.World) !QueryPair {
        const original = reference.ref_query_alloc() orelse return error.OutOfMemory;
        var pair: QueryPair = .{ .port = .empty, .original = original };
        errdefer pair.deinit(gpa);

        const expected = reference.ref_query_init(original, world.reference_nav, node_count_max);
        const actual = pair.port.init(gpa, &world.nav, node_count_max);
        try support.expectStatus(expected, actual);
        if (actual.failed) return error.InitFailed;
        return pair;
    }

    fn deinit(self: *QueryPair, gpa: std.mem.Allocator) void {
        self.port.deinit(gpa);
        reference.ref_query_free(self.original);
        self.* = undefined;
    }
};

/// One filter as the queries of each side take it.
const FilterSides = struct {
    port: *const QueryFilter,
    original: *const reference.QueryFilter,
};

/// A filter of the port and one of the original with the default functions,
/// kept alike: every change goes to both.
const FilterPair = struct {
    port: QueryFilter,
    original: *reference.QueryFilter,

    /// A filter as each side makes one. Free with `deinit`.
    fn create() !FilterPair {
        const original = reference.ref_filter_create() orelse return error.OutOfMemory;
        return .{ .port = .{}, .original = original };
    }

    fn deinit(self: *FilterPair) void {
        reference.ref_filter_destroy(self.original);
        self.* = undefined;
    }

    /// The pointers stay good while the pair stays where it is.
    fn sides(self: *const FilterPair) FilterSides {
        return .{ .port = &self.port, .original = self.original };
    }

    fn setFlags(self: *FilterPair, include_flags: u16, exclude_flags: u16) void {
        self.port.include_flags = include_flags;
        self.port.exclude_flags = exclude_flags;
        reference.ref_filter_set_include_flags(self.original, include_flags);
        reference.ref_filter_set_exclude_flags(self.original, exclude_flags);
    }

    fn setAreaCost(self: *FilterPair, area: usize, cost: f32) void {
        self.port.area_cost[area] = cost;
        reference.ref_filter_set_area_cost(self.original, @intCast(area), cost);
    }

    fn expectSameFields(self: *const FilterPair) !void {
        return expectFilterFields(self.original, &self.port);
    }
};

/// Fails unless the port's filter holds the flags and the area costs the
/// original's getters return, bit for bit.
fn expectFilterFields(original: *const reference.QueryFilter, port: *const QueryFilter) !void {
    try testing.expectEqual(reference.ref_filter_get_include_flags(original), port.include_flags);
    try testing.expectEqual(reference.ref_filter_get_exclude_flags(original), port.exclude_flags);
    for (port.area_cost, 0..) |cost, area| {
        errdefer std.debug.print("  area {d}\n", .{area});
        try support.expectFloat(reference.ref_filter_get_area_cost(original, @intCast(area)), cost);
    }
}

// ---------------------------------------------------------------------------
// Outputs a call must leave alone
// ---------------------------------------------------------------------------

// What every output holds before a call, on both sides, so that an output one
// side writes and the other leaves alone shows as a difference.
const untouched_point: [3]f32 = .{ -1234.5, 6789.25, -42.125 };
const untouched_scalar: f32 = -9876.5;
const untouched_ref: PolyRef = std.math.maxInt(PolyRef) - 0x1234;
const untouched_flag: u8 = 0xaa;
const untouched_count: i32 = -77;
const untouched_port_count: usize = std.math.maxInt(usize) - 77;

/// The original counts polygons in an `i32` and the port in a `usize`. Fails
/// unless both wrote the same count, or neither wrote one.
fn expectCount(expected: i32, actual: usize) !void {
    const expected_untouched = expected == untouched_count;
    const actual_untouched = actual == untouched_port_count;
    try testing.expectEqual(expected_untouched, actual_untouched);
    if (expected_untouched) return;
    try testing.expectEqual(@as(i64, expected), @as(i64, @intCast(actual)));
}

/// A `bool` output as the byte it lives in, so that a call that leaves it
/// alone can be told from one that writes false.
const FlagOutput = struct {
    byte: u8 = untouched_flag,

    /// The output to hand to a call: the flag when `wanted`, else none.
    fn pointer(self: *FlagOutput, wanted: bool) ?*bool {
        if (!wanted) return null;
        const flag: *bool = @ptrCast(&self.byte);
        return flag;
    }
};

// ---------------------------------------------------------------------------
// Polygons
// ---------------------------------------------------------------------------

/// One polygon as each side holds it.
const PolygonPair = struct {
    ref: PolyRef,
    tile: *const MeshTile,
    poly: *const Poly,
    original_tile: *const reference.MeshTile,
    original_poly: *const reference.Poly,
};

/// Every polygon of every tile of `world`, in tile and polygon order. The
/// pointers go stale when a tile is added or removed. The caller frees the
/// slice.
fn collectPolygons(gpa: std.mem.Allocator, world: *const support.World) ![]PolygonPair {
    var polygons: std.ArrayList(PolygonPair) = .empty;
    errdefer polygons.deinit(gpa);

    const nav = &world.nav;
    for (nav.tiles) |*tile| {
        if (tile.header == null) continue;
        const base = nav.getPolyRefBase(tile);
        for (tile.polys, 0..) |*poly, i| {
            const index: PolyRef = @intCast(i);
            const ref = base | index;

            var original_tile: ?*const reference.MeshTile = null;
            var original_poly: ?*const reference.Poly = null;
            const status = reference.ref_nav_mesh_get_tile_and_poly_by_ref(
                world.reference_nav,
                ref,
                &original_tile,
                &original_poly,
            );
            if (reference.ref_status_failed(status)) return error.PolygonMissing;

            try polygons.append(gpa, .{
                .ref = ref,
                .tile = tile,
                .poly = poly,
                .original_tile = original_tile orelse return error.PolygonMissing,
                .original_poly = original_poly orelse return error.PolygonMissing,
            });
        }
    }
    return polygons.toOwnedSlice(gpa);
}

/// Fails unless the two sides of each polygon hold the same flags, area and
/// type: what a filter reads of a polygon.
fn expectSamePolygons(polygons: []const PolygonPair) !void {
    for (polygons) |polygon| {
        errdefer std.debug.print("  polygon {x}\n", .{polygon.ref});
        try testing.expectEqual(polygon.original_poly.flags, polygon.poly.flags);
        try testing.expectEqual(
            polygon.original_poly.area_and_type,
            polygon.poly.area_and_type,
        );
    }
}

/// A polygon no mesh holds, the same on both sides. A filter's own functions
/// read only the flags, the area and the type of a polygon, so any value of
/// those can be put before them this way.
const LoosePolygon = struct {
    port: Poly,
    original: reference.Poly,

    fn init(flags: u16, area_and_type: u8) LoosePolygon {
        return .{
            .port = .{
                .first_link = 0,
                .verts = @splat(0),
                .neighbours = @splat(0),
                .flags = flags,
                .vert_count = 3,
                .area_and_type = area_and_type,
            },
            .original = .{
                .first_link = 0,
                .verts = @splat(0),
                .neis = @splat(0),
                .flags = flags,
                .vert_count = 3,
                .area_and_type = area_and_type,
            },
        };
    }

    /// The polygon as a filter is asked about it: under `ref`, in the tile
    /// of `home`. The pointers stay good while `self` stays where it is.
    fn pair(self: *const LoosePolygon, ref: PolyRef, home: PolygonPair) PolygonPair {
        return .{
            .ref = ref,
            .tile = home.tile,
            .poly = &self.port,
            .original_tile = home.original_tile,
            .original_poly = &self.original,
        };
    }
};

fn randomPolygon(random: std.Random, polygons: []const PolygonPair) PolygonPair {
    return polygons[random.uintLessThan(usize, polygons.len)];
}

// ---------------------------------------------------------------------------
// Questions to a filter
// ---------------------------------------------------------------------------

/// A flag mask with no bits, one bit, a few bits or most bits, so that a
/// random pair of masks passes about as many polygons as it refuses.
fn randomFlagMask(random: std.Random) u16 {
    return switch (random.uintLessThan(u8, 8)) {
        0 => 0,
        1 => 0xffff,
        2 => @as(u16, 1) << random.int(u4),
        3 => random.int(u16) & 0x000f,
        4 => random.int(u16) & random.int(u16),
        5 => ~(@as(u16, 1) << random.int(u4)),
        6 => random.int(u16) | random.int(u16),
        else => random.int(u16),
    };
}

/// An area cost: mostly a plain rate, now and then zero of either sign, a
/// negative, a huge or a tiny rate, an infinity, or any bits at all, which
/// are often not a number.
fn randomAreaCost(random: std.Random) f32 {
    return switch (random.uintLessThan(u8, 12)) {
        0 => 0.0,
        1 => -@as(f32, 0.0),
        2 => 1.0,
        3 => -random.float(f32) * 8.0,
        4 => std.math.inf(f32),
        5 => -std.math.inf(f32),
        6 => std.math.floatMax(f32) * random.float(f32),
        7 => std.math.floatTrueMin(f32) * @as(f32, @floatFromInt(random.uintLessThan(u32, 4096))),
        8, 9 => @bitCast(random.int(u32)),
        else => random.float(f32) * 8.0,
    };
}

/// A float from -1 up to 1.
fn signedUnit(random: std.Random) f32 {
    return random.float(f32) * 2.0 - 1.0;
}

/// A point with finite coordinates: on the level, at the origin, tiny, or so
/// far out that the square of a distance is past the largest float.
///
/// Never a coordinate that is not a number: the product of two of those
/// takes its bits from either operand, by an order the two sides are free to
/// differ in.
fn randomWalkPoint(random: std.Random, level: *const support.Level) [3]f32 {
    const scale: f32 = switch (random.uintLessThan(u8, 8)) {
        0 => std.math.floatMax(f32),
        1 => 1.0e19,
        2 => 1.0e-30,
        3 => 0.0,
        else => return level.groundPoint(random, 3.0),
    };
    return .{
        signedUnit(random) * scale,
        signedUnit(random) * scale,
        signedUnit(random) * scale,
    };
}

/// Runs `passFilter` on both sides and fails unless the answers are the
/// same. Returns the answer.
fn expectSamePass(filter: FilterSides, polygon: PolygonPair) !bool {
    const expected = reference.ref_filter_pass_filter(
        filter.original,
        polygon.ref,
        polygon.original_tile,
        polygon.original_poly,
    );
    const actual = filter.port.passFilter(polygon.ref, polygon.tile, polygon.poly);
    try testing.expectEqual(expected, actual);
    return actual;
}

/// How many polygons a test's filters passed and refused. A test whose
/// filters only ever give one answer says little about the other.
const PassTally = struct {
    passed: usize = 0,
    refused: usize = 0,

    fn add(self: *PassTally, passed: bool) void {
        if (passed) self.passed += 1 else self.refused += 1;
    }

    fn expectBothAnswers(self: PassTally) !void {
        try testing.expect(self.passed > 0);
        try testing.expect(self.refused > 0);
    }
};

/// One question to a filter's cost function: the walk from `pa` to `pb`
/// across `cur`, with the polygon before it and the one after it, if any.
const CostQuestion = struct {
    pa: [3]f32,
    pb: [3]f32,
    prev: ?PolygonPair,
    cur: PolygonPair,
    next: ?PolygonPair,
};

/// Runs `getCost` on both sides and fails unless the costs have the same
/// bits. Returns the cost.
fn expectSameCost(filter: FilterSides, question: CostQuestion) !f32 {
    const prev = question.prev;
    const cur = question.cur;
    const next = question.next;
    const expected = reference.ref_filter_get_cost(
        filter.original,
        &question.pa,
        &question.pb,
        if (prev) |polygon| polygon.ref else 0,
        if (prev) |polygon| polygon.original_tile else null,
        if (prev) |polygon| polygon.original_poly else null,
        cur.ref,
        cur.original_tile,
        cur.original_poly,
        if (next) |polygon| polygon.ref else 0,
        if (next) |polygon| polygon.original_tile else null,
        if (next) |polygon| polygon.original_poly else null,
    );
    const actual = filter.port.getCost(
        question.pa,
        question.pb,
        if (prev) |polygon| polygon.ref else 0,
        if (prev) |polygon| polygon.tile else null,
        if (prev) |polygon| polygon.poly else null,
        cur.ref,
        cur.tile,
        cur.poly,
        if (next) |polygon| polygon.ref else 0,
        if (next) |polygon| polygon.tile else null,
        if (next) |polygon| polygon.poly else null,
    );
    try support.expectFloat(expected, actual);
    return actual;
}

/// A walk between two finite points across one of `polygons`, with or
/// without a polygon before it and after it.
fn randomCostQuestion(
    random: std.Random,
    level: *const support.Level,
    polygons: []const PolygonPair,
) CostQuestion {
    const pa = randomWalkPoint(random, level);
    return .{
        .pa = pa,
        // A walk of no length now and then.
        .pb = if (random.uintLessThan(u8, 8) == 0) pa else randomWalkPoint(random, level),
        .prev = if (random.boolean()) randomPolygon(random, polygons) else null,
        .cur = randomPolygon(random, polygons),
        .next = if (random.boolean()) randomPolygon(random, polygons) else null,
    };
}
