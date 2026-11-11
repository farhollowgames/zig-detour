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

// ---------------------------------------------------------------------------
// Filters with their own functions
// ---------------------------------------------------------------------------

/// One call a query made to a custom filter, with every argument it got.
const FilterCall = struct {
    cost: bool,
    ref: PolyRef,
    prev_ref: PolyRef = 0,
    next_ref: PolyRef = 0,
    pa: [3]u32 = @splat(0),
    pb: [3]u32 = @splat(0),
    flags: u16,
    area: u8,
};

/// The calls one side's custom filter received, in order.
const CallLog = struct {
    calls: [4096]FilterCall = undefined,
    len: usize = 0,
    overflowed: bool = false,

    fn add(self: *CallLog, call: FilterCall) void {
        if (self.len == self.calls.len) {
            self.overflowed = true;
            return;
        }
        self.calls[self.len] = call;
        self.len += 1;
    }

    fn written(self: *const CallLog) []const FilterCall {
        return self.calls[0..self.len];
    }
};

/// The custom rule both sides follow: polygons of area 2 never pass, and
/// walking area 3 costs three times its distance; everything else is the
/// default.
fn customPass(flags: u16, area: u8, include_flags: u16, exclude_flags: u16) bool {
    if (area == 2) return false;
    return (flags & include_flags) != 0 and (flags & exclude_flags) == 0;
}

fn customCost(pa: [3]f32, pb: [3]f32, area: u8, area_cost: f32) f32 {
    const distance = detour.common.vdist(pa, pb);
    return if (area == 3) distance * 3.0 else distance * area_cost;
}

fn floatBits(v: [3]f32) [3]u32 {
    return .{ @bitCast(v[0]), @bitCast(v[1]), @bitCast(v[2]) };
}

/// The port's custom filter: a `QueryFilter` with its own table, and the
/// log its calls go to.
const PortCustom = struct {
    filter: QueryFilter,
    log: *CallLog,

    const vtable: if (build_options.virtual_query_filter) QueryFilter.VTable else void =
        if (build_options.virtual_query_filter) .{ .passFilter = pass, .getCost = cost } else {};

    fn pass(
        filter: *const QueryFilter,
        ref: PolyRef,
        tile: *const MeshTile,
        poly: *const Poly,
    ) bool {
        _ = tile;
        const self: *const PortCustom = @fieldParentPtr("filter", filter);
        self.log.add(.{ .cost = false, .ref = ref, .flags = poly.flags, .area = poly.getArea() });
        return customPass(poly.flags, poly.getArea(), filter.include_flags, filter.exclude_flags);
    }

    fn cost(
        filter: *const QueryFilter,
        pa: [3]f32,
        pb: [3]f32,
        prev_ref: PolyRef,
        prev_tile: ?*const MeshTile,
        prev_poly: ?*const Poly,
        cur_ref: PolyRef,
        cur_tile: *const MeshTile,
        cur_poly: *const Poly,
        next_ref: PolyRef,
        next_tile: ?*const MeshTile,
        next_poly: ?*const Poly,
    ) f32 {
        _ = .{ prev_tile, prev_poly, cur_tile, next_tile, next_poly };
        const self: *const PortCustom = @fieldParentPtr("filter", filter);
        const area = cur_poly.getArea();
        self.log.add(.{
            .cost = true,
            .ref = cur_ref,
            .prev_ref = prev_ref,
            .next_ref = next_ref,
            .pa = floatBits(pa),
            .pb = floatBits(pb),
            .flags = cur_poly.flags,
            .area = area,
        });
        return customCost(pa, pb, area, filter.area_cost[area]);
    }
};

/// The original's custom filter calls back here, with the filter's own
/// settings handed over as the user pointer.
const OriginalCustom = struct {
    log: *CallLog,
    include_flags: u16 = 0xffff,
    exclude_flags: u16 = 0,
    area_cost: [detour.nav_mesh.area_count_max]f32 = @splat(1.0),

    fn pass(
        user: ?*anyopaque,
        ref: PolyRef,
        tile: *const reference.MeshTile,
        poly: *const reference.Poly,
    ) callconv(.c) bool {
        _ = tile;
        const self: *OriginalCustom = @ptrCast(@alignCast(user.?));
        const area: u8 = poly.area_and_type & 0x3f;
        self.log.add(.{ .cost = false, .ref = ref, .flags = poly.flags, .area = area });
        return customPass(poly.flags, area, self.include_flags, self.exclude_flags);
    }

    fn cost(
        user: ?*anyopaque,
        pa: *const [3]f32,
        pb: *const [3]f32,
        prev_ref: PolyRef,
        prev_tile: ?*const reference.MeshTile,
        prev_poly: ?*const reference.Poly,
        cur_ref: PolyRef,
        cur_tile: *const reference.MeshTile,
        cur_poly: *const reference.Poly,
        next_ref: PolyRef,
        next_tile: ?*const reference.MeshTile,
        next_poly: ?*const reference.Poly,
    ) callconv(.c) f32 {
        _ = .{ prev_tile, prev_poly, cur_tile, next_tile, next_poly };
        const self: *OriginalCustom = @ptrCast(@alignCast(user.?));
        const area: u8 = cur_poly.area_and_type & 0x3f;
        self.log.add(.{
            .cost = true,
            .ref = cur_ref,
            .prev_ref = prev_ref,
            .next_ref = next_ref,
            .pa = floatBits(pa.*),
            .pb = floatBits(pb.*),
            .flags = cur_poly.flags,
            .area = area,
        });
        return customCost(pa.*, pb.*, area, self.area_cost[area]);
    }
};

/// Fails unless the two sides' filters were asked the same questions, in
/// the same order, then empties both logs.
fn expectSameCalls(expected: *CallLog, actual: *CallLog) !void {
    try testing.expect(!expected.overflowed and !actual.overflowed);
    try testing.expectEqual(expected.len, actual.len);
    for (expected.written(), actual.written(), 0..) |e, a, i| {
        errdefer std.debug.print("  filter call {d} of {d}\n", .{ i, expected.len });
        try testing.expectEqual(e, a);
    }
    expected.len = 0;
    actual.len = 0;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

test "QueryFilter: a new filter's flags and area costs" {
    var filter: FilterPair = try .create();
    defer filter.deinit();
    try filter.expectSameFields();
    try testing.expectEqual(@as(usize, 64), filter.port.area_cost.len);
}

test "QueryFilter: every setter, read back through every getter" {
    const seed: u64 = 0x66696c746572_01;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();
    var filter: FilterPair = try .create();
    defer filter.deinit();
    for (0..1200) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        filter.setFlags(randomFlagMask(random), randomFlagMask(random));
        for (0..random.intRangeAtMost(usize, 1, 8)) |_| {
            filter.setAreaCost(random.uintLessThan(usize, 64), randomAreaCost(random));
        }
        try filter.expectSameFields();
    }
}

test "passFilter: every polygon under random flags, and polygons whose flags changed" {
    const gpa = testing.allocator;
    const seed: u64 = 0x66696c746572_02;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for ([_]WorldCase{ nav_test_tiled, dungeon_one_tile, dungeon_tiled_no_tree }) |world_case| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{world_case.name});
        var world: support.World = undefined;
        try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        defer world.deinit();
        const polygons = try collectPolygons(gpa, &world);
        defer gpa.free(polygons);
        var filter: FilterPair = try .create();
        defer filter.deinit();

        var tally: PassTally = .{};
        for (0..400) |case| {
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
            filter.setFlags(randomFlagMask(random), randomFlagMask(random));

            // Some polygons get new flags on both meshes first.
            for (0..random.uintLessThan(usize, 4)) |_| {
                const polygon = randomPolygon(random, polygons);
                const flags = randomFlagMask(random);
                try support.expectStatus(
                    reference.ref_nav_mesh_set_poly_flags(world.reference_nav, polygon.ref, flags),
                    world.nav.setPolyFlags(polygon.ref, flags),
                );
            }
            try expectSamePolygons(polygons);
            for (polygons) |polygon| tally.add(try expectSamePass(filter.sides(), polygon));

            // A polygon of any flags and area, as a caller may hand one.
            const loose: LoosePolygon = .init(random.int(u16), random.int(u8));
            const home = randomPolygon(random, polygons);
            tally.add(try expectSamePass(filter.sides(), loose.pair(home.ref, home)));
        }
        try tally.expectBothAnswers();
    }
}

test "getCost: random walks across polygons of every area, under random area costs" {
    const gpa = testing.allocator;
    const seed: u64 = 0x66696c746572_03;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var world: support.World = undefined;
    try world.init(gpa, nav_test_tiled.obj_text, nav_test_tiled.config, nav_test_tiled.options);
    defer world.deinit();
    const polygons = try collectPolygons(gpa, &world);
    defer gpa.free(polygons);
    var filter: FilterPair = try .create();
    defer filter.deinit();

    var finite_count: usize = 0;
    for (0..3000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        if (case % 50 == 0) {
            for (0..64) |area| filter.setAreaCost(area, randomAreaCost(random));
        }
        var question = randomCostQuestion(random, &world.level, polygons);
        // Every third walk crosses a polygon of any of the 64 areas.
        const loose: LoosePolygon = .init(random.int(u16), random.int(u8));
        if (case % 3 == 0) question.cur = loose.pair(question.cur.ref, question.cur);
        const cost = try expectSameCost(filter.sides(), question);
        if (std.math.isFinite(cost)) finite_count += 1;
    }
    try testing.expect(finite_count > 1000);
}

test "a filter with its own functions: queries ask it the same questions in the same order" {
    if (!build_options.virtual_query_filter) return error.SkipZigTest;
    const gpa = testing.allocator;
    const seed: u64 = 0x66696c746572_04;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var original_log: CallLog = .{};
    var port_log: CallLog = .{};
    var original_custom: OriginalCustom = .{ .log = &original_log };
    const original_filter = reference.ref_filter_create_custom(
        OriginalCustom.pass,
        OriginalCustom.cost,
        &original_custom,
    ) orelse return error.OutOfMemory;
    defer reference.ref_filter_destroy(original_filter);
    var port_custom: PortCustom = .{ .filter = .{}, .log = &port_log };
    if (build_options.virtual_query_filter) port_custom.filter.vtable = &PortCustom.vtable;
    const port_filter = &port_custom.filter;

    var refused_count: usize = 0;
    var path_count_total: usize = 0;
    for ([_]WorldCase{ nav_test_tiled, dungeon_one_tile }) |world_case| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{world_case.name});
        var world: support.World = undefined;
        try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        defer world.deinit();
        var queries: QueryPair = try .create(gpa, &world);
        defer queries.deinit(gpa);

        for (0..600) |case| {
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
            if (case % 40 == 0) {
                const include = randomFlagMask(random) | 1;
                const exclude = randomFlagMask(random) & 0xfff0;
                port_filter.include_flags = include;
                port_filter.exclude_flags = exclude;
                original_custom.include_flags = include;
                original_custom.exclude_flags = exclude;
                reference.ref_filter_set_include_flags(original_filter, include);
                reference.ref_filter_set_exclude_flags(original_filter, exclude);
                const area = random.uintLessThan(usize, 6);
                const area_cost = random.float(f32) * 4.0;
                port_filter.area_cost[area] = area_cost;
                original_custom.area_cost[area] = area_cost;
                reference.ref_filter_set_area_cost(original_filter, @intCast(area), area_cost);
            }
            const start = world.level.groundPoint(random, 1.0);
            const end = world.level.groundPoint(random, 1.0);
            const half_extents: [3]f32 = .{ 2, 4, 2 };

            // The nearest polygons the filter lets through.
            var start_ref: PolyRef = untouched_ref;
            var start_pos: [3]f32 = untouched_point;
            var end_ref: PolyRef = untouched_ref;
            var end_pos: [3]f32 = untouched_point;
            const near_statuses = [2]u32{
                reference.ref_query_find_nearest_poly(
                    queries.original,
                    &start,
                    &half_extents,
                    original_filter,
                    &start_ref,
                    &start_pos,
                ),
                reference.ref_query_find_nearest_poly(
                    queries.original,
                    &end,
                    &half_extents,
                    original_filter,
                    &end_ref,
                    &end_pos,
                ),
            };
            var port_start_ref: PolyRef = untouched_ref;
            var port_start_pos: [3]f32 = untouched_point;
            var port_end_ref: PolyRef = untouched_ref;
            var port_end_pos: [3]f32 = untouched_point;
            try support.expectStatus(near_statuses[0], queries.port.findNearestPoly(
                start,
                half_extents,
                port_filter,
                &port_start_ref,
                &port_start_pos,
                null,
            ));
            try support.expectStatus(near_statuses[1], queries.port.findNearestPoly(
                end,
                half_extents,
                port_filter,
                &port_end_ref,
                &port_end_pos,
                null,
            ));
            try expectSameCalls(&original_log, &port_log);
            try testing.expectEqual(start_ref, port_start_ref);
            try testing.expectEqual(end_ref, port_end_ref);
            try support.expectPoint(start_pos, port_start_pos);
            try support.expectPoint(end_pos, port_end_pos);
            if (start_ref == 0 or end_ref == 0) {
                refused_count += 1;
                continue;
            }

            // A corridor, which asks for passes and costs.
            var expected_path: [64]PolyRef = @splat(untouched_ref);
            var actual_path: [64]PolyRef = @splat(untouched_ref);
            var expected_count: i32 = untouched_count;
            var actual_count: usize = untouched_port_count;
            try support.expectStatus(
                reference.ref_query_find_path(
                    queries.original,
                    start_ref,
                    end_ref,
                    &start_pos,
                    &end_pos,
                    original_filter,
                    &expected_path,
                    &expected_count,
                    expected_path.len,
                ),
                queries.port.findPath(
                    start_ref,
                    end_ref,
                    start_pos,
                    end_pos,
                    port_filter,
                    &actual_path,
                    &actual_count,
                ),
            );
            try expectSameCalls(&original_log, &port_log);
            try expectCount(expected_count, actual_count);
            try testing.expectEqualSlices(PolyRef, &expected_path, &actual_path);
            path_count_total += actual_count;

            // A ray with costs.
            var expected_t: f32 = untouched_scalar;
            var expected_normal: [3]f32 = untouched_point;
            var expected_edge: i32 = untouched_count;
            var expected_hit_count: i32 = untouched_count;
            var expected_cost: f32 = untouched_scalar;
            var expected_hit_path: [32]PolyRef = @splat(untouched_ref);
            const expected_ray = reference.ref_query_raycast_hit(
                queries.original,
                start_ref,
                &start_pos,
                &end_pos,
                original_filter,
                @bitCast(RaycastOptions{ .use_costs = true }),
                0,
                &expected_t,
                &expected_normal,
                &expected_edge,
                &expected_hit_path,
                &expected_hit_count,
                expected_hit_path.len,
                &expected_cost,
            );
            var actual_hit_path: [32]PolyRef = @splat(untouched_ref);
            // The shim starts the original's hit at zero, which is what the
            // fields a ray that hits nothing leaves alone read as.
            var hit: RaycastHit = .{
                .t = 0,
                .hit_normal = @splat(0),
                .hit_edge_index = 0,
                .path = &actual_hit_path,
                .path_count = 0,
                .path_cost = 0,
            };
            try support.expectStatus(expected_ray, queries.port.raycast(
                start_ref,
                start_pos,
                end_pos,
                port_filter,
                .{ .use_costs = true },
                &hit,
                0,
            ));
            try expectSameCalls(&original_log, &port_log);
            try support.expectFloat(expected_t, hit.t);
            try support.expectPoint(expected_normal, hit.hit_normal);
            try testing.expectEqual(expected_edge, hit.hit_edge_index);
            try expectCount(expected_hit_count, hit.path_count);
            try support.expectFloat(expected_cost, hit.path_cost);
            try testing.expectEqualSlices(PolyRef, &expected_hit_path, &actual_hit_path);

            // A Dijkstra search and a walk, which ask for both too.
            var expected_refs: [48]PolyRef = @splat(untouched_ref);
            var expected_costs: [48]f32 = @splat(untouched_scalar);
            var actual_refs: [48]PolyRef = @splat(untouched_ref);
            var actual_costs: [48]f32 = @splat(untouched_scalar);
            const radius = random.float(f32) * 12.0;
            try support.expectStatus(
                reference.ref_query_find_polys_around_circle(
                    queries.original,
                    start_ref,
                    &start_pos,
                    radius,
                    original_filter,
                    &expected_refs,
                    null,
                    &expected_costs,
                    &expected_count,
                    expected_refs.len,
                ),
                queries.port.findPolysAroundCircle(
                    start_ref,
                    start_pos,
                    radius,
                    port_filter,
                    &actual_refs,
                    null,
                    &actual_costs,
                    &actual_count,
                    actual_refs.len,
                ),
            );
            try expectSameCalls(&original_log, &port_log);
            try expectCount(expected_count, actual_count);
            try testing.expectEqualSlices(PolyRef, &expected_refs, &actual_refs);
            try support.expectFloats(&expected_costs, &actual_costs);

            var expected_walk: [3]f32 = untouched_point;
            var actual_walk: [3]f32 = untouched_point;
            try support.expectStatus(
                reference.ref_query_move_along_surface(
                    queries.original,
                    start_ref,
                    &start_pos,
                    &end_pos,
                    original_filter,
                    &expected_walk,
                    &expected_refs,
                    &expected_count,
                    16,
                ),
                queries.port.moveAlongSurface(
                    start_ref,
                    start_pos,
                    end_pos,
                    port_filter,
                    &actual_walk,
                    actual_refs[0..16],
                    &actual_count,
                ),
            );
            try expectSameCalls(&original_log, &port_log);
            try expectCount(expected_count, actual_count);
            try support.expectPoint(expected_walk, actual_walk);
            try testing.expectEqualSlices(PolyRef, &expected_refs, &actual_refs);
        }
    }
    try testing.expect(refused_count > 0);
    try testing.expect(path_count_total > 1000);
}
