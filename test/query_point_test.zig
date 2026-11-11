//! Differential tests of the point queries of a navigation mesh query:
//! attaching a query to a mesh, the polygon nearest a point, the polygons a
//! box touches, the closest point and the height on one polygon, and whether
//! a reference passes a filter, also for references whose tile was removed
//! and added again. The port and the original must return the same statuses
//! and the same bits, and leave the same outputs alone.

const std = @import("std");
const detour = @import("detour");
const support = @import("support.zig");
const reference = support.reference;

const testing = std.testing;

const NavMesh = detour.NavMesh;
const NavMeshQuery = detour.NavMeshQuery;
const QueryFilter = detour.QueryFilter;
const PolyQuery = detour.PolyQuery;
const MeshTile = detour.MeshTile;
const Poly = detour.Poly;
const PolyRef = detour.PolyRef;
const Status = detour.Status;

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

const nav_test_tiled: WorldCase = .{
    .name = "nav_test, tiled",
    .obj_text = support.nav_test_obj,
    .config = support.tiled_config,
    .options = .{ .off_mesh_connection_count = 24, .vary_polygons = true, .seed = 41 },
};

const dungeon_one_tile: WorldCase = .{
    .name = "dungeon, one tile",
    .obj_text = support.dungeon_obj,
    .config = support.solo_config,
    .options = .{ .off_mesh_connection_count = 12, .vary_polygons = true, .seed = 42 },
};

const dungeon_tiled_no_tree: WorldCase = .{
    .name = "dungeon, tiled, no tree",
    .obj_text = support.dungeon_obj,
    .config = support.tiled_config,
    .options = .{
        .off_mesh_connection_count = 16,
        .vary_polygons = true,
        .build_bv_tree = false,
        .seed = 43,
    },
};

const nav_test_one_tile_no_tree: WorldCase = .{
    .name = "nav_test, one tile, no tree",
    .obj_text = support.nav_test_obj,
    .config = support.solo_config,
    .options = .{ .off_mesh_connection_count = 8, .build_bv_tree = false, .seed = 44 },
};

const undulating_one_tile: WorldCase = .{
    .name = "undulating, one tile",
    .obj_text = support.undulating_obj,
    .config = support.solo_config,
    .options = .{ .seed = 45 },
};

// ---------------------------------------------------------------------------
// The two sides of a mesh, a query and a filter
// ---------------------------------------------------------------------------

/// One navigation mesh on both sides.
const MeshPair = struct {
    port: *const NavMesh,
    original: *const reference.NavMesh,

    fn of(world: *const support.World) MeshPair {
        return .{ .port = &world.nav, .original = world.reference_nav };
    }
};

/// A query of the port and one of the original, to attach to the two sides
/// of one mesh.
const QueryPair = struct {
    port: NavMeshQuery,
    original: *reference.NavMeshQuery,

    /// The node count the tests that only ask point queries attach with.
    /// Those queries search nothing, so the count does not matter to them.
    const point_query_node_count = 64;

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

    /// Frees both queries and leaves two new ones, attached to nothing.
    fn recreate(self: *QueryPair, gpa: std.mem.Allocator) !void {
        const original = reference.ref_query_alloc() orelse return error.OutOfMemory;
        reference.ref_query_free(self.original);
        self.original = original;
        self.port.deinit(gpa);
        self.port = .empty;
    }

    /// Runs `init` on both sides. Fails unless they give the same status,
    /// which is returned.
    fn attach(
        self: *QueryPair,
        gpa: std.mem.Allocator,
        mesh: MeshPair,
        node_count_max: i32,
    ) !Status {
        const expected = reference.ref_query_init(self.original, mesh.original, node_count_max);
        const actual = self.port.init(gpa, mesh.port, node_count_max);
        try support.expectStatus(expected, actual);
        return actual;
    }

    /// Attaches both queries to `world`, ready for point queries.
    fn ready(self: *QueryPair, gpa: std.mem.Allocator, world: *const support.World) !void {
        const status = try self.attach(gpa, MeshPair.of(world), point_query_node_count);
        if (status.failed) return error.InitFailed;
    }

    /// The mesh of `meshes` both queries are attached to, as its index; null
    /// when they are attached to none of them. Fails unless the two sides
    /// agree, also on being attached at all.
    fn attachedMesh(self: *const QueryPair, meshes: []const MeshPair) !?usize {
        const original_nav = reference.ref_query_get_attached_nav_mesh(self.original);
        try testing.expectEqual(original_nav == null, self.port.nav == null);

        var expected: ?usize = null;
        var actual: ?usize = null;
        for (meshes, 0..) |mesh, i| {
            if (@intFromPtr(original_nav) == @intFromPtr(mesh.original)) expected = i;
            // The port's getter is only for a query that is attached.
            if (self.port.nav != null and self.port.getAttachedNavMesh() == mesh.port) actual = i;
        }
        try testing.expectEqual(expected, actual);
        return actual;
    }
};

/// The same filter on both sides.
const FilterPair = struct {
    port: QueryFilter,
    original: *reference.QueryFilter,

    /// A filter as both sides make one by default: every polygon passes.
    /// Free with `deinit`.
    fn create() !FilterPair {
        const original = reference.ref_filter_create() orelse return error.OutOfMemory;
        return .{ .port = .{}, .original = original };
    }

    fn deinit(self: *FilterPair) void {
        reference.ref_filter_destroy(self.original);
        self.* = undefined;
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
};

/// The flags of one filter: a polygon passes with one of `include` and none
/// of `exclude`.
const FilterFlags = struct {
    include: u16,
    exclude: u16,
};

/// Filters that pass every polygon, some polygons by a flag, and none. A
/// world with varied polygons gives each polygon flag 0x1 and one of 0x1,
/// 0x2, 0x4 and 0x8.
const filter_flags = [_]FilterFlags{
    .{ .include = 0xffff, .exclude = 0 },
    .{ .include = 0x0001, .exclude = 0 },
    .{ .include = 0x0002, .exclude = 0 },
    .{ .include = 0x0004, .exclude = 0 },
    .{ .include = 0x0008, .exclude = 0 },
    .{ .include = 0x000c, .exclude = 0 },
    .{ .include = 0xffff, .exclude = 0x0002 },
    .{ .include = 0xffff, .exclude = 0x000c },
    .{ .include = 0x0006, .exclude = 0x0004 },
    .{ .include = 0xfff0, .exclude = 0 },
    .{ .include = 0, .exclude = 0 },
    .{ .include = 0xffff, .exclude = 0x0001 },
};

/// A filter pair for each entry of `filter_flags`. Every second one has its
/// area costs changed: costs steer path searches, so the answers of the
/// point queries must not move with them.
const FilterSet = struct {
    pairs: [filter_flags.len]FilterPair,

    /// Free with `deinit`.
    fn create() !FilterSet {
        var set: FilterSet = undefined;
        var made: usize = 0;
        errdefer {
            for (set.pairs[0..made]) |*pair| pair.deinit();
        }
        for (filter_flags, 0..) |flags, i| {
            set.pairs[i] = try FilterPair.create();
            made += 1;
            set.pairs[i].setFlags(flags.include, flags.exclude);
            if (i % 2 == 1) {
                for (0..8) |area| {
                    const steps: f32 = @floatFromInt(area + i);
                    set.pairs[i].setAreaCost(area, 0.25 + steps * 0.75);
                }
            }
        }
        return set;
    }

    fn deinit(self: *FilterSet) void {
        for (&self.pairs) |*pair| pair.deinit();
        self.* = undefined;
    }

    /// The filter that passes every polygon.
    fn passAll(self: *const FilterSet) *const FilterPair {
        return &self.pairs[0];
    }

    /// Any of the filters; the one that passes every polygon four times in
    /// ten, so that most queries have something to find.
    fn pick(self: *const FilterSet, random: std.Random) *const FilterPair {
        if (random.uintLessThan(u32, 10) < 4) return &self.pairs[0];
        return &self.pairs[random.uintLessThan(usize, self.pairs.len)];
    }
};

// ---------------------------------------------------------------------------
// Outputs a call must leave alone
// ---------------------------------------------------------------------------

// What every output holds before a call, on both sides, so that an output one
// side writes and the other leaves alone shows as a difference.
const untouched_point: [3]f32 = .{ -1234.5, 6789.25, -42.125 };
const untouched_height: f32 = -9876.5;
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

/// What no coordinate of a position should be.
const not_finite = [_]f32{
    std.math.nan(f32),
    std.math.inf(f32),
    -std.math.inf(f32),
};

/// Whether both sides refuse a position that is not finite. A build with
/// fast math takes every float for finite on both sides, and the original
/// then indexes its tiles with what it makes of such a float, so these
/// inputs are only for the other builds.
fn rejectsNonFinite() bool {
    const nan = std.math.nan(f32);
    return !reference.ref_math_isfinite(nan) and !detour.common.isFinite(nan);
}

// ---------------------------------------------------------------------------
// Boxes
// ---------------------------------------------------------------------------

/// The box of a query: its centre and half its size along each axis.
const Box = struct {
    center: [3]f32,
    half_extents: [3]f32,
};

/// A box to query `level` with. Its centre is mostly on or near the ground,
/// at times far above, below or beside the level; its size runs from nothing
/// to several tiles. Half extents may be negative, which turns the box inside
/// out: both sides must treat that alike.
///
/// The centre stays within a few thousand units of the level and the box
/// within tens: the original turns the box into tile coordinates with a cast
/// that is undefined for a float no `int` holds, and walks every tile between.
fn randomBox(random: std.Random, level: *const support.Level) Box {
    const bounds_min = level.mesh.bounds_min;
    const bounds_max = level.mesh.bounds_max;

    var center: [3]f32 = undefined;
    switch (random.uintLessThan(u32, 10)) {
        0...4 => center = level.groundPoint(random, 0.5),
        5, 6 => center = level.groundPoint(random, 3.0),
        7 => {
            // Straight above or below the ground, out of reach of most boxes.
            center = level.groundPoint(random, 0.25);
            center[1] += (random.float(f32) * 2.0 - 1.0) * 60.0;
        },
        8 => {
            // Anywhere in the level's bounds, mostly over no polygon.
            for (&center, bounds_min, bounds_max) |*coordinate, low, high| {
                coordinate.* = low + random.float(f32) * (high - low);
            }
        },
        else => {
            // Beside the level, where no tile is.
            center = level.groundPoint(random, 1.0);
            const axis: usize = if (random.boolean()) 0 else 2;
            const distance = 50.0 + random.float(f32) * 5000.0;
            center[axis] += if (random.boolean()) distance else -distance;
        },
    }

    var half_extents: [3]f32 = undefined;
    switch (random.uintLessThan(u32, 16)) {
        0 => half_extents = .{ 0, 0, 0 },
        1 => {
            // Inside out along some of the axes.
            for (&half_extents) |*extent| extent.* = random.float(f32) * 4.0 - 2.0;
        },
        2 => {
            for (&half_extents) |*extent| extent.* = random.float(f32) * 0.01;
        },
        3 => {
            // Several tiles wide.
            for (&half_extents) |*extent| extent.* = random.float(f32) * 40.0;
        },
        4 => {
            // Flat: reaches only the polygons at the centre's own height.
            half_extents = .{ random.float(f32) * 6.0, 0, random.float(f32) * 6.0 };
        },
        else => {
            for (&half_extents) |*extent| extent.* = random.float(f32) * 6.0;
        },
    }
    return .{ .center = center, .half_extents = half_extents };
}

/// A box that holds all of `level`.
fn wholeLevelBox(level: *const support.Level) Box {
    var box: Box = undefined;
    for (0..3) |axis| {
        const low = level.mesh.bounds_min[axis];
        const high = level.mesh.bounds_max[axis];
        box.center[axis] = (low + high) * 0.5;
        box.half_extents[axis] = (high - low) * 0.5 + 2.0;
    }
    return box;
}

// ---------------------------------------------------------------------------
// findNearestPoly
// ---------------------------------------------------------------------------

/// Runs `findNearestPoly` on both sides with each choice of the outputs a
/// caller may leave out, and fails unless the statuses, references, points
/// and flags are the same bits. Returns the reference found, zero when none.
fn expectNearestPoly(queries: *QueryPair, box: Box, filter: *const FilterPair) !PolyRef {
    errdefer std.debug.print(
        "  findNearestPoly: center {any}, half extents {any}, include {x}, exclude {x}\n",
        .{ box.center, box.half_extents, filter.port.include_flags, filter.port.exclude_flags },
    );

    // Whether the call gets a point output, and whether it gets a flag.
    const output_choices = [_][2]bool{
        .{ true, true },
        .{ true, false },
        .{ false, true },
        .{ false, false },
    };

    var found: PolyRef = untouched_ref;
    for (output_choices) |choice| {
        const want_point = choice[0];
        const want_flag = choice[1];
        errdefer std.debug.print(
            "  with a point output: {any}, with a flag output: {any}\n",
            .{ want_point, want_flag },
        );

        var expected_ref: PolyRef = untouched_ref;
        var expected_point: [3]f32 = untouched_point;
        var expected_flag: FlagOutput = .{};
        const expected = reference.ref_query_find_nearest_poly_over(
            queries.original,
            &box.center,
            &box.half_extents,
            filter.original,
            &expected_ref,
            if (want_point) &expected_point else null,
            expected_flag.pointer(want_flag),
        );

        var actual_ref: PolyRef = untouched_ref;
        var actual_point: [3]f32 = untouched_point;
        var actual_flag: FlagOutput = .{};
        const actual = queries.port.findNearestPoly(
            box.center,
            box.half_extents,
            &filter.port,
            &actual_ref,
            if (want_point) &actual_point else null,
            actual_flag.pointer(want_flag),
        );

        try support.expectStatus(expected, actual);
        try testing.expectEqual(expected_ref, actual_ref);
        try support.expectPoint(expected_point, actual_point);
        try testing.expectEqual(expected_flag.byte, actual_flag.byte);

        if (!want_flag) {
            // The original also has an overload without the flag, which the
            // port answers with the same function.
            var short_ref: PolyRef = untouched_ref;
            var short_point: [3]f32 = untouched_point;
            const short = reference.ref_query_find_nearest_poly(
                queries.original,
                &box.center,
                &box.half_extents,
                filter.original,
                &short_ref,
                if (want_point) &short_point else null,
            );
            try support.expectStatus(short, actual);
            try testing.expectEqual(short_ref, actual_ref);
            try support.expectPoint(short_point, actual_point);
        }

        // Every choice of outputs must find the same polygon.
        if (found == untouched_ref) found = actual_ref;
        try testing.expectEqual(found, actual_ref);
    }
    return if (found == untouched_ref) 0 else found;
}

// ---------------------------------------------------------------------------
// queryPolygons
// ---------------------------------------------------------------------------

/// Result buffers for the two sides of `queryPolygons`. Each has a few
/// entries past the capacity a call is told of, to catch a write beyond it.
const RefBuffers = struct {
    expected: []PolyRef,
    actual: []PolyRef,

    const guard_count = 8;

    /// Buffers for calls with a capacity up to `capacity_max`. Free with
    /// `deinit`.
    fn init(gpa: std.mem.Allocator, capacity_max: usize) !RefBuffers {
        const expected = try gpa.alloc(PolyRef, capacity_max + guard_count);
        errdefer gpa.free(expected);
        const actual = try gpa.alloc(PolyRef, capacity_max + guard_count);
        return .{ .expected = expected, .actual = actual };
    }

    fn deinit(self: *RefBuffers, gpa: std.mem.Allocator) void {
        gpa.free(self.expected);
        gpa.free(self.actual);
        self.* = undefined;
    }

    fn capacityMax(self: *const RefBuffers) usize {
        return self.expected.len - guard_count;
    }
};

/// Runs `queryPolygons` on both sides into buffers of `capacity` entries and
/// fails unless the statuses, the counts and every entry of the buffers are
/// the same, the entries no call may write included. Returns the count
/// written, null when the call failed.
fn expectQueryPolygons(
    queries: *QueryPair,
    box: Box,
    filter: *const FilterPair,
    buffers: *const RefBuffers,
    capacity: usize,
) !?usize {
    errdefer std.debug.print(
        "  queryPolygons: center {any}, half extents {any}, include {x}, exclude {x}, " ++
            "capacity {d}\n",
        .{
            box.center,
            box.half_extents,
            filter.port.include_flags,
            filter.port.exclude_flags,
            capacity,
        },
    );
    // The original writes wherever the capacity says it may, checked by
    // nothing.
    std.debug.assert(capacity <= buffers.capacityMax());

    @memset(buffers.expected, untouched_ref);
    @memset(buffers.actual, untouched_ref);

    var expected_count: i32 = untouched_count;
    const expected = reference.ref_query_query_polygons(
        queries.original,
        &box.center,
        &box.half_extents,
        filter.original,
        buffers.expected.ptr,
        &expected_count,
        @intCast(capacity),
    );

    var actual_count: usize = untouched_port_count;
    const actual = queries.port.queryPolygons(
        box.center,
        box.half_extents,
        &filter.port,
        buffers.actual[0..capacity],
        &actual_count,
    );

    try support.expectStatus(expected, actual);
    try expectCount(expected_count, actual_count);
    try testing.expectEqualSlices(PolyRef, buffers.expected, buffers.actual);
    if (actual.failed) return null;
    return actual_count;
}

// ---------------------------------------------------------------------------
// queryPolygonsWith
// ---------------------------------------------------------------------------

/// What one side handed to its polygon query, flattened so that the two
/// sides compare as one slice. Each batch is the base reference of its tile
/// and its polygon count, then for each polygon its index in the tile, as
/// its pointer tells it, and its reference.
const BatchLog = struct {
    gpa: std.mem.Allocator,
    words: std.ArrayList(u64) = .empty,
    batch_count: usize = 0,
    batch_size_max: usize = 0,
    /// A callback returns nothing, so what goes wrong inside one waits here
    /// for the test to read.
    out_of_memory: bool = false,
    /// A polygon pointer that points at none of its tile's polygons.
    stray_pointer: bool = false,
    /// A batch with another number of references than of polygons.
    uneven_batch: bool = false,

    /// What a stray pointer is logged as, in place of an index.
    const stray_index: u64 = std.math.maxInt(u64);

    fn deinit(self: *BatchLog) void {
        self.words.deinit(self.gpa);
        self.* = undefined;
    }

    fn clear(self: *BatchLog) void {
        self.words.clearRetainingCapacity();
        self.batch_count = 0;
        self.batch_size_max = 0;
        self.out_of_memory = false;
        self.stray_pointer = false;
        self.uneven_batch = false;
    }

    /// Starts a batch of `count` polygons of the tile whose first polygon
    /// has the reference `base`. False when there is no memory to log it.
    fn beginBatch(self: *BatchLog, base: PolyRef, count: usize) bool {
        self.words.ensureUnusedCapacity(self.gpa, 2 + count * 2) catch {
            self.out_of_memory = true;
            return false;
        };
        self.words.appendAssumeCapacity(base);
        self.words.appendAssumeCapacity(count);
        self.batch_count += 1;
        self.batch_size_max = @max(self.batch_size_max, count);
        return true;
    }

    /// Adds a polygon to the batch begun last: its index in its tile, null
    /// for a stray pointer, and the reference handed with it.
    fn addPolygon(self: *BatchLog, index: ?usize, ref: PolyRef) void {
        if (index == null) self.stray_pointer = true;
        const word: u64 = if (index) |i| i else stray_index;
        self.words.appendAssumeCapacity(word);
        self.words.appendAssumeCapacity(ref);
    }
};

/// The index of the polygon at `address` among the `count` polygons of
/// `size` bytes each that begin at `first`; null when it is none of them.
fn polygonIndex(first: usize, count: usize, size: usize, address: usize) ?usize {
    if (address < first) return null;
    const offset = address - first;
    if (offset % size != 0) return null;
    const index = offset / size;
    if (index >= count) return null;
    return index;
}

/// Logs what the port's `queryPolygonsWith` hands to its query.
const PortBatches = struct {
    log: *BatchLog,
    nav: *const NavMesh,

    const vtable: PolyQuery.VTable = .{ .process = process };

    fn polyQuery(self: *PortBatches) PolyQuery {
        return .{ .ptr = self, .vtable = &vtable };
    }

    fn process(
        ptr: *anyopaque,
        tile: *const MeshTile,
        polys: []const *const Poly,
        refs: []const PolyRef,
    ) void {
        const self: *PortBatches = @ptrCast(@alignCast(ptr));
        const log = self.log;
        if (polys.len != refs.len) log.uneven_batch = true;
        const count = @min(polys.len, refs.len);
        if (!log.beginBatch(self.nav.getPolyRefBase(tile), count)) return;

        const first = @intFromPtr(tile.polys.ptr);
        for (polys[0..count], refs[0..count]) |poly, ref| {
            const address = @intFromPtr(poly);
            log.addPolygon(polygonIndex(first, tile.polys.len, @sizeOf(Poly), address), ref);
        }
    }
};

/// Logs what the original's `queryPolygons` hands to its query.
const OriginalBatches = struct {
    log: *BatchLog,
    nav: *const reference.NavMesh,

    fn process(
        user: ?*anyopaque,
        tile: *const reference.MeshTile,
        polys: [*]*reference.Poly,
        refs: [*]PolyRef,
        count: i32,
    ) callconv(.c) void {
        const self: *OriginalBatches = @ptrCast(@alignCast(user.?));
        const log = self.log;
        const batch_len: usize = @intCast(count);
        const base = reference.ref_nav_mesh_get_poly_ref_base(self.nav, tile);
        if (!log.beginBatch(base, batch_len)) return;

        const first = @intFromPtr(reference.ref_tile_polys(tile));
        const poly_count: usize = @intCast(reference.ref_tile_header(tile).?.poly_count);
        for (polys[0..batch_len], refs[0..batch_len]) |poly, ref| {
            const address = @intFromPtr(poly);
            const index = polygonIndex(first, poly_count, @sizeOf(reference.Poly), address);
            log.addPolygon(index, ref);
        }
    }
};

/// The logs of the two sides, kept between calls so that they grow once.
const BatchLogs = struct {
    expected: BatchLog,
    actual: BatchLog,

    /// Free with `deinit`.
    fn init(gpa: std.mem.Allocator) BatchLogs {
        return .{ .expected = .{ .gpa = gpa }, .actual = .{ .gpa = gpa } };
    }

    fn deinit(self: *BatchLogs) void {
        self.expected.deinit();
        self.actual.deinit();
        self.* = undefined;
    }
};

/// Runs `queryPolygonsWith` on both sides and fails unless they hand their
/// query the same batches: the same tiles in the same order, the same
/// polygons by pointer and by reference, cut into batches at the same places.
fn expectPolygonBatches(
    queries: *QueryPair,
    meshes: MeshPair,
    box: Box,
    filter: *const FilterPair,
    logs: *BatchLogs,
) !void {
    errdefer std.debug.print(
        "  queryPolygonsWith: center {any}, half extents {any}, include {x}, exclude {x}\n",
        .{ box.center, box.half_extents, filter.port.include_flags, filter.port.exclude_flags },
    );
    logs.expected.clear();
    logs.actual.clear();

    var original_batches: OriginalBatches = .{ .log = &logs.expected, .nav = meshes.original };
    const expected = reference.ref_query_query_polygons_cb(
        queries.original,
        &box.center,
        &box.half_extents,
        filter.original,
        OriginalBatches.process,
        &original_batches,
    );

    var port_batches: PortBatches = .{ .log = &logs.actual, .nav = meshes.port };
    const actual = queries.port.queryPolygonsWith(
        box.center,
        box.half_extents,
        &filter.port,
        port_batches.polyQuery(),
    );

    try support.expectStatus(expected, actual);
    if (logs.expected.out_of_memory or logs.actual.out_of_memory) return error.OutOfMemory;
    try testing.expect(!logs.expected.stray_pointer);
    try testing.expect(!logs.actual.stray_pointer);
    try testing.expect(!logs.actual.uneven_batch);
    try testing.expectEqualSlices(u64, logs.expected.words.items, logs.actual.words.items);
}

// ---------------------------------------------------------------------------
// Polygons and the positions to ask about them
// ---------------------------------------------------------------------------

/// One polygon of a mesh, as the port holds it.
const PolygonAt = struct {
    ref: PolyRef,
    tile: *const MeshTile,
    poly: *const Poly,
};

/// Every polygon of every tile `nav` holds, in tile and polygon order. The
/// pointers go stale when a tile is added or removed. The caller frees the
/// slice.
fn collectPolygons(gpa: std.mem.Allocator, nav: *const NavMesh) ![]PolygonAt {
    var polygons: std.ArrayList(PolygonAt) = .empty;
    errdefer polygons.deinit(gpa);
    for (nav.tiles) |*tile| {
        if (tile.header == null) continue;
        const base = nav.getPolyRefBase(tile);
        for (tile.polys, 0..) |*poly, i| {
            const index: PolyRef = @intCast(i);
            try polygons.append(gpa, .{ .ref = base | index, .tile = tile, .poly = poly });
        }
    }
    return polygons.toOwnedSlice(gpa);
}

/// The positions to ask about one polygon.
const PointList = struct {
    items: [capacity][3]f32 = undefined,
    len: usize = 0,

    const capacity = 32;

    fn add(self: *PointList, point: [3]f32) void {
        self.items[self.len] = point;
        self.len += 1;
    }

    fn slice(self: *const PointList) []const [3]f32 {
        return self.items[0..self.len];
    }
};

fn lerp(a: [3]f32, b: [3]f32, t: f32) [3]f32 {
    var result: [3]f32 = undefined;
    for (&result, a, b) |*coordinate, from, to| coordinate.* = from + (to - from) * t;
    return result;
}

/// A point inside `poly`, which is convex: its corners weighed at random.
fn randomPointInside(random: std.Random, tile: *const MeshTile, poly: *const Poly) [3]f32 {
    var point: [3]f32 = .{ 0, 0, 0 };
    var weight_sum: f32 = 0;
    for (poly.verts[0..poly.vert_count]) |vert_index| {
        const weight = random.float(f32) + 0.01;
        weight_sum += weight;
        for (&point, tile.verts[vert_index]) |*sum, coordinate| sum.* += coordinate * weight;
    }
    for (&point) |*coordinate| coordinate.* /= weight_sum;
    return point;
}

/// Fills `points` with positions to ask about `poly` of `tile`: its centre,
/// its corners and the middles of its edges, points inside it, the same at
/// other heights, points outside it and points far away.
///
/// The far points stay within 1e15 of the level along x and z. Further out
/// the squared distances the original compares turn infinite, none of them
/// is then the smallest, and it reads through the null pointer it began with.
fn polygonPoints(
    random: std.Random,
    tile: *const MeshTile,
    poly: *const Poly,
    points: *PointList,
) void {
    points.len = 0;
    const vert_count: usize = poly.vert_count;

    var center: [3]f32 = .{ 0, 0, 0 };
    for (poly.verts[0..vert_count]) |vert_index| {
        for (&center, tile.verts[vert_index]) |*sum, coordinate| sum.* += coordinate;
    }
    const vert_count_float: f32 = @floatFromInt(vert_count);
    for (&center) |*coordinate| coordinate.* /= vert_count_float;
    points.add(center);

    for (0..vert_count) |i| {
        const corner = tile.verts[poly.verts[i]];
        const next_corner = tile.verts[poly.verts[(i + 1) % vert_count]];
        points.add(corner);
        points.add(lerp(corner, next_corner, 0.5));
    }

    for (0..4) |_| points.add(randomPointInside(random, tile, poly));

    // Inside along x and z, at other heights: no height may change which
    // point of the polygon is the closest.
    for ([_]f32{ 0.5, -7.0, 250.0, -3.0e38 }) |rise| {
        var point = randomPointInside(random, tile, poly);
        point[1] += rise;
        points.add(point);
    }

    // Outside, from just past a corner to a few polygons away.
    for (0..5) |_| {
        const corner = tile.verts[poly.verts[random.uintLessThan(usize, vert_count)]];
        var point = lerp(center, corner, 1.0 + random.float(f32) * 2.0);
        point[1] += (random.float(f32) * 2.0 - 1.0) * 4.0;
        points.add(point);
    }

    // Far away, where the steps between floats are wider than the polygon.
    for ([_]f32{ 1.0e6, 1.0e12, 1.0e15 }) |distance| {
        var point = center;
        point[0] += if (random.boolean()) distance else -distance;
        point[2] += (random.float(f32) * 2.0 - 1.0) * distance;
        point[1] = (random.float(f32) * 2.0 - 1.0) * 3.0e38;
        points.add(point);
    }
}

/// Where the parts of a polygon reference are: the polygon's index in the
/// low bits, its tile's index above it and the tile's salt above that.
const RefLayout = struct {
    poly_bits: u32,
    tile_bits: u32,
    salt_bits: u32,

    fn of(nav: *const NavMesh) RefLayout {
        if (detour.nav_mesh.polyref64) {
            return .{
                .poly_bits = detour.nav_mesh.poly_bits_64,
                .tile_bits = detour.nav_mesh.tile_bits_64,
                .salt_bits = detour.nav_mesh.salt_bits_64,
            };
        }
        return .{
            .poly_bits = nav.poly_bits,
            .tile_bits = nav.tile_bits,
            .salt_bits = nav.salt_bits,
        };
    }

    fn mask(bits: u32) u64 {
        return std.math.shl(u64, 1, bits) - 1;
    }

    fn polyIndexOf(self: RefLayout, ref: PolyRef) u64 {
        return ref & mask(self.poly_bits);
    }

    fn tileIndexOf(self: RefLayout, ref: PolyRef) u64 {
        return std.math.shr(u64, ref, self.poly_bits) & mask(self.tile_bits);
    }

    fn saltOf(self: RefLayout, ref: PolyRef) u64 {
        return std.math.shr(u64, ref, self.poly_bits + self.tile_bits) & mask(self.salt_bits);
    }

    /// The reference with these parts, each cut to the bits it has.
    fn encode(self: RefLayout, salt: u64, tile_index: u64, poly_index: u64) PolyRef {
        const tile_shift = self.poly_bits;
        const salt_shift = self.poly_bits + self.tile_bits;
        const salt_part = std.math.shl(u64, salt & mask(self.salt_bits), salt_shift);
        const tile_part = std.math.shl(u64, tile_index & mask(self.tile_bits), tile_shift);
        return @truncate(salt_part | tile_part | (poly_index & mask(self.poly_bits)));
    }
};

/// Appends references that name no polygon, or most likely none: zero, all
/// ones, references of `polygons` with one part changed, references into
/// tile entries that hold nothing, and random bits. A few may name a polygon
/// after all; the two sides must tell the same ones apart.
fn appendDoubtfulRefs(
    gpa: std.mem.Allocator,
    random: std.Random,
    nav: *const NavMesh,
    polygons: []const PolygonAt,
    refs: *std.ArrayList(PolyRef),
) !void {
    const layout: RefLayout = .of(nav);
    try refs.append(gpa, 0);
    try refs.append(gpa, std.math.maxInt(PolyRef));

    for (0..32) |_| {
        const polygon = polygons[random.uintLessThan(usize, polygons.len)];
        const salt = layout.saltOf(polygon.ref);
        const tile_index = layout.tileIndexOf(polygon.ref);
        const poly_index = layout.polyIndexOf(polygon.ref);
        const poly_count: u64 = polygon.tile.polys.len;

        // The salt of another life of the tile entry.
        try refs.append(gpa, layout.encode(salt + 1, tile_index, poly_index));
        try refs.append(gpa, layout.encode(salt -% 1, tile_index, poly_index));
        try refs.append(gpa, layout.encode(random.int(u64), tile_index, poly_index));
        // A polygon index past the tile's last polygon.
        const past_end = poly_count + random.uintLessThan(u64, 64);
        try refs.append(gpa, layout.encode(salt, tile_index, poly_count));
        try refs.append(gpa, layout.encode(salt, tile_index, past_end));
        try refs.append(gpa, layout.encode(salt, tile_index, std.math.maxInt(u64)));
        // Another tile entry: one with another tile in it, one with nothing
        // in it, or one past the last entry of the mesh.
        try refs.append(gpa, layout.encode(salt, tile_index + 1, poly_index));
        try refs.append(gpa, layout.encode(salt, random.int(u64), poly_index));
        try refs.append(gpa, layout.encode(salt, std.math.maxInt(u64), poly_index));
        // One bit flipped.
        const bit = random.uintLessThan(u32, @bitSizeOf(PolyRef));
        try refs.append(gpa, polygon.ref ^ std.math.shl(PolyRef, 1, bit));
    }
    for (0..64) |_| try refs.append(gpa, random.int(PolyRef));
}

// ---------------------------------------------------------------------------
// closestPointOnPoly, closestPointOnPolyBoundary, getPolyHeight, isValidPolyRef
// ---------------------------------------------------------------------------

/// Runs `closestPointOnPoly` on both sides, with and without the flag
/// output, and fails unless the statuses, points and flags are the same
/// bits. Returns whether the position is over the polygon, null when the
/// call failed.
fn expectClosestPointOnPoly(queries: *QueryPair, ref: PolyRef, pos: [3]f32) !?bool {
    errdefer std.debug.print("  closestPointOnPoly: ref {x}, pos {any}\n", .{ ref, pos });

    var over: ?bool = null;
    for ([_]bool{ true, false }) |want_flag| {
        var expected_point: [3]f32 = untouched_point;
        var expected_flag: FlagOutput = .{};
        const expected = reference.ref_query_closest_point_on_poly(
            queries.original,
            ref,
            &pos,
            &expected_point,
            expected_flag.pointer(want_flag),
        );

        var actual_point: [3]f32 = untouched_point;
        var actual_flag: FlagOutput = .{};
        const actual = queries.port.closestPointOnPoly(
            ref,
            pos,
            &actual_point,
            actual_flag.pointer(want_flag),
        );

        try support.expectStatus(expected, actual);
        try support.expectPoint(expected_point, actual_point);
        try testing.expectEqual(expected_flag.byte, actual_flag.byte);
        if (want_flag and actual.succeeded) over = actual_flag.byte != 0;
    }
    return over;
}

/// Runs `closestPointOnPolyBoundary` on both sides and fails unless the
/// statuses and points are the same bits. Returns whether the call
/// succeeded.
fn expectClosestPointOnPolyBoundary(queries: *QueryPair, ref: PolyRef, pos: [3]f32) !bool {
    errdefer std.debug.print(
        "  closestPointOnPolyBoundary: ref {x}, pos {any}\n",
        .{ ref, pos },
    );

    var expected_point: [3]f32 = untouched_point;
    const expected = reference.ref_query_closest_point_on_poly_boundary(
        queries.original,
        ref,
        &pos,
        &expected_point,
    );
    var actual_point: [3]f32 = untouched_point;
    const actual = queries.port.closestPointOnPolyBoundary(ref, pos, &actual_point);

    try support.expectStatus(expected, actual);
    try support.expectPoint(expected_point, actual_point);
    return actual.succeeded;
}

/// Runs `getPolyHeight` on both sides, with and without the height output,
/// and fails unless the statuses and heights are the same bits. Returns
/// whether the polygon has a height at the position.
fn expectPolyHeight(queries: *QueryPair, ref: PolyRef, pos: [3]f32) !bool {
    errdefer std.debug.print("  getPolyHeight: ref {x}, pos {any}\n", .{ ref, pos });

    var has_height = false;
    for ([_]bool{ true, false }) |want_height| {
        var expected_height: f32 = untouched_height;
        const expected = reference.ref_query_get_poly_height(
            queries.original,
            ref,
            &pos,
            if (want_height) &expected_height else null,
        );
        var actual_height: f32 = untouched_height;
        const actual = queries.port.getPolyHeight(
            ref,
            pos,
            if (want_height) &actual_height else null,
        );

        try support.expectStatus(expected, actual);
        try support.expectFloat(expected_height, actual_height);
        if (want_height) has_height = actual.succeeded;
    }
    return has_height;
}

/// Runs the query's `isValidPolyRef` on both sides and fails unless they
/// give the same answer, which is returned.
fn expectValidPolyRef(queries: *QueryPair, ref: PolyRef, filter: *const FilterPair) !bool {
    errdefer std.debug.print(
        "  isValidPolyRef: ref {x}, include {x}, exclude {x}\n",
        .{ ref, filter.port.include_flags, filter.port.exclude_flags },
    );
    const expected = reference.ref_query_is_valid_poly_ref(queries.original, ref, filter.original);
    const actual = queries.port.isValidPolyRef(ref, &filter.port);
    try testing.expectEqual(expected, actual);
    return actual;
}

// ---------------------------------------------------------------------------
// Attaching a query
// ---------------------------------------------------------------------------

/// Node counts `init` takes. The three smallest leave the original with a
/// node pool that has no hash buckets, which no point query looks into.
const node_counts_accepted = [_]i32{
    1,
    2,
    3,
    4,
    64,
    2048,
    65535,
};

/// Node counts `init` refuses: a node's index must fit in 16 bits.
const node_counts_refused = [_]i32{
    65536,
    65537,
    1 << 20,
    (1 << 24) - 1,
    1 << 24,
    std.math.maxInt(i32),
};

test "init and deinit: node counts both sides accept and refuse" {
    const gpa = testing.allocator;
    const seed: u64 = 0x51e7_0001;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();
    errdefer std.debug.print("  seed {d}\n", .{seed});

    var filter: FilterPair = try .create();
    defer filter.deinit();

    for ([_]WorldCase{ dungeon_one_tile, undulating_one_tile }) |world_case| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{world_case.name});
        var world: support.World = undefined;
        try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        defer world.deinit();
        const meshes = [_]MeshPair{MeshPair.of(&world)};
        const level = &world.level;

        var queries: QueryPair = try .create();
        defer queries.deinit(gpa);

        // A new query for each count, so that each builds its pools from
        // nothing; freeing the one before is the `deinit` under test.
        for (node_counts_accepted) |node_count| {
            errdefer std.debug.print("  node count {d} on a new query\n", .{node_count});
            try queries.recreate(gpa);
            try testing.expectEqual(@as(?usize, null), try queries.attachedMesh(&meshes));
            const status = try queries.attach(gpa, meshes[0], node_count);
            try testing.expect(status.succeeded);
            try testing.expectEqual(@as(?usize, 0), try queries.attachedMesh(&meshes));
            _ = try expectNearestPoly(&queries, randomBox(random, level), &filter);
        }

        // One query attached over and over: a larger count rebuilds its
        // pools, a smaller one keeps them.
        try queries.recreate(gpa);
        const node_counts_in_turn = [_]i32{
            1,
            4,
            64,
            2048,
            65535,
            2048,
            3,
            64,
            65535,
            2,
        };
        for (node_counts_in_turn) |node_count| {
            errdefer std.debug.print("  node count {d} on a used query\n", .{node_count});
            const status = try queries.attach(gpa, meshes[0], node_count);
            try testing.expect(status.succeeded);
            try testing.expectEqual(@as(?usize, 0), try queries.attachedMesh(&meshes));
            _ = try expectNearestPoly(&queries, randomBox(random, level), &filter);

            // A count neither side takes leaves the query as it was, still
            // able to answer.
            for (node_counts_refused) |refused_count| {
                errdefer std.debug.print("  then node count {d}\n", .{refused_count});
                const refused = try queries.attach(gpa, meshes[0], refused_count);
                try testing.expect(refused.failed);
                try testing.expect(refused.invalid_param);
                try testing.expectEqual(@as(?usize, 0), try queries.attachedMesh(&meshes));
            }
            _ = try expectNearestPoly(&queries, randomBox(random, level), &filter);
        }

        // Refused on a new query: it stays attached to nothing, and takes a
        // good count afterwards.
        for (node_counts_refused) |node_count| {
            errdefer std.debug.print("  node count {d} on a new query\n", .{node_count});
            try queries.recreate(gpa);
            const refused = try queries.attach(gpa, meshes[0], node_count);
            try testing.expect(refused.failed);
            try testing.expectEqual(@as(?usize, null), try queries.attachedMesh(&meshes));

            const status = try queries.attach(gpa, meshes[0], 64);
            try testing.expect(status.succeeded);
            try testing.expectEqual(@as(?usize, 0), try queries.attachedMesh(&meshes));
            _ = try expectNearestPoly(&queries, randomBox(random, level), &filter);
        }
    }
}

test "getAttachedNavMesh: the mesh of the last init both sides accepted" {
    const gpa = testing.allocator;

    // A query only keeps the pointer it is given, so meshes without tiles
    // do. Those of the port own nothing, so there is nothing of theirs to
    // free.
    const port_meshes: [3]NavMesh = .{ .empty, .empty, .empty };
    var original_meshes: [3]*reference.NavMesh = undefined;
    var allocated_count: usize = 0;
    defer for (original_meshes[0..allocated_count]) |mesh| reference.ref_nav_mesh_free(mesh);
    for (&original_meshes) |*mesh| {
        mesh.* = reference.ref_nav_mesh_alloc() orelse return error.OutOfMemory;
        allocated_count += 1;
    }
    var meshes: [3]MeshPair = undefined;
    for (&meshes, &port_meshes, original_meshes) |*mesh, *port, original| {
        mesh.* = .{ .port = port, .original = original };
    }

    var queries: QueryPair = try .create();
    defer queries.deinit(gpa);

    // Attached to nothing at first, and after an init neither side takes.
    try testing.expectEqual(@as(?usize, null), try queries.attachedMesh(&meshes));
    try testing.expect((try queries.attach(gpa, meshes[0], 65536)).failed);
    try testing.expectEqual(@as(?usize, null), try queries.attachedMesh(&meshes));

    const Step = struct {
        mesh: usize,
        node_count: i32,
        /// The mesh the query is attached to after the step.
        attached: usize,
    };
    const steps = [_]Step{
        .{ .mesh = 0, .node_count = 64, .attached = 0 },
        .{ .mesh = 1, .node_count = 64, .attached = 1 },
        // Refused: the mesh of the init before stays.
        .{ .mesh = 2, .node_count = 65536, .attached = 1 },
        .{ .mesh = 2, .node_count = 16, .attached = 2 },
        .{ .mesh = 0, .node_count = 256, .attached = 0 },
        .{ .mesh = 0, .node_count = 256, .attached = 0 },
        .{ .mesh = 1, .node_count = std.math.maxInt(i32), .attached = 0 },
        .{ .mesh = 2, .node_count = 1 << 24, .attached = 0 },
        .{ .mesh = 1, .node_count = 1, .attached = 1 },
        .{ .mesh = 2, .node_count = 65535, .attached = 2 },
    };
    for (steps, 0..) |step, i| {
        errdefer std.debug.print(
            "  step {d}: mesh {d} with node count {d}\n",
            .{ i, step.mesh, step.node_count },
        );
        const status = try queries.attach(gpa, meshes[step.mesh], step.node_count);
        try testing.expectEqual(step.mesh == step.attached, status.succeeded);
        try testing.expectEqual(@as(?usize, step.attached), try queries.attachedMesh(&meshes));
    }

    // New queries are attached to nothing again.
    try queries.recreate(gpa);
    try testing.expectEqual(@as(?usize, null), try queries.attachedMesh(&meshes));
}

// ---------------------------------------------------------------------------
// Boxes: findNearestPoly, queryPolygons, queryPolygonsWith
// ---------------------------------------------------------------------------

test "findNearestPoly: random boxes in every world" {
    const gpa = testing.allocator;
    const seed: u64 = 0x51e7_0003;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var filters: FilterSet = try .create();
    defer filters.deinit();

    const world_cases = [_]WorldCase{
        nav_test_tiled,
        dungeon_one_tile,
        dungeon_tiled_no_tree,
        undulating_one_tile,
    };
    var found_count: usize = 0;
    var missed_count: usize = 0;
    for (world_cases) |world_case| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{world_case.name});
        var world: support.World = undefined;
        try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        defer world.deinit();

        var queries: QueryPair = try .create();
        defer queries.deinit(gpa);
        try queries.ready(gpa, &world);

        for (0..400) |case| {
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
            const box = randomBox(random, &world.level);
            const found = try expectNearestPoly(&queries, box, filters.pick(random));
            if (found != 0) found_count += 1 else missed_count += 1;
        }
    }

    // Both answers must come up often, or the boxes compare little.
    try testing.expect(found_count >= 100);
    try testing.expect(missed_count >= 100);
}

test "queryPolygons: random boxes into buffers of every size" {
    const gpa = testing.allocator;
    const seed: u64 = 0x51e7_0004;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var filters: FilterSet = try .create();
    defer filters.deinit();
    var buffers: RefBuffers = try .init(gpa, 8192);
    defer buffers.deinit(gpa);
    const capacity_max = buffers.capacityMax();

    const world_cases = [_]WorldCase{
        nav_test_tiled,
        dungeon_one_tile,
        nav_test_one_tile_no_tree,
    };
    var overflow_count: usize = 0;
    var empty_count: usize = 0;
    for (world_cases) |world_case| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{world_case.name});
        var world: support.World = undefined;
        try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        defer world.deinit();

        var queries: QueryPair = try .create();
        defer queries.deinit(gpa);
        try queries.ready(gpa, &world);

        for (0..400) |case| {
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
            // The first box takes in every polygon of the level.
            const whole = case == 0;
            const box = if (whole) wholeLevelBox(&world.level) else randomBox(random, &world.level);
            const filter = if (whole) filters.passAll() else filters.pick(random);

            // Into all the room there is first, to learn how many polygons
            // the box touches.
            const found = (try expectQueryPolygons(
                &queries,
                box,
                filter,
                &buffers,
                capacity_max,
            )) orelse return error.QueryFailed;
            if (whole) try testing.expect(found > 32);
            if (found == 0) empty_count += 1;

            // Then into no room, room for one, just enough, one too few,
            // one too many, and any part of it.
            const capacities = [_]usize{
                0,
                1,
                found,
                found -| 1,
                found + 1,
                random.uintLessThan(usize, found + 2),
                random.uintLessThan(usize, 8),
            };
            for (capacities) |wanted_capacity| {
                const capacity = @min(wanted_capacity, capacity_max);
                const written = (try expectQueryPolygons(
                    &queries,
                    box,
                    filter,
                    &buffers,
                    capacity,
                )) orelse return error.QueryFailed;
                try testing.expectEqual(@min(found, capacity), written);
                if (capacity < found) overflow_count += 1;
            }
        }
    }

    // Buffers that are too small and boxes that touch nothing must both
    // come up often.
    try testing.expect(overflow_count >= 200);
    try testing.expect(empty_count >= 50);
}

test "queryPolygonsWith: every batch of random boxes" {
    const gpa = testing.allocator;
    const seed: u64 = 0x51e7_0005;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var filters: FilterSet = try .create();
    defer filters.deinit();
    var logs: BatchLogs = .init(gpa);
    defer logs.deinit();

    const world_cases = [_]WorldCase{
        nav_test_tiled,
        dungeon_one_tile,
        dungeon_tiled_no_tree,
        nav_test_one_tile_no_tree,
    };
    var batch_count: usize = 0;
    var batch_size_max: usize = 0;
    for (world_cases) |world_case| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{world_case.name});
        var world: support.World = undefined;
        try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        defer world.deinit();
        const meshes: MeshPair = .of(&world);

        var queries: QueryPair = try .create();
        defer queries.deinit(gpa);
        try queries.ready(gpa, &world);

        for (0..400) |case| {
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
            // The first box takes in every polygon of the level, so that
            // every tile hands out all it has.
            const whole = case == 0;
            const box = if (whole) wholeLevelBox(&world.level) else randomBox(random, &world.level);
            const filter = if (whole) filters.passAll() else filters.pick(random);

            try expectPolygonBatches(&queries, meshes, box, filter, &logs);
            if (whole) try testing.expect(logs.actual.batch_count > 0);
            batch_count += logs.actual.batch_count;
            batch_size_max = @max(batch_size_max, logs.actual.batch_size_max);
        }
    }

    // A tile with more polygons than a batch holds must have filled one.
    try testing.expectEqual(@as(usize, 32), batch_size_max);
    try testing.expect(batch_count >= 200);
}

/// Where the hand-made boxes are centred, as steps from a point on the
/// ground: on it, above and below it by less than a step, by a storey and by
/// more than any box here is high, and beside it.
const hand_made_center_steps = [_][3]f32{
    .{ 0, 0, 0 },
    .{ 0, 0.4, 0 },
    .{ 0, 3, 0 },
    .{ 0, 40, 0 },
    .{ 0, -0.4, 0 },
    .{ 0, -3, 0 },
    .{ 0, -40, 0 },
    .{ 0.3, 0, -0.3 },
    .{ 4, 0.2, 0 },
    .{ 0, 0.2, -30 },
};

/// The sizes of the hand-made boxes: a point, the usual size, next to
/// nothing, flat, a line, inside out along each axis, wide, and as tall as a
/// float allows.
const hand_made_half_extents = [_][3]f32{
    .{ 0, 0, 0 },
    .{ 2, 4, 2 },
    .{ 1.0e-6, 1.0e-6, 1.0e-6 },
    .{ 0.5, 0, 0.5 },
    .{ 0, 5, 0 },
    .{ 3, 0.25, 0 },
    .{ -1, -1, -1 },
    .{ -2, 4, 2 },
    .{ 2, -4, 2 },
    .{ 2, 4, -2 },
    .{ 12, 3, 12 },
    .{ 2, std.math.floatMax(f32), 2 },
};

/// Asks every box query about `box` on both sides. Returns how many polygons
/// the box touches, null when the queries refuse the box.
fn expectBoxQueries(
    queries: *QueryPair,
    meshes: MeshPair,
    box: Box,
    filter: *const FilterPair,
    buffers: *const RefBuffers,
    logs: *BatchLogs,
) !?usize {
    const nearest = try expectNearestPoly(queries, box, filter);
    const found = try expectQueryPolygons(queries, box, filter, buffers, buffers.capacityMax());
    try expectPolygonBatches(queries, meshes, box, filter, logs);

    const count = found orelse {
        try testing.expectEqual(@as(PolyRef, 0), nearest);
        try testing.expectEqual(@as(usize, 0), logs.actual.batch_count);
        return null;
    };
    // One entry too few, when there is an entry to leave out.
    _ = try expectQueryPolygons(queries, box, filter, buffers, count -| 1);
    // The three queries walk the same polygons: none for one is none for all.
    if (count == 0) {
        try testing.expectEqual(@as(PolyRef, 0), nearest);
        try testing.expectEqual(@as(usize, 0), logs.actual.batch_count);
    }
    return count;
}

test "findNearestPoly, queryPolygons and queryPolygonsWith: hand-made boxes" {
    const gpa = testing.allocator;
    const seed: u64 = 0x51e7_0006;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var pass_all: FilterPair = try .create();
    defer pass_all.deinit();
    var pass_some: FilterPair = try .create();
    defer pass_some.deinit();
    pass_some.setFlags(0x0002, 0);
    var pass_none: FilterPair = try .create();
    defer pass_none.deinit();
    pass_none.setFlags(0, 0);
    const filters = [_]*const FilterPair{ &pass_all, &pass_some, &pass_none };

    var buffers: RefBuffers = try .init(gpa, 8192);
    defer buffers.deinit(gpa);
    var logs: BatchLogs = .init(gpa);
    defer logs.deinit();
    var boxes: std.ArrayList(Box) = .empty;
    defer boxes.deinit(gpa);

    const float_max = std.math.floatMax(f32);
    const usual: [3]f32 = .{ 2, 4, 2 };

    for ([_]WorldCase{ nav_test_tiled, nav_test_one_tile_no_tree }) |world_case| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{world_case.name});
        var world: support.World = undefined;
        try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        defer world.deinit();
        const meshes: MeshPair = .of(&world);

        var queries: QueryPair = try .create();
        defer queries.deinit(gpa);
        try queries.ready(gpa, &world);

        var touched_count: usize = 0;
        for (0..4) |ground_index| {
            const ground = world.level.groundPoint(random, 0.0);
            errdefer std.debug.print(
                "  seed {d}, ground point {d} at {any}\n",
                .{ seed, ground_index, ground },
            );

            boxes.clearRetainingCapacity();
            for (hand_made_center_steps) |step| {
                var center = ground;
                for (&center, step) |*coordinate, offset| coordinate.* += offset;
                for (hand_made_half_extents) |half_extents| {
                    try boxes.append(gpa, .{ .center = center, .half_extents = half_extents });
                }
            }
            const x = ground[0];
            const y = ground[1];
            const z = ground[2];
            const boxes_apart = [_]Box{
                // Wide enough to take in the level from anywhere in it.
                .{ .center = ground, .half_extents = .{ 500, 500, 500 } },
                .{ .center = ground, .half_extents = .{ 500, 1, 500 } },
                .{ .center = ground, .half_extents = .{ 1, 500, 1 } },
                // Far to the side, out to where a float's steps are wider
                // than a tile.
                .{ .center = .{ x + 1.0e6, y, z }, .half_extents = usual },
                .{ .center = .{ x, y, z - 1.0e6 }, .half_extents = usual },
                .{ .center = .{ 1.0e9, y, z }, .half_extents = usual },
                .{ .center = .{ x, y, -1.0e9 }, .half_extents = usual },
                .{ .center = .{ -1.0e9, y, 1.0e9 }, .half_extents = .{ 0, 0, 0 } },
                // At heights no polygon is near. The last two are tall
                // enough to come back to height zero, one from below and
                // one from above, and so to the ground on that side of it;
                // their far side is past what a float holds.
                .{ .center = .{ x, 3.0e38, z }, .half_extents = usual },
                .{ .center = .{ x, -3.0e38, z }, .half_extents = .{ 2, 3.0e38, 2 } },
                .{ .center = .{ x, float_max, z }, .half_extents = .{ 2, float_max, 2 } },
                // Tall enough to reach the ground from such a height. From
                // the first the squared distance is still a float; from the
                // second it is infinite, and no polygon is nearer than that.
                .{ .center = .{ x, 1.0e18, z }, .half_extents = .{ 2, 2.0e18, 2 } },
                .{ .center = .{ x, 1.0e20, z }, .half_extents = .{ 2, 2.0e20, 2 } },
            };
            try boxes.appendSlice(gpa, &boxes_apart);

            for (boxes.items, 0..) |box, box_index| {
                for (filters, 0..) |filter, filter_index| {
                    errdefer std.debug.print(
                        "  box {d}, filter {d}\n",
                        .{ box_index, filter_index },
                    );
                    const count = try expectBoxQueries(
                        &queries,
                        meshes,
                        box,
                        filter,
                        &buffers,
                        &logs,
                    );
                    try testing.expect(count != null);
                    if (count.? > 0) touched_count += 1;
                    if (filter == &pass_none) try testing.expectEqual(@as(?usize, 0), count);
                }
            }

            // A position or a size that is no number is refused before
            // either side looks at a tile.
            if (!rejectsNonFinite()) continue;
            for (not_finite) |bad| {
                for (0..3) |axis| {
                    errdefer std.debug.print("  {d} along axis {d}\n", .{ bad, axis });
                    var bad_center: Box = .{ .center = ground, .half_extents = usual };
                    bad_center.center[axis] = bad;
                    var bad_size: Box = .{ .center = ground, .half_extents = usual };
                    bad_size.half_extents[axis] = bad;
                    for ([_]Box{ bad_center, bad_size }) |box| {
                        const count = try expectBoxQueries(
                            &queries,
                            meshes,
                            box,
                            &pass_all,
                            &buffers,
                            &logs,
                        );
                        try testing.expectEqual(@as(?usize, null), count);
                    }
                }
            }
        }

        // Most of the boxes are at the ground: they must have found some.
        try testing.expect(touched_count >= 40);
    }
}

// ---------------------------------------------------------------------------
// One polygon: closestPointOnPoly, closestPointOnPolyBoundary, getPolyHeight
// ---------------------------------------------------------------------------

test "closestPointOnPoly, closestPointOnPolyBoundary and getPolyHeight: around every polygon" {
    const gpa = testing.allocator;
    const seed: u64 = 0x51e7_0007;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var refs: std.ArrayList(PolyRef) = .empty;
    defer refs.deinit(gpa);

    const world_cases = [_]WorldCase{
        nav_test_tiled,
        dungeon_one_tile,
        dungeon_tiled_no_tree,
        undulating_one_tile,
    };
    var case: usize = 0;
    var over_count: usize = 0;
    var beside_count: usize = 0;
    var height_count: usize = 0;
    var refused_count: usize = 0;
    for (world_cases) |world_case| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{world_case.name});
        var world: support.World = undefined;
        try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        defer world.deinit();

        var queries: QueryPair = try .create();
        defer queries.deinit(gpa);
        try queries.ready(gpa, &world);

        const polygons = try collectPolygons(gpa, &world.nav);
        defer gpa.free(polygons);
        try testing.expect(polygons.len > 0);

        var points: PointList = .{};
        for (polygons) |polygon| {
            const ref = polygon.ref;
            polygonPoints(random, polygon.tile, polygon.poly, &points);
            for (points.slice()) |pos| {
                errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
                // A polygon the mesh holds has a closest point for every
                // position.
                const over = (try expectClosestPointOnPoly(&queries, ref, pos)) orelse
                    return error.LivePolygonRefused;
                try testing.expect(try expectClosestPointOnPolyBoundary(&queries, ref, pos));
                if (try expectPolyHeight(&queries, ref, pos)) height_count += 1;
                if (over) over_count += 1 else beside_count += 1;
                case += 1;
            }

            // A height that is no number: a position is placed on a polygon
            // by its x and z alone, in every build.
            for (not_finite) |bad| {
                errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
                var pos = randomPointInside(random, polygon.tile, polygon.poly);
                pos[1] = bad;
                if (try expectPolyHeight(&queries, ref, pos)) height_count += 1;
                case += 1;
            }
        }

        // References that name nothing, or most likely nothing.
        refs.clearRetainingCapacity();
        try appendDoubtfulRefs(gpa, random, &world.nav, polygons, &refs);
        for (0..1000) |_| try refs.append(gpa, random.int(PolyRef));
        for (refs.items) |ref| {
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
            const pos = world.level.groundPoint(random, 2.0);
            const over = try expectClosestPointOnPoly(&queries, ref, pos);
            _ = try expectClosestPointOnPolyBoundary(&queries, ref, pos);
            _ = try expectPolyHeight(&queries, ref, pos);
            if (over == null) refused_count += 1;
            case += 1;
        }

        // A reference that names nothing is refused before its position is
        // looked at, so with one a position may be anything in every build.
        const layout: RefLayout = .of(&world.nav);
        for (0..64) |_| {
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
            const polygon = polygons[random.uintLessThan(usize, polygons.len)];
            const stale = layout.encode(
                layout.saltOf(polygon.ref) + 1,
                layout.tileIndexOf(polygon.ref),
                layout.polyIndexOf(polygon.ref),
            );
            for ([_]PolyRef{ 0, stale }) |ref| {
                var pos = world.level.groundPoint(random, 1.0);
                const axis = random.uintLessThan(usize, 3);
                pos[axis] = not_finite[random.uintLessThan(usize, not_finite.len)];
                const over = try expectClosestPointOnPoly(&queries, ref, pos);
                try testing.expectEqual(@as(?bool, null), over);
                const on_boundary = try expectClosestPointOnPolyBoundary(&queries, ref, pos);
                try testing.expect(!on_boundary);
                const has_height = try expectPolyHeight(&queries, ref, pos);
                try testing.expect(!has_height);
            }
            case += 1;
        }

        // With a polygon the mesh holds, a position that is no number is
        // only for the builds that refuse it.
        if (!rejectsNonFinite()) continue;
        for (polygons, 0..) |polygon, polygon_index| {
            if (polygon_index % 5 != 0) continue;
            const ref = polygon.ref;
            for (not_finite) |bad| {
                for (0..3) |axis| {
                    errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
                    var pos = randomPointInside(random, polygon.tile, polygon.poly);
                    pos[axis] = bad;
                    const over = try expectClosestPointOnPoly(&queries, ref, pos);
                    try testing.expectEqual(@as(?bool, null), over);
                    const on_boundary = try expectClosestPointOnPolyBoundary(&queries, ref, pos);
                    try testing.expect(!on_boundary);
                    const has_height = try expectPolyHeight(&queries, ref, pos);
                    if (axis != 1) try testing.expect(!has_height);
                    case += 1;
                }
            }
        }
    }

    // Positions over a polygon and beside it, polygons with a height and
    // references both sides refuse must all have come up often.
    try testing.expect(over_count >= 1000);
    try testing.expect(beside_count >= 1000);
    try testing.expect(height_count >= 1000);
    try testing.expect(refused_count >= 1000);
}

// ---------------------------------------------------------------------------
// isValidPolyRef
// ---------------------------------------------------------------------------

test "isValidPolyRef: filters that pass and reject, references that name nothing" {
    const gpa = testing.allocator;
    const seed: u64 = 0x51e7_0008;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var filters: FilterSet = try .create();
    defer filters.deinit();
    var refs: std.ArrayList(PolyRef) = .empty;
    defer refs.deinit(gpa);

    var case: usize = 0;
    var valid_count: usize = 0;
    var invalid_count: usize = 0;
    for ([_]WorldCase{ nav_test_tiled, dungeon_one_tile }) |world_case| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{world_case.name});
        var world: support.World = undefined;
        try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        defer world.deinit();

        var queries: QueryPair = try .create();
        defer queries.deinit(gpa);
        try queries.ready(gpa, &world);

        const polygons = try collectPolygons(gpa, &world.nav);
        defer gpa.free(polygons);
        try testing.expect(polygons.len > 0);

        // Every polygon the mesh holds against every filter.
        for (polygons) |polygon| {
            for (&filters.pairs) |*filter| {
                errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
                const valid = try expectValidPolyRef(&queries, polygon.ref, filter);
                if (valid) valid_count += 1 else invalid_count += 1;
                case += 1;
            }
        }

        // References that name nothing, or most likely nothing: no filter
        // passes what is not there.
        refs.clearRetainingCapacity();
        try appendDoubtfulRefs(gpa, random, &world.nav, polygons, &refs);
        for (0..1000) |_| try refs.append(gpa, random.int(PolyRef));
        for (refs.items) |ref| {
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
            _ = try expectValidPolyRef(&queries, ref, filters.passAll());
            _ = try expectValidPolyRef(&queries, ref, filters.pick(random));
            case += 1;
        }

        // Flags changed after the query was attached: a filter reads them
        // as they are now.
        for (0..200) |_| {
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
            const ref = polygons[random.uintLessThan(usize, polygons.len)].ref;
            const flags: u16 = switch (random.uintLessThan(u32, 4)) {
                0 => 0,
                1 => 0xffff,
                else => random.int(u16),
            };
            errdefer std.debug.print("  ref {x} given the flags {x}\n", .{ ref, flags });
            const expected = reference.ref_nav_mesh_set_poly_flags(world.reference_nav, ref, flags);
            const actual = world.nav.setPolyFlags(ref, flags);
            try support.expectStatus(expected, actual);
            try testing.expect(actual.succeeded);

            for (&filters.pairs) |*filter| {
                const valid = try expectValidPolyRef(&queries, ref, filter);
                if (valid) valid_count += 1 else invalid_count += 1;
            }
            case += 1;
        }
    }

    // Both answers must have come up often.
    try testing.expect(valid_count >= 500);
    try testing.expect(invalid_count >= 500);
}

// ---------------------------------------------------------------------------
// References that outlive their tile
// ---------------------------------------------------------------------------

/// A polygon of a tile that is about to be removed: its reference and a
/// position on it, kept apart because the tile's memory goes with the tile.
const RemovedPolygon = struct {
    ref: PolyRef,
    pos: [3]f32,
};

/// The index of a tile entry of `nav` that holds a tile with ground polygons,
/// any of them with the same chance; null when there is none.
fn randomGroundTile(random: std.Random, nav: *const NavMesh) ?usize {
    var picked: ?usize = null;
    var seen_count: usize = 0;
    for (nav.tiles, 0..) |*tile, i| {
        if (tile.header == null) continue;
        if (tile.polys.len == 0 or tile.polys[0].getType() != .ground) continue;
        seen_count += 1;
        if (random.uintLessThan(usize, seen_count) == 0) picked = i;
    }
    return picked;
}

/// Asks every single-polygon query about each of `polygons` and fails unless
/// both sides refuse them all.
fn expectAllRefused(
    queries: *QueryPair,
    polygons: []const RemovedPolygon,
    filter: *const FilterPair,
) !void {
    for (polygons) |polygon| {
        const over = try expectClosestPointOnPoly(queries, polygon.ref, polygon.pos);
        try testing.expectEqual(@as(?bool, null), over);
        const on_boundary = try expectClosestPointOnPolyBoundary(queries, polygon.ref, polygon.pos);
        try testing.expect(!on_boundary);
        const has_height = try expectPolyHeight(queries, polygon.ref, polygon.pos);
        try testing.expect(!has_height);
        const valid = try expectValidPolyRef(queries, polygon.ref, filter);
        try testing.expect(!valid);
    }
}

test "every point query: references of a tile removed and added again" {
    const gpa = testing.allocator;
    const seed: u64 = 0x51e7_0009;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var pass_all: FilterPair = try .create();
    defer pass_all.deinit();
    var buffers: RefBuffers = try .init(gpa, 8192);
    defer buffers.deinit(gpa);
    var logs: BatchLogs = .init(gpa);
    defer logs.deinit();
    var removed: std.ArrayList(RemovedPolygon) = .empty;
    defer removed.deinit(gpa);

    const world_case = nav_test_tiled;
    errdefer std.debug.print("  in world \"{s}\", seed {d}\n", .{ world_case.name, seed });
    var world: support.World = undefined;
    try world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
    defer world.deinit();
    const meshes: MeshPair = .of(&world);
    const whole_level = wholeLevelBox(&world.level);

    // Attached once: the queries must follow the mesh as its tiles come and
    // go.
    var queries: QueryPair = try .create();
    defer queries.deinit(gpa);
    try queries.ready(gpa, &world);

    for (0..6) |round| {
        const tile_index = randomGroundTile(random, &world.nav) orelse return error.NoGroundTile;
        const tile = &world.nav.tiles[tile_index];
        // Read from the header now: it is part of the memory that goes.
        const header = tile.header.?;
        const x = header.x;
        const y = header.y;
        const layer = header.layer;
        var hole: Box = undefined;
        for (0..3) |axis| {
            const low = header.bounds_min[axis];
            const high = header.bounds_max[axis];
            hole.center[axis] = (low + high) * 0.5;
            hole.half_extents[axis] = (high - low) * 0.5 + 1.0;
        }
        errdefer std.debug.print(
            "  round {d}: tile entry {d}, tile ({d}, {d})\n",
            .{ round, tile_index, x, y },
        );

        removed.clearRetainingCapacity();
        const base = world.nav.getPolyRefBase(tile);
        // The polygons a filter that passes everything lets through: those
        // with a flag.
        var found_by_box_count: usize = 0;
        for (tile.polys, 0..) |*poly, i| {
            const index: PolyRef = @intCast(i);
            const pos = randomPointInside(random, tile, poly);
            try removed.append(gpa, .{ .ref = base | index, .pos = pos });
            if (poly.getType() == .ground and poly.flags != 0) found_by_box_count += 1;
        }

        // While the tile is there, each of its polygons answers.
        for (removed.items) |polygon| {
            const over = try expectClosestPointOnPoly(&queries, polygon.ref, polygon.pos);
            try testing.expect(over != null);
            _ = try expectClosestPointOnPolyBoundary(&queries, polygon.ref, polygon.pos);
            _ = try expectPolyHeight(&queries, polygon.ref, polygon.pos);
            _ = try expectValidPolyRef(&queries, polygon.ref, &pass_all);
        }
        const count_before = (try expectBoxQueries(
            &queries,
            meshes,
            whole_level,
            &pass_all,
            &buffers,
            &logs,
        )) orelse return error.QueryFailed;

        // Remove the tile on both sides.
        const tile_ref = world.nav.getTileRef(tile);
        const original_tile_ref = reference.ref_nav_mesh_get_tile_ref_at(
            world.reference_nav,
            x,
            y,
            layer,
        );
        try testing.expectEqual(original_tile_ref, tile_ref);
        const expected = reference.ref_nav_mesh_remove_tile(
            world.reference_nav,
            tile_ref,
            null,
            null,
        );
        const actual = world.nav.removeTile(tile_ref, null);
        try support.expectStatus(expected, actual);
        try testing.expect(actual.succeeded);

        // Its references name nothing now, and the boxes find a hole where
        // it was.
        try expectAllRefused(&queries, removed.items, &pass_all);
        const count_without = (try expectBoxQueries(
            &queries,
            meshes,
            whole_level,
            &pass_all,
            &buffers,
            &logs,
        )) orelse return error.QueryFailed;
        // A count as large as the buffers tells nothing: polygons were left
        // out of it.
        const counted_all = count_before < buffers.capacityMax();
        if (counted_all and found_by_box_count > 0) {
            try testing.expect(count_without < count_before);
        }
        _ = try expectBoxQueries(&queries, meshes, hole, &pass_all, &buffers, &logs);
        for (removed.items) |polygon| {
            const beside: Box = .{ .center = polygon.pos, .half_extents = .{ 2, 4, 2 } };
            _ = try expectBoxQueries(&queries, meshes, beside, &pass_all, &buffers, &logs);
        }
        for (0..20) |_| {
            const box = randomBox(random, &world.level);
            _ = try expectBoxQueries(&queries, meshes, box, &pass_all, &buffers, &logs);
        }

        // Build the tile again. It goes into the same entry under a new
        // salt, so the old references still name nothing.
        try testing.expect(try world.addTile(x, y));
        try expectAllRefused(&queries, removed.items, &pass_all);

        const new_tile = world.nav.getTileAt(x, y, layer) orelse return error.TileNotAdded;
        const new_base = world.nav.getPolyRefBase(new_tile);
        for (new_tile.polys, 0..) |*poly, i| {
            const index: PolyRef = @intCast(i);
            const ref = new_base | index;
            const pos = randomPointInside(random, new_tile, poly);
            const over = try expectClosestPointOnPoly(&queries, ref, pos);
            try testing.expect(over != null);
            _ = try expectClosestPointOnPolyBoundary(&queries, ref, pos);
            _ = try expectPolyHeight(&queries, ref, pos);
            _ = try expectValidPolyRef(&queries, ref, &pass_all);
        }
        const count_after = (try expectBoxQueries(
            &queries,
            meshes,
            whole_level,
            &pass_all,
            &buffers,
            &logs,
        )) orelse return error.QueryFailed;
        if (counted_all and found_by_box_count > 0) {
            try testing.expect(count_after > count_without);
        }
        _ = try expectBoxQueries(&queries, meshes, hole, &pass_all, &buffers, &logs);
    }
}
