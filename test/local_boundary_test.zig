//! Differential tests of the local boundary: `LocalBoundary` against the
//! original's `dtLocalBoundary`, driven with the same calls over the same
//! navigation meshes.
//!
//! After every call the whole boundary is compared: the centre, the segment
//! count and each kept segment's six floats, in order. The original shows
//! neither the distances it orders the segments by nor the polygons it
//! gathered them from, so the port's are checked against the original's own
//! queries: the polygon list is what its `findLocalNeighbourhood` returns,
//! and the kept distances are the smallest of its distances from the centre
//! to the wall segments of those polygons.

const std = @import("std");
const detour = @import("detour");
const support = @import("support.zig");
const reference = support.reference;
const reference_crowd = support.reference_crowd;

const testing = std.testing;

const PolyRef = detour.PolyRef;
const TileRef = detour.TileRef;
const Status = detour.Status;
const NavMesh = detour.NavMesh;
const NavMeshQuery = detour.NavMeshQuery;
const QueryFilter = detour.QueryFilter;
const LocalBoundary = detour.LocalBoundary;

/// Updates the fuzz test runs on each world.
const fuzz_case_count = 1200;
/// The most wall segments one polygon gives: upstream's `MAX_SEGS_PER_POLY`.
const segments_per_poly_max = detour.nav_mesh.verts_per_polygon * 3;
/// The most wall segments one update can meet.
const candidate_count_max = LocalBoundary.poly_count_max * segments_per_poly_max;

/// A reference no mesh holds: its salt or its tile index matches no tile.
const sentinel_ref: PolyRef = std.math.maxInt(PolyRef) - 0x1234;

/// False in a fast-math build, where neither side refuses a position that
/// is not finite; the tests keep such positions out then.
const non_finite_checked = !detour.common.isFinite(std.math.inf(f32));

// ---------------------------------------------------------------------------
// Worlds
// ---------------------------------------------------------------------------

const WorldCase = struct {
    name: []const u8,
    obj_text: []const u8,
    config: support.RecastConfig,
    options: support.LevelOptions,
};

const world_cases = [_]WorldCase{
    .{
        .name = "dungeon, one tile, off-mesh connections",
        .obj_text = support.dungeon_obj,
        .config = support.solo_config,
        .options = .{ .off_mesh_connection_count = 8, .vary_polygons = true, .seed = 31 },
    },
    .{
        .name = "dungeon, tiled",
        .obj_text = support.dungeon_obj,
        .config = support.tiled_config,
        .options = .{ .vary_polygons = true, .seed = 32 },
    },
    .{
        .name = "nav_test, one tile",
        .obj_text = support.nav_test_obj,
        .config = support.solo_config,
        .options = .{ .vary_polygons = true, .seed = 33 },
    },
    .{
        .name = "nav_test, tiled, off-mesh connections",
        .obj_text = support.nav_test_obj,
        .config = support.tiled_config,
        .options = .{ .off_mesh_connection_count = 8, .vary_polygons = true, .seed = 34 },
    },
};

// ---------------------------------------------------------------------------
// Both sides of a query, a filter and a boundary
// ---------------------------------------------------------------------------

/// The port's query and the original's over the same world.
const Queries = struct {
    gpa: std.mem.Allocator,
    port: NavMeshQuery,
    original: *reference.NavMeshQuery,

    /// Readies both queries in place and compares what the two `init` calls
    /// return. The world must outlive the queries.
    fn init(self: *Queries, gpa: std.mem.Allocator, world: *const support.World) !void {
        const node_count = 512;
        const original = reference.ref_query_alloc() orelse return error.OutOfMemory;
        errdefer reference.ref_query_free(original);
        self.* = .{ .gpa = gpa, .port = .empty, .original = original };
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

    fn setFlags(self: *Filters, include: u16, exclude: u16) void {
        self.port.include_flags = include;
        self.port.exclude_flags = exclude;
        reference.ref_filter_set_include_flags(self.original, include);
        reference.ref_filter_set_exclude_flags(self.original, exclude);
    }

    /// Mostly the filter that passes every polygon; otherwise one that
    /// shuts polygons out by a flag (their neighbours' shared edges turn
    /// into walls), lets in only the polygons with one flag, or lets in
    /// none.
    fn randomize(self: *Filters, random: std.Random) void {
        switch (random.uintLessThan(u32, 10)) {
            0, 1, 2 => self.setFlags(0xffff, @as(u16, 2) << random.uintLessThan(u4, 3)),
            3 => self.setFlags(0xffff, 2 | 4 | 8),
            4 => self.setFlags(@as(u16, 2) << random.uintLessThan(u4, 3), 0),
            5 => self.setFlags(0x8000, 0),
            else => self.setFlags(0xffff, 0),
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

/// A place on the mesh; an error when a world gives none in a few tries.
fn somePlace(
    queries: *const Queries,
    pass_all: *const Filters,
    world: *const support.World,
    random: std.Random,
) !Place {
    for (0..64) |_| {
        const place = nearestPlace(queries, pass_all, world.level.groundPoint(random, 0.3));
        if (place.ref != 0) return place;
    }
    return error.TestUnexpectedResult;
}

/// The same boundary on both sides.
const Boundaries = struct {
    port: LocalBoundary,
    original: *reference_crowd.LocalBoundary,

    fn init() !Boundaries {
        const original = reference_crowd.ref_local_boundary_create() orelse
            return error.OutOfMemory;
        return .{ .port = .empty, .original = original };
    }

    fn deinit(self: *Boundaries) void {
        reference_crowd.ref_local_boundary_destroy(self.original);
    }

    /// Compares all the original shows of its boundary with the port's: the
    /// centre, the segment count and every kept segment, in order.
    fn expectSame(self: *const Boundaries) !void {
        try support.expectPoint(
            reference_crowd.ref_local_boundary_get_center(self.original).*,
            self.port.getCenter(),
        );
        const count = reference_crowd.ref_local_boundary_get_segment_count(self.original);
        try testing.expectEqual(count, self.port.getSegmentCount());
        try testing.expect(count >= 0 and count <= LocalBoundary.segment_count_max);
        var i: i32 = 0;
        while (i < count) : (i += 1) {
            errdefer std.debug.print("  segment {d} of {d}\n", .{ i, count });
            const expected = reference_crowd.ref_local_boundary_get_segment(self.original, i);
            const actual = self.port.getSegment(i);
            try support.expectPoint(expected[0..3].*, actual[0]);
            try support.expectPoint(expected[3..6].*, actual[1]);
        }
    }

    /// `reset` on both sides.
    fn reset(self: *Boundaries) !void {
        reference_crowd.ref_local_boundary_reset(self.original);
        self.port.reset();
        try self.expectSame();
        try testing.expectEqual(@as(i32, 0), self.port.poly_count);
    }

    /// `update` on both sides with the same arguments, then the whole
    /// boundary compared. Returns how many wall segments lay in range, of
    /// which a boundary keeps the nearest `LocalBoundary.segment_count_max`.
    fn update(
        self: *Boundaries,
        queries: *Queries,
        filters: *const Filters,
        ref: PolyRef,
        pos: [3]f32,
        range: f32,
    ) !usize {
        errdefer std.debug.print(
            "  update: ref 0x{x}, pos {any}, range {e}, include 0x{x}, exclude 0x{x}\n",
            .{ ref, pos, range, filters.port.include_flags, filters.port.exclude_flags },
        );
        reference_crowd.ref_local_boundary_update(
            self.original,
            ref,
            &pos,
            range,
            queries.original,
            filters.original,
        );
        self.port.update(ref, pos, range, &queries.port, &filters.port);
        try self.expectSame();
        return self.expectGathered(queries, filters, ref, pos, range);
    }

    /// What the original keeps to itself, checked against its own queries
    /// run with the arguments of the `update` just made: the polygons
    /// gathered, and the kept segments' distances, which are the smallest
    /// of the distances to the wall segments in range, nearest first.
    /// Returns the number of wall segments in range.
    fn expectGathered(
        self: *const Boundaries,
        queries: *const Queries,
        filters: *const Filters,
        ref: PolyRef,
        pos: [3]f32,
        range: f32,
    ) !usize {
        const port = &self.port;
        if (ref == 0) {
            try testing.expectEqual(@as(i32, 0), port.poly_count);
            try testing.expectEqual(@as(i32, 0), port.segment_count);
            return 0;
        }

        var polys: [LocalBoundary.poly_count_max]PolyRef = @splat(sentinel_ref);
        var poly_count: i32 = 0;
        _ = reference.ref_query_find_local_neighbourhood(
            queries.original,
            ref,
            &pos,
            range,
            filters.original,
            &polys,
            null,
            &poly_count,
            LocalBoundary.poly_count_max,
        );
        try testing.expectEqual(poly_count, port.poly_count);
        const gathered = polys[0..@intCast(poly_count)];
        try testing.expectEqualSlices(PolyRef, gathered, port.polys[0..gathered.len]);

        var distances: [candidate_count_max]f32 = undefined;
        var distance_count: usize = 0;
        for (gathered) |poly| {
            var segments: [segments_per_poly_max][6]f32 = undefined;
            var segment_count: i32 = 0;
            _ = reference.ref_query_get_poly_wall_segments(
                queries.original,
                poly,
                filters.original,
                @ptrCast(&segments),
                null,
                &segment_count,
                segments_per_poly_max,
            );
            for (segments[0..@intCast(segment_count)]) |*segment| {
                var t: f32 = 0;
                const distance = reference.ref_distance_pt_seg_sqr_2d(
                    &pos,
                    segment[0..3],
                    segment[3..6],
                    &t,
                );
                if (distance > range * range) continue;
                distances[distance_count] = distance;
                distance_count += 1;
            }
        }

        const kept: usize = @min(distance_count, LocalBoundary.segment_count_max);
        try testing.expectEqual(kept, @as(usize, @intCast(port.segment_count)));
        std.mem.sort(f32, distances[0..distance_count], {}, std.sort.asc(f32));
        for (distances[0..kept], port.segments[0..kept], 0..) |expected, segment, i| {
            errdefer std.debug.print("  distance {d} of {d}\n", .{ i, kept });
            try support.expectFloat(expected, segment.d);
        }
        return distance_count;
    }

    /// `isValid` on both sides, which must agree, change nothing, and say
    /// what the original's query says of each gathered polygon.
    fn expectSameValidity(
        self: *Boundaries,
        queries: *Queries,
        filters: *const Filters,
    ) !bool {
        const expected = reference_crowd.ref_local_boundary_is_valid(
            self.original,
            queries.original,
            filters.original,
        );
        const actual = self.port.isValid(&queries.port, &filters.port);
        try testing.expectEqual(expected, actual);
        try self.expectSame();

        var every = self.port.poly_count > 0;
        for (self.port.polys[0..@intCast(self.port.poly_count)]) |poly| {
            const passes = reference.ref_query_is_valid_poly_ref(
                queries.original,
                poly,
                filters.original,
            );
            if (!passes) every = false;
        }
        try testing.expectEqual(every, actual);
        return actual;
    }
};

// ---------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------

const negative_zero: f32 = @bitCast(@as(u32, 0x8000_0000));

/// A collision query range: mostly typical (2 to 12); otherwise zero, tiny,
/// huge, or one the neighbourhood query refuses.
fn randomRange(random: std.Random) f32 {
    return switch (random.uintLessThan(u32, 20)) {
        0 => 0.0,
        1 => negative_zero,
        2 => std.math.floatTrueMin(f32),
        3 => 1.0e-6 + random.float(f32) * 1.0e-3,
        4 => random.float(f32) * 0.5,
        5 => 1.0e3 + random.float(f32) * 1.0e4,
        // Around the square root of the largest float: the squared range
        // is finite for some and infinite for the rest.
        6 => 1.0e19 + random.float(f32) * 2.0e19,
        7 => std.math.floatMax(f32),
        8 => -1.0 - random.float(f32) * 8.0,
        9 => switch (random.uintLessThan(u32, 3)) {
            0 => std.math.nan(f32),
            1 => std.math.inf(f32),
            else => -std.math.inf(f32),
        },
        else => 2.0 + random.float(f32) * 10.0,
    };
}

/// `ref` with another salt: a reference its tile no longer answers to.
fn staleRef(nav: *const NavMesh, ref: PolyRef) PolyRef {
    const parts = nav.decodePolyId(ref);
    return nav.encodePolyId(parts.salt + 1, parts.tile_index, parts.poly_index);
}

/// One of the vertices of the polygon a valid `ref` names.
fn polyVertex(nav: *const NavMesh, ref: PolyRef, random: std.Random) [3]f32 {
    const found = nav.getTileAndPolyByRefUnsafe(ref);
    const pick = random.uintLessThan(usize, found.poly.vert_count);
    return found.tile.verts[found.poly.verts[pick]];
}

/// Sets a polygon's flags on both sides, which must both take them.
fn setPolyFlags(world: *support.World, ref: PolyRef, flags: u16) !void {
    const expected = reference.ref_nav_mesh_set_poly_flags(world.reference_nav, ref, flags);
    const actual = world.nav.setPolyFlags(ref, flags);
    try support.expectStatus(expected, actual);
    try testing.expect(actual.succeeded);
}

/// A polygon's flags, which must be the same on both sides.
fn polyFlags(world: *const support.World, ref: PolyRef) !u16 {
    var expected: u16 = 0xa5a5;
    var actual: u16 = 0xa5a5;
    try support.expectStatus(
        reference.ref_nav_mesh_get_poly_flags(world.reference_nav, ref, &expected),
        world.nav.getPolyFlags(ref, &actual),
    );
    try testing.expectEqual(expected, actual);
    return actual;
}

/// A wall segment and the polygon it bounds.
const Wall = struct {
    ref: PolyRef,
    start: [3]f32,
    end: [3]f32,
};

/// The wall segments of every ground polygon of the world that passes
/// `filters`, as the original lists them, in tile then polygon order.
fn collectWalls(
    gpa: std.mem.Allocator,
    world: *const support.World,
    queries: *const Queries,
    filters: *const Filters,
) ![]Wall {
    var list: std.ArrayList(Wall) = .empty;
    errdefer list.deinit(gpa);
    const nav = &world.nav;
    var tile_index: i32 = 0;
    while (tile_index < nav.getMaxTiles()) : (tile_index += 1) {
        const tile = nav.getTile(tile_index);
        if (tile.header == null) continue;
        const base = nav.getPolyRefBase(tile);
        for (tile.polys, 0..) |*poly, poly_index| {
            if (poly.getType() != .ground) continue;
            const ref = base | @as(PolyRef, @intCast(poly_index));
            if (!reference.ref_query_is_valid_poly_ref(
                queries.original,
                ref,
                filters.original,
            )) continue;
            var segments: [segments_per_poly_max][6]f32 = undefined;
            var segment_count: i32 = 0;
            _ = reference.ref_query_get_poly_wall_segments(
                queries.original,
                ref,
                filters.original,
                @ptrCast(&segments),
                null,
                &segment_count,
                segments_per_poly_max,
            );
            for (segments[0..@intCast(segment_count)]) |segment| {
                try list.append(gpa, .{
                    .ref = ref,
                    .start = segment[0..3].*,
                    .end = segment[3..6].*,
                });
            }
        }
    }
    return list.toOwnedSlice(gpa);
}

/// The corners where two walls of one polygon meet at a distance of exactly
/// zero: each is the start of a wall that the original's distance function
/// puts on another wall of the same polygon too. `walls` is in the order
/// `collectWalls` gives, a polygon's walls together.
fn collectCorners(gpa: std.mem.Allocator, walls: []const Wall) ![]Place {
    var list: std.ArrayList(Place) = .empty;
    errdefer list.deinit(gpa);
    var first: usize = 0;
    while (first < walls.len) {
        var end = first + 1;
        while (end < walls.len and walls[end].ref == walls[first].ref) end += 1;
        const own = walls[first..end];
        for (own, 0..) |wall, wall_index| {
            for (own, 0..) |other, other_index| {
                if (other_index == wall_index) continue;
                var t: f32 = 0;
                const distance = reference.ref_distance_pt_seg_sqr_2d(
                    &wall.start,
                    &other.start,
                    &other.end,
                    &t,
                );
                if (distance != 0.0) continue;
                try list.append(gpa, .{ .ref = wall.ref, .pos = wall.start });
                break;
            }
        }
        first = end;
    }
    return list.toOwnedSlice(gpa);
}

/// An off-mesh connection's polygon and its two ends.
const Connection = struct {
    ref: PolyRef,
    ends: [2][3]f32,
};

/// The off-mesh connection polygons of the world, in tile then polygon
/// order.
fn collectConnections(gpa: std.mem.Allocator, nav: *const NavMesh) ![]Connection {
    var list: std.ArrayList(Connection) = .empty;
    errdefer list.deinit(gpa);
    var tile_index: i32 = 0;
    while (tile_index < nav.getMaxTiles()) : (tile_index += 1) {
        const tile = nav.getTile(tile_index);
        if (tile.header == null) continue;
        const base = nav.getPolyRefBase(tile);
        for (tile.polys, 0..) |*poly, poly_index| {
            if (poly.getType() != .off_mesh_connection) continue;
            try list.append(gpa, .{
                .ref = base | @as(PolyRef, @intCast(poly_index)),
                .ends = .{ tile.verts[poly.verts[0]], tile.verts[poly.verts[1]] },
            });
        }
    }
    return list.toOwnedSlice(gpa);
}

/// How many neighbours in the port's kept list lie at exactly the same
/// distance from the centre.
fn tieCount(boundary: *const LocalBoundary) usize {
    const kept: usize = @intCast(boundary.segment_count);
    var ties: usize = 0;
    var i: usize = 1;
    while (i < kept) : (i += 1) {
        if (boundary.segments[i - 1].d == boundary.segments[i].d) ties += 1;
    }
    return ties;
}

// ---------------------------------------------------------------------------
// A world with everything a boundary test needs
// ---------------------------------------------------------------------------

/// A world, a query pair over it, a filter pair that passes every polygon
/// and a boundary pair.
const Fixture = struct {
    world: support.World,
    queries: Queries,
    pass_all: Filters,
    boundaries: Boundaries,

    /// Builds everything in place; `self` must not move afterwards.
    fn init(self: *Fixture, gpa: std.mem.Allocator, world_case: WorldCase) !void {
        try self.world.init(gpa, world_case.obj_text, world_case.config, world_case.options);
        errdefer self.world.deinit();
        try self.queries.init(gpa, &self.world);
        errdefer self.queries.deinit();
        self.pass_all = try Filters.init();
        errdefer self.pass_all.deinit();
        self.boundaries = try Boundaries.init();
    }

    fn deinit(self: *Fixture) void {
        self.boundaries.deinit();
        self.pass_all.deinit();
        self.queries.deinit();
        self.world.deinit();
    }
};

/// Gathers a boundary with the pass-all filter, removes the tile under its
/// start polygon from both meshes and puts the tile back, comparing
/// `isValid` and fresh updates at every stage. With `reuse` the tile comes
/// back under its old reference, so the boundary is valid again; otherwise
/// the tile gets a new salt and the boundary stays stale.
fn expectTileRemovalSeen(fixture: *Fixture, random: std.Random, reuse: bool) !void {
    const world = &fixture.world;
    const queries = &fixture.queries;
    const pass_all = &fixture.pass_all;
    const boundaries = &fixture.boundaries;
    const nav = &world.nav;

    const place = try somePlace(queries, pass_all, world, random);
    _ = try boundaries.update(queries, pass_all, place.ref, place.pos, 6.0);
    try testing.expect(try boundaries.expectSameValidity(queries, pass_all));

    // The header goes with the tile: its place in the grid is read first.
    const parts = nav.decodePolyId(place.ref);
    const header = nav.getTile(@intCast(parts.tile_index)).header.?;
    const x = header.x;
    const y = header.y;
    const layer = header.layer;
    const tile_ref: TileRef = nav.encodePolyId(parts.salt, parts.tile_index, 0);
    try testing.expectEqual(tile_ref, nav.getTileRefAt(x, y, layer));
    try testing.expectEqual(
        reference.ref_nav_mesh_get_tile_ref_at(world.reference_nav, x, y, layer),
        tile_ref,
    );

    const expected = reference.ref_nav_mesh_remove_tile(world.reference_nav, tile_ref, null, null);
    const actual = nav.removeTile(tile_ref, null);
    try support.expectStatus(expected, actual);
    try testing.expect(actual.succeeded);

    // The boundary's start polygon went with the tile.
    try testing.expect(!try boundaries.expectSameValidity(queries, pass_all));

    // Gathering on the stale reference finds nothing; gathering around the
    // hole finds the same on both sides.
    var fresh: Boundaries = try .init();
    defer fresh.deinit();
    _ = try fresh.update(queries, pass_all, place.ref, place.pos, 6.0);
    try support.expectPoint(place.pos, fresh.port.getCenter());
    try testing.expectEqual(@as(i32, 0), fresh.port.poly_count);
    try testing.expect(!try fresh.expectSameValidity(queries, pass_all));
    for (0..6) |_| {
        var point = place.pos;
        point[0] += (random.float(f32) * 2.0 - 1.0) * 12.0;
        point[2] += (random.float(f32) * 2.0 - 1.0) * 12.0;
        const near = nearestPlace(queries, pass_all, point);
        _ = try fresh.update(queries, pass_all, near.ref, near.pos, 12.0);
        _ = try fresh.expectSameValidity(queries, pass_all);
    }

    if (reuse) {
        _ = try world.addTileReusing(x, y, tile_ref);
    } else {
        _ = try world.addTile(x, y);
    }
    const returned_ref = nav.getTileRefAt(x, y, layer);
    try testing.expectEqual(
        reference.ref_nav_mesh_get_tile_ref_at(world.reference_nav, x, y, layer),
        returned_ref,
    );
    try testing.expect(returned_ref != 0);
    try testing.expectEqual(reuse, returned_ref == tile_ref);
    try testing.expectEqual(reuse, try boundaries.expectSameValidity(queries, pass_all));

    // A boundary gathered now is valid either way.
    const again = nearestPlace(queries, pass_all, place.pos);
    try testing.expect(again.ref != 0);
    _ = try fresh.update(queries, pass_all, again.ref, again.pos, 6.0);
    try testing.expect(try fresh.expectSameValidity(queries, pass_all));
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

test "empty and reset: a boundary that holds nothing" {
    const gpa = testing.allocator;
    var fixture: Fixture = undefined;
    try fixture.init(gpa, world_cases[2]);
    defer fixture.deinit();
    const queries = &fixture.queries;
    const pass_all = &fixture.pass_all;
    const boundaries = &fixture.boundaries;
    const unset: [3]f32 = @splat(std.math.floatMax(f32));

    // As made: no centre, no segments, no polygons.
    try boundaries.expectSame();
    try support.expectPoint(unset, boundaries.port.getCenter());
    try testing.expectEqual(@as(i32, 0), boundaries.port.getSegmentCount());
    try testing.expectEqual(@as(i32, 0), boundaries.port.poly_count);
    try testing.expect(!try boundaries.expectSameValidity(queries, pass_all));

    // Resetting what holds nothing changes nothing.
    try boundaries.reset();
    try support.expectPoint(unset, boundaries.port.getCenter());
    try testing.expect(!try boundaries.expectSameValidity(queries, pass_all));

    var prng: std.Random.DefaultPrng = .init(41);
    const random = prng.random();
    var filled: usize = 0;
    for (0..32) |round| {
        errdefer std.debug.print("  round {d}\n", .{round});
        const place = try somePlace(queries, pass_all, &fixture.world, random);
        const range: f32 = if (round % 2 == 0) 6.0 else 1.0e6;
        _ = try boundaries.update(queries, pass_all, place.ref, place.pos, range);
        try support.expectPoint(place.pos, boundaries.port.getCenter());
        try testing.expect(try boundaries.expectSameValidity(queries, pass_all));
        if (boundaries.port.getSegmentCount() > 0) filled += 1;

        try boundaries.reset();
        try support.expectPoint(unset, boundaries.port.getCenter());
        try testing.expectEqual(@as(i32, 0), boundaries.port.getSegmentCount());
        try testing.expect(!try boundaries.expectSameValidity(queries, pass_all));
        // And once more on the boundary just reset.
        try boundaries.reset();
    }
    // Some of the boundaries that were reset held segments.
    try testing.expect(filled > 0);
}

test "update: a zero reference resets both sides" {
    const gpa = testing.allocator;
    var fixture: Fixture = undefined;
    try fixture.init(gpa, world_cases[2]);
    defer fixture.deinit();
    const queries = &fixture.queries;
    const pass_all = &fixture.pass_all;
    const boundaries = &fixture.boundaries;
    const unset: [3]f32 = @splat(std.math.floatMax(f32));
    const nan = std.math.nan(f32);

    var prng: std.Random.DefaultPrng = .init(42);
    const random = prng.random();
    for (0..40) |round| {
        errdefer std.debug.print("  round {d}\n", .{round});
        const place = try somePlace(queries, pass_all, &fixture.world, random);
        // Every other round the boundary holds polygons and segments first.
        if (round % 2 == 0) {
            _ = try boundaries.update(queries, pass_all, place.ref, place.pos, 8.0);
            try testing.expect(try boundaries.expectSameValidity(queries, pass_all));
        }
        // Neither side reads the position or the range of a zero reference.
        const pos: [3]f32 = switch (round % 4) {
            0 => place.pos,
            1 => .{ 0.0, 0.0, 0.0 },
            2 => @splat(nan),
            else => .{ std.math.inf(f32), place.pos[1], -std.math.inf(f32) },
        };
        const range: f32 = switch (round % 5) {
            0 => 6.0,
            1 => 0.0,
            2 => -3.0,
            3 => nan,
            else => std.math.floatMax(f32),
        };
        _ = try boundaries.update(queries, pass_all, 0, pos, range);
        try support.expectPoint(unset, boundaries.port.getCenter());
        try testing.expectEqual(@as(i32, 0), boundaries.port.getSegmentCount());
        try testing.expectEqual(@as(i32, 0), boundaries.port.poly_count);
        try testing.expect(!try boundaries.expectSameValidity(queries, pass_all));
    }
}

test "update: a reference, a range or a position the neighbourhood query refuses" {
    const gpa = testing.allocator;
    var fixture: Fixture = undefined;
    try fixture.init(gpa, world_cases[2]);
    defer fixture.deinit();
    const queries = &fixture.queries;
    const pass_all = &fixture.pass_all;
    const boundaries = &fixture.boundaries;
    const nav = &fixture.world.nav;
    const nan = std.math.nan(f32);
    const inf = std.math.inf(f32);

    var prng: std.Random.DefaultPrng = .init(43);
    const random = prng.random();
    for (0..64) |round| {
        errdefer std.debug.print("  round {d}\n", .{round});
        const place = try somePlace(queries, pass_all, &fixture.world, random);
        // The boundary holds polygons and segments first, so a refused
        // update is seen to drop them.
        _ = try boundaries.update(queries, pass_all, place.ref, place.pos, 8.0);
        try testing.expect(try boundaries.expectSameValidity(queries, pass_all));

        var ref = place.ref;
        var pos = place.pos;
        var range: f32 = 8.0;
        // A fast-math build checks no float for being finite: there a range
        // that is not finite is taken on both sides, and a position that is
        // not finite stays out of the test, as the original can write past
        // its segments with one.
        var refused = true;
        switch (round % 8) {
            0 => ref = staleRef(nav, place.ref),
            1 => ref = sentinel_ref,
            2 => range = -1.0e-6,
            3 => range = -std.math.floatMax(f32),
            4 => range = -inf,
            5 => {
                range = nan;
                refused = non_finite_checked;
            },
            6 => {
                range = inf;
                refused = non_finite_checked;
            },
            else => {
                if (!non_finite_checked) continue;
                pos[random.uintLessThan(usize, 3)] = if (random.boolean()) nan else -inf;
            },
        }
        _ = try boundaries.update(queries, pass_all, ref, pos, range);

        // The centre moves to the position even when nothing is gathered.
        try support.expectPoint(pos, boundaries.port.getCenter());
        const valid = try boundaries.expectSameValidity(queries, pass_all);
        if (refused) {
            try testing.expectEqual(@as(i32, 0), boundaries.port.poly_count);
            try testing.expectEqual(@as(i32, 0), boundaries.port.getSegmentCount());
            try testing.expect(!valid);
        }
    }
}

test "update: ranges from zero to the largest float" {
    const gpa = testing.allocator;
    var fixture: Fixture = undefined;
    try fixture.init(gpa, world_cases[0]);
    defer fixture.deinit();
    const queries = &fixture.queries;
    const pass_all = &fixture.pass_all;
    const boundaries = &fixture.boundaries;

    // The largest float's square root lies between the two values around
    // 1.8446744e19: the squared range is finite up to it, infinite past it.
    const ranges = [_]f32{
        0.0,          negative_zero,          std.math.floatTrueMin(f32),
        1.0e-30,      1.0e-6,                 1.0e-3,
        0.05,         0.3,                    0.6,
        1.0,          2.0,                    3.5,
        6.0,          12.0,                   40.0,
        1.0e3,        1.0e6,                  1.0e19,
        1.8446743e19, 1.8446745e19,           2.0e19,
        1.0e30,       std.math.floatMax(f32),
    };

    var prng: std.Random.DefaultPrng = .init(44);
    const random = prng.random();
    var none_kept: usize = 0;
    var capped: usize = 0;
    for (0..40) |round| {
        errdefer std.debug.print("  round {d}\n", .{round});
        const place = try somePlace(queries, pass_all, &fixture.world, random);
        for (ranges) |range| {
            const in_range = try boundaries.update(queries, pass_all, place.ref, place.pos, range);
            try support.expectPoint(place.pos, boundaries.port.getCenter());
            // The start polygon is gathered whatever the range.
            try testing.expect(boundaries.port.poly_count >= 1);
            try testing.expectEqual(place.ref, boundaries.port.polys[0]);
            try testing.expect(try boundaries.expectSameValidity(queries, pass_all));
            if (in_range == 0) none_kept += 1;
            if (in_range > LocalBoundary.segment_count_max) capped += 1;
        }
    }
    // Both ends were met: no wall in range, and more than a boundary keeps.
    try testing.expect(none_kept > 0);
    try testing.expect(capped > 0);
}

test "update: more walls in range than a boundary keeps" {
    const gpa = testing.allocator;
    var fixture: Fixture = undefined;
    try fixture.init(gpa, world_cases[1]);
    defer fixture.deinit();
    const queries = &fixture.queries;
    const pass_all = &fixture.pass_all;
    const boundaries = &fixture.boundaries;

    var filters: Filters = try .init();
    defer filters.deinit();

    var prng: std.Random.DefaultPrng = .init(45);
    const random = prng.random();
    var capped: usize = 0;
    for (0..240) |round| {
        errdefer std.debug.print("  round {d}\n", .{round});
        const place = try somePlace(queries, pass_all, &fixture.world, random);
        // Shut-out polygons turn their neighbours' shared edges into walls.
        switch (round % 4) {
            0 => filters.setFlags(0xffff, 0),
            1 => filters.setFlags(0xffff, 2),
            2 => filters.setFlags(0xffff, 4 | 8),
            else => filters.setFlags(0xffff, 2 | 4 | 8),
        }
        const range: f32 = switch (round % 3) {
            0 => 12.0,
            1 => 30.0,
            else => 1.0e6,
        };
        const in_range = try boundaries.update(queries, &filters, place.ref, place.pos, range);
        _ = try boundaries.expectSameValidity(queries, &filters);

        // `update` has compared which segments were kept, in which order,
        // and that their distances are the smallest in range. Here: the
        // count is capped and the order is nearest first.
        const port = &boundaries.port;
        const kept: usize = @intCast(port.getSegmentCount());
        try testing.expectEqual(@min(in_range, LocalBoundary.segment_count_max), kept);
        var i: usize = 1;
        while (i < kept) : (i += 1) {
            try testing.expect(port.segments[i - 1].d <= port.segments[i].d);
        }
        if (in_range > LocalBoundary.segment_count_max) capped += 1;
    }
    // Some rounds had more walls in range than the eight a boundary keeps.
    try testing.expect(capped > 0);
}

test "update: walls at exactly equal distances" {
    const gpa = testing.allocator;
    var fixture: Fixture = undefined;
    try fixture.init(gpa, world_cases[3]);
    defer fixture.deinit();
    const queries = &fixture.queries;
    const pass_all = &fixture.pass_all;
    const boundaries = &fixture.boundaries;

    const walls = try collectWalls(gpa, &fixture.world, queries, pass_all);
    defer gpa.free(walls);
    const corners = try collectCorners(gpa, walls);
    defer gpa.free(corners);
    try testing.expect(corners.len > 0);

    // At such a corner two walls of the start polygon lie at distance zero:
    // in every range, tied, and ahead of every other wall. Which of the two
    // comes first is the original's order of insertion, compared segment by
    // segment in `update`.
    const ranges = [_]f32{ 0.0, negative_zero, 1.0e-3, 4.0, 12.0, 1.0e6 };
    const stride = @max(1, corners.len / 300);
    var index: usize = 0;
    while (index < corners.len) : (index += stride) {
        errdefer std.debug.print("  corner {d} of {d}\n", .{ index, corners.len });
        const corner = corners[index];
        for (ranges) |range| {
            const in_range = try boundaries.update(
                queries,
                pass_all,
                corner.ref,
                corner.pos,
                range,
            );
            try testing.expect(in_range >= 2);
            try testing.expectEqual(@as(f32, 0.0), boundaries.port.segments[0].d);
            try testing.expectEqual(@as(f32, 0.0), boundaries.port.segments[1].d);
            try testing.expect(tieCount(&boundaries.port) >= 1);
            try testing.expect(try boundaries.expectSameValidity(queries, pass_all));
        }
    }
}

test "update: a wall exactly as far away as the range" {
    const gpa = testing.allocator;
    var fixture: Fixture = undefined;
    try fixture.init(gpa, world_cases[2]);
    defer fixture.deinit();
    const queries = &fixture.queries;
    const pass_all = &fixture.pass_all;
    const boundaries = &fixture.boundaries;

    var prng: std.Random.DefaultPrng = .init(46);
    const random = prng.random();
    var exact: usize = 0;
    for (0..200) |round| {
        errdefer std.debug.print("  round {d}\n", .{round});
        const place = try somePlace(queries, pass_all, &fixture.world, random);
        var segments: [segments_per_poly_max][6]f32 = undefined;
        var segment_count: i32 = 0;
        _ = reference.ref_query_get_poly_wall_segments(
            queries.original,
            place.ref,
            pass_all.original,
            @ptrCast(&segments),
            null,
            &segment_count,
            segments_per_poly_max,
        );
        for (segments[0..@intCast(segment_count)]) |*segment| {
            var t: f32 = 0;
            const distance = reference.ref_distance_pt_seg_sqr_2d(
                &place.pos,
                segment[0..3],
                segment[3..6],
                &t,
            );
            // Only a range whose square is the distance to the bit puts the
            // wall exactly on the limit.
            const range = @sqrt(distance);
            if (distance == 0.0 or range * range != distance) continue;
            exact += 1;

            // On the limit is in range: no wall in range is farther, so the
            // last one kept is this far away unless the cap cut it off.
            const in_range = try boundaries.update(
                queries,
                pass_all,
                place.ref,
                place.pos,
                range,
            );
            try testing.expect(in_range >= 1);
            if (in_range <= LocalBoundary.segment_count_max) {
                try support.expectFloat(distance, boundaries.port.segments[in_range - 1].d);
            }

            // And the ranges one step shorter and one step longer.
            const bits: u32 = @bitCast(range);
            const shorter: f32 = @bitCast(bits - 1);
            const longer: f32 = @bitCast(bits + 1);
            _ = try boundaries.update(queries, pass_all, place.ref, place.pos, shorter);
            _ = try boundaries.update(queries, pass_all, place.ref, place.pos, longer);
        }
    }
    // Some walls had a distance that is the square of a float.
    try testing.expect(exact > 0);
}

test "update: an off-mesh connection as the start polygon" {
    const gpa = testing.allocator;
    var fixture: Fixture = undefined;
    try fixture.init(gpa, world_cases[0]);
    defer fixture.deinit();
    const queries = &fixture.queries;
    const boundaries = &fixture.boundaries;

    const connections = try collectConnections(gpa, &fixture.world.nav);
    defer gpa.free(connections);
    try testing.expect(connections.len > 0);

    var filters: Filters = try .init();
    defer filters.deinit();

    // A connection is a polygon of two corners: it gives a wall from each
    // end to the other, and the ground polygons at its ends are its
    // neighbours.
    const ranges = [_]f32{ 0.0, 3.0, 12.0, 1.0e6 };
    const excludes = [_]u16{ 0, 2, 4 | 8, 2 | 4 | 8 };
    for (connections, 0..) |connection, connection_index| {
        errdefer std.debug.print("  connection {d}\n", .{connection_index});
        const a = connection.ends[0];
        const b = connection.ends[1];
        const middle: [3]f32 = .{
            (a[0] + b[0]) * 0.5,
            (a[1] + b[1]) * 0.5,
            (a[2] + b[2]) * 0.5,
        };
        for ([_][3]f32{ a, b, middle }) |pos| {
            for (ranges) |range| {
                for (excludes) |exclude| {
                    filters.setFlags(0xffff, exclude);
                    _ = try boundaries.update(queries, &filters, connection.ref, pos, range);
                    try support.expectPoint(pos, boundaries.port.getCenter());
                    try testing.expect(boundaries.port.poly_count >= 1);
                    try testing.expectEqual(connection.ref, boundaries.port.polys[0]);
                    _ = try boundaries.expectSameValidity(queries, &filters);
                }
            }
        }
    }
}

test "isValid: other filters, and polygon flags changed on both sides" {
    const gpa = testing.allocator;
    var fixture: Fixture = undefined;
    try fixture.init(gpa, world_cases[3]);
    defer fixture.deinit();
    const world = &fixture.world;
    const queries = &fixture.queries;
    const pass_all = &fixture.pass_all;
    const boundaries = &fixture.boundaries;

    var filters: Filters = try .init();
    defer filters.deinit();

    var prng: std.Random.DefaultPrng = .init(47);
    const random = prng.random();
    for (0..96) |round| {
        errdefer std.debug.print("  round {d}\n", .{round});
        const place = try somePlace(queries, pass_all, world, random);
        const range: f32 = 2.0 + random.float(f32) * 10.0;
        _ = try boundaries.update(queries, pass_all, place.ref, place.pos, range);
        try testing.expect(try boundaries.expectSameValidity(queries, pass_all));

        // Under other filters than the one the boundary was gathered with.
        for (0..4) |_| {
            filters.randomize(random);
            _ = try boundaries.expectSameValidity(queries, &filters);
        }
        // A filter no polygon passes.
        filters.setFlags(0x8000, 0);
        try testing.expect(!try boundaries.expectSameValidity(queries, &filters));
        filters.setFlags(0xffff, 0xffff);
        try testing.expect(!try boundaries.expectSameValidity(queries, &filters));

        // One gathered polygon loses its flags on both sides: no filter
        // passes it, so the boundary is stale under every filter.
        const count: usize = @intCast(boundaries.port.poly_count);
        const pick = boundaries.port.polys[random.uintLessThan(usize, count)];
        const flags = try polyFlags(world, pick);
        try setPolyFlags(world, pick, 0);
        try testing.expect(!try boundaries.expectSameValidity(queries, pass_all));

        // With a flag of its own it passes again, except where a filter
        // shuts that flag out.
        try setPolyFlags(world, pick, 0x10);
        try testing.expect(try boundaries.expectSameValidity(queries, pass_all));
        filters.setFlags(0xffff, 0x10);
        try testing.expect(!try boundaries.expectSameValidity(queries, &filters));
        filters.setFlags(0x10, 0);
        _ = try boundaries.expectSameValidity(queries, &filters);

        // Gathered again with the flag shut out, the polygon's shared edges
        // are walls; the start polygon is gathered whatever its flags.
        filters.setFlags(0xffff, 0x10);
        _ = try boundaries.update(queries, &filters, place.ref, place.pos, range);
        try testing.expectEqual(
            pick != place.ref,
            try boundaries.expectSameValidity(queries, &filters),
        );

        // The old flags back, the same boundary is whole under that filter.
        try setPolyFlags(world, pick, flags);
        try testing.expectEqual(flags, try polyFlags(world, pick));
        try testing.expect(try boundaries.expectSameValidity(queries, &filters));
        try testing.expect(try boundaries.expectSameValidity(queries, pass_all));
    }
}

test "isValid: a tile removed from both meshes, then put back" {
    const gpa = testing.allocator;
    // A tiled world, where the other tiles stay, and a world of one tile,
    // where nothing is left.
    for ([_]usize{ 1, 2 }) |world_index| {
        const world_case = world_cases[world_index];
        errdefer std.debug.print("  world {d}: {s}\n", .{ world_index, world_case.name });
        var fixture: Fixture = undefined;
        try fixture.init(gpa, world_case);
        defer fixture.deinit();

        var prng: std.Random.DefaultPrng = .init(48 + world_index);
        const random = prng.random();
        for (0..4) |round| {
            errdefer std.debug.print("  round {d}\n", .{round});
            // Back under its old reference on even rounds, a new one on odd.
            try expectTileRemovalSeen(&fixture, random, round % 2 == 0);
        }
    }
}

test "fuzz: updates, validity, flag changes and tile removals on every world" {
    const gpa = testing.allocator;
    for (world_cases, 0..) |world_case, world_index| {
        errdefer std.debug.print("  world {d}: {s}\n", .{ world_index, world_case.name });
        const seed: u64 = 0x6c62_0000 + world_index;
        var fixture: Fixture = undefined;
        try fixture.init(gpa, world_case);
        defer fixture.deinit();
        const world = &fixture.world;
        const queries = &fixture.queries;
        const pass_all = &fixture.pass_all;
        const boundaries = &fixture.boundaries;

        var filters: Filters = try .init();
        defer filters.deinit();
        var other: Filters = try .init();
        defer other.deinit();
        var connections = try collectConnections(gpa, &world.nav);
        defer gpa.free(connections);

        var prng: std.Random.DefaultPrng = .init(seed);
        const random = prng.random();

        var removals: usize = 0;
        var kept_some: usize = 0;
        var capped: usize = 0;
        var valid: usize = 0;
        var invalid: usize = 0;
        for (0..fuzz_case_count) |case| {
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });

            // Three times a world, a tile leaves both meshes and comes back,
            // by turns under its old reference and under a new one.
            if (case % 400 == 200) {
                try expectTileRemovalSeen(&fixture, random, removals % 2 == 0);
                removals += 1;
                const renewed = try collectConnections(gpa, &world.nav);
                gpa.free(connections);
                connections = renewed;
            }

            filters.randomize(random);
            const place = try somePlace(queries, pass_all, world, random);
            var ref = place.ref;
            var pos = place.pos;
            switch (random.uintLessThan(u32, 20)) {
                0 => ref = 0,
                1 => ref = staleRef(&world.nav, place.ref),
                2 => ref = sentinel_ref,
                // Beside the polygon, and far from it.
                3, 4 => for (&pos) |*coordinate| {
                    coordinate.* += (random.float(f32) * 2.0 - 1.0) * 3.0;
                },
                5 => {
                    pos[0] += (random.float(f32) * 2.0 - 1.0) * 500.0;
                    pos[2] += (random.float(f32) * 2.0 - 1.0) * 500.0;
                },
                // On a corner of the polygon, where walls tie at zero.
                6, 7, 8 => pos = polyVertex(&world.nav, place.ref, random),
                // Anywhere about the level, whatever the polygon.
                9 => pos = world.level.groundPoint(random, 2.0),
                // On an off-mesh connection, at one of its ends.
                10 => if (connections.len > 0) {
                    const pick = random.uintLessThan(usize, connections.len);
                    ref = connections[pick].ref;
                    pos = connections[pick].ends[random.uintLessThan(usize, 2)];
                },
                11 => if (non_finite_checked) {
                    pos[random.uintLessThan(usize, 3)] = switch (random.uintLessThan(u32, 3)) {
                        0 => std.math.nan(f32),
                        1 => std.math.inf(f32),
                        else => -std.math.inf(f32),
                    };
                },
                else => {},
            }
            const range = randomRange(random);
            const in_range = try boundaries.update(queries, &filters, ref, pos, range);
            if (boundaries.port.getSegmentCount() > 0) kept_some += 1;
            if (in_range > LocalBoundary.segment_count_max) capped += 1;

            // Under the filter of the update, under another, and under the
            // one that passes every polygon.
            if (try boundaries.expectSameValidity(queries, &filters)) {
                valid += 1;
            } else {
                invalid += 1;
            }
            other.randomize(random);
            _ = try boundaries.expectSameValidity(queries, &other);
            _ = try boundaries.expectSameValidity(queries, pass_all);

            // Now and then one gathered polygon's flags change on both
            // sides: the boundary is asked again, then gathered again.
            const poly_count: usize = @intCast(boundaries.port.poly_count);
            if (poly_count > 0 and random.uintLessThan(u32, 6) == 0) {
                const pick = boundaries.port.polys[random.uintLessThan(usize, poly_count)];
                const flags = try polyFlags(world, pick);
                const swapped = flags ^ 0x000e;
                const changed: u16 = switch (random.uintLessThan(u32, 3)) {
                    0 => 0,
                    1 => 0x8000,
                    else => swapped,
                };
                try setPolyFlags(world, pick, changed);
                _ = try boundaries.expectSameValidity(queries, &filters);
                _ = try boundaries.expectSameValidity(queries, &other);
                _ = try boundaries.expectSameValidity(queries, pass_all);
                _ = try boundaries.update(queries, &filters, ref, pos, range);
                _ = try boundaries.expectSameValidity(queries, &filters);
                // Swapped flags stay half the time, for the cases to come;
                // the others go, so every polygon keeps flag bit 0.
                if (changed != swapped or random.boolean()) {
                    try setPolyFlags(world, pick, flags);
                    _ = try boundaries.expectSameValidity(queries, &filters);
                }
            }

            if (random.uintLessThan(u32, 16) == 0) {
                try boundaries.reset();
                try testing.expect(!try boundaries.expectSameValidity(queries, &filters));
            }
        }

        // The run met what it is there for: kept segments, more walls in
        // range than a boundary keeps, valid and stale boundaries.
        try testing.expectEqual(@as(usize, 3), removals);
        try testing.expect(kept_some > 0);
        try testing.expect(capped > 0);
        try testing.expect(valid > 0);
        try testing.expect(invalid > 0);
    }
}
