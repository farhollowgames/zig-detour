//! Differential tests of the navigation mesh: the same tiles added to and
//! removed from the port and the original must leave the same tile bytes
//! (so the same polygons, links and snapped connection ends), the same
//! lookups and the same references.

const std = @import("std");
const detour = @import("detour");
const support = @import("support.zig");
const reference = support.reference;

const testing = std.testing;

const NavMesh = detour.NavMesh;
const MeshTile = detour.MeshTile;
const Poly = detour.Poly;
const PolyRef = detour.PolyRef;
const TileRef = detour.TileRef;
const Status = detour.Status;
const TileFlags = detour.TileFlags;

// ---------------------------------------------------------------------------
// Comparing two meshes
// ---------------------------------------------------------------------------

/// The index of one of the original's tile entries; null for null.
fn referenceTileIndex(nav: *const reference.NavMesh, tile: ?*const reference.MeshTile) ?usize {
    const t = tile orelse return null;
    const first = reference.ref_nav_mesh_get_tile(nav, 0).?;
    return (@intFromPtr(t) - @intFromPtr(first)) / reference.ref_sizeof_mesh_tile();
}

/// The index of one of the port's tile entries; null for null.
fn tileIndex(nav: *const NavMesh, tile: ?*const MeshTile) ?usize {
    const t = tile orelse return null;
    return (@intFromPtr(t) - @intFromPtr(nav.tiles.ptr)) / @sizeOf(MeshTile);
}

/// Fails unless one of a tile's slices holds `count` records and starts
/// where the original's array starts in the blob.
fn expectSection(
    expected_base: usize,
    expected: anytype,
    base: usize,
    actual: anytype,
    count: i32,
) !void {
    try testing.expectEqual(@as(usize, @intCast(count)), actual.len);
    if (count == 0) return;
    try testing.expectEqual(
        @intFromPtr(expected.?) - expected_base,
        @intFromPtr(actual.ptr) - base,
    );
}

/// Fails unless the two tile entries hold the same tile.
fn expectSameTile(
    expected_nav: *const reference.NavMesh,
    expected: *const reference.MeshTile,
    nav: *const NavMesh,
    actual: *const MeshTile,
) !void {
    try testing.expectEqual(reference.ref_tile_salt(expected), actual.salt);
    try testing.expectEqual(reference.ref_tile_links_free_list(expected), actual.links_free_list);
    try testing.expectEqual(
        @as(u32, @bitCast(reference.ref_tile_flags(expected))),
        @as(u32, @bitCast(actual.flags)),
    );
    // The chain through `next`: a lookup bucket for a tile, the free list
    // for an unused entry.
    try testing.expectEqual(
        referenceTileIndex(expected_nav, reference.ref_tile_next(expected)),
        tileIndex(nav, actual.next),
    );

    const expected_header = reference.ref_tile_header(expected);
    try testing.expectEqual(expected_header == null, actual.header == null);
    const header = actual.header orelse {
        // An unused entry of the port holds nothing. The original's keeps
        // the pointer and size of a blob it handed back to the caller.
        try testing.expectEqual(@as(usize, 0), actual.data.len);
        return;
    };
    const size: usize = @intCast(reference.ref_tile_data_size(expected));
    try testing.expectEqual(size, actual.data.len);

    // The whole blob: header, vertices, polygons with their link heads, the
    // links as both sides allocated them, detail meshes, tree, connections.
    const expected_data = reference.ref_tile_data(expected).?;
    try support.expectBytes(expected_data[0..size], actual.data);

    const expected_base = @intFromPtr(expected_data);
    const base = @intFromPtr(actual.data.ptr);
    try testing.expectEqual(
        @intFromPtr(expected_header.?) - expected_base,
        @intFromPtr(header) - base,
    );
    try expectSection(
        expected_base,
        reference.ref_tile_verts(expected),
        base,
        actual.verts,
        header.vert_count,
    );
    try expectSection(
        expected_base,
        reference.ref_tile_polys(expected),
        base,
        actual.polys,
        header.poly_count,
    );
    try expectSection(
        expected_base,
        reference.ref_tile_links(expected),
        base,
        actual.links,
        header.link_count_max,
    );
    try expectSection(
        expected_base,
        reference.ref_tile_detail_meshes(expected),
        base,
        actual.detail_meshes,
        header.detail_mesh_count,
    );
    try expectSection(
        expected_base,
        reference.ref_tile_detail_verts(expected),
        base,
        actual.detail_verts,
        header.detail_vert_count,
    );
    try expectSection(
        expected_base,
        reference.ref_tile_detail_tris(expected),
        base,
        actual.detail_tris,
        header.detail_tri_count,
    );
    try expectSection(
        expected_base,
        reference.ref_tile_bv_tree(expected),
        base,
        actual.bv_tree,
        header.bv_node_count,
    );
    try expectSection(
        expected_base,
        reference.ref_tile_off_mesh_cons(expected),
        base,
        actual.off_mesh_connections,
        header.off_mesh_connection_count,
    );
}

/// Fails unless every tile entry of the two meshes holds the same tile.
fn expectSameMeshes(expected_nav: *const reference.NavMesh, nav: *const NavMesh) !void {
    const tile_count = nav.getMaxTiles();
    try testing.expectEqual(reference.ref_nav_mesh_get_max_tiles(expected_nav), tile_count);
    try support.expectBytes(
        std.mem.asBytes(reference.ref_nav_mesh_get_params(expected_nav)),
        std.mem.asBytes(nav.getParams()),
    );
    var i: i32 = 0;
    while (i < tile_count) : (i += 1) {
        const expected = reference.ref_nav_mesh_get_tile(expected_nav, i).?;
        expectSameTile(expected_nav, expected, nav, nav.getTile(i)) catch |err| {
            std.debug.print("  in tile entry {d} of {d}\n", .{ i, tile_count });
            return err;
        };
    }
}

/// How many tile entries of the port's mesh hold a tile.
fn tileCount(nav: *const NavMesh) usize {
    var count: usize = 0;
    for (nav.tiles) |*tile| count += @intFromBool(tile.header != null);
    return count;
}

/// Fails unless the two sides resolve `ref` to the same polygon, or both
/// refuse it the same way, in every function that takes a polygon
/// reference.
fn expectSamePolygon(
    expected_nav: *const reference.NavMesh,
    nav: *const NavMesh,
    ref: PolyRef,
) !void {
    const valid = reference.ref_nav_mesh_is_valid_poly_ref(expected_nav, ref);
    try testing.expectEqual(valid, nav.isValidPolyRef(ref));

    var expected_tile: ?*const reference.MeshTile = null;
    var expected_poly: ?*const reference.Poly = null;
    var actual_tile: *const MeshTile = undefined;
    var actual_poly: *const Poly = undefined;
    const status = nav.getTileAndPolyByRef(ref, &actual_tile, &actual_poly);
    try support.expectStatus(
        reference.ref_nav_mesh_get_tile_and_poly_by_ref(
            expected_nav,
            ref,
            &expected_tile,
            &expected_poly,
        ),
        status,
    );
    try testing.expectEqual(valid, status.succeeded);

    if (valid) {
        const expected_polys = reference.ref_tile_polys(expected_tile.?).?;
        const expected_index: u32 = @intCast(
            (@intFromPtr(expected_poly.?) - @intFromPtr(expected_polys)) / @sizeOf(reference.Poly),
        );
        try testing.expectEqual(
            referenceTileIndex(expected_nav, expected_tile),
            tileIndex(nav, actual_tile),
        );
        try testing.expectEqual(expected_index, actual_tile.polyIndex(actual_poly));

        var unsafe_tile: ?*const reference.MeshTile = null;
        var unsafe_poly: ?*const reference.Poly = null;
        reference.ref_nav_mesh_get_tile_and_poly_by_ref_unsafe(
            expected_nav,
            ref,
            &unsafe_tile,
            &unsafe_poly,
        );
        const unsafe = nav.getTileAndPolyByRefUnsafe(ref);
        try testing.expectEqual(expected_tile, unsafe_tile);
        try testing.expectEqual(expected_poly, unsafe_poly);
        try testing.expectEqual(actual_tile, unsafe.tile);
        try testing.expectEqual(actual_poly, unsafe.poly);
    }

    // The outputs start the same, so a call that leaves them alone on one
    // side must leave them alone on the other.
    var expected_flags: u16 = 0xbeef;
    var actual_flags: u16 = 0xbeef;
    try support.expectStatus(
        reference.ref_nav_mesh_get_poly_flags(expected_nav, ref, &expected_flags),
        nav.getPolyFlags(ref, &actual_flags),
    );
    try testing.expectEqual(expected_flags, actual_flags);

    var expected_area: u8 = 0xee;
    var actual_area: u8 = 0xee;
    try support.expectStatus(
        reference.ref_nav_mesh_get_poly_area(expected_nav, ref, &expected_area),
        nav.getPolyArea(ref, &actual_area),
    );
    try testing.expectEqual(expected_area, actual_area);

    const expected_connection =
        reference.ref_nav_mesh_get_off_mesh_connection_by_ref(expected_nav, ref);
    const actual_connection = nav.getOffMeshConnectionByRef(ref);
    try testing.expectEqual(expected_connection == null, actual_connection == null);
    if (actual_connection) |connection| {
        const expected_first = reference.ref_tile_off_mesh_cons(expected_tile.?).?;
        const expected_index = (@intFromPtr(expected_connection.?) - @intFromPtr(expected_first)) /
            @sizeOf(reference.OffMeshConnection);
        const actual_index = (@intFromPtr(connection) -
            @intFromPtr(actual_tile.off_mesh_connections.ptr)) /
            @sizeOf(detour.nav_mesh.OffMeshConnection);
        try testing.expectEqual(expected_index, actual_index);
    }
}

/// Fails unless the two sides give the same end points for the off-mesh
/// connection `ref` entered from `prev_ref`, or refuse the same way.
fn expectSameEndPoints(
    expected_nav: *const reference.NavMesh,
    nav: *const NavMesh,
    prev_ref: PolyRef,
    ref: PolyRef,
) !void {
    var expected_start: [3]f32 = .{ 11, 12, 13 };
    var expected_end: [3]f32 = .{ 14, 15, 16 };
    var actual_start = expected_start;
    var actual_end = expected_end;
    try support.expectStatus(
        reference.ref_nav_mesh_get_off_mesh_connection_poly_end_points(
            expected_nav,
            prev_ref,
            ref,
            &expected_start,
            &expected_end,
        ),
        nav.getOffMeshConnectionPolyEndPoints(prev_ref, ref, &actual_start, &actual_end),
    );
    try support.expectPoint(expected_start, actual_start);
    try support.expectPoint(expected_end, actual_end);
}

/// References that name nothing, or might: zero, stale salts, indices past
/// the end, and random bits.
fn oddReference(nav: *const NavMesh, random: std.Random) PolyRef {
    const tile_count: u32 = @intCast(nav.getMaxTiles());
    const tile_index = random.uintLessThan(u32, tile_count + 2);
    const tile: ?*const MeshTile = if (tile_index < tile_count) &nav.tiles[tile_index] else null;
    const salt: u32 = if (tile) |t| t.salt else 1;
    const poly_count: u32 = if (tile) |t| @intCast(t.polys.len) else 0;
    return switch (random.uintLessThan(u8, 6)) {
        0 => 0,
        1 => nav.encodePolyId(salt +% 1, tile_index, random.uintLessThan(u32, poly_count + 1)),
        2 => nav.encodePolyId(salt, tile_index, poly_count),
        3 => nav.encodePolyId(salt, tile_index, poly_count + random.uintLessThan(u32, 50)),
        4 => nav.encodePolyId(0, tile_index, random.uintLessThan(u32, poly_count + 1)),
        else => random.int(PolyRef),
    };
}

/// Compares every lookup and every per-polygon read of the two meshes.
fn expectSameLookups(
    level: *const support.Level,
    expected_nav: *const reference.NavMesh,
    nav: *const NavMesh,
    seed: u64,
) !void {
    errdefer std.debug.print("  lookups with seed {d}\n", .{seed});
    const grid = level.grid;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    // Tiles by grid position, inside the grid and around it.
    var y: i32 = -2;
    while (y < grid.height + 2) : (y += 1) {
        var x: i32 = -2;
        while (x < grid.width + 2) : (x += 1) {
            for ([_]i32{ 0, 1, 2, -1 }) |layer| {
                try testing.expectEqual(
                    referenceTileIndex(
                        expected_nav,
                        reference.ref_nav_mesh_get_tile_at(expected_nav, x, y, layer),
                    ),
                    tileIndex(nav, nav.getTileAt(x, y, layer)),
                );
                try testing.expectEqual(
                    reference.ref_nav_mesh_get_tile_ref_at(expected_nav, x, y, layer),
                    nav.getTileRefAt(x, y, layer),
                );
            }
            for ([_]usize{ 0, 1, 2, 8 }) |capacity| {
                var expected_tiles: [8]?*const reference.MeshTile = undefined;
                var actual_tiles: [8]*const MeshTile = undefined;
                const expected_count = reference.ref_nav_mesh_get_tiles_at(
                    expected_nav,
                    x,
                    y,
                    &expected_tiles,
                    @intCast(capacity),
                );
                const actual_count = nav.getTilesAt(x, y, actual_tiles[0..capacity]);
                try testing.expectEqual(@as(usize, @intCast(expected_count)), actual_count);
                for (expected_tiles[0..actual_count], actual_tiles[0..actual_count]) |e, a| {
                    try testing.expectEqual(referenceTileIndex(expected_nav, e), tileIndex(nav, a));
                }
            }
        }
    }

    // The grid cell of a position, on the level and far off it.
    for (0..2000) |_| {
        var pos = level.groundPoint(random, 40.0);
        if (random.uintLessThan(u8, 8) == 0) pos[0] *= 1000.0;
        if (random.uintLessThan(u8, 8) == 0) pos[2] *= -1000.0;
        var expected_x: i32 = undefined;
        var expected_y: i32 = undefined;
        reference.ref_nav_mesh_calc_tile_loc(expected_nav, &pos, &expected_x, &expected_y);
        const actual = nav.calcTileLoc(pos);
        try testing.expectEqual(expected_x, actual.x);
        try testing.expectEqual(expected_y, actual.y);
    }

    // Tile references, and every polygon of every tile.
    try testing.expectEqual(
        reference.ref_nav_mesh_get_tile_ref(expected_nav, null),
        nav.getTileRef(null),
    );
    try testing.expectEqual(
        reference.ref_nav_mesh_get_poly_ref_base(expected_nav, null),
        nav.getPolyRefBase(null),
    );
    var i: i32 = 0;
    while (i < nav.getMaxTiles()) : (i += 1) {
        const expected_tile = reference.ref_nav_mesh_get_tile(expected_nav, i).?;
        const tile = nav.getTile(i);
        const tile_ref = nav.getTileRef(tile);
        try testing.expectEqual(
            reference.ref_nav_mesh_get_tile_ref(expected_nav, expected_tile),
            tile_ref,
        );
        const base = nav.getPolyRefBase(tile);
        try testing.expectEqual(
            reference.ref_nav_mesh_get_poly_ref_base(expected_nav, expected_tile),
            base,
        );

        const tile_index: u32 = @intCast(i);
        const tile_refs = [_]TileRef{
            tile_ref,
            nav.encodePolyId(tile.salt +% 1, tile_index, 0),
            nav.encodePolyId(tile.salt, tile_index, 3),
            nav.encodePolyId(0, tile_index, 0),
        };
        for (tile_refs) |ref| {
            try testing.expectEqual(
                referenceTileIndex(
                    expected_nav,
                    reference.ref_nav_mesh_get_tile_by_ref(expected_nav, ref),
                ),
                tileIndex(nav, nav.getTileByRef(ref)),
            );
        }

        for (tile.polys, 0..) |*poly, poly_index| {
            const ref = base | @as(PolyRef, @intCast(poly_index));
            try expectSamePolygon(expected_nav, nav, ref);

            // An off-mesh connection walked from each polygon it links to,
            // from nowhere and from itself; a ground polygon refused.
            try expectSameEndPoints(expected_nav, nav, 0, ref);
            try expectSameEndPoints(expected_nav, nav, ref, ref);
            var link = poly.first_link;
            while (link != detour.nav_mesh.null_link) : (link = tile.links[link].next) {
                try expectSameEndPoints(expected_nav, nav, tile.links[link].ref, ref);
            }
        }
    }
    try testing.expectEqual(
        referenceTileIndex(expected_nav, reference.ref_nav_mesh_get_tile_by_ref(expected_nav, 0)),
        tileIndex(nav, nav.getTileByRef(0)),
    );

    for (0..2000) |_| {
        const ref = oddReference(nav, random);
        try expectSamePolygon(expected_nav, nav, ref);
        try expectSameEndPoints(expected_nav, nav, oddReference(nav, random), ref);
        try testing.expectEqual(
            referenceTileIndex(
                expected_nav,
                reference.ref_nav_mesh_get_tile_by_ref(expected_nav, ref),
            ),
            tileIndex(nav, nav.getTileByRef(ref)),
        );
    }
}

/// A random reference to a polygon the port's mesh holds.
fn randomPolygon(nav: *const NavMesh, random: std.Random) PolyRef {
    while (true) {
        const tile = &nav.tiles[random.uintLessThan(usize, nav.tiles.len)];
        if (tile.polys.len == 0) continue;
        const poly_index = random.uintLessThan(usize, tile.polys.len);
        return nav.getPolyRefBase(tile) | @as(PolyRef, @intCast(poly_index));
    }
}

const WorldCase = struct {
    name: []const u8,
    obj_text: []const u8,
    config: support.RecastConfig,
    options: support.LevelOptions,
};

const world_cases = [_]WorldCase{
    .{
        .name = "dungeon, one tile",
        .obj_text = support.dungeon_obj,
        .config = support.solo_config,
        .options = .{ .off_mesh_connection_count = 8, .vary_polygons = true, .seed = 11 },
    },
    .{
        .name = "nav_test, tiled",
        .obj_text = support.nav_test_obj,
        .config = support.tiled_config,
        .options = .{ .off_mesh_connection_count = 24, .vary_polygons = true, .seed = 12 },
    },
    .{
        .name = "dungeon, tiled, no tree",
        .obj_text = support.dungeon_obj,
        .config = support.tiled_config,
        .options = .{ .off_mesh_connection_count = 16, .build_bv_tree = false, .seed = 13 },
    },
    .{
        .name = "undulating, one tile",
        .obj_text = support.undulating_obj,
        .config = support.solo_config,
        .options = .{ .seed = 14 },
    },
};

// ---------------------------------------------------------------------------
// Meshes filled by hand
// ---------------------------------------------------------------------------

/// The blob of every tile of a level that has ground, as the original's
/// builder writes it, in grid order.
const TileBlobs = struct {
    gpa: std.mem.Allocator,
    items: std.ArrayList([]u8),

    fn init(gpa: std.mem.Allocator, level: *const support.Level) !TileBlobs {
        var self: TileBlobs = .{ .gpa = gpa, .items = .empty };
        errdefer self.deinit();
        var y: i32 = 0;
        while (y < level.grid.height) : (y += 1) {
            var x: i32 = 0;
            while (x < level.grid.width) : (x += 1) {
                var input = (try level.tileInput(x, y)) orelse continue;
                defer input.deinit();
                const bytes = input.referenceBytes() orelse continue;
                defer reference.ref_free(bytes.ptr);
                const copy = try gpa.dupe(u8, bytes);
                errdefer gpa.free(copy);
                try self.items.append(gpa, copy);
            }
        }
        return self;
    }

    fn deinit(self: *TileBlobs) void {
        for (self.items.items) |blob| self.gpa.free(blob);
        self.items.deinit(self.gpa);
        self.* = undefined;
    }
};

/// A mesh on each side with the same parameters, for tests that add tile
/// blobs by hand.
const Pair = struct {
    gpa: std.mem.Allocator,
    nav: NavMesh,
    reference_nav: *reference.NavMesh,
    /// The copies neither mesh owns: blobs added without `free_data` and
    /// blobs a mesh refused. Freed by `deinit`, after the meshes.
    kept: std.ArrayList([]align(4) u8),
    reference_kept: std.ArrayList([*]u8),

    /// Readies both meshes. Returns false when both sides refuse `params`;
    /// `deinit` is due either way.
    fn init(self: *Pair, gpa: std.mem.Allocator, params: reference.NavMeshParams) !bool {
        self.* = .{
            .gpa = gpa,
            .nav = .empty,
            .reference_nav = reference.ref_nav_mesh_alloc() orelse return error.OutOfMemory,
            .kept = .empty,
            .reference_kept = .empty,
        };
        errdefer self.deinit();
        const expected = reference.ref_nav_mesh_init(self.reference_nav, &params);
        const actual = self.nav.init(gpa, &.{
            .origin = params.orig,
            .tile_width = params.tile_width,
            .tile_height = params.tile_height,
            .tile_count_max = params.max_tiles,
            .poly_count_max = params.max_polys,
        });
        try support.expectStatus(expected, actual);
        return actual.succeeded;
    }

    fn deinit(self: *Pair) void {
        self.nav.deinit();
        reference.ref_nav_mesh_free(self.reference_nav);
        for (self.kept.items) |blob| self.gpa.free(blob);
        for (self.reference_kept.items) |blob| reference.ref_free(blob);
        self.kept.deinit(self.gpa);
        self.reference_kept.deinit(self.gpa);
        self.* = undefined;
    }

    /// Adds a copy of `bytes` to both meshes and returns the status both
    /// gave. `result` receives the new tile's reference, the same on both
    /// sides, and is left alone when the tile is refused.
    fn add(
        self: *Pair,
        bytes: []const u8,
        flags: TileFlags,
        last_ref: TileRef,
        result: ?*TileRef,
    ) !Status {
        const gpa = self.gpa;
        const copy = try gpa.alignedAlloc(u8, .@"4", bytes.len);
        @memcpy(copy, bytes);
        const reference_copy: [*]u8 = @ptrCast(reference.ref_alloc(bytes.len, .perm) orelse {
            gpa.free(copy);
            return error.OutOfMemory;
        });
        @memcpy(reference_copy[0..bytes.len], bytes);

        var expected_ref: TileRef = 0xdead;
        var actual_ref: TileRef = 0xdead;
        const expected = reference.ref_nav_mesh_add_tile(
            self.reference_nav,
            reference_copy,
            @intCast(bytes.len),
            @bitCast(flags),
            last_ref,
            &expected_ref,
        );
        const actual = self.nav.addTile(copy, flags, last_ref, &actual_ref);
        if (!(flags.free_data and actual.succeeded)) try self.kept.append(gpa, copy);
        if (!(flags.free_data and reference.ref_status_succeeded(expected))) {
            try self.reference_kept.append(gpa, reference_copy);
        }

        try support.expectStatus(expected, actual);
        try testing.expectEqual(expected_ref, actual_ref);
        if (result) |r| {
            if (actual.succeeded) r.* = actual_ref;
        }
        return actual;
    }
};

/// Removes the tile `ref` names from both meshes and returns the status
/// both gave. With `blob`, asks both for the tile's bytes and compares what
/// comes back: the bytes of a blob the caller owns, nothing for one the mesh
/// freed, the outputs untouched when the reference is refused. On success
/// `blob` receives what the port handed back.
fn expectSameRemoval(
    expected_nav: *reference.NavMesh,
    nav: *NavMesh,
    ref: TileRef,
    blob: ?*[]align(4) u8,
) !Status {
    var untouched: [4]u8 align(4) = .{ 1, 2, 3, 4 };
    var expected_data: ?[*]u8 = &untouched;
    var expected_size: i32 = -7;
    var actual_data: []align(4) u8 = &untouched;
    const take = blob != null;

    const expected = reference.ref_nav_mesh_remove_tile(
        expected_nav,
        ref,
        if (take) &expected_data else null,
        if (take) &expected_size else null,
    );
    const actual = nav.removeTile(ref, if (take) &actual_data else null);
    try support.expectStatus(expected, actual);
    const result = blob orelse return actual;

    if (actual.failed) {
        try testing.expectEqual(@intFromPtr(&untouched), @intFromPtr(expected_data));
        try testing.expectEqual(@as(i32, -7), expected_size);
        try testing.expectEqual(@intFromPtr(&untouched), @intFromPtr(actual_data.ptr));
        try testing.expectEqual(untouched.len, actual_data.len);
    } else if (expected_data) |bytes| {
        try support.expectBytes(bytes[0..@intCast(expected_size)], actual_data);
        result.* = actual_data;
    } else {
        try testing.expectEqual(@as(i32, 0), expected_size);
        try testing.expectEqual(@as(usize, 0), actual_data.len);
        result.* = actual_data;
    }
    return actual;
}

/// A mesh shape with a unit grid at the origin.
fn shapeParams(tile_count_max: i32, poly_count_max: i32) reference.NavMeshParams {
    return .{
        .orig = .{ 0, 0, 0 },
        .tile_width = 10,
        .tile_height = 10,
        .max_tiles = tile_count_max,
        .max_polys = poly_count_max,
    };
}

/// Fails unless the two sides give the same closest point and the same
/// height for `pos` on a polygon. Returns whether `pos` is over the polygon.
fn expectSameClosestPoint(
    query: *const reference.NavMeshQuery,
    nav: *const NavMesh,
    tile: *const MeshTile,
    poly: *const Poly,
    ref: PolyRef,
    pos: [3]f32,
) !bool {
    var expected_closest: [3]f32 = .{ 21, 22, 23 };
    var expected_over = false;
    const closest_status = reference.ref_query_closest_point_on_poly(
        query,
        ref,
        &pos,
        &expected_closest,
        &expected_over,
    );
    try testing.expect(reference.ref_status_succeeded(closest_status));
    const actual = nav.closestPointOnPoly(ref, pos);
    try support.expectPoint(expected_closest, actual.closest);
    try testing.expectEqual(expected_over, actual.pos_over_poly);

    const height = tile.getPolyHeight(poly, pos);
    // The original's query gives an off-mesh connection a height of its
    // own; the mesh gives it none.
    if (poly.getType() == .off_mesh_connection) {
        try testing.expect(height == null);
        try testing.expect(!actual.pos_over_poly);
        return false;
    }
    var expected_height: f32 = 31;
    const height_status = reference.ref_query_get_poly_height(query, ref, &pos, &expected_height);
    try testing.expectEqual(reference.ref_status_succeeded(height_status), height != null);
    if (height) |h| try support.expectFloat(expected_height, h);
    try testing.expectEqual(height != null, actual.pos_over_poly);
    return height != null;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

test "addTile: the same tiles, links and lookups on both sides" {
    const gpa = testing.allocator;
    for (world_cases, 0..) |case, case_index| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{case.name});
        var world: support.World = undefined;
        try world.init(gpa, case.obj_text, case.config, case.options);
        defer world.deinit();

        // The level must give the comparison something to compare.
        var poly_count: usize = 0;
        var border_link_count: usize = 0;
        var connection_count: usize = 0;
        for (world.nav.tiles) |*tile| {
            poly_count += tile.polys.len;
            connection_count += tile.off_mesh_connections.len;
            for (tile.polys) |*poly| {
                var link = poly.first_link;
                while (link != detour.nav_mesh.null_link) : (link = tile.links[link].next) {
                    border_link_count += @intFromBool(tile.links[link].side != 0xff);
                }
            }
        }
        try testing.expect(poly_count > 50);
        if (case.config.tile_size != 0) {
            try testing.expect(tileCount(&world.nav) > 4);
            try testing.expect(border_link_count > 8);
        }
        if (case.options.off_mesh_connection_count != 0) try testing.expect(connection_count > 0);

        try expectSameMeshes(world.reference_nav, &world.nav);
        try expectSameLookups(&world.level, world.reference_nav, &world.nav, 100 + case_index);
    }
}

test "addTile: a second layer in a grid cell" {
    const gpa = testing.allocator;
    var level: support.Level = try .init(gpa, support.nav_test_obj, support.tiled_config, .{
        .off_mesh_connection_count = 12,
        .seed = 51,
    });
    defer level.deinit();
    var blobs: TileBlobs = try .init(gpa, &level);
    defer blobs.deinit();

    var params = level.navMeshParams();
    params.max_tiles *= 2;
    params.max_polys = @divTrunc(params.max_polys, 2);
    var pair: Pair = undefined;
    const ready = try pair.init(gpa, params);
    defer pair.deinit();
    try testing.expect(ready);

    // Each tile twice: as built, and again as layer 1 of the same cell.
    const layer_offset = @offsetOf(detour.MeshHeader, "layer");
    for (blobs.items.items) |blob| {
        try testing.expect((try pair.add(blob, .{ .free_data = true }, 0, null)).succeeded);
        std.mem.writeInt(i32, blob[layer_offset..][0..4], 1, .native);
        try testing.expect((try pair.add(blob, .{ .free_data = true }, 0, null)).succeeded);
    }
    try expectSameMeshes(pair.reference_nav, &pair.nav);

    try expectSameLookups(&level, pair.reference_nav, &pair.nav, 52);
}

test "setPolyFlags and setPolyArea" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, support.nav_test_obj, support.tiled_config, .{
        .off_mesh_connection_count = 8,
        .seed = 61,
    });
    defer world.deinit();
    const nav = &world.nav;
    const expected_nav = world.reference_nav;
    var prng: std.Random.DefaultPrng = .init(62);
    const random = prng.random();

    for (0..4000) |i| {
        errdefer std.debug.print("  seed 62, case {d}\n", .{i});
        const ref = if (i % 4 == 0) oddReference(nav, random) else randomPolygon(nav, random);
        const flags = random.int(u16);
        // Areas past the six bits a polygon keeps, too.
        const area = random.int(u8);
        try support.expectStatus(
            reference.ref_nav_mesh_set_poly_flags(expected_nav, ref, flags),
            nav.setPolyFlags(ref, flags),
        );
        try support.expectStatus(
            reference.ref_nav_mesh_set_poly_area(expected_nav, ref, area),
            nav.setPolyArea(ref, area),
        );
        try expectSamePolygon(expected_nav, nav, ref);
    }
    try expectSameMeshes(expected_nav, nav);
}

test "storeTileState and restoreTileState" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, support.nav_test_obj, support.tiled_config, .{
        .vary_polygons = true,
        .seed = 71,
    });
    defer world.deinit();
    const nav = &world.nav;
    const expected_nav = world.reference_nav;
    var prng: std.Random.DefaultPrng = .init(72);
    const random = prng.random();

    try testing.expectEqual(
        @as(usize, @intCast(reference.ref_nav_mesh_get_tile_state_size(expected_nav, null))),
        NavMesh.getTileStateSize(null),
    );

    var previous: ?[]u8 = null;
    defer if (previous) |p| gpa.free(p);
    var i: i32 = 0;
    while (i < nav.getMaxTiles()) : (i += 1) {
        const tile = nav.getTile(i);
        if (tile.header == null) continue;
        const expected_tile = reference.ref_nav_mesh_get_tile(expected_nav, i).?;

        const size = NavMesh.getTileStateSize(tile);
        try testing.expectEqual(
            @as(usize, @intCast(
                reference.ref_nav_mesh_get_tile_state_size(expected_nav, expected_tile),
            )),
            size,
        );

        // Both buffers start the same: a state leaves one byte a polygon
        // unwritten.
        const expected_state = try gpa.alloc(u8, size);
        defer gpa.free(expected_state);
        const state = try gpa.alloc(u8, size);
        errdefer gpa.free(state);
        @memset(expected_state, 0xaa);
        @memset(state, 0xaa);

        // A buffer one byte short is refused and left alone.
        try support.expectStatus(
            reference.ref_nav_mesh_store_tile_state(
                expected_nav,
                expected_tile,
                expected_state.ptr,
                @intCast(size - 1),
            ),
            nav.storeTileState(tile, state[0 .. size - 1]),
        );
        try support.expectBytes(expected_state, state);

        try support.expectStatus(
            reference.ref_nav_mesh_store_tile_state(
                expected_nav,
                expected_tile,
                expected_state.ptr,
                @intCast(size),
            ),
            nav.storeTileState(tile, state),
        );
        try support.expectBytes(expected_state, state);

        // Change the tile's polygons, then bring the stored state back.
        const base = nav.getPolyRefBase(tile);
        for (0..tile.polys.len) |poly_index| {
            const ref = base | @as(PolyRef, @intCast(poly_index));
            const flags = random.int(u16);
            const area = random.uintLessThan(u8, 64);
            try testing.expect(nav.setPolyFlags(ref, flags).succeeded);
            try testing.expect(nav.setPolyArea(ref, area).succeeded);
            _ = reference.ref_nav_mesh_set_poly_flags(expected_nav, ref, flags);
            _ = reference.ref_nav_mesh_set_poly_area(expected_nav, ref, area);
        }

        const mutable_tile: *reference.MeshTile = @constCast(expected_tile);
        // What both sides refuse: a short buffer, another magic, another
        // version, and the state of another tile.
        try support.expectStatus(
            reference.ref_nav_mesh_restore_tile_state(
                expected_nav,
                mutable_tile,
                state.ptr,
                @intCast(size - 1),
            ),
            nav.restoreTileState(tile, state[0 .. size - 1]),
        );
        for ([_]usize{ 0, 4 }) |offset| {
            state[offset] ^= 0x55;
            try support.expectStatus(
                reference.ref_nav_mesh_restore_tile_state(
                    expected_nav,
                    mutable_tile,
                    state.ptr,
                    @intCast(size),
                ),
                nav.restoreTileState(tile, state),
            );
            state[offset] ^= 0x55;
        }
        if (previous) |other| {
            if (other.len >= size) {
                try support.expectStatus(
                    reference.ref_nav_mesh_restore_tile_state(
                        expected_nav,
                        mutable_tile,
                        other.ptr,
                        @intCast(other.len),
                    ),
                    nav.restoreTileState(tile, other),
                );
            }
        }
        try expectSameMeshes(expected_nav, nav);

        try support.expectStatus(
            reference.ref_nav_mesh_restore_tile_state(
                expected_nav,
                mutable_tile,
                state.ptr,
                @intCast(size),
            ),
            nav.restoreTileState(tile, state),
        );
        expectSameTile(expected_nav, expected_tile, nav, tile) catch |err| {
            std.debug.print("  after restoring tile entry {d}\n", .{i});
            return err;
        };

        if (previous) |p| gpa.free(p);
        previous = state;
    }
    try testing.expect(previous != null);
    try expectSameMeshes(expected_nav, nav);
}

test "removeTile, then addTile into the entry and salt a reference names" {
    const gpa = testing.allocator;
    const seed = 81;
    var level: support.Level = try .init(gpa, support.nav_test_obj, support.tiled_config, .{
        .off_mesh_connection_count = 16,
        .vary_polygons = true,
        .seed = seed,
    });
    defer level.deinit();
    var blobs: TileBlobs = try .init(gpa, &level);
    defer blobs.deinit();
    const count = blobs.items.items.len;
    try testing.expect(count > 8);

    var pair: Pair = undefined;
    const ready = try pair.init(gpa, level.navMeshParams());
    defer pair.deinit();
    try testing.expect(ready);
    const nav = &pair.nav;
    const expected_nav = pair.reference_nav;

    const refs = try gpa.alloc(TileRef, count);
    defer gpa.free(refs);
    for (blobs.items.items, refs) |blob, *ref| {
        try testing.expect((try pair.add(blob, .{ .free_data = true }, 0, ref)).succeeded);
    }
    try expectSameMeshes(expected_nav, nav);

    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();
    const order = try gpa.alloc(usize, count);
    defer gpa.free(order);
    for (order, 0..) |*o, i| o.* = i;
    random.shuffle(usize, order);
    const removed = order[0 .. count / 3 + 1];

    // Zero names no tile.
    var taken: []align(4) u8 = &.{};
    try testing.expect((try expectSameRemoval(expected_nav, nav, 0, &taken)).failed);
    try testing.expect((try expectSameRemoval(expected_nav, nav, 0, null)).failed);

    for (removed, 0..) |tile, step| {
        errdefer std.debug.print("  seed {d}, removal {d}\n", .{ seed, step });
        const blob = if (random.boolean()) &taken else null;
        try testing.expect((try expectSameRemoval(expected_nav, nav, refs[tile], blob)).succeeded);
        // The mesh owned the blob and freed it.
        try testing.expectEqual(@as(usize, 0), taken.len);
        // The reference is stale from here on.
        try testing.expect((try expectSameRemoval(expected_nav, nav, refs[tile], &taken)).failed);
        try expectSameMeshes(expected_nav, nav);
    }
    try expectSameLookups(&level, expected_nav, nav, seed + 1);

    // Half go back into the entries they had and get the references they
    // had; then the rest go wherever the free list puts them.
    random.shuffle(usize, removed);
    const half = removed.len / 2;
    for (removed, 0..) |tile, step| {
        errdefer std.debug.print("  seed {d}, return {d}\n", .{ seed, step });
        const last_ref: TileRef = if (step < half) refs[tile] else 0;
        var ref: TileRef = 0;
        const blob = blobs.items.items[tile];
        try testing.expect((try pair.add(blob, .{ .free_data = true }, last_ref, &ref)).succeeded);
        if (last_ref != 0) try testing.expectEqual(last_ref, ref);
        try testing.expect(ref != 0);
        try expectSameMeshes(expected_nav, nav);
    }
    try testing.expectEqual(count, tileCount(nav));
    try expectSameLookups(&level, expected_nav, nav, seed + 2);
}

test "removeTile: a blob the caller owns comes back" {
    const gpa = testing.allocator;
    var level: support.Level = try .init(gpa, support.dungeon_obj, support.tiled_config, .{
        .off_mesh_connection_count = 12,
        .seed = 91,
    });
    defer level.deinit();
    var blobs: TileBlobs = try .init(gpa, &level);
    defer blobs.deinit();
    const count = blobs.items.items.len;
    try testing.expect(count > 4);

    var pair: Pair = undefined;
    const ready = try pair.init(gpa, level.navMeshParams());
    defer pair.deinit();
    try testing.expect(ready);
    const nav = &pair.nav;
    const expected_nav = pair.reference_nav;

    // Every third blob is the mesh's to free; the rest stay the caller's.
    const refs = try gpa.alloc(TileRef, count);
    defer gpa.free(refs);
    for (blobs.items.items, refs, 0..) |blob, *ref, i| {
        const flags: TileFlags = .{ .free_data = i % 3 == 0 };
        try testing.expect((try pair.add(blob, flags, 0, ref)).succeeded);
    }
    try expectSameMeshes(expected_nav, nav);

    // What comes back is the blob as the mesh left it, links and all.
    const returned = try gpa.alloc([]align(4) u8, count);
    defer gpa.free(returned);
    var returned_count: usize = 0;
    for (refs, returned, 0..) |ref, *blob, i| {
        errdefer std.debug.print("  removing tile {d}\n", .{i});
        blob.* = &.{};
        // Every fourth removal does not ask for the blob.
        const asked = i % 4 != 3;
        const status = try expectSameRemoval(expected_nav, nav, ref, if (asked) blob else null);
        try testing.expect(status.succeeded);
        if (asked) try testing.expectEqual(i % 3 == 0, blob.len == 0);
        returned_count += @intFromBool(blob.len != 0);
        try expectSameMeshes(expected_nav, nav);
    }
    try testing.expectEqual(@as(usize, 0), tileCount(nav));
    try testing.expect(returned_count > 1);

    // A blob that was a tile before is a tile again.
    for (returned, 0..) |blob, i| {
        errdefer std.debug.print("  returning tile {d}\n", .{i});
        if (blob.len == 0) continue;
        try testing.expect((try pair.add(blob, .{}, 0, null)).succeeded);
    }
    try testing.expectEqual(returned_count, tileCount(nav));
    try expectSameMeshes(expected_nav, nav);
    try expectSameLookups(&level, expected_nav, nav, 92);
}

test "addTile and removeTile: what both sides refuse" {
    const gpa = testing.allocator;
    var level: support.Level = try .init(gpa, support.nav_test_obj, support.tiled_config, .{
        .off_mesh_connection_count = 8,
        .seed = 95,
    });
    defer level.deinit();
    var blobs: TileBlobs = try .init(gpa, &level);
    defer blobs.deinit();
    const items = blobs.items.items;
    try testing.expect(items.len >= 6);

    // Five entries in a table whose references have room for eight.
    var params = level.navMeshParams();
    params.max_tiles = 5;
    var pair: Pair = undefined;
    const ready = try pair.init(gpa, params);
    defer pair.deinit();
    try testing.expect(ready);
    const nav = &pair.nav;
    const expected_nav = pair.reference_nav;
    const owned: TileFlags = .{ .free_data = true };

    // Another magic, another version.
    for ([_]usize{ 0, 4 }) |offset| {
        items[0][offset] ^= 0x10;
        var ref: TileRef = 0x77;
        const status = try pair.add(items[0], owned, 0, &ref);
        items[0][offset] ^= 0x10;
        try testing.expect(status.failed);
        try testing.expectEqual(offset == 0, status.wrong_magic);
        try testing.expectEqual(offset == 4, status.wrong_version);
        try testing.expectEqual(@as(TileRef, 0x77), ref);
    }
    try expectSameMeshes(expected_nav, nav);

    var refs: [5]TileRef = @splat(0);
    try testing.expect((try pair.add(items[0], owned, 0, &refs[0])).succeeded);
    try testing.expect((try pair.add(items[1], owned, 0, &refs[1])).succeeded);

    // The grid cell and layer hold a tile already.
    const occupied = try pair.add(items[0], owned, 0, null);
    try testing.expect(occupied.failed and occupied.already_occupied);

    // The entry a reference asks for is in use, or is past the table.
    const in_use = try pair.add(items[2], owned, refs[0], null);
    try testing.expect(in_use.failed and in_use.out_of_memory);
    for ([_]u32{ 5, 6, 7 }) |tile_index| {
        const past = nav.encodePolyId(1, tile_index, 0);
        const status = try pair.add(items[2], owned, past, null);
        try testing.expect(status.failed and status.out_of_memory);
        var taken: []align(4) u8 = &.{};
        const removal = try expectSameRemoval(expected_nav, nav, past, &taken);
        try testing.expect(removal.failed and removal.invalid_param);
    }
    try expectSameMeshes(expected_nav, nav);

    // An entry from the middle of the free list, with the salt asked for.
    const asked = nav.encodePolyId(7, 3, 0);
    try testing.expect((try pair.add(items[2], owned, asked, &refs[2])).succeeded);
    try testing.expectEqual(asked, refs[2]);
    try expectSameMeshes(expected_nav, nav);

    // The table fills; a sixth tile has no entry, asked for or not.
    try testing.expect((try pair.add(items[3], owned, 0, &refs[3])).succeeded);
    try testing.expect((try pair.add(items[4], owned, 0, &refs[4])).succeeded);
    const full = try pair.add(items[5], owned, 0, null);
    try testing.expect(full.failed and full.out_of_memory);
    const full_asked = try pair.add(items[5], owned, nav.encodePolyId(2, 4, 0), null);
    try testing.expect(full_asked.failed and full_asked.out_of_memory);
    try expectSameMeshes(expected_nav, nav);
    try expectSameLookups(&level, expected_nav, nav, 96);

    // One entry freed is the sixth tile's.
    try testing.expect((try expectSameRemoval(expected_nav, nav, refs[1], null)).succeeded);
    try testing.expect((try pair.add(items[5], owned, 0, null)).succeeded);
    try expectSameMeshes(expected_nav, nav);
}

test "addTile: more polygons than a reference's polygon bits hold" {
    const gpa = testing.allocator;
    var level: support.Level = try .init(gpa, support.nav_test_obj, support.tiled_config, .{
        .seed = 97,
    });
    defer level.deinit();
    var blobs: TileBlobs = try .init(gpa, &level);
    defer blobs.deinit();

    var params = level.navMeshParams();
    params.max_polys = 2;
    var pair: Pair = undefined;
    const ready = try pair.init(gpa, params);
    defer pair.deinit();
    try testing.expect(ready);

    var refused: usize = 0;
    for (blobs.items.items) |blob| {
        const header: *align(1) const detour.MeshHeader = @ptrCast(blob.ptr);
        const status = try pair.add(blob, .{ .free_data = true }, 0, null);
        // A 64-bit reference has the same polygon bits whatever the mesh.
        const fits = detour.nav_mesh.polyref64 or header.poly_count <= 2;
        try testing.expectEqual(fits, status.succeeded);
        if (!fits) try testing.expect(status.invalid_param);
        refused += @intFromBool(header.poly_count > 2);
    }
    try testing.expect(refused > 0);
    try expectSameMeshes(pair.reference_nav, &pair.nav);
}

test "init: the shapes both sides accept and refuse" {
    const gpa = testing.allocator;
    const shapes = [_][2]i32{
        .{ 0, 16 },
        .{ 1, 1 },
        .{ 1, 1 << 22 },
        .{ 2, 1 << 22 },
        .{ 1 << 14, 1 << 8 },
        .{ 3, 5 },
        .{ 1024, 1024 },
        .{ 4096, 4096 },
    };
    var refused: usize = 0;
    for (shapes) |shape| {
        errdefer std.debug.print("  {d} tiles of {d} polygons\n", .{ shape[0], shape[1] });
        var pair: Pair = undefined;
        const ready = try pair.init(gpa, shapeParams(shape[0], shape[1]));
        defer pair.deinit();
        if (ready) {
            try expectSameMeshes(pair.reference_nav, &pair.nav);
        } else {
            refused += 1;
            try testing.expectEqual(@as(usize, 0), pair.nav.tiles.len);
        }
    }
    // The salt needs ten bits of a 32-bit reference.
    try testing.expectEqual(@as(usize, if (detour.nav_mesh.polyref64) 0 else 2), refused);
}

test "initSingleTile" {
    const gpa = testing.allocator;
    var level: support.Level = try .init(gpa, support.dungeon_obj, support.solo_config, .{
        .off_mesh_connection_count = 6,
        .seed = 111,
    });
    defer level.deinit();
    var input = (try level.tileInput(0, 0)) orelse return error.EmptyLevel;
    defer input.deinit();
    const bytes = input.referenceBytes() orelse return error.BuilderRefused;
    defer reference.ref_free(bytes.ptr);
    const owned: TileFlags = .{ .free_data = true };

    // A byte of the magic changed, a byte of the version, and neither.
    for ([_]?usize{ 0, 4, null }) |changed| {
        const copy = try gpa.alignedAlloc(u8, .@"4", bytes.len);
        var copy_owned = true;
        defer if (copy_owned) gpa.free(copy);
        @memcpy(copy, bytes);
        const reference_copy: [*]u8 = @ptrCast(
            reference.ref_alloc(bytes.len, .perm) orelse return error.OutOfMemory,
        );
        var reference_copy_owned = true;
        defer if (reference_copy_owned) reference.ref_free(reference_copy);
        @memcpy(reference_copy[0..bytes.len], bytes);
        if (changed) |offset| {
            copy[offset] ^= 0x40;
            reference_copy[offset] ^= 0x40;
        }

        const expected_nav = reference.ref_nav_mesh_alloc() orelse return error.OutOfMemory;
        defer reference.ref_nav_mesh_free(expected_nav);
        var nav: NavMesh = .empty;
        defer nav.deinit();

        const expected = reference.ref_nav_mesh_init_single(
            expected_nav,
            reference_copy,
            @intCast(bytes.len),
            @bitCast(owned),
        );
        const actual = nav.initSingleTile(gpa, copy, owned);
        if (reference.ref_status_succeeded(expected)) reference_copy_owned = false;
        if (actual.succeeded) copy_owned = false;
        try support.expectStatus(expected, actual);
        try testing.expectEqual(changed == null, actual.succeeded);
        if (actual.succeeded) {
            try testing.expectEqual(@as(usize, 1), tileCount(&nav));
            try expectSameMeshes(expected_nav, &nav);
        } else {
            try testing.expectEqual(@as(usize, 0), nav.tiles.len);
        }
    }

    // A blob cut short names blocks it does not hold: refused, and the mesh
    // left empty. (The original reads past the end, so the port runs alone.)
    const seed = 113;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();
    for (0..1000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const len = if (case < 200) case else random.uintLessThan(usize, bytes.len);
        const copy = try gpa.alignedAlloc(u8, .@"4", len);
        defer gpa.free(copy);
        @memcpy(copy, bytes[0..len]);
        var nav: NavMesh = .empty;
        const status = nav.initSingleTile(gpa, copy, owned);
        try testing.expect(status.failed and status.invalid_param);
        try testing.expectEqual(@as(usize, 0), nav.tiles.len);
    }
}

test "closestPointOnPoly and getPolyHeight" {
    const gpa = testing.allocator;
    for (world_cases, 0..) |case, case_index| {
        errdefer std.debug.print("  in world \"{s}\"\n", .{case.name});
        var world: support.World = undefined;
        try world.init(gpa, case.obj_text, case.config, case.options);
        defer world.deinit();
        const nav = &world.nav;

        const query = reference.ref_query_alloc() orelse return error.OutOfMemory;
        defer reference.ref_query_free(query);
        const ready = reference.ref_query_init(query, world.reference_nav, 64);
        try testing.expect(reference.ref_status_succeeded(ready));

        const seed = 200 + case_index;
        var prng: std.Random.DefaultPrng = .init(seed);
        const random = prng.random();
        var over_count: usize = 0;
        var off_count: usize = 0;
        var poly_count: usize = 0;
        for (nav.tiles) |*tile| {
            const base = nav.getPolyRefBase(tile);
            poly_count += tile.polys.len;
            for (tile.polys, 0..) |*poly, poly_index| {
                const ref = base | @as(PolyRef, @intCast(poly_index));
                errdefer std.debug.print("  seed {d}, polygon 0x{x}\n", .{ seed, ref });

                // The corners, the middles of the edges, the centre, points
                // near it at other heights, points around, and one far away.
                var points: [24][3]f32 = undefined;
                var count: usize = 0;
                var center: [3]f32 = .{ 0, 0, 0 };
                const vert_count: usize = poly.vert_count;
                const share = 1.0 / @as(f32, @floatFromInt(vert_count));
                for (0..vert_count) |i| {
                    const a = tile.verts[poly.verts[i]];
                    const b = tile.verts[poly.verts[(i + 1) % vert_count]];
                    points[count] = a;
                    points[count + 1] = .{
                        (a[0] + b[0]) * 0.5,
                        (a[1] + b[1]) * 0.5,
                        (a[2] + b[2]) * 0.5,
                    };
                    count += 2;
                    for (&center, a) |*sum, coordinate| sum.* += coordinate * share;
                }
                points[count] = center;
                count += 1;
                for (0..8) |k| {
                    const spread: f32 = if (k < 4) 0.4 else 8.0;
                    points[count] = .{
                        center[0] + (random.float(f32) - 0.5) * spread,
                        center[1] + (random.float(f32) - 0.5) * 6.0,
                        center[2] + (random.float(f32) - 0.5) * spread,
                    };
                    count += 1;
                }
                points[count] = .{ center[0] * 1000.0, center[1], center[2] * -1000.0 };
                count += 1;

                for (points[0..count], 0..) |pos, i| {
                    errdefer std.debug.print("  point {d}: {any}\n", .{ i, pos });
                    const over = try expectSameClosestPoint(query, nav, tile, poly, ref, pos);
                    if (over) over_count += 1 else off_count += 1;
                }
            }
        }
        // Both answers must have been compared, several times a polygon.
        try testing.expect(poly_count > 50);
        try testing.expect(over_count > poly_count * 2);
        try testing.expect(off_count > poly_count * 2);
    }
}

test "encodePolyId and decodePolyId" {
    const gpa = testing.allocator;
    const shapes = [_][2]i32{
        .{ 1, 1 },
        .{ 1, 1 << 22 },
        .{ 5, 1000 },
        .{ 1 << 14, 1 << 8 },
        .{ 1024, 1024 },
        .{ 64, 65536 },
    };
    for (shapes, 0..) |shape, shape_index| {
        var pair: Pair = undefined;
        const ready = try pair.init(gpa, shapeParams(shape[0], shape[1]));
        defer pair.deinit();
        try testing.expect(ready);
        const nav = &pair.nav;
        const expected_nav = pair.reference_nav;

        const seed = 300 + shape_index;
        var prng: std.Random.DefaultPrng = .init(seed);
        const random = prng.random();
        for (0..2000) |case| {
            errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
            // Parts of every width: the ones wider than their bits spill
            // into the part above, the same on both sides.
            const salt = random.int(u32) >> random.int(u5);
            const tile_index = random.int(u32) >> random.int(u5);
            const poly_index = random.int(u32) >> random.int(u5);
            const encoded = nav.encodePolyId(salt, tile_index, poly_index);
            try testing.expectEqual(
                reference.ref_nav_mesh_encode_poly_id(expected_nav, salt, tile_index, poly_index),
                encoded,
            );

            for ([_]PolyRef{ encoded, random.int(PolyRef), 0 }) |ref| {
                var expected_salt: u32 = 0;
                var expected_tile: u32 = 0;
                var expected_poly: u32 = 0;
                reference.ref_nav_mesh_decode_poly_id(
                    expected_nav,
                    ref,
                    &expected_salt,
                    &expected_tile,
                    &expected_poly,
                );
                const id = nav.decodePolyId(ref);
                try testing.expectEqual(expected_salt, id.salt);
                try testing.expectEqual(expected_tile, id.tile_index);
                try testing.expectEqual(expected_poly, id.poly_index);
                try testing.expectEqual(
                    reference.ref_nav_mesh_decode_poly_id_salt(expected_nav, ref),
                    nav.decodePolyIdSalt(ref),
                );
                try testing.expectEqual(
                    reference.ref_nav_mesh_decode_poly_id_tile(expected_nav, ref),
                    nav.decodePolyIdTile(ref),
                );
                try testing.expectEqual(
                    reference.ref_nav_mesh_decode_poly_id_poly(expected_nav, ref),
                    nav.decodePolyIdPoly(ref),
                );
            }
        }
    }
}

test "restoreTileState: corrupted and truncated states" {
    const gpa = testing.allocator;
    var world: support.World = undefined;
    try world.init(gpa, support.nav_test_obj, support.tiled_config, .{
        .vary_polygons = true,
        .seed = 121,
    });
    defer world.deinit();
    const nav = &world.nav;
    const expected_nav = world.reference_nav;
    const seed = 122;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var restored: usize = 0;
    var refused: usize = 0;
    for (0..2000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        var tile_index = random.uintLessThan(usize, nav.tiles.len);
        while (nav.tiles[tile_index].header == null) tile_index = (tile_index + 1) % nav.tiles.len;
        const tile = &nav.tiles[tile_index];
        const expected_tile: *reference.MeshTile =
            @constCast(reference.ref_nav_mesh_get_tile(expected_nav, @intCast(tile_index)).?);

        const size = NavMesh.getTileStateSize(tile);
        const state = try gpa.alloc(u8, size);
        defer gpa.free(state);
        @memset(state, 0);
        try testing.expect(nav.storeTileState(tile, state).succeeded);

        // Damage the state: some bytes anywhere, the head alone, the
        // polygon records alone, all of it, or its length.
        const head_size = 8 + @sizeOf(TileRef);
        var len = size;
        switch (random.uintLessThan(u8, 6)) {
            0 => for (0..random.intRangeAtMost(usize, 1, 4)) |_| {
                state[random.uintLessThan(usize, size)] ^= random.int(u8) | 1;
            },
            1 => state[random.uintLessThan(usize, head_size)] ^= @as(u8, 1) << random.int(u3),
            2 => random.bytes(state[head_size..]),
            3 => random.bytes(state),
            4 => len = random.uintLessThan(usize, size),
            else => {
                // The state of another tile: its reference does not match.
                const other = nav.encodePolyId(tile.salt, random.int(u8), 0);
                std.mem.writeInt(TileRef, state[8..][0..@sizeOf(TileRef)], other, .native);
            },
        }

        const status = nav.restoreTileState(tile, state[0..len]);
        try support.expectStatus(
            reference.ref_nav_mesh_restore_tile_state(
                expected_nav,
                expected_tile,
                state.ptr,
                @intCast(len),
            ),
            status,
        );
        if (status.succeeded) restored += 1 else refused += 1;
        try expectSameTile(expected_nav, expected_tile, nav, tile);
    }
    try testing.expect(restored > 100);
    try testing.expect(refused > 100);
    try expectSameMeshes(expected_nav, nav);
}
