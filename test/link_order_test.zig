//! The integer mesh's key link order (`LinkOrder.key`). Opened in key order,
//! a mesh links every polygon as the original does, byte for byte, and its
//! queries answer as the original order's. With tiles removed and added
//! again, in any order, it links every polygon as a mesh opened afresh from
//! the tiles it holds, and its queries answer the same as that mesh's; with
//! the original's order they do not.

const std = @import("std");
const detour = @import("detour");
const support = @import("support.zig");

const integer = detour.integer;
const testing = std.testing;

const PolyRef = detour.PolyRef;

/// Millimetres per metre.
const scale = integer.convert.default_scale;

/// A level's integer tiles: the original builder's float tiles, converted.
const Tiles = struct {
    level: support.Level,
    params: integer.NavMeshParams,
    /// Each tile's blob as the converter made it, in key order.
    blobs: std.ArrayList([]align(4) u8),

    /// Free with `deinit`. `self` must not move afterwards.
    fn init(
        self: *Tiles,
        gpa: std.mem.Allocator,
        obj_text: []const u8,
        config: support.RecastConfig,
        options: support.LevelOptions,
    ) !void {
        self.level = try .init(gpa, obj_text, config, options);
        errdefer self.level.deinit();
        const params = self.level.navMeshParams();
        self.params = .{
            .origin = millimetres(params.orig),
            .tile_width = @intFromFloat(@round(params.tile_width * scale)),
            .tile_height = @intFromFloat(@round(params.tile_height * scale)),
            .tile_count_max = params.max_tiles,
            .poly_count_max = params.max_polys,
        };
        self.blobs = .empty;
        errdefer self.freeBlobs(gpa);
        var y: i32 = 0;
        while (y < self.level.grid.height) : (y += 1) {
            var x: i32 = 0;
            while (x < self.level.grid.width) : (x += 1) {
                var input = (try self.level.tileInput(x, y)) orelse continue;
                defer input.deinit();
                const bytes = input.referenceBytes() orelse continue;
                defer support.reference.ref_free(bytes.ptr);
                const blob = try integer.convert.tileFromFloat(gpa, bytes, scale);
                errdefer gpa.free(blob);
                try self.blobs.append(gpa, blob);
            }
        }
    }

    fn freeBlobs(self: *Tiles, gpa: std.mem.Allocator) void {
        for (self.blobs.items) |blob| gpa.free(blob);
        self.blobs.deinit(gpa);
    }

    fn deinit(self: *Tiles, gpa: std.mem.Allocator) void {
        self.freeBlobs(gpa);
        self.level.deinit();
        self.* = undefined;
    }

    /// A mesh in `order` holding the tiles `present` marks, added in key
    /// order, each a copy of its blob the mesh owns.
    fn open(
        self: *const Tiles,
        gpa: std.mem.Allocator,
        order: integer.LinkOrder,
        present: []const bool,
    ) !integer.NavMesh {
        var params = self.params;
        params.link_order = order;
        var nav: integer.NavMesh = .empty;
        if (nav.init(gpa, &params).failed) return error.OutOfMemory;
        errdefer nav.deinit();
        for (self.blobs.items, present) |blob, here| {
            if (here) try add(gpa, &nav, blob);
        }
        return nav;
    }
};

/// Adds a copy of `blob` to `nav`, which owns it.
fn add(gpa: std.mem.Allocator, nav: *integer.NavMesh, blob: []align(4) const u8) !void {
    const copy = try gpa.alignedAlloc(u8, .@"4", blob.len);
    @memcpy(copy, blob);
    if (nav.addTile(copy, .{ .free_data = true }, 0, null).failed) {
        gpa.free(copy);
        return error.TileRefused;
    }
}

/// Removes the tile `blob` describes from `nav`.
fn remove(nav: *integer.NavMesh, blob: []align(4) const u8) !void {
    const header: *const integer.MeshHeader = @ptrCast(blob.ptr);
    const ref = nav.getTileRefAt(header.x, header.y, header.layer);
    if (ref == 0 or nav.removeTile(ref, null).failed) return error.TileNotRemoved;
}

fn millimetres(p: [3]f32) [3]i32 {
    var out: [3]i32 = undefined;
    for (&out, p) |*o, m| o.* = @intFromFloat(@round(m * scale));
    return out;
}

/// The levels: the demo's tiled level with off-mesh connections and
/// polygons of differing areas and flags, and the undulating one with every
/// other tile on other cells, so a polygon meets several across one edge.
const Level = struct {
    obj: []const u8,
    config: support.RecastConfig,
    options: support.LevelOptions,
};

const levels = [_]Level{
    .{
        .obj = support.nav_test_obj,
        .config = support.tiled_config,
        .options = .{ .off_mesh_connection_count = 12, .vary_polygons = true, .seed = 91 },
    },
    .{
        .obj = support.undulating_obj,
        .config = support.tiled_config,
        .options = .{ .odd_tile_cell_size = 0.2, .off_mesh_connection_count = 6, .seed = 92 },
    },
};

// ---------------------------------------------------------------------------
// Comparing meshes
// ---------------------------------------------------------------------------

/// A polygon named by where it is, the same in every mesh that holds its
/// tile, where a reference names a tile entry and its salt.
fn nameOf(nav: *const integer.NavMesh, ref: PolyRef) u64 {
    if (ref == 0) return 0;
    const found = nav.getTileAndPolyByRefUnsafe(ref);
    const header = found.tile.header.?;
    var hasher: std.hash.Wyhash = .init(0);
    hasher.update(std.mem.asBytes(&[_]i32{ header.x, header.y, header.layer }));
    hasher.update(std.mem.asBytes(&nav.decodePolyIdPoly(ref)));
    return hasher.final() | 1;
}

/// A hash of what decides every query of `nav`: each tile with each
/// polygon's links in list order (with the polygon each leads to by name),
/// its free links counted, and its vertices, which off-mesh connections
/// move. Tiles sit in entries that depend on the order they came, so their
/// hashes are summed. The same for two meshes that link alike.
fn linkHash(nav: *const integer.NavMesh) u64 {
    var sum: u64 = 0;
    for (nav.tiles) |*tile| {
        if (tile.header == null) continue;
        sum +%= tileHash(nav, tile);
    }
    return sum;
}

fn tileHash(nav: *const integer.NavMesh, tile: *const integer.MeshTile) u64 {
    const header = tile.header.?;
    var hasher: std.hash.Wyhash = .init(2);
    hasher.update(std.mem.asBytes(&[_]i32{ header.x, header.y, header.layer }));
    for (tile.polys) |*poly| {
        var at = poly.first_link;
        while (at != integer.nav_mesh.null_link) : (at = tile.links[at].next) {
            const link = tile.links[at];
            hasher.update(std.mem.asBytes(&nameOf(nav, link.ref)));
            hasher.update(&.{ link.edge, link.side, link.bounds_min, link.bounds_max });
        }
        hasher.update("|");
    }
    var free: u32 = 0;
    var at = tile.links_free_list;
    while (at != integer.nav_mesh.null_link) : (at = tile.links[at].next) free += 1;
    hasher.update(std.mem.asBytes(&free));
    hasher.update(std.mem.sliceAsBytes(tile.verts));
    return hasher.final();
}

/// Ground points of `tiles`' level with a polygon of `nav` near, for the
/// queries.
fn groundPoints(
    gpa: std.mem.Allocator,
    tiles: *const Tiles,
    query: *const integer.NavMeshQuery,
    seed: u64,
    count: usize,
) ![][3]i32 {
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(seed));
    const filter: integer.QueryFilter = .{};
    const points = try gpa.alloc([3]i32, count);
    for (points) |*point| {
        point.* = while (true) {
            const p = millimetres(tiles.level.groundPoint(prng.random(), 0.3));
            var ref: PolyRef = 0;
            _ = query.findNearestPoly(p, .{ 2000, 4000, 2000 }, &filter, &ref, null, null);
            if (ref != 0) break p;
        };
    }
    return points;
}

/// The answers `nav` gives to a fixed workload over `points`: the nearest
/// polygon, a path to the next point and its straight path, a ray, a walk
/// along the surface and the polygons around a circle, with every polygon
/// by name. A hash of them all.
fn answerHash(gpa: std.mem.Allocator, nav: *const integer.NavMesh, points: []const [3]i32) !u64 {
    var query: integer.NavMeshQuery = .empty;
    if (query.init(gpa, nav, 2048).failed) return error.OutOfMemory;
    defer query.deinit(gpa);
    const filter: integer.QueryFilter = .{};
    const reach = [3]i32{ 2000, 4000, 2000 };
    var hasher: std.hash.Wyhash = .init(0);
    var refs: [256]PolyRef = undefined;
    var costs: [256]u64 = undefined;
    var straight: [64][3]i32 = undefined;
    var straight_refs: [64]PolyRef = undefined;
    var straight_flags: [64]detour.StraightPathFlags = undefined;
    for (points, 0..) |from, i| {
        const to = points[(i * 7 + 3) % points.len];
        var start: PolyRef = 0;
        var end: PolyRef = 0;
        var start_at: [3]i32 = from;
        var end_at: [3]i32 = to;
        _ = query.findNearestPoly(from, reach, &filter, &start, &start_at, null);
        _ = query.findNearestPoly(to, reach, &filter, &end, &end_at, null);
        hashValues(&hasher, .{ nameOf(nav, start), nameOf(nav, end), start_at, end_at });
        if (start == 0 or end == 0) continue;

        var count: usize = 0;
        const path_status = query.findPath(start, end, start_at, end_at, &filter, &refs, &count);
        hashValues(&hasher, .{ @as(u32, @bitCast(path_status)), query.getNodePool().node_count });
        hashNames(&hasher, nav, refs[0..count]);
        var straight_count: usize = 0;
        _ = query.findStraightPath(
            start_at,
            end_at,
            refs[0..count],
            &straight,
            &straight_flags,
            &straight_refs,
            &straight_count,
            .{ .area_crossings = true },
        );
        hasher.update(std.mem.sliceAsBytes(straight[0..straight_count]));
        hasher.update(std.mem.sliceAsBytes(straight_flags[0..straight_count]));
        hashNames(&hasher, nav, straight_refs[0..straight_count]);

        var hit: integer.RaycastHit = .{
            .t = .zero,
            .hit_normal = @splat(0),
            .hit_edge_index = 0,
            .path = &refs,
            .path_count = 0,
            .path_cost = 0,
        };
        _ = query.raycast(start, start_at, end_at, &filter, .{ .use_costs = true }, &hit, 0);
        hashValues(&hasher, .{ hit.t, hit.hit_normal, hit.hit_edge_index, hit.path_cost });
        hashNames(&hasher, nav, refs[0..hit.path_count]);

        var moved: [3]i32 = undefined;
        var visited_count: usize = 0;
        _ = query.moveAlongSurface(start, start_at, end_at, &filter, &moved, &refs, &visited_count);
        hashValues(&hasher, .{moved});
        hashNames(&hasher, nav, refs[0..visited_count]);

        var around: usize = 0;
        _ = query.findPolysAroundCircle(
            start,
            start_at,
            6000,
            &filter,
            &refs,
            null,
            &costs,
            &around,
            refs.len,
        );
        hashNames(&hasher, nav, refs[0..around]);
        hasher.update(std.mem.sliceAsBytes(costs[0..around]));
    }
    return hasher.final();
}

/// `got` answers the workload over `points` as `want` does.
fn expectSameAnswers(
    gpa: std.mem.Allocator,
    want: *const integer.NavMesh,
    got: *const integer.NavMesh,
    points: []const [3]i32,
) !void {
    try testing.expectEqual(try answerHash(gpa, want, points), try answerHash(gpa, got, points));
}

fn hashValues(hasher: *std.hash.Wyhash, values: anytype) void {
    inline for (values) |value| hasher.update(std.mem.asBytes(&value));
}

fn hashNames(hasher: *std.hash.Wyhash, nav: *const integer.NavMesh, refs: []const PolyRef) void {
    for (refs) |ref| hasher.update(std.mem.asBytes(&nameOf(nav, ref)));
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

test "key link order: a mesh opened in key order is the original order's, byte for byte" {
    const gpa = testing.allocator;
    for (levels) |spec| {
        var tiles: Tiles = undefined;
        try tiles.init(gpa, spec.obj, spec.config, spec.options);
        defer tiles.deinit(gpa);
        const all = try gpa.alloc(bool, tiles.blobs.items.len);
        defer gpa.free(all);
        @memset(all, true);
        try testing.expect(all.len > 4);

        var added = try tiles.open(gpa, .added, all);
        defer added.deinit();
        var keyed = try tiles.open(gpa, .key, all);
        defer keyed.deinit();
        // The same tile entries, so the blobs match byte for byte: links,
        // their lists and the vertices off-mesh connections moved.
        for (added.tiles, keyed.tiles) |*a, *k| {
            try testing.expectEqual(a.header == null, k.header == null);
            if (a.header == null) continue;
            try testing.expectEqual(a.links_free_list, k.links_free_list);
            try testing.expectEqualSlices(u8, a.data, k.data);
        }

        var query: integer.NavMeshQuery = .empty;
        if (query.init(gpa, &added, 2048).failed) return error.OutOfMemory;
        defer query.deinit(gpa);
        const points = try groundPoints(gpa, &tiles, &query, spec.options.seed, 300);
        defer gpa.free(points);
        try expectSameAnswers(gpa, &added, &keyed, points);
    }
}

test "key link order: tiles removed and added again link and answer as a fresh mesh of them" {
    const gpa = testing.allocator;
    for (levels) |spec| {
        var tiles: Tiles = undefined;
        try tiles.init(gpa, spec.obj, spec.config, spec.options);
        defer tiles.deinit(gpa);
        const count = tiles.blobs.items.len;
        const present = try gpa.alloc(bool, count);
        defer gpa.free(present);
        @memset(present, true);

        // Opened in a shuffled order.
        var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(spec.options.seed ^ 0x6c696e6b));
        const random = prng.random();
        const order = try gpa.alloc(usize, count);
        defer gpa.free(order);
        for (order, 0..) |*o, i| o.* = i;
        random.shuffle(usize, order);
        var params = tiles.params;
        params.link_order = .key;
        var nav: integer.NavMesh = .empty;
        if (nav.init(gpa, &params).failed) return error.OutOfMemory;
        defer nav.deinit();
        for (order) |i| try add(gpa, &nav, tiles.blobs.items[i]);

        var fresh_all = try tiles.open(gpa, .added, present);
        defer fresh_all.deinit();
        try testing.expectEqual(linkHash(&fresh_all), linkHash(&nav));

        var query: integer.NavMeshQuery = .empty;
        if (query.init(gpa, &fresh_all, 2048).failed) return error.OutOfMemory;
        defer query.deinit(gpa);
        const points = try groundPoints(gpa, &tiles, &query, spec.options.seed, 200);
        defer gpa.free(points);
        try expectSameAnswers(gpa, &fresh_all, &nav, points);

        // Tiles removed and added again, a few at a time, each step checked
        // against a mesh opened afresh from the tiles then present.
        for (0..12) |_| {
            const i = random.uintLessThan(usize, count);
            if (present[i]) try remove(&nav, tiles.blobs.items[i]) else {
                try add(gpa, &nav, tiles.blobs.items[i]);
            }
            present[i] = !present[i];
            var fresh = try tiles.open(gpa, .added, present);
            defer fresh.deinit();
            try testing.expectEqual(linkHash(&fresh), linkHash(&nav));
            try expectSameAnswers(gpa, &fresh, &nav, points);
        }
        // Every tile back, in a shuffled order.
        random.shuffle(usize, order);
        for (order) |i| {
            if (present[i]) try remove(&nav, tiles.blobs.items[i]);
        }
        random.shuffle(usize, order);
        for (order) |i| try add(gpa, &nav, tiles.blobs.items[i]);
        try testing.expectEqual(linkHash(&fresh_all), linkHash(&nav));
        try expectSameAnswers(gpa, &fresh_all, &nav, points);
    }
}

test "key link order: removing any one tile links as a fresh mesh of the others" {
    const gpa = testing.allocator;
    for (levels) |spec| {
        var tiles: Tiles = undefined;
        try tiles.init(gpa, spec.obj, spec.config, spec.options);
        defer tiles.deinit(gpa);
        const present = try gpa.alloc(bool, tiles.blobs.items.len);
        defer gpa.free(present);
        @memset(present, true);
        var nav = try tiles.open(gpa, .key, present);
        defer nav.deinit();
        for (tiles.blobs.items, 0..) |blob, i| {
            try remove(&nav, blob);
            present[i] = false;
            var fresh = try tiles.open(gpa, .added, present);
            defer fresh.deinit();
            try testing.expectEqual(linkHash(&fresh), linkHash(&nav));
            try add(gpa, &nav, blob);
            present[i] = true;
        }
    }
}

test "the original link order: a tile removed and added again links its neighbours otherwise" {
    const gpa = testing.allocator;
    const spec = levels[1];
    var tiles: Tiles = undefined;
    try tiles.init(gpa, spec.obj, spec.config, spec.options);
    defer tiles.deinit(gpa);
    const all = try gpa.alloc(bool, tiles.blobs.items.len);
    defer gpa.free(all);
    @memset(all, true);
    var fresh = try tiles.open(gpa, .added, all);
    defer fresh.deinit();
    var nav = try tiles.open(gpa, .added, all);
    defer nav.deinit();
    // A tile in the middle of the level, with neighbours all round.
    const middle = tiles.blobs.items[tiles.blobs.items.len / 2];
    try remove(&nav, middle);
    try add(gpa, &nav, middle);
    try testing.expect(linkHash(&fresh) != linkHash(&nav));
}
