//! Benchmark rows for the navigation mesh, the tile builder and the geometry
//! helpers.

const std = @import("std");
const support = @import("support");
const Bench = @import("bench.zig").Bench;

const detour = support.detour;
const reference = support.reference;
const common = detour.common;

const PolyRef = detour.PolyRef;
const TileRef = detour.TileRef;
const TileLocation = detour.nav_mesh.TileLocation;
const verts_per_polygon = detour.nav_mesh.verts_per_polygon;

/// How many off-mesh connections the level has: enough for the builder, the
/// linking of a new tile and the end point lookup to have some to work on.
const off_mesh_connection_count = 32;
/// How many prepared inputs a row timed per call runs over.
const call_count = 1000;
/// The seed of every prepared input.
const input_seed = 0x6e61766d657368;
/// How many tiles one `getTilesAt` call may return.
const tiles_at_count_max = 32;
/// How many times a timed call of the header row goes over the tiles.
const header_swap_passes = 8;
/// How far a segment of the geometry rows reaches from its start along x
/// and along z.
const segment_reach: f32 = 6.0;
/// The original's flag for a tile whose bytes the mesh frees.
const reference_free_data: i32 = 1;
/// The original's flag for a tile whose bytes stay the caller's.
const reference_keep_data: i32 = 0;
/// Where a tile's polygon count sits in its bytes.
const poly_count_offset = @offsetOf(detour.MeshHeader, "poly_count");

/// A status as a checksum takes it.
inline fn statusSum(status: detour.Status) u64 {
    return @as(u32, @bitCast(status));
}

/// The bits of a float as a checksum takes them.
inline fn floatSum(value: f32) u64 {
    return @as(u32, @bitCast(value));
}

/// The bits of a signed integer as a checksum takes them.
inline fn intSum(value: i32) u64 {
    return @as(u32, @bitCast(value));
}

/// The bits of a point's three floats, added up.
inline fn pointSum(point: [3]f32) u64 {
    return floatSum(point[0]) +% floatSum(point[1]) +% floatSum(point[2]);
}

/// What a checksum takes of a block of bytes: eight or so of its 32-bit
/// words, evenly spread from the first on.
fn blobSum(bytes: []const u8) u64 {
    const word_count = bytes.len / 4;
    const step = @max(word_count / 8, 1);
    var sum: u64 = 0;
    var word: usize = 0;
    while (word < word_count) : (word += step) {
        sum +%= std.mem.readInt(u32, bytes[word * 4 ..][0..4], .little);
    }
    return sum;
}

/// A convex polygon of the mesh in world space: `count` of `verts` are its
/// corners, the rest are zero.
const Polygon = struct {
    verts: [verts_per_polygon][3]f32,
    count: u32,

    /// The corners, as the port's functions take them.
    inline fn corners(self: *const Polygon) []const [3]f32 {
        return self.verts[0..self.count];
    }

    /// The corners' floats, as the original's functions take them with
    /// `count`.
    inline fn floats(self: *const Polygon) [*]const f32 {
        return @ptrCast(&self.verts);
    }

    /// The mean of the corners.
    fn center(self: *const Polygon) [3]f32 {
        var result: [3]f32 = @splat(0);
        for (self.verts[0..self.count]) |vert| {
            for (&result, vert) |*sum, coordinate| sum.* += coordinate;
        }
        const scale = 1.0 / @as(f32, @floatFromInt(self.count));
        for (&result) |*coordinate| coordinate.* *= scale;
        return result;
    }
};

/// The inputs of one call of each geometry row.
const GeometryCase = struct {
    /// A ground polygon of the mesh.
    polygon: Polygon,
    /// 0, 1, 2 and so on: the polygon's corners by index.
    indices: [verts_per_polygon]u16,
    /// Another ground polygon of the mesh, moved so its center is at `point`.
    other: Polygon,
    /// A point between the polygon's center and twice as far out as one of
    /// its corners: inside the polygon about as often as outside.
    point: [3]f32,
    /// The end of a segment from `point`, up to `segment_reach` away along
    /// x and along z.
    far: [3]f32,
    /// A point of the plane through the polygon's first three corners,
    /// inside their triangle about three times in four.
    triangle_point: [3]f32,
    /// Two numbers from 0 to 1 that pick a point of the polygon.
    s: f32,
    t: f32,
};

/// One walk over an off-mesh connection: the polygon the walker comes from
/// and the connection's own polygon.
const ConnectionStep = struct {
    prev_ref: PolyRef,
    poly_ref: PolyRef,
};

/// One tile of the level: what both builders take, the bytes they build and
/// the tile's place in each side's full mesh.
const Tile = struct {
    input: support.TileInput,
    /// The port's builder parameters. They point into what `input` owns.
    params: detour.NavMeshCreateParams,
    /// The tile's bytes for the port's mesh of the add and remove row.
    bytes: []align(4) u8,
    /// The same bytes from the original's allocator, for its mesh of that
    /// row.
    reference_bytes: []u8,
    /// The same bytes for the byte-swap rows, which both sides share.
    swap_bytes: []align(4) u8,

    /// The tile in each side's full mesh, and its last polygon there. Set by
    /// `Context.addToFullMeshes`, as is `state`.
    tile: *const detour.MeshTile,
    reference_tile: *const reference.MeshTile,
    last_poly: *const detour.Poly,
    reference_last_poly: *const reference.Poly,
    /// Zeroed room for the tile's state, which both sides write and read.
    state: []align(4) u8,

    /// Runs Recast on tile (`x`, `y`) of `level` and builds its bytes with
    /// the original's builder. Null for a tile with no ground or one the
    /// builder refuses. Free with `deinit`.
    fn init(level: *const support.Level, x: i32, y: i32) !?Tile {
        var input = (try level.tileInput(x, y)) orelse return null;
        errdefer input.deinit();
        const reference_bytes = input.referenceBytes() orelse {
            input.deinit();
            return null;
        };
        errdefer reference.ref_free(reference_bytes.ptr);

        const c_allocator = std.heap.c_allocator;
        const bytes = try c_allocator.alignedAlloc(u8, .@"4", reference_bytes.len);
        errdefer c_allocator.free(bytes);
        @memcpy(bytes, reference_bytes);
        const swap_bytes = try c_allocator.alignedAlloc(u8, .@"4", reference_bytes.len);
        @memcpy(swap_bytes, reference_bytes);

        return .{
            .input = input,
            .params = support.portCreateParams(&input.reference_params),
            .bytes = bytes,
            .reference_bytes = reference_bytes,
            .swap_bytes = swap_bytes,
            .tile = undefined,
            .reference_tile = undefined,
            .last_poly = undefined,
            .reference_last_poly = undefined,
            .state = &.{},
        };
    }

    fn deinit(self: *Tile) void {
        std.heap.c_allocator.free(self.swap_bytes);
        std.heap.c_allocator.free(self.bytes);
        reference.ref_free(self.reference_bytes.ptr);
        self.input.deinit();
        self.* = undefined;
    }
};

/// One of the original's meshes, sized for the level's tiles. Free with
/// `reference.ref_nav_mesh_free`.
fn referenceMesh(params: *const reference.NavMeshParams) !*reference.NavMesh {
    const nav = reference.ref_nav_mesh_alloc() orelse return error.OutOfMemory;
    errdefer reference.ref_nav_mesh_free(nav);
    if (reference.ref_status_failed(reference.ref_nav_mesh_init(nav, params))) {
        return error.InitFailed;
    }
    return nav;
}

/// The tiled level with its off-mesh connections, every tile's builder
/// input and bytes, two meshes on each side (one that holds every tile, one
/// that holds none) and the inputs the rows run over.
const Context = struct {
    /// Owns every slice of the context that no other field says it owns.
    arena: std.heap.ArenaAllocator,
    level: support.Level,
    /// The tiles of the level that have ground, row by row of the grid.
    tiles: []Tile,
    /// The bytes of all the tiles together.
    tile_bytes_total: usize,

    /// The meshes that hold every tile: the port's, then the original's.
    nav: detour.NavMesh,
    reference_nav: *reference.NavMesh,
    /// The meshes that hold no tile between two timed calls.
    empty_nav: detour.NavMesh,
    reference_empty_nav: *reference.NavMesh,
    /// Where one of the original's tiles keeps the pointer to its header.
    reference_header_offset: usize,

    /// Where the add and remove row keeps the references of the tiles it
    /// added, one a tile.
    tile_refs: []TileRef,
    /// The reference of every polygon of the full mesh, in random order.
    poly_refs: []PolyRef,
    /// The references of `poly_refs` again, about one in four of them made
    /// stale: its salt is one more than its tile's.
    validity_refs: []PolyRef,
    /// The flags the flags row gives each polygon of `poly_refs`.
    poly_flags: []u16,
    /// The area id the area row gives each polygon of `poly_refs`.
    poly_areas: []u8,
    /// Points on the level's ground, each up to 0.3 off it along every axis.
    points: [][3]f32,
    /// The tile grid cell of each point.
    tile_locations: []TileLocation,
    /// The reference of the tile at each of `tile_locations`; zero where the
    /// cell has no tile.
    located_tile_refs: []TileRef,
    /// Walks over the level's off-mesh connections from either end, repeated
    /// to `call_count`; empty when no connection of the level got linked.
    connection_steps: []ConnectionStep,
    /// The polygons of the off-mesh connections, repeated to `call_count`;
    /// empty when the mesh has none.
    connection_refs: []PolyRef,
    cases: []GeometryCase,

    /// Builds the level, its tiles, the meshes and the inputs. `self` must
    /// not move afterwards. Free with `deinit`.
    fn init(self: *Context, gpa: std.mem.Allocator) !void {
        self.arena = .init(gpa);
        errdefer self.arena.deinit();
        const arena = self.arena.allocator();

        self.level = try .init(gpa, support.nav_test_obj, support.tiled_config, .{
            .off_mesh_connection_count = off_mesh_connection_count,
        });
        errdefer self.level.deinit();

        const grid = self.level.grid;
        const tiles = try arena.alloc(Tile, @intCast(grid.width * grid.height));
        var tile_count: usize = 0;
        errdefer for (tiles[0..tile_count]) |*tile| tile.deinit();
        var y: i32 = 0;
        while (y < grid.height) : (y += 1) {
            var x: i32 = 0;
            while (x < grid.width) : (x += 1) {
                tiles[tile_count] = (try Tile.init(&self.level, x, y)) orelse continue;
                tile_count += 1;
            }
        }
        if (tile_count == 0) return error.EmptyLevel;
        self.tiles = tiles[0..tile_count];
        self.tile_bytes_total = 0;
        for (self.tiles) |*tile| self.tile_bytes_total += tile.bytes.len;
        self.tile_refs = try arena.alloc(TileRef, tile_count);
        @memset(self.tile_refs, 0);

        const params = self.level.navMeshParams();
        const port_params: detour.NavMeshParams = .{
            .origin = params.orig,
            .tile_width = params.tile_width,
            .tile_height = params.tile_height,
            .tile_count_max = params.max_tiles,
            .poly_count_max = params.max_polys,
        };
        self.empty_nav = .empty;
        if (self.empty_nav.init(std.heap.c_allocator, &port_params).failed) {
            return error.InitFailed;
        }
        errdefer self.empty_nav.deinit();
        self.nav = .empty;
        if (self.nav.init(std.heap.c_allocator, &port_params).failed) return error.InitFailed;
        errdefer self.nav.deinit();
        self.reference_empty_nav = try referenceMesh(&params);
        errdefer reference.ref_nav_mesh_free(self.reference_empty_nav);
        self.reference_nav = try referenceMesh(&params);
        errdefer reference.ref_nav_mesh_free(self.reference_nav);
        self.reference_header_offset = reference.ref_offsetof_mesh_tile_header();
        for (self.tiles) |*tile| try self.addToFullMeshes(tile);

        var prng: std.Random.DefaultPrng = .init(input_seed);
        const random = prng.random();
        try self.initPolygonInputs(random);
        try self.initPointInputs(random);
    }

    fn deinit(self: *Context) void {
        reference.ref_nav_mesh_free(self.reference_nav);
        reference.ref_nav_mesh_free(self.reference_empty_nav);
        self.nav.deinit();
        self.empty_nav.deinit();
        for (self.tiles) |*tile| tile.deinit();
        self.level.deinit();
        self.arena.deinit();
        self.* = undefined;
    }

    /// Adds a copy of `tile`'s bytes to each side's full mesh, which owns
    /// the copy from then on, and readies the fields of `tile` that point
    /// into those meshes.
    fn addToFullMeshes(self: *Context, tile: *Tile) !void {
        const c_allocator = std.heap.c_allocator;
        const len = tile.bytes.len;

        const copy = try c_allocator.alignedAlloc(u8, .@"4", len);
        @memcpy(copy, tile.bytes);
        var ref: TileRef = 0;
        if (self.nav.addTile(copy, .{ .free_data = true }, 0, &ref).failed) {
            c_allocator.free(copy);
            return error.AddTileFailed;
        }

        const reference_copy: [*]u8 = @ptrCast(
            reference.ref_alloc(len, .perm) orelse return error.OutOfMemory,
        );
        @memcpy(reference_copy[0..len], tile.bytes);
        var reference_ref: TileRef = 0;
        const added = reference.ref_nav_mesh_add_tile(
            self.reference_nav,
            reference_copy,
            @intCast(len),
            reference_free_data,
            0,
            &reference_ref,
        );
        if (reference.ref_status_failed(added)) {
            reference.ref_free(reference_copy);
            return error.AddTileFailed;
        }
        // The rows give both sides the same references.
        if (ref != reference_ref) return error.SidesDiffer;

        const port_tile = self.nav.getTileByRef(ref) orelse return error.AddTileFailed;
        const reference_tile =
            reference.ref_nav_mesh_get_tile_by_ref(self.reference_nav, ref) orelse
            return error.AddTileFailed;
        const reference_polys = reference.ref_tile_polys(reference_tile) orelse
            return error.AddTileFailed;
        if (port_tile.polys.len == 0) return error.AddTileFailed;
        tile.tile = port_tile;
        tile.reference_tile = reference_tile;
        tile.last_poly = &port_tile.polys[port_tile.polys.len - 1];
        tile.reference_last_poly = &reference_polys[port_tile.polys.len - 1];

        const state_size = detour.NavMesh.getTileStateSize(port_tile);
        const reference_state_size: usize = @intCast(
            reference.ref_nav_mesh_get_tile_state_size(self.reference_nav, reference_tile),
        );
        if (state_size != reference_state_size) return error.SidesDiffer;
        tile.state = try self.arena.allocator().alignedAlloc(u8, .@"4", state_size);
        @memset(tile.state, 0);
    }

    /// Lists every polygon of the full mesh and draws the inputs made from
    /// the polygons: their references in random order, the flags and area
    /// ids two rows give them, the walks over the off-mesh connections and
    /// the geometry cases.
    fn initPolygonInputs(self: *Context, random: std.Random) !void {
        const arena = self.arena.allocator();
        const null_link = detour.nav_mesh.null_link;
        const zero: [3]f32 = @splat(0);

        var refs: std.ArrayList(PolyRef) = .empty;
        var polygons: std.ArrayList(Polygon) = .empty;
        var steps: std.ArrayList(ConnectionStep) = .empty;
        var connections: std.ArrayList(PolyRef) = .empty;
        for (self.tiles) |*entry| {
            const tile = entry.tile;
            const base = self.nav.getPolyRefBase(tile);
            for (tile.polys, 0..) |*poly, index| {
                const ref = base | @as(PolyRef, @intCast(index));
                // The rows give both sides the same references.
                if (!reference.ref_nav_mesh_is_valid_poly_ref(self.reference_nav, ref)) {
                    return error.SidesDiffer;
                }
                try refs.append(arena, ref);

                if (poly.getType() == .off_mesh_connection) {
                    try connections.append(arena, ref);
                    // One walk from each polygon the connection is linked to.
                    var link = poly.first_link;
                    while (link != null_link) : (link = tile.links[link].next) {
                        try steps.append(arena, .{
                            .prev_ref = tile.links[link].ref,
                            .poly_ref = ref,
                        });
                    }
                } else if (poly.vert_count >= 3) {
                    var polygon: Polygon = .{ .verts = @splat(zero), .count = poly.vert_count };
                    const corners = poly.verts[0..poly.vert_count];
                    for (polygon.verts[0..corners.len], corners) |*vert, v| vert.* = tile.verts[v];
                    try polygons.append(arena, polygon);
                }
            }
        }
        if (polygons.items.len == 0) return error.EmptyLevel;

        random.shuffle(PolyRef, refs.items);
        self.poly_refs = refs.items;
        self.validity_refs = try arena.dupe(PolyRef, refs.items);
        for (self.validity_refs) |*ref| {
            if (random.uintLessThan(u8, 4) != 0) continue;
            const id = self.nav.decodePolyId(ref.*);
            ref.* = self.nav.encodePolyId(id.salt + 1, id.tile_index, id.poly_index);
        }
        self.poly_flags = try arena.alloc(u16, refs.items.len);
        for (self.poly_flags) |*flags| flags.* = random.int(u16);
        self.poly_areas = try arena.alloc(u8, refs.items.len);
        for (self.poly_areas) |*area| {
            area.* = random.uintLessThan(u8, detour.nav_mesh.area_count_max);
        }

        self.connection_steps = try repeated(ConnectionStep, arena, steps.items);
        self.connection_refs = try repeated(PolyRef, arena, connections.items);

        self.cases = try arena.alloc(GeometryCase, call_count);
        for (self.cases) |*case| case.* = makeCase(random, polygons.items);
    }

    /// Draws the ground points and looks up, on the port's mesh, the tile
    /// grid cell of each and the tile there.
    fn initPointInputs(self: *Context, random: std.Random) !void {
        const arena = self.arena.allocator();
        self.points = try arena.alloc([3]f32, call_count);
        self.tile_locations = try arena.alloc(TileLocation, call_count);
        self.located_tile_refs = try arena.alloc(TileRef, call_count);
        for (self.points, self.tile_locations, self.located_tile_refs) |*point, *location, *ref| {
            point.* = self.level.groundPoint(random, 0.3);
            location.* = self.nav.calcTileLoc(point.*);
            ref.* = self.nav.getTileRefAt(location.x, location.y, 0);
        }
    }

    /// The header of one of the original's tiles, read as the port reads
    /// its own: through the pointer the tile keeps.
    inline fn referenceHeader(
        self: *const Context,
        tile: *const reference.MeshTile,
    ) *const reference.MeshHeader {
        const field: *const ?*const reference.MeshHeader =
            @ptrFromInt(@intFromPtr(tile) + self.reference_header_offset);
        return field.*.?;
    }

    fn baselineCreateNavMeshData(context: *Context) u64 {
        var sum: u64 = 0;
        for (context.tiles) |*tile| {
            var data: ?[*]u8 = null;
            var size: i32 = 0;
            const built = reference.ref_create_nav_mesh_data(
                &tile.input.reference_params,
                &data,
                &size,
            );
            if (!built) continue;
            const bytes = data.?[0..@intCast(size)];
            sum +%= 1;
            sum +%= bytes.len;
            sum +%= blobSum(bytes);
            reference.ref_free(bytes.ptr);
        }
        return sum;
    }

    fn portCreateNavMeshData(context: *Context) u64 {
        var sum: u64 = 0;
        for (context.tiles) |*tile| {
            const bytes = detour.createNavMeshData(std.heap.c_allocator, &tile.params) catch
                continue;
            sum +%= 1;
            sum +%= bytes.len;
            sum +%= blobSum(bytes);
            std.heap.c_allocator.free(bytes);
        }
        return sum;
    }

    fn baselineAddAndRemoveTiles(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = context.reference_empty_nav;
        for (context.tiles, context.tile_refs) |*tile, *ref| {
            sum +%= reference.ref_nav_mesh_add_tile(
                nav,
                tile.reference_bytes.ptr,
                @intCast(tile.reference_bytes.len),
                reference_keep_data,
                0,
                ref,
            );
            sum +%= ref.*;
        }
        // What the mesh wrote into the tiles' bytes: their links.
        for (context.tiles) |*tile| sum +%= blobSum(tile.reference_bytes);
        // Last in, first out, which leaves the mesh's unused entries in the
        // order they began in.
        var index = context.tiles.len;
        while (index > 0) {
            index -= 1;
            var data: ?[*]u8 = null;
            var size: i32 = 0;
            sum +%= reference.ref_nav_mesh_remove_tile(
                nav,
                context.tile_refs[index],
                &data,
                &size,
            );
            sum +%= intSum(size);
        }
        return sum;
    }

    fn portAddAndRemoveTiles(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = &context.empty_nav;
        for (context.tiles, context.tile_refs) |*tile, *ref| {
            sum +%= statusSum(nav.addTile(tile.bytes, .{}, 0, ref));
            sum +%= ref.*;
        }
        for (context.tiles) |*tile| sum +%= blobSum(tile.bytes);
        var index = context.tiles.len;
        while (index > 0) {
            index -= 1;
            var data: []align(4) u8 = &.{};
            sum +%= statusSum(nav.removeTile(context.tile_refs[index], &data));
            sum +%= data.len;
        }
        return sum;
    }

    fn baselineStoreAndRestoreTileState(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = context.reference_nav;
        for (context.tiles) |*tile| {
            const state = tile.state;
            sum +%= reference.ref_nav_mesh_store_tile_state(
                nav,
                tile.reference_tile,
                state.ptr,
                @intCast(state.len),
            );
            sum +%= blobSum(state);
            sum +%= reference.ref_nav_mesh_restore_tile_state(
                nav,
                @constCast(tile.reference_tile),
                state.ptr,
                @intCast(state.len),
            );
            sum +%= tile.reference_last_poly.flags;
        }
        return sum;
    }

    fn portStoreAndRestoreTileState(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = &context.nav;
        for (context.tiles) |*tile| {
            const state = tile.state;
            sum +%= statusSum(nav.storeTileState(tile.tile, state));
            sum +%= blobSum(state);
            sum +%= statusSum(nav.restoreTileState(tile.tile, state));
            sum +%= tile.last_poly.flags;
        }
        return sum;
    }

    fn baselineGetTileAndPolyByRef(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = context.reference_nav;
        var tile: ?*const reference.MeshTile = context.tiles[0].reference_tile;
        var poly: ?*const reference.Poly = context.tiles[0].reference_last_poly;
        for (context.poly_refs) |ref| {
            sum +%= reference.ref_nav_mesh_get_tile_and_poly_by_ref(nav, ref, &tile, &poly);
            sum +%= context.referenceHeader(tile.?).user_id;
            sum +%= poly.?.flags;
        }
        return sum;
    }

    fn portGetTileAndPolyByRef(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = &context.nav;
        var tile: *const detour.MeshTile = context.tiles[0].tile;
        var poly: *const detour.Poly = context.tiles[0].last_poly;
        for (context.poly_refs) |ref| {
            sum +%= statusSum(nav.getTileAndPolyByRef(ref, &tile, &poly));
            sum +%= tile.header.?.user_id;
            sum +%= poly.flags;
        }
        return sum;
    }

    fn baselineCallAlone(context: *Context) u64 {
        var sum: u64 = 0;
        for (context.validity_refs) |ref| sum +%= reference.ref_bench_identity(ref);
        return sum;
    }

    fn portCallAlone(context: *Context) u64 {
        var sum: u64 = 0;
        for (context.validity_refs) |ref| sum +%= ref;
        return sum;
    }

    fn baselineIsValidPolyRef(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = context.reference_nav;
        for (context.validity_refs) |ref| {
            sum +%= @intFromBool(reference.ref_nav_mesh_is_valid_poly_ref(nav, ref));
        }
        return sum;
    }

    fn portIsValidPolyRef(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = &context.nav;
        for (context.validity_refs) |ref| {
            sum +%= @intFromBool(nav.isValidPolyRef(ref));
        }
        return sum;
    }

    fn baselineSetAndGetPolyFlags(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = context.reference_nav;
        for (context.poly_refs, context.poly_flags) |ref, flags| {
            sum +%= reference.ref_nav_mesh_set_poly_flags(nav, ref, flags);
        }
        var flags: u16 = 0;
        for (context.poly_refs) |ref| {
            sum +%= reference.ref_nav_mesh_get_poly_flags(nav, ref, &flags);
            sum +%= flags;
        }
        return sum;
    }

    fn portSetAndGetPolyFlags(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = &context.nav;
        for (context.poly_refs, context.poly_flags) |ref, flags| {
            sum +%= statusSum(nav.setPolyFlags(ref, flags));
        }
        var flags: u16 = 0;
        for (context.poly_refs) |ref| {
            sum +%= statusSum(nav.getPolyFlags(ref, &flags));
            sum +%= flags;
        }
        return sum;
    }

    fn baselineSetAndGetPolyArea(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = context.reference_nav;
        for (context.poly_refs, context.poly_areas) |ref, area| {
            sum +%= reference.ref_nav_mesh_set_poly_area(nav, ref, area);
        }
        var area: u8 = 0;
        for (context.poly_refs) |ref| {
            sum +%= reference.ref_nav_mesh_get_poly_area(nav, ref, &area);
            sum +%= area;
        }
        return sum;
    }

    fn portSetAndGetPolyArea(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = &context.nav;
        for (context.poly_refs, context.poly_areas) |ref, area| {
            sum +%= statusSum(nav.setPolyArea(ref, area));
        }
        var area: u8 = 0;
        for (context.poly_refs) |ref| {
            sum +%= statusSum(nav.getPolyArea(ref, &area));
            sum +%= area;
        }
        return sum;
    }

    fn baselineCalcTileLoc(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = context.reference_nav;
        var x: i32 = 0;
        var y: i32 = 0;
        for (context.points) |*point| {
            reference.ref_nav_mesh_calc_tile_loc(nav, point, &x, &y);
            sum +%= intSum(x);
            sum +%= intSum(y);
        }
        return sum;
    }

    fn portCalcTileLoc(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = &context.nav;
        for (context.points) |point| {
            const location = nav.calcTileLoc(point);
            sum +%= intSum(location.x);
            sum +%= intSum(location.y);
        }
        return sum;
    }

    fn baselineGetTilesAt(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = context.reference_nav;
        var tiles: [tiles_at_count_max]?*const reference.MeshTile = @splat(null);
        for (context.tile_locations) |location| {
            const count = reference.ref_nav_mesh_get_tiles_at(
                nav,
                location.x,
                location.y,
                &tiles,
                tiles_at_count_max,
            );
            sum +%= intSum(count);
            for (tiles[0..@intCast(count)]) |tile| {
                sum +%= context.referenceHeader(tile.?).user_id;
            }
        }
        return sum;
    }

    fn portGetTilesAt(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = &context.nav;
        var tiles: [tiles_at_count_max]*const detour.MeshTile = @splat(context.tiles[0].tile);
        for (context.tile_locations) |location| {
            const count = nav.getTilesAt(location.x, location.y, &tiles);
            sum +%= count;
            for (tiles[0..count]) |tile| {
                sum +%= tile.header.?.user_id;
            }
        }
        return sum;
    }

    fn baselineGetTileAt(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = context.reference_nav;
        for (context.tile_locations) |location| {
            const found = reference.ref_nav_mesh_get_tile_at(nav, location.x, location.y, 0);
            if (found) |tile| {
                sum +%= 1;
                sum +%= context.referenceHeader(tile).user_id;
            }
        }
        return sum;
    }

    fn portGetTileAt(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = &context.nav;
        for (context.tile_locations) |location| {
            const found = nav.getTileAt(location.x, location.y, 0);
            if (found) |tile| {
                sum +%= 1;
                sum +%= tile.header.?.user_id;
            }
        }
        return sum;
    }

    fn baselineGetTileRefAt(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = context.reference_nav;
        for (context.tile_locations) |location| {
            sum +%= reference.ref_nav_mesh_get_tile_ref_at(nav, location.x, location.y, 0);
        }
        return sum;
    }

    fn portGetTileRefAt(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = &context.nav;
        for (context.tile_locations) |location| {
            sum +%= nav.getTileRefAt(location.x, location.y, 0);
        }
        return sum;
    }

    fn baselineGetTileByRef(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = context.reference_nav;
        for (context.located_tile_refs) |ref| {
            const found = reference.ref_nav_mesh_get_tile_by_ref(nav, ref);
            if (found) |tile| {
                sum +%= 1;
                sum +%= context.referenceHeader(tile).user_id;
            }
        }
        return sum;
    }

    fn portGetTileByRef(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = &context.nav;
        for (context.located_tile_refs) |ref| {
            const found = nav.getTileByRef(ref);
            if (found) |tile| {
                sum +%= 1;
                sum +%= tile.header.?.user_id;
            }
        }
        return sum;
    }

    fn baselineGetOffMeshConnectionPolyEndPoints(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = context.reference_nav;
        var start: [3]f32 = @splat(0);
        var end: [3]f32 = @splat(0);
        for (context.connection_steps) |step| {
            sum +%= reference.ref_nav_mesh_get_off_mesh_connection_poly_end_points(
                nav,
                step.prev_ref,
                step.poly_ref,
                &start,
                &end,
            );
            sum +%= pointSum(start);
            sum +%= pointSum(end);
        }
        return sum;
    }

    fn portGetOffMeshConnectionPolyEndPoints(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = &context.nav;
        var start: [3]f32 = @splat(0);
        var end: [3]f32 = @splat(0);
        for (context.connection_steps) |step| {
            sum +%= statusSum(nav.getOffMeshConnectionPolyEndPoints(
                step.prev_ref,
                step.poly_ref,
                &start,
                &end,
            ));
            sum +%= pointSum(start);
            sum +%= pointSum(end);
        }
        return sum;
    }

    fn baselineGetOffMeshConnectionByRef(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = context.reference_nav;
        for (context.connection_refs) |ref| {
            const found = reference.ref_nav_mesh_get_off_mesh_connection_by_ref(nav, ref);
            if (found) |connection| {
                sum +%= 1;
                sum +%= connection.user_id;
            }
        }
        return sum;
    }

    fn portGetOffMeshConnectionByRef(context: *Context) u64 {
        var sum: u64 = 0;
        const nav = &context.nav;
        for (context.connection_refs) |ref| {
            const found = nav.getOffMeshConnectionByRef(ref);
            if (found) |connection| {
                sum +%= 1;
                sum +%= connection.user_id;
            }
        }
        return sum;
    }

    fn baselineDataSwapEndian(context: *Context) u64 {
        var sum: u64 = 0;
        for (context.tiles) |*tile| {
            const bytes = tile.swap_bytes;
            // Twice: to the other byte order and back.
            for (0..2) |_| {
                sum +%= @intFromBool(
                    reference.ref_nav_mesh_data_swap_endian(bytes.ptr, @intCast(bytes.len)),
                );
                sum +%= blobSum(bytes);
            }
        }
        return sum;
    }

    fn portDataSwapEndian(context: *Context) u64 {
        var sum: u64 = 0;
        for (context.tiles) |*tile| {
            const bytes = tile.swap_bytes;
            for (0..2) |_| {
                sum +%= @intFromBool(detour.nav_mesh_builder.navMeshDataSwapEndian(bytes));
                sum +%= blobSum(bytes);
            }
        }
        return sum;
    }

    fn baselineHeaderSwapEndian(context: *Context) u64 {
        var sum: u64 = 0;
        for (0..header_swap_passes) |_| {
            for (context.tiles) |*tile| {
                const bytes = tile.swap_bytes;
                // Twice: to the other byte order and back.
                for (0..2) |_| {
                    sum +%= @intFromBool(
                        reference.ref_nav_mesh_header_swap_endian(bytes.ptr, @intCast(bytes.len)),
                    );
                    sum +%= std.mem.readInt(u32, bytes[poly_count_offset..][0..4], .little);
                }
            }
        }
        return sum;
    }

    fn portHeaderSwapEndian(context: *Context) u64 {
        var sum: u64 = 0;
        for (0..header_swap_passes) |_| {
            for (context.tiles) |*tile| {
                const bytes = tile.swap_bytes;
                for (0..2) |_| {
                    sum +%= @intFromBool(detour.nav_mesh_builder.navMeshHeaderSwapEndian(bytes));
                    sum +%= std.mem.readInt(u32, bytes[poly_count_offset..][0..4], .little);
                }
            }
        }
        return sum;
    }

    fn baselineIntersectSegmentPoly2d(context: *Context) u64 {
        var sum: u64 = 0;
        var t_min: f32 = 0;
        var t_max: f32 = 0;
        var seg_min: i32 = 0;
        var seg_max: i32 = 0;
        for (context.cases) |*case| {
            sum +%= @intFromBool(reference.ref_intersect_segment_poly_2d(
                &case.point,
                &case.far,
                case.polygon.floats(),
                @intCast(case.polygon.count),
                &t_min,
                &t_max,
                &seg_min,
                &seg_max,
            ));
            sum +%= floatSum(t_min);
            sum +%= floatSum(t_max);
            sum +%= intSum(seg_min);
            sum +%= intSum(seg_max);
        }
        return sum;
    }

    fn portIntersectSegmentPoly2d(context: *Context) u64 {
        var sum: u64 = 0;
        for (context.cases) |*case| {
            const hit = common.intersectSegmentPoly2d(
                case.point,
                case.far,
                case.polygon.corners(),
            );
            sum +%= @intFromBool(hit.hit);
            sum +%= floatSum(hit.t_min);
            sum +%= floatSum(hit.t_max);
            sum +%= intSum(hit.seg_min);
            sum +%= intSum(hit.seg_max);
        }
        return sum;
    }

    fn baselineDistancePointPolyEdgesSqr(context: *Context) u64 {
        var sum: u64 = 0;
        var edge_dist_sqr: [verts_per_polygon]f32 = @splat(0);
        var edge_t: [verts_per_polygon]f32 = @splat(0);
        for (context.cases) |*case| {
            sum +%= @intFromBool(reference.ref_distance_pt_poly_edges_sqr(
                &case.point,
                case.polygon.floats(),
                @intCast(case.polygon.count),
                &edge_dist_sqr,
                &edge_t,
            ));
            sum +%= floatSum(edge_dist_sqr[0]);
            sum +%= floatSum(edge_t[case.polygon.count - 1]);
        }
        return sum;
    }

    fn portDistancePointPolyEdgesSqr(context: *Context) u64 {
        var sum: u64 = 0;
        var edge_dist_sqr: [verts_per_polygon]f32 = @splat(0);
        var edge_t: [verts_per_polygon]f32 = @splat(0);
        for (context.cases) |*case| {
            sum +%= @intFromBool(common.distancePointPolyEdgesSqr(
                case.point,
                case.polygon.corners(),
                &edge_dist_sqr,
                &edge_t,
            ));
            sum +%= floatSum(edge_dist_sqr[0]);
            sum +%= floatSum(edge_t[case.polygon.count - 1]);
        }
        return sum;
    }

    fn baselinePointInPolygon(context: *Context) u64 {
        var sum: u64 = 0;
        for (context.cases) |*case| {
            sum +%= @intFromBool(reference.ref_point_in_polygon(
                &case.point,
                case.polygon.floats(),
                @intCast(case.polygon.count),
            ));
        }
        return sum;
    }

    fn portPointInPolygon(context: *Context) u64 {
        var sum: u64 = 0;
        for (context.cases) |*case| {
            sum +%= @intFromBool(common.pointInPolygon(case.point, case.polygon.corners()));
        }
        return sum;
    }

    fn baselineClosestHeightOnTriangle(context: *Context) u64 {
        var sum: u64 = 0;
        var height: f32 = 0;
        for (context.cases) |*case| {
            const verts = &case.polygon.verts;
            const found = reference.ref_closest_height_point_triangle(
                &case.triangle_point,
                &verts[0],
                &verts[1],
                &verts[2],
                &height,
            );
            if (found) {
                sum +%= 1;
                sum +%= floatSum(height);
            }
        }
        return sum;
    }

    fn portClosestHeightOnTriangle(context: *Context) u64 {
        var sum: u64 = 0;
        for (context.cases) |*case| {
            const verts = &case.polygon.verts;
            const found = common.closestHeightOnTriangle(
                case.triangle_point,
                verts[0],
                verts[1],
                verts[2],
            );
            if (found) |height| {
                sum +%= 1;
                sum +%= floatSum(height);
            }
        }
        return sum;
    }

    fn baselineClosestPointOnTriangle(context: *Context) u64 {
        var sum: u64 = 0;
        var closest: [3]f32 = @splat(0);
        for (context.cases) |*case| {
            const verts = &case.polygon.verts;
            reference.ref_closest_pt_point_triangle(
                &closest,
                &case.point,
                &verts[0],
                &verts[1],
                &verts[2],
            );
            sum +%= pointSum(closest);
        }
        return sum;
    }

    fn portClosestPointOnTriangle(context: *Context) u64 {
        var sum: u64 = 0;
        for (context.cases) |*case| {
            const verts = &case.polygon.verts;
            const closest = common.closestPointOnTriangle(
                case.point,
                verts[0],
                verts[1],
                verts[2],
            );
            sum +%= pointSum(closest);
        }
        return sum;
    }

    fn baselineOverlapPolyPoly2d(context: *Context) u64 {
        var sum: u64 = 0;
        for (context.cases) |*case| {
            sum +%= @intFromBool(reference.ref_overlap_poly_poly_2d(
                case.polygon.floats(),
                @intCast(case.polygon.count),
                case.other.floats(),
                @intCast(case.other.count),
            ));
        }
        return sum;
    }

    fn portOverlapPolyPoly2d(context: *Context) u64 {
        var sum: u64 = 0;
        for (context.cases) |*case| {
            sum +%= @intFromBool(common.overlapPolyPoly2d(
                case.polygon.corners(),
                case.other.corners(),
            ));
        }
        return sum;
    }

    fn baselineRandomPointInConvexPoly(context: *Context) u64 {
        var sum: u64 = 0;
        var areas: [verts_per_polygon]f32 = @splat(0);
        var point: [3]f32 = @splat(0);
        for (context.cases) |*case| {
            reference.ref_random_point_in_convex_poly(
                case.polygon.floats(),
                @intCast(case.polygon.count),
                &areas,
                case.s,
                case.t,
                &point,
            );
            sum +%= pointSum(point);
        }
        return sum;
    }

    fn portRandomPointInConvexPoly(context: *Context) u64 {
        var sum: u64 = 0;
        var areas: [verts_per_polygon]f32 = @splat(0);
        for (context.cases) |*case| {
            const point = common.randomPointInConvexPoly(
                case.polygon.corners(),
                &areas,
                case.s,
                case.t,
            );
            sum +%= pointSum(point);
        }
        return sum;
    }

    fn baselineDistancePointSegmentSqr2d(context: *Context) u64 {
        var sum: u64 = 0;
        var t: f32 = 0;
        for (context.cases) |*case| {
            const verts = &case.polygon.verts;
            const dist_sqr = reference.ref_distance_pt_seg_sqr_2d(
                &case.point,
                &verts[0],
                &verts[1],
                &t,
            );
            sum +%= floatSum(dist_sqr);
            sum +%= floatSum(t);
        }
        return sum;
    }

    fn portDistancePointSegmentSqr2d(context: *Context) u64 {
        var sum: u64 = 0;
        for (context.cases) |*case| {
            const verts = &case.polygon.verts;
            const distance = common.distancePointSegmentSqr2d(case.point, verts[0], verts[1]);
            sum +%= floatSum(distance.dist_sqr);
            sum +%= floatSum(distance.t);
        }
        return sum;
    }

    fn baselineIntersectSegmentSegment2d(context: *Context) u64 {
        var sum: u64 = 0;
        var s: f32 = 0;
        var t: f32 = 0;
        for (context.cases) |*case| {
            const verts = &case.polygon.verts;
            const found = reference.ref_intersect_seg_seg_2d(
                &case.point,
                &case.far,
                &verts[0],
                &verts[1],
                &s,
                &t,
            );
            if (found) {
                sum +%= 1;
                sum +%= floatSum(s);
                sum +%= floatSum(t);
            }
        }
        return sum;
    }

    fn portIntersectSegmentSegment2d(context: *Context) u64 {
        var sum: u64 = 0;
        for (context.cases) |*case| {
            const verts = &case.polygon.verts;
            const found = common.intersectSegmentSegment2d(
                case.point,
                case.far,
                verts[0],
                verts[1],
            );
            if (found) |hit| {
                sum +%= 1;
                sum +%= floatSum(hit.s);
                sum +%= floatSum(hit.t);
            }
        }
        return sum;
    }

    fn baselineCalcPolyCenter(context: *Context) u64 {
        var sum: u64 = 0;
        var center: [3]f32 = @splat(0);
        for (context.cases) |*case| {
            reference.ref_calc_poly_center(
                &center,
                &case.indices,
                @intCast(case.polygon.count),
                case.polygon.floats(),
            );
            sum +%= pointSum(center);
        }
        return sum;
    }

    fn portCalcPolyCenter(context: *Context) u64 {
        var sum: u64 = 0;
        for (context.cases) |*case| {
            const center = common.calcPolyCenter(
                case.indices[0..case.polygon.count],
                case.polygon.corners(),
            );
            sum +%= pointSum(center);
        }
        return sum;
    }
};

/// `call_count` items that go over `items` again and again; none when
/// `items` is empty.
fn repeated(comptime T: type, arena: std.mem.Allocator, items: []const T) ![]T {
    if (items.len == 0) return &.{};
    const result = try arena.alloc(T, call_count);
    for (result, 0..) |*item, index| item.* = items[index % items.len];
    return result;
}

/// Draws one geometry case from the mesh's ground polygons.
fn makeCase(random: std.Random, polygons: []const Polygon) GeometryCase {
    const polygon = polygons[random.uintLessThan(usize, polygons.len)];
    var other = polygons[random.uintLessThan(usize, polygons.len)];

    const center = polygon.center();
    const corner = polygon.verts[random.uintLessThan(u32, polygon.count)];
    const reach = random.float(f32) * 2.0;
    var point: [3]f32 = undefined;
    for (&point, center, corner) |*coordinate, from, to| {
        coordinate.* = from + (to - from) * reach;
    }

    const other_center = other.center();
    for (other.verts[0..other.count]) |*vert| {
        for (vert, other_center, point) |*coordinate, from, to| coordinate.* += to - from;
    }

    const far_x = point[0] + (random.float(f32) * 2.0 - 1.0) * segment_reach;
    const far_z = point[2] + (random.float(f32) * 2.0 - 1.0) * segment_reach;

    const u = random.float(f32) * 0.75;
    const v = random.float(f32) * 0.75;
    const a = polygon.verts[0];
    const b = polygon.verts[1];
    const c = polygon.verts[2];
    var triangle_point: [3]f32 = undefined;
    for (&triangle_point, a, b, c) |*coordinate, from, to_b, to_c| {
        coordinate.* = from + (to_b - from) * u + (to_c - from) * v;
    }

    return .{
        .polygon = polygon,
        .indices = .{ 0, 1, 2, 3, 4, 5 },
        .other = other,
        .point = point,
        .far = .{ far_x, point[1], far_z },
        .triangle_point = triangle_point,
        .s = random.float(f32),
        .t = random.float(f32),
    };
}

pub fn run(bench: *Bench) !void {
    try bench.group("Navigation mesh and tile builder");

    // The tiled level with its off-mesh connections: every tile's builder
    // input from Recast, and each side's mesh of all the tiles.
    var context: Context = undefined;
    try context.init(bench.gpa);
    defer context.deinit();

    const tile_count = context.tiles.len;
    const poly_count = context.poly_refs.len;

    // What reaching the original through its C interface costs, before any
    // work: a call that returns its argument against the same sum inline.
    // The original's side of every row pays it once a call.
    try bench.compare(
        .{
            .name = "calibration: one call through the C interface",
            .per = "call",
            .count = context.validity_refs.len,
        },
        &context,
        Context.baselineCallAlone,
        Context.portCallAlone,
    );

    // Every tile of the level built from its Recast output and freed.
    try bench.compare(
        .{ .name = "createNavMeshData", .per = "tile", .count = tile_count },
        &context,
        Context.baselineCreateNavMeshData,
        Context.portCreateNavMeshData,
    );
    // Every tile's bytes added to a mesh that holds no tile, each linked to
    // the tiles added before it, then every tile removed, the last first.
    // The bytes stay the caller's, and each side has its own copy of them.
    try bench.compare(
        .{ .name = "NavMesh.addTile + removeTile", .per = "tile", .count = tile_count },
        &context,
        Context.baselineAddAndRemoveTiles,
        Context.portAddAndRemoveTiles,
    );
    // Every tile of the full mesh: its polygons' flags and areas stored into
    // a buffer of the size the tile asks for, then restored from it.
    try bench.compare(
        .{
            .name = "NavMesh.storeTileState + restoreTileState",
            .per = "tile",
            .count = tile_count,
        },
        &context,
        Context.baselineStoreAndRestoreTileState,
        Context.portStoreAndRestoreTileState,
    );

    // The reference of every polygon of the full mesh, in random order.
    try bench.compare(
        .{ .name = "NavMesh.getTileAndPolyByRef", .per = "call", .count = poly_count },
        &context,
        Context.baselineGetTileAndPolyByRef,
        Context.portGetTileAndPolyByRef,
    );
    // The same references, about one in four with a salt its tile no longer
    // has, as a reference kept across the rebuild of its tile.
    try bench.compare(
        .{ .name = "NavMesh.isValidPolyRef", .per = "call", .count = poly_count },
        &context,
        Context.baselineIsValidPolyRef,
        Context.portIsValidPolyRef,
    );
    // Every polygon of the full mesh given random flags, then every
    // polygon's flags read: two calls a polygon.
    try bench.compare(
        .{
            .name = "NavMesh.setPolyFlags + getPolyFlags",
            .per = "call",
            .count = 2 * poly_count,
        },
        &context,
        Context.baselineSetAndGetPolyFlags,
        Context.portSetAndGetPolyFlags,
    );
    // The same with a random area id for every polygon.
    try bench.compare(
        .{ .name = "NavMesh.setPolyArea + getPolyArea", .per = "call", .count = 2 * poly_count },
        &context,
        Context.baselineSetAndGetPolyArea,
        Context.portSetAndGetPolyArea,
    );

    // Points on the level's ground.
    try bench.compare(
        .{ .name = "NavMesh.calcTileLoc", .per = "call", .count = call_count },
        &context,
        Context.baselineCalcTileLoc,
        Context.portCalcTileLoc,
    );
    // The tile grid cells of those points, each asked for its tiles (one on
    // this level, none where a point's cell has no ground), then for the
    // tile of layer 0 and for that tile's reference.
    try bench.compare(
        .{ .name = "NavMesh.getTilesAt", .per = "call", .count = call_count },
        &context,
        Context.baselineGetTilesAt,
        Context.portGetTilesAt,
    );
    try bench.compare(
        .{ .name = "NavMesh.getTileAt", .per = "call", .count = call_count },
        &context,
        Context.baselineGetTileAt,
        Context.portGetTileAt,
    );
    try bench.compare(
        .{ .name = "NavMesh.getTileRefAt", .per = "call", .count = call_count },
        &context,
        Context.baselineGetTileRefAt,
        Context.portGetTileRefAt,
    );
    // The references of the tiles in those cells; zero for a cell with none.
    try bench.compare(
        .{ .name = "NavMesh.getTileByRef", .per = "call", .count = call_count },
        &context,
        Context.baselineGetTileByRef,
        Context.portGetTileByRef,
    );

    // The level's off-mesh connections, each walked from every polygon it is
    // linked to, over and over to `call_count` walks.
    if (context.connection_steps.len != 0) try bench.compare(
        .{
            .name = "NavMesh.getOffMeshConnectionPolyEndPoints",
            .per = "call",
            .count = context.connection_steps.len,
        },
        &context,
        Context.baselineGetOffMeshConnectionPolyEndPoints,
        Context.portGetOffMeshConnectionPolyEndPoints,
    );
    // The polygons of those connections, over and over to `call_count`.
    if (context.connection_refs.len != 0) try bench.compare(
        .{
            .name = "NavMesh.getOffMeshConnectionByRef",
            .per = "call",
            .count = context.connection_refs.len,
        },
        &context,
        Context.baselineGetOffMeshConnectionByRef,
        Context.portGetOffMeshConnectionByRef,
    );

    // Every tile's bytes swapped to the other byte order and back, so they
    // end as they began. The header stays as it is: this swaps what follows
    // it.
    try bench.compare(
        .{
            .name = "nav_mesh_builder.navMeshDataSwapEndian",
            .per = "tile",
            .count = 2 * tile_count,
            .bytes = 2 * context.tile_bytes_total,
        },
        &context,
        Context.baselineDataSwapEndian,
        Context.portDataSwapEndian,
    );
    // Every tile's header swapped to the other byte order and back,
    // `header_swap_passes` times over.
    try bench.compare(
        .{
            .name = "nav_mesh_builder.navMeshHeaderSwapEndian",
            .per = "call",
            .count = 2 * header_swap_passes * tile_count,
        },
        &context,
        Context.baselineHeaderSwapEndian,
        Context.portHeaderSwapEndian,
    );

    // The geometry rows each run over the same `call_count` cases: a ground
    // polygon of the mesh in world space, a point near it (inside about as
    // often as outside) and a segment from that point up to `segment_reach`
    // long along x and z.
    //
    // The segment against the polygon: hits and misses.
    try bench.compare(
        .{ .name = "common.intersectSegmentPoly2d", .per = "call", .count = call_count },
        &context,
        Context.baselineIntersectSegmentPoly2d,
        Context.portIntersectSegmentPoly2d,
    );
    // The point against the polygon's edges.
    try bench.compare(
        .{ .name = "common.distancePointPolyEdgesSqr", .per = "call", .count = call_count },
        &context,
        Context.baselineDistancePointPolyEdgesSqr,
        Context.portDistancePointPolyEdgesSqr,
    );
    try bench.compare(
        .{ .name = "common.pointInPolygon", .per = "call", .count = call_count },
        &context,
        Context.baselinePointInPolygon,
        Context.portPointInPolygon,
    );
    // A point of the plane of the polygon's first three corners, over their
    // triangle about three times in four.
    try bench.compare(
        .{ .name = "common.closestHeightOnTriangle", .per = "call", .count = call_count },
        &context,
        Context.baselineClosestHeightOnTriangle,
        Context.portClosestHeightOnTriangle,
    );
    // The point against the triangle of the polygon's first three corners.
    try bench.compare(
        .{ .name = "common.closestPointOnTriangle", .per = "call", .count = call_count },
        &context,
        Context.baselineClosestPointOnTriangle,
        Context.portClosestPointOnTriangle,
    );
    // The polygon against another ground polygon of the mesh, moved so its
    // center is at the point: overlapping and apart.
    try bench.compare(
        .{ .name = "common.overlapPolyPoly2d", .per = "call", .count = call_count },
        &context,
        Context.baselineOverlapPolyPoly2d,
        Context.portOverlapPolyPoly2d,
    );
    // The polygon with two random numbers from 0 to 1.
    try bench.compare(
        .{ .name = "common.randomPointInConvexPoly", .per = "call", .count = call_count },
        &context,
        Context.baselineRandomPointInConvexPoly,
        Context.portRandomPointInConvexPoly,
    );
    // The point against the polygon's first edge.
    try bench.compare(
        .{ .name = "common.distancePointSegmentSqr2d", .per = "call", .count = call_count },
        &context,
        Context.baselineDistancePointSegmentSqr2d,
        Context.portDistancePointSegmentSqr2d,
    );
    // The segment against the polygon's first edge.
    try bench.compare(
        .{ .name = "common.intersectSegmentSegment2d", .per = "call", .count = call_count },
        &context,
        Context.baselineIntersectSegmentSegment2d,
        Context.portIntersectSegmentSegment2d,
    );
    // The polygon's corners by index.
    try bench.compare(
        .{ .name = "common.calcPolyCenter", .per = "call", .count = call_count },
        &context,
        Context.baselineCalcPolyCenter,
        Context.portCalcPolyCenter,
    );
}
