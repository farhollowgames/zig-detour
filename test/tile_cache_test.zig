//! Differential tests of the tile cache: every public function of `TileCache`
//! against the original, on layers packed from a real level, on malformed
//! tiles and under a seeded mix of every call.

const std = @import("std");
const detour = @import("detour");
const support = @import("support.zig");

const reference = support.reference;
const reference_recast = support.reference_recast;
const reference_tile_cache = support.reference_tile_cache;
const testing = std.testing;

const tile_cache = detour.tile_cache;
const tile_cache_builder = detour.tile_cache_builder;
const CompressedTile = tile_cache.CompressedTile;
const LayerHeader = tile_cache_builder.LayerHeader;
const MeshTile = detour.MeshTile;
const NavMesh = detour.NavMesh;
const Obstacle = tile_cache.Obstacle;
const Status = detour.Status;
const TileCache = detour.TileCache;

// The original's status words.
const success_bits: u32 = 1 << 30;
const failure_bits: u32 = 1 << 31;
const out_of_memory_bits: u32 = failure_bits | (1 << 2);
const invalid_param_bits: u32 = failure_bits | (1 << 3);
const buffer_too_small_bits: u32 = failure_bits | (1 << 4);

/// Cells along one side of a tile, and what the level is cut into.
const tile_size = 48;
const cell_size: f32 = 0.3;
const cell_height: f32 = 0.2;
const header_size = @sizeOf(LayerHeader);
/// Both sides look at no more than this many layers of one cell.
const layers_per_cell_max = 32;
/// Both sides queue no more than this many obstacle requests and tile
/// rebuilds.
const request_count_max = 64;
const update_count_max = 64;
/// What a reference slot holds before a call that may leave it alone.
const untouched_ref: u32 = 0xdeadbeef;

// ---------------------------------------------------------------------------
// The compressor: stores bytes as they are, the same on both sides
// ---------------------------------------------------------------------------

var compressor_anchor: u8 = 0;

fn portMaxCompressedSize(_: *anyopaque, buffer_size: i32) i32 {
    return buffer_size;
}

fn portCompress(
    _: *anyopaque,
    buffer: []const u8,
    compressed: []u8,
    compressed_size: *i32,
) Status {
    if (buffer.len > compressed.len) return .failure;
    @memcpy(compressed[0..buffer.len], buffer);
    compressed_size.* = @intCast(buffer.len);
    return .success;
}

fn portDecompress(
    _: *anyopaque,
    compressed: []const u8,
    buffer: []u8,
    buffer_size: *i32,
) Status {
    if (compressed.len > buffer.len) return .failure;
    @memcpy(buffer[0..compressed.len], compressed);
    buffer_size.* = @intCast(compressed.len);
    return .success;
}

fn portCompressor() detour.Compressor {
    return .{
        .ptr = &compressor_anchor,
        .vtable = &.{
            .maxCompressedSize = portMaxCompressedSize,
            .compress = portCompress,
            .decompress = portDecompress,
        },
    };
}

fn referenceMaxCompressedSize(_: ?*anyopaque, buffer_size: i32) callconv(.c) i32 {
    return buffer_size;
}

fn referenceCompress(
    _: ?*anyopaque,
    buffer: [*]const u8,
    buffer_size: i32,
    compressed: [*]u8,
    max_compressed_size: i32,
    compressed_size: *i32,
) callconv(.c) u32 {
    if (buffer_size > max_compressed_size) return failure_bits;
    const size: usize = @intCast(buffer_size);
    @memcpy(compressed[0..size], buffer[0..size]);
    compressed_size.* = buffer_size;
    return success_bits;
}

fn referenceDecompress(
    _: ?*anyopaque,
    compressed: [*]const u8,
    compressed_size: i32,
    buffer: [*]u8,
    max_buffer_size: i32,
    buffer_size: *i32,
) callconv(.c) u32 {
    if (compressed_size > max_buffer_size) return failure_bits;
    const size: usize = @intCast(compressed_size);
    @memcpy(buffer[0..size], compressed[0..size]);
    buffer_size.* = compressed_size;
    return success_bits;
}

fn createReferenceCompressor() !*reference_tile_cache.Compressor {
    return reference_tile_cache.ref_tc_compressor_create(
        referenceMaxCompressedSize,
        referenceCompress,
        referenceDecompress,
        null,
    ) orelse error.OutOfMemory;
}

// ---------------------------------------------------------------------------
// The mesh process: the same retagging on both sides, with a record of what
// each side handed it
// ---------------------------------------------------------------------------

/// What one side handed its mesh process on the latest call, and how many
/// calls there were.
const ProcessRecord = struct {
    const seen_max = 1024;

    calls: u32 = 0,
    /// Tile x, y and layer, vertex count, polygon count, vertices a polygon.
    integers: [6]i32 = @splat(0),
    /// Bounds, walkable height, radius and climb, cell size and height.
    floats: [11]f32 = @splat(0.0),
    build_bv_tree: bool = false,
    seen: usize = 0,
    areas: [seen_max]u8 = @splat(0),
    flags: [seen_max]u16 = @splat(0),

    fn note(
        self: *ProcessRecord,
        integers: [6]i32,
        floats: [11]f32,
        build_bv_tree: bool,
        areas: []const u8,
        flags: []const u16,
    ) void {
        self.calls += 1;
        self.integers = integers;
        self.floats = floats;
        self.build_bv_tree = build_bv_tree;
        self.seen = @min(areas.len, seen_max);
        @memcpy(self.areas[0..self.seen], areas[0..self.seen]);
        @memcpy(self.flags[0..self.seen], flags[0..self.seen]);
    }

    fn expectSame(expected: *const ProcessRecord, actual: *const ProcessRecord) !void {
        errdefer std.debug.print("  in what the mesh process was handed\n", .{});
        try testing.expectEqual(expected.calls, actual.calls);
        try testing.expectEqualSlices(i32, &expected.integers, &actual.integers);
        try support.expectFloats(&expected.floats, &actual.floats);
        try testing.expectEqual(expected.build_bv_tree, actual.build_bv_tree);
        try testing.expectEqual(expected.seen, actual.seen);
        try testing.expectEqualSlices(
            u8,
            expected.areas[0..expected.seen],
            actual.areas[0..actual.seen],
        );
        try testing.expectEqualSlices(
            u16,
            expected.flags[0..expected.seen],
            actual.flags[0..actual.seen],
        );
    }
};

/// Turns the walkable area into ground and gives each area its flags, as the
/// demo's process does.
fn retag(poly_areas: []u8, poly_flags: []u16) void {
    for (poly_areas, poly_flags) |*area, *flags| {
        if (area.* == 63) area.* = 0;
        switch (area.*) {
            0, 2, 4 => flags.* = 0x01,
            1 => flags.* = 0x02,
            3 => flags.* = 0x01 | 0x04,
            else => {},
        }
    }
}

fn joinFloats(bounds_min: [3]f32, bounds_max: [3]f32, rest: [5]f32) [11]f32 {
    var floats: [11]f32 = undefined;
    floats[0..3].* = bounds_min;
    floats[3..6].* = bounds_max;
    floats[6..11].* = rest;
    return floats;
}

fn portProcess(
    ptr: *anyopaque,
    params: *detour.NavMeshCreateParams,
    poly_areas: []u8,
    poly_flags: []u16,
) void {
    const world: *World = @ptrCast(@alignCast(ptr));
    const count: usize = @intCast(params.poly_count);
    world.record.note(
        .{
            @intCast(params.tile_x),
            @intCast(params.tile_y),
            @intCast(params.tile_layer),
            @intCast(params.vert_count),
            @intCast(params.poly_count),
            @intCast(params.vertices_per_polygon),
        },
        joinFloats(params.bounds_min, params.bounds_max, .{
            params.walkable_height,
            params.walkable_radius,
            params.walkable_climb,
            params.cell_size,
            params.cell_height,
        }),
        params.build_bv_tree,
        poly_areas[0..count],
        poly_flags[0..count],
    );
    retag(poly_areas[0..count], poly_flags[0..count]);

    const connections = world.connections;
    const connection_count = connections.radii.len;
    const points: [*]const [2][3]f32 = @ptrCast(connections.verts.ptr);
    params.off_mesh_con_verts = points[0..connection_count];
    params.off_mesh_con_rad = connections.radii;
    params.off_mesh_con_flags = connections.flags;
    params.off_mesh_con_areas = connections.areas;
    params.off_mesh_con_dir = connections.directions;
    params.off_mesh_con_user_id = connections.user_ids;
    params.off_mesh_con_count = @intCast(connection_count);
}

fn referenceProcess(
    user: ?*anyopaque,
    params: *reference.NavMeshCreateParams,
    poly_areas: [*]u8,
    poly_flags: [*]u16,
) callconv(.c) void {
    const world: *World = @ptrCast(@alignCast(user.?));
    const count: usize = @intCast(params.poly_count);
    world.reference_record.note(
        .{
            params.tile_x,
            params.tile_y,
            params.tile_layer,
            params.vert_count,
            params.poly_count,
            params.nvp,
        },
        joinFloats(params.bmin, params.bmax, .{
            params.walkable_height,
            params.walkable_radius,
            params.walkable_climb,
            params.cs,
            params.ch,
        }),
        params.build_bv_tree,
        poly_areas[0..count],
        poly_flags[0..count],
    );
    retag(poly_areas[0..count], poly_flags[0..count]);

    const connections = world.connections;
    params.off_mesh_con_verts = connections.verts.ptr;
    params.off_mesh_con_rad = connections.radii.ptr;
    params.off_mesh_con_flags = connections.flags.ptr;
    params.off_mesh_con_areas = connections.areas.ptr;
    params.off_mesh_con_dir = connections.directions.ptr;
    params.off_mesh_con_user_id = connections.user_ids.ptr;
    params.off_mesh_con_count = @intCast(connections.radii.len);
}

// ---------------------------------------------------------------------------
// The level and its packed layers
// ---------------------------------------------------------------------------

/// One layer of one cell, packed as the cache takes it: a header and the
/// stored grids.
const PackedLayer = struct {
    tile_x: i32,
    tile_y: i32,
    layer: i32,
    bytes: []u8,
};

/// The test level cut into tiles, with every layer of every cell packed once
/// by the original's builder.
const Fixture = struct {
    gpa: std.mem.Allocator,
    level: support.Level,
    layers: []PackedLayer,

    fn init(gpa: std.mem.Allocator, off_mesh_connection_count: u32) !Fixture {
        var config = support.solo_config;
        config.tile_size = tile_size;
        var level: support.Level = try .init(gpa, support.nav_test_obj, config, .{
            .off_mesh_connection_count = off_mesh_connection_count,
            .seed = 0x7ac4e,
        });
        errdefer level.deinit();

        const compressor = try createReferenceCompressor();
        defer reference_tile_cache.ref_tc_compressor_destroy(compressor);

        var layers: std.ArrayList(PackedLayer) = .empty;
        errdefer {
            for (layers.items) |layer| gpa.free(layer.bytes);
            layers.deinit(gpa);
        }
        var tile_y: i32 = 0;
        while (tile_y < level.grid.height) : (tile_y += 1) {
            var tile_x: i32 = 0;
            while (tile_x < level.grid.width) : (tile_x += 1) {
                try packCell(gpa, &level, compressor, tile_x, tile_y, &layers);
            }
        }
        return .{ .gpa = gpa, .level = level, .layers = try layers.toOwnedSlice(gpa) };
    }

    fn deinit(self: *Fixture) void {
        for (self.layers) |layer| self.gpa.free(layer.bytes);
        self.gpa.free(self.layers);
        self.level.deinit();
        self.* = undefined;
    }

    fn packCell(
        gpa: std.mem.Allocator,
        level: *const support.Level,
        compressor: *reference_tile_cache.Compressor,
        tile_x: i32,
        tile_y: i32,
        layers: *std.ArrayList(PackedLayer),
    ) !void {
        var built: [layers_per_cell_max]reference_recast.RecastLayer = undefined;
        var owner: ?*reference_recast.LayerOwner = null;
        const built_count = reference_recast.ref_recast_build_layers(
            level.mesh.verts.ptr,
            level.mesh.vertexCount(),
            level.mesh.tris.ptr,
            level.mesh.triangleCount(),
            &level.config,
            &level.mesh.bounds_min,
            &level.mesh.bounds_max,
            tile_x,
            tile_y,
            &built,
            layers_per_cell_max,
            &owner,
        );
        defer reference_recast.ref_recast_free_layers(owner);
        if (built_count <= 0) return;

        const count: usize = @intCast(@min(built_count, layers_per_cell_max));
        for (built[0..count], 0..) |*layer, i| {
            var header = std.mem.zeroes(reference_tile_cache.TileCacheLayerHeader);
            header.magic = tile_cache_builder.tile_cache_magic;
            header.version = tile_cache_builder.tile_cache_version;
            header.tx = tile_x;
            header.ty = tile_y;
            header.tlayer = @intCast(i);
            header.bmin = layer.bmin;
            header.bmax = layer.bmax;
            header.hmin = layer.hmin;
            header.hmax = layer.hmax;
            header.width = layer.width;
            header.height = layer.height;
            header.minx = layer.minx;
            header.maxx = layer.maxx;
            header.miny = layer.miny;
            header.maxy = layer.maxy;

            var data: ?[*]u8 = null;
            var data_size: i32 = 0;
            const status = reference_tile_cache.ref_tcb_build_layer(
                compressor,
                &header,
                layer.heights.?,
                layer.areas.?,
                layer.cons.?,
                &data,
                &data_size,
            );
            defer reference_tile_cache.ref_tcb_free_data(data);
            if ((status & success_bits) == 0) return error.LayerNotPacked;

            const size: usize = @intCast(data_size);
            const bytes = try gpa.dupe(u8, data.?[0..size]);
            errdefer gpa.free(bytes);
            try layers.append(gpa, .{
                .tile_x = tile_x,
                .tile_y = tile_y,
                .layer = @intCast(i),
                .bytes = bytes,
            });
        }
    }

    /// Readies `world` over this level. The world must stay where it is.
    fn initWorld(self: *const Fixture, world: *World, options: WorldOptions) !void {
        try world.init(
            self.gpa,
            self.level.mesh.bounds_min,
            &self.level.off_mesh_connections,
            options,
        );
    }

    /// How many bits of tile index a navigation mesh needs to hold every
    /// layer, and no fewer than six.
    fn navTileBits(self: *const Fixture) u5 {
        var bits: u5 = 6;
        while ((@as(usize, 1) << bits) < self.layers.len) bits += 1;
        return bits;
    }

    /// How many layers the cell at (`tile_x`, `tile_y`) has.
    fn layerCountAt(self: *const Fixture, tile_x: i32, tile_y: i32) usize {
        var count: usize = 0;
        for (self.layers) |layer| {
            count += @intFromBool(layer.tile_x == tile_x and layer.tile_y == tile_y);
        }
        return count;
    }

    /// The layer with the most stored cells that hold ground: the one whose
    /// rebuild does the most work.
    fn fullestLayer(self: *const Fixture) *const PackedLayer {
        var best: *const PackedLayer = &self.layers[0];
        var best_count: usize = 0;
        for (self.layers) |*layer| {
            const header: *align(1) const LayerHeader = @ptrCast(layer.bytes.ptr);
            const grid_size = @as(usize, header.width) * header.height;
            // The areas follow the heights.
            const areas = layer.bytes[header_size + grid_size ..][0..grid_size];
            var count: usize = 0;
            for (areas) |area| count += @intFromBool(area != 0);
            if (count > best_count) {
                best_count = count;
                best = layer;
            }
        }
        return best;
    }
};

/// The header at the front of a packed layer.
fn headerOf(bytes: []u8) *align(1) LayerHeader {
    return @ptrCast(bytes.ptr);
}

fn cacheParams(
    origin: [3]f32,
    tile_count_max: i32,
    obstacle_count_max: i32,
) tile_cache.TileCacheParams {
    return .{
        .origin = origin,
        .cell_size = cell_size,
        .cell_height = cell_height,
        .width = tile_size,
        .height = tile_size,
        .walkable_height = 2.0,
        .walkable_radius = 0.6,
        .walkable_climb = 0.9,
        .max_simplification_error = 1.3,
        .tile_count_max = tile_count_max,
        .obstacle_count_max = obstacle_count_max,
    };
}

fn referenceParams(params: *const tile_cache.TileCacheParams) reference_tile_cache.TileCacheParams {
    return .{
        .orig = params.origin,
        .cs = params.cell_size,
        .ch = params.cell_height,
        .width = params.width,
        .height = params.height,
        .walkable_height = params.walkable_height,
        .walkable_radius = params.walkable_radius,
        .walkable_climb = params.walkable_climb,
        .max_simplification_error = params.max_simplification_error,
        .max_tiles = params.tile_count_max,
        .max_obstacles = params.obstacle_count_max,
    };
}

// ---------------------------------------------------------------------------
// The world: one cache and one navigation mesh on each side, driven by the
// same calls
// ---------------------------------------------------------------------------

const WorldOptions = struct {
    tile_count_max: i32 = 64,
    obstacle_count_max: i32 = 128,
    /// The navigation meshes hold two to this power tiles.
    nav_tile_bits: u5 = 6,
    with_process: bool = true,
};

/// A tile both caches hold, with each side's copy of its bytes.
const Held = struct {
    ref: u32,
    /// Whether the caches free the copies themselves.
    owned: bool,
    bytes: ?[]u8,
    reference_bytes: ?[*]u8,
};

/// An obstacle to add, in the terms both sides take.
const ObstacleRequest = union(enum) {
    cylinder: struct { position: [3]f32, radius: f32, height: f32 },
    box: struct { bounds_min: [3]f32, bounds_max: [3]f32 },
    oriented_box: struct { center: [3]f32, half_extents: [3]f32, y_radians: f32 },
};

/// Both sides' caches and navigation meshes. Initialised in place and never
/// moved: the caches keep pointers into it.
const World = struct {
    gpa: std.mem.Allocator,
    connections: *const support.OffMeshConnections,
    params: tile_cache.TileCacheParams,

    scratch: detour.BuildAllocator,
    cache: TileCache,
    nav: NavMesh,
    record: ProcessRecord,

    reference_alloc: *reference_tile_cache.TileCacheAlloc,
    reference_compressor: *reference_tile_cache.Compressor,
    reference_process: ?*reference_tile_cache.MeshProcess,
    reference_cache: *reference_tile_cache.TileCache,
    reference_nav: *reference.NavMesh,
    reference_record: ProcessRecord,

    held: std.ArrayList(Held),
    update_calls: u32,
    /// What the port answered the latest obstacle request with.
    last_status: Status,

    fn init(
        self: *World,
        gpa: std.mem.Allocator,
        origin: [3]f32,
        connections: *const support.OffMeshConnections,
        options: WorldOptions,
    ) !void {
        const reference_alloc = reference_tile_cache.ref_tc_alloc_create_default() orelse
            return error.OutOfMemory;
        errdefer reference_tile_cache.ref_tc_alloc_destroy(reference_alloc);
        const reference_compressor = try createReferenceCompressor();
        errdefer reference_tile_cache.ref_tc_compressor_destroy(reference_compressor);
        const reference_process: ?*reference_tile_cache.MeshProcess = if (options.with_process)
            reference_tile_cache.ref_tc_mesh_process_create(referenceProcess, self) orelse
                return error.OutOfMemory
        else
            null;
        errdefer reference_tile_cache.ref_tc_mesh_process_destroy(reference_process);
        const reference_cache = reference_tile_cache.ref_tc_create() orelse
            return error.OutOfMemory;
        errdefer reference_tile_cache.ref_tc_destroy(reference_cache);
        const reference_nav = reference.ref_nav_mesh_alloc() orelse return error.OutOfMemory;
        errdefer reference.ref_nav_mesh_free(reference_nav);

        self.* = .{
            .gpa = gpa,
            .connections = connections,
            .params = cacheParams(origin, options.tile_count_max, options.obstacle_count_max),
            .scratch = .{ .allocator = gpa },
            .cache = .empty,
            .nav = .empty,
            .record = .{},
            .reference_alloc = reference_alloc,
            .reference_compressor = reference_compressor,
            .reference_process = reference_process,
            .reference_cache = reference_cache,
            .reference_nav = reference_nav,
            .reference_record = .{},
            .held = .empty,
            .update_calls = 0,
            .last_status = .success,
        };
        errdefer self.cache.deinit();
        errdefer self.nav.deinit();

        const tile_side = @as(f32, tile_size) * cell_size;
        const nav_tile_count = @as(i32, 1) << options.nav_tile_bits;
        const nav_poly_count = @as(i32, 1) << (22 - options.nav_tile_bits);
        const expected_nav_status = reference.ref_nav_mesh_init(reference_nav, &.{
            .orig = self.params.origin,
            .tile_width = tile_side,
            .tile_height = tile_side,
            .max_tiles = nav_tile_count,
            .max_polys = nav_poly_count,
        });
        const nav_status = self.nav.init(gpa, &.{
            .origin = self.params.origin,
            .tile_width = tile_side,
            .tile_height = tile_side,
            .tile_count_max = nav_tile_count,
            .poly_count_max = nav_poly_count,
        });
        try support.expectStatus(expected_nav_status, nav_status);
        try testing.expect(nav_status.succeeded);

        const expected_params = referenceParams(&self.params);
        const expected_status = reference_tile_cache.ref_tc_init(
            reference_cache,
            &expected_params,
            reference_alloc,
            reference_compressor,
            reference_process,
        );
        const process: ?detour.MeshProcess = if (options.with_process)
            .{ .ptr = self, .vtable = &.{ .process = portProcess } }
        else
            null;
        const status = self.cache.init(gpa, &self.params, &self.scratch, portCompressor(), process);
        try support.expectStatus(expected_status, status);
        try testing.expect(status.succeeded);
    }

    fn deinit(self: *World) void {
        self.cache.deinit();
        reference_tile_cache.ref_tc_destroy(self.reference_cache);
        for (self.held.items) |held| {
            if (held.owned) continue;
            if (held.bytes) |bytes| self.gpa.free(bytes);
            if (held.reference_bytes) |bytes| reference.ref_free(bytes);
        }
        self.held.deinit(self.gpa);
        self.nav.deinit();
        reference.ref_nav_mesh_free(self.reference_nav);
        reference_tile_cache.ref_tc_mesh_process_destroy(self.reference_process);
        reference_tile_cache.ref_tc_compressor_destroy(self.reference_compressor);
        reference_tile_cache.ref_tc_alloc_destroy(self.reference_alloc);
        self.* = undefined;
    }

    fn tileCountMax(self: *const World) u32 {
        return @intCast(self.params.tile_count_max);
    }

    /// The entry of the port's cache that holds exactly `bytes`.
    fn portTileHolding(self: *const World, bytes: []const u8) ?*const CompressedTile {
        for (self.cache.tiles) |*tile| {
            if (tile.header == null) continue;
            if (tile.data.ptr == bytes.ptr and tile.data.len == bytes.len) return tile;
        }
        return null;
    }

    fn heldIndex(self: *const World, ref: u32) ?usize {
        for (self.held.items, 0..) |held, i| {
            if (held.ref == ref) return i;
        }
        return null;
    }

    /// Whether `ref` names an entry that holds no tile, with the entry's
    /// salt. The original takes such a reference for a tile.
    fn namesUnusedEntry(self: *const World, ref: u32) bool {
        if (ref == 0) return false;
        const index = self.cache.decodeTileIdTile(ref);
        if (index >= self.tileCountMax()) return false;
        const tile = &self.cache.tiles[index];
        return tile.header == null and tile.salt == self.cache.decodeTileIdSalt(ref);
    }

    /// Gives a copy of `bytes` to both caches. Returns the reference of the
    /// new tile, or null when both refused it.
    fn addBytes(self: *World, bytes: []const u8, flags: u8, want_ref: bool) !?u32 {
        const gpa = self.gpa;
        try self.held.ensureUnusedCapacity(gpa, 1);

        const copy = try gpa.dupe(u8, bytes);
        var copy_kept = false;
        defer if (!copy_kept) gpa.free(copy);
        const raw = reference.ref_alloc(@intCast(bytes.len), .perm) orelse
            return error.OutOfMemory;
        const reference_copy: [*]u8 = @ptrCast(raw);
        var reference_kept = false;
        defer if (!reference_kept) reference.ref_free(reference_copy);
        @memcpy(reference_copy[0..bytes.len], bytes);

        var expected_ref: u32 = untouched_ref;
        var ref: u32 = untouched_ref;
        const expected = reference_tile_cache.ref_tc_add_tile(
            self.reference_cache,
            reference_copy,
            @intCast(bytes.len),
            flags,
            if (want_ref) &expected_ref else null,
        );
        const status = self.cache.addTile(copy, flags, if (want_ref) &ref else null);
        reference_kept = (expected & success_bits) != 0;
        copy_kept = status.succeeded;

        const tile = if (copy_kept) self.portTileHolding(copy) else null;
        const held_ref = self.cache.getTileRef(tile);
        if (copy_kept or reference_kept) self.held.appendAssumeCapacity(.{
            .ref = held_ref,
            .owned = (flags & tile_cache.compressed_tile_free_data) != 0,
            .bytes = if (copy_kept) copy else null,
            .reference_bytes = if (reference_kept) reference_copy else null,
        });

        try support.expectStatus(expected, status);
        try testing.expectEqual(expected_ref, ref);
        if (!copy_kept) return null;

        // Both caches point into the bytes they were given.
        try testing.expect(tile != null);
        if (want_ref) try testing.expectEqual(held_ref, ref);
        const expected_tile = reference_tile_cache.ref_tc_get_tile_by_ref(
            self.reference_cache,
            held_ref,
        ) orelse return error.TestUnexpectedResult;
        try testing.expectEqual(
            @intFromPtr(reference_copy),
            @intFromPtr(reference_tile_cache.ref_tc_tile_data(expected_tile)),
        );
        return held_ref;
    }

    /// Takes the tile `ref` names out of both caches; `want_data` asks for
    /// its bytes back. Returns whether a tile went.
    fn removeRef(self: *World, ref: u32, want_data: bool) !bool {
        var sentinel: [1]u8 = .{0xa5};
        var data: []u8 = &sentinel;
        var expected_data: ?[*]u8 = &sentinel;
        var expected_size: i32 = -77;

        if (self.namesUnusedEntry(ref)) {
            // The original reads the header of the entry here, and there is
            // none. The port refuses the reference.
            const status = self.cache.removeTile(ref, if (want_data) &data else null);
            try support.expectStatus(invalid_param_bits, status);
            try testing.expectEqual(@intFromPtr(&sentinel), @intFromPtr(data.ptr));
            return false;
        }

        const expected = reference_tile_cache.ref_tc_remove_tile(
            self.reference_cache,
            ref,
            if (want_data) &expected_data else null,
            if (want_data) &expected_size else null,
        );
        const status = self.cache.removeTile(ref, if (want_data) &data else null);
        try support.expectStatus(expected, status);
        if (!status.succeeded) {
            // A refused call writes nothing back.
            try testing.expectEqual(@intFromPtr(&sentinel), @intFromPtr(expected_data));
            try testing.expectEqual(@as(i32, -77), expected_size);
            try testing.expectEqual(@intFromPtr(&sentinel), @intFromPtr(data.ptr));
            try testing.expectEqual(@as(usize, 1), data.len);
            return false;
        }

        const index = self.heldIndex(ref) orelse return error.TestUnexpectedResult;
        const held = self.held.swapRemove(index);
        defer if (!held.owned) {
            if (held.bytes) |bytes| self.gpa.free(bytes);
            if (held.reference_bytes) |bytes| reference.ref_free(bytes);
        };
        if (!want_data) return true;
        if (held.owned) {
            // The caches freed the bytes and hand back nothing.
            try testing.expectEqual(@as(usize, 0), @intFromPtr(expected_data));
            try testing.expectEqual(@as(i32, 0), expected_size);
            try testing.expectEqual(@as(usize, 0), data.len);
        } else {
            const bytes = held.bytes.?;
            try testing.expectEqual(
                @intFromPtr(held.reference_bytes.?),
                @intFromPtr(expected_data),
            );
            try testing.expectEqual(bytes.len, @as(usize, @intCast(expected_size)));
            try testing.expectEqual(@intFromPtr(bytes.ptr), @intFromPtr(data.ptr));
            try testing.expectEqual(bytes.len, data.len);
        }
        return true;
    }

    /// Builds every layer of one cell into both navigation meshes.
    fn buildAt(self: *World, tile_x: i32, tile_y: i32) !Status {
        const expected = reference_tile_cache.ref_tc_build_nav_mesh_tiles_at(
            self.reference_cache,
            tile_x,
            tile_y,
            self.reference_nav,
        );
        const status = self.cache.buildNavMeshTilesAt(tile_x, tile_y, &self.nav);
        try support.expectStatus(expected, status);
        return status;
    }

    /// Builds the tile `ref` names into both navigation meshes.
    fn buildRef(self: *World, ref: u32) !Status {
        if (self.cache.decodeTileIdTile(ref) == self.tileCountMax()) {
            // The original lets the index one past the table through and
            // reads the entry after the last one. The port refuses it.
            const status = self.cache.buildNavMeshTile(ref, &self.nav);
            try support.expectStatus(invalid_param_bits, status);
            return status;
        }
        const expected = reference_tile_cache.ref_tc_build_nav_mesh_tile(
            self.reference_cache,
            ref,
            self.reference_nav,
        );
        const status = self.cache.buildNavMeshTile(ref, &self.nav);
        try support.expectStatus(expected, status);
        return status;
    }

    /// Asks both caches for an obstacle. Returns its reference when
    /// `want_ref` asked for one and both took the request.
    fn addObstacle(self: *World, request: ObstacleRequest, want_ref: bool) !?u32 {
        var expected_ref: u32 = untouched_ref;
        var ref: u32 = untouched_ref;
        const expected_result: ?*u32 = if (want_ref) &expected_ref else null;
        const result: ?*u32 = if (want_ref) &ref else null;
        const cache = &self.cache;
        const reference_cache = self.reference_cache;
        var expected: u32 = undefined;
        var status: Status = undefined;
        switch (request) {
            .cylinder => |cylinder| {
                expected = reference_tile_cache.ref_tc_add_obstacle(
                    reference_cache,
                    &cylinder.position,
                    cylinder.radius,
                    cylinder.height,
                    expected_result,
                );
                status = cache.addObstacle(
                    cylinder.position,
                    cylinder.radius,
                    cylinder.height,
                    result,
                );
            },
            .box => |box| {
                expected = reference_tile_cache.ref_tc_add_box_obstacle(
                    reference_cache,
                    &box.bounds_min,
                    &box.bounds_max,
                    expected_result,
                );
                status = cache.addBoxObstacle(box.bounds_min, box.bounds_max, result);
            },
            .oriented_box => |oriented| {
                expected = reference_tile_cache.ref_tc_add_oriented_box_obstacle(
                    reference_cache,
                    &oriented.center,
                    &oriented.half_extents,
                    oriented.y_radians,
                    expected_result,
                );
                status = cache.addOrientedBoxObstacle(
                    oriented.center,
                    oriented.half_extents,
                    oriented.y_radians,
                    result,
                );
            },
        }
        self.last_status = status;
        try support.expectStatus(expected, status);
        try testing.expectEqual(expected_ref, ref);
        return if (status.succeeded and want_ref) ref else null;
    }

    fn removeObstacle(self: *World, ref: u32) !Status {
        const expected = reference_tile_cache.ref_tc_remove_obstacle(self.reference_cache, ref);
        const status = self.cache.removeObstacle(ref);
        self.last_status = status;
        try support.expectStatus(expected, status);
        return status;
    }

    /// One update on both sides. Returns whether the port has nothing left
    /// to do; with `want_flag` both sides must also say so themselves.
    fn update(self: *World, want_flag: bool) !bool {
        // The slot starts out true on one call and false on the next, so a
        // side that leaves it alone shows.
        const before = (self.update_calls & 1) == 0;
        self.update_calls += 1;
        var expected_up_to_date = before;
        var up_to_date = before;
        const expected = reference_tile_cache.ref_tc_update(
            self.reference_cache,
            0.05,
            self.reference_nav,
            if (want_flag) &expected_up_to_date else null,
        );
        const status = self.cache.update(0.05, &self.nav, if (want_flag) &up_to_date else null);
        try support.expectStatus(expected, status);
        try testing.expectEqual(expected_up_to_date, up_to_date);
        const idle = self.cache.request_count == 0 and self.cache.update_count == 0;
        if (want_flag) try testing.expectEqual(idle, up_to_date);
        return idle;
    }

    /// Updates until both sides have nothing left to do, comparing
    /// everything after each one.
    fn settle(self: *World) !void {
        var rounds: u32 = 0;
        while (rounds < 4 * update_count_max) : (rounds += 1) {
            const idle = try self.update(true);
            try self.expectSame();
            if (idle) return;
        }
        return error.UpdatesNeverSettled;
    }

    /// Fails unless the two sides hold the same tiles, obstacles and
    /// navigation meshes, and handed their mesh processes the same things.
    fn expectSame(self: *World) !void {
        try self.expectSameTiles();
        try self.expectSameObstacles();
        try expectSameMeshes(self.reference_nav, &self.nav);
        try ProcessRecord.expectSame(&self.reference_record, &self.record);
    }

    fn referenceTileIndex(
        self: *const World,
        tile: ?*const reference_tile_cache.CompressedTile,
    ) ?usize {
        const t = tile orelse return null;
        const first = reference_tile_cache.ref_tc_get_tile(self.reference_cache, 0);
        const size: usize = @intCast(reference_tile_cache.ref_sizeof_compressed_tile());
        return (@intFromPtr(t) - @intFromPtr(first)) / size;
    }

    fn tileIndex(self: *const World, tile: ?*const CompressedTile) ?usize {
        const t = tile orelse return null;
        return (@intFromPtr(t) - @intFromPtr(self.cache.tiles.ptr)) / @sizeOf(CompressedTile);
    }

    fn expectSameTiles(self: *World) !void {
        const reference_cache = self.reference_cache;
        const count = self.cache.getTileCount();
        try testing.expectEqual(reference_tile_cache.ref_tc_get_tile_count(reference_cache), count);
        const next_offset: usize = @intCast(
            reference_tile_cache.ref_offsetof_compressed_tile_next(),
        );
        var i: i32 = 0;
        while (i < count) : (i += 1) {
            errdefer std.debug.print("  in cache entry {d} of {d}\n", .{ i, count });
            const expected = reference_tile_cache.ref_tc_get_tile(reference_cache, i);
            const actual = self.cache.getTile(i);
            try testing.expectEqual(reference_tile_cache.ref_tc_tile_salt(expected), actual.salt);
            try testing.expectEqual(reference_tile_cache.ref_tc_tile_flags(expected), actual.flags);
            try testing.expectEqual(
                reference_tile_cache.ref_tc_get_tile_ref(reference_cache, expected),
                self.cache.getTileRef(actual),
            );

            // The chain through `next`: a lookup bucket for a tile, the free
            // list for an unused entry.
            const expected_next: *align(1) const ?*const reference_tile_cache.CompressedTile =
                @ptrFromInt(@intFromPtr(expected) + next_offset);
            try testing.expectEqual(
                self.referenceTileIndex(expected_next.*),
                self.tileIndex(actual.next),
            );

            const expected_header = reference_tile_cache.ref_tc_tile_header(expected);
            try testing.expectEqual(expected_header == null, actual.header == null);
            const expected_size = reference_tile_cache.ref_tc_tile_data_size(expected);
            const expected_compressed_size =
                reference_tile_cache.ref_tc_tile_compressed_size(expected);
            try testing.expectEqual(@as(usize, @intCast(expected_size)), actual.data.len);
            try testing.expectEqual(
                @as(usize, @intCast(expected_compressed_size)),
                actual.compressed.len,
            );
            const header = actual.header orelse continue;

            const size = actual.data.len;
            const expected_data = reference_tile_cache.ref_tc_tile_data(expected).?;
            if (!std.mem.eql(u8, expected_data[0..size], actual.data)) {
                try support.expectBytes(expected_data[0..size], actual.data);
            }
            // The header sits at the front and the stored grids after it.
            const expected_base = @intFromPtr(expected_data);
            const base = @intFromPtr(actual.data.ptr);
            try testing.expectEqual(
                @intFromPtr(expected_header.?) - expected_base,
                @intFromPtr(header) - base,
            );
            try testing.expectEqual(
                @intFromPtr(reference_tile_cache.ref_tc_tile_compressed(expected).?) -
                    expected_base,
                @intFromPtr(actual.compressed.ptr) - base,
            );
        }
    }

    fn referenceObstacleIndex(
        self: *const World,
        obstacle: ?*const reference_tile_cache.Obstacle,
    ) ?usize {
        const ob = obstacle orelse return null;
        const first = reference_tile_cache.ref_tc_get_obstacle(self.reference_cache, 0);
        const size: usize = @intCast(reference_tile_cache.ref_sizeof_tile_cache_obstacle());
        return (@intFromPtr(ob) - @intFromPtr(first)) / size;
    }

    fn obstacleIndex(self: *const World, obstacle: ?*const Obstacle) ?usize {
        const ob = obstacle orelse return null;
        return (@intFromPtr(ob) - @intFromPtr(self.cache.obstacles.ptr)) / @sizeOf(Obstacle);
    }

    fn expectSameObstacles(self: *World) !void {
        const reference_cache = self.reference_cache;
        const count = self.cache.getObstacleCount();
        try testing.expectEqual(
            reference_tile_cache.ref_tc_get_obstacle_count(reference_cache),
            count,
        );
        const next_offset: usize = @intCast(
            reference_tile_cache.ref_offsetof_tile_cache_obstacle_next(),
        );
        var i: i32 = 0;
        while (i < count) : (i += 1) {
            errdefer std.debug.print("  in obstacle entry {d} of {d}\n", .{ i, count });
            const expected = reference_tile_cache.ref_tc_get_obstacle(reference_cache, i);
            const actual = self.cache.getObstacle(i);
            try expectSameObstacle(expected, actual);
            try testing.expectEqual(
                reference_tile_cache.ref_tc_get_obstacle_ref(reference_cache, expected),
                self.cache.getObstacleRef(actual),
            );

            // The free list runs through `next`.
            const expected_next: *align(1) const ?*const reference_tile_cache.Obstacle =
                @ptrFromInt(@intFromPtr(expected) + next_offset);
            try testing.expectEqual(
                self.referenceObstacleIndex(expected_next.*),
                self.obstacleIndex(actual.next),
            );

            var expected_min: [3]f32 = undefined;
            var expected_max: [3]f32 = undefined;
            reference_tile_cache.ref_tc_get_obstacle_bounds(
                reference_cache,
                expected,
                &expected_min,
                &expected_max,
            );
            const bounds = TileCache.getObstacleBounds(actual);
            try support.expectPoint(expected_min, bounds.min);
            try support.expectPoint(expected_max, bounds.max);
        }
    }
};

/// Fails unless the two obstacle entries hold the same obstacle.
fn expectSameObstacle(
    expected: *const reference_tile_cache.Obstacle,
    actual: *const Obstacle,
) !void {
    var snapshot: reference_tile_cache.ObstacleSnapshot = undefined;
    try testing.expect(reference_tile_cache.ref_tc_obstacle_snapshot(expected, &snapshot));
    try testing.expectEqual(snapshot.state, @backingInt(actual.state));
    try testing.expectEqual(snapshot.salt, actual.salt);
    try testing.expectEqual(snapshot.ntouched, actual.touched_count);
    try testing.expectEqual(snapshot.npending, actual.pending_count);
    // All eight of each, the ones past the counts too: both sides leave the
    // same leftovers there.
    try testing.expectEqualSlices(u32, &snapshot.touched, &actual.touched);
    try testing.expectEqualSlices(u32, &snapshot.pending, &actual.pending);
    switch (actual.shape) {
        .cylinder => |cylinder| {
            try testing.expectEqual(@as(u8, 0), snapshot.obstacle_type);
            try support.expectPoint(snapshot.cylinder_pos, cylinder.position);
            try support.expectFloat(snapshot.cylinder_radius, cylinder.radius);
            try support.expectFloat(snapshot.cylinder_height, cylinder.height);
        },
        .box => |box| {
            try testing.expectEqual(@as(u8, 1), snapshot.obstacle_type);
            try support.expectPoint(snapshot.box_bmin, box.bounds_min);
            try support.expectPoint(snapshot.box_bmax, box.bounds_max);
        },
        .oriented_box => |oriented| {
            try testing.expectEqual(@as(u8, 2), snapshot.obstacle_type);
            try support.expectPoint(snapshot.oriented_center, oriented.center);
            try support.expectPoint(snapshot.oriented_half_extents, oriented.half_extents);
            try support.expectFloats(&snapshot.oriented_rot_aux, &oriented.rot_aux);
        },
    }
}

/// The index of one of the original's navigation tile entries; null for
/// null.
fn referenceNavTileIndex(nav: *const reference.NavMesh, tile: ?*const reference.MeshTile) ?usize {
    const t = tile orelse return null;
    const first = reference.ref_nav_mesh_get_tile(nav, 0).?;
    return (@intFromPtr(t) - @intFromPtr(first)) / reference.ref_sizeof_mesh_tile();
}

/// The index of one of the port's navigation tile entries; null for null.
fn navTileIndex(nav: *const NavMesh, tile: ?*const MeshTile) ?usize {
    const t = tile orelse return null;
    return (@intFromPtr(t) - @intFromPtr(nav.tiles.ptr)) / @sizeOf(MeshTile);
}

/// Fails unless every tile entry of the two navigation meshes holds the same
/// tile, byte for byte.
fn expectSameMeshes(expected_nav: *const reference.NavMesh, nav: *const NavMesh) !void {
    const tile_count = nav.getMaxTiles();
    try testing.expectEqual(reference.ref_nav_mesh_get_max_tiles(expected_nav), tile_count);
    try testing.expectEqualSlices(
        u8,
        std.mem.asBytes(reference.ref_nav_mesh_get_params(expected_nav)),
        std.mem.asBytes(nav.getParams()),
    );
    var i: i32 = 0;
    while (i < tile_count) : (i += 1) {
        errdefer std.debug.print("  in tile entry {d} of {d}\n", .{ i, tile_count });
        const expected = reference.ref_nav_mesh_get_tile(expected_nav, i).?;
        const actual = nav.getTile(i);
        try testing.expectEqual(reference.ref_tile_salt(expected), actual.salt);
        try testing.expectEqual(
            reference.ref_tile_links_free_list(expected),
            actual.links_free_list,
        );
        try testing.expectEqual(
            @as(u32, @bitCast(reference.ref_tile_flags(expected))),
            @as(u32, @bitCast(actual.flags)),
        );
        try testing.expectEqual(
            referenceNavTileIndex(expected_nav, reference.ref_tile_next(expected)),
            navTileIndex(nav, actual.next),
        );
        try testing.expectEqual(reference.ref_tile_header(expected) == null, actual.header == null);
        if (actual.header == null) {
            try testing.expectEqual(@as(usize, 0), actual.data.len);
            continue;
        }
        const size: usize = @intCast(reference.ref_tile_data_size(expected));
        try testing.expectEqual(size, actual.data.len);
        const expected_data = reference.ref_tile_data(expected).?;
        if (!std.mem.eql(u8, expected_data[0..size], actual.data)) {
            try support.expectBytes(expected_data[0..size], actual.data);
        }
    }
}

/// How many tile entries of the port's navigation mesh hold a tile.
fn navTileCount(nav: *const NavMesh) usize {
    var count: usize = 0;
    for (nav.tiles) |*tile| count += @intFromBool(tile.header != null);
    return count;
}

fn randomRange(random: std.Random, low: f32, high: f32) f32 {
    return low + (high - low) * random.float(f32);
}

/// An obstacle on the ground of `level`, no wider than `reach` to a side.
fn randomRequest(level: *const support.Level, random: std.Random, reach: f32) ObstacleRequest {
    const at = level.groundPoint(random, 1.0);
    const side_x = randomRange(random, 0.0, reach);
    const side_z = randomRange(random, 0.0, reach);
    const tall = randomRange(random, 0.2, 4.0);
    switch (random.uintLessThan(u8, 3)) {
        0 => return .{ .cylinder = .{
            .position = .{ at[0], at[1] - 0.5, at[2] },
            .radius = side_x,
            .height = tall,
        } },
        1 => return .{ .box = .{
            .bounds_min = .{ at[0] - side_x, at[1] - 0.5, at[2] - side_z },
            .bounds_max = .{ at[0] + side_x, at[1] + tall, at[2] + side_z },
        } },
        else => {
            // Whole and quarter turns come out the same on any sine; the
            // rest of the angles are free.
            const quarter_turns: f32 = @floatFromInt(random.intRangeAtMost(i32, -4, 4));
            const angle = if (random.boolean())
                quarter_turns * (std.math.pi / 2.0)
            else
                randomRange(random, -7.0, 7.0);
            return .{ .oriented_box = .{
                .center = at,
                .half_extents = .{ side_x, tall, side_z },
                .y_radians = angle,
            } };
        },
    }
}

/// A copy of a packed layer moved to another cell: its header names the new
/// place and its bounds move with it. Free with `gpa`.
fn restamped(
    gpa: std.mem.Allocator,
    bytes: []const u8,
    tile_x: i32,
    tile_y: i32,
    layer: i32,
) ![]u8 {
    const copy = try gpa.dupe(u8, bytes);
    const header = headerOf(copy);
    const tile_side = @as(f32, tile_size) * cell_size;
    const shift_x = @as(f32, @floatFromInt(tile_x - header.tile_x)) * tile_side;
    const shift_z = @as(f32, @floatFromInt(tile_y - header.tile_y)) * tile_side;
    header.bounds_min[0] += shift_x;
    header.bounds_max[0] += shift_x;
    header.bounds_min[2] += shift_z;
    header.bounds_max[2] += shift_z;
    header.tile_x = tile_x;
    header.tile_y = tile_y;
    header.tile_layer = layer;
    return copy;
}

/// Gives both caches every layer of the level, owned and borrowed in turn.
fn addEveryLayer(world: *World, fixture: *const Fixture) !void {
    for (fixture.layers, 0..) |layer, i| {
        const flags: u8 = if (i % 2 == 0) tile_cache.compressed_tile_free_data else 0;
        const ref = try world.addBytes(layer.bytes, flags, i % 3 != 0);
        try testing.expect(ref != null);
    }
    try world.expectSame();
}

/// Builds every cell of the level into both navigation meshes.
fn buildEveryCell(world: *World, fixture: *const Fixture) !void {
    var tile_y: i32 = 0;
    while (tile_y < fixture.level.grid.height) : (tile_y += 1) {
        var tile_x: i32 = 0;
        while (tile_x < fixture.level.grid.width) : (tile_x += 1) {
            _ = try world.buildAt(tile_x, tile_y);
        }
    }
    try world.expectSame();
}

/// Fails unless both sides resolve `ref` to the same tile entry.
fn expectSameTileByRef(world: *World, ref: u32) !void {
    try testing.expectEqual(
        world.referenceTileIndex(
            reference_tile_cache.ref_tc_get_tile_by_ref(world.reference_cache, ref),
        ),
        world.tileIndex(world.cache.getTileByRef(ref)),
    );
}

/// Fails unless both sides resolve `ref` to the same obstacle entry.
fn expectSameObstacleByRef(world: *World, ref: u32) !void {
    try testing.expectEqual(
        world.referenceObstacleIndex(
            reference_tile_cache.ref_tc_get_obstacle_by_ref(world.reference_cache, ref),
        ),
        world.obstacleIndex(world.cache.getObstacleByRef(ref)),
    );
}

/// Fails unless both sides find the same tiles at one cell: all of them
/// through buffers of several sizes, and each layer by its number.
fn expectSameCell(world: *World, tile_x: i32, tile_y: i32) !void {
    errdefer std.debug.print("  at cell ({d}, {d})\n", .{ tile_x, tile_y });
    const reference_cache = world.reference_cache;
    for ([_]usize{ 0, 1, 2, layers_per_cell_max }) |capacity| {
        var expected_refs: [layers_per_cell_max]u32 = @splat(untouched_ref);
        var refs: [layers_per_cell_max]u32 = @splat(untouched_ref);
        const expected_count = reference_tile_cache.ref_tc_get_tiles_at(
            reference_cache,
            tile_x,
            tile_y,
            &expected_refs,
            @intCast(capacity),
        );
        const count = world.cache.getTilesAt(tile_x, tile_y, refs[0..capacity]);
        try testing.expectEqual(@as(usize, @intCast(expected_count)), count);
        try testing.expectEqualSlices(u32, &expected_refs, &refs);
    }
    var layer: i32 = -1;
    while (layer < 4) : (layer += 1) {
        const expected = reference_tile_cache.ref_tc_get_tile_at(
            reference_cache,
            tile_x,
            tile_y,
            layer,
        );
        const actual = world.cache.getTileAt(tile_x, tile_y, layer);
        try testing.expectEqual(world.referenceTileIndex(expected), world.tileIndex(actual));
        try testing.expectEqual(
            reference_tile_cache.ref_tc_get_tile_ref(reference_cache, expected),
            world.cache.getTileRef(actual),
        );
    }
}

/// Fails unless both sides find the same tiles under a box, through a buffer
/// of `capacity` references.
fn expectSameQuery(
    world: *World,
    bounds_min: [3]f32,
    bounds_max: [3]f32,
    capacity: usize,
) !void {
    var expected_refs: [64]u32 = @splat(untouched_ref);
    var refs: [64]u32 = @splat(untouched_ref);
    var expected_count: i32 = -5;
    var count: usize = 12345;
    const expected = reference_tile_cache.ref_tc_query_tiles(
        world.reference_cache,
        &bounds_min,
        &bounds_max,
        &expected_refs,
        &expected_count,
        @intCast(capacity),
    );
    const status = world.cache.queryTiles(bounds_min, bounds_max, refs[0..capacity], &count);
    try support.expectStatus(expected, status);
    try testing.expectEqual(@as(usize, @intCast(expected_count)), count);
    try testing.expectEqualSlices(u32, &expected_refs, &refs);
}

/// A box to ask for tiles under: mostly small and on the level, now and then
/// inside out or far away.
fn randomQueryBox(level: *const support.Level, random: std.Random) [2][3]f32 {
    var center = level.groundPoint(random, 2.0);
    const far = random.uintLessThan(u8, 20) == 0;
    if (far) {
        center[0] += randomRange(random, -300.0, 300.0);
        center[2] += randomRange(random, -300.0, 300.0);
    }
    const reach: f32 = if (random.uintLessThan(u8, 8) == 0) 40.0 else 6.0;
    const half: [3]f32 = .{
        randomRange(random, 0.0, reach),
        randomRange(random, 0.0, 8.0),
        randomRange(random, 0.0, reach),
    };
    var box: [2][3]f32 = undefined;
    for (0..3) |axis| {
        box[0][axis] = center[axis] - half[axis];
        box[1][axis] = center[axis] + half[axis];
    }
    if (random.uintLessThan(u8, 20) == 0) std.mem.swap([3]f32, &box[0], &box[1]);
    return box;
}

/// A tile reference that may name a tile, an unused entry, a place past the
/// table, or nothing.
fn randomTileRef(world: *const World, random: std.Random) u32 {
    const held = world.held.items;
    switch (random.uintLessThan(u8, 6)) {
        0 => return 0,
        1 => return random.int(u32),
        2, 3 => if (held.len > 0) return held[random.uintLessThan(usize, held.len)].ref,
        else => {},
    }
    return world.cache.encodeTileId(
        random.uintLessThan(u32, 4),
        random.uintLessThan(u32, world.tileCountMax() + 4),
    );
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

test "init: statuses, parameters and the starting state over table sizes" {
    const gpa = testing.allocator;
    const origin: [3]f32 = .{ -12.5, 3.25, 40.0 };

    for ([_]i32{ 1, 2, 5, 64, 4096 }) |tile_count_max| {
        for ([_]i32{ 0, 1, 128 }) |obstacle_count_max| {
            errdefer std.debug.print(
                "  {d} tiles, {d} obstacles\n",
                .{ tile_count_max, obstacle_count_max },
            );
            var world: World = undefined;
            try world.init(gpa, origin, &support.OffMeshConnections.none, .{
                .tile_count_max = tile_count_max,
                .obstacle_count_max = obstacle_count_max,
            });
            defer world.deinit();
            const cache = &world.cache;
            const reference_cache = world.reference_cache;

            try testing.expectEqualSlices(
                u8,
                std.mem.asBytes(reference_tile_cache.ref_tc_get_params(reference_cache)),
                std.mem.asBytes(cache.getParams()),
            );
            try testing.expectEqual(
                @intFromPtr(world.reference_alloc),
                @intFromPtr(reference_tile_cache.ref_tc_get_alloc(reference_cache)),
            );
            try testing.expectEqual(@intFromPtr(&world.scratch), @intFromPtr(cache.getAlloc()));
            try testing.expectEqual(
                @intFromPtr(world.reference_compressor),
                @intFromPtr(reference_tile_cache.ref_tc_get_compressor(reference_cache)),
            );
            const compressor = cache.getCompressor() orelse return error.TestUnexpectedResult;
            try testing.expectEqual(@intFromPtr(&compressor_anchor), @intFromPtr(compressor.ptr));
            try testing.expect(cache.request_count == 0);
            try testing.expect(cache.update_count == 0);

            // Every entry is unused, salted and on its free list, and
            // nothing waits.
            try world.expectSame();
            try testing.expect(try world.update(true));
            try testing.expect(try world.update(false));
            try world.expectSame();

            // Nothing is anywhere yet.
            var refs: [4]u32 = @splat(untouched_ref);
            var expected_refs: [4]u32 = @splat(untouched_ref);
            try testing.expectEqual(
                @as(usize, @intCast(reference_tile_cache.ref_tc_get_tiles_at(
                    reference_cache,
                    0,
                    0,
                    &expected_refs,
                    expected_refs.len,
                ))),
                cache.getTilesAt(0, 0, &refs),
            );
            try testing.expectEqualSlices(u32, &expected_refs, &refs);
            try testing.expect(cache.getTileAt(0, 0, 0) == null);
            try testing.expect(
                reference_tile_cache.ref_tc_get_tile_at(reference_cache, 0, 0, 0) == null,
            );

            // A table with no room refuses an obstacle; any other takes it.
            const ref = try world.addObstacle(.{ .cylinder = .{
                .position = origin,
                .radius = 1.0,
                .height = 2.0,
            } }, true);
            try testing.expectEqual(obstacle_count_max == 0, ref == null);
            try world.expectSame();
            try world.settle();
        }
    }
}

test "init: a cache that was never readied, and counts below zero" {
    const gpa = testing.allocator;

    var cache: TileCache = .empty;
    const reference_cache = reference_tile_cache.ref_tc_create() orelse
        return error.OutOfMemory;
    defer reference_tile_cache.ref_tc_destroy(reference_cache);
    try testing.expect(reference_tile_cache.ref_tc_get_alloc(reference_cache) == null);
    try testing.expect(cache.getAlloc() == null);
    try testing.expect(reference_tile_cache.ref_tc_get_compressor(reference_cache) == null);
    try testing.expect(cache.getCompressor() == null);
    cache.deinit();

    // The original cannot be asked this: a count below zero fails its
    // allocation, and its destructor then walks a table that is not there.
    // The port refuses the counts and leaves the cache as it was, empty.
    var scratch: detour.BuildAllocator = .{ .allocator = gpa };
    for ([_][2]i32{ .{ -1, 4 }, .{ 4, -1 }, .{ -1, -1 }, .{ -2147483648, 4 } }) |counts| {
        const params = cacheParams(.{ 0.0, 0.0, 0.0 }, counts[0], counts[1]);
        cache = .empty;
        const status = cache.init(gpa, &params, &scratch, portCompressor(), null);
        defer cache.deinit();
        try testing.expect(status.failed);
        try testing.expect(!status.succeeded);
        try testing.expect(cache.getAlloc() == null);
        try testing.expect(cache.getCompressor() == null);
    }
}

// Each side holds a table of about 235 MB for the length of this test.
test "init: too many tiles to leave ten bits of salt" {
    const gpa = testing.allocator;
    const reference_alloc = reference_tile_cache.ref_tc_alloc_create_default() orelse
        return error.OutOfMemory;
    defer reference_tile_cache.ref_tc_alloc_destroy(reference_alloc);
    const reference_compressor = try createReferenceCompressor();
    defer reference_tile_cache.ref_tc_compressor_destroy(reference_compressor);
    var scratch: detour.BuildAllocator = .{ .allocator = gpa };

    // Twenty-two bits of tile index leave ten of salt, the fewest allowed;
    // one tile more needs a twenty-third.
    for ([_]i32{ 1 << 22, (1 << 22) + 1 }) |tile_count_max| {
        errdefer std.debug.print("  {d} tiles\n", .{tile_count_max});
        const params = cacheParams(.{ 0.0, 0.0, 0.0 }, tile_count_max, 1);
        const expected_params = referenceParams(&params);

        const reference_cache = reference_tile_cache.ref_tc_create() orelse
            return error.OutOfMemory;
        defer reference_tile_cache.ref_tc_destroy(reference_cache);
        const expected = reference_tile_cache.ref_tc_init(
            reference_cache,
            &expected_params,
            reference_alloc,
            reference_compressor,
            null,
        );
        var cache: TileCache = .empty;
        const status = cache.init(gpa, &params, &scratch, portCompressor(), null);
        defer cache.deinit();
        try support.expectStatus(expected, status);
        try testing.expectEqual(tile_count_max == 1 << 22, status.succeeded);
        if (!status.succeeded) {
            try testing.expect(cache.getAlloc() == null);
            continue;
        }

        try testing.expectEqual(
            reference_tile_cache.ref_tc_get_tile_count(reference_cache),
            cache.getTileCount(),
        );
        var prng: std.Random.DefaultPrng = .init(0x5a17);
        const random = prng.random();
        for (0..1000) |_| {
            const salt = random.int(u32);
            const index = random.int(u32);
            const ref = random.int(u32);
            try testing.expectEqual(
                reference_tile_cache.ref_tc_encode_tile_id(reference_cache, salt, index),
                cache.encodeTileId(salt, index),
            );
            try testing.expectEqual(
                reference_tile_cache.ref_tc_decode_tile_id_salt(reference_cache, ref),
                cache.decodeTileIdSalt(ref),
            );
            try testing.expectEqual(
                reference_tile_cache.ref_tc_decode_tile_id_tile(reference_cache, ref),
                cache.decodeTileIdTile(ref),
            );
        }
        // The last entry of the table, by index and by reference.
        const last = tile_count_max - 1;
        const expected_tile = reference_tile_cache.ref_tc_get_tile(reference_cache, last);
        const tile = cache.getTile(last);
        try testing.expectEqual(reference_tile_cache.ref_tc_tile_salt(expected_tile), tile.salt);
        try testing.expectEqual(
            reference_tile_cache.ref_tc_get_tile_ref(reference_cache, expected_tile),
            cache.getTileRef(tile),
        );
    }
}

test "encodeTileId, decodeTileIdSalt, decodeTileIdTile: random parts over table sizes" {
    const gpa = testing.allocator;
    const seed: u64 = 0x71d5;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for ([_]i32{ 1, 2, 5, 64, 4096 }) |tile_count_max| {
        var world: World = undefined;
        try world.init(gpa, .{ 0.0, 0.0, 0.0 }, &support.OffMeshConnections.none, .{
            .tile_count_max = tile_count_max,
            .obstacle_count_max = 1,
        });
        defer world.deinit();
        const cache = &world.cache;
        const reference_cache = world.reference_cache;

        for (0..1000) |case| {
            errdefer std.debug.print(
                "  seed {d}, case {d}, {d} tiles\n",
                .{ seed, case, tile_count_max },
            );
            // Parts that fit their bits in half the cases, any at all in the
            // rest: the parts then spill into each other.
            const fits = random.boolean();
            const salt = if (fits) random.uintLessThan(u32, 1 << 10) else random.int(u32);
            const index = if (fits)
                random.uintLessThan(u32, world.tileCountMax())
            else
                random.int(u32);
            const ref = reference_tile_cache.ref_tc_encode_tile_id(reference_cache, salt, index);
            try testing.expectEqual(ref, cache.encodeTileId(salt, index));
            for ([_]u32{ ref, random.int(u32), 0, 0xffffffff }) |decoded| {
                try testing.expectEqual(
                    reference_tile_cache.ref_tc_decode_tile_id_salt(reference_cache, decoded),
                    cache.decodeTileIdSalt(decoded),
                );
                try testing.expectEqual(
                    reference_tile_cache.ref_tc_decode_tile_id_tile(reference_cache, decoded),
                    cache.decodeTileIdTile(decoded),
                );
            }
        }
    }
}

test "encodeObstacleId, decodeObstacleIdSalt, decodeObstacleIdObstacle: random parts" {
    const gpa = testing.allocator;
    const seed: u64 = 0x0b57;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var world: World = undefined;
    try world.init(gpa, .{ 0.0, 0.0, 0.0 }, &support.OffMeshConnections.none, .{});
    defer world.deinit();
    const reference_cache = world.reference_cache;

    for (0..2000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const fits = random.boolean();
        const salt = if (fits) random.uintLessThan(u32, 1 << 16) else random.int(u32);
        const index = if (fits) random.uintLessThan(u32, 1 << 16) else random.int(u32);
        const ref = reference_tile_cache.ref_tc_encode_obstacle_id(reference_cache, salt, index);
        try testing.expectEqual(ref, TileCache.encodeObstacleId(salt, index));
        for ([_]u32{ ref, random.int(u32), 0, 0xffffffff }) |decoded| {
            try testing.expectEqual(
                reference_tile_cache.ref_tc_decode_obstacle_id_salt(reference_cache, decoded),
                TileCache.decodeObstacleIdSalt(decoded),
            );
            try testing.expectEqual(
                reference_tile_cache.ref_tc_decode_obstacle_id_obstacle(reference_cache, decoded),
                TileCache.decodeObstacleIdObstacle(decoded),
            );
        }
    }
}

test "addTile, getTilesAt, getTileAt, getTileRef, getTileByRef: every layer of the level" {
    const gpa = testing.allocator;
    const seed: u64 = 0x100c;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var fixture: Fixture = try .init(gpa, 0);
    defer fixture.deinit();
    try testing.expect(fixture.layers.len > 0);
    var world: World = undefined;
    try fixture.initWorld(&world, .{
        .tile_count_max = @intCast(fixture.layers.len + 3),
        .nav_tile_bits = fixture.navTileBits(),
    });
    defer world.deinit();

    for (fixture.layers, 0..) |layer, i| {
        errdefer std.debug.print("  adding layer {d} of {d}\n", .{ i, fixture.layers.len });
        const flags: u8 = if (i % 2 == 0) tile_cache.compressed_tile_free_data else 0;
        const ref = try world.addBytes(layer.bytes, flags, i % 3 != 0);
        try testing.expect(ref != null);
        try world.expectSame();
        try expectSameCell(&world, layer.tile_x, layer.tile_y);
    }

    // Every cell of the level and a ring around it.
    const grid = fixture.level.grid;
    var tile_y: i32 = -2;
    while (tile_y < grid.height + 2) : (tile_y += 1) {
        var tile_x: i32 = -2;
        while (tile_x < grid.width + 2) : (tile_x += 1) {
            try expectSameCell(&world, tile_x, tile_y);
        }
    }
    try expectSameCell(&world, 1 << 20, -(1 << 20));

    // References: live, stale by salt, past the table, none.
    try testing.expectEqual(
        reference_tile_cache.ref_tc_get_tile_ref(world.reference_cache, null),
        world.cache.getTileRef(null),
    );
    try expectSameTileByRef(&world, 0);
    for (world.held.items) |held| {
        const index = world.cache.decodeTileIdTile(held.ref);
        const salt = world.cache.decodeTileIdSalt(held.ref);
        try expectSameTileByRef(&world, held.ref);
        try expectSameTileByRef(&world, world.cache.encodeTileId(salt + 1, index));
        try expectSameTileByRef(&world, world.cache.encodeTileId(0, index));
        try expectSameTileByRef(&world, world.cache.encodeTileId(salt, world.tileCountMax()));
        try expectSameTileByRef(&world, world.cache.encodeTileId(salt, index + 0x10000));
    }
    for (0..2000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        try expectSameTileByRef(&world, randomTileRef(&world, random));
    }
}

test "queryTiles: random boxes through buffers of several sizes" {
    const gpa = testing.allocator;
    const seed: u64 = 0x9e2b;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var fixture: Fixture = try .init(gpa, 0);
    defer fixture.deinit();
    var world: World = undefined;
    try fixture.initWorld(&world, .{
        .tile_count_max = @intCast(fixture.layers.len + 3),
        .nav_tile_bits = fixture.navTileBits(),
    });
    defer world.deinit();

    // An empty cache finds nothing; a full one is what the rest asks.
    const mesh = &fixture.level.mesh;
    try expectSameQuery(&world, mesh.bounds_min, mesh.bounds_max, 64);
    try addEveryLayer(&world, &fixture);
    for ([_]usize{ 0, 1, 2, 8, 64 }) |capacity| {
        try expectSameQuery(&world, mesh.bounds_min, mesh.bounds_max, capacity);
        try expectSameQuery(&world, mesh.bounds_max, mesh.bounds_min, capacity);
        try expectSameQuery(&world, mesh.bounds_min, mesh.bounds_min, capacity);
    }

    const capacities = [_]usize{ 0, 1, 2, 8, 64 };
    for (0..3000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const box = randomQueryBox(&fixture.level, random);
        const capacity = capacities[random.uintLessThan(usize, capacities.len)];
        try expectSameQuery(&world, box[0], box[1], capacity);
    }
}

test "calcTightTileBounds: the level's headers and random ones" {
    const gpa = testing.allocator;
    const seed: u64 = 0x7167;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var fixture: Fixture = try .init(gpa, 0);
    defer fixture.deinit();
    var world: World = undefined;
    try fixture.initWorld(&world, .{
        .tile_count_max = @intCast(fixture.layers.len + 3),
        .nav_tile_bits = fixture.navTileBits(),
    });
    defer world.deinit();
    try addEveryLayer(&world, &fixture);

    var expected_min: [3]f32 = undefined;
    var expected_max: [3]f32 = undefined;
    var i: i32 = 0;
    while (i < world.cache.getTileCount()) : (i += 1) {
        errdefer std.debug.print("  in cache entry {d}\n", .{i});
        const tile = world.cache.getTile(i);
        const header = tile.header orelse continue;
        const expected_tile = reference_tile_cache.ref_tc_get_tile(world.reference_cache, i);
        reference_tile_cache.ref_tc_calc_tight_tile_bounds(
            world.reference_cache,
            reference_tile_cache.ref_tc_tile_header(expected_tile).?,
            &expected_min,
            &expected_max,
        );
        const bounds = world.cache.calcTightTileBounds(header);
        try support.expectPoint(expected_min, bounds.min);
        try support.expectPoint(expected_max, bounds.max);
    }

    for (0..2000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        var expected_header = std.mem.zeroes(reference_tile_cache.TileCacheLayerHeader);
        for (0..3) |axis| {
            expected_header.bmin[axis] = randomRange(random, -500.0, 500.0);
            expected_header.bmax[axis] = randomRange(random, -500.0, 500.0);
        }
        expected_header.width = random.int(u8);
        expected_header.height = random.int(u8);
        expected_header.minx = random.int(u8);
        expected_header.maxx = random.int(u8);
        expected_header.miny = random.int(u8);
        expected_header.maxy = random.int(u8);
        reference_tile_cache.ref_tc_calc_tight_tile_bounds(
            world.reference_cache,
            &expected_header,
            &expected_min,
            &expected_max,
        );
        const header: *align(1) const LayerHeader = @ptrCast(&expected_header);
        const bounds = world.cache.calcTightTileBounds(header);
        try support.expectPoint(expected_min, bounds.min);
        try support.expectPoint(expected_max, bounds.max);
    }
}

test "addTile: wrong magic, wrong version, a taken place and a full table" {
    const gpa = testing.allocator;
    var fixture: Fixture = try .init(gpa, 0);
    defer fixture.deinit();
    var world: World = undefined;
    try fixture.initWorld(&world, .{ .tile_count_max = 5 });
    defer world.deinit();

    const layer = &fixture.layers[0];
    const scratch = try gpa.dupe(u8, layer.bytes);
    defer gpa.free(scratch);
    const header = headerOf(scratch);
    const owned = tile_cache.compressed_tile_free_data;

    // A refused tile stays the caller's, whatever its flags say.
    for ([_]u8{ 0, owned }) |flags| {
        header.magic +%= 1;
        try testing.expect(try world.addBytes(scratch, flags, true) == null);
        header.version +%= 1;
        try testing.expect(try world.addBytes(scratch, flags, true) == null);
        header.magic -%= 1;
        try testing.expect(try world.addBytes(scratch, flags, false) == null);
        header.version -%= 1;
        try world.expectSame();
    }
    for ([_]i32{ 0, -1, 2, 0x7fffffff, -0x80000000 }) |version| {
        header.version = version;
        try testing.expect(try world.addBytes(scratch, 0, true) == null);
    }
    header.version = tile_cache_builder.tile_cache_version;
    try world.expectSame();

    // The same place twice: the second is refused, with or without bytes
    // that differ.
    const first = try world.addBytes(scratch, owned, true);
    try testing.expect(first != null);
    try world.expectSame();
    for ([_]u8{ 0, owned }) |flags| {
        try testing.expect(try world.addBytes(scratch, flags, true) == null);
        try testing.expect(try world.addBytes(scratch[0..header_size], flags, false) == null);
    }
    try world.expectSame();

    // Fill the table: other layers of the place, other places, and then one
    // tile too many.
    var added: usize = 1;
    while (added < 5) : (added += 1) {
        const moved = try restamped(
            gpa,
            layer.bytes,
            layer.tile_x + @as(i32, @intCast(added % 2)),
            layer.tile_y - @as(i32, @intCast(added / 2)),
            @intCast(added),
        );
        defer gpa.free(moved);
        const flags: u8 = if (added % 2 == 0) owned else 0;
        try testing.expect(try world.addBytes(moved, flags, true) != null);
        try world.expectSame();
    }
    for ([_]u8{ 0, owned }) |flags| {
        const moved = try restamped(gpa, layer.bytes, 100, 100, 0);
        defer gpa.free(moved);
        try testing.expect(try world.addBytes(moved, flags, true) == null);
        try testing.expect(try world.addBytes(moved, flags, false) == null);
        // A wrong magic is seen before the full table is.
        headerOf(moved).magic = 0;
        try testing.expect(try world.addBytes(moved, flags, true) == null);
    }
    try world.expectSame();
    try expectSameCell(&world, layer.tile_x, layer.tile_y);
    try expectSameCell(&world, layer.tile_x + 1, layer.tile_y);
    try expectSameCell(&world, 100, 100);
}

test "addTile: bytes too short to hold a header" {
    // The original reads a header from whatever it is given, so it cannot be
    // asked this. The port refuses the bytes and leaves them the caller's.
    const gpa = testing.allocator;
    var fixture: Fixture = try .init(gpa, 0);
    defer fixture.deinit();
    var world: World = undefined;
    try fixture.initWorld(&world, .{ .tile_count_max = 5 });
    defer world.deinit();

    for (0..header_size) |length| {
        errdefer std.debug.print("  {d} bytes\n", .{length});
        for ([_]u8{ 0, tile_cache.compressed_tile_free_data }) |flags| {
            const short = try gpa.dupe(u8, fixture.layers[0].bytes[0..length]);
            defer gpa.free(short);
            var ref: u32 = untouched_ref;
            try support.expectStatus(invalid_param_bits, world.cache.addTile(short, flags, &ref));
            try testing.expectEqual(untouched_ref, ref);
            try support.expectStatus(invalid_param_bits, world.cache.addTile(short, flags, null));
        }
    }
    try world.expectSame();
}

test "addTile, removeTile: random headers in and out of a small table" {
    // Only the headers are real here, so nothing is built from these tiles:
    // the grids behind them are noise.
    const gpa = testing.allocator;
    const seed: u64 = 0x4ead;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var fixture: Fixture = try .init(gpa, 0);
    defer fixture.deinit();
    var world: World = undefined;
    try fixture.initWorld(&world, .{ .tile_count_max = 16 });
    defer world.deinit();

    var buffer: [header_size + 64]u8 = undefined;
    for (0..3000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        switch (random.uintLessThan(u8, 8)) {
            0...4 => {
                random.bytes(&buffer);
                const bytes = buffer[0 .. header_size + random.uintLessThan(usize, 65)];
                const header = headerOf(bytes);
                header.magic = switch (random.uintLessThan(u8, 8)) {
                    0 => random.int(i32),
                    else => tile_cache_builder.tile_cache_magic,
                };
                header.version = switch (random.uintLessThan(u8, 8)) {
                    0 => random.int(i32),
                    else => tile_cache_builder.tile_cache_version,
                };
                header.tile_x = random.intRangeAtMost(i32, -2, 2);
                header.tile_y = random.intRangeAtMost(i32, -2, 2);
                header.tile_layer = random.intRangeAtMost(i32, 0, 2);
                for (0..3) |axis| {
                    header.bounds_min[axis] = randomRange(random, -100.0, 100.0);
                    header.bounds_max[axis] = randomRange(random, -100.0, 100.0);
                }
                const flags: u8 = switch (random.uintLessThan(u8, 4)) {
                    0 => 0,
                    1 => tile_cache.compressed_tile_free_data,
                    // Bits past the one that has a meaning.
                    2 => random.int(u8) & 0xfe,
                    else => random.int(u8) | tile_cache.compressed_tile_free_data,
                };
                _ = try world.addBytes(bytes, flags, random.boolean());
            },
            5, 6 => _ = try world.removeRef(randomTileRef(&world, random), random.boolean()),
            else => {
                const box = randomQueryBox(&fixture.level, random);
                try expectSameQuery(&world, box[0], box[1], 64);
                try expectSameCell(
                    &world,
                    random.intRangeAtMost(i32, -3, 3),
                    random.intRangeAtMost(i32, -3, 3),
                );
                try expectSameTileByRef(&world, randomTileRef(&world, random));
            },
        }
        try world.expectSame();
    }
}

test "removeTile: borrowed and owned tiles, bad references, entries used again" {
    const gpa = testing.allocator;
    var fixture: Fixture = try .init(gpa, 0);
    defer fixture.deinit();
    var world: World = undefined;
    try fixture.initWorld(&world, .{ .tile_count_max = 5 });
    defer world.deinit();
    const layer = &fixture.layers[0];
    const owned = tile_cache.compressed_tile_free_data;

    // References that name no tile, in an empty table.
    try testing.expect(!try world.removeRef(0, true));
    try testing.expect(!try world.removeRef(0, false));
    try testing.expect(!try world.removeRef(world.cache.encodeTileId(1, 0), true));
    try testing.expect(!try world.removeRef(world.cache.encodeTileId(2, 0), true));
    try testing.expect(!try world.removeRef(world.cache.encodeTileId(1, 5), true));
    try testing.expect(!try world.removeRef(world.cache.encodeTileId(1, 7), false));
    try testing.expect(!try world.removeRef(0xffffffff, true));
    try world.expectSame();

    // Each way to hold a tile, each way to ask for it back. The entry goes
    // round and comes back with a new salt each time.
    var round: u32 = 0;
    for ([_]u8{ 0, owned }) |flags| {
        for ([_]bool{ true, false }) |want_data| {
            round += 1;
            errdefer std.debug.print("  round {d}\n", .{round});
            const ref = (try world.addBytes(layer.bytes, flags, true)).?;
            try world.expectSame();
            try expectSameCell(&world, layer.tile_x, layer.tile_y);

            // Near misses leave the tile where it is.
            const index = world.cache.decodeTileIdTile(ref);
            const salt = world.cache.decodeTileIdSalt(ref);
            const cache = &world.cache;
            try testing.expect(!try world.removeRef(cache.encodeTileId(salt + 1, index), true));
            try testing.expect(!try world.removeRef(cache.encodeTileId(0, index), true));
            try testing.expect(!try world.removeRef(cache.encodeTileId(salt, index + 5), true));
            try world.expectSame();

            try testing.expect(try world.removeRef(ref, want_data));
            try world.expectSame();
            try expectSameCell(&world, layer.tile_x, layer.tile_y);
            try expectSameTileByRef(&world, ref);
            // Gone is gone: the old reference is stale, and the entry's new
            // one names no tile.
            try testing.expect(!try world.removeRef(ref, want_data));
            try testing.expect(!try world.removeRef(cache.encodeTileId(salt + 1, index), true));
            try world.expectSame();
        }
    }

    // Several tiles of one lookup bucket, taken out first, middle and last.
    var refs: [5]u32 = undefined;
    for (&refs, 0..) |*ref, i| {
        const moved = try restamped(gpa, layer.bytes, layer.tile_x, layer.tile_y, @intCast(i));
        defer gpa.free(moved);
        ref.* = (try world.addBytes(moved, if (i % 2 == 0) owned else 0, true)).?;
    }
    try world.expectSame();
    for ([_]usize{ 2, 0, 4, 3, 1 }) |i| {
        try testing.expect(try world.removeRef(refs[i], i % 2 == 0));
        try world.expectSame();
        try expectSameCell(&world, layer.tile_x, layer.tile_y);
    }
}

test "buildNavMeshTilesAt, buildNavMeshTile: every cell, with and without a mesh process" {
    const gpa = testing.allocator;
    var fixture: Fixture = try .init(gpa, 8);
    defer fixture.deinit();

    for ([_]bool{ true, false }) |with_process| {
        errdefer std.debug.print("  mesh process: {any}\n", .{with_process});
        var world: World = undefined;
        try fixture.initWorld(&world, .{
            .tile_count_max = @intCast(fixture.layers.len + 3),
            .nav_tile_bits = fixture.navTileBits(),
            .with_process = with_process,
        });
        defer world.deinit();

        // Nothing to build yet, anywhere.
        try testing.expect((try world.buildAt(0, 0)).succeeded);
        try world.expectSame();
        try addEveryLayer(&world, &fixture);

        // Cell by cell, a ring of empty cells included.
        const grid = fixture.level.grid;
        var tile_y: i32 = -1;
        while (tile_y <= grid.height) : (tile_y += 1) {
            var tile_x: i32 = -1;
            while (tile_x <= grid.width) : (tile_x += 1) {
                errdefer std.debug.print("  at cell ({d}, {d})\n", .{ tile_x, tile_y });
                _ = try world.buildAt(tile_x, tile_y);
                try world.expectSame();
            }
        }
        try testing.expect(navTileCount(&world.nav) > 0);
        if (with_process) try testing.expect(world.record.calls > 0);
        if (!with_process) try testing.expectEqual(@as(u32, 0), world.record.calls);

        // Tile by tile, over the tiles already there: each one is taken out
        // of the navigation mesh and put back.
        for (world.held.items) |held| {
            errdefer std.debug.print("  at tile {x}\n", .{held.ref});
            _ = try world.buildRef(held.ref);
            try world.expectSame();
        }
        // And every cell at once more, layers in the order the cell holds
        // them.
        try buildEveryCell(&world, &fixture);
    }
}

test "buildNavMeshTile: references that name no tile" {
    const gpa = testing.allocator;
    var fixture: Fixture = try .init(gpa, 0);
    defer fixture.deinit();
    var world: World = undefined;
    // Five entries under three bits of index: the indexes six and seven fit
    // a reference and lie past the table.
    try fixture.initWorld(&world, .{ .tile_count_max = 5 });
    defer world.deinit();
    const cache = &world.cache;

    const ref = (try world.addBytes(fixture.layers[0].bytes, 0, true)).?;
    try testing.expect((try world.buildRef(ref)).succeeded);
    try world.expectSame();
    const index = cache.decodeTileIdTile(ref);
    const salt = cache.decodeTileIdSalt(ref);

    const refused = [_]u32{
        0,
        // The right entry under the wrong salt.
        cache.encodeTileId(salt + 1, index),
        cache.encodeTileId(0, index),
        // An entry that holds nothing, under its own salt and another.
        cache.encodeTileId(1, index + 1),
        cache.encodeTileId(2, index + 1),
        // One past the table: the port's own refusal.
        cache.encodeTileId(1, 5),
        // Further past it.
        cache.encodeTileId(1, 6),
        cache.encodeTileId(1, 7),
        0xffffffff,
    };
    for (refused) |bad| {
        errdefer std.debug.print("  at tile {x}\n", .{bad});
        const status = try world.buildRef(bad);
        try testing.expect(status.failed);
        try world.expectSame();
    }

    // A tile that was taken out is no longer built.
    try testing.expect(try world.removeRef(ref, true));
    try testing.expect((try world.buildRef(ref)).failed);
    try world.expectSame();
}

test "buildNavMeshTile: tiles with nothing, too much or too little behind the header" {
    const gpa = testing.allocator;
    var fixture: Fixture = try .init(gpa, 0);
    defer fixture.deinit();
    var world: World = undefined;
    try fixture.initWorld(&world, .{ .tile_count_max = 8 });
    defer world.deinit();

    const layer = fixture.fullestLayer();
    const real_header: *align(1) const LayerHeader = @ptrCast(layer.bytes.ptr);
    const grid_size = @as(usize, real_header.width) * real_header.height;
    try testing.expectEqual(header_size + grid_size * 3, layer.bytes.len);

    // The real tile first, so the navigation mesh holds something here.
    const real = (try world.addBytes(layer.bytes, 0, true)).?;
    try testing.expect((try world.buildRef(real)).succeeded);
    try world.expectSame();
    try testing.expectEqual(@as(usize, 1), navTileCount(&world.nav));

    // A header with no grids behind it reads as a layer with no ground: the
    // build succeeds, makes nothing, and takes out the tile that was there.
    try testing.expect(try world.removeRef(real, false));
    const bare = (try world.addBytes(layer.bytes[0..header_size], 0, true)).?;
    try testing.expect((try world.buildRef(bare)).succeeded);
    try world.expectSame();
    try testing.expectEqual(@as(usize, 0), navTileCount(&world.nav));
    try testing.expect(try world.removeRef(bare, true));

    // Grids with a fourth part behind them still fit the buffer both sides
    // unpack into, and build the same tile as without it.
    const padded = try gpa.alloc(u8, header_size + grid_size * 4);
    defer gpa.free(padded);
    @memset(padded, 0x5a);
    @memcpy(padded[0..layer.bytes.len], layer.bytes);
    const fits = (try world.addBytes(padded, 0, true)).?;
    try testing.expect((try world.buildRef(fits)).succeeded);
    try world.expectSame();
    try testing.expectEqual(@as(usize, 1), navTileCount(&world.nav));
    try testing.expect(try world.removeRef(fits, true));

    // One byte more does not fit: the compressor refuses, and the build
    // fails with its status. The navigation tile stays.
    const long = try gpa.alloc(u8, header_size + grid_size * 4 + 1);
    defer gpa.free(long);
    @memset(long, 0);
    @memcpy(long[0..layer.bytes.len], layer.bytes);
    const too_long = (try world.addBytes(long, 0, true)).?;
    try testing.expect((try world.buildRef(too_long)).failed);
    try testing.expect((try world.buildAt(layer.tile_x, layer.tile_y)).failed);
    try world.expectSame();
    try testing.expectEqual(@as(usize, 1), navTileCount(&world.nav));
    try testing.expect(try world.removeRef(too_long, true));

    // A header that claims a smaller grid than the bytes hold fails the
    // same way.
    for ([_][2]u8{ .{ 1, 1 }, .{ 1, real_header.height }, .{ real_header.width, 1 } }) |size| {
        if (@as(usize, size[0]) * size[1] * 4 >= grid_size * 3) continue;
        const shrunk = try gpa.dupe(u8, layer.bytes);
        defer gpa.free(shrunk);
        headerOf(shrunk).width = size[0];
        headerOf(shrunk).height = size[1];
        const small = (try world.addBytes(shrunk, 0, true)).?;
        try testing.expect((try world.buildRef(small)).failed);
        try world.expectSame();
        try testing.expect(try world.removeRef(small, true));
    }
    try world.expectSame();
}

test "buildNavMeshTilesAt: a navigation mesh with no room left" {
    const gpa = testing.allocator;
    var fixture: Fixture = try .init(gpa, 0);
    defer fixture.deinit();
    var world: World = undefined;
    // One tile is all these navigation meshes hold.
    try fixture.initWorld(&world, .{
        .tile_count_max = @intCast(fixture.layers.len + 3),
        .nav_tile_bits = 0,
    });
    defer world.deinit();
    try addEveryLayer(&world, &fixture);

    var failures: usize = 0;
    for (world.held.items) |held| {
        errdefer std.debug.print("  at tile {x}\n", .{held.ref});
        const status = try world.buildRef(held.ref);
        failures += @intFromBool(status.failed);
        try world.expectSame();
    }
    try testing.expectEqual(@as(usize, 1), navTileCount(&world.nav));
    if (fixture.layers.len > 1) try testing.expect(failures > 0);
    // The cells again: the tile that is in is rebuilt in place, the rest
    // are refused again.
    try buildEveryCell(&world, &fixture);
}

/// How many tiles the obstacles of the port's cache touch between them.
fn touchedTotal(world: *const World) usize {
    var total: usize = 0;
    for (world.cache.obstacles) |*obstacle| total += obstacle.touched_count;
    return total;
}

test "addObstacle, addBoxObstacle, addOrientedBoxObstacle, removeObstacle: each shape" {
    const gpa = testing.allocator;
    const seed: u64 = 0x0b5c;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var fixture: Fixture = try .init(gpa, 4);
    defer fixture.deinit();
    var world: World = undefined;
    try fixture.initWorld(&world, .{
        .tile_count_max = @intCast(fixture.layers.len + 3),
        .nav_tile_bits = fixture.navTileBits(),
    });
    defer world.deinit();
    try addEveryLayer(&world, &fixture);
    try buildEveryCell(&world, &fixture);
    try testing.expectEqual(
        reference_tile_cache.ref_tc_get_obstacle_ref(world.reference_cache, null),
        world.cache.getObstacleRef(null),
    );
    try expectSameObstacleByRef(&world, 0);

    const quarter = std.math.pi / 2.0;
    const angles = [_]f32{ 0.0, quarter, -quarter, 2.0 * quarter, 4.0 * quarter, 0.7, -2.9, 5.5 };
    var touched_most: usize = 0;
    for (0..24) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const at = fixture.level.groundPoint(random, 0.0);
        // Now and then a cylinder with no width at all.
        const radius: f32 = if (case % 4 == 0) 0.0 else randomRange(random, 0.3, 2.5);
        const request: ObstacleRequest = switch (case % 3) {
            0 => .{ .cylinder = .{
                .position = .{ at[0], at[1] - 0.25, at[2] },
                .radius = radius,
                .height = randomRange(random, 0.5, 3.0),
            } },
            1 => .{ .box = .{
                .bounds_min = .{ at[0] - 1.0, at[1] - 0.5, at[2] - 2.0 },
                .bounds_max = .{ at[0] + 1.5, at[1] + 2.0, at[2] + 0.5 },
            } },
            else => .{ .oriented_box = .{
                .center = .{ at[0], at[1] + 0.5, at[2] },
                .half_extents = .{ 1.5, 1.0, 0.75 },
                .y_radians = angles[(case / 3) % angles.len],
            } },
        };

        // Asked for: the obstacle waits in its entry, then is cut out of
        // every tile it touches, one tile an update.
        const ref = (try world.addObstacle(request, true)).?;
        try world.expectSame();
        try expectSameObstacleByRef(&world, ref);
        try world.settle();
        try expectSameObstacleByRef(&world, ref);
        touched_most = @max(touched_most, touchedTotal(&world));

        // The same again with no slot for the reference. This one stays.
        if (case % 5 == 0) {
            try testing.expect(try world.addObstacle(request, false) == null);
            try testing.expect(world.last_status.succeeded);
            try world.expectSame();
            try world.settle();
        }

        // Taken out: its tiles are built again without it, and the old
        // reference names nothing.
        try testing.expect((try world.removeObstacle(ref)).succeeded);
        try world.expectSame();
        try world.settle();
        try expectSameObstacleByRef(&world, ref);
    }
    // The obstacles did land on tiles, so the builds above cut them out.
    try testing.expect(touched_most > 0);
}

test "addObstacle, update: more tiles than an obstacle can list, and none" {
    const gpa = testing.allocator;
    var fixture: Fixture = try .init(gpa, 0);
    defer fixture.deinit();
    var world: World = undefined;
    try fixture.initWorld(&world, .{
        .tile_count_max = @intCast(fixture.layers.len + 3),
        .nav_tile_bits = fixture.navTileBits(),
    });
    defer world.deinit();
    try addEveryLayer(&world, &fixture);
    try buildEveryCell(&world, &fixture);
    const mesh = &fixture.level.mesh;

    // Far from everything: it touches no tile, so no build ever counts it
    // done, until some other tile is built.
    const far = (try world.addObstacle(.{ .cylinder = .{
        .position = .{ mesh.bounds_max[0] + 250.0, mesh.bounds_min[1], mesh.bounds_min[2] },
        .radius = 2.0,
        .height = 2.0,
    } }, true)).?;
    try world.settle();
    try expectSameObstacleByRef(&world, far);

    // Over the whole level: only the first eight tiles found are listed.
    const whole = (try world.addObstacle(.{ .box = .{
        .bounds_min = .{
            mesh.bounds_min[0] - 5.0,
            mesh.bounds_min[1] - 5.0,
            mesh.bounds_min[2] - 5.0,
        },
        .bounds_max = .{
            mesh.bounds_max[0] + 5.0,
            mesh.bounds_max[1] + 5.0,
            mesh.bounds_max[2] + 5.0,
        },
    } }, true)).?;
    try world.expectSame();
    _ = try world.update(true);
    try world.expectSame();
    const listed_most: usize = @min(fixture.layers.len, tile_cache.touched_tile_count_max);
    const listed: usize = world.cache.getObstacleByRef(whole).?.touched_count;
    try testing.expectEqual(listed_most, listed);
    try world.settle();

    // Both go, in one batch.
    try testing.expect((try world.removeObstacle(far)).succeeded);
    try testing.expect((try world.removeObstacle(whole)).succeeded);
    try world.expectSame();
    try world.settle();
    try expectSameObstacleByRef(&world, far);
    try expectSameObstacleByRef(&world, whole);
}

test "addObstacle, removeObstacle: a full request queue and a full table" {
    const gpa = testing.allocator;
    const seed: u64 = 0xf011;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var fixture: Fixture = try .init(gpa, 0);
    defer fixture.deinit();
    var world: World = undefined;
    try fixture.initWorld(&world, .{
        .tile_count_max = @intCast(fixture.layers.len + 3),
        .nav_tile_bits = fixture.navTileBits(),
        .obstacle_count_max = 70,
    });
    defer world.deinit();
    try addEveryLayer(&world, &fixture);
    try buildEveryCell(&world, &fixture);

    // Requests wait for the next update, and only so many can wait.
    var refs: [request_count_max]u32 = undefined;
    for (&refs) |*ref| {
        ref.* = (try world.addObstacle(randomRequest(&fixture.level, random, 1.5), true)).?;
    }
    try world.expectSame();
    for (0..3) |_| {
        const request = randomRequest(&fixture.level, random, 1.5);
        try testing.expect(try world.addObstacle(request, true) == null);
        try support.expectStatus(buffer_too_small_bits, world.last_status);
        try testing.expect(try world.addObstacle(request, false) == null);
        try support.expectStatus(buffer_too_small_bits, world.last_status);
        try support.expectStatus(buffer_too_small_bits, try world.removeObstacle(refs[0]));
    }
    // No reference is no request: that one is always taken.
    try testing.expect((try world.removeObstacle(0)).succeeded);
    try world.expectSame();
    try world.settle();

    // The queue is empty again and the table has room for six more.
    for (0..6) |_| {
        const request = randomRequest(&fixture.level, random, 1.5);
        try testing.expect(try world.addObstacle(request, true) != null);
    }
    for (0..3) |_| {
        const request = randomRequest(&fixture.level, random, 1.5);
        try testing.expect(try world.addObstacle(request, true) == null);
        try support.expectStatus(out_of_memory_bits, world.last_status);
    }
    try world.expectSame();
    try world.settle();

    // Entries come back once their removal is done, not when it is asked.
    // It is done when the obstacle's tiles are built again, which never
    // happens for an obstacle that touched none.
    try testing.expect((try world.removeObstacle(refs[3])).succeeded);
    const refused = randomRequest(&fixture.level, random, 1.5);
    try testing.expect(try world.addObstacle(refused, true) == null);
    try support.expectStatus(out_of_memory_bits, world.last_status);
    try world.settle();
    const returned = world.cache.getObstacleByRef(refs[3]) == null;
    const again = try world.addObstacle(randomRequest(&fixture.level, random, 1.5), true);
    try testing.expectEqual(returned, again != null);
    try world.settle();
}

test "removeObstacle: while still waiting, twice, stale, past the table and forged" {
    const gpa = testing.allocator;
    const seed: u64 = 0x5a1e;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var fixture: Fixture = try .init(gpa, 0);
    defer fixture.deinit();
    var world: World = undefined;
    try fixture.initWorld(&world, .{
        .tile_count_max = @intCast(fixture.layers.len + 3),
        .nav_tile_bits = fixture.navTileBits(),
        .obstacle_count_max = 6,
    });
    defer world.deinit();
    try addEveryLayer(&world, &fixture);
    try buildEveryCell(&world, &fixture);
    const level = &fixture.level;

    // Asked for and taken back before any update saw it.
    const brief = (try world.addObstacle(randomRequest(level, random, 2.0), true)).?;
    try testing.expect((try world.removeObstacle(brief)).succeeded);
    try world.expectSame();
    try world.settle();
    try expectSameObstacleByRef(&world, brief);

    // Taken out twice in one batch, and once more after it is gone.
    const twice = (try world.addObstacle(randomRequest(level, random, 2.0), true)).?;
    try world.settle();
    try testing.expect((try world.removeObstacle(twice)).succeeded);
    try testing.expect((try world.removeObstacle(twice)).succeeded);
    try world.expectSame();
    try world.settle();
    try testing.expect((try world.removeObstacle(twice)).succeeded);
    try world.expectSame();
    try world.settle();

    // References that name no entry at all.
    const kept = (try world.addObstacle(randomRequest(level, random, 2.0), true)).?;
    try world.settle();
    for ([_]u32{
        TileCache.encodeObstacleId(1, 6),
        TileCache.encodeObstacleId(1, 200),
        TileCache.encodeObstacleId(0xffff, 0xffff),
        TileCache.encodeObstacleId(TileCache.decodeObstacleIdSalt(kept) + 1, 0),
        0xffffffff,
    }) |bad| {
        errdefer std.debug.print("  at obstacle {x}\n", .{bad});
        try expectSameObstacleByRef(&world, bad);
        try testing.expect((try world.removeObstacle(bad)).succeeded);
        try world.expectSame();
        try world.settle();
    }
    try expectSameObstacleByRef(&world, kept);

    // A made-up reference to an entry that holds nothing, under the entry's
    // own salt: both sides take it for an obstacle and free the entry a
    // second time. What they hand out after that, they hand out alike.
    var forged: u32 = 0;
    for (world.cache.obstacles, 0..) |*obstacle, i| {
        if (obstacle.state != .empty) continue;
        forged = TileCache.encodeObstacleId(obstacle.salt, @intCast(i));
    }
    try testing.expect(forged != 0);
    try expectSameObstacleByRef(&world, forged);
    try testing.expect((try world.removeObstacle(forged)).succeeded);
    try world.expectSame();
    // The entry is only let go when a tile is built, so give the update one
    // to build.
    try testing.expect((try world.removeObstacle(kept)).succeeded);
    try world.settle();
    for (0..10) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        _ = try world.addObstacle(randomRequest(level, random, 2.0), true);
        try world.expectSame();
        try world.settle();
    }
}

test "update: more tiles to build again in one batch than the update queue holds" {
    const gpa = testing.allocator;
    var fixture: Fixture = try .init(gpa, 0);
    defer fixture.deinit();
    var world: World = undefined;
    try fixture.initWorld(&world, .{ .tile_count_max = 128, .nav_tile_bits = 7 });
    defer world.deinit();

    // One layer of the level, copied over a grid of eight cells by ten.
    const source = fixture.fullestLayer();
    var tile_y: i32 = 0;
    while (tile_y < 10) : (tile_y += 1) {
        var tile_x: i32 = 0;
        while (tile_x < 8) : (tile_x += 1) {
            const copy = try restamped(gpa, source.bytes, tile_x, tile_y, 0);
            defer gpa.free(copy);
            const flags: u8 = if (tile_x == tile_y) tile_cache.compressed_tile_free_data else 0;
            try testing.expect(try world.addBytes(copy, flags, true) != null);
            try testing.expect((try world.buildAt(tile_x, tile_y)).succeeded);
        }
    }
    try world.expectSame();
    try testing.expectEqual(@as(usize, 80), navTileCount(&world.nav));

    const header = headerOf(source.bytes);
    const low = header.bounds_min[1] - 10.0;
    const high = header.bounds_max[1] + 10.0;
    const origin = world.params.origin;
    const side = @as(f32, tile_size) * cell_size;
    // Halfway up the rows of the layer that hold cells.
    const rows: f32 = @floatFromInt(@as(u32, header.y_min) + header.y_max + 1);
    const middle = rows * 0.5 * cell_size;

    // Ten boxes, each across its own block of four cells by two and over
    // half of every cell in it, then one box over everything: eighty tiles
    // to build again, and a queue that holds sixty-four. The tiles left out
    // are never built for these obstacles.
    var refs: [11]u32 = undefined;
    for (refs[0..10], 0..) |*ref, block| {
        const block_x: f32 = @floatFromInt(block % 2);
        const block_z: f32 = @floatFromInt(block / 2);
        const min_x = origin[0] + block_x * 4.0 * side;
        const min_z = origin[2] + block_z * 2.0 * side + middle;
        ref.* = (try world.addObstacle(.{ .box = .{
            .bounds_min = .{ min_x + 0.05, low, min_z },
            .bounds_max = .{ min_x + 4.0 * side - 0.05, high, min_z + side },
        } }, true)).?;
    }
    refs[10] = (try world.addObstacle(.{ .box = .{
        .bounds_min = .{ origin[0] - 5.0, low, origin[2] - 5.0 },
        .bounds_max = .{ origin[0] + 8.0 * side + 5.0, high, origin[2] + 10.0 * side + 5.0 },
    } }, true)).?;
    try world.expectSame();

    // The first update takes every request and builds one tile.
    _ = try world.update(true);
    try world.expectSame();
    try testing.expectEqual(@as(i32, update_count_max - 1), world.cache.update_count);
    try world.settle();
    for (refs) |ref| try expectSameObstacleByRef(&world, ref);

    // All of them out again in one batch, through the same full queue.
    for (refs) |ref| try testing.expect((try world.removeObstacle(ref)).succeeded);
    try world.expectSame();
    _ = try world.update(false);
    try world.expectSame();
    try testing.expectEqual(@as(i32, update_count_max - 1), world.cache.update_count);
    try world.settle();
    for (refs) |ref| try expectSameObstacleByRef(&world, ref);
    // Every tile that was built with an obstacle is whole again.
    try testing.expectEqual(@as(usize, 80), navTileCount(&world.nav));
}

/// An obstacle reference that may name an obstacle, an entry with none, a
/// place past the table, or nothing.
fn randomObstacleRef(world: *const World, random: std.Random) u32 {
    switch (random.uintLessThan(u8, 4)) {
        0 => return 0,
        1 => return random.int(u32),
        else => {},
    }
    const count: u32 = @intCast(world.cache.obstacles.len);
    return TileCache.encodeObstacleId(
        random.uintLessThan(u32, 4),
        random.uintLessThan(u32, count + 4),
    );
}

test "fuzz: tiles, builds, obstacles, updates and lookups in any order" {
    const gpa = testing.allocator;
    const seed: u64 = 0x7e57c0de;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var fixture: Fixture = try .init(gpa, 6);
    defer fixture.deinit();
    var world: World = undefined;
    try fixture.initWorld(&world, .{
        .tile_count_max = 24,
        .obstacle_count_max = 12,
        .nav_tile_bits = fixture.navTileBits(),
    });
    defer world.deinit();
    const level = &fixture.level;

    // The obstacles asked for and not yet taken out, and the latest one
    // that was.
    var live: std.ArrayList(u32) = .empty;
    defer live.deinit(gpa);
    var gone: u32 = 0;

    for (0..2000) |step| {
        errdefer std.debug.print("  seed {d}, step {d}\n", .{ seed, step });
        switch (random.uintLessThan(u8, 100)) {
            // A layer of the level goes in, if its place and the table
            // have room.
            0...17 => {
                const layer = &fixture.layers[random.uintLessThan(usize, fixture.layers.len)];
                _ = try world.addBytes(layer.bytes, random.int(u8), random.boolean());
            },
            // A tile comes out, or a reference that names none is refused.
            18...25 => {
                const held = world.held.items;
                const ref = if (held.len > 0 and random.boolean())
                    held[random.uintLessThan(usize, held.len)].ref
                else
                    randomTileRef(&world, random);
                _ = try world.removeRef(ref, random.boolean());
            },
            // A cell or one tile is built, whatever obstacles are on it.
            26...35 => if (random.boolean()) {
                const tile_x = random.intRangeAtMost(i32, -1, level.grid.width);
                const tile_y = random.intRangeAtMost(i32, -1, level.grid.height);
                _ = try world.buildAt(tile_x, tile_y);
            } else {
                _ = try world.buildRef(randomTileRef(&world, random));
            },
            // An obstacle is asked for: mostly small, now and then over
            // more tiles than it can list.
            36...53 => {
                const reach: f32 = if (random.uintLessThan(u8, 6) == 0) 30.0 else 2.5;
                const request = randomRequest(level, random, reach);
                if (try world.addObstacle(request, true)) |ref| try live.append(gpa, ref);
            },
            // An obstacle is taken out: mostly a live one, now and then one
            // already gone, none, or a made-up reference.
            54...63 => {
                const kind = random.uintLessThan(u8, 20);
                var ref: u32 = 0;
                if (kind < 12 and live.items.len > 0) {
                    ref = live.swapRemove(random.uintLessThan(usize, live.items.len));
                    gone = ref;
                } else if (kind < 15) {
                    ref = gone;
                } else if (kind < 17) {
                    ref = random.int(u32);
                } else if (kind < 19) {
                    ref = randomObstacleRef(&world, random);
                }
                _ = try world.removeObstacle(ref);
            },
            // A few updates in a row, so the requests do get worked off.
            64...93 => for (0..random.uintAtMost(usize, 5) + 1) |_| {
                _ = try world.update(random.boolean());
                try world.expectSame();
            },
            else => {
                const box = randomQueryBox(level, random);
                try expectSameQuery(&world, box[0], box[1], random.uintAtMost(usize, 64));
                try expectSameCell(
                    &world,
                    random.intRangeAtMost(i32, -1, level.grid.width),
                    random.intRangeAtMost(i32, -1, level.grid.height),
                );
                try expectSameTileByRef(&world, randomTileRef(&world, random));
                try expectSameObstacleByRef(&world, randomObstacleRef(&world, random));
                if (live.items.len > 0) try expectSameObstacleByRef(&world, live.items[0]);
            },
        }
        try world.expectSame();
    }
    // The steps did build tiles, with the mesh process in the way.
    try testing.expect(world.record.calls > 0);

    // What still waits gets done, then every obstacle goes, then every tile.
    try world.settle();
    for (live.items) |ref| {
        if (world.cache.request_count == request_count_max) try world.settle();
        try testing.expect((try world.removeObstacle(ref)).succeeded);
    }
    try world.settle();
    while (world.held.items.len > 0) {
        const ref = world.held.items[world.held.items.len - 1].ref;
        try testing.expect(try world.removeRef(ref, random.boolean()));
    }
    try world.expectSame();
    try world.settle();
}
