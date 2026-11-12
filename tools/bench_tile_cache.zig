//! Benchmark rows for the tile cache: rebuilding navigation tiles from
//! their layers, obstacles coming and going, and the cache's bookkeeping.

const std = @import("std");
const support = @import("support");
const Bench = @import("bench.zig").Bench;
const opaqueFn = @import("bench.zig").opaqueFn;

const detour = support.detour;
const reference = support.reference;
const reference_recast = support.reference_recast;
const reference_tile_cache = support.reference_tile_cache;
const tile_cache = detour.tile_cache;
const tile_cache_builder = detour.tile_cache_builder;
const Status = detour.Status;

/// The cells along each side of a tile, and their size.
const tile_size = 48;
const cell_size: f32 = 0.3;
const cell_height: f32 = 0.2;
/// The most layers one cell of the grid packs.
const layers_per_cell_max = 32;
/// How many obstacles the update row adds and removes.
const obstacle_count = 16;
/// The most calls of `update` one round of obstacles may take.
const update_call_max = 1024;
/// How many boxes the query row asks about, their half side, and the seed
/// of their places.
const box_count = 1000;
const box_half_side: f32 = 5.0;
const place_seed = 0x74696c6563616368;
/// The status bit the original reports success with.
const success_bits: u32 = 1 << 30;

inline fn statusSum(status: Status) u64 {
    return @as(u32, @bitCast(status));
}

// ---------------------------------------------------------------------------
// The compressor: stores bytes as they are, the same on both sides
// ---------------------------------------------------------------------------

var compressor_anchor: u8 = 0;

fn portMaxCompressedSize(_: *anyopaque, buffer_size: i32) i32 {
    return buffer_size;
}

fn portCompress(_: *anyopaque, buffer: []const u8, compressed: []u8, size: *i32) Status {
    if (buffer.len > compressed.len) return .failure;
    @memcpy(compressed[0..buffer.len], buffer);
    size.* = @intCast(buffer.len);
    return .success;
}

fn portDecompress(_: *anyopaque, compressed: []const u8, buffer: []u8, size: *i32) Status {
    if (compressed.len > buffer.len) return .failure;
    @memcpy(buffer[0..compressed.len], compressed);
    size.* = @intCast(compressed.len);
    return .success;
}

const port_compressor_vtable: detour.Compressor.VTable = .{
    .maxCompressedSize = portMaxCompressedSize,
    .compress = portCompress,
    .decompress = portDecompress,
};

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
    if (buffer_size > max_compressed_size) return 1 << 31;
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
    if (compressed_size > max_buffer_size) return 1 << 31;
    const size: usize = @intCast(compressed_size);
    @memcpy(buffer[0..size], compressed[0..size]);
    buffer_size.* = compressed_size;
    return success_bits;
}

// ---------------------------------------------------------------------------
// The level, its layers and both sides' caches
// ---------------------------------------------------------------------------

/// One packed layer, with a copy of its bytes for each side.
const Layer = struct {
    port_bytes: []u8,
    reference_bytes: []u8,
};

/// Both sides' caches over the tiled level, every layer added and built
/// into both sides' navigation meshes. Initialised in place and never
/// moved: the caches keep pointers into it.
const Context = struct {
    level: support.Level,
    layers: []Layer,

    scratch: detour.BuildAllocator,
    cache: detour.TileCache,
    nav: detour.NavMesh,
    reference_alloc: *reference_tile_cache.TileCacheAlloc,
    reference_compressor: *reference_tile_cache.Compressor,
    reference_cache: *reference_tile_cache.TileCache,
    reference_nav: *reference.NavMesh,

    /// Each layer's tile in both caches.
    refs: []u32,
    reference_refs: []u32,
    /// Where the obstacles stand: ground points spread over the level.
    obstacle_points: [obstacle_count][3]f32,
    obstacle_refs: [obstacle_count]u32,
    reference_obstacle_refs: [obstacle_count]u32,
    /// The boxes of the query row.
    box_mins: [box_count][3]f32,
    box_maxs: [box_count][3]f32,

    fn init(self: *Context, gpa: std.mem.Allocator) !void {
        var config = support.solo_config;
        config.tile_size = tile_size;
        self.level = try .init(gpa, support.nav_test_obj, config, .{ .seed = 0x7ac4e });
        errdefer self.level.deinit();

        try self.packLayers(gpa);
        errdefer self.freeLayers(gpa);

        const origin = self.level.mesh.bounds_min;
        const params: tile_cache.TileCacheParams = .{
            .origin = origin,
            .cell_size = cell_size,
            .cell_height = cell_height,
            .width = tile_size,
            .height = tile_size,
            .walkable_height = 2.0,
            .walkable_radius = 0.6,
            .walkable_climb = 0.9,
            .max_simplification_error = 1.3,
            .tile_count_max = @intCast(std.math.ceilPowerOfTwoAssert(usize, self.layers.len)),
            .obstacle_count_max = 128,
        };
        const tile_side = @as(f32, tile_size) * cell_size;
        const nav_params: detour.NavMeshParams = .{
            .origin = origin,
            .tile_width = tile_side,
            .tile_height = tile_side,
            .tile_count_max = params.tile_count_max,
            .poly_count_max = 1 << 14,
        };

        self.scratch = .{ .allocator = std.heap.c_allocator };
        self.cache = .empty;
        self.nav = .empty;
        if (self.nav.init(std.heap.c_allocator, &nav_params).failed) return error.OutOfMemory;
        errdefer self.nav.deinit();
        const compressor: detour.Compressor = .{
            .ptr = &compressor_anchor,
            .vtable = &port_compressor_vtable,
        };
        const initialised = self.cache.init(
            std.heap.c_allocator,
            &params,
            &self.scratch,
            compressor,
            null,
        );
        if (initialised.failed) return error.CacheInitFailed;
        errdefer self.cache.deinit();

        // The original's own allocator: `malloc` and `free`, no reset.
        self.reference_alloc = reference_tile_cache.ref_tc_alloc_create_default() orelse
            return error.OutOfMemory;
        errdefer reference_tile_cache.ref_tc_alloc_destroy(self.reference_alloc);
        self.reference_compressor = reference_tile_cache.ref_tc_compressor_create(
            referenceMaxCompressedSize,
            referenceCompress,
            referenceDecompress,
            null,
        ) orelse return error.OutOfMemory;
        errdefer reference_tile_cache.ref_tc_compressor_destroy(self.reference_compressor);
        self.reference_cache = reference_tile_cache.ref_tc_create() orelse
            return error.OutOfMemory;
        errdefer reference_tile_cache.ref_tc_destroy(self.reference_cache);
        self.reference_nav = reference.ref_nav_mesh_alloc() orelse return error.OutOfMemory;
        errdefer reference.ref_nav_mesh_free(self.reference_nav);
        const reference_nav_params: reference.NavMeshParams = .{
            .orig = origin,
            .tile_width = tile_side,
            .tile_height = tile_side,
            .max_tiles = nav_params.tile_count_max,
            .max_polys = nav_params.poly_count_max,
        };
        if (reference.ref_nav_mesh_init(self.reference_nav, &reference_nav_params) &
            success_bits == 0) return error.OutOfMemory;
        const reference_params: reference_tile_cache.TileCacheParams = .{
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
        const reference_initialised = reference_tile_cache.ref_tc_init(
            self.reference_cache,
            &reference_params,
            self.reference_alloc,
            self.reference_compressor,
            null,
        );
        if (reference_initialised & success_bits == 0) return error.CacheInitFailed;

        self.refs = try gpa.alloc(u32, self.layers.len);
        errdefer gpa.free(self.refs);
        self.reference_refs = try gpa.alloc(u32, self.layers.len);
        errdefer gpa.free(self.reference_refs);
        for (self.layers, self.refs, self.reference_refs) |layer, *ref, *reference_ref| {
            if (self.cache.addTile(layer.port_bytes, 0, ref).failed) return error.TileRefused;
            const added = reference_tile_cache.ref_tc_add_tile(
                self.reference_cache,
                layer.reference_bytes.ptr,
                @intCast(layer.reference_bytes.len),
                0,
                reference_ref,
            );
            if (added & success_bits == 0) return error.TileRefused;
        }
        for (self.refs, self.reference_refs) |ref, reference_ref| {
            if (self.cache.buildNavMeshTile(ref, &self.nav).failed) return error.TileNotBuilt;
            const built = reference_tile_cache.ref_tc_build_nav_mesh_tile(
                self.reference_cache,
                reference_ref,
                self.reference_nav,
            );
            if (built & success_bits == 0) return error.TileNotBuilt;
        }

        var prng: std.Random.DefaultPrng = .init(place_seed);
        const random = prng.random();
        for (&self.obstacle_points) |*point| point.* = self.level.groundPoint(random, 0);
        for (&self.box_mins, &self.box_maxs) |*low, *high| {
            const center = self.level.groundPoint(random, 0);
            const half: [3]f32 = .{ box_half_side, 2.0, box_half_side };
            for (low, high, center, half) |*l, *h, c, s| {
                l.* = c - s;
                h.* = c + s;
            }
        }
    }

    fn deinit(self: *Context, gpa: std.mem.Allocator) void {
        gpa.free(self.reference_refs);
        gpa.free(self.refs);
        reference.ref_nav_mesh_free(self.reference_nav);
        reference_tile_cache.ref_tc_destroy(self.reference_cache);
        reference_tile_cache.ref_tc_compressor_destroy(self.reference_compressor);
        reference_tile_cache.ref_tc_alloc_destroy(self.reference_alloc);
        self.cache.deinit();
        self.nav.deinit();
        self.freeLayers(gpa);
        self.level.deinit();
        self.* = undefined;
    }

    /// Packs every layer of every cell of the level with the original's
    /// builder, as a game's build step does, and keeps a copy for each side.
    fn packLayers(self: *Context, gpa: std.mem.Allocator) !void {
        const compressor = reference_tile_cache.ref_tc_compressor_create(
            referenceMaxCompressedSize,
            referenceCompress,
            referenceDecompress,
            null,
        ) orelse return error.OutOfMemory;
        defer reference_tile_cache.ref_tc_compressor_destroy(compressor);

        var layers: std.ArrayList(Layer) = .empty;
        errdefer {
            for (layers.items) |layer| {
                gpa.free(layer.port_bytes);
                gpa.free(layer.reference_bytes);
            }
            layers.deinit(gpa);
        }
        const level = &self.level;
        var tile_y: i32 = 0;
        while (tile_y < level.grid.height) : (tile_y += 1) {
            var tile_x: i32 = 0;
            while (tile_x < level.grid.width) : (tile_x += 1) {
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
                if (built_count <= 0) continue;
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
                    if (status & success_bits == 0) return error.LayerNotPacked;
                    const bytes = data.?[0..@intCast(data_size)];
                    const port_bytes = try gpa.dupe(u8, bytes);
                    errdefer gpa.free(port_bytes);
                    const reference_bytes = try gpa.dupe(u8, bytes);
                    errdefer gpa.free(reference_bytes);
                    try layers.append(gpa, .{
                        .port_bytes = port_bytes,
                        .reference_bytes = reference_bytes,
                    });
                }
            }
        }
        self.layers = try layers.toOwnedSlice(gpa);
    }

    fn freeLayers(self: *Context, gpa: std.mem.Allocator) void {
        for (self.layers) |layer| {
            gpa.free(layer.port_bytes);
            gpa.free(layer.reference_bytes);
        }
        gpa.free(self.layers);
    }

    /// The polygons of every tile of the original's navigation mesh.
    fn referencePolyCount(self: *const Context) u64 {
        const getMaxTiles = opaqueFn(reference.ref_nav_mesh_get_max_tiles);
        const getTile = opaqueFn(reference.ref_nav_mesh_get_tile);
        const tileHeader = opaqueFn(reference.ref_tile_header);
        var count: u64 = 0;
        const tile_count = getMaxTiles(self.reference_nav);
        var i: i32 = 0;
        while (i < tile_count) : (i += 1) {
            const tile = getTile(self.reference_nav, i) orelse continue;
            const header = tileHeader(tile) orelse continue;
            count +%= @as(u32, @bitCast(header.poly_count));
        }
        return count;
    }

    /// The polygons of every tile of the port's navigation mesh.
    fn portPolyCount(self: *const Context) u64 {
        var count: u64 = 0;
        for (self.nav.tiles) |*tile| {
            const header = tile.header orelse continue;
            count +%= @as(u32, @bitCast(header.poly_count));
        }
        return count;
    }

    // Every layer rebuilt into its navigation tile: decompressing, regions,
    // contours, the polygon mesh and the tile itself, which replaces the
    // tile built before.
    fn baselineBuildNavMeshTile(context: *Context) u64 {
        const buildNavMeshTile = opaqueFn(reference_tile_cache.ref_tc_build_nav_mesh_tile);
        var sum: u64 = 0;
        for (context.reference_refs) |ref| {
            sum +%= buildNavMeshTile(
                context.reference_cache,
                ref,
                context.reference_nav,
            );
        }
        return sum +% context.referencePolyCount();
    }

    fn portBuildNavMeshTile(context: *Context) u64 {
        const buildNavMeshTile = opaqueFn(detour.TileCache.buildNavMeshTile);
        var sum: u64 = 0;
        for (context.refs) |ref| {
            sum +%= statusSum(buildNavMeshTile(&context.cache, ref, &context.nav));
        }
        return sum +% context.portPolyCount();
    }

    // Obstacles of 1 m radius and 2 m height put down across the level, the
    // tiles under them rebuilt until the cache is up to date; then the
    // obstacles taken away and the tiles rebuilt again.
    fn baselineObstacles(context: *Context) u64 {
        const addObstacle = opaqueFn(reference_tile_cache.ref_tc_add_obstacle);
        const removeObstacle = opaqueFn(reference_tile_cache.ref_tc_remove_obstacle);
        const cache = context.reference_cache;
        var sum: u64 = 0;
        for (&context.obstacle_points, &context.reference_obstacle_refs) |*point, *ref| {
            sum +%= addObstacle(cache, point, 1.0, 2.0, ref);
            sum +%= ref.*;
        }
        sum +%= referenceSettle(context);
        for (context.reference_obstacle_refs) |ref| {
            sum +%= removeObstacle(cache, ref);
        }
        sum +%= referenceSettle(context);
        return sum +% context.referencePolyCount();
    }

    fn referenceSettle(context: *Context) u64 {
        const update = opaqueFn(reference_tile_cache.ref_tc_update);
        var sum: u64 = 0;
        for (0..update_call_max) |_| {
            var up_to_date = false;
            sum +%= update(
                context.reference_cache,
                0,
                context.reference_nav,
                &up_to_date,
            );
            if (up_to_date) break;
        }
        return sum;
    }

    fn portObstacles(context: *Context) u64 {
        const addObstacle = opaqueFn(detour.TileCache.addObstacle);
        const removeObstacle = opaqueFn(detour.TileCache.removeObstacle);
        const cache = &context.cache;
        var sum: u64 = 0;
        for (context.obstacle_points, &context.obstacle_refs) |point, *ref| {
            sum +%= statusSum(addObstacle(cache, point, 1.0, 2.0, ref));
            sum +%= ref.*;
        }
        sum +%= portSettle(context);
        for (context.obstacle_refs) |ref| sum +%= statusSum(removeObstacle(cache, ref));
        sum +%= portSettle(context);
        return sum +% context.portPolyCount();
    }

    fn portSettle(context: *Context) u64 {
        const update = opaqueFn(detour.TileCache.update);
        var sum: u64 = 0;
        for (0..update_call_max) |_| {
            var up_to_date = false;
            sum +%= statusSum(update(&context.cache, 0, &context.nav, &up_to_date));
            if (up_to_date) break;
        }
        return sum;
    }

    // Every tile taken out of the cache and given back, its bytes staying
    // the caller's.
    fn baselineAddRemoveTile(context: *Context) u64 {
        const removeTile = opaqueFn(reference_tile_cache.ref_tc_remove_tile);
        const addTile = opaqueFn(reference_tile_cache.ref_tc_add_tile);
        var sum: u64 = 0;
        for (context.layers, context.reference_refs) |layer, *ref| {
            const cache = context.reference_cache;
            sum +%= removeTile(cache, ref.*, null, null);
            sum +%= addTile(
                context.reference_cache,
                layer.reference_bytes.ptr,
                @intCast(layer.reference_bytes.len),
                0,
                ref,
            );
            sum +%= ref.*;
        }
        return sum;
    }

    fn portAddRemoveTile(context: *Context) u64 {
        const removeTile = opaqueFn(detour.TileCache.removeTile);
        const addTile = opaqueFn(detour.TileCache.addTile);
        var sum: u64 = 0;
        for (context.layers, context.refs) |layer, *ref| {
            sum +%= statusSum(removeTile(&context.cache, ref.*, null));
            sum +%= statusSum(addTile(&context.cache, layer.port_bytes, 0, ref));
            sum +%= ref.*;
        }
        return sum;
    }

    // The tiles a box of 10 m by 10 m around each ground point touches.
    fn baselineQueryTiles(context: *Context) u64 {
        const queryTiles = opaqueFn(reference_tile_cache.ref_tc_query_tiles);
        var sum: u64 = 0;
        var results: [64]u32 = undefined;
        var count: i32 = 0;
        for (&context.box_mins, &context.box_maxs) |*low, *high| {
            sum +%= queryTiles(
                context.reference_cache,
                low,
                high,
                &results,
                &count,
                results.len,
            );
            sum +%= @as(u32, @bitCast(count));
            if (count > 0) sum +%= results[@intCast(count - 1)];
        }
        return sum;
    }

    fn portQueryTiles(context: *Context) u64 {
        const queryTiles = opaqueFn(detour.TileCache.queryTiles);
        var sum: u64 = 0;
        var results: [64]u32 = undefined;
        var count: usize = 0;
        for (&context.box_mins, &context.box_maxs) |*low, *high| {
            sum +%= statusSum(queryTiles(&context.cache, low.*, high.*, &results, &count));
            sum +%= count;
            if (count > 0) sum +%= results[count - 1];
        }
        return sum;
    }
};

pub fn run(bench: *Bench) !void {
    try bench.group("Tile cache");

    // The tiled level in cells of 48, every layer of every cell packed by
    // the original's builder with a compressor that stores bytes as they
    // are, on both sides.
    const context = try bench.gpa.create(Context);
    defer bench.gpa.destroy(context);
    try context.init(bench.gpa);
    defer context.deinit(bench.gpa);

    try bench.compare(
        .{ .name = "TileCache.buildNavMeshTile", .per = "tile", .count = context.layers.len },
        context,
        Context.baselineBuildNavMeshTile,
        Context.portBuildNavMeshTile,
    );
    try bench.compare(
        .{
            .name = "TileCache.addObstacle + removeObstacle + update",
            .per = "obstacle",
            .count = obstacle_count,
        },
        context,
        Context.baselineObstacles,
        Context.portObstacles,
    );
    try bench.compare(
        .{ .name = "TileCache.queryTiles", .per = "query", .count = box_count },
        context,
        Context.baselineQueryTiles,
        Context.portQueryTiles,
    );
    // Last: each call moves every tile's salt on, and the two sides are
    // called a different number of times, so their references part after
    // this row.
    try bench.compare(
        .{ .name = "TileCache.removeTile + addTile", .per = "tile", .count = context.layers.len },
        context,
        Context.baselineAddRemoveTile,
        Context.portAddRemoveTile,
    );
}
