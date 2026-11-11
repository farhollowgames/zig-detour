//! Differential tests of the tile cache builder: packed layers, unpacked
//! grids, regions, contours, polygon meshes and marked areas, compared byte
//! for byte with the original's.

const std = @import("std");
const detour = @import("detour");
const support = @import("support.zig");
const reference = support.reference_tile_cache;
const reference_recast = support.reference_recast;

const testing = std.testing;
const builder = detour.tile_cache_builder;
const Status = detour.Status;

/// The bytes a packed layer's header takes.
const header_size = 56;

const out_of_memory: Status = .{ .failed = true, .out_of_memory = true };
const buffer_too_small: Status = .{ .failed = true, .buffer_too_small = true };
const invalid_param: Status = .{ .failed = true, .invalid_param = true };
const wrong_magic: Status = .{ .failed = true, .wrong_magic = true };
const wrong_version: Status = .{ .failed = true, .wrong_version = true };

// ---------------------------------------------------------------------------
// The codec both sides store grids with
// ---------------------------------------------------------------------------

const CodecKind = enum {
    /// The bytes as they are.
    store,
    /// Pairs of a run length, 1 to 255, and the byte the run repeats.
    run_length,
};

/// The sizes a codec was last called with, -1 for a call that never came.
const CodecRequests = struct {
    size_max_input: i32 = -1,
    compress_input: i32 = -1,
    compress_room: i32 = -1,
    decompress_input: i32 = -1,
    decompress_room: i32 = -1,
};

/// One codec, the same code behind the port's `Compressor` and the
/// original's `dtTileCacheCompressor`, so the bytes either side stores can
/// only differ through what the builder itself does.
const Codec = struct {
    kind: CodecKind = .store,
    /// What `compress` returns: at once when it has failed, after the work
    /// otherwise.
    compress_status: Status = .success,
    /// The same for `decompress`.
    decompress_status: Status = .success,
    requests: CodecRequests = .{},

    const port_vtable: builder.Compressor.VTable = .{
        .maxCompressedSize = portSizeMax,
        .compress = portCompress,
        .decompress = portDecompress,
    };

    fn compressor(self: *Codec) builder.Compressor {
        return .{ .ptr = self, .vtable = &port_vtable };
    }

    fn sizeMax(self: *Codec, buffer_size: i32) i32 {
        self.requests.size_max_input = buffer_size;
        return switch (self.kind) {
            .store => buffer_size,
            .run_length => buffer_size * 2,
        };
    }

    fn compress(self: *Codec, buffer: []const u8, compressed: []u8, compressed_size: *i32) Status {
        self.requests.compress_input = @intCast(buffer.len);
        self.requests.compress_room = @intCast(compressed.len);
        if (self.compress_status.failed) return self.compress_status;
        switch (self.kind) {
            .store => {
                if (compressed.len < buffer.len) return buffer_too_small;
                @memcpy(compressed[0..buffer.len], buffer);
                compressed_size.* = @intCast(buffer.len);
            },
            .run_length => {
                var used: usize = 0;
                var i: usize = 0;
                while (i < buffer.len) {
                    var run: usize = 1;
                    while (run < 255 and i + run < buffer.len and buffer[i + run] == buffer[i]) {
                        run += 1;
                    }
                    if (used + 2 > compressed.len) return buffer_too_small;
                    compressed[used] = @intCast(run);
                    compressed[used + 1] = buffer[i];
                    used += 2;
                    i += run;
                }
                compressed_size.* = @intCast(used);
            },
        }
        return self.compress_status;
    }

    /// Fails with `buffer_too_small` when the bytes do not fit `buffer`,
    /// and with `invalid_param` on a run that is cut short or empty. What
    /// it wrote before failing stays written.
    fn decompress(self: *Codec, compressed: []const u8, buffer: []u8, buffer_size: *i32) Status {
        self.requests.decompress_input = @intCast(compressed.len);
        self.requests.decompress_room = @intCast(buffer.len);
        if (self.decompress_status.failed) return self.decompress_status;
        switch (self.kind) {
            .store => {
                if (buffer.len < compressed.len) return buffer_too_small;
                @memcpy(buffer[0..compressed.len], compressed);
                buffer_size.* = @intCast(compressed.len);
            },
            .run_length => {
                if (compressed.len % 2 != 0) return invalid_param;
                var used: usize = 0;
                var i: usize = 0;
                while (i < compressed.len) : (i += 2) {
                    const run: usize = compressed[i];
                    if (run == 0) return invalid_param;
                    if (used + run > buffer.len) return buffer_too_small;
                    @memset(buffer[used..][0..run], compressed[i + 1]);
                    used += run;
                }
                buffer_size.* = @intCast(used);
            },
        }
        return self.decompress_status;
    }

    fn portSizeMax(ptr: *anyopaque, buffer_size: i32) i32 {
        const self: *Codec = @ptrCast(@alignCast(ptr));
        return self.sizeMax(buffer_size);
    }

    fn portCompress(
        ptr: *anyopaque,
        buffer: []const u8,
        compressed: []u8,
        compressed_size: *i32,
    ) Status {
        const self: *Codec = @ptrCast(@alignCast(ptr));
        return self.compress(buffer, compressed, compressed_size);
    }

    fn portDecompress(
        ptr: *anyopaque,
        compressed: []const u8,
        buffer: []u8,
        buffer_size: *i32,
    ) Status {
        const self: *Codec = @ptrCast(@alignCast(ptr));
        return self.decompress(compressed, buffer, buffer_size);
    }

    fn referenceSizeMax(user: ?*anyopaque, buffer_size: i32) callconv(.c) i32 {
        const self: *Codec = @ptrCast(@alignCast(user.?));
        return self.sizeMax(buffer_size);
    }

    fn referenceCompress(
        user: ?*anyopaque,
        buffer: [*]const u8,
        buffer_size: i32,
        compressed: [*]u8,
        max_compressed_size: i32,
        compressed_size: *i32,
    ) callconv(.c) u32 {
        const self: *Codec = @ptrCast(@alignCast(user.?));
        return @bitCast(self.compress(
            buffer[0..@intCast(buffer_size)],
            compressed[0..@intCast(max_compressed_size)],
            compressed_size,
        ));
    }

    fn referenceDecompress(
        user: ?*anyopaque,
        compressed: [*]const u8,
        compressed_size: i32,
        buffer: [*]u8,
        max_buffer_size: i32,
        buffer_size: *i32,
    ) callconv(.c) u32 {
        const self: *Codec = @ptrCast(@alignCast(user.?));
        return @bitCast(self.decompress(
            compressed[0..@intCast(compressed_size)],
            buffer[0..@intCast(max_buffer_size)],
            buffer_size,
        ));
    }
};

/// The same codec twice, one a side, so what each side asked of it can be
/// compared. Initialise in place: the original's compressor points at
/// `original`.
const CodecPair = struct {
    port: Codec,
    original: Codec,
    reference_compressor: *reference.Compressor,

    fn init(self: *CodecPair, kind: CodecKind) !void {
        self.* = .{
            .port = .{ .kind = kind },
            .original = .{ .kind = kind },
            .reference_compressor = undefined,
        };
        self.reference_compressor = reference.ref_tc_compressor_create(
            Codec.referenceSizeMax,
            Codec.referenceCompress,
            Codec.referenceDecompress,
            &self.original,
        ) orelse return error.OutOfMemory;
    }

    fn deinit(self: *CodecPair) void {
        reference.ref_tc_compressor_destroy(self.reference_compressor);
        self.* = undefined;
    }

    fn setKind(self: *CodecPair, kind: CodecKind) void {
        self.port.kind = kind;
        self.original.kind = kind;
    }

    fn setStatuses(self: *CodecPair, compress_status: Status, decompress_status: Status) void {
        self.port.compress_status = compress_status;
        self.original.compress_status = compress_status;
        self.port.decompress_status = decompress_status;
        self.original.decompress_status = decompress_status;
    }

    fn clearRequests(self: *CodecPair) void {
        self.port.requests = .{};
        self.original.requests = .{};
    }

    fn expectSameRequests(self: *const CodecPair) !void {
        try testing.expectEqual(self.original.requests, self.port.requests);
    }
};

// ---------------------------------------------------------------------------
// Layers to build from
// ---------------------------------------------------------------------------

/// One layer before packing: what `buildLayer` takes.
const LayerInput = struct {
    header: builder.LayerHeader,
    heights: []u8,
    areas: []u8,
    connections: []u8,

    fn cellCount(self: *const LayerInput) usize {
        return @as(usize, self.header.width) * self.header.height;
    }

    /// A layer of `width` by `height` cells with zeroed grids. Free with
    /// `deinit`.
    fn initBlank(gpa: std.mem.Allocator, width: u8, height: u8) !LayerInput {
        const cell_count = @as(usize, width) * height;
        const heights = try gpa.alloc(u8, cell_count);
        errdefer gpa.free(heights);
        const areas = try gpa.alloc(u8, cell_count);
        errdefer gpa.free(areas);
        const connections = try gpa.alloc(u8, cell_count);
        @memset(heights, 0);
        @memset(areas, 0);
        @memset(connections, 0);
        return .{
            .header = .{
                .magic = builder.tile_cache_magic,
                .version = builder.tile_cache_version,
                .tile_x = 0,
                .tile_y = 0,
                .tile_layer = 0,
                .bounds_min = .{ 0, 0, 0 },
                .bounds_max = .{ 0, 0, 0 },
                .height_min = 0,
                .height_max = 0,
                .width = width,
                .height = height,
                .x_min = 0,
                .x_max = width -| 1,
                .y_min = 0,
                .y_max = height -| 1,
            },
            .heights = heights,
            .areas = areas,
            .connections = connections,
        };
    }

    fn deinit(self: *LayerInput, gpa: std.mem.Allocator) void {
        gpa.free(self.heights);
        gpa.free(self.areas);
        gpa.free(self.connections);
        self.* = undefined;
    }

    /// Sets every connection bit that leads to a walkable cell within
    /// `climb` height units, as Recast's layers have them. Bits never lead
    /// out of the grid, which neither side's contour walk survives.
    fn connect(self: *LayerInput, climb: i32) void {
        const w: i32 = self.header.width;
        const h: i32 = self.header.height;
        const offset_x = [4]i32{ -1, 0, 1, 0 };
        const offset_y = [4]i32{ 0, 1, 0, -1 };
        var y: i32 = 0;
        while (y < h) : (y += 1) {
            var x: i32 = 0;
            while (x < w) : (x += 1) {
                const cell: usize = @intCast(x + y * w);
                var bits: u8 = 0;
                for (0..4) |dir| {
                    const nx = x + offset_x[dir];
                    const ny = y + offset_y[dir];
                    if (nx < 0 or ny < 0 or nx >= w or ny >= h) continue;
                    const neighbor: usize = @intCast(nx + ny * w);
                    if (self.areas[cell] == builder.null_area) continue;
                    if (self.areas[neighbor] == builder.null_area) continue;
                    const step = @as(i32, self.heights[cell]) - @as(i32, self.heights[neighbor]);
                    if (@abs(step) > climb) continue;
                    bits |= @as(u8, 1) << @intCast(dir);
                }
                self.connections[cell] = bits;
            }
        }
    }
};

// ---------------------------------------------------------------------------
// Working memory and the two sides' results
// ---------------------------------------------------------------------------

/// Working memory for the build steps of both sides: a bump allocator over
/// a fixed buffer for the port, the original's linear allocator for the
/// original. Do not move it between `allocator` and the last use of what
/// that returned.
const Scratch = struct {
    gpa: std.mem.Allocator,
    buffer: []align(16) u8,
    fixed: std.heap.FixedBufferAllocator,
    original: *reference.TileCacheAlloc,
    /// The port's allocator in place of the fixed buffer's, for a test
    /// that counts the port's allocations or makes them fail. `reset` does
    /// not free what came from it.
    port_override: ?std.mem.Allocator = null,

    const default_capacity = 1 << 20;

    fn init(gpa: std.mem.Allocator, capacity: usize) !Scratch {
        const buffer = try gpa.alignedAlloc(u8, .@"16", capacity);
        errdefer gpa.free(buffer);
        const original = reference.ref_tc_alloc_create_linear(capacity) orelse
            return error.OutOfMemory;
        return .{ .gpa = gpa, .buffer = buffer, .fixed = .init(buffer), .original = original };
    }

    fn deinit(self: *Scratch) void {
        reference.ref_tc_alloc_destroy(self.original);
        self.gpa.free(self.buffer);
        self.* = undefined;
    }

    fn allocator(self: *Scratch) std.mem.Allocator {
        return self.port_override orelse self.fixed.allocator();
    }

    /// Starts both sides over. Everything allocated from either is gone.
    fn reset(self: *Scratch) void {
        self.fixed.reset();
        reference.ref_tc_alloc_reset(self.original);
    }
};

/// One layer unpacked on both sides.
const LayerPair = struct {
    port: *builder.Layer,
    original: *reference.Layer,

    fn cellCount(self: LayerPair) usize {
        return self.port.heights.len;
    }
};

/// Compares the header, the four grids and the region count.
fn expectSameLayer(pair: LayerPair) !void {
    const reference_header = reference.ref_tcb_layer_header(pair.original);
    try support.expectBytes(
        std.mem.asBytes(reference_header)[0..header_size],
        std.mem.asBytes(pair.port.header)[0..header_size],
    );
    const cell_count = @as(usize, reference_header.width) * reference_header.height;
    try testing.expectEqual(cell_count, pair.port.heights.len);
    try testing.expectEqual(cell_count, pair.port.areas.len);
    try testing.expectEqual(cell_count, pair.port.connections.len);
    try testing.expectEqual(cell_count, pair.port.regions.len);
    try testing.expectEqual(
        reference.ref_tcb_layer_reg_count(pair.original),
        pair.port.region_count,
    );
    try support.expectBytes(
        reference.ref_tcb_layer_heights(pair.original)[0..cell_count],
        pair.port.heights,
    );
    try support.expectBytes(
        reference.ref_tcb_layer_areas(pair.original)[0..cell_count],
        pair.port.areas,
    );
    try support.expectBytes(
        reference.ref_tcb_layer_cons(pair.original)[0..cell_count],
        pair.port.connections,
    );
    try support.expectBytes(
        reference.ref_tcb_layer_regs(pair.original)[0..cell_count],
        pair.port.regions,
    );
}

/// Packs `input` on both sides and compares everything `buildLayer` gives
/// back, and what it asked of the codec. Returns the port's block, which
/// the caller frees with `gpa.free`, or null when both sides failed alike
/// and left their outputs alone.
fn packBoth(gpa: std.mem.Allocator, codecs: *CodecPair, input: *const LayerInput) !?[]u8 {
    // The original stores its header's two padding bytes as it finds them,
    // the port stores zeros: give the original a header padded with zeros.
    var reference_header_bytes: [header_size]u8 align(4) = @splat(0);
    const field_size = @offsetOf(builder.LayerHeader, "y_max") + 1;
    @memcpy(reference_header_bytes[0..field_size], std.mem.asBytes(&input.header)[0..field_size]);
    const reference_header: *reference.TileCacheLayerHeader = @ptrCast(&reference_header_bytes);

    var marker: [1]u8 = .{0};
    var reference_data: ?[*]u8 = &marker;
    var reference_size: i32 = -77;
    var port_data: []u8 = &marker;

    codecs.clearRequests();
    const reference_status = reference.ref_tcb_build_layer(
        codecs.reference_compressor,
        reference_header,
        input.heights.ptr,
        input.areas.ptr,
        input.connections.ptr,
        &reference_data,
        &reference_size,
    );
    const port_status = builder.buildLayer(
        gpa,
        codecs.port.compressor(),
        &input.header,
        input.heights,
        input.areas,
        input.connections,
        &port_data,
    );
    try support.expectStatus(reference_status, port_status);
    try codecs.expectSameRequests();
    if (port_status.failed) {
        try testing.expectEqual(@as(?[*]u8, &marker), reference_data);
        try testing.expectEqual(@as(i32, -77), reference_size);
        try testing.expectEqual(@as([*]u8, &marker), port_data.ptr);
        try testing.expectEqual(@as(usize, 1), port_data.len);
        return null;
    }
    defer reference.ref_tcb_free_data(reference_data);
    errdefer gpa.free(port_data);
    try testing.expectEqual(@as(usize, @intCast(reference_size)), port_data.len);
    try support.expectBytes(reference_data.?[0..port_data.len], port_data);
    return port_data;
}

/// Unpacks `blob` on both sides and compares everything `decompressLayer`
/// gives back, and what it asked of the codec. Null when both sides failed
/// alike. `blob` holds at least a header: the original reads one without
/// looking at the length.
fn unpackBoth(scratch: *Scratch, codecs: *CodecPair, blob: []const u8) !?LayerPair {
    var status: Status = undefined;
    return unpackBothStatus(scratch, codecs, blob, &status);
}

/// `unpackBoth` that also gives the status both sides returned.
fn unpackBothStatus(
    scratch: *Scratch,
    codecs: *CodecPair,
    blob: []const u8,
    status_out: *Status,
) !?LayerPair {
    std.debug.assert(blob.len >= header_size);
    const gpa = testing.allocator;
    // The original reads the header in place, as a struct.
    const copy = try gpa.alignedAlloc(u8, .@"4", blob.len);
    defer gpa.free(copy);
    @memcpy(copy, blob);

    var dummy: builder.Layer = undefined;
    var reference_layer: ?*reference.Layer = @ptrCast(&dummy);
    var port_layer: ?*builder.Layer = &dummy;

    codecs.clearRequests();
    const reference_status = reference.ref_tcb_decompress_layer(
        scratch.original,
        codecs.reference_compressor,
        copy.ptr,
        @intCast(copy.len),
        &reference_layer,
    );
    const port_status = builder.decompressLayer(
        scratch.allocator(),
        codecs.port.compressor(),
        blob,
        &port_layer,
    );
    try support.expectStatus(reference_status, port_status);
    status_out.* = port_status;
    try codecs.expectSameRequests();
    if (port_status.failed) {
        try testing.expectEqual(@as(?*reference.Layer, null), reference_layer);
        try testing.expectEqual(@as(?*builder.Layer, null), port_layer);
        return null;
    }
    const pair: LayerPair = .{ .port = port_layer.?, .original = reference_layer.? };
    try expectSameLayer(pair);
    return pair;
}

/// The cell sizes and the climb, in cells, the real layers are built with.
const real_cell_size: f32 = 0.3;
const real_cell_height: f32 = 0.2;
const real_climb: i32 = 4;
const real_tile_size = 48;

var real_layer_list: ?std.ArrayList(LayerInput) = null;

/// Heightfield layers of the three test levels, as the original Recast
/// builds them, portals and all. They are built once and read by every
/// test, so they come from the page allocator and stay for the whole run.
fn realLayers() ![]const LayerInput {
    if (real_layer_list) |*list| return list.items;
    var list: std.ArrayList(LayerInput) = .empty;
    try appendLevelLayers(&list, support.dungeon_obj, 12);
    try appendLevelLayers(&list, support.nav_test_obj, 12);
    try appendLevelLayers(&list, support.undulating_obj, 6);
    real_layer_list = list;
    return real_layer_list.?.items;
}

/// Appends every layer of about `tile_count_max` tiles spread over the level.
fn appendLevelLayers(
    list: *std.ArrayList(LayerInput),
    obj_text: []const u8,
    tile_count_max: usize,
) !void {
    const keep = std.heap.page_allocator;
    var config = support.solo_config;
    config.cell_size = real_cell_size;
    config.cell_height = real_cell_height;
    config.tile_size = real_tile_size;
    var level: support.Level = try .init(testing.allocator, obj_text, config, .{});
    defer level.deinit();

    const grid_width: usize = @intCast(level.grid.width);
    const grid_height: usize = @intCast(level.grid.height);
    const tile_count = grid_width * grid_height;
    var step = @max(1, tile_count / tile_count_max);
    // A step the grid's width divides would visit one column only.
    if (step > 1 and step % grid_width == 0) step += 1;

    var tile: usize = 0;
    while (tile < tile_count) : (tile += step) {
        var recast_layers: [32]reference_recast.RecastLayer = undefined;
        var owner: ?*reference_recast.LayerOwner = null;
        const count = reference_recast.ref_recast_build_layers(
            level.mesh.verts.ptr,
            level.mesh.vertexCount(),
            level.mesh.tris.ptr,
            level.mesh.triangleCount(),
            &config,
            &level.mesh.bounds_min,
            &level.mesh.bounds_max,
            @intCast(tile % grid_width),
            @intCast(tile / grid_width),
            &recast_layers,
            recast_layers.len,
            &owner,
        );
        defer reference_recast.ref_recast_free_layers(owner);

        for (recast_layers[0..@intCast(count)]) |*recast_layer| {
            var input: LayerInput = try .initBlank(keep, recast_layer.width, recast_layer.height);
            errdefer input.deinit(keep);
            input.header.tile_x = recast_layer.tile_x;
            input.header.tile_y = recast_layer.tile_y;
            input.header.tile_layer = recast_layer.layer;
            input.header.bounds_min = recast_layer.bmin;
            input.header.bounds_max = recast_layer.bmax;
            input.header.height_min = recast_layer.hmin;
            input.header.height_max = recast_layer.hmax;
            input.header.x_min = recast_layer.minx;
            input.header.x_max = recast_layer.maxx;
            input.header.y_min = recast_layer.miny;
            input.header.y_max = recast_layer.maxy;
            const cell_count = input.cellCount();
            if (cell_count > 0) {
                @memcpy(input.heights, recast_layer.heights.?[0..cell_count]);
                @memcpy(input.areas, recast_layer.areas.?[0..cell_count]);
                @memcpy(input.connections, recast_layer.cons.?[0..cell_count]);
            }
            try list.append(keep, input);
        }
    }
}

/// The real layer with the most cells.
fn largestRealLayer() !*const LayerInput {
    const layers = try realLayers();
    var largest = &layers[0];
    for (layers) |*layer| {
        if (layer.cellCount() > largest.cellCount()) largest = layer;
    }
    return largest;
}

// ---------------------------------------------------------------------------
// Contours and polygon meshes of the two sides
// ---------------------------------------------------------------------------

const ContourSetPair = struct {
    port: *builder.ContourSet,
    original: *reference.ContourSet,
};

const PolyMeshPair = struct {
    port: *builder.PolyMesh,
    original: *reference.PolyMesh,
};

/// Compares every contour: its vertex count, region, area and each of its
/// vertex bytes. Only for sets whose contour array the original allocated.
fn expectSameContourSet(pair: ContourSetPair) !void {
    const count = reference.ref_tcb_contour_set_count(pair.original);
    try testing.expectEqual(count, pair.port.contour_count);
    try testing.expectEqual(@as(usize, @intCast(count)), pair.port.contours.len);
    for (pair.port.contours, 0..) |contour, i| {
        errdefer std.debug.print("  contour {d} of {d}\n", .{ i, count });
        const index: i32 = @intCast(i);
        try testing.expectEqual(
            reference.ref_tcb_contour_nverts(pair.original, index),
            contour.vertex_count,
        );
        try testing.expectEqual(
            reference.ref_tcb_contour_reg(pair.original, index),
            contour.region,
        );
        try testing.expectEqual(reference.ref_tcb_contour_area(pair.original, index), contour.area);
        const reference_vertices = reference.ref_tcb_contour_verts(pair.original, index) orelse {
            try testing.expectEqual(@as(usize, 0), contour.vertices.len);
            continue;
        };
        const byte_count = @as(usize, @intCast(contour.vertex_count)) * 4;
        try support.expectBytes(reference_vertices[0..byte_count], contour.vertices);
    }
}

/// Compares the whole mesh: the three counts, then every element of the
/// four arrays up to the capacity both sides size them to from the
/// contours, so what lies past the counts is compared too.
fn expectSamePolyMesh(sets: ContourSetPair, pair: PolyMeshPair) !void {
    const original = pair.original;
    const port = pair.port;
    try testing.expectEqual(reference.ref_tcb_poly_mesh_nvp(original), port.vertices_per_polygon);
    try testing.expectEqual(reference.ref_tcb_poly_mesh_nverts(original), port.vertex_count);
    try testing.expectEqual(reference.ref_tcb_poly_mesh_npolys(original), port.polygon_count);

    var vertex_capacity: usize = 0;
    var polygon_capacity: usize = 0;
    const contour_count = reference.ref_tcb_contour_set_count(sets.original);
    var i: i32 = 0;
    while (i < contour_count) : (i += 1) {
        const vertex_count = reference.ref_tcb_contour_nverts(sets.original, i);
        if (vertex_count < 3) continue;
        vertex_capacity += @intCast(vertex_count);
        polygon_capacity += @intCast(vertex_count - 2);
    }
    try testing.expectEqual(vertex_capacity * 3, port.vertices.len);
    try testing.expectEqual(polygon_capacity * 12, port.polygons.len);
    try testing.expectEqual(polygon_capacity, port.flags.len);
    try testing.expectEqual(polygon_capacity, port.areas.len);
    // Nothing to read: the original's arrays are empty allocations.
    if (vertex_capacity == 0) return;

    const vertices = reference.ref_tcb_poly_mesh_verts(original).?;
    const polygons = reference.ref_tcb_poly_mesh_polys(original).?;
    const flags = reference.ref_tcb_poly_mesh_flags(original).?;
    const areas = reference.ref_tcb_poly_mesh_areas(original).?;
    const vertices_used = @as(usize, @intCast(port.vertex_count)) * 3;
    const polygons_used: usize = @intCast(port.polygon_count);

    // What is in use first, so a failure says which part differs.
    try testing.expectEqualSlices(u16, vertices[0..vertices_used], port.vertices[0..vertices_used]);
    try testing.expectEqualSlices(
        u16,
        polygons[0 .. polygons_used * 12],
        port.polygons[0 .. polygons_used * 12],
    );
    try testing.expectEqualSlices(u16, flags[0..polygons_used], port.flags[0..polygons_used]);
    try support.expectBytes(areas[0..polygons_used], port.areas[0..polygons_used]);

    errdefer std.debug.print("  the difference lies past the mesh's counts\n", .{});
    try testing.expectEqualSlices(u16, vertices[0..port.vertices.len], port.vertices);
    try testing.expectEqualSlices(u16, polygons[0..port.polygons.len], port.polygons);
    try testing.expectEqualSlices(u16, flags[0..port.flags.len], port.flags);
    try support.expectBytes(areas[0..port.areas.len], port.areas);
}

// ---------------------------------------------------------------------------
// The build steps on both sides
// ---------------------------------------------------------------------------

fn statusBits(status: Status) u32 {
    return @bitCast(status);
}

/// How `buildRegionsBoth` ended.
const RegionsOutcome = enum {
    /// Both sides built the same regions.
    built,
    /// Both sides ran out of region ids and left the same grid behind.
    failed,
    /// The port refused a region with more than sixteen neighbours. The
    /// original writes past its neighbour array there, so it was not run.
    refused,
};

/// Builds the regions on both sides, the port first, and compares the
/// status and the whole layer afterwards.
fn buildRegionsBoth(scratch: *Scratch, pair: LayerPair, climb: i32) !RegionsOutcome {
    var counting: std.testing.FailingAllocator = .init(scratch.allocator(), .{});
    const port_status = builder.buildRegions(counting.allocator(), pair.port, climb);
    // Running out of ids ends the build before its second allocation; the
    // refusal of a seventeenth neighbour comes after it.
    if (statusBits(port_status) == statusBits(buffer_too_small) and counting.allocations == 2) {
        return .refused;
    }
    const reference_status = reference.ref_tcb_build_regions(
        scratch.original,
        pair.original,
        climb,
    );
    try support.expectStatus(reference_status, port_status);
    try expectSameLayer(pair);
    return if (port_status.failed) .failed else .built;
}

/// Whether both sides can trace the layer's contours.
///
/// A walkable layer of a single cell makes both divide by zero: the walk
/// ends after one vertex, drops it as a duplicate of itself, and the
/// simplification takes a remainder by the vertex count. And a region id
/// from 0xf8 up is also what a portal reads as: the walk takes a portal
/// for more of such a region and steps through it, out of the grid, and
/// across a connection the region is taken for a portal. No layer the
/// builder is meant for has that many regions.
fn contoursAreSafe(pair: LayerPair) bool {
    if (pair.cellCount() == 1 and pair.port.region_count > 0) return false;
    if (pair.port.region_count > 0xf8) {
        for (pair.port.connections) |connection| {
            if (connection != 0) return false;
        }
    }
    return true;
}

/// Builds the contours of a layer whose regions are built, on both sides,
/// and compares the status, every contour and the layer, which is input
/// only. Null when the contours were not built: unsafe to trace, or both
/// sides failed alike.
fn buildContoursBoth(
    scratch: *Scratch,
    pair: LayerPair,
    climb: i32,
    max_error: f32,
) !?ContourSetPair {
    if (!contoursAreSafe(pair)) return null;
    const sets: ContourSetPair = .{
        .original = reference.ref_tcb_alloc_contour_set(scratch.original) orelse
            return error.OutOfMemory,
        .port = builder.allocContourSet(scratch.allocator()) orelse return error.OutOfMemory,
    };
    const reference_status = reference.ref_tcb_build_contours(
        scratch.original,
        pair.original,
        climb,
        max_error,
        sets.original,
    );
    const port_status = builder.buildContours(
        scratch.allocator(),
        pair.port,
        climb,
        max_error,
        sets.port,
    );
    try support.expectStatus(reference_status, port_status);
    try testing.expectEqual(
        reference.ref_tcb_contour_set_count(sets.original),
        sets.port.contour_count,
    );
    // Out of memory, the original may hold a count and no contours to read.
    if (!port_status.out_of_memory) {
        try expectSameContourSet(sets);
        try expectSameLayer(pair);
    }
    if (port_status.failed) {
        // The original's set stays as it is: its free function reads the
        // contour array, which a failure may have left unallocated.
        builder.freeContourSet(scratch.allocator(), sets.port);
        return null;
    }
    return sets;
}

/// Whether a vertex of `contour` lies where some contour of `set` has a
/// vertex the mesh may remove.
fn passesRemovableVertex(set: *const builder.ContourSet, contour: *const builder.Contour) bool {
    for (set.contours) |*other| {
        var k: usize = 0;
        while (k + 4 <= other.vertices.len) : (k += 4) {
            if (other.vertices[k + 3] & 0x80 == 0) continue;
            var j: usize = 0;
            while (j + 4 <= contour.vertices.len) : (j += 4) {
                if (contour.vertices[j] == other.vertices[k] and
                    contour.vertices[j + 2] == other.vertices[k + 2]) return true;
            }
        }
    }
    return false;
}

/// Whether the original builds the mesh of `set` within its own arrays.
///
/// Where it looks at a vertex to remove, it collects the edges at that
/// vertex, three numbers each, in an array of 48: more than sixteen edges
/// run past the end. A polygon of n corners has at most 2 (n - 2) edges at
/// one vertex, and every polygon at a removable vertex, those that fill
/// the hole of an earlier removal too, comes from the contours that pass
/// through such a vertex, which never hold more triangles than they
/// started with. So eight triangles in those contours are safe.
fn removalIsSafe(set: *const builder.ContourSet) bool {
    var triangle_count: i32 = 0;
    for (set.contours) |*contour| {
        if (contour.vertex_count < 3) continue;
        if (passesRemovableVertex(set, contour)) triangle_count += contour.vertex_count - 2;
    }
    return triangle_count <= 8;
}

/// Builds the polygon mesh of `sets` on both sides and compares the
/// status, the whole mesh and the contours, which are input only. Null
/// when both sides failed alike, and when the original cannot be run on
/// these contours: see `removalIsSafe`.
fn buildPolyMeshBoth(scratch: *Scratch, sets: ContourSetPair) !?PolyMeshPair {
    if (!removalIsSafe(sets.port)) return null;
    const meshes: PolyMeshPair = .{
        .original = reference.ref_tcb_alloc_poly_mesh(scratch.original) orelse
            return error.OutOfMemory,
        .port = builder.allocPolyMesh(scratch.allocator()) orelse return error.OutOfMemory,
    };
    const reference_status = reference.ref_tcb_build_poly_mesh(
        scratch.original,
        sets.original,
        meshes.original,
    );
    const port_status = builder.buildPolyMesh(scratch.allocator(), sets.port, meshes.port);
    try support.expectStatus(reference_status, port_status);
    if (port_status.failed) {
        // The arrays may be half made: compare what is set either way.
        try expectSameMeshCounts(meshes);
        builder.freePolyMesh(scratch.allocator(), meshes.port);
        return null;
    }
    try expectSamePolyMesh(sets, meshes);
    try expectSameContourSet(sets);
    return meshes;
}

fn expectSameMeshCounts(meshes: PolyMeshPair) !void {
    try testing.expectEqual(
        reference.ref_tcb_poly_mesh_nvp(meshes.original),
        meshes.port.vertices_per_polygon,
    );
    try testing.expectEqual(
        reference.ref_tcb_poly_mesh_nverts(meshes.original),
        meshes.port.vertex_count,
    );
    try testing.expectEqual(
        reference.ref_tcb_poly_mesh_npolys(meshes.original),
        meshes.port.polygon_count,
    );
}

/// How far `buildAllBoth` got.
const Reach = enum { refused, regions_failed, regions, contours, mesh };

/// Regions, contours and polygon mesh on both sides, each step compared in
/// full before the next runs on it.
fn buildAllBoth(scratch: *Scratch, pair: LayerPair, climb: i32, max_error: f32) !Reach {
    switch (try buildRegionsBoth(scratch, pair, climb)) {
        .refused => return .refused,
        .failed => return .regions_failed,
        .built => {},
    }
    const sets = try buildContoursBoth(scratch, pair, climb, max_error) orelse return .regions;
    _ = try buildPolyMeshBoth(scratch, sets) orelse return .contours;
    return .mesh;
}

/// Counts of how far a test's cases got, so it can require that enough of
/// them were compared to the end.
const ReachCounts = struct {
    counts: [5]u32 = @splat(0),

    fn add(self: *ReachCounts, reach: Reach) void {
        self.counts[@backingInt(reach)] += 1;
    }

    fn of(self: *const ReachCounts, reach: Reach) u32 {
        return self.counts[@backingInt(reach)];
    }
};

// ---------------------------------------------------------------------------
// Random layers
// ---------------------------------------------------------------------------

/// A width or height: now and then none, one, or the most a byte holds.
fn randomAxis(random: std.Random) u8 {
    return switch (random.uintLessThan(u8, 16)) {
        0 => 0,
        1 => 255,
        2 => 1,
        else => random.intRangeAtMost(u8, 1, 40),
    };
}

/// Random bytes, one byte throughout, or runs of a few bytes, so a codec
/// meets grids it cannot shrink and grids it shrinks to almost nothing.
fn fillRandomGrid(random: std.Random, grid: []u8) void {
    switch (random.uintLessThan(u8, 3)) {
        0 => random.bytes(grid),
        1 => @memset(grid, random.int(u8)),
        else => {
            var value = random.int(u8);
            for (grid) |*cell| {
                if (random.uintLessThan(u8, 6) == 0) value = random.int(u8);
                cell.* = value;
            }
        },
    }
}

/// A layer to pack: any size, any grids, and a header of random bytes
/// around the size. One in eight keeps a random magic and version too.
/// Free with `deinit`.
fn randomLayerInput(gpa: std.mem.Allocator, random: std.Random) !LayerInput {
    const width = randomAxis(random);
    const height = randomAxis(random);
    var input: LayerInput = try .initBlank(gpa, width, height);
    random.bytes(std.mem.asBytes(&input.header));
    input.header.width = width;
    input.header.height = height;
    if (random.uintLessThan(u8, 8) != 0) {
        input.header.magic = builder.tile_cache_magic;
        input.header.version = builder.tile_cache_version;
    }
    fillRandomGrid(random, input.heights);
    fillRandomGrid(random, input.areas);
    fillRandomGrid(random, input.connections);
    return input;
}

fn hasFormatHeader(input: *const LayerInput) bool {
    return input.header.magic == builder.tile_cache_magic and
        input.header.version == builder.tile_cache_version;
}

fn expectOffset(comptime field: []const u8, expected: usize) !void {
    errdefer std.debug.print("  field {s}\n", .{field});
    try testing.expectEqual(expected, @as(usize, @offsetOf(builder.LayerHeader, field)));
}

test "LayerHeader: the size and every field's offset" {
    try testing.expectEqual(
        reference.ref_sizeof_tile_cache_layer_header(),
        @as(usize, @sizeOf(builder.LayerHeader)),
    );
    try expectOffset("magic", reference.ref_offsetof_tile_cache_layer_header_magic());
    try expectOffset("version", reference.ref_offsetof_tile_cache_layer_header_version());
    try expectOffset("tile_x", reference.ref_offsetof_tile_cache_layer_header_tx());
    try expectOffset("tile_y", reference.ref_offsetof_tile_cache_layer_header_ty());
    try expectOffset("tile_layer", reference.ref_offsetof_tile_cache_layer_header_tlayer());
    try expectOffset("bounds_min", reference.ref_offsetof_tile_cache_layer_header_bmin());
    try expectOffset("bounds_max", reference.ref_offsetof_tile_cache_layer_header_bmax());
    try expectOffset("height_min", reference.ref_offsetof_tile_cache_layer_header_hmin());
    try expectOffset("height_max", reference.ref_offsetof_tile_cache_layer_header_hmax());
    try expectOffset("width", reference.ref_offsetof_tile_cache_layer_header_width());
    try expectOffset("height", reference.ref_offsetof_tile_cache_layer_header_height());
    try expectOffset("x_min", reference.ref_offsetof_tile_cache_layer_header_minx());
    try expectOffset("x_max", reference.ref_offsetof_tile_cache_layer_header_maxx());
    try expectOffset("y_min", reference.ref_offsetof_tile_cache_layer_header_miny());
    try expectOffset("y_max", reference.ref_offsetof_tile_cache_layer_header_maxy());
    // A field added to the port's header would need a line above.
    const field_names = @typeInfo(builder.LayerHeader).@"struct".field_names;
    try testing.expectEqual(@as(usize, 15), field_names.len);
    // The test file's own count of the header's bytes.
    try testing.expectEqual(
        @as(usize, header_size),
        reference.ref_sizeof_tile_cache_layer_header(),
    );
}

/// The status the port alone unpacks `blob` with. A layer, if one came,
/// stays in `scratch` until its next reset.
fn portUnpackStatus(scratch: *Scratch, codecs: *CodecPair, blob: []const u8) Status {
    var layer: ?*builder.Layer = null;
    return builder.decompressLayer(scratch.allocator(), codecs.port.compressor(), blob, &layer);
}

test "tile_cache_magic, tile_cache_version: the original takes these and nothing a bit away" {
    const gpa = testing.allocator;
    var codecs: CodecPair = undefined;
    try codecs.init(.store);
    defer codecs.deinit();
    var scratch: Scratch = try .init(gpa, Scratch.default_capacity);
    defer scratch.deinit();

    var input: LayerInput = try .initBlank(gpa, 3, 2);
    defer input.deinit(gpa);
    const blob = try packBoth(gpa, &codecs, &input) orelse return error.TestUnexpectedResult;
    defer gpa.free(blob);
    try testing.expect(try unpackBoth(&scratch, &codecs, blob) != null);

    // The magic's four bytes, then the version's.
    for (0..64) |bit| {
        errdefer std.debug.print("  bit {d}\n", .{bit});
        const mask = @as(u8, 1) << @intCast(bit % 8);
        blob[bit / 8] ^= mask;
        defer blob[bit / 8] ^= mask;
        scratch.reset();
        try testing.expect(try unpackBoth(&scratch, &codecs, blob) == null);
        // Refused before the codec is asked for anything.
        try testing.expectEqual(CodecRequests{}, codecs.port.requests);
        const expected = if (bit < 32) wrong_magic else wrong_version;
        try testing.expectEqual(
            statusBits(expected),
            statusBits(portUnpackStatus(&scratch, &codecs, blob)),
        );
    }
}

test "null_area, walkable_area, null_index: the original's values" {
    // The shim reads none of the three out of the original's headers, so
    // these are the numbers DetourTileCacheBuilder.h gives them. What the
    // builder does with the first and the last is compared in the region
    // and polygon mesh tests.
    try testing.expectEqual(@as(u8, 0), builder.null_area);
    try testing.expectEqual(@as(u8, 63), builder.walkable_area);
    try testing.expectEqual(@as(u16, 0xffff), builder.null_index);
}

test "buildLayer: every real layer, with both codecs" {
    const gpa = testing.allocator;
    var codecs: CodecPair = undefined;
    try codecs.init(.store);
    defer codecs.deinit();
    var scratch: Scratch = try .init(gpa, Scratch.default_capacity);
    defer scratch.deinit();

    const layers = try realLayers();
    if (layers.len < 8) {
        std.debug.print("  only {d} real layers\n", .{layers.len});
        return error.TestUnexpectedResult;
    }
    for (layers, 0..) |*layer, i| {
        for ([_]CodecKind{ .store, .run_length }) |kind| {
            errdefer std.debug.print("  layer {d}, codec {s}\n", .{ i, @tagName(kind) });
            codecs.setKind(kind);
            const blob = try packBoth(gpa, &codecs, layer) orelse
                return error.TestUnexpectedResult;
            defer gpa.free(blob);
            // The header's two bytes of padding.
            try support.expectBytes(&.{ 0, 0 }, blob[54..56]);
            // What went in comes out, on both sides.
            scratch.reset();
            const pair = try unpackBoth(&scratch, &codecs, blob) orelse
                return error.TestUnexpectedResult;
            try support.expectBytes(layer.heights, pair.port.heights);
            try support.expectBytes(layer.areas, pair.port.areas);
            try support.expectBytes(layer.connections, pair.port.connections);
        }
    }
}

test "buildLayer: no cells, one cell and as many as a layer holds" {
    const gpa = testing.allocator;
    var codecs: CodecPair = undefined;
    try codecs.init(.store);
    defer codecs.deinit();
    var scratch: Scratch = try .init(gpa, Scratch.default_capacity);
    defer scratch.deinit();

    const sizes = [_][2]u8{
        .{ 0, 0 },   .{ 0, 9 },   .{ 9, 0 },   .{ 1, 1 },
        .{ 255, 1 }, .{ 1, 255 }, .{ 16, 16 }, .{ 255, 255 },
    };
    for (sizes) |size| {
        for ([_]CodecKind{ .store, .run_length }) |kind| {
            errdefer std.debug.print("  {d} by {d}, codec {s}\n", .{
                size[0],
                size[1],
                @tagName(kind),
            });
            var input: LayerInput = try .initBlank(gpa, size[0], size[1]);
            defer input.deinit(gpa);
            // One area throughout, on a slope every cell connects across.
            @memset(input.areas, builder.walkable_area);
            for (input.heights, 0..) |*height, i| height.* = @truncate(i / 300);
            input.connect(1);

            codecs.setKind(kind);
            const blob = try packBoth(gpa, &codecs, &input) orelse
                return error.TestUnexpectedResult;
            defer gpa.free(blob);

            // The three grids go to the codec as one buffer, with the room
            // the codec itself asked for.
            const cell_count: i32 = @intCast(input.cellCount());
            const room = if (kind == .store) cell_count * 3 else cell_count * 6;
            try testing.expectEqual(CodecRequests{
                .size_max_input = cell_count * 3,
                .compress_input = cell_count * 3,
                .compress_room = room,
            }, codecs.port.requests);
            if (kind == .store) {
                try testing.expectEqual(header_size + input.cellCount() * 3, blob.len);
            }

            scratch.reset();
            const pair = try unpackBoth(&scratch, &codecs, blob) orelse
                return error.TestUnexpectedResult;
            try support.expectBytes(input.heights, pair.port.heights);
            try support.expectBytes(input.areas, pair.port.areas);
            try support.expectBytes(input.connections, pair.port.connections);
            // The grids get four times the cells of room, the regions too.
            try testing.expectEqual(cell_count * 4, codecs.port.requests.decompress_room);
        }
    }
}

test "buildLayer: the original stores its header's padding, the port stores zeros" {
    const gpa = testing.allocator;
    var codecs: CodecPair = undefined;
    try codecs.init(.run_length);
    defer codecs.deinit();

    var input: LayerInput = try .initBlank(gpa, 5, 4);
    defer input.deinit(gpa);
    @memset(input.areas, builder.walkable_area);

    // One header for both sides, with the two bytes after its fields set.
    var header_bytes: [header_size]u8 align(4) = @splat(0xa5);
    const field_size = @offsetOf(builder.LayerHeader, "y_max") + 1;
    try testing.expectEqual(@as(usize, 54), field_size);
    @memcpy(header_bytes[0..field_size], std.mem.asBytes(&input.header)[0..field_size]);

    var reference_data: ?[*]u8 = null;
    var reference_size: i32 = 0;
    const reference_status = reference.ref_tcb_build_layer(
        codecs.reference_compressor,
        @ptrCast(&header_bytes),
        input.heights.ptr,
        input.areas.ptr,
        input.connections.ptr,
        &reference_data,
        &reference_size,
    );
    try testing.expectEqual(statusBits(.success), reference_status);
    defer reference.ref_tcb_free_data(reference_data);

    var port_data: []u8 = &.{};
    const port_status = builder.buildLayer(
        gpa,
        codecs.port.compressor(),
        @ptrCast(&header_bytes),
        input.heights,
        input.areas,
        input.connections,
        &port_data,
    );
    try support.expectStatus(reference_status, port_status);
    defer gpa.free(port_data);

    const original = reference_data.?[0..@intCast(reference_size)];
    try testing.expectEqual(original.len, port_data.len);
    try support.expectBytes(original[0..field_size], port_data[0..field_size]);
    try support.expectBytes(&.{ 0xa5, 0xa5 }, original[field_size..header_size]);
    try support.expectBytes(&.{ 0, 0 }, port_data[field_size..header_size]);
    try support.expectBytes(original[header_size..], port_data[header_size..]);
}

/// Statuses a codec may fail with: the failure bit, whatever else is set.
const failing_statuses = [_]Status{
    out_of_memory,
    invalid_param,
    buffer_too_small,
    wrong_magic,
    wrong_version,
    .failure,
    .{ .failed = true, .succeeded = true },
    .{ .failed = true, .in_progress = true, .partial_result = true },
    @bitCast(@as(u32, 0xffff_ffff)),
};

/// Statuses without the failure bit, which the builder takes for success
/// whatever else they say.
const passing_statuses = [_]Status{
    .success,
    .{ .in_progress = true },
    .{ .succeeded = true, .partial_result = true },
    .{ .buffer_too_small = true },
    @bitCast(@as(u32, 0)),
    @bitCast(@as(u32, 0x7fff_ffff)),
};

test "buildLayer: every kind of status the codec's compress returns" {
    const gpa = testing.allocator;
    var codecs: CodecPair = undefined;
    try codecs.init(.run_length);
    defer codecs.deinit();

    var prng: std.Random.DefaultPrng = .init(0x51a7);
    var input: LayerInput = try .initBlank(gpa, 7, 5);
    defer input.deinit(gpa);
    fillRandomGrid(prng.random(), input.heights);
    fillRandomGrid(prng.random(), input.areas);
    fillRandomGrid(prng.random(), input.connections);

    const plain = try packBoth(gpa, &codecs, &input) orelse return error.TestUnexpectedResult;
    defer gpa.free(plain);

    // A failure goes to the caller as it is, and nothing is handed out.
    for (failing_statuses) |status| {
        errdefer std.debug.print("  status 0x{x:0>8}\n", .{statusBits(status)});
        codecs.setStatuses(status, .success);
        try testing.expect(try packBoth(gpa, &codecs, &input) == null);
    }

    // Anything else counts as done, and the caller gets plain success.
    for (passing_statuses) |status| {
        errdefer std.debug.print("  status 0x{x:0>8}\n", .{statusBits(status)});
        codecs.setStatuses(status, .success);
        const blob = try packBoth(gpa, &codecs, &input) orelse
            return error.TestUnexpectedResult;
        defer gpa.free(blob);
        try support.expectBytes(plain, blob);
    }
}

test "buildLayer: random layers, headers and codecs" {
    const gpa = testing.allocator;
    var codecs: CodecPair = undefined;
    try codecs.init(.store);
    defer codecs.deinit();
    var scratch: Scratch = try .init(gpa, Scratch.default_capacity);
    defer scratch.deinit();

    const seed = 0x7c1a;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();
    var unpacked_count: u32 = 0;
    for (0..1500) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        var input = try randomLayerInput(gpa, random);
        defer input.deinit(gpa);
        codecs.setKind(if (random.boolean()) .store else .run_length);

        const blob = try packBoth(gpa, &codecs, &input) orelse
            return error.TestUnexpectedResult;
        defer gpa.free(blob);
        try support.expectBytes(&.{ 0, 0 }, blob[54..56]);

        // Both sides unpack what the header lets them, and get the grids back.
        scratch.reset();
        const pair = try unpackBoth(&scratch, &codecs, blob) orelse {
            try testing.expect(!hasFormatHeader(&input));
            continue;
        };
        try testing.expect(hasFormatHeader(&input));
        try support.expectBytes(input.heights, pair.port.heights);
        try support.expectBytes(input.areas, pair.port.areas);
        try support.expectBytes(input.connections, pair.port.connections);
        unpacked_count += 1;
    }
    try testing.expect(unpacked_count >= 1000);
}

test "buildLayer: out of memory at each allocation leaves the output alone" {
    const gpa = testing.allocator;
    var codec: Codec = .{ .kind = .run_length };
    var input: LayerInput = try .initBlank(gpa, 9, 6);
    defer input.deinit(gpa);
    @memset(input.areas, builder.walkable_area);

    // The original takes its two blocks from the process heap, which no
    // test can make fail, so the port stands alone here.
    const Case = struct { config: std.testing.FailingAllocator.Config, fails: bool };
    const cases = [_]Case{
        .{ .config = .{}, .fails = false },
        .{ .config = .{ .fail_index = 0 }, .fails = true },
        .{ .config = .{ .fail_index = 1 }, .fails = true },
        // The packed block cannot shrink in place, and its replacement fails.
        .{ .config = .{ .fail_index = 2, .resize_fail_index = 0 }, .fails = true },
        // It cannot shrink in place, and moves.
        .{ .config = .{ .resize_fail_index = 0 }, .fails = false },
    };
    var whole: []u8 = &.{};
    defer gpa.free(whole);
    for (cases, 0..) |case, i| {
        errdefer std.debug.print("  case {d}\n", .{i});
        var failing: std.testing.FailingAllocator = .init(gpa, case.config);
        var marker: [1]u8 = .{0};
        var data: []u8 = &marker;
        const status = builder.buildLayer(
            failing.allocator(),
            codec.compressor(),
            &input.header,
            input.heights,
            input.areas,
            input.connections,
            &data,
        );
        if (case.fails) {
            try testing.expectEqual(statusBits(out_of_memory), statusBits(status));
            try testing.expectEqual(@as([*]u8, &marker), data.ptr);
            try testing.expectEqual(@as(usize, 1), data.len);
            try testing.expectEqual(failing.allocations, failing.deallocations);
            continue;
        }
        try testing.expectEqual(statusBits(.success), statusBits(status));
        defer failing.allocator().free(data);
        if (i == 0) whole = try gpa.dupe(u8, data);
        try support.expectBytes(whole, data);
    }
}

/// A packed layer put together by hand: `header`, two zero bytes of
/// padding, then `payload` as the codec's bytes. Caller frees with
/// `gpa.free`.
fn joinBlob(
    gpa: std.mem.Allocator,
    header: *const builder.LayerHeader,
    payload: []const u8,
) ![]u8 {
    const blob = try gpa.alloc(u8, header_size + payload.len);
    @memset(blob[0..header_size], 0);
    const field_size = @offsetOf(builder.LayerHeader, "y_max") + 1;
    @memcpy(blob[0..field_size], std.mem.asBytes(header)[0..field_size]);
    @memcpy(blob[header_size..], payload);
    return blob;
}

test "decompressLayer: payloads that fill the grids, fall short and run over" {
    const gpa = testing.allocator;
    var codecs: CodecPair = undefined;
    try codecs.init(.store);
    defer codecs.deinit();
    var scratch: Scratch = try .init(gpa, Scratch.default_capacity);
    defer scratch.deinit();

    // Two by two cells: four grids of four bytes, sixteen bytes of room.
    var input: LayerInput = try .initBlank(gpa, 2, 2);
    defer input.deinit(gpa);
    const ramp = [17]u8{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17 };

    const Case = struct { kind: CodecKind, payload: []const u8 };
    const cases = [_]Case{
        .{ .kind = .store, .payload = &.{} },
        .{ .kind = .store, .payload = ramp[0..5] },
        .{ .kind = .store, .payload = ramp[0..12] },
        .{ .kind = .store, .payload = ramp[0..15] },
        // The room takes the regions grid too.
        .{ .kind = .store, .payload = ramp[0..16] },
        // One byte more than the room.
        .{ .kind = .store, .payload = ramp[0..17] },
        .{ .kind = .run_length, .payload = &.{} },
        .{ .kind = .run_length, .payload = &.{ 4, 1, 4, 63, 4, 0 } },
        .{ .kind = .run_length, .payload = &.{ 12, 3 } },
        .{ .kind = .run_length, .payload = &.{ 16, 9 } },
        .{ .kind = .run_length, .payload = &.{ 17, 9 } },
        // Over the room after some of it is written.
        .{ .kind = .run_length, .payload = &.{ 4, 1, 13, 7 } },
        // A run cut short, and a run of nothing.
        .{ .kind = .run_length, .payload = &.{3} },
        .{ .kind = .run_length, .payload = &.{ 4, 1, 0, 5 } },
        .{ .kind = .run_length, .payload = &.{ 4, 1, 255 } },
    };
    var failure_count: u32 = 0;
    for (cases, 0..) |case, i| {
        errdefer std.debug.print("  case {d}\n", .{i});
        const blob = try joinBlob(gpa, &input.header, case.payload);
        defer gpa.free(blob);

        // What the codec makes of the payload by itself.
        var alone: Codec = .{ .kind = case.kind };
        var expected: [16]u8 = @splat(0);
        var size: i32 = 0;
        const expected_status = alone.decompress(case.payload, &expected, &size);

        codecs.setKind(case.kind);
        scratch.reset();
        const pair = try unpackBoth(&scratch, &codecs, blob) orelse {
            failure_count += 1;
            try testing.expect(expected_status.failed);
            // The port gave its block back.
            try testing.expectEqual(@as(usize, 0), scratch.fixed.end_index);
            try testing.expectEqual(
                statusBits(expected_status),
                statusBits(portUnpackStatus(&scratch, &codecs, blob)),
            );
            continue;
        };
        try testing.expect(!expected_status.failed);
        try testing.expectEqual(CodecRequests{
            .decompress_input = @intCast(case.payload.len),
            .decompress_room = 16,
        }, codecs.port.requests);
        try support.expectBytes(expected[0..4], pair.port.heights);
        try support.expectBytes(expected[4..8], pair.port.areas);
        try support.expectBytes(expected[8..12], pair.port.connections);
        try support.expectBytes(expected[12..16], pair.port.regions);
        try testing.expectEqual(@as(u8, 0), pair.port.region_count);
    }
    try testing.expectEqual(@as(u32, 6), failure_count);
}

test "decompressLayer: every kind of status the codec's decompress returns" {
    const gpa = testing.allocator;
    var codecs: CodecPair = undefined;
    try codecs.init(.run_length);
    defer codecs.deinit();
    var scratch: Scratch = try .init(gpa, Scratch.default_capacity);
    defer scratch.deinit();

    const input = try largestRealLayer();
    const blob = try packBoth(gpa, &codecs, input) orelse return error.TestUnexpectedResult;
    defer gpa.free(blob);

    // A failure goes to the caller as it is, with no layer and no memory kept.
    for (failing_statuses) |status| {
        errdefer std.debug.print("  status 0x{x:0>8}\n", .{statusBits(status)});
        codecs.setStatuses(.success, status);
        scratch.reset();
        try testing.expect(try unpackBoth(&scratch, &codecs, blob) == null);
        try testing.expectEqual(@as(usize, 0), scratch.fixed.end_index);
        try testing.expectEqual(
            statusBits(status),
            statusBits(portUnpackStatus(&scratch, &codecs, blob)),
        );
    }

    // Anything else counts as done.
    for (passing_statuses) |status| {
        errdefer std.debug.print("  status 0x{x:0>8}\n", .{statusBits(status)});
        codecs.setStatuses(.success, status);
        scratch.reset();
        const pair = try unpackBoth(&scratch, &codecs, blob) orelse
            return error.TestUnexpectedResult;
        try support.expectBytes(input.heights, pair.port.heights);
        try support.expectBytes(input.areas, pair.port.areas);
        try support.expectBytes(input.connections, pair.port.connections);
    }
}

/// Starts both sides of `scratch` over with every byte of their memory
/// set, so nothing a build step reads is zero by luck.
fn resetDirty(scratch: *Scratch) void {
    scratch.reset();
    @memset(scratch.buffer, 0xa5);
    const capacity = reference.ref_tc_alloc_linear_capacity(scratch.original);
    const whole = reference.ref_tc_alloc_allocate(scratch.original, capacity).?;
    @memset(whole[0..capacity], 0xa5);
    reference.ref_tc_alloc_reset(scratch.original);
}

test "decompressLayer: the regions start zeroed in memory that was not" {
    const gpa = testing.allocator;
    var codecs: CodecPair = undefined;
    try codecs.init(.store);
    defer codecs.deinit();
    var scratch: Scratch = try .init(gpa, Scratch.default_capacity);
    defer scratch.deinit();

    const layers = try realLayers();
    for (layers, 0..) |*input, i| {
        errdefer std.debug.print("  layer {d}\n", .{i});
        codecs.setKind(if (i % 2 == 0) .store else .run_length);
        const blob = try packBoth(gpa, &codecs, input) orelse
            return error.TestUnexpectedResult;
        defer gpa.free(blob);

        resetDirty(&scratch);
        const pair = try unpackBoth(&scratch, &codecs, blob) orelse
            return error.TestUnexpectedResult;
        try testing.expectEqual(@as(u8, 0), pair.port.region_count);
        try testing.expect(std.mem.allEqual(u8, pair.port.regions, 0));
        try support.expectBytes(input.heights, pair.port.heights);
        try support.expectBytes(input.areas, pair.port.areas);
        try support.expectBytes(input.connections, pair.port.connections);
        // The header is a copy that the packed bytes can go away under.
        try support.expectBytes(blob[0..header_size], std.mem.asBytes(pair.port.header));
    }
}

test "decompressLayer: fewer bytes than a header" {
    const gpa = testing.allocator;
    var codecs: CodecPair = undefined;
    try codecs.init(.store);
    defer codecs.deinit();

    var input: LayerInput = try .initBlank(gpa, 3, 3);
    defer input.deinit(gpa);
    const blob = try packBoth(gpa, &codecs, &input) orelse return error.TestUnexpectedResult;
    defer gpa.free(blob);

    // The original reads a header whatever the length it is given, so the
    // port stands alone: it refuses before it reads, allocates or decodes.
    for (0..header_size) |len| {
        errdefer std.debug.print("  {d} bytes\n", .{len});
        var counting: std.testing.FailingAllocator = .init(gpa, .{});
        var dummy: builder.Layer = undefined;
        var layer: ?*builder.Layer = &dummy;
        codecs.clearRequests();
        const status = builder.decompressLayer(
            counting.allocator(),
            codecs.port.compressor(),
            blob[0..len],
            &layer,
        );
        try testing.expectEqual(statusBits(invalid_param), statusBits(status));
        try testing.expectEqual(@as(?*builder.Layer, null), layer);
        try testing.expectEqual(CodecRequests{}, codecs.port.requests);
        try testing.expectEqual(@as(usize, 0), counting.allocations);
    }
}

test "decompressLayer: truncated, corrupted and random packed layers" {
    const gpa = testing.allocator;
    var codecs: CodecPair = undefined;
    try codecs.init(.store);
    defer codecs.deinit();
    var scratch: Scratch = try .init(gpa, Scratch.default_capacity);
    defer scratch.deinit();

    const seed = 0xdec0;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();
    var blob: std.ArrayList(u8) = .empty;
    defer blob.deinit(gpa);
    var success_count: u32 = 0;
    var failure_count: u32 = 0;
    for (0..2500) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        codecs.setKind(if (random.boolean()) .store else .run_length);

        // A good packed layer to start from.
        const width = random.uintLessThan(u8, 12);
        const height = random.uintLessThan(u8, 12);
        var input: LayerInput = try .initBlank(gpa, width, height);
        defer input.deinit(gpa);
        fillRandomGrid(random, input.heights);
        fillRandomGrid(random, input.areas);
        fillRandomGrid(random, input.connections);
        const good = try packBoth(gpa, &codecs, &input) orelse
            return error.TestUnexpectedResult;
        defer gpa.free(good);
        blob.clearRetainingCapacity();
        try blob.appendSlice(gpa, good);

        // Never shorter than a header: the original reads one regardless.
        switch (random.uintLessThan(u8, 6)) {
            // Cut short, down to the bare header.
            0 => blob.shrinkRetainingCapacity(
                random.intRangeAtMost(usize, header_size, blob.items.len),
            ),
            // A few bits flipped anywhere, the header included.
            1 => for (0..random.intRangeAtMost(usize, 1, 4)) |_| {
                const bit = random.uintLessThan(usize, blob.items.len * 8);
                blob.items[bit / 8] ^= @as(u8, 1) << @intCast(bit % 8);
            },
            // Another payload of the same length.
            2 => random.bytes(blob.items[header_size..]),
            // More bytes after the payload.
            3 => random.bytes(try blob.addManyAsSlice(gpa, random.uintLessThan(usize, 64))),
            // Another size in the header, which the payload was not made for.
            4 => {
                blob.items[@offsetOf(builder.LayerHeader, "width")] = randomAxis(random);
                blob.items[@offsetOf(builder.LayerHeader, "height")] = randomAxis(random);
            },
            // Nothing but noise, half the time under a good magic and version.
            else => {
                try blob.resize(gpa, random.intRangeAtMost(usize, header_size, 400));
                random.bytes(blob.items);
                if (random.boolean()) @memcpy(blob.items[0..8], good[0..8]);
            },
        }

        scratch.reset();
        if (try unpackBoth(&scratch, &codecs, blob.items) == null) {
            failure_count += 1;
        } else {
            success_count += 1;
        }
    }
    // Both outcomes are common enough to have been compared many times.
    try testing.expect(success_count >= 300);
    try testing.expect(failure_count >= 300);
}

/// The build steps that take working memory, in the order they run.
const Step = enum { decompress, regions, contours, mesh };

/// What one run of a step was given by the two sides' allocators.
const StepUse = struct {
    status: Status,
    /// Allocations the port's step was given.
    port_allocation_count: usize,
    /// Bytes the original's step was given.
    original_size: usize,
};

/// A `fail_index` no step reaches.
const never_fails = std.math.maxInt(usize);

const starved_max_error: f32 = 1.3;

/// From here on the port's allocation number `fail_index` fails and the
/// original allocates from `tight`. Returns the port's allocation index now.
fn starve(
    scratch: *Scratch,
    failing: *std.testing.FailingAllocator,
    tight: *reference.TileCacheAlloc,
    fail_index: usize,
) usize {
    scratch.original = tight;
    if (fail_index != never_fails) failing.fail_index = failing.alloc_index + fail_index;
    return failing.alloc_index;
}

/// `runStepStarved` on a fresh failing allocator, which must have got back
/// everything it gave: the free functions return what a failed step leaves.
fn starvedStep(
    scratch: *Scratch,
    codecs: *CodecPair,
    blob: []const u8,
    step: Step,
    fail_index: usize,
    capacity: usize,
) !StepUse {
    var failing: std.testing.FailingAllocator = .init(testing.allocator, .{});
    const use = try runStepStarved(scratch, &failing, codecs, blob, step, fail_index, capacity);
    try testing.expectEqual(failing.allocations, failing.deallocations);
    try testing.expectEqual(fail_index != never_fails, failing.has_induced_failure);
    return use;
}

/// Runs the steps before `step` on both sides with all the memory they ask
/// for, then `step` with the port's allocation number `fail_index` failing
/// and the original working in `capacity` bytes, and compares everything
/// the step leaves behind. The port's memory comes from `failing`, and all
/// of it is given back before this returns.
fn runStepStarved(
    scratch: *Scratch,
    failing: *std.testing.FailingAllocator,
    codecs: *CodecPair,
    blob: []const u8,
    step: Step,
    fail_index: usize,
    capacity: usize,
) !StepUse {
    const port = failing.allocator();
    const roomy = scratch.original;
    const tight = reference.ref_tc_alloc_create_linear(capacity) orelse
        return error.OutOfMemory;
    defer reference.ref_tc_alloc_destroy(tight);
    scratch.reset();
    scratch.port_override = port;
    defer scratch.port_override = null;
    defer scratch.original = roomy;

    var layers: ?LayerPair = null;
    var sets: ?ContourSetPair = null;
    var meshes: ?PolyMeshPair = null;
    // Only the port's: the original's free functions read arrays a failed
    // step never made, and its allocators forget everything when reset.
    defer {
        if (meshes) |pair| builder.freePolyMesh(port, pair.port);
        if (sets) |pair| builder.freeContourSet(port, pair.port);
        if (layers) |pair| builder.freeLayer(port, pair.port);
    }

    var status: Status = .success;
    var armed_index: usize = 0;
    run: {
        if (step == .decompress) armed_index = starve(scratch, failing, tight, fail_index);
        layers = try unpackBothStatus(scratch, codecs, blob, &status);
        if (step == .decompress) {
            // Out of memory, neither side gets as far as its codec.
            if (status.failed) try testing.expectEqual(CodecRequests{}, codecs.port.requests);
            break :run;
        }
        const layer = layers orelse return error.TestUnexpectedResult;

        if (step == .regions) {
            armed_index = starve(scratch, failing, tight, fail_index);
            // The port first: where it refuses a region's seventeenth
            // neighbour, the original writes past its array.
            status = builder.buildRegions(port, layer.port, real_climb);
            if (status.buffer_too_small) return error.TestUnexpectedResult;
            const reference_status = reference.ref_tcb_build_regions(
                tight,
                layer.original,
                real_climb,
            );
            try support.expectStatus(reference_status, status);
            try expectSameLayer(layer);
            break :run;
        }
        if (try buildRegionsBoth(scratch, layer, real_climb) != .built) {
            return error.TestUnexpectedResult;
        }
        if (!contoursAreSafe(layer)) return error.TestUnexpectedResult;

        if (step == .contours) {
            const pair: ContourSetPair = .{
                .original = reference.ref_tcb_alloc_contour_set(roomy) orelse
                    return error.OutOfMemory,
                .port = builder.allocContourSet(port) orelse return error.OutOfMemory,
            };
            sets = pair;
            armed_index = starve(scratch, failing, tight, fail_index);
            const reference_status = reference.ref_tcb_build_contours(
                tight,
                layer.original,
                real_climb,
                starved_max_error,
                pair.original,
            );
            status = builder.buildContours(
                port,
                layer.port,
                real_climb,
                starved_max_error,
                pair.port,
            );
            try support.expectStatus(reference_status, status);
            try testing.expectEqual(
                reference.ref_tcb_contour_set_count(pair.original),
                pair.port.contour_count,
            );
            // Without its contour array the original has nothing to read.
            const array_failed = fail_index == 0 and layer.port.region_count > 0;
            if (!array_failed) try expectSameContourSet(pair);
            try expectSameLayer(layer);
            break :run;
        }
        sets = try buildContoursBoth(scratch, layer, real_climb, starved_max_error) orelse
            return error.TestUnexpectedResult;

        const pair: PolyMeshPair = .{
            .original = reference.ref_tcb_alloc_poly_mesh(roomy) orelse
                return error.OutOfMemory,
            .port = builder.allocPolyMesh(port) orelse return error.OutOfMemory,
        };
        meshes = pair;
        armed_index = starve(scratch, failing, tight, fail_index);
        const reference_status = reference.ref_tcb_build_poly_mesh(
            tight,
            sets.?.original,
            pair.original,
        );
        status = builder.buildPolyMesh(port, sets.?.port, pair.port);
        try support.expectStatus(reference_status, status);
        // The mesh's four arrays are among the first five allocations, and
        // once all are made every element is set whatever fails later.
        if (fail_index >= 5) {
            try expectSamePolyMesh(sets.?, pair);
        } else {
            try expectSameMeshCounts(pair);
        }
        try expectSameContourSet(sets.?);
    }
    return .{
        .status = status,
        .port_allocation_count = failing.alloc_index - armed_index,
        .original_size = reference.ref_tc_alloc_linear_top(tight),
    };
}

test "decompressLayer, buildRegions, buildContours, buildPolyMesh: each allocation failing" {
    const gpa = testing.allocator;
    var codecs: CodecPair = undefined;
    try codecs.init(.run_length);
    defer codecs.deinit();
    var scratch: Scratch = try .init(gpa, Scratch.default_capacity);
    defer scratch.deinit();

    // A small layer of two areas, then real ones spread over the levels.
    var small: LayerInput = try .initBlank(gpa, 8, 8);
    defer small.deinit(gpa);
    for (small.areas, 0..) |*area, i| area.* = if (i % 8 < 5) builder.walkable_area else 7;
    small.connect(real_climb);

    const real = try realLayers();
    var inputs: std.ArrayList(*const LayerInput) = .empty;
    defer inputs.deinit(gpa);
    try inputs.append(gpa, &small);
    try inputs.append(gpa, try largestRealLayer());
    var real_index: usize = 0;
    while (real_index < real.len) : (real_index += @max(1, real.len / 5)) {
        try inputs.append(gpa, &real[real_index]);
    }

    var layer_count: usize = 0;
    var allocation_count_max: [4]usize = @splat(0);
    for (inputs.items, 0..) |input, i| {
        const blob = try packBoth(gpa, &codecs, input) orelse return error.TestUnexpectedResult;
        defer gpa.free(blob);
        // Only layers both sides build to the end, so that every step has
        // all of its allocations to fail.
        scratch.reset();
        const probe = try unpackBoth(&scratch, &codecs, blob) orelse
            return error.TestUnexpectedResult;
        if (try buildAllBoth(&scratch, probe, real_climb, starved_max_error) != .mesh) continue;
        layer_count += 1;

        for ([_]Step{ .decompress, .regions, .contours, .mesh }) |step| {
            errdefer std.debug.print("  layer {d}, step {s}\n", .{ i, @tagName(step) });
            const whole = try starvedStep(
                &scratch,
                &codecs,
                blob,
                step,
                never_fails,
                Scratch.default_capacity,
            );
            try testing.expectEqual(statusBits(.success), statusBits(whole.status));
            const count_max = &allocation_count_max[@backingInt(step)];
            count_max.* = @max(count_max.*, whole.port_allocation_count);

            // The original's allocator does not count its allocations, so
            // its last one is failed by a byte too few, then the one
            // before by a byte less than it got up to there, and so on.
            var capacity = whole.original_size;
            var fail_index = whole.port_allocation_count;
            while (fail_index > 0) {
                fail_index -= 1;
                errdefer std.debug.print("  allocation {d} failing\n", .{fail_index});
                if (capacity < 2) return error.TestUnexpectedResult;
                const starved = try starvedStep(
                    &scratch,
                    &codecs,
                    blob,
                    step,
                    fail_index,
                    capacity - 1,
                );
                try testing.expectEqual(statusBits(out_of_memory), statusBits(starved.status));
                try testing.expectEqual(fail_index, starved.port_allocation_count);
                try testing.expect(starved.original_size < capacity);
                capacity = starved.original_size;
            }
            // The port's first allocation failed together with the
            // original's first: both allocate the same number of times.
            try testing.expectEqual(@as(usize, 0), capacity);
        }
    }
    try testing.expect(layer_count >= 3);
    // One block to unpack, two arrays for regions, three and one per
    // contour for contours, eleven for a mesh that has polygons.
    try testing.expectEqual(@as(usize, 1), allocation_count_max[0]);
    try testing.expectEqual(@as(usize, 2), allocation_count_max[1]);
    try testing.expect(allocation_count_max[2] >= 4);
    try testing.expectEqual(@as(usize, 11), allocation_count_max[3]);
}

test "buildRegions: every real layer, from no climb to any climb" {
    const gpa = testing.allocator;
    var codecs: CodecPair = undefined;
    try codecs.init(.store);
    defer codecs.deinit();
    var scratch: Scratch = try .init(gpa, Scratch.default_capacity);
    defer scratch.deinit();

    var outcome_counts: [3]u32 = @splat(0);
    const layers = try realLayers();
    for (layers, 0..) |*input, i| {
        const blob = try packBoth(gpa, &codecs, input) orelse return error.TestUnexpectedResult;
        defer gpa.free(blob);
        // Below zero no cell connects to another, at 255 any two of one
        // area do.
        for ([_]i32{ -1, 0, 1, real_climb, 255 }) |climb| {
            errdefer std.debug.print("  layer {d}, climb {d}\n", .{ i, climb });
            resetDirty(&scratch);
            const pair = try unpackBoth(&scratch, &codecs, blob) orelse
                return error.TestUnexpectedResult;
            const outcome = try buildRegionsBoth(&scratch, pair, climb);
            outcome_counts[@backingInt(outcome)] += 1;
            if (outcome != .built) continue;
            // Every cell to walk on has a region, and no other cell has.
            for (pair.port.areas, pair.port.regions) |area, region| {
                if (area == builder.null_area) {
                    try testing.expectEqual(@as(u8, 0xff), region);
                } else {
                    try testing.expect(region < pair.port.region_count);
                }
            }
        }
    }
    // Most builds were compared with the original's, and many succeeded.
    const refused_count = outcome_counts[@backingInt(RegionsOutcome.refused)];
    try testing.expect(refused_count * 2 <= layers.len * 5);
    try testing.expect(outcome_counts[@backingInt(RegionsOutcome.built)] >= layers.len);
}

test "buildRegions: as many regions as there are ids, and one more" {
    const gpa = testing.allocator;
    var codecs: CodecPair = undefined;
    try codecs.init(.store);
    defer codecs.deinit();
    var scratch: Scratch = try .init(gpa, Scratch.default_capacity);
    defer scratch.deinit();

    // 255 cells none of which connects to another: a region each.
    var full: LayerInput = try .initBlank(gpa, 15, 17);
    defer full.deinit(gpa);
    @memset(full.areas, builder.walkable_area);
    {
        const blob = try packBoth(gpa, &codecs, &full) orelse return error.TestUnexpectedResult;
        defer gpa.free(blob);
        resetDirty(&scratch);
        const pair = try unpackBoth(&scratch, &codecs, blob) orelse
            return error.TestUnexpectedResult;
        try testing.expectEqual(RegionsOutcome.built, try buildRegionsBoth(&scratch, pair, -1));
        try testing.expectEqual(@as(u8, 255), pair.port.region_count);
        for (pair.port.regions, 0..) |region, cell| {
            try testing.expectEqual(@as(u8, @intCast(cell)), region);
        }
        // And the contours and polygons of that many regions.
        const sets = try buildContoursBoth(&scratch, pair, -1, 0) orelse
            return error.TestUnexpectedResult;
        try testing.expectEqual(@as(i32, 255), sets.port.contour_count);
        const meshes = try buildPolyMeshBoth(&scratch, sets) orelse
            return error.TestUnexpectedResult;
        try testing.expect(meshes.port.polygon_count >= 255);
    }

    // One cell more, and the last cell of the last row finds no id left.
    var over: LayerInput = try .initBlank(gpa, 16, 16);
    defer over.deinit(gpa);
    @memset(over.areas, builder.walkable_area);
    {
        const blob = try packBoth(gpa, &codecs, &over) orelse return error.TestUnexpectedResult;
        defer gpa.free(blob);
        resetDirty(&scratch);
        const pair = try unpackBoth(&scratch, &codecs, blob) orelse
            return error.TestUnexpectedResult;
        // A build that succeeds first, so the failure has a count to leave.
        try testing.expectEqual(RegionsOutcome.built, try buildRegionsBoth(&scratch, pair, 255));
        try testing.expectEqual(@as(u8, 1), pair.port.region_count);
        try testing.expectEqual(RegionsOutcome.failed, try buildRegionsBoth(&scratch, pair, -1));
        try testing.expectEqual(@as(u8, 1), pair.port.region_count);
        // The rows before the last got their ids, the last row did not.
        try testing.expectEqual(@as(u8, 239), pair.port.regions[239]);
        try testing.expectEqual(@as(u8, 15), pair.port.regions[255]);
    }
}

/// What the cells of a hand-made layer hold.
const Pattern = enum { walkable, empty, checkerboard, ring, ramp };

/// Fills the areas and heights of `input` with `pattern`, then connects
/// the cells that are within `climb` of each other.
fn fillPattern(input: *LayerInput, pattern: Pattern, climb: i32) void {
    const w: usize = input.header.width;
    const h: usize = input.header.height;
    for (input.areas, input.heights, 0..) |*area, *height, cell| {
        const x = cell % w;
        const y = cell / w;
        const inside = x > 0 and y > 0 and x + 1 < w and y + 1 < h;
        // Three up per cell along x, starting over past what a byte holds.
        height.* = if (pattern == .ramp) @truncate(x * 3) else 0;
        area.* = switch (pattern) {
            .walkable, .ramp => builder.walkable_area,
            .empty => builder.null_area,
            // Two areas, and no cell beside one of its own.
            .checkerboard => if ((x + y) % 2 == 0) builder.walkable_area else 5,
            // A border one cell wide around nothing.
            .ring => if (inside) builder.null_area else builder.walkable_area,
        };
    }
    input.connect(climb);
}

test "buildRegions, buildContours, buildPolyMesh: hand-made layers of every size" {
    const gpa = testing.allocator;
    var codecs: CodecPair = undefined;
    try codecs.init(.run_length);
    defer codecs.deinit();
    var scratch: Scratch = try .init(gpa, Scratch.default_capacity);
    defer scratch.deinit();

    // No cells, one, a row, a column, squares too small for the walk
    // around a region to finish, and the most a layer holds.
    const sizes = [_][2]u8{
        .{ 0, 0 }, .{ 0, 5 },   .{ 5, 0 },   .{ 1, 1 },     .{ 1, 2 },
        .{ 2, 1 }, .{ 2, 2 },   .{ 1, 255 }, .{ 255, 1 },   .{ 3, 3 },
        .{ 5, 5 }, .{ 16, 16 }, .{ 40, 40 }, .{ 255, 255 },
    };
    const patterns = [_]Pattern{ .walkable, .empty, .checkerboard, .ring, .ramp };
    var reached: ReachCounts = .{};
    for (sizes) |size| {
        var input: LayerInput = try .initBlank(gpa, size[0], size[1]);
        defer input.deinit(gpa);
        for (patterns) |pattern| {
            // The ramp's steps are within reach from three up.
            for ([_]i32{ 0, 2, 3, 255 }) |climb| {
                errdefer std.debug.print(
                    "  {d} by {d} cells, pattern {s}, climb {d}\n",
                    .{ size[0], size[1], @tagName(pattern), climb },
                );
                fillPattern(&input, pattern, climb);
                const blob = try packBoth(gpa, &codecs, &input) orelse
                    return error.TestUnexpectedResult;
                defer gpa.free(blob);
                for ([_]f32{ 0, 1.3 }) |max_error| {
                    scratch.reset();
                    const pair = try unpackBoth(&scratch, &codecs, blob) orelse
                        return error.TestUnexpectedResult;
                    reached.add(try buildAllBoth(&scratch, pair, climb, max_error));
                }
            }
        }
    }
    // None of these has a region with many neighbours. The checkerboards
    // from 16 by 16 up run out of region ids, and most of the rest get
    // their polygons.
    try testing.expectEqual(@as(u32, 0), reached.of(.refused));
    try testing.expect(reached.of(.regions_failed) >= 8);
    try testing.expect(reached.of(.mesh) >= 400);
}

// ---------------------------------------------------------------------------
// Random layers through every build step
// ---------------------------------------------------------------------------

/// Portal bits no two cells share exactly one of. The mesh removes the
/// vertices at corners whose cells do, and these layers are not for that.
const paired_portals = [_]u8{ 0b0000, 0b0011, 0b1100, 0b1111 };

/// A small layer for the build steps: a few areas in patches, heights that
/// step by little or by much, and connections that follow the heights or
/// are any that stay in the grid. Free with `deinit`.
fn randomBuildLayer(gpa: std.mem.Allocator, random: std.Random, climb: i32) !LayerInput {
    const width = random.intRangeAtMost(u8, 1, 24);
    const height = random.intRangeAtMost(u8, 1, 24);
    var input: LayerInput = try .initBlank(gpa, width, height);
    const w: usize = width;
    const h: usize = height;

    // Areas and height levels come in patches of a few cells a side.
    const walkable = builder.walkable_area;
    const area_palette = [_]u8{ builder.null_area, walkable, walkable, 5, 9 };
    var patch_areas: [24 * 24]u8 = undefined;
    var patch_levels: [24 * 24]u8 = undefined;
    for (&patch_areas, &patch_levels) |*area, *level| {
        area.* = area_palette[random.uintLessThan(usize, area_palette.len)];
        level.* = random.uintLessThan(u8, 4);
    }
    const patch_size = random.intRangeAtMost(usize, 1, 8);
    const level_step = random.intRangeAtMost(u8, 0, 6);
    const noisy = random.uintLessThan(u8, 4) == 0;
    for (input.areas, input.heights, 0..) |*area, *elevation, cell| {
        const patch = (cell % w) / patch_size + (cell / w) / patch_size * 24;
        area.* = patch_areas[patch];
        elevation.* = 100 + patch_levels[patch] * level_step;
        if (noisy) elevation.* += random.uintLessThan(u8, 3);
    }

    if (random.uintLessThan(u8, 5) != 0) {
        input.connect(climb);
    } else {
        // Any bits, also ones the heights and areas speak against, as long
        // as none leads out of the grid: neither side's walk checks that.
        for (input.connections, 0..) |*connection, cell| {
            var bits: u8 = random.int(u8) & 0x0f;
            if (cell % w == 0) bits &= ~@as(u8, 0b0001);
            if (cell / w + 1 == h) bits &= ~@as(u8, 0b0010);
            if (cell % w + 1 == w) bits &= ~@as(u8, 0b0100);
            if (cell / w == 0) bits &= ~@as(u8, 0b1000);
            connection.* = bits;
        }
    }
    if (random.uintLessThan(u8, 3) == 0) {
        for (input.connections) |*connection| {
            if (random.uintLessThan(u8, 4) != 0) continue;
            const portal = paired_portals[random.uintLessThan(usize, paired_portals.len)];
            connection.* |= portal << 4;
        }
    }
    return input;
}

test "buildRegions, buildContours, buildPolyMesh: random layers" {
    const gpa = testing.allocator;
    var codecs: CodecPair = undefined;
    try codecs.init(.store);
    defer codecs.deinit();
    var scratch: Scratch = try .init(gpa, Scratch.default_capacity);
    defer scratch.deinit();

    const seed: u64 = 0x6c61_7965_7273_0006;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();
    const climbs = [_]i32{ -1, 0, 1, 2, 4, 255 };
    const max_errors = [_]f32{ 0, 0.5, 1.3, 3, 100 };
    const case_count = 2000;
    var reached: ReachCounts = .{};
    for (0..case_count) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const climb = climbs[random.uintLessThan(usize, climbs.len)];
        var input = try randomBuildLayer(gpa, random, climb);
        defer input.deinit(gpa);
        const blob = try packBoth(gpa, &codecs, &input) orelse
            return error.TestUnexpectedResult;
        defer gpa.free(blob);
        scratch.reset();
        const pair = try unpackBoth(&scratch, &codecs, blob) orelse
            return error.TestUnexpectedResult;
        // The connections follow one climb. Now and then the regions and
        // the corners are built for another, so the two disagree.
        const build_climb = if (random.uintLessThan(u8, 6) == 0)
            climbs[random.uintLessThan(usize, climbs.len)]
        else
            climb;
        const max_error = max_errors[random.uintLessThan(usize, max_errors.len)];
        reached.add(try buildAllBoth(&scratch, pair, build_climb, max_error));
    }
    // Some layers run out of region ids and some outlines are too long;
    // the rest must come out of the last step compared.
    try testing.expect(reached.of(.mesh) >= case_count / 4);
}

// ---------------------------------------------------------------------------
// Contours and polygons of real layers, and outlines too long to hold
// ---------------------------------------------------------------------------

/// What holds for a mesh whatever the original made of the same contours:
/// a polygon's corners are vertices of the mesh, and across each side lies
/// nothing, another polygon of the mesh, or a portal on one of four sides.
fn expectSoundMesh(mesh: *const builder.PolyMesh) !void {
    const polygon_count: usize = @intCast(mesh.polygon_count);
    for (0..polygon_count) |i| {
        errdefer std.debug.print("  polygon {d} of {d}\n", .{ i, polygon_count });
        const polygon = mesh.polygons[i * 12 ..][0..12];
        try testing.expect(polygon[2] != builder.null_index);
        for (polygon[0..6]) |corner| {
            if (corner == builder.null_index) continue;
            try testing.expect(corner < mesh.vertex_count);
        }
        for (polygon[6..12]) |neighbor| {
            if (neighbor == builder.null_index) continue;
            if (neighbor & 0x8000 != 0) {
                try testing.expect(neighbor & 0x7fff < 4);
            } else {
                try testing.expect(neighbor < mesh.polygon_count);
            }
        }
    }
}

test "buildContours, buildPolyMesh: every real layer, from exact outlines to rough ones" {
    const gpa = testing.allocator;
    var codecs: CodecPair = undefined;
    try codecs.init(.store);
    defer codecs.deinit();
    var scratch: Scratch = try .init(gpa, Scratch.default_capacity);
    defer scratch.deinit();

    var mesh_count: usize = 0;
    const layers = try realLayers();
    for (layers, 0..) |*input, i| {
        const blob = try packBoth(gpa, &codecs, input) orelse return error.TestUnexpectedResult;
        defer gpa.free(blob);
        // With no climb the layer falls into many more regions.
        for ([_]i32{ 0, real_climb }) |climb| {
            // From every corner of the outline kept to as few as close it.
            for ([_]f32{ 0, 0.5, 1.3, 3, 100 }) |max_error| {
                errdefer std.debug.print(
                    "  layer {d}, climb {d}, error {d}\n",
                    .{ i, climb, max_error },
                );
                scratch.reset();
                const pair = try unpackBoth(&scratch, &codecs, blob) orelse
                    return error.TestUnexpectedResult;
                if (try buildRegionsBoth(&scratch, pair, climb) != .built) continue;
                const sets = try buildContoursBoth(&scratch, pair, climb, max_error) orelse
                    continue;
                const meshes = try buildPolyMeshBoth(&scratch, sets) orelse continue;
                try expectSoundMesh(meshes.port);
                mesh_count += 1;
            }
        }
    }
    // Ten builds a layer: at least one in ten got to the end.
    try testing.expect(mesh_count >= layers.len);
}

/// Fills a layer with an odd number of rows with one region that winds
/// through all of it: the even rows whole, each joined to the next by one
/// cell at alternating ends of the odd row between them. The other cells
/// of the odd rows alternate between two more areas, so each is a region
/// of its own and the winding region's outline has a vertex at every one.
fn fillSerpentine(input: *LayerInput) void {
    const w: usize = input.header.width;
    for (input.areas, 0..) |*area, cell| {
        const x = cell % w;
        const y = cell / w;
        const joint_x = if (y % 4 == 1) w - 1 else 0;
        area.* = if (y % 2 == 0 or x == joint_x)
            builder.walkable_area
        else if (x % 2 == 0)
            5
        else
            9;
    }
    @memset(input.heights, 0);
    input.connect(0);
}

test "buildContours: outlines with more vertices than the walk has room for" {
    const gpa = testing.allocator;
    var codecs: CodecPair = undefined;
    try codecs.init(.store);
    defer codecs.deinit();
    var scratch: Scratch = try .init(gpa, Scratch.default_capacity);
    defer scratch.deinit();

    // The width, and how many whole rows the region winds through. The
    // walk has room for four vertices per cell of width and of height, and
    // gives up after as many steps as the layer has cells, about two per
    // vertex here: narrow or short layers are cut off before they fill the
    // room, wide ones with many rows run out of it.
    const sizes = [_][2]u8{
        .{ 12, 2 }, .{ 12, 3 }, .{ 12, 8 }, .{ 30, 2 }, .{ 30, 4 },  .{ 30, 6 },
        .{ 30, 7 }, .{ 30, 8 }, .{ 36, 3 }, .{ 36, 8 }, .{ 20, 12 },
    };
    var too_long_count: usize = 0;
    var traced_count: usize = 0;
    for (sizes) |size| {
        var input: LayerInput = try .initBlank(gpa, size[0], size[1] * 2 - 1);
        defer input.deinit(gpa);
        fillSerpentine(&input);
        const blob = try packBoth(gpa, &codecs, &input) orelse return error.TestUnexpectedResult;
        defer gpa.free(blob);
        for ([_]f32{ 0, 1.3 }) |max_error| {
            errdefer std.debug.print(
                "  {d} cells wide, {d} whole rows, error {d}\n",
                .{ size[0], size[1], max_error },
            );
            scratch.reset();
            const pair = try unpackBoth(&scratch, &codecs, blob) orelse
                return error.TestUnexpectedResult;
            try testing.expectEqual(RegionsOutcome.built, try buildRegionsBoth(&scratch, pair, 0));
            // The winding region, and one for every other cell of the odd
            // rows: few enough that no id reads as a portal.
            const single_count = (@as(usize, size[1]) - 1) * (@as(usize, size[0]) - 1);
            try testing.expectEqual(single_count + 1, @as(usize, pair.port.region_count));

            if (try buildContoursBoth(&scratch, pair, 0, max_error)) |sets| {
                traced_count += 1;
                _ = try buildPolyMeshBoth(&scratch, sets);
                continue;
            }
            // Both sides failed alike; the port alone tells with what.
            const set = builder.allocContourSet(scratch.allocator()) orelse
                return error.OutOfMemory;
            const status = builder.buildContours(scratch.allocator(), pair.port, 0, max_error, set);
            try testing.expectEqual(statusBits(buffer_too_small), statusBits(status));
            // The winding region comes first, so no contour was stored.
            for (set.contours) |contour| {
                try testing.expectEqual(@as(i32, 0), contour.vertex_count);
            }
            builder.freeContourSet(scratch.allocator(), set);
            too_long_count += 1;
        }
    }
    try testing.expect(too_long_count >= 2);
    try testing.expect(traced_count >= 2);
}
