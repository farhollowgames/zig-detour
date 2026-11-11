//! Differential tests of the tile builder: `createNavMeshData` must give the
//! bytes `dtCreateNavMeshData` gives for the same input and refuse what it
//! refuses, and the two byte-order swaps must change the same bytes.
//!
//! The original's own tests (`Tests/Detour`) have no case for the builder or
//! the swaps, so there is nothing of theirs to mirror here.

const std = @import("std");
const builtin = @import("builtin");
const detour = @import("detour");
const support = @import("support.zig");
const reference = support.reference;

const testing = std.testing;

const MeshHeader = detour.MeshHeader;
const NavMeshCreateParams = detour.NavMeshCreateParams;
const OffMeshConnection = detour.nav_mesh.OffMeshConnection;
const BvNode = detour.nav_mesh.BvNode;
const builder = detour.nav_mesh_builder;

/// The index that ends a polygon with fewer vertices than the mesh allows.
const none: u16 = 0xffff;

// ---------------------------------------------------------------------------
// Building on both sides
// ---------------------------------------------------------------------------

/// The start of `slice` as the pointer the original takes, null for an empty
/// slice.
fn manyOrNull(comptime T: type, slice: anytype) ?[*]const T {
    if (slice.len == 0) return null;
    const many: [*]const T = @ptrCast(slice.ptr);
    return many;
}

/// The original's parameters for the same input as `params`: every slice as
/// a pointer, null when it is empty. The result points into what `params`
/// points into.
fn referenceParams(params: *const NavMeshCreateParams) reference.NavMeshCreateParams {
    return .{
        .verts = manyOrNull(u16, params.verts),
        .vert_count = params.vert_count,
        .polys = manyOrNull(u16, params.polys),
        .poly_flags = manyOrNull(u16, params.poly_flags),
        .poly_areas = manyOrNull(u8, params.poly_areas),
        .poly_count = params.poly_count,
        .nvp = params.vertices_per_polygon,
        .detail_meshes = if (params.detail_meshes) |meshes| manyOrNull(u32, meshes) else null,
        .detail_verts = manyOrNull(f32, params.detail_verts),
        .detail_verts_count = params.detail_verts_count,
        .detail_tris = manyOrNull(u8, params.detail_tris),
        .detail_tri_count = params.detail_tri_count,
        .off_mesh_con_verts = manyOrNull(f32, params.off_mesh_con_verts),
        .off_mesh_con_rad = manyOrNull(f32, params.off_mesh_con_rad),
        .off_mesh_con_flags = manyOrNull(u16, params.off_mesh_con_flags),
        .off_mesh_con_areas = manyOrNull(u8, params.off_mesh_con_areas),
        .off_mesh_con_dir = manyOrNull(u8, params.off_mesh_con_dir),
        .off_mesh_con_user_id = if (params.off_mesh_con_user_id) |ids|
            manyOrNull(u32, ids)
        else
            null,
        .off_mesh_con_count = params.off_mesh_con_count,
        .user_id = params.user_id,
        .tile_x = params.tile_x,
        .tile_y = params.tile_y,
        .tile_layer = params.tile_layer,
        .bmin = params.bounds_min,
        .bmax = params.bounds_max,
        .walkable_height = params.walkable_height,
        .walkable_radius = params.walkable_radius,
        .walkable_climb = params.walkable_climb,
        .cs = params.cell_size,
        .ch = params.cell_height,
        .build_bv_tree = params.build_bv_tree,
    };
}

/// A size the original never gives, to see that a refusal leaves its outputs
/// alone.
const untouched_size: i32 = -0x5a5a5a5b;

/// Builds the tile on both sides. Fails unless both build the same bytes, or
/// both refuse and the original leaves its outputs untouched. Returns the
/// port's bytes, which the caller frees with `gpa`, or null for a refusal.
fn buildBoth(
    gpa: std.mem.Allocator,
    reference_params: *const reference.NavMeshCreateParams,
    params: *const NavMeshCreateParams,
) !?[]align(4) u8 {
    var untouched: [1]u8 = .{0xa5};
    var expected_data: ?[*]u8 = &untouched;
    var expected_size: i32 = untouched_size;
    const expected_built = reference.ref_create_nav_mesh_data(
        reference_params,
        &expected_data,
        &expected_size,
    );
    defer if (expected_built) {
        if (expected_data) |data| reference.ref_free(data);
    };

    const built: ?[]align(4) u8 = detour.createNavMeshData(gpa, params) catch |err| switch (err) {
        error.InvalidParam => null,
        error.OutOfMemory => return err,
    };
    errdefer if (built) |bytes| gpa.free(bytes);

    if (!expected_built) {
        try testing.expectEqual(@intFromPtr(&untouched), @intFromPtr(expected_data));
        try testing.expectEqual(untouched_size, expected_size);
    }
    if (expected_built != (built != null)) {
        std.debug.print("the original {s} the input, the port {s} it\n", .{
            if (expected_built) "builds" else "refuses",
            if (built != null) "builds" else "refuses",
        });
        return error.TestExpectedEqual;
    }
    const bytes = built orelse return null;
    try testing.expect(expected_data != null);
    try testing.expect(expected_size >= 0);
    try support.expectBytes(expected_data.?[0..@intCast(expected_size)], bytes);
    return bytes;
}

/// Fails unless both sides build the same bytes from `params`. Returns the
/// port's, which the caller frees with `gpa`.
fn expectBuilt(gpa: std.mem.Allocator, params: *const NavMeshCreateParams) ![]align(4) u8 {
    const reference_params = referenceParams(params);
    return (try buildBoth(gpa, &reference_params, params)) orelse {
        std.debug.print("both sides refuse the input\n", .{});
        return error.TestUnexpectedResult;
    };
}

/// Fails unless both sides build the same bytes from `params`. Returns the
/// header of the tile.
fn expectBuiltHeader(gpa: std.mem.Allocator, params: *const NavMeshCreateParams) !MeshHeader {
    const bytes = try expectBuilt(gpa, params);
    defer gpa.free(bytes);
    return headerOf(bytes);
}

/// Fails unless both sides build the same bytes, the original from
/// `reference_params` and the port from what `support.portCreateParams`
/// makes of them. Returns the port's, which the caller frees with `gpa`.
fn expectBuiltFromReference(
    gpa: std.mem.Allocator,
    reference_params: *const reference.NavMeshCreateParams,
) ![]align(4) u8 {
    const params = support.portCreateParams(reference_params);
    return (try buildBoth(gpa, reference_params, &params)) orelse {
        std.debug.print("both sides refuse the input\n", .{});
        return error.TestUnexpectedResult;
    };
}

/// Fails unless both sides refuse `params`.
fn expectRefusal(gpa: std.mem.Allocator, params: *const NavMeshCreateParams) !void {
    const reference_params = referenceParams(params);
    if (try buildBoth(gpa, &reference_params, params)) |bytes| {
        gpa.free(bytes);
        std.debug.print("both sides build the input\n", .{});
        return error.TestUnexpectedResult;
    }
}

/// Fails unless the port refuses `params` as invalid. For input the original
/// cannot be given, because it would read or write outside its arrays.
fn expectPortRefusal(gpa: std.mem.Allocator, params: *const NavMeshCreateParams) !void {
    if (detour.createNavMeshData(gpa, params)) |bytes| {
        gpa.free(bytes);
        std.debug.print("the port builds the input\n", .{});
        return error.TestUnexpectedResult;
    } else |err| {
        try testing.expectEqual(error.InvalidParam, err);
    }
}

/// The header a tile's bytes start with.
fn headerOf(bytes: []const u8) MeshHeader {
    return std.mem.bytesToValue(MeshHeader, bytes[0..@sizeOf(MeshHeader)]);
}

/// The off-mesh connection at `index` of those a tile's bytes end with.
fn storedConnection(bytes: []const u8, index: usize) OffMeshConnection {
    const count: usize = @intCast(headerOf(bytes).off_mesh_connection_count);
    const size = @sizeOf(OffMeshConnection);
    const start = bytes.len - (count - index) * size;
    return std.mem.bytesToValue(OffMeshConnection, bytes[start..][0..size]);
}

/// The node at `index` of the bounding-volume tree in a tile's bytes, which
/// sits just before the off-mesh connections.
fn storedBvNode(bytes: []const u8, index: usize) BvNode {
    const header = headerOf(bytes);
    const node_count: usize = @intCast(header.bv_node_count);
    const connection_count: usize = @intCast(header.off_mesh_connection_count);
    const size = @sizeOf(BvNode);
    const tree_end = bytes.len - connection_count * @sizeOf(OffMeshConnection);
    const start = tree_end - (node_count - index) * size;
    return std.mem.bytesToValue(BvNode, bytes[start..][0..size]);
}

// ---------------------------------------------------------------------------
// A mesh made by hand
// ---------------------------------------------------------------------------

/// Two quads and a triangle over seven vertices, for cells of 0.5 by 0.25
/// from (10, 2, -20): every coordinate is exact in a float, so a point can be
/// put exactly on a bound. The vertices' heights run from 2 to 4.
const small_verts = [_][3]u16{
    .{ 0, 0, 0 },
    .{ 4, 0, 0 },
    .{ 4, 2, 4 },
    .{ 0, 2, 4 },
    .{ 8, 4, 0 },
    .{ 8, 8, 4 },
    .{ 4, 6, 8 },
};

/// Each polygon's vertices, then what lies beyond each of its edges: another
/// polygon, a wall (0x800f) or the tile's border in one of four directions.
const small_polys = [_]u16{
    0, 1, 2, 3,    0x8003, 1,      2,      0x8000,
    1, 4, 5, 2,    0x8003, 0x8002, 0x800f, 0,
    3, 2, 6, none, 0,      0x800f, 0x8001, none,
};
const small_flags = [_]u16{ 1, 3, 0x8001 };
const small_areas = [_]u8{ 0, 5, 63 };

/// A detail mesh for the polygons above. Each part starts with its polygon's
/// vertices; the first and the last add vertices of their own, one at height
/// 1 and one at height 6, outside what the mesh's vertices span.
const small_detail_verts = [_][3]f32{
    .{ 10, 2, -20 },
    .{ 12, 2, -20 },
    .{ 12, 2.5, -18 },
    .{ 10, 2.5, -18 },
    .{ 11, 1, -19 },

    .{ 12, 2, -20 },
    .{ 14, 3, -20 },
    .{ 14, 4, -18 },
    .{ 12, 2.5, -18 },

    .{ 10, 2.5, -18 },
    .{ 12, 2.5, -18 },
    .{ 12, 3.5, -16 },
    .{ 11.5, 6, -17 },
    .{ 11, 3, -17.5 },
};

/// The vertex base, vertex count, triangle base and triangle count of each
/// polygon's part of the detail mesh.
const small_detail_meshes = [_][4]u32{
    .{ 0, 5, 0, 4 },
    .{ 5, 4, 4, 2 },
    .{ 9, 5, 6, 3 },
};
const small_detail_tris = [_][4]u8{
    .{ 0, 1, 4, 0x01 },
    .{ 1, 2, 4, 0x01 },
    .{ 2, 3, 4, 0x01 },
    .{ 3, 0, 4, 0x01 },
    .{ 0, 1, 2, 0x05 },
    .{ 0, 2, 3, 0x14 },
    .{ 0, 1, 4, 0x01 },
    .{ 1, 2, 3, 0x05 },
    .{ 1, 3, 4, 0x00 },
};

/// The hand-made mesh without a detail mesh.
fn smallMesh() NavMeshCreateParams {
    return .{
        .verts = &small_verts,
        .vert_count = small_verts.len,
        .polys = &small_polys,
        .poly_flags = &small_flags,
        .poly_areas = &small_areas,
        .poly_count = small_flags.len,
        .vertices_per_polygon = 4,
        .bounds_min = .{ 10, 2, -20 },
        .bounds_max = .{ 14, 5, -16 },
        .walkable_height = 2,
        .walkable_radius = 0.5,
        .walkable_climb = 0.5,
        .cell_size = 0.5,
        .cell_height = 0.25,
        .build_bv_tree = true,
    };
}

/// The hand-made mesh with its detail mesh.
fn smallDetailedMesh() NavMeshCreateParams {
    var params = smallMesh();
    params.detail_meshes = &small_detail_meshes;
    params.detail_verts = &small_detail_verts;
    params.detail_verts_count = small_detail_verts.len;
    params.detail_tris = &small_detail_tris;
    params.detail_tri_count = small_detail_tris.len;
    return params;
}

// ---------------------------------------------------------------------------
// Levels
// ---------------------------------------------------------------------------

const LevelCase = struct {
    name: []const u8,
    obj_text: []const u8,
    config: support.RecastConfig,
};

const level_cases = [_]LevelCase{
    .{
        .name = "dungeon, one tile",
        .obj_text = support.dungeon_obj,
        .config = support.solo_config,
    },
    .{
        .name = "nav_test, tiled",
        .obj_text = support.nav_test_obj,
        .config = support.tiled_config,
    },
    .{
        .name = "dungeon, tiled",
        .obj_text = support.dungeon_obj,
        .config = support.tiled_config,
    },
    .{
        .name = "undulating, one tile",
        .obj_text = support.undulating_obj,
        .config = support.solo_config,
    },
};

/// What the levels get beside their meshes: connections between random
/// places on the ground, and polygons with differing flags and areas.
const level_options: support.LevelOptions = .{
    .off_mesh_connection_count = 24,
    .vary_polygons = true,
    .seed = 7,
};

test "createNavMeshData: every tile of the levels" {
    const gpa = testing.allocator;
    var tile_count: usize = 0;
    var stored_count: i64 = 0;
    var dropped_count: i64 = 0;
    var detail_vert_count: i64 = 0;
    var bv_node_count: i64 = 0;

    for (level_cases) |case| {
        var level: support.Level = try .init(gpa, case.obj_text, case.config, level_options);
        defer level.deinit();

        var y: i32 = 0;
        while (y < level.grid.height) : (y += 1) {
            var x: i32 = 0;
            while (x < level.grid.width) : (x += 1) {
                errdefer std.debug.print("  level \"{s}\", tile {d}, {d}\n", .{ case.name, x, y });
                var input = (try level.tileInput(x, y)) orelse continue;
                defer input.deinit();

                const bytes = try expectBuiltFromReference(gpa, &input.reference_params);
                defer gpa.free(bytes);

                const header = headerOf(bytes);
                const stored = header.off_mesh_connection_count;
                tile_count += 1;
                stored_count += stored;
                dropped_count += level.off_mesh_connections.count() - stored;
                detail_vert_count += header.detail_vert_count;
                bv_node_count += header.bv_node_count;
            }
        }
    }

    // The levels reach each part of a tile: connections stored and left out,
    // detail vertices beside the polygons' own, and a tree.
    try testing.expect(tile_count > level_cases.len);
    try testing.expect(stored_count > 0);
    try testing.expect(dropped_count > 0);
    try testing.expect(detail_vert_count > 0);
    try testing.expect(bv_node_count > 0);
}

/// One change to a level tile's input.
const Variant = enum {
    /// No detail mesh: polygons become fans of triangles. The detail
    /// vertices still bound the heights of the connections' starts.
    flat,
    flat_no_detail_verts,
    flat_zero_detail_count,
    /// The detail mesh stays, and the mesh's vertices bound the heights.
    vertex_heights,
    no_tree,
    flat_no_tree,
    no_connections,
    no_connection_arrays,
    no_user_ids,
    one_way,
    two_way,
    two_way_any_bit,
    layer_and_user_id,
};

/// Makes `variant`'s change to a tile's input. `directions` has an entry for
/// each of the level's connections, for the variants that set them all.
fn applyVariant(
    variant: Variant,
    params: *reference.NavMeshCreateParams,
    directions: []u8,
) void {
    switch (variant) {
        .flat => params.detail_meshes = null,
        .flat_no_detail_verts => {
            params.detail_meshes = null;
            params.detail_verts = null;
        },
        .flat_zero_detail_count => {
            params.detail_meshes = null;
            params.detail_verts_count = 0;
        },
        .vertex_heights => params.detail_verts_count = 0,
        .no_tree => params.build_bv_tree = false,
        .flat_no_tree => {
            params.detail_meshes = null;
            params.build_bv_tree = false;
        },
        .no_connections => params.off_mesh_con_count = 0,
        .no_connection_arrays => {
            params.off_mesh_con_count = 0;
            params.off_mesh_con_verts = null;
            params.off_mesh_con_rad = null;
            params.off_mesh_con_flags = null;
            params.off_mesh_con_areas = null;
            params.off_mesh_con_dir = null;
            params.off_mesh_con_user_id = null;
        },
        .no_user_ids => params.off_mesh_con_user_id = null,
        .one_way, .two_way, .two_way_any_bit => {
            // Any direction but zero means both ways.
            const direction: u8 = switch (variant) {
                .one_way => 0,
                .two_way => 1,
                else => 0xff,
            };
            @memset(directions, direction);
            params.off_mesh_con_dir = directions.ptr;
        },
        .layer_and_user_id => {
            params.tile_layer = 3;
            params.user_id = 0xdeadbeef;
        },
    }
}

test "createNavMeshData: the levels' tiles with each option changed" {
    const gpa = testing.allocator;
    const variants = comptime std.enums.values(Variant);
    var changed: [variants.len]usize = @splat(0);

    for (level_cases) |case| {
        var level: support.Level = try .init(gpa, case.obj_text, case.config, level_options);
        defer level.deinit();

        const directions = try gpa.alloc(u8, @intCast(level.off_mesh_connections.count()));
        defer gpa.free(directions);

        var y: i32 = 0;
        while (y < level.grid.height) : (y += 1) {
            var x: i32 = 0;
            while (x < level.grid.width) : (x += 1) {
                var input = (try level.tileInput(x, y)) orelse continue;
                defer input.deinit();

                const plain = try expectBuiltFromReference(gpa, &input.reference_params);
                defer gpa.free(plain);

                for (variants, &changed) |variant, *changed_count| {
                    errdefer std.debug.print("  level \"{s}\", tile {d}, {d}, variant {s}\n", .{
                        case.name,
                        x,
                        y,
                        @tagName(variant),
                    });
                    var edited = input.reference_params;
                    applyVariant(variant, &edited, directions);
                    var params = support.portCreateParams(&edited);
                    if (variant == .vertex_heights) {
                        // The count only says how many heights bound the
                        // connections' starts: the detail meshes still read
                        // the vertices, which the port takes as a slice.
                        const whole = support.portCreateParams(&input.reference_params);
                        params.detail_verts = whole.detail_verts;
                    }

                    const bytes = (try buildBoth(gpa, &edited, &params)) orelse {
                        return error.TestUnexpectedResult;
                    };
                    defer gpa.free(bytes);
                    if (!std.mem.eql(u8, plain, bytes)) changed_count.* += 1;
                }
            }
        }
    }

    // Each change reached the bytes of some tile. Taking the heights from
    // the mesh's vertices is left out: it changes a tile only when it changes
    // which connections start inside it.
    for (variants, changed) |variant, changed_count| {
        if (variant == .vertex_heights) continue;
        errdefer std.debug.print("  variant {s} changed no tile\n", .{@tagName(variant)});
        try testing.expect(changed_count > 0);
    }
}

// ---------------------------------------------------------------------------
// Off-mesh connections
// ---------------------------------------------------------------------------

/// Off-mesh connections gathered one at a time, in the layout the port's
/// builder takes.
const Connections = struct {
    verts: std.ArrayList([2][3]f32) = .empty,
    radii: std.ArrayList(f32) = .empty,
    flags: std.ArrayList(u16) = .empty,
    areas: std.ArrayList(u8) = .empty,
    directions: std.ArrayList(u8) = .empty,
    user_ids: std.ArrayList(u32) = .empty,

    fn deinit(self: *Connections, gpa: std.mem.Allocator) void {
        self.verts.deinit(gpa);
        self.radii.deinit(gpa);
        self.flags.deinit(gpa);
        self.areas.deinit(gpa);
        self.directions.deinit(gpa);
        self.user_ids.deinit(gpa);
        self.* = undefined;
    }

    fn count(self: *const Connections) i32 {
        return @intCast(self.radii.items.len);
    }

    /// Adds a connection from `start` to `end`. Everything else about it
    /// follows from its index, so the tile's bytes tell which were stored.
    fn add(self: *Connections, gpa: std.mem.Allocator, start: [3]f32, end: [3]f32) !void {
        const index: u32 = @intCast(self.radii.items.len);
        const radius = 0.25 + @as(f32, @floatFromInt(index % 16)) * 0.125;
        try self.verts.append(gpa, .{ start, end });
        try self.radii.append(gpa, radius);
        try self.flags.append(gpa, @truncate(index *% 0x9e37 +% 1));
        // Areas beyond the six bits a polygon keeps, too.
        try self.areas.append(gpa, @truncate(index *% 7));
        // Any direction but zero means both ways.
        try self.directions.append(gpa, @truncate(index % 3));
        try self.user_ids.append(gpa, 0x1000_0000 + index);
    }

    /// Points `params` at the connections: dead once another is added.
    fn giveTo(self: *const Connections, params: *NavMeshCreateParams) void {
        params.off_mesh_con_verts = self.verts.items;
        params.off_mesh_con_rad = self.radii.items;
        params.off_mesh_con_flags = self.flags.items;
        params.off_mesh_con_areas = self.areas.items;
        params.off_mesh_con_dir = self.directions.items;
        params.off_mesh_con_user_id = self.user_ids.items;
        params.off_mesh_con_count = self.count();
    }
};

/// Coordinates on, just inside, just outside and well away from the range
/// `low` to `high`, with the values that compare with nothing or lie beyond
/// everything.
fn boundaryValues(low: f32, high: f32) [10]f32 {
    const inf = std.math.inf(f32);
    return .{
        low - 1,
        low,
        std.math.nextAfter(f32, low, -inf),
        (low + high) / 2,
        std.math.nextAfter(f32, high, -inf),
        high,
        high + 1,
        std.math.nan(f32),
        inf,
        -inf,
    };
}

/// Heights on and to either side of each bound the hand-made mesh can give
/// the connections' starts: 1.5 to 4.5 from its vertices, 0.5 to 6.5 from its
/// detail vertices.
fn boundaryHeights() [16]f32 {
    const inf = std.math.inf(f32);
    var heights: [16]f32 = undefined;
    heights[0] = 3;
    heights[1] = std.math.nan(f32);
    heights[2] = inf;
    heights[3] = -inf;
    for ([_]f32{ 1.5, 4.5, 0.5, 6.5 }, 0..) |bound, i| {
        heights[4 + i * 3 + 0] = bound;
        heights[4 + i * 3 + 1] = std.math.nextAfter(f32, bound, -inf);
        heights[4 + i * 3 + 2] = std.math.nextAfter(f32, bound, inf);
    }
    return heights;
}

/// One of `values`, as `random` chooses.
fn pick(random: std.Random, values: []const f32) f32 {
    return values[random.uintLessThan(usize, values.len)];
}

/// Connections whose ends sit on, inside and outside each bound of the
/// hand-made mesh's tile: every start with an end inside, every end with a
/// start inside, then random pairs of the same places. Free with `deinit`.
fn boundaryConnections(gpa: std.mem.Allocator) !Connections {
    const xs = boundaryValues(10, 14);
    const zs = boundaryValues(-20, -16);
    const heights = boundaryHeights();
    const inside: [3]f32 = .{ 12, 3, -18 };

    var connections: Connections = .{};
    errdefer connections.deinit(gpa);
    for (xs) |x| {
        for (zs) |z| {
            for (heights) |height| try connections.add(gpa, .{ x, height, z }, inside);
        }
    }
    // An end's height does not decide its side.
    for (xs) |x| {
        for (zs) |z| {
            for ([_]f32{ 3, -50, std.math.nan(f32) }) |height| {
                try connections.add(gpa, inside, .{ x, height, z });
            }
        }
    }
    var prng: std.Random.DefaultPrng = .init(0x0ffe5);
    const random = prng.random();
    for (0..2000) |_| {
        const start: [3]f32 = .{ pick(random, &xs), pick(random, &heights), pick(random, &zs) };
        const end: [3]f32 = .{ pick(random, &xs), pick(random, &heights), pick(random, &zs) };
        try connections.add(gpa, start, end);
    }
    return connections;
}

/// Where a build of the hand-made mesh takes the heights that bound the
/// connections' starts.
const HeightSource = enum {
    vertices,
    detail_verts,
    detail_verts_without_meshes,
    first_four_detail_verts,
    no_detail_vert_count,
    nan_first,
    nan_in_the_middle,
    nan_last,
};

/// The hand-made mesh set up for `source`. Its detail vertices are copied to
/// `detail_verts`, which the result points into.
fn heightSourceParams(
    source: HeightSource,
    detail_verts: *[small_detail_verts.len][3]f32,
) NavMeshCreateParams {
    detail_verts.* = small_detail_verts;
    var params = smallDetailedMesh();
    params.detail_verts = detail_verts;
    switch (source) {
        .vertices => params = smallMesh(),
        .detail_verts => {},
        .detail_verts_without_meshes => params.detail_meshes = null,
        .first_four_detail_verts => params.detail_verts_count = 4,
        .no_detail_vert_count => params.detail_verts_count = 0,
        .nan_first, .nan_in_the_middle, .nan_last => {
            const index: usize = switch (source) {
                .nan_first => 0,
                .nan_in_the_middle => 9,
                else => small_detail_verts.len - 1,
            };
            detail_verts[index][1] = std.math.nan(f32);
            // A tree would make a box coordinate of the height that is not a
            // number, which the original leaves undefined.
            params.build_bv_tree = false;
        },
    }
    return params;
}

test "createNavMeshData: connections on, inside and outside each bound" {
    const gpa = testing.allocator;
    var connections = try boundaryConnections(gpa);
    defer connections.deinit(gpa);

    const sources = comptime std.enums.values(HeightSource);
    var stored: [sources.len]i32 = undefined;
    for (sources, &stored) |source, *stored_count| {
        errdefer std.debug.print("  heights from {s}\n", .{@tagName(source)});
        var detail_verts: [small_detail_verts.len][3]f32 = undefined;
        var params = heightSourceParams(source, &detail_verts);
        connections.giveTo(&params);

        const bytes = try expectBuilt(gpa, &params);
        defer gpa.free(bytes);
        stored_count.* = headerOf(bytes).off_mesh_connection_count;

        // Some connections are stored and some left out, and the stored
        // ones end beyond each side of the tile and inside it.
        try testing.expect(stored_count.* > 0);
        try testing.expect(stored_count.* < connections.count());
        var side_seen: [256]bool = @splat(false);
        const stored_len: usize = @intCast(stored_count.*);
        for (0..stored_len) |i| side_seen[storedConnection(bytes, i).side] = true;
        for (0..8) |side| try testing.expect(side_seen[side]);
        try testing.expect(side_seen[0xff]);
    }

    // The sources give the bounds they should: wider bounds store more. A
    // height that is not a number makes the bounds forget every height
    // before it, so as the last one it leaves no bound at all.
    const vertices = stored[@backingInt(HeightSource.vertices)];
    const detail = stored[@backingInt(HeightSource.detail_verts)];
    try testing.expect(detail > vertices);
    try testing.expectEqual(
        detail,
        stored[@backingInt(HeightSource.detail_verts_without_meshes)],
    );
    try testing.expect(stored[@backingInt(HeightSource.first_four_detail_verts)] < vertices);
    try testing.expectEqual(vertices, stored[@backingInt(HeightSource.no_detail_vert_count)]);
    try testing.expectEqual(detail, stored[@backingInt(HeightSource.nan_first)]);
    try testing.expect(stored[@backingInt(HeightSource.nan_in_the_middle)] < detail);
    try testing.expect(stored[@backingInt(HeightSource.nan_last)] > detail);
}

// ---------------------------------------------------------------------------
// What is refused
// ---------------------------------------------------------------------------

test "createNavMeshData: what both sides refuse" {
    const gpa = testing.allocator;

    // More vertices in a polygon than a tile's polygons hold.
    {
        var params = smallMesh();
        params.vertices_per_polygon = detour.nav_mesh.verts_per_polygon + 1;
        try expectRefusal(gpa, &params);
    }
    // 0xffff vertices or more: as an index 0xffff ends a polygon.
    {
        const verts = try gpa.alloc([3]u16, 0x10000);
        defer gpa.free(verts);
        @memset(verts, .{ 0, 0, 0 });
        var params = smallMesh();
        params.verts = verts;
        for ([_]i32{ 0xffff, 0x10000 }) |vert_count| {
            params.vert_count = vert_count;
            try expectRefusal(gpa, &params);
        }
    }
    // No vertices: by the count, then by the array.
    {
        var params = smallMesh();
        params.vert_count = 0;
        try expectRefusal(gpa, &params);
        params = smallMesh();
        params.verts = &.{};
        try expectRefusal(gpa, &params);
    }
    // No polygons: by the count, then by the array.
    {
        var params = smallMesh();
        params.poly_count = 0;
        try expectRefusal(gpa, &params);
        params = smallMesh();
        params.polys = &.{};
        try expectRefusal(gpa, &params);
    }
    // A refusal comes before the connections are looked at.
    {
        var connections: Connections = .{};
        defer connections.deinit(gpa);
        try connections.add(gpa, .{ 12, 3, -18 }, .{ 12, 3, -18 });
        var params = smallDetailedMesh();
        connections.giveTo(&params);
        params.poly_count = 0;
        try expectRefusal(gpa, &params);
    }
}

test "createNavMeshData: what the port refuses and the original cannot be given" {
    const gpa = testing.allocator;

    // Negative counts, which the original takes for the sizes of its arrays.
    for ([_]i32{ -1, -7, std.math.minInt(i32) }) |count| {
        var params = smallMesh();
        params.vert_count = count;
        try expectPortRefusal(gpa, &params);
        params = smallMesh();
        params.poly_count = count;
        try expectPortRefusal(gpa, &params);
        params = smallMesh();
        params.vertices_per_polygon = count;
        try expectPortRefusal(gpa, &params);
        params = smallDetailedMesh();
        params.detail_tri_count = count;
        try expectPortRefusal(gpa, &params);
    }

    // A detail mesh with fewer vertices than its polygon, of which the
    // original copies a negative number. The second polygon has four.
    for ([_]u32{ 3, 0, 0x8000_0000, 0xffff_ffff }) |detail_vert_count| {
        var detail_meshes = small_detail_meshes;
        detail_meshes[1][1] = detail_vert_count;
        var params = smallDetailedMesh();
        params.detail_meshes = &detail_meshes;
        try expectPortRefusal(gpa, &params);
    }

    // Without a detail mesh a polygon is stored as a fan of its vertex count
    // less two triangles. Polygons of fewer than two vertices take triangles
    // away from the total, and below zero the original builds a tile whose
    // blocks overlap.
    const lone_point = [_]u16{ 0, none, none, 0x800f, none, none };
    const points_beside_a_triangle = [_]u16{
        0, none, none, 0x800f, none,   none,
        1, 2,    3,    0x800f, 0x800f, 0x800f,
        4, none, none, 0x800f, none,   none,
    };
    const no_vertices = [_]u16{ none, none, none, none, none, none };
    for ([_]bool{ false, true }) |with_tree| {
        var params = smallMesh();
        params.vertices_per_polygon = 3;
        params.build_bv_tree = with_tree;

        params.polys = &lone_point;
        params.poly_count = 1;
        try expectPortRefusal(gpa, &params);

        params.polys = &points_beside_a_triangle;
        params.poly_count = 3;
        try expectPortRefusal(gpa, &params);

        params.polys = &no_vertices;
        params.poly_count = 1;
        try expectPortRefusal(gpa, &params);

        // No vertices in any polygon, by the mesh's own limit.
        params = smallMesh();
        params.vertices_per_polygon = 0;
        params.build_bv_tree = with_tree;
        try expectPortRefusal(gpa, &params);
    }
}

// ---------------------------------------------------------------------------
// The smallest meshes
// ---------------------------------------------------------------------------

/// Flags and areas for hand-made polygons. They differ, so a polygon stored
/// in the wrong place shows, and the areas go beyond the six bits a polygon
/// keeps.
const hand_flags = [_]u16{ 1, 2, 4, 8, 0x10, 0x8000, 3, 0xffff, 0, 5, 6, 7, 9, 10, 11, 12 };
const hand_areas = [_]u8{ 0, 1, 2, 3, 63, 64, 128, 255, 0x7f, 5, 6, 7, 8, 9, 10, 11 };

/// Parameters for a hand-made mesh without a detail mesh, on RecastDemo's
/// default cells from the origin, with a tree. One polygon for each entry of
/// `poly_flags`.
fn handParams(
    verts: []const [3]u16,
    polys: []const u16,
    vertices_per_polygon: i32,
    poly_flags: []const u16,
    poly_areas: []const u8,
) NavMeshCreateParams {
    return .{
        .verts = verts,
        .vert_count = @intCast(verts.len),
        .polys = polys,
        .poly_flags = poly_flags,
        .poly_areas = poly_areas,
        .poly_count = @intCast(poly_flags.len),
        .vertices_per_polygon = vertices_per_polygon,
        .bounds_min = .{ 0, 0, 0 },
        .bounds_max = .{ 30, 10, 30 },
        .walkable_height = 2,
        .walkable_radius = 0.6,
        .walkable_climb = 0.9,
        .cell_size = 0.3,
        .cell_height = 0.2,
        .build_bv_tree = true,
    };
}

test "createNavMeshData: one triangle" {
    const gpa = testing.allocator;
    const verts = [_][3]u16{ .{ 0, 0, 0 }, .{ 0, 0, 3 }, .{ 3, 1, 0 } };
    const polys = [_]u16{ 0, 1, 2, 0x800f, 0x800f, 0x800f };
    const detail_meshes = [_][4]u32{.{ 0, 3, 0, 1 }};
    const detail_verts = [_][3]f32{ .{ 0, 0, 0 }, .{ 0, 0, 0.9 }, .{ 0.9, 0.2, 0 } };
    const detail_tris = [_][4]u8{.{ 0, 1, 2, 0x15 }};

    for ([_]bool{ false, true }) |with_detail| {
        for ([_]bool{ false, true }) |with_tree| {
            errdefer std.debug.print("  detail {any}, tree {any}\n", .{ with_detail, with_tree });
            var params = handParams(&verts, &polys, 3, hand_flags[0..1], hand_areas[0..1]);
            params.build_bv_tree = with_tree;
            if (with_detail) {
                params.detail_meshes = &detail_meshes;
                params.detail_verts = &detail_verts;
                params.detail_verts_count = detail_verts.len;
                params.detail_tris = &detail_tris;
                params.detail_tri_count = detail_tris.len;
            }

            const header = try expectBuiltHeader(gpa, &params);
            try testing.expectEqual(1, header.poly_count);
            try testing.expectEqual(3, header.vert_count);
            try testing.expectEqual(3, header.link_count_max);
            try testing.expectEqual(1, header.detail_mesh_count);
            try testing.expectEqual(0, header.detail_vert_count);
            try testing.expectEqual(1, header.detail_tri_count);
            try testing.expectEqual(@as(i32, if (with_tree) 2 else 0), header.bv_node_count);
            try testing.expectEqual(0, header.off_mesh_connection_count);
        }
    }
}

test "createNavMeshData: one vertex" {
    const gpa = testing.allocator;
    // A mesh of one vertex holds a polygon of that vertex and no more, which
    // only a detail mesh can describe: it has no fan.
    const verts = [_][3]u16{.{ 5, 2, 7 }};
    const polys = [_]u16{ 0, none, none, 0x800f, none, none };
    const detail_meshes = [_][4]u32{.{ 0, 1, 0, 1 }};
    const detail_verts = [_][3]f32{.{ 1.5, 0.4, 2.1 }};
    const detail_tris = [_][4]u8{.{ 0, 0, 0, 0 }};

    for ([_]bool{ false, true }) |with_tree| {
        errdefer std.debug.print("  tree {any}\n", .{with_tree});
        var params = handParams(&verts, &polys, 3, hand_flags[0..1], hand_areas[0..1]);
        params.build_bv_tree = with_tree;
        params.detail_meshes = &detail_meshes;
        params.detail_verts = &detail_verts;
        params.detail_verts_count = detail_verts.len;
        params.detail_tris = &detail_tris;
        params.detail_tri_count = detail_tris.len;

        const header = try expectBuiltHeader(gpa, &params);
        try testing.expectEqual(1, header.poly_count);
        try testing.expectEqual(1, header.vert_count);
        try testing.expectEqual(1, header.link_count_max);
        try testing.expectEqual(0, header.detail_vert_count);
        try testing.expectEqual(1, header.detail_tri_count);
    }
}

/// Gives `params` a detail mesh, all of each array counted.
fn giveDetail(
    params: *NavMeshCreateParams,
    detail_meshes: []const [4]u32,
    detail_verts: []const [3]f32,
    detail_tris: []const [4]u8,
) void {
    params.detail_meshes = detail_meshes;
    params.detail_verts = detail_verts;
    params.detail_verts_count = @intCast(detail_verts.len);
    params.detail_tris = detail_tris;
    params.detail_tri_count = @intCast(detail_tris.len);
}

/// The polygon at `index` of those a tile's bytes hold after its vertices.
fn storedPoly(bytes: []const u8, index: usize) detour.Poly {
    const vert_count: usize = @intCast(headerOf(bytes).vert_count);
    const size = @sizeOf(detour.Poly);
    const start = @sizeOf(MeshHeader) + vert_count * @sizeOf([3]f32) + index * size;
    return std.mem.bytesToValue(detour.Poly, bytes[start..][0..size]);
}

test "createNavMeshData: polygons of one or two vertices at most" {
    const gpa = testing.allocator;
    const verts = [_][3]u16{ .{ 1, 0, 1 }, .{ 9, 3, 2 }, .{ 4, 7, 8 } };
    const detail_verts = [_][3]f32{
        .{ 0.3, 0, 0.3 },
        .{ 2.7, 0.6, 0.6 },
        .{ 2.8, 0.9, 0.7 },
        .{ 1.2, 1.4, 2.4 },
        .{ 1.3, 1.5, 2.5 },
        .{ 0.3, 0.1, 0.3 },
    };
    // No triangle is copied, but the original copies none from this array:
    // it must not be given a null pointer to copy from.
    const no_tris = [_][4]u8{.{ 9, 9, 9, 9 }};

    // One vertex to a polygon: an entry for the vertex and one for what lies
    // beyond its only edge.
    const point_polys = [_]u16{ 0, 0x800f, 1, 0x8001, 2, 0 };
    // A detail mesh's triangle base and count are stored as they are,
    // whatever they say; the count keeps its low eight bits.
    const point_detail_meshes = [_][4]u32{
        .{ 0, 1, 7, 3 },
        .{ 1, 2, 0, 0 },
        .{ 3, 1, 0xffff_ffff, 300 },
    };
    // Two vertices at most.
    const segment_polys = [_]u16{
        0, 1,    0x800f, 1,
        1, 2,    0,      0x8002,
        2, none, 0x8003, none,
    };
    const segment_detail_meshes = [_][4]u32{ .{ 0, 2, 0, 0 }, .{ 2, 3, 0, 0 }, .{ 5, 1, 0, 0 } };

    for ([_]bool{ false, true }) |with_tree| {
        errdefer std.debug.print("  tree {any}\n", .{with_tree});

        var points = handParams(&verts, &point_polys, 1, hand_flags[0..3], hand_areas[0..3]);
        points.build_bv_tree = with_tree;
        giveDetail(&points, &point_detail_meshes, &detail_verts, &no_tris);
        points.detail_tri_count = 0;
        var header = try expectBuiltHeader(gpa, &points);
        try testing.expectEqual(5, header.link_count_max);
        try testing.expectEqual(1, header.detail_vert_count);
        try testing.expectEqual(0, header.detail_tri_count);

        var segments = handParams(&verts, &segment_polys, 2, hand_flags[0..3], hand_areas[0..3]);
        segments.build_bv_tree = with_tree;
        giveDetail(&segments, &segment_detail_meshes, &detail_verts, &no_tris);
        segments.detail_tri_count = 0;
        header = try expectBuiltHeader(gpa, &segments);
        try testing.expectEqual(9, header.link_count_max);
        try testing.expectEqual(1, header.detail_vert_count);
        try testing.expectEqual(0, header.detail_tri_count);

        // Without a detail mesh a polygon of two vertices is a fan of no
        // triangles.
        var flat = handParams(&verts, segment_polys[0..8], 2, hand_flags[0..2], hand_areas[0..2]);
        flat.build_bv_tree = with_tree;
        header = try expectBuiltHeader(gpa, &flat);
        try testing.expectEqual(6, header.link_count_max);
        try testing.expectEqual(0, header.detail_tri_count);
    }
}

/// Eight vertices for hand-made polygons, at heights of 0 to 1 world units.
const ring_verts = [_][3]u16{
    .{ 0, 0, 0 },
    .{ 10, 1, 0 },
    .{ 20, 2, 5 },
    .{ 20, 3, 15 },
    .{ 10, 4, 20 },
    .{ 0, 5, 15 },
    .{ 5, 2, 10 },
    .{ 15, 3, 10 },
};

test "createNavMeshData: polygons of one or two vertices beside larger ones" {
    const gpa = testing.allocator;

    // A polygon of two vertices is a fan of no triangles.
    const with_segment = [_]u16{
        0, 1, 2,    3,    0x800f, 1,      0x800f, 0x8000,
        1, 4, none, none, 0x800f, 2,      none,   none,
        4, 5, 2,    none, 0x8002, 0x800f, 0,      none,
    };
    for ([_]bool{ false, true }) |with_tree| {
        errdefer std.debug.print("  tree {any}\n", .{with_tree});
        var params = handParams(&ring_verts, &with_segment, 4, hand_flags[0..3], hand_areas[0..3]);
        params.build_bv_tree = with_tree;
        const header = try expectBuiltHeader(gpa, &params);
        try testing.expectEqual(3, header.detail_tri_count);
    }

    // A polygon of one vertex takes a triangle away from the room made for
    // them, so the last triangles are written past it, onto the start of the
    // tree. The tree is built afterwards and covers them. Always with a tree
    // here: without one they would be written past the tile's bytes.
    const with_point = [_]u16{
        0, 1,    2,    3,    0x800f, 2,      0x800f, 0x8000,
        4, none, none, none, 0x800f, none,   none,   none,
        4, 5,    2,    none, 0x8002, 0x800f, 0,      none,
    };
    {
        var params = handParams(&ring_verts, &with_point, 4, hand_flags[0..3], hand_areas[0..3]);
        params.build_bv_tree = true;
        const header = try expectBuiltHeader(gpa, &params);
        try testing.expectEqual(2, header.detail_tri_count);
    }

    // Three of them among polygons of up to six vertices: seven triangles
    // written with room for four.
    const with_points = [_]u16{
        0, 1,    2,    3,    6,    7,    0x800f, 0x8002, 3,    0x8001, 0x800f, 0x8000,
        5, none, none, none, none, none, 0x800f, none,   none, none,   none,   none,
        6, none, none, none, none, none, 0,      none,   none, none,   none,   none,
        1, 4,    2,    none, none, none, 0x800f, 5,      0,    none,   none,   none,
        7, none, none, none, none, none, 0x8003, none,   none, none,   none,   none,
        4, 5,    7,    2,    none, none, 0x8002, 0x800f, 0,    3,      none,   none,
    };
    {
        var params = handParams(&ring_verts, &with_points, 6, hand_flags[0..6], hand_areas[0..6]);
        params.build_bv_tree = true;
        const header = try expectBuiltHeader(gpa, &params);
        try testing.expectEqual(4, header.detail_tri_count);
    }
}

test "createNavMeshData: triangles written on into the off-mesh connections" {
    const gpa = testing.allocator;

    // Nine polygons of one vertex and three of six, and the same with two
    // vertices where the first has one.
    var points: [12 * 12]u16 = @splat(none);
    var segments: [12 * 12]u16 = @splat(none);
    for (0..9) |i| {
        const vertex: u16 = @intCast(i % ring_verts.len);
        points[i * 12] = vertex;
        points[i * 12 + 6] = 0x800f;
        segments[i * 12] = vertex;
        segments[i * 12 + 1] = (vertex + 1) % 8;
        segments[i * 12 + 6] = 0x800f;
        segments[i * 12 + 7] = 0x800f;
    }
    const hexagons = [3][12]u16{
        .{ 0, 1, 2, 3, 4, 5, 0x800f, 0x8002, 10, 0x8001, 0x800f, 0x8000 },
        .{ 2, 3, 4, 5, 6, 7, 9, 0x800f, 11, 0x800f, 0x800f, 0x800f },
        .{ 0, 2, 4, 6, 7, 1, 0x800f, 10, 0x800f, 0x800f, 0x800f, 0x8003 },
    };
    for (hexagons, 9..) |hexagon, i| {
        points[i * 12 ..][0..12].* = hexagon;
        segments[i * 12 ..][0..12].* = hexagon;
    }

    // Two connections that start over the tile at the height of its
    // vertices, and between them one that starts beside it.
    var connections: Connections = .{};
    defer connections.deinit(gpa);
    try connections.add(gpa, .{ 15, 0.5, 15 }, .{ 40, 0, 15 });
    try connections.add(gpa, .{ 31, 0.5, 15 }, .{ 15, 0.5, 15 });
    try connections.add(gpa, .{ 3, 1, 29 }, .{ 3, 0, 3 });

    // Without a tree the triangles that do not fit are written onto the
    // connections, which must be there to take them: 36 bytes onto the 72
    // of two connections. The mesh of two-vertex polygons has room for all
    // its triangles, and stores the same connections, so it goes first to
    // show that there are two.
    var roomy = handParams(&ring_verts, &segments, 6, hand_flags[0..12], hand_areas[0..12]);
    roomy.build_bv_tree = false;
    connections.giveTo(&roomy);
    roomy.off_mesh_con_user_id = null;
    const roomy_header = try expectBuiltHeader(gpa, &roomy);
    try testing.expectEqual(2, roomy_header.off_mesh_connection_count);
    try testing.expectEqual(12, roomy_header.detail_tri_count);

    var params = roomy;
    params.polys = &points;
    const bytes = try expectBuilt(gpa, &params);
    defer gpa.free(bytes);
    const header = headerOf(bytes);
    try testing.expectEqual(2, header.off_mesh_connection_count);
    try testing.expectEqual(3, header.detail_tri_count);

    // Each connection's fields are stored over what the triangles left, but
    // for the id the application gives, which is stored only when there are
    // ids: the first connection keeps the last triangle there.
    const first = storedConnection(bytes, 0);
    try testing.expectEqualSlices(u8, &[_]u8{ 0, 4, 5, 0x14 }, std.mem.asBytes(&first.user_id));
    try testing.expectEqual(0, storedConnection(bytes, 1).user_id);

    // With ids nothing of the triangles is left.
    connections.giveTo(&params);
    const with_ids = try expectBuilt(gpa, &params);
    defer gpa.free(with_ids);
    try testing.expectEqual(0x1000_0000, storedConnection(with_ids, 0).user_id);
    try testing.expectEqual(0x1000_0002, storedConnection(with_ids, 1).user_id);
}

test "createNavMeshData: polygons of no vertices, with a detail mesh" {
    const gpa = testing.allocator;
    const verts = [_][3]u16{.{ 3, 1, 4 }};
    // With no vertices to a polygon the polygons' array has no entries. The
    // original still wants a pointer, and the port a slice that is not empty.
    const polys = [_]u16{none};
    // Every detail vertex is the detail mesh's own, and is stored.
    const detail_meshes = [_][4]u32{ .{ 0, 2, 0, 0 }, .{ 2, 1, 0, 1 }, .{ 3, 3, 1, 1 } };
    const detail_verts = [_][3]f32{
        .{ 0.9, 0.2, 1.2 },
        .{ 1.5, 0.4, 1.2 },
        .{ 3, 1, 3 },
        .{ 6, 0, 6 },
        .{ 6, 0.5, 9 },
        .{ 9, 2, 6 },
    };
    const detail_tris = [_][4]u8{ .{ 0, 0, 0, 0 }, .{ 0, 1, 2, 0x15 } };

    for ([_]bool{ false, true }) |with_tree| {
        errdefer std.debug.print("  tree {any}\n", .{with_tree});
        var params = handParams(&verts, &polys, 0, hand_flags[0..3], hand_areas[0..3]);
        params.build_bv_tree = with_tree;
        giveDetail(&params, &detail_meshes, &detail_verts, &detail_tris);

        const bytes = try expectBuilt(gpa, &params);
        defer gpa.free(bytes);
        const header = headerOf(bytes);
        try testing.expectEqual(3, header.poly_count);
        try testing.expectEqual(1, header.vert_count);
        try testing.expectEqual(0, header.link_count_max);
        try testing.expectEqual(6, header.detail_vert_count);
        try testing.expectEqual(2, header.detail_tri_count);
        try testing.expectEqual(0, storedPoly(bytes, 2).vert_count);
    }
}

/// Fails unless the polygon at `index` in a tile's bytes has `expected` as
/// what lies beyond its edges.
fn expectNeighbours(expected: [6]u16, bytes: []const u8, index: usize) !void {
    const poly = storedPoly(bytes, index);
    try testing.expectEqualSlices(u16, &expected, &poly.neighbours);
}

test "createNavMeshData: every kind of neighbour entry" {
    const gpa = testing.allocator;
    // With the high bit set the low four bits decide: 0xf is a wall, 0 to 3
    // a side of the tile, and 4 to 14 are counted as a side of the tile but
    // stored as a wall. Without it the entry is a polygon, whatever its
    // index.
    const polys = [_]u16{
        0, 1, 2, 3, 4, 5, 0x8000, 0x8001, 0x8002, 0x8003, 0x8004, 0x8005,
        1, 2, 3, 4, 5, 6, 0x8006, 0x8007, 0x8008, 0x8009, 0x800a, 0x800b,
        2, 3, 4, 5, 6, 7, 0x800c, 0x800d, 0x800e, 0x800f, 0x7ffe, 0x7fff,
        3, 4, 5, 6, 7, 0, 0xfff0, 0xfff1, 0xfff2, 0xfff3, 0xffff, 0xabc4,
    };

    for ([_]bool{ false, true }) |with_tree| {
        errdefer std.debug.print("  tree {any}\n", .{with_tree});
        var params = handParams(&ring_verts, &polys, 6, hand_flags[0..4], hand_areas[0..4]);
        params.build_bv_tree = with_tree;

        const bytes = try expectBuilt(gpa, &params);
        defer gpa.free(bytes);
        // 24 edges, and two more links for each of the 20 entries counted as
        // a side of the tile.
        try testing.expectEqual(64, headerOf(bytes).link_count_max);
        try expectNeighbours(.{ 0x8004, 0x8002, 0x8000, 0x8006, 0, 0 }, bytes, 0);
        try expectNeighbours(.{ 0, 0, 0, 0, 0x7fff, 0x8000 }, bytes, 2);
        try expectNeighbours(.{ 0x8004, 0x8002, 0x8000, 0x8006, 0, 0 }, bytes, 3);
    }
}

/// The polygons of a tree's leaves in the order the tree holds them. `buffer`
/// has room for one per polygon.
fn leafOrder(bytes: []const u8, buffer: []i32) []i32 {
    const node_count: usize = @intCast(headerOf(bytes).bv_node_count);
    var leaf_count: usize = 0;
    // A tree over n polygons takes one node less than the 2n it has room for.
    for (0..node_count - 1) |i| {
        const node = storedBvNode(bytes, i);
        if (node.index < 0) continue;
        buffer[leaf_count] = node.index;
        leaf_count += 1;
    }
    return buffer[0..leaf_count];
}

test "createNavMeshData: polygons with the same box, and with no extent" {
    const gpa = testing.allocator;
    const polygon_count = hand_flags.len;
    var in_order: [polygon_count]i32 = undefined;
    for (&in_order, 0..) |*index, i| index.* = @intCast(i);

    // Every polygon over the same three vertices: each sort the tree makes is
    // of equal keys, which keep their order.
    const spread_verts = [_][3]u16{ .{ 2, 1, 2 }, .{ 2, 3, 30 }, .{ 30, 2, 2 } };
    // Every vertex in the same place.
    const same_verts = [_][3]u16{ .{ 7, 7, 7 }, .{ 7, 7, 7 }, .{ 7, 7, 7 } };
    var polys: [polygon_count * 6]u16 = undefined;
    for (0..polygon_count) |i| {
        polys[i * 6 ..][0..6].* = .{ 0, 1, 2, 0x800f, 0x800f, 0x800f };
    }

    // The same again as detail meshes that all name the same three vertices.
    const detail_meshes: [polygon_count][4]u32 = @splat(.{ 0, 3, 0, 1 });
    const spread_detail_verts = [_][3]f32{ .{ 0.6, 0.2, 0.6 }, .{ 0.6, 0.6, 9 }, .{ 9, 0.4, 0.6 } };
    const same_detail_verts: [3][3]f32 = @splat(.{ 2.1, 1.4, 2.1 });
    const detail_tris = [_][4]u8{.{ 0, 1, 2, 0x15 }};

    for ([_]bool{ false, true }) |same_place| {
        for ([_]bool{ false, true }) |with_detail| {
            errdefer std.debug.print("  same place {any}, detail {any}\n", .{
                same_place,
                with_detail,
            });
            var params = handParams(
                if (same_place) &same_verts else &spread_verts,
                &polys,
                3,
                &hand_flags,
                &hand_areas,
            );
            if (with_detail) giveDetail(
                &params,
                &detail_meshes,
                if (same_place) &same_detail_verts else &spread_detail_verts,
                &detail_tris,
            );

            const bytes = try expectBuilt(gpa, &params);
            defer gpa.free(bytes);
            var leaves: [polygon_count]i32 = undefined;
            try testing.expectEqualSlices(i32, &in_order, leafOrder(bytes, &leaves));

            // An area keeps its low six bits.
            for (hand_areas, 0..) |area, i| {
                try testing.expectEqual(area & 0x3f, storedPoly(bytes, i).area_and_type);
            }
        }
    }
}

test "createNavMeshData: a negative count of off-mesh connections is none" {
    const gpa = testing.allocator;
    var connections: Connections = .{};
    defer connections.deinit(gpa);
    try connections.add(gpa, .{ 12, 3, -18 }, .{ 12, 3, -18 });

    for ([_]bool{ false, true }) |with_detail| {
        errdefer std.debug.print("  detail {any}\n", .{with_detail});
        var params = if (with_detail) smallDetailedMesh() else smallMesh();
        connections.giveTo(&params);
        params.off_mesh_con_count = 0;
        const expected = try expectBuilt(gpa, &params);
        defer gpa.free(expected);
        try testing.expectEqual(0, headerOf(expected).off_mesh_connection_count);

        // The original is given no negative count; a port that builds what
        // it builds for none builds what it would.
        for ([_]i32{ -1, -3, std.math.minInt(i32) }) |count| {
            params.off_mesh_con_count = count;
            const bytes = try detour.createNavMeshData(gpa, &params);
            defer gpa.free(bytes);
            try support.expectBytes(expected, bytes);
        }

        // With the count the connection is there.
        params.off_mesh_con_count = connections.count();
        const header = try expectBuiltHeader(gpa, &params);
        try testing.expectEqual(1, header.off_mesh_connection_count);
    }
}

// ---------------------------------------------------------------------------
// Counts beyond what their fields hold
// ---------------------------------------------------------------------------

/// The detail mesh record at `index` in a tile's bytes, which follow the
/// room left for the links.
fn storedDetail(bytes: []const u8, index: usize) detour.nav_mesh.PolyDetail {
    const header = headerOf(bytes);
    const vert_count: usize = @intCast(header.vert_count);
    const poly_count: usize = @intCast(header.poly_count);
    const link_count: usize = @intCast(header.link_count_max);
    const size = @sizeOf(detour.nav_mesh.PolyDetail);
    const start = @sizeOf(MeshHeader) + vert_count * @sizeOf([3]f32) +
        poly_count * @sizeOf(detour.Poly) + link_count * @sizeOf(detour.nav_mesh.Link) +
        index * size;
    return std.mem.bytesToValue(detour.nav_mesh.PolyDetail, bytes[start..][0..size]);
}

test "createNavMeshData: as many vertices as a mesh may have" {
    const gpa = testing.allocator;
    // One vertex less than is refused. The off-mesh connections' polygons
    // take their vertices after the mesh's, at indices that run on past
    // 0xffff and keep their low 16 bits.
    const vert_count = 0xfffe;
    const verts = try gpa.alloc([3]u16, vert_count);
    defer gpa.free(verts);
    var prng: std.Random.DefaultPrng = .init(0xfffe);
    const random = prng.random();
    // All at the height of the tile's floor, where the connections start.
    for (verts) |*vert| {
        vert.* = .{ random.uintLessThan(u16, 100), 0, random.uintLessThan(u16, 100) };
    }
    const polys = [_]u16{ 0, 1, vert_count - 1, 0x800f, 0x800f, 0x800f };

    var connections: Connections = .{};
    defer connections.deinit(gpa);
    for (0..3) |i| {
        const x: f32 = @floatFromInt(5 + i * 5);
        try connections.add(gpa, .{ x, 0, 15 }, .{ x, 0, 25 });
    }

    for ([_]bool{ false, true }) |with_tree| {
        errdefer std.debug.print("  tree {any}\n", .{with_tree});
        var params = handParams(verts, &polys, 3, hand_flags[0..1], hand_areas[0..1]);
        params.build_bv_tree = with_tree;
        connections.giveTo(&params);

        const bytes = try expectBuilt(gpa, &params);
        defer gpa.free(bytes);
        const header = headerOf(bytes);
        try testing.expectEqual(3, header.off_mesh_connection_count);
        try testing.expectEqual(vert_count + 6, header.vert_count);

        const first = storedPoly(bytes, 1);
        const second = storedPoly(bytes, 2);
        try testing.expectEqual(0xfffe, first.verts[0]);
        try testing.expectEqual(0xffff, first.verts[1]);
        try testing.expectEqual(0, second.verts[0]);
        try testing.expectEqual(1, second.verts[1]);
    }
}

test "createNavMeshData: more polygons than an off-mesh connection can name" {
    const gpa = testing.allocator;
    // A connection names its polygon in 16 bits: after more than 0xffff
    // polygons the index keeps its low bits.
    const poly_count = 0x10001;
    const polys = try gpa.alloc(u16, poly_count * 6);
    defer gpa.free(polys);
    const flags = try gpa.alloc(u16, poly_count);
    defer gpa.free(flags);
    const areas = try gpa.alloc(u8, poly_count);
    defer gpa.free(areas);

    // Triangles over eight vertices: most share their box with many others,
    // so the tree's sorts are full of equal keys.
    var prng: std.Random.DefaultPrng = .init(0x10001);
    const random = prng.random();
    for (0..poly_count) |i| {
        const p = polys[i * 6 ..][0..6];
        for (p[0..3]) |*vertex| vertex.* = random.uintLessThan(u16, ring_verts.len);
        for (p[3..6]) |*neighbour| neighbour.* = 0x800f;
        flags[i] = random.int(u16);
        areas[i] = random.int(u8);
    }

    var connections: Connections = .{};
    defer connections.deinit(gpa);
    try connections.add(gpa, .{ 15, 0.5, 15 }, .{ 15, 0.5, 20 });

    var params = handParams(&ring_verts, polys, 3, flags, areas);
    connections.giveTo(&params);

    const bytes = try expectBuilt(gpa, &params);
    defer gpa.free(bytes);
    const header = headerOf(bytes);
    try testing.expectEqual(poly_count + 1, header.poly_count);
    try testing.expectEqual(poly_count, header.off_mesh_base);
    try testing.expectEqual(poly_count * 2, header.bv_node_count);
    try testing.expectEqual(1, header.off_mesh_connection_count);
    try testing.expectEqual(1, storedConnection(bytes, 0).poly);
}

test "createNavMeshData: more detail vertices than their counters hold" {
    const gpa = testing.allocator;
    const poly_count = 300;
    var polys: [poly_count * 6]u16 = undefined;
    var flags: [poly_count]u16 = undefined;
    var areas: [poly_count]u8 = undefined;
    var detail_meshes: [poly_count][4]u32 = undefined;
    var detail_verts: [600][3]f32 = undefined;
    var detail_tris: [16][4]u8 = undefined;

    var prng: std.Random.DefaultPrng = .init(300);
    const random = prng.random();
    for (&detail_verts) |*vert| {
        vert.* = .{ random.float(f32) * 30, random.float(f32) * 10, random.float(f32) * 30 };
    }
    random.bytes(std.mem.asBytes(&detail_tris));
    for (0..poly_count) |i| {
        const p = polys[i * 6 ..][0..6];
        for (p[0..3]) |*vertex| vertex.* = random.uintLessThan(u16, ring_verts.len);
        for (p[3..6]) |*neighbour| neighbour.* = 0x800f;
        flags[i] = random.int(u16);
        areas[i] = random.uintLessThan(u8, 64);
        // The polygon's three vertices and 250 more, from the detail vertex
        // of the polygon's index on. The triangles a detail mesh names are
        // not looked at.
        detail_meshes[i] = .{ @intCast(i), 3 + 250, random.int(u32), random.int(u8) };
    }
    // A detail mesh counts its own vertices and its triangles in eight bits.
    detail_meshes[123][1] = 3 + 300;
    detail_meshes[45][3] = 300;

    var params = handParams(&ring_verts, &polys, 3, &flags, &areas);
    giveDetail(&params, &detail_meshes, &detail_verts, &detail_tris);

    const bytes = try expectBuilt(gpa, &params);
    defer gpa.free(bytes);
    try testing.expectEqual(299 * 250 + 300, headerOf(bytes).detail_vert_count);
    try testing.expectEqual(300 - 256, storedDetail(bytes, 123).vert_count);
    try testing.expectEqual(300 - 256, storedDetail(bytes, 45).tri_count);

    // Where a detail mesh's own vertices start is counted in 16 bits while
    // the vertices are stored, so past 0xffff they are stored over the first
    // ones. 261 detail meshes of 250 and one of 300 come before this one.
    try testing.expectEqual(261 * 250 + 300 - 0x10000, storedDetail(bytes, 262).vert_base);
    try testing.expectEqual(260 * 250 + 300, storedDetail(bytes, 261).vert_base);
}

// ---------------------------------------------------------------------------
// Random meshes
// ---------------------------------------------------------------------------

/// What the random meshes reached, to show they reach each part of a tile.
const Reach = struct {
    stored: usize = 0,
    dropped: usize = 0,
    with_tree: usize = 0,
    with_detail: usize = 0,
    flat: usize = 0,
    own_detail_verts: usize = 0,
};

/// A random float from `low` up to `high`.
fn randomBetween(random: std.Random, low: f32, high: f32) f32 {
    return low + (high - low) * random.float(f32);
}

/// Builds one random mesh on both sides. Nothing in it is a mesh Recast
/// would make, only one the builder takes: indices and counts fit their
/// arrays, and every number is one the original converts in a defined way.
/// `arena` holds the input.
fn buildRandomMesh(
    gpa: std.mem.Allocator,
    arena: std.mem.Allocator,
    random: std.Random,
    reach: *Reach,
) !void {
    const cell_choices = [_]f32{ 0.3, 0.2, 1.0, 0.05 };
    const cell_size = cell_choices[random.uintLessThan(usize, cell_choices.len)];
    const cell_height = cell_choices[random.uintLessThan(usize, cell_choices.len)];

    // A coarse mesh has few places for a vertex, so polygons share boxes and
    // the tree's sorts meet equal keys.
    const coarse = random.boolean();
    const span_xz: u16 = if (coarse) 3 else 1000;
    const span_y: u16 = if (coarse) 2 else 200;
    // Entries after the counted ones of each array, which a build leaves
    // alone: the counts say how much is used, not the slices' lengths.
    const slack: usize = if (random.boolean()) random.intRangeAtMost(usize, 1, 3) else 0;

    const bounds_min: [3]f32 = .{
        randomBetween(random, -500, 500),
        randomBetween(random, -50, 50),
        randomBetween(random, -500, 500),
    };
    const size: [3]f32 = .{
        @as(f32, @floatFromInt(span_xz)) * cell_size,
        @as(f32, @floatFromInt(span_y)) * cell_height,
        @as(f32, @floatFromInt(span_xz)) * cell_size,
    };
    const bounds_max: [3]f32 = .{
        bounds_min[0] + size[0],
        bounds_min[1] + size[1],
        bounds_min[2] + size[2],
    };

    const vert_count = random.intRangeAtMost(usize, 3, 40);
    const vert_limit: u16 = @intCast(vert_count);
    const verts = try arena.alloc([3]u16, vert_count + slack);
    @memset(verts, .{ 0xffff, 0xffff, 0xffff });
    for (verts[0..vert_count]) |*vert| {
        vert.* = .{
            random.uintLessThan(u16, span_xz),
            random.uintLessThan(u16, span_y),
            random.uintLessThan(u16, span_xz),
        };
    }

    const vertices_per_polygon = random.intRangeAtMost(usize, 3, 6);
    const poly_count = random.intRangeAtMost(usize, 1, 30);
    const poly_limit: u16 = @intCast(poly_count);
    const stride = vertices_per_polygon * 2;
    const polys = try arena.alloc(u16, (poly_count + slack) * stride);
    @memset(polys, none);
    const poly_vert_counts = try arena.alloc(usize, poly_count);
    for (poly_vert_counts, 0..) |*poly_vert_count, i| {
        const p = polys[i * stride ..][0..stride];
        const nv = random.intRangeAtMost(usize, 3, vertices_per_polygon);
        poly_vert_count.* = nv;
        for (0..nv) |j| {
            p[j] = random.uintLessThan(u16, vert_limit);
            p[vertices_per_polygon + j] = switch (random.uintLessThan(u8, 4)) {
                0 => none,
                1 => random.uintLessThan(u16, poly_limit),
                2 => 0x8000 | random.uintLessThan(u16, 4),
                else => 0x800f,
            };
        }
    }
    const poly_flags = try arena.alloc(u16, poly_count + slack);
    random.bytes(std.mem.sliceAsBytes(poly_flags));
    const poly_areas = try arena.alloc(u8, poly_count + slack);
    random.bytes(poly_areas);

    var detail_meshes: ?[]const [4]u32 = null;
    var detail_verts: []const [3]f32 = &.{};
    var detail_verts_count: i32 = 0;
    var detail_tris: []const [4]u8 = &.{};
    var detail_tri_count: i32 = 0;
    if (random.boolean()) {
        var mesh_list: std.ArrayList([4]u32) = .empty;
        var vert_list: std.ArrayList([3]f32) = .empty;
        var tri_list: std.ArrayList([4]u8) = .empty;
        for (poly_vert_counts, 0..) |nv, i| {
            const own = random.uintLessThan(usize, 4);
            const tri_count = random.intRangeAtMost(usize, 1, 6);
            try mesh_list.append(arena, .{
                @intCast(vert_list.items.len),
                @intCast(nv + own),
                @intCast(tri_list.items.len),
                @intCast(tri_count),
            });
            // The polygon's vertices come first, in world units.
            for (polys[i * stride ..][0..nv]) |vertex| {
                const vert = verts[vertex];
                try vert_list.append(arena, .{
                    bounds_min[0] + @as(f32, @floatFromInt(vert[0])) * cell_size,
                    bounds_min[1] + @as(f32, @floatFromInt(vert[1])) * cell_height,
                    bounds_min[2] + @as(f32, @floatFromInt(vert[2])) * cell_size,
                });
            }
            // Then its own, now and then far outside the tile, where a box
            // coordinate is held to the range it has.
            for (0..own) |_| {
                const margin: f32 = if (random.uintLessThan(u8, 16) == 0) 1e6 else 0;
                try vert_list.append(arena, .{
                    randomBetween(random, bounds_min[0] - margin, bounds_max[0] + margin),
                    randomBetween(random, bounds_min[1] - margin, bounds_max[1] + margin),
                    randomBetween(random, bounds_min[2] - margin, bounds_max[2] + margin),
                });
            }
            const index_limit: u8 = @intCast(nv + own);
            for (0..tri_count) |_| {
                try tri_list.append(arena, .{
                    random.uintLessThan(u8, index_limit),
                    random.uintLessThan(u8, index_limit),
                    random.uintLessThan(u8, index_limit),
                    random.int(u8),
                });
            }
        }
        const total_verts: i32 = @intCast(vert_list.items.len);
        detail_tri_count = @intCast(tri_list.items.len);
        for (0..slack) |_| {
            try mesh_list.append(arena, @splat(0xffff_ffff));
            try vert_list.append(arena, @splat(1e30));
            try tri_list.append(arena, @splat(0xff));
        }
        detail_meshes = mesh_list.items;
        detail_verts = vert_list.items;
        detail_tris = tri_list.items;
        // The count of detail vertices only says how many of them bound the
        // heights of the connections' starts.
        detail_verts_count = switch (random.uintLessThan(u8, 6)) {
            0 => 0,
            1 => random.intRangeAtMost(i32, 1, total_verts),
            else => total_verts,
        };
    } else if (random.uintLessThan(u8, 4) == 0) {
        // No detail mesh, but detail vertices to take those heights from.
        const heights = try arena.alloc([3]f32, random.intRangeAtMost(usize, 1, 8));
        for (heights) |*vert| {
            const height = randomBetween(random, bounds_min[1] - size[1], bounds_max[1] + size[1]);
            vert.* = .{ 0, height, 0 };
        }
        detail_verts = heights;
        detail_verts_count = @intCast(heights.len);
    }

    // Connections with their ends in and around the tile.
    const connection_count = random.uintLessThan(usize, 5);
    const connection_verts = try arena.alloc([2][3]f32, connection_count + slack);
    for (connection_verts) |*ends| {
        for (ends) |*end| {
            end.* = .{
                bounds_min[0] + size[0] * randomBetween(random, -0.25, 1.25),
                bounds_min[1] + size[1] * randomBetween(random, -0.5, 1.5),
                bounds_min[2] + size[2] * randomBetween(random, -0.25, 1.25),
            };
        }
    }
    const connection_radii = try arena.alloc(f32, connection_count + slack);
    for (connection_radii) |*radius| radius.* = randomBetween(random, 0.1, 2);
    const connection_flags = try arena.alloc(u16, connection_count + slack);
    random.bytes(std.mem.sliceAsBytes(connection_flags));
    const connection_areas = try arena.alloc(u8, connection_count + slack);
    random.bytes(connection_areas);
    const connection_directions = try arena.alloc(u8, connection_count + slack);
    for (connection_directions) |*direction| direction.* = random.uintLessThan(u8, 3);
    const connection_user_ids = try arena.alloc(u32, connection_count + slack);
    random.bytes(std.mem.sliceAsBytes(connection_user_ids));

    const params: NavMeshCreateParams = .{
        .verts = verts,
        .vert_count = @intCast(vert_count),
        .polys = polys,
        .poly_flags = poly_flags,
        .poly_areas = poly_areas,
        .poly_count = @intCast(poly_count),
        .vertices_per_polygon = @intCast(vertices_per_polygon),
        .detail_meshes = detail_meshes,
        .detail_verts = detail_verts,
        .detail_verts_count = detail_verts_count,
        .detail_tris = detail_tris,
        .detail_tri_count = detail_tri_count,
        .off_mesh_con_verts = connection_verts,
        .off_mesh_con_rad = connection_radii,
        .off_mesh_con_flags = connection_flags,
        .off_mesh_con_areas = connection_areas,
        .off_mesh_con_dir = connection_directions,
        .off_mesh_con_user_id = if (random.boolean()) connection_user_ids else null,
        .off_mesh_con_count = @intCast(connection_count),
        .user_id = random.int(u32),
        .tile_x = random.int(i32),
        .tile_y = random.int(i32),
        .tile_layer = random.int(i32),
        .bounds_min = bounds_min,
        .bounds_max = bounds_max,
        .walkable_height = randomBetween(random, 0.5, 3),
        .walkable_radius = randomBetween(random, 0, 2),
        .walkable_climb = randomBetween(random, 0, 2),
        .cell_size = cell_size,
        .cell_height = cell_height,
        .build_bv_tree = random.boolean(),
    };

    const bytes = try expectBuilt(gpa, &params);
    defer gpa.free(bytes);
    const header = headerOf(bytes);
    const stored: usize = @intCast(header.off_mesh_connection_count);
    reach.stored += stored;
    reach.dropped += connection_count - stored;
    reach.own_detail_verts += @intCast(header.detail_vert_count);
    if (params.build_bv_tree) reach.with_tree += 1;
    if (params.detail_meshes != null) reach.with_detail += 1 else reach.flat += 1;
}

test "createNavMeshData: random meshes" {
    const gpa = testing.allocator;
    const seed: u64 = 0x6e61_766d_6573_6801;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var arena_state: std.heap.ArenaAllocator = .init(gpa);
    defer arena_state.deinit();

    var reach: Reach = .{};
    for (0..3000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        _ = arena_state.reset(.retain_capacity);
        try buildRandomMesh(gpa, arena_state.allocator(), random, &reach);
    }

    try testing.expect(reach.stored > 0);
    try testing.expect(reach.dropped > 0);
    try testing.expect(reach.with_tree > 0);
    try testing.expect(reach.with_detail > 0);
    try testing.expect(reach.flat > 0);
    try testing.expect(reach.own_detail_verts > 0);
}

// ---------------------------------------------------------------------------
// Numbers a tree's box cannot hold
// ---------------------------------------------------------------------------

test "createNavMeshData: tree boxes from numbers beyond their range" {
    // The original makes these box coordinates with casts C leaves
    // undefined, so its answer is whatever its compiled code does. The port
    // gives what x86-64 does, and only there do the two have to agree.
    if (builtin.cpu.arch != .x86_64) return error.SkipZigTest;
    const gpa = testing.allocator;

    const verts = [_][3]u16{ .{ 6, 5, 6 }, .{ 12, 5, 6 }, .{ 12, 10, 12 } };
    const polys = [_]u16{ 0, 1, 2, 0x800f, 0x800f, 0x800f };
    const detail_tris = [_][4]u8{.{ 0, 1, 2, 0x15 }};

    // With a detail mesh a polygon's box is the box of its detail vertices,
    // in tree units from the tile's low corner: here twice the world units,
    // so every number below is exact.
    const plain_detail_meshes = [_][4]u32{.{ 0, 3, 0, 1 }};
    const plain_detail_verts = [_][3]f32{ .{ 3, 1, 3 }, .{ 6, 1, 3 }, .{ 6, 2, 6 } };
    {
        var params = handParams(&verts, &polys, 3, hand_flags[0..1], hand_areas[0..1]);
        params.cell_size = 0.5;
        giveDetail(&params, &plain_detail_meshes, &plain_detail_verts, &detail_tris);
        const bytes = try expectBuilt(gpa, &params);
        defer gpa.free(bytes);
        const leaf = storedBvNode(bytes, 0);
        try testing.expectEqual([3]u16{ 6, 2, 6 }, leaf.bounds_min);
        try testing.expectEqual([3]u16{ 12, 4, 12 }, leaf.bounds_max);
    }

    // One more detail vertex that no box coordinate can stand for: not a
    // number, minus infinity, and an offset below what 32 bits hold. Each
    // comes out as 0. It is the polygon's last detail vertex, because the
    // original's minimum and maximum only keep what is not a number when it
    // comes last; kept, it is both ends of the box.
    //
    // Plus infinity and offsets above what 32 bits hold are left out: the
    // original's compiler may as well hold them to 0xffff as make them 0.
    const Beyond = struct {
        value: f32,
        bounds_max: [3]u16,
    };
    const beyond = [_]Beyond{
        .{ .value = std.math.nan(f32), .bounds_max = .{ 0, 0, 0 } },
        .{ .value = -std.math.inf(f32), .bounds_max = .{ 12, 4, 12 } },
        .{ .value = -1e10, .bounds_max = .{ 12, 4, 12 } },
    };
    const detail_meshes = [_][4]u32{.{ 0, 4, 0, 1 }};
    for (beyond) |case| {
        errdefer std.debug.print("  a detail vertex of {d}\n", .{case.value});
        const detail_verts = [_][3]f32{
            plain_detail_verts[0],
            plain_detail_verts[1],
            plain_detail_verts[2],
            @splat(case.value),
        };
        var params = handParams(&verts, &polys, 3, hand_flags[0..1], hand_areas[0..1]);
        params.cell_size = 0.5;
        giveDetail(&params, &detail_meshes, &detail_verts, &detail_tris);

        const bytes = try expectBuilt(gpa, &params);
        defer gpa.free(bytes);
        try testing.expectEqual(1, headerOf(bytes).detail_vert_count);
        const leaf = storedBvNode(bytes, 0);
        try testing.expectEqual([3]u16{ 0, 0, 0 }, leaf.bounds_min);
        try testing.expectEqual(case.bounds_max, leaf.bounds_max);
    }

    // Without a detail mesh a box's heights are the vertices' heights, which
    // count cell heights, recounted in cell sizes: here doubled. A height
    // that then no longer fits 16 bits keeps its low 16.
    const high_verts = [_][3]u16{ .{ 1, 40000, 1 }, .{ 9, 50000, 2 }, .{ 4, 60000, 8 } };
    var high = handParams(&high_verts, &polys, 3, hand_flags[0..1], hand_areas[0..1]);
    high.cell_size = 0.5;
    high.cell_height = 1;
    const bytes = try expectBuilt(gpa, &high);
    defer gpa.free(bytes);
    const leaf = storedBvNode(bytes, 0);
    try testing.expectEqual([3]u16{ 1, 80000 - 0x10000, 1 }, leaf.bounds_min);
    try testing.expectEqual([3]u16{ 9, 120000 - 0x10000, 8 }, leaf.bounds_max);
}

// ---------------------------------------------------------------------------
// Out of memory
// ---------------------------------------------------------------------------

/// Three off-mesh connections for the hand-made mesh, two that start inside
/// its tile and one that starts beyond it. Free with `deinit`.
fn smallConnections(gpa: std.mem.Allocator) !Connections {
    var connections: Connections = .{};
    errdefer connections.deinit(gpa);
    try connections.add(gpa, .{ 12, 3, -18 }, .{ 20, 3, -18 });
    try connections.add(gpa, .{ 20, 3, -18 }, .{ 12, 3, -18 });
    try connections.add(gpa, .{ 11, 2.5, -19 }, .{ 13, 3, -17 });
    return connections;
}

/// Builds `params` with the port alone, failing its first allocation, then
/// its second, and so on until a build gets through. Fails unless every
/// build before that reports it ran out of memory and has freed all it
/// allocated, and the one that gets through gives the original's bytes.
/// Returns how many allocations a build makes.
fn expectEachAllocationFailing(
    gpa: std.mem.Allocator,
    params: *const NavMeshCreateParams,
) !usize {
    const expected = try expectBuilt(gpa, params);
    defer gpa.free(expected);

    // Far more than a build allocates: a build that never gets through has
    // stopped counting its allocations.
    const fail_index_limit = 64;
    for (0..fail_index_limit) |fail_index| {
        errdefer std.debug.print("  failing allocation {d}\n", .{fail_index});
        var failing: testing.FailingAllocator = .init(gpa, .{ .fail_index = fail_index });
        const failing_gpa = failing.allocator();
        if (detour.createNavMeshData(failing_gpa, params)) |bytes| {
            defer failing_gpa.free(bytes);
            try testing.expect(!failing.has_induced_failure);
            try support.expectBytes(expected, bytes);
            return fail_index;
        } else |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            try testing.expect(failing.has_induced_failure);
            try testing.expectEqual(failing.allocated_bytes, failing.freed_bytes);
        }
    }
    return error.TestUnexpectedResult;
}

test "createNavMeshData: each allocation failing" {
    const gpa = testing.allocator;
    var connections = try smallConnections(gpa);
    defer connections.deinit(gpa);

    // A build allocates the tile's bytes, and before them the sides of the
    // connections' ends when it is given connections, and after them the
    // boxes it sorts when it makes a tree.
    var params = smallDetailedMesh();
    connections.giveTo(&params);
    try testing.expectEqual(3, try expectEachAllocationFailing(gpa, &params));

    params.build_bv_tree = false;
    try testing.expectEqual(2, try expectEachAllocationFailing(gpa, &params));

    params = smallDetailedMesh();
    try testing.expectEqual(2, try expectEachAllocationFailing(gpa, &params));

    params.build_bv_tree = false;
    try testing.expectEqual(1, try expectEachAllocationFailing(gpa, &params));

    // The same without a detail mesh.
    params = smallMesh();
    connections.giveTo(&params);
    try testing.expectEqual(3, try expectEachAllocationFailing(gpa, &params));

    params = smallMesh();
    params.build_bv_tree = false;
    try testing.expectEqual(1, try expectEachAllocationFailing(gpa, &params));
}

// ---------------------------------------------------------------------------
// Byte order
// ---------------------------------------------------------------------------

/// One of the two byte-order swaps.
const Swap = enum { header, data };

/// Makes `swap` on both sides, the original on `expected` and the port on
/// `actual`, which hold the same bytes. Fails unless both give the same
/// answer and leave the same bytes. Returns the answer.
///
/// The original trusts the header's counts and reads no further than the
/// magic and version of a header that is not a tile's, so `expected` must be
/// a whole tile or have such a header.
fn swapBoth(swap: Swap, expected: []align(4) u8, actual: []align(4) u8) !bool {
    const size: i32 = @intCast(expected.len);
    const expected_swapped = switch (swap) {
        .header => reference.ref_nav_mesh_header_swap_endian(expected.ptr, size),
        .data => reference.ref_nav_mesh_data_swap_endian(expected.ptr, size),
    };
    const swapped = switch (swap) {
        .header => builder.navMeshHeaderSwapEndian(actual),
        .data => builder.navMeshDataSwapEndian(actual),
    };
    try testing.expectEqual(expected_swapped, swapped);
    try support.expectBytes(expected, actual);
    return swapped;
}

/// Fails unless `bytes` equal `before`, a copy made of them earlier.
fn expectUnchanged(before: []const u8, bytes: []const u8) !void {
    if (std.mem.eql(u8, before, bytes)) return;
    const index = std.mem.findDiff(u8, before, bytes).?;
    std.debug.print("the bytes changed, first at {d} of {d}\n", .{ index, bytes.len });
    return error.TestExpectedEqual;
}

/// Writes `header` over the header a tile's bytes start with.
fn setHeader(bytes: []u8, header: MeshHeader) void {
    @memcpy(bytes[0..@sizeOf(MeshHeader)], std.mem.asBytes(&header));
}

/// A change that leaves a tile's header the header of no tile, in either
/// byte order.
const HeaderDamage = enum {
    magic,
    version,
    /// The magic of the other byte order beside the version of this one.
    magic_swapped,
    /// The version of the other byte order beside the magic of this one.
    version_swapped,
};

/// Takes a tile's bytes through the swaps on both sides: to the other byte
/// order and back, each swap twice over, and each swap on a header that is
/// not a tile's. Fails unless the sides answer alike and leave the same
/// bytes each time, and the tile comes back as it was.
fn expectSwapsAlike(gpa: std.mem.Allocator, tile: []const u8) !void {
    const header_size = @sizeOf(MeshHeader);
    const expected = try gpa.alignedAlloc(u8, .@"4", tile.len);
    defer gpa.free(expected);
    const actual = try gpa.alignedAlloc(u8, .@"4", tile.len);
    defer gpa.free(actual);
    const before = try gpa.alloc(u8, tile.len);
    defer gpa.free(before);

    // To the other byte order: the data while the header still says where
    // it is, then the header.
    @memcpy(expected, tile);
    @memcpy(actual, tile);
    try testing.expect(try swapBoth(.data, expected, actual));
    try expectUnchanged(tile[0..header_size], actual[0..header_size]);
    try testing.expect(!std.mem.eql(u8, tile[header_size..], actual[header_size..]));
    try testing.expect(try swapBoth(.header, expected, actual));
    try testing.expect(!std.mem.eql(u8, tile[0..header_size], actual[0..header_size]));

    // In the other byte order the header's counts cannot be read, so the
    // data's swap leaves the tile alone.
    @memcpy(before, actual);
    try testing.expectEqual(false, try swapBoth(.data, expected, actual));
    try expectUnchanged(before, actual);

    // And back: the header first.
    try testing.expect(try swapBoth(.header, expected, actual));
    try testing.expect(try swapBoth(.data, expected, actual));
    try expectUnchanged(tile, actual);

    // The header alone, there and back. In between the tile is not one the
    // data's swap reads.
    try testing.expect(try swapBoth(.header, expected, actual));
    try expectUnchanged(tile[header_size..], actual[header_size..]);
    @memcpy(before, actual);
    try testing.expectEqual(false, try swapBoth(.data, expected, actual));
    try expectUnchanged(before, actual);
    try testing.expect(try swapBoth(.header, expected, actual));
    try expectUnchanged(tile, actual);

    // The data alone, there and back.
    try testing.expect(try swapBoth(.data, expected, actual));
    try testing.expect(try swapBoth(.data, expected, actual));
    try expectUnchanged(tile, actual);

    // A header that is not a tile's: both swaps leave the bytes alone.
    const damages = comptime std.enums.values(HeaderDamage);
    for (damages) |damage| {
        errdefer std.debug.print("  header damage {s}\n", .{@tagName(damage)});
        var header = headerOf(tile);
        switch (damage) {
            .magic => header.magic +%= 1,
            .version => header.version +%= 1,
            .magic_swapped => header.magic = @byteSwap(header.magic),
            .version_swapped => header.version = @byteSwap(header.version),
        }
        setHeader(expected, header);
        setHeader(actual, header);
        @memcpy(before, actual);
        for ([_]Swap{ .header, .data }) |swap| {
            try testing.expectEqual(false, try swapBoth(swap, expected, actual));
            try expectUnchanged(before, actual);
        }
    }
}

test "byte-order swaps: every tile of two levels, there and back" {
    const gpa = testing.allocator;
    var tile_count: usize = 0;
    var with_connections: usize = 0;
    var with_detail_verts: usize = 0;
    var with_tree: usize = 0;

    // One level as a single large tile, one cut into tiles.
    for (level_cases[0..2]) |case| {
        var level: support.Level = try .init(gpa, case.obj_text, case.config, level_options);
        defer level.deinit();

        var y: i32 = 0;
        while (y < level.grid.height) : (y += 1) {
            var x: i32 = 0;
            while (x < level.grid.width) : (x += 1) {
                errdefer std.debug.print("  level \"{s}\", tile {d}, {d}\n", .{ case.name, x, y });
                var input = (try level.tileInput(x, y)) orelse continue;
                defer input.deinit();

                const bytes = try expectBuiltFromReference(gpa, &input.reference_params);
                defer gpa.free(bytes);
                try expectSwapsAlike(gpa, bytes);

                const header = headerOf(bytes);
                tile_count += 1;
                if (header.off_mesh_connection_count > 0) with_connections += 1;
                if (header.detail_vert_count > 0) with_detail_verts += 1;
                if (header.bv_node_count > 0) with_tree += 1;
            }
        }
    }

    // The tiles have something in each block a swap changes.
    try testing.expect(tile_count > 2);
    try testing.expect(with_connections > 0);
    try testing.expect(with_detail_verts > 0);
    try testing.expect(with_tree > 0);
}

test "byte-order swaps: hand-made tiles with and without each block" {
    const gpa = testing.allocator;
    var connections = try smallConnections(gpa);
    defer connections.deinit(gpa);

    // Without a detail mesh a tile has no detail vertices, without a tree no
    // nodes, and without connections none of those.
    for ([_]bool{ false, true }) |with_detail| {
        for ([_]bool{ false, true }) |with_tree| {
            for ([_]bool{ false, true }) |with_connections| {
                errdefer std.debug.print("  detail {any}, tree {any}, connections {any}\n", .{
                    with_detail,
                    with_tree,
                    with_connections,
                });
                var params = if (with_detail) smallDetailedMesh() else smallMesh();
                params.build_bv_tree = with_tree;
                if (with_connections) connections.giveTo(&params);

                const bytes = try expectBuilt(gpa, &params);
                defer gpa.free(bytes);
                const header = headerOf(bytes);
                try testing.expectEqual(with_detail, header.detail_vert_count > 0);
                try testing.expectEqual(with_tree, header.bv_node_count > 0);
                try testing.expectEqual(with_connections, header.off_mesh_connection_count > 0);
                try expectSwapsAlike(gpa, bytes);
            }
        }
    }
}

// The original takes its swaps' bytes on trust: it never looks at their
// length, and reads wherever the header's counts point. What the port does
// with bytes that are too few, or whose counts ask for more than there is,
// is therefore checked on the port alone.

/// The hand-made mesh's tile with something in every block: detail
/// vertices, a tree and off-mesh connections. Both sides build it alike.
/// The caller frees it with `gpa`.
fn fullSmallTile(gpa: std.mem.Allocator) ![]align(4) u8 {
    var connections = try smallConnections(gpa);
    defer connections.deinit(gpa);
    var params = smallDetailedMesh();
    connections.giveTo(&params);
    return expectBuilt(gpa, &params);
}

/// Gives the port's swaps the first `len` bytes of `copy`, a copy of `tile`
/// that is cut short by it. Fails unless the data's swap refuses them, the
/// header's swap takes them just when they hold a whole header, and `copy`
/// ends as it started.
fn expectShortRefused(tile: []const u8, copy: []u8, len: usize) !void {
    const header_size = @sizeOf(MeshHeader);
    const short = copy[0..len];

    // The data's swap needs every block the header counts.
    try testing.expect(!builder.navMeshDataSwapEndian(short));
    try expectUnchanged(tile[0..header_size], copy[0..header_size]);

    // The header's swap needs the header alone.
    const holds_header = len >= header_size;
    try testing.expectEqual(holds_header, builder.navMeshHeaderSwapEndian(short));
    if (holds_header) {
        try testing.expect(!std.mem.eql(u8, tile[0..header_size], copy[0..header_size]));
        try testing.expect(!builder.navMeshDataSwapEndian(short));
        try testing.expect(builder.navMeshHeaderSwapEndian(short));
    }
    try expectUnchanged(tile, copy);
}

test "byte-order swaps: fewer bytes than the tile has" {
    const gpa = testing.allocator;

    // Every length short of a small tile's.
    const small = try fullSmallTile(gpa);
    defer gpa.free(small);
    const small_copy = try gpa.dupe(u8, small);
    defer gpa.free(small_copy);
    try testing.expect(small.len > @sizeOf(MeshHeader));
    for (0..small.len) |len| {
        errdefer std.debug.print("  the first {d} of {d} bytes\n", .{ len, small.len });
        try expectShortRefused(small, small_copy, len);
    }

    // Random lengths short of a level's.
    var level: support.Level = try .init(
        gpa,
        support.dungeon_obj,
        support.solo_config,
        level_options,
    );
    defer level.deinit();
    var input = (try level.tileInput(0, 0)) orelse return error.TestUnexpectedResult;
    defer input.deinit();
    const large = try expectBuiltFromReference(gpa, &input.reference_params);
    defer gpa.free(large);
    const large_copy = try gpa.dupe(u8, large);
    defer gpa.free(large_copy);

    const seed: u64 = 0x73686f7274;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();
    for (0..1000) |case| {
        // Now and then one of the last few, where only the last block is
        // cut short.
        const len = if (case % 8 == 0)
            large.len - 1 - random.uintLessThan(usize, 64)
        else
            random.uintLessThan(usize, large.len);
        errdefer std.debug.print("  seed {d}, case {d}: the first {d} of {d} bytes\n", .{
            seed,
            case,
            len,
            large.len,
        });
        try expectShortRefused(large, large_copy, len);
    }
}

/// The header's counts, each of which says how long a block of a tile is.
const count_fields = [_][]const u8{
    "poly_count",
    "vert_count",
    "link_count_max",
    "detail_mesh_count",
    "detail_vert_count",
    "detail_tri_count",
    "bv_node_count",
    "off_mesh_connection_count",
};

test "byte-order swaps: counts the bytes cannot hold" {
    const gpa = testing.allocator;
    const header_size = @sizeOf(MeshHeader);
    const tile = try fullSmallTile(gpa);
    defer gpa.free(tile);
    const copy = try gpa.dupe(u8, tile);
    defer gpa.free(copy);
    const original = headerOf(tile);

    inline for (count_fields) |field| {
        const count: i32 = @field(original, field);
        try testing.expect(count > 0);
        // Negative, one more than the tile has, and the most a count can
        // say. The last is left out where a tile's size is 32 bits wide:
        // there the blocks' sizes add up to more than a size can hold.
        const values = [_]i32{ -1, std.math.minInt(i32), count + 1, std.math.maxInt(i32) };
        const value_count: usize = if (@sizeOf(usize) >= 8) values.len else values.len - 1;
        for (values[0..value_count]) |value| {
            errdefer std.debug.print("  a {s} of {d}\n", .{ field, value });
            var header = original;
            @field(header, field) = value;
            setHeader(copy, header);

            try testing.expect(!builder.navMeshDataSwapEndian(copy));
            try expectUnchanged(std.mem.asBytes(&header), copy[0..header_size]);
            try expectUnchanged(tile[header_size..], copy[header_size..]);

            // The header's swap reads no count: it swaps them all, and back.
            try testing.expect(builder.navMeshHeaderSwapEndian(copy));
            try testing.expect(!std.mem.eql(u8, std.mem.asBytes(&header), copy[0..header_size]));
            try testing.expect(builder.navMeshHeaderSwapEndian(copy));
            try expectUnchanged(std.mem.asBytes(&header), copy[0..header_size]);
            try expectUnchanged(tile[header_size..], copy[header_size..]);
        }
    }
}

test "byte-order swaps: a tile at any address, with bytes around it" {
    const gpa = testing.allocator;
    const tile = try fullSmallTile(gpa);
    defer gpa.free(tile);

    // What the swaps make of the tile where it was built, which the original
    // makes of it too.
    const expected = try gpa.alignedAlloc(u8, .@"4", tile.len);
    defer gpa.free(expected);
    @memcpy(expected, tile);
    const swapped = try gpa.alignedAlloc(u8, .@"4", tile.len);
    defer gpa.free(swapped);
    @memcpy(swapped, tile);
    try testing.expect(try swapBoth(.data, expected, swapped));
    try testing.expect(try swapBoth(.header, expected, swapped));

    // The same tile one, two and three bytes off a multiple of four, with
    // bytes after it that belong to something else.
    const filler = 0xa5;
    const after = 7;
    const buffer = try gpa.alignedAlloc(u8, .@"4", 3 + tile.len + after);
    defer gpa.free(buffer);
    for (1..4) |before| {
        errdefer std.debug.print("  {d} bytes before the tile\n", .{before});
        @memset(buffer, filler);
        const moved = buffer[before..][0 .. tile.len + after];
        @memcpy(moved[0..tile.len], tile);

        try testing.expect(builder.navMeshDataSwapEndian(moved));
        try testing.expect(builder.navMeshHeaderSwapEndian(moved));
        try expectUnchanged(swapped, moved[0..tile.len]);
        for (buffer[0..before]) |byte| try testing.expectEqual(filler, byte);
        for (moved[tile.len..]) |byte| try testing.expectEqual(filler, byte);

        // And back to this machine's byte order.
        try testing.expect(builder.navMeshHeaderSwapEndian(moved));
        try testing.expect(builder.navMeshDataSwapEndian(moved));
        try expectUnchanged(tile, moved[0..tile.len]);
        for (moved[tile.len..]) |byte| try testing.expectEqual(filler, byte);
    }
}

test "createNavMeshData: fans that would run past the tile are refused" {
    // Two polygons of one vertex take away the room of the square's two
    // triangles, and nothing follows the triangles' block: the original
    // writes them past the tile, the port refuses.
    const verts = [_][3]u16{ .{ 0, 0, 0 }, .{ 0, 0, 10 }, .{ 10, 0, 10 }, .{ 10, 0, 0 } };
    const polys = [_]u16{
        0, 1,    2,    3,    none, none, none, none,
        0, none, none, none, none, none, none, none,
        1, none, none, none, none, none, none, none,
    };
    const flags = [_]u16{ 1, 1, 1 };
    const areas = [_]u8{ 0, 0, 0 };
    var params: NavMeshCreateParams = .{
        .verts = &verts,
        .vert_count = verts.len,
        .polys = &polys,
        .poly_flags = &flags,
        .poly_areas = &areas,
        .poly_count = 3,
        .vertices_per_polygon = 4,
        .bounds_min = .{ 0.0, 0.0, 0.0 },
        .bounds_max = .{ 10.0, 1.0, 10.0 },
        .walkable_height = 2.0,
        .walkable_radius = 0.5,
        .walkable_climb = 0.5,
        .cell_size = 1.0,
        .cell_height = 1.0,
        .build_bv_tree = false,
    };
    try testing.expectError(
        error.InvalidParam,
        detour.createNavMeshData(testing.allocator, &params),
    );

    // With a tree after the triangles they fit, as in the original.
    params.build_bv_tree = true;
    const tile = try detour.createNavMeshData(testing.allocator, &params);
    testing.allocator.free(tile);
}
