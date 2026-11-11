//! What the differential tests and the benchmark share: the level meshes,
//! navigation meshes built on both sides from the same Recast output, one
//! random sequence for both sides, and comparisons that print what differs.

const std = @import("std");

pub const detour = @import("detour");
pub const reference = @import("reference_detour.zig");
pub const reference_recast = @import("reference_recast.zig");
pub const reference_crowd = @import("reference_crowd.zig");
pub const reference_tile_cache = @import("reference_tile_cache.zig");

const Status = detour.Status;
const PolyRef = detour.PolyRef;
const TileRef = detour.TileRef;

pub const RecastConfig = reference_recast.RecastConfig;
pub const RecastTile = reference_recast.RecastTile;

/// RecastDemo's level meshes, as Wavefront OBJ text.
pub const dungeon_obj = @embedFile("dungeon");
pub const nav_test_obj = @embedFile("nav_test");
pub const undulating_obj = @embedFile("undulating");

/// RecastDemo's default build settings, the whole mesh as one tile.
pub const solo_config: RecastConfig = .{
    .cell_size = 0.3,
    .cell_height = 0.2,
    .agent_height = 2.0,
    .agent_radius = 0.6,
    .agent_max_climb = 0.9,
    .agent_max_slope = 45.0,
    .edge_max_len = 12.0,
    .edge_max_error = 1.3,
    .detail_sample_dist = 6.0,
    .detail_sample_max_error = 1.0,
    .region_min_size = 8,
    .region_merge_size = 20,
    .verts_per_poly = 6,
    .tile_size = 0,
    .partition = reference_recast.partition_watershed,
};

/// The same settings cut into tiles of 32 cells a side.
pub const tiled_config: RecastConfig = blk: {
    var config = solo_config;
    config.tile_size = 32;
    break :blk config;
};

// ---------------------------------------------------------------------------
// Comparisons
// ---------------------------------------------------------------------------

/// Fails unless the port's status has exactly the original's bits.
pub fn expectStatus(expected: u32, actual: Status) !void {
    const actual_bits: u32 = @bitCast(actual);
    if (expected != actual_bits) {
        std.debug.print("status: original 0x{x:0>8}, port 0x{x:0>8}\n", .{ expected, actual_bits });
        return error.TestExpectedEqual;
    }
}

/// Fails unless the two floats have the same bits (so NaNs and the sign of
/// zero count).
pub fn expectFloat(expected: f32, actual: f32) !void {
    const expected_bits: u32 = @bitCast(expected);
    const actual_bits: u32 = @bitCast(actual);
    if (expected_bits != actual_bits) {
        std.debug.print("float: original {e} (0x{x:0>8}), port {e} (0x{x:0>8})\n", .{
            expected,
            expected_bits,
            actual,
            actual_bits,
        });
        return error.TestExpectedEqual;
    }
}

/// Fails unless every float of the two slices has the same bits.
pub fn expectFloats(expected: []const f32, actual: []const f32) !void {
    if (expected.len != actual.len) {
        std.debug.print("floats: original has {d}, port {d}\n", .{ expected.len, actual.len });
        return error.TestExpectedEqual;
    }
    for (expected, actual, 0..) |e, a, i| {
        expectFloat(e, a) catch |err| {
            std.debug.print("  at index {d} of {d}\n", .{ i, expected.len });
            return err;
        };
    }
}

/// Fails unless the two points have the same bits.
pub fn expectPoint(expected: [3]f32, actual: [3]f32) !void {
    return expectFloats(&expected, &actual);
}

/// Fails unless the two byte ranges are equal; prints the first difference.
pub fn expectBytes(expected: []const u8, actual: []const u8) !void {
    if (expected.len != actual.len) {
        std.debug.print("bytes: original has {d}, port {d}\n", .{ expected.len, actual.len });
        return error.TestExpectedEqual;
    }
    for (expected, actual, 0..) |e, a, i| {
        if (e != a) {
            std.debug.print("bytes differ at {d} of {d}: original 0x{x:0>2}, port 0x{x:0>2}\n", .{
                i,
                expected.len,
                e,
                a,
            });
            return error.TestExpectedEqual;
        }
    }
}

/// The floats of `points` as one flat slice.
pub fn flat(points: anytype) []const f32 {
    return @ptrCast(points);
}

// ---------------------------------------------------------------------------
// One random sequence for both sides
// ---------------------------------------------------------------------------

/// The generator behind `referenceRandom`.
var reference_prng: std.Random.DefaultPrng = .init(0);

/// Starts the original's random callback over at `seed`. A port call given
/// `std.Random.DefaultPrng.init(seed).random()` then draws the same floats.
pub fn seedReferenceRandom(seed: u64) void {
    reference_prng = .init(seed);
}

/// The `frand` the original's random point queries take: the float the port
/// draws from `std.Random.float(f32)`.
pub fn referenceRandom() callconv(.c) f32 {
    return reference_prng.random().float(f32);
}

// ---------------------------------------------------------------------------
// Level meshes
// ---------------------------------------------------------------------------

/// A triangle soup read from OBJ text.
pub const TriangleMesh = struct {
    /// Three floats a vertex.
    verts: []f32,
    /// Three vertex indexes a triangle.
    tris: []i32,
    bounds_min: [3]f32,
    bounds_max: [3]f32,

    /// Reads the vertices and faces of `text`; a face of more than three
    /// corners becomes a fan. Free with `deinit`.
    pub fn parseObj(gpa: std.mem.Allocator, text: []const u8) !TriangleMesh {
        var verts: std.ArrayList(f32) = .empty;
        defer verts.deinit(gpa);
        var tris: std.ArrayList(i32) = .empty;
        defer tris.deinit(gpa);

        var lines = std.mem.tokenizeAny(u8, text, "\r\n");
        while (lines.next()) |line| {
            var words = std.mem.tokenizeAny(u8, line, " \t");
            const kind = words.next() orelse continue;
            if (std.mem.eql(u8, kind, "v")) {
                for (0..3) |_| {
                    const word = words.next() orelse return error.BadVertex;
                    try verts.append(gpa, try std.fmt.parseFloat(f32, word));
                }
            } else if (std.mem.eql(u8, kind, "f")) {
                const vertex_count: i32 = @intCast(verts.items.len / 3);
                var face: [32]i32 = undefined;
                var face_len: usize = 0;
                while (words.next()) |word| {
                    if (face_len == face.len) return error.BadFace;
                    const end = std.mem.indexOfScalar(u8, word, '/') orelse word.len;
                    const index = try std.fmt.parseInt(i32, word[0..end], 10);
                    face[face_len] = if (index < 0) index + vertex_count else index - 1;
                    face_len += 1;
                }
                for (2..face_len) |i| {
                    const a = face[0];
                    const b = face[i - 1];
                    const c = face[i];
                    if (a < 0 or a >= vertex_count) continue;
                    if (b < 0 or b >= vertex_count) continue;
                    if (c < 0 or c >= vertex_count) continue;
                    try tris.appendSlice(gpa, &.{ a, b, c });
                }
            }
        }
        if (verts.items.len == 0 or tris.items.len == 0) return error.EmptyMesh;

        var bounds_min: [3]f32 = verts.items[0..3].*;
        var bounds_max: [3]f32 = verts.items[0..3].*;
        var i: usize = 3;
        while (i < verts.items.len) : (i += 3) {
            for (0..3) |axis| {
                bounds_min[axis] = @min(bounds_min[axis], verts.items[i + axis]);
                bounds_max[axis] = @max(bounds_max[axis], verts.items[i + axis]);
            }
        }

        const owned_verts = try verts.toOwnedSlice(gpa);
        errdefer gpa.free(owned_verts);
        return .{
            .verts = owned_verts,
            .tris = try tris.toOwnedSlice(gpa),
            .bounds_min = bounds_min,
            .bounds_max = bounds_max,
        };
    }

    pub fn deinit(self: *TriangleMesh, gpa: std.mem.Allocator) void {
        gpa.free(self.verts);
        gpa.free(self.tris);
        self.* = undefined;
    }

    pub fn vertexCount(self: *const TriangleMesh) i32 {
        return @intCast(self.verts.len / 3);
    }

    pub fn triangleCount(self: *const TriangleMesh) i32 {
        return @intCast(self.tris.len / 3);
    }

    /// The centre of triangle `i`.
    pub fn triangleCenter(self: *const TriangleMesh, i: usize) [3]f32 {
        var center: [3]f32 = .{ 0, 0, 0 };
        for (0..3) |corner| {
            const v: usize = @intCast(self.tris[i * 3 + corner]);
            for (0..3) |axis| center[axis] += self.verts[v * 3 + axis] / 3.0;
        }
        return center;
    }

    /// The tile grid `config` cuts the mesh into, as RecastDemo's tiled
    /// sample sizes it. One tile when `config.tile_size` is 0.
    pub fn tileGrid(self: *const TriangleMesh, config: RecastConfig) TileGrid {
        if (config.tile_size == 0) return .{ .width = 1, .height = 1 };
        const cells_x: i32 = @trunc(
            (self.bounds_max[0] - self.bounds_min[0]) / config.cell_size + 0.5,
        );
        const cells_z: i32 = @trunc(
            (self.bounds_max[2] - self.bounds_min[2]) / config.cell_size + 0.5,
        );
        return .{
            .width = @divTrunc(cells_x + config.tile_size - 1, config.tile_size),
            .height = @divTrunc(cells_z + config.tile_size - 1, config.tile_size),
        };
    }
};

/// A tile grid's size in tiles.
pub const TileGrid = struct {
    width: i32,
    height: i32,
};

// ---------------------------------------------------------------------------
// Levels: a mesh, its build settings and what the tests add to it
// ---------------------------------------------------------------------------

/// Off-mesh connections in the layout the builders take: `verts` holds two
/// points a connection, the other slices one entry each.
pub const OffMeshConnections = struct {
    verts: []f32,
    radii: []f32,
    flags: []u16,
    areas: []u8,
    directions: []u8,
    user_ids: []u32,

    pub const none: OffMeshConnections = .{
        .verts = &.{},
        .radii = &.{},
        .flags = &.{},
        .areas = &.{},
        .directions = &.{},
        .user_ids = &.{},
    };

    pub fn count(self: *const OffMeshConnections) i32 {
        return @intCast(self.radii.len);
    }

    fn deinit(self: *OffMeshConnections, gpa: std.mem.Allocator) void {
        gpa.free(self.verts);
        gpa.free(self.radii);
        gpa.free(self.flags);
        gpa.free(self.areas);
        gpa.free(self.directions);
        gpa.free(self.user_ids);
        self.* = undefined;
    }
};

/// What a level adds to the bare mesh.
pub const LevelOptions = struct {
    /// Connections between random places on the ground.
    off_mesh_connection_count: u32 = 0,
    /// Gives polygons differing areas and flags, so filters have something
    /// to tell apart. Every polygon keeps flag bit 0.
    vary_polygons: bool = false,
    build_bv_tree: bool = true,
    seed: u64 = 1,
    /// When not zero, the tiles whose x + y is odd are built with cells of
    /// this size, as many as cover the same width, so their border edges
    /// end at other places than their neighbours': a polygon then meets
    /// several across one edge, and links cover part of an edge.
    odd_tile_cell_size: f32 = 0,
};

/// A triangle mesh with the settings its navigation mesh is built with.
pub const Level = struct {
    gpa: std.mem.Allocator,
    mesh: TriangleMesh,
    config: RecastConfig,
    grid: TileGrid,
    options: LevelOptions,
    /// Indexes of the triangles that face up: where the ground is.
    ground_triangles: []u32,
    off_mesh_connections: OffMeshConnections,

    /// Free with `deinit`.
    pub fn init(
        gpa: std.mem.Allocator,
        obj_text: []const u8,
        config: RecastConfig,
        options: LevelOptions,
    ) !Level {
        var mesh: TriangleMesh = try .parseObj(gpa, obj_text);
        errdefer mesh.deinit(gpa);

        var ground: std.ArrayList(u32) = .empty;
        defer ground.deinit(gpa);
        for (0..mesh.tris.len / 3) |i| {
            if (triangleNormalY(&mesh, i) > 0.8) try ground.append(gpa, @intCast(i));
        }
        if (ground.items.len == 0) return error.NoGround;
        const ground_triangles = try ground.toOwnedSlice(gpa);
        errdefer gpa.free(ground_triangles);

        var level: Level = .{
            .gpa = gpa,
            .mesh = mesh,
            .config = config,
            .grid = mesh.tileGrid(config),
            .options = options,
            .ground_triangles = ground_triangles,
            .off_mesh_connections = .none,
        };
        if (options.off_mesh_connection_count > 0) {
            level.off_mesh_connections = try level.makeOffMeshConnections();
        }
        return level;
    }

    pub fn deinit(self: *Level) void {
        if (self.off_mesh_connections.radii.len > 0) self.off_mesh_connections.deinit(self.gpa);
        self.gpa.free(self.ground_triangles);
        self.mesh.deinit(self.gpa);
        self.* = undefined;
    }

    /// A point on the level's ground, moved off it by up to `jitter` along
    /// each axis.
    pub fn groundPoint(self: *const Level, random: std.Random, jitter: f32) [3]f32 {
        const pick = random.uintLessThan(usize, self.ground_triangles.len);
        var point = self.mesh.triangleCenter(self.ground_triangles[pick]);
        for (&point) |*coordinate| coordinate.* += (random.float(f32) * 2.0 - 1.0) * jitter;
        return point;
    }

    fn makeOffMeshConnections(self: *const Level) !OffMeshConnections {
        const gpa = self.gpa;
        const n = self.options.off_mesh_connection_count;
        var prng: std.Random.DefaultPrng = .init(self.options.seed ^ 0x6f66666d657368);
        const random = prng.random();

        const verts = try gpa.alloc(f32, n * 6);
        errdefer gpa.free(verts);
        const radii = try gpa.alloc(f32, n);
        errdefer gpa.free(radii);
        const flags = try gpa.alloc(u16, n);
        errdefer gpa.free(flags);
        const areas = try gpa.alloc(u8, n);
        errdefer gpa.free(areas);
        const directions = try gpa.alloc(u8, n);
        errdefer gpa.free(directions);
        const user_ids = try gpa.alloc(u32, n);
        errdefer gpa.free(user_ids);

        for (0..n) |i| {
            const a = self.groundPoint(random, 0.0);
            const b = self.groundPoint(random, 0.0);
            verts[i * 6 ..][0..3].* = a;
            verts[i * 6 + 3 ..][0..3].* = b;
            radii[i] = 0.6 + random.float(f32);
            flags[i] = 1 | @as(u16, 1) << random.uintLessThan(u4, 4);
            areas[i] = random.uintLessThan(u8, 6);
            directions[i] = random.uintLessThan(u8, 2);
            user_ids[i] = @intCast(1000 + i);
        }
        return .{
            .verts = verts,
            .radii = radii,
            .flags = flags,
            .areas = areas,
            .directions = directions,
            .user_ids = user_ids,
        };
    }

    /// Runs Recast on tile (`x`, `y`) and readies what both builders take.
    /// Null for a tile with no ground. Free with `TileInput.deinit`.
    pub fn tileInput(self: *const Level, x: i32, y: i32) !?TileInput {
        var config = self.config;
        const odd_cell_size = self.options.odd_tile_cell_size;
        if (odd_cell_size != 0 and config.tile_size > 0 and @mod(x + y, 2) == 1) {
            const width = @as(f32, @floatFromInt(config.tile_size)) * config.cell_size;
            config.tile_size = @intFromFloat(@round(width / odd_cell_size));
            config.cell_size = odd_cell_size;
        }
        var recast: RecastTile = undefined;
        if (!reference_recast.ref_recast_build_tile(
            self.mesh.verts.ptr,
            self.mesh.vertexCount(),
            self.mesh.tris.ptr,
            self.mesh.triangleCount(),
            &config,
            &self.mesh.bounds_min,
            &self.mesh.bounds_max,
            x,
            y,
            &recast,
        )) return null;
        errdefer reference_recast.ref_recast_free_tile(&recast);

        const poly_count: usize = @intCast(recast.poly_count);
        const poly_flags = try self.gpa.dupe(u16, recast.poly_flags.?[0..poly_count]);
        errdefer self.gpa.free(poly_flags);
        const poly_areas = try self.gpa.dupe(u8, recast.poly_areas.?[0..poly_count]);
        errdefer self.gpa.free(poly_areas);
        if (self.options.vary_polygons) {
            var prng: std.Random.DefaultPrng = .init(
                self.options.seed +% @as(u64, @bitCast(@as(i64, x) * 7919 + y)),
            );
            const random = prng.random();
            for (poly_flags, poly_areas) |*flags, *area| {
                if (flags.* == 0) continue;
                flags.* = 1 | @as(u16, 1) << random.uintLessThan(u4, 4);
                area.* = random.uintLessThan(u8, 6);
            }
        }

        const connections = &self.off_mesh_connections;
        const has_connections = connections.count() > 0;
        return .{
            .gpa = self.gpa,
            .recast = recast,
            .poly_flags = poly_flags,
            .poly_areas = poly_areas,
            .reference_params = .{
                .verts = recast.verts,
                .vert_count = recast.vert_count,
                .polys = recast.polys,
                .poly_flags = poly_flags.ptr,
                .poly_areas = poly_areas.ptr,
                .poly_count = recast.poly_count,
                .nvp = recast.nvp,
                .detail_meshes = recast.detail_meshes,
                .detail_verts = recast.detail_verts,
                .detail_verts_count = recast.detail_vert_count,
                .detail_tris = recast.detail_tris,
                .detail_tri_count = recast.detail_tri_count,
                .off_mesh_con_verts = if (has_connections) connections.verts.ptr else null,
                .off_mesh_con_rad = if (has_connections) connections.radii.ptr else null,
                .off_mesh_con_flags = if (has_connections) connections.flags.ptr else null,
                .off_mesh_con_areas = if (has_connections) connections.areas.ptr else null,
                .off_mesh_con_dir = if (has_connections) connections.directions.ptr else null,
                .off_mesh_con_user_id = if (has_connections) connections.user_ids.ptr else null,
                .off_mesh_con_count = connections.count(),
                .user_id = @bitCast(x * 1000 + y),
                .tile_x = x,
                .tile_y = y,
                .tile_layer = 0,
                .bmin = recast.bmin,
                .bmax = recast.bmax,
                .walkable_height = recast.walkable_height,
                .walkable_radius = recast.walkable_radius,
                .walkable_climb = recast.walkable_climb,
                .cs = recast.cs,
                .ch = recast.ch,
                .build_bv_tree = self.options.build_bv_tree,
            },
        };
    }

    /// The parameters of a navigation mesh that holds every tile of the
    /// grid, sized as RecastDemo's tiled sample sizes them.
    pub fn navMeshParams(self: *const Level) reference.NavMeshParams {
        const tile_count: u32 = @intCast(self.grid.width * self.grid.height);
        const tile_bits: u5 = @intCast(@min(std.math.log2_int_ceil(u32, @max(tile_count, 1)), 14));
        const poly_bits: u5 = 22 - tile_bits;
        const tile_size: f32 = @floatFromInt(self.config.tile_size);
        return .{
            .orig = self.mesh.bounds_min,
            .tile_width = tile_size * self.config.cell_size,
            .tile_height = tile_size * self.config.cell_size,
            .max_tiles = @as(i32, 1) << tile_bits,
            .max_polys = @as(i32, 1) << poly_bits,
        };
    }
};

fn triangleNormalY(mesh: *const TriangleMesh, i: usize) f32 {
    var corners: [3][3]f32 = undefined;
    for (&corners, 0..) |*corner, c| {
        const v: usize = @intCast(mesh.tris[i * 3 + c]);
        corner.* = mesh.verts[v * 3 ..][0..3].*;
    }
    const e0: [3]f32 = .{
        corners[1][0] - corners[0][0],
        corners[1][1] - corners[0][1],
        corners[1][2] - corners[0][2],
    };
    const e1: [3]f32 = .{
        corners[2][0] - corners[0][0],
        corners[2][1] - corners[0][1],
        corners[2][2] - corners[0][2],
    };
    const normal: [3]f32 = .{
        e0[1] * e1[2] - e0[2] * e1[1],
        e0[2] * e1[0] - e0[0] * e1[2],
        e0[0] * e1[1] - e0[1] * e1[0],
    };
    const length = @sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
    if (length == 0) return 0;
    return normal[1] / length;
}

/// One tile's Recast output with the polygon areas and flags the level
/// gives it, as the original builder's parameters.
pub const TileInput = struct {
    gpa: std.mem.Allocator,
    recast: RecastTile,
    poly_flags: []u16,
    poly_areas: []u8,
    /// Points into `recast`, the two slices above and the level's
    /// connections: dead after `deinit` or the level's `deinit`.
    reference_params: reference.NavMeshCreateParams,

    pub fn deinit(self: *TileInput) void {
        self.gpa.free(self.poly_flags);
        self.gpa.free(self.poly_areas);
        reference_recast.ref_recast_free_tile(&self.recast);
        self.* = undefined;
    }

    /// The tile's bytes as the original builds them, or null when it
    /// refuses the input. Free with `reference.ref_free`.
    pub fn referenceBytes(self: *const TileInput) ?[]u8 {
        var data: ?[*]u8 = null;
        var size: i32 = 0;
        if (!reference.ref_create_nav_mesh_data(&self.reference_params, &data, &size)) return null;
        return data.?[0..@intCast(size)];
    }
};

/// The port's builder parameters for the same input as `params`: every
/// pointer with its count as a slice. The result points into what `params`
/// points into.
pub fn portCreateParams(
    params: *const reference.NavMeshCreateParams,
) detour.nav_mesh_builder.NavMeshCreateParams {
    const vert_count: usize = @intCast(@max(params.vert_count, 0));
    const poly_count: usize = @intCast(@max(params.poly_count, 0));
    const vertices_per_polygon: usize = @intCast(@max(params.nvp, 0));
    const detail_vert_count: usize = @intCast(@max(params.detail_verts_count, 0));
    const detail_tri_count: usize = @intCast(@max(params.detail_tri_count, 0));
    const connection_count: usize = @intCast(@max(params.off_mesh_con_count, 0));
    return .{
        .verts = if (params.verts) |p|
            @as([*]const [3]u16, @ptrCast(p))[0..vert_count]
        else
            &.{},
        .vert_count = params.vert_count,
        .polys = if (params.polys) |p| p[0 .. poly_count * vertices_per_polygon * 2] else &.{},
        .poly_flags = if (params.poly_flags) |p| p[0..poly_count] else &.{},
        .poly_areas = if (params.poly_areas) |p| p[0..poly_count] else &.{},
        .poly_count = params.poly_count,
        .vertices_per_polygon = params.nvp,
        .detail_meshes = if (params.detail_meshes) |p|
            @as([*]const [4]u32, @ptrCast(p))[0..poly_count]
        else
            null,
        .detail_verts = if (params.detail_verts) |p|
            @as([*]const [3]f32, @ptrCast(p))[0..detail_vert_count]
        else
            &.{},
        .detail_verts_count = params.detail_verts_count,
        .detail_tris = if (params.detail_tris) |p|
            @as([*]const [4]u8, @ptrCast(p))[0..detail_tri_count]
        else
            &.{},
        .detail_tri_count = params.detail_tri_count,
        .off_mesh_con_verts = if (params.off_mesh_con_verts) |p|
            @as([*]const [2][3]f32, @ptrCast(p))[0..connection_count]
        else
            &.{},
        .off_mesh_con_rad = if (params.off_mesh_con_rad) |p| p[0..connection_count] else &.{},
        .off_mesh_con_flags = if (params.off_mesh_con_flags) |p| p[0..connection_count] else &.{},
        .off_mesh_con_areas = if (params.off_mesh_con_areas) |p| p[0..connection_count] else &.{},
        .off_mesh_con_dir = if (params.off_mesh_con_dir) |p| p[0..connection_count] else &.{},
        .off_mesh_con_user_id = if (params.off_mesh_con_user_id) |p|
            p[0..connection_count]
        else
            null,
        .off_mesh_con_count = params.off_mesh_con_count,
        .user_id = params.user_id,
        .tile_x = params.tile_x,
        .tile_y = params.tile_y,
        .tile_layer = params.tile_layer,
        .bounds_min = params.bmin,
        .bounds_max = params.bmax,
        .walkable_height = params.walkable_height,
        .walkable_radius = params.walkable_radius,
        .walkable_climb = params.walkable_climb,
        .cell_size = params.cs,
        .cell_height = params.ch,
        .build_bv_tree = params.build_bv_tree,
    };
}

// ---------------------------------------------------------------------------
// Worlds: one navigation mesh on each side, holding the same tiles
// ---------------------------------------------------------------------------

/// `DT_TILE_FREE_DATA`.
const reference_free_data: i32 = 1;

/// A level's navigation mesh in the port and in the original, built from
/// the same tile bytes.
pub const World = struct {
    level: Level,
    nav: detour.NavMesh,
    reference_nav: *reference.NavMesh,

    /// Builds every tile of the level with the original's builder and adds
    /// it to both meshes. `self` must not move afterwards. Free with
    /// `deinit`.
    pub fn init(
        self: *World,
        gpa: std.mem.Allocator,
        obj_text: []const u8,
        config: RecastConfig,
        options: LevelOptions,
    ) !void {
        self.level = try .init(gpa, obj_text, config, options);
        errdefer self.level.deinit();
        self.nav = .empty;
        self.reference_nav = reference.ref_nav_mesh_alloc() orelse return error.OutOfMemory;
        errdefer reference.ref_nav_mesh_free(self.reference_nav);

        if (config.tile_size == 0) {
            var input = (try self.level.tileInput(0, 0)) orelse return error.EmptyLevel;
            defer input.deinit();
            const bytes = input.referenceBytes() orelse return error.BuilderRefused;
            const copy = try gpa.alignedAlloc(u8, .@"4", bytes.len);
            @memcpy(copy, bytes);
            const expected = reference.ref_nav_mesh_init_single(
                self.reference_nav,
                bytes.ptr,
                @intCast(bytes.len),
                reference_free_data,
            );
            const actual = self.nav.initSingleTile(gpa, copy, .{ .free_data = true });
            if (actual.failed) gpa.free(copy);
            try expectStatus(expected, actual);
            if (actual.failed) return error.InitFailed;
            return;
        }

        const params = self.level.navMeshParams();
        const expected_init = reference.ref_nav_mesh_init(self.reference_nav, &params);
        const actual_init = self.nav.init(gpa, &.{
            .origin = params.orig,
            .tile_width = params.tile_width,
            .tile_height = params.tile_height,
            .tile_count_max = params.max_tiles,
            .poly_count_max = params.max_polys,
        });
        try expectStatus(expected_init, actual_init);
        if (actual_init.failed) return error.InitFailed;
        errdefer self.nav.deinit();

        var y: i32 = 0;
        while (y < self.level.grid.height) : (y += 1) {
            var x: i32 = 0;
            while (x < self.level.grid.width) : (x += 1) {
                _ = try self.addTile(x, y);
            }
        }
    }

    pub fn deinit(self: *World) void {
        self.nav.deinit();
        reference.ref_nav_mesh_free(self.reference_nav);
        self.level.deinit();
        self.* = undefined;
    }

    /// Builds tile (`x`, `y`) and adds it to both meshes; false when the
    /// tile has no ground. The two sides must agree on the status and the
    /// new tile's reference.
    pub fn addTile(self: *World, x: i32, y: i32) !bool {
        return self.addTileReusing(x, y, 0);
    }

    /// `addTile`, asking both meshes for the tile entry and salt of
    /// `last_ref`.
    pub fn addTileReusing(self: *World, x: i32, y: i32, last_ref: TileRef) !bool {
        const gpa = self.level.gpa;
        var input = (try self.level.tileInput(x, y)) orelse return false;
        defer input.deinit();
        const bytes = input.referenceBytes() orelse return false;
        const copy = try gpa.alignedAlloc(u8, .@"4", bytes.len);
        @memcpy(copy, bytes);

        var expected_ref: TileRef = 0;
        const expected = reference.ref_nav_mesh_add_tile(
            self.reference_nav,
            bytes.ptr,
            @intCast(bytes.len),
            reference_free_data,
            last_ref,
            &expected_ref,
        );
        if (reference.ref_status_failed(expected)) reference.ref_free(bytes.ptr);
        var actual_ref: TileRef = 0;
        const actual = self.nav.addTile(copy, .{ .free_data = true }, last_ref, &actual_ref);
        if (actual.failed) gpa.free(copy);
        try expectStatus(expected, actual);
        try std.testing.expectEqual(expected_ref, actual_ref);
        return true;
    }
};
