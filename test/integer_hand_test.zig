//! The integer variant on meshes made by hand. The smallest meshes, a
//! point exactly on an edge or a vertex and an edge that rounds to nothing
//! are checked against the float port; meshes at the coordinate bound run
//! every query with safety checks on, so an overflow traps; a fixed
//! workload gives the same bytes on every run and thread count, and a hash
//! kept here on every build. The last test reads the variant's source and
//! finds no float in it.

const std = @import("std");
const build_options = @import("build_options");
const support = @import("support.zig");
const detour = support.detour;
const integer = detour.integer;

const testing = std.testing;
const PolyRef = detour.PolyRef;
const Status = detour.Status;
const coord_max = integer.common.coord_max;
const scale = integer.convert.default_scale;

/// A position's tolerance against the float port, in millimetres.
const tolerance: f64 = 2.0;
const result_count_max = 64;

fn statusBits(status: Status) u32 {
    return @bitCast(status);
}

fn metres(p: [3]i32) [3]f32 {
    return .{ metresOf(p[0]), metresOf(p[1]), metresOf(p[2]) };
}

fn metresOf(mm: i32) f32 {
    return @floatCast(@as(f64, @floatFromInt(mm)) / scale);
}

/// Fails unless `actual`, in millimetres, lies within `limit` of the float
/// `expected` along each axis.
fn expectNear(what: []const u8, expected: [3]f32, actual: [3]i32, limit: f64) !void {
    for (0..3) |axis| {
        const want = @as(f64, expected[axis]) * scale;
        const got: f64 = @floatFromInt(actual[axis]);
        if (@abs(want - got) <= limit) continue;
        std.debug.print("{s}: float {any} m, integer {any} mm\n", .{ what, expected, actual });
        return error.TestOutsideTolerance;
    }
}

// ---------------------------------------------------------------------------
// Grid meshes
// ---------------------------------------------------------------------------

/// A seeded level of square tiles, each a grid of square polygons over
/// heights that vary from vertex to vertex. A share of the squares is left
/// out, so the level has walls inside its tiles as well as at its edge. The
/// tiles are made with the float port's `createNavMeshData` and without a
/// detail mesh, so a level depends on nothing outside this repository.
const Grid = struct {
    /// The level's minimum corner, in metres.
    origin: [3]f32,
    /// The size of a cell along x and z, and along y, in metres.
    cell_size: f32,
    cell_height: f32,
    /// Squares along each side of a tile, and cells along each side of a
    /// square.
    squares: u16,
    square_cells: u16,
    /// Tiles along each side of the level.
    tiles: u16,
    /// Vertex heights lie in `0..=height_cells` cells.
    height_cells: u16,
    /// The share of squares left out, in 1/256.
    hole_share: u8,
    /// Off-mesh connections that start in each tile.
    connections: u8,
    /// Off-mesh connection radius, in metres.
    connection_radius: f32,
    /// How far up or down an agent steps, in metres; null for the whole
    /// height range.
    climb: ?f32 = null,
    seed: u64,

    fn tileSize(self: Grid) f32 {
        return @as(f32, @floatFromInt(self.squares)) *
            @as(f32, @floatFromInt(self.square_cells)) * self.cell_size;
    }

    fn mix(self: Grid, salt: u64, x: i64, z: i64) u64 {
        var key: [3]u64 = .{ self.seed ^ salt, @bitCast(x), @bitCast(z) };
        return std.hash.Wyhash.hash(0, std.mem.asBytes(&key));
    }

    /// The height of the vertex at global cell `x`, `z`, in cells: the same
    /// for both tiles at a tile border.
    fn heightAt(self: Grid, x: i64, z: i64) u16 {
        return @intCast(self.mix(1, x, z) % (@as(u64, self.height_cells) + 1));
    }

    /// Whether the square at global square `x`, `z` is left out.
    fn isHole(self: Grid, x: i64, z: i64) bool {
        return self.mix(2, x, z) % 256 < self.hole_share;
    }

    fn areaAt(self: Grid, x: i64, z: i64) u8 {
        return @intCast(self.mix(3, x, z) % 6);
    }

    fn flagsAt(self: Grid, x: i64, z: i64) u16 {
        return @as(u16, 1) << @intCast(self.mix(4, x, z) % 3);
    }

    fn params(self: Grid) detour.NavMeshParams {
        const size = self.tileSize();
        return .{
            .origin = self.origin,
            .tile_width = size,
            .tile_height = size,
            .tile_count_max = @as(i32, self.tiles) * self.tiles,
            .poly_count_max = @as(i32, self.squares) * self.squares + self.connections,
        };
    }

    /// The float tile at `tile_x`, `tile_y`, or null when every square of
    /// it is left out. Free with `gpa.free`.
    fn tile(self: Grid, gpa: std.mem.Allocator, tile_x: u16, tile_y: u16) !?[]align(4) u8 {
        const n = self.squares;
        const k = self.square_cells;
        const side = @as(usize, n) * k + 1;
        const verts = try gpa.alloc([3]u16, side * side);
        defer gpa.free(verts);
        const base_x = @as(i64, tile_x) * n * k;
        const base_z = @as(i64, tile_y) * n * k;
        for (0..side) |j| {
            for (0..side) |i| {
                const x: u16 = @intCast(i);
                const z: u16 = @intCast(j);
                const height = self.heightAt(base_x + x, base_z + z);
                verts[j * side + i] = .{ x, height, z };
            }
        }

        // Squares to polygons: each kept square is one polygon of four
        // vertices, wound as Recast winds them.
        const vertices_per_polygon = 6;
        const index_of = try gpa.alloc(u16, @as(usize, n) * n);
        defer gpa.free(index_of);
        var poly_count: u16 = 0;
        for (0..n) |qz| {
            for (0..n) |qx| {
                const gx = @as(i64, tile_x) * n + @as(i64, @intCast(qx));
                const gz = @as(i64, tile_y) * n + @as(i64, @intCast(qz));
                if (self.isHole(gx, gz)) {
                    index_of[qz * n + qx] = 0xffff;
                } else {
                    index_of[qz * n + qx] = poly_count;
                    poly_count += 1;
                }
            }
        }
        if (poly_count == 0) return null;

        const polys = try gpa.alloc(u16, @as(usize, poly_count) * 2 * vertices_per_polygon);
        defer gpa.free(polys);
        @memset(polys, 0xffff);
        const flags = try gpa.alloc(u16, poly_count);
        defer gpa.free(flags);
        const areas = try gpa.alloc(u8, poly_count);
        defer gpa.free(areas);
        for (0..n) |qz| {
            for (0..n) |qx| {
                const index = index_of[qz * n + qx];
                if (index == 0xffff) continue;
                const gx = @as(i64, tile_x) * n + @as(i64, @intCast(qx));
                const gz = @as(i64, tile_y) * n + @as(i64, @intCast(qz));
                flags[index] = self.flagsAt(gx, gz);
                areas[index] = self.areaAt(gx, gz);

                const x0 = qx * k;
                const z0 = qz * k;
                const p = polys[@as(usize, index) * 2 * vertices_per_polygon ..][0 .. 2 *
                    vertices_per_polygon];
                // West, north, east and south edges, in that order.
                p[0] = @intCast(z0 * side + x0);
                p[1] = @intCast((z0 + k) * side + x0);
                p[2] = @intCast((z0 + k) * side + x0 + k);
                p[3] = @intCast(z0 * side + x0 + k);
                const neighbours = [4]?usize{
                    if (qx > 0) qz * n + qx - 1 else null,
                    if (qz + 1 < n) (qz + 1) * n + qx else null,
                    if (qx + 1 < n) qz * n + qx + 1 else null,
                    if (qz > 0) (qz - 1) * n + qx else null,
                };
                for (neighbours, 0..) |neighbour, edge| {
                    const nei = &p[vertices_per_polygon + edge];
                    if (neighbour) |at| {
                        nei.* = index_of[at];
                    } else {
                        // A tile border: a portal to the tile on that side.
                        nei.* = 0x8000 | @as(u16, @intCast(edge));
                    }
                }
            }
        }

        const size = self.tileSize();
        const bounds_min: [3]f32 = .{
            self.origin[0] + @as(f32, @floatFromInt(tile_x)) * size,
            self.origin[1],
            self.origin[2] + @as(f32, @floatFromInt(tile_y)) * size,
        };
        const bounds_max: [3]f32 = .{
            bounds_min[0] + size,
            bounds_min[1] + @as(f32, @floatFromInt(self.height_cells)) * self.cell_height,
            bounds_min[2] + size,
        };

        // Connections from the middle of one kept square to the middle of
        // another, maybe in the next tile; the middle's height is that of
        // the square's diagonal from its first vertex.
        var connection_verts: [8][2][3]f32 = undefined;
        var connection_rads: [8]f32 = undefined;
        var connection_flags: [8]u16 = undefined;
        var connection_areas: [8]u8 = undefined;
        var connection_dirs: [8]u8 = undefined;
        var connection_count: usize = 0;
        var prng: std.Random.DefaultPrng = .init(self.mix(5, tile_x, tile_y));
        const random = prng.random();
        const last_square = @as(i64, self.tiles) * n - 1;
        for (0..@min(self.connections, connection_verts.len)) |_| {
            const from_x = @as(i64, tile_x) * n + random.uintLessThan(u16, n);
            const from_z = @as(i64, tile_y) * n + random.uintLessThan(u16, n);
            const to_x = std.math.clamp(from_x + random.intRangeAtMost(i64, -3, 3), 0, last_square);
            const to_z = std.math.clamp(from_z + random.intRangeAtMost(i64, -3, 3), 0, last_square);
            const c = connection_count;
            connection_verts[c] = .{
                self.squareMiddle(from_x, from_z),
                self.squareMiddle(to_x, to_z),
            };
            connection_rads[c] = self.connection_radius;
            connection_flags[c] = 1;
            connection_areas[c] = @intCast(c % 6);
            connection_dirs[c] = @intCast(c % 2);
            connection_count += 1;
        }

        const create: detour.NavMeshCreateParams = .{
            .verts = verts,
            .vert_count = @intCast(verts.len),
            .polys = polys,
            .poly_flags = flags,
            .poly_areas = areas,
            .poly_count = poly_count,
            .vertices_per_polygon = vertices_per_polygon,
            .off_mesh_con_verts = connection_verts[0..connection_count],
            .off_mesh_con_rad = connection_rads[0..connection_count],
            .off_mesh_con_flags = connection_flags[0..connection_count],
            .off_mesh_con_areas = connection_areas[0..connection_count],
            .off_mesh_con_dir = connection_dirs[0..connection_count],
            .off_mesh_con_count = @intCast(connection_count),
            .tile_x = tile_x,
            .tile_y = tile_y,
            .bounds_min = bounds_min,
            .bounds_max = bounds_max,
            .walkable_height = 2,
            .walkable_radius = 0.5,
            .walkable_climb = self.climb orelse
                @as(f32, @floatFromInt(self.height_cells)) * self.cell_height,
            .cell_size = self.cell_size,
            .cell_height = self.cell_height,
            .build_bv_tree = true,
        };
        return try detour.createNavMeshData(gpa, &create);
    }

    /// The middle of the square at global square `qx`, `qz`, in metres.
    fn squareMiddle(self: Grid, qx: i64, qz: i64) [3]f32 {
        const k: i64 = self.square_cells;
        const x0 = qx * k;
        const z0 = qz * k;
        const low: f32 = @floatFromInt(self.heightAt(x0, z0));
        const high: f32 = @floatFromInt(self.heightAt(x0 + k, z0 + k));
        const half: f32 = @floatFromInt(@divTrunc(k, 2));
        return .{
            self.origin[0] + (@as(f32, @floatFromInt(x0)) + half) * self.cell_size,
            self.origin[1] + (low + high) / 2 * self.cell_height,
            self.origin[2] + (@as(f32, @floatFromInt(z0)) + half) * self.cell_size,
        };
    }

    /// The level's integer mesh: every tile converted, in row order. Free
    /// with `deinit`.
    fn integerMesh(self: Grid, gpa: std.mem.Allocator, nav: *integer.NavMesh) !void {
        const float_params = self.params();
        const integer_params: integer.NavMeshParams = .{
            .origin = millimetres(float_params.origin),
            .tile_width = @intFromFloat(@round(@as(f64, float_params.tile_width) * scale)),
            .tile_height = @intFromFloat(@round(@as(f64, float_params.tile_height) * scale)),
            .tile_count_max = float_params.tile_count_max,
            .poly_count_max = float_params.poly_count_max,
        };
        nav.* = .empty;
        if (nav.init(gpa, &integer_params).failed) return error.OutOfMemory;
        errdefer nav.deinit();
        for (0..self.tiles) |y| {
            for (0..self.tiles) |x| {
                const float_tile = try self.tile(gpa, @intCast(x), @intCast(y)) orelse continue;
                defer gpa.free(float_tile);
                const data = try integer.convert.tileFromFloat(gpa, float_tile, scale);
                if (nav.addTile(data, .{ .free_data = true }, 0, null).failed) {
                    gpa.free(data);
                    return error.TileRefused;
                }
            }
        }
    }

    /// The level's float mesh, every tile in row order. Free with `deinit`.
    fn floatMesh(self: Grid, gpa: std.mem.Allocator, nav: *detour.NavMesh) !void {
        nav.* = .empty;
        if (nav.init(gpa, &self.params()).failed) return error.OutOfMemory;
        errdefer nav.deinit();
        for (0..self.tiles) |y| {
            for (0..self.tiles) |x| {
                const data = try self.tile(gpa, @intCast(x), @intCast(y)) orelse continue;
                if (nav.addTile(data, .{ .free_data = true }, 0, null).failed) {
                    gpa.free(data);
                    return error.TileRefused;
                }
            }
        }
    }

    /// A seeded point over the level, in whole millimetres.
    fn point(self: Grid, random: std.Random) [3]i32 {
        const low = millimetres(self.origin);
        const tiles: f64 = @floatFromInt(self.tiles);
        const size: i32 = @intFromFloat(@round(@as(f64, self.tileSize()) * scale * tiles));
        const height_cells: f64 = @floatFromInt(self.height_cells);
        const height_mm = height_cells * @as(f64, self.cell_height) * scale;
        const height: i32 = @intFromFloat(@round(height_mm));
        return .{
            low[0] + random.intRangeAtMost(i32, 0, size),
            low[1] + random.intRangeAtMost(i32, 0, height),
            low[2] + random.intRangeAtMost(i32, 0, size),
        };
    }
};

fn millimetres(p: [3]f32) [3]i32 {
    var out: [3]i32 = undefined;
    for (&out, p) |*o, v| o.* = @intFromFloat(@round(@as(f64, v) * scale));
    return out;
}

// ---------------------------------------------------------------------------
// Determinism
// ---------------------------------------------------------------------------

/// Everything the queries of one run returned, as raw bytes in call order.
const Sink = struct {
    bytes: []u8,
    len: usize = 0,
    overflowed: bool = false,

    fn put(self: *Sink, value: anytype) void {
        self.putBytes(std.mem.asBytes(&value));
    }

    fn putSlice(self: *Sink, comptime T: type, items: []const T) void {
        self.putBytes(std.mem.sliceAsBytes(items));
    }

    fn putBytes(self: *Sink, raw: []const u8) void {
        if (raw.len > self.bytes.len - self.len) {
            self.overflowed = true;
            return;
        }
        @memcpy(self.bytes[self.len..][0..raw.len], raw);
        self.len += raw.len;
    }

    fn written(self: *const Sink) []const u8 {
        return self.bytes[0..self.len];
    }
};

/// The workload's level: nine tiles of eight by eight squares within 100 m
/// of the origin, a tenth of the squares left out, with connections.
const workload_grid: Grid = .{
    .origin = .{ -40, 0.5, -30 },
    .cell_size = 0.25,
    .cell_height = 0.2,
    .squares = 8,
    .square_cells = 8,
    .tiles = 3,
    .height_cells = 4,
    .hole_share = 26,
    .connections = 2,
    .connection_radius = 0.5,
    .seed = 0x696e7465676572,
};

const workload_points = 401;

/// How far the workload's searches reach, in millimetres.
const Reach = struct {
    /// The half extents of the search for the nearest polygon.
    nearest: [3]i32,
    /// The half extents of `queryPolygons`, plus up to six steps along x.
    box: [3]i32,
    box_step: i32,
    /// The radius of the searches around a point, plus up to eight steps.
    radius: i32,
    radius_step: i32,
    /// Whether ray casts run, the any-angle search's among them. On a
    /// damaged tile a ray can walk between two polygons for ever, as the
    /// original's does.
    rays: bool = true,
};

const near_reach: Reach = .{
    .nearest = .{ 2000, 4000, 2000 },
    .box = .{ 3000, 2000, 4000 },
    .box_step = 1000,
    .radius = 1000,
    .radius_step = 700,
};

/// Up to the whole coordinate range.
const far_reach: Reach = .{
    .nearest = @splat(coord_max),
    .box = .{ coord_max / 2, coord_max, coord_max / 2 },
    .box_step = coord_max / 12,
    .radius = coord_max / 3,
    .radius_step = coord_max / 12,
};

/// Runs every query of the variant from each point to the next and records
/// all they return. Every output starts at zero, so what a failed query
/// leaves alone writes the same bytes too.
fn runQueries(
    query: *integer.NavMeshQuery,
    points: []const [3]i32,
    reach: Reach,
    sink: *Sink,
) void {
    var filters: [2]integer.QueryFilter = .{ .{}, .{} };
    for (&filters[1].area_cost, 0..) |*cost, area| cost.* = @intCast((area % 4 + 1) << 15);
    filters[1].exclude_flags = 0x4;

    for (points[0 .. points.len - 1], points[1..], 0..) |start, end, i| {
        const filter = &filters[i % 2];
        var start_ref: PolyRef = 0;
        var end_ref: PolyRef = 0;
        var from = start;
        var to = end;
        var over = false;
        const nearest = reach.nearest;
        sink.put(statusBits(query.findNearestPoly(
            start,
            nearest,
            filter,
            &start_ref,
            &from,
            &over,
        )));
        sink.put(statusBits(query.findNearestPoly(end, nearest, filter, &end_ref, &to, null)));
        sink.put(start_ref);
        sink.put(end_ref);
        sink.put(from);
        sink.put(to);
        sink.put(over);

        var refs: [result_count_max]PolyRef = @splat(0);
        var parents: [result_count_max]PolyRef = @splat(0);
        var costs: [result_count_max]u64 = @splat(0);
        var count: usize = 0;
        var box = reach.box;
        box[0] += @as(i32, @intCast(i % 7)) * reach.box_step;
        count = 0;
        sink.put(statusBits(query.queryPolygons(start, box, filter, &refs, &count)));
        sink.putSlice(PolyRef, refs[0..count]);

        var closest: [3]i32 = @splat(0);
        var height: i32 = 0;
        sink.put(statusBits(query.closestPointOnPoly(start_ref, end, &closest, &over)));
        sink.put(closest);
        sink.put(over);
        sink.put(statusBits(query.closestPointOnPolyBoundary(start_ref, end, &closest)));
        sink.put(closest);
        sink.put(statusBits(query.getPolyHeight(start_ref, from, &height)));
        sink.put(height);

        var path: [result_count_max]PolyRef = @splat(0);
        var path_count: usize = 0;
        sink.put(statusBits(query.findPath(
            start_ref,
            end_ref,
            from,
            to,
            filter,
            &path,
            &path_count,
        )));
        sink.putSlice(PolyRef, path[0..path_count]);

        const options = [_]detour.StraightPathOptions{
            .{},
            .{ .area_crossings = true },
            .{ .all_crossings = true },
        };
        var corners: [result_count_max][3]i32 = @splat(@splat(0));
        var corner_flags: [result_count_max]detour.StraightPathFlags = @splat(.{});
        var corner_refs: [result_count_max]PolyRef = @splat(0);
        var corner_count: usize = 0;
        sink.put(statusBits(query.findStraightPath(
            from,
            to,
            path[0..path_count],
            &corners,
            &corner_flags,
            &corner_refs,
            &corner_count,
            options[i % options.len],
        )));
        sink.putSlice([3]i32, corners[0..corner_count]);
        sink.putSlice(detour.StraightPathFlags, corner_flags[0..corner_count]);
        sink.putSlice(PolyRef, corner_refs[0..corner_count]);

        const find_options: detour.FindPathOptions = .{ .any_angle = reach.rays and i % 3 == 0 };
        sink.put(statusBits(query.initSlicedFindPath(
            start_ref,
            end_ref,
            from,
            to,
            filter,
            find_options,
        )));
        var done: i32 = 0;
        sink.put(statusBits(query.updateSlicedFindPath(7, &done)));
        sink.put(done);
        sink.put(statusBits(query.updateSlicedFindPath(1000, &done)));
        sink.put(done);
        path_count = 0;
        sink.put(statusBits(query.finalizeSlicedFindPath(&path, &path_count)));
        sink.putSlice(PolyRef, path[0..path_count]);

        var hit: integer.RaycastHit = .{
            .t = .zero,
            .hit_normal = @splat(0),
            .hit_edge_index = 0,
            .path = &refs,
            .path_count = 0,
            .path_cost = 0,
        };
        const ray_options: detour.RaycastOptions = .{ .use_costs = i % 2 == 0 };
        if (reach.rays) {
            sink.put(statusBits(query.raycast(start_ref, from, end, filter, ray_options, &hit, 0)));
        }
        sink.put(hit.t.num);
        sink.put(hit.t.den);
        sink.put(hit.hit_normal);
        sink.put(hit.hit_edge_index);
        sink.put(hit.path_cost);
        sink.putSlice(PolyRef, refs[0..hit.path_count]);

        var moved: [3]i32 = @splat(0);
        count = 0;
        sink.put(statusBits(query.moveAlongSurface(
            start_ref,
            from,
            end,
            filter,
            &moved,
            &refs,
            &count,
        )));
        sink.put(moved);
        sink.putSlice(PolyRef, refs[0..count]);

        const radius = reach.radius + @as(i32, @intCast(i % 9)) * reach.radius_step;
        count = 0;
        sink.put(statusBits(query.findPolysAroundCircle(
            start_ref,
            from,
            radius,
            filter,
            &refs,
            &parents,
            &costs,
            &count,
            result_count_max,
        )));
        sink.putSlice(PolyRef, refs[0..count]);
        sink.putSlice(PolyRef, parents[0..count]);
        sink.putSlice(u64, costs[0..count]);
        if (count > 0) {
            path_count = 0;
            sink.put(statusBits(query.getPathFromDijkstraSearch(
                refs[count - 1],
                &path,
                &path_count,
            )));
            sink.putSlice(PolyRef, path[0..path_count]);
        }

        const low = @max(from[0] - radius, -coord_max);
        const high = @min(from[0] + radius, coord_max);
        const shape = [4][3]i32{
            .{ low, from[1], from[2] },
            .{ from[0], from[1], @min(from[2] + radius, coord_max) },
            .{ high, from[1], from[2] },
            .{ from[0], from[1], @max(from[2] - radius, -coord_max) },
        };
        count = 0;
        sink.put(statusBits(query.findPolysAroundShape(
            start_ref,
            &shape,
            filter,
            &refs,
            &parents,
            &costs,
            &count,
            result_count_max,
        )));
        sink.putSlice(PolyRef, refs[0..count]);
        sink.putSlice(PolyRef, parents[0..count]);
        sink.putSlice(u64, costs[0..count]);

        count = 0;

        sink.put(statusBits(query.findLocalNeighbourhood(
            start_ref,
            from,
            radius,
            filter,
            &refs,
            &parents,
            &count,
        )));
        sink.putSlice(PolyRef, refs[0..count]);
        sink.putSlice(PolyRef, parents[0..count]);

        var segments: [result_count_max][2][3]i32 = @splat(@splat(@splat(0)));
        count = 0;
        sink.put(statusBits(query.getPolyWallSegments(
            start_ref,
            filter,
            &segments,
            &refs,
            &count,
        )));
        sink.putSlice([2][3]i32, segments[0..count]);
        sink.putSlice(PolyRef, refs[0..count]);

        var wall_distance: i32 = 0;
        var wall_point: [3]i32 = @splat(0);
        var wall_normal: [3]i32 = @splat(0);
        sink.put(statusBits(query.findDistanceToWall(
            start_ref,
            from,
            radius,
            filter,
            &wall_distance,
            &wall_point,
            &wall_normal,
        )));
        sink.put(wall_distance);
        sink.put(wall_point);
        sink.put(wall_normal);

        var prng: std.Random.DefaultPrng = .init(i);
        var random_ref: PolyRef = 0;
        var random_point: [3]i32 = @splat(0);
        sink.put(statusBits(query.findRandomPoint(
            filter,
            prng.random(),
            &random_ref,
            &random_point,
        )));
        sink.put(random_ref);
        sink.put(random_point);
        sink.put(statusBits(query.findRandomPointAroundCircle(
            start_ref,
            from,
            radius,
            filter,
            prng.random(),
            &random_ref,
            &random_point,
        )));
        sink.put(random_ref);
        sink.put(random_point);
    }
}

/// One thread's query object and what its run wrote.
const Worker = struct {
    query: integer.NavMeshQuery,
    sink: Sink,

    fn init(self: *Worker, gpa: std.mem.Allocator, nav: *const integer.NavMesh) !void {
        self.query = .empty;
        if (self.query.init(gpa, nav, 512).failed) return error.OutOfMemory;
        errdefer self.query.deinit(gpa);
        self.sink = .{ .bytes = try gpa.alloc(u8, sink_size) };
    }

    fn deinit(self: *Worker, gpa: std.mem.Allocator) void {
        gpa.free(self.sink.bytes);
        self.query.deinit(gpa);
        self.* = undefined;
    }

    fn run(self: *Worker, points: []const [3]i32) void {
        self.sink.len = 0;
        runQueries(&self.query, points, near_reach, &self.sink);
    }
};

/// Room for everything one run writes: each pair of points writes less
/// than 12 KiB.
const sink_size = workload_points * 12 * 1024;

fn expectSameRun(expected: *const Sink, actual: *const Sink, thread_count: usize) !void {
    try testing.expect(!actual.overflowed);
    if (std.mem.eql(u8, expected.written(), actual.written())) return;
    const first = std.mem.findDiff(u8, expected.written(), actual.written()) orelse 0;
    std.debug.print("a run on {d} threads differs from the first run at byte {d}\n", .{
        thread_count,
        first,
    });
    return error.TestExpectedEqual;
}

/// The hash of the workload's bytes, the same in every optimisation mode
/// and on every machine. References of 64 bits write other bytes.
const workload_hash: u64 = if (build_options.polyref64) 0x30a41686767b8ff7 else 0x1705edd8311a47b4;

test "integer workload: the same bytes on every run and on 1, 2 and many threads" {
    const gpa = testing.allocator;
    var nav: integer.NavMesh = undefined;
    try workload_grid.integerMesh(gpa, &nav);
    defer nav.deinit();

    var prng: std.Random.DefaultPrng = .init(0x776f726b6c6f6164);
    const points = try gpa.alloc([3]i32, workload_points);
    defer gpa.free(points);
    for (points) |*point| point.* = workload_grid.point(prng.random());

    var first: Worker = undefined;
    try first.init(gpa, &nav);
    defer first.deinit(gpa);
    first.run(points);
    try testing.expect(!first.sink.overflowed);
    try testing.expect(first.sink.len > workload_points * 256);

    // The same query object again, its node pool as the first run left it.
    {
        var again: Worker = undefined;
        try again.init(gpa, &nav);
        defer again.deinit(gpa);
        runQueries(&first.query, points, near_reach, &again.sink);
        try expectSameRun(&first.sink, &again.sink, 1);
    }

    const cpu_count = std.Thread.getCpuCount() catch 4;
    const thread_counts = [_]usize{ 1, 2, std.math.clamp(cpu_count, 3, 8) };
    for (thread_counts) |thread_count| {
        const workers = try gpa.alloc(Worker, thread_count);
        defer gpa.free(workers);
        var ready: usize = 0;
        defer for (workers[0..ready]) |*worker| worker.deinit(gpa);
        for (workers) |*worker| {
            try worker.init(gpa, &nav);
            ready += 1;
        }

        const threads = try gpa.alloc(std.Thread, thread_count);
        defer gpa.free(threads);
        var started: usize = 0;
        errdefer for (threads[0..started]) |thread| thread.join();
        for (threads, workers) |*thread, *worker| {
            thread.* = try std.Thread.spawn(.{}, Worker.run, .{ worker, points });
            started += 1;
        }
        for (threads) |thread| thread.join();
        started = 0;

        for (workers) |*worker| try expectSameRun(&first.sink, &worker.sink, thread_count);
    }

    const hash = std.hash.Wyhash.hash(0, first.sink.written());
    if (hash != workload_hash) {
        std.debug.print("workload hash 0x{x:0>16}, {d} bytes\n", .{ hash, first.sink.len });
        return error.TestExpectedEqual;
    }
}

// ---------------------------------------------------------------------------
// The coordinate bound
// ---------------------------------------------------------------------------

/// A level across the whole coordinate range: two by two tiles of two by
/// two squares, a quarter of the range each, heights across the range too,
/// and connections a fifth of the range wide.
const bound_grid: Grid = .{
    .origin = .{ -536870, -536870, -536870 },
    .cell_size = 134217.5,
    .cell_height = 268435,
    .squares = 2,
    .square_cells = 2,
    .tiles = 2,
    .height_cells = 4,
    .hole_share = 0,
    .connections = 2,
    .connection_radius = 100000,
    .climb = 536870,
    .seed = 0x626f756e64,
};

test "integer queries at the coordinate bound: no overflow, answers inside the bound" {
    const gpa = testing.allocator;
    var nav: integer.NavMesh = undefined;
    try bound_grid.integerMesh(gpa, &nav);
    defer nav.deinit();
    var query: integer.NavMeshQuery = .empty;
    if (query.init(gpa, &nav, 2048).failed) return error.OutOfMemory;
    defer query.deinit(gpa);

    // The level reaches within a millimetre of the bound on each side.
    var low: [3]i32 = @splat(coord_max);
    var high: [3]i32 = @splat(-coord_max);
    for (nav.tiles) |*tile| {
        if (tile.header == null) continue;
        for (tile.verts) |vert| {
            for (&low, &high, vert) |*l, *h, v| {
                l.* = @min(l.*, v);
                h.* = @max(h.*, v);
            }
        }
    }
    for (low, high) |l, h| {
        try testing.expect(l <= -coord_max + 1000);
        try testing.expect(h >= coord_max - 1000);
    }

    // Every corner of the bound and of each face, then seeded points.
    const m = coord_max;
    var points: [27 + 200][3]i32 = undefined;
    for (0..27) |i| {
        const at = [3]i32{ -m, 0, m };
        points[i] = .{ at[i % 3], at[i / 3 % 3], at[i / 9] };
    }
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(0x65646765));
    for (points[27..]) |*point| point.* = bound_grid.point(prng.random());
    const sink_bytes = try gpa.alloc(u8, points.len * 12 * 1024);
    defer gpa.free(sink_bytes);
    var sink: Sink = .{ .bytes = sink_bytes };
    runQueries(&query, &points, far_reach, &sink);
    runQueries(&query, &points, near_reach, &sink);
    try testing.expect(!sink.overflowed);

    // From any corner the nearest polygon is found, and a path, a ray and a
    // move to the opposite corner give points inside the bound.
    const filter: integer.QueryFilter = .{};
    for (points[0..27]) |a| {
        const b: [3]i32 = .{ -a[0], -a[1], -a[2] };
        var a_ref: PolyRef = 0;
        var b_ref: PolyRef = 0;
        var a_point: [3]i32 = undefined;
        var b_point: [3]i32 = undefined;
        try testing.expect(query.findNearestPoly(
            a,
            @splat(m),
            &filter,
            &a_ref,
            &a_point,
            null,
        ).succeeded);
        try testing.expect(query.findNearestPoly(
            b,
            @splat(m),
            &filter,
            &b_ref,
            &b_point,
            null,
        ).succeeded);
        try testing.expect(a_ref != 0 and b_ref != 0);
        try testing.expect(integer.common.inBounds(a_point) and integer.common.inBounds(b_point));

        var path: [result_count_max]PolyRef = undefined;
        var path_count: usize = 0;
        const found = query.findPath(a_ref, b_ref, a_point, b_point, &filter, &path, &path_count);
        try testing.expect(found.succeeded and path_count > 0);
        var corners: [result_count_max][3]i32 = undefined;
        var corner_count: usize = 0;
        const straight = query.findStraightPath(
            a_point,
            b_point,
            path[0..path_count],
            &corners,
            null,
            null,
            &corner_count,
            .{ .all_crossings = true },
        );
        // From the middle to itself the path is one corner.
        const corner_count_min: usize = if (std.meta.eql(a_point, b_point)) 1 else 2;
        try testing.expect(straight.succeeded and corner_count >= corner_count_min);
        for (corners[0..corner_count]) |corner| try testing.expect(integer.common.inBounds(corner));

        var moved: [3]i32 = undefined;
        var visited: [result_count_max]PolyRef = undefined;
        var visited_count: usize = 0;
        const move = query.moveAlongSurface(
            a_ref,
            a_point,
            b,
            &filter,
            &moved,
            &visited,
            &visited_count,
        );
        try testing.expect(move.succeeded and integer.common.inBounds(moved));

        var hit: integer.RaycastHit = .{
            .t = .zero,
            .hit_normal = @splat(0),
            .hit_edge_index = 0,
            .path = &visited,
            .path_count = 0,
            .path_cost = 0,
        };
        const ray = query.raycast(a_ref, a_point, b, &filter, .{ .use_costs = true }, &hit, 0);
        try testing.expect(ray.succeeded);
        try testing.expect(integer.common.inBounds(hit.point(a_point, b)));

        var distance: i32 = 0;
        var wall: [3]i32 = undefined;
        var normal: [3]i32 = undefined;
        const walls = query.findDistanceToWall(
            a_ref,
            a_point,
            m,
            &filter,
            &distance,
            &wall,
            &normal,
        );
        try testing.expect(walls.succeeded and distance >= 0 and distance <= m);
    }

    // One millimetre past the bound is refused.
    var ref: PolyRef = 0;
    var point: [3]i32 = undefined;
    const past = [3]i32{ m + 1, 0, 0 };
    const refused: Status = .{ .failed = true, .invalid_param = true };
    try testing.expectEqual(refused, query.findNearestPoly(
        past,
        @splat(1),
        &filter,
        &ref,
        &point,
        null,
    ));
    try testing.expectEqual(refused, query.findNearestPoly(
        @splat(0),
        .{ m + 1, 1, 1 },
        &filter,
        &ref,
        &point,
        null,
    ));
    _ = query.findNearestPoly(@splat(0), @splat(m), &filter, &ref, &point, null);
    var distance: i32 = 0;
    var normal: [3]i32 = undefined;
    try testing.expectEqual(refused, query.findDistanceToWall(
        ref,
        point,
        m + 1,
        &filter,
        &distance,
        &point,
        &normal,
    ));
    var count: usize = 0;
    var refs: [result_count_max]PolyRef = undefined;
    try testing.expectEqual(refused, query.findPolysAroundCircle(
        ref,
        point,
        m + 1,
        &filter,
        &refs,
        null,
        null,
        &count,
        refs.len,
    ));
}

// ---------------------------------------------------------------------------
// The smallest meshes, against the float port
// ---------------------------------------------------------------------------

/// A mesh on both sides: a float mesh and its conversion, with a query of
/// each. `self` must not move after `init`.
const HandPair = struct {
    float_nav: detour.NavMesh,
    nav: integer.NavMesh,
    float_query: detour.NavMeshQuery,
    query: integer.NavMeshQuery,

    /// Builds one tile from `create`, or a mesh with no tile when it is
    /// null. Free with `deinit`.
    fn init(self: *HandPair, gpa: std.mem.Allocator, create: ?detour.NavMeshCreateParams) !void {
        self.float_nav = .empty;
        self.nav = .empty;
        if (create) |params| {
            const float_tile = try detour.createNavMeshData(gpa, &params);
            const data = integer.convert.tileFromFloat(gpa, float_tile, scale) catch |err| {
                gpa.free(float_tile);
                return err;
            };
            if (self.float_nav.initSingleTile(gpa, float_tile, .{ .free_data = true }).failed) {
                gpa.free(float_tile);
                gpa.free(data);
                return error.TileRefused;
            }
            errdefer self.float_nav.deinit();
            if (self.nav.initSingleTile(gpa, data, .{ .free_data = true }).failed) {
                gpa.free(data);
                return error.TileRefused;
            }
        } else {
            const float_params: detour.NavMeshParams = .{
                .origin = @splat(0),
                .tile_width = 10,
                .tile_height = 10,
                .tile_count_max = 4,
                .poly_count_max = 16,
            };
            if (self.float_nav.init(gpa, &float_params).failed) return error.OutOfMemory;
            errdefer self.float_nav.deinit();
            const params: integer.NavMeshParams = .{
                .origin = @splat(0),
                .tile_width = 10000,
                .tile_height = 10000,
                .tile_count_max = 4,
                .poly_count_max = 16,
            };
            if (self.nav.init(gpa, &params).failed) return error.OutOfMemory;
        }
        errdefer self.float_nav.deinit();
        errdefer self.nav.deinit();
        self.float_query = .empty;
        if (self.float_query.init(gpa, &self.float_nav, 256).failed) return error.OutOfMemory;
        errdefer self.float_query.deinit(gpa);
        self.query = .empty;
        if (self.query.init(gpa, &self.nav, 256).failed) return error.OutOfMemory;
    }

    fn deinit(self: *HandPair, gpa: std.mem.Allocator) void {
        self.query.deinit(gpa);
        self.float_query.deinit(gpa);
        self.nav.deinit();
        self.float_nav.deinit();
        self.* = undefined;
    }

    /// The point queries at `p` on polygon `ref` give the same statuses and
    /// flags on both sides, and points within tolerance.
    fn expectSamePointQueries(self: *HandPair, ref: PolyRef, p: [3]i32) !void {
        errdefer std.debug.print("  at {any} on polygon {d}\n", .{ p, ref });
        var float_closest: [3]f32 = undefined;
        var float_over = false;
        var closest: [3]i32 = undefined;
        var over = false;
        const float_status = self.float_query.closestPointOnPoly(
            ref,
            metres(p),
            &float_closest,
            &float_over,
        );
        const status = self.query.closestPointOnPoly(ref, p, &closest, &over);
        try testing.expectEqual(statusBits(float_status), statusBits(status));
        if (!status.failed) {
            try testing.expectEqual(float_over, over);
            try expectNear("closest point", float_closest, closest, tolerance);
        }

        const float_boundary = self.float_query.closestPointOnPolyBoundary(
            ref,
            metres(p),
            &float_closest,
        );
        const boundary = self.query.closestPointOnPolyBoundary(ref, p, &closest);
        try testing.expectEqual(statusBits(float_boundary), statusBits(boundary));
        if (!boundary.failed) try expectNear("boundary point", float_closest, closest, tolerance);

        var float_height: f32 = 0;
        var height: i32 = 0;
        const float_height_status = self.float_query.getPolyHeight(ref, metres(p), &float_height);
        const height_status = self.query.getPolyHeight(ref, p, &height);
        try testing.expectEqual(statusBits(float_height_status), statusBits(height_status));
        if (!height_status.failed) {
            const want = @as(f64, float_height) * scale;
            try testing.expect(@abs(want - @as(f64, @floatFromInt(height))) <= 2 * tolerance);
        }

        const filter: integer.QueryFilter = .{};
        const float_filter: detour.QueryFilter = .{};
        var float_distance: f32 = 0;
        var float_wall: [3]f32 = undefined;
        var float_normal: [3]f32 = undefined;
        var distance: i32 = 0;
        var wall: [3]i32 = undefined;
        var normal: [3]i32 = undefined;
        const float_walls = self.float_query.findDistanceToWall(
            ref,
            metres(p),
            5,
            &float_filter,
            &float_distance,
            &float_wall,
            &float_normal,
        );
        const walls = self.query.findDistanceToWall(
            ref,
            p,
            5000,
            &filter,
            &distance,
            &wall,
            &normal,
        );
        try testing.expectEqual(statusBits(float_walls), statusBits(walls));
        if (!walls.failed) {
            const want = @as(f64, float_distance) * scale;
            try testing.expect(@abs(want - @as(f64, @floatFromInt(distance))) <= tolerance);
        }
    }

    /// The nearest polygon to `p`: the same on both sides, or, where `p`
    /// lies as near two polygons, polygons that both hold it.
    fn expectSameNearest(self: *HandPair, p: [3]i32, half_extents: [3]i32) !void {
        errdefer std.debug.print("  nearest to {any}\n", .{p});
        var float_ref: PolyRef = 0;
        var float_point: [3]f32 = undefined;
        var float_over = false;
        var ref: PolyRef = 0;
        var point: [3]i32 = undefined;
        var over = false;
        const float_status = self.float_query.findNearestPoly(
            metres(p),
            metres(half_extents),
            &.{},
            &float_ref,
            &float_point,
            &float_over,
        );
        const status = self.query.findNearestPoly(p, half_extents, &.{}, &ref, &point, &over);
        try testing.expectEqual(statusBits(float_status), statusBits(status));
        try testing.expectEqual(float_ref == 0, ref == 0);
        if (ref == 0) return;
        try expectNear("nearest point", float_point, point, tolerance);
        if (float_ref == ref) {
            try testing.expectEqual(float_over, over);
            return;
        }
        var closest: [3]i32 = undefined;
        for ([_]PolyRef{ float_ref, ref }) |either| {
            _ = self.query.closestPointOnPoly(either, p, &closest, null);
            for (closest, point) |a, b| try testing.expect(@abs(a - b) <= tolerance);
        }
    }
};

test "integer on a mesh with no tile: the float port's statuses" {
    const gpa = testing.allocator;
    var pair: HandPair = undefined;
    try pair.init(gpa, null);
    defer pair.deinit(gpa);
    const p: [3]i32 = .{ 1000, 0, 1000 };
    const q: [3]i32 = .{ 3000, 0, 2000 };
    const filter: integer.QueryFilter = .{};
    const float_filter: detour.QueryFilter = .{};

    try pair.expectSameNearest(p, .{ 2000, 2000, 2000 });
    var float_refs: [result_count_max]PolyRef = undefined;
    var refs: [result_count_max]PolyRef = undefined;
    var float_count: usize = 0;
    var count: usize = 0;
    try testing.expectEqual(
        statusBits(pair.float_query.queryPolygons(
            metres(p),
            .{ 2, 2, 2 },
            &float_filter,
            &float_refs,
            &float_count,
        )),
        statusBits(pair.query.queryPolygons(p, .{ 2000, 2000, 2000 }, &filter, &refs, &count)),
    );
    try testing.expectEqual(0, count);

    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(1));
    var float_point: [3]f32 = undefined;
    var point: [3]i32 = undefined;
    var ref: PolyRef = 0;
    try testing.expectEqual(
        statusBits(pair.float_query.findRandomPoint(
            &float_filter,
            prng.random(),
            &ref,
            &float_point,
        )),
        statusBits(pair.query.findRandomPoint(&filter, prng.random(), &ref, &point)),
    );

    // No polygon, and one that never was: the same refusals.
    for ([_]PolyRef{ 0, 12345 }) |bad| {
        errdefer std.debug.print("  polygon {d}\n", .{bad});
        try pair.expectSamePointQueries(bad, p);
        try testing.expectEqual(
            statusBits(pair.float_query.findPath(
                bad,
                bad,
                metres(p),
                metres(q),
                &float_filter,
                &float_refs,
                &float_count,
            )),
            statusBits(pair.query.findPath(bad, bad, p, q, &filter, &refs, &count)),
        );
        try testing.expectEqual(
            statusBits(pair.float_query.initSlicedFindPath(
                bad,
                bad,
                metres(p),
                metres(q),
                &float_filter,
                .{},
            )),
            statusBits(pair.query.initSlicedFindPath(bad, bad, p, q, &filter, .{})),
        );
        try testing.expectEqual(
            statusBits(pair.float_query.moveAlongSurface(
                bad,
                metres(p),
                metres(q),
                &float_filter,
                &float_point,
                &float_refs,
                &float_count,
            )),
            statusBits(pair.query.moveAlongSurface(bad, p, q, &filter, &point, &refs, &count)),
        );
        var float_hit: detour.RaycastHit = .{
            .t = 0,
            .hit_normal = @splat(0),
            .hit_edge_index = 0,
            .path = &float_refs,
            .path_count = 0,
            .path_cost = 0,
        };
        var hit: integer.RaycastHit = .{
            .t = .zero,
            .hit_normal = @splat(0),
            .hit_edge_index = 0,
            .path = &refs,
            .path_count = 0,
            .path_cost = 0,
        };
        try testing.expectEqual(
            statusBits(pair.float_query.raycast(
                bad,
                metres(p),
                metres(q),
                &float_filter,
                .{},
                &float_hit,
                0,
            )),
            statusBits(pair.query.raycast(bad, p, q, &filter, .{}, &hit, 0)),
        );
        try testing.expectEqual(
            statusBits(pair.float_query.findPolysAroundCircle(
                bad,
                metres(p),
                3,
                &float_filter,
                &float_refs,
                null,
                null,
                &float_count,
                float_refs.len,
            )),
            statusBits(pair.query.findPolysAroundCircle(
                bad,
                p,
                3000,
                &filter,
                &refs,
                null,
                null,
                &count,
                refs.len,
            )),
        );
        const float_shape = [3][3]f32{ metres(p), metres(q), .{ 1, 0, 3 } };
        const shape = [3][3]i32{ p, q, .{ 1000, 0, 3000 } };
        try testing.expectEqual(
            statusBits(pair.float_query.findPolysAroundShape(
                bad,
                &float_shape,
                &float_filter,
                &float_refs,
                null,
                null,
                &float_count,
                float_refs.len,
            )),
            statusBits(pair.query.findPolysAroundShape(
                bad,
                &shape,
                &filter,
                &refs,
                null,
                null,
                &count,
                refs.len,
            )),
        );
        try testing.expectEqual(
            statusBits(pair.float_query.findLocalNeighbourhood(
                bad,
                metres(p),
                3,
                &float_filter,
                &float_refs,
                null,
                &float_count,
            )),
            statusBits(pair.query.findLocalNeighbourhood(
                bad,
                p,
                3000,
                &filter,
                &refs,
                null,
                &count,
            )),
        );
        var float_segments: [result_count_max][2][3]f32 = undefined;
        var segments: [result_count_max][2][3]i32 = undefined;
        try testing.expectEqual(
            statusBits(pair.float_query.getPolyWallSegments(
                bad,
                &float_filter,
                &float_segments,
                null,
                &float_count,
            )),
            statusBits(pair.query.getPolyWallSegments(bad, &filter, &segments, null, &count)),
        );
        try testing.expectEqual(
            statusBits(pair.float_query.findRandomPointAroundCircle(
                bad,
                metres(p),
                3,
                &float_filter,
                prng.random(),
                &ref,
                &float_point,
            )),
            statusBits(pair.query.findRandomPointAroundCircle(
                bad,
                p,
                3000,
                &filter,
                prng.random(),
                &ref,
                &point,
            )),
        );
    }
    var float_corners: [4][3]f32 = undefined;
    var corners: [4][3]i32 = undefined;
    try testing.expectEqual(
        statusBits(pair.float_query.findStraightPath(
            metres(p),
            metres(q),
            &.{},
            &float_corners,
            null,
            null,
            &float_count,
            .{},
        )),
        statusBits(pair.query.findStraightPath(p, q, &.{}, &corners, null, null, &count, .{})),
    );
}

/// One square polygon on a slope, 4 m a side, its corners on whole
/// millimetres: cells of 0.25 m and 0.2 m from the origin.
fn squareParams(verts: []const [3]u16, polys: []const u16) detour.NavMeshCreateParams {
    return .{
        .verts = verts,
        .vert_count = @intCast(verts.len),
        .polys = polys,
        .poly_flags = &.{1},
        .poly_areas = &.{0},
        .poly_count = 1,
        .vertices_per_polygon = 4,
        .bounds_min = .{ 0, 0, 0 },
        .bounds_max = .{ 4, 2, 4 },
        .walkable_height = 2,
        .walkable_radius = 0.5,
        .walkable_climb = 0.9,
        .cell_size = 0.25,
        .cell_height = 0.2,
        .build_bv_tree = true,
    };
}

test "integer on one polygon: points inside, on an edge, at a vertex and outside" {
    const gpa = testing.allocator;
    const verts = [_][3]u16{ .{ 0, 0, 0 }, .{ 0, 5, 16 }, .{ 16, 10, 16 }, .{ 16, 5, 0 } };
    const polys = [_]u16{ 0, 1, 2, 3, 0xffff, 0xffff, 0xffff, 0xffff };
    var pair: HandPair = undefined;
    try pair.init(gpa, squareParams(&verts, &polys));
    defer pair.deinit(gpa);
    const ref = pair.float_nav.getPolyRefBase(&pair.float_nav.tiles[0]);

    // The corners lie at 0 and 4000 mm, 0 to 2000 mm up.
    const points = [_][3]i32{
        .{ 2000, 1000, 2000 }, // the middle
        .{ 1234, 900, 3071 }, // inside
        .{ 0, 500, 2000 }, // the middle of the west edge
        .{ 2000, 1500, 4000 }, // the middle of the north edge
        .{ 0, 0, 0 }, // a corner
        .{ 4000, 2000, 4000 }, // the opposite corner
        .{ -500, 0, 2000 }, // west of the polygon
        .{ -300, 700, -300 }, // beyond a corner
        .{ 2000, 5000, 2000 }, // high above the middle
    };
    for (points) |p| {
        try pair.expectSamePointQueries(ref, p);
        try pair.expectSameNearest(p, .{ 1000, 4000, 1000 });
    }

    const filter: integer.QueryFilter = .{};
    const float_filter: detour.QueryFilter = .{};
    const middle = points[0];

    // Rays from the middle: one ends inside, the others leave through each
    // edge and through a corner.
    const ends = [_][3]i32{
        .{ 3000, 1000, 1000 },
        .{ -1000, 1000, 2500 },
        .{ 2500, 1000, 6000 },
        .{ 6000, 1000, 1500 },
        .{ 1500, 1000, -3000 },
        .{ 5000, 1000, 5000 },
    };
    for (ends) |end| {
        errdefer std.debug.print("  ray to {any}\n", .{end});
        var float_path: [result_count_max]PolyRef = undefined;
        var path: [result_count_max]PolyRef = undefined;
        var float_hit: detour.RaycastHit = .{
            .t = 0,
            .hit_normal = @splat(0),
            .hit_edge_index = 0,
            .path = &float_path,
            .path_count = 0,
            .path_cost = 0,
        };
        var hit: integer.RaycastHit = .{
            .t = .zero,
            .hit_normal = @splat(0),
            .hit_edge_index = 0,
            .path = &path,
            .path_count = 0,
            .path_cost = 0,
        };
        const float_status = pair.float_query.raycast(
            ref,
            metres(middle),
            metres(end),
            &float_filter,
            .{ .use_costs = true },
            &float_hit,
            0,
        );
        const status = pair.query.raycast(
            ref,
            middle,
            end,
            &filter,
            .{ .use_costs = true },
            &hit,
            0,
        );
        try testing.expectEqual(statusBits(float_status), statusBits(status));
        try testing.expectEqual(float_hit.hit_edge_index, hit.hit_edge_index);
        try testing.expectEqualSlices(
            PolyRef,
            float_path[0..float_hit.path_count],
            path[0..hit.path_count],
        );
        const float_hit_wall = float_hit.t <= 1;
        try testing.expectEqual(float_hit_wall, hit.t.den != 0);
        if (float_hit_wall) {
            var float_at: [3]f32 = undefined;
            for (&float_at, metres(middle), metres(end)) |*a, s, e| a.* = s + (e - s) * float_hit.t;
            try expectNear("hit point", float_at, hit.point(middle, end), tolerance);
        }

        var float_moved: [3]f32 = undefined;
        var moved: [3]i32 = undefined;
        var float_count: usize = 0;
        var count: usize = 0;
        const float_move = pair.float_query.moveAlongSurface(
            ref,
            metres(middle),
            metres(end),
            &float_filter,
            &float_moved,
            &float_path,
            &float_count,
        );
        const move = pair.query.moveAlongSurface(ref, middle, end, &filter, &moved, &path, &count);
        try testing.expectEqual(statusBits(float_move), statusBits(move));
        try expectNear("moved point", float_moved, moved, tolerance);
    }

    // The walls are the polygon's four edges, at the same corners.
    var float_segments: [result_count_max][2][3]f32 = undefined;
    var segments: [result_count_max][2][3]i32 = undefined;
    var float_count: usize = 0;
    var count: usize = 0;
    _ = pair.float_query.getPolyWallSegments(
        ref,
        &float_filter,
        &float_segments,
        null,
        &float_count,
    );
    try testing.expect(pair.query.getPolyWallSegments(
        ref,
        &filter,
        &segments,
        null,
        &count,
    ).succeeded);
    try testing.expectEqual(4, count);
    try testing.expectEqual(float_count, count);
    for (float_segments[0..count], segments[0..count]) |float_segment, segment| {
        for (float_segment, segment) |float_end, end| {
            try expectNear("wall end", float_end, end, 0.5);
        }
    }

    // A path on one polygon is that polygon, and its straight path is its
    // two ends.
    var path: [result_count_max]PolyRef = undefined;
    try testing.expect(pair.query.findPath(
        ref,
        ref,
        middle,
        points[1],
        &filter,
        &path,
        &count,
    ).succeeded);
    try testing.expectEqualSlices(PolyRef, &.{ref}, path[0..count]);
    var corners: [4][3]i32 = undefined;
    _ = pair.query.findStraightPath(middle, points[1], &.{ref}, &corners, null, null, &count, .{});
    try testing.expectEqualSlices([3]i32, &.{ middle, points[1] }, corners[0..count]);

    // A random point lies on the polygon.
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(7));
    for (0..100) |_| {
        var random_ref: PolyRef = 0;
        var random_point: [3]i32 = undefined;
        try testing.expect(pair.query.findRandomPoint(
            &filter,
            prng.random(),
            &random_ref,
            &random_point,
        ).succeeded);
        try testing.expectEqual(ref, random_ref);
        var over = false;
        var closest: [3]i32 = undefined;
        _ = pair.query.closestPointOnPoly(ref, random_point, &closest, &over);
        try testing.expect(over);
    }
}

test "integer with an edge that rounds to nothing: what the float port does around it" {
    const gpa = testing.allocator;
    // Cells of 0.4 mm. Two polygons share the line x = 4 m: a pentagon to
    // the east and one to the west, joined along two edges, from A to B,
    // 0.4 mm long, which the conversion makes a point, and from B to C.
    const verts = [_][3]u16{
        .{ 10000, 0, 0 }, // A
        .{ 10000, 0, 1 }, // B
        .{ 10000, 5, 10000 }, // C
        .{ 20000, 5, 10000 }, // D
        .{ 20000, 0, 0 }, // E
        .{ 0, 0, 0 }, // F
        .{ 0, 5, 10000 }, // G
    };
    const wall = 0xffff;
    const polys = [_]u16{
        0, 1, 2, 3, 4, 0xffff, 1,    1,    wall, wall, wall, wall,
        5, 6, 2, 1, 0, 0xffff, wall, wall, 0,    0,    wall, wall,
    };
    var params = squareParams(&verts, &polys);
    params.poly_flags = &.{ 1, 1 };
    params.poly_areas = &.{ 0, 0 };
    params.poly_count = 2;
    params.vertices_per_polygon = 6;
    params.bounds_max = .{ 8, 1, 4 };
    params.cell_size = 0.0004;
    var pair: HandPair = undefined;
    try pair.init(gpa, params);
    defer pair.deinit(gpa);

    const base = pair.float_nav.getPolyRefBase(&pair.float_nav.tiles[0]);
    const east = base;
    const west = base | 1;
    const tile = &pair.nav.tiles[0];
    try testing.expectEqual(tile.verts[0], tile.verts[1]);

    const filter: integer.QueryFilter = .{};
    const float_filter: detour.QueryFilter = .{};

    // Near and at the point the short edge became.
    const points = [_][3]i32{
        .{ 4000, 0, 0 },
        .{ 4001, 0, 1 },
        .{ 3999, 0, 1 },
        .{ 4000, 0, 2000 },
        .{ 4500, 0, -500 },
        .{ 3500, 0, -500 },
    };
    for (points) |p| {
        try pair.expectSamePointQueries(east, p);
        try pair.expectSamePointQueries(west, p);
        try pair.expectSameNearest(p, .{ 300, 2000, 300 });
    }

    // Both polygons' edges, the short one a point on this side.
    for ([_]PolyRef{ east, west }) |ref| {
        var float_segments: [result_count_max][2][3]f32 = undefined;
        var float_refs: [result_count_max]PolyRef = undefined;
        var segments: [result_count_max][2][3]i32 = undefined;
        var refs: [result_count_max]PolyRef = undefined;
        var float_count: usize = 0;
        var count: usize = 0;
        _ = pair.float_query.getPolyWallSegments(
            ref,
            &float_filter,
            &float_segments,
            &float_refs,
            &float_count,
        );
        _ = pair.query.getPolyWallSegments(ref, &filter, &segments, &refs, &count);
        try testing.expectEqual(5, count);
        try testing.expectEqual(float_count, count);
        try testing.expectEqualSlices(PolyRef, float_refs[0..count], refs[0..count]);
        for (float_segments[0..count], segments[0..count]) |float_segment, segment| {
            for (float_segment, segment) |float_end, end| {
                try expectNear("edge end", float_end, end, 0.5);
            }
        }
    }

    // From the east polygon to the west one: the same corridor, the same
    // straight path, and a ray through the long shared edge.
    const from: [3]i32 = .{ 6000, 500, 2000 };
    const to: [3]i32 = .{ 2000, 500, 2000 };
    var float_path: [result_count_max]PolyRef = undefined;
    var path: [result_count_max]PolyRef = undefined;
    var float_count: usize = 0;
    var count: usize = 0;
    _ = pair.float_query.findPath(
        east,
        west,
        metres(from),
        metres(to),
        &float_filter,
        &float_path,
        &float_count,
    );
    _ = pair.query.findPath(east, west, from, to, &filter, &path, &count);
    try testing.expectEqualSlices(PolyRef, &.{ east, west }, path[0..count]);
    try testing.expectEqualSlices(PolyRef, float_path[0..float_count], path[0..count]);

    var float_corners: [8][3]f32 = undefined;
    var corners: [8][3]i32 = undefined;
    _ = pair.float_query.findStraightPath(
        metres(from),
        metres(to),
        path[0..count],
        &float_corners,
        null,
        null,
        &float_count,
        .{},
    );
    var corner_count: usize = 0;
    _ = pair.query.findStraightPath(
        from,
        to,
        path[0..count],
        &corners,
        null,
        null,
        &corner_count,
        .{},
    );
    try testing.expectEqual(float_count, corner_count);
    for (float_corners[0..float_count], corners[0..corner_count]) |float_corner, corner| {
        try expectNear("corner", float_corner, corner, tolerance);
    }

    for ([_][3]i32{ to, .{ 2000, 500, -1000 }, .{ 6000, 500, -1000 } }) |end| {
        errdefer std.debug.print("  ray to {any}\n", .{end});
        var float_hit: detour.RaycastHit = .{
            .t = 0,
            .hit_normal = @splat(0),
            .hit_edge_index = 0,
            .path = &float_path,
            .path_count = 0,
            .path_cost = 0,
        };
        var hit: integer.RaycastHit = .{
            .t = .zero,
            .hit_normal = @splat(0),
            .hit_edge_index = 0,
            .path = &path,
            .path_count = 0,
            .path_cost = 0,
        };
        const float_status = pair.float_query.raycast(
            east,
            metres(from),
            metres(end),
            &float_filter,
            .{},
            &float_hit,
            0,
        );
        const status = pair.query.raycast(east, from, end, &filter, .{}, &hit, 0);
        try testing.expectEqual(statusBits(float_status), statusBits(status));
        try testing.expectEqual(float_hit.hit_edge_index, hit.hit_edge_index);
        try testing.expectEqualSlices(
            PolyRef,
            float_path[0..float_hit.path_count],
            path[0..hit.path_count],
        );

        var float_moved: [3]f32 = undefined;
        var moved: [3]i32 = undefined;
        const float_move = pair.float_query.moveAlongSurface(
            east,
            metres(from),
            metres(end),
            &float_filter,
            &float_moved,
            &float_path,
            &float_count,
        );
        const move = pair.query.moveAlongSurface(east, from, end, &filter, &moved, &path, &count);
        try testing.expectEqual(statusBits(float_move), statusBits(move));
        try testing.expectEqualSlices(PolyRef, float_path[0..float_count], path[0..count]);
        try expectNear("moved point", float_moved, moved, tolerance);
    }
}

// ---------------------------------------------------------------------------
// No float
// ---------------------------------------------------------------------------

/// The variant's source files: all but the converter, which reads floats.
const integer_sources = [_]struct { name: []const u8, text: [:0]const u8 }{
    .{ .name = "integer.zig", .text = @embedFile("integer_source_root") },
    .{ .name = "integer/common.zig", .text = @embedFile("integer_source_common") },
    .{ .name = "integer/nav_mesh.zig", .text = @embedFile("integer_source_nav_mesh") },
    .{ .name = "integer/node.zig", .text = @embedFile("integer_source_node") },
    .{ .name = "integer/nav_mesh_query.zig", .text = @embedFile("integer_source_nav_mesh_query") },
};

/// The names the variant takes from the float port's mesh module: types
/// and values it shares, each checked below to hold no float.
const shared_names = [_][]const u8{
    "area_count_max",                    "BvNode",              "DecodedPolyId",
    "detail_edge_boundary",              "ext_link",            "FindPathOptions",
    "getDetailTriEdgeFlags",             "Link",                "nav_mesh_state_magic",
    "nav_mesh_state_version",            "nav_mesh_version",    "null_link",
    "off_mesh_connection_bidirectional", "Poly",                "poly_bits_64",
    "PolyDetail",                        "PolyRef",             "polyref64",
    "PolyType",                          "RaycastOptions",      "salt_bits_64",
    "StraightPathFlags",                 "StraightPathOptions", "tile_bits_64",
    "TileFlags",                         "TileLocation",        "TileRef",
    "verts_per_polygon",
};

/// Whether values of `T` hold no float anywhere: fields, elements, what
/// pointers and optionals hold, and a function's parameters and result.
fn holdsNoFloat(comptime T: type) bool {
    return switch (@typeInfo(T)) {
        .float, .comptime_float => false,
        .@"struct" => |info| for (info.field_types) |field_type| {
            if (!holdsNoFloat(field_type)) break false;
        } else true,
        .@"union" => |info| for (info.field_types) |field_type| {
            if (!holdsNoFloat(field_type)) break false;
        } else true,
        .array => |info| holdsNoFloat(info.child),
        .vector => |info| holdsNoFloat(info.child),
        .pointer => |info| holdsNoFloat(info.child),
        .optional => |info| holdsNoFloat(info.child),
        .error_union => |info| holdsNoFloat(info.payload),
        .@"fn" => |info| for (info.param_types) |param_type| {
            if (param_type) |P| {
                if (!holdsNoFloat(P)) break false;
            }
        } else if (info.return_type) |R| holdsNoFloat(R) else true,
        else => true,
    };
}

comptime {
    for (shared_names) |name| {
        const value = @field(detour.nav_mesh, name);
        const T = if (@TypeOf(value) == type) value else @TypeOf(value);
        if (!holdsNoFloat(T)) @compileError(name ++ " holds a float");
    }
}

/// A float in Zig source: where it starts and what it is.
const FloatUse = struct {
    offset: usize,
    text: []const u8,
};

/// The first float in Zig source `text`, comments and strings aside: a
/// float type, a builtin that takes or makes a float, a float constant or
/// test of `std.math`, a float literal, or a name from the float port's
/// mesh module outside `shared_names`. Null when there is none.
fn findFloat(text: [:0]const u8) ?FloatUse {
    const float_names = [_][]const u8{
        "f16",          "f32",      "f64",      "f80",      "f128",     "comptime_float",
        "c_longdouble", "inf",      "nan",      "pi",       "tau",      "isNan",
        "isInf",        "isFinite", "floatEps", "floatMax", "floatMin", "floatTrueMin",
        "hypot",        "atan2",    "copysign",
    };
    const float_builtins = [_][]const u8{
        "@floatFromInt", "@intFromFloat", "@floatCast", "@sqrt",  "@sin",    "@cos",
        "@tan",          "@exp",          "@exp2",      "@log",   "@log2",   "@log10",
        "@floor",        "@ceil",         "@trunc",     "@round", "@mulAdd",
    };
    var tokenizer: std.zig.Tokenizer = .init(text);
    var after_float_module = false;
    while (true) {
        const token = tokenizer.next();
        const word = text[token.loc.start..token.loc.end];
        const found: FloatUse = .{ .offset = token.loc.start, .text = word };
        switch (token.tag) {
            .eof => return null,
            .identifier => {
                if (after_float_module) {
                    after_float_module = false;
                    for (shared_names) |name| {
                        if (std.mem.eql(u8, name, word)) break;
                    } else return found;
                }
                for (float_names) |name| if (std.mem.eql(u8, name, word)) return found;
                if (std.mem.eql(u8, word, "float_nav_mesh")) {
                    if (tokenizer.next().tag == .period) after_float_module = true;
                }
            },
            .builtin => for (float_builtins) |name| {
                if (std.mem.eql(u8, name, word)) return found;
            },
            .number_literal => {
                const hex = std.mem.startsWith(u8, word, "0x");
                if (std.mem.findScalar(u8, word, '.') != null) return found;
                if (hex and std.mem.findAny(u8, word, "pP") != null) return found;
                if (!hex and std.mem.findAny(u8, word, "eE") != null) return found;
            },
            else => {},
        }
    }
}

test "integer source: the scan finds each kind of float and nothing else" {
    const floats = [_][:0]const u8{
        "const a: f32 = 0;",
        "const a = @as(f64, 0);",
        "const a: comptime_float = 0;",
        "const a = 1.5;",
        "const a = 1e3;",
        "const a = 0x1p4;",
        "const a = @sqrt(b);",
        "const a = @floatFromInt(b);",
        "const a = std.math.inf(T);",
        "const a = float_nav_mesh.MeshHeader;",
    };
    for (floats) |text| {
        errdefer std.debug.print("  missed in: {s}\n", .{text});
        try testing.expect(findFloat(text) != null);
    }
    const clean = [_][:0]const u8{
        "const a: i32 = 0x1e;",
        "for (0..n) |i| _ = i;",
        "// a f32 of 1.5 in a comment",
        "/// 2.5 mm in a doc comment",
        "const s = \"f64 1.5\";",
        "const a = @abs(b) + @truncate(c);",
        "const a = float_nav_mesh.Link;",
        "const a = 1_000_000;",
    };
    for (clean) |text| {
        errdefer std.debug.print("  found in: {s}\n", .{text});
        try testing.expect(findFloat(text) == null);
    }
}

test "integer source: no float in the variant, the converter aside" {
    for (integer_sources) |source| {
        if (findFloat(source.text)) |use| {
            const line = 1 + std.mem.countScalar(u8, source.text[0..use.offset], '\n');
            std.debug.print("{s}:{d}: {s}\n", .{ source.name, line, use.text });
            return error.FloatInIntegerSource;
        }
    }
}

// ---------------------------------------------------------------------------
// Damaged tiles (skeptic)
// ---------------------------------------------------------------------------

/// A random value near `count`: zero, one off either way, below zero or
/// anything.
fn skepticOddCount(random: std.Random, count: i32) i32 {
    return switch (random.uintLessThan(u8, 6)) {
        0 => 0,
        1 => count +% 1,
        2 => count -% 1,
        3 => -1,
        4 => std.math.maxInt(i32),
        else => random.int(i32),
    };
}

/// A coordinate within the bound, at it, or anywhere.
fn skepticCoord(random: std.Random) i32 {
    return switch (random.uintLessThan(u8, 4)) {
        0 => coord_max,
        1 => -coord_max,
        2 => random.intRangeAtMost(i32, -coord_max, coord_max),
        else => random.int(i32),
    };
}

/// Damages a tile blob of either kind in place, the float and the integer
/// tile sharing one layout, and returns how many of its bytes to hand on.
fn skepticDamage(blob: []align(4) u8, random: std.Random) usize {
    const Header = integer.MeshHeader;
    const header: *Header = @ptrCast(blob.ptr);
    const count = struct {
        fn of(v: i32) usize {
            return @intCast(@max(v, 0));
        }
    }.of;
    const verts_at = std.mem.alignForward(usize, @sizeOf(Header), 4);
    const polys_at = verts_at + 12 * count(header.vert_count);
    const links_at = polys_at + @sizeOf(detour.nav_mesh.Poly) * count(header.poly_count);
    const details_at = links_at + @sizeOf(detour.nav_mesh.Link) * count(header.link_count_max);
    const detail_verts_at = details_at + std.mem.alignForward(
        usize,
        @sizeOf(detour.nav_mesh.PolyDetail) * count(header.detail_mesh_count),
        4,
    );
    const tris_at = detail_verts_at + 12 * count(header.detail_vert_count);
    const nodes_at = tris_at + 4 * count(header.detail_tri_count);
    const connections_at = nodes_at + @sizeOf(detour.nav_mesh.BvNode) * count(header.bv_node_count);
    const end = connections_at +
        @sizeOf(integer.nav_mesh.OffMeshConnection) * count(header.off_mesh_connection_count);
    std.debug.assert(end <= blob.len);

    const polys: []align(4) detour.nav_mesh.Poly = @ptrCast(@alignCast(blob[polys_at..links_at]));
    const details: []align(4) detour.nav_mesh.PolyDetail =
        @ptrCast(@alignCast(blob[details_at..][0 .. @sizeOf(detour.nav_mesh.PolyDetail) *
            count(header.detail_mesh_count)]));
    const tris: []align(4) [4]u8 = @ptrCast(@alignCast(blob[tris_at..nodes_at]));
    const nodes: []align(4) detour.nav_mesh.BvNode = @ptrCast(@alignCast(blob[nodes_at..connections_at]));
    const connections: []align(4) integer.nav_mesh.OffMeshConnection =
        @ptrCast(@alignCast(blob[connections_at..end]));
    const coords: []align(4) i32 = @ptrCast(@alignCast(blob[verts_at..polys_at]));
    const detail_coords: []align(4) i32 = @ptrCast(@alignCast(blob[detail_verts_at..tris_at]));
    const vert_count: u32 = @intCast(count(header.vert_count));

    switch (random.uintLessThan(u8, 13)) {
        0 => for (0..random.intRangeAtMost(usize, 1, 4)) |_| {
            blob[random.uintLessThan(usize, end)] ^= @as(u8, 1) << random.int(u3);
        },
        1 => {
            const counts: [*]align(4) i32 = @ptrCast(&header.poly_count);
            const which = random.uintLessThan(usize, 8);
            counts[which] = skepticOddCount(random, counts[which]);
        },
        2 => if (polys.len > 0) {
            const poly = &polys[random.uintLessThan(usize, polys.len)];
            poly.verts[random.uintLessThan(usize, 6)] = if (random.boolean())
                @truncate(vert_count)
            else
                random.int(u16);
        },
        3 => if (polys.len > 0) {
            const poly = &polys[random.uintLessThan(usize, polys.len)];
            poly.neighbours[random.uintLessThan(usize, 6)] = switch (random.uintLessThan(u8, 4)) {
                0 => @truncate(random.uintLessThan(usize, polys.len + 2)),
                1 => 0x8000 | random.int(u16),
                2 => 0x8000 | (@as(u16, random.uintLessThan(u8, 4)) * 2),
                else => random.int(u16),
            };
        },
        4 => if (polys.len > 0) {
            const poly = &polys[random.uintLessThan(usize, polys.len)];
            if (random.boolean()) poly.vert_count = random.uintLessThan(u8, 8) else {
                poly.area_and_type = random.int(u8);
            }
        },
        5 => if (details.len > 0) {
            const detail = &details[random.uintLessThan(usize, details.len)];
            switch (random.uintLessThan(u8, 4)) {
                0 => detail.vert_base = random.int(u32) >> random.int(u5),
                1 => detail.tri_base = random.int(u32) >> random.int(u5),
                2 => detail.vert_count = random.int(u8),
                else => detail.tri_count = random.int(u8),
            }
        },
        6 => if (tris.len > 0) {
            tris[random.uintLessThan(usize, tris.len)][random.uintLessThan(usize, 4)] =
                random.int(u8);
        },
        7 => if (nodes.len > 0) {
            const node = &nodes[random.uintLessThan(usize, nodes.len)];
            switch (random.uintLessThan(u8, 4)) {
                0 => node.index = @intCast(polys.len),
                1 => node.index = -random.intRangeAtMost(i32, 0, @intCast(nodes.len + 1)),
                2 => node.index = random.int(i32),
                else => random.bytes(std.mem.asBytes(node)[0..12]),
            }
        },
        8 => if (connections.len > 0) {
            const connection = &connections[random.uintLessThan(usize, connections.len)];
            switch (random.uintLessThan(u8, 4)) {
                0 => connection.poly = @truncate(random.uintLessThan(usize, polys.len + 2)),
                1 => connection.side = random.int(u8),
                2 => connection.radius = skepticCoord(random),
                else => connection.pos[random.uintLessThan(usize, 2)][random.uintLessThan(usize, 3)] =
                    skepticCoord(random),
            }
        },
        9 => return random.uintLessThan(usize, end),
        10 => for (0..random.intRangeAtMost(usize, 1, 3)) |_| {
            const which = if (detail_coords.len > 0 and random.boolean()) detail_coords else coords;
            if (which.len > 0) which[random.uintLessThan(usize, which.len)] = skepticCoord(random);
        },
        11 => switch (random.uintLessThan(u8, 6)) {
            0 => header.x +%= random.intRangeAtMost(i32, -2, 2),
            1 => header.layer = random.intRangeAtMost(i32, -1, 3),
            2 => header.off_mesh_base = random.intRangeAtMost(i32, -1, @intCast(polys.len + 1)),
            3 => header.walkable_climb = skepticCoord(random),
            4 => header.bv_cell_size = random.intRangeAtMost(i32, -1, 3),
            else => header.bounds_min[random.uintLessThan(usize, 3)] = skepticCoord(random),
        },
        else => {
            const starts = [_]usize{ verts_at, polys_at, details_at, detail_verts_at, tris_at, nodes_at, connections_at, end };
            const block = random.uintLessThan(usize, starts.len - 1);
            random.bytes(blob[starts[block]..starts[block + 1]]);
        },
    }
    return blob.len;
}

test "skeptic: integer addTile and tileFromFloat take damaged tiles safely or refuse them" {
    const gpa = testing.allocator;
    const grid = workload_grid;
    var float_tiles: std.ArrayList([]align(4) u8) = .empty;
    defer {
        for (float_tiles.items) |t| gpa.free(t);
        float_tiles.deinit(gpa);
    }
    for (0..grid.tiles) |y| {
        for (0..grid.tiles) |x| {
            const t = try grid.tile(gpa, @intCast(x), @intCast(y)) orelse continue;
            try float_tiles.append(gpa, t);
        }
    }
    const float_params = grid.params();
    const integer_params: integer.NavMeshParams = .{
        .origin = millimetres(float_params.origin),
        .tile_width = @intFromFloat(@round(@as(f64, float_params.tile_width) * scale)),
        .tile_height = @intFromFloat(@round(@as(f64, float_params.tile_height) * scale)),
        .tile_count_max = float_params.tile_count_max,
        .poly_count_max = float_params.poly_count_max,
    };

    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(0x736b6570746963));
    const random = prng.random();
    var points: [16][3]i32 = undefined;
    const sink_bytes = try gpa.alloc(u8, points.len * 12 * 1024);
    defer gpa.free(sink_bytes);
    const no_rays: Reach = blk: {
        var r = near_reach;
        r.rays = false;
        break :blk r;
    };

    var converter_refused: usize = 0;
    var refused: usize = 0;
    var queried: usize = 0;
    for (0..1500) |case| {
        errdefer std.debug.print("  skeptic damaged-tile case {d}\n", .{case});
        const index = random.uintLessThan(usize, float_tiles.items.len);
        const through_converter = case % 3 == 0;

        var damaged: []align(4) u8 = undefined;
        var len: usize = 0;
        if (through_converter) {
            const copy = try gpa.alignedAlloc(u8, .@"4", float_tiles.items[index].len);
            defer gpa.free(copy);
            @memcpy(copy, float_tiles.items[index]);
            len = skepticDamage(copy, random);
            damaged = integer.convert.tileFromFloat(gpa, copy[0..len], scale) catch {
                converter_refused += 1;
                continue;
            };
            len = damaged.len;
        } else {
            damaged = try integer.convert.tileFromFloat(gpa, float_tiles.items[index], scale);
            len = skepticDamage(damaged, random);
        }
        defer gpa.free(damaged);

        var nav: integer.NavMesh = .empty;
        try testing.expect(nav.init(gpa, &integer_params).succeeded);
        defer nav.deinit();
        if (random.boolean()) {
            const other = random.uintLessThan(usize, float_tiles.items.len);
            if (other != index) {
                const sound = try integer.convert.tileFromFloat(gpa, float_tiles.items[other], scale);
                if (nav.addTile(sound, .{ .free_data = true }, 0, null).failed) gpa.free(sound);
            }
        }
        var tile_ref: integer.TileRef = 0;
        if (nav.addTile(damaged[0..len], .{}, 0, &tile_ref).failed) {
            refused += 1;
            continue;
        }
        const tile = nav.getTileByRef(tile_ref).?;
        for (&points, 0..) |*p, i| {
            p.* = if (i % 2 == 0 and tile.verts.len > 0)
                tile.verts[random.uintLessThan(usize, tile.verts.len)]
            else
                grid.point(random);
        }
        var query: integer.NavMeshQuery = .empty;
        try testing.expect(query.init(gpa, &nav, 128).succeeded);
        defer query.deinit(gpa);
        var sink: Sink = .{ .bytes = sink_bytes };
        runQueries(&query, &points, no_rays, &sink);
        queried += 1;
    }
    std.debug.print("skeptic damaged tiles: {d} refused by the converter, {d} by addTile, " ++
        "{d} taken and queried\n", .{ converter_refused, refused, queried });
    try testing.expect(queried > 200);
}
