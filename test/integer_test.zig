//! Tests of the integer variant against the float port, within the
//! tolerances of `doc/integer-design.md`: the float tiles of each level go
//! through the float port, their conversions through the variant, with the
//! same positions in whole millimetres, handed to the float port as metres.
//!
//! A discrete answer (which polygon, which status) can flip where a float
//! lands on the other side of an edge. A case is stable when the float port
//! gives the same discrete answer for its input and for the input moved by
//! one millimetre each way along x and z. The stable cases are compared;
//! a test fails when fewer than 80% of its cases are stable.

const std = @import("std");
const detour = @import("detour");
const support = @import("support.zig");

const integer = detour.integer;
const testing = std.testing;

const PolyRef = detour.PolyRef;
const Status = detour.Status;
const Fraction = integer.Fraction;

/// Millimetres per metre.
const scale = integer.convert.default_scale;

/// A position's tolerance per axis, in millimetres.
const tolerance: f64 = 2.0;

/// A height read from the detail surface: a slope turns a sideways
/// difference into a height difference.
const height_tolerance: f64 = 2.0 * tolerance;

/// The least share of a test's cases that must be stable.
const stable_share_min: f64 = 0.8;

/// The least number of cases a test runs.
const case_count_min = 1000;

/// Room for the results of one query.
const result_count_max = 256;

// ---------------------------------------------------------------------------
// Units
// ---------------------------------------------------------------------------

fn metresOf(mm: i64) f32 {
    return @floatCast(@as(f64, @floatFromInt(mm)) / scale);
}

fn metres(p: [3]i32) [3]f32 {
    return .{ metresOf(p[0]), metresOf(p[1]), metresOf(p[2]) };
}

fn millimetres(p: [3]f32) [3]i32 {
    return integer.convert.positionFromFloat(p, scale).?;
}

fn millimetresOf(m: f32) i32 {
    return @intFromFloat(@round(@as(f64, m) * scale));
}

/// How far, in millimetres, the integer `actual` lies from the float
/// `expected` along `axis`.
fn axisError(expected: [3]f32, actual: [3]i32, axis: usize) f64 {
    return @abs(@as(f64, @floatFromInt(actual[axis])) - @as(f64, expected[axis]) * scale);
}

/// Fails unless `actual` lies within `xz` millimetres of `expected` along x
/// and z, and within `y` along y.
fn expectNear(
    comptime what: []const u8,
    expected: [3]f32,
    actual: [3]i32,
    xz: f64,
    y: f64,
) !void {
    for (0..3) |axis| {
        const limit = if (axis == 1) y else xz;
        if (axisError(expected, actual, axis) <= limit) continue;
        std.debug.print("{s}, axis {d}: float {d:.3} mm, integer {d} mm, allowed {d}\n", .{
            what,
            axis,
            @as(f64, expected[axis]) * scale,
            actual[axis],
            limit,
        });
        return error.TestOutsideTolerance;
    }
}

/// Fails unless the integer length `actual` lies within `limit` millimetres
/// of the float length `expected`.
fn expectNearLength(comptime what: []const u8, expected: f32, actual: i64, limit: f64) !void {
    const err = @abs(@as(f64, @floatFromInt(actual)) - @as(f64, expected) * scale);
    if (err <= limit) return;
    std.debug.print("{s}: float {d:.3} mm, integer {d} mm, allowed {d}\n", .{
        what,
        @as(f64, expected) * scale,
        actual,
        limit,
    });
    return error.TestOutsideTolerance;
}

/// Fails unless the integer cost `actual`, in 1/256 millimetre, is within
/// 0.1% of the float cost `expected`, plus per polygon a millimetre and the
/// error of the point where the ray leaves it.
fn expectNearCost(expected: f32, actual: u64, polygon_count: usize) !void {
    const expected_mm = @as(f64, expected) * scale;
    const actual_mm = @as(f64, @floatFromInt(actual)) / 256.0;
    const per_polygon = 1.0 + 2.0 * tolerance;
    const limit = 0.001 * expected_mm + per_polygon * @as(f64, @floatFromInt(polygon_count)) +
        tolerance;
    if (@abs(actual_mm - expected_mm) <= limit) return;
    std.debug.print("cost: float {d:.3} mm, integer {d:.3} mm, allowed {d:.3}\n", .{
        expected_mm,
        actual_mm,
        limit,
    });
    return error.TestOutsideTolerance;
}

/// Fails unless the Q16.16 normal `actual` is within `limit` of the float
/// `expected` along each axis.
fn expectNearNormal(expected: [3]f32, actual: [3]i32, limit: f64) !void {
    for (0..3) |axis| {
        const value = @as(f64, @floatFromInt(actual[axis])) / 65536.0;
        if (@abs(value - expected[axis]) <= limit) continue;
        std.debug.print("normal, axis {d}: float {d:.6}, integer {d:.6}\n", .{
            axis,
            expected[axis],
            value,
        });
        return error.TestOutsideTolerance;
    }
}

fn statusBits(status: Status) u32 {
    return @bitCast(status);
}

/// The input moved by one millimetre each way along x and z.
const nudges = [4][3]i32{ .{ 1, 0, 0 }, .{ -1, 0, 0 }, .{ 0, 0, 1 }, .{ 0, 0, -1 } };

fn moved(p: [3]i32, by: [3]i32) [3]i32 {
    return .{ p[0] + by[0], p[1] + by[1], p[2] + by[2] };
}

/// The cases of one test: how many were stable and compared, how many
/// were not.
const Tally = struct {
    stable: usize = 0,
    unstable: usize = 0,

    fn count(self: *Tally, stable: bool) void {
        if (stable) self.stable += 1 else self.unstable += 1;
    }

    /// Fails unless the test ran enough cases and enough were stable.
    fn expectEnough(self: Tally, comptime what: []const u8) !void {
        const total = self.stable + self.unstable;
        const share = @as(f64, @floatFromInt(self.stable)) /
            @as(f64, @floatFromInt(@max(total, 1)));
        if (total >= case_count_min and share >= stable_share_min) return;
        std.debug.print("{s}: {d} cases, {d} stable\n", .{ what, total, self.stable });
        return error.TooFewStableCases;
    }
};

// ---------------------------------------------------------------------------
// Levels on both sides
// ---------------------------------------------------------------------------

/// A level's float mesh, its conversion and a query of each.
const Pair = struct {
    world: support.World,
    nav: integer.NavMesh,
    float_query: detour.NavMeshQuery,
    query: integer.NavMeshQuery,
    /// Whether the level's polygons differ in area and flags.
    varied: bool,

    /// Builds the level, converts every tile and readies both queries.
    /// `self` must not move afterwards. Free with `deinit`.
    fn init(
        self: *Pair,
        gpa: std.mem.Allocator,
        obj_text: []const u8,
        config: support.RecastConfig,
        options: support.LevelOptions,
    ) !void {
        try self.world.init(gpa, obj_text, config, options);
        errdefer self.world.deinit();
        self.varied = options.vary_polygons;
        try self.convertMesh(gpa, config.tile_size == 0);
        errdefer self.nav.deinit();
        self.float_query = .empty;
        if (self.float_query.init(gpa, &self.world.nav, 4096).failed) return error.OutOfMemory;
        errdefer self.float_query.deinit(gpa);
        self.query = .empty;
        if (self.query.init(gpa, &self.nav, 4096).failed) return error.OutOfMemory;
    }

    fn deinit(self: *Pair, gpa: std.mem.Allocator) void {
        self.query.deinit(gpa);
        self.float_query.deinit(gpa);
        self.nav.deinit();
        self.world.deinit();
        self.* = undefined;
    }

    /// Converts the float mesh's tiles, in the order of their entries, so
    /// that both meshes hand out the same references.
    fn convertMesh(self: *Pair, gpa: std.mem.Allocator, single: bool) !void {
        const float_nav = &self.world.nav;
        self.nav = .empty;
        if (single) {
            const data = try integer.convert.tileFromFloat(gpa, float_nav.tiles[0].data, scale);
            if (self.nav.initSingleTile(gpa, data, .{ .free_data = true }).failed) {
                gpa.free(data);
                return error.IntegerMeshRefused;
            }
            return;
        }
        const params = float_nav.getParams();
        const integer_params: integer.NavMeshParams = .{
            .origin = millimetres(params.origin),
            .tile_width = millimetresOf(params.tile_width),
            .tile_height = millimetresOf(params.tile_height),
            .tile_count_max = params.tile_count_max,
            .poly_count_max = params.poly_count_max,
        };
        if (self.nav.init(gpa, &integer_params).failed) return error.OutOfMemory;
        errdefer self.nav.deinit();
        for (float_nav.tiles) |*tile| {
            if (tile.header == null) continue;
            const data = try integer.convert.tileFromFloat(gpa, tile.data, scale);
            var ref: integer.TileRef = 0;
            if (self.nav.addTile(data, .{ .free_data = true }, 0, &ref).failed) {
                gpa.free(data);
                return error.IntegerMeshRefused;
            }
            try testing.expectEqual(float_nav.getTileRef(tile), ref);
        }
    }

    /// A place on the ground of the level, moved off it by up to `jitter`
    /// metres along each axis, in whole millimetres.
    fn groundPoint(self: *const Pair, random: std.Random, jitter: f32) [3]i32 {
        return millimetres(self.world.level.groundPoint(random, jitter));
    }

    /// The polygon the integer variant finds nearest `p` and the point on
    /// it, or null when it finds none or the float port finds another.
    fn startAt(self: *Pair, p: [3]i32, filters: *const Filters) ?Start {
        const half_extents = [3]i32{ 2000, 4000, 2000 };
        var ref: PolyRef = 0;
        var point: [3]i32 = p;
        _ = self.query.findNearestPoly(p, half_extents, &filters.int, &ref, &point, null);
        if (ref == 0) return null;
        var float_ref: PolyRef = 0;
        var float_point: [3]f32 = metres(p);
        _ = self.float_query.findNearestPoly(
            metres(p),
            metres(half_extents),
            &filters.float,
            &float_ref,
            &float_point,
            null,
        );
        if (float_ref != ref) return null;

        // A nearest point can lie on an edge, and once rounded a hair
        // outside the polygon on one side and inside on the other. Start
        // 20 mm further in, on the surface, where both sides agree the
        // point is inside.
        const found = self.nav.getTileAndPolyByRefUnsafe(ref);
        const poly = found.poly;
        if (poly.getType() == .off_mesh_connection) return null;
        var centre: [3]i64 = @splat(0);
        for (poly.verts[0..poly.vert_count]) |index| {
            for (0..3) |axis| centre[axis] += found.tile.verts[index][axis];
        }
        for (&centre) |*c| c.* = @divTrunc(c.*, poly.vert_count);
        const dx: f64 = @floatFromInt(centre[0] - point[0]);
        const dz: f64 = @floatFromInt(centre[2] - point[2]);
        const apart = @sqrt(dx * dx + dz * dz);
        const share = if (apart > 20.0) 20.0 / apart else 1.0;
        var inside: [3]i32 = .{
            point[0] + @as(i32, @intFromFloat(@round(dx * share))),
            point[1],
            point[2] + @as(i32, @intFromFloat(@round(dz * share))),
        };
        if (self.query.getPolyHeight(ref, inside, &inside[1]).failed) return null;
        var float_height: f32 = 0;
        if (self.float_query.getPolyHeight(ref, metres(inside), &float_height).failed) return null;
        return .{ .ref = ref, .pos = inside };
    }
};

/// A polygon and a point on it.
const Start = struct {
    ref: PolyRef,
    pos: [3]i32,
};

/// The same filter on both sides.
const Filters = struct {
    float: detour.QueryFilter,
    int: integer.QueryFilter,

    /// Area costs that are exact in both units, and one flag left out.
    const costs = [_]f32{ 1.0, 1.5, 2.0, 3.0, 1.25, 4.0 };

    fn init(varied: bool) Filters {
        var filters: Filters = .{ .float = .{}, .int = .{} };
        if (!varied) return filters;
        for (costs, 0..) |cost, area| {
            filters.float.area_cost[area] = cost;
            filters.int.area_cost[area] = @intFromFloat(cost * 65536.0);
        }
        filters.float.exclude_flags = 0x4;
        filters.int.exclude_flags = 0x4;
        return filters;
    }

    /// The default filter for even cases and, on a level whose polygons
    /// differ, the varied one for odd cases.
    fn forCase(pair: *const Pair, case: usize) Filters {
        return init(pair.varied and case % 2 == 1);
    }
};

/// A seeded random level on a grid of whole millimetres within 100 m of
/// the origin: rolling ground of 2 m squares with a few blocks standing on
/// it, as OBJ text. Free with `gpa.free`.
fn randomLevelObj(gpa: std.mem.Allocator, seed: u64) ![]u8 {
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(seed));
    const random = prng.random();
    var text: std.ArrayList(u8) = .empty;
    errdefer text.deinit(gpa);

    const side = 30;
    const step = 2000;
    for (0..side + 1) |z| {
        for (0..side + 1) |x| {
            try text.print(gpa, "v {d} {d} {d}\n", .{
                metresOf(@intCast(x * step)),
                metresOf(random.intRangeAtMost(i32, 0, 700)),
                metresOf(@intCast(z * step)),
            });
        }
    }
    for (0..side) |z| {
        for (0..side) |x| {
            const a = z * (side + 1) + x + 1;
            const b = a + 1;
            const c = a + side + 1;
            const d = c + 1;
            try text.print(gpa, "f {d} {d} {d}\nf {d} {d} {d}\n", .{ a, c, b, b, c, d });
        }
    }

    // Blocks: eight corners and their five visible faces each.
    var vertex_count: usize = (side + 1) * (side + 1);
    for (0..12) |_| {
        const x0 = random.intRangeAtMost(i32, 2, side - 6) * step +
            random.intRangeAtMost(i32, 0, 999);
        const z0 = random.intRangeAtMost(i32, 2, side - 6) * step +
            random.intRangeAtMost(i32, 0, 999);
        const x1 = x0 + random.intRangeAtMost(i32, 1500, 8000);
        const z1 = z0 + random.intRangeAtMost(i32, 1500, 8000);
        const y1 = random.intRangeAtMost(i32, 1200, 4000);
        for ([_]i32{ -500, y1 }) |y| {
            for ([_][2]i32{ .{ x0, z0 }, .{ x1, z0 }, .{ x1, z1 }, .{ x0, z1 } }) |corner| {
                try text.print(gpa, "v {d} {d} {d}\n", .{
                    metresOf(corner[0]),
                    metresOf(y),
                    metresOf(corner[1]),
                });
            }
        }
        const v = vertex_count + 1;
        const faces = [_][4]usize{
            .{ 4, 7, 6, 5 }, // top
            .{ 0, 1, 5, 4 },
            .{ 1, 2, 6, 5 },
            .{ 2, 3, 7, 6 },
            .{ 3, 0, 4, 7 },
        };
        for (faces) |face| {
            try text.print(gpa, "f {d} {d} {d}\nf {d} {d} {d}\n", .{
                v + face[0], v + face[1], v + face[2],
                v + face[0], v + face[2], v + face[3],
            });
        }
        vertex_count += 8;
    }
    return text.toOwnedSlice(gpa);
}

/// `obj_text` with every vertex moved by `dx` and `dz` metres. Free with
/// `gpa.free`.
fn shiftedObj(gpa: std.mem.Allocator, obj_text: []const u8, dx: f64, dz: f64) ![]u8 {
    var text: std.ArrayList(u8) = .empty;
    errdefer text.deinit(gpa);
    var lines = std.mem.splitScalar(u8, obj_text, '\n');
    while (lines.next()) |line| {
        if (!std.mem.startsWith(u8, line, "v ")) {
            try text.print(gpa, "{s}\n", .{line});
            continue;
        }
        var fields = std.mem.tokenizeAny(u8, line[2..], " \t\r");
        var v: [3]f64 = undefined;
        for (&v) |*c| c.* = try std.fmt.parseFloat(f64, fields.next() orelse return error.BadObj);
        try text.print(gpa, "v {d} {d} {d}\n", .{ v[0] + dx, v[1], v[2] + dz });
    }
    return text.toOwnedSlice(gpa);
}

/// The levels every comparison runs over. They are built once and read by
/// every test, so they come from the page allocator and stay for the whole
/// run.
var shared_pairs: ?[]Pair = null;

fn pairs() ![]Pair {
    if (shared_pairs) |list| return list;
    const gpa = std.heap.page_allocator;
    const random_level = try randomLevelObj(gpa, 0x6c6576656c);
    // RecastDemo puts this level 5 km out, where a float holds half a
    // millimetre; the comparisons are made within 1 km of the origin.
    const undulating = try shiftedObj(gpa, support.undulating_obj, -5000, -5000);
    const random_config: support.RecastConfig = blk: {
        var config = support.tiled_config;
        config.cell_size = 0.25;
        config.cell_height = 0.2;
        config.tile_size = 40;
        break :blk config;
    };
    const Spec = struct {
        obj: []const u8,
        config: support.RecastConfig,
        options: support.LevelOptions,
    };
    const specs = [_]Spec{
        .{
            .obj = support.nav_test_obj,
            .config = support.tiled_config,
            .options = .{ .off_mesh_connection_count = 8, .vary_polygons = true, .seed = 81 },
        },
        .{ .obj = support.dungeon_obj, .config = support.solo_config, .options = .{ .seed = 82 } },
        .{
            .obj = random_level,
            .config = random_config,
            .options = .{ .off_mesh_connection_count = 6, .vary_polygons = true, .seed = 83 },
        },
        .{
            .obj = undulating,
            .config = support.tiled_config,
            .options = .{ .odd_tile_cell_size = 0.2, .seed = 84 },
        },
    };
    const list = try gpa.alloc(Pair, specs.len);
    for (specs, list) |spec, *pair| try pair.init(gpa, spec.obj, spec.config, spec.options);
    shared_pairs = list;
    return list;
}

// ---------------------------------------------------------------------------
// Converter
// ---------------------------------------------------------------------------

test "integer tileFromFloat: vertices within half a millimetre, the same links, tree boxes hold" {
    var link_count: usize = 0;
    var link_limits_differ: usize = 0;
    for (try pairs(), 0..) |*pair, level| {
        errdefer std.debug.print("  level {d}\n", .{level});
        const float_nav = &pair.world.nav;
        for (float_nav.tiles, pair.nav.tiles, 0..) |*float_tile, *tile, t| {
            errdefer std.debug.print("  tile entry {d}\n", .{t});
            const float_header = float_tile.header orelse {
                try testing.expect(tile.header == null);
                continue;
            };
            const header = tile.header.?;
            try testing.expectEqual(float_header.poly_count, header.poly_count);
            try testing.expectEqual(float_header.vert_count, header.vert_count);
            try testing.expectEqual(float_header.detail_vert_count, header.detail_vert_count);
            try testing.expectEqual(float_header.bv_node_count, header.bv_node_count);
            try testing.expectEqual(
                float_header.off_mesh_connection_count,
                header.off_mesh_connection_count,
            );
            // The last two vertices of each off-mesh connection: adding the
            // tile moves its start onto the polygon nearest it, as a nearest
            // point, so it is compared as one.
            const ground_vert_count = float_tile.verts.len -
                2 * float_tile.off_mesh_connections.len;
            for (float_tile.verts, tile.verts, 0..) |float_vert, vert, v| {
                if (v < ground_vert_count) {
                    try expectNear("vertex", float_vert, vert, 0.5, 0.5);
                } else {
                    try expectNear(
                        "connection vertex",
                        float_vert,
                        vert,
                        tolerance,
                        height_tolerance,
                    );
                }
            }
            for (float_tile.detail_verts, tile.detail_verts) |float_vert, vert| {
                try expectNear("detail vertex", float_vert, vert, 0.5, 0.5);
            }
            for (float_tile.off_mesh_connections, tile.off_mesh_connections) |float_con, con| {
                try expectNear("connection start", float_con.pos[0], con.pos[0], 0.5, 0.5);
                try expectNear("connection end", float_con.pos[1], con.pos[1], 0.5, 0.5);
                try expectNearLength("connection radius", float_con.radius, con.radius, 0.5);
                try testing.expectEqual(float_con.poly, con.poly);
                try testing.expectEqual(float_con.flags, con.flags);
                try testing.expectEqual(float_con.side, con.side);
                try testing.expectEqual(float_con.user_id, con.user_id);
            }
            for (float_tile.bv_tree, tile.bv_tree, 0..) |float_node, node, n| {
                errdefer std.debug.print("  tree node {d}: float {any} {any}, " ++
                    "integer {any} {any}\n", .{
                    n,
                    float_node.bounds_min,
                    float_node.bounds_max,
                    node.bounds_min,
                    node.bounds_max,
                });
                try testing.expectEqual(float_node.index, node.index);
                for (0..3) |axis| {
                    try testing.expect(node.bounds_min[axis] <= float_node.bounds_min[axis] + 1);
                    try testing.expect(node.bounds_max[axis] + 1 >= float_node.bounds_max[axis]);
                }
            }

            // Polygons the same, and each polygon's links in the same order.
            for (float_tile.polys, tile.polys, 0..) |float_poly, poly, p| {
                errdefer std.debug.print("  polygon {d}\n", .{p});
                try testing.expectEqual(float_poly.verts, poly.verts);
                try testing.expectEqual(float_poly.neighbours, poly.neighbours);
                try testing.expectEqual(float_poly.flags, poly.flags);
                try testing.expectEqual(float_poly.vert_count, poly.vert_count);
                try testing.expectEqual(float_poly.area_and_type, poly.area_and_type);
                var float_link = float_poly.first_link;
                var link = poly.first_link;
                while (float_link != detour.nav_mesh.null_link) {
                    try testing.expect(link != integer.nav_mesh.null_link);
                    const a = float_tile.links[float_link];
                    const b = tile.links[link];
                    errdefer std.debug.print("  links: float {any}, integer {any}\n", .{ a, b });
                    try testing.expectEqual(a.ref, b.ref);
                    try testing.expectEqual(a.edge, b.edge);
                    try testing.expectEqual(a.side, b.side);
                    const min_step = @abs(@as(i32, a.bounds_min) - b.bounds_min);
                    const max_step = @abs(@as(i32, a.bounds_max) - b.bounds_max);
                    try testing.expect(min_step <= 1 and max_step <= 1);
                    if (min_step + max_step > 0) link_limits_differ += 1;
                    link_count += 1;
                    float_link = a.next;
                    link = b.next;
                }
                try testing.expectEqual(integer.nav_mesh.null_link, link);
            }
        }
    }
    try testing.expect(link_count > 1000);
    std.debug.print("converter: {d} links, {d} with limits a step apart\n", .{
        link_count,
        link_limits_differ,
    });
}

// ---------------------------------------------------------------------------
// findNearestPoly, queryPolygons
// ---------------------------------------------------------------------------

const extents_choices = [_][3]i32{
    .{ 2000, 4000, 2000 },
    .{ 500, 1000, 500 },
    .{ 300, 300, 300 },
    .{ 6000, 2000, 6000 },
};

const NearestAnswer = struct {
    status: u32,
    ref: PolyRef,
    over: bool,
    point: [3]f32,

    fn sameDiscrete(a: NearestAnswer, b: NearestAnswer) bool {
        return a.status == b.status and a.ref == b.ref and a.over == b.over;
    }
};

fn floatNearest(
    pair: *Pair,
    p: [3]i32,
    half_extents: [3]i32,
    filter: *const detour.QueryFilter,
) NearestAnswer {
    var answer: NearestAnswer = .{ .status = 0, .ref = 0, .over = false, .point = metres(p) };
    answer.status = statusBits(pair.float_query.findNearestPoly(
        metres(p),
        metres(half_extents),
        filter,
        &answer.ref,
        &answer.point,
        &answer.over,
    ));
    return answer;
}

test "integer findNearestPoly: the same polygon and over-flag, the point within tolerance" {
    var tally: Tally = .{};
    var border_cases: usize = 0;
    var widest_gap: f64 = 0;
    var ties: usize = 0;
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(0x6e656172));
    const random = prng.random();
    for (try pairs(), 0..) |*pair, level| {
        for (0..400) |case| {
            errdefer std.debug.print("  level {d}, case {d}\n", .{ level, case });
            const filters = Filters.forCase(pair, case);
            const p = pair.groundPoint(random, 1.5);
            const half_extents = extents_choices[random.uintLessThan(usize, extents_choices.len)];
            const expected = floatNearest(pair, p, half_extents, &filters.float);
            var stable = true;
            for (nudges) |by| {
                const near = floatNearest(pair, moved(p, by), half_extents, &filters.float);
                if (!expected.sameDiscrete(near)) stable = false;
            }
            tally.count(stable);
            if (!stable) continue;

            var ref: PolyRef = 0;
            var point: [3]i32 = p;
            var over = false;
            const status = pair.query.findNearestPoly(
                p,
                half_extents,
                &filters.int,
                &ref,
                &point,
                &over,
            );
            try testing.expectEqual(expected.status, statusBits(status));
            if (expected.ref != ref and
                try boxesDiffer(pair, p, half_extents, &filters) and
                try nearBorder(pair, if (ref != 0) ref else expected.ref, p, half_extents))
            {
                // The two trees' boxes differ by up to a cell at their
                // ends, so the polygons near the box's border that either
                // side looks at can differ, and with them the nearest.
                border_cases += 1;
                const border_ref = if (ref != 0) ref else expected.ref;
                const gap = borderGapCells(pair, border_ref, p, half_extents);
                widest_gap = @max(widest_gap, gap);
                continue;
            }
            if (expected.ref != ref and try equallyNear(pair, p, expected.ref, ref)) {
                // Two polygons as near: each side keeps the first it finds
                // nearest by its own rounding.
                ties += 1;
                continue;
            }
            try testing.expectEqual(expected.ref, ref);
            try testing.expectEqual(expected.over, over);
            if (ref != 0) {
                try expectNear("nearest point", expected.point, point, tolerance, height_tolerance);
            }
        }
    }
    try tally.expectEnough("findNearestPoly");
    std.debug.print("findNearestPoly: {d} stable cases, {d} at the box's border " ++
        "(at most {d:.2} cells off), {d} ties\n", .{
        tally.stable,
        border_cases,
        widest_gap,
        ties,
    });
    // Few cases are decided by a polygon at the border: at most 29 of 1598
    // stable cases over seeds 0 to 31 (seed 15). Ties: at most 2 (seed 0).
    try testing.expect(border_cases * 50 <= tally.stable);
    try testing.expect(ties * 500 <= tally.stable);
}

/// Collects the references `queryPolygons` hands back.
fn floatQueryPolygons(
    pair: *Pair,
    center: [3]i32,
    half_extents: [3]i32,
    filter: *const detour.QueryFilter,
    refs: []PolyRef,
) !struct { status: u32, count: usize } {
    var count: usize = 0;
    const status = pair.float_query.queryPolygons(
        metres(center),
        metres(half_extents),
        filter,
        refs,
        &count,
    );
    return .{ .status = statusBits(status), .count = count };
}

/// How far, in millimetres, the tree box of polygon `ref` of the integer
/// mesh lies outside the box around `center`: positive when apart,
/// negative when the boxes overlap, by the overlap along the axis where it
/// is least. The tree box holds the polygon's geometry and the float tile's
/// box for it, so a polygon either side finds lies within the quantizing of
/// the query box from it. A tile without a tree gives the polygon's own box.
fn boxGap(pair: *Pair, ref: PolyRef, center: [3]i32, half_extents: [3]i32) f64 {
    const found = pair.nav.getTileAndPolyByRefUnsafe(ref);
    const header = found.tile.header.?;
    const index = (@intFromPtr(found.poly) - @intFromPtr(found.tile.polys.ptr)) /
        @sizeOf(@TypeOf(found.poly.*));
    var low: [3]f64 = @splat(std.math.inf(f64));
    var high: [3]f64 = @splat(-std.math.inf(f64));
    const leaf = for (found.tile.bv_tree) |node| {
        if (node.index == index) break node;
    } else null;
    if (leaf) |node| {
        const cell: f64 = @floatFromInt(header.bv_cell_size);
        for (0..3) |axis| {
            const origin: f64 = @floatFromInt(header.bounds_min[axis]);
            low[axis] = origin + @as(f64, @floatFromInt(node.bounds_min[axis])) * cell;
            const cell_max: f64 = @floatFromInt(node.bounds_max[axis]);
            high[axis] = origin + (cell_max + 1) * cell;
        }
    } else {
        for (found.poly.verts[0..found.poly.vert_count]) |vert_index| {
            for (0..3) |axis| {
                const value: f64 = @floatFromInt(found.tile.verts[vert_index][axis]);
                low[axis] = @min(low[axis], value);
                high[axis] = @max(high[axis], value);
            }
        }
    }
    // Both sides clamp the query box to the tile's bounds before they look
    // in the tree, so a box past the tile still meets its outermost cells.
    var gap: f64 = -std.math.inf(f64);
    for (0..3) |axis| {
        const tile_low: f64 = @floatFromInt(header.bounds_min[axis]);
        const tile_high: f64 = @floatFromInt(header.bounds_max[axis]);
        const query_low: f64 = @floatFromInt(center[axis] - half_extents[axis]);
        const query_high: f64 = @floatFromInt(center[axis] + half_extents[axis]);
        const box_low = std.math.clamp(query_low, tile_low, tile_high);
        const box_high = std.math.clamp(query_high, tile_low, tile_high);
        const apart = @max(box_low - high[axis], low[axis] - box_high);
        gap = @max(gap, apart);
    }
    return gap;
}

/// Whether polygons `a` and `b` of the float mesh lie equally near `p` as
/// `findNearestPoly` measures it, within a position's tolerance.
fn equallyNear(pair: *Pair, p: [3]i32, a: PolyRef, b: PolyRef) !bool {
    if (a == 0 or b == 0) return false;
    const header = pair.world.nav.getTileAndPolyByRefUnsafe(a).tile.header.?;
    const climb: f64 = header.walkable_climb;
    var distances: [2]f64 = undefined;
    for ([_]PolyRef{ a, b }, &distances) |ref, *d| {
        var closest: [3]f32 = undefined;
        var over = false;
        _ = pair.float_query.closestPointOnPoly(ref, metres(p), &closest, &over);
        const centre = metres(p);
        if (over) {
            d.* = @max(0, @abs(@as(f64, centre[1]) - closest[1]) - climb) * scale;
        } else {
            d.* = distance(centre, closest) * scale;
        }
    }
    return @abs(distances[0] - distances[1]) <= tolerance;
}

/// Whether the two sides find other polygons in the box around `center`.
fn boxesDiffer(pair: *Pair, center: [3]i32, half_extents: [3]i32, filters: *const Filters) !bool {
    var float_refs: [result_count_max]PolyRef = undefined;
    var refs: [result_count_max]PolyRef = undefined;
    var float_count: usize = 0;
    _ = pair.float_query.queryPolygons(
        metres(center),
        metres(half_extents),
        &filters.float,
        &float_refs,
        &float_count,
    );
    var count: usize = 0;
    _ = pair.query.queryPolygons(center, half_extents, &filters.int, &refs, &count);
    if (float_count != count) return true;
    std.mem.sort(PolyRef, float_refs[0..count], {}, std.sort.asc(PolyRef));
    std.mem.sort(PolyRef, refs[0..count], {}, std.sort.asc(PolyRef));
    return !std.mem.eql(PolyRef, float_refs[0..count], refs[0..count]);
}

/// Whether polygon `ref`'s tree box lies within two tree cells of the
/// border of the box around `center`.
fn nearBorder(pair: *Pair, ref: PolyRef, center: [3]i32, half_extents: [3]i32) !bool {
    return borderGapCells(pair, ref, center, half_extents) <= border_cells_max;
}

/// How far polygon `ref`'s tree box lies from the border of the box around
/// `center`, in tree cells, a position's tolerance taken off.
fn borderGapCells(pair: *Pair, ref: PolyRef, center: [3]i32, half_extents: [3]i32) f64 {
    const header = pair.nav.getTileAndPolyByRefUnsafe(ref).tile.header.?;
    const cell_mm: f64 = @floatFromInt(header.bv_cell_size);
    return @max(0, @abs(boxGap(pair, ref, center, half_extents)) - tolerance) / cell_mm;
}

/// How far from the query box's border, in tree cells, a polygon found by
/// one side only may lie: the query box is widened by up to two cells on
/// either side. The widest measured over seeds 0 to 6 is 1.97 cells.
const border_cells_max = 2.0;

test "integer queryPolygons: the same polygons but those whose tree box lies at the border" {
    const gpa = testing.allocator;
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(0x71756572));
    const random = prng.random();
    var float_refs: [result_count_max]PolyRef = undefined;
    var refs: [result_count_max]PolyRef = undefined;
    var compared: usize = 0;
    var border_differences: usize = 0;
    var border_cases: usize = 0;
    var widest_gap: f64 = 0;
    for (try pairs(), 0..) |*pair, level| {
        for (0..300) |case| {
            errdefer std.debug.print("  level {d}, case {d}\n", .{ level, case });
            const filters = Filters.forCase(pair, case);
            const center = pair.groundPoint(random, 3.0);
            const half_extents = extents_choices[random.uintLessThan(usize, extents_choices.len)];
            const expected = try floatQueryPolygons(
                pair,
                center,
                half_extents,
                &filters.float,
                &float_refs,
            );
            var count: usize = 0;
            const status =
                pair.query.queryPolygons(center, half_extents, &filters.int, &refs, &count);
            try testing.expectEqual(expected.status, statusBits(status));
            if (expected.count == float_refs.len or count == refs.len) continue;
            compared += 1;

            var seen: std.AutoHashMapUnmanaged(PolyRef, u2) = .empty;
            defer seen.deinit(gpa);
            for (float_refs[0..expected.count]) |ref| {
                (try seen.getOrPutValue(gpa, ref, 0)).value_ptr.* |= 1;
            }
            for (refs[0..count]) |ref| (try seen.getOrPutValue(gpa, ref, 0)).value_ptr.* |= 2;
            var entries = seen.iterator();
            var differs = false;
            defer border_cases += @intFromBool(differs);
            while (entries.next()) |entry| {
                if (entry.value_ptr.* == 3) continue;
                differs = true;
                if (!try nearBorder(pair, entry.key_ptr.*, center, half_extents)) {
                    const gap = boxGap(pair, entry.key_ptr.*, center, half_extents);
                    std.debug.print("polygon {d}, found by {s} only, {d:.1} mm off\n", .{
                        entry.key_ptr.*,
                        if (entry.value_ptr.* == 1) "the float port" else "the integer variant",
                        gap,
                    });
                    return error.TestUnexpectedResult;
                }
                border_differences += 1;
                widest_gap = @max(
                    widest_gap,
                    borderGapCells(pair, entry.key_ptr.*, center, half_extents),
                );
            }
        }
    }
    try testing.expect(compared >= case_count_min);
    std.debug.print("queryPolygons: {d} cases, {d} polygons differ at the border " ++
        "(at most {d:.2} cells off), in {d} cases\n", .{
        compared,
        border_differences,
        widest_gap,
        border_cases,
    });
    // Over seeds 0 to 31, at most 106 polygons differ (seed 11), in at most
    // 50 of 1200 cases (seed 23).
    try testing.expect(border_cases * 20 <= compared);
    try testing.expect(border_differences * 10 <= compared);
}

// ---------------------------------------------------------------------------
// closestPointOnPoly, closestPointOnPolyBoundary, getPolyHeight
// ---------------------------------------------------------------------------

const ClosestAnswer = struct {
    closest_status: u32,
    over: bool,
    closest: [3]f32,
    boundary_status: u32,
    boundary: [3]f32,
    height_status: u32,
    height: f32,

    fn sameDiscrete(a: ClosestAnswer, b: ClosestAnswer) bool {
        return a.closest_status == b.closest_status and a.over == b.over and
            a.boundary_status == b.boundary_status and a.height_status == b.height_status;
    }
};

fn floatClosest(pair: *Pair, ref: PolyRef, p: [3]i32) ClosestAnswer {
    var answer: ClosestAnswer = undefined;
    answer.over = false;
    answer.closest = @splat(0);
    answer.boundary = @splat(0);
    answer.height = 0;
    answer.closest_status = statusBits(
        pair.float_query.closestPointOnPoly(ref, metres(p), &answer.closest, &answer.over),
    );
    answer.boundary_status = statusBits(
        pair.float_query.closestPointOnPolyBoundary(ref, metres(p), &answer.boundary),
    );
    const height_status = pair.float_query.getPolyHeight(ref, metres(p), &answer.height);
    answer.height_status = statusBits(height_status);
    return answer;
}

test "integer closestPointOnPoly, closestPointOnPolyBoundary, getPolyHeight: within tolerance" {
    var tally: Tally = .{};
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(0x636c6f73));
    const random = prng.random();
    for (try pairs(), 0..) |*pair, level| {
        for (0..400) |case| {
            errdefer std.debug.print("  level {d}, case {d}\n", .{ level, case });
            const filters = Filters.init(false);
            const p = pair.groundPoint(random, 2.0);
            const near = floatNearest(pair, p, .{ 3000, 5000, 3000 }, &filters.float);
            if (near.ref == 0) continue;
            // Now and then a polygon the point is not near, or no polygon.
            const ref = switch (random.uintLessThan(u8, 10)) {
                0 => 0,
                else => near.ref,
            };
            const expected = floatClosest(pair, ref, p);
            var stable = true;
            for (nudges) |by| {
                if (!expected.sameDiscrete(floatClosest(pair, ref, moved(p, by)))) stable = false;
            }
            tally.count(stable);
            if (!stable) continue;

            var closest: [3]i32 = @splat(0);
            var over = false;
            const closest_status = pair.query.closestPointOnPoly(ref, p, &closest, &over);
            try testing.expectEqual(expected.closest_status, statusBits(closest_status));
            if (closest_status.succeeded) {
                try testing.expectEqual(expected.over, over);
                try expectNear(
                    "closest point",
                    expected.closest,
                    closest,
                    tolerance,
                    height_tolerance,
                );
            }
            var boundary: [3]i32 = @splat(0);
            const boundary_status = pair.query.closestPointOnPolyBoundary(ref, p, &boundary);
            try testing.expectEqual(expected.boundary_status, statusBits(boundary_status));
            if (boundary_status.succeeded) {
                try expectNear(
                    "boundary point",
                    expected.boundary,
                    boundary,
                    tolerance,
                    height_tolerance,
                );
            }
            var height: i32 = 0;
            const height_status = pair.query.getPolyHeight(ref, p, &height);
            try testing.expectEqual(expected.height_status, statusBits(height_status));
            if (height_status.succeeded) {
                try expectNearLength("height", expected.height, height, height_tolerance);
            }
        }
    }
    try tally.expectEnough("closest points");
}

// ---------------------------------------------------------------------------
// findPath, the sliced search, findStraightPath
// ---------------------------------------------------------------------------

/// The two ends of a portal from `from` to `to` on the float mesh, as the
/// path search takes them, or null when the two are not linked.
fn floatPortal(nav: *const detour.NavMesh, from: PolyRef, to: PolyRef) ?[2][3]f32 {
    const a = nav.getTileAndPolyByRefUnsafe(from);
    const b = nav.getTileAndPolyByRefUnsafe(to);
    var link_index = a.poly.first_link;
    while (link_index != detour.nav_mesh.null_link) {
        const link = a.tile.links[link_index];
        link_index = link.next;
        if (link.ref != to) continue;
        if (a.poly.getType() == .off_mesh_connection) {
            const v = a.tile.verts[a.poly.verts[link.edge]];
            return .{ v, v };
        }
        if (b.poly.getType() == .off_mesh_connection) {
            var back = b.poly.first_link;
            while (back != detour.nav_mesh.null_link) {
                const other = b.tile.links[back];
                back = other.next;
                if (other.ref != from) continue;
                const v = b.tile.verts[b.poly.verts[other.edge]];
                return .{ v, v };
            }
            return null;
        }
        const v0 = a.tile.verts[a.poly.verts[link.edge]];
        const v1 = a.tile.verts[a.poly.verts[(link.edge + 1) % a.poly.vert_count]];
        if (link.side != 0xff and (link.bounds_min != 0 or link.bounds_max != 255)) {
            const s = 1.0 / 255.0;
            const tmin = @as(f32, @floatFromInt(link.bounds_min)) * s;
            const tmax = @as(f32, @floatFromInt(link.bounds_max)) * s;
            return .{ lerp(v0, v1, tmin), lerp(v0, v1, tmax) };
        }
        return .{ v0, v1 };
    }
    return null;
}

fn lerp(a: [3]f32, b: [3]f32, t: f32) [3]f32 {
    return .{ a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t };
}

fn distance(a: [3]f32, b: [3]f32) f64 {
    var sum: f64 = 0;
    for (0..3) |axis| {
        const d = @as(f64, a[axis]) - b[axis];
        sum += d * d;
    }
    return @sqrt(sum);
}

/// What the path search's cost of walking `corridor` from `start` to `end`
/// is, on the float mesh with the float filter: through the middle of each
/// portal, each step at its polygon's area cost. Null when two polygons of
/// the corridor are not linked.
fn corridorCost(
    pair: *Pair,
    filter: *const detour.QueryFilter,
    start: [3]f32,
    end: [3]f32,
    corridor: []const PolyRef,
) ?f64 {
    const nav = &pair.world.nav;
    var cost: f64 = 0;
    var at = start;
    for (corridor, 0..) |ref, i| {
        const area = nav.getTileAndPolyByRefUnsafe(ref).poly.getArea();
        const next = if (i + 1 < corridor.len) blk: {
            const portal = floatPortal(nav, ref, corridor[i + 1]) orelse return null;
            break :blk lerp(portal[0], portal[1], 0.5);
        } else end;
        cost += distance(at, next) * filter.area_cost[area];
        at = next;
    }
    return cost;
}

/// How far apart, in millimetres, the distances from `target` to the
/// nearest node of polygon `float_last` and to that of `last` lie, in the
/// float port's node pool after a search: the two partial results' ends.
fn nearestGap(pair: *Pair, target: [3]f32, float_last: PolyRef, last: PolyRef) !f64 {
    var reach: [2]f64 = undefined;
    for ([_]PolyRef{ float_last, last }, &reach) |ref, *d| {
        var nodes: [4]*detour.Node = undefined;
        const count = pair.float_query.node_pool.findNodes(ref, &nodes);
        if (count == 0) return error.TestUnexpectedResult;
        d.* = std.math.inf(f64);
        for (nodes[0..count]) |node| d.* = @min(d.*, distance(node.pos, target) * scale);
    }
    return @abs(reach[1] - reach[0]);
}

const PathAnswer = struct {
    status: u32,
    path: [result_count_max]PolyRef,
    count: usize,

    fn sameDiscrete(a: *const PathAnswer, b: *const PathAnswer) bool {
        return a.status == b.status and
            std.mem.eql(PolyRef, a.path[0..a.count], b.path[0..b.count]);
    }
};

/// How a path is searched for: all at once or sliced, in steps of a few
/// iterations.
const PathMode = enum { whole, sliced, sliced_any_angle };

fn floatPath(
    pair: *Pair,
    mode: PathMode,
    start: Start,
    end: Start,
    by: [3]i32,
    filter: *const detour.QueryFilter,
    answer: *PathAnswer,
) void {
    const from = metres(moved(start.pos, by));
    const to = metres(moved(end.pos, by));
    answer.count = 0;
    if (mode == .whole) {
        answer.status = statusBits(pair.float_query.findPath(
            start.ref,
            end.ref,
            from,
            to,
            filter,
            &answer.path,
            &answer.count,
        ));
        return;
    }
    var status = pair.float_query.initSlicedFindPath(start.ref, end.ref, from, to, filter, .{
        .any_angle = mode == .sliced_any_angle,
    });
    while (status.in_progress) status = pair.float_query.updateSlicedFindPath(13, null);
    answer.status = statusBits(status);
    if (status.failed) return;
    const final = pair.float_query.finalizeSlicedFindPath(&answer.path, &answer.count);
    answer.status = statusBits(final);
}

fn integerPath(
    pair: *Pair,
    mode: PathMode,
    start: Start,
    end: Start,
    filter: *const integer.QueryFilter,
    answer: *PathAnswer,
) void {
    answer.count = 0;
    if (mode == .whole) {
        answer.status = statusBits(pair.query.findPath(
            start.ref,
            end.ref,
            start.pos,
            end.pos,
            filter,
            &answer.path,
            &answer.count,
        ));
        return;
    }
    var status = pair.query.initSlicedFindPath(start.ref, end.ref, start.pos, end.pos, filter, .{
        .any_angle = mode == .sliced_any_angle,
    });
    while (status.in_progress) status = pair.query.updateSlicedFindPath(13, null);
    answer.status = statusBits(status);
    if (status.failed) return;
    answer.status = statusBits(pair.query.finalizeSlicedFindPath(&answer.path, &answer.count));
}

test "integer findPath and the sliced search: the same status, the same corridor or one as cheap" {
    var tally: Tally = .{};
    var same_corridor: usize = 0;
    var other_ends: usize = 0;
    var widest_end_gap: f64 = 0;
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(0x70617468));
    const random = prng.random();
    var expected: PathAnswer = undefined;
    var near: PathAnswer = undefined;
    var actual: PathAnswer = undefined;
    for (try pairs(), 0..) |*pair, level| {
        var case: usize = 0;
        var tries: usize = 0;
        while (case < 300 and tries < 3000) : (tries += 1) {
            errdefer std.debug.print("  level {d}, case {d}\n", .{ level, case });
            const filters = Filters.forCase(pair, case);
            const start = pair.startAt(pair.groundPoint(random, 0.3), &filters) orelse continue;
            const end = pair.startAt(pair.groundPoint(random, 0.3), &filters) orelse continue;
            const mode: PathMode = @fromBackingInt(@as(u2, @intCast(case % 3)));
            case += 1;

            floatPath(pair, mode, start, end, @splat(0), &filters.float, &expected);
            var stable = true;
            for (nudges) |by| {
                floatPath(pair, mode, start, end, by, &filters.float, &near);
                if (!expected.sameDiscrete(&near)) stable = false;
            }
            tally.count(stable);
            if (!stable) continue;

            integerPath(pair, mode, start, end, &filters.int, &actual);
            try testing.expectEqual(expected.status, actual.status);
            if (expected.sameDiscrete(&actual)) {
                same_corridor += 1;
                continue;
            }
            const status: Status = @bitCast(expected.status);
            const float_last = expected.path[expected.count -| 1];
            const last = actual.path[actual.count -| 1];
            if (status.partial_result and float_last != last) {
                // A partial result leads to the node nearest the end. Two
                // sides that stop at other polygons found two nodes as near
                // within rounding, as two corridors can cost the same: the
                // float port must have reached both, and neither corridor's
                // cost says anything of the other's.
                floatPath(pair, mode, start, end, @splat(0), &filters.float, &near);
                const gap = try nearestGap(pair, metres(end.pos), float_last, last);
                if (gap > 2 * tolerance) {
                    std.debug.print("partial results: their ends lie {d:.3} mm apart " ++
                        "in distance to the target, mode {t}\n", .{ gap, mode });
                    return error.TestOutsideTolerance;
                }
                widest_end_gap = @max(widest_end_gap, gap);
                other_ends += 1;
                continue;
            }
            // Another corridor: it must cost the same, by one evaluator.
            const from = metres(start.pos);
            const to = metres(end.pos);
            const expected_path = expected.path[0..expected.count];
            const actual_path = actual.path[0..actual.count];
            const expected_cost = corridorCost(pair, &filters.float, from, to, expected_path) orelse
                return error.TestUnexpectedResult;
            const actual_cost = corridorCost(pair, &filters.float, from, to, actual_path) orelse
                return error.TestUnexpectedResult;
            const polygons = @max(expected.count, actual.count);
            const limit = 0.001 * expected_cost + 0.001 * @as(f64, @floatFromInt(polygons));
            // The any-angle search shortcuts by rays, whose corridors cost
            // what the shortcut walks, not the portals' middles.
            if (mode != .sliced_any_angle and @abs(actual_cost - expected_cost) > limit) {
                std.debug.print("corridor costs: float {d:.4} m over {d}, " ++
                    "integer {d:.4} m over {d}, status 0x{x}, mode {t}\n", .{
                    expected_cost,
                    expected.count,
                    actual_cost,
                    actual.count,
                    expected.status,
                    mode,
                });
                return error.TestOutsideTolerance;
            }
        }
    }
    try tally.expectEnough("findPath");
    std.debug.print("findPath: {d} stable cases, {d} the same corridor, {d} partial results " ++
        "ending elsewhere (at most {d:.3} mm apart)\n", .{
        tally.stable,
        same_corridor,
        other_ends,
        widest_end_gap,
    });
    // Nearly every stable case finds the very same corridor: at most 2 of
    // about 1200 do not over seeds 0 to 31 (seeds 6, 10, 20 and 29), each a
    // partial result ending elsewhere.
    if ((tally.stable - same_corridor) * 200 > tally.stable) {
        std.debug.print("findPath: {d} of {d} stable cases found the same corridor\n", .{
            same_corridor,
            tally.stable,
        });
        return error.TooFewSameCorridors;
    }
}

/// The xz distance, in millimetres, from the integer point `p` to the
/// float polyline `line`.
fn distanceToLine(p: [3]i32, line: []const [3]f32) f64 {
    const px = @as(f64, @floatFromInt(p[0]));
    const pz = @as(f64, @floatFromInt(p[2]));
    var best = std.math.inf(f64);
    for (line, 0..) |a, i| {
        const b = if (i + 1 < line.len) line[i + 1] else a;
        const ax = @as(f64, a[0]) * scale;
        const az = @as(f64, a[2]) * scale;
        const dx = @as(f64, b[0]) * scale - ax;
        const dz = @as(f64, b[2]) * scale - az;
        const len2 = dx * dx + dz * dz;
        var t: f64 = 0;
        if (len2 > 0) t = std.math.clamp(((px - ax) * dx + (pz - az) * dz) / len2, 0, 1);
        const qx = ax + dx * t - px;
        const qz = az + dz * t - pz;
        best = @min(best, @sqrt(qx * qx + qz * qz));
    }
    return best;
}

/// The distance in the xz plane between segments `a0`-`a1` and `b0`-`b1`,
/// in millimetres.
fn segmentDistance(a0: [2]f64, a1: [2]f64, b0: [2]f64, b1: [2]f64) f64 {
    const cross = struct {
        fn f(o: [2]f64, p: [2]f64, q: [2]f64) f64 {
            return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0]);
        }
    }.f;
    const d1 = cross(a0, a1, b0);
    const d2 = cross(a0, a1, b1);
    const d3 = cross(b0, b1, a0);
    const d4 = cross(b0, b1, a1);
    if (((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and
        ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0))) return 0;
    const pointToSegment = struct {
        fn f(p: [2]f64, s0: [2]f64, s1: [2]f64) f64 {
            const dx = s1[0] - s0[0];
            const dz = s1[1] - s0[1];
            const len2 = dx * dx + dz * dz;
            var t: f64 = 0;
            const along = (p[0] - s0[0]) * dx + (p[1] - s0[1]) * dz;
            if (len2 > 0) t = std.math.clamp(along / len2, 0, 1);
            const qx = s0[0] + dx * t - p[0];
            const qz = s0[1] + dz * t - p[1];
            return @sqrt(qx * qx + qz * qz);
        }
    }.f;
    return @min(
        @min(pointToSegment(a0, b0, b1), pointToSegment(a1, b0, b1)),
        @min(pointToSegment(b0, a0, a1), pointToSegment(b1, a0, a1)),
    );
}

/// The length in the xz plane of the path through `points`, in
/// millimetres.
fn pathLength(points: []const [2]f64) f64 {
    var length: f64 = 0;
    for (points[0..points.len -| 1], points[1..]) |a, b| {
        length += std.math.hypot(b[0] - a[0], b[1] - a[1]);
    }
    return length;
}

/// The xz plane coordinates of each point, in millimetres.
fn planeOf(comptime T: type, points: []const [3]T, out: [][2]f64) [][2]f64 {
    for (points, out[0..points.len]) |p, *q| {
        q.* = if (T == f32)
            .{ @as(f64, p[0]) * scale, @as(f64, p[2]) * scale }
        else
            .{ @floatFromInt(p[0]), @floatFromInt(p[2]) };
    }
    return out[0..points.len];
}

/// How far the corridor's portal ends can lie from the float port's: a
/// 255th of the edge for each link whose limits are a step apart from the
/// float link's, as the converter may round an exact half the other way.
fn linkLimitSlack(pair: *Pair, corridor: []const PolyRef) f64 {
    var slack: f64 = 0;
    for (corridor[0..corridor.len -| 1], corridor[1..]) |from, to| {
        const a = pair.nav.getTileAndPolyByRefUnsafe(from);
        const float_a = pair.world.nav.getTileAndPolyByRefUnsafe(from);
        var k = a.poly.first_link;
        while (k != detour.nav_mesh.null_link) : (k = a.tile.links[k].next) {
            const link = a.tile.links[k];
            if (link.ref != to) continue;
            const float_link = float_a.tile.links[k];
            if (float_link.bounds_min == link.bounds_min and
                float_link.bounds_max == link.bounds_max) continue;
            const v0 = a.tile.verts[a.poly.verts[link.edge]];
            const v1 = a.tile.verts[a.poly.verts[(link.edge + 1) % a.poly.vert_count]];
            const length = @sqrt(@as(f64, @floatFromInt(integer.common.vdist2dSqr(v0, v1))));
            slack = @max(slack, length / 255.0);
        }
    }
    return slack;
}

/// Whether the path through `points` passes within tolerance of each
/// portal between two ground polygons of `corridor` on the integer mesh,
/// in the xz plane. A portal across a tile border is the part of the edge
/// its link covers.
fn throughPortals(pair: *Pair, corridor: []const PolyRef, points: []const [2]f64) bool {
    for (corridor[0..corridor.len -| 1], corridor[1..]) |from, to| {
        const a = pair.nav.getTileAndPolyByRefUnsafe(from);
        const b = pair.nav.getTileAndPolyByRefUnsafe(to);
        if (a.poly.getType() != .ground or b.poly.getType() != .ground) continue;
        var link_index = a.poly.first_link;
        const link = while (link_index != detour.nav_mesh.null_link) {
            const candidate = a.tile.links[link_index];
            if (candidate.ref == to) break candidate;
            link_index = candidate.next;
        } else return false;
        const v0 = a.tile.verts[a.poly.verts[link.edge]];
        const v1 = a.tile.verts[a.poly.verts[(link.edge + 1) % a.poly.vert_count]];
        var low: f64 = 0;
        var high: f64 = 1;
        if (link.side != 0xff and (link.bounds_min != 0 or link.bounds_max != 255)) {
            low = @as(f64, @floatFromInt(link.bounds_min)) / 255.0;
            high = @as(f64, @floatFromInt(link.bounds_max)) / 255.0;
        }
        var ends: [2][2]f64 = undefined;
        for (&ends, [_]f64{ low, high }) |*end, t| {
            for (end, [_]usize{ 0, 2 }) |*coord, axis| {
                const from_coord: f64 = @floatFromInt(v0[axis]);
                const to_coord: f64 = @floatFromInt(v1[axis]);
                coord.* = from_coord + (to_coord - from_coord) * t;
            }
        }
        var nearest = std.math.inf(f64);
        for (points[0..points.len -| 1], points[1..]) |p, q| {
            nearest = @min(nearest, segmentDistance(ends[0], ends[1], p, q));
        }
        if (nearest > tolerance) return false;
    }
    return true;
}

/// Whether the integer path `corners` passes every portal of `corridor`
/// and is no longer than the float path `float_corners`, both within
/// tolerance.
fn shorterThroughCorridor(
    pair: *Pair,
    filter: *const detour.QueryFilter,
    corridor: []const PolyRef,
    float_corners: []const [3]f32,
    corners: []const [3]i32,
    refs: []const PolyRef,
) bool {
    var float_plane: [result_count_max][2]f64 = undefined;
    var plane: [result_count_max][2]f64 = undefined;
    const float_points = planeOf(f32, float_corners, &float_plane);
    const points = planeOf(i32, corners, &plane);
    if (!throughPortals(pair, corridor, points)) return false;
    // The same start and end: a path cut short also passes the portals.
    for ([_][2]usize{ .{ 0, 0 }, .{ float_points.len - 1, points.len - 1 } }) |ends| {
        const f = float_points[ends[0]];
        const p = points[ends[1]];
        if (@abs(f[0] - p[0]) > tolerance or @abs(f[1] - p[1]) > tolerance) return false;
    }
    if (pathLength(points) > pathLength(float_points) + tolerance) return false;
    // Every leg walkable: the float port's ray from each corner to the next
    // meets no wall, apart from grazing a float corner on the leg.
    for (corners[0 .. corners.len - 1], corners[1..], refs[0 .. corners.len - 1]) |a, b, ref| {
        const found = pair.world.nav.getTileAndPolyByRefUnsafe(ref);
        if (found.poly.getType() == .off_mesh_connection) continue;
        if (!walkable(pair, filter, ref, metres(a), metres(b), float_corners)) return false;
    }
    return true;
}

/// Whether the float port's ray from `a` to `b` meets no wall. A ray that
/// runs through a vertex, such as a corner of either path, can stop on it,
/// and a ray that runs along a wall can stop where it meets it; the check
/// then goes on from just past the vertex or the wall's far end, on the
/// same floor, and fails where no floor is there.
fn walkable(
    pair: *Pair,
    filter: *const detour.QueryFilter,
    start_ref: PolyRef,
    a: [3]f32,
    b: [3]f32,
    float_corners: []const [3]f32,
) bool {
    const V = @Vector(3, f32);
    const step_past: f32 = 0.05;
    var visited: [result_count_max]PolyRef = undefined;
    var ref = start_ref;
    var from: V = a;
    for (0..result_count_max) |_| {
        var hit: detour.RaycastHit = .{
            .t = 0,
            .hit_normal = @splat(0),
            .hit_edge_index = 0,
            .path = &visited,
            .path_count = 0,
            .path_cost = 0,
        };
        const status = pair.float_query.raycast(ref, from, b, filter, .{}, &hit, 0);
        if (status.failed) return false;
        if (hit.t == std.math.floatMax(f32)) return true;
        const at = from + @as(V, @splat(hit.t)) * (@as(V, b) - from);
        // A corner stands at a vertex, where the ray can stop on the wall.
        const short: f64 = @max(@abs(b[0] - at[0]), @abs(b[2] - at[2])) * scale;
        if (short <= tolerance) return true;
        const corner: V = grazed(pair, &hit, at, a, b, float_corners) orelse return false;
        const rest = @as(V, b) - corner;
        const rest_length = @sqrt(rest[0] * rest[0] + rest[2] * rest[2]);
        if (rest_length <= step_past) return true;
        const past: [3]f32 = corner + @as(V, @splat(step_past / rest_length)) * rest;
        ref = 0;
        var point = past;
        const reach: [3]f32 = .{ 0.01, 0.5, 0.01 };
        _ = pair.float_query.findNearestPoly(past, reach, filter, &ref, &point, null);
        if (ref == 0) return false;
        from = point;
    }
    return false;
}

/// What the float port's ray from `a` to `b`, stopped at `at`, grazed
/// rather than met: the vertex it passes through, a corner of the float
/// path, the leg's own start or an end of the wall it stopped on, or the
/// far end of a wall it runs along. Null when it met a wall.
fn grazed(
    pair: *Pair,
    hit: *const detour.RaycastHit,
    at: [3]f32,
    a: [3]f32,
    b: [3]f32,
    float_corners: []const [3]f32,
) ?[3]f32 {
    var vertices: [result_count_max + 3][3]f32 = undefined;
    var count: usize = 0;
    for (float_corners) |c| {
        vertices[count] = c;
        count += 1;
    }
    vertices[count] = a;
    count += 1;
    if (hit.path_count > 0 and hit.hit_edge_index >= 0) {
        const found = pair.world.nav.getTileAndPolyByRefUnsafe(hit.path[hit.path_count - 1]);
        const edge: usize = @intCast(hit.hit_edge_index);
        vertices[count] = found.tile.verts[found.poly.verts[edge]];
        vertices[count + 1] =
            found.tile.verts[found.poly.verts[(edge + 1) % found.poly.vert_count]];
        count += 2;
    }
    for (vertices[0..count]) |v| {
        const dx: f64 = @abs(v[0] - at[0]) * scale;
        const dz: f64 = @abs(v[2] - at[2]) * scale;
        if (dx <= tolerance and dz <= tolerance) return v;
    }
    return alongWall(pair, hit, at, b);
}

/// Where a ray that stopped at `at` on its way to `b` runs along the wall it
/// stopped on: the wall's end toward `b`, or `b` when the wall reaches past
/// it. Null when the ray meets the wall at an angle, that is when `at` or
/// `b` lies more than a position's tolerance off the wall's line. Along a
/// wall the float port cannot tell which side of the wall's line the ray
/// runs on, and stops it where it starts.
fn alongWall(pair: *Pair, hit: *const detour.RaycastHit, at: [3]f32, b: [3]f32) ?[3]f32 {
    if (hit.path_count == 0 or hit.hit_edge_index < 0) return null;
    const found = pair.world.nav.getTileAndPolyByRefUnsafe(hit.path[hit.path_count - 1]);
    const edge: usize = @intCast(hit.hit_edge_index);
    const v0 = found.tile.verts[found.poly.verts[edge]];
    const v1 = found.tile.verts[found.poly.verts[(edge + 1) % found.poly.vert_count]];
    var plane: [4][2]f64 = undefined;
    const points = planeOf(f32, &.{ v0, v1, at, b }, &plane);
    const wall = [2]f64{ points[1][0] - points[0][0], points[1][1] - points[0][1] };
    const wall_length = std.math.hypot(wall[0], wall[1]);
    if (wall_length == 0) return null;
    for (points[2..]) |p| {
        const across = (p[0] - points[0][0]) * wall[1] - (p[1] - points[0][1]) * wall[0];
        if (@abs(across) / wall_length > tolerance) return null;
    }
    // How far along the leg from `at` to `b` each end of the wall lies.
    const leg = [2]f64{ points[3][0] - points[2][0], points[3][1] - points[2][1] };
    const leg_sqr = leg[0] * leg[0] + leg[1] * leg[1];
    if (leg_sqr == 0) return null;
    var along: [2]f64 = undefined;
    for (points[0..2], &along) |p, *s| {
        s.* = ((p[0] - points[2][0]) * leg[0] + (p[1] - points[2][1]) * leg[1]) / leg_sqr;
    }
    const far = @max(along[0], along[1]);
    if (far <= 0) return null;
    if (far >= 1) return b;
    return if (along[0] > along[1]) v0 else v1;
}

/// Whether polygon `ref` of the integer mesh has a vertex within tolerance
/// of `corner` in the xz plane.
fn hasVertexAt(pair: *Pair, ref: PolyRef, corner: [3]i32) bool {
    const found = pair.nav.getTileAndPolyByRefUnsafe(ref);
    for (found.poly.verts[0..found.poly.vert_count]) |index| {
        const vert = found.tile.verts[index];
        const dx: f64 = @floatFromInt(@abs(vert[0] - corner[0]));
        const dz: f64 = @floatFromInt(@abs(vert[2] - corner[2]));
        if (dx <= tolerance and dz <= tolerance) return true;
    }
    return false;
}

/// Whether a float corner and an integer one stand at the same place.
fn sameCorner(float_corner: [3]f32, corner: [3]i32) bool {
    return axisError(float_corner, corner, 0) <= tolerance and
        axisError(float_corner, corner, 2) <= tolerance and
        axisError(float_corner, corner, 1) <= height_tolerance;
}

/// The one corner of `line` at the place of `corner`, or null when none or
/// several are there.
fn soleMatch(line: []const [3]f32, corner: [3]i32) ?usize {
    var found: ?usize = null;
    for (line, 0..) |float_corner, f| {
        if (!sameCorner(float_corner, corner)) continue;
        if (found != null) return null;
        found = f;
    }
    return found;
}

/// Whether the float polylines `a` and `b` lie within a position's
/// tolerance of each other in the xz plane, each point of one near the
/// other.
fn sameLine(a: []const [3]f32, b: []const [3]f32) bool {
    for ([2][]const [3]f32{ a, b }, [2][]const [3]f32{ b, a }) |from, to| {
        for (from) |p| {
            // Rounded to whole millimetres: half a millimetre more.
            if (distanceToLine(millimetres(p), to) > tolerance + 0.5) return false;
        }
    }
    return true;
}

/// A vertex of a straight path: its place in millimetres, its flags and
/// its polygon.
const PathVertex = struct {
    at: [3]f64,
    flags: detour.StraightPathFlags,
    ref: PolyRef,
    /// Where `ref` stands in the corridor, past the vertices before it: a
    /// corridor that turns back holds a polygon twice.
    index: ?usize,

    /// Whether the vertex can be a portal crossing: neither the start, the
    /// end nor an off-mesh connection. A corner has no flags either.
    fn isCrossing(self: PathVertex) bool {
        return @as(u8, @bitCast(self.flags)) == 0;
    }

    fn near(a: PathVertex, b: PathVertex) bool {
        return @abs(a.at[0] - b.at[0]) <= tolerance and @abs(a.at[2] - b.at[2]) <= tolerance and
            @abs(a.at[1] - b.at[1]) <= height_tolerance;
    }

    /// Whether two vertices, one of each path, stand for the same one: the
    /// same polygon at the same place in the corridor, so a crossing of the
    /// same portal or a corner in the same polygon. Places are compared
    /// elsewhere: the corners against the float corners, a crossing on its
    /// portal, since where a leg meets its portal at a slant, a leg's error
    /// moves the crossing along the portal by that error over the sine of
    /// the angle between them.
    fn sameVertex(a: PathVertex, b: PathVertex) bool {
        return a.ref == b.ref and a.index == b.index;
    }

    fn corner(self: PathVertex) [3]i32 {
        var p: [3]i32 = undefined;
        for (&p, self.at) |*c, value| c.* = @intFromFloat(@round(value));
        return p;
    }
};

/// The float portal the crossing `vertex` of a path along `corridor` lies
/// on, or null when its polygon is the corridor's first or not in it.
fn crossedPortal(pair: *Pair, corridor: []const PolyRef, vertex: PathVertex) ?[2][3]f32 {
    const k = vertex.index orelse return null;
    if (k == 0) return null;
    return floatPortal(&pair.world.nav, corridor[k - 1], corridor[k]);
}

/// The vertices of a straight path in millimetres, each float crossing
/// held to its portal as the integer variant holds its own.
fn pathVertices(
    pair: *Pair,
    corridor: []const PolyRef,
    comptime T: type,
    points: []const [3]T,
    flags: []const detour.StraightPathFlags,
    refs: []const PolyRef,
    out: []PathVertex,
) []PathVertex {
    var cursor: usize = 0;
    for (points, flags, refs, out[0..points.len]) |p, flag, ref, *vertex| {
        vertex.* = .{ .at = undefined, .flags = flag, .ref = ref, .index = null };
        if (ref != 0) {
            if (std.mem.findScalarPos(PolyRef, corridor, cursor, ref)) |k| {
                cursor = k;
                vertex.index = k;
            }
        }
        for (&vertex.at, p) |*c, value| {
            c.* = if (T == f32) @as(f64, value) * scale else @floatFromInt(value);
        }
        if (T == f32 and vertex.isCrossing()) {
            if (crossedPortal(pair, corridor, vertex.*)) |portal| {
                vertex.at = clampToSegment(vertex.at, portal);
            }
        }
    }
    return out[0..points.len];
}

/// The point of the float segment `ends` nearest `p` in the xz plane, in
/// millimetres, with the segment's height there.
fn clampToSegment(p: [3]f64, ends: [2][3]f32) [3]f64 {
    var a: [3]f64 = undefined;
    var b: [3]f64 = undefined;
    for (&a, &b, ends[0], ends[1]) |*x, *y, u, v| {
        x.* = @as(f64, u) * scale;
        y.* = @as(f64, v) * scale;
    }
    const dx = b[0] - a[0];
    const dz = b[2] - a[2];
    const len_sqr = dx * dx + dz * dz;
    if (len_sqr == 0) return a;
    const t = std.math.clamp(((p[0] - a[0]) * dx + (p[2] - a[2]) * dz) / len_sqr, 0, 1);
    var point: [3]f64 = undefined;
    for (&point, a, b) |*c, u, v| c.* = u + (v - u) * t;
    return point;
}

/// Whether `path[v]` is a crossing that one path may list and the other
/// not. Where a crossing and the vertex beside it are one point, each path
/// keeps one of them: the float port counts points within 1/16384 of a
/// unit as one, the integer variant only the same point, so two points a
/// rounding apart are one on one side and two on the other. A crossing at
/// an end of its portal may be one point with a corner there, and a leg
/// through a portal's end can cross it on one path and pass beside it on
/// the other. A path that starts or ends on a portal may list its crossing
/// or not, and the float port finds no crossing where the leg runs within
/// 1e-6 of parallel to the portal, the integer variant only where it is.
fn mayBeOneSided(
    pair: *Pair,
    corridor: []const PolyRef,
    path: []const PathVertex,
    v: usize,
) bool {
    const vertex = path[v];
    if (!vertex.isCrossing()) return false;
    const portal = crossedPortal(pair, corridor, vertex) orelse return false;
    for (portal) |end| {
        const dx = @abs(@as(f64, end[0]) * scale - vertex.at[0]);
        const dz = @abs(@as(f64, end[2]) * scale - vertex.at[2]);
        if (dx <= tolerance and dz <= tolerance) return true;
    }
    for ([_]PathVertex{ path[0], path[path.len - 1] }) |end| {
        const on = clampToSegment(end.at, portal);
        if (std.math.hypot(on[0] - end.at[0], on[2] - end.at[2]) <= tolerance) return true;
    }
    for ([_]usize{ v -| 1, @min(v + 1, path.len - 1) }) |beside| {
        if (beside != v and path[beside].near(vertex)) return true;
    }
    return false;
}

/// Compares the vertices of the float and the integer straight path with
/// crossings, in order: at each vertex both have, the same flags and the
/// same polygon, or for a vertex several portals share either polygon that
/// holds it. A crossing only one side lists must be one `mayBeOneSided`
/// allows, and each integer crossing, a vertex not at one of `corners`,
/// must lie on its portal. Returns how many did, and how many vertices named another
/// polygon at a shared vertex.
fn compareCrossings(
    pair: *Pair,
    corridor: []const PolyRef,
    float_vertices: []const PathVertex,
    vertices: []const PathVertex,
    corners: []const [3]i32,
) !struct { one_sided: usize, shared: usize } {
    var one_sided: usize = 0;
    var shared: usize = 0;
    var f: usize = 0;
    var i: usize = 0;
    errdefer {
        for (float_vertices) |v| std.debug.print("  float {any}\n", .{v});
        for (vertices) |v| std.debug.print("  integer {any}\n", .{v});
    }
    // Each integer crossing, a vertex that is not one of the corners, lies
    // on the portal it crosses.
    for (vertices) |vertex| {
        if (!vertex.isCrossing()) continue;
        const corner = for (corners) |c| {
            const dx = @abs(@as(f64, @floatFromInt(c[0])) - vertex.at[0]);
            const dz = @abs(@as(f64, @floatFromInt(c[2])) - vertex.at[2]);
            if (dx <= tolerance and dz <= tolerance) break true;
        } else false;
        if (corner) continue;
        const portal = crossedPortal(pair, corridor, vertex) orelse
            return error.TestUnexpectedResult;
        const on = clampToSegment(vertex.at, portal);
        const dx = on[0] - vertex.at[0];
        const dz = on[2] - vertex.at[2];
        if (@sqrt(dx * dx + dz * dz) > tolerance + linkLimitSlack(pair, corridor)) {
            std.debug.print("a crossing {d:.2} mm off its portal\n", .{@sqrt(dx * dx + dz * dz)});
            return error.TestOutsideTolerance;
        }
    }
    while (f < float_vertices.len or i < vertices.len) {
        errdefer std.debug.print("  vertex {d} of {d} on the float path, {d} of {d}\n", .{
            f,
            float_vertices.len,
            i,
            vertices.len,
        });
        if (f < float_vertices.len and i < vertices.len) {
            const a = float_vertices[f];
            const b = vertices[i];
            // A vertex several portals share names whichever of their
            // polygons the funnel turned at; both must hold it.
            const shared_vertex = !a.sameVertex(b) and a.near(b) and
                for ([_]PolyRef{ a.ref, b.ref }) |ref| {
                    if (ref == 0 or std.mem.findScalar(PolyRef, corridor, ref) == null) break false;
                    if (!hasVertexAt(pair, ref, b.corner())) break false;
                } else true;
            if (a.sameVertex(b) or shared_vertex) {
                try testing.expectEqual(a.flags, b.flags);
                shared += @intFromBool(shared_vertex);
                f += 1;
                i += 1;
                continue;
            }
        }
        if (f < float_vertices.len and mayBeOneSided(pair, corridor, float_vertices, f)) {
            one_sided += 1;
            f += 1;
            continue;
        }
        if (i < vertices.len and mayBeOneSided(pair, corridor, vertices, i)) {
            one_sided += 1;
            i += 1;
            continue;
        }
        return error.TestUnexpectedResult;
    }
    return .{ .one_sided = one_sided, .shared = shared };
}

test "integer findStraightPath: on one corridor the two paths lie within tolerance of each other" {
    var tally: Tally = .{};
    var shared_vertex_corners: usize = 0;
    var longer_float_paths: usize = 0;
    var longest_surplus: f64 = 0;
    var crossing_cases: usize = 0;
    var one_sided_crossings: usize = 0;
    var crossing_shared_vertices: usize = 0;
    var float_disagrees: usize = 0;
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(0x73747261));
    const random = prng.random();
    var corridor: PathAnswer = undefined;
    var float_corners: [result_count_max][3]f32 = undefined;
    var float_flags: [result_count_max]detour.StraightPathFlags = undefined;
    var float_refs: [result_count_max]PolyRef = undefined;
    var corners: [result_count_max][3]i32 = undefined;
    var flags: [result_count_max]detour.StraightPathFlags = undefined;
    var refs: [result_count_max]PolyRef = undefined;
    const option_choices = [_]detour.StraightPathOptions{
        .{},
        .{ .area_crossings = true },
        .{ .all_crossings = true },
    };
    for (try pairs(), 0..) |*pair, level| {
        var case: usize = 0;
        var tries: usize = 0;
        while (case < 300 and tries < 3000) : (tries += 1) {
            errdefer std.debug.print("  level {d}, case {d}\n", .{ level, case });
            const filters = Filters.forCase(pair, case);
            const start = pair.startAt(pair.groundPoint(random, 0.3), &filters) orelse continue;
            const end = pair.startAt(pair.groundPoint(random, 0.3), &filters) orelse continue;
            floatPath(pair, .whole, start, end, @splat(0), &filters.float, &corridor);
            if (corridor.count == 0) continue;
            const options = option_choices[case % option_choices.len];
            case += 1;

            const path = corridor.path[0..corridor.count];
            var float_count: usize = 0;
            const expected = pair.float_query.findStraightPath(
                metres(start.pos),
                metres(end.pos),
                path,
                &float_corners,
                &float_flags,
                &float_refs,
                &float_count,
                options,
            );
            var stable = true;
            for (nudges) |by| {
                var count: usize = 0;
                const near = pair.float_query.findStraightPath(
                    metres(moved(start.pos, by)),
                    metres(moved(end.pos, by)),
                    path,
                    &float_corners,
                    null,
                    null,
                    &count,
                    options,
                );
                if (statusBits(near) != statusBits(expected)) stable = false;
            }
            tally.count(stable);
            if (!stable) continue;
            // The nudged runs wrote over the corners: run it again.
            _ = pair.float_query.findStraightPath(
                metres(start.pos),
                metres(end.pos),
                path,
                &float_corners,
                &float_flags,
                &float_refs,
                &float_count,
                options,
            );

            var count: usize = 0;
            const actual = pair.query.findStraightPath(
                start.pos,
                end.pos,
                path,
                &corners,
                &flags,
                &refs,
                &count,
                options,
            );
            try testing.expectEqual(statusBits(expected), statusBits(actual));
            if (count == 0 or float_count == 0) {
                try testing.expectEqual(float_count, count);
                continue;
            }
            // The float port extends a crossing past the end of its portal
            // where the integer one stops at the end, so with crossings both
            // paths are held against the float corners alone.
            const corners_only = !options.area_crossings and !options.all_crossings;
            // With crossings: the float port's vertices, each crossing held to
            // its portal as the integer variant holds its own.
            var float_vertices: [result_count_max]PathVertex = undefined;
            const float_with = pathVertices(
                pair,
                path,
                f32,
                float_corners[0..float_count],
                float_flags[0..float_count],
                float_refs[0..float_count],
                &float_vertices,
            );
            var float_with_line: [result_count_max][3]f32 = undefined;
            for (float_with, float_with_line[0..float_with.len]) |vertex, *point| {
                for (point, vertex.at) |*c, value| c.* = @floatCast(value / scale);
            }
            // The float corners alone: the float port extends a crossing past
            // the end of its portal.
            var float_line = float_corners[0..float_count];
            if (!corners_only) {
                var only_count: usize = 0;
                _ = pair.float_query.findStraightPath(
                    metres(start.pos),
                    metres(end.pos),
                    path,
                    &float_corners,
                    null,
                    null,
                    &only_count,
                    .{},
                );
                float_line = float_corners[0..only_count];
            }
            // The integer corners alone, the same way.
            var only_corners: [result_count_max][3]i32 = undefined;
            var only_refs: [result_count_max]PolyRef = undefined;
            var integer_line: []const [3]i32 = corners[0..count];
            var integer_refs: []const PolyRef = refs[0..count];
            if (!corners_only) {
                var only_count: usize = 0;
                _ = pair.query.findStraightPath(
                    start.pos,
                    end.pos,
                    path,
                    &only_corners,
                    null,
                    &only_refs,
                    &only_count,
                    .{},
                );
                integer_line = only_corners[0..only_count];
                integer_refs = only_refs[0..only_count];
            }
            var line: [result_count_max][3]f32 = undefined;
            for (integer_line, line[0..integer_line.len]) |corner, *point| point.* = metres(corner);
            var off: f64 = 0;
            for (integer_line) |corner| off = @max(off, distanceToLine(corner, float_line));
            for (float_line) |float_corner| {
                const back = distanceToLine(millimetres(float_corner), line[0..integer_line.len]);
                off = @max(off, back - 0.5);
            }
            // Links whose limits are a step apart move portal ends by a
            // 255th of their edge (integer-design.md, "Link limits can differ
            // by one step of 1/255").
            if (off > tolerance + linkLimitSlack(pair, path)) {
                // Where corridor vertices meet on one line, the float port's
                // funnel can turn on the far side of a portal and take a
                // longer way; the exact one must then pass every portal and
                // be no longer.
                if (shorterThroughCorridor(
                    pair,
                    &filters.float,
                    path,
                    float_line,
                    integer_line,
                    integer_refs,
                )) {
                    longer_float_paths += 1;
                    var float_plane: [result_count_max][2]f64 = undefined;
                    var plane: [result_count_max][2]f64 = undefined;
                    const surplus = pathLength(planeOf(f32, float_line, &float_plane)) -
                        pathLength(planeOf(i32, integer_line, &plane));
                    longest_surplus = @max(longest_surplus, surplus);
                    continue;
                }
                std.debug.print("the paths lie {d:.2} mm apart\n", .{off});
                return error.TestOutsideTolerance;
            }
            if (!corners_only) {
                crossing_cases += 1;
                // On a corridor that turns back along one line, the float
                // port's corners with crossings can differ from its corners
                // without: it gives no one answer to compare with.
                if (!sameLine(float_line, float_with_line[0..float_with.len])) {
                    float_disagrees += 1;
                    continue;
                }
                var vertices: [result_count_max]PathVertex = undefined;
                const outcome = try compareCrossings(
                    pair,
                    path,
                    float_with,
                    pathVertices(
                        pair,
                        path,
                        i32,
                        corners[0..count],
                        flags[0..count],
                        refs[0..count],
                        &vertices,
                    ),
                    integer_line,
                );
                one_sided_crossings += outcome.one_sided;
                crossing_shared_vertices += outcome.shared;
                continue;
            }
            for (corners[0..count], 0..) |corner, c| {
                errdefer std.debug.print("  integer corner {d} of {d}\n", .{ c, count });
                // A corner both have, and only one on either path lies
                // there: the same flags and polygon.
                const f = soleMatch(float_line, corner) orelse continue;
                var integer_matches: usize = 0;
                for (corners[0..count]) |other| {
                    if (sameCorner(float_line[f], other)) integer_matches += 1;
                }
                if (integer_matches != 1) continue;
                try testing.expectEqual(float_flags[f], flags[c]);
                if (float_refs[f] == refs[c]) continue;
                // A corner at a vertex several portals of the corridor
                // share names whichever of their polygons the funnel
                // turned at; both must hold that vertex.
                for ([_]PolyRef{ float_refs[f], refs[c] }) |ref| {
                    if (std.mem.findScalar(PolyRef, path, ref) == null and ref != 0) {
                        return error.TestUnexpectedResult;
                    }
                    if (ref != 0 and !hasVertexAt(pair, ref, corner)) {
                        std.debug.print("corner polygons: float {d}, integer {d}\n", .{
                            float_refs[f],
                            refs[c],
                        });
                        return error.TestUnexpectedResult;
                    }
                }
                shared_vertex_corners += 1;
            }
        }
    }
    try tally.expectEnough("findStraightPath");
    std.debug.print("findStraightPath with crossings: {d} cases, {d} crossings on one path " ++
        "only, {d} vertices at a vertex several portals share, {d} where " ++
        "the float port's corners change with the option\n", .{
        crossing_cases,
        one_sided_crossings,
        crossing_shared_vertices,
        float_disagrees,
    });
    std.debug.print(
        "findStraightPath: {d} stable cases, {d} corners at a vertex several portals share, " ++
            "{d} longer float paths, the float path at most {d:.1} mm longer\n",
        .{ tally.stable, shared_vertex_corners, longer_float_paths, longest_surplus },
    );
    // A longer float path is a rounding accident of the float port: rare,
    // at most 13 of 1200 over seeds 0 to 31 (seeds 19 and 26).
    // Shared-vertex corners: at most 25 (seeds 13 and 30). With the crossing
    // options, of about 800 cases: crossings on one path only at most 33
    // (seed 31), vertices several portals share at most 17 (seed 23), and
    // float corners that change with the option at most 2 (seed 0).
    try testing.expect(longer_float_paths * 80 <= tally.stable);
    try testing.expect(shared_vertex_corners * 45 <= tally.stable);
    try testing.expect(one_sided_crossings * 22 <= crossing_cases);
    try testing.expect(crossing_shared_vertices * 40 <= crossing_cases);
    try testing.expect(float_disagrees * 250 <= crossing_cases);
}

test "integer findStraightPath's walkability check: a leg along a wall at 45 degrees" {
    // Corners of a float path on the undulating level (fuzz seed 8, case
    // 35), and a leg of the integer path on the same corridor. The leg runs
    // exactly along a wall at 45 degrees, and the float port's ray along it
    // stops where it starts, and again just past its start.
    const float_corners = [_][3]f32{
        .{ 91.284, -2.233, 75.222 },
        .{ 85.506004, 1.94522, 72.15201 },
        .{ 85.106, 1.94522, 71.75201 },
        .{ 80.40797, -1.6390936, 66.75201 },
        .{ 56.706, 3.1452203, 47.552 },
        .{ 56.706, -2.45478, 40.652 },
        .{ 33.388, -1.411, 41.515 },
    };
    const pair = &(try pairs())[3];
    const filter: detour.QueryFilter = .{};
    const a: [3]f32 = .{ 85.506, 1.945, 72.152 };
    const b: [3]f32 = .{ 85.106, 1.945, 71.752 };
    var ref: PolyRef = 0;
    var point = a;
    _ = pair.float_query.findNearestPoly(a, .{ 0.01, 0.5, 0.01 }, &filter, &ref, &point, null);
    try testing.expect(ref != 0);
    var visited: [result_count_max]PolyRef = undefined;
    var hit: detour.RaycastHit = .{
        .t = 0,
        .hit_normal = @splat(0),
        .hit_edge_index = 0,
        .path = &visited,
        .path_count = 0,
        .path_cost = 0,
    };
    try testing.expect(pair.float_query.raycast(ref, a, b, &filter, .{}, &hit, 0).succeeded);
    try testing.expectEqual(0, hit.t);
    try testing.expect(walkable(pair, &filter, ref, a, b, &float_corners));
}

// ---------------------------------------------------------------------------
// raycast, moveAlongSurface
// ---------------------------------------------------------------------------

const RayAnswer = struct {
    status: u32,
    hit: bool,
    edge: i32,
    t: f32,
    normal: [3]f32,
    cost: f32,
    path: [result_count_max]PolyRef,
    count: usize,

    fn sameDiscrete(a: *const RayAnswer, b: *const RayAnswer) bool {
        return a.status == b.status and a.hit == b.hit and a.edge == b.edge and
            std.mem.eql(PolyRef, a.path[0..a.count], b.path[0..b.count]);
    }
};

fn floatRay(
    pair: *Pair,
    start: Start,
    end: [3]i32,
    by: [3]i32,
    filter: *const detour.QueryFilter,
    options: detour.RaycastOptions,
    answer: *RayAnswer,
) void {
    var hit: detour.RaycastHit = .{
        .t = 0,
        .hit_normal = @splat(0),
        .hit_edge_index = 0,
        .path = &answer.path,
        .path_count = 0,
        .path_cost = 0,
    };
    answer.status = statusBits(pair.float_query.raycast(
        start.ref,
        metres(moved(start.pos, by)),
        metres(moved(end, by)),
        filter,
        options,
        &hit,
        0,
    ));
    answer.hit = hit.t != std.math.floatMax(f32);
    answer.edge = hit.hit_edge_index;
    answer.t = hit.t;
    answer.normal = hit.hit_normal;
    answer.cost = hit.path_cost;
    answer.count = hit.path_count;
}

test "integer raycast: the same polygons and wall; hit point, normal and cost within tolerance" {
    var tally: Tally = .{};
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(0x72617963));
    const random = prng.random();
    var expected: RayAnswer = undefined;
    var near: RayAnswer = undefined;
    var path: [result_count_max]PolyRef = undefined;
    for (try pairs(), 0..) |*pair, level| {
        var case: usize = 0;
        var tries: usize = 0;
        while (case < 300 and tries < 3000) : (tries += 1) {
            errdefer std.debug.print("  level {d}, case {d}\n", .{ level, case });
            const filters = Filters.forCase(pair, case);
            const start = pair.startAt(pair.groundPoint(random, 0.3), &filters) orelse continue;
            var end = pair.groundPoint(random, 0.5);
            if (random.boolean()) {
                // A short ray, that often ends before a wall.
                end = .{
                    start.pos[0] + random.intRangeAtMost(i32, -6000, 6000),
                    start.pos[1],
                    start.pos[2] + random.intRangeAtMost(i32, -6000, 6000),
                };
            }
            const options: detour.RaycastOptions = .{ .use_costs = case % 2 == 0 };
            case += 1;

            floatRay(pair, start, end, @splat(0), &filters.float, options, &expected);
            var stable = true;
            for (nudges) |by| {
                floatRay(pair, start, end, by, &filters.float, options, &near);
                if (!expected.sameDiscrete(&near)) stable = false;
            }
            tally.count(stable);
            if (!stable) continue;

            var hit: integer.RaycastHit = .{
                .t = .zero,
                .hit_normal = @splat(0),
                .hit_edge_index = 0,
                .path = &path,
                .path_count = 0,
                .path_cost = 0,
            };
            const status =
                pair.query.raycast(start.ref, start.pos, end, &filters.int, options, &hit, 0);
            try testing.expectEqual(expected.status, statusBits(status));
            try testing.expectEqualSlices(
                PolyRef,
                expected.path[0..expected.count],
                path[0..hit.path_count],
            );
            try testing.expectEqual(expected.hit, !hit.t.isNever());
            try testing.expectEqual(expected.edge, hit.hit_edge_index);
            if (expected.hit) {
                const from = metres(start.pos);
                const to = metres(end);
                const float_point = lerp(from, to, expected.t);
                // Both points lie on the ray. Moving the wall by a rounding
                // error moves the hit along the ray by that error over the
                // sine of the angle between ray and wall.
                const dx = @as(f64, to[0]) - from[0];
                const dz = @as(f64, to[2]) - from[2];
                const ray_length = @sqrt(dx * dx + dz * dz);
                const sine = if (ray_length > 0)
                    @abs(dx * expected.normal[0] + dz * expected.normal[2]) / ray_length
                else
                    1.0;
                const limit = tolerance / @max(sine, 1.0 / 1024.0);
                // The point moves along the ray, so its height moves by
                // the ray's rise for each step across.
                const dy = @abs(@as(f64, to[1]) - from[1]);
                const rise = if (ray_length > 0) dy / ray_length else 0;
                const height_limit = limit * @max(1.0, rise);
                errdefer std.debug.print("  sine between ray and wall {d:.5}\n", .{sine});
                const point = hit.point(start.pos, end);
                try expectNear("hit point", float_point, point, limit, height_limit);
                try expectNearNormal(expected.normal, hit.hit_normal, 1.0 / 1024.0);
            }
            if (options.use_costs) try expectNearCost(expected.cost, hit.path_cost, hit.path_count);
        }
    }
    try tally.expectEnough("raycast");
}

const MoveAnswer = struct {
    status: u32,
    result: [3]f32,
    visited: [result_count_max]PolyRef,
    count: usize,
    /// Whether the walk ended at the target, inside a polygon, rather than
    /// on a wall. Inside, the end keeps the target's height; on a wall it
    /// takes the wall's, so the two can stand far apart in height at one
    /// place.
    reached: bool,

    fn sameDiscrete(a: *const MoveAnswer, b: *const MoveAnswer) bool {
        return a.status == b.status and a.reached == b.reached and
            std.mem.eql(PolyRef, a.visited[0..a.count], b.visited[0..b.count]);
    }
};

fn floatMove(
    pair: *Pair,
    start: Start,
    end: [3]i32,
    by: [3]i32,
    filter: *const detour.QueryFilter,
    answer: *MoveAnswer,
) void {
    answer.count = 0;
    answer.result = @splat(0);
    const target = metres(moved(end, by));
    answer.status = statusBits(pair.float_query.moveAlongSurface(
        start.ref,
        metres(moved(start.pos, by)),
        target,
        filter,
        &answer.result,
        &answer.visited,
        &answer.count,
    ));
    answer.reached = std.mem.eql(f32, &answer.result, &target);
}

test "integer moveAlongSurface: the same polygons visited, the end within tolerance" {
    var tally: Tally = .{};
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(0x6d6f7665));
    const random = prng.random();
    var expected: MoveAnswer = undefined;
    var near: MoveAnswer = undefined;
    var visited: [result_count_max]PolyRef = undefined;
    for (try pairs(), 0..) |*pair, level| {
        var case: usize = 0;
        var tries: usize = 0;
        while (case < 300 and tries < 3000) : (tries += 1) {
            errdefer std.debug.print("  level {d}, case {d}\n", .{ level, case });
            const filters = Filters.forCase(pair, case);
            const start = pair.startAt(pair.groundPoint(random, 0.3), &filters) orelse continue;
            const reach: i32 = if (random.boolean()) 1500 else 8000;
            const end = [3]i32{
                start.pos[0] + random.intRangeAtMost(i32, -reach, reach),
                start.pos[1] + random.intRangeAtMost(i32, -300, 300),
                start.pos[2] + random.intRangeAtMost(i32, -reach, reach),
            };
            case += 1;

            floatMove(pair, start, end, @splat(0), &filters.float, &expected);
            var stable = true;
            for (nudges) |by| {
                floatMove(pair, start, end, by, &filters.float, &near);
                if (!expected.sameDiscrete(&near)) stable = false;
            }
            tally.count(stable);
            if (!stable) continue;

            var result: [3]i32 = @splat(0);
            var count: usize = 0;
            const status = pair.query.moveAlongSurface(
                start.ref,
                start.pos,
                end,
                &filters.int,
                &result,
                &visited,
                &count,
            );
            try testing.expectEqual(expected.status, statusBits(status));
            try testing.expectEqualSlices(
                PolyRef,
                expected.visited[0..expected.count],
                visited[0..count],
            );
            try expectNear("walk's end", expected.result, result, tolerance, tolerance);
        }
    }
    try tally.expectEnough("moveAlongSurface");
}

// ---------------------------------------------------------------------------
// findPolysAroundCircle, findPolysAroundShape, findLocalNeighbourhood
// ---------------------------------------------------------------------------

/// What a search around a point found, sorted by reference so that the
/// order of polygons reached at the same cost does not count.
const AroundAnswer = struct {
    status: u32,
    entries: [result_count_max]Entry,
    count: usize,

    const Entry = struct {
        ref: PolyRef,
        parent: PolyRef,
        cost: f64,

        fn lessThan(_: void, a: Entry, b: Entry) bool {
            return a.ref < b.ref;
        }
    };

    fn sort(self: *AroundAnswer) void {
        std.mem.sort(Entry, self.entries[0..self.count], {}, Entry.lessThan);
    }

    fn sameDiscrete(a: *const AroundAnswer, b: *const AroundAnswer) bool {
        if (a.status != b.status or a.count != b.count) return false;
        for (a.entries[0..a.count], b.entries[0..b.count]) |x, y| {
            if (x.ref != y.ref or x.parent != y.parent) return false;
        }
        return true;
    }

    /// How many polygons lead from the start to entry `i`, itself included.
    fn depth(self: *const AroundAnswer, i: usize) usize {
        var steps: usize = 1;
        var parent = self.entries[i].parent;
        while (parent != 0 and steps <= self.count) : (steps += 1) {
            const found = for (self.entries[0..self.count]) |entry| {
                if (entry.ref == parent) break entry;
            } else break;
            parent = found.parent;
        }
        return steps;
    }
};

const AroundKind = enum { circle, shape, neighbourhood };

/// A convex shape of four to six corners around `center`, `radius` from it.
fn shapeAround(random: std.Random, center: [3]i32, radius: i32, verts: *[6][3]i32) []const [3]i32 {
    const count = random.intRangeAtMost(usize, 4, 6);
    const turn = random.float(f64) * std.math.tau;
    for (verts[0..count], 0..) |*vert, i| {
        const step: f64 = @floatFromInt(i);
        const angle = turn + std.math.tau * step / @as(f64, @floatFromInt(count));
        const reach: f64 = @floatFromInt(radius);
        vert.* = .{
            center[0] + @as(i32, @intFromFloat(@round(@cos(angle) * reach))),
            center[1],
            center[2] + @as(i32, @intFromFloat(@round(@sin(angle) * reach))),
        };
    }
    return verts[0..count];
}

fn floatAround(
    pair: *Pair,
    kind: AroundKind,
    start: Start,
    shape: []const [3]i32,
    radius: i32,
    by: [3]i32,
    filter: *const detour.QueryFilter,
    answer: *AroundAnswer,
) void {
    var refs: [result_count_max]PolyRef = undefined;
    var parents: [result_count_max]PolyRef = undefined;
    var costs: [result_count_max]f32 = @splat(0);
    var count: usize = 0;
    const center = metres(moved(start.pos, by));
    const status = switch (kind) {
        .circle => pair.float_query.findPolysAroundCircle(
            start.ref,
            center,
            metresOf(radius),
            filter,
            &refs,
            &parents,
            &costs,
            &count,
            result_count_max,
        ),
        .shape => blk: {
            var verts: [6][3]f32 = undefined;
            for (shape, verts[0..shape.len]) |vert, *out| out.* = metres(moved(vert, by));
            break :blk pair.float_query.findPolysAroundShape(
                start.ref,
                verts[0..shape.len],
                filter,
                &refs,
                &parents,
                &costs,
                &count,
                result_count_max,
            );
        },
        .neighbourhood => pair.float_query.findLocalNeighbourhood(
            start.ref,
            center,
            metresOf(radius),
            filter,
            &refs,
            &parents,
            &count,
        ),
    };
    answer.status = statusBits(status);
    answer.count = count;
    const entries = answer.entries[0..count];
    for (entries, refs[0..count], parents[0..count], costs[0..count]) |*entry, ref, parent, cost| {
        entry.* = .{ .ref = ref, .parent = parent, .cost = @as(f64, cost) * scale };
    }
    answer.sort();
}

fn integerAround(
    pair: *Pair,
    kind: AroundKind,
    start: Start,
    shape: []const [3]i32,
    radius: i32,
    filter: *const integer.QueryFilter,
    answer: *AroundAnswer,
) void {
    var refs: [result_count_max]PolyRef = undefined;
    var parents: [result_count_max]PolyRef = undefined;
    var costs: [result_count_max]u64 = @splat(0);
    var count: usize = 0;
    const status = switch (kind) {
        .circle => pair.query.findPolysAroundCircle(
            start.ref,
            start.pos,
            radius,
            filter,
            &refs,
            &parents,
            &costs,
            &count,
            result_count_max,
        ),
        .shape => pair.query.findPolysAroundShape(
            start.ref,
            shape,
            filter,
            &refs,
            &parents,
            &costs,
            &count,
            result_count_max,
        ),
        .neighbourhood => pair.query.findLocalNeighbourhood(
            start.ref,
            start.pos,
            radius,
            filter,
            &refs,
            &parents,
            &count,
        ),
    };
    answer.status = statusBits(status);
    answer.count = count;
    const entries = answer.entries[0..count];
    for (entries, refs[0..count], parents[0..count], costs[0..count]) |*entry, ref, parent, cost| {
        entry.* = .{
            .ref = ref,
            .parent = parent,
            .cost = @as(f64, @floatFromInt(cost)) / 256.0,
        };
    }
    answer.sort();
}

test "integer findPolysAround*, findLocalNeighbourhood: the same polygons, costs within tolerance" {
    var tally: Tally = .{};
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(0x726f756e));
    const random = prng.random();
    var expected: AroundAnswer = undefined;
    var near: AroundAnswer = undefined;
    var actual: AroundAnswer = undefined;
    var shape_verts: [6][3]i32 = undefined;
    for (try pairs(), 0..) |*pair, level| {
        var case: usize = 0;
        var tries: usize = 0;
        while (case < 300 and tries < 3000) : (tries += 1) {
            errdefer std.debug.print("  level {d}, case {d}\n", .{ level, case });
            const filters = Filters.forCase(pair, case);
            const start = pair.startAt(pair.groundPoint(random, 0.3), &filters) orelse continue;
            const kind: AroundKind = @fromBackingInt(@as(u2, @intCast(case % 3)));
            const radius = if (kind == .neighbourhood)
                random.intRangeAtMost(i32, 500, 6000)
            else
                random.intRangeAtMost(i32, 500, 12000);
            const shape = shapeAround(random, start.pos, radius, &shape_verts);
            case += 1;

            floatAround(pair, kind, start, shape, radius, @splat(0), &filters.float, &expected);
            var stable = true;
            for (nudges) |by| {
                floatAround(pair, kind, start, shape, radius, by, &filters.float, &near);
                if (!expected.sameDiscrete(&near)) stable = false;
            }
            tally.count(stable);
            if (!stable) continue;

            integerAround(pair, kind, start, shape, radius, &filters.int, &actual);
            try testing.expectEqual(expected.status, actual.status);
            if (!expected.sameDiscrete(&actual)) {
                std.debug.print("{t}, radius {d} mm: float found {d}, integer {d}\n", .{
                    kind,
                    radius,
                    expected.count,
                    actual.count,
                });
                for (expected.entries[0..expected.count]) |entry| {
                    std.debug.print("  float {d} from {d}\n", .{ entry.ref, entry.parent });
                }
                for (actual.entries[0..actual.count]) |entry| {
                    std.debug.print("  integer {d} from {d}\n", .{ entry.ref, entry.parent });
                }
                return error.TestExpectedEqual;
            }
            if (kind == .neighbourhood) continue;
            // Each step between edge midpoints may be off by a millimetre and
            // the error of both midpoints, times the dearest area cost.
            const cost_max: f64 = std.mem.max(f32, &filters.float.area_cost);
            const per_polygon = (1.0 + 2.0 * tolerance) * cost_max;
            const float_entries = expected.entries[0..expected.count];
            for (float_entries, actual.entries[0..actual.count], 0..) |a, b, i| {
                const depth: f64 = @floatFromInt(expected.depth(i));
                const limit = 0.001 * a.cost + per_polygon * depth + tolerance;
                if (@abs(a.cost - b.cost) <= limit) continue;
                std.debug.print("cost of polygon {d}: float {d:.3} mm, integer {d:.3} mm\n", .{
                    a.ref,
                    a.cost,
                    b.cost,
                });
                return error.TestOutsideTolerance;
            }
        }
    }
    try tally.expectEnough("searches around a point");
}

// ---------------------------------------------------------------------------
// getPolyWallSegments, findDistanceToWall
// ---------------------------------------------------------------------------

test "integer getPolyWallSegments: the same segments, their ends within tolerance" {
    var compared: usize = 0;
    var float_segments: [result_count_max][2][3]f32 = undefined;
    var float_refs: [result_count_max]PolyRef = undefined;
    var segments: [result_count_max][2][3]i32 = undefined;
    var refs: [result_count_max]PolyRef = undefined;
    for (try pairs(), 0..) |*pair, level| {
        const float_nav = &pair.world.nav;
        for (float_nav.tiles) |*tile| {
            if (tile.header == null) continue;
            const base = float_nav.getPolyRefBase(tile);
            for (tile.polys, 0..) |poly, p| {
                if (poly.getType() == .off_mesh_connection) continue;
                const ref = base | @as(PolyRef, @intCast(p));
                for ([_]bool{ false, true }) |varied| {
                    for ([_]bool{ false, true }) |with_refs| {
                        errdefer std.debug.print("  level {d}, polygon {d}\n", .{ level, ref });
                        const filters = Filters.init(varied and pair.varied);
                        var float_count: usize = 0;
                        const expected = pair.float_query.getPolyWallSegments(
                            ref,
                            &filters.float,
                            &float_segments,
                            if (with_refs) &float_refs else null,
                            &float_count,
                        );
                        var count: usize = 0;
                        const actual = pair.query.getPolyWallSegments(
                            ref,
                            &filters.int,
                            &segments,
                            if (with_refs) &refs else null,
                            &count,
                        );
                        try testing.expectEqual(statusBits(expected), statusBits(actual));
                        try testing.expectEqual(float_count, count);
                        if (with_refs) {
                            try testing.expectEqualSlices(
                                PolyRef,
                                float_refs[0..count],
                                refs[0..count],
                            );
                        }
                        // A link's limits a step apart move a segment's
                        // ends by a 255th of the edge it lies on.
                        var longest: f64 = 0;
                        for (0..poly.vert_count) |e| {
                            const va = tile.verts[poly.verts[e]];
                            const vb = tile.verts[poly.verts[(e + 1) % poly.vert_count]];
                            longest = @max(longest, distance(va, vb) * scale);
                        }
                        for (float_segments[0..count], segments[0..count]) |a, b| {
                            const limit = tolerance + longest / 255.0;
                            try expectNear("segment start", a[0], b[0], limit, limit);
                            try expectNear("segment end", a[1], b[1], limit, limit);
                        }
                        compared += 1;
                    }
                }
            }
        }
    }
    try testing.expect(compared >= case_count_min);
}

const WallAnswer = struct {
    status: u32,
    distance: f32,
    point: [3]f32,
    normal: [3]f32,
};

fn floatWall(
    pair: *Pair,
    start: Start,
    radius: i32,
    by: [3]i32,
    filter: *const detour.QueryFilter,
) WallAnswer {
    var answer: WallAnswer = .{
        .status = 0,
        .distance = 0,
        .point = @splat(0),
        .normal = @splat(0),
    };
    answer.status = statusBits(pair.float_query.findDistanceToWall(
        start.ref,
        metres(moved(start.pos, by)),
        metresOf(radius),
        filter,
        &answer.distance,
        &answer.point,
        &answer.normal,
    ));
    return answer;
}

test "integer findDistanceToWall: the distance, wall point and normal within tolerance" {
    var tally: Tally = .{};
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(0x77616c6c));
    const random = prng.random();
    var ties: usize = 0;
    for (try pairs(), 0..) |*pair, level| {
        var case: usize = 0;
        var tries: usize = 0;
        while (case < 300 and tries < 3000) : (tries += 1) {
            errdefer std.debug.print("  level {d}, case {d}\n", .{ level, case });
            const filters = Filters.forCase(pair, case);
            const start = pair.startAt(pair.groundPoint(random, 0.3), &filters) orelse continue;
            const radius = random.intRangeAtMost(i32, 500, 10000);
            case += 1;

            const expected = floatWall(pair, start, radius, @splat(0), &filters.float);
            var stable = true;
            for (nudges) |by| {
                const near = floatWall(pair, start, radius, by, &filters.float);
                if (near.status != expected.status) stable = false;
            }
            tally.count(stable);
            if (!stable) continue;

            var hit_distance: i32 = 0;
            var point: [3]i32 = @splat(0);
            var normal: [3]i32 = @splat(0);
            const status = pair.query.findDistanceToWall(
                start.ref,
                start.pos,
                radius,
                &filters.int,
                &hit_distance,
                &point,
                &normal,
            );
            try testing.expectEqual(expected.status, statusBits(status));
            if (status.failed) continue;
            try expectNearLength("wall distance", expected.distance, hit_distance, tolerance);
            const expected_mm = @as(f64, expected.distance) * scale;
            if (expected_mm >= @as(f64, @floatFromInt(radius)) - tolerance) continue;
            // Two walls equally near: either wall's point will do.
            if (axisError(expected.point, point, 0) > tolerance or
                axisError(expected.point, point, 2) > tolerance)
            {
                const dx: f64 = @floatFromInt(point[0] - start.pos[0]);
                const dz: f64 = @floatFromInt(point[2] - start.pos[2]);
                const reach = @sqrt(dx * dx + dz * dz);
                try testing.expect(@abs(reach - expected_mm) <= 2 * tolerance);
                ties += 1;
                continue;
            }
            // The normal points from the wall point to the centre: the
            // points' errors turn it by up to their sum over the distance.
            if (expected_mm > 100.0) {
                const limit = @max(1.0 / 256.0, 2 * tolerance / expected_mm);
                try expectNearNormal(expected.normal, normal, limit);
            }
        }
    }
    try tally.expectEnough("findDistanceToWall");
    std.debug.print("findDistanceToWall: {d} stable cases, {d} ties between walls\n", .{
        tally.stable,
        ties,
    });
    // Ties: at most 4 of 1200 over seeds 0 to 31 (seed 10).
    try testing.expect(ties * 300 <= tally.stable);
}

// ---------------------------------------------------------------------------
// findRandomPoint, findRandomPointAroundCircle
// ---------------------------------------------------------------------------

/// Whether `point` lies on polygon `ref` of the integer mesh, within one
/// millimetre in the xz plane.
fn onPolygon(pair: *Pair, ref: PolyRef, point: [3]i32) !bool {
    var closest: [3]i32 = @splat(0);
    if (pair.query.closestPointOnPoly(ref, point, &closest, null).failed) return false;
    const dx = closest[0] - point[0];
    const dz = closest[2] - point[2];
    return dx * dx + dz * dz <= 1;
}

fn passes(pair: *Pair, ref: PolyRef, filter: *const integer.QueryFilter) bool {
    const found = pair.nav.getTileAndPolyByRefUnsafe(ref);
    return filter.passFilter(ref, found.tile, found.poly);
}

test "integer findRandomPoint, findRandomPointAroundCircle: points on passing polygons" {
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(0x72616e64));
    const random = prng.random();
    var draws: usize = 0;
    for (try pairs(), 0..) |*pair, level| {
        for (0..600) |case| {
            errdefer std.debug.print("  level {d}, case {d}\n", .{ level, case });
            const filters = Filters.forCase(pair, case);
            var ref: PolyRef = 0;
            var point: [3]i32 = @splat(0);
            const status = pair.query.findRandomPoint(&filters.int, random, &ref, &point);
            if (status.failed) continue;
            try testing.expect(try onPolygon(pair, ref, point));
            try testing.expect(passes(pair, ref, &filters.int));
            draws += 1;

            const start = pair.startAt(point, &filters) orelse continue;
            const radius = random.intRangeAtMost(i32, 200, 10000);
            var around_ref: PolyRef = 0;
            var around: [3]i32 = @splat(0);
            const around_status = pair.query.findRandomPointAroundCircle(
                start.ref,
                start.pos,
                radius,
                &filters.int,
                random,
                &around_ref,
                &around,
            );
            if (around_status.failed) continue;
            try testing.expect(try onPolygon(pair, around_ref, around));
            try testing.expect(passes(pair, around_ref, &filters.int));
            draws += 1;
        }
    }
    try testing.expect(draws >= case_count_min);
}

test "integer findRandomPoint: over many draws polygons are picked in proportion to their area" {
    // The one-tile level: the tile is always the one picked, so each
    // polygon's share is its share of the tile's area.
    const pair = &(try pairs())[1];
    const tile = &pair.nav.tiles[0];
    const gpa = testing.allocator;
    const areas = try gpa.alloc(f64, tile.polys.len);
    defer gpa.free(areas);
    const counts = try gpa.alloc(u32, tile.polys.len);
    defer gpa.free(counts);
    @memset(counts, 0);
    var total: f64 = 0;
    for (tile.polys, areas) |poly, *area| {
        area.* = 0;
        if (poly.getType() == .off_mesh_connection) continue;
        const a = tile.verts[poly.verts[0]];
        for (2..poly.vert_count) |i| {
            const b = tile.verts[poly.verts[i - 1]];
            const c = tile.verts[poly.verts[i]];
            area.* += @abs(@as(f64, @floatFromInt(integer.common.triArea2d(a, b, c)))) / 2.0;
        }
        total += area.*;
    }

    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(0x61726561));
    const random = prng.random();
    const filter: integer.QueryFilter = .{};
    const draw_count = 200_000;
    const base = pair.nav.getPolyRefBase(tile);
    for (0..draw_count) |_| {
        var ref: PolyRef = 0;
        var point: [3]i32 = @splat(0);
        try testing.expect(pair.query.findRandomPoint(&filter, random, &ref, &point).succeeded);
        counts[@intCast(ref - base)] += 1;
    }
    const n: f64 = @floatFromInt(draw_count);
    for (areas, counts, 0..) |area, count, p| {
        const share = area / total;
        const mean = n * share;
        const spread = @sqrt(n * share * (1 - share));
        const off = @abs(@as(f64, @floatFromInt(count)) - mean);
        if (off <= 5 * spread + 3) continue;
        std.debug.print("polygon {d}: {d} draws, {d:.1} expected\n", .{ p, count, mean });
        return error.TestUnexpectedResult;
    }
}

// ---------------------------------------------------------------------------
// The undulating level at its own place
// ---------------------------------------------------------------------------

/// Whether `p` lies on polygon `ref`, within tolerance along x and z.
fn liesOnPolygon(query: *const integer.NavMeshQuery, ref: PolyRef, p: [3]i32) bool {
    return liesNear(query, ref, p, tolerance);
}

/// Whether `p` lies within `limit` of polygon `ref` along x and z.
fn liesNear(query: *const integer.NavMeshQuery, ref: PolyRef, p: [3]i32, limit: f64) bool {
    var closest: [3]i32 = p;
    if (query.closestPointOnPoly(ref, p, &closest, null).failed) return false;
    const dx: f64 = @floatFromInt(@abs(closest[0] - p[0]));
    const dz: f64 = @floatFromInt(@abs(closest[2] - p[2]));
    return dx <= limit and dz <= limit;
}

/// How far past polygon `ref` a portal across a tile border can end: a
/// link stores where it covers the edge in 255ths of the edge, so up to a
/// 255th of the polygon's longest edge.
fn linkRounding(pair: *Pair, ref: PolyRef) f64 {
    const found = pair.nav.getTileAndPolyByRefUnsafe(ref);
    const poly = found.poly;
    var longest: u64 = 0;
    for (0..poly.vert_count) |i| {
        const a = found.tile.verts[poly.verts[i]];
        const b = found.tile.verts[poly.verts[(i + 1) % poly.vert_count]];
        longest = @max(longest, integer.common.vdist2dSqr(a, b));
    }
    return @sqrt(@as(f64, @floatFromInt(longest))) / 255.0;
}

/// Fails, naming `what`, unless `ok`.
fn expectHolds(what: []const u8, ok: bool) !void {
    if (ok) return;
    std.debug.print("not so: {s}\n", .{what});
    return error.TestUnexpectedResult;
}

/// Feeds each value's bytes to `hasher`, one value at a time so no padding
/// is read.
fn hashValues(hasher: *std.hash.Wyhash, values: anytype) void {
    inline for (values) |value| hasher.update(std.mem.asBytes(&value));
}

/// One run of the queries from each point to the next on the level at its
/// own place: the hash of all they return in call order. With `check`, each
/// query must succeed and its answer hold what the query promises.
fn runAtPlace(
    pair: *Pair,
    query: *integer.NavMeshQuery,
    points: []const [3]i32,
    check: bool,
) !u64 {
    const radius = 3000;
    const half_extents = [3]i32{ 2000, 4000, 2000 };
    const filter: integer.QueryFilter = .{};
    var hasher: std.hash.Wyhash = .init(0);
    var path: [result_count_max]PolyRef = undefined;
    var refs: [result_count_max]PolyRef = undefined;
    var parents: [result_count_max]PolyRef = undefined;
    var costs: [result_count_max]u64 = undefined;
    var corners: [result_count_max][3]i32 = undefined;
    var plane: [result_count_max][2]f64 = undefined;
    for (points[0 .. points.len - 1], points[1..], 0..) |a, b, i| {
        errdefer if (check) std.debug.print("  points {d} and {d}\n", .{ i, i + 1 });
        var start_ref: PolyRef = 0;
        var end_ref: PolyRef = 0;
        var from = a;
        var to = b;
        var over = false;
        const nearest = query.findNearestPoly(a, half_extents, &filter, &start_ref, &from, &over);
        _ = query.findNearestPoly(b, half_extents, &filter, &end_ref, &to, null);
        hashValues(&hasher, .{ statusBits(nearest), start_ref, end_ref, from, to, over });
        if (start_ref == 0 or end_ref == 0) {
            if (check) return error.TestNoPolygon;
            continue;
        }
        if (check) {
            try expectHolds("nearest start on its polygon", liesOnPolygon(query, start_ref, from));
            try expectHolds("nearest end on its polygon", liesOnPolygon(query, end_ref, to));
        }

        var height: i32 = 0;
        const height_status = query.getPolyHeight(start_ref, from, &height);
        hashValues(&hasher, .{ statusBits(height_status), height });
        // A point over the polygon lies inside it, and so has a height; a
        // nearest point on the boundary can round a hair outside.
        if (check and over) {
            try expectHolds("getPolyHeight succeeds", !height_status.failed);
            try expectHolds("height on the surface", @abs(height - from[1]) <= height_tolerance);
        }

        var path_count: usize = 0;
        const path_status =
            query.findPath(start_ref, end_ref, from, to, &filter, &path, &path_count);
        hashValues(&hasher, .{statusBits(path_status)});
        hasher.update(std.mem.sliceAsBytes(path[0..path_count]));
        if (check) {
            try expectHolds("findPath succeeds", !path_status.failed and path_count > 0);
            try expectHolds("corridor starts at the start", start_ref == path[0]);
        }
        if (path_count == 0) continue;

        var count: usize = 0;
        const straight = query.findStraightPath(
            from,
            to,
            path[0..path_count],
            &corners,
            null,
            &refs,
            &count,
            .{},
        );
        hashValues(&hasher, .{statusBits(straight)});
        hasher.update(std.mem.sliceAsBytes(corners[0..count]));
        if (check) {
            try expectHolds("findStraightPath succeeds", !straight.failed and count > 0);
            const first_on = liesOnPolygon(query, start_ref, corners[0]);
            try expectHolds("first corner on the start polygon", first_on);
            for (corners[0..count], refs[0..count]) |corner, ref| {
                if (ref == 0) continue;
                const on = liesNear(query, ref, corner, tolerance + linkRounding(pair, ref));
                try expectHolds("corner on its polygon", on);
            }
            const line = planeOf(i32, corners[0..count], &plane);
            const through = throughPortals(pair, path[0..path_count], line);
            try expectHolds("path through the portals", through);
        }

        var hit: integer.RaycastHit = .{
            .t = .zero,
            .hit_normal = @splat(0),
            .hit_edge_index = 0,
            .path = &refs,
            .path_count = 0,
            .path_cost = 0,
        };
        const ray_options: detour.RaycastOptions = .{ .use_costs = i % 2 == 0 };
        const ray = query.raycast(start_ref, from, to, &filter, ray_options, &hit, 0);
        const hit_point = hit.point(from, to);
        hashValues(&hasher, .{ statusBits(ray), hit.t.num, hit.t.den, hit.hit_normal });
        hasher.update(std.mem.sliceAsBytes(refs[0..hit.path_count]));
        if (check) {
            // A ray from a start on the boundary, rounded a hair outside,
            // misses the start polygon and stops at once.
            try expectHolds("raycast succeeds", !ray.failed and (hit.path_count > 0 or !over));
            if (hit.path_count > 0) {
                const last = refs[hit.path_count - 1];
                const hit_on = liesOnPolygon(query, last, hit_point);
                try expectHolds("hit point on the last polygon", hit_on);
            }
        }

        var walked: [3]i32 = from;
        count = 0;
        const move = query.moveAlongSurface(start_ref, from, to, &filter, &walked, &refs, &count);
        hashValues(&hasher, .{ statusBits(move), walked });
        hasher.update(std.mem.sliceAsBytes(refs[0..count]));
        if (check) {
            try expectHolds("moveAlongSurface succeeds", !move.failed and count > 0);
            const walked_on = liesOnPolygon(query, refs[count - 1], walked);
            try expectHolds("walked point on the last polygon", walked_on);
        }

        count = 0;
        const around = query.findPolysAroundCircle(
            start_ref,
            from,
            radius,
            &filter,
            &refs,
            &parents,
            &costs,
            &count,
            result_count_max,
        );
        hashValues(&hasher, .{statusBits(around)});
        hasher.update(std.mem.sliceAsBytes(refs[0..count]));
        hasher.update(std.mem.sliceAsBytes(costs[0..count]));
        if (check) {
            try expectHolds("findPolysAroundCircle succeeds", !around.failed and count > 0);
            try expectHolds("search starts at the start", start_ref == refs[0]);
        }

        var wall_distance: i32 = 0;
        var wall: [3]i32 = @splat(0);
        var normal: [3]i32 = @splat(0);
        const wall_status = query.findDistanceToWall(
            start_ref,
            from,
            radius,
            &filter,
            &wall_distance,
            &wall,
            &normal,
        );
        hashValues(&hasher, .{ statusBits(wall_status), wall_distance, wall, normal });
        if (check) {
            try expectHolds("findDistanceToWall succeeds", !wall_status.failed);
            const within = wall_distance >= 0 and wall_distance <= radius;
            try expectHolds("wall within the radius", within);
            if (wall_distance < radius) {
                const dx: f64 = @floatFromInt(wall[0] - from[0]);
                const dz: f64 = @floatFromInt(wall[2] - from[2]);
                const reach = @sqrt(dx * dx + dz * dz);
                const off = @abs(reach - @as(f64, @floatFromInt(wall_distance)));
                try expectHolds("wall point at the distance", off <= tolerance);
            }
        }
    }
    return hasher.final();
}

/// One thread's run over the level at its own place.
fn runAtPlaceOnThread(
    pair: *Pair,
    nav: *const integer.NavMesh,
    points: []const [3]i32,
    hash: *u64,
) void {
    var query: integer.NavMeshQuery = .empty;
    if (query.init(std.heap.page_allocator, nav, 4096).failed) return;
    defer query.deinit(std.heap.page_allocator);
    hash.* = runAtPlace(pair, &query, points, false) catch 0;
}

// The float port is compared on this level 5 km nearer the origin. Here it
// stands where RecastDemo puts it, and the integer variant is checked
// against itself and against what each query promises.
test "integer undulating level at its own place: every query works, the same on every run" {
    const gpa = testing.allocator;
    const pair = try std.heap.page_allocator.create(Pair);
    defer std.heap.page_allocator.destroy(pair);
    try pair.init(
        std.heap.page_allocator,
        support.undulating_obj,
        support.tiled_config,
        .{ .odd_tile_cell_size = 0.2, .seed = 84 },
    );
    defer pair.deinit(std.heap.page_allocator);
    try testing.expect(@abs(pair.nav.getParams().origin[0]) > 4_000_000);

    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(0x706c616365));
    const points = try gpa.alloc([3]i32, 301);
    defer gpa.free(points);
    // Places on the ground with a polygon near: there every query has an
    // answer to give.
    const filter: integer.QueryFilter = .{};
    for (points) |*point| {
        point.* = while (true) {
            const p = pair.groundPoint(prng.random(), 0.3);
            var ref: PolyRef = 0;
            var on = p;
            _ = pair.query.findNearestPoly(p, .{ 2000, 4000, 2000 }, &filter, &ref, &on, null);
            if (ref != 0) break p;
        };
    }

    const first = try runAtPlace(pair, &pair.query, points, true);
    try testing.expectEqual(first, try runAtPlace(pair, &pair.query, points, false));
    var hash: u64 = 0;
    const thread =
        try std.Thread.spawn(.{}, runAtPlaceOnThread, .{ pair, &pair.nav, points, &hash });
    thread.join();
    try testing.expectEqual(first, hash);
}

// ---------------------------------------------------------------------------
// The cases set aside as unstable (skeptic)
// ---------------------------------------------------------------------------

// The tests above compare only the stable cases and count the others. On an
// unstable case the float port gives two or more answers for inputs a
// millimetre apart, so the integer answer must be one of them: its status
// and polygons one the float port gives for the input or a nudged input.

test "skeptic: integer findNearestPoly on unstable cases gives one of the float answers" {
    var unstable: usize = 0;
    var unmatched: usize = 0;
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(0x6e656172));
    const random = prng.random();
    for (try pairs(), 0..) |*pair, level| {
        for (0..400) |case| {
            const filters = Filters.forCase(pair, case);
            const p = pair.groundPoint(random, 1.5);
            const half_extents = extents_choices[random.uintLessThan(usize, extents_choices.len)];
            var answers: [5]NearestAnswer = undefined;
            answers[0] = floatNearest(pair, p, half_extents, &filters.float);
            var stable = true;
            for (nudges, answers[1..]) |by, *answer| {
                answer.* = floatNearest(pair, moved(p, by), half_extents, &filters.float);
                if (!answers[0].sameDiscrete(answer.*)) stable = false;
            }
            if (stable) continue;
            unstable += 1;

            var ref: PolyRef = 0;
            var point: [3]i32 = p;
            var over = false;
            const status =
                pair.query.findNearestPoly(p, half_extents, &filters.int, &ref, &point, &over);
            var matched = false;
            for (answers) |answer| {
                if (answer.status == statusBits(status) and answer.ref == ref) matched = true;
            }
            if (matched) continue;
            // The exemptions the stable cases have.
            if (ref != 0 and try boxesDiffer(pair, p, half_extents, &filters) and
                try nearBorder(pair, ref, p, half_extents)) continue;
            if (ref != 0 and answers[0].ref != 0 and
                try equallyNear(pair, p, answers[0].ref, ref)) continue;
            unmatched += 1;
            std.debug.print("findNearestPoly level {d} case {d}: integer ref {d} status 0x{x}, " ++
                "float refs {d} {d} {d} {d} {d}\n", .{
                level,          case,           ref,            statusBits(status),
                answers[0].ref, answers[1].ref, answers[2].ref, answers[3].ref,
                answers[4].ref,
            });
        }
    }
    std.debug.print("findNearestPoly: {d} unstable cases, {d} matched no float answer\n", .{
        unstable,
        unmatched,
    });
    try testing.expectEqual(@as(usize, 0), unmatched);
}

test "skeptic: integer raycast, moveAlongSurface on unstable cases give one of the float answers" {
    var unstable_rays: usize = 0;
    var unmatched_rays: usize = 0;
    var unstable_moves: usize = 0;
    var unmatched_moves: usize = 0;
    var prng: std.Random.DefaultPrng = .init(support.fuzzSeed(0x736b6570));
    const random = prng.random();
    var rays: [5]RayAnswer = undefined;
    var moves: [5]MoveAnswer = undefined;
    var path: [result_count_max]PolyRef = undefined;
    for (try pairs(), 0..) |*pair, level| {
        var case: usize = 0;
        var tries: usize = 0;
        while (case < 300 and tries < 3000) : (tries += 1) {
            const filters = Filters.forCase(pair, case);
            const start = pair.startAt(pair.groundPoint(random, 0.3), &filters) orelse continue;
            const reach: i32 = if (random.boolean()) 1500 else 8000;
            const end = [3]i32{
                start.pos[0] + random.intRangeAtMost(i32, -reach, reach),
                start.pos[1] + random.intRangeAtMost(i32, -300, 300),
                start.pos[2] + random.intRangeAtMost(i32, -reach, reach),
            };
            const options: detour.RaycastOptions = .{ .use_costs = case % 2 == 0 };
            case += 1;

            floatRay(pair, start, end, @splat(0), &filters.float, options, &rays[0]);
            var stable = true;
            for (nudges, rays[1..]) |by, *answer| {
                floatRay(pair, start, end, by, &filters.float, options, answer);
                if (!rays[0].sameDiscrete(answer)) stable = false;
            }
            if (!stable) {
                unstable_rays += 1;
                var hit: integer.RaycastHit = .{
                    .t = .zero,
                    .hit_normal = @splat(0),
                    .hit_edge_index = 0,
                    .path = &path,
                    .path_count = 0,
                    .path_cost = 0,
                };
                const status =
                    pair.query.raycast(start.ref, start.pos, end, &filters.int, options, &hit, 0);
                var matched = false;
                for (&rays) |*answer| {
                    if (answer.status == statusBits(status) and
                        answer.hit == !hit.t.isNever() and
                        answer.edge == hit.hit_edge_index and
                        std.mem.eql(PolyRef, answer.path[0..answer.count], path[0..hit.path_count]))
                    {
                        matched = true;
                    }
                }
                if (!matched) {
                    unmatched_rays += 1;
                    std.debug.print("raycast level {d} case {d}: integer {d} polygons, " ++
                        "edge {d}, hit {}; float {d} polygons, edge {d}, hit {}\n", .{
                        level,            case,          hit.path_count, hit.hit_edge_index,
                        !hit.t.isNever(), rays[0].count, rays[0].edge,   rays[0].hit,
                    });
                }
            }

            floatMove(pair, start, end, @splat(0), &filters.float, &moves[0]);
            stable = true;
            for (nudges, moves[1..]) |by, *answer| {
                floatMove(pair, start, end, by, &filters.float, answer);
                if (!moves[0].sameDiscrete(answer)) stable = false;
            }
            if (!stable) {
                unstable_moves += 1;
                var result: [3]i32 = @splat(0);
                var count: usize = 0;
                const status = pair.query.moveAlongSurface(
                    start.ref,
                    start.pos,
                    end,
                    &filters.int,
                    &result,
                    &path,
                    &count,
                );
                var matched = false;
                for (&moves) |*answer| {
                    if (answer.status == statusBits(status) and
                        std.mem.eql(PolyRef, answer.visited[0..answer.count], path[0..count]))
                    {
                        matched = true;
                    }
                }
                if (!matched) {
                    unmatched_moves += 1;
                    std.debug.print("moveAlongSurface level {d} case {d}: integer visited {d}, " ++
                        "float {d}\n", .{ level, case, count, moves[0].count });
                }
            }
        }
    }
    std.debug.print("unstable cases: raycast {d}, {d} unmatched; moveAlongSurface {d}, " ++
        "{d} unmatched\n", .{ unstable_rays, unmatched_rays, unstable_moves, unmatched_moves });
    try testing.expectEqual(@as(usize, 0), unmatched_rays);
    try testing.expectEqual(@as(usize, 0), unmatched_moves);
}
