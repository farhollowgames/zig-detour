//! Benchmark rows for the integer queries, timed against the float port's
//! same queries on the same level. The two need not agree bit for bit, so
//! the rows do not compare their checksums.

const std = @import("std");
const support = @import("support");
const Bench = @import("bench.zig").Bench;
const Row = @import("bench.zig").Row;

const detour = support.detour;
const integer = detour.integer;

const PolyRef = detour.PolyRef;

const node_count_max = 2048;
const query_count = 1000;
const point_seed = 0x696e7462656e6368;
const poly_count_max = 256;
const straight_count_max = 256;
const visited_count_max = 16;
const step_length = 0.5;
const circle_radius = 10.0;
const circle_radius_mm = 10_000;
const wall_radius = 20.0;
const wall_radius_mm = 20_000;
const half_extents: [3]f32 = .{ 2.0, 4.0, 2.0 };
const scale = integer.convert.default_scale;

inline fn statusSum(status: detour.Status) u64 {
    return @as(u32, @bitCast(status));
}

inline fn floatSum(value: f32) u64 {
    return @as(u32, @bitCast(value));
}

inline fn pointSum(point: [3]f32) u64 {
    return floatSum(point[0]) +% floatSum(point[1]) +% floatSum(point[2]);
}

inline fn intPointSum(point: [3]i32) u64 {
    const bits: [3]u32 = @bitCast(point);
    return @as(u64, bits[0]) +% bits[1] +% bits[2];
}

inline fn listSum(refs: []const PolyRef, count: usize) u64 {
    return count +% if (count == 0) 0 else @as(u64, refs[count - 1]);
}

fn millimetres(p: [3]f32) [3]i32 {
    var out: [3]i32 = undefined;
    for (&out, p) |*o, v| o.* = @intFromFloat(@round(@as(f64, v) * scale));
    return out;
}

/// The float port's mesh of the level and its conversion, a query of each,
/// and the same inputs in metres and in millimetres.
const Context = struct {
    nav: integer.NavMesh,
    float_query: detour.NavMeshQuery,
    query: integer.NavMeshQuery,
    float_filter: detour.QueryFilter,
    filter: integer.QueryFilter,

    /// Ground points with their polygon, and the same in millimetres.
    float_centers: [query_count][3]f32,
    centers: [query_count][3]i32,
    float_points: [query_count + 1][3]f32,
    points: [query_count + 1][3]i32,
    refs: [query_count + 1]PolyRef,
    float_steps: [query_count][3]f32,
    steps: [query_count][3]i32,
    /// The float port's path from each point to the next, which both sides'
    /// straight path rows walk.
    paths: []PolyRef,
    path_starts: [query_count + 1]usize,

    polys: [poly_count_max]PolyRef,
    parents: [poly_count_max]PolyRef,
    float_costs: [poly_count_max]f32,
    costs: [poly_count_max]u64,
    float_corners: [straight_count_max][3]f32,
    corners: [straight_count_max][3]i32,
    visited: [visited_count_max]PolyRef,

    fn init(self: *Context, gpa: std.mem.Allocator, world: *const support.World) !void {
        try self.convert(world);
        errdefer self.nav.deinit();
        self.float_query = .empty;
        if (self.float_query.init(std.heap.c_allocator, &world.nav, node_count_max).failed) {
            return error.OutOfMemory;
        }
        errdefer self.float_query.deinit(std.heap.c_allocator);
        self.query = .empty;
        if (self.query.init(std.heap.c_allocator, &self.nav, node_count_max).failed) {
            return error.OutOfMemory;
        }
        errdefer self.query.deinit(std.heap.c_allocator);
        self.float_filter = .{};
        self.filter = .{};
        self.polys = @splat(0);
        self.parents = @splat(0);
        self.float_costs = @splat(0);
        self.costs = @splat(0);
        self.float_corners = @splat(@splat(0));
        self.corners = @splat(@splat(0));
        self.visited = @splat(0);

        var prng: std.Random.DefaultPrng = .init(point_seed);
        const random = prng.random();
        var found: usize = 0;
        for (0..query_count * 64) |_| {
            if (found == query_count) break;
            const center = world.level.groundPoint(random, 0.3);
            var ref: PolyRef = 0;
            var point = center;
            _ = self.float_query.findNearestPoly(
                center,
                half_extents,
                &self.float_filter,
                &ref,
                &point,
                null,
            );
            if (ref == 0) continue;
            self.float_centers[found] = center;
            self.centers[found] = millimetres(center);
            self.float_points[found] = point;
            self.points[found] = millimetres(point);
            self.refs[found] = ref;
            found += 1;
        }
        if (found != query_count) return error.TooFewPoints;
        self.float_points[query_count] = self.float_points[0];
        self.points[query_count] = self.points[0];
        self.refs[query_count] = self.refs[0];

        for (&self.float_steps, &self.steps, 0..) |*float_step, *step, i| {
            const from = self.float_points[i];
            const to = self.float_points[i + 1];
            const delta: [3]f32 = .{ to[0] - from[0], to[1] - from[1], to[2] - from[2] };
            const length = @sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
            float_step.* = to;
            if (length > step_length) {
                const share = step_length / length;
                float_step.* = .{
                    from[0] + delta[0] * share,
                    from[1] + delta[1] * share,
                    from[2] + delta[2] * share,
                };
            }
            step.* = millimetres(float_step.*);
        }

        var paths: std.ArrayList(PolyRef) = .empty;
        errdefer paths.deinit(gpa);
        for (0..query_count) |i| {
            var count: usize = 0;
            _ = self.float_query.findPath(
                self.refs[i],
                self.refs[i + 1],
                self.float_points[i],
                self.float_points[i + 1],
                &self.float_filter,
                &self.polys,
                &count,
            );
            self.path_starts[i] = paths.items.len;
            try paths.appendSlice(gpa, self.polys[0..count]);
        }
        self.path_starts[query_count] = paths.items.len;
        self.paths = try paths.toOwnedSlice(gpa);
    }

    fn deinit(self: *Context, gpa: std.mem.Allocator) void {
        gpa.free(self.paths);
        self.query.deinit(std.heap.c_allocator);
        self.float_query.deinit(std.heap.c_allocator);
        self.nav.deinit();
        self.* = undefined;
    }

    /// Converts every tile of the float mesh, in the order of its entries,
    /// so that both meshes hand out the same references.
    fn convert(self: *Context, world: *const support.World) !void {
        const gpa = std.heap.c_allocator;
        const params = world.nav.getParams();
        const integer_params: integer.NavMeshParams = .{
            .origin = millimetres(params.origin),
            .tile_width = @intFromFloat(@round(@as(f64, params.tile_width) * scale)),
            .tile_height = @intFromFloat(@round(@as(f64, params.tile_height) * scale)),
            .tile_count_max = params.tile_count_max,
            .poly_count_max = params.poly_count_max,
        };
        self.nav = .empty;
        if (self.nav.init(gpa, &integer_params).failed) return error.OutOfMemory;
        errdefer self.nav.deinit();
        for (world.nav.tiles) |*tile| {
            if (tile.header == null) continue;
            const data = try integer.convert.tileFromFloat(gpa, tile.data, scale);
            if (self.nav.addTile(data, .{ .free_data = true }, 0, null).failed) {
                gpa.free(data);
                return error.TileRefused;
            }
        }
    }

    fn path(self: *const Context, i: usize) []const PolyRef {
        return self.paths[self.path_starts[i]..self.path_starts[i + 1]];
    }

    fn floatFindNearestPoly(context: *Context) u64 {
        var sum: u64 = 0;
        var ref: PolyRef = 0;
        var point: [3]f32 = @splat(0);
        for (context.float_centers) |center| {
            sum +%= statusSum(context.float_query.findNearestPoly(
                center,
                half_extents,
                &context.float_filter,
                &ref,
                &point,
                null,
            ));
            sum +%= ref +% pointSum(point);
        }
        return sum;
    }

    fn integerFindNearestPoly(context: *Context) u64 {
        var sum: u64 = 0;
        var ref: PolyRef = 0;
        var point: [3]i32 = @splat(0);
        const extents = millimetres(half_extents);
        for (context.centers) |center| {
            sum +%= statusSum(context.query.findNearestPoly(
                center,
                extents,
                &context.filter,
                &ref,
                &point,
                null,
            ));
            sum +%= ref +% intPointSum(point);
        }
        return sum;
    }

    fn floatGetPolyHeight(context: *Context) u64 {
        var sum: u64 = 0;
        var height: f32 = 0;
        for (0..query_count) |i| {
            const p = context.float_points[i];
            sum +%= statusSum(context.float_query.getPolyHeight(context.refs[i], p, &height));
            sum +%= floatSum(height);
        }
        return sum;
    }

    fn integerGetPolyHeight(context: *Context) u64 {
        var sum: u64 = 0;
        var height: i32 = 0;
        for (0..query_count) |i| {
            const p = context.points[i];
            sum +%= statusSum(context.query.getPolyHeight(context.refs[i], p, &height));
            sum +%= @as(u32, @bitCast(height));
        }
        return sum;
    }

    fn floatFindPath(context: *Context) u64 {
        var sum: u64 = 0;
        var count: usize = 0;
        for (0..query_count) |i| {
            sum +%= statusSum(context.float_query.findPath(
                context.refs[i],
                context.refs[i + 1],
                context.float_points[i],
                context.float_points[i + 1],
                &context.float_filter,
                &context.polys,
                &count,
            ));
            sum +%= listSum(&context.polys, count);
        }
        return sum;
    }

    fn integerFindPath(context: *Context) u64 {
        var sum: u64 = 0;
        var count: usize = 0;
        for (0..query_count) |i| {
            sum +%= statusSum(context.query.findPath(
                context.refs[i],
                context.refs[i + 1],
                context.points[i],
                context.points[i + 1],
                &context.filter,
                &context.polys,
                &count,
            ));
            sum +%= listSum(&context.polys, count);
        }
        return sum;
    }

    fn floatFindStraightPath(context: *Context) u64 {
        var sum: u64 = 0;
        var count: usize = 0;
        for (0..query_count) |i| {
            sum +%= statusSum(context.float_query.findStraightPath(
                context.float_points[i],
                context.float_points[i + 1],
                context.path(i),
                &context.float_corners,
                null,
                null,
                &count,
                .{},
            ));
            sum +%= count;
            if (count > 0) sum +%= pointSum(context.float_corners[count - 1]);
        }
        return sum;
    }

    fn integerFindStraightPath(context: *Context) u64 {
        var sum: u64 = 0;
        var count: usize = 0;
        for (0..query_count) |i| {
            sum +%= statusSum(context.query.findStraightPath(
                context.points[i],
                context.points[i + 1],
                context.path(i),
                &context.corners,
                null,
                null,
                &count,
                .{},
            ));
            sum +%= count;
            if (count > 0) sum +%= intPointSum(context.corners[count - 1]);
        }
        return sum;
    }

    fn floatMoveAlongSurface(context: *Context) u64 {
        var sum: u64 = 0;
        var result: [3]f32 = @splat(0);
        var count: usize = 0;
        for (0..query_count) |i| {
            sum +%= statusSum(context.float_query.moveAlongSurface(
                context.refs[i],
                context.float_points[i],
                context.float_steps[i],
                &context.float_filter,
                &result,
                &context.visited,
                &count,
            ));
            sum +%= pointSum(result) +% listSum(&context.visited, count);
        }
        return sum;
    }

    fn integerMoveAlongSurface(context: *Context) u64 {
        var sum: u64 = 0;
        var result: [3]i32 = @splat(0);
        var count: usize = 0;
        for (0..query_count) |i| {
            sum +%= statusSum(context.query.moveAlongSurface(
                context.refs[i],
                context.points[i],
                context.steps[i],
                &context.filter,
                &result,
                &context.visited,
                &count,
            ));
            sum +%= intPointSum(result) +% listSum(&context.visited, count);
        }
        return sum;
    }

    fn floatRaycast(context: *Context) u64 {
        var sum: u64 = 0;
        var hit: detour.RaycastHit = .{
            .t = 0,
            .hit_normal = @splat(0),
            .hit_edge_index = 0,
            .path = &context.polys,
            .path_count = 0,
            .path_cost = 0,
        };
        for (0..query_count) |i| {
            sum +%= statusSum(context.float_query.raycast(
                context.refs[i],
                context.float_points[i],
                context.float_points[i + 1],
                &context.float_filter,
                .{},
                &hit,
                0,
            ));
            sum +%= floatSum(hit.t) +% listSum(&context.polys, hit.path_count);
        }
        return sum;
    }

    fn integerRaycast(context: *Context) u64 {
        var sum: u64 = 0;
        var hit: integer.RaycastHit = .{
            .t = .zero,
            .hit_normal = @splat(0),
            .hit_edge_index = 0,
            .path = &context.polys,
            .path_count = 0,
            .path_cost = 0,
        };
        for (0..query_count) |i| {
            sum +%= statusSum(context.query.raycast(
                context.refs[i],
                context.points[i],
                context.points[i + 1],
                &context.filter,
                .{},
                &hit,
                0,
            ));
            sum +%= @as(u64, @bitCast(hit.t.num)) +% listSum(&context.polys, hit.path_count);
        }
        return sum;
    }

    fn floatFindPolysAroundCircle(context: *Context) u64 {
        var sum: u64 = 0;
        var count: usize = 0;
        for (0..query_count) |i| {
            sum +%= statusSum(context.float_query.findPolysAroundCircle(
                context.refs[i],
                context.float_points[i],
                circle_radius,
                &context.float_filter,
                &context.polys,
                &context.parents,
                &context.float_costs,
                &count,
                poly_count_max,
            ));
            sum +%= listSum(&context.polys, count);
        }
        return sum;
    }

    fn integerFindPolysAroundCircle(context: *Context) u64 {
        var sum: u64 = 0;
        var count: usize = 0;
        for (0..query_count) |i| {
            sum +%= statusSum(context.query.findPolysAroundCircle(
                context.refs[i],
                context.points[i],
                circle_radius_mm,
                &context.filter,
                &context.polys,
                &context.parents,
                &context.costs,
                &count,
                poly_count_max,
            ));
            sum +%= listSum(&context.polys, count);
        }
        return sum;
    }

    fn floatFindDistanceToWall(context: *Context) u64 {
        var sum: u64 = 0;
        var distance: f32 = 0;
        var point: [3]f32 = @splat(0);
        var normal: [3]f32 = @splat(0);
        for (0..query_count) |i| {
            sum +%= statusSum(context.float_query.findDistanceToWall(
                context.refs[i],
                context.float_points[i],
                wall_radius,
                &context.float_filter,
                &distance,
                &point,
                &normal,
            ));
            sum +%= floatSum(distance) +% pointSum(point);
        }
        return sum;
    }

    fn integerFindDistanceToWall(context: *Context) u64 {
        var sum: u64 = 0;
        var distance: i32 = 0;
        var point: [3]i32 = @splat(0);
        var normal: [3]i32 = @splat(0);
        for (0..query_count) |i| {
            sum +%= statusSum(context.query.findDistanceToWall(
                context.refs[i],
                context.points[i],
                wall_radius_mm,
                &context.filter,
                &distance,
                &point,
                &normal,
            ));
            sum +%= @as(u32, @bitCast(distance)) +% intPointSum(point);
        }
        return sum;
    }
};

fn queryRow(name: []const u8) Row {
    return .{
        .name = name,
        .per = "query",
        .count = query_count,
        .baseline = "float port",
        .same_results = false,
    };
}

pub fn run(bench: *Bench) !void {
    try bench.group("Integer queries against the float port");

    // The tiled level and its conversion to millimetres; the same ground
    // points on both sides, each with the polygon the float port finds
    // nearest it, and pairs of them for the rows that take two.
    var world: support.World = undefined;
    try world.init(std.heap.c_allocator, support.nav_test_obj, support.tiled_config, .{});
    defer world.deinit();
    const context = try bench.gpa.create(Context);
    defer bench.gpa.destroy(context);
    try context.init(bench.gpa, &world);
    defer context.deinit(bench.gpa);

    try bench.compare(
        queryRow("integer NavMeshQuery.findNearestPoly"),
        context,
        Context.floatFindNearestPoly,
        Context.integerFindNearestPoly,
    );
    try bench.compare(
        queryRow("integer NavMeshQuery.getPolyHeight"),
        context,
        Context.floatGetPolyHeight,
        Context.integerGetPolyHeight,
    );
    try bench.compare(
        queryRow("integer NavMeshQuery.findPath"),
        context,
        Context.floatFindPath,
        Context.integerFindPath,
    );
    // Both sides walk the corridors the float port found.
    try bench.compare(
        queryRow("integer NavMeshQuery.findStraightPath"),
        context,
        Context.floatFindStraightPath,
        Context.integerFindStraightPath,
    );
    try bench.compare(
        queryRow("integer NavMeshQuery.moveAlongSurface"),
        context,
        Context.floatMoveAlongSurface,
        Context.integerMoveAlongSurface,
    );
    try bench.compare(
        queryRow("integer NavMeshQuery.raycast"),
        context,
        Context.floatRaycast,
        Context.integerRaycast,
    );
    try bench.compare(
        queryRow("integer NavMeshQuery.findPolysAroundCircle"),
        context,
        Context.floatFindPolysAroundCircle,
        Context.integerFindPolysAroundCircle,
    );
    try bench.compare(
        queryRow("integer NavMeshQuery.findDistanceToWall"),
        context,
        Context.floatFindDistanceToWall,
        Context.integerFindDistanceToWall,
    );
}
