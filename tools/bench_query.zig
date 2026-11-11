//! Benchmark rows for the navigation mesh queries.

const std = @import("std");
const support = @import("support");
const Bench = @import("bench.zig").Bench;
const Row = @import("bench.zig").Row;

const detour = support.detour;
const reference = support.reference;

const PolyRef = detour.PolyRef;

/// How many nodes each side's query searches with.
const node_count_max = 2048;
/// How many points the rows run over, and how many pairs of them.
const query_count = 1000;
/// The seed of the points.
const point_seed = 0x71756572796273;
/// Half the box searched around a point for its polygon.
const half_extents: [3]f32 = .{ 2.0, 4.0, 2.0 };
/// How many polygons a path or a list of found polygons holds.
const poly_count_max = 256;
/// How many corners a straight path holds.
const straight_count_max = 256;
/// How many polygons `moveAlongSurface` lists: what a path corridor asks for.
const visited_count_max = 16;
/// How many wall segments of one polygon are asked for: what the crowd's
/// local boundary asks for.
const segment_count_max = 18;
/// The iterations one `updateSlicedFindPath` call may spend.
const sliced_iteration_count = 64;
/// How far `moveAlongSurface` is asked to go.
const step_length = 0.5;
/// The radius of `findPolysAroundCircle` and `findRandomPointAroundCircle`.
const circle_radius = 10.0;
/// Half the side of the square `findPolysAroundShape` searches.
const square_half_side = 5.0;
/// The radius of `findLocalNeighbourhood`.
const neighbourhood_radius = 5.0;
/// The radius of `findDistanceToWall`.
const wall_radius = 20.0;
/// The radius of the search `getPathFromDijkstraSearch` reads.
const dijkstra_radius = 30.0;
/// The seed both sides' generators start from in the random point rows.
const random_seed = 0x72616e646f6d;
/// How many nodes the node rows take from a pool or push to a queue.
const node_row_count = 1024;
/// The hash buckets of the node rows' pools: what a query gives a pool of
/// `node_count_max` nodes.
const node_hash_size = 512;
/// The seed of the node rows' polygon references and totals.
const node_seed = 0x6e6f646573;

/// A status as a checksum takes it.
inline fn statusSum(status: detour.Status) u64 {
    return @as(u32, @bitCast(status));
}

/// The bits of a float as a checksum takes them.
inline fn floatSum(value: f32) u64 {
    return @as(u32, @bitCast(value));
}

/// The bits of a point's three floats, added up.
inline fn pointSum(point: [3]f32) u64 {
    return floatSum(point[0]) +% floatSum(point[1]) +% floatSum(point[2]);
}

/// What a call that wrote `count` references to `refs` adds to a checksum:
/// the count, the last reference and the middle one.
inline fn listSum(refs: []const PolyRef, count: usize) u64 {
    if (count == 0) return 0;
    return @as(u64, count) +% refs[count - 1] +% refs[count / 2];
}

/// Adds up the polygon references a polygon query hands over: the receiver
/// both sides of the `queryPolygonsWith` row give.
const RefCollector = struct {
    sum: u64 = 0,

    const port_vtable: detour.PolyQuery.VTable = .{ .process = portProcess };

    fn portProcess(
        ptr: *anyopaque,
        tile: *const detour.MeshTile,
        polys: []const *const detour.Poly,
        refs: []const PolyRef,
    ) void {
        _ = tile;
        _ = polys;
        const self: *RefCollector = @ptrCast(@alignCast(ptr));
        for (refs) |ref| self.sum +%= ref;
    }

    fn baselineProcess(
        user: ?*anyopaque,
        tile: *const reference.MeshTile,
        polys: [*]*reference.Poly,
        refs: [*]PolyRef,
        count: i32,
    ) callconv(.c) void {
        _ = tile;
        _ = polys;
        const self: *RefCollector = @ptrCast(@alignCast(user.?));
        for (refs[0..@intCast(count)]) |ref| self.sum +%= ref;
    }
};

/// Both sides' queries and filters over one world, the inputs the rows run
/// over and the buffers the calls write to.
const Context = struct {
    query: detour.NavMeshQuery,
    reference_query: *reference.NavMeshQuery,
    filter: detour.QueryFilter,
    reference_filter: *reference.QueryFilter,

    /// The original's option bits, as its callers pass them.
    reference_any_angle: u32,
    reference_use_costs: u32,
    reference_all_crossings: i32,

    /// Points of the level's ground with a polygon in reach.
    centers: [query_count][3]f32,
    /// Each center moved onto the polygon nearest it. The entry after the
    /// last repeats the first, so point `i` always has a point `i + 1`.
    points: [query_count + 1][3]f32,
    /// The polygon of each point.
    refs: [query_count + 1]PolyRef,
    /// Each point's tile and polygon in the port's mesh and the original's.
    mesh_tiles: [query_count + 1]*const detour.MeshTile,
    mesh_polys: [query_count + 1]*const detour.Poly,
    reference_mesh_tiles: [query_count + 1]*const reference.MeshTile,
    reference_mesh_polys: [query_count + 1]*const reference.Poly,
    /// Each point moved `step_length` toward the next point, or the next
    /// point where that is nearer.
    steps: [query_count][3]f32,
    /// The square of half side `square_half_side` around each point, its
    /// corners in the order the shape search takes them.
    shapes: [query_count][4][3]f32,

    /// The polygons of the path from each point to the next: path `i` is
    /// `paths[path_starts[i]..path_starts[i + 1]]`.
    paths: []PolyRef,
    path_starts: [query_count + 1]usize,
    /// Where path `i` ends: the next point, or the point nearest it on the
    /// path's last polygon when the path stops short of the next point.
    path_ends: [query_count][3]f32,

    /// Polygons reached by the search `leaveDijkstraSearch` leaves in both
    /// queries.
    search_ends: [query_count]PolyRef,
    /// Polygons that search reached, in turn with the polygons of points.
    closed_asks: [query_count]PolyRef,

    polys: [poly_count_max]PolyRef,
    parents: [poly_count_max]PolyRef,
    costs: [poly_count_max]f32,
    straight_points: [straight_count_max][3]f32,
    straight_flags: [straight_count_max]detour.StraightPathFlags,
    straight_refs: [straight_count_max]PolyRef,
    visited: [visited_count_max]PolyRef,
    segments: [segment_count_max][2][3]f32,
    segment_refs: [segment_count_max]PolyRef,

    /// Attaches a query of each side to `world` and prepares the inputs.
    /// `gpa` holds the paths. Free with `deinit`.
    fn init(self: *Context, gpa: std.mem.Allocator, world: *const support.World) !void {
        self.query = .empty;
        if (self.query.init(std.heap.c_allocator, &world.nav, node_count_max).failed) {
            return error.QueryInitFailed;
        }
        errdefer self.query.deinit(std.heap.c_allocator);
        self.filter = .{};

        self.reference_query = reference.ref_query_alloc() orelse return error.OutOfMemory;
        errdefer reference.ref_query_free(self.reference_query);
        const attached: detour.Status = @bitCast(
            reference.ref_query_init(self.reference_query, world.reference_nav, node_count_max),
        );
        if (attached.failed) return error.QueryInitFailed;
        self.reference_filter = reference.ref_filter_create() orelse return error.OutOfMemory;
        errdefer reference.ref_filter_destroy(self.reference_filter);

        self.reference_any_angle = reference.ref_const_findpath_any_angle();
        self.reference_use_costs = reference.ref_const_raycast_use_costs();
        self.reference_all_crossings =
            @intCast(reference.ref_const_straightpath_all_crossings());

        self.search_ends = @splat(0);
        self.closed_asks = @splat(0);
        self.polys = @splat(0);
        self.parents = @splat(0);
        self.costs = @splat(0);
        self.straight_points = @splat(@splat(0));
        self.straight_flags = @splat(.{});
        self.straight_refs = @splat(0);
        self.visited = @splat(0);
        self.segments = @splat(@splat(@splat(0)));
        self.segment_refs = @splat(0);

        try self.drawPoints(world);
        try self.resolvePolygons(world);
        self.prepareMoves();
        try self.findPaths(gpa);
    }

    fn deinit(self: *Context, gpa: std.mem.Allocator) void {
        gpa.free(self.paths);
        reference.ref_filter_destroy(self.reference_filter);
        reference.ref_query_free(self.reference_query);
        self.query.deinit(std.heap.c_allocator);
        self.* = undefined;
    }

    /// Draws ground points until `query_count` of them have a polygon in
    /// reach, and keeps each with its polygon and its point on the polygon.
    fn drawPoints(self: *Context, world: *const support.World) !void {
        var prng: std.Random.DefaultPrng = .init(point_seed);
        const random = prng.random();
        var found: usize = 0;
        for (0..query_count * 64) |_| {
            if (found == query_count) break;
            const center = world.level.groundPoint(random, 0.3);
            var ref: PolyRef = 0;
            var point = center;
            _ = reference.ref_query_find_nearest_poly(
                self.reference_query,
                &center,
                &half_extents,
                self.reference_filter,
                &ref,
                &point,
            );
            if (ref == 0) continue;
            self.centers[found] = center;
            self.points[found] = point;
            self.refs[found] = ref;
            found += 1;
        }
        if (found != query_count) return error.TooFewPoints;
        self.points[query_count] = self.points[0];
        self.refs[query_count] = self.refs[0];
    }

    /// Looks up each point's tile and polygon in both meshes.
    fn resolvePolygons(self: *Context, world: *const support.World) !void {
        for (self.refs, 0..) |ref, i| {
            const resolved = world.nav.getTileAndPolyByRef(
                ref,
                &self.mesh_tiles[i],
                &self.mesh_polys[i],
            );
            if (resolved.failed) return error.PolygonNotFound;

            var tile: ?*const reference.MeshTile = null;
            var poly: ?*const reference.Poly = null;
            _ = reference.ref_nav_mesh_get_tile_and_poly_by_ref(
                world.reference_nav,
                ref,
                &tile,
                &poly,
            );
            self.reference_mesh_tiles[i] = tile orelse return error.PolygonNotFound;
            self.reference_mesh_polys[i] = poly orelse return error.PolygonNotFound;
        }
    }

    /// Fills `steps` and `shapes` from the points.
    fn prepareMoves(self: *Context) void {
        for (&self.steps, &self.shapes, 0..) |*step, *shape, i| {
            const from = self.points[i];
            const to = self.points[i + 1];
            const delta: [3]f32 = .{ to[0] - from[0], to[1] - from[1], to[2] - from[2] };
            const length = @sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
            step.* = to;
            if (length > step_length) {
                const scale = step_length / length;
                step.* = .{
                    from[0] + delta[0] * scale,
                    from[1] + delta[1] * scale,
                    from[2] + delta[2] * scale,
                };
            }

            const h = square_half_side;
            shape.* = .{
                .{ from[0] - h, from[1], from[2] - h },
                .{ from[0] - h, from[1], from[2] + h },
                .{ from[0] + h, from[1], from[2] + h },
                .{ from[0] + h, from[1], from[2] - h },
            };
        }
    }

    /// Finds the path from each point to the next with the original and
    /// keeps them all in `paths`, with where each ends.
    fn findPaths(self: *Context, gpa: std.mem.Allocator) !void {
        var paths: std.ArrayList(PolyRef) = .empty;
        errdefer paths.deinit(gpa);
        for (0..query_count) |i| {
            var count: i32 = 0;
            _ = reference.ref_query_find_path(
                self.reference_query,
                self.refs[i],
                self.refs[i + 1],
                &self.points[i],
                &self.points[i + 1],
                self.reference_filter,
                &self.polys,
                &count,
                poly_count_max,
            );
            if (count <= 0) return error.NoPath;
            const path = self.polys[0..@intCast(count)];
            self.path_starts[i] = paths.items.len;
            try paths.appendSlice(gpa, path);

            const last = path[path.len - 1];
            self.path_ends[i] = self.points[i + 1];
            if (last != self.refs[i + 1]) {
                _ = reference.ref_query_closest_point_on_poly(
                    self.reference_query,
                    last,
                    &self.points[i + 1],
                    &self.path_ends[i],
                    null,
                );
            }
        }
        self.path_starts[query_count] = paths.items.len;
        self.paths = try paths.toOwnedSlice(gpa);
    }

    fn baselineFindNearestPoly(context: *Context) u64 {
        var sum: u64 = 0;
        var nearest_ref: PolyRef = 0;
        var nearest_pt: [3]f32 = @splat(0);
        for (&context.centers) |*center| {
            sum +%= reference.ref_query_find_nearest_poly(
                context.reference_query,
                center,
                &half_extents,
                context.reference_filter,
                &nearest_ref,
                &nearest_pt,
            );
            sum +%= nearest_ref +% pointSum(nearest_pt);
        }
        return sum;
    }

    fn portFindNearestPoly(context: *Context) u64 {
        var sum: u64 = 0;
        var nearest_ref: PolyRef = 0;
        var nearest_pt: [3]f32 = @splat(0);
        for (context.centers) |center| {
            sum +%= statusSum(context.query.findNearestPoly(
                center,
                half_extents,
                &context.filter,
                &nearest_ref,
                &nearest_pt,
                null,
            ));
            sum +%= nearest_ref +% pointSum(nearest_pt);
        }
        return sum;
    }

    fn baselineQueryPolygons(context: *Context) u64 {
        var sum: u64 = 0;
        var count: i32 = 0;
        for (&context.centers) |*center| {
            sum +%= reference.ref_query_query_polygons(
                context.reference_query,
                center,
                &half_extents,
                context.reference_filter,
                &context.polys,
                &count,
                poly_count_max,
            );
            sum +%= listSum(&context.polys, @intCast(count));
        }
        return sum;
    }

    fn portQueryPolygons(context: *Context) u64 {
        var sum: u64 = 0;
        var count: usize = 0;
        for (context.centers) |center| {
            sum +%= statusSum(context.query.queryPolygons(
                center,
                half_extents,
                &context.filter,
                &context.polys,
                &count,
            ));
            sum +%= listSum(&context.polys, count);
        }
        return sum;
    }

    fn baselineQueryPolygonsWith(context: *Context) u64 {
        var sum: u64 = 0;
        var collector: RefCollector = .{};
        for (&context.centers) |*center| {
            sum +%= reference.ref_query_query_polygons_cb(
                context.reference_query,
                center,
                &half_extents,
                context.reference_filter,
                RefCollector.baselineProcess,
                &collector,
            );
        }
        return sum +% collector.sum;
    }

    fn portQueryPolygonsWith(context: *Context) u64 {
        var sum: u64 = 0;
        var collector: RefCollector = .{};
        const receiver: detour.PolyQuery = .{
            .ptr = &collector,
            .vtable = &RefCollector.port_vtable,
        };
        for (context.centers) |center| {
            sum +%= statusSum(context.query.queryPolygonsWith(
                center,
                half_extents,
                &context.filter,
                receiver,
            ));
        }
        return sum +% collector.sum;
    }

    fn baselineClosestPointOnPoly(context: *Context) u64 {
        var sum: u64 = 0;
        var closest: [3]f32 = @splat(0);
        var over_poly = false;
        for (0..query_count) |i| {
            sum +%= reference.ref_query_closest_point_on_poly(
                context.reference_query,
                context.refs[i],
                &context.centers[i],
                &closest,
                &over_poly,
            );
            sum +%= pointSum(closest) +% @intFromBool(over_poly);
        }
        return sum;
    }

    fn portClosestPointOnPoly(context: *Context) u64 {
        var sum: u64 = 0;
        var closest: [3]f32 = @splat(0);
        var over_poly = false;
        for (0..query_count) |i| {
            sum +%= statusSum(context.query.closestPointOnPoly(
                context.refs[i],
                context.centers[i],
                &closest,
                &over_poly,
            ));
            sum +%= pointSum(closest) +% @intFromBool(over_poly);
        }
        return sum;
    }

    fn baselineClosestPointOnPolyBoundary(context: *Context) u64 {
        var sum: u64 = 0;
        var closest: [3]f32 = @splat(0);
        for (0..query_count) |i| {
            sum +%= reference.ref_query_closest_point_on_poly_boundary(
                context.reference_query,
                context.refs[i],
                &context.centers[i],
                &closest,
            );
            sum +%= pointSum(closest);
        }
        return sum;
    }

    fn portClosestPointOnPolyBoundary(context: *Context) u64 {
        var sum: u64 = 0;
        var closest: [3]f32 = @splat(0);
        for (0..query_count) |i| {
            sum +%= statusSum(context.query.closestPointOnPolyBoundary(
                context.refs[i],
                context.centers[i],
                &closest,
            ));
            sum +%= pointSum(closest);
        }
        return sum;
    }

    fn baselineGetPolyHeight(context: *Context) u64 {
        var sum: u64 = 0;
        var height: f32 = 0;
        for (0..query_count) |i| {
            sum +%= reference.ref_query_get_poly_height(
                context.reference_query,
                context.refs[i],
                &context.points[i],
                &height,
            );
            sum +%= floatSum(height);
        }
        return sum;
    }

    fn portGetPolyHeight(context: *Context) u64 {
        var sum: u64 = 0;
        var height: f32 = 0;
        for (0..query_count) |i| {
            sum +%= statusSum(context.query.getPolyHeight(
                context.refs[i],
                context.points[i],
                &height,
            ));
            sum +%= floatSum(height);
        }
        return sum;
    }

    fn baselineIsValidPolyRef(context: *Context) u64 {
        var sum: u64 = 0;
        for (0..query_count) |i| {
            sum +%= @intFromBool(reference.ref_query_is_valid_poly_ref(
                context.reference_query,
                context.refs[i],
                context.reference_filter,
            ));
        }
        return sum;
    }

    fn portIsValidPolyRef(context: *Context) u64 {
        var sum: u64 = 0;
        for (0..query_count) |i| {
            sum +%= @intFromBool(context.query.isValidPolyRef(context.refs[i], &context.filter));
        }
        return sum;
    }

    fn baselineFindPath(context: *Context) u64 {
        var sum: u64 = 0;
        var count: i32 = 0;
        for (0..query_count) |i| {
            sum +%= reference.ref_query_find_path(
                context.reference_query,
                context.refs[i],
                context.refs[i + 1],
                &context.points[i],
                &context.points[i + 1],
                context.reference_filter,
                &context.polys,
                &count,
                poly_count_max,
            );
            sum +%= listSum(&context.polys, @intCast(count));
        }
        return sum;
    }

    fn portFindPath(context: *Context) u64 {
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

    /// A whole sliced search for each pair of points: started, updated
    /// `sliced_iteration_count` iterations a call until it is no longer in
    /// progress, and finalized.
    inline fn baselineSlicedFindPathWith(context: *Context, options: u32) u64 {
        var sum: u64 = 0;
        var count: i32 = 0;
        var done: i32 = 0;
        for (0..query_count) |i| {
            sum +%= reference.ref_query_init_sliced_find_path(
                context.reference_query,
                context.refs[i],
                context.refs[i + 1],
                &context.points[i],
                &context.points[i + 1],
                context.reference_filter,
                options,
            );
            while (true) {
                const status: detour.Status = @bitCast(reference.ref_query_update_sliced_find_path(
                    context.reference_query,
                    sliced_iteration_count,
                    &done,
                ));
                sum +%= statusSum(status) +% @as(u32, @bitCast(done));
                if (!status.in_progress) break;
            }
            sum +%= reference.ref_query_finalize_sliced_find_path(
                context.reference_query,
                &context.polys,
                &count,
                poly_count_max,
            );
            sum +%= listSum(&context.polys, @intCast(count));
        }
        return sum;
    }

    inline fn portSlicedFindPathWith(context: *Context, options: detour.FindPathOptions) u64 {
        var sum: u64 = 0;
        var count: usize = 0;
        var done: i32 = 0;
        for (0..query_count) |i| {
            sum +%= statusSum(context.query.initSlicedFindPath(
                context.refs[i],
                context.refs[i + 1],
                context.points[i],
                context.points[i + 1],
                &context.filter,
                options,
            ));
            while (true) {
                const status = context.query.updateSlicedFindPath(sliced_iteration_count, &done);
                sum +%= statusSum(status) +% @as(u32, @bitCast(done));
                if (!status.in_progress) break;
            }
            sum +%= statusSum(context.query.finalizeSlicedFindPath(&context.polys, &count));
            sum +%= listSum(&context.polys, count);
        }
        return sum;
    }

    fn baselineSlicedFindPath(context: *Context) u64 {
        return baselineSlicedFindPathWith(context, 0);
    }

    fn portSlicedFindPath(context: *Context) u64 {
        return portSlicedFindPathWith(context, .{});
    }

    fn baselineSlicedFindPathAnyAngle(context: *Context) u64 {
        return baselineSlicedFindPathWith(context, context.reference_any_angle);
    }

    fn portSlicedFindPathAnyAngle(context: *Context) u64 {
        return portSlicedFindPathWith(context, .{ .any_angle = true });
    }

    /// What a straight path of `count` corners adds to a checksum: the count,
    /// the middle and last corners, and the middle corner's flags and polygon.
    inline fn straightSum(context: *const Context, count: usize) u64 {
        if (count == 0) return 0;
        const middle = count / 2;
        return @as(u64, count) +%
            pointSum(context.straight_points[middle]) +%
            pointSum(context.straight_points[count - 1]) +%
            @as(u8, @bitCast(context.straight_flags[middle])) +%
            context.straight_refs[middle];
    }

    inline fn baselineFindStraightPathWith(context: *Context, options: i32) u64 {
        var sum: u64 = 0;
        var count: i32 = 0;
        for (0..query_count) |i| {
            const path = context.paths[context.path_starts[i]..context.path_starts[i + 1]];
            sum +%= reference.ref_query_find_straight_path(
                context.reference_query,
                &context.points[i],
                &context.path_ends[i],
                path.ptr,
                @intCast(path.len),
                @ptrCast(&context.straight_points),
                @as([*]u8, @ptrCast(&context.straight_flags)),
                &context.straight_refs,
                &count,
                straight_count_max,
                options,
            );
            sum +%= context.straightSum(@intCast(count));
        }
        return sum;
    }

    inline fn portFindStraightPathWith(
        context: *Context,
        options: detour.StraightPathOptions,
    ) u64 {
        var sum: u64 = 0;
        var count: usize = 0;
        for (0..query_count) |i| {
            const path = context.paths[context.path_starts[i]..context.path_starts[i + 1]];
            sum +%= statusSum(context.query.findStraightPath(
                context.points[i],
                context.path_ends[i],
                path,
                &context.straight_points,
                &context.straight_flags,
                &context.straight_refs,
                &count,
                options,
            ));
            sum +%= context.straightSum(count);
        }
        return sum;
    }

    fn baselineFindStraightPath(context: *Context) u64 {
        return baselineFindStraightPathWith(context, 0);
    }

    fn portFindStraightPath(context: *Context) u64 {
        return portFindStraightPathWith(context, .{});
    }

    fn baselineFindStraightPathAllCrossings(context: *Context) u64 {
        return baselineFindStraightPathWith(context, context.reference_all_crossings);
    }

    fn portFindStraightPathAllCrossings(context: *Context) u64 {
        return portFindStraightPathWith(context, .{ .all_crossings = true });
    }

    fn baselineMoveAlongSurface(context: *Context) u64 {
        var sum: u64 = 0;
        var result: [3]f32 = @splat(0);
        var count: i32 = 0;
        for (0..query_count) |i| {
            sum +%= reference.ref_query_move_along_surface(
                context.reference_query,
                context.refs[i],
                &context.points[i],
                &context.steps[i],
                context.reference_filter,
                &result,
                &context.visited,
                &count,
                visited_count_max,
            );
            sum +%= pointSum(result) +% listSum(&context.visited, @intCast(count));
        }
        return sum;
    }

    fn portMoveAlongSurface(context: *Context) u64 {
        var sum: u64 = 0;
        var result: [3]f32 = @splat(0);
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
            sum +%= pointSum(result) +% listSum(&context.visited, count);
        }
        return sum;
    }

    /// A ray from each point to the next, the polygons it crosses kept in
    /// `polys`. The edge it hit stays out of the checksum: the original's
    /// interface hands back zero where no edge was hit, the port leaves the
    /// field as it was.
    inline fn baselineRaycastWith(context: *Context, options: u32) u64 {
        var sum: u64 = 0;
        var t: f32 = 0;
        var normal: [3]f32 = @splat(0);
        var count: i32 = 0;
        var cost: f32 = 0;
        for (0..query_count) |i| {
            sum +%= reference.ref_query_raycast_hit(
                context.reference_query,
                context.refs[i],
                &context.points[i],
                &context.points[i + 1],
                context.reference_filter,
                options,
                0,
                &t,
                &normal,
                null,
                &context.polys,
                &count,
                poly_count_max,
                &cost,
            );
            sum +%= floatSum(t) +% pointSum(normal) +% floatSum(cost);
            sum +%= listSum(&context.polys, @intCast(count));
        }
        return sum;
    }

    inline fn portRaycastWith(context: *Context, options: detour.RaycastOptions) u64 {
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
            sum +%= statusSum(context.query.raycast(
                context.refs[i],
                context.points[i],
                context.points[i + 1],
                &context.filter,
                options,
                &hit,
                0,
            ));
            sum +%= floatSum(hit.t) +% pointSum(hit.hit_normal) +% floatSum(hit.path_cost);
            sum +%= listSum(&context.polys, hit.path_count);
        }
        return sum;
    }

    fn baselineRaycast(context: *Context) u64 {
        return baselineRaycastWith(context, 0);
    }

    fn portRaycast(context: *Context) u64 {
        return portRaycastWith(context, .{});
    }

    fn baselineRaycastWithCosts(context: *Context) u64 {
        return baselineRaycastWith(context, context.reference_use_costs);
    }

    fn portRaycastWithCosts(context: *Context) u64 {
        return portRaycastWith(context, .{ .use_costs = true });
    }

    fn baselineRaycastPath(context: *Context) u64 {
        var sum: u64 = 0;
        var t: f32 = 0;
        var normal: [3]f32 = @splat(0);
        var count: i32 = 0;
        for (0..query_count) |i| {
            sum +%= reference.ref_query_raycast(
                context.reference_query,
                context.refs[i],
                &context.points[i],
                &context.points[i + 1],
                context.reference_filter,
                &t,
                &normal,
                &context.polys,
                &count,
                poly_count_max,
            );
            sum +%= floatSum(t) +% pointSum(normal);
            sum +%= listSum(&context.polys, @intCast(count));
        }
        return sum;
    }

    fn portRaycastPath(context: *Context) u64 {
        var sum: u64 = 0;
        var t: f32 = 0;
        var normal: [3]f32 = @splat(0);
        var count: usize = 0;
        for (0..query_count) |i| {
            sum +%= statusSum(context.query.raycastPath(
                context.refs[i],
                context.points[i],
                context.points[i + 1],
                &context.filter,
                &t,
                &normal,
                &context.polys,
                &count,
            ));
            sum +%= floatSum(t) +% pointSum(normal);
            sum +%= listSum(&context.polys, count);
        }
        return sum;
    }

    fn baselineFindPolysAroundCircle(context: *Context) u64 {
        var sum: u64 = 0;
        var count: i32 = 0;
        for (0..query_count) |i| {
            sum +%= reference.ref_query_find_polys_around_circle(
                context.reference_query,
                context.refs[i],
                &context.points[i],
                circle_radius,
                context.reference_filter,
                &context.polys,
                &context.parents,
                &context.costs,
                &count,
                poly_count_max,
            );
            sum +%= listSum(&context.polys, @intCast(count));
            if (count > 0) sum +%= floatSum(context.costs[@intCast(count - 1)]);
        }
        return sum;
    }

    fn portFindPolysAroundCircle(context: *Context) u64 {
        var sum: u64 = 0;
        var count: usize = 0;
        for (0..query_count) |i| {
            sum +%= statusSum(context.query.findPolysAroundCircle(
                context.refs[i],
                context.points[i],
                circle_radius,
                &context.filter,
                &context.polys,
                &context.parents,
                &context.costs,
                &count,
                poly_count_max,
            ));
            sum +%= listSum(&context.polys, count);
            if (count > 0) sum +%= floatSum(context.costs[count - 1]);
        }
        return sum;
    }

    fn baselineFindPolysAroundShape(context: *Context) u64 {
        var sum: u64 = 0;
        var count: i32 = 0;
        for (0..query_count) |i| {
            sum +%= reference.ref_query_find_polys_around_shape(
                context.reference_query,
                context.refs[i],
                @ptrCast(&context.shapes[i]),
                4,
                context.reference_filter,
                &context.polys,
                &context.parents,
                &context.costs,
                &count,
                poly_count_max,
            );
            sum +%= listSum(&context.polys, @intCast(count));
            if (count > 0) sum +%= floatSum(context.costs[@intCast(count - 1)]);
        }
        return sum;
    }

    fn portFindPolysAroundShape(context: *Context) u64 {
        var sum: u64 = 0;
        var count: usize = 0;
        for (0..query_count) |i| {
            sum +%= statusSum(context.query.findPolysAroundShape(
                context.refs[i],
                &context.shapes[i],
                &context.filter,
                &context.polys,
                &context.parents,
                &context.costs,
                &count,
                poly_count_max,
            ));
            sum +%= listSum(&context.polys, count);
            if (count > 0) sum +%= floatSum(context.costs[count - 1]);
        }
        return sum;
    }

    fn baselineFindLocalNeighbourhood(context: *Context) u64 {
        var sum: u64 = 0;
        var count: i32 = 0;
        for (0..query_count) |i| {
            sum +%= reference.ref_query_find_local_neighbourhood(
                context.reference_query,
                context.refs[i],
                &context.points[i],
                neighbourhood_radius,
                context.reference_filter,
                &context.polys,
                &context.parents,
                &count,
                visited_count_max,
            );
            sum +%= listSum(&context.polys, @intCast(count));
        }
        return sum;
    }

    fn portFindLocalNeighbourhood(context: *Context) u64 {
        var sum: u64 = 0;
        var count: usize = 0;
        for (0..query_count) |i| {
            sum +%= statusSum(context.query.findLocalNeighbourhood(
                context.refs[i],
                context.points[i],
                neighbourhood_radius,
                &context.filter,
                context.polys[0..visited_count_max],
                context.parents[0..visited_count_max],
                &count,
            ));
            sum +%= listSum(&context.polys, count);
        }
        return sum;
    }

    fn baselineGetPolyWallSegments(context: *Context) u64 {
        var sum: u64 = 0;
        var count: i32 = 0;
        for (0..query_count) |i| {
            sum +%= reference.ref_query_get_poly_wall_segments(
                context.reference_query,
                context.refs[i],
                context.reference_filter,
                @ptrCast(&context.segments),
                &context.segment_refs,
                &count,
                segment_count_max,
            );
            sum +%= @as(u64, @intCast(count));
            if (count > 0) sum +%= pointSum(context.segments[@intCast(count - 1)][1]);
        }
        return sum;
    }

    fn portGetPolyWallSegments(context: *Context) u64 {
        var sum: u64 = 0;
        var count: usize = 0;
        for (0..query_count) |i| {
            sum +%= statusSum(context.query.getPolyWallSegments(
                context.refs[i],
                &context.filter,
                &context.segments,
                &context.segment_refs,
                &count,
            ));
            sum +%= count;
            if (count > 0) sum +%= pointSum(context.segments[count - 1][1]);
        }
        return sum;
    }

    fn baselineFindDistanceToWall(context: *Context) u64 {
        var sum: u64 = 0;
        var distance: f32 = 0;
        var point: [3]f32 = @splat(0);
        var normal: [3]f32 = @splat(0);
        for (0..query_count) |i| {
            sum +%= reference.ref_query_find_distance_to_wall(
                context.reference_query,
                context.refs[i],
                &context.points[i],
                wall_radius,
                context.reference_filter,
                &distance,
                &point,
                &normal,
            );
            sum +%= floatSum(distance) +% pointSum(point) +% pointSum(normal);
        }
        return sum;
    }

    fn portFindDistanceToWall(context: *Context) u64 {
        var sum: u64 = 0;
        var distance: f32 = 0;
        var point: [3]f32 = @splat(0);
        var normal: [3]f32 = @splat(0);
        for (0..query_count) |i| {
            sum +%= statusSum(context.query.findDistanceToWall(
                context.refs[i],
                context.points[i],
                wall_radius,
                &context.filter,
                &distance,
                &point,
                &normal,
            ));
            sum +%= floatSum(distance) +% pointSum(point) +% pointSum(normal);
        }
        return sum;
    }

    fn baselineFindRandomPoint(context: *Context) u64 {
        support.seedReferenceRandom(random_seed);
        var sum: u64 = 0;
        var ref: PolyRef = 0;
        var point: [3]f32 = @splat(0);
        for (0..query_count) |_| {
            sum +%= reference.ref_query_find_random_point(
                context.reference_query,
                context.reference_filter,
                support.referenceRandom,
                &ref,
                &point,
            );
            sum +%= ref +% pointSum(point);
        }
        return sum;
    }

    fn portFindRandomPoint(context: *Context) u64 {
        var prng: std.Random.DefaultPrng = .init(random_seed);
        const random = prng.random();
        var sum: u64 = 0;
        var ref: PolyRef = 0;
        var point: [3]f32 = @splat(0);
        for (0..query_count) |_| {
            sum +%= statusSum(context.query.findRandomPoint(&context.filter, random, &ref, &point));
            sum +%= ref +% pointSum(point);
        }
        return sum;
    }

    fn baselineFindRandomPointAroundCircle(context: *Context) u64 {
        support.seedReferenceRandom(random_seed);
        var sum: u64 = 0;
        var ref: PolyRef = 0;
        var point: [3]f32 = @splat(0);
        for (0..query_count) |i| {
            sum +%= reference.ref_query_find_random_point_around_circle(
                context.reference_query,
                context.refs[i],
                &context.points[i],
                circle_radius,
                context.reference_filter,
                support.referenceRandom,
                &ref,
                &point,
            );
            sum +%= ref +% pointSum(point);
        }
        return sum;
    }

    fn portFindRandomPointAroundCircle(context: *Context) u64 {
        var prng: std.Random.DefaultPrng = .init(random_seed);
        const random = prng.random();
        var sum: u64 = 0;
        var ref: PolyRef = 0;
        var point: [3]f32 = @splat(0);
        for (0..query_count) |i| {
            sum +%= statusSum(context.query.findRandomPointAroundCircle(
                context.refs[i],
                context.points[i],
                circle_radius,
                &context.filter,
                random,
                &ref,
                &point,
            ));
            sum +%= ref +% pointSum(point);
        }
        return sum;
    }
};

/// A row of `query_count` queries.
fn queryRow(name: []const u8) Row {
    return .{ .name = name, .per = "query", .count = query_count };
}

pub fn run(bench: *Bench) !void {
    try bench.group("Queries");

    // The tiled level: its mesh on both sides, built once for every row.
    var world: support.World = undefined;
    try world.init(std.heap.c_allocator, support.nav_test_obj, support.tiled_config, .{});
    defer world.deinit();

    var context: Context = undefined;
    try context.init(bench.gpa, &world);
    defer context.deinit(bench.gpa);

    // The box of `half_extents` around each of the 1,000 ground points.
    try bench.compare(
        queryRow("NavMeshQuery.findNearestPoly"),
        &context,
        Context.baselineFindNearestPoly,
        Context.portFindNearestPoly,
    );
    // The same boxes, into a buffer of 256 polygons.
    try bench.compare(
        queryRow("NavMeshQuery.queryPolygons"),
        &context,
        Context.baselineQueryPolygons,
        Context.portQueryPolygons,
    );
    // The same boxes, each batch of polygons handed to a receiver that adds
    // up their references.
    try bench.compare(
        queryRow("NavMeshQuery.queryPolygonsWith"),
        &context,
        Context.baselineQueryPolygonsWith,
        Context.portQueryPolygonsWith,
    );
    // Each ground point against the polygon found nearest it.
    try bench.compare(
        queryRow("NavMeshQuery.closestPointOnPoly"),
        &context,
        Context.baselineClosestPointOnPoly,
        Context.portClosestPointOnPoly,
    );
    // The same points and polygons.
    try bench.compare(
        queryRow("NavMeshQuery.closestPointOnPolyBoundary"),
        &context,
        Context.baselineClosestPointOnPolyBoundary,
        Context.portClosestPointOnPolyBoundary,
    );
    // Each point on the mesh with its polygon.
    try bench.compare(
        queryRow("NavMeshQuery.getPolyHeight"),
        &context,
        Context.baselineGetPolyHeight,
        Context.portGetPolyHeight,
    );
    // Each point's polygon, with the default filter.
    try bench.compare(
        queryRow("NavMeshQuery.isValidPolyRef"),
        &context,
        Context.baselineIsValidPolyRef,
        Context.portIsValidPolyRef,
    );
    // From each point to the next, on the polygons found nearest them.
    try bench.compare(
        queryRow("NavMeshQuery.findPath"),
        &context,
        Context.baselineFindPath,
        Context.portFindPath,
    );
    // The same pairs, the search sliced into calls of 64 iterations.
    try bench.compare(
        queryRow("NavMeshQuery sliced findPath"),
        &context,
        Context.baselineSlicedFindPath,
        Context.portSlicedFindPath,
    );
    try bench.compare(
        queryRow("NavMeshQuery sliced findPath, any angle"),
        &context,
        Context.baselineSlicedFindPathAnyAngle,
        Context.portSlicedFindPathAnyAngle,
    );
    // The corners of each path found in the setup.
    try bench.compare(
        queryRow("NavMeshQuery.findStraightPath"),
        &context,
        Context.baselineFindStraightPath,
        Context.portFindStraightPath,
    );
    try bench.compare(
        queryRow("NavMeshQuery.findStraightPath, all crossings"),
        &context,
        Context.baselineFindStraightPathAllCrossings,
        Context.portFindStraightPathAllCrossings,
    );
    // Half a metre from each point toward the next.
    try bench.compare(
        queryRow("NavMeshQuery.moveAlongSurface"),
        &context,
        Context.baselineMoveAlongSurface,
        Context.portMoveAlongSurface,
    );
    // From each point toward the next, until the ray hits a wall or ends.
    try bench.compare(
        queryRow("NavMeshQuery.raycast"),
        &context,
        Context.baselineRaycast,
        Context.portRaycast,
    );
    try bench.compare(
        queryRow("NavMeshQuery.raycast, with costs"),
        &context,
        Context.baselineRaycastWithCosts,
        Context.portRaycastWithCosts,
    );
    try bench.compare(
        queryRow("NavMeshQuery.raycastPath"),
        &context,
        Context.baselineRaycastPath,
        Context.portRaycastPath,
    );
    // A circle of 10 m around each point.
    try bench.compare(
        queryRow("NavMeshQuery.findPolysAroundCircle"),
        &context,
        Context.baselineFindPolysAroundCircle,
        Context.portFindPolysAroundCircle,
    );
    // A square of 10 m around each point.
    try bench.compare(
        queryRow("NavMeshQuery.findPolysAroundShape"),
        &context,
        Context.baselineFindPolysAroundShape,
        Context.portFindPolysAroundShape,
    );
    // 5 m around each point, up to 16 polygons: what the crowd asks for.
    try bench.compare(
        queryRow("NavMeshQuery.findLocalNeighbourhood"),
        &context,
        Context.baselineFindLocalNeighbourhood,
        Context.portFindLocalNeighbourhood,
    );
    // Each point's polygon, its portals included.
    try bench.compare(
        queryRow("NavMeshQuery.getPolyWallSegments"),
        &context,
        Context.baselineGetPolyWallSegments,
        Context.portGetPolyWallSegments,
    );
    // Within 20 m of each point.
    try bench.compare(
        queryRow("NavMeshQuery.findDistanceToWall"),
        &context,
        Context.baselineFindDistanceToWall,
        Context.portFindDistanceToWall,
    );
    // Both sides draw the same floats from the same seed.
    try bench.compare(
        queryRow("NavMeshQuery.findRandomPoint"),
        &context,
        Context.baselineFindRandomPoint,
        Context.portFindRandomPoint,
    );
    try bench.compare(
        queryRow("NavMeshQuery.findRandomPointAroundCircle"),
        &context,
        Context.baselineFindRandomPointAroundCircle,
        Context.portFindRandomPointAroundCircle,
    );
}
