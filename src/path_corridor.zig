//! A path corridor: the run of polygons an agent follows from its position
//! to its target, kept valid as the agent and the target move, and the
//! three merges that splice a freshly walked or searched run of polygons
//! into it.

const std = @import("std");
const common = @import("common.zig");
const nav_mesh = @import("nav_mesh.zig");
const nav_mesh_query = @import("nav_mesh_query.zig");

const PolyRef = nav_mesh.PolyRef;
const StraightPathFlags = nav_mesh.StraightPathFlags;
const NavMeshQuery = nav_mesh_query.NavMeshQuery;
const QueryFilter = nav_mesh_query.QueryFilter;

/// Where a corridor and a run of visited polygons last meet.
const CommonPoly = struct {
    path_index: i32,
    visited_index: i32,
};

/// Rewrites the start of a corridor after its position moved through
/// `visited`, the polygons walked from the old first polygon to the new
/// one. The corridor from its last polygon shared with `visited` onward is
/// kept, and `visited` from there to its end goes in front of it, reversed.
///
/// `path[0..path_count]` is the corridor and `path.len` its capacity; the
/// tail that does not fit is dropped. Returns the new count, `path_count`
/// unchanged when the two share no polygon.
pub fn mergeCorridorStartMoved(
    path: []PolyRef,
    path_count: usize,
    visited: []const PolyRef,
) usize {
    const npath: i32 = @intCast(path_count);
    const path_max: i32 = @intCast(path.len);
    const nvisited: i32 = @intCast(visited.len);

    // Find the furthest common polygon.
    var furthest: ?CommonPoly = null;
    var i = npath - 1;
    while (i >= 0) : (i -= 1) {
        var j = nvisited - 1;
        while (j >= 0) : (j -= 1) {
            if (path[@intCast(i)] == visited[@intCast(j)]) {
                furthest = .{ .path_index = i, .visited_index = j };
            }
        }
        if (furthest != null) break;
    }

    // Without a common polygon the path stays as it is.
    const common_poly = furthest orelse return path_count;

    // Concatenate the paths.

    // Move the kept part up to make room for the visited polygons.
    const req = nvisited - common_poly.visited_index;
    const orig = @min(common_poly.path_index + 1, npath);
    var size: i32 = @max(0, npath - orig);
    if (req + size > path_max) size = path_max - req;
    if (size > 0) {
        const len: usize = @intCast(size);
        @memmove(path[@intCast(req)..][0..len], path[@intCast(orig)..][0..len]);
    }

    // Store the visited polygons, last first.
    const stored: usize = @intCast(@min(req, path_max));
    for (path[0..stored], 0..) |*ref, k| ref.* = visited[visited.len - 1 - k];

    return @intCast(req + size);
}

/// Rewrites the end of a corridor after its target moved through
/// `visited`, the polygons walked from the old last polygon to the new
/// one. The corridor up to its first polygon shared with `visited` is kept,
/// and `visited` after that polygon is appended, as much as fits.
///
/// `path[0..path_count]` is the corridor and `path.len` its capacity.
/// `visited` must not be part of `path`. Returns the new count,
/// `path_count` unchanged when the two share no polygon.
pub fn mergeCorridorEndMoved(
    path: []PolyRef,
    path_count: usize,
    visited: []const PolyRef,
) usize {
    const npath: i32 = @intCast(path_count);
    const path_max: i32 = @intCast(path.len);
    const nvisited: i32 = @intCast(visited.len);

    // Find the furthest common polygon.
    var furthest: ?CommonPoly = null;
    var i: i32 = 0;
    while (i < npath) : (i += 1) {
        var j = nvisited - 1;
        while (j >= 0) : (j -= 1) {
            if (path[@intCast(i)] == visited[@intCast(j)]) {
                furthest = .{ .path_index = i, .visited_index = j };
            }
        }
        if (furthest != null) break;
    }

    // Without a common polygon the path stays as it is.
    const common_poly = furthest orelse return path_count;

    // Concatenate the paths.
    const ppos = common_poly.path_index + 1;
    const vpos = common_poly.visited_index + 1;
    const count = @min(nvisited - vpos, path_max - ppos);
    std.debug.assert(ppos + count <= path_max);
    if (count != 0) {
        const len: usize = @intCast(count);
        @memcpy(path[@intCast(ppos)..][0..len], visited[@intCast(vpos)..][0..len]);
    }

    return @intCast(ppos + count);
}

/// Replaces the start of a corridor with a shortcut: `visited` is a run of
/// polygons from the corridor's first polygon that meets the corridor again
/// further on. The corridor from its last polygon shared with `visited`
/// onward is kept, and `visited` before that polygon goes in front of it.
///
/// `path[0..path_count]` is the corridor and `path.len` its capacity; the
/// tail that does not fit is dropped, and so is the part of `visited`
/// beyond the capacity. Returns the new count, `path_count` unchanged when
/// the two share no polygon or meet only at the first polygon of `visited`.
pub fn mergeCorridorStartShortcut(
    path: []PolyRef,
    path_count: usize,
    visited: []const PolyRef,
) usize {
    const npath: i32 = @intCast(path_count);
    const path_max: i32 = @intCast(path.len);
    const nvisited: i32 = @intCast(visited.len);

    // Find the furthest common polygon.
    var furthest: ?CommonPoly = null;
    var i = npath - 1;
    while (i >= 0) : (i -= 1) {
        var j = nvisited - 1;
        while (j >= 0) : (j -= 1) {
            if (path[@intCast(i)] == visited[@intCast(j)]) {
                furthest = .{ .path_index = i, .visited_index = j };
            }
        }
        if (furthest != null) break;
    }

    // Without a common polygon the path stays as it is.
    const common_poly = furthest orelse return path_count;

    // Concatenate the paths.

    // Move the kept part to where the visited polygons end.
    const req = common_poly.visited_index;
    if (req <= 0) return path_count;

    const orig = common_poly.path_index;
    var size: i32 = @max(0, npath - orig);
    if (req + size > path_max) size = path_max - req;
    // The size is below zero when the visited polygons alone overflow the
    // capacity; nothing of the old path is kept then.
    if (size > 0) {
        const len: usize = @intCast(size);
        @memmove(path[@intCast(req)..][0..len], path[@intCast(orig)..][0..len]);
    }

    // Store the visited polygons.
    const stored: usize = @intCast(@min(req, path_max));
    for (path[0..stored], visited[0..stored]) |*ref, visited_ref| ref.* = visited_ref;

    return @intCast(req + size);
}

/// A run of connected polygons from the one holding an agent's position to
/// the one holding its target, with both points kept on the navigation
/// mesh.
///
/// A corridor starts as `empty`; `init` gives it its path buffer and
/// `deinit` frees it with the same allocator. Load it with `reset` and
/// `setCorridor`, plan with `findCorners`, and feed movement back with
/// `movePosition` and `moveTargetPosition`. The moves search only near the
/// corridor, so a point moved far in one step may not be reached: compare
/// `getFirstPoly` or `getLastPoly` with the polygon expected. Moving makes
/// the path less direct over time; `optimizePathVisibility` and
/// `optimizePathTopology` straighten it again.
pub const PathCorridor = struct {
    pos: [3]f32,
    target: [3]f32,
    /// The path buffer, whose length is the most polygons the corridor
    /// holds; null until `init`.
    path: ?[]PolyRef,
    /// The polygons of `path` in use.
    path_count: i32,

    /// A corridor without a path buffer. Only `init`, `deinit`, `isValid`
    /// and the getters may be called on it.
    pub const empty: PathCorridor = .{
        .pos = .{ 0, 0, 0 },
        .target = .{ 0, 0, 0 },
        .path = null,
        .path_count = 0,
    };

    /// Allocates the path buffer of an `empty` corridor, for paths of up
    /// to `path_count_max` polygons. `fixPathStart` needs room for three.
    /// Returns false when memory runs out, leaving the corridor `empty`.
    pub fn init(self: *PathCorridor, gpa: std.mem.Allocator, path_count_max: i32) bool {
        std.debug.assert(self.path == null);
        // A negative count asks for more memory than there is.
        const capacity = std.math.cast(usize, path_count_max) orelse return false;
        self.path = gpa.alloc(PolyRef, capacity) catch return false;
        self.path_count = 0;
        return true;
    }

    /// Frees the path buffer. `gpa` is the allocator `init` was given.
    pub fn deinit(self: *PathCorridor, gpa: std.mem.Allocator) void {
        if (self.path) |path| gpa.free(path);
        self.* = undefined;
    }

    /// Makes the corridor the single polygon `ref`, with position and
    /// target both at `pos`.
    pub fn reset(self: *PathCorridor, ref: PolyRef, pos: [3]f32) void {
        const path = self.path.?;
        self.pos = pos;
        self.target = pos;
        path[0] = ref;
        self.path_count = 1;
    }

    /// Finds the corners of the straight path from the position toward the
    /// target, for planning the next steps, and returns how many were
    /// written. `corner_verts.len` is the capacity, and the flag and
    /// polygon slices are at least as long.
    ///
    /// Corners within a hundredth of a unit of the position are left out,
    /// and nothing follows the start of an off-mesh connection. The target,
    /// when in range, is the last corner, with a zero polygon reference.
    /// The filter is not used.
    pub fn findCorners(
        self: *PathCorridor,
        corner_verts: [][3]f32,
        corner_flags: []StraightPathFlags,
        corner_polys: []PolyRef,
        query: *NavMeshQuery,
        _: *const QueryFilter,
    ) i32 {
        const path = self.path.?;
        std.debug.assert(self.path_count != 0);

        const target_dist_min: f32 = 0.01;

        var corner_count: usize = 0;
        _ = query.findStraightPath(
            self.pos,
            self.target,
            path[0..@intCast(self.path_count)],
            corner_verts,
            corner_flags,
            corner_polys,
            &corner_count,
            .{},
        );

        // Prune points at the beginning of the path which are too close.
        while (corner_count != 0) {
            if (corner_flags[0].off_mesh_connection or
                common.vdist2dSqr(corner_verts[0], self.pos) > common.sqr(f32, target_dist_min))
            {
                break;
            }
            corner_count -= 1;
            if (corner_count != 0) {
                const n = corner_count;
                @memmove(corner_flags[0..n], corner_flags[1..][0..n]);
                @memmove(corner_polys[0..n], corner_polys[1..][0..n]);
                @memmove(corner_verts[0..n], corner_verts[1..][0..n]);
            }
        }

        // Prune points after an off-mesh connection.
        for (corner_flags[0..corner_count], 0..) |flags, i| {
            if (flags.off_mesh_connection) {
                corner_count = i + 1;
                break;
            }
        }

        return @intCast(corner_count);
    }

    /// Shortens the start of the corridor when `next`, a point further
    /// along the path, is in straight sight of the position, by casting a
    /// ray toward it over at most `path_optimization_range` (above zero).
    ///
    /// It mends a corridor an agent has drifted off through inexact
    /// movement or avoidance, and the kinks that form at tile corners. The
    /// corridor changes only when the ray gets all the way through. Not for
    /// long distances.
    pub fn optimizePathVisibility(
        self: *PathCorridor,
        next: [3]f32,
        path_optimization_range: f32,
        query: *NavMeshQuery,
        filter: *const QueryFilter,
    ) void {
        const path = self.path.?;

        // Clamp the ray to the range.
        var goal = next;
        var dist = common.vdist2d(self.pos, goal);

        // Too close to the goal to be worth optimising.
        if (dist < 0.01) return;

        // Overshoot a little. This helps to optimise open fields in tiled
        // meshes.
        dist = common.min(f32, dist + 0.01, path_optimization_range);

        // Adjust the ray's length.
        const delta = common.vsub(goal, self.pos);
        goal = common.vmad(self.pos, delta, path_optimization_range / dist);

        const result_count_max = 32;
        var result: [result_count_max]PolyRef = undefined;
        var t: f32 = undefined;
        var normal: [3]f32 = undefined;
        var result_count: usize = 0;
        _ = query.raycastPath(
            path[0],
            self.pos,
            goal,
            filter,
            &t,
            &normal,
            &result,
            &result_count,
        );
        if (result_count > 1 and t > 0.99) {
            self.path_count = @intCast(mergeCorridorStartShortcut(
                path,
                @intCast(self.path_count),
                result[0..result_count],
            ));
        }
    }

    /// Shortens the start of the corridor with a short path search from
    /// the position toward the target, splicing in what the search found.
    /// Returns whether the corridor was rewritten; a corridor under three
    /// polygons is left alone.
    ///
    /// It mends a corridor an agent has drifted off over time. The search
    /// runs on `query`, replacing any sliced search in progress there.
    pub fn optimizePathTopology(
        self: *PathCorridor,
        query: *NavMeshQuery,
        filter: *const QueryFilter,
    ) bool {
        const path = self.path.?;

        if (self.path_count < 3) return false;

        const iteration_count_max = 32;
        const result_count_max = 32;

        var result: [result_count_max]PolyRef = undefined;
        var result_count: usize = 0;
        const path_count: usize = @intCast(self.path_count);
        _ = query.initSlicedFindPath(
            path[0],
            path[path_count - 1],
            self.pos,
            self.target,
            filter,
            .{},
        );
        _ = query.updateSlicedFindPath(iteration_count_max, null);
        const status = query.finalizeSlicedFindPathPartial(
            path[0..path_count],
            &result,
            &result_count,
        );

        if (status.succeeded and result_count > 0) {
            self.path_count = @intCast(mergeCorridorStartShortcut(
                path,
                path_count,
                result[0..result_count],
            ));
            return true;
        }

        return false;
    }

    /// Advances the corridor past the off-mesh connection
    /// `off_mesh_con_ref`: the polygons up to and including it are removed
    /// and the position jumps to the connection's far end.
    ///
    /// `refs` gets the polygon before the connection and the connection,
    /// `start_pos` and `end_pos` the connection's ends in walking order.
    /// Returns false, with the corridor unchanged, when the connection is
    /// not in the corridor or is its last polygon; and false with the
    /// corridor already advanced when the connection's ends cannot be
    /// found.
    pub fn moveOverOffmeshConnection(
        self: *PathCorridor,
        off_mesh_con_ref: PolyRef,
        refs: *[2]PolyRef,
        start_pos: *[3]f32,
        end_pos: *[3]f32,
        query: *NavMeshQuery,
    ) bool {
        const path = self.path.?;
        std.debug.assert(self.path_count != 0);

        // Advance the path up to and over the off-mesh connection.
        var prev_ref: PolyRef = 0;
        var poly_ref = path[0];
        var npos: i32 = 0;
        while (npos < self.path_count and poly_ref != off_mesh_con_ref) {
            prev_ref = poly_ref;
            poly_ref = path[@intCast(npos)];
            npos += 1;
        }
        // The connection was not found.
        if (npos == self.path_count) return false;

        // Prune the path.
        const pruned: usize = @intCast(npos);
        for (pruned..@intCast(self.path_count)) |i| path[i - pruned] = path[i];
        self.path_count -= npos;

        refs[0] = prev_ref;
        refs[1] = poly_ref;

        const nav = query.getAttachedNavMesh();

        const status = nav.getOffMeshConnectionPolyEndPoints(
            refs[0],
            refs[1],
            start_pos,
            end_pos,
        );
        if (status.succeeded) {
            self.pos = end_pos.*;
            return true;
        }

        return false;
    }

    /// Moves the position toward `new_pos` along the surface of the mesh
    /// and rewrites the start of the corridor to match, so the position
    /// stays in the corridor's first polygon. Returns whether the move ran;
    /// on false nothing changed.
    ///
    /// The position ends short of `new_pos` when that point is off the
    /// mesh or beyond the reach of the local search.
    pub fn movePosition(
        self: *PathCorridor,
        new_pos: [3]f32,
        query: *NavMeshQuery,
        filter: *const QueryFilter,
    ) bool {
        const path = self.path.?;
        std.debug.assert(self.path_count != 0);

        // Move along the mesh and update the position.
        var result: [3]f32 = undefined;
        const visited_count_max = 16;
        var visited: [visited_count_max]PolyRef = undefined;
        var visited_count: usize = 0;
        const status = query.moveAlongSurface(
            path[0],
            self.pos,
            new_pos,
            filter,
            &result,
            &visited,
            &visited_count,
        );
        if (status.succeeded) {
            self.path_count = @intCast(mergeCorridorStartMoved(
                path,
                @intCast(self.path_count),
                visited[0..visited_count],
            ));

            // Keep the position on top of the mesh. Where the polygon has
            // no height there, the old height stays.
            var h = self.pos[1];
            _ = query.getPolyHeight(path[0], result, &h);
            result[1] = h;
            self.pos = result;
            return true;
        }
        return false;
    }

    /// Moves the target toward `new_pos` along the surface of the mesh and
    /// rewrites the end of the corridor to match, so the target stays in
    /// the corridor's last polygon. Returns whether the move ran; on false
    /// nothing changed.
    ///
    /// The target ends short of `new_pos` when that point is off the mesh
    /// or beyond the reach of the local search.
    pub fn moveTargetPosition(
        self: *PathCorridor,
        new_pos: [3]f32,
        query: *NavMeshQuery,
        filter: *const QueryFilter,
    ) bool {
        const path = self.path.?;
        std.debug.assert(self.path_count != 0);

        // Move along the mesh and update the target.
        var result: [3]f32 = undefined;
        const visited_count_max = 16;
        var visited: [visited_count_max]PolyRef = undefined;
        var visited_count: usize = 0;
        const status = query.moveAlongSurface(
            path[@intCast(self.path_count - 1)],
            self.target,
            new_pos,
            filter,
            &result,
            &visited,
            &visited_count,
        );
        if (status.succeeded) {
            self.path_count = @intCast(mergeCorridorEndMoved(
                path,
                @intCast(self.path_count),
                visited[0..visited_count],
            ));

            // The target's height is left as the move gave it.
            self.target = result;

            return true;
        }
        return false;
    }

    /// Loads a path and its target: `polys` runs from the polygon holding
    /// the corridor's position to the one holding `target`. It is not
    /// empty, no longer than the corridor's capacity, and not part of the
    /// corridor's own buffer.
    pub fn setCorridor(self: *PathCorridor, target: [3]f32, polys: []const PolyRef) void {
        const path = self.path.?;
        std.debug.assert(polys.len > 0);
        std.debug.assert(polys.len <= path.len);

        self.target = target;
        @memcpy(path[0..polys.len], polys);
        self.path_count = @intCast(polys.len);
    }

    /// Restarts a corridor whose first polygon went bad from `safe_ref` at
    /// `safe_pos`. The second polygon becomes zero, a gap the next replan
    /// closes, and a corridor of one or two polygons becomes three: the
    /// safe polygon, the gap and the old last polygon. The capacity must be
    /// at least three. Always returns true.
    pub fn fixPathStart(self: *PathCorridor, safe_ref: PolyRef, safe_pos: [3]f32) bool {
        const path = self.path.?;

        self.pos = safe_pos;
        if (self.path_count < 3 and self.path_count > 0) {
            path[2] = path[@intCast(self.path_count - 1)];
            path[0] = safe_ref;
            path[1] = 0;
            self.path_count = 3;
        } else {
            path[0] = safe_ref;
            path[1] = 0;
        }

        return true;
    }

    /// Cuts the corridor at its first polygon that no longer exists or
    /// passes `filter`, and pulls the target onto the boundary of the new
    /// last polygon. When the first polygon itself is bad the corridor
    /// becomes `safe_ref` with the position at `safe_pos`. A corridor that
    /// is all valid is left alone. Always returns true.
    pub fn trimInvalidPath(
        self: *PathCorridor,
        safe_ref: PolyRef,
        safe_pos: [3]f32,
        query: *NavMeshQuery,
        filter: *const QueryFilter,
    ) bool {
        const path = self.path.?;

        // Keep the valid path as far as possible.
        var n: i32 = 0;
        while (n < self.path_count and query.isValidPolyRef(path[@intCast(n)], filter)) {
            n += 1;
        }

        if (n == self.path_count) {
            // All valid, nothing to fix.
            return true;
        } else if (n == 0) {
            // The first polygon is bad: use the safe values.
            self.pos = safe_pos;
            path[0] = safe_ref;
            self.path_count = 1;
        } else {
            // The path is usable in part.
            self.path_count = n;
        }

        // Clamp the target to the last polygon. The copy keeps the query's
        // input apart from the target it writes.
        const target = self.target;
        _ = query.closestPointOnPolyBoundary(
            path[@intCast(self.path_count - 1)],
            target,
            &self.target,
        );

        return true;
    }

    /// Whether the first `look_ahead_count_max` polygons of the corridor
    /// all still exist and pass `filter`. A polygon stops doing so when its
    /// tile is removed or rebuilt, or when its flags change.
    pub fn isValid(
        self: *const PathCorridor,
        look_ahead_count_max: i32,
        query: *NavMeshQuery,
        filter: *const QueryFilter,
    ) bool {
        // Check that all polygons still pass the query filter.
        const path = self.getPath();
        const n = @min(self.path_count, look_ahead_count_max);
        var i: i32 = 0;
        while (i < n) : (i += 1) {
            if (!query.isValidPolyRef(path[@intCast(i)], filter)) return false;
        }

        return true;
    }

    /// The position, inside the corridor's first polygon.
    pub fn getPos(self: *const PathCorridor) [3]f32 {
        return self.pos;
    }

    /// The target, inside the corridor's last polygon.
    pub fn getTarget(self: *const PathCorridor) [3]f32 {
        return self.target;
    }

    /// The polygon holding the position, or zero when there is no path.
    pub fn getFirstPoly(self: *const PathCorridor) PolyRef {
        const path = self.getPath();
        return if (path.len != 0) path[0] else 0;
    }

    /// The polygon holding the target, or zero when there is no path.
    pub fn getLastPoly(self: *const PathCorridor) PolyRef {
        const path = self.getPath();
        return if (path.len != 0) path[path.len - 1] else 0;
    }

    /// The corridor's polygons, `getPathCount()` of them, from the position
    /// to the target. The slice points into the corridor's buffer: every
    /// call that rewrites the corridor changes what it holds, and `deinit`
    /// frees it.
    pub fn getPath(self: *const PathCorridor) []const PolyRef {
        const path = self.path orelse return &.{};
        return path[0..@intCast(self.path_count)];
    }

    /// The number of polygons in the corridor.
    pub fn getPathCount(self: *const PathCorridor) i32 {
        return self.path_count;
    }
};
