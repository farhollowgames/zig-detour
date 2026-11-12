//! A path corridor on whole millimetres: the run of polygons an agent
//! follows from its position to its target, kept valid as the agent and the
//! target move, with the float port's steps in the float port's order.
//!
//! The corridor works on a polygon buffer its caller owns, so a caller that
//! keeps corridors elsewhere (in rows, say) builds one for a call and reads
//! the buffer back after. The three merges are the float port's functions:
//! they move polygon references only.

const std = @import("std");
const common = @import("common.zig");
const nav_mesh = @import("nav_mesh.zig");
const nav_mesh_query = @import("nav_mesh_query.zig");
const float_corridor = @import("../path_corridor.zig");

const PolyRef = nav_mesh.PolyRef;
const StraightPathFlags = nav_mesh.StraightPathFlags;
const NavMeshQuery = nav_mesh_query.NavMeshQuery;
const QueryFilter = nav_mesh_query.QueryFilter;
const Fraction = common.Fraction;

/// Rewrites the start of a corridor after its position moved through
/// `visited`; see the float port's function of the same name.
pub const mergeCorridorStartMoved = float_corridor.mergeCorridorStartMoved;
/// Rewrites the end of a corridor after its target moved through
/// `visited`; see the float port's function of the same name.
pub const mergeCorridorEndMoved = float_corridor.mergeCorridorEndMoved;
/// Replaces the start of a corridor with a shortcut through `visited`; see
/// the float port's function of the same name.
pub const mergeCorridorStartShortcut = float_corridor.mergeCorridorStartShortcut;

/// Corners nearer the position than this, in millimetres, are left out:
/// the float port's hundredth of a unit.
pub const corner_distance_min: u64 = 10;

/// A run of connected polygons from the one holding an agent's position to
/// the one holding its target, with both points kept on the navigation
/// mesh.
///
/// `path` is the caller's buffer, whose length is the most polygons the
/// corridor holds; `path_count` of them are in use. The corridor never
/// allocates. Load it with `reset` and `setCorridor`, or set its fields
/// from a kept corridor, plan with `findCorners`, and feed movement back
/// with `movePosition` and `moveTargetPosition`. The moves search only near
/// the corridor, so a point moved far in one step may not be reached:
/// compare `getFirstPoly` or `getLastPoly` with the polygon expected.
/// `getPath` views the caller's buffer, which every call that rewrites the
/// corridor changes.
pub const PathCorridor = struct {
    pos: [3]i32,
    target: [3]i32,
    path: []PolyRef,
    path_count: usize,

    /// A corridor over `buffer` with no polygon in it.
    pub fn init(buffer: []PolyRef) PathCorridor {
        return .{ .pos = .{ 0, 0, 0 }, .target = .{ 0, 0, 0 }, .path = buffer, .path_count = 0 };
    }

    /// Makes the corridor the single polygon `ref`, with position and
    /// target both at `pos`. The buffer holds one polygon at least.
    pub fn reset(self: *PathCorridor, ref: PolyRef, pos: [3]i32) void {
        self.pos = pos;
        self.target = pos;
        self.path[0] = ref;
        self.path_count = 1;
    }

    /// Finds the corners of the straight path from the position toward the
    /// target and returns how many were written. `corner_verts.len` is the
    /// capacity, and the flag and polygon slices are at least as long.
    ///
    /// Corners within `corner_distance_min` of the position in the xz plane
    /// are left out, and nothing follows the start of an off-mesh
    /// connection. The target, when in range, is the last corner, with a
    /// zero polygon reference. The corridor is not empty.
    pub fn findCorners(
        self: *const PathCorridor,
        corner_verts: [][3]i32,
        corner_flags: []StraightPathFlags,
        corner_polys: []PolyRef,
        query: *const NavMeshQuery,
    ) usize {
        std.debug.assert(self.path_count != 0);

        var corner_count: usize = 0;
        _ = query.findStraightPath(
            self.pos,
            self.target,
            self.path[0..self.path_count],
            corner_verts,
            corner_flags,
            corner_polys,
            &corner_count,
            .{},
        );

        // Prune points at the beginning of the path which are too close.
        while (corner_count != 0) {
            if (corner_flags[0].off_mesh_connection or
                common.vdist2dSqr(corner_verts[0], self.pos) >
                    corner_distance_min * corner_distance_min)
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

        return corner_count;
    }

    /// Shortens the start of the corridor when `next`, a point further
    /// along the path, is in straight sight of the position, by casting a
    /// ray toward it of `path_optimization_range` millimetres (above zero),
    /// as the float port does: a ray whose goal lies nearer than the range
    /// is lengthened to it. The corridor changes only when the ray gets
    /// past 99% of its length. Returns whether it got that far.
    pub fn optimizePathVisibility(
        self: *PathCorridor,
        next: [3]i32,
        path_optimization_range: u32,
        query: *const NavMeshQuery,
        filter: *const QueryFilter,
    ) bool {
        const goal = self.visibilityGoal(next, path_optimization_range) orelse return false;
        const result_count_max = 32;
        var result: [result_count_max]PolyRef = undefined;
        var t: Fraction = .zero;
        var result_count: usize = 0;
        _ = query.raycastPath(
            self.path[0],
            self.pos,
            goal,
            filter,
            &t,
            null,
            &result,
            &result_count,
        );
        if (result_count <= 1 or !passedNinetyNine(t)) return false;
        self.path_count = mergeCorridorStartShortcut(
            self.path,
            self.path_count,
            result[0..result_count],
        );
        return true;
    }

    /// The point `optimizePathVisibility` casts its ray to, or null when
    /// `next` is too near the position to be worth it.
    pub fn visibilityGoal(self: *const PathCorridor, next: [3]i32, range: u32) ?[3]i32 {
        // Lengths in 1/256 millimetre, as `common.length` gives them.
        const dist = common.length(common.vdist2dSqr(self.pos, next));
        const near: u64 = corner_distance_min << 8;
        if (dist < near) return null;
        const range_256: u64 = @as(u64, range) << 8;
        const clamped = @min(dist + near, range_256);
        const delta = common.vsub(next, self.pos);
        var goal: [3]i32 = undefined;
        for (&goal, self.pos, delta) |*g, p, d| {
            const along = common.divRound(@as(i128, d) * range_256, clamped);
            g.* = @intCast(std.math.clamp(p + along, -common.coord_max, common.coord_max));
        }
        return goal;
    }

    /// Shortens the start of the corridor with a short path search from
    /// the position toward the target, 32 polygons at most, splicing in
    /// what the search found. Returns whether the corridor was rewritten;
    /// a corridor under three polygons is left alone. The search runs on
    /// `query`, replacing any sliced search in progress there.
    pub fn optimizePathTopology(
        self: *PathCorridor,
        query: *NavMeshQuery,
        filter: *const QueryFilter,
    ) bool {
        if (self.path_count < 3) return false;

        const iteration_count_max = 32;
        const result_count_max = 32;

        var result: [result_count_max]PolyRef = undefined;
        var result_count: usize = 0;
        _ = query.initSlicedFindPath(
            self.path[0],
            self.path[self.path_count - 1],
            self.pos,
            self.target,
            filter,
            .{},
        );
        _ = query.updateSlicedFindPath(iteration_count_max, null);
        const status = query.finalizeSlicedFindPathPartial(
            self.path[0..self.path_count],
            &result,
            &result_count,
        );

        if (status.succeeded and result_count > 0) {
            self.path_count = mergeCorridorStartShortcut(
                self.path,
                self.path_count,
                result[0..result_count],
            );
            return true;
        }

        return false;
    }

    /// Advances the corridor past the off-mesh connection
    /// `off_mesh_con_ref`, as the float port does: the polygons up to and
    /// including it are removed and the position jumps to the connection's
    /// far end. Returns false, with the corridor unchanged, when the
    /// connection is not in the corridor; false with the corridor already
    /// advanced when the connection's ends cannot be found.
    pub fn moveOverOffmeshConnection(
        self: *PathCorridor,
        off_mesh_con_ref: PolyRef,
        refs: *[2]PolyRef,
        start_pos: *[3]i32,
        end_pos: *[3]i32,
        query: *const NavMeshQuery,
    ) bool {
        std.debug.assert(self.path_count != 0);
        const path = self.path;

        // Advance the path up to and over the off-mesh connection.
        var prev_ref: PolyRef = 0;
        var poly_ref = path[0];
        var npos: usize = 0;
        while (npos < self.path_count and poly_ref != off_mesh_con_ref) {
            prev_ref = poly_ref;
            poly_ref = path[npos];
            npos += 1;
        }
        // The connection was not found.
        if (npos == self.path_count) return false;

        // Prune the path.
        for (npos..self.path_count) |i| path[i - npos] = path[i];
        self.path_count -= npos;

        refs[0] = prev_ref;
        refs[1] = poly_ref;

        const nav = query.getAttachedNavMesh();
        const status = nav.getOffMeshConnectionPolyEndPoints(refs[0], refs[1], start_pos, end_pos);
        if (status.succeeded) {
            self.pos = end_pos.*;
            return true;
        }
        return false;
    }

    /// Moves the position toward `new_pos` along the surface of the mesh
    /// and rewrites the start of the corridor to match, so the position
    /// stays in the corridor's first polygon. Returns whether the move ran;
    /// on false nothing changed. The position ends short of `new_pos` when
    /// that point is off the mesh or beyond the reach of the local search.
    pub fn movePosition(
        self: *PathCorridor,
        new_pos: [3]i32,
        query: *NavMeshQuery,
        filter: *const QueryFilter,
    ) bool {
        std.debug.assert(self.path_count != 0);

        var result: [3]i32 = undefined;
        const visited_count_max = 16;
        var visited: [visited_count_max]PolyRef = undefined;
        var visited_count: usize = 0;
        const status = query.moveAlongSurface(
            self.path[0],
            self.pos,
            new_pos,
            filter,
            &result,
            &visited,
            &visited_count,
        );
        if (!status.succeeded) return false;
        self.path_count = mergeCorridorStartMoved(
            self.path,
            self.path_count,
            visited[0..visited_count],
        );
        // Keep the position on top of the mesh. Where the polygon has no
        // height there, the old height stays.
        var h = self.pos[1];
        _ = query.getPolyHeight(self.path[0], result, &h);
        result[1] = h;
        self.pos = result;
        return true;
    }

    /// Moves the target toward `new_pos` along the surface of the mesh and
    /// rewrites the end of the corridor to match, so the target stays in
    /// the corridor's last polygon. Returns whether the move ran; on false
    /// nothing changed. The target's height is left as the move gives it.
    pub fn moveTargetPosition(
        self: *PathCorridor,
        new_pos: [3]i32,
        query: *NavMeshQuery,
        filter: *const QueryFilter,
    ) bool {
        std.debug.assert(self.path_count != 0);

        var result: [3]i32 = undefined;
        const visited_count_max = 16;
        var visited: [visited_count_max]PolyRef = undefined;
        var visited_count: usize = 0;
        const status = query.moveAlongSurface(
            self.path[self.path_count - 1],
            self.target,
            new_pos,
            filter,
            &result,
            &visited,
            &visited_count,
        );
        if (!status.succeeded) return false;
        self.path_count = mergeCorridorEndMoved(
            self.path,
            self.path_count,
            visited[0..visited_count],
        );
        self.target = result;
        return true;
    }

    /// Loads a path and its target: `polys` runs from the polygon holding
    /// the corridor's position to the one holding `target`. It is not
    /// empty, no longer than the buffer, and not part of it.
    pub fn setCorridor(self: *PathCorridor, target: [3]i32, polys: []const PolyRef) void {
        std.debug.assert(polys.len > 0);
        std.debug.assert(polys.len <= self.path.len);
        self.target = target;
        @memcpy(self.path[0..polys.len], polys);
        self.path_count = polys.len;
    }

    /// Restarts a corridor whose first polygon went bad from `safe_ref` at
    /// `safe_pos`, as the float port does: the second polygon becomes zero,
    /// a gap the next replan closes, and a corridor of one or two polygons
    /// becomes three. The buffer holds three at least.
    pub fn fixPathStart(self: *PathCorridor, safe_ref: PolyRef, safe_pos: [3]i32) void {
        const path = self.path;
        self.pos = safe_pos;
        if (self.path_count < 3 and self.path_count > 0) {
            path[2] = path[self.path_count - 1];
            path[0] = safe_ref;
            path[1] = 0;
            self.path_count = 3;
        } else {
            path[0] = safe_ref;
            path[1] = 0;
        }
    }

    /// Cuts the corridor at its first polygon that no longer exists or
    /// passes `filter`, and pulls the target onto the boundary of the new
    /// last polygon. When the first polygon itself is bad the corridor
    /// becomes `safe_ref` with the position at `safe_pos`. A corridor that
    /// is all valid is left alone.
    pub fn trimInvalidPath(
        self: *PathCorridor,
        safe_ref: PolyRef,
        safe_pos: [3]i32,
        query: *const NavMeshQuery,
        filter: *const QueryFilter,
    ) void {
        const path = self.path;
        var n: usize = 0;
        while (n < self.path_count and query.isValidPolyRef(path[n], filter)) n += 1;

        if (n == self.path_count) return;
        if (n == 0) {
            self.pos = safe_pos;
            path[0] = safe_ref;
            self.path_count = 1;
        } else {
            self.path_count = n;
        }

        // Clamp the target to the last polygon. The copy keeps the query's
        // input apart from the target it writes.
        const target = self.target;
        _ = query.closestPointOnPolyBoundary(path[self.path_count - 1], target, &self.target);
    }

    /// Whether the first `look_ahead_count_max` polygons of the corridor
    /// all still exist and pass `filter`.
    pub fn isValid(
        self: *const PathCorridor,
        look_ahead_count_max: usize,
        query: *const NavMeshQuery,
        filter: *const QueryFilter,
    ) bool {
        const n = @min(self.path_count, look_ahead_count_max);
        for (self.path[0..n]) |ref| {
            if (!query.isValidPolyRef(ref, filter)) return false;
        }
        return true;
    }

    /// The polygon holding the position, or zero when there is no path.
    pub fn getFirstPoly(self: *const PathCorridor) PolyRef {
        return if (self.path_count != 0) self.path[0] else 0;
    }

    /// The polygon holding the target, or zero when there is no path.
    pub fn getLastPoly(self: *const PathCorridor) PolyRef {
        return if (self.path_count != 0) self.path[self.path_count - 1] else 0;
    }

    /// The corridor's polygons, from the position to the target: a view of
    /// the caller's buffer, which every call that rewrites the corridor
    /// changes.
    pub fn getPath(self: *const PathCorridor) []const PolyRef {
        return self.path[0..self.path_count];
    }
};

/// Whether `t` lies past 99% of a ray, as the float port's `t > 0.99`; a
/// ray that reached its end is.
pub fn passedNinetyNine(t: Fraction) bool {
    if (t.isNever()) return true;
    return @as(i128, t.num) * 100 > @as(i128, t.den) * 99;
}
