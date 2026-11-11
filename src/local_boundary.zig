//! The wall segments near an agent: the solid polygon edges within a range
//! of a position, nearest first, which the crowd hands to obstacle
//! avoidance as the walls to steer clear of.

const std = @import("std");
const common = @import("common.zig");
const nav_mesh = @import("nav_mesh.zig");
const nav_mesh_query = @import("nav_mesh_query.zig");

const PolyRef = nav_mesh.PolyRef;
const NavMeshQuery = nav_mesh_query.NavMeshQuery;
const QueryFilter = nav_mesh_query.QueryFilter;

/// The nearest wall segments around a center, and the polygons they were
/// gathered from. It allocates nothing: a plain value that starts as
/// `empty`.
pub const LocalBoundary = struct {
    /// Where the boundary was last gathered; the largest float on every
    /// axis while it holds nothing.
    center: [3]f32,
    /// The kept segments, nearest first; `segment_count` are in use.
    segments: [segment_count_max]Segment,
    segment_count: i32,
    /// The polygons around the center the segments came from;
    /// `poly_count` are in use.
    polys: [poly_count_max]PolyRef,
    poly_count: i32,

    /// The most wall segments a boundary keeps.
    pub const segment_count_max = 8;
    /// The most polygons a boundary gathers segments from.
    pub const poly_count_max = 16;

    const Segment = struct {
        /// The segment's two ends.
        s: [2][3]f32,
        /// The squared xz distance from the center, which orders the
        /// segments.
        d: f32,
    };

    const center_unset: [3]f32 = @splat(std.math.floatMax(f32));

    /// A boundary that holds no segments and no polygons.
    pub const empty: LocalBoundary = .{
        .center = center_unset,
        .segments = undefined,
        .segment_count = 0,
        .polys = undefined,
        .poly_count = 0,
    };

    /// Drops every segment and polygon, so `isValid` is false until the
    /// next `update`.
    pub fn reset(self: *LocalBoundary) void {
        self.center = center_unset;
        self.poly_count = 0;
        self.segment_count = 0;
    }

    /// Keeps `s` in distance order when it is among the nearest
    /// `segment_count_max`; a full list drops its farthest segment.
    fn addSegment(self: *LocalBoundary, dist: f32, s: [2][3]f32) void {
        // Insert the segment by its distance.
        var index: i32 = undefined;
        if (self.segment_count == 0) {
            // The first one always goes in.
            index = 0;
        } else if (dist >= self.segments[@intCast(self.segment_count - 1)].d) {
            // Farther than the last segment: there is room, or it is left.
            if (self.segment_count >= segment_count_max) return;
            index = self.segment_count;
        } else {
            // Somewhere before the last: shift the rest one place up.
            var i: i32 = 0;
            while (i < self.segment_count) : (i += 1) {
                if (dist <= self.segments[@intCast(i)].d) break;
            }
            // A NaN distance compares false with every segment and so lands
            // past the last one; in a full list there is no place for it.
            if (i >= segment_count_max) return;
            const target = i + 1;
            const n = @min(self.segment_count - i, segment_count_max - target);
            std.debug.assert(target + n <= segment_count_max);
            if (n > 0) {
                const from: usize = @intCast(i);
                const to: usize = @intCast(target);
                const len: usize = @intCast(n);
                @memmove(self.segments[to..][0..len], self.segments[from..][0..len]);
            }
            index = i;
        }

        const segment = &self.segments[@intCast(index)];
        segment.d = dist;
        segment.s = s;

        if (self.segment_count < segment_count_max) self.segment_count += 1;
    }

    /// Gathers the boundary again around `pos` on polygon `ref`: the wall
    /// segments of the polygons within `collision_query_range` that lie
    /// within that range, nearest first. A zero `ref` resets the boundary.
    pub fn update(
        self: *LocalBoundary,
        ref: PolyRef,
        pos: [3]f32,
        collision_query_range: f32,
        query: *NavMeshQuery,
        filter: *const QueryFilter,
    ) void {
        const segments_per_poly_max = nav_mesh.verts_per_polygon * 3;

        if (ref == 0) {
            self.center = center_unset;
            self.segment_count = 0;
            self.poly_count = 0;
            return;
        }

        self.center = pos;

        // First query the polygons around the position that do not overlap.
        // A query that fails before it counts leaves the count as it was.
        var poly_count: usize = @intCast(self.poly_count);
        _ = query.findLocalNeighbourhood(
            ref,
            pos,
            collision_query_range,
            filter,
            &self.polys,
            null,
            &poly_count,
        );
        self.poly_count = @intCast(poly_count);

        // Then store the wall segments of all of them.
        self.segment_count = 0;
        var segments: [segments_per_poly_max][2][3]f32 = undefined;
        var segment_count: usize = 0;
        for (self.polys[0..poly_count]) |poly| {
            _ = query.getPolyWallSegments(
                poly,
                filter,
                &segments,
                null,
                &segment_count,
            );
            for (segments[0..segment_count]) |s| {
                // Skip segments too far away.
                const dist_sqr = common.distancePointSegmentSqr2d(pos, s[0], s[1]).dist_sqr;
                if (dist_sqr > common.sqr(f32, collision_query_range)) continue;
                self.addSegment(dist_sqr, s);
            }
        }
    }

    /// Whether the boundary holds polygons and all of them still exist and
    /// pass `filter`. False after `reset`.
    pub fn isValid(
        self: *const LocalBoundary,
        query: *NavMeshQuery,
        filter: *const QueryFilter,
    ) bool {
        if (self.poly_count == 0) return false;

        // Check that all polygons still pass the query filter.
        for (self.polys[0..@intCast(self.poly_count)]) |poly| {
            if (!query.isValidPolyRef(poly, filter)) return false;
        }

        return true;
    }

    /// Where the boundary was last gathered.
    pub fn getCenter(self: *const LocalBoundary) [3]f32 {
        return self.center;
    }

    /// The number of segments held, at most `segment_count_max`.
    pub fn getSegmentCount(self: *const LocalBoundary) i32 {
        return self.segment_count;
    }

    /// The two ends of segment `i`, `i` below `getSegmentCount()`. Lower
    /// indexes are nearer the center.
    pub fn getSegment(self: *const LocalBoundary, i: i32) [2][3]f32 {
        return self.segments[@intCast(i)].s;
    }
};
