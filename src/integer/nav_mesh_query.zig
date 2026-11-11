//! Queries on a navigation mesh in whole millimetres: nearest polygons,
//! random points, polygon corridors by A* in one call or in slices, straight
//! paths, ray casts and searches around a point. Also the filter that says
//! which polygons a query may cross and what crossing them costs.
//!
//! Positions are `[3]i32` in millimetres, each coordinate within
//! `common.coord_max` of zero, and costs are `u64` in 1/256 millimetre at
//! area cost one. Nothing here uses a floating-point number.

const std = @import("std");
const build_options = @import("build_options");
const common = @import("common.zig");
const Status = @import("../status.zig").Status;
const nav_mesh = @import("nav_mesh.zig");
const node = @import("node.zig");

const Fraction = common.Fraction;
const NavMesh = nav_mesh.NavMesh;
const MeshTile = nav_mesh.MeshTile;
const Poly = nav_mesh.Poly;
const Link = nav_mesh.Link;
const PolyRef = nav_mesh.PolyRef;
const PolyType = nav_mesh.PolyType;
const TileAndPoly = nav_mesh.TileAndPoly;
const FindPathOptions = nav_mesh.FindPathOptions;
const RaycastOptions = nav_mesh.RaycastOptions;
const StraightPathFlags = nav_mesh.StraightPathFlags;
const StraightPathOptions = nav_mesh.StraightPathOptions;
const Node = node.Node;
const NodeFlags = node.NodeFlags;
const NodePool = node.NodePool;
const NodeQueue = node.NodeQueue;

const invalid_param: Status = .{ .failed = true, .invalid_param = true };
const out_of_memory: Status = .{ .failed = true, .out_of_memory = true };

/// Scales the straight-line estimate of the cost left to just under the
/// distance, in thousandths, so that rounding never lifts it above the real
/// cost. The scaled estimate is rounded down.
const h_scale: u64 = 999;

/// Which polygons a query may cross, and what crossing them costs.
///
/// A polygon passes when it has at least one of `include_flags` and none of
/// `exclude_flags`. Crossing it costs the distance walked times its area's
/// entry in `area_cost`.
///
/// With the `virtual_query_filter` build option a filter also carries a
/// table of the two functions, so an application can put its own in place:
/// its functions get the `*const QueryFilter` and reach their own fields
/// around it with `@fieldParentPtr`.
pub const QueryFilter = struct {
    /// The cost per unit of distance, by area id, in Q16.16: `1 << 16` is
    /// cost one.
    area_cost: [nav_mesh.area_count_max]u32 = @splat(1 << 16),
    /// A polygon needs one of these flags to pass.
    include_flags: u16 = 0xffff,
    /// A polygon with one of these flags does not pass.
    exclude_flags: u16 = 0,
    /// The functions `passFilter` and `getCost` call. The table must outlive
    /// the filter.
    vtable: if (build_options.virtual_query_filter) *const VTable else void =
        if (build_options.virtual_query_filter) &default_vtable else {},

    /// A filter's two functions, replaceable under `virtual_query_filter`.
    /// `getCost` returns a cost in 1/256 millimetre at area cost one.
    pub const VTable = struct {
        passFilter: *const fn (
            filter: *const QueryFilter,
            ref: PolyRef,
            tile: *const MeshTile,
            poly: *const Poly,
        ) bool,
        getCost: *const fn (
            filter: *const QueryFilter,
            pa: [3]i32,
            pb: [3]i32,
            prev_ref: PolyRef,
            prev_tile: ?*const MeshTile,
            prev_poly: ?*const Poly,
            cur_ref: PolyRef,
            cur_tile: *const MeshTile,
            cur_poly: *const Poly,
            next_ref: PolyRef,
            next_tile: ?*const MeshTile,
            next_poly: ?*const Poly,
        ) u64,
    };

    /// The flag test and the area cost described on `QueryFilter`.
    pub const default_vtable: VTable = .{
        .passFilter = passFilterByFlags,
        .getCost = getCostByArea,
    };

    fn passFilterByFlags(
        self: *const QueryFilter,
        ref: PolyRef,
        tile: *const MeshTile,
        poly: *const Poly,
    ) bool {
        _ = ref;
        _ = tile;
        return (poly.flags & self.include_flags) != 0 and (poly.flags & self.exclude_flags) == 0;
    }

    fn getCostByArea(
        self: *const QueryFilter,
        pa: [3]i32,
        pb: [3]i32,
        prev_ref: PolyRef,
        prev_tile: ?*const MeshTile,
        prev_poly: ?*const Poly,
        cur_ref: PolyRef,
        cur_tile: *const MeshTile,
        cur_poly: *const Poly,
        next_ref: PolyRef,
        next_tile: ?*const MeshTile,
        next_poly: ?*const Poly,
    ) u64 {
        _ = prev_ref;
        _ = prev_tile;
        _ = prev_poly;
        _ = cur_ref;
        _ = cur_tile;
        _ = next_ref;
        _ = next_tile;
        _ = next_poly;
        // The length times a Q16.16 cost, widened before the product and
        // held at the largest cost.
        const walked: u128 = common.vdist(pa, pb);
        const cost = (walked * self.area_cost[cur_poly.getArea()]) >> 16;
        return @intCast(@min(cost, std.math.maxInt(u64)));
    }

    /// Whether a query may cross the polygon.
    pub inline fn passFilter(
        self: *const QueryFilter,
        ref: PolyRef,
        tile: *const MeshTile,
        poly: *const Poly,
    ) bool {
        if (build_options.virtual_query_filter) {
            return self.vtable.passFilter(self, ref, tile, poly);
        }
        return passFilterByFlags(self, ref, tile, poly);
    }

    /// The cost of walking from `pa` to `pb`, both in the current polygon:
    /// `pa` on the edge shared with the previous polygon, `pb` on the edge
    /// shared with the next. The previous polygon is zero and null at the
    /// start of a path, the next at its end. The cost is in 1/256
    /// millimetre at area cost one, and never above the largest `u64`.
    pub inline fn getCost(
        self: *const QueryFilter,
        pa: [3]i32,
        pb: [3]i32,
        prev_ref: PolyRef,
        prev_tile: ?*const MeshTile,
        prev_poly: ?*const Poly,
        cur_ref: PolyRef,
        cur_tile: *const MeshTile,
        cur_poly: *const Poly,
        next_ref: PolyRef,
        next_tile: ?*const MeshTile,
        next_poly: ?*const Poly,
    ) u64 {
        if (build_options.virtual_query_filter) {
            return self.vtable.getCost(
                self,
                pa,
                pb,
                prev_ref,
                prev_tile,
                prev_poly,
                cur_ref,
                cur_tile,
                cur_poly,
                next_ref,
                next_tile,
                next_poly,
            );
        }
        // Inlined: a call would read the positions back from memory just
        // after the caller stored their parts, and wait for the stores.
        return @call(.always_inline, getCostByArea, .{
            self,
            pa,
            pb,
            prev_ref,
            prev_tile,
            prev_poly,
            cur_ref,
            cur_tile,
            cur_poly,
            next_ref,
            next_tile,
            next_poly,
        });
    }
};

/// What `NavMeshQuery.raycast` found.
pub const RaycastHit = struct {
    /// How far along the ray the wall was hit, zero at the start and one at
    /// the end; `never` when the ray reached its end with no hit.
    t: Fraction,
    /// The normal of the wall that was hit, in Q16.16: `1 << 16` is length
    /// one.
    hit_normal: [3]i32,
    /// The edge of the last polygon where the wall was hit.
    hit_edge_index: i32,
    /// The caller's buffer for the polygons the ray crossed, as many as it
    /// holds. Empty when the caller wants none.
    path: []PolyRef,
    /// How many polygons were written to `path`.
    path_count: usize,
    /// The filter's cost of the walk up to the hit, in 1/256 millimetre at
    /// area cost one.
    path_cost: u64,

    /// Where the ray from `start` to `end` stopped: the point at `t` along
    /// it, each axis rounded to the nearest millimetre, or `end` when
    /// nothing was hit before it.
    pub fn point(self: *const RaycastHit, start: [3]i32, end: [3]i32) [3]i32 {
        if (self.t.atLeastOne()) return end;
        return common.lerp(start, end, self.t);
    }
};

/// Receives the polygons `NavMeshQuery.queryPolygonsWith` finds, in batches.
pub const PolyQuery = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        /// `polys` and `refs` name the same polygons of `tile`, entry by
        /// entry. Both slices are only valid during the call.
        process: *const fn (
            ptr: *anyopaque,
            tile: *const MeshTile,
            polys: []const *const Poly,
            refs: []const PolyRef,
        ) void,
    };

    /// Hands one batch of a tile's polygons to the receiver.
    pub fn process(
        self: PolyQuery,
        tile: *const MeshTile,
        polys: []const *const Poly,
        refs: []const PolyRef,
    ) void {
        self.vtable.process(self.ptr, tile, polys, refs);
    }
};

/// A sliced path search between `initSlicedFindPath` and its finalize call.
///
/// `last_best_node` points into the query's node pool and `filter` at the
/// caller's filter. Neither is checked: the filter must stay valid until
/// the search is finalized, and no other search may run on the same query
/// in between, since every search clears the pool.
pub const SlicedPathSearch = struct {
    status: Status,
    /// The node nearest to the goal so far.
    last_best_node: ?*Node,
    /// That node's estimate of the cost left, in 1/256 millimetre.
    last_best_node_cost: u64,
    start_ref: PolyRef,
    end_ref: PolyRef,
    start_pos: [3]i32,
    end_pos: [3]i32,
    filter: ?*const QueryFilter,
    options: FindPathOptions,
    /// How far, squared, a shortcut ray may reach in an any-angle search,
    /// in square millimetres.
    raycast_limit_sqr: u64,

    /// No search: every field zero.
    pub const idle: SlicedPathSearch = .{
        .status = .{},
        .last_best_node = null,
        .last_best_node_cost = 0,
        .start_ref = 0,
        .end_ref = 0,
        .start_pos = .{ 0, 0, 0 },
        .end_pos = .{ 0, 0, 0 },
        .filter = null,
        .options = .{},
        .raycast_limit_sqr = 0,
    };
};

/// Path and proximity queries on one nav mesh.
///
/// A query starts as `empty`; `init` attaches it to a mesh and readies its
/// search memory, and `deinit`, given the same allocator, ends it. The mesh
/// must outlive the query, and a polygon reference is only as good as the
/// mesh's tiles: removing a tile ends every search that reached it.
///
/// The searches share the query's node pools, so one query serves one
/// thread at a time, and each search overwrites what the one before left in
/// the pool.
///
/// Every position, extent and radius a query takes must lie within
/// `common.coord_max` of zero; a query refuses any other with
/// `invalid_param`.
pub const NavMeshQuery = struct {
    /// The mesh the query reads. Null until `init`.
    nav: ?*const NavMesh,
    /// The sliced path search under way.
    query: SlicedPathSearch,
    /// The nodes of the searches that stay near their start.
    tiny_node_pool: NodePool,
    /// The nodes of the path searches and the searches around a point.
    node_pool: NodePool,
    /// The open list of every search.
    open_list: NodeQueue,

    /// A query attached to no mesh. Only `init` and `deinit` may be called
    /// on it.
    pub const empty: NavMeshQuery = .{
        .nav = null,
        .query = .idle,
        .tiny_node_pool = .empty,
        .node_pool = .empty,
        .open_list = .empty,
    };

    /// Attaches the query to `nav` and readies its search memory for
    /// `node_count_max` nodes, above zero and at most `node.null_index`.
    ///
    /// It may be called again, with the same allocator, to attach another
    /// mesh or to grow: memory that is large enough is kept. When memory
    /// runs out everything is freed and the query is `empty` again.
    pub fn init(
        self: *NavMeshQuery,
        gpa: std.mem.Allocator,
        nav: *const NavMesh,
        node_count_max: i32,
    ) Status {
        if (node_count_max > node.null_index or
            node_count_max > (1 << node.node_parent_bits) - 1)
        {
            return invalid_param;
        }

        self.nav = nav;
        self.readySearchMemory(gpa, node_count_max) catch {
            self.tiny_node_pool.deinit(gpa);
            self.node_pool.deinit(gpa);
            self.open_list.deinit(gpa);
            self.* = .empty;
            return out_of_memory;
        };
        return .success;
    }

    fn readySearchMemory(
        self: *NavMeshQuery,
        gpa: std.mem.Allocator,
        node_count_max: i32,
    ) error{OutOfMemory}!void {
        if (self.node_pool.node_count_max == 0 or
            self.node_pool.getMaxNodes() < node_count_max)
        {
            self.node_pool.deinit(gpa);
            self.node_pool = .empty;
            // A quarter of the nodes rounds to no bucket at all below four
            // nodes, and a pool cannot hash into nothing.
            const quarter: u32 = @bitCast(@divTrunc(node_count_max, 4));
            const hash_size: u32 = @max(1, common.nextPow2(quarter));
            try self.node_pool.init(gpa, node_count_max, @bitCast(hash_size));
        } else {
            self.node_pool.clear();
        }

        if (self.tiny_node_pool.node_count_max == 0) {
            try self.tiny_node_pool.init(gpa, 64, 32);
        } else {
            self.tiny_node_pool.clear();
        }

        if (self.open_list.capacity == 0 or self.open_list.getCapacity() < node_count_max) {
            self.open_list.deinit(gpa);
            self.open_list = .empty;
            try self.open_list.init(gpa, node_count_max);
        } else {
            self.open_list.clear();
        }
    }

    /// Frees the search memory with the allocator `init` was given. Every
    /// `*Node` and `*NodePool` from the query is invalid afterwards.
    pub fn deinit(self: *NavMeshQuery, gpa: std.mem.Allocator) void {
        self.tiny_node_pool.deinit(gpa);
        self.node_pool.deinit(gpa);
        self.open_list.deinit(gpa);
        self.* = undefined;
    }

    /// A random point on the mesh, on a polygon `filter` passes.
    ///
    /// A tile is picked first, each with the same chance, then a polygon of
    /// it by area, then a point of the polygon; so the points are spread
    /// evenly only where the tiles hold about the same area. Fails when the
    /// mesh has no tile or the picked tile has no passing ground polygon.
    ///
    /// Each draw is a `u32` from `random`, read as a number in 0..1 with
    /// 2^32 as one.
    pub fn findRandomPoint(
        self: *const NavMeshQuery,
        filter: *const QueryFilter,
        random: std.Random,
        random_ref: *PolyRef,
        random_pt: *[3]i32,
    ) Status {
        const nav = self.nav.?;

        // Reservoir sampling: the tile seen as number k replaces the pick
        // with a chance of one in k, so one pass picks evenly.
        var picked_tile: ?*const MeshTile = null;
        var tile_sum: u64 = 0;
        var tile_index: i32 = 0;
        while (tile_index < nav.getMaxTiles()) : (tile_index += 1) {
            const tile = nav.getTile(tile_index);
            if (tile.header == null) continue;

            const area: u64 = 1;
            tile_sum += area;
            const u: u64 = random.int(u32);
            // A draw below 2^32 times a count of tiles below 2^31.
            if (u * tile_sum <= area << 32) picked_tile = tile;
        }
        const tile = picked_tile orelse return .failure;

        // The same sampling over the tile's polygons, weighted by area.
        var picked_poly: ?*const Poly = null;
        var poly_ref: PolyRef = 0;
        const base = nav.getPolyRefBase(tile);

        var area_sum: i128 = 0;
        for (tile.polys, 0..) |*candidate, poly_index| {
            // Off-mesh connections have no surface to stand on.
            if (candidate.getType() != .ground) continue;
            const ref = base | @as(PolyRef, @intCast(poly_index));
            if (!filter.passFilter(ref, tile, candidate)) continue;

            const poly_area: i128 = polyArea2d(tile, candidate);
            area_sum += poly_area;
            const u: i128 = random.int(u32);
            // An area is at most 2^62 either way and a tile holds fewer
            // than 2^31 polygons, so both sides stay below 2^125.
            if (u * area_sum <= poly_area * (1 << 32)) {
                picked_poly = candidate;
                poly_ref = ref;
            }
        }
        const poly = picked_poly orelse return .failure;

        const s = random.int(u32);
        const t = random.int(u32);
        const pt = randomPointInPoly(tile, poly, s, t);

        // Lifts the point onto the detail surface; it stays as it is when
        // the lookup refuses it.
        var closest = pt;
        _ = self.closestPointOnPoly(poly_ref, pt, &closest, null);

        random_pt.* = closest;
        random_ref.* = poly_ref;
        return .success;
    }

    /// A random point within `radius_max` millimetres of `center_pos`, on a
    /// polygon `filter` passes that can be reached from `start_ref`.
    ///
    /// The polygon is picked by area among those the circle touches, and the
    /// point anywhere on it: the point itself may lie outside the circle.
    /// The status carries `out_of_nodes` when the search ran out of nodes
    /// and so did not see every polygon.
    ///
    /// Each draw is a `u32` from `random`, read as a number in 0..1 with
    /// 2^32 as one.
    pub fn findRandomPointAroundCircle(
        self: *NavMeshQuery,
        start_ref: PolyRef,
        center_pos: [3]i32,
        radius_max: i32,
        filter: *const QueryFilter,
        random: std.Random,
        random_ref: *PolyRef,
        random_pt: *[3]i32,
    ) Status {
        const nav = self.nav.?;

        if (!nav.isValidPolyRef(start_ref) or
            !common.inBounds(center_pos) or
            radius_max < 0 or
            radius_max > common.coord_max)
        {
            return invalid_param;
        }

        const start = nav.getTileAndPolyByRefUnsafe(start_ref);
        if (!filter.passFilter(start_ref, start.tile, start.poly)) return invalid_param;

        self.node_pool.clear();
        self.open_list.clear();

        const start_node = self.node_pool.getNode(start_ref, 0).?;
        start_node.pos = center_pos;
        start_node.bits.parent_index = 0;
        start_node.cost = 0;
        start_node.total = 0;
        start_node.id = start_ref;
        start_node.bits.flags = .{ .open = true };
        self.open_list.push(start_node);

        var status: Status = .success;

        const radius: u64 = @intCast(radius_max);
        const radius_sqr = radius * radius;
        var area_sum: i128 = 0;

        var picked: ?TileAndPoly = null;
        var picked_ref: PolyRef = 0;

        while (!self.open_list.isEmpty()) {
            const best_node = self.open_list.pop();
            best_node.bits.flags.open = false;
            best_node.bits.flags.closed = true;

            // Every reference in the pool came from a checked start or from
            // a link of the mesh, so the lookups skip the checks.
            const best_ref = best_node.id;
            const best = nav.getTileAndPolyByRefUnsafe(best_ref);
            const best_tile = best.tile;
            const best_poly = best.poly;

            // Reservoir sampling by area over the ground polygons reached.
            if (best_poly.getType() == .ground) {
                const poly_area: i128 = polyArea2d(best_tile, best_poly);
                area_sum += poly_area;
                const u: i128 = random.int(u32);
                // An area is at most 2^62 either way and the pool holds
                // fewer than 2^16 nodes, so both sides stay below 2^110.
                if (u * area_sum <= poly_area * (1 << 32)) {
                    picked = best;
                    picked_ref = best_ref;
                }
            }

            var parent_ref: PolyRef = 0;
            if (best_node.bits.parent_index != 0) {
                parent_ref = self.node_pool.getNodeAtIdx(best_node.bits.parent_index).?.id;
            }

            var link_index = best_poly.first_link;
            while (link_index != nav_mesh.null_link) {
                const link = best_tile.links[link_index];
                link_index = link.next;

                const neighbour_ref = link.ref;
                // Never walk straight back to the parent.
                if (neighbour_ref == 0 or neighbour_ref == parent_ref) continue;

                const neighbour = nav.getTileAndPolyByRefUnsafe(neighbour_ref);
                const neighbour_tile = neighbour.tile;
                const neighbour_poly = neighbour.poly;

                if (!filter.passFilter(neighbour_ref, neighbour_tile, neighbour_poly)) continue;

                // The edge shared with the neighbour. A link that names no
                // shared edge leaves it at the origin and is still followed.
                var va: [3]i32 = .{ 0, 0, 0 };
                var vb: [3]i32 = .{ 0, 0, 0 };
                _ = self.getPortalPointsResolved(
                    best_ref,
                    best_poly,
                    best_tile,
                    neighbour_ref,
                    neighbour_poly,
                    neighbour_tile,
                    &va,
                    &vb,
                );

                // The circle does not reach the neighbour.
                const dist_sqr = common.distancePointSegmentSqr2d(center_pos, va, vb).dist_sqr;
                if (dist_sqr > radius_sqr) continue;

                const neighbour_node = self.node_pool.getNode(neighbour_ref, 0) orelse {
                    status.out_of_nodes = true;
                    continue;
                };

                if (neighbour_node.bits.flags.closed) continue;

                if (isUnvisited(neighbour_node)) {
                    neighbour_node.pos = common.lerp(va, vb, .half);
                }

                const total = best_node.total +| common.vdist(best_node.pos, neighbour_node.pos);

                // Already queued by a way that is no longer.
                if (neighbour_node.bits.flags.open and total >= neighbour_node.total) continue;

                neighbour_node.id = neighbour_ref;
                neighbour_node.bits.flags.closed = false;
                neighbour_node.bits.parent_index = @intCast(self.node_pool.getNodeIdx(best_node));
                neighbour_node.total = total;

                if (neighbour_node.bits.flags.open) {
                    self.open_list.modify(neighbour_node);
                } else {
                    neighbour_node.bits.flags = .{ .open = true };
                    self.open_list.push(neighbour_node);
                }
            }
        }

        const found = picked orelse return .failure;

        const s = random.int(u32);
        const t = random.int(u32);
        const pt = randomPointInPoly(found.tile, found.poly, s, t);

        // Lifts the point onto the detail surface; it stays as it is when
        // the lookup refuses it.
        var closest = pt;
        _ = self.closestPointOnPoly(picked_ref, pt, &closest, null);

        random_pt.* = closest;
        random_ref.* = picked_ref;
        return status;
    }

    /// The point of polygon `ref` nearest to `pos`, with its height from the
    /// detail surface when `pos` lies over the polygon; `pos_over_poly`
    /// says whether it does. Unlike `closestPointOnPolyBoundary` this reads
    /// the detail mesh.
    pub fn closestPointOnPoly(
        self: *const NavMeshQuery,
        ref: PolyRef,
        pos: [3]i32,
        closest: *[3]i32,
        pos_over_poly: ?*bool,
    ) Status {
        const nav = self.nav.?;
        if (!nav.isValidPolyRef(ref) or !common.inBounds(pos)) return invalid_param;

        const found = nav.closestPointOnPoly(ref, pos);
        closest.* = found.closest;
        if (pos_over_poly) |over| over.* = found.pos_over_poly;
        return .success;
    }

    /// `pos` itself when it lies inside polygon `ref` in the xz plane, else
    /// the nearest point on the polygon's outline, each axis rounded to the
    /// nearest millimetre. Cheaper than `closestPointOnPoly`: the detail
    /// mesh is not read, so the height is that of `pos` or of the outline.
    pub fn closestPointOnPolyBoundary(
        self: *const NavMeshQuery,
        ref: PolyRef,
        pos: [3]i32,
        closest: *[3]i32,
    ) Status {
        const nav = self.nav.?;

        var tile: *const MeshTile = undefined;
        var poly: *const Poly = undefined;
        if (nav.getTileAndPolyByRef(ref, &tile, &poly).failed) return invalid_param;

        if (!common.inBounds(pos)) return invalid_param;

        var verts: [nav_mesh.verts_per_polygon][3]i32 = undefined;
        var edge_dist_sqr: [nav_mesh.verts_per_polygon]u64 = undefined;
        var edge_t: [nav_mesh.verts_per_polygon]Fraction = undefined;
        const vert_count: usize = poly.vert_count;
        for (verts[0..vert_count], poly.verts[0..vert_count]) |*vert, vert_index| {
            vert.* = tile.verts[vert_index];
        }

        const inside = common.distancePointPolyEdgesSqr(
            pos,
            verts[0..vert_count],
            &edge_dist_sqr,
            &edge_t,
        );
        if (inside) {
            closest.* = pos;
        } else {
            // Outside: the nearest point of the nearest edge. Of edges as
            // near as each other the first stays.
            var dist_sqr_min = edge_dist_sqr[0];
            var edge_min: usize = 0;
            var edge: usize = 1;
            while (edge < vert_count) : (edge += 1) {
                if (edge_dist_sqr[edge] < dist_sqr_min) {
                    dist_sqr_min = edge_dist_sqr[edge];
                    edge_min = edge;
                }
            }
            const va = verts[edge_min];
            const vb = verts[(edge_min + 1) % vert_count];
            closest.* = common.lerp(va, vb, edge_t[edge_min]);
        }

        return .success;
    }

    /// The height of polygon `ref`'s detail surface under `pos`, or along an
    /// off-mesh connection the height of the point of it nearest to `pos`,
    /// rounded to the nearest millimetre. Fails when `pos` is outside the
    /// polygon in the xz plane. A null `height` only asks whether the call
    /// would succeed.
    pub fn getPolyHeight(
        self: *const NavMeshQuery,
        ref: PolyRef,
        pos: [3]i32,
        height: ?*i32,
    ) Status {
        const nav = self.nav.?;

        var tile: *const MeshTile = undefined;
        var poly: *const Poly = undefined;
        if (nav.getTileAndPolyByRef(ref, &tile, &poly).failed) return invalid_param;

        // The height of `pos` is not used for the answer, but it is still
        // subtracted from on the way, so it must be in bounds as well.
        if (!common.inBounds(pos)) return invalid_param;

        // A tile has no detail surface for an off-mesh connection and
        // refuses to give its height, so the segment is read here.
        if (poly.getType() == .off_mesh_connection) {
            const v0 = tile.verts[poly.verts[0]];
            const v1 = tile.verts[poly.verts[1]];
            const t = common.distancePointSegmentSqr2d(pos, v0, v1).t;
            if (height) |result| {
                // A rise within 2^30 times a numerator within 2^61.
                const rise: i128 = @as(i64, v1[1]) - v0[1];
                result.* = @intCast(v0[1] + common.divRound(rise * t.num, t.den));
            }
            return .success;
        }

        const surface_height = tile.getPolyHeight(poly, pos) orelse return invalid_param;
        if (height) |result| result.* = surface_height;
        return .success;
    }

    /// The polygon nearest to `center` among those `filter` passes whose
    /// bounds touch the box of `half_extents` around it.
    ///
    /// `nearest_ref` is zero when the box touches no such polygon, which is
    /// still a success; `nearest_pt` and `is_over_poly` are then left alone.
    /// A polygon under or over `center` counts by its height difference
    /// less the tile's climb height, so a floor just below wins over a
    /// ledge beside. Of polygons as near as each other the first found
    /// stays.
    pub fn findNearestPoly(
        self: *const NavMeshQuery,
        center: [3]i32,
        half_extents: [3]i32,
        filter: *const QueryFilter,
        nearest_ref: *PolyRef,
        nearest_pt: ?*[3]i32,
        is_over_poly: ?*bool,
    ) Status {
        var nearest: NearestPolyQuery = .{ .query = self, .center = center };
        const status = self.queryPolygonsWith(center, half_extents, filter, nearest.polyQuery());
        if (status.failed) return status;

        nearest_ref.* = nearest.nearest_ref;
        // Without a polygon there is no nearest point to report.
        if (nearest_pt) |pt| {
            if (nearest.nearest_ref != 0) {
                pt.* = nearest.nearest_point;
                if (is_over_poly) |over| over.* = nearest.over_poly;
            }
        }

        return .success;
    }

    /// Hands `query` the polygons of `tile` that pass `filter` and whose
    /// bounds touch the box `query_min`..`query_max`, in batches.
    fn queryPolygonsInTile(
        self: *const NavMeshQuery,
        tile: *const MeshTile,
        query_min: [3]i32,
        query_max: [3]i32,
        filter: *const QueryFilter,
        query: PolyQuery,
    ) void {
        const nav = self.nav.?;

        const batch_size = 32;
        var poly_refs: [batch_size]PolyRef = undefined;
        var polys: [batch_size]*const Poly = undefined;
        var n: usize = 0;

        const base = nav.getPolyRefBase(tile);

        if (tile.bv_tree.len != 0) {
            const header = tile.header.?;
            const tile_min = header.bounds_min;
            const tile_max = header.bounds_max;
            const cell_size = header.bv_cell_size;

            // The query box clamped to the tile's, relative to its corner,
            // in the tree's cells; then widened by a cell at the far end
            // and to even and odd, so that rounding never loses a polygon.
            // A cell past the tree's last is held at the last.
            const bounds_min: [3]u16 = .{
                treeCell(query_min[0], tile_min[0], tile_max[0], cell_size) & 0xfffe,
                treeCell(query_min[1], tile_min[1], tile_max[1], cell_size) & 0xfffe,
                treeCell(query_min[2], tile_min[2], tile_max[2], cell_size) & 0xfffe,
            };
            const bounds_max: [3]u16 = .{
                (treeCell(query_max[0], tile_min[0], tile_max[0], cell_size) +| 1) | 1,
                (treeCell(query_max[1], tile_min[1], tile_max[1], cell_size) +| 1) | 1,
                (treeCell(query_max[2], tile_min[2], tile_max[2], cell_size) +| 1) | 1,
            };

            var node_index: usize = 0;
            while (node_index < tile.bv_tree.len) {
                const bv_node = &tile.bv_tree[node_index];
                const overlap = common.overlapQuantBounds(
                    bounds_min,
                    bounds_max,
                    bv_node.bounds_min,
                    bv_node.bounds_max,
                );
                const is_leaf_node = bv_node.index >= 0;

                if (is_leaf_node and overlap) {
                    const poly_index: usize = @intCast(bv_node.index);
                    const ref = base | @as(PolyRef, @intCast(poly_index));
                    const poly = &tile.polys[poly_index];
                    if (filter.passFilter(ref, tile, poly)) {
                        poly_refs[n] = ref;
                        polys[n] = poly;

                        if (n == batch_size - 1) {
                            query.process(tile, &polys, &poly_refs);
                            n = 0;
                        } else {
                            n += 1;
                        }
                    }
                }

                if (overlap or is_leaf_node) {
                    node_index += 1;
                } else {
                    // A missed inner node skips its whole subtree.
                    const escape_index: usize = @intCast(-bv_node.index);
                    node_index += escape_index;
                }
            }
        } else {
            for (tile.polys, 0..) |*poly, poly_index| {
                // Off-mesh connections are found through their links, never
                // by position.
                if (poly.getType() == .off_mesh_connection) continue;
                const ref = base | @as(PolyRef, @intCast(poly_index));
                if (!filter.passFilter(ref, tile, poly)) continue;

                var vert = tile.verts[poly.verts[0]];
                var bounds_min = vert;
                var bounds_max = vert;
                var j: usize = 1;
                while (j < poly.vert_count) : (j += 1) {
                    vert = tile.verts[poly.verts[j]];
                    bounds_min = common.vmin(bounds_min, vert);
                    bounds_max = common.vmax(bounds_max, vert);
                }
                if (common.overlapBounds(query_min, query_max, bounds_min, bounds_max)) {
                    poly_refs[n] = ref;
                    polys[n] = poly;

                    if (n == batch_size - 1) {
                        query.process(tile, &polys, &poly_refs);
                        n = 0;
                    } else {
                        n += 1;
                    }
                }
            }
        }

        // What did not fill a batch.
        if (n > 0) query.process(tile, polys[0..n], poly_refs[0..n]);
    }

    /// The polygons `filter` passes whose bounds touch the box of
    /// `half_extents` around `center`, as many as `polys` holds; the status
    /// carries `buffer_too_small` when there were more. `poly_count` is not
    /// written on failure.
    pub fn queryPolygons(
        self: *const NavMeshQuery,
        center: [3]i32,
        half_extents: [3]i32,
        filter: *const QueryFilter,
        polys: []PolyRef,
        poly_count: *usize,
    ) Status {
        var collector: CollectPolysQuery = .{ .polys = polys };
        const status = self.queryPolygonsWith(center, half_extents, filter, collector.polyQuery());
        if (status.failed) return status;

        poly_count.* = collector.collected_count;
        if (collector.overflow) return .{ .succeeded = true, .buffer_too_small = true };
        return .success;
    }

    /// Hands `query` every polygon `filter` passes whose bounds touch the
    /// box of `half_extents` around `center`, tile by tile, in batches of at
    /// most 32. The test is on bounding boxes, so a polygon may be handed
    /// over that the box itself misses.
    pub fn queryPolygonsWith(
        self: *const NavMeshQuery,
        center: [3]i32,
        half_extents: [3]i32,
        filter: *const QueryFilter,
        query: PolyQuery,
    ) Status {
        const nav = self.nav.?;

        if (!common.inBounds(center) or !common.inBounds(half_extents)) return invalid_param;

        // Both in bounds, so the box's corners lie within twice the bound.
        const bounds_min = common.vsub(center, half_extents);
        const bounds_max = common.vadd(center, half_extents);

        const cell_min = nav.calcTileLoc(bounds_min);
        const cell_max = nav.calcTileLoc(bounds_max);

        // One grid cell holds a tile per layer; more layers than this are
        // not looked at.
        const neighbours_max = 32;
        var neighbours: [neighbours_max]*const MeshTile = undefined;

        var y = cell_min.y;
        while (y <= cell_max.y) : (y += 1) {
            var x = cell_min.x;
            while (x <= cell_max.x) : (x += 1) {
                const neighbour_count = nav.getTilesAt(x, y, &neighbours);
                for (neighbours[0..neighbour_count]) |tile| {
                    self.queryPolygonsInTile(tile, bounds_min, bounds_max, filter, query);
                }
            }
        }

        return .success;
    }

    /// The polygons from `start_ref` to `end_ref` that cost the least to
    /// cross by `filter`, start first, as many as `path` holds.
    ///
    /// When the end cannot be reached the path leads to the polygon found
    /// nearest to it and the status carries `partial_result`. It carries
    /// `out_of_nodes` when the search ran out of nodes, which can make the
    /// path longer than need be or partial, and `buffer_too_small` when
    /// `path` holds only the first polygons of the path. The positions only
    /// steer the search; they are not checked against their polygons.
    pub fn findPath(
        self: *NavMeshQuery,
        start_ref: PolyRef,
        end_ref: PolyRef,
        start_pos: [3]i32,
        end_pos: [3]i32,
        filter: *const QueryFilter,
        path: []PolyRef,
        path_count: *usize,
    ) Status {
        const nav = self.nav.?;

        path_count.* = 0;

        if (!nav.isValidPolyRef(start_ref) or
            !nav.isValidPolyRef(end_ref) or
            !common.inBounds(start_pos) or
            !common.inBounds(end_pos) or
            path.len == 0)
        {
            return invalid_param;
        }

        if (start_ref == end_ref) {
            path[0] = start_ref;
            path_count.* = 1;
            return .success;
        }

        self.node_pool.clear();
        self.open_list.clear();

        const start_node = self.node_pool.getNode(start_ref, 0).?;
        start_node.pos = start_pos;
        start_node.bits.parent_index = 0;
        start_node.cost = 0;
        start_node.total = common.vdist(start_pos, end_pos) * h_scale / 1000;
        start_node.id = start_ref;
        start_node.bits.flags = .{ .open = true };
        self.open_list.push(start_node);

        var last_best_node = start_node;
        var last_best_node_cost = start_node.total;

        var out_of_nodes = false;

        while (!self.open_list.isEmpty()) {
            const best_node = self.open_list.pop();
            best_node.bits.flags.open = false;
            best_node.bits.flags.closed = true;

            if (best_node.id == end_ref) {
                last_best_node = best_node;
                break;
            }

            // Every reference in the pool came from a checked start or from
            // a link of the mesh, so the lookups skip the checks.
            const best_ref = best_node.id;
            const best = nav.getTileAndPolyByRefUnsafe(best_ref);
            const best_tile = best.tile;
            const best_poly = best.poly;

            var parent_ref: PolyRef = 0;
            var parent_tile: ?*const MeshTile = null;
            var parent_poly: ?*const Poly = null;
            if (best_node.bits.parent_index != 0) {
                parent_ref = self.node_pool.getNodeAtIdx(best_node.bits.parent_index).?.id;
            }
            if (parent_ref != 0) {
                const parent = nav.getTileAndPolyByRefUnsafe(parent_ref);
                parent_tile = parent.tile;
                parent_poly = parent.poly;
            }

            var link_index = best_poly.first_link;
            while (link_index != nav_mesh.null_link) {
                const link = best_tile.links[link_index];
                link_index = link.next;

                const neighbour_ref = link.ref;
                // Never walk straight back to the parent.
                if (neighbour_ref == 0 or neighbour_ref == parent_ref) continue;

                const neighbour = nav.getTileAndPolyByRefUnsafe(neighbour_ref);
                const neighbour_tile = neighbour.tile;
                const neighbour_poly = neighbour.poly;

                if (!filter.passFilter(neighbour_ref, neighbour_tile, neighbour_poly)) continue;

                // A polygon entered across a tile border gets a node per
                // direction of the border, since each stands on another
                // edge of it.
                const cross_side: u8 = if (link.side != 0xff) link.side >> 1 else 0;

                const neighbour_node = self.node_pool.getNode(neighbour_ref, cross_side) orelse {
                    out_of_nodes = true;
                    continue;
                };

                // A new node stands on the middle of the edge it was entered
                // by. A link that names no shared edge leaves the node where
                // it last stood.
                // The position as a value: read back from the node just after
                // its parts were stored, it would wait for the stores.
                var neighbour_pos = neighbour_node.pos;
                if (isUnvisited(neighbour_node)) {
                    _ = self.getEdgeMidPointResolved(
                        best_ref,
                        best_poly,
                        best_tile,
                        neighbour_ref,
                        neighbour_poly,
                        neighbour_tile,
                        &neighbour_pos,
                    );
                    neighbour_node.pos = neighbour_pos;
                }

                // Costs add up to the largest `u64` and stay there.
                const cur_cost = filter.getCost(
                    best_node.pos,
                    neighbour_pos,
                    parent_ref,
                    parent_tile,
                    parent_poly,
                    best_ref,
                    best_tile,
                    best_poly,
                    neighbour_ref,
                    neighbour_tile,
                    neighbour_poly,
                );
                var cost = best_node.cost +| cur_cost;
                var heuristic: u64 = 0;
                if (neighbour_ref == end_ref) {
                    // The goal also costs the walk from its edge to the end.
                    const end_cost = filter.getCost(
                        neighbour_pos,
                        end_pos,
                        best_ref,
                        best_tile,
                        best_poly,
                        neighbour_ref,
                        neighbour_tile,
                        neighbour_poly,
                        0,
                        null,
                        null,
                    );
                    cost = cost +| end_cost;
                } else {
                    heuristic = common.vdist(neighbour_pos, end_pos) * h_scale / 1000;
                }

                const total = cost +| heuristic;

                // Already queued, or already expanded, by a way that is no
                // more costly.
                if (neighbour_node.bits.flags.open and total >= neighbour_node.total) continue;
                if (neighbour_node.bits.flags.closed and total >= neighbour_node.total) continue;

                neighbour_node.bits.parent_index = @intCast(self.node_pool.getNodeIdx(best_node));
                neighbour_node.id = neighbour_ref;
                neighbour_node.bits.flags.closed = false;
                neighbour_node.cost = cost;
                neighbour_node.total = total;

                if (neighbour_node.bits.flags.open) {
                    self.open_list.modify(neighbour_node);
                } else {
                    neighbour_node.bits.flags.open = true;
                    self.open_list.push(neighbour_node);
                }

                // The node nearest to the goal ends a partial path.
                if (heuristic < last_best_node_cost) {
                    last_best_node_cost = heuristic;
                    last_best_node = neighbour_node;
                }
            }
        }

        var status = self.getPathToNode(last_best_node, path, path_count);

        if (last_best_node.id != end_ref) status.partial_result = true;
        if (out_of_nodes) status.out_of_nodes = true;

        return status;
    }

    /// Writes the polygons from the search's start to `end_node`, start
    /// first, as many as `path` holds.
    fn getPathToNode(
        self: *const NavMeshQuery,
        end_node: *Node,
        path: []PolyRef,
        path_count: *usize,
    ) Status {
        var cur_node: ?*Node = end_node;
        var length: usize = 0;
        while (cur_node) |cur| {
            length += 1;
            cur_node = self.node_pool.getNodeAtIdx(cur.bits.parent_index);
        }

        // A path longer than the buffer loses its end, not its start.
        cur_node = end_node;
        var write_count = length;
        while (write_count > path.len) : (write_count -= 1) {
            cur_node = self.node_pool.getNodeAtIdx(cur_node.?.bits.parent_index);
        }

        var i = write_count;
        while (i > 0) {
            i -= 1;
            const cur = cur_node.?;
            path[i] = cur.id;
            cur_node = self.node_pool.getNodeAtIdx(cur.bits.parent_index);
        }

        std.debug.assert(cur_node == null);

        path_count.* = @min(length, path.len);

        if (length > path.len) return .{ .succeeded = true, .buffer_too_small = true };
        return .success;
    }

    /// Starts a path search that `updateSlicedFindPath` advances a few
    /// polygons at a time and `finalizeSlicedFindPath` or
    /// `finalizeSlicedFindPathPartial` turns into a path.
    ///
    /// `filter` is kept by pointer and must stay valid until the search is
    /// finalized. Any other search on this query in between overwrites the
    /// nodes this one stands on. With `options.any_angle` the search tries a
    /// ray cast from each node's parent to shortcut around corners, which
    /// costs time and gives straighter paths.
    pub fn initSlicedFindPath(
        self: *NavMeshQuery,
        start_ref: PolyRef,
        end_ref: PolyRef,
        start_pos: [3]i32,
        end_pos: [3]i32,
        filter: *const QueryFilter,
        options: FindPathOptions,
    ) Status {
        const nav = self.nav.?;

        self.query = .{
            .status = .failure,
            .last_best_node = null,
            .last_best_node_cost = 0,
            .start_ref = start_ref,
            .end_ref = end_ref,
            .start_pos = start_pos,
            .end_pos = end_pos,
            .filter = filter,
            .options = options,
            .raycast_limit_sqr = std.math.maxInt(u64),
        };

        if (!nav.isValidPolyRef(start_ref) or
            !nav.isValidPolyRef(end_ref) or
            !common.inBounds(start_pos) or
            !common.inBounds(end_pos))
        {
            return invalid_param;
        }

        if (options.any_angle) {
            // Rays are kept to a few agent radii: long ones cost much and
            // gain little. The start tile's radius serves for the whole
            // search.
            const tile = nav.getTileByRef(start_ref).?;
            const agent_radius: u64 = @abs(tile.header.?.walkable_radius);
            // The square can pass 64 bits and is then held at the largest.
            const reach = agent_radius *| @as(u64, nav_mesh.ray_cast_limit_proportions);
            self.query.raycast_limit_sqr = reach *| reach;
        }

        if (start_ref == end_ref) {
            self.query.status = .success;
            return .success;
        }

        self.node_pool.clear();
        self.open_list.clear();

        const start_node = self.node_pool.getNode(start_ref, 0).?;
        start_node.pos = start_pos;
        start_node.bits.parent_index = 0;
        start_node.cost = 0;
        start_node.total = common.vdist(start_pos, end_pos) * h_scale / 1000;
        start_node.id = start_ref;
        start_node.bits.flags = .{ .open = true };
        self.open_list.push(start_node);

        self.query.status = .{ .in_progress = true };
        self.query.last_best_node = start_node;
        self.query.last_best_node_cost = start_node.total;

        return self.query.status;
    }

    /// Advances the sliced path search by at most `iteration_count_max`
    /// polygons; `done_iterations` gets how many it took.
    ///
    /// Returns the search's status: `in_progress` while there is more to
    /// do, success once the goal is reached or nothing is left to try, and
    /// failure when the start, the end or a polygon on the way has left the
    /// mesh. A search that is not in progress is left alone.
    pub fn updateSlicedFindPath(
        self: *NavMeshQuery,
        iteration_count_max: i32,
        done_iterations: ?*i32,
    ) Status {
        if (!self.query.status.in_progress) return self.query.status;

        const nav = self.nav.?;
        const filter = self.query.filter.?;

        // Tiles may have been removed since the last slice.
        if (!nav.isValidPolyRef(self.query.start_ref) or
            !nav.isValidPolyRef(self.query.end_ref))
        {
            self.query.status = .failure;
            return .failure;
        }

        // The shortcut rays report how far they got and what the way cost,
        // never the polygons they crossed.
        var ray_hit: RaycastHit = .{
            .t = .zero,
            .hit_normal = .{ 0, 0, 0 },
            .hit_edge_index = 0,
            .path = &.{},
            .path_count = 0,
            .path_cost = 0,
        };

        var iter: i32 = 0;
        while (iter < iteration_count_max and !self.open_list.isEmpty()) {
            iter += 1;

            const best_node = self.open_list.pop();
            best_node.bits.flags.open = false;
            best_node.bits.flags.closed = true;

            if (best_node.id == self.query.end_ref) {
                self.query.last_best_node = best_node;
                self.query.status = Status.success.with(self.query.status.detail());
                if (done_iterations) |done| done.* = iter;
                return self.query.status;
            }

            // Unlike a search in one call, a slice cannot trust the nodes
            // of earlier slices: their polygons may be gone.
            const best_ref = best_node.id;
            var best_tile: *const MeshTile = undefined;
            var best_poly: *const Poly = undefined;
            if (nav.getTileAndPolyByRef(best_ref, &best_tile, &best_poly).failed) {
                self.query.status = .failure;
                if (done_iterations) |done| done.* = iter;
                return self.query.status;
            }

            var parent_ref: PolyRef = 0;
            var grandpa_ref: PolyRef = 0;
            var parent_tile: ?*const MeshTile = null;
            var parent_poly: ?*const Poly = null;
            var parent_node: ?*Node = null;
            if (best_node.bits.parent_index != 0) {
                const parent = self.node_pool.getNodeAtIdx(best_node.bits.parent_index).?;
                parent_node = parent;
                parent_ref = parent.id;
                if (parent.bits.parent_index != 0) {
                    grandpa_ref = self.node_pool.getNodeAtIdx(parent.bits.parent_index).?.id;
                }
            }
            if (parent_ref != 0) {
                var tile: *const MeshTile = undefined;
                var poly: *const Poly = undefined;
                const invalid_parent = nav.getTileAndPolyByRef(parent_ref, &tile, &poly).failed;
                if (invalid_parent or (grandpa_ref != 0 and !nav.isValidPolyRef(grandpa_ref))) {
                    self.query.status = .failure;
                    if (done_iterations) |done| done.* = iter;
                    return self.query.status;
                }
                parent_tile = tile;
                parent_poly = poly;
            }

            // Whether to try a straight line from the parent past this node.
            var try_line_of_sight = false;
            if (self.query.options.any_angle) {
                if (parent_ref != 0 and
                    common.vdistSqr(parent_node.?.pos, best_node.pos) <
                        self.query.raycast_limit_sqr)
                {
                    try_line_of_sight = true;
                }
            }

            var link_index = best_poly.first_link;
            while (link_index != nav_mesh.null_link) {
                const link = best_tile.links[link_index];
                link_index = link.next;

                const neighbour_ref = link.ref;
                // Never walk straight back to the parent.
                if (neighbour_ref == 0 or neighbour_ref == parent_ref) continue;

                // The reference comes from a link of a tile just checked.
                const neighbour = nav.getTileAndPolyByRefUnsafe(neighbour_ref);
                const neighbour_tile = neighbour.tile;
                const neighbour_poly = neighbour.poly;

                if (!filter.passFilter(neighbour_ref, neighbour_tile, neighbour_poly)) continue;

                const neighbour_node = self.node_pool.getNode(neighbour_ref, 0) orelse {
                    self.query.status.out_of_nodes = true;
                    continue;
                };

                // Reached from the same parent before: a shortcut already
                // joins the two.
                if (neighbour_node.bits.parent_index != 0 and
                    neighbour_node.bits.parent_index == best_node.bits.parent_index)
                {
                    continue;
                }

                // A new node stands on the middle of the edge it was entered
                // by. A link that names no shared edge leaves the node where
                // it last stood.
                // The position as a value: read back from the node just after
                // its parts were stored, it would wait for the stores.
                var neighbour_pos = neighbour_node.pos;
                if (isUnvisited(neighbour_node)) {
                    _ = self.getEdgeMidPointResolved(
                        best_ref,
                        best_poly,
                        best_tile,
                        neighbour_ref,
                        neighbour_poly,
                        neighbour_tile,
                        &neighbour_pos,
                    );
                    neighbour_node.pos = neighbour_pos;
                }

                var found_short_cut = false;
                ray_hit.path_cost = 0;
                ray_hit.t = .zero;
                if (try_line_of_sight) {
                    // A ray that fails leaves `t` at zero: no shortcut.
                    _ = self.raycast(
                        parent_ref,
                        parent_node.?.pos,
                        neighbour_pos,
                        filter,
                        .{ .use_costs = true },
                        &ray_hit,
                        grandpa_ref,
                    );
                    // The ray got to its end: `t` is one or more, or never.
                    found_short_cut = ray_hit.t.atLeastOne();
                }

                // Costs add up to the largest `u64` and stay there.
                var cost: u64 = undefined;
                if (found_short_cut) {
                    // The straight way from the parent, at the ray's cost.
                    cost = parent_node.?.cost +| ray_hit.path_cost;
                } else {
                    const cur_cost = filter.getCost(
                        best_node.pos,
                        neighbour_pos,
                        parent_ref,
                        parent_tile,
                        parent_poly,
                        best_ref,
                        best_tile,
                        best_poly,
                        neighbour_ref,
                        neighbour_tile,
                        neighbour_poly,
                    );
                    cost = best_node.cost +| cur_cost;
                }

                var heuristic: u64 = 0;
                if (neighbour_ref == self.query.end_ref) {
                    // The goal also costs the walk from its edge to the end.
                    const end_cost = filter.getCost(
                        neighbour_pos,
                        self.query.end_pos,
                        best_ref,
                        best_tile,
                        best_poly,
                        neighbour_ref,
                        neighbour_tile,
                        neighbour_poly,
                        0,
                        null,
                        null,
                    );
                    cost = cost +| end_cost;
                } else {
                    heuristic =
                        common.vdist(neighbour_pos, self.query.end_pos) * h_scale / 1000;
                }

                const total = cost +| heuristic;

                // Already queued, or already expanded, by a way that is no
                // more costly.
                if (neighbour_node.bits.flags.open and total >= neighbour_node.total) continue;
                if (neighbour_node.bits.flags.closed and total >= neighbour_node.total) continue;

                neighbour_node.bits.parent_index = if (found_short_cut)
                    best_node.bits.parent_index
                else
                    @intCast(self.node_pool.getNodeIdx(best_node));
                neighbour_node.id = neighbour_ref;
                neighbour_node.bits.flags.closed = false;
                neighbour_node.bits.flags.parent_detached = found_short_cut;
                neighbour_node.cost = cost;
                neighbour_node.total = total;

                if (neighbour_node.bits.flags.open) {
                    self.open_list.modify(neighbour_node);
                } else {
                    neighbour_node.bits.flags.open = true;
                    self.open_list.push(neighbour_node);
                }

                // The node nearest to the goal ends a partial path.
                if (heuristic < self.query.last_best_node_cost) {
                    self.query.last_best_node_cost = heuristic;
                    self.query.last_best_node = neighbour_node;
                }
            }
        }

        // Nothing left to try: the search is over without the goal, and
        // finalizing gives the partial path.
        if (self.open_list.isEmpty()) {
            self.query.status = Status.success.with(self.query.status.detail());
        }

        if (done_iterations) |done| done.* = iter;

        return self.query.status;
    }

    /// Writes the polygons from the sliced search's start to `end_node`,
    /// start first, and returns how many. A step a ray cast found is walked
    /// again to fill in the polygons it crossed. What cuts the path short
    /// joins the search's status.
    ///
    /// The nodes' parent indices are turned around on the way, so the pool
    /// serves no second finalize.
    fn storeSlicedPath(self: *NavMeshQuery, end_node: *Node, path: []PolyRef) usize {
        const filter = self.query.filter.?;

        // Turn the chain around to run from the start. A node's flag told
        // that the step from its parent was a ray; it now tells the same of
        // the step to the node after it.
        var prev: ?*Node = null;
        var cur_node: ?*Node = end_node;
        var prev_ray = false;
        while (cur_node) |cur| {
            const next = self.node_pool.getNodeAtIdx(cur.bits.parent_index);
            cur.bits.parent_index = @intCast(self.node_pool.getNodeIdx(prev));
            prev = cur;
            const next_ray = cur.bits.flags.parent_detached;
            cur.bits.flags.parent_detached = prev_ray;
            prev_ray = next_ray;
            cur_node = next;
        }

        var count: usize = 0;
        cur_node = prev;
        while (cur_node) |cur| {
            const next = self.node_pool.getNodeAtIdx(cur.bits.parent_index);
            var status: Status = .{};
            if (cur.bits.flags.parent_detached) {
                var t: Fraction = undefined;
                var normal: [3]i32 = undefined;
                var ray_count: usize = 0;
                status = self.raycastPath(
                    cur.id,
                    cur.pos,
                    next.?.pos,
                    filter,
                    &t,
                    &normal,
                    path[count..],
                    &ray_count,
                );
                count += ray_count;
                // The ray ends on the next node's polygon, which that node
                // stores itself.
                if (count > 0 and path[count - 1] == next.?.id) count -= 1;
            } else {
                if (count < path.len) {
                    path[count] = cur.id;
                    count += 1;
                }
                if (count >= path.len) status = .{ .buffer_too_small = true };
            }

            if (@as(u32, @bitCast(status.detail())) != 0) {
                self.query.status = self.query.status.with(status.detail());
                break;
            }
            cur_node = next;
        }

        return count;
    }

    /// Ends the sliced path search and writes its path: the polygons from
    /// the start to the end, or to the polygon found nearest to the end with
    /// `partial_result` in the status, start first, as many as `path` holds.
    ///
    /// The status carries `buffer_too_small` as soon as `path` is filled,
    /// also when the path just fits. The search is over afterwards, whatever
    /// the outcome, except when `path` is empty.
    pub fn finalizeSlicedFindPath(
        self: *NavMeshQuery,
        path: []PolyRef,
        path_count: *usize,
    ) Status {
        path_count.* = 0;

        if (path.len == 0) return invalid_param;

        if (self.query.status.failed) {
            self.query = .idle;
            return .failure;
        }

        var count: usize = 0;
        if (self.query.start_ref == self.query.end_ref) {
            path[count] = self.query.start_ref;
            count += 1;
        } else {
            const last_best_node = self.query.last_best_node.?;
            if (last_best_node.id != self.query.end_ref) {
                self.query.status.partial_result = true;
            }
            count = self.storeSlicedPath(last_best_node, path);
        }

        const details = self.query.status.detail();

        self.query = .idle;

        path_count.* = count;
        return Status.success.with(details);
    }

    /// Ends the sliced path search and writes the path to the polygon of
    /// `existing`, taken from its end, that the search has reached; when it
    /// reached none, the path `finalizeSlicedFindPath` would give for an
    /// unreached goal, with `partial_result`.
    ///
    /// It serves an agent that follows `existing` while a new search runs:
    /// the result joins the old path as far along as the search got. The
    /// search is over afterwards, as with `finalizeSlicedFindPath`.
    pub fn finalizeSlicedFindPathPartial(
        self: *NavMeshQuery,
        existing: []const PolyRef,
        path: []PolyRef,
        path_count: *usize,
    ) Status {
        path_count.* = 0;

        if (existing.len == 0 or path.len == 0) return invalid_param;

        if (self.query.status.failed) {
            self.query = .idle;
            return .failure;
        }

        var count: usize = 0;
        if (self.query.start_ref == self.query.end_ref) {
            path[count] = self.query.start_ref;
            count += 1;
        } else {
            var visited: ?*Node = null;
            var i = existing.len;
            while (i > 0) {
                i -= 1;
                var found: [1]*Node = undefined;
                if (self.node_pool.findNodes(existing[i], &found) != 0) {
                    visited = found[0];
                    break;
                }
            }

            const end_node = visited orelse unreached: {
                self.query.status.partial_result = true;
                break :unreached self.query.last_best_node.?;
            };
            count = self.storeSlicedPath(end_node, path);
        }

        const details = self.query.status.detail();

        self.query = .idle;

        path_count.* = count;
        return Status.success.with(details);
    }

    /// Whether `ref` names a polygon of the mesh that `filter` passes.
    pub fn isValidPolyRef(
        self: *const NavMeshQuery,
        ref: PolyRef,
        filter: *const QueryFilter,
    ) bool {
        var tile: *const MeshTile = undefined;
        var poly: *const Poly = undefined;
        if (self.nav.?.getTileAndPolyByRef(ref, &tile, &poly).failed) return false;
        return filter.passFilter(ref, tile, poly);
    }

    /// Whether the last search on the main node pool expanded polygon
    /// `ref`. False before `init`.
    pub fn isInClosedList(self: *const NavMeshQuery, ref: PolyRef) bool {
        if (self.node_pool.node_count_max == 0) return false;

        var nodes: [node.node_states_max]*Node = undefined;
        const found_count = self.node_pool.findNodes(ref, &nodes);
        for (nodes[0..found_count]) |found| {
            if (found.bits.flags.closed) return true;
        }
        return false;
    }

    /// The node pool of the path searches and the searches around a point,
    /// for reading what the last one explored.
    ///
    /// The pointer is into the query and valid until `deinit`. Every search
    /// on that pool clears it, and an `init` that grows it frees its nodes.
    pub fn getNodePool(self: *NavMeshQuery) *NodePool {
        return &self.node_pool;
    }

    /// The mesh `init` attached. The query must be initialised.
    pub fn getAttachedNavMesh(self: *const NavMeshQuery) *const NavMesh {
        return self.nav.?;
    }

    /// Adds a point to a straight path, or, when it equals the last point, only
    /// replaces that point's flags and polygon. In progress while the path can
    /// take more points.
    fn appendVertex(
        _: *const NavMeshQuery,
        pos: [3]i32,
        flags: StraightPathFlags,
        ref: PolyRef,
        straight_path: [][3]i32,
        straight_path_flags: ?[]StraightPathFlags,
        straight_path_refs: ?[]PolyRef,
        straight_path_count: *usize,
    ) Status {
        const count = straight_path_count.*;
        if (count > 0 and common.vequal(straight_path[count - 1], pos)) {
            // The points are equal: update the last one's flags and polygon.
            if (straight_path_flags) |out| out[count - 1] = flags;
            if (straight_path_refs) |out| out[count - 1] = ref;
        } else {
            // A full buffer takes no point. findStraightPath gets here when a
            // broken corridor ends the path just as the buffer fills.
            if (count >= straight_path.len) {
                return Status.success.with(.{ .buffer_too_small = true });
            }

            straight_path[count] = pos;
            if (straight_path_flags) |out| out[count] = flags;
            if (straight_path_refs) |out| out[count] = ref;
            straight_path_count.* = count + 1;

            // No space is left for a point after this one.
            if (count + 1 >= straight_path.len) {
                return Status.success.with(.{ .buffer_too_small = true });
            }

            // The end of the path is reached.
            if (flags == StraightPathFlags{ .end = true }) return .success;
        }
        return in_progress;
    }

    /// Adds a point where the segment from the straight path's last point to
    /// `end_pos` crosses each portal between `path[start_index]` and
    /// `path[end_index]`, held to the portal's ends and rounded to the nearest
    /// millimetre. In progress while the path can take more points.
    fn appendPortals(
        self: *const NavMeshQuery,
        start_index: usize,
        end_index: usize,
        end_pos: [3]i32,
        path: []const PolyRef,
        straight_path: [][3]i32,
        straight_path_flags: ?[]StraightPathFlags,
        straight_path_refs: ?[]PolyRef,
        straight_path_count: *usize,
        options: StraightPathOptions,
    ) Status {
        const nav = self.nav.?;

        // The points added below leave this one where it is.
        const start_pos = straight_path[straight_path_count.* - 1];

        var i = start_index;
        while (i < end_index) : (i += 1) {
            // Calculate the portal.
            const from = path[i];
            var from_tile: *const MeshTile = undefined;
            var from_poly: *const Poly = undefined;
            if (nav.getTileAndPolyByRef(from, &from_tile, &from_poly).failed) {
                return invalid_param;
            }

            const to = path[i + 1];
            var to_tile: *const MeshTile = undefined;
            var to_poly: *const Poly = undefined;
            if (nav.getTileAndPolyByRef(to, &to_tile, &to_poly).failed) return invalid_param;

            var left: [3]i32 = undefined;
            var right: [3]i32 = undefined;
            if (self.getPortalPointsResolved(
                from,
                from_poly,
                from_tile,
                to,
                to_poly,
                to_tile,
                &left,
                &right,
            ).failed) break;

            // Skip the crossing when only area crossings are asked for.
            if (options.area_crossings and from_poly.getArea() == to_poly.getArea()) continue;

            // Append the crossing. The lines may cross beyond the portal's ends
            // when the path runs nearly along it: the point stays on the portal.
            if (common.intersectSegmentSegment2d(start_pos, end_pos, left, right)) |crossing| {
                const pt = common.lerp(left, right, crossing.t.clamped());
                const stat = self.appendVertex(
                    pt,
                    .{},
                    path[i + 1],
                    straight_path,
                    straight_path_flags,
                    straight_path_refs,
                    straight_path_count,
                );
                if (stat != in_progress) return stat;
            }
        }
        return in_progress;
    }

    /// Finds the straight path from `start_pos` to `end_pos` through the corridor
    /// `path`: the corners a taut string through the corridor bends around.
    ///
    /// The start is clamped to the first polygon of `path` and the end to the
    /// last. `straight_path` receives the points, in millimetres,
    /// `straight_path_flags` what each point is and `straight_path_refs` the
    /// polygon entered at each point, zero for the end point; the two optional
    /// buffers must be at least as long as `straight_path`. `straight_path_count`
    /// receives how many points were written. When the buffers are too small they
    /// are filled from the start as far as they go.
    ///
    /// Which side of the string a portal's end lies on is decided exactly, so
    /// three points on one line are seen as such.
    pub fn findStraightPath(
        self: *const NavMeshQuery,
        start_pos: [3]i32,
        end_pos: [3]i32,
        path: []const PolyRef,
        straight_path: [][3]i32,
        straight_path_flags: ?[]StraightPathFlags,
        straight_path_refs: ?[]PolyRef,
        straight_path_count: *usize,
        options: StraightPathOptions,
    ) Status {
        std.debug.assert(self.nav != null);

        straight_path_count.* = 0;

        if (!common.inBounds(start_pos) or
            !common.inBounds(end_pos) or
            path.len == 0 or
            path[0] == 0 or
            straight_path.len == 0)
        {
            return invalid_param;
        }

        var closest_start_pos: [3]i32 = undefined;
        if (self.closestPointOnPolyBoundary(path[0], start_pos, &closest_start_pos).failed) {
            return invalid_param;
        }

        var closest_end_pos: [3]i32 = undefined;
        if (self.closestPointOnPolyBoundary(path[path.len - 1], end_pos, &closest_end_pos).failed) {
            return invalid_param;
        }

        // Add the start point.
        var stat = self.appendVertex(
            closest_start_pos,
            .{ .start = true },
            path[0],
            straight_path,
            straight_path_flags,
            straight_path_refs,
            straight_path_count,
        );
        if (stat != in_progress) return stat;

        const crossings = options.area_crossings or options.all_crossings;

        if (path.len > 1) {
            var portal_apex = closest_start_pos;
            var portal_left = portal_apex;
            var portal_right = portal_apex;
            var apex_index: usize = 0;
            var left_index: usize = 0;
            var right_index: usize = 0;

            var left_poly_type: PolyType = .ground;
            var right_poly_type: PolyType = .ground;

            var left_poly_ref = path[0];
            var right_poly_ref = path[0];

            var i: usize = 0;
            while (i < path.len) : (i += 1) {
                var left: [3]i32 = undefined;
                var right: [3]i32 = undefined;
                var to_type: PolyType = undefined;

                if (i + 1 < path.len) {
                    var from_type: PolyType = undefined;

                    // The next portal.
                    if (self.getPortalPoints(
                        path[i],
                        path[i + 1],
                        &left,
                        &right,
                        &from_type,
                        &to_type,
                    ).failed) {
                        // No portal: in practice path[i + 1] is not a valid
                        // polygon. Clamp the end point to path[i] and return the
                        // path so far.
                        if (self.closestPointOnPolyBoundary(
                            path[i],
                            end_pos,
                            &closest_end_pos,
                        ).failed) {
                            // Only when the first polygon is invalid.
                            return invalid_param;
                        }

                        // Append the portals along the current segment. The
                        // statuses are not needed: the path ends here either way.
                        if (crossings) {
                            _ = self.appendPortals(
                                apex_index,
                                i,
                                closest_end_pos,
                                path,
                                straight_path,
                                straight_path_flags,
                                straight_path_refs,
                                straight_path_count,
                                options,
                            );
                        }

                        _ = self.appendVertex(
                            closest_end_pos,
                            .{},
                            path[i],
                            straight_path,
                            straight_path_flags,
                            straight_path_refs,
                            straight_path_count,
                        );

                        var result = Status.success.with(.{ .partial_result = true });
                        if (straight_path_count.* >= straight_path.len) {
                            result.buffer_too_small = true;
                        }
                        return result;
                    }

                    // Starting on the first portal, to within a millimetre:
                    // advance.
                    if (i == 0) {
                        const dist_sqr =
                            common.distancePointSegmentSqr2d(portal_apex, left, right).dist_sqr;
                        if (dist_sqr < 1) continue;
                    }
                } else {
                    // The end of the path.
                    left = closest_end_pos;
                    right = closest_end_pos;
                    to_type = .ground;
                }

                // The right vertex.
                if (common.triArea2d(portal_apex, portal_right, right) <= 0) {
                    if (common.vequal(portal_apex, portal_right) or
                        common.triArea2d(portal_apex, portal_left, right) > 0)
                    {
                        portal_right = right;
                        right_poly_ref = if (i + 1 < path.len) path[i + 1] else 0;
                        right_poly_type = to_type;
                        right_index = i;
                    } else {
                        // The right side crossed the left: the left corner is a
                        // point of the path. Append the portals up to it first.
                        if (crossings) {
                            stat = self.appendPortals(
                                apex_index,
                                left_index,
                                portal_left,
                                path,
                                straight_path,
                                straight_path_flags,
                                straight_path_refs,
                                straight_path_count,
                                options,
                            );
                            if (stat != in_progress) return stat;
                        }

                        portal_apex = portal_left;
                        apex_index = left_index;

                        var flags: StraightPathFlags = .{};
                        if (left_poly_ref == 0) {
                            flags = .{ .end = true };
                        } else if (left_poly_type == .off_mesh_connection) {
                            flags = .{ .off_mesh_connection = true };
                        }

                        // Append or update the vertex.
                        stat = self.appendVertex(
                            portal_apex,
                            flags,
                            left_poly_ref,
                            straight_path,
                            straight_path_flags,
                            straight_path_refs,
                            straight_path_count,
                        );
                        if (stat != in_progress) return stat;

                        portal_left = portal_apex;
                        portal_right = portal_apex;
                        left_index = apex_index;
                        right_index = apex_index;

                        // Restart from the polygon after the new apex.
                        i = apex_index;
                        continue;
                    }
                }

                // The left vertex.
                if (common.triArea2d(portal_apex, portal_left, left) >= 0) {
                    if (common.vequal(portal_apex, portal_left) or
                        common.triArea2d(portal_apex, portal_right, left) < 0)
                    {
                        portal_left = left;
                        left_poly_ref = if (i + 1 < path.len) path[i + 1] else 0;
                        left_poly_type = to_type;
                        left_index = i;
                    } else {
                        // The left side crossed the right: the right corner is a
                        // point of the path. Append the portals up to it first.
                        if (crossings) {
                            stat = self.appendPortals(
                                apex_index,
                                right_index,
                                portal_right,
                                path,
                                straight_path,
                                straight_path_flags,
                                straight_path_refs,
                                straight_path_count,
                                options,
                            );
                            if (stat != in_progress) return stat;
                        }

                        portal_apex = portal_right;
                        apex_index = right_index;

                        var flags: StraightPathFlags = .{};
                        if (right_poly_ref == 0) {
                            flags = .{ .end = true };
                        } else if (right_poly_type == .off_mesh_connection) {
                            flags = .{ .off_mesh_connection = true };
                        }

                        // Append or update the vertex.
                        stat = self.appendVertex(
                            portal_apex,
                            flags,
                            right_poly_ref,
                            straight_path,
                            straight_path_flags,
                            straight_path_refs,
                            straight_path_count,
                        );
                        if (stat != in_progress) return stat;

                        portal_left = portal_apex;
                        portal_right = portal_apex;
                        left_index = apex_index;
                        right_index = apex_index;

                        // Restart from the polygon after the new apex.
                        i = apex_index;
                        continue;
                    }
                }
            }

            // Append the portals along the last segment.
            if (crossings) {
                stat = self.appendPortals(
                    apex_index,
                    path.len - 1,
                    closest_end_pos,
                    path,
                    straight_path,
                    straight_path_flags,
                    straight_path_refs,
                    straight_path_count,
                    options,
                );
                if (stat != in_progress) return stat;
            }
        }

        // The status is not needed: the path ends here either way.
        _ = self.appendVertex(
            closest_end_pos,
            .{ .end = true },
            0,
            straight_path,
            straight_path_flags,
            straight_path_refs,
            straight_path_count,
        );

        var result: Status = .success;
        if (straight_path_count.* >= straight_path.len) result.buffer_too_small = true;
        return result;
    }

    /// Moves from `start_pos` toward `end_pos` along the surface, sliding along
    /// walls, and stops at the reachable point nearest the end.
    ///
    /// Meant for small moves over few polygons: over a long way the visited
    /// polygons no longer form a whole path. `result_pos` receives the end
    /// position when it is reached, otherwise the nearest reachable point; it is
    /// not put at the surface's height. A point on a wall is rounded to the
    /// nearest millimetre per axis, so it can lie that far outside its polygon.
    /// `visited` receives the polygons crossed, in order from the start, as many
    /// as it holds, and `visited_count` how many. The end position is treated as
    /// a point in the xz plane, as `raycast` does.
    pub fn moveAlongSurface(
        self: *NavMeshQuery,
        start_ref: PolyRef,
        start_pos: [3]i32,
        end_pos: [3]i32,
        filter: *const QueryFilter,
        result_pos: *[3]i32,
        visited: []PolyRef,
        visited_count: *usize,
    ) Status {
        const nav = self.nav.?;
        const pool = &self.tiny_node_pool;

        visited_count.* = 0;

        if (!nav.isValidPolyRef(start_ref) or
            !common.inBounds(start_pos) or
            !common.inBounds(end_pos) or
            visited.len == 0)
        {
            return invalid_param;
        }

        var status: Status = .success;

        var stack: [search_stack_size]*Node = undefined;
        var stack_count: usize = 0;

        pool.clear();

        const start_node = pool.getNode(start_ref, 0).?;
        start_node.bits.parent_index = 0;
        start_node.cost = 0;
        start_node.total = 0;
        start_node.id = start_ref;
        start_node.bits.flags = .{ .closed = true };
        stack[stack_count] = start_node;
        stack_count += 1;

        var best_pos = start_pos;
        var best_dist: u64 = std.math.maxInt(u64);
        var best_node: ?*Node = null;

        // The search stays near the segment from start to end: a circle around
        // its rounded middle, with half its length rounded up and two
        // millimetres more for the rounding of the middle and of the root.
        const search_pos: [3]i32 = .{
            @intCast(common.divRound(@as(i64, start_pos[0]) + end_pos[0], 2)),
            @intCast(common.divRound(@as(i64, start_pos[1]) + end_pos[1], 2)),
            @intCast(common.divRound(@as(i64, start_pos[2]) + end_pos[2], 2)),
        };
        // The radius stays below 2^30, so its square fits.
        const search_dist: u64 = common.isqrt(common.vdistSqr(start_pos, end_pos));
        const search_radius = (search_dist + 2) / 2 + 2;
        const search_radius_sqr = search_radius * search_radius;

        var verts: [nav_mesh.verts_per_polygon][3]i32 = undefined;

        while (stack_count != 0) {
            // Pop the front.
            const cur_node = stack[0];
            for (0..stack_count - 1) |i| stack[i] = stack[i + 1];
            stack_count -= 1;

            // The start reference was checked, the others come from the mesh.
            const cur_ref = cur_node.id;
            const cur = nav.getTileAndPolyByRefUnsafe(cur_ref);
            const cur_tile = cur.tile;
            const cur_poly = cur.poly;

            // Collect the vertices.
            const vert_count: usize = cur_poly.vert_count;
            for (0..vert_count) |i| verts[i] = cur_tile.verts[cur_poly.verts[i]];

            // The target is inside the polygon: stop the search.
            if (common.pointInPolygon(end_pos, verts[0..vert_count])) {
                best_node = cur_node;
                best_pos = end_pos;
                break;
            }

            // Find the wall edges and the nearest point inside the walls.
            var j = vert_count -% 1;
            for (0..vert_count) |i| {
                defer j = i;

                // The neighbours across this edge.
                var neighbours: [edge_neighbour_count_max]PolyRef = undefined;
                var neighbour_count: usize = 0;

                if (cur_poly.neighbours[j] & nav_mesh.ext_link != 0) {
                    // A tile border.
                    var k = cur_poly.first_link;
                    while (k != nav_mesh.null_link) : (k = cur_tile.links[k].next) {
                        const link = &cur_tile.links[k];
                        if (link.edge != j) continue;
                        if (link.ref == 0) continue;
                        const neighbour = nav.getTileAndPolyByRefUnsafe(link.ref);
                        if (!filter.passFilter(link.ref, neighbour.tile, neighbour.poly)) continue;
                        if (neighbour_count < edge_neighbour_count_max) {
                            neighbours[neighbour_count] = link.ref;
                            neighbour_count += 1;
                        }
                    }
                } else if (cur_poly.neighbours[j] != 0) {
                    // An edge inside the tile.
                    const index: u32 = cur_poly.neighbours[j] - 1;
                    const ref = nav.getPolyRefBase(cur_tile) | @as(PolyRef, index);
                    if (filter.passFilter(ref, cur_tile, &cur_tile.polys[index])) {
                        neighbours[neighbour_count] = ref;
                        neighbour_count += 1;
                    }
                }

                const vj = verts[j];
                const vi = verts[i];

                if (neighbour_count == 0) {
                    // A wall edge: keep the point on it nearest the end.
                    const nearest = common.distancePointSegmentSqr2d(end_pos, vj, vi);
                    if (nearest.dist_sqr < best_dist) {
                        best_pos = common.lerp(vj, vi, nearest.t);
                        best_dist = nearest.dist_sqr;
                        best_node = cur_node;
                    }
                } else {
                    for (neighbours[0..neighbour_count]) |neighbour_ref| {
                        // Skip when no node can be had.
                        const neighbour_node = pool.getNode(neighbour_ref, 0) orelse continue;
                        // Skip when already visited.
                        if (neighbour_node.bits.flags.closed) continue;

                        // Skip the edge when it is too far from the search
                        // area. The whole edge stands in for the portal, which
                        // is much faster to test.
                        const dist_sqr =
                            common.distancePointSegmentSqr2d(search_pos, vj, vi).dist_sqr;
                        if (dist_sqr > search_radius_sqr) continue;

                        // Mark the node visited and queue it.
                        if (stack_count < search_stack_size) {
                            neighbour_node.bits.parent_index = @intCast(pool.getNodeIdx(cur_node));
                            neighbour_node.bits.flags.closed = true;
                            stack[stack_count] = neighbour_node;
                            stack_count += 1;
                        }
                    }
                }
            }
        }

        var n: usize = 0;
        if (best_node) |best| {
            // Reverse the chain of parents, so it runs from the start.
            var prev: ?*Node = null;
            var walk: ?*Node = best;
            while (walk) |current| {
                const next = pool.getNodeAtIdx(current.bits.parent_index);
                current.bits.parent_index = @intCast(pool.getNodeIdx(prev));
                prev = current;
                walk = next;
            }

            // Store the result.
            walk = prev;
            while (walk) |current| {
                visited[n] = current.id;
                n += 1;
                if (n >= visited.len) {
                    status.buffer_too_small = true;
                    break;
                }
                walk = pool.getNodeAtIdx(current.bits.parent_index);
            }
        }

        result_pos.* = best_pos;
        visited_count.* = n;

        return status;
    }

    /// The two ends of the portal from polygon `from` to its neighbour `to`,
    /// with what kind of polygon each is. Fails when either reference is invalid
    /// or the two are not linked; `left` and `right` are then left alone.
    fn getPortalPoints(
        self: *const NavMeshQuery,
        from: PolyRef,
        to: PolyRef,
        left: *[3]i32,
        right: *[3]i32,
        from_type: *PolyType,
        to_type: *PolyType,
    ) Status {
        const nav = self.nav.?;

        var from_tile: *const MeshTile = undefined;
        var from_poly: *const Poly = undefined;
        if (nav.getTileAndPolyByRef(from, &from_tile, &from_poly).failed) return invalid_param;
        from_type.* = from_poly.getType();

        var to_tile: *const MeshTile = undefined;
        var to_poly: *const Poly = undefined;
        if (nav.getTileAndPolyByRef(to, &to_tile, &to_poly).failed) return invalid_param;
        to_type.* = to_poly.getType();

        return self.getPortalPointsResolved(
            from,
            from_poly,
            from_tile,
            to,
            to_poly,
            to_tile,
            left,
            right,
        );
    }

    /// The two ends of the portal between two polygons whose tiles are known.
    /// Fails when `from` has no link to `to`; `left` and `right` are then left
    /// alone. The ends of a link that covers part of an edge are rounded to the
    /// nearest millimetre.
    fn getPortalPointsResolved(
        _: *const NavMeshQuery,
        from: PolyRef,
        from_poly: *const Poly,
        from_tile: *const MeshTile,
        to: PolyRef,
        to_poly: *const Poly,
        to_tile: *const MeshTile,
        left: *[3]i32,
        right: *[3]i32,
    ) Status {
        // Find the link that leads to the 'to' polygon.
        var found: ?*align(4) const Link = null;
        var i = from_poly.first_link;
        while (i != nav_mesh.null_link) : (i = from_tile.links[i].next) {
            if (from_tile.links[i].ref == to) {
                found = &from_tile.links[i];
                break;
            }
        }
        const link = found orelse return invalid_param;

        // An off-mesh connection's portal is the one vertex the link leaves from.
        if (from_poly.getType() == .off_mesh_connection) {
            i = from_poly.first_link;
            while (i != nav_mesh.null_link) : (i = from_tile.links[i].next) {
                if (from_tile.links[i].ref == to) {
                    const v = from_tile.links[i].edge;
                    left.* = from_tile.verts[from_poly.verts[v]];
                    right.* = from_tile.verts[from_poly.verts[v]];
                    return .success;
                }
            }
            return invalid_param;
        }

        if (to_poly.getType() == .off_mesh_connection) {
            i = to_poly.first_link;
            while (i != nav_mesh.null_link) : (i = to_tile.links[i].next) {
                if (to_tile.links[i].ref == from) {
                    const v = to_tile.links[i].edge;
                    left.* = to_tile.verts[to_poly.verts[v]];
                    right.* = to_tile.verts[to_poly.verts[v]];
                    return .success;
                }
            }
            return invalid_param;
        }

        // The portal is the edge the link crosses.
        const edge: usize = link.edge;
        const v0 = from_tile.verts[from_poly.verts[edge]];
        const v1 = from_tile.verts[from_poly.verts[(edge + 1) % from_poly.vert_count]];
        left.* = v0;
        right.* = v1;

        // A link across a tile border may cover only part of the edge, counted
        // in 255ths of it: clamp the portal to that part.
        if (link.side != 0xff) {
            if (link.bounds_min != 0 or link.bounds_max != 255) {
                left.* = common.lerp(v0, v1, Fraction.init(link.bounds_min, 255));
                right.* = common.lerp(v0, v1, Fraction.init(link.bounds_max, 255));
            }
        }

        return .success;
    }

    /// The middle of the portal from polygon `from` to its neighbour `to`,
    /// rounded to the nearest millimetre. On failure `mid` is left alone.
    fn getEdgeMidPoint(self: *const NavMeshQuery, from: PolyRef, to: PolyRef, mid: *[3]i32) Status {
        var left: [3]i32 = undefined;
        var right: [3]i32 = undefined;
        var from_type: PolyType = undefined;
        var to_type: PolyType = undefined;
        if (self.getPortalPoints(from, to, &left, &right, &from_type, &to_type).failed) {
            return invalid_param;
        }
        mid[0] = @intCast(common.divRound(@as(i64, left[0]) + right[0], 2));
        mid[1] = @intCast(common.divRound(@as(i64, left[1]) + right[1], 2));
        mid[2] = @intCast(common.divRound(@as(i64, left[2]) + right[2], 2));
        return .success;
    }

    /// The middle of the portal between two polygons whose tiles are known,
    /// rounded to the nearest millimetre. On failure `mid` is left alone.
    fn getEdgeMidPointResolved(
        self: *const NavMeshQuery,
        from: PolyRef,
        from_poly: *const Poly,
        from_tile: *const MeshTile,
        to: PolyRef,
        to_poly: *const Poly,
        to_tile: *const MeshTile,
        mid: *[3]i32,
    ) Status {
        var left: [3]i32 = undefined;
        var right: [3]i32 = undefined;
        if (self.getPortalPointsResolved(
            from,
            from_poly,
            from_tile,
            to,
            to_poly,
            to_tile,
            &left,
            &right,
        ).failed) return invalid_param;
        mid[0] = @intCast(common.divRound(@as(i64, left[0]) + right[0], 2));
        mid[1] = @intCast(common.divRound(@as(i64, left[1]) + right[1], 2));
        mid[2] = @intCast(common.divRound(@as(i64, left[2]) + right[2], 2));
        return .success;
    }

    /// Casts a ray along the surface, as `raycast` does without costs, with the
    /// results in separate places: `t` receives the hit fraction, `hit_normal`
    /// the normal of the wall hit and `path` the polygons crossed, as many as it
    /// holds, with `path_count` how many. A null `path` stores no polygons.
    pub fn raycastPath(
        self: *const NavMeshQuery,
        start_ref: PolyRef,
        start_pos: [3]i32,
        end_pos: [3]i32,
        filter: *const QueryFilter,
        t: *Fraction,
        hit_normal: ?*[3]i32,
        path: ?[]PolyRef,
        path_count: ?*usize,
    ) Status {
        const no_path: []PolyRef = &.{};
        var hit: RaycastHit = .{
            .t = .zero,
            .hit_normal = .{ 0, 0, 0 },
            .hit_edge_index = 0,
            .path = path orelse no_path,
            .path_count = 0,
            .path_cost = 0,
        };

        const status = self.raycast(start_ref, start_pos, end_pos, filter, .{}, &hit, 0);

        t.* = hit.t;
        if (hit_normal) |out| out.* = hit.hit_normal;
        if (path_count) |out| out.* = hit.path_count;

        return status;
    }

    /// Casts a ray along the surface from `start_pos` toward `end_pos` and
    /// reports the first wall it hits. Meant for quick checks over short
    /// distances.
    ///
    /// `hit.path` is the caller's buffer for the polygons the ray crosses, filled
    /// from the start as far as it goes, with `hit.path_count` how many. `hit.t`
    /// receives how far along the ray the wall was hit, as an exact fraction:
    /// `hit.point(start_pos, end_pos)` gives the hit point. `never` means the ray
    /// reached the end position with no wall in the way; zero means the start is
    /// on the wall hit. In both of those cases `hit.hit_normal` says nothing;
    /// otherwise it is the wall's normal in Q16.16. `hit.hit_edge_index` is the
    /// edge of the last polygon the ray left through, and `hit.path_cost` the
    /// filter's cost along the ray, in 1/256 millimetre at area cost one, when
    /// `options` asks for costs. `prev_ref` is the polygon the ray's start was
    /// entered from, for the cost; zero for none.
    ///
    /// The ray ignores the end position's height: it follows the surface it
    /// starts on until it is over or under the end position, so a floor passing
    /// under a balcony reaches a point on the balcony.
    pub fn raycast(
        self: *const NavMeshQuery,
        start_ref: PolyRef,
        start_pos: [3]i32,
        end_pos: [3]i32,
        filter: *const QueryFilter,
        options: RaycastOptions,
        hit: *RaycastHit,
        prev_ref: PolyRef,
    ) Status {
        const nav = self.nav.?;

        hit.t = .zero;
        hit.path_count = 0;
        hit.path_cost = 0;

        if (!nav.isValidPolyRef(start_ref) or
            !common.inBounds(start_pos) or
            !common.inBounds(end_pos) or
            (prev_ref != 0 and !nav.isValidPolyRef(prev_ref)))
        {
            return invalid_param;
        }

        var verts: [nav_mesh.verts_per_polygon][3]i32 = undefined;
        var n: usize = 0;

        var cur_pos = start_pos;
        const dir = common.vsub(end_pos, start_pos);
        hit.hit_normal = .{ 0, 0, 0 };

        var status: Status = .success;

        // The start reference was checked, the others come from the mesh.
        var cur_ref = start_ref;
        const start = nav.getTileAndPolyByRefUnsafe(cur_ref);
        var tile = start.tile;
        var poly = start.poly;
        // The next polygon's tile and polygon keep the last link looked at, also
        // when no link is taken: the filter's cost is handed them as they are.
        var next_tile = tile;
        var next_poly = poly;
        // The polygon the ray came from, replaced as the ray advances.
        var came_from_ref = prev_ref;
        var prev_tile = tile;
        var prev_poly = poly;
        if (came_from_ref != 0) {
            const prev = nav.getTileAndPolyByRefUnsafe(came_from_ref);
            prev_tile = prev.tile;
            prev_poly = prev.poly;
        }

        while (cur_ref != 0) {
            // Cast the ray against the current polygon.
            const vert_count: usize = poly.vert_count;
            for (0..vert_count) |i| verts[i] = tile.verts[poly.verts[i]];

            const clip = common.intersectSegmentPoly2d(start_pos, end_pos, verts[0..vert_count]);
            if (!clip.hit) {
                // The ray misses the polygon: keep the old t and report the hit.
                hit.path_count = n;
                return status;
            }

            hit.hit_edge_index = clip.seg_max;

            // Keep the furthest t so far.
            if (clip.t_max.greaterThan(hit.t)) hit.t = clip.t_max;

            // Store the visited polygon.
            if (n < hit.path.len) {
                hit.path[n] = cur_ref;
                n += 1;
            } else {
                status.buffer_too_small = true;
            }

            // The ray ends inside the polygon.
            if (clip.seg_max == -1) {
                hit.t = .never;
                hit.path_count = n;

                if (options.use_costs) {
                    hit.path_cost +|= filter.getCost(
                        cur_pos,
                        end_pos,
                        came_from_ref,
                        prev_tile,
                        prev_poly,
                        cur_ref,
                        tile,
                        poly,
                        cur_ref,
                        tile,
                        poly,
                    );
                }
                return status;
            }

            // The edge the ray leaves through.
            const edge: usize = @intCast(clip.seg_max);

            // Follow the neighbours.
            var next_ref: PolyRef = 0;

            var i = poly.first_link;
            while (i != nav_mesh.null_link) : (i = tile.links[i].next) {
                const link = &tile.links[i];

                // Only the links across the edge the ray leaves through.
                if (link.edge != edge) continue;

                const next = nav.getTileAndPolyByRefUnsafe(link.ref);
                next_tile = next.tile;
                next_poly = next.poly;

                // Skip off-mesh connections.
                if (next_poly.getType() == .off_mesh_connection) continue;

                // Skip the links the filter rejects.
                if (!filter.passFilter(link.ref, next_tile, next_poly)) continue;

                // A link inside the tile covers the whole edge.
                if (link.side == 0xff) {
                    next_ref = link.ref;
                    break;
                }

                // A link across a tile border that spans the whole edge.
                if (link.bounds_min == 0 and link.bounds_max == 255) {
                    next_ref = link.ref;
                    break;
                }

                // A link over part of the edge: take it when the ray crosses the
                // edge inside the link's part. The border runs along z on sides 0
                // and 4 and along x on sides 2 and 6.
                const axis: usize = switch (link.side) {
                    0, 4 => 2,
                    2, 6 => 0,
                    else => continue,
                };
                const left = tile.verts[poly.verts[link.edge]];
                const right = tile.verts[poly.verts[(@as(usize, link.edge) + 1) % poly.vert_count]];

                // The crossing lies at `start + dir * t` on the axis and the
                // link's ends at `left + span * bounds / 255`. Both are scaled by
                // 255 and by the denominator of `t`, which is positive, so the
                // comparison is exact; each side stays within 2^100.
                const t = clip.t_max;
                const crossing =
                    (@as(i128, start_pos[axis]) * t.den + @as(i128, dir[axis]) * t.num) * 255;
                const span: i128 = @as(i64, right[axis]) - left[axis];
                var link_min = (@as(i128, left[axis]) * 255 + span * link.bounds_min) * t.den;
                var link_max = (@as(i128, left[axis]) * 255 + span * link.bounds_max) * t.den;
                if (link_min > link_max) std.mem.swap(i128, &link_min, &link_max);

                if (crossing >= link_min and crossing <= link_max) {
                    next_ref = link.ref;
                    break;
                }
            }

            if (options.use_costs) {
                // The point where the ray leaves the polygon, put at the height of
                // the edge it leaves through: the ray itself moves in the xz plane.
                const last_pos = cur_pos;
                cur_pos = common.lerp(start_pos, end_pos, hit.t);
                const e1 = verts[edge];
                const e2 = verts[(edge + 1) % vert_count];
                const edge_dir = common.vsub(e2, e1);
                const diff = common.vsub(cur_pos, e1);
                // The height follows the edge's longer axis. The point's place
                // along it is held to the edge, so the height stays between the
                // heights of the edge's two ends.
                const axis: usize = if (@abs(edge_dir[0]) > @abs(edge_dir[2])) 0 else 2;
                const run: i64 = @abs(edge_dir[axis]);
                if (run == 0) {
                    cur_pos[1] = e1[1];
                } else {
                    const place: i64 = diff[axis];
                    const offset = if (edge_dir[axis] < 0) -place else place;
                    const along = std.math.clamp(offset, 0, run);
                    const rise: i64 = edge_dir[1];
                    cur_pos[1] = @intCast(e1[1] + common.divRound(rise * along, run));
                }

                hit.path_cost +|= filter.getCost(
                    last_pos,
                    cur_pos,
                    came_from_ref,
                    prev_tile,
                    prev_poly,
                    cur_ref,
                    tile,
                    poly,
                    next_ref,
                    next_tile,
                    next_poly,
                );
            }

            if (next_ref == 0) {
                // No neighbour: the ray hit a wall.
                const va = verts[edge];
                const vb = verts[if (edge + 1 < vert_count) edge + 1 else 0];
                const dx = vb[0] - va[0];
                const dz = vb[2] - va[2];
                hit.hit_normal = common.normalize(.{ dz, 0, -dx });

                hit.path_count = n;
                return status;
            }

            // No hit: advance to the neighbour.
            came_from_ref = cur_ref;
            cur_ref = next_ref;
            prev_tile = tile;
            tile = next_tile;
            prev_poly = poly;
            poly = next_poly;

            // With the buffer full the rest of the ray's polygons would be lost.
            if (status.buffer_too_small) {
                status.partial_result = true;
                break;
            }
        }

        hit.path_count = n;

        return status;
    }

    /// Finds the polygons a search from `start_ref` reaches whose portals lie
    /// within `radius` millimetres of `center_pos`, in order of the cost to
    /// reach them.
    ///
    /// Each of `result_refs`, `result_parents` (the polygon each was reached
    /// from, zero for the start) and `result_costs` (the cost from the centre,
    /// in 1/256 millimetre at area cost one) may be null; one given must hold
    /// `result_count_max` entries. `result_count` receives how many polygons
    /// were found, at most `result_count_max`.
    ///
    /// Only polygons the search walks to are found: one that touches the circle
    /// but is reached only through polygons outside it is left out. The centre
    /// is where the costs start, at its own height; the circle test is in the xz
    /// plane. `getPathFromDijkstraSearch` reads paths from the search afterwards,
    /// until another search runs.
    pub fn findPolysAroundCircle(
        self: *NavMeshQuery,
        start_ref: PolyRef,
        center_pos: [3]i32,
        radius: i32,
        filter: *const QueryFilter,
        result_refs: ?[]PolyRef,
        result_parents: ?[]PolyRef,
        result_costs: ?[]u64,
        result_count: *usize,
        result_count_max: usize,
    ) Status {
        const nav = self.nav.?;
        const pool = &self.node_pool;
        const open_list = &self.open_list;

        result_count.* = 0;

        if (!nav.isValidPolyRef(start_ref) or
            !common.inBounds(center_pos) or
            radius < 0 or
            radius > common.coord_max)
        {
            return invalid_param;
        }

        pool.clear();
        open_list.clear();

        const start_node = pool.getNode(start_ref, 0).?;
        start_node.pos = center_pos;
        start_node.bits.parent_index = 0;
        start_node.cost = 0;
        start_node.total = 0;
        start_node.id = start_ref;
        start_node.bits.flags = .{ .open = true };
        open_list.push(start_node);

        var status: Status = .success;

        var n: usize = 0;

        const reach: u64 = @intCast(radius);
        const radius_sqr = reach * reach;

        // The portal of the link at hand. A link whose portal cannot be found
        // keeps the portal of the link before it.
        var va: [3]i32 = .{ 0, 0, 0 };
        var vb: [3]i32 = .{ 0, 0, 0 };

        while (!open_list.isEmpty()) {
            const best_node = open_list.pop();
            best_node.bits.flags.open = false;
            best_node.bits.flags.closed = true;

            // The start reference was checked, the others come from the mesh.
            const best_ref = best_node.id;
            const best = nav.getTileAndPolyByRefUnsafe(best_ref);
            const best_tile = best.tile;
            const best_poly = best.poly;

            // The parent, so the search does not walk straight back.
            var parent_ref: PolyRef = 0;
            var parent_tile: ?*const MeshTile = null;
            var parent_poly: ?*const Poly = null;
            if (best_node.bits.parent_index != 0) {
                parent_ref = pool.getNodeAtIdx(best_node.bits.parent_index).?.id;
            }
            if (parent_ref != 0) {
                const parent = nav.getTileAndPolyByRefUnsafe(parent_ref);
                parent_tile = parent.tile;
                parent_poly = parent.poly;
            }

            if (n < result_count_max) {
                if (result_refs) |out| out[n] = best_ref;
                if (result_parents) |out| out[n] = parent_ref;
                if (result_costs) |out| out[n] = best_node.total;
                n += 1;
            } else {
                status.buffer_too_small = true;
            }

            var i = best_poly.first_link;
            while (i != nav_mesh.null_link) : (i = best_tile.links[i].next) {
                const link = &best_tile.links[i];
                const neighbour_ref = link.ref;
                // Skip invalid neighbours and do not follow back to the parent.
                if (neighbour_ref == 0 or neighbour_ref == parent_ref) continue;

                const neighbour = nav.getTileAndPolyByRefUnsafe(neighbour_ref);
                const neighbour_tile = neighbour.tile;
                const neighbour_poly = neighbour.poly;

                // Do not advance when the polygon is filtered out.
                if (!filter.passFilter(neighbour_ref, neighbour_tile, neighbour_poly)) continue;

                // The portal to the neighbour. A failure does not skip the link.
                _ = self.getPortalPointsResolved(
                    best_ref,
                    best_poly,
                    best_tile,
                    neighbour_ref,
                    neighbour_poly,
                    neighbour_tile,
                    &va,
                    &vb,
                );

                // The circle does not touch the portal: skip the neighbour.
                const dist_sqr = common.distancePointSegmentSqr2d(center_pos, va, vb).dist_sqr;
                if (dist_sqr > radius_sqr) continue;

                const neighbour_node = pool.getNode(neighbour_ref, 0) orelse {
                    status.out_of_nodes = true;
                    continue;
                };

                if (neighbour_node.bits.flags.closed) continue;

                // A node seen for the first time stands at the portal's middle.
                if (neighbour_node.bits.flags == NodeFlags{}) {
                    neighbour_node.pos = common.lerp(va, vb, .half);
                }

                const cost = filter.getCost(
                    best_node.pos,
                    neighbour_node.pos,
                    parent_ref,
                    parent_tile,
                    parent_poly,
                    best_ref,
                    best_tile,
                    best_poly,
                    neighbour_ref,
                    neighbour_tile,
                    neighbour_poly,
                );

                const total = best_node.total +| cost;

                // The node is already in the open list and the new way is no
                // better: skip.
                if (neighbour_node.bits.flags.open and total >= neighbour_node.total) continue;

                neighbour_node.id = neighbour_ref;
                neighbour_node.bits.parent_index = @intCast(pool.getNodeIdx(best_node));
                neighbour_node.total = total;

                if (neighbour_node.bits.flags.open) {
                    open_list.modify(neighbour_node);
                } else {
                    neighbour_node.bits.flags = .{ .open = true };
                    open_list.push(neighbour_node);
                }
            }
        }

        result_count.* = n;

        return status;
    }

    /// Finds the polygons a search from `start_ref` reaches whose portals cross
    /// the convex polygon `verts`, in order of the cost to reach them.
    ///
    /// The outputs are as `findPolysAroundCircle`'s. The costs start at the
    /// middle of `verts`, the average of its vertices rounded to the nearest
    /// millimetre, at its own height; the shape test is in the xz plane. Only
    /// polygons the search walks to are found. `getPathFromDijkstraSearch` reads
    /// paths from the search afterwards, until another search runs.
    pub fn findPolysAroundShape(
        self: *NavMeshQuery,
        start_ref: PolyRef,
        verts: []const [3]i32,
        filter: *const QueryFilter,
        result_refs: ?[]PolyRef,
        result_parents: ?[]PolyRef,
        result_costs: ?[]u64,
        result_count: *usize,
        result_count_max: usize,
    ) Status {
        const nav = self.nav.?;
        const pool = &self.node_pool;
        const open_list = &self.open_list;

        result_count.* = 0;

        if (!nav.isValidPolyRef(start_ref) or verts.len < 3) return invalid_param;
        for (verts) |v| {
            if (!common.inBounds(v)) return invalid_param;
        }

        pool.clear();
        open_list.clear();

        var sum: [3]i64 = .{ 0, 0, 0 };
        for (verts) |v| {
            sum[0] += v[0];
            sum[1] += v[1];
            sum[2] += v[2];
        }
        const center_pos: [3]i32 = .{
            @intCast(common.divRound(sum[0], verts.len)),
            @intCast(common.divRound(sum[1], verts.len)),
            @intCast(common.divRound(sum[2], verts.len)),
        };

        const start_node = pool.getNode(start_ref, 0).?;
        start_node.pos = center_pos;
        start_node.bits.parent_index = 0;
        start_node.cost = 0;
        start_node.total = 0;
        start_node.id = start_ref;
        start_node.bits.flags = .{ .open = true };
        open_list.push(start_node);

        var status: Status = .success;

        var n: usize = 0;

        // The portal of the link at hand. A link whose portal cannot be found
        // keeps the portal of the link before it.
        var va: [3]i32 = .{ 0, 0, 0 };
        var vb: [3]i32 = .{ 0, 0, 0 };

        while (!open_list.isEmpty()) {
            const best_node = open_list.pop();
            best_node.bits.flags.open = false;
            best_node.bits.flags.closed = true;

            // The start reference was checked, the others come from the mesh.
            const best_ref = best_node.id;
            const best = nav.getTileAndPolyByRefUnsafe(best_ref);
            const best_tile = best.tile;
            const best_poly = best.poly;

            // The parent, so the search does not walk straight back.
            var parent_ref: PolyRef = 0;
            var parent_tile: ?*const MeshTile = null;
            var parent_poly: ?*const Poly = null;
            if (best_node.bits.parent_index != 0) {
                parent_ref = pool.getNodeAtIdx(best_node.bits.parent_index).?.id;
            }
            if (parent_ref != 0) {
                const parent = nav.getTileAndPolyByRefUnsafe(parent_ref);
                parent_tile = parent.tile;
                parent_poly = parent.poly;
            }

            if (n < result_count_max) {
                if (result_refs) |out| out[n] = best_ref;
                if (result_parents) |out| out[n] = parent_ref;
                if (result_costs) |out| out[n] = best_node.total;
                n += 1;
            } else {
                status.buffer_too_small = true;
            }

            var i = best_poly.first_link;
            while (i != nav_mesh.null_link) : (i = best_tile.links[i].next) {
                const link = &best_tile.links[i];
                const neighbour_ref = link.ref;
                // Skip invalid neighbours and do not follow back to the parent.
                if (neighbour_ref == 0 or neighbour_ref == parent_ref) continue;

                const neighbour = nav.getTileAndPolyByRefUnsafe(neighbour_ref);
                const neighbour_tile = neighbour.tile;
                const neighbour_poly = neighbour.poly;

                // Do not advance when the polygon is filtered out.
                if (!filter.passFilter(neighbour_ref, neighbour_tile, neighbour_poly)) continue;

                // The portal to the neighbour. A failure does not skip the link.
                _ = self.getPortalPointsResolved(
                    best_ref,
                    best_poly,
                    best_tile,
                    neighbour_ref,
                    neighbour_poly,
                    neighbour_tile,
                    &va,
                    &vb,
                );

                // The shape does not touch the portal: skip the neighbour.
                const clip = common.intersectSegmentPoly2d(va, vb, verts);
                if (!clip.hit) continue;
                if (clip.t_min.greaterThan(.one) or clip.t_max.isNegative()) continue;

                const neighbour_node = pool.getNode(neighbour_ref, 0) orelse {
                    status.out_of_nodes = true;
                    continue;
                };

                if (neighbour_node.bits.flags.closed) continue;

                // A node seen for the first time stands at the portal's middle.
                if (neighbour_node.bits.flags == NodeFlags{}) {
                    neighbour_node.pos = common.lerp(va, vb, .half);
                }

                const cost = filter.getCost(
                    best_node.pos,
                    neighbour_node.pos,
                    parent_ref,
                    parent_tile,
                    parent_poly,
                    best_ref,
                    best_tile,
                    best_poly,
                    neighbour_ref,
                    neighbour_tile,
                    neighbour_poly,
                );

                const total = best_node.total +| cost;

                // The node is already in the open list and the new way is no
                // better: skip.
                if (neighbour_node.bits.flags.open and total >= neighbour_node.total) continue;

                neighbour_node.id = neighbour_ref;
                neighbour_node.bits.parent_index = @intCast(pool.getNodeIdx(best_node));
                neighbour_node.total = total;

                if (neighbour_node.bits.flags.open) {
                    open_list.modify(neighbour_node);
                } else {
                    neighbour_node.bits.flags = .{ .open = true };
                    open_list.push(neighbour_node);
                }
            }
        }

        result_count.* = n;

        return status;
    }

    /// Reads the path from the start of the last `findPolysAroundCircle` or
    /// `findPolysAroundShape` search to `end_ref`, a polygon that search found.
    /// `path` receives the polygons from the start, as many as it holds, and
    /// `path_count` how many. Any other search on this query in between makes the
    /// answer meaningless.
    pub fn getPathFromDijkstraSearch(
        self: *const NavMeshQuery,
        end_ref: PolyRef,
        path: []PolyRef,
        path_count: *usize,
    ) Status {
        const nav = self.nav.?;

        if (!nav.isValidPolyRef(end_ref)) return invalid_param;

        path_count.* = 0;

        var end_nodes: [1]*Node = undefined;
        if (self.node_pool.findNodes(end_ref, &end_nodes) != 1 or
            !end_nodes[0].bits.flags.closed)
        {
            return invalid_param;
        }

        return self.getPathToNode(end_nodes[0], path, path_count);
    }

    /// Finds the polygons around `start_ref` whose portals lie within `radius`
    /// millimetres of `center_pos` and that do not overlap one another in the xz
    /// plane, so their wall segments together describe one flat neighbourhood.
    ///
    /// `result_refs` receives the polygons, the start first, as many as it holds;
    /// `result_parents`, when given, the polygon each was reached from, zero for
    /// the start, and must be at least as long. `result_count` receives how many
    /// were written. The search visits few polygons: it is meant for a radius of
    /// a few polygons' width.
    pub fn findLocalNeighbourhood(
        self: *NavMeshQuery,
        start_ref: PolyRef,
        center_pos: [3]i32,
        radius: i32,
        filter: *const QueryFilter,
        result_refs: []PolyRef,
        result_parents: ?[]PolyRef,
        result_count: *usize,
    ) Status {
        const nav = self.nav.?;
        const pool = &self.tiny_node_pool;

        result_count.* = 0;

        if (!nav.isValidPolyRef(start_ref) or
            !common.inBounds(center_pos) or
            radius < 0 or
            radius > common.coord_max)
        {
            return invalid_param;
        }

        var stack: [search_stack_size]*Node = undefined;
        var stack_count: usize = 0;

        pool.clear();

        const start_node = pool.getNode(start_ref, 0).?;
        start_node.bits.parent_index = 0;
        start_node.id = start_ref;
        start_node.bits.flags = .{ .closed = true };
        stack[stack_count] = start_node;
        stack_count += 1;

        const reach: u64 = @intCast(radius);
        const radius_sqr = reach * reach;

        var pa: [nav_mesh.verts_per_polygon][3]i32 = undefined;
        var pb: [nav_mesh.verts_per_polygon][3]i32 = undefined;

        var status: Status = .success;

        var n: usize = 0;
        if (n < result_refs.len) {
            result_refs[n] = start_node.id;
            if (result_parents) |out| out[n] = 0;
            n += 1;
        } else {
            status.buffer_too_small = true;
        }

        // The portal of the link at hand. A link whose portal cannot be found
        // keeps the portal of the link before it.
        var va: [3]i32 = .{ 0, 0, 0 };
        var vb: [3]i32 = .{ 0, 0, 0 };

        while (stack_count != 0) {
            // Pop the front.
            const cur_node = stack[0];
            for (0..stack_count - 1) |i| stack[i] = stack[i + 1];
            stack_count -= 1;

            // The start reference was checked, the others come from the mesh.
            const cur_ref = cur_node.id;
            const cur = nav.getTileAndPolyByRefUnsafe(cur_ref);
            const cur_tile = cur.tile;
            const cur_poly = cur.poly;

            var i = cur_poly.first_link;
            while (i != nav_mesh.null_link) : (i = cur_tile.links[i].next) {
                const link = &cur_tile.links[i];
                const neighbour_ref = link.ref;
                // Skip invalid neighbours.
                if (neighbour_ref == 0) continue;

                // Skip when no node can be had.
                const neighbour_node = pool.getNode(neighbour_ref, 0) orelse continue;
                // Skip visited polygons.
                if (neighbour_node.bits.flags.closed) continue;

                const neighbour = nav.getTileAndPolyByRefUnsafe(neighbour_ref);
                const neighbour_tile = neighbour.tile;
                const neighbour_poly = neighbour.poly;

                // Skip off-mesh connections.
                if (neighbour_poly.getType() == .off_mesh_connection) continue;

                // Do not advance when the polygon is filtered out.
                if (!filter.passFilter(neighbour_ref, neighbour_tile, neighbour_poly)) continue;

                // The portal to the neighbour. A failure does not skip the link.
                _ = self.getPortalPointsResolved(
                    cur_ref,
                    cur_poly,
                    cur_tile,
                    neighbour_ref,
                    neighbour_poly,
                    neighbour_tile,
                    &va,
                    &vb,
                );

                // The circle does not touch the portal: skip the neighbour.
                const dist_sqr = common.distancePointSegmentSqr2d(center_pos, va, vb).dist_sqr;
                if (dist_sqr > radius_sqr) continue;

                // Mark the node visited before the overlap test, so a polygon
                // that fails it is not tested again from another side.
                neighbour_node.bits.flags.closed = true;
                neighbour_node.bits.parent_index = @intCast(pool.getNodeIdx(cur_node));

                // The polygon is taken only when it overlaps none of those taken
                // so far.
                const pa_count: usize = neighbour_poly.vert_count;
                for (0..pa_count) |k| pa[k] = neighbour_tile.verts[neighbour_poly.verts[k]];

                var overlap = false;
                for (result_refs[0..n]) |past_ref| {
                    // A polygon linked to the current one is taken not to
                    // overlap its neighbours.
                    var connected = false;
                    var k = cur_poly.first_link;
                    while (k != nav_mesh.null_link) : (k = cur_tile.links[k].next) {
                        if (cur_tile.links[k].ref == past_ref) {
                            connected = true;
                            break;
                        }
                    }
                    if (connected) continue;

                    // A polygon that may overlap.
                    const past = nav.getTileAndPolyByRefUnsafe(past_ref);
                    const pb_count: usize = past.poly.vert_count;
                    for (0..pb_count) |v| pb[v] = past.tile.verts[past.poly.verts[v]];

                    if (common.overlapPolyPoly2d(pa[0..pa_count], pb[0..pb_count])) {
                        overlap = true;
                        break;
                    }
                }
                if (overlap) continue;

                // The polygon is good: store it.
                if (n < result_refs.len) {
                    result_refs[n] = neighbour_ref;
                    if (result_parents) |out| out[n] = cur_ref;
                    n += 1;
                } else {
                    status.buffer_too_small = true;
                }

                if (stack_count < search_stack_size) {
                    stack[stack_count] = neighbour_node;
                    stack_count += 1;
                }
            }
        }

        result_count.* = n;

        return status;
    }

    /// Finds the segments of polygon `ref`'s outline.
    ///
    /// With `segment_refs` null only the walls are returned: the parts of the
    /// outline with no neighbour the filter accepts. With `segment_refs` given
    /// the portals are returned too, and it receives the neighbour across each
    /// segment, zero for a wall; it must be at least as long as `segment_verts`.
    /// `segment_verts` receives the two ends of each segment, as many segments
    /// as it holds, and `segment_count` how many. An edge on a tile border is cut
    /// into the parts its links cover and the walls between them, with the cuts
    /// rounded to the nearest millimetre.
    pub fn getPolyWallSegments(
        self: *const NavMeshQuery,
        ref: PolyRef,
        filter: *const QueryFilter,
        segment_verts: [][2][3]i32,
        segment_refs: ?[]PolyRef,
        segment_count: *usize,
    ) Status {
        const nav = self.nav.?;

        segment_count.* = 0;

        var tile: *const MeshTile = undefined;
        var poly: *const Poly = undefined;
        if (nav.getTileAndPolyByRef(ref, &tile, &poly).failed) return invalid_param;

        var n: usize = 0;
        var intervals: [interval_count_max]SegmentInterval = undefined;
        var interval_count: usize = 0;

        const store_portals = segment_refs != null;

        var status: Status = .success;

        const vert_count: usize = poly.vert_count;
        var j = vert_count -% 1;
        for (0..vert_count) |i| {
            defer j = i;

            interval_count = 0;
            if (poly.neighbours[j] & nav_mesh.ext_link != 0) {
                // A tile border: the parts of the edge its links cover.
                var k = poly.first_link;
                while (k != nav_mesh.null_link) : (k = tile.links[k].next) {
                    const link = &tile.links[k];
                    if (link.edge != j) continue;
                    if (link.ref == 0) continue;
                    const neighbour = nav.getTileAndPolyByRefUnsafe(link.ref);
                    if (!filter.passFilter(link.ref, neighbour.tile, neighbour.poly)) continue;
                    insertInterval(
                        &intervals,
                        &interval_count,
                        link.bounds_min,
                        link.bounds_max,
                        link.ref,
                    );
                }
            } else {
                // An edge inside the tile.
                var neighbour_ref: PolyRef = 0;
                if (poly.neighbours[j] != 0) {
                    const index: u32 = poly.neighbours[j] - 1;
                    neighbour_ref = nav.getPolyRefBase(tile) | @as(PolyRef, index);
                    if (!filter.passFilter(neighbour_ref, tile, &tile.polys[index])) {
                        neighbour_ref = 0;
                    }
                }

                // A portal, and portals are not asked for.
                if (neighbour_ref != 0 and !store_portals) continue;

                if (n < segment_verts.len) {
                    const vj = tile.verts[poly.verts[j]];
                    const vi = tile.verts[poly.verts[i]];
                    segment_verts[n] = .{ vj, vi };
                    if (segment_refs) |out| out[n] = neighbour_ref;
                    n += 1;
                } else {
                    status.buffer_too_small = true;
                }

                continue;
            }

            // Add sentinels, so the gaps before the first and after the last
            // interval come out as walls too.
            insertInterval(&intervals, &interval_count, -1, 0, 0);
            insertInterval(&intervals, &interval_count, 255, 256, 0);

            // Store the segments. Every end used lies within 0..255: the first
            // sentinel gives only its end and the last only its start.
            const vj = tile.verts[poly.verts[j]];
            const vi = tile.verts[poly.verts[i]];
            var k: usize = 1;
            while (k < interval_count) : (k += 1) {
                // The portal of this interval.
                if (store_portals and intervals[k].ref != 0) {
                    const t_min: Fraction = .init(intervals[k].t_min, 255);
                    const t_max: Fraction = .init(intervals[k].t_max, 255);
                    if (n < segment_verts.len) {
                        segment_verts[n] = .{
                            common.lerp(vj, vi, t_min),
                            common.lerp(vj, vi, t_max),
                        };
                        if (segment_refs) |out| out[n] = intervals[k].ref;
                        n += 1;
                    } else {
                        status.buffer_too_small = true;
                    }
                }

                // The wall between this interval and the one before it.
                const gap_min: i32 = intervals[k - 1].t_max;
                const gap_max: i32 = intervals[k].t_min;
                if (gap_min != gap_max) {
                    const t_min: Fraction = .init(gap_min, 255);
                    const t_max: Fraction = .init(gap_max, 255);
                    if (n < segment_verts.len) {
                        segment_verts[n] = .{
                            common.lerp(vj, vi, t_min),
                            common.lerp(vj, vi, t_max),
                        };
                        if (segment_refs) |out| out[n] = 0;
                        n += 1;
                    } else {
                        status.buffer_too_small = true;
                    }
                }
            }
        }

        segment_count.* = n;

        return status;
    }

    /// Finds the wall nearest to `center_pos` within `radius_max` millimetres,
    /// searching from `start_ref`.
    ///
    /// `hit_dist` receives the distance to the wall in millimetres, rounded
    /// down, or `radius_max` when there is none within it. `hit_pos` receives
    /// the nearest point on the wall, rounded to the nearest millimetre; with no
    /// wall in reach it is left as the caller passed it, and `hit_normal` is
    /// computed from that. `hit_normal` receives the direction from the wall
    /// point toward the centre, of length one in Q16.16; it is zero when the two
    /// are the same point, or when `hit_pos` was left at a position outside the
    /// coordinate bound. The distances are measured in the xz plane.
    pub fn findDistanceToWall(
        self: *NavMeshQuery,
        start_ref: PolyRef,
        center_pos: [3]i32,
        radius_max: i32,
        filter: *const QueryFilter,
        hit_dist: *i32,
        hit_pos: *[3]i32,
        hit_normal: *[3]i32,
    ) Status {
        const nav = self.nav.?;
        const pool = &self.node_pool;
        const open_list = &self.open_list;

        if (!nav.isValidPolyRef(start_ref) or
            !common.inBounds(center_pos) or
            radius_max < 0 or
            radius_max > common.coord_max)
        {
            return invalid_param;
        }

        pool.clear();
        open_list.clear();

        const start_node = pool.getNode(start_ref, 0).?;
        start_node.pos = center_pos;
        start_node.bits.parent_index = 0;
        start_node.cost = 0;
        start_node.total = 0;
        start_node.id = start_ref;
        start_node.bits.flags = .{ .open = true };
        open_list.push(start_node);

        // Shrinks to the nearest wall found so far.
        const reach: u64 = @intCast(radius_max);
        var radius_sqr = reach * reach;

        var status: Status = .success;

        while (!open_list.isEmpty()) {
            const best_node = open_list.pop();
            best_node.bits.flags.open = false;
            best_node.bits.flags.closed = true;

            // The start reference was checked, the others come from the mesh.
            const best_ref = best_node.id;
            const best = nav.getTileAndPolyByRefUnsafe(best_ref);
            const best_tile = best.tile;
            const best_poly = best.poly;

            // The parent, so the search does not walk straight back.
            var parent_ref: PolyRef = 0;
            if (best_node.bits.parent_index != 0) {
                parent_ref = pool.getNodeAtIdx(best_node.bits.parent_index).?.id;
            }

            // Hit test the walls.
            const vert_count: usize = best_poly.vert_count;
            var j = vert_count -% 1;
            for (0..vert_count) |i| {
                defer j = i;

                // Skip non-solid edges.
                if (best_poly.neighbours[j] & nav_mesh.ext_link != 0) {
                    // A tile border: only the first link across the edge decides.
                    var solid = true;
                    var k = best_poly.first_link;
                    while (k != nav_mesh.null_link) : (k = best_tile.links[k].next) {
                        const link = &best_tile.links[k];
                        if (link.edge == j) {
                            if (link.ref != 0) {
                                const neighbour = nav.getTileAndPolyByRefUnsafe(link.ref);
                                if (filter.passFilter(link.ref, neighbour.tile, neighbour.poly)) {
                                    solid = false;
                                }
                            }
                            break;
                        }
                    }
                    if (!solid) continue;
                } else if (best_poly.neighbours[j] != 0) {
                    // An edge inside the tile.
                    const index: u32 = best_poly.neighbours[j] - 1;
                    const ref = nav.getPolyRefBase(best_tile) | @as(PolyRef, index);
                    if (filter.passFilter(ref, best_tile, &best_tile.polys[index])) continue;
                }

                // The distance to the wall edge.
                const vj = best_tile.verts[best_poly.verts[j]];
                const vi = best_tile.verts[best_poly.verts[i]];
                const nearest = common.distancePointSegmentSqr2d(center_pos, vj, vi);

                // The edge is too far.
                if (nearest.dist_sqr > radius_sqr) continue;

                // The hit is closer: shrink the search radius.
                radius_sqr = nearest.dist_sqr;
                hit_pos.* = common.lerp(vj, vi, nearest.t);
            }

            var i = best_poly.first_link;
            while (i != nav_mesh.null_link) : (i = best_tile.links[i].next) {
                const link = &best_tile.links[i];
                const neighbour_ref = link.ref;
                // Skip invalid neighbours and do not follow back to the parent.
                if (neighbour_ref == 0 or neighbour_ref == parent_ref) continue;

                const neighbour = nav.getTileAndPolyByRefUnsafe(neighbour_ref);
                const neighbour_tile = neighbour.tile;
                const neighbour_poly = neighbour.poly;

                // Skip off-mesh connections.
                if (neighbour_poly.getType() == .off_mesh_connection) continue;

                // The distance to the edge the link crosses.
                const edge: usize = link.edge;
                const va = best_tile.verts[best_poly.verts[edge]];
                const vb = best_tile.verts[best_poly.verts[(edge + 1) % best_poly.vert_count]];
                const dist_sqr = common.distancePointSegmentSqr2d(center_pos, va, vb).dist_sqr;

                // The circle does not touch the edge: skip the neighbour.
                if (dist_sqr > radius_sqr) continue;

                if (!filter.passFilter(neighbour_ref, neighbour_tile, neighbour_poly)) continue;

                const neighbour_node = pool.getNode(neighbour_ref, 0) orelse {
                    status.out_of_nodes = true;
                    continue;
                };

                if (neighbour_node.bits.flags.closed) continue;

                // A node seen for the first time stands at the portal's middle.
                // Without a portal it keeps the position the node last held.
                // The position as a value: read back from the node just after
                // its parts were stored, it would wait for the stores.
                var neighbour_pos = neighbour_node.pos;
                if (neighbour_node.bits.flags == NodeFlags{}) {
                    _ = self.getEdgeMidPointResolved(
                        best_ref,
                        best_poly,
                        best_tile,
                        neighbour_ref,
                        neighbour_poly,
                        neighbour_tile,
                        &neighbour_pos,
                    );
                    neighbour_node.pos = neighbour_pos;
                }

                const total = best_node.total +| common.vdist(best_node.pos, neighbour_pos);

                // The node is already in the open list and the new way is no
                // better: skip.
                if (neighbour_node.bits.flags.open and total >= neighbour_node.total) continue;

                neighbour_node.id = neighbour_ref;
                neighbour_node.bits.flags.closed = false;
                neighbour_node.bits.parent_index = @intCast(pool.getNodeIdx(best_node));
                neighbour_node.total = total;

                if (neighbour_node.bits.flags.open) {
                    open_list.modify(neighbour_node);
                } else {
                    neighbour_node.bits.flags.open = true;
                    open_list.push(neighbour_node);
                }
            }
        }

        // A wall point is within the coordinate bound; a position the caller
        // left in `hit_pos` may not be, and the difference would then not fit.
        hit_normal.* = if (common.inBounds(hit_pos.*))
            common.normalize(common.vsub(center_pos, hit_pos.*))
        else
            .{ 0, 0, 0 };

        // The squared radius is at most the square of `radius_max`.
        hit_dist.* = @intCast(common.isqrt(radius_sqr));

        return status;
    }
};

/// Whether no search has touched the node since the pool handed it out.
inline fn isUnvisited(search_node: *const Node) bool {
    return @as(u3, @bitCast(search_node.bits.flags)) == 0;
}

/// A polygon's weight when one is picked by area: the sum of `triArea2d`
/// over its triangle fan, twice its area in the xz plane in square
/// millimetres, exact. Below zero for a polygon wound the other way.
fn polyArea2d(tile: *const MeshTile, poly: *const Poly) i64 {
    // A triangle's part is at most 2^60 either way, and a fan has at most
    // four.
    var area: i64 = 0;
    var j: usize = 2;
    while (j < poly.vert_count) : (j += 1) {
        const va = tile.verts[poly.verts[0]];
        const vb = tile.verts[poly.verts[j - 1]];
        const vc = tile.verts[poly.verts[j]];
        area += common.triArea2d(va, vb, vc);
    }
    return area;
}

/// The point of a polygon that the draws `s` and `t` pick, each read as a
/// number in 0..1 with 2^32 as one, at the height of the polygon's own
/// vertices.
fn randomPointInPoly(tile: *const MeshTile, poly: *const Poly, s: u32, t: u32) [3]i32 {
    var verts: [nav_mesh.verts_per_polygon][3]i32 = undefined;
    var areas: [nav_mesh.verts_per_polygon]i64 = undefined;
    verts[0] = tile.verts[poly.verts[0]];
    var j: usize = 1;
    while (j < poly.vert_count) : (j += 1) {
        verts[j] = tile.verts[poly.verts[j]];
    }
    return common.randomPointInConvexPoly(verts[0..poly.vert_count], &areas, s, t);
}

/// The cell of a tile's bounding-volume tree that `coord` falls in along
/// one axis: `coord` held to the tile's bounds `low`..`high`, measured from
/// `low` in cells of `cell_size` millimetres, and held to the tree's range
/// of 0..65535.
fn treeCell(coord: i32, low: i32, high: i32, cell_size: i32) u16 {
    const held = if (coord < low) low else if (coord > high) high else coord;
    return @intCast(std.math.clamp(@divFloor(held - low, cell_size), 0, 65535));
}

/// Keeps the polygon nearest to a point among those a query hands over.
const NearestPolyQuery = struct {
    query: *const NavMeshQuery,
    center: [3]i32,
    /// In square millimetres; the largest `u64` while nothing is kept.
    nearest_distance_sqr: u64 = std.math.maxInt(u64),
    nearest_ref: PolyRef = 0,
    nearest_point: [3]i32 = .{ 0, 0, 0 },
    over_poly: bool = false,

    const vtable: PolyQuery.VTable = .{ .process = process };

    /// The receiver to hand to `queryPolygonsWith`. It points at `self`,
    /// which must stay in place for the call.
    fn polyQuery(self: *NearestPolyQuery) PolyQuery {
        return .{ .ptr = self, .vtable = &vtable };
    }

    fn process(
        ptr: *anyopaque,
        tile: *const MeshTile,
        polys: []const *const Poly,
        refs: []const PolyRef,
    ) void {
        const self: *NearestPolyQuery = @ptrCast(@alignCast(ptr));
        _ = polys;

        for (refs) |ref| {
            var closest: [3]i32 = .{ 0, 0, 0 };
            var pos_over_poly = false;
            _ = self.query.closestPointOnPoly(ref, self.center, &closest, &pos_over_poly);

            // A point over the polygon is as near as its height difference
            // beyond what an agent climbs, so the floor just below beats a
            // ledge beside.
            const diff = common.vsub(self.center, closest);
            var distance_sqr: u64 = undefined;
            if (pos_over_poly) {
                // At most 2^30 less a 32-bit climb height: the square fits.
                const over: i64 = @as(i64, @abs(diff[1])) - tile.header.?.walkable_climb;
                const beyond: u64 = if (over > 0) @intCast(over) else 0;
                distance_sqr = beyond * beyond;
            } else {
                distance_sqr = common.vlenSqr(diff);
            }

            if (distance_sqr < self.nearest_distance_sqr) {
                self.nearest_point = closest;
                self.nearest_distance_sqr = distance_sqr;
                self.nearest_ref = ref;
                self.over_poly = pos_over_poly;
            }
        }
    }
};

/// Copies the references a query hands over into a buffer, noting when
/// there were more than it holds.
const CollectPolysQuery = struct {
    polys: []PolyRef,
    collected_count: usize = 0,
    overflow: bool = false,

    const vtable: PolyQuery.VTable = .{ .process = process };

    /// The receiver to hand to `queryPolygonsWith`. It points at `self`,
    /// which must stay in place for the call.
    fn polyQuery(self: *CollectPolysQuery) PolyQuery {
        return .{ .ptr = self, .vtable = &vtable };
    }

    fn process(
        ptr: *anyopaque,
        tile: *const MeshTile,
        polys: []const *const Poly,
        refs: []const PolyRef,
    ) void {
        const self: *CollectPolysQuery = @ptrCast(@alignCast(ptr));
        _ = tile;
        _ = polys;

        const left_count = self.polys.len - self.collected_count;
        var copy_count = refs.len;
        if (copy_count > left_count) {
            self.overflow = true;
            copy_count = left_count;
        }

        @memcpy(self.polys[self.collected_count..][0..copy_count], refs[0..copy_count]);
        self.collected_count += copy_count;
    }
};

const in_progress: Status = .{ .in_progress = true };

/// How many polygons a breadth-first search over a few polygons queues at
/// once.
const search_stack_size = 48;

/// The most neighbours taken across one polygon edge on a tile border.
const edge_neighbour_count_max = 8;

/// The most parts one polygon edge on a tile border is cut into.
const interval_count_max = 16;

/// The part of a polygon edge one link covers, in 255ths of the edge.
const SegmentInterval = struct {
    ref: PolyRef,
    t_min: i16,
    t_max: i16,
};

/// Adds an interval, keeping them ordered along the edge. A full list takes
/// no more.
fn insertInterval(
    intervals: *[interval_count_max]SegmentInterval,
    count: *usize,
    t_min: i16,
    t_max: i16,
    ref: PolyRef,
) void {
    const n = count.*;
    if (n + 1 > interval_count_max) return;

    // Find the place: before the first interval that starts at or after this
    // one's end.
    var index: usize = 0;
    while (index < n) : (index += 1) {
        if (t_max <= intervals[index].t_min) break;
    }

    // Move the later ones up.
    if (n - index != 0) @memmove(intervals[index + 1 .. n + 1], intervals[index..n]);

    intervals[index] = .{ .ref = ref, .t_min = t_min, .t_max = t_max };
    count.* = n + 1;
}
