//! The navigation mesh on whole millimetres: a set of tiles, each a blob of
//! convex polygons with their links, height detail and bounding-volume tree,
//! addressed by polygon references that carry a salt, a tile index and a
//! polygon index.
//!
//! A tile blob has the float tile's layout with every float replaced by an
//! `i32` of the same size and place: `MeshHeader`, vertices, `Poly`s,
//! `Link`s, `PolyDetail`s, detail vertices, detail triangles, `BvNode`s and
//! `OffMeshConnection`s, each block padded to four bytes, in native byte
//! order. Its magic number is its own, so neither mesh takes the other's
//! tiles; `convert.tileFromFloat` makes an integer tile from a float one.
//!
//! Every position of a tile lies within `common.coord_max` of zero, which
//! `NavMesh.addTile` checks and the arithmetic here relies on.

const std = @import("std");
const common = @import("common.zig");
const float_nav_mesh = @import("../nav_mesh.zig");
const Status = @import("../status.zig").Status;

const Fraction = common.Fraction;

/// Whether polygon and tile references are 64 bits wide, with fixed bit
/// counts for salt, tile and polygon. Set by the `polyref64` build option.
pub const polyref64 = float_nav_mesh.polyref64;

/// A handle to one polygon of one tile. Zero is never a valid reference.
/// A reference outlives its tile: removing the tile changes the tile's
/// salt, after which `isValidPolyRef` is false for it.
pub const PolyRef = float_nav_mesh.PolyRef;

/// A handle to a tile: a polygon reference with polygon index zero. Zero
/// is never a valid reference. It goes stale as a `PolyRef` does.
pub const TileRef = float_nav_mesh.TileRef;

const RefShift = std.math.Log2Int(PolyRef);

/// The bit counts of a 64-bit reference's parts.
pub const salt_bits_64 = float_nav_mesh.salt_bits_64;
pub const tile_bits_64 = float_nav_mesh.tile_bits_64;
pub const poly_bits_64 = float_nav_mesh.poly_bits_64;

/// The most vertices a polygon has.
pub const verts_per_polygon = float_nav_mesh.verts_per_polygon;

/// Marks an integer tile blob: the bytes 'D', 'N', 'A', 'I' from the high
/// end. A float tile carries another mark.
pub const nav_mesh_magic: i32 = 'D' << 24 | 'N' << 16 | 'A' << 8 | 'I';
/// The tile blob format this code reads and writes.
pub const nav_mesh_version = float_nav_mesh.nav_mesh_version;
/// Marks a stored tile state: 'D', 'N', 'M', 'S' from the high end.
pub const nav_mesh_state_magic = float_nav_mesh.nav_mesh_state_magic;
/// The tile state format this code reads and writes.
pub const nav_mesh_state_version = float_nav_mesh.nav_mesh_state_version;

/// Set in a `Poly.neighbours` entry whose edge borders another tile; the
/// low byte is then the side of the tile the edge lies on.
pub const ext_link = float_nav_mesh.ext_link;
/// A link index that names no link.
pub const null_link = float_nav_mesh.null_link;
/// Set in `OffMeshConnection.flags` when the connection is walked both ways.
pub const off_mesh_connection_bidirectional = float_nav_mesh.off_mesh_connection_bidirectional;
/// The number of area ids a polygon can carry.
pub const area_count_max = float_nav_mesh.area_count_max;
/// Set in a detail triangle's edge flags when that edge lies on the
/// polygon's boundary.
pub const detail_edge_boundary = float_nav_mesh.detail_edge_boundary;
/// How many times the agent's radius a ray may run in an any-angle path
/// search before it is cut off.
pub const ray_cast_limit_proportions = 50;

/// Options for a tile handed to `NavMesh.addTile`.
pub const TileFlags = float_nav_mesh.TileFlags;

/// What a point of a straight path is.
pub const StraightPathFlags = float_nav_mesh.StraightPathFlags;

/// Which extra points `NavMeshQuery.findStraightPath` adds.
pub const StraightPathOptions = float_nav_mesh.StraightPathOptions;

/// Options for `NavMeshQuery.initSlicedFindPath`.
pub const FindPathOptions = float_nav_mesh.FindPathOptions;

/// Options for `NavMeshQuery.raycast`.
pub const RaycastOptions = float_nav_mesh.RaycastOptions;

/// What a polygon stands for.
pub const PolyType = float_nav_mesh.PolyType;

/// One convex polygon of a tile.
pub const Poly = float_nav_mesh.Poly;

/// Where one polygon's height detail sits in the tile's detail arrays.
pub const PolyDetail = float_nav_mesh.PolyDetail;

/// A step from one polygon to a neighbour. A tile's links live in one
/// array; a polygon's links form a list through `next`.
pub const Link = float_nav_mesh.Link;

/// A node of a tile's bounding-volume tree, in cells of
/// `MeshHeader.bv_cell_size` from the tile's minimum corner.
pub const BvNode = float_nav_mesh.BvNode;

/// A walkable jump between two points that the mesh surface does not join.
pub const OffMeshConnection = extern struct {
    /// The connection's start and end, in millimetres.
    pos: [2][3]i32,
    /// How far from an end point a polygon may be and still attach, in
    /// millimetres.
    radius: i32,
    /// The index, in the tile, of the polygon standing for the connection.
    poly: u16,
    /// `off_mesh_connection_bidirectional` or zero.
    flags: u8,
    /// The side of the tile the end point leaves through, 0xff when it
    /// stays inside.
    side: u8,
    /// An id the application defines.
    user_id: u32,
};

/// The head of a tile blob.
pub const MeshHeader = extern struct {
    magic: i32,
    version: i32,
    /// The tile's place in the tile grid.
    x: i32,
    y: i32,
    layer: i32,
    /// An id the application defines.
    user_id: u32,
    poly_count: i32,
    vert_count: i32,
    /// How many links the tile's link array holds.
    link_count_max: i32,
    detail_mesh_count: i32,
    detail_vert_count: i32,
    detail_tri_count: i32,
    /// Zero when the tile has no bounding-volume tree.
    bv_node_count: i32,
    off_mesh_connection_count: i32,
    /// The polygon index of the first off-mesh connection polygon.
    off_mesh_base: i32,
    /// The agent the tile was built for, in millimetres.
    walkable_height: i32,
    walkable_radius: i32,
    walkable_climb: i32,
    /// The tile's box, in millimetres.
    bounds_min: [3]i32,
    bounds_max: [3]i32,
    /// The width of one cell of the tree's boxes, in millimetres: at least
    /// 1 in a tile that has a tree.
    bv_cell_size: i32,
};

/// One tile of a nav mesh. The slices point into `data`, the tile's blob.
///
/// A `*const MeshTile` from a `NavMesh` stays at the same address for the
/// mesh's life, but its content goes stale: removing the tile empties every
/// slice, clears `header` and changes `salt`, and the entry is reused for a
/// tile added later. Hold a `TileRef` across changes, not the pointer.
pub const MeshTile = struct {
    /// Counts the tile entry's reuses; part of every reference into it.
    salt: u32,
    /// The first unused link, or `null_link`.
    links_free_list: u32,
    /// Null while the entry holds no tile.
    header: ?*MeshHeader,
    polys: []Poly,
    /// In millimetres.
    verts: [][3]i32,
    links: []align(4) Link,
    detail_meshes: []PolyDetail,
    /// In millimetres.
    detail_verts: [][3]i32,
    /// Three vertex indices and the edge flags of each detail triangle. An
    /// index below the polygon's vertex count names a polygon vertex, any
    /// other a detail vertex.
    detail_tris: [][4]u8,
    /// Empty when the tile has no bounding-volume tree.
    bv_tree: []BvNode,
    off_mesh_connections: []OffMeshConnection,
    /// The whole blob.
    data: []align(4) u8,
    flags: TileFlags,
    /// The next tile in the same lookup bucket, or the next free entry.
    next: ?*MeshTile,

    const unused: MeshTile = .{
        .salt = 0,
        .links_free_list = 0,
        .header = null,
        .polys = &.{},
        .verts = &.{},
        .links = &.{},
        .detail_meshes = &.{},
        .detail_verts = &.{},
        .detail_tris = &.{},
        .bv_tree = &.{},
        .off_mesh_connections = &.{},
        .data = &.{},
        .flags = .{},
        .next = null,
    };

    /// The index of `poly`, which must be one of this tile's polygons.
    pub inline fn polyIndex(tile: *const MeshTile, poly: *const Poly) u32 {
        const offset = @intFromPtr(poly) - @intFromPtr(tile.polys.ptr);
        return @intCast(offset / @sizeOf(Poly));
    }

    /// The height of the polygon's detail surface under `pos`, rounded to
    /// the nearest millimetre, or null when `pos` is outside the polygon in
    /// the xz plane or the polygon is an off-mesh connection.
    pub fn getPolyHeight(tile: *const MeshTile, poly: *const Poly, pos: [3]i32) ?i32 {
        // Off-mesh connections do not have detail polys and getting height
        // over them does not make sense.
        if (poly.getType() == .off_mesh_connection) return null;

        const detail = &tile.detail_meshes[tile.polyIndex(poly)];

        var verts: [verts_per_polygon][3]i32 = undefined;
        const vert_count: usize = poly.vert_count;
        for (0..vert_count) |i| verts[i] = tile.verts[poly.verts[i]];

        if (!common.pointInPolygon(pos, verts[0..vert_count])) return null;

        // Find height at the location.
        for (0..detail.tri_count) |j| {
            const tri = tile.detail_tris[detail.tri_base + j];
            var v: [3][3]i32 = undefined;
            for (0..3) |k| {
                v[k] = if (tri[k] < poly.vert_count)
                    tile.verts[poly.verts[tri[k]]]
                else
                    tile.detail_verts[detail.vert_base + (tri[k] - poly.vert_count)];
            }
            if (common.closestHeightOnTriangle(pos, v[0], v[1], v[2])) |height| return height;
        }

        // The point is inside the polygon yet in none of its triangles: a
        // detail vertex rounded to a whole millimetre can leave a sliver
        // along an edge that no triangle covers, and a triangle with no
        // area in the plane holds no point. The nearest detail edge gives
        // the height then.
        return closestPointOnDetailEdges(tile, poly, pos, false)[1];
    }
};

/// The point on the polygon's detail edges nearest `pos` in the xz plane,
/// rounded to the nearest millimetre: on boundary edges only, or on every
/// edge. `pos` itself when the polygon has no such edge.
fn closestPointOnDetailEdges(
    tile: *const MeshTile,
    poly: *const Poly,
    pos: [3]i32,
    only_boundary: bool,
) [3]i32 {
    const detail = &tile.detail_meshes[tile.polyIndex(poly)];

    var distance_sqr_min: u64 = std.math.maxInt(u64);
    var t_min: Fraction = .zero;
    var edge: ?[2][3]i32 = null;

    for (0..detail.tri_count) |i| {
        const tri = tile.detail_tris[detail.tri_base + i];
        const any_boundary_edge: u8 = (detail_edge_boundary << 0) |
            (detail_edge_boundary << 2) |
            (detail_edge_boundary << 4);
        if (only_boundary and (tri[3] & any_boundary_edge) == 0) continue;

        var v: [3][3]i32 = undefined;
        for (0..3) |j| {
            v[j] = if (tri[j] < poly.vert_count)
                tile.verts[poly.verts[tri[j]]]
            else
                tile.detail_verts[detail.vert_base + (tri[j] - poly.vert_count)];
        }

        var k: usize = 0;
        var j: usize = 2;
        while (k < 3) : ({
            j = k;
            k += 1;
        }) {
            // Skip an inner edge when only the boundary counts, and
            // otherwise from one of the two triangles that share it.
            const flags = getDetailTriEdgeFlags(tri[3], @intCast(j));
            if ((flags & detail_edge_boundary) == 0 and (only_boundary or tri[j] < tri[k])) {
                continue;
            }

            // Of two edges at the same distance the first one stays.
            const distance = common.distancePointSegmentSqr2d(pos, v[j], v[k]);
            if (distance.dist_sqr < distance_sqr_min) {
                distance_sqr_min = distance.dist_sqr;
                t_min = distance.t;
                edge = .{ v[j], v[k] };
            }
        }
    }

    const nearest = edge orelse return pos;
    return common.lerp(nearest[0], nearest[1], t_min);
}

/// The boundary flags of one edge of a detail triangle, from the
/// triangle's flags byte.
pub const getDetailTriEdgeFlags = float_nav_mesh.getDetailTriEdgeFlags;

/// The shape of a tiled nav mesh.
pub const NavMeshParams = extern struct {
    /// The world position of tile (0, 0)'s minimum corner, in millimetres.
    origin: [3]i32,
    /// A tile's size along x, in millimetres: at least 1.
    tile_width: i32,
    /// A tile's size along z, in millimetres: at least 1.
    tile_height: i32,
    /// How many tiles the mesh can hold; bounds the tile bits of a
    /// reference.
    tile_count_max: i32,
    /// How many polygons a tile can hold; bounds the polygon bits of a
    /// reference.
    poly_count_max: i32,
};

/// A tile's place in the tile grid.
pub const TileLocation = float_nav_mesh.TileLocation;

/// The parts of a polygon reference.
pub const DecodedPolyId = float_nav_mesh.DecodedPolyId;

/// A tile and one of its polygons. Both pointers go stale as a `MeshTile`
/// does.
pub const TileAndPoly = struct {
    tile: *const MeshTile,
    poly: *const Poly,
};

/// The point of a polygon nearest to a position.
pub const ClosestPointOnPoly = struct {
    /// In millimetres.
    closest: [3]i32,
    /// Whether the position lies over the polygon in the xz plane; the
    /// closest point is then the position at the polygon's height.
    pos_over_poly: bool,
};

/// The head of a stored tile state.
const TileStateHeader = extern struct {
    magic: i32,
    version: i32,
    ref: TileRef,
};

/// One polygon's entry in a stored tile state. Its fourth byte is never
/// written.
const PolyStateRecord = extern struct {
    flags: u16,
    area: u8,
};

/// The two ends of a tile-border edge along the border, each as its
/// position on the border's axis and its height, in millimetres.
const Slab = struct {
    min: [2]i32,
    max: [2]i32,
};

/// Whether two border edges overlap along the border, pulled in by `px` at
/// each end so edges that only touch do not count, and lie within twice
/// `py` of each other in height at an end of the stretch they share, or
/// cross there. `px` must be above zero.
inline fn overlapSlabs(a: Slab, b: Slab, px: i32, py: i32) bool {
    const minx = @max(a.min[0] + px, b.min[0] + px);
    const maxx = @min(a.max[0] - px, b.max[0] - px);
    if (minx > maxx) return false;

    // Past the test above each edge runs at least twice `px` along the
    // border, so neither length is zero.
    const adx: i128 = a.max[0] - a.min[0];
    const ady: i128 = a.max[1] - a.min[1];
    const bdx: i128 = b.max[0] - b.min[0];
    const bdy: i128 = b.max[1] - b.min[1];

    // How far `b` is above `a` at `minx` and at `maxx`, as fractions over
    // `den`. Every difference of two coordinates is below 2^31, so each
    // term, a product of three, is below 2^93.
    const den = adx * bdx;
    const gap: i128 = b.min[1] - a.min[1];
    const dmin = gap * den + bdy * (minx - b.min[0]) * adx - ady * (minx - a.min[0]) * bdx;
    const dmax = gap * den + bdy * (maxx - b.min[0]) * adx - ady * (maxx - a.min[0]) * bdx;

    // Segments that cross always overlap.
    if ((dmin < 0 and dmax > 0) or (dmin > 0 and dmax < 0)) return true;

    const threshold = 2 * @as(u128, @abs(py)) * @as(u128, @intCast(den));
    if (@abs(dmin) <= threshold or @abs(dmax) <= threshold) return true;

    return false;
}

/// Where a border on `side` lies along the axis that crosses it.
fn getSlabCoord(va: [3]i32, side: i32) i32 {
    if (side == 0 or side == 4) {
        return va[0];
    } else if (side == 2 or side == 6) {
        return va[2];
    }
    return 0;
}

/// The edge `va`, `vb` as a slab of the border on `side`. A diagonal side
/// has no border, and gives zeros.
fn calcSlabEndPoints(va: [3]i32, vb: [3]i32, side: i32) Slab {
    if (side == 0 or side == 4) {
        if (va[2] < vb[2]) {
            return .{ .min = .{ va[2], va[1] }, .max = .{ vb[2], vb[1] } };
        } else {
            return .{ .min = .{ vb[2], vb[1] }, .max = .{ va[2], va[1] } };
        }
    } else if (side == 2 or side == 6) {
        if (va[0] < vb[0]) {
            return .{ .min = .{ va[0], va[1] }, .max = .{ vb[0], vb[1] } };
        } else {
            return .{ .min = .{ vb[0], vb[1] }, .max = .{ va[0], va[1] } };
        }
    }
    return .{ .min = .{ 0, 0 }, .max = .{ 0, 0 } };
}

/// The lookup bucket of the tiles at grid position `x`, `y`.
inline fn computeTileHash(x: i32, y: i32, mask: i32) usize {
    const h1: u32 = 0x8da6b343;
    const h2: u32 = 0xd8163841;
    const n = h1 *% @as(u32, @bitCast(x)) +% h2 *% @as(u32, @bitCast(y));
    return n & @as(u32, @bitCast(mask));
}

/// Takes a link from the tile's free list; `null_link` when none is left.
inline fn allocLink(tile: *MeshTile) u32 {
    if (tile.links_free_list == null_link) return null_link;
    const link = tile.links_free_list;
    tile.links_free_list = tile.links[link].next;
    return link;
}

inline fn freeLink(tile: *MeshTile, link: u32) void {
    tile.links[link].next = tile.links_free_list;
    tile.links_free_list = link;
}

/// `count` records of `T` at `offset` in a tile blob. Every block of a blob
/// starts at a multiple of four bytes.
fn blobSlice(comptime T: type, data: []align(4) u8, offset: usize, count: usize) []align(4) T {
    const records: [*]align(4) T = @ptrCast(@alignCast(data.ptr + offset));
    return records[0..count];
}

/// The cell of a tile's bounding-volume tree that `coord` falls in along
/// one axis: `coord` held to the tile's bounds `low`..`high`, measured from
/// `low` in cells of `cell_size` millimetres, and held to the tree's range
/// of 0..65535.
fn treeCell(coord: i32, low: i32, high: i32, cell_size: i32) u16 {
    const held = if (coord < low) low else if (coord > high) high else coord;
    return @intCast(std.math.clamp(@divFloor(held - low, cell_size), 0, 65535));
}

/// Whether every index the blocks of a tile blob hold stays inside the
/// tile: exactly the reads the mesh and its queries make without checking.
fn indicesStayInside(
    header: *const MeshHeader,
    vert_count: usize,
    polys: []const Poly,
    detail_meshes: []const PolyDetail,
    detail_vert_count: usize,
    detail_tris: []const [4]u8,
    bv_tree: []const BvNode,
    off_mesh_connections: []const OffMeshConnection,
) bool {
    const off_mesh_base: u32 = @bitCast(header.off_mesh_base);
    for (polys, 0..) |*poly, i| {
        if (poly.vert_count > verts_per_polygon) return false;
        for (0..poly.vert_count) |j| {
            if (poly.verts[j] >= vert_count) return false;
            // A neighbour inside the tile is a polygon index plus one.
            const neighbour = poly.neighbours[j];
            if ((neighbour & ext_link) == 0 and neighbour > polys.len) return false;
        }

        if (poly.getType() == .off_mesh_connection) {
            // Read by its first two vertices, whatever its count says.
            if (poly.verts[0] >= vert_count or poly.verts[1] >= vert_count) return false;
            const connection = @as(u32, @intCast(i)) -% off_mesh_base;
            if (connection >= off_mesh_connections.len) return false;
            continue;
        }

        if (i >= detail_meshes.len) return false;
        const detail = &detail_meshes[i];
        if (@as(u64, detail.tri_base) + detail.tri_count > detail_tris.len) return false;
        for (detail_tris[detail.tri_base..][0..detail.tri_count]) |tri| {
            // An index past the polygon's own vertices counts on from the
            // detail's first vertex.
            for (tri[0..3]) |index| {
                if (index < poly.vert_count) continue;
                const extra = @as(u64, detail.vert_base) + (index - poly.vert_count);
                if (extra >= detail_vert_count) return false;
            }
        }
    }

    for (bv_tree) |*node| {
        // A leaf names a ground polygon, as the builder makes them: one an
        // off-mesh connection lands on is read through its edge, and an
        // off-mesh connection has none. An inner node holds minus a step
        // forward.
        if (node.index >= 0) {
            if (node.index >= polys.len) return false;
            if (polys[@intCast(node.index)].getType() == .off_mesh_connection) return false;
        } else if (node.index == std.math.minInt(i32)) return false;
    }

    for (off_mesh_connections) |*connection| {
        if (connection.poly >= polys.len) return false;
        if (polys[connection.poly].getType() != .off_mesh_connection) return false;
    }
    return true;
}

const invalid_param: Status = Status.failure.with(.{ .invalid_param = true });
const out_of_memory: Status = Status.failure.with(.{ .out_of_memory = true });

/// A navigation mesh: a table of tiles, found by grid position or by
/// reference.
///
/// A mesh starts as `empty`; `init` or `initSingleTile` readies it and
/// `deinit` ends it. It keeps the allocator it was readied with, for its
/// tile table and for the blobs of tiles added with `TileFlags.free_data`.
pub const NavMesh = struct {
    gpa: std.mem.Allocator,
    params: NavMeshParams,
    origin: [3]i32,
    tile_width: i32,
    tile_height: i32,
    tile_count_max: i32,
    tile_lookup_size: i32,
    tile_lookup_mask: i32,
    /// Buckets of tiles by grid position, chained through `MeshTile.next`.
    position_lookup: []?*MeshTile,
    /// The unused tile entries, chained through `MeshTile.next`.
    next_free: ?*MeshTile,
    tiles: []MeshTile,
    /// The bit counts of a 32-bit reference's parts. With `polyref64` the
    /// counts are fixed and these stay zero.
    salt_bits: u32,
    tile_bits: u32,
    poly_bits: u32,

    /// A mesh with no tile table: every lookup finds nothing. Its grid
    /// cells are a millimetre wide, the smallest `init` takes, so
    /// `calcTileLoc` has a size to divide by.
    pub const empty: NavMesh = .{
        .gpa = .failing,
        .params = .{
            .origin = .{ 0, 0, 0 },
            .tile_width = 1,
            .tile_height = 1,
            .tile_count_max = 0,
            .poly_count_max = 0,
        },
        .origin = .{ 0, 0, 0 },
        .tile_width = 1,
        .tile_height = 1,
        .tile_count_max = 0,
        .tile_lookup_size = 0,
        .tile_lookup_mask = 0,
        .position_lookup = &.{},
        .next_free = null,
        .tiles = &.{},
        .salt_bits = 0,
        .tile_bits = 0,
        .poly_bits = 0,
    };

    /// Readies an `empty` mesh for tiles added with `addTile`. A tile width
    /// or height below 1 and an origin outside `common.coord_max` are
    /// refused with `invalid_param`. On failure the mesh is left as it was.
    pub fn init(self: *NavMesh, gpa: std.mem.Allocator, params: *const NavMeshParams) Status {
        // `calcTileLoc` divides by the tile's size and measures from the
        // origin.
        if (params.tile_width < 1 or params.tile_height < 1) return invalid_param;
        if (!common.inBounds(params.origin)) return invalid_param;

        // A negative count asks for more memory than there is.
        const tile_count = std.math.cast(usize, params.tile_count_max) orelse
            return out_of_memory;
        var lookup_size = common.nextPow2(@intCast(@divTrunc(params.tile_count_max, 4)));
        if (lookup_size == 0) lookup_size = 1;

        const tiles = gpa.alloc(MeshTile, tile_count) catch return out_of_memory;
        const position_lookup = gpa.alloc(?*MeshTile, lookup_size) catch {
            gpa.free(tiles);
            return out_of_memory;
        };
        @memset(tiles, .unused);
        @memset(position_lookup, null);

        // The free list runs from the first entry to the last.
        var next_free: ?*MeshTile = null;
        var i = tile_count;
        while (i > 0) {
            i -= 1;
            tiles[i].salt = 1;
            tiles[i].next = next_free;
            next_free = &tiles[i];
        }

        var salt_bits: u32 = 0;
        var tile_bits: u32 = 0;
        var poly_bits: u32 = 0;
        if (!polyref64) {
            tile_bits = common.ilog2(common.nextPow2(@bitCast(params.tile_count_max)));
            poly_bits = common.ilog2(common.nextPow2(@bitCast(params.poly_count_max)));
            // At most 31 salt bits: the salt mask is worked out in 32 bits.
            salt_bits = @min(31, 32 -| (tile_bits + poly_bits));
            if (salt_bits < 10) {
                gpa.free(position_lookup);
                gpa.free(tiles);
                return invalid_param;
            }
        }

        self.* = .{
            .gpa = gpa,
            .params = params.*,
            .origin = params.origin,
            .tile_width = params.tile_width,
            .tile_height = params.tile_height,
            .tile_count_max = params.tile_count_max,
            .tile_lookup_size = @intCast(lookup_size),
            .tile_lookup_mask = @intCast(lookup_size - 1),
            .position_lookup = position_lookup,
            .next_free = next_free,
            .tiles = tiles,
            .salt_bits = salt_bits,
            .tile_bits = tile_bits,
            .poly_bits = poly_bits,
        };
        return .success;
    }

    /// Readies an `empty` mesh that holds the one tile in `data`, with its
    /// bounds as the tile grid's cell, a millimetre where the tile has no
    /// extent. `data` and `flags` are as for `addTile`. On failure the mesh
    /// is left empty and the caller keeps `data`.
    pub fn initSingleTile(
        self: *NavMesh,
        gpa: std.mem.Allocator,
        data: []align(4) u8,
        flags: TileFlags,
    ) Status {
        if (data.len < @sizeOf(MeshHeader)) return invalid_param;
        const header: *const MeshHeader = @ptrCast(data.ptr);
        if (header.magic != nav_mesh_magic) return Status.failure.with(.{ .wrong_magic = true });
        if (header.version != nav_mesh_version) {
            return Status.failure.with(.{ .wrong_version = true });
        }

        // The bounds are checked only when the tile is added, so their
        // difference is taken without overflow here.
        const params: NavMeshParams = .{
            .origin = header.bounds_min,
            .tile_width = @max(1, header.bounds_max[0] -| header.bounds_min[0]),
            .tile_height = @max(1, header.bounds_max[2] -| header.bounds_min[2]),
            .tile_count_max = 1,
            .poly_count_max = header.poly_count,
        };

        const status = self.init(gpa, &params);
        if (status.failed) return status;

        const added = self.addTile(data, flags, 0, null);
        if (added.failed) {
            self.deinit();
            self.* = .empty;
        }
        return added;
    }

    /// Frees the tile table and the blobs the mesh owns. Every tile pointer
    /// and every reference into the mesh is dead after this.
    pub fn deinit(self: *NavMesh) void {
        for (self.tiles) |*tile| {
            if (tile.flags.free_data) self.gpa.free(tile.data);
        }
        self.gpa.free(self.position_lookup);
        self.gpa.free(self.tiles);
        self.* = undefined;
    }

    /// The parameters the mesh was readied with.
    pub fn getParams(self: *const NavMesh) *const NavMeshParams {
        return &self.params;
    }

    /// A reference from its parts. Parts wider than their bit counts spill
    /// into the neighbouring part.
    pub inline fn encodePolyId(
        self: *const NavMesh,
        salt: u32,
        tile_index: u32,
        poly_index: u32,
    ) PolyRef {
        if (polyref64) {
            return (@as(PolyRef, salt) << (poly_bits_64 + tile_bits_64)) |
                (@as(PolyRef, tile_index) << poly_bits_64) |
                @as(PolyRef, poly_index);
        }
        const tile_shift: RefShift = @intCast(self.poly_bits);
        const salt_shift: RefShift = @intCast(self.poly_bits + self.tile_bits);
        return (@as(PolyRef, salt) << salt_shift) |
            (@as(PolyRef, tile_index) << tile_shift) |
            @as(PolyRef, poly_index);
    }

    /// The parts of a reference. Nothing checks that they name a polygon
    /// the mesh holds; `isValidPolyRef` does.
    pub inline fn decodePolyId(self: *const NavMesh, ref: PolyRef) DecodedPolyId {
        return .{
            .salt = self.decodePolyIdSalt(ref),
            .tile_index = self.decodePolyIdTile(ref),
            .poly_index = self.decodePolyIdPoly(ref),
        };
    }

    /// The salt part of a reference.
    pub inline fn decodePolyIdSalt(self: *const NavMesh, ref: PolyRef) u32 {
        if (polyref64) {
            const salt_mask = (@as(PolyRef, 1) << salt_bits_64) - 1;
            return @intCast((ref >> (poly_bits_64 + tile_bits_64)) & salt_mask);
        }
        const salt_mask = (@as(PolyRef, 1) << @as(RefShift, @intCast(self.salt_bits))) - 1;
        const salt_shift: RefShift = @intCast(self.poly_bits + self.tile_bits);
        return (ref >> salt_shift) & salt_mask;
    }

    /// The tile index part of a reference.
    pub inline fn decodePolyIdTile(self: *const NavMesh, ref: PolyRef) u32 {
        if (polyref64) {
            const tile_mask = (@as(PolyRef, 1) << tile_bits_64) - 1;
            return @intCast((ref >> poly_bits_64) & tile_mask);
        }
        const tile_mask = (@as(PolyRef, 1) << @as(RefShift, @intCast(self.tile_bits))) - 1;
        return (ref >> @as(RefShift, @intCast(self.poly_bits))) & tile_mask;
    }

    /// The polygon index part of a reference.
    pub inline fn decodePolyIdPoly(self: *const NavMesh, ref: PolyRef) u32 {
        if (polyref64) {
            const poly_mask = (@as(PolyRef, 1) << poly_bits_64) - 1;
            return @intCast(ref & poly_mask);
        }
        const poly_mask = (@as(PolyRef, 1) << @as(RefShift, @intCast(self.poly_bits))) - 1;
        return ref & poly_mask;
    }

    /// The tile entry and polygon a reference names, or null when its tile
    /// index, salt or polygon index matches nothing the mesh holds.
    fn resolve(self: *const NavMesh, ref: PolyRef) ?struct {
        tile: *MeshTile,
        poly: *Poly,
        poly_index: u32,
    } {
        const id = self.decodePolyId(ref);
        if (id.tile_index >= self.tiles.len) return null;
        const tile = &self.tiles[id.tile_index];
        if (tile.salt != id.salt or tile.header == null) return null;
        if (id.poly_index >= tile.polys.len) return null;
        return .{ .tile = tile, .poly = &tile.polys[id.poly_index], .poly_index = id.poly_index };
    }

    /// Adds the tile in `data`, a blob as `convert.tileFromFloat` builds it,
    /// and links it to the tiles around it. The mesh reads and writes `data`
    /// until the tile is removed, so the blob must outlive the tile; with
    /// `flags.free_data` the mesh frees it with its own allocator, which
    /// must be the one that allocated it.
    ///
    /// `last_ref`, when not zero, asks for the tile entry and salt that
    /// reference names, so references saved before the tile was removed
    /// work again; the call fails with `out_of_memory` when that entry is in
    /// use. `result`, when given, receives the new tile's reference.
    ///
    /// Fails with `invalid_param` when the blob is shorter than its header
    /// says, or when an index in one of its blocks points outside the tile.
    /// It fails the same way when a vertex, a detail vertex, an end of an
    /// off-mesh connection or a bound of the header lies outside
    /// `common.coord_max`, when a walkable size or an off-mesh radius is
    /// negative or above it, or when the tile has a tree and a
    /// `bv_cell_size` below 1.
    pub fn addTile(
        self: *NavMesh,
        data: []align(4) u8,
        flags: TileFlags,
        last_ref: TileRef,
        result: ?*TileRef,
    ) Status {
        if (data.len < @sizeOf(MeshHeader)) return invalid_param;
        const header: *MeshHeader = @ptrCast(data.ptr);
        if (header.magic != nav_mesh_magic) return Status.failure.with(.{ .wrong_magic = true });
        if (header.version != nav_mesh_version) {
            return Status.failure.with(.{ .wrong_version = true });
        }

        const cast = std.math.cast;
        const vert_count = cast(usize, header.vert_count) orelse return invalid_param;
        const poly_count = cast(usize, header.poly_count) orelse return invalid_param;
        const link_count_max = cast(usize, header.link_count_max) orelse return invalid_param;
        const detail_mesh_count = cast(usize, header.detail_mesh_count) orelse
            return invalid_param;
        const detail_vert_count = cast(usize, header.detail_vert_count) orelse
            return invalid_param;
        const detail_tri_count = cast(usize, header.detail_tri_count) orelse
            return invalid_param;
        const bv_node_count = cast(usize, header.bv_node_count) orelse return invalid_param;
        const off_mesh_connection_count = cast(usize, header.off_mesh_connection_count) orelse
            return invalid_param;

        const header_size = common.align4(@sizeOf(MeshHeader));
        const verts_size = common.align4(@sizeOf([3]i32) * vert_count);
        const polys_size = common.align4(@sizeOf(Poly) * poly_count);
        const links_size = common.align4(@sizeOf(Link) * link_count_max);
        const detail_meshes_size = common.align4(@sizeOf(PolyDetail) * detail_mesh_count);
        const detail_verts_size = common.align4(@sizeOf([3]i32) * detail_vert_count);
        const detail_tris_size = common.align4(@sizeOf([4]u8) * detail_tri_count);
        const bv_tree_size = common.align4(@sizeOf(BvNode) * bv_node_count);
        const off_mesh_connections_size =
            common.align4(@sizeOf(OffMeshConnection) * off_mesh_connection_count);
        const blob_size = header_size + verts_size + polys_size + links_size +
            detail_meshes_size + detail_verts_size + detail_tris_size + bv_tree_size +
            off_mesh_connections_size;
        if (data.len < blob_size) return invalid_param;

        // The polygon bits of a reference must hold the tile's indices.
        if (polyref64) {
            if (poly_count > 1 << poly_bits_64) return invalid_param;
        } else {
            const poly_bits = common.ilog2(common.nextPow2(@bitCast(header.poly_count)));
            if (self.poly_bits < poly_bits) return invalid_param;
        }

        var offset = header_size;
        const verts = blobSlice([3]i32, data, offset, vert_count);
        offset += verts_size;
        const polys = blobSlice(Poly, data, offset, poly_count);
        offset += polys_size;
        const links = blobSlice(Link, data, offset, link_count_max);
        offset += links_size;
        const detail_meshes = blobSlice(PolyDetail, data, offset, detail_mesh_count);
        offset += detail_meshes_size;
        const detail_verts = blobSlice([3]i32, data, offset, detail_vert_count);
        offset += detail_verts_size;
        const detail_tris = blobSlice([4]u8, data, offset, detail_tri_count);
        offset += detail_tris_size;
        const bv_tree = blobSlice(BvNode, data, offset, bv_node_count);
        offset += bv_tree_size;
        const off_mesh_connections =
            blobSlice(OffMeshConnection, data, offset, off_mesh_connection_count);

        if (!indicesStayInside(
            header,
            verts.len,
            polys,
            detail_meshes,
            detail_verts.len,
            detail_tris,
            bv_tree,
            off_mesh_connections,
        )) return invalid_param;

        // Every product and difference the mesh and its queries work out
        // fits its type only for positions inside the coordinate bound and
        // sizes that are a count of millimetres within it.
        if (!common.inBounds(header.bounds_min) or !common.inBounds(header.bounds_max)) {
            return invalid_param;
        }
        const walkable_sizes: [3]i32 = .{
            header.walkable_height,
            header.walkable_radius,
            header.walkable_climb,
        };
        for (walkable_sizes) |size| {
            if (size < 0 or size > common.coord_max) return invalid_param;
        }
        // The tree's cells are found by dividing by their size.
        if (bv_node_count != 0 and header.bv_cell_size < 1) return invalid_param;
        for (verts) |vert| {
            if (!common.inBounds(vert)) return invalid_param;
        }
        for (detail_verts) |vert| {
            if (!common.inBounds(vert)) return invalid_param;
        }
        for (off_mesh_connections) |*connection| {
            if (!common.inBounds(connection.pos[0])) return invalid_param;
            if (!common.inBounds(connection.pos[1])) return invalid_param;
            if (connection.radius < 0 or connection.radius > common.coord_max) {
                return invalid_param;
            }
        }

        if (self.getTileAt(header.x, header.y, header.layer) != null) {
            return Status.failure.with(.{ .already_occupied = true });
        }

        // Take an unused entry: any, or the one `last_ref` names.
        var free_tile: ?*MeshTile = null;
        if (last_ref == 0) {
            if (self.next_free) |next_free| {
                free_tile = next_free;
                self.next_free = next_free.next;
                next_free.next = null;
            }
        } else {
            const tile_index = self.decodePolyIdTile(last_ref);
            if (tile_index >= self.tiles.len) return out_of_memory;
            const target = &self.tiles[tile_index];
            var prev: ?*MeshTile = null;
            free_tile = self.next_free;
            while (free_tile) |candidate| {
                if (candidate == target) break;
                prev = candidate;
                free_tile = candidate.next;
            }
            // The entry is in use.
            if (free_tile != target) return out_of_memory;
            if (prev) |p| p.next = target.next else self.next_free = target.next;

            target.salt = self.decodePolyIdSalt(last_ref);
        }
        const tile = free_tile orelse return out_of_memory;

        const h = computeTileHash(header.x, header.y, self.tile_lookup_mask);
        tile.next = self.position_lookup[h];
        self.position_lookup[h] = tile;

        tile.verts = verts;
        tile.polys = polys;
        tile.links = links;
        tile.detail_meshes = detail_meshes;
        tile.detail_verts = detail_verts;
        tile.detail_tris = detail_tris;
        tile.bv_tree = bv_tree;
        tile.off_mesh_connections = off_mesh_connections;

        // Every link starts on the free list.
        if (link_count_max == 0) {
            tile.links_free_list = null_link;
        } else {
            tile.links_free_list = 0;
            tile.links[link_count_max - 1].next = null_link;
            for (0..link_count_max - 1) |i| tile.links[i].next = @intCast(i + 1);
        }

        tile.header = header;
        tile.data = data;
        tile.flags = flags;

        self.connectIntLinks(tile);

        // Base the tile's off-mesh connections on their start polygons, and
        // land the ones that end inside the tile.
        self.baseOffMeshLinks(tile);
        self.connectExtOffMeshLinks(tile, tile, -1);

        const neighbour_count_max = 32;
        var neighbours: [neighbour_count_max]*MeshTile = undefined;

        // Connect with the other layers of the same grid cell.
        var neighbour_count = self.getTilesAtMutable(header.x, header.y, &neighbours);
        for (neighbours[0..neighbour_count]) |neighbour| {
            if (neighbour == tile) continue;

            self.connectExtLinks(tile, neighbour, -1);
            self.connectExtLinks(neighbour, tile, -1);
            self.connectExtOffMeshLinks(tile, neighbour, -1);
            self.connectExtOffMeshLinks(neighbour, tile, -1);
        }

        // Connect with the tiles of the eight cells around.
        var side: i32 = 0;
        while (side < 8) : (side += 1) {
            neighbour_count = self.getNeighbourTilesAt(header.x, header.y, side, &neighbours);
            for (neighbours[0..neighbour_count]) |neighbour| {
                self.connectExtLinks(tile, neighbour, side);
                self.connectExtLinks(neighbour, tile, common.oppositeTile(side));
                self.connectExtOffMeshLinks(tile, neighbour, side);
                self.connectExtOffMeshLinks(neighbour, tile, common.oppositeTile(side));
            }
        }

        if (result) |r| r.* = self.getTileRef(tile);

        return .success;
    }

    /// Removes the tile `ref` names and unlinks it from the tiles around
    /// it. References into the tile are invalid from here on, and its
    /// `MeshTile` entry is empty until a later `addTile` reuses it.
    ///
    /// `data`, when given, receives the tile's blob, now the caller's
    /// again; an empty slice when the mesh owned the blob and freed it.
    pub fn removeTile(self: *NavMesh, ref: TileRef, data: ?*[]align(4) u8) Status {
        if (ref == 0) return invalid_param;
        const tile_index = self.decodePolyIdTile(ref);
        const tile_salt = self.decodePolyIdSalt(ref);
        if (tile_index >= self.tiles.len) return invalid_param;
        const tile = &self.tiles[tile_index];
        if (tile.salt != tile_salt) return invalid_param;
        // A reference to an unused entry, which keeps its salt, names no tile.
        const header = tile.header orelse return invalid_param;

        // Remove the tile from the position lookup.
        const h = computeTileHash(header.x, header.y, self.tile_lookup_mask);
        var prev: ?*MeshTile = null;
        var cur = self.position_lookup[h];
        while (cur) |c| {
            if (c == tile) {
                if (prev) |p| p.next = c.next else self.position_lookup[h] = c.next;
                break;
            }
            prev = c;
            cur = c.next;
        }

        const neighbour_count_max = 32;
        var neighbours: [neighbour_count_max]*MeshTile = undefined;

        // Disconnect from the other layers of the same grid cell.
        var neighbour_count = self.getTilesAtMutable(header.x, header.y, &neighbours);
        for (neighbours[0..neighbour_count]) |neighbour| {
            if (neighbour == tile) continue;
            self.unconnectLinks(neighbour, tile);
        }

        // Disconnect from the tiles of the eight cells around.
        var side: i32 = 0;
        while (side < 8) : (side += 1) {
            neighbour_count = self.getNeighbourTilesAt(header.x, header.y, side, &neighbours);
            for (neighbours[0..neighbour_count]) |neighbour| self.unconnectLinks(neighbour, tile);
        }

        if (tile.flags.free_data) {
            self.gpa.free(tile.data);
            if (data) |d| d.* = &.{};
        } else {
            if (data) |d| d.* = tile.data;
        }

        const salt = tile.salt;
        tile.* = .unused;

        // The next salt, never zero, so old references to the entry fail.
        const salt_bits: u5 = if (polyref64) salt_bits_64 else @intCast(self.salt_bits);
        tile.salt = (salt +% 1) & ((@as(u32, 1) << salt_bits) - 1);
        if (tile.salt == 0) tile.salt += 1;

        tile.next = self.next_free;
        self.next_free = tile;

        return .success;
    }

    /// The grid cell that holds `pos`: per axis the offset from the origin
    /// divided by the tile's size, rounded down.
    pub fn calcTileLoc(self: *const NavMesh, pos: [3]i32) TileLocation {
        // A corner of a query's box can lie outside the coordinate bound,
        // so the offset is taken in 64 bits and a cell that 32 bits cannot
        // hold is held at the nearest one they can.
        const x = @divFloor(@as(i64, pos[0]) - self.origin[0], self.tile_width);
        const y = @divFloor(@as(i64, pos[2]) - self.origin[2], self.tile_height);
        const cell_min = std.math.minInt(i32);
        const cell_max = std.math.maxInt(i32);
        return .{
            .x = @intCast(std.math.clamp(x, cell_min, cell_max)),
            .y = @intCast(std.math.clamp(y, cell_min, cell_max)),
        };
    }

    /// The first tile of the lookup bucket for a grid position; null on a mesh
    /// with no tables.
    inline fn firstTileAt(self: *const NavMesh, x: i32, y: i32) ?*MeshTile {
        if (self.position_lookup.len == 0) return null;
        return self.position_lookup[computeTileHash(x, y, self.tile_lookup_mask)];
    }

    /// The tile at a grid position and layer, or null. The pointer goes
    /// stale as a `MeshTile` does.
    pub fn getTileAt(self: *const NavMesh, x: i32, y: i32, layer: i32) ?*const MeshTile {
        var tile = self.firstTileAt(x, y);
        while (tile) |t| : (tile = t.next) {
            const header = t.header orelse continue;
            if (header.x == x and header.y == y and header.layer == layer) return t;
        }
        return null;
    }

    /// Every layer's tile at a grid position, as many as `tiles` holds.
    /// Returns how many were written. The pointers go stale as a `MeshTile`
    /// does.
    pub fn getTilesAt(self: *const NavMesh, x: i32, y: i32, tiles: []*const MeshTile) usize {
        var n: usize = 0;
        var tile = self.firstTileAt(x, y);
        while (tile) |t| : (tile = t.next) {
            const header = t.header orelse continue;
            if (header.x == x and header.y == y) {
                if (n < tiles.len) {
                    tiles[n] = t;
                    n += 1;
                }
            }
        }
        return n;
    }

    fn getTilesAtMutable(self: *const NavMesh, x: i32, y: i32, tiles: []*MeshTile) usize {
        var n: usize = 0;
        var tile = self.firstTileAt(x, y);
        while (tile) |t| : (tile = t.next) {
            const header = t.header orelse continue;
            if (header.x == x and header.y == y) {
                if (n < tiles.len) {
                    tiles[n] = t;
                    n += 1;
                }
            }
        }
        return n;
    }

    /// The tiles of the cell next to `x`, `y` on `side`, sides numbered
    /// 0..7 counter-clockwise from +x.
    fn getNeighbourTilesAt(
        self: *const NavMesh,
        x: i32,
        y: i32,
        side: i32,
        tiles: []*MeshTile,
    ) usize {
        var nx = x;
        var ny = y;
        switch (side) {
            0 => nx +%= 1,
            1 => {
                nx +%= 1;
                ny +%= 1;
            },
            2 => ny +%= 1,
            3 => {
                nx -%= 1;
                ny +%= 1;
            },
            4 => nx -%= 1,
            5 => {
                nx -%= 1;
                ny -%= 1;
            },
            6 => ny -%= 1,
            7 => {
                nx +%= 1;
                ny -%= 1;
            },
            else => {},
        }
        return self.getTilesAtMutable(nx, ny, tiles);
    }

    /// The reference of the tile at a grid position and layer, or zero.
    pub fn getTileRefAt(self: *const NavMesh, x: i32, y: i32, layer: i32) TileRef {
        return self.getTileRef(self.getTileAt(x, y, layer));
    }

    /// The reference of a tile entry of this mesh; zero for null.
    pub fn getTileRef(self: *const NavMesh, tile: ?*const MeshTile) TileRef {
        const t = tile orelse return 0;
        const offset = @intFromPtr(t) - @intFromPtr(self.tiles.ptr);
        return self.encodePolyId(t.salt, @intCast(offset / @sizeOf(MeshTile)), 0);
    }

    /// The tile entry a reference names, or null when the reference is zero
    /// or stale. An unused entry is returned for a reference that still
    /// carries its salt; its `header` is null. The pointer goes stale as a
    /// `MeshTile` does.
    pub fn getTileByRef(self: *const NavMesh, ref: TileRef) ?*const MeshTile {
        if (ref == 0) return null;
        const tile_index = self.decodePolyIdTile(ref);
        const tile_salt = self.decodePolyIdSalt(ref);
        if (tile_index >= self.tiles.len) return null;
        const tile = &self.tiles[tile_index];
        if (tile.salt != tile_salt) return null;
        return tile;
    }

    /// How many tile entries the mesh has, used or not.
    pub fn getMaxTiles(self: *const NavMesh) i32 {
        return self.tile_count_max;
    }

    /// The tile entry at an index below `getMaxTiles`. Its `header` is null
    /// while it is unused. The pointer goes stale as a `MeshTile` does.
    pub fn getTile(self: *const NavMesh, i: i32) *const MeshTile {
        return &self.tiles[@intCast(i)];
    }

    /// The tile and polygon a reference names, through `tile` and `poly`,
    /// which are left alone on failure. The pointers go stale as a
    /// `MeshTile` does.
    pub fn getTileAndPolyByRef(
        self: *const NavMesh,
        ref: PolyRef,
        tile: **const MeshTile,
        poly: **const Poly,
    ) Status {
        if (ref == 0) return .failure;
        const found = self.resolve(ref) orelse return invalid_param;
        tile.* = found.tile;
        poly.* = found.poly;
        return .success;
    }

    /// The tile and polygon of a reference known to be valid: nothing is
    /// checked beyond the bounds of the tile table and the tile's polygons.
    /// The pointers go stale as a `MeshTile` does.
    pub inline fn getTileAndPolyByRefUnsafe(self: *const NavMesh, ref: PolyRef) TileAndPoly {
        const id = self.decodePolyId(ref);
        const tile = &self.tiles[id.tile_index];
        return .{ .tile = tile, .poly = &tile.polys[id.poly_index] };
    }

    /// Whether a reference names a polygon the mesh holds now.
    pub fn isValidPolyRef(self: *const NavMesh, ref: PolyRef) bool {
        if (ref == 0) return false;
        return self.resolve(ref) != null;
    }

    /// The reference of a tile's polygon zero; or it with a polygon index
    /// for that polygon's reference. Zero for null.
    pub fn getPolyRefBase(self: *const NavMesh, tile: ?*const MeshTile) PolyRef {
        const t = tile orelse return 0;
        const offset = @intFromPtr(t) - @intFromPtr(self.tiles.ptr);
        return self.encodePolyId(t.salt, @intCast(offset / @sizeOf(MeshTile)), 0);
    }

    /// The two ends of an off-mesh connection in the order an agent coming
    /// from `prev_ref` walks them.
    pub fn getOffMeshConnectionPolyEndPoints(
        self: *const NavMesh,
        prev_ref: PolyRef,
        poly_ref: PolyRef,
        start_pos: *[3]i32,
        end_pos: *[3]i32,
    ) Status {
        if (poly_ref == 0) return .failure;
        const found = self.resolve(poly_ref) orelse return invalid_param;
        const tile = found.tile;
        const poly = found.poly;

        if (poly.getType() != .off_mesh_connection) return .failure;

        // The connection starts at vertex 0 unless the link over that
        // vertex leads somewhere other than where the agent comes from.
        var start: usize = 0;
        var end: usize = 1;
        var i = poly.first_link;
        while (i != null_link) : (i = tile.links[i].next) {
            if (tile.links[i].edge == 0) {
                if (tile.links[i].ref != prev_ref) {
                    start = 1;
                    end = 0;
                }
                break;
            }
        }

        start_pos.* = tile.verts[poly.verts[start]];
        end_pos.* = tile.verts[poly.verts[end]];

        return .success;
    }

    /// The off-mesh connection a polygon reference stands for, or null when
    /// the reference is invalid or names a ground polygon. The pointer goes
    /// stale as a `MeshTile` does.
    pub fn getOffMeshConnectionByRef(self: *const NavMesh, ref: PolyRef) ?*const OffMeshConnection {
        if (ref == 0) return null;
        const found = self.resolve(ref) orelse return null;
        if (found.poly.getType() != .off_mesh_connection) return null;

        const off_mesh_base: u32 = @bitCast(found.tile.header.?.off_mesh_base);
        const index = found.poly_index -% off_mesh_base;
        std.debug.assert(index < found.tile.off_mesh_connections.len);
        return &found.tile.off_mesh_connections[index];
    }

    /// Sets the flags of the polygon `ref` names.
    pub fn setPolyFlags(self: *NavMesh, ref: PolyRef, flags: u16) Status {
        if (ref == 0) return .failure;
        const found = self.resolve(ref) orelse return invalid_param;
        found.poly.flags = flags;
        return .success;
    }

    /// Reads the flags of the polygon `ref` names into `result_flags`.
    pub fn getPolyFlags(self: *const NavMesh, ref: PolyRef, result_flags: *u16) Status {
        if (ref == 0) return .failure;
        const found = self.resolve(ref) orelse return invalid_param;
        result_flags.* = found.poly.flags;
        return .success;
    }

    /// Sets the area id of the polygon `ref` names; only the low six bits
    /// of `area` are kept.
    pub fn setPolyArea(self: *NavMesh, ref: PolyRef, area: u8) Status {
        if (ref == 0) return .failure;
        const found = self.resolve(ref) orelse return invalid_param;
        found.poly.setArea(area);
        return .success;
    }

    /// Reads the area id of the polygon `ref` names into `result_area`.
    pub fn getPolyArea(self: *const NavMesh, ref: PolyRef, result_area: *u8) Status {
        if (ref == 0) return .failure;
        const found = self.resolve(ref) orelse return invalid_param;
        result_area.* = found.poly.getArea();
        return .success;
    }

    /// The bytes `storeTileState` needs for a tile; zero for null.
    pub fn getTileStateSize(tile: ?*const MeshTile) usize {
        const t = tile orelse return 0;
        const header_size = common.align4(@sizeOf(TileStateHeader));
        const poly_state_size = common.align4(@sizeOf(PolyStateRecord) * t.polys.len);
        return header_size + poly_state_size;
    }

    /// Writes the flags and area id of every polygon of a tile to `data`,
    /// the part of a tile that changes after it is built. The fourth byte
    /// of each polygon's record is left as `data` had it.
    pub fn storeTileState(self: *const NavMesh, tile: *const MeshTile, data: []u8) Status {
        const size_required = getTileStateSize(tile);
        if (data.len < size_required) return Status.failure.with(.{ .buffer_too_small = true });

        const tile_state: *align(1) TileStateHeader = @ptrCast(data.ptr);
        const poly_states: [*]align(1) PolyStateRecord =
            @ptrCast(data.ptr + common.align4(@sizeOf(TileStateHeader)));

        tile_state.magic = nav_mesh_state_magic;
        tile_state.version = nav_mesh_state_version;
        tile_state.ref = self.getTileRef(tile);

        for (tile.polys, 0..) |*poly, i| {
            const poly_state = &poly_states[i];
            poly_state.flags = poly.flags;
            poly_state.area = poly.getArea();
        }

        return .success;
    }

    /// Sets the flags and area id of every polygon of a tile from a state
    /// `storeTileState` wrote for the same tile reference.
    pub fn restoreTileState(self: *NavMesh, tile: *const MeshTile, data: []const u8) Status {
        const size_required = getTileStateSize(tile);
        if (data.len < size_required) return invalid_param;

        const tile_state: *align(1) const TileStateHeader = @ptrCast(data.ptr);
        const poly_states: [*]align(1) const PolyStateRecord =
            @ptrCast(data.ptr + common.align4(@sizeOf(TileStateHeader)));

        if (tile_state.magic != nav_mesh_state_magic) {
            return Status.failure.with(.{ .wrong_magic = true });
        }
        if (tile_state.version != nav_mesh_state_version) {
            return Status.failure.with(.{ .wrong_version = true });
        }
        if (tile_state.ref != self.getTileRef(tile)) return invalid_param;

        for (tile.polys, 0..) |*poly, i| {
            const poly_state = &poly_states[i];
            poly.flags = poly_state.flags;
            poly.setArea(poly_state.area);
        }

        return .success;
    }

    /// The polygons of `tile` with an edge on the border `side` that
    /// overlaps the edge `va`, `vb`, and the stretch of the border each
    /// shares with it. Returns how many were written, at most
    /// `connections.len`.
    fn findConnectingPolys(
        self: *const NavMesh,
        va: [3]i32,
        vb: [3]i32,
        tile: ?*const MeshTile,
        side: i32,
        connections: []PolyRef,
        connection_areas: [][2]i32,
    ) usize {
        const t = tile orelse return 0;

        const a = calcSlabEndPoints(va, vb, side);
        const a_coord = getSlabCoord(va, side);

        // Remove links pointing to 'side' and compact the links array.
        const m: u16 = ext_link | @as(u16, @truncate(@as(u32, @bitCast(side))));
        var n: usize = 0;

        const base = self.getPolyRefBase(t);
        const walkable_climb = t.header.?.walkable_climb;

        for (t.polys, 0..) |*poly, i| {
            const vert_count: usize = poly.vert_count;
            for (0..vert_count) |j| {
                // Skip edges which do not point to the right side.
                if (poly.neighbours[j] != m) continue;

                const vc = t.verts[poly.verts[j]];
                const vd = t.verts[poly.verts[(j + 1) % vert_count]];
                const b_coord = getSlabCoord(vc, side);

                // Segments are not close enough: more than 10 millimetres
                // apart across the border.
                if (@abs(a_coord - b_coord) > 10) continue;

                // Check if the segments touch, each pulled in by 10
                // millimetres at its ends.
                const b = calcSlabEndPoints(vc, vd, side);

                if (!overlapSlabs(a, b, 10, walkable_climb)) continue;

                // Add return value.
                if (n < connections.len) {
                    connection_areas[n] = .{
                        @max(a.min[0], b.min[0]),
                        @min(a.max[0], b.max[0]),
                    };
                    connections[n] = base | @as(PolyRef, @intCast(i));
                    n += 1;
                }
                break;
            }
        }
        return n;
    }

    /// Frees every link of `tile` that leads into `target`.
    fn unconnectLinks(self: *const NavMesh, tile: *MeshTile, target: *const MeshTile) void {
        const target_index = self.decodePolyIdTile(self.getTileRef(target));

        for (tile.polys) |*poly| {
            var j = poly.first_link;
            var pj = null_link;
            while (j != null_link) {
                if (self.decodePolyIdTile(tile.links[j].ref) == target_index) {
                    // Remove link.
                    const nj = tile.links[j].next;
                    if (pj == null_link) {
                        poly.first_link = nj;
                    } else {
                        tile.links[pj].next = nj;
                    }
                    freeLink(tile, j);
                    j = nj;
                } else {
                    // Advance
                    pj = j;
                    j = tile.links[j].next;
                }
            }
        }
    }

    /// Links the polygons on `tile`'s border facing `side`, or on every
    /// border for -1, to the polygons of `target` across it.
    fn connectExtLinks(
        self: *const NavMesh,
        tile: *MeshTile,
        target: *const MeshTile,
        side: i32,
    ) void {
        for (tile.polys) |*poly| {
            const vert_count: usize = poly.vert_count;
            for (0..vert_count) |j| {
                // Skip non-portal edges.
                if ((poly.neighbours[j] & ext_link) == 0) continue;

                const dir: i32 = poly.neighbours[j] & 0xff;
                if (side != -1 and dir != side) continue;

                // Create new links
                const va = tile.verts[poly.verts[j]];
                const vb = tile.verts[poly.verts[(j + 1) % vert_count]];
                var neighbours: [4]PolyRef = undefined;
                var neighbour_areas: [4][2]i32 = undefined;
                const neighbour_count = self.findConnectingPolys(
                    va,
                    vb,
                    target,
                    common.oppositeTile(dir),
                    &neighbours,
                    &neighbour_areas,
                );
                for (0..neighbour_count) |k| {
                    const index = allocLink(tile);
                    if (index == null_link) continue;

                    const link = &tile.links[index];
                    link.ref = neighbours[k];
                    link.edge = @intCast(j);
                    link.side = @intCast(dir);

                    link.next = poly.first_link;
                    poly.first_link = index;

                    // Compress portal limits to a byte value. The two
                    // fractions share a denominator, made positive, so
                    // their numerators order them.
                    if (dir == 0 or dir == 4) {
                        const edge_length = vb[2] - va[2];
                        var tmin: Fraction = .init(neighbour_areas[k][0] - va[2], edge_length);
                        var tmax: Fraction = .init(neighbour_areas[k][1] - va[2], edge_length);
                        if (tmin.num > tmax.num) std.mem.swap(Fraction, &tmin, &tmax);
                        link.bounds_min = portalLimit(tmin);
                        link.bounds_max = portalLimit(tmax);
                    } else if (dir == 2 or dir == 6) {
                        const edge_length = vb[0] - va[0];
                        var tmin: Fraction = .init(neighbour_areas[k][0] - va[0], edge_length);
                        var tmax: Fraction = .init(neighbour_areas[k][1] - va[0], edge_length);
                        if (tmin.num > tmax.num) std.mem.swap(Fraction, &tmin, &tmax);
                        link.bounds_min = portalLimit(tmin);
                        link.bounds_max = portalLimit(tmax);
                    }
                }
            }
        }
    }

    /// A fraction along a border edge as a `Link` stores it: held to 0..1
    /// and rounded to the nearest 255th, a half upward. An edge with no
    /// length along the border has a zero denominator, and gives 0 for a
    /// numerator at or below zero and 255 above.
    inline fn portalLimit(t: Fraction) u8 {
        if (t.num <= 0) return 0;
        if (t.num >= t.den) return 255;
        return @intCast(common.divRound(@as(i128, t.num) * 255, t.den));
    }

    /// Lands the off-mesh connections of `target` that leave it through the
    /// side facing `tile` on `tile`'s polygons; for -1, the ones that stay
    /// inside their tile.
    fn connectExtOffMeshLinks(
        self: *const NavMesh,
        tile: *MeshTile,
        target: *MeshTile,
        side: i32,
    ) void {
        // We are interested on links which land from target tile to this tile.
        const opposite_side: u8 = if (side == -1) 0xff else @intCast(common.oppositeTile(side));
        const walkable_climb = target.header.?.walkable_climb;

        for (target.off_mesh_connections) |*target_connection| {
            if (target_connection.side != opposite_side) continue;

            const target_poly = &target.polys[target_connection.poly];
            // Skip off-mesh connections which start location could not be connected at all.
            if (target_poly.first_link == null_link) continue;

            const radius = target_connection.radius;
            const half_extents: [3]i32 = .{ radius, walkable_climb, radius };

            // Find polygon to connect to.
            const p = target_connection.pos[1];
            var nearest_pt: [3]i32 = undefined;
            const ref = self.findNearestPolyInTile(tile, p, half_extents, &nearest_pt);
            if (ref == 0) continue;
            // findNearestPoly may return too optimistic results, further check to make sure.
            const radius_wide: u64 = @intCast(radius);
            if (common.vdist2dSqr(nearest_pt, p) > radius_wide * radius_wide) continue;
            // Make sure the location is on current mesh.
            target.verts[target_poly.verts[1]] = nearest_pt;

            // Link off-mesh connection to target poly.
            const index = allocLink(target);
            if (index != null_link) {
                const link = &target.links[index];
                link.ref = ref;
                link.edge = 1;
                link.side = opposite_side;
                link.bounds_min = 0;
                link.bounds_max = 0;
                // Add to linked list.
                link.next = target_poly.first_link;
                target_poly.first_link = index;
            }

            // Link target poly to off-mesh connection.
            if ((target_connection.flags & off_mesh_connection_bidirectional) != 0) {
                const land_index = allocLink(tile);
                if (land_index != null_link) {
                    const land_poly_index: u16 = @truncate(self.decodePolyIdPoly(ref));
                    const land_poly = &tile.polys[land_poly_index];
                    const link = &tile.links[land_index];
                    link.ref = self.getPolyRefBase(target) | @as(PolyRef, target_connection.poly);
                    link.edge = 0xff;
                    link.side = if (side == -1) 0xff else @intCast(side);
                    link.bounds_min = 0;
                    link.bounds_max = 0;
                    // Add to linked list.
                    link.next = land_poly.first_link;
                    land_poly.first_link = land_index;
                }
            }
        }
    }

    /// Links the polygons of `tile` to their neighbours inside the tile.
    fn connectIntLinks(self: *const NavMesh, tile: *MeshTile) void {
        const base = self.getPolyRefBase(tile);

        for (tile.polys) |*poly| {
            poly.first_link = null_link;

            if (poly.getType() == .off_mesh_connection) continue;

            // Build edge links backwards so that the links will be
            // in the linked list from lowest index to highest.
            var j: usize = poly.vert_count;
            while (j > 0) {
                j -= 1;
                // Skip hard and non-internal edges.
                if (poly.neighbours[j] == 0 or (poly.neighbours[j] & ext_link) != 0) continue;

                const index = allocLink(tile);
                if (index != null_link) {
                    const link = &tile.links[index];
                    link.ref = base | @as(PolyRef, poly.neighbours[j] - 1);
                    link.edge = @intCast(j);
                    link.side = 0xff;
                    link.bounds_min = 0;
                    link.bounds_max = 0;
                    // Add to linked list.
                    link.next = poly.first_link;
                    poly.first_link = index;
                }
            }
        }
    }

    /// Bases each off-mesh connection of `tile` on the polygon under its
    /// start, and moves the start vertex onto that polygon.
    fn baseOffMeshLinks(self: *const NavMesh, tile: *MeshTile) void {
        const base = self.getPolyRefBase(tile);
        const walkable_climb = tile.header.?.walkable_climb;

        for (tile.off_mesh_connections) |*connection| {
            const poly = &tile.polys[connection.poly];

            const radius = connection.radius;
            const half_extents: [3]i32 = .{ radius, walkable_climb, radius };

            // Find polygon to connect to.
            const p = connection.pos[0];
            var nearest_pt: [3]i32 = undefined;
            const ref = self.findNearestPolyInTile(tile, p, half_extents, &nearest_pt);
            if (ref == 0) continue;
            // findNearestPoly may return too optimistic results, further check to make sure.
            const radius_wide: u64 = @intCast(radius);
            if (common.vdist2dSqr(nearest_pt, p) > radius_wide * radius_wide) continue;
            // Make sure the location is on current mesh.
            tile.verts[poly.verts[0]] = nearest_pt;

            // Link off-mesh connection to target poly.
            const index = allocLink(tile);
            if (index != null_link) {
                const link = &tile.links[index];
                link.ref = ref;
                link.edge = 0;
                link.side = 0xff;
                link.bounds_min = 0;
                link.bounds_max = 0;
                // Add to linked list.
                link.next = poly.first_link;
                poly.first_link = index;
            }

            // Start end-point is always connect back to off-mesh connection.
            const land_index = allocLink(tile);
            if (land_index != null_link) {
                const land_poly_index: u16 = @truncate(self.decodePolyIdPoly(ref));
                const land_poly = &tile.polys[land_poly_index];
                const link = &tile.links[land_index];
                link.ref = base | @as(PolyRef, connection.poly);
                link.edge = 0xff;
                link.side = 0xff;
                link.bounds_min = 0;
                link.bounds_max = 0;
                // Add to linked list.
                link.next = land_poly.first_link;
                land_poly.first_link = land_index;
            }
        }
    }

    /// The point of the polygon `ref` names nearest to `pos`, rounded to
    /// the nearest millimetre. `ref` must be valid.
    pub fn closestPointOnPoly(self: *const NavMesh, ref: PolyRef, pos: [3]i32) ClosestPointOnPoly {
        const found = self.getTileAndPolyByRefUnsafe(ref);
        const tile = found.tile;
        const poly = found.poly;

        if (tile.getPolyHeight(poly, pos)) |height| {
            return .{ .closest = .{ pos[0], height, pos[2] }, .pos_over_poly = true };
        }

        // Off-mesh connections don't have detail polygons.
        if (poly.getType() == .off_mesh_connection) {
            const v0 = tile.verts[poly.verts[0]];
            const v1 = tile.verts[poly.verts[1]];
            const distance = common.distancePointSegmentSqr2d(pos, v0, v1);
            return .{ .closest = common.lerp(v0, v1, distance.t), .pos_over_poly = false };
        }

        // Outside poly that is not an offmesh connection.
        return .{
            .closest = closestPointOnDetailEdges(tile, poly, pos, true),
            .pos_over_poly = false,
        };
    }

    /// The polygon of `tile` nearest to `center` among those whose bounds
    /// touch the box `center` ± `half_extents`, or zero. `nearest_pt`
    /// receives the nearest point on it and is left alone when there is
    /// none.
    fn findNearestPolyInTile(
        self: *const NavMesh,
        tile: *const MeshTile,
        center: [3]i32,
        half_extents: [3]i32,
        nearest_pt: *[3]i32,
    ) PolyRef {
        const bounds_min = common.vsub(center, half_extents);
        const bounds_max = common.vadd(center, half_extents);

        // Get nearby polygons from proximity grid.
        var polys: [128]PolyRef = undefined;
        const poly_count = self.queryPolygonsInTile(tile, bounds_min, bounds_max, &polys);

        // Find nearest polygon amongst the nearby polygons.
        var nearest: PolyRef = 0;
        var nearest_distance_sqr: u64 = std.math.maxInt(u64);
        const walkable_climb = tile.header.?.walkable_climb;
        for (polys[0..poly_count]) |ref| {
            const point = self.closestPointOnPoly(ref, center);

            // If a point is directly over a polygon and closer than
            // climb height, favor that instead of straight line nearest point.
            const diff = common.vsub(center, point.closest);
            var d: u64 = undefined;
            if (point.pos_over_poly) {
                const over = @as(i64, @abs(diff[1])) - walkable_climb;
                d = if (over > 0) @intCast(over * over) else 0;
            } else {
                d = common.vlenSqr(diff);
            }

            if (d < nearest_distance_sqr) {
                nearest_pt.* = point.closest;
                nearest_distance_sqr = d;
                nearest = ref;
            }
        }

        return nearest;
    }

    /// The polygons of `tile` whose bounds touch the box `qmin`..`qmax`, as
    /// many as `polys` holds. Returns how many were written.
    fn queryPolygonsInTile(
        self: *const NavMesh,
        tile: *const MeshTile,
        qmin: [3]i32,
        qmax: [3]i32,
        polys: []PolyRef,
    ) usize {
        const base = self.getPolyRefBase(tile);
        var n: usize = 0;

        if (tile.bv_tree.len != 0) {
            const header = tile.header.?;
            const tbmin = header.bounds_min;
            const tbmax = header.bounds_max;
            const cell_size = header.bv_cell_size;

            // The query box held to the tile's, measured from the tile's
            // minimum corner in the tree's cells; the minimum rounded down
            // to even and the maximum, a cell further, up to odd. A cell
            // past the tree's last is held at the last.
            const bounds_min: [3]u16 = .{
                treeCell(qmin[0], tbmin[0], tbmax[0], cell_size) & 0xfffe,
                treeCell(qmin[1], tbmin[1], tbmax[1], cell_size) & 0xfffe,
                treeCell(qmin[2], tbmin[2], tbmax[2], cell_size) & 0xfffe,
            };
            const bounds_max: [3]u16 = .{
                (treeCell(qmax[0], tbmin[0], tbmax[0], cell_size) +| 1) | 1,
                (treeCell(qmax[1], tbmin[1], tbmax[1], cell_size) +| 1) | 1,
                (treeCell(qmax[2], tbmin[2], tbmax[2], cell_size) +| 1) | 1,
            };

            // Traverse tree
            var node_index: usize = 0;
            while (node_index < tile.bv_tree.len) {
                const node = &tile.bv_tree[node_index];
                const overlap = common.overlapQuantBounds(
                    bounds_min,
                    bounds_max,
                    node.bounds_min,
                    node.bounds_max,
                );
                const is_leaf_node = node.index >= 0;

                if (is_leaf_node and overlap) {
                    if (n < polys.len) {
                        polys[n] = base | @as(PolyRef, @intCast(node.index));
                        n += 1;
                    }
                }

                if (overlap or is_leaf_node) {
                    node_index += 1;
                } else {
                    const escape_index: usize = @intCast(-node.index);
                    node_index += escape_index;
                }
            }

            return n;
        }

        for (tile.polys, 0..) |*poly, i| {
            // Do not return off-mesh connection polygons.
            if (poly.getType() == .off_mesh_connection) continue;
            // Calc polygon bounds.
            var v = tile.verts[poly.verts[0]];
            var bounds_min = v;
            var bounds_max = v;
            var j: usize = 1;
            while (j < poly.vert_count) : (j += 1) {
                v = tile.verts[poly.verts[j]];
                bounds_min = common.vmin(bounds_min, v);
                bounds_max = common.vmax(bounds_max, v);
            }
            if (common.overlapBounds(qmin, qmax, bounds_min, bounds_max)) {
                if (n < polys.len) {
                    polys[n] = base | @as(PolyRef, @intCast(i));
                    n += 1;
                }
            }
        }
        return n;
    }
};
