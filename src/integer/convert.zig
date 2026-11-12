//! Builds integer tiles from float tiles.
//!
//! The one file of the integer variant that reads a floating-point number.
//! `tileFromFloat` takes a float tile, the blob the float builder's
//! `createNavMeshData` makes, and gives the tile this variant's
//! `NavMesh.addTile` loads. The two have the same layout and size: every
//! position and length becomes a whole number of integer units, and the
//! boxes of the bounding-volume tree are recomputed from the integer
//! geometry.
//!
//! The integer unit is the millimetre, and `scale` is the number of
//! millimetres in one float unit: 1000 for a mesh in metres, 10 for one in
//! centimetres. The variant's fixed distances are in millimetres at every
//! scale. A value becomes
//! `round(f64(value) * scale)`, which is the same number on every machine,
//! so a float tile converts to the same bytes everywhere.

const std = @import("std");
const common = @import("common.zig");
const nav_mesh = @import("nav_mesh.zig");
const float_nav_mesh = @import("../nav_mesh.zig");

const Poly = nav_mesh.Poly;
const PolyDetail = nav_mesh.PolyDetail;
const Link = nav_mesh.Link;
const BvNode = nav_mesh.BvNode;

/// The scale of a float mesh in metres: millimetres per metre.
pub const default_scale: f64 = 1000;

/// Why a float tile did not become an integer tile.
pub const ConvertError = error{
    OutOfMemory,
    /// The data is not a float tile: it is too short for its header or for
    /// the counts the header gives, has another magic number or version, or
    /// its tree names a node, polygon or vertex the tile does not hold.
    InvalidTile,
    /// A value is not a number, is infinite or lands outside
    /// `common.coord_max`, or `scale` is not a positive finite number.
    OutOfBounds,
};

comptime {
    // An integer tile is laid out as the float tile it is copied from.
    std.debug.assert(@sizeOf(nav_mesh.MeshHeader) == @sizeOf(float_nav_mesh.MeshHeader));
    std.debug.assert(
        @sizeOf(nav_mesh.OffMeshConnection) == @sizeOf(float_nav_mesh.OffMeshConnection),
    );
}

/// `v` in integer units: each coordinate times `scale`, rounded to the
/// nearest whole number, a half away from zero. Null when a coordinate is
/// not a finite number or lands outside `common.coord_max`, and when
/// `scale` is not a positive finite number.
pub fn positionFromFloat(v: [3]f32, scale: f64) ?[3]i32 {
    if (!isScale(scale)) return null;
    return scalePosition(v, scale);
}

/// An integer tile made from `float_tile`, a tile blob of the float
/// variant, which may sit at any address. The caller owns the result and
/// frees it with `gpa.free`.
///
/// Positions, the agent's sizes and the off-mesh radii are multiplied by
/// `scale` and rounded to the nearest whole number; the tile's bounds are
/// rounded outward. Polygons, links, detail meshes and detail triangles
/// are copied; `NavMesh.addTile` builds the links. The tree keeps its node
/// order and escape indices, and each box is recomputed in cells of the
/// new `bv_cell_size`.
pub fn tileFromFloat(
    gpa: std.mem.Allocator,
    float_tile: []const u8,
    scale: f64,
) ConvertError![]align(4) u8 {
    if (!isScale(scale)) return error.OutOfBounds;

    const FloatHeader = float_nav_mesh.MeshHeader;
    if (float_tile.len < @sizeOf(FloatHeader)) return error.InvalidTile;
    const float_header = std.mem.bytesToValue(FloatHeader, float_tile[0..@sizeOf(FloatHeader)]);
    if (float_header.magic != float_nav_mesh.nav_mesh_magic) return error.InvalidTile;
    if (float_header.version != float_nav_mesh.nav_mesh_version) return error.InvalidTile;

    const layout = Layout.init(&float_header) orelse return error.InvalidTile;
    if (float_tile.len < layout.blob_size) return error.InvalidTile;

    const out = try gpa.alignedAlloc(u8, .@"4", layout.blob_size);
    errdefer gpa.free(out);
    // Everything without a position is the float tile's, byte for byte.
    @memcpy(out, float_tile[0..layout.blob_size]);

    const header: *nav_mesh.MeshHeader = @ptrCast(out.ptr);
    header.magic = nav_mesh.nav_mesh_magic;
    header.walkable_height = scaleValue(float_header.walkable_height, scale, .nearest) orelse
        return error.OutOfBounds;
    header.walkable_radius = scaleValue(float_header.walkable_radius, scale, .nearest) orelse
        return error.OutOfBounds;
    header.walkable_climb = scaleValue(float_header.walkable_climb, scale, .nearest) orelse
        return error.OutOfBounds;
    for (0..3) |axis| {
        header.bounds_min[axis] = scaleValue(float_header.bounds_min[axis], scale, .down) orelse
            return error.OutOfBounds;
        header.bounds_max[axis] = scaleValue(float_header.bounds_max[axis], scale, .up) orelse
            return error.OutOfBounds;
    }
    header.bv_cell_size = treeCellSize(float_header.bv_quant_factor, scale);

    const float_verts = floatSlice([3]f32, float_tile, layout.verts_offset, layout.vert_count);
    const verts = blobSlice([3]i32, out, layout.verts_offset, layout.vert_count);
    for (verts, float_verts) |*vert, float_vert| {
        vert.* = scalePosition(float_vert, scale) orelse return error.OutOfBounds;
    }

    const float_detail_verts = floatSlice(
        [3]f32,
        float_tile,
        layout.detail_verts_offset,
        layout.detail_vert_count,
    );
    const detail_verts = blobSlice(
        [3]i32,
        out,
        layout.detail_verts_offset,
        layout.detail_vert_count,
    );
    for (detail_verts, float_detail_verts) |*vert, float_vert| {
        vert.* = scalePosition(float_vert, scale) orelse return error.OutOfBounds;
    }

    const float_connections = floatSlice(
        float_nav_mesh.OffMeshConnection,
        float_tile,
        layout.off_mesh_connections_offset,
        layout.off_mesh_connection_count,
    );
    const connections = blobSlice(
        nav_mesh.OffMeshConnection,
        out,
        layout.off_mesh_connections_offset,
        layout.off_mesh_connection_count,
    );
    for (connections, float_connections) |*connection, float_connection| {
        connection.pos[0] = scalePosition(float_connection.pos[0], scale) orelse
            return error.OutOfBounds;
        connection.pos[1] = scalePosition(float_connection.pos[1], scale) orelse
            return error.OutOfBounds;
        connection.radius = scaleValue(float_connection.radius, scale, .nearest) orelse
            return error.OutOfBounds;
    }

    try recomputeTree(
        header,
        verts,
        blobSlice(Poly, out, layout.polys_offset, layout.poly_count),
        blobSlice(PolyDetail, out, layout.detail_meshes_offset, layout.detail_mesh_count),
        detail_verts,
        blobSlice(BvNode, out, layout.bv_tree_offset, layout.bv_node_count),
    );

    return out;
}

/// Where each block of a tile blob starts and how many records it holds.
/// A float tile and its integer tile share one layout: their records have
/// the same sizes.
const Layout = struct {
    vert_count: usize,
    poly_count: usize,
    detail_mesh_count: usize,
    detail_vert_count: usize,
    bv_node_count: usize,
    off_mesh_connection_count: usize,
    verts_offset: usize,
    polys_offset: usize,
    detail_meshes_offset: usize,
    detail_verts_offset: usize,
    bv_tree_offset: usize,
    off_mesh_connections_offset: usize,
    /// The bytes the header's counts take, the header included.
    blob_size: usize,

    /// The layout the counts of `header` give, or null when one of them is
    /// negative.
    fn init(header: *const float_nav_mesh.MeshHeader) ?Layout {
        const cast = std.math.cast;
        const vert_count = cast(usize, header.vert_count) orelse return null;
        const poly_count = cast(usize, header.poly_count) orelse return null;
        const link_count_max = cast(usize, header.link_count_max) orelse return null;
        const detail_mesh_count = cast(usize, header.detail_mesh_count) orelse return null;
        const detail_vert_count = cast(usize, header.detail_vert_count) orelse return null;
        const detail_tri_count = cast(usize, header.detail_tri_count) orelse return null;
        const bv_node_count = cast(usize, header.bv_node_count) orelse return null;
        const off_mesh_connection_count = cast(usize, header.off_mesh_connection_count) orelse
            return null;

        const verts_offset = common.align4(@sizeOf(float_nav_mesh.MeshHeader));
        const polys_offset = verts_offset + common.align4(@sizeOf([3]f32) * vert_count);
        const links_offset = polys_offset + common.align4(@sizeOf(Poly) * poly_count);
        const detail_meshes_offset = links_offset +
            common.align4(@sizeOf(Link) * link_count_max);
        const detail_verts_offset = detail_meshes_offset +
            common.align4(@sizeOf(PolyDetail) * detail_mesh_count);
        const detail_tris_offset = detail_verts_offset +
            common.align4(@sizeOf([3]f32) * detail_vert_count);
        const bv_tree_offset = detail_tris_offset +
            common.align4(@sizeOf([4]u8) * detail_tri_count);
        const off_mesh_connections_offset = bv_tree_offset +
            common.align4(@sizeOf(BvNode) * bv_node_count);
        const blob_size = off_mesh_connections_offset +
            common.align4(@sizeOf(float_nav_mesh.OffMeshConnection) * off_mesh_connection_count);

        return .{
            .vert_count = vert_count,
            .poly_count = poly_count,
            .detail_mesh_count = detail_mesh_count,
            .detail_vert_count = detail_vert_count,
            .bv_node_count = bv_node_count,
            .off_mesh_connection_count = off_mesh_connection_count,
            .verts_offset = verts_offset,
            .polys_offset = polys_offset,
            .detail_meshes_offset = detail_meshes_offset,
            .detail_verts_offset = detail_verts_offset,
            .bv_tree_offset = bv_tree_offset,
            .off_mesh_connections_offset = off_mesh_connections_offset,
            .blob_size = blob_size,
        };
    }
};

/// `count` records of `T` at `offset` in a float tile, which may sit at any
/// address.
fn floatSlice(
    comptime T: type,
    data: []const u8,
    offset: usize,
    count: usize,
) []align(1) const T {
    return std.mem.bytesAsSlice(T, data[offset..][0 .. count * @sizeOf(T)]);
}

/// `count` records of `T` at `offset` in an integer tile. Every block of a
/// blob starts at a multiple of four bytes.
fn blobSlice(comptime T: type, data: []align(4) u8, offset: usize, count: usize) []align(4) T {
    const records: [*]align(4) T = @ptrCast(@alignCast(data.ptr + offset));
    return records[0..count];
}

/// Whether `scale` is a number a tile can be multiplied by.
inline fn isScale(scale: f64) bool {
    return scale > 0 and std.math.isFinite(scale);
}

/// Which whole number a scaled value becomes.
const Rounding = enum {
    /// The nearest, a half away from zero.
    nearest,
    /// The next at or below it.
    down,
    /// The next at or above it.
    up,
};

/// `value` times `scale` as a whole number of integer units, or null when
/// it is not a finite number or lies outside `common.coord_max`.
fn scaleValue(value: f32, scale: f64, rounding: Rounding) ?i32 {
    const product = @as(f64, value) * scale;
    const whole = switch (rounding) {
        .nearest => @round(product),
        .down => @floor(product),
        .up => @ceil(product),
    };
    const limit: f64 = @floatFromInt(common.coord_max);
    // Written so that a value that is not a number fails the test too.
    if (!(whole >= -limit and whole <= limit)) return null;
    const units: i32 = @intFromFloat(whole);
    return units;
}

/// `v` times `scale` with each coordinate rounded to the nearest whole
/// number, or null when one is not finite or lies outside
/// `common.coord_max`.
fn scalePosition(v: [3]f32, scale: f64) ?[3]i32 {
    return .{
        scaleValue(v[0], scale, .nearest) orelse return null,
        scaleValue(v[1], scale, .nearest) orelse return null,
        scaleValue(v[2], scale, .nearest) orelse return null,
    };
}

/// The width of a tree cell in integer units, from the float tile's cells
/// per float unit: `scale / quant_factor` rounded to nearest and held to
/// 1..`common.coord_max`. 1 when the factor is not a positive finite
/// number, as in a tile without a tree.
fn treeCellSize(quant_factor: f32, scale: f64) i32 {
    const factor: f64 = quant_factor;
    if (!(factor > 0 and std.math.isFinite(factor))) return 1;
    const size = @round(scale / factor);
    const limit: f64 = @floatFromInt(common.coord_max);
    if (!(size >= 1)) return 1;
    if (size >= limit) return common.coord_max;
    return @intFromFloat(size);
}

/// How many nodes `node` stands for, itself included: 1 for a leaf, the
/// escape count for an inner node.
inline fn nodeSpan(node: BvNode) usize {
    // Widened first: the lowest `i32` has no negation in `i32`.
    return if (node.index >= 0) 1 else @intCast(-@as(i64, node.index));
}

/// A box coordinate of the tree from a count of cells.
inline fn cellCoord(cell: i64) u16 {
    return @intCast(std.math.clamp(cell, 0, 65535));
}

/// Recomputes every box of the tree `nodes` from the tile's integer
/// geometry, in cells of `header.bv_cell_size` from `header.bounds_min`.
/// A leaf gets the union of the float tile's box and the cells that hold
/// the lowest and the highest of its polygon's vertices and detail
/// vertices; an inner node the union of its children. So a box holds the
/// integer geometry and never less than the float box did. The node order
/// and the escape indices stay.
///
/// The tree is the root and what its escape count covers. A float tile
/// holds one node more than its tree uses; nodes past the tree stay as
/// they were copied.
fn recomputeTree(
    header: *const nav_mesh.MeshHeader,
    verts: []const [3]i32,
    polys: []const Poly,
    detail_meshes: []const PolyDetail,
    detail_verts: []const [3]i32,
    nodes: []BvNode,
) error{InvalidTile}!void {
    if (nodes.len == 0) return;
    const tree_len = nodeSpan(nodes[0]);
    if (tree_len > nodes.len) return error.InvalidTile;

    // From the last node to the first, so that the children of a node,
    // which all follow it, have their new boxes before it takes their
    // union.
    var i = tree_len;
    while (i > 0) {
        i -= 1;
        const node = &nodes[i];

        if (node.index >= 0) {
            const poly_index: usize = @intCast(node.index);
            if (poly_index >= polys.len) return error.InvalidTile;
            const poly = &polys[poly_index];
            if (poly.vert_count == 0 or poly.vert_count > nav_mesh.verts_per_polygon) {
                return error.InvalidTile;
            }

            if (poly.verts[0] >= verts.len) return error.InvalidTile;
            var low = verts[poly.verts[0]];
            var high = low;
            for (poly.verts[1..poly.vert_count]) |vert_index| {
                if (vert_index >= verts.len) return error.InvalidTile;
                low = common.vmin(low, verts[vert_index]);
                high = common.vmax(high, verts[vert_index]);
            }

            if (poly_index < detail_meshes.len) {
                const detail = &detail_meshes[poly_index];
                if (detail.vert_count != 0) {
                    const end = @as(u64, detail.vert_base) + detail.vert_count;
                    if (end > detail_verts.len) return error.InvalidTile;
                    for (detail_verts[detail.vert_base..][0..detail.vert_count]) |vert| {
                        low = common.vmin(low, vert);
                        high = common.vmax(high, vert);
                    }
                }
            }

            const cell_size: i64 = header.bv_cell_size;
            for (0..3) |axis| {
                const origin: i64 = header.bounds_min[axis];
                // The float box came with the tile: the original builds it
                // from the detail heights of the polygon's corners, which the
                // tile does not keep.
                const cell_min = cellCoord(@divFloor(low[axis] - origin, cell_size));
                const cell_max = cellCoord(@divFloor(high[axis] - origin, cell_size));
                node.bounds_min[axis] = @min(node.bounds_min[axis], cell_min);
                node.bounds_max[axis] = @max(node.bounds_max[axis], cell_max);
            }
            continue;
        }

        // An inner node: its children fill the nodes up to its escape.
        const end = i + nodeSpan(node.*);
        if (end > tree_len or end == i + 1) return error.InvalidTile;
        var bounds_min: [3]u16 = @splat(65535);
        var bounds_max: [3]u16 = @splat(0);
        var child = i + 1;
        while (child < end) {
            const child_node = &nodes[child];
            for (0..3) |axis| {
                bounds_min[axis] = @min(bounds_min[axis], child_node.bounds_min[axis]);
                bounds_max[axis] = @max(bounds_max[axis], child_node.bounds_max[axis]);
            }
            child += nodeSpan(child_node.*);
        }
        // The last child must end where its parent does.
        if (child != end) return error.InvalidTile;
        node.bounds_min = bounds_min;
        node.bounds_max = bounds_max;
    }
}

// ----------------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------------

const testing = std.testing;

/// A float tile made by hand: two triangles that share a vertex, one
/// detail vertex above the first, a tree of three nodes with the unused
/// node a built tile has after them, and one off-mesh connection with its
/// polygon. Its values sit off the whole millimetre, so that each rounding
/// rule shows. The caller frees it with `testing.allocator`.
fn makeTestTile() ![]align(4) u8 {
    var header = std.mem.zeroes(float_nav_mesh.MeshHeader);
    header.magic = float_nav_mesh.nav_mesh_magic;
    header.version = float_nav_mesh.nav_mesh_version;
    header.x = 3;
    header.y = -2;
    header.layer = 1;
    header.user_id = 99;
    header.poly_count = 3;
    header.vert_count = 7;
    header.link_count_max = 4;
    header.detail_mesh_count = 2;
    header.detail_vert_count = 1;
    header.detail_tri_count = 1;
    header.bv_node_count = 4;
    header.off_mesh_connection_count = 1;
    header.off_mesh_base = 2;
    header.walkable_height = 1.9996;
    header.walkable_radius = 0.6;
    header.walkable_climb = 0.9;
    header.bounds_min = .{ -0.0003, -0.0012, 0 };
    header.bounds_max = .{ 1.0006, 2.0004, 2.0002 };
    // Four cells to a metre: a cell of 250 millimetres.
    header.bv_quant_factor = 4;

    const layout = Layout.init(&header).?;
    const blob = try testing.allocator.alignedAlloc(u8, .@"4", layout.blob_size);
    @memset(blob, 0);

    const blob_header: *float_nav_mesh.MeshHeader = @ptrCast(blob.ptr);
    blob_header.* = header;

    // The last two are the ends of the off-mesh connection.
    const verts = blobSlice([3]f32, blob, layout.verts_offset, 7);
    verts[0] = .{ 0.0004, 0, 0 };
    verts[1] = .{ 1.0006, 0.25, 0 };
    verts[2] = .{ 1, 0, 1 };
    verts[3] = .{ 1, 0.5, 2 };
    verts[4] = .{ 0.5, 0.5, 2 };
    verts[5] = .{ 0.5, 0.0004, 0.5 };
    verts[6] = .{ 0.7506, 0, 1.5 };

    const polys = blobSlice(Poly, blob, layout.polys_offset, 3);
    polys[0] = .{
        .first_link = nav_mesh.null_link,
        .verts = .{ 0, 1, 2, 0, 0, 0 },
        .neighbours = @splat(0),
        .flags = 1,
        .vert_count = 3,
        .area_and_type = 5,
    };
    polys[1] = .{
        .first_link = nav_mesh.null_link,
        .verts = .{ 2, 3, 4, 0, 0, 0 },
        .neighbours = @splat(0),
        .flags = 1,
        .vert_count = 3,
        .area_and_type = 5,
    };
    polys[2] = .{
        .first_link = nav_mesh.null_link,
        .verts = .{ 5, 6, 0, 0, 0, 0 },
        .neighbours = @splat(0),
        .flags = 1,
        .vert_count = 2,
        .area_and_type = 5,
    };
    polys[2].setType(.off_mesh_connection);

    // The links lie between the polygons and the detail meshes. Their
    // content is whatever the tile came with.
    const links_offset = layout.polys_offset + common.align4(@sizeOf(Poly) * 3);
    @memset(blob[links_offset..layout.detail_meshes_offset], 0xa5);

    const detail_meshes = blobSlice(PolyDetail, blob, layout.detail_meshes_offset, 2);
    detail_meshes[0] = .{ .vert_base = 0, .tri_base = 0, .vert_count = 1, .tri_count = 1 };
    detail_meshes[1] = .{ .vert_base = 1, .tri_base = 1, .vert_count = 0, .tri_count = 0 };

    const detail_verts = blobSlice([3]f32, blob, layout.detail_verts_offset, 1);
    detail_verts[0] = .{ 0.5, 2.0004, 0.25 };

    // The one detail triangle follows the one detail vertex.
    const detail_tris = blobSlice(
        [4]u8,
        blob,
        layout.detail_verts_offset + @sizeOf([3]f32),
        1,
    );
    detail_tris[0] = .{ 0, 1, 3, 0 };

    // The boxes are wrong on purpose: the converter makes its own. The
    // fourth node stays zero.
    const nodes = blobSlice(BvNode, blob, layout.bv_tree_offset, 4);
    // Float boxes the conversion widens: an empty one, and one whose top
    // reaches a cell above the geometry, as a detail height can.
    nodes[0] = .{ .bounds_min = @splat(9), .bounds_max = @splat(9), .index = -3 };
    nodes[1] = .{ .bounds_min = @splat(65535), .bounds_max = @splat(0), .index = 0 };
    nodes[2] = .{ .bounds_min = @splat(65535), .bounds_max = .{ 0, 9, 0 }, .index = 1 };

    const connections = blobSlice(
        float_nav_mesh.OffMeshConnection,
        blob,
        layout.off_mesh_connections_offset,
        1,
    );
    connections[0] = .{
        .pos = .{ .{ 0.5, 0.0004, 0.5 }, .{ 0.7506, 0, 1.5 } },
        .radius = 0.1996,
        .poly = 2,
        .flags = 1,
        .side = 0xff,
        .user_id = 77,
    };

    return blob;
}

test "positionFromFloat rounds to the nearest unit, a half away from zero" {
    try testing.expectEqual([3]i32{ 1, -1, 2 }, positionFromFloat(.{ 0.5, -0.5, 1.5 }, 1).?);
    try testing.expectEqual(
        [3]i32{ 2500, -2500, 250 },
        positionFromFloat(.{ 2.5, -2.5, 0.25 }, default_scale).?,
    );
    try testing.expectEqual(
        [3]i32{ 0, 1001, -1 },
        positionFromFloat(.{ 0.0004, 1.0006, -0.0006 }, default_scale).?,
    );
}

test "positionFromFloat refuses what is out of bounds or not finite" {
    // The bound itself is inside; the next `f32` beyond it, 64 further, is
    // not.
    const limit: f32 = @floatFromInt(common.coord_max);
    try testing.expectEqual(
        [3]i32{ common.coord_max, -common.coord_max, 0 },
        positionFromFloat(.{ limit, -limit, 0 }, 1).?,
    );
    try testing.expect(positionFromFloat(.{ limit + 64, 0, 0 }, 1) == null);
    try testing.expect(positionFromFloat(.{ 0, -limit - 64, 0 }, 1) == null);
    try testing.expect(positionFromFloat(.{ 0, 0, 600_000 }, default_scale) == null);

    try testing.expect(positionFromFloat(.{ std.math.nan(f32), 0, 0 }, default_scale) == null);
    try testing.expect(positionFromFloat(.{ 0, std.math.inf(f32), 0 }, default_scale) == null);
    try testing.expect(positionFromFloat(.{ 0, 0, -std.math.inf(f32) }, default_scale) == null);

    try testing.expect(positionFromFloat(.{ 1, 2, 3 }, 0) == null);
    try testing.expect(positionFromFloat(.{ 1, 2, 3 }, -1000) == null);
    try testing.expect(positionFromFloat(.{ 1, 2, 3 }, std.math.nan(f64)) == null);
    try testing.expect(positionFromFloat(.{ 1, 2, 3 }, std.math.inf(f64)) == null);
}

test "treeCellSize rounds to the nearest unit and is at least one" {
    try testing.expectEqual(250, treeCellSize(4, 1000));
    try testing.expectEqual(1, treeCellSize(4, 1));
    try testing.expectEqual(1, treeCellSize(0, 1000));
    try testing.expectEqual(1, treeCellSize(-4, 1000));
    try testing.expectEqual(1, treeCellSize(std.math.nan(f32), 1000));
    try testing.expectEqual(1, treeCellSize(std.math.inf(f32), 1000));
    try testing.expectEqual(common.coord_max, treeCellSize(1, 1e30));
}

test "tileFromFloat rounds a tile's values and recomputes its tree" {
    const gpa = testing.allocator;
    const float_tile = try makeTestTile();
    defer gpa.free(float_tile);

    const tile = try tileFromFloat(gpa, float_tile, default_scale);
    defer gpa.free(tile);
    try testing.expectEqual(float_tile.len, tile.len);

    const layout = Layout.init(@ptrCast(float_tile.ptr)).?;

    // The header: the agent's sizes to the nearest millimetre, the bounds
    // outward, everything else kept.
    const header: *const nav_mesh.MeshHeader = @ptrCast(tile.ptr);
    try testing.expectEqual(nav_mesh.nav_mesh_magic, header.magic);
    try testing.expectEqual(nav_mesh.nav_mesh_version, header.version);
    try testing.expectEqual(3, header.x);
    try testing.expectEqual(-2, header.y);
    try testing.expectEqual(1, header.layer);
    try testing.expectEqual(99, header.user_id);
    try testing.expectEqual(3, header.poly_count);
    try testing.expectEqual(7, header.vert_count);
    try testing.expectEqual(4, header.link_count_max);
    try testing.expectEqual(2, header.detail_mesh_count);
    try testing.expectEqual(1, header.detail_vert_count);
    try testing.expectEqual(1, header.detail_tri_count);
    try testing.expectEqual(4, header.bv_node_count);
    try testing.expectEqual(1, header.off_mesh_connection_count);
    try testing.expectEqual(2, header.off_mesh_base);
    try testing.expectEqual(2000, header.walkable_height);
    try testing.expectEqual(600, header.walkable_radius);
    try testing.expectEqual(900, header.walkable_climb);
    try testing.expectEqual([3]i32{ -1, -2, 0 }, header.bounds_min);
    try testing.expectEqual([3]i32{ 1001, 2001, 2001 }, header.bounds_max);
    try testing.expectEqual(250, header.bv_cell_size);

    const verts = blobSlice([3]i32, tile, layout.verts_offset, 7);
    try testing.expectEqual([3]i32{ 0, 0, 0 }, verts[0]);
    try testing.expectEqual([3]i32{ 1001, 250, 0 }, verts[1]);
    try testing.expectEqual([3]i32{ 1000, 0, 1000 }, verts[2]);
    try testing.expectEqual([3]i32{ 1000, 500, 2000 }, verts[3]);
    try testing.expectEqual([3]i32{ 500, 500, 2000 }, verts[4]);
    try testing.expectEqual([3]i32{ 500, 0, 500 }, verts[5]);
    try testing.expectEqual([3]i32{ 751, 0, 1500 }, verts[6]);

    const detail_verts = blobSlice([3]i32, tile, layout.detail_verts_offset, 1);
    try testing.expectEqual([3]i32{ 500, 2000, 250 }, detail_verts[0]);

    // Polygons, links and detail meshes, then the detail triangles, are
    // the float tile's bytes.
    try testing.expectEqualSlices(
        u8,
        float_tile[layout.polys_offset..layout.detail_verts_offset],
        tile[layout.polys_offset..layout.detail_verts_offset],
    );
    const detail_tris_offset = layout.detail_verts_offset + @sizeOf([3]f32);
    try testing.expectEqualSlices(
        u8,
        float_tile[detail_tris_offset..layout.bv_tree_offset],
        tile[detail_tris_offset..layout.bv_tree_offset],
    );

    // The tree, in cells of 250 millimetres from the corner (-1, -2, 0): a
    // box runs from the cell that holds its lowest point to the cell that
    // holds its highest, or further where the float box did.
    // The first triangle and its detail vertex span (0, 0, 0) to
    // (1001, 2000, 1000), the second (500, 0, 1000) to (1000, 500, 2000)
    // with its float box up to cell 9 in y; the root holds both.
    const nodes = blobSlice(BvNode, tile, layout.bv_tree_offset, 4);
    try testing.expectEqual(-3, nodes[0].index);
    try testing.expectEqual([3]u16{ 0, 0, 0 }, nodes[0].bounds_min);
    try testing.expectEqual([3]u16{ 4, 9, 8 }, nodes[0].bounds_max);
    try testing.expectEqual(0, nodes[1].index);
    try testing.expectEqual([3]u16{ 0, 0, 0 }, nodes[1].bounds_min);
    try testing.expectEqual([3]u16{ 4, 8, 4 }, nodes[1].bounds_max);
    try testing.expectEqual(1, nodes[2].index);
    try testing.expectEqual([3]u16{ 2, 0, 4 }, nodes[2].bounds_min);
    try testing.expectEqual([3]u16{ 4, 9, 8 }, nodes[2].bounds_max);
    // The node past the tree is as it was.
    try testing.expectEqual(0, nodes[3].index);
    try testing.expectEqual([3]u16{ 0, 0, 0 }, nodes[3].bounds_min);
    try testing.expectEqual([3]u16{ 0, 0, 0 }, nodes[3].bounds_max);

    const connections = blobSlice(
        nav_mesh.OffMeshConnection,
        tile,
        layout.off_mesh_connections_offset,
        1,
    );
    try testing.expectEqual([3]i32{ 500, 0, 500 }, connections[0].pos[0]);
    try testing.expectEqual([3]i32{ 751, 0, 1500 }, connections[0].pos[1]);
    try testing.expectEqual(200, connections[0].radius);
    try testing.expectEqual(2, connections[0].poly);
    try testing.expectEqual(1, connections[0].flags);
    try testing.expectEqual(0xff, connections[0].side);
    try testing.expectEqual(77, connections[0].user_id);
}

test "tileFromFloat reads a float tile at any address" {
    const gpa = testing.allocator;
    const float_tile = try makeTestTile();
    defer gpa.free(float_tile);

    // One byte into a block aligned to four is aligned to nothing.
    const shifted = try gpa.alignedAlloc(u8, .@"4", float_tile.len + 1);
    defer gpa.free(shifted);
    @memcpy(shifted[1..], float_tile);

    const expected = try tileFromFloat(gpa, float_tile, default_scale);
    defer gpa.free(expected);
    const actual = try tileFromFloat(gpa, shifted[1..], default_scale);
    defer gpa.free(actual);
    try testing.expectEqualSlices(u8, expected, actual);
}

test "tileFromFloat refuses what is not a float tile" {
    const gpa = testing.allocator;
    const float_tile = try makeTestTile();
    defer gpa.free(float_tile);
    const header: *float_nav_mesh.MeshHeader = @ptrCast(float_tile.ptr);
    const invalid = error.InvalidTile;

    // Too short for a header, then for the header's counts.
    try testing.expectError(invalid, tileFromFloat(gpa, float_tile[0..8], default_scale));
    try testing.expectError(
        invalid,
        tileFromFloat(gpa, float_tile[0 .. float_tile.len - 1], default_scale),
    );

    // An integer tile's magic number.
    header.magic = nav_mesh.nav_mesh_magic;
    try testing.expectError(invalid, tileFromFloat(gpa, float_tile, default_scale));
    header.magic = float_nav_mesh.nav_mesh_magic;

    header.version += 1;
    try testing.expectError(invalid, tileFromFloat(gpa, float_tile, default_scale));
    header.version -= 1;

    header.vert_count = -1;
    try testing.expectError(invalid, tileFromFloat(gpa, float_tile, default_scale));
    header.vert_count = 7;

    const layout = Layout.init(header).?;
    const nodes = blobSlice(BvNode, float_tile, layout.bv_tree_offset, 4);

    // A leaf for a polygon the tile does not hold.
    nodes[2].index = 3;
    try testing.expectError(invalid, tileFromFloat(gpa, float_tile, default_scale));
    nodes[2].index = 1;

    // An escape past the tile's nodes, and one that leaves a node without
    // children.
    nodes[0].index = -5;
    try testing.expectError(invalid, tileFromFloat(gpa, float_tile, default_scale));
    nodes[0].index = -1;
    try testing.expectError(invalid, tileFromFloat(gpa, float_tile, default_scale));
    nodes[0].index = -3;

    // Whole again, it converts.
    const tile = try tileFromFloat(gpa, float_tile, default_scale);
    gpa.free(tile);
}

test "tileFromFloat refuses a value outside the coordinate bound" {
    const gpa = testing.allocator;
    const float_tile = try makeTestTile();
    defer gpa.free(float_tile);
    const layout = Layout.init(@ptrCast(float_tile.ptr)).?;
    const verts = blobSlice([3]f32, float_tile, layout.verts_offset, 7);
    const out_of_bounds = error.OutOfBounds;

    // 600 kilometres is past the bound of 2^29 millimetres.
    verts[4][1] = 600_000;
    try testing.expectError(out_of_bounds, tileFromFloat(gpa, float_tile, default_scale));
    verts[4][1] = std.math.nan(f32);
    try testing.expectError(out_of_bounds, tileFromFloat(gpa, float_tile, default_scale));
    verts[4][1] = -std.math.inf(f32);
    try testing.expectError(out_of_bounds, tileFromFloat(gpa, float_tile, default_scale));
    verts[4][1] = 0.5;

    try testing.expectError(out_of_bounds, tileFromFloat(gpa, float_tile, 0));
    try testing.expectError(out_of_bounds, tileFromFloat(gpa, float_tile, std.math.nan(f64)));

    // The same tile fits at a millimetre to the unit and not at a million.
    const tile = try tileFromFloat(gpa, float_tile, default_scale);
    gpa.free(tile);
    try testing.expectError(out_of_bounds, tileFromFloat(gpa, float_tile, 1e9));
}

/// `float_tile`, a tile in metres, rewritten in units of which a metre
/// holds `per_metre`: every position, extent and agent size multiplied by
/// it, and the tree's cells per unit divided by it.
fn tileInUnits(float_tile: []align(4) u8, per_metre: f32) void {
    const header: *float_nav_mesh.MeshHeader = @ptrCast(float_tile.ptr);
    header.walkable_height *= per_metre;
    header.walkable_radius *= per_metre;
    header.walkable_climb *= per_metre;
    for (&header.bounds_min, &header.bounds_max) |*low, *high| {
        low.* *= per_metre;
        high.* *= per_metre;
    }
    header.bv_quant_factor /= per_metre;
    const layout = Layout.init(header).?;
    const verts = blobSlice([3]f32, float_tile, layout.verts_offset, layout.vert_count);
    const detail_verts = blobSlice(
        [3]f32,
        float_tile,
        layout.detail_verts_offset,
        layout.detail_vert_count,
    );
    const connections = blobSlice(
        float_nav_mesh.OffMeshConnection,
        float_tile,
        layout.off_mesh_connections_offset,
        layout.off_mesh_connection_count,
    );
    for (verts) |*v| {
        for (v) |*c| c.* *= per_metre;
    }
    for (detail_verts) |*v| {
        for (v) |*c| c.* *= per_metre;
    }
    for (connections) |*connection| {
        for (&connection.pos) |*v| {
            for (v) |*c| c.* *= per_metre;
        }
        connection.radius *= per_metre;
    }
}

test "tileFromFloat takes a tile in other units at the scale that makes millimetres" {
    // The integer unit is the millimetre at every scale: a tile in
    // centimetres at 10, or in millimetres at 1, gives the bytes the tile
    // in metres gives at 1000, so every distance the queries fix in
    // millimetres keeps its size.
    const gpa = testing.allocator;
    const float_tile = try makeTestTile();
    defer gpa.free(float_tile);
    const expected = try tileFromFloat(gpa, float_tile, default_scale);
    defer gpa.free(expected);
    for ([_]f32{ 100, 1000 }) |per_metre| {
        const other = try gpa.alignedAlloc(u8, .@"4", float_tile.len);
        defer gpa.free(other);
        @memcpy(other, float_tile);
        tileInUnits(other, per_metre);
        const tile = try tileFromFloat(gpa, other, default_scale / per_metre);
        defer gpa.free(tile);
        try testing.expectEqualSlices(u8, expected, tile);
    }
}

test "tileFromFloat reports a failed allocation" {
    const float_tile = try makeTestTile();
    defer testing.allocator.free(float_tile);
    try testing.expectError(
        error.OutOfMemory,
        tileFromFloat(testing.failing_allocator, float_tile, default_scale),
    );
}
