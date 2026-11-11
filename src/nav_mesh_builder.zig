//! Builds the bytes of a navigation mesh tile from a polygon mesh, and swaps
//! the byte order of a tile's bytes.
//!
//! The bytes have the layout `nav_mesh.zig` reads: every block, every count
//! and every padding byte is what upstream Detour writes for the same input.

const std = @import("std");
const common = @import("common.zig");
const nav_mesh = @import("nav_mesh.zig");

const MeshHeader = nav_mesh.MeshHeader;
const Poly = nav_mesh.Poly;
const Link = nav_mesh.Link;
const PolyDetail = nav_mesh.PolyDetail;
const BvNode = nav_mesh.BvNode;
const OffMeshConnection = nav_mesh.OffMeshConnection;

/// The polygon mesh's "no vertex" entry.
const mesh_null_index: u16 = 0xffff;

/// What a tile is built from: a polygon mesh in cells, optionally its
/// height detail and off-mesh connections, and where the tile sits.
///
/// `createNavMeshData` only reads the slices, and keeps none of them. Each
/// count says how many entries of its slices the build uses; a slice may be
/// longer. The height detail, the off-mesh connections and the tile's place
/// may be left at their defaults.
pub const NavMeshCreateParams = struct {
    /// The polygon mesh's vertices (x, y, z) in cells from `bounds_min`: x
    /// and z in `cell_size`, y in `cell_height`.
    verts: []const [3]u16,
    /// How many of `verts` the mesh has: at least 3, below 0xffff.
    vert_count: i32,
    /// Per polygon `2 * vertices_per_polygon` entries: its vertex indices,
    /// padded with 0xffff, then per edge the polygon across it, 0xffff for
    /// a wall, or 0x8000 plus a direction (0 x-, 1 z+, 2 x+, 3 z-) for an
    /// edge on the tile's border.
    polys: []const u16,
    /// Per polygon, the flags a query filter tests.
    poly_flags: []const u16,
    /// Per polygon, its area id.
    poly_areas: []const u8,
    /// How many polygons the mesh has: at least 1.
    poly_count: i32,
    /// The most vertices a polygon of `polys` has: 3 to
    /// `nav_mesh.verts_per_polygon`.
    vertices_per_polygon: i32,

    /// Per polygon, where its height detail sits: the first vertex in
    /// `detail_verts`, the vertex count, the first triangle in
    /// `detail_tris`, the triangle count. A polygon's detail vertices start
    /// with the polygon's own. Null builds flat detail from the polygons.
    detail_meshes: ?[]const [4]u32 = null,
    /// The detail vertices, in world units.
    detail_verts: []const [3]f32 = &.{},
    /// How many of `detail_verts` there are.
    detail_verts_count: i32 = 0,
    /// The detail triangles: three vertex indices local to the polygon's
    /// detail mesh, and the edge flags.
    detail_tris: []const [4]u8 = &.{},
    /// How many of `detail_tris` there are.
    detail_tri_count: i32 = 0,

    /// Per off-mesh connection, its start and end in world units. Only a
    /// connection that starts inside the tile is stored in it.
    off_mesh_con_verts: []const [2][3]f32 = &.{},
    /// Per off-mesh connection, how far from an end point a polygon may be
    /// and still attach.
    off_mesh_con_rad: []const f32 = &.{},
    /// Per off-mesh connection, the flags a query filter tests.
    off_mesh_con_flags: []const u16 = &.{},
    /// Per off-mesh connection, its area id.
    off_mesh_con_areas: []const u8 = &.{},
    /// Per off-mesh connection, zero when it is walked from start to end
    /// only, anything else when it is walked both ways.
    off_mesh_con_dir: []const u8 = &.{},
    /// Per off-mesh connection, an id the application defines. Null leaves
    /// every id zero.
    off_mesh_con_user_id: ?[]const u32 = null,
    /// How many off-mesh connections there are.
    off_mesh_con_count: i32 = 0,

    /// An id for the tile the application defines.
    user_id: u32 = 0,
    /// The tile's place in the tile grid, along x.
    tile_x: i32 = 0,
    /// The tile's place in the tile grid, along z.
    tile_y: i32 = 0,
    /// The tile's layer in its grid cell.
    tile_layer: i32 = 0,
    /// The tile's bounds in world units.
    bounds_min: [3]f32,
    bounds_max: [3]f32,

    /// The agent the tile is built for, in world units: its height, its
    /// radius and the highest ledge it steps over.
    walkable_height: f32,
    walkable_radius: f32,
    walkable_climb: f32,
    /// The size of a cell of the polygon mesh in the xz plane.
    cell_size: f32,
    /// The height of a cell of the polygon mesh.
    cell_height: f32,

    /// Whether the tile gets a bounding-volume tree, which speeds up
    /// finding polygons by position in a tile with many of them.
    build_bv_tree: bool,
};

/// A polygon's box while the bounding-volume tree is built.
const BvItem = struct {
    bounds_min: [3]u16,
    bounds_max: [3]u16,
    /// The polygon's index.
    index: i32,
};

/// Orders items by the low corner of their boxes along `axis`.
fn boundsMinLessThan(axis: usize, lhs: BvItem, rhs: BvItem) bool {
    return lhs.bounds_min[axis] < rhs.bounds_min[axis];
}

/// The box around all of `items`, which holds at least one.
fn calcExtends(items: []const BvItem, bounds_min: *[3]u16, bounds_max: *[3]u16) void {
    bounds_min.* = items[0].bounds_min;
    bounds_max.* = items[0].bounds_max;

    for (items[1..]) |item| {
        if (item.bounds_min[0] < bounds_min[0]) bounds_min[0] = item.bounds_min[0];
        if (item.bounds_min[1] < bounds_min[1]) bounds_min[1] = item.bounds_min[1];
        if (item.bounds_min[2] < bounds_min[2]) bounds_min[2] = item.bounds_min[2];

        if (item.bounds_max[0] > bounds_max[0]) bounds_max[0] = item.bounds_max[0];
        if (item.bounds_max[1] > bounds_max[1]) bounds_max[1] = item.bounds_max[1];
        if (item.bounds_max[2] > bounds_max[2]) bounds_max[2] = item.bounds_max[2];
    }
}

/// The axis of the largest of three extents, the earlier axis on a tie.
inline fn longestAxis(x: u16, y: u16, z: u16) usize {
    var axis: usize = 0;
    var value_max = x;
    if (y > value_max) {
        axis = 1;
        value_max = y;
    }
    if (z > value_max) {
        axis = 2;
    }
    return axis;
}

/// Writes the tree over `items[index_min..index_max]` to `nodes`, from
/// `node_cursor` on, depth first: a node, its left half, its right half.
fn subdivide(
    items: []BvItem,
    index_min: i32,
    index_max: i32,
    node_cursor: *i32,
    nodes: []BvNode,
) void {
    const item_count = index_max - index_min;
    const node_index = node_cursor.*;

    const node = &nodes[@intCast(node_cursor.*)];
    node_cursor.* += 1;

    const range = items[@intCast(index_min)..@intCast(index_max)];

    if (item_count == 1) {
        // A leaf.
        node.bounds_min = range[0].bounds_min;
        node.bounds_max = range[0].bounds_max;
        node.index = range[0].index;
    } else {
        // Split along the longest axis of the box around the items.
        calcExtends(range, &node.bounds_min, &node.bounds_max);

        const axis = longestAxis(
            node.bounds_max[0] -% node.bounds_min[0],
            node.bounds_max[1] -% node.bounds_min[1],
            node.bounds_max[2] -% node.bounds_min[2],
        );

        // A stable sort: items with equal keys keep their order.
        std.mem.sort(BvItem, range, axis, boundsMinLessThan);

        const index_split = index_min + @divTrunc(item_count, 2);

        // Left.
        subdivide(items, index_min, index_split, node_cursor, nodes);
        // Right.
        subdivide(items, index_split, index_max, node_cursor, nodes);

        // A negative index says how many nodes to skip when the box is missed.
        const escape = node_cursor.* - node_index;
        node.index = -escape;
    }
}

/// A tile-space offset in tree units as a box coordinate, kept within 0 to
/// 0xffff. An offset that is not a number or beyond `i32` gives 0.
inline fn quantize(offset: f32) u16 {
    return @intCast(std.math.clamp(common.truncToInt(i32, offset), 0, 0xffff));
}

/// Fills `nodes` with the bounding-volume tree over the mesh's polygons:
/// `2 * poly_count - 1` nodes, in tree units of `1 / cell_size`.
fn createBvTree(
    gpa: std.mem.Allocator,
    params: *const NavMeshCreateParams,
    nodes: []BvNode,
) error{OutOfMemory}!void {
    const quant_factor = 1 / params.cell_size;
    const vertices_per_polygon: usize = @intCast(params.vertices_per_polygon);

    const items = try gpa.alloc(BvItem, @intCast(params.poly_count));
    defer gpa.free(items);

    for (items, 0..) |*item, i| {
        item.index = @intCast(i);
        // Calculate the polygon's bounds, from its detail mesh if there is one.
        if (params.detail_meshes) |detail_meshes| {
            const vert_base: usize = detail_meshes[i][0];
            const detail_vert_count: i32 = @bitCast(detail_meshes[i][1]);

            const detail_verts = params.detail_verts[vert_base..];
            var bounds_min = detail_verts[0];
            var bounds_max = detail_verts[0];

            var j: i32 = 1;
            while (j < detail_vert_count) : (j += 1) {
                bounds_min = common.vmin(bounds_min, detail_verts[@intCast(j)]);
                bounds_max = common.vmax(bounds_max, detail_verts[@intCast(j)]);
            }

            // The tree uses the cell size for every axis.
            for (0..3) |axis| {
                const offset_min = bounds_min[axis] - params.bounds_min[axis];
                const offset_max = bounds_max[axis] - params.bounds_min[axis];
                item.bounds_min[axis] = quantize(offset_min * quant_factor);
                item.bounds_max[axis] = quantize(offset_max * quant_factor);
            }
        } else {
            const p = params.polys[i * vertices_per_polygon * 2 ..];
            item.bounds_min = params.verts[p[0]];
            item.bounds_max = params.verts[p[0]];

            var j: usize = 1;
            while (j < vertices_per_polygon) : (j += 1) {
                if (p[j] == mesh_null_index) break;
                const vert = params.verts[p[j]];

                if (vert[0] < item.bounds_min[0]) item.bounds_min[0] = vert[0];
                if (vert[1] < item.bounds_min[1]) item.bounds_min[1] = vert[1];
                if (vert[2] < item.bounds_min[2]) item.bounds_min[2] = vert[2];

                if (vert[0] > item.bounds_max[0]) item.bounds_max[0] = vert[0];
                if (vert[1] > item.bounds_max[1]) item.bounds_max[1] = vert[1];
                if (vert[2] > item.bounds_max[2]) item.bounds_max[2] = vert[2];
            }

            // Remap y, which the mesh counts in cell heights. A result
            // beyond 0xffff keeps its low 16 bits.
            const y_min: f32 = @floatFromInt(item.bounds_min[1]);
            const y_max: f32 = @floatFromInt(item.bounds_max[1]);
            item.bounds_min[1] = common.truncToInt(
                u16,
                @floor(y_min * params.cell_height / params.cell_size),
            );
            item.bounds_max[1] = common.truncToInt(
                u16,
                @ceil(y_max * params.cell_height / params.cell_size),
            );
        }
    }

    var node_cursor: i32 = 0;
    subdivide(items, 0, params.poly_count, &node_cursor, nodes);
}

/// Which side of the tile's bounds `pt` lies beyond in the xz plane: 0 to 7
/// counter-clockwise from x+, or 0xff when it is inside.
fn classifyOffMeshPoint(pt: [3]f32, bounds_min: [3]f32, bounds_max: [3]f32) u8 {
    const x_plus: u8 = 1 << 0;
    const z_plus: u8 = 1 << 1;
    const x_minus: u8 = 1 << 2;
    const z_minus: u8 = 1 << 3;

    var outcode: u8 = 0;
    outcode |= if (pt[0] >= bounds_max[0]) x_plus else 0;
    outcode |= if (pt[2] >= bounds_max[2]) z_plus else 0;
    outcode |= if (pt[0] < bounds_min[0]) x_minus else 0;
    outcode |= if (pt[2] < bounds_min[2]) z_minus else 0;

    return switch (outcode) {
        x_plus => 0,
        x_plus | z_plus => 1,
        z_plus => 2,
        x_minus | z_plus => 3,
        x_minus => 4,
        x_minus | z_minus => 5,
        z_minus => 6,
        x_plus | z_minus => 7,
        else => 0xff,
    };
}

/// `count` records of `T` at `offset` in a tile's bytes. Every block of a
/// tile starts at a multiple of four bytes, and no record `T` used here
/// needs more.
fn blobSlice(comptime T: type, data: []align(4) u8, offset: usize, count: usize) []T {
    comptime std.debug.assert(@alignOf(T) <= 4);
    const records: [*]T = @ptrCast(@alignCast(data.ptr + offset));
    return records[0..count];
}

/// Builds a tile's bytes from `params`, for `NavMesh.addTile` or
/// `NavMesh.initSingleTile`.
///
/// The caller owns the bytes, allocated from `gpa`. A navigation mesh given
/// them with `TileFlags.free_data` frees them itself, so it must have been
/// readied with the same allocator; otherwise the caller frees them with
/// `gpa` once the tile is removed.
///
/// `error.InvalidParam` says the mesh has more vertices per polygon than
/// `nav_mesh.verts_per_polygon`, 0xffff vertices or more, no vertices or no
/// polygons, a negative count, or a polygon with more vertices than its
/// detail mesh. Indices and counts beyond that are trusted to fit their
/// slices.
pub fn createNavMeshData(
    gpa: std.mem.Allocator,
    params: *const NavMeshCreateParams,
) error{ OutOfMemory, InvalidParam }![]align(4) u8 {
    if (params.vertices_per_polygon > nav_mesh.verts_per_polygon) return error.InvalidParam;
    if (params.vert_count >= 0xffff) return error.InvalidParam;
    if (params.vert_count == 0 or params.verts.len == 0) return error.InvalidParam;
    if (params.poly_count == 0 or params.polys.len == 0) return error.InvalidParam;
    // A negative count would index before the start of a slice.
    if (params.vert_count < 0 or params.poly_count < 0 or params.vertices_per_polygon < 0) {
        return error.InvalidParam;
    }

    const vertices_per_polygon: usize = @intCast(params.vertices_per_polygon);
    const vert_count: usize = @intCast(params.vert_count);
    const poly_count: usize = @intCast(params.poly_count);
    const off_mesh_con_count: usize = @intCast(@max(params.off_mesh_con_count, 0));

    // Classify the off-mesh connections' end points. Only the connections
    // that start inside the tile are stored.
    var off_mesh_con_class: []u8 = &.{};
    defer gpa.free(off_mesh_con_class);
    var stored_off_mesh_con_count: i32 = 0;
    var off_mesh_con_link_count: i32 = 0;

    if (params.off_mesh_con_count > 0) {
        off_mesh_con_class = try gpa.alloc(u8, off_mesh_con_count * 2);

        // Find tight height bounds, to cull the starts that cannot touch
        // the mesh.
        var height_min: f32 = std.math.floatMax(f32);
        var height_max: f32 = -std.math.floatMax(f32);

        if (params.detail_verts.len != 0 and params.detail_verts_count != 0) {
            var i: i32 = 0;
            while (i < params.detail_verts_count) : (i += 1) {
                const h = params.detail_verts[@intCast(i)][1];
                height_min = common.min(f32, height_min, h);
                height_max = common.max(f32, height_max, h);
            }
        } else {
            for (params.verts[0..vert_count]) |vert| {
                const y: f32 = @floatFromInt(vert[1]);
                const h = params.bounds_min[1] + y * params.cell_height;
                height_min = common.min(f32, height_min, h);
                height_max = common.max(f32, height_max, h);
            }
        }
        height_min -= params.walkable_climb;
        height_max += params.walkable_climb;
        var bounds_min = params.bounds_min;
        var bounds_max = params.bounds_max;
        bounds_min[1] = height_min;
        bounds_max[1] = height_max;

        for (0..off_mesh_con_count) |i| {
            const p0 = params.off_mesh_con_verts[i][0];
            const p1 = params.off_mesh_con_verts[i][1];
            off_mesh_con_class[i * 2 + 0] = classifyOffMeshPoint(p0, bounds_min, bounds_max);
            off_mesh_con_class[i * 2 + 1] = classifyOffMeshPoint(p1, bounds_min, bounds_max);

            // Clear the starts above or below the mesh: they cannot touch it.
            if (off_mesh_con_class[i * 2 + 0] == 0xff) {
                if (p0[1] < bounds_min[1] or p0[1] > bounds_max[1]) {
                    off_mesh_con_class[i * 2 + 0] = 0;
                }
            }

            // Count the links the off-mesh connections need.
            if (off_mesh_con_class[i * 2 + 0] == 0xff) off_mesh_con_link_count += 1;
            if (off_mesh_con_class[i * 2 + 1] == 0xff) off_mesh_con_link_count += 1;

            if (off_mesh_con_class[i * 2 + 0] == 0xff) stored_off_mesh_con_count += 1;
        }
    }

    // Off-mesh connections are stored as polygons of two vertices.
    const total_poly_count = params.poly_count + stored_off_mesh_con_count;
    const total_vert_count = params.vert_count + stored_off_mesh_con_count * 2;

    // Count the edges, and those that are portals on the tile's border.
    var edge_count: i32 = 0;
    var portal_count: i32 = 0;
    for (0..poly_count) |i| {
        const p = params.polys[i * 2 * vertices_per_polygon ..];
        for (0..vertices_per_polygon) |j| {
            if (p[j] == mesh_null_index) break;
            edge_count += 1;

            const neighbour = p[vertices_per_polygon + j];
            if (neighbour & 0x8000 != 0) {
                const dir = neighbour & 0xf;
                if (dir != 0xf) portal_count += 1;
            }
        }
    }

    const link_count_max = edge_count + portal_count * 2 + off_mesh_con_link_count * 2;

    // Count the detail vertices that are not polygon vertices, and the
    // detail triangles.
    var unique_detail_vert_count: i32 = 0;
    var detail_tri_count: i32 = 0;
    // The triangles the fans of polygons take when there is no detail mesh.
    var fan_tri_count: usize = 0;
    if (params.detail_meshes) |detail_meshes| {
        // The detail mesh's own triangles are stored.
        detail_tri_count = params.detail_tri_count;
        for (0..poly_count) |i| {
            const p = params.polys[i * vertices_per_polygon * 2 ..];
            const detail_vert_count: i32 = @bitCast(detail_meshes[i][1]);
            var nv: i32 = 0;
            for (0..vertices_per_polygon) |j| {
                if (p[j] == mesh_null_index) break;
                nv += 1;
            }
            // A detail mesh starts with its polygon's vertices; one with
            // fewer has a negative number of its own to copy.
            if (detail_vert_count < nv) return error.InvalidParam;
            unique_detail_vert_count += detail_vert_count - nv;
        }
    } else {
        // No detail mesh: each polygon is stored as a fan of triangles.
        for (0..poly_count) |i| {
            const p = params.polys[i * vertices_per_polygon * 2 ..];
            var nv: i32 = 0;
            for (0..vertices_per_polygon) |j| {
                if (p[j] == mesh_null_index) break;
                nv += 1;
            }
            detail_tri_count += nv - 2;
            if (nv > 2) fan_tri_count += @intCast(nv - 2);
        }
    }
    // Negative from a negative `detail_tri_count`, or from polygons of
    // fewer than two vertices.
    if (detail_tri_count < 0) return error.InvalidParam;

    // Calculate the size of each block, in the tile's block order.
    const verts_len: usize = @intCast(total_vert_count);
    const polys_len: usize = @intCast(total_poly_count);
    const links_len: usize = @intCast(link_count_max);
    const detail_verts_len: usize = @intCast(unique_detail_vert_count);
    const detail_tris_len: usize = @intCast(detail_tri_count);
    const bv_nodes_len: usize = if (params.build_bv_tree) poly_count * 2 else 0;
    const off_mesh_cons_len: usize = @intCast(stored_off_mesh_con_count);

    const header_size = common.align4(@sizeOf(MeshHeader));
    const verts_size = common.align4(@sizeOf([3]f32) * verts_len);
    const polys_size = common.align4(@sizeOf(Poly) * polys_len);
    const links_size = common.align4(@sizeOf(Link) * links_len);
    const detail_meshes_size = common.align4(@sizeOf(PolyDetail) * poly_count);
    const detail_verts_size = common.align4(@sizeOf([3]f32) * detail_verts_len);
    const detail_tris_size = common.align4(@sizeOf([4]u8) * detail_tris_len);
    const bv_tree_size = common.align4(@sizeOf(BvNode) * bv_nodes_len);
    const off_mesh_cons_size = common.align4(@sizeOf(OffMeshConnection) * off_mesh_cons_len);

    const data_size = header_size + verts_size + polys_size + links_size +
        detail_meshes_size + detail_verts_size + detail_tris_size +
        bv_tree_size + off_mesh_cons_size;

    // A polygon of fewer than two vertices takes room away from the fans of
    // the others, which then run on into the blocks that follow. They must
    // not run past the tile.
    if (@sizeOf([4]u8) * fan_tri_count > detail_tris_size + bv_tree_size + off_mesh_cons_size) {
        return error.InvalidParam;
    }

    const data = try gpa.alignedAlloc(u8, .@"4", data_size);
    errdefer gpa.free(data);
    @memset(data, 0);

    var offset: usize = 0;
    const header: *MeshHeader = @ptrCast(data.ptr);
    offset += header_size;
    const nav_verts = blobSlice([3]f32, data, offset, verts_len);
    offset += verts_size;
    const nav_polys = blobSlice(Poly, data, offset, polys_len);
    offset += polys_size;
    // The links stay zero: the navigation mesh makes them when the tile is
    // added, in the room left here.
    offset += links_size;
    const nav_detail_meshes = blobSlice(PolyDetail, data, offset, poly_count);
    offset += detail_meshes_size;
    const nav_detail_verts = blobSlice([3]f32, data, offset, detail_verts_len);
    offset += detail_verts_size;
    const detail_tris_offset = offset;
    offset += detail_tris_size;
    const nav_bv_tree = blobSlice(BvNode, data, offset, bv_nodes_len);
    offset += bv_tree_size;
    const off_mesh_cons = blobSlice(OffMeshConnection, data, offset, off_mesh_cons_len);

    // Store the header.
    header.magic = nav_mesh.nav_mesh_magic;
    header.version = nav_mesh.nav_mesh_version;
    header.x = params.tile_x;
    header.y = params.tile_y;
    header.layer = params.tile_layer;
    header.user_id = params.user_id;
    header.poly_count = total_poly_count;
    header.vert_count = total_vert_count;
    header.link_count_max = link_count_max;
    header.bounds_min = params.bounds_min;
    header.bounds_max = params.bounds_max;
    header.detail_mesh_count = params.poly_count;
    header.detail_vert_count = unique_detail_vert_count;
    header.detail_tri_count = detail_tri_count;
    header.bv_quant_factor = 1.0 / params.cell_size;
    header.off_mesh_base = params.poly_count;
    header.walkable_height = params.walkable_height;
    header.walkable_radius = params.walkable_radius;
    header.walkable_climb = params.walkable_climb;
    header.off_mesh_connection_count = stored_off_mesh_con_count;
    header.bv_node_count = if (params.build_bv_tree) params.poly_count * 2 else 0;

    // Store the vertices: the mesh's, in world units.
    for (params.verts[0..vert_count], nav_verts[0..vert_count]) |source, *v| {
        const x: f32 = @floatFromInt(source[0]);
        const y: f32 = @floatFromInt(source[1]);
        const z: f32 = @floatFromInt(source[2]);
        v[0] = params.bounds_min[0] + x * params.cell_size;
        v[1] = params.bounds_min[1] + y * params.cell_height;
        v[2] = params.bounds_min[2] + z * params.cell_size;
    }
    // Then the end points of the off-mesh connections that start in this
    // tile. `n` counts the connections stored so far.
    var n: usize = 0;
    for (0..off_mesh_con_count) |i| {
        if (off_mesh_con_class[i * 2 + 0] == 0xff) {
            nav_verts[vert_count + n * 2 + 0] = params.off_mesh_con_verts[i][0];
            nav_verts[vert_count + n * 2 + 1] = params.off_mesh_con_verts[i][1];
            n += 1;
        }
    }

    // Store the polygons: the mesh's.
    for (0..poly_count) |i| {
        const source = params.polys[i * vertices_per_polygon * 2 ..];
        const p = &nav_polys[i];
        p.vert_count = 0;
        p.flags = params.poly_flags[i];
        p.setArea(params.poly_areas[i]);
        p.setType(.ground);
        for (0..vertices_per_polygon) |j| {
            if (source[j] == mesh_null_index) break;
            p.verts[j] = source[j];
            const neighbour = source[vertices_per_polygon + j];
            if (neighbour & 0x8000 != 0) {
                // A wall, or a portal on the side of the tile the edge's
                // direction names.
                switch (neighbour & 0xf) {
                    0xf => p.neighbours[j] = 0,
                    0 => p.neighbours[j] = nav_mesh.ext_link | 4, // x-
                    1 => p.neighbours[j] = nav_mesh.ext_link | 2, // z+
                    2 => p.neighbours[j] = nav_mesh.ext_link | 0, // x+
                    3 => p.neighbours[j] = nav_mesh.ext_link | 6, // z-
                    else => {},
                }
            } else {
                // A polygon of this tile.
                p.neighbours[j] = neighbour + 1;
            }

            p.vert_count += 1;
        }
    }
    // Then one polygon per stored off-mesh connection.
    n = 0;
    for (0..off_mesh_con_count) |i| {
        if (off_mesh_con_class[i * 2 + 0] == 0xff) {
            const p = &nav_polys[poly_count + n];
            p.vert_count = 2;
            p.verts[0] = @truncate(vert_count + n * 2 + 0);
            p.verts[1] = @truncate(vert_count + n * 2 + 1);
            p.flags = params.off_mesh_con_flags[i];
            p.setArea(params.off_mesh_con_areas[i]);
            p.setType(.off_mesh_connection);
            n += 1;
        }
    }

    // Store the detail meshes and vertices. A detail mesh's first vertices
    // are its polygon's, which the tile already holds, so only the rest are
    // stored.
    if (params.detail_meshes) |detail_meshes| {
        var vert_base: u16 = 0;
        for (0..poly_count) |i| {
            const detail = &nav_detail_meshes[i];
            const source_vert_base: usize = detail_meshes[i][0];
            const source_vert_count: usize = detail_meshes[i][1];
            const nv: usize = nav_polys[i].vert_count;
            // Never negative: the counting pass above refused that.
            const own_vert_count = source_vert_count - nv;
            detail.vert_base = vert_base;
            detail.vert_count = @truncate(own_vert_count);
            detail.tri_base = detail_meshes[i][2];
            detail.tri_count = @truncate(detail_meshes[i][3]);
            if (own_vert_count != 0) {
                @memcpy(
                    nav_detail_verts[vert_base..][0..own_vert_count],
                    params.detail_verts[source_vert_base + nv ..][0..own_vert_count],
                );
                vert_base +%= @as(u16, @truncate(own_vert_count));
            }
        }
        // Store the triangles.
        const nav_detail_tris = blobSlice([4]u8, data, detail_tris_offset, detail_tris_len);
        @memcpy(nav_detail_tris, params.detail_tris[0..detail_tris_len]);
    } else {
        // Make a flat detail mesh by triangulating each polygon as a fan.
        var tri_base: i32 = 0;
        for (0..poly_count) |i| {
            const detail = &nav_detail_meshes[i];
            const nv: i32 = nav_polys[i].vert_count;
            detail.vert_base = 0;
            detail.vert_count = 0;
            detail.tri_base = @intCast(tri_base);
            detail.tri_count = @truncate(@as(u32, @bitCast(nv - 2)));
            var j: i32 = 2;
            while (j < nv) : (j += 1) {
                // Placed by byte offset, not inside the triangles' block:
                // polygons of fewer than two vertices shrink the block, and
                // the triangles then run on into the next one.
                const tri_offset = detail_tris_offset + @as(usize, @intCast(tri_base)) * 4;
                const t = data[tri_offset..][0..4];
                t[0] = 0;
                t[1] = @intCast(j - 1);
                t[2] = @intCast(j);
                // A bit for each edge that lies on the polygon's boundary.
                t[3] = 1 << 2;
                if (j == 2) t[3] |= 1 << 0;
                if (j == nv - 1) t[3] |= 1 << 4;
                tri_base += 1;
            }
        }
    }

    // Build and store the bounding-volume tree.
    if (params.build_bv_tree) try createBvTree(gpa, params, nav_bv_tree);

    // Store the off-mesh connections.
    n = 0;
    for (0..off_mesh_con_count) |i| {
        if (off_mesh_con_class[i * 2 + 0] == 0xff) {
            const con = &off_mesh_cons[n];
            con.poly = @truncate(poly_count + n);
            con.pos = params.off_mesh_con_verts[i];
            con.radius = params.off_mesh_con_rad[i];
            con.flags = if (params.off_mesh_con_dir[i] != 0)
                nav_mesh.off_mesh_connection_bidirectional
            else
                0;
            con.side = off_mesh_con_class[i * 2 + 1];
            if (params.off_mesh_con_user_id) |user_ids| con.user_id = user_ids[i];
            n += 1;
        }
    }

    return data;
}

/// Swaps the byte order of the header of a tile's bytes, in place. False,
/// with nothing changed, when `data` is too short for a header or the header
/// has the magic and version of this format in neither byte order.
///
/// Bytes from a machine of the other byte order take this first, then
/// `navMeshDataSwapEndian`; bytes going to one take them the other way
/// round.
pub fn navMeshHeaderSwapEndian(data: []u8) bool {
    if (data.len < @sizeOf(MeshHeader)) return false;
    const header = std.mem.bytesAsValue(MeshHeader, data[0..@sizeOf(MeshHeader)]);

    const swapped_magic = @byteSwap(nav_mesh.nav_mesh_magic);
    const swapped_version = @byteSwap(nav_mesh.nav_mesh_version);

    const is_native = header.magic == nav_mesh.nav_mesh_magic and
        header.version == nav_mesh.nav_mesh_version;
    const is_swapped = header.magic == swapped_magic and header.version == swapped_version;
    if (!is_native and !is_swapped) return false;

    header.magic = @byteSwap(header.magic);
    header.version = @byteSwap(header.version);
    header.x = @byteSwap(header.x);
    header.y = @byteSwap(header.y);
    header.layer = @byteSwap(header.layer);
    header.user_id = @byteSwap(header.user_id);
    header.poly_count = @byteSwap(header.poly_count);
    header.vert_count = @byteSwap(header.vert_count);
    header.link_count_max = @byteSwap(header.link_count_max);
    header.detail_mesh_count = @byteSwap(header.detail_mesh_count);
    header.detail_vert_count = @byteSwap(header.detail_vert_count);
    header.detail_tri_count = @byteSwap(header.detail_tri_count);
    header.bv_node_count = @byteSwap(header.bv_node_count);
    header.off_mesh_connection_count = @byteSwap(header.off_mesh_connection_count);
    header.off_mesh_base = @byteSwap(header.off_mesh_base);
    header.walkable_height = common.swapEndianFloat(header.walkable_height);
    header.walkable_radius = common.swapEndianFloat(header.walkable_radius);
    header.walkable_climb = common.swapEndianFloat(header.walkable_climb);
    for (0..3) |i| header.bounds_min[i] = common.swapEndianFloat(header.bounds_min[i]);
    for (0..3) |i| header.bounds_max[i] = common.swapEndianFloat(header.bounds_max[i]);
    header.bv_quant_factor = common.swapEndianFloat(header.bv_quant_factor);

    return true;
}

/// The byte order that swaps a record of type `T`: `halfword_count`
/// halfwords from `start`, then `word_count` words; every other byte stays.
fn halfwordOrder(
    comptime T: type,
    comptime start: usize,
    comptime halfword_count: usize,
    comptime word_count: usize,
) [@sizeOf(T)]i32 {
    var order: [@sizeOf(T)]i32 = undefined;
    for (&order, 0..) |*index, i| index.* = i;
    for (0..halfword_count) |h| {
        order[start + 2 * h] = start + 2 * h + 1;
        order[start + 2 * h + 1] = start + 2 * h;
    }
    const words_start = start + 2 * halfword_count;
    for (0..word_count) |w| {
        for (0..4) |b| order[words_start + 4 * w + b] = words_start + 4 * w + 3 - b;
    }
    return order;
}

/// Reorders the bytes of each record of type `T` in `bytes` by `order`, one
/// record as one vector.
fn swapRecords(comptime T: type, bytes: []u8, comptime order: [@sizeOf(T)]i32) void {
    const size = @sizeOf(T);
    var offset: usize = 0;
    while (offset < bytes.len) : (offset += size) {
        const record: *align(1) [size]u8 = bytes[offset..][0..size];
        const vector: @Vector(size, u8) = record.*;
        record.* = @shuffle(u8, vector, undefined, order);
    }
}

/// Reverses the bytes of every 4-byte word of `bytes`, whose length is a
/// multiple of four: sixteen words at a time, then one at a time.
fn swapWords(bytes: []u8) void {
    const lanes = 16;
    const chunk_size = lanes * 4;
    var offset: usize = 0;
    while (offset + chunk_size <= bytes.len) : (offset += chunk_size) {
        const words: *align(1) [lanes]u32 = @ptrCast(bytes[offset..][0..chunk_size]);
        const vector: @Vector(lanes, u32) = words.*;
        words.* = @byteSwap(vector);
    }
    while (offset < bytes.len) : (offset += 4) {
        const word: *align(1) u32 = @ptrCast(bytes[offset..][0..4]);
        word.* = @byteSwap(word.*);
    }
}

/// Swaps the byte order of everything after the header of a tile's bytes,
/// in place. The header must be in this machine's byte order, since its
/// counts say where the blocks are: see `navMeshHeaderSwapEndian` for the
/// order of the two calls. False, with nothing changed, when the header is
/// not this format's or `data` is shorter than the header's counts need.
///
/// The links are left as they are, since the navigation mesh rewrites them
/// when the tile is added, and so is each off-mesh connection's `user_id`.
pub fn navMeshDataSwapEndian(data: []u8) bool {
    // Make sure the bytes are a tile of this format.
    if (data.len < @sizeOf(MeshHeader)) return false;
    const header = std.mem.bytesAsValue(MeshHeader, data[0..@sizeOf(MeshHeader)]);
    if (header.magic != nav_mesh.nav_mesh_magic) return false;
    if (header.version != nav_mesh.nav_mesh_version) return false;

    const cast = std.math.cast;
    const vert_count = cast(usize, header.vert_count) orelse return false;
    const poly_count = cast(usize, header.poly_count) orelse return false;
    const link_count_max = cast(usize, header.link_count_max) orelse return false;
    const detail_mesh_count = cast(usize, header.detail_mesh_count) orelse return false;
    const detail_vert_count = cast(usize, header.detail_vert_count) orelse return false;
    const detail_tri_count = cast(usize, header.detail_tri_count) orelse return false;
    const bv_node_count = cast(usize, header.bv_node_count) orelse return false;
    const off_mesh_con_count = cast(usize, header.off_mesh_connection_count) orelse
        return false;

    // Find the blocks.
    const len = data.len;
    const blockSize = nav_mesh.blockSize;
    const header_size = common.align4(@sizeOf(MeshHeader));
    const verts_size = blockSize([3]f32, vert_count, len) orelse return false;
    const polys_size = blockSize(Poly, poly_count, len) orelse return false;
    const links_size = blockSize(Link, link_count_max, len) orelse return false;
    const detail_meshes_size = blockSize(PolyDetail, detail_mesh_count, len) orelse return false;
    const detail_verts_size = blockSize([3]f32, detail_vert_count, len) orelse return false;
    const detail_tris_size = blockSize([4]u8, detail_tri_count, len) orelse return false;
    const bv_tree_size = blockSize(BvNode, bv_node_count, len) orelse return false;
    const off_mesh_cons_size = blockSize(OffMeshConnection, off_mesh_con_count, len) orelse
        return false;

    const data_size = @as(u64, header_size) + verts_size + polys_size + links_size +
        detail_meshes_size + detail_verts_size + detail_tris_size +
        bv_tree_size + off_mesh_cons_size;
    if (data.len < data_size) return false;

    // `data` may sit at any address, so every record is reached through a
    // pointer that asks for no alignment.
    const bytesAsSlice = std.mem.bytesAsSlice;
    var offset = header_size;
    const verts = data[offset..][0 .. @sizeOf([3]f32) * vert_count];
    offset += verts_size;
    const polys = data[offset..][0 .. @sizeOf(Poly) * poly_count];
    offset += polys_size;
    offset += links_size;
    const detail_meshes = data[offset..][0 .. @sizeOf(PolyDetail) * detail_mesh_count];
    offset += detail_meshes_size;
    const detail_verts = data[offset..][0 .. @sizeOf([3]f32) * detail_vert_count];
    offset += detail_verts_size;
    // The detail triangles are single bytes, which have no byte order.
    offset += detail_tris_size;
    const bv_tree = data[offset..][0 .. @sizeOf(BvNode) * bv_node_count];
    offset += bv_tree_size;
    const off_mesh_cons = bytesAsSlice(
        OffMeshConnection,
        data[offset..][0 .. @sizeOf(OffMeshConnection) * off_mesh_con_count],
    );

    // Vertices.
    swapWords(verts);

    // Polygons: the vertices, neighbours and flags. `first_link` is set when
    // the tile is added.
    swapRecords(Poly, polys, comptime halfwordOrder(Poly, @offsetOf(Poly, "verts"), 13, 0));

    // Detail meshes: the bases of their vertices and triangles.
    swapRecords(PolyDetail, detail_meshes, comptime halfwordOrder(PolyDetail, 0, 0, 2));

    // Detail vertices.
    swapWords(detail_verts);

    // The bounding-volume tree: six halfwords of bounds and a word of index.
    swapRecords(BvNode, bv_tree, comptime halfwordOrder(BvNode, 0, 6, 1));

    // Off-mesh connections.
    for (off_mesh_cons) |*con| {
        for (0..2) |end| {
            for (0..3) |j| con.pos[end][j] = common.swapEndianFloat(con.pos[end][j]);
        }
        con.radius = common.swapEndianFloat(con.radius);
        con.poly = @byteSwap(con.poly);
    }

    return true;
}
