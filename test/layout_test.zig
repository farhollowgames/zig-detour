//! Differential tests of layouts and constants: the port's stored structs
//! have the sizes and field offsets of upstream's, its flags and constants
//! have upstream's values, and the bit helpers of a polygon and a status
//! give upstream's answers.

const std = @import("std");
const detour = @import("detour");
const support = @import("support.zig");
const reference = support.reference;

const testing = std.testing;

const nav_mesh = detour.nav_mesh;
const Status = detour.Status;

// ---------------------------------------------------------------------------
// Comparisons
// ---------------------------------------------------------------------------

/// The bits of a flags or status value, widened to 32 bits.
fn bitsOf(value: anytype) u32 {
    const Bits = @Int(.unsigned, @bitSizeOf(@TypeOf(value)));
    const bits: Bits = @bitCast(value);
    return bits;
}

/// Fails unless `T` is `expected` bytes in size.
fn expectSize(comptime T: type, expected: usize) !void {
    if (expected != @sizeOf(T)) {
        std.debug.print("{s}: original size {d}, port {d}\n", .{
            @typeName(T),
            expected,
            @sizeOf(T),
        });
        return error.TestExpectedEqual;
    }
}

/// Fails unless field `field` of `T` sits `expected` bytes from the start.
fn expectOffset(comptime T: type, comptime field: []const u8, expected: usize) !void {
    if (expected != @offsetOf(T, field)) {
        std.debug.print("{s}.{s}: original offset {d}, port {d}\n", .{
            @typeName(T),
            field,
            expected,
            @offsetOf(T, field),
        });
        return error.TestExpectedEqual;
    }
}

/// Fails unless a flags or status value has the bits of upstream's constant.
fn expectFlag(comptime name: []const u8, expected: u32, value: anytype) !void {
    const actual = bitsOf(value);
    if (expected != actual) {
        std.debug.print("{s}: original 0x{x:0>8}, port 0x{x:0>8}\n", .{ name, expected, actual });
        return error.TestExpectedEqual;
    }
}

/// Fails unless the port's integer constant has upstream's value.
fn expectConstant(comptime name: []const u8, expected: i64, actual: i64) !void {
    if (expected != actual) {
        std.debug.print("{s}: original {d}, port {d}\n", .{ name, expected, actual });
        return error.TestExpectedEqual;
    }
}

// ---------------------------------------------------------------------------
// Sizes and field offsets of the stored structs
// ---------------------------------------------------------------------------

test "PolyRef and TileRef: widths equal upstream's" {
    try expectSize(detour.PolyRef, reference.ref_sizeof_poly_ref());
    try expectSize(detour.TileRef, reference.ref_sizeof_tile_ref());
}

test "Poly: size and field offsets equal upstream's dtPoly" {
    const T = nav_mesh.Poly;
    try expectSize(T, reference.ref_sizeof_poly());
    try expectOffset(T, "first_link", reference.ref_offsetof_poly_first_link());
    try expectOffset(T, "verts", reference.ref_offsetof_poly_verts());
    try expectOffset(T, "neighbours", reference.ref_offsetof_poly_neis());
    try expectOffset(T, "flags", reference.ref_offsetof_poly_flags());
    try expectOffset(T, "vert_count", reference.ref_offsetof_poly_vert_count());
    try expectOffset(T, "area_and_type", reference.ref_offsetof_poly_area_and_type());
}

test "PolyDetail: size and field offsets equal upstream's dtPolyDetail" {
    const T = nav_mesh.PolyDetail;
    try expectSize(T, reference.ref_sizeof_poly_detail());
    try expectOffset(T, "vert_base", reference.ref_offsetof_poly_detail_vert_base());
    try expectOffset(T, "tri_base", reference.ref_offsetof_poly_detail_tri_base());
    try expectOffset(T, "vert_count", reference.ref_offsetof_poly_detail_vert_count());
    try expectOffset(T, "tri_count", reference.ref_offsetof_poly_detail_tri_count());

    // The port's padding field takes the two bytes upstream's compiler
    // leaves after the last field, so the record is zero there in every copy.
    try expectOffset(T, "padding", reference.ref_offsetof_poly_detail_tri_count() + 1);
    try testing.expectEqual(@as(usize, 2), @sizeOf(@FieldType(T, "padding")));
    try testing.expectEqual(reference.ref_sizeof_poly_detail(), @offsetOf(T, "padding") + 2);
}

test "Link: size and field offsets equal upstream's dtLink" {
    const T = nav_mesh.Link;
    try expectSize(T, reference.ref_sizeof_link());
    try expectOffset(T, "ref", reference.ref_offsetof_link_ref());
    try expectOffset(T, "next", reference.ref_offsetof_link_next());
    try expectOffset(T, "edge", reference.ref_offsetof_link_edge());
    try expectOffset(T, "side", reference.ref_offsetof_link_side());
    try expectOffset(T, "bounds_min", reference.ref_offsetof_link_bmin());
    try expectOffset(T, "bounds_max", reference.ref_offsetof_link_bmax());
}

test "BvNode: size and field offsets equal upstream's dtBVNode" {
    const T = nav_mesh.BvNode;
    try expectSize(T, reference.ref_sizeof_bv_node());
    try expectOffset(T, "bounds_min", reference.ref_offsetof_bv_node_bmin());
    try expectOffset(T, "bounds_max", reference.ref_offsetof_bv_node_bmax());
    try expectOffset(T, "index", reference.ref_offsetof_bv_node_i());
}

test "OffMeshConnection: size and field offsets equal upstream's dtOffMeshConnection" {
    const T = nav_mesh.OffMeshConnection;
    try expectSize(T, reference.ref_sizeof_off_mesh_connection());
    try expectOffset(T, "pos", reference.ref_offsetof_off_mesh_connection_pos());
    try expectOffset(T, "radius", reference.ref_offsetof_off_mesh_connection_rad());
    try expectOffset(T, "poly", reference.ref_offsetof_off_mesh_connection_poly());
    try expectOffset(T, "flags", reference.ref_offsetof_off_mesh_connection_flags());
    try expectOffset(T, "side", reference.ref_offsetof_off_mesh_connection_side());
    try expectOffset(T, "user_id", reference.ref_offsetof_off_mesh_connection_user_id());

    // Upstream's `pos` is six floats; the port's is two points.
    try testing.expectEqual(@as(usize, 6 * @sizeOf(f32)), @sizeOf(@FieldType(T, "pos")));
}

test "MeshHeader: size and field offsets equal upstream's dtMeshHeader" {
    const T = nav_mesh.MeshHeader;
    try expectSize(T, reference.ref_sizeof_mesh_header());
    try expectOffset(T, "magic", reference.ref_offsetof_mesh_header_magic());
    try expectOffset(T, "version", reference.ref_offsetof_mesh_header_version());
    try expectOffset(T, "x", reference.ref_offsetof_mesh_header_x());
    try expectOffset(T, "y", reference.ref_offsetof_mesh_header_y());
    try expectOffset(T, "layer", reference.ref_offsetof_mesh_header_layer());
    try expectOffset(T, "user_id", reference.ref_offsetof_mesh_header_user_id());
    try expectOffset(T, "poly_count", reference.ref_offsetof_mesh_header_poly_count());
    try expectOffset(T, "vert_count", reference.ref_offsetof_mesh_header_vert_count());
    try expectOffset(T, "link_count_max", reference.ref_offsetof_mesh_header_max_link_count());
    try expectOffset(
        T,
        "detail_mesh_count",
        reference.ref_offsetof_mesh_header_detail_mesh_count(),
    );
    try expectOffset(
        T,
        "detail_vert_count",
        reference.ref_offsetof_mesh_header_detail_vert_count(),
    );
    try expectOffset(
        T,
        "detail_tri_count",
        reference.ref_offsetof_mesh_header_detail_tri_count(),
    );
    try expectOffset(T, "bv_node_count", reference.ref_offsetof_mesh_header_bv_node_count());
    try expectOffset(
        T,
        "off_mesh_connection_count",
        reference.ref_offsetof_mesh_header_off_mesh_con_count(),
    );
    try expectOffset(T, "off_mesh_base", reference.ref_offsetof_mesh_header_off_mesh_base());
    try expectOffset(T, "walkable_height", reference.ref_offsetof_mesh_header_walkable_height());
    try expectOffset(T, "walkable_radius", reference.ref_offsetof_mesh_header_walkable_radius());
    try expectOffset(T, "walkable_climb", reference.ref_offsetof_mesh_header_walkable_climb());
    try expectOffset(T, "bounds_min", reference.ref_offsetof_mesh_header_bmin());
    try expectOffset(T, "bounds_max", reference.ref_offsetof_mesh_header_bmax());
    try expectOffset(T, "bv_quant_factor", reference.ref_offsetof_mesh_header_bv_quant_factor());
}

test "NavMeshParams: size and field offsets equal upstream's dtNavMeshParams" {
    const T = nav_mesh.NavMeshParams;
    try expectSize(T, reference.ref_sizeof_nav_mesh_params());
    try expectOffset(T, "origin", reference.ref_offsetof_nav_mesh_params_orig());
    try expectOffset(T, "tile_width", reference.ref_offsetof_nav_mesh_params_tile_width());
    try expectOffset(T, "tile_height", reference.ref_offsetof_nav_mesh_params_tile_height());
    try expectOffset(T, "tile_count_max", reference.ref_offsetof_nav_mesh_params_max_tiles());
    try expectOffset(T, "poly_count_max", reference.ref_offsetof_nav_mesh_params_max_polys());
}

// ---------------------------------------------------------------------------
// The test bindings' own mirrors of upstream's structs
// ---------------------------------------------------------------------------

// The polygon byte test below runs the original's functions on these mirrors,
// so they must have upstream's layout too.

test "mirrors of dtPoly, dtPolyDetail, dtLink and dtBVNode have upstream's layout" {
    const poly = reference.Poly;
    try expectSize(poly, reference.ref_sizeof_poly());
    try expectOffset(poly, "first_link", reference.ref_offsetof_poly_first_link());
    try expectOffset(poly, "verts", reference.ref_offsetof_poly_verts());
    try expectOffset(poly, "neis", reference.ref_offsetof_poly_neis());
    try expectOffset(poly, "flags", reference.ref_offsetof_poly_flags());
    try expectOffset(poly, "vert_count", reference.ref_offsetof_poly_vert_count());
    try expectOffset(poly, "area_and_type", reference.ref_offsetof_poly_area_and_type());

    const detail = reference.PolyDetail;
    try expectSize(detail, reference.ref_sizeof_poly_detail());
    try expectOffset(detail, "vert_base", reference.ref_offsetof_poly_detail_vert_base());
    try expectOffset(detail, "tri_base", reference.ref_offsetof_poly_detail_tri_base());
    try expectOffset(detail, "vert_count", reference.ref_offsetof_poly_detail_vert_count());
    try expectOffset(detail, "tri_count", reference.ref_offsetof_poly_detail_tri_count());

    const link = reference.Link;
    try expectSize(link, reference.ref_sizeof_link());
    try expectOffset(link, "ref", reference.ref_offsetof_link_ref());
    try expectOffset(link, "next", reference.ref_offsetof_link_next());
    try expectOffset(link, "edge", reference.ref_offsetof_link_edge());
    try expectOffset(link, "side", reference.ref_offsetof_link_side());
    try expectOffset(link, "bmin", reference.ref_offsetof_link_bmin());
    try expectOffset(link, "bmax", reference.ref_offsetof_link_bmax());

    const node = reference.BVNode;
    try expectSize(node, reference.ref_sizeof_bv_node());
    try expectOffset(node, "bmin", reference.ref_offsetof_bv_node_bmin());
    try expectOffset(node, "bmax", reference.ref_offsetof_bv_node_bmax());
    try expectOffset(node, "i", reference.ref_offsetof_bv_node_i());
}

test "mirrors of dtOffMeshConnection, dtMeshHeader and dtNavMeshParams have upstream's layout" {
    const connection = reference.OffMeshConnection;
    try expectSize(connection, reference.ref_sizeof_off_mesh_connection());
    try expectOffset(connection, "pos", reference.ref_offsetof_off_mesh_connection_pos());
    try expectOffset(connection, "rad", reference.ref_offsetof_off_mesh_connection_rad());
    try expectOffset(connection, "poly", reference.ref_offsetof_off_mesh_connection_poly());
    try expectOffset(connection, "flags", reference.ref_offsetof_off_mesh_connection_flags());
    try expectOffset(connection, "side", reference.ref_offsetof_off_mesh_connection_side());
    try expectOffset(connection, "user_id", reference.ref_offsetof_off_mesh_connection_user_id());

    const header = reference.MeshHeader;
    try expectSize(header, reference.ref_sizeof_mesh_header());
    try expectOffset(header, "magic", reference.ref_offsetof_mesh_header_magic());
    try expectOffset(header, "version", reference.ref_offsetof_mesh_header_version());
    try expectOffset(header, "x", reference.ref_offsetof_mesh_header_x());
    try expectOffset(header, "y", reference.ref_offsetof_mesh_header_y());
    try expectOffset(header, "layer", reference.ref_offsetof_mesh_header_layer());
    try expectOffset(header, "user_id", reference.ref_offsetof_mesh_header_user_id());
    try expectOffset(header, "poly_count", reference.ref_offsetof_mesh_header_poly_count());
    try expectOffset(header, "vert_count", reference.ref_offsetof_mesh_header_vert_count());
    try expectOffset(header, "max_link_count", reference.ref_offsetof_mesh_header_max_link_count());
    try expectOffset(
        header,
        "detail_mesh_count",
        reference.ref_offsetof_mesh_header_detail_mesh_count(),
    );
    try expectOffset(
        header,
        "detail_vert_count",
        reference.ref_offsetof_mesh_header_detail_vert_count(),
    );
    try expectOffset(
        header,
        "detail_tri_count",
        reference.ref_offsetof_mesh_header_detail_tri_count(),
    );
    try expectOffset(header, "bv_node_count", reference.ref_offsetof_mesh_header_bv_node_count());
    try expectOffset(
        header,
        "off_mesh_con_count",
        reference.ref_offsetof_mesh_header_off_mesh_con_count(),
    );
    try expectOffset(header, "off_mesh_base", reference.ref_offsetof_mesh_header_off_mesh_base());
    try expectOffset(
        header,
        "walkable_height",
        reference.ref_offsetof_mesh_header_walkable_height(),
    );
    try expectOffset(
        header,
        "walkable_radius",
        reference.ref_offsetof_mesh_header_walkable_radius(),
    );
    try expectOffset(
        header,
        "walkable_climb",
        reference.ref_offsetof_mesh_header_walkable_climb(),
    );
    try expectOffset(header, "bmin", reference.ref_offsetof_mesh_header_bmin());
    try expectOffset(header, "bmax", reference.ref_offsetof_mesh_header_bmax());
    try expectOffset(
        header,
        "bv_quant_factor",
        reference.ref_offsetof_mesh_header_bv_quant_factor(),
    );

    const params = reference.NavMeshParams;
    try expectSize(params, reference.ref_sizeof_nav_mesh_params());
    try expectOffset(params, "orig", reference.ref_offsetof_nav_mesh_params_orig());
    try expectOffset(params, "tile_width", reference.ref_offsetof_nav_mesh_params_tile_width());
    try expectOffset(params, "tile_height", reference.ref_offsetof_nav_mesh_params_tile_height());
    try expectOffset(params, "max_tiles", reference.ref_offsetof_nav_mesh_params_max_tiles());
    try expectOffset(params, "max_polys", reference.ref_offsetof_nav_mesh_params_max_polys());
}

test "mirrors of dtNavMeshCreateParams and dtRaycastHit have upstream's layout" {
    const create = reference.NavMeshCreateParams;
    try expectSize(create, reference.ref_sizeof_nav_mesh_create_params());
    try expectOffset(create, "verts", reference.ref_offsetof_nav_mesh_create_params_verts());
    try expectOffset(
        create,
        "vert_count",
        reference.ref_offsetof_nav_mesh_create_params_vert_count(),
    );
    try expectOffset(create, "polys", reference.ref_offsetof_nav_mesh_create_params_polys());
    try expectOffset(
        create,
        "poly_flags",
        reference.ref_offsetof_nav_mesh_create_params_poly_flags(),
    );
    try expectOffset(
        create,
        "poly_areas",
        reference.ref_offsetof_nav_mesh_create_params_poly_areas(),
    );
    try expectOffset(
        create,
        "poly_count",
        reference.ref_offsetof_nav_mesh_create_params_poly_count(),
    );
    try expectOffset(create, "nvp", reference.ref_offsetof_nav_mesh_create_params_nvp());
    try expectOffset(
        create,
        "detail_meshes",
        reference.ref_offsetof_nav_mesh_create_params_detail_meshes(),
    );
    try expectOffset(
        create,
        "detail_verts",
        reference.ref_offsetof_nav_mesh_create_params_detail_verts(),
    );
    try expectOffset(
        create,
        "detail_verts_count",
        reference.ref_offsetof_nav_mesh_create_params_detail_verts_count(),
    );
    try expectOffset(
        create,
        "detail_tris",
        reference.ref_offsetof_nav_mesh_create_params_detail_tris(),
    );
    try expectOffset(
        create,
        "detail_tri_count",
        reference.ref_offsetof_nav_mesh_create_params_detail_tri_count(),
    );
    try expectOffset(
        create,
        "off_mesh_con_verts",
        reference.ref_offsetof_nav_mesh_create_params_off_mesh_con_verts(),
    );
    try expectOffset(
        create,
        "off_mesh_con_rad",
        reference.ref_offsetof_nav_mesh_create_params_off_mesh_con_rad(),
    );
    try expectOffset(
        create,
        "off_mesh_con_flags",
        reference.ref_offsetof_nav_mesh_create_params_off_mesh_con_flags(),
    );
    try expectOffset(
        create,
        "off_mesh_con_areas",
        reference.ref_offsetof_nav_mesh_create_params_off_mesh_con_areas(),
    );
    try expectOffset(
        create,
        "off_mesh_con_dir",
        reference.ref_offsetof_nav_mesh_create_params_off_mesh_con_dir(),
    );
    try expectOffset(
        create,
        "off_mesh_con_user_id",
        reference.ref_offsetof_nav_mesh_create_params_off_mesh_con_user_id(),
    );
    try expectOffset(
        create,
        "off_mesh_con_count",
        reference.ref_offsetof_nav_mesh_create_params_off_mesh_con_count(),
    );
    try expectOffset(create, "user_id", reference.ref_offsetof_nav_mesh_create_params_user_id());
    try expectOffset(create, "tile_x", reference.ref_offsetof_nav_mesh_create_params_tile_x());
    try expectOffset(create, "tile_y", reference.ref_offsetof_nav_mesh_create_params_tile_y());
    try expectOffset(
        create,
        "tile_layer",
        reference.ref_offsetof_nav_mesh_create_params_tile_layer(),
    );
    try expectOffset(create, "bmin", reference.ref_offsetof_nav_mesh_create_params_bmin());
    try expectOffset(create, "bmax", reference.ref_offsetof_nav_mesh_create_params_bmax());
    try expectOffset(
        create,
        "walkable_height",
        reference.ref_offsetof_nav_mesh_create_params_walkable_height(),
    );
    try expectOffset(
        create,
        "walkable_radius",
        reference.ref_offsetof_nav_mesh_create_params_walkable_radius(),
    );
    try expectOffset(
        create,
        "walkable_climb",
        reference.ref_offsetof_nav_mesh_create_params_walkable_climb(),
    );
    try expectOffset(create, "cs", reference.ref_offsetof_nav_mesh_create_params_cs());
    try expectOffset(create, "ch", reference.ref_offsetof_nav_mesh_create_params_ch());
    try expectOffset(
        create,
        "build_bv_tree",
        reference.ref_offsetof_nav_mesh_create_params_build_bv_tree(),
    );

    const hit = reference.RaycastHit;
    try expectSize(hit, reference.ref_sizeof_raycast_hit());
    try expectOffset(hit, "t", reference.ref_offsetof_raycast_hit_t());
    try expectOffset(hit, "hit_normal", reference.ref_offsetof_raycast_hit_hit_normal());
    try expectOffset(hit, "hit_edge_index", reference.ref_offsetof_raycast_hit_hit_edge_index());
    try expectOffset(hit, "path", reference.ref_offsetof_raycast_hit_path());
    try expectOffset(hit, "path_count", reference.ref_offsetof_raycast_hit_path_count());
    try expectOffset(hit, "max_path", reference.ref_offsetof_raycast_hit_max_path());
    try expectOffset(hit, "path_cost", reference.ref_offsetof_raycast_hit_path_cost());
}

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

test "Status: every bit and mask equals upstream's DT_ constant" {
    try expectFlag(
        "wrong_magic",
        reference.ref_const_wrong_magic(),
        Status{ .wrong_magic = true },
    );
    try expectFlag(
        "wrong_version",
        reference.ref_const_wrong_version(),
        Status{ .wrong_version = true },
    );
    try expectFlag(
        "out_of_memory",
        reference.ref_const_out_of_memory(),
        Status{ .out_of_memory = true },
    );
    try expectFlag(
        "invalid_param",
        reference.ref_const_invalid_param(),
        Status{ .invalid_param = true },
    );
    try expectFlag(
        "buffer_too_small",
        reference.ref_const_buffer_too_small(),
        Status{ .buffer_too_small = true },
    );
    try expectFlag(
        "out_of_nodes",
        reference.ref_const_out_of_nodes(),
        Status{ .out_of_nodes = true },
    );
    try expectFlag(
        "partial_result",
        reference.ref_const_partial_result(),
        Status{ .partial_result = true },
    );
    try expectFlag(
        "already_occupied",
        reference.ref_const_already_occupied(),
        Status{ .already_occupied = true },
    );
    try expectFlag(
        "in_progress",
        reference.ref_const_in_progress(),
        Status{ .in_progress = true },
    );
    try expectFlag("success", reference.ref_const_success(), Status.success);
    try expectFlag("failure", reference.ref_const_failure(), Status.failure);
    try expectConstant(
        "detail_mask",
        reference.ref_const_status_detail_mask(),
        Status.detail_mask,
    );

    // The success and failure constants are one outcome bit each.
    try expectFlag("success", reference.ref_const_success(), Status{ .succeeded = true });
    try expectFlag("failure", reference.ref_const_failure(), Status{ .failed = true });
    try testing.expectEqual(@as(usize, 4), @sizeOf(Status));
}

test "navigation mesh constants equal upstream's" {
    try expectConstant(
        "verts_per_polygon",
        reference.ref_const_verts_per_polygon(),
        nav_mesh.verts_per_polygon,
    );
    try expectConstant(
        "nav_mesh_magic",
        reference.ref_const_navmesh_magic(),
        nav_mesh.nav_mesh_magic,
    );
    try expectConstant(
        "nav_mesh_version",
        reference.ref_const_navmesh_version(),
        nav_mesh.nav_mesh_version,
    );
    try expectConstant(
        "nav_mesh_state_magic",
        reference.ref_const_navmesh_state_magic(),
        nav_mesh.nav_mesh_state_magic,
    );
    try expectConstant(
        "nav_mesh_state_version",
        reference.ref_const_navmesh_state_version(),
        nav_mesh.nav_mesh_state_version,
    );
    try expectConstant("ext_link", reference.ref_const_ext_link(), nav_mesh.ext_link);
    try expectConstant("null_link", reference.ref_const_null_link(), nav_mesh.null_link);
    try expectConstant(
        "off_mesh_connection_bidirectional",
        reference.ref_const_offmesh_con_bidir(),
        nav_mesh.off_mesh_connection_bidirectional,
    );
    try expectConstant("area_count_max", reference.ref_const_max_areas(), nav_mesh.area_count_max);
    try expectConstant(
        "detail_edge_boundary",
        reference.ref_const_detail_edge_boundary(),
        nav_mesh.detail_edge_boundary,
    );
    try support.expectFloat(
        reference.ref_const_ray_cast_limit_proportions(),
        nav_mesh.ray_cast_limit_proportions,
    );
}

test "flag and option bits equal upstream's" {
    try expectFlag(
        "tile_free_data",
        reference.ref_const_tile_free_data(),
        nav_mesh.TileFlags{ .free_data = true },
    );
    try expectFlag(
        "straight_path_start",
        reference.ref_const_straightpath_start(),
        nav_mesh.StraightPathFlags{ .start = true },
    );
    try expectFlag(
        "straight_path_end",
        reference.ref_const_straightpath_end(),
        nav_mesh.StraightPathFlags{ .end = true },
    );
    try expectFlag(
        "straight_path_off_mesh_connection",
        reference.ref_const_straightpath_offmesh_connection(),
        nav_mesh.StraightPathFlags{ .off_mesh_connection = true },
    );
    try expectFlag(
        "straight_path_area_crossings",
        reference.ref_const_straightpath_area_crossings(),
        nav_mesh.StraightPathOptions{ .area_crossings = true },
    );
    try expectFlag(
        "straight_path_all_crossings",
        reference.ref_const_straightpath_all_crossings(),
        nav_mesh.StraightPathOptions{ .all_crossings = true },
    );
    try expectFlag(
        "find_path_any_angle",
        reference.ref_const_findpath_any_angle(),
        nav_mesh.FindPathOptions{ .any_angle = true },
    );
    try expectFlag(
        "raycast_use_costs",
        reference.ref_const_raycast_use_costs(),
        nav_mesh.RaycastOptions{ .use_costs = true },
    );
}

test "PolyType: values equal upstream's dtPolyTypes" {
    try expectConstant(
        "ground",
        reference.ref_const_polytype_ground(),
        @backingInt(nav_mesh.PolyType.ground),
    );
    try expectConstant(
        "off_mesh_connection",
        reference.ref_const_polytype_offmesh_connection(),
        @backingInt(nav_mesh.PolyType.off_mesh_connection),
    );
}

// ---------------------------------------------------------------------------
// Status helpers
// ---------------------------------------------------------------------------

/// Fails unless the port reads the status `bits`, and combines it with the
/// status `mask`, as upstream's tests and the bit operations around them do.
fn expectStatusReads(bits: u32, mask: u32) !void {
    errdefer std.debug.print("  bits 0x{x:0>8}, mask 0x{x:0>8}\n", .{ bits, mask });
    const status: Status = @bitCast(bits);
    const other: Status = @bitCast(mask);

    try testing.expectEqual(reference.ref_status_failed(bits), status.failed);
    try testing.expectEqual(reference.ref_status_succeeded(bits), status.succeeded);
    try testing.expectEqual(reference.ref_status_in_progress(bits), status.in_progress);
    try testing.expectEqual(reference.ref_status_detail(bits, mask), status.has(other));

    // The named detail flags, each as upstream's code asks for it.
    try testing.expectEqual(
        reference.ref_status_detail(bits, reference.ref_const_wrong_magic()),
        status.wrong_magic,
    );
    try testing.expectEqual(
        reference.ref_status_detail(bits, reference.ref_const_wrong_version()),
        status.wrong_version,
    );
    try testing.expectEqual(
        reference.ref_status_detail(bits, reference.ref_const_out_of_memory()),
        status.out_of_memory,
    );
    try testing.expectEqual(
        reference.ref_status_detail(bits, reference.ref_const_invalid_param()),
        status.invalid_param,
    );
    try testing.expectEqual(
        reference.ref_status_detail(bits, reference.ref_const_buffer_too_small()),
        status.buffer_too_small,
    );
    try testing.expectEqual(
        reference.ref_status_detail(bits, reference.ref_const_out_of_nodes()),
        status.out_of_nodes,
    );
    try testing.expectEqual(
        reference.ref_status_detail(bits, reference.ref_const_partial_result()),
        status.partial_result,
    );
    try testing.expectEqual(
        reference.ref_status_detail(bits, reference.ref_const_already_occupied()),
        status.already_occupied,
    );

    // Upstream masks the detail bits with `DT_STATUS_DETAIL_MASK` and combines
    // statuses with `|`.
    const detail_mask = reference.ref_const_status_detail_mask();
    try testing.expectEqual(bits & detail_mask, bitsOf(status.detail()));
    try testing.expectEqual(bits | mask, bitsOf(status.with(other)));
}

test "Status helpers: hand-made patterns" {
    const named = [_]u32{
        reference.ref_const_failure(),
        reference.ref_const_success(),
        reference.ref_const_in_progress(),
        reference.ref_const_status_detail_mask(),
        reference.ref_const_failure() | reference.ref_const_invalid_param(),
        reference.ref_const_success() | reference.ref_const_buffer_too_small(),
        reference.ref_const_success() | reference.ref_const_partial_result() |
            reference.ref_const_out_of_nodes(),
        reference.ref_const_in_progress() | reference.ref_const_out_of_nodes(),
        0,
        0xffffffff,
    };
    for (named) |bits| {
        for (named) |mask| try expectStatusReads(bits, mask);
    }

    // Each of the 32 bits alone, against each of the 32 as the mask.
    for (0..32) |bit| {
        const bits = @as(u32, 1) << @intCast(bit);
        for (0..32) |mask_bit| {
            try expectStatusReads(bits, @as(u32, 1) << @intCast(mask_bit));
        }
    }
}

test "Status helpers: fuzz" {
    const seed = 0x57a7;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..4096) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const bits = random.int(u32);
        // A mask of one bit, a few bits and any bits.
        const mask = switch (case % 3) {
            0 => @as(u32, 1) << random.int(u5),
            1 => random.int(u32) & random.int(u32) & random.int(u32),
            else => random.int(u32),
        };
        try expectStatusReads(bits, mask);
    }
}

// ---------------------------------------------------------------------------
// Polygon bits and detail triangle flags
// ---------------------------------------------------------------------------

/// Fails unless the port's polygon holds the bytes the original's does, and
/// both read the same area and type from them.
fn expectSamePoly(upstream: *const reference.Poly, port: *const nav_mesh.Poly) !void {
    try support.expectBytes(std.mem.asBytes(upstream), std.mem.asBytes(port));
    try testing.expectEqual(reference.ref_poly_get_area(upstream), port.getArea());
    try testing.expectEqual(
        reference.ref_poly_get_type(upstream),
        @as(u8, @backingInt(port.getType())),
    );
}

test "Poly: setArea and setType over every area and type, in both orders" {
    // The bytes of the two sides are compared, so the sizes must agree first.
    try testing.expectEqual(@sizeOf(reference.Poly), @sizeOf(nav_mesh.Poly));

    // Starting bytes: all clear, all set, alternating, and random. Every byte
    // of the record is meaningful, so the setters must leave all but the
    // shared one alone.
    var prng: std.Random.DefaultPrng = .init(0x9017);
    var starts: [4][@sizeOf(nav_mesh.Poly)]u8 = undefined;
    @memset(&starts[0], 0x00);
    @memset(&starts[1], 0xff);
    @memset(&starts[2], 0xaa);
    prng.random().bytes(&starts[3]);

    for (starts, 0..) |start, start_index| {
        for (0..256) |area_index| {
            const area: u8 = @intCast(area_index);
            // Four type values: the two named ones and the two the enum has
            // no name for, which tile data can still carry.
            for (0..4) |type_index| {
                const type_bits: u2 = @intCast(type_index);
                const poly_type: nav_mesh.PolyType = @fromBackingInt(type_bits);
                for ([_]bool{ false, true }) |type_first| {
                    errdefer std.debug.print(
                        "  start pattern {d}, area {d}, type {d}, type first {}\n",
                        .{ start_index, area, type_bits, type_first },
                    );
                    var upstream: reference.Poly = undefined;
                    var port: nav_mesh.Poly = undefined;
                    @memcpy(std.mem.asBytes(&upstream), &start);
                    @memcpy(std.mem.asBytes(&port), &start);
                    try expectSamePoly(&upstream, &port);

                    if (type_first) {
                        reference.ref_poly_set_type(&upstream, type_bits);
                        port.setType(poly_type);
                        reference.ref_poly_set_area(&upstream, area);
                        port.setArea(area);
                    } else {
                        reference.ref_poly_set_area(&upstream, area);
                        port.setArea(area);
                        reference.ref_poly_set_type(&upstream, type_bits);
                        port.setType(poly_type);
                    }
                    try expectSamePoly(&upstream, &port);
                }
            }
        }
    }
}

test "Poly: setArea and setType in random order, fuzz" {
    try testing.expectEqual(@sizeOf(reference.Poly), @sizeOf(nav_mesh.Poly));

    const seed = 0x9018;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var upstream: reference.Poly = undefined;
    var port: nav_mesh.Poly = undefined;
    random.bytes(std.mem.asBytes(&upstream));
    @memcpy(std.mem.asBytes(&port), std.mem.asBytes(&upstream));
    try expectSamePoly(&upstream, &port);

    for (0..4096) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        if (random.boolean()) {
            // Every byte as an area: the port masks to six bits as upstream does.
            const area = random.int(u8);
            reference.ref_poly_set_area(&upstream, area);
            port.setArea(area);
        } else {
            const type_bits = random.int(u2);
            reference.ref_poly_set_type(&upstream, type_bits);
            port.setType(@fromBackingInt(type_bits));
        }
        try expectSamePoly(&upstream, &port);
    }
}

test "getDetailTriEdgeFlags: every flag byte and edge match upstream's" {
    for (0..256) |flags_index| {
        const flags: u8 = @intCast(flags_index);
        // Edges 0 to 2 are a triangle's; the fourth pair of bits is unused
        // but reads the same.
        for (0..4) |edge_index| {
            errdefer std.debug.print("  flags 0x{x:0>2}, edge {d}\n", .{ flags, edge_index });
            const edge: u2 = @intCast(edge_index);
            const expected = reference.ref_get_detail_tri_edge_flags(flags, edge);
            const actual: i32 = nav_mesh.getDetailTriEdgeFlags(flags, edge);
            try testing.expectEqual(expected, actual);
        }
    }
}
