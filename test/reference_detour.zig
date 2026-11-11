//! Declarations for test/shim/detour_shim.cpp: a C ABI over upstream Detour
//! (recastnavigation v1.6.0), used to test the Zig port against upstream.
//!
//! Every `ref_*` function forwards to one upstream function or method with the
//! same arguments in the same order (object pointer first). Mapping rules:
//!
//! - `dtStatus` and other `unsigned int` results are `u32`; `int` is `i32`.
//! - References: `dtPolyRef` is `PolyRef`, `dtTileRef` is `TileRef`.
//! - A pointer is optional (`?*T`, `?[*]T`) where upstream accepts null or
//!   returns null, and plain otherwise.
//! - Handles are opaque; the shim owns their layout. Plain-old-data structs
//!   that upstream defines are mirrored as `extern struct` below. The
//!   `ref_sizeof_*` and `ref_offsetof_*` functions let a test compare the
//!   mirrors with upstream's real layout.
//! - Output parameters that upstream takes by reference are pointers here.

pub const PolyRef = @import("detour").PolyRef;
pub const TileRef = @import("detour").TileRef;

/// `DT_VERTS_PER_POLYGON`.
const verts_per_polygon = 6;

// ---------------------------------------------------------------------------
// Handles
// ---------------------------------------------------------------------------

/// `dtNavMesh`.
pub const NavMesh = opaque {};
/// `dtNavMeshQuery`.
pub const NavMeshQuery = opaque {};
/// `dtQueryFilter`.
pub const QueryFilter = opaque {};
/// `dtMeshTile`. Its arrays are read through the `ref_tile_*` accessors.
pub const MeshTile = opaque {};
/// `dtNodePool`.
pub const NodePool = opaque {};
/// `dtNodeQueue`.
pub const NodeQueue = opaque {};
/// `dtNode`. It has bit-fields, so it is read and written through
/// `ref_node_fields`, `ref_node_set` and the per-field accessors.
pub const Node = opaque {};

// ---------------------------------------------------------------------------
// Plain-old-data structs, in upstream's field order
// ---------------------------------------------------------------------------

/// `dtPoly`. Area and type share `area_and_type`; use `ref_poly_*` to read it.
pub const Poly = extern struct {
    first_link: u32,
    verts: [verts_per_polygon]u16,
    neis: [verts_per_polygon]u16,
    flags: u16,
    vert_count: u8,
    area_and_type: u8,
};

/// `dtPolyDetail`.
pub const PolyDetail = extern struct {
    vert_base: u32,
    tri_base: u32,
    vert_count: u8,
    tri_count: u8,
};

/// `dtLink`.
pub const Link = extern struct {
    ref: PolyRef,
    next: u32,
    edge: u8,
    side: u8,
    bmin: u8,
    bmax: u8,
};

/// `dtBVNode`.
pub const BVNode = extern struct {
    bmin: [3]u16,
    bmax: [3]u16,
    i: i32,
};

/// `dtOffMeshConnection`.
pub const OffMeshConnection = extern struct {
    pos: [6]f32,
    rad: f32,
    poly: u16,
    flags: u8,
    side: u8,
    user_id: u32,
};

/// `dtMeshHeader`.
pub const MeshHeader = extern struct {
    magic: i32,
    version: i32,
    x: i32,
    y: i32,
    layer: i32,
    user_id: u32,
    poly_count: i32,
    vert_count: i32,
    max_link_count: i32,
    detail_mesh_count: i32,
    detail_vert_count: i32,
    detail_tri_count: i32,
    bv_node_count: i32,
    off_mesh_con_count: i32,
    off_mesh_base: i32,
    walkable_height: f32,
    walkable_radius: f32,
    walkable_climb: f32,
    bmin: [3]f32,
    bmax: [3]f32,
    bv_quant_factor: f32,
};

/// `dtNavMeshParams`.
pub const NavMeshParams = extern struct {
    orig: [3]f32,
    tile_width: f32,
    tile_height: f32,
    max_tiles: i32,
    max_polys: i32,
};

/// `dtNavMeshCreateParams`.
pub const NavMeshCreateParams = extern struct {
    verts: ?[*]const u16,
    vert_count: i32,
    polys: ?[*]const u16,
    poly_flags: ?[*]const u16,
    poly_areas: ?[*]const u8,
    poly_count: i32,
    nvp: i32,
    detail_meshes: ?[*]const u32,
    detail_verts: ?[*]const f32,
    detail_verts_count: i32,
    detail_tris: ?[*]const u8,
    detail_tri_count: i32,
    off_mesh_con_verts: ?[*]const f32,
    off_mesh_con_rad: ?[*]const f32,
    off_mesh_con_flags: ?[*]const u16,
    off_mesh_con_areas: ?[*]const u8,
    off_mesh_con_dir: ?[*]const u8,
    off_mesh_con_user_id: ?[*]const u32,
    off_mesh_con_count: i32,
    user_id: u32,
    tile_x: i32,
    tile_y: i32,
    tile_layer: i32,
    bmin: [3]f32,
    bmax: [3]f32,
    walkable_height: f32,
    walkable_radius: f32,
    walkable_climb: f32,
    cs: f32,
    ch: f32,
    build_bv_tree: bool,
};

/// `dtRaycastHit`. The shim's `ref_query_raycast_hit` takes these fields as
/// separate arguments; this mirror is for layout checks.
pub const RaycastHit = extern struct {
    t: f32,
    hit_normal: [3]f32,
    hit_edge_index: i32,
    path: ?[*]PolyRef,
    path_count: i32,
    max_path: i32,
    path_cost: f32,
};

// ---------------------------------------------------------------------------
// Callback types
// ---------------------------------------------------------------------------

/// `dtAllocHint`.
pub const AllocHint = enum(c_int) { perm = 0, temp = 1 };

/// `dtAllocFunc`.
pub const AllocFn = *const fn (size: usize, hint: AllocHint) callconv(.c) ?*anyopaque;
/// `dtFreeFunc`.
pub const FreeFn = *const fn (ptr: ?*anyopaque) callconv(.c) void;
/// `dtAssertFailFunc`.
pub const AssertFailFn = *const fn (
    expression: [*:0]const u8,
    file: [*:0]const u8,
    line: i32,
) callconv(.c) void;

/// `float (*frand)()` of the random point queries: returns a value in [0, 1).
pub const RandomFn = *const fn () callconv(.c) f32;

/// Custom `passFilter`. Only used with `ref_filter_create_custom`.
pub const FilterPassFn = *const fn (
    user: ?*anyopaque,
    ref: PolyRef,
    tile: *const MeshTile,
    poly: *const Poly,
) callconv(.c) bool;

/// Custom `getCost`. Only used with `ref_filter_create_custom`.
pub const FilterCostFn = *const fn (
    user: ?*anyopaque,
    pa: *const [3]f32,
    pb: *const [3]f32,
    prev_ref: PolyRef,
    prev_tile: ?*const MeshTile,
    prev_poly: ?*const Poly,
    cur_ref: PolyRef,
    cur_tile: *const MeshTile,
    cur_poly: *const Poly,
    next_ref: PolyRef,
    next_tile: ?*const MeshTile,
    next_poly: ?*const Poly,
) callconv(.c) f32;

/// `dtPolyQuery::process`, called for each batch of polygons.
pub const PolyQueryFn = *const fn (
    user: ?*anyopaque,
    tile: *const MeshTile,
    polys: [*]*Poly,
    refs: [*]PolyRef,
    count: i32,
) callconv(.c) void;

/// The comparison function of `ref_stable_qsort`, as in libc `qsort`.
pub const CompareFn = *const fn (a: ?*const anyopaque, b: ?*const anyopaque) callconv(.c) i32;

// ---------------------------------------------------------------------------
// DetourMath.h
// ---------------------------------------------------------------------------

pub extern fn ref_math_fabsf(x: f32) f32;
pub extern fn ref_math_sqrtf(x: f32) f32;
pub extern fn ref_math_floorf(x: f32) f32;
pub extern fn ref_math_ceilf(x: f32) f32;
pub extern fn ref_math_cosf(x: f32) f32;
pub extern fn ref_math_sinf(x: f32) f32;
pub extern fn ref_math_atan2f(y: f32, x: f32) f32;
pub extern fn ref_math_isfinite(x: f32) bool;

// ---------------------------------------------------------------------------
// DetourCommon.h: general helpers (the template instantiations for f32 and i32)
// ---------------------------------------------------------------------------

pub extern fn ref_min_f32(a: f32, b: f32) f32;
pub extern fn ref_max_f32(a: f32, b: f32) f32;
pub extern fn ref_abs_f32(a: f32) f32;
pub extern fn ref_sqr_f32(a: f32) f32;
pub extern fn ref_clamp_f32(v: f32, mn: f32, mx: f32) f32;

pub extern fn ref_min_i32(a: i32, b: i32) i32;
pub extern fn ref_max_i32(a: i32, b: i32) i32;
pub extern fn ref_abs_i32(a: i32) i32;
pub extern fn ref_sqr_i32(a: i32) i32;
pub extern fn ref_clamp_i32(v: i32, mn: i32, mx: i32) i32;

pub extern fn ref_swap_f32(a: *f32, b: *f32) void;
pub extern fn ref_swap_i32(a: *i32, b: *i32) void;

// ---------------------------------------------------------------------------
// DetourCommon.h: vector helpers
// ---------------------------------------------------------------------------

pub extern fn ref_vcross(dest: *[3]f32, v1: *const [3]f32, v2: *const [3]f32) void;
pub extern fn ref_vdot(v1: *const [3]f32, v2: *const [3]f32) f32;
pub extern fn ref_vmad(dest: *[3]f32, v1: *const [3]f32, v2: *const [3]f32, s: f32) void;
pub extern fn ref_vlerp(dest: *[3]f32, v1: *const [3]f32, v2: *const [3]f32, t: f32) void;
pub extern fn ref_vadd(dest: *[3]f32, v1: *const [3]f32, v2: *const [3]f32) void;
pub extern fn ref_vsub(dest: *[3]f32, v1: *const [3]f32, v2: *const [3]f32) void;
pub extern fn ref_vscale(dest: *[3]f32, v: *const [3]f32, t: f32) void;
pub extern fn ref_vmin(mn: *[3]f32, v: *const [3]f32) void;
pub extern fn ref_vmax(mx: *[3]f32, v: *const [3]f32) void;
pub extern fn ref_vset(dest: *[3]f32, x: f32, y: f32, z: f32) void;
pub extern fn ref_vcopy(dest: *[3]f32, a: *const [3]f32) void;
pub extern fn ref_vlen(v: *const [3]f32) f32;
pub extern fn ref_vlen_sqr(v: *const [3]f32) f32;
pub extern fn ref_vdist(v1: *const [3]f32, v2: *const [3]f32) f32;
pub extern fn ref_vdist_sqr(v1: *const [3]f32, v2: *const [3]f32) f32;
pub extern fn ref_vdist_2d(v1: *const [3]f32, v2: *const [3]f32) f32;
pub extern fn ref_vdist_2d_sqr(v1: *const [3]f32, v2: *const [3]f32) f32;
pub extern fn ref_vnormalize(v: *[3]f32) void;
pub extern fn ref_vequal(p0: *const [3]f32, p1: *const [3]f32) bool;
pub extern fn ref_visfinite(v: *const [3]f32) bool;
pub extern fn ref_visfinite_2d(v: *const [3]f32) bool;
pub extern fn ref_vdot_2d(u: *const [3]f32, v: *const [3]f32) f32;
pub extern fn ref_vperp_2d(u: *const [3]f32, v: *const [3]f32) f32;

// ---------------------------------------------------------------------------
// DetourCommon.h: computational geometry
// ---------------------------------------------------------------------------

pub extern fn ref_tri_area_2d(a: *const [3]f32, b: *const [3]f32, c: *const [3]f32) f32;
pub extern fn ref_overlap_quant_bounds(
    amin: *const [3]u16,
    amax: *const [3]u16,
    bmin: *const [3]u16,
    bmax: *const [3]u16,
) bool;
pub extern fn ref_overlap_bounds(
    amin: *const [3]f32,
    amax: *const [3]f32,
    bmin: *const [3]f32,
    bmax: *const [3]f32,
) bool;
pub extern fn ref_closest_pt_point_triangle(
    closest: *[3]f32,
    p: *const [3]f32,
    a: *const [3]f32,
    b: *const [3]f32,
    c: *const [3]f32,
) void;
pub extern fn ref_closest_height_point_triangle(
    p: *const [3]f32,
    a: *const [3]f32,
    b: *const [3]f32,
    c: *const [3]f32,
    h: *f32,
) bool;
pub extern fn ref_intersect_segment_poly_2d(
    p0: *const [3]f32,
    p1: *const [3]f32,
    verts: [*]const f32,
    nverts: i32,
    tmin: *f32,
    tmax: *f32,
    seg_min: *i32,
    seg_max: *i32,
) bool;
pub extern fn ref_intersect_seg_seg_2d(
    ap: *const [3]f32,
    aq: *const [3]f32,
    bp: *const [3]f32,
    bq: *const [3]f32,
    s: *f32,
    t: *f32,
) bool;
pub extern fn ref_point_in_polygon(pt: *const [3]f32, verts: [*]const f32, nverts: i32) bool;
/// `ed` and `et` each hold `nverts` floats.
pub extern fn ref_distance_pt_poly_edges_sqr(
    pt: *const [3]f32,
    verts: [*]const f32,
    nverts: i32,
    ed: [*]f32,
    et: [*]f32,
) bool;
pub extern fn ref_distance_pt_seg_sqr_2d(
    pt: *const [3]f32,
    p: *const [3]f32,
    q: *const [3]f32,
    t: *f32,
) f32;
pub extern fn ref_calc_poly_center(
    tc: *[3]f32,
    idx: [*]const u16,
    nidx: i32,
    verts: [*]const f32,
) void;
pub extern fn ref_overlap_poly_poly_2d(
    polya: [*]const f32,
    npolya: i32,
    polyb: [*]const f32,
    npolyb: i32,
) bool;

// ---------------------------------------------------------------------------
// DetourCommon.h: miscellaneous
// ---------------------------------------------------------------------------

pub extern fn ref_next_pow2(v: u32) u32;
pub extern fn ref_ilog2(v: u32) u32;
pub extern fn ref_align4(x: i32) i32;
pub extern fn ref_opposite_tile(side: i32) i32;
pub extern fn ref_swap_byte(a: *u8, b: *u8) void;
pub extern fn ref_swap_endian_u16(v: *u16) void;
pub extern fn ref_swap_endian_i16(v: *i16) void;
pub extern fn ref_swap_endian_u32(v: *u32) void;
pub extern fn ref_swap_endian_i32(v: *i32) void;
pub extern fn ref_swap_endian_f32(v: *f32) void;
pub extern fn ref_random_point_in_convex_poly(
    pts: [*]const f32,
    npts: i32,
    areas: [*]f32,
    s: f32,
    t: f32,
    out: *[3]f32,
) void;
/// Returns the old `buffer.*` and advances `buffer.*` by `distance` bytes.
pub extern fn ref_get_then_advance_buffer_pointer(buffer: *[*]u8, distance: usize) [*]u8;
pub extern fn ref_get_then_advance_buffer_pointer_const(
    buffer: *[*]const u8,
    distance: usize,
) [*]const u8;

// ---------------------------------------------------------------------------
// DetourStatus.h
// ---------------------------------------------------------------------------

pub extern fn ref_status_failed(status: u32) bool;
pub extern fn ref_status_succeeded(status: u32) bool;
pub extern fn ref_status_in_progress(status: u32) bool;
pub extern fn ref_status_detail(status: u32, detail: u32) bool;

// ---------------------------------------------------------------------------
// Constants (values of upstream's constants in this build)
// ---------------------------------------------------------------------------

pub extern fn ref_const_failure() u32;
pub extern fn ref_const_success() u32;
pub extern fn ref_const_in_progress() u32;
pub extern fn ref_const_status_detail_mask() u32;
pub extern fn ref_const_wrong_magic() u32;
pub extern fn ref_const_wrong_version() u32;
pub extern fn ref_const_out_of_memory() u32;
pub extern fn ref_const_invalid_param() u32;
pub extern fn ref_const_buffer_too_small() u32;
pub extern fn ref_const_out_of_nodes() u32;
pub extern fn ref_const_partial_result() u32;
pub extern fn ref_const_already_occupied() u32;

pub extern fn ref_const_verts_per_polygon() i32;
pub extern fn ref_const_navmesh_magic() i32;
pub extern fn ref_const_navmesh_version() i32;
pub extern fn ref_const_navmesh_state_magic() i32;
pub extern fn ref_const_navmesh_state_version() i32;
pub extern fn ref_const_ext_link() u32;
pub extern fn ref_const_null_link() u32;
pub extern fn ref_const_offmesh_con_bidir() u32;
pub extern fn ref_const_max_areas() i32;
pub extern fn ref_const_tile_free_data() u32;
pub extern fn ref_const_straightpath_start() u32;
pub extern fn ref_const_straightpath_end() u32;
pub extern fn ref_const_straightpath_offmesh_connection() u32;
pub extern fn ref_const_straightpath_area_crossings() u32;
pub extern fn ref_const_straightpath_all_crossings() u32;
pub extern fn ref_const_findpath_any_angle() u32;
pub extern fn ref_const_raycast_use_costs() u32;
pub extern fn ref_const_detail_edge_boundary() u32;
pub extern fn ref_const_polytype_ground() u32;
pub extern fn ref_const_polytype_offmesh_connection() u32;

pub extern fn ref_const_node_open() u32;
pub extern fn ref_const_node_closed() u32;
pub extern fn ref_const_node_parent_detached() u32;
pub extern fn ref_const_null_idx() u32;
pub extern fn ref_const_node_parent_bits() i32;
pub extern fn ref_const_node_state_bits() i32;
pub extern fn ref_const_max_states_per_node() i32;

pub extern fn ref_const_ray_cast_limit_proportions() f32;

// ---------------------------------------------------------------------------
// DetourAlloc.h, DetourAssert.h
// ---------------------------------------------------------------------------

/// Null arguments restore the default malloc / free.
pub extern fn ref_alloc_set_custom(alloc_fn: ?AllocFn, free_fn: ?FreeFn) void;
pub extern fn ref_alloc(size: usize, hint: AllocHint) ?*anyopaque;
/// `dtFree`. Takes any pointer that `dtAlloc` or an upstream allocator gave.
pub extern fn ref_free(ptr: ?*anyopaque) void;
/// A no-op when upstream is built with assertions compiled out.
pub extern fn ref_assert_fail_set_custom(func: ?AssertFailFn) void;
pub extern fn ref_assert_fail_get_custom() ?AssertFailFn;

/// A stable sort with libc `qsort`'s signature. Upstream's builder sort calls
/// are renamed to it at build time.
pub extern fn ref_stable_qsort(
    base: ?*anyopaque,
    count: usize,
    size: usize,
    compare: CompareFn,
) void;

// ---------------------------------------------------------------------------
// DetourNavMeshBuilder.h
// ---------------------------------------------------------------------------

/// `params` is not modified. On success `out_data` points to memory from
/// `dtAlloc`; release it with `ref_free`, or hand it to a navigation mesh
/// with the `tile_free_data` flag.
pub extern fn ref_create_nav_mesh_data(
    params: *const NavMeshCreateParams,
    out_data: *?[*]u8,
    out_data_size: *i32,
) bool;
pub extern fn ref_nav_mesh_header_swap_endian(data: [*]u8, data_size: i32) bool;
pub extern fn ref_nav_mesh_data_swap_endian(data: [*]u8, data_size: i32) bool;

// ---------------------------------------------------------------------------
// Layout checks: sizes and field offsets, in bytes
// ---------------------------------------------------------------------------

pub extern fn ref_sizeof_poly_ref() usize;
pub extern fn ref_sizeof_tile_ref() usize;

pub extern fn ref_sizeof_poly() usize;
pub extern fn ref_offsetof_poly_first_link() usize;
pub extern fn ref_offsetof_poly_verts() usize;
pub extern fn ref_offsetof_poly_neis() usize;
pub extern fn ref_offsetof_poly_flags() usize;
pub extern fn ref_offsetof_poly_vert_count() usize;
pub extern fn ref_offsetof_poly_area_and_type() usize;

pub extern fn ref_sizeof_poly_detail() usize;
pub extern fn ref_offsetof_poly_detail_vert_base() usize;
pub extern fn ref_offsetof_poly_detail_tri_base() usize;
pub extern fn ref_offsetof_poly_detail_vert_count() usize;
pub extern fn ref_offsetof_poly_detail_tri_count() usize;

pub extern fn ref_sizeof_link() usize;
pub extern fn ref_offsetof_link_ref() usize;
pub extern fn ref_offsetof_link_next() usize;
pub extern fn ref_offsetof_link_edge() usize;
pub extern fn ref_offsetof_link_side() usize;
pub extern fn ref_offsetof_link_bmin() usize;
pub extern fn ref_offsetof_link_bmax() usize;

pub extern fn ref_sizeof_bv_node() usize;
pub extern fn ref_offsetof_bv_node_bmin() usize;
pub extern fn ref_offsetof_bv_node_bmax() usize;
pub extern fn ref_offsetof_bv_node_i() usize;

pub extern fn ref_sizeof_off_mesh_connection() usize;
pub extern fn ref_offsetof_off_mesh_connection_pos() usize;
pub extern fn ref_offsetof_off_mesh_connection_rad() usize;
pub extern fn ref_offsetof_off_mesh_connection_poly() usize;
pub extern fn ref_offsetof_off_mesh_connection_flags() usize;
pub extern fn ref_offsetof_off_mesh_connection_side() usize;
pub extern fn ref_offsetof_off_mesh_connection_user_id() usize;

pub extern fn ref_sizeof_mesh_header() usize;
pub extern fn ref_offsetof_mesh_header_magic() usize;
pub extern fn ref_offsetof_mesh_header_version() usize;
pub extern fn ref_offsetof_mesh_header_x() usize;
pub extern fn ref_offsetof_mesh_header_y() usize;
pub extern fn ref_offsetof_mesh_header_layer() usize;
pub extern fn ref_offsetof_mesh_header_user_id() usize;
pub extern fn ref_offsetof_mesh_header_poly_count() usize;
pub extern fn ref_offsetof_mesh_header_vert_count() usize;
pub extern fn ref_offsetof_mesh_header_max_link_count() usize;
pub extern fn ref_offsetof_mesh_header_detail_mesh_count() usize;
pub extern fn ref_offsetof_mesh_header_detail_vert_count() usize;
pub extern fn ref_offsetof_mesh_header_detail_tri_count() usize;
pub extern fn ref_offsetof_mesh_header_bv_node_count() usize;
pub extern fn ref_offsetof_mesh_header_off_mesh_con_count() usize;
pub extern fn ref_offsetof_mesh_header_off_mesh_base() usize;
pub extern fn ref_offsetof_mesh_header_walkable_height() usize;
pub extern fn ref_offsetof_mesh_header_walkable_radius() usize;
pub extern fn ref_offsetof_mesh_header_walkable_climb() usize;
pub extern fn ref_offsetof_mesh_header_bmin() usize;
pub extern fn ref_offsetof_mesh_header_bmax() usize;
pub extern fn ref_offsetof_mesh_header_bv_quant_factor() usize;

pub extern fn ref_sizeof_mesh_tile() usize;
pub extern fn ref_offsetof_mesh_tile_salt() usize;
pub extern fn ref_offsetof_mesh_tile_links_free_list() usize;
pub extern fn ref_offsetof_mesh_tile_header() usize;
pub extern fn ref_offsetof_mesh_tile_polys() usize;
pub extern fn ref_offsetof_mesh_tile_verts() usize;
pub extern fn ref_offsetof_mesh_tile_links() usize;
pub extern fn ref_offsetof_mesh_tile_detail_meshes() usize;
pub extern fn ref_offsetof_mesh_tile_detail_verts() usize;
pub extern fn ref_offsetof_mesh_tile_detail_tris() usize;
pub extern fn ref_offsetof_mesh_tile_bv_tree() usize;
pub extern fn ref_offsetof_mesh_tile_off_mesh_cons() usize;
pub extern fn ref_offsetof_mesh_tile_data() usize;
pub extern fn ref_offsetof_mesh_tile_data_size() usize;
pub extern fn ref_offsetof_mesh_tile_flags() usize;
pub extern fn ref_offsetof_mesh_tile_next() usize;

pub extern fn ref_sizeof_nav_mesh_params() usize;
pub extern fn ref_offsetof_nav_mesh_params_orig() usize;
pub extern fn ref_offsetof_nav_mesh_params_tile_width() usize;
pub extern fn ref_offsetof_nav_mesh_params_tile_height() usize;
pub extern fn ref_offsetof_nav_mesh_params_max_tiles() usize;
pub extern fn ref_offsetof_nav_mesh_params_max_polys() usize;

pub extern fn ref_sizeof_nav_mesh_create_params() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_verts() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_vert_count() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_polys() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_poly_flags() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_poly_areas() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_poly_count() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_nvp() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_detail_meshes() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_detail_verts() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_detail_verts_count() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_detail_tris() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_detail_tri_count() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_off_mesh_con_verts() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_off_mesh_con_rad() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_off_mesh_con_flags() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_off_mesh_con_areas() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_off_mesh_con_dir() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_off_mesh_con_user_id() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_off_mesh_con_count() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_user_id() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_tile_x() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_tile_y() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_tile_layer() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_bmin() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_bmax() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_walkable_height() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_walkable_radius() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_walkable_climb() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_cs() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_ch() usize;
pub extern fn ref_offsetof_nav_mesh_create_params_build_bv_tree() usize;

pub extern fn ref_sizeof_raycast_hit() usize;
pub extern fn ref_offsetof_raycast_hit_t() usize;
pub extern fn ref_offsetof_raycast_hit_hit_normal() usize;
pub extern fn ref_offsetof_raycast_hit_hit_edge_index() usize;
pub extern fn ref_offsetof_raycast_hit_path() usize;
pub extern fn ref_offsetof_raycast_hit_path_count() usize;
pub extern fn ref_offsetof_raycast_hit_max_path() usize;
pub extern fn ref_offsetof_raycast_hit_path_cost() usize;

/// `dtNode` has bit-fields: its size, and the offsets of its plain members.
pub extern fn ref_sizeof_node() usize;
pub extern fn ref_sizeof_node_index() usize;
pub extern fn ref_offsetof_node_pos() usize;
pub extern fn ref_offsetof_node_cost() usize;
pub extern fn ref_offsetof_node_total() usize;
pub extern fn ref_offsetof_node_id() usize;

/// What `getMemUsed()` of the pool and the queue add to their arrays.
pub extern fn ref_sizeof_node_pool() usize;
pub extern fn ref_sizeof_node_queue() usize;
pub extern fn ref_sizeof_query_filter() usize;

// ---------------------------------------------------------------------------
// dtPoly, dtMeshTile, dtGetDetailTriEdgeFlags
// ---------------------------------------------------------------------------

pub extern fn ref_poly_set_area(poly: *Poly, area: u8) void;
pub extern fn ref_poly_set_type(poly: *Poly, poly_type: u8) void;
pub extern fn ref_poly_get_area(poly: *const Poly) u8;
pub extern fn ref_poly_get_type(poly: *const Poly) u8;
pub extern fn ref_get_detail_tri_edge_flags(tri_flags: u8, edge_index: i32) i32;

/// The array accessors return null for a tile that is not populated.
pub extern fn ref_tile_header(tile: *const MeshTile) ?*const MeshHeader;
pub extern fn ref_tile_polys(tile: *const MeshTile) ?[*]const Poly;
pub extern fn ref_tile_verts(tile: *const MeshTile) ?[*]const f32;
pub extern fn ref_tile_links(tile: *const MeshTile) ?[*]const Link;
pub extern fn ref_tile_detail_meshes(tile: *const MeshTile) ?[*]const PolyDetail;
pub extern fn ref_tile_detail_verts(tile: *const MeshTile) ?[*]const f32;
pub extern fn ref_tile_detail_tris(tile: *const MeshTile) ?[*]const u8;
pub extern fn ref_tile_bv_tree(tile: *const MeshTile) ?[*]const BVNode;
pub extern fn ref_tile_off_mesh_cons(tile: *const MeshTile) ?[*]const OffMeshConnection;
pub extern fn ref_tile_data(tile: *const MeshTile) ?[*]const u8;
pub extern fn ref_tile_data_size(tile: *const MeshTile) i32;
pub extern fn ref_tile_flags(tile: *const MeshTile) i32;
pub extern fn ref_tile_salt(tile: *const MeshTile) u32;
pub extern fn ref_tile_links_free_list(tile: *const MeshTile) u32;
pub extern fn ref_tile_next(tile: *const MeshTile) ?*const MeshTile;

// ---------------------------------------------------------------------------
// dtNavMesh
// ---------------------------------------------------------------------------

pub extern fn ref_nav_mesh_alloc() ?*NavMesh;
pub extern fn ref_nav_mesh_free(nav: ?*NavMesh) void;
/// `init(const dtNavMeshParams*)`.
pub extern fn ref_nav_mesh_init(nav: *NavMesh, params: *const NavMeshParams) u32;
/// `init(unsigned char* data, int dataSize, int flags)`.
pub extern fn ref_nav_mesh_init_single(nav: *NavMesh, data: [*]u8, data_size: i32, flags: i32) u32;
pub extern fn ref_nav_mesh_get_params(nav: *const NavMesh) *const NavMeshParams;
pub extern fn ref_nav_mesh_add_tile(
    nav: *NavMesh,
    data: [*]u8,
    data_size: i32,
    flags: i32,
    last_ref: TileRef,
    result: ?*TileRef,
) u32;
pub extern fn ref_nav_mesh_remove_tile(
    nav: *NavMesh,
    ref: TileRef,
    data: ?*?[*]u8,
    data_size: ?*i32,
) u32;
pub extern fn ref_nav_mesh_calc_tile_loc(
    nav: *const NavMesh,
    pos: *const [3]f32,
    tx: *i32,
    ty: *i32,
) void;
pub extern fn ref_nav_mesh_get_tile_at(
    nav: *const NavMesh,
    x: i32,
    y: i32,
    layer: i32,
) ?*const MeshTile;
pub extern fn ref_nav_mesh_get_tiles_at(
    nav: *const NavMesh,
    x: i32,
    y: i32,
    tiles: [*]?*const MeshTile,
    max_tiles: i32,
) i32;
pub extern fn ref_nav_mesh_get_tile_ref_at(nav: *const NavMesh, x: i32, y: i32, layer: i32) TileRef;
pub extern fn ref_nav_mesh_get_tile_ref(nav: *const NavMesh, tile: ?*const MeshTile) TileRef;
pub extern fn ref_nav_mesh_get_tile_by_ref(nav: *const NavMesh, ref: TileRef) ?*const MeshTile;
pub extern fn ref_nav_mesh_get_max_tiles(nav: *const NavMesh) i32;
pub extern fn ref_nav_mesh_get_tile(nav: *const NavMesh, i: i32) ?*const MeshTile;
pub extern fn ref_nav_mesh_get_tile_and_poly_by_ref(
    nav: *const NavMesh,
    ref: PolyRef,
    tile: *?*const MeshTile,
    poly: *?*const Poly,
) u32;
pub extern fn ref_nav_mesh_get_tile_and_poly_by_ref_unsafe(
    nav: *const NavMesh,
    ref: PolyRef,
    tile: *?*const MeshTile,
    poly: *?*const Poly,
) void;
pub extern fn ref_nav_mesh_is_valid_poly_ref(nav: *const NavMesh, ref: PolyRef) bool;
pub extern fn ref_nav_mesh_get_poly_ref_base(nav: *const NavMesh, tile: ?*const MeshTile) PolyRef;
pub extern fn ref_nav_mesh_get_off_mesh_connection_poly_end_points(
    nav: *const NavMesh,
    prev_ref: PolyRef,
    poly_ref: PolyRef,
    start_pos: *[3]f32,
    end_pos: *[3]f32,
) u32;
pub extern fn ref_nav_mesh_get_off_mesh_connection_by_ref(
    nav: *const NavMesh,
    ref: PolyRef,
) ?*const OffMeshConnection;
pub extern fn ref_nav_mesh_set_poly_flags(nav: *NavMesh, ref: PolyRef, flags: u16) u32;
pub extern fn ref_nav_mesh_get_poly_flags(
    nav: *const NavMesh,
    ref: PolyRef,
    result_flags: *u16,
) u32;
pub extern fn ref_nav_mesh_set_poly_area(nav: *NavMesh, ref: PolyRef, area: u8) u32;
pub extern fn ref_nav_mesh_get_poly_area(nav: *const NavMesh, ref: PolyRef, result_area: *u8) u32;
pub extern fn ref_nav_mesh_get_tile_state_size(nav: *const NavMesh, tile: ?*const MeshTile) i32;
pub extern fn ref_nav_mesh_store_tile_state(
    nav: *const NavMesh,
    tile: *const MeshTile,
    data: [*]u8,
    max_data_size: i32,
) u32;
/// Takes a mutable tile, as upstream does: a tile from `ref_nav_mesh_get_tile*`
/// needs `@constCast`.
pub extern fn ref_nav_mesh_restore_tile_state(
    nav: *NavMesh,
    tile: *MeshTile,
    data: [*]const u8,
    max_data_size: i32,
) u32;
pub extern fn ref_nav_mesh_encode_poly_id(nav: *const NavMesh, salt: u32, it: u32, ip: u32) PolyRef;
pub extern fn ref_nav_mesh_decode_poly_id(
    nav: *const NavMesh,
    ref: PolyRef,
    salt: *u32,
    it: *u32,
    ip: *u32,
) void;
pub extern fn ref_nav_mesh_decode_poly_id_salt(nav: *const NavMesh, ref: PolyRef) u32;
pub extern fn ref_nav_mesh_decode_poly_id_tile(nav: *const NavMesh, ref: PolyRef) u32;
pub extern fn ref_nav_mesh_decode_poly_id_poly(nav: *const NavMesh, ref: PolyRef) u32;

// ---------------------------------------------------------------------------
// dtQueryFilter
// ---------------------------------------------------------------------------

pub extern fn ref_filter_create() ?*QueryFilter;
/// A filter whose `passFilter` and `getCost` call back into the test. A null
/// callback keeps the default for that function. Returns null, always, when
/// the shim is built without `DT_VIRTUAL_QUERYFILTER`.
pub extern fn ref_filter_create_custom(
    pass: ?FilterPassFn,
    cost: ?FilterCostFn,
    user: ?*anyopaque,
) ?*QueryFilter;
/// Destroys a filter from either create function.
pub extern fn ref_filter_destroy(filter: ?*QueryFilter) void;
pub extern fn ref_filter_get_area_cost(filter: *const QueryFilter, i: i32) f32;
pub extern fn ref_filter_set_area_cost(filter: *QueryFilter, i: i32, cost: f32) void;
pub extern fn ref_filter_get_include_flags(filter: *const QueryFilter) u16;
pub extern fn ref_filter_set_include_flags(filter: *QueryFilter, flags: u16) void;
pub extern fn ref_filter_get_exclude_flags(filter: *const QueryFilter) u16;
pub extern fn ref_filter_set_exclude_flags(filter: *QueryFilter, flags: u16) void;
pub extern fn ref_filter_pass_filter(
    filter: *const QueryFilter,
    ref: PolyRef,
    tile: *const MeshTile,
    poly: *const Poly,
) bool;
pub extern fn ref_filter_get_cost(
    filter: *const QueryFilter,
    pa: *const [3]f32,
    pb: *const [3]f32,
    prev_ref: PolyRef,
    prev_tile: ?*const MeshTile,
    prev_poly: ?*const Poly,
    cur_ref: PolyRef,
    cur_tile: *const MeshTile,
    cur_poly: *const Poly,
    next_ref: PolyRef,
    next_tile: ?*const MeshTile,
    next_poly: ?*const Poly,
) f32;

// ---------------------------------------------------------------------------
// dtNavMeshQuery
// ---------------------------------------------------------------------------

pub extern fn ref_query_alloc() ?*NavMeshQuery;
pub extern fn ref_query_free(query: ?*NavMeshQuery) void;
pub extern fn ref_query_init(query: *NavMeshQuery, nav: ?*const NavMesh, max_nodes: i32) u32;
pub extern fn ref_query_find_path(
    query: *const NavMeshQuery,
    start_ref: PolyRef,
    end_ref: PolyRef,
    start_pos: *const [3]f32,
    end_pos: *const [3]f32,
    filter: *const QueryFilter,
    path: [*]PolyRef,
    path_count: *i32,
    max_path: i32,
) u32;
pub extern fn ref_query_find_straight_path(
    query: *const NavMeshQuery,
    start_pos: *const [3]f32,
    end_pos: *const [3]f32,
    path: [*]const PolyRef,
    path_size: i32,
    straight_path: [*]f32,
    straight_path_flags: ?[*]u8,
    straight_path_refs: ?[*]PolyRef,
    straight_path_count: *i32,
    max_straight_path: i32,
    options: i32,
) u32;
pub extern fn ref_query_init_sliced_find_path(
    query: *NavMeshQuery,
    start_ref: PolyRef,
    end_ref: PolyRef,
    start_pos: *const [3]f32,
    end_pos: *const [3]f32,
    filter: *const QueryFilter,
    options: u32,
) u32;
pub extern fn ref_query_update_sliced_find_path(
    query: *NavMeshQuery,
    max_iter: i32,
    done_iters: ?*i32,
) u32;
pub extern fn ref_query_finalize_sliced_find_path(
    query: *NavMeshQuery,
    path: [*]PolyRef,
    path_count: *i32,
    max_path: i32,
) u32;
pub extern fn ref_query_finalize_sliced_find_path_partial(
    query: *NavMeshQuery,
    existing: [*]const PolyRef,
    existing_size: i32,
    path: [*]PolyRef,
    path_count: *i32,
    max_path: i32,
) u32;
pub extern fn ref_query_find_polys_around_circle(
    query: *const NavMeshQuery,
    start_ref: PolyRef,
    center_pos: *const [3]f32,
    radius: f32,
    filter: *const QueryFilter,
    result_ref: ?[*]PolyRef,
    result_parent: ?[*]PolyRef,
    result_cost: ?[*]f32,
    result_count: *i32,
    max_result: i32,
) u32;
pub extern fn ref_query_find_polys_around_shape(
    query: *const NavMeshQuery,
    start_ref: PolyRef,
    verts: [*]const f32,
    nverts: i32,
    filter: *const QueryFilter,
    result_ref: ?[*]PolyRef,
    result_parent: ?[*]PolyRef,
    result_cost: ?[*]f32,
    result_count: *i32,
    max_result: i32,
) u32;
pub extern fn ref_query_get_path_from_dijkstra_search(
    query: *const NavMeshQuery,
    end_ref: PolyRef,
    path: [*]PolyRef,
    path_count: *i32,
    max_path: i32,
) u32;
/// `findNearestPoly` without `isOverPoly`.
pub extern fn ref_query_find_nearest_poly(
    query: *const NavMeshQuery,
    center: *const [3]f32,
    half_extents: *const [3]f32,
    filter: *const QueryFilter,
    nearest_ref: *PolyRef,
    nearest_pt: ?*[3]f32,
) u32;
/// `findNearestPoly` with `isOverPoly`.
pub extern fn ref_query_find_nearest_poly_over(
    query: *const NavMeshQuery,
    center: *const [3]f32,
    half_extents: *const [3]f32,
    filter: *const QueryFilter,
    nearest_ref: *PolyRef,
    nearest_pt: ?*[3]f32,
    is_over_poly: ?*bool,
) u32;
/// `queryPolygons` into a plain array.
pub extern fn ref_query_query_polygons(
    query: *const NavMeshQuery,
    center: *const [3]f32,
    half_extents: *const [3]f32,
    filter: *const QueryFilter,
    polys: [*]PolyRef,
    poly_count: *i32,
    max_polys: i32,
) u32;
/// `queryPolygons` with a `dtPolyQuery`: `process` gets every batch.
pub extern fn ref_query_query_polygons_cb(
    query: *const NavMeshQuery,
    center: *const [3]f32,
    half_extents: *const [3]f32,
    filter: *const QueryFilter,
    process: PolyQueryFn,
    user: ?*anyopaque,
) u32;
pub extern fn ref_query_find_local_neighbourhood(
    query: *const NavMeshQuery,
    start_ref: PolyRef,
    center_pos: *const [3]f32,
    radius: f32,
    filter: *const QueryFilter,
    result_ref: [*]PolyRef,
    result_parent: ?[*]PolyRef,
    result_count: *i32,
    max_result: i32,
) u32;
pub extern fn ref_query_move_along_surface(
    query: *const NavMeshQuery,
    start_ref: PolyRef,
    start_pos: *const [3]f32,
    end_pos: *const [3]f32,
    filter: *const QueryFilter,
    result_pos: *[3]f32,
    visited: [*]PolyRef,
    visited_count: *i32,
    max_visited_size: i32,
) u32;
/// The `raycast` overload without `dtRaycastHit`.
pub extern fn ref_query_raycast(
    query: *const NavMeshQuery,
    start_ref: PolyRef,
    start_pos: *const [3]f32,
    end_pos: *const [3]f32,
    filter: *const QueryFilter,
    t: *f32,
    hit_normal: ?*[3]f32,
    path: ?[*]PolyRef,
    path_count: ?*i32,
    max_path: i32,
) u32;
/// The `raycast` overload with `dtRaycastHit`. The hit's fields come back
/// through the `out_*` arguments (null skips one); `path` and `max_path` are
/// the caller's buffer for the hit's visited polygons. Fields upstream leaves
/// unset read as zero.
pub extern fn ref_query_raycast_hit(
    query: *const NavMeshQuery,
    start_ref: PolyRef,
    start_pos: *const [3]f32,
    end_pos: *const [3]f32,
    filter: *const QueryFilter,
    options: u32,
    prev_ref: PolyRef,
    out_t: ?*f32,
    out_hit_normal: ?*[3]f32,
    out_hit_edge_index: ?*i32,
    path: ?[*]PolyRef,
    out_path_count: ?*i32,
    max_path: i32,
    out_path_cost: ?*f32,
) u32;
pub extern fn ref_query_find_distance_to_wall(
    query: *const NavMeshQuery,
    start_ref: PolyRef,
    center_pos: *const [3]f32,
    max_radius: f32,
    filter: *const QueryFilter,
    hit_dist: *f32,
    hit_pos: *[3]f32,
    hit_normal: *[3]f32,
) u32;
pub extern fn ref_query_get_poly_wall_segments(
    query: *const NavMeshQuery,
    ref: PolyRef,
    filter: *const QueryFilter,
    segment_verts: [*]f32,
    segment_refs: ?[*]PolyRef,
    segment_count: *i32,
    max_segments: i32,
) u32;
pub extern fn ref_query_find_random_point(
    query: *const NavMeshQuery,
    filter: *const QueryFilter,
    frand: RandomFn,
    random_ref: *PolyRef,
    random_pt: *[3]f32,
) u32;
pub extern fn ref_query_find_random_point_around_circle(
    query: *const NavMeshQuery,
    start_ref: PolyRef,
    center_pos: *const [3]f32,
    max_radius: f32,
    filter: *const QueryFilter,
    frand: RandomFn,
    random_ref: *PolyRef,
    random_pt: *[3]f32,
) u32;
pub extern fn ref_query_closest_point_on_poly(
    query: *const NavMeshQuery,
    ref: PolyRef,
    pos: *const [3]f32,
    closest: *[3]f32,
    pos_over_poly: ?*bool,
) u32;
pub extern fn ref_query_closest_point_on_poly_boundary(
    query: *const NavMeshQuery,
    ref: PolyRef,
    pos: *const [3]f32,
    closest: *[3]f32,
) u32;
pub extern fn ref_query_get_poly_height(
    query: *const NavMeshQuery,
    ref: PolyRef,
    pos: *const [3]f32,
    height: ?*f32,
) u32;
pub extern fn ref_query_is_valid_poly_ref(
    query: *const NavMeshQuery,
    ref: PolyRef,
    filter: *const QueryFilter,
) bool;
pub extern fn ref_query_is_in_closed_list(query: *const NavMeshQuery, ref: PolyRef) bool;
pub extern fn ref_query_get_node_pool(query: *const NavMeshQuery) ?*NodePool;
pub extern fn ref_query_get_attached_nav_mesh(query: *const NavMeshQuery) ?*const NavMesh;

// ---------------------------------------------------------------------------
// DetourNode.h
// ---------------------------------------------------------------------------

pub extern fn ref_node_fields(
    node: *const Node,
    pos3: *[3]f32,
    cost: *f32,
    total: *f32,
    pidx: *u32,
    state: *u32,
    flags: *u32,
    id: *PolyRef,
) void;
pub extern fn ref_node_set(
    node: *Node,
    pos3: *const [3]f32,
    cost: f32,
    total: f32,
    pidx: u32,
    state: u32,
    flags: u32,
    id: PolyRef,
) void;
pub extern fn ref_node_get_pidx(node: *const Node) u32;
pub extern fn ref_node_get_state(node: *const Node) u32;
pub extern fn ref_node_get_flags(node: *const Node) u32;
pub extern fn ref_node_set_pidx(node: *Node, pidx: u32) void;
pub extern fn ref_node_set_state(node: *Node, state: u32) void;
pub extern fn ref_node_set_flags(node: *Node, flags: u32) void;

pub extern fn ref_node_pool_create(max_nodes: i32, hash_size: i32) ?*NodePool;
pub extern fn ref_node_pool_destroy(pool: ?*NodePool) void;
pub extern fn ref_node_pool_clear(pool: *NodePool) void;
pub extern fn ref_node_pool_get_node(pool: *NodePool, id: PolyRef, state: u8) ?*Node;
pub extern fn ref_node_pool_find_node(pool: *NodePool, id: PolyRef, state: u8) ?*Node;
pub extern fn ref_node_pool_find_nodes(
    pool: *NodePool,
    id: PolyRef,
    nodes: [*]?*Node,
    max_nodes: i32,
) u32;
pub extern fn ref_node_pool_get_node_idx(pool: *const NodePool, node: ?*const Node) u32;
pub extern fn ref_node_pool_get_node_at_idx(pool: *NodePool, idx: u32) ?*Node;
pub extern fn ref_node_pool_get_mem_used(pool: *const NodePool) i32;
pub extern fn ref_node_pool_get_max_nodes(pool: *const NodePool) i32;
pub extern fn ref_node_pool_get_hash_size(pool: *const NodePool) i32;
pub extern fn ref_node_pool_get_first(pool: *const NodePool, bucket: i32) u16;
pub extern fn ref_node_pool_get_next(pool: *const NodePool, i: i32) u16;
pub extern fn ref_node_pool_get_node_count(pool: *const NodePool) i32;

pub extern fn ref_node_queue_create(n: i32) ?*NodeQueue;
pub extern fn ref_node_queue_destroy(queue: ?*NodeQueue) void;
pub extern fn ref_node_queue_clear(queue: *NodeQueue) void;
pub extern fn ref_node_queue_top(queue: *NodeQueue) ?*Node;
pub extern fn ref_node_queue_pop(queue: *NodeQueue) ?*Node;
pub extern fn ref_node_queue_push(queue: *NodeQueue, node: *Node) void;
pub extern fn ref_node_queue_modify(queue: *NodeQueue, node: *Node) void;
pub extern fn ref_node_queue_empty(queue: *const NodeQueue) bool;
pub extern fn ref_node_queue_get_mem_used(queue: *const NodeQueue) i32;
pub extern fn ref_node_queue_get_capacity(queue: *const NodeQueue) i32;
