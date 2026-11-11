//! Declarations for the C shim over upstream DetourTileCache,
//! test/shim/tile_cache_shim.cpp.
//!
//! The pure-Zig tile cache is tested against upstream through these functions,
//! byte for byte. Names are the shim's: `ref_tcb_` for the free functions of
//! DetourTileCacheBuilder.h and `ref_tc_` for everything else, one function per
//! upstream function or method, object pointer first, upstream's argument
//! order. Statuses are `u32`. Positions are `*const [3]f32` in and `*[3]f32`
//! out. Where upstream takes a reference the shim takes a pointer.
//!
//! A pointer a getter returns points into the object asked, and goes stale when
//! that object is freed or changed. The tile cache keeps the allocator,
//! compressor and mesh process given to `ref_tc_init`, and the data given to
//! `ref_tc_add_tile` (it does not copy it), so they must outlive it.

const core = @import("reference_detour.zig");

/// `dtObstacleRef`: 32 bits in every build configuration.
pub const ObstacleRef = u32;
/// `dtCompressedTileRef`: 32 bits in every build configuration, unlike
/// `core.TileRef`.
pub const CompressedTileRef = u32;

pub const TileCache = opaque {};
/// A `dtTileCacheAlloc`, from `ref_tc_alloc_create_default` or `ref_tc_alloc_create_linear`.
pub const TileCacheAlloc = opaque {};
/// A `dtTileCacheCompressor` that forwards to the callbacks given to
/// `ref_tc_compressor_create`.
pub const Compressor = opaque {};
/// A `dtTileCacheMeshProcess` that forwards to the callback given to
/// `ref_tc_mesh_process_create`.
pub const MeshProcess = opaque {};
/// A `dtTileCacheLayer`.
pub const Layer = opaque {};
/// A `dtTileCacheContourSet`.
pub const ContourSet = opaque {};
/// A `dtTileCachePolyMesh`.
pub const PolyMesh = opaque {};
/// A `dtCompressedTile`, read with the `ref_tc_tile_*` functions.
pub const CompressedTile = opaque {};
/// A `dtTileCacheObstacle`, read with `ref_tc_obstacle_snapshot`.
pub const Obstacle = opaque {};

/// `DT_MAX_TOUCHED_TILES`.
pub const max_touched_tiles = 8;

/// Gives the most bytes `buffer_size` bytes can compress to
/// (`dtTileCacheCompressor::maxCompressedSize`).
pub const CompressorMaxSizeFn = *const fn (user: ?*anyopaque, buffer_size: i32) callconv(.c) i32;

/// `dtTileCacheCompressor::compress`; returns a status.
pub const CompressorCompressFn = *const fn (
    user: ?*anyopaque,
    buffer: [*]const u8,
    buffer_size: i32,
    compressed: [*]u8,
    max_compressed_size: i32,
    compressed_size: *i32,
) callconv(.c) u32;

/// `dtTileCacheCompressor::decompress`; returns a status.
pub const CompressorDecompressFn = *const fn (
    user: ?*anyopaque,
    compressed: [*]const u8,
    compressed_size: i32,
    buffer: [*]u8,
    max_buffer_size: i32,
    buffer_size: *i32,
) callconv(.c) u32;

/// `dtTileCacheMeshProcess::process`.
pub const MeshProcessFn = *const fn (
    user: ?*anyopaque,
    params: *core.NavMeshCreateParams,
    poly_areas: [*]u8,
    poly_flags: [*]u16,
) callconv(.c) void;

/// `dtTileCacheLayerHeader`.
pub const TileCacheLayerHeader = extern struct {
    magic: i32,
    version: i32,
    tx: i32,
    ty: i32,
    tlayer: i32,
    bmin: [3]f32,
    bmax: [3]f32,
    hmin: u16,
    hmax: u16,
    width: u8,
    height: u8,
    minx: u8,
    maxx: u8,
    miny: u8,
    maxy: u8,
};

/// `dtTileCacheParams`.
pub const TileCacheParams = extern struct {
    orig: [3]f32,
    cs: f32,
    ch: f32,
    width: i32,
    height: i32,
    walkable_height: f32,
    walkable_radius: f32,
    walkable_climb: f32,
    max_simplification_error: f32,
    max_tiles: i32,
    max_obstacles: i32,
};

/// `RefObstacleSnapshot`: a flat copy of one `dtTileCacheObstacle`. Only the
/// shape that `obstacle_type` names is filled (0 cylinder, 1 box, 2 oriented
/// box); the other shapes stay zero. The shim zeroes it before filling it.
pub const ObstacleSnapshot = extern struct {
    obstacle_type: u8,
    state: u8,
    ntouched: u8,
    npending: u8,
    salt: u16,
    reserved: u16,
    cylinder_pos: [3]f32,
    cylinder_radius: f32,
    cylinder_height: f32,
    box_bmin: [3]f32,
    box_bmax: [3]f32,
    oriented_center: [3]f32,
    oriented_half_extents: [3]f32,
    oriented_rot_aux: [2]f32,
    /// All `max_touched_tiles` entries, including those past `ntouched`.
    touched: [max_touched_tiles]CompressedTileRef,
    /// All `max_touched_tiles` entries, including those past `npending`.
    pending: [max_touched_tiles]CompressedTileRef,
};

// ---------------------------------------------------------------------------
// DetourTileCacheBuilder.h: free functions
// ---------------------------------------------------------------------------

/// On success `out_data` points at a buffer from `dtAlloc` that the caller
/// owns: give it to `ref_tc_add_tile` with the free-data flag, or release it
/// with `ref_tcb_free_data`.
pub extern fn ref_tcb_build_layer(
    comp: *Compressor,
    header: *TileCacheLayerHeader,
    heights: [*]const u8,
    areas: [*]const u8,
    cons: [*]const u8,
    out_data: *?[*]u8,
    out_data_size: *i32,
) u32;
/// `dtFree`, for a buffer from `ref_tcb_build_layer` that no tile cache took over.
pub extern fn ref_tcb_free_data(data: ?[*]u8) void;
pub extern fn ref_tcb_free_layer(alloc: *TileCacheAlloc, layer: ?*Layer) void;
/// `layer_out` comes from `alloc`; it is invalid after `ref_tcb_free_layer` and
/// after anything that resets or frees `alloc`, including a tile cache build.
/// Upstream fails with invalid-param on a null `compressed` or `layer_out`.
pub extern fn ref_tcb_decompress_layer(
    alloc: *TileCacheAlloc,
    comp: *Compressor,
    compressed: ?[*]u8,
    compressed_size: i32,
    layer_out: ?*?*Layer,
) u32;
/// Upstream does not check the allocation: out of room, a linear allocator
/// makes this write through a null pointer. Size it generously.
pub extern fn ref_tcb_alloc_contour_set(alloc: *TileCacheAlloc) ?*ContourSet;
pub extern fn ref_tcb_free_contour_set(alloc: *TileCacheAlloc, cset: ?*ContourSet) void;
/// Upstream does not check the allocation: out of room, a linear allocator
/// makes this write through a null pointer. Size it generously.
pub extern fn ref_tcb_alloc_poly_mesh(alloc: *TileCacheAlloc) ?*PolyMesh;
pub extern fn ref_tcb_free_poly_mesh(alloc: *TileCacheAlloc, mesh: ?*PolyMesh) void;
pub extern fn ref_tcb_mark_cylinder_area(
    layer: *Layer,
    orig: *const [3]f32,
    cs: f32,
    ch: f32,
    pos: *const [3]f32,
    radius: f32,
    height: f32,
    area_id: u8,
) u32;
/// The overload of `dtMarkBoxArea` that takes the box's bounds.
pub extern fn ref_tcb_mark_box_area(
    layer: *Layer,
    orig: *const [3]f32,
    cs: f32,
    ch: f32,
    bmin: *const [3]f32,
    bmax: *const [3]f32,
    area_id: u8,
) u32;
/// The overload of `dtMarkBoxArea` that takes a centre, half extents and the
/// two floats of rotation (`dtObstacleOrientedBox::rotAux`).
pub extern fn ref_tcb_mark_oriented_box_area(
    layer: *Layer,
    orig: *const [3]f32,
    cs: f32,
    ch: f32,
    center: *const [3]f32,
    half_extents: *const [3]f32,
    rot_aux: *const [2]f32,
    area_id: u8,
) u32;
pub extern fn ref_tcb_build_regions(alloc: *TileCacheAlloc, layer: *Layer, walkable_climb: i32) u32;
pub extern fn ref_tcb_build_contours(
    alloc: *TileCacheAlloc,
    layer: *Layer,
    walkable_climb: i32,
    max_error: f32,
    lcset: *ContourSet,
) u32;
pub extern fn ref_tcb_build_poly_mesh(
    alloc: *TileCacheAlloc,
    lcset: *ContourSet,
    mesh: *PolyMesh,
) u32;
pub extern fn ref_tcb_header_swap_endian(data: [*]u8, data_size: i32) bool;

// ---------------------------------------------------------------------------
// DetourTileCacheBuilder.h: reading layers, contour sets and polygon meshes.
// The pointers point into the object and are invalid once it is freed.
// ---------------------------------------------------------------------------

pub extern fn ref_tcb_layer_header(layer: *const Layer) *TileCacheLayerHeader;
pub extern fn ref_tcb_layer_reg_count(layer: *const Layer) u8;
/// Each of the four grids holds `header.width * header.height` bytes.
pub extern fn ref_tcb_layer_heights(layer: *const Layer) [*]u8;
pub extern fn ref_tcb_layer_areas(layer: *const Layer) [*]u8;
pub extern fn ref_tcb_layer_cons(layer: *const Layer) [*]u8;
pub extern fn ref_tcb_layer_regs(layer: *const Layer) [*]u8;

pub extern fn ref_tcb_contour_set_count(cset: *const ContourSet) i32;
/// The per-contour readers return 0 or null when `i` is not in `0..count`.
pub extern fn ref_tcb_contour_nverts(cset: *const ContourSet, i: i32) i32;
/// `nverts * 4` bytes: x, y, z and the portal/remove byte of each vertex. Null
/// for a contour with no vertices.
pub extern fn ref_tcb_contour_verts(cset: *const ContourSet, i: i32) ?[*]const u8;
pub extern fn ref_tcb_contour_reg(cset: *const ContourSet, i: i32) u8;
pub extern fn ref_tcb_contour_area(cset: *const ContourSet, i: i32) u8;

pub extern fn ref_tcb_poly_mesh_nvp(mesh: *const PolyMesh) i32;
pub extern fn ref_tcb_poly_mesh_nverts(mesh: *const PolyMesh) i32;
pub extern fn ref_tcb_poly_mesh_npolys(mesh: *const PolyMesh) i32;
/// Three values per vertex. Null until `ref_tcb_build_poly_mesh` has run.
pub extern fn ref_tcb_poly_mesh_verts(mesh: *const PolyMesh) ?[*]const u16;
/// `nvp * 2` values per polygon. Null until `ref_tcb_build_poly_mesh` has run.
pub extern fn ref_tcb_poly_mesh_polys(mesh: *const PolyMesh) ?[*]const u16;
/// One value per polygon. Null until `ref_tcb_build_poly_mesh` has run.
pub extern fn ref_tcb_poly_mesh_flags(mesh: *const PolyMesh) ?[*]const u16;
/// One value per polygon. Null until `ref_tcb_build_poly_mesh` has run.
pub extern fn ref_tcb_poly_mesh_areas(mesh: *const PolyMesh) ?[*]const u8;

// ---------------------------------------------------------------------------
// dtTileCacheAlloc
// ---------------------------------------------------------------------------

/// The base class itself: `dtAlloc`, `dtFree` and a reset that does nothing.
pub extern fn ref_tc_alloc_create_default() ?*TileCacheAlloc;
/// A bump allocator over `capacity` bytes, as `LinearAllocator` in RecastDemo's
/// Sample_TempObstacles.cpp: free does nothing, reset rewinds, and nothing is
/// aligned or padded, so `ref_tc_alloc_linear_top` counts exactly the bytes asked for.
pub extern fn ref_tc_alloc_create_linear(capacity: usize) ?*TileCacheAlloc;
/// Destroys an allocator made by either create function. Anything still
/// allocated from it becomes invalid.
pub extern fn ref_tc_alloc_destroy(alloc: ?*TileCacheAlloc) void;
pub extern fn ref_tc_alloc_reset(alloc: *TileCacheAlloc) void;
/// Upstream's `alloc`. Null when a linear allocator is out of room.
pub extern fn ref_tc_alloc_allocate(alloc: *TileCacheAlloc, size: usize) ?[*]u8;
/// Upstream's `free`.
pub extern fn ref_tc_alloc_deallocate(alloc: *TileCacheAlloc, ptr: ?[*]u8) void;
/// The three readers below are only for allocators made by
/// `ref_tc_alloc_create_linear`; on any other allocator they read garbage.
pub extern fn ref_tc_alloc_linear_capacity(alloc: *const TileCacheAlloc) usize;
/// Bytes handed out since the last reset.
pub extern fn ref_tc_alloc_linear_top(alloc: *const TileCacheAlloc) usize;
/// The largest `top` seen at any reset.
pub extern fn ref_tc_alloc_linear_high(alloc: *const TileCacheAlloc) usize;

// ---------------------------------------------------------------------------
// dtTileCacheCompressor and dtTileCacheMeshProcess
// ---------------------------------------------------------------------------

/// Null if memory runs out. The statuses the callbacks return reach the
/// caller of compress and decompress unchanged.
pub extern fn ref_tc_compressor_create(
    max_size: CompressorMaxSizeFn,
    compress: CompressorCompressFn,
    decompress: CompressorDecompressFn,
    user: ?*anyopaque,
) ?*Compressor;
pub extern fn ref_tc_compressor_destroy(comp: ?*Compressor) void;
/// Null if memory runs out.
pub extern fn ref_tc_mesh_process_create(process: MeshProcessFn, user: ?*anyopaque) ?*MeshProcess;
pub extern fn ref_tc_mesh_process_destroy(mesh_process: ?*MeshProcess) void;

// ---------------------------------------------------------------------------
// dtTileCache
// ---------------------------------------------------------------------------

/// `dtAllocTileCache`.
pub extern fn ref_tc_create() ?*TileCache;
/// `dtFreeTileCache`. Also frees the data of every tile added with the free-data flag.
pub extern fn ref_tc_destroy(tc: ?*TileCache) void;
/// The tile cache keeps `talloc`, `tcomp` and `tmproc` (which may be null) for
/// as long as it lives.
pub extern fn ref_tc_init(
    tc: *TileCache,
    params: *const TileCacheParams,
    talloc: *TileCacheAlloc,
    tcomp: *Compressor,
    tmproc: ?*MeshProcess,
) u32;
pub extern fn ref_tc_get_alloc(tc: *TileCache) ?*TileCacheAlloc;
pub extern fn ref_tc_get_compressor(tc: *TileCache) ?*Compressor;
/// Points into the tile cache.
pub extern fn ref_tc_get_params(tc: *const TileCache) *const TileCacheParams;
pub extern fn ref_tc_get_tiles_at(
    tc: *const TileCache,
    tx: i32,
    ty: i32,
    tiles: [*]CompressedTileRef,
    max_tiles: i32,
) i32;
pub extern fn ref_tc_get_tile_at(tc: *TileCache, tx: i32, ty: i32, tlayer: i32) ?*CompressedTile;
pub extern fn ref_tc_get_tile_ref(
    tc: *const TileCache,
    tile: ?*const CompressedTile,
) CompressedTileRef;
pub extern fn ref_tc_get_tile_by_ref(
    tc: *const TileCache,
    tile_ref: CompressedTileRef,
) ?*const CompressedTile;
/// The tile cache keeps `data` and, with `flags` bit 1 (free-data), frees it
/// with `dtFree` when the tile is removed or the cache is destroyed. `result`
/// may be null.
pub extern fn ref_tc_add_tile(
    tc: *TileCache,
    data: [*]u8,
    data_size: i32,
    flags: u8,
    result: ?*CompressedTileRef,
) u32;
/// `data` and `data_size` may be null. When the tile owned its data they are
/// zeroed; otherwise they give the data back to the caller.
pub extern fn ref_tc_remove_tile(
    tc: *TileCache,
    tile_ref: CompressedTileRef,
    data: ?*?[*]u8,
    data_size: ?*i32,
) u32;
/// The cylinder obstacle. `result` may be null here and in the next two.
pub extern fn ref_tc_add_obstacle(
    tc: *TileCache,
    pos: *const [3]f32,
    radius: f32,
    height: f32,
    result: ?*ObstacleRef,
) u32;
/// The overload of `addBoxObstacle` that takes bounds.
pub extern fn ref_tc_add_box_obstacle(
    tc: *TileCache,
    bmin: *const [3]f32,
    bmax: *const [3]f32,
    result: ?*ObstacleRef,
) u32;
/// The overload of `addBoxObstacle` that takes a centre, half extents and a
/// rotation about y.
pub extern fn ref_tc_add_oriented_box_obstacle(
    tc: *TileCache,
    center: *const [3]f32,
    half_extents: *const [3]f32,
    y_radians: f32,
    result: ?*ObstacleRef,
) u32;
pub extern fn ref_tc_remove_obstacle(tc: *TileCache, obstacle_ref: ObstacleRef) u32;
pub extern fn ref_tc_query_tiles(
    tc: *const TileCache,
    bmin: *const [3]f32,
    bmax: *const [3]f32,
    results: [*]CompressedTileRef,
    result_count: *i32,
    max_results: i32,
) u32;
/// `up_to_date` may be null.
pub extern fn ref_tc_update(
    tc: *TileCache,
    dt: f32,
    navmesh: *core.NavMesh,
    up_to_date: ?*bool,
) u32;
/// Resets the tile cache's allocator, so anything allocated from it earlier is invalid.
pub extern fn ref_tc_build_nav_mesh_tiles_at(
    tc: *TileCache,
    tx: i32,
    ty: i32,
    navmesh: *core.NavMesh,
) u32;
/// Resets the tile cache's allocator, so anything allocated from it earlier is invalid.
pub extern fn ref_tc_build_nav_mesh_tile(
    tc: *TileCache,
    tile_ref: CompressedTileRef,
    navmesh: *core.NavMesh,
) u32;
pub extern fn ref_tc_calc_tight_tile_bounds(
    tc: *const TileCache,
    header: *const TileCacheLayerHeader,
    bmin: *[3]f32,
    bmax: *[3]f32,
) void;
pub extern fn ref_tc_get_obstacle_bounds(
    tc: *const TileCache,
    ob: *const Obstacle,
    bmin: *[3]f32,
    bmax: *[3]f32,
) void;
/// `params.max_tiles`. The tile pool does not move, but its entries change with
/// every add, remove and update.
pub extern fn ref_tc_get_tile_count(tc: *const TileCache) i32;
/// `i` is not checked.
pub extern fn ref_tc_get_tile(tc: *const TileCache, i: i32) *const CompressedTile;
/// `params.max_obstacles`. The obstacle pool does not move, but its entries
/// change with every add, remove and update.
pub extern fn ref_tc_get_obstacle_count(tc: *const TileCache) i32;
/// `i` is not checked.
pub extern fn ref_tc_get_obstacle(tc: *const TileCache, i: i32) *const Obstacle;
pub extern fn ref_tc_get_obstacle_by_ref(
    tc: *TileCache,
    obstacle_ref: ObstacleRef,
) ?*const Obstacle;
pub extern fn ref_tc_get_obstacle_ref(tc: *const TileCache, ob: ?*const Obstacle) ObstacleRef;
pub extern fn ref_tc_encode_tile_id(tc: *const TileCache, salt: u32, it: u32) CompressedTileRef;
pub extern fn ref_tc_decode_tile_id_salt(tc: *const TileCache, tile_ref: CompressedTileRef) u32;
pub extern fn ref_tc_decode_tile_id_tile(tc: *const TileCache, tile_ref: CompressedTileRef) u32;
pub extern fn ref_tc_encode_obstacle_id(tc: *const TileCache, salt: u32, it: u32) ObstacleRef;
pub extern fn ref_tc_decode_obstacle_id_salt(tc: *const TileCache, obstacle_ref: ObstacleRef) u32;
pub extern fn ref_tc_decode_obstacle_id_obstacle(
    tc: *const TileCache,
    obstacle_ref: ObstacleRef,
) u32;

// ---------------------------------------------------------------------------
// dtCompressedTile: plain data, read field by field. The pointers point into
// the tile cache's pool entry or the data it keeps, and go stale when the tile
// is removed.
// ---------------------------------------------------------------------------

pub extern fn ref_tc_tile_salt(tile: *const CompressedTile) u32;
pub extern fn ref_tc_tile_header(tile: *const CompressedTile) ?*TileCacheLayerHeader;
pub extern fn ref_tc_tile_compressed(tile: *const CompressedTile) ?[*]u8;
pub extern fn ref_tc_tile_compressed_size(tile: *const CompressedTile) i32;
pub extern fn ref_tc_tile_data(tile: *const CompressedTile) ?[*]u8;
pub extern fn ref_tc_tile_data_size(tile: *const CompressedTile) i32;
pub extern fn ref_tc_tile_flags(tile: *const CompressedTile) u32;

// ---------------------------------------------------------------------------
// dtTileCacheObstacle
// ---------------------------------------------------------------------------

/// Zeroes `out`, then copies the obstacle into it. Returns false, leaving `out`
/// zeroed, when `ob` is null.
pub extern fn ref_tc_obstacle_snapshot(ob: ?*const Obstacle, out: *ObstacleSnapshot) bool;

// ---------------------------------------------------------------------------
// Layout checks: compare with `@sizeOf` and `@offsetOf` of the mirrors above,
// or of the port's own structs. The union members of the obstacle all start at
// offset 0; their nested offsets are relative to the whole obstacle.
// ---------------------------------------------------------------------------

pub extern fn ref_sizeof_tile_cache_layer_header() usize;
pub extern fn ref_offsetof_tile_cache_layer_header_magic() usize;
pub extern fn ref_offsetof_tile_cache_layer_header_version() usize;
pub extern fn ref_offsetof_tile_cache_layer_header_tx() usize;
pub extern fn ref_offsetof_tile_cache_layer_header_ty() usize;
pub extern fn ref_offsetof_tile_cache_layer_header_tlayer() usize;
pub extern fn ref_offsetof_tile_cache_layer_header_bmin() usize;
pub extern fn ref_offsetof_tile_cache_layer_header_bmax() usize;
pub extern fn ref_offsetof_tile_cache_layer_header_hmin() usize;
pub extern fn ref_offsetof_tile_cache_layer_header_hmax() usize;
pub extern fn ref_offsetof_tile_cache_layer_header_width() usize;
pub extern fn ref_offsetof_tile_cache_layer_header_height() usize;
pub extern fn ref_offsetof_tile_cache_layer_header_minx() usize;
pub extern fn ref_offsetof_tile_cache_layer_header_maxx() usize;
pub extern fn ref_offsetof_tile_cache_layer_header_miny() usize;
pub extern fn ref_offsetof_tile_cache_layer_header_maxy() usize;

pub extern fn ref_sizeof_tile_cache_params() usize;
pub extern fn ref_offsetof_tile_cache_params_orig() usize;
pub extern fn ref_offsetof_tile_cache_params_cs() usize;
pub extern fn ref_offsetof_tile_cache_params_ch() usize;
pub extern fn ref_offsetof_tile_cache_params_width() usize;
pub extern fn ref_offsetof_tile_cache_params_height() usize;
pub extern fn ref_offsetof_tile_cache_params_walkable_height() usize;
pub extern fn ref_offsetof_tile_cache_params_walkable_radius() usize;
pub extern fn ref_offsetof_tile_cache_params_walkable_climb() usize;
pub extern fn ref_offsetof_tile_cache_params_max_simplification_error() usize;
pub extern fn ref_offsetof_tile_cache_params_max_tiles() usize;
pub extern fn ref_offsetof_tile_cache_params_max_obstacles() usize;

pub extern fn ref_sizeof_compressed_tile() usize;
pub extern fn ref_offsetof_compressed_tile_salt() usize;
pub extern fn ref_offsetof_compressed_tile_header() usize;
pub extern fn ref_offsetof_compressed_tile_compressed() usize;
pub extern fn ref_offsetof_compressed_tile_compressed_size() usize;
pub extern fn ref_offsetof_compressed_tile_data() usize;
pub extern fn ref_offsetof_compressed_tile_data_size() usize;
pub extern fn ref_offsetof_compressed_tile_flags() usize;
pub extern fn ref_offsetof_compressed_tile_next() usize;

pub extern fn ref_sizeof_tile_cache_obstacle() usize;
pub extern fn ref_offsetof_tile_cache_obstacle_cylinder_pos() usize;
pub extern fn ref_offsetof_tile_cache_obstacle_cylinder_radius() usize;
pub extern fn ref_offsetof_tile_cache_obstacle_cylinder_height() usize;
pub extern fn ref_offsetof_tile_cache_obstacle_box_bmin() usize;
pub extern fn ref_offsetof_tile_cache_obstacle_box_bmax() usize;
pub extern fn ref_offsetof_tile_cache_obstacle_oriented_box_center() usize;
pub extern fn ref_offsetof_tile_cache_obstacle_oriented_box_half_extents() usize;
pub extern fn ref_offsetof_tile_cache_obstacle_oriented_box_rot_aux() usize;
pub extern fn ref_offsetof_tile_cache_obstacle_touched() usize;
pub extern fn ref_offsetof_tile_cache_obstacle_pending() usize;
pub extern fn ref_offsetof_tile_cache_obstacle_salt() usize;
pub extern fn ref_offsetof_tile_cache_obstacle_type() usize;
pub extern fn ref_offsetof_tile_cache_obstacle_state() usize;
pub extern fn ref_offsetof_tile_cache_obstacle_ntouched() usize;
pub extern fn ref_offsetof_tile_cache_obstacle_npending() usize;
pub extern fn ref_offsetof_tile_cache_obstacle_next() usize;

pub extern fn ref_sizeof_obstacle_snapshot() usize;
