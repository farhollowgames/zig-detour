//! Declarations for test/shim/recast_shim.cpp: test input for the Zig port of
//! Detour, made with upstream Recast (recastnavigation v1.6.0).
//!
//! The shim runs Recast's standard pipeline on a triangle soup, following the
//! RecastDemo samples (watershed, monotone or layer partitioning; the filters
//! of the samples all on), and hands back flat arrays. They belong to the
//! shim until the matching `ref_recast_free_*` call.
//!
//! Lengths are in world units, except: `detail_sample_dist` is a multiple of
//! `cell_size` and `detail_sample_max_error` a multiple of `cell_height`, as
//! in the samples' sliders (a `detail_sample_dist` below 0.9 turns detail
//! sampling off); region sizes are cells per side.

/// What `ref_recast_build_tile` and `ref_recast_build_layers` keep alive.
pub const TileOwner = opaque {};
pub const LayerOwner = opaque {};

/// `partition` values of `RecastConfig`.
pub const partition_watershed: i32 = 0;
pub const partition_monotone: i32 = 1;
pub const partition_layers: i32 = 2;

/// `RefRecastConfig`.
pub const RecastConfig = extern struct {
    cell_size: f32,
    cell_height: f32,
    agent_height: f32,
    agent_radius: f32,
    agent_max_climb: f32,
    agent_max_slope: f32,
    edge_max_len: f32,
    edge_max_error: f32,
    detail_sample_dist: f32,
    detail_sample_max_error: f32,
    region_min_size: i32,
    region_merge_size: i32,
    verts_per_poly: i32,
    /// In cells; 0 builds the whole mesh as one tile.
    tile_size: i32,
    /// `partition_watershed`, `partition_monotone` or `partition_layers`.
    partition: i32,
};

/// `RefRecastTile`: everything `dtNavMeshCreateParams` takes from Recast.
/// Polygon areas are as Recast made them; `poly_flags` is 1 where the area is
/// `RC_WALKABLE_AREA` (63), else 0.
pub const RecastTile = extern struct {
    verts: ?[*]const u16,
    vert_count: i32,
    polys: ?[*]const u16,
    poly_flags: ?[*]const u16,
    poly_areas: ?[*]const u8,
    poly_count: i32,
    nvp: i32,
    detail_meshes: ?[*]const u32,
    detail_verts: ?[*]const f32,
    detail_vert_count: i32,
    detail_tris: ?[*]const u8,
    detail_tri_count: i32,
    bmin: [3]f32,
    bmax: [3]f32,
    walkable_height: f32,
    walkable_radius: f32,
    walkable_climb: f32,
    cs: f32,
    ch: f32,
    owner: ?*TileOwner,
};

/// `RefRecastLayer`: one heightfield layer, with the fields of
/// `dtTileCacheLayerHeader`. The three grids hold `width * height` bytes.
pub const RecastLayer = extern struct {
    tile_x: i32,
    tile_y: i32,
    layer: i32,
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
    heights: ?[*]const u8,
    areas: ?[*]const u8,
    cons: ?[*]const u8,
};

/// Sizes of the three structs above in the shim, to check the mirrors.
pub extern fn ref_recast_sizeof_config() usize;
pub extern fn ref_recast_sizeof_tile() usize;
pub extern fn ref_recast_sizeof_layer() usize;

/// Builds the polygon mesh and detail mesh of one tile, as
/// `Sample_TileMesh::buildTileMesh` does up to `dtCreateNavMeshData`; with
/// `cfg.tile_size == 0` the whole mesh is one tile, as `Sample_SoloMesh`, and
/// `tile_x` / `tile_y` are ignored. `verts` holds `3 * nverts` floats and
/// `tris` `3 * ntris` indices. `mesh_bmin` and `mesh_bmax` are the bounds the
/// tile grid starts from.
///
/// Returns false, with `out` zeroed, for an empty tile, a failed Recast step,
/// or `verts_per_poly` outside 3...6 (Detour cannot take more). On true, free
/// the result with `ref_recast_free_tile`.
pub extern fn ref_recast_build_tile(
    verts: [*]const f32,
    nverts: i32,
    tris: [*]const i32,
    ntris: i32,
    cfg: *const RecastConfig,
    mesh_bmin: *const [3]f32,
    mesh_bmax: *const [3]f32,
    tile_x: i32,
    tile_y: i32,
    out: *RecastTile,
) bool;

/// Frees everything `ref_recast_build_tile` made and zeroes `tile`.
pub extern fn ref_recast_free_tile(tile: *RecastTile) void;

/// Builds the heightfield layers of one tile, as
/// `Sample_TempObstacles::rasterizeTileLayers` does up to
/// `dtBuildTileCacheLayer`. `cfg.tile_size` must be 1...255. Only the cell
/// sizes, the agent fields and `tile_size` of `cfg` matter here; the region,
/// contour and mesh settings and `partition` are not used. Fills
/// `out[0..count]` and returns `count`, at most `max_layers`; returns 0 for an
/// empty tile, a failed step or bad arguments, with `owner` set to null. The
/// layers' grids live until `ref_recast_free_layers(owner)`.
pub extern fn ref_recast_build_layers(
    verts: [*]const f32,
    nverts: i32,
    tris: [*]const i32,
    ntris: i32,
    cfg: *const RecastConfig,
    mesh_bmin: *const [3]f32,
    mesh_bmax: *const [3]f32,
    tile_x: i32,
    tile_y: i32,
    out: [*]RecastLayer,
    max_layers: i32,
    owner: *?*LayerOwner,
) i32;

pub extern fn ref_recast_free_layers(owner: ?*LayerOwner) void;
