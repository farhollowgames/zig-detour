//! The tile cache: compressed heightfield layers kept by grid position, and
//! the obstacles laid over them. Adding or removing an obstacle queues the
//! tiles it touches, and `update` rebuilds one queued tile per call: it
//! unpacks the layer, marks the obstacles into it, builds regions, contours
//! and polygons, and swaps the result into the navigation mesh.
//!
//! Tile and obstacle references have upstream Detour's bit layout, and every
//! step keeps upstream's order, so the references handed out, the order
//! tiles are rebuilt in and the rebuilt tiles match it exactly.

const std = @import("std");
const common = @import("common.zig");
const Status = @import("status.zig").Status;
const nav_mesh = @import("nav_mesh.zig");
const NavMesh = nav_mesh.NavMesh;
const nav_mesh_builder = @import("nav_mesh_builder.zig");
const NavMeshCreateParams = nav_mesh_builder.NavMeshCreateParams;
const tile_cache_builder = @import("tile_cache_builder.zig");
const BuildAllocator = tile_cache_builder.BuildAllocator;
const Compressor = tile_cache_builder.Compressor;
const ContourSet = tile_cache_builder.ContourSet;
const Layer = tile_cache_builder.Layer;
const LayerHeader = tile_cache_builder.LayerHeader;
const PolyMesh = tile_cache_builder.PolyMesh;

/// A handle to an obstacle: the entry's salt above its 16-bit index. Zero is
/// never a valid reference. A reference outlives its obstacle: when the
/// removal has gone through, the entry's salt changes and `getObstacleByRef`
/// gives null for the reference.
pub const ObstacleRef = u32;

/// A handle to a compressed tile: the entry's salt above its index. Zero is
/// never a valid reference. Removing the tile changes the entry's salt,
/// after which `getTileByRef` gives null for the reference.
pub const CompressedTileRef = u32;

/// `addTile` flag: the cache owns the tile's bytes and frees them, with its
/// own allocator, when the tile is removed or the cache is deinitialised.
pub const compressed_tile_free_data: u8 = 0x01;

/// The most tiles an obstacle is tracked on. An obstacle that covers more
/// is marked into the first ones found only.
pub const touched_tile_count_max = 8;

/// The most tiles looked at on one grid position.
const position_tile_count_max = 32;

/// Where a tile's compressed grids start.
const layer_header_size = common.align4(@sizeOf(LayerHeader));

const invalid_param: Status = .{ .failed = true, .invalid_param = true };
const out_of_memory: Status = .{ .failed = true, .out_of_memory = true };
const buffer_too_small: Status = .{ .failed = true, .buffer_too_small = true };
const wrong_magic: Status = .{ .failed = true, .wrong_magic = true };
const wrong_version: Status = .{ .failed = true, .wrong_version = true };

/// One entry of the cache's tile table.
///
/// A `*const CompressedTile` from a `TileCache` stays at the same address
/// for the cache's life, but its content goes stale: removing the tile
/// clears `header`, empties the slices and changes `salt`, and the entry is
/// reused for a tile added later. Hold a `CompressedTileRef` across changes,
/// not the pointer.
pub const CompressedTile = struct {
    /// Counts the entry's reuses; part of every reference to it.
    salt: u32,
    /// The header at the front of `data`, wherever those bytes sit. Null
    /// while the entry holds no tile.
    header: ?*align(1) const LayerHeader,
    /// The compressed grids: `data` after the header.
    compressed: []u8,
    /// The whole tile, as given to `addTile`.
    data: []u8,
    /// `compressed_tile_free_data`, or zero.
    flags: u32,
    /// The next tile in the same lookup bucket, or the next free entry.
    next: ?*CompressedTile,

    const unused: CompressedTile = .{
        .salt = 1,
        .header = null,
        .compressed = &.{},
        .data = &.{},
        .flags = 0,
        .next = null,
    };
};

/// Where an obstacle entry is in its life.
pub const ObstaclePhase = enum(u8) {
    /// The entry holds no obstacle.
    empty,
    /// Added, with tiles it touches still to be rebuilt.
    processing,
    /// Every tile it touches has been rebuilt with it.
    processed,
    /// Being removed, with tiles it touched still to be rebuilt without it.
    removing,
};

/// The shape of an obstacle.
pub const ObstacleType = enum(u8) {
    cylinder,
    box,
    oriented_box,
};

/// An upright cylinder.
pub const ObstacleCylinder = struct {
    /// The center of the base.
    position: [3]f32,
    radius: f32,
    height: f32,
};

/// An axis-aligned box.
pub const ObstacleBox = struct {
    bounds_min: [3]f32,
    bounds_max: [3]f32,
};

/// A box turned around the y axis.
pub const ObstacleOrientedBox = struct {
    center: [3]f32,
    half_extents: [3]f32,
    /// The turn by angle `a`, as `tile_cache_builder.markOrientedBoxArea`
    /// takes it: `{ cos(a / 2) * sin(-a / 2), cos(a / 2) * cos(a / 2) - 0.5 }`.
    rot_aux: [2]f32,
};

/// One entry of the cache's obstacle table.
///
/// A `*const Obstacle` from a `TileCache` stays at the same address for the
/// cache's life, but its content goes stale: once a removal has gone
/// through, `state` is `empty`, `salt` has changed, and the entry is reused
/// for an obstacle added later. Hold an `ObstacleRef` across changes, not
/// the pointer.
pub const Obstacle = struct {
    shape: Shape,
    /// The tiles the obstacle is marked into, `touched_count` of them.
    touched: [touched_tile_count_max]CompressedTileRef,
    /// The touched tiles still to be rebuilt, `pending_count` of them.
    pending: [touched_tile_count_max]CompressedTileRef,
    /// Counts the entry's reuses; part of every reference to it.
    salt: u16,
    state: ObstaclePhase,
    touched_count: u8,
    pending_count: u8,
    /// The next free entry, while this one is free.
    next: ?*Obstacle,

    /// An obstacle's shape and measures.
    pub const Shape = union(ObstacleType) {
        cylinder: ObstacleCylinder,
        box: ObstacleBox,
        oriented_box: ObstacleOrientedBox,
    };

    const unused: Obstacle = .{
        .shape = .{ .cylinder = .{ .position = .{ 0, 0, 0 }, .radius = 0, .height = 0 } },
        .touched = @splat(0),
        .pending = @splat(0),
        .salt = 1,
        .state = .empty,
        .touched_count = 0,
        .pending_count = 0,
        .next = null,
    };
};

/// The shape of a tile cache, in upstream Detour's layout.
pub const TileCacheParams = extern struct {
    /// The world position of tile (0, 0)'s minimum corner.
    origin: [3]f32,
    /// A cell's size on the ground plane.
    cell_size: f32,
    /// A cell's height.
    cell_height: f32,
    /// A tile's size along x, in cells.
    width: i32,
    /// A tile's size along z, in cells.
    height: i32,
    /// The agent's height, radius and highest step, in world units.
    walkable_height: f32,
    walkable_radius: f32,
    walkable_climb: f32,
    /// How far, in cells, a simplified contour may stray from the raw one.
    max_simplification_error: f32,
    /// How many tiles the cache can hold, every layer counted; bounds the
    /// index bits of a tile reference.
    tile_count_max: i32,
    /// How many obstacles the cache can hold.
    obstacle_count_max: i32,
};

/// A box in the world.
pub const Bounds = struct {
    min: [3]f32,
    max: [3]f32,
};

/// The caller's hook into a tile rebuild, run when the tile's polygons are
/// known and before its navigation mesh bytes are made. It gives the
/// polygons their areas and flags and may add to the parameters, off-mesh
/// connections for one. What `ptr` points to must outlive the cache that
/// holds the hook.
pub const MeshProcess = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    /// The hook's function, taking the hook's own `ptr`.
    pub const VTable = struct {
        process: *const fn (
            ptr: *anyopaque,
            params: *NavMeshCreateParams,
            poly_areas: []u8,
            poly_flags: []u16,
        ) void,
    };

    /// Runs the hook on a rebuilt tile. `poly_areas` and `poly_flags` hold
    /// one entry per polygon and are what `params` points to, so writing
    /// them changes the tile. The slices are gone when the rebuild ends.
    pub fn process(
        self: MeshProcess,
        params: *NavMeshCreateParams,
        poly_areas: []u8,
        poly_flags: []u16,
    ) void {
        self.vtable.process(self.ptr, params, poly_areas, poly_flags);
    }
};

/// Compressed tiles by grid position and layer, the obstacles over them,
/// and the queue of tiles to rebuild.
pub const TileCache = struct {
    /// Frees the bytes of tiles added with `compressed_tile_free_data`, and
    /// allocates the bytes of every navigation mesh tile the cache builds.
    gpa: std.mem.Allocator,
    tile_lookup_size: i32,
    tile_lookup_mask: i32,
    /// Per hash bucket, the first tile in it.
    position_lookup: []?*CompressedTile,
    next_free_tile: ?*CompressedTile,
    tiles: []CompressedTile,
    /// How many bits of a tile reference hold the salt, and how many the
    /// tile index.
    salt_bits: u32,
    tile_bits: u32,
    params: TileCacheParams,
    /// Null until `init`.
    scratch: ?*BuildAllocator,
    compressor: ?Compressor,
    mesh_process: ?MeshProcess,
    obstacles: []Obstacle,
    next_free_obstacle: ?*Obstacle,
    /// The obstacles added and removed since `update` last took them in.
    requests: [request_count_max]Request,
    request_count: i32,
    /// The tiles to rebuild, the next one first.
    update_queue: [update_count_max]CompressedTileRef,
    update_count: i32,

    /// The most obstacle additions and removals that wait for `update`.
    pub const request_count_max = 64;
    /// The most tiles that wait to be rebuilt.
    pub const update_count_max = 64;

    const Request = struct {
        action: Action,
        ref: ObstacleRef,

        const Action = enum(i32) {
            add,
            remove,
        };
    };

    /// A cache with no tables. `init` readies it; until then only `init`,
    /// `deinit` and the lookups, which find nothing, may be called.
    pub const empty: TileCache = .{
        .gpa = .failing,
        .tile_lookup_size = 0,
        .tile_lookup_mask = 0,
        .position_lookup = &.{},
        .next_free_tile = null,
        .tiles = &.{},
        .salt_bits = 0,
        .tile_bits = 0,
        .params = .{
            .origin = .{ 0, 0, 0 },
            .cell_size = 0,
            .cell_height = 0,
            .width = 0,
            .height = 0,
            .walkable_height = 0,
            .walkable_radius = 0,
            .walkable_climb = 0,
            .max_simplification_error = 0,
            .tile_count_max = 0,
            .obstacle_count_max = 0,
        },
        .scratch = null,
        .compressor = null,
        .mesh_process = null,
        .obstacles = &.{},
        .next_free_obstacle = null,
        .requests = @splat(.{ .action = .add, .ref = 0 }),
        .request_count = 0,
        .update_queue = @splat(0),
        .update_count = 0,
    };

    /// Readies an `empty` cache for tiles and obstacles.
    ///
    /// `gpa` must be the allocator the navigation mesh given to `update`,
    /// `buildNavMeshTile` and `buildNavMeshTilesAt` was readied with: the
    /// cache allocates a rebuilt tile's bytes from `gpa` and that mesh frees
    /// them. `scratch`, and what `compressor` and `mesh_process` point to,
    /// must outlive the cache.
    ///
    /// On failure the cache is left as it was. Fails with `out_of_memory`,
    /// and with `invalid_param` when `tile_count_max` is so high that fewer
    /// than 10 bits of a tile reference are left for the salt.
    pub fn init(
        self: *TileCache,
        gpa: std.mem.Allocator,
        params: *const TileCacheParams,
        scratch: *BuildAllocator,
        compressor: Compressor,
        mesh_process: ?MeshProcess,
    ) Status {
        // A negative count asks for more memory than there is.
        const obstacle_count = std.math.cast(usize, params.obstacle_count_max) orelse
            return out_of_memory;
        const tile_count = std.math.cast(usize, params.tile_count_max) orelse
            return out_of_memory;
        var lookup_size = common.nextPow2(@intCast(@divTrunc(params.tile_count_max, 4)));
        if (lookup_size == 0) lookup_size = 1;

        const obstacles = gpa.alloc(Obstacle, obstacle_count) catch return out_of_memory;
        const tiles = gpa.alloc(CompressedTile, tile_count) catch {
            gpa.free(obstacles);
            return out_of_memory;
        };
        const position_lookup = gpa.alloc(?*CompressedTile, lookup_size) catch {
            gpa.free(tiles);
            gpa.free(obstacles);
            return out_of_memory;
        };

        // Running out of memory is reported before a tile count too high
        // for a reference, so the bits are checked after the allocations.
        const tile_bits = common.ilog2(common.nextPow2(@intCast(params.tile_count_max)));
        // Only allow 31 salt bits, since the salt mask is calculated using
        // a 32-bit integer and it would otherwise overflow.
        const salt_bits: u32 = @min(31, 32 - tile_bits);
        if (salt_bits < 10) {
            gpa.free(position_lookup);
            gpa.free(tiles);
            gpa.free(obstacles);
            return invalid_param;
        }

        // Both free lists run from the first entry to the last.
        var next_free_obstacle: ?*Obstacle = null;
        var i = obstacles.len;
        while (i > 0) {
            i -= 1;
            obstacles[i] = .unused;
            obstacles[i].next = next_free_obstacle;
            next_free_obstacle = &obstacles[i];
        }

        @memset(position_lookup, null);
        var next_free_tile: ?*CompressedTile = null;
        i = tiles.len;
        while (i > 0) {
            i -= 1;
            tiles[i] = .unused;
            tiles[i].next = next_free_tile;
            next_free_tile = &tiles[i];
        }

        self.* = .{
            .gpa = gpa,
            .tile_lookup_size = @intCast(lookup_size),
            .tile_lookup_mask = @intCast(lookup_size - 1),
            .position_lookup = position_lookup,
            .next_free_tile = next_free_tile,
            .tiles = tiles,
            .salt_bits = salt_bits,
            .tile_bits = tile_bits,
            .params = params.*,
            .scratch = scratch,
            .compressor = compressor,
            .mesh_process = mesh_process,
            .obstacles = obstacles,
            .next_free_obstacle = next_free_obstacle,
            .requests = @splat(.{ .action = .add, .ref = 0 }),
            .request_count = 0,
            .update_queue = @splat(0),
            .update_count = 0,
        };
        return .success;
    }

    /// Frees the tables and the tile bytes the cache owns. Every tile and
    /// obstacle pointer and every reference into the cache is dead after
    /// this. Tiles the cache built into a navigation mesh stay there.
    pub fn deinit(self: *TileCache) void {
        for (self.tiles) |*tile| {
            if ((tile.flags & compressed_tile_free_data) != 0) self.gpa.free(tile.data);
        }
        self.gpa.free(self.obstacles);
        self.gpa.free(self.position_lookup);
        self.gpa.free(self.tiles);
        self.* = undefined;
    }

    /// The scratch allocator tiles are rebuilt with; null before `init`.
    pub fn getAlloc(self: *const TileCache) ?*BuildAllocator {
        return self.scratch;
    }

    /// The codec tiles are unpacked with; null before `init`.
    pub fn getCompressor(self: *const TileCache) ?Compressor {
        return self.compressor;
    }

    /// The parameters the cache was readied with.
    pub fn getParams(self: *const TileCache) *const TileCacheParams {
        return &self.params;
    }

    /// How many tile entries the cache has, used or not.
    pub fn getTileCount(self: *const TileCache) i32 {
        return self.params.tile_count_max;
    }

    /// The tile entry at an index below `getTileCount`. Its `header` is null
    /// while it is unused. The pointer goes stale as a `CompressedTile`
    /// does.
    pub fn getTile(self: *const TileCache, i: i32) *const CompressedTile {
        return &self.tiles[@intCast(i)];
    }

    /// How many obstacle entries the cache has, used or not.
    pub fn getObstacleCount(self: *const TileCache) i32 {
        return self.params.obstacle_count_max;
    }

    /// The obstacle entry at an index below `getObstacleCount`. Its `state`
    /// is `empty` while it is unused. The pointer goes stale as an
    /// `Obstacle` does.
    pub fn getObstacle(self: *const TileCache, i: i32) *const Obstacle {
        return &self.obstacles[@intCast(i)];
    }

    /// The obstacle entry a reference names, or null when the reference is
    /// zero or stale. An unused entry is returned for a reference that
    /// still carries its salt. The pointer goes stale as an `Obstacle`
    /// does.
    pub fn getObstacleByRef(self: *const TileCache, ref: ObstacleRef) ?*const Obstacle {
        if (ref == 0) return null;
        const index = decodeObstacleIdObstacle(ref);
        if (index >= self.obstacles.len) return null;
        const ob = &self.obstacles[index];
        const salt = decodeObstacleIdSalt(ref);
        if (ob.salt != salt) return null;
        return ob;
    }

    /// The reference of an obstacle entry of this cache; zero for null.
    pub fn getObstacleRef(self: *const TileCache, ob: ?*const Obstacle) ObstacleRef {
        const o = ob orelse return 0;
        const offset = @intFromPtr(o) - @intFromPtr(self.obstacles.ptr);
        return encodeObstacleId(o.salt, @intCast(offset / @sizeOf(Obstacle)));
    }

    /// The first tile of the lookup bucket for a grid position; null on a cache
    /// with no tables.
    inline fn firstTileAt(self: *const TileCache, x: i32, y: i32) ?*CompressedTile {
        if (self.position_lookup.len == 0) return null;
        return self.position_lookup[computeTileHash(x, y, self.tile_lookup_mask)];
    }

    /// Every layer's tile at a grid position, as many as `tiles` holds.
    /// Returns how many were written.
    pub fn getTilesAt(
        self: *const TileCache,
        tile_x: i32,
        tile_y: i32,
        tiles: []CompressedTileRef,
    ) usize {
        var n: usize = 0;

        // Find tile based on hash.
        var tile = self.firstTileAt(tile_x, tile_y);
        while (tile) |t| : (tile = t.next) {
            const header = t.header orelse continue;
            if (header.tile_x == tile_x and header.tile_y == tile_y) {
                if (n < tiles.len) {
                    tiles[n] = self.getTileRef(t);
                    n += 1;
                }
            }
        }

        return n;
    }

    /// The tile at a grid position and layer, or null. The pointer goes
    /// stale as a `CompressedTile` does.
    pub fn getTileAt(
        self: *const TileCache,
        tile_x: i32,
        tile_y: i32,
        tile_layer: i32,
    ) ?*const CompressedTile {
        // Find tile based on hash.
        var tile = self.firstTileAt(tile_x, tile_y);
        while (tile) |t| : (tile = t.next) {
            const header = t.header orelse continue;
            if (header.tile_x == tile_x and
                header.tile_y == tile_y and
                header.tile_layer == tile_layer)
            {
                return t;
            }
        }
        return null;
    }

    /// The reference of a tile entry of this cache; zero for null.
    pub fn getTileRef(self: *const TileCache, tile: ?*const CompressedTile) CompressedTileRef {
        const t = tile orelse return 0;
        const offset = @intFromPtr(t) - @intFromPtr(self.tiles.ptr);
        return self.encodeTileId(t.salt, @intCast(offset / @sizeOf(CompressedTile)));
    }

    /// The tile entry a reference names, or null when the reference is zero
    /// or stale. An unused entry is returned for a reference that still
    /// carries its salt; its `header` is null. The pointer goes stale as a
    /// `CompressedTile` does.
    pub fn getTileByRef(self: *const TileCache, ref: CompressedTileRef) ?*const CompressedTile {
        if (ref == 0) return null;
        const tile_index = self.decodeTileIdTile(ref);
        const tile_salt = self.decodeTileIdSalt(ref);
        if (tile_index >= self.tiles.len) return null;
        const tile = &self.tiles[tile_index];
        if (tile.salt != tile_salt) return null;
        return tile;
    }

    /// Adds a compressed tile, as `tile_cache_builder.buildLayer` packs it,
    /// at the grid position and layer its header names, and writes its
    /// reference to `result`. Nothing is built: `buildNavMeshTile` or
    /// `buildNavMeshTilesAt` makes the navigation mesh tile.
    ///
    /// The cache keeps `data` itself, so the bytes must stay where they are,
    /// unchanged, until the tile is removed. With `compressed_tile_free_data`
    /// in `flags` the cache then frees them, so `data` must be a whole
    /// allocation of the cache's `gpa`; without it the caller frees them
    /// after removing the tile or deinitialising the cache.
    ///
    /// Fails with `invalid_param` when `data` is shorter than a header,
    /// `wrong_magic` or `wrong_version` when the header is not this
    /// format's, plain failure when the position and layer are taken, and
    /// `out_of_memory` when every tile entry is in use.
    pub fn addTile(
        self: *TileCache,
        data: []u8,
        flags: u8,
        result: ?*CompressedTileRef,
    ) Status {
        // Make sure the data is in right format.
        if (data.len < layer_header_size) return invalid_param;
        const header: *align(1) const LayerHeader = @ptrCast(data.ptr);
        if (header.magic != tile_cache_builder.tile_cache_magic) return wrong_magic;
        if (header.version != tile_cache_builder.tile_cache_version) return wrong_version;

        // Make sure the location is free.
        if (self.getTileAt(header.tile_x, header.tile_y, header.tile_layer) != null) {
            return .failure;
        }

        // Allocate a tile.
        const tile = self.next_free_tile orelse return out_of_memory;
        self.next_free_tile = tile.next;

        // Insert tile into the position lookup.
        const h = computeTileHash(header.tile_x, header.tile_y, self.tile_lookup_mask);
        tile.next = self.position_lookup[h];
        self.position_lookup[h] = tile;

        // Init tile.
        tile.header = header;
        tile.data = data;
        tile.compressed = data[layer_header_size..];
        tile.flags = flags;

        if (result) |r| r.* = self.getTileRef(tile);

        return .success;
    }

    /// Removes a tile from the cache and makes its reference stale. The
    /// navigation mesh tile built from it stays in the navigation mesh.
    ///
    /// `data`, if given, gets the tile's bytes when the caller owns them,
    /// and an empty slice when the cache owned and freed them. Fails with
    /// `invalid_param` when the reference names no tile.
    pub fn removeTile(self: *TileCache, ref: CompressedTileRef, data: ?*[]u8) Status {
        if (ref == 0) return invalid_param;
        const tile_index = self.decodeTileIdTile(ref);
        const tile_salt = self.decodeTileIdSalt(ref);
        if (tile_index >= self.tiles.len) return invalid_param;
        const tile = &self.tiles[tile_index];
        if (tile.salt != tile_salt) return invalid_param;
        // A reference to an unused entry, which keeps its salt, names no tile.
        const header = tile.header orelse return invalid_param;

        // Remove tile from hash lookup.
        const h = computeTileHash(header.tile_x, header.tile_y, self.tile_lookup_mask);
        var prev: ?*CompressedTile = null;
        var cur = self.position_lookup[h];
        while (cur) |c| {
            if (c == tile) {
                if (prev) |p| {
                    p.next = c.next;
                } else {
                    self.position_lookup[h] = c.next;
                }
                break;
            }
            prev = c;
            cur = c.next;
        }

        // Reset tile.
        if ((tile.flags & compressed_tile_free_data) != 0) {
            // Owns data.
            self.gpa.free(tile.data);
            if (data) |d| d.* = &.{};
        } else {
            if (data) |d| d.* = tile.data;
        }

        tile.header = null;
        tile.data = &.{};
        tile.compressed = &.{};
        tile.flags = 0;

        // Update salt, salt should never be zero.
        const salt_mask = (@as(u32, 1) << @intCast(self.salt_bits)) - 1;
        tile.salt = (tile.salt +% 1) & salt_mask;
        if (tile.salt == 0) tile.salt += 1;

        // Add to free list.
        tile.next = self.next_free_tile;
        self.next_free_tile = tile;

        return .success;
    }

    /// Queues an upright cylinder, `position` the center of its base, and
    /// writes its reference to `result`. The obstacle takes effect over the
    /// next calls to `update`, one touched tile per call.
    ///
    /// Fails with `buffer_too_small` when `request_count_max` requests wait
    /// for `update`, and `out_of_memory` when every obstacle entry is in
    /// use.
    pub fn addObstacle(
        self: *TileCache,
        position: [3]f32,
        radius: f32,
        height: f32,
        result: ?*ObstacleRef,
    ) Status {
        return self.queueObstacle(.{ .cylinder = .{
            .position = position,
            .radius = radius,
            .height = height,
        } }, result);
    }

    /// Queues an axis-aligned box and writes its reference to `result`.
    /// Takes effect and fails as `addObstacle` does.
    pub fn addBoxObstacle(
        self: *TileCache,
        bounds_min: [3]f32,
        bounds_max: [3]f32,
        result: ?*ObstacleRef,
    ) Status {
        return self.queueObstacle(.{ .box = .{
            .bounds_min = bounds_min,
            .bounds_max = bounds_max,
        } }, result);
    }

    /// Queues a box turned by `y_radians` around the y axis through
    /// `center`, and writes its reference to `result`. Takes effect and
    /// fails as `addObstacle` does.
    pub fn addOrientedBoxObstacle(
        self: *TileCache,
        center: [3]f32,
        half_extents: [3]f32,
        y_radians: f32,
        result: ?*ObstacleRef,
    ) Status {
        const cos_half = @cos(0.5 * y_radians);
        const sin_half = @sin(-0.5 * y_radians);
        return self.queueObstacle(.{ .oriented_box = .{
            .center = center,
            .half_extents = half_extents,
            .rot_aux = .{ cos_half * sin_half, cos_half * cos_half - 0.5 },
        } }, result);
    }

    /// Takes a free obstacle entry for `shape` and queues its addition.
    fn queueObstacle(self: *TileCache, shape: Obstacle.Shape, result: ?*ObstacleRef) Status {
        if (self.request_count >= request_count_max) return buffer_too_small;

        const ob = self.next_free_obstacle orelse return out_of_memory;
        self.next_free_obstacle = ob.next;

        // Everything but the salt starts over.
        const salt = ob.salt;
        ob.* = .{
            .shape = shape,
            .touched = @splat(0),
            .pending = @splat(0),
            .salt = salt,
            .state = .processing,
            .touched_count = 0,
            .pending_count = 0,
            .next = null,
        };

        const ref = self.getObstacleRef(ob);
        self.requests[@intCast(self.request_count)] = .{ .action = .add, .ref = ref };
        self.request_count += 1;

        if (result) |r| r.* = ref;

        return .success;
    }

    /// Queues an obstacle's removal, which goes through over the next calls
    /// to `update`, one touched tile per call; the reference is stale once
    /// the last of them is rebuilt. Zero is allowed and does nothing. A
    /// stale reference is dropped by `update`.
    ///
    /// Fails with `buffer_too_small` when `request_count_max` requests wait
    /// for `update`.
    pub fn removeObstacle(self: *TileCache, ref: ObstacleRef) Status {
        if (ref == 0) return .success;
        if (self.request_count >= request_count_max) return buffer_too_small;

        self.requests[@intCast(self.request_count)] = .{ .action = .remove, .ref = ref };
        self.request_count += 1;

        return .success;
    }

    /// The tiles whose cells overlap a box, touching counted, as many as
    /// `results` holds, by grid row and then column. `result_count` gets
    /// how many were written. Always succeeds.
    pub fn queryTiles(
        self: *const TileCache,
        bounds_min: [3]f32,
        bounds_max: [3]f32,
        results: []CompressedTileRef,
        result_count: *usize,
    ) Status {
        var tiles: [position_tile_count_max]CompressedTileRef = undefined;
        var n: usize = 0;

        const tile_width = @as(f32, @floatFromInt(self.params.width)) * self.params.cell_size;
        const tile_height = @as(f32, @floatFromInt(self.params.height)) * self.params.cell_size;
        const origin = self.params.origin;
        const tx0 = common.truncToInt(i32, @floor((bounds_min[0] - origin[0]) / tile_width));
        const tx1 = common.truncToInt(i32, @floor((bounds_max[0] - origin[0]) / tile_width));
        const ty0 = common.truncToInt(i32, @floor((bounds_min[2] - origin[2]) / tile_height));
        const ty1 = common.truncToInt(i32, @floor((bounds_max[2] - origin[2]) / tile_height));

        var ty = ty0;
        while (ty <= ty1) : (ty += 1) {
            var tx = tx0;
            while (tx <= tx1) : (tx += 1) {
                const tile_count = self.getTilesAt(tx, ty, &tiles);

                for (tiles[0..tile_count]) |ref| {
                    const tile = &self.tiles[self.decodeTileIdTile(ref)];
                    // `getTilesAt` gives only entries that hold a tile.
                    const tile_bounds = self.calcTightTileBounds(tile.header.?);

                    const overlaps = common.overlapBounds(
                        bounds_min,
                        bounds_max,
                        tile_bounds.min,
                        tile_bounds.max,
                    );
                    if (overlaps) {
                        if (n < results.len) {
                            results[n] = ref;
                            n += 1;
                        }
                    }
                }
            }
        }

        result_count.* = n;

        return .success;
    }

    /// Takes in the obstacles added and removed, once no tile waits from an
    /// earlier call, and then rebuilds one waiting tile into `nav`, which
    /// must use the cache's allocator. Call it until `up_to_date` is true
    /// to see every queued change in the navigation mesh.
    ///
    /// `up_to_date`, if given, gets whether no tile and no request waits.
    /// Returns what the tile's rebuild returned, as `buildNavMeshTile`
    /// fails, and success when no tile waited. The tile leaves the queue
    /// either way. `dt` is not used.
    pub fn update(self: *TileCache, dt: f32, nav: *NavMesh, up_to_date: ?*bool) Status {
        _ = dt;

        if (self.update_count == 0) {
            // Process requests.
            for (self.requests[0..@intCast(self.request_count)]) |request| {
                const index = decodeObstacleIdObstacle(request.ref);
                if (index >= self.obstacles.len) continue;
                const ob = &self.obstacles[index];
                const salt = decodeObstacleIdSalt(request.ref);
                if (ob.salt != salt) continue;

                switch (request.action) {
                    .add => {
                        // Find touched tiles.
                        const bounds = getObstacleBounds(ob);
                        var touched_count: usize = 0;
                        _ = self.queryTiles(bounds.min, bounds.max, &ob.touched, &touched_count);
                        ob.touched_count = @intCast(touched_count);
                        // Add tiles to update list.
                        self.queueTouchedTiles(ob);
                    },
                    .remove => {
                        // Prepare to remove obstacle.
                        ob.state = .removing;
                        // Add tiles to update list.
                        self.queueTouchedTiles(ob);
                    },
                }
            }

            self.request_count = 0;
        }

        var status: Status = .success;
        // Process updates.
        if (self.update_count != 0) {
            // Build mesh.
            const ref = self.update_queue[0];
            status = self.buildNavMeshTile(ref, nav);
            self.update_count -= 1;
            if (self.update_count > 0) {
                const left: usize = @intCast(self.update_count);
                @memmove(self.update_queue[0..left], self.update_queue[1 .. left + 1]);
            }

            // Update obstacle states.
            for (self.obstacles) |*ob| {
                if (ob.state != .processing and ob.state != .removing) continue;

                // Remove handled tile from pending list.
                for (ob.pending[0..ob.pending_count], 0..) |pending, j| {
                    if (pending == ref) {
                        ob.pending[j] = ob.pending[ob.pending_count - 1];
                        ob.pending_count -= 1;
                        break;
                    }
                }

                // If all pending tiles processed, change state.
                if (ob.pending_count == 0) {
                    if (ob.state == .processing) {
                        ob.state = .processed;
                    } else {
                        ob.state = .empty;
                        // Update salt, salt should never be zero.
                        ob.salt +%= 1;
                        if (ob.salt == 0) ob.salt += 1;
                        // Return obstacle to free list.
                        ob.next = self.next_free_obstacle;
                        self.next_free_obstacle = ob;
                    }
                }
            }
        }

        if (up_to_date) |u| u.* = self.update_count == 0 and self.request_count == 0;

        return status;
    }

    /// Makes every tile `ob` touches wait for the obstacle, and puts those
    /// not yet in the update queue at its end. A tile the full queue has no
    /// room for is left out of both, so it is not rebuilt for this change.
    fn queueTouchedTiles(self: *TileCache, ob: *Obstacle) void {
        ob.pending_count = 0;
        for (ob.touched[0..ob.touched_count]) |ref| {
            if (self.update_count >= update_count_max) continue;

            const queued = self.update_queue[0..@intCast(self.update_count)];
            if (std.mem.findScalar(CompressedTileRef, queued, ref) == null) {
                self.update_queue[@intCast(self.update_count)] = ref;
                self.update_count += 1;
            }
            ob.pending[ob.pending_count] = ref;
            ob.pending_count += 1;
        }
    }

    /// Rebuilds every layer's tile at a grid position into `nav`, as
    /// `buildNavMeshTile` does, the first 32 layers found. Stops at the
    /// first tile that fails and returns its status.
    pub fn buildNavMeshTilesAt(self: *TileCache, tile_x: i32, tile_y: i32, nav: *NavMesh) Status {
        var tiles: [position_tile_count_max]CompressedTileRef = undefined;
        const tile_count = self.getTilesAt(tile_x, tile_y, &tiles);

        for (tiles[0..tile_count]) |ref| {
            const status = self.buildNavMeshTile(ref, nav);
            if (status.failed) return status;
        }

        return .success;
    }

    /// Builds one compressed tile into `nav`: unpacks its layer, marks into
    /// it every obstacle on the tile that is not being removed, builds the
    /// polygons, runs the mesh process, and puts the result in place of the
    /// navigation mesh tile at the same position and layer. A tile left
    /// with no polygons only removes that navigation mesh tile.
    ///
    /// `nav` must have been readied with the cache's allocator: the new
    /// tile's bytes come from the cache's `gpa`, and `nav` owns and frees
    /// them. The working memory comes from the scratch allocator, which is
    /// reset first and holds none of it afterwards. Removing the old tile
    /// makes every reference into it stale.
    ///
    /// Fails with `invalid_param` when the reference names no tile, with
    /// what unpacking or a build step fails with, with plain failure when
    /// the navigation mesh bytes cannot be made, and with what
    /// `NavMesh.addTile` fails with; in the last two cases the old
    /// navigation mesh tile may already be gone.
    pub fn buildNavMeshTile(self: *TileCache, ref: CompressedTileRef, nav: *NavMesh) Status {
        std.debug.assert(self.scratch != null);
        std.debug.assert(self.compressor != null);

        const tile_index = self.decodeTileIdTile(ref);
        if (tile_index >= self.tiles.len) return invalid_param;
        const tile = &self.tiles[tile_index];
        const tile_salt = self.decodeTileIdSalt(ref);
        if (tile.salt != tile_salt) return invalid_param;

        const build_allocator = self.scratch.?;
        build_allocator.reset();
        const scratch = build_allocator.allocator;

        // Whatever the build has made is freed on every way out.
        var owned_layer: ?*Layer = null;
        var owned_contour_set: ?*ContourSet = null;
        var owned_mesh: ?*PolyMesh = null;
        defer {
            tile_cache_builder.freeLayer(scratch, owned_layer);
            tile_cache_builder.freeContourSet(scratch, owned_contour_set);
            tile_cache_builder.freePolyMesh(scratch, owned_mesh);
        }

        const cell_size = self.params.cell_size;
        const cell_height = self.params.cell_height;
        const walkable_climb = common.truncToInt(i32, self.params.walkable_climb / cell_height);

        // Decompress tile layer data. An unused entry has no bytes, which
        // fails here.
        var status = tile_cache_builder.decompressLayer(
            scratch,
            self.compressor.?,
            tile.data,
            &owned_layer,
        );
        if (status.failed) return status;
        const layer = owned_layer.?;
        // An entry with bytes has its header: `addTile` sets both.
        const header = tile.header.?;

        // Rasterize obstacles.
        for (self.obstacles) |*ob| {
            if (ob.state == .empty or ob.state == .removing) continue;
            const touched = ob.touched[0..ob.touched_count];
            if (std.mem.findScalar(CompressedTileRef, touched, ref) == null) continue;

            const origin = header.bounds_min;
            _ = switch (ob.shape) {
                .cylinder => |cylinder| tile_cache_builder.markCylinderArea(
                    layer,
                    origin,
                    cell_size,
                    cell_height,
                    cylinder.position,
                    cylinder.radius,
                    cylinder.height,
                    0,
                ),
                .box => |box| tile_cache_builder.markBoxArea(
                    layer,
                    origin,
                    cell_size,
                    cell_height,
                    box.bounds_min,
                    box.bounds_max,
                    0,
                ),
                .oriented_box => |box| tile_cache_builder.markOrientedBoxArea(
                    layer,
                    origin,
                    cell_size,
                    cell_height,
                    box.center,
                    box.half_extents,
                    box.rot_aux,
                    0,
                ),
            };
        }

        // Build navmesh.
        status = tile_cache_builder.buildRegions(scratch, layer, walkable_climb);
        if (status.failed) return status;

        owned_contour_set = tile_cache_builder.allocContourSet(scratch);
        const contour_set = owned_contour_set orelse return out_of_memory;
        status = tile_cache_builder.buildContours(
            scratch,
            layer,
            walkable_climb,
            self.params.max_simplification_error,
            contour_set,
        );
        if (status.failed) return status;

        owned_mesh = tile_cache_builder.allocPolyMesh(scratch);
        const mesh = owned_mesh orelse return out_of_memory;
        status = tile_cache_builder.buildPolyMesh(scratch, contour_set, mesh);
        if (status.failed) return status;

        // Early out if the mesh tile is empty.
        if (mesh.polygon_count == 0) {
            // Remove existing tile.
            const old = nav.getTileRefAt(header.tile_x, header.tile_y, header.tile_layer);
            _ = nav.removeTile(old, null);
            return .success;
        }

        // The mesh's arrays are longer than the counts: the build sizes
        // them for the most the contours can give.
        const vertex_count: usize = @intCast(mesh.vertex_count);
        const polygon_count: usize = @intCast(mesh.polygon_count);
        const poly_areas = mesh.areas[0..polygon_count];
        const poly_flags = mesh.flags[0..polygon_count];

        var create_params: NavMeshCreateParams = .{
            .verts = @ptrCast(mesh.vertices[0 .. vertex_count * 3]),
            .vert_count = mesh.vertex_count,
            .polys = mesh.polygons[0 .. polygon_count * nav_mesh.verts_per_polygon * 2],
            .poly_flags = poly_flags,
            .poly_areas = poly_areas,
            .poly_count = mesh.polygon_count,
            .vertices_per_polygon = nav_mesh.verts_per_polygon,
            .detail_meshes = null,
            .detail_verts = &.{},
            .detail_tris = &.{},
            .off_mesh_con_verts = &.{},
            .off_mesh_con_rad = &.{},
            .off_mesh_con_flags = &.{},
            .off_mesh_con_areas = &.{},
            .off_mesh_con_dir = &.{},
            .off_mesh_con_user_id = null,
            .off_mesh_con_count = 0,
            .user_id = 0,
            .tile_x = header.tile_x,
            .tile_y = header.tile_y,
            .tile_layer = header.tile_layer,
            .bounds_min = header.bounds_min,
            .bounds_max = header.bounds_max,
            .walkable_height = self.params.walkable_height,
            .walkable_radius = self.params.walkable_radius,
            .walkable_climb = self.params.walkable_climb,
            .cell_size = cell_size,
            .cell_height = cell_height,
            .build_bv_tree = false,
        };

        if (self.mesh_process) |mesh_process| {
            mesh_process.process(&create_params, poly_areas, poly_flags);
        }

        const nav_data = nav_mesh_builder.createNavMeshData(self.gpa, &create_params) catch
            return .failure;

        // Remove existing tile.
        const old = nav.getTileRefAt(header.tile_x, header.tile_y, header.tile_layer);
        _ = nav.removeTile(old, null);

        // Add the new tile. The navigation mesh frees its bytes from here
        // on, unless it does not take them.
        status = nav.addTile(nav_data, .{ .free_data = true }, 0, null);
        if (status.failed) {
            self.gpa.free(nav_data);
            return status;
        }

        return .success;
    }

    /// The box around the cells a tile's layer holds: the header's bounds,
    /// drawn in on the ground plane to the part of the grid in use.
    pub fn calcTightTileBounds(
        self: *const TileCache,
        header: *align(1) const LayerHeader,
    ) Bounds {
        const cell_size = self.params.cell_size;
        const x_min: f32 = @floatFromInt(header.x_min);
        const y_min: f32 = @floatFromInt(header.y_min);
        const x_end: f32 = @floatFromInt(@as(i32, header.x_max) + 1);
        const y_end: f32 = @floatFromInt(@as(i32, header.y_max) + 1);
        return .{
            .min = .{
                header.bounds_min[0] + x_min * cell_size,
                header.bounds_min[1],
                header.bounds_min[2] + y_min * cell_size,
            },
            .max = .{
                header.bounds_min[0] + x_end * cell_size,
                header.bounds_max[1],
                header.bounds_min[2] + y_end * cell_size,
            },
        };
    }

    /// A box that holds an obstacle. An oriented box gets room for any
    /// turn: 1.41 times its longer half extent to each side on the ground
    /// plane.
    pub fn getObstacleBounds(ob: *const Obstacle) Bounds {
        switch (ob.shape) {
            .cylinder => |cylinder| return .{
                .min = .{
                    cylinder.position[0] - cylinder.radius,
                    cylinder.position[1],
                    cylinder.position[2] - cylinder.radius,
                },
                .max = .{
                    cylinder.position[0] + cylinder.radius,
                    cylinder.position[1] + cylinder.height,
                    cylinder.position[2] + cylinder.radius,
                },
            },
            .box => |box| return .{
                .min = box.bounds_min,
                .max = box.bounds_max,
            },
            .oriented_box => |box| {
                const reach = 1.41 * common.max(f32, box.half_extents[0], box.half_extents[2]);
                return .{
                    .min = .{
                        box.center[0] - reach,
                        box.center[1] - box.half_extents[1],
                        box.center[2] - reach,
                    },
                    .max = .{
                        box.center[0] + reach,
                        box.center[1] + box.half_extents[1],
                        box.center[2] + reach,
                    },
                };
            },
        }
    }

    /// A tile reference from its parts. A tile index wider than the cache's
    /// tile bits spills into the salt.
    pub inline fn encodeTileId(
        self: *const TileCache,
        salt: u32,
        tile_index: u32,
    ) CompressedTileRef {
        return (salt << @intCast(self.tile_bits)) | tile_index;
    }

    /// The salt part of a tile reference.
    pub inline fn decodeTileIdSalt(self: *const TileCache, ref: CompressedTileRef) u32 {
        const salt_mask = (@as(u32, 1) << @intCast(self.salt_bits)) - 1;
        return (ref >> @intCast(self.tile_bits)) & salt_mask;
    }

    /// The tile index part of a tile reference.
    pub inline fn decodeTileIdTile(self: *const TileCache, ref: CompressedTileRef) u32 {
        const tile_mask = (@as(u32, 1) << @intCast(self.tile_bits)) - 1;
        return ref & tile_mask;
    }

    /// An obstacle reference from its parts: the salt above 16 bits of
    /// index. An index wider than that spills into the salt.
    pub inline fn encodeObstacleId(salt: u32, obstacle_index: u32) ObstacleRef {
        return (salt << 16) | obstacle_index;
    }

    /// The salt part of an obstacle reference.
    pub inline fn decodeObstacleIdSalt(ref: ObstacleRef) u32 {
        return (ref >> 16) & 0xffff;
    }

    /// The obstacle index part of an obstacle reference.
    pub inline fn decodeObstacleIdObstacle(ref: ObstacleRef) u32 {
        return ref & 0xffff;
    }
};

/// The lookup bucket of a grid position.
inline fn computeTileHash(x: i32, y: i32, mask: i32) usize {
    // Large multiplicative constants, arbitrarily chosen primes.
    const h1: u32 = 0x8da6b343;
    const h2: u32 = 0xd8163841;
    const n = h1 *% @as(u32, @bitCast(x)) +% h2 *% @as(u32, @bitCast(y));
    return n & @as(u32, @bitCast(mask));
}
