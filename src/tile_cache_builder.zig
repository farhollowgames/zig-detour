//! The tile cache's layer builder: compressed heightfield layers, the area
//! marks obstacles stamp into a layer, and the steps that turn a layer into
//! a polygon mesh (regions, then contours, then polygons).
//!
//! A layer is a grid of at most 255 by 255 cells, so the grid math is in
//! bytes and small integers. Every step keeps upstream Detour's order, so
//! regions, contours, meshes and compressed bytes match it exactly.

const std = @import("std");
const common = @import("common.zig");
const Status = @import("status.zig").Status;

/// The number that opens a compressed layer: the characters "DTLR".
pub const tile_cache_magic: i32 = 'D' << 24 | 'T' << 16 | 'L' << 8 | 'R';
/// The layer format this builder reads and writes.
pub const tile_cache_version: i32 = 1;

/// The area of a cell nothing can walk on.
pub const null_area: u8 = 0;
/// The area a walkable cell has until something marks it otherwise.
pub const walkable_area: u8 = 63;
/// The polygon mesh's "no vertex" and "no neighbor" entry.
pub const null_index: u16 = 0xffff;

const vertices_per_polygon_max = 6;
const removed_edge_count_max = 48;
const layer_neighbor_count_max = 16;
const vertex_bucket_count = 1 << 8;

const out_of_memory: Status = .{ .failed = true, .out_of_memory = true };
const buffer_too_small: Status = .{ .failed = true, .buffer_too_small = true };
const invalid_param: Status = .{ .failed = true, .invalid_param = true };
const wrong_magic: Status = .{ .failed = true, .wrong_magic = true };
const wrong_version: Status = .{ .failed = true, .wrong_version = true };

/// What opens a compressed layer, in upstream's byte layout: 54 bytes of
/// fields and two of padding.
pub const LayerHeader = extern struct {
    magic: i32,
    version: i32,
    tile_x: i32,
    tile_y: i32,
    tile_layer: i32,
    bounds_min: [3]f32,
    bounds_max: [3]f32,
    /// The lowest and highest cell height in the layer.
    height_min: u16,
    height_max: u16,
    /// The layer's size in cells.
    width: u8,
    height: u8,
    /// The part of the grid that holds cells, inclusive.
    x_min: u8,
    x_max: u8,
    y_min: u8,
    y_max: u8,
};

const layer_header_size = common.align4(@sizeOf(LayerHeader));
const layer_size = common.align4(@sizeOf(Layer));

/// A decompressed layer: a header and four grids of `width * height` cells,
/// each row after the one before.
///
/// `decompressLayer` makes one in a single block that also holds the header
/// and the grids, and `freeLayer` frees that block: every pointer and slice
/// here dies with it. A layer put together by hand borrows its header and
/// grids from the caller and is never passed to `freeLayer`.
pub const Layer = struct {
    header: *LayerHeader,
    /// How many regions `buildRegions` found.
    region_count: u8 = 0,
    heights: []u8,
    areas: []u8,
    /// Per cell, the directions it connects in (low four bits) and the
    /// directions it has a portal to another tile in (high four bits).
    connections: []u8,
    /// Per cell, the region `buildRegions` gave it, 0xff for none.
    regions: []u8,
};

/// One region's outline. `buildContours` allocates `vertices` from its
/// `scratch`, and `freeContourSet` frees it with the set.
pub const Contour = struct {
    vertex_count: i32 = 0,
    /// Four bytes per vertex: x, height, z, then the portal direction of
    /// the edge that starts there (0xf for none) with 0x80 set when the
    /// vertex can be removed from the mesh. `vertex_count * 4` long.
    vertices: []u8 = &.{},
    region: u8 = 0,
    area: u8 = 0,
};

/// The outlines of a layer's regions, indexed by region. `allocContourSet`
/// makes one, `buildContours` fills it from the same `scratch`, and
/// `freeContourSet` frees the set, `contours` and every contour's vertices.
pub const ContourSet = struct {
    contour_count: i32 = 0,
    contours: []Contour = &.{},
};

/// A layer's polygons. `allocPolyMesh` makes one, `buildPolyMesh` allocates
/// the four arrays from the same `scratch`, sized for the most the contours
/// can give, and `freePolyMesh` frees them with the mesh. The counts say how
/// much of each array is in use.
pub const PolyMesh = struct {
    vertices_per_polygon: i32 = 0,
    vertex_count: i32 = 0,
    polygon_count: i32 = 0,
    /// Three entries per vertex: x, height, z, in cells.
    vertices: []u16 = &.{},
    /// `vertices_per_polygon * 2` entries per polygon: its vertices, padded
    /// with `null_index`, then per edge the polygon across it, `null_index`
    /// for a wall, or 0x8000 plus a direction for a portal.
    polygons: []u16 = &.{},
    /// Per polygon, flags the builder leaves zero for the caller to set.
    flags: []u16 = &.{},
    /// Per polygon, its area.
    areas: []u8 = &.{},
};

/// The allocator a tile build takes its working memory from, with a hook
/// the tile cache runs before each tile build, so a linear allocator can
/// start over. The builder's functions take `allocator` as their `scratch`.
pub const BuildAllocator = struct {
    allocator: std.mem.Allocator,
    ptr: ?*anyopaque = null,
    resetFn: ?*const fn (ptr: ?*anyopaque) void = null,

    /// Runs the hook, if one is set. Whatever an earlier build allocated
    /// from `allocator` may be gone afterwards.
    pub fn reset(self: BuildAllocator) void {
        if (self.resetFn) |resetFn| resetFn(self.ptr);
    }
};

/// The codec a layer's grids are stored with, chosen by the caller.
pub const Compressor = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    /// The codec's three functions, each taking the codec's own `ptr`.
    pub const VTable = struct {
        maxCompressedSize: *const fn (ptr: *anyopaque, buffer_size: i32) i32,
        compress: *const fn (
            ptr: *anyopaque,
            buffer: []const u8,
            compressed: []u8,
            compressed_size: *i32,
        ) Status,
        decompress: *const fn (
            ptr: *anyopaque,
            compressed: []const u8,
            buffer: []u8,
            buffer_size: *i32,
        ) Status,
    };

    /// The most bytes `compress` can write for `buffer_size` bytes of input.
    pub fn maxCompressedSize(self: Compressor, buffer_size: i32) i32 {
        return self.vtable.maxCompressedSize(self.ptr, buffer_size);
    }

    /// Compresses `buffer` into `compressed`, which holds at least
    /// `maxCompressedSize(buffer.len)` bytes, and writes the bytes used to
    /// `compressed_size`.
    pub fn compress(
        self: Compressor,
        buffer: []const u8,
        compressed: []u8,
        compressed_size: *i32,
    ) Status {
        return self.vtable.compress(self.ptr, buffer, compressed, compressed_size);
    }

    /// Decompresses `compressed` into `buffer` and writes the bytes produced
    /// to `buffer_size`.
    pub fn decompress(
        self: Compressor,
        compressed: []const u8,
        buffer: []u8,
        buffer_size: *i32,
    ) Status {
        return self.vtable.decompress(self.ptr, compressed, buffer, buffer_size);
    }
};

fn dirOffsetX(dir: i32) i32 {
    const offset = [4]i32{ -1, 0, 1, 0 };
    return offset[@intCast(dir & 0x03)];
}

fn dirOffsetY(dir: i32) i32 {
    const offset = [4]i32{ 0, 1, 0, -1 };
    return offset[@intCast(dir & 0x03)];
}

/// The low byte of `v`, as C's cast to `unsigned char` gives it.
fn toByte(v: i32) u8 {
    return @truncate(@as(u32, @bitCast(v)));
}

/// A new, empty contour set from `scratch`, or null when it is out of
/// memory. `freeContourSet` with the same `scratch` frees it.
pub fn allocContourSet(scratch: std.mem.Allocator) ?*ContourSet {
    const contour_set = scratch.create(ContourSet) catch return null;
    contour_set.* = .{};
    return contour_set;
}

/// Frees a contour set, its contours and their vertices, all of which came
/// from `scratch`. Null is allowed and does nothing.
pub fn freeContourSet(scratch: std.mem.Allocator, contour_set: ?*ContourSet) void {
    const set = contour_set orelse return;
    for (set.contours) |contour| scratch.free(contour.vertices);
    scratch.free(set.contours);
    scratch.destroy(set);
}

/// A new, empty polygon mesh from `scratch`, or null when it is out of
/// memory. `freePolyMesh` with the same `scratch` frees it.
pub fn allocPolyMesh(scratch: std.mem.Allocator) ?*PolyMesh {
    const mesh = scratch.create(PolyMesh) catch return null;
    mesh.* = .{};
    return mesh;
}

/// Frees a polygon mesh and its four arrays, all of which came from
/// `scratch`. Null is allowed and does nothing.
pub fn freePolyMesh(scratch: std.mem.Allocator, mesh: ?*PolyMesh) void {
    const m = mesh orelse return;
    scratch.free(m.vertices);
    scratch.free(m.polygons);
    scratch.free(m.flags);
    scratch.free(m.areas);
    scratch.destroy(m);
}

const SweepSpan = struct {
    /// How many of the sweep's cells connect to `neighbor`.
    sample_count: u16,
    /// The region the sweep becomes.
    id: u8,
    /// The one region the sweep touches on the row before, 0xff when it
    /// touches none or several.
    neighbor: u8,
};

const MonotoneRegion = struct {
    /// The region's size in cells.
    area: i32,
    neighbors: [layer_neighbor_count_max]u8,
    neighbor_count: u8,
    region_id: u8,
    area_id: u8,
};

/// A contour being traced: the raw outline in `vertices`, four bytes per
/// vertex, and the vertices that survive simplification in `poly`.
const TempContour = struct {
    vertices: []u8,
    vertex_count: i32 = 0,
    vertex_capacity: i32,
    poly: []u16,
    poly_count: i32 = 0,
};

fn overlapRangeExclusive(a_min: u16, a_max: u16, b_min: u16, b_max: u16) bool {
    return if (a_min >= b_max or a_max <= b_min) false else true;
}

/// Adds `v` to the region's neighbors unless it is the one added last.
/// False when the list is full.
fn addUniqueLast(region: *MonotoneRegion, v: u8) bool {
    const n = region.neighbor_count;
    if (n > 0 and region.neighbors[n - 1] == v) return true;
    if (n >= layer_neighbor_count_max) return false;
    region.neighbors[n] = v;
    region.neighbor_count += 1;
    return true;
}

fn isConnected(layer: *const Layer, ia: usize, ib: usize, walkable_climb: i32) bool {
    if (layer.areas[ia] != layer.areas[ib]) return false;
    const height_a: i32 = layer.heights[ia];
    const height_b: i32 = layer.heights[ib];
    if (common.abs(i32, height_a - height_b) > walkable_climb) return false;
    return true;
}

/// Whether the regions sharing `old_region_id` touch those sharing
/// `new_region_id` through exactly one neighbor link, so merging them
/// cannot close a ring around a hole.
fn canMerge(old_region_id: u8, new_region_id: u8, regions: []const MonotoneRegion) bool {
    var count: i32 = 0;
    for (regions) |*region| {
        if (region.region_id != old_region_id) continue;
        for (region.neighbors[0..region.neighbor_count]) |neighbor| {
            if (regions[neighbor].region_id == new_region_id) count += 1;
        }
    }
    return count == 1;
}

/// Splits the layer's walkable cells into regions without holes, writing
/// each cell's region to `layer.regions` and the number of regions to
/// `layer.region_count`. Cells connect when they share an area and their
/// heights differ by at most `walkable_climb`.
///
/// Fails with `buffer_too_small` when the layer needs more than 255 regions,
/// or when one region touches more than 16 others on the rows around it.
/// The working memory comes from `scratch` and is freed before returning.
pub fn buildRegions(scratch: std.mem.Allocator, layer: *Layer, walkable_climb: i32) Status {
    const w: i32 = layer.header.width;
    const h: i32 = layer.header.height;
    const width: usize = layer.header.width;
    const cell_count: usize = @intCast(w * h);

    @memset(layer.regions[0..cell_count], 0xff);

    const sweeps = scratch.alloc(SweepSpan, width) catch return out_of_memory;
    defer scratch.free(sweeps);
    @memset(sweeps, .{ .sample_count = 0, .id = 0, .neighbor = 0 });

    // Partition the walkable area into monotone regions. Only the entries of
    // `prev_count` below `region_id` are ever read, and each row clears those.
    var prev_count: [256]u8 = undefined;
    var region_id: u8 = 0;

    var y: i32 = 0;
    while (y < h) : (y += 1) {
        if (region_id > 0) @memset(prev_count[0..region_id], 0);
        var sweep_id: u8 = 0;

        var x: i32 = 0;
        while (x < w) : (x += 1) {
            const idx: usize = @intCast(x + y * w);
            if (layer.areas[idx] == null_area) continue;

            var sid: u8 = 0xff;

            // -x
            if (x > 0 and isConnected(layer, idx, idx - 1, walkable_climb)) {
                if (layer.regions[idx - 1] != 0xff) sid = layer.regions[idx - 1];
            }

            if (sid == 0xff) {
                sid = sweep_id;
                sweep_id +%= 1;
                sweeps[sid].neighbor = 0xff;
                sweeps[sid].sample_count = 0;
            }

            // -y
            if (y > 0 and isConnected(layer, idx, idx - width, walkable_climb)) {
                const nr = layer.regions[idx - width];
                if (nr != 0xff) {
                    // Set the neighbor when the first valid one is met.
                    if (sweeps[sid].sample_count == 0) sweeps[sid].neighbor = nr;

                    if (sweeps[sid].neighbor == nr) {
                        // Update the existing neighbor.
                        sweeps[sid].sample_count +%= 1;
                        prev_count[nr] +%= 1;
                    } else {
                        // More than one neighbor: invalidate it.
                        sweeps[sid].neighbor = 0xff;
                    }
                }
            }

            layer.regions[idx] = sid;
        }

        // Create unique ids.
        for (sweeps[0..sweep_id]) |*sweep| {
            // A sweep whose one neighbor connects to nothing else merges
            // into it; any other sweep starts a new region.
            if (sweep.neighbor != 0xff and
                @as(u16, prev_count[sweep.neighbor]) == sweep.sample_count)
            {
                sweep.id = sweep.neighbor;
            } else {
                // The region ids have run out.
                if (region_id == 255) return buffer_too_small;
                sweep.id = region_id;
                region_id += 1;
            }
        }

        // Remap local sweep ids to region ids.
        const row_start: usize = @intCast(y * w);
        for (layer.regions[row_start..][0..width]) |*region| {
            if (region.* != 0xff) region.* = sweeps[region.*].id;
        }
    }

    // Allocate and init the layer's regions.
    const regions = scratch.alloc(MonotoneRegion, region_id) catch return out_of_memory;
    defer scratch.free(regions);
    @memset(regions, .{
        .area = 0,
        .neighbors = @splat(0),
        .neighbor_count = 0,
        .region_id = 0xff,
        .area_id = 0,
    });

    // Find region neighbors.
    y = 0;
    while (y < h) : (y += 1) {
        var x: i32 = 0;
        while (x < w) : (x += 1) {
            const idx: usize = @intCast(x + y * w);
            const ri = layer.regions[idx];
            if (ri == 0xff) continue;

            // Update area.
            regions[ri].area += 1;
            regions[ri].area_id = layer.areas[idx];

            // Update neighbors.
            if (y > 0 and isConnected(layer, idx, idx - width, walkable_climb)) {
                const rai = layer.regions[idx - width];
                if (rai != 0xff and rai != ri) {
                    if (!addUniqueLast(&regions[ri], rai)) return buffer_too_small;
                    if (!addUniqueLast(&regions[rai], ri)) return buffer_too_small;
                }
            }
        }
    }

    for (regions, 0..) |*region, i| region.region_id = @intCast(i);

    for (regions) |*region| {
        var merge: i32 = -1;
        var merge_area: i32 = 0;
        for (region.neighbors[0..region.neighbor_count]) |neighbor| {
            const other = &regions[neighbor];
            if (region.region_id == other.region_id) continue;
            if (region.area_id != other.area_id) continue;
            if (other.area > merge_area) {
                if (canMerge(region.region_id, other.region_id, regions)) {
                    merge_area = other.area;
                    merge = neighbor;
                }
            }
        }
        if (merge != -1) {
            const old_id = region.region_id;
            const new_id = regions[@intCast(merge)].region_id;
            for (regions) |*merged| {
                if (merged.region_id == old_id) merged.region_id = new_id;
            }
        }
    }

    // Compact ids: find the unique regions, then remap.
    var remap: [256]u8 = @splat(0);
    region_id = 0;
    for (regions) |region| remap[region.region_id] = 1;
    for (&remap) |*entry| {
        if (entry.* != 0) {
            entry.* = region_id;
            region_id +%= 1;
        }
    }
    for (regions) |*region| region.region_id = remap[region.region_id];

    layer.region_count = region_id;

    for (layer.regions[0..cell_count]) |*region| {
        if (region.* != 0xff) region.* = regions[region.*].region_id;
    }

    return .success;
}

/// Adds a vertex to the raw outline, or moves the last one when the new
/// vertex only extends its straight edge. False when the outline is full.
fn appendVertex(cont: *TempContour, x: i32, y: i32, z: i32, r: i32) bool {
    // Try to merge with existing segments.
    if (cont.vertex_count > 1) {
        const count: usize = @intCast(cont.vertex_count);
        const pa = cont.vertices[(count - 2) * 4 ..][0..4];
        const pb = cont.vertices[(count - 1) * 4 ..][0..4];
        if (pb[3] == r) {
            if (pa[0] == pb[0] and pb[0] == x) {
                // The vertices are aligned along the x axis: update z.
                pb[1] = toByte(y);
                pb[2] = toByte(z);
                return true;
            } else if (pa[2] == pb[2] and pb[2] == z) {
                // The vertices are aligned along the z axis: update x.
                pb[0] = toByte(x);
                pb[1] = toByte(y);
                return true;
            }
        }
    }

    // Add a new point.
    if (cont.vertex_count + 1 > cont.vertex_capacity) return false;

    const v = cont.vertices[@as(usize, @intCast(cont.vertex_count)) * 4 ..][0..4];
    v.* = .{ toByte(x), toByte(y), toByte(z), toByte(r) };
    cont.vertex_count += 1;

    return true;
}

/// The region across the cell's edge in direction `dir`: 0xff for a hard
/// edge, 0xf8 plus the direction for a portal.
fn neighborRegion(layer: *const Layer, ax: i32, ay: i32, dir: i32) u8 {
    const w: i32 = layer.header.width;
    const ia: usize = @intCast(ax + ay * w);

    const con = layer.connections[ia] & 0xf;
    const portal = layer.connections[ia] >> 4;
    const mask = @as(u8, 1) << @intCast(dir);

    if ((con & mask) == 0) {
        // No connection: a portal or a hard edge.
        if ((portal & mask) != 0) return 0xf8 +% toByte(dir);
        return 0xff;
    }

    const bx = ax + dirOffsetX(dir);
    const by = ay + dirOffsetY(dir);
    const ib: usize = @intCast(bx + by * w);

    return layer.regions[ib];
}

/// Traces the outline of the region holding cell `start_x`, `start_y` into
/// `cont`, keeping the region on its right. False when the outline has more
/// vertices than `cont` holds.
fn walkContour(layer: *const Layer, start_x: i32, start_y: i32, cont: *TempContour) bool {
    const w: i32 = layer.header.width;
    const h: i32 = layer.header.height;

    cont.vertex_count = 0;

    var x = start_x;
    var y = start_y;
    var start_dir: i32 = -1;

    var i: i32 = 0;
    while (i < 4) : (i += 1) {
        const dir = (i + 3) & 3;
        const rn = neighborRegion(layer, x, y, dir);
        if (rn != layer.regions[@intCast(x + y * w)]) {
            start_dir = dir;
            break;
        }
    }
    if (start_dir == -1) return true;

    var dir = start_dir;
    const iter_max = w * h;

    var iter: i32 = 0;
    while (iter < iter_max) {
        const rn = neighborRegion(layer, x, y, dir);

        var nx = x;
        var ny = y;
        var ndir = dir;

        if (rn != layer.regions[@intCast(x + y * w)]) {
            // Solid edge.
            var px = x;
            var pz = y;
            switch (dir) {
                0 => pz += 1,
                1 => {
                    px += 1;
                    pz += 1;
                },
                2 => px += 1,
                else => {},
            }

            // Try to merge with the previous vertex.
            const height: i32 = layer.heights[@intCast(x + y * w)];
            if (!appendVertex(cont, px, height, pz, rn)) return false;

            ndir = (dir + 1) & 0x3; // Rotate clockwise.
        } else {
            // Move to the next cell.
            nx = x + dirOffsetX(dir);
            ny = y + dirOffsetY(dir);
            ndir = (dir + 3) & 0x3; // Rotate counter-clockwise.
        }

        if (iter > 0 and x == start_x and y == start_y and dir == start_dir) break;

        x = nx;
        y = ny;
        dir = ndir;

        iter += 1;
    }

    // Remove the last vertex if it duplicates the first one.
    const pa = cont.vertices[@as(usize, @intCast(cont.vertex_count - 1)) * 4 ..][0..4];
    const pb = cont.vertices[0..4];
    if (pa[0] == pb[0] and pa[2] == pb[2]) cont.vertex_count -= 1;

    return true;
}

/// The squared distance from point `x`, `z` to the segment `p`..`q`.
fn distancePointSegment(x: i32, z: i32, px: i32, pz: i32, qx: i32, qz: i32) f32 {
    const pqx: f32 = @floatFromInt(qx - px);
    const pqz: f32 = @floatFromInt(qz - pz);
    var dx: f32 = @floatFromInt(x - px);
    var dz: f32 = @floatFromInt(z - pz);
    const d = pqx * pqx + pqz * pqz;
    var t = pqx * dx + pqz * dz;
    if (d > 0) t /= d;
    if (t < 0) {
        t = 0;
    } else if (t > 1) {
        t = 1;
    }

    dx = @as(f32, @floatFromInt(px)) + t * pqx - @as(f32, @floatFromInt(x));
    dz = @as(f32, @floatFromInt(pz)) + t * pqz - @as(f32, @floatFromInt(z));

    return dx * dx + dz * dz;
}

/// Reduces the raw outline in `cont` to the vertices where the neighboring
/// region changes, plus the ones needed to stay within `max_error` cells of
/// the raw outline. The kept vertices replace the raw ones, starting from
/// the one that came first.
fn simplifyContour(cont: *TempContour, max_error: f32) void {
    const vertices = cont.vertices;
    const poly = cont.poly;

    cont.poly_count = 0;

    var i: i32 = 0;
    while (i < cont.vertex_count) : (i += 1) {
        const j = @rem(i + 1, cont.vertex_count);
        // Check for the start of a wall segment.
        const ra = vertices[@intCast(j * 4 + 3)];
        const rb = vertices[@intCast(i * 4 + 3)];
        if (ra != rb) {
            poly[@intCast(cont.poly_count)] = @intCast(i);
            cont.poly_count += 1;
        }
    }
    if (cont.poly_count < 2) {
        // With no transitions at all, seed the simplification with the
        // contour's lower-left and upper-right vertices.
        var llx: i32 = vertices[0];
        var llz: i32 = vertices[2];
        var lli: i32 = 0;
        var urx: i32 = vertices[0];
        var urz: i32 = vertices[2];
        var uri: i32 = 0;
        i = 1;
        while (i < cont.vertex_count) : (i += 1) {
            const x: i32 = vertices[@intCast(i * 4 + 0)];
            const z: i32 = vertices[@intCast(i * 4 + 2)];
            if (x < llx or (x == llx and z < llz)) {
                llx = x;
                llz = z;
                lli = i;
            }
            if (x > urx or (x == urx and z > urz)) {
                urx = x;
                urz = z;
                uri = i;
            }
        }
        poly[0] = @intCast(lli);
        poly[1] = @intCast(uri);
        cont.poly_count = 2;
    }

    // Add points until all raw points are within the error tolerance of the
    // simplified shape.
    i = 0;
    while (i < cont.poly_count) {
        const ii = @rem(i + 1, cont.poly_count);

        const ai: i32 = poly[@intCast(i)];
        const ax: i32 = vertices[@intCast(ai * 4 + 0)];
        const az: i32 = vertices[@intCast(ai * 4 + 2)];

        const bi: i32 = poly[@intCast(ii)];
        const bx: i32 = vertices[@intCast(bi * 4 + 0)];
        const bz: i32 = vertices[@intCast(bi * 4 + 2)];

        // Find the maximum deviation from the segment.
        var deviation_max: f32 = 0;
        var deviation_index: i32 = -1;

        // Traverse the segment in lexicographic order, so the deviation
        // comes out the same when the opposite segment is traversed.
        const forward = bx > ax or (bx == ax and bz > az);
        const cinc: i32 = if (forward) 1 else cont.vertex_count - 1;
        var ci = @rem((if (forward) ai else bi) + cinc, cont.vertex_count);
        const endi = if (forward) bi else ai;

        // Tessellate only outer edges or edges between areas.
        while (ci != endi) {
            const d = distancePointSegment(
                vertices[@intCast(ci * 4 + 0)],
                vertices[@intCast(ci * 4 + 2)],
                ax,
                az,
                bx,
                bz,
            );
            if (d > deviation_max) {
                deviation_max = d;
                deviation_index = ci;
            }
            ci = @rem(ci + cinc, cont.vertex_count);
        }

        // If the max deviation is larger than the accepted error, add a new
        // point, else continue to the next segment.
        if (deviation_index != -1 and deviation_max > (max_error * max_error)) {
            cont.poly_count += 1;
            var j = cont.poly_count - 1;
            while (j > i) : (j -= 1) poly[@intCast(j)] = poly[@intCast(j - 1)];
            poly[@intCast(i + 1)] = @intCast(deviation_index);
        } else {
            i += 1;
        }
    }

    // Remap the vertices, starting from the one with the lowest raw index.
    var start: i32 = 0;
    i = 1;
    while (i < cont.poly_count) : (i += 1) {
        if (poly[@intCast(i)] < poly[@intCast(start)]) start = i;
    }

    cont.vertex_count = 0;
    i = 0;
    while (i < cont.poly_count) : (i += 1) {
        const j = @rem(start + i, cont.poly_count);
        // The source never lies before the destination, so copying a byte
        // at a time in place is safe.
        const src = @as(usize, poly[@intCast(j)]) * 4;
        const dst = @as(usize, @intCast(cont.vertex_count)) * 4;
        vertices[dst + 0] = vertices[src + 0];
        vertices[dst + 1] = vertices[src + 1];
        vertices[dst + 2] = vertices[src + 2];
        vertices[dst + 3] = vertices[src + 3];
        cont.vertex_count += 1;
    }
}

const Corner = struct {
    height: u8,
    /// Whether the mesh can drop the vertex: it lies on a tile's border
    /// inside one region, where the neighboring tile may have no vertex.
    should_remove: bool,
};

/// The highest of the up to four cells around grid corner `x`, `z` that lie
/// within `walkable_climb` of height `y`.
fn cornerHeight(layer: *const Layer, x: i32, y: i32, z: i32, walkable_climb: i32) Corner {
    const w: i32 = layer.header.width;
    const h: i32 = layer.header.height;

    var n: i32 = 0;

    var portal: u8 = 0xf;
    var height: u8 = 0;
    var previous_region: u8 = 0xff;
    var all_same_region = true;

    var dz: i32 = -1;
    while (dz <= 0) : (dz += 1) {
        var dx: i32 = -1;
        while (dx <= 0) : (dx += 1) {
            const px = x + dx;
            const pz = z + dz;
            if (px >= 0 and pz >= 0 and px < w and pz < h) {
                const idx: usize = @intCast(px + pz * w);
                const lh: i32 = layer.heights[idx];
                if (common.abs(i32, lh - y) <= walkable_climb and
                    layer.areas[idx] != null_area)
                {
                    height = @max(height, layer.heights[idx]);
                    portal &= (layer.connections[idx] >> 4);
                    if (previous_region != 0xff and previous_region != layer.regions[idx]) {
                        all_same_region = false;
                    }
                    previous_region = layer.regions[idx];
                    n += 1;
                }
            }
        }
    }

    var portal_count: i32 = 0;
    var dir: u3 = 0;
    while (dir < 4) : (dir += 1) {
        if ((portal & (@as(u8, 1) << dir)) != 0) portal_count += 1;
    }

    return .{
        .height = height,
        .should_remove = n > 1 and portal_count == 1 and all_same_region,
    };
}

/// Traces and simplifies the outline of every region of `layer`, which
/// `buildRegions` has run on, into `contour_set`, keeping each within
/// `max_error` cells of the raw outline.
///
/// `contour_set` comes empty from `allocContourSet`; its contours and their
/// vertices are allocated from `scratch` and stay until `freeContourSet`,
/// also after a failure. Fails with `buffer_too_small` when an outline has
/// more vertices than twice the layer's perimeter.
pub fn buildContours(
    scratch: std.mem.Allocator,
    layer: *const Layer,
    walkable_climb: i32,
    max_error: f32,
    contour_set: *ContourSet,
) Status {
    const w: i32 = layer.header.width;
    const h: i32 = layer.header.height;

    contour_set.contour_count = layer.region_count;
    contour_set.contours = scratch.alloc(Contour, layer.region_count) catch {
        contour_set.contours = &.{};
        return out_of_memory;
    };
    @memset(contour_set.contours, .{});

    // The temporary buffer for contour tracing: twice around the layer.
    const temp_vertex_count_max = (w + h) * 2 * 2;
    const temp_capacity: usize = @intCast(temp_vertex_count_max);

    const temp_vertices = scratch.alloc(u8, temp_capacity * 4) catch return out_of_memory;
    defer scratch.free(temp_vertices);

    const temp_poly = scratch.alloc(u16, temp_capacity) catch return out_of_memory;
    defer scratch.free(temp_poly);

    var temp: TempContour = .{
        .vertices = temp_vertices,
        .vertex_capacity = temp_vertex_count_max,
        .poly = temp_poly,
    };

    // Find contours.
    var y: i32 = 0;
    while (y < h) : (y += 1) {
        var x: i32 = 0;
        while (x < w) : (x += 1) {
            const idx: usize = @intCast(x + y * w);
            const ri = layer.regions[idx];
            if (ri == 0xff) continue;

            const cont = &contour_set.contours[ri];

            if (cont.vertex_count > 0) continue;

            cont.region = ri;
            cont.area = layer.areas[idx];

            // The contour is too complex for the temporary buffer.
            if (!walkContour(layer, x, y, &temp)) return buffer_too_small;

            simplifyContour(&temp, max_error);

            // Store the contour.
            cont.vertex_count = temp.vertex_count;
            if (cont.vertex_count > 0) {
                const count: usize = @intCast(temp.vertex_count);
                cont.vertices = scratch.alloc(u8, 4 * count) catch return out_of_memory;

                var j = count - 1;
                for (0..count) |i| {
                    defer j = i;
                    const dst = cont.vertices[j * 4 ..][0..4];
                    const v = temp.vertices[j * 4 ..][0..4];
                    const vn = temp.vertices[i * 4 ..][0..4];
                    // The neighbor region is stored at the segment's vertex.
                    const neighbor = vn[3];
                    const corner = cornerHeight(layer, v[0], v[1], v[2], walkable_climb);

                    dst[0] = v[0];
                    dst[1] = corner.height;
                    dst[2] = v[2];

                    // The fourth component holds the portal direction and
                    // whether the vertex can be removed.
                    dst[3] = 0x0f;
                    if (neighbor != 0xff and neighbor >= 0xf8) dst[3] = neighbor - 0xf8;
                    if (corner.should_remove) dst[3] |= 0x80;
                }
            }
        }
    }

    return .success;
}

fn computeVertexHash(x: i32, y: i32, z: i32) usize {
    // Large multiplicative constants, arbitrarily chosen primes.
    const h1: u32 = 0x8da6b343;
    const h2: u32 = 0xd8163841;
    const h3: u32 = 0xcb1ab31f;
    const n = h1 *% @as(u32, @bitCast(x)) +% h2 *% @as(u32, @bitCast(y)) +%
        h3 *% @as(u32, @bitCast(z));
    return n & (vertex_bucket_count - 1);
}

/// The index of the mesh vertex at `x`, `z` within two height units of `y`,
/// adding it when there is none. `first_vertex` and `next_vertex` chain the
/// vertices of each hash bucket.
fn addVertex(
    x: u16,
    y: u16,
    z: u16,
    vertices: []u16,
    first_vertex: *[vertex_bucket_count]u16,
    next_vertex: []u16,
    vertex_count: *i32,
) u16 {
    const bucket = computeVertexHash(x, 0, z);
    var i = first_vertex[bucket];

    while (i != null_index) {
        const v = vertices[@as(usize, i) * 3 ..][0..3];
        if (v[0] == x and v[2] == z and
            common.abs(i32, @as(i32, v[1]) - @as(i32, y)) <= 2)
        {
            return i;
        }
        i = next_vertex[i];
    }

    // Could not find one: create a new vertex.
    i = @truncate(@as(u32, @bitCast(vertex_count.*)));
    vertex_count.* += 1;
    const v = vertices[@as(usize, i) * 3 ..][0..3];
    v.* = .{ x, y, z };
    next_vertex[i] = first_vertex[bucket];
    first_vertex[bucket] = i;

    return i;
}

const Edge = struct {
    vert: [2]u16,
    poly_edge: [2]u16,
    poly: [2]u16,
};

/// Fills the second half of every polygon with what lies across each edge:
/// the neighboring polygon, or 0x8000 plus the direction for an edge on a
/// contour's portal. False when `scratch` is out of memory.
fn buildMeshAdjacency(
    scratch: std.mem.Allocator,
    polys: []u16,
    poly_count: i32,
    verts: []const u16,
    vertex_count: i32,
    contour_set: *const ContourSet,
) bool {
    const stride = vertices_per_polygon_max * 2;
    const polygon_count: usize = @intCast(poly_count);
    const first_edge_count: usize = @intCast(vertex_count);

    const edge_count_max = polygon_count * vertices_per_polygon_max;
    const edge_links = scratch.alloc(u16, first_edge_count + edge_count_max) catch return false;
    defer scratch.free(edge_links);
    const first_edge = edge_links[0..first_edge_count];
    const next_edge = edge_links[first_edge_count..];
    var edge_count: usize = 0;

    const edges = scratch.alloc(Edge, edge_count_max) catch return false;
    defer scratch.free(edges);

    @memset(first_edge, null_index);

    for (0..polygon_count) |i| {
        const t = polys[i * stride ..][0..vertices_per_polygon_max];
        for (0..vertices_per_polygon_max) |j| {
            if (t[j] == null_index) break;
            const v0 = t[j];
            const v1 = if (j + 1 >= vertices_per_polygon_max or t[j + 1] == null_index)
                t[0]
            else
                t[j + 1];
            if (v0 < v1) {
                edges[edge_count] = .{
                    .vert = .{ v0, v1 },
                    .poly = .{ @truncate(i), @truncate(i) },
                    .poly_edge = .{ @intCast(j), 0xff },
                };
                // Insert the edge.
                next_edge[edge_count] = first_edge[v0];
                first_edge[v0] = @truncate(edge_count);
                edge_count += 1;
            }
        }
    }

    for (0..polygon_count) |i| {
        const t = polys[i * stride ..][0..vertices_per_polygon_max];
        for (0..vertices_per_polygon_max) |j| {
            if (t[j] == null_index) break;
            const v0 = t[j];
            const v1 = if (j + 1 >= vertices_per_polygon_max or t[j + 1] == null_index)
                t[0]
            else
                t[j + 1];
            if (v0 > v1) {
                var found = false;
                var e = first_edge[v1];
                while (e != null_index) : (e = next_edge[e]) {
                    const edge = &edges[e];
                    if (edge.vert[1] == v0 and edge.poly[0] == edge.poly[1]) {
                        edge.poly[1] = @truncate(i);
                        edge.poly_edge[1] = @intCast(j);
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    // No matching edge: it is an open edge, add it.
                    edges[edge_count] = .{
                        .vert = .{ v1, v0 },
                        .poly = .{ @truncate(i), @truncate(i) },
                        .poly_edge = .{ @intCast(j), 0xff },
                    };
                    // Insert the edge.
                    next_edge[edge_count] = first_edge[v1];
                    first_edge[v1] = @truncate(edge_count);
                    edge_count += 1;
                }
            }
        }
    }

    // Mark portal edges.
    for (contour_set.contours[0..@intCast(contour_set.contour_count)]) |*cont| {
        if (cont.vertex_count < 3) continue;

        const count: usize = @intCast(cont.vertex_count);
        var k = count - 1;
        for (0..count) |j| {
            defer k = j;
            const va = cont.vertices[k * 4 ..][0..4];
            const vb = cont.vertices[j * 4 ..][0..4];
            const dir = va[3] & 0xf;
            if (dir == 0xf) continue;

            if (dir == 0 or dir == 2) {
                // Find matching edges along z.
                const x: u16 = va[0];
                var z_min: u16 = va[2];
                var z_max: u16 = vb[2];
                if (z_min > z_max) std.mem.swap(u16, &z_min, &z_max);

                for (edges[0..edge_count]) |*e| {
                    // Skip connected edges.
                    if (e.poly[0] != e.poly[1]) continue;
                    const eva = verts[@as(usize, e.vert[0]) * 3 ..][0..3];
                    const evb = verts[@as(usize, e.vert[1]) * 3 ..][0..3];
                    if (eva[0] == x and evb[0] == x) {
                        var edge_z_min = eva[2];
                        var edge_z_max = evb[2];
                        if (edge_z_min > edge_z_max) {
                            std.mem.swap(u16, &edge_z_min, &edge_z_max);
                        }
                        if (overlapRangeExclusive(z_min, z_max, edge_z_min, edge_z_max)) {
                            // Reuse the other polygon edge to store the direction.
                            e.poly_edge[1] = dir;
                        }
                    }
                }
            } else {
                // Find matching edges along x.
                const z: u16 = va[2];
                var x_min: u16 = va[0];
                var x_max: u16 = vb[0];
                if (x_min > x_max) std.mem.swap(u16, &x_min, &x_max);

                for (edges[0..edge_count]) |*e| {
                    // Skip connected edges.
                    if (e.poly[0] != e.poly[1]) continue;
                    const eva = verts[@as(usize, e.vert[0]) * 3 ..][0..3];
                    const evb = verts[@as(usize, e.vert[1]) * 3 ..][0..3];
                    if (eva[2] == z and evb[2] == z) {
                        var edge_x_min = eva[0];
                        var edge_x_max = evb[0];
                        if (edge_x_min > edge_x_max) {
                            std.mem.swap(u16, &edge_x_min, &edge_x_max);
                        }
                        if (overlapRangeExclusive(x_min, x_max, edge_x_min, edge_x_max)) {
                            // Reuse the other polygon edge to store the direction.
                            e.poly_edge[1] = dir;
                        }
                    }
                }
            }
        }
    }

    // Store adjacency.
    for (edges[0..edge_count]) |*e| {
        if (e.poly[0] != e.poly[1]) {
            const p0 = polys[@as(usize, e.poly[0]) * stride ..][0..stride];
            const p1 = polys[@as(usize, e.poly[1]) * stride ..][0..stride];
            p0[vertices_per_polygon_max + e.poly_edge[0]] = e.poly[1];
            p1[vertices_per_polygon_max + e.poly_edge[1]] = e.poly[0];
        } else if (e.poly_edge[1] != 0xff) {
            const p0 = polys[@as(usize, e.poly[0]) * stride ..][0..stride];
            p0[vertices_per_polygon_max + e.poly_edge[0]] = 0x8000 | e.poly_edge[1];
        }
    }

    return true;
}

fn prev(i: i32, n: i32) i32 {
    return if (i - 1 >= 0) i - 1 else n - 1;
}

fn next(i: i32, n: i32) i32 {
    return if (i + 1 < n) i + 1 else 0;
}

/// The contour vertex an index entry names, ignoring its "can be removed"
/// bit.
fn contourVertex(verts: []const u8, index: u16) *const [4]u8 {
    return verts[@as(usize, index & 0x7fff) * 4 ..][0..4];
}

fn area2(a: *const [4]u8, b: *const [4]u8, c: *const [4]u8) i32 {
    return (@as(i32, b[0]) - @as(i32, a[0])) * (@as(i32, c[2]) - @as(i32, a[2])) -
        (@as(i32, c[0]) - @as(i32, a[0])) * (@as(i32, b[2]) - @as(i32, a[2]));
}

/// Whether `c` is strictly to the left of the directed line from `a` to `b`.
fn left(a: *const [4]u8, b: *const [4]u8, c: *const [4]u8) bool {
    return area2(a, b, c) < 0;
}

fn leftOn(a: *const [4]u8, b: *const [4]u8, c: *const [4]u8) bool {
    return area2(a, b, c) <= 0;
}

fn collinear(a: *const [4]u8, b: *const [4]u8, c: *const [4]u8) bool {
    return area2(a, b, c) == 0;
}

/// Whether `ab` properly intersects `cd`: they share a point interior to
/// both segments. Strict leftness makes the intersection proper.
fn intersectProp(
    a: *const [4]u8,
    b: *const [4]u8,
    c: *const [4]u8,
    d: *const [4]u8,
) bool {
    // Eliminate improper cases.
    if (collinear(a, b, c) or collinear(a, b, d) or
        collinear(c, d, a) or collinear(c, d, b))
    {
        return false;
    }

    return (left(a, b, c) != left(a, b, d)) and (left(c, d, a) != left(c, d, b));
}

/// Whether `a`, `b`, `c` are collinear and `c` lies on the closed segment
/// `ab`.
fn between(a: *const [4]u8, b: *const [4]u8, c: *const [4]u8) bool {
    if (!collinear(a, b, c)) return false;
    // If ab is not vertical, check betweenness on x, else on z.
    if (a[0] != b[0]) {
        return ((a[0] <= c[0]) and (c[0] <= b[0])) or ((a[0] >= c[0]) and (c[0] >= b[0]));
    } else {
        return ((a[2] <= c[2]) and (c[2] <= b[2])) or ((a[2] >= c[2]) and (c[2] >= b[2]));
    }
}

/// Whether segments `ab` and `cd` intersect, properly or improperly.
fn intersect(a: *const [4]u8, b: *const [4]u8, c: *const [4]u8, d: *const [4]u8) bool {
    if (intersectProp(a, b, c, d)) {
        return true;
    } else if (between(a, b, c) or between(a, b, d) or
        between(c, d, a) or between(c, d, b))
    {
        return true;
    } else {
        return false;
    }
}

fn vertexEqual(a: *const [4]u8, b: *const [4]u8) bool {
    return a[0] == b[0] and a[2] == b[2];
}

/// Whether `i`..`j` is a proper internal or external diagonal of the
/// polygon, ignoring the edges incident to `i` and `j`.
fn diagonalie(i: i32, j: i32, n: i32, verts: []const u8, indices: []const u16) bool {
    const d0 = contourVertex(verts, indices[@intCast(i)]);
    const d1 = contourVertex(verts, indices[@intCast(j)]);

    // For each edge (k, k+1) of the polygon.
    var k: i32 = 0;
    while (k < n) : (k += 1) {
        const k1 = next(k, n);
        // Skip edges incident to i or j.
        if (!((k == i) or (k1 == i) or (k == j) or (k1 == j))) {
            const p0 = contourVertex(verts, indices[@intCast(k)]);
            const p1 = contourVertex(verts, indices[@intCast(k1)]);

            if (vertexEqual(d0, p0) or vertexEqual(d1, p0) or
                vertexEqual(d0, p1) or vertexEqual(d1, p1))
            {
                continue;
            }

            if (intersect(d0, d1, p0, p1)) return false;
        }
    }
    return true;
}

/// Whether the diagonal `i`..`j` is strictly internal to the polygon in the
/// neighborhood of the `i` endpoint.
fn inCone(i: i32, j: i32, n: i32, verts: []const u8, indices: []const u16) bool {
    const pi = contourVertex(verts, indices[@intCast(i)]);
    const pj = contourVertex(verts, indices[@intCast(j)]);
    const pi1 = contourVertex(verts, indices[@intCast(next(i, n))]);
    const pin1 = contourVertex(verts, indices[@intCast(prev(i, n))]);

    // If P[i] is a convex vertex: i+1 left of or on (i-1, i).
    if (leftOn(pin1, pi, pi1)) return left(pi, pj, pin1) and left(pj, pi, pi1);
    // Assume (i-1, i, i+1) not collinear, else P[i] is reflex.
    return !(leftOn(pi, pj, pi1) and leftOn(pj, pi, pin1));
}

/// Whether `i`..`j` is a proper internal diagonal of the polygon.
fn diagonal(i: i32, j: i32, n: i32, verts: []const u8, indices: []const u16) bool {
    return inCone(i, j, n, verts, indices) and diagonalie(i, j, n, verts, indices);
}

/// Cuts the polygon `indices[0..vertex_count]` into triangles by clipping
/// the ear with the shortest diagonal first, writing three entries of
/// `indices` per triangle to `tris`. Returns the triangle count, negated
/// when it had to stop early with no ear left. `indices` is consumed.
fn triangulate(vertex_count: i32, verts: []const u8, indices: []u16, tris: []u16) i32 {
    var n = vertex_count;
    var triangle_count: i32 = 0;
    var dst: usize = 0;

    // The top bit of an index marks a vertex that can be removed.
    var i: i32 = 0;
    while (i < n) : (i += 1) {
        const j1 = next(i, n);
        const j2 = next(j1, n);
        if (diagonal(i, j2, n, verts, indices)) indices[@intCast(j1)] |= 0x8000;
    }

    while (n > 3) {
        var length_min: i32 = -1;
        var mini: i32 = -1;
        i = 0;
        while (i < n) : (i += 1) {
            const j1 = next(i, n);
            if ((indices[@intCast(j1)] & 0x8000) != 0) {
                const p0 = contourVertex(verts, indices[@intCast(i)]);
                const p2 = contourVertex(verts, indices[@intCast(next(j1, n))]);

                const dx = @as(i32, p2[0]) - @as(i32, p0[0]);
                const dz = @as(i32, p2[2]) - @as(i32, p0[2]);
                const length = dx * dx + dz * dz;
                if (length_min < 0 or length < length_min) {
                    length_min = length;
                    mini = i;
                }
            }
        }

        // No ear is left: this should not happen.
        if (mini == -1) return -triangle_count;

        i = mini;
        var j1 = next(i, n);
        const j2 = next(j1, n);

        tris[dst + 0] = indices[@intCast(i)] & 0x7fff;
        tris[dst + 1] = indices[@intCast(j1)] & 0x7fff;
        tris[dst + 2] = indices[@intCast(j2)] & 0x7fff;
        dst += 3;
        triangle_count += 1;

        // Remove P[j1] by copying P[i+1]...P[n-1] left one index.
        n -= 1;
        var k = j1;
        while (k < n) : (k += 1) indices[@intCast(k)] = indices[@intCast(k + 1)];

        if (j1 >= n) j1 = 0;
        i = prev(j1, n);
        // Update the diagonal flags.
        if (diagonal(prev(i, n), j1, n, verts, indices)) {
            indices[@intCast(i)] |= 0x8000;
        } else {
            indices[@intCast(i)] &= 0x7fff;
        }

        if (diagonal(i, next(j1, n), n, verts, indices)) {
            indices[@intCast(j1)] |= 0x8000;
        } else {
            indices[@intCast(j1)] &= 0x7fff;
        }
    }

    // Append the remaining triangle.
    tris[dst + 0] = indices[0] & 0x7fff;
    tris[dst + 1] = indices[1] & 0x7fff;
    tris[dst + 2] = indices[2] & 0x7fff;
    triangle_count += 1;

    return triangle_count;
}

fn countPolyVerts(p: []const u16) i32 {
    for (p[0..vertices_per_polygon_max], 0..) |v, i| {
        if (v == null_index) return @intCast(i);
    }
    return vertices_per_polygon_max;
}

fn uleft(a: *const [3]u16, b: *const [3]u16, c: *const [3]u16) bool {
    return (@as(i32, b[0]) - @as(i32, a[0])) *% (@as(i32, c[2]) - @as(i32, a[2])) -%
        (@as(i32, c[0]) - @as(i32, a[0])) *% (@as(i32, b[2]) - @as(i32, a[2])) < 0;
}

const PolyMerge = struct {
    /// The squared length of the shared edge, or -1 when the two polygons
    /// cannot merge.
    value: i32,
    /// The shared edge in each polygon.
    edge_a: i32,
    edge_b: i32,
};

fn getPolyMergeValue(pa: []const u16, pb: []const u16, verts: []const u16) PolyMerge {
    const na = countPolyVerts(pa);
    const nb = countPolyVerts(pb);

    var merge: PolyMerge = .{ .value = -1, .edge_a = -1, .edge_b = -1 };

    // If the merged polygon would be too big, do not merge.
    if (na + nb - 2 > vertices_per_polygon_max) return merge;

    // Check if the polygons share an edge.
    var i: i32 = 0;
    while (i < na) : (i += 1) {
        var va0 = pa[@intCast(i)];
        var va1 = pa[@intCast(@rem(i + 1, na))];
        if (va0 > va1) std.mem.swap(u16, &va0, &va1);
        var j: i32 = 0;
        while (j < nb) : (j += 1) {
            var vb0 = pb[@intCast(j)];
            var vb1 = pb[@intCast(@rem(j + 1, nb))];
            if (vb0 > vb1) std.mem.swap(u16, &vb0, &vb1);
            if (va0 == vb0 and va1 == vb1) {
                merge.edge_a = i;
                merge.edge_b = j;
                break;
            }
        }
    }
    const ea = merge.edge_a;
    const eb = merge.edge_b;

    // No common edge, cannot merge.
    if (ea == -1 or eb == -1) return merge;

    // Check to see if the merged polygon would be convex.
    var va = pa[@intCast(@rem(ea + na - 1, na))];
    var vb = pa[@intCast(ea)];
    var vc = pb[@intCast(@rem(eb + 2, nb))];
    if (!uleft(meshVertex(verts, va), meshVertex(verts, vb), meshVertex(verts, vc))) {
        return merge;
    }

    va = pb[@intCast(@rem(eb + nb - 1, nb))];
    vb = pb[@intCast(eb)];
    vc = pa[@intCast(@rem(ea + 2, na))];
    if (!uleft(meshVertex(verts, va), meshVertex(verts, vb), meshVertex(verts, vc))) {
        return merge;
    }

    va = pa[@intCast(ea)];
    vb = pa[@intCast(@rem(ea + 1, na))];

    const a = meshVertex(verts, va);
    const b = meshVertex(verts, vb);
    const dx = @as(i32, a[0]) - @as(i32, b[0]);
    const dy = @as(i32, a[2]) - @as(i32, b[2]);

    merge.value = dx *% dx +% dy *% dy;
    return merge;
}

fn meshVertex(verts: []const u16, index: u16) *const [3]u16 {
    return verts[@as(usize, index) * 3 ..][0..3];
}

/// Replaces polygon `pa` with `pa` and `pb` joined across their shared
/// edges `ea` and `eb`.
fn mergePolys(pa: []u16, pb: []const u16, ea: i32, eb: i32) void {
    var tmp: [vertices_per_polygon_max * 2]u16 = @splat(0xffff);

    const na = countPolyVerts(pa);
    const nb = countPolyVerts(pb);

    // Merge the polygons.
    var n: usize = 0;
    // Add pa.
    var i: i32 = 0;
    while (i < na - 1) : (i += 1) {
        tmp[n] = pa[@intCast(@rem(ea + 1 + i, na))];
        n += 1;
    }
    // Add pb.
    i = 0;
    while (i < nb - 1) : (i += 1) {
        tmp[n] = pb[@intCast(@rem(eb + 1 + i, nb))];
        n += 1;
    }

    @memcpy(pa[0..vertices_per_polygon_max], tmp[0..vertices_per_polygon_max]);
}

fn pushFront(v: u16, arr: []u16, count: *i32) void {
    count.* += 1;
    var i: usize = @intCast(count.* - 1);
    while (i > 0) : (i -= 1) arr[i] = arr[i - 1];
    arr[0] = v;
}

fn pushBack(v: u16, arr: []u16, count: *i32) void {
    arr[@intCast(count.*)] = v;
    count.* += 1;
}

/// Whether removing vertex `rem` leaves a hole that can be filled again.
fn canRemoveVertex(mesh: *const PolyMesh, rem: u16) bool {
    const stride = vertices_per_polygon_max * 2;
    const polygon_count: usize = @intCast(mesh.polygon_count);

    // Count the polygons to remove.
    var touched_vertex_count: i32 = 0;
    var remaining_edge_count: i32 = 0;
    for (0..polygon_count) |i| {
        const p = mesh.polygons[i * stride ..][0..stride];
        const nv: usize = @intCast(countPolyVerts(p));
        var removed_count: i32 = 0;
        var vertex_count: i32 = 0;
        for (p[0..nv]) |v| {
            if (v == rem) {
                touched_vertex_count += 1;
                removed_count += 1;
            }
            vertex_count += 1;
        }
        if (removed_count != 0) remaining_edge_count += vertex_count - (removed_count + 1);
    }

    // There would be too few edges remaining to create a polygon. This can
    // happen when the tip of a triangle is marked for deletion, but no
    // other polygon shares the vertex: the vertex then stays.
    if (remaining_edge_count <= 2) return false;

    // Check that there is enough memory for the test.
    const edge_count_max = touched_vertex_count * 2;
    if (edge_count_max > removed_edge_count_max) return false;

    // Find the edges which share the removed vertex, three entries each:
    // the removed vertex, the other vertex, and how many polygons share it.
    var edges: [removed_edge_count_max * 3]u16 = undefined;
    var edge_count: usize = 0;

    for (0..polygon_count) |i| {
        const p = mesh.polygons[i * stride ..][0..stride];
        const nv: usize = @intCast(countPolyVerts(p));

        // Collect the edges which touch the removed vertex.
        var k = nv -% 1;
        for (0..nv) |j| {
            defer k = j;
            if (p[j] == rem or p[k] == rem) {
                // Arrange the edge so that a is the removed vertex.
                var a: i32 = p[j];
                var b: i32 = p[k];
                if (b == rem) std.mem.swap(i32, &a, &b);

                // Check if the edge exists.
                var exists = false;
                for (0..edge_count) |m| {
                    const e = edges[m * 3 ..][0..3];
                    if (e[1] == b) {
                        // It exists: increment the vertex share count.
                        e[2] += 1;
                        exists = true;
                    }
                }
                // Add a new edge.
                if (!exists) {
                    const e = edges[edge_count * 3 ..][0..3];
                    e[0] = @intCast(a);
                    e[1] = @intCast(b);
                    e[2] = 1;
                    edge_count += 1;
                }
            }
        }
    }

    // There should be no more than two open edges. This catches the case
    // of two non-adjacent polygons sharing the removed vertex, where the
    // vertex must stay.
    var open_edge_count: i32 = 0;
    for (0..edge_count) |i| {
        if (edges[i * 3 + 2] < 2) open_edge_count += 1;
    }
    if (open_edge_count > 2) return false;

    return true;
}

/// Removes vertex `rem` and the polygons around it from the mesh, then
/// fills the hole they leave with new polygons, up to `polygon_count_max`
/// in the mesh. Fails with `buffer_too_small` when the hole has more than
/// 48 edges.
fn removeVertex(mesh: *PolyMesh, rem: u16, polygon_count_max: i32) Status {
    const stride = vertices_per_polygon_max * 2;

    // The hole's edges, three entries each: both vertices and the area.
    var edge_count: usize = 0;
    var edges: [removed_edge_count_max * 3]u16 = undefined;
    var hole_count: i32 = 0;
    var hole: [removed_edge_count_max]u16 = undefined;
    var hole_area_count: i32 = 0;
    var hole_areas: [removed_edge_count_max]u16 = undefined;

    var i: usize = 0;
    while (i < @as(usize, @intCast(mesh.polygon_count))) {
        const p = mesh.polygons[i * stride ..][0..stride];
        const nv: usize = @intCast(countPolyVerts(p));
        var has_rem = false;
        for (p[0..nv]) |v| {
            if (v == rem) has_rem = true;
        }
        if (!has_rem) {
            i += 1;
            continue;
        }

        // Collect the edges which do not touch the removed vertex.
        var k = nv -% 1;
        for (0..nv) |j| {
            defer k = j;
            if (p[j] != rem and p[k] != rem) {
                if (edge_count >= removed_edge_count_max) return buffer_too_small;
                const e = edges[edge_count * 3 ..][0..3];
                e[0] = p[k];
                e[1] = p[j];
                e[2] = mesh.areas[i];
                edge_count += 1;
            }
        }
        // Remove the polygon by moving the last one into its place, then
        // look at the moved polygon next.
        const last: usize = @intCast(mesh.polygon_count - 1);
        const p2 = mesh.polygons[last * stride ..][0..stride];
        std.mem.copyForwards(
            u16,
            p[0..vertices_per_polygon_max],
            p2[0..vertices_per_polygon_max],
        );
        @memset(p[vertices_per_polygon_max..], 0xffff);
        mesh.areas[i] = mesh.areas[last];
        mesh.polygon_count -= 1;
    }

    // Remove the vertex.
    i = rem;
    while (@as(i32, @intCast(i)) < mesh.vertex_count - 1) : (i += 1) {
        mesh.vertices[i * 3 + 0] = mesh.vertices[(i + 1) * 3 + 0];
        mesh.vertices[i * 3 + 1] = mesh.vertices[(i + 1) * 3 + 1];
        mesh.vertices[i * 3 + 2] = mesh.vertices[(i + 1) * 3 + 2];
    }
    mesh.vertex_count -= 1;

    // Adjust the indices to match the removed vertex layout.
    for (0..@intCast(mesh.polygon_count)) |polygon| {
        const p = mesh.polygons[polygon * stride ..][0..stride];
        const nv: usize = @intCast(countPolyVerts(p));
        for (p[0..nv]) |*v| {
            if (v.* > rem) v.* -= 1;
        }
    }
    for (0..edge_count) |edge| {
        if (edges[edge * 3 + 0] > rem) edges[edge * 3 + 0] -= 1;
        if (edges[edge * 3 + 1] > rem) edges[edge * 3 + 1] -= 1;
    }

    if (edge_count == 0) return .success;

    // Start with one vertex, then keep appending connected segments to the
    // start and end of the hole.
    pushBack(edges[0], &hole, &hole_count);
    pushBack(edges[2], &hole_areas, &hole_area_count);

    while (edge_count != 0) {
        var match = false;

        i = 0;
        while (i < edge_count) {
            const ea = edges[i * 3 + 0];
            const eb = edges[i * 3 + 1];
            const a = edges[i * 3 + 2];
            var add = false;
            if (hole[0] == eb) {
                // The segment matches the beginning of the hole boundary.
                if (hole_count >= removed_edge_count_max) return buffer_too_small;
                pushFront(ea, &hole, &hole_count);
                pushFront(a, &hole_areas, &hole_area_count);
                add = true;
            } else if (hole[@intCast(hole_count - 1)] == ea) {
                // The segment matches the end of the hole boundary.
                if (hole_count >= removed_edge_count_max) return buffer_too_small;
                pushBack(eb, &hole, &hole_count);
                pushBack(a, &hole_areas, &hole_area_count);
                add = true;
            }
            if (add) {
                // The edge segment was added: remove it by moving the last
                // one into its place, then look at the moved segment next.
                edges[i * 3 + 0] = edges[(edge_count - 1) * 3 + 0];
                edges[i * 3 + 1] = edges[(edge_count - 1) * 3 + 1];
                edges[i * 3 + 2] = edges[(edge_count - 1) * 3 + 2];
                edge_count -= 1;
                match = true;
            } else {
                i += 1;
            }
        }

        if (!match) break;
    }

    // The hole's vertices for triangulation, four bytes each as a contour's
    // are. Zeroed, so a hole of fewer than three vertices reads zeros.
    var tris: [removed_edge_count_max * 3]u16 = @splat(0);
    var tverts: [removed_edge_count_max * 4]u8 = @splat(0);
    var tpoly: [removed_edge_count_max * 3]u16 = @splat(0);

    const hole_vertex_count: usize = @intCast(hole_count);
    for (hole[0..hole_vertex_count], 0..) |pi, h| {
        const v = meshVertex(mesh.vertices, pi);
        tverts[h * 4 + 0] = @truncate(v[0]);
        tverts[h * 4 + 1] = @truncate(v[1]);
        tverts[h * 4 + 2] = @truncate(v[2]);
        tverts[h * 4 + 3] = 0;
        tpoly[h] = @intCast(h);
    }

    // Triangulate the hole.
    var triangle_count = triangulate(hole_count, &tverts, &tpoly, &tris);
    if (triangle_count < 0) triangle_count = -triangle_count;

    if (triangle_count > removed_edge_count_max) return buffer_too_small;

    var polys: [removed_edge_count_max * vertices_per_polygon_max]u16 = undefined;
    var poly_areas: [removed_edge_count_max]u8 = undefined;

    // Build the initial polygons.
    var poly_count: usize = 0;
    const triangles: usize = @intCast(triangle_count);
    @memset(polys[0 .. triangles * vertices_per_polygon_max], 0xffff);
    for (0..triangles) |j| {
        const t = tris[j * 3 ..][0..3];
        if (t[0] != t[1] and t[0] != t[2] and t[1] != t[2]) {
            polys[poly_count * vertices_per_polygon_max + 0] = hole[t[0]];
            polys[poly_count * vertices_per_polygon_max + 1] = hole[t[1]];
            polys[poly_count * vertices_per_polygon_max + 2] = hole[t[2]];
            poly_areas[poly_count] = @truncate(hole_areas[t[0]]);
            poly_count += 1;
        }
    }
    if (poly_count == 0) return .success;

    // Merge the polygons.
    while (true) {
        const merging = polys[0 .. poly_count * vertices_per_polygon_max];
        const best = findBestMerge(merging, mesh.vertices);
        // No polygons could be merged: stop.
        if (best.value <= 0) break;

        // Found the best: merge, and move the last polygon into the place
        // of the one merged away.
        const pa = polys[best.poly_a * vertices_per_polygon_max ..][0..vertices_per_polygon_max];
        const pb = polys[best.poly_b * vertices_per_polygon_max ..][0..vertices_per_polygon_max];
        mergePolys(pa, pb, best.edge_a, best.edge_b);
        const last = polys[(poly_count - 1) * vertices_per_polygon_max ..];
        std.mem.copyForwards(u16, pb, last[0..vertices_per_polygon_max]);
        poly_areas[best.poly_b] = poly_areas[poly_count - 1];
        poly_count -= 1;
    }

    // Store the polygons.
    for (0..poly_count) |j| {
        if (mesh.polygon_count >= polygon_count_max) break;
        const p = mesh.polygons[@as(usize, @intCast(mesh.polygon_count)) * stride ..][0..stride];
        @memset(p, 0xffff);
        @memcpy(
            p[0..vertices_per_polygon_max],
            polys[j * vertices_per_polygon_max ..][0..vertices_per_polygon_max],
        );
        mesh.areas[@intCast(mesh.polygon_count)] = poly_areas[j];
        mesh.polygon_count += 1;
        if (mesh.polygon_count > polygon_count_max) return buffer_too_small;
    }

    return .success;
}

const BestMerge = struct {
    /// The squared length of the edge the two polygons share, 0 when no
    /// two polygons can merge.
    value: i32 = 0,
    poly_a: usize = 0,
    poly_b: usize = 0,
    edge_a: i32 = 0,
    edge_b: i32 = 0,
};

/// The pair of polygons in `polys`, six entries each, that share the
/// longest edge and stay convex when merged; the first such pair on a tie.
fn findBestMerge(polys: []const u16, verts: []const u16) BestMerge {
    const poly_count = polys.len / vertices_per_polygon_max;
    var best: BestMerge = .{};

    var j: usize = 0;
    while (j + 1 < poly_count) : (j += 1) {
        const pj = polys[j * vertices_per_polygon_max ..][0..vertices_per_polygon_max];
        var k = j + 1;
        while (k < poly_count) : (k += 1) {
            const pk = polys[k * vertices_per_polygon_max ..][0..vertices_per_polygon_max];
            const merge = getPolyMergeValue(pj, pk, verts);
            if (merge.value > best.value) {
                best = .{
                    .value = merge.value,
                    .poly_a = j,
                    .poly_b = k,
                    .edge_a = merge.edge_a,
                    .edge_b = merge.edge_b,
                };
            }
        }
    }
    return best;
}

/// Builds a layer's polygon mesh from its contours: each contour is
/// triangulated, the triangles are merged into convex polygons of up to six
/// vertices, the vertices `buildContours` marked removable are taken out,
/// and every polygon edge gets what lies across it.
///
/// `mesh` comes from `allocPolyMesh` and gets its four arrays from
/// `scratch`; `freePolyMesh` with the same `scratch` frees them, after a
/// failure too. Fails with `out_of_memory` when `scratch` runs out, and
/// with `buffer_too_small` when a removed vertex leaves a hole of more than
/// 48 edges.
pub fn buildPolyMesh(
    scratch: std.mem.Allocator,
    contour_set: *const ContourSet,
    mesh: *PolyMesh,
) Status {
    const stride = vertices_per_polygon_max * 2;
    const contours = contour_set.contours[0..@intCast(contour_set.contour_count)];

    var vertex_count_max: i32 = 0;
    var polygon_count_max: i32 = 0;
    var contour_vertex_count_max: i32 = 0;
    for (contours) |*cont| {
        // Skip null contours.
        if (cont.vertex_count < 3) continue;
        vertex_count_max += cont.vertex_count;
        polygon_count_max += cont.vertex_count - 2;
        contour_vertex_count_max = @max(contour_vertex_count_max, cont.vertex_count);
    }

    mesh.vertices_per_polygon = vertices_per_polygon_max;

    const vertex_capacity: usize = @intCast(vertex_count_max);
    const polygon_capacity: usize = @intCast(polygon_count_max);
    const contour_capacity: usize = @intCast(contour_vertex_count_max);

    // Per vertex, whether it is to be removed once the polygons are in.
    const vertex_flags = scratch.alloc(u8, vertex_capacity) catch return out_of_memory;
    defer scratch.free(vertex_flags);
    @memset(vertex_flags, 0);

    mesh.vertices = scratch.alloc(u16, vertex_capacity * 3) catch return out_of_memory;
    mesh.polygons = scratch.alloc(u16, polygon_capacity * stride) catch return out_of_memory;
    mesh.areas = scratch.alloc(u8, polygon_capacity) catch return out_of_memory;
    mesh.flags = scratch.alloc(u16, polygon_capacity) catch return out_of_memory;

    // The flags are only allocated and cleared: the caller fills them.
    @memset(mesh.flags, 0);

    mesh.vertex_count = 0;
    mesh.polygon_count = 0;

    @memset(mesh.vertices, 0);
    @memset(mesh.polygons, 0xffff);
    @memset(mesh.areas, 0);

    var first_vertex: [vertex_bucket_count]u16 = @splat(null_index);

    const next_vertex = scratch.alloc(u16, vertex_capacity) catch return out_of_memory;
    defer scratch.free(next_vertex);
    @memset(next_vertex, 0);

    const indices = scratch.alloc(u16, contour_capacity) catch return out_of_memory;
    defer scratch.free(indices);

    const tris = scratch.alloc(u16, contour_capacity * 3) catch return out_of_memory;
    defer scratch.free(tris);

    const polys = scratch.alloc(u16, contour_capacity * vertices_per_polygon_max) catch
        return out_of_memory;
    defer scratch.free(polys);

    for (contours) |*cont| {
        // Skip null contours.
        if (cont.vertex_count < 3) continue;
        const contour_vertex_count: usize = @intCast(cont.vertex_count);

        // Triangulate the contour.
        for (indices[0..contour_vertex_count], 0..) |*index, j| index.* = @truncate(j);

        var triangle_count = triangulate(cont.vertex_count, cont.vertices, indices, tris);
        if (triangle_count <= 0) triangle_count = -triangle_count;

        // Add and merge the vertices. The triangles keep naming contour
        // vertices, which `indices` now maps to mesh vertices.
        for (0..contour_vertex_count) |j| {
            const v = cont.vertices[j * 4 ..][0..4];
            indices[j] = addVertex(
                v[0],
                v[1],
                v[2],
                mesh.vertices,
                &first_vertex,
                next_vertex,
                &mesh.vertex_count,
            );
            if (v[3] & 0x80 != 0) {
                // This vertex should be removed.
                vertex_flags[indices[j]] = 1;
            }
        }

        // Build the initial polygons.
        var poly_count: usize = 0;
        @memset(polys, 0xffff);
        for (0..@intCast(triangle_count)) |j| {
            const t = tris[j * 3 ..][0..3];
            if (t[0] != t[1] and t[0] != t[2] and t[1] != t[2]) {
                polys[poly_count * vertices_per_polygon_max + 0] = indices[t[0]];
                polys[poly_count * vertices_per_polygon_max + 1] = indices[t[1]];
                polys[poly_count * vertices_per_polygon_max + 2] = indices[t[2]];
                poly_count += 1;
            }
        }
        if (poly_count == 0) continue;

        // Merge the polygons.
        while (true) {
            const merging = polys[0 .. poly_count * vertices_per_polygon_max];
            const best = findBestMerge(merging, mesh.vertices);
            // No polygons could be merged: stop.
            if (best.value <= 0) break;

            // Found the best: merge, and move the last polygon into the
            // place of the one merged away.
            const pa = polys[best.poly_a * vertices_per_polygon_max ..];
            const pb = polys[best.poly_b * vertices_per_polygon_max ..];
            mergePolys(pa, pb, best.edge_a, best.edge_b);
            const last = polys[(poly_count - 1) * vertices_per_polygon_max ..];
            std.mem.copyForwards(
                u16,
                pb[0..vertices_per_polygon_max],
                last[0..vertices_per_polygon_max],
            );
            poly_count -= 1;
        }

        // Store the polygons.
        for (0..poly_count) |j| {
            const p = mesh.polygons[@as(usize, @intCast(mesh.polygon_count)) * stride ..];
            const q = polys[j * vertices_per_polygon_max ..][0..vertices_per_polygon_max];
            @memcpy(p[0..vertices_per_polygon_max], q);
            mesh.areas[@intCast(mesh.polygon_count)] = cont.area;
            mesh.polygon_count += 1;
            if (mesh.polygon_count > polygon_count_max) return buffer_too_small;
        }
    }

    // Remove the vertices that only split a straight edge.
    var i: usize = 0;
    while (i < @as(usize, @intCast(mesh.vertex_count))) {
        if (vertex_flags[i] == 0 or !canRemoveVertex(mesh, @truncate(i))) {
            i += 1;
            continue;
        }
        const status = removeVertex(mesh, @truncate(i), polygon_count_max);
        if (status.failed) return status;
        // The mesh has one vertex fewer now: close the gap in the flags
        // too, then look at the vertex that moved into this place.
        for (i..@intCast(mesh.vertex_count)) |j| vertex_flags[j] = vertex_flags[j + 1];
    }

    // Calculate adjacency.
    if (!buildMeshAdjacency(
        scratch,
        mesh.polygons,
        mesh.polygon_count,
        mesh.vertices,
        mesh.vertex_count,
        contour_set,
    )) return out_of_memory;

    return .success;
}

/// Gives `area_id` to every cell an upright cylinder covers: the cells
/// whose center lies within `radius` plus half a cell of `position` on the
/// ground plane, and whose height lies from `position` up to `height`
/// above it. `origin` is the layer's lowest corner in the world, and
/// `cell_size` and `cell_height` are a cell's size there. Always succeeds.
pub fn markCylinderArea(
    layer: *Layer,
    origin: [3]f32,
    cell_size: f32,
    cell_height: f32,
    position: [3]f32,
    radius: f32,
    height: f32,
    area_id: u8,
) Status {
    const bounds_min: [3]f32 = .{
        position[0] - radius,
        position[1],
        position[2] - radius,
    };
    const bounds_max: [3]f32 = .{
        position[0] + radius,
        position[1] + height,
        position[2] + radius,
    };
    const radius_sqr = common.sqr(f32, radius / cell_size + 0.5);

    const w: i32 = layer.header.width;
    const h: i32 = layer.header.height;
    const ics: f32 = 1.0 / cell_size;
    const ich: f32 = 1.0 / cell_height;

    const px = (position[0] - origin[0]) * ics;
    const pz = (position[2] - origin[2]) * ics;

    var x_min = common.truncToInt(i32, @floor((bounds_min[0] - origin[0]) * ics));
    const y_min = common.truncToInt(i32, @floor((bounds_min[1] - origin[1]) * ich));
    var z_min = common.truncToInt(i32, @floor((bounds_min[2] - origin[2]) * ics));
    var x_max = common.truncToInt(i32, @floor((bounds_max[0] - origin[0]) * ics));
    const y_max = common.truncToInt(i32, @floor((bounds_max[1] - origin[1]) * ich));
    var z_max = common.truncToInt(i32, @floor((bounds_max[2] - origin[2]) * ics));

    if (x_max < 0) return .success;
    if (x_min >= w) return .success;
    if (z_max < 0) return .success;
    if (z_min >= h) return .success;

    if (x_min < 0) x_min = 0;
    if (x_max >= w) x_max = w - 1;
    if (z_min < 0) z_min = 0;
    if (z_max >= h) z_max = h - 1;

    var z = z_min;
    while (z <= z_max) : (z += 1) {
        var x = x_min;
        while (x <= x_max) : (x += 1) {
            const dx = (@as(f32, @floatFromInt(x)) + 0.5) - px;
            const dz = (@as(f32, @floatFromInt(z)) + 0.5) - pz;
            if (dx * dx + dz * dz > radius_sqr) continue;
            const cell: usize = @intCast(x + z * w);
            const y: i32 = layer.heights[cell];
            if (y < y_min or y > y_max) continue;
            layer.areas[cell] = area_id;
        }
    }

    return .success;
}

/// Gives `area_id` to every cell an axis-aligned box covers: the cells
/// from `bounds_min` to `bounds_max` on the ground plane whose height lies
/// between the two. `origin` is the layer's lowest corner in the world,
/// and `cell_size` and `cell_height` are a cell's size there. Always
/// succeeds.
pub fn markBoxArea(
    layer: *Layer,
    origin: [3]f32,
    cell_size: f32,
    cell_height: f32,
    bounds_min: [3]f32,
    bounds_max: [3]f32,
    area_id: u8,
) Status {
    const w: i32 = layer.header.width;
    const h: i32 = layer.header.height;
    const ics: f32 = 1.0 / cell_size;
    const ich: f32 = 1.0 / cell_height;

    var x_min = common.truncToInt(i32, @floor((bounds_min[0] - origin[0]) * ics));
    const y_min = common.truncToInt(i32, @floor((bounds_min[1] - origin[1]) * ich));
    var z_min = common.truncToInt(i32, @floor((bounds_min[2] - origin[2]) * ics));
    var x_max = common.truncToInt(i32, @floor((bounds_max[0] - origin[0]) * ics));
    const y_max = common.truncToInt(i32, @floor((bounds_max[1] - origin[1]) * ich));
    var z_max = common.truncToInt(i32, @floor((bounds_max[2] - origin[2]) * ics));

    if (x_max < 0) return .success;
    if (x_min >= w) return .success;
    if (z_max < 0) return .success;
    if (z_min >= h) return .success;

    if (x_min < 0) x_min = 0;
    if (x_max >= w) x_max = w - 1;
    if (z_min < 0) z_min = 0;
    if (z_max >= h) z_max = h - 1;

    var z = z_min;
    while (z <= z_max) : (z += 1) {
        var x = x_min;
        while (x <= x_max) : (x += 1) {
            const cell: usize = @intCast(x + z * w);
            const y: i32 = layer.heights[cell];
            if (y < y_min or y > y_max) continue;
            layer.areas[cell] = area_id;
        }
    }

    return .success;
}

/// Gives `area_id` to every cell a box turned about the up axis covers.
/// The box sits at `center` and reaches `half_extents` to each side, and
/// its height range is `center` plus and minus `half_extents[1]`.
///
/// `rot_aux` carries the turn by angle `a` as
/// `{ cos(a / 2) * sin(-a / 2), cos(a / 2) * cos(a / 2) - 0.5 }`, half the
/// sine of `-a` and half the cosine of `a`. `origin` is the layer's lowest
/// corner in the world, and `cell_size` and `cell_height` are a cell's size
/// there. Always succeeds.
pub fn markOrientedBoxArea(
    layer: *Layer,
    origin: [3]f32,
    cell_size: f32,
    cell_height: f32,
    center: [3]f32,
    half_extents: [3]f32,
    rot_aux: [2]f32,
    area_id: u8,
) Status {
    const w: i32 = layer.header.width;
    const h: i32 = layer.header.height;
    const ics: f32 = 1.0 / cell_size;
    const ich: f32 = 1.0 / cell_height;

    const cx = (center[0] - origin[0]) * ics;
    const cz = (center[2] - origin[2]) * ics;

    // A circle around the box, whatever its turn: 1.41 is the square root
    // of two, rounded down.
    const radius_max: f32 = 1.41 * common.max(f32, half_extents[0], half_extents[2]);
    var x_min = common.truncToInt(i32, @floor(cx - radius_max * ics));
    var x_max = common.truncToInt(i32, @floor(cx + radius_max * ics));
    var z_min = common.truncToInt(i32, @floor(cz - radius_max * ics));
    var z_max = common.truncToInt(i32, @floor(cz + radius_max * ics));
    const y_min = common.truncToInt(
        i32,
        @floor((center[1] - half_extents[1] - origin[1]) * ich),
    );
    const y_max = common.truncToInt(
        i32,
        @floor((center[1] + half_extents[1] - origin[1]) * ich),
    );

    if (x_max < 0) return .success;
    if (x_min >= w) return .success;
    if (z_max < 0) return .success;
    if (z_min >= h) return .success;

    if (x_min < 0) x_min = 0;
    if (x_max >= w) x_max = w - 1;
    if (z_min < 0) z_min = 0;
    if (z_max >= h) z_max = h - 1;

    const x_half = half_extents[0] * ics + 0.5;
    const z_half = half_extents[2] * ics + 0.5;

    var z = z_min;
    while (z <= z_max) : (z += 1) {
        var x = x_min;
        while (x <= x_max) : (x += 1) {
            // Doubled offsets, to go with the halved sine and cosine.
            const x2 = 2.0 * (@as(f32, @floatFromInt(x)) - cx);
            const z2 = 2.0 * (@as(f32, @floatFromInt(z)) - cz);
            const x_rot = rot_aux[1] * x2 + rot_aux[0] * z2;
            if (x_rot > x_half or x_rot < -x_half) continue;
            const z_rot = rot_aux[1] * z2 - rot_aux[0] * x2;
            if (z_rot > z_half or z_rot < -z_half) continue;
            const cell: usize = @intCast(x + z * w);
            const y: i32 = layer.heights[cell];
            if (y < y_min or y > y_max) continue;
            layer.areas[cell] = area_id;
        }
    }

    return .success;
}

/// Packs a layer for storage: `header`, then the three grids compressed as
/// one run of heights, areas and connections, `width * height` cells each.
///
/// On success `out_data` is a block from `gpa`, exactly as long as the
/// packed layer, that the caller owns and frees with `gpa.free`. Fails with
/// `out_of_memory`, or with whatever the compressor fails with, and then
/// leaves `out_data` alone.
pub fn buildLayer(
    gpa: std.mem.Allocator,
    compressor: Compressor,
    header: *const LayerHeader,
    heights: []const u8,
    areas: []const u8,
    connections: []const u8,
    out_data: *[]u8,
) Status {
    const header_size: i32 = layer_header_size;
    const grid_size = @as(i32, header.width) * @as(i32, header.height);
    const data_size_max = header_size + compressor.maxCompressedSize(grid_size * 3);
    const data = gpa.alloc(u8, @intCast(data_size_max)) catch return out_of_memory;
    @memset(data, 0);

    // Store the header's fields. The struct's two trailing padding bytes
    // hold nothing, so the stored ones stay zero.
    const field_size = @offsetOf(LayerHeader, "y_max") + 1;
    @memcpy(data[0..field_size], std.mem.asBytes(header)[0..field_size]);

    // Concatenate the grids for compression.
    const cell_count: usize = @intCast(grid_size);
    const buffer = gpa.alloc(u8, cell_count * 3) catch {
        gpa.free(data);
        return out_of_memory;
    };

    @memcpy(buffer[0..cell_count], heights[0..cell_count]);
    @memcpy(buffer[cell_count..][0..cell_count], areas[0..cell_count]);
    @memcpy(buffer[cell_count * 2 ..][0..cell_count], connections[0..cell_count]);

    // Compress.
    var compressed_size: i32 = 0;
    const status = compressor.compress(buffer, data[layer_header_size..], &compressed_size);
    if (status.failed) {
        gpa.free(buffer);
        gpa.free(data);
        return status;
    }

    gpa.free(buffer);

    // Give back only the bytes in use, so the block's length is the size
    // of the packed layer and the caller can free it by that length.
    const data_size = layer_header_size + @as(usize, @intCast(compressed_size));
    out_data.* = gpa.realloc(data, data_size) catch {
        gpa.free(data);
        return out_of_memory;
    };

    return .success;
}

/// Frees a layer `decompressLayer` made, with the same `scratch`. The
/// layer, its header and its grids are one block, so all of them are gone
/// afterwards. Null is allowed and does nothing.
pub fn freeLayer(scratch: std.mem.Allocator, layer: ?*Layer) void {
    const l = layer orelse return;
    // The grids are four of the same length, after the layer and header.
    const block_size = layer_size + layer_header_size + l.heights.len * 4;
    const block: [*]align(@alignOf(Layer)) u8 = @ptrCast(l);
    scratch.free(block[0..block_size]);
}

/// Unpacks a layer `buildLayer` packed. The layer, a copy of its header and
/// its four grids come as one block from `scratch`, with the regions grid
/// zeroed for `buildRegions` to fill.
///
/// On success `layer_out` is the layer, valid until `freeLayer` with the
/// same `scratch`, or until `scratch` itself is reset. Otherwise it is
/// null. Fails with `invalid_param` when `compressed` is shorter than a
/// header, `wrong_magic` or `wrong_version` when the header is not this
/// format's, `out_of_memory`, or whatever the compressor fails with.
pub fn decompressLayer(
    scratch: std.mem.Allocator,
    compressor: Compressor,
    compressed: []const u8,
    layer_out: *?*Layer,
) Status {
    layer_out.* = null;

    if (compressed.len < layer_header_size) return invalid_param;

    // The packed bytes may sit at any address, so read the header from a
    // copy.
    var compressed_header: LayerHeader = undefined;
    @memcpy(std.mem.asBytes(&compressed_header), compressed[0..@sizeOf(LayerHeader)]);
    if (compressed_header.magic != tile_cache_magic) return wrong_magic;
    if (compressed_header.version != tile_cache_version) return wrong_version;

    const grid_size = @as(usize, compressed_header.width) * @as(usize, compressed_header.height);
    const block_size = layer_size + layer_header_size + grid_size * 4;

    const block = scratch.alignedAlloc(u8, std.mem.Alignment.of(Layer), block_size) catch
        return out_of_memory;
    @memset(block, 0);

    const layer: *Layer = @ptrCast(block.ptr);
    const header: *LayerHeader = @ptrCast(@alignCast(block[layer_size..].ptr));
    const grids = block[layer_size + layer_header_size ..];

    // Copy the header.
    @memcpy(block[layer_size..][0..layer_header_size], compressed[0..layer_header_size]);
    // Decompress the grids.
    var size: i32 = 0;
    const status = compressor.decompress(compressed[layer_header_size..], grids, &size);
    if (status.failed) {
        scratch.free(block);
        return status;
    }

    layer.* = .{
        .header = header,
        .heights = grids[0..grid_size],
        .areas = grids[grid_size..][0..grid_size],
        .connections = grids[grid_size * 2 ..][0..grid_size],
        .regions = grids[grid_size * 3 ..][0..grid_size],
    };

    layer_out.* = layer;

    return .success;
}

/// Reverses the byte order of the header that opens a packed layer, in
/// place, turning data written on a machine of the other byte order into
/// this machine's or back. The grids are bytes and need no swapping.
///
/// False, with `data` untouched, when it is shorter than a header or its
/// magic and version are this format's in neither byte order.
pub fn swapEndianHeader(data: []u8) bool {
    if (data.len < @sizeOf(LayerHeader)) return false;

    const magic: i32 = @bitCast(data[0..4].*);
    const version: i32 = @bitCast(data[4..8].*);

    const swapped_magic = @byteSwap(tile_cache_magic);
    const swapped_version = @byteSwap(tile_cache_version);

    if ((magic != tile_cache_magic or version != tile_cache_version) and
        (magic != swapped_magic or version != swapped_version))
    {
        return false;
    }

    // Magic, version, the tile's three coordinates and the two bounds are
    // four bytes each, the two height limits two each. The rest is bytes.
    const height_min_offset = @offsetOf(LayerHeader, "height_min");
    const height_max_offset = @offsetOf(LayerHeader, "height_max");
    var offset: usize = 0;
    while (offset < height_min_offset) : (offset += 4) {
        std.mem.reverse(u8, data[offset..][0..4]);
    }
    std.mem.reverse(u8, data[height_min_offset..][0..2]);
    std.mem.reverse(u8, data[height_max_offset..][0..2]);

    return true;
}
