//! A uniform grid that buckets items by the cells their bounds cover, for
//! finding the items near a box. The crowd refills one with its agents each
//! update to find every agent's neighbours.
//!
//! The grid has two axes, x and y; the crowd lays it over the xz plane, so
//! its y is the world's z.

const std = @import("std");
const common = @import("common.zig");

/// The bucket of cell `x`, `y` among `n` buckets, `n` a power of two. The
/// products wrap as 32-bit two's complement.
inline fn hashCell(x: i32, y: i32, n: i32) i32 {
    return ((x *% 73856093) ^ (y *% 19349663)) & (n - 1);
}

/// A hash of grid cells to the items covering them. Cells are unbounded in
/// both directions: only the cells items cover take entries.
pub const ProximityGrid = struct {
    cell_size: f32,
    cell_size_inverse: f32,
    /// One entry per cell an item covers, handed out in order.
    pool: []Item,
    /// The number of pool entries in use.
    pool_head: i32,
    pool_size: i32,
    /// Per hash, the pool index of the newest entry with that hash.
    buckets: []u16,
    /// The number of buckets, a power of two.
    bucket_count: i32,
    /// The cell range the items cover: lowest x, lowest y, highest x,
    /// highest y.
    bounds: [4]i32,

    const Item = struct {
        id: u16,
        x: i16,
        y: i16,
        /// The pool index of the next entry with the same hash.
        next: u16,
    };

    /// Ends a bucket's chain, and marks an empty bucket.
    const no_item: u16 = 0xffff;

    /// A grid that holds no memory, for a field to hold before `init`. Only
    /// `init` and `deinit` may be called on it.
    pub const empty: ProximityGrid = .{
        .cell_size = 0,
        .cell_size_inverse = 0,
        .pool = &.{},
        .pool_head = 0,
        .pool_size = 0,
        .buckets = &.{},
        .bucket_count = 0,
        .bounds = .{ 0, 0, 0, 0 },
    };

    /// Sets the grid up with `pool_size` entries, one used per cell an item
    /// covers, and cells `cell_size` wide; both are above zero. Returns
    /// false when memory runs out, leaving the grid `empty`.
    ///
    /// Entries name each other by 16-bit index, so a pool above 65535
    /// entries starts over at its first entry once that many are in use,
    /// and the chains through the reused entries are no longer sound.
    pub fn init(
        self: *ProximityGrid,
        gpa: std.mem.Allocator,
        pool_size: i32,
        cell_size: f32,
    ) bool {
        std.debug.assert(pool_size > 0);
        std.debug.assert(cell_size > 0.0);

        self.* = empty;

        // Allocate the hash buckets. A pool above 2^30 entries rounds up to
        // a count that does not fit the field, and no such table exists.
        const bucket_count: i32 = @bitCast(common.nextPow2(@bitCast(pool_size)));
        const bucket_len = std.math.cast(usize, bucket_count) orelse return false;
        const buckets = gpa.alloc(u16, bucket_len) catch return false;

        // Allocate the pool of items.
        const pool = gpa.alloc(Item, @intCast(pool_size)) catch {
            gpa.free(buckets);
            return false;
        };

        self.* = .{
            .cell_size = cell_size,
            .cell_size_inverse = 1.0 / cell_size,
            .pool = pool,
            .pool_head = 0,
            .pool_size = pool_size,
            .buckets = buckets,
            .bucket_count = bucket_count,
            .bounds = undefined,
        };
        self.clear();

        return true;
    }

    /// Frees the grid's memory. `gpa` is the allocator `init` was given.
    pub fn deinit(self: *ProximityGrid, gpa: std.mem.Allocator) void {
        gpa.free(self.buckets);
        gpa.free(self.pool);
        self.* = undefined;
    }

    /// Removes every item, keeping the memory.
    pub fn clear(self: *ProximityGrid) void {
        @memset(self.buckets, no_item);
        self.pool_head = 0;
        self.bounds = .{ 0xffff, 0xffff, -0xffff, -0xffff };
    }

    /// Adds item `id` to every cell the box `x_min`..`x_max`,
    /// `y_min`..`y_max` touches, one pool entry per cell. Once the pool is
    /// full the remaining cells are left out without notice.
    ///
    /// A coordinate too large for a cell number is taken as the farthest
    /// cell on its side, and a NaN as cell 0.
    pub fn addItem(
        self: *ProximityGrid,
        id: u16,
        x_min: f32,
        y_min: f32,
        x_max: f32,
        y_max: f32,
    ) void {
        const cell_x_min = common.truncToInt(i32, @floor(x_min * self.cell_size_inverse));
        const cell_y_min = common.truncToInt(i32, @floor(y_min * self.cell_size_inverse));
        const cell_x_max = common.truncToInt(i32, @floor(x_max * self.cell_size_inverse));
        const cell_y_max = common.truncToInt(i32, @floor(y_max * self.cell_size_inverse));

        self.bounds[0] = @min(self.bounds[0], cell_x_min);
        self.bounds[1] = @min(self.bounds[1], cell_y_min);
        self.bounds[2] = @max(self.bounds[2], cell_x_max);
        self.bounds[3] = @max(self.bounds[3], cell_y_max);

        // Each loop leaves at its last cell rather than stepping past it:
        // the last cell can be the largest integer, which has no successor.
        var y = cell_y_min;
        while (y <= cell_y_max) : (y += 1) {
            var x = cell_x_min;
            while (x <= cell_x_max) : (x += 1) {
                if (self.pool_head < self.pool_size) {
                    const h: usize = @intCast(hashCell(x, y, self.bucket_count));
                    const index: u16 = @truncate(@as(u32, @bitCast(self.pool_head)));
                    self.pool_head += 1;
                    const item = &self.pool[index];
                    item.x = @truncate(x);
                    item.y = @truncate(y);
                    item.id = id;
                    item.next = self.buckets[h];
                    self.buckets[h] = index;
                }
                if (x == cell_x_max) break;
            }
            if (y == cell_y_max) break;
        }
    }

    /// Writes to `ids` the id of every item in the cells the box
    /// `x_min`..`x_max`, `y_min`..`y_max` touches, each id once, and
    /// returns how many it wrote. It stops when `ids` is full.
    ///
    /// Coordinates become cells as in `addItem`.
    pub fn queryItems(
        self: *const ProximityGrid,
        x_min: f32,
        y_min: f32,
        x_max: f32,
        y_max: f32,
        ids: []u16,
    ) i32 {
        const cell_x_min = common.truncToInt(i32, @floor(x_min * self.cell_size_inverse));
        const cell_y_min = common.truncToInt(i32, @floor(y_min * self.cell_size_inverse));
        const cell_x_max = common.truncToInt(i32, @floor(x_max * self.cell_size_inverse));
        const cell_y_max = common.truncToInt(i32, @floor(y_max * self.cell_size_inverse));

        var n: i32 = 0;

        // The loops leave at their last cell, as in `addItem`.
        var y = cell_y_min;
        while (y <= cell_y_max) : (y += 1) {
            var x = cell_x_min;
            while (x <= cell_x_max) : (x += 1) {
                const h: usize = @intCast(hashCell(x, y, self.bucket_count));
                var index = self.buckets[h];
                while (index != no_item) {
                    const item = &self.pool[index];
                    if (@as(i32, item.x) == x and @as(i32, item.y) == y) {
                        // Check if the id exists already.
                        const found = ids[0..@intCast(n)];
                        if (std.mem.findScalar(u16, found, item.id) == null) {
                            // Item not found, add it.
                            if (found.len >= ids.len) return n;
                            ids[found.len] = item.id;
                            n += 1;
                        }
                    }
                    index = item.next;
                }
                if (x == cell_x_max) break;
            }
            if (y == cell_y_max) break;
        }

        return n;
    }

    /// The number of entries in cell `x`, `y`: one for each `addItem` call
    /// whose box touched it.
    pub fn getItemCountAt(self: *const ProximityGrid, x: i32, y: i32) i32 {
        var n: i32 = 0;

        const h: usize = @intCast(hashCell(x, y, self.bucket_count));
        var index = self.buckets[h];
        while (index != no_item) {
            const item = &self.pool[index];
            if (@as(i32, item.x) == x and @as(i32, item.y) == y) n += 1;
            index = item.next;
        }

        return n;
    }

    /// The cell range the items added since the last `clear` cover: lowest
    /// x, lowest y, highest x, highest y. The pointer is into the grid: it
    /// holds while the grid stays where it is and is not deinitialized, and
    /// `clear` and `addItem` change what it shows.
    pub fn getBounds(self: *const ProximityGrid) *const [4]i32 {
        return &self.bounds;
    }

    /// The width of a cell, as `init` was given it.
    pub fn getCellSize(self: *const ProximityGrid) f32 {
        return self.cell_size;
    }
};
