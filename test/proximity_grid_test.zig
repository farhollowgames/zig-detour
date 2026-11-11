//! Differential tests of the proximity grid: the same items go into the
//! port's grid and into the original's, and every query must then give the
//! same ids in the same order, every cell the same entry count and the grid
//! the same bounds.

const std = @import("std");
const builtin = @import("builtin");
const detour = @import("detour");
const support = @import("support.zig");
const reference = support.reference_crowd;

const testing = std.testing;

const ProximityGrid = detour.ProximityGrid;

/// The room of the id buffers a query writes to. The random tests add fewer
/// items than this to a grid, so a query given all of it never runs out.
const id_count_max = 128;

/// What an id buffer holds before a query, to show what the query left alone.
const untouched_id: u16 = 0xa5a5;

/// A box in grid coordinates, as `addItem` and `queryItems` take one.
const Box = struct {
    x_min: f32,
    y_min: f32,
    x_max: f32,
    y_max: f32,
};

/// One grid on each side, set up alike and given the same calls.
const GridPair = struct {
    grid: ProximityGrid,
    reference_grid: *reference.ProximityGrid,

    fn init(gpa: std.mem.Allocator, pool_size: i32, cell_size: f32) !GridPair {
        const reference_grid = reference.ref_prox_grid_alloc() orelse return error.OutOfMemory;
        errdefer reference.ref_prox_grid_free(reference_grid);
        try testing.expect(reference.ref_prox_grid_init(reference_grid, pool_size, cell_size));

        var grid: ProximityGrid = .empty;
        try testing.expect(grid.init(gpa, pool_size, cell_size));
        return .{ .grid = grid, .reference_grid = reference_grid };
    }

    fn deinit(pair: *GridPair, gpa: std.mem.Allocator) void {
        pair.grid.deinit(gpa);
        reference.ref_prox_grid_free(pair.reference_grid);
    }

    fn clear(pair: *GridPair) void {
        pair.grid.clear();
        reference.ref_prox_grid_clear(pair.reference_grid);
    }

    fn addItem(pair: *GridPair, id: u16, box: Box) void {
        pair.grid.addItem(id, box.x_min, box.y_min, box.x_max, box.y_max);
        reference.ref_prox_grid_add_item(
            pair.reference_grid,
            id,
            box.x_min,
            box.y_min,
            box.x_max,
            box.y_max,
        );
    }

    /// Fails unless both grids report the same bounds and the same cell size.
    fn expectSameBounds(pair: *const GridPair) !void {
        const expected = reference.ref_prox_grid_get_bounds(pair.reference_grid);
        const actual = pair.grid.getBounds();
        if (!std.mem.eql(i32, expected, actual)) {
            std.debug.print("getBounds: original {any}, port {any}\n", .{ expected.*, actual.* });
            return error.TestExpectedEqual;
        }
        try support.expectFloat(
            reference.ref_prox_grid_get_cell_size(pair.reference_grid),
            pair.grid.getCellSize(),
        );
    }

    /// Fails unless cell `x`, `y` holds the same number of entries on both
    /// sides. Returns that number.
    fn expectSameCountAt(pair: *const GridPair, x: i32, y: i32) !i32 {
        const expected = reference.ref_prox_grid_get_item_count_at(pair.reference_grid, x, y);
        const actual = pair.grid.getItemCountAt(x, y);
        if (expected != actual) {
            std.debug.print(
                "getItemCountAt({d}, {d}): original {d}, port {d}\n",
                .{ x, y, expected, actual },
            );
            return error.TestExpectedEqual;
        }
        return actual;
    }

    /// Fails unless every cell from `x_min`, `y_min` to `x_max`, `y_max`
    /// holds the same number of entries on both sides. Returns the entries
    /// of all those cells together.
    fn expectSameCounts(
        pair: *const GridPair,
        x_min: i32,
        y_min: i32,
        x_max: i32,
        y_max: i32,
    ) !i32 {
        var total: i32 = 0;
        var y = y_min;
        while (y <= y_max) : (y += 1) {
            var x = x_min;
            while (x <= x_max) : (x += 1) {
                total += try pair.expectSameCountAt(x, y);
            }
        }
        return total;
    }

    /// Fails unless a query of `box` with room for `capacity` ids writes the
    /// same ids in the same order on both sides, and nothing past them.
    /// Returns how many ids it wrote.
    fn expectSameQuery(pair: *const GridPair, box: Box, capacity: usize) !usize {
        var ids: [id_count_max]u16 = @splat(untouched_id);
        var reference_ids: [id_count_max]u16 = @splat(untouched_id);
        errdefer std.debug.print("queryItems: {any}, room for {d} ids\n", .{ box, capacity });

        const expected = reference.ref_prox_grid_query_items(
            pair.reference_grid,
            box.x_min,
            box.y_min,
            box.x_max,
            box.y_max,
            &reference_ids,
            @intCast(capacity),
        );
        const actual = pair.grid.queryItems(
            box.x_min,
            box.y_min,
            box.x_max,
            box.y_max,
            ids[0..capacity],
        );
        try testing.expectEqual(expected, actual);

        // The whole buffers, so a write past the count shows too.
        for (reference_ids, ids, 0..) |expected_id, actual_id, i| {
            if (expected_id != actual_id) {
                std.debug.print(
                    "id {d} of {d}: original {d}, port {d}\n",
                    .{ i, actual, expected_id, actual_id },
                );
                return error.TestExpectedEqual;
            }
        }
        return @intCast(actual);
    }
};

/// An item id: either end of the range, one of a handful many items then
/// share, or any id.
fn randomId(random: std.Random) u16 {
    return switch (random.uintLessThan(u32, 8)) {
        0 => 0,
        1 => 0xffff,
        2, 3, 4 => random.uintLessThan(u16, 12),
        else => random.int(u16),
    };
}

/// One axis of a random box, low coordinate first: it starts within `reach`
/// cells of the origin and is up to `span_max` cells long. Now and then it
/// has no length, is inside out (the box then covers no cell) or starts at a
/// zero of either sign.
fn randomInterval(random: std.Random, cell_size: f32, reach: f32, span_max: f32) [2]f32 {
    const cell = (random.float(f32) * 2.0 - 1.0) * reach;
    const span = random.float(f32) * span_max;
    const low = cell * cell_size;
    const high = (cell + span) * cell_size;
    return switch (random.uintLessThan(u32, 24)) {
        0 => .{ low, low },
        1 => .{ high, low },
        2 => .{ 0.0, high },
        3 => .{ -0.0, high },
        else => .{ low, high },
    };
}

/// A random box near the origin. `reach` and `span_max` are in cells, which
/// bounds the cells both sides walk for it whatever the cell size.
fn randomBox(random: std.Random, cell_size: f32, reach: f32, span_max: f32) Box {
    const x = randomInterval(random, cell_size, reach, span_max);
    const y = randomInterval(random, cell_size, reach, span_max);
    return .{ .x_min = x[0], .y_min = y[0], .x_max = x[1], .y_max = y[1] };
}

/// The coordinate in the middle of cell `cell`, for cells near the origin.
fn cellMiddle(cell_size: f32, cell: i32) f32 {
    return (@as(f32, @floatFromInt(cell)) + 0.5) * cell_size;
}

/// The box that covers exactly the cells from `x_min`, `y_min` to `x_max`,
/// `y_max`, for cells near the origin.
fn cellBox(cell_size: f32, x_min: i32, y_min: i32, x_max: i32, y_max: i32) Box {
    return .{
        .x_min = cellMiddle(cell_size, x_min),
        .y_min = cellMiddle(cell_size, y_min),
        .x_max = cellMiddle(cell_size, x_max),
        .y_max = cellMiddle(cell_size, y_max),
    };
}

/// Cell coordinates at the ends of the integer range and around the edges
/// of the 16 bits the grid stores.
const notable_cells = [_]i32{
    0,
    -1,
    1,
    32767,
    32768,
    -32768,
    -32769,
    65535,
    65536,
    -65536,
    -65537,
    16777216,
    std.math.maxInt(i32),
    std.math.minInt(i32),
};

/// A cell coordinate from anywhere in the integer range, the notable ones
/// more often than chance gives them.
fn randomCell(random: std.Random) i32 {
    return switch (random.uintLessThan(u32, 4)) {
        0 => notable_cells[random.uintLessThan(usize, notable_cells.len)],
        1 => random.intRangeAtMost(i32, -40, 40),
        2 => random.intRangeAtMost(i32, -70000, 70000),
        else => random.int(i32),
    };
}

test "init, getCellSize and getBounds: a new grid is empty" {
    const gpa = testing.allocator;

    const pool_sizes = [_]i32{ 1, 2, 3, 7, 16, 100, 257, 4096, 65535, 65536, 65537, 70000 };
    const cell_sizes = [_]f32{ 0.001, 0.25, 0.3, 1.0, 1.7, 64.0, 1.0e6 };
    for (pool_sizes) |pool_size| {
        for (cell_sizes) |cell_size| {
            errdefer std.debug.print("pool of {d}, cells of {d}\n", .{ pool_size, cell_size });
            var pair = try GridPair.init(gpa, pool_size, cell_size);
            defer pair.deinit(gpa);

            try pair.expectSameBounds();
            try testing.expectEqual(0, try pair.expectSameCounts(-4, -4, 4, 4));
            for (notable_cells) |x| {
                for (notable_cells) |y| _ = try pair.expectSameCountAt(x, y);
            }

            const everything = cellBox(cell_size, -4, -4, 4, 4);
            try testing.expectEqual(0, try pair.expectSameQuery(everything, id_count_max));
            try testing.expectEqual(0, try pair.expectSameQuery(everything, 0));
        }
    }
}

test "addItem and queryItems: one item" {
    const gpa = testing.allocator;

    // Boxes in cells: low x, low y, high x, high y.
    const boxes = [_][4]f32{
        // Inside one cell, and a point.
        .{ 0.25, 0.25, 0.75, 0.75 },
        .{ 2.5, -3.5, 2.5, -3.5 },
        // Corners on cell edges, where rounding decides the cell.
        .{ 1.0, 1.0, 2.0, 2.0 },
        .{ -1.0, -1.0, 0.0, 0.0 },
        .{ -3.0, 2.0, -3.0, 2.0 },
        // Across the origin, and zeros of either sign.
        .{ -0.5, -0.5, 0.5, 0.5 },
        .{ -0.0, -0.0, 0.5, 0.5 },
        .{ 0.0, 0.0, -0.0, -0.0 },
        // Inside out along x, along y and along both: no cell.
        .{ 1.5, 0.5, 0.5, 0.5 },
        .{ 0.5, 1.5, 2.5, 0.5 },
        .{ 2.5, 2.5, -2.5, -2.5 },
        // A row, a column and a block.
        .{ -6.5, 0.5, 6.5, 0.5 },
        .{ 0.5, -6.5, 0.5, 6.5 },
        .{ -2.5, -2.5, 2.5, 2.5 },
    };
    const pool_sizes = [_]i32{ 1, 4, 64 };
    const cell_sizes = [_]f32{ 0.25, 0.3, 1.0, 1.7, 64.0 };
    const capacities = [_]usize{ 0, 1, 2, id_count_max };

    for (pool_sizes) |pool_size| {
        for (cell_sizes) |cell_size| {
            for (boxes, 0..) |cells, box_index| {
                errdefer std.debug.print(
                    "pool of {d}, cells of {d}, box {d}\n",
                    .{ pool_size, cell_size, box_index },
                );
                var pair = try GridPair.init(gpa, pool_size, cell_size);
                defer pair.deinit(gpa);

                const box: Box = .{
                    .x_min = cells[0] * cell_size,
                    .y_min = cells[1] * cell_size,
                    .x_max = cells[2] * cell_size,
                    .y_max = cells[3] * cell_size,
                };
                pair.addItem(42, box);
                try pair.expectSameBounds();
                _ = try pair.expectSameCounts(-9, -9, 9, 9);

                for (capacities) |capacity| {
                    _ = try pair.expectSameQuery(box, capacity);
                    _ = try pair.expectSameQuery(cellBox(cell_size, -9, -9, 9, 9), capacity);
                }

                // Each cell around the item by itself.
                var y: i32 = -8;
                while (y <= 8) : (y += 1) {
                    var x: i32 = -8;
                    while (x <= 8) : (x += 1) {
                        _ = try pair.expectSameQuery(cellBox(cell_size, x, y, x, y), 1);
                    }
                }
            }
        }
    }
}

test "queryItems: a buffer one short of the ids, exact and one over" {
    const gpa = testing.allocator;

    const item_counts = [_]usize{ 1, 2, 3, 8, 33, id_count_max - 2 };
    for (item_counts) |item_count| {
        errdefer std.debug.print("{d} items\n", .{item_count});
        var pair = try GridPair.init(gpa, 1024, 1.0);
        defer pair.deinit(gpa);

        // Items with ids all different, several to a cell of a 3 by 3 block,
        // and one over the whole block under an id already there, which a
        // query reports once.
        for (0..item_count) |i| {
            const x: i32 = @intCast(i % 3);
            const y: i32 = @intCast((i / 3) % 3);
            pair.addItem(@intCast(1000 + i), cellBox(1.0, x, y, x, y));
        }
        pair.addItem(1000, cellBox(1.0, 0, 0, 2, 2));

        const block = cellBox(1.0, -1, -1, 3, 3);
        const all = try pair.expectSameQuery(block, id_count_max);
        try testing.expectEqual(item_count, all);

        // Room for fewer ids than there are ends the query at a full buffer.
        for (0..all + 2) |capacity| {
            const written = try pair.expectSameQuery(block, capacity);
            try testing.expectEqual(@min(capacity, all), written);
        }
    }
}

/// Compares queries over the cells from -4, -4 to 12, 12: all of them at
/// once into buffers of several sizes, then each row and each cell.
fn expectSameQueriesAround(pair: *const GridPair, cell_size: f32) !void {
    for ([_]usize{ 0, 1, 2, id_count_max }) |capacity| {
        _ = try pair.expectSameQuery(cellBox(cell_size, -4, -4, 12, 12), capacity);
    }
    var y: i32 = -4;
    while (y <= 12) : (y += 1) {
        _ = try pair.expectSameQuery(cellBox(cell_size, -4, y, 12, y), id_count_max);
        var x: i32 = -4;
        while (x <= 12) : (x += 1) {
            _ = try pair.expectSameQuery(cellBox(cell_size, x, y, x, y), id_count_max);
        }
    }
}

test "addItem: a full pool leaves the remaining cells out" {
    const gpa = testing.allocator;

    // One item over a 10 by 10 block of cells, into pools with fewer entries
    // than the block has cells, exactly as many, and more.
    const pool_sizes = [_]i32{ 1, 2, 5, 37, 64, 99, 100, 101, 108, 109, 110, 128 };
    const cell_sizes = [_]f32{ 0.5, 1.0, 2.5 };
    for (pool_sizes) |pool_size| {
        for (cell_sizes) |cell_size| {
            errdefer std.debug.print("pool of {d}, cells of {d}\n", .{ pool_size, cell_size });
            var pair = try GridPair.init(gpa, pool_size, cell_size);
            defer pair.deinit(gpa);

            pair.addItem(11, cellBox(cell_size, 0, 0, 9, 9));
            try pair.expectSameBounds();
            var entry_count = try pair.expectSameCounts(-4, -4, 12, 12);
            try testing.expectEqual(@min(pool_size, 100), entry_count);
            try expectSameQueriesAround(&pair, cell_size);

            // A second item, of 9 cells, finds the pool full, nearly full or
            // with room.
            pair.addItem(12, cellBox(cell_size, -2, -2, 0, 0));
            try pair.expectSameBounds();
            entry_count = try pair.expectSameCounts(-4, -4, 12, 12);
            try testing.expectEqual(@min(pool_size, 109), entry_count);
            try expectSameQueriesAround(&pair, cell_size);

            // A third: by now most pools take nothing, yet the bounds grow.
            pair.addItem(13, cellBox(cell_size, 11, 11, 12, 12));
            try pair.expectSameBounds();
            entry_count = try pair.expectSameCounts(-4, -4, 12, 12);
            try testing.expectEqual(@min(pool_size, 113), entry_count);
            try expectSameQueriesAround(&pair, cell_size);
        }
    }
}

test "addItem: the last entry of a 65536-entry pool" {
    const gpa = testing.allocator;
    const seed: u64 = 0x65536;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    // Entries name each other by 16-bit index, and 0xffff also ends a chain.
    // The last entry of this pool has that index, so adding it makes its
    // bucket read as empty on both sides.
    var pair = try GridPair.init(gpa, 65536, 1.0);
    defer pair.deinit(gpa);

    // One entry to each cell of a 256 by 256 block, all but the last.
    const side = 256;
    const entry_count = side * side;
    var index: i32 = 0;
    while (index < entry_count - 1) : (index += 1) {
        const x = @rem(index, side) - side / 2;
        const y = @divTrunc(index, side) - side / 2;
        pair.addItem(random.int(u16), cellBox(1.0, x, y, x, y));
    }
    try pair.expectSameBounds();
    try testing.expectEqual(entry_count - 1, try pair.expectSameCounts(-130, -130, 130, 130));
    for (0..300) |query| {
        errdefer std.debug.print("  seed {d}, query {d} before the last entry\n", .{ seed, query });
        const box = randomBox(random, 1.0, 128.0, 3.0);
        _ = try pair.expectSameQuery(box, id_count_max);
        _ = try pair.expectSameQuery(box, random.uintLessThan(usize, 8));
    }

    // The last entry hides itself and the entries that share its bucket.
    const last = side / 2 - 1;
    pair.addItem(random.int(u16), cellBox(1.0, last, last, last, last));
    try pair.expectSameBounds();
    try testing.expect(try pair.expectSameCounts(-130, -130, 130, 130) < entry_count);
    for (0..300) |query| {
        errdefer std.debug.print("  seed {d}, query {d} after the last entry\n", .{ seed, query });
        const box = randomBox(random, 1.0, 128.0, 3.0);
        _ = try pair.expectSameQuery(box, id_count_max);
        _ = try pair.expectSameQuery(box, random.uintLessThan(usize, 8));
    }
    const around_last = cellBox(1.0, last - 1, last - 1, last + 1, last + 1);
    for ([_]usize{ 0, 1, id_count_max }) |capacity| {
        _ = try pair.expectSameQuery(around_last, capacity);
    }

    // The pool is full: a further item moves the bounds and nothing else.
    pair.addItem(9, cellBox(1.0, 200, 200, 201, 201));
    try pair.expectSameBounds();
    _ = try pair.expectSameCounts(198, 198, 203, 203);
    _ = try pair.expectSameCounts(-130, -130, 130, 130);
    _ = try pair.expectSameQuery(cellBox(1.0, 198, 198, 203, 203), id_count_max);
}

test "addItem and queryItems: one item beyond 16 bits of cells" {
    const gpa = testing.allocator;

    // The grid stores an entry's cell cut to 16 bits, so an item at cell
    // 40000 is never found there. Cell -25536, which the cut bits spell,
    // finds it while the bucket table has at most 65536 buckets, since the
    // two cells then share a bucket.
    const own_cell = 40000;
    const cut_cell = own_cell - 65536;
    const pool_sizes = [_]i32{ 1, 16, 4096, 65536, 65537, 140000 };
    for (pool_sizes) |pool_size| {
        errdefer std.debug.print("pool of {d}\n", .{pool_size});
        var pair = try GridPair.init(gpa, pool_size, 1.0);
        defer pair.deinit(gpa);

        pair.addItem(7, cellBox(1.0, own_cell, 3, own_cell, 3));
        try pair.expectSameBounds();

        try testing.expectEqual(0, try pair.expectSameCountAt(own_cell, 3));
        const cut_count = try pair.expectSameCountAt(cut_cell, 3);
        if (pool_size <= 65536) try testing.expectEqual(1, cut_count);
        _ = try pair.expectSameCounts(own_cell - 2, 1, own_cell + 2, 5);
        _ = try pair.expectSameCounts(cut_cell - 2, 1, cut_cell + 2, 5);
        _ = try pair.expectSameCountAt(own_cell + 65536, 3);
        _ = try pair.expectSameCountAt(own_cell - 131072, 3);

        for ([_]usize{ 0, 1, id_count_max }) |capacity| {
            _ = try pair.expectSameQuery(cellBox(1.0, own_cell, 3, own_cell, 3), capacity);
            _ = try pair.expectSameQuery(cellBox(1.0, cut_cell, 3, cut_cell, 3), capacity);
            _ = try pair.expectSameQuery(
                cellBox(1.0, own_cell - 2, 1, own_cell + 2, 5),
                capacity,
            );
            _ = try pair.expectSameQuery(
                cellBox(1.0, cut_cell - 2, 1, cut_cell + 2, 5),
                capacity,
            );
        }
    }
}

test "addItem and queryItems: random items, every query's ids in order" {
    const gpa = testing.allocator;
    const seed: u64 = 0x9e3779b97f4a7c15;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    const pool_sizes = [_]i32{ 1, 2, 3, 7, 16, 100, 257, 1000, 4096, 5000 };
    const cell_sizes = [_]f32{ 0.25, 0.3, 1.0, 1.7, 3.0, 10.0, 64.0 };

    var cut_short_count: u32 = 0;
    for (0..1200) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const pool_size = pool_sizes[case % pool_sizes.len];
        const cell_size = cell_sizes[random.uintLessThan(usize, cell_sizes.len)];
        var pair = try GridPair.init(gpa, pool_size, cell_size);
        defer pair.deinit(gpa);

        const reach = random.intRangeAtMost(i32, 1, 12);
        const reach_cells: f32 = @floatFromInt(reach);
        const item_count = random.uintLessThan(u32, 80);
        for (0..item_count) |_| {
            pair.addItem(randomId(random), randomBox(random, cell_size, reach_cells, 4.0));
            if (random.uintLessThan(u32, 8) == 0) try pair.expectSameBounds();
        }
        try pair.expectSameBounds();
        _ = try pair.expectSameCounts(-reach - 6, -reach - 6, reach + 6, reach + 6);

        for (0..20) |_| {
            const box = randomBox(random, cell_size, reach_cells + 2.0, 6.0);
            const all = try pair.expectSameQuery(box, id_count_max);

            // A buffer one short of the ids, then one of any size up to
            // theirs: the first always ends the query at a full buffer.
            if (all > 0) {
                _ = try pair.expectSameQuery(box, all - 1);
                cut_short_count += 1;
            }
            _ = try pair.expectSameQuery(box, random.uintAtMost(usize, all));
        }
    }
    try testing.expect(cut_short_count > 0);
}

test "getItemCountAt: stacked items, and cells anywhere in the integer range" {
    const gpa = testing.allocator;
    const seed: u64 = 0x636f756e744174;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    const pool_sizes = [_]i32{ 1, 4, 9, 64, 300, 2000 };
    const cell_sizes = [_]f32{ 0.25, 0.3, 1.0, 1.7, 64.0 };
    // What separates two cells whose 16-bit cuts are the same.
    const wraps = [_]i32{ 65536, -65536, 131072 };

    var count_max: i32 = 0;
    for (0..1000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const pool_size = pool_sizes[case % pool_sizes.len];
        const cell_size = cell_sizes[random.uintLessThan(usize, cell_sizes.len)];
        var pair = try GridPair.init(gpa, pool_size, cell_size);
        defer pair.deinit(gpa);

        // Two entries on one cell to start with, then items crowded onto few
        // cells under few ids: a cell holds several entries, some of one id.
        pair.addItem(1, cellBox(cell_size, 0, 0, 0, 0));
        pair.addItem(1, cellBox(cell_size, 0, 0, 0, 0));
        const item_count = random.uintLessThan(u32, 100);
        for (0..item_count) |_| {
            pair.addItem(random.uintLessThan(u16, 6), randomBox(random, cell_size, 2.5, 2.0));
        }

        var y: i32 = -7;
        while (y <= 7) : (y += 1) {
            var x: i32 = -7;
            while (x <= 7) : (x += 1) {
                const count = try pair.expectSameCountAt(x, y);
                count_max = @max(count_max, count);
                if (count == 0) continue;

                // The cells an entry here could be mistaken for.
                for (wraps) |wrap| {
                    _ = try pair.expectSameCountAt(x + wrap, y);
                    _ = try pair.expectSameCountAt(x, y + wrap);
                    _ = try pair.expectSameCountAt(x + wrap, y + wrap);
                }
            }
        }
        for (0..100) |_| {
            _ = try pair.expectSameCountAt(randomCell(random), randomCell(random));
        }
        _ = try pair.expectSameCountAt(randomCell(random), random.intRangeAtMost(i32, -7, 7));
        _ = try pair.expectSameCountAt(random.intRangeAtMost(i32, -7, 7), randomCell(random));
    }
    try testing.expect(count_max > 1);
}

test "addItem: random items into a pool too small for them" {
    const gpa = testing.allocator;
    const seed: u64 = 0x706f6f6c46756c6c;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    const cell_sizes = [_]f32{ 0.25, 0.3, 1.0, 1.7, 64.0 };

    var full_count: u32 = 0;
    for (0..1000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const pool_size = random.intRangeAtMost(i32, 1, 48);
        const cell_size = cell_sizes[random.uintLessThan(usize, cell_sizes.len)];
        var pair = try GridPair.init(gpa, pool_size, cell_size);
        defer pair.deinit(gpa);

        // The bounds keep growing once the pool is full, so they are
        // compared after every item.
        const item_count = random.intRangeAtMost(u32, 1, 60);
        for (0..item_count) |_| {
            pair.addItem(randomId(random), randomBox(random, cell_size, 5.0, 5.0));
            try pair.expectSameBounds();
        }
        if (try pair.expectSameCounts(-12, -12, 12, 12) == pool_size) full_count += 1;

        for (0..16) |_| {
            const box = randomBox(random, cell_size, 7.0, 6.0);
            const all = try pair.expectSameQuery(box, id_count_max);
            if (all > 0) _ = try pair.expectSameQuery(box, all - 1);
            _ = try pair.expectSameQuery(box, random.uintAtMost(usize, all));
        }
    }
    try testing.expect(full_count > 0);
}

/// Adds random items to both grids of `pair`, then compares the bounds, the
/// entries of every cell the items can reach and a few queries.
fn expectSameAfterRandomItems(pair: *GridPair, random: std.Random, cell_size: f32) !void {
    const item_count = random.uintLessThan(u32, 40);
    for (0..item_count) |_| {
        pair.addItem(randomId(random), randomBox(random, cell_size, 5.0, 3.0));
    }
    try pair.expectSameBounds();
    _ = try pair.expectSameCounts(-10, -10, 10, 10);
    for (0..6) |_| {
        const box = randomBox(random, cell_size, 7.0, 5.0);
        const all = try pair.expectSameQuery(box, id_count_max);
        _ = try pair.expectSameQuery(box, random.uintAtMost(usize, all));
    }
}

test "clear: a cleared grid is empty and fills again" {
    const gpa = testing.allocator;
    const seed: u64 = 0x636c656172;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    const pool_sizes = [_]i32{ 1, 5, 64, 500 };
    const cell_sizes = [_]f32{ 0.25, 0.3, 1.0, 1.7, 64.0 };

    for (0..1000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const pool_size = pool_sizes[case % pool_sizes.len];
        const cell_size = cell_sizes[random.uintLessThan(usize, cell_sizes.len)];
        var pair = try GridPair.init(gpa, pool_size, cell_size);
        defer pair.deinit(gpa);

        // Clearing a grid nothing was added to changes nothing.
        if (random.boolean()) {
            pair.clear();
            try pair.expectSameBounds();
        }

        for (0..3) |_| {
            try expectSameAfterRandomItems(&pair, random, cell_size);
            pair.clear();
            if (random.uintLessThan(u32, 4) == 0) pair.clear();

            try pair.expectSameBounds();
            try testing.expectEqual(0, try pair.expectSameCounts(-10, -10, 10, 10));
            const everything = cellBox(cell_size, -10, -10, 10, 10);
            try testing.expectEqual(0, try pair.expectSameQuery(everything, id_count_max));
        }

        // The cleared grid takes items as a new one does.
        try expectSameAfterRandomItems(&pair, random, cell_size);
    }
}

test "init: the bucket table follows the pool size" {
    const gpa = testing.allocator;
    const seed: u64 = 0x6275636b657473;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    // The original sizes its bucket table to the pool size rounded up to a
    // power of two. Each case puts the same items into two grids whose pools
    // differ in size, and asks both the same queries.
    const pool_sizes = [_]i32{ 1, 3, 100, 1000, 1025, 5000, 20000, 40000, 65535, 70000 };
    const cell_sizes = [_]f32{ 0.25, 0.3, 1.0, 1.7, 64.0 };
    const Item = struct { id: u16, box: Box };

    for (0..1000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const cell_size = cell_sizes[random.uintLessThan(usize, cell_sizes.len)];

        var items: [48]Item = undefined;
        const item_count = random.uintAtMost(usize, items.len);
        for (items[0..item_count]) |*item| {
            item.* = .{ .id = randomId(random), .box = randomBox(random, cell_size, 6.0, 3.0) };
        }
        var queries: [12]Box = undefined;
        for (&queries) |*query| query.* = randomBox(random, cell_size, 8.0, 5.0);

        const first = case % pool_sizes.len;
        const step = 1 + (case / pool_sizes.len) % (pool_sizes.len - 1);
        for ([_]usize{ first, (first + step) % pool_sizes.len }) |pool_index| {
            const pool_size = pool_sizes[pool_index];
            errdefer std.debug.print("  pool of {d}\n", .{pool_size});
            var pair = try GridPair.init(gpa, pool_size, cell_size);
            defer pair.deinit(gpa);

            for (items[0..item_count]) |item| pair.addItem(item.id, item.box);
            try pair.expectSameBounds();
            _ = try pair.expectSameCounts(-12, -12, 12, 12);
            for (queries) |box| {
                const all = try pair.expectSameQuery(box, id_count_max);
                if (all > 0) _ = try pair.expectSameQuery(box, all - 1);
                _ = try pair.expectSameQuery(box, all / 2);
            }
        }
    }
}

/// Cell numbers at the edge of the 16 bits the grid stores and beyond them,
/// out to the ends of the integer range. Each float holds its number exactly.
const far_cells = [_]f32{
    32766.0,
    32767.0,
    32768.0,
    32769.0,
    40000.0,
    65535.0,
    65536.0,
    65537.0,
    100000.0,
    1.0e6,
    16777216.0,
    1.0e9,
    2147483520.0,
    -32767.0,
    -32768.0,
    -32769.0,
    -40000.0,
    -65535.0,
    -65536.0,
    -65537.0,
    -1.0e6,
    -1.0e9,
    -2147483520.0,
    -2147483648.0,
};

/// A whole cell number as a float: a far one three times in four, else one
/// next to the origin.
fn randomFarCell(random: std.Random) f32 {
    if (random.uintLessThan(u32, 4) == 0) {
        return @floatFromInt(random.intRangeAtMost(i32, -3, 3));
    }
    return far_cells[random.uintLessThan(usize, far_cells.len)];
}

/// The cell number the grid keeps of `cell`: its low 16 bits, signed.
fn storedCell(cell: i32) i32 {
    return @as(i16, @truncate(cell));
}

test "addItem and queryItems: cells beyond 16 bits" {
    const gpa = testing.allocator;
    const seed: u64 = 0x66617243656c6c73;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    // The cell sizes are powers of two, so a coordinate is exactly a cell
    // number times the size, and the numbers below are the cells both grids
    // see. An item covers at most 3 by 3 cells.
    const pool_sizes = [_]i32{ 1, 2, 16, 300, 4096, 65536, 70000 };
    const cell_sizes = [_]f32{ 0.5, 1.0, 4.0 };
    const FarItem = struct {
        box: Box,
        cell_x: i32,
        cell_y: i32,
        x_span: f32,
        y_span: f32,
    };

    var moved_count: u32 = 0;
    for (0..1000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const pool_size = pool_sizes[case % pool_sizes.len];
        const cell_size = cell_sizes[random.uintLessThan(usize, cell_sizes.len)];
        var pair = try GridPair.init(gpa, pool_size, cell_size);
        defer pair.deinit(gpa);

        var items: [24]FarItem = undefined;
        const item_count = random.uintAtMost(usize, items.len);
        for (items[0..item_count]) |*item| {
            const x = randomFarCell(random);
            const y = randomFarCell(random);
            const x_span: f32 = @floatFromInt(random.uintLessThan(u32, 3));
            const y_span: f32 = @floatFromInt(random.uintLessThan(u32, 3));
            item.* = .{
                .box = .{
                    .x_min = x * cell_size,
                    .y_min = y * cell_size,
                    .x_max = (x + x_span) * cell_size,
                    .y_max = (y + y_span) * cell_size,
                },
                .cell_x = @trunc(x),
                .cell_y = @trunc(y),
                .x_span = x_span,
                .y_span = y_span,
            };
            pair.addItem(randomId(random), item.box);
            try pair.expectSameBounds();
        }

        for (items[0..item_count]) |item| {
            // The item's own cells: an entry beyond 16 bits is not found there.
            const own = try pair.expectSameQuery(item.box, id_count_max);
            _ = try pair.expectSameQuery(item.box, random.uintLessThan(usize, 3));

            // The cells the stored 16 bits spell.
            const cut_x: f32 = @floatFromInt(storedCell(item.cell_x));
            const cut_y: f32 = @floatFromInt(storedCell(item.cell_y));
            const cut_box: Box = .{
                .x_min = cut_x * cell_size,
                .y_min = cut_y * cell_size,
                .x_max = (cut_x + item.x_span) * cell_size,
                .y_max = (cut_y + item.y_span) * cell_size,
            };
            const cut = try pair.expectSameQuery(cut_box, id_count_max);
            _ = try pair.expectSameQuery(cut_box, random.uintLessThan(usize, 3));
            if (own == 0 and cut > 0) moved_count += 1;

            for ([_]i32{ -1, 0, 1 }) |y_step| {
                for ([_]i32{ -1, 0, 1 }) |x_step| {
                    _ = try pair.expectSameCountAt(item.cell_x +% x_step, item.cell_y +% y_step);
                    _ = try pair.expectSameCountAt(
                        storedCell(item.cell_x) + x_step,
                        storedCell(item.cell_y) + y_step,
                    );
                }
            }
        }
        _ = try pair.expectSameCounts(-5, -5, 5, 5);
        for (0..40) |_| {
            _ = try pair.expectSameCountAt(randomCell(random), randomCell(random));
        }
    }
    try testing.expect(moved_count > 0);
}

/// Coordinates no cell number can hold, at any cell size these tests use:
/// too large, infinite and not a number.
const beyond_coordinates = [_]f32{
    std.math.inf(f32),
    -std.math.inf(f32),
    std.math.nan(f32),
    1.0e30,
    -1.0e30,
    1.0e12,
    -1.0e12,
    3.0e38,
    -3.0e38,
};

fn randomBeyondCoordinate(random: std.Random) f32 {
    return beyond_coordinates[random.uintLessThan(usize, beyond_coordinates.len)];
}

/// One axis of a box that may reach beyond the cell range, low coordinate
/// first. Both ends are cells, or only the low end is (the box then covers
/// nothing), or neither is (it covers the one cell the lowest integer
/// names). The low end is never beyond the range while the high end is a
/// cell: from the lowest integer up to it is billions of cells to walk.
fn randomIntervalBeyond(random: std.Random, cell_size: f32) [2]f32 {
    const ordinary = randomInterval(random, cell_size, 5.0, 3.0);
    return switch (random.uintLessThan(u32, 4)) {
        0, 1 => ordinary,
        2 => .{ ordinary[0], randomBeyondCoordinate(random) },
        else => .{ randomBeyondCoordinate(random), randomBeyondCoordinate(random) },
    };
}

fn randomBoxBeyond(random: std.Random, cell_size: f32) Box {
    const x = randomIntervalBeyond(random, cell_size);
    const y = randomIntervalBeyond(random, cell_size);
    return .{ .x_min = x[0], .y_min = y[0], .x_max = x[1], .y_max = y[1] };
}

/// Compares the entries of the cells around the origin and of the cells the
/// lowest integer names along either axis, where a coordinate beyond the
/// range puts an item.
fn expectSameCountsBeyond(pair: *const GridPair) !void {
    const lowest = std.math.minInt(i32);
    _ = try pair.expectSameCounts(-10, -10, 10, 10);
    _ = try pair.expectSameCountAt(lowest, lowest);
    var cell: i32 = -10;
    while (cell <= 10) : (cell += 1) {
        _ = try pair.expectSameCountAt(lowest, cell);
        _ = try pair.expectSameCountAt(cell, lowest);
    }
}

test "addItem and queryItems: coordinates beyond the cell range" {
    // C leaves the cast of such a coordinate to a cell number undefined. The
    // original built for x86-64 gives the lowest integer, and the port is
    // written to give the same; elsewhere the original's answer differs.
    if (builtin.cpu.arch != .x86_64) return error.SkipZigTest;

    const gpa = testing.allocator;

    // By hand, in cells 1 wide: the first coordinate past each end of the
    // range, the last one within it, and an item with one end past it.
    const top = 2147483648.0;
    const bottom = -2147483904.0;
    const inf = std.math.inf(f32);
    const nan = std.math.nan(f32);
    const edge_boxes = [_]Box{
        .{ .x_min = top, .y_min = 0.5, .x_max = top, .y_max = 0.5 },
        .{ .x_min = bottom, .y_min = 0.5, .x_max = bottom, .y_max = 0.5 },
        .{ .x_min = 0.5, .y_min = top, .x_max = 0.5, .y_max = top },
        .{ .x_min = 0.5, .y_min = bottom, .x_max = 0.5, .y_max = bottom },
        .{ .x_min = top, .y_min = bottom, .x_max = bottom, .y_max = top },
        .{ .x_min = 2147483520.0, .y_min = 0.5, .x_max = 2147483520.0, .y_max = 0.5 },
        .{ .x_min = -2147483648.0, .y_min = 0.5, .x_max = -2147483648.0, .y_max = 0.5 },
        .{ .x_min = 0.5, .y_min = 0.5, .x_max = top, .y_max = 0.5 },
        .{ .x_min = 0.5, .y_min = 0.5, .x_max = 2.5, .y_max = inf },
        .{ .x_min = nan, .y_min = 0.5, .x_max = nan, .y_max = 2.5 },
        .{ .x_min = 0.5, .y_min = nan, .x_max = 2.5, .y_max = nan },
        .{ .x_min = nan, .y_min = nan, .x_max = nan, .y_max = nan },
        .{ .x_min = inf, .y_min = 0.5, .x_max = -inf, .y_max = 0.5 },
        .{ .x_min = -inf, .y_min = -inf, .x_max = inf, .y_max = inf },
    };
    for ([_]i32{ 1, 16, 70000 }) |pool_size| {
        for (edge_boxes, 0..) |box, box_index| {
            errdefer std.debug.print("pool of {d}, box {d}\n", .{ pool_size, box_index });
            var pair = try GridPair.init(gpa, pool_size, 1.0);
            defer pair.deinit(gpa);

            pair.addItem(42, box);
            pair.addItem(43, cellBox(1.0, 0, 0, 1, 1));
            try pair.expectSameBounds();
            try expectSameCountsBeyond(&pair);
            for ([_]usize{ 0, 1, id_count_max }) |capacity| {
                _ = try pair.expectSameQuery(box, capacity);
                _ = try pair.expectSameQuery(cellBox(1.0, -2, -2, 2, 2), capacity);
            }
        }
    }

    const seed: u64 = 0x6265796f6e64;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    const pool_sizes = [_]i32{ 1, 8, 64, 1000, 70000 };
    const cell_sizes = [_]f32{ 0.25, 0.3, 1.0, 1.7, 64.0 };
    for (0..1000) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const pool_size = pool_sizes[case % pool_sizes.len];
        const cell_size = cell_sizes[random.uintLessThan(usize, cell_sizes.len)];
        var pair = try GridPair.init(gpa, pool_size, cell_size);
        defer pair.deinit(gpa);

        const item_count = random.uintLessThan(u32, 30);
        for (0..item_count) |_| {
            pair.addItem(randomId(random), randomBoxBeyond(random, cell_size));
            try pair.expectSameBounds();
        }
        try expectSameCountsBeyond(&pair);

        for (0..16) |_| {
            const box = randomBoxBeyond(random, cell_size);
            const all = try pair.expectSameQuery(box, id_count_max);
            if (all > 0) _ = try pair.expectSameQuery(box, all - 1);
            _ = try pair.expectSameQuery(box, random.uintAtMost(usize, all));
        }
    }
}
