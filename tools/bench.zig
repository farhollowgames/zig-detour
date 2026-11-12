//! Times the port against the original, one feature per row.
//!
//! `zig build bench` prints a table of medians. `zig build bench -- <text>`
//! runs only the rows whose name contains `<text>`, and `--rounds=<n>` sets
//! how many times each side of a row is timed.

const std = @import("std");
const Io = std.Io;

const groups = .{
    @import("bench_nav_mesh.zig"),
    @import("bench_query.zig"),
    @import("bench_crowd.zig"),
    @import("bench_tile_cache.zig"),
    @import("bench_integer.zig"),
};

/// What a row measures and how its times are shown.
pub const Row = struct {
    /// The feature, as the table names it.
    name: []const u8,
    /// What one timed call does `count` of: "query", "tile", "agent step".
    per: []const u8,
    /// How many of them one timed call does.
    count: usize,
    /// The bytes one timed call goes through, for a row shown in MB/s; zero
    /// for a row shown as time per item.
    bytes: usize = 0,
    /// What the port is timed against.
    baseline: []const u8 = "original",
    /// Whether the two sides must return the same checksum. False where the
    /// two need not agree: the integer variant against the float port.
    same_results: bool = true,
};

/// `function` behind a pointer the compiler cannot see through. Every row
/// calls each library function it times, on either side, through one of
/// these, taken once before its loop: the call is never inlined into the
/// row, and nothing the row knows about the arguments reaches the function,
/// which runs as compiled on its own, the way the original's library does.
pub fn opaqueFn(comptime function: anytype) *const @TypeOf(function) {
    var pointer: *const @TypeOf(function) = &function;
    std.mem.doNotOptimizeAway(&pointer);
    return pointer;
}

/// The table writer and the timing settings every row shares.
pub const Bench = struct {
    io: Io,
    gpa: std.mem.Allocator,
    out: *Io.Writer,
    /// Timed rounds per side of a row.
    rounds: u32 = 21,
    /// Untimed rounds per side before the timed ones.
    warm_up_rounds: u32 = 3,
    /// The least time one timed round lasts, reached by repeating the call.
    round_ns_min: u64 = 20 * std.time.ns_per_ms,
    /// Rows run only when their name contains one of these; all when empty.
    filters: []const []const u8 = &.{},
    /// Rows whose two sides returned different checksums.
    mismatch_count: u32 = 0,

    /// Starts a table under `title`.
    pub fn group(self: *Bench, title: []const u8) !void {
        try self.out.print(
            \\
            \\### {s}
            \\
            \\| feature | per | baseline | port | port / baseline | baseline range | port range |
            \\| --- | --- | ---: | ---: | ---: | --- | --- |
            \\
        , .{title});
        try self.out.flush();
    }

    /// Times `baseline` and `port` on `context` and prints the row.
    ///
    /// Each is `fn (context) u64`, does `row.count` operations, and returns a
    /// checksum that depends on every operation's results: the checksum keeps
    /// the work alive and shows that both sides did the same work. A call
    /// must leave `context` fit for the next call. Where the context type
    /// declares `prepare`, it runs untimed before every timed call.
    pub fn compare(
        self: *Bench,
        row: Row,
        context: anytype,
        comptime baseline: fn (@TypeOf(context)) u64,
        comptime port: fn (@TypeOf(context)) u64,
    ) !void {
        if (!self.selected(row.name)) return;

        const baseline_sum = once(context, baseline);
        const port_sum = once(context, port);
        const differs = row.same_results and baseline_sum != port_sum;
        if (differs) self.mismatch_count += 1;

        const baseline_repeats = self.repeatsFor(context, baseline);
        const port_repeats = self.repeatsFor(context, port);
        for (0..self.warm_up_rounds) |_| {
            _ = self.timeRound(context, baseline, baseline_repeats);
            _ = self.timeRound(context, port, port_repeats);
        }

        const baseline_times = try self.gpa.alloc(f64, self.rounds);
        defer self.gpa.free(baseline_times);
        const port_times = try self.gpa.alloc(f64, self.rounds);
        defer self.gpa.free(port_times);
        // The sides alternate, so a slow moment of the machine falls on both.
        for (baseline_times, port_times) |*baseline_time, *port_time| {
            baseline_time.* = self.timeRound(context, baseline, baseline_repeats);
            port_time.* = self.timeRound(context, port, port_repeats);
        }
        std.mem.sort(f64, baseline_times, {}, std.sort.asc(f64));
        std.mem.sort(f64, port_times, {}, std.sort.asc(f64));

        const baseline_median = baseline_times[baseline_times.len / 2];
        const port_median = port_times[port_times.len / 2];
        try self.out.print("| {s}{s} | {s} | ", .{
            row.name,
            if (differs) " (RESULTS DIFFER)" else "",
            if (row.bytes != 0) "MB/s" else row.per,
        });
        try self.printAmount(row, baseline_median);
        try self.out.writeAll(" | ");
        try self.printAmount(row, port_median);
        try self.out.print(" | {d:.2} | ", .{port_median / baseline_median});
        try self.printRange(row, baseline_times);
        try self.out.writeAll(" | ");
        try self.printRange(row, port_times);
        try self.out.writeAll(" |\n");
        try self.out.flush();
    }

    fn selected(self: *const Bench, name: []const u8) bool {
        if (self.filters.len == 0) return true;
        for (self.filters) |filter| {
            if (std.mem.find(u8, name, filter) != null) return true;
        }
        return false;
    }

    fn once(context: anytype, comptime function: fn (@TypeOf(context)) u64) u64 {
        prepare(context);
        return function(context);
    }

    fn prepare(context: anytype) void {
        const Context = @typeInfo(@TypeOf(context)).pointer.child;
        if (@hasDecl(Context, "prepare")) context.prepare();
    }

    /// How many calls make one round last `round_ns_min`.
    fn repeatsFor(
        self: *Bench,
        context: anytype,
        comptime function: fn (@TypeOf(context)) u64,
    ) u32 {
        prepare(context);
        const start = Io.Clock.awake.now(self.io);
        std.mem.doNotOptimizeAway(function(context));
        const elapsed = start.durationTo(Io.Clock.awake.now(self.io)).nanoseconds;
        const call_ns: u64 = @intCast(@max(elapsed, 1));
        return @intCast(std.math.clamp(self.round_ns_min / call_ns, 1, 1 << 20));
    }

    /// The nanoseconds one item took over `repeats` calls of `function`.
    fn timeRound(
        self: *Bench,
        context: anytype,
        comptime function: fn (@TypeOf(context)) u64,
        repeats: u32,
    ) f64 {
        const Context = @typeInfo(@TypeOf(context)).pointer.child;
        var total_ns: i96 = 0;
        if (@hasDecl(Context, "prepare")) {
            for (0..repeats) |_| {
                context.prepare();
                const start = Io.Clock.awake.now(self.io);
                std.mem.doNotOptimizeAway(function(context));
                total_ns += start.durationTo(Io.Clock.awake.now(self.io)).nanoseconds;
            }
        } else {
            const start = Io.Clock.awake.now(self.io);
            for (0..repeats) |_| std.mem.doNotOptimizeAway(function(context));
            total_ns = start.durationTo(Io.Clock.awake.now(self.io)).nanoseconds;
        }
        return @as(f64, @floatFromInt(total_ns)) / @as(f64, @floatFromInt(repeats));
    }

    /// Prints the time of one call as the row shows it: per item, or as the
    /// megabytes a second it stands for.
    fn printAmount(self: *Bench, row: Row, call_ns: f64) !void {
        if (row.bytes != 0) {
            const megabytes = @as(f64, @floatFromInt(row.bytes)) / (1024.0 * 1024.0);
            return self.out.print("{d:.0}", .{megabytes / (call_ns / std.time.ns_per_s)});
        }
        const item_ns = call_ns / @as(f64, @floatFromInt(row.count));
        if (item_ns < 100.0) return self.out.print("{d:.1} ns", .{item_ns});
        if (item_ns < 10_000.0) return self.out.print("{d:.0} ns", .{item_ns});
        if (item_ns < 10_000_000.0) return self.out.print("{d:.1} us", .{item_ns / 1000.0});
        return self.out.print("{d:.1} ms", .{item_ns / 1_000_000.0});
    }

    /// Prints the fastest and slowest round of `sorted_times`.
    fn printRange(self: *Bench, row: Row, sorted_times: []const f64) !void {
        // A row in MB/s is fastest at its shortest time.
        const first = if (row.bytes != 0) sorted_times.len - 1 else 0;
        const last = sorted_times.len - 1 - first;
        try self.printAmount(row, sorted_times[first]);
        try self.out.writeAll(" to ");
        try self.printAmount(row, sorted_times[last]);
    }
};

pub fn main(init: std.process.Init) !void {
    var out_buffer: [4096]u8 = undefined;
    var out_writer = Io.File.stdout().writer(init.io, &out_buffer);

    var bench: Bench = .{ .io = init.io, .gpa = init.gpa, .out = &out_writer.interface };
    var filters: std.ArrayList([]const u8) = .empty;
    const arena = init.arena.allocator();
    const args = try init.minimal.args.toSlice(arena);
    for (args[1..]) |arg| {
        if (std.mem.cutPrefix(u8, arg, "--rounds=")) |text| {
            bench.rounds = @max(1, try std.fmt.parseInt(u32, text, 10));
        } else {
            try filters.append(arena, arg);
        }
    }
    bench.filters = filters.items;

    inline for (groups) |group| try group.run(&bench);
    try bench.out.flush();

    if (bench.mismatch_count != 0) {
        std.log.err("{d} rows returned different results on the two sides", .{
            bench.mismatch_count,
        });
        return error.ResultsDiffer;
    }
}
