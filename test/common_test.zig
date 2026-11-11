//! Differential tests of the shared math and geometry helpers: every public
//! function of common.zig against its upstream twin, bit for bit.
//!
//! Each function gets hand-made cases (zeros of both signs, equal points,
//! degenerate polygons, points on edges and corners, the ends of every
//! range) and a fuzz test of random cases from a fixed seed. Outputs are
//! compared whole, and the original's outputs start as a sentinel so that
//! what a call leaves untouched is compared too.

const std = @import("std");
const detour = @import("detour");
const support = @import("support.zig");
const reference = support.reference;

const testing = std.testing;

const common = detour.common;

/// The random cases each fuzz test runs.
const fuzz_cases = 3000;

/// What outputs start as on both sides, so that a write on one side only
/// shows when the two are compared.
const sentinel_bits: u32 = 0xdeadbeef;

/// Floats that comparisons and arithmetic treat specially, as bit patterns so
/// that the compiler makes no NaN of its own: zeros of both signs, small whole
/// numbers, thresholds the functions test against, the ends of the range,
/// subnormals, infinities, NaNs of both signs with and without a payload, a
/// signalling NaN, and the limits of `i32`.
const awkward_bits = [_]u32{
    0x00000000, // 0
    0x80000000, // -0
    0x3f800000, // 1
    0xbf800000, // -1
    0x3f000000, // 0.5
    0xbf000000, // -0.5
    0x40000000, // 2
    0xc0000000, // -2
    0x40400000, // 3
    0x40490fdb, // pi
    0x358637bd, // 1e-6
    0x38800000, // 1/16384
    0x00000001, // the smallest subnormal
    0x80000001, // minus the smallest subnormal
    0x007fffff, // the largest subnormal
    0x00800000, // the smallest normal
    0x7f7fffff, // the largest finite float
    0xff7fffff, // the smallest finite float
    0x7f800000, // infinity
    0xff800000, // minus infinity
    0x7fc00000, // a quiet NaN
    0xffc00000, // a quiet NaN with the sign set
    0x7fc12345, // a quiet NaN with a payload
    0x7f800001, // a signalling NaN
    0x4f000000, // 2^31
    0xcf000000, // -2^31
};

// ---------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------

/// The bits of a float, for messages.
fn bitsOf(x: f32) u32 {
    return @bitCast(x);
}

/// The float with these bits.
fn floatFrom(bits: u32) f32 {
    return @bitCast(bits);
}

/// The float the outputs of both sides start as.
fn sentinel() f32 {
    return floatFrom(sentinel_bits);
}

/// An output array of `n` floats, all the sentinel.
fn sentinelArray(comptime n: usize) [n]f32 {
    var array: [n]f32 = undefined;
    @memset(&array, sentinel());
    return array;
}

/// `x`, read back through memory the optimizer cannot see through. A NaN that
/// arithmetic makes from it is then the machine's own, as in the original,
/// and not one the compiler folded at build time.
fn runtimeFloat(x: f32) f32 {
    var copy = x;
    const through: *volatile f32 = &copy;
    return through.*;
}

/// `n`, hidden from the optimizer as `runtimeFloat` hides a float.
fn runtimeCount(n: usize) usize {
    var copy = n;
    const through: *volatile usize = &copy;
    return through.*;
}

/// `v` with each coordinate hidden from the optimizer.
fn runtimeVector(v: [3]f32) [3]f32 {
    return .{ runtimeFloat(v[0]), runtimeFloat(v[1]), runtimeFloat(v[2]) };
}

/// A float from -1 up to, not including, 1.
fn unit(random: std.Random) f32 {
    return random.float(f32) * 2.0 - 1.0;
}

/// A multiple of one half from -`extent` / 2 to `extent` / 2: a float that
/// small geometry keeps exact, so edges and corners are hit exactly.
fn halfUnits(random: std.Random, extent: i32) f32 {
    const steps: f32 = @floatFromInt(random.intRangeAtMost(i32, -extent, extent));
    return steps * 0.5;
}

/// A finite float of the kinds geometry code meets: ordinary, whole numbers
/// that tie, zeros of both signs, tiny, and so large that squares overflow.
fn randomFloat(random: std.Random) f32 {
    return switch (random.uintLessThan(u32, 32)) {
        0, 1 => 0.0,
        2 => floatFrom(0x80000000),
        3...9 => halfUnits(random, 16),
        10, 11 => unit(random) * 1.0e-3,
        12 => unit(random) * 1.0e-30,
        13, 14 => unit(random) * 1.0e4,
        15 => unit(random) * 1.0e12,
        16 => unit(random) * 1.0e20,
        else => unit(random) * 50.0,
    };
}

/// An `i32` from the whole range or from the range where squares fit, so
/// both branches of the guarded checks run.
fn randomInt(random: std.Random) i32 {
    if (random.boolean()) return random.int(i32);
    return random.intRangeAtMost(i32, -50000, 50000);
}

/// A float from `awkward_bits`.
fn awkwardFloat(random: std.Random) f32 {
    const bits = awkward_bits[random.uintLessThan(usize, awkward_bits.len)];
    return runtimeFloat(floatFrom(bits));
}

fn randomVector(random: std.Random) [3]f32 {
    return .{ randomFloat(random), randomFloat(random), randomFloat(random) };
}

/// `v`, `v` with one coordinate replaced, or an unrelated vector, so that
/// equal pairs and pairs that share a coordinate come up often.
fn relatedVector(random: std.Random, v: [3]f32) [3]f32 {
    switch (random.uintLessThan(u32, 6)) {
        0 => return v,
        1 => {
            var w = v;
            w[random.uintLessThan(usize, 3)] = randomFloat(random);
            return w;
        },
        else => return randomVector(random),
    }
}

/// A point on the half-unit grid.
fn gridPoint(random: std.Random) [3]f32 {
    return .{ halfUnits(random, 8), halfUnits(random, 4), halfUnits(random, 8) };
}

/// The point a fraction `t` of the way from `a` to `b`.
fn lerpPoint(a: [3]f32, b: [3]f32, t: f32) [3]f32 {
    return .{
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    };
}

/// The mean of the points.
fn centerOf(points: []const [3]f32) [3]f32 {
    var sum: [3]f32 = .{ 0, 0, 0 };
    for (points) |point| {
        sum[0] += point[0];
        sum[1] += point[1];
        sum[2] += point[2];
    }
    const count: f32 = @floatFromInt(points.len);
    return .{ sum[0] / count, sum[1] / count, sum[2] / count };
}

/// The floats of `points`, as the original's functions take them.
fn floatsOf(points: []const [3]f32) [*]const f32 {
    return @ptrCast(points.ptr);
}

/// The point all of a new polygon's vertices start at.
const zero_point: [3]f32 = .{ 0, 0, 0 };

/// A polygon of up to six vertices, the most a navigation polygon has.
const Polygon = struct {
    verts: [6][3]f32,
    count: usize,

    /// A polygon of `count` vertices, all at the origin.
    fn atOrigin(count: usize) Polygon {
        var polygon: Polygon = .{ .verts = undefined, .count = count };
        @memset(&polygon.verts, zero_point);
        return polygon;
    }

    fn points(self: *const Polygon) []const [3]f32 {
        return self.verts[0..self.count];
    }
};

/// A polygon with the given corners, six at most.
fn polygonOf(corners: []const [3]f32) Polygon {
    var polygon: Polygon = .atOrigin(corners.len);
    for (corners, 0..) |corner, k| polygon.verts[k] = corner;
    return polygon;
}

/// A convex polygon of `count` vertices around `center`: points on a circle
/// at ascending turns, each moved a little in angle and radius, and each at
/// a random height. They go clockwise seen from above, the winding Detour's
/// polygons have.
fn convexPolygon(random: std.Random, count: usize, center: [3]f32, radius: f32) Polygon {
    var polygon: Polygon = .atOrigin(count);
    const count_f: f32 = @floatFromInt(count);
    // A move of a few percent keeps a circle's polygon convex up to six
    // vertices: the sagitta of the shortest edge is far larger.
    const move = 0.04;
    for (0..count) |k| {
        const k_f: f32 = @floatFromInt(k);
        const turn = (k_f + unit(random) * 0.2) / count_f;
        const angle = -turn * 2.0 * std.math.pi;
        const r = radius * (1.0 + unit(random) * move);
        polygon.verts[k] = .{
            center[0] + r * @cos(angle),
            center[1] + unit(random) * 4.0,
            center[2] + r * @sin(angle),
        };
    }
    return polygon;
}

/// A convex polygon on the half-unit grid: a rectangle, a right triangle or
/// a hexagon, so corners, edges and middles are exact.
fn gridPolygon(random: std.Random) Polygon {
    const x0 = halfUnits(random, 8);
    const z0 = halfUnits(random, 8);
    const w: f32 = @floatFromInt(random.intRangeAtMost(u32, 1, 4));
    const h: f32 = @floatFromInt(random.intRangeAtMost(u32, 1, 4));
    const y = halfUnits(random, 4);
    return switch (random.uintLessThan(u32, 3)) {
        0 => polygonOf(&[_][3]f32{
            .{ x0, y, z0 },
            .{ x0, y, z0 + h },
            .{ x0 + w, y, z0 + h },
            .{ x0 + w, y, z0 },
        }),
        1 => polygonOf(&[_][3]f32{
            .{ x0, y, z0 },
            .{ x0, y, z0 + h },
            .{ x0 + w, y, z0 },
        }),
        else => polygonOf(&[_][3]f32{
            .{ x0 - 2 * w, y, z0 },
            .{ x0 - w, y, z0 + 2 * h },
            .{ x0 + w, y, z0 + 2 * h },
            .{ x0 + 2 * w, y, z0 },
            .{ x0 + w, y, z0 - 2 * h },
            .{ x0 - w, y, z0 - 2 * h },
        }),
    };
}

/// A polygon for the fuzz tests: mostly convex around a random place, some
/// on the grid, some with a repeated vertex, collinear, or all one point.
fn randomPolygon(random: std.Random) Polygon {
    if (random.uintLessThan(u32, 4) == 0) return gridPolygon(random);

    const count = random.intRangeAtMost(usize, 3, 6);
    const center: [3]f32 = .{ unit(random) * 20.0, unit(random) * 5.0, unit(random) * 20.0 };
    const radius = 0.5 + random.float(f32) * 10.0;
    var polygon = convexPolygon(random, count, center, radius);
    switch (random.uintLessThan(u32, 12)) {
        0 => {
            const k = random.intRangeAtMost(usize, 1, count - 1);
            polygon.verts[k] = polygon.verts[k - 1];
        },
        1 => {
            const a = polygon.verts[0];
            const b = polygon.verts[1];
            for (polygon.verts[0..count]) |*vert| vert.* = lerpPoint(a, b, unit(random) * 2.0);
        },
        2 => {
            for (polygon.verts[1..count]) |*vert| vert.* = polygon.verts[0];
        },
        else => {},
    }
    return polygon;
}

/// A point to test against `polygon`: a vertex, on an edge, level with a
/// vertex, inside, just outside, or far away.
fn pointNear(random: std.Random, polygon: *const Polygon) [3]f32 {
    const verts = polygon.points();
    const k = random.uintLessThan(usize, verts.len);
    const next = (k + 1) % verts.len;
    const center = centerOf(verts);
    return switch (random.uintLessThan(u32, 12)) {
        0, 1 => verts[k],
        2 => lerpPoint(verts[k], verts[next], 0.5),
        3 => lerpPoint(verts[k], verts[next], random.float(f32)),
        4 => .{ verts[k][0] + halfUnits(random, 8), verts[k][1], verts[k][2] },
        5 => .{ verts[k][0], verts[k][1], verts[k][2] + halfUnits(random, 8) },
        6, 7 => lerpPoint(center, verts[k], random.float(f32)),
        8 => lerpPoint(center, verts[k], 1.0 + random.float(f32) * 0.5),
        9 => gridPoint(random),
        10 => center,
        else => .{ unit(random) * 100.0, unit(random) * 10.0, unit(random) * 100.0 },
    };
}

// ---------------------------------------------------------------------------
// Comparisons
// ---------------------------------------------------------------------------

/// Like `support.expectFloat`, naming the function that disagreed.
fn expectFloatIn(function: []const u8, expected: f32, actual: f32) !void {
    support.expectFloat(expected, actual) catch |err| {
        std.debug.print("  in {s}\n", .{function});
        return err;
    };
}

/// Like `support.expectPoint`, naming the function that disagreed.
fn expectPointIn(function: []const u8, expected: [3]f32, actual: [3]f32) !void {
    support.expectPoint(expected, actual) catch |err| {
        std.debug.print("  in {s}\n", .{function});
        return err;
    };
}

/// Like `support.expectFloats`, naming the function that disagreed.
fn expectFloatsIn(function: []const u8, expected: []const f32, actual: []const f32) !void {
    support.expectFloats(expected, actual) catch |err| {
        std.debug.print("  in {s}\n", .{function});
        return err;
    };
}

/// Fails unless the two booleans are equal, naming the function.
fn expectBoolIn(function: []const u8, expected: bool, actual: bool) !void {
    if (expected != actual) {
        std.debug.print("  in {s}: original {}, port {}\n", .{ function, expected, actual });
        return error.TestExpectedEqual;
    }
}

/// Fails unless the two values are equal, naming the function.
fn expectEqualIn(function: []const u8, expected: anytype, actual: @TypeOf(expected)) !void {
    testing.expectEqual(expected, actual) catch |err| {
        std.debug.print("  in {s}\n", .{function});
        return err;
    };
}

// ---------------------------------------------------------------------------
// min, max, abs, sqr and clamp
// ---------------------------------------------------------------------------

/// Fails unless the five selects and products give the original's bits for
/// these inputs: `a` is the value, `b` and `c` the second operand or the
/// bounds.
fn expectSelectsF32(a: f32, b: f32, c: f32) !void {
    errdefer std.debug.print(
        "  a {e} 0x{x:0>8}, b {e} 0x{x:0>8}, c {e} 0x{x:0>8}\n",
        .{ a, bitsOf(a), b, bitsOf(b), c, bitsOf(c) },
    );
    try expectFloatIn("min", reference.ref_min_f32(a, b), common.min(f32, a, b));
    try expectFloatIn("max", reference.ref_max_f32(a, b), common.max(f32, a, b));
    try expectFloatIn("abs", reference.ref_abs_f32(a), common.abs(f32, a));
    try expectFloatIn("sqr", reference.ref_sqr_f32(a), common.sqr(f32, a));
    try expectFloatIn("clamp", reference.ref_clamp_f32(a, b, c), common.clamp(f32, a, b, c));
}

test "min, max, abs, sqr and clamp on f32: every triple of awkward floats" {
    for (awkward_bits) |a_bits| {
        const a = runtimeFloat(floatFrom(a_bits));
        for (awkward_bits) |b_bits| {
            const b = runtimeFloat(floatFrom(b_bits));
            for (awkward_bits) |c_bits| {
                try expectSelectsF32(a, b, runtimeFloat(floatFrom(c_bits)));
            }
        }
    }
}

test "min, max, abs, sqr and clamp on f32: zeros of both signs, one at a time" {
    // C's conditionals return the second operand for equal values, so the
    // sign of a zero shows which operand was picked.
    const zero = runtimeFloat(0.0);
    const negative_zero = runtimeFloat(floatFrom(0x80000000));
    const one = runtimeFloat(1.0);
    const values = [_]f32{ zero, negative_zero, one, -one };
    for (values) |a| {
        for (values) |b| {
            for (values) |c| try expectSelectsF32(a, b, c);
        }
    }
}

test "min, max, abs, sqr and clamp on f32: fuzz" {
    const seed = 0x3201;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        // Awkward and ordinary floats meet in every operand.
        const a = if (random.boolean()) awkwardFloat(random) else randomFloat(random);
        const b = if (random.boolean()) awkwardFloat(random) else randomFloat(random);
        const c = if (random.boolean()) awkwardFloat(random) else randomFloat(random);
        try expectSelectsF32(a, b, c);
    }
}

/// Fails unless the five functions give the original's answers on `i32`.
fn expectSelectsI32(a: i32, b: i32, c: i32) !void {
    errdefer std.debug.print("  a {d}, b {d}, c {d}\n", .{ a, b, c });
    try expectEqualIn("min", reference.ref_min_i32(a, b), common.min(i32, a, b));
    try expectEqualIn("max", reference.ref_max_i32(a, b), common.max(i32, a, b));
    try expectEqualIn("clamp", reference.ref_clamp_i32(a, b, c), common.clamp(i32, a, b, c));
    // The magnitude of the smallest value and the square of a value past
    // 46340 do not fit: upstream overflows there, and the port traps.
    if (a != std.math.minInt(i32)) {
        try expectEqualIn("abs", reference.ref_abs_i32(a), common.abs(i32, a));
    }
    if (a >= -46340 and a <= 46340) {
        try expectEqualIn("sqr", reference.ref_sqr_i32(a), common.sqr(i32, a));
    }
}

test "min, max, abs, sqr and clamp on i32: every triple of edge values" {
    const edges = [_]i32{
        0,
        1,
        -1,
        2,
        -2,
        46340,
        -46340,
        46341,
        -46341,
        std.math.maxInt(i32),
        std.math.minInt(i32),
        std.math.maxInt(i32) - 1,
        std.math.minInt(i32) + 1,
    };
    for (edges) |a| {
        for (edges) |b| {
            for (edges) |c| try expectSelectsI32(a, b, c);
        }
    }
}

test "min, max, abs, sqr and clamp on i32: fuzz" {
    const seed = 0x3202;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        try expectSelectsI32(randomInt(random), randomInt(random), randomInt(random));
    }
}

// ---------------------------------------------------------------------------
// isFinite and the math builtins the port uses
// ---------------------------------------------------------------------------

test "isFinite: the awkward floats" {
    // Under the fast-math switch both sides answer true for everything; the
    // test holds in both builds because it only compares.
    for (awkward_bits) |bits| {
        const x = runtimeFloat(floatFrom(bits));
        errdefer std.debug.print("  x {e} 0x{x:0>8}\n", .{ x, bits });
        try expectBoolIn("isFinite", reference.ref_math_isfinite(x), common.isFinite(x));
    }
}

test "isFinite: random bit patterns" {
    const seed = 0x3203;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        // Any pattern: a fraction of them are infinities and NaNs.
        const x = runtimeFloat(floatFrom(random.int(u32)));
        try expectBoolIn("isFinite", reference.ref_math_isfinite(x), common.isFinite(x));
    }
}

/// Fails unless the square root, floor, ceiling and magnitude builtins give
/// the C library's bits for `x`. A square root that is NaN passes when both
/// sides give a NaN: the original's `sqrtf` sets errno, so for a NaN result it
/// calls the C library, whose NaN need not have the sign and payload the
/// machine's instruction gives. No other result is relaxed.
fn expectExactMath(x: f32) !void {
    errdefer std.debug.print("  x {e} 0x{x:0>8}\n", .{ x, bitsOf(x) });
    const sqrt_expected = reference.ref_math_sqrtf(x);
    const sqrt_actual = @sqrt(x);
    if (std.math.isNan(sqrt_expected) or std.math.isNan(sqrt_actual)) {
        try expectBoolIn("sqrt: NaN", std.math.isNan(sqrt_expected), std.math.isNan(sqrt_actual));
    } else {
        try expectFloatIn("sqrt", sqrt_expected, sqrt_actual);
    }
    try expectFloatIn("floor", reference.ref_math_floorf(x), @floor(x));
    try expectFloatIn("ceil", reference.ref_math_ceilf(x), @ceil(x));
    try expectFloatIn("abs", reference.ref_math_fabsf(x), @abs(x));
}

/// Fails unless the cosine and sine builtins give the C library's bits for
/// `x`, which must be finite and not far from zero.
fn expectTrigMath(x: f32) !void {
    errdefer std.debug.print("  x {e} 0x{x:0>8}\n", .{ x, bitsOf(x) });
    try expectFloatIn("cos", reference.ref_math_cosf(x), @cos(x));
    try expectFloatIn("sin", reference.ref_math_sinf(x), @sin(x));
}

test "math builtins: sqrt, floor, ceil and abs on the awkward floats" {
    for (awkward_bits) |bits| try expectExactMath(runtimeFloat(floatFrom(bits)));
}

test "math builtins: sqrt, floor, ceil and abs, fuzz" {
    const seed = 0x3204;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const x = switch (case % 3) {
            0 => randomFloat(random),
            1 => runtimeFloat(floatFrom(random.int(u32))),
            else => halfUnits(random, 40) + unit(random) * 0.01,
        };
        try expectExactMath(x);
    }
}

test "math builtins: cos and sin at angles that matter" {
    const pi: f32 = std.math.pi;
    const angles = [_]f32{
        0.0,
        floatFrom(0x80000000),
        1.0e-8,
        -1.0e-8,
        0.5,
        1.0,
        pi / 6.0,
        pi / 4.0,
        pi / 3.0,
        pi / 2.0,
        pi,
        3.0 * pi / 2.0,
        2.0 * pi,
        -pi / 2.0,
        -pi,
        100.0,
        -100.0,
        1000.0,
        -1000.0,
    };
    for (angles) |angle| try expectTrigMath(runtimeFloat(angle));
}

test "math builtins: cos and sin, fuzz" {
    const seed = 0x3205;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        // Finite angles within 1000: a turn or so about zero, and wider.
        const x = if (random.boolean()) unit(random) * 7.0 else unit(random) * 1000.0;
        try expectTrigMath(x);
    }
}

// ---------------------------------------------------------------------------
// Vector helpers
// ---------------------------------------------------------------------------

/// Fails unless the arithmetic of two vectors and a scalar, their squared
/// lengths and distances, the 2d products and the closeness test give the
/// original's bits. The functions that take a square root are left to
/// `expectVectorLengths`.
fn expectVectorArithmetic(a: [3]f32, b: [3]f32, s: f32) !void {
    errdefer std.debug.print("  a {any}, b {any}, s {e}\n", .{ a, b, s });
    var out: [3]f32 = undefined;

    reference.ref_vcross(&out, &a, &b);
    try expectPointIn("vcross", out, common.vcross(a, b));
    try expectFloatIn("vdot", reference.ref_vdot(&a, &b), common.vdot(a, b));
    reference.ref_vmad(&out, &a, &b, s);
    try expectPointIn("vmad", out, common.vmad(a, b, s));
    reference.ref_vlerp(&out, &a, &b, s);
    try expectPointIn("vlerp", out, common.vlerp(a, b, s));
    reference.ref_vadd(&out, &a, &b);
    try expectPointIn("vadd", out, common.vadd(a, b));
    reference.ref_vsub(&out, &a, &b);
    try expectPointIn("vsub", out, common.vsub(a, b));
    reference.ref_vscale(&out, &a, s);
    try expectPointIn("vscale", out, common.vscale(a, s));

    // The original updates its first argument in place.
    var low = a;
    reference.ref_vmin(&low, &b);
    try expectPointIn("vmin", low, common.vmin(a, b));
    var high = a;
    reference.ref_vmax(&high, &b);
    try expectPointIn("vmax", high, common.vmax(a, b));

    try expectFloatIn("vlenSqr", reference.ref_vlen_sqr(&a), common.vlenSqr(a));
    try expectFloatIn("vdistSqr", reference.ref_vdist_sqr(&a, &b), common.vdistSqr(a, b));
    try expectFloatIn("vdist2dSqr", reference.ref_vdist_2d_sqr(&a, &b), common.vdist2dSqr(a, b));
    try expectFloatIn("vdot2d", reference.ref_vdot_2d(&a, &b), common.vdot2d(a, b));
    try expectFloatIn("vperp2d", reference.ref_vperp_2d(&a, &b), common.vperp2d(a, b));
    try expectBoolIn("vequal", reference.ref_vequal(&a, &b), common.vequal(a, b));
    try expectBoolIn("visfinite", reference.ref_visfinite(&a), common.visfinite(a));
    try expectBoolIn("visfinite2d", reference.ref_visfinite_2d(&a), common.visfinite2d(a));
}

/// Fails unless the length of a vector and the distances between two give the
/// original's bits. These take square roots, and the original's `sqrtf`
/// calls the C library when the result is NaN, so they are only fed values
/// whose sum of squares is not NaN.
fn expectVectorLengths(a: [3]f32, b: [3]f32) !void {
    errdefer std.debug.print("  a {any}, b {any}\n", .{ a, b });
    try expectFloatIn("vlen", reference.ref_vlen(&a), common.vlen(a));
    try expectFloatIn("vdist", reference.ref_vdist(&a, &b), common.vdist(a, b));
    try expectFloatIn("vdist2d", reference.ref_vdist_2d(&a, &b), common.vdist2d(a, b));
}

/// Fails unless every helper of two vectors and a scalar gives the original's
/// bits.
fn expectVectorMath(a: [3]f32, b: [3]f32, s: f32) !void {
    try expectVectorArithmetic(a, b, s);
    try expectVectorLengths(a, b);
}

test "vector helpers: hand-made vectors and scalars" {
    const negative_zero = floatFrom(0x80000000);
    const vectors = [_][3]f32{
        runtimeVector(.{ 0, 0, 0 }),
        runtimeVector(.{ negative_zero, negative_zero, negative_zero }),
        runtimeVector(.{ 1, 0, 0 }),
        runtimeVector(.{ 0, 1, 0 }),
        runtimeVector(.{ 0, 0, 1 }),
        runtimeVector(.{ 1, 1, 1 }),
        runtimeVector(.{ 1, 2, 3 }),
        runtimeVector(.{ -1, -2, -3 }),
        runtimeVector(.{ 0.5, -0.5, 0.25 }),
        // Squares underflow, then overflow; the last is near the largest float.
        runtimeVector(.{ 1.0e-20, -1.0e-20, 1.0e-20 }),
        runtimeVector(.{ 1.0e20, -1.0e20, 1.0e20 }),
        runtimeVector(.{ 3.0e38, 0, -3.0e38 }),
    };
    const scalars = [_]f32{ 0.0, negative_zero, 1.0, 0.5, -1.0, 2.0, 1.0e-7, 1.0e30 };
    for (vectors) |a| {
        for (vectors) |b| {
            for (scalars) |s| try expectVectorMath(a, b, runtimeFloat(s));
        }
    }
}

test "vector helpers: fuzz" {
    const seed = 0x7ec1;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const a = randomVector(random);
        try expectVectorMath(a, relatedVector(random, a), randomFloat(random));
    }
}

test "vequal: pairs on both sides of the threshold" {
    // The threshold is 1/16384, a power of two, so a step of it is exact.
    const step: f32 = 1.0 / 16384.0;
    const bases = [_][3]f32{
        .{ 0, 0, 0 },
        .{ 3, 1, 2 },
        .{ 100.25, -4, 7.5 },
        .{ 1.0e6, 0, 0 },
    };
    const fractions = [_]f32{ 0.0, 0.25, 0.5, 0.99, 0.999999, 1.0, 1.000001, 1.01, 2.0 };
    for (bases) |a| {
        for (fractions) |fraction| {
            for (0..3) |axis| {
                for ([_]f32{ 1.0, -1.0 }) |sign| {
                    var b = a;
                    b[axis] += sign * fraction * step;
                    try expectVectorMath(runtimeVector(a), runtimeVector(b), 0.5);
                }
            }
            // Spread over two axes: a diagonal step of about the threshold.
            const diagonal = fraction * step * 0.70710678;
            const c: [3]f32 = .{ a[0] + diagonal, a[1], a[2] - diagonal };
            try expectVectorMath(runtimeVector(a), runtimeVector(c), 0.5);
        }
    }
}

test "vector arithmetic: one awkward coordinate, NaNs and infinities included" {
    // One awkward float at a time, in a, in b or as the scalar: the one NaN
    // is then all arithmetic can pass on, whatever order the compiler puts
    // the operands of a sum in.
    const finite = [_][3]f32{
        .{ 1, 2, 3 },
        .{ -4, 0.5, 7 },
        .{ 0, 0, 0 },
        .{ 1.0e20, -1.0e20, 1.0e-20 },
    };
    for (awkward_bits) |bits| {
        const x = runtimeFloat(floatFrom(bits));
        for (finite) |other| {
            for (0..3) |axis| {
                var awkward = other;
                awkward[axis] = x;
                try expectVectorArithmetic(runtimeVector(awkward), runtimeVector(other), 0.5);
                try expectVectorArithmetic(runtimeVector(other), runtimeVector(awkward), 0.5);
            }
            try expectVectorArithmetic(runtimeVector(other), runtimeVector(.{ 1, -1, 2 }), x);
        }
    }
}

test "vmin and vmax: every pair of awkward floats in every position" {
    // Selects only: whichever two NaNs meet, the result is one of the two.
    for (awkward_bits) |p_bits| {
        for (awkward_bits) |q_bits| {
            const p = runtimeFloat(floatFrom(p_bits));
            const q = runtimeFloat(floatFrom(q_bits));
            const a: [3]f32 = .{ p, q, q };
            const b: [3]f32 = .{ q, p, p };
            errdefer std.debug.print("  a {any}, b {any}\n", .{ a, b });
            var low = a;
            reference.ref_vmin(&low, &b);
            try expectPointIn("vmin", low, common.vmin(a, b));
            var high = a;
            reference.ref_vmax(&high, &b);
            try expectPointIn("vmax", high, common.vmax(a, b));
        }
    }
}

test "visfinite and visfinite2d: every vector of awkward floats" {
    for (awkward_bits) |x_bits| {
        for (awkward_bits) |y_bits| {
            for (awkward_bits) |z_bits| {
                const v: [3]f32 = .{
                    runtimeFloat(floatFrom(x_bits)),
                    runtimeFloat(floatFrom(y_bits)),
                    runtimeFloat(floatFrom(z_bits)),
                };
                errdefer std.debug.print("  v {any}\n", .{v});
                try expectBoolIn("visfinite", reference.ref_visfinite(&v), common.visfinite(v));
                try expectBoolIn(
                    "visfinite2d",
                    reference.ref_visfinite_2d(&v),
                    common.visfinite2d(v),
                );
            }
        }
    }
}

/// Fails unless `v` normalizes to the original's bits.
fn expectNormalize(v: [3]f32) !void {
    errdefer std.debug.print("  v {any}\n", .{v});
    var upstream = v;
    reference.ref_vnormalize(&upstream);
    try expectPointIn("vnormalize", upstream, common.vnormalize(v));
}

test "vnormalize: zero, tiny, huge and ordinary vectors" {
    const negative_zero = floatFrom(0x80000000);
    const vectors = [_][3]f32{
        // A zero vector divides by zero: NaNs, as upstream.
        runtimeVector(.{ 0, 0, 0 }),
        runtimeVector(.{ negative_zero, negative_zero, negative_zero }),
        runtimeVector(.{ 1, 0, 0 }),
        runtimeVector(.{ 0, -1, 0 }),
        runtimeVector(.{ 0, 0, 3 }),
        runtimeVector(.{ 1, 1, 1 }),
        runtimeVector(.{ 3, 4, 0 }),
        runtimeVector(.{ -2, 5, 7 }),
        // The squares underflow to zero, and then overflow to infinity.
        runtimeVector(.{ 1.0e-30, 0, 0 }),
        runtimeVector(.{ 1.0e-30, 1.0e-30, -1.0e-30 }),
        runtimeVector(.{ 1.0e20, 0, 0 }),
        runtimeVector(.{ 1.0e20, -1.0e20, 1.0e20 }),
        runtimeVector(.{ 3.0e38, 3.0e38, 3.0e38 }),
        runtimeVector(.{ floatFrom(0x00000001), 0, 0 }),
    };
    for (vectors) |v| try expectNormalize(v);
}

test "vnormalize: fuzz" {
    const seed = 0x7ec2;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        try expectNormalize(randomVector(random));
    }
}

// ---------------------------------------------------------------------------
// Boxes
// ---------------------------------------------------------------------------

/// Fails unless the original and the port agree on whether two quantized
/// boxes overlap.
fn expectQuantOverlap(amin: [3]u16, amax: [3]u16, bmin: [3]u16, bmax: [3]u16) !void {
    errdefer std.debug.print(
        "  a {any} to {any}, b {any} to {any}\n",
        .{ amin, amax, bmin, bmax },
    );
    try expectBoolIn(
        "overlapQuantBounds",
        reference.ref_overlap_quant_bounds(&amin, &amax, &bmin, &bmax),
        common.overlapQuantBounds(amin, amax, bmin, bmax),
    );
}

/// Fails unless the original and the port agree on whether two boxes overlap.
fn expectBoundsOverlap(amin: [3]f32, amax: [3]f32, bmin: [3]f32, bmax: [3]f32) !void {
    errdefer std.debug.print(
        "  a {any} to {any}, b {any} to {any}\n",
        .{ amin, amax, bmin, bmax },
    );
    try expectBoolIn(
        "overlapBounds",
        reference.ref_overlap_bounds(&amin, &amax, &bmin, &bmax),
        common.overlapBounds(amin, amax, bmin, bmax),
    );
}

test "overlapQuantBounds: every arrangement of four values on each axis" {
    // The ends of the range and two between, taken in every order: boxes that
    // touch, nest, are inverted, or are apart on one axis while overlapping
    // on the other two.
    const values = [_]u16{ 0, 1, 32768, 65535 };
    for (0..3) |axis| {
        for (values) |a0| {
            for (values) |a1| {
                for (values) |b0| {
                    for (values) |b1| {
                        var amin: [3]u16 = .{ 1, 1, 1 };
                        var amax: [3]u16 = .{ 40000, 40000, 40000 };
                        var bmin: [3]u16 = .{ 1, 1, 1 };
                        var bmax: [3]u16 = .{ 40000, 40000, 40000 };
                        amin[axis] = a0;
                        amax[axis] = a1;
                        bmin[axis] = b0;
                        bmax[axis] = b1;
                        try expectQuantOverlap(amin, amax, bmin, bmax);
                    }
                }
            }
        }
    }
}

test "overlapBounds: every arrangement of four values on each axis" {
    // Zeros of both signs, whole numbers, infinity and NaN: comparisons with
    // a NaN are false, which leaves the boxes overlapping.
    const negative_zero = floatFrom(0x80000000);
    const values = [_]f32{
        -1.0,
        negative_zero,
        0.0,
        1.0,
        std.math.inf(f32),
        floatFrom(0x7fc00000),
    };
    for (0..3) |axis| {
        for (values) |a0| {
            for (values) |a1| {
                for (values) |b0| {
                    for (values) |b1| {
                        var amin: [3]f32 = .{ -2, -2, -2 };
                        var amax: [3]f32 = .{ 2, 2, 2 };
                        var bmin: [3]f32 = .{ -2, -2, -2 };
                        var bmax: [3]f32 = .{ 2, 2, 2 };
                        amin[axis] = runtimeFloat(a0);
                        amax[axis] = runtimeFloat(a1);
                        bmin[axis] = runtimeFloat(b0);
                        bmax[axis] = runtimeFloat(b1);
                        try expectBoundsOverlap(amin, amax, bmin, bmax);
                    }
                }
            }
        }
    }
}

test "overlapQuantBounds and overlapBounds: fuzz" {
    const seed = 0xb0e5;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });

        // Small values, so that ties and touching boxes are common, and now
        // and then any value at all.
        var quant: [4][3]u16 = undefined;
        for (&quant) |*corner| {
            for (corner) |*coordinate| {
                coordinate.* = if (random.uintLessThan(u32, 8) == 0)
                    random.int(u16)
                else
                    random.uintLessThan(u16, 5);
            }
        }
        try expectQuantOverlap(quant[0], quant[1], quant[2], quant[3]);

        var bounds: [4][3]f32 = undefined;
        for (&bounds) |*corner| {
            for (corner) |*coordinate| {
                coordinate.* = if (random.uintLessThan(u32, 8) == 0)
                    awkwardFloat(random)
                else
                    halfUnits(random, 6);
            }
        }
        try expectBoundsOverlap(bounds[0], bounds[1], bounds[2], bounds[3]);
    }
}

// ---------------------------------------------------------------------------
// Triangles
// ---------------------------------------------------------------------------

/// Triangles for the hand-made cases: a right triangle wound as Detour's are,
/// the same wound the other way, a tilted one, a thin sliver at the height
/// function's threshold, ones with two equal corners, one with three, and two
/// with collinear corners.
const hand_made_triangles = [_][3][3]f32{
    .{ .{ 0, 0, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 0 } },
    .{ .{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 0, 0, 1 } },
    .{ .{ 0, 0, 0 }, .{ 0, 1, 4 }, .{ 4, 2, 0 } },
    .{ .{ -3, 1, -2 }, .{ -3, 2, 5 }, .{ 6, 0, -2 } },
    .{ .{ 0, 0, 0 }, .{ 0, 0, 0.001 }, .{ 0.001, 0, 0 } },
    .{ .{ 0, 0, 0 }, .{ 0, 0, 0 }, .{ 1, 0, 1 } },
    .{ .{ 0, 0, 0 }, .{ 1, 0, 1 }, .{ 1, 0, 1 } },
    .{ .{ 1, 0, 1 }, .{ 0, 0, 0 }, .{ 1, 0, 1 } },
    .{ .{ 1, 2, 3 }, .{ 1, 2, 3 }, .{ 1, 2, 3 } },
    .{ .{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 2, 0, 0 } },
    .{ .{ 0, 0, 0 }, .{ 1, 5, 0 }, .{ 2, 10, 0 } },
};

/// Points to test against a triangle: each corner, the middle of each edge,
/// a point beyond each corner, one beyond each edge's end, each corner moved
/// up and down, the middle of the triangle, and two far away.
fn triangleProbes(tri: [3][3]f32) [21][3]f32 {
    const center = centerOf(&tri);
    var probes: [21][3]f32 = undefined;
    for (0..3) |k| {
        const corner = tri[k];
        const next = tri[(k + 1) % 3];
        probes[k * 6 + 0] = corner;
        probes[k * 6 + 1] = lerpPoint(corner, next, 0.5);
        probes[k * 6 + 2] = lerpPoint(center, corner, 2.0);
        probes[k * 6 + 3] = lerpPoint(corner, next, 1.5);
        probes[k * 6 + 4] = .{ corner[0], corner[1] + 2, corner[2] };
        probes[k * 6 + 5] = .{ corner[0], corner[1] - 2, corner[2] };
    }
    probes[18] = center;
    probes[19] = .{ 30, 5, -40 };
    probes[20] = .{ -30, -5, 40 };
    return probes;
}

/// A random point in the xz plane within 20 of the origin, and a few units
/// up or down.
fn randomPoint(random: std.Random) [3]f32 {
    return .{ unit(random) * 20.0, unit(random) * 5.0, unit(random) * 20.0 };
}

/// A triangle for the fuzz tests: on the grid, so corners coincide and line
/// up often; with an equal pair of corners; collinear; a sliver near the
/// height function's threshold; or just random.
fn randomTriangle(random: std.Random) [3][3]f32 {
    switch (random.uintLessThan(u32, 8)) {
        0, 1 => return .{ gridPoint(random), gridPoint(random), gridPoint(random) },
        2 => {
            const a = randomPoint(random);
            return .{ a, a, randomPoint(random) };
        },
        3 => {
            const a = randomPoint(random);
            const b = randomPoint(random);
            return .{ a, b, lerpPoint(a, b, unit(random) * 2.0) };
        },
        4 => {
            // A right triangle whose legs are about 1/1000 long: twice its
            // area is about 1e-6, the height function's threshold.
            const legs = [_]f32{ 0.9e-3, 0.99e-3, 1.0e-3, 1.01e-3, 1.1e-3 };
            const leg = legs[random.uintLessThan(usize, legs.len)];
            const a = gridPoint(random);
            return .{
                a,
                .{ a[0], a[1], a[2] + leg },
                .{ a[0] + leg, a[1], a[2] },
            };
        },
        else => return .{ randomPoint(random), randomPoint(random), randomPoint(random) },
    }
}

/// The point `u` of the way along the first edge of the triangle and `v`
/// along the second: inside it when both are at least zero and their sum is
/// at most one.
fn pointInTriangle(tri: [3][3]f32, u: f32, v: f32) [3]f32 {
    return .{
        tri[0][0] + (tri[1][0] - tri[0][0]) * u + (tri[2][0] - tri[0][0]) * v,
        tri[0][1] + (tri[1][1] - tri[0][1]) * u + (tri[2][1] - tri[0][1]) * v,
        tri[0][2] + (tri[1][2] - tri[0][2]) * u + (tri[2][2] - tri[0][2]) * v,
    };
}

/// A point to test against `tri`: a corner, on an edge, inside, in the
/// regions around it, level with a corner, on the grid, or far away.
fn pointNearTriangle(random: std.Random, tri: [3][3]f32) [3]f32 {
    const k = random.uintLessThan(usize, 3);
    const next = (k + 1) % 3;
    const u = random.float(f32);
    const v = random.float(f32) * (1.0 - u);
    return switch (random.uintLessThan(u32, 11)) {
        0 => tri[k],
        1 => lerpPoint(tri[k], tri[next], 0.5),
        2 => lerpPoint(tri[k], tri[next], unit(random) * 1.5),
        3, 4 => pointInTriangle(tri, u, v),
        5, 6 => pointInTriangle(tri, unit(random) + 0.5, unit(random) + 0.5),
        7 => gridPoint(random),
        8 => .{ tri[k][0], tri[k][1] + unit(random) * 3.0, tri[k][2] },
        9 => centerOf(&tri),
        else => randomPoint(random),
    };
}

/// Fails unless the triangle's area, closest point and height under a point
/// are the original's, bit for bit. A height the port reports as none must be
/// one the original leaves its output alone for.
fn expectTriangleMath(p: [3]f32, a: [3]f32, b: [3]f32, c: [3]f32) !void {
    errdefer std.debug.print("  p {any}, a {any}, b {any}, c {any}\n", .{ p, a, b, c });
    try expectFloatIn(
        "triArea2d",
        reference.ref_tri_area_2d(&a, &b, &c),
        common.triArea2d(a, b, c),
    );

    var closest: [3]f32 = .{ sentinel(), sentinel(), sentinel() };
    reference.ref_closest_pt_point_triangle(&closest, &p, &a, &b, &c);
    try expectPointIn("closestPointOnTriangle", closest, common.closestPointOnTriangle(p, a, b, c));

    var height = sentinel();
    const found = reference.ref_closest_height_point_triangle(&p, &a, &b, &c, &height);
    const actual = common.closestHeightOnTriangle(p, a, b, c);
    try expectBoolIn("closestHeightOnTriangle: found", found, actual != null);
    if (actual) |value| {
        try expectFloatIn("closestHeightOnTriangle: height", height, value);
    } else {
        try expectFloatIn("closestHeightOnTriangle: output left alone", sentinel(), height);
    }
}

test "triArea2d, closestPointOnTriangle and closestHeightOnTriangle: hand-made triangles" {
    for (hand_made_triangles) |tri| {
        const a = runtimeVector(tri[0]);
        const b = runtimeVector(tri[1]);
        const c = runtimeVector(tri[2]);
        const probes = triangleProbes(.{ a, b, c });
        for (probes) |p| try expectTriangleMath(p, a, b, c);
    }
}

test "triArea2d, closestPointOnTriangle and closestHeightOnTriangle: fuzz" {
    const seed = 0x7a1a;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const tri = randomTriangle(random);
        // Several points to each triangle: its regions are small next to
        // the space around it.
        for (0..4) |_| {
            try expectTriangleMath(pointNearTriangle(random, tri), tri[0], tri[1], tri[2]);
        }
    }
}

// ---------------------------------------------------------------------------
// Polygons and segments
// ---------------------------------------------------------------------------

/// Convex polygons with half-unit corners, wound as Detour's are, with
/// heights that the 2d functions must ignore: a unit square, a right
/// triangle and a hexagon, which has the most vertices a polygon of the mesh
/// can have.
const hand_made_polygons = [_][]const [3]f32{
    &[_][3]f32{ .{ 0, 0, 0 }, .{ 0, 1, 1 }, .{ 1, -2, 1 }, .{ 1, 0.5, 0 } },
    &[_][3]f32{ .{ 0, 0, 0 }, .{ 0, 0, 1 }, .{ 1, 3, 0 } },
    &[_][3]f32{
        .{ -1.5, 1, 0 },
        .{ -1, -2, 1.5 },
        .{ 1, 0.5, 1.5 },
        .{ 1.5, 4, 0 },
        .{ 1, 0, -1.5 },
        .{ -1, -3, -1.5 },
    },
};

/// Polygons that are not polygons, which the functions must still answer
/// for as upstream does: a single vertex, two, collinear ones, all alike, and
/// a square with a vertex repeated. (An empty polygon is a case of its own.)
const degenerate_polygons = [_][]const [3]f32{
    &[_][3]f32{.{ 0.5, 0, 0.5 }},
    &[_][3]f32{ .{ 0, 0, 0 }, .{ 1, 0, 1 } },
    &[_][3]f32{ .{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 2, 0, 0 } },
    &[_][3]f32{ .{ 1, 0, 1 }, .{ 1, 0, 1 }, .{ 1, 0, 1 } },
    &[_][3]f32{ .{ 0, 0, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 1 }, .{ 1, 0, 1 }, .{ 1, 0, 0 } },
};

/// The points of a square lattice, `side` by `side` in x and z, half a unit
/// apart from `first` at the corner, at height zero.
fn latticeOf(comptime side: usize, first: f32) [side * side][3]f32 {
    var points: [side * side][3]f32 = undefined;
    for (0..side) |i| {
        for (0..side) |j| {
            const x: f32 = @floatFromInt(i);
            const z: f32 = @floatFromInt(j);
            points[i * side + j] = .{ first + x * 0.5, 0, first + z * 0.5 };
        }
    }
    return points;
}

/// How much to turn a segment off an edge, as a multiple of the turn that
/// makes the two nearly parallel to the limit upstream sets (1e-6).
const parallel_factors = [_]f32{ 0.0, 0.5, 0.99, 1.0, 1.01, 2.0, -0.5, -1.0, -1.01, -2.0 };

/// The end of a segment from `start` that runs `along` times the vector
/// `edge`, turned by `factor` times the angle at which twice the cross
/// product with `edge` is 1e-6, the limit under which Detour calls two
/// segments parallel.
fn turnedEnd(start: [3]f32, edge: [3]f32, along: f32, factor: f32) [3]f32 {
    const length_sqr = edge[0] * edge[0] + edge[2] * edge[2];
    const turn = if (length_sqr > 0) factor * 1.0e-6 / length_sqr else 0.0;
    return .{
        start[0] + edge[0] * along + edge[2] * turn,
        start[1],
        start[2] + edge[2] * along - edge[0] * turn,
    };
}

/// One of `parallel_factors`.
fn randomFactor(random: std.Random) f32 {
    return parallel_factors[random.uintLessThan(usize, parallel_factors.len)];
}

/// The end of a segment from `start` against `polygon`: the start again, a
/// run along an edge, a run along an edge turned a hair, or another point
/// near the polygon.
fn segmentEnd(random: std.Random, polygon: *const Polygon, start: [3]f32) [3]f32 {
    const verts = polygon.points();
    const k = random.uintLessThan(usize, verts.len);
    const next = (k + 1) % verts.len;
    const edge: [3]f32 = .{ verts[next][0] - verts[k][0], 0, verts[next][2] - verts[k][2] };
    const along = halfUnits(random, 6);
    return switch (random.uintLessThan(u32, 7)) {
        0 => start,
        1 => turnedEnd(start, edge, along, 0.0),
        2, 3 => turnedEnd(start, edge, along, randomFactor(random)),
        else => pointNear(random, polygon),
    };
}

/// Fails unless the segment clips against the polygon exactly as the
/// original's: whether it hits, how far the clipping got, and which edges it
/// crossed.
fn expectSegmentPoly(p0: [3]f32, p1: [3]f32, verts: []const [3]f32) !void {
    errdefer std.debug.print("  p0 {any}, p1 {any}\n  polygon {any}\n", .{ p0, p1, verts });
    var t_min = sentinel();
    var t_max = sentinel();
    var seg_min: i32 = 0x5a5a5a5a;
    var seg_max: i32 = 0x5a5a5a5a;
    const hit = reference.ref_intersect_segment_poly_2d(
        &p0,
        &p1,
        floatsOf(verts),
        @intCast(verts.len),
        &t_min,
        &t_max,
        &seg_min,
        &seg_max,
    );
    const actual = common.intersectSegmentPoly2d(p0, p1, verts);
    try expectBoolIn("intersectSegmentPoly2d: hit", hit, actual.hit);
    try expectFloatIn("intersectSegmentPoly2d: t_min", t_min, actual.t_min);
    try expectFloatIn("intersectSegmentPoly2d: t_max", t_max, actual.t_max);
    try expectEqualIn("intersectSegmentPoly2d: seg_min", seg_min, actual.seg_min);
    try expectEqualIn("intersectSegmentPoly2d: seg_max", seg_max, actual.seg_max);
}

/// Every segment between two lattice points against one polygon.
fn expectLatticeSegments(lattice: []const [3]f32, verts: []const [3]f32) !void {
    for (lattice) |from| {
        for (lattice) |to| try expectSegmentPoly(runtimeVector(from), runtimeVector(to), verts);
    }
}

test "intersectSegmentPoly2d: every lattice segment against the hand-made polygons" {
    // Half-unit lattice from -2 to 2: segments that start or end inside,
    // cross a corner, run along an edge, touch a corner, miss, or have no
    // length at all, with arithmetic that stays exact.
    const lattice = latticeOf(9, -2.0);
    for (hand_made_polygons) |verts| try expectLatticeSegments(&lattice, verts);
    for (degenerate_polygons) |verts| try expectLatticeSegments(&lattice, verts);

    // No polygon at all: nothing is clipped, and the outputs keep their
    // starting values.
    const none: []const [3]f32 = &.{};
    for (lattice) |from| {
        for (lattice) |to| try expectSegmentPoly(runtimeVector(from), runtimeVector(to), none);
    }
}

test "intersectSegmentPoly2d: segments turned a hair from parallel to an edge" {
    const turns = [_]f32{ 0.25, -0.25, 2.0, -2.0 };
    for (hand_made_polygons) |verts| {
        for (verts, 0..) |from, k| {
            const to = verts[(k + 1) % verts.len];
            const edge: [3]f32 = .{ to[0] - from[0], 0, to[2] - from[2] };
            const starts = [_][3]f32{
                lerpPoint(from, to, 0.5),
                centerOf(verts),
                .{ from[0] - 1.0, 0, from[2] },
                from,
            };
            for (starts) |start| {
                for (parallel_factors) |factor| {
                    for (turns) |along| {
                        const end = turnedEnd(start, edge, along, factor);
                        try expectSegmentPoly(runtimeVector(start), runtimeVector(end), verts);
                    }
                }
            }
        }
    }
}

test "intersectSegmentPoly2d: fuzz" {
    const seed = 0x5e91;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const polygon = randomPolygon(random);
        const start = pointNear(random, &polygon);
        try expectSegmentPoly(start, segmentEnd(random, &polygon, start), polygon.points());
    }
}

/// Fails unless the two segments' lines cross as the original's do: whether
/// they do at all, and where along each.
fn expectSegmentSegment(ap: [3]f32, aq: [3]f32, bp: [3]f32, bq: [3]f32) !void {
    errdefer std.debug.print("  a {any} to {any}, b {any} to {any}\n", .{ ap, aq, bp, bq });
    var s = sentinel();
    var t = sentinel();
    const found = reference.ref_intersect_seg_seg_2d(&ap, &aq, &bp, &bq, &s, &t);
    const actual = common.intersectSegmentSegment2d(ap, aq, bp, bq);
    try expectBoolIn("intersectSegmentSegment2d: found", found, actual != null);
    if (actual) |crossing| {
        try expectFloatIn("intersectSegmentSegment2d: s", s, crossing.s);
        try expectFloatIn("intersectSegmentSegment2d: t", t, crossing.t);
    } else {
        // Parallel lines leave the original's outputs alone.
        try expectFloatIn("intersectSegmentSegment2d: s left alone", sentinel(), s);
        try expectFloatIn("intersectSegmentSegment2d: t left alone", sentinel(), t);
    }
}

test "intersectSegmentSegment2d: every pair of lattice segments" {
    // Four values a side: crossing, parallel, collinear (overlapping and
    // apart), identical, touching end to end, T-shaped, and of no length.
    const lattice = latticeOf(4, 0.0);
    for (lattice) |ap| {
        for (lattice) |aq| {
            for (lattice) |bp| {
                for (lattice) |bq| {
                    try expectSegmentSegment(
                        runtimeVector(ap),
                        runtimeVector(aq),
                        runtimeVector(bp),
                        runtimeVector(bq),
                    );
                }
            }
        }
    }
}

test "intersectSegmentSegment2d: segments turned a hair from parallel" {
    const edges = [_][3]f32{
        .{ 1, 0, 0 },
        .{ 0, 0, 1 },
        .{ 2, 0, 1 },
        .{ -3, 0, 0.5 },
        .{ 0.01, 0, 0 },
    };
    const starts = [_][3]f32{ .{ 0, 0, 0 }, .{ 1, 0, 1 }, .{ -2.5, 0, 4 } };
    for (edges) |edge| {
        for (starts) |ap| {
            const aq: [3]f32 = .{ ap[0] + edge[0], ap[1], ap[2] + edge[2] };
            for (starts) |bp| {
                for (parallel_factors) |factor| {
                    for ([_]f32{ 1.0, -1.0, 0.5 }) |along| {
                        const bq = turnedEnd(bp, edge, along, factor);
                        try expectSegmentSegment(
                            runtimeVector(ap),
                            runtimeVector(aq),
                            runtimeVector(bp),
                            runtimeVector(bq),
                        );
                    }
                }
            }
        }
    }
}

test "intersectSegmentSegment2d: fuzz" {
    const seed = 0x5e92;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const grid = random.boolean();
        const ap = if (grid) gridPoint(random) else randomPoint(random);
        const aq = if (grid) gridPoint(random) else randomPoint(random);
        const bp = if (grid) gridPoint(random) else randomPoint(random);
        // The second segment: unrelated, the first run again, the first turned
        // a hair, or of no length.
        const edge: [3]f32 = .{ aq[0] - ap[0], 0, aq[2] - ap[2] };
        const bq = switch (random.uintLessThan(u32, 6)) {
            0 => bp,
            1 => turnedEnd(bp, edge, halfUnits(random, 6), 0.0),
            2, 3 => turnedEnd(bp, edge, halfUnits(random, 6), randomFactor(random)),
            else => if (grid) gridPoint(random) else randomPoint(random),
        };
        try expectSegmentSegment(ap, aq, bp, bq);
    }
}

// ---------------------------------------------------------------------------
// Points against polygons and segments
// ---------------------------------------------------------------------------

/// Fails unless the point is inside the polygon, and at the distances from
/// its edges, that the original says. The edge arrays hold eight entries
/// each, a polygon's count written and the rest left as they started.
fn expectPointPolygon(pt: [3]f32, verts: []const [3]f32) !void {
    errdefer std.debug.print("  pt {any}\n  polygon {any}\n", .{ pt, verts });
    const inside = reference.ref_point_in_polygon(&pt, floatsOf(verts), @intCast(verts.len));
    try expectBoolIn("pointInPolygon", inside, common.pointInPolygon(pt, verts));

    var upstream_dist = sentinelArray(8);
    var upstream_t = sentinelArray(8);
    var port_dist = sentinelArray(8);
    var port_t = sentinelArray(8);
    const inside_edges = reference.ref_distance_pt_poly_edges_sqr(
        &pt,
        floatsOf(verts),
        @intCast(verts.len),
        &upstream_dist,
        &upstream_t,
    );
    const actual = common.distancePointPolyEdgesSqr(
        pt,
        verts,
        port_dist[0..verts.len],
        port_t[0..verts.len],
    );
    try expectBoolIn("distancePointPolyEdgesSqr: inside", inside_edges, actual);
    try expectFloatsIn("distancePointPolyEdgesSqr: distances", &upstream_dist, &port_dist);
    try expectFloatsIn("distancePointPolyEdgesSqr: parameters", &upstream_t, &port_t);
}

test "pointInPolygon and distancePointPolyEdgesSqr: every lattice point in the polygons" {
    // Corners, edges, middles, points level with a corner, and points
    // outside: the half-unit lattice meets each of them.
    const lattice = latticeOf(9, -2.0);
    for (hand_made_polygons) |verts| {
        for (lattice) |pt| try expectPointPolygon(runtimeVector(pt), verts);
    }
    for (degenerate_polygons) |verts| {
        for (lattice) |pt| try expectPointPolygon(runtimeVector(pt), verts);
    }

    // No polygon: nothing is inside, and no edge array entry is written.
    const none: []const [3]f32 = &.{};
    for (lattice) |pt| try expectPointPolygon(runtimeVector(pt), none);
}

test "pointInPolygon and distancePointPolyEdgesSqr: fuzz" {
    const seed = 0x9091;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const polygon = randomPolygon(random);
        for (0..3) |_| {
            try expectPointPolygon(pointNear(random, &polygon), polygon.points());
        }
    }
}

/// Fails unless the distance from the point to the segment, and where the
/// closest point lies along it, are the original's.
fn expectPointSegment(pt: [3]f32, p: [3]f32, q: [3]f32) !void {
    errdefer std.debug.print("  pt {any}, p {any}, q {any}\n", .{ pt, p, q });
    var t = sentinel();
    const dist_sqr = reference.ref_distance_pt_seg_sqr_2d(&pt, &p, &q, &t);
    const actual = common.distancePointSegmentSqr2d(pt, p, q);
    try expectFloatIn("distancePointSegmentSqr2d: dist_sqr", dist_sqr, actual.dist_sqr);
    try expectFloatIn("distancePointSegmentSqr2d: t", t, actual.t);
}

test "distancePointSegmentSqr2d: every lattice triple" {
    // A segment of no length, points before, along and after a segment, and
    // points on it: the lattice meets each.
    const lattice = latticeOf(7, -1.5);
    for (lattice) |pt| {
        for (lattice) |p| {
            for (lattice) |q| {
                try expectPointSegment(runtimeVector(pt), runtimeVector(p), runtimeVector(q));
            }
        }
    }
}

test "distancePointSegmentSqr2d: fuzz" {
    const seed = 0x9092;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const grid = random.uintLessThan(u32, 4) == 0;
        const p = if (grid) gridPoint(random) else randomPoint(random);
        const q = switch (random.uintLessThan(u32, 6)) {
            0 => p,
            else => if (grid) gridPoint(random) else randomPoint(random),
        };
        // Beside the segment, past either end, on it, or anywhere.
        const pt = switch (random.uintLessThan(u32, 6)) {
            0 => lerpPoint(p, q, random.float(f32)),
            1 => lerpPoint(p, q, 1.0 + random.float(f32)),
            2 => lerpPoint(p, q, -random.float(f32)),
            3 => p,
            4 => q,
            else => if (grid) gridPoint(random) else randomPoint(random),
        };
        try expectPointSegment(pt, p, q);
    }
}

/// Fails unless the centre of the vertices the indices pick is the original's.
fn expectPolyCenter(indices: []const u16, verts: []const [3]f32) !void {
    errdefer std.debug.print("  indices {any}\n  verts {any}\n", .{ indices, verts });
    var center: [3]f32 = .{ sentinel(), sentinel(), sentinel() };
    reference.ref_calc_poly_center(&center, indices.ptr, @intCast(indices.len), floatsOf(verts));
    try expectPointIn("calcPolyCenter", center, common.calcPolyCenter(indices, verts));
}

test "calcPolyCenter: hand-made index lists" {
    const negative_zero = floatFrom(0x80000000);
    const verts = [_][3]f32{
        runtimeVector(.{ 0, 0, 0 }),
        runtimeVector(.{ 1, 2, 3 }),
        runtimeVector(.{ -4, 5, 6.5 }),
        runtimeVector(.{ 7, -8, 9 }),
        runtimeVector(.{ 1.0e20, 1.0e-20, -1.0e20 }),
        runtimeVector(.{ negative_zero, negative_zero, negative_zero }),
        runtimeVector(.{ 0.1, 0.2, 0.3 }),
        runtimeVector(.{ 3.0e38, 3.0e38, 3.0e38 }),
    };
    const lists = [_][]const u16{
        &.{0},
        &.{7},
        &.{ 0, 1 },
        &.{ 1, 1 },
        &.{ 0, 1, 2 },
        &.{ 2, 1, 0 },
        &.{ 0, 1, 2, 3 },
        &.{ 1, 2, 3, 6, 0 },
        &.{ 0, 1, 2, 3, 6, 7 },
        &.{ 4, 4, 4, 4, 4, 4 },
        &.{ 5, 5, 5 },
        &.{ 3, 3, 0, 0, 3, 3 },
    };
    for (lists) |indices| try expectPolyCenter(indices, &verts);

    // No indices: the mean divides by zero, and the result is NaNs, as
    // upstream's. The count is hidden so that nothing is folded.
    const all = [_]u16{ 0, 1, 2 };
    try expectPolyCenter(all[0..runtimeCount(0)], &verts);
}

test "calcPolyCenter: fuzz" {
    const seed = 0x9093;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        var verts: [12][3]f32 = undefined;
        for (&verts) |*vert| vert.* = randomVector(random);
        // Up to six indices, the most a polygon has; one may repeat.
        var indices: [6]u16 = undefined;
        const count = random.intRangeAtMost(usize, 1, indices.len);
        for (indices[0..count]) |*index| index.* = random.uintLessThan(u16, verts.len);
        try expectPolyCenter(indices[0..count], &verts);
    }
}

/// Fails unless the two convex polygons overlap in the xz plane exactly when
/// the original says.
fn expectPolyOverlap(polya: []const [3]f32, polyb: []const [3]f32) !void {
    errdefer std.debug.print("  polygon a {any}\n  polygon b {any}\n", .{ polya, polyb });
    const overlap = reference.ref_overlap_poly_poly_2d(
        floatsOf(polya),
        @intCast(polya.len),
        floatsOf(polyb),
        @intCast(polyb.len),
    );
    try expectBoolIn("overlapPolyPoly2d", overlap, common.overlapPolyPoly2d(polya, polyb));
}

/// `verts` moved `dx` along x and `dz` along z.
fn shiftedPolygon(verts: []const [3]f32, dx: f32, dz: f32) Polygon {
    var polygon: Polygon = .atOrigin(verts.len);
    for (verts, 0..) |vert, k| polygon.verts[k] = .{ vert[0] + dx, vert[1], vert[2] + dz };
    return polygon;
}

test "overlapPolyPoly2d: hand-made polygons shifted over the lattice" {
    // A polygon against another moved a half unit at a time, from clear of
    // it, through touching along an edge and at a corner, to on top of it.
    const lattice = latticeOf(9, -2.0);
    for (hand_made_polygons) |verts_a| {
        for (hand_made_polygons) |verts_b| {
            for (lattice) |offset| {
                const moved = shiftedPolygon(verts_b, offset[0], offset[2]);
                try expectPolyOverlap(verts_a, moved.points());
            }
        }
    }

    // Polygons that are not polygons, against each other and the others.
    for (degenerate_polygons) |verts_a| {
        for (degenerate_polygons) |verts_b| try expectPolyOverlap(verts_a, verts_b);
        for (hand_made_polygons) |verts_b| {
            try expectPolyOverlap(verts_a, verts_b);
            try expectPolyOverlap(verts_b, verts_a);
        }
    }

    // Two polygons with no vertices have no axis to separate them.
    const none: []const [3]f32 = &.{};
    try expectPolyOverlap(none, none);
}

test "overlapPolyPoly2d: gaps around the 1e-4 tolerance" {
    // Two squares with a gap, or an overlap, of a few multiples of the
    // tolerance, along x, along z and on the diagonal.
    const gaps = [_]f32{ -2.0e-3, -2.0e-4, -1.0e-4, -5.0e-5, 0.0, 5.0e-5, 1.0e-4, 2.0e-4, 2.0e-3 };
    for (hand_made_polygons) |verts| {
        for (gaps) |gap| {
            // Moved by the width of the polygon (or its height) plus the gap.
            var low = verts[0];
            var high = verts[0];
            for (verts) |vert| {
                low = .{ @min(low[0], vert[0]), 0, @min(low[2], vert[2]) };
                high = .{ @max(high[0], vert[0]), 0, @max(high[2], vert[2]) };
            }
            const width = high[0] - low[0];
            const height = high[2] - low[2];
            const moves = [_][2]f32{
                .{ width + gap, 0 },
                .{ -width - gap, 0 },
                .{ 0, height + gap },
                .{ 0, -height - gap },
                .{ width + gap, height + gap },
            };
            for (moves) |move| {
                const moved = shiftedPolygon(verts, move[0], move[1]);
                try expectPolyOverlap(verts, moved.points());
            }
        }
    }
}

test "overlapPolyPoly2d: fuzz" {
    const seed = 0x9094;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const polya = randomPolygon(random);
        // The other polygon: unrelated, a copy moved a little or a lot, or a
        // copy exactly on top.
        const polyb = switch (random.uintLessThan(u32, 5)) {
            0 => randomPolygon(random),
            1 => shiftedPolygon(polya.points(), unit(random) * 5.0, unit(random) * 5.0),
            2 => shiftedPolygon(polya.points(), halfUnits(random, 8), halfUnits(random, 8)),
            3 => polya,
            else => gridPolygon(random),
        };
        try expectPolyOverlap(polya.points(), polyb.points());
    }
}

// ---------------------------------------------------------------------------
// A random point in a convex polygon
// ---------------------------------------------------------------------------

/// Fails unless the original's point and area scratch values are the port's,
/// for the polygon `pts` and the draws `s` and `t`. The scratch arrays hold
/// eight entries each and start as the sentinel on both sides, so the
/// entries the function leaves alone are compared too.
fn expectRandomPoint(pts: []const [3]f32, s: f32, t: f32) !void {
    errdefer std.debug.print("  s {e}, t {e}\n  polygon {any}\n", .{ s, t, pts });
    var upstream_areas = sentinelArray(8);
    var port_areas = sentinelArray(8);
    var upstream_point: [3]f32 = .{ sentinel(), sentinel(), sentinel() };
    reference.ref_random_point_in_convex_poly(
        floatsOf(pts),
        @intCast(pts.len),
        &upstream_areas,
        s,
        t,
        &upstream_point,
    );
    // The port's scratch slice is exactly the polygon's length, so a write
    // past it would trap.
    const point = common.randomPointInConvexPoly(pts, port_areas[0..pts.len], s, t);
    try expectPointIn("randomPointInConvexPoly: point", upstream_point, point);
    try expectFloatsIn("randomPointInConvexPoly: areas", &upstream_areas, &port_areas);
}

/// Catch2's `Approx(expected)`, which upstream's own test compares with: the
/// difference is at most 100 float epsilons of the expected value, so a
/// value of zero must be matched exactly.
fn expectNearCatch(expected: f32, actual: f32) !void {
    const expected_wide: f64 = expected;
    const actual_wide: f64 = actual;
    const margin = @as(f64, std.math.floatEps(f32)) * 100.0 * @abs(expected_wide);
    if (!(@abs(actual_wide - expected_wide) <= margin)) {
        std.debug.print("  expected about {e}, got {e}\n", .{ expected, actual });
        return error.TestExpectedApproxEqual;
    }
}

test "randomPointInConvexPoly: upstream's own test, with s up to 1.0" {
    // The triangle and the three draws of Tests_Detour.cpp, t being 1.0.
    const pts = [_][3]f32{ .{ 0, 0, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 0 } };
    const rows = [_]struct { s: f32, expected: [3]f32 }{
        .{ .s = 0.0, .expected = .{ 0, 0, 1 } },
        .{ .s = 0.5, .expected = .{ 0.5, 0, 0.5 } },
        .{ .s = 1.0, .expected = .{ 1, 0, 0 } },
    };
    for (rows) |row| {
        // Both sides on the same inputs, bit for bit.
        try expectRandomPoint(&pts, row.s, 1.0);

        // And each against the values upstream's test expects. The scratch
        // array holds six entries, as upstream's does.
        var upstream_areas = sentinelArray(6);
        var upstream_point: [3]f32 = .{ sentinel(), sentinel(), sentinel() };
        reference.ref_random_point_in_convex_poly(
            floatsOf(&pts),
            pts.len,
            &upstream_areas,
            row.s,
            1.0,
            &upstream_point,
        );
        var port_areas = sentinelArray(6);
        const port_point = common.randomPointInConvexPoly(&pts, port_areas[0..pts.len], row.s, 1.0);
        for (upstream_point, port_point, row.expected) |upstream, port, want| {
            try expectNearCatch(want, upstream);
            try expectNearCatch(want, port);
        }
    }
}

test "randomPointInConvexPoly: hand-made polygons with the ends of s and t" {
    // Draws at both ends, just under 1, and 1 itself: past the last
    // triangle's share of the area, where the function falls back to the last
    // triangle.
    const s_values = [_]f32{ 0.0, 1.0e-30, 0.25, 0.5, 0.75, floatFrom(0x3f7fffff), 1.0 };
    const t_values = [_]f32{ 0.0, 0.25, 0.5, 1.0 };
    for (hand_made_polygons) |pts| {
        for (s_values) |s| {
            for (t_values) |t| try expectRandomPoint(pts, s, t);
        }
    }
    // Polygons without area: every triangle's share is zero or negative. A
    // single vertex is left out; the function reads two points at least.
    for (degenerate_polygons) |pts| {
        if (pts.len < 2) continue;
        for (s_values) |s| {
            for (t_values) |t| try expectRandomPoint(pts, s, t);
        }
    }
    // The same square wound the other way round has negative areas.
    const reversed = [_][3]f32{ .{ 1, 0, 0 }, .{ 1, 0, 1 }, .{ 0, 0, 1 }, .{ 0, 0, 0 } };
    for (s_values) |s| {
        for (t_values) |t| try expectRandomPoint(&reversed, s, t);
    }
}

/// A draw for the fuzz test: the ends of the range, just under 1, or any.
fn randomDraw(random: std.Random) f32 {
    return switch (random.uintLessThan(u32, 10)) {
        0 => 0.0,
        1 => 1.0,
        2 => floatFrom(0x3f7fffff),
        else => random.float(f32),
    };
}

test "randomPointInConvexPoly: fuzz" {
    const seed = 0xa4d1;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const polygon = randomPolygon(random);
        // Now and then the first two vertices alone: the least it takes.
        const count: usize = if (random.uintLessThan(u32, 10) == 0) 2 else polygon.count;
        try expectRandomPoint(polygon.points()[0..count], randomDraw(random), randomDraw(random));
    }
}

// ---------------------------------------------------------------------------
// Integer helpers
// ---------------------------------------------------------------------------

/// Fails unless the next power of two and the highest set bit of `v` are the
/// original's.
fn expectPowerOfTwo(v: u32) !void {
    errdefer std.debug.print("  v {d} (0x{x:0>8})\n", .{ v, v });
    try expectEqualIn("nextPow2", reference.ref_next_pow2(v), common.nextPow2(v));
    try expectEqualIn("ilog2", reference.ref_ilog2(v), common.ilog2(v));
}

test "nextPow2 and ilog2: every power of two, its neighbours and the ends" {
    for (0..32) |shift| {
        const bit = @as(u32, 1) << @intCast(shift);
        const around = [_]u32{ bit -% 1, bit, bit +% 1, bit | (bit >> 1), ~bit, bit *% 3 };
        for (around) |v| try expectPowerOfTwo(v);
    }
    // Zero, which rounds to zero; one; the largest values, past which the
    // next power of two wraps to zero; and the byte and word edges.
    const edges = [_]u32{
        0,
        1,
        2,
        3,
        0xff,
        0x100,
        0xffff,
        0x10000,
        0x7fffffff,
        0x80000000,
        0x80000001,
        0xfffffffe,
        0xffffffff,
    };
    for (edges) |v| try expectPowerOfTwo(v);
}

test "nextPow2 and ilog2: fuzz" {
    const seed = 0x1092;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        // Shifted down by a random amount, so every magnitude comes up.
        try expectPowerOfTwo(random.int(u32) >> random.int(u5));
    }
}

/// Fails unless `x` rounds up to the multiple of four the original gives.
fn expectAlign4(x: i32) !void {
    errdefer std.debug.print("  x {d}\n", .{x});
    const expected: usize = @intCast(reference.ref_align4(x));
    try expectEqualIn("align4", expected, common.align4(@intCast(x)));
}

test "align4: every small value and the largest one upstream can add 3 to" {
    var x: i32 = 0;
    while (x <= 300) : (x += 1) try expectAlign4(x);

    const largest = std.math.maxInt(i32) - 3;
    for ([_]i32{ largest - 5, largest - 4, largest - 1, largest }) |v| try expectAlign4(v);
}

test "align4: fuzz" {
    const seed = 0xa164;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const largest = std.math.maxInt(i32) - 3;
        try expectAlign4(if (random.boolean())
            random.intRangeAtMost(i32, 0, largest)
        else
            random.intRangeAtMost(i32, 0, 5000));
    }
}

/// Fails unless the tile facing `side` is the original's.
fn expectOppositeTile(side: i32) !void {
    errdefer std.debug.print("  side {d}\n", .{side});
    try expectEqualIn("oppositeTile", reference.ref_opposite_tile(side), common.oppositeTile(side));
}

test "oppositeTile: the eight sides, the sides around them and the ends" {
    var side: i32 = -40;
    while (side <= 40) : (side += 1) try expectOppositeTile(side);
    // Upstream adds 4, so the largest value it can take is 4 short of the top.
    const edges = [_]i32{
        std.math.minInt(i32),
        std.math.minInt(i32) + 1,
        -1000000,
        1000000,
        std.math.maxInt(i32) - 5,
        std.math.maxInt(i32) - 4,
    };
    for (edges) |v| try expectOppositeTile(v);
}

test "oppositeTile: fuzz" {
    const seed = 0x0991;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const largest = std.math.maxInt(i32) - 4;
        try expectOppositeTile(random.intRangeAtMost(i32, std.math.minInt(i32), largest));
    }
}

/// Fails unless the float with these bits is swapped end for end as the
/// original swaps it. The original swaps the four bytes in memory, so they
/// pass through an integer here: no float register touches them.
fn expectSwapEndianFloat(bits: u32) !void {
    errdefer std.debug.print("  bits 0x{x:0>8}\n", .{bits});
    var word = bits;
    reference.ref_swap_endian_f32(@ptrCast(&word));
    const port: u32 = @bitCast(common.swapEndianFloat(floatFrom(bits)));
    try expectEqualIn("swapEndianFloat", word, port);
    // And the original's result is the plain byte reversal.
    try expectEqualIn("byte reversal", @byteSwap(bits), word);
}

test "swapEndianFloat: awkward floats and byte patterns" {
    for (awkward_bits) |bits| try expectSwapEndianFloat(bits);
    const patterns = [_]u32{
        0,
        0xffffffff,
        0x01020304,
        0x04030201,
        0x12345678,
        0x000000ff,
        0xff000000,
    };
    for (patterns) |bits| try expectSwapEndianFloat(bits);
}

test "swapEndianFloat: fuzz" {
    const seed = 0x5a4f;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        try expectSwapEndianFloat(random.int(u32));
    }
}

// ---------------------------------------------------------------------------
// truncToInt
// ---------------------------------------------------------------------------

/// Fails unless `x` truncates to `expected` as type `T`.
fn expectTrunc(comptime T: type, x: f32, expected: T) !void {
    errdefer std.debug.print("  {s} from {e} (0x{x:0>8})\n", .{ @typeName(T), x, bitsOf(x) });
    try testing.expectEqual(expected, common.truncToInt(T, x));
}

test "truncToInt: i32 truncates toward zero, and gives the smallest value outside the range" {
    const smallest = std.math.minInt(i32);
    const negative_zero = floatFrom(0x80000000);
    const rows = [_]struct { x: f32, expected: i32 }{
        .{ .x = 0.0, .expected = 0 },
        .{ .x = negative_zero, .expected = 0 },
        .{ .x = 0.5, .expected = 0 },
        .{ .x = -0.5, .expected = 0 },
        .{ .x = floatFrom(0x3f7fffff), .expected = 0 },
        .{ .x = 1.0, .expected = 1 },
        .{ .x = 1.5, .expected = 1 },
        .{ .x = -1.5, .expected = -1 },
        .{ .x = floatFrom(0x403fffff), .expected = 2 },
        .{ .x = 123456.7, .expected = 123456 },
        .{ .x = -123456.7, .expected = -123456 },
        // The largest float below 2^31, and 2^31 itself.
        .{ .x = 2147483520.0, .expected = 2147483520 },
        .{ .x = -2147483520.0, .expected = -2147483520 },
        .{ .x = -2147483648.0, .expected = smallest },
        .{ .x = 2147483648.0, .expected = smallest },
        .{ .x = -2147483904.0, .expected = smallest },
        .{ .x = 3.0e9, .expected = smallest },
        .{ .x = -3.0e9, .expected = smallest },
        .{ .x = 1.0e30, .expected = smallest },
        .{ .x = -1.0e30, .expected = smallest },
        .{ .x = floatFrom(0x7f7fffff), .expected = smallest },
        .{ .x = floatFrom(0x7f800000), .expected = smallest },
        .{ .x = floatFrom(0xff800000), .expected = smallest },
        .{ .x = floatFrom(0x7fc00000), .expected = smallest },
        .{ .x = floatFrom(0xffc00000), .expected = smallest },
    };
    for (rows) |row| try expectTrunc(i32, runtimeFloat(row.x), row.expected);
}

test "truncToInt: narrower types keep the low bits of the 32-bit result" {
    const negative_zero = floatFrom(0x80000000);
    const nan = floatFrom(0x7fc00000);
    const infinity = floatFrom(0x7f800000);

    const unsigned_16 = [_]struct { x: f32, expected: u16 }{
        .{ .x = 0.0, .expected = 0 },
        .{ .x = negative_zero, .expected = 0 },
        .{ .x = 1.0, .expected = 1 },
        .{ .x = 65535.0, .expected = 65535 },
        .{ .x = 65536.0, .expected = 0 },
        .{ .x = 65537.0, .expected = 1 },
        .{ .x = 70000.0, .expected = 4464 },
        .{ .x = 40000.5, .expected = 40000 },
        .{ .x = -1.0, .expected = 0xffff },
        .{ .x = -1.9, .expected = 0xffff },
        .{ .x = -65535.0, .expected = 1 },
        .{ .x = -65536.0, .expected = 0 },
        .{ .x = 2147483520.0, .expected = 0xff80 },
        .{ .x = -2147483648.0, .expected = 0 },
        .{ .x = 2147483648.0, .expected = 0 },
        .{ .x = 1.0e30, .expected = 0 },
        .{ .x = infinity, .expected = 0 },
        .{ .x = nan, .expected = 0 },
    };
    for (unsigned_16) |row| try expectTrunc(u16, runtimeFloat(row.x), row.expected);

    const unsigned_8 = [_]struct { x: f32, expected: u8 }{
        .{ .x = 0.0, .expected = 0 },
        .{ .x = 127.5, .expected = 127 },
        .{ .x = 255.0, .expected = 255 },
        .{ .x = 256.0, .expected = 0 },
        .{ .x = 300.0, .expected = 44 },
        .{ .x = -1.0, .expected = 255 },
        .{ .x = -128.0, .expected = 128 },
        .{ .x = 2147483520.0, .expected = 0x80 },
        .{ .x = 1.0e10, .expected = 0 },
        .{ .x = infinity, .expected = 0 },
        .{ .x = nan, .expected = 0 },
    };
    for (unsigned_8) |row| try expectTrunc(u8, runtimeFloat(row.x), row.expected);

    const signed_16 = [_]struct { x: f32, expected: i16 }{
        .{ .x = 0.0, .expected = 0 },
        .{ .x = -1.0, .expected = -1 },
        .{ .x = 32767.9, .expected = 32767 },
        .{ .x = 32768.0, .expected = -32768 },
        .{ .x = -32768.0, .expected = -32768 },
        .{ .x = 40000.0, .expected = -25536 },
        .{ .x = -40000.0, .expected = 25536 },
        .{ .x = 65535.0, .expected = -1 },
        .{ .x = 65536.0, .expected = 0 },
        .{ .x = nan, .expected = 0 },
    };
    for (signed_16) |row| try expectTrunc(i16, runtimeFloat(row.x), row.expected);

    const signed_8 = [_]struct { x: f32, expected: i8 }{
        .{ .x = 0.0, .expected = 0 },
        .{ .x = 127.0, .expected = 127 },
        .{ .x = 128.0, .expected = -128 },
        .{ .x = 200.0, .expected = -56 },
        .{ .x = 255.0, .expected = -1 },
        .{ .x = 256.0, .expected = 0 },
        .{ .x = -1.0, .expected = -1 },
        .{ .x = -129.0, .expected = 127 },
        .{ .x = nan, .expected = 0 },
    };
    for (signed_8) |row| try expectTrunc(i8, runtimeFloat(row.x), row.expected);
}

/// What the machine's conversion gives for a float in the range of `i32`,
/// worked out through 64 bits: the low bits of the truncated value.
fn truncInRange(comptime T: type, x: f32) T {
    const wide: i64 = @trunc(x);
    const Bits = @Int(.unsigned, @bitSizeOf(T));
    const bits: u64 = @bitCast(wide);
    return @bitCast(@as(Bits, @truncate(bits)));
}

/// What the machine gives outside the range of `i32`, and for NaN: the
/// smallest `i32`, whose low 8 or 16 bits are all zero.
fn truncOutOfRange(comptime T: type) T {
    return if (T == i32) std.math.minInt(i32) else 0;
}

/// Fails unless `x` truncates as it should to each of the types.
fn expectTruncAll(x: f32) !void {
    const in_range = x >= -2147483648.0 and x < 2147483648.0;
    inline for (.{ i32, u16, u8, i16, i8 }) |T| {
        const expected: T = if (in_range) truncInRange(T, x) else truncOutOfRange(T);
        try expectTrunc(T, x, expected);
    }
}

test "truncToInt: fuzz against the 64-bit working" {
    const seed = 0x7c01;
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    for (0..fuzz_cases) |case| {
        errdefer std.debug.print("  seed {d}, case {d}\n", .{ seed, case });
        const x = switch (random.uintLessThan(u32, 6)) {
            0 => unit(random) * 10.0,
            1 => unit(random) * 70000.0,
            2 => unit(random) * 3.0e9,
            3 => unit(random) * 1.0e30,
            4 => halfUnits(random, 600),
            else => awkwardFloat(random),
        };
        try expectTruncAll(x);
    }
}
