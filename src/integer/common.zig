//! Geometry helpers of the integer variant.
//!
//! Positions are `[3]i32` in millimetres with y up. Functions named `2d`
//! work on the xz plane and ignore y. Every coordinate lies within
//! `coord_max` of zero, which is what keeps each product below inside the
//! type that holds it: a difference of two coordinates fits 31 bits, a
//! product of two differences 61, a cross or dot product in the plane 62.
//! Nothing here uses a floating-point number.

const std = @import("std");

/// The largest magnitude of a coordinate, in millimetres.
pub const coord_max: i32 = 1 << 29;

/// Whether every coordinate of `v` is within `coord_max` of zero.
pub inline fn inBounds(v: [3]i32) bool {
    return @abs(v[0]) <= coord_max and @abs(v[1]) <= coord_max and @abs(v[2]) <= coord_max;
}

/// A place along a segment or ray: `num / den`, with `den` never negative.
/// A zero `den` under a positive `num` stands for a place that is never
/// reached. Fractions are compared exactly and are not reduced.
pub const Fraction = struct {
    num: i64,
    den: i64,

    pub const zero: Fraction = .{ .num = 0, .den = 1 };
    pub const half: Fraction = .{ .num = 1, .den = 2 };
    pub const one: Fraction = .{ .num = 1, .den = 1 };
    /// Beyond every place: larger than any other fraction.
    pub const never: Fraction = .{ .num = 1, .den = 0 };

    /// `num / den` for a `den` of either sign, not zero.
    pub inline fn init(num: i64, den: i64) Fraction {
        return if (den < 0) .{ .num = -num, .den = -den } else .{ .num = num, .den = den };
    }

    pub inline fn isNever(self: Fraction) bool {
        return self.den == 0;
    }

    pub inline fn order(a: Fraction, b: Fraction) std.math.Order {
        const left = @as(i128, a.num) * b.den;
        const right = @as(i128, b.num) * a.den;
        return std.math.order(left, right);
    }

    pub inline fn lessThan(a: Fraction, b: Fraction) bool {
        return a.order(b) == .lt;
    }

    pub inline fn greaterThan(a: Fraction, b: Fraction) bool {
        return a.order(b) == .gt;
    }

    /// Whether the fraction is at least one; `never` is.
    pub inline fn atLeastOne(self: Fraction) bool {
        return self.num >= self.den;
    }

    /// Whether the fraction is below zero.
    pub inline fn isNegative(self: Fraction) bool {
        return self.num < 0;
    }

    /// The fraction held to 0..1.
    pub inline fn clamped(self: Fraction) Fraction {
        if (self.num <= 0) return zero;
        if (self.num >= self.den) return one;
        return self;
    }
};

/// `n / d` rounded to the nearest whole number, halves away from zero.
/// `d` must be positive.
pub inline fn divRound(n: i128, d: i128) i128 {
    // Most operands fit 64 bits, which the processor divides in one
    // instruction; a division of 128 bits is a library call.
    if (std.math.cast(i64, n)) |narrow_n| {
        if (std.math.cast(i64, d)) |narrow_d| return divRoundNarrow(narrow_n, narrow_d);
    }
    const quotient = @divTrunc(n, d);
    const remainder = n - quotient * d;
    if (2 * @abs(remainder) < d) return quotient;
    return if (n < 0) quotient - 1 else quotient + 1;
}

inline fn divRoundNarrow(n: i64, d: i64) i64 {
    const quotient = @divTrunc(n, d);
    // Below `d`, so twice it still fits 64 bits unsigned.
    const remainder = @abs(@rem(n, d));
    if (2 * remainder < @as(u64, @intCast(d))) return quotient;
    return if (n < 0) quotient - 1 else quotient + 1;
}

/// The root of the first value of each top byte's successor, for the top
/// bytes 64 to 255 of a number of 64 bits: `ceil(sqrt((byte + 1) << 56))`
/// in units of 2^16. It is at or above the root of every number with that
/// top byte, and within 2^-7 of it.
const root_estimates: [192]u32 = blk: {
    @setEvalBranchQuota(10_000);
    var estimates: [192]u32 = undefined;
    for (&estimates, 64..) |*estimate, byte| {
        // The rounded-up root of (byte + 1) * 2^24, by bisection.
        const square: u64 = (byte + 1) << 24;
        var low: u64 = 0;
        var high: u64 = 1 << 16;
        while (low < high) {
            const middle = (low + high) / 2;
            if (middle * middle < square) low = middle + 1 else high = middle;
        }
        estimate.* = low;
    }
    break :blk estimates;
};

/// The largest `r` with `r * r <= x`.
pub fn isqrt(x: u64) u32 {
    if (x == 0) return 0;
    // Shifted by an even count so its top two bits are not both zero; the
    // root of the shifted number, shifted back by half that count, is the
    // root of `x` rounded down.
    const shift: u6 = @intCast(@clz(x) & 0x3e);
    const shifted = x << shift;
    // An estimate within 2^-7 at or above the root, then two of Newton's
    // steps, each squaring the error and staying at or above the root
    // rounded down. The error is largest at the first number of a top byte,
    // and there, worked out exactly for all 192, two steps leave less than
    // one above the root: at most one above the root rounded down.
    var root: u64 = @as(u64, root_estimates[(shifted >> 56) - 64]) << 16;
    root = (root + shifted / root) / 2;
    root = (root + shifted / root) / 2;
    root -= @intFromBool(@as(u128, root) * root > shifted);
    return @intCast(root >> (shift / 2));
}

/// The largest `r` with `r * r <= x`, for 128 bits.
pub fn isqrtWide(x: u128) u64 {
    if (x <= std.math.maxInt(u64)) return isqrt(@intCast(x));
    const bit_count: u8 = 128 - @as(u8, @clz(x));
    var root: u128 = @as(u128, 1) << @intCast((bit_count + 1) / 2);
    while (true) {
        const next = (root + x / root) / 2;
        if (next >= root) return @intCast(root);
        root = next;
    }
}

/// A distance in 1/256 millimetre from its square in square millimetres,
/// rounded down. Whole millimetres above 2^24 millimetres.
pub inline fn length(dist_sqr: u64) u64 {
    if (dist_sqr < (1 << 48)) return isqrt(dist_sqr << 16);
    return @as(u64, isqrt(dist_sqr)) << 8;
}

pub inline fn vadd(a: [3]i32, b: [3]i32) [3]i32 {
    return .{ a[0] + b[0], a[1] + b[1], a[2] + b[2] };
}

pub inline fn vsub(a: [3]i32, b: [3]i32) [3]i32 {
    return .{ a[0] - b[0], a[1] - b[1], a[2] - b[2] };
}

pub inline fn vmin(a: [3]i32, b: [3]i32) [3]i32 {
    return .{ @min(a[0], b[0]), @min(a[1], b[1]), @min(a[2], b[2]) };
}

pub inline fn vmax(a: [3]i32, b: [3]i32) [3]i32 {
    return .{ @max(a[0], b[0]), @max(a[1], b[1]), @max(a[2], b[2]) };
}

pub inline fn vequal(a: [3]i32, b: [3]i32) bool {
    return a[0] == b[0] and a[1] == b[1] and a[2] == b[2];
}

inline fn sqr(a: i32) u64 {
    const wide: i64 = a;
    return @intCast(wide * wide);
}

/// The squared length of a difference of two positions.
pub inline fn vlenSqr(v: [3]i32) u64 {
    return sqr(v[0]) + sqr(v[1]) + sqr(v[2]);
}

pub inline fn vdistSqr(a: [3]i32, b: [3]i32) u64 {
    return vlenSqr(vsub(a, b));
}

pub inline fn vdist2dSqr(a: [3]i32, b: [3]i32) u64 {
    return sqr(a[0] - b[0]) + sqr(a[2] - b[2]);
}

/// The distance between two positions in 1/256 millimetre.
pub inline fn vdist(a: [3]i32, b: [3]i32) u64 {
    return length(vdistSqr(a, b));
}

pub inline fn vdot2d(u: [3]i32, v: [3]i32) i64 {
    return @as(i64, u[0]) * v[0] + @as(i64, u[2]) * v[2];
}

/// The xz perp product: positive when `v` is clockwise of `u` seen from
/// above.
pub inline fn vperp2d(u: [3]i32, v: [3]i32) i64 {
    return @as(i64, u[2]) * v[0] - @as(i64, u[0]) * v[2];
}

/// Twice the signed xz area of triangle `a`, `b`, `c`.
pub inline fn triArea2d(a: [3]i32, b: [3]i32, c: [3]i32) i64 {
    const abx: i64 = b[0] - a[0];
    const abz: i64 = b[2] - a[2];
    const acx: i64 = c[0] - a[0];
    const acz: i64 = c[2] - a[2];
    return acx * abz - abx * acz;
}

/// The point at `t` along `a`..`b`, each axis rounded to the nearest
/// millimetre, halves away from zero. `t` must lie within 0..1, which keeps
/// the result within the box of `a` and `b`.
pub inline fn lerp(a: [3]i32, b: [3]i32, t: Fraction) [3]i32 {
    return .{
        lerpAxis(a[0], b[0], t),
        lerpAxis(a[1], b[1], t),
        lerpAxis(a[2], b[2], t),
    };
}

inline fn lerpAxis(a: i32, b: i32, t: Fraction) i32 {
    const span: i128 = @as(i64, b) - a;
    return @intCast(a + divRound(span * t.num, t.den));
}

/// A direction of length one in Q16.16 from a difference of two positions;
/// zero for the zero vector. Inline: a caller builds `v` just before, and a
/// call would read it back from memory before the stores of its parts land.
pub inline fn normalize(v: [3]i32) [3]i32 {
    const len_sqr = vlenSqr(v);
    if (len_sqr == 0) return .{ 0, 0, 0 };
    // The length in 1/65536 millimetre.
    const len: i128 = isqrtWide(@as(u128, len_sqr) << 32);
    return .{
        @intCast(divRound(@as(i128, v[0]) << 32, len)),
        @intCast(divRound(@as(i128, v[1]) << 32, len)),
        @intCast(divRound(@as(i128, v[2]) << 32, len)),
    };
}

/// Whether two quantized boxes overlap, touching counted.
pub inline fn overlapQuantBounds(amin: [3]u16, amax: [3]u16, bmin: [3]u16, bmax: [3]u16) bool {
    var overlap = true;
    overlap = if (amin[0] > bmax[0] or amax[0] < bmin[0]) false else overlap;
    overlap = if (amin[1] > bmax[1] or amax[1] < bmin[1]) false else overlap;
    overlap = if (amin[2] > bmax[2] or amax[2] < bmin[2]) false else overlap;
    return overlap;
}

/// Whether two boxes overlap, touching counted.
pub inline fn overlapBounds(amin: [3]i32, amax: [3]i32, bmin: [3]i32, bmax: [3]i32) bool {
    var overlap = true;
    overlap = if (amin[0] > bmax[0] or amax[0] < bmin[0]) false else overlap;
    overlap = if (amin[1] > bmax[1] or amax[1] < bmin[1]) false else overlap;
    overlap = if (amin[2] > bmax[2] or amax[2] < bmin[2]) false else overlap;
    return overlap;
}

/// Where a segment crosses a convex polygon in the xz plane.
pub const SegmentPolyHit = struct {
    /// False when the segment misses the polygon; the other fields then
    /// hold how far the clipping got.
    hit: bool,
    /// Where along the segment it enters the polygon.
    t_min: Fraction,
    /// Where along the segment it leaves the polygon.
    t_max: Fraction,
    /// The polygon edge the segment enters through, -1 when it starts
    /// inside.
    seg_min: i32,
    /// The polygon edge the segment leaves through, -1 when it ends inside.
    seg_max: i32,
};

/// Clips the segment `p0`..`p1` against the convex polygon `verts`.
pub fn intersectSegmentPoly2d(p0: [3]i32, p1: [3]i32, verts: []const [3]i32) SegmentPolyHit {
    var result: SegmentPolyHit = .{
        .hit = false,
        .t_min = .zero,
        .t_max = .one,
        .seg_min = -1,
        .seg_max = -1,
    };

    const dir = vsub(p1, p0);

    var j = verts.len -% 1;
    for (0..verts.len) |i| {
        defer j = i;
        const edge = vsub(verts[i], verts[j]);
        const diff = vsub(p0, verts[j]);
        const n = vperp2d(edge, diff);
        const d = vperp2d(dir, edge);
        if (d == 0) {
            // The segment is parallel to this edge.
            if (n < 0) return result;
            continue;
        }
        const t: Fraction = .init(n, d);
        if (d < 0) {
            // The segment is entering across this edge. A start inside the
            // edge's half-plane puts `t` at or below zero, so at or below
            // `t_min`, with no product to compare.
            if (n < 0 and t.greaterThan(result.t_min)) {
                result.t_min = t;
                result.seg_min = @intCast(j);
                // The segment enters after leaving the polygon.
                if (result.t_min.greaterThan(result.t_max)) return result;
            }
        } else {
            // The segment is leaving across this edge. An end inside the
            // edge's half-plane puts `t` at or above one, so at or above
            // `t_max`.
            if (n < d and t.lessThan(result.t_max)) {
                result.t_max = t;
                result.seg_max = @intCast(j);
                // The segment leaves before entering the polygon.
                if (result.t_max.lessThan(result.t_min)) return result;
            }
        }
    }

    result.hit = true;
    return result;
}

/// Whether the squared xz distance from `pt` to the segment `p`..`q`, as
/// `distancePointSegmentSqr2d` rounds it, is at most `limit_sqr`. It
/// compares products where that function divides.
pub fn segmentWithinSqr2d(pt: [3]i32, p: [3]i32, q: [3]i32, limit_sqr: u64) bool {
    const pqx: i64 = q[0] - p[0];
    const pqz: i64 = q[2] - p[2];
    const dx: i64 = pt[0] - p[0];
    const dz: i64 = pt[2] - p[2];
    const len_sqr = pqx * pqx + pqz * pqz;
    const along = pqx * dx + pqz * dz;
    if (along <= 0) return vdist2dSqr(pt, p) <= limit_sqr;
    if (along >= len_sqr) return vdist2dSqr(pt, q) <= limit_sqr;
    // The square rounded down is at most the limit when the square itself
    // is below the limit plus one: `across^2 < (limit + 1) * len_sqr`, both
    // sides within 2^126.
    const across: i128 = pqx * dz - pqz * dx;
    const across_sqr: u128 = @intCast(across * across);
    return across_sqr < (@as(u128, limit_sqr) + 1) * @as(u64, @intCast(len_sqr));
}

pub const PointSegmentDistance = struct {
    /// Rounded down to whole square millimetres.
    dist_sqr: u64,
    /// Where the closest point lies along the segment, 0 at `p`, 1 at `q`.
    t: Fraction,
};

/// The squared xz distance from `pt` to the segment `p`..`q`. A segment of
/// no length gives the distance to its point, at `t` zero.
pub fn distancePointSegmentSqr2d(pt: [3]i32, p: [3]i32, q: [3]i32) PointSegmentDistance {
    const pqx: i64 = q[0] - p[0];
    const pqz: i64 = q[2] - p[2];
    const dx: i64 = pt[0] - p[0];
    const dz: i64 = pt[2] - p[2];
    const len_sqr = pqx * pqx + pqz * pqz;
    const along = pqx * dx + pqz * dz;
    if (along <= 0) return .{ .dist_sqr = vdist2dSqr(pt, p), .t = .zero };
    if (along >= len_sqr) return .{ .dist_sqr = vdist2dSqr(pt, q), .t = .one };
    // Beside the segment: the distance to its line.
    const across: i128 = pqx * dz - pqz * dx;
    // A square below 2^62 divides in 64 bits.
    const dist_sqr: u64 = if (@abs(across) < 1 << 31)
        @as(u64, @intCast(across * across)) / @as(u64, @intCast(len_sqr))
    else
        @intCast(@divTrunc(across * across, len_sqr));
    return .{ .dist_sqr = dist_sqr, .t = .{ .num = along, .den = len_sqr } };
}

/// The height of triangle `a`, `b`, `c` under `p`, rounded to the nearest
/// millimetre; null when `p` is outside it in the xz plane or the triangle
/// has no area there.
pub fn closestHeightOnTriangle(p: [3]i32, a: [3]i32, b: [3]i32, c: [3]i32) ?i32 {
    const v0 = vsub(c, a);
    const v1 = vsub(b, a);
    const v2 = vsub(p, a);

    // Scaled barycentric coordinates.
    var denom = @as(i64, v0[0]) * v1[2] - @as(i64, v0[2]) * v1[0];
    if (denom == 0) return null;

    var u = @as(i64, v1[2]) * v2[0] - @as(i64, v1[0]) * v2[2];
    var v = @as(i64, v0[0]) * v2[2] - @as(i64, v0[2]) * v2[0];

    if (denom < 0) {
        denom = -denom;
        u = -u;
        v = -v;
    }

    // Inside the triangle: the interpolated height.
    if (u >= 0 and v >= 0 and u + v <= denom) {
        // Most rises fit 64 bits, where the processor divides in one
        // instruction; the others take 128.
        const rise_u = @mulWithOverflow(@as(i64, v0[1]), u);
        const rise_v = @mulWithOverflow(@as(i64, v1[1]), v);
        const rise = @addWithOverflow(rise_u[0], rise_v[0]);
        if (rise_u[1] | rise_v[1] | rise[1] == 0) {
            return @intCast(a[1] + divRoundNarrow(rise[0], denom));
        }
        const wide_rise = @as(i128, v0[1]) * u + @as(i128, v1[1]) * v;
        return @intCast(a[1] + divRound(wide_rise, denom));
    }
    return null;
}

/// Whether the edge `vj`..`vi` crosses the line through `pt` along x on the
/// far side of `pt`: one step of the crossing count of a point in a polygon.
inline fn crossesBeyond(pt: [3]i32, vi: [3]i32, vj: [3]i32) bool {
    if ((vi[2] > pt[2]) == (vj[2] > pt[2])) return false;
    const rise: i64 = vj[2] - vi[2];
    const left = @as(i64, pt[0] - vi[0]) * rise;
    const right = @as(i64, vj[0] - vi[0]) * @as(i64, pt[2] - vi[2]);
    return if (rise > 0) left < right else left > right;
}

/// Whether `pt` lies inside the polygon `verts` in the xz plane.
pub fn pointInPolygon(pt: [3]i32, verts: []const [3]i32) bool {
    var inside = false;
    var j = verts.len -% 1;
    for (0..verts.len) |i| {
        defer j = i;
        if (crossesBeyond(pt, verts[i], verts[j])) inside = !inside;
    }
    return inside;
}

/// Whether `pt` lies inside the polygon `verts` in the xz plane, with the
/// distance to each edge: entry `j` of `edge_dist_sqr` and `edge_t` is for
/// the edge from vertex `j` to the next.
pub fn distancePointPolyEdgesSqr(
    pt: [3]i32,
    verts: []const [3]i32,
    edge_dist_sqr: []u64,
    edge_t: []Fraction,
) bool {
    var inside = false;
    var j = verts.len -% 1;
    for (0..verts.len) |i| {
        defer j = i;
        if (crossesBeyond(pt, verts[i], verts[j])) inside = !inside;
        const distance = distancePointSegmentSqr2d(pt, verts[j], verts[i]);
        edge_dist_sqr[j] = distance.dist_sqr;
        edge_t[j] = distance.t;
    }
    return inside;
}

const Range = struct { min: i64, max: i64 };

fn projectPoly(axis: [3]i32, poly: []const [3]i32) Range {
    var range: Range = undefined;
    range.min = vdot2d(axis, poly[0]);
    range.max = range.min;
    for (poly[1..]) |vert| {
        const d = vdot2d(axis, vert);
        range.min = @min(range.min, d);
        range.max = @max(range.max, d);
    }
    return range;
}

/// Whether two convex polygons overlap in the xz plane by more than a
/// touch: their projections on every edge normal must overlap by over 100
/// square millimetres, the normals taken at the edges' own lengths.
pub fn overlapPolyPoly2d(polya: []const [3]i32, polyb: []const [3]i32) bool {
    const eps = 100;

    inline for (.{ polya, polyb }) |poly| {
        var j = poly.len -% 1;
        for (0..poly.len) |i| {
            defer j = i;
            const va = poly[j];
            const vb = poly[i];
            const n: [3]i32 = .{ vb[2] - va[2], 0, -(vb[0] - va[0]) };
            const a_range = projectPoly(n, polya);
            const b_range = projectPoly(n, polyb);
            // A separating axis was found.
            if (a_range.min + eps > b_range.max or a_range.max - eps < b_range.min) return false;
        }
    }
    return true;
}

/// The point of the convex polygon `pts` that the draws `s` and `t` pick,
/// each read as a number in 0..1 with 2^32 as one. `areas` is scratch, as
/// long as `pts`.
pub fn randomPointInConvexPoly(pts: []const [3]i32, areas: []i64, s: u32, t: u32) [3]i32 {
    // The triangle fan's areas, each counted as at least 1000.
    var area_sum: u64 = 0;
    var i: usize = 2;
    while (i < pts.len) : (i += 1) {
        areas[i] = triArea2d(pts[0], pts[i - 1], pts[i]);
        area_sum += @intCast(@max(1000, areas[i]));
    }
    // The triangle `s` falls in, by area, and how far into it.
    const threshold: i64 = @intCast((@as(u128, s) * area_sum) >> 32);
    var acc: i64 = 0;
    var u: Fraction = .one;
    var tri = pts.len - 1;
    i = 2;
    while (i < pts.len) : (i += 1) {
        const dacc = areas[i];
        if (threshold >= acc and threshold < acc + dacc) {
            u = .{ .num = threshold - acc, .den = dacc };
            tri = i;
            break;
        }
        acc += dacc;
    }

    // Weights of the triangle's corners that sum to 2^32.
    const v: u64 = isqrt(@as(u64, t) << 32);
    const a: u64 = (1 << 32) - v;
    const c: u64 = @intCast(@divTrunc(@as(i128, v) * u.num, u.den));
    const b: u64 = v - c;
    const pa = pts[0];
    const pb = pts[tri - 1];
    const pc = pts[tri];

    var out: [3]i32 = undefined;
    for (&out, pa, pb, pc) |*axis, axis_a, axis_b, axis_c| {
        const sum = @as(i128, a) * axis_a + @as(i128, b) * axis_b + @as(i128, c) * axis_c;
        axis.* = @intCast(divRound(sum, 1 << 32));
    }
    return out;
}

inline fn vperpXz(a: [3]i32, b: [3]i32) i64 {
    return @as(i64, a[0]) * b[2] - @as(i64, a[2]) * b[0];
}

/// Where two lines cross: `s` along `ap`..`aq` and `t` along `bp`..`bq`.
pub const SegmentSegmentHit = struct {
    s: Fraction,
    t: Fraction,
};

/// Where the lines through `ap`..`aq` and `bp`..`bq` cross in the xz plane;
/// null when they are parallel.
pub fn intersectSegmentSegment2d(
    ap: [3]i32,
    aq: [3]i32,
    bp: [3]i32,
    bq: [3]i32,
) ?SegmentSegmentHit {
    const u = vsub(aq, ap);
    const v = vsub(bq, bp);
    const w = vsub(ap, bp);
    const d = vperpXz(u, v);
    if (d == 0) return null;
    return .{ .s = .init(vperpXz(v, w), d), .t = .init(vperpXz(u, w), d) };
}

/// The smallest power of two at or above `value`; 0 for 0 and for values
/// above 2^31.
pub inline fn nextPow2(value: u32) u32 {
    var v = value;
    v -%= 1;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v +%= 1;
    return v;
}

/// The position of the highest set bit; 0 for 0.
pub inline fn ilog2(value: u32) u32 {
    return if (value == 0) 0 else 31 - @as(u32, @clz(value));
}

/// `x` rounded up to a multiple of four.
pub inline fn align4(x: usize) usize {
    return (x + 3) & ~@as(usize, 3);
}

/// The side opposite to `side`, of the eight around a tile.
pub inline fn oppositeTile(side: i32) i32 {
    return (side + 4) & 0x7;
}

// ----------------------------------------------------------------------------
// Tests
// ----------------------------------------------------------------------------

const testing = std.testing;

fn expectRoot(x: u64) !void {
    const root: u64 = isqrt(x);
    try testing.expect(root * root <= x);
    // The next root's square is past `x`, or past 64 bits.
    const next = root + 1;
    const next_sqr = @as(u128, next) * next;
    try testing.expect(next_sqr > x);
}

fn expectRootWide(x: u128) !void {
    const root: u128 = isqrtWide(x);
    try testing.expect(root * root <= x);
    const next = root + 1;
    // 2^64 squared does not fit; it is past every `x`.
    if (next <= std.math.maxInt(u64)) try testing.expect(next * next > x);
}

test "isqrt is the exact root rounded down" {
    var x: u64 = 0;
    while (x < 70_000) : (x += 1) try expectRoot(x);

    // Around every power of two and every square of a power of two less one.
    var shift: u7 = 1;
    while (shift < 64) : (shift += 1) {
        const power = @as(u64, 1) << @intCast(shift);
        try expectRoot(power - 1);
        try expectRoot(power);
        try expectRoot(power + 1);
    }
    try expectRoot(std.math.maxInt(u64));
    try testing.expectEqual(@as(u32, std.math.maxInt(u32)), isqrt(std.math.maxInt(u64)));

    // The ends of every top byte of the estimate table, at every even shift,
    // where the estimate is furthest from the root.
    for (64..256) |byte| {
        var even_shift: u7 = 0;
        while (even_shift < 64) : (even_shift += 2) {
            const first = @as(u64, byte) << 56 >> @intCast(even_shift);
            const last = (@as(u64, byte) << 56 | ((1 << 56) - 1)) >> @intCast(even_shift);
            try expectRoot(first);
            try expectRoot(first -| 1);
            try expectRoot(last);
        }
    }

    const seed: u64 = 0x5eed_1501;
    errdefer std.debug.print("seed {x}\n", .{seed});
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();
    for (0..100_000) |_| {
        // Every magnitude equally often.
        const value = random.int(u64) >> random.int(u6);
        try expectRoot(value);
        // A square and its neighbours, where the root steps.
        const root: u64 = random.int(u32);
        try expectRoot(root * root);
        if (root > 0) try expectRoot(root * root - 1);
    }
}

test "isqrtWide is the exact root rounded down" {
    var shift: u8 = 1;
    while (shift < 128) : (shift += 1) {
        const power = @as(u128, 1) << @intCast(shift);
        try expectRootWide(power - 1);
        try expectRootWide(power);
        try expectRootWide(power + 1);
    }
    try expectRootWide(std.math.maxInt(u128));
    try testing.expectEqual(@as(u64, std.math.maxInt(u64)), isqrtWide(std.math.maxInt(u128)));

    const seed: u64 = 0x5eed_1502;
    errdefer std.debug.print("seed {x}\n", .{seed});
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();
    for (0..20_000) |_| {
        const value = random.int(u128) >> random.int(u7);
        try expectRootWide(value);
        const root: u128 = random.int(u64);
        try expectRootWide(root * root);
        if (root > 0) try expectRootWide(root * root - 1);
    }
}

test "length never decreases and joins at its change of scale" {
    try testing.expectEqual(@as(u64, 0), length(0));
    try testing.expectEqual(@as(u64, 256), length(1));
    try testing.expectEqual(@as(u64, 256 * 1000), length(1000 * 1000));
    const edge: u64 = 1 << 48;
    try testing.expectEqual(@as(u64, (1 << 32) - 1), length(edge - 1));
    try testing.expectEqual(@as(u64, 1 << 32), length(edge));

    const seed: u64 = 0x5eed_1503;
    errdefer std.debug.print("seed {x}\n", .{seed});
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();
    for (0..10_000) |_| {
        const low = (random.int(u64) >> 2) >> random.int(u6);
        const high = low + (random.int(u32) >> random.int(u5));
        try testing.expect(length(low) <= length(high));
    }
}

test "divRound rounds halves away from zero" {
    try testing.expectEqual(@as(i128, 2), divRound(3, 2));
    try testing.expectEqual(@as(i128, -2), divRound(-3, 2));
    try testing.expectEqual(@as(i128, 1), divRound(1, 2));
    try testing.expectEqual(@as(i128, -1), divRound(-1, 2));
    try testing.expectEqual(@as(i128, 0), divRound(1, 3));
    try testing.expectEqual(@as(i128, 1), divRound(2, 3));
    try testing.expectEqual(@as(i128, -1), divRound(-2, 3));
    try testing.expectEqual(@as(i128, 0), divRound(0, 7));
    try testing.expectEqual(@as(i128, 5), divRound(35, 7));
}

test "fractions compare exactly" {
    const third: Fraction = .init(1, 3);
    const two_sixths: Fraction = .init(-2, -6);
    try testing.expectEqual(std.math.Order.eq, third.order(two_sixths));
    try testing.expect(third.lessThan(.half));
    try testing.expect(Fraction.one.lessThan(.never));
    try testing.expect(!Fraction.never.lessThan(.never));
    try testing.expect(Fraction.never.atLeastOne());
    try testing.expect(Fraction.init(1, -2).isNegative());
    try testing.expectEqual(Fraction.zero, Fraction.init(-5, 3).clamped());
    try testing.expectEqual(Fraction.one, Fraction.init(5, 3).clamped());

    // The largest parts a cross product gives still compare without overflow.
    const big: i64 = 1 << 61;
    const below: Fraction = .{ .num = big - 1, .den = big };
    const above: Fraction = .{ .num = big, .den = big - 1 };
    try testing.expect(below.lessThan(above));
}

test "lerp rounds each axis to the nearest millimetre" {
    const a: [3]i32 = .{ 0, 0, 0 };
    const b: [3]i32 = .{ 1, -1, 3 };
    try testing.expectEqual([3]i32{ 1, -1, 2 }, lerp(a, b, .half));
    try testing.expectEqual(a, lerp(a, b, .zero));
    try testing.expectEqual(b, lerp(a, b, .one));

    // Across the whole coordinate range.
    const low: [3]i32 = .{ -coord_max, -coord_max, -coord_max };
    const high: [3]i32 = .{ coord_max, coord_max, coord_max };
    try testing.expectEqual([3]i32{ 0, 0, 0 }, lerp(low, high, .half));
    const far: Fraction = .{ .num = (1 << 61) - 1, .den = 1 << 61 };
    try testing.expectEqual(high, lerp(low, high, far));
}

test "the distance to a segment is exact at its ends and beside it" {
    const p: [3]i32 = .{ 0, 5, 0 };
    const q: [3]i32 = .{ 10, 7, 0 };
    var d = distancePointSegmentSqr2d(.{ -3, 0, 4 }, p, q);
    try testing.expectEqual(@as(u64, 25), d.dist_sqr);
    try testing.expectEqual(Fraction.zero, d.t);
    d = distancePointSegmentSqr2d(.{ 13, 0, -4 }, p, q);
    try testing.expectEqual(@as(u64, 25), d.dist_sqr);
    try testing.expectEqual(Fraction.one, d.t);
    d = distancePointSegmentSqr2d(.{ 4, 0, 6 }, p, q);
    try testing.expectEqual(@as(u64, 36), d.dist_sqr);
    try testing.expectEqual(std.math.Order.eq, d.t.order(.init(2, 5)));
    // A segment of no length.
    d = distancePointSegmentSqr2d(.{ 3, 0, 4 }, p, p);
    try testing.expectEqual(@as(u64, 25), d.dist_sqr);
    try testing.expectEqual(Fraction.zero, d.t);
}

test "segmentWithinSqr2d agrees with the rounded distance at and around it" {
    const maxInt = std.math.maxInt;
    var prng: std.Random.DefaultPrng = .init(0x7365676d);
    const random = prng.random();
    // Small and full-range coordinates, so both sides of each product test
    // are reached.
    for ([_]i32{ 1000, coord_max }) |range| {
        for (0..20_000) |_| {
            var points: [3][3]i32 = undefined;
            for (&points) |*point| {
                for (point) |*c| c.* = random.intRangeAtMost(i32, -range, range);
            }
            const dist_sqr = distancePointSegmentSqr2d(points[0], points[1], points[2]).dist_sqr;
            const limits = [_]u64{ dist_sqr -| 1, dist_sqr, dist_sqr +| 1, 0, maxInt(u64) };
            for (limits) |limit| {
                try testing.expectEqual(
                    dist_sqr <= limit,
                    segmentWithinSqr2d(points[0], points[1], points[2], limit),
                );
            }
        }
    }
}

test "normalize gives length one in Q16.16" {
    try testing.expectEqual([3]i32{ 0, 0, 0 }, normalize(.{ 0, 0, 0 }));
    try testing.expectEqual([3]i32{ 65536, 0, 0 }, normalize(.{ 1, 0, 0 }));
    try testing.expectEqual([3]i32{ 0, 0, -65536 }, normalize(.{ 0, 0, -coord_max * 2 }));
    // 3-4-5: 0.6 and 0.8.
    try testing.expectEqual([3]i32{ 39322, 0, 52429 }, normalize(.{ 3, 0, 4 }));
}
