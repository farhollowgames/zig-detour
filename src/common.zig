//! Vector and geometry helpers shared by the navigation mesh, its queries,
//! the crowd and the tile cache.
//!
//! Positions are `[3]f32` with y up. Functions named `2d` work on the xz
//! plane and ignore y. Every expression keeps upstream Detour's operation
//! order, so results match it bit for bit.

const std = @import("std");
const build_options = @import("build_options");

/// The smaller of two values as C's `a < b ? a : b` picks it: `b` when
/// either is NaN, and `b` for zeros of either sign. `@min` picks otherwise
/// for floats, so float code that must match upstream uses this.
pub inline fn min(comptime T: type, a: T, b: T) T {
    return if (a < b) a else b;
}

/// The larger of two values as C's `a > b ? a : b` picks it; see `min`.
pub inline fn max(comptime T: type, a: T, b: T) T {
    return if (a > b) a else b;
}

/// The magnitude as C's `a < 0 ? -a : a` gives it: negative zero and NaN
/// pass through unchanged, where `@abs` clears their sign.
pub inline fn abs(comptime T: type, a: T) T {
    return if (a < 0) -a else a;
}

pub inline fn sqr(comptime T: type, a: T) T {
    return a * a;
}

/// `v` limited to `low..high` as C's nested conditional does it: a NaN `v`
/// passes through.
pub inline fn clamp(comptime T: type, v: T, low: T, high: T) T {
    return if (v < low) low else if (v > high) high else v;
}

/// Whether `x` is finite. Always true under the `fast-math` build option,
/// which drops the checks as upstream's RC_FAST_MATH does.
pub inline fn isFinite(x: f32) bool {
    if (build_options.fast_math) return true;
    return std.math.isFinite(x);
}

/// A float as the integer upstream's C cast gives on x86-64, the platform
/// the reference is checked on. In range that is the value truncated toward
/// zero. Where C leaves the cast undefined, the machine's conversion decides:
/// NaN and values beyond `i32` give `minInt(i32)`, and a `T` narrower than 32
/// bits keeps the low bits of that 32-bit result, so 65536.0 as `u16` is 0.
pub inline fn truncToInt(comptime T: type, x: f32) T {
    comptime std.debug.assert(@bitSizeOf(T) <= 32 and T != u32);
    const in_range = x >= -2147483648.0 and x < 2147483648.0;
    const wide: i32 = if (in_range) @trunc(x) else std.math.minInt(i32);
    if (T == i32) return wide;
    const Bits = @Int(.unsigned, @bitSizeOf(T));
    return @bitCast(@as(Bits, @truncate(@as(u32, @bitCast(wide)))));
}

pub inline fn vcross(v1: [3]f32, v2: [3]f32) [3]f32 {
    const x = v1[1] * v2[2] - v1[2] * v2[1];
    const y = v1[2] * v2[0] - v1[0] * v2[2];
    const z = v1[0] * v2[1] - v1[1] * v2[0];
    return .{ x, y, z };
}

pub inline fn vdot(v1: [3]f32, v2: [3]f32) f32 {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

/// `v1 + v2 * s`.
pub inline fn vmad(v1: [3]f32, v2: [3]f32, s: f32) [3]f32 {
    return .{ v1[0] + v2[0] * s, v1[1] + v2[1] * s, v1[2] + v2[2] * s };
}

/// The point a fraction `t` of the way from `v1` to `v2`.
pub inline fn vlerp(v1: [3]f32, v2: [3]f32, t: f32) [3]f32 {
    return .{
        v1[0] + (v2[0] - v1[0]) * t,
        v1[1] + (v2[1] - v1[1]) * t,
        v1[2] + (v2[2] - v1[2]) * t,
    };
}

pub inline fn vadd(v1: [3]f32, v2: [3]f32) [3]f32 {
    return .{ v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2] };
}

pub inline fn vsub(v1: [3]f32, v2: [3]f32) [3]f32 {
    return .{ v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2] };
}

pub inline fn vscale(v: [3]f32, t: f32) [3]f32 {
    return .{ v[0] * t, v[1] * t, v[2] * t };
}

/// The componentwise minimum, `mn` winning ties and NaNs as `min` does.
pub inline fn vmin(mn: [3]f32, v: [3]f32) [3]f32 {
    return .{ min(f32, mn[0], v[0]), min(f32, mn[1], v[1]), min(f32, mn[2], v[2]) };
}

pub inline fn vmax(mx: [3]f32, v: [3]f32) [3]f32 {
    return .{ max(f32, mx[0], v[0]), max(f32, mx[1], v[1]), max(f32, mx[2], v[2]) };
}

pub inline fn vlen(v: [3]f32) f32 {
    return @sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

pub inline fn vlenSqr(v: [3]f32) f32 {
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

pub inline fn vdist(v1: [3]f32, v2: [3]f32) f32 {
    const dx = v2[0] - v1[0];
    const dy = v2[1] - v1[1];
    const dz = v2[2] - v1[2];
    return @sqrt(dx * dx + dy * dy + dz * dz);
}

pub inline fn vdistSqr(v1: [3]f32, v2: [3]f32) f32 {
    const dx = v2[0] - v1[0];
    const dy = v2[1] - v1[1];
    const dz = v2[2] - v1[2];
    return dx * dx + dy * dy + dz * dz;
}

pub inline fn vdist2d(v1: [3]f32, v2: [3]f32) f32 {
    const dx = v2[0] - v1[0];
    const dz = v2[2] - v1[2];
    return @sqrt(dx * dx + dz * dz);
}

pub inline fn vdist2dSqr(v1: [3]f32, v2: [3]f32) f32 {
    const dx = v2[0] - v1[0];
    const dz = v2[2] - v1[2];
    return dx * dx + dz * dz;
}

/// `v` scaled to unit length. A zero vector gives NaNs, as upstream.
pub inline fn vnormalize(v: [3]f32) [3]f32 {
    const d = 1.0 / @sqrt(sqr(f32, v[0]) + sqr(f32, v[1]) + sqr(f32, v[2]));
    return .{ v[0] * d, v[1] * d, v[2] * d };
}

/// Whether two points are closer than 1/16384 units.
pub inline fn vequal(p0: [3]f32, p1: [3]f32) bool {
    const threshold = comptime sqr(f32, 1.0 / 16384.0);
    return vdistSqr(p0, p1) < threshold;
}

pub inline fn visfinite(v: [3]f32) bool {
    return isFinite(v[0]) and isFinite(v[1]) and isFinite(v[2]);
}

pub inline fn visfinite2d(v: [3]f32) bool {
    return isFinite(v[0]) and isFinite(v[2]);
}

pub inline fn vdot2d(u: [3]f32, v: [3]f32) f32 {
    return u[0] * v[0] + u[2] * v[2];
}

/// The xz perp product: positive when `v` is clockwise of `u` seen from
/// above.
pub inline fn vperp2d(u: [3]f32, v: [3]f32) f32 {
    return u[2] * v[0] - u[0] * v[2];
}

/// Twice the signed xz area of triangle `a`, `b`, `c`.
pub inline fn triArea2d(a: [3]f32, b: [3]f32, c: [3]f32) f32 {
    const abx = b[0] - a[0];
    const abz = b[2] - a[2];
    const acx = c[0] - a[0];
    const acz = c[2] - a[2];
    return acx * abz - abx * acz;
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
pub inline fn overlapBounds(amin: [3]f32, amax: [3]f32, bmin: [3]f32, bmax: [3]f32) bool {
    var overlap = true;
    overlap = if (amin[0] > bmax[0] or amax[0] < bmin[0]) false else overlap;
    overlap = if (amin[1] > bmax[1] or amax[1] < bmin[1]) false else overlap;
    overlap = if (amin[2] > bmax[2] or amax[2] < bmin[2]) false else overlap;
    return overlap;
}

/// The point of triangle `a`, `b`, `c` closest to `p`.
pub fn closestPointOnTriangle(p: [3]f32, a: [3]f32, b: [3]f32, c: [3]f32) [3]f32 {
    // Check if p is in the vertex region outside a.
    const ab = vsub(b, a);
    const ac = vsub(c, a);
    const ap = vsub(p, a);
    const d1 = vdot(ab, ap);
    const d2 = vdot(ac, ap);
    if (d1 <= 0.0 and d2 <= 0.0) return a;

    // Check if p is in the vertex region outside b.
    const bp = vsub(p, b);
    const d3 = vdot(ab, bp);
    const d4 = vdot(ac, bp);
    if (d3 >= 0.0 and d4 <= d3) return b;

    // Check if p is in the edge region of ab; if so return its projection.
    const vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0) {
        const v = d1 / (d1 - d3);
        return .{ a[0] + v * ab[0], a[1] + v * ab[1], a[2] + v * ab[2] };
    }

    // Check if p is in the vertex region outside c.
    const cp = vsub(p, c);
    const d5 = vdot(ab, cp);
    const d6 = vdot(ac, cp);
    if (d6 >= 0.0 and d5 <= d6) return c;

    // Check if p is in the edge region of ac.
    const vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0) {
        const w = d2 / (d2 - d6);
        return .{ a[0] + w * ac[0], a[1] + w * ac[1], a[2] + w * ac[2] };
    }

    // Check if p is in the edge region of bc.
    const va = d3 * d6 - d5 * d4;
    if (va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0) {
        const w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return .{
            b[0] + w * (c[0] - b[0]),
            b[1] + w * (c[1] - b[1]),
            b[2] + w * (c[2] - b[2]),
        };
    }

    // p is inside the face region.
    const denom = 1.0 / (va + vb + vc);
    const v = vb * denom;
    const w = vc * denom;
    return .{
        a[0] + ab[0] * v + ac[0] * w,
        a[1] + ab[1] * v + ac[1] * w,
        a[2] + ab[2] * v + ac[2] * w,
    };
}

/// Where a segment crosses a convex polygon in the xz plane.
pub const SegmentPolyHit = struct {
    /// False when the segment misses the polygon; the other fields then
    /// hold how far the clipping got.
    hit: bool,
    /// The parameter along the segment where it enters the polygon.
    t_min: f32,
    /// The parameter along the segment where it leaves the polygon.
    t_max: f32,
    /// The polygon edge the segment enters through, -1 when it starts
    /// inside.
    seg_min: i32,
    /// The polygon edge the segment leaves through, -1 when it ends inside.
    seg_max: i32,
};

/// Clips the segment `p0`..`p1` against the convex polygon `verts`.
pub fn intersectSegmentPoly2d(p0: [3]f32, p1: [3]f32, verts: []const [3]f32) SegmentPolyHit {
    const eps: f32 = 0.000001;

    var result: SegmentPolyHit = .{
        .hit = false,
        .t_min = 0,
        .t_max = 1,
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
        if (@abs(d) < eps) {
            // The segment is nearly parallel to this edge.
            if (n < 0) return result;
            continue;
        }
        const t = n / d;
        if (d < 0) {
            // The segment is entering across this edge.
            if (t > result.t_min) {
                result.t_min = t;
                result.seg_min = @intCast(j);
                // The segment enters after leaving the polygon.
                if (result.t_min > result.t_max) return result;
            }
        } else {
            // The segment is leaving across this edge.
            if (t < result.t_max) {
                result.t_max = t;
                result.seg_max = @intCast(j);
                // The segment leaves before entering the polygon.
                if (result.t_max < result.t_min) return result;
            }
        }
    }

    result.hit = true;
    return result;
}

pub const PointSegmentDistance = struct {
    dist_sqr: f32,
    /// Where the closest point lies along the segment, 0 at `p`, 1 at `q`.
    t: f32,
};

/// The squared xz distance from `pt` to the segment `p`..`q`.
pub fn distancePointSegmentSqr2d(pt: [3]f32, p: [3]f32, q: [3]f32) PointSegmentDistance {
    const pqx = q[0] - p[0];
    const pqz = q[2] - p[2];
    var dx = pt[0] - p[0];
    var dz = pt[2] - p[2];
    const d = pqx * pqx + pqz * pqz;
    var t = pqx * dx + pqz * dz;
    if (d > 0) t /= d;
    if (t < 0) {
        t = 0;
    } else if (t > 1) {
        t = 1;
    }
    dx = p[0] + t * pqx - pt[0];
    dz = p[2] + t * pqz - pt[2];
    return .{ .dist_sqr = dx * dx + dz * dz, .t = t };
}

/// The mean of the vertices `indices` name.
pub fn calcPolyCenter(indices: []const u16, verts: []const [3]f32) [3]f32 {
    var tc: [3]f32 = .{ 0.0, 0.0, 0.0 };
    for (indices) |index| {
        const v = verts[index];
        tc[0] += v[0];
        tc[1] += v[1];
        tc[2] += v[2];
    }
    const count: i32 = @intCast(indices.len);
    const s = 1.0 / @as(f32, @floatFromInt(count));
    tc[0] *= s;
    tc[1] *= s;
    tc[2] *= s;
    return tc;
}

/// The height of triangle `a`, `b`, `c` at `p`'s xz position, or null when
/// `p` lies outside the triangle or the triangle is degenerate in xz.
pub fn closestHeightOnTriangle(p: [3]f32, a: [3]f32, b: [3]f32, c: [3]f32) ?f32 {
    const eps: f32 = 1e-6;

    const v0 = vsub(c, a);
    const v1 = vsub(b, a);
    const v2 = vsub(p, a);

    // Compute scaled barycentric coordinates.
    var denom = v0[0] * v1[2] - v0[2] * v1[0];
    if (@abs(denom) < eps) return null;

    var u = v1[2] * v2[0] - v1[0] * v2[2];
    var v = v0[0] * v2[2] - v0[2] * v2[0];

    if (denom < 0) {
        denom = -denom;
        u = -u;
        v = -v;
    }

    // If the point lies inside the triangle, return the interpolated height.
    if (u >= 0.0 and v >= 0.0 and (u + v) <= denom) {
        return a[1] + (v0[1] * u + v1[1] * v) / denom;
    }
    return null;
}

/// Whether `pt` lies inside the polygon `verts` in the xz plane. All points
/// are projected onto the xz plane, so y is ignored.
pub fn pointInPolygon(pt: [3]f32, verts: []const [3]f32) bool {
    var c = false;
    var j = verts.len -% 1;
    for (0..verts.len) |i| {
        defer j = i;
        const vi = verts[i];
        const vj = verts[j];
        if (((vi[2] > pt[2]) != (vj[2] > pt[2])) and
            (pt[0] < (vj[0] - vi[0]) * (pt[2] - vi[2]) / (vj[2] - vi[2]) + vi[0]))
        {
            c = !c;
        }
    }
    return c;
}

/// Whether `pt` lies inside the polygon `verts` in the xz plane, also
/// writing for each edge (from vertex `j` to `j + 1`) the squared distance
/// to it into `edge_dist_sqr[j]` and the closest point's parameter into
/// `edge_t[j]`. Both slices hold at least `verts.len` entries.
pub fn distancePointPolyEdgesSqr(
    pt: [3]f32,
    verts: []const [3]f32,
    edge_dist_sqr: []f32,
    edge_t: []f32,
) bool {
    var c = false;
    var j = verts.len -% 1;
    for (0..verts.len) |i| {
        defer j = i;
        const vi = verts[i];
        const vj = verts[j];
        if (((vi[2] > pt[2]) != (vj[2] > pt[2])) and
            (pt[0] < (vj[0] - vi[0]) * (pt[2] - vi[2]) / (vj[2] - vi[2]) + vi[0]))
        {
            c = !c;
        }
        const distance = distancePointSegmentSqr2d(pt, vj, vi);
        edge_dist_sqr[j] = distance.dist_sqr;
        edge_t[j] = distance.t;
    }
    return c;
}

const Range = struct { min: f32, max: f32 };

fn projectPoly(axis: [3]f32, poly: []const [3]f32) Range {
    var range: Range = undefined;
    range.min = vdot2d(axis, poly[0]);
    range.max = range.min;
    for (poly[1..]) |vert| {
        const d = vdot2d(axis, vert);
        range.min = min(f32, range.min, d);
        range.max = max(f32, range.max, d);
    }
    return range;
}

inline fn overlapRange(a: Range, b: Range, eps: f32) bool {
    return if ((a.min + eps) > b.max or (a.max - eps) < b.min) false else true;
}

/// Whether two convex polygons overlap in the xz plane, by the separating
/// axis test over both polygons' edge normals.
pub fn overlapPolyPoly2d(polya: []const [3]f32, polyb: []const [3]f32) bool {
    const eps: f32 = 1e-4;

    inline for (.{ polya, polyb }) |poly| {
        var j = poly.len -% 1;
        for (0..poly.len) |i| {
            defer j = i;
            const va = poly[j];
            const vb = poly[i];
            const n: [3]f32 = .{ vb[2] - va[2], 0, -(vb[0] - va[0]) };
            const a_range = projectPoly(n, polya);
            const b_range = projectPoly(n, polyb);
            // A separating axis was found.
            if (!overlapRange(a_range, b_range, eps)) return false;
        }
    }
    return true;
}

/// A point of the convex polygon `pts` picked by `s` and `t`, both in
/// 0..1: uniformly distributed when they are. `pts` holds at least two
/// points, and `areas` is scratch space of at least `pts.len` entries.
pub fn randomPointInConvexPoly(pts: []const [3]f32, areas: []f32, s: f32, t: f32) [3]f32 {
    // Calculate the triangle fan's areas.
    var area_sum: f32 = 0.0;
    var i: usize = 2;
    while (i < pts.len) : (i += 1) {
        areas[i] = triArea2d(pts[0], pts[i - 1], pts[i]);
        area_sum += max(f32, 0.001, areas[i]);
    }
    // Find the sub triangle weighted by area.
    const threshold = s * area_sum;
    var acc: f32 = 0.0;
    var u: f32 = 1.0;
    var tri = pts.len - 1;
    i = 2;
    while (i < pts.len) : (i += 1) {
        const dacc = areas[i];
        if (threshold >= acc and threshold < (acc + dacc)) {
            u = (threshold - acc) / dacc;
            tri = i;
            break;
        }
        acc += dacc;
    }

    const v = @sqrt(t);

    const a = 1 - v;
    const b = (1 - u) * v;
    const c = u * v;
    const pa = pts[0];
    const pb = pts[tri - 1];
    const pc = pts[tri];

    return .{
        a * pa[0] + b * pb[0] + c * pc[0],
        a * pa[1] + b * pb[1] + c * pc[1],
        a * pa[2] + b * pb[2] + c * pc[2],
    };
}

inline fn vperpXz(a: [3]f32, b: [3]f32) f32 {
    return a[0] * b[2] - a[2] * b[0];
}

pub const SegmentSegmentHit = struct {
    /// The parameter of the crossing along the first segment.
    s: f32,
    /// The parameter of the crossing along the second segment.
    t: f32,
};

/// Where the lines through `ap`..`aq` and `bp`..`bq` cross in the xz plane,
/// or null when they are parallel. The parameters may lie outside 0..1.
pub fn intersectSegmentSegment2d(
    ap: [3]f32,
    aq: [3]f32,
    bp: [3]f32,
    bq: [3]f32,
) ?SegmentSegmentHit {
    const u = vsub(aq, ap);
    const v = vsub(bq, bp);
    const w = vsub(ap, bp);
    const d = vperpXz(u, v);
    if (@abs(d) < 1e-6) return null;
    return .{ .s = vperpXz(v, w) / d, .t = vperpXz(u, w) / d };
}

/// The next power of two at or above `v`; 0 for 0 and for values above
/// 2^31.
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
    var v = value;
    var r: u32 = @as(u32, @intFromBool(v > 0xffff)) << 4;
    v >>= @intCast(r);
    var shift: u32 = @as(u32, @intFromBool(v > 0xff)) << 3;
    v >>= @intCast(shift);
    r |= shift;
    shift = @as(u32, @intFromBool(v > 0xf)) << 2;
    v >>= @intCast(shift);
    r |= shift;
    shift = @as(u32, @intFromBool(v > 0x3)) << 1;
    v >>= @intCast(shift);
    r |= shift;
    r |= (v >> 1);
    return r;
}

/// `x` rounded up to a multiple of 4.
pub inline fn align4(x: usize) usize {
    return (x + 3) & ~@as(usize, 3);
}

/// The tile side facing `side`, sides numbered 0..7 counter-clockwise.
pub inline fn oppositeTile(side: i32) i32 {
    return (side + 4) & 0x7;
}

/// A float with its bytes reversed, for tile data of the other endianness.
pub inline fn swapEndianFloat(v: f32) f32 {
    return @bitCast(@byteSwap(@as(u32, @bitCast(v))));
}
