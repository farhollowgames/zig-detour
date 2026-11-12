# The integer variant

`detour.integer` is the query layer of this port on whole numbers: the
navigation mesh, its tiles and references, the node pool and queue, and every
query of `NavMeshQuery`. It has the same structure and the same names as the
float port, and no floating-point operation anywhere, so the same call gives
the same bytes on every machine, compiler and optimisation level.

The float port stays the exact translation of the original, checked against it
byte for byte. The integer variant is checked against the float port: same
polygons, same statuses, positions within the tolerances below, over the demo
meshes and seeded random meshes. This file is the design those checks hold the
code to.

## What is in it

| part | integer variant |
| --- | --- |
| `NavMesh`, tiles, references, tile state | yes, `src/integer/nav_mesh.zig` |
| `NodePool`, `NodeQueue` | yes, `src/integer/node.zig` |
| `QueryFilter`, `NavMeshQuery`, every query, sliced searches | yes, `src/integer/nav_mesh_query.zig` |
| geometry helpers | yes, `src/integer/common.zig` |
| tiles from float tiles | yes, `src/integer/convert.zig`, the only file that reads floats |
| `createNavMeshData` on integer input | no: a tile is built by the float builder and converted |
| crowd, local boundary, obstacle avoidance, path corridor, path queue, proximity grid | no, float only |
| tile cache and its builder | no, float only |

The crowd and the tile cache steer by velocities, sample angles and blend
weights; an integer version of them would be a new design, not a variant of
this one, so they are left out here.

## Use

Its names and calls are the float port's, with `[3]i32` positions in whole
millimetres; costs are whole numbers in 1/256 mm, and a ray's hit is an exact
fraction. A tile is baked as usual and converted:

```zig
// Metres to millimetres: a scale of 1000.
const data = try detour.integer.tileFromFloat(gpa, float_tile, 1000);
var nav: detour.integer.NavMesh = .empty;
if (nav.initSingleTile(gpa, data, .{ .free_data = true }).failed) {
    gpa.free(data);
    return error.BadTile;
}
defer nav.deinit();
```

Coordinates lie within 2^29 mm (about 536 km) of the origin.

## Units

| quantity | type | unit |
| --- | --- | --- |
| position, extent, vertex | `[3]i32` | millimetre |
| distance, radius, height | `i32` or `u32` | millimetre |
| squared distance | `u64` | square millimetre |
| path cost, heuristic, node total | `u64` | 1/256 millimetre at area cost 1 |
| area cost of a filter | `u32` | Q16.16: `1 << 16` is cost 1 |
| direction, wall normal | `[3]i32` | Q16.16: `1 << 16` is length 1 |
| place along a segment or ray | `Fraction` | `num / den`, exact |
| place of a link along an edge | `u8` | 1/255 of the edge, as in the original |
| random draw | `u32` | `r / 2^32`, in 0..1 |

One float unit of the original is one metre in Recast's demo assets and in
Loomwork; the converter multiplies by 1000 by default and takes another scale
when a mesh is in other units. The variant's unit is the millimetre at every
scale: `scale` is the number of millimetres in one float unit, so a tile in
centimetres converts at 10 to the bytes the same tile in metres gives at
1000, and every fixed distance below (the geometry tests' thresholds, the
bound, the checks' tolerances) is in millimetres whatever the scale. A scale
that makes another unit, such as 1 for a tile in metres, is a caller's
mistake the converter cannot see: every threshold is then read in that
unit.

## Bounds

Every coordinate handed to the variant, and every vertex of a tile, lies in
`-2^29 ..= 2^29` millimetres (about 536 km either side of the origin).
`NavMesh.addTile` refuses a tile with a vertex outside it, and every query
refuses a position or extent outside it with `invalid_param`, the way the
float port refuses a NaN.

From that bound:

| value | bound | held in |
| --- | --- | --- |
| difference of two coordinates | `2^30` | `i32` |
| product of two differences | `2^60` | `i64` |
| 2D cross or dot product (two such products) | `2^61` | `i64` |
| 3D squared distance (three squares) | `3 * 2^60 < 2^62` | `u64` |
| product of a cross product and a difference | `2^91` | `i128` |
| product of two cross products | `2^122` | `i128` |
| square of a cross product | `2^122` | `i128` |

No step multiplies three values of cross-product size, so `i128` is the
widest type needed. The places that would (comparing two fractions whose
parts are both products) are listed with the query they belong to, and each
reduces one side first.

### Where 64 bits are enough

A 64-bit multiply or divide is one instruction; one of 128 bits is several,
or a library call. Each place below uses 64 bits where the bound allows it,
and 128 bits otherwise, so its answers do not change:

- `divRound` divides in 64 bits when the numerator and the denominator both
  fit `i64`, checked at run time.
- Two fractions compare by cross multiplying. Each product is first taken
  in 64 bits with an overflow check, and both are taken again in 128 bits
  when either overflows.
- The height on a detail triangle is `dy0 * u + dy1 * v` over the
  barycentric denominator, with `|dy| <= 2^30` and `0 <= u, v <= 2^61`: up to
  `2^92`. It is worked out in 64 bits with overflow checks, and again in 128
  when a check fires.
- The squared distance from a point to a segment's line is `across^2 /
  len_sqr`. With `|across| < 2^31` the square is below `2^62` and divides in
  64 bits.
- Whether that distance is within a limit is `across^2 < (limit + 1) *
  len_sqr`: `across^2 <= 2^122` and `(limit + 1) * len_sqr <= 2^64 * 2^61`,
  both within `u128`. It takes two widening multiplies where the distance
  takes a division, and gives the same answer as rounding the distance down
  and comparing.
- A ray crossing a link over part of an edge compares `((start - left) *
  den + dir * num) * 255` with `span * bounds * den`. Each product is of two
  values within 64 bits (`span * bounds < 2^38`, `den <= 2^61`), so each is a
  single widening multiply; the sides stay within `2^100`.
- The root in `isqrt` takes one division. It starts from the tangent to the
  root at the middle of the number's top byte, from a 192-entry table, raised
  by three: the root is concave, so the tangent lies above it, by at most
  `2^14` over a byte's numbers. One of Newton's steps from above then leaves
  less than `1/8` above the root, and one decrement makes it exact. A unit
  test checks the ends of every top byte at every even shift.

Path costs add up with saturation at the largest `u64`, and a step's cost
(its length times the area cost) is worked out in `u128` and clamped to it. A
total can reach the limit only with an area cost above 512 on a path of tens
of thousands of polygons that each span the whole coordinate range; the
search then still ends, and totals at the limit compare as equal.

## Arithmetic

All of it is in `src/integer/common.zig`, written for this package; nothing
is shared with the float files.

**Division.** Zig's `@divTrunc`, `@divFloor` and `@rem` are used by name at
every site, never `/`. `divRound(n, d)` rounds to the nearest whole number,
halves away from zero, for `d > 0`. It is the rounding of every result that
is a position.

**Square root.** `isqrt(x: u64) u32` is the largest `r` with `r * r <= x`,
exact for every input, by Newton's iteration on integers from a starting
guess above the root. `isqrtWide(x: u128) u64` is the same for 128 bits.

**Length.** `length(d2: u64) u64` gives a distance in 1/256 millimetre from
its square in square millimetres: `isqrt(d2 << 16)` when `d2 < 2^48`, else
`isqrt(d2) << 8`. Both branches round down, and the result never decreases as
`d2` grows. Above `2^48` (16.7 km) the result is whole millimetres, which is
still finer than `f32` at that distance.

**Fractions.** A place along a segment is a `Fraction { num: i64, den: i64 }`
with `den >= 0`, the value `num / den`. `den == 0` with `num > 0` stands for
"never": a ray that hits nothing. Fractions are never reduced and never
converted to a fixed-point number on the way; two are compared by
cross-multiplying in `i128` (`a.num * b.den < b.num * a.den`), which is exact.

**Points along a segment.** `lerp(a, b, t)` is, per axis,
`a + divRound((b - a) * t.num, t.den)` in `i128`: the exact point rounded to
the nearest millimetre. A midpoint is `lerp` at `1/2`. Since each axis is
rounded on its own, a point on a slanted edge can lie up to 0.71 mm off the
edge in the plane, on either side; see "Points just outside" below.

**Normals.** A normalised 2D direction `(dx, dz)` is
`divRound(dx << 32, len16)` per axis, where
`len16 = isqrtWide((dx^2 + dz^2) << 32)` is the vector's length in 1/65536
millimetre; taking the root at that scale keeps a short vector's direction
good to the last bit. The zero vector gives the zero normal, where the float
port gives NaN.

**Area costs.** A step's cost is `(length * area_cost) >> 16` in `u128`,
clamped to `u64`. The heuristic is `length * 999 / 1000`, rounded down: the
original's `H_SCALE` of 0.999.

## Tiles

An integer tile has the float tile's layout with every `f32` replaced by an
`i32` of the same size and place, so the header, the polygons, the links, the
detail meshes and the tree keep their offsets, and a tile is still one block
the mesh points into. Its magic number differs (`'D' 'N' 'A' 'I'`), so neither
side loads the other's tiles by mistake; the version is the original's.

| float field | integer field |
| --- | --- |
| `walkable_height`, `walkable_radius`, `walkable_climb` | millimetres, rounded to nearest |
| `bounds_min`, `bounds_max` | millimetres; minimum rounded down, maximum rounded up |
| `bv_quant_factor` (cells per unit) | `bv_cell_size`: millimetres per tree cell, `max(1, round(scale / factor))` |
| `verts`, `detail_verts` | millimetres, rounded to nearest |
| off-mesh connection `pos`, `rad` | millimetres, rounded to nearest |
| polygons, links, detail meshes, detail triangles | unchanged |
| tree nodes | recomputed, see below |

`convert.tileFromFloat(gpa, float_tile, scale)` builds an integer tile from a
float one, and is the only function of the variant that reads a float. Each
value is `round(f64(v) * scale)`, exact in `f64` for every `f32` inside the
coordinate bound, so the conversion gives the same integers on every machine.
It fails when a value is NaN, infinite or outside the bound. When the mesh was
baked on a grid of whole millimetres (Loomwork's cell sizes and origin are),
the polygon vertices convert without rounding error; the detail vertices are
sampled heights and round to the nearest millimetre.

Two float vertices that round to the same millimetre make a polygon edge of
length zero. The converter keeps the polygon as it is: every query treats a
zero-length edge the way the float port treats one (it has no inside and no
crossing), and the checks against the float port cover such meshes.

**The tree.** A leaf's box is the union of the float tile's box for its
polygon and the cells, relative to `bounds_min` in cells of `bv_cell_size`
and both rounded down, that hold the lowest and the highest of the polygon's
vertices and detail vertices, clamped to 0..65535. Inner nodes are the union
of their children. The node order and the escape indices are the float
tile's. The float box has to stay: the original builds it from the detail
heights of the polygon's corners, which a tile does not keep (it keeps the
coarser polygon heights), so a box made from the tile alone can come out a
cell lower or higher than the original's. The integer geometry has to be in
it too, so that a query finds a polygon wherever its surface reaches the
query box. A box is therefore never smaller than the original's, and larger
where the polygon's corners stand outside the detail heights.

**Query boxes.** A query's box is clamped to the tile's bounds, made relative
to `bounds_min` and divided by `bv_cell_size` with `@divFloor`; then the
minimum is rounded down to even and the maximum up to odd, as in the
original. A coordinate past cell 65535 is held at 65535, where the original
wraps.

**Tile location.** `calcTileLoc` is `@divFloor(pos - origin, tile_size)` per
axis, where the float port rounds a quotient down; they agree except within
rounding of a tile border. `NavMeshParams` holds the origin and the tile
width and height in whole millimetres. `init` refuses a size below 1;
`initSingleTile` takes the tile's own extent and uses 1 when that is zero.

**References and state.** Polygon references, salts, tile state and flags are
integers already and are the float port's code unchanged.

## Geometry tests

The original guards its float tests with small thresholds. With exact
arithmetic most of them become a test against zero; the ones that express a
real distance keep it, in millimetres.

| original | where | integer variant |
| --- | --- | --- |
| `fabs(d) < 1e-6`: segment parallel to an edge | `intersectSegmentPoly2d` | `d == 0` |
| `fabs(d) < 1e-6`: parallel segments | `intersectSegmentSegment2d` | `d == 0` |
| `fabs(denom) < 1e-6`: triangle with no area | `closestHeightOnTriangle` | `denom == 0` |
| points nearer than 1/16384: `dtVequal` | `findStraightPath` | the same point, exactly |
| `d^2 < sqr(0.001)`: apex on the portal | `findStraightPath` | `d^2 < 1` square millimetre |
| `dist / 2 + 0.001`: search radius | `moveAlongSurface` | `(isqrt(d^2) + 2) / 2 + 2` millimetres |
| `eps = 1e-4` on projected ranges | `overlapPolyPoly2d` | 100 square millimetres |
| `px = 0.01`: slab ends shrunk | `overlapSlabs` | 10 millimetres |
| `fabs(apos - bpos) > 0.01`: edges on one border line | `findConnectingPolys` | more than 10 millimetres |
| `max(0.001, area)`: least weight of a triangle | `randomPointInConvexPoly` | 1000 square millimetres (twice the area, as the original counts it) |
| `FLT_MAX`: no hit | `raycast`, nearest searches | the "never" fraction, the largest `u64` |

The comparisons the original makes between float values that are quotients
(`t = n / d`) are made on the fractions themselves, exactly:

- `intersectSegmentPoly2d` keeps `tmin` and `tmax` as fractions with a
  positive denominator and compares by cross-multiplying. Numerator and
  denominator are 2D cross products, at most `2^61`, so a product is at most
  `2^122`.
- `distancePointSegmentSqr2d` returns `t` as `dot / len2` clamped to 0..1 and
  the squared distance rounded down: the squared distance to an end when `t`
  was clamped, else `cross^2 / len2` with the square in `i128`. A segment of
  length zero gives `t = 0` and the distance to its point.
- `pointInPolygon` and `distancePointPolyEdgesSqr` do the crossing test by
  comparing two products instead of dividing, exact for every input.
- `closestHeightOnTriangle` tests `u >= 0`, `v >= 0`, `u + v <= denom`
  exactly and returns `a.y + divRound(v0.y * u + v1.y * v, denom)`, the sum in
  `i128`.
- `overlapSlabs` compares the heights of the two edges at the ends of their
  shared span as fractions over `adx * bdx`: the sign test is exact, and the
  climb test is `|num| <= 2 * |climb| * den` in `i128`.

Where the original picks the nearest of several candidates with `<`, the
variant compares the rounded-down squared distances with `<`, so on a tie the
first candidate stays, as in the original.

## Links

A link across a tile border keeps the original's `bounds_min` and
`bounds_max`: the part of the edge it covers in 255ths. `connectExtLinks`
computes them as `divRound(num * 255, den)` from the exact fraction, clamped
to 0..255. The float port computes `roundf(t * 255.0f)` on a rounded `t`; the
two can differ by one step when the exact value is a half (a link that ends
exactly in the middle of an edge two cells long, say). Then every point
derived from the link (portal ends, wall segments) differs between the two by
up to 1/255 of the edge's length. The checks against the float port find
such links by comparing the link tables first and compare those cases with
the wider tolerance.

Portal ends on a partly covered edge are `lerp(v0, v1, bounds / 255)`, rounded
to the nearest millimetre. `raycast` tests whether the ray leaves through a
link's part of the edge without rounding: with the crossing at `tn / td` and
the link's end at `b / 255`, it compares `(s * td + d * tn) * 255` with
`(l * 255 + e * b) * td` in `i128` (`s`, `d` the ray's start and extent on
the axis, `l`, `e` the edge's), both at most `2^100`.

Off-mesh connections are linked as in the original: the end point is snapped
to the nearest polygon within the connection's radius, by the same nearest
search as `findNearestPoly`, and the radius test is on exact squares.

### Link order

The original puts each new link at the front of its polygon's list, and a
search meets a polygon's neighbours in that order, so where two ways tie
the order picks the corridor. The order follows the history of the mesh: a
tile removed and added again leaves its neighbours' lists, and the links
their free lists hand out, otherwise than a mesh opened afresh from the same
tiles, and a server that restores its mesh from saved tiles could then find
other paths than the one that ran on.

`NavMeshParams.link_order` chooses:

| order | each polygon's links |
| --- | --- |
| `added` (the default) | as the original makes them, in the order tiles came and went |
| `key` | as the original makes them when every tile present is added in key order (grid y, then x, then layer), whatever order the tiles came and went in |

With `key`, a tile added after every tile within two cells of it in key
order links as with `added`, which is then already that order: opening a
mesh in key order costs nothing more, and gives the original's links byte
for byte. Any other add, and every removal, empties the links of the tiles
in its own cell and the eight around it and makes them again, each tile's
own links first (which off-mesh connections start anywhere, read by the
others), then each tile's links across its borders, replaying the order a
key-order opening would make them in: the earlier tiles as adding the tile
meets them, then the later tiles in key order. Each lookup bucket keeps its
tiles from the latest key to the earliest, as a key-order opening leaves it,
so a cell's layers are met in one order too. An off-mesh connection's
polygon takes the connection's own end points back before its tile links
again, as a tile fresh from the builder holds them, so an end snapped to a
tile since removed does not stay there.

What still follows the history is which tile entry a tile takes, and so the
references: `findRandomPoint`, which walks the tiles by entry, can draw
otherwise. Every other query names polygons only by reference equality.

The tests (`test/link_order_test.zig`) open each level in key order with
both orders and compare every tile blob byte for byte and a workload of
queries; open it in a shuffled order, remove and add tiles at random, and
remove each tile in turn, comparing after each step every polygon's list,
the free links and the vertices, and the workload's answers, with polygons
named by tile and index, against a mesh opened afresh in key order; and show
that with `added` a tile removed and added again links otherwise.

## Queries

Every query has the float port's steps in the float port's order; what is
listed is where a number is computed differently.

**Nearest polygon** (`findNearestPoly`, `findNearestPolyInTile`). The
distance to a polygon under or over the point is `|dy| - walkable_climb`,
squared when positive and zero otherwise; else the 3D squared distance. Both
exact in `u64`.

**Closest points** (`closestPointOnPoly`, `closestPointOnPolyBoundary`,
`getPolyHeight`). Inside tests are exact. A point on an edge is
`lerp(va, vb, t)`. A height inside a detail triangle is rounded to the
nearest millimetre. Along an off-mesh connection the height is
`v0.y + divRound((v1.y - v0.y) * t.num, t.den)`.

**Path search** (`findPath`, the sliced searches). A node holds its position
in millimetres and `cost` and `total` in 1/256 millimetre. A node's position
is the midpoint of the portal it was first reached through. A step costs
`length * area_cost >> 16`; the heuristic is `length * 999 / 1000`; totals
add with saturation. The open list orders by `total` with the original's
heap. The any-angle search takes its ray limit as
`(walkable_radius * 50)^2` with a saturating product, and a shortcut counts
when the ray's `t` is at least 1 or "never".

**Straight path** (`findStraightPath`). The funnel's side tests are the sign
of an exact 2D cross product. A portal crossing appended for the
`area_crossings` and `all_crossings` options is `lerp(left, right, t)` with
`t` clamped to 0..1, where the original leaves `t` as it comes; the clamp
keeps the point on the portal when the path runs nearly along it.

**Ray** (`raycast`, `raycastPath`). `RaycastHit.t` is a fraction, "never"
when the ray reached its end. `RaycastHit.point(start, end)` gives the hit
position, `lerp(start, end, t)`, and `end` when nothing was hit. The hit
normal is Q16.16. With the `use_costs` option the walked point on each
crossed edge is `lerp(start, end, t)` with its height taken from the edge:
`e1.y + divRound(edge.y * diff, edge_axis)` on the edge's longer axis, and
`e1.y` when the edge has no length.

**Walk** (`moveAlongSurface`). The search circle is centred on the rounded
midpoint of start and end with the radius of the table above, never smaller
than the float port's circle. The result on a wall is `lerp(vj, vi, t)`.

**Around a point or shape** (`findPolysAroundCircle`, `findPolysAroundShape`,
`findLocalNeighbourhood`, `findDistanceToWall`,
`findRandomPointAroundCircle`). A portal or wall is in reach when its
rounded-down squared distance is at most the squared radius. Result costs
are in 1/256 millimetre. `findPolysAroundShape` takes the shape's centre as
the average of its vertices, `divRound(sum, count)`.
`findDistanceToWall` returns `isqrt` of the squared distance, the wall point
as `lerp`, and the normal from the wall point to the centre in Q16.16, zero
when the centre is on the wall.

**Wall segments** (`getPolyWallSegments`). Segment ends on a tile border are
`lerp(vj, vi, t / 255)`.

**Random points** (`findRandomPoint`, `findRandomPointAroundCircle`). Draws
are `random.int(u32)`, read as `r / 2^32`. A polygon replaces the pick when
`r * area_sum <= area << 32` in `u128`, with areas exact. The point in the
polygon comes from two draws `s` and `t`: the triangle of the fan by `s` and
the exact areas, then weights `a = 2^32 - v`, `c = v * u`, `b = v - c` with
`v = isqrt(t << 32)` and `u` the exact fraction left of `s`; the point is
`divRound(a * pa + b * pb + c * pc, 2^32)` per axis. The weights sum to
`2^32`, so the point is within half a millimetre per axis of a point of the
triangle. The same generator and seed give other points than the float
port, which draws 24-bit floats; the distribution is the same.

## Points just outside

A point computed on an edge is rounded to whole millimetres per axis, so it
can lie up to 0.71 mm off the edge in the xz plane, on either side: a wall
point from `moveAlongSurface` or `raycast` can be that far outside its
polygon. The float port has the same property at the scale of `f32`'s
rounding. The variant does not correct it, because every query takes a
position together with a polygon reference and none requires the position to
be inside: `moveAlongSurface` and `raycast` start from the polygon given, and
a start a fraction of a millimetre outside behaves as a start on the edge.

Heights are rounded to the nearest millimetre and are not otherwise bounded:
a query in the xz plane gives the height its inputs give.

## What a caller sees differently

These are the choices of the variant that show in its results or its
interface, beyond the units:

1. **No crowd and no tile cache** on integers; they stay float only.
2. **Coordinates within 536 km of the origin.** Outside it a query fails with
   `invalid_param` and a tile is refused.
3. **`RaycastHit.t` is a fraction**, not a number in 0..1, with
   `RaycastHit.point` for the position. No precision is lost on a long ray.
4. **Costs are `u64` in 1/256 millimetre**, area costs Q16.16, normals
   Q16.16. A filter that replaces `getCost` returns `u64` in the same unit.
5. **Random points differ** from the float port's for the same seed.
6. **Equal-cost paths.** Where two corridors cost the same to within the
   float port's rounding, the variant may return the other one. Both are
   cheapest by the searches' own costs. Where two parents reach a polygon
   at nearly the same cost, rounding picks either, and a node keeps the
   place it was first reached from, so the corridors, summed at their
   portals' middles, can then differ by more than rounding.
7. **Collinear corners.** `findStraightPath` decides exactly whether three
   points lie on a line; the float port decides by rounded products. A
   corner on a straight stretch can appear in one result and not the other.
   The path walked is the same.
8. **Portal crossings are clamped** to their portal in `findStraightPath`
   with the crossing options.
9. **Link limits can differ by one step of 1/255** from the float port's on
   exact halves.
10. **Tiles are a separate format**, made from float tiles by the converter.
    The builder itself is float only, so baking uses the float builder.
11. **Degenerate input has a defined result** where the float port gives
    NaN: the zero normal for a zero vector, `t = 0` on a segment of no
    length, the edge's first height on an edge of no length.
12. **Tree boxes hold the tile's own geometry**, so `queryPolygons` and the
    searches built on it can find a polygon whose surface reaches the query
    box where the original's box, built from detail heights, did not.
13. **The shorter path where corridor vertices meet on one line.** Exact
    side tests take the straight way where the float funnel, misled by
    rounding, can turn at the far end of a portal.

## Checks against the float port

The tests run both on the same meshes: the float tile through the float
port, its conversion through the variant, with the same positions (whole
millimetres, handed to the float port as metres). Meshes are RecastDemo's
levels and seeded random meshes baked on a grid of whole millimetres, all
within 1 km of the origin, where `f32` still resolves a tenth of a
millimetre: `undulating.obj`, which lies about 5 km out, is moved 5 km
toward the origin first.

**Tolerance.** `T = 2 mm` per axis for a position: half a millimetre for the
converter's rounding, half for the variant's, and one for the float port's
own error at these sizes. Heights read from the detail surface get `2 T`,
since a slope turns a sideways difference into a height difference.

**Stable cases.** A discrete answer (which polygon, how many, which status)
can flip when a float lands on the other side of an edge. A case is *stable*
when the float port gives the same discrete answer for the input and for the
input moved by 1 mm each way along x and z. The tests compare the stable
cases and count the others; a test fails when fewer than 80% of its cases
are stable, so it cannot pass by skipping.

| query | what must hold on a stable case |
| --- | --- |
| converter | every vertex within 0.5 mm of the float one, an off-mesh connection's ends within `T` as the nearest points they are snapped to; links the same, limits within one step; every tree box holds the float box and the integer geometry |
| `findNearestPoly` | same polygon and over-flag, point within `T`; a different polygon only where the two trees' searches differ at the box's border, or where two polygons lie equally near within `T`; border cases at most 1/50 of the stable cases and ties at most 1/500 |
| `queryPolygons` | same set, but for polygons whose integer tree box is within two tree cells of the query box's border (the query box is widened by up to two cells on either side); such cases at most 1/20 of the cases, and such polygons at most 1/10 |
| `closestPointOnPoly`, `closestPointOnPolyBoundary`, `getPolyHeight` | same status and over-flag, point within `T`, height within `2 T` |
| `findPath`, sliced | same status; the same corridor, or one whose cost, summed by one evaluator on both, is within 0.1% plus 1 mm per polygon; at most 1/200 of the stable cases on another corridor. Where the search's choice between two parents of nearly the same cost flips (6, "Equal-cost paths"), the integer corridor may cost less beyond that, never more: at most 1/1000 of the stable cases (fuzz seed 13: 71.4 m against 73.4 m, the two parents 9 mm apart in the float search). A partial result (`partial_result`: the end is not reached) leads to the node nearest the end, and is compared so only where both sides stopped at the same polygon. Where they stopped at others, two nodes lie as near the end within rounding, as two corridors can cost the same (6, "Equal-cost paths", below): the float port must have reached both polygons, their nearest nodes' distances to the end must lie within `2 T` of each other, and the corridors and their costs are not compared |
| `findStraightPath` on one corridor | the two paths as lines lie within `T` of each other, plus 1/255 of the edge of each corridor link whose limits are a step apart; same flags and polygons at the corners both have, where a corner at a vertex several portals share may name either polygon that holds it; with the crossing options the corners of both, each side run again without the options, are held so, since the float port extends a crossing past its portal; then the crossings are compared in order: the same flags and polygon at each vertex both list, a vertex several portals share naming either polygon, and each integer crossing on its portal within `T` (plus the link step); a crossing only one path lists only where it and the vertex beside it are one point within `T`, at an end of its portal, on a portal the path starts or ends on, or where the leg runs along its portal (the float port takes lines within `1e-6` of parallel as parallel); where the float port's own corners change with the option, on a corridor that turns back along one line, the crossings are not compared; where vertices of the corridor meet on one line the float funnel can turn on the far side of a portal and take a longer way, and the integer path must then pass within `T` of every portal and be no longer (at most 1/80 of the stable cases); where the end is clamped to the last polygon's boundary and an edge and a vertex lie equally near the target, each side can pick either, and the two ends must then lie within `2 T` of each other on that polygon, the rest of the paths within `T`: at most 1/1000 of the stable cases (fuzz seed 3: 3.6 mm); corners at a vertex several portals share at most 1/45 of the stable cases, and of the cases with the crossing options, crossings on one path only at most 1/22, crossings at such a vertex at most 1/40, and float corners that change with the option at most 1/250 |
| `raycast` | same polygons crossed and same hit edge; hit point within `T` over the sine of the angle between ray and wall (a wall moved by `T` moves the hit along the ray by that much), and its height within that times the ray's rise over its run; normal within `2^-10`; cost within 0.1% plus `1 + 2 T` mm per polygon crossed plus `T`, or, where a ray near parallel to its wall moves the hit point along it, plus that move times the dearest area cost, since the last step's cost ends at the hit point: at most 1/500 of the stable cases (fuzz seeds 2, 3, 4, 12, 13, 14, 20, 23, 26, 29; sines 0.006 to 0.037, cost off by the point's move, up to 36 mm) |
| `moveAlongSurface` | same polygons visited, result within `T`; where the walk ends on a wall at a vertex several polygons share, which polygon's wall it ends on flips, and the list is the way the search first reached that polygon, so the lists may differ from the start on: the last polygon of each must hold the end within `T`, at most 1/1000 of the stable cases (fuzz seeds 2, 9, 10, 12, 15, 27, 29); a case is stable only if the float port's walk ends at the target, or on a wall, for every nudge, since the end keeps the target's height inside and takes the wall's on a wall |
| `findPolysAroundCircle`, `findPolysAroundShape`, `findLocalNeighbourhood` | same polygons and parents; costs within 0.1% plus, per polygon of depth, `1 + 2 T` mm times the dearest area cost, plus `T` |
| `getPolyWallSegments` | same count and references; ends within `T` plus 1/255 of the polygon's longest edge |
| `findDistanceToWall` | distance within `T`; wall point within `T` unless two walls are equally near; normal within `max(2^-8, 2 T / distance)` when the distance is above 100 mm; two walls equally near at most 1/300 of the stable cases; two walls stacked at one place in the xz plane, where the distance measured across ties and either wall's height will do, at most 1/1000, the normal then held to the direction from the variant's own wall point (fuzz seeds 4, 11: 200 mm apart in height) |
| random points | the point lies on the returned polygon within 1 mm, the polygon passes the filter, and over many draws polygons are picked in proportion to area |

Every row is a seeded test of at least a thousand cases that prints the
level and the case on failure (the seeds are constants of the tests), plus
hand-made cases on both sides: a mesh with no tile, one polygon with points
inside, exactly on an edge, at a vertex and outside, and two polygons joined
along an edge that the conversion makes a point.

Each rate above stands just above the worst measured over fuzz seeds 0 to 31,
the same in Debug and in ReleaseSafe; the tests write the measured worst
beside each limit.

**Overflow.** A test runs every query on a level of four tiles that spans
the whole coordinate range, its vertices within a millimetre of the bound,
from the corners of the bound and seeded points, with radii and boxes up to
the whole range, built with safety checks on, so an overflow anywhere would
trap. One millimetre past the bound is refused.

## Determinism

- No float: a test reads the variant's source files, `convert.zig` aside,
  and fails on any float type, float builtin or float literal.
- Run to run: a fixed workload of every query gives the same bytes twice in
  one process.
- Threads: the same workload on 1, 2 and N threads, each with its own
  `NavMeshQuery` over one shared mesh, gives the same bytes.
- Across builds and machines: the workload's output is hashed and compared
  with a value kept in the test, which must hold in every optimisation mode
  and on every target the tests run on. The workload's level is made by the
  float port's own builder, not by Recast, so the hash depends on no C or
  C++ compiler; it holds in Debug, ReleaseSafe and ReleaseFast.
