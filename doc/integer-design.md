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
code to. It was written before the code.

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
when a mesh is in other units.

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

**The tree.** A tree node's box is recomputed from the integer geometry: per
polygon the box of its vertices and of its detail vertices, relative to
`bounds_min`, in cells of `bv_cell_size`, the minimum rounded down and the
maximum rounded up, clamped to 0..65535. Inner nodes are the union of their
children. The node order and the escape indices are the float tile's. The
original truncates both ends and relies on the query widening its box by a
cell; the variant keeps that widening too, so its boxes are never smaller
than the original's except where a float product lands a hair on the other
side of a cell border.

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
   cheapest.
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

## Checks against the float port

The tests run both on the same meshes: the float tile through the float
port, its conversion through the variant, with the same positions (whole
millimetres, handed to the float port as metres). Meshes are RecastDemo's
levels and seeded random meshes baked on a grid of whole millimetres within
1 km of the origin, where `f32` still resolves a tenth of a millimetre.

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
| converter | every vertex within 0.5 mm of the float one; links the same, limits within one step |
| `findNearestPoly` | same polygon and over-flag, point within `T` |
| `queryPolygons` | same set, but for polygons whose box is within one tree cell of the query box's border |
| `closestPointOnPoly`, `closestPointOnPolyBoundary`, `getPolyHeight` | same status and over-flag, point within `T`, height within `2 T` |
| `findPath`, sliced | same status; the same corridor, or one whose cost, summed by one evaluator on both, is within 0.1% plus 1 mm per polygon; at least 90% the same corridor |
| `findStraightPath` on one corridor | the two paths as lines lie within `T` of each other; same flags and polygons at the corners both have |
| `raycast` | same polygons crossed and same hit edge; hit point within `T`; normal within `2^-10`; cost within 0.1% plus 1 mm per polygon |
| `moveAlongSurface` | same polygons visited, result within `T` |
| `findPolysAroundCircle`, `findPolysAroundShape`, `findLocalNeighbourhood` | same polygons and parents; costs within 0.1% plus 1 mm per polygon |
| `getPolyWallSegments` | same count and references; ends within `T`, or within `T` plus 1/255 of the edge at a link whose limits differ |
| `findDistanceToWall` | distance within `T`; wall point within `T` unless two walls are equally near; normal within `2^-8` when the distance is above 100 mm |
| random points | the point lies on the returned polygon within 1 mm, the polygon passes the filter, and over many draws polygons are picked in proportion to area |

Every row is a seeded test of at least a thousand cases that prints its seed
on failure, plus hand-made cases: the empty mesh, one polygon, zero-length
edges, a point exactly on an edge or vertex, and geometry at the coordinate
bound.

**Overflow.** A test runs every query on meshes with vertices at the corners
of the coordinate bound, with rays and radii across the whole range, built
with safety checks on, so an overflow anywhere would trap.

## Determinism

- No float: a test reads the variant's source files, `convert.zig` aside,
  and fails on any float type, float builtin or float literal.
- Run to run: a fixed workload of every query gives the same bytes twice in
  one process.
- Threads: the same workload on 1, 2 and N threads, each with its own
  `NavMeshQuery` over one shared mesh, gives the same bytes.
- Across builds and machines: the workload's output is hashed and compared
  with a value kept in the test, which must hold in every optimisation mode
  and on every target the tests run on.
