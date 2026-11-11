# zig-detour

Detour, DetourCrowd and DetourTileCache, the navigation mesh runtime, in pure
Zig.

zig-detour holds navigation meshes in tiles and answers questions about them:
the nearest polygon to a point, the cheapest corridor of polygons between two
points and the corners of the straight path through it, how far a walk or a
ray gets, what lies around a point. Its crowd moves agents along such paths
and steers them around each other, and its tile cache rebuilds tiles around
obstacles at run time. It is a translation of Detour, DetourCrowd and
DetourTileCache from
[recastnavigation](https://github.com/recastnavigation/recastnavigation)
1.6.0. Tests run the original and the port on the same inputs and compare
every returned value and every byte written. It has no dependencies beyond
Zig's standard library, and every function that allocates takes the allocator
it uses.

**The port is in progress.** Every part is translated, and the tests compare
all of it with the original: the navigation mesh, the tile builder, the
queries and the node pool, the crowd and its parts, the tile cache and its
builder. An integer version of the navigation mesh and its queries is
written and checked against the float port. Benchmarks against the original
are [below](#benchmarks); a performance pass and an outside review come next.
[`STATUS.md`](STATUS.md) lists
every function, option and mode of the original with its state, and what
comes next.
[`doc/parity.md`](doc/parity.md) lists every place where the port differs
from the original on purpose.

This port was written by Claude (Anthropic's AI model) for Farhollow Games.
How the code is verified can be found
[below](#how-the-tests-prove-equivalence).

Requires Zig 0.17.0.

## Use

Add the package:

```sh
zig fetch --save git+https://github.com/farhollowgames/zig-detour
```

and import its module in `build.zig`:

```zig
const detour = b.dependency("detour", .{ .target = target, .optimize = optimize });
exe.root_module.addImport("detour", detour.module("detour"));
```

Find the corners of the path between two points on a mesh of one tile:

```zig
const std = @import("std");
const detour = @import("detour");

/// Writes the corners of the path from `start` to `end` into `corners` and
/// returns how many there are. `tile` is a tile as `detour.createNavMeshData`
/// builds it, and must outlive the call.
pub fn pathCorners(
    gpa: std.mem.Allocator,
    tile: []align(4) u8,
    start: [3]f32,
    end: [3]f32,
    corners: [][3]f32,
) !usize {
    var nav: detour.NavMesh = .empty;
    if (nav.initSingleTile(gpa, tile, .{}).failed) return error.BadTile;
    defer nav.deinit();

    var query: detour.NavMeshQuery = .empty;
    if (query.init(gpa, &nav, 2048).failed) return error.OutOfMemory;
    defer query.deinit(gpa);

    const filter: detour.QueryFilter = .{};
    const reach: [3]f32 = .{ 2, 4, 2 };
    var start_ref: detour.PolyRef = 0;
    var end_ref: detour.PolyRef = 0;
    var from = start;
    var to = end;
    _ = query.findNearestPoly(start, reach, &filter, &start_ref, &from, null);
    _ = query.findNearestPoly(end, reach, &filter, &end_ref, &to, null);
    if (start_ref == 0 or end_ref == 0) return error.OffTheMesh;

    var polys: [256]detour.PolyRef = undefined;
    var poly_count: usize = 0;
    const found = query.findPath(start_ref, end_ref, from, to, &filter, &polys, &poly_count);
    if (found.failed) return error.NoPath;

    var corner_count: usize = 0;
    _ = query.findStraightPath(
        from,
        to,
        polys[0..poly_count],
        corners,
        null,
        null,
        &corner_count,
        .{},
    );
    return corner_count;
}
```

The API follows the original's, function for function (`dtNavMeshQuery::findPath`
is `NavMeshQuery.findPath`), with Zig's conventions:

- a value is declared where the caller wants it (`var nav: NavMesh = .empty;`),
  readied with `init` and freed with `deinit`;
- buffers are slices, and counts come back as `usize`;
- calls return the original's status, as `detour.Status`: a packed struct of
  the same bits, with `succeeded`, `failed`, `in_progress` and the detail
  flags as fields;
- flags and options are packed structs of the original's bits;
- structures keep the original's fields in snake_case, and tiles keep the
  original's byte layout, so a tile built by either works in both.

[`STATUS.md`](STATUS.md) maps every function and type of the original to its
counterpart.

### Integer queries

`detour.integer` is the navigation mesh and every query of `NavMeshQuery` on
whole millimetres, with no floating-point operation, so a call gives the same
bytes on every machine, compiler and optimisation mode: for a server whose
answers clients must be able to repeat. Its names and calls are the float
port's, with `[3]i32` positions; costs are whole numbers in 1/256 mm, and a
ray's hit is an exact fraction. A tile is baked as usual and converted:

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

Coordinates lie within 2^29 mm (about 536 km) of the origin. The crowd and
the tile cache stay float only. [`doc/integer-design.md`](doc/integer-design.md)
gives the units, the overflow bounds, what a caller sees differently, and the
tolerances the tests hold it to.

### Build switches

The original's three compile-time switches are build options:

| option | original | effect |
| --- | --- | --- |
| `-Dpolyref64` | `DT_POLYREF64` | polygon and tile references of 64 bits instead of 32 |
| `-Dvirtual-query-filter` | `DT_VIRTUAL_QUERYFILTER` | a query filter can replace its pass and cost functions |
| `-Dfast-math` | `RC_FAST_MATH` | no checks for infinite and NaN inputs |

Pass them where the dependency is declared:
`b.dependency("detour", .{ .target = target, .optimize = optimize, .polyref64 = true })`.

## How the tests prove equivalence

`zig build test` fetches the original at the pinned tag, builds it from source
(test-only code, never part of the package) and runs it and the port on the
same inputs: RecastDemo's three level meshes built into single tiles and into
tile grids, hand-made meshes for the edge cases (empty, one polygon, the
largest sizes, degenerate geometry), and seeded random inputs, at least a
thousand cases per test, with the seed printed when one fails. After every
call the two must agree on every returned value and every byte written:
tiles, links, saved tile state, references, points, every field of every
crowd agent, the calls a query makes to a filter's own functions. Tiles
damaged at random are refused by the port where the original would read or
write outside them, and taken as the original takes them otherwise.

The tests run twice: with every build switch off, and with every one on.

The integer queries cannot match the original byte for byte, so the tests
hold them to the float port within the tolerances written in
[`doc/integer-design.md`](doc/integer-design.md) (2 mm for a position), on
four levels and about 1,200 seeded cases per query, comparing only the cases
where the float port's answer does not change when the input moves by a
millimetre. Hand-made meshes cover the empty mesh, a single polygon, points
exactly on an edge or a vertex, and an edge that rounds to nothing. A level
spanning the whole coordinate range runs every query with overflow checks
on. A fixed workload gives the same bytes twice, on 1, 2 and many threads,
and a hash kept in the test, the same in Debug, ReleaseSafe and ReleaseFast.
A test reads the integer source and finds no float type, float builtin or
float literal in it.

Both sides are built with fused multiply-add off, so float steps round as
written. Where the original's behaviour is undefined (out-of-range float
conversions, reads past a buffer), the port does what the original does on
x86-64 or refuses the input, the same on every machine.

## Benchmarks

`zig build bench` times the port against the original on the same inputs, one
feature per row. The port is built in ReleaseFast; the original at `-O3` by
the Clang that Zig 0.17.0 carries (22.1.8), for the same CPU, with every build
switch off on both sides and fused multiply-add off on both, as in the tests.
`zig build bench -- <text>` runs only the rows whose name holds `<text>`, and
`--rounds=<n>` sets how many rounds each side is timed.

Each row warms both sides up for three rounds, then times 21 rounds of each,
taking turns, every round at least 20 ms of calls. The tables show the median
round, per item, and the fastest to the slowest round. Each side sums what it
returns into a checksum that is kept, so the compiler cannot drop the work,
and the two checksums must be equal (the integer table excepted). A ratio
under 1 means the port takes less time; the row in MB/s gives the ratio of
times too. The baseline is the original, and in the last table the float
port.

The inputs: RecastDemo's `nav_test.obj` built by Recast into tiles of 32 cells
of 0.3 m, with 32 seeded off-mesh connections for the navigation mesh rows;
1,000 seeded ground points, and pairs of them, for the queries; crowds of 16
and 128 agents walking between those points; the tile cache over the same
level in tiles of 48 cells, its layers stored as they are by one pass-through
compressor on both sides.

Measured on an AMD Ryzen 9 9950X3D under Linux 7.3, pinned to one core
(`taskset -c 6 zig build bench`), with Zig 0.17.0.

Where the port is slower:
- `navMeshDataSwapEndian`: the original's compiler swaps the vertex floats
  sixteen at a time with AVX-512; the port's loops over unaligned floats swap
  one at a time.
- `closestHeightOnTriangle`, `overlapPolyPoly2d`, `NavMesh.addTile +
  removeTile` and the tile cache rows: each takes the same steps as the
  original, and the cause is not found yet. They are the first targets of the
  performance pass.
- The integer queries take 1.3 to 2.2 times as long as the float port's: their
  arithmetic is exact, in 64-bit and, for products of products, 128-bit
  integers, and a hit along a ray or an edge is kept as an exact fraction
  rather than rounded.

#### Navigation mesh and tile builder

| feature | per | baseline | port | port / baseline | baseline range | port range |
| --- | --- | ---: | ---: | ---: | --- | --- |
| createNavMeshData | tile | 352 ns | 346 ns | 0.98 | 349 ns to 356 ns | 342 ns to 349 ns |
| NavMesh.addTile + removeTile | tile | 303 ns | 326 ns | 1.07 | 301 ns to 307 ns | 325 ns to 329 ns |
| NavMesh.storeTileState + restoreTileState | tile | 9.0 ns | 7.8 ns | 0.87 | 8.9 ns to 9.0 ns | 7.7 ns to 8.0 ns |
| NavMesh.getTileAndPolyByRef | call | 1.5 ns | 1.2 ns | 0.77 | 1.5 ns to 1.6 ns | 1.2 ns to 1.2 ns |
| NavMesh.isValidPolyRef | call | 1.2 ns | 0.7 ns | 0.61 | 1.2 ns to 1.2 ns | 0.7 ns to 0.7 ns |
| NavMesh.setPolyFlags + getPolyFlags | call | 1.3 ns | 0.9 ns | 0.71 | 1.3 ns to 1.3 ns | 0.9 ns to 0.9 ns |
| NavMesh.setPolyArea + getPolyArea | call | 1.4 ns | 1.0 ns | 0.74 | 1.4 ns to 1.4 ns | 1.0 ns to 1.1 ns |
| NavMesh.calcTileLoc | call | 1.2 ns | 0.9 ns | 0.77 | 1.2 ns to 1.2 ns | 0.9 ns to 0.9 ns |
| NavMesh.getTilesAt | call | 2.1 ns | 1.8 ns | 0.84 | 2.1 ns to 2.2 ns | 1.8 ns to 2.0 ns |
| NavMesh.getTileAt | call | 1.7 ns | 1.0 ns | 0.60 | 1.7 ns to 1.7 ns | 1.0 ns to 1.0 ns |
| NavMesh.getTileRefAt | call | 1.5 ns | 1.3 ns | 0.85 | 1.5 ns to 1.6 ns | 1.3 ns to 1.3 ns |
| NavMesh.getTileByRef | call | 1.2 ns | 0.6 ns | 0.48 | 1.2 ns to 1.2 ns | 0.6 ns to 0.6 ns |
| NavMesh.getOffMeshConnectionPolyEndPoints | call | 3.9 ns | 2.8 ns | 0.73 | 3.8 ns to 4.6 ns | 2.8 ns to 3.6 ns |
| NavMesh.getOffMeshConnectionByRef | call | 1.5 ns | 1.1 ns | 0.76 | 1.5 ns to 1.5 ns | 1.1 ns to 1.2 ns |
| nav_mesh_builder.navMeshDataSwapEndian | MB/s | 73195 | 45167 | 1.62 | 72968 to 73251 | 45019 to 45233 |
| nav_mesh_builder.navMeshHeaderSwapEndian | call | 2.9 ns | 2.9 ns | 1.03 | 2.9 ns to 2.9 ns | 2.9 ns to 2.9 ns |
| common.intersectSegmentPoly2d | call | 4.6 ns | 3.7 ns | 0.79 | 4.6 ns to 4.7 ns | 3.6 ns to 3.7 ns |
| common.distancePointPolyEdgesSqr | call | 7.1 ns | 6.7 ns | 0.95 | 7.1 ns to 7.1 ns | 6.7 ns to 6.8 ns |
| common.pointInPolygon | call | 2.6 ns | 2.2 ns | 0.82 | 2.5 ns to 2.7 ns | 2.1 ns to 2.2 ns |
| common.closestHeightOnTriangle | call | 2.7 ns | 4.9 ns | 1.82 | 2.7 ns to 2.7 ns | 4.9 ns to 4.9 ns |
| common.closestPointOnTriangle | call | 3.5 ns | 3.5 ns | 1.00 | 3.5 ns to 3.5 ns | 3.5 ns to 3.5 ns |
| common.overlapPolyPoly2d | call | 18.7 ns | 20.7 ns | 1.11 | 18.6 ns to 18.7 ns | 20.4 ns to 21.1 ns |
| common.randomPointInConvexPoly | call | 4.5 ns | 3.8 ns | 0.85 | 4.5 ns to 4.6 ns | 3.8 ns to 3.8 ns |
| common.distancePointSegmentSqr2d | call | 1.8 ns | 1.5 ns | 0.85 | 1.8 ns to 1.9 ns | 1.5 ns to 1.5 ns |
| common.intersectSegmentSegment2d | call | 1.8 ns | 1.3 ns | 0.71 | 1.8 ns to 1.8 ns | 1.3 ns to 1.3 ns |
| common.calcPolyCenter | call | 2.7 ns | 1.8 ns | 0.68 | 2.7 ns to 2.7 ns | 1.8 ns to 1.8 ns |

#### Queries

| feature | per | baseline | port | port / baseline | baseline range | port range |
| --- | --- | ---: | ---: | ---: | --- | --- |
| NavMeshQuery.findNearestPoly | query | 177 ns | 144 ns | 0.81 | 176 ns to 179 ns | 143 ns to 146 ns |
| NavMeshQuery.queryPolygons | query | 50.7 ns | 41.0 ns | 0.81 | 50.5 ns to 51.2 ns | 40.9 ns to 41.3 ns |
| NavMeshQuery.queryPolygonsWith | query | 45.2 ns | 38.2 ns | 0.85 | 45.0 ns to 45.6 ns | 38.1 ns to 38.4 ns |
| NavMeshQuery.closestPointOnPoly | query | 23.1 ns | 17.9 ns | 0.77 | 23.1 ns to 23.2 ns | 17.8 ns to 17.9 ns |
| NavMeshQuery.closestPointOnPolyBoundary | query | 13.5 ns | 11.1 ns | 0.82 | 13.4 ns to 13.5 ns | 11.1 ns to 11.2 ns |
| NavMeshQuery.getPolyHeight | query | 20.7 ns | 16.8 ns | 0.81 | 20.7 ns to 20.9 ns | 16.8 ns to 16.9 ns |
| NavMeshQuery.isValidPolyRef | query | 2.3 ns | 1.0 ns | 0.45 | 2.3 ns to 2.3 ns | 1.0 ns to 1.0 ns |
| NavMeshQuery.findPath | query | 3079 ns | 2795 ns | 0.91 | 3075 ns to 3086 ns | 2786 ns to 2800 ns |
| NavMeshQuery sliced findPath | query | 1427 ns | 1200 ns | 0.84 | 1421 ns to 1435 ns | 1197 ns to 1203 ns |
| NavMeshQuery sliced findPath, any angle | query | 2758 ns | 2365 ns | 0.86 | 2752 ns to 2777 ns | 2356 ns to 2369 ns |
| NavMeshQuery.findStraightPath | query | 127 ns | 101 ns | 0.80 | 126 ns to 129 ns | 101 ns to 102 ns |
| NavMeshQuery.findStraightPath, all crossings | query | 220 ns | 158 ns | 0.72 | 219 ns to 221 ns | 157 ns to 159 ns |
| NavMeshQuery.moveAlongSurface | query | 26.4 ns | 20.3 ns | 0.77 | 26.3 ns to 26.7 ns | 20.2 ns to 20.5 ns |
| NavMeshQuery.raycast | query | 52.1 ns | 37.2 ns | 0.71 | 52.0 ns to 52.3 ns | 37.1 ns to 37.3 ns |
| NavMeshQuery.raycast, with costs | query | 59.0 ns | 44.4 ns | 0.75 | 58.9 ns to 60.8 ns | 44.3 ns to 45.0 ns |
| NavMeshQuery.raycastPath | query | 51.1 ns | 37.5 ns | 0.73 | 51.0 ns to 51.2 ns | 37.4 ns to 37.5 ns |
| NavMeshQuery.findPolysAroundCircle | query | 431 ns | 331 ns | 0.77 | 429 ns to 432 ns | 329 ns to 334 ns |
| NavMeshQuery.findPolysAroundShape | query | 214 ns | 112 ns | 0.52 | 214 ns to 215 ns | 112 ns to 112 ns |
| NavMeshQuery.findLocalNeighbourhood | query | 261 ns | 225 ns | 0.86 | 260 ns to 263 ns | 224 ns to 227 ns |
| NavMeshQuery.getPolyWallSegments | query | 37.3 ns | 29.8 ns | 0.80 | 37.2 ns to 37.5 ns | 29.7 ns to 30.0 ns |
| NavMeshQuery.findDistanceToWall | query | 164 ns | 96.6 ns | 0.59 | 164 ns to 165 ns | 96.3 ns to 97.1 ns |
| NavMeshQuery.findRandomPoint | query | 372 ns | 144 ns | 0.39 | 371 ns to 373 ns | 143 ns to 146 ns |
| NavMeshQuery.findRandomPointAroundCircle | query | 628 ns | 532 ns | 0.85 | 625 ns to 632 ns | 529 ns to 534 ns |

#### Crowd

| feature | per | baseline | port | port / baseline | baseline range | port range |
| --- | --- | ---: | ---: | ---: | --- | --- |
| Crowd.update, 128 agents | agent step | 4232 ns | 4089 ns | 0.97 | 4225 ns to 4255 ns | 4083 ns to 4099 ns |
| Crowd.update, 16 agents | agent step | 2066 ns | 1976 ns | 0.96 | 2057 ns to 2075 ns | 1972 ns to 1979 ns |
| Crowd.update, 128 agents, no avoidance or separation | agent step | 1860 ns | 1750 ns | 0.94 | 1856 ns to 1864 ns | 1740 ns to 1761 ns |
| Crowd.addAgent + removeAgent | agent | 133 ns | 111 ns | 0.84 | 132 ns to 134 ns | 111 ns to 112 ns |
| Crowd.requestMoveTarget | call | 1.3 ns | 1.0 ns | 0.75 | 1.3 ns to 1.3 ns | 1.0 ns to 1.0 ns |
| ObstacleAvoidanceQuery.sampleVelocityGrid | situation | 13.9 us | 13.6 us | 0.98 | 13.9 us to 13.9 us | 13.6 us to 13.7 us |
| ObstacleAvoidanceQuery.sampleVelocityAdaptive | situation | 2023 ns | 1946 ns | 0.96 | 2014 ns to 2045 ns | 1928 ns to 2006 ns |
| ObstacleAvoidanceQuery.addCircle + addSegment + reset | situation | 16.9 ns | 14.2 ns | 0.84 | 16.9 ns to 17.0 ns | 14.1 ns to 14.2 ns |

#### Tile cache

| feature | per | baseline | port | port / baseline | baseline range | port range |
| --- | --- | ---: | ---: | ---: | --- | --- |
| TileCache.buildNavMeshTile | tile | 14.8 us | 15.3 us | 1.04 | 14.7 us to 15.1 us | 15.2 us to 15.6 us |
| TileCache.addObstacle + removeObstacle + update | obstacle | 36.4 us | 39.3 us | 1.08 | 35.9 us to 36.8 us | 39.1 us to 39.9 us |
| TileCache.queryTiles | query | 22.1 ns | 27.2 ns | 1.23 | 21.9 ns to 22.3 ns | 26.9 ns to 27.3 ns |
| TileCache.removeTile + addTile | tile | 8.3 ns | 9.3 ns | 1.13 | 8.2 ns to 8.3 ns | 9.3 ns to 9.5 ns |

#### Integer queries against the float port

| feature | per | baseline | port | port / baseline | baseline range | port range |
| --- | --- | ---: | ---: | ---: | --- | --- |
| integer NavMeshQuery.findNearestPoly | query | 143 ns | 239 ns | 1.67 | 142 ns to 146 ns | 238 ns to 241 ns |
| integer NavMeshQuery.getPolyHeight | query | 16.3 ns | 25.7 ns | 1.57 | 16.3 ns to 16.5 ns | 25.6 ns to 25.9 ns |
| integer NavMeshQuery.findPath | query | 2683 ns | 4247 ns | 1.58 | 2668 ns to 2708 ns | 4213 ns to 4285 ns |
| integer NavMeshQuery.findStraightPath | query | 97.9 ns | 123 ns | 1.26 | 97.5 ns to 135 ns | 123 ns to 197 ns |
| integer NavMeshQuery.moveAlongSurface | query | 19.3 ns | 25.9 ns | 1.35 | 19.0 ns to 23.4 ns | 25.0 ns to 29.7 ns |
| integer NavMeshQuery.raycast | query | 34.1 ns | 74.6 ns | 2.19 | 33.3 ns to 45.1 ns | 73.9 ns to 82.5 ns |
| integer NavMeshQuery.findPolysAroundCircle | query | 336 ns | 483 ns | 1.44 | 331 ns to 456 ns | 458 ns to 661 ns |
| integer NavMeshQuery.findDistanceToWall | query | 90.6 ns | 201 ns | 2.22 | 89.0 ns to 115 ns | 197 ns to 229 ns |

## Licence and credits

zlib licence ([`LICENSE`](LICENSE)), the original's.

Translated from Detour, DetourCrowd and DetourTileCache in
[recastnavigation](https://github.com/recastnavigation/recastnavigation)
1.6.0, by Mikko Mononen and the recastnavigation contributors. This is an
altered version: the code is rewritten in Zig with its own interface.

Made by Farhollow Games for the Loomwork Engine.
