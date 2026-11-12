# Benchmarks

`zig build bench` times the port against the original on the same inputs, one
feature per row. `zig build bench -- <text>` runs only the rows whose name
holds `<text>` (several texts may be given), and `--rounds=<n>` sets how many
rounds each side is timed.

## Machine and flags

The tables below were measured on an AMD Ryzen 9 9950X3D under Linux 7.3,
with the process pinned to one core (`taskset -c 6 zig build bench`), on
11 November 2026. The runs began at a load average under 3 on its 32
threads, and other work on the machine raised it to 20 to 26 while they ran,
so the whole benchmark ran three times and each side's best median of the
three is given, with the range of that run.

- The port: Zig 0.17.0, ReleaseFast, for the machine's own CPU, every build
  switch off.
- The original: recastnavigation 1.6.0, built from source by the Clang that
  Zig 0.17.0 carries (22.1.8), at `-O3` for the same CPU, with
  `-ffp-contract=off -fno-fast-math -fno-exceptions -fno-rtti -DNDEBUG
  -DRC_DISABLE_ASSERTS` and every build switch off.

## Method

Each row first runs both sides once and compares their checksums, then warms
both up for three rounds, then times 21 rounds of each side, taking turns, so
a slow moment of the machine falls on both. A round is as many calls as last
at least 20 ms. The tables give the median round, per item, and the fastest
to the slowest round. Each side adds what it returns into a checksum that is
kept, so the compiler cannot drop the work, and the two checksums must be
equal; a row whose checksums differ is marked and the run fails. The integer
rows are the exception: the integer variant need not agree with the float
port bit for bit, so its checksums are not compared.

The ratio is port time divided by baseline time: under 1, the port takes
less time. The row in MB/s gives the ratio of times too. The baseline is the
original, and in the last table the float port.

## Inputs

- RecastDemo's `nav_test.obj`, built by Recast into tiles of 32 cells of
  0.3 m, with 32 seeded off-mesh connections for the navigation mesh rows.
- 1,000 seeded ground points, and pairs of them, for the queries.
- For the crowd, 256 places on the level that one path joins; crowds of 16
  and 128 agents walking between them, 64 updates of 1/20 s a call; and 128
  paths between pairs of places for the corridor and the path queue.
- For the tile cache, the same level in tiles of 48 cells, every layer
  packed by the original's builder and stored as it is by one pass-through
  compressor on both sides.

## Known biases

- Every row calls each library function it times, on both sides, through a
  function pointer the compiler cannot see through (`opaqueFn` in
  `tools/bench.zig`), taken once before the row's loop. The port's functions
  are therefore never inlined into a row, nor specialised on what the row
  passes them: each runs as compiled on its own, as the original's library
  does. Without it, the port's side could be folded into the timing loop; a
  generator handed to `findRandomPoint` from a single call site was
  devirtualized that way and the row read 0.39 where it is 0.69.
- The original is reached through a thin C function that calls the library,
  built separately, so each of its calls costs one call more than the
  port's. The row "calibration: one call through an opaque pointer" times
  that one call alone on each side, through a function that returns its
  argument. On rows of a few nanoseconds (the navigation mesh lookups, the
  geometry helpers, `isValidPolyRef`, `requestMoveTarget`) the extra call is
  a large part of the difference; rows of a whole query, a crowd update or a
  tile rebuild are not much affected.
- The original is built with `-ffp-contract=off`, as in the tests, so
  neither side fuses a multiply and an add. Clang's default would fuse within
  an expression on this CPU.
- Both sides allocate through the C library's `malloc`: the original through
  its default allocator, the port through `std.heap.c_allocator`.
- One machine, one core, one level.

## Where the port is slower

- `navMeshDataSwapEndian` (1.15): both sides swap the vertices sixteen words
  at a time, and the port swaps each polygon, tree node and detail record
  with one shuffle; the cause of the rest of the gap is not found yet.
- The geometry helpers of a few nanoseconds, called through the opaque
  pointer: `intersectSegmentPoly2d` (2.10), `intersectSegmentSegment2d`
  (1.39), `closestHeightOnTriangle` (1.32), `distancePointSegmentSqr2d`
  (1.28), `closestPointOnTriangle` (1.19), `overlapPolyPoly2d` (1.10). Each
  takes the same steps as the original; on calls this short the way the
  result comes back (an optional or a struct, against the original's out
  pointers) and the call itself weigh, and the cause is not found yet.
- `ObstacleAvoidanceQuery.addCircle + addSegment + reset` (1.22),
  `NavMesh.addTile + removeTile` (1.10) and the tile cache rows (1.02 to
  1.23): each takes the same steps as the original, and the cause is not
  found yet.
- The integer queries take 1.07 to 1.68 times as long as the float port's.
  Their arithmetic is exact: a product of two products is 128 bits, a hit
  along a ray or an edge is an exact fraction compared by cross
  multiplication, and a rounded division is an integer division, slower
  than the float port's. The widest gaps are `raycast` (1.68: two fraction
  compares per polygon edge, and a normal made unit length by a root and
  integer divisions), `moveAlongSurface` (1.46), `getPolyHeight` (1.40: an
  integer division per height) and `findDistanceToWall` (1.39). Before this
  round of work they took 1.27 to 2.23 times as long; see
  `integer-design.md` for where 64 bits replaced 128 and divisions became
  products.

## Navigation mesh and tile builder

| feature | per | baseline | port | port / baseline | baseline range | port range |
| --- | --- | ---: | ---: | ---: | --- | --- |
| calibration: one call through an opaque pointer | call | 1.1 ns | 1.1 ns | 1.00 | 1.1 ns to 1.1 ns | 1.1 ns to 1.1 ns |
| createNavMeshData | tile | 351 ns | 342 ns | 0.97 | 349 ns to 355 ns | 339 ns to 344 ns |
| NavMesh.addTile + removeTile | tile | 302 ns | 331 ns | 1.10 | 301 ns to 307 ns | 328 ns to 365 ns |
| NavMesh.storeTileState + restoreTileState | tile | 9.4 ns | 9.0 ns | 0.96 | 9.4 ns to 9.5 ns | 9.0 ns to 9.0 ns |
| NavMesh.getTileAndPolyByRef | call | 2.1 ns | 2.0 ns | 0.95 | 2.1 ns to 2.1 ns | 2.0 ns to 2.0 ns |
| NavMesh.isValidPolyRef | call | 1.5 ns | 1.3 ns | 0.87 | 1.4 ns to 1.7 ns | 1.1 ns to 1.3 ns |
| NavMesh.setPolyFlags + getPolyFlags | call | 1.6 ns | 1.5 ns | 0.94 | 1.6 ns to 1.6 ns | 1.5 ns to 1.5 ns |
| NavMesh.setPolyArea + getPolyArea | call | 1.7 ns | 1.5 ns | 0.88 | 1.7 ns to 1.7 ns | 1.5 ns to 1.5 ns |
| NavMesh.calcTileLoc | call | 1.4 ns | 1.4 ns | 1.00 | 1.2 ns to 1.4 ns | 1.4 ns to 1.4 ns |
| NavMesh.getTilesAt | call | 2.5 ns | 2.0 ns | 0.80 | 2.4 ns to 2.6 ns | 2.0 ns to 2.1 ns |
| NavMesh.getTileAt | call | 1.7 ns | 1.5 ns | 0.88 | 1.6 ns to 1.7 ns | 1.5 ns to 1.6 ns |
| NavMesh.getTileRefAt | call | 1.6 ns | 1.6 ns | 1.00 | 1.6 ns to 1.7 ns | 1.6 ns to 1.6 ns |
| NavMesh.getTileByRef | call | 1.4 ns | 1.2 ns | 0.86 | 1.4 ns to 1.4 ns | 1.2 ns to 1.2 ns |
| NavMesh.getOffMeshConnectionPolyEndPoints | call | 4.0 ns | 3.7 ns | 0.93 | 4.0 ns to 4.0 ns | 3.7 ns to 3.8 ns |
| NavMesh.getOffMeshConnectionByRef | call | 1.9 ns | 1.6 ns | 0.84 | 1.9 ns to 2.0 ns | 1.5 ns to 1.7 ns |
| nav_mesh_builder.navMeshDataSwapEndian | MB/s | 72717 | 63156 | 1.15 | 69685 to 73013 | 60533 to 63484 |
| nav_mesh_builder.navMeshHeaderSwapEndian | call | 3.3 ns | 2.3 ns | 0.70 | 3.3 ns to 3.4 ns | 2.3 ns to 2.5 ns |
| common.intersectSegmentPoly2d | call | 4.9 ns | 10.3 ns | 2.10 | 4.9 ns to 7.5 ns | 10.2 ns to 10.3 ns |
| common.distancePointPolyEdgesSqr | call | 7.2 ns | 7.1 ns | 0.99 | 7.1 ns to 7.5 ns | 7.0 ns to 7.4 ns |
| common.pointInPolygon | call | 2.6 ns | 2.5 ns | 0.96 | 2.5 ns to 2.6 ns | 2.5 ns to 2.6 ns |
| common.closestHeightOnTriangle | call | 2.8 ns | 3.7 ns | 1.32 | 2.7 ns to 2.8 ns | 3.7 ns to 3.7 ns |
| common.closestPointOnTriangle | call | 3.6 ns | 4.3 ns | 1.19 | 3.6 ns to 4.9 ns | 4.2 ns to 6.4 ns |
| common.overlapPolyPoly2d | call | 18.9 ns | 20.8 ns | 1.10 | 18.8 ns to 19.8 ns | 20.6 ns to 21.1 ns |
| common.randomPointInConvexPoly | call | 4.9 ns | 4.0 ns | 0.82 | 4.8 ns to 4.9 ns | 4.0 ns to 4.1 ns |
| common.distancePointSegmentSqr2d | call | 1.8 ns | 2.3 ns | 1.28 | 1.8 ns to 2.6 ns | 2.3 ns to 3.7 ns |
| common.intersectSegmentSegment2d | call | 1.8 ns | 2.5 ns | 1.39 | 1.8 ns to 1.8 ns | 2.5 ns to 2.6 ns |
| common.calcPolyCenter | call | 2.7 ns | 2.2 ns | 0.81 | 2.7 ns to 2.8 ns | 2.2 ns to 2.4 ns |

## Queries

| feature | per | baseline | port | port / baseline | baseline range | port range |
| --- | --- | ---: | ---: | ---: | --- | --- |
| NavMeshQuery.findNearestPoly | query | 178 ns | 150 ns | 0.84 | 176 ns to 179 ns | 149 ns to 151 ns |
| NavMeshQuery.queryPolygons | query | 50.3 ns | 45.1 ns | 0.90 | 50.1 ns to 51.0 ns | 44.9 ns to 51.9 ns |
| NavMeshQuery.queryPolygonsWith | query | 46.2 ns | 42.4 ns | 0.92 | 45.9 ns to 46.8 ns | 42.3 ns to 43.8 ns |
| NavMeshQuery.closestPointOnPoly | query | 23.9 ns | 22.1 ns | 0.92 | 23.7 ns to 24.0 ns | 21.9 ns to 22.2 ns |
| NavMeshQuery.closestPointOnPolyBoundary | query | 13.7 ns | 11.5 ns | 0.84 | 13.6 ns to 13.8 ns | 11.4 ns to 11.6 ns |
| NavMeshQuery.getPolyHeight | query | 21.1 ns | 17.7 ns | 0.84 | 20.9 ns to 22.9 ns | 17.6 ns to 17.9 ns |
| NavMeshQuery.isValidPolyRef | query | 2.4 ns | 1.4 ns | 0.58 | 2.3 ns to 2.5 ns | 1.4 ns to 1.5 ns |
| NavMeshQuery.findPath | query | 3117 ns | 2831 ns | 0.91 | 3094 ns to 3124 ns | 2808 ns to 2860 ns |
| NavMeshQuery sliced findPath | query | 1430 ns | 1250 ns | 0.87 | 1423 ns to 1443 ns | 1243 ns to 1254 ns |
| NavMeshQuery sliced findPath, any angle | query | 2780 ns | 2436 ns | 0.88 | 2758 ns to 2858 ns | 2407 ns to 2685 ns |
| NavMeshQuery.findStraightPath | query | 133 ns | 105 ns | 0.79 | 131 ns to 136 ns | 104 ns to 110 ns |
| NavMeshQuery.findStraightPath, all crossings | query | 229 ns | 165 ns | 0.72 | 224 ns to 234 ns | 163 ns to 169 ns |
| NavMeshQuery.moveAlongSurface | query | 26.7 ns | 18.4 ns | 0.69 | 26.5 ns to 27.4 ns | 18.3 ns to 19.0 ns |
| NavMeshQuery.raycast | query | 53.0 ns | 38.0 ns | 0.72 | 52.8 ns to 91.0 ns | 37.8 ns to 70.3 ns |
| NavMeshQuery.raycast, with costs | query | 59.7 ns | 46.0 ns | 0.77 | 59.6 ns to 62.3 ns | 45.8 ns to 46.6 ns |
| NavMeshQuery.raycastPath | query | 51.9 ns | 37.6 ns | 0.72 | 51.8 ns to 52.1 ns | 37.4 ns to 37.8 ns |
| NavMeshQuery.findPolysAroundCircle | query | 437 ns | 340 ns | 0.78 | 433 ns to 448 ns | 335 ns to 355 ns |
| NavMeshQuery.findPolysAroundShape | query | 219 ns | 130 ns | 0.59 | 217 ns to 231 ns | 129 ns to 161 ns |
| NavMeshQuery.findLocalNeighbourhood | query | 269 ns | 229 ns | 0.85 | 268 ns to 281 ns | 228 ns to 234 ns |
| NavMeshQuery.getPolyWallSegments | query | 37.9 ns | 31.2 ns | 0.82 | 37.6 ns to 40.0 ns | 31.0 ns to 33.0 ns |
| NavMeshQuery.findDistanceToWall | query | 169 ns | 103 ns | 0.61 | 168 ns to 177 ns | 102 ns to 106 ns |
| NavMeshQuery.findRandomPoint | query | 380 ns | 262 ns | 0.69 | 378 ns to 395 ns | 259 ns to 267 ns |
| NavMeshQuery.findRandomPointAroundCircle | query | 645 ns | 541 ns | 0.84 | 641 ns to 649 ns | 536 ns to 549 ns |

## Crowd

| feature | per | baseline | port | port / baseline | baseline range | port range |
| --- | --- | ---: | ---: | ---: | --- | --- |
| Crowd.update, 128 agents | agent step | 4340 ns | 4222 ns | 0.97 | 4287 ns to 4608 ns | 4175 ns to 4470 ns |
| Crowd.update, 16 agents | agent step | 2123 ns | 2049 ns | 0.97 | 2109 ns to 2154 ns | 2026 ns to 2076 ns |
| Crowd.update, 128 agents, no avoidance or separation | agent step | 1906 ns | 1810 ns | 0.95 | 1889 ns to 1926 ns | 1765 ns to 1826 ns |
| Crowd.addAgent + removeAgent | agent | 138 ns | 121 ns | 0.88 | 137 ns to 140 ns | 120 ns to 121 ns |
| Crowd.requestMoveTarget | call | 1.7 ns | 1.3 ns | 0.76 | 1.6 ns to 1.8 ns | 1.3 ns to 1.4 ns |
| ObstacleAvoidanceQuery.sampleVelocityGrid | situation | 14.1 us | 13.8 us | 0.98 | 14.1 us to 14.5 us | 13.7 us to 14.1 us |
| ObstacleAvoidanceQuery.sampleVelocityAdaptive | situation | 1989 ns | 1981 ns | 1.00 | 1979 ns to 2047 ns | 1971 ns to 2192 ns |
| ObstacleAvoidanceQuery.addCircle + addSegment + reset | situation | 17.2 ns | 21.0 ns | 1.22 | 17.1 ns to 18.8 ns | 20.8 ns to 23.7 ns |
| PathCorridor.setCorridor + findCorners | path | 119 ns | 99.7 ns | 0.84 | 118 ns to 186 ns | 99.1 ns to 161 ns |
| PathCorridor.setCorridor + movePosition | path | 67.7 ns | 60.9 ns | 0.90 | 67.0 ns to 78.9 ns | 60.2 ns to 61.8 ns |
| PathCorridor.setCorridor + optimizePathVisibility | path | 70.4 ns | 50.2 ns | 0.71 | 70.2 ns to 71.1 ns | 50.0 ns to 50.4 ns |
| PathCorridor.setCorridor + optimizePathTopology | path | 363 ns | 258 ns | 0.71 | 362 ns to 493 ns | 257 ns to 283 ns |
| LocalBoundary.update | call | 701 ns | 622 ns | 0.89 | 694 ns to 705 ns | 621 ns to 625 ns |
| ProximityGrid.clear + addItem + queryItems | agent | 355 ns | 231 ns | 0.65 | 350 ns to 362 ns | 229 ns to 235 ns |
| PathQueue.request + update + getPathResult | path | 362 ns | 263 ns | 0.73 | 361 ns to 364 ns | 262 ns to 264 ns |

## Tile cache

| feature | per | baseline | port | port / baseline | baseline range | port range |
| --- | --- | ---: | ---: | ---: | --- | --- |
| TileCache.buildNavMeshTile | tile | 15.0 us | 15.3 us | 1.02 | 15.0 us to 16.1 us | 15.2 us to 15.6 us |
| TileCache.addObstacle + removeObstacle + update | obstacle | 36.9 us | 39.9 us | 1.08 | 36.5 us to 58.3 us | 39.3 us to 64.6 us |
| TileCache.queryTiles | query | 22.6 ns | 27.9 ns | 1.23 | 22.5 ns to 22.8 ns | 27.8 ns to 29.6 ns |
| TileCache.removeTile + addTile | tile | 10.4 ns | 8.2 ns | 0.79 | 10.2 ns to 11.6 ns | 8.0 ns to 11.1 ns |

## Integer queries against the float port

| feature | per | baseline | port | port / baseline | baseline range | port range |
| --- | --- | ---: | ---: | ---: | --- | --- |
| integer NavMeshQuery.findNearestPoly | query | 153 ns | 194 ns | 1.27 | 149 ns to 160 ns | 190 ns to 204 ns |
| integer NavMeshQuery.getPolyHeight | query | 18.0 ns | 25.2 ns | 1.40 | 17.6 ns to 18.5 ns | 24.7 ns to 26.1 ns |
| integer NavMeshQuery.findPath | query | 2690 ns | 2939 ns | 1.09 | 2684 ns to 2694 ns | 2934 ns to 3294 ns |
| integer NavMeshQuery.findStraightPath | query | 110 ns | 128 ns | 1.16 | 106 ns to 141 ns | 125 ns to 143 ns |
| integer NavMeshQuery.moveAlongSurface | query | 19.4 ns | 28.3 ns | 1.46 | 18.9 ns to 34.0 ns | 28.0 ns to 47.1 ns |
| integer NavMeshQuery.raycast | query | 35.9 ns | 60.3 ns | 1.68 | 35.6 ns to 36.5 ns | 59.9 ns to 62.5 ns |
| integer NavMeshQuery.findPolysAroundCircle | query | 341 ns | 365 ns | 1.07 | 334 ns to 497 ns | 360 ns to 558 ns |
| integer NavMeshQuery.findDistanceToWall | query | 98.5 ns | 137 ns | 1.39 | 97.6 ns to 99.9 ns | 136 ns to 145 ns |
