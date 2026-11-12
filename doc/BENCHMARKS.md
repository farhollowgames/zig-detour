# Benchmarks

`zig build bench` times the port against the original on the same inputs, one
feature per row. `zig build bench -- <text>` runs only the rows whose name
holds `<text>` (several texts may be given), and `--rounds=<n>` sets how many
rounds each side is timed.

## Machine and flags

The tables below were measured on an AMD Ryzen 9 9950X3D under Linux 7.3,
with the process pinned to one core (`taskset -c 6 zig build bench`), on
11 November 2026. The machine was busy with other work while they ran (a
load average of 8 to 26 on its 32 threads), so the whole benchmark ran three
times and each side's best median of the three is given, with the range of
that run.

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
  devirtualized that way and the row read 0.39 where it is 0.68.
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

- `navMeshDataSwapEndian`: both sides swap the vertices sixteen words at a
  time, and the port swaps each polygon, tree node and detail record with
  one shuffle; the cause of the rest of the gap is not found yet.
- `closestHeightOnTriangle`: the timing loop inlines the port's function,
  and the optional it returns stays in registers; the inlined code takes the
  same steps, in the same order, as the original's function. The cause of
  the gap is not found yet. A version that joins the inside tests without
  branches was no faster.
- `overlapPolyPoly2d`, `NavMesh.addTile + removeTile` and the tile cache
  rows: each takes the same steps as the original, and the cause is not
  found yet.
- The integer queries take 1.09 to 1.59 times as long as the float port's.
  Their arithmetic is exact: a product of two products is 128 bits, a hit
  along a ray or an edge is an exact fraction compared by cross
  multiplication, and a rounded division is an integer division, slower
  than the float port's. The widest gaps are `raycast` (1.59: two fraction
  compares per polygon edge, and a normal made unit length by a root and
  integer divisions), `getPolyHeight` (1.43: an integer division per height)
  and `findDistanceToWall` (1.42). Before this round of work they took 1.27 to
  2.23 times as long; see `integer-design.md` for where 64 bits replaced 128
  and divisions became products.

## Navigation mesh and tile builder

| feature | per | baseline | port | port / baseline | baseline range | port range |
| --- | --- | ---: | ---: | ---: | --- | --- |
| calibration: one call through the C interface | call | 0.8 ns | 0.1 ns | 0.12 | 0.8 ns to 0.8 ns | 0.1 ns to 0.1 ns |
| createNavMeshData | tile | 352 ns | 354 ns | 1.01 | 348 ns to 360 ns | 351 ns to 358 ns |
| NavMesh.addTile + removeTile | tile | 282 ns | 348 ns | 1.23 | 281 ns to 284 ns | 346 ns to 358 ns |
| NavMesh.storeTileState + restoreTileState | tile | 9.3 ns | 7.7 ns | 0.83 | 9.3 ns to 9.4 ns | 7.7 ns to 7.7 ns |
| NavMesh.getTileAndPolyByRef | call | 1.5 ns | 1.2 ns | 0.80 | 1.5 ns to 1.6 ns | 1.2 ns to 1.2 ns |
| NavMesh.isValidPolyRef | call | 1.1 ns | 0.7 ns | 0.64 | 1.1 ns to 1.1 ns | 0.7 ns to 0.7 ns |
| NavMesh.setPolyFlags + getPolyFlags | call | 1.4 ns | 0.9 ns | 0.64 | 1.4 ns to 1.4 ns | 0.9 ns to 0.9 ns |
| NavMesh.setPolyArea + getPolyArea | call | 1.4 ns | 1.0 ns | 0.71 | 1.4 ns to 1.4 ns | 1.0 ns to 1.0 ns |
| NavMesh.calcTileLoc | call | 1.0 ns | 0.9 ns | 0.90 | 1.0 ns to 1.0 ns | 0.9 ns to 0.9 ns |
| NavMesh.getTilesAt | call | 2.2 ns | 1.8 ns | 0.82 | 2.1 ns to 2.2 ns | 1.8 ns to 1.8 ns |
| NavMesh.getTileAt | call | 1.7 ns | 1.0 ns | 0.59 | 1.7 ns to 1.7 ns | 1.0 ns to 1.0 ns |
| NavMesh.getTileRefAt | call | 6.3 ns | 1.3 ns | 0.21 | 6.2 ns to 11.0 ns | 1.3 ns to 1.3 ns |
| NavMesh.getTileByRef | call | 1.2 ns | 0.6 ns | 0.50 | 1.2 ns to 1.2 ns | 0.6 ns to 0.6 ns |
| NavMesh.getOffMeshConnectionPolyEndPoints | call | 3.9 ns | 2.8 ns | 0.72 | 3.9 ns to 4.0 ns | 2.8 ns to 2.8 ns |
| NavMesh.getOffMeshConnectionByRef | call | 1.5 ns | 1.2 ns | 0.80 | 1.5 ns to 1.5 ns | 1.2 ns to 1.2 ns |
| nav_mesh_builder.navMeshDataSwapEndian | MB/s | 71452 | 61603 | 1.16 | 70281 to 72573 | 61043 to 61746 |
| nav_mesh_builder.navMeshHeaderSwapEndian | call | 3.0 ns | 3.0 ns | 1.00 | 2.9 ns to 3.0 ns | 2.9 ns to 3.0 ns |
| common.intersectSegmentPoly2d | call | 4.6 ns | 3.7 ns | 0.80 | 4.6 ns to 4.6 ns | 3.7 ns to 3.8 ns |
| common.distancePointPolyEdgesSqr | call | 7.1 ns | 6.7 ns | 0.94 | 7.1 ns to 7.2 ns | 6.7 ns to 6.8 ns |
| common.pointInPolygon | call | 2.5 ns | 2.2 ns | 0.88 | 2.5 ns to 2.6 ns | 2.2 ns to 2.2 ns |
| common.closestHeightOnTriangle | call | 2.7 ns | 5.0 ns | 1.85 | 2.7 ns to 2.7 ns | 4.9 ns to 5.0 ns |
| common.closestPointOnTriangle | call | 3.6 ns | 3.5 ns | 0.97 | 3.5 ns to 3.6 ns | 3.5 ns to 3.6 ns |
| common.overlapPolyPoly2d | call | 18.7 ns | 20.6 ns | 1.10 | 18.6 ns to 20.4 ns | 20.4 ns to 21.0 ns |
| common.randomPointInConvexPoly | call | 4.5 ns | 3.8 ns | 0.84 | 4.5 ns to 4.5 ns | 3.8 ns to 3.9 ns |
| common.distancePointSegmentSqr2d | call | 1.8 ns | 1.5 ns | 0.83 | 1.8 ns to 1.8 ns | 1.5 ns to 1.6 ns |
| common.intersectSegmentSegment2d | call | 1.8 ns | 1.3 ns | 0.72 | 1.8 ns to 1.8 ns | 1.3 ns to 1.3 ns |
| common.calcPolyCenter | call | 2.7 ns | 1.8 ns | 0.67 | 2.7 ns to 2.8 ns | 1.8 ns to 1.8 ns |

## Queries

| feature | per | baseline | port | port / baseline | baseline range | port range |
| --- | --- | ---: | ---: | ---: | --- | --- |
| NavMeshQuery.findNearestPoly | query | 175 ns | 143 ns | 0.82 | 173 ns to 177 ns | 142 ns to 145 ns |
| NavMeshQuery.queryPolygons | query | 49.5 ns | 41.3 ns | 0.83 | 49.3 ns to 50.2 ns | 41.1 ns to 41.6 ns |
| NavMeshQuery.queryPolygonsWith | query | 45.0 ns | 38.3 ns | 0.85 | 44.8 ns to 45.4 ns | 38.1 ns to 38.6 ns |
| NavMeshQuery.closestPointOnPoly | query | 24.1 ns | 17.9 ns | 0.74 | 24.0 ns to 24.2 ns | 17.8 ns to 18.0 ns |
| NavMeshQuery.closestPointOnPolyBoundary | query | 13.6 ns | 11.0 ns | 0.81 | 13.6 ns to 13.7 ns | 11.0 ns to 11.2 ns |
| NavMeshQuery.getPolyHeight | query | 20.7 ns | 16.7 ns | 0.81 | 20.6 ns to 32.0 ns | 16.7 ns to 31.3 ns |
| NavMeshQuery.isValidPolyRef | query | 2.3 ns | 1.0 ns | 0.43 | 2.3 ns to 2.3 ns | 1.0 ns to 1.0 ns |
| NavMeshQuery.findPath | query | 3097 ns | 2806 ns | 0.91 | 3089 ns to 3113 ns | 2794 ns to 2824 ns |
| NavMeshQuery sliced findPath | query | 1437 ns | 1213 ns | 0.84 | 1435 ns to 1441 ns | 1208 ns to 1222 ns |
| NavMeshQuery sliced findPath, any angle | query | 2755 ns | 2403 ns | 0.87 | 2745 ns to 2764 ns | 2394 ns to 2418 ns |
| NavMeshQuery.findStraightPath | query | 127 ns | 101 ns | 0.80 | 127 ns to 128 ns | 101 ns to 101 ns |
| NavMeshQuery.findStraightPath, all crossings | query | 225 ns | 157 ns | 0.70 | 224 ns to 228 ns | 156 ns to 158 ns |
| NavMeshQuery.moveAlongSurface | query | 26.2 ns | 19.1 ns | 0.73 | 26.2 ns to 26.4 ns | 19.0 ns to 19.1 ns |
| NavMeshQuery.raycast | query | 52.1 ns | 36.9 ns | 0.71 | 52.0 ns to 52.3 ns | 36.8 ns to 37.1 ns |
| NavMeshQuery.raycast, with costs | query | 59.2 ns | 44.7 ns | 0.76 | 59.0 ns to 59.7 ns | 44.6 ns to 44.8 ns |
| NavMeshQuery.raycastPath | query | 51.3 ns | 37.2 ns | 0.73 | 51.1 ns to 51.5 ns | 37.1 ns to 37.4 ns |
| NavMeshQuery.findPolysAroundCircle | query | 441 ns | 330 ns | 0.75 | 440 ns to 443 ns | 327 ns to 331 ns |
| NavMeshQuery.findPolysAroundShape | query | 215 ns | 113 ns | 0.53 | 214 ns to 216 ns | 113 ns to 114 ns |
| NavMeshQuery.findLocalNeighbourhood | query | 263 ns | 223 ns | 0.85 | 262 ns to 264 ns | 221 ns to 224 ns |
| NavMeshQuery.getPolyWallSegments | query | 37.6 ns | 29.7 ns | 0.79 | 37.5 ns to 37.6 ns | 29.7 ns to 29.8 ns |
| NavMeshQuery.findDistanceToWall | query | 166 ns | 97.8 ns | 0.59 | 165 ns to 167 ns | 97.4 ns to 98.6 ns |
| NavMeshQuery.findRandomPoint | query | 371 ns | 144 ns | 0.39 | 371 ns to 371 ns | 143 ns to 146 ns |
| NavMeshQuery.findRandomPointAroundCircle | query | 642 ns | 522 ns | 0.81 | 638 ns to 644 ns | 520 ns to 534 ns |

## Crowd

| feature | per | baseline | port | port / baseline | baseline range | port range |
| --- | --- | ---: | ---: | ---: | --- | --- |
| Crowd.update, 128 agents | agent step | 4284 ns | 4151 ns | 0.97 | 4233 ns to 4307 ns | 4101 ns to 4178 ns |
| Crowd.update, 16 agents | agent step | 2117 ns | 1990 ns | 0.94 | 2099 ns to 2131 ns | 1971 ns to 2012 ns |
| Crowd.update, 128 agents, no avoidance or separation | agent step | 1872 ns | 1773 ns | 0.95 | 1854 ns to 1895 ns | 1752 ns to 1795 ns |
| Crowd.addAgent + removeAgent | agent | 133 ns | 112 ns | 0.84 | 132 ns to 134 ns | 111 ns to 114 ns |
| Crowd.requestMoveTarget | call | 1.3 ns | 1.0 ns | 0.77 | 1.3 ns to 1.3 ns | 1.0 ns to 1.0 ns |
| ObstacleAvoidanceQuery.sampleVelocityGrid | situation | 13.9 us | 13.7 us | 0.99 | 13.9 us to 14.0 us | 13.7 us to 13.8 us |
| ObstacleAvoidanceQuery.sampleVelocityAdaptive | situation | 1990 ns | 1940 ns | 0.97 | 1968 ns to 2004 ns | 1926 ns to 1957 ns |
| ObstacleAvoidanceQuery.addCircle + addSegment + reset | situation | 16.9 ns | 13.4 ns | 0.79 | 16.9 ns to 16.9 ns | 13.4 ns to 13.4 ns |
| PathCorridor.setCorridor + findCorners | path | 118 ns | 94.8 ns | 0.80 | 118 ns to 118 ns | 94.6 ns to 94.9 ns |
| PathCorridor.setCorridor + movePosition | path | 64.8 ns | 52.4 ns | 0.81 | 64.5 ns to 65.3 ns | 52.2 ns to 52.6 ns |
| PathCorridor.setCorridor + optimizePathVisibility | path | 66.9 ns | 43.6 ns | 0.65 | 66.7 ns to 67.3 ns | 43.5 ns to 43.8 ns |
| PathCorridor.setCorridor + optimizePathTopology | path | 358 ns | 248 ns | 0.69 | 357 ns to 361 ns | 247 ns to 250 ns |
| LocalBoundary.update | call | 693 ns | 602 ns | 0.87 | 692 ns to 695 ns | 600 ns to 604 ns |
| ProximityGrid.clear + addItem + queryItems | agent | 330 ns | 231 ns | 0.70 | 317 ns to 348 ns | 228 ns to 234 ns |
| PathQueue.request + update + getPathResult | path | 361 ns | 259 ns | 0.72 | 361 ns to 365 ns | 258 ns to 263 ns |

## Tile cache

| feature | per | baseline | port | port / baseline | baseline range | port range |
| --- | --- | ---: | ---: | ---: | --- | --- |
| TileCache.buildNavMeshTile | tile | 15.0 us | 15.2 us | 1.01 | 14.9 us to 15.1 us | 15.2 us to 15.3 us |
| TileCache.addObstacle + removeObstacle + update | obstacle | 36.7 us | 39.6 us | 1.08 | 36.5 us to 36.9 us | 39.4 us to 39.9 us |
| TileCache.queryTiles | query | 22.4 ns | 27.1 ns | 1.21 | 22.2 ns to 22.7 ns | 26.8 ns to 28.8 ns |
| TileCache.removeTile + addTile | tile | 8.5 ns | 9.2 ns | 1.08 | 8.4 ns to 8.5 ns | 9.1 ns to 9.3 ns |

## Integer queries against the float port

| feature | per | baseline | port | port / baseline | baseline range | port range |
| --- | --- | ---: | ---: | ---: | --- | --- |
| integer NavMeshQuery.findNearestPoly | query | 143 ns | 181 ns | 1.27 | 142 ns to 145 ns | 180 ns to 182 ns |
| integer NavMeshQuery.getPolyHeight | query | 16.4 ns | 23.5 ns | 1.43 | 16.4 ns to 16.5 ns | 23.5 ns to 23.6 ns |
| integer NavMeshQuery.findPath | query | 2638 ns | 2931 ns | 1.11 | 2632 ns to 2649 ns | 2926 ns to 2939 ns |
| integer NavMeshQuery.findStraightPath | query | 98.1 ns | 110 ns | 1.12 | 97.7 ns to 98.3 ns | 110 ns to 111 ns |
| integer NavMeshQuery.moveAlongSurface | query | 18.9 ns | 23.3 ns | 1.23 | 18.8 ns to 19.1 ns | 23.2 ns to 23.3 ns |
| integer NavMeshQuery.raycast | query | 33.2 ns | 52.9 ns | 1.59 | 33.0 ns to 33.6 ns | 52.8 ns to 53.1 ns |
| integer NavMeshQuery.findPolysAroundCircle | query | 321 ns | 350 ns | 1.09 | 320 ns to 322 ns | 348 ns to 353 ns |
| integer NavMeshQuery.findDistanceToWall | query | 88.8 ns | 126 ns | 1.42 | 88.2 ns to 89.1 ns | 125 ns to 126 ns |
