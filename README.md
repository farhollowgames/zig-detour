# zig-detour

Tracks recastnavigation 1.6.0 (Detour, DetourCrowd, DetourTileCache) · Zig 0.17 ·
[status](STATUS.md)

## What it is

A Zig port of Detour, DetourCrowd and DetourTileCache, the navigation mesh
runtime of [recastnavigation](#licence-and-references). It holds navigation
meshes in tiles and answers questions about them, moves crowds of agents
along paths, and rebuilds tiles around obstacles at run time. It is pure Zig,
with no dependencies beyond the standard library.

This port is not affiliated with or reviewed by the original authors. Its
code was written with AI assistance for Farhollow Games, and is only machine
verified. See [the tests](doc/TESTING.md) and [parity with the original](doc/parity.md).

## Features

- Navigation meshes of one tile or many, tiles added and removed at run time,
  tile state saved and restored, the tile builder and byte-order swapping.
- Queries: the nearest polygon, paths (whole or sliced, optionally any-angle),
  straight paths, moving along the surface, raycasts, the polygons around a
  circle or a shape, wall segments, the distance to a wall and random points,
  all through a query filter.
- The crowd: agents with path corridors, local boundaries, obstacle avoidance
  by grid or adaptive sampling, a proximity grid and a path queue.
- The tile cache: compressed tile layers, cylinder, box and oriented box
  obstacles, and tiles rebuilt around them, with the layer builder.
- The original's build switches as build options.
- `detour.integer`: the navigation mesh and its queries on whole millimetres,
  with no floating-point operation, so a call gives the same bytes on every
  machine.

## Install

```sh
zig fetch --save git+https://github.com/farhollowgames/zig-detour#v0.1.0
```

`v0.1.0` is the latest release.

In `build.zig`:

```zig
const detour = b.dependency("detour", .{ .target = target, .optimize = optimize });
exe.root_module.addImport("detour", detour.module("detour"));
```

The original's compile-time switches are build options, passed where the
dependency is declared, such as `.polyref64 = true`:

| option | original | effect |
| --- | --- | --- |
| `polyref64` | `DT_POLYREF64` | polygon and tile references of 64 bits instead of 32 |
| `virtual-query-filter` | `DT_VIRTUAL_QUERYFILTER` | a query filter can replace its pass and cost functions |
| `fast-math` | `RC_FAST_MATH` | no checks for infinite and NaN inputs |

## Use

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

The API follows the original's, function for function
(`dtNavMeshQuery::findPath` is `NavMeshQuery.findPath`).
[`STATUS.md`](STATUS.md) maps every function and type of the original to its
counterpart, and [`doc/integer-design.md`](doc/integer-design.md) shows the
integer variant's use, units and bounds.

## Differences from the original

The reasons are in [`doc/parity.md`](doc/parity.md).

*Additions*:

- `detour.integer`, the integer navigation mesh and queries, with
  `tileFromFloat` to convert a float tile.

*Changes to expect*:

- Every function that allocates takes a `std.mem.Allocator`, in place of the
  `dtAlloc` hooks.
- A value is declared where the caller wants it (`.empty`), readied with
  `init` and freed with `deinit`, in place of `dtAlloc*` and `dtFree*`.
- Buffers are slices, and counts come back as `usize`.
- Statuses are `detour.Status`, a packed struct of the original's bits;
  flags and options are packed structs too.
- A tile that would make the original read or write outside it is refused.
- Float to integer conversions the original leaves undefined give one fixed
  result.
- Sorting is stable where the original's `qsort` may order equal items any
  way.

## Not included

The parts of the original that are left out, each with what takes its place,
are listed in [`STATUS.md`](STATUS.md#left-out).

## Testing and benchmarks

`zig build test` runs 368 tests, each with every build switch off and with
every one on. They compare every returned value and every byte written by the
original and the port on the same inputs. The integer variant is held to the
float port within the tolerances its design states. RecastDemo's
`undulating.obj` lies 5 km from the origin, where a float's own rounding is
half a millimetre, so it is compared with the float port only after a move to
near the origin; at its own place the integer variant is checked against
itself. `-Dfuzz-seed=<n>` moves the seeded tests to other inputs.

Port time against original time, one core:

| feature | port vs original |
| --- | ---: |
| `NavMeshQuery.findPath` | 9% faster |
| `NavMeshQuery.raycast` | 28% faster |
| `Crowd.update`, 128 agents | 3% faster |
| `NavMesh.addTile + removeTile` | 10% slower |
| `TileCache.addObstacle + removeObstacle + update` | 8% slower |

The integer queries are 7% to 68% slower than the float port's (`findPath`
9%, `raycast` 68%).

AMD Ryzen 9 9950X3D, one core, Zig 0.17.0 ReleaseFast against Clang 22.1.8
`-O3`; each row is the best median of three runs.

- [`doc/TESTING.md`](doc/TESTING.md): how the tests compare the port with
  the original, and what this repository's tests cover.
- [`doc/BENCHMARKS.md`](doc/BENCHMARKS.md): ratios and full tables: every
  row, the flags on both sides, the biases and the command.

## Licence and references

zlib licence, the original's ([`LICENSE`](LICENSE)). The original's licence
and copyright notice are kept in `LICENSE` as they are:
"Copyright (c) 2009 Mikko Mononen memon@inside.org".

- [recastnavigation](https://github.com/recastnavigation/recastnavigation),
  the original, by [Mikko Mononen](https://github.com/memononen) and the
  recastnavigation contributors.
- [recastnav.com](https://recastnav.com), its documentation.

Made by Farhollow Games for the Loomwork Engine.
