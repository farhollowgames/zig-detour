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
come next. [`STATUS.md`](STATUS.md) lists
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

## Licence and credits

zlib licence ([`LICENSE`](LICENSE)), the original's.

Translated from Detour, DetourCrowd and DetourTileCache in
[recastnavigation](https://github.com/recastnavigation/recastnavigation)
1.6.0, by Mikko Mononen and the recastnavigation contributors. This is an
altered version: the code is rewritten in Zig with its own interface.

Made by Farhollow Games for the Loomwork Engine.
