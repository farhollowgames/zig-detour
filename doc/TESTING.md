# Testing

`zig build test` runs every test. The tests fetch the original at the pinned
tag, build it from source (test-only code, never part of the package), and
run the original and the port on the same inputs.

## What the oracle tests compare

After every call, the two sides must agree on every returned value and every
byte written, not on a summary such as a hash of one field. Where a call
writes into a buffer or a structure, the whole buffer or structure is
compared.

## What "the same output" means

Bytes are compared as bytes. Floats are compared by their bits, so the sign
of zero counts, and a NaN must be the same NaN. Where the port deliberately
differs from the original, a written design gives the units, the scales, the
overflow bounds and the tolerances, and the tests hold the port to the exact
port within those tolerances.

## Inputs and fuzzing

The inputs are the original's own test data and demo assets, hand-made edge
cases (empty, one element, the largest sizes, degenerate geometry), and
seeded random inputs. Each random test runs at least 1,000 cases, and prints
its seed and case number when one fails.

## Malformed input

Decoders and parsers must give the original's result, an error, or a
refusal on truncated, corrupted and random bytes. Where the original would
read or write outside its input, the port refuses it.

## Determinism

The same inputs give the same output on every run, and on 1, 2 and many
threads where the library can be used from threads.

## Build modes

The tests build in ReleaseSafe by default, which keeps every assert and
overflow check; `-Dtest-optimize=Debug` and the other modes work too.

## This repository

The inputs are RecastDemo's three level meshes (`dungeon.obj`,
`nav_test.obj`, `undulating.obj`), built by Recast into single tiles and into
tile grids; hand-made meshes for the edge cases; and seeded random points,
paths, agents, obstacles and tiles. The checks cover tiles, links, saved tile
state, references, points, every field of every crowd agent, and the calls a
query makes to a filter's own functions. Tiles damaged at random are refused
by the port where the original would read or write outside them, and taken
as the original takes them otherwise.

The suite has 368 tests, and it runs twice: with every build switch off, and
with every one on (`-Dpolyref64`, `-Dvirtual-query-filter`, `-Dfast-math`).
One test, of a filter's own functions, needs the virtual query filter and is
skipped in the first run. Both sides are built with fused multiply-add off,
so float steps round as written. Where the original's behaviour is undefined
(out-of-range float conversions, reads past a buffer), the port does what
the original does on x86-64 or refuses the input, the same on every machine;
[`parity.md`](parity.md) lists each case.

Each test has its own fixed seed. `zig build test -Dfuzz-seed=<n>` moves
every seeded differential test to other inputs at once; `0`, the default,
keeps the tests' own seeds. A few seeds stay fixed whatever the option: those
paired with the original's own random number generator, the integer
workload and the level it runs on (its hash is kept in the test), and the
benchmark's inputs. Under seeds 1 to 31 every comparison with the original
passes, in Debug and in ReleaseSafe. Where an integer case lies outside a
tolerance the design states, the test applies that tolerance (a link's
limits a 255th apart move a portal end by a 255th of its edge). The cases
below need a tolerance or rule the design does not state. They are left
failing under those seeds, the same in both modes, until the author decides:

- `raycast`: the cost of a grazing hit, 9 to 31 mm off where 7 to 16 mm is
  allowed (seeds 2, 3, 4, 12, 13, 14, 20, 23, 26, 29); the cost's tolerance
  lacks the 1/sine factor the hit point's has.
- `moveAlongSurface`: the same end with a different list of visited
  polygons, from a tie at a shared edge (seeds 2, 9, 10, 12, 15, 27, 29).
- `findDistanceToWall`: one normal 0.016 off, from two walls at nearly the
  same distance (seeds 4, 11).
- `findStraightPath`: an end clamped to the boundary where the float port has
  a near tie between an edge and a vertex, 3.6 mm apart (seed 3).
- `findPath`: a complete sliced search where the integer corridor, 71.4 m
  over 15 polygons, costs 2.7% less than the float port's, 73.4 m over 17
  (seed 13); the design allows 0.1% either way.

Two cases on the dungeon level under seed 5 were test errors, now fixed. A
walk's end 0.28 mm outside a polygon edge on the integer mesh lay 0.5 mm
inside it on the float mesh, so one side kept the target's height and the
other took the wall's, 405 mm apart; the test now counts a case as stable
only if the float walk ends the same way, at the target or on a wall, for
every nudge. A ray rising 4.9 m over 0.3 m across had its hit height held
to the tolerance across the ground; the test now scales it by the ray's
rise.

Wider seeds found more test errors, now fixed. The check that a straight
path is walkable casts the float port's ray along each leg, and a ray that
runs exactly along a wall at 45 degrees stops where it starts; the check now
steps on to the wall's far end there (seeds 8, 20, 21, 24, and a test of the
seed-8 leg). Where both searches end on a partial result at different
polygons, the corridors are no longer compared: two nodes lie as near the
end within rounding, and the test holds their distances to the end within
`2 T` of each other instead. With the crossing options, the crossings are
compared one by one under the rules of the design rather than as one line.
A random point on one polygon was held to the polygon's interior, where the
design allows it within 1 mm of the polygon (seed 12). Two comparisons with the original
were in error too. A damaged tile went to the original as the port had left
it after adding it: adding a tile moves each off-mesh connection's start
vertex onto the mesh, and where the damage made that vertex a polygon's too,
the original read a different polygon (seed 23); the original now gets the
tile as damaged. A ray from a start on its polygon's edge that leaves the
polygon misses it, and both sides give no polygon, where the test expected
the start polygon (seed 12).

Queries on separate `NavMeshQuery` values over one shared `NavMesh` give the
same bytes on 1, 2 and many threads, the way the original documents its use
from threads.

### The integer variant

The integer queries cannot match the original byte for byte, so the tests
hold them to the float port within the tolerances written in
[`integer-design.md`](integer-design.md) (2 mm for a position), on four
levels and about 1,200 seeded cases per query, comparing only the cases
where the float port's answer does not change when the input moves by a
millimetre. Hand-made meshes cover the empty mesh, a single polygon, points
exactly on an edge or a vertex, and an edge that rounds to nothing. A level
spanning the whole coordinate range runs every query with overflow checks
on. A fixed workload gives the same bytes twice, on 1, 2 and many threads,
and a hash kept in the test, the same in Debug, ReleaseSafe and ReleaseFast.
A test reads the integer source and finds no float type, float builtin or
float literal in it.

RecastDemo places `undulating.obj` about 5 km from the origin. There a
32-bit float steps in about half a millimetre, so the float port's own
rounding would take up a quarter of the 2 mm tolerance before any query
runs. The comparisons therefore use the level moved 5 km nearer the origin.
At its own place the integer variant is checked against itself instead: the
same hash on a second run and on another thread, every query succeeding, and
the answers holding what each query promises (points on their polygons,
straight paths through every portal of their corridor, wall points at the
distance given).

Where the two sides may differ (a polygon at the border of a query box, two
polygons or walls equally near, a corner at a vertex several portals share,
a float path that takes a longer way), each test counts the cases and holds
the count, as a rate of the stable cases, to a limit just above the worst
it measures over seeds 0 to 31 in Debug and in ReleaseSafe; the measured
worst is written beside each limit in `test/integer_test.zig`.
