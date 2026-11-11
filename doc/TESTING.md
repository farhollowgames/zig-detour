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
benchmark's inputs. Under seeds 1 to 6 every comparison with the original
passes, in Debug and in ReleaseSafe. The integer comparisons then fail on a
few cases per seed, each traced to a tolerance the tests do not yet apply,
not to a wrong answer:

- `findPath` with a partial result: the two sides stop at different nodes
  equally near the end, so their corridors' costs are not comparable.
- `findStraightPath`: an end clamped to the boundary where the float port has
  a near tie between an edge and a vertex (3.6 mm); a portal end at a tile
  border whose link limits differ by a 255th of the edge (up to 14 mm, which
  the test of corners does not allow for); and a corner 2.1 mm off.
- `raycast`: the cost of a grazing hit, about 10 mm off, where the cost's
  tolerance lacks the 1/sine factor the hit point's has.
- `moveAlongSurface`: the same end within 0.5 mm with a different list of
  visited polygons, from a tie at a shared edge.
- `findDistanceToWall`: one normal off by 0.016, from two walls at nearly
  the same distance.

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
the count to a limit just above what it measures over seeds 0 to 6 (read
with the failing cases listed above set aside); the measured values are
written beside the limits in `test/integer_test.zig`.
