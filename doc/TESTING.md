# Testing

The sections up to "In this repository" are the same in every Farhollow
Games port of a C or C++ library to Zig. The last section says how this
repository applies them.

## Oracle tests

The original is pinned to one upstream commit and kept in `reference/` (or
fetched as a lazy dependency), byte for byte as upstream has it. It is built
only for the tests and the benchmark, and is never part of the package.

An oracle test runs the original and the port on the same inputs and
compares the results. Where the original is exact, the test compares every
output byte and every returned value, not a summary such as a hash or one
field of the result. The inputs are:

- the original's own test data and demo assets, where it has them;
- hand-made edge cases: empty input, one element, the largest size, and
  degenerate cases;
- seeded random inputs (see Fuzzing).

`STATUS.md` lists every public function, option and mode of the original,
and names the tests that check each one. Code coverage is not the measure:
a feature counts as checked when a test compares it, not when a line is
reached.

## What "the same output" means

- Bytes and integers are equal.
- Floats are compared by their bit patterns, so `-0.0` differs from `0.0`,
  and a NaN matches only a NaN with the same bits.
- Where the original's output depends on how it is compiled (fused
  multiply-add, fast-math, signed overflow), the reference is built with
  flags that fix one result, and the port computes that result. The last
  section names the flags.
- Where a port deliberately computes differently from the original, a
  written design gives the units, scales, overflow bounds and tolerances.
  The port is then checked against an exact port within those tolerances,
  and against itself exactly.

## Fuzzing

Each fuzz test draws its inputs from a seeded generator and runs at least
1,000 cases. The seeds are recorded in the tests, so every run checks the
same inputs. A failure prints its seed, and the configuration where there is
one, so that it can be repeated. Where a repository has the option,
`zig build test -Dfuzz-seed=N` changes every fuzz test's seed, to explore
other inputs.

## Malformed input

Decoders and parsers are given truncated, corrupted and random bytes, and
must give the original's result: the same output, the same error, or the
same rejection. Where the port asserts a condition that the original
reports as an error code, or that the original does not check, `STATUS.md`
says so. Where a repository has contract tests, a program breaks one
contract per run, and the test expects the abort and its message in the safe
build modes.

## Determinism

The same inputs give the same output on every run. Where the library splits
work over threads, the tests compare the output on 1, 2 and more threads
with the output of one.

## Build modes

`zig build test` runs the tests in ReleaseSafe unless
`-Dtest-optimize=<mode>` says otherwise. ReleaseSafe keeps every assert and
integer overflow check on and runs the slow cases in reasonable time. The
last section says which modes have been run.

## Review

After a port passes its oracle tests, a separate AI agent profiles it
against the original and tunes the hot paths; each change is kept only if
the oracle tests still pass and the benchmark shows a gain. Another AI agent
then tries to break the claims: features left untested, comparisons that
check too little, fuzzing too narrow, benchmarks that favour the port,
undefined behaviour, integer overflow, and references that can outlive what
they point to. Each confirmed finding is fixed with a test that failed
before the fix. No person has yet reviewed the code or the tests.

## In this repository

The tests fetch the original at the pinned tag as a lazy dependency and
build it from source; it is test-only code, never part of the package. After
every call, the two sides must agree on every returned value and every byte
written; where a call writes into a buffer or a structure, the whole buffer
or structure is compared. Where the original would read or write outside
its input, the port refuses it.

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
benchmark's inputs. Under seeds 0 to 31 every test passes, in Debug and in
ReleaseSafe. Where an integer case lies outside a tolerance the design
states, the test applies that tolerance (a link's limits a 255th apart move
a portal end by a 255th of its edge). Five kinds of case are ties, where a
discrete choice or a near-parallel line lets the two sides differ by more
than a position's tolerance; each is recognised by its own rule, counted,
and held to a rate of the stable cases just above the worst measured:

- `raycast`, a ray near parallel to the wall it meets (sines 0.006 to
  0.037): the hit point moves along the ray by up to `T` over the sine, and
  the last step's cost ends there, so the cost may be off by the point's
  move times the dearest area cost, up to 36 mm (seeds 2, 3, 4, 12, 13, 14,
  20, 23, 26, 29; at most 2 a seed).
- `moveAlongSurface`, a walk that ends on a wall at a vertex several
  polygons share: which polygon's wall the end lands on flips, and the
  visited list is the way the search first reached that polygon, so the
  lists differ; the last polygon of each holds the end (seeds 2, 9, 10, 12,
  15, 27, 29; at most 1 a seed).
- `findDistanceToWall`, two walls stacked at one place in the xz plane: the
  distance is measured across and ties, the chosen wall flips, and the wall
  points stand 200 mm apart in height, so the normal is held to the
  direction from the integer side's own wall point (seeds 4, 11).
- `findStraightPath`, an end clamped to the last polygon's boundary where an
  edge and a vertex lie equally near the target: the clamp picks either,
  3.6 mm apart (seed 3).
- `findPath`, two parents that reach a polygon at nearly the same cost, 9 mm
  apart in the float search: the parent chosen flips, and since a node keeps
  the place it was first reached from, the integer corridor, 71.4 m over 15
  polygons, costs 2.7% less than the float port's, 73.4 m over 17, summed at
  the portals' middles (seed 13). The two searches popped the same polygons
  in the same order, and each one's own cost to the end is 73.4 m. The
  integer corridor may cost less, never more.

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
