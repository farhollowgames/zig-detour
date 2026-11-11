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

The suite has 365 tests, and it runs twice: with every build switch off, and
with every one on (`-Dpolyref64`, `-Dvirtual-query-filter`, `-Dfast-math`).
One test, of a filter's own functions, needs the virtual query filter and is
skipped in the first run. Both sides are built with fused multiply-add off,
so float steps round as written. Where the original's behaviour is undefined
(out-of-range float conversions, reads past a buffer), the port does what
the original does on x86-64 or refuses the input, the same on every machine;
[`parity.md`](parity.md) lists each case.

The seeds are fixed in each test. There is no `-Dfuzz-seed` option yet to
offset them all at once.

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
