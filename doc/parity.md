# Where the port differs from the original

The port gives the original's results: the tests compare every returned
value and every byte written. This file lists the places where it does
something else on purpose, each with its reason. Most are inputs on which the
original's behaviour is undefined (it reads past a buffer, divides by zero or
converts a float no integer holds), where the port picks one defined result:
what the original does on x86-64 when that is harmless, a refusal when it is
not.

[`STATUS.md`](../STATUS.md) lists what was left out of the port altogether
and what it adds.

## Float to integer conversions

C++ leaves the conversion of a float outside the integer's range, or of a
NaN, undefined. On x86-64 the original's compilers emit `cvttss2si`, which
gives `0x80000000` for such a value, then keep the low bits for a narrower
type. `common.truncToInt` does exactly that, on every target. It is used
where the original casts a float it has not bounded:

| where | sites |
| --- | --- |
| `createNavMeshData`: the tree's boxes | 8 |
| `NavMesh.calcTileLoc` and the tile queries of `NavMesh` | the tile cell and the query box |
| `NavMeshQuery.queryPolygons`: the query box in tree units | 6 |
| `TileCache`: obstacle and query boxes in tiles | 5 |

Everywhere else the float is bounded before the cast and the plain
conversion is used.

## Sorting

`createNavMeshData` orders the tree's items with C's `qsort`, whose order of
equal items depends on the C library. The port uses `std.mem.sort`, which
keeps equal items in their first order. The tests build the original with a
stable sort in place of the library's, so both give the same tree. Against a
C library whose `qsort` is not stable, the original's tree can hold equal
items in another order. Both trees are valid: a query finds the same
polygons in either, in another order where items were equal.

## Trigonometry

`@sin` and `@cos` stand in for `sinf` and `cosf`. The tests compare them
with the C library's bit for bit over finite inputs within 1000 radians
either way, which covers every use in the library (the angles of sampling
patterns and of oriented boxes).

## Inputs the original does not guard

| call | original | port |
| --- | --- | --- |
| `NavMesh.init` with more tile and polygon bits than a reference holds, or negative counts | undefined shifts, a huge allocation | `invalid_param` |
| `NavMesh.addTile` with a header whose counts need more bytes than the data has | reads past the data | `invalid_param` |
| `NavMesh.removeTile` with the reference of an unused entry whose salt matches | dereferences a null header | `invalid_param` |
| `NavMesh.initSingleTile` that fails | leaves the mesh half set up | leaves it `empty` |
| `createNavMeshData` with negative counts | undefined | `error.InvalidParam` |
| `createNavMeshData` without a detail mesh, with polygons of fewer than two vertices, when the triangle fans of the others no longer fit in the tile | writes past the tile | `error.InvalidParam` |
| `createNavMeshData` with a detail mesh of fewer vertices than its polygon | a negative count of vertices to copy | `error.InvalidParam` |
| the three byte-swap functions on data shorter than its header says | read and write past the data | return false, data untouched |
| `NodePool.init` with 1 to 3 nodes | a hash size of 0, then a modulo by zero | a hash size of 1 |
| `NodeQueue.pop` and `top` on an empty queue | read before the heap | assert |
| `NavMeshQuery.init` that runs out of memory | keeps what it had allocated | frees it all and stays `empty` |
| `finalizeSlicedFindPath` and `finalizeSlicedFindPathPartial` with an empty buffer | undefined for some states | `invalid_param`, the search not ended |
| `findStraightPath` on a corridor with an invalid polygon, when the buffer fills at that point | writes one corner past the buffer | refuses the write and reports `buffer_too_small` |
| a query started on a node pool of zero nodes | dereferences null | panics at the first node |
| `Crowd.init` on a crowd already set up | leaks the first set-up | frees it first |
| a crowd agent with a filter or avoidance type out of range | reads past the arrays | panics |
| `TileCache.init` with a negative count | a failed allocation, then a destructor that walks a table that is not there | `out_of_memory`, the cache left as it was |
| `TileCache.addTile` and the layer decoder on short data | read past the data | `invalid_param` |
| `TileCache.removeTile` with the reference of an unused entry whose salt matches | dereferences a null header | `invalid_param` |
| `TileCache.buildNavMeshTile` with a reference whose tile index is the table's size | reads the entry after the last one | `invalid_param` |
| `LocalBoundary.addSegment` with a NaN distance and a full list | writes past the list | the segment is dropped |
| `PathCorridor.mergeCorridorStartShortcut` with more polygons than the corridor holds | writes past the corridor | keeps the first that fit |
| `PathQueue`'s queue head | signed overflow after 2^31 requests | wraps |

## Values the original leaves unset

Where the original reads memory it never wrote, the port reads zero:

- the node array of a new `NodePool` is zero-filled;
- in `findRandomPointAroundCircle` and `findPolysAroundCircle`, a link whose
  portal cannot be found leaves the portal at the origin (the original keeps
  whatever the last one was, or stack contents on the first);
- `raycastPath` gives the zero normal when its inputs are refused;
- `PolyDetail` has its padding as a named field, always written;
- an unused tile entry has no data pointer left in it.

## Functions that return a little more

- `Status.has(detail)` is `dtStatusDetail`.
- `MeshTile.getPolyHeight` is `dtNavMesh::getPolyHeight`, which takes the
  tile it reads as its first argument; the port makes it the tile's method.
- `NavMesh.getTileStateSize` and `storeTileState` are the original's, with
  the size of each stored part computed from the port's own layouts, which
  the tests show to be the original's byte for byte.
- `common.sqrt` of a NaN: the tests accept any NaN from either side, since
  the bits of a NaN result differ between the C library and the processor.

## Fused multiply-add

The original's results depend on whether its compiler contracts `a * b + c`
into one fused step. The port never does; it gives the results of the
original compiled with `-ffp-contract=off`, which is how the tests build it.
