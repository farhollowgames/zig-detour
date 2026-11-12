# Status

Every public function, option and mode of Detour, DetourCrowd and DetourTileCache in
recastnavigation 1.6.0, taken from the headers, the documentation and the release notes,
with its state:

- **not started**;
- **ported**: translated, and it compiles; no test compares it with the original yet;
- **oracle-tested**: a differential test (`zig build test`) runs the original and the port
  on the same inputs and compares every returned value and every byte written;
- **benchmarked**: oracle-tested, and timed against the original by `zig build bench`;
- **reviewed**: checked by the performance pass and the skeptic.

A performance pass and a skeptic's review by agents that did not write the port have gone
over every feature below, and the author has decided their findings, so every row is
reviewed; the state after "reviewed" is the furthest its tests and the benchmark go.

A struct's row names its fields, and an enum's row its values: those are the options and
modes. `doc/parity.md` lists every place where the port's behaviour differs from the
original, and why.

## Summary

| library | upstream header | Zig file | entries | not started | ported | oracle-tested | benchmarked | reviewed |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Detour | DetourNavMesh.h | `src/nav_mesh.zig` | 69 |  |  | 52 | 17 | 69 |
| Detour | DetourNavMeshBuilder.h | `src/nav_mesh_builder.zig` | 4 |  |  | 1 | 3 | 4 |
| Detour | DetourNavMeshQuery.h | `src/nav_mesh_query.zig` | 45 |  |  | 22 | 23 | 45 |
| Detour | DetourNode.h | `src/node.zig` | 33 |  |  | 33 |  | 33 |
| Detour | DetourCommon.h | `src/common.zig` | 44 |  |  | 34 | 10 | 44 |
| Detour | DetourStatus.h | `src/status.zig` | 17 |  |  | 17 |  | 17 |
| Detour | DetourMath.h | `src/common.zig` | 1 |  |  | 1 |  | 1 |
| DetourCrowd | DetourCrowd.h | `src/crowd.zig` | 37 |  |  | 33 | 4 | 37 |
| DetourCrowd | DetourPathCorridor.h | `src/path_corridor.zig` | 24 |  |  | 19 | 5 | 24 |
| DetourCrowd | DetourPathQueue.h | `src/path_queue.zig` | 11 |  |  | 7 | 4 | 11 |
| DetourCrowd | DetourLocalBoundary.h | `src/local_boundary.zig` | 8 |  |  | 7 | 1 | 8 |
| DetourCrowd | DetourObstacleAvoidance.h | `src/obstacle_avoidance.zig` | 33 |  |  | 28 | 5 | 33 |
| DetourCrowd | DetourProximityGrid.h | `src/proximity_grid.zig` | 10 |  |  | 7 | 3 | 10 |
| DetourTileCache | DetourTileCache.h | `src/tile_cache.zig` | 49 |  |  | 42 | 7 | 49 |
| DetourTileCache | DetourTileCacheBuilder.h | `src/tile_cache_builder.zig` | 30 |  |  | 30 |  | 30 |
| | | **all** | 415 |  |  | 333 | 82 | 415 |

The differential tests run twice: with every build switch off, and with every one on.

## Build switches

| upstream | Zig | state |
| --- | --- | --- |
| `DT_POLYREF64` (`RECASTNAVIGATION_DT_POLYREF64`): 64-bit polygon and tile references | `-Dpolyref64` | reviewed, oracle-tested |
| `DT_VIRTUAL_QUERYFILTER` (`RECASTNAVIGATION_DT_VIRTUAL_QUERYFILTER`): a filter replaces its pass and cost functions | `-Dvirtual-query-filter` | reviewed, oracle-tested |
| `RC_FAST_MATH` (`RECASTNAVIGATION_ENABLE_FAST_MATH`): no checks for infinite and NaN inputs | `-Dfast-math` | reviewed, oracle-tested |

## Left out

Nothing upstream does is missing. These names have no function of their own in the port,
because the language or the allocator interface does their work. Each needs the author's
yes.

| upstream | what the port has in its place |
| --- | --- |
| DetourNavMesh.h: `dtAllocNavMesh`, `dtFreeNavMesh`; DetourNavMeshQuery.h: `dtAllocNavMeshQuery`, `dtFreeNavMeshQuery`; DetourCrowd.h: `dtAllocCrowd`, `dtFreeCrowd`; DetourObstacleAvoidance.h: `dtAllocObstacleAvoidanceDebugData`, `dtFreeObstacleAvoidanceDebugData`, `dtAllocObstacleAvoidanceQuery`, `dtFreeObstacleAvoidanceQuery`; DetourProximityGrid.h: `dtAllocProximityGrid`, `dtFreeProximityGrid`; DetourTileCache.h: `dtAllocTileCache`, `dtFreeTileCache` | The caller declares the value where it wants it (`var nav: NavMesh = .empty;`) and calls `init` and `deinit`, so no function allocates or frees the object itself. |
| DetourNavMeshQuery.h: `dtQueryFilter::~dtQueryFilter`, `dtPolyQuery::~dtPolyQuery`; DetourLocalBoundary.h: `dtLocalBoundary::~dtLocalBoundary`; DetourTileCache.h: `dtTileCacheMeshProcess::~dtTileCacheMeshProcess`; DetourTileCacheBuilder.h: `dtTileCacheAlloc::~dtTileCacheAlloc`, `dtTileCacheCompressor::~dtTileCacheCompressor` | An interface is a pointer with a table of functions, and a filter or a boundary owns no memory, so there is nothing to destroy. |
| DetourCommon.h: `dtIgnoreUnused`, `dtSwap`, `dtVset`, `dtVcopy` | Part of the language: `_ = x`, `std.mem.swap`, an array literal, an assignment. |
| DetourCommon.h: `dtSwapByte`, `dtSwapEndian` for `unsigned short`, `short`, `unsigned int` and `int` | `@byteSwap`; `swapEndianFloat` is the one for a float. |
| DetourCommon.h: `dtGetThenAdvanceBufferPointer` | Slices of the tile's bytes. |
| DetourMath.h: `dtMathFabsf`, `dtMathSqrtf`, `dtMathFloorf`, `dtMathCeilf`, `dtMathCosf`, `dtMathSinf`, `dtMathAtan2f` | The builtins `@abs`, `@sqrt`, `@floor`, `@ceil`, `@cos`, `@sin`. Nothing in the three libraries calls `dtMathAtan2f`. |
| DetourAlloc.h: `dtAllocHint`: `DT_ALLOC_PERM`, `DT_ALLOC_TEMP`, `dtAllocFunc`, `dtFreeFunc`, `dtAllocSetCustom`, `dtAlloc`, `dtFree` | Every function that allocates takes a `std.mem.Allocator`, which is how a caller chooses the allocator. There is no global allocator to replace, and no permanent or temporary hint: both kinds come from the allocator passed in. |
| DetourAssert.h: `dtAssert`, `dtAssertFailFunc`, `dtAssertFailSetCustom`, `dtAssertFailGetCustom`, the `RC_DISABLE_ASSERTS` switch | `std.debug.assert`. A failed assert goes to the program's panic handler, which is where a program installs its own. ReleaseFast and ReleaseSmall compile asserts out, as `RC_DISABLE_ASSERTS` and `NDEBUG` do. |
| DetourTileCacheBuilder.h: `dtTileCacheAlloc::alloc`, `dtTileCacheAlloc::free` | `BuildAllocator.allocator`, a `std.mem.Allocator`. |
| DebugUtils: `DetourDebugDraw.h` (`duDebugDrawNavMesh` and the rest), the dump and draw helpers | Not part of Detour, DetourCrowd or DetourTileCache. Drawing belongs to the program that shows the mesh. |
| RecastDemo: the sample tools, `fastlz`, the sample `dtTileCacheAlloc`, `dtTileCacheCompressor` and `dtTileCacheMeshProcess` classes | Demo code. The tests carry their own compressor and mesh process to drive the tile cache on both sides. |

## Detour

### DetourNavMesh.h → `src/nav_mesh.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `DT_SALT_BITS` | `salt_bits_64` | reviewed, oracle-tested |
| `DT_TILE_BITS` | `tile_bits_64` | reviewed, oracle-tested |
| `DT_POLY_BITS` | `poly_bits_64` | reviewed, oracle-tested |
| `dtPolyRef` | `PolyRef` | reviewed, oracle-tested |
| `dtTileRef` | `TileRef` | reviewed, oracle-tested |
| `DT_VERTS_PER_POLYGON` | `verts_per_polygon` | reviewed, oracle-tested |
| `DT_NAVMESH_MAGIC` | `nav_mesh_magic` | reviewed, oracle-tested |
| `DT_NAVMESH_VERSION` | `nav_mesh_version` | reviewed, oracle-tested |
| `DT_NAVMESH_STATE_MAGIC` | `nav_mesh_state_magic` | reviewed, oracle-tested |
| `DT_NAVMESH_STATE_VERSION` | `nav_mesh_state_version` | reviewed, oracle-tested |
| `DT_EXT_LINK` | `ext_link` | reviewed, oracle-tested |
| `DT_NULL_LINK` | `null_link` | reviewed, oracle-tested |
| `DT_OFFMESH_CON_BIDIR` | `off_mesh_connection_bidirectional` | reviewed, oracle-tested |
| `DT_MAX_AREAS` | `area_count_max` | reviewed, oracle-tested |
| `dtTileFlags`: `DT_TILE_FREE_DATA` | `TileFlags` | reviewed, oracle-tested |
| `dtStraightPathFlags`: `DT_STRAIGHTPATH_START`, `DT_STRAIGHTPATH_END`, `DT_STRAIGHTPATH_OFFMESH_CONNECTION` | `StraightPathFlags` | reviewed, oracle-tested |
| `dtStraightPathOptions`: `DT_STRAIGHTPATH_AREA_CROSSINGS`, `DT_STRAIGHTPATH_ALL_CROSSINGS` | `StraightPathOptions` | reviewed, oracle-tested |
| `dtFindPathOptions`: `DT_FINDPATH_ANY_ANGLE` | `FindPathOptions` | reviewed, oracle-tested |
| `dtRaycastOptions`: `DT_RAYCAST_USE_COSTS` | `RaycastOptions` | reviewed, oracle-tested |
| `dtDetailTriEdgeFlags`: `DT_DETAIL_EDGE_BOUNDARY` | `detail_edge_boundary` | reviewed, oracle-tested |
| `DT_RAY_CAST_LIMIT_PROPORTIONS` | `ray_cast_limit_proportions` | reviewed, oracle-tested |
| `dtPolyTypes`: `DT_POLYTYPE_GROUND`, `DT_POLYTYPE_OFFMESH_CONNECTION` | `PolyType` | reviewed, oracle-tested |
| `dtPoly`: `firstLink`, `verts`, `neis`, `flags`, `vertCount`, `areaAndtype` | `Poly` | reviewed, oracle-tested |
| `dtPoly::setArea` | `Poly.setArea` | reviewed, oracle-tested |
| `dtPoly::setType` | `Poly.setType` | reviewed, oracle-tested |
| `dtPoly::getArea` | `Poly.getArea` | reviewed, oracle-tested |
| `dtPoly::getType` | `Poly.getType` | reviewed, oracle-tested |
| `dtPolyDetail`: `vertBase`, `triBase`, `vertCount`, `triCount` | `PolyDetail` | reviewed, oracle-tested |
| `dtLink`: `ref`, `next`, `edge`, `side`, `bmin`, `bmax` | `Link` | reviewed, oracle-tested |
| `dtBVNode`: `bmin`, `bmax`, `i` | `BvNode` | reviewed, oracle-tested |
| `dtOffMeshConnection`: `pos`, `rad`, `poly`, `flags`, `side`, `userId` | `OffMeshConnection` | reviewed, oracle-tested |
| `dtMeshHeader`: `magic`, `version`, `x`, `y`, `layer`, `userId`, `polyCount`, `vertCount`, `maxLinkCount`, `detailMeshCount`, `detailVertCount`, `detailTriCount`, `bvNodeCount`, `offMeshConCount`, `offMeshBase`, `walkableHeight`, `walkableRadius`, `walkableClimb`, `bmin`, `bmax`, `bvQuantFactor` | `MeshHeader` | reviewed, oracle-tested |
| `dtMeshTile`: `salt`, `linksFreeList`, `header`, `polys`, `verts`, `links`, `detailMeshes`, `detailVerts`, `detailTris`, `bvTree`, `offMeshCons`, `data`, `dataSize`, `flags`, `next` | `MeshTile` | reviewed, oracle-tested |
| `dtGetDetailTriEdgeFlags` | `getDetailTriEdgeFlags` | reviewed, oracle-tested |
| `dtNavMeshParams`: `orig`, `tileWidth`, `tileHeight`, `maxTiles`, `maxPolys` | `NavMeshParams` | reviewed, oracle-tested |
| `dtNavMesh` | `NavMesh` | reviewed, oracle-tested |
| `dtNavMesh::dtNavMesh` | `NavMesh.empty` | reviewed, oracle-tested |
| `dtNavMesh::~dtNavMesh` | `NavMesh.deinit` | reviewed, oracle-tested |
| `dtNavMesh::init(params)` | `NavMesh.init` | reviewed, oracle-tested |
| `dtNavMesh::init(data, dataSize, flags)` | `NavMesh.initSingleTile` | reviewed, oracle-tested |
| `dtNavMesh::getParams` | `NavMesh.getParams` | reviewed, oracle-tested |
| `dtNavMesh::addTile` | `NavMesh.addTile` | reviewed, benchmarked |
| `dtNavMesh::removeTile` | `NavMesh.removeTile` | reviewed, benchmarked |
| `dtNavMesh::calcTileLoc` | `NavMesh.calcTileLoc` | reviewed, benchmarked |
| `dtNavMesh::getTileAt` | `NavMesh.getTileAt` | reviewed, benchmarked |
| `dtNavMesh::getTilesAt` | `NavMesh.getTilesAt` | reviewed, benchmarked |
| `dtNavMesh::getTileRefAt` | `NavMesh.getTileRefAt` | reviewed, benchmarked |
| `dtNavMesh::getTileRef` | `NavMesh.getTileRef` | reviewed, oracle-tested |
| `dtNavMesh::getTileByRef` | `NavMesh.getTileByRef` | reviewed, benchmarked |
| `dtNavMesh::getMaxTiles` | `NavMesh.getMaxTiles` | reviewed, oracle-tested |
| `dtNavMesh::getTile` | `NavMesh.getTile` | reviewed, oracle-tested |
| `dtNavMesh::getTileAndPolyByRef` | `NavMesh.getTileAndPolyByRef` | reviewed, benchmarked |
| `dtNavMesh::getTileAndPolyByRefUnsafe` | `NavMesh.getTileAndPolyByRefUnsafe` | reviewed, oracle-tested |
| `dtNavMesh::isValidPolyRef` | `NavMesh.isValidPolyRef` | reviewed, benchmarked |
| `dtNavMesh::getPolyRefBase` | `NavMesh.getPolyRefBase` | reviewed, oracle-tested |
| `dtNavMesh::getOffMeshConnectionPolyEndPoints` | `NavMesh.getOffMeshConnectionPolyEndPoints` | reviewed, benchmarked |
| `dtNavMesh::getOffMeshConnectionByRef` | `NavMesh.getOffMeshConnectionByRef` | reviewed, benchmarked |
| `dtNavMesh::setPolyFlags` | `NavMesh.setPolyFlags` | reviewed, benchmarked |
| `dtNavMesh::getPolyFlags` | `NavMesh.getPolyFlags` | reviewed, benchmarked |
| `dtNavMesh::setPolyArea` | `NavMesh.setPolyArea` | reviewed, benchmarked |
| `dtNavMesh::getPolyArea` | `NavMesh.getPolyArea` | reviewed, benchmarked |
| `dtNavMesh::getTileStateSize` | `NavMesh.getTileStateSize` | reviewed, oracle-tested |
| `dtNavMesh::storeTileState` | `NavMesh.storeTileState` | reviewed, benchmarked |
| `dtNavMesh::restoreTileState` | `NavMesh.restoreTileState` | reviewed, benchmarked |
| `dtNavMesh::encodePolyId` | `NavMesh.encodePolyId` | reviewed, oracle-tested |
| `dtNavMesh::decodePolyId` | `NavMesh.decodePolyId` | reviewed, oracle-tested |
| `dtNavMesh::decodePolyIdSalt` | `NavMesh.decodePolyIdSalt` | reviewed, oracle-tested |
| `dtNavMesh::decodePolyIdTile` | `NavMesh.decodePolyIdTile` | reviewed, oracle-tested |
| `dtNavMesh::decodePolyIdPoly` | `NavMesh.decodePolyIdPoly` | reviewed, oracle-tested |

### DetourNavMeshBuilder.h → `src/nav_mesh_builder.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtNavMeshCreateParams`: `verts`, `vertCount`, `polys`, `polyFlags`, `polyAreas`, `polyCount`, `nvp`, `detailMeshes`, `detailVerts`, `detailVertsCount`, `detailTris`, `detailTriCount`, `offMeshConVerts`, `offMeshConRad`, `offMeshConFlags`, `offMeshConAreas`, `offMeshConDir`, `offMeshConUserID`, `offMeshConCount`, `userId`, `tileX`, `tileY`, `tileLayer`, `bmin`, `bmax`, `walkableHeight`, `walkableRadius`, `walkableClimb`, `cs`, `ch`, `buildBvTree` | `NavMeshCreateParams` | reviewed, oracle-tested |
| `dtCreateNavMeshData` | `createNavMeshData` | reviewed, benchmarked |
| `dtNavMeshHeaderSwapEndian` | `navMeshHeaderSwapEndian` | reviewed, benchmarked |
| `dtNavMeshDataSwapEndian` | `navMeshDataSwapEndian` | reviewed, benchmarked |

### DetourNavMeshQuery.h → `src/nav_mesh_query.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtQueryFilter` | `QueryFilter` | reviewed, oracle-tested |
| `dtQueryFilter::dtQueryFilter` | `QueryFilter`, its default field values | reviewed, oracle-tested |
| `dtQueryFilter::passFilter` | `QueryFilter.passFilter` | reviewed, oracle-tested |
| `dtQueryFilter::getCost` | `QueryFilter.getCost` | reviewed, oracle-tested |
| `dtQueryFilter::getAreaCost` | `QueryFilter`, the `area_cost` field | reviewed, oracle-tested |
| `dtQueryFilter::setAreaCost` | `QueryFilter`, the `area_cost` field | reviewed, oracle-tested |
| `dtQueryFilter::getIncludeFlags` | `QueryFilter`, the `include_flags` field | reviewed, oracle-tested |
| `dtQueryFilter::setIncludeFlags` | `QueryFilter`, the `include_flags` field | reviewed, oracle-tested |
| `dtQueryFilter::getExcludeFlags` | `QueryFilter`, the `exclude_flags` field | reviewed, oracle-tested |
| `dtQueryFilter::setExcludeFlags` | `QueryFilter`, the `exclude_flags` field | reviewed, oracle-tested |
| `dtRaycastHit`: `t`, `hitNormal`, `hitEdgeIndex`, `path`, `pathCount`, `maxPath`, `pathCost` | `RaycastHit` | reviewed, oracle-tested |
| `dtPolyQuery` | `PolyQuery` | reviewed, oracle-tested |
| `dtPolyQuery::process` | `PolyQuery.process` | reviewed, oracle-tested |
| `dtNavMeshQuery` | `NavMeshQuery` | reviewed, oracle-tested |
| `dtNavMeshQuery::dtNavMeshQuery` | `NavMeshQuery.empty` | reviewed, oracle-tested |
| `dtNavMeshQuery::~dtNavMeshQuery` | `NavMeshQuery.deinit` | reviewed, oracle-tested |
| `dtNavMeshQuery::init` | `NavMeshQuery.init` | reviewed, oracle-tested |
| `dtNavMeshQuery::findPath` | `NavMeshQuery.findPath` | reviewed, benchmarked |
| `dtNavMeshQuery::findStraightPath` | `NavMeshQuery.findStraightPath` | reviewed, benchmarked |
| `dtNavMeshQuery::initSlicedFindPath` | `NavMeshQuery.initSlicedFindPath` | reviewed, benchmarked |
| `dtNavMeshQuery::updateSlicedFindPath` | `NavMeshQuery.updateSlicedFindPath` | reviewed, benchmarked |
| `dtNavMeshQuery::finalizeSlicedFindPath` | `NavMeshQuery.finalizeSlicedFindPath` | reviewed, benchmarked |
| `dtNavMeshQuery::finalizeSlicedFindPathPartial` | `NavMeshQuery.finalizeSlicedFindPathPartial` | reviewed, oracle-tested |
| `dtNavMeshQuery::findPolysAroundCircle` | `NavMeshQuery.findPolysAroundCircle` | reviewed, benchmarked |
| `dtNavMeshQuery::findPolysAroundShape` | `NavMeshQuery.findPolysAroundShape` | reviewed, benchmarked |
| `dtNavMeshQuery::getPathFromDijkstraSearch` | `NavMeshQuery.getPathFromDijkstraSearch` | reviewed, oracle-tested |
| `dtNavMeshQuery::findNearestPoly(center, halfExtents, filter, nearestRef, nearestPt)` | `NavMeshQuery.findNearestPoly`, `is_over_poly` null | reviewed, benchmarked |
| `dtNavMeshQuery::findNearestPoly(center, halfExtents, filter, nearestRef, nearestPt, isOverPoly)` | `NavMeshQuery.findNearestPoly` | reviewed, benchmarked |
| `dtNavMeshQuery::queryPolygons(center, halfExtents, filter, polys, polyCount, maxPolys)` | `NavMeshQuery.queryPolygons` | reviewed, benchmarked |
| `dtNavMeshQuery::queryPolygons(center, halfExtents, filter, query)` | `NavMeshQuery.queryPolygonsWith` | reviewed, benchmarked |
| `dtNavMeshQuery::findLocalNeighbourhood` | `NavMeshQuery.findLocalNeighbourhood` | reviewed, benchmarked |
| `dtNavMeshQuery::moveAlongSurface` | `NavMeshQuery.moveAlongSurface` | reviewed, benchmarked |
| `dtNavMeshQuery::raycast(startRef, startPos, endPos, filter, t, hitNormal, path, pathCount, maxPath)` | `NavMeshQuery.raycastPath` | reviewed, benchmarked |
| `dtNavMeshQuery::raycast(startRef, startPos, endPos, filter, options, hit, prevRef)` | `NavMeshQuery.raycast` | reviewed, benchmarked |
| `dtNavMeshQuery::findDistanceToWall` | `NavMeshQuery.findDistanceToWall` | reviewed, benchmarked |
| `dtNavMeshQuery::getPolyWallSegments` | `NavMeshQuery.getPolyWallSegments` | reviewed, benchmarked |
| `dtNavMeshQuery::findRandomPoint` | `NavMeshQuery.findRandomPoint` | reviewed, benchmarked |
| `dtNavMeshQuery::findRandomPointAroundCircle` | `NavMeshQuery.findRandomPointAroundCircle` | reviewed, benchmarked |
| `dtNavMeshQuery::closestPointOnPoly` | `NavMeshQuery.closestPointOnPoly` | reviewed, benchmarked |
| `dtNavMeshQuery::closestPointOnPolyBoundary` | `NavMeshQuery.closestPointOnPolyBoundary` | reviewed, benchmarked |
| `dtNavMeshQuery::getPolyHeight` | `NavMeshQuery.getPolyHeight` | reviewed, benchmarked |
| `dtNavMeshQuery::isValidPolyRef` | `NavMeshQuery.isValidPolyRef` | reviewed, benchmarked |
| `dtNavMeshQuery::isInClosedList` | `NavMeshQuery.isInClosedList` | reviewed, oracle-tested |
| `dtNavMeshQuery::getNodePool` | `NavMeshQuery.getNodePool` | reviewed, oracle-tested |
| `dtNavMeshQuery::getAttachedNavMesh` | `NavMeshQuery.getAttachedNavMesh` | reviewed, oracle-tested |

### DetourNode.h → `src/node.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtNodeFlags`: `DT_NODE_OPEN`, `DT_NODE_CLOSED`, `DT_NODE_PARENT_DETACHED` | `NodeFlags` | reviewed, oracle-tested |
| `dtNodeIndex` | `NodeIndex` | reviewed, oracle-tested |
| `DT_NULL_IDX` | `null_index` | reviewed, oracle-tested |
| `DT_NODE_PARENT_BITS` | `node_parent_bits` | reviewed, oracle-tested |
| `DT_NODE_STATE_BITS` | `node_state_bits` | reviewed, oracle-tested |
| `dtNode`: `pos`, `cost`, `total`, `DT_NODE_PARENT_BITS`, `DT_NODE_STATE_BITS`, `3`, `id` | `Node` | reviewed, oracle-tested |
| `DT_MAX_STATES_PER_NODE` | `node_states_max` | reviewed, oracle-tested |
| `dtNodePool` | `NodePool` | reviewed, oracle-tested |
| `dtNodePool::dtNodePool` | `NodePool.init` | reviewed, oracle-tested |
| `dtNodePool::~dtNodePool` | `NodePool.deinit` | reviewed, oracle-tested |
| `dtNodePool::clear` | `NodePool.clear` | reviewed, oracle-tested |
| `dtNodePool::getNode` | `NodePool.getNode` | reviewed, oracle-tested |
| `dtNodePool::findNode` | `NodePool.findNode` | reviewed, oracle-tested |
| `dtNodePool::findNodes` | `NodePool.findNodes` | reviewed, oracle-tested |
| `dtNodePool::getNodeIdx` | `NodePool.getNodeIdx` | reviewed, oracle-tested |
| `dtNodePool::getNodeAtIdx` | `NodePool.getNodeAtIdx` | reviewed, oracle-tested |
| `dtNodePool::getMemUsed` | `NodePool.getMemUsed` | reviewed, oracle-tested |
| `dtNodePool::getMaxNodes` | `NodePool.getMaxNodes` | reviewed, oracle-tested |
| `dtNodePool::getHashSize` | `NodePool.getHashSize` | reviewed, oracle-tested |
| `dtNodePool::getFirst` | `NodePool.getFirst` | reviewed, oracle-tested |
| `dtNodePool::getNext` | `NodePool.getNext` | reviewed, oracle-tested |
| `dtNodePool::getNodeCount` | `NodePool.getNodeCount` | reviewed, oracle-tested |
| `dtNodeQueue` | `NodeQueue` | reviewed, oracle-tested |
| `dtNodeQueue::dtNodeQueue` | `NodeQueue.init` | reviewed, oracle-tested |
| `dtNodeQueue::~dtNodeQueue` | `NodeQueue.deinit` | reviewed, oracle-tested |
| `dtNodeQueue::clear` | `NodeQueue.clear` | reviewed, oracle-tested |
| `dtNodeQueue::top` | `NodeQueue.top` | reviewed, oracle-tested |
| `dtNodeQueue::pop` | `NodeQueue.pop` | reviewed, oracle-tested |
| `dtNodeQueue::push` | `NodeQueue.push` | reviewed, oracle-tested |
| `dtNodeQueue::modify` | `NodeQueue.modify` | reviewed, oracle-tested |
| `dtNodeQueue::empty` | `NodeQueue.isEmpty` | reviewed, oracle-tested |
| `dtNodeQueue::getMemUsed` | `NodeQueue.getMemUsed` | reviewed, oracle-tested |
| `dtNodeQueue::getCapacity` | `NodeQueue.getCapacity` | reviewed, oracle-tested |

### DetourCommon.h → `src/common.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtMin` | `min` | reviewed, oracle-tested |
| `dtMax` | `max` | reviewed, oracle-tested |
| `dtAbs` | `abs` | reviewed, oracle-tested |
| `dtSqr` | `sqr` | reviewed, oracle-tested |
| `dtClamp` | `clamp` | reviewed, oracle-tested |
| `dtVcross` | `vcross` | reviewed, oracle-tested |
| `dtVdot` | `vdot` | reviewed, oracle-tested |
| `dtVmad` | `vmad` | reviewed, oracle-tested |
| `dtVlerp` | `vlerp` | reviewed, oracle-tested |
| `dtVadd` | `vadd` | reviewed, oracle-tested |
| `dtVsub` | `vsub` | reviewed, oracle-tested |
| `dtVscale` | `vscale` | reviewed, oracle-tested |
| `dtVmin` | `vmin` | reviewed, oracle-tested |
| `dtVmax` | `vmax` | reviewed, oracle-tested |
| `dtVlen` | `vlen` | reviewed, oracle-tested |
| `dtVlenSqr` | `vlenSqr` | reviewed, oracle-tested |
| `dtVdist` | `vdist` | reviewed, oracle-tested |
| `dtVdistSqr` | `vdistSqr` | reviewed, oracle-tested |
| `dtVdist2D` | `vdist2d` | reviewed, oracle-tested |
| `dtVdist2DSqr` | `vdist2dSqr` | reviewed, oracle-tested |
| `dtVnormalize` | `vnormalize` | reviewed, oracle-tested |
| `dtVequal` | `vequal` | reviewed, oracle-tested |
| `dtVisfinite` | `visfinite` | reviewed, oracle-tested |
| `dtVisfinite2D` | `visfinite2d` | reviewed, oracle-tested |
| `dtVdot2D` | `vdot2d` | reviewed, oracle-tested |
| `dtVperp2D` | `vperp2d` | reviewed, oracle-tested |
| `dtTriArea2D` | `triArea2d` | reviewed, oracle-tested |
| `dtOverlapQuantBounds` | `overlapQuantBounds` | reviewed, oracle-tested |
| `dtOverlapBounds` | `overlapBounds` | reviewed, oracle-tested |
| `dtClosestPtPointTriangle` | `closestPointOnTriangle` | reviewed, benchmarked |
| `dtClosestHeightPointTriangle` | `closestHeightOnTriangle` | reviewed, benchmarked |
| `dtIntersectSegmentPoly2D` | `intersectSegmentPoly2d` | reviewed, benchmarked |
| `dtIntersectSegSeg2D` | `intersectSegmentSegment2d` | reviewed, benchmarked |
| `dtPointInPolygon` | `pointInPolygon` | reviewed, benchmarked |
| `dtDistancePtPolyEdgesSqr` | `distancePointPolyEdgesSqr` | reviewed, benchmarked |
| `dtDistancePtSegSqr2D` | `distancePointSegmentSqr2d` | reviewed, benchmarked |
| `dtCalcPolyCenter` | `calcPolyCenter` | reviewed, benchmarked |
| `dtOverlapPolyPoly2D` | `overlapPolyPoly2d` | reviewed, benchmarked |
| `dtNextPow2` | `nextPow2` | reviewed, oracle-tested |
| `dtIlog2` | `ilog2` | reviewed, oracle-tested |
| `dtAlign4` | `align4` | reviewed, oracle-tested |
| `dtOppositeTile` | `oppositeTile` | reviewed, oracle-tested |
| `dtSwapEndian` for `float` | `swapEndianFloat` | reviewed, oracle-tested |
| `dtRandomPointInConvexPoly` | `randomPointInConvexPoly` | reviewed, benchmarked |

### DetourStatus.h → `src/status.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtStatus` | `Status` | reviewed, oracle-tested |
| `DT_FAILURE` | `Status.failure`, and the `failed` field | reviewed, oracle-tested |
| `DT_SUCCESS` | `Status.success`, and the `succeeded` field | reviewed, oracle-tested |
| `DT_IN_PROGRESS` | `Status`, the `in_progress` field | reviewed, oracle-tested |
| `DT_STATUS_DETAIL_MASK` | `Status.detail_mask` | reviewed, oracle-tested |
| `DT_WRONG_MAGIC` | `Status`, the `wrong_magic` field | reviewed, oracle-tested |
| `DT_WRONG_VERSION` | `Status`, the `wrong_version` field | reviewed, oracle-tested |
| `DT_OUT_OF_MEMORY` | `Status`, the `out_of_memory` field | reviewed, oracle-tested |
| `DT_INVALID_PARAM` | `Status`, the `invalid_param` field | reviewed, oracle-tested |
| `DT_BUFFER_TOO_SMALL` | `Status`, the `buffer_too_small` field | reviewed, oracle-tested |
| `DT_OUT_OF_NODES` | `Status`, the `out_of_nodes` field | reviewed, oracle-tested |
| `DT_PARTIAL_RESULT` | `Status`, the `partial_result` field | reviewed, oracle-tested |
| `DT_ALREADY_OCCUPIED` | `Status`, the `already_occupied` field | reviewed, oracle-tested |
| `dtStatusSucceed` | `Status`, the `succeeded` field | reviewed, oracle-tested |
| `dtStatusFailed` | `Status`, the `failed` field | reviewed, oracle-tested |
| `dtStatusInProgress` | `Status`, the `in_progress` field | reviewed, oracle-tested |
| `dtStatusDetail` | `Status.has` | reviewed, oracle-tested |

### DetourMath.h → `src/common.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtMathIsfinite` | `isFinite` | reviewed, oracle-tested |

## DetourCrowd

### DetourCrowd.h → `src/crowd.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `DT_CROWDAGENT_MAX_NEIGHBOURS` | `agent_neighbour_count_max` | reviewed, oracle-tested |
| `DT_CROWDAGENT_MAX_CORNERS` | `agent_corner_count_max` | reviewed, oracle-tested |
| `DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS` | `obstacle_avoidance_params_count_max` | reviewed, oracle-tested |
| `DT_CROWD_MAX_QUERY_FILTER_TYPE` | `query_filter_type_count_max` | reviewed, oracle-tested |
| `dtCrowdNeighbour`: `idx`, `dist` | `CrowdNeighbour` | reviewed, oracle-tested |
| `CrowdAgentState`: `DT_CROWDAGENT_STATE_INVALID`, `DT_CROWDAGENT_STATE_WALKING`, `DT_CROWDAGENT_STATE_OFFMESH` | `CrowdAgentTraversal` | reviewed, oracle-tested |
| `dtCrowdAgentParams`: `radius`, `height`, `maxAcceleration`, `maxSpeed`, `collisionQueryRange`, `pathOptimizationRange`, `separationWeight`, `updateFlags`, `obstacleAvoidanceType`, `queryFilterType`, `userData` | `CrowdAgentParams` | reviewed, oracle-tested |
| `MoveRequestState`: `DT_CROWDAGENT_TARGET_NONE`, `DT_CROWDAGENT_TARGET_FAILED`, `DT_CROWDAGENT_TARGET_VALID`, `DT_CROWDAGENT_TARGET_REQUESTING`, `DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE`, `DT_CROWDAGENT_TARGET_WAITING_FOR_PATH`, `DT_CROWDAGENT_TARGET_VELOCITY` | `MoveRequestStage` | reviewed, oracle-tested |
| `dtCrowdAgent`: `active`, `state`, `partial`, `corridor`, `boundary`, `topologyOptTime`, `neis`, `nneis`, `desiredSpeed`, `npos`, `disp`, `dvel`, `nvel`, `vel`, `params`, `cornerVerts`, `cornerFlags`, `cornerPolys`, `ncorners`, `targetState`, `targetRef`, `targetPos`, `targetPathqRef`, `targetReplan`, `targetReplanTime` | `CrowdAgent` | reviewed, oracle-tested |
| `dtCrowdAgentAnimation`: `active`, `initPos`, `startPos`, `endPos`, `polyRef`, `t`, `tmax` | `CrowdAgentAnimation` | reviewed, oracle-tested |
| `UpdateFlags`: `DT_CROWD_ANTICIPATE_TURNS`, `DT_CROWD_OBSTACLE_AVOIDANCE`, `DT_CROWD_SEPARATION`, `DT_CROWD_OPTIMIZE_VIS`, `DT_CROWD_OPTIMIZE_TOPO` | `UpdateFlags` | reviewed, oracle-tested |
| `dtCrowdAgentDebugInfo`: `idx`, `optStart`, `optEnd`, `vod` | `CrowdAgentDebugInfo` | reviewed, oracle-tested |
| `dtCrowd` | `Crowd` | reviewed, oracle-tested |
| `dtCrowd::dtCrowd` | `Crowd.empty` | reviewed, oracle-tested |
| `dtCrowd::~dtCrowd` | `Crowd.deinit` | reviewed, oracle-tested |
| `dtCrowd::init` | `Crowd.init` | reviewed, oracle-tested |
| `dtCrowd::setObstacleAvoidanceParams` | `Crowd.setObstacleAvoidanceParams` | reviewed, oracle-tested |
| `dtCrowd::getObstacleAvoidanceParams` | `Crowd.getObstacleAvoidanceParams` | reviewed, oracle-tested |
| `dtCrowd::getAgent` | `Crowd.getAgent` | reviewed, oracle-tested |
| `dtCrowd::getEditableAgent` | `Crowd.getEditableAgent` | reviewed, oracle-tested |
| `dtCrowd::getAgentCount` | `Crowd.getAgentCount` | reviewed, oracle-tested |
| `dtCrowd::addAgent` | `Crowd.addAgent` | reviewed, benchmarked |
| `dtCrowd::updateAgentParameters` | `Crowd.updateAgentParameters` | reviewed, oracle-tested |
| `dtCrowd::removeAgent` | `Crowd.removeAgent` | reviewed, benchmarked |
| `dtCrowd::requestMoveTarget` | `Crowd.requestMoveTarget` | reviewed, benchmarked |
| `dtCrowd::requestMoveVelocity` | `Crowd.requestMoveVelocity` | reviewed, oracle-tested |
| `dtCrowd::resetMoveTarget` | `Crowd.resetMoveTarget` | reviewed, oracle-tested |
| `dtCrowd::getActiveAgents` | `Crowd.getActiveAgents` | reviewed, oracle-tested |
| `dtCrowd::update` | `Crowd.update` | reviewed, benchmarked |
| `dtCrowd::getFilter` | `Crowd.getFilter` | reviewed, oracle-tested |
| `dtCrowd::getEditableFilter` | `Crowd.getEditableFilter` | reviewed, oracle-tested |
| `dtCrowd::getQueryHalfExtents` | `Crowd.getQueryHalfExtents` | reviewed, oracle-tested |
| `dtCrowd::getQueryExtents` | `Crowd.getQueryExtents` | reviewed, oracle-tested |
| `dtCrowd::getVelocitySampleCount` | `Crowd.getVelocitySampleCount` | reviewed, oracle-tested |
| `dtCrowd::getGrid` | `Crowd.getGrid` | reviewed, oracle-tested |
| `dtCrowd::getPathQueue` | `Crowd.getPathQueue` | reviewed, oracle-tested |
| `dtCrowd::getNavMeshQuery` | `Crowd.getNavMeshQuery` | reviewed, oracle-tested |

### DetourPathCorridor.h → `src/path_corridor.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtPathCorridor` | `PathCorridor` | reviewed, oracle-tested |
| `dtPathCorridor::dtPathCorridor` | `PathCorridor.empty` | reviewed, oracle-tested |
| `dtPathCorridor::~dtPathCorridor` | `PathCorridor.deinit` | reviewed, oracle-tested |
| `dtPathCorridor::init` | `PathCorridor.init` | reviewed, oracle-tested |
| `dtPathCorridor::reset` | `PathCorridor.reset` | reviewed, oracle-tested |
| `dtPathCorridor::findCorners` | `PathCorridor.findCorners` | reviewed, benchmarked |
| `dtPathCorridor::optimizePathVisibility` | `PathCorridor.optimizePathVisibility` | reviewed, benchmarked |
| `dtPathCorridor::optimizePathTopology` | `PathCorridor.optimizePathTopology` | reviewed, benchmarked |
| `dtPathCorridor::moveOverOffmeshConnection` | `PathCorridor.moveOverOffmeshConnection` | reviewed, oracle-tested |
| `dtPathCorridor::fixPathStart` | `PathCorridor.fixPathStart` | reviewed, oracle-tested |
| `dtPathCorridor::trimInvalidPath` | `PathCorridor.trimInvalidPath` | reviewed, oracle-tested |
| `dtPathCorridor::isValid` | `PathCorridor.isValid` | reviewed, oracle-tested |
| `dtPathCorridor::movePosition` | `PathCorridor.movePosition` | reviewed, benchmarked |
| `dtPathCorridor::moveTargetPosition` | `PathCorridor.moveTargetPosition` | reviewed, oracle-tested |
| `dtPathCorridor::setCorridor` | `PathCorridor.setCorridor` | reviewed, benchmarked |
| `dtPathCorridor::getPos` | `PathCorridor.getPos` | reviewed, oracle-tested |
| `dtPathCorridor::getTarget` | `PathCorridor.getTarget` | reviewed, oracle-tested |
| `dtPathCorridor::getFirstPoly` | `PathCorridor.getFirstPoly` | reviewed, oracle-tested |
| `dtPathCorridor::getLastPoly` | `PathCorridor.getLastPoly` | reviewed, oracle-tested |
| `dtPathCorridor::getPath` | `PathCorridor.getPath` | reviewed, oracle-tested |
| `dtPathCorridor::getPathCount` | `PathCorridor.getPathCount` | reviewed, oracle-tested |
| `dtMergeCorridorStartMoved` | `mergeCorridorStartMoved` | reviewed, oracle-tested |
| `dtMergeCorridorEndMoved` | `mergeCorridorEndMoved` | reviewed, oracle-tested |
| `dtMergeCorridorStartShortcut` | `mergeCorridorStartShortcut` | reviewed, oracle-tested |

### DetourPathQueue.h → `src/path_queue.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `DT_PATHQ_INVALID` | `path_queue_invalid` | reviewed, oracle-tested |
| `dtPathQueueRef` | `PathQueueRef` | reviewed, oracle-tested |
| `dtPathQueue` | `PathQueue` | reviewed, oracle-tested |
| `dtPathQueue::dtPathQueue` | `PathQueue.empty` | reviewed, oracle-tested |
| `dtPathQueue::~dtPathQueue` | `PathQueue.deinit` | reviewed, oracle-tested |
| `dtPathQueue::init` | `PathQueue.init` | reviewed, oracle-tested |
| `dtPathQueue::update` | `PathQueue.update` | reviewed, benchmarked |
| `dtPathQueue::request` | `PathQueue.request` | reviewed, benchmarked |
| `dtPathQueue::getRequestStatus` | `PathQueue.getRequestStatus` | reviewed, benchmarked |
| `dtPathQueue::getPathResult` | `PathQueue.getPathResult` | reviewed, benchmarked |
| `dtPathQueue::getNavQuery` | `PathQueue.getNavQuery` | reviewed, oracle-tested |

### DetourLocalBoundary.h → `src/local_boundary.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtLocalBoundary` | `LocalBoundary` | reviewed, oracle-tested |
| `dtLocalBoundary::dtLocalBoundary` | `LocalBoundary.empty` | reviewed, oracle-tested |
| `dtLocalBoundary::reset` | `LocalBoundary.reset` | reviewed, oracle-tested |
| `dtLocalBoundary::update` | `LocalBoundary.update` | reviewed, benchmarked |
| `dtLocalBoundary::isValid` | `LocalBoundary.isValid` | reviewed, oracle-tested |
| `dtLocalBoundary::getCenter` | `LocalBoundary.getCenter` | reviewed, oracle-tested |
| `dtLocalBoundary::getSegmentCount` | `LocalBoundary.getSegmentCount` | reviewed, oracle-tested |
| `dtLocalBoundary::getSegment` | `LocalBoundary.getSegment` | reviewed, oracle-tested |

### DetourObstacleAvoidance.h → `src/obstacle_avoidance.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtObstacleCircle`: `p`, `vel`, `dvel`, `rad`, `dp`, `np` | `ObstacleCircle` | reviewed, oracle-tested |
| `dtObstacleSegment`: `p`, `q`, `touch` | `ObstacleSegment` | reviewed, oracle-tested |
| `dtObstacleAvoidanceDebugData` | `ObstacleAvoidanceDebug` | reviewed, oracle-tested |
| `dtObstacleAvoidanceDebugData::dtObstacleAvoidanceDebugData` | `ObstacleAvoidanceDebug.empty` | reviewed, oracle-tested |
| `dtObstacleAvoidanceDebugData::~dtObstacleAvoidanceDebugData` | `ObstacleAvoidanceDebug.deinit` | reviewed, oracle-tested |
| `dtObstacleAvoidanceDebugData::init` | `ObstacleAvoidanceDebug.init` | reviewed, oracle-tested |
| `dtObstacleAvoidanceDebugData::reset` | `ObstacleAvoidanceDebug.reset` | reviewed, oracle-tested |
| `dtObstacleAvoidanceDebugData::addSample` | `ObstacleAvoidanceDebug.addSample` | reviewed, oracle-tested |
| `dtObstacleAvoidanceDebugData::normalizeSamples` | `ObstacleAvoidanceDebug.normalizeSamples` | reviewed, oracle-tested |
| `dtObstacleAvoidanceDebugData::getSampleCount` | `ObstacleAvoidanceDebug.getSampleCount` | reviewed, oracle-tested |
| `dtObstacleAvoidanceDebugData::getSampleVelocity` | `ObstacleAvoidanceDebug.getSampleVelocity` | reviewed, oracle-tested |
| `dtObstacleAvoidanceDebugData::getSampleSize` | `ObstacleAvoidanceDebug.getSampleSize` | reviewed, oracle-tested |
| `dtObstacleAvoidanceDebugData::getSamplePenalty` | `ObstacleAvoidanceDebug.getSamplePenalty` | reviewed, oracle-tested |
| `dtObstacleAvoidanceDebugData::getSampleDesiredVelocityPenalty` | `ObstacleAvoidanceDebug.getSampleDesiredVelocityPenalty` | reviewed, oracle-tested |
| `dtObstacleAvoidanceDebugData::getSampleCurrentVelocityPenalty` | `ObstacleAvoidanceDebug.getSampleCurrentVelocityPenalty` | reviewed, oracle-tested |
| `dtObstacleAvoidanceDebugData::getSamplePreferredSidePenalty` | `ObstacleAvoidanceDebug.getSamplePreferredSidePenalty` | reviewed, oracle-tested |
| `dtObstacleAvoidanceDebugData::getSampleCollisionTimePenalty` | `ObstacleAvoidanceDebug.getSampleCollisionTimePenalty` | reviewed, oracle-tested |
| `DT_MAX_PATTERN_DIVS` | `pattern_division_count_max` | reviewed, oracle-tested |
| `DT_MAX_PATTERN_RINGS` | `pattern_ring_count_max` | reviewed, oracle-tested |
| `dtObstacleAvoidanceParams`: `velBias`, `weightDesVel`, `weightCurVel`, `weightSide`, `weightToi`, `horizTime`, `gridSize`, `adaptiveDivs`, `adaptiveRings`, `adaptiveDepth` | `ObstacleAvoidanceParams` | reviewed, oracle-tested |
| `dtObstacleAvoidanceQuery` | `ObstacleAvoidanceQuery` | reviewed, oracle-tested |
| `dtObstacleAvoidanceQuery::dtObstacleAvoidanceQuery` | `ObstacleAvoidanceQuery.empty` | reviewed, oracle-tested |
| `dtObstacleAvoidanceQuery::~dtObstacleAvoidanceQuery` | `ObstacleAvoidanceQuery.deinit` | reviewed, oracle-tested |
| `dtObstacleAvoidanceQuery::init` | `ObstacleAvoidanceQuery.init` | reviewed, oracle-tested |
| `dtObstacleAvoidanceQuery::reset` | `ObstacleAvoidanceQuery.reset` | reviewed, benchmarked |
| `dtObstacleAvoidanceQuery::addCircle` | `ObstacleAvoidanceQuery.addCircle` | reviewed, benchmarked |
| `dtObstacleAvoidanceQuery::addSegment` | `ObstacleAvoidanceQuery.addSegment` | reviewed, benchmarked |
| `dtObstacleAvoidanceQuery::sampleVelocityGrid` | `ObstacleAvoidanceQuery.sampleVelocityGrid` | reviewed, benchmarked |
| `dtObstacleAvoidanceQuery::sampleVelocityAdaptive` | `ObstacleAvoidanceQuery.sampleVelocityAdaptive` | reviewed, benchmarked |
| `dtObstacleAvoidanceQuery::getObstacleCircleCount` | `ObstacleAvoidanceQuery.getObstacleCircleCount` | reviewed, oracle-tested |
| `dtObstacleAvoidanceQuery::getObstacleCircle` | `ObstacleAvoidanceQuery.getObstacleCircle` | reviewed, oracle-tested |
| `dtObstacleAvoidanceQuery::getObstacleSegmentCount` | `ObstacleAvoidanceQuery.getObstacleSegmentCount` | reviewed, oracle-tested |
| `dtObstacleAvoidanceQuery::getObstacleSegment` | `ObstacleAvoidanceQuery.getObstacleSegment` | reviewed, oracle-tested |

### DetourProximityGrid.h → `src/proximity_grid.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtProximityGrid` | `ProximityGrid` | reviewed, oracle-tested |
| `dtProximityGrid::dtProximityGrid` | `ProximityGrid.empty` | reviewed, oracle-tested |
| `dtProximityGrid::~dtProximityGrid` | `ProximityGrid.deinit` | reviewed, oracle-tested |
| `dtProximityGrid::init` | `ProximityGrid.init` | reviewed, oracle-tested |
| `dtProximityGrid::clear` | `ProximityGrid.clear` | reviewed, benchmarked |
| `dtProximityGrid::addItem` | `ProximityGrid.addItem` | reviewed, benchmarked |
| `dtProximityGrid::queryItems` | `ProximityGrid.queryItems` | reviewed, benchmarked |
| `dtProximityGrid::getItemCountAt` | `ProximityGrid.getItemCountAt` | reviewed, oracle-tested |
| `dtProximityGrid::getBounds` | `ProximityGrid.getBounds` | reviewed, oracle-tested |
| `dtProximityGrid::getCellSize` | `ProximityGrid.getCellSize` | reviewed, oracle-tested |

## DetourTileCache

### DetourTileCache.h → `src/tile_cache.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtObstacleRef` | `ObstacleRef` | reviewed, oracle-tested |
| `dtCompressedTileRef` | `CompressedTileRef` | reviewed, oracle-tested |
| `dtCompressedTileFlags`: `DT_COMPRESSEDTILE_FREE_DATA` | `compressed_tile_free_data` | reviewed, oracle-tested |
| `dtCompressedTile`: `salt`, `header`, `compressed`, `compressedSize`, `data`, `dataSize`, `flags`, `next` | `CompressedTile` | reviewed, oracle-tested |
| `ObstacleState`: `DT_OBSTACLE_EMPTY`, `DT_OBSTACLE_PROCESSING`, `DT_OBSTACLE_PROCESSED`, `DT_OBSTACLE_REMOVING` | `ObstaclePhase` | reviewed, oracle-tested |
| `ObstacleType`: `DT_OBSTACLE_CYLINDER`, `DT_OBSTACLE_BOX`, `DT_OBSTACLE_ORIENTED_BOX` | `ObstacleType` | reviewed, oracle-tested |
| `dtObstacleCylinder`: `pos`, `radius`, `height` | `ObstacleCylinder` | reviewed, oracle-tested |
| `dtObstacleBox`: `bmin`, `bmax` | `ObstacleBox` | reviewed, oracle-tested |
| `dtObstacleOrientedBox`: `center`, `halfExtents`, `rotAux` | `ObstacleOrientedBox` | reviewed, oracle-tested |
| `DT_MAX_TOUCHED_TILES` | `touched_tile_count_max` | reviewed, oracle-tested |
| `dtTileCacheObstacle`: `touched`, `pending`, `salt`, `type`, `state`, `ntouched`, `npending`, `next` | `Obstacle` | reviewed, oracle-tested |
| `dtTileCacheParams`: `orig`, `cs`, `ch`, `width`, `height`, `walkableHeight`, `walkableRadius`, `walkableClimb`, `maxSimplificationError`, `maxTiles`, `maxObstacles` | `TileCacheParams` | reviewed, oracle-tested |
| `dtTileCacheMeshProcess` | `MeshProcess` | reviewed, oracle-tested |
| `dtTileCacheMeshProcess::process` | `MeshProcess.process` | reviewed, oracle-tested |
| `dtTileCache` | `TileCache` | reviewed, oracle-tested |
| `dtTileCache::dtTileCache` | `TileCache.empty` | reviewed, oracle-tested |
| `dtTileCache::~dtTileCache` | `TileCache.deinit` | reviewed, oracle-tested |
| `dtTileCache::getAlloc` | `TileCache.getAlloc` | reviewed, oracle-tested |
| `dtTileCache::getCompressor` | `TileCache.getCompressor` | reviewed, oracle-tested |
| `dtTileCache::getParams` | `TileCache.getParams` | reviewed, oracle-tested |
| `dtTileCache::getTileCount` | `TileCache.getTileCount` | reviewed, oracle-tested |
| `dtTileCache::getTile` | `TileCache.getTile` | reviewed, oracle-tested |
| `dtTileCache::getObstacleCount` | `TileCache.getObstacleCount` | reviewed, oracle-tested |
| `dtTileCache::getObstacle` | `TileCache.getObstacle` | reviewed, oracle-tested |
| `dtTileCache::getObstacleByRef` | `TileCache.getObstacleByRef` | reviewed, oracle-tested |
| `dtTileCache::getObstacleRef` | `TileCache.getObstacleRef` | reviewed, oracle-tested |
| `dtTileCache::init` | `TileCache.init` | reviewed, oracle-tested |
| `dtTileCache::getTilesAt` | `TileCache.getTilesAt` | reviewed, oracle-tested |
| `dtTileCache::getTileAt` | `TileCache.getTileAt` | reviewed, oracle-tested |
| `dtTileCache::getTileRef` | `TileCache.getTileRef` | reviewed, oracle-tested |
| `dtTileCache::getTileByRef` | `TileCache.getTileByRef` | reviewed, oracle-tested |
| `dtTileCache::addTile` | `TileCache.addTile` | reviewed, benchmarked |
| `dtTileCache::removeTile` | `TileCache.removeTile` | reviewed, benchmarked |
| `dtTileCache::addObstacle` | `TileCache.addObstacle` | reviewed, benchmarked |
| `dtTileCache::addBoxObstacle(bmin, bmax, result)` | `TileCache.addBoxObstacle` | reviewed, oracle-tested |
| `dtTileCache::addBoxObstacle(center, halfExtents, yRadians, result)` | `TileCache.addOrientedBoxObstacle` | reviewed, oracle-tested |
| `dtTileCache::removeObstacle` | `TileCache.removeObstacle` | reviewed, benchmarked |
| `dtTileCache::queryTiles` | `TileCache.queryTiles` | reviewed, benchmarked |
| `dtTileCache::update` | `TileCache.update` | reviewed, benchmarked |
| `dtTileCache::buildNavMeshTilesAt` | `TileCache.buildNavMeshTilesAt` | reviewed, oracle-tested |
| `dtTileCache::buildNavMeshTile` | `TileCache.buildNavMeshTile` | reviewed, benchmarked |
| `dtTileCache::calcTightTileBounds` | `TileCache.calcTightTileBounds` | reviewed, oracle-tested |
| `dtTileCache::getObstacleBounds` | `TileCache.getObstacleBounds` | reviewed, oracle-tested |
| `dtTileCache::encodeTileId` | `TileCache.encodeTileId` | reviewed, oracle-tested |
| `dtTileCache::decodeTileIdSalt` | `TileCache.decodeTileIdSalt` | reviewed, oracle-tested |
| `dtTileCache::decodeTileIdTile` | `TileCache.decodeTileIdTile` | reviewed, oracle-tested |
| `dtTileCache::encodeObstacleId` | `TileCache.encodeObstacleId` | reviewed, oracle-tested |
| `dtTileCache::decodeObstacleIdSalt` | `TileCache.decodeObstacleIdSalt` | reviewed, oracle-tested |
| `dtTileCache::decodeObstacleIdObstacle` | `TileCache.decodeObstacleIdObstacle` | reviewed, oracle-tested |

### DetourTileCacheBuilder.h → `src/tile_cache_builder.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `DT_TILECACHE_MAGIC` | `tile_cache_magic` | reviewed, oracle-tested |
| `DT_TILECACHE_VERSION` | `tile_cache_version` | reviewed, oracle-tested |
| `DT_TILECACHE_NULL_AREA` | `null_area` | reviewed, oracle-tested |
| `DT_TILECACHE_WALKABLE_AREA` | `walkable_area` | reviewed, oracle-tested |
| `DT_TILECACHE_NULL_IDX` | `null_index` | reviewed, oracle-tested |
| `dtTileCacheLayerHeader`: `magic`, `version`, `tx`, `ty`, `tlayer`, `bmin`, `bmax`, `hmin`, `hmax`, `width`, `height`, `minx`, `maxx`, `miny`, `maxy` | `LayerHeader` | reviewed, oracle-tested |
| `dtTileCacheLayer`: `header`, `regCount`, `heights`, `areas`, `cons`, `regs` | `Layer` | reviewed, oracle-tested |
| `dtTileCacheContour`: `nverts`, `verts`, `reg`, `area` | `Contour` | reviewed, oracle-tested |
| `dtTileCacheContourSet`: `nconts`, `conts` | `ContourSet` | reviewed, oracle-tested |
| `dtTileCachePolyMesh`: `nvp`, `nverts`, `npolys`, `verts`, `polys`, `flags`, `areas` | `PolyMesh` | reviewed, oracle-tested |
| `dtTileCacheAlloc` | `BuildAllocator` | reviewed, oracle-tested |
| `dtTileCacheAlloc::reset` | `BuildAllocator.reset` | reviewed, oracle-tested |
| `dtTileCacheCompressor` | `Compressor` | reviewed, oracle-tested |
| `dtTileCacheCompressor::maxCompressedSize` | `Compressor.maxCompressedSize` | reviewed, oracle-tested |
| `dtTileCacheCompressor::compress` | `Compressor.compress` | reviewed, oracle-tested |
| `dtTileCacheCompressor::decompress` | `Compressor.decompress` | reviewed, oracle-tested |
| `dtBuildTileCacheLayer` | `buildLayer` | reviewed, oracle-tested |
| `dtFreeTileCacheLayer` | `freeLayer` | reviewed, oracle-tested |
| `dtDecompressTileCacheLayer` | `decompressLayer` | reviewed, oracle-tested |
| `dtAllocTileCacheContourSet` | `allocContourSet` | reviewed, oracle-tested |
| `dtFreeTileCacheContourSet` | `freeContourSet` | reviewed, oracle-tested |
| `dtAllocTileCachePolyMesh` | `allocPolyMesh` | reviewed, oracle-tested |
| `dtFreeTileCachePolyMesh` | `freePolyMesh` | reviewed, oracle-tested |
| `dtMarkCylinderArea` | `markCylinderArea` | reviewed, oracle-tested |
| `dtMarkBoxArea(layer, orig, cs, ch, bmin, bmax, areaId)` | `markBoxArea` | reviewed, oracle-tested |
| `dtMarkBoxArea(layer, orig, cs, ch, center, halfExtents, rotAux, areaId)` | `markOrientedBoxArea` | reviewed, oracle-tested |
| `dtBuildTileCacheRegions` | `buildRegions` | reviewed, oracle-tested |
| `dtBuildTileCacheContours` | `buildContours` | reviewed, oracle-tested |
| `dtBuildTileCachePolyMesh` | `buildPolyMesh` | reviewed, oracle-tested |
| `dtTileCacheHeaderSwapEndian` | `swapEndianHeader` | reviewed, oracle-tested |

## The integer variant

Not in upstream: the same queries over whole-millimetre coordinates, for a server that
must give the same answer on every machine. `doc/integer-design.md` gives its units,
scales, overflow bounds and tolerances. Here **built** means it compiles and passes its
own unit tests, and **checked** means the tests compare it with the float port within the
design's tolerances.

| piece | Zig | state |
| --- | --- | --- |
| design: units, scales, overflow bounds, tolerances | `doc/integer-design.md` | reviewed, written |
| vector and geometry helpers | `src/integer/common.zig` | reviewed, built, checked through the queries |
| navigation mesh | `src/integer/nav_mesh.zig` | reviewed, built, checked |
| tile conversion from the float format | `src/integer/convert.zig` | reviewed, built, checked |
| node pool and queue | `src/integer/node.zig` | reviewed, built, checked through the searches |
| queries, every one of `NavMeshQuery` | `src/integer/nav_mesh_query.zig` | reviewed, built, checked |
| checked against the float port within the stated tolerances | `test/integer_test.zig`, `test/integer_hand_test.zig` | reviewed, done: 4 levels, about 1,200 seeded cases per query, hand-made meshes |
| no overflow across the whole coordinate range | `test/integer_hand_test.zig` | reviewed, done |
| no float in the variant's source | `test/integer_hand_test.zig` | reviewed, done |
| exact self-determinism: twice, on 1, 2 and N threads, a hash kept in the test | `test/integer_hand_test.zig` | reviewed, done, the same hash in Debug, ReleaseSafe and ReleaseFast |
| crowd and tile cache over integers | | left out, see `doc/integer-design.md` |

## Next

Version 0.1.0 is released: the performance pass, the skeptic's review and their fixes are
in, every test passes on fuzz seeds 0 to 31 in Debug and ReleaseSafe, and
`doc/BENCHMARKS.md` holds the benchmark. Left open, as `doc/BENCHMARKS.md` lists: the
causes of the rows where the port is slower than the original.
