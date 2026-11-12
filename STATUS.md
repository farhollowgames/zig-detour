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

A struct's row names its fields, and an enum's row its values: those are the options and
modes. `doc/parity.md` lists every place where the port's behaviour differs from the
original, and why.

## Summary

| library | upstream header | Zig file | entries | not started | ported | oracle-tested | benchmarked | reviewed |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Detour | DetourNavMesh.h | `src/nav_mesh.zig` | 69 |  |  | 52 | 17 |  |
| Detour | DetourNavMeshBuilder.h | `src/nav_mesh_builder.zig` | 4 |  |  | 1 | 3 |  |
| Detour | DetourNavMeshQuery.h | `src/nav_mesh_query.zig` | 45 |  |  | 22 | 23 |  |
| Detour | DetourNode.h | `src/node.zig` | 33 |  |  | 33 |  |  |
| Detour | DetourCommon.h | `src/common.zig` | 44 |  |  | 34 | 10 |  |
| Detour | DetourStatus.h | `src/status.zig` | 17 |  |  | 17 |  |  |
| Detour | DetourMath.h | `src/common.zig` | 1 |  |  | 1 |  |  |
| DetourCrowd | DetourCrowd.h | `src/crowd.zig` | 37 |  |  | 33 | 4 |  |
| DetourCrowd | DetourPathCorridor.h | `src/path_corridor.zig` | 24 |  |  | 19 | 5 |  |
| DetourCrowd | DetourPathQueue.h | `src/path_queue.zig` | 11 |  |  | 7 | 4 |  |
| DetourCrowd | DetourLocalBoundary.h | `src/local_boundary.zig` | 8 |  |  | 7 | 1 |  |
| DetourCrowd | DetourObstacleAvoidance.h | `src/obstacle_avoidance.zig` | 33 |  |  | 28 | 5 |  |
| DetourCrowd | DetourProximityGrid.h | `src/proximity_grid.zig` | 10 |  |  | 7 | 3 |  |
| DetourTileCache | DetourTileCache.h | `src/tile_cache.zig` | 49 |  |  | 42 | 7 |  |
| DetourTileCache | DetourTileCacheBuilder.h | `src/tile_cache_builder.zig` | 30 |  |  | 30 |  |  |
| | | **all** | 415 |  |  | 333 | 82 |  |

The differential tests run twice: with every build switch off, and with every one on.

## Build switches

| upstream | Zig | state |
| --- | --- | --- |
| `DT_POLYREF64` (`RECASTNAVIGATION_DT_POLYREF64`): 64-bit polygon and tile references | `-Dpolyref64` | oracle-tested |
| `DT_VIRTUAL_QUERYFILTER` (`RECASTNAVIGATION_DT_VIRTUAL_QUERYFILTER`): a filter replaces its pass and cost functions | `-Dvirtual-query-filter` | oracle-tested |
| `RC_FAST_MATH` (`RECASTNAVIGATION_ENABLE_FAST_MATH`): no checks for infinite and NaN inputs | `-Dfast-math` | oracle-tested |

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
| `DT_SALT_BITS` | `salt_bits_64` | oracle-tested |
| `DT_TILE_BITS` | `tile_bits_64` | oracle-tested |
| `DT_POLY_BITS` | `poly_bits_64` | oracle-tested |
| `dtPolyRef` | `PolyRef` | oracle-tested |
| `dtTileRef` | `TileRef` | oracle-tested |
| `DT_VERTS_PER_POLYGON` | `verts_per_polygon` | oracle-tested |
| `DT_NAVMESH_MAGIC` | `nav_mesh_magic` | oracle-tested |
| `DT_NAVMESH_VERSION` | `nav_mesh_version` | oracle-tested |
| `DT_NAVMESH_STATE_MAGIC` | `nav_mesh_state_magic` | oracle-tested |
| `DT_NAVMESH_STATE_VERSION` | `nav_mesh_state_version` | oracle-tested |
| `DT_EXT_LINK` | `ext_link` | oracle-tested |
| `DT_NULL_LINK` | `null_link` | oracle-tested |
| `DT_OFFMESH_CON_BIDIR` | `off_mesh_connection_bidirectional` | oracle-tested |
| `DT_MAX_AREAS` | `area_count_max` | oracle-tested |
| `dtTileFlags`: `DT_TILE_FREE_DATA` | `TileFlags` | oracle-tested |
| `dtStraightPathFlags`: `DT_STRAIGHTPATH_START`, `DT_STRAIGHTPATH_END`, `DT_STRAIGHTPATH_OFFMESH_CONNECTION` | `StraightPathFlags` | oracle-tested |
| `dtStraightPathOptions`: `DT_STRAIGHTPATH_AREA_CROSSINGS`, `DT_STRAIGHTPATH_ALL_CROSSINGS` | `StraightPathOptions` | oracle-tested |
| `dtFindPathOptions`: `DT_FINDPATH_ANY_ANGLE` | `FindPathOptions` | oracle-tested |
| `dtRaycastOptions`: `DT_RAYCAST_USE_COSTS` | `RaycastOptions` | oracle-tested |
| `dtDetailTriEdgeFlags`: `DT_DETAIL_EDGE_BOUNDARY` | `detail_edge_boundary` | oracle-tested |
| `DT_RAY_CAST_LIMIT_PROPORTIONS` | `ray_cast_limit_proportions` | oracle-tested |
| `dtPolyTypes`: `DT_POLYTYPE_GROUND`, `DT_POLYTYPE_OFFMESH_CONNECTION` | `PolyType` | oracle-tested |
| `dtPoly`: `firstLink`, `verts`, `neis`, `flags`, `vertCount`, `areaAndtype` | `Poly` | oracle-tested |
| `dtPoly::setArea` | `Poly.setArea` | oracle-tested |
| `dtPoly::setType` | `Poly.setType` | oracle-tested |
| `dtPoly::getArea` | `Poly.getArea` | oracle-tested |
| `dtPoly::getType` | `Poly.getType` | oracle-tested |
| `dtPolyDetail`: `vertBase`, `triBase`, `vertCount`, `triCount` | `PolyDetail` | oracle-tested |
| `dtLink`: `ref`, `next`, `edge`, `side`, `bmin`, `bmax` | `Link` | oracle-tested |
| `dtBVNode`: `bmin`, `bmax`, `i` | `BvNode` | oracle-tested |
| `dtOffMeshConnection`: `pos`, `rad`, `poly`, `flags`, `side`, `userId` | `OffMeshConnection` | oracle-tested |
| `dtMeshHeader`: `magic`, `version`, `x`, `y`, `layer`, `userId`, `polyCount`, `vertCount`, `maxLinkCount`, `detailMeshCount`, `detailVertCount`, `detailTriCount`, `bvNodeCount`, `offMeshConCount`, `offMeshBase`, `walkableHeight`, `walkableRadius`, `walkableClimb`, `bmin`, `bmax`, `bvQuantFactor` | `MeshHeader` | oracle-tested |
| `dtMeshTile`: `salt`, `linksFreeList`, `header`, `polys`, `verts`, `links`, `detailMeshes`, `detailVerts`, `detailTris`, `bvTree`, `offMeshCons`, `data`, `dataSize`, `flags`, `next` | `MeshTile` | oracle-tested |
| `dtGetDetailTriEdgeFlags` | `getDetailTriEdgeFlags` | oracle-tested |
| `dtNavMeshParams`: `orig`, `tileWidth`, `tileHeight`, `maxTiles`, `maxPolys` | `NavMeshParams` | oracle-tested |
| `dtNavMesh` | `NavMesh` | oracle-tested |
| `dtNavMesh::dtNavMesh` | `NavMesh.empty` | oracle-tested |
| `dtNavMesh::~dtNavMesh` | `NavMesh.deinit` | oracle-tested |
| `dtNavMesh::init(params)` | `NavMesh.init` | oracle-tested |
| `dtNavMesh::init(data, dataSize, flags)` | `NavMesh.initSingleTile` | oracle-tested |
| `dtNavMesh::getParams` | `NavMesh.getParams` | oracle-tested |
| `dtNavMesh::addTile` | `NavMesh.addTile` | benchmarked |
| `dtNavMesh::removeTile` | `NavMesh.removeTile` | benchmarked |
| `dtNavMesh::calcTileLoc` | `NavMesh.calcTileLoc` | benchmarked |
| `dtNavMesh::getTileAt` | `NavMesh.getTileAt` | benchmarked |
| `dtNavMesh::getTilesAt` | `NavMesh.getTilesAt` | benchmarked |
| `dtNavMesh::getTileRefAt` | `NavMesh.getTileRefAt` | benchmarked |
| `dtNavMesh::getTileRef` | `NavMesh.getTileRef` | oracle-tested |
| `dtNavMesh::getTileByRef` | `NavMesh.getTileByRef` | benchmarked |
| `dtNavMesh::getMaxTiles` | `NavMesh.getMaxTiles` | oracle-tested |
| `dtNavMesh::getTile` | `NavMesh.getTile` | oracle-tested |
| `dtNavMesh::getTileAndPolyByRef` | `NavMesh.getTileAndPolyByRef` | benchmarked |
| `dtNavMesh::getTileAndPolyByRefUnsafe` | `NavMesh.getTileAndPolyByRefUnsafe` | oracle-tested |
| `dtNavMesh::isValidPolyRef` | `NavMesh.isValidPolyRef` | benchmarked |
| `dtNavMesh::getPolyRefBase` | `NavMesh.getPolyRefBase` | oracle-tested |
| `dtNavMesh::getOffMeshConnectionPolyEndPoints` | `NavMesh.getOffMeshConnectionPolyEndPoints` | benchmarked |
| `dtNavMesh::getOffMeshConnectionByRef` | `NavMesh.getOffMeshConnectionByRef` | benchmarked |
| `dtNavMesh::setPolyFlags` | `NavMesh.setPolyFlags` | benchmarked |
| `dtNavMesh::getPolyFlags` | `NavMesh.getPolyFlags` | benchmarked |
| `dtNavMesh::setPolyArea` | `NavMesh.setPolyArea` | benchmarked |
| `dtNavMesh::getPolyArea` | `NavMesh.getPolyArea` | benchmarked |
| `dtNavMesh::getTileStateSize` | `NavMesh.getTileStateSize` | oracle-tested |
| `dtNavMesh::storeTileState` | `NavMesh.storeTileState` | benchmarked |
| `dtNavMesh::restoreTileState` | `NavMesh.restoreTileState` | benchmarked |
| `dtNavMesh::encodePolyId` | `NavMesh.encodePolyId` | oracle-tested |
| `dtNavMesh::decodePolyId` | `NavMesh.decodePolyId` | oracle-tested |
| `dtNavMesh::decodePolyIdSalt` | `NavMesh.decodePolyIdSalt` | oracle-tested |
| `dtNavMesh::decodePolyIdTile` | `NavMesh.decodePolyIdTile` | oracle-tested |
| `dtNavMesh::decodePolyIdPoly` | `NavMesh.decodePolyIdPoly` | oracle-tested |

### DetourNavMeshBuilder.h → `src/nav_mesh_builder.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtNavMeshCreateParams`: `verts`, `vertCount`, `polys`, `polyFlags`, `polyAreas`, `polyCount`, `nvp`, `detailMeshes`, `detailVerts`, `detailVertsCount`, `detailTris`, `detailTriCount`, `offMeshConVerts`, `offMeshConRad`, `offMeshConFlags`, `offMeshConAreas`, `offMeshConDir`, `offMeshConUserID`, `offMeshConCount`, `userId`, `tileX`, `tileY`, `tileLayer`, `bmin`, `bmax`, `walkableHeight`, `walkableRadius`, `walkableClimb`, `cs`, `ch`, `buildBvTree` | `NavMeshCreateParams` | oracle-tested |
| `dtCreateNavMeshData` | `createNavMeshData` | benchmarked |
| `dtNavMeshHeaderSwapEndian` | `navMeshHeaderSwapEndian` | benchmarked |
| `dtNavMeshDataSwapEndian` | `navMeshDataSwapEndian` | benchmarked |

### DetourNavMeshQuery.h → `src/nav_mesh_query.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtQueryFilter` | `QueryFilter` | oracle-tested |
| `dtQueryFilter::dtQueryFilter` | `QueryFilter`, its default field values | oracle-tested |
| `dtQueryFilter::passFilter` | `QueryFilter.passFilter` | oracle-tested |
| `dtQueryFilter::getCost` | `QueryFilter.getCost` | oracle-tested |
| `dtQueryFilter::getAreaCost` | `QueryFilter`, the `area_cost` field | oracle-tested |
| `dtQueryFilter::setAreaCost` | `QueryFilter`, the `area_cost` field | oracle-tested |
| `dtQueryFilter::getIncludeFlags` | `QueryFilter`, the `include_flags` field | oracle-tested |
| `dtQueryFilter::setIncludeFlags` | `QueryFilter`, the `include_flags` field | oracle-tested |
| `dtQueryFilter::getExcludeFlags` | `QueryFilter`, the `exclude_flags` field | oracle-tested |
| `dtQueryFilter::setExcludeFlags` | `QueryFilter`, the `exclude_flags` field | oracle-tested |
| `dtRaycastHit`: `t`, `hitNormal`, `hitEdgeIndex`, `path`, `pathCount`, `maxPath`, `pathCost` | `RaycastHit` | oracle-tested |
| `dtPolyQuery` | `PolyQuery` | oracle-tested |
| `dtPolyQuery::process` | `PolyQuery.process` | oracle-tested |
| `dtNavMeshQuery` | `NavMeshQuery` | oracle-tested |
| `dtNavMeshQuery::dtNavMeshQuery` | `NavMeshQuery.empty` | oracle-tested |
| `dtNavMeshQuery::~dtNavMeshQuery` | `NavMeshQuery.deinit` | oracle-tested |
| `dtNavMeshQuery::init` | `NavMeshQuery.init` | oracle-tested |
| `dtNavMeshQuery::findPath` | `NavMeshQuery.findPath` | benchmarked |
| `dtNavMeshQuery::findStraightPath` | `NavMeshQuery.findStraightPath` | benchmarked |
| `dtNavMeshQuery::initSlicedFindPath` | `NavMeshQuery.initSlicedFindPath` | benchmarked |
| `dtNavMeshQuery::updateSlicedFindPath` | `NavMeshQuery.updateSlicedFindPath` | benchmarked |
| `dtNavMeshQuery::finalizeSlicedFindPath` | `NavMeshQuery.finalizeSlicedFindPath` | benchmarked |
| `dtNavMeshQuery::finalizeSlicedFindPathPartial` | `NavMeshQuery.finalizeSlicedFindPathPartial` | oracle-tested |
| `dtNavMeshQuery::findPolysAroundCircle` | `NavMeshQuery.findPolysAroundCircle` | benchmarked |
| `dtNavMeshQuery::findPolysAroundShape` | `NavMeshQuery.findPolysAroundShape` | benchmarked |
| `dtNavMeshQuery::getPathFromDijkstraSearch` | `NavMeshQuery.getPathFromDijkstraSearch` | oracle-tested |
| `dtNavMeshQuery::findNearestPoly(center, halfExtents, filter, nearestRef, nearestPt)` | `NavMeshQuery.findNearestPoly`, `is_over_poly` null | benchmarked |
| `dtNavMeshQuery::findNearestPoly(center, halfExtents, filter, nearestRef, nearestPt, isOverPoly)` | `NavMeshQuery.findNearestPoly` | benchmarked |
| `dtNavMeshQuery::queryPolygons(center, halfExtents, filter, polys, polyCount, maxPolys)` | `NavMeshQuery.queryPolygons` | benchmarked |
| `dtNavMeshQuery::queryPolygons(center, halfExtents, filter, query)` | `NavMeshQuery.queryPolygonsWith` | benchmarked |
| `dtNavMeshQuery::findLocalNeighbourhood` | `NavMeshQuery.findLocalNeighbourhood` | benchmarked |
| `dtNavMeshQuery::moveAlongSurface` | `NavMeshQuery.moveAlongSurface` | benchmarked |
| `dtNavMeshQuery::raycast(startRef, startPos, endPos, filter, t, hitNormal, path, pathCount, maxPath)` | `NavMeshQuery.raycastPath` | benchmarked |
| `dtNavMeshQuery::raycast(startRef, startPos, endPos, filter, options, hit, prevRef)` | `NavMeshQuery.raycast` | benchmarked |
| `dtNavMeshQuery::findDistanceToWall` | `NavMeshQuery.findDistanceToWall` | benchmarked |
| `dtNavMeshQuery::getPolyWallSegments` | `NavMeshQuery.getPolyWallSegments` | benchmarked |
| `dtNavMeshQuery::findRandomPoint` | `NavMeshQuery.findRandomPoint` | benchmarked |
| `dtNavMeshQuery::findRandomPointAroundCircle` | `NavMeshQuery.findRandomPointAroundCircle` | benchmarked |
| `dtNavMeshQuery::closestPointOnPoly` | `NavMeshQuery.closestPointOnPoly` | benchmarked |
| `dtNavMeshQuery::closestPointOnPolyBoundary` | `NavMeshQuery.closestPointOnPolyBoundary` | benchmarked |
| `dtNavMeshQuery::getPolyHeight` | `NavMeshQuery.getPolyHeight` | benchmarked |
| `dtNavMeshQuery::isValidPolyRef` | `NavMeshQuery.isValidPolyRef` | benchmarked |
| `dtNavMeshQuery::isInClosedList` | `NavMeshQuery.isInClosedList` | oracle-tested |
| `dtNavMeshQuery::getNodePool` | `NavMeshQuery.getNodePool` | oracle-tested |
| `dtNavMeshQuery::getAttachedNavMesh` | `NavMeshQuery.getAttachedNavMesh` | oracle-tested |

### DetourNode.h → `src/node.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtNodeFlags`: `DT_NODE_OPEN`, `DT_NODE_CLOSED`, `DT_NODE_PARENT_DETACHED` | `NodeFlags` | oracle-tested |
| `dtNodeIndex` | `NodeIndex` | oracle-tested |
| `DT_NULL_IDX` | `null_index` | oracle-tested |
| `DT_NODE_PARENT_BITS` | `node_parent_bits` | oracle-tested |
| `DT_NODE_STATE_BITS` | `node_state_bits` | oracle-tested |
| `dtNode`: `pos`, `cost`, `total`, `DT_NODE_PARENT_BITS`, `DT_NODE_STATE_BITS`, `3`, `id` | `Node` | oracle-tested |
| `DT_MAX_STATES_PER_NODE` | `node_states_max` | oracle-tested |
| `dtNodePool` | `NodePool` | oracle-tested |
| `dtNodePool::dtNodePool` | `NodePool.init` | oracle-tested |
| `dtNodePool::~dtNodePool` | `NodePool.deinit` | oracle-tested |
| `dtNodePool::clear` | `NodePool.clear` | oracle-tested |
| `dtNodePool::getNode` | `NodePool.getNode` | oracle-tested |
| `dtNodePool::findNode` | `NodePool.findNode` | oracle-tested |
| `dtNodePool::findNodes` | `NodePool.findNodes` | oracle-tested |
| `dtNodePool::getNodeIdx` | `NodePool.getNodeIdx` | oracle-tested |
| `dtNodePool::getNodeAtIdx` | `NodePool.getNodeAtIdx` | oracle-tested |
| `dtNodePool::getMemUsed` | `NodePool.getMemUsed` | oracle-tested |
| `dtNodePool::getMaxNodes` | `NodePool.getMaxNodes` | oracle-tested |
| `dtNodePool::getHashSize` | `NodePool.getHashSize` | oracle-tested |
| `dtNodePool::getFirst` | `NodePool.getFirst` | oracle-tested |
| `dtNodePool::getNext` | `NodePool.getNext` | oracle-tested |
| `dtNodePool::getNodeCount` | `NodePool.getNodeCount` | oracle-tested |
| `dtNodeQueue` | `NodeQueue` | oracle-tested |
| `dtNodeQueue::dtNodeQueue` | `NodeQueue.init` | oracle-tested |
| `dtNodeQueue::~dtNodeQueue` | `NodeQueue.deinit` | oracle-tested |
| `dtNodeQueue::clear` | `NodeQueue.clear` | oracle-tested |
| `dtNodeQueue::top` | `NodeQueue.top` | oracle-tested |
| `dtNodeQueue::pop` | `NodeQueue.pop` | oracle-tested |
| `dtNodeQueue::push` | `NodeQueue.push` | oracle-tested |
| `dtNodeQueue::modify` | `NodeQueue.modify` | oracle-tested |
| `dtNodeQueue::empty` | `NodeQueue.isEmpty` | oracle-tested |
| `dtNodeQueue::getMemUsed` | `NodeQueue.getMemUsed` | oracle-tested |
| `dtNodeQueue::getCapacity` | `NodeQueue.getCapacity` | oracle-tested |

### DetourCommon.h → `src/common.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtMin` | `min` | oracle-tested |
| `dtMax` | `max` | oracle-tested |
| `dtAbs` | `abs` | oracle-tested |
| `dtSqr` | `sqr` | oracle-tested |
| `dtClamp` | `clamp` | oracle-tested |
| `dtVcross` | `vcross` | oracle-tested |
| `dtVdot` | `vdot` | oracle-tested |
| `dtVmad` | `vmad` | oracle-tested |
| `dtVlerp` | `vlerp` | oracle-tested |
| `dtVadd` | `vadd` | oracle-tested |
| `dtVsub` | `vsub` | oracle-tested |
| `dtVscale` | `vscale` | oracle-tested |
| `dtVmin` | `vmin` | oracle-tested |
| `dtVmax` | `vmax` | oracle-tested |
| `dtVlen` | `vlen` | oracle-tested |
| `dtVlenSqr` | `vlenSqr` | oracle-tested |
| `dtVdist` | `vdist` | oracle-tested |
| `dtVdistSqr` | `vdistSqr` | oracle-tested |
| `dtVdist2D` | `vdist2d` | oracle-tested |
| `dtVdist2DSqr` | `vdist2dSqr` | oracle-tested |
| `dtVnormalize` | `vnormalize` | oracle-tested |
| `dtVequal` | `vequal` | oracle-tested |
| `dtVisfinite` | `visfinite` | oracle-tested |
| `dtVisfinite2D` | `visfinite2d` | oracle-tested |
| `dtVdot2D` | `vdot2d` | oracle-tested |
| `dtVperp2D` | `vperp2d` | oracle-tested |
| `dtTriArea2D` | `triArea2d` | oracle-tested |
| `dtOverlapQuantBounds` | `overlapQuantBounds` | oracle-tested |
| `dtOverlapBounds` | `overlapBounds` | oracle-tested |
| `dtClosestPtPointTriangle` | `closestPointOnTriangle` | benchmarked |
| `dtClosestHeightPointTriangle` | `closestHeightOnTriangle` | benchmarked |
| `dtIntersectSegmentPoly2D` | `intersectSegmentPoly2d` | benchmarked |
| `dtIntersectSegSeg2D` | `intersectSegmentSegment2d` | benchmarked |
| `dtPointInPolygon` | `pointInPolygon` | benchmarked |
| `dtDistancePtPolyEdgesSqr` | `distancePointPolyEdgesSqr` | benchmarked |
| `dtDistancePtSegSqr2D` | `distancePointSegmentSqr2d` | benchmarked |
| `dtCalcPolyCenter` | `calcPolyCenter` | benchmarked |
| `dtOverlapPolyPoly2D` | `overlapPolyPoly2d` | benchmarked |
| `dtNextPow2` | `nextPow2` | oracle-tested |
| `dtIlog2` | `ilog2` | oracle-tested |
| `dtAlign4` | `align4` | oracle-tested |
| `dtOppositeTile` | `oppositeTile` | oracle-tested |
| `dtSwapEndian` for `float` | `swapEndianFloat` | oracle-tested |
| `dtRandomPointInConvexPoly` | `randomPointInConvexPoly` | benchmarked |

### DetourStatus.h → `src/status.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtStatus` | `Status` | oracle-tested |
| `DT_FAILURE` | `Status.failure`, and the `failed` field | oracle-tested |
| `DT_SUCCESS` | `Status.success`, and the `succeeded` field | oracle-tested |
| `DT_IN_PROGRESS` | `Status`, the `in_progress` field | oracle-tested |
| `DT_STATUS_DETAIL_MASK` | `Status.detail_mask` | oracle-tested |
| `DT_WRONG_MAGIC` | `Status`, the `wrong_magic` field | oracle-tested |
| `DT_WRONG_VERSION` | `Status`, the `wrong_version` field | oracle-tested |
| `DT_OUT_OF_MEMORY` | `Status`, the `out_of_memory` field | oracle-tested |
| `DT_INVALID_PARAM` | `Status`, the `invalid_param` field | oracle-tested |
| `DT_BUFFER_TOO_SMALL` | `Status`, the `buffer_too_small` field | oracle-tested |
| `DT_OUT_OF_NODES` | `Status`, the `out_of_nodes` field | oracle-tested |
| `DT_PARTIAL_RESULT` | `Status`, the `partial_result` field | oracle-tested |
| `DT_ALREADY_OCCUPIED` | `Status`, the `already_occupied` field | oracle-tested |
| `dtStatusSucceed` | `Status`, the `succeeded` field | oracle-tested |
| `dtStatusFailed` | `Status`, the `failed` field | oracle-tested |
| `dtStatusInProgress` | `Status`, the `in_progress` field | oracle-tested |
| `dtStatusDetail` | `Status.has` | oracle-tested |

### DetourMath.h → `src/common.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtMathIsfinite` | `isFinite` | oracle-tested |

## DetourCrowd

### DetourCrowd.h → `src/crowd.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `DT_CROWDAGENT_MAX_NEIGHBOURS` | `agent_neighbour_count_max` | oracle-tested |
| `DT_CROWDAGENT_MAX_CORNERS` | `agent_corner_count_max` | oracle-tested |
| `DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS` | `obstacle_avoidance_params_count_max` | oracle-tested |
| `DT_CROWD_MAX_QUERY_FILTER_TYPE` | `query_filter_type_count_max` | oracle-tested |
| `dtCrowdNeighbour`: `idx`, `dist` | `CrowdNeighbour` | oracle-tested |
| `CrowdAgentState`: `DT_CROWDAGENT_STATE_INVALID`, `DT_CROWDAGENT_STATE_WALKING`, `DT_CROWDAGENT_STATE_OFFMESH` | `CrowdAgentTraversal` | oracle-tested |
| `dtCrowdAgentParams`: `radius`, `height`, `maxAcceleration`, `maxSpeed`, `collisionQueryRange`, `pathOptimizationRange`, `separationWeight`, `updateFlags`, `obstacleAvoidanceType`, `queryFilterType`, `userData` | `CrowdAgentParams` | oracle-tested |
| `MoveRequestState`: `DT_CROWDAGENT_TARGET_NONE`, `DT_CROWDAGENT_TARGET_FAILED`, `DT_CROWDAGENT_TARGET_VALID`, `DT_CROWDAGENT_TARGET_REQUESTING`, `DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE`, `DT_CROWDAGENT_TARGET_WAITING_FOR_PATH`, `DT_CROWDAGENT_TARGET_VELOCITY` | `MoveRequestStage` | oracle-tested |
| `dtCrowdAgent`: `active`, `state`, `partial`, `corridor`, `boundary`, `topologyOptTime`, `neis`, `nneis`, `desiredSpeed`, `npos`, `disp`, `dvel`, `nvel`, `vel`, `params`, `cornerVerts`, `cornerFlags`, `cornerPolys`, `ncorners`, `targetState`, `targetRef`, `targetPos`, `targetPathqRef`, `targetReplan`, `targetReplanTime` | `CrowdAgent` | oracle-tested |
| `dtCrowdAgentAnimation`: `active`, `initPos`, `startPos`, `endPos`, `polyRef`, `t`, `tmax` | `CrowdAgentAnimation` | oracle-tested |
| `UpdateFlags`: `DT_CROWD_ANTICIPATE_TURNS`, `DT_CROWD_OBSTACLE_AVOIDANCE`, `DT_CROWD_SEPARATION`, `DT_CROWD_OPTIMIZE_VIS`, `DT_CROWD_OPTIMIZE_TOPO` | `UpdateFlags` | oracle-tested |
| `dtCrowdAgentDebugInfo`: `idx`, `optStart`, `optEnd`, `vod` | `CrowdAgentDebugInfo` | oracle-tested |
| `dtCrowd` | `Crowd` | oracle-tested |
| `dtCrowd::dtCrowd` | `Crowd.empty` | oracle-tested |
| `dtCrowd::~dtCrowd` | `Crowd.deinit` | oracle-tested |
| `dtCrowd::init` | `Crowd.init` | oracle-tested |
| `dtCrowd::setObstacleAvoidanceParams` | `Crowd.setObstacleAvoidanceParams` | oracle-tested |
| `dtCrowd::getObstacleAvoidanceParams` | `Crowd.getObstacleAvoidanceParams` | oracle-tested |
| `dtCrowd::getAgent` | `Crowd.getAgent` | oracle-tested |
| `dtCrowd::getEditableAgent` | `Crowd.getEditableAgent` | oracle-tested |
| `dtCrowd::getAgentCount` | `Crowd.getAgentCount` | oracle-tested |
| `dtCrowd::addAgent` | `Crowd.addAgent` | benchmarked |
| `dtCrowd::updateAgentParameters` | `Crowd.updateAgentParameters` | oracle-tested |
| `dtCrowd::removeAgent` | `Crowd.removeAgent` | benchmarked |
| `dtCrowd::requestMoveTarget` | `Crowd.requestMoveTarget` | benchmarked |
| `dtCrowd::requestMoveVelocity` | `Crowd.requestMoveVelocity` | oracle-tested |
| `dtCrowd::resetMoveTarget` | `Crowd.resetMoveTarget` | oracle-tested |
| `dtCrowd::getActiveAgents` | `Crowd.getActiveAgents` | oracle-tested |
| `dtCrowd::update` | `Crowd.update` | benchmarked |
| `dtCrowd::getFilter` | `Crowd.getFilter` | oracle-tested |
| `dtCrowd::getEditableFilter` | `Crowd.getEditableFilter` | oracle-tested |
| `dtCrowd::getQueryHalfExtents` | `Crowd.getQueryHalfExtents` | oracle-tested |
| `dtCrowd::getQueryExtents` | `Crowd.getQueryExtents` | oracle-tested |
| `dtCrowd::getVelocitySampleCount` | `Crowd.getVelocitySampleCount` | oracle-tested |
| `dtCrowd::getGrid` | `Crowd.getGrid` | oracle-tested |
| `dtCrowd::getPathQueue` | `Crowd.getPathQueue` | oracle-tested |
| `dtCrowd::getNavMeshQuery` | `Crowd.getNavMeshQuery` | oracle-tested |

### DetourPathCorridor.h → `src/path_corridor.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtPathCorridor` | `PathCorridor` | oracle-tested |
| `dtPathCorridor::dtPathCorridor` | `PathCorridor.empty` | oracle-tested |
| `dtPathCorridor::~dtPathCorridor` | `PathCorridor.deinit` | oracle-tested |
| `dtPathCorridor::init` | `PathCorridor.init` | oracle-tested |
| `dtPathCorridor::reset` | `PathCorridor.reset` | oracle-tested |
| `dtPathCorridor::findCorners` | `PathCorridor.findCorners` | benchmarked |
| `dtPathCorridor::optimizePathVisibility` | `PathCorridor.optimizePathVisibility` | benchmarked |
| `dtPathCorridor::optimizePathTopology` | `PathCorridor.optimizePathTopology` | benchmarked |
| `dtPathCorridor::moveOverOffmeshConnection` | `PathCorridor.moveOverOffmeshConnection` | oracle-tested |
| `dtPathCorridor::fixPathStart` | `PathCorridor.fixPathStart` | oracle-tested |
| `dtPathCorridor::trimInvalidPath` | `PathCorridor.trimInvalidPath` | oracle-tested |
| `dtPathCorridor::isValid` | `PathCorridor.isValid` | oracle-tested |
| `dtPathCorridor::movePosition` | `PathCorridor.movePosition` | benchmarked |
| `dtPathCorridor::moveTargetPosition` | `PathCorridor.moveTargetPosition` | oracle-tested |
| `dtPathCorridor::setCorridor` | `PathCorridor.setCorridor` | benchmarked |
| `dtPathCorridor::getPos` | `PathCorridor.getPos` | oracle-tested |
| `dtPathCorridor::getTarget` | `PathCorridor.getTarget` | oracle-tested |
| `dtPathCorridor::getFirstPoly` | `PathCorridor.getFirstPoly` | oracle-tested |
| `dtPathCorridor::getLastPoly` | `PathCorridor.getLastPoly` | oracle-tested |
| `dtPathCorridor::getPath` | `PathCorridor.getPath` | oracle-tested |
| `dtPathCorridor::getPathCount` | `PathCorridor.getPathCount` | oracle-tested |
| `dtMergeCorridorStartMoved` | `mergeCorridorStartMoved` | oracle-tested |
| `dtMergeCorridorEndMoved` | `mergeCorridorEndMoved` | oracle-tested |
| `dtMergeCorridorStartShortcut` | `mergeCorridorStartShortcut` | oracle-tested |

### DetourPathQueue.h → `src/path_queue.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `DT_PATHQ_INVALID` | `path_queue_invalid` | oracle-tested |
| `dtPathQueueRef` | `PathQueueRef` | oracle-tested |
| `dtPathQueue` | `PathQueue` | oracle-tested |
| `dtPathQueue::dtPathQueue` | `PathQueue.empty` | oracle-tested |
| `dtPathQueue::~dtPathQueue` | `PathQueue.deinit` | oracle-tested |
| `dtPathQueue::init` | `PathQueue.init` | oracle-tested |
| `dtPathQueue::update` | `PathQueue.update` | benchmarked |
| `dtPathQueue::request` | `PathQueue.request` | benchmarked |
| `dtPathQueue::getRequestStatus` | `PathQueue.getRequestStatus` | benchmarked |
| `dtPathQueue::getPathResult` | `PathQueue.getPathResult` | benchmarked |
| `dtPathQueue::getNavQuery` | `PathQueue.getNavQuery` | oracle-tested |

### DetourLocalBoundary.h → `src/local_boundary.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtLocalBoundary` | `LocalBoundary` | oracle-tested |
| `dtLocalBoundary::dtLocalBoundary` | `LocalBoundary.empty` | oracle-tested |
| `dtLocalBoundary::reset` | `LocalBoundary.reset` | oracle-tested |
| `dtLocalBoundary::update` | `LocalBoundary.update` | benchmarked |
| `dtLocalBoundary::isValid` | `LocalBoundary.isValid` | oracle-tested |
| `dtLocalBoundary::getCenter` | `LocalBoundary.getCenter` | oracle-tested |
| `dtLocalBoundary::getSegmentCount` | `LocalBoundary.getSegmentCount` | oracle-tested |
| `dtLocalBoundary::getSegment` | `LocalBoundary.getSegment` | oracle-tested |

### DetourObstacleAvoidance.h → `src/obstacle_avoidance.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtObstacleCircle`: `p`, `vel`, `dvel`, `rad`, `dp`, `np` | `ObstacleCircle` | oracle-tested |
| `dtObstacleSegment`: `p`, `q`, `touch` | `ObstacleSegment` | oracle-tested |
| `dtObstacleAvoidanceDebugData` | `ObstacleAvoidanceDebug` | oracle-tested |
| `dtObstacleAvoidanceDebugData::dtObstacleAvoidanceDebugData` | `ObstacleAvoidanceDebug.empty` | oracle-tested |
| `dtObstacleAvoidanceDebugData::~dtObstacleAvoidanceDebugData` | `ObstacleAvoidanceDebug.deinit` | oracle-tested |
| `dtObstacleAvoidanceDebugData::init` | `ObstacleAvoidanceDebug.init` | oracle-tested |
| `dtObstacleAvoidanceDebugData::reset` | `ObstacleAvoidanceDebug.reset` | oracle-tested |
| `dtObstacleAvoidanceDebugData::addSample` | `ObstacleAvoidanceDebug.addSample` | oracle-tested |
| `dtObstacleAvoidanceDebugData::normalizeSamples` | `ObstacleAvoidanceDebug.normalizeSamples` | oracle-tested |
| `dtObstacleAvoidanceDebugData::getSampleCount` | `ObstacleAvoidanceDebug.getSampleCount` | oracle-tested |
| `dtObstacleAvoidanceDebugData::getSampleVelocity` | `ObstacleAvoidanceDebug.getSampleVelocity` | oracle-tested |
| `dtObstacleAvoidanceDebugData::getSampleSize` | `ObstacleAvoidanceDebug.getSampleSize` | oracle-tested |
| `dtObstacleAvoidanceDebugData::getSamplePenalty` | `ObstacleAvoidanceDebug.getSamplePenalty` | oracle-tested |
| `dtObstacleAvoidanceDebugData::getSampleDesiredVelocityPenalty` | `ObstacleAvoidanceDebug.getSampleDesiredVelocityPenalty` | oracle-tested |
| `dtObstacleAvoidanceDebugData::getSampleCurrentVelocityPenalty` | `ObstacleAvoidanceDebug.getSampleCurrentVelocityPenalty` | oracle-tested |
| `dtObstacleAvoidanceDebugData::getSamplePreferredSidePenalty` | `ObstacleAvoidanceDebug.getSamplePreferredSidePenalty` | oracle-tested |
| `dtObstacleAvoidanceDebugData::getSampleCollisionTimePenalty` | `ObstacleAvoidanceDebug.getSampleCollisionTimePenalty` | oracle-tested |
| `DT_MAX_PATTERN_DIVS` | `pattern_division_count_max` | oracle-tested |
| `DT_MAX_PATTERN_RINGS` | `pattern_ring_count_max` | oracle-tested |
| `dtObstacleAvoidanceParams`: `velBias`, `weightDesVel`, `weightCurVel`, `weightSide`, `weightToi`, `horizTime`, `gridSize`, `adaptiveDivs`, `adaptiveRings`, `adaptiveDepth` | `ObstacleAvoidanceParams` | oracle-tested |
| `dtObstacleAvoidanceQuery` | `ObstacleAvoidanceQuery` | oracle-tested |
| `dtObstacleAvoidanceQuery::dtObstacleAvoidanceQuery` | `ObstacleAvoidanceQuery.empty` | oracle-tested |
| `dtObstacleAvoidanceQuery::~dtObstacleAvoidanceQuery` | `ObstacleAvoidanceQuery.deinit` | oracle-tested |
| `dtObstacleAvoidanceQuery::init` | `ObstacleAvoidanceQuery.init` | oracle-tested |
| `dtObstacleAvoidanceQuery::reset` | `ObstacleAvoidanceQuery.reset` | benchmarked |
| `dtObstacleAvoidanceQuery::addCircle` | `ObstacleAvoidanceQuery.addCircle` | benchmarked |
| `dtObstacleAvoidanceQuery::addSegment` | `ObstacleAvoidanceQuery.addSegment` | benchmarked |
| `dtObstacleAvoidanceQuery::sampleVelocityGrid` | `ObstacleAvoidanceQuery.sampleVelocityGrid` | benchmarked |
| `dtObstacleAvoidanceQuery::sampleVelocityAdaptive` | `ObstacleAvoidanceQuery.sampleVelocityAdaptive` | benchmarked |
| `dtObstacleAvoidanceQuery::getObstacleCircleCount` | `ObstacleAvoidanceQuery.getObstacleCircleCount` | oracle-tested |
| `dtObstacleAvoidanceQuery::getObstacleCircle` | `ObstacleAvoidanceQuery.getObstacleCircle` | oracle-tested |
| `dtObstacleAvoidanceQuery::getObstacleSegmentCount` | `ObstacleAvoidanceQuery.getObstacleSegmentCount` | oracle-tested |
| `dtObstacleAvoidanceQuery::getObstacleSegment` | `ObstacleAvoidanceQuery.getObstacleSegment` | oracle-tested |

### DetourProximityGrid.h → `src/proximity_grid.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtProximityGrid` | `ProximityGrid` | oracle-tested |
| `dtProximityGrid::dtProximityGrid` | `ProximityGrid.empty` | oracle-tested |
| `dtProximityGrid::~dtProximityGrid` | `ProximityGrid.deinit` | oracle-tested |
| `dtProximityGrid::init` | `ProximityGrid.init` | oracle-tested |
| `dtProximityGrid::clear` | `ProximityGrid.clear` | benchmarked |
| `dtProximityGrid::addItem` | `ProximityGrid.addItem` | benchmarked |
| `dtProximityGrid::queryItems` | `ProximityGrid.queryItems` | benchmarked |
| `dtProximityGrid::getItemCountAt` | `ProximityGrid.getItemCountAt` | oracle-tested |
| `dtProximityGrid::getBounds` | `ProximityGrid.getBounds` | oracle-tested |
| `dtProximityGrid::getCellSize` | `ProximityGrid.getCellSize` | oracle-tested |

## DetourTileCache

### DetourTileCache.h → `src/tile_cache.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtObstacleRef` | `ObstacleRef` | oracle-tested |
| `dtCompressedTileRef` | `CompressedTileRef` | oracle-tested |
| `dtCompressedTileFlags`: `DT_COMPRESSEDTILE_FREE_DATA` | `compressed_tile_free_data` | oracle-tested |
| `dtCompressedTile`: `salt`, `header`, `compressed`, `compressedSize`, `data`, `dataSize`, `flags`, `next` | `CompressedTile` | oracle-tested |
| `ObstacleState`: `DT_OBSTACLE_EMPTY`, `DT_OBSTACLE_PROCESSING`, `DT_OBSTACLE_PROCESSED`, `DT_OBSTACLE_REMOVING` | `ObstaclePhase` | oracle-tested |
| `ObstacleType`: `DT_OBSTACLE_CYLINDER`, `DT_OBSTACLE_BOX`, `DT_OBSTACLE_ORIENTED_BOX` | `ObstacleType` | oracle-tested |
| `dtObstacleCylinder`: `pos`, `radius`, `height` | `ObstacleCylinder` | oracle-tested |
| `dtObstacleBox`: `bmin`, `bmax` | `ObstacleBox` | oracle-tested |
| `dtObstacleOrientedBox`: `center`, `halfExtents`, `rotAux` | `ObstacleOrientedBox` | oracle-tested |
| `DT_MAX_TOUCHED_TILES` | `touched_tile_count_max` | oracle-tested |
| `dtTileCacheObstacle`: `touched`, `pending`, `salt`, `type`, `state`, `ntouched`, `npending`, `next` | `Obstacle` | oracle-tested |
| `dtTileCacheParams`: `orig`, `cs`, `ch`, `width`, `height`, `walkableHeight`, `walkableRadius`, `walkableClimb`, `maxSimplificationError`, `maxTiles`, `maxObstacles` | `TileCacheParams` | oracle-tested |
| `dtTileCacheMeshProcess` | `MeshProcess` | oracle-tested |
| `dtTileCacheMeshProcess::process` | `MeshProcess.process` | oracle-tested |
| `dtTileCache` | `TileCache` | oracle-tested |
| `dtTileCache::dtTileCache` | `TileCache.empty` | oracle-tested |
| `dtTileCache::~dtTileCache` | `TileCache.deinit` | oracle-tested |
| `dtTileCache::getAlloc` | `TileCache.getAlloc` | oracle-tested |
| `dtTileCache::getCompressor` | `TileCache.getCompressor` | oracle-tested |
| `dtTileCache::getParams` | `TileCache.getParams` | oracle-tested |
| `dtTileCache::getTileCount` | `TileCache.getTileCount` | oracle-tested |
| `dtTileCache::getTile` | `TileCache.getTile` | oracle-tested |
| `dtTileCache::getObstacleCount` | `TileCache.getObstacleCount` | oracle-tested |
| `dtTileCache::getObstacle` | `TileCache.getObstacle` | oracle-tested |
| `dtTileCache::getObstacleByRef` | `TileCache.getObstacleByRef` | oracle-tested |
| `dtTileCache::getObstacleRef` | `TileCache.getObstacleRef` | oracle-tested |
| `dtTileCache::init` | `TileCache.init` | oracle-tested |
| `dtTileCache::getTilesAt` | `TileCache.getTilesAt` | oracle-tested |
| `dtTileCache::getTileAt` | `TileCache.getTileAt` | oracle-tested |
| `dtTileCache::getTileRef` | `TileCache.getTileRef` | oracle-tested |
| `dtTileCache::getTileByRef` | `TileCache.getTileByRef` | oracle-tested |
| `dtTileCache::addTile` | `TileCache.addTile` | benchmarked |
| `dtTileCache::removeTile` | `TileCache.removeTile` | benchmarked |
| `dtTileCache::addObstacle` | `TileCache.addObstacle` | benchmarked |
| `dtTileCache::addBoxObstacle(bmin, bmax, result)` | `TileCache.addBoxObstacle` | oracle-tested |
| `dtTileCache::addBoxObstacle(center, halfExtents, yRadians, result)` | `TileCache.addOrientedBoxObstacle` | oracle-tested |
| `dtTileCache::removeObstacle` | `TileCache.removeObstacle` | benchmarked |
| `dtTileCache::queryTiles` | `TileCache.queryTiles` | benchmarked |
| `dtTileCache::update` | `TileCache.update` | benchmarked |
| `dtTileCache::buildNavMeshTilesAt` | `TileCache.buildNavMeshTilesAt` | oracle-tested |
| `dtTileCache::buildNavMeshTile` | `TileCache.buildNavMeshTile` | benchmarked |
| `dtTileCache::calcTightTileBounds` | `TileCache.calcTightTileBounds` | oracle-tested |
| `dtTileCache::getObstacleBounds` | `TileCache.getObstacleBounds` | oracle-tested |
| `dtTileCache::encodeTileId` | `TileCache.encodeTileId` | oracle-tested |
| `dtTileCache::decodeTileIdSalt` | `TileCache.decodeTileIdSalt` | oracle-tested |
| `dtTileCache::decodeTileIdTile` | `TileCache.decodeTileIdTile` | oracle-tested |
| `dtTileCache::encodeObstacleId` | `TileCache.encodeObstacleId` | oracle-tested |
| `dtTileCache::decodeObstacleIdSalt` | `TileCache.decodeObstacleIdSalt` | oracle-tested |
| `dtTileCache::decodeObstacleIdObstacle` | `TileCache.decodeObstacleIdObstacle` | oracle-tested |

### DetourTileCacheBuilder.h → `src/tile_cache_builder.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `DT_TILECACHE_MAGIC` | `tile_cache_magic` | oracle-tested |
| `DT_TILECACHE_VERSION` | `tile_cache_version` | oracle-tested |
| `DT_TILECACHE_NULL_AREA` | `null_area` | oracle-tested |
| `DT_TILECACHE_WALKABLE_AREA` | `walkable_area` | oracle-tested |
| `DT_TILECACHE_NULL_IDX` | `null_index` | oracle-tested |
| `dtTileCacheLayerHeader`: `magic`, `version`, `tx`, `ty`, `tlayer`, `bmin`, `bmax`, `hmin`, `hmax`, `width`, `height`, `minx`, `maxx`, `miny`, `maxy` | `LayerHeader` | oracle-tested |
| `dtTileCacheLayer`: `header`, `regCount`, `heights`, `areas`, `cons`, `regs` | `Layer` | oracle-tested |
| `dtTileCacheContour`: `nverts`, `verts`, `reg`, `area` | `Contour` | oracle-tested |
| `dtTileCacheContourSet`: `nconts`, `conts` | `ContourSet` | oracle-tested |
| `dtTileCachePolyMesh`: `nvp`, `nverts`, `npolys`, `verts`, `polys`, `flags`, `areas` | `PolyMesh` | oracle-tested |
| `dtTileCacheAlloc` | `BuildAllocator` | oracle-tested |
| `dtTileCacheAlloc::reset` | `BuildAllocator.reset` | oracle-tested |
| `dtTileCacheCompressor` | `Compressor` | oracle-tested |
| `dtTileCacheCompressor::maxCompressedSize` | `Compressor.maxCompressedSize` | oracle-tested |
| `dtTileCacheCompressor::compress` | `Compressor.compress` | oracle-tested |
| `dtTileCacheCompressor::decompress` | `Compressor.decompress` | oracle-tested |
| `dtBuildTileCacheLayer` | `buildLayer` | oracle-tested |
| `dtFreeTileCacheLayer` | `freeLayer` | oracle-tested |
| `dtDecompressTileCacheLayer` | `decompressLayer` | oracle-tested |
| `dtAllocTileCacheContourSet` | `allocContourSet` | oracle-tested |
| `dtFreeTileCacheContourSet` | `freeContourSet` | oracle-tested |
| `dtAllocTileCachePolyMesh` | `allocPolyMesh` | oracle-tested |
| `dtFreeTileCachePolyMesh` | `freePolyMesh` | oracle-tested |
| `dtMarkCylinderArea` | `markCylinderArea` | oracle-tested |
| `dtMarkBoxArea(layer, orig, cs, ch, bmin, bmax, areaId)` | `markBoxArea` | oracle-tested |
| `dtMarkBoxArea(layer, orig, cs, ch, center, halfExtents, rotAux, areaId)` | `markOrientedBoxArea` | oracle-tested |
| `dtBuildTileCacheRegions` | `buildRegions` | oracle-tested |
| `dtBuildTileCacheContours` | `buildContours` | oracle-tested |
| `dtBuildTileCachePolyMesh` | `buildPolyMesh` | oracle-tested |
| `dtTileCacheHeaderSwapEndian` | `swapEndianHeader` | oracle-tested |

## The integer variant

Not in upstream: the same queries over whole-millimetre coordinates, for a server that
must give the same answer on every machine. `doc/integer-design.md` gives its units,
scales, overflow bounds and tolerances. Here **built** means it compiles and passes its
own unit tests, and **checked** means the tests compare it with the float port within the
design's tolerances.

| piece | Zig | state |
| --- | --- | --- |
| design: units, scales, overflow bounds, tolerances | `doc/integer-design.md` | written |
| vector and geometry helpers | `src/integer/common.zig` | built, checked through the queries |
| navigation mesh | `src/integer/nav_mesh.zig` | built, checked |
| tile conversion from the float format | `src/integer/convert.zig` | built, checked |
| node pool and queue | `src/integer/node.zig` | built, checked through the searches |
| queries, every one of `NavMeshQuery` | `src/integer/nav_mesh_query.zig` | built, checked |
| checked against the float port within the stated tolerances | `test/integer_test.zig`, `test/integer_hand_test.zig` | done: 4 levels, about 1,200 seeded cases per query, hand-made meshes |
| no overflow across the whole coordinate range | `test/integer_hand_test.zig` | done |
| no float in the variant's source | `test/integer_hand_test.zig` | done |
| exact self-determinism: twice, on 1, 2 and N threads, a hash kept in the test | `test/integer_hand_test.zig` | done, the same hash in Debug, ReleaseSafe and ReleaseFast |
| crowd and tile cache over integers | | left out, see `doc/integer-design.md` |

## Next

The performance pass and the skeptic are done (branches `review/perf-and-skeptic`,
`review/skeptic`). The fixes are on `review/fixes`:

- done: every bench row calls each timed function on both sides through `opaqueFn`
  (`tools/bench.zig`); the integer tests' limits are rates set just above the worst
  over fuzz seeds 0 to 31, Debug and ReleaseSafe the same, the measured worst beside
  each; findPath partial results that stop at other polygons are held by their distance
  to the end; the walkability check of findStraightPath steps past a ray along a wall
  at 45 degrees (test of the seed-8 leg); crossings compared one by one under the
  design's rules; `tileInUnits` and a test of `tileFromFloat` at other scales; two
  oracle tests that failed under seeds 12 and 23 were in error and are fixed (see
  `doc/TESTING.md`). Every other test passes on seeds 0 to 31, Debug and ReleaseSafe.
- open, for the author: five cases need a tolerance or rule the design does not state
  and fail under some seeds of 1 to 31 (`doc/TESTING.md` lists them with their seeds):
  raycast's cost on grazing hits, moveAlongSurface's visited list at a shared-edge
  tie, findDistanceToWall's normal between two near walls, findStraightPath's end at
  an edge-vertex tie (seed 3), and findPath on seed 13 (integer corridor 2.7% cheaper).
- to do: a quiet benchmark (three runs pinned with `taskset -c 6`, load under 3; the
  machine stayed above that for over an hour on 11 November), then the tables of
  `doc/BENCHMARKS.md` (still from before the opaque calls; "Known biases" quotes
  findRandomPoint at 0.68 from a one-round run), the README's headline table in percent
  ("N% faster" or "N% slower" than the original, ratios only in `doc/BENCHMARKS.md`),
  and the "Where the port is slower" text (`closestHeightOnTriangle` is no longer
  inlined into its row).
- release, once every test passes on seeds 0 to 31 in Debug and ReleaseSafe: merge
  `review/fixes` into `main`, mark this file's entries reviewed, put `#v0.1.0` in the
  README's install line, tag `v0.1.0` (annotated) and push `main` and the tag.
