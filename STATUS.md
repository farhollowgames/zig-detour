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
| Detour | DetourNavMesh.h | `src/nav_mesh.zig` | 69 |  |  | 69 |  |  |
| Detour | DetourNavMeshBuilder.h | `src/nav_mesh_builder.zig` | 4 |  |  | 4 |  |  |
| Detour | DetourNavMeshQuery.h | `src/nav_mesh_query.zig` | 45 |  |  | 45 |  |  |
| Detour | DetourNode.h | `src/node.zig` | 33 |  |  | 33 |  |  |
| Detour | DetourCommon.h | `src/common.zig` | 44 |  |  | 44 |  |  |
| Detour | DetourStatus.h | `src/status.zig` | 17 |  |  | 17 |  |  |
| Detour | DetourMath.h | `src/common.zig` | 1 |  |  | 1 |  |  |
| DetourCrowd | DetourCrowd.h | `src/crowd.zig` | 37 |  |  | 37 |  |  |
| DetourCrowd | DetourPathCorridor.h | `src/path_corridor.zig` | 24 |  |  | 24 |  |  |
| DetourCrowd | DetourPathQueue.h | `src/path_queue.zig` | 11 |  |  | 11 |  |  |
| DetourCrowd | DetourLocalBoundary.h | `src/local_boundary.zig` | 8 |  |  | 8 |  |  |
| DetourCrowd | DetourObstacleAvoidance.h | `src/obstacle_avoidance.zig` | 33 |  |  | 33 |  |  |
| DetourCrowd | DetourProximityGrid.h | `src/proximity_grid.zig` | 10 |  |  | 10 |  |  |
| DetourTileCache | DetourTileCache.h | `src/tile_cache.zig` | 49 |  |  | 49 |  |  |
| DetourTileCache | DetourTileCacheBuilder.h | `src/tile_cache_builder.zig` | 30 |  |  | 30 |  |  |
| | | **all** | 415 |  |  | 415 |  |  |

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
| `dtNavMesh::addTile` | `NavMesh.addTile` | oracle-tested |
| `dtNavMesh::removeTile` | `NavMesh.removeTile` | oracle-tested |
| `dtNavMesh::calcTileLoc` | `NavMesh.calcTileLoc` | oracle-tested |
| `dtNavMesh::getTileAt` | `NavMesh.getTileAt` | oracle-tested |
| `dtNavMesh::getTilesAt` | `NavMesh.getTilesAt` | oracle-tested |
| `dtNavMesh::getTileRefAt` | `NavMesh.getTileRefAt` | oracle-tested |
| `dtNavMesh::getTileRef` | `NavMesh.getTileRef` | oracle-tested |
| `dtNavMesh::getTileByRef` | `NavMesh.getTileByRef` | oracle-tested |
| `dtNavMesh::getMaxTiles` | `NavMesh.getMaxTiles` | oracle-tested |
| `dtNavMesh::getTile` | `NavMesh.getTile` | oracle-tested |
| `dtNavMesh::getTileAndPolyByRef` | `NavMesh.getTileAndPolyByRef` | oracle-tested |
| `dtNavMesh::getTileAndPolyByRefUnsafe` | `NavMesh.getTileAndPolyByRefUnsafe` | oracle-tested |
| `dtNavMesh::isValidPolyRef` | `NavMesh.isValidPolyRef` | oracle-tested |
| `dtNavMesh::getPolyRefBase` | `NavMesh.getPolyRefBase` | oracle-tested |
| `dtNavMesh::getOffMeshConnectionPolyEndPoints` | `NavMesh.getOffMeshConnectionPolyEndPoints` | oracle-tested |
| `dtNavMesh::getOffMeshConnectionByRef` | `NavMesh.getOffMeshConnectionByRef` | oracle-tested |
| `dtNavMesh::setPolyFlags` | `NavMesh.setPolyFlags` | oracle-tested |
| `dtNavMesh::getPolyFlags` | `NavMesh.getPolyFlags` | oracle-tested |
| `dtNavMesh::setPolyArea` | `NavMesh.setPolyArea` | oracle-tested |
| `dtNavMesh::getPolyArea` | `NavMesh.getPolyArea` | oracle-tested |
| `dtNavMesh::getTileStateSize` | `NavMesh.getTileStateSize` | oracle-tested |
| `dtNavMesh::storeTileState` | `NavMesh.storeTileState` | oracle-tested |
| `dtNavMesh::restoreTileState` | `NavMesh.restoreTileState` | oracle-tested |
| `dtNavMesh::encodePolyId` | `NavMesh.encodePolyId` | oracle-tested |
| `dtNavMesh::decodePolyId` | `NavMesh.decodePolyId` | oracle-tested |
| `dtNavMesh::decodePolyIdSalt` | `NavMesh.decodePolyIdSalt` | oracle-tested |
| `dtNavMesh::decodePolyIdTile` | `NavMesh.decodePolyIdTile` | oracle-tested |
| `dtNavMesh::decodePolyIdPoly` | `NavMesh.decodePolyIdPoly` | oracle-tested |

### DetourNavMeshBuilder.h → `src/nav_mesh_builder.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtNavMeshCreateParams`: `verts`, `vertCount`, `polys`, `polyFlags`, `polyAreas`, `polyCount`, `nvp`, `detailMeshes`, `detailVerts`, `detailVertsCount`, `detailTris`, `detailTriCount`, `offMeshConVerts`, `offMeshConRad`, `offMeshConFlags`, `offMeshConAreas`, `offMeshConDir`, `offMeshConUserID`, `offMeshConCount`, `userId`, `tileX`, `tileY`, `tileLayer`, `bmin`, `bmax`, `walkableHeight`, `walkableRadius`, `walkableClimb`, `cs`, `ch`, `buildBvTree` | `NavMeshCreateParams` | oracle-tested |
| `dtCreateNavMeshData` | `createNavMeshData` | oracle-tested |
| `dtNavMeshHeaderSwapEndian` | `navMeshHeaderSwapEndian` | oracle-tested |
| `dtNavMeshDataSwapEndian` | `navMeshDataSwapEndian` | oracle-tested |

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
| `dtNavMeshQuery::findPath` | `NavMeshQuery.findPath` | oracle-tested |
| `dtNavMeshQuery::findStraightPath` | `NavMeshQuery.findStraightPath` | oracle-tested |
| `dtNavMeshQuery::initSlicedFindPath` | `NavMeshQuery.initSlicedFindPath` | oracle-tested |
| `dtNavMeshQuery::updateSlicedFindPath` | `NavMeshQuery.updateSlicedFindPath` | oracle-tested |
| `dtNavMeshQuery::finalizeSlicedFindPath` | `NavMeshQuery.finalizeSlicedFindPath` | oracle-tested |
| `dtNavMeshQuery::finalizeSlicedFindPathPartial` | `NavMeshQuery.finalizeSlicedFindPathPartial` | oracle-tested |
| `dtNavMeshQuery::findPolysAroundCircle` | `NavMeshQuery.findPolysAroundCircle` | oracle-tested |
| `dtNavMeshQuery::findPolysAroundShape` | `NavMeshQuery.findPolysAroundShape` | oracle-tested |
| `dtNavMeshQuery::getPathFromDijkstraSearch` | `NavMeshQuery.getPathFromDijkstraSearch` | oracle-tested |
| `dtNavMeshQuery::findNearestPoly(center, halfExtents, filter, nearestRef, nearestPt)` | `NavMeshQuery.findNearestPoly`, `is_over_poly` null | oracle-tested |
| `dtNavMeshQuery::findNearestPoly(center, halfExtents, filter, nearestRef, nearestPt, isOverPoly)` | `NavMeshQuery.findNearestPoly` | oracle-tested |
| `dtNavMeshQuery::queryPolygons(center, halfExtents, filter, polys, polyCount, maxPolys)` | `NavMeshQuery.queryPolygons` | oracle-tested |
| `dtNavMeshQuery::queryPolygons(center, halfExtents, filter, query)` | `NavMeshQuery.queryPolygonsWith` | oracle-tested |
| `dtNavMeshQuery::findLocalNeighbourhood` | `NavMeshQuery.findLocalNeighbourhood` | oracle-tested |
| `dtNavMeshQuery::moveAlongSurface` | `NavMeshQuery.moveAlongSurface` | oracle-tested |
| `dtNavMeshQuery::raycast(startRef, startPos, endPos, filter, t, hitNormal, path, pathCount, maxPath)` | `NavMeshQuery.raycastPath` | oracle-tested |
| `dtNavMeshQuery::raycast(startRef, startPos, endPos, filter, options, hit, prevRef)` | `NavMeshQuery.raycast` | oracle-tested |
| `dtNavMeshQuery::findDistanceToWall` | `NavMeshQuery.findDistanceToWall` | oracle-tested |
| `dtNavMeshQuery::getPolyWallSegments` | `NavMeshQuery.getPolyWallSegments` | oracle-tested |
| `dtNavMeshQuery::findRandomPoint` | `NavMeshQuery.findRandomPoint` | oracle-tested |
| `dtNavMeshQuery::findRandomPointAroundCircle` | `NavMeshQuery.findRandomPointAroundCircle` | oracle-tested |
| `dtNavMeshQuery::closestPointOnPoly` | `NavMeshQuery.closestPointOnPoly` | oracle-tested |
| `dtNavMeshQuery::closestPointOnPolyBoundary` | `NavMeshQuery.closestPointOnPolyBoundary` | oracle-tested |
| `dtNavMeshQuery::getPolyHeight` | `NavMeshQuery.getPolyHeight` | oracle-tested |
| `dtNavMeshQuery::isValidPolyRef` | `NavMeshQuery.isValidPolyRef` | oracle-tested |
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
| `dtClosestPtPointTriangle` | `closestPointOnTriangle` | oracle-tested |
| `dtClosestHeightPointTriangle` | `closestHeightOnTriangle` | oracle-tested |
| `dtIntersectSegmentPoly2D` | `intersectSegmentPoly2d` | oracle-tested |
| `dtIntersectSegSeg2D` | `intersectSegmentSegment2d` | oracle-tested |
| `dtPointInPolygon` | `pointInPolygon` | oracle-tested |
| `dtDistancePtPolyEdgesSqr` | `distancePointPolyEdgesSqr` | oracle-tested |
| `dtDistancePtSegSqr2D` | `distancePointSegmentSqr2d` | oracle-tested |
| `dtCalcPolyCenter` | `calcPolyCenter` | oracle-tested |
| `dtOverlapPolyPoly2D` | `overlapPolyPoly2d` | oracle-tested |
| `dtNextPow2` | `nextPow2` | oracle-tested |
| `dtIlog2` | `ilog2` | oracle-tested |
| `dtAlign4` | `align4` | oracle-tested |
| `dtOppositeTile` | `oppositeTile` | oracle-tested |
| `dtSwapEndian` for `float` | `swapEndianFloat` | oracle-tested |
| `dtRandomPointInConvexPoly` | `randomPointInConvexPoly` | oracle-tested |

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
| `dtCrowd::addAgent` | `Crowd.addAgent` | oracle-tested |
| `dtCrowd::updateAgentParameters` | `Crowd.updateAgentParameters` | oracle-tested |
| `dtCrowd::removeAgent` | `Crowd.removeAgent` | oracle-tested |
| `dtCrowd::requestMoveTarget` | `Crowd.requestMoveTarget` | oracle-tested |
| `dtCrowd::requestMoveVelocity` | `Crowd.requestMoveVelocity` | oracle-tested |
| `dtCrowd::resetMoveTarget` | `Crowd.resetMoveTarget` | oracle-tested |
| `dtCrowd::getActiveAgents` | `Crowd.getActiveAgents` | oracle-tested |
| `dtCrowd::update` | `Crowd.update` | oracle-tested |
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
| `dtPathCorridor::findCorners` | `PathCorridor.findCorners` | oracle-tested |
| `dtPathCorridor::optimizePathVisibility` | `PathCorridor.optimizePathVisibility` | oracle-tested |
| `dtPathCorridor::optimizePathTopology` | `PathCorridor.optimizePathTopology` | oracle-tested |
| `dtPathCorridor::moveOverOffmeshConnection` | `PathCorridor.moveOverOffmeshConnection` | oracle-tested |
| `dtPathCorridor::fixPathStart` | `PathCorridor.fixPathStart` | oracle-tested |
| `dtPathCorridor::trimInvalidPath` | `PathCorridor.trimInvalidPath` | oracle-tested |
| `dtPathCorridor::isValid` | `PathCorridor.isValid` | oracle-tested |
| `dtPathCorridor::movePosition` | `PathCorridor.movePosition` | oracle-tested |
| `dtPathCorridor::moveTargetPosition` | `PathCorridor.moveTargetPosition` | oracle-tested |
| `dtPathCorridor::setCorridor` | `PathCorridor.setCorridor` | oracle-tested |
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
| `dtPathQueue::update` | `PathQueue.update` | oracle-tested |
| `dtPathQueue::request` | `PathQueue.request` | oracle-tested |
| `dtPathQueue::getRequestStatus` | `PathQueue.getRequestStatus` | oracle-tested |
| `dtPathQueue::getPathResult` | `PathQueue.getPathResult` | oracle-tested |
| `dtPathQueue::getNavQuery` | `PathQueue.getNavQuery` | oracle-tested |

### DetourLocalBoundary.h → `src/local_boundary.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtLocalBoundary` | `LocalBoundary` | oracle-tested |
| `dtLocalBoundary::dtLocalBoundary` | `LocalBoundary.empty` | oracle-tested |
| `dtLocalBoundary::reset` | `LocalBoundary.reset` | oracle-tested |
| `dtLocalBoundary::update` | `LocalBoundary.update` | oracle-tested |
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
| `dtObstacleAvoidanceQuery::reset` | `ObstacleAvoidanceQuery.reset` | oracle-tested |
| `dtObstacleAvoidanceQuery::addCircle` | `ObstacleAvoidanceQuery.addCircle` | oracle-tested |
| `dtObstacleAvoidanceQuery::addSegment` | `ObstacleAvoidanceQuery.addSegment` | oracle-tested |
| `dtObstacleAvoidanceQuery::sampleVelocityGrid` | `ObstacleAvoidanceQuery.sampleVelocityGrid` | oracle-tested |
| `dtObstacleAvoidanceQuery::sampleVelocityAdaptive` | `ObstacleAvoidanceQuery.sampleVelocityAdaptive` | oracle-tested |
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
| `dtProximityGrid::clear` | `ProximityGrid.clear` | oracle-tested |
| `dtProximityGrid::addItem` | `ProximityGrid.addItem` | oracle-tested |
| `dtProximityGrid::queryItems` | `ProximityGrid.queryItems` | oracle-tested |
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
| `dtTileCache::addTile` | `TileCache.addTile` | oracle-tested |
| `dtTileCache::removeTile` | `TileCache.removeTile` | oracle-tested |
| `dtTileCache::addObstacle` | `TileCache.addObstacle` | oracle-tested |
| `dtTileCache::addBoxObstacle(bmin, bmax, result)` | `TileCache.addBoxObstacle` | oracle-tested |
| `dtTileCache::addBoxObstacle(center, halfExtents, yRadians, result)` | `TileCache.addOrientedBoxObstacle` | oracle-tested |
| `dtTileCache::removeObstacle` | `TileCache.removeObstacle` | oracle-tested |
| `dtTileCache::queryTiles` | `TileCache.queryTiles` | oracle-tested |
| `dtTileCache::update` | `TileCache.update` | oracle-tested |
| `dtTileCache::buildNavMeshTilesAt` | `TileCache.buildNavMeshTilesAt` | oracle-tested |
| `dtTileCache::buildNavMeshTile` | `TileCache.buildNavMeshTile` | oracle-tested |
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

1. `zig build bench`, and its tables in the README.
2. The performance pass, the skeptic and the fixes, by fresh agents.
