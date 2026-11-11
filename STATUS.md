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
| Detour | DetourNavMeshBuilder.h | `src/nav_mesh_builder.zig` | 4 |  | 2 | 2 |  |  |
| Detour | DetourNavMeshQuery.h | `src/nav_mesh_query.zig` | 45 |  | 44 | 1 |  |  |
| Detour | DetourNode.h | `src/node.zig` | 33 |  | 33 |  |  |  |
| Detour | DetourCommon.h | `src/common.zig` | 44 |  |  | 44 |  |  |
| Detour | DetourStatus.h | `src/status.zig` | 17 |  |  | 17 |  |  |
| Detour | DetourMath.h | `src/common.zig` | 1 |  |  | 1 |  |  |
| DetourCrowd | DetourCrowd.h | `src/crowd.zig` | 37 |  | 37 |  |  |  |
| DetourCrowd | DetourPathCorridor.h | `src/path_corridor.zig` | 24 |  | 24 |  |  |  |
| DetourCrowd | DetourPathQueue.h | `src/path_queue.zig` | 11 |  | 11 |  |  |  |
| DetourCrowd | DetourLocalBoundary.h | `src/local_boundary.zig` | 8 |  | 8 |  |  |  |
| DetourCrowd | DetourObstacleAvoidance.h | `src/obstacle_avoidance.zig` | 33 |  | 33 |  |  |  |
| DetourCrowd | DetourProximityGrid.h | `src/proximity_grid.zig` | 10 |  |  | 10 |  |  |
| DetourTileCache | DetourTileCache.h | `src/tile_cache.zig` | 49 |  | 49 |  |  |  |
| DetourTileCache | DetourTileCacheBuilder.h | `src/tile_cache_builder.zig` | 30 |  | 30 |  |  |  |
| | | **all** | 415 |  | 271 | 144 |  |  |

The differential tests run twice: with every build switch off, and with every one on.

## Build switches

| upstream | Zig | state |
| --- | --- | --- |
| `DT_POLYREF64` (`RECASTNAVIGATION_DT_POLYREF64`): 64-bit polygon and tile references | `-Dpolyref64` | oracle-tested |
| `DT_VIRTUAL_QUERYFILTER` (`RECASTNAVIGATION_DT_VIRTUAL_QUERYFILTER`): a filter replaces its pass and cost functions | `-Dvirtual-query-filter` | ported |
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
| `dtNavMeshHeaderSwapEndian` | `navMeshHeaderSwapEndian` | ported |
| `dtNavMeshDataSwapEndian` | `navMeshDataSwapEndian` | ported |

### DetourNavMeshQuery.h → `src/nav_mesh_query.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtQueryFilter` | `QueryFilter` | ported |
| `dtQueryFilter::dtQueryFilter` | `QueryFilter`, its default field values | ported |
| `dtQueryFilter::passFilter` | `QueryFilter.passFilter` | ported |
| `dtQueryFilter::getCost` | `QueryFilter.getCost` | ported |
| `dtQueryFilter::getAreaCost` | `QueryFilter`, the `area_cost` field | ported |
| `dtQueryFilter::setAreaCost` | `QueryFilter`, the `area_cost` field | ported |
| `dtQueryFilter::getIncludeFlags` | `QueryFilter`, the `include_flags` field | ported |
| `dtQueryFilter::setIncludeFlags` | `QueryFilter`, the `include_flags` field | ported |
| `dtQueryFilter::getExcludeFlags` | `QueryFilter`, the `exclude_flags` field | ported |
| `dtQueryFilter::setExcludeFlags` | `QueryFilter`, the `exclude_flags` field | ported |
| `dtRaycastHit`: `t`, `hitNormal`, `hitEdgeIndex`, `path`, `pathCount`, `maxPath`, `pathCost` | `RaycastHit` | oracle-tested |
| `dtPolyQuery` | `PolyQuery` | ported |
| `dtPolyQuery::process` | `PolyQuery.process` | ported |
| `dtNavMeshQuery` | `NavMeshQuery` | ported |
| `dtNavMeshQuery::dtNavMeshQuery` | `NavMeshQuery.empty` | ported |
| `dtNavMeshQuery::~dtNavMeshQuery` | `NavMeshQuery.deinit` | ported |
| `dtNavMeshQuery::init` | `NavMeshQuery.init` | ported |
| `dtNavMeshQuery::findPath` | `NavMeshQuery.findPath` | ported |
| `dtNavMeshQuery::findStraightPath` | `NavMeshQuery.findStraightPath` | ported |
| `dtNavMeshQuery::initSlicedFindPath` | `NavMeshQuery.initSlicedFindPath` | ported |
| `dtNavMeshQuery::updateSlicedFindPath` | `NavMeshQuery.updateSlicedFindPath` | ported |
| `dtNavMeshQuery::finalizeSlicedFindPath` | `NavMeshQuery.finalizeSlicedFindPath` | ported |
| `dtNavMeshQuery::finalizeSlicedFindPathPartial` | `NavMeshQuery.finalizeSlicedFindPathPartial` | ported |
| `dtNavMeshQuery::findPolysAroundCircle` | `NavMeshQuery.findPolysAroundCircle` | ported |
| `dtNavMeshQuery::findPolysAroundShape` | `NavMeshQuery.findPolysAroundShape` | ported |
| `dtNavMeshQuery::getPathFromDijkstraSearch` | `NavMeshQuery.getPathFromDijkstraSearch` | ported |
| `dtNavMeshQuery::findNearestPoly(center, halfExtents, filter, nearestRef, nearestPt)` | `NavMeshQuery.findNearestPoly`, `is_over_poly` null | ported |
| `dtNavMeshQuery::findNearestPoly(center, halfExtents, filter, nearestRef, nearestPt, isOverPoly)` | `NavMeshQuery.findNearestPoly` | ported |
| `dtNavMeshQuery::queryPolygons(center, halfExtents, filter, polys, polyCount, maxPolys)` | `NavMeshQuery.queryPolygons` | ported |
| `dtNavMeshQuery::queryPolygons(center, halfExtents, filter, query)` | `NavMeshQuery.queryPolygonsWith` | ported |
| `dtNavMeshQuery::findLocalNeighbourhood` | `NavMeshQuery.findLocalNeighbourhood` | ported |
| `dtNavMeshQuery::moveAlongSurface` | `NavMeshQuery.moveAlongSurface` | ported |
| `dtNavMeshQuery::raycast(startRef, startPos, endPos, filter, t, hitNormal, path, pathCount, maxPath)` | `NavMeshQuery.raycastPath` | ported |
| `dtNavMeshQuery::raycast(startRef, startPos, endPos, filter, options, hit, prevRef)` | `NavMeshQuery.raycast` | ported |
| `dtNavMeshQuery::findDistanceToWall` | `NavMeshQuery.findDistanceToWall` | ported |
| `dtNavMeshQuery::getPolyWallSegments` | `NavMeshQuery.getPolyWallSegments` | ported |
| `dtNavMeshQuery::findRandomPoint` | `NavMeshQuery.findRandomPoint` | ported |
| `dtNavMeshQuery::findRandomPointAroundCircle` | `NavMeshQuery.findRandomPointAroundCircle` | ported |
| `dtNavMeshQuery::closestPointOnPoly` | `NavMeshQuery.closestPointOnPoly` | ported |
| `dtNavMeshQuery::closestPointOnPolyBoundary` | `NavMeshQuery.closestPointOnPolyBoundary` | ported |
| `dtNavMeshQuery::getPolyHeight` | `NavMeshQuery.getPolyHeight` | ported |
| `dtNavMeshQuery::isValidPolyRef` | `NavMeshQuery.isValidPolyRef` | ported |
| `dtNavMeshQuery::isInClosedList` | `NavMeshQuery.isInClosedList` | ported |
| `dtNavMeshQuery::getNodePool` | `NavMeshQuery.getNodePool` | ported |
| `dtNavMeshQuery::getAttachedNavMesh` | `NavMeshQuery.getAttachedNavMesh` | ported |

### DetourNode.h → `src/node.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtNodeFlags`: `DT_NODE_OPEN`, `DT_NODE_CLOSED`, `DT_NODE_PARENT_DETACHED` | `NodeFlags` | ported |
| `dtNodeIndex` | `NodeIndex` | ported |
| `DT_NULL_IDX` | `null_index` | ported |
| `DT_NODE_PARENT_BITS` | `node_parent_bits` | ported |
| `DT_NODE_STATE_BITS` | `node_state_bits` | ported |
| `dtNode`: `pos`, `cost`, `total`, `DT_NODE_PARENT_BITS`, `DT_NODE_STATE_BITS`, `3`, `id` | `Node` | ported |
| `DT_MAX_STATES_PER_NODE` | `node_states_max` | ported |
| `dtNodePool` | `NodePool` | ported |
| `dtNodePool::dtNodePool` | `NodePool.init` | ported |
| `dtNodePool::~dtNodePool` | `NodePool.deinit` | ported |
| `dtNodePool::clear` | `NodePool.clear` | ported |
| `dtNodePool::getNode` | `NodePool.getNode` | ported |
| `dtNodePool::findNode` | `NodePool.findNode` | ported |
| `dtNodePool::findNodes` | `NodePool.findNodes` | ported |
| `dtNodePool::getNodeIdx` | `NodePool.getNodeIdx` | ported |
| `dtNodePool::getNodeAtIdx` | `NodePool.getNodeAtIdx` | ported |
| `dtNodePool::getMemUsed` | `NodePool.getMemUsed` | ported |
| `dtNodePool::getMaxNodes` | `NodePool.getMaxNodes` | ported |
| `dtNodePool::getHashSize` | `NodePool.getHashSize` | ported |
| `dtNodePool::getFirst` | `NodePool.getFirst` | ported |
| `dtNodePool::getNext` | `NodePool.getNext` | ported |
| `dtNodePool::getNodeCount` | `NodePool.getNodeCount` | ported |
| `dtNodeQueue` | `NodeQueue` | ported |
| `dtNodeQueue::dtNodeQueue` | `NodeQueue.init` | ported |
| `dtNodeQueue::~dtNodeQueue` | `NodeQueue.deinit` | ported |
| `dtNodeQueue::clear` | `NodeQueue.clear` | ported |
| `dtNodeQueue::top` | `NodeQueue.top` | ported |
| `dtNodeQueue::pop` | `NodeQueue.pop` | ported |
| `dtNodeQueue::push` | `NodeQueue.push` | ported |
| `dtNodeQueue::modify` | `NodeQueue.modify` | ported |
| `dtNodeQueue::empty` | `NodeQueue.isEmpty` | ported |
| `dtNodeQueue::getMemUsed` | `NodeQueue.getMemUsed` | ported |
| `dtNodeQueue::getCapacity` | `NodeQueue.getCapacity` | ported |

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
| `DT_CROWDAGENT_MAX_NEIGHBOURS` | `agent_neighbour_count_max` | ported |
| `DT_CROWDAGENT_MAX_CORNERS` | `agent_corner_count_max` | ported |
| `DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS` | `obstacle_avoidance_params_count_max` | ported |
| `DT_CROWD_MAX_QUERY_FILTER_TYPE` | `query_filter_type_count_max` | ported |
| `dtCrowdNeighbour`: `idx`, `dist` | `CrowdNeighbour` | ported |
| `CrowdAgentState`: `DT_CROWDAGENT_STATE_INVALID`, `DT_CROWDAGENT_STATE_WALKING`, `DT_CROWDAGENT_STATE_OFFMESH` | `CrowdAgentTraversal` | ported |
| `dtCrowdAgentParams`: `radius`, `height`, `maxAcceleration`, `maxSpeed`, `collisionQueryRange`, `pathOptimizationRange`, `separationWeight`, `updateFlags`, `obstacleAvoidanceType`, `queryFilterType`, `userData` | `CrowdAgentParams` | ported |
| `MoveRequestState`: `DT_CROWDAGENT_TARGET_NONE`, `DT_CROWDAGENT_TARGET_FAILED`, `DT_CROWDAGENT_TARGET_VALID`, `DT_CROWDAGENT_TARGET_REQUESTING`, `DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE`, `DT_CROWDAGENT_TARGET_WAITING_FOR_PATH`, `DT_CROWDAGENT_TARGET_VELOCITY` | `MoveRequestStage` | ported |
| `dtCrowdAgent`: `active`, `state`, `partial`, `corridor`, `boundary`, `topologyOptTime`, `neis`, `nneis`, `desiredSpeed`, `npos`, `disp`, `dvel`, `nvel`, `vel`, `params`, `cornerVerts`, `cornerFlags`, `cornerPolys`, `ncorners`, `targetState`, `targetRef`, `targetPos`, `targetPathqRef`, `targetReplan`, `targetReplanTime` | `CrowdAgent` | ported |
| `dtCrowdAgentAnimation`: `active`, `initPos`, `startPos`, `endPos`, `polyRef`, `t`, `tmax` | `CrowdAgentAnimation` | ported |
| `UpdateFlags`: `DT_CROWD_ANTICIPATE_TURNS`, `DT_CROWD_OBSTACLE_AVOIDANCE`, `DT_CROWD_SEPARATION`, `DT_CROWD_OPTIMIZE_VIS`, `DT_CROWD_OPTIMIZE_TOPO` | `UpdateFlags` | ported |
| `dtCrowdAgentDebugInfo`: `idx`, `optStart`, `optEnd`, `vod` | `CrowdAgentDebugInfo` | ported |
| `dtCrowd` | `Crowd` | ported |
| `dtCrowd::dtCrowd` | `Crowd.empty` | ported |
| `dtCrowd::~dtCrowd` | `Crowd.deinit` | ported |
| `dtCrowd::init` | `Crowd.init` | ported |
| `dtCrowd::setObstacleAvoidanceParams` | `Crowd.setObstacleAvoidanceParams` | ported |
| `dtCrowd::getObstacleAvoidanceParams` | `Crowd.getObstacleAvoidanceParams` | ported |
| `dtCrowd::getAgent` | `Crowd.getAgent` | ported |
| `dtCrowd::getEditableAgent` | `Crowd.getEditableAgent` | ported |
| `dtCrowd::getAgentCount` | `Crowd.getAgentCount` | ported |
| `dtCrowd::addAgent` | `Crowd.addAgent` | ported |
| `dtCrowd::updateAgentParameters` | `Crowd.updateAgentParameters` | ported |
| `dtCrowd::removeAgent` | `Crowd.removeAgent` | ported |
| `dtCrowd::requestMoveTarget` | `Crowd.requestMoveTarget` | ported |
| `dtCrowd::requestMoveVelocity` | `Crowd.requestMoveVelocity` | ported |
| `dtCrowd::resetMoveTarget` | `Crowd.resetMoveTarget` | ported |
| `dtCrowd::getActiveAgents` | `Crowd.getActiveAgents` | ported |
| `dtCrowd::update` | `Crowd.update` | ported |
| `dtCrowd::getFilter` | `Crowd.getFilter` | ported |
| `dtCrowd::getEditableFilter` | `Crowd.getEditableFilter` | ported |
| `dtCrowd::getQueryHalfExtents` | `Crowd.getQueryHalfExtents` | ported |
| `dtCrowd::getQueryExtents` | `Crowd.getQueryExtents` | ported |
| `dtCrowd::getVelocitySampleCount` | `Crowd.getVelocitySampleCount` | ported |
| `dtCrowd::getGrid` | `Crowd.getGrid` | ported |
| `dtCrowd::getPathQueue` | `Crowd.getPathQueue` | ported |
| `dtCrowd::getNavMeshQuery` | `Crowd.getNavMeshQuery` | ported |

### DetourPathCorridor.h → `src/path_corridor.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtPathCorridor` | `PathCorridor` | ported |
| `dtPathCorridor::dtPathCorridor` | `PathCorridor.empty` | ported |
| `dtPathCorridor::~dtPathCorridor` | `PathCorridor.deinit` | ported |
| `dtPathCorridor::init` | `PathCorridor.init` | ported |
| `dtPathCorridor::reset` | `PathCorridor.reset` | ported |
| `dtPathCorridor::findCorners` | `PathCorridor.findCorners` | ported |
| `dtPathCorridor::optimizePathVisibility` | `PathCorridor.optimizePathVisibility` | ported |
| `dtPathCorridor::optimizePathTopology` | `PathCorridor.optimizePathTopology` | ported |
| `dtPathCorridor::moveOverOffmeshConnection` | `PathCorridor.moveOverOffmeshConnection` | ported |
| `dtPathCorridor::fixPathStart` | `PathCorridor.fixPathStart` | ported |
| `dtPathCorridor::trimInvalidPath` | `PathCorridor.trimInvalidPath` | ported |
| `dtPathCorridor::isValid` | `PathCorridor.isValid` | ported |
| `dtPathCorridor::movePosition` | `PathCorridor.movePosition` | ported |
| `dtPathCorridor::moveTargetPosition` | `PathCorridor.moveTargetPosition` | ported |
| `dtPathCorridor::setCorridor` | `PathCorridor.setCorridor` | ported |
| `dtPathCorridor::getPos` | `PathCorridor.getPos` | ported |
| `dtPathCorridor::getTarget` | `PathCorridor.getTarget` | ported |
| `dtPathCorridor::getFirstPoly` | `PathCorridor.getFirstPoly` | ported |
| `dtPathCorridor::getLastPoly` | `PathCorridor.getLastPoly` | ported |
| `dtPathCorridor::getPath` | `PathCorridor.getPath` | ported |
| `dtPathCorridor::getPathCount` | `PathCorridor.getPathCount` | ported |
| `dtMergeCorridorStartMoved` | `mergeCorridorStartMoved` | ported |
| `dtMergeCorridorEndMoved` | `mergeCorridorEndMoved` | ported |
| `dtMergeCorridorStartShortcut` | `mergeCorridorStartShortcut` | ported |

### DetourPathQueue.h → `src/path_queue.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `DT_PATHQ_INVALID` | `path_queue_invalid` | ported |
| `dtPathQueueRef` | `PathQueueRef` | ported |
| `dtPathQueue` | `PathQueue` | ported |
| `dtPathQueue::dtPathQueue` | `PathQueue.empty` | ported |
| `dtPathQueue::~dtPathQueue` | `PathQueue.deinit` | ported |
| `dtPathQueue::init` | `PathQueue.init` | ported |
| `dtPathQueue::update` | `PathQueue.update` | ported |
| `dtPathQueue::request` | `PathQueue.request` | ported |
| `dtPathQueue::getRequestStatus` | `PathQueue.getRequestStatus` | ported |
| `dtPathQueue::getPathResult` | `PathQueue.getPathResult` | ported |
| `dtPathQueue::getNavQuery` | `PathQueue.getNavQuery` | ported |

### DetourLocalBoundary.h → `src/local_boundary.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtLocalBoundary` | `LocalBoundary` | ported |
| `dtLocalBoundary::dtLocalBoundary` | `LocalBoundary.empty` | ported |
| `dtLocalBoundary::reset` | `LocalBoundary.reset` | ported |
| `dtLocalBoundary::update` | `LocalBoundary.update` | ported |
| `dtLocalBoundary::isValid` | `LocalBoundary.isValid` | ported |
| `dtLocalBoundary::getCenter` | `LocalBoundary.getCenter` | ported |
| `dtLocalBoundary::getSegmentCount` | `LocalBoundary.getSegmentCount` | ported |
| `dtLocalBoundary::getSegment` | `LocalBoundary.getSegment` | ported |

### DetourObstacleAvoidance.h → `src/obstacle_avoidance.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `dtObstacleCircle`: `p`, `vel`, `dvel`, `rad`, `dp`, `np` | `ObstacleCircle` | ported |
| `dtObstacleSegment`: `p`, `q`, `touch` | `ObstacleSegment` | ported |
| `dtObstacleAvoidanceDebugData` | `ObstacleAvoidanceDebug` | ported |
| `dtObstacleAvoidanceDebugData::dtObstacleAvoidanceDebugData` | `ObstacleAvoidanceDebug.empty` | ported |
| `dtObstacleAvoidanceDebugData::~dtObstacleAvoidanceDebugData` | `ObstacleAvoidanceDebug.deinit` | ported |
| `dtObstacleAvoidanceDebugData::init` | `ObstacleAvoidanceDebug.init` | ported |
| `dtObstacleAvoidanceDebugData::reset` | `ObstacleAvoidanceDebug.reset` | ported |
| `dtObstacleAvoidanceDebugData::addSample` | `ObstacleAvoidanceDebug.addSample` | ported |
| `dtObstacleAvoidanceDebugData::normalizeSamples` | `ObstacleAvoidanceDebug.normalizeSamples` | ported |
| `dtObstacleAvoidanceDebugData::getSampleCount` | `ObstacleAvoidanceDebug.getSampleCount` | ported |
| `dtObstacleAvoidanceDebugData::getSampleVelocity` | `ObstacleAvoidanceDebug.getSampleVelocity` | ported |
| `dtObstacleAvoidanceDebugData::getSampleSize` | `ObstacleAvoidanceDebug.getSampleSize` | ported |
| `dtObstacleAvoidanceDebugData::getSamplePenalty` | `ObstacleAvoidanceDebug.getSamplePenalty` | ported |
| `dtObstacleAvoidanceDebugData::getSampleDesiredVelocityPenalty` | `ObstacleAvoidanceDebug.getSampleDesiredVelocityPenalty` | ported |
| `dtObstacleAvoidanceDebugData::getSampleCurrentVelocityPenalty` | `ObstacleAvoidanceDebug.getSampleCurrentVelocityPenalty` | ported |
| `dtObstacleAvoidanceDebugData::getSamplePreferredSidePenalty` | `ObstacleAvoidanceDebug.getSamplePreferredSidePenalty` | ported |
| `dtObstacleAvoidanceDebugData::getSampleCollisionTimePenalty` | `ObstacleAvoidanceDebug.getSampleCollisionTimePenalty` | ported |
| `DT_MAX_PATTERN_DIVS` | `pattern_division_count_max` | ported |
| `DT_MAX_PATTERN_RINGS` | `pattern_ring_count_max` | ported |
| `dtObstacleAvoidanceParams`: `velBias`, `weightDesVel`, `weightCurVel`, `weightSide`, `weightToi`, `horizTime`, `gridSize`, `adaptiveDivs`, `adaptiveRings`, `adaptiveDepth` | `ObstacleAvoidanceParams` | ported |
| `dtObstacleAvoidanceQuery` | `ObstacleAvoidanceQuery` | ported |
| `dtObstacleAvoidanceQuery::dtObstacleAvoidanceQuery` | `ObstacleAvoidanceQuery.empty` | ported |
| `dtObstacleAvoidanceQuery::~dtObstacleAvoidanceQuery` | `ObstacleAvoidanceQuery.deinit` | ported |
| `dtObstacleAvoidanceQuery::init` | `ObstacleAvoidanceQuery.init` | ported |
| `dtObstacleAvoidanceQuery::reset` | `ObstacleAvoidanceQuery.reset` | ported |
| `dtObstacleAvoidanceQuery::addCircle` | `ObstacleAvoidanceQuery.addCircle` | ported |
| `dtObstacleAvoidanceQuery::addSegment` | `ObstacleAvoidanceQuery.addSegment` | ported |
| `dtObstacleAvoidanceQuery::sampleVelocityGrid` | `ObstacleAvoidanceQuery.sampleVelocityGrid` | ported |
| `dtObstacleAvoidanceQuery::sampleVelocityAdaptive` | `ObstacleAvoidanceQuery.sampleVelocityAdaptive` | ported |
| `dtObstacleAvoidanceQuery::getObstacleCircleCount` | `ObstacleAvoidanceQuery.getObstacleCircleCount` | ported |
| `dtObstacleAvoidanceQuery::getObstacleCircle` | `ObstacleAvoidanceQuery.getObstacleCircle` | ported |
| `dtObstacleAvoidanceQuery::getObstacleSegmentCount` | `ObstacleAvoidanceQuery.getObstacleSegmentCount` | ported |
| `dtObstacleAvoidanceQuery::getObstacleSegment` | `ObstacleAvoidanceQuery.getObstacleSegment` | ported |

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
| `dtObstacleRef` | `ObstacleRef` | ported |
| `dtCompressedTileRef` | `CompressedTileRef` | ported |
| `dtCompressedTileFlags`: `DT_COMPRESSEDTILE_FREE_DATA` | `compressed_tile_free_data` | ported |
| `dtCompressedTile`: `salt`, `header`, `compressed`, `compressedSize`, `data`, `dataSize`, `flags`, `next` | `CompressedTile` | ported |
| `ObstacleState`: `DT_OBSTACLE_EMPTY`, `DT_OBSTACLE_PROCESSING`, `DT_OBSTACLE_PROCESSED`, `DT_OBSTACLE_REMOVING` | `ObstaclePhase` | ported |
| `ObstacleType`: `DT_OBSTACLE_CYLINDER`, `DT_OBSTACLE_BOX`, `DT_OBSTACLE_ORIENTED_BOX` | `ObstacleType` | ported |
| `dtObstacleCylinder`: `pos`, `radius`, `height` | `ObstacleCylinder` | ported |
| `dtObstacleBox`: `bmin`, `bmax` | `ObstacleBox` | ported |
| `dtObstacleOrientedBox`: `center`, `halfExtents`, `rotAux` | `ObstacleOrientedBox` | ported |
| `DT_MAX_TOUCHED_TILES` | `touched_tile_count_max` | ported |
| `dtTileCacheObstacle`: `touched`, `pending`, `salt`, `type`, `state`, `ntouched`, `npending`, `next` | `Obstacle` | ported |
| `dtTileCacheParams`: `orig`, `cs`, `ch`, `width`, `height`, `walkableHeight`, `walkableRadius`, `walkableClimb`, `maxSimplificationError`, `maxTiles`, `maxObstacles` | `TileCacheParams` | ported |
| `dtTileCacheMeshProcess` | `MeshProcess` | ported |
| `dtTileCacheMeshProcess::process` | `MeshProcess.process` | ported |
| `dtTileCache` | `TileCache` | ported |
| `dtTileCache::dtTileCache` | `TileCache.empty` | ported |
| `dtTileCache::~dtTileCache` | `TileCache.deinit` | ported |
| `dtTileCache::getAlloc` | `TileCache.getAlloc` | ported |
| `dtTileCache::getCompressor` | `TileCache.getCompressor` | ported |
| `dtTileCache::getParams` | `TileCache.getParams` | ported |
| `dtTileCache::getTileCount` | `TileCache.getTileCount` | ported |
| `dtTileCache::getTile` | `TileCache.getTile` | ported |
| `dtTileCache::getObstacleCount` | `TileCache.getObstacleCount` | ported |
| `dtTileCache::getObstacle` | `TileCache.getObstacle` | ported |
| `dtTileCache::getObstacleByRef` | `TileCache.getObstacleByRef` | ported |
| `dtTileCache::getObstacleRef` | `TileCache.getObstacleRef` | ported |
| `dtTileCache::init` | `TileCache.init` | ported |
| `dtTileCache::getTilesAt` | `TileCache.getTilesAt` | ported |
| `dtTileCache::getTileAt` | `TileCache.getTileAt` | ported |
| `dtTileCache::getTileRef` | `TileCache.getTileRef` | ported |
| `dtTileCache::getTileByRef` | `TileCache.getTileByRef` | ported |
| `dtTileCache::addTile` | `TileCache.addTile` | ported |
| `dtTileCache::removeTile` | `TileCache.removeTile` | ported |
| `dtTileCache::addObstacle` | `TileCache.addObstacle` | ported |
| `dtTileCache::addBoxObstacle(bmin, bmax, result)` | `TileCache.addBoxObstacle` | ported |
| `dtTileCache::addBoxObstacle(center, halfExtents, yRadians, result)` | `TileCache.addOrientedBoxObstacle` | ported |
| `dtTileCache::removeObstacle` | `TileCache.removeObstacle` | ported |
| `dtTileCache::queryTiles` | `TileCache.queryTiles` | ported |
| `dtTileCache::update` | `TileCache.update` | ported |
| `dtTileCache::buildNavMeshTilesAt` | `TileCache.buildNavMeshTilesAt` | ported |
| `dtTileCache::buildNavMeshTile` | `TileCache.buildNavMeshTile` | ported |
| `dtTileCache::calcTightTileBounds` | `TileCache.calcTightTileBounds` | ported |
| `dtTileCache::getObstacleBounds` | `TileCache.getObstacleBounds` | ported |
| `dtTileCache::encodeTileId` | `TileCache.encodeTileId` | ported |
| `dtTileCache::decodeTileIdSalt` | `TileCache.decodeTileIdSalt` | ported |
| `dtTileCache::decodeTileIdTile` | `TileCache.decodeTileIdTile` | ported |
| `dtTileCache::encodeObstacleId` | `TileCache.encodeObstacleId` | ported |
| `dtTileCache::decodeObstacleIdSalt` | `TileCache.decodeObstacleIdSalt` | ported |
| `dtTileCache::decodeObstacleIdObstacle` | `TileCache.decodeObstacleIdObstacle` | ported |

### DetourTileCacheBuilder.h → `src/tile_cache_builder.zig`

| upstream | Zig | state |
| --- | --- | --- |
| `DT_TILECACHE_MAGIC` | `tile_cache_magic` | ported |
| `DT_TILECACHE_VERSION` | `tile_cache_version` | ported |
| `DT_TILECACHE_NULL_AREA` | `null_area` | ported |
| `DT_TILECACHE_WALKABLE_AREA` | `walkable_area` | ported |
| `DT_TILECACHE_NULL_IDX` | `null_index` | ported |
| `dtTileCacheLayerHeader`: `magic`, `version`, `tx`, `ty`, `tlayer`, `bmin`, `bmax`, `hmin`, `hmax`, `width`, `height`, `minx`, `maxx`, `miny`, `maxy` | `LayerHeader` | ported |
| `dtTileCacheLayer`: `header`, `regCount`, `heights`, `areas`, `cons`, `regs` | `Layer` | ported |
| `dtTileCacheContour`: `nverts`, `verts`, `reg`, `area` | `Contour` | ported |
| `dtTileCacheContourSet`: `nconts`, `conts` | `ContourSet` | ported |
| `dtTileCachePolyMesh`: `nvp`, `nverts`, `npolys`, `verts`, `polys`, `flags`, `areas` | `PolyMesh` | ported |
| `dtTileCacheAlloc` | `BuildAllocator` | ported |
| `dtTileCacheAlloc::reset` | `BuildAllocator.reset` | ported |
| `dtTileCacheCompressor` | `Compressor` | ported |
| `dtTileCacheCompressor::maxCompressedSize` | `Compressor.maxCompressedSize` | ported |
| `dtTileCacheCompressor::compress` | `Compressor.compress` | ported |
| `dtTileCacheCompressor::decompress` | `Compressor.decompress` | ported |
| `dtBuildTileCacheLayer` | `buildLayer` | ported |
| `dtFreeTileCacheLayer` | `freeLayer` | ported |
| `dtDecompressTileCacheLayer` | `decompressLayer` | ported |
| `dtAllocTileCacheContourSet` | `allocContourSet` | ported |
| `dtFreeTileCacheContourSet` | `freeContourSet` | ported |
| `dtAllocTileCachePolyMesh` | `allocPolyMesh` | ported |
| `dtFreeTileCachePolyMesh` | `freePolyMesh` | ported |
| `dtMarkCylinderArea` | `markCylinderArea` | ported |
| `dtMarkBoxArea(layer, orig, cs, ch, bmin, bmax, areaId)` | `markBoxArea` | ported |
| `dtMarkBoxArea(layer, orig, cs, ch, center, halfExtents, rotAux, areaId)` | `markOrientedBoxArea` | ported |
| `dtBuildTileCacheRegions` | `buildRegions` | ported |
| `dtBuildTileCacheContours` | `buildContours` | ported |
| `dtBuildTileCachePolyMesh` | `buildPolyMesh` | ported |
| `dtTileCacheHeaderSwapEndian` | `swapEndianHeader` | ported |

## The integer variant

Not in upstream: the same queries over whole-millimetre coordinates, for a server that
must give the same answer on every machine. `doc/integer-design.md` gives its units,
scales, overflow bounds and tolerances.

| piece | Zig | state |
| --- | --- | --- |
| design: units, scales, overflow bounds, tolerances | `doc/integer-design.md` | not started |
| vector and geometry helpers | `src/integer/common.zig` | not started |
| navigation mesh, tile conversion from the float format | `src/integer/nav_mesh.zig` | not started |
| node pool and queue | `src/integer/node.zig` | not started |
| queries | `src/integer/nav_mesh_query.zig` | not started |
| checked against the float port within the stated tolerances | | not started |
| exact self-determinism, on 1, 2 and N threads | | not started |

## Next

1. Integrate the differential tests of the queries, the crowd and the tile cache.
