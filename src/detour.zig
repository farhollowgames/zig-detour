//! Detour, DetourCrowd and DetourTileCache in Zig: navigation meshes built
//! from polygon meshes, path and movement queries over them, crowds of
//! agents that steer around each other, and tile caches that rebuild tiles
//! around obstacles at run time.
//!
//! Every file is a namespace here; the types most code names are also at the
//! top level.

pub const common = @import("common.zig");
pub const status = @import("status.zig");
pub const nav_mesh = @import("nav_mesh.zig");
pub const nav_mesh_builder = @import("nav_mesh_builder.zig");
pub const node = @import("node.zig");
pub const nav_mesh_query = @import("nav_mesh_query.zig");
pub const proximity_grid = @import("proximity_grid.zig");
pub const obstacle_avoidance = @import("obstacle_avoidance.zig");
pub const local_boundary = @import("local_boundary.zig");
pub const path_corridor = @import("path_corridor.zig");
pub const path_queue = @import("path_queue.zig");
pub const crowd = @import("crowd.zig");
pub const tile_cache_builder = @import("tile_cache_builder.zig");
pub const tile_cache = @import("tile_cache.zig");

pub const Status = status.Status;

pub const PolyRef = nav_mesh.PolyRef;
pub const TileRef = nav_mesh.TileRef;
pub const NavMesh = nav_mesh.NavMesh;
pub const NavMeshParams = nav_mesh.NavMeshParams;
pub const MeshTile = nav_mesh.MeshTile;
pub const MeshHeader = nav_mesh.MeshHeader;
pub const Poly = nav_mesh.Poly;
pub const PolyType = nav_mesh.PolyType;
pub const TileFlags = nav_mesh.TileFlags;
pub const StraightPathFlags = nav_mesh.StraightPathFlags;
pub const StraightPathOptions = nav_mesh.StraightPathOptions;
pub const FindPathOptions = nav_mesh.FindPathOptions;
pub const RaycastOptions = nav_mesh.RaycastOptions;

pub const NavMeshCreateParams = nav_mesh_builder.NavMeshCreateParams;
pub const createNavMeshData = nav_mesh_builder.createNavMeshData;

pub const NavMeshQuery = nav_mesh_query.NavMeshQuery;
pub const QueryFilter = nav_mesh_query.QueryFilter;
pub const RaycastHit = nav_mesh_query.RaycastHit;
pub const PolyQuery = nav_mesh_query.PolyQuery;
pub const Node = node.Node;
pub const NodePool = node.NodePool;
pub const NodeQueue = node.NodeQueue;

pub const ProximityGrid = proximity_grid.ProximityGrid;
pub const ObstacleAvoidanceQuery = obstacle_avoidance.ObstacleAvoidanceQuery;
pub const ObstacleAvoidanceParams = obstacle_avoidance.ObstacleAvoidanceParams;
pub const ObstacleAvoidanceDebug = obstacle_avoidance.ObstacleAvoidanceDebug;
pub const LocalBoundary = local_boundary.LocalBoundary;
pub const PathCorridor = path_corridor.PathCorridor;
pub const PathQueue = path_queue.PathQueue;
pub const Crowd = crowd.Crowd;
pub const CrowdAgent = crowd.CrowdAgent;
pub const CrowdAgentParams = crowd.CrowdAgentParams;
pub const CrowdAgentDebugInfo = crowd.CrowdAgentDebugInfo;

pub const TileCache = tile_cache.TileCache;
pub const TileCacheParams = tile_cache.TileCacheParams;
pub const MeshProcess = tile_cache.MeshProcess;
pub const Compressor = tile_cache_builder.Compressor;
pub const BuildAllocator = tile_cache_builder.BuildAllocator;

test {
    _ = common;
    _ = status;
    _ = nav_mesh;
    _ = nav_mesh_builder;
    _ = node;
    _ = nav_mesh_query;
    _ = proximity_grid;
    _ = obstacle_avoidance;
    _ = local_boundary;
    _ = path_corridor;
    _ = path_queue;
    _ = crowd;
    _ = tile_cache_builder;
    _ = tile_cache;
}
