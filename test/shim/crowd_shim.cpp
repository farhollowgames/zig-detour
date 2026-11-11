// Test-only C ABI over upstream DetourCrowd (recastnavigation v1.6.0).
//
// A separate pure-Zig port of the crowd is tested against upstream through
// this file, byte for byte. Every function is `extern "C"`, prefixed `ref_`,
// and forwards its arguments unchanged in upstream's order (object pointer
// first). Statuses are `unsigned int`, booleans are `bool`. The Zig
// declarations live in test/reference_crowd.zig; keep the two in step.
//
// Departures from a pure forward, each forced by upstream's C++ shape:
//   - dtLocalBoundary, dtPathCorridor and dtPathQueue have no alloc/free
//     functions upstream, so ref_*_create builds them with dtAlloc and
//     placement new and ref_*_destroy runs the destructor and dtFree.
//   - getObstacleCircle and getObstacleSegment copy the struct out through
//     RefObstacleCircle / RefObstacleSegment instead of returning a pointer.
//   - getActiveAgents returns agent indices in an int buffer instead of
//     agent pointers.
//   - update takes the active-agent index to trace plus a RefCrowdDebugInfo
//     and builds the dtCrowdAgentDebugInfo itself.
//   - dtCrowdAgent holds C++ objects, so ref_crowd_agent_snapshot flattens
//     everything observable about one agent into RefAgentSnapshot.
//
// Pointers that getters return point into the object asked and go stale when
// that object is freed, re-initialised or changed; reference_crowd.zig says
// which. Pointers an object stores (a filter given to a path request, the
// navigation mesh given to init) must outlive it.
//
// Build notes (done by the integrator, never by this file):
//   - compiled with the Detour, DetourCrowd, DetourTileCache and Recast sources
//   - flags: -std=c++17 -ffp-contract=off -fno-fast-math -fno-exceptions
//     -fno-rtti -fno-threadsafe-statics
//   - must also compile with -DDT_POLYREF64 and/or -DDT_VIRTUAL_QUERYFILTER,
//     so refs always use the dtPolyRef / dtPathQueueRef typedefs

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <new>

#include "DetourAlloc.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourObstacleAvoidance.h"
#include "DetourProximityGrid.h"
#include "DetourLocalBoundary.h"
#include "DetourPathCorridor.h"
#include "DetourPathQueue.h"
#include "DetourCrowd.h"

// ---------------------------------------------------------------------------
// Structs that cross the boundary. reference_crowd.zig mirrors each one field
// for field, in this order.
// ---------------------------------------------------------------------------

// dtCrowd::init gives every agent's corridor room for this many polygons.
static const int REF_MAX_CORRIDOR_PATH = 256;

// dtLocalBoundary keeps this limit private (MAX_LOCAL_SEGS).
static const int REF_MAX_BOUNDARY_SEGMENTS = 8;

// A copy of dtObstacleCircle.
struct RefObstacleCircle
{
	float p[3];
	float vel[3];
	float dvel[3];
	float rad;
	float dp[3];
	float np[3];
};

// A copy of dtObstacleSegment; touch is 0 or 1.
struct RefObstacleSegment
{
	float p[3];
	float q[3];
	int32_t touch;
};

// What ref_crowd_update reports back, and the one thing it takes in.
//   opt_start, opt_end  out: the optimisation segment of the traced agent
//   vod                 in: debug data for that agent's velocity sampling, or null
struct RefCrowdDebugInfo
{
	float opt_start[3];
	float opt_end[3];
	dtObstacleAvoidanceDebugData* vod;
};

// Everything observable about one agent. The wide-aligned members come first
// and the narrow ones last, so no field has hidden padding before it; the
// whole struct is zeroed before it is filled, so every byte compares equal.
//   corridor_path          the first min(corridor_npath, 256) polygons
//   boundary_segments      the first min(boundary_nsegments, 8) segments, 6 floats each
//   anim_*                 filled only while anim_active: upstream never
//                          initialises an animation slot beyond its flag
struct RefAgentSnapshot
{
	dtCrowdAgentParams params;
	dtPolyRef corridor_path[REF_MAX_CORRIDOR_PATH];
	dtPolyRef corner_polys[DT_CROWDAGENT_MAX_CORNERS];
	dtPolyRef target_ref;
	dtPolyRef anim_poly_ref;
	float corridor_pos[3];
	float corridor_target[3];
	float boundary_center[3];
	float boundary_segments[REF_MAX_BOUNDARY_SEGMENTS][6];
	float topology_opt_time;
	dtCrowdNeighbour neis[DT_CROWDAGENT_MAX_NEIGHBOURS];
	float desired_speed;
	float npos[3];
	float disp[3];
	float dvel[3];
	float nvel[3];
	float vel[3];
	float corner_verts[DT_CROWDAGENT_MAX_CORNERS * 3];
	float target_pos[3];
	float target_replan_time;
	float anim_init_pos[3];
	float anim_start_pos[3];
	float anim_end_pos[3];
	float anim_t;
	float anim_tmax;
	dtPathQueueRef target_pathq_ref;
	int32_t corridor_npath;
	int32_t boundary_nsegments;
	int32_t nneis;
	int32_t ncorners;
	uint8_t corner_flags[DT_CROWDAGENT_MAX_CORNERS];
	uint8_t active;
	uint8_t state;
	uint8_t partial;
	uint8_t target_state;
	uint8_t target_replan;
	uint8_t anim_active;
	uint8_t reserved[6];
};

// ---------------------------------------------------------------------------
// Access to dtCrowd::m_agentAnims. Upstream keeps each agent's off-mesh
// animation in a private array with no accessor. The language lets an explicit
// instantiation name private members, so this reads the array without
// touching upstream's headers.
// ---------------------------------------------------------------------------

template <typename Tag, typename Tag::Type Member>
struct RefPrivateKey
{
	friend typename Tag::Type refPrivateMember(Tag) { return Member; }
};

struct RefCrowdAnimsTag
{
	typedef dtCrowdAgentAnimation* dtCrowd::*Type;
	friend Type refPrivateMember(RefCrowdAnimsTag);
};

template struct RefPrivateKey<RefCrowdAnimsTag, &dtCrowd::m_agentAnims>;

static const dtCrowdAgentAnimation* refCrowdAnimations(const dtCrowd* crowd)
{
	return crowd->*refPrivateMember(RefCrowdAnimsTag());
}

static void refCopyFloats(float* dst, const float* src, int count)
{
	memcpy(dst, src, sizeof(float) * count);
}

#define REF_SIZEOF(NAME, TYPE) size_t ref_sizeof_##NAME(void) { return sizeof(TYPE); }
#define REF_OFFSETOF(NAME, TYPE, FIELD) size_t ref_offsetof_##NAME(void) { return offsetof(TYPE, FIELD); }

extern "C" {

// ---------------------------------------------------------------------------
// DetourProximityGrid.h
// ---------------------------------------------------------------------------

dtProximityGrid* ref_prox_grid_alloc(void)
{
	return dtAllocProximityGrid();
}

void ref_prox_grid_free(dtProximityGrid* grid)
{
	dtFreeProximityGrid(grid);
}

bool ref_prox_grid_init(dtProximityGrid* grid, int pool_size, float cell_size)
{
	return grid->init(pool_size, cell_size);
}

void ref_prox_grid_clear(dtProximityGrid* grid)
{
	grid->clear();
}

void ref_prox_grid_add_item(dtProximityGrid* grid, unsigned short id,
							float minx, float miny, float maxx, float maxy)
{
	grid->addItem(id, minx, miny, maxx, maxy);
}

int ref_prox_grid_query_items(const dtProximityGrid* grid,
							  float minx, float miny, float maxx, float maxy,
							  unsigned short* ids, int max_ids)
{
	return grid->queryItems(minx, miny, maxx, maxy, ids, max_ids);
}

int ref_prox_grid_get_item_count_at(const dtProximityGrid* grid, int x, int y)
{
	return grid->getItemCountAt(x, y);
}

// Points at four ints: minx, miny, maxx, maxy in cells.
const int* ref_prox_grid_get_bounds(const dtProximityGrid* grid)
{
	return grid->getBounds();
}

float ref_prox_grid_get_cell_size(const dtProximityGrid* grid)
{
	return grid->getCellSize();
}

// ---------------------------------------------------------------------------
// DetourObstacleAvoidance.h: dtObstacleAvoidanceDebugData
// ---------------------------------------------------------------------------

dtObstacleAvoidanceDebugData* ref_oa_debug_alloc(void)
{
	return dtAllocObstacleAvoidanceDebugData();
}

void ref_oa_debug_free(dtObstacleAvoidanceDebugData* debug)
{
	dtFreeObstacleAvoidanceDebugData(debug);
}

bool ref_oa_debug_init(dtObstacleAvoidanceDebugData* debug, int max_samples)
{
	return debug->init(max_samples);
}

void ref_oa_debug_reset(dtObstacleAvoidanceDebugData* debug)
{
	debug->reset();
}

void ref_oa_debug_add_sample(dtObstacleAvoidanceDebugData* debug, const float* vel,
							 float ssize, float pen, float vpen, float vcpen,
							 float spen, float tpen)
{
	debug->addSample(vel, ssize, pen, vpen, vcpen, spen, tpen);
}

void ref_oa_debug_normalize_samples(dtObstacleAvoidanceDebugData* debug)
{
	debug->normalizeSamples();
}

int ref_oa_debug_get_sample_count(const dtObstacleAvoidanceDebugData* debug)
{
	return debug->getSampleCount();
}

// Points at three floats.
const float* ref_oa_debug_get_sample_velocity(const dtObstacleAvoidanceDebugData* debug, int i)
{
	return debug->getSampleVelocity(i);
}

float ref_oa_debug_get_sample_size(const dtObstacleAvoidanceDebugData* debug, int i)
{
	return debug->getSampleSize(i);
}

float ref_oa_debug_get_sample_penalty(const dtObstacleAvoidanceDebugData* debug, int i)
{
	return debug->getSamplePenalty(i);
}

float ref_oa_debug_get_sample_desired_velocity_penalty(const dtObstacleAvoidanceDebugData* debug, int i)
{
	return debug->getSampleDesiredVelocityPenalty(i);
}

float ref_oa_debug_get_sample_current_velocity_penalty(const dtObstacleAvoidanceDebugData* debug, int i)
{
	return debug->getSampleCurrentVelocityPenalty(i);
}

float ref_oa_debug_get_sample_preferred_side_penalty(const dtObstacleAvoidanceDebugData* debug, int i)
{
	return debug->getSamplePreferredSidePenalty(i);
}

float ref_oa_debug_get_sample_collision_time_penalty(const dtObstacleAvoidanceDebugData* debug, int i)
{
	return debug->getSampleCollisionTimePenalty(i);
}

// ---------------------------------------------------------------------------
// DetourObstacleAvoidance.h: dtObstacleAvoidanceQuery
// ---------------------------------------------------------------------------

dtObstacleAvoidanceQuery* ref_oa_query_alloc(void)
{
	return dtAllocObstacleAvoidanceQuery();
}

void ref_oa_query_free(dtObstacleAvoidanceQuery* query)
{
	dtFreeObstacleAvoidanceQuery(query);
}

bool ref_oa_query_init(dtObstacleAvoidanceQuery* query, int max_circles, int max_segments)
{
	return query->init(max_circles, max_segments);
}

void ref_oa_query_reset(dtObstacleAvoidanceQuery* query)
{
	query->reset();
}

void ref_oa_query_add_circle(dtObstacleAvoidanceQuery* query, const float* pos, float rad,
							 const float* vel, const float* dvel)
{
	query->addCircle(pos, rad, vel, dvel);
}

void ref_oa_query_add_segment(dtObstacleAvoidanceQuery* query, const float* p, const float* q)
{
	query->addSegment(p, q);
}

// `debug` may be null.
int ref_oa_query_sample_velocity_grid(dtObstacleAvoidanceQuery* query,
									  const float* pos, float rad, float vmax,
									  const float* vel, const float* dvel, float* nvel,
									  const dtObstacleAvoidanceParams* params,
									  dtObstacleAvoidanceDebugData* debug)
{
	return query->sampleVelocityGrid(pos, rad, vmax, vel, dvel, nvel, params, debug);
}

// `debug` may be null.
int ref_oa_query_sample_velocity_adaptive(dtObstacleAvoidanceQuery* query,
										  const float* pos, float rad, float vmax,
										  const float* vel, const float* dvel, float* nvel,
										  const dtObstacleAvoidanceParams* params,
										  dtObstacleAvoidanceDebugData* debug)
{
	return query->sampleVelocityAdaptive(pos, rad, vmax, vel, dvel, nvel, params, debug);
}

int ref_oa_query_get_obstacle_circle_count(const dtObstacleAvoidanceQuery* query)
{
	return query->getObstacleCircleCount();
}

// Copies circle `i` into `out`. Returns false, leaving `out` alone, when `i`
// is not in 0..count (upstream does not check).
bool ref_oa_query_get_obstacle_circle(dtObstacleAvoidanceQuery* query, int i, RefObstacleCircle* out)
{
	if (i < 0 || i >= query->getObstacleCircleCount())
		return false;
	const dtObstacleCircle* c = query->getObstacleCircle(i);
	refCopyFloats(out->p, c->p, 3);
	refCopyFloats(out->vel, c->vel, 3);
	refCopyFloats(out->dvel, c->dvel, 3);
	out->rad = c->rad;
	refCopyFloats(out->dp, c->dp, 3);
	refCopyFloats(out->np, c->np, 3);
	return true;
}

int ref_oa_query_get_obstacle_segment_count(const dtObstacleAvoidanceQuery* query)
{
	return query->getObstacleSegmentCount();
}

// Copies segment `i` into `out`. Returns false, leaving `out` alone, when `i`
// is not in 0..count (upstream does not check).
bool ref_oa_query_get_obstacle_segment(dtObstacleAvoidanceQuery* query, int i, RefObstacleSegment* out)
{
	if (i < 0 || i >= query->getObstacleSegmentCount())
		return false;
	const dtObstacleSegment* s = query->getObstacleSegment(i);
	refCopyFloats(out->p, s->p, 3);
	refCopyFloats(out->q, s->q, 3);
	out->touch = s->touch ? 1 : 0;
	return true;
}

// ---------------------------------------------------------------------------
// Layout checks for the plain structs the Zig side mirrors.
// ---------------------------------------------------------------------------

REF_SIZEOF(obstacle_avoidance_params, dtObstacleAvoidanceParams)
REF_OFFSETOF(obstacle_avoidance_params_vel_bias, dtObstacleAvoidanceParams, velBias)
REF_OFFSETOF(obstacle_avoidance_params_weight_des_vel, dtObstacleAvoidanceParams, weightDesVel)
REF_OFFSETOF(obstacle_avoidance_params_weight_cur_vel, dtObstacleAvoidanceParams, weightCurVel)
REF_OFFSETOF(obstacle_avoidance_params_weight_side, dtObstacleAvoidanceParams, weightSide)
REF_OFFSETOF(obstacle_avoidance_params_weight_toi, dtObstacleAvoidanceParams, weightToi)
REF_OFFSETOF(obstacle_avoidance_params_horiz_time, dtObstacleAvoidanceParams, horizTime)
REF_OFFSETOF(obstacle_avoidance_params_grid_size, dtObstacleAvoidanceParams, gridSize)
REF_OFFSETOF(obstacle_avoidance_params_adaptive_divs, dtObstacleAvoidanceParams, adaptiveDivs)
REF_OFFSETOF(obstacle_avoidance_params_adaptive_rings, dtObstacleAvoidanceParams, adaptiveRings)
REF_OFFSETOF(obstacle_avoidance_params_adaptive_depth, dtObstacleAvoidanceParams, adaptiveDepth)

REF_SIZEOF(crowd_agent_params, dtCrowdAgentParams)
REF_OFFSETOF(crowd_agent_params_radius, dtCrowdAgentParams, radius)
REF_OFFSETOF(crowd_agent_params_height, dtCrowdAgentParams, height)
REF_OFFSETOF(crowd_agent_params_max_acceleration, dtCrowdAgentParams, maxAcceleration)
REF_OFFSETOF(crowd_agent_params_max_speed, dtCrowdAgentParams, maxSpeed)
REF_OFFSETOF(crowd_agent_params_collision_query_range, dtCrowdAgentParams, collisionQueryRange)
REF_OFFSETOF(crowd_agent_params_path_optimization_range, dtCrowdAgentParams, pathOptimizationRange)
REF_OFFSETOF(crowd_agent_params_separation_weight, dtCrowdAgentParams, separationWeight)
REF_OFFSETOF(crowd_agent_params_update_flags, dtCrowdAgentParams, updateFlags)
REF_OFFSETOF(crowd_agent_params_obstacle_avoidance_type, dtCrowdAgentParams, obstacleAvoidanceType)
REF_OFFSETOF(crowd_agent_params_query_filter_type, dtCrowdAgentParams, queryFilterType)
REF_OFFSETOF(crowd_agent_params_user_data, dtCrowdAgentParams, userData)

// The structs this file defines, so the Zig mirrors can be checked as well.
REF_SIZEOF(crowd_neighbour, dtCrowdNeighbour)
REF_SIZEOF(obstacle_circle, RefObstacleCircle)
REF_SIZEOF(obstacle_segment, RefObstacleSegment)
REF_SIZEOF(crowd_debug_info, RefCrowdDebugInfo)
REF_SIZEOF(agent_snapshot, RefAgentSnapshot)

// ---------------------------------------------------------------------------
// DetourLocalBoundary.h
// ---------------------------------------------------------------------------

// Built with dtAlloc and placement new; there is no upstream alloc function.
dtLocalBoundary* ref_local_boundary_create(void)
{
	void* mem = dtAlloc(sizeof(dtLocalBoundary), DT_ALLOC_PERM);
	if (!mem)
		return 0;
	return new (mem) dtLocalBoundary;
}

void ref_local_boundary_destroy(dtLocalBoundary* boundary)
{
	if (!boundary)
		return;
	boundary->~dtLocalBoundary();
	dtFree(boundary);
}

void ref_local_boundary_reset(dtLocalBoundary* boundary)
{
	boundary->reset();
}

void ref_local_boundary_update(dtLocalBoundary* boundary, dtPolyRef ref, const float* pos,
							   float collision_query_range, dtNavMeshQuery* navquery,
							   const dtQueryFilter* filter)
{
	boundary->update(ref, pos, collision_query_range, navquery, filter);
}

bool ref_local_boundary_is_valid(dtLocalBoundary* boundary, dtNavMeshQuery* navquery,
								 const dtQueryFilter* filter)
{
	return boundary->isValid(navquery, filter);
}

// Points at three floats.
const float* ref_local_boundary_get_center(const dtLocalBoundary* boundary)
{
	return boundary->getCenter();
}

int ref_local_boundary_get_segment_count(const dtLocalBoundary* boundary)
{
	return boundary->getSegmentCount();
}

// Points at six floats: the segment's start then end. `i` is not checked.
const float* ref_local_boundary_get_segment(const dtLocalBoundary* boundary, int i)
{
	return boundary->getSegment(i);
}

// ---------------------------------------------------------------------------
// DetourPathCorridor.h
// ---------------------------------------------------------------------------

// Built with dtAlloc and placement new; there is no upstream alloc function.
dtPathCorridor* ref_corridor_create(void)
{
	void* mem = dtAlloc(sizeof(dtPathCorridor), DT_ALLOC_PERM);
	if (!mem)
		return 0;
	return new (mem) dtPathCorridor;
}

void ref_corridor_destroy(dtPathCorridor* corridor)
{
	if (!corridor)
		return;
	corridor->~dtPathCorridor();
	dtFree(corridor);
}

bool ref_corridor_init(dtPathCorridor* corridor, int max_path)
{
	return corridor->init(max_path);
}

void ref_corridor_reset(dtPathCorridor* corridor, dtPolyRef ref, const float* pos)
{
	corridor->reset(ref, pos);
}

int ref_corridor_find_corners(dtPathCorridor* corridor, float* corner_verts,
							  unsigned char* corner_flags, dtPolyRef* corner_polys,
							  int max_corners, dtNavMeshQuery* navquery,
							  const dtQueryFilter* filter)
{
	return corridor->findCorners(corner_verts, corner_flags, corner_polys, max_corners,
								 navquery, filter);
}

void ref_corridor_optimize_path_visibility(dtPathCorridor* corridor, const float* next,
										   float path_optimization_range,
										   dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	corridor->optimizePathVisibility(next, path_optimization_range, navquery, filter);
}

bool ref_corridor_optimize_path_topology(dtPathCorridor* corridor, dtNavMeshQuery* navquery,
										 const dtQueryFilter* filter)
{
	return corridor->optimizePathTopology(navquery, filter);
}

// `refs` receives two polygon references, `start_pos` and `end_pos` three floats each.
bool ref_corridor_move_over_offmesh_connection(dtPathCorridor* corridor,
											   dtPolyRef off_mesh_con_ref, dtPolyRef* refs,
											   float* start_pos, float* end_pos,
											   dtNavMeshQuery* navquery)
{
	return corridor->moveOverOffmeshConnection(off_mesh_con_ref, refs, start_pos, end_pos,
											   navquery);
}

bool ref_corridor_fix_path_start(dtPathCorridor* corridor, dtPolyRef safe_ref,
								 const float* safe_pos)
{
	return corridor->fixPathStart(safe_ref, safe_pos);
}

bool ref_corridor_trim_invalid_path(dtPathCorridor* corridor, dtPolyRef safe_ref,
									const float* safe_pos, dtNavMeshQuery* navquery,
									const dtQueryFilter* filter)
{
	return corridor->trimInvalidPath(safe_ref, safe_pos, navquery, filter);
}

bool ref_corridor_is_valid(dtPathCorridor* corridor, int max_look_ahead,
						   dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	return corridor->isValid(max_look_ahead, navquery, filter);
}

bool ref_corridor_move_position(dtPathCorridor* corridor, const float* npos,
								dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	return corridor->movePosition(npos, navquery, filter);
}

bool ref_corridor_move_target_position(dtPathCorridor* corridor, const float* npos,
									   dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	return corridor->moveTargetPosition(npos, navquery, filter);
}

void ref_corridor_set_corridor(dtPathCorridor* corridor, const float* target,
							   const dtPolyRef* polys, int npath)
{
	corridor->setCorridor(target, polys, npath);
}

// Points at three floats.
const float* ref_corridor_get_pos(const dtPathCorridor* corridor)
{
	return corridor->getPos();
}

// Points at three floats.
const float* ref_corridor_get_target(const dtPathCorridor* corridor)
{
	return corridor->getTarget();
}

dtPolyRef ref_corridor_get_first_poly(const dtPathCorridor* corridor)
{
	return corridor->getFirstPoly();
}

dtPolyRef ref_corridor_get_last_poly(const dtPathCorridor* corridor)
{
	return corridor->getLastPoly();
}

// Null until init. Points at getPathCount() polygons and goes stale when the
// corridor changes.
const dtPolyRef* ref_corridor_get_path(const dtPathCorridor* corridor)
{
	return corridor->getPath();
}

int ref_corridor_get_path_count(const dtPathCorridor* corridor)
{
	return corridor->getPathCount();
}

int ref_merge_corridor_start_moved(dtPolyRef* path, int npath, int max_path,
								   const dtPolyRef* visited, int nvisited)
{
	return dtMergeCorridorStartMoved(path, npath, max_path, visited, nvisited);
}

int ref_merge_corridor_end_moved(dtPolyRef* path, int npath, int max_path,
								 const dtPolyRef* visited, int nvisited)
{
	return dtMergeCorridorEndMoved(path, npath, max_path, visited, nvisited);
}

int ref_merge_corridor_start_shortcut(dtPolyRef* path, int npath, int max_path,
									  const dtPolyRef* visited, int nvisited)
{
	return dtMergeCorridorStartShortcut(path, npath, max_path, visited, nvisited);
}

// ---------------------------------------------------------------------------
// DetourPathQueue.h
// ---------------------------------------------------------------------------

// Built with dtAlloc and placement new; there is no upstream alloc function.
dtPathQueue* ref_path_queue_create(void)
{
	void* mem = dtAlloc(sizeof(dtPathQueue), DT_ALLOC_PERM);
	if (!mem)
		return 0;
	return new (mem) dtPathQueue;
}

void ref_path_queue_destroy(dtPathQueue* queue)
{
	if (!queue)
		return;
	queue->~dtPathQueue();
	dtFree(queue);
}

bool ref_path_queue_init(dtPathQueue* queue, int max_path_size, int max_search_node_count,
						 dtNavMesh* nav)
{
	return queue->init(max_path_size, max_search_node_count, nav);
}

void ref_path_queue_update(dtPathQueue* queue, int max_iters)
{
	queue->update(max_iters);
}

// The queue keeps the `filter` pointer until the request is consumed.
dtPathQueueRef ref_path_queue_request(dtPathQueue* queue, dtPolyRef start_ref, dtPolyRef end_ref,
									  const float* start_pos, const float* end_pos,
									  const dtQueryFilter* filter)
{
	return queue->request(start_ref, end_ref, start_pos, end_pos, filter);
}

unsigned int ref_path_queue_get_request_status(const dtPathQueue* queue, dtPathQueueRef ref)
{
	return queue->getRequestStatus(ref);
}

unsigned int ref_path_queue_get_path_result(dtPathQueue* queue, dtPathQueueRef ref,
											dtPolyRef* path, int* path_size, int max_path)
{
	return queue->getPathResult(ref, path, path_size, max_path);
}

const dtNavMeshQuery* ref_path_queue_get_nav_query(const dtPathQueue* queue)
{
	return queue->getNavQuery();
}

// ---------------------------------------------------------------------------
// DetourCrowd.h
// ---------------------------------------------------------------------------

dtCrowd* ref_crowd_alloc(void)
{
	return dtAllocCrowd();
}

void ref_crowd_free(dtCrowd* crowd)
{
	dtFreeCrowd(crowd);
}

// The crowd keeps `nav` for as long as it lives.
bool ref_crowd_init(dtCrowd* crowd, int max_agents, float max_agent_radius, dtNavMesh* nav)
{
	return crowd->init(max_agents, max_agent_radius, nav);
}

void ref_crowd_set_obstacle_avoidance_params(dtCrowd* crowd, int idx,
											 const dtObstacleAvoidanceParams* params)
{
	crowd->setObstacleAvoidanceParams(idx, params);
}

// Null when `idx` is not in 0..8.
const dtObstacleAvoidanceParams* ref_crowd_get_obstacle_avoidance_params(const dtCrowd* crowd, int idx)
{
	return crowd->getObstacleAvoidanceParams(idx);
}

// Null when `idx` is not in 0..agent count. The pool never moves, but an
// agent's contents change with every update, and init frees the pool.
const dtCrowdAgent* ref_crowd_get_agent(dtCrowd* crowd, int idx)
{
	return crowd->getAgent(idx);
}

dtCrowdAgent* ref_crowd_get_editable_agent(dtCrowd* crowd, int idx)
{
	return crowd->getEditableAgent(idx);
}

int ref_crowd_get_agent_count(const dtCrowd* crowd)
{
	return crowd->getAgentCount();
}

int ref_crowd_add_agent(dtCrowd* crowd, const float* pos, const dtCrowdAgentParams* params)
{
	return crowd->addAgent(pos, params);
}

void ref_crowd_update_agent_parameters(dtCrowd* crowd, int idx, const dtCrowdAgentParams* params)
{
	crowd->updateAgentParameters(idx, params);
}

void ref_crowd_remove_agent(dtCrowd* crowd, int idx)
{
	crowd->removeAgent(idx);
}

bool ref_crowd_request_move_target(dtCrowd* crowd, int idx, dtPolyRef ref, const float* pos)
{
	return crowd->requestMoveTarget(idx, ref, pos);
}

bool ref_crowd_request_move_velocity(dtCrowd* crowd, int idx, const float* vel)
{
	return crowd->requestMoveVelocity(idx, vel);
}

bool ref_crowd_reset_move_target(dtCrowd* crowd, int idx)
{
	return crowd->resetMoveTarget(idx);
}

// Upstream fills an array of agent pointers; this writes each active agent's
// index in the pool into `out_indices` instead (at most `max_agents` of them)
// and returns how many. Returns -1 if the temporary pointer array cannot be
// allocated.
int ref_crowd_get_active_agents(dtCrowd* crowd, int* out_indices, int max_agents)
{
	const int agent_count = crowd->getAgentCount();
	int capacity = max_agents > 0 ? max_agents : 0;
	if (capacity > agent_count)
		capacity = agent_count;

	dtCrowdAgent** agents =
		(dtCrowdAgent**)dtAlloc(sizeof(dtCrowdAgent*) * (capacity > 0 ? capacity : 1), DT_ALLOC_TEMP);
	if (!agents)
		return -1;

	const int n = crowd->getActiveAgents(agents, capacity);
	const dtCrowdAgent* first = crowd->getAgent(0);
	for (int i = 0; i < n; ++i)
		out_indices[i] = (int)(agents[i] - first);

	dtFree(agents);
	return n;
}

// Runs one update. `debug_idx` is the position in the list of active agents
// (the order ref_crowd_get_active_agents reports) that upstream traces; upstream
// compares it with that position, not with the pool index. With a negative
// `debug_idx` and a null `out` this passes no debug info at all. Otherwise it
// builds the dtCrowdAgentDebugInfo, hands `out->vod` to it (null for none) and
// copies the optimisation segment into `out`; both stay zero when the traced
// agent is skipped.
void ref_crowd_update(dtCrowd* crowd, float dt, int debug_idx, RefCrowdDebugInfo* out)
{
	if (debug_idx < 0 && !out)
	{
		crowd->update(dt, 0);
		return;
	}

	dtCrowdAgentDebugInfo debug;
	memset(&debug, 0, sizeof(debug));
	debug.idx = debug_idx;
	debug.vod = out ? out->vod : 0;

	crowd->update(dt, &debug);

	if (out)
	{
		refCopyFloats(out->opt_start, debug.optStart, 3);
		refCopyFloats(out->opt_end, debug.optEnd, 3);
	}
}

// Null when `i` is not in 0..16.
const dtQueryFilter* ref_crowd_get_filter(const dtCrowd* crowd, int i)
{
	return crowd->getFilter(i);
}

// Null when `i` is not in 0..16.
dtQueryFilter* ref_crowd_get_editable_filter(dtCrowd* crowd, int i)
{
	return crowd->getEditableFilter(i);
}

// Points at three floats.
const float* ref_crowd_get_query_half_extents(const dtCrowd* crowd)
{
	return crowd->getQueryHalfExtents();
}

// Points at three floats.
const float* ref_crowd_get_query_extents(const dtCrowd* crowd)
{
	return crowd->getQueryExtents();
}

int ref_crowd_get_velocity_sample_count(const dtCrowd* crowd)
{
	return crowd->getVelocitySampleCount();
}

const dtProximityGrid* ref_crowd_get_grid(const dtCrowd* crowd)
{
	return crowd->getGrid();
}

const dtPathQueue* ref_crowd_get_path_queue(const dtCrowd* crowd)
{
	return crowd->getPathQueue();
}

const dtNavMeshQuery* ref_crowd_get_nav_mesh_query(const dtCrowd* crowd)
{
	return crowd->getNavMeshQuery();
}

// The agent's own corridor and boundary, for driving them with the ref_corridor_*
// and ref_local_boundary_* functions. They live and die with the crowd.
dtPathCorridor* ref_crowd_agent_corridor(const dtCrowdAgent* agent)
{
	return &const_cast<dtCrowdAgent*>(agent)->corridor;
}

dtLocalBoundary* ref_crowd_agent_boundary(const dtCrowdAgent* agent)
{
	return &const_cast<dtCrowdAgent*>(agent)->boundary;
}

// Zeroes `out`, then copies every observable field of agent `idx` into it.
// Returns false, leaving `out` zeroed, when `idx` is not in 0..agent count.
bool ref_crowd_agent_snapshot(const dtCrowd* crowd, int idx, RefAgentSnapshot* out)
{
	memset(out, 0, sizeof(*out));

	const dtCrowdAgent* ag = const_cast<dtCrowd*>(crowd)->getAgent(idx);
	if (!ag)
		return false;

	out->active = ag->active ? 1 : 0;
	out->state = ag->state;
	out->partial = ag->partial ? 1 : 0;

	// The corridor, through its public getters.
	refCopyFloats(out->corridor_pos, ag->corridor.getPos(), 3);
	refCopyFloats(out->corridor_target, ag->corridor.getTarget(), 3);
	const int npath = ag->corridor.getPathCount();
	out->corridor_npath = npath;
	const dtPolyRef* path = ag->corridor.getPath();
	for (int i = 0; path && i < npath && i < REF_MAX_CORRIDOR_PATH; ++i)
		out->corridor_path[i] = path[i];

	// The local boundary, through its public getters.
	refCopyFloats(out->boundary_center, ag->boundary.getCenter(), 3);
	const int nsegs = ag->boundary.getSegmentCount();
	out->boundary_nsegments = nsegs;
	for (int i = 0; i < nsegs && i < REF_MAX_BOUNDARY_SEGMENTS; ++i)
		refCopyFloats(out->boundary_segments[i], ag->boundary.getSegment(i), 6);

	out->topology_opt_time = ag->topologyOptTime;

	for (int i = 0; i < DT_CROWDAGENT_MAX_NEIGHBOURS; ++i)
	{
		out->neis[i].idx = ag->neis[i].idx;
		out->neis[i].dist = ag->neis[i].dist;
	}
	out->nneis = ag->nneis;

	out->desired_speed = ag->desiredSpeed;
	refCopyFloats(out->npos, ag->npos, 3);
	refCopyFloats(out->disp, ag->disp, 3);
	refCopyFloats(out->dvel, ag->dvel, 3);
	refCopyFloats(out->nvel, ag->nvel, 3);
	refCopyFloats(out->vel, ag->vel, 3);

	// Field by field: the caller's padding bytes may have been copied into the
	// agent, and they must not reach the snapshot.
	out->params.radius = ag->params.radius;
	out->params.height = ag->params.height;
	out->params.maxAcceleration = ag->params.maxAcceleration;
	out->params.maxSpeed = ag->params.maxSpeed;
	out->params.collisionQueryRange = ag->params.collisionQueryRange;
	out->params.pathOptimizationRange = ag->params.pathOptimizationRange;
	out->params.separationWeight = ag->params.separationWeight;
	out->params.updateFlags = ag->params.updateFlags;
	out->params.obstacleAvoidanceType = ag->params.obstacleAvoidanceType;
	out->params.queryFilterType = ag->params.queryFilterType;
	out->params.userData = ag->params.userData;

	refCopyFloats(out->corner_verts, ag->cornerVerts, DT_CROWDAGENT_MAX_CORNERS * 3);
	for (int i = 0; i < DT_CROWDAGENT_MAX_CORNERS; ++i)
	{
		out->corner_flags[i] = ag->cornerFlags[i];
		out->corner_polys[i] = ag->cornerPolys[i];
	}
	out->ncorners = ag->ncorners;

	out->target_state = ag->targetState;
	out->target_ref = ag->targetRef;
	refCopyFloats(out->target_pos, ag->targetPos, 3);
	out->target_pathq_ref = ag->targetPathqRef;
	out->target_replan = ag->targetReplan ? 1 : 0;
	out->target_replan_time = ag->targetReplanTime;

	// The animation slot of this agent. Upstream sets only `active` when the
	// crowd is initialised and the rest when an animation starts, so the other
	// fields are copied only for an active one.
	const dtCrowdAgentAnimation* anims = refCrowdAnimations(crowd);
	if (anims)
	{
		const dtCrowdAgentAnimation& anim = anims[idx];
		out->anim_active = anim.active ? 1 : 0;
		if (anim.active)
		{
			refCopyFloats(out->anim_init_pos, anim.initPos, 3);
			refCopyFloats(out->anim_start_pos, anim.startPos, 3);
			refCopyFloats(out->anim_end_pos, anim.endPos, 3);
			out->anim_poly_ref = anim.polyRef;
			out->anim_t = anim.t;
			out->anim_tmax = anim.tmax;
		}
	}

	return true;
}

} // extern "C"

#undef REF_SIZEOF
#undef REF_OFFSETOF
