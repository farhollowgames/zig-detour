// Test-only C ABI over upstream DetourTileCache (recastnavigation v1.6.0).
//
// A separate pure-Zig port of the tile cache is tested against upstream
// through this file, byte for byte. Every function is `extern "C"`, prefixed
// `ref_tcb_` (DetourTileCacheBuilder.h) or `ref_tc_` (everything else), and
// forwards its arguments unchanged in upstream's order (object pointer first).
// Statuses are `unsigned int`, booleans are `bool`. Where upstream takes a
// reference this file takes a pointer. The Zig declarations live in
// test/reference_tile_cache.zig; keep the two in step.
//
// Departures from a pure forward, each forced by upstream's C++ shape:
//   - dtTileCacheAlloc, dtTileCacheCompressor and dtTileCacheMeshProcess are
//     interfaces. ref_tc_alloc_create_default builds the base class (dtAlloc,
//     dtFree, no-op reset), ref_tc_alloc_create_linear the bump allocator of
//     RecastDemo's Sample_TempObstacles.cpp, and the compressor and the mesh
//     process are subclasses that forward to C callbacks, so the Zig tests can
//     hand the same implementation to both sides.
//   - the overload of dtMarkBoxArea that takes a rotation is
//     ref_tcb_mark_oriented_box_area, and the overload of addBoxObstacle that
//     takes one is ref_tc_add_oriented_box_obstacle.
//   - dtCompressedTile is read through accessors, and a dtTileCacheObstacle is
//     flattened by ref_tc_obstacle_snapshot because it holds a union.
//
// Pointers that getters return point into the object asked and go stale when
// that object is freed or changed; reference_tile_cache.zig says which. The
// tile cache keeps the allocator, compressor and mesh process given to init,
// and the data given to addTile, so they must outlive it.
//
// Build notes (done by the integrator, never by this file):
//   - compiled with the Detour, DetourCrowd, DetourTileCache and Recast sources
//   - flags: -std=c++17 -ffp-contract=off -fno-fast-math -fno-exceptions
//     -fno-rtti -fno-threadsafe-statics
//   - must also compile with -DDT_POLYREF64 and/or -DDT_VIRTUAL_QUERYFILTER;
//     dtCompressedTileRef and dtObstacleRef stay 32 bits in every configuration

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <new>

#include "DetourAlloc.h"
#include "DetourStatus.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"

// ---------------------------------------------------------------------------
// Structs that cross the boundary. reference_tile_cache.zig mirrors each one
// field for field, in this order.
// ---------------------------------------------------------------------------

// A flat copy of one dtTileCacheObstacle. Only the union member that `type`
// names is filled; the others stay zero. The struct is zeroed before it is
// filled, so every byte compares equal.
struct RefObstacleSnapshot
{
	uint8_t obstacle_type;
	uint8_t state;
	uint8_t ntouched;
	uint8_t npending;
	uint16_t salt;
	uint16_t reserved;
	float cylinder_pos[3];
	float cylinder_radius;
	float cylinder_height;
	float box_bmin[3];
	float box_bmax[3];
	float oriented_center[3];
	float oriented_half_extents[3];
	float oriented_rot_aux[2];
	dtCompressedTileRef touched[DT_MAX_TOUCHED_TILES];
	dtCompressedTileRef pending[DT_MAX_TOUCHED_TILES];
};

// ---------------------------------------------------------------------------
// Callback shapes for the compressor and the mesh process. `user` is passed
// back unchanged.
// ---------------------------------------------------------------------------

typedef int (*RefCompressorMaxSizeFn)(void* user, int buffer_size);

typedef unsigned int (*RefCompressorCompressFn)(void* user, const unsigned char* buffer,
												int buffer_size, unsigned char* compressed,
												int max_compressed_size, int* compressed_size);

typedef unsigned int (*RefCompressorDecompressFn)(void* user, const unsigned char* compressed,
												  int compressed_size, unsigned char* buffer,
												  int max_buffer_size, int* buffer_size);

typedef void (*RefMeshProcessFn)(void* user, dtNavMeshCreateParams* params,
								 unsigned char* poly_areas, unsigned short* poly_flags);

// ---------------------------------------------------------------------------
// Subclasses of upstream's interfaces.
// ---------------------------------------------------------------------------

// The allocator of RecastDemo's Sample_TempObstacles.cpp: a bump pointer over
// one buffer, free does nothing, reset rewinds. Like that one it does not
// align or pad, so `top` counts exactly the bytes asked for and a test can
// compare it with the port's.
struct RefLinearAllocator : dtTileCacheAlloc
{
	unsigned char* buffer;
	size_t capacity;
	size_t top;
	size_t high;

	explicit RefLinearAllocator(const size_t cap)
		: buffer(0), capacity(cap), top(0), high(0)
	{
		buffer = static_cast<unsigned char*>(dtAlloc(cap, DT_ALLOC_PERM));
	}

	~RefLinearAllocator() override
	{
		dtFree(buffer);
	}

	void reset() override
	{
		high = high > top ? high : top;
		top = 0;
	}

	void* alloc(const size_t size) override
	{
		if (!buffer)
			return 0;
		if (top + size > capacity)
			return 0;

		unsigned char* mem = &buffer[top];
		top += size;
		return mem;
	}

	void free(void* /*ptr*/) override
	{
	}
};

// The base class's dtAlloc and dtFree, with a count of the calls to reset,
// so a test can compare them with the port's.
struct RefCountingAllocator : dtTileCacheAlloc
{
	int resets;

	RefCountingAllocator() : resets(0) {}

	void reset() override
	{
		++resets;
	}

	void* alloc(const size_t size) override
	{
		return dtAlloc(size, DT_ALLOC_TEMP);
	}

	void free(void* ptr) override
	{
		dtFree(ptr);
	}
};

// Forwards the three compressor methods to C callbacks.
struct RefCompressor : dtTileCacheCompressor
{
	RefCompressorMaxSizeFn max_size_fn;
	RefCompressorCompressFn compress_fn;
	RefCompressorDecompressFn decompress_fn;
	void* user;

	RefCompressor(RefCompressorMaxSizeFn max_size_cb, RefCompressorCompressFn compress_cb,
				  RefCompressorDecompressFn decompress_cb, void* user_data)
		: max_size_fn(max_size_cb), compress_fn(compress_cb), decompress_fn(decompress_cb),
		  user(user_data)
	{
	}

	~RefCompressor() override
	{
	}

	int maxCompressedSize(const int bufferSize) override
	{
		return max_size_fn(user, bufferSize);
	}

	dtStatus compress(const unsigned char* buffer, const int bufferSize,
					  unsigned char* compressed, const int maxOutSize,
					  int* compressedSize) override
	{
		return compress_fn(user, buffer, bufferSize, compressed, maxOutSize, compressedSize);
	}

	dtStatus decompress(const unsigned char* compressed, const int compressedSize,
						unsigned char* buffer, const int maxBufferSize,
						int* bufferSize) override
	{
		return decompress_fn(user, compressed, compressedSize, buffer, maxBufferSize, bufferSize);
	}
};

// Forwards process to a C callback.
struct RefMeshProcess : dtTileCacheMeshProcess
{
	RefMeshProcessFn process_fn;
	void* user;

	RefMeshProcess(RefMeshProcessFn process_cb, void* user_data)
		: process_fn(process_cb), user(user_data)
	{
	}

	~RefMeshProcess() override
	{
	}

	void process(dtNavMeshCreateParams* params, unsigned char* polyAreas,
				 unsigned short* polyFlags) override
	{
		process_fn(user, params, polyAreas, polyFlags);
	}
};

#define REF_SIZEOF(NAME, TYPE) size_t ref_sizeof_##NAME(void) { return sizeof(TYPE); }
#define REF_OFFSETOF(NAME, TYPE, FIELD) size_t ref_offsetof_##NAME(void) { return offsetof(TYPE, FIELD); }

extern "C" {

// ---------------------------------------------------------------------------
// DetourTileCacheBuilder.h: free functions
// ---------------------------------------------------------------------------

// On success `*out_data` is a dtAlloc buffer the caller owns: give it to
// ref_tc_add_tile with the free-data flag, or release it with ref_tcb_free_data.
unsigned int ref_tcb_build_layer(dtTileCacheCompressor* comp, dtTileCacheLayerHeader* header,
								 const unsigned char* heights, const unsigned char* areas,
								 const unsigned char* cons, unsigned char** out_data,
								 int* out_data_size)
{
	return dtBuildTileCacheLayer(comp, header, heights, areas, cons, out_data, out_data_size);
}

// Releases a buffer from ref_tcb_build_layer that no tile cache took over.
void ref_tcb_free_data(void* data)
{
	dtFree(data);
}

void ref_tcb_free_layer(dtTileCacheAlloc* alloc, dtTileCacheLayer* layer)
{
	dtFreeTileCacheLayer(alloc, layer);
}

// `*layer_out` comes from `alloc`; it is invalid after ref_tcb_free_layer and
// after anything that resets or frees `alloc`, including a tile cache build.
unsigned int ref_tcb_decompress_layer(dtTileCacheAlloc* alloc, dtTileCacheCompressor* comp,
									  unsigned char* compressed, int compressed_size,
									  dtTileCacheLayer** layer_out)
{
	return dtDecompressTileCacheLayer(alloc, comp, compressed, compressed_size, layer_out);
}

dtTileCacheContourSet* ref_tcb_alloc_contour_set(dtTileCacheAlloc* alloc)
{
	return dtAllocTileCacheContourSet(alloc);
}

void ref_tcb_free_contour_set(dtTileCacheAlloc* alloc, dtTileCacheContourSet* cset)
{
	dtFreeTileCacheContourSet(alloc, cset);
}

dtTileCachePolyMesh* ref_tcb_alloc_poly_mesh(dtTileCacheAlloc* alloc)
{
	return dtAllocTileCachePolyMesh(alloc);
}

void ref_tcb_free_poly_mesh(dtTileCacheAlloc* alloc, dtTileCachePolyMesh* mesh)
{
	dtFreeTileCachePolyMesh(alloc, mesh);
}

unsigned int ref_tcb_mark_cylinder_area(dtTileCacheLayer* layer, const float* orig, float cs,
										float ch, const float* pos, float radius, float height,
										unsigned char area_id)
{
	return dtMarkCylinderArea(*layer, orig, cs, ch, pos, radius, height, area_id);
}

// The overload of dtMarkBoxArea that takes the box's bounds.
unsigned int ref_tcb_mark_box_area(dtTileCacheLayer* layer, const float* orig, float cs, float ch,
								   const float* bmin, const float* bmax, unsigned char area_id)
{
	return dtMarkBoxArea(*layer, orig, cs, ch, bmin, bmax, area_id);
}

// The overload of dtMarkBoxArea that takes a centre, half extents and the two
// floats of rotation (see dtObstacleOrientedBox::rotAux).
unsigned int ref_tcb_mark_oriented_box_area(dtTileCacheLayer* layer, const float* orig, float cs,
											float ch, const float* center,
											const float* half_extents, const float* rot_aux,
											unsigned char area_id)
{
	return dtMarkBoxArea(*layer, orig, cs, ch, center, half_extents, rot_aux, area_id);
}

unsigned int ref_tcb_build_regions(dtTileCacheAlloc* alloc, dtTileCacheLayer* layer,
								   int walkable_climb)
{
	return dtBuildTileCacheRegions(alloc, *layer, walkable_climb);
}

unsigned int ref_tcb_build_contours(dtTileCacheAlloc* alloc, dtTileCacheLayer* layer,
									int walkable_climb, float max_error,
									dtTileCacheContourSet* lcset)
{
	return dtBuildTileCacheContours(alloc, *layer, walkable_climb, max_error, *lcset);
}

unsigned int ref_tcb_build_poly_mesh(dtTileCacheAlloc* alloc, dtTileCacheContourSet* lcset,
									 dtTileCachePolyMesh* mesh)
{
	return dtBuildTileCachePolyMesh(alloc, *lcset, *mesh);
}

bool ref_tcb_header_swap_endian(unsigned char* data, int data_size)
{
	return dtTileCacheHeaderSwapEndian(data, data_size);
}

// ---------------------------------------------------------------------------
// DetourTileCacheBuilder.h: reading dtTileCacheLayer, dtTileCacheContourSet and
// dtTileCachePolyMesh. The pointers point into the object and are invalid once
// it is freed.
// ---------------------------------------------------------------------------

dtTileCacheLayerHeader* ref_tcb_layer_header(const dtTileCacheLayer* layer)
{
	return layer->header;
}

unsigned char ref_tcb_layer_reg_count(const dtTileCacheLayer* layer)
{
	return layer->regCount;
}

// Each of the four grids holds header width * header height bytes.
unsigned char* ref_tcb_layer_heights(const dtTileCacheLayer* layer)
{
	return layer->heights;
}

unsigned char* ref_tcb_layer_areas(const dtTileCacheLayer* layer)
{
	return layer->areas;
}

unsigned char* ref_tcb_layer_cons(const dtTileCacheLayer* layer)
{
	return layer->cons;
}

unsigned char* ref_tcb_layer_regs(const dtTileCacheLayer* layer)
{
	return layer->regs;
}

int ref_tcb_contour_set_count(const dtTileCacheContourSet* cset)
{
	return cset->nconts;
}

// The per-contour readers return 0 (or null) when `i` is not in 0..count.
int ref_tcb_contour_nverts(const dtTileCacheContourSet* cset, int i)
{
	if (i < 0 || i >= cset->nconts)
		return 0;
	return cset->conts[i].nverts;
}

// Points at nverts * 4 bytes: x, y, z and the portal/remove byte of each vertex.
const unsigned char* ref_tcb_contour_verts(const dtTileCacheContourSet* cset, int i)
{
	if (i < 0 || i >= cset->nconts)
		return 0;
	return cset->conts[i].verts;
}

unsigned char ref_tcb_contour_reg(const dtTileCacheContourSet* cset, int i)
{
	if (i < 0 || i >= cset->nconts)
		return 0;
	return cset->conts[i].reg;
}

unsigned char ref_tcb_contour_area(const dtTileCacheContourSet* cset, int i)
{
	if (i < 0 || i >= cset->nconts)
		return 0;
	return cset->conts[i].area;
}

int ref_tcb_poly_mesh_nvp(const dtTileCachePolyMesh* mesh)
{
	return mesh->nvp;
}

int ref_tcb_poly_mesh_nverts(const dtTileCachePolyMesh* mesh)
{
	return mesh->nverts;
}

int ref_tcb_poly_mesh_npolys(const dtTileCachePolyMesh* mesh)
{
	return mesh->npolys;
}

// Three shorts per vertex.
const unsigned short* ref_tcb_poly_mesh_verts(const dtTileCachePolyMesh* mesh)
{
	return mesh->verts;
}

// nvp * 2 shorts per polygon.
const unsigned short* ref_tcb_poly_mesh_polys(const dtTileCachePolyMesh* mesh)
{
	return mesh->polys;
}

// One short per polygon.
const unsigned short* ref_tcb_poly_mesh_flags(const dtTileCachePolyMesh* mesh)
{
	return mesh->flags;
}

// One byte per polygon.
const unsigned char* ref_tcb_poly_mesh_areas(const dtTileCachePolyMesh* mesh)
{
	return mesh->areas;
}

// ---------------------------------------------------------------------------
// dtTileCacheAlloc
// ---------------------------------------------------------------------------

// The base class itself: dtAlloc, dtFree and a reset that does nothing.
dtTileCacheAlloc* ref_tc_alloc_create_default(void)
{
	void* mem = dtAlloc(sizeof(dtTileCacheAlloc), DT_ALLOC_PERM);
	if (!mem)
		return 0;
	return new (mem) dtTileCacheAlloc;
}

// A bump allocator over `capacity` bytes, as RefLinearAllocator describes.
dtTileCacheAlloc* ref_tc_alloc_create_linear(size_t capacity)
{
	void* mem = dtAlloc(sizeof(RefLinearAllocator), DT_ALLOC_PERM);
	if (!mem)
		return 0;
	return new (mem) RefLinearAllocator(capacity);
}

// The base class's allocation, counting resets as RefCountingAllocator says.
dtTileCacheAlloc* ref_tc_alloc_create_counting(void)
{
	void* mem = dtAlloc(sizeof(RefCountingAllocator), DT_ALLOC_PERM);
	if (!mem)
		return 0;
	return new (mem) RefCountingAllocator;
}

// The calls to reset so far, only for allocators made by
// ref_tc_alloc_create_counting; on any other allocator it reads garbage.
int ref_tc_alloc_reset_count(const dtTileCacheAlloc* alloc)
{
	return static_cast<const RefCountingAllocator*>(alloc)->resets;
}

// Destroys an allocator made by any of the create functions. Anything still
// allocated from it becomes invalid.
void ref_tc_alloc_destroy(dtTileCacheAlloc* alloc)
{
	if (!alloc)
		return;
	alloc->~dtTileCacheAlloc();
	dtFree(alloc);
}

void ref_tc_alloc_reset(dtTileCacheAlloc* alloc)
{
	alloc->reset();
}

// Upstream's `alloc`. Null when a linear allocator is out of room.
void* ref_tc_alloc_allocate(dtTileCacheAlloc* alloc, size_t size)
{
	return alloc->alloc(size);
}

// Upstream's `free`.
void ref_tc_alloc_deallocate(dtTileCacheAlloc* alloc, void* ptr)
{
	alloc->free(ptr);
}

// The three readers below are only for allocators made by
// ref_tc_alloc_create_linear; on any other allocator they read garbage.
size_t ref_tc_alloc_linear_capacity(const dtTileCacheAlloc* alloc)
{
	return static_cast<const RefLinearAllocator*>(alloc)->capacity;
}

// Bytes handed out since the last reset.
size_t ref_tc_alloc_linear_top(const dtTileCacheAlloc* alloc)
{
	return static_cast<const RefLinearAllocator*>(alloc)->top;
}

// The largest `top` seen at any reset.
size_t ref_tc_alloc_linear_high(const dtTileCacheAlloc* alloc)
{
	return static_cast<const RefLinearAllocator*>(alloc)->high;
}

// ---------------------------------------------------------------------------
// dtTileCacheCompressor
// ---------------------------------------------------------------------------

// Null when a callback is null or memory runs out. Statuses the callbacks
// return reach the caller of compress / decompress unchanged.
dtTileCacheCompressor* ref_tc_compressor_create(RefCompressorMaxSizeFn max_size,
												RefCompressorCompressFn compress,
												RefCompressorDecompressFn decompress,
												void* user)
{
	if (!max_size || !compress || !decompress)
		return 0;
	void* mem = dtAlloc(sizeof(RefCompressor), DT_ALLOC_PERM);
	if (!mem)
		return 0;
	return new (mem) RefCompressor(max_size, compress, decompress, user);
}

void ref_tc_compressor_destroy(dtTileCacheCompressor* comp)
{
	if (!comp)
		return;
	comp->~dtTileCacheCompressor();
	dtFree(comp);
}

// ---------------------------------------------------------------------------
// dtTileCacheMeshProcess
// ---------------------------------------------------------------------------

// Null when `process` is null or memory runs out.
dtTileCacheMeshProcess* ref_tc_mesh_process_create(RefMeshProcessFn process, void* user)
{
	if (!process)
		return 0;
	void* mem = dtAlloc(sizeof(RefMeshProcess), DT_ALLOC_PERM);
	if (!mem)
		return 0;
	return new (mem) RefMeshProcess(process, user);
}

void ref_tc_mesh_process_destroy(dtTileCacheMeshProcess* mesh_process)
{
	if (!mesh_process)
		return;
	mesh_process->~dtTileCacheMeshProcess();
	dtFree(mesh_process);
}

// ---------------------------------------------------------------------------
// dtTileCache
// ---------------------------------------------------------------------------

// dtAllocTileCache.
dtTileCache* ref_tc_create(void)
{
	return dtAllocTileCache();
}

// dtFreeTileCache. Also frees the data of every tile added with the free-data flag.
void ref_tc_destroy(dtTileCache* tc)
{
	dtFreeTileCache(tc);
}

// The tile cache keeps `talloc`, `tcomp` and `tmproc` (which may be null) for
// as long as it lives.
unsigned int ref_tc_init(dtTileCache* tc, const dtTileCacheParams* params, dtTileCacheAlloc* talloc,
						 dtTileCacheCompressor* tcomp, dtTileCacheMeshProcess* tmproc)
{
	return tc->init(params, talloc, tcomp, tmproc);
}

dtTileCacheAlloc* ref_tc_get_alloc(dtTileCache* tc)
{
	return tc->getAlloc();
}

dtTileCacheCompressor* ref_tc_get_compressor(dtTileCache* tc)
{
	return tc->getCompressor();
}

// Points into the tile cache.
const dtTileCacheParams* ref_tc_get_params(const dtTileCache* tc)
{
	return tc->getParams();
}

int ref_tc_get_tiles_at(const dtTileCache* tc, int tx, int ty, dtCompressedTileRef* tiles,
						int max_tiles)
{
	return tc->getTilesAt(tx, ty, tiles, max_tiles);
}

dtCompressedTile* ref_tc_get_tile_at(dtTileCache* tc, int tx, int ty, int tlayer)
{
	return tc->getTileAt(tx, ty, tlayer);
}

dtCompressedTileRef ref_tc_get_tile_ref(const dtTileCache* tc, const dtCompressedTile* tile)
{
	return tc->getTileRef(tile);
}

const dtCompressedTile* ref_tc_get_tile_by_ref(const dtTileCache* tc, dtCompressedTileRef ref)
{
	return tc->getTileByRef(ref);
}

// The tile cache keeps `data` (it does not copy it) and, with the free-data
// flag (1), frees it with dtFree when the tile is removed or the cache is destroyed.
unsigned int ref_tc_add_tile(dtTileCache* tc, unsigned char* data, int data_size,
							 unsigned char flags, dtCompressedTileRef* result)
{
	return tc->addTile(data, data_size, flags, result);
}

// `*data` and `*data_size` are zeroed when the tile owned its data, and
// otherwise give the data back to the caller.
unsigned int ref_tc_remove_tile(dtTileCache* tc, dtCompressedTileRef ref, unsigned char** data,
								int* data_size)
{
	return tc->removeTile(ref, data, data_size);
}

// The cylinder overload of addObstacle.
unsigned int ref_tc_add_obstacle(dtTileCache* tc, const float* pos, float radius, float height,
								 dtObstacleRef* result)
{
	return tc->addObstacle(pos, radius, height, result);
}

// The overload of addBoxObstacle that takes bounds.
unsigned int ref_tc_add_box_obstacle(dtTileCache* tc, const float* bmin, const float* bmax,
									 dtObstacleRef* result)
{
	return tc->addBoxObstacle(bmin, bmax, result);
}

// The overload of addBoxObstacle that takes a centre, half extents and a
// rotation about y.
unsigned int ref_tc_add_oriented_box_obstacle(dtTileCache* tc, const float* center,
											  const float* half_extents, float y_radians,
											  dtObstacleRef* result)
{
	return tc->addBoxObstacle(center, half_extents, y_radians, result);
}

unsigned int ref_tc_remove_obstacle(dtTileCache* tc, dtObstacleRef ref)
{
	return tc->removeObstacle(ref);
}

unsigned int ref_tc_query_tiles(const dtTileCache* tc, const float* bmin, const float* bmax,
								dtCompressedTileRef* results, int* result_count, int max_results)
{
	return tc->queryTiles(bmin, bmax, results, result_count, max_results);
}

// `up_to_date` may be null.
unsigned int ref_tc_update(dtTileCache* tc, float dt, dtNavMesh* navmesh, bool* up_to_date)
{
	return tc->update(dt, navmesh, up_to_date);
}

// Resets the tile cache's allocator, so anything allocated from it earlier is invalid.
unsigned int ref_tc_build_nav_mesh_tiles_at(dtTileCache* tc, int tx, int ty, dtNavMesh* navmesh)
{
	return tc->buildNavMeshTilesAt(tx, ty, navmesh);
}

// Resets the tile cache's allocator, so anything allocated from it earlier is invalid.
unsigned int ref_tc_build_nav_mesh_tile(dtTileCache* tc, dtCompressedTileRef ref,
										dtNavMesh* navmesh)
{
	return tc->buildNavMeshTile(ref, navmesh);
}

void ref_tc_calc_tight_tile_bounds(const dtTileCache* tc, const dtTileCacheLayerHeader* header,
								   float* bmin, float* bmax)
{
	tc->calcTightTileBounds(header, bmin, bmax);
}

void ref_tc_get_obstacle_bounds(const dtTileCache* tc, const dtTileCacheObstacle* ob, float* bmin,
								float* bmax)
{
	tc->getObstacleBounds(ob, bmin, bmax);
}

// The tile and obstacle pools never move, but their entries change with every
// add, remove and update.
int ref_tc_get_tile_count(const dtTileCache* tc)
{
	return tc->getTileCount();
}

// `i` is not checked.
const dtCompressedTile* ref_tc_get_tile(const dtTileCache* tc, int i)
{
	return tc->getTile(i);
}

int ref_tc_get_obstacle_count(const dtTileCache* tc)
{
	return tc->getObstacleCount();
}

// `i` is not checked.
const dtTileCacheObstacle* ref_tc_get_obstacle(const dtTileCache* tc, int i)
{
	return tc->getObstacle(i);
}

const dtTileCacheObstacle* ref_tc_get_obstacle_by_ref(dtTileCache* tc, dtObstacleRef ref)
{
	return tc->getObstacleByRef(ref);
}

dtObstacleRef ref_tc_get_obstacle_ref(const dtTileCache* tc, const dtTileCacheObstacle* ob)
{
	return tc->getObstacleRef(ob);
}

dtCompressedTileRef ref_tc_encode_tile_id(const dtTileCache* tc, unsigned int salt,
										  unsigned int it)
{
	return tc->encodeTileId(salt, it);
}

unsigned int ref_tc_decode_tile_id_salt(const dtTileCache* tc, dtCompressedTileRef ref)
{
	return tc->decodeTileIdSalt(ref);
}

unsigned int ref_tc_decode_tile_id_tile(const dtTileCache* tc, dtCompressedTileRef ref)
{
	return tc->decodeTileIdTile(ref);
}

dtObstacleRef ref_tc_encode_obstacle_id(const dtTileCache* tc, unsigned int salt, unsigned int it)
{
	return tc->encodeObstacleId(salt, it);
}

unsigned int ref_tc_decode_obstacle_id_salt(const dtTileCache* tc, dtObstacleRef ref)
{
	return tc->decodeObstacleIdSalt(ref);
}

unsigned int ref_tc_decode_obstacle_id_obstacle(const dtTileCache* tc, dtObstacleRef ref)
{
	return tc->decodeObstacleIdObstacle(ref);
}

// ---------------------------------------------------------------------------
// dtCompressedTile: plain data, read field by field. The pointers point into
// the tile cache's pool entry or the data it keeps, and go stale when the tile
// is removed.
// ---------------------------------------------------------------------------

unsigned int ref_tc_tile_salt(const dtCompressedTile* tile)
{
	return tile->salt;
}

dtTileCacheLayerHeader* ref_tc_tile_header(const dtCompressedTile* tile)
{
	return tile->header;
}

unsigned char* ref_tc_tile_compressed(const dtCompressedTile* tile)
{
	return tile->compressed;
}

int ref_tc_tile_compressed_size(const dtCompressedTile* tile)
{
	return tile->compressedSize;
}

unsigned char* ref_tc_tile_data(const dtCompressedTile* tile)
{
	return tile->data;
}

int ref_tc_tile_data_size(const dtCompressedTile* tile)
{
	return tile->dataSize;
}

unsigned int ref_tc_tile_flags(const dtCompressedTile* tile)
{
	return tile->flags;
}

// ---------------------------------------------------------------------------
// dtTileCacheObstacle
// ---------------------------------------------------------------------------

// Zeroes `out`, then copies the obstacle into it. Returns false, leaving `out`
// zeroed, when `ob` is null.
bool ref_tc_obstacle_snapshot(const dtTileCacheObstacle* ob, RefObstacleSnapshot* out)
{
	memset(out, 0, sizeof(*out));
	if (!ob)
		return false;

	out->obstacle_type = ob->type;
	out->state = ob->state;
	out->ntouched = ob->ntouched;
	out->npending = ob->npending;
	out->salt = ob->salt;

	if (ob->type == DT_OBSTACLE_CYLINDER)
	{
		memcpy(out->cylinder_pos, ob->cylinder.pos, sizeof(float) * 3);
		out->cylinder_radius = ob->cylinder.radius;
		out->cylinder_height = ob->cylinder.height;
	}
	else if (ob->type == DT_OBSTACLE_BOX)
	{
		memcpy(out->box_bmin, ob->box.bmin, sizeof(float) * 3);
		memcpy(out->box_bmax, ob->box.bmax, sizeof(float) * 3);
	}
	else if (ob->type == DT_OBSTACLE_ORIENTED_BOX)
	{
		memcpy(out->oriented_center, ob->orientedBox.center, sizeof(float) * 3);
		memcpy(out->oriented_half_extents, ob->orientedBox.halfExtents, sizeof(float) * 3);
		memcpy(out->oriented_rot_aux, ob->orientedBox.rotAux, sizeof(float) * 2);
	}

	for (int i = 0; i < DT_MAX_TOUCHED_TILES; ++i)
	{
		out->touched[i] = ob->touched[i];
		out->pending[i] = ob->pending[i];
	}

	return true;
}

// ---------------------------------------------------------------------------
// Layout checks. The Zig side mirrors dtTileCacheLayerHeader and
// dtTileCacheParams field for field; the other three are for comparing the
// port's own structs (dtCompressedTile and dtTileCacheObstacle) and the
// snapshot this file defines.
// ---------------------------------------------------------------------------

REF_SIZEOF(tile_cache_layer_header, dtTileCacheLayerHeader)
REF_OFFSETOF(tile_cache_layer_header_magic, dtTileCacheLayerHeader, magic)
REF_OFFSETOF(tile_cache_layer_header_version, dtTileCacheLayerHeader, version)
REF_OFFSETOF(tile_cache_layer_header_tx, dtTileCacheLayerHeader, tx)
REF_OFFSETOF(tile_cache_layer_header_ty, dtTileCacheLayerHeader, ty)
REF_OFFSETOF(tile_cache_layer_header_tlayer, dtTileCacheLayerHeader, tlayer)
REF_OFFSETOF(tile_cache_layer_header_bmin, dtTileCacheLayerHeader, bmin)
REF_OFFSETOF(tile_cache_layer_header_bmax, dtTileCacheLayerHeader, bmax)
REF_OFFSETOF(tile_cache_layer_header_hmin, dtTileCacheLayerHeader, hmin)
REF_OFFSETOF(tile_cache_layer_header_hmax, dtTileCacheLayerHeader, hmax)
REF_OFFSETOF(tile_cache_layer_header_width, dtTileCacheLayerHeader, width)
REF_OFFSETOF(tile_cache_layer_header_height, dtTileCacheLayerHeader, height)
REF_OFFSETOF(tile_cache_layer_header_minx, dtTileCacheLayerHeader, minx)
REF_OFFSETOF(tile_cache_layer_header_maxx, dtTileCacheLayerHeader, maxx)
REF_OFFSETOF(tile_cache_layer_header_miny, dtTileCacheLayerHeader, miny)
REF_OFFSETOF(tile_cache_layer_header_maxy, dtTileCacheLayerHeader, maxy)

REF_SIZEOF(tile_cache_params, dtTileCacheParams)
REF_OFFSETOF(tile_cache_params_orig, dtTileCacheParams, orig)
REF_OFFSETOF(tile_cache_params_cs, dtTileCacheParams, cs)
REF_OFFSETOF(tile_cache_params_ch, dtTileCacheParams, ch)
REF_OFFSETOF(tile_cache_params_width, dtTileCacheParams, width)
REF_OFFSETOF(tile_cache_params_height, dtTileCacheParams, height)
REF_OFFSETOF(tile_cache_params_walkable_height, dtTileCacheParams, walkableHeight)
REF_OFFSETOF(tile_cache_params_walkable_radius, dtTileCacheParams, walkableRadius)
REF_OFFSETOF(tile_cache_params_walkable_climb, dtTileCacheParams, walkableClimb)
REF_OFFSETOF(tile_cache_params_max_simplification_error, dtTileCacheParams, maxSimplificationError)
REF_OFFSETOF(tile_cache_params_max_tiles, dtTileCacheParams, maxTiles)
REF_OFFSETOF(tile_cache_params_max_obstacles, dtTileCacheParams, maxObstacles)

REF_SIZEOF(compressed_tile, dtCompressedTile)
REF_OFFSETOF(compressed_tile_salt, dtCompressedTile, salt)
REF_OFFSETOF(compressed_tile_header, dtCompressedTile, header)
REF_OFFSETOF(compressed_tile_compressed, dtCompressedTile, compressed)
REF_OFFSETOF(compressed_tile_compressed_size, dtCompressedTile, compressedSize)
REF_OFFSETOF(compressed_tile_data, dtCompressedTile, data)
REF_OFFSETOF(compressed_tile_data_size, dtCompressedTile, dataSize)
REF_OFFSETOF(compressed_tile_flags, dtCompressedTile, flags)
REF_OFFSETOF(compressed_tile_next, dtCompressedTile, next)

// The union's three members all start at offset 0; the nested offsets are
// relative to the whole obstacle.
REF_SIZEOF(tile_cache_obstacle, dtTileCacheObstacle)
REF_OFFSETOF(tile_cache_obstacle_cylinder_pos, dtTileCacheObstacle, cylinder.pos)
REF_OFFSETOF(tile_cache_obstacle_cylinder_radius, dtTileCacheObstacle, cylinder.radius)
REF_OFFSETOF(tile_cache_obstacle_cylinder_height, dtTileCacheObstacle, cylinder.height)
REF_OFFSETOF(tile_cache_obstacle_box_bmin, dtTileCacheObstacle, box.bmin)
REF_OFFSETOF(tile_cache_obstacle_box_bmax, dtTileCacheObstacle, box.bmax)
REF_OFFSETOF(tile_cache_obstacle_oriented_box_center, dtTileCacheObstacle, orientedBox.center)
REF_OFFSETOF(tile_cache_obstacle_oriented_box_half_extents, dtTileCacheObstacle, orientedBox.halfExtents)
REF_OFFSETOF(tile_cache_obstacle_oriented_box_rot_aux, dtTileCacheObstacle, orientedBox.rotAux)
REF_OFFSETOF(tile_cache_obstacle_touched, dtTileCacheObstacle, touched)
REF_OFFSETOF(tile_cache_obstacle_pending, dtTileCacheObstacle, pending)
REF_OFFSETOF(tile_cache_obstacle_salt, dtTileCacheObstacle, salt)
REF_OFFSETOF(tile_cache_obstacle_type, dtTileCacheObstacle, type)
REF_OFFSETOF(tile_cache_obstacle_state, dtTileCacheObstacle, state)
REF_OFFSETOF(tile_cache_obstacle_ntouched, dtTileCacheObstacle, ntouched)
REF_OFFSETOF(tile_cache_obstacle_npending, dtTileCacheObstacle, npending)
REF_OFFSETOF(tile_cache_obstacle_next, dtTileCacheObstacle, next)

REF_SIZEOF(obstacle_snapshot, RefObstacleSnapshot)

} // extern "C"

#undef REF_SIZEOF
#undef REF_OFFSETOF
