// Test-only C ABI over upstream Detour (recastnavigation v1.6.0).
//
// A separate pure-Zig port of Detour is tested against upstream through this
// file, byte for byte. Every function is a plain forward: arguments keep
// upstream's order (object pointer first), results keep upstream's types.
// Where upstream passes references, this file passes pointers. The Zig
// declarations live in test/reference_detour.zig; keep the two in step.
//
// Build notes (done by the integrator, never by this file):
//   - compiled with all of Detour/Source/*.cpp and Recast/Source/*.cpp
//   - flags: -std=c++17 -ffp-contract=off -fno-fast-math -fno-exceptions
//     -fno-rtti -fno-threadsafe-statics
//   - must also compile with -DDT_POLYREF64 and/or -DDT_VIRTUAL_QUERYFILTER,
//     so refs always use the dtPolyRef / dtTileRef typedefs
//   - upstream's builder is compiled with its sort call renamed to
//     ref_stable_qsort (defined below), so BV tree order is deterministic
//
// Two deliberate departures from a pure forward, both forced by upstream:
//   - dtQueryFilter::passFilter / getCost are defined `inline` inside
//     DetourNavMeshQuery.cpp when DT_VIRTUAL_QUERYFILTER is off, so no other
//     translation unit can link to them. In that configuration this file
//     repeats upstream's two one-line bodies through the public accessors.
//   - the dtRaycastHit overload of raycast is given a zeroed hit, because
//     upstream leaves some fields unset on some paths.

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <new>

#include "DetourAlloc.h"
#include "DetourAssert.h"
#include "DetourCommon.h"
#include "DetourMath.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourNode.h"
#include "DetourStatus.h"

// ---------------------------------------------------------------------------
// DetourMath.h
// ---------------------------------------------------------------------------

extern "C" float ref_math_fabsf(float x) { return dtMathFabsf(x); }
extern "C" float ref_math_sqrtf(float x) { return dtMathSqrtf(x); }
extern "C" float ref_math_floorf(float x) { return dtMathFloorf(x); }
extern "C" float ref_math_ceilf(float x) { return dtMathCeilf(x); }
extern "C" float ref_math_cosf(float x) { return dtMathCosf(x); }
extern "C" float ref_math_sinf(float x) { return dtMathSinf(x); }
extern "C" float ref_math_atan2f(float y, float x) { return dtMathAtan2f(y, x); }
extern "C" bool ref_math_isfinite(float x) { return dtMathIsfinite(x); }

// ---------------------------------------------------------------------------
// DetourCommon.h: general helpers (template instantiations for float and int)
// ---------------------------------------------------------------------------

extern "C" float ref_min_f32(float a, float b) { return dtMin(a, b); }
extern "C" float ref_max_f32(float a, float b) { return dtMax(a, b); }
extern "C" float ref_abs_f32(float a) { return dtAbs(a); }
extern "C" float ref_sqr_f32(float a) { return dtSqr(a); }
extern "C" float ref_clamp_f32(float v, float mn, float mx) { return dtClamp(v, mn, mx); }

extern "C" int ref_min_i32(int a, int b) { return dtMin(a, b); }
extern "C" int ref_max_i32(int a, int b) { return dtMax(a, b); }
extern "C" int ref_abs_i32(int a) { return dtAbs(a); }
extern "C" int ref_sqr_i32(int a) { return dtSqr(a); }
extern "C" int ref_clamp_i32(int v, int mn, int mx) { return dtClamp(v, mn, mx); }

extern "C" void ref_swap_f32(float* a, float* b) { dtSwap(*a, *b); }
extern "C" void ref_swap_i32(int* a, int* b) { dtSwap(*a, *b); }

// ---------------------------------------------------------------------------
// DetourCommon.h: vector helpers
// ---------------------------------------------------------------------------

extern "C" void ref_vcross(float* dest, const float* v1, const float* v2) { dtVcross(dest, v1, v2); }
extern "C" float ref_vdot(const float* v1, const float* v2) { return dtVdot(v1, v2); }
extern "C" void ref_vmad(float* dest, const float* v1, const float* v2, float s) { dtVmad(dest, v1, v2, s); }
extern "C" void ref_vlerp(float* dest, const float* v1, const float* v2, float t) { dtVlerp(dest, v1, v2, t); }
extern "C" void ref_vadd(float* dest, const float* v1, const float* v2) { dtVadd(dest, v1, v2); }
extern "C" void ref_vsub(float* dest, const float* v1, const float* v2) { dtVsub(dest, v1, v2); }
extern "C" void ref_vscale(float* dest, const float* v, float t) { dtVscale(dest, v, t); }
extern "C" void ref_vmin(float* mn, const float* v) { dtVmin(mn, v); }
extern "C" void ref_vmax(float* mx, const float* v) { dtVmax(mx, v); }
extern "C" void ref_vset(float* dest, float x, float y, float z) { dtVset(dest, x, y, z); }
extern "C" void ref_vcopy(float* dest, const float* a) { dtVcopy(dest, a); }
extern "C" float ref_vlen(const float* v) { return dtVlen(v); }
extern "C" float ref_vlen_sqr(const float* v) { return dtVlenSqr(v); }
extern "C" float ref_vdist(const float* v1, const float* v2) { return dtVdist(v1, v2); }
extern "C" float ref_vdist_sqr(const float* v1, const float* v2) { return dtVdistSqr(v1, v2); }
extern "C" float ref_vdist_2d(const float* v1, const float* v2) { return dtVdist2D(v1, v2); }
extern "C" float ref_vdist_2d_sqr(const float* v1, const float* v2) { return dtVdist2DSqr(v1, v2); }
extern "C" void ref_vnormalize(float* v) { dtVnormalize(v); }
extern "C" bool ref_vequal(const float* p0, const float* p1) { return dtVequal(p0, p1); }
extern "C" bool ref_visfinite(const float* v) { return dtVisfinite(v); }
extern "C" bool ref_visfinite_2d(const float* v) { return dtVisfinite2D(v); }
extern "C" float ref_vdot_2d(const float* u, const float* v) { return dtVdot2D(u, v); }
extern "C" float ref_vperp_2d(const float* u, const float* v) { return dtVperp2D(u, v); }

// ---------------------------------------------------------------------------
// DetourCommon.h: computational geometry
// ---------------------------------------------------------------------------

extern "C" float ref_tri_area_2d(const float* a, const float* b, const float* c)
{
	return dtTriArea2D(a, b, c);
}

extern "C" bool ref_overlap_quant_bounds(const unsigned short* amin, const unsigned short* amax,
										 const unsigned short* bmin, const unsigned short* bmax)
{
	return dtOverlapQuantBounds(amin, amax, bmin, bmax);
}

extern "C" bool ref_overlap_bounds(const float* amin, const float* amax,
								   const float* bmin, const float* bmax)
{
	return dtOverlapBounds(amin, amax, bmin, bmax);
}

extern "C" void ref_closest_pt_point_triangle(float* closest, const float* p,
											  const float* a, const float* b, const float* c)
{
	dtClosestPtPointTriangle(closest, p, a, b, c);
}

extern "C" bool ref_closest_height_point_triangle(const float* p, const float* a, const float* b,
												  const float* c, float* h)
{
	return dtClosestHeightPointTriangle(p, a, b, c, *h);
}

extern "C" bool ref_intersect_segment_poly_2d(const float* p0, const float* p1,
											  const float* verts, int nverts,
											  float* tmin, float* tmax,
											  int* seg_min, int* seg_max)
{
	return dtIntersectSegmentPoly2D(p0, p1, verts, nverts, *tmin, *tmax, *seg_min, *seg_max);
}

extern "C" bool ref_intersect_seg_seg_2d(const float* ap, const float* aq,
										 const float* bp, const float* bq,
										 float* s, float* t)
{
	return dtIntersectSegSeg2D(ap, aq, bp, bq, *s, *t);
}

extern "C" bool ref_point_in_polygon(const float* pt, const float* verts, int nverts)
{
	return dtPointInPolygon(pt, verts, nverts);
}

extern "C" bool ref_distance_pt_poly_edges_sqr(const float* pt, const float* verts, int nverts,
											   float* ed, float* et)
{
	return dtDistancePtPolyEdgesSqr(pt, verts, nverts, ed, et);
}

extern "C" float ref_distance_pt_seg_sqr_2d(const float* pt, const float* p, const float* q, float* t)
{
	return dtDistancePtSegSqr2D(pt, p, q, *t);
}

extern "C" void ref_calc_poly_center(float* tc, const unsigned short* idx, int nidx, const float* verts)
{
	dtCalcPolyCenter(tc, idx, nidx, verts);
}

extern "C" bool ref_overlap_poly_poly_2d(const float* polya, int npolya, const float* polyb, int npolyb)
{
	return dtOverlapPolyPoly2D(polya, npolya, polyb, npolyb);
}

// ---------------------------------------------------------------------------
// DetourCommon.h: miscellaneous
// ---------------------------------------------------------------------------

extern "C" unsigned int ref_next_pow2(unsigned int v) { return dtNextPow2(v); }
extern "C" unsigned int ref_ilog2(unsigned int v) { return dtIlog2(v); }
extern "C" int ref_align4(int x) { return dtAlign4(x); }
extern "C" int ref_opposite_tile(int side) { return dtOppositeTile(side); }

extern "C" void ref_swap_byte(unsigned char* a, unsigned char* b) { dtSwapByte(a, b); }
extern "C" void ref_swap_endian_u16(unsigned short* v) { dtSwapEndian(v); }
extern "C" void ref_swap_endian_i16(short* v) { dtSwapEndian(v); }
extern "C" void ref_swap_endian_u32(unsigned int* v) { dtSwapEndian(v); }
extern "C" void ref_swap_endian_i32(int* v) { dtSwapEndian(v); }
extern "C" void ref_swap_endian_f32(float* v) { dtSwapEndian(v); }

extern "C" void ref_random_point_in_convex_poly(const float* pts, int npts, float* areas,
												float s, float t, float* out)
{
	dtRandomPointInConvexPoly(pts, npts, areas, s, t, out);
}

// Returns the old position of *buffer and advances *buffer by distance bytes.
extern "C" unsigned char* ref_get_then_advance_buffer_pointer(unsigned char** buffer, size_t distance)
{
	return dtGetThenAdvanceBufferPointer<unsigned char>(*buffer, distance);
}

extern "C" const unsigned char* ref_get_then_advance_buffer_pointer_const(const unsigned char** buffer,
																		  size_t distance)
{
	return dtGetThenAdvanceBufferPointer<const unsigned char>(*buffer, distance);
}

// ---------------------------------------------------------------------------
// DetourStatus.h
// ---------------------------------------------------------------------------

extern "C" bool ref_status_failed(dtStatus status) { return dtStatusFailed(status); }
extern "C" bool ref_status_succeeded(dtStatus status) { return dtStatusSucceed(status); }
extern "C" bool ref_status_in_progress(dtStatus status) { return dtStatusInProgress(status); }
extern "C" bool ref_status_detail(dtStatus status, unsigned int detail) { return dtStatusDetail(status, detail); }

// ---------------------------------------------------------------------------
// DetourStatus.h, DetourNavMesh.h, DetourNode.h: constants
// ---------------------------------------------------------------------------

#define REF_CONST_U32(NAME, EXPR) extern "C" unsigned int ref_const_##NAME() { return (unsigned int)(EXPR); }
#define REF_CONST_I32(NAME, EXPR) extern "C" int ref_const_##NAME() { return (int)(EXPR); }

REF_CONST_U32(failure, DT_FAILURE)
REF_CONST_U32(success, DT_SUCCESS)
REF_CONST_U32(in_progress, DT_IN_PROGRESS)
REF_CONST_U32(status_detail_mask, DT_STATUS_DETAIL_MASK)
REF_CONST_U32(wrong_magic, DT_WRONG_MAGIC)
REF_CONST_U32(wrong_version, DT_WRONG_VERSION)
REF_CONST_U32(out_of_memory, DT_OUT_OF_MEMORY)
REF_CONST_U32(invalid_param, DT_INVALID_PARAM)
REF_CONST_U32(buffer_too_small, DT_BUFFER_TOO_SMALL)
REF_CONST_U32(out_of_nodes, DT_OUT_OF_NODES)
REF_CONST_U32(partial_result, DT_PARTIAL_RESULT)
REF_CONST_U32(already_occupied, DT_ALREADY_OCCUPIED)

REF_CONST_I32(verts_per_polygon, DT_VERTS_PER_POLYGON)
REF_CONST_I32(navmesh_magic, DT_NAVMESH_MAGIC)
REF_CONST_I32(navmesh_version, DT_NAVMESH_VERSION)
REF_CONST_I32(navmesh_state_magic, DT_NAVMESH_STATE_MAGIC)
REF_CONST_I32(navmesh_state_version, DT_NAVMESH_STATE_VERSION)
REF_CONST_U32(ext_link, DT_EXT_LINK)
REF_CONST_U32(null_link, DT_NULL_LINK)
REF_CONST_U32(offmesh_con_bidir, DT_OFFMESH_CON_BIDIR)
REF_CONST_I32(max_areas, DT_MAX_AREAS)
REF_CONST_U32(tile_free_data, DT_TILE_FREE_DATA)
REF_CONST_U32(straightpath_start, DT_STRAIGHTPATH_START)
REF_CONST_U32(straightpath_end, DT_STRAIGHTPATH_END)
REF_CONST_U32(straightpath_offmesh_connection, DT_STRAIGHTPATH_OFFMESH_CONNECTION)
REF_CONST_U32(straightpath_area_crossings, DT_STRAIGHTPATH_AREA_CROSSINGS)
REF_CONST_U32(straightpath_all_crossings, DT_STRAIGHTPATH_ALL_CROSSINGS)
REF_CONST_U32(findpath_any_angle, DT_FINDPATH_ANY_ANGLE)
REF_CONST_U32(raycast_use_costs, DT_RAYCAST_USE_COSTS)
REF_CONST_U32(detail_edge_boundary, DT_DETAIL_EDGE_BOUNDARY)
REF_CONST_U32(polytype_ground, DT_POLYTYPE_GROUND)
REF_CONST_U32(polytype_offmesh_connection, DT_POLYTYPE_OFFMESH_CONNECTION)

REF_CONST_U32(node_open, DT_NODE_OPEN)
REF_CONST_U32(node_closed, DT_NODE_CLOSED)
REF_CONST_U32(node_parent_detached, DT_NODE_PARENT_DETACHED)
REF_CONST_U32(null_idx, DT_NULL_IDX)
REF_CONST_I32(node_parent_bits, DT_NODE_PARENT_BITS)
REF_CONST_I32(node_state_bits, DT_NODE_STATE_BITS)
REF_CONST_I32(max_states_per_node, DT_MAX_STATES_PER_NODE)

extern "C" float ref_const_ray_cast_limit_proportions() { return DT_RAY_CAST_LIMIT_PROPORTIONS; }

// ---------------------------------------------------------------------------
// DetourAlloc.h, DetourAssert.h
// ---------------------------------------------------------------------------

extern "C" void ref_alloc_set_custom(dtAllocFunc* alloc_fn, dtFreeFunc* free_fn)
{
	dtAllocSetCustom(alloc_fn, free_fn);
}

extern "C" void* ref_alloc(size_t size, dtAllocHint hint) { return dtAlloc(size, hint); }
extern "C" void ref_free(void* ptr) { dtFree(ptr); }

#ifndef RC_DISABLE_ASSERTS
extern "C" void ref_assert_fail_set_custom(dtAssertFailFunc* fn) { dtAssertFailSetCustom(fn); }
extern "C" dtAssertFailFunc* ref_assert_fail_get_custom() { return dtAssertFailGetCustom(); }
#else
// Assertions are compiled out upstream: there is nothing to hook.
extern "C" void ref_assert_fail_set_custom(void (*fn)(const char*, const char*, int)) { (void)fn; }
extern "C" void (*ref_assert_fail_get_custom())(const char*, const char*, int) { return 0; }
#endif

// ---------------------------------------------------------------------------
// Stable sort. The integrator renames upstream's builder sort call to this
// function, which has libc qsort's exact signature. Nothing in this file may
// call libc's sort by its own name, because that name may be renamed to this
// function.
//
// Elements that compare equal keep their input order. Merge sort over a
// temporary buffer; if the buffer cannot be had, an in-place insertion sort
// (adjacent swaps, strict greater-than) gives the same order.
// ---------------------------------------------------------------------------

static void ref_swap_bytes(unsigned char* a, unsigned char* b, size_t size)
{
	for (size_t i = 0; i < size; ++i)
	{
		const unsigned char t = a[i];
		a[i] = b[i];
		b[i] = t;
	}
}

extern "C" void ref_stable_qsort(void* base, size_t count, size_t size,
								 int (*compare)(const void*, const void*))
{
	if (count < 2 || size == 0)
		return;

	unsigned char* a = (unsigned char*)base;
	unsigned char* tmp = (unsigned char*)dtAlloc(count * size, DT_ALLOC_TEMP);
	if (!tmp)
	{
		for (size_t i = 1; i < count; ++i)
		{
			for (size_t j = i; j > 0; --j)
			{
				unsigned char* left = a + (j - 1) * size;
				unsigned char* right = a + j * size;
				if (compare(left, right) <= 0)
					break;
				ref_swap_bytes(left, right, size);
			}
		}
		return;
	}

	unsigned char* src = a;
	unsigned char* dst = tmp;
	for (size_t width = 1; width < count; width *= 2)
	{
		for (size_t lo = 0; lo < count; lo += 2 * width)
		{
			size_t mid = lo + width;
			if (mid > count)
				mid = count;
			size_t hi = lo + 2 * width;
			if (hi > count)
				hi = count;

			size_t i = lo;
			size_t j = mid;
			size_t k = lo;
			while (i < mid && j < hi)
			{
				// The left element wins ties, which keeps the sort stable.
				if (compare(src + i * size, src + j * size) <= 0)
				{
					memcpy(dst + k * size, src + i * size, size);
					++i;
				}
				else
				{
					memcpy(dst + k * size, src + j * size, size);
					++j;
				}
				++k;
			}
			if (i < mid)
				memcpy(dst + k * size, src + i * size, (mid - i) * size);
			else if (j < hi)
				memcpy(dst + k * size, src + j * size, (hi - j) * size);
		}

		unsigned char* swap = src;
		src = dst;
		dst = swap;
	}

	if (src != a)
		memcpy(a, src, count * size);
	dtFree(tmp);
}

// ---------------------------------------------------------------------------
// DetourNavMeshBuilder.h
// ---------------------------------------------------------------------------

// Upstream takes a mutable params pointer; a copy keeps the caller's const.
extern "C" bool ref_create_nav_mesh_data(const dtNavMeshCreateParams* params,
										 unsigned char** out_data, int* out_data_size)
{
	dtNavMeshCreateParams local = *params;
	return dtCreateNavMeshData(&local, out_data, out_data_size);
}

extern "C" bool ref_nav_mesh_header_swap_endian(unsigned char* data, int data_size)
{
	return dtNavMeshHeaderSwapEndian(data, data_size);
}

extern "C" bool ref_nav_mesh_data_swap_endian(unsigned char* data, int data_size)
{
	return dtNavMeshDataSwapEndian(data, data_size);
}

// ---------------------------------------------------------------------------
// Layout checks: sizes and field offsets of the structs the Zig port mirrors.
// dtNode has bit-fields, so only its size and its plain members are covered;
// its bit-fields have getters and setters further down.
// ---------------------------------------------------------------------------

#define REF_SIZEOF(NAME, TYPE) extern "C" size_t ref_sizeof_##NAME() { return sizeof(TYPE); }
#define REF_OFFSETOF(NAME, TYPE, FIELD) extern "C" size_t ref_offsetof_##NAME() { return offsetof(TYPE, FIELD); }

REF_SIZEOF(poly_ref, dtPolyRef)
REF_SIZEOF(tile_ref, dtTileRef)

REF_SIZEOF(poly, dtPoly)
REF_OFFSETOF(poly_first_link, dtPoly, firstLink)
REF_OFFSETOF(poly_verts, dtPoly, verts)
REF_OFFSETOF(poly_neis, dtPoly, neis)
REF_OFFSETOF(poly_flags, dtPoly, flags)
REF_OFFSETOF(poly_vert_count, dtPoly, vertCount)
REF_OFFSETOF(poly_area_and_type, dtPoly, areaAndtype)

REF_SIZEOF(poly_detail, dtPolyDetail)
REF_OFFSETOF(poly_detail_vert_base, dtPolyDetail, vertBase)
REF_OFFSETOF(poly_detail_tri_base, dtPolyDetail, triBase)
REF_OFFSETOF(poly_detail_vert_count, dtPolyDetail, vertCount)
REF_OFFSETOF(poly_detail_tri_count, dtPolyDetail, triCount)

REF_SIZEOF(link, dtLink)
REF_OFFSETOF(link_ref, dtLink, ref)
REF_OFFSETOF(link_next, dtLink, next)
REF_OFFSETOF(link_edge, dtLink, edge)
REF_OFFSETOF(link_side, dtLink, side)
REF_OFFSETOF(link_bmin, dtLink, bmin)
REF_OFFSETOF(link_bmax, dtLink, bmax)

REF_SIZEOF(bv_node, dtBVNode)
REF_OFFSETOF(bv_node_bmin, dtBVNode, bmin)
REF_OFFSETOF(bv_node_bmax, dtBVNode, bmax)
REF_OFFSETOF(bv_node_i, dtBVNode, i)

REF_SIZEOF(off_mesh_connection, dtOffMeshConnection)
REF_OFFSETOF(off_mesh_connection_pos, dtOffMeshConnection, pos)
REF_OFFSETOF(off_mesh_connection_rad, dtOffMeshConnection, rad)
REF_OFFSETOF(off_mesh_connection_poly, dtOffMeshConnection, poly)
REF_OFFSETOF(off_mesh_connection_flags, dtOffMeshConnection, flags)
REF_OFFSETOF(off_mesh_connection_side, dtOffMeshConnection, side)
REF_OFFSETOF(off_mesh_connection_user_id, dtOffMeshConnection, userId)

REF_SIZEOF(mesh_header, dtMeshHeader)
REF_OFFSETOF(mesh_header_magic, dtMeshHeader, magic)
REF_OFFSETOF(mesh_header_version, dtMeshHeader, version)
REF_OFFSETOF(mesh_header_x, dtMeshHeader, x)
REF_OFFSETOF(mesh_header_y, dtMeshHeader, y)
REF_OFFSETOF(mesh_header_layer, dtMeshHeader, layer)
REF_OFFSETOF(mesh_header_user_id, dtMeshHeader, userId)
REF_OFFSETOF(mesh_header_poly_count, dtMeshHeader, polyCount)
REF_OFFSETOF(mesh_header_vert_count, dtMeshHeader, vertCount)
REF_OFFSETOF(mesh_header_max_link_count, dtMeshHeader, maxLinkCount)
REF_OFFSETOF(mesh_header_detail_mesh_count, dtMeshHeader, detailMeshCount)
REF_OFFSETOF(mesh_header_detail_vert_count, dtMeshHeader, detailVertCount)
REF_OFFSETOF(mesh_header_detail_tri_count, dtMeshHeader, detailTriCount)
REF_OFFSETOF(mesh_header_bv_node_count, dtMeshHeader, bvNodeCount)
REF_OFFSETOF(mesh_header_off_mesh_con_count, dtMeshHeader, offMeshConCount)
REF_OFFSETOF(mesh_header_off_mesh_base, dtMeshHeader, offMeshBase)
REF_OFFSETOF(mesh_header_walkable_height, dtMeshHeader, walkableHeight)
REF_OFFSETOF(mesh_header_walkable_radius, dtMeshHeader, walkableRadius)
REF_OFFSETOF(mesh_header_walkable_climb, dtMeshHeader, walkableClimb)
REF_OFFSETOF(mesh_header_bmin, dtMeshHeader, bmin)
REF_OFFSETOF(mesh_header_bmax, dtMeshHeader, bmax)
REF_OFFSETOF(mesh_header_bv_quant_factor, dtMeshHeader, bvQuantFactor)

REF_SIZEOF(mesh_tile, dtMeshTile)
REF_OFFSETOF(mesh_tile_salt, dtMeshTile, salt)
REF_OFFSETOF(mesh_tile_links_free_list, dtMeshTile, linksFreeList)
REF_OFFSETOF(mesh_tile_header, dtMeshTile, header)
REF_OFFSETOF(mesh_tile_polys, dtMeshTile, polys)
REF_OFFSETOF(mesh_tile_verts, dtMeshTile, verts)
REF_OFFSETOF(mesh_tile_links, dtMeshTile, links)
REF_OFFSETOF(mesh_tile_detail_meshes, dtMeshTile, detailMeshes)
REF_OFFSETOF(mesh_tile_detail_verts, dtMeshTile, detailVerts)
REF_OFFSETOF(mesh_tile_detail_tris, dtMeshTile, detailTris)
REF_OFFSETOF(mesh_tile_bv_tree, dtMeshTile, bvTree)
REF_OFFSETOF(mesh_tile_off_mesh_cons, dtMeshTile, offMeshCons)
REF_OFFSETOF(mesh_tile_data, dtMeshTile, data)
REF_OFFSETOF(mesh_tile_data_size, dtMeshTile, dataSize)
REF_OFFSETOF(mesh_tile_flags, dtMeshTile, flags)
REF_OFFSETOF(mesh_tile_next, dtMeshTile, next)

REF_SIZEOF(nav_mesh_params, dtNavMeshParams)
REF_OFFSETOF(nav_mesh_params_orig, dtNavMeshParams, orig)
REF_OFFSETOF(nav_mesh_params_tile_width, dtNavMeshParams, tileWidth)
REF_OFFSETOF(nav_mesh_params_tile_height, dtNavMeshParams, tileHeight)
REF_OFFSETOF(nav_mesh_params_max_tiles, dtNavMeshParams, maxTiles)
REF_OFFSETOF(nav_mesh_params_max_polys, dtNavMeshParams, maxPolys)

REF_SIZEOF(nav_mesh_create_params, dtNavMeshCreateParams)
REF_OFFSETOF(nav_mesh_create_params_verts, dtNavMeshCreateParams, verts)
REF_OFFSETOF(nav_mesh_create_params_vert_count, dtNavMeshCreateParams, vertCount)
REF_OFFSETOF(nav_mesh_create_params_polys, dtNavMeshCreateParams, polys)
REF_OFFSETOF(nav_mesh_create_params_poly_flags, dtNavMeshCreateParams, polyFlags)
REF_OFFSETOF(nav_mesh_create_params_poly_areas, dtNavMeshCreateParams, polyAreas)
REF_OFFSETOF(nav_mesh_create_params_poly_count, dtNavMeshCreateParams, polyCount)
REF_OFFSETOF(nav_mesh_create_params_nvp, dtNavMeshCreateParams, nvp)
REF_OFFSETOF(nav_mesh_create_params_detail_meshes, dtNavMeshCreateParams, detailMeshes)
REF_OFFSETOF(nav_mesh_create_params_detail_verts, dtNavMeshCreateParams, detailVerts)
REF_OFFSETOF(nav_mesh_create_params_detail_verts_count, dtNavMeshCreateParams, detailVertsCount)
REF_OFFSETOF(nav_mesh_create_params_detail_tris, dtNavMeshCreateParams, detailTris)
REF_OFFSETOF(nav_mesh_create_params_detail_tri_count, dtNavMeshCreateParams, detailTriCount)
REF_OFFSETOF(nav_mesh_create_params_off_mesh_con_verts, dtNavMeshCreateParams, offMeshConVerts)
REF_OFFSETOF(nav_mesh_create_params_off_mesh_con_rad, dtNavMeshCreateParams, offMeshConRad)
REF_OFFSETOF(nav_mesh_create_params_off_mesh_con_flags, dtNavMeshCreateParams, offMeshConFlags)
REF_OFFSETOF(nav_mesh_create_params_off_mesh_con_areas, dtNavMeshCreateParams, offMeshConAreas)
REF_OFFSETOF(nav_mesh_create_params_off_mesh_con_dir, dtNavMeshCreateParams, offMeshConDir)
REF_OFFSETOF(nav_mesh_create_params_off_mesh_con_user_id, dtNavMeshCreateParams, offMeshConUserID)
REF_OFFSETOF(nav_mesh_create_params_off_mesh_con_count, dtNavMeshCreateParams, offMeshConCount)
REF_OFFSETOF(nav_mesh_create_params_user_id, dtNavMeshCreateParams, userId)
REF_OFFSETOF(nav_mesh_create_params_tile_x, dtNavMeshCreateParams, tileX)
REF_OFFSETOF(nav_mesh_create_params_tile_y, dtNavMeshCreateParams, tileY)
REF_OFFSETOF(nav_mesh_create_params_tile_layer, dtNavMeshCreateParams, tileLayer)
REF_OFFSETOF(nav_mesh_create_params_bmin, dtNavMeshCreateParams, bmin)
REF_OFFSETOF(nav_mesh_create_params_bmax, dtNavMeshCreateParams, bmax)
REF_OFFSETOF(nav_mesh_create_params_walkable_height, dtNavMeshCreateParams, walkableHeight)
REF_OFFSETOF(nav_mesh_create_params_walkable_radius, dtNavMeshCreateParams, walkableRadius)
REF_OFFSETOF(nav_mesh_create_params_walkable_climb, dtNavMeshCreateParams, walkableClimb)
REF_OFFSETOF(nav_mesh_create_params_cs, dtNavMeshCreateParams, cs)
REF_OFFSETOF(nav_mesh_create_params_ch, dtNavMeshCreateParams, ch)
REF_OFFSETOF(nav_mesh_create_params_build_bv_tree, dtNavMeshCreateParams, buildBvTree)

REF_SIZEOF(raycast_hit, dtRaycastHit)
REF_OFFSETOF(raycast_hit_t, dtRaycastHit, t)
REF_OFFSETOF(raycast_hit_hit_normal, dtRaycastHit, hitNormal)
REF_OFFSETOF(raycast_hit_hit_edge_index, dtRaycastHit, hitEdgeIndex)
REF_OFFSETOF(raycast_hit_path, dtRaycastHit, path)
REF_OFFSETOF(raycast_hit_path_count, dtRaycastHit, pathCount)
REF_OFFSETOF(raycast_hit_max_path, dtRaycastHit, maxPath)
REF_OFFSETOF(raycast_hit_path_cost, dtRaycastHit, pathCost)

REF_SIZEOF(node, dtNode)
REF_OFFSETOF(node_pos, dtNode, pos)
REF_OFFSETOF(node_cost, dtNode, cost)
REF_OFFSETOF(node_total, dtNode, total)
REF_OFFSETOF(node_id, dtNode, id)

// getMemUsed() of the pool and the queue include these.
REF_SIZEOF(node_pool, dtNodePool)
REF_SIZEOF(node_queue, dtNodeQueue)
REF_SIZEOF(query_filter, dtQueryFilter)

// ---------------------------------------------------------------------------
// dtPoly, dtMeshTile, dtGetDetailTriEdgeFlags
// ---------------------------------------------------------------------------

extern "C" void ref_poly_set_area(dtPoly* poly, unsigned char area) { poly->setArea(area); }
extern "C" void ref_poly_set_type(dtPoly* poly, unsigned char type) { poly->setType(type); }
extern "C" unsigned char ref_poly_get_area(const dtPoly* poly) { return poly->getArea(); }
extern "C" unsigned char ref_poly_get_type(const dtPoly* poly) { return poly->getType(); }

extern "C" int ref_get_detail_tri_edge_flags(unsigned char tri_flags, int edge_index)
{
	return dtGetDetailTriEdgeFlags(tri_flags, edge_index);
}

// dtMeshTile holds pointers; these hand back each array. A tile that is not
// populated has null arrays, and its header polyCount is zero.
extern "C" const dtMeshHeader* ref_tile_header(const dtMeshTile* tile) { return tile->header; }
extern "C" const dtPoly* ref_tile_polys(const dtMeshTile* tile) { return tile->polys; }
extern "C" const float* ref_tile_verts(const dtMeshTile* tile) { return tile->verts; }
extern "C" const dtLink* ref_tile_links(const dtMeshTile* tile) { return tile->links; }
extern "C" const dtPolyDetail* ref_tile_detail_meshes(const dtMeshTile* tile) { return tile->detailMeshes; }
extern "C" const float* ref_tile_detail_verts(const dtMeshTile* tile) { return tile->detailVerts; }
extern "C" const unsigned char* ref_tile_detail_tris(const dtMeshTile* tile) { return tile->detailTris; }
extern "C" const dtBVNode* ref_tile_bv_tree(const dtMeshTile* tile) { return tile->bvTree; }
extern "C" const dtOffMeshConnection* ref_tile_off_mesh_cons(const dtMeshTile* tile) { return tile->offMeshCons; }
extern "C" const unsigned char* ref_tile_data(const dtMeshTile* tile) { return tile->data; }
extern "C" int ref_tile_data_size(const dtMeshTile* tile) { return tile->dataSize; }
extern "C" int ref_tile_flags(const dtMeshTile* tile) { return tile->flags; }
extern "C" unsigned int ref_tile_salt(const dtMeshTile* tile) { return tile->salt; }
extern "C" unsigned int ref_tile_links_free_list(const dtMeshTile* tile) { return tile->linksFreeList; }
extern "C" const dtMeshTile* ref_tile_next(const dtMeshTile* tile) { return tile->next; }

// ---------------------------------------------------------------------------
// dtNavMesh
// ---------------------------------------------------------------------------

extern "C" dtNavMesh* ref_nav_mesh_alloc() { return dtAllocNavMesh(); }
extern "C" void ref_nav_mesh_free(dtNavMesh* nav) { dtFreeNavMesh(nav); }

// init(const dtNavMeshParams*)
extern "C" dtStatus ref_nav_mesh_init(dtNavMesh* nav, const dtNavMeshParams* params)
{
	return nav->init(params);
}

// init(unsigned char* data, int dataSize, int flags)
extern "C" dtStatus ref_nav_mesh_init_single(dtNavMesh* nav, unsigned char* data, int data_size, int flags)
{
	return nav->init(data, data_size, flags);
}

extern "C" const dtNavMeshParams* ref_nav_mesh_get_params(const dtNavMesh* nav)
{
	return nav->getParams();
}

extern "C" dtStatus ref_nav_mesh_add_tile(dtNavMesh* nav, unsigned char* data, int data_size, int flags,
										  dtTileRef last_ref, dtTileRef* result)
{
	return nav->addTile(data, data_size, flags, last_ref, result);
}

extern "C" dtStatus ref_nav_mesh_remove_tile(dtNavMesh* nav, dtTileRef ref, unsigned char** data, int* data_size)
{
	return nav->removeTile(ref, data, data_size);
}

extern "C" void ref_nav_mesh_calc_tile_loc(const dtNavMesh* nav, const float* pos, int* tx, int* ty)
{
	nav->calcTileLoc(pos, tx, ty);
}

extern "C" const dtMeshTile* ref_nav_mesh_get_tile_at(const dtNavMesh* nav, int x, int y, int layer)
{
	return nav->getTileAt(x, y, layer);
}

extern "C" int ref_nav_mesh_get_tiles_at(const dtNavMesh* nav, int x, int y,
										 const dtMeshTile** tiles, int max_tiles)
{
	return nav->getTilesAt(x, y, tiles, max_tiles);
}

extern "C" dtTileRef ref_nav_mesh_get_tile_ref_at(const dtNavMesh* nav, int x, int y, int layer)
{
	return nav->getTileRefAt(x, y, layer);
}

extern "C" dtTileRef ref_nav_mesh_get_tile_ref(const dtNavMesh* nav, const dtMeshTile* tile)
{
	return nav->getTileRef(tile);
}

extern "C" const dtMeshTile* ref_nav_mesh_get_tile_by_ref(const dtNavMesh* nav, dtTileRef ref)
{
	return nav->getTileByRef(ref);
}

extern "C" int ref_nav_mesh_get_max_tiles(const dtNavMesh* nav) { return nav->getMaxTiles(); }

// The mutable getTile(int) is private; the const one is public. `nav` is const
// here, so overload resolution picks the public one.
extern "C" const dtMeshTile* ref_nav_mesh_get_tile(const dtNavMesh* nav, int i) { return nav->getTile(i); }

extern "C" dtStatus ref_nav_mesh_get_tile_and_poly_by_ref(const dtNavMesh* nav, dtPolyRef ref,
														  const dtMeshTile** tile, const dtPoly** poly)
{
	return nav->getTileAndPolyByRef(ref, tile, poly);
}

extern "C" void ref_nav_mesh_get_tile_and_poly_by_ref_unsafe(const dtNavMesh* nav, dtPolyRef ref,
															 const dtMeshTile** tile, const dtPoly** poly)
{
	nav->getTileAndPolyByRefUnsafe(ref, tile, poly);
}

extern "C" bool ref_nav_mesh_is_valid_poly_ref(const dtNavMesh* nav, dtPolyRef ref)
{
	return nav->isValidPolyRef(ref);
}

extern "C" dtPolyRef ref_nav_mesh_get_poly_ref_base(const dtNavMesh* nav, const dtMeshTile* tile)
{
	return nav->getPolyRefBase(tile);
}

extern "C" dtStatus ref_nav_mesh_get_off_mesh_connection_poly_end_points(const dtNavMesh* nav,
																		  dtPolyRef prev_ref, dtPolyRef poly_ref,
																		  float* start_pos, float* end_pos)
{
	return nav->getOffMeshConnectionPolyEndPoints(prev_ref, poly_ref, start_pos, end_pos);
}

extern "C" const dtOffMeshConnection* ref_nav_mesh_get_off_mesh_connection_by_ref(const dtNavMesh* nav,
																				   dtPolyRef ref)
{
	return nav->getOffMeshConnectionByRef(ref);
}

extern "C" dtStatus ref_nav_mesh_set_poly_flags(dtNavMesh* nav, dtPolyRef ref, unsigned short flags)
{
	return nav->setPolyFlags(ref, flags);
}

extern "C" dtStatus ref_nav_mesh_get_poly_flags(const dtNavMesh* nav, dtPolyRef ref, unsigned short* result_flags)
{
	return nav->getPolyFlags(ref, result_flags);
}

extern "C" dtStatus ref_nav_mesh_set_poly_area(dtNavMesh* nav, dtPolyRef ref, unsigned char area)
{
	return nav->setPolyArea(ref, area);
}

extern "C" dtStatus ref_nav_mesh_get_poly_area(const dtNavMesh* nav, dtPolyRef ref, unsigned char* result_area)
{
	return nav->getPolyArea(ref, result_area);
}

extern "C" int ref_nav_mesh_get_tile_state_size(const dtNavMesh* nav, const dtMeshTile* tile)
{
	return nav->getTileStateSize(tile);
}

extern "C" dtStatus ref_nav_mesh_store_tile_state(const dtNavMesh* nav, const dtMeshTile* tile,
												  unsigned char* data, int max_data_size)
{
	return nav->storeTileState(tile, data, max_data_size);
}

extern "C" dtStatus ref_nav_mesh_restore_tile_state(dtNavMesh* nav, dtMeshTile* tile,
													const unsigned char* data, int max_data_size)
{
	return nav->restoreTileState(tile, data, max_data_size);
}

extern "C" dtPolyRef ref_nav_mesh_encode_poly_id(const dtNavMesh* nav, unsigned int salt,
												 unsigned int it, unsigned int ip)
{
	return nav->encodePolyId(salt, it, ip);
}

extern "C" void ref_nav_mesh_decode_poly_id(const dtNavMesh* nav, dtPolyRef ref,
											unsigned int* salt, unsigned int* it, unsigned int* ip)
{
	nav->decodePolyId(ref, *salt, *it, *ip);
}

extern "C" unsigned int ref_nav_mesh_decode_poly_id_salt(const dtNavMesh* nav, dtPolyRef ref)
{
	return nav->decodePolyIdSalt(ref);
}

extern "C" unsigned int ref_nav_mesh_decode_poly_id_tile(const dtNavMesh* nav, dtPolyRef ref)
{
	return nav->decodePolyIdTile(ref);
}

extern "C" unsigned int ref_nav_mesh_decode_poly_id_poly(const dtNavMesh* nav, dtPolyRef ref)
{
	return nav->decodePolyIdPoly(ref);
}

// ---------------------------------------------------------------------------
// dtQueryFilter
// ---------------------------------------------------------------------------

typedef bool (*RefFilterPassFn)(void* user, dtPolyRef ref, const dtMeshTile* tile, const dtPoly* poly);
typedef float (*RefFilterCostFn)(void* user, const float* pa, const float* pb,
								 dtPolyRef prev_ref, const dtMeshTile* prev_tile, const dtPoly* prev_poly,
								 dtPolyRef cur_ref, const dtMeshTile* cur_tile, const dtPoly* cur_poly,
								 dtPolyRef next_ref, const dtMeshTile* next_tile, const dtPoly* next_poly);

#ifdef DT_VIRTUAL_QUERYFILTER
namespace
{

// A filter whose two virtual functions call back into the caller. A null
// callback falls back to the default implementation for that function.
class RefCustomFilter : public dtQueryFilter
{
public:
	RefCustomFilter(RefFilterPassFn pass, RefFilterCostFn cost, void* user) :
		m_pass(pass), m_cost(cost), m_user(user)
	{
	}

	virtual ~RefCustomFilter() {}

	virtual bool passFilter(const dtPolyRef ref, const dtMeshTile* tile, const dtPoly* poly) const
	{
		if (!m_pass)
			return dtQueryFilter::passFilter(ref, tile, poly);
		return m_pass(m_user, ref, tile, poly);
	}

	virtual float getCost(const float* pa, const float* pb,
						  const dtPolyRef prevRef, const dtMeshTile* prevTile, const dtPoly* prevPoly,
						  const dtPolyRef curRef, const dtMeshTile* curTile, const dtPoly* curPoly,
						  const dtPolyRef nextRef, const dtMeshTile* nextTile, const dtPoly* nextPoly) const
	{
		if (!m_cost)
			return dtQueryFilter::getCost(pa, pb, prevRef, prevTile, prevPoly, curRef, curTile, curPoly,
										  nextRef, nextTile, nextPoly);
		return m_cost(m_user, pa, pb, prevRef, prevTile, prevPoly, curRef, curTile, curPoly,
					  nextRef, nextTile, nextPoly);
	}

private:
	RefFilterPassFn m_pass;
	RefFilterCostFn m_cost;
	void* m_user;
};

} // namespace
#endif

extern "C" dtQueryFilter* ref_filter_create()
{
	void* mem = dtAlloc(sizeof(dtQueryFilter), DT_ALLOC_PERM);
	if (!mem)
		return 0;
	return new (mem) dtQueryFilter();
}

// Without DT_VIRTUAL_QUERYFILTER a custom filter cannot exist: returns null.
extern "C" dtQueryFilter* ref_filter_create_custom(RefFilterPassFn pass, RefFilterCostFn cost, void* user)
{
#ifdef DT_VIRTUAL_QUERYFILTER
	void* mem = dtAlloc(sizeof(RefCustomFilter), DT_ALLOC_PERM);
	if (!mem)
		return 0;
	return new (mem) RefCustomFilter(pass, cost, user);
#else
	(void)pass;
	(void)cost;
	(void)user;
	return 0;
#endif
}

// Safe for both kinds of filter: with virtual filters the destructor is virtual.
extern "C" void ref_filter_destroy(dtQueryFilter* filter)
{
	if (!filter)
		return;
	filter->~dtQueryFilter();
	dtFree(filter);
}

extern "C" float ref_filter_get_area_cost(const dtQueryFilter* filter, int i) { return filter->getAreaCost(i); }
extern "C" void ref_filter_set_area_cost(dtQueryFilter* filter, int i, float cost) { filter->setAreaCost(i, cost); }
extern "C" unsigned short ref_filter_get_include_flags(const dtQueryFilter* filter) { return filter->getIncludeFlags(); }
extern "C" void ref_filter_set_include_flags(dtQueryFilter* filter, unsigned short flags) { filter->setIncludeFlags(flags); }
extern "C" unsigned short ref_filter_get_exclude_flags(const dtQueryFilter* filter) { return filter->getExcludeFlags(); }
extern "C" void ref_filter_set_exclude_flags(dtQueryFilter* filter, unsigned short flags) { filter->setExcludeFlags(flags); }

extern "C" bool ref_filter_pass_filter(const dtQueryFilter* filter, dtPolyRef ref,
									   const dtMeshTile* tile, const dtPoly* poly)
{
#ifdef DT_VIRTUAL_QUERYFILTER
	return filter->passFilter(ref, tile, poly);
#else
	// Same expression as upstream's default passFilter (see the file header).
	(void)ref;
	(void)tile;
	return (poly->flags & filter->getIncludeFlags()) != 0 && (poly->flags & filter->getExcludeFlags()) == 0;
#endif
}

extern "C" float ref_filter_get_cost(const dtQueryFilter* filter, const float* pa, const float* pb,
									 dtPolyRef prev_ref, const dtMeshTile* prev_tile, const dtPoly* prev_poly,
									 dtPolyRef cur_ref, const dtMeshTile* cur_tile, const dtPoly* cur_poly,
									 dtPolyRef next_ref, const dtMeshTile* next_tile, const dtPoly* next_poly)
{
#ifdef DT_VIRTUAL_QUERYFILTER
	return filter->getCost(pa, pb, prev_ref, prev_tile, prev_poly, cur_ref, cur_tile, cur_poly,
						   next_ref, next_tile, next_poly);
#else
	// Same expression as upstream's default getCost (see the file header).
	(void)prev_ref;
	(void)prev_tile;
	(void)prev_poly;
	(void)cur_ref;
	(void)cur_tile;
	(void)next_ref;
	(void)next_tile;
	(void)next_poly;
	return dtVdist(pa, pb) * filter->getAreaCost(cur_poly->getArea());
#endif
}

// ---------------------------------------------------------------------------
// dtNavMeshQuery
// ---------------------------------------------------------------------------

extern "C" dtNavMeshQuery* ref_query_alloc() { return dtAllocNavMeshQuery(); }
extern "C" void ref_query_free(dtNavMeshQuery* query) { dtFreeNavMeshQuery(query); }

extern "C" dtStatus ref_query_init(dtNavMeshQuery* query, const dtNavMesh* nav, int max_nodes)
{
	return query->init(nav, max_nodes);
}

extern "C" dtStatus ref_query_find_path(const dtNavMeshQuery* query, dtPolyRef start_ref, dtPolyRef end_ref,
										const float* start_pos, const float* end_pos,
										const dtQueryFilter* filter,
										dtPolyRef* path, int* path_count, int max_path)
{
	return query->findPath(start_ref, end_ref, start_pos, end_pos, filter, path, path_count, max_path);
}

extern "C" dtStatus ref_query_find_straight_path(const dtNavMeshQuery* query,
												 const float* start_pos, const float* end_pos,
												 const dtPolyRef* path, int path_size,
												 float* straight_path, unsigned char* straight_path_flags,
												 dtPolyRef* straight_path_refs,
												 int* straight_path_count, int max_straight_path, int options)
{
	return query->findStraightPath(start_pos, end_pos, path, path_size, straight_path, straight_path_flags,
								   straight_path_refs, straight_path_count, max_straight_path, options);
}

extern "C" dtStatus ref_query_init_sliced_find_path(dtNavMeshQuery* query,
													dtPolyRef start_ref, dtPolyRef end_ref,
													const float* start_pos, const float* end_pos,
													const dtQueryFilter* filter, unsigned int options)
{
	return query->initSlicedFindPath(start_ref, end_ref, start_pos, end_pos, filter, options);
}

extern "C" dtStatus ref_query_update_sliced_find_path(dtNavMeshQuery* query, int max_iter, int* done_iters)
{
	return query->updateSlicedFindPath(max_iter, done_iters);
}

extern "C" dtStatus ref_query_finalize_sliced_find_path(dtNavMeshQuery* query,
														dtPolyRef* path, int* path_count, int max_path)
{
	return query->finalizeSlicedFindPath(path, path_count, max_path);
}

extern "C" dtStatus ref_query_finalize_sliced_find_path_partial(dtNavMeshQuery* query,
																const dtPolyRef* existing, int existing_size,
																dtPolyRef* path, int* path_count, int max_path)
{
	return query->finalizeSlicedFindPathPartial(existing, existing_size, path, path_count, max_path);
}

extern "C" dtStatus ref_query_find_polys_around_circle(const dtNavMeshQuery* query, dtPolyRef start_ref,
													   const float* center_pos, float radius,
													   const dtQueryFilter* filter,
													   dtPolyRef* result_ref, dtPolyRef* result_parent,
													   float* result_cost, int* result_count, int max_result)
{
	return query->findPolysAroundCircle(start_ref, center_pos, radius, filter,
										result_ref, result_parent, result_cost, result_count, max_result);
}

extern "C" dtStatus ref_query_find_polys_around_shape(const dtNavMeshQuery* query, dtPolyRef start_ref,
													  const float* verts, int nverts,
													  const dtQueryFilter* filter,
													  dtPolyRef* result_ref, dtPolyRef* result_parent,
													  float* result_cost, int* result_count, int max_result)
{
	return query->findPolysAroundShape(start_ref, verts, nverts, filter,
									   result_ref, result_parent, result_cost, result_count, max_result);
}

extern "C" dtStatus ref_query_get_path_from_dijkstra_search(const dtNavMeshQuery* query, dtPolyRef end_ref,
															dtPolyRef* path, int* path_count, int max_path)
{
	return query->getPathFromDijkstraSearch(end_ref, path, path_count, max_path);
}

// findNearestPoly(center, halfExtents, filter, nearestRef, nearestPt)
extern "C" dtStatus ref_query_find_nearest_poly(const dtNavMeshQuery* query,
												const float* center, const float* half_extents,
												const dtQueryFilter* filter,
												dtPolyRef* nearest_ref, float* nearest_pt)
{
	return query->findNearestPoly(center, half_extents, filter, nearest_ref, nearest_pt);
}

// findNearestPoly(center, halfExtents, filter, nearestRef, nearestPt, isOverPoly)
extern "C" dtStatus ref_query_find_nearest_poly_over(const dtNavMeshQuery* query,
													 const float* center, const float* half_extents,
													 const dtQueryFilter* filter,
													 dtPolyRef* nearest_ref, float* nearest_pt, bool* is_over_poly)
{
	return query->findNearestPoly(center, half_extents, filter, nearest_ref, nearest_pt, is_over_poly);
}

// queryPolygons(center, halfExtents, filter, polys, polyCount, maxPolys)
extern "C" dtStatus ref_query_query_polygons(const dtNavMeshQuery* query,
											const float* center, const float* half_extents,
											const dtQueryFilter* filter,
											dtPolyRef* polys, int* poly_count, int max_polys)
{
	return query->queryPolygons(center, half_extents, filter, polys, poly_count, max_polys);
}

// queryPolygons(center, halfExtents, filter, dtPolyQuery*): the callback gets
// every batch upstream passes to dtPolyQuery::process.
typedef void (*RefPolyQueryFn)(void* user, const dtMeshTile* tile, dtPoly** polys, dtPolyRef* refs, int count);

namespace
{

class RefPolyQuery : public dtPolyQuery
{
public:
	RefPolyQuery(RefPolyQueryFn process, void* user) : m_process(process), m_user(user) {}
	virtual ~RefPolyQuery() {}

	virtual void process(const dtMeshTile* tile, dtPoly** polys, dtPolyRef* refs, int count)
	{
		m_process(m_user, tile, polys, refs, count);
	}

private:
	RefPolyQueryFn m_process;
	void* m_user;
};

} // namespace

extern "C" dtStatus ref_query_query_polygons_cb(const dtNavMeshQuery* query,
												const float* center, const float* half_extents,
												const dtQueryFilter* filter,
												RefPolyQueryFn process, void* user)
{
	RefPolyQuery poly_query(process, user);
	return query->queryPolygons(center, half_extents, filter, &poly_query);
}

extern "C" dtStatus ref_query_find_local_neighbourhood(const dtNavMeshQuery* query, dtPolyRef start_ref,
													   const float* center_pos, float radius,
													   const dtQueryFilter* filter,
													   dtPolyRef* result_ref, dtPolyRef* result_parent,
													   int* result_count, int max_result)
{
	return query->findLocalNeighbourhood(start_ref, center_pos, radius, filter,
										 result_ref, result_parent, result_count, max_result);
}

extern "C" dtStatus ref_query_move_along_surface(const dtNavMeshQuery* query, dtPolyRef start_ref,
												 const float* start_pos, const float* end_pos,
												 const dtQueryFilter* filter,
												 float* result_pos, dtPolyRef* visited, int* visited_count,
												 int max_visited_size)
{
	return query->moveAlongSurface(start_ref, start_pos, end_pos, filter,
								   result_pos, visited, visited_count, max_visited_size);
}

// raycast(startRef, startPos, endPos, filter, t, hitNormal, path, pathCount, maxPath)
extern "C" dtStatus ref_query_raycast(const dtNavMeshQuery* query, dtPolyRef start_ref,
									  const float* start_pos, const float* end_pos,
									  const dtQueryFilter* filter,
									  float* t, float* hit_normal, dtPolyRef* path, int* path_count, int max_path)
{
	return query->raycast(start_ref, start_pos, end_pos, filter, t, hit_normal, path, path_count, max_path);
}

// raycast(startRef, startPos, endPos, filter, options, dtRaycastHit*, prevRef).
// The hit's fields come back through the out pointers; a null out pointer is
// skipped. `path` and `max_path` are the caller's buffer and its capacity.
extern "C" dtStatus ref_query_raycast_hit(const dtNavMeshQuery* query, dtPolyRef start_ref,
										  const float* start_pos, const float* end_pos,
										  const dtQueryFilter* filter, unsigned int options, dtPolyRef prev_ref,
										  float* out_t, float* out_hit_normal, int* out_hit_edge_index,
										  dtPolyRef* path, int* out_path_count, int max_path,
										  float* out_path_cost)
{
	dtRaycastHit hit;
	memset(&hit, 0, sizeof(hit));
	hit.path = path;
	hit.maxPath = max_path;

	const dtStatus status = query->raycast(start_ref, start_pos, end_pos, filter, options, &hit, prev_ref);

	if (out_t)
		*out_t = hit.t;
	if (out_hit_normal)
	{
		out_hit_normal[0] = hit.hitNormal[0];
		out_hit_normal[1] = hit.hitNormal[1];
		out_hit_normal[2] = hit.hitNormal[2];
	}
	if (out_hit_edge_index)
		*out_hit_edge_index = hit.hitEdgeIndex;
	if (out_path_count)
		*out_path_count = hit.pathCount;
	if (out_path_cost)
		*out_path_cost = hit.pathCost;
	return status;
}

extern "C" dtStatus ref_query_find_distance_to_wall(const dtNavMeshQuery* query, dtPolyRef start_ref,
													const float* center_pos, float max_radius,
													const dtQueryFilter* filter,
													float* hit_dist, float* hit_pos, float* hit_normal)
{
	return query->findDistanceToWall(start_ref, center_pos, max_radius, filter, hit_dist, hit_pos, hit_normal);
}

extern "C" dtStatus ref_query_get_poly_wall_segments(const dtNavMeshQuery* query, dtPolyRef ref,
													 const dtQueryFilter* filter,
													 float* segment_verts, dtPolyRef* segment_refs,
													 int* segment_count, int max_segments)
{
	return query->getPolyWallSegments(ref, filter, segment_verts, segment_refs, segment_count, max_segments);
}

extern "C" dtStatus ref_query_find_random_point(const dtNavMeshQuery* query, const dtQueryFilter* filter,
												float (*frand)(),
												dtPolyRef* random_ref, float* random_pt)
{
	return query->findRandomPoint(filter, frand, random_ref, random_pt);
}

extern "C" dtStatus ref_query_find_random_point_around_circle(const dtNavMeshQuery* query, dtPolyRef start_ref,
															  const float* center_pos, float max_radius,
															  const dtQueryFilter* filter, float (*frand)(),
															  dtPolyRef* random_ref, float* random_pt)
{
	return query->findRandomPointAroundCircle(start_ref, center_pos, max_radius, filter, frand,
											  random_ref, random_pt);
}

extern "C" dtStatus ref_query_closest_point_on_poly(const dtNavMeshQuery* query, dtPolyRef ref,
													const float* pos, float* closest, bool* pos_over_poly)
{
	return query->closestPointOnPoly(ref, pos, closest, pos_over_poly);
}

extern "C" dtStatus ref_query_closest_point_on_poly_boundary(const dtNavMeshQuery* query, dtPolyRef ref,
															 const float* pos, float* closest)
{
	return query->closestPointOnPolyBoundary(ref, pos, closest);
}

extern "C" dtStatus ref_query_get_poly_height(const dtNavMeshQuery* query, dtPolyRef ref,
											  const float* pos, float* height)
{
	return query->getPolyHeight(ref, pos, height);
}

extern "C" bool ref_query_is_valid_poly_ref(const dtNavMeshQuery* query, dtPolyRef ref,
											const dtQueryFilter* filter)
{
	return query->isValidPolyRef(ref, filter);
}

extern "C" bool ref_query_is_in_closed_list(const dtNavMeshQuery* query, dtPolyRef ref)
{
	return query->isInClosedList(ref);
}

extern "C" dtNodePool* ref_query_get_node_pool(const dtNavMeshQuery* query) { return query->getNodePool(); }

extern "C" const dtNavMesh* ref_query_get_attached_nav_mesh(const dtNavMeshQuery* query)
{
	return query->getAttachedNavMesh();
}

// ---------------------------------------------------------------------------
// DetourNode.h: dtNode (bit-fields), dtNodePool, dtNodeQueue
// ---------------------------------------------------------------------------

extern "C" void ref_node_fields(const dtNode* node, float* pos3, float* cost, float* total,
								unsigned int* pidx, unsigned int* state, unsigned int* flags, dtPolyRef* id)
{
	pos3[0] = node->pos[0];
	pos3[1] = node->pos[1];
	pos3[2] = node->pos[2];
	*cost = node->cost;
	*total = node->total;
	*pidx = node->pidx;
	*state = node->state;
	*flags = node->flags;
	*id = node->id;
}

extern "C" void ref_node_set(dtNode* node, const float* pos3, float cost, float total,
							 unsigned int pidx, unsigned int state, unsigned int flags, dtPolyRef id)
{
	node->pos[0] = pos3[0];
	node->pos[1] = pos3[1];
	node->pos[2] = pos3[2];
	node->cost = cost;
	node->total = total;
	node->pidx = pidx;
	node->state = state;
	node->flags = flags;
	node->id = id;
}

extern "C" unsigned int ref_node_get_pidx(const dtNode* node) { return node->pidx; }
extern "C" unsigned int ref_node_get_state(const dtNode* node) { return node->state; }
extern "C" unsigned int ref_node_get_flags(const dtNode* node) { return node->flags; }
extern "C" void ref_node_set_pidx(dtNode* node, unsigned int pidx) { node->pidx = pidx; }
extern "C" void ref_node_set_state(dtNode* node, unsigned int state) { node->state = state; }
extern "C" void ref_node_set_flags(dtNode* node, unsigned int flags) { node->flags = flags; }

extern "C" dtNodePool* ref_node_pool_create(int max_nodes, int hash_size)
{
	void* mem = dtAlloc(sizeof(dtNodePool), DT_ALLOC_PERM);
	if (!mem)
		return 0;
	return new (mem) dtNodePool(max_nodes, hash_size);
}

extern "C" void ref_node_pool_destroy(dtNodePool* pool)
{
	if (!pool)
		return;
	pool->~dtNodePool();
	dtFree(pool);
}

extern "C" void ref_node_pool_clear(dtNodePool* pool) { pool->clear(); }

extern "C" dtNode* ref_node_pool_get_node(dtNodePool* pool, dtPolyRef id, unsigned char state)
{
	return pool->getNode(id, state);
}

extern "C" dtNode* ref_node_pool_find_node(dtNodePool* pool, dtPolyRef id, unsigned char state)
{
	return pool->findNode(id, state);
}

extern "C" unsigned int ref_node_pool_find_nodes(dtNodePool* pool, dtPolyRef id, dtNode** nodes, int max_nodes)
{
	return pool->findNodes(id, nodes, max_nodes);
}

extern "C" unsigned int ref_node_pool_get_node_idx(const dtNodePool* pool, const dtNode* node)
{
	return pool->getNodeIdx(node);
}

extern "C" dtNode* ref_node_pool_get_node_at_idx(dtNodePool* pool, unsigned int idx)
{
	return pool->getNodeAtIdx(idx);
}

extern "C" int ref_node_pool_get_mem_used(const dtNodePool* pool) { return pool->getMemUsed(); }
extern "C" int ref_node_pool_get_max_nodes(const dtNodePool* pool) { return pool->getMaxNodes(); }
extern "C" int ref_node_pool_get_hash_size(const dtNodePool* pool) { return pool->getHashSize(); }
extern "C" dtNodeIndex ref_node_pool_get_first(const dtNodePool* pool, int bucket) { return pool->getFirst(bucket); }
extern "C" dtNodeIndex ref_node_pool_get_next(const dtNodePool* pool, int i) { return pool->getNext(i); }
extern "C" int ref_node_pool_get_node_count(const dtNodePool* pool) { return pool->getNodeCount(); }

extern "C" dtNodeQueue* ref_node_queue_create(int n)
{
	void* mem = dtAlloc(sizeof(dtNodeQueue), DT_ALLOC_PERM);
	if (!mem)
		return 0;
	return new (mem) dtNodeQueue(n);
}

extern "C" void ref_node_queue_destroy(dtNodeQueue* queue)
{
	if (!queue)
		return;
	queue->~dtNodeQueue();
	dtFree(queue);
}

extern "C" void ref_node_queue_clear(dtNodeQueue* queue) { queue->clear(); }
extern "C" dtNode* ref_node_queue_top(dtNodeQueue* queue) { return queue->top(); }
extern "C" dtNode* ref_node_queue_pop(dtNodeQueue* queue) { return queue->pop(); }
extern "C" void ref_node_queue_push(dtNodeQueue* queue, dtNode* node) { queue->push(node); }
extern "C" void ref_node_queue_modify(dtNodeQueue* queue, dtNode* node) { queue->modify(node); }
extern "C" bool ref_node_queue_empty(const dtNodeQueue* queue) { return queue->empty(); }
extern "C" int ref_node_queue_get_mem_used(const dtNodeQueue* queue) { return queue->getMemUsed(); }
extern "C" int ref_node_queue_get_capacity(const dtNodeQueue* queue) { return queue->getCapacity(); }
