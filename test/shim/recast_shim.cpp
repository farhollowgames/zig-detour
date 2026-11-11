// Test-only input generation with upstream Recast (recastnavigation v1.6.0).
//
// Runs Recast's standard pipeline on a triangle soup and hands back flat
// arrays that Detour's tile builder takes as input, so the Zig port of Detour
// can be tested on realistic meshes. The Zig declarations live in
// test/reference_recast.zig; keep the two in step.
//
// The steps follow the RecastDemo samples closely:
//   - ref_recast_build_tile: Sample_TileMesh::buildTileMesh, or
//     Sample_SoloMesh::build when tile_size is 0, up to (not including)
//     dtCreateNavMeshData
//   - ref_recast_build_layers: Sample_TempObstacles::rasterizeTileLayers, up to
//     (not including) dtBuildTileCacheLayer
// The filters of the samples (low hanging obstacles, ledge spans, low height
// spans) are all on. Logging and timers are off.
//
// Differences from the samples:
//   - the samples pick the triangles that overlap the tile through a chunked
//     copy of the mesh, and rasterize them chunk by chunk. Here every triangle
//     whose xz-box touches the tile's box (border included) is taken, in input
//     order. The rasterizer skips the others anyway, so only the order of
//     rasterization differs, which can change which of two close spans gets
//     the area of a merge. The result is a valid Recast output either way.
//   - the samples' convex volumes and off-mesh connections do not exist here.
//
// Units: detail_sample_dist is a multiple of cell_size and
// detail_sample_max_error a multiple of cell_height, exactly like the sample's
// sliders (a detail_sample_dist below 0.9 turns detail sampling off). The
// other lengths are in world units; region sizes are in cells per side.

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "DetourNavMesh.h"
#include "Recast.h"

struct RefRecastConfig
{
	float cell_size, cell_height, agent_height, agent_radius, agent_max_climb, agent_max_slope;
	float edge_max_len, edge_max_error, detail_sample_dist, detail_sample_max_error;
	int region_min_size, region_merge_size, verts_per_poly;
	int tile_size; // cells; 0 = one tile for the whole mesh
	int partition; // 0 watershed, 1 monotone, 2 layers
};

// Everything dtNavMeshCreateParams needs from Recast. The arrays belong to
// `owner` and live until ref_recast_free_tile.
struct RefRecastTile
{
	const unsigned short* verts;
	int vert_count;
	const unsigned short* polys;
	const unsigned short* poly_flags;
	const unsigned char* poly_areas;
	int poly_count;
	int nvp;
	const unsigned int* detail_meshes;
	const float* detail_verts;
	int detail_vert_count;
	const unsigned char* detail_tris;
	int detail_tri_count;
	float bmin[3];
	float bmax[3];
	float walkable_height, walkable_radius, walkable_climb, cs, ch;
	void* owner;
};

// A heightfield layer, with the fields of dtTileCacheLayerHeader. The arrays
// belong to the owner given back by ref_recast_build_layers.
struct RefRecastLayer
{
	int tile_x, tile_y, layer;
	float bmin[3];
	float bmax[3];
	unsigned short hmin, hmax;
	unsigned char width, height, minx, maxx, miny, maxy;
	const unsigned char* heights;
	const unsigned char* areas;
	const unsigned char* cons;
};

namespace
{

struct TileOwner
{
	rcPolyMesh* pmesh;
	rcPolyMeshDetail* dmesh;
};

// Frees every intermediate a build allocates, however the build ends.
struct Scratch
{
	rcHeightfield* solid;
	rcCompactHeightfield* chf;
	rcContourSet* cset;
	rcPolyMesh* pmesh;
	rcPolyMeshDetail* dmesh;
	int* tri_buf;
	unsigned char* area_buf;

	Scratch() : solid(0), chf(0), cset(0), pmesh(0), dmesh(0), tri_buf(0), area_buf(0) {}

	~Scratch()
	{
		rcFreeHeightField(solid);
		rcFreeCompactHeightfield(chf);
		rcFreeContourSet(cset);
		rcFreePolyMesh(pmesh);
		rcFreePolyMeshDetail(dmesh);
		free(tri_buf);
		free(area_buf);
	}

private:
	Scratch(const Scratch&);
	Scratch& operator=(const Scratch&);
};

// The sample's rcConfig for the whole mesh (not yet moved to a tile).
void make_config(const RefRecastConfig& c, const float* mesh_bmin, const float* mesh_bmax, rcConfig& out)
{
	memset(&out, 0, sizeof(out));
	out.cs = c.cell_size;
	out.ch = c.cell_height;
	out.walkableSlopeAngle = c.agent_max_slope;
	out.walkableHeight = static_cast<int>(ceilf(c.agent_height / out.ch));
	out.walkableClimb = static_cast<int>(floorf(c.agent_max_climb / out.ch));
	out.walkableRadius = static_cast<int>(ceilf(c.agent_radius / out.cs));
	out.maxEdgeLen = static_cast<int>(c.edge_max_len / c.cell_size);
	out.maxSimplificationError = c.edge_max_error;
	out.minRegionArea = static_cast<int>(rcSqr(c.region_min_size)); // area = size * size
	out.mergeRegionArea = static_cast<int>(rcSqr(c.region_merge_size)); // area = size * size
	out.maxVertsPerPoly = c.verts_per_poly;
	out.detailSampleDist = c.detail_sample_dist < 0.9f ? 0 : c.cell_size * c.detail_sample_dist;
	out.detailSampleMaxError = c.cell_height * c.detail_sample_max_error;
	out.tileSize = c.tile_size;
	rcVcopy(out.bmin, mesh_bmin);
	rcVcopy(out.bmax, mesh_bmax);

	if (c.tile_size > 0)
	{
		out.borderSize = out.walkableRadius + 3; // reserve enough padding
		out.width = c.tile_size + out.borderSize * 2;
		out.height = c.tile_size + out.borderSize * 2;
	}
	else
	{
		rcCalcGridSize(out.bmin, out.bmax, out.cs, &out.width, &out.height);
	}
}

// Moves the config to one tile and grows its box by the border, as the samples do.
void make_tile_config(const RefRecastConfig& c, const rcConfig& base, int tile_x, int tile_y, rcConfig& out)
{
	out = base;
	if (c.tile_size <= 0)
		return;

	const float tcs = static_cast<float>(c.tile_size) * base.cs;
	out.bmin[0] = base.bmin[0] + static_cast<float>(tile_x) * tcs;
	out.bmin[1] = base.bmin[1];
	out.bmin[2] = base.bmin[2] + static_cast<float>(tile_y) * tcs;
	out.bmax[0] = base.bmin[0] + static_cast<float>(tile_x + 1) * tcs;
	out.bmax[1] = base.bmax[1];
	out.bmax[2] = base.bmin[2] + static_cast<float>(tile_y + 1) * tcs;
	out.bmin[0] -= static_cast<float>(out.borderSize) * out.cs;
	out.bmin[2] -= static_cast<float>(out.borderSize) * out.cs;
	out.bmax[0] += static_cast<float>(out.borderSize) * out.cs;
	out.bmax[2] += static_cast<float>(out.borderSize) * out.cs;
}

// Rasterizes the triangles that touch the tile, filters the spans, compacts
// the heightfield and erodes it by the agent radius. The shared first half of
// both pipelines. False when nothing is left or a step fails.
bool build_compact(rcContext& ctx, const float* verts, int nverts, const int* tris, int ntris,
				   const rcConfig& tcfg, Scratch& s)
{
	s.solid = rcAllocHeightfield();
	if (!s.solid)
		return false;
	if (!rcCreateHeightfield(&ctx, *s.solid, tcfg.width, tcfg.height, tcfg.bmin, tcfg.bmax, tcfg.cs, tcfg.ch))
		return false;

	s.tri_buf = static_cast<int*>(malloc(sizeof(int) * 3 * static_cast<size_t>(ntris)));
	s.area_buf = static_cast<unsigned char*>(malloc(static_cast<size_t>(ntris)));
	if (!s.tri_buf || !s.area_buf)
		return false;

	int n = 0;
	for (int i = 0; i < ntris; ++i)
	{
		const int* t = &tris[i * 3];
		float minx = verts[t[0] * 3 + 0];
		float maxx = minx;
		float minz = verts[t[0] * 3 + 2];
		float maxz = minz;
		for (int k = 1; k < 3; ++k)
		{
			const float x = verts[t[k] * 3 + 0];
			const float z = verts[t[k] * 3 + 2];
			if (x < minx)
				minx = x;
			if (x > maxx)
				maxx = x;
			if (z < minz)
				minz = z;
			if (z > maxz)
				maxz = z;
		}
		if (minx > tcfg.bmax[0] || maxx < tcfg.bmin[0] || minz > tcfg.bmax[2] || maxz < tcfg.bmin[2])
			continue;

		s.tri_buf[n * 3 + 0] = t[0];
		s.tri_buf[n * 3 + 1] = t[1];
		s.tri_buf[n * 3 + 2] = t[2];
		++n;
	}
	if (n == 0)
		return false;

	memset(s.area_buf, 0, static_cast<size_t>(n));
	rcMarkWalkableTriangles(&ctx, tcfg.walkableSlopeAngle, verts, nverts, s.tri_buf, n, s.area_buf);
	if (!rcRasterizeTriangles(&ctx, verts, nverts, s.tri_buf, s.area_buf, n, *s.solid, tcfg.walkableClimb))
		return false;

	// Remove overhangs caused by the conservative rasterization, and spans
	// where the agent cannot stand.
	rcFilterLowHangingWalkableObstacles(&ctx, tcfg.walkableClimb, *s.solid);
	rcFilterLedgeSpans(&ctx, tcfg.walkableHeight, tcfg.walkableClimb, *s.solid);
	rcFilterWalkableLowHeightSpans(&ctx, tcfg.walkableHeight, *s.solid);

	s.chf = rcAllocCompactHeightfield();
	if (!s.chf)
		return false;
	if (!rcBuildCompactHeightfield(&ctx, tcfg.walkableHeight, tcfg.walkableClimb, *s.solid, *s.chf))
		return false;
	if (!rcErodeWalkableArea(&ctx, tcfg.walkableRadius, *s.chf))
		return false;
	return true;
}

bool check_common(const float* verts, int nverts, const int* tris, int ntris, const RefRecastConfig* cfg,
				  const float* mesh_bmin, const float* mesh_bmax)
{
	if (!verts || !tris || !cfg || !mesh_bmin || !mesh_bmax)
		return false;
	if (nverts <= 0 || ntris <= 0)
		return false;
	if (!(cfg->cell_size > 0.0f) || !(cfg->cell_height > 0.0f))
		return false;
	if (cfg->tile_size < 0)
		return false;
	return true;
}

} // namespace

extern "C" size_t ref_recast_sizeof_config() { return sizeof(RefRecastConfig); }
extern "C" size_t ref_recast_sizeof_tile() { return sizeof(RefRecastTile); }
extern "C" size_t ref_recast_sizeof_layer() { return sizeof(RefRecastLayer); }

// Builds the polygon mesh and detail mesh of one tile. With tile_size 0 the
// whole mesh is one tile and tile_x / tile_y are ignored. Returns false for an
// empty tile, a Recast failure, or verts_per_poly above DT_VERTS_PER_POLYGON
// (Detour cannot take such a mesh); *out is zeroed then.
//
// Polygon areas stay as Recast made them (RC_WALKABLE_AREA for walkable
// ground). poly_flags is 1 where the area is RC_WALKABLE_AREA and 0 elsewhere.
extern "C" bool ref_recast_build_tile(const float* verts, int nverts, const int* tris, int ntris,
									  const RefRecastConfig* cfg, const float* mesh_bmin, const float* mesh_bmax,
									  int tile_x, int tile_y, RefRecastTile* out)
{
	if (!out)
		return false;
	memset(out, 0, sizeof(*out));

	if (!check_common(verts, nverts, tris, ntris, cfg, mesh_bmin, mesh_bmax))
		return false;
	if (cfg->verts_per_poly < 3 || cfg->verts_per_poly > DT_VERTS_PER_POLYGON)
		return false;
	if (cfg->partition < 0 || cfg->partition > 2)
		return false;

	rcContext ctx(false);

	rcConfig base;
	make_config(*cfg, mesh_bmin, mesh_bmax, base);
	rcConfig tcfg;
	make_tile_config(*cfg, base, tile_x, tile_y, tcfg);

	Scratch s;
	if (!build_compact(ctx, verts, nverts, tris, ntris, tcfg, s))
		return false;

	if (cfg->partition == 0)
	{
		// Watershed: distance field, then regions.
		if (!rcBuildDistanceField(&ctx, *s.chf))
			return false;
		if (!rcBuildRegions(&ctx, *s.chf, tcfg.borderSize, tcfg.minRegionArea, tcfg.mergeRegionArea))
			return false;
	}
	else if (cfg->partition == 1)
	{
		if (!rcBuildRegionsMonotone(&ctx, *s.chf, tcfg.borderSize, tcfg.minRegionArea, tcfg.mergeRegionArea))
			return false;
	}
	else
	{
		if (!rcBuildLayerRegions(&ctx, *s.chf, tcfg.borderSize, tcfg.minRegionArea))
			return false;
	}

	s.cset = rcAllocContourSet();
	if (!s.cset)
		return false;
	if (!rcBuildContours(&ctx, *s.chf, tcfg.maxSimplificationError, tcfg.maxEdgeLen, *s.cset))
		return false;
	if (s.cset->nconts == 0)
		return false;

	s.pmesh = rcAllocPolyMesh();
	if (!s.pmesh)
		return false;
	if (!rcBuildPolyMesh(&ctx, *s.cset, tcfg.maxVertsPerPoly, *s.pmesh))
		return false;

	s.dmesh = rcAllocPolyMeshDetail();
	if (!s.dmesh)
		return false;
	if (!rcBuildPolyMeshDetail(&ctx, *s.pmesh, *s.chf, tcfg.detailSampleDist, tcfg.detailSampleMaxError, *s.dmesh))
		return false;

	// The vertex indices are unsigned shorts and cannot point past 0xffff.
	if (s.pmesh->nverts >= 0xffff)
		return false;
	if (s.pmesh->npolys <= 0)
		return false;

	for (int i = 0; i < s.pmesh->npolys; ++i)
	{
		if (s.pmesh->areas[i] == RC_WALKABLE_AREA)
			s.pmesh->flags[i] = 1;
	}

	TileOwner* owner = static_cast<TileOwner*>(malloc(sizeof(TileOwner)));
	if (!owner)
		return false;
	owner->pmesh = s.pmesh;
	owner->dmesh = s.dmesh;

	out->verts = s.pmesh->verts;
	out->vert_count = s.pmesh->nverts;
	out->polys = s.pmesh->polys;
	out->poly_flags = s.pmesh->flags;
	out->poly_areas = s.pmesh->areas;
	out->poly_count = s.pmesh->npolys;
	out->nvp = s.pmesh->nvp;
	out->detail_meshes = s.dmesh->meshes;
	out->detail_verts = s.dmesh->verts;
	out->detail_vert_count = s.dmesh->nverts;
	out->detail_tris = s.dmesh->tris;
	out->detail_tri_count = s.dmesh->ntris;
	rcVcopy(out->bmin, s.pmesh->bmin);
	rcVcopy(out->bmax, s.pmesh->bmax);
	out->walkable_height = cfg->agent_height;
	out->walkable_radius = cfg->agent_radius;
	out->walkable_climb = cfg->agent_max_climb;
	out->cs = tcfg.cs;
	out->ch = tcfg.ch;
	out->owner = owner;

	// The tile owns the two meshes now; the scratch must not free them.
	s.pmesh = 0;
	s.dmesh = 0;
	return true;
}

extern "C" void ref_recast_free_tile(RefRecastTile* tile)
{
	if (!tile)
		return;
	TileOwner* owner = static_cast<TileOwner*>(tile->owner);
	if (owner)
	{
		rcFreePolyMesh(owner->pmesh);
		rcFreePolyMeshDetail(owner->dmesh);
		free(owner);
	}
	memset(tile, 0, sizeof(*tile));
}

// Builds the heightfield layers of one tile for the tile cache. cfg->tile_size
// must be in 1..255. Fills the first `count` entries of out and returns count, at most
// max_layers; 0 for an empty tile, a Recast failure or bad arguments. The
// arrays the layers point to belong to *owner, which ref_recast_free_layers
// releases; *owner is null when 0 is returned.
extern "C" int ref_recast_build_layers(const float* verts, int nverts, const int* tris, int ntris,
									   const RefRecastConfig* cfg, const float* mesh_bmin, const float* mesh_bmax,
									   int tile_x, int tile_y, RefRecastLayer* out, int max_layers, void** owner)
{
	if (owner)
		*owner = 0;
	if (!out || !owner || max_layers <= 0)
		return 0;

	if (!check_common(verts, nverts, tris, ntris, cfg, mesh_bmin, mesh_bmax))
		return 0;
	if (cfg->tile_size < 1 || cfg->tile_size > 255)
		return 0;

	rcContext ctx(false);

	rcConfig base;
	make_config(*cfg, mesh_bmin, mesh_bmax, base);
	rcConfig tcfg;
	make_tile_config(*cfg, base, tile_x, tile_y, tcfg);

	Scratch s;
	if (!build_compact(ctx, verts, nverts, tris, ntris, tcfg, s))
		return 0;

	rcHeightfieldLayerSet* lset = rcAllocHeightfieldLayerSet();
	if (!lset)
		return 0;
	if (!rcBuildHeightfieldLayers(&ctx, *s.chf, tcfg.borderSize, tcfg.walkableHeight, *lset))
	{
		rcFreeHeightfieldLayerSet(lset);
		return 0;
	}

	const int count = rcMin(lset->nlayers, max_layers);
	if (count <= 0)
	{
		rcFreeHeightfieldLayerSet(lset);
		return 0;
	}

	for (int i = 0; i < count; ++i)
	{
		const rcHeightfieldLayer* layer = &lset->layers[i];
		RefRecastLayer* o = &out[i];

		o->tile_x = tile_x;
		o->tile_y = tile_y;
		o->layer = i;
		rcVcopy(o->bmin, layer->bmin);
		rcVcopy(o->bmax, layer->bmax);
		o->hmin = static_cast<unsigned short>(layer->hmin);
		o->hmax = static_cast<unsigned short>(layer->hmax);
		o->width = static_cast<unsigned char>(layer->width);
		o->height = static_cast<unsigned char>(layer->height);
		o->minx = static_cast<unsigned char>(layer->minx);
		o->maxx = static_cast<unsigned char>(layer->maxx);
		o->miny = static_cast<unsigned char>(layer->miny);
		o->maxy = static_cast<unsigned char>(layer->maxy);
		o->heights = layer->heights;
		o->areas = layer->areas;
		o->cons = layer->cons;
	}

	*owner = lset;
	return count;
}

extern "C" void ref_recast_free_layers(void* owner)
{
	rcFreeHeightfieldLayerSet(static_cast<rcHeightfieldLayerSet*>(owner));
}
