#include "cm_terrain.hpp"
#include "cm_typedefs.hpp"

#include "cg/cg_local.hpp"
#include "cg/cg_offsets.hpp"
#include <unordered_map>
#include <cstdint>
#include <algorithm>
#include <ranges>
#include <com/com_vector.hpp>

void CM_LoadAllTerrainToClipMapWithFilter(const std::string& filter)
{
	std::unique_lock<std::mutex> lock(CClipMap::GetLock());

	CClipMap::ClearAllOfType(cm_geomtype::terrain);

	const auto filters = CM_TokenizeFilters(filter);

	if (filters.empty())
		return;

	CM_DiscoverTerrain(filters);
}

bool CM_AabbTreeHasCollision(const CollisionAabbTree* tree)
{
	const dmaterial_t* materialInfo = &cm->materials[tree->materialIndex];
	return (materialInfo->contentFlags & MASK_PLAYERSOLID) != 0;
}

std::unordered_map<CollisionPartition*, CollisionPartition*> discovered_partitions;
static std::unordered_map<std::uint32_t, unsigned int> terrain_edge_counts;

static std::uint32_t CM_TerrainEdgeKey(const unsigned short a, const unsigned short b)
{
	const auto lo = std::min(a, b);
	const auto hi = std::max(a, b);
	return (static_cast<std::uint32_t>(lo) << 16) | hi;
}

static void CM_BuildTerrainEdgeCounts()
{
	terrain_edge_counts.clear();

	for (int triIndex = 0; triIndex < cm->triCount; ++triIndex) {
		const int index = 3 * triIndex;
		const unsigned short indices[3] = {
			cm->triIndices[index],
			cm->triIndices[index + 1],
			cm->triIndices[index + 2]
		};

		for (int edge = 0; edge < 3; ++edge)
			++terrain_edge_counts[CM_TerrainEdgeKey(indices[edge], indices[(edge + 1) % 3])];
	}
}

static bool CM_IsLowerBoundaryBounceEdge(
	const cm_triangle& tri,
	const unsigned short aIndex,
	const unsigned short bIndex,
	const fvec3& a,
	const fvec3& b)
{
	if (tri.plane[2] >= 0.3f)
		return false;

	const auto it = terrain_edge_counts.find(CM_TerrainEdgeKey(aIndex, bIndex));
	if (it == terrain_edge_counts.end() || it->second != 1)
		return false;

	const float minZ = std::min({ tri.a.z, tri.b.z, tri.c.z });
	constexpr float zEpsilon = 0.01f;

	if (std::fabs(a.z - minZ) > zEpsilon || std::fabs(b.z - minZ) > zEpsilon)
		return false;

	// Lower rims are expected to be approximately horizontal. This excludes
	// vertical borders and the internal diagonal of a triangulated quad.
	return std::fabs(a.z - b.z) <= zEpsilon;
}

static unsigned char CM_GetTerrainBounceEdgeMask(
	const cm_triangle& tri,
	const unsigned short i0,
	const unsigned short i1,
	const unsigned short i2)
{
	unsigned char mask = 0;

	if (CM_IsLowerBoundaryBounceEdge(tri, i0, i1, tri.a, tri.b))
		mask |= 1 << 0;
	if (CM_IsLowerBoundaryBounceEdge(tri, i1, i2, tri.b, tri.c))
		mask |= 1 << 1;
	if (CM_IsLowerBoundaryBounceEdge(tri, i2, i0, tri.c, tri.a))
		mask |= 1 << 2;

	return mask;
}


bool CM_DiscoverTerrain([[maybe_unused]]const std::unordered_set<std::string>& filters)
{

	try {
		discovered_partitions.clear();
		CM_BuildTerrainEdgeCounts();
		for (const auto i : std::views::iota(0u, cm->numLeafs)) {
			cm_terrain terrain;

			if(terrain.CM_LeafToGeometry(&cm->leafs[i], filters))
				CClipMap::Insert(std::make_unique<cm_terrain>(terrain));

		}
		return true;
	}
	catch (...) {
		return false;
	}
}

void cm_terrain::CM_AdvanceAabbTree(const CollisionAabbTree* aabbTree, const std::unordered_set<std::string>& filters, const float* color)
{
	if (aabbTree->childCount) {
		auto child = &cm->aabbTrees[aabbTree->u.firstChildIndex];
		for ([[maybe_unused]]const auto i : std::views::iota(0u, aabbTree->childCount)) {
			CM_AdvanceAabbTree(child, filters, color);
			++child;
		}
		return;
	}

	const auto mat = cm->materials[aabbTree->materialIndex].material;
	if (CM_IsMatchingFilter(filters, mat) == false) {
		return;
	}

	material = mat;
	originalContents = leaf->terrainContents;
	num_verts = 3;

	CollisionAabbTreeIndex fChild = aabbTree->u;
	CollisionPartition* partition = &cm->partitions[fChild.partitionIndex];

	if (discovered_partitions.contains(partition))
		return;

	discovered_partitions[partition] = partition;

	auto firstTri = partition->firstTri;
	if (firstTri < firstTri + partition->triCount)
	{

		auto triIndice = 3 * firstTri;

		do {
			cm_triangle tri;
			tri.has_collision = CM_AabbTreeHasCollision(aabbTree);
			tri.a = cm->verts[cm->triIndices[triIndice]];
			tri.b = cm->verts[cm->triIndices[triIndice + 1]];
			tri.c = cm->verts[cm->triIndices[triIndice + 2]];

			PlaneFromPointsASM(tri.plane, tri.a, tri.b, tri.c);

			const unsigned short i0 = cm->triIndices[triIndice];
			const unsigned short i1 = cm->triIndices[triIndice + 1];
			const unsigned short i2 = cm->triIndices[triIndice + 2];
			tri.bounce_edge_mask = CM_GetTerrainBounceEdgeMask(tri, i0, i1, i2);

			tri.color[0] = color[0];
			tri.color[1] = color[1];
			tri.color[2] = color[2];
			tri.color[3] = 0.3f;

			tris.emplace_back(tri);

			++firstTri;
			triIndice += 3;

		} while (firstTri < partition->firstTri + partition->triCount);

	}
}

bool cm_terrain::CM_LeafToGeometry(const cLeaf_t* aleaf, const std::unordered_set<std::string>& filters)
{
	if (!aleaf || !aleaf->collAabbCount)
		return 0;

	std::int32_t aabbIdx = 0;
	
	leaf = aleaf;

	do {
		const CollisionAabbTree* aabb = &cm->aabbTrees[aabbIdx + leaf->firstCollAabbIndex];
		CM_AdvanceAabbTree(aabb, filters, vec4_t{ 0,0.1f,1.f, 0.8f });
		++aabbIdx;
	} while (aabbIdx < leaf->collAabbCount);


	return true;
}

bool CM_TriangleInView(const cm_triangle* tris, struct cplane_s* frustumPlanes, int numPlanes)
{
	if (numPlanes <= 0)
		return 1;
	cplane_s* plane = frustumPlanes;
	int idx = 0;
	while ((BoxOnPlaneSide(tris->get_mins(), tris->get_maxs(), plane) & 1) != 0) {
		++plane;
		++idx;

		if (idx >= numPlanes)
			return 1;
	}

	return 0;
}