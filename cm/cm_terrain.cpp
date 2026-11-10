#include "cm_terrain.hpp"
#include "cm_typedefs.hpp"

#include "cg/cg_local.hpp"
#include "cg/cg_offsets.hpp"
#include <unordered_map>
#include <ranges>
#include <cmath>
#include <cstdint>
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

static bool CM_TerrainTriangleHasWalkableEdge(const int triIndex)
{
	if (!cm->triEdgeIsWalkable)
		return false;

	for (int edge = 0; edge < 3; ++edge) {
		const int bitIndex = 3 * triIndex + edge;
		if ((cm->triEdgeIsWalkable[bitIndex >> 3] & (1 << (bitIndex & 7))) != 0)
			return true;
	}

	return false;
}

struct TerrainEdgeInfo
{
	int triangleCount = 0;
	fvec3 firstNormal = {};
	bool crease = false;
};

static std::unordered_map<std::uint32_t, TerrainEdgeInfo> terrainEdges;

static std::uint32_t CM_TerrainEdgeKey(const unsigned short a, const unsigned short b)
{
	const auto lo = std::min(a, b);
	const auto hi = std::max(a, b);
	return (static_cast<std::uint32_t>(lo) << 16) | hi;
}

static fvec3 CM_TerrainTriangleNormal(const int triIndex)
{
	const int index = 3 * triIndex;
	vec4_t plane = {};
	PlaneFromPointsASM(
		plane,
		cm->verts[cm->triIndices[index]],
		cm->verts[cm->triIndices[index + 1]],
		cm->verts[cm->triIndices[index + 2]]);

	return { plane[0], plane[1], plane[2] };
}

static void CM_BuildTerrainEdgeInfo()
{
	terrainEdges.clear();

	for (int triIndex = 0; triIndex < cm->triCount; ++triIndex) {
		const int index = 3 * triIndex;
		const unsigned short indices[3] = {
			cm->triIndices[index],
			cm->triIndices[index + 1],
			cm->triIndices[index + 2]
		};
		const auto normal = CM_TerrainTriangleNormal(triIndex);

		for (int edge = 0; edge < 3; ++edge) {
			const auto key = CM_TerrainEdgeKey(indices[edge], indices[(edge + 1) % 3]);
			auto& info = terrainEdges[key];

			if (info.triangleCount == 0) {
				info.firstNormal = normal;
			}
			else {
				const float dot = info.firstNormal.x * normal.x
					+ info.firstNormal.y * normal.y
					+ info.firstNormal.z * normal.z;

				if (dot < 0.999f)
					info.crease = true;
			}

			++info.triangleCount;
		}
	}
}

static bool CM_TerrainEdgeIsPhysical(const unsigned short a, const unsigned short b)
{
	const auto it = terrainEdges.find(CM_TerrainEdgeKey(a, b));
	if (it == terrainEdges.end())
		return false;

	return it->second.triangleCount == 1 || it->second.crease;
}

static bool CM_TerrainEdgeCanProduceBounceNormal(const fvec3& a, const fvec3& b)
{
	const float edgeDx = b.x - a.x;
	const float edgeDy = b.y - a.y;
	const float edgeDz = b.z - a.z;
	const float lengthSq = edgeDx * edgeDx + edgeDy * edgeDy + edgeDz * edgeDz;

	if (lengthSq <= 1e-6f)
		return false;

	const float edgeZ = edgeDz / std::sqrt(lengthSq);
	const float maxContactNormalZ = std::sqrt(std::max(0.0f, 1.0f - edgeZ * edgeZ));

	return maxContactNormalZ >= 0.3f;
}

static bool CM_TerrainTriangleHasBounceEdgeContact(const cm_triangle& tri, const int triIndex)
{
	const int index = 3 * triIndex;
	const unsigned short i0 = cm->triIndices[index];
	const unsigned short i1 = cm->triIndices[index + 1];
	const unsigned short i2 = cm->triIndices[index + 2];

	return (CM_TerrainEdgeIsPhysical(i0, i1) && CM_TerrainEdgeCanProduceBounceNormal(tri.a, tri.b))
		|| (CM_TerrainEdgeIsPhysical(i1, i2) && CM_TerrainEdgeCanProduceBounceNormal(tri.b, tri.c))
		|| (CM_TerrainEdgeIsPhysical(i2, i0) && CM_TerrainEdgeCanProduceBounceNormal(tri.c, tri.a));
}

std::unordered_map<CollisionPartition*, CollisionPartition*> discovered_partitions;


bool CM_DiscoverTerrain([[maybe_unused]]const std::unordered_set<std::string>& filters)
{

	try {
		discovered_partitions.clear();
		CM_BuildTerrainEdgeInfo();
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

			// PM_StepSlideMove tests the runtime trace normal, not necessarily
			// the triangle plane. Capsule-vs-edge contacts produce a normal
			// perpendicular to the edge, so include triangles whose edges can
			// generate a contact normal with z >= 0.3.
			const bool faceBounceCandidate = tri.plane[2] >= 0.3f && tri.plane[2] < 0.7f;
			tri.is_bounce = faceBounceCandidate
				|| CM_TerrainTriangleHasBounceEdgeContact(tri, firstTri);

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