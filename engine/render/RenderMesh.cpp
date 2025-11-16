#include "render/RenderMesh.h"

#include <cstdio>

#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"

using std::uint32_t;

void RenderMesh::uploadGrid(const geom::GridPlane& grid) {
  // Assume GridPlane exposes positions/indices as CPU vectors.
  // We copy them into this CPU-side mesh container.
  positions = grid.positions; // std::vector<Vec3>
  indices.clear();
  indices.reserve(grid.indices.size());
  for (auto idx : grid.indices) {
    indices.push_back(static_cast<uint32_t>(idx));
  }
}

void RenderMesh::uploadMarker(const geom::MarkerCross& marker) {
  positions = marker.positions;
  indices.clear();
  indices.reserve(marker.indices.size());
  for (auto idx : marker.indices) {
    indices.push_back(static_cast<uint32_t>(idx));
  }
}

void RenderMesh::debugPrint(const char* label) const {
  const std::size_t v = positions.size();
  const std::size_t i = indices.size();

  std::printf("[RenderMesh] %s: %zu verts, %zu indices\n",
              label ? label : "(unnamed)", v, i);
}
