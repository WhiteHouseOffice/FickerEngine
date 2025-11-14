#include "render/RenderMesh.h"

using namespace render;

void RenderMesh::clear() {
  positions.clear();
  indices.clear();
}

void RenderMesh::uploadGrid(const geom::GridPlane& grid) {
  // GridPlane is pure CPU geom; just mirror its buffers
  positions = grid.positions;
  indices   = grid.indices;
}

void RenderMesh::uploadMarker(const geom::MarkerCross& marker) {
  positions = marker.positions;
  indices   = marker.indices;
}
