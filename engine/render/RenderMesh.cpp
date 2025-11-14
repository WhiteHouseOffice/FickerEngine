#include "render/RenderMesh.h"

#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"

namespace render {

void RenderMesh::uploadGrid(const geom::GridPlane& grid) {
  positions = grid.positions;
  indices   = grid.indices;
}

void RenderMesh::uploadMarker(const geom::MarkerCross& marker) {
  positions = marker.positions;
  indices   = marker.indices;
}

void RenderMesh::clear() {
  positions.clear();
  indices.clear();
}

} // namespace render
