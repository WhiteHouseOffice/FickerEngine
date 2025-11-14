#include "render/RenderMesh.h"
#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"

using namespace render;

void RenderMesh::clear() {
  positions.clear();
  indices.clear();
}

void RenderMesh::uploadGrid(const GridPlane& grid) {
  positions = grid.positions;
  indices   = grid.indices;
}

void RenderMesh::uploadMarker(const MarkerCross& marker) {
  positions = marker.positions;
  indices   = marker.indices;
}
