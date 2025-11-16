#include "render/RenderMesh.h"

// Pull in the *definitions* of the geometry structs.
#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"

namespace render {

void RenderMesh::clear() {
  positions.clear();
  indices.clear();
}

void RenderMesh::uploadGrid(const GridPlane& grid) {
  // Assuming GridPlane exposes:
  //   std::vector<float> positions;
  //   std::vector<unsigned int> indices;
  positions = grid.positions;
  indices   = grid.indices;
}

void RenderMesh::uploadMarker(const MarkerCross& marker) {
  // Assuming MarkerCross exposes:
  //   std::vector<float> positions;
  //   std::vector<unsigned int> indices;
  positions = marker.positions;
  indices   = marker.indices;
}

} // namespace render
