#include "render/RenderMesh.h"

namespace render {

void RenderMesh::uploadGrid(const GridPlane& plane) {
  mesh.positions = plane.positions;
  mesh.indices   = plane.indices;
}

void RenderMesh::uploadMarker(const MarkerCross& marker) {
  mesh.positions = marker.positions;
  mesh.indices   = marker.indices;
}

void RenderMesh::release() {
  mesh.positions.clear();
  mesh.indices.clear();
}

void RenderMesh::bind() const {
  // Stub – no real GPU binding yet.
}

} // namespace render
