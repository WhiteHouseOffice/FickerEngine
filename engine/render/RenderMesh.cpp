#include "render/RenderMesh.h"

namespace render {

void RenderMesh::uploadCPU(const MeshData& src) {
  mesh = src;
}

MeshData MakeMesh(const geom::GridData& grid) {
  MeshData out;
  out.positions = grid.positions;
  out.indices   = grid.indices;
  return out;
}

MeshData MakeMesh(const geom::LinesData& lines) {
  MeshData out;
  out.positions = lines.positions;
  out.indices   = lines.indices;
  return out;
}

} // namespace render
