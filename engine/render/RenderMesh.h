#pragma once

#include <vector>

namespace geom {
  struct GridPlane;
  struct MarkerCross;
}

namespace render {

class RenderMesh {
public:
  // 3 floats per vertex: x, y, z
  std::vector<float>    positions;
  std::vector<unsigned> indices;

  // Upload from CPU geom types
  void uploadGrid(const geom::GridPlane& grid);
  void uploadMarker(const geom::MarkerCross& marker);

  void clear();
};

} // namespace render
