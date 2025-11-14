#pragma once
#include <vector>
#include <cstdint>

namespace geom {
  struct GridPlane;
  struct MarkerCross;
}

namespace render {

class RenderMesh {
public:
  // CPU-only mesh data for now
  std::vector<float>         positions;
  std::vector<std::uint32_t> indices;

  void clear();

  void uploadGrid(const geom::GridPlane& grid);
  void uploadMarker(const geom::MarkerCross& marker);
};

} // namespace render
