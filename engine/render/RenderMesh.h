#pragma once
#include <vector>
#include <cstdint>

// Geometry types are defined in engine/geom/*.h in the global namespace.
struct GridPlane;
struct MarkerCross;

namespace render {

class RenderMesh {
public:
  // CPU-only mesh data for now
  std::vector<float>         positions;
  std::vector<std::uint32_t> indices;

  void clear();

  void uploadGrid(const GridPlane& grid);
  void uploadMarker(const MarkerCross& marker);
};

} // namespace render
