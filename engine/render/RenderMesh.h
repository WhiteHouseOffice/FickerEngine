#pragma once
#include <vector>
#include <cstdint>

struct GridPlane;
struct MarkerCross;

namespace render {

class RenderMesh {
public:
  // CPU-side data only for now
  std::vector<float>         positions;
  std::vector<std::uint32_t> indices;

  void clear();

  void uploadGrid(const GridPlane& grid);
  void uploadMarker(const MarkerCross& marker);
};

} // namespace render
