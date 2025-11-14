#pragma once

#include <vector>
#include <cstdint>

// CPU geometry types
#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"

namespace render {

class RenderMesh {
public:
  // CPU-only mesh for now
  std::vector<float>         positions;
  std::vector<std::uint32_t> indices;

  void clear();

  // Upload from CPU geometry
  void uploadGrid(const GridPlane& grid);
  void uploadMarker(const MarkerCross& marker);
};

} // namespace render
