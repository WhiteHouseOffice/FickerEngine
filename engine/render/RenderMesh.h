#pragma once
#include <vector>
#include <cstdint>

#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"

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
