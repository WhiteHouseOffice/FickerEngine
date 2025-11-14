#pragma once

#include <vector>
#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"

namespace render {

// CPU-only mesh container for now: positions as flat float array, plus indices.
class RenderMesh {
public:
  std::vector<float>     positions;
  std::vector<unsigned>  indices;

  void uploadGrid(const geom::GridPlane& grid);
  void uploadMarker(const geom::MarkerCross& marker);

  void clear();
};

} // namespace render
