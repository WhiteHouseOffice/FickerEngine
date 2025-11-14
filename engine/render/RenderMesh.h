#pragma once

#include <vector>
#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"

namespace render {

class RenderMesh {
public:
  // CPU-side mesh data (3 floats per vertex: x,y,z)
  std::vector<float>    positions;
  std::vector<unsigned> indices;

  void uploadGrid(const geom::GridPlane& grid);
  void uploadMarker(const geom::MarkerCross& marker);

  void clear();
};

} // namespace render
